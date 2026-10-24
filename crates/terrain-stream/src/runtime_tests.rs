use super::*;
use crate::JobHandle;
use crate::tests::{field, view};
use std::sync::mpsc::sync_channel;
use thessa_rcbt_core::HeightPage;

fn policy() -> StreamingPolicy {
    StreamingPolicy {
        persistent_topology: false,
        selection: SelectionPolicy {
            coarse_level: 0,
            coarse_budget: 6,
            fine_level: 0,
            fine_budget: 6,
        },
        height_cache: 32,
        material_cache: 6,
        draw_capacity: 6,
        normal_maps: true,
        ground_detail: false,
        ground_relief_max_level: 17,
        material_max_level: 17,
    }
}

#[test]
fn disabled_normal_policy_omits_bootstrap_planes() {
    let stream = TerrainStream::new(
        field(),
        StreamingPolicy {
            normal_maps: false,
            ..policy()
        },
    )
    .unwrap();
    assert!(
        stream
            .materials()
            .values()
            .all(|(_, page)| page.normal_mips().is_none())
    );
}

#[test]
fn asynchronous_publication_keeps_cover_and_reports_exhaustion() {
    let mut stream = TerrainStream::new(field(), policy()).unwrap();
    let roots = stream.visible.clone();
    stream.tick(view(), 0.01, 0.0).unwrap();
    stream.visible = roots.clone();
    let root = TileKey::root(4);
    let children = root.children();
    stream.wanted = roots
        .iter()
        .copied()
        .filter(|k| *k != root)
        .chain(children)
        .collect();
    let mut producers = Vec::new();
    for child in children {
        let (tx, rx) = sync_channel(1);
        stream.heights.jobs.insert(child, JobHandle::new(rx));
        producers.push(tx);
    }
    let page = || HeightPage::bake(&[100.0; 4], 2, 0.01).unwrap();
    // Deliberately complete out of order; partial siblings cannot replace root.
    producers[2]
        .send(Ok(HeightBuild {
            page: page(),
            payload: (),
        }))
        .unwrap();
    assert!(!stream.tick(view(), 0.01, 0.01).unwrap().geometry_changed);
    assert_eq!(stream.visible, roots);
    for i in [3, 0, 1] {
        producers[i]
            .send(Ok(HeightBuild {
                page: page(),
                payload: (),
            }))
            .unwrap();
    }
    let update = stream.tick(view(), 0.01, 0.02).unwrap();
    assert_eq!(update.height_ready, 3);
    assert!(update.capacity_rejected);
    assert_eq!(stream.visible, roots);
    stream.policy.draw_capacity = 9;
    // A new completion retries the complete partition without a new selection.
    let (tx, rx) = sync_channel(1);
    tx.send(Ok(HeightBuild {
        page: page(),
        payload: (),
    }))
    .unwrap();
    stream.heights.jobs.insert(root, JobHandle::new(rx));
    assert!(stream.tick(view(), 0.01, 0.03).unwrap().geometry_changed);
    assert_eq!(stream.visible.len(), 9);
    assert!(!stream.visible.contains(&root));
    assert!(children.iter().all(|k| stream.visible.contains(k)));
    assert!(stream.height(root).unwrap().generation > 1);
    assert!(
        stream
            .visible
            .iter()
            .all(|k| stream.heights.contains_key(k))
    );
    assert!(stream.cache_entries().1 <= 6);
    assert!(stream.pending_jobs().0 <= 8 && stream.pending_jobs().1 <= 4);
}

#[test]
fn hard_height_budget_includes_inflight_results_and_invalid_views_are_rejected() {
    let mut policy = policy();
    policy.height_cache = 6;
    policy.selection.coarse_level = 2;
    policy.selection.coarse_budget = 24;
    let mut stream = TerrainStream::new(field(), policy).unwrap();
    let mut invalid = view();
    invalid.eye.x = f64::NAN;
    assert!(stream.tick(invalid, 0.01, 0.0).is_err());
    assert_eq!(stream.wanted_count(), 0);
    let update = stream.tick(view(), 0.01, 0.0).unwrap();
    assert!(update.capacity_rejected);
    assert_eq!(stream.heights.len(), 6);
    assert_eq!(stream.heights.pending_count(), 0);
    assert!(
        stream
            .visible
            .iter()
            .all(|key| stream.heights.contains_key(key))
    );
}

#[test]
fn material_completion_is_not_discarded_for_missing_height_key() {
    let mut policy = policy();
    policy.material_cache = 32;
    let mut stream = TerrainStream::new(field(), policy).unwrap();
    stream.tick(view(), 0.01, 0.0).unwrap();
    let key = TileKey {
        face: 4,
        level: 4,
        x: 8,
        y: 8,
    };
    assert!(!stream.heights.contains_key(&key));
    let id = lod::cbt_node_for_tile(key).unwrap().id();
    let page = CbtMaterialPage::from_rgba8(vec![100; 128 * 128 * 4]).unwrap();
    let (tx, rx) = sync_channel(1);
    tx.send(page.clone()).unwrap();
    stream.materials.jobs.insert(key, JobHandle::new(rx));
    let update = stream.tick(view(), 0.01, 0.01).unwrap();
    assert!(update.materials_changed);
    assert_eq!(update.material_ready, 1);
    let generation = stream.materials().get(&id).unwrap().0;
    assert!(generation > 1);
    // Evict/reinsert the same key: its identity cannot reuse a stale generation.
    stream.materials.source.remove_page(id);
    let (tx, rx) = sync_channel(1);
    tx.send(page).unwrap();
    stream.materials.jobs.insert(key, JobHandle::new(rx));
    stream.tick(view(), 0.01, 0.02).unwrap();
    assert!(stream.materials()[&id].0 > generation);
}

#[test]
fn disconnected_material_worker_is_reported_without_failing_native_frame() {
    let mut stream = TerrainStream::new(field(), policy()).unwrap();
    stream.tick(view(), 1.0, 0.0).unwrap();
    stream.fine.clear();
    stream.materials = MaterialStream::with_source(6, stream.materials.source().clone());
    let (tx, rx) = sync_channel(1);
    drop(tx);
    stream
        .materials
        .jobs
        .insert(TileKey::root(4), JobHandle::new(rx));
    let update = stream.tick(view(), 1.0, 0.01).unwrap();
    assert_eq!(update.material_failed, 1);
    assert_eq!(update.material_ready, 0);
    assert!(!update.selected);
    assert_eq!(stream.materials.pending_count(), 0);
    assert_eq!(stream.tick(view(), 1.0, 0.02).unwrap().material_failed, 0);
}

#[test]
fn stationary_pixel_scale_change_reaches_material_reconcile_without_geometry_change() {
    let mut stream = TerrainStream::new(field(), policy()).unwrap();
    stream.tick(view(), 1.0, 0.0).unwrap();
    let before = stream.material_demand().to_vec();
    let update = stream.tick(view(), 1e-6, 0.01).unwrap();
    assert!(!update.selected, "resize is not a new geometry selection");
    assert!(!update.geometry_changed);
    assert!(update.materials_changed);
    assert_ne!(stream.material_demand(), before);
}

#[test]
fn visible_height_edit_republishes_unchanged_cover_and_failed_workers_are_retryable() {
    let mut stream = TerrainStream::new(field(), policy()).unwrap();
    stream.tick(view(), 1.0, 0.0).unwrap();
    let visible = stream.visible.clone();
    let key = TileKey::root(4);
    let generation = stream.height(key).unwrap().generation;
    let (tx, rx) = sync_channel(1);
    tx.send(Ok(HeightBuild {
        page: HeightPage::bake(&[700.0; 4], 2, 0.01).unwrap(),
        payload: (),
    }))
    .unwrap();
    stream.heights.jobs.insert(key, JobHandle::new(rx));
    let update = stream.tick(view(), 1.0, 0.01).unwrap();
    assert!(update.geometry_changed);
    assert!(!update.selected);
    assert_eq!(stream.visible, visible);
    assert!(stream.height(key).unwrap().generation > generation);
    let (tx, rx) = sync_channel(1);
    drop(tx);
    stream.heights.jobs.insert(key, JobHandle::new(rx));
    let update = stream.tick(view(), 1.0, 0.02).unwrap();
    assert_eq!(update.height_failed, 1);
    assert!(!update.geometry_changed);
    assert_eq!(
        stream.height(key).unwrap().generation,
        stream
            .heights
            .source()
            .page_version(lod::cbt_node_for_tile(key).unwrap().id())
            .unwrap()
    );
}

#[test]
fn fine_geometry_above_l17_requires_explicit_ground_detail_and_stays_bounded() {
    let mut policy = policy();
    policy.selection.fine_level = 22;
    assert!(TerrainStream::new(field(), policy).is_err());
    policy.ground_detail = true;
    assert!(TerrainStream::new(field(), policy).is_ok());
    policy.selection.fine_level = 23;
    assert!(TerrainStream::new(field(), policy).is_err());
}

#[test]
fn persistent_residency_admits_one_child_without_siblings_and_releases_cold_pages() {
    let mut policy = policy();
    policy.persistent_topology = true;
    policy.height_cache = 14;
    policy.draw_capacity = 14;
    let mut stream = TerrainStream::new(field(), policy).unwrap();
    stream.tick(view(), 1.0, 0.0).unwrap();
    let roots = stream.visible.clone();
    let child = TileKey::root(4).children()[2];
    stream.wanted.push(child);
    let (tx, rx) = sync_channel(1);
    tx.send(Ok(HeightBuild {
        page: HeightPage::bake(&[100.0; 4], 2, 0.01).unwrap(),
        payload: (),
    }))
    .unwrap();
    stream.heights.jobs.insert(child, JobHandle::new(rx));
    let update = stream.tick(view(), 1.0, 0.01).unwrap();
    assert!(update.geometry_changed);
    assert_eq!(stream.visible.len(), 7);
    assert!(roots.iter().all(|key| stream.visible.contains(key)));
    assert!(stream.visible.contains(&child));
    assert!(
        stream.visible.contains(&TileKey::root(4)),
        "overlapping data, not a draw partition"
    );
    stream.wanted = roots.clone();
    let (tx, rx) = sync_channel(1);
    tx.send(Ok(HeightBuild {
        page: HeightPage::bake(&[1.0; 4], 2, 0.01).unwrap(),
        payload: (),
    }))
    .unwrap();
    stream
        .heights
        .jobs
        .insert(TileKey::root(4), JobHandle::new(rx));
    assert!(stream.tick(view(), 1.0, 0.02).unwrap().geometry_changed);
    assert_eq!(stream.visible, roots);
    assert!(
        !stream.heights.contains_key(&child),
        "GPU publication must not pin cold data forever"
    );
    stream.configure_topology(false).unwrap();
    assert_eq!(stream.visible.len(), 6);
    assert!(!stream.policy.persistent_topology);
    assert_eq!(stream.pending_jobs(), (0, 0));
}

#[test]
fn persistent_page_publication_budget_and_relief_tiers_are_validated() {
    let mut p = policy();
    p.persistent_topology = true;
    assert!(TerrainStream::new(field(), p).is_err());
    p.draw_capacity = p.height_cache;
    assert!(TerrainStream::new(field(), p).is_ok());
    p.ground_relief_max_level = 22;
    assert!(TerrainStream::new(field(), p).is_err());
    p.ground_detail = true;
    assert!(TerrainStream::new(field(), p).is_ok());
    p.ground_relief_max_level = 23;
    assert!(TerrainStream::new(field(), p).is_err());
}
