use super::*;
use crate::tests::{field, view};
use std::sync::mpsc::sync_channel;

fn policy() -> StreamingPolicy {
    StreamingPolicy {
        selection: SelectionPolicy {
            coarse_level: 0,
            coarse_budget: 6,
            fine_level: 0,
            fine_budget: 6,
        },
        height_cache: 32,
        material_cache: 6,
        draw_capacity: 6,
    }
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
    stream.material_wanted.clear();
    let mut producers = Vec::new();
    for child in children {
        let (tx, rx) = sync_channel(1);
        stream.height_jobs.insert(child, JobHandle::new(rx));
        producers.push(tx);
    }
    let page = || HeightPage::bake(&[100.0; 4], 2, 0.01).unwrap();
    // Deliberately complete out of order; partial siblings cannot replace root.
    producers[2].send(Ok(page())).unwrap();
    assert!(!stream.tick(view(), 0.01, 0.01).unwrap().geometry_changed);
    assert_eq!(stream.visible, roots);
    for i in [3, 0, 1] {
        producers[i].send(Ok(page())).unwrap();
    }
    let update = stream.tick(view(), 0.01, 0.02).unwrap();
    assert_eq!(update.height_ready, 3);
    assert!(update.capacity_rejected);
    assert_eq!(stream.visible, roots);
    stream.policy.draw_capacity = 9;
    // A new completion retries the complete partition without a new selection.
    let (tx, rx) = sync_channel(1);
    tx.send(Ok(page())).unwrap();
    stream.height_jobs.insert(root, JobHandle::new(rx));
    assert!(stream.tick(view(), 0.01, 0.03).unwrap().geometry_changed);
    assert_eq!(stream.visible.len(), 9);
    assert!(!stream.visible.contains(&root));
    assert!(children.iter().all(|k| stream.visible.contains(k)));
    assert!(stream.heights[&root].generation > 1);
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
    assert!(stream.height_jobs.is_empty());
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
    stream.material_jobs.insert(key, JobHandle::new(rx));
    let update = stream.tick(view(), 0.01, 0.01).unwrap();
    assert!(update.materials_changed);
    assert_eq!(update.material_ready, 1);
    let generation = stream.materials[&id].0;
    assert!(generation > 1);
    // Evict/reinsert the same key: its identity cannot reuse a stale generation.
    stream.materials.remove(&id);
    let (tx, rx) = sync_channel(1);
    tx.send(page).unwrap();
    stream.material_jobs.insert(key, JobHandle::new(rx));
    stream.tick(view(), 0.01, 0.02).unwrap();
    assert!(stream.materials[&id].0 > generation);
}
