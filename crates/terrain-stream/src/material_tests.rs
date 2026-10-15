use super::*;
use crate::tests::view;
use std::sync::mpsc::{SyncSender, sync_channel};

fn page(value: u8) -> CbtMaterialPage {
    CbtMaterialPage::from_rgba8(vec![value; 128 * 128 * 4]).unwrap()
}

fn demand(fine: &[TileKey], height_jobs: usize) -> MaterialDemand<'_> {
    MaterialDemand {
        selection_epoch: 1,
        fine,
        view: view(),
        pixel_angle: 1.0,
        height_jobs,
    }
}

fn pending(
    outputs: &mut BTreeMap<TileKey, SyncSender<CbtMaterialPage>>,
    key: TileKey,
) -> JobHandle<CbtMaterialPage> {
    let (tx, rx) = std::sync::mpsc::sync_channel(1);
    assert!(outputs.insert(key, tx).is_none(), "duplicate build");
    JobHandle::new(rx)
}

#[test]
fn job_budget_defers_to_heights_and_deduplicates_shared_ancestors() {
    let mut stream = MaterialStream::new(32);
    let roots: Vec<_> = (0..6).map(TileKey::root).collect();
    let mut outputs = BTreeMap::new();
    stream.reconcile_with(demand(&[], 4), roots.clone(), |key| {
        pending(&mut outputs, key)
    });
    assert_eq!(stream.pending_count(), 2);
    stream.reconcile_with(demand(&[], 0), roots.clone(), |key| {
        pending(&mut outputs, key)
    });
    assert_eq!(stream.pending_count(), 4);
    stream.reconcile_with(demand(&[], 4), roots, |_| {
        panic!("do not spawn above pressure budget")
    });
    assert_eq!(
        stream.pending_count(),
        4,
        "do not cancel already running jobs"
    );

    let root = TileKey::root(4);
    let fine = TileKey {
        level: 4,
        x: 8,
        y: 8,
        ..root
    };
    let mut stream = MaterialStream::new(32);
    let mut spawned = Vec::new();
    stream.reconcile_with(demand(&[], 0), fine.children(), |key| {
        spawned.push(key);
        pending(&mut outputs, key)
    });
    assert_eq!(
        spawned,
        [fine.parent().unwrap().parent().unwrap().parent().unwrap()]
    );
}

#[test]
fn bootstrap_advances_regional_ancestors_to_exact_demand_without_height_pages() {
    let root = TileKey::root(4);
    let fine = TileKey {
        level: 6,
        x: 32,
        y: 32,
        ..root
    };
    let mut stream = MaterialStream::new(32);
    let mut outputs = BTreeMap::new();
    for level in [2, 4, 6] {
        stream.reconcile_with(demand(&[], 0), [fine], |key| pending(&mut outputs, key));
        assert_eq!(outputs.len(), 1);
        let (key, tx) = outputs.pop_first().unwrap();
        assert_eq!(key.level, level);
        assert_eq!(stream.poll().ready, 0, "empty mailbox is not a failure");
        tx.send(page(level)).unwrap();
        assert_eq!(stream.poll().ready, 1);
        assert_eq!(stream.pending_count(), 0);
    }
    let revision = stream.source().generation();
    let snapshot = stream.source().clone();
    stream.reconcile_with(demand(&[], 0), [fine], |_| {
        panic!("exact page is already resident")
    });
    assert_eq!(stream.source().generation(), revision);
    assert!(std::ptr::eq(
        stream.source().source_pages(),
        snapshot.source_pages()
    ));
}

#[test]
fn disconnected_worker_releases_slot_and_retries_current_demand() {
    let root = TileKey::root(4);
    let mut stream = MaterialStream::new(8);
    stream.reconcile_with(demand(&[], 0), [root], |_| {
        let (tx, rx) = sync_channel(1);
        drop(tx);
        JobHandle::new(rx)
    });
    let polled = stream.poll();
    assert_eq!(
        (polled.ready, polled.failed, stream.pending_count()),
        (0, 1, 0)
    );
    let mut outputs = BTreeMap::new();
    stream.reconcile_with(demand(&[], 0), [root], |key| pending(&mut outputs, key));
    outputs.remove(&root).unwrap().send(page(9)).unwrap();
    assert_eq!(stream.poll().ready, 1);
    assert!(
        stream
            .source()
            .contains_page(lod::cbt_node_for_tile(root).unwrap().id())
    );
}

#[test]
fn completion_batch_and_cache_pressure_preserve_snapshot_and_bound_residency() {
    let mut stream = MaterialStream::new(2);
    let roots: Vec<_> = (0..6).map(TileKey::root).collect();
    let mut outputs = BTreeMap::new();
    stream.reconcile_with(demand(&[], 0), roots.clone(), |key| {
        pending(&mut outputs, key)
    });
    for (_, tx) in std::mem::take(&mut outputs) {
        tx.send(page(4)).unwrap();
    }
    assert_eq!(stream.poll().ready, 4);
    let snapshot = stream.source().clone();
    stream.reconcile_with(demand(&[], 0), roots, |key| pending(&mut outputs, key));
    assert_eq!(stream.source().source_pages().len(), 2);
    assert_eq!(snapshot.source_pages().len(), 4);
    assert!(stream.pending_count() <= 4);
    // Dropping a host closes its bounded receiver; no worker needs the host to drain it.
    drop(stream);
    for (_, tx) in outputs {
        assert!(tx.send(page(5)).is_err());
    }
}

#[test]
fn pixel_scale_invalidates_refinement_without_geometry_reselection() {
    let fine = [TileKey::root(4)];
    let mut stream = MaterialStream::new(0);
    let mut input = demand(&fine, 0);
    stream.reconcile_with(input, [], |_| panic!("zero budget cannot spawn"));
    let coarse = stream.refined.clone();
    assert!(!stream.needs_refinement(1, 1.0));
    input.pixel_angle = 1e-6;
    assert!(stream.needs_refinement(1, input.pixel_angle));
    stream.reconcile_with(input, [], |_| panic!("zero budget cannot spawn"));
    assert!(stream.refined.len() > coarse.len());
    assert!(stream.refined.len() <= 512);
    assert!(stream.source().source_pages().is_empty());
    assert_eq!(stream.pending_count(), 0);
    let direct = selection::material_view_selection(&fine, input.view, input.pixel_angle);
    assert_eq!(stream.refined, direct, "same policy as pre-extraction host");
    input.selection_epoch += 1;
    input.view.eye *= 2.0;
    stream.reconcile_with(input, [], |_| panic!("zero budget cannot spawn"));
    assert_eq!(
        stream.refined,
        selection::material_view_selection(&fine, input.view, input.pixel_angle)
    );
}

#[test]
fn source_trim_is_independent_of_geometry_lifetime_and_demand_order_is_stable() {
    let root = TileKey::root(4);
    let id = lod::cbt_node_for_tile(root).unwrap().id();
    let mut source = MaterialPageStore::default();
    source.set_page(id, page(12));
    let mut stream = MaterialStream::with_source(8, source);
    stream.reconcile_with(demand(&[], 0), [root, root], |_| panic!("resident"));
    let revision = stream.source().generation();
    stream.reconcile_with(demand(&[], 0), [], |_| panic!("no demand"));
    assert!(
        stream.source().contains_page(id),
        "height eviction is not material eviction"
    );
    assert!(
        stream.source().generation() > revision,
        "priority change is published too"
    );
    let revision = stream.source().generation();
    stream.reconcile_with(demand(&[], 0), [], |_| panic!("no demand"));
    assert_eq!(stream.source().generation(), revision);
    let mut a = MaterialStream::new(0);
    let mut b = MaterialStream::new(0);
    a.reconcile_with(demand(&[], 0), root.children(), |_| panic!());
    b.reconcile_with(
        demand(&[], 0),
        root.children().into_iter().rev(),
        |_| panic!(),
    );
    assert_eq!(a.source().priority(), b.source().priority());
}
