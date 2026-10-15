use super::*;
use crate::tests::{field, view};
use std::sync::mpsc::{SyncSender, sync_channel};

fn build(value: f64, payload: usize) -> HeightBuild<usize> {
    HeightBuild {
        page: HeightPage::bake(&[value; 4], 2, 0.01).unwrap(),
        payload,
    }
}
fn policy(capacity: usize) -> HeightPolicy {
    HeightPolicy {
        capacity,
        max_jobs: 2,
        ancestors: true,
        pin_roots: true,
    }
}
fn pending(
    outputs: &mut BTreeMap<TileKey, SyncSender<Result<HeightBuild<usize>, String>>>,
    key: TileKey,
) -> JobHandle<Result<HeightBuild<usize>, String>> {
    let (tx, rx) = sync_channel(1);
    assert!(outputs.insert(key, tx).is_none());
    JobHandle::new(rx)
}
fn roots() -> HeightStream<usize, usize> {
    let mut stream = HeightStream::default();
    let batch = stream.insert_batch(
        0,
        (0..6).map(|face| (TileKey::root(face), build(1.0, face as usize))),
        |_, p| p,
    );
    assert_eq!(batch.ready, 6);
    stream
}

#[test]
fn admission_accounts_for_inflight_and_protects_drawn_and_bootstrap_pages() {
    let mut stream = roots();
    let wanted = TileKey::root(4).children();
    let mut outputs = BTreeMap::new();
    let admission = stream.reconcile(
        &field(),
        view(),
        &wanted,
        (0..6).map(TileKey::root),
        1,
        policy(8),
        |key| pending(&mut outputs, key),
    );
    assert_eq!(admission.spawned, 2);
    assert!(admission.retired.is_empty());
    assert_eq!(stream.len() + stream.pending_count(), 8);
    let again = stream.reconcile(
        &field(),
        view(),
        &wanted,
        (0..6).map(TileKey::root),
        1,
        policy(8),
        |_| panic!("duplicate/excess job"),
    );
    assert!(again.capacity_rejected);
    assert_eq!(again.spawned, 0);
    for (_, tx) in std::mem::take(&mut outputs) {
        tx.send(Ok(build(2.0, 7))).unwrap();
    }
    assert_eq!(stream.poll_with(1, |_, p| p).ready, 2);
    assert_eq!(stream.len(), 8);
    assert!((0..6).all(|face| stream.contains_key(&TileKey::root(face))));
}

#[test]
fn mixed_failures_release_slots_without_losing_other_completed_outputs() {
    let mut stream = HeightStream::<usize, usize>::default();
    let mut outputs = BTreeMap::new();
    for face in 0..4 {
        let key = TileKey::root(face);
        stream.jobs.insert(key, pending(&mut outputs, key));
    }
    outputs
        .remove(&TileKey::root(0))
        .unwrap()
        .send(Err("bad field output".into()))
        .unwrap();
    drop(outputs.remove(&TileKey::root(1)));
    outputs
        .remove(&TileKey::root(2))
        .unwrap()
        .send(Ok(build(3.0, 23)))
        .unwrap();
    let result = stream.poll_with(1, |_, p| p);
    assert_eq!(
        (result.ready, result.failed.len(), stream.pending_count()),
        (1, 2, 1)
    );
    assert_eq!(stream.get(&TileKey::root(2)), Some(&23));
    assert_eq!(stream.source().len(), 1);
    let wanted = [TileKey::root(0)];
    let admission = stream.reconcile(&field(), view(), &wanted, [], 2, policy(8), |key| {
        pending(&mut outputs, key)
    });
    assert_eq!(admission.spawned, 1, "failed request is retryable");
    drop(stream);
    for (_, tx) in outputs {
        assert!(tx.send(Ok(build(4.0, 1))).is_err());
    }
}

#[test]
fn replacement_and_eviction_retire_payloads_once_and_preserve_old_snapshots() {
    let mut stream = roots();
    let key = TileKey::root(0);
    let id = lod::cbt_node_for_tile(key).unwrap().id();
    let old = stream.source().clone();
    let replacement = stream.insert_batch(1, [(key, build(10.0, 99))], |_, p| p);
    assert_eq!(replacement.retired, [(key, 0)]);
    assert_ne!(stream.source().page_version(id), old.page_version(id));
    assert_ne!(stream.source().get(id), old.get(id));
    let admission = stream.reconcile(
        &field(),
        view(),
        &[],
        [key],
        2,
        HeightPolicy {
            capacity: 1,
            max_jobs: 0,
            ancestors: false,
            pin_roots: false,
        },
        |_| panic!(),
    );
    assert_eq!(admission.retired.len(), 5);
    assert_eq!(stream.len(), 1);
    assert_eq!(stream.get(&key), Some(&99));
    assert_eq!(stream.source().len(), stream.len());
    let again = stream.reconcile(
        &field(),
        view(),
        &[],
        [key],
        3,
        HeightPolicy {
            capacity: 1,
            max_jobs: 0,
            ancestors: false,
            pin_roots: false,
        },
        |_| panic!(),
    );
    assert!(again.retired.is_empty());
    assert_eq!(old.len(), 6);
}

#[test]
fn pressure_and_zero_capacity_never_evict_protected_cover_or_admit_new_work() {
    let mut stream = roots();
    let admission = stream.reconcile(
        &field(),
        view(),
        &TileKey::root(4).children(),
        (0..6).map(TileKey::root),
        1,
        policy(0),
        |_| panic!("no capacity"),
    );
    assert!(admission.capacity_rejected);
    assert!(admission.retired.is_empty());
    assert_eq!(
        stream.len(),
        6,
        "external budget shrink must not punch holes"
    );
    assert_eq!(stream.pending_count(), 0);
}

#[test]
fn deterministic_lru_ties_and_identical_republish_keep_page_versions() {
    let mut stream = roots();
    let key = TileKey::root(5);
    let version = stream.height(key).unwrap().generation;
    let result = stream.insert_batch(9, [(key, build(1.0, 42))], |_, p| p);
    assert_eq!(result.retired, [(key, 5)]);
    assert_eq!(stream.height(key).unwrap().generation, version);
    let admission = stream.reconcile(
        &field(),
        view(),
        &[],
        [],
        10,
        HeightPolicy {
            capacity: 3,
            max_jobs: 0,
            ancestors: false,
            pin_roots: false,
        },
        |_| panic!(),
    );
    assert_eq!(
        admission
            .retired
            .iter()
            .map(|(k, _)| k.face)
            .collect::<Vec<_>>(),
        [0, 1, 2]
    );
    assert_eq!(stream.len(), 3);
}

#[test]
fn invalid_address_is_rejected_before_host_asset_conversion() {
    let mut stream = HeightStream::<usize, usize>::default();
    let result = stream.insert_batch(0, [(TileKey::root(99), build(0.0, 9))], |_, _| {
        panic!("invalid output cannot create assets")
    });
    assert_eq!(result.failed.len(), 1);
    assert_eq!(result.ready, 0);
    assert!(stream.is_empty());
    assert!(stream.source().is_empty());
}
