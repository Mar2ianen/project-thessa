//! Backend-neutral terrain presentation and cache retention policy.
use crate::lod::TileKey;
use std::collections::BTreeSet;

fn split_ancestors(leaves: &[TileKey]) -> BTreeSet<TileKey> {
    let mut split = BTreeSet::new();
    for leaf in leaves {
        let mut node = leaf.parent();
        while let Some(key) = node {
            split.insert(key);
            node = key.parent();
        }
    }
    split
}

/// Complete non-overlapping cover of the requested area, using cached parents
/// or the previous finer cover where replacement leaves are still loading.
/// Ready sibling groups can advance independently; one missing distant page
/// does not stall every ready group. None means bootstrap is still required.
pub fn resident_cover(
    wanted: &[TileKey],
    previous: &[TileKey],
    ready: impl Fn(TileKey) -> bool,
) -> Option<Vec<TileKey>> {
    let leaves: BTreeSet<_> = wanted.iter().copied().collect();
    let split = split_ancestors(wanted);
    let old_split = split_ancestors(previous);
    fn visit(
        key: TileKey,
        inherited: bool,
        leaves: &BTreeSet<TileKey>,
        split: &BTreeSet<TileKey>,
        old_split: &BTreeSet<TileKey>,
        ready: &impl Fn(TileKey) -> bool,
    ) -> Option<Vec<TileKey>> {
        let entire = inherited || leaves.contains(&key);
        if !entire && !split.contains(&key) {
            return Some(Vec::new());
        }
        let resident = ready(key);
        if !split.contains(&key) && resident {
            return Some(vec![key]);
        }
        if split.contains(&key) || old_split.contains(&key) {
            let children: Option<Vec<_>> = key
                .children()
                .into_iter()
                .map(|child| visit(child, entire, leaves, split, old_split, ready))
                .collect();
            if let Some(children) = children {
                return Some(children.into_iter().flatten().collect());
            }
        }
        resident.then_some(vec![key])
    }
    let mut result = Vec::new();
    for face in 0..6 {
        result.extend(visit(
            TileKey::root(face),
            false,
            &leaves,
            &split,
            &old_split,
            &ready,
        )?);
    }
    result.sort();
    Some(result)
}

/// Evict only the excess entries, oldest first, with deterministic tile ties.
/// Protected presentation/request dependencies may temporarily exceed budget.
pub fn lru_evictions(
    entries: &[(TileKey, u64)],
    protected: &BTreeSet<TileKey>,
    capacity: usize,
) -> Vec<TileKey> {
    let mut candidates: Vec<_> = entries
        .iter()
        .filter(|(key, _)| !protected.contains(key))
        .copied()
        .collect();
    candidates.sort_by_key(|(key, age)| (*age, *key));
    candidates
        .into_iter()
        .take(entries.len().saturating_sub(capacity))
        .map(|(key, _)| key)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn assert_cover(cover: &[TileKey], area: f64) {
        assert_eq!(
            cover
                .iter()
                .map(|k| 4f64.powi(-(k.level as i32)))
                .sum::<f64>(),
            area
        );
        for a in cover {
            for b in cover {
                if a != b && a.face == b.face && a.level <= b.level {
                    let shift = b.level - a.level;
                    assert!(a.x != b.x >> shift || a.y != b.y >> shift);
                }
            }
        }
    }
    #[test]
    fn ready_family_advances_without_waiting_for_other_face() {
        let a = TileKey::root(0);
        let b = TileKey::root(1);
        let wanted: Vec<_> = a.children().into_iter().chain(b.children()).collect();
        let ready = |k| k == a || k == b || a.children().contains(&k) || k == b.children()[0];
        let cover = resident_cover(&wanted, &[a, b], ready).unwrap();
        assert!(cover.contains(&b));
        assert!(a.children().iter().all(|k| cover.contains(k)));
        assert_cover(&cover, 2.0);
    }
    #[test]
    fn no_partial_family_or_bootstrap_holes() {
        let root = TileKey::root(2);
        let wanted = root.children();
        assert!(resident_cover(&wanted, &[], |k| k == wanted[0]).is_none());
        let cover = resident_cover(&wanted, &[root], |k| k == root || k == wanted[0]).unwrap();
        assert_eq!(cover, vec![root]);
        assert_cover(&cover, 1.0);
    }
    #[test]
    fn cached_children_bridge_unready_coarsening_parent() {
        let root = TileKey::root(3);
        let mut old = root.children();
        old.sort();
        let cover = resident_cover(&[root], &old, |k| old.contains(&k)).unwrap();
        assert_eq!(cover, old);
        assert_cover(&cover, 1.0);
        assert_eq!(resident_cover(&[root], &old, |_| true).unwrap(), vec![root]);
    }
    #[test]
    fn cache_evicts_only_excess_and_keeps_current_cover() {
        let keys = TileKey::root(0).children();
        let entries = [(keys[0], 0), (keys[1], 3), (keys[2], 2), (keys[3], 1)];
        let protected = BTreeSet::from([keys[0]]);
        assert_eq!(lru_evictions(&entries, &protected, 3), vec![keys[3]]);
        assert!(lru_evictions(&entries, &protected, 4).is_empty());
        assert!(!lru_evictions(&entries, &protected, 0).contains(&keys[0]));
    }
}
