//! Fixed-capacity material-page slot assignment for the render adapter.
//!
//! A texture-array page layer is a resource with a hard layer limit. The
//! topology stream can reorder every frame, so assigning layers from the
//! current stream would cause avoidable page uploads. [`SlotCache`] keeps a
//! deterministic node-to-layer assignment across updates and reports only
//! pages whose source generation needs to be uploaded.

use std::fmt;

/// A page that should be (re)uploaded to one material-array layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotChange {
    pub slot: u32,
    pub node_id: u64,
    pub generation: u64,
}

/// An occupied layer in the cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotEntry {
    pub slot: u32,
    pub node_id: u64,
    pub generation: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Occupied {
    node_id: u64,
    generation: u64,
}

/// Construction errors for [`SlotCache`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotCacheError {
    InvalidCapacity,
    CapacityTooLarge { capacity: usize },
}

impl fmt::Display for SlotCacheError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCapacity => f.write_str("material slot-cache capacity must be non-zero"),
            Self::CapacityTooLarge { capacity } => {
                write!(
                    f,
                    "material slot-cache capacity {capacity} exceeds u32 slot space"
                )
            }
        }
    }
}

impl std::error::Error for SlotCacheError {}

/// Deterministic, fixed-capacity page-layer assignment.
///
/// `update` accepts `(node_id, source_generation)` pairs in priority order.
/// Only the first `capacity` distinct, non-zero node IDs are selected. An
/// already occupied selected node keeps its layer even when the input order
/// changes; newly selected nodes take the lowest free layer. A changed source
/// generation keeps its layer and appears in the returned upload list.
///
/// The cache itself never grows after construction. The returned change list
/// is a short per-update staging value and is ordered by the selected input,
/// which lets callers upload high-priority pages first. Evicted layers are
/// absent from [`Self::iter`] and from [`Self::slot`]; they do not produce an
/// upload because no new page owns the layer.
#[derive(Debug, Clone)]
pub struct SlotCache {
    slots: Vec<Option<Occupied>>,
}

impl SlotCache {
    pub fn new(capacity: usize) -> Result<Self, SlotCacheError> {
        u32::try_from(capacity).map_err(|_| SlotCacheError::CapacityTooLarge { capacity })?;
        if capacity == 0 {
            return Err(SlotCacheError::InvalidCapacity);
        }
        Ok(Self {
            slots: vec![None; capacity],
        })
    }

    pub const fn capacity(&self) -> usize {
        self.slots.len()
    }

    pub fn len(&self) -> usize {
        self.slots.iter().filter(|slot| slot.is_some()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the layer currently assigned to `node_id`.
    pub fn slot(&self, node_id: u64) -> Option<u32> {
        self.slots
            .iter()
            .position(|entry| entry.is_some_and(|occupied| occupied.node_id == node_id))
            .map(|slot| slot as u32)
    }

    /// Returns the occupied entry at a layer.
    pub fn entry(&self, slot: u32) -> Option<SlotEntry> {
        self.slots.get(slot as usize)?.map(|occupied| SlotEntry {
            slot,
            node_id: occupied.node_id,
            generation: occupied.generation,
        })
    }

    /// Iterates occupied layers in ascending slot order.
    pub fn iter(&self) -> impl Iterator<Item = SlotEntry> + '_ {
        self.slots.iter().enumerate().filter_map(|(slot, entry)| {
            entry.map(|occupied| SlotEntry {
                slot: slot as u32,
                node_id: occupied.node_id,
                generation: occupied.generation,
            })
        })
    }

    /// Retains the highest-priority distinct pages and reports page uploads.
    pub fn update(&mut self, requested: &[(u64, u64)]) -> Vec<SlotChange> {
        // Deduplicate in priority order. Node id 0 is the CBT null node and is
        // ignored so it can never accidentally claim a material layer.
        let mut selected = Vec::with_capacity(self.capacity().min(requested.len()));
        for &(node_id, generation) in requested {
            if node_id == 0 || selected.iter().any(|(id, _)| *id == node_id) {
                continue;
            }
            selected.push((node_id, generation));
            if selected.len() == self.capacity() {
                break;
            }
        }

        // First retain all selected residents. This is the key anti-churn
        // rule: layer identity follows the node, never the input ordinal.
        let mut retained = vec![false; self.capacity()];
        let mut changes = Vec::with_capacity(selected.len());
        for &(node_id, _) in &selected {
            let Some(slot) = self.slot(node_id).map(|slot| slot as usize) else {
                continue;
            };
            retained[slot] = true;
        }

        // Evict pages outside the selected prefix before allocating newcomers.
        // Clearing in slot order makes reuse deterministic and avoids a free
        // list whose state depends on the order of prior frames.
        for (slot, entry) in self.slots.iter_mut().enumerate() {
            if entry.is_some() && !retained[slot] {
                *entry = None;
            }
        }

        // New pages claim the lowest currently free slot, while changes stay
        // in requested priority order.
        for &(node_id, generation) in &selected {
            if let Some(slot) = self.slot(node_id).map(|slot| slot as usize) {
                let occupied = self.slots[slot].as_mut().expect("slot lookup is occupied");
                if occupied.generation != generation {
                    occupied.generation = generation;
                    changes.push(SlotChange {
                        slot: slot as u32,
                        node_id,
                        generation,
                    });
                }
                continue;
            }
            let slot = self
                .slots
                .iter()
                .position(Option::is_none)
                .expect("selected count is bounded by capacity");
            self.slots[slot] = Some(Occupied {
                node_id,
                generation,
            });
            changes.push(SlotChange {
                slot: slot as u32,
                node_id,
                generation,
            });
        }
        changes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_reuse_survives_priority_reordering() {
        let mut cache = SlotCache::new(3).unwrap();
        let first = cache.update(&[(11, 1), (22, 1), (33, 1)]);
        assert_eq!(first.len(), 3);
        let before: Vec<_> = cache.iter().collect();

        assert!(cache.update(&[(33, 1), (11, 1), (22, 1)]).is_empty());
        assert_eq!(cache.iter().collect::<Vec<_>>(), before);
    }

    #[test]
    fn capacity_keeps_only_the_first_distinct_pages() {
        let mut cache = SlotCache::new(2).unwrap();
        let changes = cache.update(&[(11, 1), (22, 1), (33, 1), (22, 9)]);
        assert_eq!(changes.len(), 2);
        assert_eq!(cache.len(), 2);
        assert_eq!(cache.slot(11), Some(0));
        assert_eq!(cache.slot(22), Some(1));
        assert_eq!(cache.slot(33), None);
    }

    #[test]
    fn source_generation_change_reuploads_in_the_same_slot() {
        let mut cache = SlotCache::new(2).unwrap();
        cache.update(&[(11, 4), (22, 8)]);
        let changes = cache.update(&[(22, 8), (11, 5)]);
        assert_eq!(
            changes,
            vec![SlotChange {
                slot: 0,
                node_id: 11,
                generation: 5,
            }]
        );
        assert_eq!(cache.entry(0).unwrap().generation, 5);
    }

    #[test]
    fn changes_follow_selected_priority_for_new_and_updated_pages() {
        let mut cache = SlotCache::new(2).unwrap();
        cache.update(&[(11, 4), (22, 8)]);
        let changes = cache.update(&[(33, 1), (22, 9)]);
        assert_eq!(
            changes,
            vec![
                SlotChange {
                    slot: 0,
                    node_id: 33,
                    generation: 1,
                },
                SlotChange {
                    slot: 1,
                    node_id: 22,
                    generation: 9,
                },
            ]
        );
    }

    #[test]
    fn eviction_releases_layer_for_deterministic_lowest_slot_reuse() {
        let mut cache = SlotCache::new(2).unwrap();
        cache.update(&[(11, 1), (22, 1)]);
        let changes = cache.update(&[(22, 1), (33, 1)]);
        assert_eq!(cache.slot(11), None);
        assert_eq!(cache.slot(22), Some(1));
        assert_eq!(cache.slot(33), Some(0));
        assert_eq!(changes[0].node_id, 33);
    }

    #[test]
    fn empty_update_evicts_all_pages_and_zero_capacity_is_rejected() {
        assert_eq!(
            SlotCache::new(0).unwrap_err(),
            SlotCacheError::InvalidCapacity
        );
        let mut cache = SlotCache::new(2).unwrap();
        cache.update(&[(11, 1), (22, 1)]);
        assert!(cache.update(&[]).is_empty());
        assert!(cache.is_empty());
        assert_eq!(cache.iter().count(), 0);
    }
}

/// Resolve a resident cube-tile ancestor and map child UVs into that page.
/// Two heap bits encode one quadtree level (x then y); depth three is a face.
/// This keeps local materials visible while replacement geometry refines.
pub fn resolve_material_ancestor(
    mut node_id: u64,
    mut resident: impl FnMut(u64) -> bool,
) -> Option<(u64, [f32; 3])> {
    let mut scale = 1.0;
    let mut offset = [0.0, 0.0];
    while node_id >= 8 {
        if resident(node_id) {
            return Some((node_id, [scale, offset[0], offset[1]]));
        }
        offset[0] = (offset[0] + ((node_id >> 1) & 1) as f32) * 0.5;
        offset[1] = (offset[1] + (node_id & 1) as f32) * 0.5;
        scale *= 0.5;
        node_id >>= 2;
    }
    None
}

#[cfg(test)]
mod ancestor_tests {
    use super::*;
    #[test]
    fn child_quadrants_reuse_resident_parent_until_exact_page_arrives() {
        let parent = 12u64;
        for quadrant in 0..4 {
            let child = parent * 4 + quadrant;
            let (_, uv) = resolve_material_ancestor(child, |id| id == parent).unwrap();
            assert_eq!(
                uv,
                [
                    0.5,
                    (quadrant >> 1) as f32 * 0.5,
                    (quadrant & 1) as f32 * 0.5
                ]
            );
            assert_eq!(
                resolve_material_ancestor(child, |id| id == parent || id == child),
                Some((child, [1.0, 0.0, 0.0]))
            );
        }
        // Face 4, child (1,0), grandchild (0,1): x in [.5,.75], y in [.25,.5].
        let grandchild = (parent * 4 + 2) * 4 + 1;
        assert_eq!(
            resolve_material_ancestor(grandchild, |id| id == parent),
            Some((parent, [0.25, 0.5, 0.25]))
        );
        assert!(resolve_material_ancestor(grandchild, |id| id == 11).is_none());
    }
}
