//! Slab allocator for variable-size block uploads (follow-up to the
//! `Relocated` flush: instead of re-sending whole pages, blocks live at
//! allocator-assigned offsets in one compact buffer and only move when
//! they grow).
//!
//! Backend-neutral policy: free ranges and placements are `BTreeMap`s, so
//! a given operation sequence allocates identically on every run. The
//! backend owns the actual bytes; this module owns the accounting and the
//! move plans.
//!
//! Layout model: the compact buffer holds whole encoded pages back to
//! back (page images, not individual blocks). A page whose blocks change
//! size is re-placed as one image; the offset table travels with it. This
//! keeps one table per page valid and avoids per-block free-list
//! churn — the allocator problem stays one-dimensional (pages in bytes).

use std::collections::BTreeMap;

/// Allocation failure modes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AllocError {
    /// No run of free bytes fits `size` (see [`SlabAllocator::largest_free`]).
    OutOfMemory {
        /// Requested bytes.
        want: u64,
        /// Largest free run.
        largest_free: u64,
    },
    /// Unknown placement id.
    UnknownId(u64),
}

impl std::fmt::Display for AllocError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AllocError::OutOfMemory { want, largest_free } => write!(
                f,
                "slab out of memory: want {want} B, largest free run {largest_free} B"
            ),
            AllocError::UnknownId(id) => write!(f, "unknown slab placement {id}"),
        }
    }
}

impl std::error::Error for AllocError {}

/// One planned byte move for [`SlabAllocator::plan_compact`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlabMove {
    /// Placement id.
    pub id: u64,
    /// Current offset.
    pub from: u64,
    /// Compacted offset (always `<= from`).
    pub to: u64,
    /// Bytes to copy.
    pub bytes: u64,
}

/// Fixed-capacity first-fit allocator over byte ranges.
pub struct SlabAllocator {
    capacity: u64,
    /// Free runs by offset (coalesced on free).
    free: BTreeMap<u64, u64>,
    /// Placements by id: `(offset, size)`.
    placed: BTreeMap<u64, (u64, u64)>,
    /// Lifetime allocated bytes (counts reallocs separately).
    allocated_total: u64,
    /// Lifetime freed bytes.
    freed_total: u64,
}

/// Fragmentation and usage snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlabStats {
    /// Pool capacity in bytes.
    pub capacity: u64,
    /// Sum of placed sizes.
    pub used: u64,
    /// Free bytes (`capacity - used`).
    pub free: u64,
    /// Largest single free run (the biggest placeable image).
    pub largest_free: u64,
    /// Live placements.
    pub placements: usize,
    /// Lifetime allocated bytes.
    pub allocated_total: u64,
    /// Lifetime freed bytes.
    pub freed_total: u64,
}

impl SlabAllocator {
    /// Empty pool of `capacity` bytes.
    pub fn new(capacity: u64) -> Self {
        let mut free = BTreeMap::new();
        if capacity > 0 {
            free.insert(0, capacity);
        }
        Self {
            capacity,
            free,
            placed: BTreeMap::new(),
            allocated_total: 0,
            freed_total: 0,
        }
    }

    /// Current snapshot.
    pub fn stats(&self) -> SlabStats {
        let used: u64 = self.placed.values().map(|(_, size)| *size).sum();
        SlabStats {
            capacity: self.capacity,
            used,
            free: self.capacity - used,
            largest_free: self.free.values().copied().max().unwrap_or(0),
            placements: self.placed.len(),
            allocated_total: self.allocated_total,
            freed_total: self.freed_total,
        }
    }

    /// Placement of one id, if resident.
    pub fn placement(&self, id: u64) -> Option<(u64, u64)> {
        self.placed.get(&id).copied()
    }

    /// First-fit allocate `size` bytes for `id`. Zero-size placements are
    /// rejected (they make free accounting ambiguous); re-allocating a
    /// live id frees it first, which keeps ids stable across re-uploads.
    pub fn alloc(&mut self, id: u64, size: u64) -> Result<u64, AllocError> {
        if size == 0 {
            return Err(AllocError::OutOfMemory {
                want: 0,
                largest_free: self.stats().largest_free,
            });
        }
        if self.placed.contains_key(&id) {
            self.free_id(id).expect("checked id is resident");
        }
        let run = self
            .free
            .iter()
            .find(|(_, run)| **run >= size)
            .map(|(offset, _)| *offset)
            .ok_or(AllocError::OutOfMemory {
                want: size,
                largest_free: self.stats().largest_free,
            })?;
        let len = self.free.remove(&run).expect("free run exists");
        if len > size {
            self.free.insert(run + size, len - size);
        }
        self.placed.insert(id, (run, size));
        self.allocated_total += size;
        Ok(run)
    }

    /// Free one placement, coalescing with neighbours.
    pub fn free_id(&mut self, id: u64) -> Result<(u64, u64), AllocError> {
        let (offset, size) = self.placed.remove(&id).ok_or(AllocError::UnknownId(id))?;
        self.freed_total += size;
        self.release(offset, size);
        Ok((offset, size))
    }

    /// Return a raw range to the free list with neighbour coalescing.
    fn release(&mut self, offset: u64, size: u64) {
        let mut base = offset;
        let mut len = size;
        // Merge with the preceding run if adjacent.
        if let Some((prev, prev_len)) = self
            .free
            .range(..base)
            .next_back()
            .map(|(offset, len)| (*offset, *len))
            .filter(|(prev, prev_len)| prev + prev_len == base)
        {
            self.free.remove(&prev);
            base = prev;
            len += prev_len;
        }
        // Merge with the following run if adjacent.
        let end = base + len;
        if let Some(following) = self.free.remove(&end) {
            len += following;
        }
        self.free.insert(base, len);
    }

    /// Resize one placement. Grows in place when the following bytes are
    /// free, else moves to the first fit (the caller copies bytes from
    /// the returned old offset to the new one). Shrinking splits the tail
    /// back into the free list. Returns `(offset, moved)`.
    pub fn realloc(&mut self, id: u64, size: u64) -> Result<(u64, bool), AllocError> {
        if size == 0 {
            return Err(AllocError::OutOfMemory {
                want: 0,
                largest_free: self.stats().largest_free,
            });
        }
        let (offset, old) = self
            .placed
            .get(&id)
            .copied()
            .ok_or(AllocError::UnknownId(id))?;
        if size == old {
            return Ok((offset, false));
        }
        if size < old {
            // Shrink in place: the tail goes back through the merging
            // release path (it may touch runs on either side).
            self.placed.insert(id, (offset, size));
            self.freed_total += old - size;
            self.release(offset + size, old - size);
            return Ok((offset, false));
        }
        // Grow: absorb the following run when it fits exactly the growth.
        let end = offset + old;
        if let Some(following) = self
            .free
            .get(&end)
            .copied()
            .filter(|following| old + following >= size)
        {
            self.free.remove(&end);
            let spill = old + following - size;
            if spill > 0 {
                self.free.insert(offset + size, spill);
            }
            self.placed.insert(id, (offset, size));
            self.allocated_total += size - old;
            return Ok((offset, false));
        }
        // Move: allocate first (may fail), then release the old range. The
        // old range frees into the pool the new placement just left, so a
        // swapped layout cannot strand itself.
        let saved = self.placed.remove(&id).expect("placement exists");
        match self.alloc(id, size) {
            Ok(next) => {
                self.release(saved.0, saved.1);
                self.freed_total += saved.1;
                Ok((next, true))
            }
            Err(e) => {
                self.placed.insert(id, saved);
                Err(e)
            }
        }
    }

    /// Deterministic compaction plan: placements slide down in offset
    /// order, preserving relative order, so every move goes down
    /// (`to <= from`) and the caller can copy ascending without scratch
    /// space. Id ties are impossible (offsets are disjoint); equal
    /// offsets cannot happen.
    pub fn plan_compact(&self) -> Vec<SlabMove> {
        let mut by_offset: Vec<(u64, u64, u64)> = self
            .placed
            .iter()
            .map(|(id, (offset, size))| (*offset, *id, *size))
            .collect();
        by_offset.sort();
        let mut moves = Vec::new();
        let mut cursor = 0u64;
        for (offset, id, size) in by_offset {
            if offset != cursor {
                moves.push(SlabMove {
                    id,
                    from: offset,
                    to: cursor,
                    bytes: size,
                });
            }
            cursor += size;
        }
        moves
    }

    /// Apply a [`SlabAllocator::plan_compact`] result after the caller
    /// copied the bytes. Rebuilds placement and free state from scratch,
    /// so partially applied plans cannot corrupt accounting: either call
    /// this once with the full move list or not at all.
    pub fn apply_compact(&mut self, moves: &[SlabMove]) {
        for mv in moves {
            let entry = self.placed.get_mut(&mv.id).expect("compact id resident");
            assert_eq!(entry.0, mv.from, "compact moves apply in plan order");
            entry.0 = mv.to;
        }
        self.free.clear();
        let used: u64 = self.placed.values().map(|(_, size)| *size).sum();
        if used < self.capacity {
            self.free.insert(used, self.capacity - used);
        }
    }

    /// Debug invariant check for integration workloads: placements are
    /// pairwise disjoint, inside the pool, free runs are disjoint and
    /// cover exactly the complement, and stats agree. Panics with a
    /// message naming the violated invariant.
    pub fn check_invariants(&self) {
        let mut cursor = 0u64;
        let mut placed: Vec<(u64, u64)> = self.placed.values().copied().collect();
        placed.sort();
        for (offset, size) in &placed {
            assert!(*offset >= cursor, "slab placements overlap at {offset}");
            assert!(
                offset
                    .checked_add(*size)
                    .is_some_and(|end| end <= self.capacity),
                "slab placement {offset}+{size} escapes the pool"
            );
            cursor = *offset + *size;
        }
        let placed_bytes: u64 = placed.iter().map(|(_, size)| *size).sum();
        let free_bytes: u64 = self.free.values().sum();
        assert_eq!(
            placed_bytes + free_bytes,
            self.capacity,
            "slab placed + free must equal capacity"
        );
        let mut prev_end = None;
        for (offset, len) in &self.free {
            if let Some(end) = prev_end {
                assert!(*offset > end, "slab free runs overlap or touch");
            }
            prev_end = Some(*offset + *len);
        }
        assert_eq!(self.stats().used, placed_bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_fit_packs_in_order() {
        let mut slab = SlabAllocator::new(100);
        assert_eq!(slab.alloc(1, 30), Ok(0));
        assert_eq!(slab.alloc(2, 20), Ok(30));
        assert_eq!(slab.alloc(3, 50), Ok(50));
        assert_eq!(
            slab.alloc(4, 1),
            Err(AllocError::OutOfMemory {
                want: 1,
                largest_free: 0
            })
        );
        slab.check_invariants();
    }

    #[test]
    fn free_coalesces_both_sides() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 25).unwrap();
        slab.alloc(2, 25).unwrap();
        slab.alloc(3, 25).unwrap();
        slab.alloc(4, 25).unwrap();
        slab.free_id(2).unwrap();
        slab.free_id(4).unwrap();
        // Freeing 3 merges [25..50) with [50..75) and [75..100).
        slab.free_id(3).unwrap();
        assert_eq!(slab.stats().largest_free, 75);
        assert_eq!(slab.alloc(5, 75), Ok(25));
        slab.check_invariants();
    }

    #[test]
    fn realloc_grows_in_place_when_possible() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 20).unwrap();
        slab.alloc(2, 20).unwrap();
        slab.free_id(2).unwrap();
        assert_eq!(slab.realloc(1, 35), Ok((0, false)));
        assert_eq!(slab.placement(1), Some((0, 35)));
        slab.check_invariants();
    }

    #[test]
    fn realloc_moves_when_blocked() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 20).unwrap();
        slab.alloc(2, 60).unwrap();
        // 1 cannot grow past live 2, and the 20-byte hole at 80 is too
        // small for 21: clean failure with the true largest run.
        assert_eq!(
            slab.realloc(1, 20 + 1),
            Err(AllocError::OutOfMemory {
                want: 21,
                largest_free: 20
            })
        );
        slab.check_invariants();
        // After freeing 2, growth absorbs the following run in place.
        slab.free_id(2).unwrap();
        assert_eq!(slab.realloc(1, 50), Ok((0, false)));
        assert_eq!(slab.placement(1), Some((0, 50)));
        assert_eq!(slab.stats().largest_free, 50);
        slab.check_invariants();
    }

    #[test]
    fn realloc_moves_when_fenced_on_both_sides() {
        // B (20..40) is fenced by live A on the left and live C on the
        // right: growth cannot extend in place and must move whole.
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 20).unwrap(); // A: 0..20
        slab.alloc(2, 20).unwrap(); // B: 20..40
        slab.alloc(3, 20).unwrap(); // C: 40..60, hole 60..100
        assert_eq!(slab.realloc(2, 30), Ok((60, true)));
        assert_eq!(slab.placement(2), Some((60, 30)));
        // The old 20..40 range is back in the pool, merged with nothing
        // (neighbours live): largest free run is max(20, 10).
        assert_eq!(slab.stats().largest_free, 20);
        slab.check_invariants();
        // Bytes still add up exactly after the move.
        assert_eq!(slab.stats().used, 70);
    }

    #[test]
    fn realloc_shrink_splits_tail() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 60).unwrap();
        assert_eq!(slab.realloc(1, 20), Ok((0, false)));
        assert_eq!(slab.stats().largest_free, 80);
        assert_eq!(slab.alloc(2, 80), Ok(20));
        slab.check_invariants();
    }

    #[test]
    fn realloc_unknown_and_zero_rejected() {
        let mut slab = SlabAllocator::new(64);
        assert_eq!(slab.realloc(9, 10), Err(AllocError::UnknownId(9)));
        assert!(slab.alloc(1, 0).is_err());
        assert_eq!(slab.free_id(1), Err(AllocError::UnknownId(1)));
    }

    #[test]
    fn compact_plan_slides_down_in_offset_order() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(3, 10).unwrap();
        slab.alloc(1, 20).unwrap();
        slab.alloc(2, 30).unwrap();
        slab.free_id(1).unwrap();
        // Offset order (0,3,10),(30,2,30): id 3 home, id 2 slides 30->10.
        let moves = slab.plan_compact();
        assert_eq!(
            moves,
            vec![SlabMove {
                id: 2,
                from: 30,
                to: 10,
                bytes: 30
            }]
        );
        slab.apply_compact(&moves);
        assert_eq!(slab.placement(2), Some((10, 30)));
        assert_eq!(slab.placement(3), Some((0, 10)));
        assert_eq!(slab.stats().largest_free, 60);
        slab.check_invariants();
    }

    #[test]
    fn compact_noop_when_packed() {
        let mut slab = SlabAllocator::new(100);
        slab.alloc(1, 40).unwrap();
        slab.alloc(2, 60).unwrap();
        assert!(slab.plan_compact().is_empty());
    }

    #[test]
    fn error_strings() {
        assert!(
            AllocError::OutOfMemory {
                want: 8,
                largest_free: 4
            }
            .to_string()
            .contains('8')
        );
        assert!(AllocError::UnknownId(3).to_string().contains('3'));
    }
}
