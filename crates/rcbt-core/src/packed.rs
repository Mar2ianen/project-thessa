//! Packed single-threaded CBT: dense active-bitfield plus per-node sums.
//!
//! Same observable semantics as [`crate::Tree`] (split a leaf, merge the two
//! leaf children of a parent, left-to-right leaf order), but without pointer
//! chasing or allocation in the hot path: one bit op per touched node plus a
//! plain scalar ancestor walk. The layout is deliberately NOT shared with the
//! GPU mirror or `libcbt` — only leaf sets and counts are comparable across
//! implementations, never heap bytes.
//!
//! Memory is `2^(D+1)` u32 sums plus `2^(D+1)` bits, so depths are capped at
//! [`MAX_PACKED_DEPTH`] (20: 8 MiB sums + 256 KiB bits). Deeper trees stay on
//! [`crate::Tree`]; the crossover bench picks the implementation by measured
//! cost, and this cap is part of that contract.

use crate::{Node, TreeError};

/// Depth cap from the dense-sums footprint (8 MiB at 20, 32 MiB at 21).
pub const MAX_PACKED_DEPTH: u8 = 20;

/// Dense sparse-commit tree. Single-threaded by design; cross-thread use
/// needs external synchronization (same rule as the rest of this crate).
#[derive(Debug, Clone)]
pub struct PackedTree {
    max_depth: u8,
    active: Vec<u64>,
    sums: Vec<u32>,
}

impl PackedTree {
    pub fn new(max_depth: u8) -> Result<Self, TreeError> {
        if max_depth == 0 || max_depth > MAX_PACKED_DEPTH {
            return Err(TreeError::InvalidMaxDepth { depth: max_depth });
        }
        let nodes = 1_usize << (max_depth + 1);
        let mut tree = Self {
            max_depth,
            active: vec![0; nodes.div_ceil(64)],
            sums: vec![0; nodes],
        };
        tree.reset_root();
        Ok(tree)
    }

    pub fn at_depth(max_depth: u8, depth: u8) -> Result<Self, TreeError> {
        let mut tree = Self::new(max_depth)?;
        tree.reset_to_depth(depth)?;
        Ok(tree)
    }

    pub const fn max_depth(&self) -> u8 {
        self.max_depth
    }

    pub fn reset_root(&mut self) {
        self.active.fill(0);
        self.sums.fill(0);
        self.set_bit(1);
        self.sums[1] = 1;
    }

    pub fn reset_to_depth(&mut self, depth: u8) -> Result<(), TreeError> {
        if depth > self.max_depth {
            return Err(TreeError::InvalidMaxDepth { depth });
        }
        self.active.fill(0);
        self.sums.fill(0);
        let first = 1u64 << depth;
        let count = 1u64 << depth;
        for id in first..first + count {
            let bit = (id - 1) as usize;
            self.active[bit / 64] |= 1 << (bit % 64);
            self.sums[id as usize] = 1;
        }
        for d in (0..depth).rev() {
            let level_count = 1u32 << (depth - d);
            for id in (1u64 << d)..(1u64 << (d + 1)) {
                self.sums[id as usize] = level_count;
            }
        }
        Ok(())
    }

    #[inline]
    fn set_bit(&mut self, id: u64) {
        let bit = (id - 1) as usize;
        self.active[bit / 64] |= 1 << (bit % 64);
    }

    #[inline]
    fn clear_bit(&mut self, id: u64) {
        let bit = (id - 1) as usize;
        self.active[bit / 64] &= !(1 << (bit % 64));
    }

    #[inline]
    pub fn is_active(&self, id: u64) -> bool {
        let bit = (id - 1) as usize;
        (self.active[bit / 64] >> (bit % 64)) & 1 == 1
    }

    pub fn leaf_count(&self) -> usize {
        self.sums[1] as usize
    }

    pub fn contains(&self, node: Node) -> bool {
        self.is_active(node.id())
    }

    pub fn split(&mut self, node: Node) -> Result<[Node; 2], TreeError> {
        let (id, depth) = (node.id(), node.depth());
        if depth >= self.max_depth {
            return Err(TreeError::AtMaximumDepth(node));
        }
        if !self.is_active(id) {
            return Err(TreeError::NotALeaf(node));
        }
        let left = Node::new(id * 2, depth + 1).expect("child in range");
        let right = Node::new(id * 2 + 1, depth + 1).expect("child in range");
        self.set_bit(left.id());
        self.set_bit(right.id());
        self.clear_bit(id);
        self.sums[left.id() as usize] = 1;
        self.sums[right.id() as usize] = 1;
        self.sums[id as usize] = 2;
        let mut ancestor = id / 2;
        while ancestor >= 1 {
            self.sums[ancestor as usize] += 1;
            if ancestor == 1 {
                break;
            }
            ancestor /= 2;
        }
        Ok([left, right])
    }

    pub fn merge(&mut self, parent: Node) -> Result<(), TreeError> {
        if parent.is_root() {
            return Err(TreeError::CannotMergeRoot);
        }
        let Some([left, right]) = parent.children() else {
            return Err(TreeError::ChildrenNotLeaves(parent));
        };
        if !self.is_active(left.id())
            || !self.is_active(right.id())
            || self.sums[left.id() as usize] != 1
            || self.sums[right.id() as usize] != 1
        {
            return Err(TreeError::ChildrenNotLeaves(parent));
        }
        self.clear_bit(left.id());
        self.clear_bit(right.id());
        self.set_bit(parent.id());
        self.sums[left.id() as usize] = 0;
        self.sums[right.id() as usize] = 0;
        self.sums[parent.id() as usize] = 1;
        let mut ancestor = parent.id() / 2;
        while ancestor >= 1 {
            self.sums[ancestor as usize] -= 1;
            if ancestor == 1 {
                break;
            }
            ancestor /= 2;
        }
        Ok(())
    }

    pub fn apply_one(&mut self, update: crate::Update) -> Result<(), TreeError> {
        match update {
            crate::Update::Split(node) => {
                self.split(node)?;
            }
            crate::Update::Merge(node) => {
                self.merge(node)?;
            }
        }
        Ok(())
    }

    pub fn apply_batch(&mut self, updates: &[crate::Update]) -> Result<(), TreeError> {
        let mut candidate = self.clone();
        for update in updates {
            candidate.apply_one(*update)?;
        }
        *self = candidate;
        Ok(())
    }

    /// Ordered leaf list, same left-to-right order as [`crate::Tree::leaves`].
    pub fn leaves(&self) -> Vec<Node> {
        let mut out = Vec::with_capacity(self.leaf_count());
        self.collect(Node::root(), &mut out);
        out
    }

    fn collect(&self, node: Node, out: &mut Vec<Node>) {
        if self.is_active(node.id()) {
            out.push(node);
        } else if let Some([left, right]) = node.children() {
            // Depth guard: inactive nodes below max_depth always have
            // children states; beyond it the tree cannot grow.
            if node.depth() < self.max_depth {
                self.collect(left, out);
                self.collect(right, out);
            }
        }
    }

    /// Heap footprint in bytes (sums + bits). Reported by benches so the
    /// memory side of the tradeoff stays visible next to time.
    pub fn footprint_bytes(&self) -> usize {
        self.sums.len() * 4 + self.active.len() * 8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Tree;

    fn drive_both(max_depth: u8, ops: &[(u64, u8, u8)]) {
        let mut a = Tree::new(max_depth).unwrap();
        let mut b = PackedTree::new(max_depth.min(MAX_PACKED_DEPTH)).unwrap();
        for (id, depth, kind) in ops {
            let node = Node::new(*id, *depth).unwrap();
            match kind {
                0 => {
                    let ra = a.split(node);
                    let rb = b.split(node);
                    assert_eq!(ra.is_ok(), rb.is_ok(), "split agreement {node:?}");
                }
                _ => {
                    let ra = a.merge(node);
                    let rb = b.merge(node);
                    assert_eq!(ra.is_ok(), rb.is_ok(), "merge agreement {node:?}");
                }
            }
            assert_eq!(
                a.leaf_count(),
                b.leaf_count(),
                "count agreement after {node:?}"
            );
            assert_eq!(a.leaves(), b.leaves(), "leaf agreement after {node:?}");
        }
    }

    #[test]
    fn packed_matches_tree_on_refine_and_sparse_sequences() {
        // Full refine to depth 8, one level per batch.
        let mut ops = Vec::new();
        let mut level: Vec<(u64, u8)> = vec![(1, 0)];
        for _ in 0..8 {
            let mut next = Vec::new();
            for (id, d) in level {
                ops.push((id, d, 0));
                next.push((id * 2, d + 1));
                next.push((id * 2 + 1, d + 1));
            }
            level = next;
        }
        // Sparse oscillation on top: merge whole depth-7 pairs back
        // (valid on both), re-split two of them, then invalid ops that
        // must fail identically (merge of non-leaves, split of interior).
        for parent in [128u64, 129, 200, 255] {
            ops.push((parent, 7, 1));
        }
        for parent in [128u64, 255] {
            ops.push((parent, 7, 0));
        }
        ops.push((3, 1, 1)); // children split further: invalid both sides
        ops.push((128, 7, 0)); // interior after re-split: invalid both sides
        drive_both(10, &ops);
    }

    #[test]
    fn packed_rejects_depth_beyond_cap() {
        assert!(PackedTree::new(0).is_err());
        assert!(PackedTree::new(21).is_err());
        let mut t = PackedTree::new(8).unwrap();
        assert!(t.reset_to_depth(9).is_err());
        assert_eq!(
            t.footprint_bytes(),
            (1_usize << 9) * 4 + (1_usize << 9).div_ceil(64) * 8
        );
    }

    #[test]
    fn packed_reset_full_matches_tree() {
        let mut a = Tree::new(12).unwrap();
        a.reset_to_depth(6).unwrap();
        let mut b = PackedTree::new(12).unwrap();
        b.reset_to_depth(6).unwrap();
        assert_eq!(a.leaves(), b.leaves());
    }

    /// Wave-like dynamic load: 60 frames of alternating full split-all and
    /// merge-all waves over a small tree. Topology must stay bounded and
    /// identical to `Tree` on every frame (this is the `cargo test`
    /// counterpart of the `dynamic` bench scenarios).
    #[test]
    fn packed_tracks_oscillating_wave_load() {
        let mut a = Tree::at_depth(10, 4).unwrap();
        let mut b = PackedTree::at_depth(10, 4).unwrap();
        for frame in 0..60 {
            if frame % 2 == 0 {
                let splits: Vec<Node> = a.leaves();
                for leaf in splits {
                    assert!(a.split(leaf).is_ok());
                    assert!(b.split(leaf).is_ok());
                }
            } else {
                // Merge every pair whose children are both leaves.
                let mut parents = Vec::new();
                for leaf in a.leaves() {
                    if leaf.id() & 1 == 0
                        && let Some(p) = leaf.parent()
                    {
                        parents.push(p);
                    }
                }
                parents.sort();
                parents.dedup();
                for p in parents {
                    let ra = a.merge(p);
                    let rb = b.merge(p);
                    assert_eq!(ra.is_ok(), rb.is_ok(), "merge agreement {p:?}");
                }
            }
            assert_eq!(a.leaf_count(), b.leaf_count(), "frame {frame} count");
            assert_eq!(a.leaves(), b.leaves(), "frame {frame} leaves");
            assert!(a.leaf_count() <= 1 << 9, "frame {frame} bounded");
        }
    }
}
