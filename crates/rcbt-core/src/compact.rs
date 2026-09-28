//! Exact rank-count storage with large_cbt-style reduced precision.
//!
//! A node at depth `d` can contain at most `2^(D-d)` leaves, so its exact
//! count needs only `D-d+1` bits. `CompactTree` stores those counters in
//! per-level bit-packed arrays instead of one `u32` per heap node. No count
//! is approximated and mutations still update only the changed node path.
//!
//! This is a CPU/reference representation for the compact GPU contract. The
//! existing [`crate::packed::PackedTree`] remains the low-latency scalar
//! implementation until the target GPU workload proves that packed bitfield
//! writes are the better runtime choice.

use std::collections::HashMap;

use crate::{Node, TreeError};

/// Maximum depth for the compact dense representation.
pub const MAX_COMPACT_DEPTH: u8 = 20;

#[derive(Debug, Clone)]
struct PackedLevel {
    width: u8,
    values: Vec<u64>,
}

impl PackedLevel {
    fn new(count: usize, width: u8) -> Self {
        let bits = count
            .checked_mul(width as usize)
            .expect("compact level bit count fits usize");
        Self {
            width,
            values: vec![0; bits.div_ceil(64)],
        }
    }

    fn get(&self, index: usize) -> u32 {
        let bit = index * self.width as usize;
        let word = bit / 64;
        let shift = bit % 64;
        let width = self.width as usize;
        let mask = mask(width);
        let available = 64 - shift;
        if width <= available {
            ((self.values[word] >> shift) & mask) as u32
        } else {
            let lower = self.values[word] >> shift;
            let upper = self.values[word + 1] << available;
            ((lower | upper) & mask) as u32
        }
    }

    fn set(&mut self, index: usize, value: u32) {
        let bit = index * self.width as usize;
        let word = bit / 64;
        let shift = bit % 64;
        let width = self.width as usize;
        let value = u64::from(value) & mask(width);
        let available = 64 - shift;
        if width <= available {
            let field = mask(width) << shift;
            self.values[word] = (self.values[word] & !field) | (value << shift);
        } else {
            let lower_field = u64::MAX << shift;
            self.values[word] = (self.values[word] & !lower_field) | (value << shift);
            let upper_width = width - available;
            let upper_field = mask(upper_width);
            self.values[word + 1] =
                (self.values[word + 1] & !upper_field) | ((value >> available) & upper_field);
        }
    }

    fn bytes(&self) -> usize {
        self.values.len() * std::mem::size_of::<u64>()
    }
}

fn mask(width: usize) -> u64 {
    if width == 64 {
        u64::MAX
    } else {
        (1_u64 << width) - 1
    }
}

/// Dense CBT with exact, per-level bit-packed rank counts.
#[derive(Debug, Clone)]
pub struct CompactTree {
    max_depth: u8,
    active: Vec<u64>,
    sums: Vec<PackedLevel>,
}

impl CompactTree {
    pub fn new(max_depth: u8) -> Result<Self, TreeError> {
        if max_depth == 0 || max_depth > MAX_COMPACT_DEPTH {
            return Err(TreeError::InvalidMaxDepth { depth: max_depth });
        }
        let nodes = 1_usize << (max_depth + 1);
        let sums = (0..=max_depth)
            .map(|depth| {
                let count = 1_usize << depth;
                let width = max_depth - depth + 1;
                PackedLevel::new(count, width)
            })
            .collect();
        let mut tree = Self {
            max_depth,
            active: vec![0; nodes.div_ceil(64)],
            sums,
        };
        tree.reset_to_root();
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

    pub fn reset_to_root(&mut self) {
        self.active.fill(0);
        for level in &mut self.sums {
            level.values.fill(0);
        }
        self.set_active(1, true);
        self.set_sum(Node::root(), 1);
    }

    pub fn reset_to_depth(&mut self, depth: u8) -> Result<(), TreeError> {
        if depth > self.max_depth {
            return Err(TreeError::InvalidMaxDepth { depth });
        }
        self.active.fill(0);
        for level in &mut self.sums {
            level.values.fill(0);
        }
        let first = 1_u64 << depth;
        let count = 1_u64 << depth;
        for id in first..first + count {
            self.set_active(id, true);
            self.set_sum(Node::new(id, depth).expect("reset node in range"), 1);
        }
        for current_depth in (0..depth).rev() {
            let count = 1_u32 << (depth - current_depth);
            for id in (1_u64 << current_depth)..(1_u64 << (current_depth + 1)) {
                self.set_sum(
                    Node::new(id, current_depth).expect("reset node in range"),
                    count,
                );
            }
        }
        Ok(())
    }

    fn set_active(&mut self, id: u64, state: bool) {
        let bit = (id - 1) as usize;
        let word = &mut self.active[bit / 64];
        let mask = 1_u64 << (bit % 64);
        if state {
            *word |= mask;
        } else {
            *word &= !mask;
        }
    }

    pub fn is_active(&self, id: u64) -> bool {
        if id == 0 || id >= (1_u64 << (self.max_depth + 1)) {
            return false;
        }
        let bit = (id - 1) as usize;
        self.active[bit / 64] & (1_u64 << (bit % 64)) != 0
    }

    fn set_sum(&mut self, node: Node, value: u32) {
        let level = &mut self.sums[node.depth() as usize];
        let first = 1_u64 << node.depth();
        level.set((node.id() - first) as usize, value);
    }

    pub fn sum(&self, node: Node) -> u32 {
        if node.depth() > self.max_depth {
            return 0;
        }
        let level = &self.sums[node.depth() as usize];
        let first = 1_u64 << node.depth();
        level.get((node.id() - first) as usize)
    }

    pub fn leaf_count(&self) -> usize {
        self.sum(Node::root()) as usize
    }

    pub fn contains(&self, node: Node) -> bool {
        self.is_active(node.id())
    }

    pub fn split(&mut self, node: Node) -> Result<[Node; 2], TreeError> {
        if node.depth() >= self.max_depth {
            return Err(TreeError::AtMaximumDepth(node));
        }
        if !self.is_active(node.id()) {
            return Err(TreeError::NotALeaf(node));
        }
        let [left, right] = node.children().expect("depth checked above");
        self.set_active(node.id(), false);
        self.set_active(left.id(), true);
        self.set_active(right.id(), true);
        self.set_sum(left, 1);
        self.set_sum(right, 1);
        self.set_sum(node, 2);
        self.adjust_ancestors(node, 1);
        Ok([left, right])
    }

    pub fn merge(&mut self, parent: Node) -> Result<(), TreeError> {
        if parent.is_root() {
            return Err(TreeError::CannotMergeRoot);
        }
        if parent.depth() >= self.max_depth {
            return Err(TreeError::ChildrenNotLeaves(parent));
        }
        let Some([left, right]) = parent.children() else {
            return Err(TreeError::ChildrenNotLeaves(parent));
        };
        if !self.is_active(left.id())
            || !self.is_active(right.id())
            || self.sum(left) != 1
            || self.sum(right) != 1
        {
            return Err(TreeError::ChildrenNotLeaves(parent));
        }
        self.set_active(left.id(), false);
        self.set_active(right.id(), false);
        self.set_active(parent.id(), true);
        self.set_sum(left, 0);
        self.set_sum(right, 0);
        self.set_sum(parent, 1);
        self.adjust_ancestors(parent, -1);
        Ok(())
    }

    pub fn apply_one(&mut self, update: crate::Update) -> Result<(), TreeError> {
        match update {
            crate::Update::Split(node) => self.split(node).map(|_| ()),
            crate::Update::Merge(node) => self.merge(node),
        }
    }

    pub fn apply_batch(&mut self, updates: &[crate::Update]) -> Result<(), TreeError> {
        let mut candidate = self.clone();
        let mut ancestor_deltas = HashMap::<u64, i32>::new();
        for update in updates {
            candidate.apply_one_batched(*update, &mut ancestor_deltas)?;
        }
        for (id, delta) in ancestor_deltas {
            let node = Node::from_heap_id(id).expect("recorded ancestor is a valid node");
            let value = candidate.sum(node) as i32 + delta;
            debug_assert!(value >= 0);
            candidate.set_sum(node, value as u32);
        }
        *self = candidate;
        Ok(())
    }

    fn apply_one_batched(
        &mut self,
        update: crate::Update,
        ancestor_deltas: &mut HashMap<u64, i32>,
    ) -> Result<(), TreeError> {
        match update {
            crate::Update::Split(node) => {
                if node.depth() >= self.max_depth {
                    return Err(TreeError::AtMaximumDepth(node));
                }
                if !self.is_active(node.id()) {
                    return Err(TreeError::NotALeaf(node));
                }
                let [left, right] = node.children().expect("depth checked above");
                self.set_active(node.id(), false);
                self.set_active(left.id(), true);
                self.set_active(right.id(), true);
                Self::materialize_sum(&mut self.sums, left, 1, ancestor_deltas);
                Self::materialize_sum(&mut self.sums, right, 1, ancestor_deltas);
                Self::materialize_sum(&mut self.sums, node, 2, ancestor_deltas);
                self.record_ancestor_delta(node, 1, ancestor_deltas);
                Ok(())
            }
            crate::Update::Merge(parent) => {
                if parent.is_root() {
                    return Err(TreeError::CannotMergeRoot);
                }
                let Some([left, right]) = parent.children() else {
                    return Err(TreeError::ChildrenNotLeaves(parent));
                };
                if !self.is_active(left.id())
                    || !self.is_active(right.id())
                    || Self::effective_sum(&self.sums, left, ancestor_deltas) != 1
                    || Self::effective_sum(&self.sums, right, ancestor_deltas) != 1
                {
                    return Err(TreeError::ChildrenNotLeaves(parent));
                }
                self.set_active(left.id(), false);
                self.set_active(right.id(), false);
                self.set_active(parent.id(), true);
                Self::materialize_sum(&mut self.sums, left, 0, ancestor_deltas);
                Self::materialize_sum(&mut self.sums, right, 0, ancestor_deltas);
                Self::materialize_sum(&mut self.sums, parent, 1, ancestor_deltas);
                self.record_ancestor_delta(parent, -1, ancestor_deltas);
                Ok(())
            }
        }
    }

    fn record_ancestor_delta(
        &self,
        node: Node,
        delta: i32,
        ancestor_deltas: &mut HashMap<u64, i32>,
    ) {
        let mut ancestor = node.parent();
        while let Some(current) = ancestor {
            *ancestor_deltas.entry(current.id()).or_default() += delta;
            ancestor = current.parent();
        }
    }

    fn effective_sum(sums: &[PackedLevel], node: Node, ancestor_deltas: &HashMap<u64, i32>) -> i32 {
        sums[node.depth() as usize].get((node.id() - (1_u64 << node.depth())) as usize) as i32
            + ancestor_deltas.get(&node.id()).copied().unwrap_or(0)
    }

    fn materialize_sum(
        sums: &mut [PackedLevel],
        node: Node,
        value: u32,
        ancestor_deltas: &mut HashMap<u64, i32>,
    ) {
        ancestor_deltas.remove(&node.id());
        let first = 1_u64 << node.depth();
        sums[node.depth() as usize].set((node.id() - first) as usize, value);
    }

    fn adjust_ancestors(&mut self, node: Node, delta: i32) {
        let mut ancestor = node.parent();
        while let Some(current) = ancestor {
            let value = self.sum(current) as i32 + delta;
            debug_assert!(value >= 0);
            self.set_sum(current, value as u32);
            ancestor = current.parent();
        }
    }

    pub fn leaves(&self) -> Vec<Node> {
        let mut out = Vec::with_capacity(self.leaf_count());
        self.collect(Node::root(), &mut out);
        out
    }

    fn collect(&self, node: Node, out: &mut Vec<Node>) {
        if self.is_active(node.id()) {
            out.push(node);
        } else if node.depth() < self.max_depth {
            let [left, right] = node.children().expect("depth guard");
            self.collect(left, out);
            self.collect(right, out);
        }
    }

    /// Exact storage footprint, including the active bitfield and all packed
    /// rank levels. It excludes allocator capacity outside these buffers.
    pub fn footprint_bytes(&self) -> usize {
        self.active.len() * std::mem::size_of::<u64>()
            + self.sums.iter().map(PackedLevel::bytes).sum::<usize>()
    }

    pub fn active_buffer_bytes(&self) -> Vec<u8> {
        self.active
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }

    /// Concatenated little-endian level buffers. Level `d` starts at
    /// [`Self::sum_level_offset_bytes`]; the explicit offsets keep WGSL and
    /// other backends independent of Rust's `Vec` layout.
    pub fn sums_buffer_bytes(&self) -> Vec<u8> {
        self.sums
            .iter()
            .flat_map(|level| level.values.iter().flat_map(|word| word.to_le_bytes()))
            .collect()
    }

    pub fn sums_buffer_bytes_len(&self) -> usize {
        self.sums.iter().map(PackedLevel::bytes).sum()
    }

    pub fn sum_level_offset_bytes(&self, depth: u8) -> usize {
        self.sums[..depth as usize]
            .iter()
            .map(PackedLevel::bytes)
            .sum()
    }

    pub fn sum_width_bits(&self, depth: u8) -> u8 {
        self.sums[depth as usize].width
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Tree, Update};

    #[test]
    fn packed_fields_cross_word_boundaries_without_losing_bits() {
        let mut level = PackedLevel::new(20, 21);
        for index in 0..20 {
            level.set(index, index as u32 + 1);
        }
        for index in 0..20 {
            assert_eq!(level.get(index), index as u32 + 1);
        }
    }

    #[test]
    fn compact_matches_reference_tree_on_sparse_operations() {
        let mut reference = Tree::at_depth(12, 5).unwrap();
        let mut compact = CompactTree::at_depth(12, 5).unwrap();
        let updates = [
            Update::Split(Node::new(32, 5).unwrap()),
            Update::Split(Node::new(64, 6).unwrap()),
            Update::Merge(Node::new(64, 6).unwrap()),
            Update::Merge(Node::new(32, 5).unwrap()),
            Update::Split(Node::new(33, 5).unwrap()),
            Update::Merge(Node::new(33, 5).unwrap()),
            Update::Merge(Node::new(16, 4).unwrap()),
        ];
        for update in updates {
            assert_eq!(
                reference.apply_one(update).is_ok(),
                compact.apply_one(update).is_ok()
            );
            assert_eq!(reference.leaves(), compact.leaves());
            assert_eq!(reference.leaf_count(), compact.leaf_count());
        }
    }

    #[test]
    fn compact_batched_ancestor_deltas_match_sequential_commit() {
        let updates = [
            Update::Split(Node::new(32, 5).unwrap()),
            Update::Split(Node::new(64, 6).unwrap()),
            Update::Merge(Node::new(64, 6).unwrap()),
            Update::Merge(Node::new(32, 5).unwrap()),
            Update::Split(Node::new(33, 5).unwrap()),
            Update::Merge(Node::new(33, 5).unwrap()),
            Update::Merge(Node::new(16, 4).unwrap()),
        ];
        let mut sequential = CompactTree::at_depth(12, 5).unwrap();
        for update in updates {
            sequential.apply_one(update).unwrap();
        }
        let mut batched = CompactTree::at_depth(12, 5).unwrap();
        batched.apply_batch(&updates).unwrap();
        assert_eq!(batched.leaves(), sequential.leaves());
        assert_eq!(batched.leaf_count(), sequential.leaf_count());
    }

    #[test]
    fn compact_uses_exact_level_widths_and_less_memory_than_u32_sums() {
        let compact = CompactTree::new(MAX_COMPACT_DEPTH).unwrap();
        assert_eq!(compact.sum_width_bits(0), 21);
        assert_eq!(compact.sum_width_bits(10), 11);
        assert_eq!(compact.sum_width_bits(20), 1);
        assert!(compact.footprint_bytes() < (1_usize << 21) * 4);
        assert_eq!(compact.leaf_count(), 1);
    }

    #[test]
    fn compact_out_of_range_queries_and_merges_are_safe() {
        let mut tree = CompactTree::new(4).unwrap();
        let outside = Node::new(32, 5).unwrap();
        assert!(!tree.is_active(0));
        assert!(!tree.is_active(65));
        assert!(!tree.contains(outside));
        assert_eq!(tree.sum(outside), 0);
        assert_eq!(
            tree.merge(outside),
            Err(TreeError::ChildrenNotLeaves(outside))
        );
    }
}
