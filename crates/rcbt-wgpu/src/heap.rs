//! CPU mirror of the GPU CBT heap layout.
//!
//! Pure Rust, no wgpu types: the same layout code initializes GPU buffers,
//! verifies readbacks, and documents the contract the WGSL kernels implement.
//!
//! Layout (heap ids are dense per level, so no offset tables are needed):
//!
//! ```text
//! node (id, depth), root = (1, 0), children of id = 2*id, 2*id+1
//! active bit index  = id - 1            (bitfield over every node)
//! sums index        = id                (u32 leaf-count under the node)
//! ```
//!
//! Invariants the kernels rely on (batches replayed from validated CPU
//! sequences satisfy them by construction):
//!
//! ```text
//! split(leaf):   children must be inactive; sets both, clears leaf,
//!                sums[children] = 1, sums[leaf] = 2, ancestors += 1
//! merge(parent): both children must be active leaves; clears both, sets
//!                parent, sums[children] = 0, sums[parent] = 1, ancestors -= 1
//! ```
//!
//! Only `u32` atomics are used, so the layout stays inside portable WGSL.

/// Portable limits for the GPU heap: sums need `2^(D+1)` u32 words.
pub const MAX_GPU_DEPTH: u8 = 20;

#[derive(Debug, Clone)]
pub struct CpuMirror {
    max_depth: u8,
    active: Vec<u32>,
    sums: Vec<u32>,
}

impl CpuMirror {
    pub fn new(max_depth: u8) -> Result<Self, &'static str> {
        if max_depth == 0 || max_depth > MAX_GPU_DEPTH {
            return Err("GPU heap depth outside 1..=20");
        }
        let nodes = 1_usize << (max_depth + 1);
        let words = nodes.div_ceil(32);
        let mut mirror = Self {
            max_depth,
            active: vec![0; words],
            sums: vec![0; nodes],
        };
        mirror.reset_root();
        Ok(mirror)
    }

    pub const fn max_depth(&self) -> u8 {
        self.max_depth
    }

    /// Node capacity (heap ids `1 .. 2^(D+1)`).
    pub fn capacity(&self) -> usize {
        1_usize << (self.max_depth + 1)
    }

    pub fn reset_root(&mut self) {
        self.active.fill(0);
        self.sums.fill(0);
        self.set_bit(1);
        self.sums[1] = 1;
    }

    fn set_bit(&mut self, id: u64) {
        let bit = (id - 1) as usize;
        self.active[bit / 32] |= 1 << (bit % 32);
    }

    fn clear_bit(&mut self, id: u64) {
        let bit = (id - 1) as usize;
        self.active[bit / 32] &= !(1 << (bit % 32));
    }

    pub fn is_active(&self, id: u64) -> bool {
        let bit = (id - 1) as usize;
        self.active[bit / 32] & (1 << (bit % 32)) != 0
    }

    pub fn split(&mut self, id: u64) -> Result<(), &'static str> {
        if !self.is_active(id) || id >= (1 << self.max_depth) {
            return Err("GPU mirror: split of inactive or ceil node");
        }
        let (left, right) = (id * 2, id * 2 + 1);
        self.set_bit(left);
        self.set_bit(right);
        self.clear_bit(id);
        self.sums[left as usize] = 1;
        self.sums[right as usize] = 1;
        self.sums[id as usize] = 2;
        let mut ancestor = id / 2;
        while ancestor >= 1 {
            self.sums[ancestor as usize] += 1;
            if ancestor == 1 {
                break;
            }
            ancestor /= 2;
        }
        Ok(())
    }

    pub fn merge_children(&mut self, parent: u64) -> Result<(), &'static str> {
        if parent < 1 || !self.is_active(parent * 2) || !self.is_active(parent * 2 + 1) {
            return Err("GPU mirror: merge of non-leaf pair");
        }
        // Both children are leaves iff each subtree holds exactly one leaf.
        if self.sums[(parent * 2) as usize] != 1 || self.sums[(parent * 2 + 1) as usize] != 1 {
            return Err("GPU mirror: merge of non-leaf pair");
        }
        self.clear_bit(parent * 2);
        self.clear_bit(parent * 2 + 1);
        self.set_bit(parent);
        self.sums[(parent * 2) as usize] = 0;
        self.sums[(parent * 2 + 1) as usize] = 0;
        self.sums[parent as usize] = 1;
        let mut ancestor = parent / 2;
        while ancestor >= 1 {
            self.sums[ancestor as usize] -= 1;
            if ancestor == 1 {
                break;
            }
            ancestor /= 2;
        }
        Ok(())
    }

    pub fn node_count(&self) -> u32 {
        self.sums[1]
    }

    pub fn active_words(&self) -> &[u32] {
        &self.active
    }

    pub fn sums(&self) -> &[u32] {
        &self.sums
    }

    pub fn active_byte_len(&self) -> usize {
        self.active.len() * 4
    }

    pub fn sums_byte_len(&self) -> usize {
        self.sums.len() * 4
    }

    /// Ordered leaf list, same left-to-right order as `Tree::leaves`.
    pub fn leaves(&self) -> Vec<(u64, u8)> {
        let mut out = Vec::with_capacity(self.sums[1] as usize);
        self.collect(1, 0, &mut out);
        out
    }

    fn collect(&self, id: u64, depth: u8, out: &mut Vec<(u64, u8)>) {
        if self.is_active(id) {
            out.push((id, depth));
        } else {
            self.collect(id * 2, depth + 1, out);
            self.collect(id * 2 + 1, depth + 1, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mirror_matches_tree_leaf_sets() {
        let mut mirror = CpuMirror::new(10).unwrap();
        assert_eq!(mirror.leaves(), vec![(1, 0)]);
        mirror.split(1).unwrap();
        mirror.split(2).unwrap();
        assert_eq!(mirror.leaves(), vec![(4, 2), (5, 2), (3, 1)]);
        assert_eq!(mirror.node_count(), 3);
        mirror.merge_children(2).unwrap();
        assert_eq!(mirror.leaves(), vec![(2, 1), (3, 1)]);
    }

    #[test]
    fn mirror_rejects_invalid_ops() {
        let mut mirror = CpuMirror::new(5).unwrap();
        assert!(mirror.split(3).is_err());
        assert!(mirror.merge_children(1).is_err());
        mirror.split(1).unwrap();
        // Merging the root pair is structurally fine for the mirror.
        mirror.merge_children(1).unwrap();
        assert_eq!(mirror.leaves(), vec![(1, 0)]);
    }
}
