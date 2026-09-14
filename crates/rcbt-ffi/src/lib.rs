//! Safe wrapper over the vendored upstream `libcbt`.
//!
//! BENCH AND TEST TOOLING ONLY. This crate must never become a dependency of
//! the game runtime, the server, or `thessa-rcbt-core`: the C library is an
//! oracle and benchmark target, not the production topology implementation.
//! The API mirrors `thessa-rcbt-core::Tree` one to one (split takes a leaf,
//! merge takes the parent) so both implementations can replay identical
//! operation sequences.
//!
//! The shim is compiled without OpenMP: serial bitfield updates versus the
//! serial Rust tree. Thread-pool scaling is a separate future experiment.

use std::ffi::c_void;

unsafe extern "C" {
    fn thessa_cbt_create(max_depth: i64, depth: i64) -> *mut c_void;
    fn thessa_cbt_release(tree: *mut c_void);
    fn thessa_cbt_reset_to_depth(tree: *mut c_void, depth: i64);
    fn thessa_cbt_split(tree: *mut c_void, id: u64, depth: i64);
    fn thessa_cbt_merge_children(tree: *mut c_void, parent_id: u64, parent_depth: i64);
    fn thessa_cbt_reduce(tree: *mut c_void);
    fn thessa_cbt_node_count(tree: *const c_void) -> i64;
    fn thessa_cbt_decode(
        tree: *const c_void,
        index: i64,
        out_id: *mut u64,
        out_depth: *mut i64,
    ) -> i32;
    fn thessa_cbt_is_leaf(tree: *const c_void, id: u64, depth: i64) -> i32;
    fn thessa_cbt_heap_bytes(tree: *const c_void) -> i64;
    fn thessa_cbt_max_depth(tree: *const c_void) -> i64;
}

/// Upstream requires `max_depth >= 5`; the heap grows as `2^(depth-1)` bytes,
/// so bench scales are capped to keep the reference allocation bounded.
pub const MIN_DEPTH: u8 = 5;
pub const MAX_BENCH_DEPTH: u8 = 24;

#[derive(Debug)]
pub enum FfiError {
    NullTree,
    DepthOutOfRange { depth: u8 },
}

impl std::fmt::Display for FfiError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NullTree => f.write_str("libcbt returned a null tree"),
            Self::DepthOutOfRange { depth } => {
                write!(f, "libcbt bench depth {depth} outside 5..=24")
            }
        }
    }
}

impl std::error::Error for FfiError {}

/// Owned handle to a `libcbt` tree. Deliberately `!Send + !Sync`: the handle
/// is only driven from a single bench thread.
pub struct LibcbtTree {
    raw: *mut c_void,
}

impl Drop for LibcbtTree {
    fn drop(&mut self) {
        // SAFETY: `raw` comes from `thessa_cbt_create` and is released once.
        unsafe {
            thessa_cbt_release(self.raw);
        }
    }
}

impl LibcbtTree {
    pub fn new(max_depth: u8) -> Result<Self, FfiError> {
        Self::at_depth(max_depth, 0)
    }

    pub fn at_depth(max_depth: u8, depth: u8) -> Result<Self, FfiError> {
        if !(MIN_DEPTH..=MAX_BENCH_DEPTH).contains(&max_depth) || depth > max_depth {
            return Err(FfiError::DepthOutOfRange { depth: max_depth });
        }
        // SAFETY: valid depths; null is checked below.
        let raw = unsafe { thessa_cbt_create(max_depth as i64, depth as i64) };
        if raw.is_null() {
            return Err(FfiError::NullTree);
        }
        Ok(Self { raw })
    }

    pub fn max_depth(&self) -> u8 {
        // SAFETY: `raw` is a live tree for the lifetime of `self`.
        unsafe { thessa_cbt_max_depth(self.raw) as u8 }
    }

    pub fn reset_to_depth(&mut self, depth: u8) {
        debug_assert!(depth <= self.max_depth());
        // SAFETY: `raw` is a live tree; depth is range-checked in debug.
        unsafe {
            thessa_cbt_reset_to_depth(self.raw, depth as i64);
        }
    }

    pub fn split(&mut self, id: u64, depth: u8) {
        // SAFETY: `raw` is a live tree; the caller replays valid leaf splits.
        unsafe {
            thessa_cbt_split(self.raw, id, depth as i64);
        }
    }

    /// Merge the two leaf children of `parent`, mirroring `Tree::merge`.
    pub fn merge_children(&mut self, parent_id: u64, parent_depth: u8) {
        // SAFETY: `raw` is a live tree; the caller replays valid merges.
        unsafe {
            thessa_cbt_merge_children(self.raw, parent_id, parent_depth as i64);
        }
    }

    /// Commit pending bit writes: full decode pass plus sum reduction,
    /// i.e. the honest per-batch commit cost on the C side.
    pub fn reduce(&mut self) {
        // SAFETY: `raw` is a live tree.
        unsafe {
            thessa_cbt_reduce(self.raw);
        }
    }

    pub fn node_count(&self) -> usize {
        // SAFETY: `raw` is a live tree.
        unsafe { thessa_cbt_node_count(self.raw) as usize }
    }

    pub fn decode(&self, index: usize) -> (u64, u8) {
        let mut id = 0_u64;
        let mut depth = 0_i64;
        // SAFETY: `raw` is live, index is in range by contract, out-pointers
        // are valid stack slots for the duration of the call.
        unsafe {
            thessa_cbt_decode(self.raw, index as i64, &mut id, &mut depth);
        }
        (id, depth as u8)
    }

    pub fn is_leaf(&self, id: u64, depth: u8) -> bool {
        // SAFETY: `raw` is a live tree.
        unsafe { thessa_cbt_is_leaf(self.raw, id, depth as i64) != 0 }
    }

    pub fn heap_bytes(&self) -> usize {
        // SAFETY: `raw` is a live tree.
        unsafe { thessa_cbt_heap_bytes(self.raw) as usize }
    }

    /// All leaves in left-to-right decode order.
    pub fn leaves(&self) -> Vec<(u64, u8)> {
        (0..self.node_count()).map(|i| self.decode(i)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_merge_roundtrip_matches_rust_tree_semantics() {
        let mut tree = LibcbtTree::new(10).unwrap();
        assert_eq!(tree.node_count(), 1);
        tree.split(1, 0);
        tree.reduce();
        assert_eq!(tree.node_count(), 2);
        assert_eq!(tree.leaves(), vec![(2, 1), (3, 1)]);
        assert!(tree.is_leaf(2, 1));
        tree.split(2, 1);
        tree.reduce();
        assert_eq!(tree.node_count(), 3);
        // Merge the two children of node 2 back into their parent.
        tree.merge_children(2, 1);
        tree.reduce();
        assert_eq!(tree.leaves(), vec![(2, 1), (3, 1)]);
    }

    #[test]
    fn out_of_range_depth_is_rejected() {
        assert!(LibcbtTree::new(4).is_err());
        assert!(LibcbtTree::new(25).is_err());
    }
}
