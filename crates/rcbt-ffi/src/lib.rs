//! Safe wrapper over the vendored upstream `libcbt`.
//!
//! BENCH AND TEST TOOLING ONLY. This crate must never become a dependency of
//! the game runtime, the server, or `thessa-rcbt-core`: the C library is an
//! oracle and benchmark target, not the production topology implementation.
//! The API mirrors `thessa-rcbt-core::Tree` one to one (split takes a leaf,
//! merge takes the parent) so both implementations can replay identical
//! operation sequences.
//!
//! The serial shim compares against the serial Rust tree. The `thessa_mt_`
//! entry points (same source built with OpenMP) back thread-scaling
//! experiments through `MtCbtTree`.

use std::ffi::c_void;

unsafe extern "C" {
    fn thessa_cbt_create(max_depth: i64, depth: i64) -> *mut c_void;
    fn thessa_cbt_release(tree: *mut c_void);
    fn thessa_cbt_reset_to_depth(tree: *mut c_void, depth: i64);
    fn thessa_cbt_split(tree: *mut c_void, id: u64, depth: i64);
    fn thessa_cbt_merge_children(tree: *mut c_void, parent_id: u64, parent_depth: i64);
    fn thessa_cbt_reduce(tree: *mut c_void);
    fn thessa_cbt_reduce_only(tree: *mut c_void);
    fn thessa_cbt_apply_batch(
        tree: *mut c_void,
        ids: *const u64,
        depths: *const i64,
        kinds: *const u8,
        count: i64,
    ) -> i64;
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

    fn thessa_mt_cbt_create(max_depth: i64, depth: i64) -> *mut c_void;
    fn thessa_mt_cbt_release(tree: *mut c_void);
    fn thessa_mt_cbt_reset_to_depth(tree: *mut c_void, depth: i64);
    fn thessa_mt_cbt_split(tree: *mut c_void, id: u64, depth: i64);
    fn thessa_mt_cbt_merge_children(tree: *mut c_void, parent_id: u64, parent_depth: i64);
    fn thessa_mt_cbt_reduce(tree: *mut c_void);
    fn thessa_mt_cbt_reduce_only(tree: *mut c_void);
    fn thessa_mt_cbt_apply_batch(
        tree: *mut c_void,
        ids: *const u64,
        depths: *const i64,
        kinds: *const u8,
        count: i64,
    ) -> i64;
    fn thessa_mt_cbt_node_count(tree: *const c_void) -> i64;
    fn thessa_mt_cbt_decode(
        tree: *const c_void,
        index: i64,
        out_id: *mut u64,
        out_depth: *mut i64,
    ) -> i32;
    fn thessa_mt_cbt_is_leaf(tree: *const c_void, id: u64, depth: i64) -> i32;
    fn thessa_mt_cbt_heap_bytes(tree: *const c_void) -> i64;
    fn thessa_mt_cbt_max_depth(tree: *const c_void) -> i64;
    fn thessa_mt_cbt_set_threads(n: i32);
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

macro_rules! impl_tree {
    ($name:ident, $doc:expr, $create:ident, $release:ident, $reset:ident, $split:ident, $merge:ident, $reduce:ident, $reduce_only:ident, $batch:ident, $count:ident, $decode:ident, $is_leaf:ident, $heap:ident, $max:ident) => {
        #[doc = $doc]
        pub struct $name {
            raw: *mut c_void,
        }

        impl Drop for $name {
            fn drop(&mut self) {
                // SAFETY: `raw` comes from the matching create fn, released once.
                unsafe {
                    $release(self.raw);
                }
            }
        }

        impl $name {
            pub fn new(max_depth: u8) -> Result<Self, FfiError> {
                Self::at_depth(max_depth, 0)
            }

            pub fn at_depth(max_depth: u8, depth: u8) -> Result<Self, FfiError> {
                if !(MIN_DEPTH..=MAX_BENCH_DEPTH).contains(&max_depth) || depth > max_depth {
                    return Err(FfiError::DepthOutOfRange { depth: max_depth });
                }
                // SAFETY: valid depths; null is checked below.
                let raw = unsafe { $create(max_depth as i64, depth as i64) };
                if raw.is_null() {
                    return Err(FfiError::NullTree);
                }
                Ok(Self { raw })
            }

            pub fn max_depth(&self) -> u8 {
                // SAFETY: `raw` is a live tree for the lifetime of `self`.
                unsafe { $max(self.raw) as u8 }
            }

            pub fn reset_to_depth(&mut self, depth: u8) {
                debug_assert!(depth <= self.max_depth());
                // SAFETY: `raw` is a live tree; depth is range-checked in debug.
                unsafe {
                    $reset(self.raw, depth as i64);
                }
            }

            pub fn split(&mut self, id: u64, depth: u8) {
                // SAFETY: `raw` is a live tree; the caller replays valid leaf splits.
                unsafe {
                    $split(self.raw, id, depth as i64);
                }
            }

            /// Merge the two leaf children of `parent`, mirroring `Tree::merge`.
            pub fn merge_children(&mut self, parent_id: u64, parent_depth: u8) {
                // SAFETY: `raw` is a live tree; the caller replays valid merges.
                unsafe {
                    $merge(self.raw, parent_id, parent_depth as i64);
                }
            }

            /// Commit pending bit writes: full decode pass plus sum reduction,
            /// i.e. the honest per-batch commit cost on the C side.
            pub fn reduce(&mut self) {
                // SAFETY: `raw` is a live tree.
                unsafe {
                    $reduce(self.raw);
                }
            }

            /// Commit without the decode pass: only the sum reduction. Valid
            /// after a known op list applied through the entry points above.
            /// This is the scary baseline for sparse-commit comparisons: no
            /// wasted decode work, just the rank-structure refresh.
            pub fn reduce_only(&mut self) {
                // SAFETY: `raw` is a live tree.
                unsafe {
                    $reduce_only(self.raw);
                }
            }

            /// Replay a whole batch across a single FFI boundary. Each tuple
            /// is `(id, depth, kind)` with kind 0 = split leaf, 1 = merge the
            /// children of that parent. Returns the issued write count.
            pub fn apply_batch_ops(&mut self, ops: &[(u64, i64, u8)]) -> usize {
                if ops.is_empty() {
                    return 0;
                }
                let mut ids = Vec::with_capacity(ops.len());
                let mut depths = Vec::with_capacity(ops.len());
                let mut kinds = Vec::with_capacity(ops.len());
                for (id, depth, kind) in ops {
                    ids.push(*id);
                    depths.push(*depth);
                    kinds.push(*kind);
                }
                // SAFETY: `raw` is live; the three slices are valid, equally
                // long, and borrowed for the duration of the call.
                unsafe {
                    $batch(
                        self.raw,
                        ids.as_ptr(),
                        depths.as_ptr(),
                        kinds.as_ptr(),
                        ops.len() as i64,
                    ) as usize
                }
            }

            pub fn node_count(&self) -> usize {
                // SAFETY: `raw` is a live tree.
                unsafe { $count(self.raw) as usize }
            }

            pub fn decode(&self, index: usize) -> (u64, u8) {
                let mut id = 0_u64;
                let mut depth = 0_i64;
                // SAFETY: `raw` is live, index is in range by contract,
                // out-pointers are valid stack slots for the duration.
                unsafe {
                    $decode(self.raw, index as i64, &mut id, &mut depth);
                }
                (id, depth as u8)
            }

            pub fn is_leaf(&self, id: u64, depth: u8) -> bool {
                // SAFETY: `raw` is a live tree.
                unsafe { $is_leaf(self.raw, id, depth as i64) != 0 }
            }

            pub fn heap_bytes(&self) -> usize {
                // SAFETY: `raw` is a live tree.
                unsafe { $heap(self.raw) as usize }
            }

            /// All leaves in left-to-right decode order.
            pub fn leaves(&self) -> Vec<(u64, u8)> {
                (0..self.node_count()).map(|i| self.decode(i)).collect()
            }
        }
    };
}

impl_tree!(
    LibcbtTree,
    "Owned handle to a serial `libcbt` tree. Deliberately `!Send + !Sync`.",
    thessa_cbt_create,
    thessa_cbt_release,
    thessa_cbt_reset_to_depth,
    thessa_cbt_split,
    thessa_cbt_merge_children,
    thessa_cbt_reduce,
    thessa_cbt_reduce_only,
    thessa_cbt_apply_batch,
    thessa_cbt_node_count,
    thessa_cbt_decode,
    thessa_cbt_is_leaf,
    thessa_cbt_heap_bytes,
    thessa_cbt_max_depth
);

impl_tree!(
    MtCbtTree,
    "Owned handle to the OpenMP build. Scaling experiments only.",
    thessa_mt_cbt_create,
    thessa_mt_cbt_release,
    thessa_mt_cbt_reset_to_depth,
    thessa_mt_cbt_split,
    thessa_mt_cbt_merge_children,
    thessa_mt_cbt_reduce,
    thessa_mt_cbt_reduce_only,
    thessa_mt_cbt_apply_batch,
    thessa_mt_cbt_node_count,
    thessa_mt_cbt_decode,
    thessa_mt_cbt_is_leaf,
    thessa_mt_cbt_heap_bytes,
    thessa_mt_cbt_max_depth
);

/// Cap the OpenMP worker count for a scaling run. Without OpenMP support in
/// the toolchain this is a no-op and scaling reads flat.
pub fn set_mt_threads(n: u32) {
    // SAFETY: plain integer argument, no retained state.
    unsafe {
        thessa_mt_cbt_set_threads(n as i32);
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
    fn batch_and_reduce_only_match_single_op_path() {
        let mut a = LibcbtTree::new(10).unwrap();
        a.split(1, 0);
        a.split(2, 1);
        a.reduce();
        let mut b = LibcbtTree::new(10).unwrap();
        let n = b.apply_batch_ops(&[(1, 0, 0), (2, 1, 0)]);
        assert_eq!(n, 2);
        b.reduce_only();
        assert_eq!(a.leaves(), b.leaves());
        // Merge path through the batch entry point as well.
        let n = b.apply_batch_ops(&[(2, 1, 1)]);
        assert_eq!(n, 1);
        b.reduce_only();
        a.merge_children(2, 1);
        a.reduce();
        assert_eq!(a.leaves(), b.leaves());
    }

    #[test]
    fn out_of_range_depth_is_rejected() {
        assert!(LibcbtTree::new(4).is_err());
        assert!(LibcbtTree::new(25).is_err());
    }

    #[test]
    fn mt_build_matches_serial_topology() {
        set_mt_threads(2);
        let mut tree = MtCbtTree::new(10).unwrap();
        tree.split(1, 0);
        tree.reduce();
        assert_eq!(tree.node_count(), 2);
        assert_eq!(tree.leaves(), vec![(2, 1), (3, 1)]);
        tree.merge_children(1, 0);
        tree.reduce();
        assert_eq!(tree.node_count(), 1);
        set_mt_threads(1);
    }
}
