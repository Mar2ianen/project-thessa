//! Safe binding to the CPU OCBT layout shipped with `large_cbt`.
//!
//! This is a reference/performance backend for the GPU-oriented representation:
//! a packed upper tree plus a dense leaf bitfield. It is deliberately separate
//! from `thessa-rcbt-ffi` because its observable unit is a bitfield handle, not
//! the sparse `Tree` split/merge API. The Rust runtime remains responsible for
//! sparse dirty-path commits; this crate measures the upstream OCBT reduction
//! cost against that path.

use std::ffi::c_void;

unsafe extern "C" {
    fn thessa_large_cbt_create(variant: u32) -> *mut c_void;
    fn thessa_large_cbt_destroy(handle: *mut c_void);
    fn thessa_large_cbt_num_elements(handle: *const c_void) -> u32;
    fn thessa_large_cbt_max_depth(handle: *const c_void) -> u32;
    fn thessa_large_cbt_last_level_size(handle: *const c_void) -> u32;
    fn thessa_large_cbt_memory_footprint(handle: *const c_void) -> u32;
    fn thessa_large_cbt_buffer_size(handle: *const c_void, index: u32) -> u32;
    fn thessa_large_cbt_element_size(handle: *const c_void, index: u32) -> u32;
    fn thessa_large_cbt_buffer(handle: *const c_void, index: u32) -> *const u8;
    fn thessa_large_cbt_set_bit(handle: *mut c_void, bit: u32, state: u32);
    fn thessa_large_cbt_get_bit(handle: *const c_void, bit: u32) -> u32;
    fn thessa_large_cbt_bit_count(handle: *const c_void) -> u32;
    fn thessa_large_cbt_decode_bit(handle: *const c_void, ordinal: u32) -> u32;
    fn thessa_large_cbt_decode_bit_complement(handle: *const c_void, ordinal: u32) -> u32;
    fn thessa_large_cbt_heap_element(handle: *const c_void, id: u32) -> u32;
    fn thessa_large_cbt_reduce(handle: *mut c_void);
    fn thessa_large_cbt_clear(handle: *mut c_void);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Variant {
    Ocbt128k = 0,
    Ocbt256k = 1,
    Ocbt512k = 2,
    Ocbt1m = 3,
}

impl Variant {
    pub const fn num_elements(self) -> usize {
        match self {
            Self::Ocbt128k => 131_072,
            Self::Ocbt256k => 262_144,
            Self::Ocbt512k => 524_288,
            Self::Ocbt1m => 1_048_576,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    NullHandle,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NullHandle => f.write_str("large_cbt returned a null handle"),
        }
    }
}

impl std::error::Error for Error {}

/// Single-threaded handle to one of the upstream OCBT capacities.
pub struct LargeOcbt {
    raw: *mut c_void,
    variant: Variant,
}

impl Drop for LargeOcbt {
    fn drop(&mut self) {
        // SAFETY: raw was returned by the matching constructor and is owned.
        unsafe { thessa_large_cbt_destroy(self.raw) }
    }
}

impl LargeOcbt {
    pub fn new(variant: Variant) -> Result<Self, Error> {
        // SAFETY: variant is a closed Rust enum represented as u32.
        let raw = unsafe { thessa_large_cbt_create(variant as u32) };
        if raw.is_null() {
            return Err(Error::NullHandle);
        }
        Ok(Self { raw, variant })
    }

    pub const fn variant(&self) -> Variant {
        self.variant
    }

    pub fn num_elements(&self) -> usize {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_num_elements(self.raw) as usize }
    }

    pub fn max_depth(&self) -> u8 {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_max_depth(self.raw) as u8 }
    }

    pub fn last_level_size(&self) -> usize {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_last_level_size(self.raw) as usize }
    }

    pub fn memory_footprint(&self) -> usize {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_memory_footprint(self.raw) as usize }
    }

    pub fn buffer_size(&self, index: usize) -> usize {
        // SAFETY: the index is range-checked against the two OCBT buffers
        // before the FFI call; upstream does not bounds-check it.
        assert!(index < 2, "large CBT exposes two packed buffers");
        unsafe { thessa_large_cbt_buffer_size(self.raw, index as u32) as usize }
    }

    pub fn element_size(&self, index: usize) -> usize {
        // SAFETY: see buffer_size.
        assert!(index < 2, "large CBT exposes two packed buffers");
        unsafe { thessa_large_cbt_element_size(self.raw, index as u32) as usize }
    }

    /// Copy one of the two packed buffers for upload or inspection.
    pub fn buffer(&self, index: usize) -> Vec<u8> {
        assert!(index < 2, "large CBT exposes two packed buffers");
        let len = self.buffer_size(index);
        // SAFETY: the pointer is valid for `len` bytes while self is alive.
        unsafe { std::slice::from_raw_parts(thessa_large_cbt_buffer(self.raw, index as u32), len) }
            .to_vec()
    }

    pub fn set_bit(&mut self, bit: usize, state: bool) {
        assert!(bit < self.num_elements());
        // SAFETY: bit is in the upstream bitfield range.
        unsafe { thessa_large_cbt_set_bit(self.raw, bit as u32, state as u32) }
    }

    pub fn get_bit(&self, bit: usize) -> bool {
        assert!(bit < self.num_elements());
        // SAFETY: bit is in the upstream bitfield range.
        unsafe { thessa_large_cbt_get_bit(self.raw, bit as u32) != 0 }
    }

    pub fn bit_count(&self) -> usize {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_bit_count(self.raw) as usize }
    }

    pub fn decode_bit(&self, ordinal: usize) -> usize {
        assert!(ordinal < self.bit_count());
        // SAFETY: ordinal is below the current one-bit count.
        unsafe { thessa_large_cbt_decode_bit(self.raw, ordinal as u32) as usize }
    }

    pub fn decode_bit_complement(&self, ordinal: usize) -> usize {
        assert!(ordinal < self.num_elements() - self.bit_count());
        // SAFETY: ordinal is below the current zero-bit count.
        unsafe { thessa_large_cbt_decode_bit_complement(self.raw, ordinal as u32) as usize }
    }

    pub fn heap_element(&self, id: usize) -> usize {
        // `get_heap_element` indexes per-depth offset/mask tables and the
        // packed heap/bitfield. Checking only that the id fits in `u32` would
        // let safe callers request depths beyond those allocated tables.
        assert!(
            id > 0 && id < (1usize << (self.max_depth() as usize + 1)),
            "large CBT node id is out of range"
        );
        // SAFETY: id is in the allocated heap rows, and therefore in u32 range.
        let id = u32::try_from(id).expect("large CBT node id exceeds u32 range");
        unsafe { thessa_large_cbt_heap_element(self.raw, id) as usize }
    }

    /// Rebuild upstream packed sums from the dense bitfield.
    pub fn reduce(&mut self) {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_reduce(self.raw) }
    }

    pub fn clear(&mut self) {
        // SAFETY: raw is a live handle.
        unsafe { thessa_large_cbt_clear(self.raw) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_upstream_capacities_construct_and_report_layout() {
        for variant in [
            Variant::Ocbt128k,
            Variant::Ocbt256k,
            Variant::Ocbt512k,
            Variant::Ocbt1m,
        ] {
            let tree = LargeOcbt::new(variant).unwrap();
            assert_eq!(tree.num_elements(), variant.num_elements());
            assert_eq!(tree.max_depth(), variant.num_elements().ilog2() as u8);
            assert_eq!(
                tree.buffer_size(0) + tree.buffer_size(1),
                tree.memory_footprint()
            );
            assert_eq!(tree.buffer(0).len(), tree.buffer_size(0));
            assert_eq!(tree.buffer(1).len(), tree.buffer_size(1));
        }
    }

    #[test]
    fn sparse_bitfield_reduces_and_decodes_without_rebuilding_the_bitfield() {
        let mut tree = LargeOcbt::new(Variant::Ocbt128k).unwrap();
        for bit in [3, 17, 1023, 65_000] {
            tree.set_bit(bit, true);
        }
        tree.reduce();
        assert_eq!(tree.bit_count(), 4);
        assert_eq!(
            (0..tree.bit_count())
                .map(|ordinal| tree.decode_bit(ordinal))
                .collect::<Vec<_>>(),
            vec![3, 17, 1023, 65_000]
        );
        assert!(!tree.get_bit(4));
        assert_eq!(tree.decode_bit_complement(0), 0);
    }

    #[test]
    #[should_panic(expected = "large CBT node id is out of range")]
    fn heap_element_rejects_nodes_beyond_allocated_depth() {
        let tree = LargeOcbt::new(Variant::Ocbt128k).unwrap();
        tree.heap_element(1usize << (tree.max_depth() as usize + 1));
    }

    #[test]
    fn vendored_sources_match_recorded_fingerprint() {
        // Tamper-evidence for third_party/large_cbt (see large_cbt.REVISION):
        // the vendored tree is unlicensed upstream code under quarantine as
        // a dev-time benchmark oracle. Any content drift must be a deliberate
        // update of both the tree and the recorded hashes.
        use sha2::{Digest, Sha256};
        let root =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../third_party/large_cbt");
        let expected: &[(&str, &str)] = &[
            (
                "cbt.h",
                "f1371328957f13f31910b8efe28114aea4e25f01e8c0831b6c3d94f2be4813ba",
            ),
            (
                "cbt_utility.h",
                "5157346bb10e695dc5a4fa88191eabf13759e53e711a07e4daeec43f356e2b36",
            ),
            (
                "ocbt_128k.h",
                "2c8d1a44e6356ece7d3d3eb2754e92c6c1ec7a41aeb99922cc0950b06e9222e1",
            ),
            (
                "ocbt_128k.cpp",
                "f2b7cf0aa1fe66b0d737df6c4b365b4c34e4345148dfdfefe4940603aa421c2e",
            ),
            (
                "ocbt_256k.h",
                "1ab89ab746db5194cdd2e961a61b22390cecebacb137c89183cd8de7cb2697db",
            ),
            (
                "ocbt_256k.cpp",
                "9bf2b3e4aa6592543013356e9adc6d1d7ce5698717e72a0910d7ee4b2aa2584d",
            ),
            (
                "ocbt_512k.h",
                "57e8f46001b0b500623cb9c83bf98d1922768e50be7ca6220edcbeed8f50cd3b",
            ),
            (
                "ocbt_512k.cpp",
                "ed3b2a24a9c6f72768c4875e54d4fc2abbecd50acaf5e76857f91619974c70d7",
            ),
            (
                "ocbt_1m.h",
                "280b2c5e0e7103870f6e4087812ef08707ae74788869f98dbbb029ed321bcd85",
            ),
            (
                "ocbt_1m.cpp",
                "d39a03780669c0e539d18cd8c07763e1deadf161ad31745f68c5fbac4e890422",
            ),
            (
                "ocbt_generic.h",
                "948e5167f3b2a3f25626dda6c26e2f3606defdf5689982a7716f854e815525a9",
            ),
            (
                "graphics/types.h",
                "ebdbc96b9d727fc04f715cf86940c58d4b2f4876f4d83a984da1a513a8d7fd9b",
            ),
            (
                "math/operators.h",
                "6cfd7efc04c4fb94f1fd69552a340d069f2ea5281678e97a17b8c7a50cbb23b5",
            ),
            (
                "tools/security.h",
                "f9e07421fadcf533ba813bc75d36e5add790400a5d88a5937c1dadb3a06e89b8",
            ),
        ];
        for (file, hash) in expected {
            let bytes = std::fs::read(root.join(file))
                .unwrap_or_else(|_| panic!("missing vendored file {file}"));
            let digest = format!("{:x}", Sha256::digest(&bytes));
            assert_eq!(
                &digest, hash,
                "drift in third_party/large_cbt/{file}: update the tree and large_cbt.REVISION deliberately"
            );
        }
    }
}
