//! Portable mirror of the compact OCBT memory-pool representation.
//!
//! `large_cbt` stores a dense allocation bitfield and a packed upper tree of
//! rank counts.  The upstream implementation rebuilds the packed counts with
//! `reduce()`.  This mirror keeps the same two-buffer shape, but commits a
//! changed bit by walking only its packed ancestor path.  That is the bridge
//! between the small upstream GPU representation and Thessa's no-full-tree
//! recomputation invariant.
//!
//! The representation is intentionally independent of `wgpu`: callers can
//! upload [`OcbtPoolMirror::tree_bytes`] and [`OcbtPoolMirror::bitfield_bytes`]
//! to storage buffers, or use the same layout in another backend.

const FIRST_PACKED_WIDTH32_LEVELS: u8 = 7;
const MIN_DEPTH: u8 = 17;
const MAX_DEPTH: u8 = 24;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OcbtError {
    DepthOutOfRange(u8),
    BitOutOfRange(usize),
    OrdinalOutOfRange(usize),
}

impl std::fmt::Display for OcbtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DepthOutOfRange(depth) => {
                write!(f, "OCBT depth {depth} outside {MIN_DEPTH}..={MAX_DEPTH}")
            }
            Self::BitOutOfRange(bit) => write!(f, "OCBT bit {bit} is outside the bitfield"),
            Self::OrdinalOutOfRange(ordinal) => {
                write!(f, "OCBT one-bit ordinal {ordinal} is outside the set bits")
            }
        }
    }
}

impl std::error::Error for OcbtError {}

/// The words touched by one incremental allocation/free operation.
///
/// `tree_word_indices` is suitable for a sparse upload or a GPU dirty-list.
/// It contains only the packed ancestor path; no level-wide reduction is
/// implied by this type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirtyPath {
    pub bit: usize,
    pub state: bool,
    pub changed: bool,
    pub tree_word_indices: Vec<usize>,
    pub bitfield_word_index: Option<usize>,
}

/// Compact dense-bitfield plus packed-rank mirror of an OCBT.
#[derive(Debug, Clone)]
pub struct OcbtPoolMirror {
    max_depth: u8,
    first_virtual_level: u8,
    tree_words: Vec<u32>,
    bitfield_words: Vec<u64>,
}

impl OcbtPoolMirror {
    /// Construct an OCBT layout compatible with the 128K..16M family.
    ///
    /// The public upstream variants currently use depths 17..=20.  Depths up
    /// to 24 are accepted here so the portable mirror can cover a larger
    /// terrain allocation pool without changing its buffer contract.
    pub fn new(max_depth: u8) -> Result<Self, OcbtError> {
        if !(MIN_DEPTH..=MAX_DEPTH).contains(&max_depth) {
            return Err(OcbtError::DepthOutOfRange(max_depth));
        }
        let tree_bits = Self::tree_bits_for_depth(max_depth);
        let mut mirror = Self {
            max_depth,
            first_virtual_level: max_depth - 6,
            tree_words: vec![0; tree_bits.div_ceil(32)],
            bitfield_words: vec![0; (1_usize << max_depth).div_ceil(64)],
        };
        mirror.clear();
        Ok(mirror)
    }

    pub const fn max_depth(&self) -> u8 {
        self.max_depth
    }

    pub const fn first_virtual_level(&self) -> u8 {
        self.first_virtual_level
    }

    pub const fn num_elements(&self) -> usize {
        1_usize << self.max_depth
    }

    pub fn tree_words(&self) -> &[u32] {
        &self.tree_words
    }

    pub fn bitfield_words(&self) -> &[u64] {
        &self.bitfield_words
    }

    pub fn tree_buffer_bytes(&self) -> usize {
        self.tree_words.len() * std::mem::size_of::<u32>()
    }

    pub fn bitfield_buffer_bytes(&self) -> usize {
        self.bitfield_words.len() * std::mem::size_of::<u64>()
    }

    pub fn memory_footprint(&self) -> usize {
        self.tree_buffer_bytes() + self.bitfield_buffer_bytes()
    }

    /// Stable little-endian upload payload for the packed tree buffer.
    pub fn tree_bytes(&self) -> Vec<u8> {
        self.tree_words
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }

    /// Stable little-endian upload payload for the dense bitfield buffer.
    pub fn bitfield_bytes(&self) -> Vec<u8> {
        self.bitfield_words
            .iter()
            .flat_map(|word| word.to_le_bytes())
            .collect()
    }

    /// Clear the pool and packed counts in O(buffer size), for initialization
    /// or a deliberate generation reset. Hot-path mutations use `set_bit`.
    pub fn clear(&mut self) {
        self.tree_words.fill(0);
        self.bitfield_words.fill(0);
    }

    pub fn get_bit(&self, bit: usize) -> Result<bool, OcbtError> {
        self.check_bit(bit)?;
        Ok((self.bitfield_words[bit / 64] >> (bit % 64)) & 1 != 0)
    }

    /// Set one allocation bit and update only the packed ancestors covering
    /// that bit. A repeated write of the same state is a no-op.
    pub fn set_bit(&mut self, bit: usize, state: bool) -> Result<DirtyPath, OcbtError> {
        self.check_bit(bit)?;
        let word_index = bit / 64;
        let mask = 1_u64 << (bit % 64);
        let old_state = self.bitfield_words[word_index] & mask != 0;
        let mut dirty_words = Vec::new();
        if old_state != state {
            if state {
                self.bitfield_words[word_index] |= mask;
            } else {
                self.bitfield_words[word_index] &= !mask;
            }
            let delta = if state { 1 } else { -1 };
            let mut node = self.packed_leaf_parent(bit);
            loop {
                let (word, shift, width) = self.packed_location(node);
                let value = self.packed_value_at(word, shift, width) as i32 + delta;
                debug_assert!(value >= 0);
                self.set_packed_value(word, shift, width, value as u32);
                dirty_words.push(word);
                if node == 1 {
                    break;
                }
                node /= 2;
            }
        }
        Ok(DirtyPath {
            bit,
            state,
            changed: old_state != state,
            tree_word_indices: dirty_words,
            bitfield_word_index: (old_state != state).then_some(word_index),
        })
    }

    /// Apply a batch without reducing untouched levels. The returned paths
    /// are intentionally explicit so a renderer can coalesce their word
    /// ranges before an upload.
    pub fn apply_bits(&mut self, updates: &[(usize, bool)]) -> Result<Vec<DirtyPath>, OcbtError> {
        updates
            .iter()
            .map(|&(bit, state)| self.set_bit(bit, state))
            .collect()
    }

    /// Number of set allocation bits, held in the packed root count.
    pub fn bit_count(&self) -> usize {
        self.packed_value(1) as usize
    }

    /// Return a packed rank count for a heap node. Virtual levels are read
    /// directly from the dense bitfield, matching the upstream contract.
    pub fn heap_element(&self, node: usize) -> usize {
        assert!(node > 0 && node < (1_usize << (self.max_depth + 1)));
        let depth = (usize::BITS - node.leading_zeros() - 1) as u8;
        if depth < self.first_virtual_level {
            self.packed_value(node) as usize
        } else {
            let span = 1_usize << (self.max_depth - depth);
            let first = (node - (1_usize << depth)) * span;
            self.range_popcount(first, span)
        }
    }

    /// Decode the zero-based ordinal of a set bit in left-to-right order.
    pub fn decode_bit(&self, ordinal: usize) -> Result<usize, OcbtError> {
        if ordinal >= self.bit_count() {
            return Err(OcbtError::OrdinalOutOfRange(ordinal));
        }
        let mut node = 1_usize;
        let mut rank = ordinal;
        for depth in 0..self.max_depth {
            let left = self.heap_element(node * 2);
            if rank < left {
                node *= 2;
            } else {
                rank -= left;
                node = node * 2 + 1;
            }
            if depth + 1 == self.max_depth {
                return Ok(node - (1_usize << self.max_depth));
            }
        }
        unreachable!("OCBT traversal always reaches a leaf")
    }

    /// Decode a zero bit, useful for a free-slot allocator using the same
    /// packed rank tree.
    pub fn decode_bit_complement(&self, ordinal: usize) -> Result<usize, OcbtError> {
        let zero_count = self.num_elements() - self.bit_count();
        if ordinal >= zero_count {
            return Err(OcbtError::OrdinalOutOfRange(ordinal));
        }
        let mut node = 1_usize;
        let mut rank = ordinal;
        for depth in 0..self.max_depth {
            let capacity = 1_usize << (self.max_depth - depth - 1);
            let left_zeroes = capacity - self.heap_element(node * 2);
            if rank < left_zeroes {
                node *= 2;
            } else {
                rank -= left_zeroes;
                node = node * 2 + 1;
            }
            if depth + 1 == self.max_depth {
                return Ok(node - (1_usize << self.max_depth));
            }
        }
        unreachable!("OCBT traversal always reaches a leaf")
    }

    fn check_bit(&self, bit: usize) -> Result<(), OcbtError> {
        if bit < self.num_elements() {
            Ok(())
        } else {
            Err(OcbtError::BitOutOfRange(bit))
        }
    }

    fn packed_leaf_parent(&self, bit: usize) -> usize {
        // The deepest packed level is max_depth - 7. Its nodes cover 128
        // leaves, exactly the granularity before the virtual bitfield levels.
        (1_usize << (self.first_virtual_level - 1))
            + (bit >> (self.max_depth - self.first_virtual_level + 1))
    }

    fn packed_location(&self, node: usize) -> (usize, u32, u8) {
        let depth = (usize::BITS - node.leading_zeros() - 1) as u8;
        debug_assert!(depth < self.first_virtual_level);
        let width = Self::packed_width(self.max_depth, depth);
        let level_first = 1_usize << depth;
        let level_offset_bits = Self::level_offset_bits(self.max_depth, depth);
        let bit_offset = level_offset_bits + (node - level_first) * width as usize;
        (bit_offset / 32, (bit_offset % 32) as u32, width)
    }

    fn packed_value(&self, node: usize) -> u32 {
        let (word, shift, width) = self.packed_location(node);
        self.packed_value_at(word, shift, width)
    }

    fn packed_value_at(&self, word: usize, shift: u32, width: u8) -> u32 {
        let mask = if width == 32 {
            u32::MAX
        } else {
            (1_u32 << width) - 1
        };
        (self.tree_words[word] >> shift) & mask
    }

    fn set_packed_value(&mut self, word: usize, shift: u32, width: u8, value: u32) {
        let mask = if width == 32 {
            u32::MAX
        } else {
            (1_u32 << width) - 1
        };
        self.tree_words[word] =
            (self.tree_words[word] & !(mask << shift)) | ((value & mask) << shift);
    }

    fn range_popcount(&self, first: usize, len: usize) -> usize {
        let mut cursor = first;
        let end = first + len;
        let mut count = 0;
        while cursor < end {
            let word_index = cursor / 64;
            let offset = cursor % 64;
            let width = (end - cursor).min(64 - offset);
            let mask = if width == 64 {
                u64::MAX
            } else {
                ((1_u64 << width) - 1) << offset
            };
            count += (self.bitfield_words[word_index] & mask).count_ones() as usize;
            cursor += width;
        }
        count
    }

    fn packed_width(max_depth: u8, depth: u8) -> u8 {
        if depth < FIRST_PACKED_WIDTH32_LEVELS {
            32
        } else if depth < max_depth - 7 {
            16
        } else {
            8
        }
    }

    fn level_offset_bits(max_depth: u8, depth: u8) -> usize {
        (0..depth)
            .map(|level| (1_usize << level) * Self::packed_width(max_depth, level) as usize)
            .sum()
    }

    fn tree_bits_for_depth(max_depth: u8) -> usize {
        (0..(max_depth - 6))
            .map(|depth| (1_usize << depth) * Self::packed_width(max_depth, depth) as usize)
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_rcbt_large_ffi::{LargeOcbt, Variant};

    #[test]
    fn matches_large_cbt_buffer_sizes() {
        let expected = [
            (17, 3_324, 16_384, 19_708),
            (18, 6_396, 32_768, 39_164),
            (19, 12_540, 65_536, 78_076),
            (20, 24_828, 131_072, 155_900),
        ];
        for (depth, tree, bitfield, total) in expected {
            let mirror = OcbtPoolMirror::new(depth).unwrap();
            assert_eq!(mirror.tree_buffer_bytes(), tree);
            assert_eq!(mirror.bitfield_buffer_bytes(), bitfield);
            assert_eq!(mirror.memory_footprint(), total);
        }
    }

    #[test]
    fn incremental_path_updates_counts_and_reports_only_dirty_words() {
        let mut mirror = OcbtPoolMirror::new(17).unwrap();
        let path = mirror.set_bit(65_000, true).unwrap();
        assert_eq!(mirror.bit_count(), 1);
        assert_eq!(mirror.decode_bit(0).unwrap(), 65_000);
        assert_eq!(
            path.tree_word_indices.len(),
            mirror.first_virtual_level() as usize
        );
        assert_eq!(path.bitfield_word_index, Some(65_000 / 64));
        let noop = mirror.set_bit(65_000, true).unwrap();
        assert!(!noop.changed);
        assert!(noop.tree_word_indices.is_empty());
        assert_eq!(noop.bitfield_word_index, None);
        let clear = mirror.set_bit(65_000, false).unwrap();
        assert_eq!(mirror.bit_count(), 0);
        assert!(clear.tree_word_indices.len() == mirror.first_virtual_level() as usize);
    }

    #[test]
    fn decode_set_and_clear_complements() {
        let mut mirror = OcbtPoolMirror::new(17).unwrap();
        for bit in [3, 17, 1_023, 65_000] {
            mirror.set_bit(bit, true).unwrap();
        }
        assert_eq!(
            (0..mirror.bit_count())
                .map(|ordinal| mirror.decode_bit(ordinal).unwrap())
                .collect::<Vec<_>>(),
            vec![3, 17, 1_023, 65_000]
        );
        assert_eq!(mirror.decode_bit_complement(0).unwrap(), 0);
        assert_eq!(mirror.decode_bit_complement(3).unwrap(), 4);
        assert!(!mirror.get_bit(4).unwrap());
    }

    #[test]
    fn incremental_counts_match_large_cbt_after_reference_reduce() {
        let mut mirror = OcbtPoolMirror::new(17).unwrap();
        let mut reference = LargeOcbt::new(Variant::Ocbt128k).unwrap();
        for bit in [3, 17, 1_023, 65_000, 100_000] {
            mirror.set_bit(bit, true).unwrap();
            reference.set_bit(bit, true);
        }
        reference.reduce();

        assert_eq!(mirror.tree_bytes(), reference.buffer(0));
        assert_eq!(mirror.bitfield_bytes(), reference.buffer(1));
        for node in [1, 2, 3, 4, 5, 1_024, 1_025, 1_535, 65_536, 65_537] {
            assert_eq!(
                mirror.heap_element(node),
                reference.heap_element(node),
                "rank mismatch at heap node {node}"
            );
        }
        assert_eq!(
            (0..mirror.bit_count())
                .map(|ordinal| mirror.decode_bit(ordinal).unwrap())
                .collect::<Vec<_>>(),
            (0..reference.bit_count())
                .map(|ordinal| reference.decode_bit(ordinal))
                .collect::<Vec<_>>()
        );
    }
}
