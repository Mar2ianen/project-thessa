//! Deterministic in-memory snapshot codec for [`super::SurfaceSculpt`].
//!
//! The sparse, bounded little-endian page snapshot is an independent format
//! informed by OpenEngine's MIT `SphereSculptSerialization.h/.cpp` (revision
//! bdf5c27). It does not reuse that engine's TSCP format or page records.

use std::fmt;

use super::{PAGE_BYTES, PAGE_TEXELS, SurfaceSculpt, SurfaceSculptError};
use crate::tile_anchor::TileKey;

const MAGIC: [u8; 4] = *b"TSCS";
const VERSION: u16 = 1;
const PAGE_CELLS: u16 = 32;
const HEADER_BYTES: usize = 48;
const RECORD_PREFIX_BYTES: usize = 12;
const CHECKSUM_BYTES: usize = 4;

/// Snapshot parsing errors. The caller-supplied page limit is always
/// authoritative; no capacity is trusted from serialized data.
#[derive(Clone, Debug, PartialEq)]
pub enum SurfaceSculptSnapshotError {
    Truncated,
    InvalidMagic,
    UnsupportedVersion,
    InvalidPageCellCount,
    NonZeroReserved,
    InvalidGeometry(SurfaceSculptError),
    InconsistentLevel,
    PageLimitExceeded,
    LengthOverflow,
    LengthMismatch,
    InvalidPageKey,
    DuplicateOrUnsortedPage,
    NonFiniteSample,
    ChecksumMismatch,
}

impl fmt::Display for SurfaceSculptSnapshotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated => f.write_str("sculpt snapshot is truncated"),
            Self::InvalidMagic => f.write_str("sculpt snapshot magic is invalid"),
            Self::UnsupportedVersion => f.write_str("sculpt snapshot version is unsupported"),
            Self::InvalidPageCellCount => f.write_str("sculpt snapshot page schema is unsupported"),
            Self::NonZeroReserved => f.write_str("sculpt snapshot reserved bytes must be zero"),
            Self::InvalidGeometry(error) => {
                write!(f, "sculpt snapshot geometry is invalid: {error}")
            }
            Self::InconsistentLevel => {
                f.write_str("sculpt snapshot level does not match its geometry")
            }
            Self::PageLimitExceeded => f.write_str("sculpt snapshot exceeds the caller page limit"),
            Self::LengthOverflow => {
                f.write_str("sculpt snapshot length overflows the platform size")
            }
            Self::LengthMismatch => {
                f.write_str("sculpt snapshot length does not match its page count")
            }
            Self::InvalidPageKey => f.write_str("sculpt snapshot contains an invalid page key"),
            Self::DuplicateOrUnsortedPage => {
                f.write_str("sculpt snapshot page keys must be strictly sorted and unique")
            }
            Self::NonFiniteSample => f.write_str("sculpt snapshot contains a non-finite sample"),
            Self::ChecksumMismatch => f.write_str("sculpt snapshot checksum does not match"),
        }
    }
}

impl std::error::Error for SurfaceSculptSnapshotError {}

impl SurfaceSculpt {
    /// Return the maximum v1 byte length for `max_pages`, including the header
    /// and checksum. Callers can use this to bound file reads before allocating.
    pub fn snapshot_size_limit(max_pages: usize) -> Option<usize> {
        HEADER_BYTES
            .checked_add(max_pages.checked_mul(RECORD_PREFIX_BYTES.checked_add(PAGE_BYTES)?)?)?
            .checked_add(CHECKSUM_BYTES)
    }

    /// Serialize the exact sparse page state into deterministic little-endian
    /// bytes. The CRC-32 covers the complete header and page payload.
    pub fn to_bytes(&self) -> Vec<u8> {
        let record_bytes = RECORD_PREFIX_BYTES + PAGE_BYTES;
        let mut out = Vec::with_capacity(
            HEADER_BYTES
                .saturating_add(self.pages.len().saturating_mul(record_bytes))
                .saturating_add(CHECKSUM_BYTES),
        );
        out.extend_from_slice(&MAGIC);
        put_u16(&mut out, VERSION);
        put_u16(&mut out, PAGE_CELLS);
        put_f64(&mut out, self.radius_m);
        put_f64(&mut out, self.target_texel_m);
        put_u64(&mut out, self.revision);
        out.push(self.level);
        out.extend_from_slice(&[0; 7]);
        put_u64(&mut out, self.pages.len() as u64);
        for (&(face, level, x, y), samples) in &self.pages {
            out.push(face);
            out.push(level);
            put_u16(&mut out, 0);
            put_u32(&mut out, x);
            put_u32(&mut out, y);
            for &sample in samples {
                put_f64(&mut out, sample);
            }
        }
        let checksum = crc32(&out);
        put_u32(&mut out, checksum);
        out
    }

    /// Parse a snapshot under a caller-owned physical page budget. The input
    /// must be canonical, fully checksummed, and contain exactly its declared
    /// records; file IO and planet identity are intentionally caller concerns.
    pub fn from_bytes(bytes: &[u8], max_pages: usize) -> Result<Self, SurfaceSculptSnapshotError> {
        if bytes.len() < HEADER_BYTES + CHECKSUM_BYTES {
            return Err(SurfaceSculptSnapshotError::Truncated);
        }
        if bytes[..4] != MAGIC {
            return Err(SurfaceSculptSnapshotError::InvalidMagic);
        }
        if read_u16(bytes, 4) != VERSION {
            return Err(SurfaceSculptSnapshotError::UnsupportedVersion);
        }
        if read_u16(bytes, 6) != PAGE_CELLS {
            return Err(SurfaceSculptSnapshotError::InvalidPageCellCount);
        }
        if bytes[33..40].iter().any(|byte| *byte != 0) {
            return Err(SurfaceSculptSnapshotError::NonZeroReserved);
        }

        let radius_m = read_f64(bytes, 8);
        let target_texel_m = read_f64(bytes, 16);
        let revision = read_u64(bytes, 24);
        let level = bytes[32];
        let page_count_u64 = read_u64(bytes, 40);
        let page_count = usize::try_from(page_count_u64)
            .map_err(|_| SurfaceSculptSnapshotError::PageLimitExceeded)?;
        if page_count > max_pages {
            return Err(SurfaceSculptSnapshotError::PageLimitExceeded);
        }

        let expected_level = SurfaceSculpt::new(radius_m, target_texel_m, max_pages)
            .map_err(SurfaceSculptSnapshotError::InvalidGeometry)?;
        let pages_per_axis = 1_u128 << expected_level.level;
        let grid_page_limit = 6_u128 * pages_per_axis * pages_per_axis;
        if page_count_u64 as u128 > grid_page_limit {
            return Err(SurfaceSculptSnapshotError::PageLimitExceeded);
        }
        if level != expected_level.level {
            return Err(SurfaceSculptSnapshotError::InconsistentLevel);
        }
        let record_bytes = RECORD_PREFIX_BYTES
            .checked_add(PAGE_BYTES)
            .ok_or(SurfaceSculptSnapshotError::LengthOverflow)?;
        let payload_bytes = page_count
            .checked_mul(record_bytes)
            .ok_or(SurfaceSculptSnapshotError::LengthOverflow)?;
        let expected_len = HEADER_BYTES
            .checked_add(payload_bytes)
            .and_then(|len| len.checked_add(CHECKSUM_BYTES))
            .ok_or(SurfaceSculptSnapshotError::LengthOverflow)?;
        if bytes.len() != expected_len {
            return Err(SurfaceSculptSnapshotError::LengthMismatch);
        }
        let checksum_offset = bytes.len() - CHECKSUM_BYTES;
        if read_u32(bytes, checksum_offset) != crc32(&bytes[..checksum_offset]) {
            return Err(SurfaceSculptSnapshotError::ChecksumMismatch);
        }

        let mut layer = expected_level;
        layer.revision = revision;
        let mut previous = None;
        let mut offset = HEADER_BYTES;
        // Exact length and caller capacity have both been verified before the
        // first page allocation; each record is fixed-size in version 1.
        for _ in 0..page_count {
            let face = bytes[offset];
            let page_level = bytes[offset + 1];
            if read_u16(bytes, offset + 2) != 0 {
                return Err(SurfaceSculptSnapshotError::NonZeroReserved);
            }
            let x = read_u32(bytes, offset + 4);
            let y = read_u32(bytes, offset + 8);
            let tile = TileKey::new(face, page_level, x, y)
                .filter(|tile| tile.level == layer.level)
                .ok_or(SurfaceSculptSnapshotError::InvalidPageKey)?;
            let key = (tile.face, tile.level, tile.x, tile.y);
            if previous.is_some_and(|last| last >= key) {
                return Err(SurfaceSculptSnapshotError::DuplicateOrUnsortedPage);
            }
            previous = Some(key);
            offset += RECORD_PREFIX_BYTES;
            let mut samples = Vec::with_capacity(PAGE_TEXELS);
            for _ in 0..PAGE_TEXELS {
                let sample = read_f64(bytes, offset);
                if !sample.is_finite() {
                    return Err(SurfaceSculptSnapshotError::NonFiniteSample);
                }
                samples.push(sample);
                offset += std::mem::size_of::<f64>();
            }
            layer.pages.insert(key, samples);
        }
        Ok(layer)
    }
}

fn put_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn put_f64(out: &mut Vec<u8>, value: f64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn read_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn read_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(
        bytes[offset..offset + 4]
            .try_into()
            .expect("fixed-width header field"),
    )
}

fn read_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed-width header field"),
    )
}

fn read_f64(bytes: &[u8], offset: usize) -> f64 {
    f64::from_le_bytes(
        bytes[offset..offset + 8]
            .try_into()
            .expect("fixed-width header field"),
    )
}

// IEEE CRC-32; a small local implementation avoids a dependency for this codec.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & 0_u32.wrapping_sub(crc & 1));
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layer_with_pages() -> SurfaceSculpt {
        let mut layer = SurfaceSculpt::new(2_000.0, 10.0, 64).unwrap();
        layer
            .apply_dab([0.0, 0.0, 1.0], 20.0, 1.25)
            .expect("valid dab");
        layer
    }

    fn refresh_checksum(bytes: &mut [u8]) {
        let checksum_offset = bytes.len() - CHECKSUM_BYTES;
        let checksum = crc32(&bytes[..checksum_offset]);
        bytes[checksum_offset..].copy_from_slice(&checksum.to_le_bytes());
    }

    #[test]
    fn size_limit_is_checked_and_crc_matches_the_standard_vector() {
        assert_eq!(
            SurfaceSculpt::snapshot_size_limit(0),
            Some(HEADER_BYTES + CHECKSUM_BYTES)
        );
        assert_eq!(
            SurfaceSculpt::snapshot_size_limit(2),
            Some(HEADER_BYTES + 2 * (RECORD_PREFIX_BYTES + PAGE_BYTES) + CHECKSUM_BYTES)
        );
        assert_eq!(SurfaceSculpt::snapshot_size_limit(usize::MAX), None);
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn empty_and_edited_snapshots_round_trip_exactly_and_remain_editable() {
        let empty = SurfaceSculpt::new(2_000.0, 10.0, 64).unwrap();
        let empty_bytes = empty.to_bytes();
        let empty_copy = SurfaceSculpt::from_bytes(&empty_bytes, 64).unwrap();
        assert_eq!(empty_bytes, empty_copy.to_bytes());
        assert_eq!(empty_copy.allocated_pages(), 0);
        assert_eq!(empty_copy.revision(), 0);

        let layer = layer_with_pages();
        let bytes = layer.to_bytes();
        let mut restored = SurfaceSculpt::from_bytes(&bytes, 64).unwrap();
        assert_eq!(bytes, restored.to_bytes());
        assert_eq!(layer.revision(), restored.revision());
        assert_eq!(layer.allocated_pages(), restored.allocated_pages());
        assert_eq!(
            layer.sample([0.0, 0.0, 1.0]),
            restored.sample([0.0, 0.0, 1.0])
        );
        let a = layer.sample([0.000_1, 0.0, 1.0]).expect("valid direction");
        let b = restored
            .sample([0.000_1, 0.0, 1.0])
            .expect("valid direction");
        assert_eq!(a, b);
        layer_comparable_followup(&layer, &mut restored);
    }

    fn layer_comparable_followup(original: &SurfaceSculpt, restored: &mut SurfaceSculpt) {
        let mut original = SurfaceSculpt::from_bytes(&original.to_bytes(), 64).unwrap();
        let direction = [0.0, 0.0, 1.0];
        let a = original.apply_dab(direction, 20.0, -0.5).unwrap();
        let b = restored.apply_dab(direction, 20.0, -0.5).unwrap();
        assert_eq!(a, b);
        assert_eq!(original.to_bytes(), restored.to_bytes());
    }

    #[test]
    fn rejects_truncation_and_checksum_corruption() {
        let bytes = layer_with_pages().to_bytes();
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes[..bytes.len() - 1], 64),
            Err(SurfaceSculptSnapshotError::LengthMismatch)
        ));
        let mut trailing = bytes.clone();
        trailing.push(0);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&trailing, 64),
            Err(SurfaceSculptSnapshotError::LengthMismatch)
        ));
        let mut corrupt = bytes;
        corrupt[HEADER_BYTES + RECORD_PREFIX_BYTES + 9] ^= 0x40;
        assert!(matches!(
            SurfaceSculpt::from_bytes(&corrupt, 64),
            Err(SurfaceSculptSnapshotError::ChecksumMismatch)
        ));
    }

    #[test]
    fn rejects_page_budget_and_malicious_count_before_allocating() {
        let bytes = layer_with_pages().to_bytes();
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes, 0),
            Err(SurfaceSculptSnapshotError::PageLimitExceeded)
        ));
        let mut malicious = bytes;
        malicious[40..48].copy_from_slice(&u64::MAX.to_le_bytes());
        refresh_checksum(&mut malicious);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&malicious, usize::MAX),
            Err(SurfaceSculptSnapshotError::PageLimitExceeded)
        ));
    }

    #[test]
    fn rejects_nonfinite_samples() {
        let mut bytes = layer_with_pages().to_bytes();
        bytes[HEADER_BYTES + RECORD_PREFIX_BYTES..HEADER_BYTES + RECORD_PREFIX_BYTES + 8]
            .copy_from_slice(&f64::NAN.to_le_bytes());
        refresh_checksum(&mut bytes);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes, 64),
            Err(SurfaceSculptSnapshotError::NonFiniteSample)
        ));
    }

    #[test]
    fn rejects_duplicate_and_invalid_level_records() {
        let mut header = layer_with_pages().to_bytes();
        header[32] ^= 1;
        refresh_checksum(&mut header);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&header, 64),
            Err(SurfaceSculptSnapshotError::InconsistentLevel)
        ));
        let mut layer = layer_with_pages();
        let page = vec![0.0; PAGE_TEXELS];
        let key = (4, layer.level, 0, 0);
        layer.pages.insert(key, page);
        let mut bytes = layer.to_bytes();
        assert!(read_u64(&bytes, 40) >= 2);
        let second = HEADER_BYTES + RECORD_PREFIX_BYTES + PAGE_BYTES;
        let first_prefix = bytes[HEADER_BYTES..HEADER_BYTES + 12].to_vec();
        bytes[second..second + 12].copy_from_slice(&first_prefix);
        refresh_checksum(&mut bytes);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes, 64),
            Err(SurfaceSculptSnapshotError::DuplicateOrUnsortedPage)
        ));

        let mut bytes = layer.to_bytes();
        bytes[HEADER_BYTES + 1] = layer.level.saturating_add(1);
        refresh_checksum(&mut bytes);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes, 64),
            Err(SurfaceSculptSnapshotError::InvalidPageKey)
        ));

        let mut bytes = layer.to_bytes();
        bytes[HEADER_BYTES + 4..HEADER_BYTES + 8]
            .copy_from_slice(&(1_u32 << layer.level).to_le_bytes());
        refresh_checksum(&mut bytes);
        assert!(matches!(
            SurfaceSculpt::from_bytes(&bytes, 64),
            Err(SurfaceSculptSnapshotError::InvalidPageKey)
        ));
    }
}
