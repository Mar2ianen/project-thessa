//! Multi-channel color pages (doc §18 Phase C, §5.1).
//!
//! Baseline representation per the doc: each channel is an independent
//! scalar page with its own local base color component plus per-channel
//! scale/range and packed residuals. No cross-channel transform is assumed;
//! decorrelated spaces are allowed only with a measured win (none claimed
//! here).
//!
//! The error metric for color is linear-light (§5.1), not raw byte
//! equality — see [`crate::metrics::measure_linear`].

use crate::codec::{
    ADAPTIVE_LADDER, BLOCK_TEXELS, CodecError, EncodeMode, EncodedBlock, EncodedPage, MicroCodec,
    ScalarField, block_texels, blocks_for_extent, decode_block, encode_block_for,
};
use crate::metrics::srgb_to_linear;

/// Selection metric for adaptive color encoding.
///
/// Scalar adaptive (`EncodeMode::Adaptive`) always budgets in code levels.
/// Color additionally offers linear-light budgets: a 2-level error near
/// white is a far bigger visual error than the same 2 levels near black,
/// so the honest color policy selects rungs in linear light.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ColorBudget {
    /// Per-texel absolute error in sRGB code levels (matches scalar).
    Levels(f64),
    /// Per-texel absolute error in linear light (`0..=1`).
    Linear(f64),
}

const COLOR_MAGIC: [u8; 4] = *b"MICC";
const COLOR_VERSION: u8 = 1;

/// One RGB color field as three independent scalar planes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ColorField {
    /// Texels per row (shared by all planes).
    pub width: u32,
    /// Rows (shared by all planes).
    pub height: u32,
    /// Red, green, blue planes.
    pub channels: [ScalarField; 3],
}

impl ColorField {
    /// Build from three planes; rejects extent mismatches.
    pub fn new(channels: [ScalarField; 3]) -> Result<Self, CodecError> {
        let (width, height) = (channels[0].width, channels[0].height);
        if width == 0 || height == 0 {
            return Err(CodecError::EmptyField);
        }
        for plane in &channels[1..] {
            if (plane.width, plane.height) != (width, height) {
                return Err(CodecError::BadExtent {
                    width,
                    height,
                    samples: plane.data.len(),
                });
            }
        }
        Ok(Self {
            width,
            height,
            channels,
        })
    }
}

/// One encoded color page: one scalar [`EncodedPage`] per channel.
/// The fixed three-element array (not a `Vec`) is the invariant: R, G, B
/// are always all present, so `decode` never needs `expect` calls.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ColorPage {
    /// Texels per row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// Encoded R, G, B planes.
    pub channels: [EncodedPage; 3],
}

impl ColorPage {
    /// Encode every channel with the same mode.
    pub fn encode(field: &ColorField, mode: EncodeMode) -> Self {
        let [r, g, b] = &field.channels;
        Self {
            width: field.width,
            height: field.height,
            channels: [
                EncodedPage::encode(r, mode),
                EncodedPage::encode(g, mode),
                EncodedPage::encode(b, mode),
            ],
        }
    }

    /// Encode all channels with one adaptive budget.
    ///
    /// `Levels` behaves exactly like [`EncodeMode::Adaptive`]. `Linear`
    /// selects rungs per block in linear light: each candidate block is
    /// decoded and both sides are linearized before comparing, so bright
    /// regions (where one code level spans more light) automatically get
    /// finer rungs than dark ones at the same nominal budget.
    pub fn encode_with_budget(field: &ColorField, budget: ColorBudget) -> Self {
        match budget {
            ColorBudget::Levels(max_abs_error) => {
                Self::encode(field, EncodeMode::Adaptive { max_abs_error })
            }
            ColorBudget::Linear(max_abs_error) => {
                let budget = max_abs_error.max(0.0);
                let mut channels = Vec::with_capacity(3);
                for plane in &field.channels {
                    channels.push(encode_plane_linear(plane, budget));
                }
                let channels: [EncodedPage; 3] = channels.try_into().expect("three channels");
                Self {
                    width: field.width,
                    height: field.height,
                    channels,
                }
            }
        }
    }

    /// Decode all channels.
    pub fn decode(&self) -> [ScalarField; 3] {
        let [r, g, b] = &self.channels;
        [r.decode(), g.decode(), b.decode()]
    }

    /// Exact wire size in bytes.
    pub fn encoded_bytes(&self) -> usize {
        Self::HEADER_LEN
            + self
                .channels
                .iter()
                .map(|c| 4 + c.encoded_bytes())
                .sum::<usize>()
    }

    /// Bytes per texel across all channels.
    pub fn bytes_per_texel(&self) -> f64 {
        self.encoded_bytes() as f64 / (self.width as f64 * self.height as f64)
    }

    const HEADER_LEN: usize = 4 + 1 + 4 + 4 + 1;

    /// Deterministic wire encoding: magic, version, extent, channel count,
    /// then per channel a u32 length plus the scalar page bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.encoded_bytes());
        out.extend_from_slice(&COLOR_MAGIC);
        out.push(COLOR_VERSION);
        out.extend_from_slice(&self.width.to_le_bytes());
        out.extend_from_slice(&self.height.to_le_bytes());
        out.push(self.channels.len() as u8);
        for channel in &self.channels {
            let bytes = channel.to_bytes();
            out.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
            out.extend_from_slice(&bytes);
        }
        out
    }

    /// Parse [`ColorPage::to_bytes`] output strictly. Only exactly three
    /// channels are accepted: anything else is a different format, not a
    /// color page.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CodecError> {
        let mut cursor = bytes;
        let take = |cursor: &mut &[u8], n: usize| -> Result<Vec<u8>, CodecError> {
            if cursor.len() < n {
                return Err(CodecError::Truncated);
            }
            let (head, tail) = cursor.split_at(n);
            *cursor = tail;
            Ok(head.to_vec())
        };
        if take(&mut cursor, 4)? != COLOR_MAGIC {
            return Err(CodecError::BadMagic);
        }
        if take(&mut cursor, 1)?[0] != COLOR_VERSION {
            return Err(CodecError::UnsupportedVersion(
                bytes.get(4).copied().unwrap_or(255),
            ));
        }
        let width = u32::from_le_bytes(take(&mut cursor, 4)?[..4].try_into().expect("4 bytes"));
        let height = u32::from_le_bytes(take(&mut cursor, 4)?[..4].try_into().expect("4 bytes"));
        if width == 0 || height == 0 {
            return Err(CodecError::EmptyField);
        }
        if take(&mut cursor, 1)?[0] != 3 {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: 0,
            });
        }
        let mut channels = Vec::with_capacity(3);
        for _ in 0..3 {
            let len = u32::from_le_bytes(take(&mut cursor, 4)?[..4].try_into().expect("4 bytes"))
                as usize;
            let raw = take(&mut cursor, len)?;
            let page = EncodedPage::from_bytes(&raw)?;
            if (page.width, page.height) != (width, height) {
                return Err(CodecError::InconsistentGrid);
            }
            channels.push(page);
        }
        if !cursor.is_empty() {
            return Err(CodecError::TrailingBytes(cursor.len()));
        }
        let channels: [EncodedPage; 3] = channels.try_into().expect("three channels");
        Ok(Self {
            width,
            height,
            channels,
        })
    }
}

/// Worst linear-light error of one candidate block against its source.
fn linear_block_error(texels: [u8; BLOCK_TEXELS], candidate: &EncodedBlock) -> f64 {
    let decoded = decode_block(candidate);
    let mut worst = 0.0f64;
    for (a, b) in texels.iter().zip(decoded.iter()) {
        worst = worst.max((srgb_to_linear(*a) - srgb_to_linear(*b)).abs());
    }
    worst
}

/// Encode one plane with per-block linear-light rung selection.
fn encode_plane_linear(plane: &ScalarField, budget: f64) -> EncodedPage {
    let blocks_x = blocks_for_extent(plane.width);
    let blocks_y = blocks_for_extent(plane.height);
    let mut blocks = Vec::with_capacity((blocks_x * blocks_y) as usize);
    for by in 0..blocks_y {
        for bx in 0..blocks_x {
            let texels = block_texels(plane, bx, by);
            let mut pick = None;
            for codec in ADAPTIVE_LADDER {
                let candidate = encode_block_for(texels, codec);
                if linear_block_error(texels, &candidate) <= budget {
                    pick = Some(codec);
                    break;
                }
            }
            blocks.push(encode_block_for(texels, pick.unwrap_or(MicroCodec::Raw8)));
        }
    }
    EncodedPage::from_blocks(plane.width, plane.height, blocks)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{fixtures, measure_linear};

    fn rgb_fixture(width: u32, height: u32) -> ColorField {
        ColorField::new([
            fixtures::coast(width, height, 0xC0),
            fixtures::noise(width, height, 0x10),
            fixtures::gradient(width, height),
        ])
        .expect("fixture planes share extent")
    }

    #[test]
    fn levels_budget_matches_scalar_adaptive() {
        // Levels selection must be byte-identical to the scalar adaptive
        // mode: same ladder, same budget, same tie-breaks.
        let field = rgb_fixture(24, 24);
        let a = ColorPage::encode_with_budget(&field, ColorBudget::Levels(2.0));
        let b = ColorPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
        assert_eq!(a, b);
    }

    #[test]
    fn linear_budget_respected_on_real_albedo() {
        use crate::height::thessa_albedo_coast;
        let rgb = thessa_albedo_coast();
        let field = ColorField::new(rgb).expect("planes");
        let page = ColorPage::encode_with_budget(&field, ColorBudget::Linear(0.01));
        let stats = measure_linear(&field.channels, &page.decode());
        assert!(stats.max_abs <= 0.01, "linear {}", stats.max_abs);
    }

    #[test]
    fn linear_and_levels_select_differently_on_bright_fields() {
        // On bright ramps one code level spans more light, so a linear
        // budget spends more bytes than the same-named levels budget.
        let bright = ColorField::new([
            fixtures::gradient(32, 32),
            fixtures::gradient(32, 32),
            fixtures::gradient(32, 32),
        ])
        .expect("planes");
        let levels = ColorPage::encode_with_budget(&bright, ColorBudget::Levels(2.0));
        let linear = ColorPage::encode_with_budget(&bright, ColorBudget::Linear(0.008));
        assert!(linear.encoded_bytes() > levels.encoded_bytes());
        let linear_stats = measure_linear(&bright.channels, &linear.decode());
        assert!(linear_stats.max_abs <= 0.008);
    }

    #[test]
    fn srgb_transfer_known_values() {
        use crate::srgb_to_linear;
        assert_eq!(srgb_to_linear(0), 0.0);
        assert_eq!(srgb_to_linear(255), 1.0);
        assert!((srgb_to_linear(128) - 0.2158).abs() < 1e-4);
        // Continuous across the linear/gamma junction at 0.04045.
        let below = srgb_to_linear(10);
        let above = srgb_to_linear(11);
        assert!((above - below).abs() < 0.01);
    }

    #[test]
    fn lossless_color_round_trip() {
        let field = rgb_fixture(40, 32);
        for mode in [EncodeMode::Raw8, EncodeMode::Residual8] {
            let page = ColorPage::encode(&field, mode);
            assert_eq!(page.decode(), field.channels);
            let back = ColorPage::from_bytes(&page.to_bytes()).expect("wire");
            assert_eq!(back, page);
        }
    }

    #[test]
    fn adaptive_color_respects_budget_per_channel() {
        let field = rgb_fixture(32, 32);
        let page = ColorPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
        let decoded = page.decode();
        for (plane, back) in field.channels.iter().zip(decoded.iter()) {
            let stats = crate::measure(plane, back);
            assert!(stats.max_abs <= 2.0, "channel error {}", stats.max_abs);
        }
        // Three channels cost the sum of three scalar pages plus the
        // color header (14 B) and one u32 length per channel.
        let mode = EncodeMode::Adaptive { max_abs_error: 2.0 };
        let expect = 14
            + field
                .channels
                .iter()
                .map(|plane| 4 + EncodedPage::encode(plane, mode).encoded_bytes())
                .sum::<usize>();
        assert_eq!(page.encoded_bytes(), expect);
    }

    #[test]
    fn linear_error_is_zero_on_identity() {
        let field = rgb_fixture(16, 16);
        let stats = measure_linear(&field.channels, &field.channels);
        assert_eq!(stats.max_abs, 0.0);
        assert_eq!(stats.rms, 0.0);
    }

    #[test]
    fn linear_error_stays_small_for_adaptive_color() {
        // Linear-light error of a budget-2 adaptive page must stay visually
        // negligible: byte steps in dark tones map to tiny linear deltas.
        let field = rgb_fixture(48, 48);
        let page = ColorPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
        let stats = measure_linear(&field.channels, &page.decode());
        assert!(stats.max_abs <= 0.05, "linear max {}", stats.max_abs);
        assert!(stats.rms <= 0.01, "linear rms {}", stats.rms);
    }

    #[test]
    fn wire_rejects_micr_and_bad_channel_counts() {
        let field = rgb_fixture(8, 8);
        let page = ColorPage::encode(&field, EncodeMode::Raw8);
        // A scalar MICR page is not a color page.
        let scalar = EncodedPage::encode(&field.channels[0], EncodeMode::Raw8).to_bytes();
        assert_eq!(ColorPage::from_bytes(&scalar), Err(CodecError::BadMagic));
        // Patched channel count.
        let mut bytes = page.to_bytes();
        let nchan_at = 4 + 1 + 4 + 4;
        bytes[nchan_at] = 2;
        assert!(ColorPage::from_bytes(&bytes).is_err());
        // Truncation of a well-formed buffer.
        let bytes = page.to_bytes();
        assert_eq!(
            ColorPage::from_bytes(&bytes[..bytes.len() - 1]),
            Err(CodecError::Truncated)
        );
        let mut bytes = page.to_bytes();
        bytes.push(0);
        assert_eq!(
            ColorPage::from_bytes(&bytes),
            Err(CodecError::TrailingBytes(1))
        );
    }

    #[test]
    fn constructor_rejects_mismatched_planes() {
        let ok = fixtures::gradient(8, 8);
        let bad = fixtures::gradient(8, 4);
        assert!(ColorField::new([ok.clone(), ok.clone(), bad]).is_err());
        let empty = ScalarField::new(0, 0, vec![]).unwrap_err();
        assert_eq!(empty, CodecError::EmptyField);
        assert!(ColorField::new([ok.clone(), ok.clone(), ok]).is_ok());
    }
}
