//! Scalar field type, 4x4 microscaled block codec, and page container.
//!
//! All arithmetic is integer and data-independent in iteration order, so
//! encoding is bit-deterministic for a given input.

use std::fmt;

/// Block edge in texels. Fixed at 4x4 for Phase A (doc §16 also lists
/// 8x4/8x8 as future experiments, not part of this codec).
pub const BLOCK_EDGE: u32 = 4;
/// Texels per block.
pub const BLOCK_TEXELS: usize = 16;
/// Payload bytes for the verbatim and 8-bit residual forms.
pub const PAYLOAD_RAW_LEN: usize = 16;
/// Payload bytes for the packed 4-bit form (two nibbles per byte).
pub const PAYLOAD_R4_LEN: usize = 8;
/// Payload bytes for the packed 6-bit form (four 6-bit codes per 3 bytes).
pub const PAYLOAD_R6_LEN: usize = 12;
/// Payload bytes for the packed 2-bit form (four 2-bit codes per byte).
pub const PAYLOAD_R2_LEN: usize = 4;
/// 4-bit quantizer steps.
pub const R4_STEPS: u32 = 15;
/// 6-bit quantizer steps.
pub const R6_STEPS: u32 = 63;
/// 2-bit quantizer steps.
pub const R2_STEPS: u32 = 3;

const MAGIC: [u8; 4] = *b"MICR";
const VERSION: u8 = 1;
const TAG_RAW8: u8 = 0;
const TAG_RESIDUAL8: u8 = 1;
const TAG_RESIDUAL4: u8 = 2;
const TAG_RESIDUAL6: u8 = 3;
const TAG_RESIDUAL2: u8 = 4;

/// One scalar `u8` channel, row-major.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScalarField {
    /// Texels per row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// `width * height` samples, row-major.
    pub data: Vec<u8>,
}

impl ScalarField {
    /// Build a field; rejects empty extents and short/long buffers.
    pub fn new(width: u32, height: u32, data: Vec<u8>) -> Result<Self, CodecError> {
        if width == 0 || height == 0 {
            return Err(CodecError::EmptyField);
        }
        let Some(expect) = (width as usize).checked_mul(height as usize) else {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: data.len(),
            });
        };
        if data.len() != expect {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: data.len(),
            });
        }
        Ok(Self {
            width,
            height,
            data,
        })
    }

    /// Sample with edge replication outside `[0, width) x [0, height)`.
    /// Used to pad partial edge blocks deterministically.
    pub fn sample_edge(&self, x: i64, y: i64) -> u8 {
        let x = x.clamp(0, self.width as i64 - 1) as usize;
        let y = y.clamp(0, self.height as i64 - 1) as usize;
        self.data[y * self.width as usize + x]
    }
}

/// Block codec tag.
///
/// Wire tags are stable: `0` Raw8, `1` Residual8, `2` Residual4 (Phase A),
/// `3` Residual6, `4` Residual2 (Phase C). The version stays 1 because the
/// extension only adds tags the old parser rejects explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicroCodec {
    /// 16 verbatim bytes. Lossless baseline (doc §16 "raw baseline"), and
    /// the adaptive pathological-block fallback (doc §4).
    Raw8,
    /// `offset = min` plus 16 `value - min` bytes. Lossless, with the same
    /// offset-based decode shape as the packed forms so a future
    /// sample-time decoder can share one path.
    Residual8,
    /// `offset = min`, `scale = max - min`, plus 16 packed 4-bit residuals.
    /// Lossy; per-texel error is at most `scale / 30 + 0.5`.
    Residual4,
    /// `offset = min`, `scale = max - min`, plus 16 packed 6-bit residuals
    /// (12 bytes). Lossy; per-texel error is at most `scale / 126 + 0.5`.
    Residual6,
    /// `offset = min`, `scale = max - min`, plus 16 packed 2-bit residuals
    /// (4 bytes). Lossy; per-texel error is at most `scale / 6 + 0.5`.
    Residual2,
}

impl MicroCodec {
    /// Wire tag byte.
    pub fn tag(self) -> u8 {
        match self {
            MicroCodec::Raw8 => TAG_RAW8,
            MicroCodec::Residual8 => TAG_RESIDUAL8,
            MicroCodec::Residual4 => TAG_RESIDUAL4,
            MicroCodec::Residual6 => TAG_RESIDUAL6,
            MicroCodec::Residual2 => TAG_RESIDUAL2,
        }
    }

    fn from_tag(tag: u8) -> Result<Self, CodecError> {
        match tag {
            TAG_RAW8 => Ok(MicroCodec::Raw8),
            TAG_RESIDUAL8 => Ok(MicroCodec::Residual8),
            TAG_RESIDUAL4 => Ok(MicroCodec::Residual4),
            TAG_RESIDUAL6 => Ok(MicroCodec::Residual6),
            TAG_RESIDUAL2 => Ok(MicroCodec::Residual2),
            other => Err(CodecError::BadCodecTag(other)),
        }
    }

    /// Wire payload length in bytes.
    pub fn payload_len(self) -> usize {
        match self {
            MicroCodec::Raw8 | MicroCodec::Residual8 => PAYLOAD_RAW_LEN,
            MicroCodec::Residual4 => PAYLOAD_R4_LEN,
            MicroCodec::Residual6 => PAYLOAD_R6_LEN,
            MicroCodec::Residual2 => PAYLOAD_R2_LEN,
        }
    }

    /// Whether decode reproduces the input bit-exactly.
    pub fn is_lossless(self) -> bool {
        matches!(self, MicroCodec::Raw8 | MicroCodec::Residual8)
    }

    /// Quantizer steps (`0` for the verbatim forms).
    pub fn steps(self) -> u32 {
        match self {
            MicroCodec::Raw8 | MicroCodec::Residual8 => 0,
            MicroCodec::Residual4 => R4_STEPS,
            MicroCodec::Residual6 => R6_STEPS,
            MicroCodec::Residual2 => R2_STEPS,
        }
    }
}

/// How to encode a page.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EncodeMode {
    /// Every block verbatim.
    Raw8,
    /// Every block as offset + 8-bit residuals (lossless).
    Residual8,
    /// Every block as offset + scale + packed 4-bit nibbles (lossy).
    Residual4,
    /// Every block as offset + scale + packed 6-bit codes (lossy).
    Residual6,
    /// Every block as offset + scale + packed 2-bit codes (lossy).
    Residual2,
    /// Per block (doc §4, cheapest first): `Residual2`, then `Residual4`,
    /// then `Residual6`, each when its worst error fits `max_abs_error`;
    /// otherwise verbatim `Raw8`. Negative/NaN budgets clamp to zero
    /// (strict), `+inf` takes the cheapest lossy form everywhere.
    Adaptive {
        /// Allowed per-texel absolute error in `u8` levels.
        max_abs_error: f64,
    },
}

/// Cheapest-first lossy ladder for [`EncodeMode::Adaptive`].
/// Crate-visible: color selection walks the same rungs.
pub(crate) const ADAPTIVE_LADDER: [MicroCodec; 3] = [
    MicroCodec::Residual2,
    MicroCodec::Residual4,
    MicroCodec::Residual6,
];

/// One encoded 4x4 block: header plus payload bytes.
///
/// `payload` always holds 16 bytes; [`MicroCodec::Residual4`] uses the
/// first 8 (packed nibbles, low nibble = even texel). The wire format
/// writes only [`MicroCodec::payload_len`] bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncodedBlock {
    /// Codec tag.
    pub codec: MicroCodec,
    /// Reference level: verbatim-unused for `Raw8`, block minimum otherwise.
    pub offset: u8,
    /// Reference range (`max - min`): meaningful for `Residual4`, zeroed
    /// otherwise so headers stay canonical.
    pub scale: u8,
    /// Payload bytes (see type docs for used length).
    pub payload: [u8; PAYLOAD_RAW_LEN],
}

/// A microscaled page: true extent plus row-major 4x4 blocks.
///
/// Fields whose extent is not a multiple of 4 are padded by edge
/// replication at encode time; decode crops back to the true extent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EncodedPage {
    /// True field width in texels.
    pub width: u32,
    /// True field height in texels.
    pub height: u32,
    /// Blocks per row (`ceil(width / 4)`).
    pub blocks_x: u32,
    /// Block rows (`ceil(height / 4)`).
    pub blocks_y: u32,
    /// Row-major blocks, `blocks_x * blocks_y` entries.
    pub blocks: Vec<EncodedBlock>,
}

/// Codec failure modes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CodecError {
    /// No texels (`width == 0 || height == 0`).
    EmptyField,
    /// `data.len() != width * height`.
    BadExtent {
        /// Declared width.
        width: u32,
        /// Declared height.
        height: u32,
        /// Actual sample count.
        samples: usize,
    },
    /// First four bytes are not `MICR`.
    BadMagic,
    /// Version byte is not 1.
    UnsupportedVersion(u8),
    /// Buffer ends mid-header or mid-payload.
    Truncated,
    /// Bytes remain after the declared blocks.
    TrailingBytes(usize),
    /// Unknown block codec tag.
    BadCodecTag(u8),
    /// Block grid does not match the declared extent.
    InconsistentGrid,
    /// A float that must be finite (header bound or verbatim sample) is
    /// NaN or infinite.
    NonFiniteFloat,
    /// A verbatim sample or block range escapes the page-declared
    /// conservative bounds.
    OutOfBoundsSample,
}

impl fmt::Display for CodecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CodecError::EmptyField => write!(f, "field extent must be non-zero"),
            CodecError::BadExtent {
                width,
                height,
                samples,
            } => write!(f, "field {width}x{height} does not match {samples} samples"),
            CodecError::BadMagic => write!(f, "bad microscale magic, want MICR"),
            CodecError::UnsupportedVersion(v) => {
                write!(f, "unsupported microscale version {v}")
            }
            CodecError::Truncated => write!(f, "truncated microscale page"),
            CodecError::TrailingBytes(n) => write!(f, "{n} trailing bytes after page"),
            CodecError::BadCodecTag(t) => write!(f, "unknown block codec tag {t}"),
            CodecError::InconsistentGrid => {
                write!(f, "block grid does not match field extent")
            }
            CodecError::NonFiniteFloat => write!(f, "non-finite float in page"),
            CodecError::OutOfBoundsSample => {
                write!(f, "sample escapes declared page bounds")
            }
        }
    }
}

impl std::error::Error for CodecError {}

/// Number of blocks per row for a field width.
pub fn blocks_for_extent(extent: u32) -> u32 {
    extent.div_ceil(BLOCK_EDGE)
}

/// Extract one 4x4 block with edge replication (crate-internal: the color
/// module reuses block addressing for metric-pluggable selection).
pub(crate) fn block_texels(field: &ScalarField, bx: u32, by: u32) -> [u8; BLOCK_TEXELS] {
    let mut out = [0u8; BLOCK_TEXELS];
    for y in 0..BLOCK_EDGE {
        for x in 0..BLOCK_EDGE {
            out[(y * BLOCK_EDGE + x) as usize] =
                field.sample_edge((bx * BLOCK_EDGE + x) as i64, (by * BLOCK_EDGE + y) as i64);
        }
    }
    out
}

fn encode_raw8(texels: [u8; BLOCK_TEXELS]) -> EncodedBlock {
    EncodedBlock {
        codec: MicroCodec::Raw8,
        offset: 0,
        scale: 0,
        payload: texels,
    }
}

fn block_min_max(texels: [u8; BLOCK_TEXELS]) -> (u8, u8) {
    let mut min = u8::MAX;
    let mut max = u8::MIN;
    for v in texels {
        min = min.min(v);
        max = max.max(v);
    }
    (min, max)
}

fn encode_residual8(texels: [u8; BLOCK_TEXELS]) -> EncodedBlock {
    let (min, _) = block_min_max(texels);
    let mut payload = [0u8; PAYLOAD_RAW_LEN];
    for (i, v) in texels.iter().enumerate() {
        payload[i] = v.wrapping_sub(min);
    }
    EncodedBlock {
        codec: MicroCodec::Residual8,
        offset: min,
        scale: 0,
        payload,
    }
}

/// Quantize one texel against `[min, min + range]` to `steps` levels.
///
/// Returns the code in `0..=steps` minimizing
/// `|q * range / steps - (v - min)|` with ties going up (deterministic).
fn quantize(value: u8, min: u8, range: u32, steps: u32) -> u32 {
    debug_assert!(steps > 0);
    if range == 0 {
        return 0;
    }
    let v = (value - min) as u32;
    // round(steps * v / range), clamped to the code range.
    ((steps * v + range / 2) / range).min(steps)
}

/// Decode one code with integer rounding: `min + round(q * range / steps)`.
fn dequantize(q: u32, min: u8, range: u32, steps: u32) -> u8 {
    debug_assert!(steps > 0);
    if range == 0 {
        return min;
    }
    (min as u32 + (q * range + steps / 2) / steps).min(255) as u8
}

/// Quantize one texel to 4 bits against `[min, min + range]`.
///
/// Returns the nibble `q` in `0..=15` minimizing `|q * range / 15 - (v - min)|`
/// with ties going up (deterministic).
fn quantize_nibble(value: u8, min: u8, range: u32) -> u8 {
    quantize(value, min, range, R4_STEPS) as u8
}

/// Decode one 4-bit nibble with integer rounding:
/// `min + round(q * range / 15)`.
fn dequantize_nibble(q: u8, min: u8, range: u32) -> u8 {
    dequantize(q as u32, min, range, R4_STEPS)
}

fn encode_residual4(texels: [u8; BLOCK_TEXELS]) -> EncodedBlock {
    let (min, max) = block_min_max(texels);
    let range = (max - min) as u32;
    let mut payload = [0u8; PAYLOAD_RAW_LEN];
    for (i, v) in texels.iter().enumerate() {
        let q = quantize_nibble(*v, min, range);
        if i % 2 == 0 {
            payload[i / 2] |= q;
        } else {
            payload[i / 2] |= q << 4;
        }
    }
    EncodedBlock {
        codec: MicroCodec::Residual4,
        offset: min,
        scale: range as u8,
        payload,
    }
}

/// Pack sixteen 6-bit codes into 12 bytes, little-endian bit stream:
/// texel `i` occupies bits `[6i, 6i + 6)`.
fn encode_residual6(texels: [u8; BLOCK_TEXELS]) -> EncodedBlock {
    let (min, max) = block_min_max(texels);
    let range = (max - min) as u32;
    let mut bits: u128 = 0;
    for (i, v) in texels.iter().enumerate() {
        bits |= (quantize(*v, min, range, R6_STEPS) as u128) << (i * 6);
    }
    let mut payload = [0u8; PAYLOAD_RAW_LEN];
    payload[..PAYLOAD_R6_LEN].copy_from_slice(&bits.to_le_bytes()[..PAYLOAD_R6_LEN]);
    EncodedBlock {
        codec: MicroCodec::Residual6,
        offset: min,
        scale: range as u8,
        payload,
    }
}

/// Pack sixteen 2-bit codes into 4 bytes: texel `i` occupies bits
/// `[2i, 2i + 2)` of byte `i / 4`.
fn encode_residual2(texels: [u8; BLOCK_TEXELS]) -> EncodedBlock {
    let (min, max) = block_min_max(texels);
    let range = (max - min) as u32;
    let mut payload = [0u8; PAYLOAD_RAW_LEN];
    for (i, v) in texels.iter().enumerate() {
        payload[i / 4] |= (quantize(*v, min, range, R2_STEPS) as u8) << ((i % 4) * 2);
    }
    EncodedBlock {
        codec: MicroCodec::Residual2,
        offset: min,
        scale: range as u8,
        payload,
    }
}

/// Worst per-texel absolute error of one lossy codec on these texels.
fn lossy_block_error(texels: [u8; BLOCK_TEXELS], codec: MicroCodec) -> f64 {
    let steps = codec.steps();
    debug_assert!(steps > 0);
    let (min, max) = block_min_max(texels);
    let range = (max - min) as u32;
    let mut worst = 0u32;
    for v in texels {
        let q = quantize(v, min, range, steps);
        let got = dequantize(q, min, range, steps);
        worst = worst.max(v.abs_diff(got) as u32);
    }
    worst as f64
}

fn decode_packed6(payload: &[u8], index: usize) -> u32 {
    // 6-bit code spanning at most two bytes of the little-endian stream.
    let bit = index * 6;
    let byte = bit / 8;
    let shift = bit % 8;
    let lo = payload[byte] as u32;
    let hi = payload.get(byte + 1).copied().unwrap_or(0) as u32;
    ((lo >> shift) | (hi << (8 - shift))) & 63
}

/// Encode one block with an explicit codec (crate-internal selection).
pub(crate) fn encode_block_for(texels: [u8; BLOCK_TEXELS], codec: MicroCodec) -> EncodedBlock {
    match codec {
        MicroCodec::Raw8 => encode_raw8(texels),
        MicroCodec::Residual8 => encode_residual8(texels),
        MicroCodec::Residual4 => encode_residual4(texels),
        MicroCodec::Residual6 => encode_residual6(texels),
        MicroCodec::Residual2 => encode_residual2(texels),
    }
}

/// Decode one block (crate-internal: metric-pluggable selection).
pub(crate) fn decode_block(block: &EncodedBlock) -> [u8; BLOCK_TEXELS] {
    let mut out = [0u8; BLOCK_TEXELS];
    match block.codec {
        MicroCodec::Raw8 => out.copy_from_slice(&block.payload),
        MicroCodec::Residual8 => {
            for (i, r) in block.payload.iter().enumerate() {
                out[i] = block.offset.wrapping_add(*r);
            }
        }
        MicroCodec::Residual4 => {
            let range = block.scale as u32;
            for (i, slot) in out.iter_mut().enumerate() {
                let byte = block.payload[i / 2];
                let q = if i % 2 == 0 { byte & 0x0F } else { byte >> 4 };
                *slot = dequantize_nibble(q, block.offset, range);
            }
        }
        MicroCodec::Residual6 => {
            let range = block.scale as u32;
            for (i, slot) in out.iter_mut().enumerate() {
                *slot = dequantize(
                    decode_packed6(&block.payload, i),
                    block.offset,
                    range,
                    R6_STEPS,
                );
            }
        }
        MicroCodec::Residual2 => {
            let range = block.scale as u32;
            for (i, slot) in out.iter_mut().enumerate() {
                let q = (block.payload[i / 4] >> ((i % 4) * 2)) & 0x03;
                *slot = dequantize(q as u32, block.offset, range, R2_STEPS);
            }
        }
    }
    out
}

impl EncodedPage {
    /// Assemble a page from pre-encoded blocks (crate-internal: metric-
    /// pluggable selection in the color module). The grid must match the
    /// true extent; use [`EncodedPage::validate`] before uploading pages
    /// built this way.
    pub(crate) fn from_blocks(width: u32, height: u32, blocks: Vec<EncodedBlock>) -> Self {
        Self {
            width,
            height,
            blocks_x: blocks_for_extent(width),
            blocks_y: blocks_for_extent(height),
            blocks,
        }
    }

    /// Encode a field with the given mode.
    ///
    /// `Adaptive` walks [`ADAPTIVE_LADDER`] cheapest-first and takes the
    /// first codec whose worst block error fits the budget, else verbatim
    /// `Raw8` (doc §4 pathological fallback).
    pub fn encode(field: &ScalarField, mode: EncodeMode) -> Self {
        let blocks_x = blocks_for_extent(field.width);
        let blocks_y = blocks_for_extent(field.height);
        let mut blocks = Vec::with_capacity((blocks_x * blocks_y) as usize);
        for by in 0..blocks_y {
            for bx in 0..blocks_x {
                let texels = block_texels(field, bx, by);
                let block = match mode {
                    EncodeMode::Raw8 => encode_raw8(texels),
                    EncodeMode::Residual8 => encode_residual8(texels),
                    EncodeMode::Residual4 => encode_residual4(texels),
                    EncodeMode::Residual6 => encode_residual6(texels),
                    EncodeMode::Residual2 => encode_residual2(texels),
                    EncodeMode::Adaptive { max_abs_error } => {
                        // `f64::max` maps negative to 0.0 and NaN to 0.0
                        // (strict), while +inf stays infinite (all lossy).
                        let budget = max_abs_error.max(0.0);
                        let mut pick = None;
                        for codec in ADAPTIVE_LADDER {
                            if lossy_block_error(texels, codec) <= budget {
                                pick = Some(codec);
                                break;
                            }
                        }
                        match pick {
                            Some(MicroCodec::Residual2) => encode_residual2(texels),
                            Some(MicroCodec::Residual4) => encode_residual4(texels),
                            Some(MicroCodec::Residual6) => encode_residual6(texels),
                            _ => encode_raw8(texels),
                        }
                    }
                };
                blocks.push(block);
            }
        }
        Self {
            width: field.width,
            height: field.height,
            blocks_x,
            blocks_y,
            blocks,
        }
    }

    /// Decode to the true field extent (edge padding is cropped).
    pub fn decode(&self) -> ScalarField {
        let mut data = vec![0u8; self.width as usize * self.height as usize];
        for (index, block) in self.blocks.iter().enumerate() {
            let bx = index as u32 % self.blocks_x;
            let by = index as u32 / self.blocks_x;
            let texels = decode_block(block);
            for y in 0..BLOCK_EDGE {
                for x in 0..BLOCK_EDGE {
                    let fx = bx * BLOCK_EDGE + x;
                    let fy = by * BLOCK_EDGE + y;
                    if fx < self.width && fy < self.height {
                        data[(fy * self.width + fx) as usize] =
                            texels[(y * BLOCK_EDGE + x) as usize];
                    }
                }
            }
        }
        // Extent is internally consistent by construction.
        ScalarField {
            width: self.width,
            height: self.height,
            data,
        }
    }

    /// Exact wire size in bytes.
    pub fn encoded_bytes(&self) -> usize {
        Self::HEADER_LEN
            + self
                .blocks
                .iter()
                .map(|b| Self::BLOCK_HEADER_LEN + b.codec.payload_len())
                .sum::<usize>()
    }

    /// Bytes per texel of the true extent (the §9 residency metric).
    pub fn bytes_per_texel(&self) -> f64 {
        self.encoded_bytes() as f64 / (self.width as f64 * self.height as f64)
    }

    /// Metadata overhead split (doc §18 Phase C: "measure block metadata
    /// overhead"): `(header_bytes, payload_bytes)`. Headers are the page
    /// header plus one 3-byte block header per block; everything else is
    /// payload.
    pub fn overhead_bytes(&self) -> (usize, usize) {
        let header = Self::HEADER_LEN + self.blocks.len() * Self::BLOCK_HEADER_LEN;
        let total = self.encoded_bytes();
        (header, total - header)
    }

    /// Fraction of wire bytes spent on headers rather than residuals.
    pub fn header_fraction(&self) -> f64 {
        let (header, payload) = self.overhead_bytes();
        header as f64 / (header + payload) as f64
    }

    /// Codec histogram over blocks, indexable by [`MicroCodec::tag`]
    /// (`[Raw8, Residual8, Residual4, Residual6, Residual2]`).
    pub fn codec_histogram(&self) -> [usize; 5] {
        let mut hist = [0usize; 5];
        for block in &self.blocks {
            hist[block.codec.tag() as usize] += 1;
        }
        hist
    }

    /// Check the structural invariant the GPU backend relies on: the block
    /// grid matches the true extent and holds exactly
    /// `blocks_x * blocks_y` blocks. Backends must call this on any page
    /// they did not encode themselves (hand-built or FFI pages) before
    /// uploading: block indexing does `by * blocks_x + bx` with no
    /// further bounds checks on the hot path.
    pub fn validate(&self) -> Result<(), CodecError> {
        if self.width == 0 || self.height == 0 {
            return Err(CodecError::EmptyField);
        }
        if self.blocks_x != blocks_for_extent(self.width)
            || self.blocks_y != blocks_for_extent(self.height)
        {
            return Err(CodecError::InconsistentGrid);
        }
        if self.blocks.len() != self.blocks_x as usize * self.blocks_y as usize {
            return Err(CodecError::InconsistentGrid);
        }
        Ok(())
    }

    const HEADER_LEN: usize = 4 + 1 + 4 * 4;
    const BLOCK_HEADER_LEN: usize = 3;

    /// Deterministic little-endian wire encoding.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.encoded_bytes());
        out.extend_from_slice(&MAGIC);
        out.push(VERSION);
        for v in [self.width, self.height, self.blocks_x, self.blocks_y] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        for block in &self.blocks {
            out.push(block.codec.tag());
            out.push(block.offset);
            out.push(block.scale);
            out.extend_from_slice(&block.payload[..block.codec.payload_len()]);
        }
        out
    }

    /// Parse [`EncodedPage::to_bytes`] output, rejecting any deviation:
    /// bad magic/version/tag, truncation, trailing bytes, or a block grid
    /// inconsistent with the declared extent.
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
        let magic = take(&mut cursor, 4)?;
        if magic != MAGIC {
            return Err(CodecError::BadMagic);
        }
        let version = take(&mut cursor, 1)?[0];
        if version != VERSION {
            return Err(CodecError::UnsupportedVersion(version));
        }
        let mut dims = [0u32; 4];
        for d in &mut dims {
            let raw = take(&mut cursor, 4)?;
            *d = u32::from_le_bytes(raw.try_into().expect("4 bytes"));
        }
        let [width, height, blocks_x, blocks_y] = dims;
        if width == 0 || height == 0 {
            return Err(CodecError::EmptyField);
        }
        if blocks_x != blocks_for_extent(width) || blocks_y != blocks_for_extent(height) {
            return Err(CodecError::InconsistentGrid);
        }
        let count = blocks_x as u64 * blocks_y as u64;
        // Refuse absurd grids before allocating: every block needs at
        // least its 3-byte header plus the smallest payload (4 bytes,
        // Residual2). A u32::MAX extent would otherwise turn
        // `with_capacity` into an OOM instead of a clean Truncated.
        let min_needed = count
            .checked_mul(3 + PAYLOAD_R2_LEN as u64)
            .and_then(|payload| payload.checked_add(Self::HEADER_LEN as u64));
        if min_needed.is_none_or(|need| (bytes.len() as u64) < need) {
            return Err(CodecError::Truncated);
        }
        let mut blocks = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let head = take(&mut cursor, Self::BLOCK_HEADER_LEN)?;
            let codec = MicroCodec::from_tag(head[0])?;
            let payload = take(&mut cursor, codec.payload_len())?;
            let mut raw = [0u8; PAYLOAD_RAW_LEN];
            raw[..payload.len()].copy_from_slice(&payload);
            blocks.push(EncodedBlock {
                codec,
                offset: head[1],
                scale: head[2],
                payload: raw,
            });
        }
        if !cursor.is_empty() {
            return Err(CodecError::TrailingBytes(cursor.len()));
        }
        Ok(Self {
            width,
            height,
            blocks_x,
            blocks_y,
            blocks,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{fixtures, measure};

    fn modes() -> Vec<EncodeMode> {
        vec![
            EncodeMode::Raw8,
            EncodeMode::Residual8,
            EncodeMode::Residual4,
            EncodeMode::Residual6,
            EncodeMode::Residual2,
            EncodeMode::Adaptive { max_abs_error: 2.0 },
        ]
    }

    #[test]
    fn scalar_field_rejects_empty_and_ragged() {
        assert_eq!(
            ScalarField::new(0, 4, vec![0; 8]),
            Err(CodecError::EmptyField)
        );
        assert_eq!(
            ScalarField::new(4, 0, vec![0; 8]),
            Err(CodecError::EmptyField)
        );
        assert_eq!(
            ScalarField::new(4, 4, vec![0; 15]),
            Err(CodecError::BadExtent {
                width: 4,
                height: 4,
                samples: 15
            })
        );
        assert_eq!(
            ScalarField::new(4, 4, vec![0; 17]),
            Err(CodecError::BadExtent {
                width: 4,
                height: 4,
                samples: 17
            })
        );
        assert!(ScalarField::new(4, 4, vec![7; 16]).is_ok());
    }

    #[test]
    fn scalar_field_reports_extent_overflow_without_panicking() {
        let error = ScalarField::new(u32::MAX, u32::MAX, Vec::new()).unwrap_err();
        assert!(matches!(error, CodecError::BadExtent { .. }));
        assert!(error.to_string().contains("4294967295x4294967295"));
    }

    #[test]
    fn lossless_modes_round_trip_all_fixtures() {
        for (name, field) in fixtures::all(37, 41) {
            for mode in [EncodeMode::Raw8, EncodeMode::Residual8] {
                let page = EncodedPage::encode(&field, mode);
                assert_eq!(page.decode(), field, "{name} {mode:?} round trip");
            }
        }
    }

    #[test]
    fn packed_error_bounds_hold_on_adversarial() {
        // Full-range checker is the worst case for every packed form.
        let field = fixtures::checker(32, 32, 1);
        for (mode, bound) in [
            (EncodeMode::Residual4, 9.0),
            (EncodeMode::Residual6, 2.6),
            (EncodeMode::Residual2, 43.0),
        ] {
            let page = EncodedPage::encode(&field, mode);
            let stats = measure(&field, &page.decode());
            assert!(
                stats.max_abs <= bound,
                "{mode:?}: worst error {} > {bound}",
                stats.max_abs
            );
        }
        // And the bounds hold on every other fixture too.
        for (name, field) in fixtures::all(64, 64) {
            for (mode, bound) in [
                (EncodeMode::Residual4, 9.0),
                (EncodeMode::Residual6, 2.6),
                (EncodeMode::Residual2, 43.0),
            ] {
                let page = EncodedPage::encode(&field, mode);
                let stats = measure(&field, &page.decode());
                assert!(stats.max_abs <= bound, "{name} {mode:?}: {}", stats.max_abs);
            }
        }
    }

    #[test]
    fn quantizer_error_bound_is_tight_for_all_ranges() {
        // Exhaustive per-(value, range) check of the documented bound
        // `range / (2 * steps) + 0.5` behind every packed codec.
        for (steps, divisor) in [(R4_STEPS, 30.0), (R6_STEPS, 126.0), (R2_STEPS, 6.0)] {
            for range in [1u32, 7, 63, 200, 255] {
                for v in 0..=255u8 {
                    let min = 0u8;
                    let value = ((v as u32).min(range)) as u8;
                    let q = quantize(value, min, range, steps);
                    assert!(q <= steps);
                    let got = dequantize(q, min, range, steps);
                    let err = value.abs_diff(got) as f64;
                    assert!(
                        err <= range as f64 / divisor + 0.5 + 1e-9,
                        "steps={steps} range={range} v={value} err={err}"
                    );
                }
            }
        }
    }

    #[test]
    fn residual4_uniform_block_is_exact_and_compact() {
        let field = fixtures::uniform(128, 128, 200);
        let page = EncodedPage::encode(&field, EncodeMode::Residual4);
        assert_eq!(page.codec_histogram(), [0, 0, 1024, 0, 0]);
        let stats = measure(&field, &page.decode());
        assert_eq!(stats.max_abs, 0.0);
        // Header (21 B) + 1024 blocks x (3 B header + 8 B payload).
        assert_eq!(page.encoded_bytes(), 21 + 1024 * 11);
        assert_eq!(page.to_bytes().len(), page.encoded_bytes());
    }

    #[test]
    fn adaptive_zero_budget_selects_lossless_on_texture() {
        // White noise never lands exactly on any packed grid, so every
        // block misses a zero budget and falls back to verbatim Raw8.
        let noise = fixtures::noise(16, 16, 0xABCD);
        let page = EncodedPage::encode(&noise, EncodeMode::Adaptive { max_abs_error: 0.0 });
        assert_eq!(page.codec_histogram(), [16, 0, 0, 0, 0]);
        assert_eq!(page.decode(), noise);

        // A flat block is exact in 2 bits, the cheapest rung.
        let flat = fixtures::uniform(16, 16, 90);
        let page = EncodedPage::encode(&flat, EncodeMode::Adaptive { max_abs_error: 0.0 });
        assert_eq!(page.codec_histogram(), [0, 0, 0, 0, 16]);
        assert_eq!(page.decode(), flat);
    }

    #[test]
    fn adaptive_huge_budget_selects_cheapest_everywhere() {
        let checker = fixtures::checker(16, 16, 1);
        let page = EncodedPage::encode(&checker, EncodeMode::Adaptive { max_abs_error: 1e9 });
        assert_eq!(page.codec_histogram(), [0, 0, 0, 0, 16]);
    }

    #[test]
    fn adaptive_global_error_never_exceeds_finite_budget() {
        // Every block is either within budget (packed rungs) or exact
        // (Raw8 fallback), so the page error is bounded by construction.
        for (_, field) in fixtures::all(40, 32) {
            for budget in [0.0, 1.0, 2.0, 5.0, 40.0] {
                let page = EncodedPage::encode(
                    &field,
                    EncodeMode::Adaptive {
                        max_abs_error: budget,
                    },
                );
                let stats = measure(&field, &page.decode());
                assert!(
                    stats.max_abs <= budget,
                    "budget {budget}: error {}",
                    stats.max_abs
                );
            }
        }
    }

    #[test]
    fn adaptive_bytes_shrink_monotonically_with_budget() {
        // Looser budgets can only move blocks to cheaper rungs.
        let noise = fixtures::noise(32, 32, 0x5EED);
        let mut prev = usize::MAX;
        for budget in [0.0, 1.0, 2.0, 5.0, 10.0, 50.0, 1e9] {
            let page = EncodedPage::encode(
                &noise,
                EncodeMode::Adaptive {
                    max_abs_error: budget,
                },
            );
            assert!(
                page.encoded_bytes() <= prev,
                "budget {budget}: {} > {prev}",
                page.encoded_bytes()
            );
            prev = page.encoded_bytes();
        }
    }

    #[test]
    fn adaptive_negative_nan_and_infinite_budgets() {
        let texture = fixtures::noise(8, 8, 0x777);
        let strict = EncodeMode::Adaptive {
            max_abs_error: -5.0,
        };
        let nan = EncodeMode::Adaptive {
            max_abs_error: f64::NAN,
        };
        let zero = EncodeMode::Adaptive { max_abs_error: 0.0 };
        // Negative and NaN budgets behave as zero (strict): textured blocks
        // fall back to verbatim.
        assert_eq!(
            EncodedPage::encode(&texture, strict).codec_histogram(),
            [4, 0, 0, 0, 0]
        );
        assert_eq!(
            EncodedPage::encode(&texture, strict).codec_histogram(),
            EncodedPage::encode(&texture, zero).codec_histogram()
        );
        assert_eq!(
            EncodedPage::encode(&texture, nan).codec_histogram(),
            EncodedPage::encode(&texture, zero).codec_histogram()
        );
        let inf = EncodeMode::Adaptive {
            max_abs_error: f64::INFINITY,
        };
        assert_eq!(
            EncodedPage::encode(&texture, inf).codec_histogram(),
            [0, 0, 0, 0, 4]
        );
    }

    #[test]
    fn adaptive_mixed_page_histogram() {
        // Left half constant (cheapest Residual2 rung at zero budget),
        // right half white noise (verbatim Raw8): 8x8 field -> 2x2 blocks.
        let noisy = fixtures::noise(8, 8, 0x1234);
        let mut data = vec![0u8; 64];
        for y in 0..8 {
            for x in 0..8 {
                data[y * 8 + x] = if x < 4 { 100 } else { noisy.data[y * 8 + x] };
            }
        }
        let field = ScalarField::new(8, 8, data).unwrap();
        let page = EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 0.0 });
        assert_eq!(page.codec_histogram(), [2, 0, 0, 0, 2]);
    }

    #[test]
    fn encode_is_deterministic_across_modes_and_fixtures() {
        for (_, field) in fixtures::all(48, 32) {
            for mode in modes() {
                let a = EncodedPage::encode(&field, mode).to_bytes();
                let b = EncodedPage::encode(&field, mode).to_bytes();
                assert_eq!(a, b, "{mode:?} determinism");
            }
        }
    }

    #[test]
    fn bytes_per_texel_math() {
        // One 4x4 Residual4 block: 21 B header + 3 B block header + 8 B payload.
        let field = fixtures::uniform(4, 4, 5);
        let page = EncodedPage::encode(&field, EncodeMode::Residual4);
        assert_eq!(page.encoded_bytes(), 21 + 11);
        assert!((page.bytes_per_texel() - 32.0 / 16.0).abs() < 1e-12);
    }

    #[test]
    fn wire_round_trip_all_modes_and_fixtures() {
        for (_, field) in fixtures::all(24, 20) {
            for mode in modes() {
                let page = EncodedPage::encode(&field, mode);
                let back = EncodedPage::from_bytes(&page.to_bytes()).expect("wire parse");
                assert_eq!(back, page);
                assert_eq!(back.decode(), page.decode());
            }
        }
    }

    #[test]
    fn absurd_dimensions_fail_cleanly_without_allocating() {
        // u32::MAX extents would need ~2^60 blocks: the parser must answer
        // Truncated from checked arithmetic, never attempt the allocation.
        let mut header = Vec::new();
        header.extend_from_slice(b"MICR");
        header.push(1);
        for v in [u32::MAX, u32::MAX, 1 << 30, 1 << 30] {
            header.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(EncodedPage::from_bytes(&header), Err(CodecError::Truncated));
    }

    #[test]
    fn wire_rejects_bad_magic() {
        let field = fixtures::uniform(8, 8, 1);
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes[0] = b'X';
        assert_eq!(EncodedPage::from_bytes(&bytes), Err(CodecError::BadMagic));
        assert_eq!(EncodedPage::from_bytes(&[]), Err(CodecError::Truncated));
        assert_eq!(EncodedPage::from_bytes(b"MI"), Err(CodecError::Truncated));
    }

    #[test]
    fn wire_rejects_bad_version_and_tag() {
        let field = fixtures::uniform(8, 8, 1);
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes[4] = 99;
        assert_eq!(
            EncodedPage::from_bytes(&bytes),
            Err(CodecError::UnsupportedVersion(99))
        );
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes[21] = 7;
        assert_eq!(
            EncodedPage::from_bytes(&bytes),
            Err(CodecError::BadCodecTag(7))
        );
    }

    #[test]
    fn wire_rejects_truncation_at_every_cut() {
        let field = fixtures::checker(8, 8, 2);
        let bytes = EncodedPage::encode(&field, EncodeMode::Residual4).to_bytes();
        // Cut at header, mid-header, mid-payload, and last byte.
        for cut in [5, 10, 20, 21, 30, bytes.len() - 1] {
            assert_eq!(
                EncodedPage::from_bytes(&bytes[..cut]),
                Err(CodecError::Truncated),
                "cut at {cut}"
            );
        }
    }

    #[test]
    fn wire_rejects_trailing_bytes_and_bad_grid() {
        let field = fixtures::uniform(8, 8, 1);
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes.push(0xAA);
        assert_eq!(
            EncodedPage::from_bytes(&bytes),
            Err(CodecError::TrailingBytes(1))
        );
        // Patch blocks_x from 2 to 3: grid no longer matches the 8x8 extent.
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes[13] = 3;
        assert_eq!(
            EncodedPage::from_bytes(&bytes),
            Err(CodecError::InconsistentGrid)
        );
        // Zero extent in an otherwise shaped header.
        let mut bytes = EncodedPage::encode(&field, EncodeMode::Raw8).to_bytes();
        bytes[5] = 0;
        bytes[6] = 0;
        bytes[7] = 0;
        bytes[8] = 0;
        assert_eq!(EncodedPage::from_bytes(&bytes), Err(CodecError::EmptyField));
    }

    #[test]
    fn odd_extents_pad_by_edge_replication() {
        // 5x7 needs a 2x2 block grid; the padded rim replicates the edge.
        let field = fixtures::noise(5, 7, 42);
        for mode in [EncodeMode::Raw8, EncodeMode::Residual8] {
            let page = EncodedPage::encode(&field, mode);
            assert_eq!((page.blocks_x, page.blocks_y), (2, 2));
            assert_eq!(page.decode(), field);
        }
        let page = EncodedPage::encode(&field, EncodeMode::Residual4);
        assert_eq!((page.blocks_x, page.blocks_y), (2, 2));
        // Lossy decode still covers the true extent exactly.
        let decoded = page.decode();
        assert_eq!((decoded.width, decoded.height), (5, 7));
    }

    #[test]
    fn tiny_pages_round_trip() {
        for (w, h) in [(1, 1), (4, 4), (3, 5)] {
            let field = fixtures::noise(w, h, 7);
            for mode in [EncodeMode::Raw8, EncodeMode::Residual8] {
                assert_eq!(EncodedPage::encode(&field, mode).decode(), field);
            }
        }
    }

    #[test]
    fn nibble_layout_is_low_even_high_odd() {
        // Texels 0..16, min 0, range 15: nibble q equals the texel value.
        let texels: [u8; 16] = std::array::from_fn(|i| i as u8);
        let block = encode_residual4(texels);
        assert_eq!(block.codec, MicroCodec::Residual4);
        assert_eq!(block.offset, 0);
        assert_eq!(block.scale, 15);
        for i in 0..8 {
            let expect = (2 * i as u8) | ((2 * i as u8 + 1) << 4);
            assert_eq!(block.payload[i], expect, "byte {i}");
        }
        assert_eq!(decode_block(&block), texels);
    }

    #[test]
    fn dequantize_rounding_cases() {
        // round(7 * 255 / 15) = round(119.0) = 119.
        assert_eq!(dequantize_nibble(7, 0, 255), 119);
        // Zero range always reproduces the reference.
        assert_eq!(dequantize_nibble(15, 200, 0), 200);
        assert_eq!(quantize_nibble(123, 123, 0), 0);
        // Full-scale excess clamps to the nibble, never wraps.
        assert_eq!(quantize_nibble(255, 0, 1), 15);
    }

    #[test]
    fn zero_range_block_is_exact() {
        let texels = [77u8; 16];
        let block = encode_residual4(texels);
        assert_eq!((block.offset, block.scale), (77, 0));
        assert_eq!(decode_block(&block), texels);
        assert_eq!(lossy_block_error(texels, MicroCodec::Residual4), 0.0);
    }

    #[test]
    fn codec_predicates_and_helpers() {
        assert!(MicroCodec::Raw8.is_lossless());
        assert!(MicroCodec::Residual8.is_lossless());
        assert!(!MicroCodec::Residual4.is_lossless());
        assert!(!MicroCodec::Residual6.is_lossless());
        assert!(!MicroCodec::Residual2.is_lossless());
        assert_eq!(MicroCodec::Raw8.payload_len(), 16);
        assert_eq!(MicroCodec::Residual8.payload_len(), 16);
        assert_eq!(MicroCodec::Residual4.payload_len(), 8);
        assert_eq!(MicroCodec::Residual6.payload_len(), 12);
        assert_eq!(MicroCodec::Residual2.payload_len(), 4);
        assert_eq!(MicroCodec::Raw8.steps(), 0);
        assert_eq!(MicroCodec::Residual8.steps(), 0);
        assert_eq!(MicroCodec::Residual4.steps(), 15);
        assert_eq!(MicroCodec::Residual6.steps(), 63);
        assert_eq!(MicroCodec::Residual2.steps(), 3);
        for (tag, codec) in [
            (0, MicroCodec::Raw8),
            (1, MicroCodec::Residual8),
            (2, MicroCodec::Residual4),
            (3, MicroCodec::Residual6),
            (4, MicroCodec::Residual2),
        ] {
            assert_eq!(MicroCodec::from_tag(tag), Ok(codec));
            assert_eq!(codec.tag(), tag);
        }
        // Tags stop at 4: the next tag is a future codec, not a silent alias.
        assert_eq!(MicroCodec::from_tag(5), Err(CodecError::BadCodecTag(5)));
        assert_eq!(blocks_for_extent(1), 1);
        assert_eq!(blocks_for_extent(4), 1);
        assert_eq!(blocks_for_extent(5), 2);
        assert_eq!(blocks_for_extent(128), 32);
    }

    #[test]
    fn residual6_bit_layout_spans_byte_pairs() {
        // Two-level {0, 63} block: codes alternate 0 and 63 exactly.
        let mut texels = [0u8; BLOCK_TEXELS];
        for (i, v) in texels.iter_mut().enumerate() {
            *v = if i % 2 == 0 { 0 } else { 63 };
        }
        let block = encode_residual6(texels);
        assert_eq!((block.offset, block.scale), (0, 63));
        // Texel 0: bits 0..6 = 0; texel 1: bits 6..12 = all ones.
        assert_eq!(block.payload[0], 0xC0);
        assert_eq!(block.payload[1], 0x0F);
        assert_eq!(decode_block(&block), texels);
    }

    #[test]
    fn residual2_byte_layout_packs_four_codes() {
        // Levels {0, 85, 170, 255} quantize to codes 0..=3 exactly.
        let texels: [u8; BLOCK_TEXELS] = std::array::from_fn(|i| [0, 85, 170, 255][i % 4]);
        let block = encode_residual2(texels);
        assert_eq!((block.offset, block.scale), (0, 255));
        // Byte 0: q0 | q1<<2 | q2<<4 | q3<<6 = 0xE4, repeated.
        assert_eq!(&block.payload[..4], &[0xE4; 4]);
        assert_eq!(decode_block(&block), texels);
    }

    #[test]
    fn overhead_split_accounts_headers_and_payload() {
        // 8x8 uniform Residual4: 21 B page header + 4 x 3 B block headers
        // = 33 B metadata over 4 x 8 B payload.
        let field = fixtures::uniform(8, 8, 3);
        let page = EncodedPage::encode(&field, EncodeMode::Residual4);
        assert_eq!(page.overhead_bytes(), (33, 32));
        assert!((page.header_fraction() - 33.0 / 65.0).abs() < 1e-12);
        // Metadata share shrinks as blocks get denser: Raw8 carries more
        // payload per the same 3-byte header.
        let raw = EncodedPage::encode(&field, EncodeMode::Raw8);
        assert!(raw.header_fraction() < page.header_fraction());
    }

    #[test]
    fn validate_accepts_encoded_pages_and_rejects_structural_lies() {
        let field = fixtures::uniform(8, 8, 1);
        let mut page = EncodedPage::encode(&field, EncodeMode::Residual4);
        assert!(page.validate().is_ok());
        // Dropped block.
        page.blocks.pop();
        assert_eq!(page.validate(), Err(CodecError::InconsistentGrid));
        // Wrong grid for the extent.
        let mut page = EncodedPage::encode(&field, EncodeMode::Residual4);
        page.blocks_x = 3;
        assert_eq!(page.validate(), Err(CodecError::InconsistentGrid));
    }

    #[test]
    fn error_display_strings() {
        assert_eq!(
            CodecError::EmptyField.to_string(),
            "field extent must be non-zero"
        );
        assert!(CodecError::BadMagic.to_string().contains("MICR"));
        assert!(CodecError::UnsupportedVersion(3).to_string().contains('3'));
        assert!(CodecError::Truncated.to_string().contains("truncated"));
        assert!(CodecError::TrailingBytes(2).to_string().contains('2'));
        assert!(CodecError::BadCodecTag(9).to_string().contains('9'));
        assert!(CodecError::InconsistentGrid.to_string().contains("grid"));
        assert!(
            CodecError::BadExtent {
                width: 2,
                height: 3,
                samples: 5
            }
            .to_string()
            .contains("2x3")
        );
    }
}
