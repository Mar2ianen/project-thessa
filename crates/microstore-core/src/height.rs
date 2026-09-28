//! Height microscaling evaluation (doc §18 Phase E, §7).
//!
//! Prototype f32 codec for baked height grids: page min/max header plus
//! per-4x4-block offset/scale with adaptive 8/16-bit codes and a verbatim
//! f32 fallback. Selection is cheapest-first under an absolute error
//! budget in metres.
//!
//! The doc sets stricter rules for height than for materials, so this
//! module also ships the verifiers Phase E demands:
//!
//! - [`verify`] checks absolute error, conservative min/max bound
//!   validity, and normal angular error (normals amplify height error);
//! - [`shared_edge_error`] measures cracks where two independently
//!   encoded pages share an edge strip — the failure mode §7 forbids;
//! - real Thessa grids come from vendored field bytes (doc fixture 8),
//!   parsed hermetically with `include_bytes!` (see `tests/assets`).

use crate::codec::{BLOCK_EDGE, BLOCK_TEXELS, CodecError, blocks_for_extent};

const HEIGHT_MAGIC: [u8; 4] = *b"MICH";
const HEIGHT_VERSION: u8 = 1;
const TAG_RAW32: u8 = 0;
const TAG_RESIDUAL16: u8 = 1;
const TAG_RESIDUAL8: u8 = 2;

/// 16-bit quantizer steps.
pub const H16_STEPS: u32 = 65_535;
/// 8-bit quantizer steps.
pub const H8_STEPS: u32 = 255;

/// Payload bytes for the verbatim f32 form (16 x f32).
pub const H_PAYLOAD_RAW_LEN: usize = 64;
/// Payload bytes for the packed 16-bit form (16 x u16 LE).
pub const H_PAYLOAD_R16_LEN: usize = 32;
/// Payload bytes for the packed 8-bit form.
pub const H_PAYLOAD_R8_LEN: usize = 16;

/// One f32 height field, row-major metres.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightGrid {
    /// Texels per row.
    pub width: u32,
    /// Rows.
    pub height: u32,
    /// `width * height` samples, row-major.
    pub heights: Vec<f32>,
}

impl HeightGrid {
    /// Build a grid; rejects empty/ragged extents and non-finite samples
    /// (a NaN would silently poison every block minimum).
    pub fn new(width: u32, height: u32, heights: Vec<f32>) -> Result<Self, CodecError> {
        if width == 0 || height == 0 {
            return Err(CodecError::EmptyField);
        }
        if heights.len() != width as usize * height as usize {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: heights.len(),
            });
        }
        if heights.iter().any(|v| !v.is_finite()) {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: heights.len(),
            });
        }
        Ok(Self {
            width,
            height,
            heights,
        })
    }

    /// Sample with edge replication (pads partial edge blocks).
    pub fn sample_edge(&self, x: i64, y: i64) -> f32 {
        let x = x.clamp(0, self.width as i64 - 1) as usize;
        let y = y.clamp(0, self.height as i64 - 1) as usize;
        self.heights[y * self.width as usize + x]
    }

    /// Minimum and maximum sample.
    pub fn min_max(&self) -> (f32, f32) {
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for v in &self.heights {
            min = min.min(*v);
            max = max.max(*v);
        }
        (min, max)
    }
}

/// Height block codec tag. Wire tags are stable: `0` Raw32, `1`
/// Residual16, `2` Residual8.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightCodec {
    /// 16 verbatim f32 LE samples. Lossless baseline and pathological
    /// fallback (cliffs wider than any codebook step).
    Raw32,
    /// Block min/max f32 plus 16 packed u16 codes. Lossy.
    Residual16,
    /// Block min/max f32 plus 16 packed u8 codes. Lossy.
    Residual8,
}

impl HeightCodec {
    fn tag(self) -> u8 {
        match self {
            HeightCodec::Raw32 => TAG_RAW32,
            HeightCodec::Residual16 => TAG_RESIDUAL16,
            HeightCodec::Residual8 => TAG_RESIDUAL8,
        }
    }

    fn from_tag(tag: u8) -> Result<Self, CodecError> {
        match tag {
            TAG_RAW32 => Ok(HeightCodec::Raw32),
            TAG_RESIDUAL16 => Ok(HeightCodec::Residual16),
            TAG_RESIDUAL8 => Ok(HeightCodec::Residual8),
            other => Err(CodecError::BadCodecTag(other)),
        }
    }

    /// Wire payload length in bytes.
    pub fn payload_len(self) -> usize {
        match self {
            HeightCodec::Raw32 => H_PAYLOAD_RAW_LEN,
            HeightCodec::Residual16 => H_PAYLOAD_R16_LEN,
            HeightCodec::Residual8 => H_PAYLOAD_R8_LEN,
        }
    }

    /// Whether decode reproduces the input bit-exactly.
    pub fn is_lossless(self) -> bool {
        matches!(self, HeightCodec::Raw32)
    }
}

/// How to encode a height page.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeightMode {
    /// Every block verbatim f32.
    Raw32,
    /// Every block 16-bit packed.
    Residual16,
    /// Every block 8-bit packed.
    Residual8,
    /// Per block cheapest-first (`Residual8`, then `Residual16`) within
    /// `max_abs_error_m`, else verbatim `Raw32`. Budgets clamp like the
    /// scalar adaptive mode (negative/NaN strict, +inf all lossy).
    Adaptive {
        /// Allowed per-texel absolute error in metres.
        max_abs_error_m: f64,
    },
}

/// Cheapest-first lossy ladder for [`HeightMode::Adaptive`].
const HEIGHT_LADDER: [HeightCodec; 2] = [HeightCodec::Residual8, HeightCodec::Residual16];

/// One encoded 4x4 height block.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightBlock {
    /// Codec tag.
    pub codec: HeightCodec,
    /// Block minimum in metres.
    pub offset: f32,
    /// Block range (`max - min`) in metres; zeroed for `Raw32` so headers
    /// stay canonical.
    pub scale: f32,
    /// Payload bytes (used prefix length is [`HeightCodec::payload_len`]).
    pub payload: [u8; H_PAYLOAD_RAW_LEN],
}

/// An encoded height page with its conservative declared bounds.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightPage {
    /// True grid width.
    pub width: u32,
    /// True grid height.
    pub height: u32,
    /// Page minimum: doubles as the conservative declared lower bound.
    pub base: f32,
    /// Page maximum: doubles as the conservative declared upper bound.
    pub ceiling: f32,
    /// Blocks per row.
    pub blocks_x: u32,
    /// Block rows.
    pub blocks_y: u32,
    /// Row-major blocks.
    pub blocks: Vec<HeightBlock>,
}

fn block_samples(grid: &HeightGrid, bx: u32, by: u32) -> [f32; BLOCK_TEXELS] {
    let mut out = [0.0f32; BLOCK_TEXELS];
    for y in 0..BLOCK_EDGE {
        for x in 0..BLOCK_EDGE {
            out[(y * BLOCK_EDGE + x) as usize] =
                grid.sample_edge((bx * BLOCK_EDGE + x) as i64, (by * BLOCK_EDGE + y) as i64);
        }
    }
    out
}

fn height_min_max(texels: [f32; BLOCK_TEXELS]) -> (f32, f32) {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    for v in texels {
        min = min.min(v);
        max = max.max(v);
    }
    (min, max)
}

fn quantize_h(value: f32, min: f32, range: f32, steps: u32) -> u32 {
    if range <= 0.0 {
        return 0;
    }
    let v = (value - min).max(0.0);
    ((steps as f32 * v / range).round() as u32).min(steps)
}

fn dequantize_h(q: u32, min: f32, range: f32, steps: u32) -> f32 {
    if range <= 0.0 {
        return min;
    }
    min + (q as f32 * range) / steps as f32
}

fn encode_height_block(texels: [f32; BLOCK_TEXELS], codec: HeightCodec) -> HeightBlock {
    let (min, max) = height_min_max(texels);
    let range = max - min;
    let mut payload = [0u8; H_PAYLOAD_RAW_LEN];
    match codec {
        HeightCodec::Raw32 => {
            for (i, v) in texels.iter().enumerate() {
                payload[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            HeightBlock {
                codec,
                offset: min,
                scale: 0.0,
                payload,
            }
        }
        HeightCodec::Residual16 => {
            for (i, v) in texels.iter().enumerate() {
                let q = quantize_h(*v, min, range, H16_STEPS);
                payload[i * 2..i * 2 + 2].copy_from_slice(&(q as u16).to_le_bytes());
            }
            HeightBlock {
                codec,
                offset: min,
                scale: range,
                payload,
            }
        }
        HeightCodec::Residual8 => {
            for (i, v) in texels.iter().enumerate() {
                payload[i] = quantize_h(*v, min, range, H8_STEPS) as u8;
            }
            HeightBlock {
                codec,
                offset: min,
                scale: range,
                payload,
            }
        }
    }
}

/// Worst per-texel absolute error of one lossy codec on these samples.
fn lossy_height_error(texels: [f32; BLOCK_TEXELS], codec: HeightCodec) -> f64 {
    let steps = match codec {
        HeightCodec::Residual16 => H16_STEPS,
        HeightCodec::Residual8 => H8_STEPS,
        HeightCodec::Raw32 => return 0.0,
    };
    let (min, max) = height_min_max(texels);
    let range = max - min;
    let mut worst = 0.0f64;
    for v in texels {
        let got = dequantize_h(quantize_h(v, min, range, steps), min, range, steps);
        worst = worst.max((v - got).abs() as f64);
    }
    worst
}

fn decode_height_block(block: &HeightBlock) -> [f32; BLOCK_TEXELS] {
    let mut out = [0.0f32; BLOCK_TEXELS];
    match block.codec {
        HeightCodec::Raw32 => {
            for (i, slot) in out.iter_mut().enumerate() {
                *slot = f32::from_le_bytes(
                    block.payload[i * 4..i * 4 + 4].try_into().expect("4 bytes"),
                );
            }
        }
        HeightCodec::Residual16 => {
            for (i, slot) in out.iter_mut().enumerate() {
                let q = u16::from_le_bytes(
                    block.payload[i * 2..i * 2 + 2].try_into().expect("2 bytes"),
                ) as u32;
                *slot = dequantize_h(q, block.offset, block.scale, H16_STEPS);
            }
        }
        HeightCodec::Residual8 => {
            for (i, slot) in out.iter_mut().enumerate() {
                *slot = dequantize_h(block.payload[i] as u32, block.offset, block.scale, H8_STEPS);
            }
        }
    }
    out
}

impl HeightPage {
    /// Encode a grid with the given mode.
    pub fn encode(grid: &HeightGrid, mode: HeightMode) -> Self {
        let (base, ceiling) = grid.min_max();
        let blocks_x = blocks_for_extent(grid.width);
        let blocks_y = blocks_for_extent(grid.height);
        let mut blocks = Vec::with_capacity((blocks_x * blocks_y) as usize);
        for by in 0..blocks_y {
            for bx in 0..blocks_x {
                let texels = block_samples(grid, bx, by);
                let block = match mode {
                    HeightMode::Raw32 => encode_height_block(texels, HeightCodec::Raw32),
                    HeightMode::Residual16 => encode_height_block(texels, HeightCodec::Residual16),
                    HeightMode::Residual8 => encode_height_block(texels, HeightCodec::Residual8),
                    HeightMode::Adaptive { max_abs_error_m } => {
                        let budget = max_abs_error_m.max(0.0);
                        let mut pick = None;
                        for codec in HEIGHT_LADDER {
                            if lossy_height_error(texels, codec) <= budget {
                                pick = Some(codec);
                                break;
                            }
                        }
                        encode_height_block(texels, pick.unwrap_or(HeightCodec::Raw32))
                    }
                };
                blocks.push(block);
            }
        }
        Self {
            width: grid.width,
            height: grid.height,
            base,
            ceiling,
            blocks_x,
            blocks_y,
            blocks,
        }
    }

    /// Decode to the true grid extent (edge padding is cropped).
    pub fn decode(&self) -> HeightGrid {
        let mut heights = vec![0.0f32; self.width as usize * self.height as usize];
        for (index, block) in self.blocks.iter().enumerate() {
            let bx = index as u32 % self.blocks_x;
            let by = index as u32 / self.blocks_x;
            let texels = decode_height_block(block);
            for y in 0..BLOCK_EDGE {
                for x in 0..BLOCK_EDGE {
                    let fx = bx * BLOCK_EDGE + x;
                    let fy = by * BLOCK_EDGE + y;
                    if fx < self.width && fy < self.height {
                        heights[(fy * self.width + fx) as usize] =
                            texels[(y * BLOCK_EDGE + x) as usize];
                    }
                }
            }
        }
        HeightGrid {
            width: self.width,
            height: self.height,
            heights,
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

    /// Bytes per texel of the true extent.
    pub fn bytes_per_texel(&self) -> f64 {
        self.encoded_bytes() as f64 / (self.width as f64 * self.height as f64)
    }

    /// Codec histogram, indexable by wire tag (`[Raw32, R16, R8]`).
    pub fn codec_histogram(&self) -> [usize; 3] {
        let mut hist = [0usize; 3];
        for block in &self.blocks {
            hist[block.codec.tag() as usize] += 1;
        }
        hist
    }

    /// Whether every border block is verbatim. Only then can a neighbor
    /// page sharing an edge decode it identically: lossy border blocks
    /// quantize shared samples under different references and crack.
    /// Interior lossy blocks do not affect shared edges.
    pub fn border_is_lossless(&self) -> bool {
        for (index, block) in self.blocks.iter().enumerate() {
            let bx = index as u32 % self.blocks_x;
            let by = index as u32 / self.blocks_x;
            let border = bx == 0 || by == 0 || bx + 1 == self.blocks_x || by + 1 == self.blocks_y;
            if border && block.codec != HeightCodec::Raw32 {
                return false;
            }
        }
        true
    }

    const HEADER_LEN: usize = 4 + 1 + 4 + 4 + 4 + 4 + 4 + 4;
    const BLOCK_HEADER_LEN: usize = 1 + 4 + 4;

    /// Deterministic little-endian wire encoding.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.encoded_bytes());
        out.extend_from_slice(&HEIGHT_MAGIC);
        out.push(HEIGHT_VERSION);
        for v in [self.width, self.height, self.blocks_x, self.blocks_y] {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.extend_from_slice(&self.base.to_le_bytes());
        out.extend_from_slice(&self.ceiling.to_le_bytes());
        for block in &self.blocks {
            out.push(block.codec.tag());
            out.extend_from_slice(&block.offset.to_le_bytes());
            out.extend_from_slice(&block.scale.to_le_bytes());
            out.extend_from_slice(&block.payload[..block.codec.payload_len()]);
        }
        out
    }

    /// Strict parse of [`HeightPage::to_bytes`] output.
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
        if take(&mut cursor, 4)? != HEIGHT_MAGIC {
            return Err(CodecError::BadMagic);
        }
        if take(&mut cursor, 1)?[0] != HEIGHT_VERSION {
            return Err(CodecError::UnsupportedVersion(
                bytes.get(4).copied().unwrap_or(255),
            ));
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
        let f32_of = |raw: Vec<u8>| f32::from_le_bytes(raw.try_into().expect("4 bytes"));
        let base = f32_of(take(&mut cursor, 4)?);
        let ceiling = f32_of(take(&mut cursor, 4)?);
        if !base.is_finite() || !ceiling.is_finite() || ceiling < base {
            return Err(CodecError::BadExtent {
                width,
                height,
                samples: 0,
            });
        }
        let count = blocks_x as u64 * blocks_y as u64;
        // Same allocation guard as the scalar parser: every block needs
        // at least its 9-byte header plus the smallest payload (16 bytes,
        // Residual8). Refuse first, allocate after.
        let min_needed = count
            .checked_mul((Self::BLOCK_HEADER_LEN + H_PAYLOAD_R8_LEN) as u64)
            .and_then(|payload| payload.checked_add(Self::HEADER_LEN as u64));
        if min_needed.is_none_or(|need| (bytes.len() as u64) < need) {
            return Err(CodecError::Truncated);
        }
        let mut blocks = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let head = take(&mut cursor, Self::BLOCK_HEADER_LEN)?;
            let codec = HeightCodec::from_tag(head[0])?;
            let offset = f32_of(head[1..5].to_vec());
            let scale = f32_of(head[5..9].to_vec());
            if !offset.is_finite() || !scale.is_finite() || scale < 0.0 {
                return Err(CodecError::NonFiniteFloat);
            }
            // The block range must sit inside the page-declared bounds.
            // `offset + scale` gets an f32 rounding slack: honest encoders
            // compute both from the same samples, but the addition can
            // round one ulp past the ceiling.
            let slack = 8.0 * f32::EPSILON * base.abs().max(ceiling.abs()).max(1.0);
            if offset < base || offset + scale > ceiling + slack {
                return Err(CodecError::OutOfBoundsSample);
            }
            let payload = take(&mut cursor, codec.payload_len())?;
            if codec == HeightCodec::Raw32 {
                // Verbatim floats are trusted bytes: reject NaN/Inf and
                // anything outside the declared conservative bounds, so a
                // strict parse never smuggles non-geometry past `verify`.
                for sample in payload.as_chunks::<4>().0 {
                    let v = f32_of(sample.to_vec());
                    if !v.is_finite() {
                        return Err(CodecError::NonFiniteFloat);
                    }
                    if v < base || v > ceiling {
                        return Err(CodecError::OutOfBoundsSample);
                    }
                }
            }
            let mut raw = [0u8; H_PAYLOAD_RAW_LEN];
            raw[..payload.len()].copy_from_slice(&payload);
            blocks.push(HeightBlock {
                codec,
                offset,
                scale,
                payload: raw,
            });
        }
        if !cursor.is_empty() {
            return Err(CodecError::TrailingBytes(cursor.len()));
        }
        Ok(Self {
            width,
            height,
            base,
            ceiling,
            blocks_x,
            blocks_y,
            blocks,
        })
    }
}

/// Phase E verification summary for one decoded page.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeightVerify {
    /// Worst `|decoded - original|` in metres.
    pub max_abs_error_m: f64,
    /// Worst normal angle between decoded and original grids in degrees,
    /// by central differences on interior texels.
    pub normal_max_angle_deg: f64,
    /// Conservative bound slack in metres: how far decoded samples escape
    /// `[declared_min - 0, declared_max + 0]` outward. Must stay within
    /// the measured error for the declared bounds to remain valid.
    pub bound_slack_m: f64,
}

/// Angle between central-difference normals in degrees. Unit-texel spacing
/// on both sides, so the comparison is spacing-invariant.
fn normal_angle(a: [f32; 3], b: [f32; 3]) -> f64 {
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) as f64;
    let na = ((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) as f64).sqrt();
    let nb = ((b[0] * b[0] + b[1] * b[1] + b[2] * b[2]) as f64).sqrt();
    if na == 0.0 || nb == 0.0 {
        return 0.0;
    }
    (dot / (na * nb)).clamp(-1.0, 1.0).acos().to_degrees()
}

fn grid_normal(h: &[f32], width: u32, x: u32, y: u32, sx_m: f32, sy_m: f32) -> [f32; 3] {
    let w = width as usize;
    let at = |xx: u32, yy: u32| h[yy as usize * w + xx as usize];
    // Physical slopes: height delta over metres, not texels. Unit-texel
    // spacing would silently claim metres-wide texels are 1 m wide and
    // inflate every angle on real pages.
    let dx = (at(x + 1, y) - at(x - 1, y)) / (2.0 * sx_m);
    let dy = (at(x, y + 1) - at(x, y - 1)) / (2.0 * sy_m);
    [-dx, -dy, 1.0]
}

/// Texel spacing in metres for an equirectangular lat/lon window:
/// `[x_spacing, y_spacing]`. Longitude spacing shrinks with latitude;
/// callers must pass the window's own geometry (see the fixture README
/// for the vendored windows).
pub fn latlon_window_spacing_m(
    center_lat_deg: f64,
    span_deg: f64,
    samples: u32,
    radius_m: f64,
) -> [f32; 2] {
    assert!(samples >= 2, "spacing needs at least two samples");
    let span_m = span_deg.to_radians() * radius_m / (samples - 1) as f64;
    [
        (span_m * center_lat_deg.to_radians().cos()) as f32,
        span_m as f32,
    ]
}

/// Verify a decoded grid against the original and the page-declared
/// conservative bounds, with physical texel spacing in metres
/// (`[x, y]`, longitude/latitude for lat/lon windows).
pub fn verify(
    original: &HeightGrid,
    decoded: &HeightGrid,
    declared_min: f32,
    declared_max: f32,
    spacing_m: [f32; 2],
) -> HeightVerify {
    assert_eq!(
        (original.width, original.height),
        (decoded.width, decoded.height),
        "height verify needs identical extents"
    );
    assert!(
        spacing_m[0] > 0.0
            && spacing_m[1] > 0.0
            && spacing_m[0].is_finite()
            && spacing_m[1].is_finite(),
        "physical texel spacing must be positive and finite"
    );
    let mut worst = 0.0f64;
    let mut slack = 0.0f64;
    for (a, b) in original.heights.iter().zip(decoded.heights.iter()) {
        worst = worst.max((a - b).abs() as f64);
        slack = slack.max((declared_min - *b).max(0.0) as f64);
        slack = slack.max((*b - declared_max).max(0.0) as f64);
    }
    // No interior texels on degenerate grids: nothing to angle.
    let mut angle = 0.0f64;
    if original.width >= 3 && original.height >= 3 {
        for y in 1..original.height - 1 {
            for x in 1..original.width - 1 {
                let a = grid_normal(
                    &original.heights,
                    original.width,
                    x,
                    y,
                    spacing_m[0],
                    spacing_m[1],
                );
                let b = grid_normal(
                    &decoded.heights,
                    decoded.width,
                    x,
                    y,
                    spacing_m[0],
                    spacing_m[1],
                );
                angle = angle.max(normal_angle(a, b));
            }
        }
    }
    HeightVerify {
        max_abs_error_m: worst,
        normal_max_angle_deg: angle,
        bound_slack_m: slack,
    }
}

/// Crack metric for one shared edge strip: max `|a - b|` over two
/// equal-length sample runs decoded from independently encoded pages.
/// This is the §7 page-boundary test: the two sides share source samples
/// but were quantized under different block references.
pub fn shared_edge_error(a: &[f32], b: &[f32]) -> f64 {
    assert_eq!(a.len(), b.len(), "shared edge strips must match");
    assert!(!a.is_empty(), "shared edge strip must be non-empty");
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).abs() as f64)
        .fold(0.0f64, f64::max)
}

// ---------------------------------------------------------------------------
// Fixture 8: vendored real Thessa bytes (doc §16 item 8).
// ---------------------------------------------------------------------------

fn parse_f32_grid(bytes: &[u8]) -> HeightGrid {
    let err = "vendored height asset is corrupt";
    assert!(bytes.len() >= 8, "{err}");
    let w = u32::from_le_bytes(bytes[0..4].try_into().expect(err));
    let h = u32::from_le_bytes(bytes[4..8].try_into().expect(err));
    let body = &bytes[8..];
    assert_eq!(body.len(), w as usize * h as usize * 4, "{err}");
    let heights = body
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| f32::from_le_bytes(*c))
        .collect();
    HeightGrid::new(w, h, heights).expect(err)
}

/// Real Thessa deep-ocean height window, 65x65 f32 metres.
pub fn thessa_height_ocean() -> HeightGrid {
    parse_f32_grid(include_bytes!(
        "../tests/assets/thessa_height_ocean_65x65.f32"
    ))
}

/// Real Thessa coast height window, 65x65 f32 metres.
pub fn thessa_height_coast() -> HeightGrid {
    parse_f32_grid(include_bytes!(
        "../tests/assets/thessa_height_coast_65x65.f32"
    ))
}

/// Real Thessa mountain height window, 65x65 f32 metres.
pub fn thessa_height_mountain() -> HeightGrid {
    parse_f32_grid(include_bytes!(
        "../tests/assets/thessa_height_mountain_65x65.f32"
    ))
}

fn parse_u8_planes(bytes: &[u8], channels: usize) -> Vec<crate::ScalarField> {
    let err = "vendored material asset is corrupt";
    assert!(bytes.len() >= 8, "{err}");
    let w = u32::from_le_bytes(bytes[0..4].try_into().expect(err));
    let h = u32::from_le_bytes(bytes[4..8].try_into().expect(err));
    let body = &bytes[8..];
    assert_eq!(body.len(), w as usize * h as usize * channels, "{err}");
    (0..channels)
        .map(|c| {
            let data = body
                .chunks_exact(channels)
                .map(|px| px[c])
                .collect::<Vec<_>>();
            crate::ScalarField::new(w, h, data).expect(err)
        })
        .collect()
}

/// Real Thessa coast albedo planes (sRGB bytes), 128x128 R/G/B.
pub fn thessa_albedo_coast() -> [crate::ScalarField; 3] {
    let planes = parse_u8_planes(
        include_bytes!("../tests/assets/thessa_albedo_coast_128x128.rgb"),
        3,
    );
    [planes[0].clone(), planes[1].clone(), planes[2].clone()]
}

/// Real Thessa coast roughness plane (linear bytes), 128x128.
pub fn thessa_rough_coast() -> crate::ScalarField {
    parse_u8_planes(
        include_bytes!("../tests/assets/thessa_rough_coast_128x128.r8"),
        1,
    )
    .into_iter()
    .next()
    .expect("one roughness plane")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::EncodeMode;

    fn real_grids() -> Vec<(&'static str, HeightGrid)> {
        vec![
            ("ocean", thessa_height_ocean()),
            ("coast", thessa_height_coast()),
            ("mountain", thessa_height_mountain()),
        ]
    }

    #[test]
    fn vendored_assets_parse_with_documented_shapes() {
        for (name, grid) in real_grids() {
            assert_eq!((grid.width, grid.height), (65, 65), "{name}");
            assert!(grid.heights.iter().all(|v| v.is_finite()));
        }
        let (omin, omax) = thessa_height_ocean().min_max();
        assert!(omin < -1000.0 && omax < -500.0, "ocean {omin}..{omax}");
        let (mmin, mmax) = thessa_height_mountain().min_max();
        // The mountain window bottoms out on the recipe datum floor.
        assert!(mmin <= -7999.0, "datum floor {mmin}");
        assert!(mmax > 3000.0, "relief {mmax}");
        for plane in thessa_albedo_coast() {
            assert_eq!((plane.width, plane.height), (128, 128));
        }
        assert_eq!(
            (thessa_rough_coast().width, thessa_rough_coast().height),
            (128, 128)
        );
    }

    #[test]
    fn grid_rejects_empty_ragged_and_nonfinite() {
        assert_eq!(
            HeightGrid::new(0, 4, vec![0.0; 8]),
            Err(CodecError::EmptyField)
        );
        assert_eq!(
            HeightGrid::new(4, 4, vec![0.0; 15]),
            Err(CodecError::BadExtent {
                width: 4,
                height: 4,
                samples: 15
            })
        );
        assert!(HeightGrid::new(2, 2, vec![0.0, 1.0, f32::NAN, 3.0]).is_err());
        assert!(HeightGrid::new(2, 2, vec![0.0, 1.0, 2.0, f32::INFINITY]).is_err());
        assert!(HeightGrid::new(2, 2, vec![0.0, 1.0, 2.0, 3.0]).is_ok());
    }

    #[test]
    fn raw32_round_trips_real_grids_bit_exact() {
        for (name, grid) in real_grids() {
            let page = HeightPage::encode(&grid, HeightMode::Raw32);
            assert_eq!(page.decode(), grid, "{name}");
            assert_eq!(page.base, grid.min_max().0);
            assert_eq!(page.ceiling, grid.min_max().1);
        }
    }

    /// Physical texel spacing for a real window (see tests/assets
    /// README for coordinates): 2x2 deg windows at 65 samples on a
    /// 3200 km datum.
    fn spacing(name: &str) -> [f32; 2] {
        let lat = match name {
            "ocean" => -60.0,
            "coast" => -60.0,
            "mountain" => -54.0,
            _ => panic!("known fixture window"),
        };
        latlon_window_spacing_m(lat, 2.0, 65, 3_200_000.0)
    }

    #[test]
    fn window_spacing_math() {
        // Equator: isotropic. At -60 deg longitude halves.
        let eq = latlon_window_spacing_m(0.0, 2.0, 65, 3_200_000.0);
        assert!((eq[0] - eq[1]).abs() < 1e-3);
        assert!((eq[1] - 1745.3).abs() < 0.5, "{eq:?}");
        let s60 = latlon_window_spacing_m(-60.0, 2.0, 65, 3_200_000.0);
        assert!((s60[0] - eq[0] * 0.5).abs() < 0.5, "{s60:?}");
        assert!((s60[1] - eq[1]).abs() < 1e-3);
    }

    #[test]
    #[should_panic(expected = "at least two samples")]
    fn window_spacing_rejects_single_sample() {
        latlon_window_spacing_m(0.0, 2.0, 1, 3_200_000.0);
    }

    #[test]
    fn verify_rejects_nonphysical_spacing() {
        let grid = thessa_height_coast();
        let back = grid.clone();
        for spacing in [[0.0, 1745.0], [872.0, -1.0], [f32::NAN, 1.0]] {
            let r = std::panic::catch_unwind(|| verify(&grid, &back, 0.0, 1.0, spacing));
            assert!(r.is_err(), "{spacing:?}");
        }
    }

    #[test]
    fn degenerate_grids_have_no_interior_angle() {
        // 1x1 and 2x2 grids have no interior texels: must not underflow,
        // angle is defined as zero.
        for (w, h) in [(1, 1), (2, 2), (1, 5), (5, 1)] {
            let grid = HeightGrid::new(w, h, vec![10.0; w as usize * h as usize]).unwrap();
            let v = verify(&grid, &grid, 10.0, 10.0, [100.0, 100.0]);
            assert_eq!(v.normal_max_angle_deg, 0.0);
            assert_eq!(v.max_abs_error_m, 0.0);
        }
    }

    #[test]
    fn adaptive_respects_budget_on_real_grids() {
        for (name, grid) in real_grids() {
            for budget in [1.0, 10.0, 100.0] {
                let page = HeightPage::encode(
                    &grid,
                    HeightMode::Adaptive {
                        max_abs_error_m: budget,
                    },
                );
                let back = page.decode();
                let v = verify(&grid, &back, page.base, page.ceiling, spacing(name));
                assert!(
                    v.max_abs_error_m <= budget,
                    "{name}@{budget}: {}",
                    v.max_abs_error_m
                );
                assert!(
                    v.bound_slack_m <= v.max_abs_error_m + 1e-3,
                    "{name}: slack {} > err {}",
                    v.bound_slack_m,
                    v.max_abs_error_m
                );
            }
        }
    }

    #[test]
    fn residual16_bound_holds_on_mountain() {
        // 16-bit over a ~14 km range: half-step ~0.11 m plus f32 rounding.
        let grid = thessa_height_mountain();
        let page = HeightPage::encode(&grid, HeightMode::Residual16);
        let v = verify(
            &grid,
            &page.decode(),
            page.base,
            page.ceiling,
            spacing("mountain"),
        );
        assert!(v.max_abs_error_m <= 0.5, "r16 {}", v.max_abs_error_m);
        assert!(
            v.normal_max_angle_deg <= 0.05,
            "r16 angle {}",
            v.normal_max_angle_deg
        );
    }

    #[test]
    fn normal_error_stays_small_on_coast() {
        let grid = thessa_height_coast();
        let page = HeightPage::encode(
            &grid,
            HeightMode::Adaptive {
                max_abs_error_m: 10.0,
            },
        );
        let v = verify(
            &grid,
            &page.decode(),
            page.base,
            page.ceiling,
            spacing("coast"),
        );
        assert!(
            v.normal_max_angle_deg <= 0.5,
            "angle {}",
            v.normal_max_angle_deg
        );
    }

    #[test]
    fn independent_lossy_pages_crack_on_shared_edges() {
        // Split the coast grid into left/right pages sharing column 32 and
        // encode separately: the shared samples land in blocks with
        // different offset/scale references, so the edge diverges. This
        // test pins the phenomenon (it is nonzero), not an acceptance
        // bound: a nonzero crack FAILS the geometry bar, which is why
        // only border-lossless pages qualify as geometry sources.
        let grid = thessa_height_coast();
        let split = |lo: usize| {
            HeightGrid::new(
                33,
                65,
                grid.heights
                    .as_chunks::<65>()
                    .0
                    .iter()
                    .flat_map(|row| row[lo..lo + 33].to_vec())
                    .collect(),
            )
            .unwrap()
        };
        let (left, right) = (split(0), split(32));
        let mode = HeightMode::Adaptive {
            max_abs_error_m: 10.0,
        };
        let a = HeightPage::encode(&left, mode).decode();
        let b = HeightPage::encode(&right, mode).decode();
        let edge = |h: &[f32], col: usize| {
            h.as_chunks::<33>()
                .0
                .iter()
                .map(|row| row[col])
                .collect::<Vec<_>>()
        };
        let crack = shared_edge_error(&edge(&a.heights, 32), &edge(&b.heights, 0));
        assert!(crack > 0.0, "independent references must diverge");
        assert!(crack <= 20.0 + 1e-3, "runaway crack {crack}");
    }

    #[test]
    fn lossless_pages_share_exact_edges() {
        // Sufficiency direction: verbatim pages decode shared samples
        // bit-exactly, so the crack is exactly zero.
        let grid = thessa_height_coast();
        let split = |lo: usize| {
            HeightGrid::new(
                33,
                65,
                grid.heights
                    .as_chunks::<65>()
                    .0
                    .iter()
                    .flat_map(|row| row[lo..lo + 33].to_vec())
                    .collect(),
            )
            .unwrap()
        };
        let (left, right) = (split(0), split(32));
        let a = HeightPage::encode(&left, HeightMode::Raw32).decode();
        let b = HeightPage::encode(&right, HeightMode::Raw32).decode();
        let edge = |h: &[f32], col: usize| {
            h.as_chunks::<33>()
                .0
                .iter()
                .map(|row| row[col])
                .collect::<Vec<_>>()
        };
        assert_eq!(
            shared_edge_error(&edge(&a.heights, 32), &edge(&b.heights, 0)),
            0.0
        );
    }

    #[test]
    fn border_lossless_flags_geometry_suitability() {
        let grid = thessa_height_coast();
        let raw = HeightPage::encode(&grid, HeightMode::Raw32);
        assert!(raw.border_is_lossless());
        let adaptive = HeightPage::encode(
            &grid,
            HeightMode::Adaptive {
                max_abs_error_m: 10.0,
            },
        );
        // Real coast data puts lossy blocks on the border.
        assert!(!adaptive.border_is_lossless());
        assert_eq!(adaptive.codec_histogram().iter().sum::<usize>(), 289);
    }

    #[test]
    fn constant_grid_is_exact_and_compact() {
        let grid = HeightGrid::new(16, 16, vec![-1234.5; 256]).unwrap();
        let page = HeightPage::encode(&grid, HeightMode::Residual8);
        assert_eq!(page.codec_histogram(), [0, 0, 16]);
        assert_eq!(page.decode(), grid);
    }

    #[test]
    fn determinism_and_wire_round_trip() {
        for (_, grid) in real_grids() {
            for mode in [
                HeightMode::Raw32,
                HeightMode::Residual16,
                HeightMode::Residual8,
                HeightMode::Adaptive {
                    max_abs_error_m: 5.0,
                },
            ] {
                let a = HeightPage::encode(&grid, mode).to_bytes();
                let b = HeightPage::encode(&grid, mode).to_bytes();
                assert_eq!(a, b);
                let back = HeightPage::from_bytes(&a).expect("wire");
                assert_eq!(back.decode(), HeightPage::encode(&grid, mode).decode());
            }
        }
    }

    #[test]
    fn absurd_dimensions_fail_cleanly_without_allocating() {
        // Same allocation guard as the scalar parser, with the taller
        // height header (29 bytes) and minimum block (25 bytes).
        let mut header = Vec::new();
        header.extend_from_slice(b"MICH");
        header.push(1);
        for v in [u32::MAX, u32::MAX, 1 << 30, 1 << 30] {
            header.extend_from_slice(&v.to_le_bytes());
        }
        header.extend_from_slice(&0.0f32.to_le_bytes());
        header.extend_from_slice(&1.0f32.to_le_bytes());
        assert_eq!(HeightPage::from_bytes(&header), Err(CodecError::Truncated));
    }

    #[test]
    fn raw32_rejects_nonfinite_and_out_of_bounds_samples() {
        let grid = HeightGrid::new(8, 8, vec![100.0; 64]).unwrap();
        let page = HeightPage::encode(&grid, HeightMode::Raw32);
        let (min, max) = (page.base, page.ceiling);
        // NaN payload.
        let mut bad = page.to_bytes();
        bad[29 + 9 + 4..29 + 9 + 8].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::NonFiniteFloat)
        );
        // Infinite payload.
        let mut bad = page.to_bytes();
        bad[29 + 9..29 + 9 + 4].copy_from_slice(&f32::INFINITY.to_le_bytes());
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::NonFiniteFloat)
        );
        // Finite but outside the declared conservative bounds.
        let mut bad = page.to_bytes();
        bad[29 + 9..29 + 9 + 4].copy_from_slice(&(max + 100.0).to_le_bytes());
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::OutOfBoundsSample)
        );
        let _ = min;
    }

    #[test]
    fn block_range_escaping_bounds_is_rejected() {
        let grid = HeightGrid::new(8, 8, vec![100.0; 64]).unwrap();
        let page = HeightPage::encode(&grid, HeightMode::Residual16);
        // Inflate the first block's scale far past the page ceiling.
        let mut bad = page.to_bytes();
        bad[29 + 5..29 + 9].copy_from_slice(&1.0e9f32.to_le_bytes());
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::OutOfBoundsSample)
        );
    }

    #[test]
    fn wire_rejections() {
        let grid = thessa_height_ocean();
        let page = HeightPage::encode(&grid, HeightMode::Residual16);
        let bytes = page.to_bytes();
        let mut bad = bytes.clone();
        bad[0] = b'X';
        assert_eq!(HeightPage::from_bytes(&bad), Err(CodecError::BadMagic));
        let mut bad = bytes.clone();
        bad[4] = 7;
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::UnsupportedVersion(7))
        );
        let mut bad = bytes.clone();
        bad[29] = 9;
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::BadCodecTag(9))
        );
        assert_eq!(
            HeightPage::from_bytes(&bytes[..bytes.len() - 1]),
            Err(CodecError::Truncated)
        );
        let mut bad = bytes.clone();
        bad.push(0);
        assert_eq!(
            HeightPage::from_bytes(&bad),
            Err(CodecError::TrailingBytes(1))
        );
        // Ceiling below base.
        let mut bad = bytes.clone();
        bad[21..25].copy_from_slice(&f32::to_le_bytes(1.0e9));
        assert!(HeightPage::from_bytes(&bad).is_err());
    }

    #[test]
    fn real_material_planes_encode() {
        let rgb = thessa_albedo_coast();
        let field = crate::ColorField::new(rgb.clone()).expect("rgb planes");
        let page = crate::ColorPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
        let linear = crate::measure_linear(&field.channels, &page.decode());
        assert!(linear.max_abs <= 0.05, "real albedo {}", linear.max_abs);
        let rough = thessa_rough_coast();
        let rpage = crate::EncodedPage::encode(&rough, EncodeMode::Adaptive { max_abs_error: 2.0 });
        let stats = crate::measure(&rough, &rpage.decode());
        assert!(stats.max_abs <= 2.0);
    }

    #[test]
    fn shared_edge_helper_contract() {
        assert_eq!(shared_edge_error(&[1.0, 2.0], &[1.5, 1.0]), 1.0);
        assert_eq!(shared_edge_error(&[3.0], &[3.0]), 0.0);
    }

    #[test]
    #[should_panic(expected = "must match")]
    fn shared_edge_rejects_mismatched_strips() {
        shared_edge_error(&[1.0], &[1.0, 2.0]);
    }

    #[test]
    fn verify_bounds_and_angles_on_identity() {
        let grid = thessa_height_coast();
        let (min, max) = grid.min_max();
        let v = verify(&grid, &grid, min, max, spacing("coast"));
        assert_eq!(v.max_abs_error_m, 0.0);
        // Identical f32 normals agree up to dot-product rounding.
        assert!(v.normal_max_angle_deg <= 1e-4, "{}", v.normal_max_angle_deg);
        assert_eq!(v.bound_slack_m, 0.0);
    }
}
