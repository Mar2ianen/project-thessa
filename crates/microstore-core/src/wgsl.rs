//! Sample-time WGSL decoder for microscaled pages (doc §18 Phase B).
//!
//! This module is backend-neutral text plus CPU-side upload helpers: the
//! WGSL mirrors [`crate::codec`] decode bit-exactly with integer math, so a
//! shader sample and the CPU reference decoder agree on every texel.
//!
//! Binding layout for the `decode_page` entry point:
//!
//! ```text
//! 0: uniform  Params { width, height, blocks_x, blocks_y }
//! 1: storage  page wire bytes as u32 words (zero-padded to a multiple
//!              of 4; see [`padded_upload_bytes`])
//! 2: storage  per-block header byte offsets, row-major u32
//!              (see [`block_base_table`])
//! 3: storage  decoded texel bytes as u32, row-major, read_write
//! ```
//!
//! One thread decodes one texel independently — exactly the sample-time
//! decode shape a material shader will use later. Variable-length blocks
//! are resolved through the CPU-built offset table (binding 2) instead of
//! a per-thread header walk.

use crate::codec::EncodedPage;

/// Portable sample-time decode kernel. One thread per texel, workgroup 64,
/// only plain integer shift/mask/multiply (doc §15 baseline).
pub const MICROSTORE_DECODE_WGSL: &str = r#"
struct Params {
    width: u32,
    height: u32,
    blocks_x: u32,
    blocks_y: u32,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> page_words: array<u32>;
@group(0) @binding(2) var<storage, read> block_base: array<u32>;
@group(0) @binding(3) var<storage, read_write> out_texels: array<u32>;

fn load_byte(byte_index: u32) -> u32 {
    return (page_words[byte_index / 4u] >> ((byte_index % 4u) * 8u)) & 0xFFu;
}

@compute @workgroup_size(64)
fn decode_page(@builtin(global_invocation_id) gid: vec3<u32>) {
    let texel = gid.x;
    let count = params.width * params.height;
    if (texel >= count) {
        return;
    }
    let tx = texel % params.width;
    let ty = texel / params.width;
    let bx = tx / 4u;
    let by = ty / 4u;
    let lx = tx % 4u;
    let ly = ty % 4u;
    let block = by * params.blocks_x + bx;
    let base = block_base[block];
    let tag = load_byte(base);
    if (tag > 4u) {
        // Unreachable through the CPU parser (tags 0..=4 only); a visual
        // corruption sentinel instead of silent garbage.
        out_texels[texel] = 0xDEADu;
        return;
    }
    let offset = load_byte(base + 1u);
    var value = 0u;
    if (tag == 0u) {
        // Raw8: verbatim payload after the 3-byte header.
        value = load_byte(base + 3u + ly * 4u + lx);
    } else if (tag == 1u) {
        // Residual8: offset + stored (value - offset).
        value = offset + load_byte(base + 3u + ly * 4u + lx);
    } else if (tag == 2u) {
        // Residual4: offset + round(q * scale / 15). Integer math
        // mirrors the CPU reference decoder exactly.
        let scale = load_byte(base + 2u);
        let cell = ly * 4u + lx;
        let packed = load_byte(base + 3u + cell / 2u);
        var q = packed & 15u;
        if (cell % 2u == 1u) {
            q = packed >> 4u;
        }
        value = offset + (q * scale + 7u) / 15u;
    } else if (tag == 3u) {
        // Residual6: 12 payload bytes, texel i occupies bits [6i, 6i+6)
        // of the little-endian bit stream.
        let scale = load_byte(base + 2u);
        let cell = ly * 4u + lx;
        let bit = cell * 6u;
        let byte = bit / 8u;
        let shift = bit % 8u;
        let lo = load_byte(base + 3u + byte);
        let hi = load_byte(base + 3u + byte + 1u);
        let q = ((lo >> shift) | (hi << (8u - shift))) & 63u;
        value = offset + (q * scale + 31u) / 63u;
    } else {
        // Residual2: 4 payload bytes, texel i occupies bits [2i, 2i+2).
        let scale = load_byte(base + 2u);
        let cell = ly * 4u + lx;
        let packed = load_byte(base + 3u + cell / 4u);
        let q = (packed >> ((cell % 4u) * 2u)) & 3u;
        value = offset + (q * scale + 1u) / 3u;
    }
    out_texels[texel] = min(value, 255u);
}
"#;

/// Byte offset of every block header in [`EncodedPage::to_bytes`] output,
/// row-major. Lets each decoder thread jump straight to its block instead
/// of walking variable-length headers.
pub fn block_base_table(page: &EncodedPage) -> Vec<u32> {
    // Page header: 4 magic + 1 version + 4 u32 dims.
    let mut base = 4 + 1 + 4 * 4;
    let mut table = Vec::with_capacity(page.blocks.len());
    for block in &page.blocks {
        table.push(base as u32);
        base += 3 + block.codec.payload_len();
    }
    table
}

/// Wire bytes zero-padded to a u32 multiple for `array<u32>` upload.
/// Returns the padded bytes.
///
/// Note for uploaders: the Residual6 path reads the byte after the last
/// payload byte for the final code, so backends must additionally
/// guarantee slack past this buffer (the wgpu backend appends one zero
/// word). The padded length itself stays exact here.
pub fn padded_upload_bytes(page: &EncodedPage) -> Vec<u8> {
    let mut bytes = page.to_bytes();
    while !bytes.len().is_multiple_of(4) {
        bytes.push(0);
    }
    bytes
}

/// Exact upload accounting for one page (doc §9 telemetry): wire bytes,
/// offset-table bytes, and 16 params bytes. The padded page buffer is what
/// the GPU allocation must fit.
///
/// Three currencies are deliberately separate (they answer different
/// questions, and mixing them overstates wins):
///
/// - `wire_bytes`: canonical `to_bytes` length (residency payload);
///
/// - [`MicrostoreUpload::gpu_upload_bytes`]: padded page + offset table +
///   params actually transferred;
///
/// - [`MicrostoreUpload::gpu_resident_bytes`]: upload plus the
///   u32-per-texel decode target a sampler would read.
pub struct MicrostoreUpload {
    /// `to_bytes` length (residency-relevant payload).
    pub wire_bytes: usize,
    /// Padded buffer size actually uploaded.
    pub padded_bytes: usize,
    /// Offset-table buffer size (`4 * block count`).
    pub table_bytes: usize,
    /// Uniform params buffer size (always 16).
    pub params_bytes: usize,
    /// Decode-target size (`4 * texels`, u32 per texel).
    pub expanded_bytes: usize,
}

impl MicrostoreUpload {
    /// Bytes transferred for one page: padded wire + table + params.
    pub fn gpu_upload_bytes(&self) -> usize {
        self.padded_bytes + self.table_bytes + self.params_bytes
    }

    /// Bytes resident for one decoded page: upload plus decode target.
    pub fn gpu_resident_bytes(&self) -> usize {
        self.gpu_upload_bytes() + self.expanded_bytes
    }
}

/// Upload accounting for a page without touching a GPU.
pub fn upload_size(page: &EncodedPage) -> MicrostoreUpload {
    let wire_bytes = page.encoded_bytes();
    MicrostoreUpload {
        wire_bytes,
        padded_bytes: wire_bytes.div_ceil(4) * 4,
        table_bytes: page.blocks.len() * 4,
        params_bytes: 16,
        expanded_bytes: page.width as usize * page.height as usize * 4,
    }
}

/// Box-downsample kernel over u32-per-texel buffers: one thread per
/// destination texel averages a clamped 2x2 quad with `(a+b+c+d+2)/4`
/// integer math, bit-exact with [`crate::mips`] box filtering.
/// Operates on decoded (not packed) data: the chain is
/// decode -> mip -> sample, each stage independently testable.
pub const MICROSTORE_MIP_WGSL: &str = r#"
struct MipParams {
    src_w: u32,
    src_h: u32,
    dst_w: u32,
    dst_h: u32,
};

@group(0) @binding(0) var<uniform> params: MipParams;
@group(0) @binding(1) var<storage, read> src_texels: array<u32>;
@group(0) @binding(2) var<storage, read_write> dst_texels: array<u32>;

@compute @workgroup_size(64)
fn mip_downsample(@builtin(global_invocation_id) gid: vec3<u32>) {
    let t = gid.x;
    let count = params.dst_w * params.dst_h;
    if (t >= count) {
        return;
    }
    let dx = t % params.dst_w;
    let dy = t / params.dst_w;
    let sx = min(dx * 2u + 1u, params.src_w - 1u);
    let sy = min(dy * 2u + 1u, params.src_h - 1u);
    let x0 = dx * 2u;
    let y0 = dy * 2u;
    let a = src_texels[y0 * params.src_w + x0];
    let b = src_texels[y0 * params.src_w + sx];
    let c = src_texels[sy * params.src_w + x0];
    let d = src_texels[sy * params.src_w + sx];
    dst_texels[t] = (a + b + c + d + 2u) / 4u;
}
"#;

/// True sample-time kernel: normalized UVs in, filtered bytes out. Each
/// thread decodes its four neighbours straight from the packed page
/// (offset table + block headers, no expanded cache) and bilinearly
/// mixes them in f32.
///
/// The decode logic duplicates `decode_page` per sample on purpose: that
/// duplication IS the sample-time access pattern under test. CPU mirror:
/// [`crate::sample_bilinear`]. Outputs round to bytes; expect at most one
/// code level of f32-vs-f64 rounding drift against the CPU mirror.
/// True sample-time kernel: normalized UVs in, filtered bytes out. Each
/// thread decodes its four neighbours straight from the packed page
/// (offset table + block headers, no expanded cache) and bilinearly
/// mixes them in f32.
///
/// The decode logic duplicates `decode_page` per sample on purpose: that
/// duplication IS the sample-time access pattern under test. CPU mirror:
/// [`crate::sample_bilinear`]. Outputs round to bytes; expect at most one
/// code level of f32-vs-f64 rounding drift against the CPU mirror.
pub const MICROSTORE_SAMPLE_WGSL: &str = r#"
struct Params {
    width: u32,
    height: u32,
    blocks_x: u32,
    blocks_y: u32,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> page_words: array<u32>;
@group(0) @binding(2) var<storage, read> block_base: array<u32>;
@group(0) @binding(3) var<storage, read> sample_uv: array<vec2<f32>>;
@group(0) @binding(4) var<storage, read_write> out_texels: array<u32>;

fn load_byte(byte_index: u32) -> u32 {
    return (page_words[byte_index / 4u] >> ((byte_index % 4u) * 8u)) & 0xFFu;
}

fn decode_texel(tx: u32, ty: u32) -> u32 {
    let bx = tx / 4u;
    let by = ty / 4u;
    let lx = tx % 4u;
    let ly = ty % 4u;
    let base = block_base[by * params.blocks_x + bx];
    let tag = load_byte(base);
    if (tag > 4u) {
        return 0xDEADu;
    }
    let offset = load_byte(base + 1u);
    if (tag == 0u) {
        return load_byte(base + 3u + ly * 4u + lx);
    }
    if (tag == 1u) {
        return min(offset + load_byte(base + 3u + ly * 4u + lx), 255u);
    }
    let scale = load_byte(base + 2u);
    if (tag == 2u) {
        let cell = ly * 4u + lx;
        let packed = load_byte(base + 3u + cell / 2u);
        var q = packed & 15u;
        if (cell % 2u == 1u) {
            q = packed >> 4u;
        }
        return min(offset + (q * scale + 7u) / 15u, 255u);
    }
    if (tag == 3u) {
        let cell = ly * 4u + lx;
        let bit = cell * 6u;
        let byte = bit / 8u;
        let shift = bit % 8u;
        let lo = load_byte(base + 3u + byte);
        let hi = load_byte(base + 3u + byte + 1u);
        let q = ((lo >> shift) | (hi << (8u - shift))) & 63u;
        return min(offset + (q * scale + 31u) / 63u, 255u);
    }
    let cell = ly * 4u + lx;
    let packed = load_byte(base + 3u + cell / 4u);
    let q = (packed >> ((cell % 4u) * 2u)) & 3u;
    return min(offset + (q * scale + 1u) / 3u, 255u);
}

@compute @workgroup_size(64)
fn sample_packed(@builtin(global_invocation_id) gid: vec3<u32>) {
    let s = gid.x;
    if (s >= arrayLength(&sample_uv)) {
        return;
    }
    let w = f32(params.width);
    let h = f32(params.height);
    let uv = sample_uv[s];
    let x = clamp(uv.x, 0.0, 1.0) * (w - 1.0);
    let y = clamp(uv.y, 0.0, 1.0) * (h - 1.0);
    let x0 = u32(floor(x));
    let y0 = u32(floor(y));
    let x1 = min(x0 + 1u, params.width - 1u);
    let y1 = min(y0 + 1u, params.height - 1u);
    let fx = x - f32(x0);
    let fy = y - f32(y0);
    let a = f32(decode_texel(x0, y0));
    let b = f32(decode_texel(x1, y0));
    let c = f32(decode_texel(x0, y1));
    let d = f32(decode_texel(x1, y1));
    let mixed = a * (1.0 - fx) * (1.0 - fy) + b * fx * (1.0 - fy)
        + c * (1.0 - fx) * fy + d * fx * fy;
    out_texels[s] = u32(mixed + 0.5);
}
"#;

/// LOD selection kernel: one thread per footprint outputs the mip level
/// for the max-footprint rule (`floor(log2(max_axis) / 2)` clamped).
/// Integer-exact away from power-of-two boundaries; CPU mirror
/// [`crate::lod_level`]. Boundary-adjacent footprints may differ by one
/// level between f32 log2 implementations (GPUs vary here too), which the
/// tests pin explicitly instead of pretending exactness.
pub const MICROSTORE_LOD_WGSL: &str = r#"
struct LodParams {
    width: u32,
    height: u32,
    max_level: u32,
    _p0: u32,
};

@group(0) @binding(0) var<uniform> params: LodParams;
@group(0) @binding(1) var<storage, read> jac: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> out_level: array<u32>;

@compute @workgroup_size(64)
fn lod_select(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= arrayLength(&jac)) {
        return;
    }
    let j = jac[i];
    let px = j.x * f32(params.width);
    let py = j.y * f32(params.width);
    let qx = j.z * f32(params.height);
    let qy = j.w * f32(params.height);
    let rho_sq = max(px * px + py * py, qx * qx + qy * qy);
    var level = 0u;
    if (rho_sq > 1.0) {
        level = u32(floor(0.5 * log2(rho_sq)));
    }
    out_level[i] = min(level, params.max_level);
}
"#;

/// Anisotropic sample-time kernel: normalized UV plus Jacobian in,
/// filtered bytes out. Each thread spreads taps across one pixel
/// footprint along the major axis (1/2/4/8 by the `taps` uniform),
/// decoding every tap straight from the packed page. CPU mirror:
/// [`crate::sample_aniso`]. Outputs round to bytes; expect small
/// f32-vs-f64 drift (pinned by tests, not assumed zero).
pub const MICROSTORE_ANISO_WGSL: &str = r#"
struct AnisoParams {
    width: u32,
    height: u32,
    blocks_x: u32,
    blocks_y: u32,
    taps: u32,
    _p0: u32,
    _p1: u32,
    _p2: u32,
};

@group(0) @binding(0) var<uniform> params: AnisoParams;
@group(0) @binding(1) var<storage, read> page_words: array<u32>;
@group(0) @binding(2) var<storage, read> block_base: array<u32>;
@group(0) @binding(3) var<storage, read> sample_uv: array<vec2<f32>>;
@group(0) @binding(4) var<storage, read> sample_jac: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read_write> out_texels: array<u32>;

fn load_byte(byte_index: u32) -> u32 {
    return (page_words[byte_index / 4u] >> ((byte_index % 4u) * 8u)) & 0xFFu;
}

fn decode_texel(tx: u32, ty: u32) -> u32 {
    let bx = tx / 4u;
    let by = ty / 4u;
    let lx = tx % 4u;
    let ly = ty % 4u;
    let base = block_base[by * params.blocks_x + bx];
    let tag = load_byte(base);
    if (tag > 4u) {
        return 0xDEADu;
    }
    let offset = load_byte(base + 1u);
    if (tag == 0u) {
        return load_byte(base + 3u + ly * 4u + lx);
    }
    if (tag == 1u) {
        return min(offset + load_byte(base + 3u + ly * 4u + lx), 255u);
    }
    let scale = load_byte(base + 2u);
    if (tag == 2u) {
        let cell = ly * 4u + lx;
        let packed = load_byte(base + 3u + cell / 2u);
        var q = packed & 15u;
        if (cell % 2u == 1u) {
            q = packed >> 4u;
        }
        return min(offset + (q * scale + 7u) / 15u, 255u);
    }
    if (tag == 3u) {
        let cell = ly * 4u + lx;
        let bit = cell * 6u;
        let byte = bit / 8u;
        let shift = bit % 8u;
        let lo = load_byte(base + 3u + byte);
        let hi = load_byte(base + 3u + byte + 1u);
        let q = ((lo >> shift) | (hi << (8u - shift))) & 63u;
        return min(offset + (q * scale + 31u) / 63u, 255u);
    }
    let cell = ly * 4u + lx;
    let packed = load_byte(base + 3u + cell / 4u);
    let q = (packed >> ((cell % 4u) * 2u)) & 3u;
    return min(offset + (q * scale + 1u) / 3u, 255u);
}

fn sample_bilinear_texel(x: f32, y: f32) -> f32 {
    let w = f32(params.width);
    let h = f32(params.height);
    let xc = clamp(x, 0.0, w - 1.0);
    let yc = clamp(y, 0.0, h - 1.0);
    let x0 = u32(floor(xc));
    let y0 = u32(floor(yc));
    let x1 = min(x0 + 1u, params.width - 1u);
    let y1 = min(y0 + 1u, params.height - 1u);
    let fx = xc - f32(x0);
    let fy = yc - f32(y0);
    let a = f32(decode_texel(x0, y0));
    let b = f32(decode_texel(x1, y0));
    let c = f32(decode_texel(x0, y1));
    let d = f32(decode_texel(x1, y1));
    return a * (1.0 - fx) * (1.0 - fy) + b * fx * (1.0 - fy)
        + c * (1.0 - fx) * fy + d * fx * fy;
}

@compute @workgroup_size(64)
fn sample_aniso_packed(@builtin(global_invocation_id) gid: vec3<u32>) {
    let s = gid.x;
    if (s >= arrayLength(&sample_uv)) {
        return;
    }
    let w = f32(params.width);
    let h = f32(params.height);
    let uv = sample_uv[s];
    let j = sample_jac[s];
    var taps = 1u;
    if (params.taps >= 8u) {
        taps = 8u;
    } else if (params.taps >= 4u) {
        taps = 4u;
    } else if (params.taps >= 2u) {
        taps = 2u;
    }
    if (taps == 1u) {
        let x = clamp(uv.x, 0.0, 1.0) * (w - 1.0);
        let y = clamp(uv.y, 0.0, 1.0) * (h - 1.0);
        out_texels[s] = u32(sample_bilinear_texel(x, y) + 0.5);
        return;
    }
    let a_len_sq = j.x * j.x + j.y * j.y;
    let b_len_sq = j.z * j.z + j.w * j.w;
    var dx = j.x;
    var dy = j.y;
    if (b_len_sq > a_len_sq) {
        dx = j.z;
        dy = j.w;
    }
    let len = max(sqrt(dx * dx + dy * dy), 1e-12);
    dx = dx / len;
    dy = dy / len;
    var sum = 0.0;
    for (var i = 0u; i < taps; i++) {
        let t = (f32(i) + 0.5) / f32(taps) - 0.5;
        let x = clamp(uv.x + dx * t * len, 0.0, 1.0) * (w - 1.0);
        let y = clamp(uv.y + dy * t * len, 0.0, 1.0) * (h - 1.0);
        sum += sample_bilinear_texel(x, y);
    }
    out_texels[s] = u32(sum / f32(taps) + 0.5);
}
"#;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EncodeMode, fixtures};

    #[test]
    fn base_table_matches_wire_walk() {
        for (_, field) in fixtures::all(20, 12) {
            for mode in [
                EncodeMode::Raw8,
                EncodeMode::Residual8,
                EncodeMode::Residual6,
                EncodeMode::Residual4,
                EncodeMode::Residual2,
                EncodeMode::Adaptive { max_abs_error: 1.5 },
            ] {
                let page = EncodedPage::encode(&field, mode);
                let bytes = page.to_bytes();
                let table = block_base_table(&page);
                assert_eq!(table.len(), page.blocks.len());
                // First block starts right after the 21-byte page header.
                assert_eq!(table[0], 21);
                for (i, block) in page.blocks.iter().enumerate() {
                    let base = table[i] as usize;
                    assert_eq!(bytes[base], block.codec.tag());
                    assert_eq!(bytes[base + 1], block.offset);
                    assert_eq!(bytes[base + 2], block.scale);
                    let next = base + 3 + block.codec.payload_len();
                    if i + 1 < table.len() {
                        assert_eq!(table[i + 1] as usize, next);
                    } else {
                        assert_eq!(next, bytes.len());
                    }
                }
            }
        }
    }

    #[test]
    fn padded_upload_is_word_aligned_and_prefixed() {
        for (_, field) in fixtures::all(13, 7) {
            let page = EncodedPage::encode(&field, EncodeMode::Residual4);
            let padded = padded_upload_bytes(&page);
            assert_eq!(padded.len() % 4, 0);
            assert_eq!(&padded[..page.encoded_bytes()], &page.to_bytes()[..]);
            assert!(padded.len() - page.encoded_bytes() < 4);
        }
    }

    #[test]
    fn upload_size_accounting() {
        // 8x8 uniform Residual4: 21 B header + 4 blocks x 11 B.
        let field = fixtures::uniform(8, 8, 3);
        let page = EncodedPage::encode(&field, EncodeMode::Residual4);
        let up = upload_size(&page);
        assert_eq!(up.wire_bytes, 21 + 4 * 11);
        assert_eq!(up.padded_bytes, 65 + 3);
        assert_eq!(up.table_bytes, 16);
        assert_eq!(up.params_bytes, 16);
        assert_eq!(up.expanded_bytes, 64 * 4);
        assert_eq!(up.gpu_upload_bytes(), 68 + 16 + 16);
        assert_eq!(up.gpu_resident_bytes(), 100 + 256);
    }

    #[test]
    fn wgsl_source_is_nonempty_and_names_entry() {
        assert!(MICROSTORE_DECODE_WGSL.contains("fn decode_page"));
        assert!(MICROSTORE_DECODE_WGSL.contains("@workgroup_size(64)"));
        // No vendor-specific or subgroup constructs in the baseline.
        assert!(!MICROSTORE_DECODE_WGSL.contains("subgroup"));
        assert!(MICROSTORE_MIP_WGSL.contains("fn mip_downsample"));
        assert!(MICROSTORE_SAMPLE_WGSL.contains("fn sample_packed"));
        assert!(MICROSTORE_LOD_WGSL.contains("fn lod_select"));
        assert!(MICROSTORE_ANISO_WGSL.contains("fn sample_aniso_packed"));
        for src in [
            MICROSTORE_MIP_WGSL,
            MICROSTORE_SAMPLE_WGSL,
            MICROSTORE_LOD_WGSL,
            MICROSTORE_ANISO_WGSL,
        ] {
            assert!(src.contains("@workgroup_size(64)"));
            assert!(!src.contains("subgroup"));
        }
    }
}
