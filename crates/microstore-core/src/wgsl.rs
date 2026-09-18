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
    }
}
