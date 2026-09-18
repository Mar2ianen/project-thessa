//! Backend-neutral microscaled residual codec for scalar surface fields.
//!
//! Implements Phase A of `docs/41_MICROSCALED_SURFACE_STORAGE.md`: a CPU
//! reference encoder/decoder for scalar `u8` material channels (albedo,
//! roughness, mineral weights) using 4x4 blocks with a local offset/range
//! reference plus packed residuals.
//!
//! Design rules (from the doc):
//!
//! - precision follows locality: each block stores its own reference, so a
//!   globally wide range costs only small local residuals;
//! - the CPU reference codec exists before any backend fast path;
//! - encode and decode are deterministic: same input bytes give same output
//!   bytes, with no hash maps or data-dependent iteration order;
//! - the codec never touches simulation or canonical terrain semantics; it
//!   only packs renderer-side scalar fields.
//!
//! Block layout (all integers, little-endian on the wire):
//!
//! ```text
//! Raw8:      [tag=0] [16 verbatim bytes]
//! Residual8: [tag=1] [offset=min u8] [16 residual bytes, value-min]
//! Residual4: [tag=2] [offset=min u8] [scale=range u8] [8 packed nibbles]
//! ```
//!
//! Decode is uniform: `value = offset + residual` for 8-bit forms, and
//! `value = offset + round(q * scale / 15)` for 4-bit nibbles `q`.
//! `Raw8` and `Residual8` are lossless; `Residual4` has a measured error
//! bounded by `range / 30 + 0.5` per texel (half of one 4-bit step plus
//! output rounding), i.e. at most 9.0 for a full-range block.
//!
//! # Example
//!
//! ```rust
//! use thessa_microstore_core::{EncodeMode, EncodedPage, ScalarField, fixtures};
//!
//! let field = fixtures::gradient(16, 16);
//! let page = EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
//! let bytes = page.to_bytes();
//! let back = EncodedPage::from_bytes(&bytes).expect("round trip");
//! assert_eq!(back.decode(), field);
//! ```

pub mod codec;
pub mod fixtures;
pub mod metrics;
pub mod pgm;
pub mod wgsl;

pub use codec::{CodecError, EncodeMode, EncodedBlock, EncodedPage, MicroCodec, ScalarField};
pub use metrics::{ErrorStats, measure};
