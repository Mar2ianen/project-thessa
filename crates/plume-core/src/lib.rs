//! Backend-neutral rocket-plume semantic field (`docs/38_ENGINE_PLUME_RENDERING.md`).
//!
//! Layering contract:
//!
//! ```text
//! engine/nozzle state + environment
//!     -> PlumeSource (+ PlumeEnvironment)
//!     -> analytic mean field (AxialProfile)
//!        + adaptive RCBT residual (later crate)
//!     -> PlumeField
//!        -> wgpu raster/compute, lighting proxies, future ray queries
//! ```
//!
//! This crate knows nothing about Bevy, wgpu, Vulkan, entities, or GPU
//! buffers. It works in SI units (`f64`) and is deterministic in its inputs:
//! the same [`source::PlumeSource`] + [`source::PlumeEnvironment`] always
//! yields the same [`profile::AxialProfile`].
//!
//! Deliberately NOT modeled here: combustion chemistry, CFD turbulence,
//! vehicle/propulsion state (owned by simulation), and any renderer (the
//! `beauty.rs` cone stays the `Low`/fallback impostor until the volume pass
//! lands with measured parity).

#![forbid(unsafe_code)]

pub mod cbt_volume;
pub mod medium;
pub mod optics;
pub mod profile;
pub mod source;

pub use cbt_volume::{PlumeBound, PlumeRegion, region_for_node, residual_error_estimate};
pub use medium::{
    MediumSample, integrate_ray, radial_weight, radiant_power, sample_medium,
};
pub use optics::{OpticalMaterial, hue_divisor, optical_material, ramp_rgb};
pub use source::ExhaustFamily;
pub use profile::{
    AxialProfile, AxialStation, ExpansionRegime, emission_gain, expansion_regime,
    shock_cell_spacing_m,
};
pub use source::{PlumeEnvironment, PlumeSource, RigidTransform};
