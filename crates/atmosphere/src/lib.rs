//! Shared visual-atmosphere optics for Project Thessa.
//!
//! One atmosphere definition feeds every backend (spec §13-14): raster/LUT
//! rendering, ray-aware CPU queries, and future Solari proxies. Domain code
//! stays free of graphics APIs; all spatial state is `f64` SI.

#![forbid(unsafe_code)]

pub mod clouds;
mod eclipse;
mod emission;
mod lights;
mod optics;
mod scattering;

pub use clouds::{
    CloudSlab, SlabError, ambient_rgb, beer_powder, clip_to_planet, dual_lobe_phase, erode_density,
    height_gradient, integrate_slab, intersect_shell, march_step_m, remap_coverage, type_gradient,
    vertical_profile,
};
pub use eclipse::eclipse_visibility;
pub use emission::{aurora_emission, aurora_phase, aurora_radiance, aurora_radiance_to};
/// Backend-neutral GPU mirror of analytic emission and finite optical transport.
pub const AURORA_WGSL: &str = include_str!("aurora.wgsl");
pub use lights::{CelestialLight, angular_radius, blackbody_rgb, irradiance_at_distance};
pub use optics::{
    AbsorptionLayer, AtmosphereOptics, AuroraParams, EmissionParams, OpticsError, ScatteringLayer,
    nitrogen_oxygen_optics,
};
pub use scattering::{
    SkyResult, aerial_perspective, airglow_radiance, hg_phase, rayleigh_phase, sky_radiance,
    transmittance,
};
