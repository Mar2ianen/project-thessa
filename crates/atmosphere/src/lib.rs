//! Shared visual-atmosphere optics for Project Thessa.
//!
//! One atmosphere definition feeds every backend (spec §13-14): raster/LUT
//! rendering, ray-aware CPU queries, and future Solari proxies. Domain code
//! stays free of graphics APIs; all spatial state is `f64` SI.

#![forbid(unsafe_code)]

mod eclipse;
mod emission;
mod lights;
mod optics;
mod scattering;

pub use eclipse::eclipse_visibility;
pub use emission::aurora_emission;
pub use lights::{CelestialLight, angular_radius, blackbody_rgb, irradiance_at_distance};
pub use optics::{
    AbsorptionLayer, AtmosphereOptics, AuroraParams, EmissionParams, OpticsError, ScatteringLayer,
    nitrogen_oxygen_optics,
};
pub use scattering::{
    SkyResult, aerial_perspective, airglow_radiance, hg_phase, rayleigh_phase, sky_radiance,
    transmittance,
};
