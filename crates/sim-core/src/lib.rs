//! Project Thessa reusable authoritative simulation core.
//!
//! The first vertical slice deliberately contains only deterministic celestial
//! ephemerides, point-mass gravity, reference-frame-labelled state vectors, and
//! orbital propagators. It has no renderer, async runtime, or game-specific API.

#![forbid(unsafe_code)]

mod ephemeris;
mod frames;
mod gravity;
mod integrator;
mod system;
mod time;
mod units;

pub use ephemeris::{BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, KeplerOrbit};
pub use frames::{ReferenceFrame, StateVector};
pub use gravity::{GravityError, GravityField};
pub use integrator::{
    AdaptiveIntegratorConfig, ImpulsiveBurn, IntegratorError, IntegratorStats, PropagationResult,
    TestParticleState, VerletConfig, propagate_adaptive, propagate_adaptive_with_burns,
    propagate_velocity_verlet,
};
pub use system::{
    BinaryOrbitConfig, CelestialConfig, OrbitConfig, StarConfig, SystemConfig, SystemMeta,
    SystemSpecError,
};
pub use time::SimTime;
pub use units::{AU_M, DAY_S, EARTH_MASS_KG, G, JUPITER_MASS_KG, SOLAR_MASS_KG, TAU};

#[cfg(test)]
mod tests;
