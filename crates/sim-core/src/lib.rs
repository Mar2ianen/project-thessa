//! Project Thessa reusable authoritative simulation core.
//!
//! The first vertical slice deliberately contains only deterministic celestial
//! ephemerides, point-mass gravity, reference-frame-labelled state vectors, and
//! orbital propagators. It has no renderer, async runtime, or game-specific API.

#![forbid(unsafe_code)]

mod aero;
mod atmosphere;
mod ephemeris;
mod flight;
mod frames;
mod gravity;
mod integrator;
mod system;
mod time;
mod units;
mod vehicle;

pub use aero::{
    AeroCase, AeroCoefficientTable, AeroCoefficients, AeroConfig, AeroEnvironment, AeroError,
    AeroGeometry, AeroModel, AeroPanel, AeroPanelLoad, AeroResult, AeroState, PanelAeroModel,
    evaluate_batch,
};
pub use atmosphere::{AtmosphereConfig, AtmosphereError, AtmosphereSample};
pub use ephemeris::{BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, KeplerOrbit};
pub use flight::{
    FlightError, FlightForces, FlightStepInput, RigidBodyProperties, RigidBodyState,
    evaluate_flight_forces, integrate_rigid_body_duration, integrate_rigid_body_duration_sampled,
    integrate_rigid_body_step,
};
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
pub use vehicle::{ControlSurfaceDefinition, VehicleDefinition, VehicleError, X15StarterProfile};

#[cfg(test)]
mod tests;
