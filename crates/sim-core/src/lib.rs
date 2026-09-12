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
mod onrails;
mod scheduler;
mod system;
mod table;
mod time;
mod units;
mod vehicle;

pub use aero::{
    AeroCase, AeroCoefficientTable, AeroCoefficients, AeroConfig, AeroEnvironment, AeroError,
    AeroGeometry, AeroModel, AeroPanel, AeroPanelLoad, AeroResult, AeroState, PanelAeroModel,
    evaluate_batch,
};
pub use atmosphere::{AtmosphereConfig, AtmosphereError, AtmosphereSample};
pub use ephemeris::{
    BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, KeplerOrbit, OsculatingElements,
};
pub use flight::{
    FlightError, FlightForces, FlightStepInput, RigidBodyProperties, RigidBodyState,
    constant_spin_orientation, evaluate_flight_forces, integrate_attitude_step, integrate_rigid_body_duration,
    integrate_rigid_body_duration_sampled, integrate_rigid_body_step,
};
pub use frames::{ReferenceFrame, StateVector};
pub use gravity::{GravityError, GravityField};
pub use integrator::{
    AdaptiveIntegratorConfig, ImpulsiveBurn, IntegratorError, IntegratorStats, PropagationResult,
    SampledPath, SampledPathEnd, TestParticleState, VerletConfig, propagate_adaptive,
    propagate_adaptive_with_burns, propagate_sampled_extend, propagate_sampled_verlet,
    propagate_sampled_verlet_fast, propagate_sampled_verlet_scaled, propagate_velocity_verlet,
};
pub use onrails::{
    COAST_RAILS_EXTEND_CHUNK, COAST_RAILS_HEAD_STEPS, COAST_RAILS_MAX_STEPS,
    COAST_RAILS_MIN_AHEAD_S, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_STEP_S,
    COAST_RAILS_VELOCITY_TOL_MPS, DISPLAY_SCALED_ETA, DISPLAY_SCALED_H_MAX_S,
    DISPLAY_SCALED_H_MIN_S, DISPLAY_SCALED_MAX_SAMPLES, OnRailsCache, OnRailsWake,
};
pub use scheduler::{EventScheduler, ScheduledEvent, ScheduledKind};
pub use system::{
    BinaryOrbitConfig, CelestialConfig, OrbitConfig, StarConfig, SystemConfig, SystemMeta,
    SystemSpecError,
};
pub use table::{EphemerisTable, TABLE_NODE_EVERY_STEPS};
pub use time::SimTime;
pub use units::{AU_M, DAY_S, EARTH_MASS_KG, G, JUPITER_MASS_KG, SOLAR_MASS_KG, TAU};
pub use vehicle::{ControlSurfaceDefinition, VehicleDefinition, VehicleError, X15StarterProfile};

#[cfg(test)]
mod tests;
