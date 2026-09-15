//! Project Thessa reusable authoritative simulation core.
//!
//! The first vertical slice deliberately contains only deterministic celestial
//! ephemerides, point-mass gravity, reference-frame-labelled state vectors, and
//! orbital propagators. It has no renderer, async runtime, or game-specific API.

#![forbid(unsafe_code)]

mod aero;
mod affine_propagator;
mod atmosphere;
mod ephemeris;
mod flight;
mod frames;
mod gravity;
mod gravity_patch;
mod gravity_tree;
mod integrator;
mod onrails;
mod scheduler;
mod system;
mod table;
mod tick_integrator;
mod time;
mod units;
mod vehicle;

pub use aero::{
    AeroCase, AeroCoefficientTable, AeroCoefficients, AeroConfig, AeroEnvironment, AeroError,
    AeroGeometry, AeroModel, AeroPanel, AeroPanelLoad, AeroResult, AeroSimdScratch, AeroState,
    PanelAeroModel, PanelSoA, evaluate_batch,
};
pub use affine_propagator::{
    AffinePropagator, AnalyticError, AnalyticFallback, AnalyticStep, ModeCoefficients,
    PiecewiseReport, PropagatorError, StepCoefficients, propagate_piecewise,
};
pub use atmosphere::{AtmosphereConfig, AtmosphereError, AtmosphereSample, BakedAtmosphere};
pub use ephemeris::{
    BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, EphemerisFrame, EphemerisScratch,
    KeplerOrbit, OsculatingElements,
};
pub use flight::{
    FlightError, FlightForces, FlightStepInput, RigidBodyProperties, RigidBodyState,
    constant_spin_orientation, evaluate_flight_forces, evaluate_flight_forces_soa,
    integrate_attitude_step, integrate_rigid_body_duration, integrate_rigid_body_duration_sampled,
    integrate_rigid_body_step, integrate_rigid_body_step_soa,
};
pub use frames::{ReferenceFrame, StateVector};
pub use gravity::{GravityError, GravityField};
pub use gravity_patch::{
    CohortConfig, CohortEval, CohortEvaluator, CohortReport, GravityPatch, HESSIAN_FROBENIUS_NORM,
    HESSIAN_REMAINDER, PatchError, affine_segment_bound, compile_patch, evaluate_cohorts,
};
pub use gravity_tree::{
    GravityNode, GravityNodeFrame, GravitySourceTree, TreeEval, monopole_error_estimate,
};
pub use integrator::{
    AdaptiveIntegratorConfig, ImpulsiveBurn, IntegratorError, IntegratorStats, PropagationResult,
    SampledPath, SampledPathEnd, TestParticleState, ThrustArc, ThrustDirection,
    ThrustPropagationResult, VerletConfig, propagate_adaptive, propagate_adaptive_with_burns,
    propagate_adaptive_with_thrust, propagate_sampled_extend, propagate_sampled_verlet,
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
pub use table::{EphemerisTable, TABLE_NODE_EVERY_STEPS, TableSnapshot};
pub use tick_integrator::{TickIntegratorConfig, propagate_tick_adaptive};
pub use time::{SimTime, WORLD_TICK_HZ, WORLD_TICK_S, WorldTick};
pub use units::{AU_M, DAY_S, EARTH_MASS_KG, G, JUPITER_MASS_KG, SOLAR_MASS_KG, TAU};
pub use vehicle::{ControlSurfaceDefinition, VehicleDefinition, VehicleError, X15StarterProfile};

#[cfg(test)]
mod tests;
