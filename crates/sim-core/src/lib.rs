//! Project Thessa reusable authoritative simulation core.
//!
//! The first vertical slice deliberately contains only deterministic celestial
//! ephemerides, point-mass gravity, reference-frame-labelled state vectors, and
//! orbital propagators. It has no renderer, async runtime, or game-specific API.

#![forbid(unsafe_code)]

mod aero;
mod affine_propagator;
mod atmosphere;
mod collision;
mod docking;
mod ephemeris;
mod feed;
mod flight;
mod frames;
mod gravity;
mod gravity_patch;
mod gravity_tree;
mod high_speed;
mod integrator;
mod onrails;
mod propulsion;
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
pub use collision::{
    CollisionAxis, CollisionError, CollisionGeometry, CollisionMaterial, CollisionPart,
    CollisionShape,
};
pub use docking::{
    DockingError, DockingKinematics, DockingPortClass, DockingPortSpec, DockingPortState,
    DockingSession,
};
pub use ephemeris::{
    BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, EphemerisFrame, EphemerisScratch,
    KeplerOrbit, OsculatingElements,
};
pub use feed::{CompiledTank, FEED_MAX_VELOCITY_MPS, FeedLine, TankMount, TankShape, TankSpec};
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
    quadrupole_correction, quadrupole_error_estimate,
};
pub use high_speed::{
    BOOM_ANCHOR_PSF, BOOM_OVERPRESSURE_GAIN, BoomCarpet, HighSpeedError, boom_carpet,
    buffet_fluctuation, buffet_gain, vapor_cone_active,
};
pub use integrator::{
    AdaptiveIntegratorConfig, ImpulsiveBurn, IntegratorError, IntegratorStats, PropagationResult,
    SampledPath, SampledPathEnd, SensitivityPropagation, TestParticleState, ThrustArc,
    ThrustDirection, ThrustPropagationResult, VelocitySensitivity, VerletConfig,
    propagate_adaptive, propagate_adaptive_dop853, propagate_adaptive_sensitivity,
    propagate_adaptive_with_burns, propagate_adaptive_with_thrust, propagate_sampled_extend,
    propagate_sampled_verlet, propagate_sampled_verlet_fast, propagate_sampled_verlet_scaled,
    propagate_velocity_verlet, rtn_basis,
};
pub use onrails::{
    COAST_RAILS_EXTEND_CHUNK, COAST_RAILS_HEAD_STEPS, COAST_RAILS_MAX_STEPS,
    COAST_RAILS_MIN_AHEAD_S, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_STEP_S,
    COAST_RAILS_VELOCITY_TOL_MPS, DISPLAY_SCALED_ETA, DISPLAY_SCALED_H_MAX_S,
    DISPLAY_SCALED_H_MIN_S, DISPLAY_SCALED_MAX_SAMPLES, OnRailsCache, OnRailsWake,
};
pub use propulsion::{
    AIR_CP_J_KG_K, AIR_GAMMA, AirAltitudePoint, AirCycle, AirOperatingPoint, AirbreathingSpec,
    AltitudePoint, BurnPoint, ChamberMaterial, ChamberSpec, ColdGasThrusterSpec,
    CompiledAirbreather, CompiledChamber, CompiledColdGas, CompiledEngine, CompiledEstoc,
    CompiledJet, CompiledLiquid, CompiledMonoprop, CompiledPropulsionSystem, CompiledSolid,
    CoolingMode, CycleLimits, EARTH_OXYGEN_FRACTION, ESTOC_DEFAULT_SWITCH_MACH_HI,
    ESTOC_DEFAULT_SWITCH_MACH_LO, ESTOC_DEFAULT_TRANSITION_TAU_S, ESTOC_MAX_ROCKET_PC_PA,
    ESTOC_REINFORCEMENT_FRACTION, EngineCycle, EngineMount, EngineOperatingPoint, EnginePlumeState,
    EngineSpool, EstocCommand, EstocMode, EstocPoint, EstocSpec, FlightCondition, GimbalEffector,
    IntakeKind, JetFuel, JetMount, LiquidEngineSpec, MAX_SYSTEM_CHAMBERS, MonopropThrusterSpec,
    NTR_COOLDOWN_FRACTION, NTR_DEFAULT_RATED_BURN_S, NTR_DEFAULT_SPECIFIC_MASS_KG_PER_MW,
    NTR_DEFAULT_STARTUP_TAU_S, NTR_INLET_TEMP_K, NTR_KINETIC_EFFICIENCY,
    NTR_MAX_CHAMBER_PRESSURE_PA, NozzleContour, NozzleExitState, NtrFluid, NtrSupplement,
    NuclearThermalSpec, Propellant, PropellantThermo, PropulsionError, PropulsionSystemSpec,
    RCS_DEFAULT_MIN_ON_TIME_S, RCS_DEFAULT_RISE_TIME_S, RcsCluster, RcsMount, RcsPulse,
    RcsThruster, SEPARATION_PRESSURE_RATIO, STANDARD_GRAVITY_MPS2, SolidMotorSpec,
    SystemAltitudePoint, SystemMount, SystemOperatingPoint, THESSA_OXYGEN_MASS_FRACTION,
    advance_jet_spool, advance_spool, analyze_airbreathing, analyze_altitude,
    characteristic_velocity, flight_condition, mach_from_area_ratio, thrust_coefficient,
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
pub use vehicle::{
    ControlKind, ControlSurfaceDefinition, FoldJointRecord, VehicleDefinition, VehicleError,
    X15StarterProfile, x15_contact_geometry,
};

#[cfg(test)]
mod tests;
