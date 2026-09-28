//! Deterministic propagation and cached coast-trajectory algorithms.

#![forbid(unsafe_code)]

use thessa_celestial::*;

mod affine_propagator;
mod gravity_patch;
mod integrator;
mod onrails;
mod table;
mod tick_integrator;

pub use affine_propagator::{
    AffinePropagator, AnalyticError, AnalyticFallback, AnalyticStep, ModeCoefficients,
    PiecewiseReport, PropagatorError, StepCoefficients, propagate_piecewise,
};
pub use gravity_patch::{
    CohortConfig, CohortEval, CohortEvaluator, CohortReport, GravityPatch, HESSIAN_FROBENIUS_NORM,
    HESSIAN_REMAINDER, PatchError, affine_segment_bound, compile_patch, evaluate_cohorts,
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
pub use table::{EphemerisTable, TABLE_NODE_EVERY_STEPS, TableSnapshot};
pub use tick_integrator::{TickIntegratorConfig, propagate_tick_adaptive};
