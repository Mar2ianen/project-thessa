//! Authoritative flight stepping shared by the headless server and the
//! client (game layer, GPL).
//!
//! The authoritative equations stay in `thessa-sim-core`; this crate owns
//! the flight runtime: input conditioning, regime selection, the fixed-step
//! advance loop and rails-bake orchestration. Ring-fenced from Bevy by
//! construction — background bakes go through [`BakeQueue`], whose
//! implementations use the shared native thread worker, the legacy Bevy pool,
//! or the deterministic inline test queue.
//!
//! The authoritative runtime lives here. Native and retained Bevy hosts share
//! it and own only integration/presentation adapters.

pub mod bake;
pub mod camera;
pub mod contact;
pub(crate) mod control;
pub mod input;
pub mod instruments;
pub mod launch_site;
pub mod mode;
pub mod navball;
pub mod navigation;
pub mod presentation;
pub mod runtime;
pub mod survey;
#[cfg(not(target_arch = "wasm32"))]
pub mod thread_bake;

pub use thessa_flight_control::{
    ActuatorDynamics, ActuatorGroup, AircraftControlLaw, AircraftState, AllocationResult,
    AttitudeState, ControlDemand, ControlError, ControlLawState, DirectControlLaw, DirectionFrame,
    DirectionTarget, EffectorContribution, FlightControlLaw, FlightPolicy, FlightPolicyContext,
    GuidanceIntent, InputScheme, PilotAxes, PropulsionDemand, RollPolicy, SpacecraftControlLaw,
    TrajectoryPlanId, allocate_wrench,
};

pub use bake::{BakeQueue, BakedRails, InlineBakeQueue, RailsBakeRequest};
pub use contact::{
    BroadPhasePair, ContactActivation, ContactBroadPhase, ContactCandidate, ContactRuntime,
};
pub use launch_site::{
    SURVEY_SITE_LABELS, canonical_launch_setup, canonical_world_field, survey_bookmarks,
};
pub use mode::{ControlMode, FlightRegime};
pub use runtime::{
    COAST_DENSITY_KG_M3, FlightAuthority, FlightTraceWriter, LocalAirKinematics,
    TerrainTrackCoverage, X15_STALL_ANGLE_DEG, conventional_angle_of_attack_deg,
    local_air_kinematics,
};
pub use thessa_collision::{ContactPartyKind, ContactSummary, JointId};
pub use thessa_collision::{DynamicBodyConfig, ExternalWrench};
pub use thessa_worldgen_rocky::field::{ObstacleReport, ObstacleWithstandProof};
#[cfg(not(target_arch = "wasm32"))]
pub use thread_bake::ThreadBakeQueue;
