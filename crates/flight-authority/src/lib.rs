//! Authoritative flight stepping shared by the headless server and the
//! client (game layer, GPL).
//!
//! The authoritative equations stay in `thessa-sim-core`; this crate owns
//! the flight runtime: input conditioning, regime selection, the fixed-step
//! advance loop and rails-bake orchestration. Ring-fenced from Bevy by
//! construction — background bakes go through [`BakeQueue`], whose
//! implementations live beside their runtimes (Bevy pool in the client,
//! threads on the server, inline in tests).
//!
//! Migration note: the runtime still lives in the client. Types move here
//! one at a time with the client re-importing them, so every step stays
//! green.

pub mod bake;
pub mod mode;
pub mod runtime;

pub use bake::{BakeQueue, BakedRails, InlineBakeQueue, RailsBakeRequest};
pub use mode::{ControlMode, FlightRegime};
pub use runtime::{
    FlightAuthority, FlightTraceWriter, LocalAirKinematics, X15_STALL_ANGLE_DEG,
    conventional_angle_of_attack_deg, local_air_kinematics,
};
