//! Procedural aerodynamic surfaces: authoring model plus hangar compiler.
//!
//! This crate implements the authoring-to-compiled pipeline from
//! `docs/details/02_PROCEDURAL_AERO_SURFACES.md`. The editor stores one
//! procedural surface as authoring geometry; leaving the hangar compiles it
//! into solver-ready [`AeroPanel`](thessa_sim_core::AeroPanel) zones plus
//! control/fold mechanism data. Runtime flight code consumes only the
//! compiled output, never the splines below.
//!
//! # Formal model
//!
//! * State: [`ProceduralSurface`] — flat planform splines `x_le(s)`,
//!   `x_te(s)`, bend curve `z(s)`, per-station section data, control regions,
//!   and fold joints over the normalized span coordinate `s in [0, 1]`.
//! * Inputs: [`CompileOptions`] — subdivision tolerances, recursion cap, and
//!   the mechanism state (fold angles) to compile.
//! * Outputs: [`CompiledSurface`] — panels, per-panel ownership tags,
//!   [`ControlSurfaceDefinition`](thessa_sim_core::ControlSurfaceDefinition)
//!   entries, compiled fold records, and a [`CompiledSurfaceSummary`]
//!   telemetry record.
//!
//! # Frames
//!
//! Surface-local axes: `x` chordwise positive aft (leading edge towards
//! trailing edge), `y` spanwise positive towards the tip, `z` up. The
//! body frame is sim-core convention (`+X` forward, `+Y` right, `+Z`
//! up), so mounting reflects chordwise positions (`body.x = -local.x`):
//! the trailing edge lands aft. Solver axes are directions, not
//! geometry: `chord_axis` stays `+X` as the solver's forward reference
//! (it addresses the chord trailing-to-leading) and `lift_axis` stays
//! near `+Z`; only points (panel positions, centers of pressure,
//! hinges, bounding boxes) conjugate. Fold records conjugate fully
//! (hinge point and axis reflect, stored angle negates under the
//! orientation-reversing map) so the runtime reproduces compiled
//! positions exactly. Mounting into the body frame is an origin offset
//! plus an optional roll (`mount_roll_rad`, fins and keels) and an
//! optional left/right mirror; full orientation mounts are out of scope
//! for this slice and will arrive with the vehicle assembly step.
//!
//! All geometry is `f64`, SI units, no global coefficients: areas, spans,
//! sweeps, frames, and ownership are derived from the authored splines.
//!
//! # Architectural boundary: hangar entity, never flight code
//!
//! This crate is a hangar-side compiler. It runs in the editor, in
//! `vehicle-baker`, and in tests; it never runs in the flight hot path.
//! Flight consumes only compiled artifacts: [`AeroPanel`](thessa_sim_core::AeroPanel)
//! zones inside
//! [`AeroGeometry`](thessa_sim_core::AeroGeometry) plus
//! [`ControlSurfaceDefinition`](thessa_sim_core::ControlSurfaceDefinition)
//! entries, shipped across the boundary as serialized data (the roundtrip
//! test below pins exactly that). Consequently this crate depends only on
//! `glam`, `serde`, and `thessa-sim-core`: no Bevy, no Tokio, no render or
//! physics-loop crates. Flight crates must never depend on it; the baker
//! test next door proves the hangar-side wiring instead.

mod bend;
mod collision;
mod compile;
mod error;
mod golden;
mod mechanism;
mod planform;
mod preset;
mod profile;
mod section;
mod structure;
mod summary;
mod surface;

pub use bend::{BendCurve, BendStation};
pub use collision::CollisionOptions;
pub use compile::{
    CompileOptions, CompiledFold, CompiledSurface, MechanismState, PanelTag, RefinementMode,
    compile_surface,
};
pub use error::SurfaceError;
pub use golden::{
    boeing_777x, boeing_777x_half_wing, concorde, concorde_wing, dream_chaser, dream_chaser_wing,
    pathfinder_wing, shuttle_orbiter, shuttle_orbiter_wing,
};
pub use mechanism::{ControlRegion, ControlRegionKind, FoldJoint};
pub use planform::{Planform, SpanStation};
pub use preset::{
    ControlChannels, ControlMixing, aileron, airbrake, anti_servo_tab, elevator, elevon, flap,
    flaperon, mix_command, rudder, ruddervator, slat, spoiler, stabilator, trim_tab,
};
pub use profile::{CruiseRequirement, Naca4, ProfilePick, recommend_cruise_profile};
pub use section::{AeroProfileId, SectionData, SectionStation};
pub use structure::{CompiledStructure, SolidMaterial, StructuralLayout};
pub use summary::{CompiledSurfaceSummary, ControlSummary};
pub use surface::{ProceduralSurface, SurfaceTopology};

#[cfg(test)]
mod tests;
