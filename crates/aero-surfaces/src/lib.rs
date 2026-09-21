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
//! trailing edge), `y` spanwise positive towards the tip, `z` up. A flat
//! unmirrored surface therefore compiles to panels with chord axis `+X` and
//! lift axis `+Z`, matching
//! [`AeroPanel::flat_plate`](thessa_sim_core::AeroPanel::flat_plate).
//! Mounting into the body frame is an origin offset plus an optional
//! left/right mirror; full orientation mounts are out of scope for this
//! slice and will arrive with the vehicle assembly step.
//!
//! All geometry is `f64`, SI units, no global coefficients: areas, spans,
//! sweeps, frames, and ownership are derived from the authored splines.

mod bend;
mod compile;
mod error;
mod golden;
mod mechanism;
mod planform;
mod section;
mod summary;
mod surface;

pub use bend::{BendCurve, BendStation};
pub use compile::{
    CompileOptions, CompiledFold, CompiledSurface, MechanismState, PanelTag, compile_surface,
};
pub use error::SurfaceError;
pub use golden::{
    boeing_777x, boeing_777x_half_wing, concorde, concorde_wing, dream_chaser, dream_chaser_wing,
    shuttle_orbiter, shuttle_orbiter_wing,
};
pub use mechanism::{ControlRegion, FoldJoint};
pub use planform::{Planform, SpanStation};
pub use section::{AeroProfileId, SectionData, SectionStation};
pub use summary::CompiledSurfaceSummary;
pub use surface::ProceduralSurface;

#[cfg(test)]
mod tests;
