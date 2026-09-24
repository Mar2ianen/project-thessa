//! Procedural fuselages: authoring model plus hangar compiler.
//!
//! This crate implements the station-loft half of
//! `docs/details/03_PROCEDURAL_FUSELAGES.md`. The editor stores one body
//! as a tail-to-nose [`BodyStation`] loft with superellipse sections;
//! leaving the hangar compiles it into solver-ready
//! [`AeroPanel`](thessa_sim_core::AeroPanel) strips plus feed-pipeline
//! tanks, hull mass, contact parts, interior volumes, and port anchors.
//! Runtime flight code consumes only the compiled output, never the
//! stations below.
//!
//! # Formal model
//!
//! * State: [`ProceduralBody`] — station spline `w(x)`, `h(x)`, `n(x)`
//!   with lateral offsets, interior [`InteriorRegion`] allocations, and
//!   [`BodyPort`] anchors over the body axis `x`.
//! * Inputs: [`BodyCompileOptions`] — zone-length and area-change
//!   tolerances, recursion cap, and surface integration density; body control
//!   regions select pitch/yaw strips by an axial interval.
//! * Outputs: [`CompiledBody`] — panels, resolved control channels, per-tank
//!   [`TankMount`](thessa_sim_core::TankMount) entries, hull mass and
//!   inertia, contact parts, interior volumes, and a
//!   [`CompiledBodySummary`] telemetry record.
//!
//! # Frames
//!
//! Body axes are sim-core convention (`+X` forward, `+Y` right, `+Z` up)
//! with stations running tail-to-nose (nose last). Solver strip chords
//! follow the local section-centroid line; pitch/yaw lift axes are
//! orthogonalized against it and reverse for negative area gradients.
//! Mounting is an origin offset: orientation mounts arrive with vehicle
//! assembly. Renderer-neutral [`FuselageMesh`] tessellation is available
//! separately from flight panels.
//!
//! All geometry is `f64`, SI units, no global coefficients: areas,
//! volumes, masses, and slopes derive from the authored stations.
//!
//! # Architectural boundary: hangar entity, never flight code
//!
//! This crate is a hangar-side compiler. It runs in the editor, in
//! `vehicle-baker`, and in tests; it never runs in the flight hot path.
//! Flight consumes only compiled artifacts: [`AeroPanel`] strips inside
//! [`AeroGeometry`](thessa_sim_core::AeroGeometry),
//! [`TankMount`](thessa_sim_core::TankMount) entries, and mass data
//! shipped across the boundary as serialized data (the roundtrip test
//! below pins exactly that). Consequently this crate depends only on
//! `glam`, `serde`, and `thessa-sim-core`: no Bevy, no Tokio, no render
//! or physics-loop crates. Flight crates must never depend on it; the
//! baker test next door proves the hangar-side wiring instead.

mod body;
mod collision;
mod compile;
mod error;
mod golden;
mod mesh;
mod section;
mod structure;
mod summary;

pub use body::{
    BodyControlPlane, BodyControlRegion, BodyPort, BodyStructuralLayout, HullMaterial,
    InteriorRegion, PortKind, ProceduralBody, RegionKind,
};
pub use collision::{BodyCollisionOptions, body_collision_parts};
pub use compile::{
    BodyCompileOptions, BodyPortCompiled, CompiledBody, CompiledBodyTank, CompiledRegion,
    compile_body,
};
pub use error::FuselageError;
pub use golden::{
    dream_chaser_body, dream_chaser_style_body, juno_stack, juno_style_stack,
    simpleplanes_style_block, sp_block,
};
pub use mesh::{FuselageMesh, tessellate_fuselage};
pub use section::{
    BodyStation, gamma, outline_point, section_area_m2, section_centroid_yz, superellipse_area,
    superellipse_perimeter,
};
pub use structure::{CompiledHull, point_inertia, ring_inertia, solid_inertia, tube_inertia};
pub use summary::CompiledBodySummary;

#[cfg(test)]
mod tests;
