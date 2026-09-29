//! Rapier collision backend for Project Thessa.
//!
//! `thessa-sim-core` owns authoritative state, units, mass properties and
//! backend-neutral collision geometry. This crate owns only the transient
//! contact scene and solver state. No Rapier handle or math type crosses this
//! public API.
//!
//! Rapier's built-in gravity is deliberately disabled. Thessa samples the
//! multi-body gravity field itself and supplies the resulting force per body,
//! alongside aerodynamic, propulsion and actuator loads. This prevents a
//! second gravity model from silently entering the authoritative equations.
//!
//! With the `parallel` feature (enabled by default), Rapier executes on the
//! Rayon pool active on the calling thread. This crate intentionally creates no
//! dedicated pool, so the simulation scheduler remains the owner of CPU budget.

#![forbid(unsafe_code)]

mod bodies;
mod contact_queries;
mod joints;

use std::collections::BTreeMap;
use std::{error::Error, fmt};

use glam::{DMat3, DQuat, DVec3};
use rapier3d_f64::dynamics::MassProperties;
use rapier3d_f64::math::{Matrix, Pose, Rotation, Vector};
use rapier3d_f64::prelude::{
    BroadPhaseBvh, CCDSolver, ColliderBuilder, ColliderHandle, ColliderSet, FixedJointBuilder,
    GenericJointBuilder, ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager,
    JointAxesMask, JointAxis, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    PrismaticJointBuilder, QueryFilter, Ray, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
};
use thessa_sim_core::{
    CollisionAxis, CollisionGeometry, CollisionMaterial, CollisionShape, CompiledLandingLeg,
    CompiledWheelChassis, FlightForces, LandingLegState, RigidBodyProperties, RigidBodyState,
};

const QUATERNION_TOLERANCE: f64 = 1.0e-6;
const MIN_MASS_KG: f64 = 1.0e-9;
const MIN_INERTIA: f64 = 1.0e-12;
const MIN_STEP_S: f64 = 1.0e-9;
const WRENCH_CHANGE_ABS: f64 = 1.0e-9;
const WRENCH_CHANGE_REL: f64 = 1.0e-10;

/// Stable Thessa-side identifier for a dynamic contact body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CollisionBodyId(u64);

impl CollisionBodyId {
    pub const fn raw(self) -> u64 {
        self.0
    }
}

/// Stable Thessa-side identifier for a fixed world collider.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct StaticColliderId(u64);

impl StaticColliderId {
    pub const fn raw(self) -> u64 {
        self.0
    }
}

/// Stable Thessa-side identifier for a kinematic world body.
///
/// Kinematic bodies are the production seam for rotating planetary terrain:
/// their pose at tick `n+1` is derived from the canonical ephemeris and body
/// rotation model, and Rapier derives the surface velocity that participates
/// in contacts. They are never integrated from forces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct KinematicBodyId(u64);

impl KinematicBodyId {
    pub const fn raw(self) -> u64 {
        self.0
    }
}

/// Stable Thessa-side identifier for a docking (fixed) joint between two
/// dynamic contact bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(u64);

impl JointId {
    pub const fn raw(self) -> u64 {
        self.0
    }
}

struct JointEntry {
    rapier: ImpulseJointHandle,
    a: CollisionBodyId,
    b: CollisionBodyId,
}

/// A small inertial frame used by the contact scene.
///
/// `orientation_local_to_inertial` is constant while the frame is active. The
/// origin may translate at the constant velocity supplied here. Rotating
/// planetary terrain should therefore be represented as moving/kinematic
/// geometry in the production integration rather than by rotating this frame;
/// that avoids hidden Coriolis/centrifugal terms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionFrame {
    pub origin_inertial_m: DVec3,
    pub origin_velocity_inertial_mps: DVec3,
    pub orientation_local_to_inertial: DQuat,
}

impl CollisionFrame {
    pub fn inertial_at(origin_inertial_m: DVec3, origin_velocity_inertial_mps: DVec3) -> Self {
        Self {
            origin_inertial_m,
            origin_velocity_inertial_mps,
            orientation_local_to_inertial: DQuat::IDENTITY,
        }
    }

    pub fn new(
        origin_inertial_m: DVec3,
        origin_velocity_inertial_mps: DVec3,
        orientation_local_to_inertial: DQuat,
    ) -> Result<Self, CollisionBackendError> {
        let frame = Self {
            origin_inertial_m,
            origin_velocity_inertial_mps,
            orientation_local_to_inertial,
        };
        frame.validate()?;
        Ok(frame)
    }

    pub fn validate(self) -> Result<(), CollisionBackendError> {
        if !self.origin_inertial_m.is_finite()
            || !self.origin_velocity_inertial_mps.is_finite()
            || !self.orientation_local_to_inertial.is_finite()
        {
            return Err(CollisionBackendError::InvalidFrame(
                "collision frame contains a non-finite value".into(),
            ));
        }
        let error = (self.orientation_local_to_inertial.length_squared() - 1.0).abs();
        if error > QUATERNION_TOLERANCE {
            return Err(CollisionBackendError::InvalidFrame(
                "collision-frame orientation must be a unit quaternion".into(),
            ));
        }
        Ok(())
    }

    fn local_from_inertial(self) -> DQuat {
        self.orientation_local_to_inertial.inverse()
    }

    /// Authoritative inertial point into contact-local coordinates. The
    /// flight loop uses this to place ephemeris-derived terrain patches.
    pub fn position_to_local(self, inertial_m: DVec3) -> DVec3 {
        self.local_from_inertial() * (inertial_m - self.origin_inertial_m)
    }

    fn position_to_inertial(self, local_m: DVec3) -> DVec3 {
        self.origin_inertial_m + self.orientation_local_to_inertial * local_m
    }

    fn velocity_to_local(self, inertial_mps: DVec3) -> DVec3 {
        self.local_from_inertial() * (inertial_mps - self.origin_velocity_inertial_mps)
    }

    fn velocity_to_inertial(self, local_mps: DVec3) -> DVec3 {
        self.origin_velocity_inertial_mps + self.orientation_local_to_inertial * local_mps
    }

    fn vector_to_local(self, inertial: DVec3) -> DVec3 {
        self.local_from_inertial() * inertial
    }

    /// Authoritative inertial direction into contact-local axes.
    pub fn direction_to_local(self, inertial: DVec3) -> DVec3 {
        self.vector_to_local(inertial)
    }

    /// Authoritative body-to-inertial orientation into contact-local
    /// orientation. Mirrors the conversion applied on body insertion.
    pub fn orientation_to_local(self, orientation_body_to_inertial: DQuat) -> DQuat {
        (self.local_from_inertial() * orientation_body_to_inertial).normalize()
    }

    fn vector_to_inertial(self, local: DVec3) -> DVec3 {
        self.orientation_local_to_inertial * local
    }
}

/// External physical load for one collision step, expressed in the global
/// inertial frame. Contact impulses are added by Rapier; no gameplay
/// coefficient is hidden in this structure.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ExternalWrench {
    pub force_inertial_n: DVec3,
    pub torque_inertial_nm: DVec3,
}

impl ExternalWrench {
    pub const ZERO: Self = Self {
        force_inertial_n: DVec3::ZERO,
        torque_inertial_nm: DVec3::ZERO,
    };

    pub fn validate(self) -> Result<(), CollisionBackendError> {
        if !self.force_inertial_n.is_finite() || !self.torque_inertial_nm.is_finite() {
            return Err(CollisionBackendError::InvalidWrench(
                "external wrench contains a non-finite value".into(),
            ));
        }
        Ok(())
    }

    /// Convert the existing flight-force result into the load Rapier must see.
    ///
    /// `FlightForces::total_force_inertial_n` contains aero/propulsion/other
    /// forces, while gravity is represented separately as an acceleration in
    /// sim-core. Rapier receives their physical sum as force. The flight moment
    /// is body-frame, so it is rotated into the inertial frame here.
    pub fn from_flight_forces(
        state: RigidBodyState,
        properties: RigidBodyProperties,
        gravity_acceleration_inertial_mps2: DVec3,
        forces: &FlightForces,
    ) -> Result<Self, CollisionBackendError> {
        let wrench = Self {
            force_inertial_n: forces.total_force_inertial_n
                + gravity_acceleration_inertial_mps2 * properties.mass_kg,
            torque_inertial_nm: state.orientation_body_to_inertial * forces.total_moment_body_nm,
        };
        wrench.validate()?;
        Ok(wrench)
    }
}

/// Per-body solver policy. Full CCD is useful for fast vehicle-to-vehicle
/// contacts; Rapier already performs automatic CCD against fixed geometry for
/// sufficiently fast dynamic bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DynamicBodyConfig {
    pub full_ccd: bool,
    pub can_sleep: bool,
}

impl Default for DynamicBodyConfig {
    fn default() -> Self {
        Self {
            full_ccd: true,
            can_sleep: true,
        }
    }
}

struct DynamicBodyEntry {
    rapier: RigidBodyHandle,
    colliders: Vec<ColliderHandle>,
    last_wrench: ExternalWrench,
}

struct KinematicBodyEntry {
    rapier: RigidBodyHandle,
    collider: ColliderHandle,
    /// AABB of the attached patch in body coordinates: cuboid patches sit at
    /// the origin, trimesh patches carry their precomputed AABB offset.
    shape_offset_m: DVec3,
    half_extents_m: DVec3,
}

struct StaticEntry {
    handle: ColliderHandle,
    center_local_m: DVec3,
    orientation_local: DQuat,
    half_extents_m: DVec3,
}

/// Which side of a contact pair a collider belongs to, in stable Thessa ids.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize)]
pub enum ContactPartyKind {
    Dynamic,
    KinematicTerrain,
    StaticTerrain,
}

/// One attributed side of a contact pair. No Rapier handles cross the API.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize)]
pub struct ContactParty {
    pub kind: ContactPartyKind,
    pub id_raw: u64,
}

/// One touching pair reduced to physical load evidence: deepest penetration,
/// world normal, and approach speed along it. This is the explicit boundary
/// where a future structural/damage model consumes solver output: the
/// backend reports loads, never damage verdicts.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct ContactSummary {
    pub a: ContactParty,
    pub b: ContactParty,
    pub normal_inertial: [f64; 3],
    pub penetration_m: f64,
    /// `(v_a - v_b) . normal`, positive while the pair closes. Static sides
    /// contribute zero velocity.
    pub approach_speed_mps: f64,
}

/// Constraint load transmitted by one impulse joint during the most recent
/// contact step. Rapier's generalized impulse is divided by the caller's step
/// duration; translational and rotational channels remain separate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLoadSummary {
    pub joint_id: JointId,
    pub body_a: CollisionBodyId,
    pub body_b: CollisionBodyId,
    /// Resultant constraint force magnitude (N).
    pub force_n: f64,
    /// Resultant constraint moment magnitude (N·m).
    pub torque_nm: f64,
}

/// One reduced-order tire/terrain contact evaluated from Rapier ray geometry.
/// Wheel colliders are deliberately absent from the solid solver path, so this
/// force is the sole tire reaction for the reported wheel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelContactSample {
    pub wheel_index: u16,
    pub contact_point_inertial_m: DVec3,
    pub terrain_normal_inertial: DVec3,
    pub forward_axis_inertial: DVec3,
    pub lateral_axis_inertial: DVec3,
    /// Vehicle/wheel contact-patch velocity relative to terrain, including the
    /// supplied wheel spin rate.
    pub relative_contact_velocity_inertial_mps: DVec3,
    pub radial_penetration_m: f64,
    pub strut_compression_m: f64,
    pub tire_compression_m: f64,
    /// Positive while the terrain and wheel are closing along the surface
    /// normal.
    pub compression_rate_mps: f64,
    pub normal_load_n: f64,
    pub longitudinal_force_n: f64,
    pub lateral_force_n: f64,
    pub contact_friction: f64,
    pub saturated: bool,
}

/// Tire contacts and their summed external wrench about the vehicle center of
/// mass. The caller adds this wrench to gravity/aero/propulsion before the same
/// Rapier step; the query itself never advances or mutates world state.
#[derive(Debug, Clone, PartialEq)]
pub struct WheelContactResult {
    pub contacts: Vec<WheelContactSample>,
    pub wrench: ExternalWrench,
}

/// Reduced footpad/terrain contact from a ray aligned with a deployed support.
/// The absorber is evaluated once per contact; terrain supplies the normal
/// direction while the strut law supplies the axial load.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingLegContactSample {
    pub leg_index: u16,
    pub contact_point_inertial_m: DVec3,
    pub terrain_normal_inertial: DVec3,
    pub relative_contact_velocity_inertial_mps: DVec3,
    pub compression_m: f64,
    pub compression_rate_mps: f64,
    pub axial_force_n: f64,
    pub normal_load_n: f64,
    pub tangential_force_inertial_n: DVec3,
    pub contact_friction: f64,
    pub permanent_crush_m: f64,
    pub absorbed_energy_delta_j: f64,
    pub actuator_resisting_torque_nm: f64,
    pub bottomed_out: bool,
    pub exhausted: bool,
    pub saturated: bool,
}

/// Landing-leg load evidence, updated absorber states, and vehicle wrench.
#[derive(Debug, Clone, PartialEq)]
pub struct LandingLegContactResult {
    pub contacts: Vec<LandingLegContactSample>,
    pub states: Vec<LandingLegState>,
    pub wrench: ExternalWrench,
}

/// Deployment target applied while evaluating fold-out landing-leg contacts.
/// `All` is allocation-free for legacy/group controls; `PerLeg` lets part
/// commands independently control each authored support.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandingLegDeploymentCommand<'a> {
    All(bool),
    PerLeg(&'a [bool]),
}

/// Backend binding for a wheel rigid body attached to the sprung vehicle by a
/// slider/spin joint. The nominal center is expressed from the sprung body's
/// center at full strut extension.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArticulatedWheelBinding {
    pub wheel_body: CollisionBodyId,
    pub wheel_index: u16,
    pub nominal_center_sprung_local_m: DVec3,
    pub slide_axis_sprung_local: DVec3,
    pub axle_axis_sprung_local: DVec3,
}

/// Tire and strut reactions for one articulated wheel. The tire wrench acts on
/// the unsprung body; `sprung_wrench` is the equal-and-opposite strut reaction
/// on the chassis. Dynamic terrain remains excluded from this reduced model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArticulatedWheelForce {
    pub wheel_body: CollisionBodyId,
    pub wheel_index: u16,
    pub spin_rate_rad_s: f64,
    pub contact: Option<WheelContactSample>,
    pub wheel_wrench: ExternalWrench,
    pub sprung_wrench: ExternalWrench,
}

/// One terrain patch reduced to a debug box: live inertial center,
/// orientation, and half extents. Trimesh patches report their AABB.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct PatchDebug {
    pub party: ContactParty,
    pub center_inertial_m: [f64; 3],
    pub half_extents_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

/// Debug snapshot of one solved body, expressed in authoritative Thessa terms.
///
/// Positions and velocities are inertial (SI/f64); `sleeping` mirrors the
/// solver's sleep state so landing/settle cases can be inspected without
/// reading Rapier types.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct CollisionBodyDebug {
    pub id_raw: u64,
    pub kinematic: bool,
    pub position_inertial_m: [f64; 3],
    pub velocity_inertial_mps: [f64; 3],
    pub sleeping: bool,
}

/// Telemetry snapshot of a contact scene for debug tooling and regression
/// fixtures. It contains no Rapier handles: every identifier is a stable
/// Thessa-side id.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct CollisionDebugSnapshot {
    pub dynamic_bodies: Vec<CollisionBodyDebug>,
    pub kinematic_bodies: Vec<CollisionBodyDebug>,
    pub fixed_collider_count: usize,
    pub patches: Vec<PatchDebug>,
    pub contacts: Vec<ContactSummary>,
    pub active_contact_pairs: usize,
    pub touching_contact_pairs: usize,
}

/// Maximum contact summaries per snapshot/drain. Pair counts are unbounded
/// telemetry; per-pair details are capped so a debris pile cannot turn
/// debug output into a memory event.
pub const MAX_CONTACT_SUMMARIES: usize = 64;

/// Transient local contact scene.
///
/// It is rebuilt/synchronized from authoritative simulation state when a body
/// enters the contact-active regime. Authoritative persistence must serialize
/// Thessa state, not this structure.
pub struct CollisionWorld {
    frame: CollisionFrame,
    pipeline: PhysicsPipeline,
    integration: IntegrationParameters,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    dynamic: BTreeMap<CollisionBodyId, DynamicBodyEntry>,
    fixed: BTreeMap<StaticColliderId, StaticEntry>,
    kinematic: BTreeMap<KinematicBodyId, KinematicBodyEntry>,
    joints: BTreeMap<JointId, JointEntry>,
    last_contacts: Vec<ContactSummary>,
    next_body_id: u64,
    next_static_id: u64,
    next_kinematic_id: u64,
    next_joint_id: u64,
}

impl CollisionWorld {
    /// Evaluate wheel contacts, add their wrenches to caller-supplied external
    /// loads, and advance the scene exactly once. Multiple wheel chassis may
    /// attach to one vehicle body. Each input carries one scalar spin rate per
    /// wheel station, owned by the caller's authoritative vehicle state.
    pub fn step_with_wheel_contacts<'a, I, W>(
        &mut self,
        step_s: f64,
        wrenches: I,
        wheel_chassis: W,
    ) -> Result<Vec<WheelContactResult>, CollisionBackendError>
    where
        I: IntoIterator<Item = (CollisionBodyId, ExternalWrench)>,
        W: IntoIterator<Item = (CollisionBodyId, &'a CompiledWheelChassis, &'a [f64])>,
    {
        let mut accumulated = BTreeMap::new();
        for (body_id, wrench) in wrenches {
            wrench.validate()?;
            if !self.dynamic.contains_key(&body_id) {
                return Err(CollisionBackendError::UnknownBody(body_id));
            }
            if accumulated.insert(body_id, wrench).is_some() {
                return Err(CollisionBackendError::DuplicateWrench(body_id));
            }
        }

        let mut results = Vec::new();
        for (body_id, chassis, wheel_spin_rad_s) in wheel_chassis {
            let result = self.evaluate_wheel_contacts(body_id, chassis, wheel_spin_rad_s)?;
            let accumulated_wrench = accumulated.entry(body_id).or_insert(ExternalWrench::ZERO);
            accumulated_wrench.force_inertial_n += result.wrench.force_inertial_n;
            accumulated_wrench.torque_inertial_nm += result.wrench.torque_inertial_nm;
            accumulated_wrench.validate()?;
            results.push(result);
        }
        self.step(step_s, accumulated)?;
        Ok(results)
    }

    /// Advance one contact step with an explicit per-body external wrench.
    ///
    /// Every body's previous user force/torque is cleared first, so omitting an
    /// id means zero external load for this step rather than accidentally
    /// reusing stale thrust. A body is woken only when its wrench materially
    /// changes; a landed vehicle under steady gravity may therefore sleep.
    pub fn step<I>(&mut self, step_s: f64, wrenches: I) -> Result<(), CollisionBackendError>
    where
        I: IntoIterator<Item = (CollisionBodyId, ExternalWrench)>,
    {
        if !step_s.is_finite() || step_s < MIN_STEP_S {
            return Err(CollisionBackendError::InvalidStep(step_s));
        }
        let mut requested = BTreeMap::new();
        for (id, wrench) in wrenches {
            wrench.validate()?;
            if !self.dynamic.contains_key(&id) {
                return Err(CollisionBackendError::UnknownBody(id));
            }
            if requested.insert(id, wrench).is_some() {
                return Err(CollisionBackendError::DuplicateWrench(id));
            }
        }

        for (id, entry) in &mut self.dynamic {
            let wrench = requested.get(id).copied().unwrap_or(ExternalWrench::ZERO);
            let body = self
                .bodies
                .get_mut(entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLost(*id))?;
            body.reset_forces(false);
            body.reset_torques(false);
            let wake = wrench_materially_changed(entry.last_wrench, wrench);
            body.add_force(
                to_rapier_vector(self.frame.vector_to_local(wrench.force_inertial_n)),
                wake,
            );
            body.add_torque(
                to_rapier_vector(self.frame.vector_to_local(wrench.torque_inertial_nm)),
                wake,
            );
            entry.last_wrench = wrench;
        }

        self.integration.dt = step_s;
        self.pipeline.step(
            Vector::ZERO,
            &self.integration,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
        self.frame.origin_inertial_m += self.frame.origin_velocity_inertial_mps * step_s;
        self.last_contacts = self.contact_summaries();
        Ok(())
    }

    /// Read a solved body back into the authoritative Thessa representation.
    pub fn body_state(&self, id: CollisionBodyId) -> Result<RigidBodyState, CollisionBackendError> {
        let entry = self
            .dynamic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        let body = self
            .bodies
            .get(entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLost(id))?;
        let pose = body.position();
        let local_position = from_rapier_vector(pose.translation);
        let local_orientation = from_rapier_rotation(pose.rotation);
        let orientation_body_to_inertial =
            (self.frame.orientation_local_to_inertial * local_orientation).normalize();
        let velocity_inertial_mps = self
            .frame
            .velocity_to_inertial(from_rapier_vector(body.linvel()));
        let angular_velocity_inertial_rps = self
            .frame
            .vector_to_inertial(from_rapier_vector(body.angvel()));
        let angular_velocity_body_rps =
            orientation_body_to_inertial.inverse() * angular_velocity_inertial_rps;
        RigidBodyState::new(
            self.frame.position_to_inertial(local_position),
            velocity_inertial_mps,
            orientation_body_to_inertial,
            angular_velocity_body_rps,
        )
        .map_err(|error| CollisionBackendError::InvalidSolvedState(error.to_string()))
    }
}

fn collider_builder(shape: CollisionShape) -> ColliderBuilder {
    match shape {
        CollisionShape::Sphere { radius_m } => ColliderBuilder::ball(radius_m),
        CollisionShape::Cuboid { half_extents_m } => {
            ColliderBuilder::cuboid(half_extents_m.x, half_extents_m.y, half_extents_m.z)
        }
        CollisionShape::Capsule {
            axis,
            half_segment_m,
            radius_m,
        } => match axis {
            CollisionAxis::X => ColliderBuilder::capsule_x(half_segment_m, radius_m),
            CollisionAxis::Y => ColliderBuilder::capsule_y(half_segment_m, radius_m),
            CollisionAxis::Z => ColliderBuilder::capsule_z(half_segment_m, radius_m),
        },
    }
}

fn validate_properties(properties: RigidBodyProperties) -> Result<(), CollisionBackendError> {
    if !properties.mass_kg.is_finite()
        || properties.mass_kg <= MIN_MASS_KG
        || !properties.inertia_body_kg_m2.is_finite()
    {
        return Err(CollisionBackendError::InvalidMassProperties);
    }
    let matrix = properties.inertia_body_kg_m2;
    let entries = [
        matrix.x_axis.x,
        matrix.x_axis.y,
        matrix.x_axis.z,
        matrix.y_axis.x,
        matrix.y_axis.y,
        matrix.y_axis.z,
        matrix.z_axis.x,
        matrix.z_axis.y,
        matrix.z_axis.z,
    ];
    let scale = entries
        .iter()
        .map(|entry| entry.abs())
        .fold(1.0_f64, f64::max);
    let symmetric_tolerance = 1.0e-10 * scale;
    let symmetric = (matrix.x_axis.y - matrix.y_axis.x).abs() <= symmetric_tolerance
        && (matrix.x_axis.z - matrix.z_axis.x).abs() <= symmetric_tolerance
        && (matrix.y_axis.z - matrix.z_axis.y).abs() <= symmetric_tolerance;
    let leading_minor_2 = matrix.x_axis.x * matrix.y_axis.y - matrix.y_axis.x.powi(2);
    let determinant = matrix.determinant();
    if !symmetric
        || !leading_minor_2.is_finite()
        || leading_minor_2 <= MIN_INERTIA
        || matrix.x_axis.x <= MIN_INERTIA
        || !determinant.is_finite()
        || determinant <= MIN_INERTIA
    {
        return Err(CollisionBackendError::InvalidMassProperties);
    }
    Ok(())
}

fn validate_body_state(state: RigidBodyState) -> Result<(), CollisionBackendError> {
    if !state.position_inertial_m.is_finite()
        || !state.velocity_inertial_mps.is_finite()
        || !state.orientation_body_to_inertial.is_finite()
        || !state.angular_velocity_body_rps.is_finite()
    {
        return Err(CollisionBackendError::InvalidBodyState(
            "rigid-body state contains a non-finite value".into(),
        ));
    }
    if (state.orientation_body_to_inertial.length_squared() - 1.0).abs() > QUATERNION_TOLERANCE {
        return Err(CollisionBackendError::InvalidBodyState(
            "body orientation must be a unit quaternion".into(),
        ));
    }
    Ok(())
}

fn body_state_in_frame(
    frame: CollisionFrame,
    state: RigidBodyState,
) -> Result<(DVec3, DQuat, DVec3, DVec3), CollisionBackendError> {
    let local_position = frame.position_to_local(state.position_inertial_m);
    let local_orientation = frame.orientation_to_local(state.orientation_body_to_inertial);
    let local_linear_velocity = frame.velocity_to_local(state.velocity_inertial_mps);
    let local_angular_velocity =
        frame.vector_to_local(state.orientation_body_to_inertial * state.angular_velocity_body_rps);
    if !local_position.is_finite()
        || !local_orientation.is_finite()
        || !local_linear_velocity.is_finite()
        || !local_angular_velocity.is_finite()
    {
        return Err(CollisionBackendError::InvalidBodyState(
            "rigid-body state is not representable in the collision frame".into(),
        ));
    }
    Ok((
        local_position,
        local_orientation,
        local_linear_velocity,
        local_angular_velocity,
    ))
}

fn validate_local_pose(position: DVec3, orientation: DQuat) -> Result<(), CollisionBackendError> {
    if !position.is_finite() || !orientation.is_finite() {
        return Err(CollisionBackendError::InvalidGeometry(
            "local collider pose contains a non-finite value".into(),
        ));
    }
    if (orientation.length_squared() - 1.0).abs() > QUATERNION_TOLERANCE {
        return Err(CollisionBackendError::InvalidGeometry(
            "local collider orientation must be a unit quaternion".into(),
        ));
    }
    Ok(())
}

fn validate_local_vector(value: DVec3, label: &str) -> Result<(), CollisionBackendError> {
    if !value.is_finite() {
        return Err(CollisionBackendError::InvalidGeometry(format!(
            "{label} contains a non-finite value"
        )));
    }
    Ok(())
}

fn wheel_joint_frame(
    slide_axis: DVec3,
    axle_axis: DVec3,
    label: &str,
) -> Result<DQuat, CollisionBackendError> {
    if !slide_axis.is_finite()
        || !axle_axis.is_finite()
        || slide_axis.length_squared() <= QUATERNION_TOLERANCE
        || axle_axis.length_squared() <= QUATERNION_TOLERANCE
    {
        return Err(CollisionBackendError::InvalidGeometry(format!(
            "{label} must be finite and non-zero"
        )));
    }
    let x = slide_axis.normalize();
    let axle = axle_axis.normalize();
    let projected_axle = axle - x * axle.dot(x);
    if projected_axle.length_squared() <= QUATERNION_TOLERANCE {
        return Err(CollisionBackendError::InvalidGeometry(format!(
            "{label} must be orthogonal"
        )));
    }
    let y = projected_axle.normalize();
    let z = x.cross(y).normalize();
    Ok(DQuat::from_mat3(&DMat3::from_cols(x, y, z)).normalize())
}

/// Reject out-of-range triangle indices before touching the mesh builder:
/// the underlying geometry crate indexes vertices directly and panics on a
/// bad index instead of returning an error.
fn validate_trimesh_indices(
    vertices: &[DVec3],
    indices: &[[u32; 3]],
) -> Result<(), CollisionBackendError> {
    let vertex_count = vertices.len() as u32;
    if indices.iter().flatten().any(|index| *index >= vertex_count) {
        return Err(CollisionBackendError::InvalidGeometry(
            "terrain triangle mesh references a missing vertex".into(),
        ));
    }
    Ok(())
}

/// Axis-aligned bounding box of a terrain mesh: (center, half extents).
/// Debug boxes and gizmos draw this instead of the full mesh.
fn trimesh_aabb(vertices: &[DVec3]) -> (DVec3, DVec3) {
    let mut min = DVec3::splat(f64::INFINITY);
    let mut max = DVec3::splat(f64::NEG_INFINITY);
    for vertex in vertices {
        min = min.min(*vertex);
        max = max.max(*vertex);
    }
    let center = (min + max) * 0.5;
    let half = ((max - min) * 0.5).max(DVec3::splat(1.0e-6));
    (center, half)
}

fn wrench_materially_changed(previous: ExternalWrench, next: ExternalWrench) -> bool {
    vector_materially_changed(previous.force_inertial_n, next.force_inertial_n)
        || vector_materially_changed(previous.torque_inertial_nm, next.torque_inertial_nm)
}

fn vector_materially_changed(previous: DVec3, next: DVec3) -> bool {
    let delta = (next - previous).length();
    let scale = previous.length().max(next.length()).max(1.0);
    delta > WRENCH_CHANGE_ABS + WRENCH_CHANGE_REL * scale
}

fn to_rapier_vector(value: DVec3) -> Vector {
    Vector::new(value.x, value.y, value.z)
}

fn from_rapier_vector(value: Vector) -> DVec3 {
    DVec3::new(value.x, value.y, value.z)
}

fn to_rapier_rotation(value: DQuat) -> Rotation {
    Rotation::from_xyzw(value.x, value.y, value.z, value.w)
}

fn from_rapier_rotation(value: Rotation) -> DQuat {
    DQuat::from_xyzw(value.x, value.y, value.z, value.w)
}

fn to_rapier_matrix(value: DMat3) -> Matrix {
    Matrix::from_cols(
        to_rapier_vector(value.x_axis),
        to_rapier_vector(value.y_axis),
        to_rapier_vector(value.z_axis),
    )
}

#[derive(Debug, Clone, PartialEq)]
pub enum CollisionBackendError {
    InvalidFrame(String),
    InvalidGeometry(String),
    InvalidWrench(String),
    InvalidBodyState(String),
    InvalidMassProperties,
    InvalidWheelContact(String),
    InvalidLandingLegContact(String),
    InvalidStep(f64),
    UnknownBody(CollisionBodyId),
    UnknownKinematicBody(KinematicBodyId),
    UnknownStaticCollider(StaticColliderId),
    UnknownJoint(JointId),
    DuplicateWrench(CollisionBodyId),
    BackendStateLost(CollisionBodyId),
    BackendStateLostKinematic(KinematicBodyId),
    InvalidSolvedState(String),
    IdentifierExhausted,
}

impl fmt::Display for CollisionBackendError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFrame(message) => write!(formatter, "invalid collision frame: {message}"),
            Self::InvalidGeometry(message) => {
                write!(formatter, "invalid collision geometry: {message}")
            }
            Self::InvalidWrench(message) => {
                write!(formatter, "invalid collision wrench: {message}")
            }
            Self::InvalidBodyState(message) => {
                write!(formatter, "invalid collision body state: {message}")
            }
            Self::InvalidMassProperties => write!(formatter, "invalid rigid-body mass properties"),
            Self::InvalidWheelContact(message) => {
                write!(formatter, "invalid wheel contact: {message}")
            }
            Self::InvalidLandingLegContact(message) => {
                write!(formatter, "invalid landing-leg contact: {message}")
            }
            Self::InvalidStep(step) => write!(formatter, "invalid collision step duration: {step}"),
            Self::UnknownBody(id) => write!(formatter, "unknown collision body {}", id.raw()),
            Self::UnknownKinematicBody(id) => {
                write!(formatter, "unknown kinematic collision body {}", id.raw())
            }
            Self::UnknownStaticCollider(id) => {
                write!(formatter, "unknown static collider {}", id.raw())
            }
            Self::UnknownJoint(id) => {
                write!(formatter, "unknown docking joint {}", id.raw())
            }
            Self::DuplicateWrench(id) => {
                write!(
                    formatter,
                    "duplicate wrench for collision body {}",
                    id.raw()
                )
            }
            Self::BackendStateLost(id) => {
                write!(
                    formatter,
                    "Rapier body missing for collision body {}",
                    id.raw()
                )
            }
            Self::BackendStateLostKinematic(id) => {
                write!(
                    formatter,
                    "Rapier body missing for kinematic collision body {}",
                    id.raw()
                )
            }
            Self::InvalidSolvedState(message) => {
                write!(formatter, "invalid solved collision state: {message}")
            }
            Self::IdentifierExhausted => write!(formatter, "collision identifier space exhausted"),
        }
    }
}

impl Error for CollisionBackendError {}

#[cfg(test)]
mod joint_and_replay_tests;
#[cfg(test)]
mod tests;
