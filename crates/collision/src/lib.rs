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
    CollisionAxis, CollisionGeometry, CollisionMaterial, CollisionShape, CompiledWheelChassis,
    FlightForces, RigidBodyProperties, RigidBodyState,
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
    pub fn new(frame: CollisionFrame) -> Result<Self, CollisionBackendError> {
        frame.validate()?;
        // Rapier's game-tuned default clamps linear velocity to 400 m/s.
        // Thessa flies co-moving orbital velocities far above that, so the
        // clamp would silently rewrite authoritative state (and trip the
        // flight solver bounds). SI/f64 needs no solver-imposed speed limit:
        // disable it. Tunnelling stays covered by CCD, not by a cap.
        let integration = IntegrationParameters {
            normalized_max_linear_velocity: f64::MAX,
            ..Default::default()
        };
        Ok(Self {
            frame,
            pipeline: PhysicsPipeline::new(),
            integration,
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            dynamic: BTreeMap::new(),
            fixed: BTreeMap::new(),
            kinematic: BTreeMap::new(),
            joints: BTreeMap::new(),
            last_contacts: Vec::new(),
            next_body_id: 0,
            next_static_id: 0,
            next_kinematic_id: 0,
            next_joint_id: 0,
        })
    }

    pub const fn frame(&self) -> CollisionFrame {
        self.frame
    }

    pub fn dynamic_body_count(&self) -> usize {
        self.dynamic.len()
    }

    pub fn fixed_collider_count(&self) -> usize {
        self.fixed.len()
    }

    pub fn kinematic_body_count(&self) -> usize {
        self.kinematic.len()
    }

    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Validate a prospective dynamic body without mutating the contact
    /// scene. State and mass properties are public wire/domain structs, so
    /// callers that must perform cleanup before insertion can preflight the
    /// same checks used by the insertion path.
    pub fn validate_dynamic_body_inputs(
        &self,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
    ) -> Result<(), CollisionBackendError> {
        validate_body_state(state)?;
        if geometry.is_empty() {
            return Err(CollisionBackendError::InvalidGeometry(
                "dynamic contact body needs at least one collision part".into(),
            ));
        }
        geometry
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
        validate_properties(properties)?;
        body_state_in_frame(self.frame, state)?;
        Ok(())
    }

    /// Insert one authoritative rigid body and attach its backend-neutral
    /// collision primitives. Collider density is zero because sim-core's mass
    /// and inertia are authoritative and are installed explicitly on the body.
    pub fn insert_dynamic_body(
        &mut self,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
        config: DynamicBodyConfig,
    ) -> Result<CollisionBodyId, CollisionBackendError> {
        self.insert_dynamic_body_with_sensor(state, properties, geometry, config, false)
    }

    /// Insert a dynamic body whose colliders participate in geometric queries
    /// but produce no solid solver impulses. Used for articulated wheel tires:
    /// the tire law owns the terrain reaction, while Rapier integrates the
    /// wheel body's mass and suspension constraints.
    pub fn insert_dynamic_sensor_body(
        &mut self,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
        config: DynamicBodyConfig,
    ) -> Result<CollisionBodyId, CollisionBackendError> {
        self.insert_dynamic_body_with_sensor(state, properties, geometry, config, true)
    }

    fn insert_dynamic_body_with_sensor(
        &mut self,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        geometry: &CollisionGeometry,
        config: DynamicBodyConfig,
        sensor: bool,
    ) -> Result<CollisionBodyId, CollisionBackendError> {
        self.validate_dynamic_body_inputs(state, properties, geometry)?;
        let (local_position, local_orientation, local_linear_velocity, local_angular_velocity) =
            body_state_in_frame(self.frame, state)?;

        let id = CollisionBodyId(self.next_body_id);
        self.next_body_id = self
            .next_body_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;

        let mass_properties = MassProperties::with_inertia_matrix(
            Vector::ZERO,
            properties.mass_kg,
            to_rapier_matrix(properties.inertia_body_kg_m2),
        );
        let rigid_body = RigidBodyBuilder::dynamic()
            .pose(Pose::from_parts(
                to_rapier_vector(local_position),
                to_rapier_rotation(local_orientation),
            ))
            .linvel(to_rapier_vector(local_linear_velocity))
            .angvel(to_rapier_vector(local_angular_velocity))
            .additional_mass_properties(mass_properties)
            .gravity_scale(0.0)
            .gyroscopic_forces_enabled(true)
            .ccd_enabled(config.full_ccd)
            .can_sleep(config.can_sleep)
            .user_data(id.raw() as u128)
            .build();
        let handle = self.bodies.insert(rigid_body);

        let mut colliders = Vec::with_capacity(geometry.parts.len());
        for part in &geometry.parts {
            let collider = collider_builder(part.shape)
                .density(0.0)
                .friction(part.material.friction)
                .restitution(part.material.restitution)
                .sensor(sensor)
                .position(Pose::from_parts(
                    to_rapier_vector(part.local_position_m),
                    to_rapier_rotation(part.local_orientation),
                ))
                .build();
            colliders.push(
                self.colliders
                    .insert_with_parent(collider, handle, &mut self.bodies),
            );
        }

        self.dynamic.insert(
            id,
            DynamicBodyEntry {
                rapier: handle,
                colliders,
                last_wrench: ExternalWrench::ZERO,
            },
        );
        Ok(id)
    }

    /// Re-mirror an existing backend body from fresh authoritative data
    /// without rebuilding its compound. Pose and velocity are teleported
    /// (regime entry, never a mid-contact correction), mass/inertia are
    /// reinstalled, and the CCD policy is updated. A changed sleep policy
    /// still requires remove + insert through the caller.
    pub fn resync_dynamic_body(
        &mut self,
        id: CollisionBodyId,
        state: RigidBodyState,
        properties: RigidBodyProperties,
        config: DynamicBodyConfig,
    ) -> Result<(), CollisionBackendError> {
        validate_body_state(state)?;
        validate_properties(properties)?;
        let (local_position, local_orientation, local_linear_velocity, local_angular_velocity) =
            body_state_in_frame(self.frame, state)?;
        let entry = self
            .dynamic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        let handle = entry.rapier;
        let body = self
            .bodies
            .get_mut(handle)
            .ok_or(CollisionBackendError::BackendStateLost(id))?;
        body.set_position(
            Pose::from_parts(
                to_rapier_vector(local_position),
                to_rapier_rotation(local_orientation),
            ),
            true,
        );
        body.set_linvel(to_rapier_vector(local_linear_velocity), true);
        body.set_angvel(to_rapier_vector(local_angular_velocity), true);
        body.set_additional_mass_properties(
            MassProperties::with_inertia_matrix(
                Vector::ZERO,
                properties.mass_kg,
                to_rapier_matrix(properties.inertia_body_kg_m2),
            ),
            true,
        );
        body.enable_ccd(config.full_ccd);
        Ok(())
    }

    /// Fixed cuboid convenience path for pads, test floors and coarse terrain
    /// proxies. Production rotating terrain should use a kinematic world-body
    /// integration so its ephemeris-derived surface velocity participates in
    /// contacts.
    pub fn insert_static_cuboid(
        &mut self,
        center_local_m: DVec3,
        orientation_local: DQuat,
        half_extents_m: DVec3,
        material: CollisionMaterial,
    ) -> Result<StaticColliderId, CollisionBackendError> {
        validate_local_pose(center_local_m, orientation_local)?;
        if !half_extents_m.is_finite()
            || half_extents_m.x <= 0.0
            || half_extents_m.y <= 0.0
            || half_extents_m.z <= 0.0
        {
            return Err(CollisionBackendError::InvalidGeometry(
                "static cuboid half-extents must be finite and positive".into(),
            ));
        }
        material
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;

        let collider =
            ColliderBuilder::cuboid(half_extents_m.x, half_extents_m.y, half_extents_m.z)
                .friction(material.friction)
                .restitution(material.restitution)
                .position(Pose::from_parts(
                    to_rapier_vector(center_local_m),
                    to_rapier_rotation(orientation_local),
                ))
                .build();
        let handle = self.colliders.insert(collider);
        self.register_fixed(handle, center_local_m, orientation_local, half_extents_m)
    }

    /// Insert an already-localized terrain triangle mesh. Keep terrain
    /// generation outside this crate: the authoritative world/terrain system
    /// decides which patch is required and supplies physical vertices here.
    pub fn insert_static_trimesh(
        &mut self,
        vertices_local_m: Vec<DVec3>,
        indices: Vec<[u32; 3]>,
        material: CollisionMaterial,
    ) -> Result<StaticColliderId, CollisionBackendError> {
        if vertices_local_m.is_empty() || indices.is_empty() {
            return Err(CollisionBackendError::InvalidGeometry(
                "terrain triangle mesh must contain vertices and triangles".into(),
            ));
        }
        if vertices_local_m.iter().any(|vertex| !vertex.is_finite()) {
            return Err(CollisionBackendError::InvalidGeometry(
                "terrain triangle mesh contains a non-finite vertex".into(),
            ));
        }
        validate_trimesh_indices(&vertices_local_m, &indices)?;
        material
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
        let (aabb_center_m, aabb_half_m) = trimesh_aabb(&vertices_local_m);
        let vertices = vertices_local_m
            .into_iter()
            .map(to_rapier_vector)
            .collect::<Vec<_>>();
        let builder = ColliderBuilder::trimesh(vertices, indices).map_err(|error| {
            CollisionBackendError::InvalidGeometry(format!(
                "invalid terrain triangle mesh: {error:?}"
            ))
        })?;
        let handle = self.colliders.insert(
            builder
                .friction(material.friction)
                .restitution(material.restitution)
                .build(),
        );
        self.register_fixed(handle, aabb_center_m, DQuat::IDENTITY, aabb_half_m)
    }

    fn register_fixed(
        &mut self,
        handle: ColliderHandle,
        center_local_m: DVec3,
        orientation_local: DQuat,
        half_extents_m: DVec3,
    ) -> Result<StaticColliderId, CollisionBackendError> {
        let id = StaticColliderId(self.next_static_id);
        self.next_static_id = self
            .next_static_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.fixed.insert(
            id,
            StaticEntry {
                handle,
                center_local_m,
                orientation_local,
                half_extents_m,
            },
        );
        Ok(id)
    }

    /// Insert a position-based kinematic cuboid for moving terrain patches,
    /// landing pads on rotating bodies, or scripted obstacles.
    ///
    /// The pose is local to the collision frame. The caller owns the motion:
    /// derive the pose at tick `n+1` from the canonical ephemeris and body
    /// rotation model, then publish it with
    /// [`set_next_kinematic_pose`](Self::set_next_kinematic_pose) before the
    /// step. Rapier derives the surface velocity that enters contacts, so a
    /// rotating planet's ground moves under a landed craft instead of being
    /// pinned to a static plane.
    pub fn insert_kinematic_cuboid(
        &mut self,
        center_local_m: DVec3,
        orientation_local: DQuat,
        half_extents_m: DVec3,
        material: CollisionMaterial,
    ) -> Result<KinematicBodyId, CollisionBackendError> {
        validate_local_pose(center_local_m, orientation_local)?;
        if !half_extents_m.is_finite()
            || half_extents_m.x <= 0.0
            || half_extents_m.y <= 0.0
            || half_extents_m.z <= 0.0
        {
            return Err(CollisionBackendError::InvalidGeometry(
                "kinematic cuboid half-extents must be finite and positive".into(),
            ));
        }
        material
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
        let id = KinematicBodyId(self.next_kinematic_id);
        self.next_kinematic_id = self
            .next_kinematic_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        let body = rapier3d_f64::prelude::RigidBodyBuilder::kinematic_position_based()
            .pose(Pose::from_parts(
                to_rapier_vector(center_local_m),
                to_rapier_rotation(orientation_local),
            ))
            .user_data(id.raw() as u128)
            .build();
        let handle = self.bodies.insert(body);
        let collider =
            ColliderBuilder::cuboid(half_extents_m.x, half_extents_m.y, half_extents_m.z)
                .friction(material.friction)
                .restitution(material.restitution)
                .build();
        let collider = self
            .colliders
            .insert_with_parent(collider, handle, &mut self.bodies);
        self.kinematic.insert(
            id,
            KinematicBodyEntry {
                rapier: handle,
                collider,
                shape_offset_m: DVec3::ZERO,
                half_extents_m,
            },
        );
        Ok(id)
    }

    /// Insert a position-based kinematic terrain triangle mesh. The vertices
    /// are already localized by the terrain system; the returned body carries
    /// the patch so streaming/eviction moves one handle per patch.
    ///
    /// The mesh is fully validated and built before the backend body is
    /// created, so a rejected patch never leaves an untracked body behind.
    pub fn insert_kinematic_trimesh(
        &mut self,
        vertices_local_m: Vec<DVec3>,
        indices: Vec<[u32; 3]>,
        material: CollisionMaterial,
    ) -> Result<KinematicBodyId, CollisionBackendError> {
        if vertices_local_m.is_empty() || indices.is_empty() {
            return Err(CollisionBackendError::InvalidGeometry(
                "terrain triangle mesh must contain vertices and triangles".into(),
            ));
        }
        if vertices_local_m.iter().any(|vertex| !vertex.is_finite()) {
            return Err(CollisionBackendError::InvalidGeometry(
                "terrain triangle mesh contains a non-finite vertex".into(),
            ));
        }
        validate_trimesh_indices(&vertices_local_m, &indices)?;
        material
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
        let (aabb_center_m, aabb_half_m) = trimesh_aabb(&vertices_local_m);
        let vertices = vertices_local_m
            .into_iter()
            .map(to_rapier_vector)
            .collect::<Vec<_>>();
        // Fallible build first: only a valid collider earns a backend body.
        let collider = ColliderBuilder::trimesh(vertices, indices)
            .map_err(|error| {
                CollisionBackendError::InvalidGeometry(format!(
                    "invalid terrain triangle mesh: {error:?}"
                ))
            })?
            .friction(material.friction)
            .restitution(material.restitution)
            .build();
        let id = KinematicBodyId(self.next_kinematic_id);
        self.next_kinematic_id = self
            .next_kinematic_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        let body = rapier3d_f64::prelude::RigidBodyBuilder::kinematic_position_based()
            .user_data(id.raw() as u128)
            .build();
        let handle = self.bodies.insert(body);
        let collider = self
            .colliders
            .insert_with_parent(collider, handle, &mut self.bodies);
        self.kinematic.insert(
            id,
            KinematicBodyEntry {
                rapier: handle,
                collider,
                shape_offset_m: aabb_center_m,
                half_extents_m: aabb_half_m,
            },
        );
        Ok(id)
    }

    /// Publish the ephemeris-derived pose a kinematic body must reach by the
    /// next step. This is a motion prescription, not a teleport: Rapier
    /// interpolates the velocity that carries dynamic bodies in contact.
    pub fn set_next_kinematic_pose(
        &mut self,
        id: KinematicBodyId,
        center_local_m: DVec3,
        orientation_local: DQuat,
    ) -> Result<(), CollisionBackendError> {
        validate_local_pose(center_local_m, orientation_local).map_err(|_| {
            CollisionBackendError::InvalidGeometry(
                "kinematic target pose contains a non-finite value".into(),
            )
        })?;
        let entry = self
            .kinematic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownKinematicBody(id))?;
        let body = self
            .bodies
            .get_mut(entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLostKinematic(id))?;
        body.set_next_kinematic_position(Pose::from_parts(
            to_rapier_vector(center_local_m),
            to_rapier_rotation(orientation_local),
        ));
        Ok(())
    }

    /// Read a kinematic body back in authoritative inertial terms.
    pub fn kinematic_body_state(
        &self,
        id: KinematicBodyId,
    ) -> Result<RigidBodyState, CollisionBackendError> {
        let entry = self
            .kinematic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownKinematicBody(id))?;
        let body = self
            .bodies
            .get(entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLostKinematic(id))?;
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

    /// Remove a dynamic body and its attached colliders. Structural topology
    /// changes, staging, and docking call this before rebuilding the backend
    /// body so no stale compound survives a configuration change. Docking
    /// joints attached to the body are removed with it.
    pub fn remove_dynamic_body(
        &mut self,
        id: CollisionBodyId,
    ) -> Result<(), CollisionBackendError> {
        let entry = self
            .dynamic
            .remove(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        self.bodies.remove(
            entry.rapier,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        self.joints
            .retain(|_, joint| joint.a != id && joint.b != id);
        Ok(())
    }

    /// Remove one static collider. Terrain streaming calls this when a patch
    /// leaves the contact-active envelope.
    pub fn remove_static_collider(
        &mut self,
        id: StaticColliderId,
    ) -> Result<(), CollisionBackendError> {
        let entry = self
            .fixed
            .remove(&id)
            .ok_or(CollisionBackendError::UnknownStaticCollider(id))?;
        self.colliders
            .remove(entry.handle, &mut self.islands, &mut self.bodies, true);
        Ok(())
    }

    /// Remove a kinematic body and its attached patch colliders.
    pub fn remove_kinematic_body(
        &mut self,
        id: KinematicBodyId,
    ) -> Result<(), CollisionBackendError> {
        let entry = self
            .kinematic
            .remove(&id)
            .ok_or(CollisionBackendError::UnknownKinematicBody(id))?;
        self.bodies.remove(
            entry.rapier,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        Ok(())
    }

    /// Rigidly dock two dynamic bodies at their local port frames: the
    /// docking/undocking primitive the flight layer drives around
    /// staging and docking events. Contacts between the joined bodies are
    /// disabled so the constraint never fights contact response; contacts
    /// with everything else are unaffected. Undock with
    /// [`remove_joint`](Self::remove_joint); removing either body drops the
    /// joint with it.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_fixed_joint(
        &mut self,
        a: CollisionBodyId,
        b: CollisionBodyId,
        frame_a_local_position_m: DVec3,
        frame_a_local_orientation: DQuat,
        frame_b_local_position_m: DVec3,
        frame_b_local_orientation: DQuat,
    ) -> Result<JointId, CollisionBackendError> {
        if a == b {
            return Err(CollisionBackendError::InvalidGeometry(
                "docking joint needs two distinct bodies".into(),
            ));
        }
        validate_local_pose(frame_a_local_position_m, frame_a_local_orientation).map_err(|_| {
            CollisionBackendError::InvalidGeometry(
                "docking frame on body A contains a non-finite value".into(),
            )
        })?;
        validate_local_pose(frame_b_local_position_m, frame_b_local_orientation).map_err(|_| {
            CollisionBackendError::InvalidGeometry(
                "docking frame on body B contains a non-finite value".into(),
            )
        })?;
        let handle_a = self
            .dynamic
            .get(&a)
            .ok_or(CollisionBackendError::UnknownBody(a))?
            .rapier;
        let handle_b = self
            .dynamic
            .get(&b)
            .ok_or(CollisionBackendError::UnknownBody(b))?
            .rapier;
        let joint = FixedJointBuilder::new()
            .local_frame1(Pose::from_parts(
                to_rapier_vector(frame_a_local_position_m),
                to_rapier_rotation(frame_a_local_orientation),
            ))
            .local_frame2(Pose::from_parts(
                to_rapier_vector(frame_b_local_position_m),
                to_rapier_rotation(frame_b_local_orientation),
            ))
            .contacts_enabled(false)
            .build();
        let rapier = self.impulse_joints.insert(handle_a, handle_b, joint, true);
        let id = JointId(self.next_joint_id);
        self.next_joint_id = self
            .next_joint_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.joints.insert(id, JointEntry { rapier, a, b });
        Ok(id)
    }

    /// Attach a hinge for a moving mechanism such as a D1 soft-capture petal.
    ///
    /// The two axes are expressed in their respective body-local frames. The
    /// joint locks the two local anchor points together while leaving rotation
    /// about the mapped hinge axis free. Contacts between the mechanism bodies
    /// are disabled so explicit hinge kinematics remain the sole owner of the
    /// mechanism constraint.
    pub fn attach_revolute_joint(
        &mut self,
        a: CollisionBodyId,
        b: CollisionBodyId,
        anchor_a_local_m: DVec3,
        anchor_b_local_m: DVec3,
        axis_a_local_m: DVec3,
        axis_b_local_m: DVec3,
    ) -> Result<JointId, CollisionBackendError> {
        if a == b {
            return Err(CollisionBackendError::InvalidGeometry(
                "revolute joint needs two distinct bodies".into(),
            ));
        }
        validate_local_vector(anchor_a_local_m, "revolute anchor on body A")?;
        validate_local_vector(anchor_b_local_m, "revolute anchor on body B")?;
        if !axis_a_local_m.is_finite()
            || axis_a_local_m.length_squared() <= QUATERNION_TOLERANCE
            || !axis_b_local_m.is_finite()
            || axis_b_local_m.length_squared() <= QUATERNION_TOLERANCE
        {
            return Err(CollisionBackendError::InvalidGeometry(
                "revolute axes must be finite and non-zero".into(),
            ));
        }
        let handle_a = self
            .dynamic
            .get(&a)
            .ok_or(CollisionBackendError::UnknownBody(a))?
            .rapier;
        let handle_b = self
            .dynamic
            .get(&b)
            .ok_or(CollisionBackendError::UnknownBody(b))?
            .rapier;
        let joint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_axis1(to_rapier_vector(axis_a_local_m.normalize()))
            .local_axis2(to_rapier_vector(axis_b_local_m.normalize()))
            .local_anchor1(to_rapier_vector(anchor_a_local_m))
            .local_anchor2(to_rapier_vector(anchor_b_local_m))
            .contacts_enabled(false)
            .build();
        let rapier = self.impulse_joints.insert(handle_a, handle_b, joint, true);
        let id = JointId(self.next_joint_id);
        self.next_joint_id = self
            .next_joint_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.joints.insert(id, JointEntry { rapier, a, b });
        Ok(id)
    }

    /// Attach a bounded slider, suitable for a landing-gear strut or another
    /// telescoping mechanism. Anchors and axes are body-local; limits are
    /// signed translations along the common slider axis in metres. The caller
    /// remains responsible for applying the authored spring/damper wrench.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_prismatic_joint(
        &mut self,
        a: CollisionBodyId,
        b: CollisionBodyId,
        anchor_a_local_m: DVec3,
        anchor_b_local_m: DVec3,
        axis_a_local: DVec3,
        axis_b_local: DVec3,
        limits_m: [f64; 2],
    ) -> Result<JointId, CollisionBackendError> {
        if a == b {
            return Err(CollisionBackendError::InvalidGeometry(
                "prismatic joint needs two distinct bodies".into(),
            ));
        }
        validate_local_vector(anchor_a_local_m, "prismatic anchor on body A")?;
        validate_local_vector(anchor_b_local_m, "prismatic anchor on body B")?;
        if !axis_a_local.is_finite()
            || axis_a_local.length_squared() <= QUATERNION_TOLERANCE
            || !axis_b_local.is_finite()
            || axis_b_local.length_squared() <= QUATERNION_TOLERANCE
        {
            return Err(CollisionBackendError::InvalidGeometry(
                "prismatic axes must be finite and non-zero".into(),
            ));
        }
        if limits_m.iter().any(|limit| !limit.is_finite()) || limits_m[0] > limits_m[1] {
            return Err(CollisionBackendError::InvalidGeometry(
                "prismatic limits must be finite and ordered".into(),
            ));
        }
        let handle_a = self
            .dynamic
            .get(&a)
            .ok_or(CollisionBackendError::UnknownBody(a))?
            .rapier;
        let handle_b = self
            .dynamic
            .get(&b)
            .ok_or(CollisionBackendError::UnknownBody(b))?
            .rapier;
        let joint = PrismaticJointBuilder::new(to_rapier_vector(axis_a_local.normalize()))
            .local_anchor1(to_rapier_vector(anchor_a_local_m))
            .local_anchor2(to_rapier_vector(anchor_b_local_m))
            .local_axis1(to_rapier_vector(axis_a_local.normalize()))
            .local_axis2(to_rapier_vector(axis_b_local.normalize()))
            .limits(limits_m)
            .contacts_enabled(false)
            .build();
        let rapier = self.impulse_joints.insert(handle_a, handle_b, joint, true);
        let id = JointId(self.next_joint_id);
        self.next_joint_id = self
            .next_joint_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.joints.insert(id, JointEntry { rapier, a, b });
        Ok(id)
    }

    /// Attach a wheel body with two deliberately free degrees of freedom:
    /// slider translation along the strut and wheel rotation about its axle.
    /// The caller supplies local axis pairs that are orthogonal within each
    /// body. Joint coordinates are `LIN_X` for suspension travel and `ANG_Y`
    /// for wheel spin; stroke limits are signed translations in metres.
    #[allow(clippy::too_many_arguments)]
    pub fn attach_suspension_wheel_joint(
        &mut self,
        sprung_body: CollisionBodyId,
        wheel_body: CollisionBodyId,
        anchor_sprung_local_m: DVec3,
        anchor_wheel_local_m: DVec3,
        slide_axis_sprung_local: DVec3,
        axle_axis_sprung_local: DVec3,
        slide_axis_wheel_local: DVec3,
        axle_axis_wheel_local: DVec3,
        stroke_limits_m: [f64; 2],
    ) -> Result<JointId, CollisionBackendError> {
        if sprung_body == wheel_body {
            return Err(CollisionBackendError::InvalidGeometry(
                "suspension wheel joint needs two distinct bodies".into(),
            ));
        }
        validate_local_vector(anchor_sprung_local_m, "suspension anchor on sprung body")?;
        validate_local_vector(anchor_wheel_local_m, "suspension anchor on wheel body")?;
        if stroke_limits_m.iter().any(|limit| !limit.is_finite())
            || stroke_limits_m[0] > stroke_limits_m[1]
        {
            return Err(CollisionBackendError::InvalidGeometry(
                "suspension stroke limits must be finite and ordered".into(),
            ));
        }
        let frame_sprung = wheel_joint_frame(
            slide_axis_sprung_local,
            axle_axis_sprung_local,
            "sprung wheel-joint axes",
        )?;
        let frame_wheel = wheel_joint_frame(
            slide_axis_wheel_local,
            axle_axis_wheel_local,
            "wheel wheel-joint axes",
        )?;
        let sprung_handle = self
            .dynamic
            .get(&sprung_body)
            .ok_or(CollisionBackendError::UnknownBody(sprung_body))?
            .rapier;
        let wheel_handle = self
            .dynamic
            .get(&wheel_body)
            .ok_or(CollisionBackendError::UnknownBody(wheel_body))?
            .rapier;
        let locked = JointAxesMask::LIN_Y
            | JointAxesMask::LIN_Z
            | JointAxesMask::ANG_X
            | JointAxesMask::ANG_Z;
        let joint = GenericJointBuilder::new(locked)
            .local_frame1(Pose::from_parts(
                to_rapier_vector(anchor_sprung_local_m),
                to_rapier_rotation(frame_sprung),
            ))
            .local_frame2(Pose::from_parts(
                to_rapier_vector(anchor_wheel_local_m),
                to_rapier_rotation(frame_wheel),
            ))
            .limits(JointAxis::LinX, stroke_limits_m)
            .contacts_enabled(false)
            .build();
        let rapier = self
            .impulse_joints
            .insert(sprung_handle, wheel_handle, joint, true);
        let id = JointId(self.next_joint_id);
        self.next_joint_id = self
            .next_joint_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.joints.insert(
            id,
            JointEntry {
                rapier,
                a: sprung_body,
                b: wheel_body,
            },
        );
        Ok(id)
    }

    /// Undock a fixed joint. Both bodies keep their solved pose/velocity;
    /// the flight layer re-owns them as independent clusters from here.
    pub fn remove_joint(&mut self, id: JointId) -> Result<(), CollisionBackendError> {
        let entry = self
            .joints
            .remove(&id)
            .ok_or(CollisionBackendError::UnknownJoint(id))?;
        self.impulse_joints.remove(entry.rapier, true);
        Ok(())
    }

    /// Number of narrow-phase contact pairs, touching or not.
    pub fn active_contact_pair_count(&self) -> usize {
        self.narrow_phase.contact_pairs().count()
    }

    /// Number of contact pairs with at least one active contact point.
    pub fn touching_contact_pair_count(&self) -> usize {
        self.narrow_phase
            .contact_pairs()
            .filter(|pair| pair.has_any_active_contact())
            .count()
    }

    /// Whether a dynamic body currently sleeps. Landed vehicles under steady
    /// load should sleep; a body that never sleeps under constant wrench
    /// points at bad geometry, friction, or solver parameters rather than at
    /// a need for artificial damping.
    pub fn dynamic_body_sleeping(
        &self,
        id: CollisionBodyId,
    ) -> Result<bool, CollisionBackendError> {
        let entry = self
            .dynamic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        self.bodies
            .get(entry.rapier)
            .map(|body| body.is_sleeping())
            .ok_or(CollisionBackendError::BackendStateLost(id))
    }

    /// Serializable telemetry snapshot for debug tooling, regression
    /// fixtures, and the contact MVP probe. Contains no Rapier types.
    pub fn debug_snapshot(&self) -> Result<CollisionDebugSnapshot, CollisionBackendError> {
        let mut dynamic_bodies = Vec::with_capacity(self.dynamic.len());
        for (id, entry) in &self.dynamic {
            let body = self
                .bodies
                .get(entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLost(*id))?;
            let state = self.body_state(*id)?;
            dynamic_bodies.push(CollisionBodyDebug {
                id_raw: id.raw(),
                kinematic: false,
                position_inertial_m: state.position_inertial_m.to_array(),
                velocity_inertial_mps: state.velocity_inertial_mps.to_array(),
                sleeping: body.is_sleeping(),
            });
        }
        let mut kinematic_bodies = Vec::with_capacity(self.kinematic.len());
        for (id, entry) in &self.kinematic {
            let body = self
                .bodies
                .get(entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLostKinematic(*id))?;
            let state = self.kinematic_body_state(*id)?;
            kinematic_bodies.push(CollisionBodyDebug {
                id_raw: id.raw(),
                kinematic: true,
                position_inertial_m: state.position_inertial_m.to_array(),
                velocity_inertial_mps: state.velocity_inertial_mps.to_array(),
                sleeping: body.is_sleeping(),
            });
        }
        Ok(CollisionDebugSnapshot {
            dynamic_bodies,
            kinematic_bodies,
            fixed_collider_count: self.fixed.len(),
            patches: self.patch_debug()?,
            contacts: self.contact_summaries(),
            active_contact_pairs: self.active_contact_pair_count(),
            touching_contact_pairs: self.touching_contact_pair_count(),
        })
    }

    /// Attribute a narrow-phase collider to its stable Thessa party.
    fn party_of(&self, handle: ColliderHandle) -> Option<ContactParty> {
        for (id, entry) in &self.dynamic {
            if entry.colliders.contains(&handle) {
                return Some(ContactParty {
                    kind: ContactPartyKind::Dynamic,
                    id_raw: id.raw(),
                });
            }
        }
        for (id, entry) in &self.kinematic {
            if entry.collider == handle {
                return Some(ContactParty {
                    kind: ContactPartyKind::KinematicTerrain,
                    id_raw: id.raw(),
                });
            }
        }
        for (id, entry) in &self.fixed {
            if entry.handle == handle {
                return Some(ContactParty {
                    kind: ContactPartyKind::StaticTerrain,
                    id_raw: id.raw(),
                });
            }
        }
        None
    }

    fn party_velocity_inertial_mps(&self, party: ContactParty) -> DVec3 {
        match party.kind {
            ContactPartyKind::Dynamic => self
                .dynamic
                .iter()
                .find(|(id, _)| id.raw() == party.id_raw)
                .and_then(|(id, _)| self.body_state(*id).ok())
                .map(|state| state.velocity_inertial_mps)
                .unwrap_or(DVec3::ZERO),
            ContactPartyKind::KinematicTerrain => self
                .kinematic
                .iter()
                .find(|(id, _)| id.raw() == party.id_raw)
                .and_then(|(id, _)| self.kinematic_body_state(*id).ok())
                .map(|state| state.velocity_inertial_mps)
                .unwrap_or(DVec3::ZERO),
            ContactPartyKind::StaticTerrain => DVec3::ZERO,
        }
    }

    /// Reduce every touching pair to physical load evidence, capped at
    /// [`MAX_CONTACT_SUMMARIES`]. Contact points are deliberately omitted:
    /// the pair-local point frame is solver-internal, while normal,
    /// penetration, and approach speed are exact in the inertial frame.
    pub fn contact_summaries(&self) -> Vec<ContactSummary> {
        let mut summaries = Vec::new();
        for pair in self.narrow_phase.contact_pairs() {
            if summaries.len() >= MAX_CONTACT_SUMMARIES {
                break;
            }
            if !pair.has_any_active_contact() {
                continue;
            }
            let Some((manifold, contact)) = pair.find_deepest_contact() else {
                continue;
            };
            let (Some(a), Some(b)) = (self.party_of(pair.collider1), self.party_of(pair.collider2))
            else {
                continue;
            };
            let normal_inertial = self
                .frame
                .vector_to_inertial(from_rapier_vector(manifold.data.normal));
            let velocity_a = self.party_velocity_inertial_mps(a);
            let velocity_b = self.party_velocity_inertial_mps(b);
            summaries.push(ContactSummary {
                a,
                b,
                normal_inertial: normal_inertial.to_array(),
                penetration_m: (-contact.dist).max(0.0),
                approach_speed_mps: (velocity_a - velocity_b).dot(normal_inertial),
            });
        }
        summaries
    }

    /// Take the contact summaries recorded at the last step. A damage or
    /// telemetry consumer drains once per tick; undrained summaries are
    /// replaced, never accumulated.
    pub fn drain_contact_events(&mut self) -> Vec<ContactSummary> {
        std::mem::take(&mut self.last_contacts)
    }

    fn patch_debug(&self) -> Result<Vec<PatchDebug>, CollisionBackendError> {
        let mut patches = Vec::with_capacity(self.fixed.len() + self.kinematic.len());
        for (id, entry) in &self.fixed {
            let orientation = entry.orientation_local;
            patches.push(PatchDebug {
                party: ContactParty {
                    kind: ContactPartyKind::StaticTerrain,
                    id_raw: id.raw(),
                },
                center_inertial_m: self
                    .frame
                    .position_to_inertial(entry.center_local_m)
                    .to_array(),
                half_extents_m: entry.half_extents_m.to_array(),
                orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
            });
        }
        for (id, entry) in &self.kinematic {
            let state = self.kinematic_body_state(*id)?;
            let center_inertial_m = state.position_inertial_m
                + state.orientation_body_to_inertial * entry.shape_offset_m;
            let orientation = state.orientation_body_to_inertial;
            patches.push(PatchDebug {
                party: ContactParty {
                    kind: ContactPartyKind::KinematicTerrain,
                    id_raw: id.raw(),
                },
                center_inertial_m: center_inertial_m.to_array(),
                half_extents_m: entry.half_extents_m.to_array(),
                orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
            });
        }
        Ok(patches)
    }

    /// Query every wheel station against fixed or kinematic terrain and
    /// evaluate its tire/strut load and tangent-plane force. `wheel_spin_rad_s`
    /// carries the caller-owned scalar spin state in each station's authored
    /// axle direction. The returned wrench can be added to the ordinary
    /// external loads before [`Self::step`].
    ///
    /// The broad phase is the one produced by the preceding Rapier step. This
    /// is intentional: contact-active callers sync terrain and the vehicle,
    /// evaluate forces from that same scene, then advance exactly once.
    pub fn evaluate_articulated_wheel_contacts(
        &self,
        sprung_body_id: CollisionBodyId,
        chassis: &CompiledWheelChassis,
        bindings: &[ArticulatedWheelBinding],
    ) -> Result<Vec<ArticulatedWheelForce>, CollisionBackendError> {
        let sprung_entry = self
            .dynamic
            .get(&sprung_body_id)
            .ok_or(CollisionBackendError::UnknownBody(sprung_body_id))?;
        let sprung_body = self
            .bodies
            .get(sprung_entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLost(sprung_body_id))?;
        if bindings.len() != chassis.wheel_stations.len()
            || bindings
                .iter()
                .enumerate()
                .any(|(index, binding)| usize::from(binding.wheel_index) != index)
        {
            return Err(CollisionBackendError::InvalidWheelContact(
                "articulated wheel bindings must match compiled station order".into(),
            ));
        }

        let sprung_position_local = from_rapier_vector(sprung_body.position().translation);
        let sprung_orientation_local = from_rapier_rotation(sprung_body.position().rotation);
        let forward_body = chassis.spec.mount_orientation_body * DVec3::X;
        let query = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.bodies,
            &self.colliders,
            QueryFilter::default()
                .exclude_rigid_body(sprung_entry.rapier)
                .exclude_sensors(),
        );

        let max_toi = chassis.spec.strut.extended_length_m
            + chassis.spec.strut.stroke_m
            + chassis.spec.tire.radius_m;
        let mut forces = Vec::with_capacity(bindings.len());
        for (station, binding) in chassis.wheel_stations.iter().zip(bindings) {
            let wheel_entry = self
                .dynamic
                .get(&binding.wheel_body)
                .ok_or(CollisionBackendError::UnknownBody(binding.wheel_body))?;
            let wheel_body = self
                .bodies
                .get(wheel_entry.rapier)
                .ok_or(CollisionBackendError::BackendStateLost(binding.wheel_body))?;
            let wheel_position_local = from_rapier_vector(wheel_body.position().translation);
            let wheel_orientation_local = from_rapier_rotation(wheel_body.position().rotation);
            let down = (sprung_orientation_local * binding.slide_axis_sprung_local).normalize();
            let axle_axis_local =
                (wheel_orientation_local * binding.axle_axis_sprung_local).normalize();
            let nominal_center_local = sprung_position_local
                + sprung_orientation_local * binding.nominal_center_sprung_local_m;
            let relative_center_local = wheel_position_local - nominal_center_local;
            let strut_compression_m =
                (-relative_center_local.dot(down)).clamp(0.0, chassis.spec.strut.stroke_m);
            let nominal_center_rapier = to_rapier_vector(nominal_center_local);
            let wheel_center_rapier = to_rapier_vector(wheel_position_local);
            let sprung_velocity_at_hub = sprung_body.velocity_at_point(nominal_center_rapier);
            let wheel_velocity_at_hub = wheel_body.velocity_at_point(wheel_center_rapier);
            let relative_hub_velocity_local =
                from_rapier_vector(wheel_velocity_at_hub - sprung_velocity_at_hub);
            let strut_compression_rate_mps = -relative_hub_velocity_local.dot(down);
            let strut_load = chassis
                .spec
                .strut
                .axial_force(strut_compression_m, strut_compression_rate_mps)
                .map_err(|error| CollisionBackendError::InvalidWheelContact(error.to_string()))?;

            // The slider axis points from the chassis mount toward the wheel;
            // a compressed strut pushes the wheel down and the chassis up.
            let strut_force_local = down * strut_load.axial_force_n;
            let mount_local = nominal_center_local - down * chassis.spec.strut.extended_length_m;
            let wheel_strut_force_inertial = self.frame.vector_to_inertial(strut_force_local);
            let sprung_strut_force_inertial = -wheel_strut_force_inertial;
            let sprung_torque_local =
                (mount_local - sprung_position_local).cross(-strut_force_local);
            let mut wheel_wrench = ExternalWrench {
                force_inertial_n: wheel_strut_force_inertial,
                torque_inertial_nm: DVec3::ZERO,
            };
            let sprung_wrench = ExternalWrench {
                force_inertial_n: sprung_strut_force_inertial,
                torque_inertial_nm: self.frame.vector_to_inertial(sprung_torque_local),
            };

            let wheel_center_ray = Ray::new(
                to_rapier_vector(wheel_position_local),
                to_rapier_vector(down),
            );
            let nearest = query
                .intersect_ray(wheel_center_ray, max_toi, true)
                .filter_map(|(handle, collider, hit)| {
                    let normal = from_rapier_vector(hit.normal);
                    let alignment = -normal.dot(down);
                    if alignment <= 1.0e-6 {
                        return None;
                    }
                    if let Some(parent) = collider.parent()
                        && self
                            .bodies
                            .get(parent)
                            .is_some_and(|body| body.is_dynamic())
                    {
                        return None;
                    }
                    Some((handle, collider, hit, alignment.min(1.0)))
                })
                .min_by(|left, right| left.2.time_of_impact.total_cmp(&right.2.time_of_impact));

            let mut contact = None;
            if let Some((_, terrain_collider, hit, alignment)) = nearest {
                let terrain_normal_local = from_rapier_vector(hit.normal).normalize();
                let contact_point_local = wheel_position_local + down * hit.time_of_impact;
                let radial_penetration_m =
                    chassis.spec.tire.radius_m - hit.time_of_impact * alignment;
                if radial_penetration_m > 0.0 {
                    let point_rapier = to_rapier_vector(contact_point_local);
                    let wheel_velocity_local = wheel_body.velocity_at_point(point_rapier);
                    let terrain_velocity_local = terrain_collider
                        .parent()
                        .and_then(|parent| self.bodies.get(parent))
                        .map(|terrain_body| terrain_body.velocity_at_point(point_rapier))
                        .unwrap_or_else(|| Vector::new(0.0, 0.0, 0.0));
                    let relative_velocity_local =
                        from_rapier_vector(wheel_velocity_local - terrain_velocity_local);
                    let radial_compression_rate_mps =
                        -relative_velocity_local.dot(terrain_normal_local);
                    // The wheel body's pose already includes live strut travel;
                    // radial overlap is therefore tire deflection directly.
                    let tire_compression_m = radial_penetration_m;
                    let tire_compression_rate_mps = radial_compression_rate_mps;
                    let tire_load = chassis
                        .spec
                        .tire
                        .normal_load(tire_compression_m, tire_compression_rate_mps)
                        .map_err(|error| {
                            CollisionBackendError::InvalidWheelContact(error.to_string())
                        })?;

                    let axle_tangent = axle_axis_local
                        - terrain_normal_local * axle_axis_local.dot(terrain_normal_local);
                    let forward_hint_local = sprung_orientation_local * forward_body;
                    let (mut forward_local, mut lateral_local) = if axle_tangent.length_squared()
                        > 1.0e-12
                    {
                        let lateral = axle_tangent.normalize();
                        (lateral.cross(terrain_normal_local).normalize(), lateral)
                    } else {
                        let projected_forward = forward_hint_local
                            - terrain_normal_local * forward_hint_local.dot(terrain_normal_local);
                        let forward = if projected_forward.length_squared() > 1.0e-12 {
                            projected_forward.normalize()
                        } else {
                            let seed = if terrain_normal_local.x.abs() < 0.9 {
                                DVec3::X
                            } else {
                                DVec3::Y
                            };
                            (seed - terrain_normal_local * seed.dot(terrain_normal_local))
                                .normalize()
                        };
                        (forward, terrain_normal_local.cross(forward).normalize())
                    };
                    if forward_local.dot(forward_hint_local) < 0.0 {
                        forward_local = -forward_local;
                        lateral_local = -lateral_local;
                    }
                    let contact_friction = chassis
                        .spec
                        .tire
                        .contact_friction(terrain_collider.friction())
                        .map_err(|error| {
                            CollisionBackendError::InvalidWheelContact(error.to_string())
                        })?;
                    let tangent_force = chassis
                        .spec
                        .tire
                        .tangential_force(
                            relative_velocity_local.dot(forward_local),
                            relative_velocity_local.dot(lateral_local),
                            tire_load.normal_load_n,
                            contact_friction,
                        )
                        .map_err(|error| {
                            CollisionBackendError::InvalidWheelContact(error.to_string())
                        })?;
                    let contact_force_local = terrain_normal_local * tire_load.normal_load_n
                        + forward_local * tangent_force.longitudinal_force_n
                        + lateral_local * tangent_force.lateral_force_n;
                    wheel_wrench.force_inertial_n +=
                        self.frame.vector_to_inertial(contact_force_local);
                    wheel_wrench.torque_inertial_nm += self.frame.vector_to_inertial(
                        (contact_point_local - wheel_position_local).cross(contact_force_local),
                    );
                    sprung_wrench.validate()?;
                    wheel_wrench.validate()?;
                    let angular_velocity_sprung_local = from_rapier_vector(sprung_body.angvel());
                    let angular_velocity_wheel_local = from_rapier_vector(wheel_body.angvel());
                    let spin_rate_rad_s = (angular_velocity_wheel_local
                        - angular_velocity_sprung_local)
                        .dot(axle_axis_local);
                    contact = Some(WheelContactSample {
                        wheel_index: station.index,
                        contact_point_inertial_m: self
                            .frame
                            .position_to_inertial(contact_point_local),
                        terrain_normal_inertial: self
                            .frame
                            .vector_to_inertial(terrain_normal_local),
                        forward_axis_inertial: self.frame.vector_to_inertial(forward_local),
                        lateral_axis_inertial: self.frame.vector_to_inertial(lateral_local),
                        relative_contact_velocity_inertial_mps: self
                            .frame
                            .vector_to_inertial(relative_velocity_local),
                        radial_penetration_m,
                        strut_compression_m,
                        tire_compression_m: tire_load.compression_m,
                        compression_rate_mps: radial_compression_rate_mps,
                        normal_load_n: tire_load.normal_load_n,
                        longitudinal_force_n: tangent_force.longitudinal_force_n,
                        lateral_force_n: tangent_force.lateral_force_n,
                        contact_friction,
                        saturated: strut_load.saturated
                            || tire_load.saturated
                            || tangent_force.saturated,
                    });
                    forces.push(ArticulatedWheelForce {
                        wheel_body: binding.wheel_body,
                        wheel_index: station.index,
                        spin_rate_rad_s,
                        contact,
                        wheel_wrench,
                        sprung_wrench,
                    });
                    continue;
                }
            }

            let angular_velocity_sprung_local = from_rapier_vector(sprung_body.angvel());
            let angular_velocity_wheel_local = from_rapier_vector(wheel_body.angvel());
            let spin_rate_rad_s =
                (angular_velocity_wheel_local - angular_velocity_sprung_local).dot(axle_axis_local);
            wheel_wrench.validate()?;
            sprung_wrench.validate()?;
            forces.push(ArticulatedWheelForce {
                wheel_body: binding.wheel_body,
                wheel_index: station.index,
                spin_rate_rad_s,
                contact,
                wheel_wrench,
                sprung_wrench,
            });
        }
        Ok(forces)
    }

    /// Query every wheel station against fixed or kinematic terrain and
    /// evaluate its tire/strut load and tangent-plane force. `wheel_spin_rad_s`
    pub fn evaluate_wheel_contacts(
        &self,
        body_id: CollisionBodyId,
        chassis: &CompiledWheelChassis,
        wheel_spin_rad_s: &[f64],
    ) -> Result<WheelContactResult, CollisionBackendError> {
        let entry = self
            .dynamic
            .get(&body_id)
            .ok_or(CollisionBackendError::UnknownBody(body_id))?;
        let body = self
            .bodies
            .get(entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLost(body_id))?;
        if chassis.wheel_stations.len() != usize::from(chassis.spec.wheel_count)
            || wheel_spin_rad_s.len() != chassis.wheel_stations.len()
            || wheel_spin_rad_s.iter().any(|speed| !speed.is_finite())
        {
            return Err(CollisionBackendError::InvalidWheelContact(
                "compiled wheel stations and finite spin rates must match wheel_count".into(),
            ));
        }

        let root_pose = body.position();
        let root_position_local = from_rapier_vector(root_pose.translation);
        let root_orientation_local = from_rapier_rotation(root_pose.rotation);
        let down_body = chassis.spec.mount_orientation_body * DVec3::NEG_Z;
        let down_local = (root_orientation_local * down_body).normalize();
        let forward_body = chassis.spec.mount_orientation_body * DVec3::X;
        let query = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.bodies,
            &self.colliders,
            QueryFilter::default().exclude_rigid_body(entry.rapier),
        );

        let mut contacts = Vec::with_capacity(chassis.wheel_stations.len());
        let mut wrench = ExternalWrench::ZERO;
        let max_toi = chassis.spec.strut.extended_length_m
            + chassis.spec.strut.stroke_m
            + chassis.spec.tire.radius_m;
        for (station, spin_rate_rad_s) in chassis
            .wheel_stations
            .iter()
            .zip(wheel_spin_rad_s.iter().copied())
        {
            let wheel_center_local =
                root_position_local + root_orientation_local * station.position_body_m;
            let ray = Ray::new(
                to_rapier_vector(wheel_center_local),
                to_rapier_vector(down_local),
            );
            let nearest = query
                .intersect_ray(ray, max_toi, true)
                .filter_map(|(handle, collider, hit)| {
                    let normal = from_rapier_vector(hit.normal);
                    let alignment = -normal.dot(down_local);
                    if alignment <= 1.0e-6 {
                        return None;
                    }
                    if let Some(parent) = collider.parent() {
                        let terrain_body = self.bodies.get(parent)?;
                        // Tire forces are one-sided against terrain in this
                        // slice. Dynamic-body interaction remains Rapier's
                        // solid-contact responsibility.
                        if terrain_body.is_dynamic() {
                            return None;
                        }
                    }
                    Some((handle, collider, hit, alignment.min(1.0)))
                })
                .min_by(|left, right| left.2.time_of_impact.total_cmp(&right.2.time_of_impact));
            let Some((_, terrain_collider, hit, alignment)) = nearest else {
                continue;
            };

            let terrain_normal_local = from_rapier_vector(hit.normal).normalize();
            let contact_point_local = wheel_center_local + down_local * hit.time_of_impact;
            let radial_penetration_m = chassis.spec.tire.radius_m - hit.time_of_impact * alignment;
            if radial_penetration_m <= 0.0 {
                continue;
            }

            let wheel_velocity_local =
                body.velocity_at_point(to_rapier_vector(contact_point_local));
            let terrain_velocity_local = terrain_collider
                .parent()
                .and_then(|parent| self.bodies.get(parent))
                .map(|terrain_body| {
                    terrain_body.velocity_at_point(to_rapier_vector(contact_point_local))
                })
                .unwrap_or_else(|| Vector::new(0.0, 0.0, 0.0));
            let axle_axis_local = (root_orientation_local * station.axle_axis_body).normalize();
            let wheel_spin_velocity_local = axle_axis_local
                .cross(-terrain_normal_local * chassis.spec.tire.radius_m)
                * spin_rate_rad_s;
            let relative_velocity_local = from_rapier_vector(wheel_velocity_local)
                - from_rapier_vector(terrain_velocity_local)
                + wheel_spin_velocity_local;
            let compression_rate_mps = -relative_velocity_local.dot(terrain_normal_local);
            let load = chassis
                .spec
                .contact_load(radial_penetration_m, compression_rate_mps, alignment)
                .map_err(|error| CollisionBackendError::InvalidWheelContact(error.to_string()))?;

            let axle_tangent =
                axle_axis_local - terrain_normal_local * axle_axis_local.dot(terrain_normal_local);
            if axle_tangent.length_squared() <= 1.0e-12 {
                continue;
            }
            let mut lateral_local = axle_tangent.normalize();
            let mut forward_local = lateral_local.cross(terrain_normal_local).normalize();
            let forward_hint_local = root_orientation_local * forward_body;
            if forward_local.dot(forward_hint_local) < 0.0 {
                forward_local = -forward_local;
                lateral_local = -lateral_local;
            }
            let contact_friction = chassis
                .spec
                .tire
                .contact_friction(terrain_collider.friction())
                .map_err(|error| CollisionBackendError::InvalidWheelContact(error.to_string()))?;
            let tangent_force = chassis
                .spec
                .tire
                .tangential_force(
                    relative_velocity_local.dot(forward_local),
                    relative_velocity_local.dot(lateral_local),
                    load.normal_load_n,
                    contact_friction,
                )
                .map_err(|error| CollisionBackendError::InvalidWheelContact(error.to_string()))?;
            let force_local = terrain_normal_local * load.normal_load_n
                + forward_local * tangent_force.longitudinal_force_n
                + lateral_local * tangent_force.lateral_force_n;
            let torque_local = (contact_point_local - root_position_local).cross(force_local);
            let contact = WheelContactSample {
                wheel_index: station.index,
                contact_point_inertial_m: self.frame.position_to_inertial(contact_point_local),
                terrain_normal_inertial: self.frame.vector_to_inertial(terrain_normal_local),
                forward_axis_inertial: self.frame.vector_to_inertial(forward_local),
                lateral_axis_inertial: self.frame.vector_to_inertial(lateral_local),
                relative_contact_velocity_inertial_mps: self
                    .frame
                    .vector_to_inertial(relative_velocity_local),
                radial_penetration_m,
                strut_compression_m: load.strut_compression_m,
                tire_compression_m: load.tire_compression_m,
                compression_rate_mps,
                normal_load_n: load.normal_load_n,
                longitudinal_force_n: tangent_force.longitudinal_force_n,
                lateral_force_n: tangent_force.lateral_force_n,
                contact_friction,
                saturated: load.saturated || tangent_force.saturated,
            };
            wrench.force_inertial_n += self.frame.vector_to_inertial(force_local);
            wrench.torque_inertial_nm += self.frame.vector_to_inertial(torque_local);
            contacts.push(contact);
        }

        wrench.validate()?;
        Ok(WheelContactResult { contacts, wrench })
    }

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
mod tests {
    use super::*;
    use thessa_sim_core::{
        CollisionPart, CollisionShape, TireConstruction, WheelBrakeSpec, WheelChassisSpec,
        WheelLayout, WheelStrutSpec, WheelTireSpec,
    };

    fn sphere_geometry(radius_m: f64) -> CollisionGeometry {
        CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap()
    }

    fn wheel_chassis() -> CompiledWheelChassis {
        WheelChassisSpec {
            name: "query-test-wheel".into(),
            mount_position_body_m: DVec3::new(0.0, 0.0, 0.7),
            mount_orientation_body: DQuat::IDENTITY,
            length_m: 1.0,
            layout: WheelLayout::Inline,
            wheel_count: 1,
            structural_mass_kg: 10.0,
            structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(0.2)),
            tire: WheelTireSpec {
                construction: TireConstruction::Pneumatic {
                    inflation_pressure_pa: 220_000.0,
                    reference_temperature_k: 293.15,
                },
                radius_m: 0.32,
                width_m: 0.2,
                mass_kg: 2.0,
                spin_inertia_kg_m2: 0.08,
                radial_stiffness_n_m: 200_000.0,
                radial_damping_n_s_m: 5_000.0,
                longitudinal_slip_stiffness_n_per_mps: 12_000.0,
                lateral_slip_stiffness_n_per_mps: 9_000.0,
                maximum_deflection_m: 0.08,
                maximum_load_n: 8_000.0,
                surface_friction: 0.9,
            },
            strut: WheelStrutSpec {
                extended_length_m: 0.4,
                stroke_m: 0.15,
                spring_rate_n_m: 30_000.0,
                damping_n_s_m: 2_000.0,
                preload_n: 0.0,
                minimum_force_n: 0.0,
                maximum_force_n: 10_000.0,
                mass_per_wheel_kg: 0.5,
            },
            brake: WheelBrakeSpec {
                maximum_torque_nm: 200.0,
                response_time_s: 0.1,
                mass_per_wheel_kg: 0.3,
            },
            drive: None,
        }
        .compile()
        .unwrap()
    }

    #[test]
    fn collision_frame_round_trips_authoritative_state() {
        let frame = CollisionFrame::new(
            DVec3::new(1.0e9, -2.0e9, 3.0e9),
            DVec3::new(1200.0, -30.0, 8.0),
            DQuat::from_rotation_z(0.7),
        )
        .unwrap();
        let state = RigidBodyState::new(
            frame.origin_inertial_m + DVec3::new(10.0, 20.0, -5.0),
            DVec3::new(1210.0, -25.0, 11.0),
            DQuat::from_rotation_y(0.3),
            DVec3::new(0.1, -0.2, 0.3),
        )
        .unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
        let mut world = CollisionWorld::new(frame).unwrap();
        let id = world
            .insert_dynamic_body(
                state,
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let round_trip = world.body_state(id).unwrap();
        assert!((round_trip.position_inertial_m - state.position_inertial_m).length() < 1.0e-6);
        assert!((round_trip.velocity_inertial_mps - state.velocity_inertial_mps).length() < 1.0e-9);
        assert!(
            round_trip
                .orientation_body_to_inertial
                .dot(state.orientation_body_to_inertial)
                .abs()
                > 1.0 - 1.0e-12
        );
        assert!(
            (round_trip.angular_velocity_body_rps - state.angular_velocity_body_rps).length()
                < 1.0e-9
        );
    }

    #[test]
    fn dynamic_body_inputs_reject_invalid_public_state_and_inertia() {
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
        let geometry = sphere_geometry(0.5);
        let state = RigidBodyState::stationary(DVec3::ZERO);

        let invalid_orientation = RigidBodyState {
            orientation_body_to_inertial: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            ..state
        };
        assert!(matches!(
            world.insert_dynamic_body(
                invalid_orientation,
                properties,
                &geometry,
                DynamicBodyConfig::default()
            ),
            Err(CollisionBackendError::InvalidBodyState(_))
        ));

        let indefinite_inertia = RigidBodyProperties {
            mass_kg: 5.0,
            inertia_body_kg_m2: DMat3::from_diagonal(DVec3::new(1.0, -1.0, -1.0)),
        };
        assert!(matches!(
            world.insert_dynamic_body(
                state,
                indefinite_inertia,
                &geometry,
                DynamicBodyConfig::default()
            ),
            Err(CollisionBackendError::InvalidMassProperties)
        ));
        assert_eq!(world.dynamic_body_count(), 0);

        let id = world
            .insert_dynamic_body(state, properties, &geometry, DynamicBodyConfig::default())
            .unwrap();
        assert!(matches!(
            world.resync_dynamic_body(
                id,
                invalid_orientation,
                properties,
                DynamicBodyConfig::default()
            ),
            Err(CollisionBackendError::InvalidBodyState(_))
        ));
        assert_eq!(world.body_state(id).unwrap(), state);
    }

    #[test]
    fn body_states_unrepresentable_in_the_collision_frame_are_rejected() {
        let origin = DVec3::splat(-f64::MAX);
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(origin, DVec3::ZERO)).unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
        let geometry = sphere_geometry(0.5);
        let initial = RigidBodyState::stationary(origin);
        let id = world
            .insert_dynamic_body(initial, properties, &geometry, DynamicBodyConfig::default())
            .unwrap();
        let unrepresentable = RigidBodyState::stationary(DVec3::splat(f64::MAX));

        assert!(matches!(
            world.resync_dynamic_body(
                id,
                unrepresentable,
                properties,
                DynamicBodyConfig::default()
            ),
            Err(CollisionBackendError::InvalidBodyState(_))
        ));
        assert_eq!(world.dynamic_body_count(), 1);
        assert_eq!(world.body_state(id).unwrap(), initial);
    }

    #[test]
    fn rapier_resolves_gravity_driven_ground_contact() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.5, 10.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let mass_kg = 5.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..360 {
            world.step(1.0 / 120.0, [(id, gravity)]).unwrap();
        }
        let solved = world.body_state(id).unwrap();
        assert!((solved.position_inertial_m.y - 0.5).abs() < 0.05);
        assert!(solved.velocity_inertial_mps.length() < 0.2);
    }

    #[test]
    fn wheel_query_resolves_tire_strut_load_and_returns_single_contact_wrench() {
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(10.0, 10.0, 0.5),
                CollisionMaterial::new(0.5, 0.0).unwrap(),
            )
            .unwrap();
        let properties =
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
        let state = RigidBodyState::new(
            DVec3::ZERO,
            DVec3::new(0.1, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let id = world
            .insert_dynamic_body(
                state,
                properties,
                &CollisionGeometry::new(vec![
                    CollisionPart::new(
                        DVec3::Z,
                        DQuat::IDENTITY,
                        CollisionShape::Sphere { radius_m: 0.1 },
                        CollisionMaterial::default(),
                    )
                    .unwrap(),
                ])
                .unwrap(),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        world
            .step(1.0 / 120.0, [(id, ExternalWrench::ZERO)])
            .unwrap();

        let chassis = wheel_chassis();
        let result = world.evaluate_wheel_contacts(id, &chassis, &[0.0]).unwrap();
        assert_eq!(result.contacts.len(), 1);
        let contact = result.contacts[0];
        assert!((contact.radial_penetration_m - 0.02).abs() < 1.0e-8);
        assert!(contact.normal_load_n > 0.0);
        assert!((contact.contact_friction - 0.7).abs() < 1.0e-12);
        assert!(contact.longitudinal_force_n < 0.0);
        assert!(
            contact.longitudinal_force_n.hypot(contact.lateral_force_n)
                <= contact.contact_friction * contact.normal_load_n + 1.0e-9
        );
        assert!(result.wrench.force_inertial_n.z > 0.0);
        assert!(result.wrench.force_inertial_n.x < 0.0);
        assert!(result.wrench.torque_inertial_nm.is_finite());

        let rolling = world
            .evaluate_wheel_contacts(id, &chassis, &[0.1 / chassis.spec.tire.radius_m])
            .unwrap();
        assert!(
            rolling.contacts[0]
                .relative_contact_velocity_inertial_mps
                .x
                .abs()
                < 1.0e-10
        );
        assert!(rolling.contacts[0].longitudinal_force_n.abs() < 1.0e-8);

        assert!(world.evaluate_wheel_contacts(id, &chassis, &[]).is_err());
    }

    #[test]
    fn wheel_query_uses_kinematic_terrain_velocity_for_tire_slip() {
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
        let terrain = world
            .insert_kinematic_cuboid(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(10.0, 10.0, 0.5),
                CollisionMaterial::default(),
            )
            .unwrap();
        let properties =
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
        let geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::Z,
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.1 },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let body = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::ZERO),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        world
            .set_next_kinematic_pose(terrain, DVec3::new(0.1, 0.0, -0.5), DQuat::IDENTITY)
            .unwrap();
        world.step(0.1, [(body, ExternalWrench::ZERO)]).unwrap();

        let chassis = wheel_chassis();
        let contact = world
            .evaluate_wheel_contacts(body, &chassis, &[0.0])
            .unwrap()
            .contacts[0];
        assert!((contact.relative_contact_velocity_inertial_mps.x + 1.0).abs() < 1.0e-10);
        assert!(contact.longitudinal_force_n > 0.0);
    }

    #[test]
    fn articulated_wheel_keeps_tire_deflection_separate_from_strut_travel() {
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(10.0, 10.0, 0.5),
                CollisionMaterial::default(),
            )
            .unwrap();

        let mut chassis = wheel_chassis();
        chassis.spec.mount_position_body_m = DVec3::ZERO;
        chassis = chassis.spec.clone().compile().unwrap();
        let station = chassis.wheel_stations[0];
        let sprung_state = RigidBodyState::stationary(DVec3::new(0.0, 0.0, 0.65));
        let sprung_geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::new(0.0, 0.0, 5.0),
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.1 },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let sprung = world
            .insert_dynamic_body(
                sprung_state,
                RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap(),
                &sprung_geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let wheel_mass = chassis.wheel_body_mass_properties(0)[0];
        let wheel_geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Sphere {
                    radius_m: chassis.spec.tire.radius_m,
                },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let wheel = world
            .insert_dynamic_sensor_body(
                RigidBodyState::stationary(
                    sprung_state.position_inertial_m + station.position_body_m + DVec3::Z * 0.05,
                ),
                RigidBodyProperties::new(wheel_mass.mass_kg, wheel_mass.inertia_body_kg_m2)
                    .unwrap(),
                &wheel_geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        world
            .attach_suspension_wheel_joint(
                sprung,
                wheel,
                station.position_body_m,
                DVec3::ZERO,
                DVec3::NEG_Z,
                station.axle_axis_body,
                DVec3::NEG_Z,
                station.axle_axis_body,
                [-chassis.spec.strut.stroke_m, 0.0],
            )
            .unwrap();
        world.step(1.0 / 120.0, []).unwrap();

        let binding = ArticulatedWheelBinding {
            wheel_body: wheel,
            wheel_index: station.index,
            nominal_center_sprung_local_m: station.position_body_m,
            slide_axis_sprung_local: DVec3::NEG_Z,
            axle_axis_sprung_local: station.axle_axis_body,
        };
        let result = world
            .evaluate_articulated_wheel_contacts(sprung, &chassis, &[binding])
            .unwrap();
        let contact = result[0].contact.expect("wheel/terrain contact");
        assert!((contact.strut_compression_m - 0.05).abs() < 1.0e-9);
        assert!((contact.radial_penetration_m - 0.02).abs() < 1.0e-9);
        assert!((contact.tire_compression_m - 0.02).abs() < 1.0e-9);
        assert!(result[0].wheel_wrench.force_inertial_n.z > 0.0);
        assert!(result[0].sprung_wrench.force_inertial_n.z > 0.0);
        assert!(
            (result[0].wheel_wrench.force_inertial_n.z
                + result[0].sprung_wrench.force_inertial_n.z
                - contact.normal_load_n)
                .abs()
                < 1.0e-9,
            "strut reaction must cancel internally, leaving only tire contact load"
        );
    }

    #[test]
    fn one_wheel_contact_settles_under_gravity_without_solid_wheel_impulses() {
        let mut world =
            CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, 0.0, -0.5),
                DQuat::IDENTITY,
                DVec3::new(10.0, 10.0, 0.5),
                CollisionMaterial::default(),
            )
            .unwrap();
        let mass_kg = 100.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
        let geometry = CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::Z,
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.1 },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::ZERO),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let chassis = wheel_chassis();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, 0.0, -9.81 * mass_kg),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..1_200 {
            world
                .step_with_wheel_contacts(
                    1.0 / 240.0,
                    [(id, gravity)],
                    [(id, &chassis, &[0.0][..])],
                )
                .unwrap();
        }
        let solved = world.body_state(id).unwrap();
        let contact = world
            .evaluate_wheel_contacts(id, &chassis, &[0.0])
            .unwrap()
            .contacts[0];
        assert!((contact.normal_load_n - mass_kg * 9.81).abs() < 0.005 * mass_kg * 9.81);
        assert!(solved.velocity_inertial_mps.length() < 0.1);
        assert!(solved.position_inertial_m.z < 0.0);
        assert!(!contact.saturated);
    }

    #[test]
    fn translating_frame_advances_inertial_origin() {
        let frame = CollisionFrame::inertial_at(
            DVec3::new(1.0e9, -2.0e9, 3.0e9),
            DVec3::new(125.0, -7.0, 3.0),
        );
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
        let initial = RigidBodyState::new(
            frame.origin_inertial_m + DVec3::new(2.0, 3.0, 4.0),
            frame.origin_velocity_inertial_mps,
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let mut world = CollisionWorld::new(frame).unwrap();
        let id = world
            .insert_dynamic_body(
                initial,
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();

        let step_s = 2.0;
        world.step(step_s, [(id, ExternalWrench::ZERO)]).unwrap();
        let solved = world.body_state(id).unwrap();
        let expected_position =
            initial.position_inertial_m + frame.origin_velocity_inertial_mps * step_s;

        assert!((solved.position_inertial_m - expected_position).length() < 1.0e-6);
        assert!((solved.velocity_inertial_mps - initial.velocity_inertial_mps).length() < 1.0e-12);
        assert!(
            (world.frame().origin_inertial_m
                - (frame.origin_inertial_m + frame.origin_velocity_inertial_mps * step_s))
                .length()
                < 1.0e-9
        );
    }

    #[test]
    fn free_rapier_motion_matches_symplectic_euler_envelope() {
        // Vacuum force/torque fixture before contacts are enabled: a constant
        // inertial force with zero moment and zero spin must track the
        // authoritative symplectic-Euler translation (v then x) inside a
        // bounded envelope. This pins the wrench conversion and readback, not
        // solver identity: Rapier integrates the same load with its own
        // scheme, so the bound is physical, not bitwise.
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let mass_kg = 10.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(4.0))).unwrap();
        let initial = RigidBodyState::new(
            DVec3::new(0.0, 100.0, 0.0),
            DVec3::new(12.0, 3.0, -1.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let id = world
            .insert_dynamic_body(
                initial,
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let acceleration = DVec3::new(1.5, -9.81, 0.25);
        let wrench = ExternalWrench {
            force_inertial_n: acceleration * mass_kg,
            torque_inertial_nm: DVec3::ZERO,
        };
        let dt = 1.0 / 120.0;
        let steps = 120;
        for _ in 0..steps {
            world.step(dt, [(id, wrench)]).unwrap();
        }
        let solved = world.body_state(id).unwrap();
        // Reference: closed-form constant-acceleration motion plus one
        // symplectic-Euler step offset (v-then-x ordering advances position
        // with the end-of-step velocity).
        let time_s = dt * steps as f64;
        let expected_velocity = initial.velocity_inertial_mps + acceleration * time_s;
        let expected_position = initial.position_inertial_m
            + initial.velocity_inertial_mps * time_s
            + 0.5 * acceleration * time_s * time_s;
        let velocity_error = (solved.velocity_inertial_mps - expected_velocity).length();
        let position_error = (solved.position_inertial_m - expected_position).length();
        assert!(
            velocity_error < 0.05,
            "free-flight velocity drift {velocity_error}"
        );
        assert!(
            position_error < 0.10,
            "free-flight position drift {position_error}"
        );
        assert!(
            (solved.angular_velocity_body_rps - DVec3::ZERO).length() < 1.0e-6,
            "torque-free spin must not appear"
        );
    }

    #[test]
    fn fast_body_does_not_tunnel_through_floor() {
        // Fast-impact regression: a sphere at 60 m/s toward a thin floor must
        // be caught by CCD instead of tunnelling. One 120 Hz step moves
        // 0.5 m; the floor top is at y=0 and the sphere starts 3 m above it.
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, -0.25, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.25, 10.0),
                CollisionMaterial::new(0.7, 0.0).unwrap(),
            )
            .unwrap();
        let mass_kg = 5.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::new(
                    DVec3::new(0.0, 3.0, 0.0),
                    DVec3::new(0.0, -60.0, 0.0),
                    DQuat::IDENTITY,
                    DVec3::ZERO,
                )
                .unwrap(),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        for _ in 0..240 {
            world
                .step(1.0 / 120.0, [(id, ExternalWrench::ZERO)])
                .unwrap();
        }
        let solved = world.body_state(id).unwrap();
        assert!(
            solved.position_inertial_m.y > -0.5,
            "fast body tunnelled through the floor: y={}",
            solved.position_inertial_m.y
        );
        assert!(
            (solved.position_inertial_m.y - 0.5).abs() < 0.6,
            "fast body did not settle near the floor: y={}",
            solved.position_inertial_m.y
        );
    }

    #[test]
    fn kinematic_terrain_carries_a_landed_body() {
        // Kinematic seam: an elevator platform rising at 1 m/s must carry a
        // resting sphere with it. The platform pose is prescribed per tick,
        // exactly how the ephemeris/body-rotation model will drive planetary
        // terrain in production.
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let platform = world
            .insert_kinematic_cuboid(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(5.0, 0.5, 5.0),
                CollisionMaterial::new(0.9, 0.0).unwrap(),
            )
            .unwrap();
        let mass_kg = 5.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 0.6, 0.0)),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig {
                    full_ccd: true,
                    can_sleep: false,
                },
            )
            .unwrap();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        let dt = 1.0 / 120.0;
        // Settle onto the platform first.
        for _ in 0..120 {
            world.step(dt, [(id, gravity)]).unwrap();
        }
        // Rise 1 m over the next second.
        for step in 1..=120 {
            let lift = step as f64 * dt * 1.0;
            world
                .set_next_kinematic_pose(
                    platform,
                    DVec3::new(0.0, -0.5 + lift, 0.0),
                    DQuat::IDENTITY,
                )
                .unwrap();
            world.step(dt, [(id, gravity)]).unwrap();
        }
        let solved = world.body_state(id).unwrap();
        assert!(
            (solved.position_inertial_m.y - 1.5).abs() < 0.15,
            "landed body did not ride the kinematic platform: y={}",
            solved.position_inertial_m.y
        );
    }

    #[test]
    fn debug_snapshot_reports_settled_contact() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.5, 10.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let mass_kg = 5.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..360 {
            world.step(1.0 / 120.0, [(id, gravity)]).unwrap();
        }
        let snapshot = world.debug_snapshot().unwrap();
        assert_eq!(snapshot.dynamic_bodies.len(), 1);
        assert_eq!(snapshot.fixed_collider_count, 1);
        assert!(
            snapshot.touching_contact_pairs >= 1,
            "settled body must report a touching pair: {snapshot:?}"
        );
        assert!(
            snapshot.touching_contact_pairs <= snapshot.active_contact_pairs,
            "touching pairs must be a subset of active pairs: {snapshot:?}"
        );
        let json = serde_json::to_string(&snapshot).expect("snapshot must serialize");
        assert!(json.contains("touching_contact_pairs"));
    }

    #[test]
    fn orbital_scale_velocity_survives_a_step() {
        // Rapier's default 400 m/s velocity clamp would silently rewrite a
        // co-moving orbital velocity. The backend disables it: a fast body
        // with no contacts and no wrench must keep its velocity through a
        // step, with position advancing by the full displacement.
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
        let initial = RigidBodyState::new(
            DVec3::new(1.0e9, 2.0e9, 3.0e9),
            DVec3::new(1200.0, 51_000.0, -300.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let id = world
            .insert_dynamic_body(
                initial,
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let dt = 1.0 / 120.0;
        world.step(dt, [(id, ExternalWrench::ZERO)]).unwrap();
        let solved = world.body_state(id).unwrap();
        assert!(
            (solved.velocity_inertial_mps - initial.velocity_inertial_mps).length() < 1.0e-6,
            "orbital velocity was clamped: {:?}",
            solved.velocity_inertial_mps
        );
        let expected_position = initial.position_inertial_m + initial.velocity_inertial_mps * dt;
        assert!(
            (solved.position_inertial_m - expected_position).length() < 1.0e-3,
            "orbital displacement is wrong: {:?}",
            solved.position_inertial_m
        );
    }

    #[test]
    fn rejected_trimesh_leaves_no_untracked_body() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        // Out-of-range triangle index fails the mesh build.
        let vertices = vec![DVec3::ZERO, DVec3::X, DVec3::Y];
        let result =
            world.insert_kinematic_trimesh(vertices, vec![[0, 1, 7]], CollisionMaterial::default());
        assert!(result.is_err(), "bad trimesh indices must be rejected");
        assert_eq!(world.kinematic_body_count(), 0);
        assert!(world.debug_snapshot().unwrap().kinematic_bodies.is_empty());
    }

    #[test]
    fn removing_bodies_and_patches_clears_the_scene() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let patch = world
            .insert_static_cuboid(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.5, 10.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let platform = world
            .insert_kinematic_cuboid(
                DVec3::new(8.0, 0.0, 0.0),
                DQuat::IDENTITY,
                DVec3::new(1.0, 0.5, 1.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        assert_eq!(world.dynamic_body_count(), 1);
        assert_eq!(world.fixed_collider_count(), 1);
        assert_eq!(world.kinematic_body_count(), 1);
        world.remove_dynamic_body(id).unwrap();
        world.remove_static_collider(patch).unwrap();
        world.remove_kinematic_body(platform).unwrap();
        assert_eq!(world.dynamic_body_count(), 0);
        assert_eq!(world.fixed_collider_count(), 0);
        assert_eq!(world.kinematic_body_count(), 0);
        assert!(world.debug_snapshot().unwrap().dynamic_bodies.is_empty());
    }
}

#[cfg(test)]
mod joint_and_replay_tests {
    use super::*;
    use thessa_sim_core::{
        CollisionPart, CollisionShape, DockingKinematics, DockingPortSpec, DockingPortState,
        DockingSession,
    };

    fn sphere_geometry(radius_m: f64) -> CollisionGeometry {
        CollisionGeometry::new(vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m },
                CollisionMaterial::default(),
            )
            .unwrap(),
        ])
        .unwrap()
    }

    fn two_spheres() -> (CollisionWorld, CollisionBodyId, CollisionBodyId) {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let properties =
            RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let geometry = sphere_geometry(0.5);
        let a = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(-2.0, 0.0, 0.0)),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let b = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(2.0, 0.0, 0.0)),
                properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        (world, a, b)
    }

    #[test]
    fn prismatic_joint_keeps_its_anchor_and_enforces_strut_stroke_limits() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let root_properties =
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(20.0))).unwrap();
        let wheel_properties =
            RigidBodyProperties::new(2.0, DMat3::from_diagonal(DVec3::splat(0.2))).unwrap();
        let geometry = sphere_geometry(0.1);
        let root = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::ZERO),
                root_properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let wheel = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 0.0, -0.5)),
                wheel_properties,
                &geometry,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let joint = world
            .attach_prismatic_joint(
                root,
                wheel,
                DVec3::new(0.0, 0.0, -0.5),
                DVec3::ZERO,
                DVec3::NEG_Z,
                DVec3::NEG_Z,
                [0.0, 0.2],
            )
            .unwrap();
        let push_down = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, 0.0, -100.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..600 {
            world
                .step(
                    1.0 / 240.0,
                    [(root, ExternalWrench::ZERO), (wheel, push_down)],
                )
                .unwrap();
        }
        let root_state = world.body_state(root).unwrap();
        let wheel_state = world.body_state(wheel).unwrap();
        let relative = wheel_state.position_inertial_m - root_state.position_inertial_m;
        let extension_m = (relative - DVec3::new(0.0, 0.0, -0.5)).dot(DVec3::NEG_Z);
        assert!((extension_m - 0.2).abs() < 0.02);
        assert!(world.joint_count() == 1);
        world.remove_joint(joint).unwrap();
        assert_eq!(world.joint_count(), 0);
    }

    fn d1_craft_geometry(port_x_m: f64) -> CollisionGeometry {
        let material = CollisionMaterial {
            friction: 0.45,
            restitution: 0.0,
        };
        let mut parts = vec![
            CollisionPart::new(
                DVec3::ZERO,
                DQuat::IDENTITY,
                CollisionShape::Cuboid {
                    half_extents_m: DVec3::new(0.35, 0.45, 0.45),
                },
                material,
            )
            .unwrap(),
        ];
        for index in 0..8 {
            let angle = f64::from(index) * std::f64::consts::TAU / 8.0;
            parts.push(
                CollisionPart::new(
                    DVec3::new(
                        port_x_m - port_x_m.signum() * 0.01,
                        0.55 * angle.cos(),
                        0.55 * angle.sin(),
                    ),
                    DQuat::from_rotation_x(angle),
                    CollisionShape::Cuboid {
                        half_extents_m: DVec3::new(0.01, 0.12, 0.12),
                    },
                    material,
                )
                .unwrap(),
            );
        }
        CollisionGeometry::new(parts).unwrap()
    }

    fn port_frame_world(state: RigidBodyState, local_position_m: DVec3) -> DVec3 {
        state.position_inertial_m + state.orientation_body_to_inertial * local_position_m
    }

    #[test]
    fn docked_bodies_move_as_one_and_undock_cleanly() {
        let (mut world, a, b) = two_spheres();
        // Dock nose to nose: A's +X port meets B's -X port at the origin.
        let joint = world
            .attach_fixed_joint(
                a,
                b,
                DVec3::new(2.0, 0.0, 0.0),
                DQuat::IDENTITY,
                DVec3::new(-2.0, 0.0, 0.0),
                DQuat::IDENTITY,
            )
            .unwrap();
        assert_eq!(world.joint_count(), 1);
        // Shove A; the joint must drag B along.
        let push = ExternalWrench {
            force_inertial_n: DVec3::new(500.0, 0.0, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..120 {
            world
                .step(1.0 / 120.0, [(a, push), (b, ExternalWrench::ZERO)])
                .unwrap();
        }
        let state_a = world.body_state(a).unwrap();
        let state_b = world.body_state(b).unwrap();
        let separation = (state_b.position_inertial_m - state_a.position_inertial_m).length();
        assert!(
            (separation - 4.0).abs() < 0.05,
            "docked separation must hold, got {separation}"
        );
        assert!(
            state_b.velocity_inertial_mps.x > 1.0,
            "docked partner must be dragged along"
        );
        // Undock with no wrenches anywhere: both clusters coast at their
        // solved velocities (A would otherwise ram B from behind and the
        // contact, not the joint, would do work).
        world.remove_joint(joint).unwrap();
        assert_eq!(world.joint_count(), 0);
        let sep_before = (world.body_state(b).unwrap().position_inertial_m
            - world.body_state(a).unwrap().position_inertial_m)
            .length();
        let v_before = world.body_state(b).unwrap().velocity_inertial_mps;
        for _ in 0..60 {
            world
                .step(
                    1.0 / 120.0,
                    [(a, ExternalWrench::ZERO), (b, ExternalWrench::ZERO)],
                )
                .unwrap();
        }
        let after_b = world.body_state(b).unwrap();
        let sep_after = (after_b.position_inertial_m
            - world.body_state(a).unwrap().position_inertial_m)
            .length();
        assert!(
            (after_b.velocity_inertial_mps - v_before).length() < 1.0e-6,
            "undocked partner must coast force-free"
        );
        assert!(
            (sep_after - sep_before).abs() < 1.0e-6,
            "undock must not kick the clusters: separation {sep_before} -> {sep_after}"
        );
    }

    #[test]
    fn two_d1_craft_progress_through_rapier_docking_and_undock() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        let properties =
            RigidBodyProperties::new(1_200.0, DMat3::from_diagonal(DVec3::splat(900.0))).unwrap();
        let geometry_a = d1_craft_geometry(0.8);
        let geometry_b = d1_craft_geometry(-0.8);
        let state_a = RigidBodyState::new(
            DVec3::new(-0.805, 0.0, 0.0),
            DVec3::new(0.01, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let state_b = RigidBodyState::new(
            DVec3::new(0.805, 0.0, 0.0),
            DVec3::new(-0.01, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::ZERO,
        )
        .unwrap();
        let a = world
            .insert_dynamic_body(
                state_a,
                properties,
                &geometry_a,
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let b = world
            .insert_dynamic_body(
                state_b,
                properties,
                &geometry_b,
                DynamicBodyConfig::default(),
            )
            .unwrap();

        let port_a =
            DockingPortSpec::d1("craft-a-d1", DVec3::new(0.8, 0.0, 0.0), DQuat::IDENTITY).unwrap();
        let port_b =
            DockingPortSpec::d1("craft-b-d1", DVec3::new(-0.8, 0.0, 0.0), DQuat::IDENTITY).unwrap();
        let mut docking = DockingSession::new(port_a.clone(), port_b.clone(), 0.25).unwrap();
        let initial_relative_position = port_frame_world(state_b, port_b.local_position_m)
            - port_frame_world(state_a, port_a.local_position_m);
        docking.begin_soft_capture(0.02).unwrap();
        world
            .step(
                1.0 / 120.0,
                [(a, ExternalWrench::ZERO), (b, ExternalWrench::ZERO)],
            )
            .unwrap();
        let solved_a = world.body_state(a).unwrap();
        let solved_b = world.body_state(b).unwrap();
        docking
            .align(DockingKinematics::between(solved_a, &port_a, solved_b, &port_b).unwrap())
            .unwrap();
        assert_eq!(docking.state, DockingPortState::Aligned);

        docking.hard_dock().unwrap();
        let joint = world
            .attach_fixed_joint(
                a,
                b,
                port_a.local_position_m,
                port_a.local_orientation,
                port_b.local_position_m,
                port_b.local_orientation,
            )
            .unwrap();
        docking.engage_outer_structure().unwrap();
        docking.advance_pressure_equalization(0.25).unwrap();
        assert_eq!(docking.state, DockingPortState::PressureEqualized);
        assert_eq!(world.joint_count(), 1);

        let push = ExternalWrench {
            force_inertial_n: DVec3::new(1_500.0, 0.0, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..120 {
            world
                .step(1.0 / 120.0, [(a, push), (b, ExternalWrench::ZERO)])
                .unwrap();
        }
        let moved_a = world.body_state(a).unwrap();
        let moved_b = world.body_state(b).unwrap();
        let center_separation =
            (moved_b.position_inertial_m - moved_a.position_inertial_m).length();
        assert!((center_separation - 1.61).abs() < 0.05);
        assert!(moved_b.velocity_inertial_mps.x > 0.5);
        let solved_relative_position = port_frame_world(moved_b, port_b.local_position_m)
            - port_frame_world(moved_a, port_a.local_position_m);
        assert!(solved_relative_position.length() < 0.02);

        docking.undock().unwrap();
        world.remove_joint(joint).unwrap();
        assert_eq!(docking.state, DockingPortState::Free);
        assert!(initial_relative_position.length() < 0.02);
        assert_eq!(world.joint_count(), 0);
    }

    #[test]
    fn revolute_joint_keeps_d1_mechanism_anchor_and_allows_hinge_rotation() {
        let (mut world, a, b) = two_spheres();
        let joint = world
            .attach_revolute_joint(a, b, DVec3::X, DVec3::NEG_X, DVec3::Z, DVec3::Z)
            .unwrap();
        let torque = ExternalWrench {
            force_inertial_n: DVec3::ZERO,
            torque_inertial_nm: DVec3::new(0.0, 0.0, 10.0),
        };
        for _ in 0..120 {
            world
                .step(1.0 / 120.0, [(a, torque), (b, ExternalWrench::ZERO)])
                .unwrap();
        }
        let state_a = world.body_state(a).unwrap();
        let state_b = world.body_state(b).unwrap();
        let anchor_error = ((state_a.position_inertial_m
            + state_a.orientation_body_to_inertial * DVec3::X)
            - (state_b.position_inertial_m + state_b.orientation_body_to_inertial * DVec3::NEG_X))
            .length();
        assert!(
            anchor_error < 0.02,
            "hinge anchor drifted by {anchor_error} m"
        );
        assert!(state_a.angular_velocity_body_rps.z > 0.1);
        assert_eq!(world.joint_count(), 1);
        world.remove_joint(joint).unwrap();
    }

    #[test]
    fn removing_a_body_drops_its_joints() {
        let (mut world, a, b) = two_spheres();
        world
            .attach_fixed_joint(
                a,
                b,
                DVec3::X,
                DQuat::IDENTITY,
                DVec3::NEG_X,
                DQuat::IDENTITY,
            )
            .unwrap();
        world.remove_dynamic_body(a).unwrap();
        assert_eq!(world.joint_count(), 0);
    }

    #[test]
    fn settled_contact_reports_load_evidence_and_drains_once() {
        let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
        let mut world = CollisionWorld::new(frame).unwrap();
        world
            .insert_static_cuboid(
                DVec3::new(0.0, -0.5, 0.0),
                DQuat::IDENTITY,
                DVec3::new(10.0, 0.5, 10.0),
                CollisionMaterial::default(),
            )
            .unwrap();
        let mass_kg = 5.0;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
        let id = world
            .insert_dynamic_body(
                RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
            torque_inertial_nm: DVec3::ZERO,
        };
        for _ in 0..360 {
            world.step(1.0 / 120.0, [(id, gravity)]).unwrap();
        }
        let snapshot = world.debug_snapshot().unwrap();
        assert_eq!(snapshot.contacts.len(), 1);
        let contact = snapshot.contacts[0];
        assert!(contact.penetration_m >= 0.0);
        assert!(contact.penetration_m < 0.05);
        assert!(contact.approach_speed_mps.abs() < 0.2);
        assert_eq!(snapshot.patches.len(), 1);
        // Drain semantics: first drain takes the step's events, the second
        // is empty until another step records.
        let drained = world.drain_contact_events();
        assert_eq!(drained.len(), 1);
        assert!(world.drain_contact_events().is_empty());
    }

    #[test]
    fn identical_input_sequences_replay_identically() {
        // Same-binary replay gate for the determinism story: two worlds from
        // the same setup and wrench tape must produce identical snapshots.
        fn run_tape() -> String {
            let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
            let mut world = CollisionWorld::new(frame).unwrap();
            world
                .insert_static_cuboid(
                    DVec3::new(0.0, -0.5, 0.0),
                    DQuat::IDENTITY,
                    DVec3::new(10.0, 0.5, 10.0),
                    CollisionMaterial::default(),
                )
                .unwrap();
            let mass_kg = 5.0;
            let properties =
                RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
            let id = world
                .insert_dynamic_body(
                    RigidBodyState::new(
                        DVec3::new(0.5, 3.0, -0.25),
                        DVec3::new(2.0, -1.0, 0.5),
                        DQuat::from_rotation_y(0.4),
                        DVec3::new(0.5, -0.3, 0.2),
                    )
                    .unwrap(),
                    properties,
                    &sphere_geometry(0.5),
                    DynamicBodyConfig::default(),
                )
                .unwrap();
            for step in 0..180 {
                let thrust = if step < 60 {
                    DVec3::new(30.0, 5.0, -10.0)
                } else {
                    DVec3::ZERO
                };
                world
                    .step(
                        1.0 / 120.0,
                        [(
                            id,
                            ExternalWrench {
                                force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0) + thrust,
                                torque_inertial_nm: DVec3::new(0.0, 0.0, 1.5),
                            },
                        )],
                    )
                    .unwrap();
            }
            let snapshot = world.debug_snapshot().unwrap();
            serde_json::to_string(&snapshot).unwrap()
        }
        assert_eq!(run_tape(), run_tape());
    }
}
