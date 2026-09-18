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
    ImpulseJointHandle, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    NarrowPhase, PhysicsPipeline, RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
};
use thessa_sim_core::{
    CollisionAxis, CollisionGeometry, CollisionMaterial, CollisionShape, FlightForces,
    RigidBodyProperties, RigidBodyState,
};

const QUATERNION_TOLERANCE: f64 = 1.0e-6;
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
        if geometry.is_empty() {
            return Err(CollisionBackendError::InvalidGeometry(
                "dynamic contact body needs at least one collision part".into(),
            ));
        }
        geometry
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
        validate_properties(properties)?;

        let id = CollisionBodyId(self.next_body_id);
        self.next_body_id = self
            .next_body_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;

        let local_position = self.frame.position_to_local(state.position_inertial_m);
        let local_orientation =
            (self.frame.local_from_inertial() * state.orientation_body_to_inertial).normalize();
        let local_linear_velocity = self.frame.velocity_to_local(state.velocity_inertial_mps);
        let angular_velocity_inertial =
            state.orientation_body_to_inertial * state.angular_velocity_body_rps;
        let local_angular_velocity = self.frame.vector_to_local(angular_velocity_inertial);

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
        validate_properties(properties)?;
        let entry = self
            .dynamic
            .get(&id)
            .ok_or(CollisionBackendError::UnknownBody(id))?;
        let handle = entry.rapier;
        let body = self
            .bodies
            .get_mut(handle)
            .ok_or(CollisionBackendError::BackendStateLost(id))?;
        let local_position = self.frame.position_to_local(state.position_inertial_m);
        let local_orientation =
            (self.frame.local_from_inertial() * state.orientation_body_to_inertial).normalize();
        let local_linear_velocity = self.frame.velocity_to_local(state.velocity_inertial_mps);
        let angular_velocity_inertial =
            state.orientation_body_to_inertial * state.angular_velocity_body_rps;
        let local_angular_velocity = self.frame.vector_to_local(angular_velocity_inertial);
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
        || properties.mass_kg <= 0.0
        || !properties.inertia_body_kg_m2.is_finite()
        || properties.inertia_body_kg_m2.determinant() <= 0.0
    {
        return Err(CollisionBackendError::InvalidMassProperties);
    }
    Ok(())
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
    InvalidMassProperties,
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
            Self::InvalidMassProperties => write!(formatter, "invalid rigid-body mass properties"),
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
    use thessa_sim_core::{CollisionPart, CollisionShape};

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
    use thessa_sim_core::{CollisionPart, CollisionShape};

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
