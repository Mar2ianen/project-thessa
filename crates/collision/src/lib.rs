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
    BroadPhaseBvh, CCDSolver, ColliderBuilder, ColliderHandle, ColliderSet, ImpulseJointSet,
    IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
    RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
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

    fn position_to_local(self, inertial_m: DVec3) -> DVec3 {
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
    last_wrench: ExternalWrench,
}

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
    fixed: BTreeMap<StaticColliderId, ColliderHandle>,
    next_body_id: u64,
    next_static_id: u64,
}

impl CollisionWorld {
    pub fn new(frame: CollisionFrame) -> Result<Self, CollisionBackendError> {
        frame.validate()?;
        Ok(Self {
            frame,
            pipeline: PhysicsPipeline::new(),
            integration: IntegrationParameters::default(),
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
            next_body_id: 0,
            next_static_id: 0,
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
            self.colliders
                .insert_with_parent(collider, handle, &mut self.bodies);
        }

        self.dynamic.insert(
            id,
            DynamicBodyEntry {
                rapier: handle,
                last_wrench: ExternalWrench::ZERO,
            },
        );
        Ok(id)
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
        self.register_fixed(handle)
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
        material
            .validate()
            .map_err(|error| CollisionBackendError::InvalidGeometry(error.to_string()))?;
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
        self.register_fixed(handle)
    }

    fn register_fixed(
        &mut self,
        handle: ColliderHandle,
    ) -> Result<StaticColliderId, CollisionBackendError> {
        let id = StaticColliderId(self.next_static_id);
        self.next_static_id = self
            .next_static_id
            .checked_add(1)
            .ok_or(CollisionBackendError::IdentifierExhausted)?;
        self.fixed.insert(id, handle);
        Ok(id)
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
    DuplicateWrench(CollisionBodyId),
    BackendStateLost(CollisionBodyId),
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
}
