//! Dynamic bodies and static/kinematic collider lifecycle.

use super::*;

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
}
