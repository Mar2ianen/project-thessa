//! Fixed, hinge, slider, suspension, and wheel-joint operations.

use super::*;

impl CollisionWorld {
    /// Read the current joint solver impulses as average force/moment loads
    /// over `step_s`. This reports solver evidence only; structural ratings
    /// and failure policy belong to the vehicle domain.
    pub fn joint_loads(&self, step_s: f64) -> Result<Vec<JointLoadSummary>, CollisionBackendError> {
        if !step_s.is_finite() || step_s < MIN_STEP_S {
            return Err(CollisionBackendError::InvalidStep(step_s));
        }
        let mut loads = Vec::with_capacity(self.joints.len());
        for (joint_id, entry) in &self.joints {
            let Some(joint) = self.impulse_joints.get(entry.rapier) else {
                continue;
            };
            // Rapier's 3D spatial joint vector stores the three translational
            // DOFs first and the three rotational DOFs second.
            let force_impulse = DVec3::new(joint.impulses[0], joint.impulses[1], joint.impulses[2]);
            let torque_impulse =
                DVec3::new(joint.impulses[3], joint.impulses[4], joint.impulses[5]);
            loads.push(JointLoadSummary {
                joint_id: *joint_id,
                body_a: entry.a,
                body_b: entry.b,
                force_n: force_impulse.length() / step_s,
                torque_nm: torque_impulse.length() / step_s,
            });
        }
        Ok(loads)
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

    /// Update a suspension joint's frames as a wheel chassis folds about its
    /// vehicle hinge. Both bodies use vehicle-aligned local axes in this
    /// reduced wheel model, so callers provide the current slide/axle pair for
    /// each frame. The solver is woken before the next contact step.
    #[allow(clippy::too_many_arguments)]
    pub fn update_suspension_wheel_joint_frames(
        &mut self,
        id: JointId,
        anchor_sprung_local_m: DVec3,
        anchor_wheel_local_m: DVec3,
        slide_axis_sprung_local: DVec3,
        axle_axis_sprung_local: DVec3,
        slide_axis_wheel_local: DVec3,
        axle_axis_wheel_local: DVec3,
    ) -> Result<(), CollisionBackendError> {
        validate_local_vector(anchor_sprung_local_m, "sprung wheel-joint anchor")?;
        validate_local_vector(anchor_wheel_local_m, "wheel wheel-joint anchor")?;
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
        let entry = self
            .joints
            .get(&id)
            .ok_or(CollisionBackendError::UnknownJoint(id))?;
        let joint = self
            .impulse_joints
            .get_mut(entry.rapier, true)
            .ok_or(CollisionBackendError::BackendStateLost(entry.a))?;
        joint.data.set_local_frame1(Pose::from_parts(
            to_rapier_vector(anchor_sprung_local_m),
            to_rapier_rotation(frame_sprung),
        ));
        joint.data.set_local_frame2(Pose::from_parts(
            to_rapier_vector(anchor_wheel_local_m),
            to_rapier_rotation(frame_wheel),
        ));
        Ok(())
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
}
