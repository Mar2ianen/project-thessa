//! Landing-footpad contact and absorber queries.

use super::super::*;

impl CollisionWorld {
    /// Query fold-out footpads against fixed or kinematic terrain and evaluate
    /// their reusable or crushable absorber laws. `body_origin_offset_body_m`
    /// maps the authored total-COM frame into the synced sprung-body frame.
    pub fn evaluate_landing_leg_contacts(
        &self,
        body_id: CollisionBodyId,
        legs: &[CompiledLandingLeg],
        states: &[LandingLegState],
        body_origin_offset_body_m: DVec3,
        deployment_command: LandingLegDeploymentCommand<'_>,
        step_s: f64,
    ) -> Result<LandingLegContactResult, CollisionBackendError> {
        let entry = self
            .dynamic
            .get(&body_id)
            .ok_or(CollisionBackendError::UnknownBody(body_id))?;
        let body = self
            .bodies
            .get(entry.rapier)
            .ok_or(CollisionBackendError::BackendStateLost(body_id))?;
        if legs.len() != states.len()
            || matches!(deployment_command, LandingLegDeploymentCommand::PerLeg(commands) if commands.len() != legs.len())
            || !body_origin_offset_body_m.is_finite()
            || !step_s.is_finite()
            || step_s <= 0.0
        {
            return Err(CollisionBackendError::InvalidLandingLegContact(
                "compiled legs, persistent states, body offset and step must be valid".into(),
            ));
        }

        let root_pose = body.position();
        let root_position_local = from_rapier_vector(root_pose.translation);
        let root_orientation_local = from_rapier_rotation(root_pose.rotation);
        let query = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.bodies,
            &self.colliders,
            QueryFilter::default().exclude_rigid_body(entry.rapier),
        );
        let mut contacts = Vec::with_capacity(legs.len());
        let mut next_states = states.to_vec();
        let mut wrench = ExternalWrench::ZERO;

        for (index, (leg, state)) in legs.iter().zip(states.iter().copied()).enumerate() {
            leg.spec.validate().map_err(|error| {
                CollisionBackendError::InvalidLandingLegContact(error.to_string())
            })?;
            let fraction = state.deployment_fraction;
            if !fraction.is_finite()
                || !(0.0..=1.0).contains(&fraction)
                || !state.permanent_crush_m.is_finite()
                || state.permanent_crush_m < 0.0
                || !state.absorbed_energy_j.is_finite()
                || state.absorbed_energy_j < 0.0
                || state.permanent_crush_m > leg.spec.shock_absorber.maximum_permanent_crush_m()
            {
                return Err(CollisionBackendError::InvalidLandingLegContact(
                    "deployment, crush and absorbed-energy state must be finite and in range"
                        .into(),
                ));
            }
            if fraction <= 1.0e-9 {
                continue;
            }

            let spec = &leg.spec;
            let axis_body = spec.leg_axis_body_at_fraction(fraction).normalize();
            let axis_local = (root_orientation_local * axis_body).normalize();
            let mount_local = root_position_local
                + root_orientation_local * (spec.mount_position_body_m - body_origin_offset_body_m);
            // At shallow angles the sphere-foot contact becomes ill-conditioned
            // as an axial support. Excluding alignments below 0.1 bounds the
            // terrain-normal-to-strut load amplification to 10.
            const MIN_SUPPORT_ALIGNMENT: f64 = 0.1;
            let effective_leg_length_m = spec.leg_length_m - state.permanent_crush_m;
            let max_toi = effective_leg_length_m + spec.footpad_radius_m / MIN_SUPPORT_ALIGNMENT;
            let ray = Ray::new(to_rapier_vector(mount_local), to_rapier_vector(axis_local));
            let nearest = query
                .intersect_ray(ray, max_toi, true)
                .filter_map(|(handle, collider, hit)| {
                    let normal = from_rapier_vector(hit.normal).normalize();
                    let alignment = -normal.dot(axis_local);
                    if alignment < MIN_SUPPORT_ALIGNMENT {
                        return None;
                    }
                    if let Some(parent) = collider.parent() {
                        let terrain_body = self.bodies.get(parent)?;
                        if terrain_body.is_dynamic() {
                            return None;
                        }
                    }
                    Some((handle, collider, hit, alignment))
                })
                .min_by(|left, right| left.2.time_of_impact.total_cmp(&right.2.time_of_impact));
            let Some((_, terrain_collider, hit, alignment)) = nearest else {
                continue;
            };

            let normal_local = from_rapier_vector(hit.normal).normalize();
            let ray_surface_point_local = mount_local + axis_local * hit.time_of_impact;
            // Shock compression is referenced to the pristine leg length.
            // The ray reach uses the shortened installed length above, while
            // adding permanent crush back here prevents consuming that crush
            // twice in the absorber law.
            let compression_m = (spec.leg_length_m - hit.time_of_impact
                + spec.footpad_radius_m / alignment)
                .max(0.0);
            if compression_m <= 0.0 {
                continue;
            }
            let foot_center_local = mount_local + axis_local * (spec.leg_length_m - compression_m);
            let signed_center_distance_m =
                (foot_center_local - ray_surface_point_local).dot(normal_local);
            let contact_point_local = foot_center_local - normal_local * signed_center_distance_m;

            let body_velocity_local =
                from_rapier_vector(body.velocity_at_point(to_rapier_vector(contact_point_local)));
            let terrain_velocity_local = terrain_collider
                .parent()
                .and_then(|parent| self.bodies.get(parent))
                .map(|terrain_body| {
                    from_rapier_vector(
                        terrain_body.velocity_at_point(to_rapier_vector(contact_point_local)),
                    )
                })
                .unwrap_or(DVec3::ZERO);
            let relative_velocity_local = body_velocity_local - terrain_velocity_local;
            let compression_rate_mps = -relative_velocity_local.dot(normal_local) / alignment;
            let (shock, next_state) = spec
                .shock_absorber
                .evaluate(state, compression_m, compression_rate_mps, step_s)
                .map_err(|error| {
                    CollisionBackendError::InvalidLandingLegContact(error.to_string())
                })?;
            next_states[index] = next_state;

            let friction = 0.5 * (spec.footpad_friction + terrain_collider.friction());
            let normal_load_n = shock.force_axial_n / alignment;
            let tangential_velocity_local =
                relative_velocity_local - normal_local * relative_velocity_local.dot(normal_local);
            let requested_tangent_local =
                -tangential_velocity_local * spec.footpad_slip_stiffness_n_per_mps;
            let requested_tangent_n = requested_tangent_local.length();
            let friction_limit_n = friction * normal_load_n;
            let saturated = requested_tangent_n > friction_limit_n;
            let tangent_force_local = if saturated && requested_tangent_n > 1.0e-12 {
                requested_tangent_local * (friction_limit_n / requested_tangent_n)
            } else {
                requested_tangent_local
            };
            let contact_force_local = normal_local * normal_load_n + tangent_force_local;
            let contact_force_inertial = self.frame.vector_to_inertial(contact_force_local);
            let contact_point_inertial = self.frame.position_to_inertial(contact_point_local);
            wrench.force_inertial_n += contact_force_inertial;
            wrench.torque_inertial_nm += self.frame.vector_to_inertial(
                (contact_point_local - root_position_local).cross(contact_force_local),
            );

            let hinge_axis_local = (root_orientation_local * spec.hinge_axis_body).normalize();
            let moment_about_hinge_nm = (contact_point_local - mount_local)
                .cross(contact_force_local)
                .dot(hinge_axis_local);
            let mut commanded_rotation_sign =
                (spec.deployed_angle_rad - spec.stowed_angle_rad).signum();
            let command_deployed = match deployment_command {
                LandingLegDeploymentCommand::All(deployed) => deployed,
                LandingLegDeploymentCommand::PerLeg(commands) => commands[index],
            };
            if !command_deployed {
                commanded_rotation_sign = -commanded_rotation_sign;
            }
            let resisting_torque_nm = (-moment_about_hinge_nm * commanded_rotation_sign).max(0.0);
            contacts.push(LandingLegContactSample {
                leg_index: u16::try_from(index).map_err(|_| {
                    CollisionBackendError::InvalidLandingLegContact(
                        "landing-leg index exceeds the telemetry format".into(),
                    )
                })?,
                contact_point_inertial_m: contact_point_inertial,
                terrain_normal_inertial: self.frame.vector_to_inertial(normal_local),
                relative_contact_velocity_inertial_mps: self
                    .frame
                    .vector_to_inertial(relative_velocity_local),
                compression_m,
                compression_rate_mps,
                axial_force_n: shock.force_axial_n,
                normal_load_n,
                tangential_force_inertial_n: self.frame.vector_to_inertial(tangent_force_local),
                contact_friction: friction,
                permanent_crush_m: shock.permanent_crush_m,
                absorbed_energy_delta_j: shock.absorbed_energy_delta_j,
                actuator_resisting_torque_nm: resisting_torque_nm,
                bottomed_out: shock.bottomed_out,
                exhausted: shock.exhausted,
                saturated: saturated || shock.exhausted,
            });
        }
        wrench.validate()?;
        Ok(LandingLegContactResult {
            contacts,
            states: next_states,
            wrench,
        })
    }
}
