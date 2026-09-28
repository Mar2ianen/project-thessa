//! Articulated and conventional wheel contact force queries.

use super::super::*;

impl CollisionWorld {
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
}
