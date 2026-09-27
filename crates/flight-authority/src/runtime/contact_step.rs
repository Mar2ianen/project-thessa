//! Terrain-backed contact activation, patch anchoring, and rigid-body ticks.

use super::*;

impl FlightAuthority {
    /// Observe terrain evidence and maintain the contact regime. Returns true
    /// when this tick must integrate through Rapier. A rising edge
    /// invalidates rails; a falling edge drops the backend body and patch so
    /// no stale backend state survives the return to free flight.
    pub(super) fn poll_contact_activation(
        &mut self,
        kinematics: LocalAirKinematics,
    ) -> Result<bool, FlightError> {
        if self.contact.is_none() || self.vehicle.collision_geometry.is_empty() {
            return Ok(false);
        }
        let clearance_m = match &self.terrain_field {
            Some(field) => {
                let dir = ground_dir_body_fixed(
                    kinematics.relative_position_inertial_m,
                    self.flight_time_s,
                    self.body_rotation_period_s,
                );
                let surface = field.params.radius_m + field.height_m(dir.to_array(), 32.0).max(0.0);
                kinematics.relative_position_inertial_m.length() - surface
            }
            None => kinematics.relative_position_inertial_m.length() - self.planet_radius_m,
        };
        let runtime = self
            .contact
            .as_mut()
            .ok_or_else(|| FlightError::InvalidInput("contact mode is not enabled".into()))?;
        let was_active = runtime.is_active();
        let (active, just_activated) = runtime.observe_distance(clearance_m);
        if just_activated {
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
        }
        if was_active && !active {
            runtime.remove_body()?;
            self.last_wheel_contacts.clear();
            self.last_wheel_drive_points.clear();
            self.last_wheel_gear_actuators.clear();
            self.last_landing_leg_contacts.clear();
            self.last_landing_leg_actuators.clear();
            if let Some(patch) = self.contact_patch.take() {
                runtime.evict_kinematic_terrain(patch)?;
            }
            self.contact_patch_anchor = None;
        }
        Ok(active)
    }

    pub(super) fn make_contact_patch_anchor(
        &self,
        kinematics: LocalAirKinematics,
    ) -> Result<ContactPatchAnchor, FlightError> {
        let direction_body_fixed = ground_dir_body_fixed(
            kinematics.relative_position_inertial_m,
            self.flight_time_s,
            self.body_rotation_period_s,
        );
        let terrain_height_m = self
            .terrain_field
            .as_ref()
            .map(|field| {
                field
                    .height_m(direction_body_fixed.to_array(), 32.0)
                    .max(0.0)
            })
            .unwrap_or(0.0);
        let body_to_inertial =
            body_fixed_to_inertial_rotation(self.flight_time_s, self.body_rotation_period_s);
        let up = body_to_inertial * direction_body_fixed;
        let (east, north) = surface_tangent_basis(up)?;
        let orientation_inertial = DQuat::from_mat3(&DMat3::from_cols(east, up, -north));
        Ok(ContactPatchAnchor {
            direction_body_fixed,
            orientation_body_fixed: (body_to_inertial.inverse() * orientation_inertial).normalize(),
            terrain_height_m,
        })
    }

    pub(super) fn contact_patch_pose(
        &self,
        anchor: ContactPatchAnchor,
        body_state: BodyState,
        time_s: f64,
    ) -> (DVec3, DQuat) {
        let body_to_inertial = body_fixed_to_inertial_rotation(time_s, self.body_rotation_period_s);
        let up = body_to_inertial * anchor.direction_body_fixed;
        let surface_radius_m = self.planet_radius_m + anchor.terrain_height_m;
        let center_inertial =
            body_state.position_inertial + up * (surface_radius_m - CONTACT_PATCH_HALF_THICK_M);
        let orientation_inertial = (body_to_inertial * anchor.orientation_body_fixed).normalize();
        (center_inertial, orientation_inertial)
    }

    pub(super) fn contact_patch_needs_recenter(
        &self,
        anchor: ContactPatchAnchor,
        kinematics: LocalAirKinematics,
    ) -> bool {
        let body_to_inertial =
            body_fixed_to_inertial_rotation(self.flight_time_s, self.body_rotation_period_s);
        let up = body_to_inertial * anchor.direction_body_fixed;
        let anchor_radius_m = self.planet_radius_m + anchor.terrain_height_m;
        let displacement = kinematics.relative_position_inertial_m - up * anchor_radius_m;
        let tangent_displacement = displacement - up * displacement.dot(up);
        tangent_displacement.length() > CONTACT_PATCH_RECENTER_DISTANCE_M
    }

    /// One contact-active tick: the same sampled loads as the powered step,
    /// integrated by Rapier instead of `integrate_rigid_body_step_soa`. The
    /// kinematic terrain patch is re-posed from the next-tick ephemeris so
    /// the position-based body carries the surface velocity into this step.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn step_contact_active(
        &mut self,
        ephemeris: &BakedEphemeris,
        body_state: BodyState,
        gravity: DVec3,
        kinematics: LocalAirKinematics,
        jet_moment: DVec3,
        rcs_force_body_n: DVec3,
        thrust_n: f64,
        band_drag_body_n: DVec3,
        parachute_force_body_n: DVec3,
        parachute_moment_body_nm: DVec3,
        skip_aero: bool,
        precomputed_aero: Option<AeroResult>,
    ) -> Result<(RigidBodyState, FlightForces), FlightError> {
        if !self.vehicle.wheel_chassis.is_empty() || !self.vehicle.landing_legs.is_empty() {
            self.sync_gear_deployment_commands();
            self.sync_wheel_runtime_state();
            self.sync_wheel_gear_runtime_state();
            self.sync_landing_leg_runtime_state();
        }
        let state = self.state;
        let properties = self.vehicle.mass_properties;
        let input = powered_step_input(
            state,
            kinematics,
            body_state.velocity_inertial,
            gravity,
            jet_moment,
            rcs_force_body_n,
            thrust_n,
            band_drag_body_n,
            parachute_force_body_n,
            parachute_moment_body_nm,
            skip_aero,
        );
        let forces = match precomputed_aero {
            Some(aero_result) => evaluate_flight_forces_with_aero_result(
                self.atmosphere,
                state,
                properties,
                input,
                aero_result,
            )?,
            None => self.evaluate_forces(state, input)?,
        };
        let wrench = ContactRuntime::evaluate_wrench(state, properties, gravity, &forces)?;

        let next_time = self.time_after_ticks(1)?;
        let next_home = ephemeris
            .body_state(self.reference_body, next_time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let existing_patch = self.contact_patch;
        let existing_anchor = self.contact_patch_anchor;
        let recenter_patch = existing_patch.is_some()
            && existing_anchor
                .is_none_or(|anchor| self.contact_patch_needs_recenter(anchor, kinematics));
        let anchor = match existing_anchor {
            Some(anchor) if !recenter_patch => anchor,
            _ => self.make_contact_patch_anchor(kinematics)?,
        };
        // Attach at the current ephemeris pose, then publish the next-tick
        // pose. Rapier therefore observes only the body's real translation
        // and rotation, including on the first contact-active tick.
        let (current_center_inertial, current_orientation) =
            self.contact_patch_pose(anchor, body_state, self.flight_time_s);
        let (next_center_inertial, next_orientation) =
            self.contact_patch_pose(anchor, next_home, next_time.0);
        let half_extents = DVec3::new(
            CONTACT_PATCH_HALF_M,
            CONTACT_PATCH_HALF_THICK_M,
            CONTACT_PATCH_HALF_M,
        );
        let articulated_gear =
            !self.vehicle.wheel_chassis.is_empty() || !self.vehicle.landing_legs.is_empty();
        let (
            next,
            patch,
            wheel_contacts,
            wheel_drive_points,
            landing_leg_contacts,
            landing_leg_actuators,
            wheel_gear_actuators,
        ) = {
            let runtime = self
                .contact
                .as_mut()
                .ok_or_else(|| FlightError::InvalidInput("contact mode is not enabled".into()))?;
            let frame = runtime.frame();
            let current_center_local = frame.position_to_local(current_center_inertial);
            let current_orientation_local = frame.orientation_to_local(current_orientation);
            let next_center_local = frame.position_to_local(next_center_inertial);
            let next_orientation_local = frame.orientation_to_local(next_orientation);
            let patch = if recenter_patch {
                if let Some(id) = existing_patch {
                    runtime.evict_kinematic_terrain(id)?;
                }
                runtime.attach_kinematic_terrain(
                    current_center_local,
                    current_orientation_local,
                    half_extents,
                    CollisionMaterial::default(),
                )?
            } else if let Some(id) = existing_patch {
                id
            } else {
                runtime.attach_kinematic_terrain(
                    current_center_local,
                    current_orientation_local,
                    half_extents,
                    CollisionMaterial::default(),
                )?
            };
            runtime.move_kinematic_terrain(patch, next_center_local, next_orientation_local)?;
            let (
                next,
                wheel_contacts,
                wheel_drive_points,
                landing_leg_contacts,
                landing_leg_actuators,
                wheel_gear_actuators,
            ) = if articulated_gear {
                runtime.step_articulated_vehicle_with_gear_targets(
                    FLIGHT_STEP_S,
                    state,
                    gravity,
                    &forces,
                    &self.vehicle,
                    self.wheel_brake_command,
                    self.wheel_drive_command,
                    &mut self.wheel_spin_rad_s,
                    &mut self.wheel_brake_states,
                    &mut self.wheel_chassis_states,
                    &mut self.landing_leg_states,
                    &self.landing_leg_deployment_commands,
                    &self.wheel_chassis_deployment_commands,
                )?
            } else {
                runtime.sync_body(
                    state,
                    properties,
                    &self.vehicle.collision_geometry,
                    DynamicBodyConfig::default(),
                )?;
                (
                    runtime.step(FLIGHT_STEP_S, wrench)?,
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                )
            };
            (
                next,
                patch,
                wheel_contacts,
                wheel_drive_points,
                landing_leg_contacts,
                landing_leg_actuators,
                wheel_gear_actuators,
            )
        };
        self.contact_patch = Some(patch);
        self.contact_patch_anchor = Some(anchor);
        self.last_wheel_contacts = wheel_contacts;
        self.last_wheel_drive_points = wheel_drive_points;
        self.last_wheel_gear_actuators = wheel_gear_actuators;
        self.last_landing_leg_contacts = landing_leg_contacts;
        self.last_landing_leg_actuators = landing_leg_actuators;
        Ok((next, forces))
    }
}
