//! Flight assist requests moments, but only surfaces and finite RCS force
//! couples deliver them. No controller torque is added directly to the body.
use super::*;
use bevy::math::DMat3;
use thessa_sim_core::{AeroModel, AeroState};

pub(super) const FLIGHT_STEP_S: f64 = 1.0 / 120.0;
const SURFACE_COMMAND_RATE_S: f64 = 2.4; // 60 deg/s for the 25-degree elevator

fn body_axes(command: DVec3) -> DVec3 {
    DVec3::new(command.z, -command.x, -command.y)
}

// Each opposed pair has zero net force. Locations and force directions are
// expressed in body metres/newtons; these are prototype jets, not X-15 data.
fn rcs_couples() -> [DVec3; 3] {
    [
        2.0 * DVec3::new(0.0, 1.4, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(-5.0, 0.0, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(5.0, 0.0, 0.0).cross(DVec3::Y * 400.0),
    ]
}

fn rcs_moment(request: DVec3, enabled: bool) -> DVec3 {
    if !enabled {
        return DVec3::ZERO;
    }
    rcs_couples().into_iter().fold(DVec3::ZERO, |sum, couple| {
        sum + couple * (request.dot(couple) / couple.length_squared()).clamp(-1.0, 1.0)
    })
}

impl PilotFlightRuntime {
    /// Advance the exact production flight path at a fixed physics cadence.
    /// Render frames only contribute elapsed time; they never set solver dt.
    pub(super) fn advance(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
    ) -> Result<(), FlightError> {
        self.accumulator_s += elapsed_s;
        let gravity_field = GravityField::from_ephemeris(ephemeris);
        while self.accumulator_s + 1.0e-12 >= FLIGHT_STEP_S {
            self.step(ephemeris, &gravity_field, mode)?;
            self.accumulator_s = (self.accumulator_s - FLIGHT_STEP_S).max(0.0);
        }
        Ok(())
    }

    fn allocate_controls(
        &mut self,
        kinematics: LocalAirKinematics,
        mode: ControlMode,
    ) -> Result<DVec3, FlightError> {
        let axes = body_axes(self.control_input);
        let assisted = mode != ControlMode::Direct;
        let attitude_hold =
            self.sas_enabled && matches!(mode, ControlMode::Navball | ControlMode::MouseAim);
        let desired_rate = if attitude_hold {
            self.sas_target_orientation = (self.sas_target_orientation
                * DQuat::from_scaled_axis(
                    axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S * FLIGHT_STEP_S,
                ))
            .normalize();
            let mut error =
                self.state.orientation_body_to_inertial.inverse() * self.sas_target_orientation;
            // q and -q represent the same attitude; use the short rotation.
            if error.w < 0.0 {
                error = -error;
            }
            (error.to_scaled_axis() * 1.6).clamp_length_max(0.35)
        } else {
            // Capturing here avoids a jump to an old SAS target when re-enabled.
            self.sas_target_orientation = self.state.orientation_body_to_inertial;
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S
        };
        let inertia = self.vehicle.mass_properties.inertia_body_kg_m2;
        let omega = self.state.angular_velocity_body_rps;
        let requested = inertia * ((desired_rate - omega) / 0.35) + omega.cross(inertia * omega);
        let environment = self
            .atmosphere
            .aero_environment(kinematics.altitude_m.max(0.0), DVec3::ZERO)?;
        let aero_state = AeroState::new(kinematics.air_velocity_body_mps, omega);
        let mut command = if assisted {
            self.surface_input
        } else {
            self.control_input
        };
        if assisted {
            // Linearize actual panel response at this flow/deflection. Two
            // bounded Newton passes handle cross-axis coupling near stall.
            for _ in 0..2 {
                self.command_controls(command.x, command.y, command.z);
                let baseline = self
                    .aero_model
                    .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
                    .moment_body_nm;
                let mut columns = [DVec3::ZERO; 3];
                for axis in 0..3 {
                    let mut probe = command;
                    let delta = if command[axis] > 0.9 { -0.02 } else { 0.02 };
                    probe[axis] += delta;
                    self.command_controls(probe.x, probe.y, probe.z);
                    columns[axis] = (self
                        .aero_model
                        .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
                        .moment_body_nm
                        - baseline)
                        / delta;
                }
                let effectiveness = DMat3::from_cols(columns[0], columns[1], columns[2]);
                if effectiveness.determinant().abs() > 1.0e-6 {
                    command = (command + effectiveness.inverse() * (requested - baseline))
                        .clamp(DVec3::splat(-1.0), DVec3::ONE);
                }
            }
        }
        let max_change = SURFACE_COMMAND_RATE_S * FLIGHT_STEP_S;
        self.surface_input += (command - self.surface_input)
            .clamp(DVec3::splat(-max_change), DVec3::splat(max_change));
        self.command_controls(
            self.surface_input.x,
            self.surface_input.y,
            self.surface_input.z,
        );
        let actual_aero = self
            .aero_model
            .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
            .moment_body_nm;
        let jets = if assisted {
            rcs_moment(requested - actual_aero, self.rcs_enabled)
        } else {
            let couples = rcs_couples();
            rcs_moment(
                DVec3::new(couples[0].x, couples[1].y, couples[2].z) * axes,
                self.rcs_enabled,
            )
        };
        self.actuator_saturated = assisted
            && command.abs().max_element() > 0.99
            && (requested - actual_aero - jets).length() > 1_000.0;
        Ok(jets)
    }

    fn step(
        &mut self,
        ephemeris: &BakedEphemeris,
        gravity_field: &GravityField,
        mode: ControlMode,
    ) -> Result<(), FlightError> {
        let time = SimTime(self.flight_time_s);
        let body_state = ephemeris
            .body_state(self.reference_body, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        let gravity = gravity_field
            .acceleration(self.state.position_inertial_m, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let jet_moment = self.allocate_controls(kinematics, mode)?;
        let (mut next, forces) = thessa_sim_core::integrate_rigid_body_step(
            &self.aero_model,
            &self.vehicle.aero_geometry,
            self.atmosphere,
            self.state,
            self.vehicle.mass_properties,
            FlightStepInput {
                altitude_m: kinematics.altitude_m.max(0.0),
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: kinematics.relative_position_body_m,
                wind_velocity_body_mps: self.state.orientation_body_to_inertial.inverse()
                    * body_state.velocity_inertial,
                extra_force_body_n: DVec3::X * self.thrust_n(),
                extra_moment_body_nm: jet_moment,
            },
            FLIGHT_STEP_S,
        )?;
        let next_body = ephemeris
            .body_state(
                self.reference_body,
                SimTime(self.flight_time_s + FLIGHT_STEP_S),
            )
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let relative = next.position_inertial_m - next_body.position_inertial;
        if (relative.length() - self.planet_radius_m).abs() > MAX_PILOT_ALTITUDE_M
            || (next.velocity_inertial_mps - next_body.velocity_inertial).length()
                > MAX_PILOT_RELATIVE_SPEED_MPS
            || next.angular_velocity_body_rps.length() > MAX_PILOT_ANGULAR_RATE_RPS
        {
            return Err(FlightError::InvalidInput(
                "flight state exceeded solver bounds".into(),
            ));
        }
        // Existing spherical contact boundary; no invented angular damping.
        if relative.length() < self.planet_radius_m + PILOT_SURFACE_CLEARANCE_M {
            let up = relative.normalize();
            next.position_inertial_m = next_body.position_inertial
                + up * (self.planet_radius_m + PILOT_SURFACE_CLEARANCE_M);
            let radial_speed = (next.velocity_inertial_mps - next_body.velocity_inertial).dot(up);
            if radial_speed < 0.0 {
                next.velocity_inertial_mps -= up * radial_speed;
            }
        }
        if let Some(trace) = self.trace.as_mut() {
            trace.record(
                self.flight_time_s,
                kinematics.altitude_m,
                kinematics.relative_velocity_inertial_mps,
                kinematics.radial_up,
                kinematics.air_velocity_body_mps,
                self.state,
                self.control_input,
                self.throttle,
                self.engine_active,
                self.sas_enabled,
                self.rcs_enabled,
                &forces,
                mode,
                self.surface_input,
                self.actuator_saturated,
                self.sas_target_orientation,
            );
        }
        self.state = next;
        self.flight_time_s += FLIGHT_STEP_S;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(forces);
        self.render_position = pilot_render_offset(
            (next.position_inertial_m - next_body.position_inertial)
                - self.initial_relative_position_m,
        );
        self.render_orientation = render_orientation(next.orientation_body_to_inertial);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (BakedEphemeris, PilotFlightRuntime) {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../../data/system.toml")).unwrap();
        let ephemeris = config.bake().unwrap();
        let runtime =
            PilotFlightRuntime::new(&ephemeris, ephemeris.body_id("thessa").unwrap()).unwrap();
        (ephemeris, runtime)
    }

    #[test]
    fn surface_moments_follow_pilot_axes_without_rcs() {
        let (_, mut flight) = fixture();
        let env = flight
            .atmosphere
            .aero_environment(500.0, DVec3::ZERO)
            .unwrap();
        let flow = AeroState::new(DVec3::X * 180.0, DVec3::ZERO);
        let base = flight
            .aero_model
            .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
            .unwrap()
            .moment_body_nm;
        for command in [
            DVec3::X,
            DVec3::Y,
            DVec3::Z,
            -DVec3::X,
            -DVec3::Y,
            -DVec3::Z,
        ] {
            flight.command_controls(command.x, command.y, command.z);
            let moment = flight
                .aero_model
                .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
                .unwrap()
                .moment_body_nm;
            assert!(
                (moment - base).dot(body_axes(command)) > 0.0,
                "wrong surface sign for {command:?}"
            );
        }
        assert_eq!(rcs_moment(DVec3::splat(1.0e9), false), DVec3::ZERO);
        assert_eq!(
            rcs_moment(DVec3::splat(1.0e9), true),
            DVec3::new(1120.0, 4000.0, 4000.0)
        );
    }

    #[test]
    fn production_flight_is_stable_for_fifteen_minutes_and_independent_of_render_cadence() {
        let (ephemeris, mut flight) = fixture();
        let (_, mut other) = fixture();
        let start = std::time::Instant::now();
        let mut min_alt = f64::INFINITY;
        let mut max_aoa: f64 = 0.0;
        let mut max_rate: f64 = 0.0;
        for frame in 0..45_000 {
            flight
                .advance(&ephemeris, ControlMode::Navball, 0.02)
                .unwrap();
            // Same 0.02 seconds split into two irregular render frames.
            other
                .advance(&ephemeris, ControlMode::Navball, 0.007)
                .unwrap();
            other
                .advance(&ephemeris, ControlMode::Navball, 0.013)
                .unwrap();
            let body = ephemeris
                .body_state(flight.reference_body, SimTime(flight.flight_time_s))
                .unwrap();
            let k = local_air_kinematics(
                flight.atmosphere,
                flight.state,
                body,
                flight.planet_radius_m,
            )
            .unwrap();
            min_alt = min_alt.min(k.altitude_m);
            max_aoa = max_aoa.max(conventional_angle_of_attack_deg(k.air_velocity_body_mps).abs());
            max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
            assert!(
                k.altitude_m > 100.0,
                "surface departure failed at frame {frame}: {}",
                k.altitude_m
            );
            assert!(
                max_aoa < 22.0,
                "uncommanded stall at frame {frame}: {max_aoa}"
            );
            assert!(max_rate < 0.35, "uncommanded spin: {max_rate}");
            assert!(
                (flight.state.position_inertial_m - other.state.position_inertial_m).length()
                    < 1.0e-6
            );
            assert!(
                (flight.state.angular_velocity_body_rps - other.state.angular_velocity_body_rps)
                    .length()
                    < 1.0e-10
            );
        }
        println!(
            "900 s production path: min altitude {min_alt:.3} m, max AoA {max_aoa:.3} deg, max rate {max_rate:.6} rad/s; two runs {:?}",
            start.elapsed()
        );
    }

    #[test]
    fn wasd_moves_the_rendered_nose_in_the_expected_direction() {
        for (key, right_component, up_component) in [
            (KeyCode::KeyW, 0.0, -1.0),
            (KeyCode::KeyS, 0.0, 1.0),
            (KeyCode::KeyA, -1.0, 0.0),
            (KeyCode::KeyD, 1.0, 0.0),
        ] {
            let (ephemeris, mut flight) = fixture();
            let (_, mut baseline) = fixture();
            let initial = flight.state.orientation_body_to_inertial;
            let forward = pilot_render_offset(initial * DVec3::X);
            let up = pilot_render_offset(initial * DVec3::Z);
            let screen_right = forward.cross(up).normalize();
            let expected = screen_right * right_component + up * up_component;
            let mut keys = ButtonInput::default();
            keys.press(key);
            flight.control_input = keyboard_control_input(&keys);
            flight
                .advance(&ephemeris, ControlMode::Navball, 1.0)
                .unwrap();
            baseline
                .advance(&ephemeris, ControlMode::Navball, 1.0)
                .unwrap();
            // Follow the real GLB nose (-X) through both render transforms.
            let nose = flight.render_orientation * x15_asset_to_craft_rotation() * Vec3::NEG_X;
            let neutral = baseline.render_orientation * x15_asset_to_craft_rotation() * Vec3::NEG_X;
            let response = (nose - neutral).dot(expected);
            assert!(
                response > 0.025,
                "{key:?} moved the model the wrong way: {response}"
            );
        }
    }

    #[test]
    fn guidance_and_direct_modes_remain_finite_through_control_changes() {
        let (ephemeris, mut flight) = fixture();
        for mode in [
            ControlMode::Navball,
            ControlMode::MouseAim,
            ControlMode::Rate,
            ControlMode::Direct,
        ] {
            for frame in 0..1500 {
                flight.control_input = if (200..250).contains(&frame) {
                    DVec3::new(0.4, 0.15, 0.2)
                } else {
                    DVec3::ZERO
                };
                flight.advance(&ephemeris, mode, 0.02).unwrap();
                assert!(flight.state.velocity_inertial_mps.is_finite());
            }
        }
    }
}
