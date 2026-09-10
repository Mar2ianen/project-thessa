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
        let desired_rate = if attitude_hold && axes.length_squared() > 1.0e-8 {
            // Manual input overrides attitude hold. Capture the achieved
            // attitude, so a long turn cannot wind an unreachable target past
            // 180 degrees and make shortest-path SAS reverse the manoeuvre.
            self.sas_target_orientation = self.state.orientation_body_to_inertial;
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S
        } else if attitude_hold {
            let mut error =
                self.state.orientation_body_to_inertial.inverse() * self.sas_target_orientation;
            // q and -q represent the same attitude; use the short rotation.
            if error.w < 0.0 {
                error = -error;
            }
            let angle = error.to_scaled_axis();
            let couples = rcs_couples();
            let inertia = self.vehicle.mass_properties.inertia_body_kg_m2;
            let acceleration = DVec3::new(
                couples[0].x / inertia.x_axis.x,
                couples[1].y / inertia.y_axis.y,
                couples[2].z / inertia.z_axis.z,
            );
            // Brake early enough for the finite jets. A fixed high-gain
            // attitude loop saturates in vacuum and keeps overshooting.
            let braking_rate = (acceleration * angle.abs() * 0.5).sqrt();
            (angle * 1.6)
                .clamp(-braking_rate, braking_rate)
                .clamp_length_max(0.35)
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
        self.actuator_saturated = assisted && (requested - actual_aero - jets).length() > 1_000.0;
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
        self.render_relative_position_m = next.position_inertial_m - next_body.position_inertial;
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
    fn recorded_high_rate_flight_continues_past_the_previous_stop() {
        // Live capture, 514.808--581.375 s: restore its first physical state,
        // then replay the pilot inputs (not the recorded resulting forces).
        let capture = include_str!("../../../../logs/flight-traces/2026-09-09-high-rate-stop.csv");
        let mut lines = capture.lines();
        let columns: Vec<_> = lines.next().unwrap().split(',').collect();
        let column = |name: &str| columns.iter().position(|c| *c == name).unwrap();
        let first: Vec<_> = lines.next().unwrap().split(',').collect();
        let number = |row: &[&str], name: &str| row[column(name)].parse::<f64>().unwrap();
        let vector = |row: &[&str], names: [&str; 3]| {
            DVec3::new(
                number(row, names[0]),
                number(row, names[1]),
                number(row, names[2]),
            )
        };
        let (ephemeris, mut flight) = fixture();
        flight.flight_time_s = (number(&first, "t_s") * 120.0).round() / 120.0;
        flight.state = RigidBodyState::new(
            vector(&first, ["pos_x_m", "pos_y_m", "pos_z_m"]),
            vector(&first, ["vel_x_mps", "vel_y_mps", "vel_z_mps"]),
            DQuat::from_xyzw(
                number(&first, "quat_x"),
                number(&first, "quat_y"),
                number(&first, "quat_z"),
                number(&first, "quat_w"),
            )
            .normalize(),
            vector(&first, ["omega_x_rps", "omega_y_rps", "omega_z_rps"]),
        )
        .unwrap();
        flight.surface_input = vector(&first, ["surface_pitch", "surface_yaw", "surface_roll"]);
        let mut max_rate: f64 = 0.0;
        for line in lines {
            let row: Vec<_> = line.split(',').collect();
            assert_eq!(row[column("control_mode")], "DIRECT / RAW");
            flight.control_input = vector(&row, ["pitch_cmd", "yaw_cmd", "roll_cmd"]);
            flight.throttle = number(&row, "throttle");
            flight.engine_active = row[column("engine")] == "1";
            flight.sas_enabled = row[column("sas")] == "1";
            flight.rcs_enabled = row[column("rcs")] == "1";
            flight
                .advance(&ephemeris, ControlMode::Direct, FLIGHT_STEP_S)
                .unwrap();
            max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
        }
        flight.control_input = DVec3::ZERO;
        flight
            .advance(&ephemeris, ControlMode::Direct, 60.0)
            .unwrap();
        println!(
            "recorded spin replay: t={:.3} s, max rate={max_rate:.6}, final rate={:.6}",
            flight.flight_time_s,
            flight.state.angular_velocity_body_rps.length()
        );
        assert!(flight.flight_time_s > 640.0);
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
    fn sas_allows_full_orbital_turn_and_holds_after_release() {
        for command in [DVec3::Y, DVec3::X] {
            let (ephemeris, mut flight) = fixture();
            let body = ephemeris.body(flight.reference_body).unwrap();
            let origin = ephemeris
                .body_state(flight.reference_body, SimTime::EPOCH)
                .unwrap();
            let radius = body.radius_m + 300_000.0;
            flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
            flight.state.velocity_inertial_mps =
                origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
            flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
            flight.sas_target_orientation = DQuat::IDENTITY;
            flight.engine_active = false;
            flight.control_input = command;
            let axis = if command == DVec3::Y {
                DVec3::NEG_Z
            } else {
                DVec3::NEG_Y
            };
            let mut accumulated_angle = 0.0;
            for _ in 0..5400 {
                let previous = flight.state.orientation_body_to_inertial;
                flight
                    .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
                    .unwrap();
                let delta = (previous.inverse() * flight.state.orientation_body_to_inertial)
                    .to_scaled_axis();
                assert!(
                    delta.dot(axis) >= -1.0e-8,
                    "SAS reversed a commanded orbital turn"
                );
                accumulated_angle += delta.dot(axis);
            }
            assert!(
                accumulated_angle.to_degrees() > 360.0,
                "turn stopped at {} deg",
                accumulated_angle.to_degrees()
            );
            flight.control_input = DVec3::ZERO;
            let released = flight.state.orientation_body_to_inertial;
            flight
                .advance(&ephemeris, ControlMode::Navball, 20.0)
                .unwrap();
            let hold_error = flight
                .state
                .orientation_body_to_inertial
                .angle_between(released)
                .to_degrees();
            println!(
                "orbital turn: {:.2} deg, hold error {hold_error:.6} deg, rate {:?}",
                accumulated_angle.to_degrees(),
                flight.state.angular_velocity_body_rps
            );
            assert!(hold_error < 1.0, "SAS hold error {hold_error}");
            assert!(flight.state.angular_velocity_body_rps.length() < 0.001);
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
