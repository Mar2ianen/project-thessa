//! Flight assist requests moments, but only surfaces and finite RCS force
//! couples deliver them. No controller torque is added directly to the body.
use super::*;
use bevy::math::DMat3;
use thessa_sim_core::{
    AeroModel, AeroState, COAST_RAILS_EXTEND_CHUNK, COAST_RAILS_MAX_STEPS,
    COAST_RAILS_MIN_AHEAD_S, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_STEP_S,
    COAST_RAILS_VELOCITY_TOL_MPS, TestParticleState, VerletConfig, evaluate_flight_forces,
    integrate_attitude_step,
};

pub(super) const FLIGHT_STEP_S: f64 = 1.0 / 120.0;
const SURFACE_COMMAND_RATE_S: f64 = 2.4; // 60 deg/s for the 25-degree elevator

/// Trim-conditioning threshold, not a force cutoff. At low density the
/// surface-response matrix becomes ill-conditioned; RCS handles attitude.
/// The flight solver still evaluates residual aero loads at every nonzero
/// density (q grows with speed squared even above this threshold).
pub(super) const COAST_DENSITY_KG_M3: f64 = 1.0e-7;

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
    #[cfg(test)]
    pub(super) fn advance(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
    ) -> Result<(), FlightError> {
        self.advance_with_budget(ephemeris, mode, elapsed_s, None)
    }

    /// The client may slow requested warp when CPU-bound. Never skip a physics
    /// step or change solver dt to catch up; only advance the clock by work done.
    /// Wall time determines batching only, never a force or physical state.
    pub(super) fn advance_with_budget(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), FlightError> {
        let started = std::time::Instant::now();
        self.steps_this_frame = 0;
        self.rails_advanced_this_frame = 0.0;
        self.accumulator_s += elapsed_s;
        let gravity_field = GravityField::from_ephemeris(ephemeris);
        while self.accumulator_s + 1.0e-12 >= FLIGHT_STEP_S {
            let coast = self.try_advance_cached_coast(ephemeris, mode, self.accumulator_s)?;
            if coast > 0.0 {
                self.accumulator_s = (self.accumulator_s - coast).max(0.0);
                self.rails_advanced_this_frame += coast;
                // Wake at the event boundary, allowing the owner to react
                // before any further physical work in this frame.
                if self.scheduler.next().is_some_and(|event| event.time.0 <= self.flight_time_s + FLIGHT_STEP_S) {
                    break;
                }
                continue;
            }
            self.step(ephemeris, &gravity_field, mode)?;
            self.steps_this_frame += 1;
            self.accumulator_s = (self.accumulator_s - FLIGHT_STEP_S).max(0.0);
            if budget.is_some_and(|limit| started.elapsed() >= limit) {
                // Excess requested warp is unserved wall-time demand, not
                // elapsed simulation time. Keep only the fractional tick;
                // don't build a catch-up queue that delays later inputs.
                let whole_ticks = ((self.accumulator_s + 1.0e-12) / FLIGHT_STEP_S).floor();
                self.accumulator_s = (self.accumulator_s - whole_ticks * FLIGHT_STEP_S).max(0.0);
                break;
            }
        }
        // Event-driven wakes: due scheduler events fire here instead of being
        // polled every physics tick.
        for event in self.scheduler.drain_due(SimTime(self.flight_time_s)) {
            self.wake_notice = Some(match event.kind {
                ScheduledKind::RailsImpact { .. } => {
                    format!("WAKE IMPACT T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::RailsHorizon => {
                    format!("WAKE HORIZON T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::ManeuverNode { .. } => {
                    format!("WAKE NODE T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::Alarm => {
                    format!("WAKE ALARM T+{:.0}s", event.time.seconds())
                }
            });
        }
        Ok(())
    }

    fn validate_endpoint(&mut self, ephemeris: &BakedEphemeris, next: &mut RigidBodyState,
        next_body: BodyState, time: SimTime) -> Result<(), FlightError> {
        // Solver guard rails read in the display (dominant-pull) frame, not
        // the launch frame: a Nereid escape at 33 km/s is routine flight,
        // while the same speed against the launch body would be nonsense.
        // Statically bounding against the launch world stopped every real
        // interlunar coast at the first handoff.
        let guard_body = ephemeris
            .dominant_body(next.position_inertial_m, time)
            .unwrap_or(self.reference_body);
        let guard_state = ephemeris
            .body_state(guard_body, time)
            .unwrap_or(next_body);
        let guard_radius = ephemeris
            .body(guard_body)
            .map(|body| body.radius_m)
            .unwrap_or(self.planet_radius_m);
        let relative = next.position_inertial_m - next_body.position_inertial;
        let guard_relative = next.position_inertial_m - guard_state.position_inertial;
        if (guard_relative.length() - guard_radius).abs() > MAX_PILOT_ALTITUDE_M
            || (next.velocity_inertial_mps - guard_state.velocity_inertial).length()
                > MAX_PILOT_RELATIVE_SPEED_MPS
            || next.angular_velocity_body_rps.length() > MAX_PILOT_ANGULAR_RATE_RPS
        {
            return Err(FlightError::InvalidInput(
                "flight state exceeded solver bounds".into(),
            ));
        }
        if let Some(field) = &self.terrain_field {
            let body_dir = DQuat::from_rotation_y(-(time.0 * std::f64::consts::TAU / (80.0 * 3600.0)))
                * DVec3::new(relative.x, relative.z, -relative.y).normalize();
            let surface =
                field.params.radius_m + field.height_m(body_dir.to_array(), 32.0).max(0.0);
            if relative.length() < surface + PILOT_SURFACE_CLEARANCE_M {
                return Err(FlightError::InvalidInput(
                    "terrain impact; contact dynamics are not implemented".into(),
                ));
            }
        }
        if guard_radius > 0.0 && guard_relative.length() <= guard_radius {
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Err(FlightError::InvalidInput(format!(
                "surface contact with {guard_body:?}"
            )));
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
            // Contact clamp rewrote the state off the baked path.
            self.rails.invalidate();
        }
        Ok(())
    }

    /// One read of a valid coast instead of replaying every translation tick.
    /// The curve hull and each body's maximum orbital speed certify that the
    /// entire interval stays outside geometry and in exactly sampled vacuum.
    fn try_advance_cached_coast(&mut self, ephemeris: &BakedEphemeris, mode: ControlMode,
        requested_s: f64) -> Result<f64, FlightError> {
        if self.thrust_n() != 0.0 || self.rails.is_empty() || self.trace.is_some() { return Ok(0.0); }
        let time = SimTime(self.flight_time_s);
        let mut duration = ((requested_s + 1.0e-12) / FLIGHT_STEP_S).floor() * FLIGHT_STEP_S;
        if let Some(event) = self.scheduler.next() {
            duration = duration.min(((event.time.0 - time.0) / FLIGHT_STEP_S).floor().max(0.0) * FLIGHT_STEP_S);
        }
        if let Some(path) = self.rails.path() {
            duration = duration.min(((path.end_time.0 - time.0) / FLIGHT_STEP_S).floor().max(0.0) * FLIGHT_STEP_S);
        }
        if duration < 2.0 * FLIGHT_STEP_S { return Ok(0.0); }
        let impact_bodies: Vec<_> = ephemeris.bodies.iter().filter(|b| b.radius_m > 0.0).map(|b| b.id).collect();
        if !self.rails.usable_for(ephemeris, TestParticleState { position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps }, time,
            VerletConfig { step_s: COAST_RAILS_STEP_S, max_steps: COAST_RAILS_MAX_STEPS },
            &impact_bodies, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_VELOCITY_TOL_MPS) { return Ok(0.0); }
        let end = time.offset(duration);
        let Some((min, max)) = self.rails.position_bounds(time, end) else { return Ok(0.0); };
        for body in ephemeris.bodies.iter().filter(|body| body.radius_m > 0.0 || body.id == self.reference_body) {
            let center = ephemeris.body_state(body.id, time).map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let speed = ephemeris.maximum_body_speed(body.id).map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let clearance = center.position_inertial.distance(center.position_inertial.clamp(min, max))
                - speed * duration - body.radius_m;
            let terrain_bound = if body.id == self.reference_body {
                self.terrain_field.as_ref().map_or(0.0, |field| field.params.height_max_m.max(0.0))
            } else { 0.0 };
            if clearance <= terrain_bound + PILOT_SURFACE_CLEARANCE_M { return Ok(0.0); }
            if body.id == self.reference_body && self.atmosphere.sample(clearance)?.density_kg_m3 != 0.0 {
                return Ok(0.0);
            }
        }
        let Some(orientation) = thessa_sim_core::constant_spin_orientation(self.state.orientation_body_to_inertial,
            self.state.angular_velocity_body_rps, self.vehicle.mass_properties.inertia_body_kg_m2, duration)
            else { return Ok(0.0); };
        let home = ephemeris.body_state(self.reference_body, time).map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let kinematics = local_air_kinematics(self.atmosphere, self.state, home, self.planet_radius_m)?;
        self.regime = FlightRegime::Coast;
        if self.allocate_controls(kinematics, mode)? != DVec3::ZERO { return Ok(0.0); }
        let (position, velocity) = self.rails.sample_at(end).expect("bounded coast interval");
        let mut next = RigidBodyState::new(position, velocity, orientation, self.state.angular_velocity_body_rps)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let next_home = ephemeris.body_state(self.reference_body, end).map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.validate_endpoint(ephemeris, &mut next, next_home, end)?;
        // One telemetry sample per batch, never per skipped translation tick.
        let gravity = GravityField::from_ephemeris(ephemeris).acceleration(position, end)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(evaluate_flight_forces(&self.aero_model, &self.vehicle.aero_geometry,
            self.atmosphere, next, self.vehicle.mass_properties, FlightStepInput {
                altitude_m: (position - next_home.position_inertial).length() - self.planet_radius_m,
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: orientation.inverse() * (position - next_home.position_inertial),
                wind_velocity_body_mps: orientation.inverse() * next_home.velocity_inertial,
                extra_force_body_n: DVec3::ZERO, extra_moment_body_nm: DVec3::ZERO, skip_aero: true,
            })?);
        self.state = next;
        self.flight_time_s = end.0;
        self.render_relative_position_m = position - next_home.position_inertial;
        self.render_orientation = render_orientation(orientation);
        Ok(duration)
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
        // Vacuum fast path: no air load exists, so the trim solve and the
        // response evaluation are skipped outright (exactly zero aero
        // moment); attitude flies on RCS alone.
        if self.regime == FlightRegime::Coast {
            let jets = if assisted {
                rcs_moment(requested, self.rcs_enabled)
            } else {
                let couples = rcs_couples();
                rcs_moment(
                    DVec3::new(couples[0].x, couples[1].y, couples[2].z) * axes,
                    self.rcs_enabled,
                )
            };
            self.actuator_saturated = assisted && (requested - jets).length() > 1_000.0;
            return Ok(jets);
        }
        let environment = self
            .atmosphere
            .aero_environment(kinematics.altitude_m.max(0.0), DVec3::ZERO)?;
        let aero_state = AeroState::new(kinematics.air_velocity_body_mps, omega);
        let mut command = if assisted {
            self.surface_input
        } else {
            self.control_input
        };
        // In coast the surfaces stay where the trim left them: with no air
        // load there is nothing to trim against, and the Newton solve below
        // would invert aerodynamic noise. Direct mode is already Newton-free
        // raw passthrough, so only the assisted solve is gated.
        let solve_trim = assisted && self.regime == FlightRegime::Aero;
        if solve_trim {
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

    /// Full rigid-body step for powered/aero flight (translation integrated).
    fn integrate_powered_step(
        &mut self,
        gravity: DVec3,
        kinematics: LocalAirKinematics,
        body_state: BodyState,
        jet_moment: DVec3,
        thrust_n: f64,
        skip_aero: bool,
    ) -> Result<(RigidBodyState, FlightForces), FlightError> {
        thessa_sim_core::integrate_rigid_body_step(
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
                extra_force_body_n: DVec3::X * thrust_n,
                extra_moment_body_nm: jet_moment,
                skip_aero,
            },
            FLIGHT_STEP_S,
        )
    }

    /// Arm the baked path's wake condition in the simulation-time scheduler:
    /// one path owns exactly one wake (impact epoch or horizon end), so the
    /// flight loop and autopilot wait on events instead of polling.
    fn arm_rails_wake(&mut self) {
        let Some(wake) = self.rails.wake() else {
            return;
        };
        match wake {
            thessa_sim_core::OnRailsWake::Impact { time, body } => {
                self.scheduler
                    .arm_rails_wake(ScheduledKind::RailsImpact { body }, time);
            }
            thessa_sim_core::OnRailsWake::HorizonEnd { time } => {
                self.scheduler
                    .arm_rails_wake(ScheduledKind::RailsHorizon, time);
            }
        }
    }

    /// Unpowered vacuum coast on the shared baked trajectory. Translation is
    /// sampled (cubic Hermite) from the rails path the map prediction draws;
    /// attitude keeps integrating under the RCS moment. Returns `None` when
    /// no rails path can serve this step (bake failure, horizon exhausted
    /// twice in a row) so the caller falls back to a normal integrated step.
    /// The rails config is the shared [`COAST_RAILS_STEP_S`] sizing, so the
    /// prediction reuses this exact bake instead of integrating its own.
    fn try_coast_step_on_rails(
        &mut self,
        ephemeris: &BakedEphemeris,
        time: SimTime,
        jet_moment: DVec3,
        body_state: BodyState,
        gravity: DVec3,
    ) -> Result<Option<(RigidBodyState, FlightForces)>, FlightError> {
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let config = VerletConfig {
            step_s: COAST_RAILS_STEP_S,
            max_steps: COAST_RAILS_MAX_STEPS,
        };
        let impact_bodies: Vec<BodyId> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        if let Some(task) = self.rails_job.as_mut()
            && let Some(result) = bevy::tasks::block_on(bevy::tasks::poll_once(task))
        {
            self.rails_job = None;
            if let Ok((rails, seconds)) = result {
                self.rails = rails;
                self.rails_bake_seconds = Some(seconds);
                self.arm_rails_wake();
            }
        }
        if !self.rails.usable_for(
            ephemeris,
            initial,
            time,
            config,
            &impact_bodies,
            COAST_RAILS_POSITION_TOL_M,
            COAST_RAILS_VELOCITY_TOL_MPS,
        ) {
            self.scheduler.clear_rails_wakes();
            if let Some(pool) = bevy::tasks::AsyncComputeTaskPool::try_get() {
                if self.rails_job.is_none() {
                    let ephemeris = ephemeris.clone();
                    let bodies = impact_bodies.clone();
                    self.rails_job = Some(pool.spawn(async move {
                        let started = std::time::Instant::now();
                        let mut rails = thessa_sim_core::OnRailsCache::new();
                        rails
                            .bake(&ephemeris, initial, time, config, &bodies)
                            .map_err(|error| error.to_string())?;
                        Ok((rails, started.elapsed().as_secs_f64()))
                    }));
                }
                // Continue the ordinary physical step while the worker bakes.
                // The finished path must pass the state/version check above.
                return Ok(None);
            }
            // Headless solver tests can run without a Bevy task pool.
            // Head bake only (~2.8 h, milliseconds): the step below samples
            // 1/120 s ahead, and coverage grows via extension underneath.
            if self
                .rails
                .bake_head(ephemeris, initial, time, config, &impact_bodies)
                .is_err()
            {
                return Ok(None);
            }
            self.arm_rails_wake();
        }
        // Grow coverage toward the full horizon one small chunk per step.
        // Each chunk is milliseconds, so the frame budget holds steady while
        // the 180 ms full bake never blocks the loop.
        if self.rails.needs_extension(time, COAST_RAILS_MIN_AHEAD_S) {
            match self.rails.extend(ephemeris, COAST_RAILS_EXTEND_CHUNK) {
                Ok(true) => self.arm_rails_wake(),
                Ok(false) => {}
                Err(_) => {
                    self.rails.invalidate();
                    self.scheduler.clear_rails_wakes();
                    return Ok(None);
                }
            }
        }
        let next_time = time.offset(FLIGHT_STEP_S);
        if let Some(thessa_sim_core::OnRailsWake::Impact {
            time: impact_time,
            body,
        }) = self.rails.wake()
            && impact_time.0 <= next_time.0
        {
            self.wake_notice = Some(format!("WAKE IMPACT {body:?} T+{:.3}s", impact_time.0));
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Err(FlightError::InvalidInput(format!(
                "coast reached surface contact with {body:?}"
            )));
        }
        let sampled = self.rails.sample_at(next_time);
        if sampled.is_none() {
            // Never rebake across an impact or block on horizon extension.
            // Ordinary physics handles the next step; a future request can
            // build a new horizon on the worker.
            if let Some(wake) = self.rails.wake() {
                self.wake_notice = Some(format!("WAKE {wake:?}"));
            }
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Ok(None);
        }
        let Some((position, velocity)) = sampled else {
            // Baking while already inside a body yields a single-sample path
            // with no forward coverage: let the normal step (and its contact
            // handling) deal with it.
            return Ok(None);
        };
        let (orientation, omega) = integrate_attitude_step(
            self.state.orientation_body_to_inertial,
            self.state.angular_velocity_body_rps,
            self.vehicle.mass_properties.inertia_body_kg_m2,
            jet_moment,
            FLIGHT_STEP_S,
        )?;
        let next =
            RigidBodyState::new(position, velocity, orientation, omega).map_err(|error| {
                self.rails.invalidate();
                FlightError::InvalidInput(error.to_string())
            })?;
        // Zero-load bookkeeping for the trace: aero is skipped, thrust is
        // zero, so this only evaluates the (empty) panel response.
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        let forces = match evaluate_flight_forces(
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
                extra_force_body_n: DVec3::ZERO,
                extra_moment_body_nm: jet_moment,
                skip_aero: true,
            },
        ) {
            Ok(forces) => forces,
            Err(error) => {
                self.rails.invalidate();
                return Err(error);
            }
        };
        Ok(Some((next, forces)))
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
        // Regime follows sampled density, never altitude: an invalid sample
        // keeps Aero so the existing atmosphere error paths still fire.
        let density_kg_m3 = self
            .atmosphere
            .sample(kinematics.altitude_m.max(0.0))
            .map(|sample| sample.density_kg_m3)
            .unwrap_or(f64::INFINITY);
        self.regime = if density_kg_m3 < COAST_DENSITY_KG_M3 {
            FlightRegime::Coast
        } else {
            FlightRegime::Aero
        };
        let gravity = gravity_field
            .acceleration(self.state.position_inertial_m, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let jet_moment = self.allocate_controls(kinematics, mode)?;
        // Only an exactly empty sampled medium permits zero force.
        // Low density alone does not bound drag at high speed;
        // Coast freezes trim but retains residual panel forces.
        let skip_aero = density_kg_m3 == 0.0;
        let thrust_n = self.thrust_n();
        // Unpowered vacuum coast rides the single baked trajectory instead
        // of integrating translation per tick; the map prediction draws the
        // same path. Attitude (RCS) still integrates at full rate below.
        let (mut next, forces) = if self.regime == FlightRegime::Coast
            && thrust_n == 0.0
            && skip_aero
        {
            match self.try_coast_step_on_rails(ephemeris, time, jet_moment, body_state, gravity)? {
                Some(coasted) => coasted,
                None => {
                    self.rails.invalidate();
                    self.integrate_powered_step(
                        gravity, kinematics, body_state, jet_moment, thrust_n, skip_aero,
                    )?
                }
            }
        } else {
            self.rails.invalidate();
            self.rails_job = None;
            self.scheduler.clear_rails_wakes();
            self.integrate_powered_step(
                gravity, kinematics, body_state, jet_moment, thrust_n, skip_aero,
            )?
        };
        let next_body = ephemeris
            .body_state(
                self.reference_body,
                SimTime(self.flight_time_s + FLIGHT_STEP_S),
            )
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.validate_endpoint(ephemeris, &mut next, next_body, SimTime(self.flight_time_s + FLIGHT_STEP_S))?;
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
    fn solver_guard_reads_dominant_frame_not_launch_frame() {
        // Live failure: a Nereid escape at 33 km/s Nereid-relative latched
        // FLIGHT STOPPED because the guard measured 50+ km/s against the
        // LAUNCH body. Same state must pass with the guard in the dominant
        // frame. v_mag sits strictly between the bound and bound+R so the
        // old launch-frame check would trip while the dominant check clears.
        let (ephemeris, mut flight) = fixture();
        let thessa = ephemeris.body_id("thessa").unwrap();
        let nereid = ephemeris.body_id("nereid").unwrap();
        let time = SimTime::EPOCH;
        let thessa_state = ephemeris.body_state(thessa, time).unwrap();
        let nereid_state = ephemeris.body_state(nereid, time).unwrap();
        let giant = ephemeris.body(nereid).unwrap();
        let frame_gap = (thessa_state.velocity_inertial - nereid_state.velocity_inertial).length();
        assert!(frame_gap > 1000.0, "frames must differ, gap {frame_gap}");
        let v_mag = MAX_PILOT_RELATIVE_SPEED_MPS + frame_gap / 2.0;
        let away =
            (thessa_state.velocity_inertial - nereid_state.velocity_inertial).normalize_or_zero();
        flight.state.position_inertial_m =
            nereid_state.position_inertial + DVec3::Z * (giant.radius_m + 100_000.0);
        flight.state.velocity_inertial_mps = thessa_state.velocity_inertial - away * v_mag;
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        flight.flight_time_s = 0.0;
        assert_eq!(
            ephemeris.dominant_body(flight.state.position_inertial_m, time),
            Some(nereid)
        );
        let launch_relative =
            (flight.state.velocity_inertial_mps - thessa_state.velocity_inertial).length();
        assert!(
            launch_relative > MAX_PILOT_RELATIVE_SPEED_MPS,
            "setup must trip the old guard, got {launch_relative}"
        );
        flight
            .step(
                &ephemeris,
                &GravityField::from_ephemeris(&ephemeris),
                ControlMode::Direct,
            )
            .expect("dominant-frame guard must clear interlunar coast");
    }

    #[test]
    fn warp_budget_keeps_exact_steps_and_does_not_queue_unserved_warp() {
        let (ephemeris, mut limited) = fixture();
        let (_, mut exact) = fixture();
        limited
            .advance_with_budget(
                &ephemeris,
                ControlMode::Navball,
                3.2,
                Some(std::time::Duration::ZERO),
            )
            .unwrap();
        assert_eq!(limited.steps_this_frame, 1);
        assert!(limited.accumulator_s < FLIGHT_STEP_S);
        exact
            .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
            .unwrap();
        assert_eq!(limited.state, exact.state);
        assert_eq!(limited.flight_time_s, exact.flight_time_s);
        limited
            .advance(&ephemeris, ControlMode::Navball, 0.0)
            .unwrap();
        assert_eq!(limited.steps_this_frame, 0);
    }

    #[test]
    #[ignore = "wall-clock diagnostic; run with --ignored --nocapture"]
    fn profile_warp_frame_budget() {
        let (ephemeris, mut full) = fixture();
        let (_, mut bounded) = fixture();
        let now = std::time::Instant::now();
        full.advance(&ephemeris, ControlMode::Navball, 3.2).unwrap();
        eprintln!(
            "warp unbounded: {:?}, {} steps",
            now.elapsed(),
            full.steps_this_frame
        );
        let now = std::time::Instant::now();
        bounded
            .advance_with_budget(
                &ephemeris,
                ControlMode::Navball,
                3.2,
                Some(std::time::Duration::from_millis(8)),
            )
            .unwrap();
        eprintln!(
            "warp bounded: {:?}, {} steps",
            now.elapsed(),
            bounded.steps_this_frame
        );
        assert!(bounded.steps_this_frame < full.steps_this_frame);
    }

    #[test]
    fn terrain_contact_stops_before_committing_an_underground_pose() {
        let (ephemeris, mut flight) = fixture();
        let recipe = toml::from_str(include_str!(
            "../../../../data/worldgen/worldgen_recipe.toml"
        ))
        .unwrap();
        let field = std::sync::Arc::new(
            thessa_worldgen_rocky::field::field_from_manifest(
                &thessa_worldgen_rocky::spec_recipe::manifest_from_spec(&recipe).unwrap(),
            )
            .unwrap(),
        );
        let dir = [1.0, 0.0, 0.0];
        flight.initialize_world_site(field.clone(), dir, &ephemeris);
        let body = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        let r = flight.planet_radius_m + field.height_m(dir, 32.0).max(0.0);
        flight.state.position_inertial_m = body.position_inertial + DVec3::X * (r + 0.1);
        flight.state.velocity_inertial_mps = body.velocity_inertial - DVec3::X * 100.0;
        let before = flight.state;
        let error = flight
            .step(
                &ephemeris,
                &GravityField::from_ephemeris(&ephemeris),
                ControlMode::Direct,
            )
            .unwrap_err();
        assert!(error.to_string().contains("terrain impact"), "{error}");
        assert_eq!(flight.state.position_inertial_m, before.position_inertial_m);
        assert_eq!(flight.flight_time_s, 0.0);
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
    #[test]
    fn bank_rotates_lift_out_of_the_vertical_plane() {
        let (_, flight) = fixture();
        let env = flight
            .atmosphere
            .aero_environment(500.0, DVec3::ZERO)
            .unwrap();
        let mut vertical = vec![];
        for bank in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0] {
            let attitude = DQuat::from_rotation_x(bank.to_radians())
                * DQuat::from_rotation_y(-8.0_f64.to_radians());
            let force = flight
                .aero_model
                .evaluate_state(
                    AeroState::new(attitude.inverse() * DVec3::X * 180.0, DVec3::ZERO),
                    env,
                    &flight.vehicle.aero_geometry,
                )
                .unwrap()
                .force_body_n;
            let inertial = attitude * force;
            assert!(inertial.is_finite());
            vertical.push(inertial.z);
            eprintln!(
                "bank {bank:5.0} deg: vertical aero force {:10.3} N; side {:10.3} N",
                inertial.z, inertial.y
            );
        }
        assert!(vertical[0] > 100.0);
        for (i, angle) in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0]
            .into_iter()
            .enumerate()
        {
            assert!((vertical[i] / vertical[0] - angle.to_radians().cos()).abs() < 1.0e-10);
        }
    }

    fn circular_orbit_fixture(altitude_m: f64) -> (BakedEphemeris, PilotFlightRuntime) {
        let (ephemeris, mut flight) = fixture();
        let body = ephemeris.body(flight.reference_body).unwrap();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        let radius = body.radius_m + altitude_m;
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
        flight.state.velocity_inertial_mps =
            origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        (ephemeris, flight)
    }

    #[test]
    fn coast_threshold_brackets_vacuum() {
        let (_, flight) = fixture();
        let sea_level = flight.atmosphere.sample(0.0).unwrap().density_kg_m3;
        assert!(
            sea_level > COAST_DENSITY_KG_M3,
            "launch pad must be Aero, got {sea_level}"
        );
        let high = flight.atmosphere.sample(300_000.0).unwrap().density_kg_m3;
        assert!(
            high < COAST_DENSITY_KG_M3,
            "300 km must be Coast, got {high}"
        );
        assert_eq!(FlightRegime::Aero.label(), "AERO");
        assert_eq!(FlightRegime::Coast.label(), "COAST");
    }

    #[test]
    fn coast_freezes_trim_and_keeps_rcs_authority() {
        let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
        flight
            .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
            .unwrap();
        assert_eq!(flight.regime(), FlightRegime::Coast);
        // Assisted trim solve is frozen in vacuum: surfaces hold, no stall
        // chase from a near-singular effectiveness matrix.
        assert_eq!(flight.surface_input, DVec3::ZERO);
        // Full yaw stick still turns the craft on RCS alone.
        flight.control_input = DVec3::Y;
        flight
            .advance(&ephemeris, ControlMode::Navball, 2.0)
            .unwrap();
        assert_eq!(flight.regime(), FlightRegime::Coast);
        assert_eq!(flight.surface_input, DVec3::ZERO);
        assert!(
            flight.state.angular_velocity_body_rps.length() > 0.05,
            "RCS must answer in coast, got {:?}",
            flight.state.angular_velocity_body_rps
        );
    }

    #[test]
    fn coast_orbit_stays_bounded_without_stops() {
        let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
        for _ in 0..30_000 {
            // Every step must succeed: leaving the atmosphere never latches
            // FLIGHT STOPPED, it switches the solver to Coast instead.
            flight
                .advance(&ephemeris, ControlMode::Navball, 0.02)
                .unwrap();
            assert_eq!(flight.regime(), FlightRegime::Coast);
        }
        assert!((flight.flight_time_s - 600.0).abs() < 1.0);
        let body = ephemeris.body(flight.reference_body).unwrap();
        let state = ephemeris
            .body_state(flight.reference_body, SimTime(flight.flight_time_s))
            .unwrap();
        let altitude =
            (flight.state.position_inertial_m - state.position_inertial).length() - body.radius_m;
        assert!(
            (280_000.0..320_000.0).contains(&altitude),
            "coast must hold the orbit, altitude drifted to {altitude}"
        );
    }

    #[test]
    fn unpowered_vacuum_coast_rides_the_shared_rails() {
        // One trajectory, two consumers: in an unpowered vacuum coast the
        // flight loop must sample translation from the baked rails (the same
        // bake the map prediction draws) instead of integrating per tick.
        // Deep space gives exactly-zero sampled density, which is the same
        // condition that permits the aero fast path.
        let (ephemeris, mut flight) = fixture();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
        flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        flight
            .advance(&ephemeris, ControlMode::Direct, 0.05)
            .expect("coast advances");
        assert!(
            !flight.rails.is_empty(),
            "vacuum coast must bake the shared rails"
        );
        // The bake arms its wake in simulation time: the loop waits on the
        // horizon event instead of polling the trajectory.
        let wake = flight.scheduler.next().expect("coast arms a wake");
        assert!(
            matches!(wake.kind, thessa_sim_core::ScheduledKind::RailsHorizon),
            "deep-space coast wake must be the horizon, got {:?}",
            wake.kind
        );
        assert!(
            wake.time.seconds() > flight.flight_time_s,
            "wake must lie ahead"
        );
        // Translation now tracks the rails bake to interpolation precision.
        let (position, _) = flight.inertial_state_m();
        let (sampled, _) = flight
            .rails
            .sample_at(SimTime(flight.flight_time_s))
            .expect("rails cover the flown epoch");
        assert!(
            (position - sampled).length() < 1.0,
            "flown state left the rails: {:?} m",
            (position - sampled).length()
        );
        // Lighting the engine invalidates the bake: thrust is a maneuver.
        flight.engine_active = true;
        flight.throttle = 1.0;
        flight
            .advance(&ephemeris, ControlMode::Direct, 0.05)
            .expect("powered flight advances");
        assert!(
            flight.rails.is_empty(),
            "thrust must invalidate the coast rails"
        );
    }
}
