//! Coast eligibility, terrain guards, and asynchronous rails cache service.

use super::*;

impl FlightAuthority {
    pub(super) fn validate_endpoint(
        &mut self,
        ephemeris: &BakedEphemeris,
        next: &mut RigidBodyState,
        next_body: BodyState,
        time: SimTime,
    ) -> Result<(), FlightError> {
        // Batch path: one validation per rails jump, so naive guard lookups
        // are amortized. The per-tick `step` below serves the same reads
        // from its memoized frame and calls `validate_endpoint_with_guard`.
        let guard_body = ephemeris
            .dominant_body(next.position_inertial_m, time)
            .unwrap_or(self.reference_body);
        let guard_state = ephemeris.body_state(guard_body, time).unwrap_or(next_body);
        let guard_radius = ephemeris
            .body(guard_body)
            .map(|body| body.radius_m)
            .unwrap_or(self.planet_radius_m);
        self.validate_endpoint_with_guard(
            next,
            next_body,
            time,
            guard_body,
            guard_state,
            guard_radius,
        )
    }

    pub(super) fn validate_endpoint_with_guard(
        &mut self,
        next: &mut RigidBodyState,
        next_body: BodyState,
        time: SimTime,
        guard_body: BodyId,
        guard_state: BodyState,
        guard_radius: f64,
    ) -> Result<(), FlightError> {
        // Solver guard rails read in the display (dominant-pull) frame, not
        // the launch frame: a Nereid escape at 33 km/s is routine flight,
        // while the same speed against the launch body would be nonsense.
        // Statically bounding against the launch world stopped every real
        // interlunar coast at the first handoff.
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
            // Contact-active ticks resolve terrain through Rapier; the
            // stop-before-penetration error below only guards free flight.
            if !self.contact_active() {
                let body_dir = ground_dir_body_fixed(relative, time.0, self.body_rotation_period_s);
                let surface =
                    field.params.radius_m + field.height_m(body_dir.to_array(), 32.0).max(0.0);
                if relative.length() < surface + PILOT_SURFACE_CLEARANCE_M {
                    return Err(FlightError::InvalidInput(
                        "terrain impact; contact dynamics are not implemented".into(),
                    ));
                }
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
        // Skipped while contact-active: Rapier owns the contact response and
        // the clamp would fight the solver by rewriting its solved pose.
        if !self.contact_active()
            && relative.length() < self.planet_radius_m + PILOT_SURFACE_CLEARANCE_M
        {
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

    /// Declared obstacle ceiling for rails batch certification, in metres
    /// above the datum. Live field maximum when terrain is loaded, else the
    /// baked-in recipe maximum: the bake clamps every height into
    /// `[height_min_m, height_max_m]`, so the recipe bound is a sound upper
    /// bound with no field build required. A missing field used to certify
    /// against bare datum (0 m) — low batches could ghost through unmapped
    /// mountains. Unobserved craft declare this ceiling instead of visual
    /// tiles; physics stays identical, only mesh/texture synthesis is
    /// skipped.
    pub(super) fn rails_terrain_bound_m(&self) -> f64 {
        self.terrain_field
            .as_ref()
            .map(|field| field.params.height_max_m.max(0.0))
            .unwrap_or(self.recipe_max_elevation_m)
    }

    /// One read of a valid coast instead of replaying every translation tick.
    /// The curve hull and each body's maximum orbital speed certify that the
    /// entire interval stays outside geometry and in exactly sampled vacuum.
    /// Current-state usability probe shared by the batch path.
    pub(super) fn rails_usable_now(&self, ephemeris: &BakedEphemeris) -> bool {
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let time = SimTime(self.flight_time_s);
        let config = TickIntegratorConfig::default();
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        self.rails.usable_tick_for(
            ephemeris,
            initial,
            time,
            config,
            &impact_bodies,
            COAST_RAILS_POSITION_TOL_M,
            COAST_RAILS_VELOCITY_TOL_MPS,
        )
    }

    /// Shared adoption: integrity check (this bake, this universe) plus
    /// timeliness (current state still on it), then swap. A worker that
    /// finished after the sim outran it is rejected instead of regressing
    /// the cache; the loop re-requests from live state. Used by the async
    /// poll and the blocking catch-up alike.
    pub(super) fn adopt_rails_bake(&mut self, ephemeris: &BakedEphemeris, baked: BakedRails) {
        let rails = baked.rails;
        if let Some(path) = rails.path()
            && rails.usable_tick_for(
                ephemeris,
                TestParticleState {
                    position: path.positions[0],
                    velocity: path.velocities[0],
                },
                path.times[0],
                TickIntegratorConfig::default(),
                &ephemeris
                    .bodies
                    .iter()
                    .filter(|b| b.radius_m > 0.0)
                    .map(|b| b.id)
                    .collect::<Vec<_>>(),
                0.0,
                0.0,
            )
        {
            self.rails = rails;
            self.rails_bake_seconds = Some(baked.bake_seconds);
        }
    }

    pub(super) fn try_advance_cached_coast(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        requested_s: f64,
    ) -> Result<CoastAdvance, FlightError> {
        // A contact-active craft never rides baked translation: Rapier owns
        // its pose every tick, so a rails batch would integrate a second,
        // divergent trajectory through the same interval.
        if self.contact_active() {
            return Ok(CoastAdvance::NotEligible);
        }
        if self.landing_gear_transitioning() {
            return Ok(CoastAdvance::NotEligible);
        }
        if self.guidance_state_dependent {
            return Ok(CoastAdvance::NotEligible);
        }
        // A declarative moment is an active actuator demand. The rails path
        // only integrates free translation and constant-spin attitude, so a
        // non-zero explicit moment must stay on the fixed-step allocator path.
        if self
            .explicit_moment_demand_nm
            .is_some_and(|moment| moment.length_squared() > 1.0e-24)
        {
            return Ok(CoastAdvance::NotEligible);
        }
        if self
            .explicit_force_demand_body_n
            .is_some_and(|force| force.length_squared() > 1.0e-24)
        {
            return Ok(CoastAdvance::NotEligible);
        }
        if self.thrust_n() != 0.0
            || (self.engine_active
                && self.throttle > 1.0e-8
                && (!self.vehicle.engines.is_empty() || !self.vehicle.systems.is_empty()))
            || (self.propulsion_dynamics_active
                && (self.propulsion_target > 1.0e-8 || self.propulsion_actual > 1.0e-8))
            || self
                .engine_throttle_overrides
                .iter()
                .flatten()
                .any(|throttle| *throttle > 1.0e-8)
            || self
                .system_throttle_overrides
                .iter()
                .flatten()
                .flatten()
                .any(|throttle| *throttle > 1.0e-8)
            || self.trace.is_some()
        {
            return Ok(CoastAdvance::NotEligible);
        }
        // A controlled coast must stay on the fixed-step path so RCS/SAS can
        // update attitude. Check this before touching the bake queue: a cold
        // forecast must never delay responsive maneuver input.
        let attitude_hold =
            self.sas_enabled && matches!(mode, ControlMode::Navball | ControlMode::MouseAim);
        if (self.rcs_enabled || self.reaction_wheels_enabled)
            && (self.control_input != DVec3::ZERO
                || (mode != ControlMode::Direct
                    && self.state.angular_velocity_body_rps != DVec3::ZERO)
                || (attitude_hold
                    && self.sas_target_orientation != self.state.orientation_body_to_inertial
                    && self.sas_target_orientation != -self.state.orientation_body_to_inertial))
        {
            return Ok(CoastAdvance::NotEligible);
        }
        // The exact declared-vacuum boundary is the contract that makes
        // translation batching physically valid. Do not prepare or wait for
        // a forecast while the craft is still in sampled atmosphere.
        let body_state = ephemeris
            .body_state(self.reference_body, SimTime(self.flight_time_s))
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let altitude_m = (self.state.position_inertial_m - body_state.position_inertial).length()
            - self.planet_radius_m;
        let density_kg_m3 = self
            .atmosphere
            .sample(altitude_m.max(0.0))
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?
            .density_kg_m3;
        if density_kg_m3 != 0.0 {
            return Ok(CoastAdvance::NotEligible);
        }
        // Collect a worker bake that finished while earlier batches flew;
        // the batch loop otherwise never polls, and coverage would stall.
        self.poll_rails_bake(ephemeris);
        // A cold or stale cache is a cooperative wait, never a synchronous
        // bake and never a fallback replay of the whole requested warp.
        // Inline queues finish during poll (useful for deterministic tests);
        // worker queues return WaitingForBake until the next driver quantum.
        if !self.rails_usable_now(ephemeris) {
            if !self.bake.has_pending() {
                self.spawn_rails_bake(ephemeris);
                self.poll_rails_bake(ephemeris);
            }
            if !self.rails_usable_now(ephemeris) {
                return if self.bake.has_pending() {
                    Ok(CoastAdvance::WaitingForBake)
                } else {
                    Ok(CoastAdvance::NotEligible)
                };
            }
        }
        self.arm_rails_wake();
        let time = SimTime(self.flight_time_s);
        let mut duration = ((requested_s + 1.0e-12) / FLIGHT_STEP_S).floor() * FLIGHT_STEP_S;
        // Capped jumps: vacuum batches are drag-free by construction
        // (the cutoff declares exact vacuum), so this bounds wake/event
        // latency and forces periodic revalidation instead of certifying
        // dynamics — one horizon rides as capped jumps, never one leap.
        duration = duration.min(MAX_COAST_BATCH_JUMP_S);
        if let Some(event) = self.scheduler.next() {
            duration = duration
                .min(((event.time.0 - time.0) / FLIGHT_STEP_S).floor().max(0.0) * FLIGHT_STEP_S);
        }
        if let Some(path) = self.rails.path() {
            duration = duration.min(
                ((path.end_time.0 - time.0) / FLIGHT_STEP_S)
                    .floor()
                    .max(0.0)
                    * FLIGHT_STEP_S,
            );
        }
        if duration < 2.0 * FLIGHT_STEP_S {
            return Ok(CoastAdvance::NotEligible);
        }
        // Proactive JIT preparation for sustained warp: when baked coverage
        // ahead drops under a day, start a fresh full-horizon worker bake
        // now so high warp never stalls on a synchronous rebake at the
        // horizon end. One job at most; the swap path validates on arrival.
        if !self.bake.has_pending()
            && let Some(covered) = self.rails.covered_until()
            && covered.0 - time.0 < PROACTIVE_REBAKE_AHEAD_S
        {
            self.spawn_rails_bake(ephemeris);
        }
        let ticks = (duration / FLIGHT_STEP_S).round() as u64;
        let end = self.time_after_ticks(ticks)?;
        // Coverage guard only; separation is certified per-sample below.
        if self.rails.position_bounds(time, end).is_none() {
            return Ok(CoastAdvance::NotEligible);
        }
        // A live field can replace the global recipe ceiling with a track
        // report, but only when the five-point batch is close enough to
        // terrain for that distinction to matter. Above the known ceiling the
        // recipe bound is already a proof and avoids rebuilding obstacle grids
        // on every high-warp batch.
        let terrain_track = if self.terrain_field.is_some() {
            let mut track = Vec::with_capacity(5);
            let mut near_terrain = false;
            for point in 0..5 {
                let sample_time = SimTime(time.0 + (end.0 - time.0) * f64::from(point) / 4.0);
                let Some((position, _)) = self.rails.sample_at(sample_time) else {
                    return Ok(CoastAdvance::NotEligible);
                };
                let body_state = ephemeris
                    .body_state(self.reference_body, sample_time)
                    .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
                let altitude_m =
                    (position - body_state.position_inertial).length() - self.planet_radius_m;
                near_terrain |=
                    altitude_m <= self.rails_terrain_bound_m() + PILOT_SURFACE_CLEARANCE_M;
                track.push((sample_time, position));
            }
            near_terrain.then_some(track)
        } else {
            None
        };
        for body in ephemeris
            .bodies
            .iter()
            .filter(|body| body.radius_m > 0.0 || body.id == self.reference_body)
        {
            // Exact segment certification: five smooth samples of the true
            // craft-body separation, both endpoints evaluated on the live
            // ephemeris. The old worst-case margin (body approaching at
            // full chain speed for the whole batch) treated a co-moving
            // planet as hostile and refused every near-planet batch; the
            // between-sample curvature error is sub-meter, far inside the
            // surface clearance. Geometry keeps a hard refuse; only the
            // margin heuristic is gone.
            let terrain_bound = if body.id == self.reference_body && terrain_track.is_none() {
                self.rails_terrain_bound_m()
            } else {
                0.0
            };
            for point in 0..5 {
                let sample_time = SimTime(time.0 + (end.0 - time.0) * f64::from(point) / 4.0);
                let Some((position, _)) = self.rails.sample_at(sample_time) else {
                    return Ok(CoastAdvance::NotEligible);
                };
                let body_state = ephemeris
                    .body_state(body.id, sample_time)
                    .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
                let clearance_m =
                    (position - body_state.position_inertial).length() - body.radius_m;
                if clearance_m <= terrain_bound + PILOT_SURFACE_CLEARANCE_M
                    && !(body.id == self.reference_body && terrain_track.is_some())
                {
                    return Ok(CoastAdvance::NotEligible);
                }
                if body.id == self.reference_body {
                    let altitude_m =
                        (position - body_state.position_inertial).length() - self.planet_radius_m;
                    let density_kg_m3 = self
                        .atmosphere
                        .sample(altitude_m.max(0.0))
                        .map(|sample| sample.density_kg_m3)
                        .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
                    if density_kg_m3 != 0.0 {
                        return Ok(CoastAdvance::NotEligible);
                    }
                }
            }
        }
        if let Some(track) = terrain_track {
            let Ok(coverage) = self.certify_terrain_track(ephemeris, &track) else {
                return Ok(CoastAdvance::NotEligible);
            };
            // Coverage is proven by the obstacle report; this sampled margin
            // is the current gate. A sub-grid withstand bound is deliberately
            // left for the next certification layer.
            if coverage.min_obstacle_clearance_m <= PILOT_SURFACE_CLEARANCE_M {
                return Ok(CoastAdvance::NotEligible);
            }
        }
        let Some(orientation) = thessa_sim_core::constant_spin_orientation(
            self.state.orientation_body_to_inertial,
            self.state.angular_velocity_body_rps,
            self.vehicle.mass_properties.inertia_body_kg_m2,
            duration,
        ) else {
            return Ok(CoastAdvance::NotEligible);
        };
        self.regime = FlightRegime::Coast;
        let (position, velocity) = self.rails.sample_at(end).expect("bounded coast interval");
        let mut next = RigidBodyState::new(
            position,
            velocity,
            orientation,
            self.state.angular_velocity_body_rps,
        )
        .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let next_home = ephemeris
            .body_state(self.reference_body, end)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.validate_endpoint(ephemeris, &mut next, next_home, end)?;
        // One telemetry sample per batch, never per skipped translation tick.
        let gravity = GravityField::from_ephemeris(ephemeris)
            .acceleration(position, end)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(self.evaluate_forces(
            next,
            FlightStepInput {
                altitude_m: (position - next_home.position_inertial).length()
                    - self.planet_radius_m,
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: orientation.inverse() * (position - next_home.position_inertial),
                wind_velocity_body_mps: orientation.inverse() * next_home.velocity_inertial,
                extra_force_body_n: DVec3::ZERO,
                extra_moment_body_nm: DVec3::ZERO,
                skip_aero: true,
            },
        )?);
        self.state = next;
        self.commit_ticks(ticks)?;
        self.relative_position_m = position - next_home.position_inertial;
        Ok(CoastAdvance::Advanced(duration))
    }

    pub(super) fn arm_rails_wake(&mut self) {
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

    /// Spawn a full-horizon worker bake from the live state unless one is
    /// already running. No-op headless (no pool). The swap path validates
    /// ephemeris/config on arrival, so a bake that goes stale mid-flight
    /// is rejected harmlessly instead of corrupting the cache.
    /// Request a full-horizon worker bake from the live state unless one is
    /// already running. The [`BakeQueue`] decides how it runs (Bevy pool in
    /// the client, thread on the server, inline in tests). The swap path
    /// validates ephemeris/config on arrival, so a bake that goes stale
    /// mid-flight is rejected harmlessly instead of corrupting the cache.
    pub(super) fn spawn_rails_bake(&mut self, ephemeris: &BakedEphemeris) {
        if self.bake.has_pending() {
            return;
        }
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let time = SimTime(self.flight_time_s);
        let config = TickIntegratorConfig::default();
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        self.bake.request_bake(RailsBakeRequest {
            initial,
            time,
            config,
            impact_bodies,
            ephemeris: ephemeris.clone(),
        });
    }

    /// Poll a finished worker bake, adopting it only after the state/key
    /// check (which also rejects bakes from a pre-edit universe).
    pub(super) fn poll_rails_bake(&mut self, ephemeris: &BakedEphemeris) {
        let Some(result) = self.bake.poll_bake() else {
            return;
        };
        if let Ok(baked) = result {
            self.adopt_rails_bake(ephemeris, baked);
        }
    }

    /// Poll/build the common gravity forecast. Reading the map never runs a
    /// second integrator. Flight adopts it only after the state/key check.
    pub fn prepare_shared_trajectory(&mut self, ephemeris: &BakedEphemeris) -> bool {
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let time = SimTime(self.flight_time_s);
        let config = TickIntegratorConfig::default();
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        self.poll_rails_bake(ephemeris);
        if self.rails.usable_tick_for(
            ephemeris,
            initial,
            time,
            config,
            &impact_bodies,
            COAST_RAILS_POSITION_TOL_M,
            COAST_RAILS_VELOCITY_TOL_MPS,
        ) {
            return true;
        }
        // The queue decides how the bake runs (pool, thread, inline).
        // Poll again right away: inline queues already finished, so the
        // first coast step rides (and arms its wake) without deferring.
        self.spawn_rails_bake(ephemeris);
        self.poll_rails_bake(ephemeris);
        self.rails_usable_now(ephemeris)
    }

    /// Unpowered vacuum coast on the shared baked trajectory. Translation is
    /// sampled (cubic Hermite) from the rails path the map prediction draws;
    /// attitude keeps integrating under the RCS moment. Returns `None` when
    /// no rails path can serve this step (bake failure, horizon exhausted
    /// twice in a row) so the caller falls back to a normal integrated step.
    /// Flight and map share the same tick-adaptive bake and interpolant.
    pub(super) fn try_coast_step_on_rails(
        &mut self,
        ephemeris: &BakedEphemeris,
        time: SimTime,
        jet_moment: DVec3,
        body_state: BodyState,
        gravity: DVec3,
    ) -> Result<Option<(RigidBodyState, FlightForces)>, FlightError> {
        if !self.prepare_shared_trajectory(ephemeris) {
            return Ok(None);
        }
        self.arm_rails_wake();
        self.rails.trim_before(time, 3_600.0, 1_800.0);
        let next_time = self.time_after_ticks(1)?;
        if let Some(thessa_sim_core::OnRailsWake::Impact {
            time: impact_time,
            body,
        }) = self.rails.wake()
            && impact_time.0 <= next_time.0
        {
            self.wake_events.push(ScheduledEvent {
                id: 0,
                time: impact_time,
                kind: ScheduledKind::RailsImpact { body },
            });
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
                let event = match wake {
                    thessa_sim_core::OnRailsWake::Impact { time, body } => ScheduledEvent {
                        id: 0,
                        time,
                        kind: ScheduledKind::RailsImpact { body },
                    },
                    thessa_sim_core::OnRailsWake::HorizonEnd { time } => ScheduledEvent {
                        id: 0,
                        time,
                        kind: ScheduledKind::RailsHorizon,
                    },
                };
                self.wake_events.push(event);
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
        let forces = match self.evaluate_forces(
            self.state,
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
}
