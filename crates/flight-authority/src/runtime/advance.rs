//! Fixed-step advancement, work budgets, and on-rails coast execution.

use super::*;

impl FlightAuthority {
    /// Advance the exact production flight path at a fixed physics cadence.
    /// Render frames only contribute elapsed time; they never set solver dt.
    pub fn advance(
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
    pub(super) fn sync_world_tick(&mut self) -> Result<(), FlightError> {
        if self.world_tick.time().0 != self.flight_time_s {
            self.world_tick = thessa_sim_core::WorldTick::from_time(SimTime(self.flight_time_s))
                .ok_or_else(|| FlightError::InvalidInput("invalid world tick epoch".into()))?;
            self.flight_time_s = self.world_tick.time().0;
        }
        Ok(())
    }

    pub(super) fn time_after_ticks(&self, ticks: u64) -> Result<SimTime, FlightError> {
        self.world_tick
            .checked_add(ticks)
            .map(|tick| tick.time())
            .ok_or_else(|| FlightError::InvalidInput("world tick overflow".into()))
    }

    pub(super) fn commit_ticks(&mut self, ticks: u64) -> Result<(), FlightError> {
        self.world_tick = self
            .world_tick
            .checked_add(ticks)
            .ok_or_else(|| FlightError::InvalidInput("world tick overflow".into()))?;
        self.flight_time_s = self.world_tick.time().0;
        Ok(())
    }

    pub fn advance_with_budget(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), FlightError> {
        self.advance_with_budget_hook(ephemeris, mode, elapsed_s, budget, |_| Ok(()))
    }

    pub(super) fn advance_with_budget_hook<F>(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
        mut before_physical_step: F,
    ) -> Result<(), FlightError>
    where
        F: FnMut(&mut Self) -> Result<(), FlightError>,
    {
        self.sync_world_tick()?;
        let started = std::time::Instant::now();
        self.steps_this_frame = 0;
        self.rails_advanced_this_frame = 0.0;
        self.work_budget_exhausted = false;
        self.waiting_for_rails_bake = false;
        self.accumulator_s += elapsed_s;
        let gravity_field = GravityField::from_ephemeris(ephemeris);
        while self.accumulator_s + 1.0e-12 >= FLIGHT_STEP_S {
            before_physical_step(self)?;
            self.advance_propulsion_actuator()?;
            match self.try_advance_cached_coast(ephemeris, mode, self.accumulator_s)? {
                CoastAdvance::Advanced(coast) => {
                    self.accumulator_s = (self.accumulator_s - coast).max(0.0);
                    self.rails_advanced_this_frame += coast;
                    // Wake at the event boundary, allowing the owner to react
                    // before any further physical work in this frame.
                    if self
                        .scheduler
                        .next()
                        .is_some_and(|event| event.time.0 <= self.flight_time_s + FLIGHT_STEP_S)
                    {
                        break;
                    }
                    if budget.is_some_and(|limit| started.elapsed() >= limit) {
                        // A rails batch is one cooperative unit. If it used
                        // the current work quantum, discard only the unserved
                        // whole ticks; never turn a transient bake/CPU delay
                        // into a catch-up queue that starves later input.
                        self.discard_unserved_whole_ticks();
                        self.work_budget_exhausted = true;
                        break;
                    }
                    continue;
                }
                CoastAdvance::WaitingForBake => {
                    // The worker owns the expensive forecast. Preserve only
                    // the fractional lattice remainder while it finishes;
                    // later wall slices will request fresh demand.
                    self.discard_unserved_whole_ticks();
                    self.waiting_for_rails_bake = true;
                    break;
                }
                CoastAdvance::NotEligible => {}
            }
            self.step(ephemeris, &gravity_field, mode)?;
            self.steps_this_frame += 1;
            self.accumulator_s = (self.accumulator_s - FLIGHT_STEP_S).max(0.0);
            if budget.is_some_and(|limit| started.elapsed() >= limit) {
                // Excess requested warp is unserved wall-time demand, not
                // elapsed simulation time. Keep only the fractional tick;
                // don't build a catch-up queue that delays later inputs.
                self.discard_unserved_whole_ticks();
                self.work_budget_exhausted = true;
                break;
            }
        }
        // Event-driven wakes: due scheduler events fire here instead of being
        // polled every physics tick.
        for event in self.scheduler.drain_due(SimTime(self.flight_time_s)) {
            self.wake_events.push(event);
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
                ScheduledKind::BurnSegment { .. } => {
                    format!("WAKE BURN T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::Alarm => {
                    format!("WAKE ALARM T+{:.0}s", event.time.seconds())
                }
            });
        }
        Ok(())
    }

    pub(super) fn discard_unserved_whole_ticks(&mut self) {
        self.accumulator_s = self.accumulator_s.rem_euclid(FLIGHT_STEP_S);
    }

    pub(super) fn allocate_controls(
        &mut self,
        kinematics: LocalAirKinematics,
        mode: ControlMode,
    ) -> Result<ControlAllocation, FlightError> {
        let attitude = attitude_demand(
            mode,
            self.sas_enabled,
            self.control_input,
            self.state,
            self.sas_target_orientation,
            self.vehicle.mass_properties.inertia_body_kg_m2,
        );
        let explicit_moment = self.explicit_moment_demand_nm;
        if explicit_moment.is_none() {
            self.sas_target_orientation = attitude.target_orientation;
        }
        let axes = explicit_moment.map_or(attitude.axes, |_| DVec3::ZERO);
        let assisted = explicit_moment.is_some() || mode != ControlMode::Direct;
        let requested = explicit_moment.unwrap_or(attitude.requested_moment_nm);
        // Vacuum fast path: no air load exists, so the trim solve and the
        // response evaluation are skipped outright (exactly zero aero
        // moment); attitude flies on RCS alone. Physical actuators still
        // advance in vacuum, where aerodynamic hinge load is exactly zero.
        if self.regime == FlightRegime::Coast {
            let surface_command = if assisted {
                self.surface_input
            } else {
                self.control_input
            };
            let actuator_saturated = self.advance_coast_control_actuators(surface_command)?;
            if !self.vehicle.reaction_wheels.is_empty() {
                let wheel_request = if !assisted && axes == DVec3::ZERO {
                    DVec3::ZERO
                } else {
                    requested
                };
                let moment = self.allocate_reaction_wheel_residual(wheel_request, DVec3::ZERO)?;
                self.actuator_saturated |= actuator_saturated;
                return Ok(ControlAllocation {
                    moment_body_nm: moment,
                    aero_result: None,
                });
            }
            let allocation = allocate_rcs(requested, DVec3::ZERO, axes, assisted, self.rcs_enabled);
            self.reaction_wheel_torque_body_nm = DVec3::ZERO;
            self.actuator_saturated = allocation.saturated || actuator_saturated;
            return Ok(ControlAllocation {
                moment_body_nm: allocation.moment_body_nm,
                aero_result: None,
            });
        }
        let environment = self
            .atmosphere
            .aero_environment(kinematics.altitude_m.max(0.0), DVec3::ZERO)?;
        let omega = self.state.angular_velocity_body_rps;
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
            self.trim_solves += 1;
            let result = solve_aero_trim(
                &mut self.vehicle,
                &self.control_reference_geometry,
                &self.aero_model,
                aero_state,
                environment,
                requested,
                command,
                self.force_trim_two_pass,
            )?;
            command = result.command;
            self.trim_second_passes += result.second_passes;
        }
        let max_change = SURFACE_COMMAND_RATE_S * FLIGHT_STEP_S;
        self.surface_input = slew_surface_command(self.surface_input, command, max_change);
        // A trim probe temporarily mutates panel geometry while estimating
        // commanded effectiveness. Direct allocation leaves the last actual
        // geometry intact, so only the trim path needs this restore.
        if solve_trim {
            self.vehicle
                .apply_control_deflections(
                    &self.control_reference_geometry,
                    &self.control_deflections_rad,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        }
        let legacy_commands = control_surface_commands(
            &self.vehicle.control_surfaces,
            ControlChannels {
                pitch: self.surface_input.x,
                yaw: self.surface_input.y,
                roll: self.surface_input.z,
                ..ControlChannels::default()
            },
        );
        let actuator_commands = control_surface_commands(
            &self.vehicle.control_surfaces,
            ControlChannels {
                pitch: command.x,
                yaw: command.y,
                roll: command.z,
                ..ControlChannels::default()
            },
        );
        let commands: Vec<_> = self
            .vehicle
            .control_surfaces
            .iter()
            .enumerate()
            .map(|(index, surface)| {
                if surface.actuator.is_some() {
                    actuator_commands[index]
                } else {
                    legacy_commands[index]
                }
            })
            .collect();
        let actuator_loads_nm = if self
            .vehicle
            .control_surfaces
            .iter()
            .any(|surface| surface.actuator.is_some())
        {
            let detailed = self.aero_model.evaluate_state_detailed(
                aero_state,
                environment,
                &self.vehicle.aero_geometry,
            )?;
            self.vehicle
                .control_hinge_moments(&detailed)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?
        } else {
            vec![0.0; self.vehicle.control_surfaces.len()]
        };
        let (next_deflections, actuator_saturated) = self
            .vehicle
            .advance_control_actuators(
                &self.control_deflections_rad,
                &commands,
                &actuator_loads_nm,
                FLIGHT_STEP_S,
            )
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        self.control_deflections_rad = next_deflections;
        self.vehicle
            .apply_control_deflections(
                &self.control_reference_geometry,
                &self.control_deflections_rad,
            )
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        if self
            .vehicle
            .control_surfaces
            .iter()
            .any(|surface| surface.hinge.is_some())
        {
            self.aero_panels
                .sync_geometry(&self.vehicle.aero_geometry)
                .map_err(FlightError::Aero)?;
        } else {
            self.aero_panels
                .sync_deflections(&self.vehicle.aero_geometry)
                .map_err(FlightError::Aero)?;
        }
        let aero_result = self
            .aero_model
            .evaluate_soa_simd_scratch(
                aero_state,
                environment,
                &self.aero_panels,
                false,
                &mut self.aero_scratch,
            )
            .map_err(FlightError::Aero)?;
        let actual_aero = aero_result.moment_body_nm;
        if !self.vehicle.reaction_wheels.is_empty() {
            let (wheel_request, wheel_aero_moment) = if !assisted && axes == DVec3::ZERO {
                (DVec3::ZERO, DVec3::ZERO)
            } else {
                (requested, actual_aero)
            };
            let moment = self.allocate_reaction_wheel_residual(wheel_request, wheel_aero_moment)?;
            self.actuator_saturated |= actuator_saturated;
            return Ok(ControlAllocation {
                moment_body_nm: moment,
                aero_result: Some(aero_result),
            });
        }
        let allocation = allocate_rcs(requested, actual_aero, axes, assisted, self.rcs_enabled);
        self.reaction_wheel_torque_body_nm = DVec3::ZERO;
        self.actuator_saturated = allocation.saturated || actuator_saturated;
        Ok(ControlAllocation {
            moment_body_nm: allocation.moment_body_nm,
            aero_result: Some(aero_result),
        })
    }

    /// Allocate the post-aerodynamic attitude demand to internal wheels first,
    /// then use RCS for any remaining moment. This preserves the physical
    /// moment interface while giving KSP-style wheels continuous authority at
    /// their configured motor torque, with no rotor-speed saturation.
    pub(super) fn allocate_reaction_wheel_residual(
        &mut self,
        requested_moment_nm: DVec3,
        actual_aero_moment_nm: DVec3,
    ) -> Result<DVec3, FlightError> {
        let residual = requested_moment_nm - actual_aero_moment_nm;
        self.sync_reaction_wheel_runtime_state();
        let wheel = if self.reaction_wheels_enabled {
            allocate_reaction_wheels_with_enabled_banks(
                &self.vehicle.reaction_wheels,
                &self.reaction_wheel_bank_enabled,
                residual,
            )
            .map_err(|error| {
                FlightError::InvalidInput(format!("reaction-wheel allocation failed: {error}"))
            })?
        } else {
            ReactionWheelAllocation {
                delivered_torque_body_nm: DVec3::ZERO,
                saturated: false,
            }
        };
        self.reaction_wheel_torque_body_nm = wheel.delivered_torque_body_nm;
        let rcs_request = residual - wheel.delivered_torque_body_nm;
        let rcs_torque = rcs_moment(rcs_request, self.rcs_enabled);
        let unserved = rcs_request - rcs_torque;
        self.actuator_saturated = unserved.length_squared() > 1.0e-12;
        Ok(wheel.delivered_torque_body_nm + rcs_torque)
    }

    pub(super) fn advance_coast_control_actuators(
        &mut self,
        normalized_command: DVec3,
    ) -> Result<bool, FlightError> {
        let commands = control_surface_commands(
            &self.vehicle.control_surfaces,
            ControlChannels {
                pitch: normalized_command.x,
                yaw: normalized_command.y,
                roll: normalized_command.z,
                ..ControlChannels::default()
            },
        );
        let mut next = self.control_deflections_rad.clone();
        let mut saturated = false;
        for (index, surface) in self.vehicle.control_surfaces.iter().enumerate() {
            let Some(actuator) = surface.actuator else {
                continue;
            };
            let target = surface
                .deflection_for_command(commands[index])
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            let actual = actuator
                .advance(
                    self.control_deflections_rad[index],
                    target,
                    0.0,
                    FLIGHT_STEP_S,
                    surface.minimum_deflection_rad,
                    surface.maximum_deflection_rad,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            saturated |= (target - actual).abs() > 1.0e-12;
            next[index] = actual;
        }
        self.control_deflections_rad = next;
        self.vehicle
            .apply_control_deflections(
                &self.control_reference_geometry,
                &self.control_deflections_rad,
            )
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        if self
            .vehicle
            .control_surfaces
            .iter()
            .any(|surface| surface.hinge.is_some())
        {
            self.aero_panels
                .sync_geometry(&self.vehicle.aero_geometry)
                .map_err(FlightError::Aero)?;
        }
        Ok(saturated)
    }

    pub(super) fn allocate_explicit_force(&mut self) -> DVec3 {
        let Some(request) = self.explicit_force_demand_body_n else {
            return DVec3::ZERO;
        };
        let allocation = allocate_rcs_force(request, self.rcs_enabled);
        self.actuator_saturated |= allocation.saturated;
        allocation.force_body_n
    }

    /// Upper-atmosphere band model: between the vacuum cutoff and the
    /// Coast density the panel loop is replaced by reference-area drag
    /// along the airstream (no lift, no aero moment; RCS unchanged).
    /// Returns the body drag force plus the measured dynamic pressure for
    /// display, or `None` outside the band. The calibration test pins the
    /// absolute error against the full panel sum; the batch cap certifies
    /// the drift this approximation can accumulate over one jump.
    pub(super) fn upper_band_drag(
        &self,
        kinematics: LocalAirKinematics,
        density_kg_m3: f64,
    ) -> Option<(DVec3, f64)> {
        // This reduction is calibrated against panel-only geometry. Keep
        // blunt shields in the shared aero solver until the reduction has a
        // pinned drag/lift error envelope for their incidence-dependent load.
        if !self.vehicle.aero_geometry.blunt_discs.is_empty()
            || density_kg_m3 <= self.atmosphere.vacuum_cutoff_density_kg_m3
            || density_kg_m3 >= COAST_DENSITY_KG_M3
        {
            return None;
        }
        let airspeed_mps = kinematics.air_velocity_body_mps.length();
        if airspeed_mps == 0.0 {
            return Some((DVec3::ZERO, 0.0));
        }
        let dynamic_pressure_pa = 0.5 * density_kg_m3 * airspeed_mps * airspeed_mps;
        let drag_body_n = -kinematics.air_velocity_body_mps / airspeed_mps
            * (dynamic_pressure_pa
                * self.aero_model.config.upper_atmosphere_drag_coefficient
                * self.reference_area_m2);
        Some((drag_body_n, dynamic_pressure_pa))
    }

    /// Full rigid-body step for powered/aero flight (translation integrated).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn integrate_powered_step(
        &mut self,
        gravity: DVec3,
        kinematics: LocalAirKinematics,
        body_state: BodyState,
        jet_moment: DVec3,
        rcs_force_body_n: DVec3,
        thrust_n: f64,
        band_drag_body_n: DVec3,
        parachute_force_body_n: DVec3,
        parachute_moment_body_nm: DVec3,
        skip_aero: bool,
        precomputed_aero: Option<AeroResult>,
    ) -> Result<(RigidBodyState, FlightForces), FlightError> {
        let input = powered_step_input(
            self.state,
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
        if let Some(aero_result) = precomputed_aero {
            integrate_rigid_body_step_with_aero_result(
                self.atmosphere,
                self.state,
                self.vehicle.mass_properties,
                input,
                FLIGHT_STEP_S,
                aero_result,
            )
        } else {
            self.aero_panels
                .sync_deflections(&self.vehicle.aero_geometry)
                .map_err(FlightError::Aero)?;
            integrate_rigid_body_step_soa(
                &self.aero_model,
                &self.aero_panels,
                &mut self.aero_scratch,
                self.atmosphere,
                self.state,
                self.vehicle.mass_properties,
                input,
                FLIGHT_STEP_S,
            )
        }
    }

    /// Arm the baked path's wake condition in the simulation-time scheduler:
    /// one path owns exactly one wake (impact epoch or horizon end), so the
    /// flight loop and autopilot wait on events instead of polling.
    pub(super) fn step(
        &mut self,
        ephemeris: &BakedEphemeris,
        gravity_field: &GravityField,
        mode: ControlMode,
    ) -> Result<(), FlightError> {
        self.sync_world_tick()?;
        let time = self.world_tick.time();
        // One memoized frame per tick serves the reference body and gravity.
        // Values are bitwise identical to individual lookups (see
        // `EphemerisFrame`); the borrow ends before any `&mut self` use.
        let position_inertial_m = self.state.position_inertial_m;
        let (body_state, gravity) = {
            let states = self
                .ephemeris_frame
                .evaluate(ephemeris, time)
                .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let body_state = *states.get(self.reference_body.index()).ok_or_else(|| {
                FlightError::InvalidInput(
                    thessa_sim_core::EphemerisError::UnknownBody(self.reference_body).to_string(),
                )
            })?;
            let gravity = gravity_field
                .acceleration_from_states(position_inertial_m, states, time)
                .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            (body_state, gravity)
        };
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        // Regime follows sampled density, never altitude: an invalid sample
        // keeps Aero so the existing atmosphere error paths still fire.
        let atmosphere_sample = self.atmosphere.sample(kinematics.altitude_m.max(0.0))?;
        let density_kg_m3 = atmosphere_sample.density_kg_m3;
        self.regime = if density_kg_m3 < COAST_DENSITY_KG_M3 {
            FlightRegime::Coast
        } else {
            FlightRegime::Aero
        };
        let (parachute_force_body_n, parachute_moment_body_nm) =
            self.advance_parachutes(kinematics, atmosphere_sample)?;
        let control_allocation = self.allocate_controls(kinematics, mode)?;
        let jet_moment = control_allocation.moment_body_nm;
        let precomputed_aero = control_allocation.aero_result;
        let rcs_force_body_n = self.allocate_explicit_force();
        // Only an exactly empty sampled medium permits zero force; the
        // vacuum cutoff guarantees exactness below its threshold.
        let vacuum = density_kg_m3 == 0.0;
        // Upper band: reference-area drag replaces the panel loop, whose
        // cost dominates multi-vehicle steps while its output is deep
        // below the gravity/thrust scales. Moments stay RCS-only.
        let band = self.upper_band_drag(kinematics, density_kg_m3);
        let skip_aero = vacuum || band.is_some();
        let (band_drag_body_n, band_q_pa) = band.unwrap_or((DVec3::ZERO, 0.0));
        let thrust_n = self.thrust_n();
        // Contact-active ticks integrate through Rapier with the same
        // sampled loads; the free-flight integrator never runs for them.
        // The rails coast below is additionally guarded, so no baked batch
        // can span the regime change from either direction.
        let contact_active = self.poll_contact_activation(kinematics)?;
        let (mut next, mut forces) = if contact_active {
            self.scheduler.clear_rails_wakes();
            self.step_contact_active(
                ephemeris,
                body_state,
                gravity,
                kinematics,
                jet_moment,
                rcs_force_body_n,
                thrust_n,
                band_drag_body_n,
                parachute_force_body_n,
                parachute_moment_body_nm,
                skip_aero,
                precomputed_aero,
            )?
        } else if self.regime == FlightRegime::Coast
            && thrust_n == 0.0
            && skip_aero
            && !self.landing_gear_transitioning()
            && !self.parachute_rails_ineligible()
        {
            match self.try_coast_step_on_rails(ephemeris, time, jet_moment, body_state, gravity)? {
                Some(coasted) => coasted,
                None => self.integrate_powered_step(
                    gravity,
                    kinematics,
                    body_state,
                    jet_moment,
                    rcs_force_body_n,
                    thrust_n,
                    band_drag_body_n,
                    parachute_force_body_n,
                    parachute_moment_body_nm,
                    skip_aero,
                    precomputed_aero,
                )?,
            }
        } else {
            self.scheduler.clear_rails_wakes();
            self.integrate_powered_step(
                gravity,
                kinematics,
                body_state,
                jet_moment,
                rcs_force_body_n,
                thrust_n,
                band_drag_body_n,
                parachute_force_body_n,
                parachute_moment_body_nm,
                skip_aero,
                precomputed_aero,
            )?
        };
        if !contact_active {
            self.advance_freeflight_landing_gear()?;
        }
        // skip_aero zeroes the aero summary; restore the measured dynamic
        // pressure so HUD readouts stay on-model through the band.
        if band_q_pa > 0.0 {
            forces.aero.dynamic_pressure_pa = band_q_pa;
        }
        // Guard reads come from the same memoized frame at the endpoint
        // time: dominant-pull scan plus reference/ guard states, no chains.
        let next_time = self.time_after_ticks(1)?;
        let next_position_inertial_m = next.position_inertial_m;
        let reference_body = self.reference_body;
        let (next_body, guard_body, guard_state, guard_radius) = {
            let states = self
                .ephemeris_frame
                .evaluate(ephemeris, next_time)
                .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let missing = |id: BodyId| {
                FlightError::InvalidInput(
                    thessa_sim_core::EphemerisError::UnknownBody(id).to_string(),
                )
            };
            let next_body = *states
                .get(reference_body.index())
                .ok_or_else(|| missing(reference_body))?;
            let guard_body = ephemeris
                .dominant_body_from_states(next_position_inertial_m, states)
                .unwrap_or(reference_body);
            let guard_state = *states.get(guard_body.index()).unwrap_or(&next_body);
            let guard_radius = ephemeris
                .body(guard_body)
                .map(|body| body.radius_m)
                .unwrap_or(self.planet_radius_m);
            (next_body, guard_body, guard_state, guard_radius)
        };
        self.validate_endpoint_with_guard(
            &mut next,
            next_body,
            next_time,
            guard_body,
            guard_state,
            guard_radius,
        )?;
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
        self.commit_ticks(1)?;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(forces);
        self.relative_position_m = next.position_inertial_m - next_body.position_inertial;
        Ok(())
    }
}
