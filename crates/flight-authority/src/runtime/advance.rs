//! Fixed-step advancement, work budgets, and on-rails coast execution.

use super::*;

struct InstalledResourceStep {
    propulsion: Option<VehiclePropulsionAllocation>,
    electrical_power_state: thessa_sim_core::ElectricalPowerState,
    electrical_power_telemetry: thessa_sim_core::ElectricalPowerTelemetry,
    auxiliary_power_unit_states: Vec<thessa_sim_core::AuxiliaryPowerUnitState>,
    jet_commands: Vec<thessa_sim_core::JetCommand>,
    pulsed_fusion_states: Vec<thessa_sim_core::PulsedFusionState>,
    turboprop_commands: Vec<thessa_sim_core::TurbopropCommand>,
    accessory_force_body_n: DVec3,
    accessory_moment_body_nm: DVec3,
    rcs_force_body_n: DVec3,
    rcs_moment_body_nm: DVec3,
    resource_limited: bool,
}

fn set_named_power_load_request(
    vehicle: &thessa_sim_core::VehicleDefinition,
    command: &mut thessa_sim_core::ElectricalPowerCommand,
    name: &str,
    requested_power_w: f64,
) {
    if let Some((index, consumer)) = vehicle
        .electrical_power
        .consumers
        .iter()
        .enumerate()
        .find(|(_, consumer)| consumer.name == name)
    {
        command.consumer_power_w[index] =
            if requested_power_w.is_finite() && requested_power_w >= 0.0 {
                requested_power_w.min(consumer.rated_power_w)
            } else {
                requested_power_w
            };
    }
}

fn delivered_power_fraction(
    telemetry: &thessa_sim_core::ElectricalPowerTelemetry,
    consumer_name: &str,
    requested_power_w: f64,
) -> f64 {
    if requested_power_w > 0.0 && requested_power_w.is_finite() {
        (telemetry.supplied_power_w(consumer_name).unwrap_or(0.0) / requested_power_w)
            .clamp(0.0, 1.0)
    } else {
        0.0
    }
}

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
                let ambient_pa = self
                    .atmosphere
                    .sample(kinematics.altitude_m.max(0.0))?
                    .pressure_pa;
                let (moment, rcs_duties) = self.allocate_reaction_wheel_residual_with_force(
                    wheel_request,
                    DVec3::ZERO,
                    self.explicit_force_demand_body_n.unwrap_or(DVec3::ZERO),
                    ambient_pa,
                )?;
                self.actuator_saturated |= actuator_saturated;
                return Ok(ControlAllocation {
                    moment_body_nm: moment,
                    rcs_duties,
                    aero_result: None,
                });
            }
            if !self.vehicle.rcs_mounts.is_empty() {
                let (rcs_duties, saturated) = self.allocate_mounted_rcs(
                    self.explicit_force_demand_body_n.unwrap_or(DVec3::ZERO),
                    requested,
                    0.0,
                )?;
                self.actuator_saturated = saturated || actuator_saturated;
                return Ok(ControlAllocation {
                    moment_body_nm: DVec3::ZERO,
                    rcs_duties,
                    aero_result: None,
                });
            }
            let allocation = allocate_rcs(requested, DVec3::ZERO, axes, assisted, self.rcs_enabled);
            self.reaction_wheel_torque_body_nm = DVec3::ZERO;
            self.actuator_saturated = allocation.saturated || actuator_saturated;
            return Ok(ControlAllocation {
                moment_body_nm: allocation.moment_body_nm,
                rcs_duties: Vec::new(),
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
            let ambient_pa = self
                .atmosphere
                .sample(kinematics.altitude_m.max(0.0))?
                .pressure_pa;
            let (moment, rcs_duties) = self.allocate_reaction_wheel_residual_with_force(
                wheel_request,
                wheel_aero_moment,
                self.explicit_force_demand_body_n.unwrap_or(DVec3::ZERO),
                ambient_pa,
            )?;
            self.actuator_saturated |= actuator_saturated;
            return Ok(ControlAllocation {
                moment_body_nm: moment,
                rcs_duties,
                aero_result: Some(aero_result),
            });
        }
        if !self.vehicle.rcs_mounts.is_empty() {
            let ambient_pa = self
                .atmosphere
                .sample(kinematics.altitude_m.max(0.0))?
                .pressure_pa;
            let rcs_moment_request = if assisted {
                requested - actual_aero
            } else {
                requested
            };
            let (rcs_duties, saturated) = self.allocate_mounted_rcs(
                self.explicit_force_demand_body_n.unwrap_or(DVec3::ZERO),
                rcs_moment_request,
                ambient_pa,
            )?;
            self.reaction_wheel_torque_body_nm = DVec3::ZERO;
            self.actuator_saturated = saturated || actuator_saturated;
            return Ok(ControlAllocation {
                moment_body_nm: DVec3::ZERO,
                rcs_duties,
                aero_result: Some(aero_result),
            });
        }
        let allocation = allocate_rcs(requested, actual_aero, axes, assisted, self.rcs_enabled);
        self.reaction_wheel_torque_body_nm = DVec3::ZERO;
        self.actuator_saturated = allocation.saturated || actuator_saturated;
        Ok(ControlAllocation {
            moment_body_nm: allocation.moment_body_nm,
            rcs_duties: Vec::new(),
            aero_result: Some(aero_result),
        })
    }

    /// Allocate the post-aerodynamic attitude demand to internal wheels first,
    /// then use RCS for any remaining moment. This preserves the physical
    /// moment interface while giving KSP-style wheels continuous authority at
    /// their configured motor torque, with no rotor-speed saturation.
    #[cfg(test)]
    pub(super) fn allocate_reaction_wheel_residual(
        &mut self,
        requested_moment_nm: DVec3,
        actual_aero_moment_nm: DVec3,
    ) -> Result<DVec3, FlightError> {
        self.allocate_reaction_wheel_residual_with_force(
            requested_moment_nm,
            actual_aero_moment_nm,
            DVec3::ZERO,
            0.0,
        )
        .map(|(moment, _)| moment)
    }

    fn allocate_reaction_wheel_residual_with_force(
        &mut self,
        requested_moment_nm: DVec3,
        actual_aero_moment_nm: DVec3,
        requested_force_body_n: DVec3,
        ambient_pa: f64,
    ) -> Result<(DVec3, Vec<f64>), FlightError> {
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
        if !self.vehicle.rcs_mounts.is_empty() {
            let (duties, saturated) =
                self.allocate_mounted_rcs(requested_force_body_n, rcs_request, ambient_pa)?;
            self.actuator_saturated |= wheel.saturated || saturated;
            return Ok((wheel.delivered_torque_body_nm, duties));
        }
        let rcs_torque = rcs_moment(rcs_request, self.rcs_enabled);
        let unserved = rcs_request - rcs_torque;
        self.actuator_saturated = unserved.length_squared() > 1.0e-12;
        Ok((wheel.delivered_torque_body_nm + rcs_torque, Vec::new()))
    }

    /// Allocate a simultaneous body force/moment request over the actually
    /// installed RCS nozzles. Each command is a pulse duty for this fixed
    /// step; one full-command effector is calibrated from its valve-rise-aware
    /// pulse impulse rather than an authored control coefficient.
    fn allocate_mounted_rcs(
        &self,
        requested_force_body_n: DVec3,
        requested_moment_body_nm: DVec3,
        ambient_pa: f64,
    ) -> Result<(Vec<f64>, bool), FlightError> {
        if !self.rcs_enabled {
            return Ok((
                vec![0.0; self.vehicle.rcs_mounts.len()],
                requested_force_body_n.length_squared() > 1.0e-12
                    || requested_moment_body_nm.length_squared() > 1.0e-12,
            ));
        }
        let dt_s = FLIGHT_STEP_S;
        let mut effectors = Vec::with_capacity(self.vehicle.rcs_mounts.len());
        for mount in &self.vehicle.rcs_mounts {
            let pulse = mount
                .thruster
                .pulse(dt_s, mount.thruster.rated_inlet_pressure_pa(), ambient_pa)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            let force = DVec3::from_array(mount.direction_body) * (pulse.impulse_ns / dt_s);
            let moment = DVec3::from_array(mount.position_body_m).cross(force);
            effectors.push(thessa_flight_control::EffectorContribution {
                group: thessa_flight_control::ActuatorGroup::Rcs,
                force_per_command_n: force,
                moment_per_command_nm: moment,
                max_command: 1.0,
                weight: 1.0,
            });
        }
        let allocation = thessa_flight_control::allocate_wrench(
            thessa_flight_control::ControlDemand {
                force_body_n: requested_force_body_n,
                moment_body_nm: requested_moment_body_nm,
                propulsion: thessa_flight_control::PropulsionDemand { normalized: 0.0 },
            },
            &effectors,
        )
        .map_err(|error| {
            FlightError::InvalidInput(format!("mounted RCS allocation failed: {error}"))
        })?;
        Ok((allocation.commands, allocation.saturated))
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
        propulsion_force_body_n: DVec3,
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
            propulsion_force_body_n,
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

    fn plan_installed_propulsion(
        &self,
        ambient_pa: f64,
        additional_demands: &[thessa_sim_core::VehicleResourceDemand],
    ) -> Result<Option<VehiclePropulsionAllocation>, FlightError> {
        if self.vehicle.engines.is_empty()
            && self.vehicle.systems.is_empty()
            && additional_demands.is_empty()
        {
            return Ok(None);
        }
        let vessel_throttle = if self.engine_active {
            self.propulsion_actual
        } else {
            0.0
        };
        let engine_throttles: Vec<_> = self
            .engine_throttle_overrides
            .iter()
            .map(|override_throttle| override_throttle.unwrap_or(vessel_throttle))
            .collect();
        let system_throttles: Vec<Vec<_>> = self
            .system_throttle_overrides
            .iter()
            .map(|chambers| {
                chambers
                    .iter()
                    .map(|override_throttle| override_throttle.unwrap_or(vessel_throttle))
                    .collect()
            })
            .collect();
        self.vehicle
            .plan_propulsion_step_with_resource_demands(
                &self.resource_state,
                &engine_throttles,
                &system_throttles,
                ambient_pa,
                FLIGHT_STEP_S,
                additional_demands,
            )
            .map(Some)
            .map_err(|error| FlightError::InvalidInput(error.to_string()))
    }

    /// Evaluate APU, air-breathing jet, fuel-cell, and rocket operating points
    /// against one fixed-step tank transaction. Resource-limited auxiliary
    /// commands are reduced and re-evaluated before their forces or bus output
    /// are accepted.
    fn plan_installed_resource_step(
        &self,
        ambient_pa: f64,
        condition: &thessa_sim_core::FlightCondition,
        requested_rcs_duties: &[f64],
    ) -> Result<InstalledResourceStep, FlightError> {
        let mut apu_commands = self.auxiliary_power_unit_commands.clone();
        if apu_commands.len() != self.vehicle.auxiliary_power_units.len()
            || self.auxiliary_power_unit_states.len() != self.vehicle.auxiliary_power_units.len()
        {
            return Err(FlightError::InvalidInput(
                "APU runtime state does not match installed mounts".into(),
            ));
        }
        for command in &mut apu_commands {
            command.dt_s = FLIGHT_STEP_S;
        }
        let jet_commands = self.jet_commands.clone();
        if jet_commands.len() != self.vehicle.jets.len() {
            return Err(FlightError::InvalidInput(
                "jet runtime state does not match installed mounts".into(),
            ));
        }
        if requested_rcs_duties.len() != self.vehicle.rcs_mounts.len() {
            return Err(FlightError::InvalidInput(
                "mounted RCS commands do not match installed thrusters".into(),
            ));
        }
        if self.electric_thruster_requested_flow_kg_s.len() != self.vehicle.electric_thrusters.len()
            || self.fusion_torch_commands.len() != self.vehicle.fusion_torches.len()
            || self.pulsed_fusion_states.len() != self.vehicle.pulsed_fusion_systems.len()
            || self.pulsed_fusion_commands.len() != self.vehicle.pulsed_fusion_systems.len()
            || self.propeller_drive_commands.len() != self.vehicle.propeller_drives.len()
            || self.turboprop_commands.len() != self.vehicle.turboprops.len()
        {
            return Err(FlightError::InvalidInput(
                "installed propulsion runtime commands do not match vehicle mounts".into(),
            ));
        }
        let turboprop_commands = self.turboprop_commands.clone();
        let pneumatic_starter_requests: Vec<_> = self
            .vehicle
            .jets
            .iter()
            .zip(&jet_commands)
            .map(|(mount, command)| {
                if command.starter_engaged {
                    mount.pneumatic_starter_input_power_w()
                } else {
                    0.0
                }
            })
            .collect();
        let turboprop_starter_requests: Vec<_> = self
            .vehicle
            .turboprops
            .iter()
            .zip(&turboprop_commands)
            .map(|(mount, command)| {
                let starter = &mount.drive.air.shaft.starter;
                if command.shaft.starter_engaged
                    && starter.kind == thessa_sim_core::StarterKind::Pneumatic
                {
                    starter.power_w / starter.kind.efficiency()
                } else {
                    0.0
                }
            })
            .collect();
        let total_pneumatic_starter_request_w: f64 = pneumatic_starter_requests
            .iter()
            .chain(&turboprop_starter_requests)
            .sum();
        if total_pneumatic_starter_request_w > 0.0 {
            let active_apus = self
                .auxiliary_power_unit_states
                .iter()
                .filter(|state| state.shaft.lit)
                .count();
            for (index, command) in apu_commands.iter_mut().enumerate() {
                command.pneumatic_bleed_power_w =
                    if active_apus > 0 && self.auxiliary_power_unit_states[index].shaft.lit {
                        total_pneumatic_starter_request_w / active_apus as f64
                    } else {
                        0.0
                    };
            }
        }
        let mut jet_throttle_scales = vec![1.0; self.vehicle.jets.len()];
        let mut fuel_cell_scales = vec![1.0; self.vehicle.electrical_power.fuel_cells.len()];
        let mut rcs_duty_scales = vec![1.0; self.vehicle.rcs_mounts.len()];
        let mut electric_thruster_flow_scales = vec![1.0; self.vehicle.electric_thrusters.len()];
        let mut fusion_torch_scales = vec![1.0; self.vehicle.fusion_torches.len()];
        let mut pulsed_fusion_allowed = vec![true; self.vehicle.pulsed_fusion_systems.len()];
        let mut propeller_drive_scales = vec![1.0; self.vehicle.propeller_drives.len()];
        let mut turboprop_scales = vec![1.0; self.vehicle.turboprops.len()];
        let mut resource_limited = false;
        for _ in 0..24 {
            let apu_step = self
                .vehicle
                .plan_auxiliary_power_units(
                    &self.resource_state,
                    &self.auxiliary_power_unit_states,
                    &apu_commands,
                    condition,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            let mut accessory_force_body_n = DVec3::ZERO;
            let mut accessory_moment_body_nm = DVec3::ZERO;
            let mut rcs_force_body_n = DVec3::ZERO;
            let mut rcs_moment_body_nm = DVec3::ZERO;
            let mut additional_demands = Vec::new();
            for ((mount, point), _) in self
                .vehicle
                .auxiliary_power_units
                .iter()
                .zip(&apu_step.operating_points)
                .zip(&apu_step.next_states)
            {
                let thrust = DVec3::from_array(mount.thrust_axis_body) * point.air.thrust_n;
                accessory_force_body_n += thrust;
                accessory_moment_body_nm += DVec3::from_array(mount.position_body_m).cross(thrust);
                additional_demands.extend(thessa_sim_core::VehicleResourceDemand::from_apu_point(
                    &mount.name,
                    point,
                    mount.feed_port_name.as_deref(),
                ));
            }

            let vessel_throttle = if self.engine_active {
                self.propulsion_actual
            } else {
                0.0
            };
            let mut next_jet_commands = Vec::with_capacity(self.vehicle.jets.len());
            for (index, (mount, previous)) in
                self.vehicle.jets.iter().zip(&jet_commands).enumerate()
            {
                let mut command = *previous;
                command.dt_s = FLIGHT_STEP_S;
                command.pneumatic_starter_power_w = if total_pneumatic_starter_request_w > 0.0 {
                    apu_step.pneumatic_bleed_power_w * pneumatic_starter_requests[index]
                        / total_pneumatic_starter_request_w
                } else {
                    0.0
                };
                let throttle = vessel_throttle * jet_throttle_scales[index];
                let (point, transient, shaft) = mount
                    .estoc_point(throttle, condition, &command)
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                let thrust = DVec3::from_array(mount.thrust_axis_body) * point.thrust_n;
                accessory_force_body_n += thrust;
                accessory_moment_body_nm += DVec3::from_array(mount.position_body_m).cross(thrust);
                additional_demands.extend(thessa_sim_core::VehicleResourceDemand::from_jet_point(
                    mount, &point, None,
                ));
                next_jet_commands.push(command.with_state(&point, transient, shaft));
            }
            for (index, mount) in self.vehicle.rcs_mounts.iter().enumerate() {
                let duty = (requested_rcs_duties[index] * rcs_duty_scales[index]).clamp(0.0, 1.0);
                let pulse = mount
                    .thruster
                    .pulse(
                        FLIGHT_STEP_S * duty,
                        mount.thruster.rated_inlet_pressure_pa(),
                        ambient_pa,
                    )
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                let impulse = DVec3::from_array(mount.direction_body) * pulse.impulse_ns;
                let mount_position = DVec3::from_array(mount.position_body_m);
                rcs_force_body_n += impulse / FLIGHT_STEP_S;
                rcs_moment_body_nm += mount_position.cross(impulse) / FLIGHT_STEP_S;
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_rcs_pulse(
                        mount,
                        pulse,
                        FLIGHT_STEP_S,
                        None,
                    )
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?,
                );
            }

            let mut power_command = self.electrical_power_command.clone();
            power_command.dt_s = FLIGHT_STEP_S;
            power_command.auxiliary_generation_power_w = apu_step.generated_electrical_power_w;
            for (fraction, scale) in power_command
                .fuel_cell_power_fraction
                .iter_mut()
                .zip(&fuel_cell_scales)
            {
                *fraction *= *scale;
            }

            // Mounted electrical propulsion declares its real operating-point
            // demand to the common load allocator under the implicit
            // same-name consumer convention. Generic consumer requests remain
            // available for everything else on the vessel.
            for (index, mount) in self.vehicle.electric_thrusters.iter().enumerate() {
                let estimate = mount
                    .operating_point(thessa_sim_core::ElectricThrusterCommand {
                        available_power_w: mount.engine.maximum_power_w,
                        requested_mass_flow_kg_s: self.electric_thruster_requested_flow_kg_s[index]
                            * electric_thruster_flow_scales[index],
                    })
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                set_named_power_load_request(
                    &self.vehicle,
                    &mut power_command,
                    &mount.name,
                    estimate.electrical_power_w,
                );
            }
            for (index, mount) in self.vehicle.fusion_torches.iter().enumerate() {
                set_named_power_load_request(
                    &self.vehicle,
                    &mut power_command,
                    &mount.name,
                    self.fusion_torch_commands[index].available_driver_power_w
                        * fusion_torch_scales[index],
                );
            }
            for (mount, requested) in self
                .vehicle
                .pulsed_fusion_systems
                .iter()
                .zip(&self.pulsed_fusion_commands)
            {
                set_named_power_load_request(
                    &self.vehicle,
                    &mut power_command,
                    &mount.name,
                    requested.available_charge_power_w,
                );
            }
            let mut electric_propeller_requested_power_w =
                vec![0.0; self.vehicle.propeller_drives.len()];
            for (index, (mount, requested)) in self
                .vehicle
                .propeller_drives
                .iter()
                .zip(&self.propeller_drive_commands)
                .enumerate()
            {
                if matches!(
                    &mount.drive.source,
                    thessa_sim_core::CompiledShaftPowerSource::Electric(_)
                ) {
                    let estimate = mount
                        .drive
                        .operating_point(
                            condition,
                            thessa_sim_core::PropellerDriveCommand {
                                throttle: requested.throttle * propeller_drive_scales[index],
                                source_rpm: requested.source_rpm,
                            },
                        )
                        .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                    electric_propeller_requested_power_w[index] = estimate.electrical_power_w;
                    set_named_power_load_request(
                        &self.vehicle,
                        &mut power_command,
                        &mount.name,
                        estimate.electrical_power_w,
                    );
                }
            }
            let (electrical_power_state, electrical_power_telemetry) = self
                .vehicle
                .electrical_power
                .advance(&self.electrical_power_state, &power_command)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            for (cell, output) in self
                .vehicle
                .electrical_power
                .fuel_cells
                .iter()
                .zip(&electrical_power_telemetry.fuel_cells)
            {
                let mut hydrogen = thessa_sim_core::VehicleResourceDemand::new(
                    cell.name.clone(),
                    thessa_sim_core::StoredPropellant::LiquidHydrogen,
                    output.hydrogen_flow_kg_s,
                );
                let mut oxygen = thessa_sim_core::VehicleResourceDemand::new(
                    cell.name.clone(),
                    thessa_sim_core::StoredPropellant::Lox,
                    output.oxygen_flow_kg_s,
                );
                if let Some(port) = &cell.feed_port_name {
                    hydrogen.feed_port_name = Some(port.clone());
                    oxygen.feed_port_name = Some(port.clone());
                }
                additional_demands.extend([hydrogen, oxygen]);
            }

            let requested_electric_flows: Vec<_> = self
                .electric_thruster_requested_flow_kg_s
                .iter()
                .zip(&electric_thruster_flow_scales)
                .map(|(requested, scale)| requested * scale)
                .collect();
            let electric_commands = self
                .vehicle
                .electric_thruster_commands_from_bus(
                    &electrical_power_telemetry,
                    &requested_electric_flows,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            let ((electric_force, electric_moment), electric_points) = self
                .vehicle
                .electric_thrusters_wrench_body_n(&electric_commands)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            accessory_force_body_n += electric_force;
            accessory_moment_body_nm += electric_moment;
            for (mount, point) in self.vehicle.electric_thrusters.iter().zip(&electric_points) {
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_electric_thruster_point(
                        mount, point, None,
                    ),
                );
            }

            let fusion_commands: Vec<_> = self
                .vehicle
                .fusion_torches
                .iter()
                .zip(&self.fusion_torch_commands)
                .zip(&fusion_torch_scales)
                .map(|((mount, requested), scale)| {
                    let available_bus_w = electrical_power_telemetry
                        .supplied_power_w(&mount.name)
                        .unwrap_or(0.0);
                    thessa_sim_core::FusionTorchCommand {
                        available_driver_power_w: available_bus_w,
                        requested_working_flow_kg_s: requested.requested_working_flow_kg_s * scale,
                    }
                })
                .collect();
            let ((fusion_force, fusion_moment), fusion_points) = self
                .vehicle
                .fusion_torches_wrench_body_n(&fusion_commands)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            accessory_force_body_n += fusion_force;
            accessory_moment_body_nm += fusion_moment;
            for (mount, point) in self.vehicle.fusion_torches.iter().zip(&fusion_points) {
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_fusion_torch_point(
                        mount, point, None,
                    ),
                );
            }

            let pulsed_commands: Vec<_> = self
                .vehicle
                .pulsed_fusion_systems
                .iter()
                .zip(&self.pulsed_fusion_commands)
                .zip(&pulsed_fusion_allowed)
                .map(|((mount, requested), allowed)| {
                    let available_bus_w = electrical_power_telemetry
                        .supplied_power_w(&mount.name)
                        .unwrap_or(0.0);
                    thessa_sim_core::PulsedFusionCommand {
                        available_charge_power_w: available_bus_w,
                        armed: requested.armed && *allowed,
                    }
                })
                .collect();
            let ((pulsed_force, pulsed_moment), pulsed_points) = self
                .vehicle
                .pulsed_fusion_wrench_body_n_stateful(
                    &self
                        .vehicle
                        .pulsed_fusion_systems
                        .iter()
                        .enumerate()
                        .map(|(index, _)| {
                            (self.pulsed_fusion_states[index], pulsed_commands[index])
                        })
                        .collect::<Vec<_>>(),
                    FLIGHT_STEP_S,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            accessory_force_body_n += pulsed_force;
            accessory_moment_body_nm += pulsed_moment;
            let next_pulsed_fusion_states: Vec<_> =
                pulsed_points.iter().map(|(state, _)| *state).collect();
            for (mount, (_, point)) in self
                .vehicle
                .pulsed_fusion_systems
                .iter()
                .zip(&pulsed_points)
            {
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_pulsed_fusion_point(
                        mount, point, None,
                    ),
                );
            }

            let propeller_commands: Vec<_> = self
                .propeller_drive_commands
                .iter()
                .zip(&self.vehicle.propeller_drives)
                .zip(&propeller_drive_scales)
                .enumerate()
                .map(|(index, ((requested, mount), scale))| {
                    let bus_scale = if matches!(
                        &mount.drive.source,
                        thessa_sim_core::CompiledShaftPowerSource::Electric(_)
                    ) {
                        delivered_power_fraction(
                            &electrical_power_telemetry,
                            &mount.name,
                            electric_propeller_requested_power_w[index],
                        )
                    } else {
                        1.0
                    };
                    thessa_sim_core::PropellerDriveCommand {
                        throttle: requested.throttle * scale * bus_scale,
                        source_rpm: requested.source_rpm,
                    }
                })
                .collect();
            let ((propeller_force, propeller_moment), propeller_points) = self
                .vehicle
                .propeller_drives_wrench_body_n(&propeller_commands, condition)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            accessory_force_body_n += propeller_force;
            accessory_moment_body_nm += propeller_moment;
            for (mount, point) in self.vehicle.propeller_drives.iter().zip(&propeller_points) {
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_propeller_drive_point(
                        mount, point, None,
                    ),
                );
            }

            let mut next_turboprop_commands = Vec::with_capacity(self.vehicle.turboprops.len());
            for (index, (mount, previous)) in self
                .vehicle
                .turboprops
                .iter()
                .zip(&turboprop_commands)
                .enumerate()
            {
                let mut command = *previous;
                command.dt_s = FLIGHT_STEP_S;
                command.shaft.throttle *= turboprop_scales[index];
                command.pneumatic_starter_power_w = if total_pneumatic_starter_request_w > 0.0 {
                    apu_step.pneumatic_bleed_power_w * turboprop_starter_requests[index]
                        / total_pneumatic_starter_request_w
                } else {
                    0.0
                };
                let (shaft_state, point) = mount
                    .drive
                    .advance(condition, &command)
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                let thrust = DVec3::from_array(mount.thrust_axis_body) * point.total_thrust_n;
                accessory_force_body_n += thrust;
                accessory_moment_body_nm += DVec3::from_array(mount.position_body_m).cross(thrust);
                additional_demands.extend(
                    thessa_sim_core::VehicleResourceDemand::from_turboprop_point(
                        mount, &point, None,
                    ),
                );
                next_turboprop_commands.push(command.with_state(shaft_state));
            }

            let mut propulsion = self.plan_installed_propulsion(ambient_pa, &additional_demands)?;
            let mut changed = false;
            if let Some(allocation) = &mut propulsion {
                for consumer in &allocation.additional_resource_consumers {
                    if consumer.scale >= 1.0 - 1.0e-10 {
                        continue;
                    }
                    if let Some(index) = self
                        .vehicle
                        .auxiliary_power_units
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        apu_commands[index].throttle *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .jets
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        jet_throttle_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .electrical_power
                        .fuel_cells
                        .iter()
                        .position(|cell| cell.name == consumer.consumer_name)
                    {
                        fuel_cell_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .rcs_mounts
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        rcs_duty_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .electric_thrusters
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        electric_thruster_flow_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .fusion_torches
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        fusion_torch_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .pulsed_fusion_systems
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        pulsed_fusion_allowed[index] = false;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .propeller_drives
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        propeller_drive_scales[index] *= consumer.scale;
                        changed = true;
                    } else if let Some(index) = self
                        .vehicle
                        .turboprops
                        .iter()
                        .position(|mount| mount.name == consumer.consumer_name)
                    {
                        turboprop_scales[index] *= consumer.scale;
                        changed = true;
                    } else {
                        return Err(FlightError::InvalidInput(format!(
                            "no operating-point feedback path for resource consumer '{}'",
                            consumer.consumer_name
                        )));
                    }
                    resource_limited = true;
                }
                allocation.fuel_limited |= resource_limited;
            }
            if changed {
                continue;
            }
            resource_limited |= apu_step.resource_limited;
            if let Some(allocation) = &mut propulsion {
                allocation.fuel_limited |= resource_limited;
            }
            return Ok(InstalledResourceStep {
                propulsion,
                electrical_power_state,
                electrical_power_telemetry,
                auxiliary_power_unit_states: apu_step.next_states,
                jet_commands: next_jet_commands,
                pulsed_fusion_states: next_pulsed_fusion_states,
                turboprop_commands: next_turboprop_commands,
                accessory_force_body_n,
                accessory_moment_body_nm,
                rcs_force_body_n,
                rcs_moment_body_nm,
                resource_limited,
            });
        }
        Err(FlightError::InvalidInput(
            "installed resource-limited operating points did not converge".into(),
        ))
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
        let mut jet_moment = control_allocation.moment_body_nm;
        let precomputed_aero = control_allocation.aero_result;
        let legacy_rcs_force_body_n = if self.vehicle.rcs_mounts.is_empty() {
            self.allocate_explicit_force()
        } else {
            DVec3::ZERO
        };
        // Only an exactly empty sampled medium permits zero force; the
        // vacuum cutoff guarantees exactness below its threshold.
        let vacuum = density_kg_m3 == 0.0;
        // Upper band: reference-area drag replaces the panel loop, whose
        // cost dominates multi-vehicle steps while its output is deep
        // below the gravity/thrust scales. Moments stay RCS-only.
        let band = self.upper_band_drag(kinematics, density_kg_m3);
        let skip_aero = vacuum || band.is_some();
        let (band_drag_body_n, band_q_pa) = band.unwrap_or((DVec3::ZERO, 0.0));
        let propulsion_condition = thessa_sim_core::flight_condition(
            &atmosphere_sample,
            kinematics.air_velocity_body_mps.length(),
        )
        .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        let installed_resource_step = self.plan_installed_resource_step(
            atmosphere_sample.pressure_pa,
            &propulsion_condition,
            &control_allocation.rcs_duties,
        )?;
        let rcs_force_body_n = if self.vehicle.rcs_mounts.is_empty() {
            legacy_rcs_force_body_n
        } else {
            installed_resource_step.rcs_force_body_n
        };
        jet_moment += installed_resource_step.rcs_moment_body_nm;
        let propulsion_allocation = installed_resource_step.propulsion.as_ref();
        let propulsion_force_body_n = if let Some(allocation) = &propulsion_allocation {
            jet_moment += allocation.moment_body_nm;
            allocation.force_body_n + installed_resource_step.accessory_force_body_n
        } else {
            DVec3::X * self.thrust_n() + installed_resource_step.accessory_force_body_n
        };
        jet_moment += installed_resource_step.accessory_moment_body_nm;
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
                propulsion_force_body_n,
                band_drag_body_n,
                parachute_force_body_n,
                parachute_moment_body_nm,
                skip_aero,
                precomputed_aero,
            )?
        } else if self.regime == FlightRegime::Coast
            && propulsion_force_body_n == DVec3::ZERO
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
                    propulsion_force_body_n,
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
                propulsion_force_body_n,
                band_drag_body_n,
                parachute_force_body_n,
                parachute_moment_body_nm,
                skip_aero,
                precomputed_aero,
            )?
        };
        if let Some(allocation) = propulsion_allocation {
            let frame_shift = self
                .vehicle
                .commit_propulsion_step(&mut self.resource_state, allocation)
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            self.apply_resource_frame_shift(frame_shift, &mut next)?;
            self.last_propulsion_force_body_n =
                allocation.force_body_n + installed_resource_step.accessory_force_body_n;
            self.last_propellant_flow_kg_s = allocation.total_propellant_flow_kg_s;
            self.fuel_limited = allocation.fuel_limited || installed_resource_step.resource_limited;
        } else {
            self.last_propulsion_force_body_n = propulsion_force_body_n;
            self.last_propellant_flow_kg_s = 0.0;
            self.fuel_limited = installed_resource_step.resource_limited;
        }
        self.electrical_power_state = installed_resource_step.electrical_power_state;
        self.electrical_power_telemetry = Some(installed_resource_step.electrical_power_telemetry);
        self.auxiliary_power_unit_states = installed_resource_step.auxiliary_power_unit_states;
        self.jet_commands = installed_resource_step.jet_commands;
        self.pulsed_fusion_states = installed_resource_step.pulsed_fusion_states;
        self.turboprop_commands = installed_resource_step.turboprop_commands;
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
