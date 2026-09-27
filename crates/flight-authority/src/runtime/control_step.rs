//! Pilot control, typed guidance resolution, and propulsion actuation.

use super::*;

impl FlightAuthority {
    pub(super) fn reset_control_surfaces(&mut self) -> Result<(), FlightError> {
        self.control_input = DVec3::ZERO;
        self.surface_input = DVec3::ZERO;
        self.control_deflections_rad.fill(0.0);
        self.actuator_saturated = false;
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
        Ok(())
    }

    /// Submit normalized pilot-axis input. Physical surface angles change on
    /// the fixed-step path after allocation and actuator rate/load limits.
    pub fn command_controls(&mut self, pitch: f64, yaw: f64, roll: f64) {
        let input = DVec3::new(pitch, yaw, roll);
        if input.is_finite() && input.abs().max_element() <= 1.0 {
            self.control_input = input;
        }
    }

    /// Apply a legacy pilot/compatibility command with immediate actuator
    /// semantics. This is used by the old wire representation and by tests
    /// that directly exercise the original X-15 path.
    pub fn set_legacy_propulsion(&mut self, throttle: f64, active: bool) {
        self.throttle = if throttle.is_finite() {
            throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
        self.propulsion_target = self.throttle;
        self.propulsion_actual = self.throttle;
        self.propulsion_dynamics_active = false;
        self.engine_active = active;
    }

    /// Set a typed/autopilot propulsion target. Allocation produces this
    /// target; the fixed-step loop advances the physical command below it.
    pub fn set_propulsion_target(
        &mut self,
        propulsion: PropulsionDemand,
    ) -> Result<(), FlightError> {
        if !propulsion.normalized.is_finite() || !(0.0..=1.0).contains(&propulsion.normalized) {
            return Err(FlightError::InvalidInput(
                "starter propulsion actuator accepts nominal demand in [0, 1]".into(),
            ));
        }
        self.throttle = propulsion.normalized;
        self.propulsion_target = propulsion.normalized;
        self.propulsion_dynamics_active = true;
        if propulsion.normalized > 0.0 {
            self.engine_active = true;
        }
        Ok(())
    }

    /// Safety cutoff for cancellation, contact, and manual takeover. A hard
    /// cutoff is intentional here: a disabled engine must not keep producing
    /// force while the high-level owner is being replaced.
    pub fn stop_propulsion(&mut self) {
        self.throttle = 0.0;
        self.propulsion_target = 0.0;
        self.propulsion_actual = 0.0;
        self.propulsion_dynamics_active = false;
        self.engine_active = false;
    }

    pub(super) fn advance_propulsion_actuator(&mut self) -> Result<(), FlightError> {
        if !self.propulsion_dynamics_active {
            return Ok(());
        }
        self.propulsion_actual = self
            .propulsion_dynamics
            .advance(
                self.propulsion_actual,
                self.propulsion_target,
                FLIGHT_STEP_S,
            )
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        self.engine_active = self.propulsion_actual > 1.0e-8 || self.propulsion_target > 0.0;
        Ok(())
    }

    pub(super) fn direction_to_inertial(
        &self,
        ephemeris: &BakedEphemeris,
        target: DirectionTarget,
        roll_policy: RollPolicy,
    ) -> Result<(DVec3, DVec3), FlightError> {
        let body_direction = match target.frame {
            DirectionFrame::Body => self.state.orientation_body_to_inertial * target.direction,
            DirectionFrame::Inertial => target.direction,
            DirectionFrame::Surface => {
                let body = ephemeris
                    .body_state(self.reference_body, SimTime(self.flight_time_s))
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                let up = normalize_direction(
                    self.state.position_inertial_m - body.position_inertial,
                    "surface up",
                )?;
                let (east, north) = surface_tangent_basis(up)?;
                east * target.direction.x + north * target.direction.y + up * target.direction.z
            }
            DirectionFrame::Orbit => {
                let body = ephemeris
                    .body_state(self.reference_body, SimTime(self.flight_time_s))
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                let radial = normalize_direction(
                    self.state.position_inertial_m - body.position_inertial,
                    "orbit radial",
                )?;
                let prograde = normalize_direction(
                    self.state.velocity_inertial_mps - body.velocity_inertial,
                    "orbit prograde",
                )?;
                let normal = normalize_direction(radial.cross(prograde), "orbit normal")?;
                prograde * target.direction.x
                    + normal * target.direction.y
                    + radial * target.direction.z
            }
            DirectionFrame::Target => {
                let target_body = target.target_body.ok_or_else(|| {
                    FlightError::InvalidInput(
                        "target-frame guidance requires a resolved target body".into(),
                    )
                })?;
                let body = ephemeris
                    .body_state(BodyId(target_body), SimTime(self.flight_time_s))
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                body.orientation * target.direction
            }
        };
        let forward = normalize_direction(body_direction, "guidance direction")?;
        let current_up = normalize_direction(
            self.state.orientation_body_to_inertial * DVec3::Z,
            "current up",
        )?;
        let roll_up = match target.frame {
            DirectionFrame::Surface => {
                let body = ephemeris
                    .body_state(self.reference_body, SimTime(self.flight_time_s))
                    .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
                normalize_direction(
                    self.state.position_inertial_m - body.position_inertial,
                    "surface up",
                )?
            }
            _ => current_up,
        };
        let mut up_reference = match roll_policy {
            RollPolicy::Fixed => roll_up,
            RollPolicy::Free | RollPolicy::Hold => current_up,
        };
        if up_reference.cross(forward).length_squared() <= 1.0e-12 {
            up_reference = if forward.z.abs() < 0.9 {
                DVec3::Z
            } else {
                DVec3::Y
            };
        }
        let side = normalize_direction(up_reference.cross(forward), "guidance roll basis")?;
        let up = normalize_direction(forward.cross(side), "guidance up basis")?;
        Ok((forward, up))
    }

    pub(super) fn direction_target_orientation(
        &self,
        ephemeris: &BakedEphemeris,
        target: DirectionTarget,
        roll_policy: RollPolicy,
    ) -> Result<DQuat, FlightError> {
        let (forward, up) = self.direction_to_inertial(ephemeris, target, roll_policy)?;
        let side = normalize_direction(up.cross(forward), "guidance side basis")?;
        Ok(DQuat::from_mat3(&DMat3::from_cols(forward, side, up)))
    }

    /// Apply a typed guidance intent through the compatibility control path.
    /// The returned mode is an execution detail for the legacy stepper; no
    /// caller gets mutable access to physics or actuator state.
    pub fn apply_guidance_intent(
        &mut self,
        ephemeris: &BakedEphemeris,
        intent: &GuidanceIntent,
    ) -> Result<ControlMode, FlightError> {
        intent
            .validate()
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        match intent {
            GuidanceIntent::ManualAxes(axes) => {
                self.control_input = DVec3::new(axes.pitch, axes.yaw, axes.roll)
                    .clamp(DVec3::splat(-1.0), DVec3::ONE);
                self.sas_enabled = false;
                Ok(ControlMode::Direct)
            }
            GuidanceIntent::AngularRate { rate_body_rps } => {
                // The compatibility rate law consumes normalized control
                // axes and applies its native 0.16 rad/s scale. Preserve the
                // requested body-rate signs through the inverse mapping and
                // clamp only at the physical legacy input boundary.
                let input = DVec3::new(
                    -rate_body_rps.y / PILOT_ATTITUDE_COMMAND_RATE_RAD_S,
                    -rate_body_rps.z / PILOT_ATTITUDE_COMMAND_RATE_RAD_S,
                    rate_body_rps.x / PILOT_ATTITUDE_COMMAND_RATE_RAD_S,
                );
                self.control_input = input.clamp(DVec3::splat(-1.0), DVec3::ONE);
                self.sas_enabled = false;
                Ok(ControlMode::Rate)
            }
            GuidanceIntent::Attitude {
                target_body_to_inertial,
                ..
            } => {
                self.sas_target_orientation = *target_body_to_inertial;
                self.control_input = DVec3::ZERO;
                self.sas_enabled = true;
                Ok(ControlMode::Navball)
            }
            GuidanceIntent::VelocityDirection {
                direction,
                roll_policy,
            } => {
                self.sas_target_orientation = self.direction_target_orientation(
                    ephemeris,
                    DirectionTarget {
                        direction: direction.direction,
                        frame: direction.frame,
                        target_body: direction.target_body,
                    },
                    *roll_policy,
                )?;
                self.control_input = DVec3::ZERO;
                self.sas_enabled = true;
                Ok(ControlMode::Navball)
            }
            GuidanceIntent::FlightPath { target } => {
                self.sas_target_orientation = self.direction_target_orientation(
                    ephemeris,
                    DirectionTarget {
                        direction: target.direction.direction,
                        frame: target.direction.frame,
                        target_body: target.direction.target_body,
                    },
                    target.roll_policy,
                )?;
                self.control_input = DVec3::ZERO;
                self.sas_enabled = true;
                Ok(ControlMode::Navball)
            }
            GuidanceIntent::Trajectory { .. } => Err(FlightError::InvalidInput(
                "trajectory guidance requires an active plan runner".into(),
            )),
        }
    }

    /// Execute one typed-guidance quantum. Starter propulsion currently
    /// realizes nominal demand only; reverse/augmentation are validated by
    /// the policy/vehicle layer before they can reach this physical model.
    pub fn advance_guidance(
        &mut self,
        ephemeris: &BakedEphemeris,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
        elapsed_s: f64,
    ) -> Result<(), FlightError> {
        self.advance_guidance_with_budget(ephemeris, intent, propulsion, elapsed_s, None)
    }

    /// Budgeted typed-guidance entry point used by the server driver. It keeps
    /// the same cooperative pacing guarantees as legacy `ControlMode` input.
    pub fn advance_guidance_with_budget(
        &mut self,
        ephemeris: &BakedEphemeris,
        intent: &GuidanceIntent,
        propulsion: PropulsionDemand,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), FlightError> {
        let mode = self.apply_guidance_intent(ephemeris, intent)?;
        self.set_propulsion_target(propulsion)?;
        let state_dependent = matches!(
            intent,
            GuidanceIntent::VelocityDirection {
                direction: DirectionTarget {
                    frame: DirectionFrame::Surface | DirectionFrame::Orbit | DirectionFrame::Target,
                    ..
                },
                ..
            } | GuidanceIntent::FlightPath {
                target: thessa_flight_control::FlightPathTarget {
                    direction: DirectionTarget {
                        frame: DirectionFrame::Surface
                            | DirectionFrame::Orbit
                            | DirectionFrame::Target,
                        ..
                    },
                    ..
                },
            }
        );
        self.guidance_state_dependent = state_dependent;
        let translation_force = match intent {
            GuidanceIntent::ManualAxes(axes) => {
                axes.translation * SpacecraftControlLaw::default().max_translation_force_n
            }
            _ => DVec3::ZERO,
        };
        self.explicit_force_demand_body_n =
            (translation_force.length_squared() > 1.0e-24).then_some(translation_force);
        let result =
            self.advance_with_budget_hook(ephemeris, mode, elapsed_s, budget, |authority| {
                if state_dependent {
                    authority.apply_guidance_intent(ephemeris, intent)?;
                }
                Ok(())
            });
        self.guidance_state_dependent = false;
        self.explicit_force_demand_body_n = None;
        result
    }

    /// Advance a declarative wrench through the native actuator path. The
    /// starter vehicle realizes translation through bounded paired RCS force
    /// effectors while propulsion remains the physical axial effector.
    /// Moments use the existing aerodynamic trim plus RCS residual allocator
    /// and retain its saturation semantics.
    pub fn advance_control_demand_with_budget(
        &mut self,
        ephemeris: &BakedEphemeris,
        demand: ControlDemand,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), FlightError> {
        demand
            .validate()
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        self.set_propulsion_target(demand.propulsion)?;
        if demand.force_body_n.length_squared() > 1.0e-24
            || demand.moment_body_nm.length_squared() > 1.0e-24
        {
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
        }
        self.explicit_force_demand_body_n = Some(demand.force_body_n);
        self.explicit_moment_demand_nm = Some(demand.moment_body_nm);
        let result = self.advance_with_budget(ephemeris, ControlMode::Direct, elapsed_s, budget);
        self.explicit_force_demand_body_n = None;
        self.explicit_moment_demand_nm = None;
        result
    }

    pub fn thrust_n(&self) -> f64 {
        if self.propulsion_dynamics_active {
            self.propulsion_actual * 254_000.0
        } else if self.engine_active {
            self.throttle * 254_000.0
        } else {
            0.0
        }
    }
}
