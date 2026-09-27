//! Installed wheel, landing-gear, and parachute state transitions.

use super::*;

impl FlightAuthority {
    pub(super) fn sync_wheel_runtime_state(&mut self) {
        self.wheel_spin_rad_s
            .resize_with(self.vehicle.wheel_chassis.len(), Vec::new);
        self.wheel_brake_states
            .resize_with(self.vehicle.wheel_chassis.len(), Vec::new);
        for (index, chassis) in self.vehicle.wheel_chassis.iter().enumerate() {
            let station_count = chassis.wheel_stations.len();
            self.wheel_spin_rad_s[index].resize(station_count, 0.0);
            self.wheel_brake_states[index].resize(station_count, WheelBrakeState::default());
        }
    }

    pub(super) fn sync_reaction_wheel_runtime_state(&mut self) {
        self.reaction_wheel_bank_enabled.resize(
            self.vehicle.reaction_wheels.len(),
            self.reaction_wheels_enabled,
        );
        self.reaction_wheel_bank_enabled
            .truncate(self.vehicle.reaction_wheels.len());
    }

    pub(super) fn sync_gear_deployment_commands(&mut self) {
        self.wheel_chassis_deployment_commands
            .resize(self.vehicle.wheel_chassis.len(), self.gear_down);
        self.wheel_chassis_deployment_commands
            .truncate(self.vehicle.wheel_chassis.len());
        self.landing_leg_deployment_commands
            .resize(self.vehicle.landing_legs.len(), self.gear_down);
        self.landing_leg_deployment_commands
            .truncate(self.vehicle.landing_legs.len());
    }

    pub(super) fn sync_landing_leg_runtime_state(&mut self) {
        self.landing_leg_states
            .truncate(self.vehicle.landing_legs.len());
        while self.landing_leg_states.len() < self.vehicle.landing_legs.len() {
            let index = self.landing_leg_states.len();
            self.landing_leg_states
                .push(self.vehicle.landing_legs[index].spec.initial_state());
        }
    }

    pub(super) fn sync_parachute_runtime_state(&mut self) {
        self.parachute_states
            .resize(self.vehicle.parachutes.len(), ParachuteState::default());
        self.last_parachute_loads
            .resize(self.vehicle.parachutes.len(), ParachuteLoad::default());
    }

    pub(super) fn parachute_rails_ineligible(&self) -> bool {
        self.parachute_states.iter().any(|state| {
            matches!(
                state.phase,
                ParachutePhase::Armed | ParachutePhase::Reefed | ParachutePhase::Deployed
            )
        })
    }

    pub(super) fn advance_parachutes(
        &mut self,
        kinematics: LocalAirKinematics,
        atmosphere: thessa_sim_core::AtmosphereSample,
    ) -> Result<(DVec3, DVec3), FlightError> {
        if self.vehicle.parachutes.is_empty() {
            self.last_parachute_loads.clear();
            self.parachute_states.clear();
            return Ok((DVec3::ZERO, DVec3::ZERO));
        }
        self.sync_parachute_runtime_state();
        let mut total_force_body_n = DVec3::ZERO;
        let mut total_moment_body_nm = DVec3::ZERO;
        for index in 0..self.vehicle.parachutes.len() {
            let spec = &self.vehicle.parachutes[index];
            let load = spec
                .advance(
                    self.parachute_states[index],
                    ParachuteEnvironment {
                        atmosphere,
                        radial_velocity_mps: kinematics
                            .relative_velocity_inertial_mps
                            .dot(kinematics.radial_up),
                        center_of_mass_air_velocity_body_mps: kinematics.air_velocity_body_mps,
                        angular_velocity_body_rps: self.state.angular_velocity_body_rps,
                        dt_s: FLIGHT_STEP_S,
                    },
                )
                .map_err(|error| {
                    FlightError::InvalidInput(format!(
                        "parachute '{}' could not advance: {error}",
                        spec.name
                    ))
                })?;
            self.parachute_states[index] = load.state;
            self.last_parachute_loads[index] = load;
            total_force_body_n += load.force_body_n;
            total_moment_body_nm += load.moment_body_nm;
        }
        Ok((total_force_body_n, total_moment_body_nm))
    }

    pub(super) fn sync_wheel_gear_runtime_state(&mut self) {
        self.wheel_chassis_states
            .truncate(self.vehicle.wheel_chassis.len());
        while self.wheel_chassis_states.len() < self.vehicle.wheel_chassis.len() {
            let index = self.wheel_chassis_states.len();
            let chassis = &self.vehicle.wheel_chassis[index];
            self.wheel_chassis_states.push(
                chassis
                    .spec
                    .retraction
                    .map(|retraction| retraction.initial_state())
                    .unwrap_or_else(WheelChassisState::deployed),
            );
        }
    }

    pub(super) fn landing_gear_transitioning(&self) -> bool {
        self.landing_leg_states
            .iter()
            .enumerate()
            .any(|(index, state)| {
                let deployed = self
                    .landing_leg_deployment_commands
                    .get(index)
                    .copied()
                    .unwrap_or(self.gear_down);
                let target = if deployed { 1.0 } else { 0.0 };
                (state.deployment_fraction - target).abs() > 1.0e-12
            })
            || self
                .vehicle
                .wheel_chassis
                .iter()
                .zip(&self.wheel_chassis_states)
                .enumerate()
                .any(|(index, (chassis, state))| {
                    let deployed = self
                        .wheel_chassis_deployment_commands
                        .get(index)
                        .copied()
                        .unwrap_or(self.gear_down);
                    let target = if deployed { 1.0 } else { 0.0 };
                    chassis.spec.retraction.is_some()
                        && (state.deployment_fraction - target).abs() > 1.0e-12
                })
    }

    pub(super) fn advance_freeflight_landing_gear(&mut self) -> Result<(), FlightError> {
        if self.vehicle.landing_legs.is_empty()
            && self
                .vehicle
                .wheel_chassis
                .iter()
                .all(|chassis| chassis.spec.retraction.is_none())
        {
            return Ok(());
        }
        self.sync_gear_deployment_commands();
        self.sync_landing_leg_runtime_state();
        self.sync_wheel_gear_runtime_state();
        self.last_landing_leg_actuators.clear();
        self.last_wheel_gear_actuators.clear();
        for (index, leg) in self.vehicle.landing_legs.iter().enumerate() {
            let (state, point) = leg
                .spec
                .advance_deployment(
                    self.landing_leg_states[index],
                    self.landing_leg_deployment_commands[index],
                    FLIGHT_STEP_S,
                    0.0,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            self.landing_leg_states[index] = state;
            self.last_landing_leg_actuators.push((index, point));
        }
        for (index, chassis) in self.vehicle.wheel_chassis.iter().enumerate() {
            let Some(retraction) = chassis.spec.retraction else {
                self.wheel_chassis_states[index] = WheelChassisState::deployed();
                continue;
            };
            let (state, point) = retraction
                .advance_deployment(
                    self.wheel_chassis_states[index],
                    self.wheel_chassis_deployment_commands[index],
                    FLIGHT_STEP_S,
                    0.0,
                )
                .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
            self.wheel_chassis_states[index] = state;
            self.last_wheel_gear_actuators.push((index, point));
        }
        Ok(())
    }
}
