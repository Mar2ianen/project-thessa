//! Installed propellant inventory, feed allocation, and moving-mass updates.
//!
//! Resource connectivity remains the assembly graph's simple crossfeed
//! contract. This module allocates real engine mass flow over reachable tank
//! inventory; it deliberately does not add a fluid-network solver.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};

use crate::{
    CompiledEngine, Propellant, RigidBodyProperties, StoredPropellant, TankResource,
    VehicleDefinition, VehicleError,
};

/// Live, server-authoritative resource state for one compiled vehicle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VehicleResourceState {
    /// Remaining mass in `VehicleDefinition::tanks` order.
    pub tank_propellant_kg: Vec<f64>,
    /// Solid-motor burn clocks in `VehicleDefinition::engines` order.
    pub solid_burn_time_s: Vec<f64>,
    /// Solid motors remain ignited until their compiled burn curve ends.
    pub solid_ignited: Vec<bool>,
}

impl VehicleResourceState {
    pub fn from_vehicle(vehicle: &VehicleDefinition) -> Self {
        Self {
            tank_propellant_kg: vehicle
                .tanks
                .iter()
                .map(|mount| mount.loaded_propellant_kg())
                .collect(),
            solid_burn_time_s: vec![0.0; vehicle.engines.len()],
            solid_ignited: vec![false; vehicle.engines.len()],
        }
    }

    pub fn total_tank_propellant_kg(&self) -> f64 {
        self.tank_propellant_kg.iter().sum()
    }
}

/// Planned physical propulsion for one fixed step. Applying the plan commits
/// its tank draw, solid burn clocks, and corresponding moving-mass properties.
#[derive(Debug, Clone, PartialEq)]
pub struct VehiclePropulsionAllocation {
    pub force_body_n: DVec3,
    pub moment_body_nm: DVec3,
    pub tank_consumption_kg: Vec<f64>,
    pub solid_burn_time_s: Vec<f64>,
    pub solid_ignited: Vec<bool>,
    pub engine_throttles: Vec<f64>,
    pub system_chamber_throttles: Vec<Vec<f64>>,
    pub total_propellant_flow_kg_s: f64,
    pub fuel_limited: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ComponentRole {
    Mixed,
    Oxidizer,
    Fuel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ResourceKey {
    Chemical(Propellant),
    Pure(StoredPropellant),
}

#[derive(Debug, Clone, Copy)]
struct FlowDemand {
    resource: ResourceKey,
    propellant: Propellant,
    flow_kg_s: f64,
    mixture_ratio: Option<f64>,
}

impl VehicleDefinition {
    /// Initialize mutable inventory from baked tank fills and unburned motors.
    pub fn initial_resource_state(&self) -> VehicleResourceState {
        VehicleResourceState::from_vehicle(self)
    }

    /// Read the live fill of one uniquely named tank.
    pub fn tank_propellant_kg(
        &self,
        state: &VehicleResourceState,
        tank_name: &str,
    ) -> Result<f64, VehicleError> {
        self.validate_resource_state(state)?;
        let index = self.unique_tank_index(tank_name)?;
        Ok(state.tank_propellant_kg[index])
    }

    /// Explicitly move an exact mass between compatible tanks. Transfer is
    /// instantaneous at the gameplay command boundary; no pump, pipe, or
    /// passive balancing behavior is inferred.
    pub fn transfer_propellant(
        &mut self,
        state: &mut VehicleResourceState,
        source_name: &str,
        destination_name: &str,
        mass_kg: f64,
    ) -> Result<DVec3, VehicleError> {
        self.validate_resource_state(state)?;
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return Err(VehicleError::InvalidVehicle(
                "propellant transfer mass must be finite and > 0".into(),
            ));
        }
        let source = self.unique_tank_index(source_name)?;
        let destination = self.unique_tank_index(destination_name)?;
        if source == destination {
            return Err(VehicleError::InvalidVehicle(
                "propellant transfer needs two different tanks".into(),
            ));
        }
        if self.tanks[source].resource != self.tanks[destination].resource {
            return Err(VehicleError::InvalidVehicle(format!(
                "tanks '{source_name}' and '{destination_name}' store different resources"
            )));
        }
        if !self.tanks_share_resource_path(source, destination)? {
            return Err(VehicleError::InvalidVehicle(format!(
                "no open resource path between tanks '{source_name}' and '{destination_name}'"
            )));
        }
        let source_mass = state.tank_propellant_kg[source];
        let destination_mass = state.tank_propellant_kg[destination];
        let destination_capacity = self.tanks[destination].tank.full_propellant_kg;
        if source_mass + 1.0e-12 < mass_kg
            || destination_mass + mass_kg > destination_capacity + 1.0e-9
        {
            return Err(VehicleError::InvalidVehicle(format!(
                "propellant transfer of {mass_kg:.6} kg exceeds source inventory or destination capacity"
            )));
        }
        let mut next_tanks = state.tank_propellant_kg.clone();
        next_tanks[source] = (source_mass - mass_kg).max(0.0);
        next_tanks[destination] = (destination_mass + mass_kg).min(destination_capacity);
        self.commit_resource_state(
            state,
            next_tanks,
            state.solid_burn_time_s.clone(),
            state.solid_ignited.clone(),
        )
    }

    /// Allocate per-engine throttle requests against reachable propellant,
    /// returning the physical wrench and exact resource draw for one step.
    pub fn plan_propulsion_step(
        &self,
        state: &VehicleResourceState,
        engine_throttles: &[f64],
        system_chamber_throttles: &[Vec<f64>],
        ambient_pa: f64,
        dt_s: f64,
    ) -> Result<VehiclePropulsionAllocation, VehicleError> {
        self.validate_resource_state(state)?;
        if !ambient_pa.is_finite() || ambient_pa < 0.0 || !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(VehicleError::InvalidVehicle(
                "propulsion step needs finite ambient pressure >= 0 and dt > 0".into(),
            ));
        }
        if engine_throttles.len() != self.engines.len() {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} engine throttles, got {}",
                self.engines.len(),
                engine_throttles.len()
            )));
        }
        if system_chamber_throttles.len() != self.systems.len()
            || system_chamber_throttles
                .iter()
                .zip(&self.systems)
                .any(|(commands, mount)| commands.len() != mount.system.chambers.len())
        {
            return Err(VehicleError::InvalidVehicle(
                "system chamber throttle dimensions must match installed systems".into(),
            ));
        }
        if engine_throttles
            .iter()
            .chain(system_chamber_throttles.iter().flatten())
            .any(|throttle| !throttle.is_finite() || !(0.0..=1.0).contains(throttle))
        {
            return Err(VehicleError::InvalidVehicle(
                "installed engine throttles must be finite and in [0, 1]".into(),
            ));
        }

        let (actual_engines, actual_systems, tank_draw, group_scales) = self.allocate_throttles(
            state,
            engine_throttles,
            system_chamber_throttles,
            ambient_pa,
            dt_s,
        )?;
        let mut force = DVec3::ZERO;
        let mut moment = DVec3::ZERO;
        let mut solid_burn_time_s = state.solid_burn_time_s.clone();
        let mut solid_ignited = state.solid_ignited.clone();
        let mut solid_draw_kg = 0.0;
        let mut fuel_limited = group_scales.iter().any(|(_, scale)| *scale < 1.0 - 1.0e-10);

        for (index, mount) in self.engines.iter().enumerate() {
            match &mount.engine {
                CompiledEngine::Liquid(engine) => {
                    let throttle = actual_engines[index];
                    let point = mount
                        .engine
                        .operating_point(throttle, ambient_pa, 0.0)
                        .map_err(VehicleError::Propulsion)?;
                    let thrust = DVec3::from_array(mount.thrust_axis_body) * point.thrust_n;
                    force += thrust;
                    moment += DVec3::from_array(mount.position_body_m).cross(thrust);
                    fuel_limited |=
                        throttle + 1.0e-12 < engine.min_throttle && engine_throttles[index] > 0.0;
                }
                CompiledEngine::Solid(_) => {
                    let old_time = state.solid_burn_time_s[index];
                    let ignited = state.solid_ignited[index] || engine_throttles[index] > 0.0;
                    solid_ignited[index] = ignited;
                    let Some(duration) = solid_burn_duration(&mount.engine) else {
                        continue;
                    };
                    if !ignited || old_time >= duration {
                        continue;
                    }
                    let new_time = (old_time + dt_s).min(duration);
                    let active_s = new_time - old_time;
                    let midpoint = old_time + 0.5 * active_s;
                    let point = mount
                        .engine
                        .operating_point(1.0, ambient_pa, midpoint)
                        .map_err(VehicleError::Propulsion)?;
                    let thrust = DVec3::from_array(mount.thrust_axis_body)
                        * (point.thrust_n * active_s / dt_s);
                    force += thrust;
                    moment += DVec3::from_array(mount.position_body_m).cross(thrust);
                    let old_remaining = mount
                        .engine
                        .propellant_remaining_kg(old_time)
                        .unwrap_or(0.0);
                    let new_remaining = mount
                        .engine
                        .propellant_remaining_kg(new_time)
                        .unwrap_or(0.0);
                    solid_draw_kg += (old_remaining - new_remaining).max(0.0);
                    solid_burn_time_s[index] = new_time;
                }
            }
        }

        for (system_index, mount) in self.systems.iter().enumerate() {
            let (system_force, system_moment) = mount
                .system
                .wrench_body_n(&actual_systems[system_index], ambient_pa)
                .map_err(VehicleError::Propulsion)?;
            force += system_force;
            moment += system_moment;
            if actual_systems[system_index]
                .iter()
                .zip(&system_chamber_throttles[system_index])
                .any(|(actual, requested)| *actual + 1.0e-12 < *requested)
            {
                fuel_limited = true;
            }
        }
        if !force.is_finite() || !moment.is_finite() {
            return Err(VehicleError::InvalidVehicle(
                "propulsion allocation produced a non-finite wrench".into(),
            ));
        }
        let tank_draw_kg: f64 = tank_draw.iter().sum();
        Ok(VehiclePropulsionAllocation {
            force_body_n: force,
            moment_body_nm: moment,
            tank_consumption_kg: tank_draw,
            solid_burn_time_s,
            solid_ignited,
            engine_throttles: actual_engines,
            system_chamber_throttles: actual_systems,
            total_propellant_flow_kg_s: (tank_draw_kg + solid_draw_kg) / dt_s,
            fuel_limited,
        })
    }

    /// Commit an allocated tick and recenter baked geometry around the new
    /// center of mass. Returns the applied body-frame origin shift.
    pub fn commit_propulsion_step(
        &mut self,
        state: &mut VehicleResourceState,
        allocation: &VehiclePropulsionAllocation,
    ) -> Result<DVec3, VehicleError> {
        self.validate_resource_state(state)?;
        if allocation.tank_consumption_kg.len() != self.tanks.len()
            || allocation.solid_burn_time_s.len() != self.engines.len()
            || allocation.solid_ignited.len() != self.engines.len()
        {
            return Err(VehicleError::InvalidVehicle(
                "propulsion allocation does not match the installed resource state".into(),
            ));
        }
        let mut next_tanks = state.tank_propellant_kg.clone();
        for (index, consumed) in allocation.tank_consumption_kg.iter().copied().enumerate() {
            if !consumed.is_finite() || consumed < 0.0 || consumed > next_tanks[index] + 1.0e-9 {
                return Err(VehicleError::InvalidVehicle(format!(
                    "propulsion allocation overdraws tank '{}'",
                    tank_display_name(&self.tanks[index])
                )));
            }
            next_tanks[index] = (next_tanks[index] - consumed).max(0.0);
        }
        self.commit_resource_state(
            state,
            next_tanks,
            allocation.solid_burn_time_s.clone(),
            allocation.solid_ignited.clone(),
        )
    }

    fn allocate_throttles(
        &self,
        state: &VehicleResourceState,
        engine_requests: &[f64],
        system_requests: &[Vec<f64>],
        ambient_pa: f64,
        dt_s: f64,
    ) -> Result<(Vec<f64>, Vec<Vec<f64>>, Vec<f64>, Vec<(ResourceKey, f64)>), VehicleError> {
        let mut engines = engine_requests.to_vec();
        let mut systems = system_requests.to_vec();
        let accessible = self.feedable_tanks()?;
        let max_floor_passes = self.engines.len()
            + self
                .systems
                .iter()
                .map(|system| system.system.chambers.len())
                .sum::<usize>()
            + 1;
        let mut scales = Vec::new();

        for _ in 0..max_floor_passes {
            let demands = self.flow_demands(&engines, &systems, ambient_pa)?;
            let (next_scales, _) = self.allocate_flow_demands(&demands, state, &accessible, dt_s);
            scales = next_scales;
            let mut removed = false;
            for (index, mount) in self.engines.iter().enumerate() {
                let CompiledEngine::Liquid(engine) = &mount.engine else {
                    continue;
                };
                let actual = engines[index] * group_scale(&scales, engine_resource_key(engine));
                if actual > 0.0 && actual + 1.0e-12 < engine.min_throttle {
                    engines[index] = 0.0;
                    removed = true;
                }
            }
            for (index, mount) in self.systems.iter().enumerate() {
                let scale = group_scale(&scales, ResourceKey::Chemical(mount.system.propellant));
                for throttle in &mut systems[index] {
                    if *throttle > 0.0 && *throttle * scale + 1.0e-12 < mount.system.min_throttle {
                        *throttle = 0.0;
                        removed = true;
                    }
                }
            }
            if !removed {
                break;
            }
        }

        for (index, mount) in self.engines.iter().enumerate() {
            if let CompiledEngine::Liquid(engine) = &mount.engine {
                engines[index] *= group_scale(&scales, engine_resource_key(engine));
            }
        }
        for (index, mount) in self.systems.iter().enumerate() {
            let scale = group_scale(&scales, ResourceKey::Chemical(mount.system.propellant));
            for throttle in &mut systems[index] {
                *throttle *= scale;
            }
        }

        // Rebuild the exact tank draw after applying the group throttle
        // scales. If this is not fully supplied, the output/flow plan differs
        // from the allocator's inventory contract and fails closed.
        let demands = self.flow_demands(&engines, &systems, ambient_pa)?;
        let (final_scales, final_draw) =
            self.allocate_flow_demands(&demands, state, &accessible, dt_s);
        if final_scales.iter().any(|(_, scale)| *scale < 1.0 - 1.0e-10) {
            return Err(VehicleError::InvalidVehicle(
                "propellant allocation changed after stable-throttle resolution".into(),
            ));
        }
        Ok((engines, systems, final_draw, scales))
    }

    fn allocate_flow_demands(
        &self,
        demands: &[FlowDemand],
        state: &VehicleResourceState,
        accessible: &[bool],
        dt_s: f64,
    ) -> (Vec<(ResourceKey, f64)>, Vec<f64>) {
        let mut propellants = Vec::new();
        for demand in demands {
            if !propellants.contains(&demand.resource) {
                propellants.push(demand.resource);
            }
        }
        let mut scales = Vec::with_capacity(propellants.len());
        let mut reserved = vec![0.0; self.tanks.len()];
        for resource in propellants {
            let group: Vec<_> = demands
                .iter()
                .copied()
                .filter(|demand| demand.resource == resource)
                .collect();
            let (demanded_kg, oxidizer_fraction, fuel_fraction) =
                group_resource_demand(&group, dt_s);
            let available: Vec<f64> = state
                .tank_propellant_kg
                .iter()
                .zip(&reserved)
                .map(|(mass, used)| (mass - used).max(0.0))
                .collect();
            let scale = resource_scale(
                &self.tanks,
                &available,
                accessible,
                resource,
                demanded_kg,
                oxidizer_fraction,
                fuel_fraction,
            );
            draw_resource(
                &self.tanks,
                &available,
                accessible,
                resource,
                demanded_kg * scale,
                oxidizer_fraction,
                fuel_fraction,
                &mut reserved,
            );
            scales.push((resource, scale));
        }
        (scales, reserved)
    }

    fn commit_resource_state(
        &mut self,
        state: &mut VehicleResourceState,
        next_tanks: Vec<f64>,
        next_solid_times: Vec<f64>,
        next_solid_ignited: Vec<bool>,
    ) -> Result<DVec3, VehicleError> {
        if next_tanks.len() != self.tanks.len()
            || next_solid_times.len() != self.engines.len()
            || next_solid_ignited.len() != self.engines.len()
        {
            return Err(VehicleError::InvalidVehicle(
                "next resource inventory does not match installed parts".into(),
            ));
        }
        for (mount, mass) in self.tanks.iter().zip(&next_tanks) {
            if !mass.is_finite() || *mass < 0.0 || *mass > mount.tank.full_propellant_kg + 1.0e-9 {
                return Err(VehicleError::InvalidVehicle(format!(
                    "tank '{}' inventory is outside capacity",
                    tank_display_name(mount)
                )));
            }
        }
        for (index, (mount, time)) in self.engines.iter().zip(&next_solid_times).enumerate() {
            if !time.is_finite() || *time < state.solid_burn_time_s[index] {
                return Err(VehicleError::InvalidVehicle(format!(
                    "engine '{}' solid burn clock must be finite and monotone",
                    mount.name
                )));
            }
            if let CompiledEngine::Solid(engine) = &mount.engine
                && *time > engine.burn_time_s + 1.0e-9
            {
                return Err(VehicleError::InvalidVehicle(format!(
                    "engine '{}' solid burn clock exceeds burn duration",
                    mount.name
                )));
            }
        }

        let mut mass_delta_kg = 0.0;
        let mut first_moment_delta = DVec3::ZERO;
        let mut inertia_delta = DMat3::ZERO;
        for (index, mount) in self.tanks.iter().enumerate() {
            let delta = next_tanks[index] - state.tank_propellant_kg[index];
            if delta == 0.0 {
                continue;
            }
            let position = DVec3::from_array(mount.position_body_m);
            mass_delta_kg += delta;
            first_moment_delta += position * delta;
            inertia_delta += parallel_axis(delta, position);
            if let Some(shape) = mount.tank.shape {
                let old_intrinsic = shape
                    .intrinsic_inertia_body_kg_m2(
                        mount.tank.dry_mass_kg,
                        state.tank_propellant_kg[index],
                    )
                    .map_err(VehicleError::Propulsion)?;
                let new_intrinsic = shape
                    .intrinsic_inertia_body_kg_m2(mount.tank.dry_mass_kg, next_tanks[index])
                    .map_err(VehicleError::Propulsion)?;
                inertia_delta += new_intrinsic - old_intrinsic;
            }
        }
        for (index, mount) in self.engines.iter().enumerate() {
            if !matches!(mount.engine, CompiledEngine::Solid(_)) {
                continue;
            }
            let old_remaining = mount
                .engine
                .propellant_remaining_kg(state.solid_burn_time_s[index])
                .unwrap_or(0.0);
            let new_remaining = mount
                .engine
                .propellant_remaining_kg(next_solid_times[index])
                .unwrap_or(0.0);
            let delta = new_remaining - old_remaining;
            if delta == 0.0 {
                continue;
            }
            let position = DVec3::from_array(mount.position_body_m);
            mass_delta_kg += delta;
            first_moment_delta += position * delta;
            inertia_delta += parallel_axis(delta, position);
        }

        let updated_mass_kg = self.mass_properties.mass_kg + mass_delta_kg;
        if !updated_mass_kg.is_finite() || updated_mass_kg <= 1.0e-9 {
            return Err(VehicleError::InvalidVehicle(
                "resource change produced non-positive vehicle mass".into(),
            ));
        }
        let center_shift = first_moment_delta / updated_mass_kg;
        let updated_inertia = self.mass_properties.inertia_body_kg_m2 + inertia_delta
            - parallel_axis(updated_mass_kg, center_shift);
        let properties = RigidBodyProperties::new(updated_mass_kg, updated_inertia)
            .map_err(VehicleError::MassProperties)?;
        let frame_shift = -center_shift;
        if !self.body_frame_shift_is_finite(frame_shift) {
            return Err(VehicleError::InvalidVehicle(
                "resource change would move body-frame coordinates out of range".into(),
            ));
        }

        self.shift_body_frame_origin(frame_shift);
        self.mass_properties = properties;
        state.tank_propellant_kg = next_tanks;
        state.solid_burn_time_s = next_solid_times;
        state.solid_ignited = next_solid_ignited;
        Ok(frame_shift)
    }

    fn validate_resource_state(&self, state: &VehicleResourceState) -> Result<(), VehicleError> {
        if state.tank_propellant_kg.len() != self.tanks.len()
            || state.solid_burn_time_s.len() != self.engines.len()
            || state.solid_ignited.len() != self.engines.len()
        {
            return Err(VehicleError::InvalidVehicle(
                "resource state dimensions must match the vehicle definition".into(),
            ));
        }
        for (mount, mass) in self.tanks.iter().zip(&state.tank_propellant_kg) {
            if !mass.is_finite() || *mass < 0.0 || *mass > mount.tank.full_propellant_kg + 1.0e-9 {
                return Err(VehicleError::InvalidVehicle(format!(
                    "tank '{}' inventory is outside capacity",
                    tank_display_name(mount)
                )));
            }
        }
        for (mount, time) in self.engines.iter().zip(&state.solid_burn_time_s) {
            if !time.is_finite() || *time < 0.0 {
                return Err(VehicleError::InvalidVehicle(format!(
                    "engine '{}' burn clock must be finite and non-negative",
                    mount.name
                )));
            }
            if let CompiledEngine::Solid(engine) = &mount.engine
                && *time > engine.burn_time_s + 1.0e-9
            {
                return Err(VehicleError::InvalidVehicle(format!(
                    "engine '{}' burn clock exceeds burn duration",
                    mount.name
                )));
            }
        }
        Ok(())
    }

    fn unique_tank_index(&self, name: &str) -> Result<usize, VehicleError> {
        let mut matching = self
            .tanks
            .iter()
            .enumerate()
            .filter(|(_, mount)| !mount.name.is_empty() && mount.name == name);
        let Some((index, _)) = matching.next() else {
            return Err(VehicleError::InvalidVehicle(format!(
                "vehicle has no addressable tank named '{name}'"
            )));
        };
        if matching.next().is_some() {
            return Err(VehicleError::InvalidVehicle(format!(
                "tank name '{name}' is ambiguous"
            )));
        }
        Ok(index)
    }

    fn tanks_share_resource_path(&self, a: usize, b: usize) -> Result<bool, VehicleError> {
        let Some(assembly) = &self.assembly else {
            return Ok(true);
        };
        let Some(endpoint_a) = assembly
            .tanks
            .iter()
            .find(|endpoint| endpoint.name == self.tanks[a].name)
        else {
            return Ok(true);
        };
        let Some(endpoint_b) = assembly
            .tanks
            .iter()
            .find(|endpoint| endpoint.name == self.tanks[b].name)
        else {
            return Ok(true);
        };
        let mut adjacency = vec![Vec::new(); assembly.body_names.len()];
        for link in &assembly.links {
            if link.state.resource_open() {
                adjacency[link.state.a].push(link.state.b);
                adjacency[link.state.b].push(link.state.a);
            }
        }
        let mut seen = vec![false; adjacency.len()];
        let mut stack = vec![endpoint_a.body];
        seen[endpoint_a.body] = true;
        while let Some(body) = stack.pop() {
            if body == endpoint_b.body {
                return Ok(true);
            }
            for neighbor in &adjacency[body] {
                if !seen[*neighbor] {
                    seen[*neighbor] = true;
                    stack.push(*neighbor);
                }
            }
        }
        Ok(false)
    }

    fn feedable_tanks(&self) -> Result<Vec<bool>, VehicleError> {
        let mut accessible = vec![true; self.tanks.len()];
        let Some(assembly) = &self.assembly else {
            return Ok(accessible);
        };
        let links: Vec<_> = assembly.links.iter().map(|link| link.state).collect();
        let tank_bodies: Vec<_> = assembly.tanks.iter().map(|tank| tank.body).collect();
        let port_bodies: Vec<_> = if assembly.engine_ports.is_empty() {
            // Legacy mounts are vehicle-level rather than attached to a
            // named procedural-body port. Treat them as mounted on the
            // assembly root so closed hatches still constrain crossfeed.
            vec![assembly.root_body]
        } else {
            assembly.engine_ports.iter().map(|port| port.body).collect()
        };
        let reachable = crate::feed_reachable(
            assembly.body_names.len(),
            &links,
            &tank_bodies,
            &port_bodies,
        )
        .map_err(|error| VehicleError::InvalidVehicle(error.to_string()))?;
        for (index, mount) in self.tanks.iter().enumerate() {
            if let Some(endpoint) = assembly
                .tanks
                .iter()
                .find(|endpoint| endpoint.name == mount.name)
            {
                accessible[index] = reachable
                    .iter()
                    .any(|(tank_body, _)| *tank_body == endpoint.body);
            }
        }
        Ok(accessible)
    }

    fn flow_demands(
        &self,
        engine_throttles: &[f64],
        system_throttles: &[Vec<f64>],
        ambient_pa: f64,
    ) -> Result<Vec<FlowDemand>, VehicleError> {
        let mut demands = Vec::new();
        for (mount, throttle) in self.engines.iter().zip(engine_throttles) {
            if let CompiledEngine::Liquid(engine) = &mount.engine {
                if *throttle == 0.0 {
                    continue;
                }
                let point = mount
                    .engine
                    .operating_point(*throttle, ambient_pa, 0.0)
                    .map_err(VehicleError::Propulsion)?;
                demands.push(FlowDemand {
                    resource: engine
                        .working_fluid
                        .map(ResourceKey::Pure)
                        .unwrap_or(ResourceKey::Chemical(engine.propellant)),
                    propellant: engine.propellant,
                    flow_kg_s: point.mass_flow_kg_s,
                    mixture_ratio: engine.mixture_ratio,
                });
            }
        }
        for (mount, throttles) in self.systems.iter().zip(system_throttles) {
            if throttles.iter().all(|throttle| *throttle == 0.0) {
                continue;
            }
            let point = mount
                .system
                .operating_point(throttles, ambient_pa)
                .map_err(VehicleError::Propulsion)?;
            demands.push(FlowDemand {
                resource: ResourceKey::Chemical(mount.system.propellant),
                propellant: mount.system.propellant,
                flow_kg_s: point.mass_flow_kg_s,
                mixture_ratio: mount.system.mixture_ratio,
            });
        }
        Ok(demands)
    }
}

fn solid_burn_duration(engine: &CompiledEngine) -> Option<f64> {
    match engine {
        CompiledEngine::Liquid(_) => None,
        CompiledEngine::Solid(engine) => Some(engine.burn_time_s),
    }
}

fn tank_display_name(mount: &crate::TankMount) -> &str {
    &mount.name
}

fn group_resource_demand(demands: &[FlowDemand], dt_s: f64) -> (f64, f64, f64) {
    let mut total_kg = 0.0;
    let mut oxidizer_kg = 0.0;
    let mut fuel_kg = 0.0;
    for demand in demands {
        let amount = demand.flow_kg_s * dt_s;
        total_kg += amount;
        if matches!(demand.resource, ResourceKey::Chemical(_))
            && let Some(ratio) = demand
                .mixture_ratio
                .or_else(|| demand.propellant.reference_mixture_ratio())
        {
            oxidizer_kg += amount * ratio / (1.0 + ratio);
            fuel_kg += amount / (1.0 + ratio);
        }
    }
    let (oxidizer_fraction, fuel_fraction) = if total_kg > 0.0 {
        (oxidizer_kg / total_kg, fuel_kg / total_kg)
    } else {
        (0.0, 0.0)
    };
    (total_kg, oxidizer_fraction, fuel_fraction)
}

fn engine_resource_key(engine: &crate::CompiledLiquid) -> ResourceKey {
    engine
        .working_fluid
        .map(ResourceKey::Pure)
        .unwrap_or(ResourceKey::Chemical(engine.propellant))
}

fn group_scale(scales: &[(ResourceKey, f64)], resource: ResourceKey) -> f64 {
    scales
        .iter()
        .find_map(|(candidate, scale)| (*candidate == resource).then_some(*scale))
        .unwrap_or(1.0)
}

fn resource_scale(
    tanks: &[crate::TankMount],
    available: &[f64],
    accessible: &[bool],
    resource: ResourceKey,
    demanded_kg: f64,
    oxidizer_fraction: f64,
    fuel_fraction: f64,
) -> f64 {
    if demanded_kg <= 0.0 {
        return 1.0;
    }
    let mut mixed_kg = 0.0;
    let mut oxidizer_kg = 0.0;
    let mut fuel_kg = 0.0;
    for (index, mount) in tanks.iter().enumerate() {
        if !accessible[index] {
            continue;
        }
        match component_role(mount.resource, resource) {
            Some(ComponentRole::Mixed) => mixed_kg += available[index],
            Some(ComponentRole::Oxidizer) => oxidizer_kg += available[index],
            Some(ComponentRole::Fuel) => fuel_kg += available[index],
            None => {}
        }
    }
    let split_capacity = if oxidizer_fraction > 0.0 && fuel_fraction > 0.0 {
        (oxidizer_kg / oxidizer_fraction).min(fuel_kg / fuel_fraction)
    } else {
        0.0
    };
    ((mixed_kg + split_capacity) / demanded_kg).clamp(0.0, 1.0)
}

fn draw_resource(
    tanks: &[crate::TankMount],
    available: &[f64],
    accessible: &[bool],
    resource: ResourceKey,
    demanded_kg: f64,
    oxidizer_fraction: f64,
    fuel_fraction: f64,
    reserved: &mut [f64],
) {
    let mut remaining = demanded_kg;
    for (index, mount) in tanks.iter().enumerate() {
        if !accessible[index]
            || component_role(mount.resource, resource) != Some(ComponentRole::Mixed)
        {
            continue;
        }
        let take = remaining.min(available[index]);
        reserved[index] += take;
        remaining -= take;
        if remaining <= 1.0e-12 {
            return;
        }
    }
    if oxidizer_fraction <= 0.0 || fuel_fraction <= 0.0 {
        return;
    }
    let oxidizer_need = remaining * oxidizer_fraction;
    let fuel_need = remaining * fuel_fraction;
    take_component(
        tanks,
        available,
        accessible,
        resource,
        ComponentRole::Oxidizer,
        oxidizer_need,
        reserved,
    );
    take_component(
        tanks,
        available,
        accessible,
        resource,
        ComponentRole::Fuel,
        fuel_need,
        reserved,
    );
}

fn take_component(
    tanks: &[crate::TankMount],
    available: &[f64],
    accessible: &[bool],
    resource: ResourceKey,
    role: ComponentRole,
    mut needed: f64,
    reserved: &mut [f64],
) {
    for (index, mount) in tanks.iter().enumerate() {
        if !accessible[index] || component_role(mount.resource, resource) != Some(role) {
            continue;
        }
        let take = needed.min(available[index]).max(0.0);
        reserved[index] += take;
        needed -= take;
        if needed <= 1.0e-12 {
            return;
        }
    }
}

fn component_role(tank_resource: TankResource, resource: ResourceKey) -> Option<ComponentRole> {
    match resource {
        ResourceKey::Pure(pure) => match tank_resource {
            TankResource::Unspecified => Some(ComponentRole::Mixed),
            TankResource::Stored(candidate) if candidate == pure => Some(ComponentRole::Mixed),
            _ => None,
        },
        ResourceKey::Chemical(propellant) => match tank_resource {
            TankResource::Unspecified => Some(ComponentRole::Mixed),
            TankResource::Pair(candidate) if candidate == propellant => Some(ComponentRole::Mixed),
            TankResource::Oxidizer(candidate) if candidate == propellant => {
                Some(ComponentRole::Oxidizer)
            }
            TankResource::Fuel(candidate) if candidate == propellant => Some(ComponentRole::Fuel),
            TankResource::Stored(stored) => stored_role(stored, propellant),
            _ => None,
        },
    }
}

fn stored_role(stored: StoredPropellant, propellant: Propellant) -> Option<ComponentRole> {
    match (stored, propellant) {
        (StoredPropellant::Hydrazine, Propellant::MonopropHydrazine) => Some(ComponentRole::Mixed),
        (
            StoredPropellant::Lox,
            Propellant::LoxRp1 | Propellant::LoxMethane | Propellant::LoxHydrogen,
        ) => Some(ComponentRole::Oxidizer),
        (StoredPropellant::Rp1, Propellant::LoxRp1)
        | (StoredPropellant::LiquidMethane, Propellant::LoxMethane)
        | (StoredPropellant::LiquidHydrogen, Propellant::LoxHydrogen)
        | (StoredPropellant::Mmh, Propellant::NtoMmh) => Some(ComponentRole::Fuel),
        (StoredPropellant::Nto, Propellant::NtoMmh) => Some(ComponentRole::Oxidizer),
        _ => None,
    }
}

fn parallel_axis(mass_kg: f64, position_body_m: DVec3) -> DMat3 {
    mass_kg
        * (DMat3::IDENTITY * position_body_m.length_squared()
            - DMat3::from_cols(
                position_body_m * position_body_m.x,
                position_body_m * position_body_m.y,
                position_body_m * position_body_m.z,
            ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AeroGeometry, AeroPanel, AssemblyEndpoint, AssemblyLinkState, ChamberMaterial, CoolingMode,
        EngineCycle, EngineMount, LiquidEngineSpec, NamedAssemblyLink, NozzleContour, NtrFluid,
        NuclearThermalSpec, RigidBodyProperties, SolidGrainGeometry, SolidMotorSpec, TankMount,
        TankShape, TankSpec, VehicleAssembly,
    };

    fn compiled_engine() -> CompiledEngine {
        CompiledEngine::Liquid(
            LiquidEngineSpec {
                name: "resource-test-engine".into(),
                propellant: Propellant::LoxRp1,
                cycle: EngineCycle::GasGenerator,
                chamber_pressure_pa: 9.7e6,
                throat_radius_m: 0.08,
                expansion_ratio: 18.0,
                nozzle_length_m: 0.9,
                contour: NozzleContour::Bell,
                chamber_material: ChamberMaterial::nickel_superalloy(),
                cooling: CoolingMode::Regenerative,
                mixture_ratio: None,
                characteristic_length_m: None,
                gimbal_range_rad: 0.0,
                min_throttle: Some(0.4),
                restartable: true,
            }
            .compile()
            .expect("compile liquid engine"),
        )
    }

    fn tank(
        name: &str,
        resource: TankResource,
        density_kg_m3: f64,
        fill_fraction: f64,
        position: DVec3,
    ) -> TankMount {
        let shape = TankShape::Sphere { diameter_m: 0.5 };
        let compiled = TankSpec {
            shape,
            pressure_pa: 500_000.0,
            material: ChamberMaterial::nickel_superalloy(),
        }
        .compile(density_kg_m3)
        .expect("compile tank");
        let loaded = compiled.full_propellant_kg * fill_fraction;
        TankMount {
            name: name.into(),
            tank: compiled,
            position_body_m: position.to_array(),
            intrinsic_inertia_body_kg_m2: shape
                .intrinsic_inertia_body_kg_m2(compiled.dry_mass_kg, loaded)
                .expect("tank inertia"),
            initial_propellant_kg: Some(loaded),
            resource,
        }
    }

    fn vehicle(tanks: Vec<TankMount>, engine_count: usize) -> VehicleDefinition {
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("test panel"),
        ])
        .expect("test geometry");
        let properties = RigidBodyProperties::new(1_000.0, DMat3::IDENTITY * 1_000.0)
            .expect("base mass properties");
        let engine = compiled_engine();
        let engines = (0..engine_count)
            .map(|index| EngineMount {
                name: format!("engine-{index}"),
                engine: engine.clone(),
                position_body_m: [0.0; 3],
                thrust_axis_body: [1.0, 0.0, 0.0],
            })
            .collect();
        let mut vehicle = VehicleDefinition::new("resource-test", geometry, properties, vec![])
            .expect("vehicle")
            .with_engines(engines)
            .expect("engines")
            .with_tanks(tanks)
            .expect("tanks");
        vehicle.bake_engine_masses().expect("bake engines");
        vehicle.bake_tank_masses().expect("bake tanks");
        vehicle
    }

    #[test]
    fn multiple_engines_draw_operating_point_flow_and_update_mass() {
        let propellant = Propellant::LoxRp1;
        let tanks = vec![
            tank(
                "oxidizer",
                TankResource::Oxidizer(propellant),
                1_141.0,
                1.0,
                DVec3::ZERO,
            ),
            tank(
                "fuel",
                TankResource::Fuel(propellant),
                810.0,
                1.0,
                DVec3::ZERO,
            ),
        ];
        let mut vehicle = vehicle(tanks, 2);
        let mut state = vehicle.initial_resource_state();
        let initial_mass = vehicle.mass_properties.mass_kg;
        let point_a = vehicle.engines[0]
            .engine
            .operating_point(0.5, 0.0, 0.0)
            .unwrap();
        let point_b = vehicle.engines[1]
            .engine
            .operating_point(1.0, 0.0, 0.0)
            .unwrap();
        let dt_s = 0.1;

        let allocation = vehicle
            .plan_propulsion_step(&state, &[0.5, 1.0], &[], 0.0, dt_s)
            .expect("allocate engine flow");
        let expected_mass = (point_a.mass_flow_kg_s + point_b.mass_flow_kg_s) * dt_s;
        assert!((allocation.total_propellant_flow_kg_s * dt_s - expected_mass).abs() < 1.0e-10);
        assert!(
            (allocation.tank_consumption_kg.iter().sum::<f64>() - expected_mass).abs() < 1.0e-10
        );
        let mixture_ratio = propellant.reference_mixture_ratio().unwrap();
        let expected_oxidizer = expected_mass * mixture_ratio / (1.0 + mixture_ratio);
        let expected_fuel = expected_mass / (1.0 + mixture_ratio);
        assert!((allocation.tank_consumption_kg[0] - expected_oxidizer).abs() < 1.0e-10);
        assert!((allocation.tank_consumption_kg[1] - expected_fuel).abs() < 1.0e-10);
        assert!((allocation.force_body_n.x - point_a.thrust_n - point_b.thrust_n).abs() < 1.0e-8);

        vehicle
            .commit_propulsion_step(&mut state, &allocation)
            .expect("commit propellant burn");
        assert!((vehicle.mass_properties.mass_kg - (initial_mass - expected_mass)).abs() < 1.0e-9);
        assert!(
            (state.total_tank_propellant_kg()
                - (vehicle.tanks[0].loaded_propellant_kg()
                    + vehicle.tanks[1].loaded_propellant_kg()
                    - expected_mass))
                .abs()
                < 1.0e-9
        );
    }

    #[test]
    fn partial_inventory_scales_engines_proportionally_and_respects_throttle_floor() {
        let vehicle = vehicle(
            vec![tank(
                "mixed",
                TankResource::Pair(Propellant::LoxRp1),
                800.0,
                1.0,
                DVec3::ZERO,
            )],
            2,
        );
        let mut state = vehicle.initial_resource_state();
        let dt_s = 0.1;
        let requested_flow_kg = (vehicle.engines[0]
            .engine
            .operating_point(0.6, 0.0, 0.0)
            .unwrap()
            .mass_flow_kg_s
            + vehicle.engines[1]
                .engine
                .operating_point(1.0, 0.0, 0.0)
                .unwrap()
                .mass_flow_kg_s)
            * dt_s;
        let available_kg = requested_flow_kg * 0.75;
        assert!(available_kg < vehicle.tanks[0].tank.full_propellant_kg);
        state.tank_propellant_kg[0] = available_kg;

        let allocation = vehicle
            .plan_propulsion_step(&state, &[0.6, 1.0], &[], 0.0, dt_s)
            .expect("allocate partial inventory");

        assert!((allocation.engine_throttles[0] - 0.45).abs() < 1.0e-10);
        assert!((allocation.engine_throttles[1] - 0.75).abs() < 1.0e-10);
        assert!((allocation.tank_consumption_kg[0] - available_kg).abs() < 1.0e-10);
        assert!(allocation.fuel_limited);

        state.tank_propellant_kg[0] = requested_flow_kg * 0.1;
        let starved = vehicle
            .plan_propulsion_step(&state, &[0.6, 1.0], &[], 0.0, dt_s)
            .expect("resolve below-minimum throttle");
        assert_eq!(starved.engine_throttles, vec![0.0, 0.0]);
        assert_eq!(starved.tank_consumption_kg, vec![0.0]);
        assert!(starved.fuel_limited);
    }

    #[test]
    fn closed_assembly_hatch_blocks_tank_draw_for_mounted_engines() {
        let propellant = Propellant::LoxRp1;
        let tanks = vec![tank(
            "tank-body.main",
            TankResource::Pair(propellant),
            800.0,
            1.0,
            DVec3::ZERO,
        )];
        let mut vehicle = vehicle(tanks, 1);
        vehicle.assembly = Some(VehicleAssembly {
            root_body: 0,
            body_names: vec!["engine-body".into(), "tank-body".into()],
            links: vec![NamedAssemblyLink {
                name: "sealed-hatch".into(),
                state: AssemblyLinkState {
                    a: 0,
                    b: 1,
                    hatch: true,
                    open: false,
                },
            }],
            volumes: vec![],
            tanks: vec![AssemblyEndpoint {
                name: "tank-body.main".into(),
                body: 1,
            }],
            engine_ports: vec![AssemblyEndpoint {
                name: "engine-body.port".into(),
                body: 0,
            }],
        });
        let state = vehicle.initial_resource_state();

        let allocation = vehicle
            .plan_propulsion_step(&state, &[1.0], &[], 0.0, 0.1)
            .expect("plan disconnected engine");

        assert_eq!(allocation.force_body_n, DVec3::ZERO);
        assert_eq!(allocation.tank_consumption_kg, vec![0.0]);
        assert!(allocation.fuel_limited);
    }

    #[test]
    fn manual_transfer_conserves_mass_and_recomputes_the_center_of_mass() {
        let resource = TankResource::Pair(Propellant::LoxRp1);
        let tanks = vec![
            tank("left", resource, 800.0, 0.5, DVec3::new(-2.0, 0.0, 0.0)),
            tank("right", resource, 800.0, 0.5, DVec3::new(2.0, 0.0, 0.0)),
        ];
        let mut vehicle = vehicle(tanks, 0);
        let mut state = vehicle.initial_resource_state();
        let initial_mass = vehicle.mass_properties.mass_kg;
        let initial_total = state.total_tank_propellant_kg();
        let source_before = state.tank_propellant_kg[0];
        let destination_before = state.tank_propellant_kg[1];
        let transfer_kg = 1.0;
        let frame_shift = vehicle
            .transfer_propellant(&mut state, "left", "right", transfer_kg)
            .expect("manual transfer");

        assert!((state.total_tank_propellant_kg() - initial_total).abs() < 1.0e-12);
        assert!((vehicle.mass_properties.mass_kg - initial_mass).abs() < 1.0e-12);
        assert!((state.tank_propellant_kg[0] - (source_before - transfer_kg)).abs() < 1.0e-12);
        assert!((state.tank_propellant_kg[1] - (destination_before + transfer_kg)).abs() < 1.0e-12);
        assert!(frame_shift.x < 0.0);
        assert!(
            vehicle
                .tanks
                .iter()
                .all(|tank| tank.position_body_m[0] < 2.0)
        );
    }

    #[test]
    fn transfer_requires_open_assembly_path_and_respects_destination_capacity() {
        let resource = TankResource::Pair(Propellant::LoxRp1);
        let tanks = vec![
            tank("source-body.left", resource, 800.0, 1.0, DVec3::ZERO),
            tank("destination-body.right", resource, 800.0, 0.9, DVec3::X),
        ];
        let mut vehicle = vehicle(tanks, 0);
        vehicle.assembly = Some(VehicleAssembly {
            root_body: 0,
            body_names: vec!["source-body".into(), "destination-body".into()],
            links: vec![NamedAssemblyLink {
                name: "crossfeed-hatch".into(),
                state: AssemblyLinkState {
                    a: 0,
                    b: 1,
                    hatch: true,
                    open: false,
                },
            }],
            volumes: vec![],
            tanks: vec![
                AssemblyEndpoint {
                    name: "source-body.left".into(),
                    body: 0,
                },
                AssemblyEndpoint {
                    name: "destination-body.right".into(),
                    body: 1,
                },
            ],
            engine_ports: vec![],
        });
        let mut state = vehicle.initial_resource_state();
        let initial_state = state.clone();
        let initial_mass = vehicle.mass_properties.mass_kg;

        assert!(
            vehicle
                .transfer_propellant(
                    &mut state,
                    "source-body.left",
                    "destination-body.right",
                    1.0
                )
                .is_err()
        );
        assert_eq!(state, initial_state);
        assert_eq!(vehicle.mass_properties.mass_kg, initial_mass);

        vehicle.assembly.as_mut().unwrap().links[0].state.open = true;
        let destination_capacity = vehicle.tanks[1].tank.full_propellant_kg;
        let destination_fill = state.tank_propellant_kg[1];
        let excess = destination_capacity - destination_fill + 0.01;
        assert!(
            vehicle
                .transfer_propellant(
                    &mut state,
                    "source-body.left",
                    "destination-body.right",
                    excess,
                )
                .is_err()
        );
        assert_eq!(state, initial_state);
        assert_eq!(vehicle.mass_properties.mass_kg, initial_mass);

        vehicle
            .transfer_propellant(
                &mut state,
                "source-body.left",
                "destination-body.right",
                1.0,
            )
            .expect("transfer over open path");
        assert!(
            (state.tank_propellant_kg[0] - (initial_state.tank_propellant_kg[0] - 1.0)).abs()
                < 1.0e-12
        );
        assert!(
            (state.tank_propellant_kg[1] - (initial_state.tank_propellant_kg[1] + 1.0)).abs()
                < 1.0e-12
        );
    }

    #[test]
    fn nuclear_thermal_engine_draws_only_its_compiled_working_fluid() {
        let engine = NuclearThermalSpec {
            name: "hydrogen-ntr".into(),
            fluid: NtrFluid::Hydrogen,
            core_temp_k: 2_800.0,
            core_power_mw: 500.0,
            reactor_specific_mass_kg_per_mw: None,
            throat_radius_m: 0.1,
            expansion_ratio: 100.0,
            nozzle_length_m: 5.0,
            contour: NozzleContour::Bell,
            material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            gimbal_range_rad: 0.0,
            startup_tau_s: None,
            min_throttle: None,
        }
        .compile()
        .expect("compile NTR")
        .0;
        let engine = EngineMount {
            name: "hydrogen-ntr".into(),
            engine: CompiledEngine::Liquid(engine),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        let compatible = tank(
            "hydrogen",
            TankResource::Stored(StoredPropellant::LiquidHydrogen),
            71.0,
            1.0,
            DVec3::ZERO,
        );
        let incompatible = tank(
            "methane",
            TankResource::Stored(StoredPropellant::LiquidMethane),
            422.0,
            1.0,
            DVec3::ZERO,
        );
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("test panel"),
        ])
        .expect("test geometry");
        let properties = RigidBodyProperties::new(1_000.0, DMat3::IDENTITY * 1_000.0)
            .expect("base mass properties");
        let mut vehicle = VehicleDefinition::new("ntr-resource-test", geometry, properties, vec![])
            .expect("vehicle")
            .with_engines(vec![engine])
            .expect("engine")
            .with_tanks(vec![incompatible, compatible])
            .expect("tanks");
        vehicle.bake_engine_masses().expect("bake engine");
        vehicle.bake_tank_masses().expect("bake tanks");
        let state = vehicle.initial_resource_state();
        let requested_flow = vehicle.engines[0]
            .engine
            .operating_point(1.0, 0.0, 0.0)
            .expect("NTR operating point")
            .mass_flow_kg_s
            * 0.1;

        let allocation = vehicle
            .plan_propulsion_step(&state, &[1.0], &[], 0.0, 0.1)
            .expect("allocate NTR working fluid");

        assert_eq!(allocation.tank_consumption_kg[0], 0.0);
        assert!((allocation.tank_consumption_kg[1] - requested_flow).abs() < 1.0e-10);
        assert!((allocation.total_propellant_flow_kg_s * 0.1 - requested_flow).abs() < 1.0e-10);
    }

    #[test]
    fn solid_motor_burn_clock_depletes_grain_mass_without_an_external_tank() {
        let (burn_rate_coeff, burn_rate_exponent) = SolidMotorSpec::apcp_ballistics();
        let motor = SolidMotorSpec {
            name: "solid-resource-test".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.32,
            grain_geometry: SolidGrainGeometry::Circular,
            segment_length_m: 1.5,
            segments: 4,
            burn_rate_coeff,
            burn_rate_exponent,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        }
        .compile()
        .expect("compile solid motor");
        let initial_propellant_kg = motor.propellant_mass_kg;
        let engine = EngineMount {
            name: "booster".into(),
            engine: CompiledEngine::Solid(motor.clone()),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        let geometry = AeroGeometry::new(vec![
            AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("test panel"),
        ])
        .expect("test geometry");
        let properties = RigidBodyProperties::new(1_000.0, DMat3::IDENTITY * 1_000.0)
            .expect("base mass properties");
        let mut vehicle =
            VehicleDefinition::new("solid-resource-test", geometry, properties, vec![])
                .expect("vehicle")
                .with_engines(vec![engine])
                .expect("engine mount");
        vehicle.bake_engine_masses().expect("bake motor");
        let mut state = vehicle.initial_resource_state();
        let initial_mass = vehicle.mass_properties.mass_kg;
        let dt_s = motor.burn_time_s / 100.0;
        let allocation = vehicle
            .plan_propulsion_step(&state, &[1.0], &[], 0.0, dt_s)
            .expect("plan solid burn");
        let remaining = CompiledEngine::Solid(motor.clone())
            .propellant_remaining_kg(allocation.solid_burn_time_s[0])
            .expect("solid remaining mass");
        assert!(remaining < initial_propellant_kg);
        assert!(allocation.tank_consumption_kg.is_empty());
        assert!(allocation.total_propellant_flow_kg_s > 0.0);

        vehicle
            .commit_propulsion_step(&mut state, &allocation)
            .expect("commit solid burn");
        assert!(
            (vehicle.mass_properties.mass_kg
                - (initial_mass - (initial_propellant_kg - remaining)))
                .abs()
                < 1.0e-8
        );
        assert!(state.solid_ignited[0]);
    }
}
