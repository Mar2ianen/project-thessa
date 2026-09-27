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
    /// Availability scales for additional operating-point consumers booked
    /// alongside the installed liquid/solid rocket mounts.
    pub additional_resource_consumers: Vec<ConsumerResourceAllocation>,
}

/// One operating-point mass flow into a named installed-tank resource.
///
/// `feed_port_name` is an assembly endpoint name such as `stage.engine-feed`.
/// When omitted, legacy vehicle-level feed reachability is used. Multiple
/// demands with the same consumer name share one availability scale, so a
/// two-reactant consumer cannot consume one reactant without the other.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleResourceDemand {
    pub consumer_name: String,
    pub feed_port_name: Option<String>,
    pub resource: StoredPropellant,
    pub mass_flow_kg_s: f64,
}

/// Authored route from an installed resource consumer name to one assembly
/// engine-feed endpoint. This keeps routing independent of any one propulsion
/// family and also covers electrical sources such as fuel cells.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct VehicleResourceFeedPort {
    pub consumer_name: String,
    pub feed_port_name: String,
}

impl VehicleResourceDemand {
    pub fn new(
        consumer_name: impl Into<String>,
        resource: StoredPropellant,
        mass_flow_kg_s: f64,
    ) -> Self {
        Self {
            consumer_name: consumer_name.into(),
            feed_port_name: None,
            resource,
            mass_flow_kg_s,
        }
    }

    pub fn through_feed_port(mut self, feed_port_name: impl Into<String>) -> Self {
        self.feed_port_name = Some(feed_port_name.into());
        self
    }

    /// Resource flows reported by an installed jet/ESTOC operating point.
    pub fn from_jet_point(
        mount: &crate::JetMount,
        point: &crate::EstocPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        let (bulk_fuel, boost_fuel) = match &mount.engine {
            crate::CompiledJet::Air(engine) => (engine.fuel, None),
            crate::CompiledJet::Estoc(engine) => (engine.bulk_fuel, engine.boost_coolant_fuel),
        };
        let mut demands = Vec::new();
        push_stored_demand(
            &mut demands,
            &mount.name,
            feed_port_name,
            jet_fuel_resource(bulk_fuel),
            point.bulk_fuel_flow_kg_s,
        );
        if point.boost_fuel_flow_kg_s > 0.0 {
            // An ESTOC may use a second fuel for its boost/coolant circuit.
            // The compiler only permits positive boost flow when that fuel is
            // explicitly authored.
            if let Some(boost_fuel) = boost_fuel {
                push_stored_demand(
                    &mut demands,
                    &mount.name,
                    feed_port_name,
                    jet_fuel_resource(boost_fuel),
                    point.boost_fuel_flow_kg_s,
                );
            }
        }
        push_stored_demand(
            &mut demands,
            &mount.name,
            feed_port_name,
            StoredPropellant::Lox,
            point.oxidizer_flow_kg_s,
        );
        demands
    }

    /// Resource flow reported by an installed electric thruster.
    pub fn from_electric_thruster_point(
        mount: &crate::ElectricThrusterMount,
        point: &crate::ElectricThrusterPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        let resource = electric_propellant_resource(mount.engine.propellant);
        vec![stored_demand(
            &mount.name,
            feed_port_name,
            resource,
            point.mass_flow_kg_s,
        )]
    }

    /// Tank fuel flow from a piston/electric propeller-drive operating point.
    /// Electric source drives report zero fuel flow and therefore no demand.
    pub fn from_propeller_drive_point(
        mount: &crate::PropellerDriveMount,
        point: &crate::PropDrivePoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        let crate::CompiledShaftPowerSource::Piston(engine) = mount.drive.source else {
            return Vec::new();
        };
        vec![stored_demand(
            &mount.name,
            feed_port_name,
            jet_fuel_resource(engine.fuel),
            point.fuel_flow_kg_s,
        )]
    }

    /// Fuel flow from an installed turbine-propeller operating point.
    pub fn from_turboprop_point(
        mount: &crate::TurbopropMount,
        point: &crate::TurbopropOperatingPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        vec![stored_demand(
            &mount.name,
            feed_port_name,
            jet_fuel_resource(mount.drive.air.fuel),
            point.air.fuel_flow_kg_s,
        )]
    }

    /// Fuel flow from the shared-shaft turbogen APU model.
    pub fn from_apu_point(
        consumer_name: &str,
        point: &crate::AuxiliaryPowerUnitOperatingPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        vec![stored_demand(
            consumer_name,
            feed_port_name,
            jet_fuel_resource(point.fuel),
            point.fuel_flow_kg_s,
        )]
    }

    /// Fuel and working-fluid flows from a continuous fusion torch.
    pub fn from_fusion_torch_point(
        mount: &crate::FusionTorchMount,
        point: &crate::FusionTorchOperatingPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        let mut demands = fusion_reactant_demands(
            &mount.name,
            mount.engine.reaction,
            point.fusion_fuel_flow_kg_s,
            feed_port_name,
        );
        push_stored_demand(
            &mut demands,
            &mount.name,
            feed_port_name,
            electric_propellant_resource(mount.engine.working_fluid),
            point.working_flow_kg_s,
        );
        demands
    }

    /// Fuel and working-fluid flows from a pulsed fusion step.
    pub fn from_pulsed_fusion_point(
        mount: &crate::PulsedFusionMount,
        point: &crate::PulsedFusionOperatingPoint,
        feed_port_name: Option<&str>,
    ) -> Vec<Self> {
        let mut demands = fusion_reactant_demands(
            &mount.name,
            mount.engine.reaction,
            point.fuel_mass_flow_kg_s,
            feed_port_name,
        );
        push_stored_demand(
            &mut demands,
            &mount.name,
            feed_port_name,
            electric_propellant_resource(mount.engine.working_fluid),
            point.working_flow_kg_s,
        );
        demands
    }

    /// Propellant used by one delivered RCS pulse, expressed as a per-step
    /// flow so it enters the same fixed-step inventory commit.
    pub fn from_rcs_pulse(
        mount: &crate::RcsMount,
        pulse: crate::RcsPulse,
        dt_s: f64,
        feed_port_name: Option<&str>,
    ) -> Result<Vec<Self>, VehicleError> {
        if !dt_s.is_finite()
            || dt_s <= 0.0
            || !pulse.propellant_kg.is_finite()
            || pulse.propellant_kg < 0.0
        {
            return Err(VehicleError::InvalidVehicle(
                "RCS resource flow needs finite pulse mass and positive step duration".into(),
            ));
        }
        let resource = match &mount.thruster {
            crate::RcsThruster::Monoprop(_) => StoredPropellant::Hydrazine,
            crate::RcsThruster::ColdGas(thruster) => match thruster.gas {
                Propellant::ColdGasNitrogen => StoredPropellant::Nitrogen,
                Propellant::ColdGasHelium => StoredPropellant::Helium,
                _ => {
                    return Err(VehicleError::InvalidVehicle(format!(
                        "RCS mount '{}' has an invalid cold-gas identity",
                        mount.name
                    )));
                }
            },
        };
        Ok(vec![stored_demand(
            &mount.name,
            feed_port_name,
            resource,
            pulse.propellant_kg / dt_s,
        )])
    }
}

/// Actual draw and availability scale for one named consumer.
#[derive(Debug, Clone, PartialEq)]
pub struct ConsumerResourceAllocation {
    pub consumer_name: String,
    pub requested_mass_kg: f64,
    pub actual_mass_kg: f64,
    /// Common scale applied to every resource flow for this consumer.
    pub scale: f64,
}

/// Tank draw planned from operating-point flows for one fixed step.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleResourcePlan {
    pub tank_consumption_kg: Vec<f64>,
    pub consumers: Vec<ConsumerResourceAllocation>,
    pub total_consumption_kg: f64,
    pub dt_s: f64,
}

/// Stateful APU outputs and matching resource transaction for one fixed step.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleAuxiliaryPowerUnitStep {
    pub next_states: Vec<crate::AuxiliaryPowerUnitState>,
    pub operating_points: Vec<crate::AuxiliaryPowerUnitOperatingPoint>,
    pub generated_electrical_power_w: f64,
    pub pneumatic_bleed_power_w: f64,
    pub resource_limited: bool,
    pub resource_plan: VehicleResourcePlan,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ResourceFlowGroup {
    resource: StoredPropellant,
    feed_port_name: Option<String>,
    demand_indices: Vec<usize>,
    accessible_tanks: Vec<bool>,
}

fn stored_demand(
    consumer_name: &str,
    feed_port_name: Option<&str>,
    resource: StoredPropellant,
    mass_flow_kg_s: f64,
) -> VehicleResourceDemand {
    let mut demand = VehicleResourceDemand::new(consumer_name, resource, mass_flow_kg_s);
    if let Some(port_name) = feed_port_name {
        demand.feed_port_name = Some(port_name.to_owned());
    }
    demand
}

fn push_stored_demand(
    demands: &mut Vec<VehicleResourceDemand>,
    consumer_name: &str,
    feed_port_name: Option<&str>,
    resource: StoredPropellant,
    mass_flow_kg_s: f64,
) {
    demands.push(stored_demand(
        consumer_name,
        feed_port_name,
        resource,
        mass_flow_kg_s,
    ));
}

fn jet_fuel_resource(fuel: crate::JetFuel) -> StoredPropellant {
    match fuel {
        crate::JetFuel::Kerosene => StoredPropellant::Rp1,
        crate::JetFuel::Methane => StoredPropellant::LiquidMethane,
        crate::JetFuel::Hydrogen => StoredPropellant::LiquidHydrogen,
    }
}

fn electric_propellant_resource(propellant: crate::ElectricPropellant) -> StoredPropellant {
    match propellant {
        crate::ElectricPropellant::Xenon => StoredPropellant::Xenon,
        crate::ElectricPropellant::Krypton => StoredPropellant::Krypton,
        crate::ElectricPropellant::Argon => StoredPropellant::Argon,
        crate::ElectricPropellant::Iodine => StoredPropellant::Iodine,
        crate::ElectricPropellant::Nitrogen => StoredPropellant::Nitrogen,
        crate::ElectricPropellant::Hydrogen => StoredPropellant::LiquidHydrogen,
        crate::ElectricPropellant::Ammonia => StoredPropellant::Ammonia,
        crate::ElectricPropellant::Water => StoredPropellant::Water,
    }
}

fn fusion_reactant_demands(
    consumer_name: &str,
    reaction: crate::FusionReaction,
    fuel_flow_kg_s: f64,
    feed_port_name: Option<&str>,
) -> Vec<VehicleResourceDemand> {
    let mut demands = Vec::with_capacity(2);
    let (first, first_fraction, second, second_fraction) = match reaction {
        crate::FusionReaction::DeuteriumTritium => {
            let deuterium_kg_mol = 0.002_014_101_778;
            let tritium_kg_mol = 0.003_016_049_278;
            let total = deuterium_kg_mol + tritium_kg_mol;
            (
                Some(StoredPropellant::Deuterium),
                deuterium_kg_mol / total,
                Some(StoredPropellant::Tritium),
                tritium_kg_mol / total,
            )
        }
        crate::FusionReaction::DeuteriumDeuterium => {
            (Some(StoredPropellant::Deuterium), 1.0, None, 0.0)
        }
        crate::FusionReaction::DeuteriumHelium3 => {
            let deuterium_kg_mol = 0.002_014_101_778;
            let helium3_kg_mol = 0.003_016_029_322;
            let total = deuterium_kg_mol + helium3_kg_mol;
            (
                Some(StoredPropellant::Deuterium),
                deuterium_kg_mol / total,
                Some(StoredPropellant::Helium3),
                helium3_kg_mol / total,
            )
        }
        crate::FusionReaction::ProtonBoron11 => {
            let protium_kg_mol = 0.001_007_825_032;
            let boron11_kg_mol = 0.011_009_305_36;
            let total = protium_kg_mol + boron11_kg_mol;
            (
                Some(StoredPropellant::Protium),
                protium_kg_mol / total,
                Some(StoredPropellant::Boron11),
                boron11_kg_mol / total,
            )
        }
    };
    if let Some(resource) = first {
        push_stored_demand(
            &mut demands,
            consumer_name,
            feed_port_name,
            resource,
            fuel_flow_kg_s * first_fraction,
        );
    }
    if let Some(resource) = second {
        push_stored_demand(
            &mut demands,
            consumer_name,
            feed_port_name,
            resource,
            fuel_flow_kg_s * second_fraction,
        );
    }
    demands
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

    /// Total compatible inventory reachable from an optional named feed port.
    pub fn available_stored_resource_kg(
        &self,
        state: &VehicleResourceState,
        resource: StoredPropellant,
        feed_port_name: Option<&str>,
    ) -> Result<f64, VehicleError> {
        self.validate_resource_state(state)?;
        let accessible = self.feedable_tanks_for_port(feed_port_name)?;
        Ok(self
            .tanks
            .iter()
            .zip(&state.tank_propellant_kg)
            .zip(accessible)
            .filter(|((tank, _), reachable)| {
                *reachable && stored_tank_resource_compatible(tank.resource, resource)
            })
            .map(|((_, mass), _)| *mass)
            .sum())
    }

    /// Evaluate every installed APU from its real airbreather/shaft operating
    /// point and throttle it against reachable tank inventory. Returned
    /// generator output is actual electrical power for
    /// `ElectricalPowerCommand::auxiliary_generation_power_w`; commit its
    /// resource plan with `commit_resource_flows` after the vehicle step.
    pub fn plan_auxiliary_power_units(
        &self,
        resource_state: &VehicleResourceState,
        apu_states: &[crate::AuxiliaryPowerUnitState],
        commands: &[crate::AuxiliaryPowerUnitCommand],
        condition: &crate::FlightCondition,
    ) -> Result<VehicleAuxiliaryPowerUnitStep, VehicleError> {
        self.validate_resource_state(resource_state)?;
        if apu_states.len() != self.auxiliary_power_units.len()
            || commands.len() != self.auxiliary_power_units.len()
        {
            return Err(VehicleError::InvalidVehicle(format!(
                "expected {} APU states and commands, got {} and {}",
                self.auxiliary_power_units.len(),
                apu_states.len(),
                commands.len()
            )));
        }
        let Some(first_command) = commands.first() else {
            return Ok(VehicleAuxiliaryPowerUnitStep {
                next_states: Vec::new(),
                operating_points: Vec::new(),
                generated_electrical_power_w: 0.0,
                pneumatic_bleed_power_w: 0.0,
                resource_limited: false,
                resource_plan: self.plan_resource_flows(resource_state, &[], 1.0)?,
            });
        };
        if commands
            .iter()
            .any(|command| (command.dt_s - first_command.dt_s).abs() > 1.0e-12)
        {
            return Err(VehicleError::InvalidVehicle(
                "all APU commands in one vehicle step must have the same dt".into(),
            ));
        }
        let dt_s = first_command.dt_s;
        let mut throttle_scales = vec![1.0; commands.len()];
        let mut resource_limited = false;
        for _ in 0..16 {
            let mut points = Vec::with_capacity(commands.len());
            let mut demands = Vec::with_capacity(commands.len());
            for (index, ((mount, state), command)) in self
                .auxiliary_power_units
                .iter()
                .zip(apu_states)
                .zip(commands)
                .enumerate()
            {
                let mut limited_command = *command;
                limited_command.throttle *= throttle_scales[index];
                let (_, point) = mount
                    .unit
                    .advance(*state, limited_command, condition)
                    .map_err(VehicleError::Propulsion)?;
                demands.extend(VehicleResourceDemand::from_apu_point(
                    &mount.name,
                    &point,
                    mount.feed_port_name.as_deref(),
                ));
                points.push(point);
            }
            let plan = self.plan_resource_flows(resource_state, &demands, dt_s)?;
            let mut changed = false;
            for allocation in &plan.consumers {
                if allocation.scale < 1.0 - 1.0e-10 {
                    if let Some(index) = self
                        .auxiliary_power_units
                        .iter()
                        .position(|mount| mount.name == allocation.consumer_name)
                    {
                        throttle_scales[index] *= allocation.scale;
                        changed = true;
                    }
                }
            }
            if !changed {
                let next_states = points
                    .iter()
                    .map(|point| crate::AuxiliaryPowerUnitState {
                        shaft: point.shaft_state,
                    })
                    .collect();
                let generated_electrical_power_w = points
                    .iter()
                    .map(|point| point.electrical_power_w)
                    .sum::<f64>();
                let pneumatic_bleed_power_w = points
                    .iter()
                    .map(|point| point.pneumatic_bleed_power_w)
                    .sum::<f64>();
                if !generated_electrical_power_w.is_finite() || !pneumatic_bleed_power_w.is_finite()
                {
                    return Err(VehicleError::InvalidVehicle(
                        "APU aggregate output is non-finite".into(),
                    ));
                }
                return Ok(VehicleAuxiliaryPowerUnitStep {
                    next_states,
                    operating_points: points,
                    generated_electrical_power_w,
                    pneumatic_bleed_power_w,
                    resource_limited,
                    resource_plan: plan,
                });
            }
            resource_limited = true;
        }
        Err(VehicleError::InvalidVehicle(
            "APU tank-limited operating points did not converge".into(),
        ))
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

    /// Allocate explicit pure-fluid operating-point flows against compatible,
    /// reachable installed tanks. Consumers that need several substances
    /// receive one common scale across all their demands.
    pub fn plan_resource_flows(
        &self,
        state: &VehicleResourceState,
        demands: &[VehicleResourceDemand],
        dt_s: f64,
    ) -> Result<VehicleResourcePlan, VehicleError> {
        self.validate_resource_state(state)?;
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(VehicleError::InvalidVehicle(
                "resource-flow step must be finite and positive".into(),
            ));
        }

        let mut consumer_names = Vec::<String>::new();
        let mut demand_consumers = Vec::with_capacity(demands.len());
        let mut groups = Vec::<ResourceFlowGroup>::new();
        for (demand_index, demand) in demands.iter().enumerate() {
            if demand.consumer_name.trim().is_empty()
                || !demand.mass_flow_kg_s.is_finite()
                || demand.mass_flow_kg_s < 0.0
                || !(demand.mass_flow_kg_s * dt_s).is_finite()
                || demand
                    .feed_port_name
                    .as_ref()
                    .is_some_and(|port| port.trim().is_empty())
            {
                return Err(VehicleError::InvalidVehicle(
                    "resource demands need a named consumer, optional named feed port, and finite non-negative flow".into(),
                ));
            }
            let consumer_index = consumer_names
                .iter()
                .position(|name| name == &demand.consumer_name)
                .unwrap_or_else(|| {
                    consumer_names.push(demand.consumer_name.clone());
                    consumer_names.len() - 1
                });
            demand_consumers.push(consumer_index);

            let resolved_port = demand
                .feed_port_name
                .as_deref()
                .or_else(|| self.resource_feed_port(&demand.consumer_name));

            let group_index = groups.iter().position(|group| {
                group.resource == demand.resource
                    && group.feed_port_name.as_deref() == resolved_port
            });
            if let Some(group_index) = group_index {
                groups[group_index].demand_indices.push(demand_index);
            } else {
                groups.push(ResourceFlowGroup {
                    resource: demand.resource,
                    feed_port_name: resolved_port.map(str::to_owned),
                    demand_indices: vec![demand_index],
                    accessible_tanks: self.feedable_tanks_for_port(resolved_port)?,
                });
            }
        }

        let mut resources = Vec::new();
        for demand in demands {
            if !resources.contains(&demand.resource) {
                resources.push(demand.resource);
            }
        }
        let mut consumer_scales = vec![1.0; consumer_names.len()];
        let iteration_limit = (groups.len() + consumer_names.len() + 1) * 4;
        for _ in 0..iteration_limit {
            let mut reserved = vec![0.0; self.tanks.len()];
            let mut next_scales = consumer_scales.clone();
            for resource in &resources {
                let resource_groups: Vec<_> = groups
                    .iter()
                    .filter(|group| group.resource == *resource)
                    .collect();
                let group_demands: Vec<_> = resource_groups
                    .iter()
                    .map(|group| {
                        group
                            .demand_indices
                            .iter()
                            .map(|index| {
                                demands[*index].mass_flow_kg_s
                                    * dt_s
                                    * consumer_scales[demand_consumers[*index]]
                            })
                            .sum::<f64>()
                    })
                    .collect();
                let total_requested_kg: f64 = group_demands.iter().sum();
                if !total_requested_kg.is_finite() {
                    return Err(VehicleError::InvalidVehicle(
                        "resource demand aggregation overflowed".into(),
                    ));
                }
                if total_requested_kg <= 0.0 {
                    continue;
                }
                let full_assignment = assign_resource_groups(
                    &self.tanks,
                    &state.tank_propellant_kg,
                    &reserved,
                    *resource,
                    &resource_groups,
                    &group_demands,
                );
                let scale = if full_assignment.0 + 1.0e-12 >= total_requested_kg {
                    1.0
                } else {
                    let mut low = 0.0;
                    let mut high = 1.0;
                    for _ in 0..48 {
                        let middle = 0.5 * (low + high);
                        let scaled_demands: Vec<_> =
                            group_demands.iter().map(|demand| demand * middle).collect();
                        let assigned = assign_resource_groups(
                            &self.tanks,
                            &state.tank_propellant_kg,
                            &reserved,
                            *resource,
                            &resource_groups,
                            &scaled_demands,
                        )
                        .0;
                        if assigned >= total_requested_kg * middle {
                            low = middle;
                        } else {
                            high = middle;
                        }
                    }
                    low
                };
                for group in &resource_groups {
                    for demand_index in &group.demand_indices {
                        let consumer = demand_consumers[*demand_index];
                        next_scales[consumer] =
                            next_scales[consumer].min(consumer_scales[consumer] * scale);
                    }
                }
                let scaled_demands: Vec<_> =
                    group_demands.iter().map(|demand| demand * scale).collect();
                let (_, tank_draw) = assign_resource_groups(
                    &self.tanks,
                    &state.tank_propellant_kg,
                    &reserved,
                    *resource,
                    &resource_groups,
                    &scaled_demands,
                );
                for (reserved, draw) in reserved.iter_mut().zip(tank_draw) {
                    *reserved += draw;
                }
            }
            let converged = consumer_scales
                .iter()
                .zip(&next_scales)
                .all(|(old, new)| (old - new).abs() <= 1.0e-12);
            consumer_scales = next_scales;
            if converged {
                break;
            }
        }

        // Rebuild a feasible tank-by-tank assignment after cross-resource
        // consumer scales settle. Each demand group can draw only from tanks
        // reachable through its named feed port.
        let mut tank_consumption_kg = vec![0.0; self.tanks.len()];
        for resource in &resources {
            let resource_groups: Vec<_> = groups
                .iter()
                .filter(|group| group.resource == *resource)
                .collect();
            let group_demands: Vec<_> = resource_groups
                .iter()
                .map(|group| {
                    group
                        .demand_indices
                        .iter()
                        .map(|index| {
                            demands[*index].mass_flow_kg_s
                                * dt_s
                                * consumer_scales[demand_consumers[*index]]
                        })
                        .sum::<f64>()
                })
                .collect();
            let requested_kg: f64 = group_demands.iter().sum();
            if !requested_kg.is_finite() {
                return Err(VehicleError::InvalidVehicle(
                    "resource demand aggregation overflowed".into(),
                ));
            }
            let (assigned_kg, tank_draw) = assign_resource_groups(
                &self.tanks,
                &state.tank_propellant_kg,
                &tank_consumption_kg,
                *resource,
                &resource_groups,
                &group_demands,
            );
            if requested_kg > assigned_kg + 1.0e-8 {
                return Err(VehicleError::InvalidVehicle(
                    "resource allocation did not converge to a feasible tank draw".into(),
                ));
            }
            for (reserved, draw) in tank_consumption_kg.iter_mut().zip(tank_draw) {
                *reserved += draw;
            }
        }

        let consumers = consumer_names
            .into_iter()
            .enumerate()
            .map(|(consumer_index, consumer_name)| {
                let requested_mass_kg: f64 = demands
                    .iter()
                    .zip(&demand_consumers)
                    .filter(|(_, index)| **index == consumer_index)
                    .map(|(demand, _)| demand.mass_flow_kg_s * dt_s)
                    .sum();
                if !requested_mass_kg.is_finite() {
                    return Err(VehicleError::InvalidVehicle(
                        "resource consumer demand aggregation overflowed".into(),
                    ));
                }
                let scale = consumer_scales[consumer_index];
                Ok(ConsumerResourceAllocation {
                    consumer_name,
                    requested_mass_kg,
                    actual_mass_kg: requested_mass_kg * scale,
                    scale,
                })
            })
            .collect::<Result<Vec<_>, VehicleError>>()?;
        let total_consumption_kg: f64 = tank_consumption_kg.iter().sum();
        if !total_consumption_kg.is_finite() {
            return Err(VehicleError::InvalidVehicle(
                "resource allocation produced a non-finite tank draw".into(),
            ));
        }
        Ok(VehicleResourcePlan {
            tank_consumption_kg,
            consumers,
            total_consumption_kg,
            dt_s,
        })
    }

    /// Commit a planned pure-fluid draw through the same moving-mass,
    /// center-of-mass, inertia, and body-frame update as rocket consumption.
    pub fn commit_resource_flows(
        &mut self,
        state: &mut VehicleResourceState,
        plan: &VehicleResourcePlan,
    ) -> Result<DVec3, VehicleError> {
        self.validate_resource_state(state)?;
        if plan.tank_consumption_kg.len() != self.tanks.len()
            || !plan.dt_s.is_finite()
            || plan.dt_s <= 0.0
        {
            return Err(VehicleError::InvalidVehicle(
                "resource plan does not match the installed tank inventory".into(),
            ));
        }
        let mut next_tanks = state.tank_propellant_kg.clone();
        for (index, consumed) in plan.tank_consumption_kg.iter().copied().enumerate() {
            if !consumed.is_finite() || consumed < 0.0 || consumed > next_tanks[index] + 1.0e-9 {
                return Err(VehicleError::InvalidVehicle(format!(
                    "resource plan overdraws tank '{}'",
                    tank_display_name(&self.tanks[index])
                )));
            }
            next_tanks[index] = (next_tanks[index] - consumed).max(0.0);
        }
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
        self.plan_propulsion_step_with_resource_demands(
            state,
            engine_throttles,
            system_chamber_throttles,
            ambient_pa,
            dt_s,
            &[],
        )
    }

    /// Plan rocket mounts and other installed operating-point consumers
    /// against one shared tank inventory. Existing rocket/chamber allocation
    /// is reserved first; accessory draws are planned from the exact remaining
    /// inventory and returned with per-consumer scales.
    pub fn plan_propulsion_step_with_resource_demands(
        &self,
        state: &VehicleResourceState,
        engine_throttles: &[f64],
        system_chamber_throttles: &[Vec<f64>],
        ambient_pa: f64,
        dt_s: f64,
        additional_demands: &[VehicleResourceDemand],
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

        let (actual_engines, actual_systems, mut tank_draw, group_scales) = self
            .allocate_throttles(
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
        let additional_resource_consumers = if additional_demands.is_empty() {
            Vec::new()
        } else {
            let mut remaining_state = state.clone();
            for (inventory, reserved) in remaining_state
                .tank_propellant_kg
                .iter_mut()
                .zip(&tank_draw)
            {
                *inventory = (*inventory - reserved).max(0.0);
            }
            let plan = self.plan_resource_flows(&remaining_state, additional_demands, dt_s)?;
            for (reserved, additional) in tank_draw.iter_mut().zip(&plan.tank_consumption_kg) {
                *reserved += additional;
            }
            fuel_limited |= plan
                .consumers
                .iter()
                .any(|consumer| consumer.scale < 1.0 - 1.0e-10);
            plan.consumers
        };
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
            additional_resource_consumers,
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

    fn feedable_tanks_for_port(
        &self,
        feed_port_name: Option<&str>,
    ) -> Result<Vec<bool>, VehicleError> {
        let Some(port_name) = feed_port_name else {
            return self.feedable_tanks();
        };
        let mut accessible = vec![false; self.tanks.len()];
        let Some(assembly) = &self.assembly else {
            accessible.fill(true);
            return Ok(accessible);
        };
        let endpoint = assembly
            .engine_ports
            .iter()
            .find(|endpoint| endpoint.name == port_name)
            .ok_or_else(|| {
                VehicleError::InvalidVehicle(format!(
                    "assembly has no engine feed port named '{port_name}'"
                ))
            })?;
        let mut reachable = vec![false; assembly.body_names.len()];
        let mut stack = vec![endpoint.body];
        reachable[endpoint.body] = true;
        while let Some(body) = stack.pop() {
            for link in &assembly.links {
                if !link.state.resource_open() {
                    continue;
                }
                let neighbor = if link.state.a == body {
                    Some(link.state.b)
                } else if link.state.b == body {
                    Some(link.state.a)
                } else {
                    None
                };
                if let Some(neighbor) = neighbor
                    && !reachable[neighbor]
                {
                    reachable[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        for (tank_index, tank) in self.tanks.iter().enumerate() {
            if let Some(tank_endpoint) = assembly
                .tanks
                .iter()
                .find(|endpoint| endpoint.name == tank.name)
            {
                accessible[tank_index] = reachable[tank_endpoint.body];
            }
        }
        Ok(accessible)
    }

    fn resource_feed_port(&self, consumer_name: &str) -> Option<&str> {
        self.resource_feed_ports
            .iter()
            .find(|route| route.consumer_name == consumer_name)
            .map(|route| route.feed_port_name.as_str())
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

fn stored_tank_resource_compatible(
    tank_resource: TankResource,
    resource: StoredPropellant,
) -> bool {
    match tank_resource {
        TankResource::Unspecified => true,
        TankResource::Stored(stored) => stored == resource,
        TankResource::Oxidizer(propellant) => match propellant {
            Propellant::LoxRp1 | Propellant::LoxMethane | Propellant::LoxHydrogen => {
                resource == StoredPropellant::Lox
            }
            Propellant::NtoMmh => resource == StoredPropellant::Nto,
            _ => false,
        },
        TankResource::Fuel(propellant) => match propellant {
            Propellant::LoxRp1 => resource == StoredPropellant::Rp1,
            Propellant::LoxMethane => resource == StoredPropellant::LiquidMethane,
            Propellant::LoxHydrogen => resource == StoredPropellant::LiquidHydrogen,
            Propellant::NtoMmh => resource == StoredPropellant::Mmh,
            Propellant::MonopropHydrazine => resource == StoredPropellant::Hydrazine,
            Propellant::ColdGasNitrogen => resource == StoredPropellant::Nitrogen,
            Propellant::ColdGasHelium => resource == StoredPropellant::Helium,
            Propellant::SolidApcp => false,
        },
        TankResource::Pair(_) => false,
    }
}

#[derive(Debug, Clone, Copy)]
struct FlowEdge {
    to: usize,
    reverse: usize,
    residual_kg: f64,
}

fn add_flow_edge(graph: &mut [Vec<FlowEdge>], from: usize, to: usize, capacity_kg: f64) -> usize {
    let forward_index = graph[from].len();
    let reverse_index = graph[to].len();
    graph[from].push(FlowEdge {
        to,
        reverse: reverse_index,
        residual_kg: capacity_kg,
    });
    graph[to].push(FlowEdge {
        to: from,
        reverse: forward_index,
        residual_kg: 0.0,
    });
    forward_index
}

/// Maximum compatible tank-to-feed-port assignment for one stored resource.
/// The return value is the assigned mass and exact tank withdrawals. A small
/// Edmonds-Karp network is sufficient here: it runs only at a vehicle fixed
/// step, and node count is bounded by installed tanks/ports.
fn assign_resource_groups(
    tanks: &[crate::TankMount],
    inventory_kg: &[f64],
    reserved_kg: &[f64],
    resource: StoredPropellant,
    groups: &[&ResourceFlowGroup],
    group_demands_kg: &[f64],
) -> (f64, Vec<f64>) {
    if groups.is_empty() || groups.len() != group_demands_kg.len() {
        return (0.0, vec![0.0; tanks.len()]);
    }
    let total_demand_kg: f64 = group_demands_kg.iter().sum();
    if !total_demand_kg.is_finite() || total_demand_kg <= 0.0 {
        return (0.0, vec![0.0; tanks.len()]);
    }
    let source = 0;
    let tank_start = 1;
    let group_start = tank_start + tanks.len();
    let sink = group_start + groups.len();
    let mut graph = vec![Vec::<FlowEdge>::new(); sink + 1];
    let mut source_edges = Vec::with_capacity(tanks.len());
    for (tank_index, tank) in tanks.iter().enumerate() {
        let available_kg = (inventory_kg[tank_index] - reserved_kg[tank_index])
            .max(0.0)
            .min(total_demand_kg);
        source_edges.push((
            source,
            add_flow_edge(&mut graph, source, tank_start + tank_index, available_kg),
        ));
        if available_kg <= 0.0 || !stored_tank_resource_compatible(tank.resource, resource) {
            continue;
        }
        for (group_index, group) in groups.iter().enumerate() {
            if group.accessible_tanks[tank_index] && group_demands_kg[group_index] > 0.0 {
                add_flow_edge(
                    &mut graph,
                    tank_start + tank_index,
                    group_start + group_index,
                    available_kg.min(group_demands_kg[group_index]),
                );
            }
        }
    }
    for (group_index, demand_kg) in group_demands_kg.iter().copied().enumerate() {
        if demand_kg > 0.0 {
            add_flow_edge(&mut graph, group_start + group_index, sink, demand_kg);
        }
    }

    let mut flow_kg = 0.0;
    loop {
        let mut parent = vec![None::<(usize, usize)>; graph.len()];
        let mut queue = std::collections::VecDeque::new();
        parent[source] = Some((source, usize::MAX));
        queue.push_back(source);
        while let Some(node) = queue.pop_front() {
            if node == sink {
                break;
            }
            for (edge_index, edge) in graph[node].iter().enumerate() {
                if edge.residual_kg > 1.0e-12 && parent[edge.to].is_none() {
                    parent[edge.to] = Some((node, edge_index));
                    queue.push_back(edge.to);
                }
            }
        }
        if parent[sink].is_none() {
            break;
        }
        let mut augmentation_kg = f64::INFINITY;
        let mut node = sink;
        while node != source {
            let (previous, edge_index) = parent[node].expect("augmenting path parent");
            augmentation_kg = augmentation_kg.min(graph[previous][edge_index].residual_kg);
            node = previous;
        }
        if !augmentation_kg.is_finite() || augmentation_kg <= 1.0e-12 {
            break;
        }
        node = sink;
        while node != source {
            let (previous, edge_index) = parent[node].expect("augmenting path parent");
            let reverse = graph[previous][edge_index].reverse;
            graph[previous][edge_index].residual_kg -= augmentation_kg;
            graph[node][reverse].residual_kg += augmentation_kg;
            node = previous;
        }
        flow_kg += augmentation_kg;
    }

    let mut tank_draw_kg = vec![0.0; tanks.len()];
    for (tank_index, (node, edge_index)) in source_edges.into_iter().enumerate() {
        let initial_capacity = (inventory_kg[tank_index] - reserved_kg[tank_index])
            .max(0.0)
            .min(total_demand_kg);
        tank_draw_kg[tank_index] =
            (initial_capacity - graph[node][edge_index].residual_kg).max(0.0);
    }
    (flow_kg, tank_draw_kg)
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
    fn generic_multireactant_consumers_share_one_scale_and_update_mass_frame() {
        let capacity_kg = std::f64::consts::PI / 6.0 * 0.5_f64.powi(3) * 1_000.0;
        let deuterium = tank(
            "deuterium",
            TankResource::Stored(StoredPropellant::Deuterium),
            1_000.0,
            1.0,
            DVec3::new(-1.0, 0.0, 0.0),
        );
        let mut tritium = tank(
            "tritium",
            TankResource::Stored(StoredPropellant::Tritium),
            1_000.0,
            1.0,
            DVec3::new(1.0, 0.0, 0.0),
        );
        tritium.initial_propellant_kg = Some(0.3);
        tritium.intrinsic_inertia_body_kg_m2 = tritium
            .tank
            .shape
            .expect("test tank shape")
            .intrinsic_inertia_body_kg_m2(tritium.tank.dry_mass_kg, 0.3)
            .expect("filled-tank inertia");
        assert!(0.3 < capacity_kg);

        let mut vehicle = vehicle(vec![deuterium, tritium], 0);
        let mut state = vehicle.initial_resource_state();
        let initial_mass_kg = vehicle.mass_properties.mass_kg;
        let plan = vehicle
            .plan_resource_flows(
                &state,
                &[
                    VehicleResourceDemand::new("fusion-core", StoredPropellant::Deuterium, 0.4),
                    VehicleResourceDemand::new("fusion-core", StoredPropellant::Tritium, 0.6),
                ],
                1.0,
            )
            .expect("plan a paired isotope draw");

        assert!(
            (plan.consumers[0].scale - 0.5).abs() < 1.0e-12,
            "consumer scale = {}",
            plan.consumers[0].scale
        );
        assert!((plan.tank_consumption_kg[0] - 0.2).abs() < 1.0e-12);
        assert!((plan.tank_consumption_kg[1] - 0.3).abs() < 1.0e-12);
        let frame_shift = vehicle
            .commit_resource_flows(&mut state, &plan)
            .expect("commit resource mass change");
        assert!((initial_mass_kg - vehicle.mass_properties.mass_kg - 0.5).abs() < 1.0e-9);
        assert!((state.total_tank_propellant_kg() - (capacity_kg - 0.2)).abs() < 1.0e-9);
        assert!(frame_shift.is_finite());
        assert!(vehicle.mass_properties.inertia_body_kg_m2.is_finite());
    }

    #[test]
    fn fuel_cell_draws_stoichiometric_hydrogen_and_lox_from_installed_tanks() {
        let mut vehicle = vehicle(
            vec![
                tank(
                    "fuel-cell-hydrogen",
                    TankResource::Stored(StoredPropellant::LiquidHydrogen),
                    71.0,
                    1.0,
                    DVec3::new(-0.5, 0.0, 0.0),
                ),
                tank(
                    "fuel-cell-oxygen",
                    TankResource::Stored(StoredPropellant::Lox),
                    1_141.0,
                    1.0,
                    DVec3::new(0.5, 0.0, 0.0),
                ),
            ],
            0,
        )
        .with_electrical_power(crate::ElectricalPowerSystem {
            fuel_cells: vec![crate::FuelCellSpec {
                name: "service-cell".into(),
                rated_electrical_power_w: 1_000.0,
                electrical_efficiency: 0.5,
                dry_mass_kg: 20.0,
                dimensions_body_m: DVec3::splat(0.5),
                position_body_m: DVec3::ZERO,
                feed_port_name: None,
            }],
            consumers: vec![crate::PowerConsumerSpec {
                name: "life-support".into(),
                rated_power_w: 500.0,
                priority: crate::PowerPriority::LifeSupport,
            }],
            ..crate::ElectricalPowerSystem::default()
        })
        .expect("fuel cell system");
        vehicle
            .bake_electrical_power_masses()
            .expect("fuel-cell mass");
        let mut resource_state = vehicle.initial_resource_state();
        let initial_inventory = resource_state.tank_propellant_kg.clone();
        let initial_mass_kg = vehicle.mass_properties.mass_kg;
        let power_state = vehicle.initial_electrical_power_state().unwrap();
        let mut command = crate::ElectricalPowerCommand::idle_for(&vehicle.electrical_power, 1.0);
        command.consumer_power_w = vec![500.0];
        assert!(
            vehicle
                .advance_electrical_power(&power_state, &command)
                .is_err()
        );

        let (_, telemetry, frame_shift) = vehicle
            .advance_electrical_power_with_resources(&mut resource_state, &power_state, &command)
            .expect("fuel cell and tank inventory share one transaction");
        let cell = &telemetry.fuel_cells[0];
        let hydrogen_draw = initial_inventory[0] - resource_state.tank_propellant_kg[0];
        let oxygen_draw = initial_inventory[1] - resource_state.tank_propellant_kg[1];
        assert!(telemetry.fuel_cell_output_power_w > 0.0);
        assert!((hydrogen_draw - cell.hydrogen_flow_kg_s).abs() < 1.0e-12);
        assert!((oxygen_draw - cell.oxygen_flow_kg_s).abs() < 1.0e-12);
        assert!((oxygen_draw / hydrogen_draw - 8.0).abs() < 1.0e-8);
        assert!(
            (initial_mass_kg - vehicle.mass_properties.mass_kg - hydrogen_draw - oxygen_draw).abs()
                < 1.0e-9
        );
        assert!(frame_shift.is_finite());
    }

    #[test]
    fn fuel_cells_reserve_reactants_after_external_demands_in_one_mass_commit() {
        let hydrogen_flow_for_full_cell = 1_000.0 / (0.5 * crate::FUEL_CELL_HYDROGEN_LHV_J_KG);
        let mut hydrogen = tank(
            "shared-hydrogen",
            TankResource::Stored(StoredPropellant::LiquidHydrogen),
            71.0,
            1.0,
            DVec3::new(-0.5, 0.0, 0.0),
        );
        hydrogen.initial_propellant_kg = Some(hydrogen_flow_for_full_cell * 1.5);
        let mut oxygen = tank(
            "fuel-cell-oxygen",
            TankResource::Stored(StoredPropellant::Lox),
            1_141.0,
            1.0,
            DVec3::new(0.5, 0.0, 0.0),
        );
        oxygen.initial_propellant_kg = Some(hydrogen_flow_for_full_cell * 8.0 * 2.0);
        let mut vehicle = vehicle(vec![hydrogen, oxygen], 0)
            .with_electrical_power(crate::ElectricalPowerSystem {
                fuel_cells: vec![crate::FuelCellSpec {
                    name: "service-cell".into(),
                    rated_electrical_power_w: 1_000.0,
                    electrical_efficiency: 0.5,
                    dry_mass_kg: 20.0,
                    dimensions_body_m: DVec3::splat(0.5),
                    position_body_m: DVec3::ZERO,
                    feed_port_name: None,
                }],
                consumers: vec![crate::PowerConsumerSpec {
                    name: "life-support".into(),
                    rated_power_w: 1_000.0,
                    priority: crate::PowerPriority::LifeSupport,
                }],
                ..crate::ElectricalPowerSystem::default()
            })
            .expect("fuel-cell vehicle");
        vehicle
            .bake_electrical_power_masses()
            .expect("fuel-cell mass");
        let mut resources = vehicle.initial_resource_state();
        let initial_tanks = resources.tank_propellant_kg.clone();
        let initial_mass_kg = vehicle.mass_properties.mass_kg;
        let power_state = vehicle.initial_electrical_power_state().unwrap();
        let mut command = crate::ElectricalPowerCommand::idle_for(&vehicle.electrical_power, 1.0);
        command.consumer_power_w = vec![1_000.0];
        let external_hydrogen_flow = hydrogen_flow_for_full_cell;
        let external_demands = [
            VehicleResourceDemand::new(
                "hydrogen-thruster",
                StoredPropellant::LiquidHydrogen,
                external_hydrogen_flow,
            ),
            VehicleResourceDemand::new(
                "hydrogen-thruster",
                StoredPropellant::Lox,
                external_hydrogen_flow * 8.0,
            ),
        ];
        let (_, telemetry, _) = vehicle
            .advance_electrical_power_with_demands(
                &mut resources,
                &power_state,
                &command,
                &external_demands,
            )
            .expect("one shared reactant transaction");
        let cell = &telemetry.fuel_cells[0];
        assert!((cell.electrical_power_w - 500.0).abs() < 1.0e-8);
        assert!((cell.hydrogen_flow_kg_s - hydrogen_flow_for_full_cell * 0.5).abs() < 1.0e-16);
        let hydrogen_draw = initial_tanks[0] - resources.tank_propellant_kg[0];
        let oxygen_draw = initial_tanks[1] - resources.tank_propellant_kg[1];
        assert!((hydrogen_draw - external_hydrogen_flow - cell.hydrogen_flow_kg_s).abs() < 1.0e-16);
        assert!(
            (oxygen_draw - external_hydrogen_flow * 8.0 - cell.oxygen_flow_kg_s).abs() < 1.0e-15
        );
        assert!(
            (initial_mass_kg - vehicle.mass_properties.mass_kg - hydrogen_draw - oxygen_draw).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn installed_apu_reports_actual_generator_power_and_consumes_reachable_jet_fuel() {
        let unit = crate::AuxiliaryPowerUnitSpec {
            name: "service-apu".into(),
            engine: crate::AirbreathingSpec {
                name: "service-apu-turbine".into(),
                cycle: crate::AirCycle::Turbojet,
                fuel: crate::JetFuel::Kerosene,
                intake_area_m2: 0.2,
                intake: crate::IntakeKind::Pitot,
                compressor_ratio: 8.0,
                bypass_ratio: 0.0,
                fan_pressure_ratio: 1.0,
                turbine_inlet_temp_k: 1_350.0,
                afterburner: false,
                reheat_temp_k: 0.0,
                turbine_material: ChamberMaterial::nickel_superalloy(),
                spool_tau_s: 3.0,
                shaft: crate::ShaftSpec {
                    generator: crate::GeneratorSpec {
                        fitted: true,
                        power_w: 5_000.0,
                        efficiency: 0.9,
                        cut_in_spool_n: 0.5,
                        mass_kg: 2.0,
                    },
                    ..crate::ShaftSpec::default()
                },
            },
        }
        .compile()
        .expect("compiled APU");
        let mut vehicle = vehicle(
            vec![tank(
                "apu-fuel",
                TankResource::Stored(StoredPropellant::Rp1),
                810.0,
                1.0,
                DVec3::X,
            )],
            0,
        )
        .with_auxiliary_power_units(vec![crate::AuxiliaryPowerUnitMount {
            name: "service-apu".into(),
            unit: unit.clone(),
            position_body_m: DVec3::ZERO.to_array(),
            thrust_axis_body: DVec3::X.to_array(),
            feed_port_name: None,
        }])
        .expect("install APU");
        vehicle
            .bake_auxiliary_power_unit_masses()
            .expect("APU mass");
        let mut resources = vehicle.initial_resource_state();
        let initial_mass_kg = vehicle.mass_properties.mass_kg;
        let atmosphere = crate::AtmosphereConfig::default();
        let sample = atmosphere.sample(0.0).expect("sea-level atmosphere");
        let condition = crate::flight_condition(&sample, 0.0).expect("static condition");
        let states = [crate::AuxiliaryPowerUnitState {
            shaft: crate::JetShaftState::running(&unit.engine),
        }];
        let commands = [crate::AuxiliaryPowerUnitCommand {
            throttle: 1.0,
            starter_engaged: false,
            generator_load_w: 5_000.0,
            pneumatic_bleed_power_w: 0.0,
            dt_s: 0.1,
        }];
        let step = vehicle
            .plan_auxiliary_power_units(&resources, &states, &commands, &condition)
            .expect("fuel-limited APU operating point");
        assert!(step.generated_electrical_power_w > 0.0);
        assert!(step.operating_points[0].fuel_flow_kg_s > 0.0);
        assert_eq!(
            step.generated_electrical_power_w,
            step.operating_points[0].electrical_power_w
        );
        assert_eq!(step.resource_plan.consumers[0].scale, 1.0);
        vehicle
            .commit_resource_flows(&mut resources, &step.resource_plan)
            .expect("commit actual APU fuel flow");
        assert!(
            (initial_mass_kg
                - vehicle.mass_properties.mass_kg
                - step.operating_points[0].fuel_flow_kg_s * commands[0].dt_s)
                .abs()
                < 1.0e-9
        );
        assert!(step.next_states[0].shaft.spool_n <= 1.0);
    }

    #[test]
    fn named_feed_port_limits_a_consumer_to_its_reachable_tanks() {
        let tanks = vec![
            tank(
                "core-hydrogen",
                TankResource::Stored(StoredPropellant::LiquidHydrogen),
                71.0,
                1.0,
                DVec3::ZERO,
            ),
            tank(
                "booster-hydrogen",
                TankResource::Stored(StoredPropellant::LiquidHydrogen),
                71.0,
                1.0,
                DVec3::X,
            ),
        ];
        let vehicle = vehicle(tanks, 0)
            .with_assembly(VehicleAssembly {
                root_body: 0,
                body_names: vec!["core".into(), "booster".into()],
                links: vec![NamedAssemblyLink {
                    name: "closed-hatch".into(),
                    state: AssemblyLinkState {
                        a: 0,
                        b: 1,
                        hatch: true,
                        open: false,
                    },
                }],
                volumes: Vec::new(),
                tanks: vec![
                    AssemblyEndpoint {
                        name: "core-hydrogen".into(),
                        body: 0,
                    },
                    AssemblyEndpoint {
                        name: "booster-hydrogen".into(),
                        body: 1,
                    },
                ],
                engine_ports: vec![
                    AssemblyEndpoint {
                        name: "core.feed".into(),
                        body: 0,
                    },
                    AssemblyEndpoint {
                        name: "booster.feed".into(),
                        body: 1,
                    },
                ],
            })
            .expect("valid assembly")
            .with_resource_feed_ports(vec![crate::VehicleResourceFeedPort {
                consumer_name: "ntr-core".into(),
                feed_port_name: "core.feed".into(),
            }])
            .expect("named consumer route");
        let state = vehicle.initial_resource_state();
        let core_draw = vehicle
            .plan_resource_flows(
                &state,
                &[VehicleResourceDemand::new(
                    "ntr-core",
                    StoredPropellant::LiquidHydrogen,
                    0.1,
                )],
                0.1,
            )
            .expect("core feed allocation");
        assert!(core_draw.tank_consumption_kg[0] > 0.0);
        assert_eq!(core_draw.tank_consumption_kg[1], 0.0);

        let booster_draw = vehicle
            .plan_resource_flows(
                &state,
                &[VehicleResourceDemand::new(
                    "booster-engine",
                    StoredPropellant::LiquidHydrogen,
                    0.1,
                )
                .through_feed_port("booster.feed")],
                0.1,
            )
            .expect("booster feed allocation");
        assert_eq!(booster_draw.tank_consumption_kg[0], 0.0);
        assert!(booster_draw.tank_consumption_kg[1] > 0.0);
    }

    #[test]
    fn overlapping_named_feed_ports_share_one_tank_proportionally() {
        let mut shared = tank(
            "shared-hydrogen",
            TankResource::Stored(StoredPropellant::LiquidHydrogen),
            71.0,
            1.0,
            DVec3::ZERO,
        );
        shared.initial_propellant_kg = Some(1.0);
        let vehicle = vehicle(vec![shared], 0)
            .with_assembly(VehicleAssembly {
                root_body: 0,
                body_names: vec!["core".into(), "upper".into()],
                links: vec![NamedAssemblyLink {
                    name: "open-stack".into(),
                    state: AssemblyLinkState {
                        a: 0,
                        b: 1,
                        hatch: false,
                        open: true,
                    },
                }],
                volumes: Vec::new(),
                tanks: vec![AssemblyEndpoint {
                    name: "shared-hydrogen".into(),
                    body: 0,
                }],
                engine_ports: vec![
                    AssemblyEndpoint {
                        name: "core.feed".into(),
                        body: 0,
                    },
                    AssemblyEndpoint {
                        name: "upper.feed".into(),
                        body: 1,
                    },
                ],
            })
            .expect("shared-feed assembly");
        let state = vehicle.initial_resource_state();
        let plan = vehicle
            .plan_resource_flows(
                &state,
                &[
                    VehicleResourceDemand::new(
                        "core-engine",
                        StoredPropellant::LiquidHydrogen,
                        1.0,
                    )
                    .through_feed_port("core.feed"),
                    VehicleResourceDemand::new(
                        "upper-engine",
                        StoredPropellant::LiquidHydrogen,
                        1.0,
                    )
                    .through_feed_port("upper.feed"),
                ],
                1.0,
            )
            .expect("proportional allocation through overlapping ports");
        assert!((plan.consumers[0].scale - 0.5).abs() < 1.0e-12);
        assert!((plan.consumers[1].scale - 0.5).abs() < 1.0e-12);
        assert!((plan.tank_consumption_kg[0] - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn fusion_reactant_demands_preserve_stoichiometric_mass_fractions() {
        let demands =
            fusion_reactant_demands("torch", crate::FusionReaction::DeuteriumTritium, 1.0, None);
        assert_eq!(demands.len(), 2);
        assert_eq!(demands[0].resource, StoredPropellant::Deuterium);
        assert_eq!(demands[1].resource, StoredPropellant::Tritium);
        assert!((demands[0].mass_flow_kg_s + demands[1].mass_flow_kg_s - 1.0).abs() < 1.0e-15);
        assert!((demands[0].mass_flow_kg_s / demands[1].mass_flow_kg_s - 2.0 / 3.0).abs() < 0.01);
    }

    #[test]
    fn mounted_cold_gas_pulse_commits_its_actual_propellant_mass() {
        let rcs_mount = crate::RcsMount {
            name: "translation-rcs".into(),
            thruster: crate::RcsThruster::ColdGas(
                crate::ColdGasThrusterSpec {
                    name: "nitrogen-nozzle".into(),
                    gas: Propellant::ColdGasNitrogen,
                    storage_temp_k: 300.0,
                    rated_pressure_pa: 2.0e6,
                    throat_radius_m: 0.001,
                    expansion_ratio: 10.0,
                    nozzle_length_m: 0.02,
                    contour: crate::NozzleContour::Conical,
                    material: crate::ChamberMaterial::nickel_superalloy(),
                    valve_rise_time_s: 0.005,
                    min_on_time_s: 0.02,
                }
                .compile()
                .expect("cold-gas thruster"),
            ),
            position_body_m: DVec3::X.to_array(),
            direction_body: DVec3::Y.to_array(),
        };
        let mut nitrogen_tank = tank(
            "rcs-nitrogen",
            TankResource::Stored(StoredPropellant::Nitrogen),
            25.0,
            1.0,
            -DVec3::X,
        );
        nitrogen_tank.initial_propellant_kg = Some(0.5);
        let mut vehicle = vehicle(vec![nitrogen_tank], 0)
            .with_rcs_mounts(vec![rcs_mount.clone()])
            .expect("install RCS mount");
        let mut state = vehicle.initial_resource_state();
        let dt_s = 0.1;
        let pulse = rcs_mount
            .thruster
            .pulse(dt_s, 2.0e6, 0.0)
            .expect("finite RCS pulse");
        let demands = VehicleResourceDemand::from_rcs_pulse(&rcs_mount, pulse, dt_s, None)
            .expect("RCS pulse resource flow");
        let plan = vehicle
            .plan_resource_flows(&state, &demands, dt_s)
            .expect("reachable nitrogen draw");
        assert!((plan.total_consumption_kg - pulse.propellant_kg).abs() < 1.0e-15);
        let initial_mass_kg = vehicle.mass_properties.mass_kg;
        vehicle
            .commit_resource_flows(&mut state, &plan)
            .expect("commit mounted RCS pulse");
        assert!((state.tank_propellant_kg[0] - (0.5 - pulse.propellant_kg)).abs() < 1.0e-15);
        assert!(
            (initial_mass_kg - vehicle.mass_properties.mass_kg - pulse.propellant_kg).abs() < 1e-9
        );
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
