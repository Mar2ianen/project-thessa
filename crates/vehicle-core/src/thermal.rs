//! Lumped thermal-node network: conduction, radiation, and radiators.
//!
//! Each node is one isothermal lump with heat capacity `mass * cp`. Nodes
//! exchange heat through conduction links, radiate to the cold background,
//! absorb sunlight through the same occluded [`SolarFluxSource`] inputs as
//! the power bus, pick up stagnation aero heating, and accept explicit
//! internal loads (engine/reactor waste heat wired by the caller). Radiators
//! are area devices tied to a node. There is no ablation, no heat shields,
//! and no automatic damage: overheating is reported, never auto-exploded.

use glam::{DMat3, DVec3};
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

use crate::SolarFluxSource;

const STEFAN_BOLTZMANN_W_M2_K4: f64 = 5.670_374_419e-8;
const SPACE_BACKGROUND_TEMP_K: f64 = 2.725;
/// Hard cap on internal stability substeps per call. Beyond it the step is
/// rejected instead of silently integrating garbage.
const MAX_THERMAL_SUBSTEPS: usize = 4_096;

#[derive(Debug, Clone, PartialEq)]
pub enum ThermalError {
    InvalidSpec(String),
    InvalidCommand(String),
    InvalidState(String),
}

impl fmt::Display for ThermalError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid thermal spec: {message}"),
            Self::InvalidCommand(message) => {
                write!(formatter, "invalid thermal command: {message}")
            }
            Self::InvalidState(message) => write!(formatter, "invalid thermal state: {message}"),
        }
    }
}

impl Error for ThermalError {}

/// Default Sutton-Graves stagnation correlation constant for Earth air
/// (W/m^2 per sqrt(kg/m^3/m) per (m/s)^3). Alien atmospheres override it
/// per system; it is authored, never inferred.
pub fn default_convective_k() -> f64 {
    1.83e-4
}

/// One isothermal lump: structure, tank wall, avionics box, reactor block.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalNodeSpec {
    pub name: String,
    pub mass_kg: f64,
    pub specific_heat_j_kg_k: f64,
    pub initial_temp_k: f64,
    pub max_temp_k: f64,
    pub emissivity: f64,
    pub solar_absorptivity: f64,
    /// Total area radiating to the background (m^2).
    pub radiating_area_m2: f64,
    /// Flat-plate area facing `solar_normal_body` (m^2, 0 = no solar load).
    pub solar_exposed_area_m2: f64,
    pub solar_normal_body: DVec3,
    /// Windward area for stagnation aero heating (m^2, 0 = none).
    pub aero_area_m2: f64,
    /// Effective nose radius for the Sutton-Graves correlation (m).
    pub nose_radius_m: f64,
    pub position_body_m: DVec3,
}

impl ThermalNodeSpec {
    pub fn validate(&self) -> Result<(), ThermalError> {
        if self.name.trim().is_empty() {
            return Err(ThermalError::InvalidSpec(
                "thermal node name must not be empty".into(),
            ));
        }
        for (value, label) in [
            (self.mass_kg, "thermal node mass"),
            (self.specific_heat_j_kg_k, "thermal node specific heat"),
            (self.nose_radius_m, "thermal node nose radius"),
        ] {
            require_positive(value, label)?;
        }
        for (value, label) in [
            (self.initial_temp_k, "thermal node initial temperature"),
            (self.max_temp_k, "thermal node maximum temperature"),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal node {label} must be finite and positive"
                )));
            }
        }
        if self.max_temp_k < self.initial_temp_k {
            return Err(ThermalError::InvalidSpec(format!(
                "thermal node '{}' starts above its maximum temperature",
                self.name
            )));
        }
        require_emissivity(self.emissivity, "thermal node emissivity")?;
        require_absorptivity(self.solar_absorptivity, "thermal node absorptivity")?;
        for (value, label) in [
            (self.radiating_area_m2, "thermal node radiating area"),
            (self.solar_exposed_area_m2, "thermal node solar area"),
            (self.aero_area_m2, "thermal node aero area"),
        ] {
            require_non_negative(value, label)?;
        }
        validate_unit_vector(self.solar_normal_body, "thermal node solar normal")?;
        validate_finite_vector(self.position_body_m, "thermal node position")
    }

    pub fn heat_capacity_j_k(&self) -> f64 {
        self.mass_kg * self.specific_heat_j_kg_k
    }
}

/// Conductive edge between two named nodes (W/K, symmetric).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalLinkSpec {
    pub node_a: String,
    pub node_b: String,
    pub conductance_w_k: f64,
}

/// Area radiator tied to one node: rejects `εσA(T_node^4 − T_bg^4)` and
/// absorbs sunlight on the same area with its own absorptivity/normal.
/// Fixed radiators stay exposed; foldable ones change effective area at
/// their authored deployment rate, drawing bus power booked by the caller
/// through `radiator_power_fraction` (same pattern as electric-thruster
/// available power: thermal reports the request, power allocates it).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RadiatorSpec {
    pub name: String,
    pub attached_node: String,
    pub area_m2: f64,
    pub emissivity: f64,
    pub solar_absorptivity: f64,
    pub normal_body: DVec3,
    /// Panel areal density used to derive installed mass (kg/m^2).
    pub areal_density_kg_m2: f64,
    pub position_body_m: DVec3,
    #[serde(default)]
    pub deployment: RadiatorDeployment,
}

/// Fixed radiators are permanently exposed. Foldable radiators change
/// effective area at their authored deployment rate and consume bus power
/// while moving.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum RadiatorDeployment {
    #[default]
    Fixed,
    Foldable {
        deployment_rate_per_s: f64,
        actuator_power_w: f64,
        initial_fraction: f64,
    },
}

impl RadiatorDeployment {
    fn initial_fraction(&self) -> f64 {
        match self {
            Self::Fixed => 1.0,
            Self::Foldable {
                initial_fraction, ..
            } => *initial_fraction,
        }
    }
}

impl RadiatorSpec {
    pub fn validate(&self) -> Result<(), ThermalError> {
        if self.name.trim().is_empty() {
            return Err(ThermalError::InvalidSpec(
                "radiator name must not be empty".into(),
            ));
        }
        if self.attached_node.trim().is_empty() {
            return Err(ThermalError::InvalidSpec(format!(
                "radiator '{}' needs an attached node",
                self.name
            )));
        }
        require_positive(self.area_m2, "radiator area")?;
        require_emissivity(self.emissivity, "radiator emissivity")?;
        require_absorptivity(self.solar_absorptivity, "radiator absorptivity")?;
        validate_unit_vector(self.normal_body, "radiator normal")?;
        require_positive(self.areal_density_kg_m2, "radiator areal density")?;
        validate_finite_vector(self.position_body_m, "radiator position")?;
        match self.deployment {
            RadiatorDeployment::Fixed => Ok(()),
            RadiatorDeployment::Foldable {
                deployment_rate_per_s,
                actuator_power_w,
                initial_fraction,
            } => {
                require_positive(deployment_rate_per_s, "radiator deployment rate")?;
                require_positive(actuator_power_w, "radiator deployment actuator power")?;
                if !initial_fraction.is_finite() || !(0.0..=1.0).contains(&initial_fraction) {
                    return Err(ThermalError::InvalidSpec(format!(
                        "radiator '{}' initial deployment must be in [0, 1]",
                        self.name
                    )));
                }
                Ok(())
            }
        }
    }

    pub fn mass_kg(&self) -> f64 {
        self.area_m2 * self.areal_density_kg_m2
    }
}

/// Local airflow for stagnation heating, wired by the caller from the
/// authoritative atmosphere/aero sampling. `None` means vacuum: no aero load.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct ThermalFlowCondition {
    pub density_kg_m3: f64,
    pub speed_mps: f64,
}

impl ThermalFlowCondition {
    fn validate(&self) -> Result<(), ThermalError> {
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 < 0.0 {
            return Err(ThermalError::InvalidCommand(
                "thermal flow density must be finite and non-negative".into(),
            ));
        }
        if !self.speed_mps.is_finite() || self.speed_mps < 0.0 {
            return Err(ThermalError::InvalidCommand(
                "thermal flow speed must be finite and non-negative".into(),
            ));
        }
        Ok(())
    }
}

/// Static thermal authoring for one vessel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThermalSystem {
    pub nodes: Vec<ThermalNodeSpec>,
    pub links: Vec<ThermalLinkSpec>,
    pub radiators: Vec<RadiatorSpec>,
    /// Sutton-Graves correlation constant for the operating atmosphere.
    pub convective_k: f64,
}

impl Default for ThermalSystem {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            links: Vec::new(),
            radiators: Vec::new(),
            convective_k: default_convective_k(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ThermalMassProperties {
    pub mass_kg: f64,
    pub center_of_mass_body_m: DVec3,
    pub inertia_body_kg_m2: DMat3,
}

impl ThermalSystem {
    pub fn validate(&self) -> Result<(), ThermalError> {
        let mut names = std::collections::HashSet::new();
        for node in &self.nodes {
            node.validate()?;
            if !names.insert(node.name.as_str()) {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal part name '{}' is duplicated",
                    node.name
                )));
            }
        }
        for radiator in &self.radiators {
            radiator.validate()?;
            if !names.insert(radiator.name.as_str()) {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal part name '{}' is duplicated",
                    radiator.name
                )));
            }
            if !self
                .nodes
                .iter()
                .any(|node| node.name == radiator.attached_node)
            {
                return Err(ThermalError::InvalidSpec(format!(
                    "radiator '{}' attaches to unknown node '{}'",
                    radiator.name, radiator.attached_node
                )));
            }
        }
        let node_known = |name: &str| self.nodes.iter().any(|node| node.name == name);
        let mut pairs = std::collections::HashSet::new();
        for link in &self.links {
            if !node_known(&link.node_a) {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal link references unknown node '{}'",
                    link.node_a
                )));
            }
            if !node_known(&link.node_b) {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal link references unknown node '{}'",
                    link.node_b
                )));
            }
            if link.node_a == link.node_b {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal link on node '{}' connects the node to itself",
                    link.node_a
                )));
            }
            require_positive(link.conductance_w_k, "thermal link conductance")?;
            let (first, second) = if link.node_a < link.node_b {
                (link.node_a.as_str(), link.node_b.as_str())
            } else {
                (link.node_b.as_str(), link.node_a.as_str())
            };
            if !pairs.insert((first, second)) {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal link '{first}'–'{second}' is duplicated"
                )));
            }
        }
        if !self.nodes.is_empty() {
            require_positive(self.convective_k, "thermal convective constant")?;
        }
        Ok(())
    }

    fn node_index(&self, name: &str) -> Option<usize> {
        self.nodes.iter().position(|node| node.name == name)
    }

    pub fn initial_state(&self) -> Result<ThermalState, ThermalError> {
        self.validate()?;
        Ok(ThermalState {
            node_temp_k: self.nodes.iter().map(|node| node.initial_temp_k).collect(),
            radiator_deployed_fraction: self
                .radiators
                .iter()
                .map(|radiator| radiator.deployment.initial_fraction())
                .collect(),
        })
    }

    /// Point-mass aggregation (nodes + radiator panels at their stations).
    /// Documented simplification for this slice: small thermal hardware is
    /// not given shape inertia.
    pub fn mass_properties(&self) -> Result<ThermalMassProperties, ThermalError> {
        self.validate()?;
        let mut mass_kg = 0.0;
        let mut first_moment = DVec3::ZERO;
        let mut inertia_about_origin = DMat3::ZERO;
        for (mass, position) in self
            .nodes
            .iter()
            .map(|node| (node.mass_kg, node.position_body_m))
            .chain(
                self.radiators
                    .iter()
                    .map(|radiator| (radiator.mass_kg(), radiator.position_body_m)),
            )
        {
            mass_kg += mass;
            first_moment += position * mass;
            inertia_about_origin += parallel_axis(mass, position);
        }
        if !mass_kg.is_finite() || !first_moment.is_finite() || !inertia_about_origin.is_finite() {
            return Err(ThermalError::InvalidSpec(
                "thermal mass aggregation overflowed".into(),
            ));
        }
        if mass_kg == 0.0 {
            return Ok(ThermalMassProperties {
                mass_kg: 0.0,
                center_of_mass_body_m: DVec3::ZERO,
                inertia_body_kg_m2: DMat3::ZERO,
            });
        }
        let center = first_moment / mass_kg;
        Ok(ThermalMassProperties {
            mass_kg,
            center_of_mass_body_m: center,
            inertia_body_kg_m2: inertia_about_origin - parallel_axis(mass_kg, center),
        })
    }

    /// Advance all nodes by one positive simulation-time step (explicit Euler
    /// with internal stability substepping). Transactional: failure leaves the
    /// caller's state untouched.
    pub fn advance(
        &self,
        state: &ThermalState,
        command: &ThermalCommand,
    ) -> Result<(ThermalState, ThermalTelemetry), ThermalError> {
        self.validate()?;
        state.validate_for(self)?;
        command.validate_for(self)?;

        // Radiator deployment planning: rate-limited targets become a bus
        // power request; the caller books it on the power bus and returns
        // the granted share via `radiator_power_fraction`.
        let mut radiator_motor_request_w = vec![0.0; self.radiators.len()];
        let mut radiator_planned = state.radiator_deployed_fraction.clone();
        for (index, (radiator, target)) in self
            .radiators
            .iter()
            .zip(&command.radiator_deployment_targets)
            .enumerate()
        {
            if let (
                RadiatorDeployment::Foldable {
                    deployment_rate_per_s,
                    actuator_power_w,
                    ..
                },
                Some(target),
            ) = (radiator.deployment, target)
            {
                let current = state.radiator_deployed_fraction[index];
                let maximum_delta = deployment_rate_per_s * command.dt_s;
                let planned_delta = (target - current).clamp(-maximum_delta, maximum_delta);
                let duty = if maximum_delta > 0.0 {
                    (planned_delta.abs() / maximum_delta).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                radiator_planned[index] = current + planned_delta;
                radiator_motor_request_w[index] = actuator_power_w * duty;
            }
        }

        let link_edges: Vec<(usize, usize, f64)> = self
            .links
            .iter()
            .map(|link| {
                (
                    self.node_index(&link.node_a).expect("validated link"),
                    self.node_index(&link.node_b).expect("validated link"),
                    link.conductance_w_k,
                )
            })
            .collect();
        // Explicit Euler uses the exposed fraction at the beginning of the
        // step; a deploying radiator contributes from the following step.
        let radiator_edges: Vec<(usize, &RadiatorSpec, f64)> = self
            .radiators
            .iter()
            .zip(&state.radiator_deployed_fraction)
            .map(|(radiator, deployed)| {
                (
                    self.node_index(&radiator.attached_node)
                        .expect("validated node"),
                    radiator,
                    *deployed,
                )
            })
            .collect();

        // Conservative stability bound from the hottest plausible slope:
        // radiation linearized at max(current, rated) plus all links.
        let substeps = self.stability_substeps(
            &state.node_temp_k,
            &link_edges,
            &radiator_edges,
            command.dt_s,
        )?;
        let h = command.dt_s / substeps as f64;

        let mut temps = state.node_temp_k.clone();
        let mut acc = Accumulator::new(self.nodes.len());
        for _ in 0..substeps {
            let rates = self.step_rates(&temps, &link_edges, &radiator_edges, command);
            for (index, node) in self.nodes.iter().enumerate() {
                temps[index] += rates.dtemp_dt[index] * h;
                acc.add(
                    index,
                    &rates,
                    h,
                    command.internal_heat_w[index],
                    node.heat_capacity_j_k(),
                );
            }
        }
        if temps.iter().any(|temp| !temp.is_finite()) {
            return Err(ThermalError::InvalidState(
                "thermal integration produced non-finite temperature".into(),
            ));
        }
        let mut next = state.clone();
        next.node_temp_k = temps;
        let mut radiator_telemetry = Vec::with_capacity(self.radiators.len());
        let mut radiator_deployment_power_w = 0.0;
        for (index, radiator) in self.radiators.iter().enumerate() {
            let current = state.radiator_deployed_fraction[index];
            let granted = command.radiator_power_fraction[index].clamp(0.0, 1.0);
            let planned = radiator_planned[index];
            let final_fraction = match radiator.deployment {
                RadiatorDeployment::Fixed => 1.0,
                RadiatorDeployment::Foldable {
                    deployment_rate_per_s,
                    ..
                } => {
                    let delta = planned - current;
                    let allowed = deployment_rate_per_s * command.dt_s * granted;
                    (current + delta.signum() * delta.abs().min(allowed)).clamp(0.0, 1.0)
                }
            };
            next.radiator_deployed_fraction[index] = final_fraction;
            radiator_deployment_power_w += radiator_motor_request_w[index];
            radiator_telemetry.push(RadiatorTelemetry {
                name: radiator.name.clone(),
                deployed_fraction: final_fraction,
                target_fraction: radiator_planned[index],
                rejected_heat_w: acc.radiator_w[index] / command.dt_s,
                deployment_power_w: radiator_motor_request_w[index],
            });
        }

        if next
            .node_temp_k
            .iter()
            .chain(&next.radiator_deployed_fraction)
            .any(|value| !value.is_finite())
        {
            return Err(ThermalError::InvalidState(
                "thermal integration produced non-finite state".into(),
            ));
        }

        let mut node_telemetry = Vec::with_capacity(self.nodes.len());
        let mut hottest_name = String::new();
        let mut hottest_temp_k = f64::NEG_INFINITY;
        for (index, node) in self.nodes.iter().enumerate() {
            let temp_k = next.node_temp_k[index];
            if temp_k > hottest_temp_k {
                hottest_temp_k = temp_k;
                hottest_name.clone_from(&node.name);
            }
            node_telemetry.push(ThermalNodeTelemetry {
                name: node.name.clone(),
                temp_k,
                solar_heat_w: acc.solar_w[index] / command.dt_s,
                aero_heat_w: acc.aero_w[index] / command.dt_s,
                internal_heat_w: acc.internal_w[index] / command.dt_s,
                radiated_heat_w: acc.radiated_w[index] / command.dt_s,
                radiator_rejected_heat_w: acc.radiator_w[index] / command.dt_s,
                conducted_net_heat_w: acc.conducted_w[index] / command.dt_s,
                margin_to_max_k: node.max_temp_k - temp_k,
                overheated: temp_k > node.max_temp_k,
            });
        }
        let telemetry = ThermalTelemetry {
            total_solar_heat_w: acc.total_solar / command.dt_s,
            total_aero_heat_w: acc.total_aero / command.dt_s,
            total_internal_heat_w: acc.total_internal / command.dt_s,
            total_rejected_heat_w: (acc.total_radiated + acc.total_radiator) / command.dt_s,
            stored_energy_change_j: acc.total_stored,
            hottest_node: hottest_name,
            hottest_temp_k,
            substeps,
            radiator_deployment_power_w,
            nodes: node_telemetry,
            radiators: radiator_telemetry,
        };
        Ok((next, telemetry))
    }

    fn stability_substeps(
        &self,
        temps: &[f64],
        links: &[(usize, usize, f64)],
        radiators: &[(usize, &RadiatorSpec, f64)],
        dt_s: f64,
    ) -> Result<usize, ThermalError> {
        let mut link_loss = vec![0.0; self.nodes.len()];
        for (a, b, conductance) in links {
            link_loss[*a] += conductance;
            link_loss[*b] += conductance;
        }
        let mut radiator_loss = vec![0.0; self.nodes.len()];
        for (index, radiator, deployed) in radiators {
            radiator_loss[*index] += radiator.emissivity * radiator.area_m2 * deployed;
        }
        let mut worst_dt = f64::INFINITY;
        for (index, node) in self.nodes.iter().enumerate() {
            let bound_k = temps[index].max(node.max_temp_k).max(1.0);
            let slope = 4.0
                * STEFAN_BOLTZMANN_W_M2_K4
                * (node.emissivity * node.radiating_area_m2 + radiator_loss[index])
                * bound_k.powi(3)
                + link_loss[index];
            if slope <= 0.0 {
                continue;
            }
            worst_dt = worst_dt.min(node.heat_capacity_j_k() / slope);
        }
        if !worst_dt.is_finite() {
            return Ok(1);
        }
        let needed = (dt_s / worst_dt).ceil() as usize;
        let needed = needed.max(1);
        if needed > MAX_THERMAL_SUBSTEPS {
            return Err(ThermalError::InvalidCommand(format!(
                "thermal step needs {needed} stability substeps (cap {MAX_THERMAL_SUBSTEPS}); \
                 call with a shorter dt"
            )));
        }
        Ok(needed)
    }

    fn step_rates(
        &self,
        temps: &[f64],
        links: &[(usize, usize, f64)],
        radiators: &[(usize, &RadiatorSpec, f64)],
        command: &ThermalCommand,
    ) -> StepRates {
        let mut solar = vec![0.0; self.nodes.len()];
        let mut aero = vec![0.0; self.nodes.len()];
        let mut conducted = vec![0.0; self.nodes.len()];
        let mut radiated = vec![0.0; self.nodes.len()];
        let mut radiator_out = vec![0.0; self.nodes.len()];

        for (index, node) in self.nodes.iter().enumerate() {
            if node.solar_exposed_area_m2 > 0.0 {
                let projected: f64 = command
                    .solar_flux
                    .iter()
                    .map(|source| {
                        source.effective_irradiance_w_m2()
                            * node.solar_normal_body.dot(source.direction_body).max(0.0)
                    })
                    .sum();
                solar[index] = node.solar_absorptivity * node.solar_exposed_area_m2 * projected;
            }
            if node.aero_area_m2 > 0.0
                && let Some(flow) = command.flow
            {
                aero[index] = stagnation_heat_w(
                    self.convective_k,
                    flow.density_kg_m3,
                    flow.speed_mps,
                    node.nose_radius_m,
                    node.aero_area_m2,
                );
            }
            let temp4 = temps[index].max(0.0).powi(4);
            radiated[index] = node.emissivity
                * STEFAN_BOLTZMANN_W_M2_K4
                * node.radiating_area_m2
                * (temp4 - SPACE_BACKGROUND_TEMP_K.powi(4));
        }
        for (node_index, radiator, deployed) in radiators {
            let effective_area = radiator.area_m2 * deployed;
            let temp4 = temps[*node_index].max(0.0).powi(4);
            radiator_out[*node_index] += radiator.emissivity
                * STEFAN_BOLTZMANN_W_M2_K4
                * effective_area
                * (temp4 - SPACE_BACKGROUND_TEMP_K.powi(4));
            if radiator.solar_absorptivity > 0.0 {
                let projected: f64 = command
                    .solar_flux
                    .iter()
                    .map(|source| {
                        source.effective_irradiance_w_m2()
                            * radiator.normal_body.dot(source.direction_body).max(0.0)
                    })
                    .sum();
                solar[*node_index] += radiator.solar_absorptivity * effective_area * projected;
            }
        }
        for (a, b, conductance) in links {
            let flow_w = conductance * (temps[*a] - temps[*b]);
            conducted[*a] -= flow_w;
            conducted[*b] += flow_w;
        }
        let mut dtemp_dt = vec![0.0; self.nodes.len()];
        for (index, node) in self.nodes.iter().enumerate() {
            let net =
                solar[index] + aero[index] + command.internal_heat_w[index] + conducted[index]
                    - radiated[index]
                    - radiator_out[index];
            dtemp_dt[index] = net / node.heat_capacity_j_k();
        }
        StepRates {
            dtemp_dt,
            solar,
            aero,
            conducted,
            radiated,
            radiator_out,
        }
    }
}

/// Sutton-Graves stagnation-point heating power (W): engineering correlation
/// driven by local density, speed, and nose geometry — not a tuned constant.
fn stagnation_heat_w(
    convective_k: f64,
    density_kg_m3: f64,
    speed_mps: f64,
    nose_radius_m: f64,
    area_m2: f64,
) -> f64 {
    if density_kg_m3 <= 0.0 || speed_mps <= 0.0 {
        return 0.0;
    }
    convective_k * (density_kg_m3 / nose_radius_m).sqrt() * speed_mps.powi(3) * area_m2
}

/// Persistent authoritative node temperatures for one vessel.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ThermalState {
    pub node_temp_k: Vec<f64>,
    #[serde(default)]
    pub radiator_deployed_fraction: Vec<f64>,
}

impl ThermalState {
    pub fn validate_for(&self, system: &ThermalSystem) -> Result<(), ThermalError> {
        if self.node_temp_k.len() != system.nodes.len()
            || self.radiator_deployed_fraction.len() != system.radiators.len()
        {
            return Err(ThermalError::InvalidState(
                "state array lengths do not match the authored thermal system".into(),
            ));
        }
        for (node, temp) in system.nodes.iter().zip(&self.node_temp_k) {
            if !temp.is_finite() || *temp <= 0.0 {
                return Err(ThermalError::InvalidState(format!(
                    "thermal node '{}' temperature must be finite and positive",
                    node.name
                )));
            }
        }
        for (radiator, deployed) in system
            .radiators
            .iter()
            .zip(&self.radiator_deployed_fraction)
        {
            if !deployed.is_finite() || !(0.0..=1.0).contains(deployed) {
                return Err(ThermalError::InvalidState(format!(
                    "radiator '{}' deployment must be in [0, 1]",
                    radiator.name
                )));
            }
            if radiator.deployment == RadiatorDeployment::Fixed && *deployed != 1.0 {
                return Err(ThermalError::InvalidState(format!(
                    "fixed radiator '{}' must remain fully exposed",
                    radiator.name
                )));
            }
        }
        Ok(())
    }
}

/// Per-step commands use the same order as the authored node/radiator lists.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ThermalCommand {
    pub dt_s: f64,
    pub solar_flux: Vec<SolarFluxSource>,
    /// Local airflow for aero heating; `None` means vacuum.
    pub flow: Option<ThermalFlowCondition>,
    /// Explicit internal loads per node (W): engine/reactor waste heat wired
    /// by the caller. Negative values model active cooling.
    pub internal_heat_w: Vec<f64>,
    /// `None` holds; `Some(0..=1)` targets a foldable radiator.
    pub radiator_deployment_targets: Vec<Option<f64>>,
    /// Granted share of each deployment motor's requested power (0..=1),
    /// booked by the caller on the electrical bus. `idle_for` grants full.
    pub radiator_power_fraction: Vec<f64>,
}

impl ThermalCommand {
    pub fn idle_for(system: &ThermalSystem, dt_s: f64) -> Self {
        Self {
            dt_s,
            solar_flux: Vec::new(),
            flow: None,
            internal_heat_w: vec![0.0; system.nodes.len()],
            radiator_deployment_targets: vec![None; system.radiators.len()],
            radiator_power_fraction: vec![1.0; system.radiators.len()],
        }
    }

    fn validate_for(&self, system: &ThermalSystem) -> Result<(), ThermalError> {
        if !self.dt_s.is_finite() || self.dt_s <= 0.0 {
            return Err(ThermalError::InvalidCommand(
                "thermal step duration must be finite and positive".into(),
            ));
        }
        if self.internal_heat_w.len() != system.nodes.len()
            || self.radiator_deployment_targets.len() != system.radiators.len()
            || self.radiator_power_fraction.len() != system.radiators.len()
        {
            return Err(ThermalError::InvalidCommand(
                "command vector lengths do not match the authored thermal system".into(),
            ));
        }
        for heat in &self.internal_heat_w {
            if !heat.is_finite() {
                return Err(ThermalError::InvalidCommand(
                    "thermal internal load must be finite".into(),
                ));
            }
        }
        for (radiator, target) in system
            .radiators
            .iter()
            .zip(&self.radiator_deployment_targets)
        {
            if let Some(target) = target {
                if !target.is_finite() || !(0.0..=1.0).contains(target) {
                    return Err(ThermalError::InvalidCommand(format!(
                        "radiator '{}' deployment target must be in [0, 1]",
                        radiator.name
                    )));
                }
                if radiator.deployment == RadiatorDeployment::Fixed {
                    return Err(ThermalError::InvalidCommand(format!(
                        "fixed radiator '{}' cannot deploy or stow",
                        radiator.name
                    )));
                }
            }
        }
        for fraction in &self.radiator_power_fraction {
            if !fraction.is_finite() || !(0.0..=1.0).contains(fraction) {
                return Err(ThermalError::InvalidCommand(
                    "radiator power fraction must be in [0, 1]".into(),
                ));
            }
        }
        if let Some(flow) = &self.flow {
            flow.validate()?;
        }
        for source in &self.solar_flux {
            source.validate_for_command().map_err(|error| match error {
                crate::ElectricalPowerError::InvalidCommand(message) => {
                    ThermalError::InvalidCommand(message)
                }
                crate::ElectricalPowerError::InvalidSpec(message) => {
                    ThermalError::InvalidSpec(message)
                }
                crate::ElectricalPowerError::InvalidState(message) => {
                    ThermalError::InvalidState(message)
                }
            })?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalNodeTelemetry {
    pub name: String,
    pub temp_k: f64,
    /// Step-averaged powers (W).
    pub solar_heat_w: f64,
    pub aero_heat_w: f64,
    pub internal_heat_w: f64,
    pub radiated_heat_w: f64,
    pub radiator_rejected_heat_w: f64,
    /// Net conduction inflow (W, negative = net outflow).
    pub conducted_net_heat_w: f64,
    pub margin_to_max_k: f64,
    pub overheated: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalTelemetry {
    pub total_solar_heat_w: f64,
    pub total_aero_heat_w: f64,
    pub total_internal_heat_w: f64,
    pub total_rejected_heat_w: f64,
    /// Net stored-energy change over the step (J, closes the balance).
    pub stored_energy_change_j: f64,
    pub hottest_node: String,
    pub hottest_temp_k: f64,
    pub substeps: usize,
    /// Requested deployment-motor power for the power bus to book (W).
    pub radiator_deployment_power_w: f64,
    pub nodes: Vec<ThermalNodeTelemetry>,
    pub radiators: Vec<RadiatorTelemetry>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RadiatorTelemetry {
    pub name: String,
    pub deployed_fraction: f64,
    pub target_fraction: f64,
    pub rejected_heat_w: f64,
    /// Requested motor power this step (W, before the granted share).
    pub deployment_power_w: f64,
}

struct StepRates {
    dtemp_dt: Vec<f64>,
    solar: Vec<f64>,
    aero: Vec<f64>,
    conducted: Vec<f64>,
    radiated: Vec<f64>,
    radiator_out: Vec<f64>,
}

struct Accumulator {
    solar_w: Vec<f64>,
    aero_w: Vec<f64>,
    internal_w: Vec<f64>,
    conducted_w: Vec<f64>,
    radiated_w: Vec<f64>,
    radiator_w: Vec<f64>,
    total_solar: f64,
    total_aero: f64,
    total_internal: f64,
    total_radiated: f64,
    total_radiator: f64,
    total_stored: f64,
}

impl Accumulator {
    fn new(nodes: usize) -> Self {
        Self {
            solar_w: vec![0.0; nodes],
            aero_w: vec![0.0; nodes],
            internal_w: vec![0.0; nodes],
            conducted_w: vec![0.0; nodes],
            radiated_w: vec![0.0; nodes],
            radiator_w: vec![0.0; nodes],
            total_solar: 0.0,
            total_aero: 0.0,
            total_internal: 0.0,
            total_radiated: 0.0,
            total_radiator: 0.0,
            total_stored: 0.0,
        }
    }

    fn add(&mut self, index: usize, rates: &StepRates, h: f64, internal_w: f64, capacity_j_k: f64) {
        self.solar_w[index] += rates.solar[index] * h;
        self.aero_w[index] += rates.aero[index] * h;
        self.internal_w[index] += internal_w * h;
        self.conducted_w[index] += rates.conducted[index] * h;
        self.radiated_w[index] += rates.radiated[index] * h;
        self.radiator_w[index] += rates.radiator_out[index] * h;
        self.total_solar += rates.solar[index] * h;
        self.total_aero += rates.aero[index] * h;
        self.total_internal += internal_w * h;
        self.total_radiated += rates.radiated[index] * h;
        self.total_radiator += rates.radiator_out[index] * h;
        self.total_stored += rates.dtemp_dt[index] * h * capacity_j_k;
    }
}

fn parallel_axis(mass_kg: f64, center_m: DVec3) -> DMat3 {
    (DMat3::IDENTITY * center_m.length_squared()
        - DMat3::from_cols(
            center_m * center_m.x,
            center_m * center_m.y,
            center_m * center_m.z,
        ))
        * mass_kg
}

fn require_positive(value: f64, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(ThermalError::InvalidSpec(format!(
            "{label} must be finite and positive"
        )));
    }
    Ok(())
}

fn require_non_negative(value: f64, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() || value < 0.0 {
        return Err(ThermalError::InvalidSpec(format!(
            "{label} must be finite and non-negative"
        )));
    }
    Ok(())
}

fn require_emissivity(value: f64, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() || value <= 0.0 || value > 1.0 {
        return Err(ThermalError::InvalidSpec(format!(
            "{label} must be finite and in (0, 1]"
        )));
    }
    Ok(())
}

fn require_absorptivity(value: f64, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(ThermalError::InvalidSpec(format!(
            "{label} must be finite and in [0, 1]"
        )));
    }
    Ok(())
}

fn validate_finite_vector(value: DVec3, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() {
        return Err(ThermalError::InvalidSpec(format!("{label} must be finite")));
    }
    Ok(())
}

fn validate_unit_vector(value: DVec3, label: &str) -> Result<(), ThermalError> {
    if !value.is_finite() || (value.length() - 1.0).abs() > 1.0e-9 {
        return Err(ThermalError::InvalidSpec(format!(
            "{label} must be finite and unit length"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SolarFluxSource;

    fn node(name: &str) -> ThermalNodeSpec {
        ThermalNodeSpec {
            name: name.into(),
            mass_kg: 100.0,
            specific_heat_j_kg_k: 900.0,
            initial_temp_k: 280.0,
            max_temp_k: 500.0,
            emissivity: 0.8,
            solar_absorptivity: 0.5,
            radiating_area_m2: 2.0,
            solar_exposed_area_m2: 1.0,
            solar_normal_body: DVec3::Z,
            aero_area_m2: 0.0,
            nose_radius_m: 1.0,
            position_body_m: DVec3::ZERO,
        }
    }

    fn flux(power_w_m2: f64) -> SolarFluxSource {
        SolarFluxSource::new(power_w_m2, DVec3::Z, 1.0).expect("valid flux")
    }

    #[test]
    fn conduction_converges_to_capacity_weighted_mean() {
        let mut hot = node("hot");
        hot.initial_temp_k = 400.0;
        let mut cold = node("cold");
        cold.initial_temp_k = 200.0;
        cold.mass_kg = 300.0;
        for alone in [&mut hot, &mut cold] {
            alone.radiating_area_m2 = 0.0;
            alone.solar_exposed_area_m2 = 0.0;
        }
        let system = ThermalSystem {
            nodes: vec![hot, cold],
            links: vec![ThermalLinkSpec {
                node_a: "hot".into(),
                node_b: "cold".into(),
                conductance_w_k: 50.0,
            }],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        let state = system.initial_state().unwrap();
        let command = ThermalCommand::idle_for(&system, 12_000.0);
        let (next, _) = system.advance(&state, &command).expect("conduct");
        // Isolated pair equilibrates at (100*900*400 + 300*900*200)/360000 = 250 K.
        for temp in &next.node_temp_k {
            assert!((temp - 250.0).abs() < 0.5, "temp was {temp}");
        }
    }

    #[test]
    fn solar_step_matches_flat_plate_analytic_energy() {
        let mut plate = node("plate");
        plate.radiating_area_m2 = 0.0;
        let system = ThermalSystem {
            nodes: vec![plate],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        let state = system.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&system, 10.0);
        command.solar_flux = vec![flux(1_000.0)];
        let (next, report) = system.advance(&state, &command).expect("solar step");
        // 0.5 * 1.0 m^2 * 1000 W/m^2 * 10 s / 90000 J/K = 0.555... K.
        assert!((next.node_temp_k[0] - (280.0 + 500.0 / 9_000.0)).abs() < 1.0e-9);
        assert_eq!(report.total_solar_heat_w, 500.0);
        assert!((report.stored_energy_change_j - 5_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn radiator_holds_closed_form_equilibrium() {
        let mut block = node("block");
        block.radiating_area_m2 = 0.0;
        block.solar_exposed_area_m2 = 0.0;
        let system = ThermalSystem {
            nodes: vec![block],
            links: vec![],
            radiators: vec![RadiatorSpec {
                name: "wing-rad".into(),
                attached_node: "block".into(),
                area_m2: 5.0,
                emissivity: 0.9,
                solar_absorptivity: 0.0,
                normal_body: DVec3::X,
                areal_density_kg_m2: 5.0,
                position_body_m: DVec3::ZERO,
                deployment: RadiatorDeployment::Fixed,
            }],
            convective_k: default_convective_k(),
        };
        // Equilibrium at P_in = εσA(T^4 − T_bg^4) with P_in = 2000 W.
        let equilibrium_k = (2_000.0 / (0.9 * STEFAN_BOLTZMANN_W_M2_K4 * 5.0)
            + SPACE_BACKGROUND_TEMP_K.powi(4))
        .powf(0.25);
        let state = system.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&system, 20_000.0);
        command.internal_heat_w = vec![2_000.0];
        let (next, _) = system.advance(&state, &command).expect("soak");
        assert!(
            (next.node_temp_k[0] - equilibrium_k).abs() < 1.0,
            "got {}, want {equilibrium_k}",
            next.node_temp_k[0]
        );
        // A short step from the soaked state rejects the input almost exactly.
        let mut settled_command = ThermalCommand::idle_for(&system, 1.0);
        settled_command.internal_heat_w = vec![2_000.0];
        let settled = ThermalState {
            node_temp_k: next.node_temp_k.clone(),
            radiator_deployed_fraction: vec![1.0],
        };
        let (_, report) = system
            .advance(&settled, &settled_command)
            .expect("settle step");
        assert!((report.total_rejected_heat_w - 2_000.0).abs() < 5.0);
        assert!(!report.nodes[0].overheated);
    }

    #[test]
    fn aero_heating_matches_sutton_graves_number() {
        // rho = 0.01 kg/m^3, v = 3000 m/s, Rn = 1 m:
        // q = 1.83e-4 * sqrt(0.01) * 27e9 = 494100 W/m^2.
        assert!((stagnation_heat_w(1.83e-4, 0.01, 3_000.0, 1.0, 1.0) - 494_100.0).abs() < 1.0);
        let mut nose = node("nose");
        nose.aero_area_m2 = 0.5;
        let system = ThermalSystem {
            nodes: vec![nose],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        let state = system.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&system, 1.0);
        command.flow = Some(ThermalFlowCondition {
            density_kg_m3: 0.01,
            speed_mps: 3_000.0,
        });
        let (_, report) = system.advance(&state, &command).expect("aero step");
        assert!((report.total_aero_heat_w - 247_050.0).abs() < 1.0);
        // Vacuum flow means no aero load at all.
        command.flow = Some(ThermalFlowCondition {
            density_kg_m3: 0.0,
            speed_mps: 3_000.0,
        });
        let (_, vacuum) = system.advance(&state, &command).expect("vacuum step");
        assert_eq!(vacuum.total_aero_heat_w, 0.0);
    }

    #[test]
    fn energy_balance_closes_and_overheat_is_reported() {
        let mut a = node("a");
        a.position_body_m = DVec3::X;
        let mut b = node("b");
        b.initial_temp_k = 320.0;
        b.position_body_m = DVec3::NEG_X;
        let system = ThermalSystem {
            nodes: vec![a, b],
            links: vec![ThermalLinkSpec {
                node_a: "a".into(),
                node_b: "b".into(),
                conductance_w_k: 20.0,
            }],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        let mass = system.mass_properties().expect("mass");
        assert!((mass.mass_kg - 200.0).abs() < 1.0e-12);
        assert!(mass.center_of_mass_body_m.length() < 1.0e-12);
        let state = system.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&system, 60.0);
        command.solar_flux = vec![flux(1_360.0)];
        command.internal_heat_w = vec![300.0, 0.0];
        let (next, report) = system.advance(&state, &command).expect("step");
        let stored: f64 = next
            .node_temp_k
            .iter()
            .zip(&state.node_temp_k)
            .map(|(new, old)| (new - old) * 90_000.0)
            .sum();
        let flows =
            (report.total_solar_heat_w + report.total_aero_heat_w + report.total_internal_heat_w
                - report.total_rejected_heat_w)
                * 60.0;
        assert!((stored - flows).abs() < 1.0e-6 * flows.abs().max(1.0));
        assert!((stored - report.stored_energy_change_j).abs() < 1.0e-6);
        // Node b starts at 320 K under full sun: still far from 500 K.
        assert!(!report.nodes[1].overheated);
        assert!(report.nodes[1].margin_to_max_k > 100.0);
    }

    #[test]
    fn foldable_radiator_is_rate_and_bus_power_limited() {
        let mut block = node("block");
        block.radiating_area_m2 = 0.0;
        block.solar_exposed_area_m2 = 0.0;
        let system = ThermalSystem {
            nodes: vec![block],
            links: vec![],
            radiators: vec![RadiatorSpec {
                name: "fold-rad".into(),
                attached_node: "block".into(),
                area_m2: 5.0,
                emissivity: 0.9,
                solar_absorptivity: 0.0,
                normal_body: DVec3::X,
                areal_density_kg_m2: 5.0,
                position_body_m: DVec3::ZERO,
                deployment: RadiatorDeployment::Foldable {
                    deployment_rate_per_s: 0.1,
                    actuator_power_w: 80.0,
                    initial_fraction: 0.0,
                },
            }],
            convective_k: default_convective_k(),
        };
        let state = system.initial_state().unwrap();
        assert_eq!(state.radiator_deployed_fraction, vec![0.0]);
        // Stowed: no rejection even under load.
        let mut command = ThermalCommand::idle_for(&system, 10.0);
        command.internal_heat_w = vec![2_000.0];
        let (stowed, report) = system.advance(&state, &command).expect("stowed step");
        assert_eq!(stowed.radiator_deployed_fraction[0], 0.0);
        assert_eq!(report.total_rejected_heat_w, 0.0);
        assert_eq!(report.radiator_deployment_power_w, 0.0);

        // Deploy request with no granted bus power: holds, requests 80 W.
        command.radiator_deployment_targets = vec![Some(1.0)];
        command.radiator_power_fraction = vec![0.0];
        let (held, report) = system.advance(&state, &command).expect("held step");
        assert_eq!(held.radiator_deployed_fraction[0], 0.0);
        assert_eq!(report.radiator_deployment_power_w, 80.0);
        assert_eq!(report.radiators[0].target_fraction, 1.0);

        // Granted power deploys at the authored rate: 0.1/s over 10 s.
        command.radiator_power_fraction = vec![1.0];
        let (next, _) = system.advance(&state, &command).expect("deploy step");
        assert!((next.radiator_deployed_fraction[0] - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn bad_specs_commands_and_states_fail_closed() {
        let mut broken = node("x");
        broken.mass_kg = -1.0;
        let system = ThermalSystem {
            nodes: vec![broken],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        assert!(system.validate().is_err());

        let single = ThermalSystem {
            nodes: vec![node("ok")],
            links: vec![ThermalLinkSpec {
                node_a: "ok".into(),
                node_b: "ghost".into(),
                conductance_w_k: 1.0,
            }],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        assert!(single.validate().is_err());

        let good = ThermalSystem {
            nodes: vec![node("ok")],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
        };
        let state = good.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&good, 1.0);
        command.internal_heat_w = vec![1.0, 2.0];
        assert!(good.advance(&state, &command).is_err());
        assert_eq!(state.node_temp_k[0], 280.0);
        command.internal_heat_w = vec![0.0];
        command.dt_s = -1.0;
        assert!(good.advance(&state, &command).is_err());
    }
}
