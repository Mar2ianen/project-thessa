//! Lumped thermal-node network: conduction, radiation, and radiators.
//!
//! Each node is one isothermal lump with heat capacity `mass * cp`. Nodes
//! exchange heat through conduction links, radiate to the cold background,
//! absorb sunlight through the same occluded [`SolarFluxSource`] inputs as
//! the power bus, pick up stagnation aero heating, and accept explicit
//! internal loads (engine/reactor waste heat wired by the caller). Radiators
//! are area devices tied to a node. There is no ablation, no heat shields,
//! and no automatic damage: overheating is reported, never auto-exploded.

use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

use crate::{ElectricalPowerTelemetry, SolarFluxSource};

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
    /// Pointing gimbal for the panel normal. `Fixed` (default) keeps the
    /// authored normal forever; `SingleAxis` steers it about a body-frame
    /// axis to turn the panel edge-on to the sun.
    #[serde(default)]
    pub tracking: RadiatorTracking,
}

/// Pointing drive for a radiator panel. Unlike solar arrays the automatic
/// mode *minimizes* incident sunlight (edge-on rejection) instead of
/// maximizing it; a manual angle target overrides it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum RadiatorTracking {
    #[default]
    Fixed,
    SingleAxis {
        /// Unit rotation axis in body coordinates.
        rotation_axis_body: DVec3,
        minimum_angle_rad: f64,
        maximum_angle_rad: f64,
        slew_rate_rad_s: f64,
        actuator_power_w: f64,
        initial_angle_rad: f64,
    },
}

impl RadiatorTracking {
    fn validate(&self, radiator_name: &str) -> Result<(), ThermalError> {
        let RadiatorTracking::SingleAxis {
            rotation_axis_body,
            minimum_angle_rad,
            maximum_angle_rad,
            slew_rate_rad_s,
            actuator_power_w,
            initial_angle_rad,
        } = self
        else {
            return Ok(());
        };
        validate_unit_vector(*rotation_axis_body, "radiator tracking axis")?;
        for (value, label) in [
            (*minimum_angle_rad, "radiator tracking minimum angle"),
            (*maximum_angle_rad, "radiator tracking maximum angle"),
            (*initial_angle_rad, "initial radiator tracking angle"),
        ] {
            if !value.is_finite() {
                return Err(ThermalError::InvalidSpec(format!(
                    "radiator '{radiator_name}' {label} must be finite"
                )));
            }
        }
        if minimum_angle_rad > maximum_angle_rad {
            return Err(ThermalError::InvalidSpec(format!(
                "radiator '{radiator_name}' tracking limits must satisfy min <= max"
            )));
        }
        if !(*minimum_angle_rad..=*maximum_angle_rad).contains(initial_angle_rad) {
            return Err(ThermalError::InvalidSpec(format!(
                "radiator '{radiator_name}' initial tracking angle must lie within its limits"
            )));
        }
        require_positive(*slew_rate_rad_s, "radiator tracking slew rate")?;
        require_positive(*actuator_power_w, "radiator tracking actuator power")?;
        Ok(())
    }

    fn initial_angle_rad(&self) -> f64 {
        match self {
            RadiatorTracking::Fixed => 0.0,
            RadiatorTracking::SingleAxis {
                initial_angle_rad, ..
            } => *initial_angle_rad,
        }
    }
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
        self.tracking.validate(&self.name)?;
        // A gimbal axis parallel to the panel normal cannot change
        // incidence; reject the degenerate authoring.
        if let RadiatorTracking::SingleAxis {
            rotation_axis_body, ..
        } = self.tracking
            && rotation_axis_body.dot(self.normal_body).abs() > 1.0 - 1.0e-6
        {
            return Err(ThermalError::InvalidSpec(format!(
                "radiator '{}' tracking axis must not be parallel to the panel normal",
                self.name
            )));
        }
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

    /// Panel normal at the given gimbal angle. Fixed panels ignore it.
    pub fn normal_at_angle(&self, angle_rad: f64) -> DVec3 {
        let RadiatorTracking::SingleAxis {
            rotation_axis_body, ..
        } = self.tracking
        else {
            return self.normal_body;
        };
        DQuat::from_axis_angle(rotation_axis_body, angle_rad) * self.normal_body
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
    /// Authored waste-heat routing from the shared power bus (and shaft
    /// generators, and wheel banks) into nodes. Replaces hand-rolled caller
    /// mapping: each entry sends a fraction of one named reactor/fuel-cell
    /// source's waste heat — or one named APU/jet generator mount's or
    /// wheel bank's losses — to one node.
    #[serde(default)]
    pub heat_sources: Vec<ThermalHeatSource>,
}

/// One waste-heat route: a share of a named power source's rejected heat
/// lands on a named thermal node as an internal load.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalHeatSource {
    /// Reactor or fuel-cell name in the vessel power system (bus-wide unique).
    /// For `Generator` routes, an APU or jet mount name carrying shaft
    /// generator telemetry instead.
    pub source_name: String,
    /// Thermal node receiving the heat.
    pub node: String,
    /// Share of that source's waste heat in [0, 1].
    pub fraction: f64,
    /// Which inventory the source name resolves against. Defaults to the
    /// power bus so older assets keep their meaning.
    #[serde(default)]
    pub kind: ThermalHeatSourceKind,
}

/// Waste-heat inventory a [`ThermalHeatSource`] route draws from.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ThermalHeatSourceKind {
    /// Reactor or fuel-cell waste heat from `ElectricalPowerTelemetry`.
    #[default]
    Power,
    /// Shaft-generator losses from an APU or jet mount, supplied by the
    /// caller as [`NamedWasteHeat`] (generator shaft draw minus electrical
    /// output — a measured loss, not a coefficient).
    Generator,
    /// Reaction-wheel motor losses by bank name, supplied by the caller as
    /// [`NamedWasteHeat`] (bus draw minus rotor kinetic-energy rate via
    /// `reaction_wheel_step_heat_w`).
    ReactionWheel,
}

/// Named generator loss snapshot for one APU or jet mount (W). Build with
/// [`generator_waste_heat_w`] from shaft telemetry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NamedWasteHeat {
    pub name: String,
    pub waste_heat_w: f64,
}

/// Generator loss from shaft telemetry: shaft power drawn to produce the
/// electrical output, minus that output. Clamped at zero; non-finite or
/// negative inputs fail closed.
pub fn generator_waste_heat_w(
    generator_shaft_draw_w: f64,
    generator_electrical_w: f64,
) -> Result<f64, ThermalError> {
    if !generator_shaft_draw_w.is_finite() || !generator_electrical_w.is_finite() {
        return Err(ThermalError::InvalidCommand(
            "generator shaft draw and electrical output must be finite".into(),
        ));
    }
    if generator_shaft_draw_w < 0.0 || generator_electrical_w < 0.0 {
        return Err(ThermalError::InvalidCommand(
            "generator shaft draw and electrical output must be non-negative".into(),
        ));
    }
    Ok((generator_shaft_draw_w - generator_electrical_w).max(0.0))
}

impl ThermalHeatSource {
    fn validate(&self, system_nodes: &[ThermalNodeSpec]) -> Result<(), ThermalError> {
        if self.source_name.trim().is_empty() {
            return Err(ThermalError::InvalidSpec(
                "thermal heat source name must not be empty".into(),
            ));
        }
        if !system_nodes.iter().any(|node| node.name == self.node) {
            return Err(ThermalError::InvalidSpec(format!(
                "thermal heat source '{}' targets unknown node '{}'",
                self.source_name, self.node
            )));
        }
        if !self.fraction.is_finite() || !(0.0..=1.0).contains(&self.fraction) {
            return Err(ThermalError::InvalidSpec(format!(
                "thermal heat source '{}' fraction must be in [0, 1]",
                self.source_name
            )));
        }
        Ok(())
    }
}

impl Default for ThermalSystem {
    fn default() -> Self {
        Self {
            nodes: Vec::new(),
            links: Vec::new(),
            radiators: Vec::new(),
            convective_k: default_convective_k(),
            heat_sources: Vec::new(),
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
        let mut source_share = std::collections::HashMap::new();
        for source in &self.heat_sources {
            source.validate(&self.nodes)?;
            let total = source_share
                .entry((source.kind, source.source_name.as_str()))
                .or_insert(0.0);
            *total += source.fraction;
            if *total > 1.0 + 1.0e-12 {
                return Err(ThermalError::InvalidSpec(format!(
                    "thermal heat routing for '{}' exceeds its waste heat",
                    source.source_name
                )));
            }
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
            radiator_tracking_angle_rad: self
                .radiators
                .iter()
                .map(|radiator| radiator.tracking.initial_angle_rad())
                .collect(),
        })
    }

    /// Per-node internal loads (W, node order) routed from one power-bus
    /// step's waste-heat telemetry through the authored [`ThermalHeatSource`]
    /// table. Reactor and fuel-cell waste heat are matched by their bus-wide
    /// unique source name; an unmapped source name fails closed so a renamed
    /// power part cannot silently stop heating its node. Generator-kind
    /// routes resolve against an empty inventory here — use
    /// [`Self::waste_heat_loads_w_with_generators`] when mounts report.
    pub fn waste_heat_loads_w(
        &self,
        power: &ElectricalPowerTelemetry,
    ) -> Result<Vec<f64>, ThermalError> {
        self.waste_heat_loads_w_with_generators(power, &[])
    }

    /// Same as [`Self::waste_heat_loads_w`], plus generator-kind routes
    /// resolved against caller-supplied per-mount losses (APU/jet shaft
    /// generators via [`generator_waste_heat_w`]).
    pub fn waste_heat_loads_w_with_generators(
        &self,
        power: &ElectricalPowerTelemetry,
        generators: &[NamedWasteHeat],
    ) -> Result<Vec<f64>, ThermalError> {
        self.validate()?;
        let mut loads = vec![0.0; self.nodes.len()];
        for route in &self.heat_sources {
            let waste_w = match route.kind {
                ThermalHeatSourceKind::Power => power
                    .reactors
                    .iter()
                    .find(|source| source.name == route.source_name)
                    .map(|source| source.waste_heat_w)
                    .or_else(|| {
                        power
                            .fuel_cells
                            .iter()
                            .find(|source| source.name == route.source_name)
                            .map(|source| source.waste_heat_w)
                    })
                    .ok_or_else(|| {
                        ThermalError::InvalidCommand(format!(
                            "thermal heat source '{}' matches no reactor or fuel cell",
                            route.source_name
                        ))
                    })?,
                ThermalHeatSourceKind::Generator | ThermalHeatSourceKind::ReactionWheel => {
                    let inventory = match route.kind {
                        ThermalHeatSourceKind::Generator => "reported generator mount",
                        ThermalHeatSourceKind::ReactionWheel => "reported wheel bank",
                        ThermalHeatSourceKind::Power => unreachable!(
                            "power-kind routes resolve against the bus inventory above"
                        ),
                    };
                    generators
                        .iter()
                        .find(|source| source.name == route.source_name)
                        .map(|source| source.waste_heat_w)
                        .ok_or_else(|| {
                            ThermalError::InvalidCommand(format!(
                                "thermal heat source '{}' matches no {inventory}",
                                route.source_name
                            ))
                        })?
                }
            };
            if !waste_w.is_finite() || waste_w < 0.0 {
                return Err(ThermalError::InvalidCommand(format!(
                    "thermal heat source '{}' reports non-physical waste heat",
                    route.source_name
                )));
            }
            let index = self
                .node_index(&route.node)
                .expect("validated heat-source node");
            loads[index] += waste_w * route.fraction;
        }
        Ok(loads)
    }

    /// Add the routed waste heat into a thermal command's per-node internal
    /// loads, keeping any caller-wired loads already present. This is the
    /// single call site that replaces hand-rolled reactor/fuel-cell mapping.
    pub fn apply_waste_heat(
        &self,
        command: &mut ThermalCommand,
        power: &ElectricalPowerTelemetry,
    ) -> Result<(), ThermalError> {
        self.apply_waste_heat_with_generators(command, power, &[])
    }

    /// Same as [`Self::apply_waste_heat`], plus generator-kind routes
    /// resolved against caller-supplied per-mount losses.
    pub fn apply_waste_heat_with_generators(
        &self,
        command: &mut ThermalCommand,
        power: &ElectricalPowerTelemetry,
        generators: &[NamedWasteHeat],
    ) -> Result<(), ThermalError> {
        let routed = self.waste_heat_loads_w_with_generators(power, generators)?;
        if command.internal_heat_w.len() != self.nodes.len() {
            return Err(ThermalError::InvalidCommand(
                "command vector lengths do not match the authored thermal system".into(),
            ));
        }
        for (slot, routed_w) in command.internal_heat_w.iter_mut().zip(&routed) {
            *slot += routed_w;
        }
        Ok(())
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

        // Gimbal planning: manual target wins, otherwise the edge-on
        // minimum when enabled, otherwise hold. The gimbal draw joins the
        // same per-radiator motor request the caller books on the bus.
        let mut radiator_tracking_targets_rad = state.radiator_tracking_angle_rad.clone();
        for (index, radiator) in self.radiators.iter().enumerate() {
            let RadiatorTracking::SingleAxis {
                minimum_angle_rad,
                maximum_angle_rad,
                slew_rate_rad_s,
                actuator_power_w,
                ..
            } = radiator.tracking
            else {
                continue;
            };
            let current = state.radiator_tracking_angle_rad[index];
            let desired = if let Some(manual) = command.radiator_tracking_targets[index] {
                manual.clamp(minimum_angle_rad, maximum_angle_rad)
            } else if command.radiator_tracking_auto[index] {
                best_radiator_angle(radiator, current, &command.solar_flux)
            } else {
                current
            };
            let maximum_delta = slew_rate_rad_s * command.dt_s;
            let planned_delta = (desired - current).clamp(-maximum_delta, maximum_delta);
            let duty = if maximum_delta > 0.0 {
                (planned_delta.abs() / maximum_delta).clamp(0.0, 1.0)
            } else {
                0.0
            };
            radiator_tracking_targets_rad[index] = current + planned_delta;
            radiator_motor_request_w[index] += actuator_power_w * duty;
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
        // The gimbal angle likewise applies from the next step.
        let radiator_edges: Vec<(usize, &RadiatorSpec, f64, f64)> = self
            .radiators
            .iter()
            .zip(&state.radiator_deployed_fraction)
            .zip(&state.radiator_tracking_angle_rad)
            .map(|((radiator, deployed), angle)| {
                (
                    self.node_index(&radiator.attached_node)
                        .expect("validated node"),
                    radiator,
                    *deployed,
                    *angle,
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
        let mut acc = Accumulator::new(self.nodes.len(), self.radiators.len());
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
            acc.add_radiators(&rates, h);
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
            let current_angle = state.radiator_tracking_angle_rad[index];
            let final_angle = match radiator.tracking {
                RadiatorTracking::Fixed => 0.0,
                RadiatorTracking::SingleAxis {
                    slew_rate_rad_s, ..
                } => {
                    let target = radiator_tracking_targets_rad[index];
                    let delta = target - current_angle;
                    let allowed = slew_rate_rad_s * command.dt_s * granted;
                    current_angle + delta.signum() * delta.abs().min(allowed)
                }
            };
            next.radiator_tracking_angle_rad[index] = final_angle;
            radiator_deployment_power_w += radiator_motor_request_w[index];
            radiator_telemetry.push(RadiatorTelemetry {
                name: radiator.name.clone(),
                deployed_fraction: final_fraction,
                target_fraction: radiator_planned[index],
                rejected_heat_w: acc.radiator_w[index] / command.dt_s,
                deployment_power_w: radiator_motor_request_w[index],
                tracking_angle_rad: final_angle,
                tracking_target_rad: radiator_tracking_targets_rad[index],
            });
        }

        if next
            .node_temp_k
            .iter()
            .chain(&next.radiator_deployed_fraction)
            .chain(&next.radiator_tracking_angle_rad)
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
                radiator_rejected_heat_w: acc.radiator_node_w[index] / command.dt_s,
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
        radiators: &[(usize, &RadiatorSpec, f64, f64)],
        dt_s: f64,
    ) -> Result<usize, ThermalError> {
        let mut link_loss = vec![0.0; self.nodes.len()];
        for (a, b, conductance) in links {
            link_loss[*a] += conductance;
            link_loss[*b] += conductance;
        }
        let mut radiator_loss = vec![0.0; self.nodes.len()];
        for (index, radiator, deployed, _) in radiators {
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
        radiators: &[(usize, &RadiatorSpec, f64, f64)],
        command: &ThermalCommand,
    ) -> StepRates {
        let mut solar = vec![0.0; self.nodes.len()];
        let mut aero = vec![0.0; self.nodes.len()];
        let mut conducted = vec![0.0; self.nodes.len()];
        let mut radiated = vec![0.0; self.nodes.len()];
        let mut radiator_out = vec![0.0; self.nodes.len()];
        let mut radiator_by_device = vec![0.0; radiators.len()];

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
        for (radiator_index, (node_index, radiator, deployed, angle)) in
            radiators.iter().enumerate()
        {
            let effective_area = radiator.area_m2 * deployed;
            let temp4 = temps[*node_index].max(0.0).powi(4);
            let rejected_heat_w = radiator.emissivity
                * STEFAN_BOLTZMANN_W_M2_K4
                * effective_area
                * (temp4 - SPACE_BACKGROUND_TEMP_K.powi(4));
            radiator_out[*node_index] += rejected_heat_w;
            radiator_by_device[radiator_index] = rejected_heat_w;
            if radiator.solar_absorptivity > 0.0 {
                let normal = radiator.normal_at_angle(*angle);
                let projected: f64 = command
                    .solar_flux
                    .iter()
                    .map(|source| {
                        source.effective_irradiance_w_m2()
                            * normal.dot(source.direction_body).max(0.0)
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
            radiator_by_device,
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
    /// Gimbal angle per radiator. Zero for fixed panels.
    #[serde(default)]
    pub radiator_tracking_angle_rad: Vec<f64>,
}

impl ThermalState {
    pub fn validate_for(&self, system: &ThermalSystem) -> Result<(), ThermalError> {
        if self.node_temp_k.len() != system.nodes.len()
            || self.radiator_deployed_fraction.len() != system.radiators.len()
            || self.radiator_tracking_angle_rad.len() != system.radiators.len()
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
        for (radiator, angle) in system
            .radiators
            .iter()
            .zip(&self.radiator_tracking_angle_rad)
        {
            if !angle.is_finite() {
                return Err(ThermalError::InvalidState(format!(
                    "radiator '{}' tracking angle must be finite",
                    radiator.name
                )));
            }
            if let RadiatorTracking::SingleAxis {
                minimum_angle_rad,
                maximum_angle_rad,
                ..
            } = radiator.tracking
                && !(minimum_angle_rad..=maximum_angle_rad).contains(angle)
            {
                return Err(ThermalError::InvalidState(format!(
                    "radiator '{}' tracking angle is outside its limits",
                    radiator.name
                )));
            }
            if radiator.tracking == RadiatorTracking::Fixed && *angle != 0.0 {
                return Err(ThermalError::InvalidState(format!(
                    "fixed radiator '{}' tracking angle must be zero",
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
    /// `None` holds (or auto-tracks); `Some(angle)` drives a gimballed
    /// radiator toward that body-frame angle in radians.
    pub radiator_tracking_targets: Vec<Option<f64>>,
    /// `true` steers a gimballed radiator edge-on to the sun.
    pub radiator_tracking_auto: Vec<bool>,
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
            radiator_tracking_targets: vec![None; system.radiators.len()],
            radiator_tracking_auto: vec![false; system.radiators.len()],
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
            || self.radiator_tracking_targets.len() != system.radiators.len()
            || self.radiator_tracking_auto.len() != system.radiators.len()
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
        for ((radiator, target), auto) in system
            .radiators
            .iter()
            .zip(&self.radiator_tracking_targets)
            .zip(&self.radiator_tracking_auto)
        {
            if let Some(target) = target {
                if !target.is_finite() {
                    return Err(ThermalError::InvalidCommand(format!(
                        "radiator '{}' tracking target must be finite",
                        radiator.name
                    )));
                }
                if radiator.tracking == RadiatorTracking::Fixed {
                    return Err(ThermalError::InvalidCommand(format!(
                        "fixed radiator '{}' cannot slew",
                        radiator.name
                    )));
                }
            }
            if *auto && radiator.tracking == RadiatorTracking::Fixed {
                return Err(ThermalError::InvalidCommand(format!(
                    "fixed radiator '{}' cannot auto-track",
                    radiator.name
                )));
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
    /// Gimbal angle/target (rad); zero without a tracking drive.
    #[serde(default)]
    pub tracking_angle_rad: f64,
    #[serde(default)]
    pub tracking_target_rad: f64,
}

/// Best gimbal angle in `[min, max]` for the current stellar geometry.
/// Mirrors the solar search but *minimizes* incident sunlight (edge-on
/// rejection); holds the current angle in darkness or when nothing
/// improves on it, so the drive never burns power for zero gain.
fn best_radiator_angle(
    radiator: &RadiatorSpec,
    current_angle_rad: f64,
    sources: &[SolarFluxSource],
) -> f64 {
    let RadiatorTracking::SingleAxis {
        minimum_angle_rad,
        maximum_angle_rad,
        ..
    } = radiator.tracking
    else {
        return 0.0;
    };
    let load_at = |angle: f64| {
        let normal = radiator.normal_at_angle(angle);
        sources
            .iter()
            .map(|source| {
                source.effective_irradiance_w_m2() * normal.dot(source.direction_body).max(0.0)
            })
            .sum::<f64>()
    };
    let total: f64 = sources
        .iter()
        .map(SolarFluxSource::effective_irradiance_w_m2)
        .sum();
    if !total.is_finite() || total <= 0.0 {
        return current_angle_rad;
    }
    let current_value = load_at(current_angle_rad);
    const SAMPLES: usize = 72;
    let mut best_angle = current_angle_rad;
    let mut best_value = current_value;
    for sample in 0..=SAMPLES {
        let angle = minimum_angle_rad
            + (maximum_angle_rad - minimum_angle_rad) * sample as f64 / SAMPLES as f64;
        let value = load_at(angle);
        if value < best_value {
            best_value = value;
            best_angle = angle;
        }
    }
    let mut step = (maximum_angle_rad - minimum_angle_rad) / SAMPLES as f64;
    for _ in 0..6 {
        step *= 0.5;
        let mut improved = false;
        for candidate in [best_angle - step, best_angle + step] {
            let candidate = candidate.clamp(minimum_angle_rad, maximum_angle_rad);
            let value = load_at(candidate);
            if value < best_value {
                best_value = value;
                best_angle = candidate;
                improved = true;
            }
        }
        if !improved && step < 1.0e-9 {
            break;
        }
    }
    if best_value >= current_value - 1.0e-9 * (1.0 + total) {
        current_angle_rad
    } else {
        best_angle
    }
}

struct StepRates {
    dtemp_dt: Vec<f64>,
    solar: Vec<f64>,
    aero: Vec<f64>,
    conducted: Vec<f64>,
    radiated: Vec<f64>,
    radiator_out: Vec<f64>,
    radiator_by_device: Vec<f64>,
}

struct Accumulator {
    solar_w: Vec<f64>,
    aero_w: Vec<f64>,
    internal_w: Vec<f64>,
    conducted_w: Vec<f64>,
    radiated_w: Vec<f64>,
    radiator_node_w: Vec<f64>,
    radiator_w: Vec<f64>,
    total_solar: f64,
    total_aero: f64,
    total_internal: f64,
    total_radiated: f64,
    total_radiator: f64,
    total_stored: f64,
}

impl Accumulator {
    fn new(nodes: usize, radiators: usize) -> Self {
        Self {
            solar_w: vec![0.0; nodes],
            aero_w: vec![0.0; nodes],
            internal_w: vec![0.0; nodes],
            conducted_w: vec![0.0; nodes],
            radiated_w: vec![0.0; nodes],
            radiator_node_w: vec![0.0; nodes],
            radiator_w: vec![0.0; radiators],
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
        self.radiator_node_w[index] += rates.radiator_out[index] * h;
        self.total_solar += rates.solar[index] * h;
        self.total_aero += rates.aero[index] * h;
        self.total_internal += internal_w * h;
        self.total_radiated += rates.radiated[index] * h;
        self.total_radiator += rates.radiator_out[index] * h;
        self.total_stored += rates.dtemp_dt[index] * h * capacity_j_k;
    }

    fn add_radiators(&mut self, rates: &StepRates, h: f64) {
        for (accumulated, rejected_heat_w) in
            self.radiator_w.iter_mut().zip(&rates.radiator_by_device)
        {
            *accumulated += rejected_heat_w * h;
        }
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
    use crate::{
        ElectricalPowerCommand, ElectricalPowerSystem, PowerConsumerSpec, PowerPriority,
        ReactorSpec, SolarFluxSource,
    };
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
            heat_sources: vec![],
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
            heat_sources: vec![],
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
                tracking: RadiatorTracking::Fixed,
            }],
            convective_k: default_convective_k(),
            heat_sources: vec![],
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
            radiator_tracking_angle_rad: vec![0.0],
        };
        let (_, report) = system
            .advance(&settled, &settled_command)
            .expect("settle step");
        assert!((report.total_rejected_heat_w - 2_000.0).abs() < 5.0);
        assert!(!report.nodes[0].overheated);
    }

    #[test]
    fn multiple_radiators_report_per_device_heat_on_their_attached_node() {
        let mut first = node("first");
        first.radiating_area_m2 = 0.0;
        first.solar_exposed_area_m2 = 0.0;
        let mut second = node("second");
        second.radiating_area_m2 = 0.0;
        second.solar_exposed_area_m2 = 0.0;
        let radiators = [
            ("small", 1.0, 0.4),
            ("medium", 2.0, 0.7),
            ("large", 3.0, 0.9),
        ]
        .into_iter()
        .map(|(name, area_m2, emissivity)| RadiatorSpec {
            name: name.into(),
            attached_node: "second".into(),
            area_m2,
            emissivity,
            solar_absorptivity: 0.0,
            normal_body: DVec3::X,
            areal_density_kg_m2: 1.0,
            position_body_m: DVec3::ZERO,
            deployment: RadiatorDeployment::Fixed,
            tracking: RadiatorTracking::Fixed,
        })
        .collect();
        let system = ThermalSystem {
            nodes: vec![first, second],
            links: vec![],
            radiators,
            convective_k: default_convective_k(),
            heat_sources: vec![],
        };
        let state = system.initial_state().expect("initial state");
        let command = ThermalCommand::idle_for(&system, 1.0);
        let (_, report) = system.advance(&state, &command).expect("radiator step");

        let base_flux =
            STEFAN_BOLTZMANN_W_M2_K4 * (280.0_f64.powi(4) - SPACE_BACKGROUND_TEMP_K.powi(4));
        for (telemetry, (name, area_m2, emissivity)) in report.radiators.iter().zip([
            ("small", 1.0, 0.4),
            ("medium", 2.0, 0.7),
            ("large", 3.0, 0.9),
        ]) {
            assert_eq!(telemetry.name, name);
            let expected = base_flux * area_m2 * emissivity;
            assert!((telemetry.rejected_heat_w - expected).abs() < 1.0e-9 * expected);
        }
        let radiator_total: f64 = report
            .radiators
            .iter()
            .map(|radiator| radiator.rejected_heat_w)
            .sum();
        assert!((report.nodes[1].radiator_rejected_heat_w - radiator_total).abs() < 1.0e-9);
        assert_eq!(report.nodes[0].radiator_rejected_heat_w, 0.0);
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
            heat_sources: vec![],
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
            heat_sources: vec![],
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
                tracking: RadiatorTracking::Fixed,
            }],
            convective_k: default_convective_k(),
            heat_sources: vec![],
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

    fn gimballed_radiator() -> RadiatorSpec {
        RadiatorSpec {
            name: "gimbal-rad".into(),
            attached_node: "block".into(),
            area_m2: 5.0,
            emissivity: 0.05,
            solar_absorptivity: 1.0,
            normal_body: DVec3::Z,
            areal_density_kg_m2: 5.0,
            position_body_m: DVec3::ZERO,
            deployment: RadiatorDeployment::Fixed,
            tracking: RadiatorTracking::SingleAxis {
                rotation_axis_body: DVec3::X,
                minimum_angle_rad: -std::f64::consts::FRAC_PI_2,
                maximum_angle_rad: std::f64::consts::FRAC_PI_2,
                slew_rate_rad_s: 0.5,
                actuator_power_w: 120.0,
                initial_angle_rad: 0.0,
            },
        }
    }

    fn gimbal_system() -> ThermalSystem {
        let mut block = node("block");
        block.radiating_area_m2 = 0.0;
        block.solar_exposed_area_m2 = 0.0;
        ThermalSystem {
            nodes: vec![block],
            links: vec![],
            radiators: vec![gimballed_radiator()],
            convective_k: default_convective_k(),
            heat_sources: vec![],
        }
    }

    #[test]
    fn gimbal_auto_steers_edge_on_and_books_motor_power() {
        let system = gimbal_system();
        let state = system.initial_state().unwrap();
        assert_eq!(state.radiator_tracking_angle_rad, vec![0.0]);
        let mut command = ThermalCommand::idle_for(&system, 2.0);
        command.solar_flux = vec![flux(1_000.0)];
        command.radiator_tracking_auto = vec![true];
        let (next, report) = system.advance(&state, &command).expect("track step");
        // Edge-on optimum at ±90° (either side sheds the sun); slew
        // 0.5 rad/s over 2 s moves 1.0 rad toward it.
        assert!(
            next.radiator_tracking_angle_rad[0].abs() - 1.0 < 1.0e-12,
            "angle: {:?}",
            next.radiator_tracking_angle_rad[0]
        );
        assert_eq!(
            report.radiators[0].tracking_target_rad,
            next.radiator_tracking_angle_rad[0]
        );
        assert_eq!(report.radiator_deployment_power_w, 120.0);
        // First step still prices face-on sunlight (explicit Euler);
        // the tipped panel pays off from the second step on.
        // Explicit Euler prices sunlight at the beginning-of-step angle:
        // the second step sees the tipped panel and absorbs less.
        let (second, second_report) = system.advance(&next, &command).expect("second step");
        assert!(second_report.total_solar_heat_w < 5_000.0);
        assert!(second.radiator_tracking_angle_rad[0].abs() > 1.0);
        // Manual target wins over automatic tracking.
        let mut manual = ThermalCommand::idle_for(&system, 2.0);
        manual.solar_flux = vec![flux(1_000.0)];
        manual.radiator_tracking_targets = vec![Some(-0.5)];
        let (posed, _) = system.advance(&state, &manual).expect("manual slew");
        assert!((posed.radiator_tracking_angle_rad[0] + 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn gimbal_without_power_holds_but_reports_request() {
        let system = gimbal_system();
        let state = system.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&system, 2.0);
        command.solar_flux = vec![flux(1_000.0)];
        command.radiator_tracking_auto = vec![true];
        command.radiator_power_fraction = vec![0.0];
        let (held, report) = system.advance(&state, &command).expect("held step");
        assert_eq!(held.radiator_tracking_angle_rad[0], 0.0);
        assert_eq!(report.radiator_deployment_power_w, 120.0);
    }

    #[test]
    fn degenerate_gimbal_authoring_fails_closed() {
        let mut parallel = gimballed_radiator();
        if let RadiatorTracking::SingleAxis {
            ref mut rotation_axis_body,
            ..
        } = parallel.tracking
        {
            *rotation_axis_body = DVec3::Z;
        }
        assert!(parallel.validate().is_err());
        let system = ThermalSystem {
            nodes: vec![node("block")],
            links: vec![],
            radiators: vec![gimballed_radiator()],
            convective_k: default_convective_k(),
            heat_sources: vec![],
        };
        let state = system.initial_state().unwrap();
        // Fixed panels reject tracking targets; gimballed panels accept them
        // but a fixed panel behind the same command fails instead.
        let mut fixed_system = system.clone();
        fixed_system.radiators[0].tracking = RadiatorTracking::Fixed;
        let mut command = ThermalCommand::idle_for(&fixed_system, 1.0);
        command.radiator_tracking_targets = vec![Some(0.1)];
        assert!(fixed_system.advance(&state, &command).is_err());
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
            heat_sources: vec![],
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
            heat_sources: vec![],
        };
        assert!(single.validate().is_err());

        let good = ThermalSystem {
            nodes: vec![node("ok")],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
            heat_sources: vec![],
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

    fn routed_system() -> ThermalSystem {
        ThermalSystem {
            nodes: vec![node("block"), node("shell")],
            links: vec![],
            radiators: vec![],
            convective_k: default_convective_k(),
            heat_sources: vec![ThermalHeatSource {
                source_name: "rx".into(),
                node: "block".into(),
                fraction: 0.5,
                kind: ThermalHeatSourceKind::Power,
            }],
        }
    }

    fn reactor_bus() -> ElectricalPowerSystem {
        ElectricalPowerSystem {
            reactors: vec![ReactorSpec {
                name: "rx".into(),
                rated_thermal_power_w: 1_000.0,
                electric_efficiency: 0.5,
                radiator_capacity_w: 10_000.0,
                initial_fuel_mass_kg: 1.0,
                fuel_specific_energy_j_kg: 1.0e9,
                dry_mass_kg: 10.0,
                dimensions_body_m: DVec3::splat(1.0),
                position_body_m: DVec3::ZERO,
            }],
            consumers: vec![PowerConsumerSpec {
                name: "life-support".into(),
                rated_power_w: 1_000.0,
                priority: PowerPriority::LifeSupport,
            }],
            ..ElectricalPowerSystem::default()
        }
    }

    #[test]
    fn authored_waste_heat_routes_reactor_losses_into_nodes() {
        let bus = reactor_bus();
        let power_state = bus.initial_state().unwrap();
        let mut power_command = ElectricalPowerCommand::idle_for(&bus, 1.0);
        power_command.consumer_power_w = vec![400.0];
        let (_, power_report) = bus.advance(&power_state, &power_command).expect("bus step");
        // 400 W electric at 50% efficiency rejects 400 W of waste heat.
        assert_eq!(power_report.reactors[0].waste_heat_w, 400.0);

        let thermal = routed_system();
        let loads = thermal.waste_heat_loads_w(&power_report).unwrap();
        assert_eq!(loads, vec![200.0, 0.0]);

        // apply_waste_heat keeps caller-wired loads and adds the routed share;
        // the step's internal-heat telemetry closes the balance exactly.
        let state = thermal.initial_state().unwrap();
        let mut command = ThermalCommand::idle_for(&thermal, 1.0);
        command.internal_heat_w = vec![50.0, 0.0];
        thermal
            .apply_waste_heat(&mut command, &power_report)
            .unwrap();
        assert_eq!(command.internal_heat_w, vec![250.0, 0.0]);
        let (_, report) = thermal.advance(&state, &command).expect("thermal step");
        assert_eq!(report.total_internal_heat_w, 250.0);
    }

    #[test]
    fn waste_heat_routing_fails_closed() {
        let bus = reactor_bus();
        let power_state = bus.initial_state().unwrap();
        let power_command = ElectricalPowerCommand::idle_for(&bus, 1.0);
        let (_, power_report) = bus.advance(&power_state, &power_command).expect("bus step");

        // Unknown power source: renamed parts cannot silently stop heating.
        let mut ghost = routed_system();
        ghost.heat_sources[0].source_name = "ghost-reactor".into();
        assert!(ghost.waste_heat_loads_w(&power_report).is_err());

        // Unknown node and over-subscribed or out-of-range fractions: spec errors.
        let mut bad_node = routed_system();
        bad_node.heat_sources[0].node = "ghost-node".into();
        assert!(bad_node.validate().is_err());
        let mut over = routed_system();
        over.heat_sources.push(ThermalHeatSource {
            source_name: "rx".into(),
            node: "shell".into(),
            fraction: 0.6,
            kind: ThermalHeatSourceKind::Power,
        });
        assert!(over.validate().is_err());
        let mut negative = routed_system();
        negative.heat_sources[0].fraction = -0.1;
        assert!(negative.validate().is_err());
    }

    #[test]
    fn generator_routes_draw_shaft_losses_into_nodes() {
        // 150 W shaft draw for 120 W electrical output rejects 30 W.
        assert_eq!(generator_waste_heat_w(150.0, 120.0).unwrap(), 30.0);
        assert_eq!(generator_waste_heat_w(0.0, 0.0).unwrap(), 0.0);
        assert!(generator_waste_heat_w(f64::NAN, 1.0).is_err());
        assert!(generator_waste_heat_w(-1.0, 0.0).is_err());

        let mut thermal = routed_system();
        thermal.heat_sources.push(ThermalHeatSource {
            source_name: "apu-1".into(),
            node: "shell".into(),
            fraction: 1.0,
            kind: ThermalHeatSourceKind::Generator,
        });
        let bus = reactor_bus();
        let power_state = bus.initial_state().unwrap();
        let power_command = ElectricalPowerCommand::idle_for(&bus, 1.0);
        let (_, power_report) = bus.advance(&power_state, &power_command).expect("bus step");
        let generators = [NamedWasteHeat {
            name: "apu-1".into(),
            waste_heat_w: 30.0,
        }];
        let loads = thermal
            .waste_heat_loads_w_with_generators(&power_report, &generators)
            .unwrap();
        // Reactor route lands 0 W (no load this step); generator route 30 W.
        assert_eq!(loads, vec![0.0, 30.0]);

        // Unknown generator mount fails closed; legacy entrypoint ignores
        // generator routes only by failing on the unknown generator name.
        let missing = [NamedWasteHeat {
            name: "other-apu".into(),
            waste_heat_w: 30.0,
        }];
        assert!(
            thermal
                .waste_heat_loads_w_with_generators(&power_report, &missing)
                .is_err()
        );
        assert!(thermal.waste_heat_loads_w(&power_report).is_err());
    }

    #[test]
    fn wheel_bank_losses_route_like_generator_mounts() {
        let mut thermal = routed_system();
        thermal.heat_sources.push(ThermalHeatSource {
            source_name: "trim-wheel".into(),
            node: "shell".into(),
            fraction: 0.5,
            kind: ThermalHeatSourceKind::ReactionWheel,
        });
        let bus = reactor_bus();
        let power_state = bus.initial_state().unwrap();
        let power_command = ElectricalPowerCommand::idle_for(&bus, 1.0);
        let (_, power_report) = bus.advance(&power_state, &power_command).expect("bus step");
        let inventory = [NamedWasteHeat {
            name: "trim-wheel".into(),
            waste_heat_w: 40.0,
        }];
        let loads = thermal
            .waste_heat_loads_w_with_generators(&power_report, &inventory)
            .unwrap();
        assert_eq!(loads, vec![0.0, 20.0]);
    }
}
