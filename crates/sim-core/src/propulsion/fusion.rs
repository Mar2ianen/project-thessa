//! Continuous and pulsed fusion propulsion (doc04 §14).
//!
//! The continuous torch converts charged fusion-product energy through a
//! magnetic nozzle into a selected working mass flow. The pulsed engine is a
//! separate event-driven pellet/energy-buffer model; it returns impulse per
//! shot rather than disguising pulses as a steady chemical-style engine.

use serde::{Deserialize, Serialize};

use super::{
    ElectricPropellant, PropulsionError, STANDARD_GRAVITY_MPS2, require_non_negative,
    require_positive, require_unit_interval,
};

const AVOGADRO_PER_MOL: f64 = 6.022_140_76e23;
const ELEMENTARY_CHARGE_C: f64 = 1.602_176_634e-19;
const MEV_TO_J: f64 = 1.0e6 * ELEMENTARY_CHARGE_C;
const VACUUM_PERMEABILITY_H_M: f64 = 4.0e-7 * std::f64::consts::PI;
const STEFAN_BOLTZMANN_W_M2_K4: f64 = 5.670_374_419e-8;
const COSMIC_BACKGROUND_TEMP_K: f64 = 2.725;
const COPPER_DENSITY_KG_M3: f64 = 8_960.0;

/// Fuel reactions with their nuclear energy release and charged-product
/// energy share. Values are per primary reaction event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum FusionReaction {
    DeuteriumTritium,
    /// Equal-weight D-D branches: `D+D -> T+p` and `D+D -> He3+n`.
    DeuteriumDeuterium,
    DeuteriumHelium3,
    ProtonBoron11,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct FusionReactionData {
    fuel_molar_mass_kg_mol: f64,
    q_mev: f64,
    charged_energy_fraction: f64,
}

impl FusionReaction {
    fn data(self) -> FusionReactionData {
        match self {
            Self::DeuteriumTritium => FusionReactionData {
                fuel_molar_mass_kg_mol: 0.005_030_15,
                q_mev: 17.6,
                charged_energy_fraction: 3.5 / 17.6,
            },
            Self::DeuteriumDeuterium => FusionReactionData {
                fuel_molar_mass_kg_mol: 0.004_028_20,
                q_mev: 3.65,
                charged_energy_fraction: 2.425 / 3.65,
            },
            Self::DeuteriumHelium3 => FusionReactionData {
                fuel_molar_mass_kg_mol: 0.005_030_15,
                q_mev: 18.3,
                charged_energy_fraction: 1.0,
            },
            Self::ProtonBoron11 => FusionReactionData {
                fuel_molar_mass_kg_mol: 0.012_012_5,
                q_mev: 8.68,
                charged_energy_fraction: 1.0,
            },
        }
    }

    /// Nuclear energy released per kilogram of reacting fuel mixture (J/kg).
    pub fn energy_release_j_kg(self) -> f64 {
        let data = self.data();
        data.q_mev * MEV_TO_J * AVOGADRO_PER_MOL / data.fuel_molar_mass_kg_mol
    }

    /// Fraction of reaction energy initially carried by charged products.
    pub fn charged_energy_fraction(self) -> f64 {
        self.data().charged_energy_fraction
    }
}

/// Geometry, reaction, and power/heat limits of a continuous fusion torch.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FusionTorchSpec {
    pub name: String,
    pub reaction: FusionReaction,
    pub working_fluid: ElectricPropellant,
    pub maximum_fusion_power_w: f64,
    /// Fusion energy released / external driver power at the operating point.
    pub fusion_gain: f64,
    pub maximum_working_flow_kg_s: f64,
    pub reactor_specific_power_w_kg: f64,
    pub plasma_coupling_efficiency: f64,
    pub magnetic_nozzle_efficiency: f64,
    pub nozzle_radius_m: f64,
    pub nozzle_length_m: f64,
    pub magnetic_field_t: f64,
    pub coil_current_density_a_m2: f64,
    pub structure_density_kg_m3: f64,
    pub structure_thickness_m: f64,
    pub radiator_area_m2: f64,
    pub radiator_temperature_k: f64,
    pub radiator_emissivity: f64,
    pub radiator_areal_density_kg_m2: f64,
}

/// Compiled continuous fusion torch.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledFusionTorch {
    pub reaction: FusionReaction,
    pub working_fluid: ElectricPropellant,
    pub maximum_fusion_power_w: f64,
    pub fusion_gain: f64,
    pub maximum_working_flow_kg_s: f64,
    pub charged_energy_fraction: f64,
    pub plasma_coupling_efficiency: f64,
    pub magnetic_nozzle_efficiency: f64,
    pub radiator_heat_rejection_w: f64,
    pub dry_mass_kg: f64,
}

impl FusionTorchSpec {
    pub fn compile(self) -> Result<CompiledFusionTorch, PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "fusion torch name must not be empty".into(),
            ));
        }
        let values = [
            self.maximum_fusion_power_w,
            self.fusion_gain,
            self.maximum_working_flow_kg_s,
            self.reactor_specific_power_w_kg,
            self.plasma_coupling_efficiency,
            self.magnetic_nozzle_efficiency,
            self.nozzle_radius_m,
            self.nozzle_length_m,
            self.magnetic_field_t,
            self.coil_current_density_a_m2,
            self.structure_density_kg_m3,
            self.structure_thickness_m,
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
            self.radiator_areal_density_kg_m2,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "fusion torch authoring values must be finite".into(),
            ));
        }
        require_positive(self.maximum_fusion_power_w, "fusion torch rated power")?;
        if self.fusion_gain <= 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "continuous fusion gain must exceed one".into(),
            ));
        }
        require_positive(self.maximum_working_flow_kg_s, "fusion working-fluid flow")?;
        require_positive(
            self.reactor_specific_power_w_kg,
            "fusion reactor specific power",
        )?;
        require_unit_interval(
            self.plasma_coupling_efficiency,
            "plasma coupling efficiency",
        )?;
        require_unit_interval(
            self.magnetic_nozzle_efficiency,
            "magnetic nozzle efficiency",
        )?;
        if self.plasma_coupling_efficiency == 0.0 || self.magnetic_nozzle_efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "fusion coupling and nozzle efficiencies must be in (0, 1]".into(),
            ));
        }
        require_positive(self.nozzle_radius_m, "fusion nozzle radius")?;
        require_positive(self.nozzle_length_m, "fusion nozzle length")?;
        require_positive(self.magnetic_field_t, "fusion nozzle field")?;
        require_positive(
            self.coil_current_density_a_m2,
            "fusion coil current density",
        )?;
        require_positive(self.structure_density_kg_m3, "fusion structure density")?;
        require_positive(self.structure_thickness_m, "fusion structure thickness")?;
        validate_radiator(
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
            self.radiator_areal_density_kg_m2,
        )?;

        let wall_volume_m3 = 2.0
            * std::f64::consts::PI
            * self.nozzle_radius_m
            * self.nozzle_length_m
            * self.structure_thickness_m
            + std::f64::consts::PI * self.nozzle_radius_m.powi(2) * self.structure_thickness_m;
        let coil_volume_m3 = self.magnetic_field_t * self.nozzle_length_m
            / (VACUUM_PERMEABILITY_H_M * self.coil_current_density_a_m2)
            * std::f64::consts::TAU
            * self.nozzle_radius_m;
        let radiator_heat_rejection_w = radiator_capacity(
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
        );
        let dry_mass_kg = self.maximum_fusion_power_w / self.reactor_specific_power_w_kg
            + wall_volume_m3 * self.structure_density_kg_m3
            + coil_volume_m3 * COPPER_DENSITY_KG_M3
            + self.radiator_area_m2 * self.radiator_areal_density_kg_m2;
        if !dry_mass_kg.is_finite() || dry_mass_kg <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "fusion torch mass sizing overflowed".into(),
            ));
        }
        Ok(CompiledFusionTorch {
            reaction: self.reaction,
            working_fluid: self.working_fluid,
            maximum_fusion_power_w: self.maximum_fusion_power_w,
            fusion_gain: self.fusion_gain,
            maximum_working_flow_kg_s: self.maximum_working_flow_kg_s,
            charged_energy_fraction: self.reaction.charged_energy_fraction(),
            plasma_coupling_efficiency: self.plasma_coupling_efficiency,
            magnetic_nozzle_efficiency: self.magnetic_nozzle_efficiency,
            radiator_heat_rejection_w,
            dry_mass_kg,
        })
    }
}

/// Continuous torch command: available driver power and working-fluid flow.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FusionTorchCommand {
    pub available_driver_power_w: f64,
    pub requested_working_flow_kg_s: f64,
}

/// Continuous torch operating point and energy/limit telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FusionTorchOperatingPoint {
    pub thrust_n: f64,
    pub exhaust_velocity_mps: f64,
    pub isp_s: f64,
    pub fusion_power_w: f64,
    pub driver_power_w: f64,
    pub fusion_fuel_flow_kg_s: f64,
    pub working_flow_kg_s: f64,
    pub total_exhaust_flow_kg_s: f64,
    pub jet_kinetic_power_w: f64,
    pub exhaust_internal_power_w: f64,
    pub waste_heat_w: f64,
    pub radiator_capacity_w: f64,
    pub power_limited: bool,
    pub flow_limited: bool,
    pub thermal_limited: bool,
}

impl CompiledFusionTorch {
    pub fn operating_point(
        &self,
        command: FusionTorchCommand,
    ) -> Result<FusionTorchOperatingPoint, PropulsionError> {
        if !command.available_driver_power_w.is_finite() || command.available_driver_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "available fusion driver power must be finite and >= 0".into(),
            ));
        }
        if !command.requested_working_flow_kg_s.is_finite()
            || command.requested_working_flow_kg_s < 0.0
        {
            return Err(PropulsionError::InvalidCommand(
                "fusion working-fluid flow must be finite and >= 0".into(),
            ));
        }
        let charged = self.charged_energy_fraction;
        let local_heat_per_fusion_w = 1.0 / self.fusion_gain
            + (1.0 - charged)
            + charged * (1.0 - self.plasma_coupling_efficiency);
        let thermal_fusion_limit_w = self.radiator_heat_rejection_w / local_heat_per_fusion_w;
        let requested_fusion_power_w = command.available_driver_power_w * self.fusion_gain;
        let fusion_power_w = requested_fusion_power_w
            .min(self.maximum_fusion_power_w)
            .min(thermal_fusion_limit_w);
        let driver_power_w = fusion_power_w / self.fusion_gain;
        let fusion_fuel_flow_kg_s = fusion_power_w / self.reaction.energy_release_j_kg();
        let working_flow_kg_s = command
            .requested_working_flow_kg_s
            .min(self.maximum_working_flow_kg_s);
        let total_exhaust_flow_kg_s = fusion_fuel_flow_kg_s + working_flow_kg_s;
        let useful_fraction =
            charged * self.plasma_coupling_efficiency * self.magnetic_nozzle_efficiency;
        let jet_kinetic_power_w = fusion_power_w * useful_fraction;
        let exhaust_internal_power_w = fusion_power_w
            * charged
            * self.plasma_coupling_efficiency
            * (1.0 - self.magnetic_nozzle_efficiency);
        let waste_heat_w = driver_power_w
            + fusion_power_w * (1.0 - charged)
            + fusion_power_w * charged * (1.0 - self.plasma_coupling_efficiency);
        let thrust_n = if total_exhaust_flow_kg_s > 0.0 {
            (2.0 * jet_kinetic_power_w * total_exhaust_flow_kg_s).sqrt()
        } else {
            0.0
        };
        let exhaust_velocity_mps = if total_exhaust_flow_kg_s > 0.0 {
            thrust_n / total_exhaust_flow_kg_s
        } else {
            0.0
        };
        let point = FusionTorchOperatingPoint {
            thrust_n,
            exhaust_velocity_mps,
            isp_s: exhaust_velocity_mps / STANDARD_GRAVITY_MPS2,
            fusion_power_w,
            driver_power_w,
            fusion_fuel_flow_kg_s,
            working_flow_kg_s,
            total_exhaust_flow_kg_s,
            jet_kinetic_power_w,
            exhaust_internal_power_w,
            waste_heat_w,
            radiator_capacity_w: self.radiator_heat_rejection_w,
            power_limited: requested_fusion_power_w > self.maximum_fusion_power_w,
            flow_limited: command.requested_working_flow_kg_s > self.maximum_working_flow_kg_s,
            thermal_limited: thermal_fusion_limit_w
                < requested_fusion_power_w.min(self.maximum_fusion_power_w),
        };
        if [
            point.thrust_n,
            point.exhaust_velocity_mps,
            point.isp_s,
            point.fusion_power_w,
            point.driver_power_w,
            point.fusion_fuel_flow_kg_s,
            point.working_flow_kg_s,
            point.total_exhaust_flow_kg_s,
            point.jet_kinetic_power_w,
            point.exhaust_internal_power_w,
            point.waste_heat_w,
        ]
        .iter()
        .any(|value| !value.is_finite())
        {
            return Err(PropulsionError::InvalidCommand(
                "fusion torch operating point overflowed".into(),
            ));
        }
        Ok(point)
    }
}

fn radiator_capacity(area_m2: f64, temperature_k: f64, emissivity: f64) -> f64 {
    emissivity
        * STEFAN_BOLTZMANN_W_M2_K4
        * area_m2
        * (temperature_k.powi(4) - COSMIC_BACKGROUND_TEMP_K.powi(4))
}

fn validate_radiator(
    area_m2: f64,
    temperature_k: f64,
    emissivity: f64,
    areal_density_kg_m2: f64,
) -> Result<(), PropulsionError> {
    require_positive(area_m2, "radiator area")?;
    require_positive(temperature_k, "radiator temperature")?;
    require_unit_interval(emissivity, "radiator emissivity")?;
    if emissivity == 0.0 {
        return Err(PropulsionError::InvalidSpec(
            "radiator emissivity must be in (0, 1]".into(),
        ));
    }
    require_positive(areal_density_kg_m2, "radiator areal density")?;
    let capacity_w = radiator_capacity(area_m2, temperature_k, emissivity);
    if !capacity_w.is_finite() || capacity_w <= 0.0 {
        return Err(PropulsionError::InvalidSpec(
            "radiator heat rejection must be finite and above the cosmic background".into(),
        ));
    }
    Ok(())
}

/// One installed continuous fusion torch.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FusionTorchMount {
    pub name: String,
    pub engine: CompiledFusionTorch,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
}

impl FusionTorchMount {
    pub fn validate(&self) -> Result<(), PropulsionError> {
        validate_mount_frame(&self.name, &self.position_body_m, &self.thrust_axis_body)
    }

    pub fn operating_point(
        &self,
        command: FusionTorchCommand,
    ) -> Result<FusionTorchOperatingPoint, PropulsionError> {
        self.engine.operating_point(command)
    }
}

/// Authoring data for a discrete pellet/impulse fusion engine.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PulsedFusionSpec {
    pub name: String,
    pub reaction: FusionReaction,
    pub working_fluid: ElectricPropellant,
    pub fuel_mass_per_pulse_kg: f64,
    pub working_fluid_mass_per_pulse_kg: f64,
    pub fusion_gain: f64,
    pub plasma_coupling_efficiency: f64,
    pub magnetic_nozzle_efficiency: f64,
    pub maximum_pulse_frequency_hz: f64,
    pub pulse_duration_s: f64,
    pub maximum_charge_power_w: f64,
    pub energy_buffer_capacity_pulses: u8,
    pub energy_buffer_specific_energy_j_kg: f64,
    pub pulse_system_specific_power_w_kg: f64,
    pub chamber_radius_m: f64,
    pub chamber_length_m: f64,
    pub magnetic_field_t: f64,
    pub coil_current_density_a_m2: f64,
    pub structure_density_kg_m3: f64,
    pub structure_thickness_m: f64,
    pub radiator_area_m2: f64,
    pub radiator_temperature_k: f64,
    pub radiator_emissivity: f64,
    pub radiator_areal_density_kg_m2: f64,
}

/// Compiled pulsed fusion system. Driver storage and pulse cadence are
/// separate from the continuous fusion-torch model.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledPulsedFusion {
    pub reaction: FusionReaction,
    pub working_fluid: ElectricPropellant,
    pub fuel_mass_per_pulse_kg: f64,
    pub working_fluid_mass_per_pulse_kg: f64,
    pub fusion_energy_per_pulse_j: f64,
    pub peak_fusion_power_w: f64,
    pub driver_energy_per_pulse_j: f64,
    pub jet_energy_per_pulse_j: f64,
    pub exhaust_internal_energy_per_pulse_j: f64,
    pub waste_heat_per_pulse_j: f64,
    pub rated_pulse_interval_s: f64,
    pub pulse_interval_s: f64,
    pub thermal_pulse_interval_s: f64,
    pub maximum_charge_power_w: f64,
    pub buffer_capacity_j: f64,
    pub radiator_heat_rejection_w: f64,
    pub dry_mass_kg: f64,
}

impl PulsedFusionSpec {
    pub fn compile(self) -> Result<CompiledPulsedFusion, PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "pulsed fusion name must not be empty".into(),
            ));
        }
        let values = [
            self.fuel_mass_per_pulse_kg,
            self.working_fluid_mass_per_pulse_kg,
            self.fusion_gain,
            self.plasma_coupling_efficiency,
            self.magnetic_nozzle_efficiency,
            self.maximum_pulse_frequency_hz,
            self.pulse_duration_s,
            self.maximum_charge_power_w,
            self.energy_buffer_specific_energy_j_kg,
            self.pulse_system_specific_power_w_kg,
            self.chamber_radius_m,
            self.chamber_length_m,
            self.magnetic_field_t,
            self.coil_current_density_a_m2,
            self.structure_density_kg_m3,
            self.structure_thickness_m,
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
            self.radiator_areal_density_kg_m2,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "pulsed fusion authoring values must be finite".into(),
            ));
        }
        require_positive(self.fuel_mass_per_pulse_kg, "fusion fuel mass per pulse")?;
        require_non_negative(
            self.working_fluid_mass_per_pulse_kg,
            "working-fluid mass per pulse",
        )?;
        if self.fusion_gain <= 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "pulsed fusion gain must exceed one".into(),
            ));
        }
        require_unit_interval(self.plasma_coupling_efficiency, "pulse plasma coupling")?;
        require_unit_interval(self.magnetic_nozzle_efficiency, "pulse nozzle efficiency")?;
        if self.plasma_coupling_efficiency == 0.0 || self.magnetic_nozzle_efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "pulse coupling and nozzle efficiencies must be in (0, 1]".into(),
            ));
        }
        require_positive(self.maximum_pulse_frequency_hz, "maximum pulse frequency")?;
        require_positive(self.pulse_duration_s, "fusion pulse duration")?;
        if self.maximum_pulse_frequency_hz * self.pulse_duration_s > 1.0 {
            return Err(PropulsionError::InvalidSpec(
                "pulse duration must fit within the rated pulse interval".into(),
            ));
        }
        require_positive(self.maximum_charge_power_w, "fusion driver charge power")?;
        if !(1..=16).contains(&self.energy_buffer_capacity_pulses) {
            return Err(PropulsionError::InvalidSpec(
                "fusion buffer capacity must be in 1..=16 pulse energies".into(),
            ));
        }
        require_positive(
            self.energy_buffer_specific_energy_j_kg,
            "fusion energy-buffer specific energy",
        )?;
        require_positive(
            self.pulse_system_specific_power_w_kg,
            "fusion pulse-system specific power",
        )?;
        require_positive(self.chamber_radius_m, "fusion pulse chamber radius")?;
        require_positive(self.chamber_length_m, "fusion pulse chamber length")?;
        require_positive(self.magnetic_field_t, "pulsed fusion field")?;
        require_positive(
            self.coil_current_density_a_m2,
            "pulsed fusion coil current density",
        )?;
        require_positive(
            self.structure_density_kg_m3,
            "pulsed fusion structure density",
        )?;
        require_positive(self.structure_thickness_m, "pulsed fusion wall thickness")?;
        validate_radiator(
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
            self.radiator_areal_density_kg_m2,
        )?;

        let reaction_data = self.reaction.data();
        let fusion_energy_per_pulse_j =
            self.fuel_mass_per_pulse_kg * self.reaction.energy_release_j_kg();
        let driver_energy_per_pulse_j = fusion_energy_per_pulse_j / self.fusion_gain;
        let charged = reaction_data.charged_energy_fraction;
        let coupled_nozzle_fraction =
            self.plasma_coupling_efficiency * self.magnetic_nozzle_efficiency;
        let jet_energy_per_pulse_j = fusion_energy_per_pulse_j * charged * coupled_nozzle_fraction;
        let exhaust_internal_energy_per_pulse_j = fusion_energy_per_pulse_j
            * charged
            * self.plasma_coupling_efficiency
            * (1.0 - self.magnetic_nozzle_efficiency);
        let waste_heat_per_pulse_j = driver_energy_per_pulse_j
            + fusion_energy_per_pulse_j * (1.0 - charged)
            + fusion_energy_per_pulse_j * charged * (1.0 - self.plasma_coupling_efficiency);
        // The buffer is modeled as lossless: driver energy is charged into
        // storage and later released to the pellet system without hidden heat.
        let rated_pulse_interval_s = 1.0 / self.maximum_pulse_frequency_hz;
        let radiator_heat_rejection_w = radiator_capacity(
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
        );
        let thermal_pulse_interval_s = if waste_heat_per_pulse_j > 0.0 {
            waste_heat_per_pulse_j / radiator_heat_rejection_w
        } else {
            0.0
        };
        let pulse_interval_s = rated_pulse_interval_s
            .max(thermal_pulse_interval_s)
            .max(self.pulse_duration_s);
        let peak_fusion_power_w = fusion_energy_per_pulse_j / self.pulse_duration_s;
        let buffer_capacity_j =
            driver_energy_per_pulse_j * f64::from(self.energy_buffer_capacity_pulses);
        let wall_volume_m3 = 2.0
            * std::f64::consts::PI
            * self.chamber_radius_m
            * self.chamber_length_m
            * self.structure_thickness_m
            + std::f64::consts::PI * self.chamber_radius_m.powi(2) * self.structure_thickness_m;
        let coil_volume_m3 = self.magnetic_field_t * self.chamber_length_m
            / (VACUUM_PERMEABILITY_H_M * self.coil_current_density_a_m2)
            * std::f64::consts::TAU
            * self.chamber_radius_m;
        let dry_mass_kg = peak_fusion_power_w / self.pulse_system_specific_power_w_kg
            + buffer_capacity_j / self.energy_buffer_specific_energy_j_kg
            + self.maximum_charge_power_w / self.pulse_system_specific_power_w_kg
            + wall_volume_m3 * self.structure_density_kg_m3
            + coil_volume_m3 * COPPER_DENSITY_KG_M3
            + self.radiator_area_m2 * self.radiator_areal_density_kg_m2;
        if !buffer_capacity_j.is_finite()
            || !pulse_interval_s.is_finite()
            || !thermal_pulse_interval_s.is_finite()
            || !fusion_energy_per_pulse_j.is_finite()
            || fusion_energy_per_pulse_j <= 0.0
            || !driver_energy_per_pulse_j.is_finite()
            || driver_energy_per_pulse_j <= 0.0
            || !jet_energy_per_pulse_j.is_finite()
            || jet_energy_per_pulse_j <= 0.0
            || !exhaust_internal_energy_per_pulse_j.is_finite()
            || !waste_heat_per_pulse_j.is_finite()
            || !peak_fusion_power_w.is_finite()
            || peak_fusion_power_w <= 0.0
            || buffer_capacity_j <= 0.0
            || !dry_mass_kg.is_finite()
            || dry_mass_kg <= 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "pulsed fusion mass/energy sizing overflowed".into(),
            ));
        }
        Ok(CompiledPulsedFusion {
            reaction: self.reaction,
            working_fluid: self.working_fluid,
            fuel_mass_per_pulse_kg: self.fuel_mass_per_pulse_kg,
            working_fluid_mass_per_pulse_kg: self.working_fluid_mass_per_pulse_kg,
            fusion_energy_per_pulse_j,
            peak_fusion_power_w,
            driver_energy_per_pulse_j,
            jet_energy_per_pulse_j,
            exhaust_internal_energy_per_pulse_j,
            waste_heat_per_pulse_j,
            rated_pulse_interval_s,
            pulse_interval_s,
            thermal_pulse_interval_s,
            maximum_charge_power_w: self.maximum_charge_power_w,
            buffer_capacity_j,
            radiator_heat_rejection_w,
            dry_mass_kg,
        })
    }
}

/// Persistent state for a pulsed fusion drive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct PulsedFusionState {
    /// Elapsed time since the last shot, paused while the system is disarmed.
    pub pulse_phase_s: f64,
    /// Charged driver energy currently held by the finite pulse buffer (J).
    pub stored_driver_energy_j: f64,
    pub cumulative_shots: u64,
}

/// Bus/arming command for a pulsed fusion engine.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PulsedFusionCommand {
    pub available_charge_power_w: f64,
    pub armed: bool,
}

/// Step result for a pulsed fusion engine. Impulse is accumulated over this
/// physics step; the returned next state must be stored for the next step.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PulsedFusionOperatingPoint {
    pub pulses_fired: u32,
    pub impulse_ns: f64,
    pub average_thrust_n: f64,
    pub fuel_mass_flow_kg_s: f64,
    pub working_flow_kg_s: f64,
    pub total_exhaust_flow_kg_s: f64,
    pub effective_exhaust_velocity_mps: f64,
    pub effective_isp_s: f64,
    pub fusion_power_w: f64,
    pub charged_driver_power_w: f64,
    pub driver_energy_consumed_j: f64,
    pub bus_energy_input_j: f64,
    pub buffer_energy_delta_j: f64,
    pub fusion_energy_j: f64,
    pub jet_kinetic_energy_j: f64,
    pub exhaust_internal_energy_j: f64,
    pub waste_heat_j: f64,
    pub waste_heat_w: f64,
    pub radiator_capacity_w: f64,
    pub stored_driver_energy_j: f64,
    pub power_limited: bool,
    pub thermal_limited: bool,
}

impl CompiledPulsedFusion {
    pub fn advance(
        &self,
        state: PulsedFusionState,
        command: PulsedFusionCommand,
        dt_s: f64,
    ) -> Result<(PulsedFusionState, PulsedFusionOperatingPoint), PropulsionError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "pulsed fusion step must be finite and > 0 seconds".into(),
            ));
        }
        if !command.available_charge_power_w.is_finite() || command.available_charge_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "fusion buffer charge power must be finite and >= 0".into(),
            ));
        }
        if !state.pulse_phase_s.is_finite()
            || state.pulse_phase_s < 0.0
            || !state.stored_driver_energy_j.is_finite()
            || state.stored_driver_energy_j < 0.0
            || state.stored_driver_energy_j > self.buffer_capacity_j
        {
            return Err(PropulsionError::InvalidCommand(
                "pulsed fusion state is outside its finite energy/phase bounds".into(),
            ));
        }

        let rated_charge_power_w = command
            .available_charge_power_w
            .min(self.maximum_charge_power_w);
        let mut stored_j = state.stored_driver_energy_j;
        let initial_stored_j = stored_j;
        let mut phase_s = state.pulse_phase_s.min(self.pulse_interval_s);
        let mut remaining_s = dt_s;
        let mut bus_energy_input_j = 0.0;
        let mut pulses_fired = 0_u32;
        let epsilon_s = dt_s * 16.0 * f64::EPSILON;
        let energy_epsilon_j = self.driver_energy_per_pulse_j * 16.0 * f64::EPSILON;

        let charge = |elapsed_s: f64, stored: &mut f64, bus_input: &mut f64| {
            let added_j = (self.buffer_capacity_j - *stored)
                .max(0.0)
                .min(rated_charge_power_w * elapsed_s);
            *stored = (*stored + added_j).min(self.buffer_capacity_j);
            *bus_input += added_j;
        };

        if !command.armed {
            charge(remaining_s, &mut stored_j, &mut bus_energy_input_j);
            remaining_s = 0.0;
        }
        let mut iterations = 0_u32;
        while remaining_s > epsilon_s {
            iterations += 1;
            if iterations > 1_000_000 {
                return Err(PropulsionError::InvalidCommand(
                    "pulsed fusion cadence exceeded the bounded step event count".into(),
                ));
            }
            let time_to_pulse_s = (self.pulse_interval_s - phase_s).max(0.0);
            let time_to_charge_s = if stored_j + energy_epsilon_j >= self.driver_energy_per_pulse_j
            {
                0.0
            } else if rated_charge_power_w > 0.0 {
                (self.driver_energy_per_pulse_j - stored_j) / rated_charge_power_w
            } else {
                f64::INFINITY
            };
            let event_wait_s = time_to_pulse_s.max(time_to_charge_s);
            if !event_wait_s.is_finite() || event_wait_s > remaining_s {
                charge(remaining_s, &mut stored_j, &mut bus_energy_input_j);
                phase_s = (phase_s + remaining_s).min(self.pulse_interval_s);
                break;
            }
            charge(event_wait_s, &mut stored_j, &mut bus_energy_input_j);
            phase_s = (phase_s + event_wait_s).min(self.pulse_interval_s);
            remaining_s -= event_wait_s;
            if phase_s + epsilon_s >= self.pulse_interval_s
                && stored_j + energy_epsilon_j >= self.driver_energy_per_pulse_j
            {
                stored_j = (stored_j - self.driver_energy_per_pulse_j).max(0.0);
                phase_s = 0.0;
                pulses_fired += 1;
            } else if event_wait_s <= epsilon_s {
                break;
            }
        }

        let fusion_energy_j = f64::from(pulses_fired) * self.fusion_energy_per_pulse_j;
        let jet_kinetic_energy_j = f64::from(pulses_fired) * self.jet_energy_per_pulse_j;
        let exhaust_internal_energy_j =
            f64::from(pulses_fired) * self.exhaust_internal_energy_per_pulse_j;
        let waste_heat_j = f64::from(pulses_fired) * self.waste_heat_per_pulse_j;
        let buffer_energy_delta_j = stored_j - initial_stored_j;
        let impulse_per_pulse_ns = (2.0
            * self.jet_energy_per_pulse_j
            * (self.fuel_mass_per_pulse_kg + self.working_fluid_mass_per_pulse_kg))
            .sqrt();
        let impulse_ns = f64::from(pulses_fired) * impulse_per_pulse_ns;
        let total_mass = f64::from(pulses_fired)
            * (self.fuel_mass_per_pulse_kg + self.working_fluid_mass_per_pulse_kg);
        let total_exhaust_flow_kg_s = total_mass / dt_s;
        let effective_exhaust_velocity_mps = if total_mass > 0.0 {
            impulse_ns / total_mass
        } else {
            0.0
        };
        let point = PulsedFusionOperatingPoint {
            pulses_fired,
            impulse_ns,
            average_thrust_n: impulse_ns / dt_s,
            fuel_mass_flow_kg_s: f64::from(pulses_fired) * self.fuel_mass_per_pulse_kg / dt_s,
            working_flow_kg_s: f64::from(pulses_fired) * self.working_fluid_mass_per_pulse_kg
                / dt_s,
            total_exhaust_flow_kg_s,
            effective_exhaust_velocity_mps,
            effective_isp_s: effective_exhaust_velocity_mps / STANDARD_GRAVITY_MPS2,
            fusion_power_w: fusion_energy_j / dt_s,
            charged_driver_power_w: f64::from(pulses_fired) * self.driver_energy_per_pulse_j / dt_s,
            driver_energy_consumed_j: f64::from(pulses_fired) * self.driver_energy_per_pulse_j,
            bus_energy_input_j,
            buffer_energy_delta_j,
            fusion_energy_j,
            jet_kinetic_energy_j,
            exhaust_internal_energy_j,
            waste_heat_j,
            waste_heat_w: waste_heat_j / dt_s,
            radiator_capacity_w: self.radiator_heat_rejection_w,
            stored_driver_energy_j: stored_j,
            power_limited: command.armed
                && rated_charge_power_w * self.pulse_interval_s < self.driver_energy_per_pulse_j,
            thermal_limited: self.thermal_pulse_interval_s > self.rated_pulse_interval_s,
        };
        let cumulative_shots = state
            .cumulative_shots
            .checked_add(u64::from(pulses_fired))
            .ok_or_else(|| {
                PropulsionError::InvalidCommand("pulsed fusion shot counter overflowed".into())
            })?;
        if [
            point.impulse_ns,
            point.average_thrust_n,
            point.fuel_mass_flow_kg_s,
            point.working_flow_kg_s,
            point.total_exhaust_flow_kg_s,
            point.effective_exhaust_velocity_mps,
            point.effective_isp_s,
            point.fusion_power_w,
            point.charged_driver_power_w,
            point.bus_energy_input_j,
            point.buffer_energy_delta_j,
            point.fusion_energy_j,
            point.jet_kinetic_energy_j,
            point.exhaust_internal_energy_j,
            point.waste_heat_j,
            point.waste_heat_w,
            point.stored_driver_energy_j,
        ]
        .iter()
        .any(|value| !value.is_finite())
        {
            return Err(PropulsionError::InvalidCommand(
                "pulsed fusion operating point overflowed".into(),
            ));
        }
        Ok((
            PulsedFusionState {
                pulse_phase_s: phase_s,
                stored_driver_energy_j: stored_j,
                cumulative_shots,
            },
            point,
        ))
    }
}

/// One installed pulsed fusion system.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PulsedFusionMount {
    pub name: String,
    pub engine: CompiledPulsedFusion,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
}

impl PulsedFusionMount {
    pub fn validate(&self) -> Result<(), PropulsionError> {
        validate_mount_frame(&self.name, &self.position_body_m, &self.thrust_axis_body)
    }
}

fn validate_mount_frame(
    name: &str,
    position_body_m: &[f64; 3],
    thrust_axis_body: &[f64; 3],
) -> Result<(), PropulsionError> {
    if name.trim().is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "fusion mount needs a name".into(),
        ));
    }
    if position_body_m.iter().any(|value| !value.is_finite())
        || thrust_axis_body.iter().any(|value| !value.is_finite())
    {
        return Err(PropulsionError::InvalidSpec(
            "fusion mount frame must be finite".into(),
        ));
    }
    let axis_norm = thrust_axis_body
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if (axis_norm - 1.0).abs() > 1e-9 {
        return Err(PropulsionError::InvalidSpec(
            "fusion mount thrust axis must be unit length".into(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn torch_spec() -> FusionTorchSpec {
        FusionTorchSpec {
            name: "dt-torch-test".into(),
            reaction: FusionReaction::DeuteriumTritium,
            working_fluid: ElectricPropellant::Hydrogen,
            maximum_fusion_power_w: 100.0e6,
            fusion_gain: 10.0,
            maximum_working_flow_kg_s: 1.0e-3,
            reactor_specific_power_w_kg: 10_000.0,
            plasma_coupling_efficiency: 0.9,
            magnetic_nozzle_efficiency: 0.8,
            nozzle_radius_m: 0.5,
            nozzle_length_m: 2.0,
            magnetic_field_t: 1.0,
            coil_current_density_a_m2: 4.0e7,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.01,
            radiator_area_m2: 3_000.0,
            radiator_temperature_k: 1_000.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
        }
    }

    fn pulsed_spec() -> PulsedFusionSpec {
        PulsedFusionSpec {
            name: "pellet-drive-test".into(),
            reaction: FusionReaction::DeuteriumTritium,
            working_fluid: ElectricPropellant::Hydrogen,
            fuel_mass_per_pulse_kg: 1.0e-9,
            working_fluid_mass_per_pulse_kg: 1.0e-7,
            fusion_gain: 10.0,
            plasma_coupling_efficiency: 0.9,
            magnetic_nozzle_efficiency: 0.8,
            maximum_pulse_frequency_hz: 0.1,
            pulse_duration_s: 0.01,
            maximum_charge_power_w: 100_000.0,
            energy_buffer_capacity_pulses: 2,
            energy_buffer_specific_energy_j_kg: 1.0e6,
            pulse_system_specific_power_w_kg: 1.0e6,
            chamber_radius_m: 0.1,
            chamber_length_m: 0.5,
            magnetic_field_t: 1.0,
            coil_current_density_a_m2: 4.0e7,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.01,
            radiator_area_m2: 10.0,
            radiator_temperature_k: 1_000.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
        }
    }

    fn assert_close(actual: f64, expected: f64, relative_tolerance: f64) {
        assert!(
            (actual - expected).abs() <= relative_tolerance * expected.abs().max(1.0),
            "{actual} != {expected}"
        );
    }

    #[test]
    fn dt_reaction_energy_and_torch_first_law_match_reference_values() {
        let dt_specific_energy = FusionReaction::DeuteriumTritium.energy_release_j_kg();
        assert!((3.3e14..=3.5e14).contains(&dt_specific_energy));
        assert_close(
            FusionReaction::DeuteriumTritium.charged_energy_fraction(),
            3.5 / 17.6,
            1e-14,
        );

        let engine = torch_spec().compile().expect("torch compiles");
        let point = engine
            .operating_point(FusionTorchCommand {
                available_driver_power_w: 20.0e6,
                requested_working_flow_kg_s: 2.0e-3,
            })
            .expect("torch point");
        assert_close(point.fusion_power_w, 100.0e6, 1e-14);
        assert!(point.power_limited);
        assert!(point.flow_limited);
        assert!(point.waste_heat_w <= point.radiator_capacity_w);
        let energy_out_w =
            point.jet_kinetic_power_w + point.exhaust_internal_power_w + point.waste_heat_w;
        assert_close(
            energy_out_w,
            point.fusion_power_w + point.driver_power_w,
            1e-12,
        );
        assert!(point.thrust_n > 0.0);
        assert_close(
            point.thrust_n,
            point.total_exhaust_flow_kg_s * point.exhaust_velocity_mps,
            1e-12,
        );

        let mut small_radiator = torch_spec();
        small_radiator.radiator_area_m2 = 0.01;
        let thermal_engine = small_radiator.compile().expect("small radiator");
        let thermal_point = thermal_engine
            .operating_point(FusionTorchCommand {
                available_driver_power_w: 100.0e6,
                requested_working_flow_kg_s: 1.0e-4,
            })
            .expect("thermally limited torch point");
        assert!(thermal_point.thermal_limited);
        assert!(thermal_point.waste_heat_w <= thermal_point.radiator_capacity_w + 1e-8);
    }

    #[test]
    fn pulsed_fusion_respects_pulse_events_and_buffer_energy_balance() {
        let engine = pulsed_spec().compile().expect("pulsed system compiles");
        assert_close(engine.pulse_interval_s, 10.0, 1e-14);
        assert!(engine.peak_fusion_power_w > engine.fusion_energy_per_pulse_j);
        let initial = PulsedFusionState {
            pulse_phase_s: 0.0,
            stored_driver_energy_j: engine.buffer_capacity_j,
            cumulative_shots: 7,
        };
        let command = PulsedFusionCommand {
            available_charge_power_w: 100_000.0,
            armed: true,
        };
        let (next, point) = engine.advance(initial, command, 25.0).expect("advance");
        assert_eq!(point.pulses_fired, 2);
        assert_eq!(next.cumulative_shots, 9);
        assert_close(next.pulse_phase_s, 5.0, 1e-12);
        assert!(point.impulse_ns > 0.0);
        assert_close(
            point.bus_energy_input_j - point.driver_energy_consumed_j,
            point.buffer_energy_delta_j,
            1e-10,
        );
        assert_close(
            point.bus_energy_input_j + point.fusion_energy_j,
            point.buffer_energy_delta_j
                + point.jet_kinetic_energy_j
                + point.exhaust_internal_energy_j
                + point.waste_heat_j,
            1e-10,
        );

        // Five smaller steps must preserve the two scheduled shots and the
        // energy-buffer state of one 25-second step.
        let mut partitioned_state = initial;
        let mut partitioned_shots = 0;
        let mut partitioned_impulse = 0.0;
        for _ in 0..5 {
            let (state, step) = engine
                .advance(partitioned_state, command, 5.0)
                .expect("partitioned advance");
            partitioned_state = state;
            partitioned_shots += step.pulses_fired;
            partitioned_impulse += step.impulse_ns;
        }
        assert_eq!(partitioned_shots, point.pulses_fired);
        assert_close(partitioned_state.pulse_phase_s, next.pulse_phase_s, 1e-12);
        assert_close(
            partitioned_state.stored_driver_energy_j,
            next.stored_driver_energy_j,
            1e-12,
        );
        assert_close(partitioned_impulse, point.impulse_ns, 1e-12);
    }

    #[test]
    fn pulse_driver_and_radiator_reduce_cadence_and_disarm_only_charges() {
        let mut spec = pulsed_spec();
        spec.radiator_area_m2 = 1.0e-4;
        let engine = spec.compile().expect("thermally throttled pulses");
        assert!(engine.thermal_pulse_interval_s > engine.rated_pulse_interval_s);
        let ready = PulsedFusionState {
            pulse_phase_s: engine.pulse_interval_s - 1.0,
            stored_driver_energy_j: engine.driver_energy_per_pulse_j,
            cumulative_shots: 0,
        };
        let (after_shot, point) = engine
            .advance(
                ready,
                PulsedFusionCommand {
                    available_charge_power_w: engine.maximum_charge_power_w,
                    armed: true,
                },
                1.0,
            )
            .expect("thermal boundary shot");
        assert_eq!(point.pulses_fired, 1);
        assert!(point.thermal_limited);
        assert!(
            engine.waste_heat_per_pulse_j / engine.pulse_interval_s
                <= point.radiator_capacity_w + 1e-8
        );
        assert!(after_shot.pulse_phase_s < 1e-9);

        let cold_start = PulsedFusionState::default();
        let (charged, no_fire) = engine
            .advance(
                cold_start,
                PulsedFusionCommand {
                    available_charge_power_w: engine.maximum_charge_power_w,
                    armed: false,
                },
                1.0,
            )
            .expect("disarmed charge");
        assert_eq!(no_fire.pulses_fired, 0);
        assert_eq!(charged.pulse_phase_s, 0.0);
        assert!(charged.stored_driver_energy_j > 0.0);
        assert_eq!(charged.cumulative_shots, 0);

        let mut limited_spec = pulsed_spec();
        limited_spec.maximum_charge_power_w = 1_000.0;
        let limited = limited_spec.compile().expect("low-power charger");
        let (_, limited_point) = limited
            .advance(
                PulsedFusionState::default(),
                PulsedFusionCommand {
                    available_charge_power_w: 1_000.0,
                    armed: true,
                },
                10.0,
            )
            .expect("power-limited pulse advance");
        assert_eq!(limited_point.pulses_fired, 0);
        assert!(limited_point.power_limited);
    }

    #[test]
    fn invalid_fusion_designs_commands_and_state_fail_closed() {
        let mut bad_torch = torch_spec();
        bad_torch.fusion_gain = f64::NAN;
        assert!(bad_torch.compile().is_err());
        let torch = torch_spec().compile().expect("torch");
        assert!(
            torch
                .operating_point(FusionTorchCommand {
                    available_driver_power_w: f64::INFINITY,
                    requested_working_flow_kg_s: 0.0,
                })
                .is_err()
        );

        let pulsed = pulsed_spec().compile().expect("pulsed system");
        assert!(
            pulsed
                .advance(
                    PulsedFusionState {
                        stored_driver_energy_j: pulsed.buffer_capacity_j + 1.0,
                        ..PulsedFusionState::default()
                    },
                    PulsedFusionCommand {
                        available_charge_power_w: 1.0,
                        armed: true,
                    },
                    1.0,
                )
                .is_err()
        );
        assert!(
            pulsed
                .advance(
                    PulsedFusionState::default(),
                    PulsedFusionCommand {
                        available_charge_power_w: 1.0,
                        armed: true,
                    },
                    0.0,
                )
                .is_err()
        );
    }
}
