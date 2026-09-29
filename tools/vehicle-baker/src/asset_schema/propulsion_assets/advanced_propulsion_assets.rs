//! Electric, fusion, and shaft-driven propulsion asset compilation.

use super::*;

/// One installed reaction-control nozzle. The nested `thruster` table is
/// externally tagged as `monoprop` or `cold-gas` and uses the corresponding
/// validated propulsion design.
#[derive(Debug, Deserialize)]
pub(crate) struct RcsMountAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    direction_body: [f64; 3],
    thruster: RcsThrusterAsset,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum RcsThrusterAsset {
    Monoprop(MonopropThrusterSpec),
    ColdGas(ColdGasThrusterSpec),
}

impl RcsMountAsset {
    pub(crate) fn bake(self) -> Result<RcsMount, Box<dyn Error>> {
        let thruster = match self.thruster {
            RcsThrusterAsset::Monoprop(spec) => RcsThruster::Monoprop(spec.compile()?),
            RcsThrusterAsset::ColdGas(spec) => RcsThruster::ColdGas(spec.compile()?),
        };
        let mount = RcsMount {
            name: self.name,
            thruster,
            position_body_m: self.mount_position_body_m,
            direction_body: self.direction_body,
        };
        mount.validate()?;
        Ok(mount)
    }
}

/// Installed electric spacecraft thruster (`[[electric_thrusters]]`).
/// `design` is a tagged table selecting gridded-ion, Hall, MPD, resistojet,
/// or arcjet hardware; each compiled mount includes its power processor and
/// radiator mass.
#[derive(Debug, Deserialize)]
pub(crate) struct ElectricThrusterAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    propellant: ElectricPropellant,
    design: ElectricThrusterDesign,
    maximum_power_w: f64,
    maximum_mass_flow_kg_s: f64,
    power_processor_specific_power_w_kg: f64,
    structure_density_kg_m3: f64,
    structure_thickness_m: f64,
    radiator_area_m2: f64,
    radiator_temperature_k: f64,
    radiator_emissivity: f64,
    radiator_areal_density_kg_m2: f64,
    ionization_efficiency: f64,
    #[serde(default = "standard_propellant_inlet_temp")]
    inlet_temperature_k: f64,
}

fn standard_propellant_inlet_temp() -> f64 {
    300.0
}

impl ElectricThrusterAsset {
    pub(crate) fn bake(self) -> Result<ElectricThrusterMount, Box<dyn Error>> {
        let engine = ElectricThrusterSpec {
            name: self.name.clone(),
            propellant: self.propellant,
            design: self.design,
            maximum_power_w: self.maximum_power_w,
            maximum_mass_flow_kg_s: self.maximum_mass_flow_kg_s,
            power_processor_specific_power_w_kg: self.power_processor_specific_power_w_kg,
            structure_density_kg_m3: self.structure_density_kg_m3,
            structure_thickness_m: self.structure_thickness_m,
            radiator_area_m2: self.radiator_area_m2,
            radiator_temperature_k: self.radiator_temperature_k,
            radiator_emissivity: self.radiator_emissivity,
            radiator_areal_density_kg_m2: self.radiator_areal_density_kg_m2,
            ionization_efficiency: self.ionization_efficiency,
            inlet_temperature_k: self.inlet_temperature_k,
        }
        .compile()?;
        Ok(ElectricThrusterMount {
            name: self.name,
            engine,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }
}

/// Continuous magnetic-nozzle fusion torch (`[[fusion_torches]]`).
#[derive(Debug, Deserialize)]
pub(crate) struct FusionTorchAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    reaction: FusionReaction,
    working_fluid: ElectricPropellant,
    maximum_fusion_power_w: f64,
    fusion_gain: f64,
    maximum_working_flow_kg_s: f64,
    reactor_specific_power_w_kg: f64,
    plasma_coupling_efficiency: f64,
    magnetic_nozzle_efficiency: f64,
    nozzle_radius_m: f64,
    nozzle_length_m: f64,
    magnetic_field_t: f64,
    coil_current_density_a_m2: f64,
    structure_density_kg_m3: f64,
    structure_thickness_m: f64,
    radiator_area_m2: f64,
    radiator_temperature_k: f64,
    radiator_emissivity: f64,
    radiator_areal_density_kg_m2: f64,
}

impl FusionTorchAsset {
    pub(crate) fn bake(self) -> Result<FusionTorchMount, Box<dyn Error>> {
        let engine = FusionTorchSpec {
            name: self.name.clone(),
            reaction: self.reaction,
            working_fluid: self.working_fluid,
            maximum_fusion_power_w: self.maximum_fusion_power_w,
            fusion_gain: self.fusion_gain,
            maximum_working_flow_kg_s: self.maximum_working_flow_kg_s,
            reactor_specific_power_w_kg: self.reactor_specific_power_w_kg,
            plasma_coupling_efficiency: self.plasma_coupling_efficiency,
            magnetic_nozzle_efficiency: self.magnetic_nozzle_efficiency,
            nozzle_radius_m: self.nozzle_radius_m,
            nozzle_length_m: self.nozzle_length_m,
            magnetic_field_t: self.magnetic_field_t,
            coil_current_density_a_m2: self.coil_current_density_a_m2,
            structure_density_kg_m3: self.structure_density_kg_m3,
            structure_thickness_m: self.structure_thickness_m,
            radiator_area_m2: self.radiator_area_m2,
            radiator_temperature_k: self.radiator_temperature_k,
            radiator_emissivity: self.radiator_emissivity,
            radiator_areal_density_kg_m2: self.radiator_areal_density_kg_m2,
        }
        .compile()?;
        Ok(FusionTorchMount {
            name: self.name,
            engine,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }
}

/// Discrete fusion pellet drive with pulse-energy storage
/// (`[[pulsed_fusion_systems]]`).
#[derive(Debug, Deserialize)]
pub(crate) struct PulsedFusionAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    reaction: FusionReaction,
    working_fluid: ElectricPropellant,
    fuel_mass_per_pulse_kg: f64,
    working_fluid_mass_per_pulse_kg: f64,
    fusion_gain: f64,
    plasma_coupling_efficiency: f64,
    magnetic_nozzle_efficiency: f64,
    maximum_pulse_frequency_hz: f64,
    pulse_duration_s: f64,
    maximum_charge_power_w: f64,
    energy_buffer_capacity_pulses: u8,
    energy_buffer_specific_energy_j_kg: f64,
    pulse_system_specific_power_w_kg: f64,
    chamber_radius_m: f64,
    chamber_length_m: f64,
    magnetic_field_t: f64,
    coil_current_density_a_m2: f64,
    structure_density_kg_m3: f64,
    structure_thickness_m: f64,
    radiator_area_m2: f64,
    radiator_temperature_k: f64,
    radiator_emissivity: f64,
    radiator_areal_density_kg_m2: f64,
}

impl PulsedFusionAsset {
    pub(crate) fn bake(self) -> Result<PulsedFusionMount, Box<dyn Error>> {
        let engine = PulsedFusionSpec {
            name: self.name.clone(),
            reaction: self.reaction,
            working_fluid: self.working_fluid,
            fuel_mass_per_pulse_kg: self.fuel_mass_per_pulse_kg,
            working_fluid_mass_per_pulse_kg: self.working_fluid_mass_per_pulse_kg,
            fusion_gain: self.fusion_gain,
            plasma_coupling_efficiency: self.plasma_coupling_efficiency,
            magnetic_nozzle_efficiency: self.magnetic_nozzle_efficiency,
            maximum_pulse_frequency_hz: self.maximum_pulse_frequency_hz,
            pulse_duration_s: self.pulse_duration_s,
            maximum_charge_power_w: self.maximum_charge_power_w,
            energy_buffer_capacity_pulses: self.energy_buffer_capacity_pulses,
            energy_buffer_specific_energy_j_kg: self.energy_buffer_specific_energy_j_kg,
            pulse_system_specific_power_w_kg: self.pulse_system_specific_power_w_kg,
            chamber_radius_m: self.chamber_radius_m,
            chamber_length_m: self.chamber_length_m,
            magnetic_field_t: self.magnetic_field_t,
            coil_current_density_a_m2: self.coil_current_density_a_m2,
            structure_density_kg_m3: self.structure_density_kg_m3,
            structure_thickness_m: self.structure_thickness_m,
            radiator_area_m2: self.radiator_area_m2,
            radiator_temperature_k: self.radiator_temperature_k,
            radiator_emissivity: self.radiator_emissivity,
            radiator_areal_density_kg_m2: self.radiator_areal_density_kg_m2,
        }
        .compile()?;
        Ok(PulsedFusionMount {
            name: self.name,
            engine,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }
}

/// One piston/electric source and ideal actuator-disk propulsor.
///
/// Example TOML:
///
/// ```text
/// [[propeller_drives]]
/// name = "electric-cruise"
/// mount_position_body_m = [-1.2, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// reduction_ratio = 2.0
///
/// [propeller_drives.propeller]
/// blade_count = 4
/// diameter_m = 2.0
/// hub_diameter_m = 0.25
/// blade_chord_m = 0.12
/// blade_thickness_m = 0.018
/// blade_material_density_kg_m3 = 1600.0
/// gearbox_efficiency = 0.97
/// gearbox_mass_kg = 12.0
///
/// [propeller_drives.source]
/// kind = "electric"
///
/// [propeller_drives.source.spec]
/// rated_power_w = 100000.0
/// peak_torque_nm = 400.0
/// maximum_rpm = 12000.0
/// efficiency = 0.94
/// cooling_capacity_w = 6400.0
/// dry_mass_kg = 35.0
/// ```
#[derive(Debug, Deserialize)]
pub(crate) struct PropellerDriveAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    #[serde(default)]
    propeller: PropellerSpec,
    source: ShaftPowerSourceSpec,
    #[serde(default = "one")]
    reduction_ratio: f64,
}

impl PropellerDriveAsset {
    pub(crate) fn bake(self) -> Result<PropellerDriveMount, Box<dyn Error>> {
        let drive = PropellerDriveSpec {
            propeller: self.propeller,
            source: self.source,
            reduction_ratio: self.reduction_ratio,
        }
        .compile()?;
        println!(
            "propeller drive {}: {:.2} m ideal disk, {:.1} kg installed",
            self.name, drive.propeller.diameter_m, drive.dry_mass_kg,
        );
        Ok(PropellerDriveMount {
            name: self.name,
            drive,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }
}

/// Gas-turbine propeller mount with an energy-accounted power turbine.
/// The nested `drive.air.shaft.power_turbine_heat_fraction` reserves the
/// mechanical takeoff share from combustor heat.
#[derive(Debug, Deserialize)]
pub(crate) struct TurbopropAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    drive: TurbopropDriveSpec,
}

impl TurbopropAsset {
    pub(crate) fn bake(self) -> Result<TurbopropMount, Box<dyn Error>> {
        let drive = self.drive.compile()?;
        println!(
            "turboprop {}: {:.2} m ideal disk, {:.1}% PTO heat, {:.1} kg installed",
            self.name,
            drive.propeller.diameter_m,
            drive.air.shaft.power_turbine_heat_fraction * 100.0,
            drive.dry_mass_kg,
        );
        Ok(TurbopropMount {
            name: self.name,
            drive,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }
}
