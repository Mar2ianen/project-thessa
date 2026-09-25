//! Steady electric spacecraft thrusters (doc04 §13).
//!
//! The model couples requested electrical power and propellant flow to the
//! acceleration mechanism, then limits operation by current, rated flow, and
//! radiative heat rejection. It covers singly charged gridded-ion and Hall
//! accelerators, self-field MPD acceleration, and electrothermal resistojet /
//! arcjet nozzles. Plasma kinetics, erosion, magnetic-field topology, and
//! detailed electrode transport remain explicit fidelity limits.

use serde::{Deserialize, Serialize};

use super::{PropulsionError, STANDARD_GRAVITY_MPS2, require_positive, require_unit_interval};

const AVOGADRO_PER_MOL: f64 = 6.022_140_76e23;
const ELEMENTARY_CHARGE_C: f64 = 1.602_176_634e-19;
const BOLTZMANN_R_J_MOL_K: f64 = 8.314_462_618_153_24;
const STEFAN_BOLTZMANN_W_M2_K4: f64 = 5.670_374_419e-8;
const COSMIC_BACKGROUND_TEMP_K: f64 = 2.725;
const VACUUM_PERMEABILITY_H_M: f64 = 4.0e-7 * std::f64::consts::PI;
const COPPER_DENSITY_KG_M3: f64 = 8_960.0;

/// Working species with molecular properties used by the electric model.
/// Ionization energies are first-ionization reference values; neutral gas
/// heat capacity uses a constant-gamma engineering approximation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ElectricPropellant {
    Xenon,
    Krypton,
    Argon,
    Iodine,
    Nitrogen,
    Hydrogen,
    Ammonia,
    Water,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SpeciesProperties {
    molar_mass_kg_mol: f64,
    gamma: f64,
    first_ionization_ev: f64,
}

impl ElectricPropellant {
    fn properties(self) -> SpeciesProperties {
        match self {
            Self::Xenon => SpeciesProperties {
                molar_mass_kg_mol: 0.131_293,
                gamma: 5.0 / 3.0,
                first_ionization_ev: 12.129_8,
            },
            Self::Krypton => SpeciesProperties {
                molar_mass_kg_mol: 0.083_798,
                gamma: 5.0 / 3.0,
                first_ionization_ev: 13.999_6,
            },
            Self::Argon => SpeciesProperties {
                molar_mass_kg_mol: 0.039_948,
                gamma: 5.0 / 3.0,
                first_ionization_ev: 15.759_6,
            },
            Self::Iodine => SpeciesProperties {
                molar_mass_kg_mol: 0.126_904_47,
                gamma: 5.0 / 3.0,
                first_ionization_ev: 10.451_3,
            },
            Self::Nitrogen => SpeciesProperties {
                molar_mass_kg_mol: 0.028_013_4,
                gamma: 1.4,
                first_ionization_ev: 15.581,
            },
            Self::Hydrogen => SpeciesProperties {
                molar_mass_kg_mol: 0.002_015_88,
                gamma: 1.4,
                first_ionization_ev: 15.426,
            },
            Self::Ammonia => SpeciesProperties {
                molar_mass_kg_mol: 0.017_030_5,
                gamma: 1.31,
                first_ionization_ev: 10.18,
            },
            Self::Water => SpeciesProperties {
                molar_mass_kg_mol: 0.018_015_28,
                gamma: 1.33,
                first_ionization_ev: 12.621,
            },
        }
    }

    fn ionization_energy_j_kg(self) -> f64 {
        let properties = self.properties();
        properties.first_ionization_ev * ELEMENTARY_CHARGE_C * AVOGADRO_PER_MOL
            / properties.molar_mass_kg_mol
    }

    fn specific_heat_j_kg_k(self) -> f64 {
        let properties = self.properties();
        properties.gamma / (properties.gamma - 1.0) * BOLTZMANN_R_J_MOL_K
            / properties.molar_mass_kg_mol
    }
}

/// Acceleration hardware and limits for an electric thruster family.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum ElectricThrusterDesign {
    /// Perforated two-grid accelerator; singly charged ions leave at the
    /// energy set by `accelerator_voltage_v`.
    GriddedIon {
        accelerator_voltage_v: f64,
        grid_diameter_m: f64,
        grid_gap_m: f64,
        max_beam_current_density_a_m2: f64,
        propellant_utilization: f64,
        accelerator_efficiency: f64,
    },
    /// Annular crossed-field discharge with an electrostatic ion exit.
    HallEffect {
        accelerator_voltage_v: f64,
        channel_inner_radius_m: f64,
        channel_outer_radius_m: f64,
        channel_length_m: f64,
        magnetic_field_t: f64,
        coil_current_density_a_m2: f64,
        max_discharge_current_a: f64,
        propellant_utilization: f64,
        accelerator_efficiency: f64,
    },
    /// Self-field Lorentz acceleration. `ln(anode/cathode)` is the radial
    /// current-path term in the reduced Maecker thrust relation.
    Magnetoplasmadynamic {
        arc_voltage_v: f64,
        cathode_radius_m: f64,
        anode_radius_m: f64,
        electrode_length_m: f64,
        max_current_a: f64,
        jet_power_efficiency: f64,
    },
    /// Electrically heated gas expanded through an idealized nozzle.
    Resistojet {
        chamber_radius_m: f64,
        chamber_length_m: f64,
        max_exhaust_temp_k: f64,
        heater_efficiency: f64,
        nozzle_efficiency: f64,
    },
    /// Arc-heated electrothermal thruster; the arc-current rating is an
    /// electrical limit, while the same energy/nozzle balance sets thrust.
    Arcjet {
        chamber_radius_m: f64,
        chamber_length_m: f64,
        max_exhaust_temp_k: f64,
        arc_voltage_v: f64,
        max_arc_current_a: f64,
        heater_efficiency: f64,
        nozzle_efficiency: f64,
    },
}

/// Authoring data for an electrically powered spacecraft thruster.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ElectricThrusterSpec {
    pub name: String,
    pub propellant: ElectricPropellant,
    pub design: ElectricThrusterDesign,
    /// Continuous electrical bus power rating (W).
    pub maximum_power_w: f64,
    /// Continuous feed-system mass-flow rating (kg/s).
    pub maximum_mass_flow_kg_s: f64,
    /// Power-processing unit specific power (W/kg).
    pub power_processor_specific_power_w_kg: f64,
    /// Structure material density for grids, channel, electrodes, or chamber.
    pub structure_density_kg_m3: f64,
    /// Effective wall/plate thickness used by the geometry mass estimate (m).
    pub structure_thickness_m: f64,
    /// Radiator emitting area (m²), radiator temperature, emissivity, and
    /// areal mass. Both faces are not counted: area is emitting area.
    pub radiator_area_m2: f64,
    pub radiator_temperature_k: f64,
    pub radiator_emissivity: f64,
    pub radiator_areal_density_kg_m2: f64,
    /// Plasma electrical energy converted to first ionization energy.
    pub ionization_efficiency: f64,
    /// Neutral propellant inlet temperature for electrothermal designs (K).
    pub inlet_temperature_k: f64,
}

/// Hangar-compiled electric thruster with geometry-derived hardware mass.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CompiledElectricThruster {
    pub propellant: ElectricPropellant,
    pub design: ElectricThrusterDesign,
    pub maximum_power_w: f64,
    pub maximum_mass_flow_kg_s: f64,
    pub ionization_efficiency: f64,
    pub inlet_temperature_k: f64,
    pub radiator_heat_rejection_w: f64,
    pub dry_mass_kg: f64,
}

impl ElectricThrusterSpec {
    pub fn compile(self) -> Result<CompiledElectricThruster, PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "electric thruster name must not be empty".into(),
            ));
        }
        let mut authored_numbers = vec![
            self.maximum_power_w,
            self.maximum_mass_flow_kg_s,
            self.power_processor_specific_power_w_kg,
            self.structure_density_kg_m3,
            self.structure_thickness_m,
            self.radiator_area_m2,
            self.radiator_temperature_k,
            self.radiator_emissivity,
            self.radiator_areal_density_kg_m2,
            self.ionization_efficiency,
            self.inlet_temperature_k,
        ];
        match self.design {
            ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v,
                grid_diameter_m,
                grid_gap_m,
                max_beam_current_density_a_m2,
                propellant_utilization,
                accelerator_efficiency,
            } => authored_numbers.extend([
                accelerator_voltage_v,
                grid_diameter_m,
                grid_gap_m,
                max_beam_current_density_a_m2,
                propellant_utilization,
                accelerator_efficiency,
            ]),
            ElectricThrusterDesign::HallEffect {
                accelerator_voltage_v,
                channel_inner_radius_m,
                channel_outer_radius_m,
                channel_length_m,
                magnetic_field_t,
                coil_current_density_a_m2,
                max_discharge_current_a,
                propellant_utilization,
                accelerator_efficiency,
            } => authored_numbers.extend([
                accelerator_voltage_v,
                channel_inner_radius_m,
                channel_outer_radius_m,
                channel_length_m,
                magnetic_field_t,
                coil_current_density_a_m2,
                max_discharge_current_a,
                propellant_utilization,
                accelerator_efficiency,
            ]),
            ElectricThrusterDesign::Magnetoplasmadynamic {
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                electrode_length_m,
                max_current_a,
                jet_power_efficiency,
            } => authored_numbers.extend([
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                electrode_length_m,
                max_current_a,
                jet_power_efficiency,
            ]),
            ElectricThrusterDesign::Resistojet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
            } => authored_numbers.extend([
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
            ]),
            ElectricThrusterDesign::Arcjet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                arc_voltage_v,
                max_arc_current_a,
                heater_efficiency,
                nozzle_efficiency,
            } => authored_numbers.extend([
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                arc_voltage_v,
                max_arc_current_a,
                heater_efficiency,
                nozzle_efficiency,
            ]),
        }
        if authored_numbers.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "electric thruster authoring values must be finite".into(),
            ));
        }
        require_positive(self.maximum_power_w, "electric thruster rated power")?;
        require_positive(
            self.maximum_mass_flow_kg_s,
            "electric thruster rated mass flow",
        )?;
        require_positive(
            self.power_processor_specific_power_w_kg,
            "power processor specific power",
        )?;
        require_positive(self.structure_density_kg_m3, "thruster structure density")?;
        require_positive(self.structure_thickness_m, "thruster structure thickness")?;
        require_positive(self.radiator_area_m2, "electric thruster radiator area")?;
        require_positive(self.radiator_temperature_k, "radiator temperature")?;
        require_unit_interval(self.radiator_emissivity, "radiator emissivity")?;
        if self.radiator_emissivity == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "radiator emissivity must be in (0, 1]".into(),
            ));
        }
        require_positive(self.radiator_areal_density_kg_m2, "radiator areal density")?;
        require_unit_interval(self.ionization_efficiency, "ionization efficiency")?;
        if self.ionization_efficiency == 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "ionization efficiency must be in (0, 1]".into(),
            ));
        }
        require_positive(self.inlet_temperature_k, "propellant inlet temperature")?;

        let mut field_coil_volume_m3 = 0.0;
        let geometry_volume_m3 = match self.design {
            ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v,
                grid_diameter_m,
                grid_gap_m,
                max_beam_current_density_a_m2,
                propellant_utilization,
                accelerator_efficiency,
            } => {
                require_positive(accelerator_voltage_v, "ion accelerator voltage")?;
                require_positive(grid_diameter_m, "ion grid diameter")?;
                require_positive(grid_gap_m, "ion grid gap")?;
                require_positive(max_beam_current_density_a_m2, "ion beam current density")?;
                require_active_efficiency(propellant_utilization, "ion propellant utilization")?;
                require_active_efficiency(accelerator_efficiency, "ion accelerator efficiency")?;
                let area = std::f64::consts::PI * grid_diameter_m.powi(2) / 4.0;
                let rim_area = std::f64::consts::PI * grid_diameter_m * grid_gap_m;
                2.0 * area * self.structure_thickness_m + rim_area * self.structure_thickness_m
            }
            ElectricThrusterDesign::HallEffect {
                accelerator_voltage_v,
                channel_inner_radius_m,
                channel_outer_radius_m,
                channel_length_m,
                magnetic_field_t,
                coil_current_density_a_m2,
                max_discharge_current_a,
                propellant_utilization,
                accelerator_efficiency,
            } => {
                require_positive(accelerator_voltage_v, "Hall accelerator voltage")?;
                require_positive(channel_inner_radius_m, "Hall channel inner radius")?;
                require_positive(channel_outer_radius_m, "Hall channel outer radius")?;
                if channel_inner_radius_m >= channel_outer_radius_m {
                    return Err(PropulsionError::InvalidSpec(
                        "Hall channel outer radius must exceed inner radius".into(),
                    ));
                }
                require_positive(channel_length_m, "Hall channel length")?;
                require_positive(magnetic_field_t, "Hall magnetic field")?;
                require_positive(coil_current_density_a_m2, "Hall coil current density")?;
                require_positive(max_discharge_current_a, "Hall discharge current")?;
                require_active_efficiency(propellant_utilization, "Hall propellant utilization")?;
                require_active_efficiency(accelerator_efficiency, "Hall accelerator efficiency")?;
                let annular_area = std::f64::consts::PI
                    * (channel_outer_radius_m.powi(2) - channel_inner_radius_m.powi(2));
                let mean_radius_m = 0.5 * (channel_inner_radius_m + channel_outer_radius_m);
                field_coil_volume_m3 = magnetic_field_t * channel_length_m
                    / (VACUUM_PERMEABILITY_H_M * coil_current_density_a_m2)
                    * std::f64::consts::TAU
                    * mean_radius_m;
                2.0 * std::f64::consts::PI
                    * (channel_outer_radius_m + channel_inner_radius_m)
                    * channel_length_m
                    * self.structure_thickness_m
                    + 2.0 * annular_area * self.structure_thickness_m
            }
            ElectricThrusterDesign::Magnetoplasmadynamic {
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                electrode_length_m,
                max_current_a,
                jet_power_efficiency,
            } => {
                require_positive(arc_voltage_v, "MPD arc voltage")?;
                require_positive(cathode_radius_m, "MPD cathode radius")?;
                require_positive(anode_radius_m, "MPD anode radius")?;
                if cathode_radius_m >= anode_radius_m {
                    return Err(PropulsionError::InvalidSpec(
                        "MPD anode radius must exceed cathode radius".into(),
                    ));
                }
                require_positive(electrode_length_m, "MPD electrode length")?;
                require_positive(max_current_a, "MPD current rating")?;
                require_active_efficiency(jet_power_efficiency, "MPD jet-power efficiency")?;
                std::f64::consts::PI
                    * (cathode_radius_m.powi(2) + anode_radius_m.powi(2))
                    * electrode_length_m
            }
            ElectricThrusterDesign::Resistojet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
            } => {
                validate_thermal_design(
                    chamber_radius_m,
                    chamber_length_m,
                    max_exhaust_temp_k,
                    self.inlet_temperature_k,
                    heater_efficiency,
                    nozzle_efficiency,
                )?;
                thermal_chamber_volume(
                    chamber_radius_m,
                    chamber_length_m,
                    self.structure_thickness_m,
                )
            }
            ElectricThrusterDesign::Arcjet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                arc_voltage_v,
                max_arc_current_a,
                heater_efficiency,
                nozzle_efficiency,
            } => {
                validate_thermal_design(
                    chamber_radius_m,
                    chamber_length_m,
                    max_exhaust_temp_k,
                    self.inlet_temperature_k,
                    heater_efficiency,
                    nozzle_efficiency,
                )?;
                require_positive(arc_voltage_v, "arcjet voltage")?;
                require_positive(max_arc_current_a, "arcjet current rating")?;
                thermal_chamber_volume(
                    chamber_radius_m,
                    chamber_length_m,
                    self.structure_thickness_m,
                )
            }
        };
        let radiator_heat_rejection_w = self.radiator_emissivity
            * STEFAN_BOLTZMANN_W_M2_K4
            * self.radiator_area_m2
            * (self.radiator_temperature_k.powi(4) - COSMIC_BACKGROUND_TEMP_K.powi(4));
        let dry_mass_kg = self.maximum_power_w / self.power_processor_specific_power_w_kg
            + geometry_volume_m3 * self.structure_density_kg_m3
            + field_coil_volume_m3 * COPPER_DENSITY_KG_M3
            + self.radiator_area_m2 * self.radiator_areal_density_kg_m2;
        if !radiator_heat_rejection_w.is_finite()
            || radiator_heat_rejection_w <= 0.0
            || !dry_mass_kg.is_finite()
            || dry_mass_kg <= 0.0
        {
            return Err(PropulsionError::InvalidSpec(
                "electric thruster thermal or mass sizing overflowed".into(),
            ));
        }
        Ok(CompiledElectricThruster {
            propellant: self.propellant,
            design: self.design,
            maximum_power_w: self.maximum_power_w,
            maximum_mass_flow_kg_s: self.maximum_mass_flow_kg_s,
            ionization_efficiency: self.ionization_efficiency,
            inlet_temperature_k: self.inlet_temperature_k,
            radiator_heat_rejection_w,
            dry_mass_kg,
        })
    }
}

/// Unit-interval efficiency that must also be strictly positive. A zero
/// efficiency divides by zero in the operating-point energy balance and would
/// produce NaN telemetry instead of an `InvalidSpec` error.
fn require_active_efficiency(value: f64, name: &str) -> Result<f64, PropulsionError> {
    require_unit_interval(value, name)?;
    if value == 0.0 {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be in (0, 1]"
        )));
    }
    Ok(value)
}

fn validate_thermal_design(
    radius_m: f64,
    length_m: f64,
    max_temperature_k: f64,
    inlet_temperature_k: f64,
    heater_efficiency: f64,
    nozzle_efficiency: f64,
) -> Result<(), PropulsionError> {
    require_positive(radius_m, "electrothermal chamber radius")?;
    require_positive(length_m, "electrothermal chamber length")?;
    require_positive(max_temperature_k, "electrothermal exhaust temperature")?;
    if max_temperature_k <= inlet_temperature_k {
        return Err(PropulsionError::InvalidSpec(
            "electrothermal exhaust temperature must exceed propellant inlet temperature".into(),
        ));
    }
    require_unit_interval(heater_efficiency, "electric heater efficiency")?;
    require_unit_interval(nozzle_efficiency, "electrothermal nozzle efficiency")?;
    if heater_efficiency == 0.0 || nozzle_efficiency == 0.0 {
        return Err(PropulsionError::InvalidSpec(
            "heater and nozzle efficiencies must be in (0, 1]".into(),
        ));
    }
    Ok(())
}

fn thermal_chamber_volume(radius_m: f64, length_m: f64, thickness_m: f64) -> f64 {
    2.0 * std::f64::consts::PI * radius_m * length_m * thickness_m
        + std::f64::consts::PI * radius_m.powi(2) * thickness_m
}

/// One electric-thruster command: available bus power and requested propellant
/// flow. Ratings and thermal rejection may reduce either at runtime.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElectricThrusterCommand {
    pub available_power_w: f64,
    pub requested_mass_flow_kg_s: f64,
}

/// Operating telemetry for an electric thruster.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ElectricThrusterPoint {
    pub thrust_n: f64,
    pub exhaust_velocity_mps: f64,
    pub isp_s: f64,
    /// Actual propellant flow through the thruster (kg/s).
    pub mass_flow_kg_s: f64,
    pub electrical_power_w: f64,
    pub jet_kinetic_power_w: f64,
    /// Residual exhaust enthalpy and ionization potential carried away by
    /// the propellant rather than rejected by the engine radiator (W).
    pub exhaust_internal_power_w: f64,
    pub waste_heat_w: f64,
    pub radiator_capacity_w: f64,
    pub discharge_current_a: f64,
    pub power_limited: bool,
    pub flow_limited: bool,
    pub current_limited: bool,
    pub thermal_limited: bool,
}

impl CompiledElectricThruster {
    /// Validate compiled engine data (finite, positive ratings, active
    /// efficiencies, coherent design geometry). This is the same domain check
    /// `ElectricThrusterSpec::compile` applies, re-run at use time so a
    /// deserialized/edited engine fails closed instead of reaching the
    /// operating-point formulas with zero efficiencies or non-finite ratings.
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if [
            self.maximum_power_w,
            self.maximum_mass_flow_kg_s,
            self.ionization_efficiency,
            self.inlet_temperature_k,
            self.radiator_heat_rejection_w,
            self.dry_mass_kg,
        ]
        .iter()
        .any(|value| !value.is_finite())
        {
            return Err(PropulsionError::InvalidSpec(
                "compiled electric thruster values must be finite".into(),
            ));
        }
        require_positive(self.maximum_power_w, "electric thruster rated power")?;
        require_positive(
            self.maximum_mass_flow_kg_s,
            "electric thruster rated mass flow",
        )?;
        require_positive(self.inlet_temperature_k, "propellant inlet temperature")?;
        require_positive(
            self.radiator_heat_rejection_w,
            "electric thruster heat rejection",
        )?;
        require_positive(self.dry_mass_kg, "electric thruster dry mass")?;
        require_active_efficiency(self.ionization_efficiency, "ionization efficiency")?;
        match self.design {
            ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v,
                grid_diameter_m,
                grid_gap_m,
                max_beam_current_density_a_m2,
                propellant_utilization,
                accelerator_efficiency,
            } => {
                require_positive(accelerator_voltage_v, "ion accelerator voltage")?;
                require_positive(grid_diameter_m, "ion grid diameter")?;
                require_positive(grid_gap_m, "ion grid gap")?;
                require_positive(max_beam_current_density_a_m2, "ion beam current density")?;
                require_active_efficiency(propellant_utilization, "ion propellant utilization")?;
                require_active_efficiency(accelerator_efficiency, "ion accelerator efficiency")?;
            }
            ElectricThrusterDesign::HallEffect {
                accelerator_voltage_v,
                channel_inner_radius_m,
                channel_outer_radius_m,
                channel_length_m,
                magnetic_field_t,
                coil_current_density_a_m2,
                max_discharge_current_a,
                propellant_utilization,
                accelerator_efficiency,
            } => {
                require_positive(accelerator_voltage_v, "Hall accelerator voltage")?;
                require_positive(channel_inner_radius_m, "Hall channel inner radius")?;
                require_positive(channel_outer_radius_m, "Hall channel outer radius")?;
                if channel_inner_radius_m >= channel_outer_radius_m {
                    return Err(PropulsionError::InvalidSpec(
                        "Hall channel outer radius must exceed inner radius".into(),
                    ));
                }
                require_positive(channel_length_m, "Hall channel length")?;
                require_positive(magnetic_field_t, "Hall magnetic field")?;
                require_positive(coil_current_density_a_m2, "Hall coil current density")?;
                require_positive(max_discharge_current_a, "Hall discharge current")?;
                require_active_efficiency(propellant_utilization, "Hall propellant utilization")?;
                require_active_efficiency(accelerator_efficiency, "Hall accelerator efficiency")?;
            }
            ElectricThrusterDesign::Magnetoplasmadynamic {
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                electrode_length_m,
                max_current_a,
                jet_power_efficiency,
            } => {
                require_positive(arc_voltage_v, "MPD arc voltage")?;
                require_positive(cathode_radius_m, "MPD cathode radius")?;
                require_positive(anode_radius_m, "MPD anode radius")?;
                if cathode_radius_m >= anode_radius_m {
                    return Err(PropulsionError::InvalidSpec(
                        "MPD anode radius must exceed cathode radius".into(),
                    ));
                }
                require_positive(electrode_length_m, "MPD electrode length")?;
                require_positive(max_current_a, "MPD current rating")?;
                require_active_efficiency(jet_power_efficiency, "MPD jet-power efficiency")?;
            }
            ElectricThrusterDesign::Resistojet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
            } => validate_thermal_design(
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                self.inlet_temperature_k,
                heater_efficiency,
                nozzle_efficiency,
            )?,
            ElectricThrusterDesign::Arcjet {
                chamber_radius_m,
                chamber_length_m,
                max_exhaust_temp_k,
                arc_voltage_v,
                max_arc_current_a,
                heater_efficiency,
                nozzle_efficiency,
            } => {
                validate_thermal_design(
                    chamber_radius_m,
                    chamber_length_m,
                    max_exhaust_temp_k,
                    self.inlet_temperature_k,
                    heater_efficiency,
                    nozzle_efficiency,
                )?;
                require_positive(arc_voltage_v, "arcjet voltage")?;
                require_positive(max_arc_current_a, "arcjet current rating")?;
            }
        }
        Ok(())
    }

    /// Evaluate a steady operating point from bus power and feed command.
    pub fn operating_point(
        &self,
        command: ElectricThrusterCommand,
    ) -> Result<ElectricThrusterPoint, PropulsionError> {
        self.validate()?;
        if !command.available_power_w.is_finite() || command.available_power_w < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "available electric power must be finite and >= 0".into(),
            ));
        }
        if !command.requested_mass_flow_kg_s.is_finite() || command.requested_mass_flow_kg_s < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "requested propellant flow must be finite and >= 0".into(),
            ));
        }
        let available_power_w = command.available_power_w.min(self.maximum_power_w);
        let requested_flow = command
            .requested_mass_flow_kg_s
            .min(self.maximum_mass_flow_kg_s);
        let flow_limited = command.requested_mass_flow_kg_s > self.maximum_mass_flow_kg_s;
        let mut point = match self.design {
            ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v,
                grid_diameter_m,
                max_beam_current_density_a_m2,
                propellant_utilization,
                accelerator_efficiency,
                ..
            } => {
                let grid_area_m2 = std::f64::consts::PI * grid_diameter_m.powi(2) / 4.0;
                let current_limit_a = max_beam_current_density_a_m2 * grid_area_m2;
                self.ion_operating_point(
                    available_power_w,
                    requested_flow,
                    current_limit_a,
                    accelerator_voltage_v,
                    propellant_utilization,
                    accelerator_efficiency,
                    flow_limited,
                )
            }
            ElectricThrusterDesign::HallEffect {
                accelerator_voltage_v,
                max_discharge_current_a,
                propellant_utilization,
                accelerator_efficiency,
                ..
            } => self.ion_operating_point(
                available_power_w,
                requested_flow,
                max_discharge_current_a,
                accelerator_voltage_v,
                propellant_utilization,
                accelerator_efficiency,
                flow_limited,
            ),
            ElectricThrusterDesign::Magnetoplasmadynamic {
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                max_current_a,
                jet_power_efficiency,
                ..
            } => self.mpd_operating_point(
                available_power_w,
                requested_flow,
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                max_current_a,
                jet_power_efficiency,
                flow_limited,
            ),
            ElectricThrusterDesign::Resistojet {
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
                ..
            } => self.thermal_operating_point(
                available_power_w,
                requested_flow,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
                None,
                flow_limited,
            ),
            ElectricThrusterDesign::Arcjet {
                max_exhaust_temp_k,
                arc_voltage_v,
                max_arc_current_a,
                heater_efficiency,
                nozzle_efficiency,
                ..
            } => self.thermal_operating_point(
                available_power_w,
                requested_flow,
                max_exhaust_temp_k,
                heater_efficiency,
                nozzle_efficiency,
                Some((arc_voltage_v, max_arc_current_a)),
                flow_limited,
            ),
        }?;
        point.power_limited |= command.available_power_w > self.maximum_power_w;
        Ok(point)
    }

    #[allow(clippy::too_many_arguments)]
    fn ion_operating_point(
        &self,
        available_power_w: f64,
        requested_flow: f64,
        current_limit_a: f64,
        accelerator_voltage_v: f64,
        propellant_utilization: f64,
        accelerator_efficiency: f64,
        flow_limited: bool,
    ) -> Result<ElectricThrusterPoint, PropulsionError> {
        let species = self.propellant.properties();
        let particle_mass_kg = species.molar_mass_kg_mol / AVOGADRO_PER_MOL;
        let ion_velocity_mps =
            (2.0 * ELEMENTARY_CHARGE_C * accelerator_voltage_v / particle_mass_kg).sqrt();
        let ionization_specific_j_kg = self.propellant.ionization_energy_j_kg();
        let acceleration_specific_j_kg =
            ELEMENTARY_CHARGE_C * accelerator_voltage_v * AVOGADRO_PER_MOL
                / species.molar_mass_kg_mol;
        let input_energy_per_feed_kg_j = propellant_utilization
            * (ionization_specific_j_kg / self.ionization_efficiency
                + acceleration_specific_j_kg / accelerator_efficiency);
        let useful_kinetic_per_feed_kg_j = propellant_utilization * acceleration_specific_j_kg;
        let exhaust_internal_per_feed_kg_j = propellant_utilization * ionization_specific_j_kg;
        let heat_per_feed_kg_j = (input_energy_per_feed_kg_j
            - useful_kinetic_per_feed_kg_j
            - exhaust_internal_per_feed_kg_j)
            .max(0.0);
        let ion_current_per_feed_kg_s_a =
            propellant_utilization * ELEMENTARY_CHARGE_C * AVOGADRO_PER_MOL
                / species.molar_mass_kg_mol;
        let flow_by_power = if input_energy_per_feed_kg_j > 0.0 {
            available_power_w / input_energy_per_feed_kg_j
        } else {
            0.0
        };
        let flow_by_radiator = if heat_per_feed_kg_j > 0.0 {
            self.radiator_heat_rejection_w / heat_per_feed_kg_j
        } else {
            f64::INFINITY
        };
        let flow_by_current = if ion_current_per_feed_kg_s_a > 0.0 {
            current_limit_a / ion_current_per_feed_kg_s_a
        } else {
            0.0
        };
        let mass_flow_kg_s = requested_flow
            .min(flow_by_power)
            .min(flow_by_radiator)
            .min(flow_by_current);
        let electrical_power_w = mass_flow_kg_s * input_energy_per_feed_kg_j;
        let thrust_n = mass_flow_kg_s * propellant_utilization * ion_velocity_mps;
        let jet_kinetic_power_w = mass_flow_kg_s * useful_kinetic_per_feed_kg_j;
        let exhaust_internal_power_w = mass_flow_kg_s * exhaust_internal_per_feed_kg_j;
        let waste_heat_w =
            (electrical_power_w - jet_kinetic_power_w - exhaust_internal_power_w).max(0.0);
        let discharge_current_a = mass_flow_kg_s * ion_current_per_feed_kg_s_a;
        Ok(make_electric_point(
            thrust_n,
            mass_flow_kg_s,
            electrical_power_w,
            jet_kinetic_power_w,
            waste_heat_w,
            self.radiator_heat_rejection_w,
            discharge_current_a,
            flow_by_power < requested_flow
                && flow_by_power <= flow_by_current
                && flow_by_power <= flow_by_radiator,
            flow_limited,
            flow_by_current < requested_flow
                && flow_by_current <= flow_by_power
                && flow_by_current <= flow_by_radiator,
            flow_by_radiator < requested_flow
                && flow_by_radiator <= flow_by_power
                && flow_by_radiator <= flow_by_current,
            exhaust_internal_power_w,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn thermal_operating_point(
        &self,
        available_power_w: f64,
        requested_flow: f64,
        exhaust_temp_k: f64,
        heater_efficiency: f64,
        nozzle_efficiency: f64,
        arc: Option<(f64, f64)>,
        flow_limited: bool,
    ) -> Result<ElectricThrusterPoint, PropulsionError> {
        let cp_j_kg_k = self.propellant.specific_heat_j_kg_k();
        let specific_enthalpy_j_kg = cp_j_kg_k * (exhaust_temp_k - self.inlet_temperature_k);
        let exit_velocity_mps = (2.0 * nozzle_efficiency * specific_enthalpy_j_kg).sqrt();
        let electrical_energy_per_kg_j = specific_enthalpy_j_kg / heater_efficiency;
        let kinetic_energy_per_kg_j = 0.5 * exit_velocity_mps.powi(2);
        let heat_per_kg_j = (electrical_energy_per_kg_j - specific_enthalpy_j_kg).max(0.0);
        let flow_by_power = available_power_w / electrical_energy_per_kg_j;
        let flow_by_radiator = if heat_per_kg_j > 0.0 {
            self.radiator_heat_rejection_w / heat_per_kg_j
        } else {
            f64::INFINITY
        };
        let flow_by_current = arc.map_or(f64::INFINITY, |(voltage_v, max_current_a)| {
            max_current_a * voltage_v / electrical_energy_per_kg_j
        });
        let mass_flow_kg_s = requested_flow
            .min(flow_by_power)
            .min(flow_by_radiator)
            .min(flow_by_current);
        let electrical_power_w = mass_flow_kg_s * electrical_energy_per_kg_j;
        let jet_kinetic_power_w = mass_flow_kg_s * kinetic_energy_per_kg_j;
        let exhaust_internal_power_w =
            mass_flow_kg_s * (specific_enthalpy_j_kg - kinetic_energy_per_kg_j).max(0.0);
        let waste_heat_w =
            (electrical_power_w - jet_kinetic_power_w - exhaust_internal_power_w).max(0.0);
        let discharge_current_a = arc.map_or(0.0, |(voltage_v, _)| {
            if voltage_v > 0.0 {
                electrical_power_w / voltage_v
            } else {
                0.0
            }
        });
        let current_limited = flow_by_current < requested_flow
            && flow_by_current <= flow_by_power
            && flow_by_current <= flow_by_radiator;
        let thermal_limited = flow_by_radiator < requested_flow
            && flow_by_radiator <= flow_by_power
            && flow_by_radiator <= flow_by_current;
        Ok(make_electric_point(
            mass_flow_kg_s * exit_velocity_mps,
            mass_flow_kg_s,
            electrical_power_w,
            jet_kinetic_power_w,
            waste_heat_w,
            self.radiator_heat_rejection_w,
            discharge_current_a,
            flow_by_power < requested_flow
                && flow_by_power <= flow_by_current
                && flow_by_power <= flow_by_radiator,
            flow_limited,
            current_limited,
            thermal_limited,
            exhaust_internal_power_w,
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn mpd_operating_point(
        &self,
        available_power_w: f64,
        requested_flow: f64,
        arc_voltage_v: f64,
        cathode_radius_m: f64,
        anode_radius_m: f64,
        max_current_a: f64,
        jet_power_efficiency: f64,
        flow_limited: bool,
    ) -> Result<ElectricThrusterPoint, PropulsionError> {
        let maecker_coefficient_n_a2 = VACUUM_PERMEABILITY_H_M / (4.0 * std::f64::consts::PI)
            * (anode_radius_m / cathode_radius_m).ln();
        let ionization_specific_j_kg =
            self.propellant.ionization_energy_j_kg() / self.ionization_efficiency;
        // Charge carried per unit of ionized feed mass (A per kg/s). Mirrors
        // the ion-path term with full propellant utilization, which the MPD
        // model assumes (all feed mass enters the discharge).
        let ion_current_per_feed_kg_s_a =
            ELEMENTARY_CHARGE_C * AVOGADRO_PER_MOL / self.propellant.properties().molar_mass_kg_mol;
        let evaluate = |power_w: f64| {
            let flow_by_ionization = if ionization_specific_j_kg > 0.0 {
                power_w / ionization_specific_j_kg
            } else {
                0.0
            };
            let flow_by_current = if ion_current_per_feed_kg_s_a > 0.0 {
                max_current_a / ion_current_per_feed_kg_s_a
            } else {
                0.0
            };
            let mass_flow_kg_s = requested_flow.min(flow_by_ionization).min(flow_by_current);
            if mass_flow_kg_s <= 0.0 || power_w <= 0.0 {
                return make_electric_point(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    self.radiator_heat_rejection_w,
                    0.0,
                    false,
                    flow_limited,
                    false,
                    false,
                    0.0,
                );
            }
            let ionization_power_w = mass_flow_kg_s * ionization_specific_j_kg;
            // Fixed point on the discharge current: the jet budget is funded
            // from the actual arc draw V*I, never from the full bus, so
            // jet_kinetic_power <= jet_power_efficiency * (V*I - ionization)
            // holds by construction and no over-unity point can appear.
            let current_cap_a = (power_w / arc_voltage_v).min(max_current_a);
            let mut current_a = current_cap_a;
            for _ in 0..24 {
                let available_jet_power_w = (arc_voltage_v * current_a - ionization_power_w)
                    .max(0.0)
                    * jet_power_efficiency;
                let energy_limited_thrust_n = (2.0 * mass_flow_kg_s * available_jet_power_w).sqrt();
                let energy_limited_current_a =
                    (energy_limited_thrust_n / maecker_coefficient_n_a2).sqrt();
                current_a = current_cap_a.min(energy_limited_current_a);
            }
            if !current_a.is_finite() || current_a <= 0.0 {
                // Ionizing this feed would consume at least the whole budget
                // (V*I <= power_w <= ionization power): no self-consistent
                // drawing point exists, so report a finite idle instead of
                // consuming power for zero thrust.
                return make_electric_point(
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    self.radiator_heat_rejection_w,
                    0.0,
                    true,
                    flow_limited,
                    false,
                    false,
                    0.0,
                );
            }
            let electrical_power_w = (arc_voltage_v * current_a).min(power_w);
            let thrust_n = maecker_coefficient_n_a2 * current_a.powi(2);
            let jet_kinetic_power_w = thrust_n.powi(2) / (2.0 * mass_flow_kg_s);
            let exhaust_internal_power_w =
                mass_flow_kg_s * self.propellant.ionization_energy_j_kg();
            let waste_heat_w =
                (electrical_power_w - jet_kinetic_power_w - exhaust_internal_power_w).max(0.0);
            let current_from_bus_a = power_w / arc_voltage_v;
            let available_jet_power_w =
                (arc_voltage_v * current_a - ionization_power_w).max(0.0) * jet_power_efficiency;
            let energy_limited_current_a = ((2.0 * mass_flow_kg_s * available_jet_power_w).sqrt()
                / maecker_coefficient_n_a2)
                .sqrt();
            make_electric_point(
                thrust_n,
                mass_flow_kg_s,
                electrical_power_w,
                jet_kinetic_power_w,
                waste_heat_w,
                self.radiator_heat_rejection_w,
                current_a,
                (flow_by_ionization < requested_flow && flow_by_ionization <= flow_by_current)
                    || (current_from_bus_a <= max_current_a
                        && current_from_bus_a <= energy_limited_current_a),
                flow_limited,
                max_current_a <= current_from_bus_a && max_current_a <= energy_limited_current_a,
                false,
                exhaust_internal_power_w,
            )
        };
        let mut point = evaluate(available_power_w);
        let mut thermal_limited = false;
        if point.waste_heat_w > self.radiator_heat_rejection_w {
            let mut low = 0.0;
            let mut high = available_power_w;
            for _ in 0..56 {
                let middle = 0.5 * (low + high);
                if evaluate(middle).waste_heat_w <= self.radiator_heat_rejection_w {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            point = evaluate(low);
            thermal_limited = true;
        }
        point.thermal_limited = thermal_limited;
        Ok(point)
    }
}

#[allow(clippy::too_many_arguments)]
fn make_electric_point(
    thrust_n: f64,
    mass_flow_kg_s: f64,
    electrical_power_w: f64,
    jet_kinetic_power_w: f64,
    waste_heat_w: f64,
    radiator_capacity_w: f64,
    discharge_current_a: f64,
    power_limited: bool,
    flow_limited: bool,
    current_limited: bool,
    thermal_limited: bool,
    exhaust_internal_power_w: f64,
) -> ElectricThrusterPoint {
    let exhaust_velocity_mps = if mass_flow_kg_s > 0.0 {
        thrust_n / mass_flow_kg_s
    } else {
        0.0
    };
    ElectricThrusterPoint {
        thrust_n,
        exhaust_velocity_mps,
        isp_s: exhaust_velocity_mps / STANDARD_GRAVITY_MPS2,
        mass_flow_kg_s,
        electrical_power_w,
        jet_kinetic_power_w,
        exhaust_internal_power_w,
        waste_heat_w,
        radiator_capacity_w,
        discharge_current_a,
        power_limited,
        flow_limited,
        current_limited,
        thermal_limited,
    }
}

/// One installed electric spacecraft thruster.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ElectricThrusterMount {
    pub name: String,
    pub engine: CompiledElectricThruster,
    pub position_body_m: [f64; 3],
    pub thrust_axis_body: [f64; 3],
}

impl ElectricThrusterMount {
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "electric-thruster mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "electric-thruster position must be finite".into(),
            ));
        }
        if self.thrust_axis_body.iter().any(|value| !value.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "electric-thruster axis must be finite".into(),
            ));
        }
        let norm = self
            .thrust_axis_body
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "electric-thruster axis must be unit length".into(),
            ));
        }
        self.engine.validate()?;
        Ok(())
    }

    pub fn operating_point(
        &self,
        command: ElectricThrusterCommand,
    ) -> Result<ElectricThrusterPoint, PropulsionError> {
        self.validate()?;
        self.engine.operating_point(command)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_spec(design: ElectricThrusterDesign) -> ElectricThrusterSpec {
        ElectricThrusterSpec {
            name: "electric-test".into(),
            propellant: ElectricPropellant::Xenon,
            design,
            maximum_power_w: 10_000.0,
            maximum_mass_flow_kg_s: 1.0e-4,
            power_processor_specific_power_w_kg: 2_000.0,
            structure_density_kg_m3: 2_700.0,
            structure_thickness_m: 0.003,
            radiator_area_m2: 10.0,
            radiator_temperature_k: 700.0,
            radiator_emissivity: 0.9,
            radiator_areal_density_kg_m2: 8.0,
            ionization_efficiency: 0.75,
            inlet_temperature_k: 300.0,
        }
    }

    fn command(power_w: f64, flow_kg_s: f64) -> ElectricThrusterCommand {
        ElectricThrusterCommand {
            available_power_w: power_w,
            requested_mass_flow_kg_s: flow_kg_s,
        }
    }

    fn assert_energy_balance(point: ElectricThrusterPoint) {
        let accounted_w =
            point.jet_kinetic_power_w + point.exhaust_internal_power_w + point.waste_heat_w;
        assert!(
            (point.electrical_power_w - accounted_w).abs()
                <= 1e-10 * point.electrical_power_w.max(1.0),
            "input {} W != accounted {} W",
            point.electrical_power_w,
            accounted_w
        );
    }

    #[test]
    fn gridded_ion_velocity_matches_accelerator_energy_and_power_balance() {
        let engine = base_spec(ElectricThrusterDesign::GriddedIon {
            accelerator_voltage_v: 1_000.0,
            grid_diameter_m: 0.4,
            grid_gap_m: 0.002,
            max_beam_current_density_a_m2: 100.0,
            propellant_utilization: 0.95,
            accelerator_efficiency: 0.9,
        })
        .compile()
        .expect("ion thruster compiles");
        let point = engine
            .operating_point(command(5_000.0, 1.0e-5))
            .expect("ion point");
        let expected_velocity = 0.95
            * (2.0 * ELEMENTARY_CHARGE_C * 1_000.0
                / (engine.propellant.properties().molar_mass_kg_mol / AVOGADRO_PER_MOL))
                .sqrt();
        assert!((point.exhaust_velocity_mps - expected_velocity).abs() / expected_velocity < 1e-12);
        assert!(point.thrust_n > 0.0);
        assert!(point.jet_kinetic_power_w <= point.electrical_power_w);
        assert!(point.waste_heat_w <= point.radiator_capacity_w);
        assert_energy_balance(point);
        assert!(point.discharge_current_a > 0.0);
        assert!(point.power_limited);
        let momentum = point.mass_flow_kg_s * point.exhaust_velocity_mps;
        assert!((point.thrust_n - momentum).abs() < 1e-12);
    }

    #[test]
    fn hall_current_grid_and_radiator_limits_are_reported() {
        let engine = base_spec(ElectricThrusterDesign::HallEffect {
            accelerator_voltage_v: 300.0,
            channel_inner_radius_m: 0.025,
            channel_outer_radius_m: 0.05,
            channel_length_m: 0.04,
            magnetic_field_t: 0.02,
            coil_current_density_a_m2: 4.0e7,
            max_discharge_current_a: 0.2,
            propellant_utilization: 0.9,
            accelerator_efficiency: 0.8,
        })
        .compile()
        .expect("Hall thruster compiles");
        let point = engine
            .operating_point(command(10_000.0, 1.0e-4))
            .expect("Hall point");
        assert!(point.current_limited);
        assert!(!point.thermal_limited);
        assert!(point.discharge_current_a <= 0.2 + 1e-12);
        assert!(point.waste_heat_w <= point.radiator_capacity_w + 1e-8);
        assert_energy_balance(point);

        let mut thermally_limited_spec = base_spec(ElectricThrusterDesign::HallEffect {
            accelerator_voltage_v: 300.0,
            channel_inner_radius_m: 0.025,
            channel_outer_radius_m: 0.05,
            channel_length_m: 0.04,
            magnetic_field_t: 0.02,
            coil_current_density_a_m2: 4.0e7,
            max_discharge_current_a: 0.2,
            propellant_utilization: 0.9,
            accelerator_efficiency: 0.8,
        });
        thermally_limited_spec.radiator_area_m2 = 1.0e-6;
        let thermally_limited = thermally_limited_spec
            .compile()
            .expect("small radiator compiles");
        let point = thermally_limited
            .operating_point(command(10_000.0, 1.0e-4))
            .expect("thermally limited Hall point");
        assert!(point.thermal_limited);
        assert!(!point.current_limited);
        assert!(point.waste_heat_w <= point.radiator_capacity_w + 1e-8);
        assert_energy_balance(point);
    }

    #[test]
    fn mpd_thrust_follows_self_field_current_squared_relation() {
        let engine = base_spec(ElectricThrusterDesign::Magnetoplasmadynamic {
            arc_voltage_v: 100.0,
            cathode_radius_m: 0.01,
            anode_radius_m: 0.05,
            electrode_length_m: 0.1,
            max_current_a: 20.0,
            jet_power_efficiency: 0.55,
        })
        .compile()
        .expect("MPD compiles");
        let point = engine
            .operating_point(command(5_000.0, 1.0e-4))
            .expect("MPD point");
        let maecker = VACUUM_PERMEABILITY_H_M / (4.0 * std::f64::consts::PI) * (5.0_f64).ln();
        assert!((point.thrust_n - maecker * point.discharge_current_a.powi(2)).abs() < 1e-9);
        assert!(point.jet_kinetic_power_w <= point.electrical_power_w);
        assert!(point.waste_heat_w <= point.radiator_capacity_w + 1e-8);
        assert_energy_balance(point);
    }

    #[test]
    fn electrothermal_nozzle_uses_enthalpy_and_efficiency() {
        let engine = base_spec(ElectricThrusterDesign::Resistojet {
            chamber_radius_m: 0.02,
            chamber_length_m: 0.1,
            max_exhaust_temp_k: 1_000.0,
            heater_efficiency: 0.9,
            nozzle_efficiency: 0.8,
        })
        .compile()
        .expect("resistojet compiles");
        let point = engine
            .operating_point(command(5_000.0, 1.0e-4))
            .expect("resistojet point");
        let cp = engine.propellant.specific_heat_j_kg_k();
        let expected_velocity = (2.0 * 0.8 * cp * (1_000.0 - engine.inlet_temperature_k)).sqrt();
        assert!((point.exhaust_velocity_mps - expected_velocity).abs() / expected_velocity < 1e-12);
        assert!(point.thrust_n > 0.0);
        assert!(point.waste_heat_w <= point.radiator_capacity_w + 1e-8);
        assert_energy_balance(point);
    }

    #[test]
    fn arcjet_current_limit_and_invalid_commands_fail_closed() {
        let engine = base_spec(ElectricThrusterDesign::Arcjet {
            chamber_radius_m: 0.02,
            chamber_length_m: 0.1,
            max_exhaust_temp_k: 2_500.0,
            arc_voltage_v: 80.0,
            max_arc_current_a: 0.01,
            heater_efficiency: 0.8,
            nozzle_efficiency: 0.75,
        })
        .compile()
        .expect("arcjet compiles");
        let point = engine
            .operating_point(command(10_000.0, 1.0e-4))
            .expect("arcjet point");
        assert!(point.current_limited);
        assert!(point.discharge_current_a <= 0.01 + 1e-12);
        assert_energy_balance(point);
        assert!(engine.operating_point(command(f64::NAN, 1.0e-6)).is_err());
        assert!(engine.operating_point(command(1.0, -1.0)).is_err());
    }

    #[test]
    fn thermodynamic_species_and_radiator_sizing_are_finite() {
        for propellant in [
            ElectricPropellant::Xenon,
            ElectricPropellant::Krypton,
            ElectricPropellant::Argon,
            ElectricPropellant::Iodine,
            ElectricPropellant::Nitrogen,
            ElectricPropellant::Hydrogen,
            ElectricPropellant::Ammonia,
            ElectricPropellant::Water,
        ] {
            assert!(propellant.specific_heat_j_kg_k().is_finite());
            assert!(propellant.ionization_energy_j_kg().is_finite());
        }
        let mut invalid = base_spec(ElectricThrusterDesign::Resistojet {
            chamber_radius_m: 0.02,
            chamber_length_m: 0.1,
            max_exhaust_temp_k: 250.0,
            heater_efficiency: 0.9,
            nozzle_efficiency: 0.8,
        });
        assert!(invalid.clone().compile().is_err());
        invalid.radiator_temperature_k = 2.0;
        assert!(invalid.compile().is_err());
    }

    fn assert_all_finite(point: ElectricThrusterPoint) {
        assert!(point.thrust_n.is_finite());
        assert!(point.exhaust_velocity_mps.is_finite());
        assert!(point.isp_s.is_finite());
        assert!(point.mass_flow_kg_s.is_finite());
        assert!(point.electrical_power_w.is_finite());
        assert!(point.jet_kinetic_power_w.is_finite());
        assert!(point.exhaust_internal_power_w.is_finite());
        assert!(point.waste_heat_w.is_finite());
        assert!(point.radiator_capacity_w.is_finite());
        assert!(point.discharge_current_a.is_finite());
    }

    fn mpd_spec(jet_power_efficiency: f64) -> ElectricThrusterSpec {
        base_spec(ElectricThrusterDesign::Magnetoplasmadynamic {
            arc_voltage_v: 100.0,
            cathode_radius_m: 0.01,
            anode_radius_m: 0.05,
            electrode_length_m: 0.1,
            max_current_a: 20.0,
            jet_power_efficiency,
        })
    }

    #[test]
    fn zero_efficiencies_fail_closed_at_compile_and_validation() {
        let gridded = |propellant_utilization: f64, accelerator_efficiency: f64| {
            base_spec(ElectricThrusterDesign::GriddedIon {
                accelerator_voltage_v: 1_000.0,
                grid_diameter_m: 0.4,
                grid_gap_m: 0.002,
                max_beam_current_density_a_m2: 100.0,
                propellant_utilization,
                accelerator_efficiency,
            })
        };
        assert!(gridded(0.0, 0.9).compile().is_err());
        assert!(gridded(0.95, 0.0).compile().is_err());

        let hall = |propellant_utilization: f64, accelerator_efficiency: f64| {
            base_spec(ElectricThrusterDesign::HallEffect {
                accelerator_voltage_v: 300.0,
                channel_inner_radius_m: 0.025,
                channel_outer_radius_m: 0.05,
                channel_length_m: 0.04,
                magnetic_field_t: 0.02,
                coil_current_density_a_m2: 4.0e7,
                max_discharge_current_a: 0.2,
                propellant_utilization,
                accelerator_efficiency,
            })
        };
        assert!(hall(0.0, 0.8).compile().is_err());
        assert!(hall(0.9, 0.0).compile().is_err());

        assert!(mpd_spec(0.0).compile().is_err());
        assert!(mpd_spec(1.0).compile().is_ok());
    }

    #[test]
    fn forged_engine_data_is_rejected_before_the_formulas() {
        let engine = mpd_spec(0.55).compile().expect("MPD compiles");
        // Simulate hand-edited JSON: zero jet-power efficiency and a
        // non-finite rating must not reach the operating-point formulas.
        let ElectricThrusterDesign::Magnetoplasmadynamic {
            arc_voltage_v,
            cathode_radius_m,
            anode_radius_m,
            electrode_length_m,
            max_current_a,
            ..
        } = engine.design
        else {
            panic!("expected an MPD design");
        };
        let zero_efficiency = CompiledElectricThruster {
            design: ElectricThrusterDesign::Magnetoplasmadynamic {
                arc_voltage_v,
                cathode_radius_m,
                anode_radius_m,
                electrode_length_m,
                max_current_a,
                jet_power_efficiency: 0.0,
            },
            ..engine
        };
        assert!(zero_efficiency.validate().is_err());
        assert!(
            zero_efficiency
                .operating_point(command(1_000.0, 1.0e-4))
                .is_err()
        );
        let non_finite = CompiledElectricThruster {
            maximum_power_w: f64::NAN,
            ..engine
        };
        assert!(non_finite.validate().is_err());
        assert!(
            non_finite
                .operating_point(command(1_000.0, 1.0e-4))
                .is_err()
        );
    }

    #[test]
    fn mount_operating_point_validates_engine_and_mount_first() {
        let engine = base_spec(ElectricThrusterDesign::GriddedIon {
            accelerator_voltage_v: 1_000.0,
            grid_diameter_m: 0.4,
            grid_gap_m: 0.002,
            max_beam_current_density_a_m2: 100.0,
            propellant_utilization: 0.95,
            accelerator_efficiency: 0.9,
        })
        .compile()
        .expect("ion thruster compiles");
        let valid = ElectricThrusterMount {
            name: "aft-ion".into(),
            engine,
            position_body_m: [0.0, 1.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        assert!(valid.operating_point(command(5_000.0, 1.0e-5)).is_ok());
        // A forged non-unit axis previously reached the formulas unchecked.
        let bad_axis = ElectricThrusterMount {
            thrust_axis_body: [1.0, 1.0, 0.0],
            ..valid.clone()
        };
        assert!(bad_axis.operating_point(command(5_000.0, 1.0e-5)).is_err());
        // A forged engine (zero ionization efficiency) fails closed too.
        let bad_engine = ElectricThrusterMount {
            engine: CompiledElectricThruster {
                ionization_efficiency: 0.0,
                ..engine
            },
            ..valid
        };
        assert!(bad_engine.validate().is_err());
        let bad_point = bad_engine.operating_point(command(5_000.0, 1.0e-5));
        assert!(bad_point.is_err());
    }

    #[test]
    fn mpd_typical_point_draws_exactly_voltage_times_current() {
        let engine = mpd_spec(0.55).compile().expect("MPD compiles");
        // Requested flow exceeds both the rated flow and the ionization
        // budget P / E_ion; the point must stay self-consistent anyway.
        let point = engine
            .operating_point(command(10_000.0, 1.0e-3))
            .expect("MPD typical point");
        assert_all_finite(point);
        assert!(point.thrust_n > 0.0);
        assert!(point.discharge_current_a > 0.0);
        assert!(point.discharge_current_a <= 20.0 + 1e-9);
        let draw_w = 100.0 * point.discharge_current_a;
        assert!((point.electrical_power_w - draw_w).abs() <= 1e-9 * draw_w);
        assert!(point.electrical_power_w <= 10_000.0 + 1e-9);
        assert!(point.waste_heat_w >= 0.0);
        assert!(point.waste_heat_w <= point.radiator_capacity_w + 1e-8);
        assert!(point.flow_limited);
        assert!(point.current_limited);
        assert_energy_balance(point);

        // Energy-limited inequality: kinetic power never exceeds the jet
        // efficiency of the power left after ionization.
        let ionization_power_w = point.mass_flow_kg_s * engine.propellant.ionization_energy_j_kg()
            / engine.ionization_efficiency;
        assert!(
            point.jet_kinetic_power_w
                <= 0.55 * (point.electrical_power_w - ionization_power_w) + 1e-9,
            "jet power {} W exceeds the {} W jet budget",
            point.jet_kinetic_power_w,
            0.55 * (point.electrical_power_w - ionization_power_w)
        );

        // Bus-power-limited branch: current below the current rating.
        let low_power = engine
            .operating_point(command(1_000.0, 1.0e-4))
            .expect("MPD low-power point");
        assert_all_finite(low_power);
        assert!(low_power.thrust_n > 0.0);
        assert!(
            (low_power.electrical_power_w - 100.0 * low_power.discharge_current_a).abs() <= 1e-9
        );
        assert!(low_power.power_limited);
        assert!(!low_power.current_limited);
        assert!(low_power.waste_heat_w >= 0.0);
        assert_energy_balance(low_power);
    }

    #[test]
    fn mpd_starved_feed_reports_finite_idle_instead_of_nan() {
        let engine = mpd_spec(0.55).compile().expect("MPD compiles");
        // P / E_ion = 8.4e-6 kg/s < requested: ionization alone would eat
        // the whole budget, so there is no self-consistent drawing point.
        // The engine must report a finite idle, not a power-burning zero
        // thrust or NaN telemetry.
        let point = engine
            .operating_point(command(100.0, 1.0e-3))
            .expect("starved MPD point");
        assert_all_finite(point);
        assert_eq!(point.thrust_n, 0.0);
        assert_eq!(point.electrical_power_w, 0.0);
        assert_eq!(point.discharge_current_a, 0.0);
        assert!(point.power_limited);
        assert_energy_balance(point);

        // The same bus power with a moderate feed request still works.
        let fed = engine
            .operating_point(command(100.0, 1.0e-6))
            .expect("low-power MPD point");
        assert_all_finite(fed);
        assert!(fed.thrust_n > 0.0);
        assert!((fed.electrical_power_w - 100.0 * fed.discharge_current_a).abs() <= 1e-9);
        assert!(fed.waste_heat_w >= 0.0);
        assert_energy_balance(fed);
    }
}
