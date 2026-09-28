//! Vehicle-TOML authoring for the ideal shared electrical power bus.

use super::*;

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
pub(crate) struct ElectricalPowerAsset {
    batteries: Vec<BatteryAsset>,
    #[serde(default)]
    ultracapacitors: Vec<UltracapacitorAsset>,
    solar_arrays: Vec<SolarArrayAsset>,
    reactors: Vec<ReactorAsset>,
    consumers: Vec<PowerConsumerAsset>,
}

impl ElectricalPowerAsset {
    pub(super) fn bake(self) -> ElectricalPowerSystem {
        ElectricalPowerSystem {
            batteries: self.batteries.into_iter().map(BatteryAsset::bake).collect(),
            ultracapacitors: self
                .ultracapacitors
                .into_iter()
                .map(UltracapacitorAsset::bake)
                .collect(),
            solar_arrays: self
                .solar_arrays
                .into_iter()
                .map(SolarArrayAsset::bake)
                .collect(),
            reactors: self.reactors.into_iter().map(ReactorAsset::bake).collect(),
            consumers: self
                .consumers
                .into_iter()
                .map(PowerConsumerAsset::bake)
                .collect(),
        }
    }
}

#[derive(Debug, Deserialize)]
struct BatteryAsset {
    name: String,
    capacity_j: f64,
    initial_charge_fraction: f64,
    maximum_charge_power_w: f64,
    maximum_discharge_power_w: f64,
    charge_efficiency: f64,
    discharge_efficiency: f64,
    specific_energy_j_kg: f64,
    dimensions_body_m: [f64; 3],
    position_body_m: [f64; 3],
}

impl BatteryAsset {
    fn bake(self) -> BatterySpec {
        BatterySpec {
            name: self.name,
            capacity_j: self.capacity_j,
            initial_charge_fraction: self.initial_charge_fraction,
            maximum_charge_power_w: self.maximum_charge_power_w,
            maximum_discharge_power_w: self.maximum_discharge_power_w,
            charge_efficiency: self.charge_efficiency,
            discharge_efficiency: self.discharge_efficiency,
            specific_energy_j_kg: self.specific_energy_j_kg,
            dimensions_body_m: vector(self.dimensions_body_m),
            position_body_m: vector(self.position_body_m),
        }
    }
}

#[derive(Debug, Deserialize)]
struct UltracapacitorAsset {
    name: String,
    capacity_j: f64,
    initial_charge_fraction: f64,
    maximum_charge_power_w: f64,
    maximum_discharge_power_w: f64,
    charge_efficiency: f64,
    discharge_efficiency: f64,
    specific_energy_j_kg: f64,
    dimensions_body_m: [f64; 3],
    position_body_m: [f64; 3],
}

impl UltracapacitorAsset {
    fn bake(self) -> UltracapacitorSpec {
        UltracapacitorSpec {
            name: self.name,
            capacity_j: self.capacity_j,
            initial_charge_fraction: self.initial_charge_fraction,
            maximum_charge_power_w: self.maximum_charge_power_w,
            maximum_discharge_power_w: self.maximum_discharge_power_w,
            charge_efficiency: self.charge_efficiency,
            discharge_efficiency: self.discharge_efficiency,
            specific_energy_j_kg: self.specific_energy_j_kg,
            dimensions_body_m: vector(self.dimensions_body_m),
            position_body_m: vector(self.position_body_m),
        }
    }
}

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum SolarArrayDeploymentAsset {
    #[default]
    Fixed,
    Foldable,
}

#[derive(Debug, Deserialize)]
struct SolarArrayAsset {
    name: String,
    cell_count_x: u32,
    cell_count_y: u32,
    cell_size_x_m: f64,
    cell_size_y_m: f64,
    cell_efficiency: f64,
    cell_areal_density_kg_m2: f64,
    support_areal_density_kg_m2: f64,
    panel_u_axis_body: [f64; 3],
    panel_v_axis_body: [f64; 3],
    position_body_m: [f64; 3],
    #[serde(default)]
    deployment: SolarArrayDeploymentAsset,
    #[serde(default)]
    deployment_rate_per_s: f64,
    #[serde(default)]
    actuator_power_w: f64,
    #[serde(default)]
    initial_deployment_fraction: f64,
    /// Optional single-axis sun-tracking drive. Absent means fixed.
    #[serde(default)]
    tracking: Option<SolarTrackingAsset>,
}

#[derive(Debug, Deserialize)]
struct SolarTrackingAsset {
    rotation_axis_body: [f64; 3],
    minimum_angle_rad: f64,
    maximum_angle_rad: f64,
    slew_rate_rad_s: f64,
    actuator_power_w: f64,
    initial_angle_rad: f64,
}

impl SolarArrayAsset {
    fn bake(self) -> SolarArraySpec {
        let deployment = match self.deployment {
            SolarArrayDeploymentAsset::Fixed => SolarArrayDeployment::Fixed,
            SolarArrayDeploymentAsset::Foldable => SolarArrayDeployment::Foldable {
                deployment_rate_per_s: self.deployment_rate_per_s,
                actuator_power_w: self.actuator_power_w,
                initial_fraction: self.initial_deployment_fraction,
            },
        };
        let tracking = self
            .tracking
            .map(|drive| SolarArrayTracking::SingleAxis {
                rotation_axis_body: vector(drive.rotation_axis_body),
                minimum_angle_rad: drive.minimum_angle_rad,
                maximum_angle_rad: drive.maximum_angle_rad,
                slew_rate_rad_s: drive.slew_rate_rad_s,
                actuator_power_w: drive.actuator_power_w,
                initial_angle_rad: drive.initial_angle_rad,
            })
            .unwrap_or(SolarArrayTracking::Fixed);
        SolarArraySpec {
            name: self.name,
            cell_count_x: self.cell_count_x,
            cell_count_y: self.cell_count_y,
            cell_size_x_m: self.cell_size_x_m,
            cell_size_y_m: self.cell_size_y_m,
            cell_efficiency: self.cell_efficiency,
            cell_areal_density_kg_m2: self.cell_areal_density_kg_m2,
            support_areal_density_kg_m2: self.support_areal_density_kg_m2,
            panel_u_axis_body: vector(self.panel_u_axis_body),
            panel_v_axis_body: vector(self.panel_v_axis_body),
            position_body_m: vector(self.position_body_m),
            deployment,
            tracking,
        }
    }
}

#[derive(Debug, Deserialize)]
struct ReactorAsset {
    name: String,
    rated_thermal_power_w: f64,
    electric_efficiency: f64,
    radiator_capacity_w: f64,
    initial_fuel_mass_kg: f64,
    fuel_specific_energy_j_kg: f64,
    dry_mass_kg: f64,
    dimensions_body_m: [f64; 3],
    position_body_m: [f64; 3],
}

impl ReactorAsset {
    fn bake(self) -> ReactorSpec {
        ReactorSpec {
            name: self.name,
            rated_thermal_power_w: self.rated_thermal_power_w,
            electric_efficiency: self.electric_efficiency,
            radiator_capacity_w: self.radiator_capacity_w,
            initial_fuel_mass_kg: self.initial_fuel_mass_kg,
            fuel_specific_energy_j_kg: self.fuel_specific_energy_j_kg,
            dry_mass_kg: self.dry_mass_kg,
            dimensions_body_m: vector(self.dimensions_body_m),
            position_body_m: vector(self.position_body_m),
        }
    }
}

#[derive(Debug, Deserialize)]
struct PowerConsumerAsset {
    name: String,
    rated_power_w: f64,
    priority: PowerPriority,
}

impl PowerConsumerAsset {
    fn bake(self) -> PowerConsumerSpec {
        PowerConsumerSpec {
            name: self.name,
            rated_power_w: self.rated_power_w,
            priority: self.priority,
        }
    }
}
