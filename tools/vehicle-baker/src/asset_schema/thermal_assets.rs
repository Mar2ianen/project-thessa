//! Vehicle-TOML authoring for the lumped thermal-node network.

use super::*;

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
pub(crate) struct ThermalAsset {
    nodes: Vec<ThermalNodeAsset>,
    links: Vec<ThermalLinkAsset>,
    radiators: Vec<RadiatorAsset>,
    heat_sources: Vec<ThermalHeatSourceAsset>,
    convective_k: Option<f64>,
}

impl ThermalAsset {
    pub(super) fn bake(self) -> ThermalSystem {
        ThermalSystem {
            nodes: self.nodes.into_iter().map(ThermalNodeAsset::bake).collect(),
            links: self.links.into_iter().map(ThermalLinkAsset::bake).collect(),
            radiators: self
                .radiators
                .into_iter()
                .map(RadiatorAsset::bake)
                .collect(),
            heat_sources: self
                .heat_sources
                .into_iter()
                .map(ThermalHeatSourceAsset::bake)
                .collect(),
            convective_k: self.convective_k.unwrap_or_else(default_convective_k),
        }
    }
}

#[derive(Debug, Deserialize)]
struct ThermalNodeAsset {
    name: String,
    mass_kg: f64,
    specific_heat_j_kg_k: f64,
    initial_temp_k: f64,
    max_temp_k: f64,
    emissivity: f64,
    solar_absorptivity: f64,
    radiating_area_m2: f64,
    solar_exposed_area_m2: f64,
    solar_normal_body: [f64; 3],
    aero_area_m2: f64,
    nose_radius_m: f64,
    position_body_m: [f64; 3],
}

impl ThermalNodeAsset {
    fn bake(self) -> ThermalNodeSpec {
        ThermalNodeSpec {
            name: self.name,
            mass_kg: self.mass_kg,
            specific_heat_j_kg_k: self.specific_heat_j_kg_k,
            initial_temp_k: self.initial_temp_k,
            max_temp_k: self.max_temp_k,
            emissivity: self.emissivity,
            solar_absorptivity: self.solar_absorptivity,
            radiating_area_m2: self.radiating_area_m2,
            solar_exposed_area_m2: self.solar_exposed_area_m2,
            solar_normal_body: vector(self.solar_normal_body),
            aero_area_m2: self.aero_area_m2,
            nose_radius_m: self.nose_radius_m,
            position_body_m: vector(self.position_body_m),
        }
    }
}

#[derive(Debug, Deserialize)]
struct ThermalLinkAsset {
    node_a: String,
    node_b: String,
    conductance_w_k: f64,
}

impl ThermalLinkAsset {
    fn bake(self) -> ThermalLinkSpec {
        ThermalLinkSpec {
            node_a: self.node_a,
            node_b: self.node_b,
            conductance_w_k: self.conductance_w_k,
        }
    }
}

#[derive(Debug, Default, Deserialize)]
#[serde(rename_all = "kebab-case")]
enum RadiatorDeploymentAsset {
    #[default]
    Fixed,
    Foldable,
}

#[derive(Debug, Deserialize)]
struct RadiatorAsset {
    name: String,
    attached_node: String,
    area_m2: f64,
    emissivity: f64,
    solar_absorptivity: f64,
    normal_body: [f64; 3],
    areal_density_kg_m2: f64,
    position_body_m: [f64; 3],
    #[serde(default)]
    deployment: RadiatorDeploymentAsset,
    #[serde(default)]
    deployment_rate_per_s: f64,
    #[serde(default)]
    actuator_power_w: f64,
    #[serde(default)]
    initial_deployment_fraction: f64,
    /// Optional pointing gimbal. Absent means a fixed normal.
    #[serde(default)]
    tracking: Option<RadiatorTrackingAsset>,
}

#[derive(Debug, Deserialize)]
struct RadiatorTrackingAsset {
    rotation_axis_body: [f64; 3],
    minimum_angle_rad: f64,
    maximum_angle_rad: f64,
    slew_rate_rad_s: f64,
    actuator_power_w: f64,
    initial_angle_rad: f64,
}

impl RadiatorAsset {
    fn bake(self) -> RadiatorSpec {
        let deployment = match self.deployment {
            RadiatorDeploymentAsset::Fixed => RadiatorDeployment::Fixed,
            RadiatorDeploymentAsset::Foldable => RadiatorDeployment::Foldable {
                deployment_rate_per_s: self.deployment_rate_per_s,
                actuator_power_w: self.actuator_power_w,
                initial_fraction: self.initial_deployment_fraction,
            },
        };
        let tracking = self
            .tracking
            .map(|drive| RadiatorTracking::SingleAxis {
                rotation_axis_body: vector(drive.rotation_axis_body),
                minimum_angle_rad: drive.minimum_angle_rad,
                maximum_angle_rad: drive.maximum_angle_rad,
                slew_rate_rad_s: drive.slew_rate_rad_s,
                actuator_power_w: drive.actuator_power_w,
                initial_angle_rad: drive.initial_angle_rad,
            })
            .unwrap_or(RadiatorTracking::Fixed);
        RadiatorSpec {
            name: self.name,
            attached_node: self.attached_node,
            area_m2: self.area_m2,
            emissivity: self.emissivity,
            solar_absorptivity: self.solar_absorptivity,
            normal_body: vector(self.normal_body),
            areal_density_kg_m2: self.areal_density_kg_m2,
            position_body_m: vector(self.position_body_m),
            deployment,
            tracking,
        }
    }
}

/// One authored waste-heat route from a named bus source to a thermal node.
#[derive(Debug, Deserialize)]
struct ThermalHeatSourceAsset {
    source_name: String,
    node: String,
    fraction: f64,
    /// `power` (reactor/fuel cell, default) or `generator` (APU/jet mount).
    #[serde(default)]
    kind: ThermalHeatSourceKind,
}

impl ThermalHeatSourceAsset {
    fn bake(self) -> ThermalHeatSource {
        ThermalHeatSource {
            source_name: self.source_name,
            node: self.node,
            fraction: self.fraction,
            kind: self.kind,
        }
    }
}
