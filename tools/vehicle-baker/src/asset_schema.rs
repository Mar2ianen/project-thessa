//! Vehicle TOML schema and hangar-side compilation into runtime definitions.

use super::*;

mod assembly;
mod power_assets;
mod propulsion_assets;
mod thermal_assets;
mod vehicle_asset;
#[cfg(test)]
pub(super) use assembly::AssemblyLinkAsset;
pub(super) use assembly::{AssemblyAsset, resolve_assembly_links, runtime_assembly};
pub(super) use power_assets::ElectricalPowerAsset;
pub(super) use propulsion_assets::*;
pub(super) use thermal_assets::ThermalAsset;
pub(super) use vehicle_asset::VehicleAsset;

#[derive(Debug, Deserialize)]
pub(super) struct ResourceFeedPortAsset {
    consumer_name: String,
    feed_port_name: String,
    #[serde(default)]
    fluid_properties: Vec<FeedResourceProperties>,
}

impl ResourceFeedPortAsset {
    pub(super) fn bake(self) -> VehicleResourceFeedPort {
        VehicleResourceFeedPort {
            consumer_name: self.consumer_name,
            feed_port_name: self.feed_port_name,
            fluid_properties: self.fluid_properties,
        }
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct ReactionWheelAsset {
    name: String,
    max_torque_body_nm: [f64; 3],
    mass_kg: f64,
    position_body_m: [f64; 3],
    /// Matrix is authored as rows for readability.
    inertia_body_kg_m2: [[f64; 3]; 3],
}

impl ReactionWheelAsset {
    pub(super) fn bake(self) -> ReactionWheelBankSpec {
        ReactionWheelBankSpec {
            name: self.name,
            max_torque_body_nm: vector(self.max_torque_body_nm),
            mass_kg: self.mass_kg,
            position_body_m: vector(self.position_body_m),
            inertia_body_kg_m2: rows_to_matrix(self.inertia_body_kg_m2),
        }
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct ParachuteAsset {
    name: String,
    reference_area_m2: f64,
    drag_coefficient: f64,
    reefed_area_fraction: f64,
    inflation_time_s: f64,
    deploy_pressure_pa: f64,
    max_deploy_dynamic_pressure_pa: f64,
    max_canopy_load_n: f64,
    pack_mass_kg: f64,
    position_body_m: [f64; 3],
    /// Matrix is authored as rows for readability.
    inertia_body_kg_m2: [[f64; 3]; 3],
}

impl ParachuteAsset {
    pub(super) fn bake(self) -> ParachuteSpec {
        ParachuteSpec {
            name: self.name,
            reference_area_m2: self.reference_area_m2,
            drag_coefficient: self.drag_coefficient,
            reefed_area_fraction: self.reefed_area_fraction,
            inflation_time_s: self.inflation_time_s,
            deploy_pressure_pa: self.deploy_pressure_pa,
            max_deploy_dynamic_pressure_pa: self.max_deploy_dynamic_pressure_pa,
            max_canopy_load_n: self.max_canopy_load_n,
            pack_mass_kg: self.pack_mass_kg,
            position_body_m: vector(self.position_body_m),
            inertia_body_kg_m2: rows_to_matrix(self.inertia_body_kg_m2),
        }
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct WheelChassisAsset {
    name: String,
    mount_position_body_m: [f64; 3],
    mount_orientation_body_xyzw: [f64; 4],
    length_m: f64,
    layout: WheelLayout,
    wheel_count: u16,
    structural_mass_kg: f64,
    /// Matrix is authored as rows for readability.
    structural_inertia_local_kg_m2: [[f64; 3]; 3],
    tire: WheelTireSpec,
    strut: WheelStrutSpec,
    brake: WheelBrakeSpec,
    #[serde(default)]
    drive: Option<WheelDriveSpec>,
    /// Optional aircraft-style fold hinge. The authored chassis pose is fully
    /// deployed; this record defines its stowed angle relative to that pose.
    #[serde(default)]
    retraction: Option<WheelChassisRetractionAsset>,
}

#[derive(Debug, Deserialize)]
pub(super) struct WheelChassisRetractionAsset {
    pivot_position_body_m: [f64; 3],
    hinge_axis_body: [f64; 3],
    stowed_angle_rad: f64,
    #[serde(default)]
    deployed_angle_rad: f64,
    #[serde(default = "default_true")]
    initially_deployed: bool,
    deployment_rate_rad_s: f64,
    actuator_max_torque_nm: f64,
}

impl From<WheelChassisRetractionAsset> for WheelChassisRetractionSpec {
    fn from(asset: WheelChassisRetractionAsset) -> Self {
        Self {
            pivot_position_body_m: vector(asset.pivot_position_body_m),
            hinge_axis_body: vector(asset.hinge_axis_body),
            stowed_angle_rad: asset.stowed_angle_rad,
            deployed_angle_rad: asset.deployed_angle_rad,
            initially_deployed: asset.initially_deployed,
            deployment_rate_rad_s: asset.deployment_rate_rad_s,
            actuator_max_torque_nm: asset.actuator_max_torque_nm,
        }
    }
}

impl WheelChassisAsset {
    pub(super) fn bake(self) -> Result<WheelChassisSpec, Box<dyn Error>> {
        let orientation = DQuat::from_xyzw(
            self.mount_orientation_body_xyzw[0],
            self.mount_orientation_body_xyzw[1],
            self.mount_orientation_body_xyzw[2],
            self.mount_orientation_body_xyzw[3],
        );
        Ok(WheelChassisSpec {
            name: self.name,
            mount_position_body_m: vector(self.mount_position_body_m),
            mount_orientation_body: orientation,
            length_m: self.length_m,
            layout: self.layout,
            wheel_count: self.wheel_count,
            structural_mass_kg: self.structural_mass_kg,
            structural_inertia_local_kg_m2: rows_to_matrix(self.structural_inertia_local_kg_m2),
            tire: self.tire,
            strut: self.strut,
            brake: self.brake,
            drive: self.drive,
            retraction: self.retraction.map(Into::into),
        })
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct LandingLegAsset {
    name: String,
    mount_position_body_m: [f64; 3],
    hinge_axis_body: [f64; 3],
    stowed_leg_axis_body: [f64; 3],
    stowed_angle_rad: f64,
    deployed_angle_rad: f64,
    #[serde(default = "default_true")]
    initially_deployed: bool,
    deployment_rate_rad_s: f64,
    actuator_max_torque_nm: f64,
    leg_length_m: f64,
    leg_mass_kg: f64,
    footpad_radius_m: f64,
    footpad_mass_kg: f64,
    footpad_friction: f64,
    footpad_slip_stiffness_n_per_mps: f64,
    shock_absorber: LandingShockAbsorberSpec,
}

impl LandingLegAsset {
    pub(super) fn bake(self) -> LandingLegSpec {
        LandingLegSpec {
            name: self.name,
            mount_position_body_m: vector(self.mount_position_body_m),
            hinge_axis_body: vector(self.hinge_axis_body),
            stowed_leg_axis_body: vector(self.stowed_leg_axis_body),
            stowed_angle_rad: self.stowed_angle_rad,
            deployed_angle_rad: self.deployed_angle_rad,
            initially_deployed: self.initially_deployed,
            deployment_rate_rad_s: self.deployment_rate_rad_s,
            actuator_max_torque_nm: self.actuator_max_torque_nm,
            leg_length_m: self.leg_length_m,
            leg_mass_kg: self.leg_mass_kg,
            footpad_radius_m: self.footpad_radius_m,
            footpad_mass_kg: self.footpad_mass_kg,
            footpad_friction: self.footpad_friction,
            footpad_slip_stiffness_n_per_mps: self.footpad_slip_stiffness_n_per_mps,
            shock_absorber: self.shock_absorber,
        }
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct PanelAsset {
    position_body_m: [f64; 3],
    chord_axis_body: [f64; 3],
    lift_axis_body: [f64; 3],
    area_m2: f64,
    chord_m: f64,
    #[serde(default)]
    center_of_pressure_body_m: Option<[f64; 3]>,
    #[serde(default)]
    span_m: Option<f64>,
    #[serde(default)]
    aspect_ratio: Option<f64>,
    #[serde(default)]
    sweep_rad: Option<f64>,
    #[serde(default)]
    lift_interference_factor: Option<f64>,
    #[serde(default = "one")]
    lift_coefficient_sign: f64,
    #[serde(default = "one")]
    side_force_scale: f64,
    #[serde(default)]
    thickness_to_chord_ratio: f64,
    #[serde(default)]
    control_deflection_rad: f64,
    #[serde(default = "one")]
    exposure: f64,
}

impl PanelAsset {
    pub(super) fn bake(self) -> Result<AeroPanel, Box<dyn Error>> {
        let mut panel = AeroPanel::new(
            vector(self.position_body_m),
            vector(self.chord_axis_body),
            vector(self.lift_axis_body),
            self.area_m2,
            self.chord_m,
        )?;
        if self.span_m.is_some()
            || self.aspect_ratio.is_some()
            || self.sweep_rad.is_some()
            || self.lift_interference_factor.is_some()
        {
            let span_m = self.span_m.unwrap_or(panel.span_m);
            let aspect_ratio = self.aspect_ratio.unwrap_or(span_m.powi(2) / self.area_m2);
            panel = panel.with_planform(
                span_m,
                aspect_ratio,
                self.sweep_rad.unwrap_or(0.0),
                self.lift_interference_factor.unwrap_or(1.0),
            )?;
        }
        if let Some(center_of_pressure) = self.center_of_pressure_body_m {
            panel = panel.with_center_of_pressure(vector(center_of_pressure))?;
        }
        panel = panel.with_lift_sign(self.lift_coefficient_sign)?;
        panel = panel.with_side_force_scale(self.side_force_scale)?;
        panel = panel.with_thickness_ratio(self.thickness_to_chord_ratio)?;
        panel.control_deflection_rad = self.control_deflection_rad;
        panel.exposure = self.exposure;
        Ok(panel)
    }
}

#[derive(Debug, Deserialize)]
pub(super) struct ControlSurfaceAsset {
    name: String,
    panel_indices: Vec<usize>,
    minimum_deflection_rad: f64,
    maximum_deflection_rad: f64,
    #[serde(default)]
    mixing: Option<ControlMixing>,
}

impl ControlSurfaceAsset {
    pub(super) fn bake(self) -> Result<ControlSurfaceDefinition, Box<dyn Error>> {
        let mut definition = ControlSurfaceDefinition::new(
            self.name,
            self.panel_indices,
            self.minimum_deflection_rad,
            self.maximum_deflection_rad,
        )?;
        if let Some(mixing) = self.mixing {
            definition = definition.with_mixing(mixing);
        }
        Ok(definition)
    }
}

fn mount_position_default() -> [f64; 3] {
    [0.0, 0.0, 0.0]
}

fn thrust_axis_default() -> [f64; 3] {
    [1.0, 0.0, 0.0]
}

fn default_true() -> bool {
    true
}

fn default_one_shot() -> u32 {
    1
}

fn vector(values: [f64; 3]) -> DVec3 {
    DVec3::from_array(values)
}

fn rows_to_matrix(rows: [[f64; 3]; 3]) -> DMat3 {
    DMat3::from_cols(
        vector([rows[0][0], rows[1][0], rows[2][0]]),
        vector([rows[0][1], rows[1][1], rows[2][1]]),
        vector([rows[0][2], rows[1][2], rows[2][2]]),
    )
}

/// Parallel-axis term `m(|c|^2 I - c c^T)` for recentering inertia.
pub(super) fn parallel_axis(mass_kg: f64, center: DVec3) -> DMat3 {
    let outer = DMat3::from_cols(center * center.x, center * center.y, center * center.z);
    (DMat3::from_diagonal(DVec3::splat(center.length_squared())) - outer) * mass_kg
}

/// Shift an array station by the recenter offset.
fn shift_array(station: &mut [f64; 3], shift: DVec3) {
    station[0] += shift.x;
    station[1] += shift.y;
    station[2] += shift.z;
}

fn one() -> f64 {
    1.0
}

fn identity_quaternion() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

fn default_friction() -> f64 {
    CollisionMaterial::default().friction
}
