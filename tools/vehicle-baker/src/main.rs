use std::{env, error::Error, fs, path::PathBuf};

use glam::{DMat3, DVec3};
use serde::Deserialize;
use thessa_sim_core::{
    AeroGeometry, AeroPanel, ControlSurfaceDefinition, RigidBodyProperties, VehicleDefinition,
};

fn main() -> Result<(), Box<dyn Error>> {
    let options = Options::parse(env::args().skip(1))?;
    if options.help {
        print_help();
        return Ok(());
    }
    let source = fs::read_to_string(&options.input)?;
    let asset: VehicleAsset = toml::from_str(&source)?;
    let vehicle = asset.bake()?;
    println!("vehicle: {}", vehicle.name);
    println!("panels: {}", vehicle.aero_geometry.panels.len());
    println!("control surfaces: {}", vehicle.control_surfaces.len());
    println!("mass: {:.3} kg", vehicle.mass_properties.mass_kg);
    if let Some(output) = options.output {
        let json = serde_json::to_string_pretty(&vehicle)?;
        fs::write(&output, format!("{json}\n"))?;
        println!("wrote: {}", output.display());
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
struct VehicleAsset {
    name: String,
    mass_kg: f64,
    /// Matrix is written as rows in the TOML file for readability.
    inertia_body_kg_m2: [[f64; 3]; 3],
    panels: Vec<PanelAsset>,
    #[serde(default)]
    control_surfaces: Vec<ControlSurfaceAsset>,
}

impl VehicleAsset {
    fn bake(self) -> Result<VehicleDefinition, Box<dyn Error>> {
        let panels = self
            .panels
            .into_iter()
            .map(PanelAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let geometry = AeroGeometry::new(panels)?;
        let inertia = rows_to_matrix(self.inertia_body_kg_m2);
        let properties = RigidBodyProperties::new(self.mass_kg, inertia)?;
        let controls = self
            .control_surfaces
            .into_iter()
            .map(ControlSurfaceAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        Ok(VehicleDefinition::new(
            self.name, geometry, properties, controls,
        )?)
    }
}

#[derive(Debug, Deserialize)]
struct PanelAsset {
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
    #[serde(default)]
    thickness_to_chord_ratio: f64,
    #[serde(default)]
    control_deflection_rad: f64,
    #[serde(default = "one")]
    exposure: f64,
}

impl PanelAsset {
    fn bake(self) -> Result<AeroPanel, Box<dyn Error>> {
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
        panel = panel.with_thickness_ratio(self.thickness_to_chord_ratio)?;
        panel.control_deflection_rad = self.control_deflection_rad;
        panel.exposure = self.exposure;
        Ok(panel)
    }
}

#[derive(Debug, Deserialize)]
struct ControlSurfaceAsset {
    name: String,
    panel_indices: Vec<usize>,
    minimum_deflection_rad: f64,
    maximum_deflection_rad: f64,
}

impl ControlSurfaceAsset {
    fn bake(self) -> Result<ControlSurfaceDefinition, Box<dyn Error>> {
        Ok(ControlSurfaceDefinition::new(
            self.name,
            self.panel_indices,
            self.minimum_deflection_rad,
            self.maximum_deflection_rad,
        )?)
    }
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

fn one() -> f64 {
    1.0
}

struct Options {
    input: PathBuf,
    output: Option<PathBuf>,
    help: bool,
}

impl Options {
    fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, Box<dyn Error>> {
        let mut input = PathBuf::from("data/vehicles/example_aircraft.toml");
        let mut output = None;
        let mut help = false;
        let mut arguments = arguments.peekable();
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--input" => input = PathBuf::from(required_value(&mut arguments, "--input")?),
                "--output" => {
                    output = Some(PathBuf::from(required_value(&mut arguments, "--output")?))
                }
                "--help" | "-h" => help = true,
                unknown => return Err(format!("unknown argument {unknown}; use --help").into()),
            }
        }
        Ok(Self {
            input,
            output,
            help,
        })
    }
}

fn required_value(
    arguments: &mut impl Iterator<Item = String>,
    option: &str,
) -> Result<String, Box<dyn Error>> {
    arguments
        .next()
        .ok_or_else(|| format!("{option} requires a value").into())
}

fn print_help() {
    println!(
        "Usage: thessa-vehicle-baker [--input data/vehicles/example_aircraft.toml] [--output data/vehicles/example_aircraft.baked.json]"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn example_vehicle_asset_bakes_to_valid_generic_definition() {
        let asset: VehicleAsset =
            toml::from_str(include_str!("../../../data/vehicles/example_aircraft.toml"))
                .expect("vehicle TOML should parse");
        let vehicle = asset.bake().expect("vehicle asset should bake");
        assert_eq!(vehicle.aero_geometry.panels.len(), 4);
        assert_eq!(vehicle.control_surfaces.len(), 2);
        assert_eq!(vehicle.mass_properties.mass_kg, 1_000.0);
        let json = serde_json::to_string(&vehicle).expect("vehicle JSON should serialize");
        let round_trip: VehicleDefinition =
            serde_json::from_str(&json).expect("vehicle JSON should deserialize");
        assert_eq!(round_trip, vehicle);
    }
}
