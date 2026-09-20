use std::{env, error::Error, fs, path::PathBuf};

use glam::{DMat3, DQuat, DVec3};
use serde::Deserialize;
use thessa_sim_core::{
    AeroGeometry, AeroPanel, ChamberMaterial, CollisionAxis, CollisionGeometry, CollisionMaterial,
    CollisionPart, CollisionShape, CompiledEngine, ControlSurfaceDefinition, CoolingMode,
    EngineCycle, EngineMount, LiquidEngineSpec, NozzleContour, Propellant, RigidBodyProperties,
    SolidMotorSpec, VehicleDefinition,
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
    println!(
        "collision parts: {}",
        vehicle.collision_geometry.parts.len()
    );
    println!("mass: {:.3} kg", vehicle.mass_properties.mass_kg);
    for mount in &vehicle.engines {
        let (thrust_vac_n, kind) = match &mount.engine {
            CompiledEngine::Liquid(engine) => (engine.thrust_vac_n, "liquid"),
            CompiledEngine::Solid(engine) => (
                engine
                    .burn_curve
                    .iter()
                    .map(|point| point.thrust_vac_n)
                    .fold(0.0_f64, f64::max),
                "solid",
            ),
        };
        println!(
            "engine {} ({kind}): vacuum thrust {:.1} kN, bake mass {:.1} kg",
            mount.name,
            thrust_vac_n / 1000.0,
            mount.engine.bake_mass_kg(),
        );
    }
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
    /// Solver-neutral contact primitives. Legacy assets may omit this while
    /// collision geometry is migrated; contact-active runtime code must not.
    #[serde(default)]
    collision_parts: Vec<CollisionPartAsset>,
    /// Procedural engine mounts (Juno-simple-mode authoring). Empty keeps
    /// engine-less assets valid; input mass semantics are "structure
    /// without engines" once mounts are present.
    #[serde(default)]
    engines: Vec<EngineAsset>,
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
        let collision_geometry = CollisionGeometry::new(
            self.collision_parts
                .into_iter()
                .map(CollisionPartAsset::bake)
                .collect::<Result<Vec<_>, _>>()?,
        )?;
        let mounts = self
            .engines
            .into_iter()
            .map(EngineAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mut vehicle = VehicleDefinition::new(self.name, geometry, properties, controls)?
            .with_collision_geometry(collision_geometry)?
            .with_engines(mounts)?;
        vehicle.bake_engine_masses()?;
        Ok(vehicle)
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

/// One procedural engine mount from the source vehicle asset.
///
/// Example TOML (liquid):
///
/// ```text
/// [[engines]]
/// name = "main"
/// kind = "liquid"
/// mount_position_body_m = [-3.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// propellant = "lox-methane"
/// cycle = "gas-generator"
/// chamber_pressure_mpa = 12.0
/// throat_radius_m = 0.15
/// expansion_ratio = 35.0
/// nozzle_length_m = 1.8
/// contour = "bell"
/// material = "nickel-superalloy"
/// cooling = "regenerative"
/// gimbal_range_rad = 0.09
/// restartable = true
/// ```
///
/// Solid motors use `kind = "solid"` with grain fields
/// (`outer_radius_m`, `core_radius_m`, `segment_length_m`, `segments`,
/// optional APCP ballistics overrides, `ignition_shots`) instead of the
/// chamber/cycle fields.
#[derive(Debug, Deserialize)]
struct EngineAsset {
    name: String,
    kind: EngineKind,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    // Liquid fields.
    #[serde(default)]
    propellant: Option<Propellant>,
    #[serde(default)]
    cycle: Option<EngineCycle>,
    #[serde(default)]
    chamber_pressure_mpa: Option<f64>,
    #[serde(default)]
    throat_radius_m: Option<f64>,
    #[serde(default)]
    expansion_ratio: Option<f64>,
    #[serde(default)]
    nozzle_length_m: Option<f64>,
    #[serde(default)]
    contour: Option<NozzleContour>,
    #[serde(default)]
    material: Option<MaterialAsset>,
    #[serde(default)]
    cooling: Option<CoolingMode>,
    #[serde(default)]
    gimbal_range_rad: f64,
    #[serde(default)]
    min_throttle: Option<f64>,
    #[serde(default = "default_true")]
    restartable: bool,
    // Solid grain fields.
    #[serde(default)]
    outer_radius_m: Option<f64>,
    #[serde(default)]
    core_radius_m: Option<f64>,
    #[serde(default)]
    segment_length_m: Option<f64>,
    #[serde(default)]
    segments: Option<u32>,
    #[serde(default)]
    burn_rate_coeff: Option<f64>,
    #[serde(default)]
    burn_rate_exponent: Option<f64>,
    #[serde(default = "default_one_shot")]
    ignition_shots: u32,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum EngineKind {
    Liquid,
    Solid,
}

#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum MaterialAsset {
    Preset(String),
    Custom(ChamberMaterial),
}

impl EngineAsset {
    fn bake(self) -> Result<EngineMount, Box<dyn Error>> {
        let compiled = match self.kind {
            EngineKind::Liquid => CompiledEngine::Liquid(self.liquid_spec()?.compile()?),
            EngineKind::Solid => CompiledEngine::Solid(self.solid_spec()?.compile()?),
        };
        Ok(EngineMount {
            name: self.name.clone(),
            engine: compiled,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
        })
    }

    fn material(&self) -> Result<ChamberMaterial, Box<dyn Error>> {
        let asset = self.material.as_ref().ok_or("engine needs material")?;
        match asset {
            MaterialAsset::Custom(material) => Ok(*material),
            MaterialAsset::Preset(name) => match name.as_str() {
                "regen-alloy" => Ok(ChamberMaterial::regen_alloy()),
                "nickel-superalloy" => Ok(ChamberMaterial::nickel_superalloy()),
                "radiative-niobium" => Ok(ChamberMaterial::radiative_niobium()),
                "ablative" => Ok(ChamberMaterial::ablative()),
                unknown => Err(format!("unknown material preset {unknown}").into()),
            },
        }
    }

    fn liquid_spec(&self) -> Result<LiquidEngineSpec, Box<dyn Error>> {
        Ok(LiquidEngineSpec {
            name: self.name.clone(),
            propellant: self.propellant.ok_or("liquid engine needs propellant")?,
            cycle: self.cycle.ok_or("liquid engine needs cycle")?,
            chamber_pressure_pa: required_mpa(self.chamber_pressure_mpa, "chamber_pressure_mpa")?,
            throat_radius_m: self
                .throat_radius_m
                .ok_or("liquid engine needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("liquid engine needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("liquid engine needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Bell),
            chamber_material: self.material()?,
            cooling: self.cooling.unwrap_or(CoolingMode::Regenerative),
            characteristic_length_m: None,
            gimbal_range_rad: self.gimbal_range_rad,
            min_throttle: self.min_throttle,
            restartable: self.restartable,
        })
    }

    fn solid_spec(&self) -> Result<SolidMotorSpec, Box<dyn Error>> {
        let (default_a, default_n) = SolidMotorSpec::apcp_ballistics();
        Ok(SolidMotorSpec {
            name: self.name.clone(),
            propellant: self.propellant.unwrap_or(Propellant::SolidApcp),
            outer_radius_m: self
                .outer_radius_m
                .ok_or("solid motor needs outer_radius_m")?,
            core_radius_m: self
                .core_radius_m
                .ok_or("solid motor needs core_radius_m")?,
            segment_length_m: self
                .segment_length_m
                .ok_or("solid motor needs segment_length_m")?,
            segments: self.segments.ok_or("solid motor needs segments")?,
            burn_rate_coeff: self.burn_rate_coeff.unwrap_or(default_a),
            burn_rate_exponent: self.burn_rate_exponent.unwrap_or(default_n),
            throat_radius_m: self
                .throat_radius_m
                .ok_or("solid motor needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("solid motor needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("solid motor needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Conical),
            casing_material: self.material()?,
            inhibited_ends: true,
            gimbal_range_rad: self.gimbal_range_rad,
            ignition_shots: self.ignition_shots,
        })
    }
}

fn required_mpa(value: Option<f64>, name: &str) -> Result<f64, Box<dyn Error>> {
    match value {
        Some(mpa) if mpa.is_finite() && mpa > 0.0 => Ok(mpa * 1.0e6),
        _ => Err(format!("liquid engine needs {name} in MPa").into()),
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

/// One body-local collision primitive from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[collision_parts]]
/// shape = "capsule"
/// position_body_m = [0.0, 0.0, 0.0]
/// orientation_body_xyzw = [0.0, 0.0, 0.0, 1.0]
/// axis = "x"
/// half_segment_m = 2.0
/// radius_m = 0.5
/// friction = 0.7
/// restitution = 0.0
/// ```
#[derive(Debug, Deserialize)]
struct CollisionPartAsset {
    #[serde(default)]
    position_body_m: [f64; 3],
    #[serde(default = "identity_quaternion")]
    orientation_body_xyzw: [f64; 4],
    #[serde(default = "default_friction")]
    friction: f64,
    #[serde(default)]
    restitution: f64,
    #[serde(flatten)]
    shape: CollisionShapeAsset,
}

impl CollisionPartAsset {
    fn bake(self) -> Result<CollisionPart, Box<dyn Error>> {
        let [x, y, z, w] = self.orientation_body_xyzw;
        Ok(CollisionPart::new(
            vector(self.position_body_m),
            DQuat::from_xyzw(x, y, z, w),
            self.shape.bake(),
            CollisionMaterial::new(self.friction, self.restitution)?,
        )?)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "shape", rename_all = "kebab-case")]
enum CollisionShapeAsset {
    Sphere {
        radius_m: f64,
    },
    Cuboid {
        half_extents_m: [f64; 3],
    },
    Capsule {
        axis: CollisionAxisAsset,
        half_segment_m: f64,
        radius_m: f64,
    },
}

impl CollisionShapeAsset {
    fn bake(self) -> CollisionShape {
        match self {
            Self::Sphere { radius_m } => CollisionShape::Sphere { radius_m },
            Self::Cuboid { half_extents_m } => CollisionShape::Cuboid {
                half_extents_m: vector(half_extents_m),
            },
            Self::Capsule {
                axis,
                half_segment_m,
                radius_m,
            } => CollisionShape::Capsule {
                axis: axis.into(),
                half_segment_m,
                radius_m,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum CollisionAxisAsset {
    X,
    Y,
    Z,
}

impl From<CollisionAxisAsset> for CollisionAxis {
    fn from(value: CollisionAxisAsset) -> Self {
        match value {
            CollisionAxisAsset::X => Self::X,
            CollisionAxisAsset::Y => Self::Y,
            CollisionAxisAsset::Z => Self::Z,
        }
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

fn identity_quaternion() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

fn default_friction() -> f64 {
    CollisionMaterial::default().friction
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
        assert_eq!(vehicle.collision_geometry.parts.len(), 4);
        assert!(vehicle.collision_geometry.validate().is_ok());
        assert_eq!(vehicle.mass_properties.mass_kg, 1_000.0);
        let json = serde_json::to_string(&vehicle).expect("vehicle JSON should serialize");
        let round_trip: VehicleDefinition =
            serde_json::from_str(&json).expect("vehicle JSON should deserialize");
        assert_eq!(round_trip, vehicle);
    }

    #[test]
    fn collision_part_asset_bakes_without_backend_types() {
        let part: CollisionPartAsset = toml::from_str(
            r#"
shape = "capsule"
axis = "x"
half_segment_m = 2.0
radius_m = 0.5
friction = 0.8
"#,
        )
        .expect("collision part TOML should parse");
        let baked = part.bake().expect("collision part should validate");
        assert_eq!(baked.local_position_m, DVec3::ZERO);
        assert_eq!(baked.local_orientation, DQuat::IDENTITY);
        assert_eq!(baked.material.friction, 0.8);
        assert!(matches!(
            baked.shape,
            CollisionShape::Capsule {
                axis: CollisionAxis::X,
                half_segment_m: 2.0,
                radius_m: 0.5,
            }
        ));
    }

    #[test]
    fn example_rocket_bakes_compiled_engines_with_mass() {
        let asset: VehicleAsset =
            toml::from_str(include_str!("../../../data/vehicles/example_rocket.toml"))
                .expect("rocket TOML should parse");
        let vehicle = asset.bake().expect("rocket asset should bake");
        assert_eq!(vehicle.engines.len(), 2);
        // Engine masses aggregate on top of the 2000 kg structure.
        assert!(vehicle.mass_properties.mass_kg > 2000.0);
        let engines_mass: f64 = vehicle
            .engines
            .iter()
            .map(|mount| mount.engine.bake_mass_kg())
            .sum();
        assert!(
            (vehicle.mass_properties.mass_kg - 2000.0 - engines_mass).abs() < 1e-6,
            "baked mass must equal structure plus engines"
        );
        // Uniform full-throttle command produces +X thrust at sea level.
        let thrust = vehicle
            .total_thrust_body_n(1.0, 101_325.0, 0.0)
            .expect("thrust evaluates");
        assert!(thrust.x > 1.0e6, "main + booster must clear 1 MN");
        assert_eq!(thrust.y, 0.0);
        assert_eq!(thrust.z, 0.0);
        let json = serde_json::to_string(&vehicle).expect("vehicle JSON serializes");
        let round_trip: VehicleDefinition =
            serde_json::from_str(&json).expect("vehicle JSON deserializes");
        // JSON is a transfer artifact, not a bitwise archive (serde_json
        // float parsing can dust the last ulp): compare structurally with
        // a tight relative tolerance instead of exact equality.
        assert_eq!(round_trip.engines.len(), vehicle.engines.len());
        for (actual, expected) in round_trip.engines.iter().zip(vehicle.engines.iter()) {
            assert_eq!(actual.name, expected.name);
            let mass_delta = (actual.engine.bake_mass_kg() - expected.engine.bake_mass_kg()).abs()
                / expected.engine.bake_mass_kg();
            assert!(mass_delta < 1e-9, "engine mass drift {mass_delta:e}");
        }
        let mass_delta = (round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg)
            .abs()
            / vehicle.mass_properties.mass_kg;
        assert!(mass_delta < 1e-12, "vehicle mass drift {mass_delta:e}");
    }

    #[test]
    fn engine_mount_axis_must_be_unit() {
        let asset: VehicleAsset =
            toml::from_str(include_str!("../../../data/vehicles/example_rocket.toml"))
                .expect("rocket TOML should parse");
        let mut vehicle = asset.bake().expect("rocket asset should bake");
        vehicle.engines[0].thrust_axis_body = [2.0, 0.0, 0.0];
        assert!(vehicle.validate().is_err());
    }
}
