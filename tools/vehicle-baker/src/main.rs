use std::{env, error::Error, fs, path::PathBuf};

use analyzer::run_analyzer;
use asset_schema::VehicleAsset;
#[cfg(test)]
use asset_schema::{CollisionPartAsset, parallel_axis};
use glam::{DMat3, DQuat, DVec3};
use serde::Deserialize;
use thessa_aero_surfaces::{
    CollisionOptions, CompileOptions, MechanismState, ProceduralSurface, compile_surface,
};
use thessa_fuselage::{
    AssemblyLink, AttachKind, BodyCollisionOptions, BodyCompileOptions, BodyTransform,
    CabinSeatRole, CompiledBody, DoorSide, ExitType, PortKind, ProceduralBody, RegionKind,
    body_collision_parts, compile_assembly, compile_body,
};
use thessa_sim_core::{
    AeroGeometry, AeroPanel, AirCycle, AirbreathingSpec, AssemblyEndpoint, AssemblyLinkState,
    AssemblyVolume, AtmosphereConfig, CabinExit, CabinExitSide, CabinExitType, CabinMonument,
    CabinMonumentKind, CabinSeat, CabinSeatClass, CabinSeatRole as RuntimeCabinSeatRole,
    CabinSeatStyle, CabinSuitType, ChamberMaterial, ChamberSpec, CollisionAxis, CollisionGeometry,
    CollisionMaterial, CollisionPart, CollisionShape, CompiledEngine, CompiledJet, ControlCore,
    ControlMixing, ControlStation, ControlSurfaceDefinition, CoolingMode, ElectricPropellant,
    ElectricThrusterDesign, ElectricThrusterMount, ElectricThrusterSpec, EngineCycle, EngineMount,
    EstocEjectorSpec, EstocPrecoolerSpec, EstocSpec, FoldJointRecord, FusionReaction,
    FusionTorchMount, FusionTorchSpec, IntakeKind, JetFuel, JetMount, LandingLegSpec,
    LandingShockAbsorberSpec, LiquidEngineSpec, NamedAssemblyLink, NozzleContour, NtrFluid,
    NuclearThermalSpec, ParachuteSpec, PressurizedCabin, Propellant, PropellerDriveMount,
    PropellerDriveSpec, PropellerSpec, PropulsionSystemSpec, PulsedFusionMount, PulsedFusionSpec,
    ReactionWheelBankSpec, RigidBodyProperties, ShaftPowerSourceSpec, ShaftSpec,
    SolidGrainGeometry, SolidMotorSpec, SystemMount, TankMount, TankShape, TankSpec,
    TurbopropDriveSpec, TurbopropMount, VehicleAssembly, VehicleDefinition, WheelBrakeSpec,
    WheelChassisRetractionSpec, WheelChassisSpec, WheelDriveSpec, WheelLayout, WheelStrutSpec,
    WheelTireSpec, analyze_airbreathing, analyze_altitude, analyze_estoc, analyze_propeller_drive,
    analyze_turboprop_drive,
};

mod analyzer;
mod asset_schema;
mod debug_mesh;

#[cfg(test)]
mod integration;

fn main() -> Result<(), Box<dyn Error>> {
    let options = Options::parse(env::args().skip(1))?;
    if options.help {
        print_help();
        return Ok(());
    }
    let source = fs::read_to_string(&options.input)?;
    let asset: VehicleAsset = toml::from_str(&source)?;
    if let Some(output_dir) = options.debug_body_mesh_dir.as_deref() {
        debug_mesh::export_body_meshes(&asset.procedural_bodies, output_dir)?;
    }
    let vehicle = asset.bake()?;
    println!("vehicle: {}", vehicle.name);
    println!("panels: {}", vehicle.aero_geometry.panels.len());
    println!("control surfaces: {}", vehicle.control_surfaces.len());
    println!(
        "collision parts: {}",
        vehicle.collision_geometry.parts.len()
    );
    println!("mass: {:.3} kg", vehicle.mass_properties.mass_kg);
    for chassis in &vehicle.wheel_chassis {
        println!(
            "wheel chassis {}: {} stations, {:.3} kg",
            chassis.spec.name,
            chassis.wheel_stations.len(),
            chassis.dry_mass_kg(),
        );
    }
    let authority = vehicle.control_authority();
    println!(
        "control authority: {} ({:?})",
        if authority.controllable {
            "controllable"
        } else {
            "uncontrollable"
        },
        authority.reason
    );
    for mount in &vehicle.tanks {
        println!(
            "tank: {:.3} m^3 capacity, dry {:.1} kg, loaded {:.0}/{:.0} kg",
            mount.tank.volume_m3,
            mount.tank.dry_mass_kg,
            mount.loaded_propellant_kg(),
            mount.tank.full_propellant_kg,
        );
    }
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
    for mount in &vehicle.systems {
        println!(
            "system {} ({} chambers): vacuum thrust {:.1} kN, dry {:.1} kg",
            mount.name,
            mount.system.chambers.len(),
            mount.system.total_thrust_vac_n / 1000.0,
            mount.system.dry_mass_kg,
        );
    }
    for mount in &vehicle.jets {
        let (kind, static_thrust_n) = match &mount.engine {
            CompiledJet::Air(engine) => ("jet", engine.design_static_thrust_n),
            CompiledJet::Estoc(engine) => (
                "estoc",
                engine.air.design_static_thrust_n + engine.rocket_thrust_vac_n,
            ),
        };
        println!(
            "jet {} ({kind}): static thrust {:.1} kN, dry {:.1} kg",
            mount.name,
            static_thrust_n / 1000.0,
            mount.engine.dry_mass_kg(),
        );
    }
    for mount in &vehicle.propeller_drives {
        println!(
            "propeller drive {}: ideal disk {:.2} m, dry {:.1} kg",
            mount.name, mount.drive.propeller.diameter_m, mount.drive.dry_mass_kg,
        );
    }
    if let Some(output) = options.output {
        let json = serde_json::to_string_pretty(&vehicle)?;
        fs::write(&output, format!("{json}\n"))?;
        println!("wrote: {}", output.display());
    }
    if options.analyze {
        run_analyzer(
            &vehicle,
            options.throttle,
            options.burn_time_s,
            options.analyze_json,
            &options.composition,
            options.source_rpm,
            options.power_takeoff_fraction,
        )?;
    }
    Ok(())
}

struct Options {
    input: PathBuf,
    output: Option<PathBuf>,
    debug_body_mesh_dir: Option<PathBuf>,
    help: bool,
    analyze: bool,
    throttle: f64,
    burn_time_s: f64,
    analyze_json: bool,
    /// Atmosphere composition design string for the jet analyzer (the
    /// species basis is explicit; default is Thessa air `N2/O2/AR/CO2` —
    /// never a hard-coded oxygen scalar).
    composition: String,
    /// Commanded source-shaft speed for propeller-drive analyzer rows.
    source_rpm: f64,
    /// Requested fraction of available power-turbine output in the analyzer.
    power_takeoff_fraction: f64,
}

impl Options {
    fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, Box<dyn Error>> {
        let mut input = PathBuf::from("data/vehicles/example_aircraft.toml");
        let mut output = None;
        let mut debug_body_mesh_dir = None;
        let mut help = false;
        let mut analyze = false;
        let mut throttle = 1.0;
        let mut burn_time_s = 0.0;
        let mut analyze_json = false;
        let mut composition = "N2/O2/AR/CO2".to_string();
        let mut source_rpm: f64 = 2_400.0;
        let mut power_takeoff_fraction: f64 = 0.25;
        let mut arguments = arguments.peekable();
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--input" => input = PathBuf::from(required_value(&mut arguments, "--input")?),
                "--output" => {
                    output = Some(PathBuf::from(required_value(&mut arguments, "--output")?))
                }
                "--debug-body-mesh-dir" => {
                    debug_body_mesh_dir = Some(PathBuf::from(required_value(
                        &mut arguments,
                        "--debug-body-mesh-dir",
                    )?));
                }
                "--analyze" => analyze = true,
                "--analyze-json" => {
                    analyze = true;
                    analyze_json = true;
                }
                "--throttle" => {
                    throttle = required_value(&mut arguments, "--throttle")?
                        .parse()
                        .map_err(|_| "--throttle needs a number in [0, 1]")?;
                }
                "--burn-time" => {
                    burn_time_s = required_value(&mut arguments, "--burn-time")?
                        .parse()
                        .map_err(|_| "--burn-time needs seconds >= 0")?;
                }
                "--composition" => {
                    composition = required_value(&mut arguments, "--composition")?;
                }
                "--source-rpm" => {
                    source_rpm = required_value(&mut arguments, "--source-rpm")?
                        .parse()
                        .map_err(|_| "--source-rpm needs a finite RPM >= 0")?;
                }
                "--power-takeoff-fraction" => {
                    power_takeoff_fraction =
                        required_value(&mut arguments, "--power-takeoff-fraction")?
                            .parse()
                            .map_err(|_| "--power-takeoff-fraction needs a number in [0, 1]")?;
                }
                "--help" | "-h" => help = true,
                unknown => return Err(format!("unknown argument {unknown}; use --help").into()),
            }
        }
        if !source_rpm.is_finite() || source_rpm < 0.0 {
            return Err("--source-rpm must be finite and >= 0".into());
        }
        if !power_takeoff_fraction.is_finite() || !(0.0..=1.0).contains(&power_takeoff_fraction) {
            return Err("--power-takeoff-fraction must be finite in [0, 1]".into());
        }
        Ok(Self {
            input,
            output,
            debug_body_mesh_dir,
            help,
            analyze,
            throttle,
            burn_time_s,
            analyze_json,
            composition,
            source_rpm,
            power_takeoff_fraction,
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
        "Usage: thessa-vehicle-baker [--input data/vehicles/example_aircraft.toml] [--output data/vehicles/example_aircraft.baked.json] [--debug-body-mesh-dir DIR] [--analyze [--throttle 1.0] [--burn-time 0.0] [--analyze-json] [--composition N2/O2/AR/CO2] [--source-rpm 2400] [--power-takeoff-fraction 0.25]]"
    );
    println!("--debug-body-mesh-dir exports procedural fuselage meshes as OBJ before baking.");
    println!(
        "--composition sets the analyzer atmosphere species (design string, default Thessa air N2/O2/AR/CO2; unknown gases are refused)."
    );
    println!("--source-rpm sets the steady shaft speed used by propeller-drive analyzer rows.");
    println!(
        "--power-takeoff-fraction requests this share of available turboprop shaft output [0, 1]."
    );
}

#[cfg(test)]
mod body_tests;
#[cfg(test)]
mod cabin_tests;
#[cfg(test)]
mod tests;
