use std::{env, error::Error, fs, path::PathBuf};

use glam::{DMat3, DQuat, DVec3};
use serde::Deserialize;
use thessa_aero_surfaces::{
    CollisionOptions, CompileOptions, MechanismState, ProceduralSurface, compile_surface,
};
use thessa_sim_core::{
    AeroGeometry, AeroPanel, AirCycle, AirbreathingSpec, AtmosphereConfig, ChamberMaterial,
    ChamberSpec, CollisionAxis, CollisionGeometry, CollisionMaterial, CollisionPart,
    CollisionShape, CompiledEngine, CompiledJet, ControlSurfaceDefinition, CoolingMode,
    ElectricPropellant, ElectricThrusterDesign, ElectricThrusterMount, ElectricThrusterSpec,
    EngineCycle, EngineMount, EstocSpec, FoldJointRecord, FusionReaction, FusionTorchMount,
    FusionTorchSpec, IntakeKind, JetFuel, JetMount, LiquidEngineSpec, NozzleContour, NtrFluid,
    NuclearThermalSpec, Propellant, PropellerDriveMount, PropellerDriveSpec, PropellerSpec,
    PropulsionSystemSpec, PulsedFusionMount, PulsedFusionSpec, RigidBodyProperties,
    ShaftPowerSourceSpec, ShaftSpec, SolidGrainGeometry, SolidMotorSpec, SystemMount, TankMount,
    TankShape, TankSpec, TurbopropDriveSpec, TurbopropMount, VehicleDefinition,
    analyze_airbreathing, analyze_altitude, analyze_propeller_drive, analyze_turboprop_drive,
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
    for mount in &vehicle.tanks {
        println!(
            "tank: {:.3} m^3, dry {:.1} kg, full fill {:.0} kg",
            mount.tank.volume_m3, mount.tank.dry_mass_kg, mount.tank.full_propellant_kg,
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

/// Juno Performance Analyzer equivalent for the terminal: thrust/Isp over
/// altitude per engine (plus the uniform-command vehicle total) at fixed
/// throttle. `--analyze-json` emits the same rows as JSON for the future
/// editor UI to consume.
fn run_analyzer(
    vehicle: &VehicleDefinition,
    throttle: f64,
    burn_time_s: f64,
    as_json: bool,
    composition: &str,
    source_rpm: f64,
    power_takeoff_fraction: f64,
) -> Result<(), Box<dyn Error>> {
    // Species basis is explicit: the atmosphere derives both its gas
    // properties and its species from the design composition string
    // (default Thessa air), never from a hard-coded oxygen scalar
    // (doc 04 section 10 / 18.6).
    let atmosphere = AtmosphereConfig::from_composition(composition, 288.15, 101_325.0, 9.80665)?;
    let altitudes: Vec<f64> = (0..=10).map(|k| k as f64 * 8000.0).collect();
    let airspeeds_mps = [0.0, 50.0, 100.0, 150.0];
    if as_json {
        let mut rows = Vec::new();
        for mount in &vehicle.engines {
            rows.push(serde_json::json!({
                "engine": mount.name,
                "points": analyze_altitude(
                    &mount.engine,
                    &atmosphere,
                    &altitudes,
                    throttle,
                    burn_time_s,
                )?,
            }));
        }
        for mount in &vehicle.systems {
            let throttles = vec![throttle; mount.system.chambers.len()];
            rows.push(serde_json::json!({
                "system": mount.name,
                "points": mount.system.analyze_system_altitude(
                    &atmosphere,
                    &altitudes,
                    &throttles,
                )?,
            }));
        }
        for mount in &vehicle.jets {
            let air = match &mount.engine {
                CompiledJet::Air(engine) => engine.as_ref(),
                CompiledJet::Estoc(engine) => &engine.air,
            };
            let mach_grid: &[f64] = match air.cycle {
                thessa_sim_core::AirCycle::Scramjet => &[0.0, 1.0, 2.0, 4.0, 6.0, 8.0],
                thessa_sim_core::AirCycle::Ramjet => &[0.0, 1.0, 2.0, 3.0, 4.0],
                _ => &[0.0, 1.0, 2.0, 3.0],
            };
            rows.push(serde_json::json!({
                "jet": mount.name,
                "points": analyze_airbreathing(
                    air,
                    &atmosphere,
                    &altitudes,
                    mach_grid,
                    throttle,
                )?,
            }));
        }
        for mount in &vehicle.propeller_drives {
            rows.push(serde_json::json!({
                "propeller_drive": mount.name,
                "points": analyze_propeller_drive(
                    &mount.drive,
                    &atmosphere,
                    &altitudes,
                    &airspeeds_mps,
                    throttle,
                    source_rpm,
                )?,
            }));
        }
        for mount in &vehicle.turboprops {
            rows.push(serde_json::json!({
                "turboprop": mount.name,
                "points": analyze_turboprop_drive(
                    &mount.drive,
                    &atmosphere,
                    &altitudes,
                    &airspeeds_mps,
                    throttle,
                    power_takeoff_fraction,
                )?,
            }));
        }
        println!("{}", serde_json::to_string_pretty(&rows)?);
        return Ok(());
    }
    for mount in &vehicle.engines {
        println!("--- analyzer: {} (throttle {throttle})", mount.name);
        println!(
            "{:>10} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "p_amb", "thrust_kN", "isp_s", "sep"
        );
        let curve = analyze_altitude(
            &mount.engine,
            &atmosphere,
            &altitudes,
            throttle,
            burn_time_s,
        )?;
        for point in &curve {
            println!(
                "{:>10.0} {:>10.0} {:>12.1} {:>10.1} {:>5}",
                point.altitude_m,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                if point.separation_risk { "SEP" } else { "" },
            );
        }
    }
    for mount in &vehicle.systems {
        println!(
            "--- analyzer: {} (throttle {throttle}, {} chambers)",
            mount.name,
            mount.system.chambers.len()
        );
        println!(
            "{:>10} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "p_amb", "thrust_kN", "isp_s", "sep"
        );
        let throttles = vec![throttle; mount.system.chambers.len()];
        let curve = mount
            .system
            .analyze_system_altitude(&atmosphere, &altitudes, &throttles)?;
        for point in &curve {
            println!(
                "{:>10.0} {:>10.0} {:>12.1} {:>10.1} {:>5}",
                point.altitude_m,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                if point.separation_any { "SEP" } else { "" },
            );
        }
    }
    for mount in &vehicle.jets {
        let air = match &mount.engine {
            CompiledJet::Air(engine) => engine.as_ref(),
            CompiledJet::Estoc(engine) => &engine.air,
        };
        let mach_grid: &[f64] = match air.cycle {
            thessa_sim_core::AirCycle::Scramjet => &[0.0, 1.0, 2.0, 4.0, 6.0, 8.0],
            thessa_sim_core::AirCycle::Ramjet => &[0.0, 1.0, 2.0, 3.0, 4.0],
            _ => &[0.0, 1.0, 2.0, 3.0],
        };
        println!(
            "--- analyzer: {} (throttle {throttle}, air path)",
            mount.name
        );
        println!(
            "{:>10} {:>6} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "mach", "p_amb", "thrust_kN", "isp_s", "flags"
        );
        let grid = analyze_airbreathing(air, &atmosphere, &altitudes, mach_grid, throttle)?;
        for point in &grid {
            let mut flags = String::new();
            if point.air_limited {
                flags.push('A');
            }
            if point.oxygen_limited {
                flags.push('O');
            }
            if point.drive_limited {
                flags.push('D');
            }
            if point.combustion_thermal_limited {
                flags.push('T');
            }
            if point.scramjet_limited {
                flags.push('M');
            }
            if point.separation_risk {
                flags.push('S');
            }
            println!(
                "{:>10.0} {:>6.1} {:>10.0} {:>12.1} {:>10.0} {:>5}",
                point.altitude_m,
                point.mach,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                flags,
            );
        }
    }
    for mount in &vehicle.propeller_drives {
        println!(
            "--- analyzer: {} (throttle {throttle}, source {source_rpm:.0} RPM)",
            mount.name
        );
        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>10} {:>10} {:>5}",
            "alt_m", "speed", "p_amb", "thrust_kN", "fuel_g/s", "bus_kW", "flags"
        );
        let rows = analyze_propeller_drive(
            &mount.drive,
            &atmosphere,
            &altitudes,
            &airspeeds_mps,
            throttle,
            source_rpm,
        )?;
        for point in &rows {
            let mut flags = String::new();
            if point.oxygen_limited {
                flags.push('O');
            }
            if point.thermal_limited {
                flags.push('T');
            }
            if point.density_limited {
                flags.push('V');
            }
            if point.source_speed_limited {
                flags.push('R');
            }
            println!(
                "{:>10.0} {:>8.0} {:>10.0} {:>12.2} {:>10.2} {:>10.2} {:>5}",
                point.altitude_m,
                point.airspeed_mps,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.fuel_flow_kg_s * 1000.0,
                point.electrical_power_w / 1000.0,
                flags,
            );
        }
    }
    for mount in &vehicle.turboprops {
        println!(
            "--- analyzer: {} (throttle {throttle}, PTO {:.0}% of available)",
            mount.name,
            power_takeoff_fraction * 100.0,
        );
        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>12} {:>10} {:>5}",
            "alt_m", "speed", "p_amb", "total_kN", "prop_kN", "PTO_kW", "spool"
        );
        let rows = analyze_turboprop_drive(
            &mount.drive,
            &atmosphere,
            &altitudes,
            &airspeeds_mps,
            throttle,
            power_takeoff_fraction,
        )?;
        for point in &rows {
            println!(
                "{:>10.0} {:>8.0} {:>10.0} {:>12.2} {:>12.2} {:>10.1} {:>5.2}",
                point.altitude_m,
                point.airspeed_mps,
                point.ambient_pa,
                point.total_thrust_n / 1000.0,
                point.propeller_thrust_n / 1000.0,
                point.power_takeoff_w / 1000.0,
                point.spool_n,
            );
        }
    }
    Ok(())
}

#[derive(Debug, Deserialize)]
struct VehicleAsset {
    name: String,
    mass_kg: f64,
    /// Matrix is written as rows in the TOML file for readability.
    inertia_body_kg_m2: [[f64; 3]; 3],
    /// Hand-authored solver panels. Empty for all-procedural assets;
    /// legacy assets may carry only these while panels are migrated.
    #[serde(default)]
    panels: Vec<PanelAsset>,
    #[serde(default)]
    control_surfaces: Vec<ControlSurfaceAsset>,
    /// Procedural wing surfaces compiled hangar-side into solver panels.
    /// The compiler never runs in flight: it bakes `AeroPanel` zones plus
    /// control definitions here, and the vehicle asset carries only the
    /// compiled output. Empty keeps legacy hand-panel assets valid.
    #[serde(default)]
    procedural_surfaces: Vec<ProceduralSurface>,
    /// Compile contact boxes from procedural surfaces into the collision
    /// geometry (one body-axis box per mechanism region). Default true:
    /// the documented hangar pipeline; set false to keep hand-authored
    /// contact geometry only.
    #[serde(default = "default_true")]
    surface_collision: bool,
    /// Solver-neutral contact primitives. Legacy assets may omit this while
    /// collision geometry is migrated; contact-active runtime code must not.
    #[serde(default)]
    collision_parts: Vec<CollisionPartAsset>,
    /// Procedural engine mounts (Juno-simple-mode authoring). Empty keeps
    /// engine-less assets valid; input mass semantics are "structure
    /// without engines" once mounts are present.
    #[serde(default)]
    engines: Vec<EngineAsset>,
    /// Propellant tanks (dry + full-fill mass aggregates at bake).
    #[serde(default)]
    tanks: Vec<TankAsset>,
    /// Multi-chamber propulsion systems (shared feed, per-chamber nozzles).
    #[serde(default)]
    systems: Vec<SystemAsset>,
    /// Air-breathing jets and ESTOCs (mass aggregates at bake; thrust
    /// needs a flight condition at query time).
    #[serde(default)]
    jets: Vec<JetAsset>,
    /// Electric spacecraft thrusters, with power processor and radiator mass.
    #[serde(default)]
    electric_thrusters: Vec<ElectricThrusterAsset>,
    /// Continuous magnetic-nozzle fusion torches.
    #[serde(default)]
    fusion_torches: Vec<FusionTorchAsset>,
    /// Discrete pellet/impulse fusion systems with finite energy buffers.
    #[serde(default)]
    pulsed_fusion_systems: Vec<PulsedFusionAsset>,
    /// Piston/electric shaft sources driving reusable ideal propeller disks.
    #[serde(default)]
    propeller_drives: Vec<PropellerDriveAsset>,
    /// Gas turbines coupled to propellers through an explicit power turbine.
    #[serde(default)]
    turboprops: Vec<TurbopropAsset>,
}

impl VehicleAsset {
    fn bake(self) -> Result<VehicleDefinition, Box<dyn Error>> {
        let mut panels = self
            .panels
            .into_iter()
            .map(PanelAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mut controls = self
            .control_surfaces
            .into_iter()
            .map(ControlSurfaceAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        // Hangar-side procedural compilation: each surface contributes
        // its panels (appended after hand panels) with control indices
        // rebased onto the merged panel list. Structural mass, first
        // moments, and inertia aggregate about the authoring origin;
        // fuel volume is reported for tank placement (no TankShape fits a
        // wing box, so mounts are not fabricated).
        let mut surface_mass_kg = 0.0;
        let mut surface_moment = DVec3::ZERO;
        let mut surface_inertia = DMat3::ZERO;
        let mut surface_fuel_m3 = 0.0;
        let mut fold_joints = Vec::new();
        let mut parked_tags: Vec<(usize, usize)> = Vec::new();
        let mut surface_collision_parts = Vec::new();
        for surface in &self.procedural_surfaces {
            let compiled = compile_surface(
                surface,
                &CompileOptions::default(),
                &MechanismState::deployed(),
            )
            .map_err(|error| format!("surface '{}': {error}", surface.name))?;
            println!(
                "surface '{}': {} panels, estimated error {:.3e} m^2",
                surface.name,
                compiled.panels.len(),
                compiled.summary.estimated_error_m2
            );
            if let Some(structure) = &compiled.structure {
                println!(
                    "surface '{}': structure {:.1} kg, fuel {:.3} m^3 at {:?}",
                    surface.name,
                    structure.mass_kg,
                    structure.fuel_volume_m3,
                    structure.fuel_centroid_body_m
                );
                surface_mass_kg += structure.mass_kg;
                surface_moment += structure.center_of_mass_body_m * structure.mass_kg;
                surface_inertia += structure.inertia_body_kg_m2;
                surface_fuel_m3 += structure.fuel_volume_m3;
            }
            let base = panels.len();
            let joint_base = fold_joints.len();
            // Fold tags resolve against joints attached later, so park
            // them aside: VehicleDefinition::new validates eagerly, then
            // with_fold_joints re-validates, then tags restore plus a
            // final validation below.
            for (panel_index, panel) in compiled.panels.iter().enumerate() {
                let mut panel = *panel;
                if let Some(joint) = compiled.tags[panel_index].fold {
                    parked_tags.push((panels.len(), joint_base + joint));
                    panel.fold_index = None;
                }
                panels.push(panel);
            }
            let def_base = controls.len();
            for definition in &compiled.controls {
                let mut rebased = ControlSurfaceDefinition::new(
                    definition.name.clone(),
                    definition
                        .panel_indices
                        .iter()
                        .map(|index| base + index)
                        .collect(),
                    definition.minimum_deflection_rad,
                    definition.maximum_deflection_rad,
                )?;
                rebased = rebased.with_kind(definition.kind);
                if let Some(parent) = definition.parent_index {
                    rebased = rebased.with_parent(def_base + parent);
                }
                controls.push(rebased);
            }
            for (fold, joint) in compiled.folds.iter().zip(surface.folds.iter()) {
                fold_joints.push(FoldJointRecord {
                    name: format!("{}.{}", surface.name, fold.name),
                    hinge_body_m: fold.hinge_body_m,
                    axis_body: fold.axis_body,
                    angle_rad: fold.angle_rad,
                    deployed_angle_rad: fold.deployed_angle_rad,
                    deployment_rate_rad_s: joint.deployment_rate_rad_s,
                    lock_window_rad: joint.lock_window_rad,
                    max_dynamic_pressure_pa: joint.max_dynamic_pressure_pa,
                    parent_joint: fold.parent_joint.map(|parent| joint_base + parent),
                });
            }
            if self.surface_collision {
                let parts = compiled
                    .collision_parts(&CollisionOptions::default())
                    .map_err(|error| format!("surface '{}': {error}", surface.name))?;
                println!("surface '{}': {} contact boxes", surface.name, parts.len());
                surface_collision_parts.extend(parts);
            }
        }
        // Bake mounts first (authoring stations): the single final COM
        // below needs every mass contributor before anything shifts.
        let mut collision_parts = self
            .collision_parts
            .into_iter()
            .map(CollisionPartAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let mounts = self
            .engines
            .into_iter()
            .map(EngineAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let tank_mounts = self
            .tanks
            .into_iter()
            .map(TankAsset::bake)
            .collect::<Result<Vec<TankMount>, _>>()?;
        let system_mounts = self
            .systems
            .into_iter()
            .map(SystemAsset::bake)
            .collect::<Result<Vec<SystemMount>, _>>()?;
        let jet_mounts = self
            .jets
            .into_iter()
            .map(JetAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let electric_thruster_mounts = self
            .electric_thrusters
            .into_iter()
            .map(ElectricThrusterAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let fusion_torch_mounts = self
            .fusion_torches
            .into_iter()
            .map(FusionTorchAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let pulsed_fusion_mounts = self
            .pulsed_fusion_systems
            .into_iter()
            .map(PulsedFusionAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let propeller_drive_mounts = self
            .propeller_drives
            .into_iter()
            .map(PropellerDriveAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let turboprop_mounts = self
            .turboprops
            .into_iter()
            .map(TurbopropAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        // Assembly center of mass over EVERYTHING: hand mass rides the
        // authoring origin, surfaces/engine/tank/system/jet masses ride
        // their stations. Flight integrates moments about the body
        // origin, so the baker recenters the whole asset onto the final
        // COM in one shift (legacy hand-only assets sit at zero and
        // shift by nothing). Engine/tank/system/jet mass calls below
        // then add point terms about already-centered stations, and the
        // same accumulator shape serves future fuel-driven COM motion.
        let mut total_mass_kg = self.mass_kg + surface_mass_kg;
        let mut total_moment = surface_moment;
        for mount in &mounts {
            let mass = mount.engine.bake_mass_kg();
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &tank_mounts {
            let mass = mount.tank.dry_mass_kg + mount.tank.full_propellant_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &system_mounts {
            let mut chamber_mass = 0.0;
            let mut centroid = DVec3::ZERO;
            for chamber in &mount.system.chambers {
                total_mass_kg += chamber.dry_mass_kg;
                chamber_mass += chamber.dry_mass_kg;
                let at = DVec3::from_array(chamber.position_body_m);
                total_moment += at * chamber.dry_mass_kg;
                centroid += at * chamber.dry_mass_kg;
            }
            let shared = (mount.system.dry_mass_kg - chamber_mass).max(0.0);
            total_mass_kg += shared;
            if chamber_mass > 0.0 {
                total_moment += centroid / chamber_mass * shared;
            }
        }
        for mount in &jet_mounts {
            let mass = mount.engine.dry_mass_kg();
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &electric_thruster_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &fusion_torch_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &pulsed_fusion_mounts {
            let mass = mount.engine.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &propeller_drive_mounts {
            let mass = mount.drive.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        for mount in &turboprop_mounts {
            let mass = mount.drive.dry_mass_kg;
            total_mass_kg += mass;
            total_moment += DVec3::from_array(mount.position_body_m) * mass;
        }
        let assembly_com = if total_mass_kg > 0.0 {
            total_moment / total_mass_kg
        } else {
            DVec3::ZERO
        };
        if assembly_com.length() > 1e-12 {
            println!(
                "assembly center of mass at [{:.3}, {:.3}, {:.3}], recentering",
                assembly_com.x, assembly_com.y, assembly_com.z
            );
        }
        let shift = -assembly_com;
        let geometry = AeroGeometry::new(panels)?;
        // Properties stay in the authoring frame here (hand plus
        // surfaces, always positive-definite); the single recenter to
        // the final COM happens on the built vehicle below, after every
        // mass contributor is attached. Recentering an intermediate sum
        // that misses mount masses can go indefinite.
        let inertia = rows_to_matrix(self.inertia_body_kg_m2);
        let properties =
            RigidBodyProperties::new(self.mass_kg + surface_mass_kg, inertia + surface_inertia)?;
        if surface_fuel_m3 > 0.0 {
            println!("wing fuel volume: {surface_fuel_m3:.3} m^3");
        }
        collision_parts.extend(surface_collision_parts);
        let collision_geometry = CollisionGeometry::new(collision_parts)?;
        // Feed cross-check: pressure-fed engines have no pump to hide
        // behind, so a tank must hold their full feed pressure. Pump-fed
        // cycles generate the rise themselves (chamber pressure is already
        // gated by the cycle cap at compile time).
        let strongest_tank_pa = tank_mounts
            .iter()
            .map(|mount| mount.tank.max_pressure_pa)
            .fold(0.0_f64, f64::max);
        for mount in &mounts {
            if let CompiledEngine::Liquid(engine) = &mount.engine
                && engine.cycle == EngineCycle::PressureFed
                && strongest_tank_pa < engine.feed_pressure_required_pa
            {
                return Err(format!(
                    "tank pressure {:.2} MPa cannot pressure-feed {} (needs {:.2} MPa)",
                    strongest_tank_pa / 1.0e6,
                    mount.name,
                    engine.feed_pressure_required_pa / 1.0e6
                )
                .into());
            }
        }
        for mount in &system_mounts {
            if mount.system.cycle == EngineCycle::PressureFed
                && strongest_tank_pa < mount.system.feed_pressure_required_pa
            {
                return Err(format!(
                    "tank pressure {:.2} MPa cannot pressure-feed {} (needs {:.2} MPa)",
                    strongest_tank_pa / 1.0e6,
                    mount.name,
                    mount.system.feed_pressure_required_pa / 1.0e6
                )
                .into());
            }
        }
        let mut vehicle = VehicleDefinition::new(self.name, geometry, properties, controls)?
            .with_collision_geometry(collision_geometry)?
            .with_engines(mounts)?
            .with_tanks(tank_mounts)?
            .with_systems(system_mounts)?
            .with_fold_joints(fold_joints)?
            .with_jets(jet_mounts)?
            .with_electric_thrusters(electric_thruster_mounts)?
            .with_fusion_torches(fusion_torch_mounts)?
            .with_pulsed_fusion_systems(pulsed_fusion_mounts)?
            .with_propeller_drives(propeller_drive_mounts)?
            .with_turboprops(turboprop_mounts)?;
        vehicle.bake_engine_masses()?;
        vehicle.bake_tank_masses()?;
        vehicle.bake_system_masses()?;
        vehicle.bake_jet_masses()?;
        vehicle.bake_electric_thruster_masses()?;
        vehicle.bake_fusion_masses()?;
        vehicle.bake_propeller_drive_masses()?;
        vehicle.bake_turboprop_masses()?;
        for (panel_index, joint) in parked_tags {
            vehicle.aero_geometry.panels[panel_index].fold_index = Some(joint);
        }
        // Single final recenter onto the assembly COM: every station
        // rides along, and the total inertia shifts by one parallel-axis
        // term. Doing it here (all masses attached) instead of on an
        // intermediate sum keeps every step positive-definite.
        let shift_point = |point: DVec3| point + shift;
        for panel in &mut vehicle.aero_geometry.panels {
            panel.position_body_m = shift_point(panel.position_body_m);
            panel.center_of_pressure_body_m = shift_point(panel.center_of_pressure_body_m);
        }
        for joint in &mut vehicle.fold_joints {
            joint.hinge_body_m = shift_point(joint.hinge_body_m);
        }
        for part in &mut vehicle.collision_geometry.parts {
            part.local_position_m = shift_point(part.local_position_m);
        }
        for mount in &mut vehicle.engines {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.tanks {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.systems {
            for chamber in &mut mount.system.chambers {
                shift_array(&mut chamber.position_body_m, shift);
            }
        }
        for mount in &mut vehicle.jets {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.electric_thrusters {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.fusion_torches {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.pulsed_fusion_systems {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.propeller_drives {
            shift_array(&mut mount.position_body_m, shift);
        }
        for mount in &mut vehicle.turboprops {
            shift_array(&mut mount.position_body_m, shift);
        }
        let total = vehicle.mass_properties.mass_kg;
        let recentered =
            vehicle.mass_properties.inertia_body_kg_m2 - parallel_axis(total, assembly_com);
        vehicle.mass_properties = RigidBodyProperties::new(total, recentered)?;
        vehicle.validate()?;
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
/// chamber/cycle fields. Optional `grain_geometry` selects star or finocyl
/// port burnback; omitted geometry is circular BATES:
///
/// ```text
/// grain_geometry = { kind = "star", tip_count = 6, tip_radius_m = 0.30 }
/// grain_geometry = { kind = "finocyl", fin_count = 8, fin_tip_radius_m = 0.32, fin_width_rad = 0.24 }
/// ```
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
    mixture_ratio: Option<f64>,
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
    segment_core_radii_m: Option<Vec<f64>>,
    #[serde(default)]
    grain_geometry: SolidGrainGeometry,
    #[serde(default)]
    burn_rate_coeff: Option<f64>,
    #[serde(default)]
    burn_rate_exponent: Option<f64>,
    #[serde(default = "default_one_shot")]
    ignition_shots: u32,
    // Nuclear thermal fields.
    #[serde(default)]
    fluid: Option<NtrFluid>,
    #[serde(default)]
    core_temp_k: Option<f64>,
    #[serde(default)]
    core_power_mw: Option<f64>,
    #[serde(default)]
    reactor_specific_mass_kg_per_mw: Option<f64>,
    #[serde(default)]
    startup_tau_s: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum EngineKind {
    Liquid,
    Solid,
    Nuclear,
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
            EngineKind::Nuclear => {
                let (engine, supplement) = self.nuclear_spec()?.compile()?;
                println!(
                    "engine {} (ntr): {:.0} MWt at {:.0} K, reactor {:.0} kg, startup {:.0} s",
                    self.name,
                    supplement.core_power_mw,
                    supplement.core_temp_k,
                    supplement.reactor_mass_kg,
                    supplement.startup_tau_s,
                );
                CompiledEngine::Liquid(engine)
            }
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
            mixture_ratio: self.mixture_ratio,
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
            grain_geometry: self.grain_geometry,
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
            segment_core_radii_m: self.segment_core_radii_m.clone(),
            gimbal_range_rad: self.gimbal_range_rad,
            ignition_shots: self.ignition_shots,
        })
    }

    fn nuclear_spec(&self) -> Result<NuclearThermalSpec, Box<dyn Error>> {
        Ok(NuclearThermalSpec {
            name: self.name.clone(),
            fluid: self.fluid.ok_or("nuclear engine needs fluid")?,
            core_temp_k: self.core_temp_k.ok_or("nuclear engine needs core_temp_k")?,
            core_power_mw: self
                .core_power_mw
                .ok_or("nuclear engine needs core_power_mw")?,
            reactor_specific_mass_kg_per_mw: self.reactor_specific_mass_kg_per_mw,
            throat_radius_m: self
                .throat_radius_m
                .ok_or("nuclear engine needs throat_radius_m")?,
            expansion_ratio: self
                .expansion_ratio
                .ok_or("nuclear engine needs expansion_ratio")?,
            nozzle_length_m: self
                .nozzle_length_m
                .ok_or("nuclear engine needs nozzle_length_m")?,
            contour: self.contour.unwrap_or(NozzleContour::Bell),
            material: self.material()?,
            cooling: self.cooling.unwrap_or(CoolingMode::Regenerative),
            gimbal_range_rad: self.gimbal_range_rad,
            startup_tau_s: self.startup_tau_s,
            min_throttle: self.min_throttle,
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

/// One propellant tank from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[tanks]]
/// name = "methane-tank"
/// shape = "cylinder"
/// diameter_m = 1.3
/// length_m = 3.0
/// pressure_mpa = 0.5
/// material = "regen-alloy"
/// position_body_m = [1.0, 0.0, 0.0]
/// propellant = "lox-methane"
/// ```
#[derive(Debug, Deserialize)]
struct TankAsset {
    name: String,
    shape: TankShapeAsset,
    diameter_m: f64,
    #[serde(default)]
    length_m: Option<f64>,
    pressure_mpa: f64,
    material: MaterialAsset,
    #[serde(default = "mount_position_default")]
    position_body_m: [f64; 3],
    /// Bulk density source: named propellant or an explicit value (one is
    /// required so tank fill mass stays physical).
    #[serde(default)]
    propellant: Option<Propellant>,
    #[serde(default)]
    bulk_density_kg_m3: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum TankShapeAsset {
    Sphere,
    Cylinder,
}

impl TankAsset {
    fn bake(self) -> Result<TankMount, Box<dyn Error>> {
        if self.name.trim().is_empty() {
            return Err("tank needs a name".into());
        }
        let shape = match self.shape {
            TankShapeAsset::Sphere => TankShape::Sphere {
                diameter_m: self.diameter_m,
            },
            TankShapeAsset::Cylinder => TankShape::Cylinder {
                diameter_m: self.diameter_m,
                length_m: self.length_m.ok_or("cylindrical tanks need length_m")?,
            },
        };
        if !self.pressure_mpa.is_finite() || self.pressure_mpa <= 0.0 {
            return Err("tank pressure_mpa must be finite and > 0".into());
        }
        let density = match (self.propellant, self.bulk_density_kg_m3) {
            (_, Some(density)) if density.is_finite() && density > 0.0 => density,
            (Some(propellant), None) => propellant.thermo().bulk_density_kg_m3,
            _ => {
                return Err(
                    format!("tank {} needs propellant or bulk_density_kg_m3", self.name).into(),
                );
            }
        };
        let spec = TankSpec {
            shape,
            pressure_pa: self.pressure_mpa * 1.0e6,
            material: match &self.material {
                MaterialAsset::Custom(material) => *material,
                MaterialAsset::Preset(name) => match name.as_str() {
                    "regen-alloy" => ChamberMaterial::regen_alloy(),
                    "nickel-superalloy" => ChamberMaterial::nickel_superalloy(),
                    "radiative-niobium" => ChamberMaterial::radiative_niobium(),
                    "ablative" => ChamberMaterial::ablative(),
                    unknown => {
                        return Err(format!("unknown material preset {unknown}").into());
                    }
                },
            },
        };
        Ok(TankMount {
            tank: spec.compile(density)?,
            position_body_m: self.position_body_m,
        })
    }
}

/// One chamber of a multi-chamber system from the source vehicle asset.
///
/// Example TOML:
///
/// ```text
/// [[systems]]
/// name = "quad"
/// propellant = "lox-rp1"
/// cycle = "gas-generator"
/// chamber_pressure_mpa = 9.7
/// material = "nickel-superalloy"
/// cooling = "regenerative"
///
/// [[systems.chambers]]
/// name = "a"
/// throat_radius_m = 0.134
/// expansion_ratio = 16.0
/// nozzle_length_m = 1.5
/// contour = "bell"
/// position_body_m = [-3.0, 0.0, 0.5]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// gimbal_range_rad = 0.09
/// ```
#[derive(Debug, Deserialize)]
struct ChamberAsset {
    name: String,
    throat_radius_m: f64,
    expansion_ratio: f64,
    nozzle_length_m: f64,
    #[serde(default = "bell_contour")]
    contour: NozzleContour,
    #[serde(default = "mount_position_default")]
    position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    #[serde(default)]
    gimbal_range_rad: f64,
}

fn bell_contour() -> NozzleContour {
    NozzleContour::Bell
}

impl ChamberAsset {
    fn bake(self) -> Result<ChamberSpec, Box<dyn Error>> {
        Ok(ChamberSpec {
            name: self.name,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            nozzle_length_m: self.nozzle_length_m,
            contour: self.contour,
            position_body_m: self.position_body_m,
            thrust_axis_body: self.thrust_axis_body,
            gimbal_range_rad: self.gimbal_range_rad,
        })
    }
}

/// One multi-chamber propulsion system: shared feed plus chamber list.
#[derive(Debug, Deserialize)]
struct SystemAsset {
    name: String,
    propellant: Propellant,
    cycle: EngineCycle,
    chamber_pressure_mpa: f64,
    #[serde(default)]
    mixture_ratio: Option<f64>,
    material: MaterialAsset,
    #[serde(default = "regen_cooling")]
    cooling: CoolingMode,
    #[serde(default)]
    characteristic_length_m: Option<f64>,
    #[serde(default)]
    min_throttle: Option<f64>,
    #[serde(default = "default_true")]
    restartable: bool,
    chambers: Vec<ChamberAsset>,
}

fn regen_cooling() -> CoolingMode {
    CoolingMode::Regenerative
}

impl SystemAsset {
    fn bake(self) -> Result<SystemMount, Box<dyn Error>> {
        if !self.chamber_pressure_mpa.is_finite() || self.chamber_pressure_mpa <= 0.0 {
            return Err("system needs chamber_pressure_mpa in MPa".into());
        }
        let material = match &self.material {
            MaterialAsset::Custom(material) => *material,
            MaterialAsset::Preset(name) => match name.as_str() {
                "regen-alloy" => ChamberMaterial::regen_alloy(),
                "nickel-superalloy" => ChamberMaterial::nickel_superalloy(),
                "radiative-niobium" => ChamberMaterial::radiative_niobium(),
                "ablative" => ChamberMaterial::ablative(),
                unknown => return Err(format!("unknown material preset {unknown}").into()),
            },
        };
        let chambers = self
            .chambers
            .into_iter()
            .map(ChamberAsset::bake)
            .collect::<Result<Vec<_>, _>>()?;
        let spec = PropulsionSystemSpec {
            name: self.name.clone(),
            propellant: self.propellant,
            cycle: self.cycle,
            chamber_pressure_pa: self.chamber_pressure_mpa * 1.0e6,
            mixture_ratio: self.mixture_ratio,
            chamber_material: material,
            cooling: self.cooling,
            characteristic_length_m: self.characteristic_length_m,
            min_throttle: self.min_throttle,
            restartable: self.restartable,
            chambers,
        };
        Ok(SystemMount {
            name: self.name,
            system: spec.compile()?,
        })
    }
}

/// One air-breathing jet or ESTOC from the source vehicle asset.
///
/// Example TOML (turbojet):
///
/// ```text
/// [[jets]]
/// name = "cruise-jet"
/// kind = "jet"
/// mount_position_body_m = [2.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// fuel = "kerosene"
/// intake_area_m2 = 0.5
/// intake = "pitot"
/// compressor_ratio = 8.0
/// turbine_inlet_temp_k = 1400.0
/// material = "nickel-superalloy"
/// ```
///
/// Example TOML (ESTOC adds the rocket block):
///
/// ```text
/// [[jets]]
/// name = "estoc-1"
/// kind = "estoc"
/// mount_position_body_m = [0.0, 0.0, 0.0]
/// thrust_axis_body = [1.0, 0.0, 0.0]
/// fuel = "kerosene"
/// intake_area_m2 = 0.9
/// intake = "pitot"
/// compressor_ratio = 12.0
/// turbine_inlet_temp_k = 1500.0
/// material = "nickel-superalloy"
/// rocket_chamber_pressure_mpa = 7.0
/// rocket_throat_radius_m = 0.09
/// ```
///
/// Optional shaft topology block (section 8.1): starter, generator, and
/// light-off/self-sustain thresholds. Omitted = inert default
/// (starterless, windmill/relight only):
///
/// ```text
/// [jets.shaft]
/// light_off_n = 0.15
/// self_sustain_n = 0.10
///
/// [jets.shaft.starter]
/// kind = "electric"       # "none" | "electric" | "pneumatic" | "rocket-bootstrap"
/// power_w = 200000.0
/// charge_j = 20000000.0
/// mass_kg = 12.0
///
/// [jets.shaft.generator]
/// fitted = true
/// power_w = 50000.0
/// efficiency = 0.92
/// cut_in_spool_n = 0.5
/// mass_kg = 25.0
/// ```
#[derive(Debug, Deserialize)]
struct JetAsset {
    name: String,
    kind: JetKind,
    /// Airbreathing cycle topology; omitted assets keep the legacy inference
    /// from bypass ratio (turbofan if positive, turbojet otherwise).
    #[serde(default)]
    cycle: Option<AirCycle>,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    #[serde(default)]
    gimbal_range_rad: f64,
    fuel: JetFuel,
    intake_area_m2: f64,
    #[serde(default = "pitot_intake")]
    intake: IntakeKind,
    #[serde(default = "default_compressor_ratio")]
    compressor_ratio: f64,
    #[serde(default)]
    bypass_ratio: f64,
    #[serde(default = "default_fan_ratio")]
    fan_pressure_ratio: f64,
    turbine_inlet_temp_k: f64,
    #[serde(default)]
    afterburner: bool,
    #[serde(default)]
    reheat_temp_k: f64,
    material: MaterialAsset,
    #[serde(default = "default_spool_tau")]
    spool_tau_s: f64,
    /// Shaft topology block (`[jets.shaft]`): starter, generator,
    /// light-off/self-sustain. Inert default when omitted.
    #[serde(default)]
    shaft: ShaftSpec,
    // ESTOC-only rocket block.
    #[serde(default)]
    rocket_chamber_pressure_mpa: Option<f64>,
    #[serde(default)]
    rocket_throat_radius_m: Option<f64>,
    #[serde(default)]
    oxidizer_fuel_ratio: Option<f64>,
    #[serde(default)]
    switch_mach_hi: Option<f64>,
    #[serde(default)]
    switch_mach_lo: Option<f64>,
    #[serde(default)]
    transition_tau_s: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum JetKind {
    Jet,
    Estoc,
}

fn pitot_intake() -> IntakeKind {
    IntakeKind::Pitot
}

fn default_compressor_ratio() -> f64 {
    8.0
}

fn default_fan_ratio() -> f64 {
    1.6
}

fn default_spool_tau() -> f64 {
    5.0
}

impl JetAsset {
    fn air_spec(&self) -> Result<AirbreathingSpec, Box<dyn Error>> {
        Ok(AirbreathingSpec {
            name: self.name.clone(),
            cycle: self.cycle.unwrap_or({
                if self.bypass_ratio > 0.0 {
                    AirCycle::Turbofan
                } else {
                    AirCycle::Turbojet
                }
            }),
            fuel: self.fuel,
            intake_area_m2: self.intake_area_m2,
            intake: self.intake,
            compressor_ratio: self.compressor_ratio,
            bypass_ratio: self.bypass_ratio,
            fan_pressure_ratio: self.fan_pressure_ratio,
            turbine_inlet_temp_k: self.turbine_inlet_temp_k,
            afterburner: self.afterburner,
            reheat_temp_k: self.reheat_temp_k,
            turbine_material: self.material()?,
            spool_tau_s: self.spool_tau_s,
            shaft: self.shaft.clone(),
        })
    }

    fn material(&self) -> Result<ChamberMaterial, Box<dyn Error>> {
        match &self.material {
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

    fn bake(self) -> Result<JetMount, Box<dyn Error>> {
        let engine = match self.kind {
            JetKind::Jet => CompiledJet::Air(Box::new(self.air_spec()?.compile()?)),
            JetKind::Estoc => {
                let spec = EstocSpec {
                    name: self.name.clone(),
                    air: self.air_spec()?,
                    rocket_chamber_pressure_pa: required_mpa(
                        self.rocket_chamber_pressure_mpa,
                        "rocket_chamber_pressure_mpa",
                    )?,
                    rocket_throat_radius_m: self
                        .rocket_throat_radius_m
                        .ok_or("estoc needs rocket_throat_radius_m")?,
                    oxidizer_fuel_ratio: self.oxidizer_fuel_ratio,
                    switch_mach_hi: self.switch_mach_hi,
                    switch_mach_lo: self.switch_mach_lo,
                    transition_tau_s: self.transition_tau_s,
                };
                let compiled = spec.compile()?;
                println!(
                    "jet {} (estoc): rocket {:.0} kN vac, shared expansion {:.1}x",
                    self.name,
                    compiled.rocket_thrust_vac_n / 1000.0,
                    compiled.rocket_expansion_ratio,
                );
                CompiledJet::Estoc(Box::new(compiled))
            }
        };
        Ok(JetMount {
            name: self.name,
            engine,
            position_body_m: self.mount_position_body_m,
            thrust_axis_body: self.thrust_axis_body,
            gimbal_range_rad: self.gimbal_range_rad,
        })
    }
}

/// Installed electric spacecraft thruster (`[[electric_thrusters]]`).
/// `design` is a tagged table selecting gridded-ion, Hall, MPD, resistojet,
/// or arcjet hardware; each compiled mount includes its power processor and
/// radiator mass.
#[derive(Debug, Deserialize)]
struct ElectricThrusterAsset {
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
    fn bake(self) -> Result<ElectricThrusterMount, Box<dyn Error>> {
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
struct FusionTorchAsset {
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
    fn bake(self) -> Result<FusionTorchMount, Box<dyn Error>> {
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
struct PulsedFusionAsset {
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
    fn bake(self) -> Result<PulsedFusionMount, Box<dyn Error>> {
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
struct PropellerDriveAsset {
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
    fn bake(self) -> Result<PropellerDriveMount, Box<dyn Error>> {
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
struct TurbopropAsset {
    name: String,
    #[serde(default = "mount_position_default")]
    mount_position_body_m: [f64; 3],
    #[serde(default = "thrust_axis_default")]
    thrust_axis_body: [f64; 3],
    drive: TurbopropDriveSpec,
}

impl TurbopropAsset {
    fn bake(self) -> Result<TurbopropMount, Box<dyn Error>> {
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

/// Parallel-axis term `m(|c|^2 I - c c^T)` for recentering inertia.
fn parallel_axis(mass_kg: f64, center: DVec3) -> DMat3 {
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

struct Options {
    input: PathBuf,
    output: Option<PathBuf>,
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
        "Usage: thessa-vehicle-baker [--input data/vehicles/example_aircraft.toml] [--output data/vehicles/example_aircraft.baked.json] [--analyze [--throttle 1.0] [--burn-time 0.0] [--analyze-json] [--composition N2/O2/AR/CO2] [--source-rpm 2400] [--power-takeoff-fraction 0.25]]"
    );
    println!(
        "--composition sets the analyzer atmosphere species (design string, default Thessa air N2/O2/AR/CO2; unknown gases are refused)."
    );
    println!("--source-rpm sets the steady shaft speed used by propeller-drive analyzer rows.");
    println!(
        "--power-takeoff-fraction requests this share of available turboprop shaft output [0, 1]."
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{
        AtmosphereComposition, CompiledShaftPowerSource, PropellerDriveCommand, TurbopropCommand,
    };

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
    fn electric_propeller_drive_bakes_mass_wrench_and_json() {
        let doc = r#"
name = "electric-prop-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[propeller_drives]]
name = "electric-cruise"
mount_position_body_m = [0.0, 1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reduction_ratio = 2.0

[propeller_drives.propeller]
diameter_m = 2.2
gearbox_efficiency = 0.96

[propeller_drives.source]
kind = "electric"

[propeller_drives.source.spec]
rated_power_w = 90000.0
peak_torque_nm = 420.0
maximum_rpm = 10000.0
efficiency = 0.92
cooling_capacity_w = 10000.0
dry_mass_kg = 32.0
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("drive TOML parses");
        let vehicle = asset.bake().expect("drive asset bakes");
        assert_eq!(vehicle.propeller_drives.len(), 1);
        assert!(matches!(
            vehicle.propeller_drives[0].drive.source,
            CompiledShaftPowerSource::Electric(_)
        ));
        assert!(vehicle.mass_properties.mass_kg > 1_032.0);

        let sample = AtmosphereConfig::default().sample(0.0).expect("atmosphere");
        let condition = thessa_sim_core::flight_condition(&sample, 60.0).expect("condition");
        let ((force, moment), points) = vehicle
            .propeller_drives_wrench_body_n(
                &[PropellerDriveCommand {
                    throttle: 1.0,
                    source_rpm: 6_000.0,
                }],
                &condition,
            )
            .expect("wrench");
        assert!(force.x > 0.0);
        assert!(moment.z.abs() > 0.0);
        assert_eq!(points.len(), 1);

        let json = serde_json::to_string(&vehicle).expect("vehicle JSON");
        let round_trip: VehicleDefinition = serde_json::from_str(&json).expect("JSON round-trip");
        assert_eq!(round_trip.propeller_drives.len(), 1);
        assert!(
            (round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs() < 1e-9
        );
    }
    #[test]
    fn turboprop_asset_bakes_mass_wrench_and_serialized_state() {
        let doc = r#"
name = "turboprop-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[turboprops]]
name = "left-turboprop"
mount_position_body_m = [0.0, 1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]

[turboprops.drive]
shaft_rpm_at_full_spool = 12000.0
reduction_ratio = 6.0
power_turbine_mass_kg = 40.0

[turboprops.drive.air]
name = "left-core"
cycle = "turbojet"
fuel = "kerosene"
intake_area_m2 = 0.8
intake = "pitot"
compressor_ratio = 8.0
bypass_ratio = 0.0
fan_pressure_ratio = 1.0
turbine_inlet_temp_k = 1400.0
afterburner = false
reheat_temp_k = 0.0
turbine_material = { density_kg_m3 = 8190.0, yield_strength_pa = 1000000000.0, max_wall_temp_k = 1350.0 }
spool_tau_s = 4.0

[turboprops.drive.air.shaft]
power_turbine_heat_fraction = 0.15

[turboprops.drive.propeller]
diameter_m = 2.4
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("turboprop TOML parses");
        let vehicle = asset.bake().expect("turboprop bakes");
        assert_eq!(vehicle.turboprops.len(), 1);
        assert!(vehicle.mass_properties.mass_kg > 1_040.0);

        let sample = AtmosphereConfig::default().sample(0.0).expect("atmosphere");
        let condition = thessa_sim_core::flight_condition(&sample, 0.0).expect("condition");
        let drive = &vehicle.turboprops[0].drive;
        let (_, balance) = drive
            .air
            .operating_point_at_spool_loaded(&condition, 1.0, 1.0, true, 0.0)
            .expect("takeoff capacity");
        let mut command = TurbopropCommand::running(drive);
        command.propeller_power_w = balance.power_takeoff_capacity_w * 0.1;
        let ((force, moment), next) = vehicle
            .turboprops_wrench_body_n_stateful(&[command], &condition)
            .expect("wrench");
        assert!(force.x > 0.0);
        assert!(moment.z.abs() > 0.0);
        assert_eq!(next.len(), 1);

        let json = serde_json::to_string(&vehicle).expect("serialize");
        let round_trip: VehicleDefinition = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(round_trip.turboprops.len(), 1);
        assert!(
            (round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs() < 1e-9
        );
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
        assert_eq!(vehicle.tanks.len(), 2);
        // Engine + tank masses aggregate on top of the 2000 kg structure.
        assert!(vehicle.mass_properties.mass_kg > 2000.0);
        let engines_mass: f64 = vehicle
            .engines
            .iter()
            .map(|mount| mount.engine.bake_mass_kg())
            .sum();
        let tanks_mass: f64 = vehicle
            .tanks
            .iter()
            .map(|mount| mount.tank.dry_mass_kg + mount.tank.full_propellant_kg)
            .sum();
        assert!(
            (vehicle.mass_properties.mass_kg - 2000.0 - engines_mass - tanks_mass).abs() < 1e-6,
            "baked mass must equal structure plus engines plus tanks"
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

    #[test]
    fn pressure_fed_demands_tank_pressure() {
        // A pressure-fed engine with only a 0.5 MPa tank must refuse: no
        // pump hides the shortfall. Pump-fed engines pass the same tanks.
        let doc = r#"
name = "fed-test"
mass_kg = 500.0
inertia_body_kg_m2 = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[engines]]
name = "fed"
kind = "liquid"
propellant = "lox-methane"
cycle = "pressure-fed"
chamber_pressure_mpa = 2.0
throat_radius_m = 0.05
expansion_ratio = 10.0
nozzle_length_m = 0.5
contour = "conical"
material = "regen-alloy"
cooling = "regenerative"
[[tanks]]
name = "weak-tank"
shape = "sphere"
diameter_m = 1.0
pressure_mpa = 0.5
material = "regen-alloy"
propellant = "lox-methane"
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
        assert!(
            asset.bake().is_err(),
            "0.5 MPa tank cannot pressure-feed a 2.4 MPa circuit"
        );
        let strong = doc.replace("pressure_mpa = 0.5", "pressure_mpa = 3.0");
        let asset: VehicleAsset = toml::from_str(&strong).expect("TOML parses");
        assert!(asset.bake().is_ok());
    }

    #[test]
    fn nuclear_and_rcs_assets_bake() {
        // NTR upper stage plus a hydrazine RCS block on one airframe.
        let doc = r#"
name = "ntr-test"
mass_kg = 8000.0
inertia_body_kg_m2 = [[20000.0, 0.0, 0.0], [0.0, 20000.0, 0.0], [0.0, 0.0, 8000.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[engines]]
name = "ntr-main"
kind = "nuclear"
mount_position_body_m = [-4.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fluid = "hydrogen"
core_temp_k = 2700.0
core_power_mw = 600.0
throat_radius_m = 0.09
expansion_ratio = 60.0
nozzle_length_m = 1.6
contour = "bell"
material = "nickel-superalloy"
cooling = "regenerative"
gimbal_range_rad = 0.05
[[engines]]
name = "rcs-a"
kind = "liquid"
mount_position_body_m = [2.0, 0.0, 1.0]
thrust_axis_body = [0.0, 0.0, -1.0]
propellant = "monoprop-hydrazine"
cycle = "pressure-fed"
chamber_pressure_mpa = 1.0
throat_radius_m = 0.002
expansion_ratio = 60.0
nozzle_length_m = 0.06
contour = "conical"
material = "regen-alloy"
cooling = "regenerative"
min_throttle = 1.0
[[tanks]]
name = "hydrazine-tank"
shape = "sphere"
diameter_m = 0.6
pressure_mpa = 2.0
material = "regen-alloy"
position_body_m = [1.0, 0.0, 0.0]
propellant = "monoprop-hydrazine"
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
        let vehicle = asset.bake().expect("NTR+RCS bakes");
        assert_eq!(vehicle.engines.len(), 2);
        // Reactor-dominated mass far above the 8 t structure.
        assert!(vehicle.mass_properties.mass_kg > 12_000.0);
        // NTR thrust clears 100 kN in vacuum; the RCS block is a small
        // transverse couple, not axial thrust.
        let ntr = vehicle
            .engine_thrust_body_n(0, 1.0, 0.0, 0.0)
            .expect("ntr thrust");
        assert!(ntr.x > 100_000.0);
        let (force, moment) = vehicle
            .wrench_body_n(&[(1.0, 0.0), (1.0, 0.0)], 0.0)
            .expect("wrench");
        assert!((force.x - ntr.x).abs() < 1.0, "axial thrust is the NTR");
        assert!(force.z.abs() < 100.0, "RCS fires transversely");
        assert!(moment.length() > 0.0, "offset RCS must couple");
    }

    #[test]
    fn twin_chamber_system_bakes_with_shared_feed() {
        // Two chambers on one GG feed: totals match the parts, system dry
        // mass lands in the baked vehicle mass, differential throttle
        // couples through the stations.
        let doc = r#"
name = "twin-test"
mass_kg = 2000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 1500.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[tanks]]
name = "rp1-tank"
shape = "sphere"
diameter_m = 2.0
pressure_mpa = 0.5
material = "regen-alloy"
position_body_m = [1.0, 0.0, 0.0]
propellant = "lox-rp1"
[[systems]]
name = "twin"
propellant = "lox-rp1"
cycle = "gas-generator"
chamber_pressure_mpa = 9.7
material = "nickel-superalloy"
cooling = "regenerative"
[[systems.chambers]]
name = "a"
throat_radius_m = 0.134
expansion_ratio = 16.0
nozzle_length_m = 1.5
contour = "bell"
position_body_m = [-3.0, 0.0, 0.5]
thrust_axis_body = [1.0, 0.0, 0.0]
gimbal_range_rad = 0.09
[[systems.chambers]]
name = "b"
throat_radius_m = 0.100
expansion_ratio = 16.0
nozzle_length_m = 1.2
contour = "bell"
position_body_m = [-3.0, 0.0, -0.5]
thrust_axis_body = [1.0, 0.0, 0.0]
gimbal_range_rad = 0.09
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
        let vehicle = asset.bake().expect("twin system bakes");
        assert_eq!(vehicle.systems.len(), 1);
        let system = &vehicle.systems[0].system;
        assert_eq!(system.chambers.len(), 2);
        // System dry (shared turbo booked once) sits inside baked mass.
        let tanks_mass: f64 = vehicle
            .tanks
            .iter()
            .map(|mount| mount.tank.dry_mass_kg + mount.tank.full_propellant_kg)
            .sum();
        assert!(
            (vehicle.mass_properties.mass_kg - 2000.0 - tanks_mass - system.dry_mass_kg).abs()
                < 1e-6
        );
        // Uniform full throttle matches the vacuum total; differential
        // throttle steers about Y.
        let total = vehicle.total_thrust_body_n(1.0, 0.0, 0.0).expect("total");
        assert!((total.x - system.total_thrust_vac_n).abs() / system.total_thrust_vac_n < 1e-9);
        let (force, moment) = vehicle
            .system_wrench_body_n(0, &[1.0, 0.5], 0.0)
            .expect("system wrench");
        let expected = system
            .operating_point(&[1.0, 0.5], 0.0)
            .expect("system point")
            .thrust_n;
        assert!((force.x - expected).abs() / expected < 1e-12);
        assert!(moment.y.abs() > 0.0, "differential must couple");
        assert!(vehicle.system_wrench_body_n(0, &[1.0], 0.0).is_err());
    }

    #[test]
    fn jet_and_estoc_assets_bake() {
        // Turbojet plus an ESTOC sharing the airframe: jet mass lands in
        // baked mass, static thrust is axial, ESTOC rocket branch compiles.
        let doc = r#"
name = "jet-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0
[[jets]]
name = "cruise-jet"
kind = "jet"
mount_position_body_m = [1.0, -1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fuel = "kerosene"
intake_area_m2 = 0.5
intake = "pitot"
compressor_ratio = 8.0
turbine_inlet_temp_k = 1400.0
material = "nickel-superalloy"
[[jets]]
name = "estoc-1"
kind = "estoc"
mount_position_body_m = [0.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fuel = "kerosene"
intake_area_m2 = 0.9
intake = "pitot"
compressor_ratio = 12.0
turbine_inlet_temp_k = 1500.0
material = "nickel-superalloy"
rocket_chamber_pressure_mpa = 7.0
rocket_throat_radius_m = 0.09
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
        let vehicle = asset.bake().expect("jets bake");
        assert_eq!(vehicle.jets.len(), 2);
        let jets_mass: f64 = vehicle
            .jets
            .iter()
            .map(|mount| mount.engine.dry_mass_kg())
            .sum();
        assert!(
            (vehicle.mass_properties.mass_kg - 3000.0 - jets_mass).abs() < 1e-6,
            "baked mass must equal structure plus jets"
        );
        assert!(vehicle.jets[1].engine.dry_mass_kg() > vehicle.jets[0].engine.dry_mass_kg());
    }

    #[test]
    fn shaped_solid_grain_profiles_are_authorable_in_toml() {
        let doc = r#"
name = "shaped-solid-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[engines]]
name = "star-booster"
kind = "solid"
outer_radius_m = 0.5
core_radius_m = 0.16
segment_length_m = 1.0
segments = 2
throat_radius_m = 0.12
expansion_ratio = 10.0
nozzle_length_m = 0.9
material = "nickel-superalloy"
grain_geometry = { kind = "star", tip_count = 6, tip_radius_m = 0.30 }

[[engines]]
name = "finocyl-sustainer"
kind = "solid"
outer_radius_m = 0.5
core_radius_m = 0.16
segment_length_m = 1.0
segments = 2
throat_radius_m = 0.12
expansion_ratio = 10.0
nozzle_length_m = 0.9
material = "nickel-superalloy"
grain_geometry = { kind = "finocyl", fin_count = 8, fin_tip_radius_m = 0.32, fin_width_rad = 0.24 }
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("shaped grain TOML parses");
        let vehicle = asset.bake().expect("shaped solid assets bake");
        assert_eq!(vehicle.engines.len(), 2);
        for (engine, expected_geometry) in [
            (
                &vehicle.engines[0].engine,
                SolidGrainGeometry::Star {
                    tip_count: 6,
                    tip_radius_m: 0.30,
                },
            ),
            (
                &vehicle.engines[1].engine,
                SolidGrainGeometry::Finocyl {
                    fin_count: 8,
                    fin_tip_radius_m: 0.32,
                    fin_width_rad: 0.24,
                },
            ),
        ] {
            let CompiledEngine::Solid(motor) = engine else {
                panic!("expected solid motor");
            };
            assert_eq!(motor.grain_geometry, expected_geometry);
            assert!(motor.burn_curve[0].burn_surface_area_m2 > 0.0);
        }
    }

    #[test]
    fn electric_space_thruster_is_authorable_and_bakes_wrench_mass_and_state() {
        let doc = r#"
name = "electric-spacecraft-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[electric_thrusters]]
name = "aft-ion"
mount_position_body_m = [-1.0, 0.8, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "xenon"
design = { kind = "gridded-ion", accelerator_voltage_v = 1000.0, grid_diameter_m = 0.4, grid_gap_m = 0.002, max_beam_current_density_a_m2 = 100.0, propellant_utilization = 0.95, accelerator_efficiency = 0.9 }
maximum_power_w = 5000.0
maximum_mass_flow_kg_s = 0.00001
power_processor_specific_power_w_kg = 2000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.003
radiator_area_m2 = 10.0
radiator_temperature_k = 700.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0
ionization_efficiency = 0.75
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("electric-thruster TOML parses");
        let vehicle = asset.bake().expect("electric-thruster asset bakes");
        assert_eq!(vehicle.electric_thrusters.len(), 1);
        assert!(vehicle.mass_properties.mass_kg > 3_000.0);
        assert!(vehicle.electric_thrusters[0].engine.dry_mass_kg > 0.0);
        let ((force, moment), points) = vehicle
            .electric_thrusters_wrench_body_n(&[thessa_sim_core::ElectricThrusterCommand {
                available_power_w: 5_000.0,
                requested_mass_flow_kg_s: 1.0e-6,
            }])
            .expect("mounted electric drive wrench");
        assert!(force.x > 0.0);
        assert!(moment.z < 0.0);
        assert_eq!(points.len(), 1);
        assert!(points[0].waste_heat_w <= points[0].radiator_capacity_w);
    }

    #[test]
    fn continuous_and_pulsed_fusion_mounts_are_authorable_and_recentered() {
        let doc = r#"
name = "fusion-spacecraft-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[fusion_torches]]
name = "dt-torch"
mount_position_body_m = [-2.0, 0.5, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reaction = "deuterium-tritium"
working_fluid = "hydrogen"
maximum_fusion_power_w = 100000000.0
fusion_gain = 10.0
maximum_working_flow_kg_s = 0.001
reactor_specific_power_w_kg = 10000.0
plasma_coupling_efficiency = 0.9
magnetic_nozzle_efficiency = 0.8
nozzle_radius_m = 0.5
nozzle_length_m = 2.0
magnetic_field_t = 1.0
coil_current_density_a_m2 = 40000000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.01
radiator_area_m2 = 3000.0
radiator_temperature_k = 1000.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0

[[pulsed_fusion_systems]]
name = "pellet-drive"
mount_position_body_m = [2.0, -0.5, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reaction = "deuterium-tritium"
working_fluid = "hydrogen"
fuel_mass_per_pulse_kg = 0.000000001
working_fluid_mass_per_pulse_kg = 0.0000001
fusion_gain = 10.0
plasma_coupling_efficiency = 0.9
magnetic_nozzle_efficiency = 0.8
maximum_pulse_frequency_hz = 0.1
pulse_duration_s = 0.01
maximum_charge_power_w = 100000.0
energy_buffer_capacity_pulses = 2
energy_buffer_specific_energy_j_kg = 1000000.0
pulse_system_specific_power_w_kg = 1000000.0
chamber_radius_m = 0.1
chamber_length_m = 0.5
magnetic_field_t = 1.0
coil_current_density_a_m2 = 40000000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.01
radiator_area_m2 = 10.0
radiator_temperature_k = 1000.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("fusion TOML parses");
        let vehicle = asset.bake().expect("fusion vehicle bakes");
        assert_eq!(vehicle.fusion_torches.len(), 1);
        assert_eq!(vehicle.pulsed_fusion_systems.len(), 1);
        assert!(vehicle.mass_properties.mass_kg > 3_000.0);
        let torch_mass = vehicle.fusion_torches[0].engine.dry_mass_kg;
        let pulse_mass = vehicle.pulsed_fusion_systems[0].engine.dry_mass_kg;
        let expected_shift = -(DVec3::new(-2.0, 0.5, 0.0) * torch_mass
            + DVec3::new(2.0, -0.5, 0.0) * pulse_mass)
            / (3_000.0 + torch_mass + pulse_mass);
        assert!(
            (DVec3::from_array(vehicle.fusion_torches[0].position_body_m)
                - (DVec3::new(-2.0, 0.5, 0.0) + expected_shift))
                .length()
                < 1e-10
        );
        assert!(
            (DVec3::from_array(vehicle.pulsed_fusion_systems[0].position_body_m)
                - (DVec3::new(2.0, -0.5, 0.0) + expected_shift))
                .length()
                < 1e-10
        );
        let total_first_moment = DVec3::from_array(vehicle.fusion_torches[0].position_body_m)
            * torch_mass
            + DVec3::from_array(vehicle.pulsed_fusion_systems[0].position_body_m) * pulse_mass
            + expected_shift * 3_000.0;
        assert!(total_first_moment.length() < 1e-7);

        let ((force, moment), points) = vehicle
            .fusion_torches_wrench_body_n(&[thessa_sim_core::FusionTorchCommand {
                available_driver_power_w: 20.0e6,
                requested_working_flow_kg_s: 1.0e-4,
            }])
            .expect("torch wrench");
        assert!(force.x > 0.0);
        assert!(moment.is_finite());
        assert_eq!(points.len(), 1);
        let pulse_mount = &vehicle.pulsed_fusion_systems[0];
        let ((pulse_force, pulse_moment), next) = vehicle
            .pulsed_fusion_wrench_body_n_stateful(
                &[(
                    thessa_sim_core::PulsedFusionState {
                        pulse_phase_s: pulse_mount.engine.pulse_interval_s - 1.0,
                        stored_driver_energy_j: pulse_mount.engine.driver_energy_per_pulse_j,
                        cumulative_shots: 0,
                    },
                    thessa_sim_core::PulsedFusionCommand {
                        available_charge_power_w: 100_000.0,
                        armed: true,
                    },
                )],
                1.0,
            )
            .expect("pulse wrench");
        assert!(pulse_force.x > 0.0);
        assert!(pulse_moment.is_finite());
        assert_eq!(next[0].1.pulses_fired, 1);
    }

    #[test]
    fn scramjet_cycle_is_authorable_and_analyzer_reaches_hypersonic_rows() {
        let doc = r#"
name = "scramjet-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[jets]]
name = "scramjet"
kind = "jet"
cycle = "scramjet"
fuel = "hydrogen"
intake_area_m2 = 0.5
intake = "ramp"
compressor_ratio = 1.0
turbine_inlet_temp_k = 2300.0
material = "nickel-superalloy"
"#;
        let asset: VehicleAsset = toml::from_str(doc).expect("scramjet TOML parses");
        let vehicle = asset.bake().expect("scramjet asset bakes");
        let engine = match &vehicle.jets[0].engine {
            CompiledJet::Air(engine) => engine.as_ref(),
            CompiledJet::Estoc(_) => panic!("scramjet is not ESTOC"),
        };
        assert_eq!(engine.cycle, AirCycle::Scramjet);
        assert_eq!(engine.shaft_reference_power_w, 0.0);

        let atmosphere = AtmosphereConfig::default();
        let rows =
            analyze_airbreathing(engine, &atmosphere, &[20_000.0], &[0.0, 1.0, 6.0, 8.0], 1.0)
                .expect("scramjet analyzer");
        assert_eq!(rows.len(), 4);
        assert!(rows[0].scramjet_limited);
        assert!(rows[1].scramjet_limited);
        assert!(!rows[2].scramjet_limited);
        assert!(rows[2].thrust_n > 0.0);
        assert!(rows[3].combustion_thermal_limited);
        assert!(!rows[3].drive_limited);
    }

    #[test]
    fn analyzer_options_preserve_explicit_composition() {
        let options = Options::parse(
            [
                "--analyze",
                "--composition",
                "N2/O2/AR/CO2",
                "--source-rpm",
                "2700",
                "--power-takeoff-fraction",
                "0.4",
            ]
            .into_iter()
            .map(String::from),
        )
        .expect("options parse");
        assert!(options.analyze);
        assert_eq!(options.composition, "N2/O2/AR/CO2");
        assert_eq!(options.source_rpm, 2700.0);
        assert_eq!(options.power_takeoff_fraction, 0.4);
        // Default is Thessa air, not a hard-coded Earth scalar.
        let default = Options::parse(std::iter::empty()).expect("defaults parse");
        assert_eq!(default.composition, "N2/O2/AR/CO2");
        assert_eq!(default.source_rpm, 2_400.0);
        assert_eq!(default.power_takeoff_fraction, 0.25);
        assert!(
            Options::parse(
                ["--power-takeoff-fraction", "1.1"]
                    .into_iter()
                    .map(String::from)
            )
            .is_err()
        );
        // Unknown species are refused instead of becoming Earth air.
        assert!(AtmosphereComposition::parse(&default.composition).is_ok());
        assert!(AtmosphereComposition::parse("XYZ").is_err());
    }
}

#[test]
fn procedural_surface_bakes_into_merged_panels_and_rebased_controls() {
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "procedural-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 2.0
chord_m = 1.0

[[control_surfaces]]
name = "hand-elevator"
panel_indices = [0]
minimum_deflection_rad = -0.4
maximum_deflection_rad = 0.4

[[procedural_surfaces]]
name = "wing-right"
span_m = 8.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 2.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 2.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "aileron"
span = [0.6, 0.9]
chord = [0.25, 1.0]
hinge_u = 0.25
min_deflection_rad = -0.35
max_deflection_rad = 0.35
"#,
    )
    .expect("procedural vehicle TOML should parse");
    let vehicle = asset.bake().expect("procedural asset should bake");
    // One hand panel plus the rectangular compiled wing (no features:
    // splits at 0.6/0.9 with one chord cut inside -> 4 zones).
    assert_eq!(vehicle.aero_geometry.panels.len(), 1 + 4);
    // Hand control keeps index 0; the compiled aileron rebases onto
    // the merged list and owns exactly its region panel.
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.control_surfaces[0].panel_indices, vec![0]);
    let aileron = &vehicle.control_surfaces[1];
    assert_eq!(aileron.name, "aileron");
    assert_eq!(aileron.panel_indices.len(), 1);
    assert!(aileron.panel_indices[0] >= 1);
    let owned = &vehicle.aero_geometry.panels[aileron.panel_indices[0]];
    assert!((owned.area_m2 - 16.0 * 0.3 * 0.75).abs() < 1e-9);
}

#[test]
fn full_procedural_aircraft_merges_wing_and_v_tail() {
    // Hangar-side full-vehicle assembly: a wing plus a canted V-tail
    // half (mount roll through TOML), each with its own controls.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "v-tail-test"
mass_kg = 500.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[procedural_surfaces]]
name = "wing-right"
span_m = 6.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 1.5
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 1.5

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "aileron"
span = [0.5, 0.9]
chord = [0.25, 1.0]
hinge_u = 0.25
min_deflection_rad = -0.4
max_deflection_rad = 0.4
kind = "TrailingEdgeDevice"

[[procedural_surfaces]]
name = "v-tail-right"
span_m = 2.0
origin_body_m = [-2.5, 0.0, 0.2]
mount_roll_rad = 0.7853981633974483

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 1.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 1.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "ruddervator"
span = [0.3, 0.9]
chord = [0.3, 1.0]
hinge_u = 0.3
min_deflection_rad = -0.4
max_deflection_rad = 0.4

[[procedural_surfaces.folds]]
name = "tip-fold"
station_s = 0.5
axis = [1.0, 0.0, 0.0]
deployed_angle_rad = 0.0
stowed_angle_rad = 0.6
travel_limit_rad = 0.7
deployment_rate_rad_s = 0.1
lock_window_rad = [-0.05, 0.05]
"#,
    )
    .expect("v-tail vehicle TOML should parse");
    let vehicle = asset.bake().expect("v-tail asset should bake");
    // Wing: splits at 0.5/0.9 with a chord cut inside -> 1 + 2 + 1.
    // V-tail: splits at 0.3/0.5/0.9 (fold at 0.5) with chord cuts
    // inside -> 1 + 2 + 2 + 1. Total 10 panels, 2 controls.
    assert_eq!(vehicle.aero_geometry.panels.len(), 10);
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.control_surfaces[0].name, "aileron");
    assert_eq!(vehicle.control_surfaces[1].name, "ruddervator");
    // The canted tail panels sit up-out of the body axis.
    let tail_panel = &vehicle.aero_geometry.panels[9];
    assert!(tail_panel.position_body_m.z > 0.5);
    assert!(tail_panel.position_body_m.y > 0.5);
    assert!(tail_panel.lift_axis_body.z > 0.5);
    // Mechanism metadata survives the bake: the fold joint merges
    // under a surface-qualified name, tail panels point at it, and
    // the ruddervator keeps its hinge marker with no parent.
    assert_eq!(vehicle.fold_joints.len(), 1);
    assert_eq!(vehicle.fold_joints[0].name, "v-tail-right.tip-fold");
    assert!((vehicle.fold_joints[0].angle_rad - 0.0).abs() < 1e-12);
    // Operating data rides along: rate, lock window, envelope gate.
    assert!((vehicle.fold_joints[0].deployment_rate_rad_s - 0.1).abs() < 1e-12);
    assert_eq!(vehicle.fold_joints[0].lock_window_rad, (-0.05, 0.05));
    assert_eq!(vehicle.fold_joints[0].max_dynamic_pressure_pa, None);
    let tagged = vehicle
        .aero_geometry
        .panels
        .iter()
        .filter(|panel| panel.fold_index == Some(0))
        .count();
    assert!(tagged > 0);
    assert!(
        vehicle.aero_geometry.panels[..4]
            .iter()
            .all(|panel| panel.fold_index.is_none())
    );
    assert_eq!(
        vehicle.control_surfaces[1].kind,
        thessa_sim_core::ControlKind::Hinge
    );
    assert_eq!(vehicle.control_surfaces[1].parent_index, None);
    // Contact boxes: wing plain plus aileron regions, tail plain,
    // ruddervator, folded, and folded-ruddervator regions.
    assert_eq!(vehicle.collision_geometry.parts.len(), 6);
    assert!(vehicle.collision_geometry.validate().is_ok());
}

#[test]
fn structured_surface_aggregates_mass_and_inertia() {
    // Hand mass 1000 kg plus an 8x2 aluminum wing (264.3648 kg
    // pinned in-crate); inertia sums about the body origin.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "structured-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[procedural_surfaces]]
name = "wing"
span_m = 8.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 2.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 2.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.10
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.10

[procedural_surfaces.structure]
skin_gauge_mm = 2.0
spar_depth_fraction = 0.6
spar_web_gauge_mm = 3.0
rib_spacing_m = 0.5
rib_gauge_mm = 1.0
design_limit_lift_n = 12000.0
fuel_box_chord = [0.15, 0.65]
fuel_sump_fraction = 0.03

[procedural_surfaces.structure.skin_material]
name = "Al-7075-T6"
density_kg_m3 = 2810.0
allowable_stress_mpa = 503.0

[procedural_surfaces.structure.spar_material]
name = "Al-7075-T6"
density_kg_m3 = 2810.0
allowable_stress_mpa = 503.0
"#,
    )
    .expect("structured vehicle TOML should parse");
    let vehicle = asset.bake().expect("structured asset should bake");
    // Single-zone wing: every identity below is exact, recomputed
    // from measured values rather than hand arithmetic.
    assert_eq!(vehicle.aero_geometry.panels.len(), 1);
    let wing_mass = vehicle.mass_properties.mass_kg - 1000.0;
    assert!((240.0..260.0).contains(&wing_mass));
    let total = vehicle.mass_properties.mass_kg;
    let com = DVec3::new(-wing_mass / total, 4.0 * wing_mass / total, 0.0);
    assert!(
        (vehicle.aero_geometry.panels[0].center_of_pressure_body_m
            - (DVec3::new(-1.0, 4.0, 0.0) - com))
            .length()
            < 1e-6
    );
    let expected_xy = wing_mass * 1.0 * 4.0 + total * com.x * com.y;
    assert!((vehicle.mass_properties.inertia_body_kg_m2.x_axis.y - expected_xy).abs() < 1e-3);
}

#[test]
fn engine_mass_joins_single_final_com() {
    // Hand 1000 kg at the origin plus one engine at x = 10 m: the
    // reviewer's trap (COM must land at 100*10/1100, not zero).
    // Engine mass is measured back from the baked mount; the shift,
    // recenter, and bake order are what this pins.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "engine-com-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[engines]]
name = "main"
kind = "liquid"
mount_position_body_m = [10.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "lox-methane"
cycle = "gas-generator"
chamber_pressure_mpa = 12.0
throat_radius_m = 0.15
expansion_ratio = 35.0
nozzle_length_m = 1.8
contour = "bell"
material = "nickel-superalloy"
"#,
    )
    .expect("engine TOML should parse");
    let vehicle = asset.bake().expect("engine asset should bake");
    let engine_mass = vehicle.engines[0].engine.bake_mass_kg();
    assert!(engine_mass > 0.0);
    let total = 1000.0 + engine_mass;
    let com_x = 10.0 * engine_mass / total;
    // Engine station rides the shift; the hand panel at the origin
    // moves to minus the assembly COM.
    assert!((vehicle.engines[0].position_body_m[0] - (10.0 - com_x)).abs() < 1e-9);
    assert!(
        (vehicle.aero_geometry.panels[0].position_body_m - DVec3::new(-com_x, 0.0, 0.0)).length()
            < 1e-9
    );
    // Inertia: hand plus engine point term about the authoring
    // station, minus the single total parallel-axis shift.
    let hand = 1000.0;
    let expected_yy = hand + engine_mass * 10.0 * 10.0 - total * com_x * com_x;
    assert!((vehicle.mass_properties.mass_kg - total).abs() < 1e-9);
    assert!(
        (vehicle.mass_properties.inertia_body_kg_m2.y_axis.y - expected_yy).abs()
            < 1e-6 * expected_yy.abs().max(1.0)
    );
}

#[test]
fn dbg_engine_com() {
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "engine-com-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[engines]]
name = "main"
kind = "liquid"
mount_position_body_m = [10.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "lox-methane"
cycle = "gas-generator"
chamber_pressure_mpa = 12.0
throat_radius_m = 0.15
expansion_ratio = 35.0
nozzle_length_m = 1.8
contour = "bell"
material = "nickel-superalloy"
"#,
    )
    .expect("parse");
    eprintln!("engines={}", asset.engines.len());
}
