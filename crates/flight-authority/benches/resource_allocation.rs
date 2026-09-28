use std::hint::black_box;
use std::time::Instant;

use glam::{DMat3, DVec3};
use thessa_sim_core::{
    AeroGeometry, AeroPanel, ChamberMaterial, CoolingMode, EngineCycle, EngineMount,
    LiquidEngineSpec, NozzleContour, Propellant, RigidBodyProperties, TankMount, TankResource,
    TankShape, TankSpec, VehicleDefinition,
};

const ENGINES: usize = 32;
const TANKS: usize = 64;
const ITERATIONS: usize = 20_000;
const REPETITIONS: usize = 5;
const STEP_S: f64 = 1.0 / 120.0;

fn workload()
-> Result<(VehicleDefinition, thessa_sim_core::VehicleResourceState), Box<dyn std::error::Error>> {
    let panel = AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0)?;
    let geometry = AeroGeometry::new(vec![panel])?;
    let properties = RigidBodyProperties::new(10_000.0, DMat3::IDENTITY * 1.0e6)?;
    let engine = thessa_sim_core::CompiledEngine::Liquid(
        LiquidEngineSpec {
            name: "allocator-benchmark".into(),
            propellant: Propellant::LoxMethane,
            cycle: EngineCycle::GasGenerator,
            chamber_pressure_pa: 9.7e6,
            throat_radius_m: 0.08,
            expansion_ratio: 18.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Bell,
            chamber_material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            mixture_ratio: None,
            characteristic_length_m: None,
            gimbal_range_rad: 0.0,
            min_throttle: None,
            restartable: true,
        }
        .compile()?,
    );
    let engines = (0..ENGINES)
        .map(|index| EngineMount {
            name: format!("engine-{index}"),
            engine: engine.clone(),
            position_body_m: [0.0; 3],
            thrust_axis_body: [1.0, 0.0, 0.0],
        })
        .collect();
    let shape = TankShape::Sphere { diameter_m: 0.8 };
    let compiled_tank = TankSpec {
        shape,
        pressure_pa: 500_000.0,
        material: ChamberMaterial::nickel_superalloy(),
    }
    .compile(422.0)?;
    let tank_inertia = shape.intrinsic_inertia_body_kg_m2(
        compiled_tank.dry_mass_kg,
        compiled_tank.full_propellant_kg,
    )?;
    let tanks = (0..TANKS)
        .map(|index| TankMount {
            name: format!("tank-{index}"),
            tank: compiled_tank,
            position_body_m: [0.0; 3],
            intrinsic_inertia_body_kg_m2: tank_inertia,
            initial_propellant_kg: Some(compiled_tank.full_propellant_kg),
            resource: TankResource::Pair(Propellant::LoxMethane),
        })
        .collect();
    let mut vehicle = VehicleDefinition::new(
        "resource-allocation-benchmark",
        geometry,
        properties,
        vec![],
    )?
    .with_engines(engines)?
    .with_tanks(tanks)?;
    vehicle.bake_engine_masses()?;
    vehicle.bake_tank_masses()?;
    let state = vehicle.initial_resource_state();
    Ok((vehicle, state))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (vehicle, state) = workload()?;
    let engine_throttles = vec![1.0; ENGINES];
    let system_throttles = [];
    for _ in 0..REPETITIONS {
        let started = Instant::now();
        for _ in 0..ITERATIONS {
            black_box(vehicle.plan_propulsion_step(
                &state,
                &engine_throttles,
                &system_throttles,
                101_325.0,
                STEP_S,
            )?);
        }
        let elapsed = started.elapsed();
        println!(
            "{ENGINES} engines, {TANKS} tanks: {:.1} ns / fixed-step allocation",
            elapsed.as_nanos() as f64 / ITERATIONS as f64
        );
    }
    Ok(())
}
