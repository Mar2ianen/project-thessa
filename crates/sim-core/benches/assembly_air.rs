//! Runtime cost of resolving connected cabin pressure domains on a
//! representative 64-part craft. Run with
//! `cargo bench -p thessa-sim-core --bench assembly_air`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::*;

const BODIES: usize = 64;
const ITERATIONS: usize = 20_000;

fn make_vehicle() -> VehicleDefinition {
    let mut cabins = Vec::with_capacity(BODIES);
    let mut volumes = Vec::with_capacity(BODIES);
    let mut links = Vec::with_capacity(BODIES - 1);
    let body_names: Vec<String> = (0..BODIES).map(|index| format!("part-{index}")).collect();

    for (index, body_name) in body_names.iter().enumerate() {
        let volume_m3 = 3.0 + (index % 5) as f64;
        let temp_k = 260.0 + (index % 31) as f64;
        let pressure_kpa = 70.0 + (index % 7) as f64 * 10.0;
        let name = format!("{body_name}.cabin");
        let cabin = PressurizedCabin::new(
            name.clone(),
            volume_m3,
            pressure_kpa,
            temp_k,
            0.16 + (index % 8) as f64 * 0.01,
            pressure_kpa * 1000.0 / (R_DRY_AIR_J_KG_K * temp_k) * volume_m3,
        )
        .expect("valid pressure volume");
        cabins.push(cabin);
        volumes.push(AssemblyVolume {
            name,
            body: index,
            pressurized: true,
            volume_m3,
            centroid_body_m: DVec3::ZERO,
            seats: 0,
            seat_positions_body_m: Vec::new(),
        });
        if index > 0 {
            links.push(NamedAssemblyLink {
                name: format!("hatch-{}", index - 1),
                state: AssemblyLinkState {
                    a: index - 1,
                    b: index,
                    hatch: true,
                    open: true,
                },
            });
        }
    }
    let geometry = AeroGeometry::new(vec![
        AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"),
    ])
    .expect("geometry");
    let mass = RigidBodyProperties::new(10_000.0, DMat3::from_diagonal(DVec3::splat(5_000.0)))
        .expect("mass");
    let assembly = VehicleAssembly {
        root_body: 0,
        body_names,
        links,
        volumes,
        tanks: Vec::new(),
        engine_ports: Vec::new(),
    };
    VehicleDefinition::new("assembly-air-bench", geometry, mass, Vec::new())
        .expect("vehicle")
        .with_cabins(cabins)
        .expect("cabins")
        .with_assembly(assembly)
        .expect("assembly")
}

fn main() {
    let mut vehicle = make_vehicle();
    let start = Instant::now();
    for step in 0..ITERATIONS {
        for (index, cabin) in vehicle.cabins.iter_mut().enumerate() {
            let fill = 0.65 + ((index + step) % 30) as f64 * 0.01;
            cabin.air_kg = cabin.full_charge_kg() * fill;
        }
        // Seed a self-consistent pre-transition mass state after authoring
        // each inventory distribution. All benchmark cabin centroids are at
        // the origin, so their point-mass inertia contribution is zero.
        vehicle.mass_properties.mass_kg =
            10_000.0 + vehicle.cabins.iter().map(|cabin| cabin.air_kg).sum::<f64>();
        vehicle
            .equalize_assembly_air_domains()
            .expect("valid cabin state");
        black_box(&vehicle.cabins);
    }
    let elapsed = start.elapsed();
    println!(
        "assembly cabin pressure transition: {BODIES} cabins x {ITERATIONS} transitions: {elapsed:?} total, {:.2} us/transition",
        elapsed.as_secs_f64() * 1.0e6 / ITERATIONS as f64,
    );
}
