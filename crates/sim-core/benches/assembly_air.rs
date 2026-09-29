//! Runtime cost of resolving connected cabin pressure domains on a
//! representative 64-part craft. Run with
//! `cargo bench -p thessa-sim-core --bench assembly_air`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::*;

const BODIES: usize = 64;
const ITERATIONS: usize = 20_000;
const BREAKUP_ITERATIONS: usize = 2_000;
const DEFINITION_BODIES: usize = 4;
const DEFINITION_PANELS_PER_BODY: usize = 16;
const DEFINITION_ITERATIONS: usize = 500;

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
                feed_line: None,
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
        resource_edges: Vec::new(),
        joint_strengths: Vec::new(),
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

fn breakup_mass_fixture() -> (Vec<AssemblyBodyMassProperties>, RigidBodyProperties) {
    let bodies = (0..BODIES)
        .map(|index| AssemblyBodyMassProperties {
            mass_kg: 10.0,
            center_of_mass_body_m: DVec3::X * (index as f64 - (BODIES - 1) as f64 / 2.0),
            inertia_about_center_body_kg_m2: DMat3::from_diagonal(DVec3::splat(10.0)),
        })
        .collect::<Vec<_>>();
    let mass_kg = bodies.iter().map(|body| body.mass_kg).sum::<f64>();
    let inertia = bodies.iter().fold(DMat3::ZERO, |inertia, body| {
        let center = body.center_of_mass_body_m;
        let outer = DMat3::from_cols(center * center.x, center * center.y, center * center.z);
        inertia
            + body.inertia_about_center_body_kg_m2
            + (DMat3::IDENTITY * center.length_squared() - outer) * body.mass_kg
    });
    (
        bodies,
        RigidBodyProperties::new(mass_kg, inertia).expect("valid breakup mass fixture"),
    )
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

    let assembly = vehicle.assembly.as_ref().expect("assembly topology");
    let (body_mass_properties, source_mass_properties) = breakup_mass_fixture();
    let source_state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::ZERO,
        glam::DQuat::IDENTITY,
        DVec3::new(0.0, 0.0, 0.01),
    )
    .expect("valid source state");
    let start = Instant::now();
    for _ in 0..BREAKUP_ITERATIONS {
        black_box(
            assembly
                .reconstruct_clusters_after_link_failure(
                    "hatch-31",
                    &body_mass_properties,
                    source_mass_properties,
                    source_state,
                )
                .expect("conservative cluster reconstruction"),
        );
    }
    let elapsed = start.elapsed();
    println!(
        "assembly breakup reconstruction: {BODIES} bodies x {BREAKUP_ITERATIONS} reconstructions: {elapsed:?} total, {:.2} us/reconstruction",
        elapsed.as_secs_f64() * 1.0e6 / BREAKUP_ITERATIONS as f64,
    );

    let (split_vehicle, split_state) = split_definitions_fixture();
    // Sanity before timing: the middle link yields two validating clusters.
    assert_eq!(
        split_vehicle
            .split_definitions_after_link_failure("link-1", split_state)
            .expect("fixture splits")
            .len(),
        2
    );
    let start = Instant::now();
    for _ in 0..DEFINITION_ITERATIONS {
        black_box(
            split_vehicle
                .split_definitions_after_link_failure("link-1", split_state)
                .expect("cluster definition split"),
        );
    }
    let elapsed = start.elapsed();
    println!(
        "assembly definition split: {DEFINITION_BODIES} bodies x {} panels x {DEFINITION_ITERATIONS} splits: {elapsed:?} total, {:.2} us/split",
        DEFINITION_BODIES * DEFINITION_PANELS_PER_BODY,
        elapsed.as_secs_f64() * 1.0e6 / DEFINITION_ITERATIONS as f64,
    );
}

fn split_definitions_fixture() -> (VehicleDefinition, RigidBodyState) {
    let mut panels = Vec::new();
    let mut panel_bodies = Vec::new();
    let mut parts = Vec::new();
    let mut collision_bodies = Vec::new();
    let mut body_names = Vec::new();
    let mut links = Vec::new();
    let mut body_masses = Vec::new();
    let mut total_mass = 0.0;
    for body in 0..DEFINITION_BODIES {
        body_names.push(format!("part-{body}"));
        let station = DVec3::X * body as f64;
        for _ in 0..DEFINITION_PANELS_PER_BODY {
            panels.push(AeroPanel::new(station, DVec3::X, DVec3::Z, 1.0, 1.0).expect("panel"));
            panel_bodies.push(body);
        }
        parts.push(
            CollisionPart::new(
                station,
                glam::DQuat::IDENTITY,
                CollisionShape::Sphere { radius_m: 0.1 },
                CollisionMaterial::default(),
            )
            .expect("contact"),
        );
        collision_bodies.push(body);
        if body > 0 {
            links.push(NamedAssemblyLink {
                name: format!("link-{}", body - 1),
                state: AssemblyLinkState {
                    a: body - 1,
                    b: body,
                    hatch: false,
                    open: true,
                },
                feed_line: None,
            });
        }
        let mass = AssemblyBodyMassProperties {
            mass_kg: 10.0,
            center_of_mass_body_m: station,
            inertia_about_center_body_kg_m2: DMat3::from_diagonal(DVec3::splat(1.0)),
        };
        body_masses.push(mass);
        total_mass += mass.mass_kg;
    }
    // Source COM sits at the body-mean station; shift the source frame so
    // the conservation gate sees a centered vehicle.
    let source_com = DVec3::X * (DEFINITION_BODIES as f64 - 1.0) / 2.0;
    for panel in &mut panels {
        panel.position_body_m -= source_com;
        panel.center_of_pressure_body_m -= source_com;
    }
    for part in &mut parts {
        part.local_position_m -= source_com;
    }
    for mass in &mut body_masses {
        mass.center_of_mass_body_m -= source_com;
    }
    let mut recentered_inertia = DMat3::ZERO;
    for mass in &body_masses {
        let center = mass.center_of_mass_body_m;
        let outer = DMat3::from_cols(center * center.x, center * center.y, center * center.z);
        recentered_inertia += mass.inertia_about_center_body_kg_m2
            + (DMat3::IDENTITY * center.length_squared() - outer) * mass.mass_kg;
    }
    let geometry = AeroGeometry::new(panels).expect("geometry");
    let properties = RigidBodyProperties::new(total_mass, recentered_inertia).expect("source mass");
    let collision_geometry = CollisionGeometry::new(parts).expect("collision");
    let mut vehicle = VehicleDefinition::new("split-bench", geometry, properties, Vec::new())
        .expect("vehicle")
        .with_collision_geometry(collision_geometry)
        .expect("collision")
        .with_assembly(VehicleAssembly {
            root_body: 0,
            body_names,
            links,
            resource_edges: Vec::new(),
            joint_strengths: Vec::new(),
            volumes: Vec::new(),
            tanks: Vec::new(),
            engine_ports: Vec::new(),
        })
        .expect("assembly");
    vehicle.assembly_ownership = Some(AssemblyOwnership {
        panel_bodies,
        collision_bodies,
        cabin_bodies: Vec::new(),
        cabin_exit_bodies: Vec::new(),
        cabin_seat_bodies: Vec::new(),
        cabin_monument_bodies: Vec::new(),
        control_core_bodies: Vec::new(),
        control_station_bodies: Vec::new(),
        engine_bodies: Vec::new(),
        tank_bodies: Vec::new(),
        system_bodies: Vec::new(),
        jet_bodies: Vec::new(),
        rcs_bodies: Vec::new(),
        heat_shield_bodies: Vec::new(),
        body_masses,
    });
    vehicle.validate().expect("fixture validates");
    let state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::ZERO,
        glam::DQuat::IDENTITY,
        DVec3::new(0.0, 0.0, 0.01),
    )
    .expect("valid source state");
    (vehicle, state)
}
