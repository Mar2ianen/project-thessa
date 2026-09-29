//! Static pressure-limited feed allocation for one representative 64-part
//! vehicle. Run with `cargo bench -p thessa-sim-core --bench resource_routing`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::*;

const BODIES: usize = 64;
const TANKS: usize = 32;
const CONSUMERS: usize = 16;
const ITERATIONS: usize = 1_000;

fn make_vehicle() -> VehicleDefinition {
    let material = ChamberMaterial::nickel_superalloy();
    let line = FeedLine {
        diameter_m: 0.012,
        length_m: 1.0,
        bends: 1,
        rated_pressure_pa: 1.0e6,
        material,
    };
    let names: Vec<String> = (0..BODIES).map(|index| format!("part-{index}")).collect();
    let mut links = Vec::with_capacity(BODIES - 1);
    for index in 1..BODIES {
        links.push(NamedAssemblyLink {
            name: format!("joint-{}-{index}", index - 1),
            state: AssemblyLinkState {
                a: index - 1,
                b: index,
                hatch: false,
                open: true,
            },
            feed_line: Some(line),
        });
    }

    let resource = StoredPropellant::Nitrogen;
    let tank_spec = TankSpec {
        shape: TankShape::Sphere { diameter_m: 0.1 },
        pressure_pa: 1.0e6,
        material,
    };
    let compiled_tank = tank_spec.compile(1_000.0).expect("tank compiles");
    let mut tanks = Vec::with_capacity(TANKS);
    let mut tank_endpoints = Vec::with_capacity(TANKS);
    for index in 0..TANKS {
        let name = format!("part-{index}.tank");
        tanks.push(TankMount {
            name: name.clone(),
            tank: compiled_tank,
            position_body_m: [0.0; 3],
            intrinsic_inertia_body_kg_m2: DMat3::ZERO,
            initial_propellant_kg: None,
            resource: TankResource::Stored(resource),
        });
        tank_endpoints.push(AssemblyEndpoint { name, body: index });
    }

    let feed_port_name = format!("part-{}.engine", BODIES - 1);
    let assembly = VehicleAssembly {
        root_body: 0,
        body_names: names,
        links,
        resource_edges: Vec::new(),
        joint_strengths: Vec::new(),
        volumes: Vec::new(),
        tanks: tank_endpoints,
        engine_ports: vec![AssemblyEndpoint {
            name: feed_port_name.clone(),
            body: BODIES - 1,
        }],
    };
    let fluid = FeedResourceProperties {
        resource,
        density_kg_m3: 1_000.0,
        viscosity_pa_s: 0.001,
        source_pressure_pa: 800_000.0,
        minimum_pressure_pa: 100_000.0,
    };
    let routes = (0..CONSUMERS)
        .map(|index| VehicleResourceFeedPort {
            consumer_name: format!("consumer-{index}"),
            feed_port_name: feed_port_name.clone(),
            fluid_properties: vec![fluid],
        })
        .collect();

    VehicleDefinition::new(
        "resource-routing-bench",
        AeroGeometry::default(),
        RigidBodyProperties::new(10_000.0, DMat3::from_diagonal(DVec3::splat(5_000.0)))
            .expect("mass properties"),
        Vec::new(),
    )
    .expect("vehicle")
    .with_tanks(tanks)
    .expect("tank mounts")
    .with_assembly(assembly)
    .expect("assembly")
    .with_resource_feed_ports(routes)
    .expect("feed routes")
}

fn main() {
    let vehicle = make_vehicle();
    let state = vehicle.initial_resource_state();
    let feed_port_name = format!("part-{}.engine", BODIES - 1);
    let demands: Vec<_> = (0..CONSUMERS)
        .map(|index| {
            VehicleResourceDemand::new(
                format!("consumer-{index}"),
                StoredPropellant::Nitrogen,
                0.05,
            )
            .through_feed_port(feed_port_name.clone())
        })
        .collect();

    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let plan = vehicle
            .plan_resource_flows(&state, &demands, 0.1)
            .expect("pressure-limited flow plan");
        black_box(plan);
    }
    let elapsed = start.elapsed();
    println!(
        "pressure-limited routing: {BODIES} bodies, {TANKS} tanks, {CONSUMERS} consumers, {} feed-line links: {elapsed:?} total, {:.2} us/plan",
        BODIES - 1,
        elapsed.as_secs_f64() * 1.0e6 / ITERATIONS as f64,
    );
}
