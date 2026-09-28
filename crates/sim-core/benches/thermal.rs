//! Lumped-thermal runtime cost for a 64-vessel fleet: four nodes, three
//! links, two radiators, one sun, entry-interface airflow, and a reactor
//! waste-heat load per vessel. Run with
//! `cargo bench -p thessa-sim-core --bench thermal`.

use std::{hint::black_box, time::Instant};

use glam::DVec3;
use thessa_sim_core::*;

const VESSELS: usize = 64;
const ITERATIONS: usize = 10_000;

fn make_system(index: usize) -> ThermalSystem {
    let hull = ThermalNodeSpec {
        name: format!("hull-{index}"),
        mass_kg: 400.0,
        specific_heat_j_kg_k: 900.0,
        initial_temp_k: 280.0,
        max_temp_k: 450.0,
        emissivity: 0.8,
        solar_absorptivity: 0.5,
        radiating_area_m2: 6.0,
        solar_exposed_area_m2: 2.0,
        solar_normal_body: DVec3::Z,
        aero_area_m2: 1.5,
        nose_radius_m: 1.2,
        position_body_m: DVec3::new(-0.5, 0.0, 0.0),
    };
    let block = ThermalNodeSpec {
        name: format!("reactor-{index}"),
        mass_kg: 420.0,
        specific_heat_j_kg_k: 600.0,
        initial_temp_k: 300.0,
        max_temp_k: 900.0,
        emissivity: 0.7,
        solar_absorptivity: 0.4,
        radiating_area_m2: 3.0,
        solar_exposed_area_m2: 0.0,
        solar_normal_body: DVec3::Z,
        aero_area_m2: 0.0,
        nose_radius_m: 1.0,
        position_body_m: DVec3::new(0.8, 0.0, -0.4),
    };
    let avionics = ThermalNodeSpec {
        name: format!("avionics-{index}"),
        mass_kg: 60.0,
        specific_heat_j_kg_k: 900.0,
        initial_temp_k: 285.0,
        max_temp_k: 380.0,
        emissivity: 0.85,
        solar_absorptivity: 0.6,
        radiating_area_m2: 1.0,
        solar_exposed_area_m2: 0.4,
        solar_normal_body: DVec3::Y,
        aero_area_m2: 0.2,
        nose_radius_m: 0.5,
        position_body_m: DVec3::new(-0.2, 0.3, 0.1),
    };
    let tank = ThermalNodeSpec {
        name: format!("tank-{index}"),
        mass_kg: 200.0,
        specific_heat_j_kg_k: 1_200.0,
        initial_temp_k: 270.0,
        max_temp_k: 400.0,
        emissivity: 0.6,
        solar_absorptivity: 0.3,
        radiating_area_m2: 4.0,
        solar_exposed_area_m2: 1.2,
        solar_normal_body: DVec3::NEG_Z,
        aero_area_m2: 0.8,
        nose_radius_m: 1.0,
        position_body_m: DVec3::new(0.1, -0.3, 0.0),
    };
    ThermalSystem {
        nodes: vec![hull, block, avionics, tank],
        links: vec![
            ThermalLinkSpec {
                node_a: format!("hull-{index}"),
                node_b: format!("reactor-{index}"),
                conductance_w_k: 25.0,
            },
            ThermalLinkSpec {
                node_a: format!("hull-{index}"),
                node_b: format!("avionics-{index}"),
                conductance_w_k: 40.0,
            },
            ThermalLinkSpec {
                node_a: format!("hull-{index}"),
                node_b: format!("tank-{index}"),
                conductance_w_k: 15.0,
            },
        ],
        radiators: vec![
            RadiatorSpec {
                name: format!("rad-a-{index}"),
                attached_node: format!("reactor-{index}"),
                area_m2: 8.0,
                emissivity: 0.9,
                solar_absorptivity: 0.2,
                normal_body: DVec3::Z,
                areal_density_kg_m2: 5.0,
                position_body_m: DVec3::new(0.8, 0.0, 0.8),
                deployment: RadiatorDeployment::Fixed,
            },
            RadiatorSpec {
                name: format!("rad-b-{index}"),
                attached_node: format!("avionics-{index}"),
                area_m2: 2.0,
                emissivity: 0.9,
                solar_absorptivity: 0.2,
                normal_body: DVec3::Y,
                areal_density_kg_m2: 5.0,
                position_body_m: DVec3::new(-0.2, 0.8, 0.1),
                deployment: RadiatorDeployment::Foldable {
                    deployment_rate_per_s: 0.2,
                    actuator_power_w: 60.0,
                    initial_fraction: 0.5,
                },
            },
        ],
        convective_k: default_convective_k(),
    }
}

fn main() {
    let systems: Vec<_> = (0..VESSELS).map(make_system).collect();
    let mut states: Vec<_> = systems
        .iter()
        .map(|system| system.initial_state().expect("valid system"))
        .collect();
    let commands: Vec<_> = systems
        .iter()
        .map(|system| {
            let mut command = ThermalCommand::idle_for(system, 1.0);
            command.solar_flux =
                vec![SolarFluxSource::new(1_360.0, DVec3::Z, 1.0).expect("valid flux")];
            command.flow = Some(ThermalFlowCondition {
                density_kg_m3: 0.005,
                speed_mps: 2_500.0,
            });
            command.internal_heat_w = vec![200.0, 33_600.0, 900.0, 0.0];
            command.radiator_deployment_targets = vec![None, Some(1.0)];
            command
        })
        .collect();

    let started = Instant::now();
    for _ in 0..ITERATIONS {
        for ((system, state), command) in systems.iter().zip(&mut states).zip(&commands) {
            let (next, telemetry) = system
                .advance(black_box(state), black_box(command))
                .expect("thermal step");
            *state = black_box(next);
            black_box(telemetry);
        }
    }
    let elapsed = started.elapsed();
    let steps = VESSELS * ITERATIONS;
    println!(
        "{steps} vessel thermal steps in {elapsed:.3?} ({:.0} steps/s)",
        steps as f64 / elapsed.as_secs_f64()
    );
}
