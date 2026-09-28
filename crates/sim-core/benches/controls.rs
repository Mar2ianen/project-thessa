//! Batch cost of moving hinged body-control panels, evaluating detailed aero
//! loads, and advancing load-limited actuators.
//! Run with `cargo bench -p thessa-sim-core --bench controls`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::*;

const VEHICLES: usize = 256;
const PANELS: usize = 16;
const CONTROLS: usize = 4;
const ITERATIONS: usize = 24;

fn make_vehicle() -> VehicleDefinition {
    let mut panels = Vec::with_capacity(PANELS);
    for index in 0..PANELS {
        let position = DVec3::new(
            -2.0 + (index / CONTROLS) as f64,
            (index % 2) as f64 * 0.2,
            ((index / 2) % 2) as f64 * 0.1,
        );
        let panel = AeroPanel::flat_plate(position, 1.0, 0.8)
            .unwrap()
            .with_center_of_pressure(position + DVec3::X * 0.4)
            .unwrap();
        panels.push(panel);
    }
    let geometry = AeroGeometry::new(panels).unwrap();
    let properties =
        RigidBodyProperties::new(1_000.0, DMat3::from_diagonal(DVec3::splat(500.0))).unwrap();
    let actuator = ControlSurfaceActuator {
        max_rate_rad_s: 0.6,
        max_torque_nm: 2_000.0,
    };
    let controls = (0..CONTROLS)
        .map(|index| {
            let hinge_axis = if index % 2 == 0 { -DVec3::Y } else { DVec3::Z };
            ControlSurfaceDefinition::new(
                format!("body-control-{index}"),
                (index..PANELS).step_by(CONTROLS).collect(),
                -0.4,
                0.4,
            )
            .unwrap()
            .with_hinge(ControlHinge::new(DVec3::ZERO, hinge_axis).unwrap())
            .with_actuator(actuator)
        })
        .collect();
    VehicleDefinition::new("control-bench", geometry, properties, controls).unwrap()
}

fn main() {
    let template = make_vehicle();
    let reference = template.aero_geometry.clone();
    let mut fleet: Vec<_> = (0..VEHICLES).map(|_| template.clone()).collect();
    let mut panel_soa: Vec<_> = fleet
        .iter()
        .map(|vehicle| PanelSoA::from_geometry(&vehicle.aero_geometry).unwrap())
        .collect();
    let mut states = vec![vec![0.0; CONTROLS]; VEHICLES];
    let commands: Vec<Vec<f64>> = (0..VEHICLES)
        .map(|vehicle| {
            (0..CONTROLS)
                .map(|control| {
                    if (vehicle + control) % 2 == 0 {
                        0.8
                    } else {
                        -0.8
                    }
                })
                .collect()
        })
        .collect();
    let model = PanelAeroModel::new(AeroConfig::default()).unwrap();
    let atmosphere = AtmosphereConfig::default();
    let environment = atmosphere.aero_environment(2_000.0, DVec3::ZERO).unwrap();
    let aero_state = AeroState::new(DVec3::new(120.0, 4.0, -12.0), DVec3::new(0.0, 0.01, 0.0));
    let dt_s = 1.0 / 120.0;

    let start = Instant::now();
    for _ in 0..ITERATIONS {
        for vehicle_index in 0..VEHICLES {
            let vehicle = &mut fleet[vehicle_index];
            let loads = model
                .evaluate_state_detailed(aero_state, environment, &vehicle.aero_geometry)
                .unwrap();
            let hinge_moments = vehicle.control_hinge_moments(&loads).unwrap();
            let (next, saturated) = vehicle
                .advance_control_actuators(
                    &states[vehicle_index],
                    &commands[vehicle_index],
                    &hinge_moments,
                    dt_s,
                )
                .unwrap();
            black_box(saturated);
            states[vehicle_index] = next;
            vehicle
                .apply_control_deflections(&reference, &states[vehicle_index])
                .unwrap();
            panel_soa[vehicle_index]
                .sync_geometry(&vehicle.aero_geometry)
                .unwrap();
            black_box(&panel_soa[vehicle_index]);
            black_box(&vehicle.aero_geometry);
        }
    }
    let elapsed = start.elapsed();
    let vehicle_steps = VEHICLES * ITERATIONS;
    println!(
        "hinged body controls: {VEHICLES} vehicles x {PANELS} panels x {CONTROLS} actuators x {ITERATIONS} steps: {elapsed:?} total, {:.2} us/vehicle-step, {:.1} million panel-steps/s",
        elapsed.as_secs_f64() * 1.0e6 / vehicle_steps as f64,
        (vehicle_steps * PANELS) as f64 / elapsed.as_secs_f64() / 1.0e6,
    );
}
