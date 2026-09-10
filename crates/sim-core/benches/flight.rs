use std::hint::black_box;

use glam::{DMat3, DQuat, DVec3};
use rayon::prelude::*;
use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroPanel, AtmosphereConfig, FlightStepInput, PanelAeroModel,
    RigidBodyProperties, RigidBodyState, integrate_rigid_body_step,
};

fn main() {
    let panels: Vec<_> = (0..16)
        .map(|index| {
            let x = (index as f64 - 8.0) * 0.4;
            AeroPanel::flat_plate(DVec3::new(x, 0.0, 0.0), 1.5, 1.0).expect("valid panel")
        })
        .collect();
    let geometry = AeroGeometry::new(panels).expect("valid geometry");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");
    let atmosphere = AtmosphereConfig::default();
    let properties = RigidBodyProperties::new(1_000.0, DMat3::from_diagonal(DVec3::splat(1_000.0)))
        .expect("valid mass properties");
    let input = FlightStepInput::new(10_000.0, DVec3::new(0.0, 0.0, -9.80665));
    let states: Vec<_> = (0..256)
        .map(|index| {
            RigidBodyState::new(
                DVec3::new(index as f64, 0.0, 10_000.0),
                DVec3::new(350.0 + index as f64 * 0.25, 0.0, 12.0),
                DQuat::IDENTITY,
                DVec3::ZERO,
            )
            .expect("valid state")
        })
        .collect();

    for (name, angular_rate, inertia) in [
        ("cruise", DVec3::ZERO, properties.inertia_body_kg_m2),
        (
            "fast asymmetric rotation",
            DVec3::new(-8.87, -2.54, 0.053),
            thessa_sim_core::X15StarterProfile::new()
                .unwrap()
                .vehicle
                .mass_properties
                .inertia_body_kg_m2,
        ),
    ] {
        let properties = RigidBodyProperties::new(properties.mass_kg, inertia).unwrap();
        let iterations = 500;
        let started = std::time::Instant::now();
        for _ in 0..iterations {
            let next_states: Vec<_> = states
                .par_iter()
                .map(|state| {
                    let mut state = *state;
                    state.angular_velocity_body_rps = angular_rate;
                    integrate_rigid_body_step(
                        black_box(&model),
                        black_box(&geometry),
                        atmosphere,
                        state,
                        properties,
                        input,
                        1.0 / 120.0,
                    )
                    .expect("finite flight step")
                    .0
                })
                .collect();
            black_box(next_states);
        }
        let elapsed = started.elapsed();
        println!(
            "{name}: 256 vehicles x 16 panels x 6-DoF: {elapsed:?} total, {:?} average batch",
            elapsed / iterations
        );
    }
}
