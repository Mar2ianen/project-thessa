//! Cost of the deterministic parachute state/load update across vehicle fleets.
//! Run with `cargo bench -p thessa-sim-core --bench parachutes`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::{
    AtmosphereComposition, AtmosphereSample, ParachuteEnvironment, ParachutePhase, ParachuteSpec,
    ParachuteState,
};

const VEHICLES: usize = 256;
const ITERATIONS: usize = 10_000;
const DT_S: f64 = 0.02;

fn parachute(index: usize) -> ParachuteSpec {
    ParachuteSpec {
        name: format!("bench-canopy-{index}"),
        reference_area_m2: 28.0,
        drag_coefficient: 1.5,
        reefed_area_fraction: 0.12,
        inflation_time_s: 3.0,
        deploy_pressure_pa: 9_000.0,
        max_deploy_dynamic_pressure_pa: 1_800.0,
        max_canopy_load_n: 180_000.0,
        pack_mass_kg: 24.0,
        position_body_m: DVec3::new(-2.0 - index as f64 * 0.01, 0.0, 0.0),
        inertia_body_kg_m2: DMat3::IDENTITY * 4.0,
    }
}

fn main() {
    let atmosphere = AtmosphereSample {
        altitude_m: 2_000.0,
        temperature_k: 270.0,
        pressure_pa: 20_000.0,
        density_kg_m3: 0.2,
        speed_of_sound_mps: 330.0,
        dynamic_viscosity_pa_s: 1.7e-5,
        composition: AtmosphereComposition::parse("N2/O2").unwrap(),
    };
    for bank_count in [1, 2, 4, 8] {
        let mut fleet: Vec<_> = (0..VEHICLES)
            .map(|_| {
                (0..bank_count)
                    .map(|index| {
                        (
                            parachute(index),
                            ParachuteState {
                                phase: ParachutePhase::Deployed,
                                inflation_elapsed_s: 0.0,
                            },
                        )
                    })
                    .collect::<Vec<_>>()
            })
            .collect();
        let start = Instant::now();
        for _ in 0..ITERATIONS {
            for vehicle in &mut fleet {
                for (spec, state) in vehicle {
                    let load = spec
                        .advance(
                            *state,
                            ParachuteEnvironment {
                                atmosphere,
                                radial_velocity_mps: -10.0,
                                center_of_mass_air_velocity_body_mps: DVec3::X * 90.0,
                                angular_velocity_body_rps: DVec3::ZERO,
                                dt_s: DT_S,
                            },
                        )
                        .unwrap();
                    *state = load.state;
                    black_box(load);
                }
            }
        }
        let elapsed = start.elapsed();
        let vehicle_steps = VEHICLES * ITERATIONS;
        println!(
            "parachute update: {VEHICLES} vehicles x {bank_count} packs x {ITERATIONS} steps: {elapsed:?}, {:.2} us/vehicle-step, {:.1} million canopy-steps/s",
            elapsed.as_secs_f64() * 1.0e6 / vehicle_steps as f64,
            (vehicle_steps * bank_count) as f64 / elapsed.as_secs_f64() / 1.0e6,
        );
    }
}
