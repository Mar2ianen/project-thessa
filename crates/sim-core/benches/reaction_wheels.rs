//! Cost of per-step reaction-wheel torque allocation across vehicle fleets.
//! Run with `cargo bench -p thessa-sim-core --bench reaction_wheels`.
use std::{hint::black_box, time::Instant};

use glam::{DMat3, DVec3};
use thessa_sim_core::{ReactionWheelBankSpec, allocate_reaction_wheels};

const VEHICLES: usize = 256;
const ITERATIONS: usize = 20_000;

fn bank(index: usize) -> ReactionWheelBankSpec {
    ReactionWheelBankSpec {
        name: format!("wheel-bank-{index}"),
        max_torque_body_nm: DVec3::new(240.0, 240.0, 180.0),
        mass_kg: 24.0,
        position_body_m: DVec3::ZERO,
        inertia_body_kg_m2: DMat3::IDENTITY * 8.0,
    }
}

fn main() {
    for bank_count in [1, 4, 16, 64] {
        let fleet: Vec<_> = (0..VEHICLES)
            .map(|_| (0..bank_count).map(bank).collect::<Vec<_>>())
            .collect();
        let start = Instant::now();
        for step in 0..ITERATIONS {
            let requested = DVec3::new(if step % 2 == 0 { 500.0 } else { -500.0 }, 120.0, -80.0);
            for banks in &fleet {
                black_box(allocate_reaction_wheels(banks, requested).unwrap());
            }
        }
        let elapsed = start.elapsed();
        let vehicle_steps = VEHICLES * ITERATIONS;
        println!(
            "reaction-wheel allocation: {VEHICLES} vehicles x {bank_count} banks x {ITERATIONS} steps: {elapsed:?}, {:.2} us/vehicle-step, {:.1} million bank-steps/s",
            elapsed.as_secs_f64() * 1.0e6 / vehicle_steps as f64,
            (vehicle_steps * bank_count) as f64 / elapsed.as_secs_f64() / 1.0e6,
        );
    }
}
