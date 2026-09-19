//! Axial-profile build cost: one methalox sea-level source, 48 stations.
//! Run with `cargo bench -p thessa-plume-core --bench profile`.

use std::{hint::black_box, time::Instant};

use thessa_plume_core::profile::build_axial_profile;
use thessa_plume_core::source::{ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform};

fn main() {
    let source = PlumeSource {
        nozzle_to_vehicle: RigidTransform::IDENTITY,
        exit_radius_m: 0.65,
        mass_flow_kg_s: 520.0,
        exhaust_velocity_mps: 3560.0,
        exit_pressure_pa: 68_000.0,
        exit_temperature_k: 1900.0,
        exit_mach: 3.4,
        throttle: 1.0,
        exhaust: ExhaustFamily::Methalox,
    };
    let env = PlumeEnvironment {
        pressure_pa: 101_325.0,
        density_kg_m3: 1.225,
        temperature_k: 288.15,
        oxygen_fraction: 0.21,
        flow_velocity_local_mps: [0.0; 3],
    };
    // Warm up (first build also validates).
    let profile = build_axial_profile(&source, &env, 48).expect("profile builds");
    assert_eq!(profile.stations.len(), 48);

    let iterations = 20000;
    let started = Instant::now();
    let mut checksum = 0.0;
    for _ in 0..iterations {
        let profile = build_axial_profile(&source, &env, 48).expect("profile builds");
        checksum += profile.stations[profile.stations.len() - 1].emission_rgb[1];
        black_box(&profile);
    }
    let elapsed = started.elapsed();
    println!(
        "build_axial_profile x{iterations}: {:.3} ms total, {:.1} ns/iter, checksum={checksum:.6}",
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1e9 / iterations as f64,
    );
}
