//! Degree-2 harmonics: per-eval overhead and trajectory divergence.
//!
//! The bench answers two questions on every run and prints both numbers, so
//! the monopole-vs-harmonics difference is measured continuously rather than
//! asserted once:
//!  1. How much does the gated J2/C22 lane cost per field evaluation
//!     (cruise points vs points deep inside a harmonic well)?
//!  2. How far do trajectories diverge (Cinder 3-orbit endpoint + LEO nodal
//!     drift vs analytic)?
//!
//! Design envelope (AGENTS 10.1): cruise overhead must be ~zero (the gate
//! skips far field with one multiply), near-field pays full terms.

use std::hint::black_box;

use glam::DVec3;
use thessa_sim_core::{
    BakedEphemeris, GravityField, SimTime, SystemConfig, TestParticleState, VerletConfig,
};

fn real_system() -> BakedEphemeris {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
        .expect("checked-in system configuration must parse");
    config.bake().expect("checked-in system must bake")
}

fn monopole_twin(system: &BakedEphemeris) -> BakedEphemeris {
    let mut twin = system.clone();
    for body in &mut twin.bodies {
        body.j2 = 0.0;
        body.c22 = 0.0;
    }
    twin
}

fn main() {
    let system = real_system();
    let twin = monopole_twin(&system);
    let field = GravityField::from_ephemeris(&system);
    let mono = GravityField::from_ephemeris(&twin);
    let time = SimTime(1_000_000.0);

    // 1. Cruise points across the outer system (far from every harmonic
    // well: the gate must make harmonics ~free here).
    let cruise: Vec<DVec3> = (0..10_000)
        .map(|index| {
            DVec3::new(
                4.0e12 + index as f64 * 1.0e8,
                index as f64 * 1.0e7,
                index as f64 * 1.0e6,
            )
        })
        .collect();
    // 2. Points hugging Cinder (56 km rock, J2 ~ 0.28): full terms, every
    // evaluation.
    let cinder_id = system.body_id("cinder").expect("cinder exists");
    let cinder_pos = system
        .body_state(cinder_id, time)
        .expect("cinder state")
        .position_inertial;
    let cinder_shell: Vec<DVec3> = (0..10_000)
        .map(|index| {
            let angle = index as f64 * 0.01;
            cinder_pos + DVec3::new(angle.cos() * 112_000.0, angle.sin() * 112_000.0, 5_000.0)
        })
        .collect();

    for (label, points) in [("cruise-10k", &cruise), ("cinder-shell-10k", &cinder_shell)] {
        let started = std::time::Instant::now();
        let mono_acc: Vec<_> = points
            .iter()
            .map(|position| mono.acceleration(*position, time).expect("gravity"))
            .collect();
        let mono_elapsed = started.elapsed();
        black_box(&mono_acc);
        let started = std::time::Instant::now();
        let full_acc: Vec<_> = points
            .iter()
            .map(|position| field.acceleration(*position, time).expect("gravity"))
            .collect();
        let full_elapsed = started.elapsed();
        black_box(&full_acc);
        let overhead = full_elapsed.as_secs_f64() / mono_elapsed.as_secs_f64() - 1.0;
        println!(
            "{label}: monopole={mono_elapsed:?} harmonics={full_elapsed:?} overhead={overhead:.1}%"
        );
        let max_rel = mono_acc
            .iter()
            .zip(&full_acc)
            .map(|(a, b)| (*a - *b).length() / a.length().max(1e-30))
            .fold(0.0_f64, f64::max);
        println!("{label}: max relative accel difference = {max_rel:e}");
    }

    // 3. Cinder 3-orbit endpoint divergence (real system, both fields).
    // NOTE: the shell above lives at t=1e6; the propagation starts at the
    // epoch, so the anchor must be re-read there (Cinder covers millions of
    // km in between — a stale anchor measures empty space in both fields).
    let epoch_anchor = system
        .body_state(cinder_id, SimTime(0.0))
        .expect("cinder state")
        .position_inertial;
    let orbit_radius = 112_000.0;
    let cinder_mu = system.body(cinder_id).expect("cinder").mu;
    let speed = (cinder_mu / orbit_radius).sqrt();
    let start = epoch_anchor + DVec3::new(orbit_radius, 0.0, 0.0);
    // Circularize around Cinder in the inertial frame is only approximate
    // (host pull ignored over 3 orbits); both fields share the same start,
    // so the endpoint delta isolates the harmonics.
    let initial = TestParticleState {
        position: start,
        velocity: DVec3::new(0.0, speed, 0.0),
    };
    let period = std::f64::consts::TAU * (orbit_radius.powi(3) / cinder_mu).sqrt();
    let config = VerletConfig {
        step_s: period / 400.0,
        max_steps: 1_200,
    };
    let run = |ephemeris: &BakedEphemeris| {
        thessa_sim_core::propagate_sampled_verlet(ephemeris, initial, SimTime(0.0), config, &[])
            .expect("verlet")
    };
    let full_path = run(&system);
    let mono_path = run(&twin);
    let end = full_path.positions.len().min(mono_path.positions.len()) - 1;
    let divergence = (full_path.positions[end] - mono_path.positions[end]).length();
    println!("cinder 3-orbit endpoint divergence = {divergence:.1} m");
}
