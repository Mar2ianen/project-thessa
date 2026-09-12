//! Experimental causal force interpolation, deliberately not enabled in flight.
//! Endpoint force is evaluated at a quadratic predicted position; the actual
//! endpoint is not known until integration completes. Re-anchor every interval.
use glam::DVec3;
use std::{hint::black_box, time::Instant};
use thessa_sim_core::{EphemerisTable, GravityField, SimTime, SystemConfig, TestParticleState};

const DT: f64 = 1.0 / 120.0;
const STEPS: usize = 72_000;

fn integrate(
    initial: TestParticleState,
    acceleration: impl Fn(DVec3, f64) -> DVec3,
    block: usize,
) -> Vec<TestParticleState> {
    let mut state = initial;
    let mut path = Vec::with_capacity(STEPS);
    let mut a = acceleration(state.position, 0.0);
    let mut start_a = a;
    let mut end_a = a;
    for step in 0..STEPS {
        let t = step as f64 * DT;
        if block > 1 && step % block == 0 {
            start_a = acceleration(state.position, t);
            a = start_a;
            let h = block as f64 * DT;
            end_a = acceleration(
                state.position + state.velocity * h + a * (0.5 * h * h),
                t + h,
            );
        }
        let position = state.position + state.velocity * DT + a * (0.5 * DT * DT);
        let next_a = if block == 1 {
            acceleration(position, t + DT)
        } else {
            start_a.lerp(end_a, (step % block + 1) as f64 / block as f64)
        };
        state.velocity += (a + next_a) * (0.5 * DT);
        state.position = position;
        a = next_a;
        path.push(state);
    }
    path
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let gravity = GravityField::from_ephemeris(&ephemeris);
    let sources: Vec<_> = ephemeris
        .bodies
        .iter()
        .filter(|body| body.gravity_source)
        .map(|body| body.id)
        .collect();
    let started = Instant::now();
    let table =
        EphemerisTable::build(&ephemeris, &sources, SimTime::EPOCH, SimTime(610.0), 1.0).unwrap();
    println!(
        "shared ephemeris table build {:.3} ms (22 sources, 1 s nodes)",
        started.elapsed().as_secs_f64() * 1000.0
    );
    println!(
        "scenario,force_interval_s,wall_ms,speedup_vs_table_per_tick,max_position_error_m,max_velocity_error_mps"
    );
    for (label, body_name, radius, speed_factor) in [
        ("thessa_low_orbit", "thessa", 3.4e6, 1.0),
        ("nereid_fast_periapsis", "nereid", 7.0e7, 1.4),
        ("deep_coast", "thessa", 1.0e9, 0.2),
    ] {
        let id = ephemeris.body_id(body_name).unwrap();
        let body = ephemeris.body(id).unwrap();
        let center = ephemeris.body_state(id, SimTime::EPOCH).unwrap();
        let initial = TestParticleState {
            position: center.position_inertial + DVec3::Z * radius,
            velocity: center.velocity_inertial
                + DVec3::X * ((body.mu / radius).sqrt() * speed_factor),
        };
        let started = Instant::now();
        let exact = integrate(
            initial,
            |p, t| gravity.acceleration(p, SimTime(t)).unwrap(),
            1,
        );
        let exact_ms = started.elapsed().as_secs_f64() * 1000.0;
        let mut table_ms = 0.0;
        for block in [1, 12, 60, 120, 240, 600] {
            let mut runs = Vec::new();
            let mut errors = (0.0_f64, 0.0_f64);
            for _ in 0..3 {
                let started = Instant::now();
                let path = integrate(
                    initial,
                    |p, t| table.acceleration_at(p, SimTime(t)).unwrap(),
                    block,
                );
                runs.push(started.elapsed().as_secs_f64() * 1000.0);
                for (state, reference) in path.iter().zip(&exact) {
                    errors.0 = errors.0.max((state.position - reference.position).length());
                    errors.1 = errors.1.max((state.velocity - reference.velocity).length());
                }
                black_box(path);
            }
            runs.sort_by(f64::total_cmp);
            if block == 1 {
                table_ms = runs[1];
                println!(
                    "{label}: exact Kepler per tick {exact_ms:.3} ms; table speedup {:.2}x",
                    exact_ms / table_ms
                );
            }
            println!(
                "{label},{:.5},{:.3},{:.2},{:.6},{:.9}",
                block as f64 * DT,
                runs[1],
                table_ms / runs[1],
                errors.0,
                errors.1
            );
        }
    }
}
