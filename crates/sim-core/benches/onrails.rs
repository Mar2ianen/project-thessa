//! Full-horizon bake and warm sampling for 256 independent coast caches.
use glam::DVec3;
use std::{hint::black_box, time::Instant};
use thessa_sim_core::{
    BakedBody, BakedEphemeris, BodyId, OnRailsCache, SimTime, SystemConfig, TestParticleState,
    VerletConfig,
};
fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let initial = TestParticleState {
        position: home.position_inertial + DVec3::Z * 1e9,
        velocity: home.velocity_inertial + DVec3::X * 100.0,
    };
    let bodies: Vec<_> = ephemeris
        .bodies
        .iter()
        .filter(|b| b.radius_m > 0.0)
        .map(|b| b.id)
        .collect();
    let mut full = OnRailsCache::new();
    let started = Instant::now();
    full.bake(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 5.0,
            max_steps: 40_000,
        },
        &bodies,
    )
    .unwrap();
    println!(
        "22-source full horizon: {:.3} ms, {} nodes",
        started.elapsed().as_secs_f64() * 1000.0,
        full.path().unwrap().positions.len()
    );
    let central = BakedEphemeris::new(
        "BENCH",
        vec![BakedBody::fixed(BodyId(0), "body", 4e13, 3.2e6)],
    )
    .unwrap();
    let caches: Vec<_> = (0..256)
        .map(|i| {
            let radius = 1e7 + i as f64 * 1000.0;
            let mut cache = OnRailsCache::new();
            cache
                .bake(
                    &central,
                    TestParticleState {
                        position: DVec3::X * radius,
                        velocity: DVec3::Y * (4e13 / radius).sqrt(),
                    },
                    SimTime::EPOCH,
                    VerletConfig {
                        step_s: 5.0,
                        max_steps: 1000,
                    },
                    &[],
                )
                .unwrap();
            cache
        })
        .collect();
    let mut samples = Vec::new();
    for frame in 0..2100 {
        let started = Instant::now();
        for (index, cache) in caches.iter().enumerate() {
            black_box(cache.sample_at(black_box(SimTime((frame + index) as f64 * 0.5))));
        }
        if frame >= 100 {
            samples.push(started.elapsed().as_secs_f64() * 1e6);
        }
    }
    samples.sort_by(f64::total_cmp);
    println!(
        "256 warm trajectory samples: p50 {:.3} us, p95 {:.3} us, p99 {:.3} us",
        samples[1000], samples[1900], samples[1980]
    );
}
