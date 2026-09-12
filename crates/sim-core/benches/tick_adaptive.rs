//! Tick-adaptive coast cost and accuracy against an analytic circular orbit.
use glam::DVec3;
use std::time::Instant;
use thessa_sim_core::*;
fn main() {
    let mu = 4e13;
    let r: f64 = 1e7;
    let speed = (mu / r).sqrt();
    let ephemeris =
        BakedEphemeris::new("bench", vec![BakedBody::fixed(BodyId(0), "body", mu, 0.0)]).unwrap();
    let initial = TestParticleState {
        position: DVec3::X * r,
        velocity: DVec3::Y * speed,
    };
    let config = TickIntegratorConfig {
        duration_ticks: 200_000 * WORLD_TICK_HZ,
        ..Default::default()
    };
    let now = Instant::now();
    let path = propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[]).unwrap();
    let elapsed = now.elapsed();
    let angle = speed / r * 200_000.0;
    let error = path
        .positions
        .last()
        .unwrap()
        .distance(DVec3::new(angle.cos(), angle.sin(), 0.0) * r);
    println!(
        "Circular 200000 s: {:.3} ms, {} accepted / {} rejected, endpoint error {:.6} m",
        elapsed.as_secs_f64() * 1000.0,
        path.stats.accepted_steps,
        path.stats.rejected_steps,
        error
    );
    let system: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = system.bake().unwrap();
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
    let config = TickIntegratorConfig::default();
    let now = Instant::now();
    let path =
        propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &bodies).unwrap();
    println!(
        "22 sources 200000 s: {:.3} ms, {} accepted / {} rejected, {} nodes",
        now.elapsed().as_secs_f64() * 1000.0,
        path.stats.accepted_steps,
        path.stats.rejected_steps,
        path.positions.len()
    );
}
