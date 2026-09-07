use std::hint::black_box;

use glam::DVec3;
use thessa_sim_core::{
    AU_M, BakedBody, BakedEphemeris, BodyId, G, GravityField, SOLAR_MASS_KG, SimTime,
};

fn main() {
    let central_mu = G * SOLAR_MASS_KG;
    let ephemeris = BakedEphemeris::new(
        "BENCH_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "central", central_mu, 0.0)],
    )
    .expect("valid benchmark ephemeris");
    let field = GravityField::from_ephemeris(&ephemeris);
    let positions: Vec<_> = (0..10_000)
        .map(|index| {
            let radius = AU_M * (0.5 + (index % 100) as f64 / 100.0);
            DVec3::new(radius, index as f64 * 1_000.0, 0.0)
        })
        .collect();
    let started = std::time::Instant::now();
    let accelerations = field
        .accelerations(black_box(&positions), SimTime::EPOCH)
        .expect("finite benchmark acceleration");
    let elapsed = started.elapsed();
    black_box(accelerations);
    println!("10k gravity states: {elapsed:?}");
}
