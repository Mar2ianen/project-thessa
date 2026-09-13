use std::hint::black_box;

use glam::DVec3;
use thessa_sim_core::{BakedEphemeris, EphemerisFrame, GravityField, SimTime, SystemConfig};

fn main() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris: BakedEphemeris = config.bake().expect("bake");
    let field = GravityField::from_ephemeris(&ephemeris);
    let sources = field.source_count();
    println!(
        "bodies: {}, gravity sources: {sources}",
        ephemeris.bodies.len()
    );

    // Positions near Thessa's orbit: representative per-tick gravity queries.
    let thessa = ephemeris.body_id("thessa").expect("thessa");
    let home = ephemeris
        .body_state(thessa, SimTime::EPOCH)
        .expect("home state");
    let positions: Vec<_> = (0..32)
        .map(|i| home.position_inertial + DVec3::new(i as f64 * 1_000.0, 0.0, 500_000.0))
        .collect();

    // One frame evaluation per query at advancing time, like consecutive
    // flight ticks: each tick serves one gravity call plus one dominant
    // scan from the same timestamp.
    let rounds = 20;
    let mut naive_total = std::time::Duration::ZERO;
    let mut batch_total = std::time::Duration::ZERO;
    let mut frame = EphemerisFrame::new();
    for round in 0..rounds {
        // Alternate order per round so a one-directional drift cannot bias
        // the comparison.
        let base = round as f64 * 32.0 * 8.0 / 120.0;
        let tick = |i: usize| SimTime(base + i as f64 * 8.0 / 120.0);
        if round % 2 == 0 {
            let started = std::time::Instant::now();
            for (i, position) in positions.iter().enumerate() {
                let time = tick(i);
                black_box(field.acceleration(*position, time).expect("gravity"));
                black_box(ephemeris.dominant_body(*position, time));
            }
            naive_total += started.elapsed();
            let started = std::time::Instant::now();
            for (i, position) in positions.iter().enumerate() {
                let states = frame.evaluate(&ephemeris, tick(i)).expect("frame");
                black_box(
                    field
                        .acceleration_from_states(*position, states)
                        .expect("gravity"),
                );
                black_box(ephemeris.dominant_body_from_states(*position, states));
            }
            batch_total += started.elapsed();
        } else {
            let started = std::time::Instant::now();
            for (i, position) in positions.iter().enumerate() {
                let states = frame.evaluate(&ephemeris, tick(i)).expect("frame");
                black_box(
                    field
                        .acceleration_from_states(*position, states)
                        .expect("gravity"),
                );
                black_box(ephemeris.dominant_body_from_states(*position, states));
            }
            batch_total += started.elapsed();
            let started = std::time::Instant::now();
            for (i, position) in positions.iter().enumerate() {
                let time = tick(i);
                black_box(field.acceleration(*position, time).expect("gravity"));
                black_box(ephemeris.dominant_body(*position, time));
            }
            naive_total += started.elapsed();
        }
    }
    let queries = (rounds * 32) as f64;
    let naive_us = naive_total.as_micros() as f64 / queries;
    let batch_us = batch_total.as_micros() as f64 / queries;
    println!("naive gravity+dominant per tick: {naive_us:.2} us (32 queries x {rounds} rounds)");
    println!("batch frame+gravity+dominant:    {batch_us:.2} us (32 queries x {rounds} rounds)");
    println!("speedup: {:.2}x", naive_us / batch_us.max(1e-9));
}
