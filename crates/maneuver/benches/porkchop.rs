//! Staged porkchop search on the real system (doc 24 §15 benchmark C-lite):
//! moon tour Pelagos -> Thessa around Nereid. Broad Lambert grid, local
//! refinement, exact N-body revalidation of survivors. Reports candidates
//! evaluated per second, revalidation count, winning Δv and measured miss.
use std::{hint::black_box, time::Instant};

use thessa_maneuver::{SearchConfig, porkchop_search};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    println!("sources: {}", field.source_count());
    let central = ephemeris.body_id("nereid").expect("nereid");
    let departure = ephemeris.body_id("pelagos").expect("pelagos");
    let arrival = ephemeris.body_id("thessa").expect("thessa");
    let search = SearchConfig {
        central_body: central,
        departure_body: departure,
        arrival_body: arrival,
        window_start: SimTime::EPOCH,
        departure_span_s: 10.0 * 86_400.0,
        departure_steps: 25,
        tof_min_s: 20.0 * 3_600.0,
        tof_max_s: 120.0 * 3_600.0,
        tof_steps: 25,
        keep_candidates: 3,
        standoff_m: 100_000.0,
        max_broad_dv_mps: 5_000.0,
        max_miss_m: 1.0e6,
    };
    let started = Instant::now();
    let (ranked, stats) =
        porkchop_search(black_box(&ephemeris), black_box(&field), search).expect("search finds");
    let elapsed = started.elapsed();
    let winner = &ranked[0];
    println!(
        "porkchop pelagos->thessa: {} broad cells ({:.0}/s), {} degenerate, {} impact, {} exact revalidations ({} failed) in {elapsed:?}",
        stats.broad_evaluations,
        stats.broad_evaluations as f64 / elapsed.as_secs_f64(),
        stats.degenerate_cells,
        stats.impact_cells,
        stats.exact_revalidations,
        stats.failed_revalidations,
    );
    println!(
        "winner: depart T+{:.1}h, tof {:.1}h, broad dv {:.1} m/s, exact dv {:.1} m/s, miss {:.1} km",
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / 3_600.0,
        winner.time_of_flight_s / 3_600.0,
        winner.broad_total_dv_mps,
        winner.exact_total_dv_mps,
        winner.exact_miss_m / 1_000.0,
    );
    for (index, plan) in ranked.iter().enumerate() {
        println!(
            "  #{index}: dv {:.1} m/s, miss {:.1} km, nodes {}",
            plan.exact_total_dv_mps,
            plan.exact_miss_m / 1_000.0,
            plan.plan.nodes.len(),
        );
    }
}
