//! Inter-body route survey: Pelagos (a moon of Nereid) -> Mira (the
//! double-planet companion of Orthea) around Asterion A. Broad Lambert
//! scouting over a 400-day window prices route energy and geometry;
//! year-long arcs make full N-body correction cost-prohibitive, so routes
//! are reported UNVALIDATED (no miss measurement, no executor admission
//! — see `BroadRoute` docs).
//! With THESSA_INTERBODY_EXACT=1, attempts one exact revalidation to
//! characterize its cost (slow: budget minutes, run with `timeout`).
use std::{env, hint::black_box, time::Instant};

use thessa_maneuver::{FlybyConfig, SearchConfig, broad_survey, flyby_search, porkchop_search};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    println!("sources: {}", field.source_count());
    let search = SearchConfig {
        central_body: ephemeris.body_id("asterion_a").expect("asterion_a"),
        departure_body: ephemeris.body_id("pelagos").expect("pelagos"),
        arrival_body: ephemeris.body_id("mira").expect("mira"),
        window_start: SimTime::EPOCH,
        departure_span_s: 400.0 * 86_400.0,
        departure_steps: 40,
        tof_min_s: 200.0 * 86_400.0,
        tof_max_s: 500.0 * 86_400.0,
        tof_steps: 30,
        keep_candidates: 10,
        standoff_m: 100_000.0,
        max_broad_dv_mps: 30_000.0,
        max_miss_m: 1.0e6,
    };
    let started = Instant::now();
    let (routes, stats) = broad_survey(black_box(&ephemeris), search).expect("survey finds routes");
    println!(
        "survey pelagos->mira: {} broad cells in {elapsed:?} (UNVALIDATED broad estimates)",
        stats.broad_evaluations,
        elapsed = started.elapsed(),
    );
    for (index, route) in routes.iter().take(5).enumerate() {
        println!(
            "  broad #{index}: depart T+{:.1}d, tof {:.1}d, dv {:.0} m/s (dep {:.0} + arr {:.0})",
            (route.departure_epoch.0 - SimTime::EPOCH.0) / 86_400.0,
            route.time_of_flight_s / 86_400.0,
            route.broad_total_dv_mps,
            route.departure_burn_mag_mps,
            route.arrival_burn_mag_mps,
        );
    }
    if env::var("THESSA_INTERBODY_EXACT").is_ok() {
        // Route A (recommended): fly Pelagos -> Orthea periapsis -> Mira.
        // The cruise TCM targets the PLANET (big, smooth) and the short
        // terminal leg threads the 4100 km companion with proven
        // lunar-like shooting. A single cruise TCM cannot thread a small
        // body next to a 1.8-Earth planet from 2 AU away (route B below
        // measures that).
        let orthea = ephemeris.body_id("orthea").expect("orthea");
        let started = Instant::now();
        match flyby_search(
            black_box(&ephemeris),
            black_box(&field),
            FlybyConfig {
                central_body: search.central_body,
                departure_body: search.departure_body,
                arrival_body: search.arrival_body,
                flyby_bodies: vec![orthea],
                window_start: search.window_start,
                departure_span_s: search.departure_span_s,
                departure_steps: 20,
                leg1_min_s: 200.0 * 86_400.0,
                leg1_max_s: 500.0 * 86_400.0,
                leg1_steps: 20,
                leg2_min_s: 5.0 * 86_400.0,
                leg2_max_s: 60.0 * 86_400.0,
                leg2_steps: 8,
                keep_routes: 2,
                standoff_m: 100_000.0,
                max_broad_dv_mps: 30_000.0,
                // Terminal-rendezvous scale, not lunar scale.
                max_miss_m: 1.0e8,
            },
        ) {
            Ok((ranked, stats)) => {
                let winner = &ranked[0];
                let nodes: Vec<String> = winner
                    .plan
                    .nodes
                    .iter()
                    .map(|node| format!("{:.0}", node.magnitude_mps()))
                    .collect();
                println!(
                    "exact pelagos->orthea->mira: dv {:.0} m/s [{}], miss {:.1} km, {} newton props in {elapsed:?}",
                    winner.exact_total_dv_mps,
                    nodes.join("+"),
                    winner.exact_miss_m / 1_000.0,
                    stats.newton_propagations,
                    elapsed = started.elapsed(),
                );
            }
            Err(error) => println!("exact pelagos->orthea->mira failed: {error}"),
        }
        // Route B (documented hard): single cruise TCM straight to Mira.
        // Scale-appropriate miss filter: 1e8 m (terminal rendezvous
        // corrects the rest on approach) instead of the lunar 1e6 m.
        let started = Instant::now();
        match porkchop_search(
            black_box(&ephemeris),
            black_box(&field),
            SearchConfig {
                keep_candidates: 1,
                max_miss_m: 1.0e8,
                ..search
            },
        ) {
            Ok((ranked, stats)) => {
                let winner = &ranked[0];
                let nodes: Vec<String> = winner
                    .plan
                    .nodes
                    .iter()
                    .map(|node| format!("{:.0}", node.magnitude_mps()))
                    .collect();
                println!(
                    "exact pelagos->mira: dv {:.0} m/s [{}], miss {:.1} km, {} newton props in {elapsed:?}",
                    winner.exact_total_dv_mps,
                    nodes.join("+"),
                    winner.exact_miss_m / 1_000.0,
                    stats.newton_propagations,
                    elapsed = started.elapsed(),
                );
            }
            Err(error) => println!("exact pelagos->mira failed: {error}"),
        }
    } else {
        println!("exact revalidation skipped (set THESSA_INTERBODY_EXACT=1 to attempt)");
    }
}
