//! Chained-assist demo on the fictional system. This bench stays inside
//! one central body (Nereid): Pelagos -> Thessa flyby -> Auron rendezvous,
//! i.e. the two-leg chain form of the single-tour `flyby` bench. Broad
//! survey (milliseconds) plus exact revalidation.
use std::{hint::black_box, time::Instant};

use thessa_maneuver::{
    ChainConfig, ChainEncounter, EncounterKind, broad_chain_survey, chain_search,
};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let central = ephemeris.body_id("nereid").expect("nereid");
    let pelagos = ephemeris.body_id("pelagos").expect("pelagos");
    let thessa = ephemeris.body_id("thessa").expect("thessa");
    let auron = ephemeris.body_id("auron").expect("auron");
    let chain = ChainConfig {
        central_body: central,
        departure_body: pelagos,
        encounters: vec![
            ChainEncounter {
                body: thessa,
                kind: EncounterKind::Flyby,
                standoff_m: None,
            },
            ChainEncounter {
                body: auron,
                kind: EncounterKind::Rendezvous,
                standoff_m: None,
            },
        ],
        window_start: SimTime::EPOCH,
        departure_span_s: 10.0 * 86_400.0,
        departure_steps: 24,
        leg_tof_min_s: vec![20.0 * 3_600.0, 20.0 * 3_600.0],
        leg_tof_max_s: vec![120.0 * 3_600.0, 150.0 * 3_600.0],
        leg_tof_steps: vec![6, 6],
        keep_routes: 3,
        standoff_m: 100_000.0,
        max_broad_dv_mps: 15_000.0,
        max_miss_m: 1.0e6,
    };
    let started = Instant::now();
    let (routes, stats) = broad_chain_survey(black_box(&ephemeris), chain.clone()).expect("survey");
    println!(
        "broad pelagos->thessa->auron: {} routes, {} cells in {:?}; winner tot {:.0} [dep {:.0} + turns {} + arr {:.0}]",
        routes.len(),
        stats.broad_evaluations,
        started.elapsed(),
        routes[0].broad_total_dv_mps,
        routes[0].departure_burn_mag_mps,
        routes[0]
            .flyby_burns_mag_mps
            .iter()
            .map(|mag| format!("{mag:.0}"))
            .collect::<Vec<_>>()
            .join("+"),
        routes[0].arrival_burn_mag_mps,
    );
    let started = Instant::now();
    let (ranked, stats) =
        chain_search(black_box(&ephemeris), black_box(&field), chain).expect("search");
    let winner = &ranked[0];
    let nodes: Vec<String> = winner
        .plan
        .nodes
        .iter()
        .map(|node| format!("{:.0}", node.magnitude_mps()))
        .collect();
    println!(
        "exact pelagos->thessa->auron: dv {:.0} [{}], miss {:.1} km, {} assists, {} newton props in {:?}",
        winner.exact_total_dv_mps,
        nodes.join("+"),
        winner.exact_miss_m / 1_000.0,
        winner.plan.flybys.len(),
        stats.newton_propagations,
        started.elapsed(),
    );
}
