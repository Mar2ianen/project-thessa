//! Moon-tour demo with a gravity assist: Pelagos -> (Thessa flyby) ->
//! Auron around Nereid, against the two-direct-transfers stack on the same
//! window. The tour avoids stopping at Thessa (no 5.5 km/s arrival match,
//! no redeparture): the flyby bend — powered only as needed — does the
//! retargeting. Both sides return validated plans (measured miss).
use std::{hint::black_box, time::Instant};

use thessa_maneuver::{FlybyConfig, SearchConfig, flyby_search, porkchop_search};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

fn direct_config(
    central: thessa_sim_core::BodyId,
    departure: thessa_sim_core::BodyId,
    arrival: thessa_sim_core::BodyId,
) -> SearchConfig {
    SearchConfig {
        central_body: central,
        departure_body: departure,
        arrival_body: arrival,
        window_start: SimTime::EPOCH,
        departure_span_s: 10.0 * 86_400.0,
        departure_steps: 40,
        tof_min_s: 20.0 * 3_600.0,
        tof_max_s: 150.0 * 3_600.0,
        tof_steps: 40,
        keep_candidates: 4,
        standoff_m: 100_000.0,
        max_broad_dv_mps: 8_000.0,
        max_miss_m: 1.0e6,
    }
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let central = ephemeris.body_id("nereid").expect("nereid");
    let pelagos = ephemeris.body_id("pelagos").expect("pelagos");
    let thessa = ephemeris.body_id("thessa").expect("thessa");
    let auron = ephemeris.body_id("auron").expect("auron");
    // Tour: Pelagos -> Thessa flyby -> Auron.
    let tour = FlybyConfig {
        central_body: central,
        departure_body: pelagos,
        arrival_body: auron,
        flyby_bodies: vec![thessa],
        window_start: SimTime::EPOCH,
        departure_span_s: 10.0 * 86_400.0,
        departure_steps: 36,
        leg1_min_s: 20.0 * 3_600.0,
        leg1_max_s: 120.0 * 3_600.0,
        leg1_steps: 7,
        leg2_min_s: 20.0 * 3_600.0,
        leg2_max_s: 150.0 * 3_600.0,
        leg2_steps: 7,
        keep_routes: 5,
        standoff_m: 100_000.0,
        max_broad_dv_mps: 10_000.0,
        max_miss_m: 1.0e6,
    };
    let started = Instant::now();
    let (ranked, stats) =
        flyby_search(black_box(&ephemeris), black_box(&field), tour).expect("tour finds");
    let winner = &ranked[0];
    let event = winner.plan.flybys[0];
    let nodes: Vec<String> = winner
        .plan
        .nodes
        .iter()
        .map(|node| format!("{:.0}", node.magnitude_mps()))
        .collect();
    println!(
        "tour pelagos->thessa->auron: {} broad cells, {} revalidated in {:?}",
        stats.broad_evaluations,
        stats.exact_revalidations,
        started.elapsed(),
    );
    println!(
        "tour winner: depart T+{:.1}h, legs {:.1}h + {:.1}h, broad dv {:.0} m/s, exact dv {:.0} m/s [{}], miss {:.1} km, thessa burn {:.0} m/s at rp={:.0} km",
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / 3_600.0,
        (event.epoch.0 - winner.departure_epoch.0) / 3_600.0,
        winner.time_of_flight_s / 3_600.0 - (event.epoch.0 - winner.departure_epoch.0) / 3_600.0,
        winner.broad_total_dv_mps,
        winner.exact_total_dv_mps,
        nodes.join("+"),
        winner.exact_miss_m / 1_000.0,
        event.burn_mps,
        event.periapsis_m / 1_000.0,
    );
    // Stack: two direct transfers covering the same moons.
    let started = Instant::now();
    let (first, _) = porkchop_search(
        black_box(&ephemeris),
        black_box(&field),
        direct_config(central, pelagos, thessa),
    )
    .expect("direct pelagos->thessa finds");
    let (second, _) = porkchop_search(
        black_box(&ephemeris),
        black_box(&field),
        direct_config(central, thessa, auron),
    )
    .expect("direct thessa->auron finds");
    println!(
        "stack pelagos->thessa + thessa->auron: exact dv {:.0} + {:.0} = {:.0} m/s in {:?}",
        first[0].exact_total_dv_mps,
        second[0].exact_total_dv_mps,
        first[0].exact_total_dv_mps + second[0].exact_total_dv_mps,
        started.elapsed(),
    );
}
