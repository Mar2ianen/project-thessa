//! Mission replay fixtures (docs/07 section 7.14, fidelity L0-L2).
//!
//! Data-driven broad-survey qualification on the real Solar System:
//! encounter order (L0), design-relative windows and leg TOF class (L1),
//! and transfer energy class (L2) against flown-mission anchors. Exact
//! N-body revalidation and live-authority agreement are L3-L4 and live in
//! slower jobs (the `solar_system` bench); this test must stay
//! millisecond-class for CI.

use std::collections::BTreeMap;

use thessa_maneuver::{
    BroadChainRoute, BroadFlybyRoute, BroadRoute, ChainConfig, ChainEncounter, EncounterKind,
    FlybyConfig, SearchConfig, broad_chain_survey, broad_flyby_survey, broad_survey,
};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};

const DAY: f64 = 86_400.0;

#[derive(Debug, serde::Deserialize)]
struct Fixtures {
    #[serde(default)]
    direct: Vec<DirectFixture>,
    #[serde(default)]
    tour: Vec<TourFixture>,
    #[serde(default)]
    chain: Vec<ChainFixture>,
}

#[derive(Debug, serde::Deserialize)]
struct DirectFixture {
    name: String,
    central: String,
    departure: String,
    arrival: String,
    departure_span_d: f64,
    departure_steps: usize,
    tof_min_d: f64,
    tof_max_d: f64,
    tof_steps: usize,
    keep: usize,
    max_broad_dv_mps: f64,
    max_winner_total_dv_mps: f64,
    max_winner_departure_dv_mps: f64,
    max_winner_arrival_vinf_mps: f64,
    winner_tof_min_d: f64,
    winner_tof_max_d: f64,
}

#[derive(Debug, serde::Deserialize)]
struct TourFixture {
    name: String,
    central: String,
    departure: String,
    flyby: Vec<String>,
    arrival: String,
    departure_span_d: f64,
    departure_steps: usize,
    leg1_min_d: f64,
    leg1_max_d: f64,
    leg1_steps: usize,
    leg2_min_d: f64,
    leg2_max_d: f64,
    leg2_steps: usize,
    keep: usize,
    max_broad_dv_mps: f64,
    max_winner_total_dv_mps: f64,
    max_winner_departure_dv_mps: f64,
    max_winner_turn_dv_mps: f64,
    winner_leg1_min_d: f64,
    winner_leg1_max_d: f64,
    winner_leg2_min_d: f64,
    winner_leg2_max_d: f64,
}

fn load_system() -> BakedEphemeris {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/solar_system.toml")).expect("solar system");
    config.bake().expect("solar bake")
}

fn load_fixtures() -> Fixtures {
    toml::from_str(include_str!("../../../data/mission_replays.toml")).expect("replay fixtures")
}

fn body_ids(ephemeris: &BakedEphemeris, names: &[&str]) -> BTreeMap<String, BodyId> {
    names
        .iter()
        .map(|name| {
            (
                (*name).to_string(),
                ephemeris.body_id(name).expect("fixture body exists"),
            )
        })
        .collect()
}

fn check_direct(ephemeris: &BakedEphemeris, fixture: &DirectFixture) {
    let ids = body_ids(
        ephemeris,
        &[&fixture.central, &fixture.departure, &fixture.arrival],
    );
    let (routes, stats) = broad_survey(
        ephemeris,
        SearchConfig {
            central_body: ids[&fixture.central],
            departure_body: ids[&fixture.departure],
            arrival_body: ids[&fixture.arrival],
            window_start: SimTime::EPOCH,
            departure_span_s: fixture.departure_span_d * DAY,
            departure_steps: fixture.departure_steps,
            tof_min_s: fixture.tof_min_d * DAY,
            tof_max_s: fixture.tof_max_d * DAY,
            tof_steps: fixture.tof_steps,
            keep_candidates: fixture.keep,
            standoff_m: 100_000.0,
            max_broad_dv_mps: fixture.max_broad_dv_mps,
            max_miss_m: 1.0e6,
        },
    )
    .unwrap_or_else(|error| panic!("{}: survey finds routes: {error}", fixture.name));
    assert!(
        !routes.is_empty(),
        "{}: at least one route (L0 connectivity)",
        fixture.name
    );
    let winner: &BroadRoute = &routes[0];
    let tof_d = winner.time_of_flight_s / DAY;
    println!(
        "replay {}: T+{:.0}d tof {:.0}d dep {:.0} arr {:.0} tot {:.0} ({} cells)",
        fixture.name,
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / DAY,
        tof_d,
        winner.departure_burn_mag_mps,
        winner.arrival_burn_mag_mps,
        winner.broad_total_dv_mps,
        stats.broad_evaluations,
    );
    assert!(
        tof_d >= fixture.winner_tof_min_d && tof_d <= fixture.winner_tof_max_d,
        "{}: TOF class {tof_d:.0}d outside [{}, {}]",
        fixture.name,
        fixture.winner_tof_min_d,
        fixture.winner_tof_max_d,
    );
    assert!(
        winner.broad_total_dv_mps <= fixture.max_winner_total_dv_mps,
        "{}: total {:.0} above cap {:.0}",
        fixture.name,
        winner.broad_total_dv_mps,
        fixture.max_winner_total_dv_mps,
    );
    assert!(
        winner.departure_burn_mag_mps <= fixture.max_winner_departure_dv_mps,
        "{}: departure {:.0} above cap {:.0}",
        fixture.name,
        winner.departure_burn_mag_mps,
        fixture.max_winner_departure_dv_mps,
    );
    assert!(
        winner.arrival_burn_mag_mps <= fixture.max_winner_arrival_vinf_mps,
        "{}: arrival v_inf {:.0} above cap {:.0}",
        fixture.name,
        winner.arrival_burn_mag_mps,
        fixture.max_winner_arrival_vinf_mps,
    );
}

fn check_tour(ephemeris: &BakedEphemeris, fixture: &TourFixture) {
    let mut names: Vec<&str> = vec![&fixture.central, &fixture.departure, &fixture.arrival];
    names.extend(fixture.flyby.iter().map(String::as_str));
    let ids = body_ids(ephemeris, &names);
    let (routes, stats) = broad_flyby_survey(
        ephemeris,
        FlybyConfig {
            central_body: ids[&fixture.central],
            departure_body: ids[&fixture.departure],
            arrival_body: ids[&fixture.arrival],
            flyby_bodies: fixture.flyby.iter().map(|name| ids[name as &str]).collect(),
            window_start: SimTime::EPOCH,
            departure_span_s: fixture.departure_span_d * DAY,
            departure_steps: fixture.departure_steps,
            leg1_min_s: fixture.leg1_min_d * DAY,
            leg1_max_s: fixture.leg1_max_d * DAY,
            leg1_steps: fixture.leg1_steps,
            leg2_min_s: fixture.leg2_min_d * DAY,
            leg2_max_s: fixture.leg2_max_d * DAY,
            leg2_steps: fixture.leg2_steps,
            keep_routes: fixture.keep,
            standoff_m: 100_000.0,
            max_broad_dv_mps: fixture.max_broad_dv_mps,
            max_miss_m: 1.0e8,
        },
    )
    .unwrap_or_else(|error| panic!("{}: tour finds routes: {error}", fixture.name));
    assert!(
        !routes.is_empty(),
        "{}: at least one tour route (L0 encounter order closes)",
        fixture.name
    );
    let winner: &BroadFlybyRoute = &routes[0];
    let leg1_d = winner.tof_leg1_s / DAY;
    let leg2_d = winner.tof_leg2_s / DAY;
    println!(
        "replay {}: T+{:.0}d legs {:.0}d+{:.0}d dep {:.0} turn {:.0} arr {:.0} tot {:.0} ({} cells)",
        fixture.name,
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / DAY,
        leg1_d,
        leg2_d,
        winner.departure_burn_mag_mps,
        winner.turn_burn_mag_mps,
        winner.arrival_burn_mag_mps,
        winner.broad_total_dv_mps,
        stats.broad_evaluations,
    );
    assert!(
        leg1_d >= fixture.winner_leg1_min_d && leg1_d <= fixture.winner_leg1_max_d,
        "{}: leg1 {leg1_d:.0}d outside family",
        fixture.name,
    );
    assert!(
        leg2_d >= fixture.winner_leg2_min_d && leg2_d <= fixture.winner_leg2_max_d,
        "{}: leg2 {leg2_d:.0}d outside family",
        fixture.name,
    );
    assert!(
        winner.broad_total_dv_mps <= fixture.max_winner_total_dv_mps,
        "{}: total {:.0} above cap {:.0}",
        fixture.name,
        winner.broad_total_dv_mps,
        fixture.max_winner_total_dv_mps,
    );
    assert!(
        winner.departure_burn_mag_mps <= fixture.max_winner_departure_dv_mps,
        "{}: departure {:.0} above cap {:.0}",
        fixture.name,
        winner.departure_burn_mag_mps,
        fixture.max_winner_departure_dv_mps,
    );
    assert!(
        winner.turn_burn_mag_mps <= fixture.max_winner_turn_dv_mps,
        "{}: turn {:.0} above cap {:.0}",
        fixture.name,
        winner.turn_burn_mag_mps,
        fixture.max_winner_turn_dv_mps,
    );
}

#[test]
fn mission_replay_fixtures_hold_l0_l1_l2() {
    let ephemeris = load_system();
    let fixtures = load_fixtures();
    assert!(!fixtures.direct.is_empty(), "direct fixtures exist");
    assert!(!fixtures.tour.is_empty(), "tour fixtures exist");
    assert!(!fixtures.chain.is_empty(), "chain fixtures exist");
    for fixture in &fixtures.direct {
        check_direct(&ephemeris, fixture);
    }
    for fixture in &fixtures.tour {
        check_tour(&ephemeris, fixture);
    }
    for fixture in &fixtures.chain {
        check_chain(&ephemeris, fixture);
    }
}

#[derive(Debug, serde::Deserialize)]
struct ChainFixtureEncounter {
    body: String,
    kind: String,
    standoff_m: Option<f64>,
}

#[derive(Debug, serde::Deserialize)]
struct ChainFixture {
    name: String,
    central: String,
    departure: String,
    encounters: Vec<ChainFixtureEncounter>,
    #[serde(default)]
    window_start_d: Option<f64>,
    departure_span_d: f64,
    departure_steps: usize,
    leg_tof_min_d: Vec<f64>,
    leg_tof_max_d: Vec<f64>,
    leg_tof_steps: Vec<usize>,
    keep: usize,
    max_broad_dv_mps: f64,
    max_winner_total_dv_mps: f64,
    max_winner_departure_dv_mps: f64,
    max_winner_turn_dv_mps: f64,
    winner_leg_tof_min_d: Vec<f64>,
    winner_leg_tof_max_d: Vec<f64>,
}

fn check_chain(ephemeris: &BakedEphemeris, fixture: &ChainFixture) {
    let mut names: Vec<&str> = vec![&fixture.central, &fixture.departure];
    names.extend(
        fixture
            .encounters
            .iter()
            .map(|encounter| encounter.body.as_str()),
    );
    let ids = body_ids(ephemeris, &names);
    let encounters: Vec<ChainEncounter> = fixture
        .encounters
        .iter()
        .map(|encounter| ChainEncounter {
            body: ids[&encounter.body],
            kind: match encounter.kind.as_str() {
                "flyby" => EncounterKind::Flyby,
                "rendezvous" => EncounterKind::Rendezvous,
                other => panic!("{}: unknown encounter kind {other}", fixture.name),
            },
            standoff_m: encounter.standoff_m,
        })
        .collect();
    let legs = encounters.len();
    let (routes, stats) = broad_chain_survey(
        ephemeris,
        ChainConfig {
            central_body: ids[&fixture.central],
            departure_body: ids[&fixture.departure],
            encounters,
            window_start: SimTime(SimTime::EPOCH.0 + fixture.window_start_d.unwrap_or(0.0) * DAY),
            departure_span_s: fixture.departure_span_d * DAY,
            departure_steps: fixture.departure_steps,
            leg_tof_min_s: fixture
                .leg_tof_min_d
                .iter()
                .map(|days| days * DAY)
                .collect(),
            leg_tof_max_s: fixture
                .leg_tof_max_d
                .iter()
                .map(|days| days * DAY)
                .collect(),
            leg_tof_steps: fixture.leg_tof_steps.clone(),
            keep_routes: fixture.keep,
            standoff_m: 100_000.0,
            max_broad_dv_mps: fixture.max_broad_dv_mps,
            max_miss_m: 1.0e8,
        },
    )
    .unwrap_or_else(|error| panic!("{}: chain finds routes: {error}", fixture.name));
    assert!(
        !routes.is_empty(),
        "{}: at least one chain route (L0 encounter order closes)",
        fixture.name
    );
    let winner: &BroadChainRoute = &routes[0];
    assert_eq!(
        winner.encounter_bodies.len(),
        legs,
        "{}: winner visits every encounter in order",
        fixture.name
    );
    assert_eq!(winner.leg_tofs_s.len(), legs);
    assert_eq!(winner.flyby_burns_mag_mps.len(), legs - 1);
    let legs_str: Vec<String> = winner
        .leg_tofs_s
        .iter()
        .map(|tof| format!("{:.0}", tof / DAY))
        .collect();
    let turns_str: Vec<String> = winner
        .flyby_burns_mag_mps
        .iter()
        .map(|mag| format!("{mag:.0}"))
        .collect();
    println!(
        "replay {}: T+{:.0}d legs [{}]d dep {:.0} turns [{}] arr {:.0} tot {:.0} out {:.0} ({} cells)",
        fixture.name,
        (winner.departure_epoch.0 - SimTime::EPOCH.0) / DAY,
        legs_str.join("+"),
        winner.departure_burn_mag_mps,
        turns_str.join("+"),
        winner.arrival_burn_mag_mps,
        winner.broad_total_dv_mps,
        winner.final_outgoing_speed_mps,
        stats.broad_evaluations,
    );
    for (index, tof) in winner.leg_tofs_s.iter().enumerate() {
        let days = tof / DAY;
        assert!(
            days >= fixture.winner_leg_tof_min_d[index]
                && days <= fixture.winner_leg_tof_max_d[index],
            "{}: leg{index} {days:.0}d outside family",
            fixture.name,
        );
    }
    assert!(
        winner.broad_total_dv_mps <= fixture.max_winner_total_dv_mps,
        "{}: total {:.0} above cap {:.0}",
        fixture.name,
        winner.broad_total_dv_mps,
        fixture.max_winner_total_dv_mps,
    );
    assert!(
        winner.departure_burn_mag_mps <= fixture.max_winner_departure_dv_mps,
        "{}: departure {:.0} above cap {:.0}",
        fixture.name,
        winner.departure_burn_mag_mps,
        fixture.max_winner_departure_dv_mps,
    );
    let turn_sum: f64 = winner.flyby_burns_mag_mps.iter().sum();
    assert!(
        turn_sum <= fixture.max_winner_turn_dv_mps,
        "{}: turn sum {turn_sum:.0} above cap {:.0}",
        fixture.name,
        fixture.max_winner_turn_dv_mps,
    );
}
