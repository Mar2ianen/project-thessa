//! Solar System mission replay: planner validation against flown transfer
//! classes on real data (data/solar_system.toml, approx J2000 phases).
//! Default run: broad surveys (milliseconds) for Earth->Mars/Venus/Jupiter/
//! Mercury/Saturn and Mars->Earth around the Sun, Apollo-class Earth->Luna
//! around Earth, plus an EXACT Luna revalidation (days-long arcs, seconds).
//! Literature anchors are transfer ENERGIES (Hohmann classes, C3/v_inf,
//! LEO-escape + arrival), not dates: phases are approximate, windows
//! design-relative.
//! THESSA_SOLAR_EXACT=1 adds exact Mars/Venus revalidations and the
//! Earth->Venus->Mercury flyby tour.
//!
//! Anchors (see validation log):
//! - Apollo: TLI ~3033 m/s from ~170 km parking (10_837 vs 7_804 m/s,
//!   AS-512), LOI1 ~912 + LOI2 ~42 m/s into 60x170 nmi, TOF ~66-73 h
//!   free-return (barely-bound ellipse, C3 ~ -1.7). Planner broad prices
//!   the LEO vector change; exact arrival nulls velocity at the 100 km
//!   rendezvous sphere (~2.4-2.5 km/s = sqrt(vinf^2 + vesc^2)), larger
//!   than LOI capture (~0.9) by ~lunar vcirc.
//! - Mars: optimal C3 7.8-14.8 km2/s2, arrival vinf 2.5-3.3 km/s
//!   (2026-2045 handbook, Type II); Hohmann 259 d, heliocentric 2.94 +
//!   2.64 km/s, LEO escape ~3.6 km/s.
//! - Venus: Hohmann 146 d; Mariner-class Earth->Venus ~90 d leg.
//! - Mariner 10 (1973): min C3 ~18.15 km2/s2 (vinf ~4.26), launch Oct-Nov
//!   1973 -> Venus Feb 5 1974 (~90 d) -> Mercury Mar 29 (~52 d); Venus
//!   flyby UNPOWERED (energy removed by gravity), TCM budget 122 m/s
//!   total (mean+3sigma max 120). Direct Mercury C3 ~40 vs swingby ~18.
//! - Jupiter: Hohmann 2.73 yr; Saturn 6.05 yr (direct, expensive —
//!   Cassini used VVEJ tour).
//!
//! Semantics: broad arrival = v_inf at the target well edge (compares to
//! handbook vinf/C3); exact arrival = rendezvous null at the standoff
//! sphere (sqrt(vinf^2 + vesc^2), compares to low-orbit capture + vcirc).
//! A km/s-scale exact TCM means the broad geometry missed (cf. Mariner
//! 122 m/s budget) — Mars TCM ~300-450 is borderline, Venus best ~1000
//! (93% plane change: departure phasing now scans parking-plane tilt so
//! the burn carries declination, but a single TCM still pays most of the
//! inclined-rendezvous plane price; arrival null itself matches
//! sqrt(vinf^2 + vesc^2) within ~4%).
use std::{env, hint::black_box, time::Instant};

use thessa_maneuver::{FlybyConfig, SearchConfig, broad_survey, flyby_search, porkchop_search};
use thessa_sim_core::{BakedEphemeris, BodyId, GravityField, SimTime, SystemConfig};

const DAY: f64 = 86_400.0;

#[allow(clippy::too_many_arguments)]
fn survey_config(
    central: BodyId,
    departure: BodyId,
    arrival: BodyId,
    span_d: f64,
    dep_steps: usize,
    tof_min_d: f64,
    tof_max_d: f64,
    tof_steps: usize,
    keep: usize,
    max_broad: f64,
) -> SearchConfig {
    SearchConfig {
        central_body: central,
        departure_body: departure,
        arrival_body: arrival,
        window_start: SimTime::EPOCH,
        departure_span_s: span_d * DAY,
        departure_steps: dep_steps,
        tof_min_s: tof_min_d * DAY,
        tof_max_s: tof_max_d * DAY,
        tof_steps,
        keep_candidates: keep,
        standoff_m: 100_000.0,
        max_broad_dv_mps: max_broad,
        max_miss_m: 1.0e6,
    }
}

fn report_route(
    name: &str,
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: SearchConfig,
    earth_departure: bool,
) {
    let started = Instant::now();
    match broad_survey(black_box(ephemeris), config) {
        Ok((routes, stats)) => {
            let winner = &routes[0];
            // C3/vinf for Earth-departure legs (hyperbolic only; Apollo
            // TLI is a barely-bound ellipse so C3 is negative/none).
            let c3_note = if earth_departure {
                earth_c3_note(ephemeris, winner)
            } else {
                String::new()
            };
            println!(
                "broad {name}: depart T+{:.0}d, tof {:.0}d, dv {:.0} m/s (dep {:.0} + arr {:.0}){}, {} cells in {:?}",
                (winner.departure_epoch.0 - SimTime::EPOCH.0) / DAY,
                winner.time_of_flight_s / DAY,
                winner.broad_total_dv_mps,
                winner.departure_burn_mag_mps,
                winner.arrival_burn_mag_mps,
                c3_note,
                stats.broad_evaluations,
                started.elapsed(),
            );
        }
        Err(error) => println!("broad {name} failed: {error}"),
    }
    let _ = field;
}

/// Earth-departure energy note: invert the patched escape to vinf/C3.
/// Returns "" for non-Earth departures or bound (elliptical) cases.
fn earth_c3_note(ephemeris: &BakedEphemeris, winner: &thessa_maneuver::BroadRoute) -> String {
    let earth = match ephemeris.body_id("earth") {
        Some(id) => id,
        None => return String::new(),
    };
    let body = match ephemeris.body(earth) {
        Ok(b) => b,
        Err(_) => return String::new(),
    };
    // Recompute park radius with the bench standoff (100 km).
    let park = body.radius_m + 100_000.0;
    let v_circ = (body.mu / park).sqrt();
    let v_esc_sq = 2.0 * body.mu / park;
    let v_hyp = winner.departure_burn_mag_mps + v_circ;
    let vinf_sq = v_hyp * v_hyp - v_esc_sq;
    if vinf_sq <= 0.0 {
        return " [bound ellipse, C3<0, Apollo-class]".to_string();
    }
    let vinf = vinf_sq.sqrt();
    format!(
        " [vinf {:.2} km/s, C3 {:.1}]",
        vinf / 1000.0,
        vinf * vinf / 1.0e6
    )
}

fn main() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/solar_system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    println!("sources: {}", field.source_count());
    let sun = ephemeris.body_id("sol").expect("sol");
    let earth = ephemeris.body_id("earth").expect("earth");
    let luna = ephemeris.body_id("luna").expect("luna");
    let mars = ephemeris.body_id("mars").expect("mars");
    let venus = ephemeris.body_id("venus").expect("venus");
    let jupiter = ephemeris.body_id("jupiter").expect("jupiter");
    let mercury = ephemeris.body_id("mercury").expect("mercury");
    let saturn = ephemeris.body_id("saturn").expect("saturn");

    // Hohmann-class references (LEO escape + arrival vinf):
    // Earth->Mars C3 ~8-15 / vinf-arr ~2.5-3.3, TOF ~259 d;
    // Earth->Venus ~3.5 + vinf ~2.7, TOF ~146 d;
    // Earth->Jupiter C3 ~80-100, TOF ~2.7 yr;
    // Earth->Mercury Hohmann total ~15.4k (dep ~5.5-7 + arr vinf ~9-10,
    // TOF ~105 d) — the handbook C3 ~40 is departure-only for an
    // aphelion arrival (arrival pays more); the planner minimizes total;
    // Earth->Saturn Hohmann total ~12.8k, TOF ~6 yr (Cassini flew VVEJ);
    // Apollo TLI ~3033 (bound) + LOI ~954, TOF ~2.75-3 d.
    report_route(
        "earth->mars",
        &ephemeris,
        &field,
        survey_config(sun, earth, mars, 800.0, 40, 150.0, 320.0, 30, 8, 12_000.0),
        true,
    );
    report_route(
        "earth->venus",
        &ephemeris,
        &field,
        survey_config(sun, earth, venus, 600.0, 40, 100.0, 220.0, 30, 8, 12_000.0),
        true,
    );
    report_route(
        "mars->earth",
        &ephemeris,
        &field,
        survey_config(sun, mars, earth, 800.0, 40, 150.0, 320.0, 30, 8, 12_000.0),
        false,
    );
    report_route(
        "earth->jupiter",
        &ephemeris,
        &field,
        survey_config(
            sun, earth, jupiter, 800.0, 30, 700.0, 1_100.0, 30, 8, 20_000.0,
        ),
        true,
    );
    // Mariner-class legs: direct Mercury (Hohmann-total reference ~15.4k;
    // the Venus swingby's prize is the DEPARTURE side: swingby C3 ~18 vs
    // direct min-departure C3 ~40 for aphelion arrival) and Saturn direct
    // (Hohmann total ~12.8k — shows why outer missions fly assists).
    // Mercury moves fast (88 d period): needs a fine departure grid or
    // windows step over.
    report_route(
        "earth->mercury-direct",
        &ephemeris,
        &field,
        survey_config(sun, earth, mercury, 600.0, 80, 80.0, 200.0, 40, 8, 25_000.0),
        true,
    );
    report_route(
        "earth->saturn-direct",
        &ephemeris,
        &field,
        survey_config(
            sun, earth, saturn, 1_200.0, 30, 1_800.0, 2_500.0, 30, 4, 30_000.0,
        ),
        true,
    );
    // Fine grid: TLI windows are hours-wide inside each month; a coarse
    // grid steps over them (cells degenerate or over cap).
    report_route(
        "earth->luna",
        &ephemeris,
        &field,
        survey_config(
            ephemeris.body_id("earth").expect("earth"),
            earth,
            luna,
            30.0,
            120,
            3.0,
            7.0,
            40,
            4,
            15_000.0,
        ),
        true,
    );

    // EXACT Luna revalidation (days-long arcs — cheap, always on).
    let started = Instant::now();
    match porkchop_search(
        black_box(&ephemeris),
        black_box(&field),
        SearchConfig {
            keep_candidates: 2,
            ..survey_config(
                ephemeris.body_id("earth").expect("earth"),
                earth,
                luna,
                30.0,
                120,
                3.0,
                7.0,
                40,
                2,
                15_000.0,
            )
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
                "exact earth->luna: dv {:.0} m/s [{}], miss {:.1} km, {} newton props in {:?}",
                winner.exact_total_dv_mps,
                nodes.join("+"),
                winner.exact_miss_m / 1_000.0,
                stats.newton_propagations,
                started.elapsed(),
            );
        }
        Err(error) => println!("exact earth->luna failed: {error}"),
    }

    if env::var("THESSA_SOLAR_EXACT").is_ok() {
        // Exact outer-planet revalidations (minute-class each).
        for (name, dep, arr, span, tof_min, tof_max) in [
            ("earth->mars", earth, mars, 800.0, 150.0, 320.0),
            ("earth->venus", earth, venus, 600.0, 100.0, 220.0),
        ] {
            let started = Instant::now();
            match porkchop_search(
                black_box(&ephemeris),
                black_box(&field),
                SearchConfig {
                    keep_candidates: 1,
                    max_miss_m: 1.0e8,
                    ..survey_config(sun, dep, arr, span, 40, tof_min, tof_max, 30, 1, 15_000.0)
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
                        "exact {name}: dv {:.0} m/s [{}], miss {:.1} km, {} newton props in {:?}",
                        winner.exact_total_dv_mps,
                        nodes.join("+"),
                        winner.exact_miss_m / 1_000.0,
                        stats.newton_propagations,
                        started.elapsed(),
                    );
                }
                Err(error) => println!("exact {name} failed: {error}"),
            }
        }
        // Gravity-assist tour Earth->Venus->Mercury (Mariner-10 class:
        // launch C3 ~18, Earth->Venus ~90 d + Venus->Mercury ~52 d,
        // Venus burn ~0 unpowered). Interplanetary terminal scale:
        // miss filter 1e8 m (not lunar 1e6), broad cap 25k (deep-well
        // turn floor + Mercury arrival), finer grid for the 7-deg plane
        // change. Validates against direct-Mercury above (swingby must
        // beat direct C3 ~40).
        let started = Instant::now();
        match flyby_search(
            black_box(&ephemeris),
            black_box(&field),
            FlybyConfig {
                central_body: sun,
                departure_body: earth,
                arrival_body: mercury,
                flyby_bodies: vec![venus],
                window_start: SimTime::EPOCH,
                departure_span_s: 600.0 * DAY,
                departure_steps: 24,
                leg1_min_s: 80.0 * DAY,
                leg1_max_s: 220.0 * DAY,
                leg1_steps: 8,
                leg2_min_s: 40.0 * DAY,
                leg2_max_s: 200.0 * DAY,
                leg2_steps: 8,
                keep_routes: 3,
                standoff_m: 100_000.0,
                max_broad_dv_mps: 25_000.0,
                max_miss_m: 1.0e8,
            },
        ) {
            Ok((ranked, stats)) => {
                let winner = &ranked[0];
                let event = winner.plan.flybys[0];
                let nodes: Vec<String> = winner
                    .plan
                    .nodes
                    .iter()
                    .map(|node| format!("{:.0}", node.magnitude_mps()))
                    .collect();
                println!(
                    "tour earth->venus->mercury: exact dv {:.0} m/s [{}] (broad {:.0}), miss {:.1} km, venus burn {:.0} m/s at rp {:.0} km, legs {:.0}d + {:.0}d, {} newton props in {:?}",
                    winner.exact_total_dv_mps,
                    nodes.join("+"),
                    winner.broad_total_dv_mps,
                    winner.exact_miss_m / 1_000.0,
                    event.burn_mps,
                    event.periapsis_m / 1_000.0,
                    (event.epoch.0 - winner.departure_epoch.0) / DAY,
                    winner.time_of_flight_s / DAY
                        - (event.epoch.0 - winner.departure_epoch.0) / DAY,
                    stats.newton_propagations,
                    started.elapsed(),
                );
                // Mariner anchor: Venus burn ~0 (unpowered, max free bend
                // ~100 deg at vinf 4 km/s). A km/s-scale burn = powered
                // tour (wrong window/geometry), not the flown free assist.
                if event.burn_mps > 1_000.0 {
                    println!(
                        "  NOTE: powered tour (venus {:.0} m/s >> Mariner ~0): free-assist window not resolved with approx phases + coarse grid — see direct-mercury reference above",
                        event.burn_mps
                    );
                }
            }
            Err(error) => println!("tour earth->venus->mercury failed: {error}"),
        }
    } else {
        println!("exact outer-planet + tour revalidation skipped (set THESSA_SOLAR_EXACT=1)");
    }
}
