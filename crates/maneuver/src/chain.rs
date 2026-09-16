//! Chained gravity-assist tour search: departure body -> N intermediate
//! flybys -> final flyby or rendezvous around one central body, with exact
//! N-body revalidation.
//!
//! This is the multi-encounter generalization of [`flyby_search`](crate::flyby_search)
//! (which handles exactly one assist). It exists for Grand-Tour-class
//! missions (Voyager: Earth -> Jupiter -> Saturn -> Uranus -> Neptune)
//! where errors compound across encounters and no single-assist planner
//! can discover the chain.
//!
//! Anti-cheat contract (docs/07 §7.14): the search discovers encounter
//! epochs, times of flight and bend geometry itself inside broad
//! declarative domains (departure span, per-leg TOF ranges). There are no
//! per-mission code paths, no hand-fed dates, no scripted burns. A mission
//! fixture declares bodies, domains and tolerances; the same code must
//! serve any chain.
//!
//! Model (patched-conic broad, exact narrow — docs/23 §12 pattern):
//! - Broad: N Lambert arcs sharing encounter-body positions at encounter
//!   epochs. Departure is priced as patched parking-orbit escape (same
//!   helper as the direct search), each flyby as a powered bend at
//!   periapsis (periapsis-energy floored lower bound), a rendezvous final
//!   as a standoff match. Bend limits are REPORTED, not enforced.
//! - Narrow: leg 1 is anomaly-phased and differentially corrected to the
//!   first aim point exactly like a direct arrival; each subsequent leg
//!   SOLVES the flyby burn (seeded by the broad bend) to hit the next aim
//!   point with full N-body dynamics inside every evaluation — well depth,
//!   focusing and handoff slop are absorbed into the solved burns, so no
//!   downstream TCMs are needed. A rendezvous final adds the arrival match;
//!   a flyby final ends at the encounter with a measured periapsis miss
//!   and a reported outgoing central-frame speed (escape proxy).
//! - Intermediate encounters must be flybys: a stop-and-go rendezvous is
//!   two separate transfers, not a chain (rejected as invalid config).

use glam::DVec3;
use thessa_sim_core::{BakedEphemeris, BodyId, GravityField, SimTime};

use crate::lambert::solve_lambert_prograde;
use crate::patch::{planet_arrival_match_mag, planet_escape_moon_vinf, planet_of};
use crate::plan::{FlybyEvent, ManeuverNode, ManeuverPlan};
use crate::search::{
    RankedPlan, SearchError, SearchStats, correct_shooting, midcourse_time_s, patched_escape_mag,
    phase_departure_topk, transfer_perigee_m,
};

/// How a chain encounter is treated at exact level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncounterKind {
    /// Swing by at the standoff periapsis: burn only as needed for the
    /// turn, no arrival match. The only legal intermediate kind.
    Flyby,
    /// Null the arrival velocity at the standoff sphere. Final only.
    Rendezvous,
}

/// One encounter in a chain: which body, treated how, and how deep.
/// The standoff override exists because one size does not fit all wells:
/// a 100 km aim sphere at Jupiter allows ~2° of free bend at v_inf 10
/// km/s (the assist is then powered by construction), while the flown
/// Voyager 1 passage at ~4 Jovian radii allows ~110°. Closest-approach
/// class is a first-class replay milestone (docs/07 §7.14); fixtures
/// declare it per encounter instead of the planner hiding it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainEncounter {
    pub body: BodyId,
    pub kind: EncounterKind,
    /// Periapsis altitude override (m). `None` uses the config standoff.
    pub standoff_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ChainConfig {
    /// Body hosting the two-body Lambert model for every leg.
    pub central_body: BodyId,
    /// Depot body: the craft departs with its full inertial state.
    pub departure_body: BodyId,
    /// Ordered encounters (at least one). Intermediates must be flybys.
    pub encounters: Vec<ChainEncounter>,
    pub window_start: SimTime,
    pub departure_span_s: f64,
    pub departure_steps: usize,
    /// Per-leg time-of-flight ranges, one entry per encounter in order.
    pub leg_tof_min_s: Vec<f64>,
    pub leg_tof_max_s: Vec<f64>,
    pub leg_tof_steps: Vec<usize>,
    /// Routes carried into exact revalidation (and reported by the survey).
    pub keep_routes: usize,
    /// Parking-orbit / flyby-periapsis / rendezvous-approach altitude (m).
    pub standoff_m: f64,
    /// Broad routes above this total Δv are pruned, not ranked.
    pub max_broad_dv_mps: f64,
    /// Plans missing by more than this are dropped, not ranked.
    pub max_miss_m: f64,
}

impl ChainConfig {
    fn validate(&self) -> Result<(), SearchError> {
        if self.encounters.is_empty()
            || self.keep_routes == 0
            || self.departure_steps == 0
            || !self.window_start.0.is_finite()
            || !self.departure_span_s.is_finite()
            || self.departure_span_s < 0.0
            || !self.standoff_m.is_finite()
            || self.standoff_m <= 0.0
            || !self.max_broad_dv_mps.is_finite()
            || self.max_broad_dv_mps <= 0.0
            || !self.max_miss_m.is_finite()
            || self.max_miss_m < 0.0
            || self.leg_tof_min_s.len() != self.encounters.len()
            || self.leg_tof_max_s.len() != self.encounters.len()
            || self.leg_tof_steps.len() != self.encounters.len()
        {
            return Err(SearchError::InvalidConfig);
        }
        for (index, encounter) in self.encounters.iter().enumerate() {
            let last = index + 1 == self.encounters.len();
            if !last && encounter.kind != EncounterKind::Flyby {
                // A mid-chain rendezvous is two transfers, not a chain.
                return Err(SearchError::InvalidConfig);
            }
            if !self.leg_tof_min_s[index].is_finite()
                || self.leg_tof_min_s[index] <= 0.0
                || !self.leg_tof_max_s[index].is_finite()
                || self.leg_tof_max_s[index] < self.leg_tof_min_s[index]
                || self.leg_tof_steps[index] == 0
            {
                return Err(SearchError::InvalidConfig);
            }
        }
        Ok(())
    }

    fn legs(&self) -> usize {
        self.encounters.len()
    }
}

/// One broad-phase chain route: pure patched-conic scouting with NO exact
/// revalidation and NO miss measurement. Route selection only; flyable
/// plans come from [`chain_search`]. Cannot feed the executor.
#[derive(Debug, Clone, PartialEq)]
pub struct BroadChainRoute {
    pub departure_epoch: SimTime,
    pub time_of_flight_s: f64,
    /// Encounter bodies in order (flybys then the final).
    pub encounter_bodies: Vec<BodyId>,
    /// Per-leg times of flight in order.
    pub leg_tofs_s: Vec<f64>,
    pub departure_burn_mag_mps: f64,
    /// Powered-bend prices per intermediate flyby, in order.
    pub flyby_burns_mag_mps: Vec<f64>,
    /// Rendezvous-match price (0 when the final is a flyby).
    pub arrival_burn_mag_mps: f64,
    /// Central-frame speed leaving the final encounter: escape proxy for
    /// flyby finals, matched body speed for rendezvous finals.
    pub final_outgoing_speed_mps: f64,
    pub broad_total_dv_mps: f64,
}

#[derive(Clone)]
struct ChainCell {
    departure_epoch: SimTime,
    leg_tofs_s: Vec<f64>,
    encounter_bodies: Vec<BodyId>,
    departure_burn_mag_mps: f64,
    /// Desired outgoing asymptotes per intermediate flyby (broad seeds).
    v_out_frames_mps: Vec<DVec3>,
    turn_mags_mps: Vec<f64>,
    arrival_mag_mps: f64,
    final_outgoing_speed_mps: f64,
    total_dv: f64,
}

struct ChainCtx<'a> {
    ephemeris: &'a BakedEphemeris,
    config: ChainConfig,
    central_mu: f64,
    central_radius_m: f64,
    depot_mu: f64,
    depot_radius_m: f64,
    departure_planet: Option<BodyId>,
    arrival_planet: Option<BodyId>,
}

fn lerp_range(min: f64, max: f64, steps: usize, index: usize) -> f64 {
    if steps <= 1 {
        min
    } else {
        min + (max - min) * index as f64 / (steps - 1) as f64
    }
}

/// Aim-sphere altitude for one encounter: per-encounter override or the
/// config default (which always serves the departure parking orbit).
fn encounter_standoff(config: &ChainConfig, encounter: &ChainEncounter) -> f64 {
    encounter.standoff_m.unwrap_or(config.standoff_m)
}

fn chain_context<'a>(
    ephemeris: &'a BakedEphemeris,
    config: &ChainConfig,
) -> Result<ChainCtx<'a>, SearchError> {
    config.validate()?;
    let central = ephemeris
        .body(config.central_body)
        .map_err(SearchError::Ephemeris)?;
    if !central.mu.is_finite() || central.mu <= 0.0 {
        return Err(SearchError::InvalidConfig);
    }
    let depot = ephemeris
        .body(config.departure_body)
        .map_err(SearchError::Ephemeris)?;
    Ok(ChainCtx {
        ephemeris,
        config: config.clone(),
        central_mu: central.mu,
        central_radius_m: central.radius_m,
        depot_mu: depot.mu,
        depot_radius_m: depot.radius_m,
        departure_planet: planet_of(ephemeris, config.central_body, config.departure_body),
        arrival_planet: planet_of(
            ephemeris,
            config.central_body,
            config.encounters.last().expect("nonempty").body,
        ),
    })
}

/// Price one chain cell: N Lambert arcs through the encounter bodies at
/// cumulative epochs. Returns `None` for degenerate arcs, over-cap
/// totals, or central-body impacts (with stats counted like the direct
/// and single-flyby grids).
fn chain_cell(
    ctx: &ChainCtx<'_>,
    departure_epoch: SimTime,
    leg_tofs_s: &[f64],
    stats: &mut SearchStats,
) -> Option<ChainCell> {
    let ephemeris = ctx.ephemeris;
    let config = &ctx.config;
    let legs = config.legs();
    debug_assert_eq!(leg_tofs_s.len(), legs);
    // Encounter epochs and central-relative states.
    let mut epochs = Vec::with_capacity(legs + 1);
    epochs.push(departure_epoch);
    for tof in leg_tofs_s {
        epochs.push(SimTime(epochs.last().expect("nonempty").0 + tof));
    }
    let central_of = |epoch: SimTime| ephemeris.body_state(config.central_body, epoch).ok();
    let body_of = |body: BodyId, epoch: SimTime| ephemeris.body_state(body, epoch).ok();
    let departure = body_of(config.departure_body, epochs[0])?;
    let central_dep = central_of(epochs[0])?;
    if !central_dep.position_inertial.is_finite() {
        stats.degenerate_cells += 1;
        return None;
    }
    // Central-relative positions/velocities: index 0 is the departure
    // body, 1..=legs are the encounters in order.
    let mut rel_pos = Vec::with_capacity(legs + 1);
    let mut rel_vel = Vec::with_capacity(legs + 1);
    rel_pos.push(departure.position_inertial - central_dep.position_inertial);
    rel_vel.push(departure.velocity_inertial - central_dep.velocity_inertial);
    for (index, encounter) in config.encounters.iter().enumerate() {
        let epoch = epochs[index + 1];
        let central = central_of(epoch)?;
        let body = body_of(encounter.body, epoch)?;
        if !central.position_inertial.is_finite() {
            stats.degenerate_cells += 1;
            return None;
        }
        rel_pos.push(body.position_inertial - central.position_inertial);
        rel_vel.push(body.velocity_inertial - central.velocity_inertial);
    }
    // One Lambert arc per leg; leg k references the k-th body's own
    // motion for the prograde branch, like the single-flyby grid.
    let park_radius = ctx.depot_radius_m + config.standoff_m;
    let mut arcs_dep = Vec::with_capacity(legs);
    let mut arcs_arr = Vec::with_capacity(legs);
    for leg in 0..legs {
        if leg == 0 && rel_pos[0].cross(rel_vel[0]).length_squared() <= 0.0 {
            // Radial depot trajectory: no parking-orbit plane to phase in.
            stats.degenerate_cells += 1;
            return None;
        }
        let arc = match solve_lambert_prograde(
            rel_pos[leg],
            rel_vel[leg],
            rel_pos[leg + 1],
            leg_tofs_s[leg],
            ctx.central_mu,
        ) {
            Ok(arc) => arc,
            Err(_) => {
                stats.degenerate_cells += 1;
                return None;
            }
        };
        if transfer_perigee_m(rel_pos[leg], arc.departure_velocity_mps, ctx.central_mu)
            < ctx.central_radius_m * 1.05
        {
            stats.impact_cells += 1;
            return None;
        }
        arcs_dep.push(arc.departure_velocity_mps);
        arcs_arr.push(arc.arrival_velocity_mps);
    }
    // Departure pricing: planet patch when the depot sits behind a planet
    // well, patched parking-orbit escape otherwise.
    let dep_mag = match ctx.departure_planet {
        None => {
            let v_inf = (arcs_dep[0] - rel_vel[0]).length();
            match patched_escape_mag(ctx.depot_mu, park_radius, v_inf) {
                Some(mag) => mag,
                None => {
                    stats.degenerate_cells += 1;
                    return None;
                }
            }
        }
        Some(planet) => {
            let (planet_state, planet_mu) = match (
                ephemeris.body_state(planet, epochs[0]),
                ephemeris.body(planet),
            ) {
                (Ok(state), Ok(body)) => (state, body.mu),
                _ => {
                    stats.degenerate_cells += 1;
                    return None;
                }
            };
            let moon_rel_pos = departure.position_inertial - planet_state.position_inertial;
            let moon_rel_vel = departure.velocity_inertial - planet_state.velocity_inertial;
            let v_inf_planet = arcs_dep[0] - planet_state.velocity_inertial;
            match planet_escape_moon_vinf(v_inf_planet, moon_rel_pos, moon_rel_vel, planet_mu) {
                Some(v_inf_moon) => {
                    match patched_escape_mag(ctx.depot_mu, park_radius, v_inf_moon.length()) {
                        Some(mag) => mag,
                        None => {
                            stats.degenerate_cells += 1;
                            return None;
                        }
                    }
                }
                None => {
                    stats.degenerate_cells += 1;
                    return None;
                }
            }
        }
    };
    // Bend pass over consecutive arc pairs: incoming vs outgoing asymptote
    // in each intermediate encounter frame, periapsis-energy floored.
    let mut turns = Vec::with_capacity(legs.saturating_sub(1));
    let mut seeds = Vec::with_capacity(legs.saturating_sub(1));
    for leg in 1..legs {
        let encounter = &config.encounters[leg - 1];
        let body_id = encounter.body;
        let flyby_mu = ephemeris.body(body_id).ok()?.mu;
        let flyby_radius = ephemeris.body(body_id).ok()?.radius_m;
        let v_in_f = arcs_arr[leg - 1] - rel_vel[leg];
        let v_out_f = arcs_dep[leg] - rel_vel[leg];
        if v_in_f.length_squared() <= 0.0 || v_out_f.length_squared() <= 0.0 {
            stats.degenerate_cells += 1;
            return None;
        }
        let mut turn_price = (v_out_f - v_in_f).length();
        let periapsis_m = flyby_radius + encounter_standoff(config, encounter);
        if flyby_mu > 0.0 && periapsis_m > 0.0 {
            let v_esc_sq = 2.0 * flyby_mu / periapsis_m;
            let q_in = (v_in_f.length_squared() + v_esc_sq).sqrt();
            let q_out = (v_out_f.length_squared() + v_esc_sq).sqrt();
            if q_in.is_finite() && q_out.is_finite() {
                turn_price = turn_price.max((q_out - q_in).abs());
            }
        }
        if !turn_price.is_finite() {
            stats.degenerate_cells += 1;
            return None;
        }
        turns.push(turn_price);
        seeds.push(v_out_f);
    }
    // Final pricing.
    let final_encounter = config.encounters.last().expect("nonempty");
    let (arrival_mag, final_speed) = match final_encounter.kind {
        EncounterKind::Rendezvous => {
            let arr_mag = match ctx.arrival_planet {
                None => {
                    let arr_burn = rel_vel[legs] - arcs_arr[legs - 1];
                    if !arr_burn.is_finite() {
                        stats.degenerate_cells += 1;
                        return None;
                    }
                    arr_burn.length()
                }
                Some(planet) => {
                    let arrival_epoch = epochs[legs];
                    let (planet_state, planet_mu, moon_mu, moon_radius) = match (
                        ephemeris.body_state(planet, arrival_epoch),
                        ephemeris.body(planet),
                        ephemeris.body(final_encounter.body),
                    ) {
                        (Ok(state), Ok(planet_body), Ok(moon_body)) => {
                            (state, planet_body.mu, moon_body.mu, moon_body.radius_m)
                        }
                        _ => {
                            stats.degenerate_cells += 1;
                            return None;
                        }
                    };
                    let arrival = body_of(final_encounter.body, arrival_epoch)?;
                    let moon_rel_pos = arrival.position_inertial - planet_state.position_inertial;
                    let moon_rel_vel = arrival.velocity_inertial - planet_state.velocity_inertial;
                    let v_inf_planet = arcs_arr[legs - 1] - planet_state.velocity_inertial;
                    match planet_arrival_match_mag(
                        v_inf_planet,
                        moon_rel_pos,
                        moon_rel_vel,
                        planet_mu,
                        moon_mu,
                        moon_radius + config.standoff_m,
                    ) {
                        Some(mag) => mag,
                        None => {
                            stats.degenerate_cells += 1;
                            return None;
                        }
                    }
                }
            };
            // Post-match speed is the target body's central-frame speed.
            (arr_mag, rel_vel[legs].length())
        }
        EncounterKind::Flyby => {
            // No arrival burn: with no maneuver the craft continues on
            // the arrival asymptote, so the outgoing central-frame speed
            // (escape proxy) is the transfer arrival speed itself.
            (0.0, arcs_arr[legs - 1].length())
        }
    };
    let total_dv = dep_mag + turns.iter().sum::<f64>() + arrival_mag;
    if total_dv > config.max_broad_dv_mps {
        return None;
    }
    Some(ChainCell {
        departure_epoch,
        leg_tofs_s: leg_tofs_s.to_vec(),
        encounter_bodies: config
            .encounters
            .iter()
            .map(|encounter| encounter.body)
            .collect(),
        departure_burn_mag_mps: dep_mag,
        v_out_frames_mps: seeds,
        turn_mags_mps: turns,
        arrival_mag_mps: arrival_mag,
        final_outgoing_speed_mps: final_speed,
        total_dv,
    })
}

/// Recursive mixed-radix grid walk over per-leg TOF steps. Deterministic:
/// departure-major order like the single-tour grids.
fn walk_legs(
    ctx: &ChainCtx<'_>,
    config: &ChainConfig,
    departure_epoch: SimTime,
    leg: usize,
    tofs: &mut Vec<f64>,
    stats: &mut SearchStats,
    best: &mut Vec<ChainCell>,
) {
    if leg == config.legs() {
        stats.broad_evaluations += 1;
        if let Some(cell) = chain_cell(ctx, departure_epoch, tofs, stats) {
            best.push(cell);
        }
        return;
    }
    for step in 0..config.leg_tof_steps[leg] {
        tofs.push(lerp_range(
            config.leg_tof_min_s[leg],
            config.leg_tof_max_s[leg],
            config.leg_tof_steps[leg],
            step,
        ));
        walk_legs(ctx, config, departure_epoch, leg + 1, tofs, stats, best);
        tofs.pop();
    }
}

fn chain_grid<'a>(
    ephemeris: &'a BakedEphemeris,
    config: &ChainConfig,
) -> Result<(Vec<ChainCell>, SearchStats, ChainCtx<'a>), SearchError> {
    let ctx = chain_context(ephemeris, config)?;
    let mut stats = SearchStats::default();
    let mut best = Vec::new();
    for i in 0..config.departure_steps {
        let epoch = SimTime(if config.departure_steps == 1 {
            config.window_start.0
        } else {
            config.window_start.0
                + config.departure_span_s * i as f64 / (config.departure_steps - 1) as f64
        });
        walk_legs(
            &ctx,
            config,
            epoch,
            0,
            &mut Vec::new(),
            &mut stats,
            &mut best,
        );
    }
    best.sort_by(|a, b| a.total_dv.total_cmp(&b.total_dv));
    best.truncate(config.keep_routes);
    if best.is_empty() {
        return Err(SearchError::NoViableTransfer { stats });
    }
    Ok((best, stats, ctx))
}

/// Broad chain survey: the N-leg Lambert grid without any exact
/// revalidation (no phasing, no correction, no N-body propagation).
/// Millisecond-to-second route scouting for CI replay fixtures (docs/07
/// §7.14 L0-L2): which encounter orders close geometrically and at what
/// patched-conic energy. Deterministic like the other surveys.
pub fn broad_chain_survey(
    ephemeris: &BakedEphemeris,
    config: ChainConfig,
) -> Result<(Vec<BroadChainRoute>, SearchStats), SearchError> {
    let (cells, stats, _) = chain_grid(ephemeris, &config)?;
    let routes = cells
        .into_iter()
        .map(|cell| BroadChainRoute {
            departure_epoch: cell.departure_epoch,
            time_of_flight_s: cell.leg_tofs_s.iter().sum(),
            encounter_bodies: cell.encounter_bodies,
            leg_tofs_s: cell.leg_tofs_s,
            departure_burn_mag_mps: cell.departure_burn_mag_mps,
            flyby_burns_mag_mps: cell.turn_mags_mps,
            arrival_burn_mag_mps: cell.arrival_mag_mps,
            final_outgoing_speed_mps: cell.final_outgoing_speed_mps,
            broad_total_dv_mps: cell.total_dv,
        })
        .collect();
    Ok((routes, stats))
}

/// Chained tour search. Returns validated plans ranked by exact total Δv,
/// best first, each carrying its measured final miss and one
/// [`FlybyEvent`] per intermediate assist. Deterministic: config order,
/// grid order, total_cmp ordering.
pub fn chain_search(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: ChainConfig,
) -> Result<(Vec<RankedPlan>, SearchStats), SearchError> {
    let (best, mut stats, ctx) = chain_grid(ephemeris, &config)?;
    let mut ranked = Vec::new();
    for cell in &best {
        if let Some(plan) = revalidate_chain(ephemeris, field, &ctx, cell, &mut stats)? {
            ranked.push(plan);
        }
    }
    if ranked.is_empty() {
        return Err(SearchError::NoViableTransfer { stats });
    }
    ranked.sort_by(|a, b| {
        a.exact_total_dv_mps
            .total_cmp(&b.exact_total_dv_mps)
            .then(a.exact_miss_m.total_cmp(&b.exact_miss_m))
    });
    Ok((ranked, stats))
}

#[allow(clippy::too_many_lines)]
fn revalidate_chain(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    ctx: &ChainCtx<'_>,
    cell: &ChainCell,
    stats: &mut SearchStats,
) -> Result<Option<RankedPlan>, SearchError> {
    let config = &ctx.config;
    let legs = config.legs();
    // Cumulative encounter epochs.
    let mut epochs = Vec::with_capacity(legs + 1);
    epochs.push(cell.departure_epoch);
    for tof in &cell.leg_tofs_s {
        epochs.push(SimTime(epochs.last().expect("nonempty").0 + tof));
    }
    let depot = ephemeris
        .body_state(config.departure_body, epochs[0])
        .map_err(SearchError::Ephemeris)?;
    let central_dep = ephemeris
        .body_state(config.central_body, epochs[0])
        .map_err(SearchError::Ephemeris)?;
    let depot_mu = ephemeris
        .body(config.departure_body)
        .map_err(SearchError::Ephemeris)?
        .mu;
    let park_radius = ephemeris
        .body(config.departure_body)
        .map_err(SearchError::Ephemeris)?
        .radius_m
        + config.standoff_m;
    let first = &config.encounters[0];
    let first_state = ephemeris
        .body_state(first.body, epochs[1])
        .map_err(SearchError::Ephemeris)?;
    // Departure shortlist, not a single winner: loose screens rank by
    // arrival POSITION only, so distinct anomalies can carry wildly
    // different arrival VELOCITIES (measured ~5 km/s spreads on the V1
    // window) — and the next leg's seed lives or dies by that velocity.
    // The exact stage below arbitrates every start; cheapest total wins.
    let starts = phase_departure_topk(
        field,
        cell.departure_epoch,
        &depot,
        &central_dep,
        &first_state,
        depot_mu,
        park_radius,
        cell.departure_burn_mag_mps,
        cell.leg_tofs_s[0],
        PHASING_BRANCHES,
        stats,
    );
    if starts.is_empty() {
        return Ok(None);
    }
    let mut best: Option<RankedPlan> = None;
    for (point, park_velocity, phased_burn) in starts {
        if let Some(plan) =
            revalidate_chain_from_start(ephemeris, field, ctx, cell, &epochs, point, park_velocity, phased_burn, stats)?
        {
            let better = match &best {
                None => true,
                Some(current) => {
                    plan.exact_total_dv_mps < current.exact_total_dv_mps
                        || (plan.exact_total_dv_mps == current.exact_total_dv_mps
                            && plan.exact_miss_m < current.exact_miss_m)
                }
            };
            if better {
                best = Some(plan);
            }
        }
    }
    Ok(best)
}

/// Departure starts kept per chain cell: enough arrival-asymptote
/// diversity to matter, few enough to keep exact budgets sane (each start
/// costs a full chain correction).
const PHASING_BRANCHES: usize = 3;

#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn revalidate_chain_from_start(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    ctx: &ChainCtx<'_>,
    cell: &ChainCell,
    epochs: &[SimTime],
    point: DVec3,
    park_velocity: DVec3,
    phased_burn: DVec3,
    stats: &mut SearchStats,
) -> Result<Option<RankedPlan>, SearchError> {
    let config = &ctx.config;
    let legs = config.legs();
    // Leg 1: correct the phased departure to its aim point (periapsis
    // for a flyby, standoff sphere for a lone rendezvous) with the
    // departure FROZEN at its phased broad value: correcting the escape
    // burn itself is ill-conditioned (measured JJᵀ cond ~1e18 on the V1
    // leg — normal equations square it), so only the cruise TCM varies.
    // Arrival-velocity diversity across handoffs comes from the top-K
    // starts above, not from freeing the departure.
    let first = &config.encounters[0];
    let first_state = ephemeris
        .body_state(first.body, epochs[1])
        .map_err(SearchError::Ephemeris)?;
    let aim1 = aim_point(
        ephemeris,
        config.central_body,
        first.body,
        epochs[1],
        encounter_standoff(config, first),
    )
    .map_err(SearchError::Ephemeris)?;
    let first_body = ephemeris.body(first.body).map_err(SearchError::Ephemeris)?;
    let mid1_s = midcourse_time_s(cell.leg_tofs_s[0]);
    let (dep_burn, tcm1_burn, mut leg_end, miss1) = match correct_shooting(
        field,
        point,
        park_velocity,
        phased_burn,
        cell.departure_epoch,
        cell.leg_tofs_s[0],
        mid1_s,
        aim1,
        DVec3::ZERO,
        stats,
    ) {
        Some(corrected) => corrected,
        None => {
            stats.failed_revalidations += 1;
            return Ok(None);
        }
    };
    stats.exact_revalidations += 1;
    // Leg-1 gates: must reach the encounter without lithobraking it.
    if !miss1.is_finite() || miss1 > config.max_miss_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    if (leg_end.position - first_state.position_inertial).length() < first_body.radius_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    let mut nodes = vec![
        ManeuverNode::new(cell.departure_epoch, dep_burn)
            .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
    ];
    if tcm1_burn.length() >= 1.0 {
        nodes.push(
            ManeuverNode::new(SimTime(cell.departure_epoch.0 + mid1_s), tcm1_burn)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
        );
    }
    let mut flybys = Vec::new();
    // Intermediate legs: solve each flyby burn to the next aim point,
    // starting from the previous leg's converged handoff. Greedy
    // leg-by-leg correction (not joint): it converges crisply where it
    // converges (Nereid 0.3 km). Its blind spot is velocity: a converged
    // leg-1 handoff with the wrong arrival asymptote forces the next leg
    // hot (measured 35 km/s at Jupiter) or stalls it. Cooling that needs
    // joint multiple shooting (docs/07 §7.14 L4), not a better seed.
    // Encounter burns and events in chain order.
    for leg in 1..legs {
        let encounter = &config.encounters[leg - 1];
        let flyby_epoch = epochs[leg];
        let next_epoch = epochs[leg + 1];
        let flyby_state = ephemeris
            .body_state(encounter.body, flyby_epoch)
            .map_err(SearchError::Ephemeris)?;
        let flyby_body = ephemeris
            .body(encounter.body)
            .map_err(SearchError::Ephemeris)?;
        let periapsis_m = flyby_body.radius_m + encounter_standoff(config, encounter);
        let next = &config.encounters[leg];
        let next_aim = aim_point(
            ephemeris,
            config.central_body,
            next.body,
            next_epoch,
            encounter_standoff(config, next),
        )
        .map_err(SearchError::Ephemeris)?;
        // Burn seed from the broad outgoing asymptote with periapsis
        // energy against the TRUE handoff velocity.
        let v_in_exact = leg_end.velocity - flyby_state.velocity_inertial;
        let seed_dir = cell.v_out_frames_mps[leg - 1];
        let naive_seed = seed_dir - v_in_exact;
        let flyby_seed = match seed_dir.try_normalize().map(|dir| {
            dir * (seed_dir.length_squared() + 2.0 * flyby_body.mu / periapsis_m).sqrt()
                - v_in_exact
        }) {
            Some(seed) if seed.is_finite() => seed,
            _ => naive_seed,
        };
        if !flyby_seed.is_finite() {
            stats.filtered_by_miss += 1;
            return Ok(None);
        }
        let (_, flyby_burn, end, miss) = match correct_shooting(
            field,
            leg_end.position,
            leg_end.velocity,
            DVec3::ZERO,
            flyby_epoch,
            cell.leg_tofs_s[leg],
            0.0,
            next_aim,
            flyby_seed,
            stats,
        ) {
            Some(corrected) => corrected,
            None => {
                stats.failed_revalidations += 1;
                return Ok(None);
            }
        };
        // Lithobraking is not a flyby: the end state must stay outside
        // the encounter body whatever its kind.
        let next_state = ephemeris
            .body_state(next.body, next_epoch)
            .map_err(SearchError::Ephemeris)?;
        let target_radius = ephemeris
            .body(next.body)
            .map_err(SearchError::Ephemeris)?
            .radius_m;
        let dist_center = (end.position - next_state.position_inertial).length();
        if !miss.is_finite() || miss > config.max_miss_m || dist_center < target_radius {
            stats.filtered_by_miss += 1;
            return Ok(None);
        }
        if flyby_burn.length() >= 1.0 {
            nodes.push(
                ManeuverNode::new(flyby_epoch, flyby_burn)
                    .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
            );
        }
        flybys.push(FlybyEvent {
            body: encounter.body,
            epoch: flyby_epoch,
            periapsis_m,
            burn_mps: flyby_burn.length(),
        });
        // Final rendezvous: null the arrival velocity at the aim sphere.
        if leg + 1 == legs && next.kind == EncounterKind::Rendezvous {
            let arrival = ephemeris
                .body_state(next.body, next_epoch)
                .map_err(SearchError::Ephemeris)?;
            let arrival_burn = arrival.velocity_inertial - end.velocity;
            if !arrival_burn.is_finite() {
                stats.filtered_by_miss += 1;
                return Ok(None);
            }
            nodes.push(
                ManeuverNode::new(next_epoch, arrival_burn)
                    .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
            );
        }
        // Final encounter bookkeeping.
        if leg + 1 == legs {
            if next.kind == EncounterKind::Flyby {
                // Unpowered arrival: record the encounter (zero burn) so
                // the plan carries the full encounter order for replays.
                let final_body = ephemeris.body(next.body).map_err(SearchError::Ephemeris)?;
                flybys.push(FlybyEvent {
                    body: next.body,
                    epoch: next_epoch,
                    periapsis_m: final_body.radius_m + encounter_standoff(config, next),
                    burn_mps: 0.0,
                });
            }
            let mut plan =
                ManeuverPlan::new(nodes, point, park_velocity + dep_burn, cell.departure_epoch)
                    .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?;
            plan.predicted_miss_m = Some(miss);
            plan = plan.with_flybys(flybys);
            let total = cell.leg_tofs_s.iter().sum();
            return Ok(Some(RankedPlan {
                exact_total_dv_mps: plan.total_dv_mps(),
                departure_epoch: cell.departure_epoch,
                time_of_flight_s: total,
                broad_total_dv_mps: cell.total_dv,
                plan,
                exact_miss_m: miss,
            }));
        }
        // Advance down the chain: the next leg starts where this one ended.
        leg_end = end;
    }
    // Single-encounter chain: leg 1 already reached the only encounter.
    if legs == 1 {
        let encounter = &config.encounters[0];
        let only_body = ephemeris
            .body(encounter.body)
            .map_err(SearchError::Ephemeris)?;
        match encounter.kind {
            EncounterKind::Flyby => {
                flybys.push(FlybyEvent {
                    body: encounter.body,
                    epoch: epochs[1],
                    periapsis_m: only_body.radius_m + encounter_standoff(config, encounter),
                    burn_mps: 0.0,
                });
            }
            EncounterKind::Rendezvous => {
                let arrival = ephemeris
                    .body_state(encounter.body, epochs[1])
                    .map_err(SearchError::Ephemeris)?;
                let arrival_burn = arrival.velocity_inertial - leg_end.velocity;
                if !arrival_burn.is_finite() {
                    stats.filtered_by_miss += 1;
                    return Ok(None);
                }
                nodes.push(
                    ManeuverNode::new(epochs[1], arrival_burn)
                        .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
                );
            }
        }
        let mut plan =
            ManeuverPlan::new(nodes, point, park_velocity + dep_burn, cell.departure_epoch)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?;
        plan.predicted_miss_m = Some(miss1);
        plan = plan.with_flybys(flybys);
        return Ok(Some(RankedPlan {
            exact_total_dv_mps: plan.total_dv_mps(),
            departure_epoch: cell.departure_epoch,
            time_of_flight_s: cell.leg_tofs_s[0],
            broad_total_dv_mps: cell.total_dv,
            plan,
            exact_miss_m: miss1,
        }));
    }
    Ok(None)
}

/// Aim point for an encounter at its epoch: body center plus the
/// central-radial offset at standoff altitude. Never inside the
/// point-mass singularity; the miss is measured against a physical
/// rendezvous/flyby sphere.
///
/// NOTE (measured 2026-09-16): a B-plane-side variant (periapsis from the
/// broad incoming/outgoing asymptotes) was tried here and reverted. It
/// sharpened leg-1 hits (20 m -> sub-meter) but destabilized downstream
/// legs (Nereid final 0.3 km -> 986 km; V1 leg-2 convergence coin flip).
/// Position-only aims converge crisply; making multi-leg exact COOL
/// (not just converged) needs joint multiple shooting, not aim-point
/// side-picking. See docs/07 §7.14 L4.
fn aim_point(
    ephemeris: &BakedEphemeris,
    central: BodyId,
    body: BodyId,
    epoch: SimTime,
    standoff_m: f64,
) -> Result<DVec3, thessa_sim_core::EphemerisError> {
    let central_state = ephemeris.body_state(central, epoch)?;
    let body_state = ephemeris.body_state(body, epoch)?;
    let target = ephemeris.body(body)?;
    let aim_dir = (body_state.position_inertial - central_state.position_inertial).normalize();
    if !aim_dir.is_finite() {
        return Err(thessa_sim_core::EphemerisError::InvalidBody(
            "encounter aim direction degenerates".into(),
        ));
    }
    Ok(body_state.position_inertial + aim_dir * (target.radius_m + standoff_m))
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{BakedBody, KeplerOrbit};

    /// Center + depot + two swing bodies + target on nested orbits.
    /// Bodies carry real radii: pinpoint aims above point masses are
    /// singular terminal problems (100 km over mu-5e11 filtered at 2e6
    /// even jointly); physical radii keep the test about chaining.
    fn chain_system() -> BakedEphemeris {
        let orbit = |a: f64, m0: f64| {
            KeplerOrbit::new(1.0e14, a, 0.0, 0.0, 0.0, 0.0, m0).expect("valid test orbit")
        };
        BakedEphemeris::new(
            "TEST_CHAIN",
            vec![
                BakedBody::fixed(BodyId(0), "center", 1.0e14, 0.0),
                BakedBody::orbital(
                    BodyId(1),
                    "depot",
                    1.0e12,
                    1.0e6,
                    BodyId(0),
                    orbit(1.0e7, 0.0),
                ),
                BakedBody::orbital(
                    BodyId(2),
                    "swing1",
                    5.0e11,
                    1.0e6,
                    BodyId(0),
                    orbit(1.25e7, 1.0),
                ),
                BakedBody::orbital(
                    BodyId(3),
                    "swing2",
                    5.0e11,
                    1.0e6,
                    BodyId(0),
                    orbit(1.4e7, 2.2),
                ),
                BakedBody::orbital(
                    BodyId(4),
                    "target",
                    1.0e12,
                    1.0e6,
                    BodyId(0),
                    orbit(1.6e7, 3.0),
                ),
            ],
        )
        .expect("valid test system")
    }

    fn chain_config() -> ChainConfig {
        ChainConfig {
            central_body: BodyId(0),
            departure_body: BodyId(1),
            encounters: vec![
                ChainEncounter {
                    body: BodyId(2),
                    kind: EncounterKind::Flyby,
                    standoff_m: None,
                },
                ChainEncounter {
                    body: BodyId(3),
                    kind: EncounterKind::Flyby,
                    standoff_m: None,
                },
                ChainEncounter {
                    body: BodyId(4),
                    kind: EncounterKind::Rendezvous,
                    standoff_m: None,
                },
            ],
            window_start: SimTime(0.0),
            departure_span_s: 40_000.0,
            departure_steps: 6,
            leg_tof_min_s: vec![4_000.0, 4_000.0, 4_000.0],
            leg_tof_max_s: vec![10_000.0, 10_000.0, 10_000.0],
            leg_tof_steps: vec![3, 3, 3],
            keep_routes: 2,
            standoff_m: 100_000.0,
            max_broad_dv_mps: 30_000.0,
            max_miss_m: 1.0e6,
        }
    }

    #[test]
    fn broad_chain_survey_ranks_two_assist_routes() {
        let ephemeris = chain_system();
        let (routes, stats) = broad_chain_survey(&ephemeris, chain_config()).expect("survey finds");
        assert!(!routes.is_empty());
        assert!(stats.broad_evaluations > 0);
        let winner = &routes[0];
        assert_eq!(
            winner.encounter_bodies,
            vec![BodyId(2), BodyId(3), BodyId(4)]
        );
        assert_eq!(winner.leg_tofs_s.len(), 3);
        assert_eq!(winner.flyby_burns_mag_mps.len(), 2);
        assert!(winner.broad_total_dv_mps.is_finite() && winner.broad_total_dv_mps > 0.0);
        // Deterministic: same inputs, same routes.
        let (again, _) = broad_chain_survey(&ephemeris, chain_config()).expect("survey finds");
        assert_eq!(routes, again);
    }

    #[test]
    fn chain_search_validates_two_assist_plan() {
        let ephemeris = chain_system();
        let field = GravityField::from_ephemeris(&ephemeris);
        let (ranked, stats) =
            chain_search(&ephemeris, &field, chain_config()).expect("search finds");
        assert!(!ranked.is_empty());
        let winner = &ranked[0];
        assert!(winner.exact_miss_m <= 1.0e6);
        assert_eq!(winner.plan.flybys.len(), 2);
        assert_eq!(winner.plan.flybys[0].body, BodyId(2));
        assert_eq!(winner.plan.flybys[1].body, BodyId(3));
        // Leg continuity tripwire: nodes fire in strict time order, and
        // no departure/correction burn explodes an order of magnitude
        // past the whole broad budget — a stale leg-handoff state
        // converges hot or not at all. (Bound is 5x, not 1x: the broad
        // turn price is a periapsis-energy lower bound and exact deep-well
        // bends legitimately exceed it. The final rendezvous match is
        // exempt entirely: nulling velocity at the standoff sphere prices
        // the well fall-in while broad arrival prices v_inf.)
        let mut prev_epoch = f64::NEG_INFINITY;
        let last = winner.plan.nodes.len() - 1;
        for (index, node) in winner.plan.nodes.iter().enumerate() {
            assert!(node.epoch.0 > prev_epoch, "nodes strictly increase in time");
            prev_epoch = node.epoch.0;
            if index == last {
                continue;
            }
            assert!(
                node.magnitude_mps() <= 5.0 * winner.broad_total_dv_mps,
                "no correction burn explodes past the broad budget"
            );
        }
        assert!(winner.exact_total_dv_mps.is_finite() && winner.exact_total_dv_mps > 0.0);
        assert!(winner.plan.predicted_miss_m.is_some());
        assert!(stats.exact_revalidations >= 1);
        let (ranked2, _) = chain_search(&ephemeris, &field, chain_config()).expect("search finds");
        assert_eq!(ranked, ranked2);
    }

    #[test]
    fn chain_search_handles_single_rendezvous() {
        // One-encounter rendezvous chain: degenerate chain that must
        // behave like a direct transfer with an arrival match.
        let ephemeris = chain_system();
        let field = GravityField::from_ephemeris(&ephemeris);
        let mut config = chain_config();
        config.encounters = vec![ChainEncounter {
            body: BodyId(4),
            kind: EncounterKind::Rendezvous,
            standoff_m: None,
        }];
        config.leg_tof_min_s = vec![8_000.0];
        config.leg_tof_max_s = vec![12_000.0];
        config.leg_tof_steps = vec![3];
        config.keep_routes = 1;
        let (ranked, _) = chain_search(&ephemeris, &field, config).expect("search finds");
        let winner = &ranked[0];
        assert!(winner.exact_miss_m <= 1.0e6);
        assert!(winner.plan.flybys.is_empty());
        assert!(winner.plan.nodes.len() >= 2);
    }

    #[test]
    fn chain_search_reports_escape_outgoing_speed() {
        let ephemeris = chain_system();
        let mut config = chain_config();
        config.encounters = vec![
            ChainEncounter {
                body: BodyId(2),
                kind: EncounterKind::Flyby,
                standoff_m: None,
            },
            ChainEncounter {
                body: BodyId(3),
                kind: EncounterKind::Flyby,
                standoff_m: None,
            },
        ];
        config.leg_tof_min_s = vec![4_000.0, 4_000.0];
        config.leg_tof_max_s = vec![10_000.0, 10_000.0];
        config.leg_tof_steps = vec![2, 2];
        let (routes, _) = broad_chain_survey(&ephemeris, config.clone()).expect("survey finds");
        assert_eq!(routes[0].arrival_burn_mag_mps, 0.0);
        assert!(routes[0].final_outgoing_speed_mps > 0.0);
        let field = GravityField::from_ephemeris(&ephemeris);
        let (ranked, _) = chain_search(&ephemeris, &field, config).expect("search finds");
        assert_eq!(ranked[0].plan.flybys.len(), 2);
    }

    #[test]
    fn chain_rejects_mid_chain_rendezvous_and_bad_shapes() {
        let ephemeris = chain_system();
        let field = GravityField::from_ephemeris(&ephemeris);
        // Mid-chain rendezvous is two transfers, not a chain.
        let mut bad = chain_config();
        bad.encounters[0].kind = EncounterKind::Rendezvous;
        assert_eq!(
            broad_chain_survey(&ephemeris, bad.clone()),
            Err(SearchError::InvalidConfig)
        );
        assert_eq!(
            chain_search(&ephemeris, &field, bad),
            Err(SearchError::InvalidConfig)
        );
        // Empty encounters and mismatched leg vectors.
        let mut empty = chain_config();
        empty.encounters.clear();
        assert_eq!(
            broad_chain_survey(&ephemeris, empty),
            Err(SearchError::InvalidConfig)
        );
        let mut ragged = chain_config();
        ragged.leg_tof_min_s.pop();
        assert_eq!(
            broad_chain_survey(&ephemeris, ragged),
            Err(SearchError::InvalidConfig)
        );
    }
}
