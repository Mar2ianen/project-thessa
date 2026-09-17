//! Staged porkchop search: broad Lambert grid, local refinement, phasing,
//! midcourse correction, exact ranking (docs/23 §12, 24 §7 pattern).
//!
//! Phase 1 (broad, loose): two-body Lambert arcs on osculating endpoint
//! states with patched-conic departure energies (parking-orbit escape
//! around the depot moon — the depot well is NOT ignored), prograde branch
//! per cell, perigee impact screen, Δv cap. All endpoint states share one
//! epoch convention: arrival-minus-central-AT-arrival (mixing epochs
//! injects billions of metres of fictitious displacement).
//! Phase 2 (narrow): local grid refinement around the best cell, then
//! parking-orbit anomaly phasing against loose full-N-body screens.
//! Phase 3 (final): midcourse differential correction on full N-body
//! dynamics — the departure burn stays at its phased value (correcting it
//! stalls in the escape turn), a post-escape TCM absorbs everything
//! downstream. The converged trajectory IS the revalidation. Plans carry
//! departure + TCM (+ arrival match) nodes with measured miss.
//! Arrival-match semantics (rendezvous/landing price, NOT orbit-joining):
//! the match nulls the full N-body arrival velocity at the standoff aim
//! point, so it includes the fall into the target well — its floor is the
//! local escape velocity (v_esc-scale for massive moons: Pelagos->Thessa
//! prices ~5.5 km/s against vesc_aim ~5.5 km/s, while the SOI-edge v_inf is
//! Hohmann-like ~1.5 km/s). Capture-into-orbit / flyby-tour arrival modes
//! are follow-ups, not this node's job.
//! Pruning approximations never define physical truth: only corrected
//! plans with measured miss execute.

use glam::{DMat3, DVec3};
use rayon::prelude::*;
use thessa_sim_core::{
    AdaptiveIntegratorConfig, BakedEphemeris, BodyId, BodyState, GravityField, ImpulsiveBurn,
    SimTime, TestParticleState, propagate_adaptive_with_burns,
};

use crate::{
    ManeuverNode, ManeuverPlan,
    lambert::solve_lambert_prograde,
    patch::{planet_arrival_match_mag, planet_escape_moon_vinf, planet_of},
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SearchConfig {
    /// Body hosting the two-body Lambert model (positions/velocities are
    /// taken relative to it). Patched-conic-style planning choice, stated
    /// explicitly: the N-body truth is checked in phase 3.
    pub central_body: BodyId,
    /// Depot body: the craft departs with its full inertial state.
    pub departure_body: BodyId,
    /// Target body for arrival matching.
    pub arrival_body: BodyId,
    pub window_start: SimTime,
    pub departure_span_s: f64,
    pub departure_steps: usize,
    pub tof_min_s: f64,
    pub tof_max_s: f64,
    pub tof_steps: usize,
    /// Survivors carried into exact revalidation.
    pub keep_candidates: usize,
    /// Parking-orbit / rendezvous-approach altitude (m) at both ends:
    /// departure starts there (never at the depot center — point-mass
    /// singularity), arrival is measured there. Typical 50–500 km.
    pub standoff_m: f64,
    /// Broad cells above this total Δv are pruned, not ranked: a 50 km/s
    /// "optimum" through the planet is grid noise, not a transfer.
    pub max_broad_dv_mps: f64,
    /// Plans missing by more than this are dropped, not ranked.
    pub max_miss_m: f64,
}

impl SearchConfig {
    fn validate(&self) -> Result<(), SearchError> {
        if !self.window_start.0.is_finite()
            || !self.departure_span_s.is_finite()
            || self.departure_span_s < 0.0
            || self.departure_steps == 0
            || !self.tof_min_s.is_finite()
            || !self.tof_max_s.is_finite()
            || self.tof_min_s <= 0.0
            || self.tof_max_s < self.tof_min_s
            || self.tof_steps == 0
            || self.keep_candidates == 0
            || !self.standoff_m.is_finite()
            || self.standoff_m <= 0.0
            || !self.max_broad_dv_mps.is_finite()
            || self.max_broad_dv_mps <= 0.0
            || !self.max_miss_m.is_finite()
            || self.max_miss_m < 0.0
        {
            return Err(SearchError::InvalidConfig);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RankedPlan {
    pub plan: ManeuverPlan,
    pub departure_epoch: SimTime,
    pub time_of_flight_s: f64,
    /// Two-body Lambert estimate (broad/narrow phase).
    pub broad_total_dv_mps: f64,
    /// After exact revalidation: realized departure + arrival-match burns.
    pub exact_total_dv_mps: f64,
    /// Arrival miss from exact N-body propagation (m).
    pub exact_miss_m: f64,
}

/// One broad-phase route: pure two-body scouting with NO exact revalidation
/// and NO arrival-miss measurement. This is route selection (which windows
/// and geometries close, at what energy), not a flyable plan: convert to a
/// plan only through exact correction (`porkchop_search` / `flyby_search`),
/// which sets `predicted_miss_m`. The type boundary is the execution gate —
/// a `BroadRoute` cannot be fed to the executor.
#[derive(Debug, Clone, PartialEq)]
pub struct BroadRoute {
    pub departure_epoch: SimTime,
    pub time_of_flight_s: f64,
    pub broad_total_dv_mps: f64,
    pub departure_burn_mag_mps: f64,
    pub arrival_burn_mag_mps: f64,
}

/// Broad survey: phases 1–2 of the porkchop (grid + refinement) without
/// phase 3 (no phasing, no correction, no N-body propagation at all).
/// Works at ANY scale — including inter-body transfers (moons of different
/// planets around the star) whose year-long arcs make full correction
/// cost-prohibitive: the survey prices the route energy honestly while
/// marking it unvalidated. Deterministic like the full search.
pub fn broad_survey(
    ephemeris: &BakedEphemeris,
    config: SearchConfig,
) -> Result<(Vec<BroadRoute>, SearchStats), SearchError> {
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
    let ctx = BroadCtx {
        ephemeris,
        config,
        central_mu: central.mu,
        central_radius_m: central.radius_m,
        depot_mu: depot.mu,
        depot_radius_m: depot.radius_m,
        departure_planet: planet_of(ephemeris, config.central_body, config.departure_body),
        arrival_planet: planet_of(ephemeris, config.central_body, config.arrival_body),
    };
    let mut stats = SearchStats::default();
    let mut best = Vec::new();
    grid_best(
        &ctx,
        GridWindow {
            start_s: config.window_start.0,
            span_s: config.departure_span_s,
            steps: config.departure_steps,
            tof_min: config.tof_min_s,
            tof_max: config.tof_max_s,
            tof_steps: config.tof_steps,
        },
        &mut stats,
        &mut best,
    );
    if best.is_empty() {
        return Err(SearchError::NoViableTransfer { stats });
    }
    let routes = best
        .into_iter()
        .map(|cell| BroadRoute {
            departure_epoch: cell.departure_epoch,
            time_of_flight_s: cell.time_of_flight_s,
            broad_total_dv_mps: cell.total_dv,
            departure_burn_mag_mps: cell.departure_burn_mag_mps,
            arrival_burn_mag_mps: cell.total_dv - cell.departure_burn_mag_mps,
        })
        .collect();
    Ok((routes, stats))
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SearchStats {
    pub broad_evaluations: usize,
    pub degenerate_cells: usize,
    pub impact_cells: usize,
    pub phase_screens: usize,
    pub newton_propagations: usize,
    pub exact_revalidations: usize,
    pub failed_revalidations: usize,
    pub filtered_by_miss: usize,
    /// Differential corrections stopped by the hot-stall guard (burn past
    /// the hot floor with no miss improvement) rather than by convergence
    /// or by an integrator failure. A hot stall is "this start cannot
    /// close cheaply", not "the propagator broke" — kept separate so
    /// failure triage does not conflate the two.
    pub hot_stall_exits: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SearchError {
    InvalidConfig,
    Ephemeris(thessa_sim_core::EphemerisError),
    /// No transfer survived, with the counters showing why (all cells
    /// degenerate vs all survivors filtered by miss).
    NoViableTransfer {
        stats: SearchStats,
    },
}

impl std::fmt::Display for SearchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig => write!(formatter, "invalid search config"),
            Self::Ephemeris(error) => write!(formatter, "ephemeris error: {error}"),
            Self::NoViableTransfer { stats } => write!(
                formatter,
                "no viable transfer (broad {}, degenerate {}, impact {}, screens {}, newton props {}, revalidated {}/{}, miss-filtered {})",
                stats.broad_evaluations,
                stats.degenerate_cells,
                stats.impact_cells,
                stats.phase_screens,
                stats.newton_propagations,
                stats.exact_revalidations,
                stats.failed_revalidations,
                stats.filtered_by_miss,
            ),
        }
    }
}

impl std::error::Error for SearchError {}

#[derive(Clone)]
struct Cell {
    departure_epoch: SimTime,
    time_of_flight_s: f64,
    /// Patched escape magnitude (energy); direction comes from phasing.
    departure_burn_mag_mps: f64,
    total_dv: f64,
}

/// Shared broad-phase inputs: keeps grid signatures from bleeding params.
struct BroadCtx<'a> {
    ephemeris: &'a BakedEphemeris,
    config: SearchConfig,
    central_mu: f64,
    central_radius_m: f64,
    depot_mu: f64,
    depot_radius_m: f64,
    /// Intermediate planet wells (None for direct-central moons — the
    /// bit-identical lunar path). Precomputed: hierarchy is static.
    departure_planet: Option<BodyId>,
    arrival_planet: Option<BodyId>,
}

fn lambert_cell(
    ctx: &BroadCtx<'_>,
    departure_epoch: SimTime,
    time_of_flight_s: f64,
    stats: &mut SearchStats,
) -> Option<Cell> {
    let ephemeris = ctx.ephemeris;
    let config = &ctx.config;
    let central_mu = ctx.central_mu;
    let central_radius_m = ctx.central_radius_m;
    let arrival_epoch = SimTime(departure_epoch.0 + time_of_flight_s);
    // Central states at BOTH epochs: r2 must be arrival-minus-central-at-
    // arrival, not minus-central-at-departure (the central body itself
    // moves ~37 km/s around the star; mixing epochs injects billions of
    // metres of fictitious displacement and prices every cell at tens of
    // km/s — caught by the shoot-the-arc audit reading 0.0 km miss on
    // garbage geometry).
    let central_dep = ephemeris
        .body_state(config.central_body, departure_epoch)
        .ok()?;
    let central_arr = ephemeris
        .body_state(config.central_body, arrival_epoch)
        .ok()?;
    let departure = ephemeris
        .body_state(config.departure_body, departure_epoch)
        .ok()?;
    let arrival = ephemeris
        .body_state(config.arrival_body, arrival_epoch)
        .ok()?;
    let r2 = arrival.position_inertial - central_arr.position_inertial;
    let v2 = arrival.velocity_inertial - central_arr.velocity_inertial;
    // Depot == central (Apollo-class departure from the central body's own
    // parking orbit): the Lambert arc starts at the parking point ANTI-
    // facing the target — the Hohmann half-ellipse geometry (perigee burn
    // opposite the arrival, ~180° transfer; the target's own motion during
    // flight keeps the arc well-conditioned). Facing-side starts force
    // expensive loop/fast-chord arcs or degenerate Lambert pairs. The burn
    // is the direct vector change (no patch — depot IS the well); both
    // tangential senses price it while exact anomaly phasing (full 360°
    // scan) refines the true asymptote.
    let depot_is_central = config.departure_body == config.central_body;
    let park_radius = ctx.depot_radius_m + ctx.config.standoff_m;
    // Depot == central needs a parking-orbit start direction BEFORE the
    // Lambert solve (r1 lives on the parking circle: depot == central
    // would give r1 = 0). Candidates: anti-facing DEPARTURE (robust,
    // always solves, but 40-65 deg off Hohmann — measured Luna 4725 vs
    // Apollo 3033) plus anti-facing ARRIVAL nudged +-12 deg in-plane
    // (near-Hohmann 168 deg, well-conditioned; exact 180 deg is a
    // singularity). Price the cheapest solved side. Cost: up to 3 Lambert
    // solves per broad cell on this lunar-class path only.
    let depot_central_candidates: Option<Vec<(DVec3, DVec3)>> = if depot_is_central {
        let target_dep = ephemeris
            .body_state(config.arrival_body, departure_epoch)
            .ok()?;
        let facing_dep_raw = target_dep.position_inertial - central_dep.position_inertial;
        let facing_arr_raw = arrival.position_inertial - central_arr.position_inertial;
        if facing_dep_raw.length_squared() <= 0.0 || facing_arr_raw.length_squared() <= 0.0 {
            stats.degenerate_cells += 1;
            return None;
        }
        let facing_dep = facing_dep_raw.normalize();
        let facing_arr = facing_arr_raw.normalize();
        let plane_raw = r2.cross(v2);
        if !facing_dep.is_finite() || !facing_arr.is_finite() || plane_raw.length_squared() <= 0.0 {
            stats.degenerate_cells += 1;
            return None;
        }
        let plane_normal = plane_raw.normalize();
        let v_circ = (ctx.depot_mu / park_radius).sqrt();
        if !v_circ.is_finite() {
            stats.degenerate_cells += 1;
            return None;
        }
        let mut cands = Vec::with_capacity(3);
        // 1) departure-facing (robust fallback).
        {
            let start_dir = -facing_dep;
            let prograde = plane_normal.cross(start_dir).normalize();
            if prograde.is_finite() {
                cands.push((start_dir * park_radius, prograde * v_circ));
            }
        }
        // 2-3) arrival-nudged +-12 deg (near-Hohmann).
        for sign in [1.0, -1.0] {
            let delta = sign * 12.0_f64.to_radians();
            let anti = -facing_arr;
            let in_plane = plane_normal.cross(anti);
            if !in_plane.is_finite() || in_plane.length_squared() <= 0.0 {
                continue;
            }
            let start_dir = (anti * delta.cos() + in_plane.normalize() * delta.sin()).normalize();
            if !start_dir.is_finite() {
                continue;
            }
            let prograde = plane_normal.cross(start_dir).normalize();
            if !prograde.is_finite() {
                continue;
            }
            cands.push((start_dir * park_radius, prograde * v_circ));
        }
        if cands.is_empty() {
            stats.degenerate_cells += 1;
            return None;
        }
        Some(cands)
    } else {
        None
    };
    let (r1, v1) = if let Some(ref cands) = depot_central_candidates {
        // Placeholder; the real solve tries each candidate below.
        cands[0]
    } else {
        (
            departure.position_inertial - central_dep.position_inertial,
            departure.velocity_inertial - central_dep.velocity_inertial,
        )
    };
    // Prograde branch per cell: the cheap side is short-way on one side
    // of the sky and long-way on the other; a fixed flag would price half
    // the grid as retrograde.
    // Depot == central: solve each start, keep the cheaper TOTAL
    // (dep + arrival proxy). Selecting on dep alone prefers hot
    // departures with cool arrivals over balanced Hohmann-like arcs.
    let (arc, r1, v1) = if let Some(cands) = depot_central_candidates {
        let mut best: Option<(crate::lambert::LambertArc, DVec3, DVec3, f64)> = None;
        for (try_r1, try_v1) in cands {
            let Ok(try_arc) =
                solve_lambert_prograde(try_r1, try_v1, r2, time_of_flight_s, central_mu)
            else {
                continue;
            };
            let pro = (try_arc.departure_velocity_mps - try_v1).length();
            let retro = (try_arc.departure_velocity_mps + try_v1).length();
            if !pro.is_finite() || !retro.is_finite() {
                continue;
            }
            let dep = pro.min(retro);
            // Arrival proxy: direct match (exact for Luna-class where
            // arrival_planet is None; patched arrivals refine below).
            let arr_proxy = (v2 - try_arc.arrival_velocity_mps).length();
            if !arr_proxy.is_finite() {
                continue;
            }
            let total = dep + arr_proxy;
            if best.is_none_or(|(_, _, _, best_total)| total < best_total) {
                best = Some((try_arc, try_r1, try_v1, total));
            }
        }
        match best {
            Some((arc, r1, v1, _)) => (arc, r1, v1),
            None => {
                stats.degenerate_cells += 1;
                return None;
            }
        }
    } else {
        match solve_lambert_prograde(r1, v1, r2, time_of_flight_s, central_mu) {
            Ok(arc) => (arc, r1, v1),
            Err(_) => {
                stats.degenerate_cells += 1;
                return None;
            }
        }
    };
    // Patched escape magnitude (energy fixed here); the burn direction and
    // parking-orbit anomaly come from phasing against screened flights.
    // Depot momentum sanity stays: a radial depot trajectory has no
    // parking-orbit plane to phase in.
    let depot_momentum = r1.cross(v1);
    if depot_momentum.length_squared() <= 0.0 {
        stats.degenerate_cells += 1;
        return None;
    }
    // Departure pricing: depot == central prices the direct vector change
    // from the parking orbit (both senses, cheaper seeds); an endpoint
    // moon behind a planet well flies the planet patch; a direct-central
    // moon keeps moon-only pricing (bit-identical lunar path).
    let dep_mag = if depot_is_central {
        // v1 is the prograde parking velocity; retrograde is its negation.
        let prograde = (arc.departure_velocity_mps - v1).length();
        let retrograde = (arc.departure_velocity_mps + v1).length();
        if !prograde.is_finite() || !retrograde.is_finite() {
            stats.degenerate_cells += 1;
            return None;
        }
        prograde.min(retrograde)
    } else {
        match ctx.departure_planet {
            None => {
                let v_inf = (arc.departure_velocity_mps - v1).length();
                match patched_escape_mag(ctx.depot_mu, park_radius, v_inf) {
                    Some(mag) => mag,
                    None => {
                        stats.degenerate_cells += 1;
                        return None;
                    }
                }
            }
            Some(planet) => {
                let planet_state = match ephemeris.body_state(planet, departure_epoch) {
                    Ok(state) => state,
                    Err(_) => {
                        stats.degenerate_cells += 1;
                        return None;
                    }
                };
                let planet_mu = match ephemeris.body(planet) {
                    Ok(body) => body.mu,
                    Err(_) => {
                        stats.degenerate_cells += 1;
                        return None;
                    }
                };
                let moon_rel_pos = departure.position_inertial - planet_state.position_inertial;
                let moon_rel_vel = departure.velocity_inertial - planet_state.velocity_inertial;
                let v_inf_planet = arc.departure_velocity_mps - planet_state.velocity_inertial;
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
        }
    };
    // Arrival pricing: symmetric patch (planet-well AND moon-well
    // inclusive rendezvous price) or the direct match.
    let arr_mag = match ctx.arrival_planet {
        None => {
            let arr_burn = v2 - arc.arrival_velocity_mps;
            if !arr_burn.is_finite() {
                stats.degenerate_cells += 1;
                return None;
            }
            arr_burn.length()
        }
        Some(planet) => {
            let planet_state = match ephemeris.body_state(planet, arrival_epoch) {
                Ok(state) => state,
                Err(_) => {
                    stats.degenerate_cells += 1;
                    return None;
                }
            };
            let (planet_mu, moon_mu, moon_radius) =
                match (ephemeris.body(planet), ephemeris.body(config.arrival_body)) {
                    (Ok(planet_body), Ok(moon_body)) => {
                        (planet_body.mu, moon_body.mu, moon_body.radius_m)
                    }
                    _ => {
                        stats.degenerate_cells += 1;
                        return None;
                    }
                };
            let moon_rel_pos = arrival.position_inertial - planet_state.position_inertial;
            let moon_rel_vel = arrival.velocity_inertial - planet_state.velocity_inertial;
            let v_inf_planet = arc.arrival_velocity_mps - planet_state.velocity_inertial;
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
    let total_dv = dep_mag + arr_mag;
    if total_dv > config.max_broad_dv_mps {
        return None;
    }
    // Perigee impact screen on the departure arc: arcs through the central
    // body are grid noise (the exact propagator would just hit singularity).
    // Skipped for depot == central: the arc starts at the parking orbit by
    // construction, so the screen would eat every valid cell.
    if !depot_is_central
        && transfer_perigee_m(r1, arc.departure_velocity_mps, central_mu) < central_radius_m * 1.05
    {
        stats.impact_cells += 1;
        return None;
    }
    Some(Cell {
        departure_epoch,
        time_of_flight_s,
        departure_burn_mag_mps: dep_mag,
        total_dv,
    })
}

/// Patched-conic escape magnitude from a circular parking orbit: the energy
/// needed to leave with `v_inf_mps` at infinity. Shared by the direct and
/// flyby broad phases (same depot physics, one formula).
pub(crate) fn patched_escape_mag(depot_mu: f64, park_radius_m: f64, v_inf_mps: f64) -> Option<f64> {
    let v_circ = (depot_mu / park_radius_m).sqrt();
    let v_esc_sq = 2.0 * depot_mu / park_radius_m;
    if !v_circ.is_finite() || !v_inf_mps.is_finite() {
        return None;
    }
    let mag = (v_inf_mps * v_inf_mps + v_esc_sq).sqrt() - v_circ;
    if !mag.is_finite() || mag < 0.0 {
        return None;
    }
    Some(mag)
}

/// Transfer-ellipse perigee from one state vector (also correct for
/// hyperbolic energy via the same `a(1-e)` form).
pub(crate) fn transfer_perigee_m(position: DVec3, velocity: DVec3, mu: f64) -> f64 {
    let radius = position.length();
    if radius <= 0.0 {
        return 0.0;
    }
    let energy = velocity.length_squared() / 2.0 - mu / radius;
    if energy >= 0.0 {
        // Unbound: still report the pericenter radius honestly.
        let momentum = position.cross(velocity);
        let semi_latus = momentum.length_squared() / mu;
        let ecc_vector = (position * (velocity.length_squared() - mu / radius)
            - velocity * position.dot(velocity))
            / mu;
        let eccentricity = ecc_vector.length();
        return semi_latus / (1.0 + eccentricity);
    }
    let semi_major = -mu / (2.0 * energy);
    let ecc_vector = (position * (velocity.length_squared() - mu / radius)
        - velocity * position.dot(velocity))
        / mu;
    semi_major * (1.0 - ecc_vector.length())
}

/// One Lambert grid patch: departure window plus time-of-flight range.
#[derive(Debug, Clone, Copy)]
struct GridWindow {
    start_s: f64,
    span_s: f64,
    steps: usize,
    tof_min: f64,
    tof_max: f64,
    tof_steps: usize,
}

fn grid_best(
    ctx: &BroadCtx<'_>,
    window: GridWindow,
    stats: &mut SearchStats,
    best: &mut Vec<Cell>,
) {
    for i in 0..window.steps {
        let epoch = SimTime(if window.steps == 1 {
            window.start_s
        } else {
            window.start_s + window.span_s * i as f64 / (window.steps - 1) as f64
        });
        for j in 0..window.tof_steps {
            let tof = if window.tof_steps == 1 {
                window.tof_min
            } else {
                window.tof_min
                    + (window.tof_max - window.tof_min) * j as f64 / (window.tof_steps - 1) as f64
            };
            stats.broad_evaluations += 1;
            if let Some(cell) = lambert_cell(ctx, epoch, tof, stats) {
                best.push(cell);
            }
        }
    }
    best.sort_by(|a, b| a.total_dv.total_cmp(&b.total_dv));
    best.truncate(ctx.config.keep_candidates);
}

/// Porkchop rendezvous search between two ephemeris bodies. Returns plans
/// ranked by exact total Δv (all survivors passed the miss filter), best
/// first, each carrying its measured miss. Deterministic: grid order and
/// total_cmp ordering, no hash iteration.
pub fn porkchop_search(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: SearchConfig,
) -> Result<(Vec<RankedPlan>, SearchStats), SearchError> {
    config.validate()?;
    let central = ephemeris
        .body(config.central_body)
        .map_err(SearchError::Ephemeris)?;
    let central_mu = central.mu;
    if !central_mu.is_finite() || central_mu <= 0.0 {
        return Err(SearchError::InvalidConfig);
    }
    let ctx = BroadCtx {
        ephemeris,
        config,
        central_mu,
        central_radius_m: central.radius_m,
        depot_mu: ephemeris
            .body(config.departure_body)
            .map_err(SearchError::Ephemeris)?
            .mu,
        depot_radius_m: ephemeris
            .body(config.departure_body)
            .map_err(SearchError::Ephemeris)?
            .radius_m,
        departure_planet: planet_of(ephemeris, config.central_body, config.departure_body),
        arrival_planet: planet_of(ephemeris, config.central_body, config.arrival_body),
    };
    let mut stats = SearchStats::default();
    // Phase 1: broad grid.
    let mut best = Vec::new();
    grid_best(
        &ctx,
        GridWindow {
            start_s: config.window_start.0,
            span_s: config.departure_span_s,
            steps: config.departure_steps,
            tof_min: config.tof_min_s,
            tof_max: config.tof_max_s,
            tof_steps: config.tof_steps,
        },
        &mut stats,
        &mut best,
    );
    if best.is_empty() {
        return Err(SearchError::NoViableTransfer { stats });
    }
    // Phase 2: refine around the winner (quarter window, 5x5, twice).
    let mut focus = best[0].clone();
    for _ in 0..2 {
        let mut local = Vec::new();
        let span = (focus.time_of_flight_s * 0.25).max(1.0);
        let tof_span = (config.tof_max_s - config.tof_min_s).max(1.0) * 0.125;
        grid_best(
            &ctx,
            GridWindow {
                start_s: focus.departure_epoch.0 - span,
                span_s: span * 2.0,
                steps: 5,
                tof_min: (focus.time_of_flight_s - tof_span).max(config.tof_min_s),
                tof_max: (focus.time_of_flight_s + tof_span).min(config.tof_max_s),
                tof_steps: 5,
            },
            &mut stats,
            &mut local,
        );
        if local.is_empty() {
            break;
        }
        focus = local[0].clone();
        if !best.iter().any(|cell| {
            (cell.departure_epoch.0 - focus.departure_epoch.0).abs() < 1.0
                && (cell.time_of_flight_s - focus.time_of_flight_s).abs() < 1.0
        }) {
            best.push(focus.clone());
        }
    }
    best.sort_by(|a, b| a.total_dv.total_cmp(&b.total_dv));
    best.truncate(config.keep_candidates);
    // Phase 3: phase the departure anomaly, then correct and rank.
    let mut ranked = Vec::new();
    for cell in &best {
        if let Some(plan) = revalidate(ephemeris, field, config, cell, &mut stats)? {
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

/// Loose propagation config for anomaly screening: 1e5x looser than the
/// exact pass, so screens run in a fraction of the time while keeping the
/// full N-body model (wells, moons, moving central body) — unlike a
/// fixed-central screen, which omits the depot-well turn entirely and
/// ranks anomalies by fiction.
fn loose_config() -> AdaptiveIntegratorConfig {
    AdaptiveIntegratorConfig {
        initial_step_s: 60.0,
        min_step_s: 1.0e-6,
        // Screens rank candidates against each other; the exact correction
        // afterwards measures truth with the tight config. A day-long
        // ceiling lets the adaptive controller stride deep cruise instead
        // of paying 24 forced steps per day (500 d = 12k minimum steps at
        // an hour cap); near a well the error controller shrinks the step
        // itself. Validated by unchanged cold-trajectory digits, not by
        // the tolerance name.
        max_step_s: 86_400.0,
        absolute_position_tolerance_m: 100.0,
        absolute_velocity_tolerance_mps: 1.0e-3,
        relative_tolerance: 1.0e-8,
        max_steps: 100_000,
    }
}

/// Exact correction config: same tight tolerances as the integrator
/// default, but a 6-hour ceiling instead of one hour. Deep-cruise arcs
/// are smooth on day scales (the error controller proves it by growing
/// the step itself); the hour cap forced >=4 steps/day of pure overhead
/// (500 d = 2k minimum steps). Near a well the controller shrinks below
/// the ceiling on its own, so encounter resolution is untouched.
/// Validated by unchanged cold-trajectory digits on the replay corpus,
/// not by the ceiling value. Deliberately maneuver-local: the global
/// default stays conservative for the authoritative flight loop.
fn exact_config() -> AdaptiveIntegratorConfig {
    AdaptiveIntegratorConfig {
        max_step_s: 21_600.0,
        ..AdaptiveIntegratorConfig::default()
    }
}

/// Departure-anomaly phasing: scan parking-orbit true anomalies with
/// loose-tolerance FULL N-body screens and keep the best departure state.
/// Screens rank candidates against each other; exact N-body correction
/// afterwards measures truth. Returns inertial departure point, parking
/// velocity and burn vector.
///
/// The scan covers parking-PLANE tilt as well as anomaly (launch-plane
/// selection). An in-plane-only departure forces the single midcourse TCM
/// to pay 100% of the plane change: measured on the Earth->Venus window, a
/// 93%-out-of-plane 1092 m/s TCM plus a 6.7 km/s-normal arrival mismatch,
/// while the Mars window from the same code pays 75% out-of-plane on a
/// smaller bill. Real launches pick the parking plane with the transfer;
/// the tilt scan (deg-scale, inner-planet inclinations are 0-7 deg) lets
/// the departure burn carry the declination instead of the TCM.
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_departure(
    field: &GravityField<'_>,
    depot_epoch: SimTime,
    depot: &BodyState,
    central: &BodyState,
    arrival: &BodyState,
    depot_mu: f64,
    park_radius: f64,
    burn_magnitude_mps: f64,
    time_of_flight_s: f64,
    stats: &mut SearchStats,
) -> Option<(DVec3, DVec3, DVec3)> {
    // Phasing basis: the depot orbit around the central body — or, when
    // depot == central (Apollo-class departure from the central body's own
    // parking orbit), the arrival orbit plane around the same body. The
    // anomaly scan below is identical either way: a circular parking orbit
    // of park_radius around `depot.position_inertial`.
    let radial_raw = depot.position_inertial - central.position_inertial;
    let (radial_unit, normal) = if radial_raw.length_squared() > 0.0 {
        let momentum = radial_raw.cross(depot.velocity_inertial - central.velocity_inertial);
        let normal = momentum.normalize();
        let radial_unit = radial_raw.normalize();
        if !normal.is_finite() || !radial_unit.is_finite() {
            return None;
        }
        (radial_unit, normal)
    } else {
        let to_arrival = arrival.position_inertial - central.position_inertial;
        let arrival_plane = to_arrival.cross(arrival.velocity_inertial - central.velocity_inertial);
        if to_arrival.length_squared() <= 0.0 || arrival_plane.length_squared() <= 0.0 {
            return None;
        }
        (to_arrival.normalize(), arrival_plane.normalize())
    };
    let tangent0 = normal.cross(radial_unit);
    let v_circ = (depot_mu / park_radius).sqrt();
    if !v_circ.is_finite() {
        return None;
    }
    // Evaluate one (tilt, anomaly) departure candidate with a loose
    // full-N-body screen; returns miss and departure state on success.
    // Tilt rotates the parking plane around the radial axis so the burn
    // can carry transfer declination, not just in-plane direction.
    // Pure function of its inputs (no stats mutation — the caller counts
    // the grid size once), so round grids below run on rayon. Minimum
    // selection stays a strict left-to-right fold over input order, hence
    // bit-identical to the serial loop on any worker count.
    let screen = |tilt_rad: f64, anomaly: f64| -> Option<(f64, DVec3, DVec3, DVec3)> {
        let (sin_t, cos_t) = tilt_rad.sin_cos();
        let tilted_tangent = tangent0 * cos_t - normal * sin_t;
        let (point_dir, tangent) = (
            radial_unit * anomaly.cos() + tilted_tangent * anomaly.sin(),
            tilted_tangent * anomaly.cos() - radial_unit * anomaly.sin(),
        );
        let point = depot.position_inertial + point_dir * park_radius;
        let park_velocity = depot.velocity_inertial + tangent * v_circ;
        let burn = tangent * burn_magnitude_mps;
        let flow = propagate_adaptive_with_burns(
            field,
            TestParticleState {
                position: point,
                velocity: park_velocity + burn,
            },
            depot_epoch,
            time_of_flight_s,
            &[],
            loose_config(),
        )
        .ok()?;
        let miss = (flow.state.position - arrival.position_inertial).length();
        if !miss.is_finite() {
            return None;
        }
        Some((miss, point, park_velocity, burn))
    };
    // Round 0: coarse tilt x anomaly grid. Round 1: refine around the
    // winner in both axes.
    let mut best: Option<(f64, f64, f64, DVec3, DVec3, DVec3)> = None;
    let mut center_tilt = 0.0;
    let mut center_angle = 0.0;
    for round in 0..2 {
        // Input grid in serial-loop order; rayon collect preserves it.
        let grid: Vec<(f64, f64)> = if round == 0 {
            [-30.0f64, -15.0, -7.5, 0.0, 7.5, 15.0, 30.0]
                .into_iter()
                .flat_map(|tilt_deg| {
                    let tilt = tilt_deg.to_radians();
                    (0..12).map(move |i| (tilt, std::f64::consts::TAU * i as f64 / 12.0))
                })
                .collect()
        } else {
            [-4.0f64, 0.0, 4.0]
                .into_iter()
                .flat_map(|tilt_step| {
                    let tilt = center_tilt + tilt_step.to_radians();
                    (0..8).map(move |i| {
                        let span = std::f64::consts::TAU / 6.0;
                        (tilt, center_angle - span / 2.0 + span * i as f64 / 7.0)
                    })
                })
                .collect()
        };
        stats.phase_screens += grid.len();
        let mut local_best: Option<(f64, f64, f64, DVec3, DVec3, DVec3)> = None;
        for (tilt, anomaly, miss, point, park_velocity, burn) in grid
            .par_iter()
            .map(|(tilt, anomaly)| screen(*tilt, *anomaly).map(|s| (*tilt, *anomaly, s)))
            .collect::<Vec<_>>()
            .into_iter()
            .flatten()
            .map(|(tilt, anomaly, (miss, point, park_velocity, burn))| {
                (tilt, anomaly, miss, point, park_velocity, burn)
            })
        {
            if local_best.is_none_or(|(best_miss, _, _, _, _, _)| miss < best_miss) {
                local_best = Some((miss, tilt, anomaly, point, park_velocity, burn));
            }
        }
        match local_best {
            Some((_, tilt, anomaly, _, _, _)) => {
                center_tilt = tilt;
                center_angle = anomaly;
                best = local_best;
            }
            None => break,
        }
    }
    best.map(|(_, _, _, point, park_velocity, burn)| (point, park_velocity, burn))
}

/// Departure phasing with a diverse shortlist: same round-0 tilt x anomaly
/// grid as [`phase_departure`], but keep the top-K DISTINCT starts instead
/// of refining a single winner.
///
/// Screens rank by miss against the AIM POINT (not the body center):
/// ranking against the center systematically prefers deep divers that
/// thread the singularity over grazers that bend near the aim sphere —
/// while the exact stage targets the sphere. Same reason the eccentricity
/// class filter below is meaningful: ranked screens end near the aim
/// (gentle dynamics), so their osculating e is well-defined instead of
/// periapsis garbage.
/// Ranking is position miss PLUS encounter-plane mismatch scaled to
/// meters (time of flight × broad encounter speed × plane angle). Loose
/// screens that arrive near the body but in the wrong plane poison the
/// next leg (a 30° plane error at 10 km/s costs kilometers per second to
/// fix downstream, while position errors are routine TCM work) — and
/// comparing raw velocities would drown in well fall-in instead, so only
/// plane NORMALS are compared (conserved, fall-in-free). Deterministic:
/// grid order, score order, angular separation greed.
#[allow(clippy::too_many_arguments)]
pub(crate) fn phase_departure_topk(
    field: &GravityField<'_>,
    depot_epoch: SimTime,
    depot: &BodyState,
    central: &BodyState,
    arrival: &BodyState,
    depot_mu: f64,
    park_radius: f64,
    burn_magnitude_mps: f64,
    time_of_flight_s: f64,
    target_plane_normal: DVec3,
    plane_speed_mps: f64,
    aim_point_m: DVec3,
    encounter_template: Option<EncounterTemplate>,
    keep: usize,
    stats: &mut SearchStats,
) -> Vec<(DVec3, DVec3, DVec3)> {
    if keep == 0 {
        return Vec::new();
    }
    let radial_raw = depot.position_inertial - central.position_inertial;
    let (radial_unit, normal) = if radial_raw.length_squared() > 0.0 {
        let momentum = radial_raw.cross(depot.velocity_inertial - central.velocity_inertial);
        let normal = momentum.normalize();
        let radial_unit = radial_raw.normalize();
        if !normal.is_finite() || !radial_unit.is_finite() {
            return Vec::new();
        }
        (radial_unit, normal)
    } else {
        let to_arrival = arrival.position_inertial - central.position_inertial;
        let arrival_plane = to_arrival.cross(arrival.velocity_inertial - central.velocity_inertial);
        if to_arrival.length_squared() <= 0.0 || arrival_plane.length_squared() <= 0.0 {
            return Vec::new();
        }
        (to_arrival.normalize(), arrival_plane.normalize())
    };
    let tangent0 = normal.cross(radial_unit);
    let v_circ = (depot_mu / park_radius).sqrt();
    if !v_circ.is_finite() {
        return Vec::new();
    }
    // Round-0 tilt x anomaly grid, evaluated in parallel: the 84 screens
    // are independent full-arc propagations (embarrassingly parallel —
    // parallelism belongs here, across independent jobs, never inside the
    // gravity kernel). rayon `collect` preserves input order, and every
    // step below (score sort, class filter, greedy distinct pick) is the
    // same deterministic sequence as the serial loop, so the shortlist is
    // identical bit-for-bit on any worker count.
    let grid: Vec<(f64, f64)> = [-30.0f64, -15.0, -7.5, 0.0, 7.5, 15.0, 30.0]
        .into_iter()
        .flat_map(|tilt_deg| {
            let tilt = tilt_deg.to_radians();
            (0..12).map(move |i| (tilt, std::f64::consts::TAU * i as f64 / 12.0))
        })
        .collect();
    let screened: Vec<_> = grid
        .par_iter()
        .map(|(tilt, anomaly)| {
            let (sin_t, cos_t) = tilt.sin_cos();
            let tilted_tangent = tangent0 * cos_t - normal * sin_t;
            let (point_dir, tangent) = (
                radial_unit * anomaly.cos() + tilted_tangent * anomaly.sin(),
                tilted_tangent * anomaly.cos() - radial_unit * anomaly.sin(),
            );
            let point = depot.position_inertial + point_dir * park_radius;
            let park_velocity = depot.velocity_inertial + tangent * v_circ;
            let burn = tangent * burn_magnitude_mps;
            let flow = propagate_adaptive_with_burns(
                field,
                TestParticleState {
                    position: point,
                    velocity: park_velocity + burn,
                },
                depot_epoch,
                time_of_flight_s,
                &[],
                loose_config(),
            )
            .ok()?;
            let miss = (flow.state.position - aim_point_m).length();
            if !miss.is_finite() {
                return None;
            }
            // Encounter-relative state for the fingerprint below.
            let rel_pos = flow.state.position - arrival.position_inertial;
            let rel_vel = flow.state.velocity - arrival.velocity_inertial;
            // Plane term: encounter-relative angular-momentum direction
            // vs broad's encounter plane. Degenerate screens score neutral.
            let plane_angle = (rel_pos.cross(rel_vel).try_normalize())
                .map(|screen_normal| {
                    screen_normal
                        .dot(target_plane_normal)
                        .clamp(-1.0, 1.0)
                        .acos()
                })
                .unwrap_or(std::f64::consts::FRAC_PI_2);
            // Periapsis-direction term: in-plane orientation of the
            // encounter hyperbola vs broad's. Energy-adjacent trajectories
            // can still arrive 90° off; only the full (plane, shape,
            // orientation) fingerprint sees that. Degenerate: neutral.
            let (screen_e, peri_angle) = match &encounter_template {
                Some(template) => {
                    let r = rel_pos.length();
                    let v2 = rel_vel.length_squared();
                    let e_vec = if r.is_finite() && r > 0.0 && v2.is_finite() {
                        ((v2 - template.mu / r) * rel_pos - rel_pos.dot(rel_vel) * rel_vel)
                            / template.mu
                    } else {
                        DVec3::NAN
                    };
                    let e = e_vec.length();
                    let peri = if e.is_finite() && e > 1.0 {
                        e_vec
                            .normalize()
                            .dot(template.periapsis_dir)
                            .clamp(-1.0, 1.0)
                            .acos()
                    } else {
                        std::f64::consts::FRAC_PI_2
                    };
                    (
                        if e.is_finite() { e } else { f64::NAN },
                        if peri.is_finite() {
                            peri
                        } else {
                            std::f64::consts::FRAC_PI_2
                        },
                    )
                }
                None => (f64::NAN, 0.0),
            };
            let score = if plane_angle.is_finite() {
                miss + time_of_flight_s * plane_speed_mps * (plane_angle + peri_angle)
            } else {
                return None;
            };
            Some((score, screen_e, *tilt, *anomaly, point, park_velocity, burn))
        })
        .collect();
    stats.phase_screens += grid.len();
    let mut scored: Vec<(f64, f64, f64, f64, DVec3, DVec3, DVec3)> =
        screened.into_iter().flatten().collect();
    scored.sort_by(|a, b| a.0.total_cmp(&b.0));
    // Encounter-class filter: keep screens whose osculating eccentricity
    // is within 5x of broad's (graze vs dive is an order-of-magnitude
    // distinction; a 44x-off dive needs a different burn architecture,
    // not a better seed). Falls back to the unfiltered pool when nothing
    // passes, so exotic-but-valid windows still fly.
    let pool: Vec<(f64, f64, f64, f64, DVec3, DVec3, DVec3)> = match encounter_template {
        Some(template)
            if template.eccentricity.is_finite() && template.eccentricity > 1.0 =>
        {
            let e_broad = template.eccentricity;
            let kept: Vec<_> = scored
                .iter()
                .filter(|(_, e, _, _, _, _, _)| {
                    e.is_finite() && *e > 1.0 && *e <= 5.0 * e_broad && *e >= e_broad / 5.0
                })
                .cloned()
                .collect();
            if std::env::var("THESSA_E_DBG").is_ok() {
                eprintln!("EFILTER e_broad={e_broad:.1} scored={} kept={}", scored.len(), kept.len());
            }
            if kept.is_empty() {
                scored.clone()
            } else {
                kept
            }
        }
        _ => scored.clone(),
    };
    // Greedy distinct pick: a new start must differ in anomaly (>30 deg
    // around the parking circle) or tilt (>5 deg) from every accepted one.
    // Nearby grid twins would waste exact budgets on the same handoff.
    let mut kept: Vec<(f64, f64, f64, f64, DVec3, DVec3, DVec3)> = Vec::new();
    for candidate in pool {
        if kept.len() >= keep {
            break;
        }
        let (_, _, tilt, anomaly, _, _, _) = candidate;
        let distinct = kept.iter().all(|(_, _, kept_tilt, kept_anomaly, _, _, _)| {
            let mut delta_angle = (anomaly - kept_anomaly).abs() % std::f64::consts::TAU;
            if delta_angle > std::f64::consts::PI {
                delta_angle = std::f64::consts::TAU - delta_angle;
            }
            delta_angle > 30.0f64.to_radians() || (tilt - kept_tilt).abs() > 5.0f64.to_radians()
        });
        if distinct {
            kept.push(candidate);
        }
    }
    kept.into_iter()
        .map(|(_, _, _, _, point, park_velocity, burn)| (point, park_velocity, burn))
        .collect()
}
/// Broad encounter fingerprint for phasing selection: eccentricity
/// (graze vs dive energy class), periapsis direction (in-plane
/// orientation), and body mu. Screens match against it instead of trusting
/// position miss alone. All free vectors (frame-independent).
#[derive(Debug, Clone, Copy)]
pub(crate) struct EncounterTemplate {
    pub eccentricity: f64,
    pub periapsis_dir: DVec3,
    pub mu: f64,
}

impl EncounterTemplate {
    /// Build from broad incoming/outgoing asymptotes (encounter frame).
    /// Returns `None` for degenerate/straight encounters with no
    /// meaningful class to enforce.
    pub(crate) fn from_bend(v_in: DVec3, v_out: DVec3, mu: f64) -> Option<Self> {
        if !mu.is_finite() || mu <= 0.0 {
            return None;
        }
        let cos_turn = v_in.normalize().dot(v_out.normalize()).clamp(-1.0, 1.0);
        if !cos_turn.is_finite() || cos_turn >= 1.0 {
            return None;
        }
        let delta = cos_turn.acos();
        let eccentricity = 1.0 / (delta / 2.0).sin();
        let periapsis_dir = (v_out.normalize() - v_in.normalize()).try_normalize()?;
        if !eccentricity.is_finite()
            || eccentricity <= 1.0
            || !periapsis_dir.is_finite()
        {
            return None;
        }
        Some(Self {
            eccentricity,
            periapsis_dir,
            mu,
        })
    }
}

/// Midcourse epoch: past depot-escape, with margin on both sides. Shared
/// by direct and flyby legs (same correction architecture).
pub(crate) fn midcourse_time_s(time_of_flight_s: f64) -> f64 {
    (time_of_flight_s / 4.0)
        .max(3_600.0)
        .min((time_of_flight_s - 3_600.0).max(3_600.0))
}
/// Midcourse differential correction on the full N-body dynamics.
///
/// Architecture (measured, not assumed): correcting the DEPARTURE burn
/// stalls — the escape turn off the parking orbit makes departure-burn
/// targeting ill-conditioned (narrow valley, trust collapse, ~2%/iter
/// crawl). Correcting a MIDCOURSE burn placed post-escape in clean cruise
/// converges smoothly with plain damped Newton (~2x/iter): the departure
/// burn stays at its phased patched-conic value, the midcourse burn
/// absorbs everything downstream. The plan gains a TCM node, exactly like
/// flown missions.
///
/// Varies the midcourse burn (3 DOF) to drive the arrival miss to ~km
/// with a finite-difference STM. Each iteration is exact propagation, so
/// the converged trajectory needs no separate revalidation pass. Returns
/// departure burn (unchanged), midcourse burn, end state and miss; None
/// only on total failure (no finite evaluation at all).
#[allow(clippy::too_many_arguments)]
pub(crate) fn correct_shooting(
    field: &GravityField<'_>,
    start_pos: DVec3,
    park_velocity: DVec3,
    departure_burn: DVec3,
    departure_epoch: SimTime,
    time_of_flight_s: f64,
    mid_time_s: f64,
    aim_point_m: DVec3,
    initial_mid_burn: DVec3,
    stats: &mut SearchStats,
) -> Option<(DVec3, DVec3, TestParticleState, f64)> {
    const TARGET_MISS_M: f64 = 2_000.0;
    const MAX_ITERS: usize = 30;
    // Trust cap, not damping: starting steps above ~300 m/s overshoot the
    // gentle basin into hot regimes the correction then cannot leave
    // (measured: 2000 m/s first steps converged 5+ km/s hot). Inside the
    // cap, plain Newton finishes quadratically on its own.
    let mut shoot = |mid_burn: DVec3| -> Option<TestParticleState> {
        stats.newton_propagations += 1;
        propagate_adaptive_with_burns(
            field,
            TestParticleState {
                position: start_pos,
                velocity: park_velocity + departure_burn,
            },
            departure_epoch,
            time_of_flight_s,
            &[ImpulsiveBurn {
                time_s: mid_time_s,
                delta_v_mps: mid_burn,
            }],
            exact_config(),
        )
        .ok()
        .map(|result| result.state)
    };
    let mut mid_burn = if initial_mid_burn.is_finite() {
        initial_mid_burn
    } else {
        DVec3::ZERO
    };
    let mut best: Option<(DVec3, TestParticleState, f64)> = None;
    // The current-point evaluation is carried between iterations so the
    // backtracking trial below is not paid twice. Success-path behavior is
    // intentionally identical to plain accept-always Newton (same
    // trajectories, same count): backtracking only engages on evaluation
    // failure, which previously killed the whole run.
    let mut end = shoot(mid_burn)?;
    // Hot-stall early exit (Voyager lesson): a converged leg improves its
    // miss by orders of magnitude per iteration, so five straight
    // iterations without even a 1% gain mean Newton is wandering, not
    // converging. Past a 2 km/s midcourse burn (cold TCMs measure <= ~900
    // on Luna/Mars/Venus legs) that wander is a hot leg: quit and return
    // the best seen instead of burning the remaining iterations (each
    // costs four full-arc N-body propagations — minutes on 1000 d legs).
    // Cold trajectories never trip this: they either converge (exiting
    // above) or improve by orders per step (resetting the stall count),
    // so their paths stay bit-identical.
    const HOT_BURN_MPS: f64 = 2_000.0;
    const STALL_ITERS: u32 = 5;
    let mut stall_iters: u32 = 0;
    for _ in 0..MAX_ITERS {
        let miss_vec = aim_point_m - end.position;
        let miss = miss_vec.length();
        if !miss.is_finite() {
            break;
        }
        if best.is_none_or(|(_, _, best_miss)| miss < best_miss) {
            let improved = match best {
                Some((_, _, best_miss)) => miss < 0.99 * best_miss,
                None => true,
            };
            best = Some((mid_burn, end, miss));
            stall_iters = if improved { 0 } else { stall_iters.saturating_add(1) };
        } else {
            stall_iters = stall_iters.saturating_add(1);
        }
        if miss <= TARGET_MISS_M {
            break;
        }
        if mid_burn.length() > HOT_BURN_MPS && stall_iters >= STALL_ITERS {
            stats.hot_stall_exits += 1;
            break;
        }
        // Finite-difference step scaled for CONSTANT ~1e5 m displacement at
        // the target: h = 0.5 m/s resolves lunar legs (proven); year-long
        // inter-body arcs need ~1e-3, otherwise the perturbation spans
        // nonlinear encounter regimes and the Jacobian is garbage. Capped
        // at the proven 0.5 so short legs follow bit-identical paths.
        let leverage_s = (time_of_flight_s - mid_time_s).max(1.0);
        let h = (1.0e5 / leverage_s).min(0.5);
        let mut columns = [DVec3::ZERO; 3];
        let mut ok = true;
        for (column, axis) in [DVec3::X, DVec3::Y, DVec3::Z].iter().enumerate() {
            match shoot(mid_burn + *axis * h) {
                Some(perturbed) => {
                    columns[column] = (perturbed.position - end.position) / h;
                }
                None => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            break;
        }
        let mut step = DMat3::from_cols(columns[0], columns[1], columns[2]).inverse() * miss_vec;
        if !step.is_finite() {
            break;
        }
        const TRUST_MPS: f64 = 300.0;
        if step.length() > TRUST_MPS {
            step *= TRUST_MPS / step.length();
        }
        // Backtracking line search on evaluation failure ONLY: a full
        // Newton step near a deep well can capture into a MaxSteps orbit
        // or impact, while a halved step stays in the escaping regime.
        // Finite steps are accepted exactly as before, so smooth runs pay
        // nothing and follow the identical path.
        let mut trial = step;
        let mut next_end = None;
        for _ in 0..4 {
            match shoot(mid_burn + trial) {
                Some(state) => {
                    next_end = Some(state);
                    break;
                }
                None => trial *= 0.5,
            }
        }
        match next_end {
            Some(state) => {
                mid_burn += trial;
                end = state;
            }
            None => break,
        }
        if !mid_burn.is_finite() || mid_burn.length() > 50_000.0 {
            break;
        }
    }
    best.map(|(burn1, end, miss)| (departure_burn, burn1, end, miss))
}

/// B-plane differential correction for flyby encounters: vary one burn
/// (3 DOF) against the 2D encounter-plane miss, minimum-norm.
///
/// A flyby needs the right B-plane crossing at the right epoch, not an
/// exact 3D point hit: along-track slop at fixed epoch just shifts the
/// encounter slightly, while forcing an arbitrary sphere point can demand
/// absurd burns (measured 35 km/s at Jupiter for a broad 0.4 km/s bend —
/// the 3D aim over-constrains what the assist actually needs). Two
/// constraints with three controls gives an underdetermined system, so
/// the minimum-norm step keeps burns small BY CONSTRUCTION (Δ =
/// Jᵀ(JJᵀ)⁻¹·r with an explicit 2x2 inverse — no new solver machinery).
/// The S axis (incoming asymptote direction, broad-stable) is fixed for
/// the solve; callers keep the honest 3D miss for gates and reporting.
///
/// Returns burn, end state and the 2D miss; None only on total failure.
/// Callers must check the 3D miss themselves.
#[allow(clippy::too_many_arguments)]
pub(crate) fn correct_bplane_shooting(
    field: &GravityField<'_>,
    start_pos: DVec3,
    start_vel: DVec3,
    departure_epoch: SimTime,
    time_of_flight_s: f64,
    mid_time_s: f64,
    aim_point_m: DVec3,
    s_dir: DVec3,
    initial_burn: DVec3,
    stats: &mut SearchStats,
) -> Option<(DVec3, TestParticleState, f64)> {
    const TARGET_MISS_M: f64 = 2_000.0;
    const MAX_ITERS: usize = 30;
    const TRUST_MPS: f64 = 300.0;
    // Encounter-plane basis: T ⊥ S, R = S × T. Fixed for the solve
    // (broad-stable, not re-estimated from noisy exact states).
    let s = s_dir.try_normalize()?;
    if !s.is_finite() {
        return None;
    }
    let reference = if s.x.abs() < 0.9 && s.y.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::X
    };
    let t_axis = s.cross(reference).normalize();
    let r_axis = s.cross(t_axis).normalize();
    if !t_axis.is_finite() || !r_axis.is_finite() {
        return None;
    }
    let project = |point: DVec3| -> (f64, f64) {
        let relative = aim_point_m - point;
        (relative.dot(t_axis), relative.dot(r_axis))
    };
    let mut shoot = |burn: DVec3| -> Option<TestParticleState> {
        stats.newton_propagations += 1;
        propagate_adaptive_with_burns(
            field,
            TestParticleState {
                position: start_pos,
                velocity: start_vel,
            },
            departure_epoch,
            time_of_flight_s,
            &[ImpulsiveBurn {
                time_s: mid_time_s,
                delta_v_mps: burn,
            }],
            exact_config(),
        )
        .ok()
        .map(|result| result.state)
    };
    let mut burn = if initial_burn.is_finite() {
        initial_burn
    } else {
        DVec3::ZERO
    };
    let mut end = shoot(burn)?;
    let mut best: Option<(DVec3, TestParticleState, f64)> = None;
    for _ in 0..MAX_ITERS {
        let (miss_t, miss_r) = project(end.position);
        let miss = (miss_t * miss_t + miss_r * miss_r).sqrt();
        if !miss.is_finite() {
            break;
        }
        if best.is_none_or(|(_, _, best_miss)| miss < best_miss) {
            best = Some((burn, end, miss));
        }
        if miss <= TARGET_MISS_M {
            break;
        }
        let leverage_s = (time_of_flight_s - mid_time_s).max(1.0);
        let h = (1.0e5 / leverage_s).min(0.5);
        // 2x3 Jacobian of END POSITION (NOT of the miss): with
        // miss = aim − end the step solves J·Δ = miss exactly like
        // single-leg shooting. Differentiating (aim − end) instead
        // negates every step into an ascent while magnitudes look sane.
        let mut jac = [[0.0f64; 3]; 2];
        let mut ok = true;
        for (axis_n, axis) in [DVec3::X, DVec3::Y, DVec3::Z].iter().enumerate() {
            match shoot(burn + *axis * h) {
                Some(perturbed) => {
                    let slope = (perturbed.position - end.position) / h;
                    jac[0][axis_n] = slope.dot(t_axis);
                    jac[1][axis_n] = slope.dot(r_axis);
                }
                None => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            break;
        }
        // Minimum-norm step via explicit 2x2 (JJᵀ)⁻¹.
        let jjt = [
            [
                jac[0][0] * jac[0][0] + jac[0][1] * jac[0][1] + jac[0][2] * jac[0][2],
                jac[0][0] * jac[1][0] + jac[0][1] * jac[1][1] + jac[0][2] * jac[1][2],
            ],
            [
                jac[0][0] * jac[1][0] + jac[0][1] * jac[1][1] + jac[0][2] * jac[1][2],
                jac[1][0] * jac[1][0] + jac[1][1] * jac[1][1] + jac[1][2] * jac[1][2],
            ],
        ];
        let det = jjt[0][0] * jjt[1][1] - jjt[0][1] * jjt[1][0];
        if !det.is_finite() || det.abs() <= 0.0 {
            break;
        }
        let inv = [
            [jjt[1][1] / det, -jjt[0][1] / det],
            [-jjt[1][0] / det, jjt[0][0] / det],
        ];
        let lambda_t = inv[0][0] * miss_t + inv[0][1] * miss_r;
        let lambda_r = inv[1][0] * miss_t + inv[1][1] * miss_r;
        // Δ = Jᵀλ: burn-axis j gets T-col[j]·λt + R-col[j]·λr.
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let mut step = DVec3::ZERO;
        for (axis_n, axis) in axes.iter().enumerate() {
            step += *axis * (jac[0][axis_n] * lambda_t + jac[1][axis_n] * lambda_r);
        }
        if !step.is_finite() {
            break;
        }
        if step.length() > TRUST_MPS {
            step *= TRUST_MPS / step.length();
        }
        // Merit acceptance on the 2D miss (coupled-leg lesson: never
        // accept a worsening step blindly). Halve to a genuinely better
        // point or give up with the best seen.
        let mut trial = step;
        let mut next: Option<(DVec3, TestParticleState, f64)> = None;
        for _ in 0..6 {
            if let Some(state) = shoot(burn + trial) {
                let (ct, cr) = project(state.position);
                let candidate = (ct * ct + cr * cr).sqrt();
                if candidate.is_finite() && candidate < miss {
                    next = Some((burn + trial, state, candidate));
                    break;
                }
            }
            trial *= 0.5;
        }
        match next {
            Some((candidate, state, _)) => {
                burn = candidate;
                end = state;
            }
            None => break,
        }
        if !burn.is_finite() || burn.length() > 50_000.0 {
            break;
        }
    }
    best
}
fn revalidate(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: SearchConfig,
    cell: &Cell,
    stats: &mut SearchStats,
) -> Result<Option<RankedPlan>, SearchError> {
    let arrival_epoch = SimTime(cell.departure_epoch.0 + cell.time_of_flight_s);
    let arrival = ephemeris
        .body_state(config.arrival_body, arrival_epoch)
        .map_err(SearchError::Ephemeris)?;
    let target = ephemeris
        .body(config.arrival_body)
        .map_err(SearchError::Ephemeris)?;
    let depot = ephemeris
        .body_state(config.departure_body, cell.departure_epoch)
        .map_err(SearchError::Ephemeris)?;
    let central = ephemeris
        .body_state(config.central_body, cell.departure_epoch)
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
    // Phase 2.5: pick the parking-orbit anomaly whose loose-tolerance
    // full N-body screen comes closest to the arrival body.
    let (point, park_velocity, phased_burn) = match phase_departure(
        field,
        cell.departure_epoch,
        &depot,
        &central,
        &arrival,
        depot_mu,
        park_radius,
        cell.departure_burn_mag_mps,
        cell.time_of_flight_s,
        stats,
    ) {
        Some(phased) => phased,
        None => return Ok(None),
    };
    // Aim at the standoff point (target center plus radial offset): the
    // correction converges there without ever integrating into the
    // point-mass singularity, and the miss is measured against a physical
    // rendezvous sphere rather than a mathematical point.
    let central_arr = ephemeris
        .body_state(config.central_body, arrival_epoch)
        .map_err(SearchError::Ephemeris)?;
    let aim_dir = (arrival.position_inertial - central_arr.position_inertial).normalize();
    if !aim_dir.is_finite() {
        stats.failed_revalidations += 1;
        return Ok(None);
    }
    let aim = arrival.position_inertial + aim_dir * (target.radius_m + config.standoff_m);
    // Phase 3: midcourse correction on full N-body dynamics (departure
    // fixed at its phased value — correcting it stalls in the escape
    // turn). The converged trajectory IS the revalidation.
    let mid_time_s = midcourse_time_s(cell.time_of_flight_s);
    // Diagnostic: how hot is the phased (uncorrected) arrival?
    let (dep_burn, tcm_burn, end, miss) = match correct_shooting(
        field,
        point,
        park_velocity,
        phased_burn,
        cell.departure_epoch,
        cell.time_of_flight_s,
        mid_time_s,
        aim,
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
    // Lithobraking is not rendezvous: trajectories ending inside the target
    // body are filtered, not ranked (and never integrated further). Note
    // the impact guard compares distance-to-CENTER against the body radius;
    // `miss` is measured against the standoff aim point, so comparing it
    // against R would eat every converged trajectory.
    let dist_center = (end.position - arrival.position_inertial).length();
    if !miss.is_finite() || miss > config.max_miss_m || dist_center < target.radius_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    let arrival_burn = arrival.velocity_inertial - end.velocity;
    if !arrival_burn.is_finite() {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    // Three-node plan (departure, TCM, arrival-match); a sub-1 m/s TCM is
    // omitted for clean plans (the executor skips zero nodes anyway).
    let tcm_epoch = SimTime(cell.departure_epoch.0 + mid_time_s);
    let mut nodes = vec![
        ManeuverNode::new(cell.departure_epoch, dep_burn)
            .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
    ];
    if tcm_burn.length() >= 1.0 {
        nodes.push(
            ManeuverNode::new(tcm_epoch, tcm_burn)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
        );
    }
    nodes.push(
        ManeuverNode::new(arrival_epoch, arrival_burn)
            .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
    );
    let mut plan = ManeuverPlan::new(
        nodes,
        // The plan describes what actually flies: phased parking-orbit
        // departure state with the escape burn applied.
        point,
        park_velocity + dep_burn,
        cell.departure_epoch,
    )
    .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?;
    plan.predicted_miss_m = Some(miss);
    Ok(Some(RankedPlan {
        exact_total_dv_mps: plan.total_dv_mps(),
        departure_epoch: cell.departure_epoch,
        time_of_flight_s: cell.time_of_flight_s,
        broad_total_dv_mps: cell.total_dv,
        plan,
        exact_miss_m: miss,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{BakedBody, BodyId, KeplerOrbit};

    fn mini_system() -> BakedEphemeris {
        let orbit = |a: f64, m0: f64| {
            KeplerOrbit::new(1.0e14, a, 0.0, 0.0, 0.0, 0.0, m0).expect("valid test orbit")
        };
        BakedEphemeris::new(
            "TEST_MINI",
            vec![
                BakedBody::fixed(BodyId(0), "center", 1.0e14, 0.0),
                BakedBody::orbital(
                    BodyId(1),
                    "depot",
                    1.0e12,
                    0.0,
                    BodyId(0),
                    orbit(1.0e7, 0.0),
                ),
                BakedBody::orbital(
                    BodyId(2),
                    "target",
                    1.0e12,
                    0.0,
                    BodyId(0),
                    orbit(1.5e7, 2.0),
                ),
            ],
        )
        .expect("valid test system")
    }

    fn mini_config() -> SearchConfig {
        SearchConfig {
            central_body: BodyId(0),
            departure_body: BodyId(1),
            arrival_body: BodyId(2),
            window_start: SimTime(0.0),
            departure_span_s: 40_000.0,
            departure_steps: 6,
            tof_min_s: 8_000.0,
            tof_max_s: 20_000.0,
            tof_steps: 6,
            keep_candidates: 1,
            standoff_m: 100_000.0,
            max_broad_dv_mps: 20_000.0,
            max_miss_m: 1.0e6,
        }
    }

    #[test]
    fn porkchop_finds_validated_plan() {
        let ephemeris = mini_system();
        let field = GravityField::from_ephemeris(&ephemeris);
        let (ranked, stats) =
            porkchop_search(&ephemeris, &field, mini_config()).expect("search finds");
        assert_eq!(ranked.len(), 1);
        let winner = &ranked[0];
        assert!(winner.plan.nodes.len() >= 2);
        assert!(winner.exact_miss_m <= 1.0e6);
        assert!(winner.exact_total_dv_mps.is_finite() && winner.exact_total_dv_mps > 0.0);
        assert!(winner.plan.predicted_miss_m.is_some());
        assert!(stats.exact_revalidations >= 1);
        // Deterministic: same inputs, same winner.
        let (ranked2, _) =
            porkchop_search(&ephemeris, &field, mini_config()).expect("search finds");
        assert_eq!(ranked, ranked2);
    }

    #[test]
    fn invalid_config_rejected() {
        let ephemeris = mini_system();
        let field = GravityField::from_ephemeris(&ephemeris);
        let mut bad = mini_config();
        bad.departure_steps = 0;
        assert_eq!(
            porkchop_search(&ephemeris, &field, bad),
            Err(SearchError::InvalidConfig)
        );
    }
}
