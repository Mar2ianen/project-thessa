//! Single-gravity-assist tour search: departure moon -> flyby moon ->
//! arrival moon around one central body, with exact N-body revalidation.
//!
//! Formal I/O:
//! - IN: [`FlybyConfig`] (bodies, departure window, per-leg time-of-flight
//!   ranges, Δv caps) plus the baked ephemeris and gravity field.
//! - OUT: [`RankedPlan`]s (the shared validated-plan type) ordered by exact
//!   total Δv, each carrying its measured arrival miss AND its
//!   [`FlybyEvent`] (body, epoch, periapsis, powered-burn share). Only
//!   corrected plans return; broad-only routes never leave this module.
//!
//! Model (patched-conic broad, exact narrow — docs/23 §12 pattern):
//! - Broad: two Lambert arcs sharing the flyby body's position at the
//!   flyby epoch. Departure is priced as patched parking-orbit escape (the
//!   same helper as the direct search), the flyby as an impulsive powered
//!   bend `|v_out - v_in|` at periapsis in the flyby-body frame, arrival as
//!   a rendezvous match. The bend limit is REPORTED (free vs paid bend),
//!   not enforced: a powered burn realizes any turn angle impulsively at
//!   periapsis, so the limit only tells whether the bend was free.
//! - Narrow: leg 1 is anomaly-phased and differentially corrected to a
//!   periapsis aim point exactly like a direct arrival; leg 2 SOLVES the
//!   periapsis burn (seeded by the broad burn) to hit the arrival aim
//!   point — the full N-body dynamics inside every correction evaluation
//!   absorbs well depth, focusing and handoff slop into the solved burn,
//!   so no downstream TCM is needed. The converged two-leg trajectory IS
//!   the revalidation.

use glam::DVec3;
use thessa_sim_core::{BakedEphemeris, BodyId, GravityField, SimTime};

use crate::lambert::solve_lambert_prograde;
use crate::patch::{planet_arrival_match_mag, planet_escape_moon_vinf, planet_of};
use crate::plan::{FlybyEvent, ManeuverNode, ManeuverPlan};
use crate::search::{
    RankedPlan, SearchError, SearchStats, correct_shooting, midcourse_time_s, patched_escape_mag,
    phase_departure, transfer_perigee_m,
};

/// Maximum unpowered turn angle (rad) for a flyby with `v_inf_mps` at
/// infinity past periapsis `periapsis_m` around `mu`: `2*asin(1/e)` with
/// `e = 1 + r_p*v_inf^2/mu`. Returns 0 for degenerate inputs (no
/// encounter, no bend) rather than NaN.
pub fn max_bend_angle_rad(v_inf_mps: f64, mu: f64, periapsis_m: f64) -> f64 {
    // Explicit finite checks (not `<= 0.0`, which lets NaN through).
    if !v_inf_mps.is_finite() || v_inf_mps <= 0.0 {
        return 0.0;
    }
    if !mu.is_finite() || mu <= 0.0 {
        return 0.0;
    }
    if !periapsis_m.is_finite() || periapsis_m <= 0.0 {
        return 0.0;
    }
    let eccentricity = 1.0 + periapsis_m * v_inf_mps * v_inf_mps / mu;
    if !eccentricity.is_finite() || eccentricity < 1.0 {
        return 0.0;
    }
    2.0 * (1.0 / eccentricity).clamp(-1.0, 1.0).asin()
}

/// Powered-flyby burn at periapsis (body frame): the vector change between
/// the incoming and outgoing asymptotes. Exact for an impulsive burn;
/// near-zero length means the bend was free (unpowered gravity assist).
pub fn powered_flyby_burn_mps(v_in_body_frame_mps: DVec3, v_out_body_frame_mps: DVec3) -> DVec3 {
    v_out_body_frame_mps - v_in_body_frame_mps
}

#[derive(Debug, Clone, PartialEq)]
pub struct FlybyConfig {
    /// Body hosting the two-body Lambert model for both legs.
    pub central_body: BodyId,
    /// Depot body: the craft departs with its full inertial state.
    pub departure_body: BodyId,
    /// Target body for arrival matching.
    pub arrival_body: BodyId,
    /// Bodies to swing by (see [`candidate_flyby_bodies`]).
    pub flyby_bodies: Vec<BodyId>,
    pub window_start: SimTime,
    pub departure_span_s: f64,
    pub departure_steps: usize,
    pub leg1_min_s: f64,
    pub leg1_max_s: f64,
    pub leg1_steps: usize,
    pub leg2_min_s: f64,
    pub leg2_max_s: f64,
    pub leg2_steps: usize,
    /// Routes carried into exact revalidation.
    pub keep_routes: usize,
    /// Parking-orbit / flyby-periapsis / rendezvous-approach altitude (m).
    pub standoff_m: f64,
    /// Broad routes above this total Δv are pruned, not ranked.
    pub max_broad_dv_mps: f64,
    /// Plans missing by more than this are dropped, not ranked.
    pub max_miss_m: f64,
}

impl FlybyConfig {
    fn validate(&self) -> Result<(), SearchError> {
        if !self.window_start.0.is_finite()
            || !self.departure_span_s.is_finite()
            || self.departure_span_s < 0.0
            || self.departure_steps == 0
            || !self.leg1_min_s.is_finite()
            || self.leg1_min_s <= 0.0
            || !self.leg1_max_s.is_finite()
            || self.leg1_max_s < self.leg1_min_s
            || self.leg1_steps == 0
            || !self.leg2_min_s.is_finite()
            || self.leg2_min_s <= 0.0
            || !self.leg2_max_s.is_finite()
            || self.leg2_max_s < self.leg2_min_s
            || self.leg2_steps == 0
            || self.keep_routes == 0
            || !self.standoff_m.is_finite()
            || self.standoff_m <= 0.0
            || !self.max_broad_dv_mps.is_finite()
            || self.max_broad_dv_mps <= 0.0
            || !self.max_miss_m.is_finite()
            || self.max_miss_m < 0.0
            || self.flyby_bodies.is_empty()
        {
            return Err(SearchError::InvalidConfig);
        }
        Ok(())
    }
}

/// Moons to swing by: gravity sources orbiting `central` directly, except
/// the departure/arrival bodies and massless bookkeeping points. Deterministic:
/// ephemeris order.
pub fn candidate_flyby_bodies(
    ephemeris: &BakedEphemeris,
    central: BodyId,
    departure: BodyId,
    arrival: BodyId,
) -> Vec<BodyId> {
    ephemeris
        .bodies()
        .iter()
        .filter(|body| {
            body.parent == Some(central)
                && body.id != departure
                && body.id != arrival
                && body.gravity_source
                && body.mu > 0.0
        })
        .map(|body| body.id)
        .collect()
}

#[derive(Clone)]
struct FlybyCell {
    departure_epoch: SimTime,
    tof_leg1_s: f64,
    tof_leg2_s: f64,
    flyby_body: BodyId,
    departure_burn_mag_mps: f64,
    /// Desired leg-2 departure asymptote in the flyby-body frame (broad);
    /// the exact flyby burn is recomputed against the corrected leg 1.
    v_out_flyby_frame_mps: DVec3,
    total_dv: f64,
}

struct FlybyCtx<'a> {
    ephemeris: &'a BakedEphemeris,
    config: FlybyConfig,
    central_mu: f64,
    central_radius_m: f64,
    depot_mu: f64,
    depot_radius_m: f64,
    /// Intermediate planet wells (None for direct-central moons).
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

#[allow(clippy::too_many_arguments)]
fn flyby_cell(
    ctx: &FlybyCtx<'_>,
    flyby_body: BodyId,
    flyby_mu: f64,
    flyby_radius_m: f64,
    departure_epoch: SimTime,
    tof_leg1_s: f64,
    tof_leg2_s: f64,
    stats: &mut SearchStats,
) -> Option<FlybyCell> {
    let ephemeris = ctx.ephemeris;
    let config = &ctx.config;
    let flyby_epoch = SimTime(departure_epoch.0 + tof_leg1_s);
    let arrival_epoch = SimTime(flyby_epoch.0 + tof_leg2_s);
    let central_dep = ephemeris
        .body_state(config.central_body, departure_epoch)
        .ok()?;
    let central_fly = ephemeris
        .body_state(config.central_body, flyby_epoch)
        .ok()?;
    let central_arr = ephemeris
        .body_state(config.central_body, arrival_epoch)
        .ok()?;
    let departure = ephemeris
        .body_state(config.departure_body, departure_epoch)
        .ok()?;
    let flyby = ephemeris.body_state(flyby_body, flyby_epoch).ok()?;
    let arrival = ephemeris
        .body_state(config.arrival_body, arrival_epoch)
        .ok()?;
    if !central_dep.position_inertial.is_finite()
        || !central_fly.position_inertial.is_finite()
        || !central_arr.position_inertial.is_finite()
    {
        stats.degenerate_cells += 1;
        return None;
    }
    let r1 = departure.position_inertial - central_dep.position_inertial;
    let v1 = departure.velocity_inertial - central_dep.velocity_inertial;
    let rf = flyby.position_inertial - central_fly.position_inertial;
    let vf = flyby.velocity_inertial - central_fly.velocity_inertial;
    let r2 = arrival.position_inertial - central_arr.position_inertial;
    let v2 = arrival.velocity_inertial - central_arr.velocity_inertial;
    let arc1 = match solve_lambert_prograde(r1, v1, rf, tof_leg1_s, ctx.central_mu) {
        Ok(arc) => arc,
        Err(_) => {
            stats.degenerate_cells += 1;
            return None;
        }
    };
    let arc2 = match solve_lambert_prograde(rf, vf, r2, tof_leg2_s, ctx.central_mu) {
        Ok(arc) => arc,
        Err(_) => {
            stats.degenerate_cells += 1;
            return None;
        }
    };
    // Same depot sanity as the direct search: a radial depot trajectory
    // has no parking-orbit plane to phase in.
    if r1.cross(v1).length_squared() <= 0.0 {
        stats.degenerate_cells += 1;
        return None;
    }
    let park_radius = ctx.depot_radius_m + config.standoff_m;
    // Departure pricing with the planet patch when the depot sits behind
    // a planet well (moon-only burns cannot leave it); the flyby bend
    // itself stays single-central by construction (candidates orbit the
    // central body directly).
    let dep_mag = match ctx.departure_planet {
        None => {
            let v_inf = (arc1.departure_velocity_mps - v1).length();
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
                ephemeris.body_state(planet, departure_epoch),
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
            let v_inf_planet = arc1.departure_velocity_mps - planet_state.velocity_inertial;
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
    // Flyby in the flyby-body frame: incoming vs outgoing asymptote.
    let v_in_f = arc1.arrival_velocity_mps - vf;
    let v_out_f = arc2.departure_velocity_mps - vf;
    if v_in_f.length_squared() <= 0.0 || v_out_f.length_squared() <= 0.0 {
        stats.degenerate_cells += 1;
        return None;
    }
    let flyby_burn = powered_flyby_burn_mps(v_in_f, v_out_f);
    if !flyby_burn.is_finite() {
        stats.degenerate_cells += 1;
        return None;
    }
    // Energy-aware broad turn price: the burn happens at periapsis where
    // the well adds v_esc, so center-velocity pricing underprices deep-well
    // turns by km/s (measured: broad ~1-2k vs exact 7.8k at Thessa) and the
    // grid then prefers hot arrivals. Floor the price at the
    // periapsis-energy difference — a valid lower bound by the triangle
    // inequality; the exact solver prices truth for ranking.
    let mut turn_price = flyby_burn.length();
    let periapsis_m = flyby_radius_m + config.standoff_m;
    if flyby_mu > 0.0 && periapsis_m > 0.0 {
        let v_esc_sq = 2.0 * flyby_mu / periapsis_m;
        let q_in = (v_in_f.length_squared() + v_esc_sq).sqrt();
        let q_out = (v_out_f.length_squared() + v_esc_sq).sqrt();
        if q_in.is_finite() && q_out.is_finite() {
            turn_price = turn_price.max((q_out - q_in).abs());
        }
    }
    // Arrival pricing: planet patch (both wells inclusive) or direct match.
    let arr_mag = match ctx.arrival_planet {
        None => {
            let arr_burn = v2 - arc2.arrival_velocity_mps;
            if !arr_burn.is_finite() {
                stats.degenerate_cells += 1;
                return None;
            }
            arr_burn.length()
        }
        Some(planet) => {
            let (planet_state, planet_mu, moon_mu, moon_radius) = match (
                ephemeris.body_state(planet, arrival_epoch),
                ephemeris.body(planet),
                ephemeris.body(config.arrival_body),
            ) {
                (Ok(state), Ok(planet_body), Ok(moon_body)) => {
                    (state, planet_body.mu, moon_body.mu, moon_body.radius_m)
                }
                _ => {
                    stats.degenerate_cells += 1;
                    return None;
                }
            };
            let moon_rel_pos = arrival.position_inertial - planet_state.position_inertial;
            let moon_rel_vel = arrival.velocity_inertial - planet_state.velocity_inertial;
            let v_inf_planet = arc2.arrival_velocity_mps - planet_state.velocity_inertial;
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
    let total_dv = dep_mag + turn_price + arr_mag;
    if total_dv > config.max_broad_dv_mps {
        return None;
    }
    // Perigee impact screens on both arcs (central body). Flyby-body
    // impact is decided by the exact pass, not the broad arcs.
    if transfer_perigee_m(r1, arc1.departure_velocity_mps, ctx.central_mu)
        < ctx.central_radius_m * 1.05
        || transfer_perigee_m(rf, arc2.departure_velocity_mps, ctx.central_mu)
            < ctx.central_radius_m * 1.05
    {
        stats.impact_cells += 1;
        return None;
    }
    Some(FlybyCell {
        departure_epoch,
        tof_leg1_s,
        tof_leg2_s,
        flyby_body,
        departure_burn_mag_mps: dep_mag,
        v_out_flyby_frame_mps: v_out_f,
        total_dv,
    })
}

/// Single-flyby tour search. Returns validated plans ranked by exact total
/// Δv, best first. Deterministic: config body order, grid order,
/// total_cmp ordering.
pub fn flyby_search(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: FlybyConfig,
) -> Result<(Vec<RankedPlan>, SearchStats), SearchError> {
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
    let ctx = FlybyCtx {
        ephemeris,
        config: config.clone(),
        central_mu: central.mu,
        central_radius_m: central.radius_m,
        depot_mu: depot.mu,
        depot_radius_m: depot.radius_m,
        departure_planet: planet_of(ephemeris, config.central_body, config.departure_body),
        arrival_planet: planet_of(ephemeris, config.central_body, config.arrival_body),
    };
    let mut stats = SearchStats::default();
    // Broad: departure x leg1 x leg2 grid per flyby body.
    let mut best: Vec<FlybyCell> = Vec::new();
    for flyby_body in config.flyby_bodies.iter() {
        let flyby = ephemeris
            .body(*flyby_body)
            .map_err(SearchError::Ephemeris)?;
        if !flyby.mu.is_finite() || flyby.mu <= 0.0 {
            continue;
        }
        for i in 0..config.departure_steps {
            // Same endpoint-inclusive convention as the direct grid.
            let epoch = SimTime(if config.departure_steps == 1 {
                config.window_start.0
            } else {
                config.window_start.0
                    + config.departure_span_s * i as f64 / (config.departure_steps - 1) as f64
            });
            for j in 0..config.leg1_steps {
                let tof1 = lerp_range(config.leg1_min_s, config.leg1_max_s, config.leg1_steps, j);
                for k in 0..config.leg2_steps {
                    let tof2 =
                        lerp_range(config.leg2_min_s, config.leg2_max_s, config.leg2_steps, k);
                    stats.broad_evaluations += 1;
                    if let Some(cell) = flyby_cell(
                        &ctx,
                        *flyby_body,
                        flyby.mu,
                        flyby.radius_m,
                        epoch,
                        tof1,
                        tof2,
                        &mut stats,
                    ) {
                        best.push(cell);
                    }
                }
            }
        }
    }
    best.sort_by(|a, b| a.total_dv.total_cmp(&b.total_dv));
    best.truncate(config.keep_routes);
    if best.is_empty() {
        return Err(SearchError::NoViableTransfer { stats });
    }
    let mut ranked = Vec::new();
    for cell in &best {
        if let Some(plan) = revalidate_flyby(ephemeris, field, &ctx, cell, &mut stats)? {
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

fn revalidate_flyby(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    ctx: &FlybyCtx<'_>,
    cell: &FlybyCell,
    stats: &mut SearchStats,
) -> Result<Option<RankedPlan>, SearchError> {
    let config = &ctx.config;
    let flyby_epoch = SimTime(cell.departure_epoch.0 + cell.tof_leg1_s);
    let arrival_epoch = SimTime(flyby_epoch.0 + cell.tof_leg2_s);
    let arrival = ephemeris
        .body_state(config.arrival_body, arrival_epoch)
        .map_err(SearchError::Ephemeris)?;
    let target = ephemeris
        .body(config.arrival_body)
        .map_err(SearchError::Ephemeris)?;
    let depot = ephemeris
        .body_state(config.departure_body, cell.departure_epoch)
        .map_err(SearchError::Ephemeris)?;
    let central_dep = ephemeris
        .body_state(config.central_body, cell.departure_epoch)
        .map_err(SearchError::Ephemeris)?;
    let flyby_state = ephemeris
        .body_state(cell.flyby_body, flyby_epoch)
        .map_err(SearchError::Ephemeris)?;
    let flyby_body = ephemeris
        .body(cell.flyby_body)
        .map_err(SearchError::Ephemeris)?;
    let park_radius = ctx.depot_radius_m + config.standoff_m;
    // Leg 1: phase the departure anomaly against the flyby body, then
    // correct to the periapsis aim point (standoff above the surface —
    // never into the point-mass singularity).
    let (point, park_velocity, phased_burn) = match phase_departure(
        field,
        cell.departure_epoch,
        &depot,
        &central_dep,
        &flyby_state,
        ctx.depot_mu,
        park_radius,
        cell.departure_burn_mag_mps,
        cell.tof_leg1_s,
        stats,
    ) {
        Some(phased) => phased,
        None => return Ok(None),
    };
    let central_fly = ephemeris
        .body_state(config.central_body, flyby_epoch)
        .map_err(SearchError::Ephemeris)?;
    let aim1_dir = (flyby_state.position_inertial - central_fly.position_inertial).normalize();
    if !aim1_dir.is_finite() {
        stats.failed_revalidations += 1;
        return Ok(None);
    }
    let periapsis_m = flyby_body.radius_m + config.standoff_m;
    let aim1 = flyby_state.position_inertial + aim1_dir * periapsis_m;
    let mid1_s = midcourse_time_s(cell.tof_leg1_s);
    let (dep_burn, tcm1_burn, end1, miss1) = match correct_shooting(
        field,
        point,
        park_velocity,
        phased_burn,
        cell.departure_epoch,
        cell.tof_leg1_s,
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
    // Leg-1 gates: must actually reach the flyby, and must not lithobrake
    // on it (rendezvous with the flyby body at full speed is not a flyby).
    if !miss1.is_finite() || miss1 > config.max_miss_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    if (end1.position - flyby_state.position_inertial).length() < flyby_body.radius_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    // Broad flyby-burn seed: desired leg-2 asymptote against the CORRECTED
    // leg-1 arrival velocity. Energy-corrected for the well: the burn
    // happens at periapsis where the well adds v_esc, so the seed carries
    // the periapsis energy of the desired outgoing asymptote along its
    // direction. Escape-clean by construction (no capture-side excursions
    // for Newton to trip on); only the turn-geometry direction error is
    // left for the solver. Falls back to the naive vector difference when
    // the asymptote degenerates.
    let v_in_exact = end1.velocity - flyby_state.velocity_inertial;
    let naive_seed = cell.v_out_flyby_frame_mps - v_in_exact;
    let flyby_seed = match cell.v_out_flyby_frame_mps.try_normalize().map(|dir| {
        dir * (cell.v_out_flyby_frame_mps.length_squared() + 2.0 * flyby_body.mu / periapsis_m)
            .sqrt()
            - v_in_exact
    }) {
        Some(seed) if seed.is_finite() => seed,
        _ => naive_seed,
    };
    if !flyby_seed.is_finite() {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    // Leg 2: SOLVE the periapsis burn (not a downstream TCM) to hit the
    // arrival aim point. The broad seed is directionally sane; differential
    // correction varies the burn with full N-body dynamics inside every
    // evaluation, so well depth, gravitational focusing and leg-1 handoff
    // slop are absorbed INTO the solved burn. Mechanically this reuses
    // correct_shooting with the "midcourse" burn scheduled at time 0: a
    // burn at departure IS the departure burn, so departure stays fixed at
    // zero, the time-0 burn is the corrected flyby burn, and its node epoch
    // lands exactly on the flyby epoch.
    let central_arr = ephemeris
        .body_state(config.central_body, arrival_epoch)
        .map_err(SearchError::Ephemeris)?;
    let aim2_dir = (arrival.position_inertial - central_arr.position_inertial).normalize();
    if !aim2_dir.is_finite() {
        stats.failed_revalidations += 1;
        return Ok(None);
    }
    let aim2 = arrival.position_inertial + aim2_dir * (target.radius_m + config.standoff_m);
    let (_, flyby_burn, end2, miss2) = match correct_shooting(
        field,
        end1.position,
        end1.velocity,
        DVec3::ZERO,
        flyby_epoch,
        cell.tof_leg2_s,
        0.0,
        aim2,
        flyby_seed,
        stats,
    ) {
        Some(corrected) => corrected,
        None => {
            stats.failed_revalidations += 1;
            return Ok(None);
        }
    };
    let dist_center = (end2.position - arrival.position_inertial).length();
    if !miss2.is_finite() || miss2 > config.max_miss_m || dist_center < target.radius_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    let arrival_burn = arrival.velocity_inertial - end2.velocity;
    if !arrival_burn.is_finite() {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    // Four-node plan at most (departure, TCM1, solved flyby burn,
    // arrival match); sub-1 m/s trims omitted like in the direct search.
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
    if flyby_burn.length() >= 1.0 {
        nodes.push(
            ManeuverNode::new(flyby_epoch, flyby_burn)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
        );
    }
    nodes.push(
        ManeuverNode::new(arrival_epoch, arrival_burn)
            .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
    );
    let mut plan = ManeuverPlan::new(nodes, point, park_velocity + dep_burn, cell.departure_epoch)
        .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?;
    plan.predicted_miss_m = Some(miss2);
    plan = plan.with_flybys(vec![FlybyEvent {
        body: cell.flyby_body,
        epoch: flyby_epoch,
        periapsis_m,
        burn_mps: flyby_burn.length(),
    }]);
    Ok(Some(RankedPlan {
        exact_total_dv_mps: plan.total_dv_mps(),
        departure_epoch: cell.departure_epoch,
        time_of_flight_s: cell.tof_leg1_s + cell.tof_leg2_s,
        broad_total_dv_mps: cell.total_dv,
        plan,
        exact_miss_m: miss2,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{BakedBody, KeplerOrbit};

    #[test]
    fn bend_limit_matches_hyperbola() {
        // e = 1 + r_p v^2 / mu = 1 + 1e6*1e6/1e12 = 2 -> delta = 2*asin(1/2).
        let delta = max_bend_angle_rad(1_000.0, 1.0e12, 1.0e6);
        assert!((delta - std::f64::consts::PI / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn bend_limit_degenerate_inputs_are_zero() {
        assert_eq!(max_bend_angle_rad(0.0, 1.0e12, 1.0e6), 0.0);
        assert_eq!(max_bend_angle_rad(1_000.0, 0.0, 1.0e6), 0.0);
        assert_eq!(max_bend_angle_rad(1_000.0, 1.0e12, -1.0), 0.0);
        assert_eq!(max_bend_angle_rad(f64::NAN, 1.0e12, 1.0e6), 0.0);
    }

    #[test]
    fn powered_burn_is_vector_difference() {
        // Orthogonal 1 km/s turn costs sqrt(2) km/s impulsively.
        let burn = powered_flyby_burn_mps(DVec3::X * 1_000.0, DVec3::Y * 1_000.0);
        assert!((burn.length() - 1_000.0 * std::f64::consts::SQRT_2).abs() < 1.0e-9);
        // Parallel asymptotes: free bend.
        let free = powered_flyby_burn_mps(DVec3::X * 1_000.0, DVec3::X * 1_000.0);
        assert_eq!(free.length(), 0.0);
    }

    fn mini_system_with_flyby() -> BakedEphemeris {
        let orbit = |a: f64, m0: f64| {
            KeplerOrbit::new(1.0e14, a, 0.0, 0.0, 0.0, 0.0, m0).expect("valid test orbit")
        };
        BakedEphemeris::new(
            "TEST_MINI_FLYBY",
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
                    "swing",
                    5.0e11,
                    0.0,
                    BodyId(0),
                    orbit(1.25e7, 1.0),
                ),
                BakedBody::orbital(
                    BodyId(3),
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

    #[test]
    fn candidates_enumerate_swing_moons_only() {
        let ephemeris = mini_system_with_flyby();
        assert_eq!(
            candidate_flyby_bodies(&ephemeris, BodyId(0), BodyId(1), BodyId(3)),
            vec![BodyId(2)]
        );
    }

    fn mini_config() -> FlybyConfig {
        FlybyConfig {
            central_body: BodyId(0),
            departure_body: BodyId(1),
            arrival_body: BodyId(3),
            flyby_bodies: vec![BodyId(2)],
            window_start: SimTime(0.0),
            departure_span_s: 40_000.0,
            departure_steps: 6,
            leg1_min_s: 4_000.0,
            leg1_max_s: 10_000.0,
            leg1_steps: 3,
            leg2_min_s: 4_000.0,
            leg2_max_s: 10_000.0,
            leg2_steps: 3,
            // Route diversity: broad winners near chaotic encounters fail
            // exact shooting; ranking over several routes keeps clean ones.
            keep_routes: 3,
            standoff_m: 100_000.0,
            max_broad_dv_mps: 20_000.0,
            max_miss_m: 1.0e6,
        }
    }

    #[test]
    fn flyby_search_finds_validated_plan() {
        let ephemeris = mini_system_with_flyby();
        let field = GravityField::from_ephemeris(&ephemeris);
        let (ranked, stats) =
            flyby_search(&ephemeris, &field, mini_config()).expect("search finds");
        assert!(!ranked.is_empty());
        let winner = &ranked[0];
        assert!(winner.exact_miss_m <= 1.0e6);
        assert!(winner.exact_total_dv_mps.is_finite() && winner.exact_total_dv_mps > 0.0);
        assert!(winner.plan.predicted_miss_m.is_some());
        assert_eq!(winner.plan.flybys.len(), 1);
        assert_eq!(winner.plan.flybys[0].body, BodyId(2));
        assert!(stats.exact_revalidations >= 1);
        // Deterministic: same inputs, same winner.
        let (ranked2, _) = flyby_search(&ephemeris, &field, mini_config()).expect("search finds");
        assert_eq!(ranked, ranked2);
    }

    #[test]
    fn flyby_invalid_config_rejected() {
        let ephemeris = mini_system_with_flyby();
        let field = GravityField::from_ephemeris(&ephemeris);
        let mut bad = mini_config();
        bad.flyby_bodies.clear();
        assert_eq!(
            flyby_search(&ephemeris, &field, bad),
            Err(SearchError::InvalidConfig)
        );
    }
}
