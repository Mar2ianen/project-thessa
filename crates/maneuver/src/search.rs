//! Staged porkchop search: broad Lambert grid, local refinement, exact
//! N-body revalidation (docs/23 §12, 24 §7 pattern).
//!
//! Phase 1 (broad, loose): two-body Lambert arcs on osculating endpoint
//! states — analytic, thousands of cells, no force evaluations. Phase 2
//! (narrow): local grid refinement around the best cell. Phase 3 (final):
//! full N-body propagation of survivors with the departure burn, arrival
//! miss measured against the target ephemeris, arrival burn to match.
//! Pruning approximations never define physical truth: only revalidated
//! plans with measured miss execute.

use glam::DVec3;
use thessa_sim_core::{
    AdaptiveIntegratorConfig, BakedEphemeris, BodyId, GravityField, ImpulsiveBurn, SimTime,
    TestParticleState, propagate_adaptive_with_burns,
};

use crate::{
    ManeuverNode, ManeuverPlan,
    lambert::solve_lambert_prograde,
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

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SearchStats {
    pub broad_evaluations: usize,
    pub degenerate_cells: usize,
    pub impact_cells: usize,
    pub exact_revalidations: usize,
    pub failed_revalidations: usize,
    pub filtered_by_miss: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SearchError {
    InvalidConfig,
    Ephemeris(thessa_sim_core::EphemerisError),
    /// No transfer survived, with the counters showing why (all cells
    /// degenerate vs all survivors filtered by miss).
    NoViableTransfer { stats: SearchStats },
}

impl std::fmt::Display for SearchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfig => write!(formatter, "invalid search config"),
            Self::Ephemeris(error) => write!(formatter, "ephemeris error: {error}"),
            Self::NoViableTransfer { stats } => write!(
                formatter,
                "no viable transfer (broad {}, degenerate {}, impact {}, revalidated {}/{}, miss-filtered {})",
                stats.broad_evaluations,
                stats.degenerate_cells,
                stats.impact_cells,
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
    departure_burn: DVec3,
    total_dv: f64,
}

/// Shared broad-phase inputs: keeps grid signatures from bleeding params.
struct BroadCtx<'a> {
    ephemeris: &'a BakedEphemeris,
    config: SearchConfig,
    central_mu: f64,
    central_radius_m: f64,
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
    let central = ephemeris.body_state(config.central_body, departure_epoch).ok()?;
    let arrival_epoch = SimTime(departure_epoch.0 + time_of_flight_s);
    let departure = ephemeris.body_state(config.departure_body, departure_epoch).ok()?;
    let arrival = ephemeris.body_state(config.arrival_body, arrival_epoch).ok()?;
    let r1 = departure.position_inertial - central.position_inertial;
    let v1 = departure.velocity_inertial - central.velocity_inertial;
    let r2 = arrival.position_inertial - central.position_inertial;
    let v2 = arrival.velocity_inertial - central.velocity_inertial;
    // Prograde branch per cell: the cheap side is short-way on one side
    // of the sky and long-way on the other; a fixed flag would price half
    // the grid as retrograde.
    let arc = match solve_lambert_prograde(r1, v1, r2, time_of_flight_s, central_mu) {
        Ok(arc) => arc,
        Err(_) => {
            stats.degenerate_cells += 1;
            return None;
        }
    };
    let dep_burn = arc.departure_velocity_mps - v1;
    let arr_burn = v2 - arc.arrival_velocity_mps;
    if !dep_burn.is_finite() || !arr_burn.is_finite() {
        stats.degenerate_cells += 1;
        return None;
    }
    let total_dv = dep_burn.length() + arr_burn.length();
    if total_dv > config.max_broad_dv_mps {
        return None;
    }
    // Perigee impact screen on the departure arc: arcs through the central
    // body are grid noise (the exact propagator would just hit singularity).
    if transfer_perigee_m(r1, arc.departure_velocity_mps, central_mu)
        < central_radius_m * 1.05
    {
        stats.impact_cells += 1;
        return None;
    }
    Some(Cell {
        departure_epoch,
        time_of_flight_s,
        departure_burn: dep_burn,
        total_dv,
    })
}

/// Transfer-ellipse perigee from one state vector (also correct for
/// hyperbolic energy via the same `a(1-e)` form).
fn transfer_perigee_m(position: DVec3, velocity: DVec3, mu: f64) -> f64 {
    let radius = position.length();
    if radius <= 0.0 {
        return 0.0;
    }
    let energy = velocity.length_squared() / 2.0 - mu / radius;
    if energy >= 0.0 {
        // Unbound: still report the pericenter radius honestly.
        let momentum = position.cross(velocity);
        let semi_latus = momentum.length_squared() / mu;
        let ecc_vector =
            (position * (velocity.length_squared() - mu / radius) - velocity * position.dot(velocity))
                / mu;
        let eccentricity = ecc_vector.length();
        return semi_latus / (1.0 + eccentricity);
    }
    let semi_major = -mu / (2.0 * energy);
    let momentum = position.cross(velocity);
    let ecc_vector =
        (position * (velocity.length_squared() - mu / radius) - velocity * position.dot(velocity))
            / mu;
    semi_major * (1.0 - ecc_vector.length())
}

fn grid_best(
    ctx: &BroadCtx<'_>,
    start_s: f64,
    span_s: f64,
    steps: usize,
    tof_min: f64,
    tof_max: f64,
    tof_steps: usize,
    stats: &mut SearchStats,
    best: &mut Vec<Cell>,
) {
    for i in 0..steps {
        let epoch = SimTime(if steps == 1 {
            start_s
        } else {
            start_s + span_s * i as f64 / (steps - 1) as f64
        });
        for j in 0..tof_steps {
            let tof = if tof_steps == 1 {
                tof_min
            } else {
                tof_min + (tof_max - tof_min) * j as f64 / (tof_steps - 1) as f64
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
    };
    let mut stats = SearchStats::default();
    // Phase 1: broad grid.
    let mut best = Vec::new();
    grid_best(
        &ctx,
        config.window_start.0,
        config.departure_span_s,
        config.departure_steps,
        config.tof_min_s,
        config.tof_max_s,
        config.tof_steps,
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
            focus.departure_epoch.0 - span,
            span * 2.0,
            5,
            (focus.time_of_flight_s - tof_span).max(config.tof_min_s),
            (focus.time_of_flight_s + tof_span).min(config.tof_max_s),
            5,
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
    // Phase 3: exact N-body revalidation of survivors.
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

fn revalidate(
    ephemeris: &BakedEphemeris,
    field: &GravityField<'_>,
    config: SearchConfig,
    cell: &Cell,
    stats: &mut SearchStats,
) -> Result<Option<RankedPlan>, SearchError> {
    let departure = ephemeris
        .body_state(config.departure_body, cell.departure_epoch)
        .map_err(SearchError::Ephemeris)?;
    let arrival_epoch = SimTime(cell.departure_epoch.0 + cell.time_of_flight_s);
    let arrival = ephemeris
        .body_state(config.arrival_body, arrival_epoch)
        .map_err(SearchError::Ephemeris)?;
    let burns = [ImpulsiveBurn {
        time_s: 0.0,
        delta_v_mps: cell.departure_burn,
    }];
    let result = match propagate_adaptive_with_burns(
        field,
        TestParticleState {
            position: departure.position_inertial,
            velocity: departure.velocity_inertial,
        },
        cell.departure_epoch,
        cell.time_of_flight_s,
        &burns,
        AdaptiveIntegratorConfig::default(),
    ) {
        Ok(result) => result,
        // A survivor that hits a singularity (or otherwise fails exact
        // propagation) is a rejected candidate, not a search failure:
        // other survivors may still validate.
        Err(_) => {
            stats.failed_revalidations += 1;
            return Ok(None);
        }
    };
    stats.exact_revalidations += 1;
    let miss = (result.state.position - arrival.position_inertial).length();
    if !miss.is_finite() || miss > config.max_miss_m {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    let arrival_burn = arrival.velocity_inertial - result.state.velocity;
    if !arrival_burn.is_finite() {
        stats.filtered_by_miss += 1;
        return Ok(None);
    }
    let mut plan = ManeuverPlan::new(
        vec![
            ManeuverNode::new(cell.departure_epoch, cell.departure_burn)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
            ManeuverNode::new(arrival_epoch, arrival_burn)
                .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?,
        ],
        departure.position_inertial,
        departure.velocity_inertial,
        cell.departure_epoch,
    )
    .map_err(|_| SearchError::NoViableTransfer { stats: *stats })?;
    plan.predicted_miss_m = Some(miss);
    Ok(Some(RankedPlan {
        exact_total_dv_mps: cell.departure_burn.length() + arrival_burn.length(),
        departure_epoch: cell.departure_epoch,
        time_of_flight_s: cell.time_of_flight_s,
        broad_total_dv_mps: cell.total_dv,
        plan,
        exact_miss_m: miss,
    }))
}

#[cfg(test)]
mod probe_tests {
    use super::*;
    use thessa_sim_core::SystemConfig;
#[test]
fn probe_single_cell() {
    use crate::lambert::{solve_lambert, solve_lambert_prograde};
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let t_dep = SimTime::EPOCH;
    let tof = 72.0 * 3600.0;
    for name in ["nereid", "pelagos", "thessa"] {
        let id = ephemeris.body_id(name).unwrap();
        let body = ephemeris.body(id).unwrap();
        let state = ephemeris.body_state(id, t_dep).unwrap();
        eprintln!("{name}: mu={:e} r={} pos={:?} vel={:?}", body.mu, body.radius_m, state.position_inertial, state.velocity_inertial);
    }
    let central = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t_dep).unwrap();
    let dep = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t_dep).unwrap();
    let arr = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(tof)).unwrap();
    let r1 = dep.position_inertial - central.position_inertial;
    let v1 = dep.velocity_inertial - central.velocity_inertial;
    let r2 = arr.position_inertial - central.position_inertial;
    let v2 = arr.velocity_inertial - central.velocity_inertial;
    eprintln!("r1={r1:?} v1={v1:?}\nr2={r2:?} v2={v2:?}");
    match solve_lambert(r1, r2, tof, ephemeris.body(ephemeris.body_id("nereid").unwrap()).unwrap().mu, true) {
        Ok(arc) => eprintln!("arc dep={:?} arr={:?}", arc.departure_velocity_mps, arc.arrival_velocity_mps),
        Err(e) => eprintln!("lambert err: {e:?}"),
    }
    // Mini grid diagnostic: distribution of broad totals.
    let mu_c = ephemeris.body(ephemeris.body_id("nereid").unwrap()).unwrap().mu;
    let mut totals: Vec<f64> = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            let t = SimTime(i as f64 * 86_400.0);
            let dt = 20.0 * 3_600.0 + j as f64 * 10.0 * 3_600.0;
            let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t).unwrap();
            let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t).unwrap();
            let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t.0 + dt)).unwrap();
            let q1 = d.position_inertial - c.position_inertial;
            let w1 = d.velocity_inertial - c.velocity_inertial;
            let q2 = a.position_inertial - c.position_inertial;
            let w2 = a.velocity_inertial - c.velocity_inertial;
            if let Ok(arc) = solve_lambert(q1, q2, dt, mu_c, true) {
                totals.push((arc.departure_velocity_mps - w1).length() + (w2 - arc.arrival_velocity_mps).length());
            }
        }
    }
    totals.sort_by(f64::total_cmp);
    eprintln!("grid totals: n={} best5={:?}", totals.len(), &totals[..totals.len().min(5)]);
    // Same grid, prograde branch selection.
    let mut pro: Vec<f64> = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            let t = SimTime(i as f64 * 86_400.0);
            let dt = 20.0 * 3_600.0 + j as f64 * 10.0 * 3_600.0;
            let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t).unwrap();
            let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t).unwrap();
            let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t.0 + dt)).unwrap();
            let q1 = d.position_inertial - c.position_inertial;
            let w1 = d.velocity_inertial - c.velocity_inertial;
            let q2 = a.position_inertial - c.position_inertial;
            let w2 = a.velocity_inertial - c.velocity_inertial;
            if let Ok(arc) = solve_lambert_prograde(q1, w1, q2, dt, mu_c) {
                pro.push((arc.departure_velocity_mps - w1).length() + (w2 - arc.arrival_velocity_mps).length());
            }
        }
    }
    pro.sort_by(f64::total_cmp);
    eprintln!("pro grid: n={} best5={:?}", pro.len(), &pro[..pro.len().min(5)]);
    // Thessa orbit audit: parent, elements, Nereid-relative distance over time.
    {
        let th = ephemeris.body_id("thessa").unwrap();
        let body = ephemeris.body(th).unwrap();
        eprintln!("thessa parent={:?} mu={:e} orbit={:?}", body.parent, body.mu, body.orbit.map(|o| (o.semi_major_axis_m, o.eccentricity)));
        let ne = ephemeris.body_id("nereid").unwrap();
        for h in [0.0, 20.0, 40.0, 60.0, 80.0] {
            let t = SimTime(h * 3_600.0);
            let a = ephemeris.body_state(th, t).unwrap();
            let c = ephemeris.body_state(ne, t).unwrap();
            eprintln!(
                "t={h}h thessa-nereid dist={:.3e} relvel={:.0}",
                (a.position_inertial - c.position_inertial).length(),
                (a.velocity_inertial - c.velocity_inertial).length(),
            );
        }
    }
    // Best-cell anatomy: full geometry dump.
    {
        let mut cells: Vec<(f64, f64, f64)> = Vec::new();
        for i in 0..25 {
            for j in 0..25 {
                let t = SimTime(i as f64 * 10.0 * 3_600.0);
                let dt = 20.0 * 3_600.0 + j as f64 * (100.0 * 3_600.0 / 24.0);
                let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t).unwrap();
                let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t).unwrap();
                let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t.0 + dt)).unwrap();
                let q1 = d.position_inertial - c.position_inertial;
                let w1 = d.velocity_inertial - c.velocity_inertial;
                let q2 = a.position_inertial - c.position_inertial;
                let w2 = a.velocity_inertial - c.velocity_inertial;
                if let Ok(arc) = solve_lambert_prograde(q1, w1, q2, dt, mu_c) {
                    let total = (arc.departure_velocity_mps - w1).length() + (w2 - arc.arrival_velocity_mps).length();
                    cells.push((total, t.0, dt));
                }
            }
        }
        cells.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (total, t, dt) in cells.iter().take(3) {
            let tt = SimTime(*t);
            let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), tt).unwrap();
            let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), tt).unwrap();
            let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t + dt)).unwrap();
            let q1 = d.position_inertial - c.position_inertial;
            let q2 = a.position_inertial - c.position_inertial;
            let cosang = (q1.dot(q2) / (q1.length() * q2.length())).clamp(-1.0, 1.0);
            eprintln!("cell t={:.1}h tof={:.1}h total={:.0} angle={:.1}deg r1={:.3e} r2={:.3e}",
                t / 3600.0, dt / 3600.0, total, cosang.acos().to_degrees(), q1.length(), q2.length());
        }
    }
    // Shoot-the-arc audit: propagate the solved departure velocity under
    // pure two-body gravity and measure the actual arrival miss. If the
    // solver is right, the miss is ~integrator tolerance; if the geometry
    // is just expensive, the miss is small AND the dv huge.
    {
        use thessa_sim_core::{
            AdaptiveIntegratorConfig, BakedBody, BakedEphemeris, BodyId, GravityField,
            TestParticleState, propagate_adaptive,
        };
        let t = SimTime(60.0 * 3_600.0);
        let dt = 58.0 * 3_600.0;
        let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t).unwrap();
        let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t).unwrap();
        let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t.0 + dt)).unwrap();
        let q1 = d.position_inertial - c.position_inertial;
        let w1 = d.velocity_inertial - c.velocity_inertial;
        let q2 = a.position_inertial - c.position_inertial;
        let solo = BakedEphemeris::new(
            "PROBE",
            vec![BakedBody::fixed(BodyId(0), "center", mu_c, 0.0)],
        )
        .unwrap();
        let field = GravityField::from_ephemeris(&solo);
        // NOTE: fixed center at origin, but q1 is Nereid-centered while
        // Nereid itself sits at ~4.8e12 m: shift r1/r2/q-frame consistently
        // by solving in Nereid-centered coords (origin = Nereid center).
        match solve_lambert_prograde(q1, w1, q2, dt, mu_c) {
            Ok(arc) => {
                let flown = propagate_adaptive(
                    &field,
                    TestParticleState {
                        position: q1,
                        velocity: arc.departure_velocity_mps,
                    },
                    SimTime(0.0),
                    dt,
                    AdaptiveIntegratorConfig::default(),
                )
                .unwrap();
                eprintln!(
                    "shoot: miss={:.1} km depdv={:.0}",
                    (flown.state.position - q2).length() / 1000.0,
                    (arc.departure_velocity_mps - w1).length(),
                );
            }
            Err(e) => eprintln!("shoot: {e:?}"),
        }
    }
    for (t_dep_h, tof_h) in [(60.0, 58.0), (140.0, 58.0), (60.0, 40.0)] {
        let t = SimTime(t_dep_h * 3_600.0);
        let dt = tof_h * 3_600.0;
        let c = ephemeris.body_state(ephemeris.body_id("nereid").unwrap(), t).unwrap();
        let d = ephemeris.body_state(ephemeris.body_id("pelagos").unwrap(), t).unwrap();
        let a = ephemeris.body_state(ephemeris.body_id("thessa").unwrap(), SimTime(t.0 + dt)).unwrap();
        let q1 = d.position_inertial - c.position_inertial;
        let w1 = d.velocity_inertial - c.velocity_inertial;
        let q2 = a.position_inertial - c.position_inertial;
        let w2 = a.velocity_inertial - c.velocity_inertial;
        match solve_lambert_prograde(q1, w1, q2, dt, mu_c) {
            Ok(arc) => eprintln!(
                "hand t={t_dep_h}h tof={tof_h}h: depdv={:.0} arrdv={:.0} total={:.0}",
                (arc.departure_velocity_mps - w1).length(),
                (w2 - arc.arrival_velocity_mps).length(),
                (arc.departure_velocity_mps - w1).length()
                    + (w2 - arc.arrival_velocity_mps).length(),
            ),
            Err(e) => eprintln!("hand t={t_dep_h}h tof={tof_h}h: {e:?}"),
        }
    }
}

}
