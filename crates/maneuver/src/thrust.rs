//! Finite-burn planning: low-thrust engines, burn segmentation, and
//! multi-day thrust arcs.
//!
//! Formal I/O:
//! - IN: an impulsive [`ManeuverPlan`], an [`EngineSpec`], spacecraft mass,
//!   a segment cap, and a [`SplitMode`]; or a hand-built
//!   [`FiniteBurnPlan`].
//! - OUT: a [`FiniteBurnPlan`] (time-ordered [`BurnSegment`]s with engine,
//!   mass profile, and validation slots) plus, after
//!   [`validate_finite_burn`], the measured finite-vs-impulsive divergence,
//!   realized shortfall, and propellant bill.
//!
//! Model: burns are rocket-equation exact (durations and propellant from
//! Tsiolkovsky, mass tracked through conversion), directions are inertial
//! or velocity-aligned (prograde/retrograde steering is evaluated inside
//! the exact propagation, not frozen at planning time), and validation
//! propagates the arcs with [`propagate_adaptive_with_thrust`] — mass in
//! closed form, no extra ODE state. Converted plans carry
//! `predicted_miss_m: None` until validated: the type boundary is the
//! execution gate, same as broad survey routes.
//!
//! [`ManeuverPlan`]: crate::ManeuverPlan
//! [`propagate_adaptive_with_thrust`]: thessa_sim_core::propagate_adaptive_with_thrust

use glam::DVec3;
use serde::{Deserialize, Serialize};
use thessa_sim_core::{
    AdaptiveIntegratorConfig, BakedEphemeris, BodyId, GravityField, ImpulsiveBurn, SimTime,
    TestParticleState, ThrustArc, ThrustDirection, propagate_adaptive,
    propagate_adaptive_with_burns, propagate_adaptive_with_thrust,
};

use crate::{
    ManeuverPlan,
    plan::{ManeuverNode, PlanValidation},
};

/// Engine ratings (SI): full-throttle thrust and exhaust velocity
/// (`ve = isp * g0`; stored directly so the planner never depends on g0).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineSpec {
    pub thrust_n: f64,
    pub exhaust_velocity_mps: f64,
}

impl EngineSpec {
    pub fn validate(&self) -> Result<(), ThrustPlanError> {
        if !self.thrust_n.is_finite() || self.thrust_n <= 0.0 {
            return Err(ThrustPlanError::InvalidEngine);
        }
        if !self.exhaust_velocity_mps.is_finite() || self.exhaust_velocity_mps <= 0.0 {
            return Err(ThrustPlanError::InvalidEngine);
        }
        Ok(())
    }

    /// Full-throttle mass flow (kg/s).
    pub fn mass_flow_kgs(&self) -> f64 {
        self.thrust_n / self.exhaust_velocity_mps
    }

    /// Burn seconds for `delta_v_mps` starting at `mass_kg` (Tsiolkovsky,
    /// exact): `t = m*ve/F * (1 - exp(-dv/ve))`.
    pub fn burn_duration_s(&self, delta_v_mps: f64, mass_kg: f64) -> Option<f64> {
        if !delta_v_mps.is_finite() || delta_v_mps < 0.0 {
            return None;
        }
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return None;
        }
        let duration = mass_kg * self.exhaust_velocity_mps / self.thrust_n
            * (1.0 - (-delta_v_mps / self.exhaust_velocity_mps).exp());
        if !duration.is_finite() || duration < 0.0 {
            return None;
        }
        Some(duration)
    }

    /// Propellant (kg) for `delta_v_mps` at `mass_kg`: `m*(1-exp(-dv/ve))`.
    pub fn propellant_kg(&self, delta_v_mps: f64, mass_kg: f64) -> Option<f64> {
        if !delta_v_mps.is_finite() || delta_v_mps < 0.0 {
            return None;
        }
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return None;
        }
        let propellant = mass_kg * (1.0 - (-delta_v_mps / self.exhaust_velocity_mps).exp());
        if !propellant.is_finite() || propellant < 0.0 || propellant >= mass_kg {
            return None;
        }
        Some(propellant)
    }

    /// Δv capacity (m/s) from full to dry mass: `ve*ln(m0/m_dry)`.
    pub fn delta_v_capacity_mps(&self, mass_full_kg: f64, mass_dry_kg: f64) -> Option<f64> {
        if !mass_full_kg.is_finite() || !mass_dry_kg.is_finite() {
            return None;
        }
        if mass_full_kg <= mass_dry_kg || mass_dry_kg <= 0.0 {
            return None;
        }
        let capacity = self.exhaust_velocity_mps * (mass_full_kg / mass_dry_kg).ln();
        if !capacity.is_finite() || capacity < 0.0 {
            return None;
        }
        Some(capacity)
    }
}

/// Thrust direction of one burn segment.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SegmentDirection {
    /// Fixed unit vector, inertial frame.
    Inertial(DVec3),
    /// Along the instantaneous inertial velocity (evaluated inside the
    /// exact propagation and by the executor at poll time).
    Prograde,
    /// Against the instantaneous inertial velocity.
    Retrograde,
    /// LVLH components (radial outward, in-track, orbit-normal) in the
    /// spacecraft-centered RTN frame around `central` — same basis as the
    /// propagation RHS (single definition, frames agree exactly).
    Rtn {
        central: BodyId,
        radial: f64,
        transverse: f64,
        normal: f64,
    },
}

/// One finite burn: throttle schedule over `[start, start + duration)`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BurnSegment {
    pub start: SimTime,
    pub duration_s: f64,
    /// Planned Δv (m/s) this segment realizes (exact for converted plans:
    /// node Δv or its equal share; an estimate for hand-built plans —
    /// feeds the executor shortfall telemetry).
    pub planned_dv_mps: f64,
    pub direction: SegmentDirection,
    pub throttle_01: f64,
}

impl BurnSegment {
    pub fn end(&self) -> SimTime {
        SimTime(self.start.0 + self.duration_s)
    }
}

/// Time-ordered burn schedule with engine and mass profile: the finite-burn
/// analog of [`ManeuverPlan`]. `predicted_miss_m` is the validated
/// finite-vs-impulsive divergence (`None` = unvalidated, do not fly).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FiniteBurnPlan {
    pub segments: Vec<BurnSegment>,
    pub engine: EngineSpec,
    pub initial_mass_kg: f64,
    pub departure_position_m: DVec3,
    pub departure_velocity_mps: DVec3,
    pub departure_epoch: SimTime,
    pub predicted_miss_m: Option<f64>,
    pub final_mass_kg: Option<f64>,
}

impl FiniteBurnPlan {
    pub fn new(
        segments: Vec<BurnSegment>,
        engine: EngineSpec,
        initial_mass_kg: f64,
        departure_position_m: DVec3,
        departure_velocity_mps: DVec3,
        departure_epoch: SimTime,
    ) -> Result<Self, ThrustPlanError> {
        engine.validate()?;
        if !initial_mass_kg.is_finite() || initial_mass_kg <= 0.0 {
            return Err(ThrustPlanError::InvalidMass);
        }
        if !departure_position_m.is_finite()
            || !departure_velocity_mps.is_finite()
            || !departure_epoch.0.is_finite()
        {
            return Err(ThrustPlanError::InvalidSegment);
        }
        let mut previous_end = f64::NEG_INFINITY;
        for segment in &segments {
            if !segment.start.0.is_finite()
                || !segment.duration_s.is_finite()
                || segment.duration_s < 0.0
                || !segment.planned_dv_mps.is_finite()
                || segment.planned_dv_mps < 0.0
                || !segment.throttle_01.is_finite()
                || segment.throttle_01 < 0.0
                || segment.throttle_01 > 1.0
            {
                return Err(ThrustPlanError::InvalidSegment);
            }
            if let SegmentDirection::Inertial(direction) = segment.direction
                && (!direction.is_finite() || direction.length_squared() <= 0.0)
            {
                return Err(ThrustPlanError::InvalidSegment);
            }
            if let SegmentDirection::Rtn {
                radial,
                transverse,
                normal,
                ..
            } = segment.direction
            {
                if !radial.is_finite() || !transverse.is_finite() || !normal.is_finite() {
                    return Err(ThrustPlanError::InvalidSegment);
                }
                if radial == 0.0 && transverse == 0.0 && normal == 0.0 {
                    return Err(ThrustPlanError::InvalidSegment);
                }
            }
            if segment.start.0 < previous_end {
                return Err(ThrustPlanError::OverlappingSegments);
            }
            previous_end = segment.start.0 + segment.duration_s;
        }
        Ok(Self {
            segments,
            engine,
            initial_mass_kg,
            departure_position_m,
            departure_velocity_mps,
            departure_epoch,
            predicted_miss_m: None,
            final_mass_kg: None,
        })
    }

    /// Total planned Δv (m/s) over all segments.
    pub fn total_planned_dv_mps(&self) -> f64 {
        self.segments
            .iter()
            .map(|segment| segment.planned_dv_mps)
            .sum()
    }

    /// Total burn time (s) over all segments.
    pub fn total_burn_s(&self) -> f64 {
        self.segments.iter().map(|segment| segment.duration_s).sum()
    }

    /// Attach validation results (divergence becomes the execution gate).
    pub fn with_validation(mut self, validation: &BurnValidation) -> Self {
        self.predicted_miss_m = Some(validation.divergence_m);
        self.final_mass_kg = Some(validation.final_mass_kg);
        self
    }

    /// Validation outcome for server admission: either executable or a
    /// named reason the block must take its abort path. Mirrors the node
    /// plan gate (same enum, same stale semantics against the first
    /// segment start).
    pub fn validate_for_execution(&self, now: SimTime) -> PlanValidation {
        if self.segments.is_empty() {
            return PlanValidation::Empty;
        }
        match self.segments.first() {
            Some(segment) if segment.start.0 < now.0 => PlanValidation::Stale {
                now_s: now.0,
                first_node_s: segment.start.0,
            },
            Some(_) => PlanValidation::Executable,
            None => PlanValidation::Empty,
        }
    }
}

/// How a node burn longer than the segment cap is split.
#[derive(Debug, Clone, PartialEq)]
pub enum SplitMode {
    /// Equal-Δv burns at the same orbital phase on successive orbits
    /// (efficient phasing). The caller owns frames: pass the osculating
    /// period around the relevant central body (see [`orbit_period`]) —
    /// the plan state itself is system-frame and must NOT be differenced
    /// against a bare μ here. One period for every node (departure-period
    /// phasing — fine when one orbit dominates the plan).
    PerOrbit { orbit_period_s: f64 },
    /// Same phasing with an exact period per nonzero node in plan order
    /// (see [`node_osculating_periods`]): later nodes on changed orbits
    /// phase on their own period, not the departure one. Length must
    /// match the nonzero-node count.
    PerOrbitNodes { periods_s: Vec<f64> },
    /// Contiguous chop with `cooldown_s` coasts between pieces
    /// (thermal/duty constraint); works on any trajectory.
    Contiguous { cooldown_s: f64 },
}

/// Realize every impulsive node as finite-burn segments: durations and
/// propellant from the rocket equation with mass tracked across nodes
/// (later burns cost more time per m/s — exactly, not approximately).
/// Short burns center on their node epoch; long burns split per
/// [`SplitMode`]. Zero-magnitude nodes produce no segments. Returns an
/// UNVALIDATED plan (`predicted_miss_m: None`); run [`validate_finite_burn`].
pub fn realize_impulsive(
    plan: &ManeuverPlan,
    engine: &EngineSpec,
    initial_mass_kg: f64,
    max_segment_s: f64,
    split: SplitMode,
) -> Result<FiniteBurnPlan, ThrustPlanError> {
    engine.validate()?;
    if !initial_mass_kg.is_finite() || initial_mass_kg <= 0.0 {
        return Err(ThrustPlanError::InvalidMass);
    }
    if !max_segment_s.is_finite() || max_segment_s <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    // Spacing source: one caller period for every node, an exact period
    // per nonzero node, or contiguous chop (frames are the caller's job).
    let nonzero_nodes = plan
        .nodes
        .iter()
        .filter(|node| node.magnitude_mps() > 0.0)
        .count();
    enum Spacing {
        Periods(std::vec::IntoIter<f64>),
        Chop(f64),
    }
    let mut spacing = match &split {
        SplitMode::PerOrbit { orbit_period_s } => {
            if !orbit_period_s.is_finite() || *orbit_period_s <= 0.0 {
                return Err(ThrustPlanError::InvalidSegment);
            }
            Spacing::Periods(vec![*orbit_period_s; nonzero_nodes].into_iter())
        }
        SplitMode::PerOrbitNodes { periods_s } => {
            if periods_s.len() != nonzero_nodes
                || periods_s
                    .iter()
                    .any(|period| !period.is_finite() || *period <= 0.0)
            {
                return Err(ThrustPlanError::InvalidSegment);
            }
            Spacing::Periods(periods_s.clone().into_iter())
        }
        SplitMode::Contiguous { cooldown_s } => {
            if !cooldown_s.is_finite() || *cooldown_s < 0.0 {
                return Err(ThrustPlanError::InvalidSegment);
            }
            Spacing::Chop(*cooldown_s)
        }
    };
    let mut mass_kg = initial_mass_kg;
    let mut segments = Vec::new();
    for node in &plan.nodes {
        let node_dv = node.magnitude_mps();
        if node_dv == 0.0 {
            continue;
        }
        let direction = node_direction(node)?;
        let total_duration = engine
            .burn_duration_s(node_dv, mass_kg)
            .ok_or(ThrustPlanError::PropellantExceeded)?;
        if total_duration <= max_segment_s {
            segments.push(centered_segment(
                node.epoch,
                node_dv,
                total_duration,
                direction,
                plan.departure_epoch,
            ));
            mass_kg -= engine
                .propellant_kg(node_dv, mass_kg)
                .ok_or(ThrustPlanError::PropellantExceeded)?;
            continue;
        }
        // Split into K equal-Δv pieces. Count from the entry mass, then
        // bumped until every piece fits the cap (later pieces run
        // shorter as mass drops, so the entry estimate usually fits
        // first try — the loop is pure defense).
        let pieces = split_count(engine, node_dv, mass_kg, max_segment_s)?;
        let piece_dv = node_dv / pieces as f64;
        // Per-piece durations chained from the entry mass (the same
        // arithmetic as the fit check above, so scheduling matches mass).
        let mut durations = Vec::with_capacity(pieces);
        let mut chain = mass_kg;
        for _ in 0..pieces {
            let duration = engine
                .burn_duration_s(piece_dv, chain)
                .ok_or(ThrustPlanError::PropellantExceeded)?;
            durations.push(duration);
            chain -= engine
                .propellant_kg(piece_dv, chain)
                .ok_or(ThrustPlanError::PropellantExceeded)?;
        }
        match &mut spacing {
            Spacing::Periods(periods) => {
                // Symmetric phasing around the node epoch on this node's
                // own period (departure-period for the single-period mode).
                // Pieces longer than one orbit (or nodes closer than their
                // spans) fail honestly at plan construction with Overlap.
                let period = periods.next().ok_or(ThrustPlanError::InvalidSegment)?;
                for (k, duration) in durations.iter().enumerate() {
                    let center = node.epoch.0 + (k as f64 - (pieces as f64 - 1.0) / 2.0) * period;
                    segments.push(BurnSegment {
                        start: SimTime((center - duration / 2.0).max(plan.departure_epoch.0)),
                        duration_s: *duration,
                        planned_dv_mps: piece_dv,
                        direction,
                        throttle_01: 1.0,
                    });
                }
            }
            Spacing::Chop(cooldown_s) => {
                // Contiguous chop from a centered first piece (works on any
                // trajectory, including hyperbolic escapes).
                let cooldown_s = *cooldown_s;
                let mut start = (node.epoch.0 - durations[0] / 2.0).max(plan.departure_epoch.0);
                for duration in &durations {
                    segments.push(BurnSegment {
                        start: SimTime(start),
                        duration_s: *duration,
                        planned_dv_mps: piece_dv,
                        direction,
                        throttle_01: 1.0,
                    });
                    start += duration + cooldown_s;
                }
            }
        }
        mass_kg = chain;
    }
    let mut realized = FiniteBurnPlan::new(
        segments,
        *engine,
        initial_mass_kg,
        plan.departure_position_m,
        plan.departure_velocity_mps,
        plan.departure_epoch,
    )?;
    realized.final_mass_kg = Some(mass_kg);
    Ok(realized)
}

/// Osculating period (s) of a bound orbit from central-relative state;
/// hyperbolic/parabolic rejected. Frame helper for [`SplitMode::PerOrbit`]
/// callers (pass moon/planet-relative state, never system-frame).
pub fn orbit_period(
    position_rel_central_m: DVec3,
    velocity_rel_central_mps: DVec3,
    central_mu: f64,
) -> Result<f64, ThrustPlanError> {
    if !central_mu.is_finite() || central_mu <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    let radius = position_rel_central_m.length();
    let speed_squared = velocity_rel_central_mps.length_squared();
    if !radius.is_finite() || radius <= 0.0 || !speed_squared.is_finite() {
        return Err(ThrustPlanError::InvalidSegment);
    }
    let semi_major = 1.0 / (2.0 / radius - speed_squared / central_mu);
    if !semi_major.is_finite() || semi_major <= 0.0 {
        return Err(ThrustPlanError::RequiresBoundOrbit);
    }
    let period = std::f64::consts::TAU * (semi_major.powi(3) / central_mu).sqrt();
    if !period.is_finite() || period <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    Ok(period)
}

fn node_direction(node: &ManeuverNode) -> Result<SegmentDirection, ThrustPlanError> {
    let magnitude = node.magnitude_mps();
    if !magnitude.is_finite() || magnitude <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    Ok(SegmentDirection::Inertial(node.delta_v_mps / magnitude))
}

fn centered_segment(
    epoch: SimTime,
    planned_dv_mps: f64,
    duration_s: f64,
    direction: SegmentDirection,
    departure_epoch: SimTime,
) -> BurnSegment {
    BurnSegment {
        start: SimTime((epoch.0 - duration_s / 2.0).max(departure_epoch.0)),
        duration_s,
        planned_dv_mps,
        direction,
        throttle_01: 1.0,
    }
}

/// Exact osculating period per nonzero node in plan order: propagate
/// impulsively node to node (cumulative burns, exact N-body states),
/// relativize to `central` at each node epoch, and take the period of the
/// POST-burn (outbound) orbit — the orbit the later pieces actually fly.
/// Hyperbolic nodes rejected (cannot phase). Pair with
/// [`SplitMode::PerOrbitNodes`].
pub fn node_osculating_periods(
    field: &GravityField<'_>,
    ephemeris: &BakedEphemeris,
    central: BodyId,
    plan: &ManeuverPlan,
) -> Result<Vec<f64>, ThrustPlanError> {
    let central_mu = ephemeris
        .body(central)
        .map_err(|error| ThrustPlanError::Ephemeris(error.to_string()))?
        .mu;
    if !central_mu.is_finite() || central_mu <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    let config = AdaptiveIntegratorConfig::default();
    let mut state = TestParticleState {
        position: plan.departure_position_m,
        velocity: plan.departure_velocity_mps,
    };
    let mut now = plan.departure_epoch;
    let mut periods = Vec::new();
    for node in &plan.nodes {
        let horizon = (node.epoch.0 - now.0).max(0.0);
        if horizon > 0.0 {
            state = propagate_adaptive(field, state, now, horizon, config)
                .map_err(ThrustPlanError::Propagation)?
                .state;
            now = node.epoch;
        }
        if node.magnitude_mps() == 0.0 {
            continue;
        }
        state.velocity += node.delta_v_mps;
        let center = ephemeris
            .body_state(central, node.epoch)
            .map_err(|error| ThrustPlanError::Ephemeris(error.to_string()))?;
        periods.push(orbit_period(
            state.position - center.position_inertial,
            state.velocity - center.velocity_inertial,
            central_mu,
        )?);
    }
    Ok(periods)
}

/// Piece count for an equal-Δv split: from the entry-mass estimate, then
/// bumped until every piece fits the cap (later pieces run shorter as
/// mass drops, so this converges immediately in practice).
/// Sanity-capped against runaway.
fn split_count(
    engine: &EngineSpec,
    node_dv_mps: f64,
    entry_mass_kg: f64,
    max_segment_s: f64,
) -> Result<usize, ThrustPlanError> {
    let first_estimate = engine
        .burn_duration_s(node_dv_mps, entry_mass_kg)
        .ok_or(ThrustPlanError::PropellantExceeded)?;
    let mut pieces = ((first_estimate / max_segment_s).ceil() as usize).max(1);
    loop {
        if pieces > 10_000 {
            return Err(ThrustPlanError::InvalidSegment);
        }
        let piece_dv = node_dv_mps / pieces as f64;
        let mut mass = entry_mass_kg;
        let mut fits = true;
        for _ in 0..pieces {
            let duration = engine
                .burn_duration_s(piece_dv, mass)
                .ok_or(ThrustPlanError::PropellantExceeded)?;
            mass -= engine
                .propellant_kg(piece_dv, mass)
                .ok_or(ThrustPlanError::PropellantExceeded)?;
            if duration > max_segment_s {
                fits = false;
                break;
            }
        }
        if fits {
            return Ok(pieces);
        }
        pieces += 1;
    }
}

/// Validated outcome of flying a finite-burn plan against its impulsive
/// reference over the same horizon (both propagations exact).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BurnValidation {
    /// End state after flying the thrust arcs.
    pub end_state: TestParticleState,
    /// Spacecraft mass after the last arc (must equal the converted
    /// analytic mass to integration tolerance — checked in tests).
    pub final_mass_kg: f64,
    /// End state of the impulsive reference (same nodes, same horizon).
    pub impulsive_end_state: TestParticleState,
    /// Position divergence finite-vs-impulsive (m): the finite-burn
    /// penalty (gravity + steering + rephasing losses), measured not
    /// modeled. For split plans this includes rephasing effects (burns at
    /// different orbit phases genuinely send you elsewhere) — that IS the
    /// operator-relevant number.
    pub divergence_m: f64,
    /// End-velocity difference finite-vs-impulsive (m/s, frame-invariant):
    /// how differently you end up moving. No ballistic baseline is used —
    /// velocity differences against a thrust-free reference conflate orbit
    /// geometry on multi-day horizons and are reported nowhere.
    pub velocity_divergence_mps: f64,
    /// Propellant consumed (kg).
    pub propellant_kg: f64,
}

/// Validate a finite-burn plan: fly the arcs with
/// [`propagate_adaptive_with_thrust`], fly the impulsive reference over
/// the same horizon, and measure divergence. Horizon covers the last
/// segment end.
pub fn validate_finite_burn(
    field: &GravityField<'_>,
    plan: &FiniteBurnPlan,
    impulsive: &ManeuverPlan,
) -> Result<BurnValidation, ThrustPlanError> {
    let departure = TestParticleState {
        position: plan.departure_position_m,
        velocity: plan.departure_velocity_mps,
    };
    let horizon_s = plan
        .segments
        .iter()
        .map(|segment| segment.end().0 - plan.departure_epoch.0)
        .fold(0.0_f64, f64::max);
    if !horizon_s.is_finite() || horizon_s <= 0.0 {
        return Err(ThrustPlanError::InvalidSegment);
    }
    let arcs: Vec<ThrustArc> = plan
        .segments
        .iter()
        .map(|segment| ThrustArc {
            start_s: segment.start.0 - plan.departure_epoch.0,
            duration_s: segment.duration_s,
            direction: match segment.direction {
                SegmentDirection::Inertial(fixed) => ThrustDirection::Inertial(fixed),
                SegmentDirection::Prograde => ThrustDirection::Prograde,
                SegmentDirection::Retrograde => ThrustDirection::Retrograde,
                SegmentDirection::Rtn {
                    central,
                    radial,
                    transverse,
                    normal,
                } => ThrustDirection::Rtn {
                    central,
                    radial,
                    transverse,
                    normal,
                },
            },
            throttle_01: segment.throttle_01,
            thrust_n: plan.engine.thrust_n,
            mass_flow_kgs: plan.engine.mass_flow_kgs(),
        })
        .collect();
    let config = AdaptiveIntegratorConfig::default();
    let finite = propagate_adaptive_with_thrust(
        field,
        departure,
        plan.initial_mass_kg,
        plan.departure_epoch,
        horizon_s,
        &arcs,
        config,
    )
    .map_err(ThrustPlanError::Propagation)?;
    let burns: Vec<ImpulsiveBurn> = impulsive
        .nodes
        .iter()
        .filter(|node| node.magnitude_mps() > 0.0)
        .map(|node| ImpulsiveBurn {
            time_s: node.epoch.0 - plan.departure_epoch.0,
            delta_v_mps: node.delta_v_mps,
        })
        .collect();
    let reference = propagate_adaptive_with_burns(
        field,
        departure,
        plan.departure_epoch,
        horizon_s,
        &burns,
        config,
    )
    .map_err(ThrustPlanError::Propagation)?;
    let divergence_m = (finite.state.position - reference.state.position).length();
    let velocity_divergence_mps = (finite.state.velocity - reference.state.velocity).length();
    Ok(BurnValidation {
        end_state: finite.state,
        final_mass_kg: finite.final_mass_kg,
        impulsive_end_state: reference.state,
        divergence_m,
        velocity_divergence_mps,
        propellant_kg: plan.initial_mass_kg - finite.final_mass_kg,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub enum ThrustPlanError {
    InvalidEngine,
    InvalidMass,
    InvalidSegment,
    OverlappingSegments,
    RequiresBoundOrbit,
    PropellantExceeded,
    Ephemeris(String),
    Propagation(thessa_sim_core::IntegratorError),
}

impl std::fmt::Display for ThrustPlanError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidEngine => write!(formatter, "engine ratings must be finite and positive"),
            Self::InvalidMass => write!(formatter, "spacecraft mass must be finite and positive"),
            Self::InvalidSegment => write!(formatter, "burn segment is invalid"),
            Self::OverlappingSegments => write!(formatter, "burn segments overlap"),
            Self::RequiresBoundOrbit => write!(formatter, "per-orbit split needs a bound orbit"),
            Self::PropellantExceeded => {
                write!(formatter, "maneuver exceeds propellant capacity")
            }
            Self::Ephemeris(error) => write!(formatter, "ephemeris: {error}"),
            Self::Propagation(error) => write!(formatter, "propagation failed: {error}"),
        }
    }
}

impl std::error::Error for ThrustPlanError {}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{BakedBody, BakedEphemeris, BodyId, GravityField};

    const MU: f64 = 1.2e17;
    const RADIUS: f64 = 1.1e9;

    fn chem_engine() -> EngineSpec {
        EngineSpec {
            thrust_n: 100_000.0,
            exhaust_velocity_mps: 4_400.0,
        }
    }

    fn ion_engine() -> EngineSpec {
        EngineSpec {
            thrust_n: 2.0,
            exhaust_velocity_mps: 30_000.0,
        }
    }

    fn circular_velocity() -> DVec3 {
        DVec3::new(0.0, (MU / RADIUS).sqrt(), 0.0)
    }

    fn period() -> f64 {
        orbit_period(DVec3::new(RADIUS, 0.0, 0.0), circular_velocity(), MU)
            .expect("bound test orbit")
    }

    fn single_body_ephemeris() -> BakedEphemeris {
        BakedEphemeris::new(
            "TEST_THRUST",
            vec![BakedBody::fixed(BodyId(0), "central", MU, 0.0)],
        )
        .expect("valid test ephemeris")
    }

    #[test]
    fn rocket_equation_known_values() {
        let engine = chem_engine();
        // m*ve/F = 20000*4400/1e5 = 880 s exactly.
        let duration = engine.burn_duration_s(800.0, 20_000.0).expect("burns");
        let propellant = engine.propellant_kg(800.0, 20_000.0).expect("burns");
        // Cross-identity (exact, not same-formula): t == prop / mdot.
        assert!((duration - propellant / engine.mass_flow_kgs()).abs() < 1e-9);
        // Rough guards against swapped formulas.
        assert!((140.0..=150.0).contains(&duration), "duration {duration}");
        assert!(
            (3_200.0..=3_400.0).contains(&propellant),
            "propellant {propellant}"
        );
        // Tsiolkovsky roundtrip: capacity down to post-burn mass == Δv.
        let dry = 20_000.0 - propellant;
        let capacity = engine
            .delta_v_capacity_mps(20_000.0, dry)
            .expect("capacity");
        assert!((capacity - 800.0).abs() < 1e-9, "capacity {capacity}");
        // Degenerate inputs refused.
        assert!(engine.burn_duration_s(f64::NAN, 20_000.0).is_none());
        assert!(engine.propellant_kg(800.0, 0.0).is_none());
        assert!(engine.delta_v_capacity_mps(20_000.0, 20_000.0).is_none());
        assert!(
            EngineSpec {
                thrust_n: 0.0,
                exhaust_velocity_mps: 4_400.0
            }
            .validate()
            .is_err()
        );
    }

    fn impulsive_plan(delta_v_mps: f64, epoch_s: f64) -> ManeuverPlan {
        ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(epoch_s), DVec3::Y * delta_v_mps).unwrap()],
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap()
    }

    #[test]
    fn realize_single_node_centers_burn() {
        let engine = chem_engine();
        let plan = realize_impulsive(
            &impulsive_plan(800.0, 3_600.0),
            &engine,
            20_000.0,
            600.0,
            SplitMode::PerOrbit {
                orbit_period_s: period(),
            },
        )
        .expect("realizes");
        assert_eq!(plan.segments.len(), 1);
        let segment = plan.segments[0];
        let expected_duration = engine.burn_duration_s(800.0, 20_000.0).unwrap();
        assert!((segment.duration_s - expected_duration).abs() < 1e-9);
        // Centered on the node epoch.
        assert!((segment.start.0 + segment.duration_s / 2.0 - 3_600.0).abs() < 1e-9);
        assert_eq!(segment.planned_dv_mps, 800.0);
        assert!(plan.predicted_miss_m.is_none());
        // Analytic final mass attached at conversion.
        let expected_mass = 20_000.0 - engine.propellant_kg(800.0, 20_000.0).unwrap();
        assert!((plan.final_mass_kg.unwrap() - expected_mass).abs() < 1e-9);
    }

    #[test]
    fn per_orbit_split_phases_on_period() {
        let engine = chem_engine();
        // 3000 m/s needs ~435 s single-burn; equal-Δv pieces at entry mass
        // run ~138 s each, so the fit check settles on 5 pieces of ~110 s.
        // Node epoch 23 d out: symmetric phasing (±2 orbits at T=7.66 d)
        // stays after departure (no clamp, no overlap).
        let expected_period = std::f64::consts::TAU * (RADIUS.powi(3) / MU).sqrt();
        let epoch = 2_000_000.0;
        let plan = realize_impulsive(
            &impulsive_plan(3_000.0, epoch),
            &engine,
            20_000.0,
            120.0,
            SplitMode::PerOrbit {
                orbit_period_s: period(),
            },
        )
        .expect("splits");
        assert_eq!(plan.segments.len(), 5);
        for window in plan.segments.windows(2) {
            let gap = window[1].start.0 - window[0].start.0;
            assert!(
                (gap - expected_period).abs() < 120.0,
                "spacing {gap} vs period {expected_period}"
            );
            assert!(window[0].end().0 <= window[1].start.0);
        }
        // Planned Δv preserved exactly; symmetric about the node epoch.
        let total: f64 = plan
            .segments
            .iter()
            .map(|segment| segment.planned_dv_mps)
            .sum();
        assert_eq!(total, 3_000.0);
        let mean_center = plan
            .segments
            .iter()
            .map(|segment| segment.start.0 + segment.duration_s / 2.0)
            .sum::<f64>()
            / plan.segments.len() as f64;
        assert!((mean_center - epoch).abs() < 1.0);
        // Later pieces run shorter (mass drops) and all fit the cap.
        for segment in &plan.segments {
            assert!(segment.duration_s <= 120.0);
        }
    }

    #[test]
    fn contiguous_split_works_on_hyperbolic_escape() {
        let engine = chem_engine();
        // 1.5x circular speed: unbound, per-orbit phasing meaningless.
        let mut plan = impulsive_plan(3_000.0, 100_000.0);
        plan.departure_velocity_mps = DVec3::new(0.0, 1.5 * (MU / RADIUS).sqrt(), 0.0);
        let chopped = realize_impulsive(
            &plan,
            &engine,
            20_000.0,
            120.0,
            SplitMode::Contiguous { cooldown_s: 600.0 },
        )
        .expect("chops hyperbolic");
        assert_eq!(chopped.segments.len(), 5);
        for window in chopped.segments.windows(2) {
            assert_eq!(window[1].start.0 - window[0].end().0, 600.0);
        }
        assert_eq!(
            chopped
                .segments
                .iter()
                .map(|segment| segment.planned_dv_mps)
                .sum::<f64>(),
            3_000.0
        );
        // The period helper (not conversion) refuses hyperbolic states
        // honestly — callers must pass central-relative state.
        assert_eq!(
            orbit_period(
                DVec3::new(RADIUS, 0.0, 0.0),
                DVec3::new(0.0, 1.5 * (MU / RADIUS).sqrt(), 0.0),
                MU,
            ),
            Err(ThrustPlanError::RequiresBoundOrbit)
        );
    }

    #[test]
    fn burn_plan_execution_gate_mirrors_nodes() {
        use crate::plan::PlanValidation;
        let engine = chem_engine();
        let future = FiniteBurnPlan::new(
            vec![BurnSegment {
                start: SimTime(100.0),
                duration_s: 10.0,
                planned_dv_mps: 10.0,
                direction: SegmentDirection::Prograde,
                throttle_01: 1.0,
            }],
            engine,
            20_000.0,
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        assert_eq!(
            future.validate_for_execution(SimTime(50.0)),
            PlanValidation::Executable
        );
        assert!(matches!(
            future.validate_for_execution(SimTime(150.0)),
            PlanValidation::Stale { .. }
        ));
        let empty = FiniteBurnPlan::new(
            vec![],
            engine,
            20_000.0,
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        assert_eq!(
            empty.validate_for_execution(SimTime(0.0)),
            PlanValidation::Empty
        );
    }

    #[test]
    fn propellant_exceeded_and_bad_inputs_rejected() {
        let engine = chem_engine();
        assert_eq!(
            realize_impulsive(
                &impulsive_plan(1.0e6, 3_600.0),
                &engine,
                20_000.0,
                600.0,
                SplitMode::PerOrbit {
                    orbit_period_s: period(),
                },
            ),
            Err(ThrustPlanError::PropellantExceeded)
        );
        assert!(
            realize_impulsive(
                &impulsive_plan(800.0, 3_600.0),
                &engine,
                0.0,
                600.0,
                SplitMode::PerOrbit {
                    orbit_period_s: period(),
                },
            )
            .is_err()
        );
        // Overlapping hand-built segments refused.
        let overlapping = FiniteBurnPlan::new(
            vec![
                BurnSegment {
                    start: SimTime(100.0),
                    duration_s: 100.0,
                    planned_dv_mps: 10.0,
                    direction: SegmentDirection::Prograde,
                    throttle_01: 1.0,
                },
                BurnSegment {
                    start: SimTime(150.0),
                    duration_s: 100.0,
                    planned_dv_mps: 10.0,
                    direction: SegmentDirection::Prograde,
                    throttle_01: 1.0,
                },
            ],
            engine,
            20_000.0,
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        );
        assert_eq!(overlapping, Err(ThrustPlanError::OverlappingSegments));
    }

    #[test]
    fn validate_chemical_node_measures_small_penalty() {
        let ephemeris = single_body_ephemeris();
        let field = GravityField::from_ephemeris(&ephemeris);
        let engine = chem_engine();
        let impulsive = impulsive_plan(800.0, 3_600.0);
        let plan = realize_impulsive(
            &impulsive,
            &engine,
            20_000.0,
            600.0,
            SplitMode::PerOrbit {
                orbit_period_s: period(),
            },
        )
        .expect("realizes");
        let validation = validate_finite_burn(&field, &plan, &impulsive).expect("validates");
        // A 146 s burn centered on a 7.6-day orbit is near-impulsive:
        // divergence at km scale, end-velocity within tens of m/s.
        assert!(
            validation.divergence_m < 2_000.0,
            "divergence {}",
            validation.divergence_m
        );
        assert!(
            validation.velocity_divergence_mps < 100.0,
            "velocity divergence {}",
            validation.velocity_divergence_mps
        );
        // Propagated mass matches the analytic rocket bill.
        let analytic = 20_000.0 - engine.propellant_kg(800.0, 20_000.0).unwrap();
        assert!((validation.final_mass_kg - analytic).abs() / analytic < 1e-9);
        assert!((validation.propellant_kg - (20_000.0 - analytic)).abs() < 1e-6);
        // Gate attaches like any validated plan.
        let gated = plan.with_validation(&validation);
        assert_eq!(gated.predicted_miss_m, Some(validation.divergence_m));
    }

    #[test]
    fn validate_ion_spiral_gains_energy_over_days() {
        let ephemeris = single_body_ephemeris();
        let field = GravityField::from_ephemeris(&ephemeris);
        let engine = ion_engine();
        // Hand-built 5-day full-throttle prograde arc (the multi-day burn
        // primitive); crude impulsive reference — divergence is a
        // measurement here, energy and mass are the assertions.
        let duration = 5.0 * 86_400.0;
        let plan = FiniteBurnPlan::new(
            vec![BurnSegment {
                start: SimTime(0.0),
                duration_s: duration,
                planned_dv_mps: engine.thrust_n / 2_000.0 * duration,
                direction: SegmentDirection::Prograde,
                throttle_01: 1.0,
            }],
            engine,
            2_000.0,
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        let reference = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(0.0), DVec3::Y * 400.0).unwrap()],
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        let validation = validate_finite_burn(&field, &plan, &reference).expect("validates");
        let energy = |state: TestParticleState| {
            state.velocity.length_squared() / 2.0 - MU / state.position.length()
        };
        let start = TestParticleState {
            position: DVec3::new(RADIUS, 0.0, 0.0),
            velocity: circular_velocity(),
        };
        assert!(energy(validation.end_state) > energy(start));
        assert!(validation.end_state.position.length() > RADIUS);
        let expected_mass = 2_000.0 - engine.mass_flow_kgs() * duration;
        assert!((validation.final_mass_kg - expected_mass).abs() / expected_mass < 1e-12);
        assert!(validation.divergence_m.is_finite());
    }

    #[test]
    fn per_node_periods_follow_raised_orbits() {
        // Two prograde nodes a full orbit apart: each burn raises the
        // orbit, so each period exceeds the previous one. Same anomaly
        // (single static body = exact Kepler return), so the fixed +Y
        // node direction is prograde both times.
        let ephemeris = single_body_ephemeris();
        let field = GravityField::from_ephemeris(&ephemeris);
        let one = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(1_000.0), DVec3::Y * 800.0).unwrap()],
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        let first =
            node_osculating_periods(&field, &ephemeris, BodyId(0), &one).expect("periods")[0];
        let two = ManeuverPlan::new(
            vec![
                ManeuverNode::new(SimTime(1_000.0), DVec3::Y * 800.0).unwrap(),
                ManeuverNode::new(SimTime(1_000.0 + first), DVec3::Y * 800.0).unwrap(),
            ],
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        let periods =
            node_osculating_periods(&field, &ephemeris, BodyId(0), &two).expect("periods");
        assert_eq!(periods.len(), 2);
        assert!((periods[0] - first).abs() / first < 1e-9);
        assert!(periods[0] > period());
        assert!(periods[1] > periods[0]);
    }

    #[test]
    fn per_node_split_phases_each_node_on_its_period() {
        let engine = chem_engine();
        let ephemeris = single_body_ephemeris();
        let field = GravityField::from_ephemeris(&ephemeris);
        // Node 1 (800 m/s) splits in two on periods[0]; node 2 (3000 m/s)
        // splits on its own (longer) periods[1].
        let two = ManeuverPlan::new(
            vec![
                ManeuverNode::new(SimTime(2_000_000.0), DVec3::Y * 800.0).unwrap(),
                ManeuverNode::new(SimTime(4_000_000.0), DVec3::Y * 3_000.0).unwrap(),
            ],
            DVec3::new(RADIUS, 0.0, 0.0),
            circular_velocity(),
            SimTime(0.0),
        )
        .unwrap();
        let periods =
            node_osculating_periods(&field, &ephemeris, BodyId(0), &two).expect("periods");
        assert_eq!(periods.len(), 2);
        let plan = realize_impulsive(
            &two,
            &engine,
            20_000.0,
            120.0,
            SplitMode::PerOrbitNodes {
                periods_s: periods.clone(),
            },
        )
        .expect("splits per node");
        // Node-1 pieces (first two segments) space on periods[0], node-2
        // pieces on periods[1].
        assert!(plan.segments.len() >= 4);
        let gap_node1 = plan.segments[1].start.0 - plan.segments[0].start.0;
        assert!((gap_node1 - periods[0]).abs() < 120.0);
        let tail = &plan.segments[2..];
        for window in tail.windows(2) {
            // All tail pieces belong to node 2 (node-1 span is ~1 period,
            // nodes are 2e6 s apart): spacing must match periods[1].
            let gap = window[1].start.0 - window[0].start.0;
            assert!((gap - periods[1]).abs() < 120.0, "gap {gap}");
        }
        // Length mismatch refused.
        assert!(
            realize_impulsive(
                &two,
                &engine,
                20_000.0,
                120.0,
                SplitMode::PerOrbitNodes {
                    periods_s: vec![periods[0]],
                },
            )
            .is_err()
        );
    }
}
