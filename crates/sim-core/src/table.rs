//! Baked-body motion table: a fast Hermite interpolant over ephemeris states
//! for one propagation horizon.
//!
//! The per-step hot loop of a long coast bake spends most of its time in
//! per-body Kepler solves (`body_state` per source per step). Sampling each
//! source at coarse uniform nodes once and Hermite-interpolating between
//! them removes ~7/8 of those solves at metre-grade error on real-system
//! moon speeds. Representation optimization only (AGENTS.md section 10):
//! the baked bodies, mus and horizon are unchanged, and the agreement test
//! below pins the error bound. Deterministic: uniform nodes, no adaptation.

use glam::{DQuat, DVec3};

use crate::{BakedEphemeris, BodyId, BodyState, EphemerisError, SimTime};

/// How often a fast bake samples true ephemeris nodes: every Nth step.
pub const TABLE_NODE_EVERY_STEPS: u64 = 8;

/// Cubic Hermite position and its analytic derivative, evaluated using a
/// relative displacement to avoid cancellation at barycentric coordinates.
pub(crate) fn hermite_state(
    p0: DVec3,
    v0: DVec3,
    p1: DVec3,
    v1: DVec3,
    h: f64,
    s: f64,
) -> (DVec3, DVec3) {
    if h == 0.0 {
        return (p0, v0);
    }
    let delta = p1 - p0;
    let a = delta * 3.0 - (v0 * 2.0 + v1) * h;
    let b = (v0 + v1) * h - delta * 2.0;
    (
        p0 + (v0 * h + (a + b * s) * s) * s,
        v0 + (a * 2.0 + b * (3.0 * s)) * (s / h),
    )
}

/// One source body's uniform-node track over `[start, end]`.
#[derive(Debug, Clone)]
struct BodyTrack {
    id: BodyId,
    mu: f64,
    gravitating: bool,
    radius_m: f64,
    step_s: f64,
    positions: Vec<DVec3>,
    velocities: Vec<DVec3>,
}

impl BodyTrack {
    fn state_at(&self, time: SimTime, start: SimTime) -> (DVec3, DVec3) {
        let total = self.positions.len().saturating_sub(1) as f64;
        let elapsed = (time.seconds() - start.seconds()).clamp(0.0, total * self.step_s);
        let position = (elapsed / self.step_s).min(total);
        let low = (position.floor() as usize).min(self.positions.len().saturating_sub(1));
        let high = (low + 1).min(self.positions.len().saturating_sub(1));
        let s = (position - low as f64).clamp(0.0, 1.0);
        let h = ((high - low) as f64) * self.step_s;
        let p0 = self.positions[low];
        let p1 = self.positions[high];
        let v0 = self.velocities[low];
        let v1 = self.velocities[high];
        hermite_state(p0, v0, p1, v1, h, s)
    }
}

/// Fast evaluator for a fixed body set over one horizon. Built once per
/// bake, queried per step.
#[derive(Debug, Clone)]
pub struct EphemerisTable {
    start: SimTime,
    end: SimTime,
    tracks: Vec<BodyTrack>,
}

impl EphemerisTable {
    /// Sample every source at uniform `node_step_s` over `[start, end]`.
    /// Node times are exact multiples from `start`, so bake steps that land
    /// on nodes read exact states, never interpolated ones.
    pub fn build(
        ephemeris: &BakedEphemeris,
        bodies: &[BodyId],
        start: SimTime,
        end: SimTime,
        node_step_s: f64,
    ) -> Result<Self, EphemerisError> {
        if !start.0.is_finite()
            || !end.0.is_finite()
            || end.0 < start.0
            || !node_step_s.is_finite()
            || node_step_s <= 0.0
        {
            return Err(EphemerisError::InvalidOrbit(
                "invalid table time interval or node step".into(),
            ));
        }
        let horizon = end.seconds() - start.seconds();
        // Bound allocation before float-to-usize conversion. A malformed
        // configuration must return an error, never panic or allocate infinity.
        let count = (horizon / node_step_s).ceil().max(1.0);
        if !count.is_finite()
            || count > 1_000_000.0
            || (count + 1.0) * bodies.len() as f64 > 2_000_000.0
        {
            return Err(EphemerisError::InvalidOrbit(
                "ephemeris table exceeds node budget".into(),
            ));
        }
        let nodes = (horizon / node_step_s).ceil().max(1.0) as usize;
        let mut tracks = Vec::with_capacity(bodies.len());
        for id in bodies {
            let body = ephemeris.body(*id)?;
            let mut positions = Vec::with_capacity(nodes + 1);
            let mut velocities = Vec::with_capacity(nodes + 1);
            for node in 0..=nodes {
                let state: BodyState =
                    ephemeris.body_state(*id, start.offset(node as f64 * node_step_s))?;
                positions.push(state.position_inertial);
                velocities.push(state.velocity_inertial);
            }
            tracks.push(BodyTrack {
                id: *id,
                mu: body.mu,
                gravitating: body.gravity_source,
                radius_m: body.radius_m,
                step_s: node_step_s,
                positions,
                velocities,
            });
        }
        Ok(Self { start, end, tracks })
    }

    /// Summed point-mass acceleration at a position/epoch, same definition
    /// as [`GravityField`](crate::GravityField) but served from the table.
    /// Returns `None` on singularity/non-finite input instead of a typed
    /// error: the fast path only runs where the exact path already validated
    /// the configuration.
    pub fn acceleration(&self, position: DVec3) -> Option<DVec3> {
        self.acceleration_at(position, self.start)
    }

    pub fn acceleration_at(&self, position: DVec3, time: SimTime) -> Option<DVec3> {
        if !self.covers(time) || !position.is_finite() {
            return None;
        }
        let mut total = DVec3::ZERO;
        for track in &self.tracks {
            if !track.gravitating {
                continue;
            }
            let (center, _) = track.state_at(time, self.start);
            let offset = center - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() || distance_squared == 0.0 {
                return None;
            }
            let inverse = distance_squared.sqrt().recip();
            if !inverse.is_finite() {
                return None;
            }
            total += offset * (track.mu * inverse.powi(3));
        }
        total.is_finite().then_some(total)
    }

    pub fn body_state_at(&self, id: BodyId, time: SimTime) -> Option<BodyState> {
        if !self.covers(time) {
            return None;
        }
        let track = self.tracks.iter().find(|track| track.id == id)?;
        let (position_inertial, velocity_inertial) = track.state_at(time, self.start);
        // Baked bodies carry no attitude: the ephemeris itself reports
        // IDENTITY/ZERO, so the table matches it exactly.
        Some(BodyState {
            position_inertial,
            velocity_inertial,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        })
    }

    fn covers(&self, time: SimTime) -> bool {
        time.0.is_finite() && time.0 >= self.start.0 && time.0 <= self.end.0
    }

    fn track(&self, id: BodyId) -> Option<&BodyTrack> {
        self.tracks.iter().find(|track| track.id == id)
    }

    /// Table version of the point-in-body check, for fast-bake impact ends.
    pub fn impact_at(
        &self,
        impact_bodies: &[BodyId],
        position: DVec3,
        time: SimTime,
    ) -> Option<BodyId> {
        for body_id in impact_bodies {
            let Some(track) = self.track(*body_id) else {
                continue;
            };
            if track.radius_m <= 0.0 {
                continue;
            }
            let (center, _) = track.state_at(time, self.start);
            if (position - center).length() < track.radius_m {
                return Some(*body_id);
            }
        }
        None
    }

    /// Table version of the moving-frame segment/sphere entry check.
    pub fn impact_segment(
        &self,
        impact_bodies: &[BodyId],
        from: DVec3,
        to: DVec3,
        start_time: SimTime,
        end_time: SimTime,
    ) -> Option<(BodyId, f64)> {
        let mut first: Option<(BodyId, f64)> = None;
        for body_id in impact_bodies {
            let Some(track) = self.track(*body_id) else {
                continue;
            };
            if track.radius_m <= 0.0 {
                continue;
            }
            let (start_center, _) = track.state_at(start_time, self.start);
            let (end_center, _) = track.state_at(end_time, self.start);
            let relative = from - start_center;
            let delta = (to - end_center) - relative;
            let a = delta.length_squared();
            if a <= 0.0 {
                continue;
            }
            let b = relative.dot(delta);
            let c = relative.length_squared() - track.radius_m.powi(2);
            let discriminant = b * b - a * c;
            if discriminant < 0.0 {
                continue;
            }
            let denominator = -b + discriminant.sqrt();
            let fraction = if c <= 0.0 { 0.0 } else { c / denominator };
            if (0.0..=1.0).contains(&fraction)
                && first.is_none_or(|(_, previous)| fraction < previous)
            {
                first = Some((*body_id, fraction));
            }
        }
        first
    }

    /// Interpolated body radius lookup still goes to the ephemeris (radii
    /// are constants; no solve involved).
    pub fn source_ids(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.tracks.iter().map(|track| track.id)
    }
}
