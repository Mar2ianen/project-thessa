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

/// Cubic Hermite velocity from endpoint velocities and accelerations.
///
/// Companion to [`hermite_state`], but intentionally NOT its analytic
/// derivative: differentiating the position Hermite divides f64 rounding of
/// ~1e12 m barycentric positions (~1e-3 m) by the step, manufacturing
/// 0.01-0.1 m/s of pure noise at second-scale nodes. Interpolating the
/// stored endpoint accelerations instead is rounding-clean by construction
/// (all terms stay O(v) or O(a*h)) and exactly matches endpoint slopes.
pub(crate) fn hermite_velocity(
    v0: DVec3,
    a0: DVec3,
    v1: DVec3,
    a1: DVec3,
    h: f64,
    s: f64,
) -> DVec3 {
    // Cubic Hermite on endpoint velocities with endpoint slopes a0*h,
    // a1*h, in Horner form. No division by h anywhere: every term stays
    // O(v) or O(a*h), so ~1e-3 m f64 rounding of barycentric positions
    // never enters at all.
    let dv = v1 - v0;
    let m0 = a0 * h;
    let m1 = a1 * h;
    let c2 = dv * 3.0 - m0 * 2.0 - m1;
    let c3 = m0 + m1 - dv * 2.0;
    v0 + (m0 + (c2 + c3 * s) * s) * s
}

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

/// Per-body constants, index-aligned with the component arrays below.
#[derive(Debug, Clone, Copy)]
struct TrackMeta {
    id: BodyId,
    /// Effective mu: real mu for gravity sources, exactly 0.0 otherwise, so
    /// the vector loop needs no skip mask (`x + ±0.0 == x` bitwise, and
    /// `offset * 0.0` contributes nothing).
    eff_mu: f64,
    gravitating: bool,
    radius_m: f64,
}

/// One body's six component runs, as produced per parallel task.
type BuiltTrack = (TrackMeta, [Vec<f64>; 6]);

/// Component-major node storage: `pos_x[node * bodies + body]` and siblings.
/// One node interval of all bodies sits in six contiguous runs, so an
/// 8-wide kernel loads 8 bodies with plain `loadu` — no gathers. Scalar
/// readers use the same arrays through [`EphemerisTable::node_state`].
#[derive(Debug, Clone, Default)]
struct NodeArrays {
    bodies: usize,
    points: usize,
    pos_x: Vec<f64>,
    pos_y: Vec<f64>,
    pos_z: Vec<f64>,
    vel_x: Vec<f64>,
    vel_y: Vec<f64>,
    vel_z: Vec<f64>,
}

impl NodeArrays {
    fn node_state(&self, body: usize, low: usize, high: usize, h: f64, s: f64) -> (DVec3, DVec3) {
        let stride = self.bodies;
        let p0 = DVec3::new(
            self.pos_x[low * stride + body],
            self.pos_y[low * stride + body],
            self.pos_z[low * stride + body],
        );
        let p1 = DVec3::new(
            self.pos_x[high * stride + body],
            self.pos_y[high * stride + body],
            self.pos_z[high * stride + body],
        );
        let v0 = DVec3::new(
            self.vel_x[low * stride + body],
            self.vel_y[low * stride + body],
            self.vel_z[low * stride + body],
        );
        let v1 = DVec3::new(
            self.vel_x[high * stride + body],
            self.vel_y[high * stride + body],
            self.vel_z[high * stride + body],
        );
        hermite_state(p0, v0, p1, v1, h, s)
    }
}

/// One instant of every track's centers, in meta order, component-major.
/// Reused across the two accel evals and the impact test of a single Verlet
/// step; refilled per endpoint by [`snapshot`](EphemerisTable::snapshot).
/// Component runs (not `Vec<DVec3>`) so an 8-wide kernel loads 8 bodies
/// with plain `loadu` — no gathers.
#[derive(Debug, Clone, Default)]
pub struct TableSnapshot {
    time: SimTime,
    pub(crate) cx: Vec<f64>,
    pub(crate) cy: Vec<f64>,
    pub(crate) cz: Vec<f64>,
}

/// Fast evaluator for a fixed body set over one horizon. Built once per
/// bake, queried per step.
#[derive(Debug, Clone)]
pub struct EphemerisTable {
    start: SimTime,
    end: SimTime,
    node_step_s: f64,
    meta: Vec<TrackMeta>,
    /// Effective mu per body (zero for non-gravitating lanes), contiguous
    /// for the 8-wide gravity kernel.
    eff_mu: Vec<f64>,
    arrays: NodeArrays,
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
        // Tracks are independent per body (no shared mutable state, no
        // reduction), so a parallel fill is bitwise identical to the serial
        // loop: Rayon preserves encounter order on collect. Kepler solves
        // dominate (~85% of a full-horizon bake). Each task returns its own
        // component runs; interleaving below is a memcpy next to the solves.
        use rayon::prelude::*;
        let built: Result<Vec<BuiltTrack>, EphemerisError> = bodies
            .par_iter()
            .map(|id| {
                let body = ephemeris.body(*id)?;
                let mut px = Vec::with_capacity(nodes + 1);
                let mut py = Vec::with_capacity(nodes + 1);
                let mut pz = Vec::with_capacity(nodes + 1);
                let mut vx = Vec::with_capacity(nodes + 1);
                let mut vy = Vec::with_capacity(nodes + 1);
                let mut vz = Vec::with_capacity(nodes + 1);
                for node in 0..=nodes {
                    let state: BodyState =
                        ephemeris.body_state(*id, start.offset(node as f64 * node_step_s))?;
                    px.push(state.position_inertial.x);
                    py.push(state.position_inertial.y);
                    pz.push(state.position_inertial.z);
                    vx.push(state.velocity_inertial.x);
                    vy.push(state.velocity_inertial.y);
                    vz.push(state.velocity_inertial.z);
                }
                Ok((
                    TrackMeta {
                        id: *id,
                        eff_mu: if body.gravity_source { body.mu } else { 0.0 },
                        gravitating: body.gravity_source,
                        radius_m: body.radius_m,
                    },
                    [px, py, pz, vx, vy, vz],
                ))
            })
            .collect();
        let built = built?;
        let nbodies = built.len();
        let points = nodes + 1;
        let mut meta = Vec::with_capacity(nbodies);
        let mut arrays = NodeArrays {
            bodies: nbodies,
            points,
            pos_x: vec![0.0; points * nbodies],
            pos_y: vec![0.0; points * nbodies],
            pos_z: vec![0.0; points * nbodies],
            vel_x: vec![0.0; points * nbodies],
            vel_y: vec![0.0; points * nbodies],
            vel_z: vec![0.0; points * nbodies],
        };
        for (body, (track_meta, comps)) in built.into_iter().enumerate() {
            meta.push(track_meta);
            let [px, py, pz, vx, vy, vz] = comps;
            for node in 0..points {
                let base = node * nbodies + body;
                arrays.pos_x[base] = px[node];
                arrays.pos_y[base] = py[node];
                arrays.pos_z[base] = pz[node];
                arrays.vel_x[base] = vx[node];
                arrays.vel_y[base] = vy[node];
                arrays.vel_z[base] = vz[node];
            }
        }
        let eff_mu = meta.iter().map(|track| track.eff_mu).collect();
        Ok(Self {
            start,
            end,
            node_step_s,
            meta,
            eff_mu,
            arrays,
        })
    }

    /// Node interval bracketing `time`, or `None` outside coverage.
    fn segment(&self, time: SimTime) -> Option<(usize, usize, f64, f64)> {
        if !self.covers(time) || self.arrays.points < 2 {
            return None;
        }
        let total = (self.arrays.points - 1) as f64;
        let elapsed = (time.seconds() - self.start.seconds()).clamp(0.0, total * self.node_step_s);
        let position = (elapsed / self.node_step_s).min(total);
        let low = (position.floor() as usize).min(self.arrays.points - 1);
        let high = (low + 1).min(self.arrays.points - 1);
        let s = (position - low as f64).clamp(0.0, 1.0);
        let h = ((high - low) as f64) * self.node_step_s;
        Some((low, high, h, s))
    }

    fn node_state(&self, body: usize, low: usize, high: usize, h: f64, s: f64) -> (DVec3, DVec3) {
        self.arrays.node_state(body, low, high, h, s)
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
        let (low, high, h, s) = self.segment(time)?;
        if !position.is_finite() {
            return None;
        }
        let mut total = DVec3::ZERO;
        for body in 0..self.meta.len() {
            if !self.meta[body].gravitating {
                continue;
            }
            let (center, _) = self.node_state(body, low, high, h, s);
            let offset = center - position;
            let distance_squared = offset.length_squared();
            if !distance_squared.is_finite() || distance_squared == 0.0 {
                return None;
            }
            let inverse = distance_squared.sqrt().recip();
            if !inverse.is_finite() {
                return None;
            }
            total += offset * (self.meta[body].eff_mu * inverse.powi(3));
        }
        total.is_finite().then_some(total)
    }

    /// Fill `out` with every track's center at `time`, in meta order.
    /// A Verlet step needs body centers at both endpoints for two accel
    /// evals plus the impact segment test. Snapshotting once per endpoint
    /// cuts Hermite evals with zero id lookups.
    pub fn snapshot(&self, time: SimTime, out: &mut TableSnapshot) {
        self.snapshot_with(time, out, true);
    }

    /// Snapshot with an explicit SIMD switch: the cross-path agreement test
    /// forces both sides on the same table. Production always passes `true`
    /// (dispatch inside degrades gracefully per chunk).
    pub(crate) fn snapshot_with(&self, time: SimTime, out: &mut TableSnapshot, use_simd: bool) {
        out.time = time;
        let n = self.meta.len();
        out.cx.resize(n, 0.0);
        out.cy.resize(n, 0.0);
        out.cz.resize(n, 0.0);
        let Some((low, high, h, s)) = self.segment(time) else {
            out.cx.clear();
            out.cy.clear();
            out.cz.clear();
            return;
        };
        let a = &self.arrays;
        let (lx, hx) = (low * n, high * n);
        // Width cascade: 8-wide AVX-512, then 4-wide AVX2, then scalar.
        // Each tier degrades per chunk, so a missing tier falls through
        // without disturbing the rest.
        let mut body = 0;
        while body + 8 <= n {
            let done8 = use_simd
                && thessa_simd::hermite_snapshot_chunk(
                    &a.pos_x[lx..],
                    &a.pos_x[hx..],
                    &a.vel_x[lx..],
                    &a.vel_x[hx..],
                    &a.pos_y[lx..],
                    &a.pos_y[hx..],
                    &a.vel_y[lx..],
                    &a.vel_y[hx..],
                    &a.pos_z[lx..],
                    &a.pos_z[hx..],
                    &a.vel_z[lx..],
                    &a.vel_z[hx..],
                    body,
                    h,
                    s,
                    &mut out.cx,
                    &mut out.cy,
                    &mut out.cz,
                );
            if !done8 {
                // Split 8-lane miss into two 4-wide attempts before scalar.
                for half in [body, body + 4] {
                    let done4 = use_simd
                        && thessa_simd::hermite_snapshot_quad(
                            &a.pos_x[lx..],
                            &a.pos_x[hx..],
                            &a.vel_x[lx..],
                            &a.vel_x[hx..],
                            &a.pos_y[lx..],
                            &a.pos_y[hx..],
                            &a.vel_y[lx..],
                            &a.vel_y[hx..],
                            &a.pos_z[lx..],
                            &a.pos_z[hx..],
                            &a.vel_z[lx..],
                            &a.vel_z[hx..],
                            half,
                            h,
                            s,
                            &mut out.cx,
                            &mut out.cy,
                            &mut out.cz,
                        );
                    if !done4 {
                        self.snapshot_scalar_range(out, low, high, h, s, half, half + 4);
                    }
                }
            }
            body += 8;
        }
        while body + 4 <= n {
            let done4 = use_simd
                && thessa_simd::hermite_snapshot_quad(
                    &a.pos_x[lx..],
                    &a.pos_x[hx..],
                    &a.vel_x[lx..],
                    &a.vel_x[hx..],
                    &a.pos_y[lx..],
                    &a.pos_y[hx..],
                    &a.vel_y[lx..],
                    &a.vel_y[hx..],
                    &a.pos_z[lx..],
                    &a.pos_z[hx..],
                    &a.vel_z[lx..],
                    &a.vel_z[hx..],
                    body,
                    h,
                    s,
                    &mut out.cx,
                    &mut out.cy,
                    &mut out.cz,
                );
            if !done4 {
                self.snapshot_scalar_range(out, low, high, h, s, body, body + 4);
            }
            body += 4;
        }
        self.snapshot_scalar_range(out, low, high, h, s, body, n);
    }

    /// Scalar snapshot fill over `[from, to)`: the pre-SIMD loop, bitwise
    /// identical to it, also serving every fallback lane.
    #[allow(clippy::too_many_arguments)]
    fn snapshot_scalar_range(
        &self,
        out: &mut TableSnapshot,
        low: usize,
        high: usize,
        h: f64,
        s: f64,
        from: usize,
        to: usize,
    ) {
        for b in from..to {
            let (center, _) = self.node_state(b, low, high, h, s);
            out.cx[b] = center.x;
            out.cy[b] = center.y;
            out.cz[b] = center.z;
        }
    }

    /// Summed gravity from a snapshot (centers must come from [`snapshot`](Self::snapshot)).
    /// Index-aligned with meta order; use [`impact_from`](Self::impact_from)
    /// for the segment test on a pair.
    pub fn accel_from(&self, snapshot: &TableSnapshot, position: DVec3) -> Option<DVec3> {
        self.accel_with(snapshot, position, true)
    }

    /// Accel with an explicit SIMD switch (see [`snapshot_with`](Self::snapshot_with)).
    pub(crate) fn accel_with(
        &self,
        snapshot: &TableSnapshot,
        position: DVec3,
        use_simd: bool,
    ) -> Option<DVec3> {
        if snapshot.cx.len() != self.meta.len() || !position.is_finite() {
            return None;
        }
        let n = self.meta.len();
        let mut total = (0.0, 0.0, 0.0);
        let mut body = 0;
        while body + 8 <= n {
            let done = use_simd
                && thessa_simd::gravity_chunk(
                    &snapshot.cx,
                    &snapshot.cy,
                    &snapshot.cz,
                    &self.eff_mu,
                    body,
                    position.x,
                    position.y,
                    position.z,
                    &mut total,
                );
            if !done {
                // Singular 8-lane (or no AVX-512): two 4-wide attempts, then
                // scalar, same `None` semantics as the pre-SIMD loop.
                for half in [body, body + 4] {
                    let done4 = use_simd
                        && thessa_simd::gravity_quad(
                            &snapshot.cx,
                            &snapshot.cy,
                            &snapshot.cz,
                            &self.eff_mu,
                            half,
                            position.x,
                            position.y,
                            position.z,
                            &mut total,
                        );
                    if !done4 {
                        for index in half..half + 4 {
                            total = self.scalar_term(snapshot, position, index, total)?;
                        }
                    }
                }
            }
            body += 8;
        }
        while body + 4 <= n {
            let done4 = use_simd
                && thessa_simd::gravity_quad(
                    &snapshot.cx,
                    &snapshot.cy,
                    &snapshot.cz,
                    &self.eff_mu,
                    body,
                    position.x,
                    position.y,
                    position.z,
                    &mut total,
                );
            if !done4 {
                for index in body..body + 4 {
                    total = self.scalar_term(snapshot, position, index, total)?;
                }
            }
            body += 4;
        }
        for index in body..n {
            total = self.scalar_term(snapshot, position, index, total)?;
        }
        let total = DVec3::new(total.0, total.1, total.2);
        total.is_finite().then_some(total)
    }

    /// One scalar gravity term; `None` on singularity/non-finite input.
    /// Skip non-gravitating bodies before checking distance: their centers
    /// are not gravitational singularities, including in SIMD fallback.
    fn scalar_term(
        &self,
        snapshot: &TableSnapshot,
        position: DVec3,
        index: usize,
        total: (f64, f64, f64),
    ) -> Option<(f64, f64, f64)> {
        if !self.meta[index].gravitating {
            return Some(total);
        }
        let dx = snapshot.cx[index] - position.x;
        let dy = snapshot.cy[index] - position.y;
        let dz = snapshot.cz[index] - position.z;
        let distance_squared = dx * dx + dy * dy + dz * dz;
        if !distance_squared.is_finite() || distance_squared == 0.0 {
            return None;
        }
        let inverse = distance_squared.sqrt().recip();
        if !inverse.is_finite() {
            return None;
        }
        let t = self.eff_mu[index] * inverse.powi(3);
        let out = (total.0 + dx * t, total.1 + dy * t, total.2 + dz * t);
        if out.0.is_finite() && out.1.is_finite() && out.2.is_finite() {
            Some(out)
        } else {
            None
        }
    }

    /// Moving-frame segment/sphere entry between two snapshots. Iterates
    /// meta once (index-aligned centers, integer membership test).
    pub fn impact_from(
        &self,
        start_snap: &TableSnapshot,
        end_snap: &TableSnapshot,
        from: DVec3,
        to: DVec3,
        impact_bodies: &[BodyId],
    ) -> Option<(BodyId, f64)> {
        if start_snap.cx.len() != self.meta.len() || end_snap.cx.len() != self.meta.len() {
            return None;
        }
        let mut first: Option<(BodyId, f64)> = None;
        for (index, track) in self.meta.iter().enumerate() {
            if track.radius_m <= 0.0 || !impact_bodies.contains(&track.id) {
                continue;
            }
            let start_center = DVec3::new(
                start_snap.cx[index],
                start_snap.cy[index],
                start_snap.cz[index],
            );
            let end_center = DVec3::new(end_snap.cx[index], end_snap.cy[index], end_snap.cz[index]);
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
                first = Some((track.id, fraction));
            }
        }
        first
    }

    pub fn body_state_at(&self, id: BodyId, time: SimTime) -> Option<BodyState> {
        if !self.covers(time) {
            return None;
        }
        let index = self.meta.iter().position(|track| track.id == id)?;
        let (low, high, h, s) = self.segment(time)?;
        let (position_inertial, velocity_inertial) = self.node_state(index, low, high, h, s);
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

    /// Table version of the point-in-body check, for fast-bake impact ends.
    pub fn impact_at(
        &self,
        impact_bodies: &[BodyId],
        position: DVec3,
        time: SimTime,
    ) -> Option<BodyId> {
        let (low, high, h, s) = self.segment(time)?;
        for (index, track) in self.meta.iter().enumerate() {
            if track.radius_m <= 0.0 || !impact_bodies.contains(&track.id) {
                continue;
            }
            let (center, _) = self.node_state(index, low, high, h, s);
            if (position - center).length() < track.radius_m {
                return Some(track.id);
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
        let (slow, shigh, sh, ss) = self.segment(start_time)?;
        let (elow, ehigh, eh, es) = self.segment(end_time)?;
        let mut first: Option<(BodyId, f64)> = None;
        for (index, track) in self.meta.iter().enumerate() {
            if track.radius_m <= 0.0 || !impact_bodies.contains(&track.id) {
                continue;
            }
            let (start_center, _) = self.node_state(index, slow, shigh, sh, ss);
            let (end_center, _) = self.node_state(index, elow, ehigh, eh, es);
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
                first = Some((track.id, fraction));
            }
        }
        first
    }

    /// Body ids in meta order.
    pub fn source_ids(&self) -> impl Iterator<Item = BodyId> + '_ {
        self.meta.iter().map(|track| track.id)
    }
}

#[cfg(test)]
mod hermite_velocity_tests {
    use super::*;
    use glam::DVec3;

    /// Quadratic motion is reproduced exactly, including at barycentric
    /// ~1e12 m coordinates where differentiating the position Hermite
    /// amplifies f64 rounding (~1e-3 m) by ~1/h into 0.01-0.1 m/s of noise.
    /// This test pins the regression that broke adaptive baking on the real
    /// system: position-derived velocity noise tripped the interpolation
    /// budget at every small step.
    #[test]
    fn barycentric_quadratic_is_exact() {
        // x(t) = origin + v*t + a*t^2/2 with a barycentric-scale origin.
        let origin = DVec3::new(4.82199934755193e12, 1.101318193678849e8, 1.0001922164810932e9);
        let v = DVec3::new(-2310.338894347516, 50918.39341240515, 240.330294051677);
        let a = DVec3::new(0.5, -0.3, 0.1);
        let h = 1.0 / 60.0;
        let state_at = |t: f64| (origin + v * t + a * (0.5 * t * t), v + a * t);
        let (p0, v0) = state_at(0.0);
        let (p1, v1) = state_at(h);
        for s in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let t = s * h;
            let (pe, ve) = state_at(t);
            let got = hermite_velocity(v0, a, v1, a, h, s);
            // Exact up to f64 rounding of O(v) terms (~1e-11 at 5e4 m/s).
            assert!(got.distance(ve) < 1e-9, "s={s}: got {got:?}, want {ve:?}");
            let (ph, _) = hermite_state(p0, v0, p1, v1, h, s);
            assert!(ph.distance(pe) < 1e-3, "position baseline moved at s={s}");
        }
    }

    #[test]
    fn endpoints_recover_exactly() {
        let v0 = DVec3::new(1.0, -2.0, 3.0);
        let a0 = DVec3::new(0.1, 0.2, -0.1);
        let v1 = DVec3::new(1.5, -1.0, 4.0);
        let a1 = DVec3::new(-0.2, 0.1, 0.3);
        let h = 5.0;
        assert_eq!(hermite_velocity(v0, a0, v1, a1, h, 0.0), v0);
        assert_eq!(hermite_velocity(v0, a0, v1, a1, h, 1.0), v1);
    }
}
