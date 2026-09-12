//! On-rails coast cache: bake the unpowered prediction once, sample it until
//! the craft leaves it.
//!
//! Invariant (AGENTS.md sections 1, 8, 10): this cache optimizes
//! *representation*, never physics. It is valid only for an unpowered coast —
//! zero thrust, zero aerodynamic load — where translation is exactly the test
//! particle the prediction integrates (see
//! `sampled_verlet_agrees_with_adaptive_on_unpowered_coast`). Any burn,
//! throttle, atmosphere entry (density above the Coast threshold) or contact
//! handling must invalidate the cache; the cache itself is pure data and
//! cannot observe inputs, so the caller owns invalidation and the tolerance
//! check below is the contract.
//!
//! Time is [`SimTime`], positions/velocities are inertial SI (`f64`). No
//! Bevy, no Tokio, no wall clock.

use glam::DVec3;

use crate::{
    BakedEphemeris, BodyId, IntegratorError, SampledPath, SimTime, TABLE_NODE_EVERY_STEPS,
    TestParticleState, VerletConfig, propagate_sampled_verlet, propagate_sampled_verlet_fast,
};

/// Authoritative on-rails bake sizing, shared by the flight loop and the map
/// prediction so both consume one trajectory: 5 s samples keep cubic-Hermite
/// interpolation error metre-grade (~1 m) at orbital speeds, and 40 000
/// samples cover ~2.3 days of cruise per bake. Interpolation and accumulated
/// integration errors differ: the circular 200000 s regression measures
/// about 130 m position error at 5 s (33 m at 2.5 s), not a global 1 m guarantee.
pub const COAST_RAILS_STEP_S: f64 = 5.0;
pub const COAST_RAILS_MAX_STEPS: u64 = 40_000;

/// Chunked-bake sizing for hitch-free coast entry: the head bake covers
/// ~2.8 h of cruise synchronously (~9 ms release), then per-frame extensions
/// of ~0.7 h grow coverage toward the full horizon. Extension chunks stay
/// small enough to hold a steady frame budget.
pub const COAST_RAILS_HEAD_STEPS: u64 = 2048;
pub const COAST_RAILS_EXTEND_CHUNK: u64 = 512;
/// Keep at least this much baked lookahead ahead of the flown epoch; the
/// flight loop extends toward it one chunk per frame.
pub const COAST_RAILS_MIN_AHEAD_S: f64 = 86_400.0;

/// Position/velocity reuse tolerances for the flight loop: tight enough that
/// a maneuver fails the check on its first step, loose enough that solver
/// noise between the 120 Hz stepper and the baked path never forces a rebake
/// on an unpowered coast.
pub const COAST_RAILS_POSITION_TOL_M: f64 = 5.0;
pub const COAST_RAILS_VELOCITY_TOL_MPS: f64 = 0.05;

/// Why a baked coast path ends: the wake condition the caller arms in
/// simulation time instead of polling every tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OnRailsWake {
    /// Predicted surface contact at this epoch (display fact from the test
    /// particle, not authoritative contact dynamics).
    Impact { time: SimTime, body: BodyId },
    /// Path ran to the configured horizon without impact.
    HorizonEnd { time: SimTime },
}

/// Baked unpowered coast: same gravity field and ephemerides as the live
/// simulation, sampled forward once. Querying is interpolation; integration
/// happens only on bake.
#[derive(Debug, Clone, Default)]
pub struct OnRailsCache {
    // Full immutable snapshot: names/count alone miss edited masses/orbits.
    ephemeris: Option<BakedEphemeris>,
    impact_bodies: Vec<BodyId>,
    config: Option<VerletConfig>,
    path: Option<SampledPath>,
}

impl OnRailsCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// True when no baked path is held.
    pub fn is_empty(&self) -> bool {
        self.path.is_none()
    }

    /// Drop the baked path. Call on any thrust, burn, regime exit from Coast,
    /// atmosphere entry, or contact event.
    pub fn invalidate(&mut self) {
        self.path = None;
    }

    /// Bake (or rebake) the coast path from this state and epoch. The
    /// `impact_bodies` order is normalized so caller ordering never changes
    /// the result. Uses the fast table-backed propagation: same impact
    /// semantics as the exact variant at a fraction of the Kepler solves
    /// (error bound pinned by `fast_bake_matches_exact_within_metres`).
    pub fn bake(
        &mut self,
        ephemeris: &BakedEphemeris,
        initial: TestParticleState,
        start: SimTime,
        config: VerletConfig,
        impact_bodies: &[BodyId],
    ) -> Result<&SampledPath, IntegratorError> {
        self.bake_impl(ephemeris, initial, start, config, impact_bodies, None)
    }

    /// Hitch-free entry bake: integrates only the head horizon synchronously
    /// (~2.8 h, milliseconds) so the flight loop rides the rails from the
    /// first coast tick, then grows coverage via [`extend`](Self::extend).
    /// Stores the full `config` as the sizing target, so reuse checks keep
    /// matching while the path grows underneath.
    pub fn bake_head(
        &mut self,
        ephemeris: &BakedEphemeris,
        initial: TestParticleState,
        start: SimTime,
        config: VerletConfig,
        impact_bodies: &[BodyId],
    ) -> Result<&SampledPath, IntegratorError> {
        let head = COAST_RAILS_HEAD_STEPS.min(config.max_steps.max(1));
        let mut headed = config;
        headed.max_steps = head;
        self.bake_impl(ephemeris, initial, start, headed, impact_bodies, Some(config))?;
        Ok(self.path.as_ref().expect("just baked"))
    }

    fn bake_impl(
        &mut self,
        ephemeris: &BakedEphemeris,
        initial: TestParticleState,
        start: SimTime,
        bake_config: VerletConfig,
        impact_bodies: &[BodyId],
        store_config: Option<VerletConfig>,
    ) -> Result<&SampledPath, IntegratorError> {
        let mut bodies = impact_bodies.to_vec();
        bodies.sort();
        bodies.dedup();
        // The table error regression covers 5 s integration / 40 s body
        // nodes. Coarse map forecasts can use 900 s steps: do not stretch
        // those table nodes to two hours and silently degrade ephemerides.
        let path = if bake_config.step_s <= COAST_RAILS_STEP_S {
            propagate_sampled_verlet_fast(
                ephemeris,
                initial,
                start,
                bake_config,
                &bodies,
                TABLE_NODE_EVERY_STEPS,
            )?
        } else {
            propagate_sampled_verlet(ephemeris, initial, start, bake_config, &bodies)?
        };
        self.ephemeris = Some(ephemeris.clone());
        self.impact_bodies = bodies;
        self.config = Some(store_config.unwrap_or(bake_config));
        self.path = Some(path);
        Ok(self.path.as_ref().expect("just baked"))
    }

    /// Grow a head-baked path toward its stored horizon by up to
    /// `extra_steps` samples. Returns `Ok(true)` when samples were appended
    /// (callers re-arm the wake: the end moved), `Ok(false)` when the path
    /// already ended in impact or reached the stored horizon.
    pub fn extend(
        &mut self,
        ephemeris: &BakedEphemeris,
        extra_steps: u64,
    ) -> Result<bool, IntegratorError> {
        let (config, bodies) = match (self.config, self.path.is_some()) {
            (Some(config), true) => (config, self.impact_bodies.clone()),
            _ => return Ok(false),
        };
        if self.ephemeris.as_ref() != Some(ephemeris) {
            return Ok(false);
        }
        let path = self.path.as_mut().expect("checked present");
        crate::propagate_sampled_extend(
            ephemeris,
            path,
            config.step_s,
            config.max_steps,
            extra_steps,
            &bodies,
            TABLE_NODE_EVERY_STEPS,
        )
    }

    /// True when coverage should grow: the baked end lies closer than
    /// `min_ahead_s` ahead of `now` and the stored horizon is not reached.
    /// Impact-ended paths never extend.
    pub fn needs_extension(&self, now: SimTime, min_ahead_s: f64) -> bool {
        let Some(path) = self.path.as_ref() else {
            return false;
        };
        if !matches!(path.end, crate::SampledPathEnd::Completed) {
            return false;
        }
        let target_max = self.config.map(|c| c.max_steps).unwrap_or(0);
        if path.stats.accepted_steps >= target_max {
            return false;
        }
        path.end_time.seconds() - now.seconds() < min_ahead_s
    }

    /// Latest covered epoch, if any path is held.
    pub fn covered_until(&self) -> Option<SimTime> {
        self.path.as_ref().map(|path| path.end_time)
    }

    /// Reuse check: the baked path covers `start` and its interpolated state
    /// there matches `initial` within tolerances. Time advancing along the
    /// same coast keeps matching — that is the on-rails win: no rebake per
    /// tick. A maneuver changes the state, the comparison fails, the caller
    /// rebakes. A different step config or impact set also forces a rebake so
    /// horizon sizing decisions stay with the caller.
    #[allow(clippy::too_many_arguments)]
    pub fn usable_for(
        &self,
        ephemeris: &BakedEphemeris,
        initial: TestParticleState,
        start: SimTime,
        config: VerletConfig,
        impact_bodies: &[BodyId],
        position_tolerance_m: f64,
        velocity_tolerance_mps: f64,
    ) -> bool {
        let Some(path) = self.path.as_ref() else {
            return false;
        };
        if self.ephemeris.as_ref() != Some(ephemeris)
            || !position_tolerance_m.is_finite()
            || position_tolerance_m < 0.0
            || !velocity_tolerance_mps.is_finite()
            || velocity_tolerance_mps < 0.0
        {
            return false;
        }
        if self.config != Some(config) {
            return false;
        }
        let mut wanted = impact_bodies.to_vec();
        wanted.sort();
        wanted.dedup();
        if self.impact_bodies != wanted {
            return false;
        }
        if start.seconds()
            < path
                .times
                .first()
                .map(|t| t.seconds())
                .unwrap_or(f64::INFINITY)
            || start.seconds() > path.end_time.seconds()
        {
            return false;
        }
        match self.sample_at(start) {
            Some((position, velocity)) => {
                position.distance(initial.position) <= position_tolerance_m
                    && velocity.distance(initial.velocity) <= velocity_tolerance_mps
            }
            None => false,
        }
    }

    /// Interpolated inertial (position, velocity) at this epoch, or `None`
    /// outside the baked coverage. Position uses cubic Hermite over the
    /// stored endpoint velocities (metre-grade at orbital speeds for the
    /// 5 s rail step, so the flight loop can sample translation from here);
    /// velocity is the analytic derivative of that same interpolant.
    /// This is a sampled trajectory, not a fresh integration.
    pub fn sample_at(&self, time: SimTime) -> Option<(DVec3, DVec3)> {
        if !time.0.is_finite() {
            return None;
        }
        let path = self.path.as_ref()?;
        let first = path.times.first()?.seconds();
        let t = time.seconds();
        if t < first || t > path.end_time.seconds() || path.positions.len() < 2 {
            // Exact-start single-sample paths (baked while already inside a
            // body) still answer their own epoch.
            if path.positions.len() == 1 && t == first {
                return Some((path.positions[0], path.velocities[0]));
            }
            return None;
        }
        let mut low = 0;
        let mut high = path.times.len() - 1;
        while high - low > 1 {
            let mid = (low + high) / 2;
            if path.times[mid].seconds() <= t {
                low = mid;
            } else {
                high = mid;
            }
        }
        let t0 = path.times[low].seconds();
        let t1 = path.times[high].seconds();
        let h = t1 - t0;
        let s = if h > 0.0 {
            ((t - t0) / h).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let p0 = path.positions[low];
        let p1 = path.positions[high];
        let v0 = path.velocities[low];
        let v1 = path.velocities[high];
        Some(crate::table::hermite_state(p0, v0, p1, v1, h, s))
    }

    /// Conservative axis-aligned bounds of the Hermite curve, including
    /// between-node excursions. Cubic Bezier control points enclose a segment;
    /// testing endpoints alone can miss an atmosphere/body crossing.
    pub fn position_bounds(&self, start: SimTime, end: SimTime) -> Option<(DVec3, DVec3)> {
        if end.0 < start.0 { return None; }
        let (p, _) = self.sample_at(start)?;
        self.sample_at(end)?;
        let path = self.path.as_ref()?;
        let mut min = p;
        let mut max = p;
        let mut t = start;
        let mut next = path.times.partition_point(|node| node.0 <= start.0);
        while t.0 < end.0 {
            let finish = SimTime(path.times.get(next).map_or(end.0, |node| node.0.min(end.0)));
            let (p0, v0) = self.sample_at(t)?;
            let (p1, v1) = self.sample_at(finish)?;
            let h = (finish.0 - t.0) / 3.0;
            for point in [p0, p0 + v0 * h, p1 - v1 * h, p1] {
                min = min.min(point);
                max = max.max(point);
            }
            t = finish;
            next += 1;
        }
        Some((min, max))
    }

    /// Wake condition at the end of the baked path: arm this epoch in the
    /// event scheduler instead of polling the trajectory every tick.
    pub fn wake(&self) -> Option<OnRailsWake> {
        let path = self.path.as_ref()?;
        match path.end {
            crate::SampledPathEnd::Impact(body) => Some(OnRailsWake::Impact {
                time: path.end_time,
                body,
            }),
            crate::SampledPathEnd::Completed => Some(OnRailsWake::HorizonEnd {
                time: path.end_time,
            }),
        }
    }

    /// The baked path itself, for line rendering and horizon checks.
    pub fn path(&self) -> Option<&SampledPath> {
        self.path.as_ref()
    }
}
