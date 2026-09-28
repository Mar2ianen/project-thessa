//! Deterministic gravity propagation on the world's 120 Hz time lattice.
//! Accepted step spans are 1, 2, 4, ... ticks. RK stages sample the field
//! between ticks; they do not advance the world clock. Step decisions use
//! numeric error and collision bounds only, never frame time or worker load.
use crate::integrator::{impact_at, impact_segment};
use crate::{
    BakedEphemeris, BodyId, GravityField, IntegratorError, IntegratorStats, SampledPath,
    SampledPathEnd, SimTime, TestParticleState,
};
use glam::DVec3;

use crate::{WORLD_TICK_HZ, WORLD_TICK_S};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TickIntegratorConfig {
    /// Exact duration from the starting epoch, expressed in whole world ticks.
    pub duration_ticks: u64,
    pub initial_step_ticks: u64,
    pub max_step_ticks: u64,
    /// Local RK4 step-doubling error budgets; not a global error guarantee.
    pub position_tolerance_m: f64,
    pub velocity_tolerance_mps: f64,
    /// Midpoint consistency budget for the cubic consumed by flight and map.
    pub interpolation_tolerance_m: f64,
    pub interpolation_velocity_tolerance_mps: f64,
    pub max_attempts: u64,
}
impl Default for TickIntegratorConfig {
    fn default() -> Self {
        Self {
            duration_ticks: 600_000 * WORLD_TICK_HZ,
            initial_step_ticks: 64,
            max_step_ticks: 524_288,
            position_tolerance_m: 0.01,
            velocity_tolerance_mps: 1e-5,
            interpolation_tolerance_m: 0.05,
            interpolation_velocity_tolerance_mps: 0.001,
            max_attempts: 200_000,
        }
    }
}
impl TickIntegratorConfig {
    fn validate(self) -> Result<(), IntegratorError> {
        if !self.initial_step_ticks.is_power_of_two()
            || !self.max_step_ticks.is_power_of_two()
            || self.initial_step_ticks > self.max_step_ticks
            || self.duration_ticks > (1_u64 << 52)
            || self.max_step_ticks > (1_u64 << 52)
            || self.max_attempts == 0
            || [
                self.position_tolerance_m,
                self.velocity_tolerance_mps,
                self.interpolation_tolerance_m,
                self.interpolation_velocity_tolerance_mps,
            ]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(IntegratorError::InvalidConfig(
                "invalid tick-adaptive budget".into(),
            ));
        }
        Ok(())
    }
}

// Sum increments before adding the barycentric origin once. Repeatedly
// adding small increments to a ~1e12 m coordinate loses low bits needlessly.
/// One RK4 step plus the endpoint acceleration, so bakes can store it for
/// rounding-safe velocity sampling without an extra field evaluation.
fn rk4(
    field: &GravityField<'_>,
    state: TestParticleState,
    time: SimTime,
    h: f64,
) -> Result<(TestParticleState, DVec3), IntegratorError> {
    let a1 = field.acceleration(state.position, time)?;
    let v2 = state.velocity + a1 * (h * 0.5);
    let a2 = field.acceleration(
        state.position + state.velocity * (h * 0.5),
        time.offset(h * 0.5),
    )?;
    let v3 = state.velocity + a2 * (h * 0.5);
    let a3 = field.acceleration(state.position + v2 * (h * 0.5), time.offset(h * 0.5))?;
    let v4 = state.velocity + a3 * h;
    let a4 = field.acceleration(state.position + v3 * h, time.offset(h))?;
    Ok((
        TestParticleState {
            position: state.position + (state.velocity + v2 * 2.0 + v3 * 2.0 + v4) * (h / 6.0),
            velocity: state.velocity + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (h / 6.0),
        },
        a4,
    ))
}

/// Bake one gravity-only trajectory; map and flight sample these same nodes.
/// Error rejection halves the step. Growth doubles it only below 1/32 of
/// both local budgets and 1/16 of the interpolation budget (hysteresis).
/// Events/endpoints split dyadic spans. Contact brackets refine to one tick;
/// the final sphere-entry time can be fractional, as in other sampled paths.
pub fn propagate_tick_adaptive(
    ephemeris: &BakedEphemeris,
    initial: TestParticleState,
    start: SimTime,
    config: TickIntegratorConfig,
    impact_bodies: &[BodyId],
) -> Result<SampledPath, IntegratorError> {
    config.validate()?;
    if !initial.position.is_finite()
        || !initial.velocity.is_finite()
        || !start.0.is_finite()
        || !(start.0 + config.duration_ticks as f64 * WORLD_TICK_S).is_finite()
        || (config.duration_ticks > 0 && start.offset(WORLD_TICK_S) == start)
    {
        return Err(IntegratorError::InvalidConfig(
            "non-finite or unresolvable tick epoch".into(),
        ));
    }
    let mut contact_bounds = Vec::new();
    for id in impact_bodies {
        let body = ephemeris
            .body(*id)
            .map_err(|e| IntegratorError::InvalidConfig(e.to_string()))?;
        let accel = ephemeris
            .maximum_body_acceleration(*id)
            .map_err(|e| IntegratorError::InvalidConfig(e.to_string()))?;
        if body.radius_m > 0.0 {
            contact_bounds.push((*id, body.radius_m, accel));
        }
    }
    let field = GravityField::from_ephemeris(ephemeris);
    let mut path = SampledPath {
        positions: vec![initial.position],
        velocities: vec![initial.velocity],
        accelerations: vec![field.acceleration(initial.position, start)?],
        times: vec![start],
        end_time: start,
        end: SampledPathEnd::Completed,
        stats: IntegratorStats::default(),
    };
    if let Some(body) = impact_at(ephemeris, impact_bodies, initial.position, start) {
        path.end = SampledPathEnd::Impact(body);
        return Ok(path);
    }
    let mut state = initial;
    let mut tick = 0_u64;
    let mut preferred = config.initial_step_ticks;
    while tick < config.duration_ticks {
        if path.stats.accepted_steps + path.stats.rejected_steps >= config.max_attempts {
            return Err(IntegratorError::MaxSteps);
        }
        let remaining = config.duration_ticks - tick;
        let mut ticks = preferred.min(1 << remaining.ilog2());
        // Growth respects previously accepted tick boundaries, making the
        // sequence independent of how often the renderer asks for samples.
        if tick != 0 {
            ticks = ticks.min(1 << tick.trailing_zeros());
        }
        let h = ticks as f64 * WORLD_TICK_S;
        let time = start.offset(tick as f64 * WORLD_TICK_S);
        let next_time = start.offset((tick + ticks) as f64 * WORLD_TICK_S);
        let candidate = (|| {
            let (coarse, _) = rk4(&field, state, time, h)?;
            let (middle, _) = rk4(&field, state, time, h * 0.5)?;
            let (fine, fine_accel) = rk4(&field, middle, time.offset(h * 0.5), h * 0.5)?;
            Ok::<_, IntegratorError>((coarse, middle, fine, fine_accel))
        })();
        let (coarse, middle, fine, fine_accel) = match candidate {
            Ok(value) => value,
            Err(_) if ticks > 1 => {
                path.stats.rejected_steps += 1;
                preferred = ticks / 2;
                continue;
            }
            Err(error) => return Err(error),
        };
        let local_error = ((fine.position - coarse.position).abs().max_element()
            / (15.0 * config.position_tolerance_m))
            .max(
                (fine.velocity - coarse.velocity).abs().max_element()
                    / (15.0 * config.velocity_tolerance_mps),
            );
        // Midpoint velocity comes from the stored-acceleration Hermite, not
        // from differentiating positions: at ~1e12 m barycentric coordinates
        // f64 rounding is ~1e-3 m, and the position-Hermite derivative
        // amplifies it by ~1/h into 0.01-0.1 m/s of pure noise. Acceleration
        // data is rounding-clean by construction.
        let (curve_mid, _) = crate::table::hermite_state(
            state.position,
            state.velocity,
            fine.position,
            fine.velocity,
            h,
            0.5,
        );
        let curve_velocity = crate::table::hermite_velocity(
            state.velocity,
            path.accelerations.last().copied().unwrap_or(DVec3::ZERO),
            fine.velocity,
            fine_accel,
            h,
            0.5,
        );
        let interpolation_error =
            (curve_mid.distance(middle.position) / config.interpolation_tolerance_m).max(
                curve_velocity.distance(middle.velocity)
                    / config.interpolation_velocity_tolerance_mps,
            );
        let mut near_contact = false;
        for &(id, radius, accel) in &contact_bounds {
            let c0 = ephemeris
                .body_state(id, time)
                .map_err(|e| IntegratorError::InvalidConfig(e.to_string()))?
                .position_inertial;
            let c1 = ephemeris
                .body_state(id, next_time)
                .map_err(|e| IntegratorError::InvalidConfig(e.to_string()))?
                .position_inertial;
            let controls = [
                state.position - c0,
                state.position + state.velocity * (h / 3.0) - c0.lerp(c1, 1.0 / 3.0),
                fine.position - fine.velocity * (h / 3.0) - c0.lerp(c1, 2.0 / 3.0),
                fine.position - c1,
            ];
            let mut min = controls[0];
            let mut max = controls[0];
            for point in controls {
                min = min.min(point);
                max = max.max(point);
            }
            // A bounded-acceleration body's deviation from its endpoint
            // chord is <= a_max*h²/8. The craft's cubic lies in this hull.
            near_contact |= DVec3::ZERO.clamp(min, max).length()
                <= radius + accel * h * h / 8.0 + config.interpolation_tolerance_m;
        }
        if !local_error.is_finite()
            || !interpolation_error.is_finite()
            || local_error > 1.0
            || interpolation_error > 1.0
            || (near_contact && ticks > 1)
        {
            if ticks == 1 {
                return Err(IntegratorError::StepUnderflow { step_s: h });
            }
            preferred = ticks / 2;
            path.stats.rejected_steps += 1;
            continue;
        }
        if near_contact
            && let Some((body, fraction)) = impact_segment(
                ephemeris,
                impact_bodies,
                state.position,
                fine.position,
                time,
                next_time,
            )
        {
            path.positions
                .push(state.position.lerp(fine.position, fraction));
            path.velocities
                .push(state.velocity.lerp(fine.velocity, fraction));
            path.accelerations
                .push(path.accelerations.last().copied().unwrap_or(DVec3::ZERO));
            path.end_time = time.offset(h * fraction);
            path.times.push(path.end_time);
            path.stats.accepted_steps += 1;
            path.end = SampledPathEnd::Impact(body);
            return Ok(path);
        }
        state = fine;
        tick += ticks;
        path.positions.push(state.position);
        path.velocities.push(state.velocity);
        path.accelerations.push(fine_accel);
        path.times.push(next_time);
        path.end_time = next_time;
        path.stats.accepted_steps += 1;
        preferred = if local_error < 1.0 / 32.0 && interpolation_error < 1.0 / 16.0 {
            (ticks * 2).min(config.max_step_ticks)
        } else {
            ticks
        };
    }
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BakedBody, OnRailsCache};

    fn center(mu: f64, radius: f64, gravity: bool) -> BakedEphemeris {
        let mut body = BakedBody::fixed(BodyId(0), "test", mu, radius);
        body.gravity_source = gravity;
        BakedEphemeris::new("test", vec![body]).unwrap()
    }

    #[test]
    fn straight_motion_grows_on_ticks_and_stops_at_an_odd_event_tick() {
        let ephemeris = center(1.0, 0.0, false);
        let initial = TestParticleState {
            position: DVec3::X * 1000.0,
            velocity: DVec3::Y * 20.0,
        };
        let config = TickIntegratorConfig {
            duration_ticks: 1003,
            initial_step_ticks: 1,
            max_step_ticks: 128,
            ..Default::default()
        };
        let start = SimTime(123.45);
        let path = propagate_tick_adaptive(&ephemeris, initial, start, config, &[]).unwrap();
        assert_eq!(path.end_time, start.offset(1003.0 * WORLD_TICK_S));
        assert!(path.stats.accepted_steps < 30);
        let mut grew = false;
        for pair in path.times.windows(2) {
            let span = (pair[1].0 - pair[0].0) * WORLD_TICK_HZ as f64;
            assert!((span - span.round()).abs() < 1e-8);
            assert!((span.round() as u64).is_power_of_two());
            grew |= span.round() == 128.0;
        }
        assert!(grew);
        let expected = initial.position + initial.velocity * (1003.0 * WORLD_TICK_S);
        assert!(path.positions.last().unwrap().distance(expected) < 1e-9);
    }

    #[test]
    fn curvature_rejects_large_steps_and_replay_is_worker_independent() {
        let ephemeris = center(4e13, 3.2e6, true);
        let initial = TestParticleState {
            position: DVec3::X * 1e7,
            velocity: DVec3::Y * 2000.0,
        };
        let config = TickIntegratorConfig {
            duration_ticks: 60_000,
            initial_step_ticks: 32768,
            max_step_ticks: 32768,
            ..Default::default()
        };
        let bake = || {
            propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[BodyId(0)])
                .unwrap()
        };
        let a = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap()
            .install(bake);
        let b = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            .build()
            .unwrap()
            .install(bake);
        assert_eq!(a, b);
        assert!(a.stats.rejected_steps > 0);
        assert!(
            a.times
                .windows(2)
                .any(|t| t[1].0 - t[0].0 < 32768.0 * WORLD_TICK_S)
        );
    }

    #[test]
    fn circle_stays_accurate_at_nodes_and_between_them() {
        let ephemeris = center(4e13, 3.2e6, true);
        let initial = TestParticleState {
            position: DVec3::X * 1e7,
            velocity: DVec3::Y * 2000.0,
        };
        let config = TickIntegratorConfig {
            duration_ticks: 200_000 * WORLD_TICK_HZ,
            ..Default::default()
        };
        let mut cache = OnRailsCache::new();
        cache
            .bake_tick(&ephemeris, initial, SimTime::EPOCH, config, &[BodyId(0)])
            .unwrap();
        let path = cache.path().unwrap();
        let (mut max_p, mut max_v, mut max_energy) = (0.0_f64, 0.0_f64, 0.0_f64);
        for span in path.times.windows(2) {
            for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let t = span[0].0 + (span[1].0 - span[0].0) * fraction;
                let (p, v) = cache.sample_at(SimTime(t)).unwrap();
                let angle = 2000.0 / 1e7 * t;
                max_p = max_p.max(p.distance(DVec3::new(angle.cos(), angle.sin(), 0.0) * 1e7));
                max_v = max_v.max(v.distance(DVec3::new(-angle.sin(), angle.cos(), 0.0) * 2000.0));
                max_energy = max_energy
                    .max(((v.length_squared() * 0.5 - 4e13 / p.length()) / -2e6 - 1.0).abs());
            }
        }
        eprintln!(
            "tick circle: max position {max_p} m, velocity {max_v} m/s, relative energy {max_energy}"
        );
        assert!(max_p < 1.0);
        assert!(max_v < 0.002);
        assert!(max_energy < 2e-6);
        assert!(path.stats.accepted_steps < 4000);
    }

    #[test]
    fn contact_refines_even_in_constant_zero_field() {
        let ephemeris = center(1.0, 5.0, false);
        let initial = TestParticleState {
            position: DVec3::X * 10.0,
            velocity: -DVec3::X * 20.0,
        };
        let config = TickIntegratorConfig {
            duration_ticks: 120,
            ..Default::default()
        };
        let path =
            propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[BodyId(0)])
                .unwrap();
        assert_eq!(path.end, SampledPathEnd::Impact(BodyId(0)));
        assert!((path.end_time.0 - 0.25).abs() < 1e-10);
        assert!(path.positions.last().unwrap().distance(DVec3::X * 5.0) < 1e-10);
    }

    #[test]
    fn invalid_budgets_and_exhausted_attempts_fail_explicitly() {
        let ephemeris = center(1.0, 0.0, false);
        let initial = TestParticleState {
            position: DVec3::X,
            velocity: DVec3::Y,
        };
        let config = TickIntegratorConfig {
            max_step_ticks: 3,
            ..Default::default()
        };
        assert!(propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[]).is_err());
        let config = TickIntegratorConfig {
            duration_ticks: 0,
            ..Default::default()
        };
        assert_eq!(
            propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[])
                .unwrap()
                .times,
            vec![SimTime::EPOCH]
        );
        let config = TickIntegratorConfig {
            max_attempts: 1,
            ..Default::default()
        };
        assert!(matches!(
            propagate_tick_adaptive(&ephemeris, initial, SimTime::EPOCH, config, &[]),
            Err(IntegratorError::MaxSteps)
        ));
    }
}
