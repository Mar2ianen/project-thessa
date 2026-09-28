//! Phase laws for the executing authority.
//!
//! The graph IR (profiles embedded in phase nodes) is executable data, but
//! a parked phase steers nothing by itself. These pure functions close that
//! gap: given a phase node plus live vehicle state they return the tick's
//! guidance command and whether the phase's completion predicate tripped.
//! The server applies the command every tick, publishes the node-name and
//! domain events on `done`, and enforces the watchdog from
//! [`phase_watchdog_s`].
//!
//! Laws never touch flight state and never panic on degenerate input:
//! zero vectors and non-finite telemetry degrade to an attitude hold with
//! zero propulsion (the watchdog then bounds the hang). Mass properties
//! stay with the authority — laws command directions and throttle only.

use glam::{DQuat, DVec3};
use thessa_flight_control::{
    DirectionFrame, DirectionTarget, GuidanceIntent, PropulsionDemand, RollPolicy,
};

use crate::{
    GraphNodeConfig,
    ascent::{self, AscentPhase},
    execute::ExecutePhase,
    landing::{self, LandingPhase},
    rendezvous::{self, RendezvousPhase},
};

/// Tower-clearance altitude (m): vertical rise ends here. A launch mount
/// is order-100 m; the exact structure is not modeled, so this is a
/// documented constant, not a profile field.
pub const TOWER_CLEARANCE_M: f64 = 150.0;

/// Touchdown declaration altitude (m) over the reference sphere. Landing
/// legs and terrain relief are not modeled in v1; the spherical
/// approximation is documented at the call site.
pub const TOUCHDOWN_ALTITUDE_M: f64 = 3.0;

/// Braking-gate margin (m) when the law evaluates the suicide distance.
/// The graph waits carry the profile margin separately; the law keeps a
/// fixed standoff so ignition never starts inside the gate.
pub const BRAKING_GATE_MARGIN_M: f64 = 500.0;

/// Back-away completion range as a multiple of the hold distance.
pub const BACK_AWAY_RANGE_FACTOR: f64 = 2.0;

/// Coast handoff lead (m/s): retire the coast while still ascending this
/// slowly, so the circularization arc centers on apoapsis instead of
/// starting past it and pumping the apoapsis one-sidedly.
pub const COAST_HANDOFF_CLIMB_MPS: f64 = 10.0;

/// MECO flight-path gate: cutoff needs predicted apoapsis AND a nearly
/// level trajectory. Cutting off on apoapsis alone fires on steep lobs
/// (energy high, horizontal velocity missing) and strands the coast at
/// apoapsis with nothing to circularize from.
pub const MECO_FLIGHT_PATH_SIN_LIMIT: f64 = 0.34;

/// Live vehicle state sampled by the authority each tick. Positions and
/// velocities are BODY-centered inertial (reference-body motion already
/// subtracted — feeding star-centered inertial state reads interplanetary
/// distances as altitude); `target` serves the rendezvous phases only
/// (`None` degrades them to hold-and-watchdog — single-vehicle servers
/// have no target to close with).
#[derive(Debug, Clone, Copy)]
pub struct LiveState {
    pub position_m: DVec3,
    pub velocity_mps: DVec3,
    pub orientation_body_to_inertial: DQuat,
    pub body_mu_m3_s2: f64,
    pub body_radius_m: f64,
    pub target: Option<TargetState>,
}

/// Target-relative state for rendezvous phases.
#[derive(Debug, Clone, Copy)]
pub struct TargetState {
    pub position_m: DVec3,
    pub velocity_mps: DVec3,
}

/// One tick of phase law output: what to command now, and whether the
/// completion predicate tripped (the caller then publishes the node-name
/// event plus [`completion_event`]).
#[derive(Debug, Clone)]
pub struct PhaseTick {
    pub intent: GuidanceIntent,
    pub propulsion: PropulsionDemand,
    pub done: bool,
}

/// Zero-throttle propulsion, infallible by construction.
fn propulsion_zero() -> PropulsionDemand {
    PropulsionDemand { normalized: 0.0 }
}

/// Attitude hold on the current orientation with zero propulsion: the
/// universal safe output for degenerate input and arrival latches.
fn hold_tick(state: &LiveState) -> PhaseTick {
    PhaseTick {
        intent: GuidanceIntent::Attitude {
            target_body_to_inertial: state.orientation_body_to_inertial,
            roll_policy: RollPolicy::Hold,
        },
        propulsion: propulsion_zero(),
        done: false,
    }
}

/// Throttle clamped into the propulsion range; non-finite falls back to
/// zero (node validation already passed at build, this is belt-and-braces
/// for hand-built phases).
fn propulsion_or_zero(throttle: f64) -> PropulsionDemand {
    if throttle.is_finite() {
        PropulsionDemand::new(throttle.clamp(-0.2, 1.2)).unwrap_or(propulsion_zero())
    } else {
        propulsion_zero()
    }
}

/// Velocity-direction steering, degrading to hold on degenerate vectors.
fn steer_toward(direction: DVec3, throttle: f64, state: &LiveState) -> PhaseTick {
    match DirectionTarget::new(direction, DirectionFrame::Inertial) {
        Ok(target) => PhaseTick {
            intent: GuidanceIntent::VelocityDirection {
                direction: target,
                roll_policy: RollPolicy::Hold,
            },
            propulsion: propulsion_or_zero(throttle),
            done: false,
        },
        Err(_) => hold_tick(state),
    }
}

fn radial(state: &LiveState) -> Option<DVec3> {
    let length = state.position_m.length();
    if length.is_finite() && length > 1.0e-6 {
        Some(state.position_m / length)
    } else {
        None
    }
}

fn altitude_m(state: &LiveState) -> Option<f64> {
    let altitude = state.position_m.length() - state.body_radius_m;
    altitude.is_finite().then_some(altitude)
}

/// Prograde direction: velocity with the radial component removed, so the
/// pitch program measures above the local horizon. When velocity is radial
/// (or zero) there is no motion-defined horizontal — the profile carries
/// no target plane, so any plane satisfies it, and the law bootstraps
/// from a deterministic reference instead of locking radial forever
/// (falling back to raw velocity would preserve a radial vector exactly
/// and the turn would never turn). Inclination targeting is future IR.
fn prograde_horizontal(state: &LiveState) -> Option<DVec3> {
    let up = radial(state)?;
    let horizontal = state.velocity_mps - up * state.velocity_mps.dot(up);
    if horizontal.length_squared() > 1.0e-6 {
        return Some(horizontal.normalize());
    }
    if state.velocity_mps.length_squared() > 1.0 {
        return Some(reference_horizontal(up));
    }
    None
}

/// Deterministic horizontal reference for plane-less steering.
fn reference_horizontal(up: DVec3) -> DVec3 {
    let candidate = up.cross(DVec3::X);
    if candidate.length_squared() > 1.0e-6 {
        return candidate.normalize();
    }
    up.cross(DVec3::Y).normalize()
}

/// Horizontal prograde in the osculating orbital plane: perpendicular to
/// the radius, signed along the motion. Unlike instantaneous prograde it
/// stays correct for steep arrivals — burning along a near-radial velocity
/// vector escapes instead of circularizing. `None` when no orbital plane
/// exists yet (pure radial or zero velocity).
fn orbital_horizontal(state: &LiveState) -> Option<DVec3> {
    let up = radial(state)?;
    let normal = state.position_m.cross(state.velocity_mps);
    if normal.length_squared() <= 1.0e-12 {
        return None;
    }
    let direction = normal.cross(up).normalize();
    Some(if direction.dot(state.velocity_mps) >= 0.0 {
        direction
    } else {
        -direction
    })
}

/// Completion domain event for a parked phase node, if the phase carries
/// one. The caller publishes the node-name event (completes the phase)
/// followed by this (wakes the downstream domain wait) on `done`.
pub fn completion_event(config: &GraphNodeConfig) -> Option<&'static str> {
    match config {
        GraphNodeConfig::AscentPhase { phase } => match phase {
            AscentPhase::VerticalRise { .. } => Some(ascent::event::TOWER_CLEARED),
            AscentPhase::GravityTurn { .. } => Some(ascent::event::MECO),
            AscentPhase::Coast { .. } => Some(ascent::event::APOAPSIS_APPROACH),
            AscentPhase::Circularize { .. } | AscentPhase::Abort { .. } => None,
        },
        GraphNodeConfig::LandingPhase { phase } => match phase {
            LandingPhase::DeorbitBurn { .. } => Some(landing::event::ENTRY_INTERFACE),
            LandingPhase::BrakingBurn { .. } => Some(landing::event::TOUCHDOWN_APPROACH),
            LandingPhase::TerminalDescent { .. } => Some(landing::event::TOUCHDOWN),
            LandingPhase::CoastToEntry { .. } => Some(landing::event::ENTRY_INTERFACE),
            LandingPhase::AbortToOrbit { .. } => None,
        },
        GraphNodeConfig::RendezvousPhase { phase } => match phase {
            RendezvousPhase::Approach { .. } => Some(rendezvous::event::PROXIMITY),
            RendezvousPhase::MatchVelocity { .. } => Some(rendezvous::event::VELOCITY_MATCHED),
            RendezvousPhase::StationKeep { .. } => Some(rendezvous::event::KEEP_STABLE),
            RendezvousPhase::BackAway { .. } => None,
        },
        GraphNodeConfig::ExecutePhase { .. } => None,
        _ => None,
    }
}

/// Watchdog bound carried by a parked phase node, if any.
pub fn phase_watchdog_s(config: &GraphNodeConfig) -> Option<f64> {
    let seconds = match config {
        GraphNodeConfig::AscentPhase { phase } => match phase {
            AscentPhase::VerticalRise {
                max_phase_time_s, ..
            }
            | AscentPhase::GravityTurn {
                max_phase_time_s, ..
            }
            | AscentPhase::Coast {
                max_phase_time_s, ..
            }
            | AscentPhase::Circularize {
                max_phase_time_s, ..
            }
            | AscentPhase::Abort { max_phase_time_s } => *max_phase_time_s,
        },
        GraphNodeConfig::LandingPhase { phase } => match phase {
            LandingPhase::DeorbitBurn {
                max_phase_time_s, ..
            }
            | LandingPhase::CoastToEntry {
                max_phase_time_s, ..
            }
            | LandingPhase::BrakingBurn {
                max_phase_time_s, ..
            }
            | LandingPhase::TerminalDescent {
                max_phase_time_s, ..
            }
            | LandingPhase::AbortToOrbit { max_phase_time_s } => *max_phase_time_s,
        },
        GraphNodeConfig::RendezvousPhase { phase } => match phase {
            RendezvousPhase::Approach {
                max_phase_time_s, ..
            }
            | RendezvousPhase::MatchVelocity {
                max_phase_time_s, ..
            }
            | RendezvousPhase::StationKeep {
                max_phase_time_s, ..
            }
            | RendezvousPhase::BackAway {
                max_phase_time_s, ..
            } => *max_phase_time_s,
        },
        GraphNodeConfig::ExecutePhase { phase } => match phase {
            ExecutePhase::Arm {
                max_phase_time_s, ..
            }
            | ExecutePhase::Burn {
                max_phase_time_s, ..
            }
            | ExecutePhase::Verify {
                max_phase_time_s, ..
            }
            | ExecutePhase::Abort { max_phase_time_s } => *max_phase_time_s,
        },
        _ => return None,
    };
    seconds.is_finite().then_some(seconds)
}

/// Ascent phase law: steer each tick, report completion.
pub fn ascent_tick(phase: &AscentPhase, state: &LiveState) -> PhaseTick {
    let mu = state.body_mu_m3_s2;
    match phase {
        AscentPhase::VerticalRise { throttle, .. } => {
            let done = altitude_m(state).is_some_and(|alt| alt >= TOWER_CLEARANCE_M);
            match radial(state) {
                Some(up) => {
                    let mut tick = steer_toward(up, *throttle, state);
                    tick.done = done;
                    tick
                }
                None => hold_tick(state),
            }
        }
        AscentPhase::GravityTurn {
            turn_start_altitude_m,
            turn_end_altitude_m,
            target_apoapsis_m,
            ..
        } => {
            let altitude = altitude_m(state).unwrap_or(0.0);
            let span = (*turn_end_altitude_m - *turn_start_altitude_m).max(1.0);
            let t = ((altitude - *turn_start_altitude_m) / span).clamp(0.0, 1.0);
            let pitch =
                std::f64::consts::FRAC_PI_2 * 0.5 * (1.0 + (std::f64::consts::PI * t).cos());
            let direction = match (radial(state), prograde_horizontal(state)) {
                (Some(up), Some(prograde)) => {
                    (up * pitch.sin() + prograde * pitch.cos()).normalize()
                }
                (Some(up), None) => up,
                _ => return hold_tick(state),
            };
            // MECO is predicted apoapsis, never a clock: cut off when the
            // osculating orbit reaches the target on a level trajectory.
            let speed_sq = state.velocity_mps.length_squared();
            let climb_sin = if speed_sq > 1.0 {
                state.position_m.dot(state.velocity_mps)
                    / (state.position_m.length() * speed_sq.sqrt())
            } else {
                1.0
            };
            let done = ascent::apoapsis_reached(
                mu,
                state.position_m,
                state.velocity_mps,
                state.body_radius_m + *target_apoapsis_m,
            ) && climb_sin.abs() <= MECO_FLIGHT_PATH_SIN_LIMIT;
            let mut tick = steer_toward(direction, 1.0, state);
            tick.done = done;
            tick
        }
        AscentPhase::Coast { .. } => {
            // Unpowered drift onto the circularization direction: a hot
            // engine with a slewing nose wastes the burn in cosine
            // losses. Retire inside a narrow band around apoapsis —
            // climbing no faster than the handoff limit and not yet
            // descending past it either: a one-sided `climb <= L` also
            // fires during a near-vertical fall, handing circularize
            // below apoapsis. A shattered ascent that misses the band
            // hangs here for the watchdog — that is the honest signal,
            // not a coasted crash.
            let radial_momentum = state.position_m.dot(state.velocity_mps);
            let done = radial_momentum.abs() <= COAST_HANDOFF_CLIMB_MPS * state.position_m.length();
            let mut tick = match orbital_horizontal(state) {
                Some(direction) => steer_toward(direction, 0.0, state),
                None => hold_tick(state),
            };
            tick.done = done;
            tick
        }
        AscentPhase::Circularize {
            target_periapsis_m, ..
        } => {
            let done = ascent::predict_periapsis_m(mu, state.position_m, state.velocity_mps)
                .is_some_and(|periapsis| periapsis >= state.body_radius_m + *target_periapsis_m);
            if done {
                let mut tick = hold_tick(state);
                tick.done = true;
                return tick;
            }
            // Horizontal in the orbital plane, not instantaneous prograde:
            // steep arrivals have near-radial velocity, and burning along
            // it escapes instead of circularizing. No apoapsis gate: the
            // coast hands over near apoapsis by construction, and a gate
            // deadlocks when drag decays the apoapsis between handoff and
            // burn. The burn stops on periapsis, so overshoot is bounded
            // by the remaining raise.
            match orbital_horizontal(state) {
                Some(direction) => {
                    let mut tick = steer_toward(direction, 1.0, state);
                    tick.done = false;
                    tick
                }
                _ => hold_tick(state),
            }
        }
        AscentPhase::Abort { .. } => {
            let mut tick = hold_tick(state);
            tick.done = true;
            tick
        }
    }
}

/// Landing phase law: steer each tick, report completion.
pub fn landing_tick(phase: &LandingPhase, state: &LiveState) -> PhaseTick {
    let altitude = altitude_m(state).unwrap_or(0.0);
    match phase {
        LandingPhase::DeorbitBurn {
            throttle,
            entry_altitude_m,
            ..
        } => {
            // Retrograde burn until the osculating periapsis drops to the
            // entry interface: the burn achieved its targeting goal. The
            // downstream entry wait fires on current altitude; between the
            // two the completion latch holds unpowered attitude.
            let done = ascent::predict_periapsis_m(
                state.body_mu_m3_s2,
                state.position_m,
                state.velocity_mps,
            )
            .is_some_and(|periapsis| periapsis <= state.body_radius_m + *entry_altitude_m);
            if state.velocity_mps.length_squared() > 1.0 {
                let mut tick = steer_toward(-state.velocity_mps.normalize(), *throttle, state);
                tick.done = done;
                tick
            } else {
                let mut tick = hold_tick(state);
                tick.done = done;
                tick
            }
        }
        LandingPhase::CoastToEntry {
            entry_altitude_m, ..
        } => {
            let done = altitude <= *entry_altitude_m;
            let mut tick = hold_tick(state);
            tick.done = done;
            tick
        }
        LandingPhase::BrakingBurn {
            net_braking_decel_mps2,
            touchdown_speed_limit_mps,
            burn_throttle,
            ..
        } => {
            // Coast until the suicide gate opens, then burn retrograde
            // until the descent rate is inside the touchdown limit. The
            // downstream approach wait refires on the same gate, so the
            // completion double-publish hands over cleanly.
            let up = radial(state).unwrap_or(DVec3::Y);
            let descent_rate = -state.velocity_mps.dot(up);
            let gate_open = landing::braking_distance_m(
                descent_rate,
                *touchdown_speed_limit_mps,
                *net_braking_decel_mps2,
            )
            .is_some_and(|distance| {
                altitude_m(state).is_some_and(|alt| alt <= distance + BRAKING_GATE_MARGIN_M)
            });
            let mut tick = if gate_open && state.velocity_mps.length_squared() > 1.0 {
                steer_toward(-state.velocity_mps.normalize(), *burn_throttle, state)
            } else {
                hold_tick(state)
            };
            tick.done = gate_open && descent_rate <= *touchdown_speed_limit_mps;
            tick
        }
        LandingPhase::TerminalDescent {
            touchdown_speed_limit_mps,
            burn_throttle,
            ..
        } => {
            // Bang-bang around the touchdown speed: thrust must oppose
            // the fall, so the nose points along `-down` (up) — body +X
            // carries the thrust, and steering along `down` would fire
            // the engine straight into the ground and accelerate the
            // descent. Touchdown declares only inside the altitude gate
            // *and* with the descent rate inside the limit, so a hot
            // impact keeps braking instead of reporting a success.
            let down = radial(state).map(|up| -up).unwrap_or(DVec3::NEG_Y);
            let descent_rate = -state.velocity_mps.dot(-down);
            let throttle = if descent_rate > *touchdown_speed_limit_mps {
                *burn_throttle
            } else {
                0.0
            };
            let mut tick = steer_toward(-down, throttle, state);
            tick.done =
                altitude <= TOUCHDOWN_ALTITUDE_M && descent_rate <= *touchdown_speed_limit_mps;
            tick
        }
        LandingPhase::AbortToOrbit { .. } => {
            let mut tick = hold_tick(state);
            tick.done = true;
            tick
        }
    }
}

/// Rendezvous phase law. Without a target the phases cannot execute:
/// hold with zero propulsion and let the watchdog bound the hang.
pub fn rendezvous_tick(phase: &RendezvousPhase, state: &LiveState) -> PhaseTick {
    let Some(target) = state.target else {
        return hold_tick(state);
    };
    let relative = target.position_m - state.position_m;
    let relative_velocity = target.velocity_mps - state.velocity_mps;
    let range = relative.length();
    let range_rate = if range > 1.0e-6 {
        relative.dot(relative_velocity) / range
    } else {
        0.0
    };
    match phase {
        RendezvousPhase::Approach {
            hold_distance_m,
            closing_rate_limit_mps,
            ..
        } => {
            let closing = (-range_rate).max(0.0);
            let hot =
                closing > *closing_rate_limit_mps && relative_velocity.length_squared() > 1.0e-12;
            let mut tick = if range > 1.0e-6 {
                if hot {
                    // Arriving hot: the gate can never open while the
                    // closing rate is over the limit (space has no drag
                    // to bleed it), so brake along the target-minus-us
                    // velocity — retrograde relative — until the
                    // approach slows back into the envelope.
                    steer_toward(relative_velocity.normalize(), 1.0, state)
                } else {
                    let throttle = if closing < 0.5 * *closing_rate_limit_mps {
                        1.0
                    } else {
                        0.0
                    };
                    steer_toward(relative.normalize(), throttle, state)
                }
            } else {
                hold_tick(state)
            };
            tick.done = rendezvous::approach_gate_ok_params(
                range,
                range_rate,
                *hold_distance_m,
                *closing_rate_limit_mps,
            );
            tick
        }
        RendezvousPhase::MatchVelocity {
            match_tolerance_mps,
            ..
        } => {
            let speed = relative_velocity.length();
            let mut tick = if speed > 1.0e-6 {
                // Matching means burning along the delta-v that closes the
                // gap: v_target - v_us = +relative_velocity. The negated
                // form thrusts along our own motion relative to the target
                // and accelerates the divergence instead.
                steer_toward(
                    relative_velocity.normalize(),
                    if speed > *match_tolerance_mps {
                        1.0
                    } else {
                        0.0
                    },
                    state,
                )
            } else {
                hold_tick(state)
            };
            tick.done = rendezvous::velocity_matched(
                state.velocity_mps,
                target.velocity_mps,
                *match_tolerance_mps,
            );
            tick
        }
        RendezvousPhase::StationKeep {
            hold_distance_m,
            match_tolerance_mps,
            ..
        } => {
            let drift = relative_velocity.length();
            let mut tick = if drift > *match_tolerance_mps && drift > 1.0e-6 {
                // Nulling the drift burns along +relative_velocity (the
                // delta-v closing v_us to v_target); the negated form
                // pushes the drift apart, so the keep never settles.
                steer_toward(relative_velocity.normalize(), 1.0, state)
            } else {
                hold_tick(state)
            };
            // Dwell is not yet an IR parameter (future field); arrival in
            // the box completes the keep.
            tick.done = range <= *hold_distance_m && drift <= *match_tolerance_mps;
            tick
        }
        RendezvousPhase::BackAway {
            hold_distance_m, ..
        } => {
            let mut tick = if range > 1.0e-6 {
                steer_toward(-relative.normalize(), 1.0, state)
            } else {
                hold_tick(state)
            };
            tick.done = range >= BACK_AWAY_RANGE_FACTOR * *hold_distance_m;
            tick
        }
    }
}

/// Execute phase law. Arm and Verify retire immediately (arming is fast
/// against burn timescales; the miss was validated at build and is
/// recorded by the caller as telemetry). Burn holds here: the server
/// delegates the impulse to its `NodeExecutor` machinery, which integrates
/// delivered delta-v and publishes burnout — the law must not freelance a
/// second integrator.
pub fn execute_tick(phase: &ExecutePhase, state: &LiveState) -> PhaseTick {
    match phase {
        ExecutePhase::Arm { .. } | ExecutePhase::Verify { .. } => {
            let mut tick = hold_tick(state);
            tick.done = true;
            tick
        }
        ExecutePhase::Burn { .. } => hold_tick(state),
        ExecutePhase::Abort { .. } => {
            let mut tick = hold_tick(state);
            tick.done = true;
            tick
        }
    }
}

/// Dispatch any phase node config to its law. Returns `None` for
/// non-phase configs (waits, guidance, sources, sinks).
pub fn phase_tick(config: &GraphNodeConfig, state: &LiveState) -> Option<PhaseTick> {
    match config {
        GraphNodeConfig::AscentPhase { phase } => Some(ascent_tick(phase, state)),
        GraphNodeConfig::LandingPhase { phase } => Some(landing_tick(phase, state)),
        GraphNodeConfig::RendezvousPhase { phase } => Some(rendezvous_tick(phase, state)),
        GraphNodeConfig::ExecutePhase { phase } => Some(execute_tick(phase, state)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        LandingSite, ascent::AscentProfile, landing::LandingProfile, rendezvous::RendezvousProfile,
    };

    fn earth_state(position_m: DVec3, velocity_mps: DVec3) -> LiveState {
        LiveState {
            position_m,
            velocity_mps,
            orientation_body_to_inertial: DQuat::IDENTITY,
            body_mu_m3_s2: 3.986_004_418e14,
            body_radius_m: 6_371_000.0,
            target: None,
        }
    }

    fn circular_state(radius_m: f64, mu: f64) -> LiveState {
        earth_state(
            DVec3::new(radius_m, 0.0, 0.0),
            DVec3::new(0.0, (mu / radius_m).sqrt(), 0.0),
        )
    }

    #[test]
    fn vertical_rise_steers_up_and_clears_the_tower() {
        let phase = AscentPhase::VerticalRise {
            throttle: 1.0,
            max_phase_time_s: 600.0,
        };
        let low = earth_state(DVec3::new(6_371_100.0, 0.0, 0.0), DVec3::ZERO);
        let tick = ascent_tick(&phase, &low);
        assert!(!tick.done);
        match tick.intent {
            GuidanceIntent::VelocityDirection { direction, .. } => {
                assert!((direction.direction - DVec3::X).length() < 1.0e-9);
            }
            intent => panic!("must steer radial, got {intent:?}"),
        }
        let high = earth_state(DVec3::new(6_371_000.0 + 200.0, 0.0, 0.0), DVec3::ZERO);
        assert!(ascent_tick(&phase, &high).done);
    }

    #[test]
    fn gravity_turn_blends_to_horizontal_and_cuts_off_on_apoapsis() {
        let phase = AscentPhase::GravityTurn {
            turn_start_altitude_m: 1_000.0,
            turn_end_altitude_m: 120_000.0,
            target_apoapsis_m: 200_000.0,
            max_phase_time_s: 600.0,
        };
        // Deep in the turn at 60 km doing 2 km/s flat: pitch ~45°.
        let state = earth_state(
            DVec3::new(6_431_000.0, 0.0, 0.0),
            DVec3::new(1_000.0, 2_000.0, 0.0),
        );
        let tick = ascent_tick(&phase, &state);
        assert!(!tick.done, "apoapsis far below target, must keep burning");
        match tick.intent {
            GuidanceIntent::VelocityDirection { direction, .. } => {
                // Pitch above horizon strictly between vertical and flat.
                let up = DVec3::X;
                let pitch = direction.direction.dot(up).asin();
                assert!(
                    pitch > 0.3 && pitch < 1.2,
                    "mid-turn pitch must blend, got {pitch}"
                );
            }
            intent => panic!("must steer a blended direction, got {intent:?}"),
        }
        // Orbital already above target: MECO immediately.
        let orbital = circular_state(6_371_000.0 + 200_000.0, 3.986_004_418e14);
        assert!(ascent_tick(&phase, &orbital).done);
    }

    #[test]
    fn coast_drifts_unpowered_and_hands_over_near_apoapsis() {
        let phase = AscentPhase::Coast {
            max_phase_time_s: 3_600.0,
        };
        // Climbing toward a 200 km apoapsis from 150 km: not done.
        let climbing = earth_state(
            DVec3::new(6_521_000.0, 0.0, 0.0),
            DVec3::new(500.0, 7_200.0, 0.0),
        );
        let tick = ascent_tick(&phase, &climbing);
        assert!(!tick.done);
        match tick.propulsion {
            p if p.normalized == 0.0 => {}
            p => panic!("coast must be unpowered, got {}", p.normalized),
        }
        // Circular at target: inside the band, done.
        let orbital = circular_state(6_371_000.0 + 200_000.0, 3.986_004_418e14);
        assert!(ascent_tick(&phase, &orbital).done);
    }

    #[test]
    fn circularize_burns_prograde_until_periapsis() {
        let phase = AscentPhase::Circularize {
            target_periapsis_m: 190_000.0,
            max_phase_time_s: 600.0,
        };
        // Eccentric transfer orbit at apoapsis: periapsis ~0, must burn.
        let mu: f64 = 3.986_004_418e14;
        let r: f64 = 6_371_000.0 + 200_000.0;
        let v_circ = (mu / r).sqrt();
        let state = earth_state(DVec3::new(r, 0.0, 0.0), DVec3::new(0.0, v_circ * 0.93, 0.0));
        let tick = ascent_tick(&phase, &state);
        assert!(!tick.done);
        match tick.intent {
            GuidanceIntent::VelocityDirection { direction, .. } => {
                assert!(direction.direction.dot(DVec3::Y) > 0.99);
            }
            intent => panic!("must burn prograde, got {intent:?}"),
        }
        // Circular at target: done.
        let orbital = circular_state(6_371_000.0 + 190_000.0, mu);
        assert!(ascent_tick(&phase, &orbital).done);
    }

    #[test]
    fn deorbit_targets_the_interface_and_coast_holds() {
        let burn = LandingPhase::DeorbitBurn {
            throttle: 1.0,
            entry_altitude_m: 120_000.0,
            max_phase_time_s: 600.0,
        };
        // Circular 300 km: periapsis far above the interface, keep burning
        // retrograde.
        let orbital = circular_state(6_371_000.0 + 300_000.0, 3.986_004_418e14);
        let tick = landing_tick(&burn, &orbital);
        assert!(!tick.done);
        match tick.intent {
            GuidanceIntent::VelocityDirection { direction, .. } => {
                assert!(direction.direction.dot(DVec3::Y) < -0.99);
            }
            intent => panic!("must burn retrograde, got {intent:?}"),
        }
        // Coast holds unpowered until the interface.
        let coast = LandingPhase::CoastToEntry {
            entry_altitude_m: 120_000.0,
            max_phase_time_s: 600.0,
        };
        let high = earth_state(DVec3::new(6_371_000.0 + 200_000.0, 0.0, 0.0), DVec3::ZERO);
        assert!(!landing_tick(&coast, &high).done);
        let low = earth_state(DVec3::new(6_371_000.0 + 100_000.0, 0.0, 0.0), DVec3::ZERO);
        assert!(landing_tick(&coast, &low).done);
    }

    #[test]
    fn braking_coasts_to_the_gate_then_burns_to_the_limit() {
        let phase = LandingPhase::BrakingBurn {
            net_braking_decel_mps2: 8.0,
            touchdown_speed_limit_mps: 2.0,
            burn_throttle: 1.0,
            max_phase_time_s: 600.0,
        };
        // 100 m/s at 50 km: suicide distance is 625 m, gate closed.
        let high = earth_state(
            DVec3::new(6_371_000.0 + 50_000.0, 0.0, 0.0),
            DVec3::new(-100.0, 0.0, 0.0),
        );
        let tick = landing_tick(&phase, &high);
        assert!(!tick.done);
        assert_eq!(tick.propulsion.normalized, 0.0);
        // 100 m/s at 500 m: inside gate + margin, burning, not done.
        let gate = earth_state(
            DVec3::new(6_371_000.0 + 500.0, 0.0, 0.0),
            DVec3::new(-100.0, 0.0, 0.0),
        );
        let tick = landing_tick(&phase, &gate);
        assert!(!tick.done);
        assert_eq!(tick.propulsion.normalized, 1.0);
        // 1 m/s at 500 m: inside the limit, hand over.
        let slow = earth_state(
            DVec3::new(6_371_000.0 + 500.0, 0.0, 0.0),
            DVec3::new(-1.0, 0.0, 0.0),
        );
        assert!(landing_tick(&phase, &slow).done);
    }

    #[test]
    fn rendezvous_needs_a_target_and_gates_arrival() {
        let approach = RendezvousPhase::Approach {
            hold_distance_m: 100.0,
            closing_rate_limit_mps: 2.0,
            max_phase_time_s: 600.0,
        };
        // No target: safe hold, hang for the watchdog.
        let alone = LiveState {
            target: None,
            ..circular_state(6_871_000.0, 3.986_004_418e14)
        };
        let tick = rendezvous_tick(&approach, &alone);
        assert!(!tick.done);
        assert_eq!(tick.propulsion.normalized, 0.0);
        // 500 m out closing at 0.5 m/s: burn toward the target, not done.
        let closing = LiveState {
            target: Some(TargetState {
                position_m: DVec3::new(6_871_500.0, 0.0, 0.0),
                velocity_mps: DVec3::new(0.0, 7_500.0, 0.0),
            }),
            ..circular_state(6_871_000.0, 3.986_004_418e14)
        };
        let mut closing = closing;
        closing.velocity_mps = DVec3::new(0.5, 7_500.0, 0.0);
        let tick = rendezvous_tick(&approach, &closing);
        assert!(!tick.done);
        assert_eq!(tick.propulsion.normalized, 1.0);
        // Inside at rest: done.
        let held = LiveState {
            target: Some(TargetState {
                position_m: DVec3::new(6_871_050.0, 0.0, 0.0),
                velocity_mps: DVec3::new(0.0, 7_500.0, 0.0),
            }),
            ..circular_state(6_871_000.0, 3.986_004_418e14)
        };
        let mut held = held;
        held.velocity_mps = DVec3::new(0.0, 7_500.0, 0.0);
        assert!(rendezvous_tick(&approach, &held).done);
    }

    #[test]
    fn execute_arm_and_verify_retire_immediately() {
        let state = circular_state(6_871_000.0, 3.986_004_418e14);
        let arm = ExecutePhase::Arm {
            node: 0,
            max_phase_time_s: 60.0,
        };
        assert!(execute_tick(&arm, &state).done);
        let verify = ExecutePhase::Verify {
            predicted_miss_m: 1500.0,
            max_phase_time_s: 60.0,
        };
        assert!(execute_tick(&verify, &state).done);
        let burn = ExecutePhase::Burn {
            node: 0,
            delta_v_mps: [100.0, 0.0, 0.0],
            max_phase_time_s: 600.0,
        };
        // Burn never self-completes: the server delegates the impulse to
        // its NodeExecutor, which publishes burnout.
        assert!(!execute_tick(&burn, &state).done);
    }

    #[test]
    fn completion_events_cover_every_domain_wait() {
        use crate::GraphNodeConfig as Config;
        let profile = AscentProfile {
            target_apoapsis_m: 200_000.0,
            target_periapsis_m: 190_000.0,
            turn_start_altitude_m: 1_000.0,
            turn_end_altitude_m: 120_000.0,
            liftoff_throttle: 1.0,
            max_phase_time_s: 3_600.0,
        };
        let graph = crate::ascent::ascent_graph(&profile).expect("graph builds");
        let events: Vec<&str> = graph
            .nodes
            .iter()
            .filter_map(|node| node.config.as_ref())
            .filter_map(completion_event)
            .collect();
        assert!(events.contains(&ascent::event::TOWER_CLEARED));
        assert!(events.contains(&ascent::event::MECO));
        assert!(events.contains(&ascent::event::APOAPSIS_APPROACH));
        // Watchdog readable off every phase node.
        for node in &graph.nodes {
            if let Some(config) = node.config.as_ref()
                && matches!(
                    config,
                    Config::AscentPhase { .. }
                        | Config::LandingPhase { .. }
                        | Config::RendezvousPhase { .. }
                        | Config::ExecutePhase { .. }
                )
            {
                assert_eq!(phase_watchdog_s(config), Some(3_600.0));
            }
        }
        let _ = LandingProfile {
            site: LandingSite::new([0.0, 0.0, 1.0], 500.0).expect("site"),
            touchdown_speed_limit_mps: 2.0,
            entry_altitude_m: 50_000.0,
            net_braking_decel_mps2: 5.0,
            burn_throttle: 1.0,
            max_phase_time_s: 3_600.0,
        };
        let _ = RendezvousProfile {
            hold_distance_m: 100.0,
            closing_rate_limit_mps: 2.0,
            match_tolerance_mps: 0.1,
            max_phase_time_s: 3_600.0,
        };
    }
}

#[cfg(test)]
mod flight_tests {
    use super::*;
    use crate::ascent::AscentProfile;

    /// Closed-loop ascent to orbit on hand-rolled point-mass dynamics with
    /// perfect attitude (the laws command directions; the test applies
    /// thrust along them plus gravity). This proves the phase laws fly the
    /// profile they were built from — not just that each law looks sane
    /// in isolation.
    #[test]
    fn laws_fly_the_profile_to_orbit() {
        let mu = 3.986_004_418e14;
        let radius = 6_371_000.0;
        // Heavy lifter: mass and thrust are test-rig numbers, the
        // profile targets are the real contract under test.
        let mass_kg = 8_000.0;
        let thrust_n = 160_000.0;
        let profile = AscentProfile {
            target_apoapsis_m: 200_000.0,
            target_periapsis_m: 190_000.0,
            turn_start_altitude_m: 1_000.0,
            turn_end_altitude_m: 120_000.0,
            liftoff_throttle: 1.0,
            max_phase_time_s: 3_600.0,
        };
        let graph = crate::ascent::ascent_graph(&profile).expect("graph builds");
        let _ = graph;
        let phases = [
            AscentPhase::VerticalRise {
                throttle: profile.liftoff_throttle,
                max_phase_time_s: profile.max_phase_time_s,
            },
            AscentPhase::GravityTurn {
                turn_start_altitude_m: profile.turn_start_altitude_m,
                turn_end_altitude_m: profile.turn_end_altitude_m,
                target_apoapsis_m: profile.target_apoapsis_m,
                max_phase_time_s: profile.max_phase_time_s,
            },
            AscentPhase::Coast {
                max_phase_time_s: profile.max_phase_time_s,
            },
            AscentPhase::Circularize {
                target_periapsis_m: profile.target_periapsis_m,
                max_phase_time_s: profile.max_phase_time_s,
            },
        ];
        let mut position = DVec3::new(radius + 10.0, 0.0, 0.0);
        let mut velocity = DVec3::ZERO;
        let dt = 0.5;
        let mut completed = Vec::new();
        for (index, phase) in phases.iter().enumerate() {
            let mut steps = 0u32;
            loop {
                let state = LiveState {
                    position_m: position,
                    velocity_mps: velocity,
                    orientation_body_to_inertial: DQuat::IDENTITY,
                    body_mu_m3_s2: mu,
                    body_radius_m: radius,
                    target: None,
                };
                let tick = ascent_tick(phase, &state);
                let thrust_dir = match tick.intent {
                    GuidanceIntent::VelocityDirection { direction, .. } => direction.direction,
                    _ => DVec3::ZERO,
                };
                let gravity = -position.normalize() * (mu / position.length_squared());
                let acceleration =
                    thrust_dir * (tick.propulsion.normalized * thrust_n / mass_kg) + gravity;
                velocity += acceleration * dt;
                position += velocity * dt;
                steps += 1;
                assert!(steps < 20_000, "phase {index} never completes");
                assert!(
                    position.length() > radius,
                    "phase {index} impacted the ground"
                );
                if tick.done {
                    completed.push(index);
                    let apo = crate::ascent::predict_apoapsis_m(mu, position, velocity);
                    let peri = crate::ascent::predict_periapsis_m(mu, position, velocity);
                    eprintln!(
                        "phase {index} done after {steps} steps: alt {:.1} km speed {:.0} m/s apo {:?} peri {:?}",
                        (position.length() - radius) / 1000.0,
                        velocity.length(),
                        apo.map(|a| (a - radius) / 1000.0),
                        peri.map(|p| (p - radius) / 1000.0),
                    );
                    break;
                }
            }
        }
        assert_eq!(completed, vec![0, 1, 2, 3], "phases retire in order");
        let periapsis =
            crate::ascent::predict_periapsis_m(mu, position, velocity).expect("bound orbit");
        let apoapsis =
            crate::ascent::predict_apoapsis_m(mu, position, velocity).expect("bound orbit");
        eprintln!(
            "achieved orbit: periapsis {:.0} km, apoapsis {:.0} km",
            (periapsis - radius) / 1000.0,
            (apoapsis - radius) / 1000.0
        );
        assert!(
            periapsis >= radius + 185_000.0,
            "must circularize near the 190 km target"
        );
        assert!(
            apoapsis <= radius + 260_000.0,
            "must not overshoot the 200 km target wildly"
        );
    }
}
