//! Wall-clock pacing and simulation wake-boundary calculations.

use super::*;

pub(super) fn driver_sleep_duration(
    ingress_saturated: bool,
    budget_exhausted: bool,
    bake_wait: bool,
    lag_s: f64,
    requested_warp: f64,
    work_elapsed: Duration,
) -> Duration {
    // Saturation is a yield point, not a 4 ms / 20 ms duty cycle. Returning
    // zero makes the next iteration run immediately after ingress handling.
    if ingress_saturated || budget_exhausted {
        return Duration::ZERO;
    }
    if bake_wait {
        return BAKE_POLL_INTERVAL;
    }
    if lag_s + 1.0e-12 >= thessa_sim_core::WORLD_TICK_S {
        return Duration::ZERO;
    }
    let until_tick_s = (thessa_sim_core::WORLD_TICK_S - lag_s) / requested_warp;
    let remaining_s = until_tick_s - work_elapsed.as_secs_f64();
    if remaining_s > 0.0 {
        Duration::from_secs_f64(remaining_s.clamp(0.001, DRIVER_WORK_QUANTUM_S))
    } else {
        Duration::ZERO
    }
}

/// Simulation time already covered by the pacing target. The authority keeps
/// a fractional fixed-step remainder in its accumulator, so that remainder
/// must not be requested again by the driver as fresh demand.
pub(super) fn pacing_demand_s(target_s: f64, advanced_s: f64, backlog_s: f64, tick_s: f64) -> f64 {
    let covered_s = advanced_s + backlog_s;
    let demand_s = (target_s - covered_s).max(0.0);
    // A fresh sub-tick request is runnable when it completes the tick that
    // is already partially queued in the authority accumulator.
    if backlog_s + demand_s + 1.0e-12 >= tick_s {
        demand_s
    } else {
        0.0
    }
}

pub(super) fn pacing_work_pending(chunk_s: f64, backlog_s: f64, tick_s: f64) -> bool {
    chunk_s > 0.0 || backlog_s + 1.0e-12 >= tick_s
}

/// Clip a requested chunk to the first fixed-step boundary at or after a
/// scheduler wake. The authority can only drain events after whole physics
/// ticks, so a wake inside the next tick must still be allowed to reach that
/// tick; clipping to the raw wake-minus-backlog distance would strand a
/// fractional accumulator forever.
pub(super) fn clip_pacing_chunk_to_wake(
    chunk_s: f64,
    flight_time_s: f64,
    backlog_s: f64,
    wake: SimTime,
    tick_s: f64,
) -> f64 {
    if chunk_s <= 0.0 || wake.0 <= flight_time_s {
        return chunk_s;
    }
    let ticks_until_wake = ((wake.0 - flight_time_s) / tick_s).ceil().max(1.0);
    let boundary_s = flight_time_s + ticks_until_wake * tick_s;
    chunk_s.min((boundary_s - flight_time_s - backlog_s).max(0.0))
}
