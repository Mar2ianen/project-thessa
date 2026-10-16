//! Existing pilot snapshot interpolation, extracted from the Bevy host without
//! moving its receive clock, inbox or authoritative-state ownership here.
use crate::Snapshot;

/// Presentation copy only: keep the latest authoritative metadata and blend
/// the same time/rigid-state fields as the old embedded client. No extrapolation.
/// The receive-policy owner supplies finite alpha; a pose consumer must validate
/// the result before publishing it. This does not validate/adopt server truth.
pub fn interpolate_snapshot(previous: &Snapshot, latest: &Snapshot, alpha: f64) -> Snapshot {
    let alpha = alpha.clamp(0.0, 1.0);
    let mut snapshot = latest.clone();
    snapshot.flight_time_s =
        previous.flight_time_s + (latest.flight_time_s - previous.flight_time_s) * alpha;
    snapshot.state.position_inertial_m = previous
        .state
        .position_inertial_m
        .lerp(latest.state.position_inertial_m, alpha);
    snapshot.state.velocity_inertial_mps = previous
        .state
        .velocity_inertial_mps
        .lerp(latest.state.velocity_inertial_mps, alpha);
    snapshot.state.orientation_body_to_inertial = previous
        .state
        .orientation_body_to_inertial
        .slerp(latest.state.orientation_body_to_inertial, alpha)
        .normalize();
    snapshot.state.angular_velocity_body_rps = previous
        .state
        .angular_velocity_body_rps
        .lerp(latest.state.angular_velocity_body_rps, alpha);
    snapshot
}
