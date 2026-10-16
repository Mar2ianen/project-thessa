//! Shared client replication/presentation policy. No transport clock, physics
//! stepping or renderer handles; hosts retain intent and presentation ownership.
use crate::{ClientInput, Command, Snapshot};
use thessa_flight_authority::{
    COAST_DENSITY_KG_M3, ControlMode, FlightAuthority, FlightRegime, presentation::FlightPose,
};
use thessa_sim_core::{BakedEphemeris, SimTime, VehicleId, WorldTick};

/// A checked non-primary observation at its own authoritative sample time.
/// No vehicle design, renderer proxy, prediction or cross-vehicle clock is
/// invented from a lean fleet payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FleetPose {
    pub vehicle_id: VehicleId,
    pub tick: u64,
    pub pose: FlightPose,
    pub stopped: bool,
}

/// Validate a complete fleet transaction before a host replaces membership.
/// Every pose subtracts the caller's reference ephemeris at its own wire time;
/// fleet samples need not share PRIMARY's time (secondary debt can differ).
pub fn fleet_poses(
    fleet: &crate::FleetSnapshot,
    ephemeris: &BakedEphemeris,
    reference: thessa_sim_core::BodyId,
) -> Result<Vec<FleetPose>, String> {
    fleet.validate_membership()?;
    fleet
        .vehicles
        .iter()
        .map(|vehicle| {
            Ok(FleetPose {
                vehicle_id: vehicle.vehicle_id,
                tick: vehicle.snapshot.tick,
                pose: checked_snapshot_pose(&vehicle.snapshot, ephemeris, reference)?,
                stopped: vehicle.snapshot.flight_error.is_some(),
            })
        })
        .collect()
}

/// Checked pose from a wire sample, subtracting the reference at that sample's
/// own time. It never adopts interpolation into authoritative client state.
pub fn snapshot_pose(
    snapshot: &Snapshot,
    ephemeris: &BakedEphemeris,
    body: thessa_sim_core::BodyId,
) -> Option<FlightPose> {
    FlightPose::from_inertial(
        ephemeris,
        body,
        SimTime(snapshot.flight_time_s),
        &snapshot.state,
    )
}

/// Existing pilot snapshot adoption without a local physics step. Validate
/// kinematics and consumed scalar metadata before mutating the replica. Local
/// input intent, SAS/part demands, resource inventories and scheduler are not
/// replaced by this lean snapshot. Parachute loads use the authority's existing
/// checked telemetry-adoption contract (mismatched/malformed loads ignored).
pub fn adopt_snapshot(
    authority: &mut FlightAuthority,
    ephemeris: &BakedEphemeris,
    mode: ControlMode,
    snapshot: &Snapshot,
) -> Result<FlightPose, String> {
    let pose = checked_snapshot_pose(snapshot, ephemeris, authority.reference_body)?;
    let density = authority
        .atmosphere
        .sample((pose.relative_position_m().length() - authority.planet_radius_m).max(0.0));
    authority.state = snapshot.state;
    authority.flight_time_s = snapshot.flight_time_s;
    authority.world_tick = WorldTick(snapshot.tick);
    authority.accumulator_s = 0.0;
    authority.throttle = snapshot.throttle;
    authority.engine_active = snapshot.engine_active;
    authority.steps_this_frame = snapshot.steps_this_frame;
    authority.rails_advanced_this_frame = snapshot.rails_advanced_s;
    authority.reaction_wheel_torque_body_nm =
        glam::DVec3::from_array(snapshot.reaction_wheel_torque_body_nm);
    authority.adopt_parachute_telemetry(&snapshot.parachutes);
    authority.wake_notice = snapshot.wake_notice.clone();
    authority.flight_error = snapshot.flight_error.clone();
    authority.relative_position_m = pose.relative_position_m();
    authority.regime = match density {
        Ok(sample) if sample.density_kg_m3 < COAST_DENSITY_KG_M3 => FlightRegime::Coast,
        _ => FlightRegime::Aero,
    };
    if let Ok((gravity, forces)) = authority.display_loads(ephemeris, pose.time(), mode) {
        authority.last_gravity_acceleration_inertial_mps2 = gravity;
        authority.last_forces = Some(forces);
    }
    Ok(pose)
}

/// Validate the kinematics and scalar telemetry consumed by client replicas.
/// Does not resize/validate authored vehicle resources from wire telemetry.
pub fn checked_snapshot_pose(
    snapshot: &Snapshot,
    ephemeris: &BakedEphemeris,
    body: thessa_sim_core::BodyId,
) -> Result<FlightPose, String> {
    let pose = snapshot_pose(snapshot, ephemeris, body)
        .ok_or("invalid snapshot pose or reference ephemeris")?;
    if !snapshot.state.velocity_inertial_mps.is_finite()
        || !snapshot.state.angular_velocity_body_rps.is_finite()
        || snapshot.flight_time_s < 0.0
        || pose.relative_position_m().try_normalize().is_none()
        || !snapshot.throttle.is_finite()
        || !(0.0..=1.0).contains(&snapshot.throttle)
        || [
            snapshot.effective_warp,
            snapshot.server_compute_s,
            snapshot.server_wall_s,
            snapshot.rails_advanced_s,
        ]
        .iter()
        .any(|value| !value.is_finite() || *value < 0.0)
        || !glam::DVec3::from_array(snapshot.reaction_wheel_torque_body_nm).is_finite()
    {
        return Err("invalid snapshot kinematics or telemetry".into());
    }
    Ok(pose)
}

/// Shared PRIMARY-vehicle input construction. Conditioned stick/SAS/part
/// demands live in the client replica; throttle and engine intent are supplied
/// separately so a delayed acknowledgement cannot overwrite the next request.
/// Hosts supply ordered events and their own warp/pause votes.
pub fn input_for_server(
    authority: &FlightAuthority,
    mode: ControlMode,
    throttle: f64,
    engine_active: bool,
    commands: Vec<Command>,
) -> ClientInput {
    ClientInput {
        tick: authority.world_tick.0,
        vehicle_id: VehicleId::PRIMARY,
        control_input: authority.control_input.to_array(),
        control_mode: mode,
        sas_target_xyzw: authority.sas_target_orientation.to_array(),
        throttle,
        engine_active,
        sas_enabled: authority.sas_enabled,
        rcs_enabled: authority.rcs_enabled,
        reaction_wheels_enabled: authority.reaction_wheels_enabled,
        gear_down: authority.gear_down,
        parachutes_armed: authority.parachutes_armed,
        commands,
    }
}

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
