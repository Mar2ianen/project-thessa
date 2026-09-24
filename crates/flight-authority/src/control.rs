//! Native flight-control primitives used by the authoritative runtime.
//!
//! Phase 1 of the control refactor keeps the existing X-15 behavior intact,
//! but gives each responsibility an explicit boundary.  These functions are
//! deliberately synchronous and allocation-free in the per-tick path; the
//! higher-level guidance and allocator graph can be layered on them later.

use glam::{DMat3, DQuat, DVec3};
use thessa_flight_control::{
    ActuatorGroup, ControlDemand, EffectorContribution, PropulsionDemand, allocate_wrench,
};
use thessa_sim_core::{
    AeroEnvironment, AeroGeometry, AeroModel, AeroState, FlightError, PanelAeroModel,
    RigidBodyState, VehicleDefinition,
};

use crate::ControlMode;

/// Legacy normalized-command slew for controls without authored physical
/// actuator data. Mechanized surfaces use their own rad/s and torque ratings.
pub(crate) const SURFACE_COMMAND_RATE_S: f64 = 2.4;
// Residual moment below which the trim Newton skips its second pass.  The
// tolerance is intentionally part of the control primitive so its bounded
// approximation remains pinned by the same regression tests as the full solve.
pub(crate) const TRIM_SECOND_PASS_RESIDUAL_TOL_NM: f64 = 50.0;
// KSP-style attitude keys change the SAS target at a pilotable rate.
pub(crate) const PILOT_ATTITUDE_COMMAND_RATE_RAD_S: f64 = 0.16;

/// Output of the native attitude/rate controller before actuator allocation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AttitudeDemand {
    pub axes: DVec3,
    pub desired_rate_rps: DVec3,
    pub requested_moment_nm: DVec3,
    pub target_orientation: DQuat,
}

/// Derive a body-rate target and requested body moment from the current pilot
/// command and the existing attitude-hold state.  Capturing the target is
/// returned to the caller instead of mutating runtime state inside this law.
pub(crate) fn attitude_demand(
    mode: ControlMode,
    sas_enabled: bool,
    control_input: DVec3,
    state: RigidBodyState,
    sas_target_orientation: DQuat,
    inertia: DMat3,
) -> AttitudeDemand {
    let axes = body_axes(control_input);
    let attitude_hold = sas_enabled && matches!(mode, ControlMode::Navball | ControlMode::MouseAim);
    let (desired_rate_rps, target_orientation) = if attitude_hold && axes.length_squared() > 1.0e-8
    {
        // Manual input overrides attitude hold. Capturing the achieved
        // attitude prevents a long turn from winding an unreachable target
        // past 180 degrees and making shortest-path SAS reverse it.
        (
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S,
            state.orientation_body_to_inertial,
        )
    } else if attitude_hold {
        let mut error = state.orientation_body_to_inertial.inverse() * sas_target_orientation;
        // q and -q represent the same attitude; use the short rotation.
        if error.w < 0.0 {
            error = -error;
        }
        let angle = error.to_scaled_axis();
        let couples = rcs_couples();
        let acceleration = DVec3::new(
            couples[0].x / inertia.x_axis.x,
            couples[1].y / inertia.y_axis.y,
            couples[2].z / inertia.z_axis.z,
        );
        // Brake early enough for the finite jets. A fixed high-gain attitude
        // loop saturates in vacuum and keeps overshooting.
        let braking_rate = (acceleration * angle.abs() * 0.5).sqrt();
        (
            (angle * 1.6)
                .clamp(-braking_rate, braking_rate)
                .clamp_length_max(0.35),
            sas_target_orientation,
        )
    } else {
        // Capturing here avoids a jump to an old SAS target when re-enabled.
        (
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S,
            state.orientation_body_to_inertial,
        )
    };
    let omega = state.angular_velocity_body_rps;
    let requested_moment_nm =
        inertia * ((desired_rate_rps - omega) / 0.35) + omega.cross(inertia * omega);
    AttitudeDemand {
        axes,
        desired_rate_rps,
        requested_moment_nm,
        target_orientation,
    }
}

/// The three opposed RCS couples used by the starter vehicle.
pub(crate) fn rcs_couples() -> [DVec3; 3] {
    [
        2.0 * DVec3::new(0.0, 1.4, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(-5.0, 0.0, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(5.0, 0.0, 0.0).cross(DVec3::Y * 400.0),
    ]
}

pub(crate) fn rcs_moment(request: DVec3, enabled: bool) -> DVec3 {
    if !enabled {
        return DVec3::ZERO;
    }
    let couples = rcs_couples();
    let effectors: [EffectorContribution; 6] = std::array::from_fn(|index| {
        let couple = couples[index / 2] * if index % 2 == 0 { 1.0 } else { -1.0 };
        EffectorContribution {
            group: ActuatorGroup::Rcs,
            force_per_command_n: DVec3::ZERO,
            moment_per_command_nm: couple,
            max_command: 1.0,
            weight: 1.0,
        }
    });
    match allocate_wrench(
        ControlDemand {
            moment_body_nm: request,
            propulsion: PropulsionDemand { normalized: 0.0 },
            force_body_n: DVec3::ZERO,
        },
        &effectors,
    ) {
        Ok(allocation) => allocation.achieved_moment_body_nm,
        Err(_) => DVec3::ZERO,
    }
}

/// A translation command uses opposed RCS jets as a force-balanced pair. The
/// starter vehicle's moment couples are built from two 400 N jets, so each
/// body axis has 800 N of net translation authority in either direction.
pub(crate) const RCS_TRANSLATION_FORCE_N: f64 = 800.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RcsForceAllocation {
    pub force_body_n: DVec3,
    pub saturated: bool,
}

pub(crate) fn allocate_rcs_force(request: DVec3, enabled: bool) -> RcsForceAllocation {
    if !enabled {
        return RcsForceAllocation {
            force_body_n: DVec3::ZERO,
            saturated: request.length_squared() > 1.0e-12,
        };
    }
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let effectors: [EffectorContribution; 6] = std::array::from_fn(|index| EffectorContribution {
        group: ActuatorGroup::Rcs,
        force_per_command_n: axes[index / 2]
            * (if index % 2 == 0 {
                RCS_TRANSLATION_FORCE_N
            } else {
                -RCS_TRANSLATION_FORCE_N
            }),
        moment_per_command_nm: DVec3::ZERO,
        max_command: 1.0,
        weight: 1.0,
    });
    let Ok(allocation) = allocate_wrench(
        ControlDemand {
            force_body_n: request,
            moment_body_nm: DVec3::ZERO,
            propulsion: PropulsionDemand { normalized: 0.0 },
        },
        &effectors,
    ) else {
        return RcsForceAllocation {
            force_body_n: DVec3::ZERO,
            saturated: true,
        };
    };
    RcsForceAllocation {
        force_body_n: allocation.achieved_force_body_n,
        saturated: allocation.saturated,
    }
}

/// Result of allocating a requested moment across residual RCS authority.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct RcsAllocation {
    pub moment_body_nm: DVec3,
    pub saturated: bool,
}

/// Preserve the current two-stage behavior: assisted control asks RCS for the
/// residual after aero surfaces, while direct control maps raw body axes to
/// the available opposed couples. Saturation is reported against the whole
/// assisted demand, not merely against the RCS residual.
pub(crate) fn allocate_rcs(
    requested_moment_nm: DVec3,
    actual_aero_moment_nm: DVec3,
    axes: DVec3,
    assisted: bool,
    rcs_enabled: bool,
) -> RcsAllocation {
    let request = if assisted {
        requested_moment_nm - actual_aero_moment_nm
    } else {
        let couples = rcs_couples();
        DVec3::new(couples[0].x, couples[1].y, couples[2].z) * axes
    };
    let moment_body_nm = rcs_moment(request, rcs_enabled);
    let saturated = assisted
        && (requested_moment_nm - actual_aero_moment_nm - moment_body_nm).length() > 1_000.0;
    RcsAllocation {
        moment_body_nm,
        saturated,
    }
}

/// Advance normalized surface commands by the physical command-rate limit.
pub(crate) fn slew_surface_command(current: DVec3, target: DVec3, max_change: f64) -> DVec3 {
    current + (target - current).clamp(DVec3::splat(-max_change), DVec3::splat(max_change))
}

/// Result of an aero trim solve. Counters are returned so the runtime can
/// expose its existing telemetry without coupling the solver to FlightAuthority.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct TrimSolveResult {
    pub command: DVec3,
    pub second_passes: u64,
}

/// Solve the current aerodynamic surface command against a requested moment.
/// This is the existing Newton trim algorithm extracted verbatim in behavior:
/// one baseline/effectiveness pass, with an adaptive second pass near stall or
/// whenever the first pass materially changes the command.
#[allow(clippy::too_many_arguments)]
pub(crate) fn solve_aero_trim(
    vehicle: &mut VehicleDefinition,
    reference_geometry: &AeroGeometry,
    aero_model: &PanelAeroModel,
    aero_state: AeroState,
    environment: AeroEnvironment,
    requested_moment_nm: DVec3,
    mut command: DVec3,
    force_two_pass: bool,
) -> Result<TrimSolveResult, FlightError> {
    let mut second_passes = 0;
    for pass in 0..2 {
        let before = command;
        apply_trim_command(vehicle, reference_geometry, command)?;
        let baseline = aero_model
            .evaluate_state(aero_state, environment, &vehicle.aero_geometry)
            .map_err(FlightError::Aero)?
            .moment_body_nm;
        if pass == 1
            && !force_two_pass
            && (requested_moment_nm - baseline).length() <= TRIM_SECOND_PASS_RESIDUAL_TOL_NM
        {
            break;
        }
        let mut columns = [DVec3::ZERO; 3];
        for axis in 0..3 {
            let mut probe = command;
            let delta = if command[axis] > 0.9 { -0.02 } else { 0.02 };
            probe[axis] += delta;
            apply_trim_command(vehicle, reference_geometry, probe)?;
            columns[axis] = (aero_model
                .evaluate_state(aero_state, environment, &vehicle.aero_geometry)
                .map_err(FlightError::Aero)?
                .moment_body_nm
                - baseline)
                / delta;
        }
        let effectiveness = DMat3::from_cols(columns[0], columns[1], columns[2]);
        if effectiveness.determinant().abs() > 1.0e-6 {
            command = (command + effectiveness.inverse() * (requested_moment_nm - baseline))
                .clamp(DVec3::splat(-1.0), DVec3::ONE);
        }
        if pass == 0 && command == before {
            break;
        }
        if pass == 1 {
            second_passes += 1;
        }
    }
    Ok(TrimSolveResult {
        command,
        second_passes,
    })
}

fn apply_trim_command(
    vehicle: &mut VehicleDefinition,
    reference_geometry: &AeroGeometry,
    command: DVec3,
) -> Result<(), FlightError> {
    let commands = surface_commands(command.x, command.y, command.z);
    let deflections = vehicle
        .control_surfaces
        .iter()
        .zip(commands)
        .map(|(surface, command)| surface.deflection_for_command(command))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
    vehicle
        .apply_control_deflections(reference_geometry, &deflections)
        .map_err(|error| FlightError::InvalidInput(error.to_string()))
}

pub(crate) fn body_axes(command: DVec3) -> DVec3 {
    DVec3::new(command.z, -command.x, -command.y)
}

/// Map the three normalized pilot surface axes to the starter vehicle's four
/// physical channels: elevator, rudder, left aileron and right aileron.
pub(crate) fn surface_commands(pitch: f64, yaw: f64, roll: f64) -> [f64; 4] {
    [-pitch, yaw, -roll, roll]
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::RigidBodyProperties;

    #[test]
    fn direct_axes_match_the_starter_vehicle_mapping() {
        assert_eq!(
            body_axes(DVec3::new(0.2, -0.3, 0.4)),
            DVec3::new(0.4, -0.2, 0.3)
        );
    }

    #[test]
    fn disabled_rcs_has_no_moment_or_saturation() {
        let result = allocate_rcs(DVec3::splat(10_000.0), DVec3::ZERO, DVec3::ONE, true, false);
        assert_eq!(result.moment_body_nm, DVec3::ZERO);
        assert!(result.saturated);
    }

    #[test]
    fn translation_rcs_is_bounded_and_reports_saturation() {
        let result = allocate_rcs_force(DVec3::new(1_000.0, -900.0, 20.0), true);
        assert_eq!(result.force_body_n, DVec3::new(800.0, -800.0, 20.0));
        assert!(result.saturated);
        assert_eq!(
            allocate_rcs_force(DVec3::X * 10.0, false).force_body_n,
            DVec3::ZERO
        );
    }

    #[test]
    fn surface_slew_is_componentwise_and_bounded() {
        assert_eq!(
            slew_surface_command(DVec3::ZERO, DVec3::new(1.0, -1.0, 0.25), 0.2),
            DVec3::new(0.2, -0.2, 0.2)
        );
    }

    #[test]
    fn attitude_demand_captures_manual_input_target() {
        let state = RigidBodyState::stationary(DVec3::ZERO);
        let inertia = RigidBodyProperties::new(1.0, DMat3::from_diagonal(DVec3::splat(2.0)))
            .unwrap()
            .inertia_body_kg_m2;
        let demand = attitude_demand(
            ControlMode::Navball,
            true,
            DVec3::X,
            state,
            DQuat::IDENTITY,
            inertia,
        );
        assert_eq!(
            demand.target_orientation,
            state.orientation_body_to_inertial
        );
        assert!(demand.desired_rate_rps.length() > 0.0);
        assert!(demand.requested_moment_nm.length() > 0.0);
    }
}
