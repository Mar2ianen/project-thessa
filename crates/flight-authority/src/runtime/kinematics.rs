//! Body-relative flight kinematics and terrain-track evidence values.

use super::*;

#[derive(Debug, Clone, Copy)]
pub struct LocalAirKinematics {
    pub relative_position_inertial_m: DVec3,
    pub relative_position_body_m: DVec3,
    pub relative_velocity_inertial_mps: DVec3,
    pub air_velocity_body_mps: DVec3,
    pub surface_velocity_inertial_mps: DVec3,
    pub radial_up: DVec3,
    pub altitude_m: f64,
}

/// Terrain evidence for a sampled flight track. The obstacle reports prove
/// geometric coverage of the piecewise track and provide a conservative
/// sampled terrain ceiling. Vehicle-clearance evidence is available through
/// the worldgen obstacle report's explicit withstand proof.
#[derive(Debug, Clone, PartialEq)]
pub struct TerrainTrackCoverage {
    pub obstacles: ObstacleTrackCertificate,
    pub min_altitude_m: f64,
    pub min_obstacle_clearance_m: f64,
}

/// Sample all local flight kinematics from one rigid-body state and one
/// ephemeris body state. The rotating atmosphere vector is converted into the
/// vehicle frame by sim-core; callers must not cross a body-frame position with
/// an inertial-frame angular velocity directly.
pub fn local_air_kinematics(
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    body_state: BodyState,
    body_radius_m: f64,
) -> Result<LocalAirKinematics, AtmosphereError> {
    let relative_position_inertial_m = state.position_inertial_m - body_state.position_inertial;
    let radial_up = relative_position_inertial_m
        .try_normalize()
        .unwrap_or(DVec3::Z);
    let orientation_inverse = state.orientation_body_to_inertial.inverse();
    let relative_position_body_m = orientation_inverse * relative_position_inertial_m;
    let relative_velocity_inertial_mps = state.velocity_inertial_mps - body_state.velocity_inertial;
    let relative_velocity_body_mps = orientation_inverse * relative_velocity_inertial_mps;
    let rotating_air_velocity_body_mps = atmosphere.rotating_air_velocity_body_mps(
        relative_position_body_m,
        state.orientation_body_to_inertial,
    )?;
    let air_velocity_body_mps = relative_velocity_body_mps - rotating_air_velocity_body_mps;
    let surface_velocity_inertial_mps = relative_velocity_inertial_mps
        - state.orientation_body_to_inertial * rotating_air_velocity_body_mps;
    let altitude_m = relative_position_inertial_m.length() - body_radius_m;
    Ok(LocalAirKinematics {
        relative_position_inertial_m,
        relative_position_body_m,
        relative_velocity_inertial_mps,
        air_velocity_body_mps,
        surface_velocity_inertial_mps,
        radial_up,
        altitude_m,
    })
}
