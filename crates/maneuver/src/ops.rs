//! MechJeb-analog planning operations on an explicit two-body model.
//!
//! `Circularize`, Hohmann `PlanTransfer`, Lambert rendezvous,
//! `MatchVelocity`: the same verbs as the reference, computed against one
//! central `mu` in a central-body-centered inertial frame. That model is a
//! PLANNING approximation (patched-conic style), stated on every function:
//! broad-search pruning may use it, but execution requires exact
//! revalidation (`search`), which re-propagates finalists through the full
//! N-body field before anything flies.

use glam::DVec3;
use thessa_sim_core::SimTime;

use crate::{
    ManeuverNode, ManeuverPlan, PlanError,
    lambert::{LambertError, solve_lambert},
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OpsError {
    Plan(PlanError),
    Lambert(LambertError),
    NonFiniteInput,
    NotCircular { eccentricity: f64 },
}

impl std::fmt::Display for OpsError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Plan(error) => write!(formatter, "plan error: {error}"),
            Self::Lambert(error) => write!(formatter, "lambert error: {error}"),
            Self::NonFiniteInput => write!(formatter, "non-finite op input"),
            Self::NotCircular { eccentricity } => {
                write!(formatter, "hohmann needs a near-circular orbit, e={eccentricity}")
            }
        }
    }
}

impl std::error::Error for OpsError {}

impl From<PlanError> for OpsError {
    fn from(error: PlanError) -> Self {
        Self::Plan(error)
    }
}

impl From<LambertError> for OpsError {
    fn from(error: LambertError) -> Self {
        Self::Lambert(error)
    }
}

fn check_finite_vectors(vectors: &[DVec3], time_s: f64, mu: f64) -> Result<(), OpsError> {
    if !time_s.is_finite() || !mu.is_finite() || mu <= 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    if vectors.iter().any(|v| !v.is_finite()) {
        return Err(OpsError::NonFiniteInput);
    }
    Ok(())
}

fn prograde_tangent(position: DVec3, velocity: DVec3) -> Result<DVec3, OpsError> {
    let momentum = position.cross(velocity);
    if momentum.length_squared() <= 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    Ok(momentum.cross(position).normalize())
}

/// Single-impulse burn onto the circular velocity at the current radius.
/// Optimal at an apsis (trigger it there with a `WaitUntil`); valid but
/// suboptimal elsewhere. Returns the node only, not a full plan.
pub fn circularize_at_apse(
    position_m: DVec3,
    velocity_mps: DVec3,
    mu_m3_s2: f64,
    epoch: SimTime,
) -> Result<ManeuverNode, OpsError> {
    check_finite_vectors(&[position_m, velocity_mps], epoch.0, mu_m3_s2)?;
    let radius = position_m.length();
    if radius <= 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    let tangent = prograde_tangent(position_m, velocity_mps)?;
    let circular = tangent * (mu_m3_s2 / radius).sqrt();
    ManeuverNode::new(epoch, circular - velocity_mps).map_err(OpsError::from)
}

/// Hohmann transfer between coplanar circular orbits. Requires a
/// near-circular departure orbit (`e <= 0.05`); returns departure + arrival
/// nodes, the arrival one placed half a transfer period later at the
/// antipodal point. `predicted_miss_m` stays `None`: revalidate.
pub fn hohmann_transfer(
    departure_position_m: DVec3,
    departure_velocity_mps: DVec3,
    target_radius_m: f64,
    mu_m3_s2: f64,
    departure_epoch: SimTime,
) -> Result<ManeuverPlan, OpsError> {
    check_finite_vectors(
        &[departure_position_m, departure_velocity_mps],
        departure_epoch.0,
        mu_m3_s2,
    )?;
    if !target_radius_m.is_finite() || target_radius_m <= 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    let r1 = departure_position_m.length();
    // Eccentricity vector magnitude: the circularity gate.
    let energy_term = departure_velocity_mps.length_squared() - mu_m3_s2 / r1;
    let radial = departure_position_m.dot(departure_velocity_mps);
    let ecc_vector =
        (departure_position_m * energy_term - departure_velocity_mps * radial) / mu_m3_s2;
    let eccentricity = ecc_vector.length();
    if !(eccentricity <= 0.05) {
        return Err(OpsError::NotCircular { eccentricity });
    }
    let semi = 0.5 * (r1 + target_radius_m);
    let dt = std::f64::consts::PI * (semi * semi * semi / mu_m3_s2).sqrt();
    let dep_speed = (mu_m3_s2 * (2.0 / r1 - 1.0 / semi)).sqrt();
    let tangent = prograde_tangent(departure_position_m, departure_velocity_mps)?;
    let node1 = ManeuverNode::new(
        departure_epoch,
        tangent * dep_speed - departure_velocity_mps,
    )?;
    let arrival_position = -departure_position_m.normalize() * target_radius_m;
    let arr_speed = (mu_m3_s2 * (2.0 / target_radius_m - 1.0 / semi)).sqrt();
    let momentum = departure_position_m.cross(departure_velocity_mps);
    let arrival_tangent = (momentum.cross(arrival_position)).normalize();
    let node2 = ManeuverNode::new(
        SimTime(departure_epoch.0 + dt),
        arrival_tangent * (mu_m3_s2 / target_radius_m).sqrt()
            - arrival_tangent * arr_speed,
    )?;
    ManeuverPlan::new(
        vec![node1, node2],
        departure_position_m,
        departure_velocity_mps,
        departure_epoch,
    )
    .map_err(OpsError::from)
}

/// Plane-change cost at orbital speed (helper for `ChangePlane` blocks).
pub fn plane_change_dv(speed_mps: f64, angle_rad: f64) -> Result<f64, OpsError> {
    if !speed_mps.is_finite() || !angle_rad.is_finite() || speed_mps < 0.0 || angle_rad < 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    Ok(2.0 * speed_mps * (angle_rad / 2.0).sin())
}

/// Match a target velocity vector at one epoch (rendezvous arrival,
/// `MatchVelocity` block): a single node.
pub fn match_velocity(
    current_velocity_mps: DVec3,
    target_velocity_mps: DVec3,
    epoch: SimTime,
) -> Result<ManeuverNode, OpsError> {
    check_finite_vectors(&[current_velocity_mps, target_velocity_mps], epoch.0, 1.0)?;
    ManeuverNode::new(epoch, target_velocity_mps - current_velocity_mps)
        .map_err(OpsError::from)
}

/// Lambert rendezvous: departure burn from the current state plus arrival
/// burn matching the target state after `dt_s`. All vectors in the central
/// body's inertial frame. `predicted_miss_m` stays `None`: revalidate.
#[allow(clippy::too_many_arguments)]
pub fn lambert_rendezvous(
    departure_position_m: DVec3,
    departure_velocity_mps: DVec3,
    arrival_position_m: DVec3,
    arrival_velocity_mps: DVec3,
    time_of_flight_s: f64,
    mu_m3_s2: f64,
    departure_epoch: SimTime,
    short_way: bool,
) -> Result<ManeuverPlan, OpsError> {
    check_finite_vectors(
        &[
            departure_position_m,
            departure_velocity_mps,
            arrival_position_m,
            arrival_velocity_mps,
        ],
        departure_epoch.0,
        mu_m3_s2,
    )?;
    if !time_of_flight_s.is_finite() || time_of_flight_s <= 0.0 {
        return Err(OpsError::NonFiniteInput);
    }
    let arc = solve_lambert(
        departure_position_m,
        arrival_position_m,
        time_of_flight_s,
        mu_m3_s2,
        short_way,
    )?;
    let node1 = ManeuverNode::new(
        departure_epoch,
        arc.departure_velocity_mps - departure_velocity_mps,
    )?;
    let node2 = ManeuverNode::new(
        SimTime(departure_epoch.0 + time_of_flight_s),
        arrival_velocity_mps - arc.arrival_velocity_mps,
    )?;
    ManeuverPlan::new(
        vec![node1, node2],
        departure_position_m,
        departure_velocity_mps,
        departure_epoch,
    )
    .map_err(OpsError::from)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MU: f64 = 3.986_004_418e14;

    #[test]
    fn circularize_matches_vis_viva_at_apoapsis() {
        // Elliptical orbit, apoapsis state: circularize burn is exact there.
        let rp = 6_878_000.0;
        let ra = 2.0 * rp;
        let semi = 0.5 * (rp + ra);
        let v_apo = (MU * (2.0 / ra - 1.0 / semi)).sqrt();
        let node = circularize_at_apse(
            DVec3::new(-ra, 0.0, 0.0),
            DVec3::new(0.0, -v_apo, 0.0),
            MU,
            SimTime(0.0),
        )
        .unwrap();
        let expected = (MU / ra).sqrt() - v_apo;
        assert!((node.magnitude_mps() - expected) / expected <= 1.0e-9);
    }

    #[test]
    fn hohmann_plan_has_two_nodes_and_known_cost() {
        let r1 = 6_878_000.0;
        let r2 = 2.0 * r1;
        let semi = 0.5 * (r1 + r2);
        let speed = (MU / r1).sqrt();
        let plan = hohmann_transfer(
            DVec3::new(r1, 0.0, 0.0),
            DVec3::new(0.0, speed, 0.0),
            r2,
            MU,
            SimTime(1_000.0),
        )
        .unwrap();
        assert_eq!(plan.nodes.len(), 2);
        // Vis-viva derived in-test (no magic textbook numbers).
        let expected_dep = (MU * (2.0 / r1 - 1.0 / semi)).sqrt() - speed;
        let expected_arr = (MU / r2).sqrt() - (MU * (2.0 / r2 - 1.0 / semi)).sqrt();
        assert!((plan.nodes[0].magnitude_mps() - expected_dep) / expected_dep <= 1.0e-9);
        assert!((plan.nodes[1].magnitude_mps() - expected_arr) / expected_arr <= 1.0e-9);
        assert!((plan.total_dv_mps() - (expected_dep + expected_arr)).abs() <= 1.0e-9);
        assert!(plan.nodes[1].epoch.0 > plan.nodes[0].epoch.0);
        // Eccentric departure is refused, not silently mistreated.
        let fast = DVec3::new(0.0, speed * 1.2, 0.0);
        assert!(matches!(
            hohmann_transfer(DVec3::new(r1, 0.0, 0.0), fast, 2.0 * r1, MU, SimTime(0.0)),
            Err(OpsError::NotCircular { .. })
        ));
    }

    #[test]
    fn plane_change_helper_matches_textbook() {
        assert!((plane_change_dv(7_800.0, 0.1).unwrap() - 779.6).abs() <= 1.0);
    }
}
