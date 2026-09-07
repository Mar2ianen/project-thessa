use std::{error::Error, ops::Index, sync::Arc};

use anise::{constants::frames::EARTH_J2000, prelude::Almanac};
use glam::DVec3;
use hifitime::{Epoch, JD_J2000, Unit};
use nyx_space::{
    Spacecraft,
    cosmic::Orbit,
    dynamics::{OrbitalDynamics, SpacecraftDynamics},
    linalg::Vector3,
    propagators::Propagator,
};
use thessa_sim_core::{
    AdaptiveIntegratorConfig, BakedBody, BakedEphemeris, BodyId, GravityField, ImpulsiveBurn,
    KeplerOrbit, SimTime, SystemConfig, TestParticleState, propagate_adaptive,
    propagate_adaptive_with_burns,
};

const REFERENCE_MU_KM3_S2: f64 = 398_600.433;
const REFERENCE_MU_M3_S2: f64 = REFERENCE_MU_KM3_S2 * 1.0e9;

#[derive(Debug, Clone, Copy)]
struct Case {
    name: &'static str,
    semi_major_axis_km: f64,
    eccentricity: f64,
    inclination_deg: f64,
    raan_deg: f64,
    argument_of_periapsis_deg: f64,
    true_anomaly_deg: f64,
    durations_s: &'static [f64],
}

#[derive(Debug, Clone, Copy)]
struct Comparison {
    case: &'static str,
    duration_s: f64,
    initial_state_position_delta_m: f64,
    initial_state_velocity_delta_mps: f64,
    anise_at_epoch_position_error_m: f64,
    anise_at_epoch_velocity_error_mps: f64,
    nyx_propagator_position_error_m: f64,
    nyx_propagator_velocity_error_mps: f64,
    analytic_position_error_m: f64,
    analytic_velocity_error_mps: f64,
    accepted_steps: u64,
    rejected_steps: u64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let cases = [
        Case {
            name: "leo-eccentric-inclined",
            semi_major_axis_km: 8_000.0,
            eccentricity: 0.2,
            inclination_deg: 30.0,
            raan_deg: 60.0,
            argument_of_periapsis_deg: 60.0,
            true_anomaly_deg: 0.0,
            durations_s: &[6.0 * 3_600.0, 86_400.0, 3.0 * 86_400.0],
        },
        Case {
            name: "geo-mild-eccentricity",
            semi_major_axis_km: 42_164.0,
            eccentricity: 0.05,
            inclination_deg: 0.1,
            raan_deg: 15.0,
            argument_of_periapsis_deg: 25.0,
            true_anomaly_deg: 180.0,
            durations_s: &[86_400.0, 7.0 * 86_400.0],
        },
    ];

    println!(
        "reference=Nyx 2.5.3 DP78; analytic_diagnostic=ANISE 0.10.6 at_epoch; mu={REFERENCE_MU_KM3_S2:.3} km^3/s^2"
    );
    println!(
        "case,duration_s,initial_state_position_delta_m,initial_state_velocity_delta_mps,anise_at_epoch_position_error_m,anise_at_epoch_velocity_error_mps,nyx_propagator_position_error_m,nyx_propagator_velocity_error_mps,analytic_position_error_m,analytic_velocity_error_mps,accepted_steps,rejected_steps"
    );
    for case in cases {
        for &duration_s in case.durations_s {
            let comparison = compare_case(case, duration_s)?;
            validate_comparison(comparison)?;
            println!(
                "{},{:.3},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{:.9e},{},{}",
                comparison.case,
                comparison.duration_s,
                comparison.initial_state_position_delta_m,
                comparison.initial_state_velocity_delta_mps,
                comparison.anise_at_epoch_position_error_m,
                comparison.anise_at_epoch_velocity_error_mps,
                comparison.nyx_propagator_position_error_m,
                comparison.nyx_propagator_velocity_error_mps,
                comparison.analytic_position_error_m,
                comparison.analytic_velocity_error_mps,
                comparison.accepted_steps,
                comparison.rejected_steps,
            );
        }
    }
    validate_lagrange_suite()?;
    validate_maneuver_sequence()?;
    validate_design_system()?;
    println!("validation=PASS (Nyx numerical and internal analytic gates)");
    Ok(())
}

fn compare_case(case: Case, duration_s: f64) -> Result<Comparison, Box<dyn Error>> {
    let epoch = Epoch::from_mjd_tai(JD_J2000);
    let frame = EARTH_J2000.with_mu_km3_s2(REFERENCE_MU_KM3_S2);
    let reference_initial = Orbit::keplerian(
        case.semi_major_axis_km,
        case.eccentricity,
        case.inclination_deg,
        case.raan_deg,
        case.argument_of_periapsis_deg,
        case.true_anomaly_deg,
        epoch,
        frame,
    );
    let reference_final = reference_initial.at_epoch(epoch + duration_s * Unit::Second)?;
    let nyx_final = Propagator::default_dp78(SpacecraftDynamics::new(OrbitalDynamics::two_body()))
        .with(
            Spacecraft::from(reference_initial),
            std::sync::Arc::new(Almanac::default()),
        )
        .quiet()
        .for_duration(duration_s * Unit::Second)?;

    let initial = TestParticleState {
        position: vec3_from_km(&reference_initial.radius_km) * 1_000.0,
        velocity: vec3_from_km(&reference_initial.velocity_km_s) * 1_000.0,
    };
    let candidate_orbit = KeplerOrbit::new(
        REFERENCE_MU_M3_S2,
        case.semi_major_axis_km * 1_000.0,
        case.eccentricity,
        case.inclination_deg.to_radians(),
        case.raan_deg.to_radians(),
        case.argument_of_periapsis_deg.to_radians(),
        mean_anomaly_from_true_anomaly(case.true_anomaly_deg, case.eccentricity),
    )?;
    let (candidate_initial_position, candidate_initial_velocity) =
        candidate_orbit.state_relative_at(SimTime::EPOCH)?;
    let (analytic_position, analytic_velocity) =
        candidate_orbit.state_relative_at(SimTime::EPOCH.offset(duration_s))?;
    let ephemeris = BakedEphemeris::new(
        "NYX_COMPARE_EPOCH",
        vec![BakedBody::fixed(
            BodyId(0),
            "reference-central-body",
            REFERENCE_MU_M3_S2,
            0.0,
        )],
    )?;
    let field = GravityField::from_ephemeris(&ephemeris);
    let result = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        duration_s,
        AdaptiveIntegratorConfig {
            initial_step_s: 30.0,
            min_step_s: 1.0e-6,
            max_step_s: 300.0,
            absolute_position_tolerance_m: 1.0e-4,
            absolute_velocity_tolerance_mps: 1.0e-7,
            relative_tolerance: 1.0e-12,
            max_steps: 1_000_000,
        },
    )?;

    let expected_position = vec3_from_km(&reference_final.radius_km) * 1_000.0;
    let expected_velocity = vec3_from_km(&reference_final.velocity_km_s) * 1_000.0;
    let nyx_propagator_position = vec3_from_km(&nyx_final.orbit.radius_km) * 1_000.0;
    let nyx_propagator_velocity = vec3_from_km(&nyx_final.orbit.velocity_km_s) * 1_000.0;
    Ok(Comparison {
        case: case.name,
        duration_s,
        initial_state_position_delta_m: initial.position.distance(candidate_initial_position),
        initial_state_velocity_delta_mps: initial.velocity.distance(candidate_initial_velocity),
        anise_at_epoch_position_error_m: result.state.position.distance(expected_position),
        anise_at_epoch_velocity_error_mps: result.state.velocity.distance(expected_velocity),
        nyx_propagator_position_error_m: result.state.position.distance(nyx_propagator_position),
        nyx_propagator_velocity_error_mps: result.state.velocity.distance(nyx_propagator_velocity),
        analytic_position_error_m: result.state.position.distance(analytic_position),
        analytic_velocity_error_mps: result.state.velocity.distance(analytic_velocity),
        accepted_steps: result.stats.accepted_steps,
        rejected_steps: result.stats.rejected_steps,
    })
}

fn validate_comparison(comparison: Comparison) -> Result<(), Box<dyn Error>> {
    if comparison.initial_state_position_delta_m > 1.0e-6
        || comparison.initial_state_velocity_delta_mps > 1.0e-9
    {
        return Err(format!(
            "{} initial state conversion mismatch: {:.3e} m, {:.3e} m/s",
            comparison.case,
            comparison.initial_state_position_delta_m,
            comparison.initial_state_velocity_delta_mps,
        )
        .into());
    }

    // The gate scales with arc length because this first comparison uses two
    // different adaptive RK implementations. It is deliberately tighter than
    // the diagnostic output's observed multi-day drift.
    let position_gate_m = 1.0e-3 + comparison.duration_s * 1.0e-5;
    let velocity_gate_mps = 1.0e-6 + comparison.duration_s * 1.0e-8;
    if comparison.nyx_propagator_position_error_m > position_gate_m
        || comparison.nyx_propagator_velocity_error_mps > velocity_gate_mps
        || comparison.analytic_position_error_m > position_gate_m
        || comparison.analytic_velocity_error_mps > velocity_gate_mps
    {
        return Err(format!(
            "{} exceeds candidate validation gate after {:.0} s: Nyx {:.3e} m / {:.3e} m/s; analytic {:.3e} m / {:.3e} m/s; gates {:.3e} m / {:.3e} m/s",
            comparison.case,
            comparison.duration_s,
            comparison.nyx_propagator_position_error_m,
            comparison.nyx_propagator_velocity_error_mps,
            comparison.analytic_position_error_m,
            comparison.analytic_velocity_error_mps,
            position_gate_m,
            velocity_gate_mps,
        )
        .into());
    }
    Ok(())
}

fn mean_anomaly_from_true_anomaly(true_anomaly_deg: f64, eccentricity: f64) -> f64 {
    let true_anomaly = true_anomaly_deg.to_radians();
    let eccentric_anomaly = 2.0
        * ((1.0 - eccentricity).sqrt() * (true_anomaly * 0.5).sin())
            .atan2((1.0 + eccentricity).sqrt() * (true_anomaly * 0.5).cos());
    eccentric_anomaly - eccentricity * eccentric_anomaly.sin()
}

fn vec3_from_km<T>(vector: &T) -> DVec3
where
    T: Index<usize, Output = f64>,
{
    DVec3::new(vector[0], vector[1], vector[2])
}

fn validate_lagrange_suite() -> Result<(), Box<dyn Error>> {
    let mu_primary = 1.0e14;
    let mu_secondary = 1.0e12;
    let separation_m = 1.0e7;
    let total_mu = mu_primary + mu_secondary;
    let ephemeris = binary_ephemeris(mu_primary, mu_secondary, separation_m)?;
    let field = GravityField::from_ephemeris(&ephemeris);
    let primary = ephemeris.body_state(BodyId(1), SimTime::EPOCH)?;
    let secondary = ephemeris.body_state(BodyId(2), SimTime::EPOCH)?;
    let barycenter = DVec3::ZERO;
    let axis = (secondary.position_inertial - primary.position_inertial).normalize();
    let perpendicular = DVec3::new(-axis.y, axis.x, 0.0);
    let angular_rate = (total_mu / separation_m.powi(3)).sqrt();
    let geometry = RotatingBinary {
        barycenter,
        axis,
        perpendicular,
        angular_rate,
        separation_m,
    };
    let mass_ratio = mu_secondary / total_mu;
    let collinear = [
        ("L1", solve_collinear_lagrange(1.0 - mass_ratio - (mass_ratio / 3.0).cbrt(), mass_ratio)?),
        ("L2", solve_collinear_lagrange(1.0 - mass_ratio + (mass_ratio / 3.0).cbrt(), mass_ratio)?),
        ("L3", solve_collinear_lagrange(-1.0 - 5.0 * mass_ratio / 12.0, mass_ratio)?),
    ];

    println!("lagrange_case,point,normalized_acceleration_residual");
    for (name, x) in collinear {
        let residual = lagrange_residual(
            &field,
            geometry,
            x,
            0.0,
        )?;
        println!("circular-restricted-three-body,{name},{residual:.6e}");
        if residual > 1.0e-11 {
            return Err(format!("{name} residual is too large: {residual:.3e}").into());
        }
    }

    let l4_l5_duration_s = 5.0 * std::f64::consts::TAU / angular_rate;
    for (name, y) in [("L4", 3.0_f64.sqrt() * 0.5), ("L5", -3.0_f64.sqrt() * 0.5)] {
        let x = 0.5 - mass_ratio;
        let initial_position = geometry.barycenter
            + geometry.axis * (x * geometry.separation_m)
            + geometry.perpendicular * (y * geometry.separation_m);
        let initial = TestParticleState {
            position: initial_position,
            velocity: rotating_velocity(initial_position, geometry.angular_rate),
        };
        let result = propagate_adaptive(
            &field,
            initial,
            SimTime::EPOCH,
            l4_l5_duration_s,
            AdaptiveIntegratorConfig {
                initial_step_s: 10.0,
                min_step_s: 1.0e-7,
                max_step_s: 120.0,
                absolute_position_tolerance_m: 1.0e-4,
                absolute_velocity_tolerance_mps: 1.0e-7,
                relative_tolerance: 1.0e-11,
                max_steps: 1_000_000,
            },
        )?;
        let expected_position = rotate_z(initial_position, geometry.angular_rate * l4_l5_duration_s);
        let expected_velocity = rotating_velocity(expected_position, geometry.angular_rate);
        let position_error_m = result.state.position.distance(expected_position);
        let velocity_error_mps = result.state.velocity.distance(expected_velocity);
        println!(
            "circular-restricted-three-body,{name}-5-periods,{position_error_m:.6e} m / {velocity_error_mps:.6e} mps"
        );
        if position_error_m > 1.0 || velocity_error_mps > 1.0e-3 {
            return Err(format!(
                "{name} did not remain on its rotating equilibrium: {position_error_m:.3e} m, {velocity_error_mps:.3e} m/s"
            )
            .into());
        }
    }
    Ok(())
}

fn binary_ephemeris(
    mu_primary: f64,
    mu_secondary: f64,
    separation_m: f64,
) -> Result<BakedEphemeris, Box<dyn Error>> {
    let total_mu = mu_primary + mu_secondary;
    let mean_motion = (total_mu / separation_m.powi(3)).sqrt();
    let primary_orbit = KeplerOrbit::new(
        total_mu,
        separation_m * mu_secondary / total_mu,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )?
    .with_mean_motion(mean_motion)?;
    let secondary_orbit = KeplerOrbit::new(
        total_mu,
        separation_m * mu_primary / total_mu,
        0.0,
        0.0,
        0.0,
        0.0,
        std::f64::consts::PI,
    )?
    .with_mean_motion(mean_motion)?;
    Ok(BakedEphemeris::new(
        "LAGRANGE_VALIDATION_EPOCH",
        vec![
            BakedBody::synthetic_barycenter(BodyId(0), "barycenter", total_mu, None, None),
            BakedBody::orbital(
                BodyId(1),
                "primary",
                mu_primary,
                0.0,
                BodyId(0),
                primary_orbit,
            ),
            BakedBody::orbital(
                BodyId(2),
                "secondary",
                mu_secondary,
                0.0,
                BodyId(0),
                secondary_orbit,
            ),
        ],
    )?)
}

fn solve_collinear_lagrange(mut x: f64, mass_ratio: f64) -> Result<f64, Box<dyn Error>> {
    for _ in 0..64 {
        let value = collinear_equilibrium_function(x, mass_ratio);
        let h = 1.0e-6;
        let derivative = (collinear_equilibrium_function(x + h, mass_ratio)
            - collinear_equilibrium_function(x - h, mass_ratio))
            / (2.0 * h);
        if !derivative.is_finite() || derivative.abs() < 1.0e-14 {
            return Err(format!("collinear Lagrange solve became singular at x={x}").into());
        }
        let next = x - value / derivative;
        if (next - x).abs() < 1.0e-14 {
            return Ok(next);
        }
        x = next;
    }
    Err("collinear Lagrange solve did not converge".into())
}

fn collinear_equilibrium_function(x: f64, mass_ratio: f64) -> f64 {
    let primary_x = -mass_ratio;
    let secondary_x = 1.0 - mass_ratio;
    x - (1.0 - mass_ratio) * (x - primary_x) / (x - primary_x).abs().powi(3)
        - mass_ratio * (x - secondary_x) / (x - secondary_x).abs().powi(3)
}

#[derive(Debug, Clone, Copy)]
struct RotatingBinary {
    barycenter: DVec3,
    axis: DVec3,
    perpendicular: DVec3,
    angular_rate: f64,
    separation_m: f64,
}

fn lagrange_residual(
    field: &GravityField<'_>,
    geometry: RotatingBinary,
    x: f64,
    y: f64,
) -> Result<f64, Box<dyn Error>> {
    let position = geometry.barycenter
        + geometry.axis * (x * geometry.separation_m)
        + geometry.perpendicular * (y * geometry.separation_m);
    let expected = -geometry.angular_rate.powi(2) * (position - geometry.barycenter);
    let actual = field.acceleration(position, SimTime::EPOCH)?;
    Ok((actual - expected).length() / (geometry.angular_rate.powi(2) * geometry.separation_m))
}

fn rotating_velocity(position: DVec3, angular_rate: f64) -> DVec3 {
    DVec3::new(-angular_rate * position.y, angular_rate * position.x, 0.0)
}

fn rotate_z(vector: DVec3, angle: f64) -> DVec3 {
    let (sin_angle, cos_angle) = angle.sin_cos();
    DVec3::new(
        vector.x * cos_angle - vector.y * sin_angle,
        vector.x * sin_angle + vector.y * cos_angle,
        vector.z,
    )
}

fn validate_maneuver_sequence() -> Result<(), Box<dyn Error>> {
    let epoch = Epoch::from_mjd_tai(JD_J2000);
    let initial_orbit = Orbit::keplerian(
        7_000.0,
        0.001,
        28.5,
        40.0,
        25.0,
        15.0,
        epoch,
        EARTH_J2000.with_mu_km3_s2(REFERENCE_MU_KM3_S2),
    );
    let initial = TestParticleState {
        position: vec3_from_km(&initial_orbit.radius_km) * 1_000.0,
        velocity: vec3_from_km(&initial_orbit.velocity_km_s) * 1_000.0,
    };
    let burns = vec![
        ImpulsiveBurn {
            time_s: 900.0,
            delta_v_mps: DVec3::new(0.0, 35.0, 8.0),
        },
        ImpulsiveBurn {
            time_s: 3_600.0,
            delta_v_mps: DVec3::new(-18.0, 4.0, 12.0),
        },
        ImpulsiveBurn {
            time_s: 7_200.0,
            delta_v_mps: DVec3::new(0.0, -46.0, -6.0),
        },
        ImpulsiveBurn {
            time_s: 14_000.0,
            delta_v_mps: DVec3::new(27.0, 11.0, 0.0),
        },
        ImpulsiveBurn {
            time_s: 24_000.0,
            delta_v_mps: DVec3::new(-21.0, 0.0, 15.0),
        },
        ImpulsiveBurn {
            time_s: 40_000.0,
            delta_v_mps: DVec3::new(0.0, 22.0, -13.0),
        },
        ImpulsiveBurn {
            time_s: 60_000.0,
            delta_v_mps: DVec3::new(16.0, -25.0, 7.0),
        },
        ImpulsiveBurn {
            time_s: 80_000.0,
            delta_v_mps: DVec3::new(-9.0, 13.0, -11.0),
        },
    ];
    let duration_s = 120_000.0;
    let ephemeris = BakedEphemeris::new(
        "MANEUVER_VALIDATION_EPOCH",
        vec![BakedBody::fixed(
            BodyId(0),
            "reference-central-body",
            REFERENCE_MU_M3_S2,
            0.0,
        )],
    )?;
    let candidate = propagate_adaptive_with_burns(
        &GravityField::from_ephemeris(&ephemeris),
        initial,
        SimTime::EPOCH,
        duration_s,
        &burns,
        AdaptiveIntegratorConfig {
            initial_step_s: 30.0,
            min_step_s: 1.0e-6,
            max_step_s: 300.0,
            absolute_position_tolerance_m: 1.0e-4,
            absolute_velocity_tolerance_mps: 1.0e-7,
            relative_tolerance: 1.0e-12,
            max_steps: 1_000_000,
        },
    )?;
    let reference = propagate_nyx_with_burns(initial_orbit, duration_s, &burns)?;
    let reference_position = vec3_from_km(&reference.radius_km) * 1_000.0;
    let reference_velocity = vec3_from_km(&reference.velocity_km_s) * 1_000.0;
    let position_error_m = candidate.state.position.distance(reference_position);
    let velocity_error_mps = candidate.state.velocity.distance(reference_velocity);
    println!(
        "maneuver-sequence,burns={},duration_s={duration_s:.0},position_error_m={position_error_m:.6e},velocity_error_mps={velocity_error_mps:.6e},accepted_steps={},rejected_steps={}",
        burns.len(),
        candidate.stats.accepted_steps,
        candidate.stats.rejected_steps,
    );
    if position_error_m > 5.0 || velocity_error_mps > 5.0e-3 {
        return Err(format!(
            "multi-maneuver candidate diverged from Nyx: {position_error_m:.3e} m, {velocity_error_mps:.3e} m/s"
        )
        .into());
    }
    Ok(())
}

fn propagate_nyx_with_burns(
    mut orbit: Orbit,
    duration_s: f64,
    burns: &[ImpulsiveBurn],
) -> Result<Orbit, Box<dyn Error>> {
    let mut elapsed_s = 0.0;
    for burn in burns {
        orbit = propagate_nyx_coast(orbit, burn.time_s - elapsed_s)?;
        orbit = orbit.with_dv_km_s(Vector3::new(
            burn.delta_v_mps.x / 1_000.0,
            burn.delta_v_mps.y / 1_000.0,
            burn.delta_v_mps.z / 1_000.0,
        ));
        elapsed_s = burn.time_s;
    }
    propagate_nyx_coast(orbit, duration_s - elapsed_s)
}

fn propagate_nyx_coast(orbit: Orbit, duration_s: f64) -> Result<Orbit, Box<dyn Error>> {
    if duration_s == 0.0 {
        return Ok(orbit);
    }
    let spacecraft = Propagator::default_dp78(SpacecraftDynamics::new(OrbitalDynamics::two_body()))
        .with(Spacecraft::from(orbit), Arc::new(Almanac::default()))
        .quiet()
        .for_duration(duration_s * Unit::Second)?;
    Ok(spacecraft.orbit)
}

fn validate_design_system() -> Result<(), Box<dyn Error>> {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))?;
    let ephemeris = config.bake()?;
    if ephemeris.bodies.len() != 24 || ephemeris.gravity_sources().count() != 22 {
        return Err("design system body/source counts changed unexpectedly".into());
    }
    for seconds in [0.0, 86_400.0, 30.0 * 86_400.0] {
        for body in &ephemeris.bodies {
            let state = ephemeris.body_state(body.id, SimTime(seconds))?;
            if !state.position_inertial.is_finite() || !state.velocity_inertial.is_finite() {
                return Err(format!("non-finite design-system state for {}", body.name).into());
            }
        }
    }

    let nered_id = ephemeris.body_id("nereid").ok_or("missing nereid")?;
    let borea_id = ephemeris.body_id("borea").ok_or("missing borea")?;
    let halo_id = ephemeris.body_id("halo").ok_or("missing halo")?;
    let nered = ephemeris.body(nered_id)?;
    let borea = ephemeris.body(borea_id)?;
    let nered_state = ephemeris.body_state(nered_id, SimTime::EPOCH)?;
    let borea_state = ephemeris.body_state(borea_id, SimTime::EPOCH)?;
    let halo_state = ephemeris.body_state(halo_id, SimTime::EPOCH)?;
    let relative_position = borea_state.position_inertial - nered_state.position_inertial;
    let relative_velocity = borea_state.velocity_inertial - nered_state.velocity_inertial;
    let separation_m = relative_position.length();
    let angular_rate = relative_position.cross(relative_velocity).length() / separation_m.powi(2);
    let pair_barycenter = (nered_state.position_inertial * nered.mu
        + borea_state.position_inertial * borea.mu)
        / (nered.mu + borea.mu);
    let pair_ephemeris = BakedEphemeris::new(
        "DESIGN_SYSTEM_L4_PAIR",
        ephemeris
            .bodies
            .iter()
            .cloned()
            .map(|mut body| {
                body.gravity_source = body.id == nered_id || body.id == borea_id;
                body
            })
            .collect(),
    )?;
    let pair_field = GravityField::from_ephemeris(&pair_ephemeris);
    let actual = pair_field.acceleration(halo_state.position_inertial, SimTime::EPOCH)?;
    let expected = -angular_rate.powi(2) * (halo_state.position_inertial - pair_barycenter);
    let l4_residual = (actual - expected).length() / (angular_rate.powi(2) * separation_m);
    let l4_status = if l4_residual <= 1.0e-10 {
        "exact"
    } else {
        "diagnostic-approximation"
    };
    println!(
        "design-system,bodies=24,gravity_sources=22,halo_l4_residual={l4_residual:.6e},halo_l4_status={l4_status}"
    );

    let thessa_id = ephemeris.body_id("thessa").ok_or("missing thessa")?;
    let thessa = ephemeris.body(thessa_id)?;
    let thessa_state = ephemeris.body_state(thessa_id, SimTime::EPOCH)?;
    let orbit_radius_m = thessa.radius_m + 500_000.0;
    let initial = TestParticleState {
        position: thessa_state.position_inertial + DVec3::new(orbit_radius_m, 0.0, 0.0),
        velocity: thessa_state.velocity_inertial
            + DVec3::new(0.0, (thessa.mu / orbit_radius_m).sqrt(), 0.0),
    };
    let burns = vec![
        ImpulsiveBurn {
            time_s: 0.0,
            delta_v_mps: DVec3::new(0.0, 2.0, 0.0),
        },
        ImpulsiveBurn {
            time_s: 3_600.0,
            delta_v_mps: DVec3::new(0.5, 0.0, 0.2),
        },
        ImpulsiveBurn {
            time_s: 12_000.0,
            delta_v_mps: DVec3::new(-0.4, 0.3, 0.0),
        },
        ImpulsiveBurn {
            time_s: 24_000.0,
            delta_v_mps: DVec3::new(0.0, -0.8, 0.4),
        },
        ImpulsiveBurn {
            time_s: 48_000.0,
            delta_v_mps: DVec3::new(0.6, 0.0, -0.3),
        },
        ImpulsiveBurn {
            time_s: 72_000.0,
            delta_v_mps: DVec3::new(-0.3, 0.4, 0.1),
        },
        ImpulsiveBurn {
            time_s: 108_000.0,
            delta_v_mps: DVec3::new(0.0, 0.6, -0.2),
        },
        ImpulsiveBurn {
            time_s: 144_000.0,
            delta_v_mps: DVec3::new(-0.2, -0.3, 0.2),
        },
    ];
    let field = GravityField::from_ephemeris(&ephemeris);
    let simulation_config = AdaptiveIntegratorConfig {
        initial_step_s: 10.0,
        min_step_s: 1.0e-4,
        max_step_s: 600.0,
        absolute_position_tolerance_m: 1.0e-1,
        absolute_velocity_tolerance_mps: 1.0e-4,
        relative_tolerance: 1.0e-10,
        max_steps: 2_000_000,
    };
    let first = propagate_adaptive_with_burns(
        &field,
        initial,
        SimTime::EPOCH,
        172_800.0,
        &burns,
        simulation_config,
    )?;
    let second = propagate_adaptive_with_burns(
        &field,
        initial,
        SimTime::EPOCH,
        172_800.0,
        &burns,
        simulation_config,
    )?;
    if first != second || !first.state.position.is_finite() || !first.state.velocity.is_finite() {
        return Err("design-system maneuver replay is not deterministic and finite".into());
    }
    println!(
        "design-system,thessa-vehicle,burns={},duration_s=172800,accepted_steps={},rejected_steps={}",
        burns.len(),
        first.stats.accepted_steps,
        first.stats.rejected_steps,
    );
    Ok(())
}
