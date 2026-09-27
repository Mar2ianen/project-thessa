use super::*;
use thessa_sim_core::SystemConfig;

#[test]
fn trim_adaptive_second_pass_matches_full_solve_within_envelope() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut full = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
    let mut adaptive = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
    full.force_trim_two_pass = true;
    // Sustained maneuvering climb: attitude-hold capture plus steady
    // manual rate commands exercise the trim loop under load, including
    // transients where the second pass must still fire.
    for flight in [&mut full, &mut adaptive] {
        flight.control_input = DVec3::new(0.3, 0.1, -0.2);
        flight.throttle = 0.8;
    }
    full.advance(&ephemeris, ControlMode::Navball, 10.0)
        .expect("full flight");
    adaptive
        .advance(&ephemeris, ControlMode::Navball, 10.0)
        .expect("adaptive flight");
    assert!(full.flight_error.is_none(), "{:?}", full.flight_error);
    assert!(
        adaptive.flight_error.is_none(),
        "{:?}",
        adaptive.flight_error
    );
    assert_eq!(full.steps_this_frame, adaptive.steps_this_frame);
    assert!(adaptive.trim_solves > 0);
    assert!(
        adaptive.trim_second_passes < adaptive.trim_solves,
        "no skip fired: {}/{}",
        adaptive.trim_second_passes,
        adaptive.trim_solves,
    );
    let position_drift =
        (full.state.position_inertial_m - adaptive.state.position_inertial_m).length();
    let velocity_drift =
        (full.state.velocity_inertial_mps - adaptive.state.velocity_inertial_mps).length();
    let attitude_drift = (full.state.orientation_body_to_inertial.inverse()
        * adaptive.state.orientation_body_to_inertial)
        .to_scaled_axis()
        .length();
    // Envelope vs decision scales: 1 cm is 500x below the 5 m batch
    // adoption gate and terrain clearance; 1e-6 rad is 1000x below one
    // tick of SAS attitude command (1.3e-3 rad). Calibrated drift over
    // this 10 s maneuvering climb: 3.7e-5 m, 6.3e-7 m/s, 0 rad.
    assert!(position_drift <= 1.0e-2, "pos drift {position_drift:.6e} m");
    assert!(
        velocity_drift <= 1.0e-4,
        "vel drift {velocity_drift:.6e} m/s"
    );
    assert!(
        attitude_drift <= 1.0e-6,
        "att drift {attitude_drift:.6e} rad"
    );
    eprintln!(
        "trim A/B: solves={} second_passes={} skip_rate={:.2} pos_drift={:.6e} m vel_drift={:.6e} m/s att_drift={:.6e} rad",
        adaptive.trim_solves,
        adaptive.trim_second_passes,
        1.0 - adaptive.trim_second_passes as f64 / adaptive.trim_solves as f64,
        position_drift,
        velocity_drift,
        attitude_drift,
    );
}
