//! Native adapter regression, kept beside the existing scene/input fixtures.
use super::*;

fn scene() -> Scene {
    let mut scene = Scene::load(Default::default()).unwrap();
    scene.enable_live_flight().unwrap();
    scene
}

#[test]
fn flyby_engagement_and_frame_updates_only_write_existing_pilot_demands() {
    let mut scene = scene();
    scene.input.cut();
    scene.input.engine_active = false;
    scene.control_mode = thessa_flight_authority::ControlMode::MouseAim;
    let before = scene.flight.as_ref().unwrap().state;
    let time = scene.flight.as_ref().unwrap().flight_time_s;
    scene.set_flyby(true).unwrap();
    assert!(
        scene.flyby.active
            && scene.input.engine_active
            && scene.input.sas_enabled
            && scene.camera.chase()
    );
    assert_eq!(
        scene.control_mode,
        thessa_flight_authority::ControlMode::Navball
    );
    scene.push_live_input(0.1, false, false);
    assert_eq!(scene.flight.as_ref().unwrap().state, before);
    assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, time);
    assert!((scene.input.throttle - 0.055).abs() < 1e-12);
    let wire = thessa_flight_net::presentation::input_for_server(
        scene.flight.as_ref().unwrap(),
        scene.control_mode,
        scene.input.throttle,
        scene.input.engine_active,
        vec![],
    );
    wire.validate().unwrap();
    assert_eq!(wire.control_input, scene.control_axes().to_array());
    assert_eq!(
        scene.flight.as_ref().unwrap().control_input,
        scene.control_axes()
    );
    assert!(
        scene
            .flight_telemetry()
            .unwrap()
            .contains("flyby_active=true")
    );
    scene.set_flyby(false).unwrap();
    assert_eq!(scene.control_axes(), DVec3::ZERO);
    let throttle = scene.input.throttle;
    scene.push_live_input(0.1, false, false);
    assert_eq!(
        scene.input.throttle, throttle,
        "cancel cannot resume from a stale auto-engage flag"
    );
}

#[test]
fn flyby_takeover_cancels_even_for_opposing_keys_and_preserves_manual_input() {
    let mut scene = scene();
    scene.set_flyby(true).unwrap();
    scene.input.keys.nose_up = true;
    scene.input.keys.nose_down = true;
    scene.push_live_input(0.1, false, false);
    assert!(!scene.flyby.active);
    assert_eq!(scene.control_axes(), DVec3::ZERO);
    scene.input.keys = Default::default();
    scene.set_flyby(true).unwrap();
    scene.input.keys.yaw_right = true;
    scene.push_live_input(0.1, false, false);
    assert!(!scene.flyby.active);
    assert_eq!(scene.flight.as_ref().unwrap().control_input, DVec3::Y);
    scene.input.keys = Default::default();
    scene.set_flyby(true).unwrap();
    let throttle = scene.input.throttle;
    scene.push_live_input(0.1, false, true);
    assert!(!scene.flyby.active);
    assert!((scene.input.throttle - (throttle - 0.055).clamp(0.0, 1.0)).abs() < 1e-12);
}

#[test]
fn flyby_map_focus_help_pause_warp_reset_and_mode_changes_do_not_auto_resume() {
    let mut scene = scene();
    for action in 0..6 {
        scene.request_pause(false);
        scene.request_warp(1.0).unwrap();
        scene.set_flyby(true).unwrap();
        match action {
            0 => scene.release_manual_input(),
            1 => scene.request_pause(true),
            2 => {
                scene.request_warp(2.0).unwrap();
            }
            3 => {
                scene.reset_flight().unwrap();
            }
            4 => scene.cycle_control_mode(),
            _ => scene.toggle_sas(),
        }
        assert!(!scene.flyby.active);
        scene.push_live_input(0.0, false, false);
        assert!(!scene.flyby.active);
    }
    #[cfg(feature = "hud")]
    {
        scene.set_flyby(true).unwrap();
        scene.set_map_open(true);
        assert!(!scene.flyby.active);
        assert!(scene.set_flyby(true).is_err());
        scene.set_map_open(false);
        assert!(!scene.flyby.active);
    }
}

#[test]
fn flyby_rejects_foreign_body_vacuum_and_stopped_flight_without_moving_state() {
    let mut scene = scene();
    scene.request_pause(true);
    assert!(scene.set_flyby(true).is_err());
    scene.request_pause(false);
    let e = scene.celestial.ephemeris();
    let id = e.body_id("nereid").unwrap();
    let body = e.body_state(id, thessa_sim_core::SimTime::EPOCH).unwrap();
    let radius = e.body(id).unwrap().radius_m;
    let flight = scene.flight.as_mut().unwrap();
    flight.state.position_inertial_m = body.position_inertial + DVec3::X * (radius + 500000.0);
    flight.state.velocity_inertial_mps = body.velocity_inertial;
    let before = flight.state;
    assert!(scene.set_flyby(true).is_err());
    assert_eq!(scene.flight.as_ref().unwrap().state, before);
    scene.reset_flight().unwrap();
    let flight = scene.flight.as_mut().unwrap();
    let origin = scene
        .celestial
        .ephemeris()
        .body_state(
            flight.reference_body,
            thessa_sim_core::SimTime(flight.flight_time_s),
        )
        .unwrap();
    flight.state.position_inertial_m =
        origin.position_inertial + DVec3::X * (flight.planet_radius_m + 2_000_000.0);
    assert!(
        scene.set_flyby(true).is_err(),
        "vacuum coast is not the photo-flight fixture"
    );
    scene.reset_flight().unwrap();
    scene.set_flyby(true).unwrap();
    scene.flight.as_mut().unwrap().flight_error = Some("fixture stopped".into());
    scene.push_live_input(0.1, false, false);
    assert!(!scene.flyby.active);
}

#[test]
#[cfg(feature = "hud")]
fn flyby_hud_edits_keep_passive_frames_and_do_not_rearm_after_takeover() {
    let mut scene = scene();
    let mut controls = crate::hud::Controls::from_scene(&scene);
    controls.flyby = true;
    controls.apply(&mut scene);
    assert!(scene.flyby.active);
    scene.push_live_input(0.1, false, false);
    crate::hud::Controls::from_scene(&scene).apply(&mut scene);
    assert!(
        scene.flyby.active,
        "passive HUD frames must not cancel guidance"
    );
    let mut controls = crate::hud::Controls::from_scene(&scene);
    controls.throttle = 0.0;
    controls.apply(&mut scene);
    scene.push_live_input(0.1, false, false);
    assert!(
        !scene.flyby.active && scene.input.throttle == 0.0,
        "a stale true checkbox must not rearm after throttle takeover"
    );
    let mut controls = crate::hud::Controls::from_scene(&scene);
    controls.flyby = true;
    controls.apply(&mut scene);
    assert!(scene.flyby.active);
    let mut controls = crate::hud::Controls::from_scene(&scene);
    controls.sas = false;
    controls.apply(&mut scene);
    assert!(!scene.flyby.active && !scene.input.sas_enabled);
}
