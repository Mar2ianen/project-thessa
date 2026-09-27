use super::*;

fn sim_with_terrain() -> Sim {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
    sim.init_launch_site().expect("launch site");
    sim.register("pilot");
    sim
}

fn input(commands: Vec<Command>) -> ClientInput {
    ClientInput {
        tick: 0,
        control_input: [0.0; 3],
        control_mode: ControlMode::Direct,
        sas_target_xyzw: [0.0, 0.0, 0.0, 1.0],
        throttle: 0.0,
        engine_active: false,
        sas_enabled: false,
        rcs_enabled: false,
        reaction_wheels_enabled: true,
        gear_down: false,
        parachutes_armed: false,
        commands,
    }
}

#[test]
fn reset_relaunches_at_canonical_site_and_forces_snapshot() {
    let mut sim = sim_with_terrain();
    // Fly away from the launch state first: full throttle climb.
    // (Plain inputs force no snapshot; the return is a force flag.)
    let mut climb = input(vec![]);
    climb.throttle = 1.0;
    climb.engine_active = true;
    assert!(!sim.apply_input("pilot", &climb));
    sim.advance_chunk(5.0).expect("climb");
    let displaced = sim.authority.state.position_inertial_m;
    // Reset preserves the clock but rebuilds the launch state.
    let before = sim.authority.flight_time_s;
    let forced = sim.apply_input("pilot", &input(vec![Command::Reset]));
    assert!(forced, "reset must force a prompt snapshot");
    assert_eq!(sim.authority.flight_time_s, before);
    assert_ne!(sim.authority.state.position_inertial_m, displaced);
    assert!(sim.authority.flight_error.is_none());
    // Second reset from the pad is idempotent on state.
    let pad = sim.authority.state.position_inertial_m;
    assert!(sim.apply_input("pilot", &input(vec![Command::Reset])));
    assert_eq!(sim.authority.state.position_inertial_m, pad);
}

#[test]
fn reset_without_terrain_is_a_quiet_noop() {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    // Bench-style sims never init the launch site: Reset must not fail
    // the driver, it just does nothing.
    let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
    sim.register("pilot");
    assert!(!sim.apply_input("pilot", &input(vec![Command::Reset])));
}

#[test]
fn reset_survives_input_coalescing_as_event() {
    // Reset is edge/event-like: every one must reach the authoritative
    // state in order, like Stage. Warp votes stay last-wins around it.
    let mut pending = PendingInput::default();
    pending.push(input(vec![Command::SetWarp { factor: 64.0 }]), 1);
    pending.push(input(vec![Command::Reset]), 2);
    pending.push(input(vec![Command::SetWarp { factor: 128.0 }]), 3);
    let merged = pending.take().expect("merged input");
    assert_eq!(merged.commands.len(), 2);
    assert_eq!(merged.commands[0], Command::Reset);
    assert_eq!(merged.commands[1], Command::SetWarp { factor: 128.0 });
}

#[test]
fn ascent_graph_flies_to_orbit_through_phase_laws() {
    use thessa_autopilot::ascent as ascent_api;
    let mut sim = sim_with_terrain();
    let body = sim
        .ephemeris
        .body(sim.authority.reference_body)
        .expect("thessa body");
    let (mu, radius) = (body.mu, body.radius_m);
    eprintln!("thessa mu={mu:.3e} R={radius:.0}");
    // Profile scales with the body: low orbit just above the surface,
    // turn horizontal long before the apoapsis target.
    let profile = ascent_api::AscentProfile {
        target_apoapsis_m: 400_000.0,
        target_periapsis_m: 300_000.0,
        turn_start_altitude_m: 500.0,
        turn_end_altitude_m: 80_000.0,
        liftoff_throttle: 1.0,
        max_phase_time_s: 3_600.0,
    };
    let graph = ascent_api::ascent_graph(&profile).expect("graph builds");
    assert!(sim.submit_graph(graph));
    let mut phases_seen = std::collections::BTreeSet::new();
    // Vertical rise retires almost immediately (tower cleared from
    // the pad altitude): catch it here, the loop observes the rest.
    if let Some(park) = sim.phase_park.as_ref() {
        phases_seen.insert(park.name.clone());
    }
    let deadline = sim.authority.flight_time_s + 3_000.0;
    let mut next_report = 0.0;
    while sim.authority.flight_time_s < deadline {
        let mut advanced = 0.0;
        for _ in 0..400 {
            advanced += sim.advance_chunk(0.5).expect("advance");
            if advanced > 0.0
                || sim.authority.flight_error.is_some()
                || !sim.authority.bake.has_pending()
            {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        if let Some(park) = sim.phase_park.as_ref() {
            phases_seen.insert(park.name.clone());
        }
        if sim.authority.flight_time_s >= next_report {
            next_report = sim.authority.flight_time_s + 300.0;
            let body_now = sim
                .ephemeris
                .body_state(
                    sim.authority.reference_body,
                    SimTime(sim.authority.flight_time_s),
                )
                .expect("body state");
            let rel = sim.authority.state.position_inertial_m - body_now.position_inertial;
            let r = rel.length();
            let rel_vel = sim.authority.state.velocity_inertial_mps - body_now.velocity_inertial;
            let climb = if rel_vel.length_squared() > 1.0 {
                rel.dot(rel_vel) / (r * rel_vel.length())
            } else {
                1.0
            };
            let (cmd, nose_err) = match sim.guidance.as_ref().map(|(i, _)| i) {
                Some(thessa_flight_control::GuidanceIntent::VelocityDirection {
                    direction,
                    ..
                }) => {
                    let nose = sim.authority.state.orientation_body_to_inertial * glam::DVec3::X;
                    let err = nose.angle_between(direction.direction);
                    let up = rel / r;
                    let cmd_up = direction.direction.dot(up);
                    let vel_up = if rel_vel.length_squared() > 1.0 {
                        rel_vel.normalize().dot(up)
                    } else {
                        9.0
                    };
                    let cmd_vel = if rel_vel.length_squared() > 1.0 {
                        direction.direction.dot(rel_vel.normalize())
                    } else {
                        9.0
                    };
                    (
                        format!("cmd_up={cmd_up:.2} vel_up={vel_up:.2} cmd_vel={cmd_vel:.2}"),
                        format!("{err:.2}"),
                    )
                }
                other => (format!("{other:?}"), "-".into()),
            };
            eprintln!(
                "t={:.0} phase={:?} alt={:.1}km speed={:.0}m/s climb_sin={:+.2} apo={:.0}km peri={:.0}km [{cmd}] nose=[{nose_err}] thr={:?} mass={:.0}kg",
                sim.authority.flight_time_s,
                sim.phase_park.as_ref().map(|p| p.name.as_str()),
                (r - radius) / 1000.0,
                rel_vel.length(),
                climb,
                thessa_autopilot::ascent::predict_apoapsis_m(mu, rel, rel_vel)
                    .map(|a| (a - radius) / 1000.0)
                    .unwrap_or(-1.0),
                thessa_autopilot::ascent::predict_periapsis_m(mu, rel, rel_vel)
                    .map(|p| (p - radius) / 1000.0)
                    .unwrap_or(-1.0),
                sim.guidance.as_ref().map(|(_, p)| p.normalized),
                sim.authority.vehicle.mass_properties.mass_kg,
            );
        }
        assert!(
            sim.authority.flight_error.is_none(),
            "flight error: {:?}",
            sim.authority.flight_error
        );
        // Terminal state is Waiting on the armed abort watcher (node
        // 9), not Complete: the sink (node 8) reached means orbit
        // achieved under guard. See the ascent script test.
        if sim.graph_runner.as_ref().is_some_and(|runner| {
            runner.status(thessa_autopilot::NodeId(8)) == Some(thessa_autopilot::BlockStatus::Ok)
        }) {
            break;
        }
    }
    eprintln!("phases seen: {phases_seen:?}");
    for phase in ["vertical-rise", "gravity-turn", "coast", "circularize"] {
        assert!(phases_seen.contains(phase), "phase {phase} never parked");
    }
    assert!(
        sim.graph_runner.as_ref().is_some_and(|runner| {
            runner.status(thessa_autopilot::NodeId(8)) == Some(thessa_autopilot::BlockStatus::Ok)
        }),
        "ascent graph must reach the orbit-achieved sink"
    );
    let body_now = sim
        .ephemeris
        .body_state(
            sim.authority.reference_body,
            SimTime(sim.authority.flight_time_s),
        )
        .expect("body state");
    let rel_pos = sim.authority.state.position_inertial_m - body_now.position_inertial;
    let rel_vel = sim.authority.state.velocity_inertial_mps - body_now.velocity_inertial;
    let periapsis = ascent_api::predict_periapsis_m(mu, rel_pos, rel_vel).expect("bound orbit");
    let apoapsis = ascent_api::predict_apoapsis_m(mu, rel_pos, rel_vel).expect("bound orbit");
    eprintln!(
        "achieved: peri {:.0} km, apo {:.0} km (targets {:.0}/{:.0})",
        (periapsis - radius) / 1000.0,
        (apoapsis - radius) / 1000.0,
        profile.target_periapsis_m / 1000.0,
        profile.target_apoapsis_m / 1000.0,
    );
    assert!(
        periapsis >= radius + profile.target_periapsis_m * 0.9,
        "must circularize near target periapsis"
    );
    assert!(
        apoapsis <= radius + profile.target_apoapsis_m + 100_000.0,
        "must not overshoot the target apoapsis wildly"
    );
}
