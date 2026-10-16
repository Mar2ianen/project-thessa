//! Native replication regressions: no GPU or second simulation clock required.
use super::*;
use std::time::{Duration, Instant};
use thessa_flight_net::{Command, Snapshot, presentation::*};

fn snapshot(scene: &Scene, time: f64) -> Snapshot {
    let flight = scene.flight.as_ref().unwrap();
    let body = scene
        .celestial
        .ephemeris()
        .body_state(flight.reference_body, thessa_sim_core::SimTime(time))
        .unwrap();
    let mut state = flight.state;
    state.position_inertial_m =
        body.position_inertial + flight.relative_position_m + DVec3::new(time * 100.0, 0.0, 0.0);
    Snapshot {
        tick: (time * 120.0) as u64,
        flight_time_s: time,
        state,
        throttle: 0.25,
        engine_active: false,
        paused: false,
        effective_warp: 1.0,
        server_compute_s: 0.01,
        server_wall_s: 0.02,
        steps_this_frame: 5,
        rails_advanced_s: 0.0,
        reaction_wheel_torque_body_nm: [0.0; 3],
        parachutes: Vec::new(),
        wake_notice: None,
        flight_error: None,
    }
}

fn replica() -> Scene {
    let mut scene = Scene::load(Default::default()).unwrap();
    scene.setup_flight().unwrap();
    scene.remote = Some(RemoteFlight {
        link: None,
        commands: Vec::new(),
        started: Instant::now(),
        timeout: Duration::from_secs(5),
        snapshots: 0,
        last_tick: None,
        fleet: FleetObserver::default(),
    });
    scene
}

fn fleet(scene: &Scene, ids_and_times: &[(u32, f64)]) -> thessa_flight_net::FleetSnapshot {
    thessa_flight_net::FleetSnapshot {
        geometry_revision: 1,
        vehicles: ids_and_times
            .iter()
            .map(|&(id, time)| thessa_flight_net::FleetVehicleSnapshot {
                vehicle_id: thessa_sim_core::VehicleId::new(id),
                snapshot: snapshot(scene, time),
            })
            .collect(),
    }
}

fn geometry(
    scene: &Scene,
    ids: &[u32],
    revision: u64,
) -> std::sync::Arc<thessa_flight_net::geometry::FleetGeometry> {
    std::sync::Arc::new(thessa_flight_net::geometry::FleetGeometry {
        revision,
        vehicles: ids
            .iter()
            .map(|id| {
                thessa_flight_net::geometry::FleetVehicleGeometry::from_vehicle(
                    thessa_sim_core::VehicleId::new(*id),
                    &scene.flight.as_ref().unwrap().vehicle,
                )
            })
            .collect(),
    })
}

#[test]
fn geometry_pairs_only_exact_revisions_in_either_delivery_order_and_rejects_reuse() {
    let mut scene = replica();
    let sample = fleet(&scene, &[(1, 1.0)]);
    scene.accept_fleet_snapshot(&sample).unwrap();
    assert!(
        scene
            .remote
            .as_ref()
            .unwrap()
            .fleet
            .matched_geometry()
            .is_none()
    );
    let catalogue = geometry(&scene, &[1], 1);
    scene.accept_fleet_geometry(catalogue.clone()).unwrap();
    assert_eq!(
        scene.remote.as_ref().unwrap().fleet.matched_geometry(),
        Some(catalogue.as_ref())
    );
    let mut corrupt = (*catalogue).clone();
    corrupt.vehicles[0].display_name = "changed under same revision".into();
    assert!(
        scene
            .accept_fleet_geometry(std::sync::Arc::new(corrupt))
            .is_err()
    );
    assert_eq!(scene.remote.as_ref().unwrap().fleet.geometry_updates, 1);
    scene.accept_fleet_geometry(catalogue.clone()).unwrap();
    assert_eq!(scene.remote.as_ref().unwrap().fleet.geometry_updates, 1);
    let next = geometry(&scene, &[1, 7], 2);
    scene.accept_fleet_geometry(next.clone()).unwrap();
    assert!(
        scene
            .remote
            .as_ref()
            .unwrap()
            .fleet
            .matched_geometry()
            .is_none(),
        "never attach a future geometry to old poses"
    );
    let mut sample = fleet(&scene, &[(1, 2.0), (7, 0.5)]);
    sample.geometry_revision = 2;
    scene.accept_fleet_snapshot(&sample).unwrap();
    assert_eq!(
        scene.remote.as_ref().unwrap().fleet.matched_geometry(),
        Some(next.as_ref())
    );
    assert!(scene.accept_fleet_geometry(catalogue).is_err());
    let previous = scene.remote.as_ref().unwrap().fleet.rows.clone();
    let mut corrupt = sample.clone();
    corrupt.vehicles.pop();
    assert!(scene.accept_fleet_snapshot(&corrupt).is_err());
    assert_eq!(scene.remote.as_ref().unwrap().fleet.rows, previous);
    sample.geometry_revision = 1;
    assert!(scene.accept_fleet_snapshot(&sample).is_err());
    let mut future = fleet(&scene, &[(1, 3.0)]);
    future.geometry_revision = 3;
    scene.accept_fleet_snapshot(&future).unwrap();
    assert!(
        scene
            .accept_fleet_geometry(geometry(&scene, &[1, 7], 3))
            .is_err()
    );
    assert_eq!(
        scene.remote.as_ref().unwrap().fleet.geometry.as_ref(),
        Some(&next)
    );
    assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, 0.0);
}

#[test]
fn fleet_membership_is_atomic_and_each_vehicle_keeps_its_own_reference_time() {
    let mut scene = replica();
    let before = (scene.center, scene.sample, scene.input.throttle);
    let samples = fleet(&scene, &[(1, 5.0), (4, 7.5)]);
    scene.accept_fleet_snapshot(&samples).unwrap();
    let rows = &scene.remote.as_ref().unwrap().fleet.rows;
    for (row, wire) in rows.iter().zip(&samples.vehicles) {
        assert_eq!(row.vehicle_id, wire.vehicle_id);
        assert_eq!(row.pose.time().0, wire.snapshot.flight_time_s);
        let reference = scene
            .celestial
            .ephemeris()
            .body_state(
                scene.flight.as_ref().unwrap().reference_body,
                row.pose.time(),
            )
            .unwrap();
        assert!(
            (row.pose.relative_position_m()
                - (wire.snapshot.state.position_inertial_m - reference.position_inertial))
                .length()
                < 1e-8
        );
    }
    let previous = rows.clone();
    let mut malformed = fleet(&scene, &[(1, 6.0), (4, 8.0)]);
    malformed.vehicles[1].snapshot.state.velocity_inertial_mps = DVec3::NAN;
    assert!(scene.accept_fleet_snapshot(&malformed).is_err());
    assert_eq!(scene.remote.as_ref().unwrap().fleet.rows, previous);
    assert!(
        scene
            .accept_fleet_snapshot(&fleet(&scene, &[(1, 4.0), (4, 8.0)]))
            .is_err()
    );
    assert!(
        scene
            .accept_fleet_snapshot(&fleet(&scene, &[(4, 8.0), (1, 6.0)]))
            .is_err()
    );
    assert_eq!(scene.remote.as_ref().unwrap().fleet.updates, 1);
    let changed = fleet(&scene, &[(4, 9.0), (8, 1.0)]);
    assert!(
        scene.accept_fleet_snapshot(&changed).is_err(),
        "a revision cannot change membership even before its catalogue arrives"
    );
    let mut changed = changed;
    changed.geometry_revision = 2;
    scene.accept_fleet_snapshot(&changed).unwrap();
    assert_eq!(
        scene
            .remote
            .as_ref()
            .unwrap()
            .fleet
            .rows
            .iter()
            .map(|row| row.vehicle_id.0)
            .collect::<Vec<_>>(),
        vec![4, 8]
    );
    let mut empty = fleet(&scene, &[]);
    empty.geometry_revision = 3;
    scene.accept_fleet_snapshot(&empty).unwrap();
    assert!(scene.remote.as_ref().unwrap().fleet.rows.is_empty());
    assert_eq!(scene.center, before.0);
    assert_eq!(scene.sample.time, before.1.time);
    assert_eq!(scene.sample.frame, before.1.frame);
    assert_eq!(scene.input.throttle, before.2);
    assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, 0.0);
    assert!(
        scene.vehicle.is_some(),
        "fleet updates cannot despawn PRIMARY"
    );
}

#[test]
fn server_truth_input_intent_and_interpolated_scene_remain_separate() {
    let mut scene = replica();
    let first = snapshot(&scene, 10.0);
    let latest = snapshot(&scene, 11.0);
    scene.input.throttle = 0.9;
    scene.input.engine_active = true;
    scene.accept_server_snapshot(&latest).unwrap();
    let replica_state = scene.flight.as_ref().unwrap().state;
    let display = interpolate_snapshot(&first, &latest, 0.25);
    let pose = checked_snapshot_pose(
        &display,
        scene.celestial.ephemeris(),
        scene.flight.as_ref().unwrap().reference_body,
    )
    .unwrap();
    scene.publish_vehicle_pose(pose).unwrap();
    scene.push_live_input(0.1, false, false);
    let flight = scene.flight.as_ref().unwrap();
    assert_eq!(flight.flight_time_s, 11.0);
    assert_eq!(
        flight.state.position_inertial_m,
        replica_state.position_inertial_m
    );
    assert_eq!(flight.throttle, 0.25);
    assert!(!flight.engine_active);
    assert_eq!(flight.accumulator_s, 0.0);
    assert_eq!(scene.sample.time.0, 10.25);
    let sample = scene.celestial.sample(pose.time()).unwrap();
    assert_eq!(scene.sample.frame, sample.frame);
    assert_eq!(scene.sample.lighting, sample.lighting);
    assert!(
        (scene.sample.frame.body_to_render_point(scene.center) - pose.render_position_m()).length()
            < 1e-8
    );
    assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&scene.view(1280, 720, 1.0)).is_some());
    let input = input_for_server(
        flight,
        scene.control_mode,
        scene.input.throttle,
        scene.input.engine_active,
        vec![Command::Pause { paused: false }],
    );
    input.validate().unwrap();
    assert_eq!(input.throttle, 0.9);
    assert!(input.engine_active);
    assert_eq!(input.tick, latest.tick);
    // A stopped/disconnected server never enters the local stepping branch,
    // even for an elapsed value that local mode would reject.
    scene.advance_flight(f64::NAN, None).unwrap();
    assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, 11.0);
    assert_eq!(scene.sample.time.0, 10.25);
}

#[test]
fn malformed_and_backwards_snapshots_do_not_mutate_replica_or_visual_sample() {
    let mut scene = replica();
    let good = snapshot(&scene, 10.0);
    scene.accept_server_snapshot(&good).unwrap();
    let before = scene.flight.as_ref().unwrap().state;
    for index in 0..9 {
        let mut bad = good.clone();
        match index {
            0 => bad.state.position_inertial_m = DVec3::NAN,
            1 => bad.state.velocity_inertial_mps = DVec3::INFINITY,
            2 => bad.state.orientation_body_to_inertial = DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            3 => bad.state.angular_velocity_body_rps = DVec3::NAN,
            4 => bad.throttle = 1.1,
            5 => bad.effective_warp = f64::NAN,
            6 => bad.server_wall_s = -1.0,
            7 => bad.reaction_wheel_torque_body_nm[0] = f64::INFINITY,
            _ => bad.flight_time_s = -1.0,
        }
        assert!(scene.accept_server_snapshot(&bad).is_err(), "case {index}");
        assert_eq!(
            scene.flight.as_ref().unwrap().state.position_inertial_m,
            before.position_inertial_m
        );
        assert_eq!(
            scene
                .flight
                .as_ref()
                .unwrap()
                .state
                .orientation_body_to_inertial,
            before.orientation_body_to_inertial
        );
        assert_eq!(
            scene.flight.as_ref().unwrap().flight_time_s,
            good.flight_time_s
        );
        assert_eq!(scene.sample.time.0, 0.0);
    }
    let mut bad = good.clone();
    bad.tick -= 1;
    assert!(scene.accept_server_snapshot(&bad).is_err());
    bad.tick = good.tick;
    bad.flight_time_s -= 1.0;
    assert!(scene.accept_server_snapshot(&bad).is_err());
    assert_eq!(scene.remote.as_ref().unwrap().snapshots, 1);
}

/// A real executable stdio peer, with framed Rust-produced messages; only
/// lifecycle/silence is mocked, not the transport. Scratch stays crate-local.
#[cfg(unix)]
fn peer(after_welcome: &str) -> std::path::PathBuf {
    use std::os::unix::fs::PermissionsExt;
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(format!(
        "native-peer-{}-{}.sh",
        std::process::id(),
        NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    ));
    let hello_len = thessa_flight_net::encode_hello(&thessa_flight_net::Hello {
        client_name: "thessa-client-embedded".into(),
    })
    .unwrap()
    .len();
    let welcome = thessa_flight_net::encode_welcome(&thessa_flight_net::Welcome {
        tick: 0,
        flight_time_s: 0.0,
    })
    .unwrap();
    let bytes: String = welcome.iter().map(|byte| format!("\\{byte:03o}")).collect();
    std::fs::write(&path, format!("#!/bin/sh\ndd bs=1 count={hello_len} of=/dev/null 2>/dev/null\nprintf '{bytes}'\n{after_welcome}\n")).unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
    path
}

#[cfg(unix)]
#[test]
fn server_silence_and_pipe_loss_freeze_without_local_fallback() {
    for (script, expected) in [
        ("exec sleep 30", "deadline"),
        ("exec 1>&-; exec sleep 30", "link lost"),
    ] {
        let path = peer(script);
        let mut scene = Scene::load(Default::default()).unwrap();
        scene
            .enable_server_flight(Some(&path), Duration::from_secs(5))
            .unwrap();
        assert!(
            scene.flight_data().is_none(),
            "constructor pose is not a server acknowledgement"
        );
        // Expire host silence without a wall-clock sleep or scheduling race.
        if expected == "deadline" {
            for _ in 0..20 {
                scene.advance_flight(0.1, None).unwrap();
            }
            assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, 0.0);
            scene.remote.as_mut().unwrap().started = Instant::now() - Duration::from_secs(10);
        }
        let before = scene.center;
        let deadline = Instant::now() + Duration::from_secs(5);
        while !scene.flight_stopped() {
            scene.push_live_input(0.1, false, false);
            scene.advance_flight(0.1, None).unwrap();
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            scene
                .flight
                .as_ref()
                .unwrap()
                .flight_error
                .as_ref()
                .unwrap()
                .contains(expected)
        );
        assert!(scene.remote.as_ref().unwrap().link.is_none());
        for _ in 0..10 {
            scene.advance_flight(0.1, None).unwrap();
        }
        assert_eq!(scene.center, before);
        assert_eq!(scene.sample.time.0, 0.0);
        assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, 0.0);
        std::fs::remove_file(path).unwrap();
    }
}

#[cfg(unix)]
#[test]
fn received_snapshot_silence_uses_reader_age_and_preserves_the_published_scene() {
    let mut scene = Scene::load(Default::default()).unwrap();
    scene.setup_flight().unwrap();
    let good = snapshot(&scene, 5.0);
    let bytes: String = thessa_flight_net::encode_snapshot(&good)
        .unwrap()
        .iter()
        .map(|byte| format!("\\{byte:03o}"))
        .collect();
    let path = peer(&format!("printf '{bytes}'; exec sleep 30"));
    let link =
        thessa_flight_net::embedded::EmbeddedLink::spawn_server(&path, Duration::from_secs(5))
            .unwrap();
    scene.remote = Some(RemoteFlight {
        link: Some(link),
        commands: Vec::new(),
        started: Instant::now(),
        timeout: Duration::from_secs(5),
        snapshots: 0,
        last_tick: None,
        fleet: FleetObserver::default(),
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    while scene.server_snapshot_count() == Some(0) {
        scene.advance_flight(0.1, None).unwrap();
        assert!(Instant::now() < deadline);
        std::thread::yield_now();
    }
    // A received sample supersedes the startup clock, even without another
    // generation arriving. Its reader timestamp, not host polling, sets age.
    scene.remote.as_mut().unwrap().started = Instant::now() - Duration::from_secs(30);
    scene.advance_flight(0.1, None).unwrap();
    assert!(!scene.flight_stopped());
    assert_eq!(scene.sample.time.0, 5.0);
    let before = (scene.center, scene.sample);
    scene.remote.as_mut().unwrap().timeout = Duration::ZERO;
    scene.advance_flight(0.1, None).unwrap();
    assert!(scene.flight_stopped());
    assert_eq!(scene.center, before.0);
    assert_eq!(scene.sample.frame, before.1.frame);
    assert_eq!(scene.sample.lighting, before.1.lighting);
    assert_eq!(scene.sample.time, before.1.time);
    assert!(
        scene
            .flight
            .as_ref()
            .unwrap()
            .flight_error
            .as_ref()
            .unwrap()
            .contains("deadline")
    );
    std::fs::remove_file(path).unwrap();
}

#[cfg(unix)]
#[test]
fn fleet_silence_does_not_reuse_primary_age_or_infer_despawn() {
    let mut scene = Scene::load(Default::default()).unwrap();
    scene.setup_flight().unwrap();
    let wire_fleet = fleet(&scene, &[(1, 5.0)]);
    let primary = snapshot(&scene, 6.0);
    let bytes = |wire: Vec<u8>| {
        wire.iter()
            .map(|byte| format!("\\{byte:03o}"))
            .collect::<String>()
    };
    let path = peer(&format!(
        "printf '{}'; while :; do printf '{}'; sleep 0.01; done",
        bytes(thessa_flight_net::encode_fleet_snapshot(&wire_fleet).unwrap()),
        bytes(thessa_flight_net::encode_snapshot(&primary).unwrap())
    ));
    let link =
        thessa_flight_net::embedded::EmbeddedLink::spawn_server(&path, Duration::from_secs(5))
            .unwrap();
    scene.remote = Some(RemoteFlight {
        link: Some(link),
        commands: Vec::new(),
        started: Instant::now(),
        timeout: Duration::from_secs(5),
        snapshots: 0,
        last_tick: None,
        fleet: FleetObserver::default(),
    });
    let deadline = Instant::now() + Duration::from_secs(5);
    while scene.remote.as_ref().unwrap().fleet.updates == 0
        || scene.server_snapshot_count() == Some(0)
    {
        scene.advance_flight(0.0, None).unwrap();
        assert!(!scene.flight_stopped());
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(1));
    }
    scene.remote.as_mut().unwrap().timeout = Duration::from_millis(250);
    while !scene.remote.as_ref().unwrap().fleet.stale {
        scene.advance_flight(0.0, None).unwrap();
        assert!(!scene.flight_stopped(), "{:?}", scene.flight_telemetry());
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(1));
    }
    let remote = scene.remote.as_ref().unwrap();
    assert_eq!(
        remote.fleet.rows.len(),
        1,
        "keep last-known membership, hide it as stale in HUD"
    );
    assert_eq!(remote.fleet.updates, 1);
    assert!(remote.snapshots > 1);
    assert!(remote.fleet.age_s.unwrap() >= 0.25);
    scene.stop_server_flight("test peer completed".into());
    std::fs::remove_file(path).unwrap();
}

#[cfg(unix)]
#[test]
fn server_part_events_are_ordered_bounded_and_not_rebuilt_from_acknowledgements() {
    let path = peer("exec sleep 30");
    let mut scene = Scene::load(Default::default()).unwrap();
    scene
        .enable_server_flight(Some(&path), Duration::from_secs(5))
        .unwrap();
    scene.apply_part_toggle(PartToggle::Gear);
    scene.apply_part_toggle(PartToggle::Gear);
    scene.apply_part_toggle(PartToggle::Parachutes);
    let commands = &scene.remote.as_ref().unwrap().commands;
    use thessa_sim_core::VehiclePartCommand::*;
    assert_eq!(
        *commands,
        vec![
            Command::Part {
                command: SetLandingGearDeployed { deployed: false }
            },
            Command::Part {
                command: SetLandingGearDeployed { deployed: true }
            },
            Command::Part {
                command: SetParachutesArmed { armed: true }
            }
        ]
    );
    let ack = snapshot(&scene, 1.0);
    scene.accept_server_snapshot(&ack).unwrap();
    assert_eq!(scene.part_states(), Some((true, true, true, true)));
    assert_eq!(scene.remote.as_ref().unwrap().commands.len(), 3);
    for _ in 3..thessa_flight_net::MAX_COMMANDS_PER_INPUT - 2 {
        scene.apply_part_toggle(PartToggle::Gear);
    }
    assert!(!scene.flight_stopped());
    scene.apply_part_toggle(PartToggle::Gear);
    assert!(scene.flight_stopped());
    assert!(scene.remote.as_ref().unwrap().link.is_none());
    std::fs::remove_file(path).unwrap();
}

#[test]
#[ignore = "requires a built thessa-server executable beside the native viewer"]
fn real_server_adoption_advances_and_acknowledges_input_without_local_steps() {
    let server = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(format!("thessa-server{}", std::env::consts::EXE_SUFFIX));
    let mut scene = Scene::load(Default::default()).unwrap();
    scene
        .enable_server_flight(Some(&server), Duration::from_secs(10))
        .unwrap();
    scene.input.cut();
    scene.input.engine_active = false;
    let deadline = Instant::now() + Duration::from_secs(10);
    while scene.server_snapshot_count().unwrap() < 5
        || scene.sample.time.0 <= 0.1
        || scene.remote.as_ref().unwrap().fleet.updates < 2
        || scene
            .remote
            .as_ref()
            .unwrap()
            .fleet
            .matched_geometry()
            .is_none()
    {
        scene.push_live_input(0.0, false, false);
        scene.advance_flight(f64::NAN, None).unwrap();
        assert!(!scene.flight_stopped(), "{:?}", scene.flight_telemetry());
        assert!(Instant::now() < deadline, "{:?}", scene.flight_telemetry());
        std::thread::sleep(Duration::from_millis(2));
    }
    let flight = scene.flight.as_ref().unwrap();
    assert_eq!(flight.throttle, 0.0);
    assert!(!flight.engine_active);
    assert!(scene.remote.as_ref().unwrap().fleet.rows.is_empty());
    assert!(!scene.remote.as_ref().unwrap().fleet.stale);
    assert_eq!(
        scene.remote.as_ref().unwrap().fleet.geometry_updates,
        1,
        "unchanged geometry is not resent on each snapshot"
    );
    assert_eq!(flight.accumulator_s, 0.0);
    assert!(scene.sample.time.0 <= flight.flight_time_s);
    assert!(scene.flight_data().is_some());
    assert!(scene.view(1280, 720, 1.0).world_position_m.is_finite());
    eprintln!("{}", scene.flight_telemetry().unwrap());
}

#[test]
#[cfg(all(unix, feature = "hud"))]
#[ignore = "requires a built sibling native viewer and wgpu adapter; framed fixture, not physical fleet replay"]
fn fleet_inspector_capture_through_actual_native_cli_and_shared_link() {
    let mut scene = Scene::load(Default::default()).unwrap();
    scene.setup_flight().unwrap();
    let primary = snapshot(&scene, 6.0);
    let members: Vec<_> = (1..thessa_flight_net::MAX_FLEET_VEHICLES as u32)
        .map(|id| (id, 4.0 + f64::from(id) / 32.0))
        .collect();
    let mut samples = fleet(&scene, &members);
    samples.vehicles[6].snapshot.flight_error = Some("fixture stopped secondary".into());
    let catalogue = geometry(&scene, &(1..64).collect::<Vec<_>>(), 1);
    let catalogue_wire: String = thessa_flight_net::geometry::encode_fleet_geometry(&catalogue)
        .unwrap()
        .into_iter()
        .map(|byte| format!("\\{byte:03o}"))
        .collect();
    let wire: String = thessa_flight_net::encode_snapshot(&primary)
        .unwrap()
        .into_iter()
        .chain(thessa_flight_net::encode_fleet_snapshot(&samples).unwrap())
        .map(|byte| format!("\\{byte:03o}"))
        .collect();
    let path = peer(&format!(
        "printf '{catalogue_wire}'; while :; do printf '{wire}'; sleep 0.05; done"
    ));
    let viewer = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join(format!(
            "thessa-terrain-native{}",
            std::env::consts::EXE_SUFFIX
        ));
    let capture = path.with_extension("png");
    let output = std::process::Command::new(viewer)
        .current_dir(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args(["--server-flight", "--server"])
        .arg(&path)
        .args(["--headless"])
        .arg(&capture)
        .args(["--hud", "--frames", "60", "--prestream-seconds", "2"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "{stdout}\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    for expected in [
        "fleet_last_known=63",
        "fleet_stale=false",
        "stopped=false",
        "local_steps=0",
        "geometry_matched=true",
        "geometry_updates=1",
    ] {
        assert!(stdout.contains(expected), "{stdout}");
    }
    assert!(capture.exists());
    eprintln!(
        "Fleet inspector framed-fixture capture (not a physical fleet replay): {}\n{stdout}",
        capture.display()
    );
    std::fs::remove_file(path).unwrap();
}
