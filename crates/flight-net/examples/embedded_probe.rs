//! Headless transport smoke against an actual native thessa-server binary.
//! Not a renderer, native-viewer network adoption or a performance benchmark.
use std::{
    error::Error,
    path::PathBuf,
    time::{Duration, Instant},
};
use thessa_flight_net::{ClientInput, Command, Snapshot, embedded::EmbeddedLink};

fn next_snapshot(link: &EmbeddedLink, timeout: Duration) -> Result<Snapshot, Box<dyn Error>> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(snapshot) = link.latest_snapshot() {
            return Ok(snapshot);
        }
        if Instant::now() >= deadline {
            return Err("snapshot deadline exceeded".into());
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let server = PathBuf::from(
        args.next()
            .ok_or("usage: embedded_probe SERVER_EXECUTABLE")?,
    );
    if args.next().is_some() {
        return Err("usage: embedded_probe SERVER_EXECUTABLE".into());
    }
    let timeout = Duration::from_secs(10);
    let start = Instant::now();
    let link = EmbeddedLink::spawn_server(&server, timeout)?;
    let handshake_ms = start.elapsed().as_secs_f64() * 1000.0;
    let first = next_snapshot(&link, timeout)?;
    let input = ClientInput {
        tick: first.tick,
        vehicle_id: thessa_sim_core::VehicleId::PRIMARY,
        control_input: [0.0; 3],
        control_mode: thessa_flight_authority::ControlMode::Navball,
        sas_target_xyzw: first.state.orientation_body_to_inertial.to_array(),
        throttle: 0.0,
        engine_active: false,
        sas_enabled: true,
        rcs_enabled: true,
        reaction_wheels_enabled: true,
        gear_down: true,
        parachutes_armed: false,
        commands: vec![
            Command::SetWarp { factor: 1.0 },
            Command::Pause { paused: false },
        ],
    };
    input.validate()?;
    if !link.send_input(&input) {
        return Err("input mailbox refused probe input".into());
    }
    let mut latest_tick = first.tick;
    let mut last_time = first.flight_time_s;
    let mut acknowledged = false;
    for _ in 0..8 {
        let snapshot = next_snapshot(&link, timeout)?;
        if snapshot.tick < latest_tick || snapshot.flight_time_s < last_time {
            return Err("authoritative snapshot time/tick moved backwards".into());
        }
        if snapshot.flight_error.is_some() {
            return Err(format!("server flight error: {:?}", snapshot.flight_error).into());
        }
        latest_tick = snapshot.tick;
        last_time = snapshot.flight_time_s;
        acknowledged |= snapshot.throttle == 0.0 && !snapshot.engine_active;
        let display = link
            .interpolated_snapshot()
            .ok_or("missing presentation snapshot")?;
        if !display.state.position_inertial_m.is_finite()
            || !display.state.orientation_body_to_inertial.is_finite()
            || !display.flight_time_s.is_finite()
        {
            return Err("nonfinite presentation snapshot".into());
        }
    }
    if !acknowledged || latest_tick <= first.tick || last_time <= first.flight_time_s {
        return Err("server did not advance and acknowledge throttle cut/engine off".into());
    }
    drop(link);
    println!(
        "embedded_probe: handshake_ms={handshake_ms:.3} snapshots=9 first_tick={} last_tick={latest_tick} first_sim_s={} last_sim_s={last_time} input_ack=true link_dropped=true elapsed_s={:.3}; transport smoke, not game FPS",
        first.tick,
        first.flight_time_s,
        start.elapsed().as_secs_f64(),
    );
    Ok(())
}
