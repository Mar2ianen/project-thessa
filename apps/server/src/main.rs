//! Headless authoritative flight server.
//!
//! Owns one [`FlightAuthority`] on a dedicated sim thread.tick loop runs the
//! 120 Hz lattice with no frame budget: warped chunks are bounded only by
//! chunk size (input responsiveness), never by wall time. Transports:
//! length-prefixed frames over stdio (embedded local client) today, TCP
//! (remote clients) next. Stdout is the wire — logs go to stderr.

mod thread_bake;

use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Instant;

use glam::DVec3;
use thessa_flight_authority::{ControlMode, FlightAuthority};
use thessa_flight_net::{ClientInput, Command, Snapshot};
use thessa_protocol::{FrameDecoder, kind};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};
use thread_bake::ThreadBakeQueue;

/// Sim time per loop iteration: 120 ticks. Bounds input latency at high
/// warp without capping throughput (the frame budget this replaces).
const CHUNK_S: f64 = 1.0;
/// Upper clamp for requested warp (2^17, same ceiling as the client).
const MAX_WARP: f64 = 131072.0;

struct Args {
    system_path: Option<String>,
    /// Batch throughput probe: advance this many sim-seconds flat out,
    /// print the effective warp, exit. No IO after setup.
    measure_s: Option<f64>,
    /// Start in a 300 km circular coast instead of on the pad.
    vacuum: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        system_path: None,
        measure_s: None,
        vacuum: false,
    };
    let mut rest = std::env::args().skip(1);
    while let Some(arg) = rest.next() {
        match arg.as_str() {
            "--system" => {
                args.system_path = Some(rest.next().ok_or("--system needs a path")?);
            }
            "--measure" => {
                let seconds: f64 = rest
                    .next()
                    .ok_or("--measure needs sim-seconds")?
                    .parse()
                    .map_err(|_| "--measure needs a number")?;
                if seconds <= 0.0 {
                    return Err("--measure needs positive sim-seconds".into());
                }
                args.measure_s = Some(seconds);
            }
            "--vacuum" => args.vacuum = true,
            "--help" | "-h" => {
                eprintln!(
                    "usage: thessa-server [--system PATH] [--measure SIM_S] [--vacuum]\n\
                     default: stdio wire server (frames on stdin/stdout, logs on stderr)"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(args)
}

fn find_system(cli_path: Option<String>) -> Result<String, String> {
    if let Some(path) = cli_path {
        return Ok(path);
    }
    for candidate in ["data/system.toml"] {
        if std::path::Path::new(candidate).exists() {
            return Ok(candidate.into());
        }
    }
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    for ancestor in exe.ancestors().skip(1).take(4) {
        let candidate = ancestor.join("data/system.toml");
        if candidate.exists() {
            return Ok(candidate.to_string_lossy().into_owned());
        }
    }
    Err("system.toml not found: pass --system PATH".into())
}

fn load_system(path: &str) -> Result<(BakedEphemeris, BodyId), String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let config: SystemConfig = toml::from_str(&text).map_err(|e| e.to_string())?;
    let ephemeris = config.bake().map_err(|e| e.to_string())?;
    let body = ephemeris
        .body_id("thessa")
        .ok_or_else(|| "no thessa body in system".to_string())?;
    Ok((ephemeris, body))
}

/// Driver around the authority: inputs in, snapshots out, warp accounting.
struct Sim {
    authority: FlightAuthority,
    ephemeris: BakedEphemeris,
    warp: f64,
    paused: bool,
    advanced_s: f64,
    wall_s: f64,
    steps: u64,
    rails_s: f64,
}

impl Sim {
    fn new(ephemeris: BakedEphemeris, reference_body: BodyId, vacuum: bool) -> Result<Self, String> {
        let mut authority = FlightAuthority::new(&ephemeris, reference_body)
            .map_err(|e| format!("authority init: {e}"))?
            .with_bake_queue(Box::new(ThreadBakeQueue::new()));
        if vacuum {
            place_in_circular_orbit(&ephemeris, &mut authority)?;
        }
        Ok(Self {
            authority,
            ephemeris,
            warp: 1.0,
            paused: false,
            advanced_s: 0.0,
            wall_s: 0.0,
            steps: 0,
            rails_s: 0.0,
        })
    }

    fn apply_input(&mut self, input: &ClientInput) {
        let authority = &mut self.authority;
        authority.control_input = DVec3::from_array(input.control_input);
        authority.throttle = input.throttle.clamp(0.0, 1.0);
        authority.engine_active = input.engine_active;
        authority.sas_enabled = input.sas_enabled;
        authority.rcs_enabled = input.rcs_enabled;
        authority.gear_down = input.gear_down;
        for command in &input.commands {
            match command {
                Command::SetWarp { factor } => {
                    self.warp = factor.clamp(0.0, MAX_WARP);
                }
                // Slice semantics (matches the client): staging drives the
                // engine cutoff for the single X-15 plant.
                Command::Stage => authority.engine_active = !authority.engine_active,
                Command::Engine { active } => authority.engine_active = *active,
                Command::Pause { paused } => self.paused = *paused,
            }
        }
    }

    /// Advance one chunk (or less at 1x); returns sim-seconds advanced.
    fn advance_chunk(&mut self, chunk_s: f64) -> Result<f64, String> {
        if self.paused || self.authority.flight_error.is_some() {
            return Ok(0.0);
        }
        let before = self.authority.flight_time_s;
        let started = Instant::now();
        self.authority
            .advance_with_budget(&self.ephemeris, ControlMode::Navball, chunk_s, None)
            .map_err(|e| {
                self.authority.engine_active = false;
                self.authority.flight_error = Some(e.to_string());
                e.to_string()
            })?;
        self.wall_s += started.elapsed().as_secs_f64();
        let advanced = self.authority.flight_time_s - before;
        self.advanced_s += advanced;
        self.steps += self.authority.steps_this_frame as u64;
        self.rails_s += self.authority.rails_advanced_this_frame;
        Ok(advanced)
    }

    fn effective_warp(&self) -> f64 {
        if self.wall_s <= 0.0 {
            0.0
        } else {
            (self.advanced_s / self.wall_s).min(self.warp.max(0.0))
        }
    }

    fn snapshot(&self) -> Snapshot {
        let authority = &self.authority;
        Snapshot {
            tick: authority.world_tick.0,
            flight_time_s: authority.flight_time_s,
            state: authority.state,
            throttle: authority.throttle,
            engine_active: authority.engine_active,
            paused: self.paused,
            effective_warp: self.effective_warp(),
            steps_this_frame: authority.steps_this_frame,
            rails_advanced_s: authority.rails_advanced_this_frame,
            wake_notice: authority.wake_notice.clone(),
            flight_error: authority.flight_error.clone(),
        }
    }
}

fn place_in_circular_orbit(
    ephemeris: &BakedEphemeris,
    authority: &mut FlightAuthority,
) -> Result<(), String> {
    use glam::DQuat;
    let body = ephemeris
        .body(authority.reference_body)
        .map_err(|e| e.to_string())?;
    let origin = ephemeris
        .body_state(authority.reference_body, SimTime::EPOCH)
        .map_err(|e| e.to_string())?;
    let radius = body.radius_m + 300_000.0;
    authority.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
    authority.state.velocity_inertial_mps =
        origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
    authority.state.orientation_body_to_inertial = DQuat::IDENTITY;
    authority.state.angular_velocity_body_rps = DVec3::ZERO;
    authority.sas_target_orientation = DQuat::IDENTITY;
    authority.throttle = 1.0;
    authority.engine_active = true;
    authority.control_input = DVec3::ZERO;
    Ok(())
}

fn send_frame(out: &Sender<Vec<u8>>, kind: u32, message: &impl serde::Serialize) {
    match thessa_flight_net::encode_frame(kind, message) {
        Ok(frame) => {
            if out.send(frame).is_err() {
                eprintln!("[server] stdout writer gone");
            }
        }
        Err(error) => eprintln!("[server] encode error: {error}"),
    }
}

fn run_stdio(mut sim: Sim) -> Result<(), String> {
    // Stdout writer thread: frames in, bytes out. Stdout is the wire.
    // Joined on shutdown after all senders drop, so the tail flushes.
    let (wire_out, wire_rx): (Sender<Vec<u8>>, _) = channel();
    let writer = std::thread::spawn(move || {
        let mut stdout = std::io::stdout().lock();
        for frame in wire_rx {
            if stdout.write_all(&frame).is_err() {
                break;
            }
        }
        let _ = stdout.flush();
    });

    // Stdin pump thread owns the pipe end to end: bytes in, complete
    // frames out. A single lock means no buffered byte is ever stranded
    // between a handshake read and the steady-state loop.
    let (frames_tx, frames_rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut locked = stdin.lock();
        let mut decoder = FrameDecoder::new();
        let mut buffer = [0u8; 65536];
        loop {
            match locked.read(&mut buffer) {
                Ok(0) => break, // EOF: client gone.
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        for frame in frames {
                            if frames_tx.send(frame).is_err() {
                                return;
                            }
                        }
                    }
                    Err(error) => {
                        eprintln!("[server] frame error: {error}");
                        break;
                    }
                },
                Err(_) => break,
            }
        }
    });

    // Handshake on the main thread: first frame must be Hello.
    let frame = frames_rx.recv().map_err(|_| "stdin closed before handshake")?;
    let envelope = thessa_flight_net::decode_frame(&frame).map_err(|e| e.to_string())?;
    if envelope.kind != kind::HELLO {
        return Err(format!("expected HELLO, got kind {}", envelope.kind));
    }
    let hello: thessa_flight_net::Hello =
        thessa_flight_net::decode_payload(&envelope).map_err(|e| e.to_string())?;
    eprintln!("[server] hello from {:?}", hello.client_name);
    let welcome = thessa_flight_net::Welcome {
        tick: sim.authority.world_tick.0,
        flight_time_s: sim.authority.flight_time_s,
    };
    send_frame(&wire_out, kind::WELCOME, &welcome);

    // Tick loop: drain inputs, advance a chunk, publish a snapshot.
    // At 1x the loop paces to wall time; above 1x it runs flat out.
    // Client EOF (pump exit) ends the loop after draining.
    let tick_s = thessa_sim_core::WORLD_TICK_S;
    let mut next_deadline = Instant::now();
    loop {
        while let Ok(frame) = frames_rx.try_recv() {
            let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
                continue;
            };
            if envelope.kind != kind::CLIENT_INPUT {
                continue;
            }
            if let Ok(input) = thessa_flight_net::decode_payload::<ClientInput>(&envelope) {
                sim.apply_input(&input);
            }
        }
        if sim.warp <= 0.0 || sim.paused {
            // Frozen time, live snapshots (client stays attached).
            std::thread::sleep(std::time::Duration::from_secs_f64(tick_s));
        } else if sim.warp <= 1.0 {
            let chunk = (sim.warp * tick_s).max(tick_s);
            sim.advance_chunk(chunk).map_err(|e| format!("advance: {e}"))?;
            next_deadline += std::time::Duration::from_secs_f64(chunk / sim.warp);
            let now = Instant::now();
            if next_deadline > now {
                std::thread::sleep(next_deadline - now);
            } else {
                next_deadline = now;
            }
        } else {
            sim.advance_chunk(CHUNK_S).map_err(|e| format!("advance: {e}"))?;
        }
        let snapshot = sim.snapshot();
        send_frame(&wire_out, kind::SNAPSHOT, &snapshot);
        // Pump exit drops the sender; try_recv errors only on disconnect
        // once drained, so probe cheaply: a failed non-blocking recv with
        // no senders left means EOF.
        let client_gone = matches!(
            frames_rx.try_recv(),
            Err(std::sync::mpsc::TryRecvError::Disconnected)
        );
        if client_gone {
            // Drain race: the last frames may have arrived with the
            // disconnect; one more pass keeps them, then exit.
            while let Ok(frame) = frames_rx.try_recv() {
                let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
                    continue;
                };
                if envelope.kind != kind::CLIENT_INPUT {
                    continue;
                }
                if let Ok(input) = thessa_flight_net::decode_payload::<ClientInput>(&envelope) {
                    sim.apply_input(&input);
                }
            }
            break;
        }
    }
    drop(wire_out);
    let _ = writer.join();
    eprintln!(
        "[server] done: {:.1} sim-s, {:.1} wall-s, effective warp x{:.1}, {} steps ({:.1} rails-s)",
        sim.advanced_s,
        sim.wall_s,
        sim.effective_warp(),
        sim.steps,
        sim.rails_s
    );
    Ok(())
}

fn run_measure(mut sim: Sim, target_s: f64) -> Result<(), String> {
    let t0 = Instant::now();
    while sim.advanced_s < target_s {
        sim.advance_chunk(CHUNK_S)
            .map_err(|e| format!("advance: {e}"))?;
        if sim.authority.flight_error.is_some() {
            break;
        }
    }
    let wall = t0.elapsed().as_secs_f64();
    println!(
        "sim_s={:.1} wall_s={:.3} effective_warp=x{:.1} steps={} rails_s={:.1} flight_t={:.1} err={:?}",
        sim.advanced_s,
        wall,
        sim.advanced_s / wall.max(1e-9),
        sim.steps,
        sim.rails_s,
        sim.authority.flight_time_s,
        sim.authority.flight_error,
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("[server] fatal: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let system_path = find_system(args.system_path)?;
    eprintln!("[server] system: {system_path}");
    let (ephemeris, reference_body) = load_system(&system_path)?;
    let sim = Sim::new(ephemeris, reference_body, args.vacuum)?;
    match args.measure_s {
        Some(target_s) => run_measure(sim, target_s),
        None => run_stdio(sim),
    }
}
