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
use std::time::{Duration, Instant};

use glam::DVec3;
use thessa_flight_authority::{ControlMode, FlightAuthority};
use thessa_flight_net::{ClientInput, Command, Snapshot};
use thessa_protocol::{FrameDecoder, kind};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};
use thread_bake::ThreadBakeQueue;

/// Wall seconds covered by one warped iteration. The chunk is sized from
/// the vote (2 wall-s worth of sim time), so input latency and snapshot
/// cadence stay interactive at any warp while big requests still trip the
/// authority catch-up rule (blocking first bake, then unlimited batches).
const WARP_CHUNK_WALL_S: f64 = 2.0;
/// Floor for warped chunks: keeps modest requests on the async path
/// (not the blocking catch-up) while staying far above one tick.
const WARP_CHUNK_MIN_S: f64 = 4.0;
/// Wall-pacing for snapshots so high warp does not flood the pipe; the
/// client interpolates between them.
const SNAPSHOT_MIN_INTERVAL_S: f64 = 0.05;
/// Upper clamp for requested warp (2^17, same ceiling as the client).
const MAX_WARP: f64 = 131072.0;

struct Args {
    system_path: Option<String>,
    tcp_addr: Option<String>,
    /// Batch throughput probe: advance this many sim-seconds flat out,
    /// print the effective warp, exit. No IO after setup.
    measure_s: Option<f64>,
    /// Cut the engine for the run: unpowered coast rides rails batches.
    drift: bool,
    /// Start in a 300 km circular coast instead of on the pad.
    vacuum: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        system_path: None,
        tcp_addr: None,
        measure_s: None,
        vacuum: false,
        drift: false,
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
            "--drift" => args.drift = true,
            "--tcp" => {
                args.tcp_addr = Some(rest.next().ok_or("--tcp needs ADDR:PORT")?);
            }
            "--help" | "-h" => {
                eprintln!(
                    "usage: thessa-server [--system PATH] [--measure SIM_S] [--vacuum] [--drift] [--tcp ADDR:PORT]\n\
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

/// One connected pilot: warp vote and pause vote. Effective warp is the
/// minimum of all votes (nobody gets dragged faster than they asked);
/// the sim pauses while anyone votes pause.
#[derive(Debug, Clone, Copy)]
struct ClientVote {
    warp: f64,
    paused: bool,
}

/// Driver around the authority: inputs in, snapshots out, warp accounting.
struct Sim {
    authority: FlightAuthority,
    ephemeris: BakedEphemeris,
    control_mode: ControlMode,
    clients: std::collections::HashMap<String, ClientVote>,
    advanced_s: f64,
    wall_s: f64,
    steps: u64,
    rails_s: f64,
}

impl Sim {
    fn new(
        ephemeris: BakedEphemeris,
        reference_body: BodyId,
        vacuum: bool,
        drift: bool,
    ) -> Result<Self, String> {
        let mut authority = FlightAuthority::new(&ephemeris, reference_body)
            .map_err(|e| format!("authority init: {e}"))?
            .with_bake_queue(Box::new(ThreadBakeQueue::new()));
        // Drift rides above the atmosphere top (declared vacuum) so
        // batches engage; plain vacuum stays at 300 km in the band.
        let altitude_m = if drift { 400_000.0 } else { 300_000.0 };
        if vacuum || drift {
            place_in_circular_orbit(&ephemeris, &mut authority, altitude_m)?;
        }
        if drift {
            authority.engine_active = false;
            authority.throttle = 0.0;
        }
        Ok(Self {
            authority,
            ephemeris,
            control_mode: ControlMode::Navball,
            clients: std::collections::HashMap::new(),
            advanced_s: 0.0,
            wall_s: 0.0,
            steps: 0,
            rails_s: 0.0,
        })
    }

    /// Register a pilot (handshake); idempotent reconnect refreshes.
    fn register(&mut self, id: &str) {
        self.clients.entry(id.to_string()).or_insert(ClientVote {
            warp: 1.0,
            paused: false,
        });
    }

    /// Drop a pilot's votes on disconnect.
    fn unregister(&mut self, id: &str) {
        self.clients.remove(id);
    }

    /// Consensual warp: the minimum vote wins.
    fn effective_warp_limit(&self) -> f64 {
        self.clients
            .values()
            .map(|vote| vote.warp)
            .fold(f64::INFINITY, f64::min)
            .min(MAX_WARP)
    }

    fn paused(&self) -> bool {
        self.clients.values().any(|vote| vote.paused)
    }

    /// Requested (consensual) warp for pacing the loop.
    fn requested_warp(&self) -> f64 {
        self.effective_warp_limit()
    }

    fn apply_input(&mut self, id: &str, input: &ClientInput) {
        let authority = &mut self.authority;
        authority.control_input = DVec3::from_array(input.control_input);
        self.control_mode = input.control_mode;
        // A degenerate wire target must never reach the attitude law:
        // keep the previous target unless the new one is finite nonzero.
        let target = glam::DQuat::from_xyzw(
            input.sas_target_xyzw[0],
            input.sas_target_xyzw[1],
            input.sas_target_xyzw[2],
            input.sas_target_xyzw[3],
        );
        if target.is_finite() && target.length_squared() > 1e-12 {
            authority.sas_target_orientation = target.normalize();
        }
        authority.throttle = input.throttle.clamp(0.0, 1.0);
        authority.engine_active = input.engine_active;
        authority.sas_enabled = input.sas_enabled;
        authority.rcs_enabled = input.rcs_enabled;
        authority.gear_down = input.gear_down;
        for command in &input.commands {
            match command {
                Command::SetWarp { factor } => {
                    if let Some(vote) = self.clients.get_mut(id) {
                        vote.warp = factor.clamp(0.0, MAX_WARP);
                    }
                }
                // Slice semantics (matches the client): staging drives the
                // engine cutoff for the single X-15 plant.
                Command::Stage => authority.engine_active = !authority.engine_active,
                Command::Engine { active } => authority.engine_active = *active,
                Command::Pause { paused } => {
                    if let Some(vote) = self.clients.get_mut(id) {
                        vote.paused = *paused;
                    }
                }
            }
        }
    }

    /// Advance one chunk (or less at 1x); returns sim-seconds advanced.
    fn advance_chunk(&mut self, chunk_s: f64) -> Result<f64, String> {
        if self.paused() || self.authority.flight_error.is_some() {
            return Ok(0.0);
        }
        let before = self.authority.flight_time_s;
        let started = Instant::now();
        self.authority
            .advance_with_budget(&self.ephemeris, self.control_mode, chunk_s, None)
            .map_err(|e| {
                self.authority.engine_active = false;
                self.authority.flight_error = Some(e.to_string());
                eprintln!(
                    "[server] advance failed at t={:.1} mode={:?} thrust={:.0} thr={:.2} eng={} om={:.3}: {e}",
                    self.authority.flight_time_s,
                    self.control_mode,
                    self.authority.thrust_n(),
                    self.authority.throttle,
                    self.authority.engine_active,
                    self.authority.state.angular_velocity_body_rps.length(),
                );
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
            (self.advanced_s / self.wall_s).min(self.effective_warp_limit().max(0.0))
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
            paused: self.paused(),
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
    altitude_m: f64,
) -> Result<(), String> {
    use glam::DQuat;
    let body = ephemeris
        .body(authority.reference_body)
        .map_err(|e| e.to_string())?;
    let origin = ephemeris
        .body_state(authority.reference_body, SimTime::EPOCH)
        .map_err(|e| e.to_string())?;
    let radius = body.radius_m + altitude_m;
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

fn send_frame(
    out: &tokio::sync::mpsc::UnboundedSender<Vec<u8>>,
    kind: u32,
    message: &impl serde::Serialize,
) {
    match thessa_flight_net::encode_frame(kind, message) {
        Ok(frame) => {
            if out.send(frame).is_err() {
                eprintln!("[server] stdout writer gone");
            }
        }
        Err(error) => eprintln!("[server] encode error: {error}"),
    }
}

/// Traffic from every transport into the sim driver. Subscriber
/// channels are tokio unbounded senders: `send` never blocks, so the sync
/// driver and both async/sync writers share one type.
enum Upstream {
    Input(String, ClientInput),
    Leave(String),
    Subscribe(String, tokio::sync::mpsc::UnboundedSender<Vec<u8>>),
}

/// Shared sim driver: one authoritative tick order for every transport.
/// stdio and TCP only differ in how frames arrive and where snapshots go.
struct Driver {
    sim: Sim,
    upstream: Receiver<Upstream>,
    subscribers: Vec<(String, tokio::sync::mpsc::UnboundedSender<Vec<u8>>)>,
    next_deadline: Instant,
    last_snapshot: Instant,
    last_status: Instant,
    exit_when_empty: bool,
}

impl Driver {
    fn broadcast_snapshot(&mut self) {
        let snapshot = self.sim.snapshot();
        let frame = match thessa_flight_net::encode_snapshot(&snapshot) {
            Ok(frame) => frame,
            Err(error) => {
                eprintln!("[server] snapshot encode error: {error}");
                return;
            }
        };
        self.subscribers
            .retain(|(_, tx)| tx.send(frame.clone()).is_ok());
    }

    /// One iteration; `Ok(true)` asks for orderly shutdown (empty room in
    /// exit mode). Inputs drain first so a warp vote lands before pacing.
    fn iterate(&mut self) -> Result<bool, String> {
        while let Ok(message) = self.upstream.try_recv() {
            match message {
                Upstream::Input(id, input) => self.sim.apply_input(&id, &input),
                Upstream::Leave(id) => {
                    self.sim.unregister(&id);
                    self.subscribers.retain(|(other, _)| other != &id);
                }
                Upstream::Subscribe(id, tx) => {
                    self.sim.register(&id);
                    let welcome = thessa_flight_net::Welcome {
                        tick: self.sim.authority.world_tick.0,
                        flight_time_s: self.sim.authority.flight_time_s,
                    };
                    match thessa_flight_net::encode_welcome(&welcome) {
                        Ok(frame) => {
                            if tx.send(frame).is_ok() {
                                self.subscribers.push((id, tx));
                            }
                        }
                        Err(error) => eprintln!("[server] welcome encode error: {error}"),
                    }
                }
            }
        }
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        // No pilots, no flight: hold instead of sprinting at unbounded
        // warp (a fresh server idles at its initial state until the first
        // vote; a deserted one freezes instead of flying away).
        if self.sim.clients.is_empty() {
            std::thread::sleep(std::time::Duration::from_secs_f64(tick_s));
            return Ok(self.exit_when_empty);
        }
        // A latched flight error stops advancing but never the loop:
        // snapshots keep flowing with the error visible (the client
        // offers reset). Killing the driver here would hang every peer.
        if self.sim.authority.flight_error.is_some() {
            std::thread::sleep(std::time::Duration::from_secs_f64(tick_s));
        } else if self.sim.requested_warp() <= 0.0 || self.sim.paused() {
            // Frozen time, live snapshots (clients stay attached).
            std::thread::sleep(std::time::Duration::from_secs_f64(tick_s));
        } else {
            // Paced warp: the deadline advances by chunk/vote, so the
            // long-run rate matches the consensus instead of flat-out.
            // Past machine throughput the deadline falls behind and the
            // loop degrades to flat-out honestly (effective < requested).
            let chunk = if self.sim.requested_warp() <= 1.0 {
                (self.sim.requested_warp() * tick_s).max(tick_s)
            } else {
                // 60 s and up trips the blocking first bake; below it the
                // async worker converges (drift during a 2 s bake stays
                // inside the 5 m adoption gate).
                (self.sim.requested_warp() * WARP_CHUNK_WALL_S).max(WARP_CHUNK_MIN_S)
            };
            if let Err(error) = self.sim.advance_chunk(chunk) {
                // Latched in the authority (engine cut + flight_error);
                // the loop survives so peers see the stop, not a hang.
                eprintln!("[server] advance failed: {error}");
            }
            self.next_deadline +=
                std::time::Duration::from_secs_f64(chunk / self.sim.requested_warp());
            let now = Instant::now();
            if self.next_deadline > now {
                std::thread::sleep(self.next_deadline - now);
            } else {
                self.next_deadline = now;
            }
        }
        let now = Instant::now();
        if now.duration_since(self.last_snapshot).as_secs_f64() >= SNAPSHOT_MIN_INTERVAL_S {
            self.last_snapshot = now;
            self.broadcast_snapshot();
        }
        // Ops telemetry: consensus and throughput at a glance, also the
        // machine-readable hook for the two-peer consensus test.
        if now.duration_since(self.last_status).as_secs_f64() >= 2.0 {
            self.last_status = now;
            eprintln!(
                "[server] status: clients={} reqwarp=x{:.1} effwarp=x{:.1} sim_s={:.0} steps={} rails_s={:.0}",
                self.sim.clients.len(),
                self.sim.requested_warp(),
                self.sim.effective_warp(),
                self.sim.advanced_s,
                self.sim.steps,
                self.sim.rails_s,
            );
        }
        Ok(self.exit_when_empty && self.sim.clients.is_empty())
    }
}

fn run_stdio(sim: Sim) -> Result<(), String> {
    // Stdout writer thread: frames in, bytes out. Stdout is the wire.
    // Joined on shutdown after all senders drop, so the tail flushes.
    let (wire_out, mut wire_rx) = tokio::sync::mpsc::unbounded_channel::<Vec<u8>>();
    let writer = std::thread::spawn(move || {
        let mut stdout = std::io::stdout().lock();
        while let Some(frame) = wire_rx.blocking_recv() {
            if stdout.write_all(&frame).is_err() {
                break;
            }
        }
        let _ = stdout.flush();
    });

    let (upstream_tx, upstream_rx): (Sender<Upstream>, _) = channel();

    // Handshake on the main thread: first frame must be Hello. Frames
    // pipelined after it decode straight into the driver queue.
    let mut decoder = FrameDecoder::new();
    let stdin = std::io::stdin();
    let mut locked = stdin.lock();
    let welcome = loop {
        let mut chunk = [0u8; 65536];
        let n = locked.read(&mut chunk).map_err(|e| e.to_string())?;
        if n == 0 {
            return Err("stdin closed before handshake".into());
        }
        let mut frames = decoder.push(&chunk[..n]).map_err(|e| e.to_string())?;
        if frames.is_empty() {
            continue;
        }
        let envelope =
            thessa_flight_net::decode_frame(&frames.remove(0)).map_err(|e| e.to_string())?;
        if envelope.kind != kind::HELLO {
            return Err(format!("expected HELLO, got kind {}", envelope.kind));
        }
        let hello: thessa_flight_net::Hello =
            thessa_flight_net::decode_payload(&envelope).map_err(|e| e.to_string())?;
        eprintln!("[server] hello from {:?}", hello.client_name);
        for frame in frames {
            if let Ok(envelope) = thessa_flight_net::decode_frame(&frame)
                && envelope.kind == kind::CLIENT_INPUT
                && let Ok(input) = thessa_flight_net::decode_payload::<ClientInput>(&envelope)
            {
                let _ = upstream_tx.send(Upstream::Input("local".into(), input));
            }
        }
        break thessa_flight_net::Welcome {
            tick: sim.authority.world_tick.0,
            flight_time_s: sim.authority.flight_time_s,
        };
    };
    send_frame(&wire_out, kind::WELCOME, &welcome);
    drop(locked);

    // Input pump thread: stdin bytes -> decoded inputs. EOF becomes a
    // Leave, which ends the driver loop after the queued inputs drain
    // (channel FIFO, no drain race). Own lock: the handshake lock above
    // is dropped, so no buffered byte is stranded between the two.
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut locked = stdin.lock();
        let mut decoder = FrameDecoder::new();
        let mut buffer = [0u8; 65536];
        loop {
            match locked.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        for frame in frames {
                            let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
                                continue;
                            };
                            if envelope.kind != kind::CLIENT_INPUT {
                                continue;
                            }
                            if let Ok(input) =
                                thessa_flight_net::decode_payload::<ClientInput>(&envelope)
                                && upstream_tx
                                    .send(Upstream::Input("local".into(), input))
                                    .is_err()
                            {
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
        let _ = upstream_tx.send(Upstream::Leave("local".into()));
    });

    let mut driver = Driver {
        sim,
        upstream: upstream_rx,
        subscribers: Vec::new(),
        next_deadline: Instant::now(),
        last_snapshot: Instant::now(),
        last_status: Instant::now(),
        exit_when_empty: true,
    };
    // Local subscription goes through the driver so Welcome/ordering
    // match the TCP path exactly (handshake Welcome was already sent).
    driver.sim.register("local");
    driver.subscribers.push(("local".to_string(), wire_out));
    // Opening snapshot so the client never waits a full interval.
    driver.broadcast_snapshot();
    while !driver.iterate()? {}
    let sim = driver.sim;
    let _ = writer.join();
    eprintln!(
        "[server] done: {:.1} sim-s, {:.1} wall-s, x{:.1} (votes at exit: {}), {} steps ({:.1} rails-s)",
        sim.advanced_s,
        sim.wall_s,
        sim.advanced_s / sim.wall_s.max(1e-9),
        sim.clients.len(),
        sim.steps,
        sim.rails_s
    );
    Ok(())
}

fn run_measure(mut sim: Sim, target_s: f64) -> Result<(), String> {
    let t0 = Instant::now();
    let mut chunks = 0u64;
    while sim.advanced_s < target_s {
        // One max-batch per chunk, like the live loop at high warp
        // (catch-up rule applies).
        sim.advance_chunk(3600.0)
            .map_err(|e| format!("advance: {e}"))?;
        chunks += 1;
        if chunks.is_multiple_of(500) {
            eprintln!(
                "  [progress] sim={:.0} rails_total={:.0} steps_total={} bake_pending={}",
                sim.advanced_s,
                sim.rails_s,
                sim.steps,
                sim.authority.bake.has_pending()
            );
        }
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

/// One TCP client: handshake, subscribe, pump inputs, forward snapshots.
/// Ends by voting Leave; the driver prunes the subscriber on it.
async fn handle_tcp_conn(stream: tokio::net::TcpStream, peer: String, upstream: Sender<Upstream>) {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    // Small sim frames at 20 Hz: Nagle would hold them up to ~40 ms
    // waiting for ACKs (classic delayed-ACK interplay). QUIC would not
    // have this knob at all; with TCP it must be off explicitly.
    if let Err(error) = stream.set_nodelay(true) {
        eprintln!("[server] tcp nodelay failed: {error}");
        return;
    }
    let (mut reader, mut writer) = stream.into_split();
    let mut decoder = FrameDecoder::new();
    let mut buffer = vec![0u8; 65536];
    // Handshake with a deadline: first frame must be Hello.
    let hello = tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let n = reader.read(&mut buffer).await.map_err(|e| e.to_string())?;
            if n == 0 {
                return Err("eof before hello".to_string());
            }
            let frames = decoder.push(&buffer[..n]).map_err(|e| e.to_string())?;
            if let Some(frame) = frames.into_iter().next() {
                return Ok(frame);
            }
        }
    })
    .await;
    let frame = match hello {
        Ok(Ok(frame)) => frame,
        _ => return,
    };
    let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
        return;
    };
    if envelope.kind != kind::HELLO {
        return;
    }
    let Ok(hello_msg) = thessa_flight_net::decode_payload::<thessa_flight_net::Hello>(&envelope)
    else {
        return;
    };
    // Client id couples the name with the peer so two same-named
    // processes never share a vote.
    let id = format!("{}@{peer}", hello_msg.client_name);
    eprintln!("[server] tcp hello from {id}");
    let (btx, mut brx) = tokio::sync::mpsc::unbounded_channel::<Vec<u8>>();
    if upstream.send(Upstream::Subscribe(id.clone(), btx)).is_err() {
        return;
    }
    // Writer task: subscribed frames -> socket. Ends when the driver
    // prunes us (Leave processed) or the socket breaks.
    let write_task = tokio::spawn(async move {
        while let Some(frame) = brx.recv().await {
            if writer.write_all(&frame).await.is_err() {
                break;
            }
        }
    });
    // Reader loop: socket -> inputs. Any end votes Leave.
    loop {
        match reader.read(&mut buffer).await {
            Ok(0) => break,
            Ok(n) => match decoder.push(&buffer[..n]) {
                Ok(frames) => {
                    for frame in frames {
                        let Ok(envelope) = thessa_flight_net::decode_frame(&frame) else {
                            continue;
                        };
                        if envelope.kind != kind::CLIENT_INPUT {
                            continue;
                        }
                        if let Ok(input) =
                            thessa_flight_net::decode_payload::<ClientInput>(&envelope)
                            && upstream.send(Upstream::Input(id.clone(), input)).is_err()
                        {
                            break;
                        }
                    }
                }
                Err(error) => {
                    eprintln!("[server] tcp frame error from {id}: {error}");
                    break;
                }
            },
            Err(_) => break,
        }
    }
    let _ = upstream.send(Upstream::Leave(id));
    write_task.abort();
}

fn run_tcp(sim: Sim, addr: &str) -> Result<(), String> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?;
    runtime.block_on(async move {
        let (upstream_tx, upstream_rx) = channel::<Upstream>();
        // Sim driver thread: blocking sleeps stay off the tokio workers.
        std::thread::spawn(move || {
            let mut driver = Driver {
                sim,
                upstream: upstream_rx,
                subscribers: Vec::new(),
                next_deadline: Instant::now(),
                last_snapshot: Instant::now(),
                last_status: Instant::now(),
                exit_when_empty: false,
            };
            loop {
                if let Err(error) = driver.iterate() {
                    eprintln!("[server] driver error: {error}");
                    break;
                }
            }
        });
        let listener = tokio::net::TcpListener::bind(addr)
            .await
            .map_err(|e| e.to_string())?;
        eprintln!("[server] tcp listening on {addr}");
        loop {
            let (stream, peer) = listener.accept().await.map_err(|e| e.to_string())?;
            eprintln!("[server] tcp accept {peer}");
            let upstream_tx = upstream_tx.clone();
            tokio::spawn(handle_tcp_conn(stream, peer.to_string(), upstream_tx));
        }
    })
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
    let sim = Sim::new(ephemeris, reference_body, args.vacuum, args.drift)?;
    match args.measure_s {
        Some(target_s) => run_measure(sim, target_s),
        None => match args.tcp_addr {
            Some(addr) => run_tcp(sim, &addr),
            None => run_stdio(sim),
        },
    }
}
