//! Headless authoritative flight server.
//!
//! Owns one [`FlightAuthority`] on a dedicated sim thread.tick loop runs the
//! 120 Hz lattice with no frame budget: warped chunks are bounded only by
//! chunk size (input responsiveness), never by wall time. Transports:
//! length-prefixed frames over stdio (embedded local client) today, TCP
//! (remote clients) next. Stdout is the wire — logs go to stderr.

mod thread_bake;

use std::collections::BTreeMap;
use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::{Duration, Instant};

use glam::DVec3;
use thessa_autopilot::{PlanAction, PlanPoll, TrajectoryPlan, TrajectoryPlanRunner};
use thessa_autopilot_js::{
    ScriptEngine, ScriptLimits, ScriptResult, ScriptScheduler, ScriptSchedulerStep,
};
use thessa_flight_authority::{
    ControlMode, FlightAuthority, FlightPolicy, GuidanceIntent, PropulsionDemand,
    canonical_launch_setup,
};
use thessa_flight_net::{
    AutopilotCommand, AutopilotInput, ClientInput, Command, GuidanceInput, Snapshot,
};
use thessa_protocol::{FrameDecoder, kind};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};
use thread_bake::ThreadBakeQueue;

/// Wall-time quantum for the authoritative driver. It bounds how long the
/// driver can stay away from input/snapshot handling; it is not a simulation
/// rate cap. Each quantum requests `warp * quantum` simulation seconds and
/// the authority serves as much as its work budget/rails path allows.
const DRIVER_WORK_QUANTUM_S: f64 = 0.020;
/// CPU work quantum for ordinary fixed-step physics. When the solver is slower
/// than the requested warp, unserved demand is dropped and effective warp
/// reports the actual result; rails batches remain the high-throughput path.
const SIM_WORK_BUDGET: Duration = Duration::from_millis(4);
/// Retry cadence while a background rails bake is still warming the cache.
const BAKE_POLL_INTERVAL: Duration = Duration::from_millis(2);
/// Wall-pacing for snapshots so high warp does not flood the pipe; the
/// client renders between them.
const SNAPSHOT_MIN_INTERVAL_S: f64 = 0.05;
/// Do not let a producer that ignores input coalescing monopolise the driver.
/// This is an ingress fairness quantum, not a simulation/TPS limit.
const MAX_UPSTREAM_MESSAGES_PER_ITERATION: usize = 256;
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
    /// Latest typed guidance command. Legacy ClientInput clears this so the
    /// two input protocols cannot fight over the same vehicle.
    guidance: Option<(GuidanceIntent, PropulsionDemand)>,
    plan_runner: Option<TrajectoryPlanRunner>,
    last_autopilot_notice: Option<String>,
    clients: std::collections::HashMap<String, ClientVote>,
    advanced_s: f64,
    compute_s: f64,
    wall_started: Instant,
    steps: u64,
    rails_s: f64,
}

/// QuickJS is intentionally pinned to the authoritative driver thread:
/// `rquickjs` contexts and persistent continuations are not `Send`.
struct AutopilotHost {
    scheduler: ScriptScheduler,
    engine: ScriptEngine,
}

impl AutopilotHost {
    fn new() -> Result<Self, String> {
        Ok(Self {
            engine: ScriptEngine::new(ScriptLimits::default())
                .map_err(|error| format!("autopilot init: {error}"))?,
            scheduler: ScriptScheduler::default(),
        })
    }
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
            guidance: None,
            plan_runner: None,
            last_autopilot_notice: None,
            clients: std::collections::HashMap::new(),
            advanced_s: 0.0,
            compute_s: 0.0,
            wall_started: Instant::now(),
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
        if self.clients.is_empty() {
            return 0.0;
        }
        self.clients
            .values()
            .map(|vote| vote.warp)
            .map(|warp| if warp.is_finite() { warp.max(0.0) } else { 0.0 })
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

    fn apply_input(&mut self, id: &str, input: &ClientInput) -> bool {
        if !self.clients.contains_key(id) {
            return false;
        }
        let mut force_snapshot = false;
        self.cancel_autopilot_tasks();
        self.guidance = None;
        let has_engine_command = input
            .commands
            .iter()
            .any(|command| matches!(command, Command::Stage | Command::Engine { .. }));
        // Field-level borrows (no `let authority` alias): the Reset arm
        // below needs `&mut self.authority` together with `&self.ephemeris`,
        // which an aliased borrow would not allow.
        let control_input = DVec3::from_array(input.control_input);
        if control_input.is_finite() {
            self.authority.control_input = control_input.clamp(DVec3::splat(-1.0), DVec3::ONE);
        }
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
            self.authority.sas_target_orientation = target.normalize();
        }
        self.authority.throttle = input.throttle.clamp(0.0, 1.0);
        // A full input carries a last-value state, but an explicit staging or
        // engine command is an edge. Do not let a coalesced stale state field
        // overwrite the result of those preserved events.
        if !has_engine_command {
            self.authority.engine_active = input.engine_active;
        }
        self.authority.sas_enabled = input.sas_enabled;
        self.authority.rcs_enabled = input.rcs_enabled;
        self.authority.gear_down = input.gear_down;
        for command in &input.commands {
            match command {
                Command::SetWarp { factor } => {
                    if let Some(vote) = self.clients.get_mut(id) {
                        let warp = if factor.is_finite() {
                            factor.clamp(0.0, MAX_WARP)
                        } else {
                            0.0
                        };
                        force_snapshot |= vote.warp != warp;
                        vote.warp = warp;
                    }
                }
                // Slice semantics (matches the client): staging drives the
                // engine cutoff for the single X-15 plant.
                Command::Stage => {
                    self.authority.engine_active = !self.authority.engine_active;
                    force_snapshot = true;
                }
                Command::Engine { active } => {
                    force_snapshot |= self.authority.engine_active != *active;
                    self.authority.engine_active = *active;
                }
                Command::Pause { paused } => {
                    if let Some(vote) = self.clients.get_mut(id) {
                        force_snapshot |= vote.paused != *paused;
                        vote.paused = *paused;
                    }
                }
                // Relaunch at the canonical site (same baked-in recipe as
                // the client survey derives it from). Reuses the client
                // reset path verbatim: clock preserved, controls cleared,
                // engine armed at zero throttle. No terrain, no relaunch.
                Command::Reset => {
                    if self.authority.reset_to_launch_site(&self.ephemeris).is_ok() {
                        self.last_autopilot_notice = self.authority.wake_notice.clone();
                        force_snapshot = true;
                    }
                }
            }
        }
        force_snapshot
    }

    fn apply_guidance(&mut self, id: &str, input: &GuidanceInput) -> bool {
        if !self.clients.contains_key(id) {
            return false;
        }
        self.cancel_autopilot_tasks();
        if let Err(error) = input.validate() {
            self.authority.flight_error = Some(format!("invalid guidance input: {error}"));
            self.authority.engine_active = false;
            return true;
        }
        let mode = match self.authority.apply_guidance_intent(&input.intent) {
            Ok(mode) => mode,
            Err(error) => {
                self.authority.flight_error = Some(error.to_string());
                self.authority.engine_active = false;
                return true;
            }
        };
        // The starter vehicle has no reverse or augmentation actuator yet.
        // Apply the native policy at the boundary instead of silently giving
        // a script extra thrust authority.
        let propulsion = FlightPolicy::default().constrain_propulsion(input.propulsion, true, true);
        self.authority.throttle = propulsion.normalized;
        self.authority.engine_active = propulsion.normalized > 0.0;
        self.control_mode = mode;
        self.guidance = Some((input.intent.clone(), propulsion));
        true
    }

    fn apply_autopilot(
        &mut self,
        id: &str,
        input: &AutopilotInput,
        host: &mut AutopilotHost,
    ) -> bool {
        if !self.clients.contains_key(id) {
            return false;
        }
        if let Err(error) = input.validate() {
            return self.fail_autopilot(error);
        }
        match &input.command {
            AutopilotCommand::Cancel => {
                self.cancel_autopilot_tasks();
                self.clear_autopilot_controls();
                true
            }
            AutopilotCommand::Deoptimize { reason } => self
                .plan_runner
                .as_mut()
                .is_some_and(|runner| runner.deoptimize(*reason)),
            AutopilotCommand::SubmitPlan { plan } => self.start_plan(plan.clone(), host),
            AutopilotCommand::StartScript { source } => {
                self.plan_runner = None;
                host.scheduler.cancel_all();
                self.last_autopilot_notice = self.authority.wake_notice.clone();
                let now = SimTime(self.authority.flight_time_s);
                match host.scheduler.start(&host.engine, now, source) {
                    Ok(step) => self.apply_script_step(step, host),
                    Err(error) => self.fail_autopilot(error.to_string()),
                }
            }
        }
    }

    fn apply_script_step(&mut self, step: ScriptSchedulerStep, host: &mut AutopilotHost) -> bool {
        match step {
            ScriptSchedulerStep::Waiting { condition, .. } => {
                eprintln!("[server] autopilot script waiting on {condition:?}");
                self.clear_autopilot_controls();
                true
            }
            ScriptSchedulerStep::Completed { result, .. } => match result {
                ScriptResult::Guidance(intent) => self.apply_script_guidance(intent),
                ScriptResult::Plan(plan) => self.start_plan(plan, host),
                ScriptResult::Diagnostic(diagnostic) => {
                    eprintln!(
                        "[server] autopilot diagnostic {}: {}",
                        diagnostic.code, diagnostic.message
                    );
                    self.authority.wake_notice = Some(format!(
                        "AUTOPILOT {}: {}",
                        diagnostic.code, diagnostic.message
                    ));
                    true
                }
                ScriptResult::Wait(condition) => self.fail_autopilot(format!(
                    "autopilot script returned a wait without an await continuation: {condition:?}"
                )),
            },
        }
    }

    fn apply_script_guidance(&mut self, intent: GuidanceIntent) -> bool {
        let mode = match self.authority.apply_guidance_intent(&intent) {
            Ok(mode) => mode,
            Err(error) => return self.fail_autopilot(error.to_string()),
        };
        let requested = match intent {
            GuidanceIntent::ManualAxes(axes) => PropulsionDemand::new(axes.propulsion)
                .unwrap_or(PropulsionDemand { normalized: 0.0 }),
            _ => PropulsionDemand { normalized: 0.0 },
        };
        let propulsion = FlightPolicy::default().constrain_propulsion(requested, true, true);
        self.authority.throttle = propulsion.normalized;
        self.authority.engine_active = propulsion.normalized > 0.0;
        self.control_mode = mode;
        self.guidance = Some((intent, propulsion));
        true
    }

    fn start_plan(&mut self, plan: TrajectoryPlan, host: &mut AutopilotHost) -> bool {
        host.scheduler.cancel_all();
        self.guidance = None;
        self.last_autopilot_notice = self.authority.wake_notice.clone();
        let now = SimTime(self.authority.flight_time_s);
        let runner = match TrajectoryPlanRunner::new(plan, now) {
            Ok(runner) => runner,
            Err(error) => return self.fail_autopilot(error.to_string()),
        };
        self.plan_runner = Some(runner);
        match self.poll_plan(None) {
            Ok(changed) => changed,
            Err(error) => self.fail_autopilot(error),
        }
    }

    fn prepare_plan(&mut self, event: Option<&str>) -> Result<Option<SimTime>, String> {
        let now = SimTime(self.authority.flight_time_s);
        let poll = self
            .plan_runner
            .as_mut()
            .ok_or_else(|| "no active autopilot plan".to_string())?
            .poll(now, event)
            .map_err(|error| error.to_string())?;
        match poll {
            PlanPoll::Action { action, .. } => {
                let until = match &action {
                    PlanAction::Coast { until }
                    | PlanAction::Burn { until, .. }
                    | PlanAction::Guidance { until, .. } => *until,
                };
                self.apply_plan_action(action)?;
                Ok(Some(until))
            }
            PlanPoll::Waiting { condition, .. } => {
                eprintln!("[server] autopilot plan waiting on {condition:?}");
                self.clear_autopilot_controls();
                Ok(match condition {
                    thessa_autopilot::WaitCondition::At(time) if time.0 > now.0 => Some(time),
                    _ => None,
                })
            }
            PlanPoll::Complete { .. } => {
                self.plan_runner = None;
                self.clear_autopilot_controls();
                Ok(None)
            }
        }
    }

    fn poll_plan(&mut self, event: Option<&str>) -> Result<bool, String> {
        self.prepare_plan(event).map(|_| true)
    }

    fn apply_plan_action(&mut self, action: PlanAction) -> Result<bool, String> {
        match action {
            PlanAction::Coast { .. } => {
                self.clear_autopilot_controls();
                Ok(true)
            }
            PlanAction::Burn { demand, .. } => {
                if demand.force_body_n.length_squared() > 1.0e-24
                    || demand.moment_body_nm.length_squared() > 1.0e-24
                {
                    return Err(
                        "starter authority cannot realize force/moment fields in a plan burn"
                            .into(),
                    );
                }
                let propulsion =
                    FlightPolicy::default().constrain_propulsion(demand.propulsion, true, true);
                self.authority.control_input = DVec3::ZERO;
                self.authority.sas_enabled = false;
                self.authority.throttle = propulsion.normalized;
                self.authority.engine_active = propulsion.normalized > 0.0;
                self.control_mode = ControlMode::Direct;
                self.guidance = Some((GuidanceIntent::ManualAxes(Default::default()), propulsion));
                Ok(true)
            }
            PlanAction::Guidance { intent, .. } => Ok(self.apply_script_guidance(intent)),
        }
    }

    fn clear_autopilot_controls(&mut self) {
        self.guidance = None;
        self.control_mode = ControlMode::Direct;
        self.authority.control_input = DVec3::ZERO;
        self.authority.sas_enabled = false;
        self.authority.throttle = 0.0;
        self.authority.engine_active = false;
    }

    fn cancel_autopilot_tasks(&mut self) {
        self.plan_runner = None;
    }

    fn fail_autopilot(&mut self, error: impl Into<String>) -> bool {
        let error = error.into();
        self.cancel_autopilot_tasks();
        self.clear_autopilot_controls();
        self.authority.flight_error = Some(error.clone());
        eprintln!("[server] autopilot rejected: {error}");
        true
    }

    fn wake_autopilot(
        &mut self,
        host: &mut AutopilotHost,
        event: Option<&str>,
    ) -> Result<bool, String> {
        if host.scheduler.is_empty() {
            return Ok(false);
        }
        let now = SimTime(self.authority.flight_time_s);
        let steps = host
            .scheduler
            .wake(&host.engine, now, event)
            .map_err(|error| error.to_string())?;
        let mut changed = false;
        for step in steps {
            changed |= self.apply_script_step(step, host);
        }
        Ok(changed)
    }

    fn take_autopilot_event(&mut self) -> Option<&'static str> {
        let notice = self.authority.wake_notice.clone()?;
        if self.last_autopilot_notice.as_deref() == Some(notice.as_str()) {
            return None;
        }
        self.last_autopilot_notice = Some(notice.clone());
        let upper = notice.to_ascii_uppercase();
        if upper.contains("IMPACT") {
            Some("impact")
        } else if upper.contains("HORIZON") {
            Some("horizon")
        } else if upper.contains("NODE") {
            Some("node")
        } else if upper.contains("ALARM") {
            Some("alarm")
        } else {
            None
        }
    }

    /// Advance one requested wall quantum (or a benchmark chunk); returns
    /// sim-seconds actually advanced. The optional budget only makes ordinary
    /// physics cooperative; it never changes the fixed solver dt.
    fn advance_chunk(&mut self, chunk_s: f64) -> Result<f64, String> {
        self.advance_chunk_with_budget(chunk_s, None)
    }

    fn advance_chunk_with_budget(
        &mut self,
        chunk_s: f64,
        budget: Option<Duration>,
    ) -> Result<f64, String> {
        if self.paused() || self.authority.flight_error.is_some() {
            return Ok(0.0);
        }
        let mut chunk_s = chunk_s;
        if self.plan_runner.is_some() {
            let now = SimTime(self.authority.flight_time_s);
            if let Some(until) = self.prepare_plan(None)? {
                chunk_s = chunk_s.min((until.0 - now.0).max(0.0));
            }
        }
        let before = self.authority.flight_time_s;
        let started = Instant::now();
        let guidance = self.guidance.clone();
        let result = if let Some((intent, propulsion)) = guidance {
            // Typed guidance uses the same authoritative stepper; the legacy
            // mode is only the compatibility representation used by traces.
            self.authority.advance_guidance_with_budget(
                &self.ephemeris,
                &intent,
                propulsion,
                chunk_s,
                budget,
            )
        } else {
            self.authority
                .advance_with_budget(&self.ephemeris, self.control_mode, chunk_s, budget)
        };
        self.compute_s += started.elapsed().as_secs_f64();
        result.map_err(|e| {
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
        let advanced = self.authority.flight_time_s - before;
        self.advanced_s += advanced;
        self.steps += self.authority.steps_this_frame as u64;
        self.rails_s += self.authority.rails_advanced_this_frame;
        Ok(advanced)
    }

    fn effective_warp(&self) -> f64 {
        let wall_s = self.wall_started.elapsed().as_secs_f64();
        if wall_s <= 0.0 {
            0.0
        } else {
            self.advanced_s / wall_s
        }
    }

    fn wall_s(&self) -> f64 {
        self.wall_started.elapsed().as_secs_f64()
    }

    /// Serve-path launch site: canonical field plus the COAST survey
    /// bookmark, derived exactly like the client survey (same recipe, same
    /// scan), so terrain collisions and spawn state match without ever
    /// transferring world state. Bench paths (`--measure`, `--drift`,
    /// `--vacuum`) skip this deliberately: they place the craft in orbit
    /// and must not pay field-build time or terrain checks.
    fn init_launch_site(&mut self) -> Result<(), String> {
        let started = Instant::now();
        let (field, sites) = canonical_launch_setup(&self.ephemeris)?;
        self.authority
            .initialize_world_site(field, sites[0], &self.ephemeris);
        eprintln!(
            "[server] launch site ready in {:.3}s",
            started.elapsed().as_secs_f64()
        );
        Ok(())
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
            server_compute_s: self.compute_s,
            server_wall_s: self.wall_s(),
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

fn decode_client_input(frame: &[u8]) -> Option<ClientInput> {
    let envelope = thessa_flight_net::decode_frame(frame).ok()?;
    if envelope.kind != kind::CLIENT_INPUT {
        return None;
    }
    thessa_flight_net::decode_payload::<ClientInput>(&envelope).ok()
}

fn decode_guidance_input(frame: &[u8]) -> Option<GuidanceInput> {
    let envelope = thessa_flight_net::decode_frame(frame).ok()?;
    if envelope.kind != kind::GUIDANCE_COMMAND {
        return None;
    }
    thessa_flight_net::decode_payload::<GuidanceInput>(&envelope).ok()
}

fn decode_autopilot_input(frame: &[u8]) -> Option<AutopilotInput> {
    let envelope = thessa_flight_net::decode_frame(frame).ok()?;
    if envelope.kind != kind::AUTOPILOT_COMMAND {
        return None;
    }
    thessa_flight_net::decode_payload::<AutopilotInput>(&envelope).ok()
}

fn enqueue_client_inputs(
    id: &str,
    frames: impl IntoIterator<Item = Vec<u8>>,
    upstream: &Sender<Upstream>,
) -> bool {
    for frame in frames {
        if let Some(input) = decode_client_input(&frame)
            && upstream
                .send(Upstream::Input(id.to_string(), input))
                .is_err()
        {
            return false;
        } else if let Some(guidance) = decode_guidance_input(&frame)
            && upstream
                .send(Upstream::Guidance(id.to_string(), guidance))
                .is_err()
        {
            return false;
        } else if let Some(autopilot) = decode_autopilot_input(&frame)
            && upstream
                .send(Upstream::Autopilot(id.to_string(), autopilot))
                .is_err()
        {
            return false;
        }
    }
    true
}

fn driver_sleep_duration(
    ingress_saturated: bool,
    budget_exhausted: bool,
    bake_wait: bool,
    lag_s: f64,
    requested_warp: f64,
    work_elapsed: Duration,
) -> Duration {
    // Saturation is a yield point, not a 4 ms / 20 ms duty cycle. Returning
    // zero makes the next iteration run immediately after ingress handling.
    if ingress_saturated || budget_exhausted {
        return Duration::ZERO;
    }
    if bake_wait {
        return BAKE_POLL_INTERVAL;
    }
    if lag_s + 1.0e-12 >= thessa_sim_core::WORLD_TICK_S {
        return Duration::ZERO;
    }
    let until_tick_s = (thessa_sim_core::WORLD_TICK_S - lag_s) / requested_warp;
    let remaining_s = until_tick_s - work_elapsed.as_secs_f64();
    if remaining_s > 0.0 {
        Duration::from_secs_f64(remaining_s.clamp(0.001, DRIVER_WORK_QUANTUM_S))
    } else {
        Duration::ZERO
    }
}

/// Traffic from every transport into the sim driver. Subscriber
/// channels are tokio unbounded senders: `send` never blocks, so the sync
/// driver and both async/sync writers share one type.
enum Upstream {
    Input(String, ClientInput),
    Guidance(String, GuidanceInput),
    Autopilot(String, AutopilotInput),
    Leave(String),
    Subscribe(String, tokio::sync::mpsc::UnboundedSender<Vec<u8>>),
}

/// Mailbox for one client during a driver quantum. Continuous controls are
/// last-value-wins; event-like commands remain ordered. Warp and pause are
/// state votes, so an older vote can be replaced without losing a stage or
/// toggle event behind it.
#[derive(Default)]
struct PendingInput {
    latest: Option<ClientInput>,
    commands: Vec<Command>,
}

impl PendingInput {
    fn push(&mut self, input: ClientInput) {
        let mut input = input;
        let commands = std::mem::take(&mut input.commands);
        self.latest = Some(input);
        for command in commands {
            match command {
                Command::SetWarp { .. } => {
                    self.commands
                        .retain(|queued| !matches!(queued, Command::SetWarp { .. }));
                    self.commands.push(command);
                }
                Command::Pause { .. } => {
                    self.commands
                        .retain(|queued| !matches!(queued, Command::Pause { .. }));
                    self.commands.push(command);
                }
                // Stage and explicit engine commands are edge/event-like:
                // every one must reach the authoritative state in order.
                event => self.commands.push(event),
            }
        }
    }

    fn take(&mut self) -> Option<ClientInput> {
        let mut input = self.latest.take()?;
        input.commands = std::mem::take(&mut self.commands);
        Some(input)
    }
}

/// Shared sim driver: one authoritative tick order for every transport.
/// stdio and TCP only differ in how frames arrive and where snapshots go.
struct Driver {
    sim: Sim,
    autopilot: AutopilotHost,
    upstream: Receiver<Upstream>,
    subscribers: Vec<(String, tokio::sync::mpsc::UnboundedSender<Vec<u8>>)>,
    /// Desired authoritative sim time generated from real wall time and the
    /// current consensus warp. Actual service catches this target when the
    /// machine is fast enough; when it is not, the driver stays busy between
    /// quanta and effective warp falls honestly.
    pacing_target_s: f64,
    last_pacing: Instant,
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

    /// Drain a bounded ingress slice. The limit only prevents a hot producer
    /// from monopolising the sim thread; per-client continuous input is still
    /// coalesced and event commands are retained in order.
    fn drain_upstream(&mut self) -> (bool, bool) {
        let mut pending = BTreeMap::<String, PendingInput>::new();
        let mut force_snapshot = false;
        let mut processed = 0;
        while processed < MAX_UPSTREAM_MESSAGES_PER_ITERATION {
            let message = match self.upstream.try_recv() {
                Ok(message) => message,
                Err(std::sync::mpsc::TryRecvError::Empty)
                | Err(std::sync::mpsc::TryRecvError::Disconnected) => break,
            };
            processed += 1;
            match message {
                Upstream::Input(id, input) => pending.entry(id).or_default().push(input),
                Upstream::Guidance(id, input) => {
                    self.autopilot.scheduler.cancel_all();
                    force_snapshot |= self.sim.apply_guidance(&id, &input);
                }
                Upstream::Autopilot(id, input) => {
                    force_snapshot |= self.sim.apply_autopilot(&id, &input, &mut self.autopilot);
                }
                Upstream::Leave(id) => {
                    if let Some(mut queued) = pending.remove(&id)
                        && let Some(input) = queued.take()
                    {
                        let _ = self.sim.apply_input(&id, &input);
                    }
                    self.sim.unregister(&id);
                    self.subscribers.retain(|(other, _)| other != &id);
                    force_snapshot = true;
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
                            } else {
                                self.sim.unregister(&id);
                            }
                        }
                        Err(error) => eprintln!("[server] welcome encode error: {error}"),
                    }
                    force_snapshot = true;
                }
            }
        }
        for (id, mut queued) in pending {
            if let Some(input) = queued.take() {
                self.autopilot.scheduler.cancel_all();
                force_snapshot |= self.sim.apply_input(&id, &input);
            }
        }
        (
            force_snapshot,
            processed == MAX_UPSTREAM_MESSAGES_PER_ITERATION,
        )
    }

    /// One iteration; `Ok(true)` asks for orderly shutdown (empty room in
    /// exit mode). Inputs are coalesced before the pacing target is sampled.
    fn iterate(&mut self) -> Result<bool, String> {
        let warp_before_inputs = self.sim.requested_warp();
        let (mut force_snapshot, ingress_saturated) = self.drain_upstream();
        let autopilot_event = self.sim.take_autopilot_event();
        match self
            .sim
            .wake_autopilot(&mut self.autopilot, autopilot_event)
        {
            Ok(changed) => force_snapshot |= changed,
            Err(error) => {
                self.autopilot.scheduler.cancel_all();
                force_snapshot |= self.sim.fail_autopilot(error);
            }
        }
        if let Some(event) = autopilot_event
            && self.sim.plan_runner.is_some()
        {
            match self.sim.poll_plan(Some(event)) {
                Ok(changed) => force_snapshot |= changed,
                Err(error) => {
                    self.autopilot.scheduler.cancel_all();
                    force_snapshot |= self.sim.fail_autopilot(error);
                }
            }
        }
        let warp_after_inputs = self.sim.requested_warp();
        if warp_after_inputs < warp_before_inputs {
            // A lower vote changes the wall-time target immediately. Keeping
            // the old target would make the driver spend the next many
            // iterations catching up to demand generated at the old warp.
            self.pacing_target_s = self.sim.advanced_s;
        }
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        // No pilots, no flight: hold instead of sprinting at unbounded
        // warp (a fresh server idles at its initial state until the first
        // vote; a deserted one freezes instead of flying away).
        if self.sim.clients.is_empty() {
            self.pacing_target_s = self.sim.advanced_s;
            self.last_pacing = Instant::now();
            std::thread::sleep(Duration::from_secs_f64(tick_s));
            return Ok(self.exit_when_empty);
        }
        // A latched flight error stops advancing but never the loop:
        // snapshots keep flowing with the error visible (the client
        // offers reset). Killing the driver here would hang every peer.
        if self.sim.authority.flight_error.is_some() {
            self.pacing_target_s = self.sim.advanced_s;
            self.last_pacing = Instant::now();
            std::thread::sleep(Duration::from_secs_f64(tick_s));
        } else if self.sim.requested_warp() <= 0.0 || self.sim.paused() {
            // Frozen time, live snapshots (clients stay attached).
            self.pacing_target_s = self.sim.advanced_s;
            self.last_pacing = Instant::now();
            std::thread::sleep(Duration::from_secs_f64(tick_s));
        } else {
            let now = Instant::now();
            let wall_delta = now.duration_since(self.last_pacing).as_secs_f64();
            self.last_pacing = now;
            let requested_warp = self.sim.requested_warp();
            self.pacing_target_s += wall_delta * requested_warp;
            let demand_s = (self.pacing_target_s - self.sim.advanced_s).max(0.0);
            let mut chunk = if demand_s + 1.0e-12 >= tick_s {
                demand_s
            } else {
                0.0
            };
            if let Some(wake) = self.autopilot.scheduler.next_time() {
                let now = SimTime(self.sim.authority.flight_time_s);
                if wake.0 > now.0 {
                    chunk = chunk.min(wake.0 - now.0);
                }
            }
            let advanced_before = self.sim.advanced_s;
            let mut advanced_delta = 0.0;
            let mut budget_exhausted = false;
            if chunk > 0.0
                && let Err(error) = self
                    .sim
                    .advance_chunk_with_budget(chunk, Some(SIM_WORK_BUDGET))
            {
                // Latched in the authority (engine cut + flight_error);
                // the loop survives so peers see the stop, not a hang.
                eprintln!("[server] advance failed: {error}");
            }
            if chunk > 0.0 {
                // Sim::advanced_s is cumulative; use its delta so a pending
                // bake is distinguishable from an already-running flight.
                advanced_delta = self.sim.advanced_s - advanced_before;
                budget_exhausted = self.sim.authority.work_budget_exhausted;
            }
            let autopilot_event = self.sim.take_autopilot_event();
            match self
                .sim
                .wake_autopilot(&mut self.autopilot, autopilot_event)
            {
                Ok(changed) => force_snapshot |= changed,
                Err(error) => {
                    self.autopilot.scheduler.cancel_all();
                    force_snapshot |= self.sim.fail_autopilot(error);
                }
            }
            if let Some(event) = autopilot_event
                && self.sim.plan_runner.is_some()
            {
                match self.sim.poll_plan(Some(event)) {
                    Ok(changed) => force_snapshot |= changed,
                    Err(error) => {
                        self.autopilot.scheduler.cancel_all();
                        force_snapshot |= self.sim.fail_autopilot(error);
                    }
                }
            }
            let lag_s = (self.pacing_target_s - self.sim.advanced_s).max(0.0);
            let bake_wait = chunk > 0.0
                && advanced_delta <= 0.0
                && self.sim.authority.waiting_for_rails_bake
                && self.sim.authority.bake.has_pending();
            let work_elapsed = now.elapsed();
            let sleep_s = driver_sleep_duration(
                ingress_saturated,
                budget_exhausted,
                bake_wait,
                lag_s,
                requested_warp,
                work_elapsed,
            );
            if !sleep_s.is_zero() {
                std::thread::sleep(sleep_s);
            }
        }
        let now = Instant::now();
        if force_snapshot
            || now.duration_since(self.last_snapshot).as_secs_f64() >= SNAPSHOT_MIN_INTERVAL_S
        {
            self.last_snapshot = now;
            self.broadcast_snapshot();
        }
        // Ops telemetry: consensus and throughput at a glance, also the
        // machine-readable hook for the two-peer consensus test. The prefix
        // up to rails_s stays stable for parsing; compute/wall are appended
        // so effective warp remains reproducible as sim_s / wall_s.
        if now.duration_since(self.last_status).as_secs_f64() >= 2.0 {
            self.last_status = now;
            eprintln!(
                "[server] status: clients={} reqwarp=x{:.1} effwarp=x{:.1} sim_s={:.0} steps={} rails_s={:.0} compute_s={:.1} wall_s={:.1}",
                self.sim.clients.len(),
                self.sim.requested_warp(),
                self.sim.effective_warp(),
                self.sim.advanced_s,
                self.sim.steps,
                self.sim.rails_s,
                self.sim.compute_s,
                self.sim.wall_s(),
            );
        }
        Ok(self.exit_when_empty && self.sim.clients.is_empty())
    }
}

fn run_stdio(mut sim: Sim) -> Result<(), String> {
    // Canonical terrain: same site the client survey selects. Soft-fails
    // to terrain-free flight (today's embedded behavior) instead of
    // refusing to serve.
    if let Err(error) = sim.init_launch_site() {
        eprintln!("[server] launch site unavailable ({error}); terrain-free flight");
    }
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
    let (welcome, post_handshake_frames) = loop {
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
        break (
            thessa_flight_net::Welcome {
                tick: sim.authority.world_tick.0,
                flight_time_s: sim.authority.flight_time_s,
            },
            frames,
        );
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
        // Keep the decoder that performed the handshake: it may contain the
        // partial prefix/body of the first pipelined input frame.
        let mut decoder = decoder;
        let mut buffer = [0u8; 65536];
        if !enqueue_client_inputs("local", post_handshake_frames, &upstream_tx) {
            return;
        }
        loop {
            match locked.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        if !enqueue_client_inputs("local", frames, &upstream_tx) {
                            return;
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
        autopilot: AutopilotHost::new()?,
        upstream: upstream_rx,
        subscribers: Vec::new(),
        pacing_target_s: 0.0,
        last_pacing: Instant::now(),
        last_snapshot: Instant::now(),
        last_status: Instant::now(),
        exit_when_empty: true,
    };
    // Local subscription goes through the driver so Welcome/ordering
    // match the TCP path exactly (handshake Welcome was already sent).
    driver.sim.register("local");
    driver.sim.wall_started = Instant::now();
    driver.subscribers.push(("local".to_string(), wire_out));
    // Opening snapshot so the client never waits a full interval.
    driver.broadcast_snapshot();
    while !driver.iterate()? {}
    let sim = driver.sim;
    let _ = writer.join();
    eprintln!(
        "[server] done: {:.1} sim-s, {:.1} compute-s, x{:.1} (votes at exit: {}), {} steps ({:.1} rails-s)",
        sim.advanced_s,
        sim.compute_s,
        sim.effective_warp(),
        sim.clients.len(),
        sim.steps,
        sim.rails_s
    );
    Ok(())
}

fn run_measure(mut sim: Sim, target_s: f64) -> Result<(), String> {
    let t0 = Instant::now();
    sim.wall_started = t0;
    let mut chunks = 0u64;
    while sim.advanced_s < target_s {
        // One max-batch per chunk, like the live loop at high warp
        // (catch-up rule applies).
        let advanced = sim
            .advance_chunk(3600.0)
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
        if advanced <= 0.0
            && sim.authority.waiting_for_rails_bake
            && sim.authority.bake.has_pending()
        {
            // A cold drift measurement waits for the same asynchronous bake
            // as the live driver. Avoid burning a core while the worker runs.
            std::thread::sleep(BAKE_POLL_INTERVAL);
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
            let mut frames = decoder.push(&buffer[..n]).map_err(|e| e.to_string())?;
            if !frames.is_empty() {
                let frame = frames.remove(0);
                return Ok((frame, frames));
            }
        }
    })
    .await;
    let (frame, post_handshake_frames) = match hello {
        Ok(Ok(frames)) => frames,
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
    if !enqueue_client_inputs(&id, post_handshake_frames, &upstream) {
        let _ = upstream.send(Upstream::Leave(id));
        write_task.abort();
        return;
    }
    // Reader loop: socket -> inputs. Any end votes Leave.
    loop {
        match reader.read(&mut buffer).await {
            Ok(0) => break,
            Ok(n) => match decoder.push(&buffer[..n]) {
                Ok(frames) => {
                    if !enqueue_client_inputs(&id, frames, &upstream) {
                        break;
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

fn run_tcp(mut sim: Sim, addr: &str) -> Result<(), String> {
    if let Err(error) = sim.init_launch_site() {
        eprintln!("[server] launch site unavailable ({error}); terrain-free flight");
    }
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?;
    runtime.block_on(async move {
        let (upstream_tx, upstream_rx) = channel::<Upstream>();
        // Sim driver thread: blocking sleeps stay off the tokio workers.
        std::thread::spawn(move || {
            let autopilot = match AutopilotHost::new() {
                Ok(autopilot) => autopilot,
                Err(error) => {
                    eprintln!("[server] {error}");
                    return;
                }
            };
            let mut driver = Driver {
                sim,
                autopilot,
                upstream: upstream_rx,
                subscribers: Vec::new(),
                pacing_target_s: 0.0,
                last_pacing: Instant::now(),
                last_snapshot: Instant::now(),
                last_status: Instant::now(),
                exit_when_empty: false,
            };
            driver.sim.wall_started = Instant::now();
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

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    use thessa_autopilot::PlanExecutionMode;

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
            gear_down: false,
            commands,
        }
    }

    #[test]
    fn typed_guidance_reaches_the_authoritative_stepper() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        sim.register("pilot");
        let guidance = GuidanceInput {
            tick: 0,
            intent: GuidanceIntent::Attitude {
                target_body_to_inertial: DQuat::from_rotation_y(0.05),
                roll_policy: thessa_flight_authority::RollPolicy::Hold,
            },
            propulsion: PropulsionDemand::new(0.0).unwrap(),
        };
        assert!(sim.apply_guidance("pilot", &guidance));
        sim.advance_chunk(0.02).expect("advance typed guidance");
        assert_eq!(sim.control_mode, ControlMode::Navball);
        assert!(sim.authority.state.position_inertial_m.is_finite());
    }

    #[test]
    fn server_owns_script_waits_and_wakes_them_on_sim_time() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");

        let input = AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(0.05); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        };
        assert!(sim.apply_autopilot("pilot", &input, &mut host));
        assert_eq!(host.scheduler.pending(), 1);
        assert!(sim.guidance.is_none());
        assert_eq!(host.scheduler.next_time(), Some(SimTime(0.05)));

        sim.authority.flight_time_s = 0.05;
        assert!(sim.wake_autopilot(&mut host, None).expect("wake script"));
        assert!(matches!(
            sim.guidance,
            Some((
                GuidanceIntent::AngularRate { .. },
                PropulsionDemand { normalized: 0.0 }
            ))
        ));
        assert_eq!(sim.control_mode, ControlMode::Rate);
    }

    #[test]
    fn server_executes_and_deoptimizes_a_submitted_plan() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");
        let plan = TrajectoryPlan {
            id: thessa_flight_authority::TrajectoryPlanId(33),
            segments: vec![
                thessa_autopilot::TrajectorySegment::Coast { duration_s: 0.02 },
                thessa_autopilot::TrajectorySegment::Guidance {
                    duration_s: 0.1,
                    intent: GuidanceIntent::ManualAxes(Default::default()),
                },
            ],
            bakeability: thessa_autopilot::Bakeability::Guarded,
        };
        let input = AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitPlan { plan },
        };
        assert!(sim.apply_autopilot("pilot", &input, &mut host));
        assert!(sim.plan_runner.is_some());
        assert!(!sim.authority.engine_active);

        sim.authority.flight_time_s = 0.02;
        sim.poll_plan(None).expect("advance plan cursor");
        assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 1);
        assert!(sim.plan_runner.as_ref().unwrap().mode() == PlanExecutionMode::Baked);

        let deoptimize = AutopilotInput {
            tick: 1,
            command: AutopilotCommand::Deoptimize {
                reason: thessa_autopilot::PlanDeoptimizationReason::GuardInvalidated,
            },
        };
        assert!(sim.apply_autopilot("pilot", &deoptimize, &mut host));
        assert_eq!(
            sim.plan_runner.as_ref().unwrap().mode(),
            PlanExecutionMode::Live
        );
    }

    #[test]
    fn server_wakes_a_plan_from_an_authoritative_event() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");
        let plan = TrajectoryPlan {
            id: thessa_flight_authority::TrajectoryPlanId(34),
            segments: vec![
                thessa_autopilot::TrajectorySegment::Wait {
                    condition: thessa_autopilot::WaitCondition::Event("impact".into()),
                },
                thessa_autopilot::TrajectorySegment::Guidance {
                    duration_s: 0.1,
                    intent: GuidanceIntent::AngularRate {
                        rate_body_rps: DVec3::new(0.1, 0.0, 0.0),
                    },
                },
            ],
            bakeability: thessa_autopilot::Bakeability::Live,
        };
        let input = AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitPlan { plan },
        };
        assert!(sim.apply_autopilot("pilot", &input, &mut host));
        assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 0);

        sim.authority.wake_notice = Some("IMPACT detected".into());
        let event = sim.take_autopilot_event();
        assert_eq!(event, Some("impact"));
        assert!(sim.poll_plan(event).expect("wake plan"));
        assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 1);
        assert_eq!(sim.control_mode, ControlMode::Rate);
    }

    #[test]
    fn coalescing_keeps_latest_controls_and_all_event_commands() {
        let mut pending = PendingInput::default();
        let mut first = input(vec![Command::Stage]);
        first.control_input = [0.1, 0.0, 0.0];
        pending.push(first);
        let mut second = input(vec![
            Command::Stage,
            Command::Pause { paused: true },
            Command::SetWarp { factor: 128.0 },
        ]);
        second.control_input = [0.2, 0.0, 0.0];
        pending.push(second);
        let mut third = input(vec![
            Command::Pause { paused: false },
            Command::SetWarp { factor: 256.0 },
        ]);
        third.control_input = [0.3, 0.0, 0.0];
        pending.push(third);

        let merged = pending.take().expect("coalesced input");
        assert_eq!(merged.control_input, [0.3, 0.0, 0.0]);
        assert_eq!(merged.commands[0], Command::Stage);
        assert_eq!(merged.commands[1], Command::Stage);
        assert_eq!(merged.commands[2], Command::Pause { paused: false });
        assert_eq!(merged.commands[3], Command::SetWarp { factor: 256.0 });
    }

    #[test]
    fn coalesced_engine_events_match_sequential_application() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sequential = Sim::new(ephemeris.clone(), reference_body, false, true).expect("sim");
        sequential.register("pilot");
        let first = input(vec![Command::Engine { active: true }]);
        let second = input(vec![Command::Stage]);
        let _ = sequential.apply_input("pilot", &first);
        let _ = sequential.apply_input("pilot", &second);
        let expected = sequential.authority.engine_active;

        let mut pending = PendingInput::default();
        pending.push(first);
        pending.push(second);
        let merged = pending.take().expect("merged input");
        let mut coalesced = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        coalesced.register("pilot");
        let _ = coalesced.apply_input("pilot", &merged);
        assert_eq!(coalesced.authority.engine_active, expected);
    }

    #[test]
    fn saturated_work_quantum_does_not_sleep_as_a_fixed_duty_cycle() {
        assert_eq!(
            driver_sleep_duration(false, true, false, 0.0, 256.0, Duration::from_millis(100),),
            Duration::ZERO
        );
        assert_eq!(
            driver_sleep_duration(true, false, false, 0.0, 256.0, Duration::from_millis(100),),
            Duration::ZERO
        );
        assert_eq!(
            driver_sleep_duration(false, false, true, 0.0, 256.0, Duration::ZERO,),
            BAKE_POLL_INTERVAL
        );
    }

    #[test]
    fn lowering_warp_rebases_old_pacing_debt() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let (upstream_tx, upstream_rx) = channel();
        let mut driver = Driver {
            sim,
            autopilot: AutopilotHost::new().expect("autopilot host"),
            upstream: upstream_rx,
            subscribers: Vec::new(),
            pacing_target_s: 1.0e9,
            last_pacing: Instant::now(),
            last_snapshot: Instant::now(),
            last_status: Instant::now(),
            exit_when_empty: false,
        };
        driver.sim.register("pilot");
        driver.sim.clients.get_mut("pilot").expect("pilot").warp = 256.0;
        upstream_tx
            .send(Upstream::Input(
                "pilot".into(),
                input(vec![Command::SetWarp { factor: 64.0 }]),
            ))
            .expect("queue warp vote");

        assert!(!driver.iterate().expect("driver iteration"));
        assert!(
            driver.pacing_target_s < 100.0,
            "old pacing debt survived warp reduction: {}",
            driver.pacing_target_s
        );
    }

    #[test]
    fn pause_vote_forces_a_prompt_snapshot_even_at_high_warp() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let (upstream_tx, upstream_rx) = channel();
        let (snapshot_tx, mut snapshot_rx) = tokio::sync::mpsc::unbounded_channel();
        let mut driver = Driver {
            sim,
            autopilot: AutopilotHost::new().expect("autopilot host"),
            upstream: upstream_rx,
            subscribers: vec![("pilot".into(), snapshot_tx)],
            pacing_target_s: 0.0,
            last_pacing: Instant::now(),
            last_snapshot: Instant::now(),
            last_status: Instant::now(),
            exit_when_empty: false,
        };
        driver.sim.register("pilot");
        upstream_tx
            .send(Upstream::Input(
                "pilot".into(),
                input(vec![
                    Command::SetWarp { factor: MAX_WARP },
                    Command::Pause { paused: true },
                ]),
            ))
            .expect("queue input");

        let started = Instant::now();
        assert!(!driver.iterate().expect("driver iteration"));
        assert!(started.elapsed() < Duration::from_millis(100));
        let frame = snapshot_rx.try_recv().expect("forced snapshot");
        let mut decoder = FrameDecoder::new();
        let frames = decoder.push(&frame).expect("snapshot frame");
        let envelope = thessa_flight_net::decode_frame(&frames[0]).expect("envelope");
        let snapshot: Snapshot = thessa_flight_net::decode_payload(&envelope).expect("snapshot");
        assert!(snapshot.paused);
        assert_eq!(snapshot.server_compute_s, 0.0);
        assert!(snapshot.server_wall_s > 0.0);
    }

    #[test]
    fn transport_handshake_preserves_pipelined_and_partial_frames() {
        let hello = thessa_flight_net::encode_hello(&thessa_flight_net::Hello {
            client_name: "test".into(),
        })
        .expect("hello");
        let input = thessa_flight_net::encode_input(&input(vec![Command::Stage])).expect("input");
        let mut stream = hello.clone();
        stream.extend_from_slice(&input);
        let split = hello.len() + input.len() / 2;
        let mut decoder = FrameDecoder::new();
        let first = decoder.push(&stream[..split]).expect("first read");
        assert_eq!(first.len(), 1);
        assert_eq!(
            thessa_flight_net::decode_frame(&first[0])
                .expect("hello envelope")
                .kind,
            kind::HELLO
        );
        let second = decoder.push(&stream[split..]).expect("second read");
        assert_eq!(second.len(), 1);
        assert_eq!(
            thessa_flight_net::decode_frame(&second[0])
                .expect("input envelope")
                .kind,
            kind::CLIENT_INPUT
        );

        let mut pipelined = hello;
        pipelined.extend_from_slice(&input);
        pipelined.extend_from_slice(&input);
        let frames = FrameDecoder::new();
        let mut frames = frames;
        let all = frames.push(&pipelined).expect("pipelined read");
        assert_eq!(all.len(), 3, "all post-hello frames must survive");
    }
}

#[cfg(test)]
mod reset_tests {
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
            gear_down: false,
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
        pending.push(input(vec![Command::SetWarp { factor: 64.0 }]));
        pending.push(input(vec![Command::Reset]));
        pending.push(input(vec![Command::SetWarp { factor: 128.0 }]));
        let merged = pending.take().expect("merged input");
        assert_eq!(merged.commands.len(), 2);
        assert_eq!(merged.commands[0], Command::Reset);
        assert_eq!(merged.commands[1], Command::SetWarp { factor: 128.0 });
    }
}
