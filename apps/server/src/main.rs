//! Headless authoritative flight server.
//!
//! Owns one [`FlightAuthority`] on a dedicated sim thread.tick loop runs the
//! 120 Hz lattice with no frame budget: warped chunks are bounded only by
//! chunk size (input responsiveness), never by wall time. Transports:
//! length-prefixed frames over stdio (embedded local client) today, TCP
//! (remote clients) next. Stdout is the wire — logs go to stderr.

mod thread_bake;

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io::{Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use glam::DVec3;
use thessa_autopilot::{
    AutopilotGraph, BlockStatus, GraphBlock, GraphControlAction, GraphNode, GraphNodeConfig,
    GraphNodeOutcome, GraphRunner, GraphValue, ImpactSite, LandingSite, NodeKind, PlanAction,
    PlanPoll, PortDirection, PortType, TrajectoryPlan, TrajectoryPlanRunner, WaitCondition,
};
use thessa_autopilot_js::{
    ScriptEngine, ScriptLimits, ScriptResult, ScriptScheduler, ScriptSchedulerStep,
};
use thessa_flight_authority::{
    ControlMode, FlightAuthority, FlightPolicy, GuidanceIntent, ObstacleReport, PropulsionDemand,
    canonical_launch_setup,
};
use thessa_flight_control::{ControlDemand, DirectionFrame, DirectionTarget, RollPolicy};
use thessa_flight_net::{
    AutopilotCommand, AutopilotInput, BurnDirectionCommand, ClientInput, Command, GuidanceInput,
    Snapshot,
};
use thessa_maneuver::{
    BurnSegment, EngineSpec, FiniteBurnPlan, ManeuverPlan, NodeExecutor, PlanValidation,
    SegmentDirection, SegmentExecutor, SteeringSample,
};
use thessa_protocol::{FrameDecoder, kind};
use thessa_sim_core::{BakedEphemeris, BodyId, BodyState, ScheduledKind, SimTime, SystemConfig};
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AutopilotEvent {
    Impact,
    Horizon,
    Node,
    Alarm,
    Stage,
    EngineReady,
}

impl AutopilotEvent {
    fn name(self) -> &'static str {
        match self {
            Self::Impact => "impact",
            Self::Horizon => "horizon",
            Self::Node => "node",
            Self::Alarm => "alarm",
            Self::Stage => "stage",
            Self::EngineReady => "engine-ready",
        }
    }
}

impl From<ScheduledKind> for AutopilotEvent {
    fn from(kind: ScheduledKind) -> Self {
        match kind {
            ScheduledKind::RailsImpact { .. } => Self::Impact,
            ScheduledKind::RailsHorizon => Self::Horizon,
            ScheduledKind::ManeuverNode { .. } => Self::Node,
            ScheduledKind::BurnSegment { .. } => Self::Node,
            ScheduledKind::Alarm => Self::Alarm,
        }
    }
}

fn client_input_takes_over(previous: Option<&ClientInput>, input: &ClientInput) -> bool {
    if input.commands.iter().any(|command| {
        matches!(
            command,
            Command::Stage
                | Command::Engine { .. }
                | Command::Reset
                | Command::ExecuteManeuver { .. }
                | Command::ExecuteBurnPlan { .. }
        )
    }) {
        return true;
    }
    let Some(previous) = previous else {
        return input.control_input.iter().any(|axis| axis.abs() > 1.0e-12);
    };
    input.control_input != previous.control_input
        || input.control_mode != previous.control_mode
        || input.sas_target_xyzw != previous.sas_target_xyzw
        || input.throttle != previous.throttle
        || input.engine_active != previous.engine_active
        || input.sas_enabled != previous.sas_enabled
        || input.rcs_enabled != previous.rcs_enabled
        || input.gear_down != previous.gear_down
}

/// Driver around the authority: inputs in, snapshots out, warp accounting.
struct Sim {
    authority: FlightAuthority,
    ephemeris: BakedEphemeris,
    control_mode: ControlMode,
    /// Latest typed guidance command. Legacy ClientInput clears this so the
    /// two input protocols cannot fight over the same vehicle.
    guidance: Option<(GuidanceIntent, PropulsionDemand)>,
    plan_demand: Option<ControlDemand>,
    autopilot_graph: Option<AutopilotGraph>,
    graph_runner: Option<GraphRunner>,
    graph_block: NativeGraphBlock,
    /// Declared autopilot targets stay data-only until a later landing/
    /// impact planner consumes them and asks the field for obstacle evidence.
    landing_site: Option<LandingSite>,
    landing_obstacles: Option<ObstacleReport>,
    impact_site: Option<ImpactSite>,
    impact_obstacles: Option<ObstacleReport>,
    plan_runner: Option<TrajectoryPlanRunner>,
    /// Active maneuver-plan execution (ExecuteManeuver block). Drives
    /// `guidance` through a `NodeExecutor`; cleared on completion/abort,
    /// after latching a zero-throttle hold so handoff is never abrupt.
    maneuver_execution: Option<NodeExecutor>,
    /// Active finite-burn execution (ExecuteBurnPlan block). Drives
    /// `guidance` through a `SegmentExecutor` with the same handoff
    /// contract. Mutually exclusive with `maneuver_execution`: starting
    /// one refuses while the other is active rather than fighting it.
    burn_execution: Option<SegmentExecutor>,
    clients: std::collections::HashMap<String, ClientVote>,
    /// Connection-order pilot lease. The first registered client owns all
    /// vehicle controls; on departure the lease moves to the earliest client
    /// still connected, making transfer deterministic across transports.
    pilot_owner: Option<String>,
    client_order: Vec<String>,
    last_client_inputs: std::collections::HashMap<String, ClientInput>,
    autopilot_events: VecDeque<AutopilotEvent>,
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

/// Structural native graph host used until domain-specific blocks are
/// registered. It forwards matching typed values and makes a wait node an
/// event boundary; it never receives mutable access to flight state.
#[derive(Debug, Default)]
struct NativeGraphBlock {
    waited: std::collections::BTreeSet<thessa_autopilot::NodeId>,
    control_actions: Vec<GraphControlAction>,
}

impl GraphBlock for NativeGraphBlock {
    fn execute(
        &mut self,
        node: &GraphNode,
        inputs: &BTreeMap<String, GraphValue>,
    ) -> GraphNodeOutcome {
        let waiting_node = matches!(node.kind, NodeKind::Wait)
            || matches!(
                node.kind,
                NodeKind::Custom {
                    wait_capable: true,
                    ..
                }
            );
        if waiting_node && self.waited.insert(node.id) {
            return GraphNodeOutcome::Wait {
                condition: match node.config.as_ref() {
                    Some(GraphNodeConfig::Wait { condition }) => condition.clone(),
                    _ => WaitCondition::Event(node.name.clone()),
                },
            };
        }
        match node.config.as_ref() {
            Some(GraphNodeConfig::Guidance { intent, propulsion }) => {
                self.control_actions.push(GraphControlAction::Guidance {
                    intent: intent.clone(),
                    propulsion: *propulsion,
                })
            }
            Some(GraphNodeConfig::Demand { demand }) => {
                self.control_actions
                    .push(GraphControlAction::Demand { demand: *demand });
            }
            _ => {}
        }
        let mut outputs = BTreeMap::new();
        for port in node
            .ports
            .iter()
            .filter(|port| port.direction == PortDirection::Output)
        {
            let configured_number = match node.config.as_ref() {
                Some(GraphNodeConfig::Number { value }) if port.ty == PortType::Number => {
                    Some(GraphValue::Number(*value))
                }
                _ => None,
            };
            let value = configured_number.or_else(|| {
                inputs
                    .get(&port.name)
                    .cloned()
                    .or_else(|| {
                        (inputs.len() == 1)
                            .then(|| inputs.values().next().cloned())
                            .flatten()
                    })
                    .or(match port.ty {
                        PortType::Unit => Some(GraphValue::Unit),
                        PortType::Bool => Some(GraphValue::Bool(false)),
                        PortType::Number => Some(GraphValue::Number(0.0)),
                        PortType::Any => Some(GraphValue::Unit),
                        ty => Some(GraphValue::Typed(ty)),
                    })
            });
            let Some(value) = value else {
                return GraphNodeOutcome::Fail {
                    diagnostic: thessa_autopilot::Diagnostic {
                        kind: thessa_autopilot::DiagnosticKind::NoSolution,
                        code: "GRAPH_OUTPUT_UNRESOLVED".into(),
                        message: format!(
                            "graph node {:?} has no value for output {:?}",
                            node.id, port.name
                        ),
                        value: None,
                        limit: None,
                    },
                };
            };
            outputs.insert(port.name.clone(), value);
        }
        GraphNodeOutcome::Complete { outputs }
    }

    fn take_control_actions(&mut self) -> Vec<GraphControlAction> {
        std::mem::take(&mut self.control_actions)
    }
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
            authority.stop_propulsion();
        }
        Ok(Self {
            authority,
            ephemeris,
            control_mode: ControlMode::Navball,
            guidance: None,
            plan_demand: None,
            autopilot_graph: None,
            graph_runner: None,
            graph_block: NativeGraphBlock::default(),
            landing_site: None,
            landing_obstacles: None,
            impact_site: None,
            impact_obstacles: None,
            plan_runner: None,
            maneuver_execution: None,
            burn_execution: None,
            clients: std::collections::HashMap::new(),
            pilot_owner: None,
            client_order: Vec::new(),
            last_client_inputs: std::collections::HashMap::new(),
            autopilot_events: VecDeque::new(),
            advanced_s: 0.0,
            compute_s: 0.0,
            wall_started: Instant::now(),
            steps: 0,
            rails_s: 0.0,
        })
    }

    /// Register a client (handshake); idempotent reconnects retain their
    /// original connection-order position. The first client becomes pilot.
    fn register(&mut self, id: &str) {
        if self.clients.contains_key(id) {
            return;
        }
        self.clients.insert(
            id.to_string(),
            ClientVote {
                warp: 1.0,
                paused: false,
            },
        );
        self.client_order.push(id.to_string());
        if self.pilot_owner.is_none() {
            self.pilot_owner = Some(id.to_string());
            eprintln!("[server] pilot owner assigned: {id}");
        }
    }

    fn is_pilot(&self, id: &str) -> bool {
        self.pilot_owner.as_deref() == Some(id)
    }

    /// Drop a client's votes on disconnect. The caller cancels the host
    /// scheduler before this method; changing ownership also clears all
    /// stale vehicle control state.
    fn unregister(&mut self, id: &str) {
        self.clients.remove(id);
        self.client_order.retain(|client| client != id);
        self.last_client_inputs.remove(id);
        if self.is_pilot(id) {
            self.cancel_autopilot_tasks();
            self.clear_autopilot_controls();
            self.authority.flight_error = None;
            self.pilot_owner = self.client_order.first().cloned();
            if let Some(owner) = &self.pilot_owner {
                eprintln!("[server] pilot owner transferred: {id} -> {owner}");
            } else {
                eprintln!("[server] pilot owner released: {id}");
            }
        }
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
        if let Err(error) = input.validate() {
            eprintln!("[server] rejected invalid input from {id}: {error}");
            return false;
        }
        if !self.is_pilot(id) {
            // Spectators can participate in shared warp/pause policy but can
            // never alter flight controls or generate an edge command.
            let mut force_snapshot = false;
            for command in &input.commands {
                match command {
                    Command::SetWarp { factor } => {
                        if let Some(vote) = self.clients.get_mut(id) {
                            let warp = factor.clamp(0.0, MAX_WARP);
                            force_snapshot |= vote.warp != warp;
                            vote.warp = warp;
                        }
                    }
                    Command::Pause { paused } => {
                        if let Some(vote) = self.clients.get_mut(id) {
                            force_snapshot |= vote.paused != *paused;
                            vote.paused = *paused;
                        }
                    }
                    _ => {}
                }
            }
            return force_snapshot;
        }
        let mut force_snapshot = false;
        let previous = self
            .last_client_inputs
            .insert(id.to_string(), input.clone());
        if client_input_takes_over(previous.as_ref(), input) {
            self.cancel_autopilot_tasks();
            self.clear_autopilot_controls();
        }
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
        // A full input carries a last-value state, but an explicit staging or
        // engine command is an edge. Do not let a coalesced stale state field
        // overwrite the result of those preserved events.
        let active = if has_engine_command {
            self.authority.engine_active
        } else {
            input.engine_active
        };
        self.authority.set_legacy_propulsion(input.throttle, active);
        self.authority.sas_enabled = input.sas_enabled;
        self.authority.rcs_enabled = input.rcs_enabled;
        self.authority.gear_down = input.gear_down;
        let mut engine_command_seen = false;
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
                // engine cutoff for the single X-15 plant. Single-vehicle
                // slice: no VehicleId branching yet (fleet/M5 future work).
                // Emit domain events so `Wait(Event("stage"))` /
                // `All(stage, engine-ready)` guards can resolve instead of
                // parking forever.
                Command::Stage => {
                    // PendingInput preserves multiple edge commands from
                    // separate frames. Each such frame used to cut off the
                    // active autopilot before applying its edge; reproduce
                    // that cutoff between coalesced engine events without
                    // resetting the coalesced flight controls.
                    if engine_command_seen {
                        self.authority.stop_propulsion();
                    }
                    engine_command_seen = true;
                    self.authority.engine_active = !self.authority.engine_active;
                    self.autopilot_events.push_back(AutopilotEvent::Stage);
                    if self.authority.engine_active {
                        self.autopilot_events.push_back(AutopilotEvent::EngineReady);
                    }
                    force_snapshot = true;
                }
                Command::Engine { active } => {
                    if engine_command_seen {
                        self.authority.stop_propulsion();
                    }
                    engine_command_seen = true;
                    force_snapshot |= self.authority.engine_active != *active;
                    let became_ready = *active && !self.authority.engine_active;
                    self.authority.engine_active = *active;
                    if became_ready {
                        self.autopilot_events.push_back(AutopilotEvent::EngineReady);
                    }
                }
                Command::ExecuteManeuver { nodes } => {
                    // Wire cap: node vectors are unbounded on the transport.
                    const MAX_MANEUVER_NODES: usize = 16;
                    let rejected = |sim: &mut Self, reason: String| {
                        sim.authority.wake_notice = Some(format!("maneuver rejected: {reason}"));
                    };
                    if nodes.len() > MAX_MANEUVER_NODES {
                        rejected(
                            self,
                            format!("{} nodes over cap {MAX_MANEUVER_NODES}", nodes.len()),
                        );
                        continue;
                    }
                    let mut plan_nodes = Vec::with_capacity(nodes.len());
                    let mut bad_node = false;
                    for node in nodes {
                        match thessa_maneuver::ManeuverNode::new(
                            SimTime(node.epoch_s),
                            DVec3::from_array(node.delta_v_mps),
                        ) {
                            Ok(node) => plan_nodes.push(node),
                            Err(_) => {
                                bad_node = true;
                                break;
                            }
                        }
                    }
                    if bad_node {
                        rejected(self, "non-finite node".into());
                        continue;
                    }
                    let plan = match ManeuverPlan::new(
                        plan_nodes,
                        self.authority.state.position_inertial_m,
                        self.authority.state.velocity_inertial_mps,
                        SimTime(self.authority.flight_time_s),
                    ) {
                        Ok(plan) => plan,
                        Err(error) => {
                            rejected(self, format!("invalid plan: {error}"));
                            continue;
                        }
                    };
                    match self.start_maneuver_execution(plan) {
                        Ok(()) => {
                            force_snapshot = true;
                        }
                        Err(error) => rejected(self, error),
                    }
                }
                Command::ExecuteBurnPlan {
                    engine_thrust_n,
                    engine_exhaust_velocity_mps,
                    initial_mass_kg,
                    segments,
                } => {
                    // Wire cap: segments are unbounded on the transport;
                    // scaled up from the node cap for split burns.
                    const MAX_BURN_SEGMENTS: usize = 64;
                    let rejected = |sim: &mut Self, reason: String| {
                        sim.authority.wake_notice = Some(format!("burn plan rejected: {reason}"));
                    };
                    if segments.len() > MAX_BURN_SEGMENTS {
                        rejected(
                            self,
                            format!("{} segments over cap {MAX_BURN_SEGMENTS}", segments.len()),
                        );
                        continue;
                    }
                    let mut plan_segments = Vec::with_capacity(segments.len());
                    let mut bad_segment = false;
                    for segment in segments {
                        let direction = match &segment.direction {
                            BurnDirectionCommand::Inertial { unit } => {
                                SegmentDirection::Inertial(DVec3::from_array(*unit))
                            }
                            BurnDirectionCommand::Prograde => SegmentDirection::Prograde,
                            BurnDirectionCommand::Retrograde => SegmentDirection::Retrograde,
                            BurnDirectionCommand::Rtn {
                                central,
                                radial,
                                transverse,
                                normal,
                            } => match self.ephemeris.body_id(central.as_str()) {
                                Some(id) => SegmentDirection::Rtn {
                                    central: id,
                                    radial: *radial,
                                    transverse: *transverse,
                                    normal: *normal,
                                },
                                None => {
                                    bad_segment = true;
                                    break;
                                }
                            },
                        };
                        plan_segments.push(BurnSegment {
                            start: SimTime(segment.start_s),
                            duration_s: segment.duration_s,
                            planned_dv_mps: segment.planned_dv_mps,
                            direction,
                            throttle_01: segment.throttle_01,
                        });
                    }
                    if bad_segment {
                        rejected(self, "unknown rtn central body".into());
                        continue;
                    }
                    let plan = match FiniteBurnPlan::new(
                        plan_segments,
                        EngineSpec {
                            thrust_n: *engine_thrust_n,
                            exhaust_velocity_mps: *engine_exhaust_velocity_mps,
                        },
                        *initial_mass_kg,
                        self.authority.state.position_inertial_m,
                        self.authority.state.velocity_inertial_mps,
                        SimTime(self.authority.flight_time_s),
                    ) {
                        Ok(plan) => plan,
                        Err(error) => {
                            rejected(self, format!("invalid burn plan: {error}"));
                            continue;
                        }
                    };
                    match self.start_burn_execution(plan) {
                        Ok(()) => {
                            force_snapshot = true;
                        }
                        Err(error) => rejected(self, error),
                    }
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
                        self.autopilot_events.clear();
                        force_snapshot = true;
                    }
                }
            }
        }
        force_snapshot
    }

    fn apply_guidance(&mut self, id: &str, input: &GuidanceInput) -> bool {
        if !self.is_pilot(id) {
            return false;
        }
        if let Err(error) = input.validate() {
            eprintln!("[server] rejected invalid guidance from {id}: {error}");
            return false;
        }
        self.cancel_autopilot_tasks();
        self.clear_autopilot_controls();
        self.apply_guidance_command(input.intent.clone(), input.propulsion)
    }

    fn apply_guidance_command(
        &mut self,
        intent: GuidanceIntent,
        requested_propulsion: PropulsionDemand,
    ) -> bool {
        let mode = match self
            .authority
            .apply_guidance_intent(&self.ephemeris, &intent)
        {
            Ok(mode) => mode,
            Err(error) => {
                self.authority.flight_error = Some(error.to_string());
                self.authority.stop_propulsion();
                return false;
            }
        };
        let propulsion =
            FlightPolicy::default().constrain_propulsion(requested_propulsion, true, true);
        self.plan_demand = None;
        if let Err(error) = self.authority.set_propulsion_target(propulsion) {
            self.authority.flight_error = Some(error.to_string());
            self.authority.stop_propulsion();
            return false;
        }
        self.control_mode = mode;
        self.guidance = Some((intent, propulsion));
        true
    }

    fn apply_autopilot(
        &mut self,
        id: &str,
        input: &AutopilotInput,
        host: &mut AutopilotHost,
    ) -> bool {
        if !self.is_pilot(id) {
            return false;
        }
        if let Err(error) = input.validate() {
            eprintln!("[server] rejected invalid autopilot from {id}: {error}");
            return false;
        }
        match &input.command {
            AutopilotCommand::SubmitGraph { graph } => {
                host.scheduler.cancel_all();
                self.submit_graph(graph.clone())
            }
            AutopilotCommand::ClearGraph => {
                host.scheduler.cancel_all();
                self.autopilot_graph = None;
                self.cancel_autopilot_tasks();
                self.clear_autopilot_controls();
                true
            }
            AutopilotCommand::Cancel => {
                host.scheduler.cancel_all();
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
                self.cancel_autopilot_tasks();
                host.scheduler.cancel_all();
                self.clear_autopilot_controls();
                let now = SimTime(self.authority.flight_time_s);
                match host.scheduler.start(&host.engine, now, source) {
                    Ok(step) => self.apply_script_step(step, host),
                    Err(error) => self.fail_autopilot(error.to_string()),
                }
            }
        }
    }

    fn submit_graph(&mut self, graph: AutopilotGraph) -> bool {
        let validation = match graph.validate() {
            Ok(validation) => validation,
            Err(errors) => return self.fail_autopilot(graph_errors(errors)),
        };
        self.plan_runner = None;
        self.clear_autopilot_controls();
        self.graph_block = NativeGraphBlock::default();
        self.graph_runner = match GraphRunner::new(graph.clone()) {
            Ok(runner) => Some(runner),
            Err(errors) => return self.fail_autopilot(graph_errors(errors)),
        };
        self.autopilot_graph = Some(graph);
        if let Err(error) = self.poll_graph(None) {
            return self.fail_autopilot(error);
        }
        self.authority.wake_notice = Some(format!(
            "AUTOPILOT GRAPH READY nodes={}",
            validation.topological_order.len()
        ));
        true
    }

    fn poll_graph(&mut self, event: Option<&str>) -> Result<(), String> {
        let (state, waiting_status) = {
            let Some(runner) = self.graph_runner.as_mut() else {
                return Ok(());
            };
            let state = runner
                .poll(
                    SimTime(self.authority.flight_time_s),
                    event,
                    &mut self.graph_block,
                )
                .map_err(|error| error.to_string())?;
            let status = match &state {
                thessa_autopilot::GraphRunState::Waiting { node, .. } => {
                    Some((*node, runner.status(*node).unwrap_or(BlockStatus::Waiting)))
                }
                _ => None,
            };
            (state, status)
        };
        let actions = self.graph_block.take_control_actions();
        for action in actions {
            let applied = match action {
                GraphControlAction::Guidance { intent, propulsion } => {
                    self.apply_guidance_command(intent, propulsion)
                }
                GraphControlAction::Demand { demand } => {
                    if let Err(error) = demand.validate_envelope() {
                        self.authority.flight_error = Some(error.to_string());
                        false
                    } else {
                        let demand = FlightPolicy::default().constrain_demand(demand, true, true);
                        if let Err(error) = demand.validate() {
                            self.authority.flight_error = Some(error.to_string());
                            false
                        } else {
                            self.guidance = None;
                            self.plan_demand = Some(demand);
                            self.control_mode = ControlMode::Direct;
                            self.authority.control_input = DVec3::ZERO;
                            self.authority.sas_enabled = false;
                            self.authority
                                .set_propulsion_target(demand.propulsion)
                                .is_ok()
                        }
                    }
                }
            };
            if !applied {
                return Err(self
                    .authority
                    .flight_error
                    .clone()
                    .unwrap_or_else(|| "native graph control action failed".into()));
            }
        }
        match state {
            thessa_autopilot::GraphRunState::Progress { .. } => Ok(()),
            thessa_autopilot::GraphRunState::Waiting { node, .. } => {
                let status = waiting_status
                    .map(|(_, status)| status)
                    .unwrap_or(BlockStatus::Waiting);
                self.authority.wake_notice = Some(format!(
                    "AUTOPILOT GRAPH WAIT node={:?} status={:?}",
                    node, status
                ));
                Ok(())
            }
            thessa_autopilot::GraphRunState::Complete => {
                self.authority.wake_notice = Some("AUTOPILOT GRAPH COMPLETE".into());
                Ok(())
            }
            thessa_autopilot::GraphRunState::Failed { diagnostic, .. }
            | thessa_autopilot::GraphRunState::Aborted { diagnostic, .. } => Err(format!(
                "autopilot graph {}: {}",
                diagnostic.code, diagnostic.message
            )),
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
                ScriptResult::LandingSite(site) => self.declare_landing_site(site),
                ScriptResult::ImpactSite(site) => self.declare_impact_site(site),
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

    fn declare_landing_site(&mut self, site: LandingSite) -> bool {
        if let Err(error) = site.validate() {
            return self.fail_autopilot(error.to_string());
        }
        let obstacles = match self
            .authority
            .obstacle_report(site.center_dir, site.radius_m)
        {
            Ok(obstacles) => obstacles,
            Err(error) => return self.fail_autopilot(error),
        };
        let obstacle_state = if obstacles.is_some() {
            "ready"
        } else {
            "pending"
        };
        self.landing_site = Some(site);
        self.landing_obstacles = obstacles;
        self.authority.wake_notice = Some(format!(
            "AUTOPILOT LANDING SITE center=({:.6},{:.6},{:.6}) radius={:.1}m obstacles={}",
            site.center_dir[0],
            site.center_dir[1],
            site.center_dir[2],
            site.radius_m,
            obstacle_state
        ));
        true
    }

    fn declare_impact_site(&mut self, site: ImpactSite) -> bool {
        if let Err(error) = site.validate() {
            return self.fail_autopilot(error.to_string());
        }
        let obstacles = match self
            .authority
            .obstacle_report(site.center_dir, site.radius_m)
        {
            Ok(obstacles) => obstacles,
            Err(error) => return self.fail_autopilot(error),
        };
        let obstacle_state = if obstacles.is_some() {
            "ready"
        } else {
            "pending"
        };
        self.impact_site = Some(site);
        self.impact_obstacles = obstacles;
        self.authority.wake_notice = Some(format!(
            "AUTOPILOT IMPACT SITE center=({:.6},{:.6},{:.6}) radius={:.1}m obstacles={}",
            site.center_dir[0],
            site.center_dir[1],
            site.center_dir[2],
            site.radius_m,
            obstacle_state
        ));
        true
    }

    fn apply_script_guidance(&mut self, intent: GuidanceIntent) -> bool {
        let requested = match intent {
            GuidanceIntent::ManualAxes(axes) => PropulsionDemand::new(axes.propulsion)
                .unwrap_or(PropulsionDemand { normalized: 0.0 }),
            _ => PropulsionDemand { normalized: 0.0 },
        };
        if self.apply_guidance_command(intent, requested) {
            true
        } else {
            self.fail_autopilot(
                self.authority
                    .flight_error
                    .clone()
                    .unwrap_or_else(|| "autopilot guidance failed".into()),
            )
        }
    }

    /// Start maneuver-plan execution (ExecuteManeuver block): validate the
    /// plan against now, arm one scheduler wake per node, and hand
    /// `guidance` to the executor on every tick until done/aborted.
    /// Refuses when a trajectory `plan_demand` is active rather than
    /// silently fighting it; legacy client input clears `guidance` as
    /// usual, which also preempts an in-flight execution.
    /// Invoked from tests and the `ExecuteManeuver` wire command.
    fn start_maneuver_execution(&mut self, plan: ManeuverPlan) -> Result<(), String> {
        let now = SimTime(self.authority.flight_time_s);
        match plan.validate_for_execution(now) {
            PlanValidation::Executable => {}
            PlanValidation::Empty => return Err("maneuver plan is empty".into()),
            PlanValidation::Stale {
                now_s,
                first_node_s,
            } => {
                return Err(format!(
                    "maneuver plan is stale (now {now_s:.1}, first node {first_node_s:.1})"
                ));
            }
        }
        if self.plan_demand.is_some() {
            return Err("trajectory plan demand is active; clear it first".into());
        }
        if self.burn_execution.is_some() {
            return Err("burn plan execution is active; clear it first".into());
        }
        let executor =
            NodeExecutor::new(&plan).map_err(|error| format!("maneuver plan: {error}"))?;
        for (epoch, delta_v) in executor.to_scheduler_events() {
            self.authority.scheduler.arm(
                ScheduledKind::ManeuverNode {
                    delta_v_mps: delta_v,
                },
                epoch,
            );
        }
        self.maneuver_execution = Some(executor);
        Ok(())
    }

    /// Poll the active execution before stepping: integrate measured thrust
    /// acceleration (total minus gravity, same tick) and map the command to
    /// typed guidance. Idle/done latch a zero-throttle attitude hold, never
    /// an abrupt handoff.
    fn poll_maneuver_execution(&mut self) -> Result<(), String> {
        let Some(executor) = self.maneuver_execution.as_mut() else {
            return Ok(());
        };
        let now = SimTime(self.authority.flight_time_s);
        let thrust_accel = match &self.authority.last_forces {
            Some(forces) => {
                forces.acceleration_inertial_mps2
                    - self.authority.last_gravity_acceleration_inertial_mps2
            }
            None => DVec3::ZERO,
        };
        let output = executor
            .poll(now, thrust_accel)
            .map_err(|error| format!("maneuver poll: {error}"))?;
        let hold = || GuidanceIntent::Attitude {
            target_body_to_inertial: self.authority.state.orientation_body_to_inertial,
            roll_policy: RollPolicy::Hold,
        };
        if output.done {
            self.guidance = Some((hold(), PropulsionDemand::new(0.0).unwrap()));
            self.maneuver_execution = None;
            self.authority.wake_notice = Some("maneuver complete".into());
            return Ok(());
        }
        let direction = output.command.point_inertial;
        let intent = if direction == DVec3::ZERO {
            hold()
        } else {
            let target =
                DirectionTarget::new(direction, DirectionFrame::Inertial).map_err(|error| {
                    self.maneuver_execution = None;
                    format!("maneuver direction: {error}")
                })?;
            GuidanceIntent::VelocityDirection {
                direction: target,
                roll_policy: RollPolicy::Hold,
            }
        };
        let propulsion = PropulsionDemand::new(output.command.throttle_01).map_err(|error| {
            self.maneuver_execution = None;
            format!("maneuver throttle: {error}")
        })?;
        self.guidance = Some((intent, propulsion));
        Ok(())
    }

    /// Map an executor direction+throttle command to typed guidance (shared
    /// by node and segment execution: both executors resolve their frames
    /// to inertial before emitting). Zero direction latches an attitude
    /// hold, never an abrupt handoff.
    fn execution_guidance(
        &self,
        command: thessa_maneuver::ExecutionCommand,
    ) -> Result<
        (
            thessa_flight_control::GuidanceIntent,
            thessa_flight_control::PropulsionDemand,
        ),
        String,
    > {
        use thessa_flight_control::{GuidanceIntent, PropulsionDemand};
        let hold = || GuidanceIntent::Attitude {
            target_body_to_inertial: self.authority.state.orientation_body_to_inertial,
            roll_policy: RollPolicy::Hold,
        };
        let direction = command.point_inertial;
        let intent = if direction == DVec3::ZERO {
            hold()
        } else {
            let target = DirectionTarget::new(direction, DirectionFrame::Inertial)
                .map_err(|error| format!("maneuver direction: {error}"))?;
            GuidanceIntent::VelocityDirection {
                direction: target,
                roll_policy: RollPolicy::Hold,
            }
        };
        let propulsion = PropulsionDemand::new(command.throttle_01)
            .map_err(|error| format!("maneuver throttle: {error}"))?;
        Ok((intent, propulsion))
    }

    /// Start finite-burn execution (ExecuteBurnPlan block): same contract
    /// as node execution (validate against now, refuse while another
    /// execution or plan demand is active, arm one scheduler wake per
    /// segment start). Invoked from tests and the wire command.
    fn start_burn_execution(&mut self, plan: FiniteBurnPlan) -> Result<(), String> {
        let now = SimTime(self.authority.flight_time_s);
        match plan.validate_for_execution(now) {
            PlanValidation::Executable => {}
            PlanValidation::Empty => return Err("burn plan is empty".into()),
            PlanValidation::Stale {
                now_s,
                first_node_s,
            } => {
                return Err(format!(
                    "burn plan is stale (now {now_s:.1}, first segment {first_node_s:.1})"
                ));
            }
        }
        if self.plan_demand.is_some() {
            return Err("trajectory plan demand is active; clear it first".into());
        }
        if self.maneuver_execution.is_some() {
            return Err("maneuver node execution is active; clear it first".into());
        }
        let executor =
            SegmentExecutor::new(&plan).map_err(|error| format!("burn plan: {error}"))?;
        for (start, _, planned_dv_mps) in executor.to_scheduler_events() {
            self.authority
                .scheduler
                .arm(ScheduledKind::BurnSegment { planned_dv_mps }, start);
        }
        self.burn_execution = Some(executor);
        Ok(())
    }

    /// Poll the active burn execution before stepping: same thrust
    /// measurement as node execution, plus the flight sample (inertial
    /// velocity/position and the LVLH central state for RTN segments,
    /// resolved per active segment). Idle/done latch a zero-throttle
    /// attitude hold.
    fn poll_burn_execution(&mut self) -> Result<(), String> {
        let Some(executor) = self.burn_execution.as_mut() else {
            return Ok(());
        };
        let now = SimTime(self.authority.flight_time_s);
        let thrust_accel = match &self.authority.last_forces {
            Some(forces) => {
                forces.acceleration_inertial_mps2
                    - self.authority.last_gravity_acceleration_inertial_mps2
            }
            None => DVec3::ZERO,
        };
        // LVLH central state for the active segment (ORIGIN when the
        // segment steers inertially — the executor ignores it there).
        let mut central = BodyState::ORIGIN;
        if let Some(segment) = executor.active_segment() {
            if let thessa_maneuver::SegmentDirection::Rtn { central: body, .. } = segment.direction
            {
                central = self
                    .ephemeris
                    .body_state(body, now)
                    .map_err(|error| format!("burn central body: {error}"))?;
            }
        }
        let sample = SteeringSample {
            velocity_inertial_mps: self.authority.state.velocity_inertial_mps,
            position_inertial_m: self.authority.state.position_inertial_m,
            central,
        };
        let output = executor
            .poll(now, thrust_accel, &sample)
            .map_err(|error| format!("burn poll: {error}"))?;
        if output.done {
            let hold = thessa_flight_control::GuidanceIntent::Attitude {
                target_body_to_inertial: self.authority.state.orientation_body_to_inertial,
                roll_policy: RollPolicy::Hold,
            };
            self.guidance = Some((
                hold,
                thessa_flight_control::PropulsionDemand::new(0.0).unwrap(),
            ));
            self.burn_execution = None;
            self.authority.wake_notice = Some("burn plan complete".into());
            return Ok(());
        }
        match self.execution_guidance(output.command) {
            Ok((intent, propulsion)) => {
                self.guidance = Some((intent, propulsion));
                Ok(())
            }
            Err(error) => {
                self.burn_execution = None;
                Err(error)
            }
        }
    }

    fn start_plan(&mut self, plan: TrajectoryPlan, host: &mut AutopilotHost) -> bool {
        host.scheduler.cancel_all();
        self.graph_runner = None;
        self.clear_autopilot_controls();
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
                demand
                    .validate_envelope()
                    .map_err(|error| error.to_string())?;
                let demand = FlightPolicy::default().constrain_demand(demand, true, true);
                let propulsion = demand.propulsion;
                self.plan_demand = Some(demand);
                self.authority.control_input = DVec3::ZERO;
                self.authority.sas_enabled = false;
                self.authority
                    .set_propulsion_target(propulsion)
                    .map_err(|error| error.to_string())?;
                self.control_mode = ControlMode::Direct;
                self.guidance = None;
                Ok(true)
            }
            PlanAction::Guidance { intent, .. } => Ok(self.apply_script_guidance(intent)),
        }
    }

    fn clear_autopilot_controls(&mut self) {
        self.plan_demand = None;
        self.guidance = None;
        // Manual takeover disengages in-flight executions, same as any
        // other automation (MechJeb-style disengage on stick input).
        self.maneuver_execution = None;
        self.burn_execution = None;
        self.control_mode = ControlMode::Direct;
        self.authority.control_input = DVec3::ZERO;
        self.authority.sas_enabled = false;
        self.authority.stop_propulsion();
    }

    fn cancel_autopilot_tasks(&mut self) {
        self.plan_runner = None;
        self.graph_runner = None;
        // The block's one-shot wait parking is only meaningful while its
        // runner is alive. Reset it here so a later graph re-parks its
        // waits instead of completing them immediately from stale state.
        // (Single-pass runners execute each node once; true loop/retry
        // repetition is still a missing Loop combinator, not this set.)
        self.graph_block = NativeGraphBlock::default();
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
        events: &[AutopilotEvent],
    ) -> Result<bool, String> {
        let now = SimTime(self.authority.flight_time_s);
        let mut changed = false;
        if events.is_empty() {
            if !host.scheduler.is_empty() {
                let steps = host
                    .scheduler
                    .wake(&host.engine, now, None)
                    .map_err(|error| error.to_string())?;
                for step in steps {
                    changed |= self.apply_script_step(step, host);
                }
            }
            self.poll_graph(None)?;
            if self.plan_runner.is_some() {
                self.poll_plan(None)?;
            }
        } else {
            for event in events {
                if !host.scheduler.is_empty() {
                    let steps = host
                        .scheduler
                        .wake(&host.engine, now, Some(event.name()))
                        .map_err(|error| error.to_string())?;
                    for step in steps {
                        changed |= self.apply_script_step(step, host);
                    }
                }
                self.poll_graph(Some(event.name()))?;
                if self.plan_runner.is_some() {
                    self.poll_plan(Some(event.name()))?;
                }
            }
        }
        Ok(changed)
    }

    fn take_autopilot_events(&mut self) -> Vec<AutopilotEvent> {
        self.autopilot_events.drain(..).collect()
    }

    fn next_autopilot_wake(&self, host: &AutopilotHost) -> Option<SimTime> {
        [
            host.scheduler.next_time(),
            self.graph_runner.as_ref().and_then(GraphRunner::next_time),
            self.plan_runner
                .as_ref()
                .and_then(|runner| runner.next_time()),
        ]
        .into_iter()
        .flatten()
        .min_by(|a, b| a.0.total_cmp(&b.0))
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
        // Maneuver executions poll before stepping so commands ride the
        // freshest thrust measurement from the previous tick.
        self.poll_maneuver_execution()?;
        self.poll_burn_execution()?;
        let mut chunk_s = chunk_s;
        if self.plan_runner.is_some() {
            let now = SimTime(self.authority.flight_time_s);
            if let Some(until) = self.prepare_plan(None)? {
                chunk_s = chunk_s.min((until.0 - now.0).max(0.0));
            }
        }
        let before = self.authority.flight_time_s;
        let started = Instant::now();
        let plan_demand = self.plan_demand;
        let guidance = self.guidance.clone();
        let result = if let Some(demand) = plan_demand {
            self.authority.advance_control_demand_with_budget(
                &self.ephemeris,
                demand,
                chunk_s,
                budget,
            )
        } else if let Some((intent, propulsion)) = guidance {
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
        self.autopilot_events.extend(
            self.authority
                .take_wake_events()
                .into_iter()
                .map(|event| AutopilotEvent::from(event.kind)),
        );
        self.poll_graph(None)?;
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
    authority.set_legacy_propulsion(1.0, true);
    authority.control_input = DVec3::ZERO;
    Ok(())
}

const RELIABLE_OUTBOUND_CAPACITY: usize = 32;

struct OutboundState {
    reliable: VecDeque<Vec<u8>>,
    latest_snapshot: Option<Vec<u8>>,
    closed: bool,
}

/// Per-client outbound mailbox. Welcome and future control frames are
/// bounded reliable messages; snapshots are a single latest-wins slot. The
/// simulation thread only takes a short mutex and never waits for a socket.
struct OutboundMailbox {
    state: Mutex<OutboundState>,
    blocking_wake: Condvar,
    async_wake: tokio::sync::Notify,
}

impl OutboundMailbox {
    fn new() -> Self {
        Self {
            state: Mutex::new(OutboundState {
                reliable: VecDeque::with_capacity(RELIABLE_OUTBOUND_CAPACITY),
                latest_snapshot: None,
                closed: false,
            }),
            blocking_wake: Condvar::new(),
            async_wake: tokio::sync::Notify::new(),
        }
    }

    fn push_reliable(&self, frame: Vec<u8>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed || state.reliable.len() >= RELIABLE_OUTBOUND_CAPACITY {
            return false;
        }
        state.reliable.push_back(frame);
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    fn replace_snapshot(&self, frame: Vec<u8>) -> bool {
        let Ok(mut state) = self.state.lock() else {
            return false;
        };
        if state.closed {
            return false;
        }
        state.latest_snapshot = Some(frame);
        drop(state);
        self.blocking_wake.notify_one();
        self.async_wake.notify_one();
        true
    }

    fn try_next(&self) -> Option<Vec<u8>> {
        let mut state = self.state.lock().ok()?;
        state
            .reliable
            .pop_front()
            .or_else(|| state.latest_snapshot.take())
    }

    fn blocking_next(&self) -> Option<Vec<u8>> {
        let mut state = self.state.lock().ok()?;
        loop {
            if let Some(frame) = state
                .reliable
                .pop_front()
                .or_else(|| state.latest_snapshot.take())
            {
                return Some(frame);
            }
            if state.closed {
                return None;
            }
            state = match self.blocking_wake.wait(state) {
                Ok(guard) => guard,
                // A poisoned waiter cannot proceed; ending the writer
                // thread is strictly better than panicking the server.
                Err(_) => return None,
            };
        }
    }

    async fn next(&self) -> Option<Vec<u8>> {
        loop {
            let notified = self.async_wake.notified();
            if let Some(frame) = self.try_next() {
                return Some(frame);
            }
            if self.state.lock().ok()?.closed {
                return None;
            }
            notified.await;
        }
    }

    fn close(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.closed = true;
        }
        self.blocking_wake.notify_all();
        self.async_wake.notify_waiters();
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

fn graph_errors(errors: Vec<thessa_autopilot::GraphError>) -> String {
    errors
        .into_iter()
        .map(|error| error.to_string())
        .collect::<Vec<_>>()
        .join("; ")
}

fn enqueue_client_inputs(
    id: &str,
    frames: impl IntoIterator<Item = Vec<u8>>,
    upstream: &IngressSender,
) -> bool {
    for frame in frames {
        if let Some(input) = decode_client_input(&frame) {
            if !upstream.send_input(id, input) {
                return false;
            }
        } else if let Some(guidance) = decode_guidance_input(&frame) {
            if !upstream.send_guidance(id, guidance) {
                return false;
            }
        } else if let Some(autopilot) = decode_autopilot_input(&frame)
            && !upstream.send_autopilot(id, autopilot)
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

/// Simulation time already covered by the pacing target. The authority keeps
/// a fractional fixed-step remainder in its accumulator, so that remainder
/// must not be requested again by the driver as fresh demand.
fn pacing_demand_s(target_s: f64, advanced_s: f64, backlog_s: f64, tick_s: f64) -> f64 {
    let covered_s = advanced_s + backlog_s;
    let demand_s = (target_s - covered_s).max(0.0);
    // A fresh sub-tick request is runnable when it completes the tick that
    // is already partially queued in the authority accumulator.
    if backlog_s + demand_s + 1.0e-12 >= tick_s {
        demand_s
    } else {
        0.0
    }
}

fn pacing_work_pending(chunk_s: f64, backlog_s: f64, tick_s: f64) -> bool {
    chunk_s > 0.0 || backlog_s + 1.0e-12 >= tick_s
}

/// Clip a requested chunk to the first fixed-step boundary at or after a
/// scheduler wake. The authority can only drain events after whole physics
/// ticks, so a wake inside the next tick must still be allowed to reach that
/// tick; clipping to the raw wake-minus-backlog distance would strand a
/// fractional accumulator forever.
fn clip_pacing_chunk_to_wake(
    chunk_s: f64,
    flight_time_s: f64,
    backlog_s: f64,
    wake: SimTime,
    tick_s: f64,
) -> f64 {
    if chunk_s <= 0.0 || wake.0 <= flight_time_s {
        return chunk_s;
    }
    let ticks_until_wake = ((wake.0 - flight_time_s) / tick_s).ceil().max(1.0);
    let boundary_s = flight_time_s + ticks_until_wake * tick_s;
    chunk_s.min((boundary_s - flight_time_s - backlog_s).max(0.0))
}

const MAX_INGRESS_MESSAGES: usize = 512;
const MAX_CLIENTS: usize = 256;
const MAX_PENDING_EDGE_COMMANDS: usize = 64;
const MAX_CONTINUOUS_SEGMENTS: usize = MAX_INGRESS_MESSAGES + 1;

/// Traffic from every transport into the sim driver. Continuous input is
/// kept in one latest-value slot per client. Edge-like input, guidance,
/// autopilot and subscriptions use a bounded async-safe queue. Leaves use a
/// separate id-set so cleanup cannot be lost when the event queue is full.
enum Upstream {
    Input {
        id: String,
        input: ClientInput,
        sequence: u64,
    },
    Guidance {
        id: String,
        input: GuidanceInput,
        sequence: u64,
    },
    Autopilot {
        id: String,
        input: AutopilotInput,
        sequence: u64,
    },
    Subscribe {
        id: String,
        mailbox: Arc<OutboundMailbox>,
        sequence: u64,
    },
}

impl Upstream {
    fn sequence(&self) -> u64 {
        match self {
            Self::Input { sequence, .. }
            | Self::Guidance { sequence, .. }
            | Self::Autopilot { sequence, .. }
            | Self::Subscribe { sequence, .. } => *sequence,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConnectionState {
    Pending,
    Active,
    Closed,
}

#[derive(Clone)]
struct IngressSender {
    events: tokio::sync::mpsc::Sender<Upstream>,
    latest_inputs: Arc<Mutex<BTreeMap<String, VecDeque<(u64, ClientInput)>>>>,
    leaves: Arc<Mutex<BTreeSet<String>>>,
    connections: Arc<Mutex<BTreeMap<String, ConnectionState>>>,
    next_connection: Arc<AtomicU64>,
    next_sequence: Arc<AtomicU64>,
    reliable_fences: Arc<Mutex<BTreeMap<String, u64>>>,
    ingress_lock: Arc<Mutex<()>>,
}

struct IngressReceiver {
    events: tokio::sync::mpsc::Receiver<Upstream>,
    latest_inputs: Arc<Mutex<BTreeMap<String, VecDeque<(u64, ClientInput)>>>>,
    leaves: Arc<Mutex<BTreeSet<String>>>,
    connections: Arc<Mutex<BTreeMap<String, ConnectionState>>>,
    reliable_fences: Arc<Mutex<BTreeMap<String, u64>>>,
    ingress_lock: Arc<Mutex<()>>,
}

impl IngressSender {
    fn new() -> (Self, IngressReceiver) {
        let (events, event_receiver) = tokio::sync::mpsc::channel(MAX_INGRESS_MESSAGES);
        let latest_inputs = Arc::new(Mutex::new(BTreeMap::new()));
        let leaves = Arc::new(Mutex::new(BTreeSet::new()));
        let connections = Arc::new(Mutex::new(BTreeMap::new()));
        let next_sequence = Arc::new(AtomicU64::new(1));
        let reliable_fences = Arc::new(Mutex::new(BTreeMap::new()));
        let ingress_lock = Arc::new(Mutex::new(()));
        (
            Self {
                events,
                latest_inputs: latest_inputs.clone(),
                leaves: leaves.clone(),
                connections: connections.clone(),
                next_connection: Arc::new(AtomicU64::new(1)),
                next_sequence,
                reliable_fences: reliable_fences.clone(),
                ingress_lock: ingress_lock.clone(),
            },
            IngressReceiver {
                events: event_receiver,
                latest_inputs,
                leaves,
                connections,
                reliable_fences,
                ingress_lock,
            },
        )
    }

    /// Reserve an internal connection token before enqueueing Subscribe.
    /// This makes a queued Hello+EOF pair distinguishable from a later
    /// connection and bounds pending handshakes by the client limit.
    fn reserve_connection(&self, label: &str) -> Option<String> {
        let token = self.next_connection.fetch_add(1, Ordering::Relaxed);
        let id = format!("{label}#{token}");
        self.reserve_exact_connection(id.clone()).then_some(id)
    }

    fn reserve_exact_connection(&self, id: String) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        if connections.len() >= MAX_CLIENTS || connections.contains_key(&id) {
            return false;
        }
        connections.insert(id, ConnectionState::Pending);
        true
    }

    fn can_enqueue(&self, id: &str) -> bool {
        self.connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
            .is_some_and(|state| state != ConnectionState::Closed)
    }

    fn mark_active_for_sender(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        if *state != ConnectionState::Pending {
            return false;
        }
        *state = ConnectionState::Active;
        true
    }

    fn send_input(&self, id: &str, input: ClientInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        if input.commands.iter().any(is_edge_command) {
            // Fairness: the event queue is shared by all clients. A full
            // queue means someone (possibly another client) is flooding;
            // dropping this edge with a warning keeps the victim's
            // connection alive instead of disconnecting whoever failed
            // to enqueue last. The driver drains every iteration, so a
            // transient full is recoverable.
            match self.events.try_send(Upstream::Input {
                id: id.to_string(),
                input,
                sequence,
            }) {
                Ok(()) => {
                    if let Ok(mut fences) = self.reliable_fences.lock() {
                        fences.insert(id.to_string(), sequence);
                    }
                    true
                }
                Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                    eprintln!("[server] ingress full; dropping edge input from {id}");
                    true
                }
                Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
            }
        } else {
            let Ok(mut latest) = self.latest_inputs.lock() else {
                return false;
            };
            if latest.len() >= MAX_CLIENTS && !latest.contains_key(id) {
                return false;
            }
            let segments = latest.entry(id.to_string()).or_default();
            let fence = self
                .reliable_fences
                .lock()
                .ok()
                .and_then(|fences| fences.get(id).copied())
                .unwrap_or(0);
            if segments.back().is_some_and(|(last, _)| *last > fence) {
                segments.back_mut().expect("segment exists").1 = input;
                true
            } else if segments.len() >= MAX_CONTINUOUS_SEGMENTS {
                false
            } else {
                segments.push_back((sequence, input));
                true
            }
        }
    }

    fn send_guidance(&self, id: &str, input: GuidanceInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        match self.events.try_send(Upstream::Guidance {
            id: id.to_string(),
            input,
            sequence,
        }) {
            Ok(()) => {
                if let Ok(mut fences) = self.reliable_fences.lock() {
                    fences.insert(id.to_string(), sequence);
                }
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                eprintln!("[server] ingress full; dropping guidance from {id}");
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    fn send_autopilot(&self, id: &str, input: AutopilotInput) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        if !self.can_enqueue(id) {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        match self.events.try_send(Upstream::Autopilot {
            id: id.to_string(),
            input,
            sequence,
        }) {
            Ok(()) => {
                if let Ok(mut fences) = self.reliable_fences.lock() {
                    fences.insert(id.to_string(), sequence);
                }
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                eprintln!("[server] ingress full; dropping autopilot from {id}");
                true
            }
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => false,
        }
    }

    fn send_subscribe(&self, id: &str, mailbox: Arc<OutboundMailbox>) -> bool {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return false;
        };
        let Some(state) = self
            .connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
        else {
            return false;
        };
        if state != ConnectionState::Pending {
            return false;
        }
        let sequence = self.next_sequence.fetch_add(1, Ordering::Relaxed);
        if self
            .events
            .try_send(Upstream::Subscribe {
                id: id.to_string(),
                mailbox,
                sequence,
            })
            .is_ok()
        {
            if let Ok(mut fences) = self.reliable_fences.lock() {
                fences.insert(id.to_string(), sequence);
            }
            true
        } else {
            self.close_connection(id);
            false
        }
    }

    fn send_leave(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        *state = ConnectionState::Closed;
        drop(connections);
        let Ok(mut leaves) = self.leaves.lock() else {
            return false;
        };
        leaves.insert(id.to_string());
        true
    }

    fn close_connection(&self, id: &str) {
        if let Ok(mut connections) = self.connections.lock()
            && let Some(state) = connections.get_mut(id)
        {
            *state = ConnectionState::Closed;
        }
    }
}

impl IngressReceiver {
    fn take_leaves(&self) -> BTreeSet<String> {
        let Ok(mut leaves) = self.leaves.lock() else {
            return BTreeSet::new();
        };
        std::mem::take(&mut *leaves)
    }

    fn remove_latest_input(&self, id: &str) {
        if let Ok(mut latest) = self.latest_inputs.lock() {
            latest.remove(id);
        }
    }

    /// Atomically cut an ingress batch. Producers cannot allocate a sequence
    /// number and publish only half of a message while this boundary is held.
    /// Continuous segments after the last event in this slice stay queued for
    /// the next slice, so a later state packet cannot leap over an older
    /// reliable edge that was left behind by the per-iteration cap.
    fn take_batch(
        &mut self,
        max_events: usize,
    ) -> (Vec<Upstream>, Vec<(String, u64, ClientInput)>, bool) {
        let Ok(_ingress) = self.ingress_lock.lock() else {
            return (Vec::new(), Vec::new(), false);
        };
        let mut events = Vec::with_capacity(max_events);
        while events.len() < max_events {
            match self.events.try_recv() {
                Ok(message) => events.push(message),
                Err(tokio::sync::mpsc::error::TryRecvError::Empty)
                | Err(tokio::sync::mpsc::error::TryRecvError::Disconnected) => break,
            }
        }
        // If the bounded event slice exhausted the queue, all currently
        // published continuous segments are safe to take. Otherwise the
        // remaining event(s) form an older fence for any later state.
        let watermark = (!self.events.is_empty())
            .then(|| events.iter().map(Upstream::sequence).max())
            .flatten();
        let mut continuous = Vec::new();
        if let Ok(mut latest) = self.latest_inputs.lock() {
            for (id, segments) in latest.iter_mut() {
                while let Some((sequence, _)) = segments.front()
                    && watermark.is_none_or(|limit| *sequence <= limit)
                {
                    let (sequence, input) = segments.pop_front().expect("segment exists");
                    continuous.push((id.clone(), sequence, input));
                }
            }
            latest.retain(|_, segments| !segments.is_empty());
        }
        let saturated = events.len() == max_events;
        (events, continuous, saturated)
    }

    fn is_connection_live(&self, id: &str) -> bool {
        self.connections
            .lock()
            .ok()
            .and_then(|connections| connections.get(id).copied())
            .is_some_and(|state| state != ConnectionState::Closed)
    }

    fn mark_active(&self, id: &str) -> bool {
        let Ok(mut connections) = self.connections.lock() else {
            return false;
        };
        let Some(state) = connections.get_mut(id) else {
            return false;
        };
        if *state != ConnectionState::Pending {
            return false;
        }
        *state = ConnectionState::Active;
        true
    }

    fn close_connection(&self, id: &str) {
        if let Ok(mut connections) = self.connections.lock()
            && let Some(state) = connections.get_mut(id)
        {
            *state = ConnectionState::Closed;
        }
    }

    fn finish_closed(&self, ids: &BTreeSet<String>) {
        if let Ok(mut connections) = self.connections.lock() {
            for id in ids {
                if connections.get(id) == Some(&ConnectionState::Closed) {
                    connections.remove(id);
                }
            }
        }
        if let Ok(mut fences) = self.reliable_fences.lock() {
            for id in ids {
                fences.remove(id);
            }
        }
    }
}

fn is_edge_command(command: &Command) -> bool {
    matches!(
        command,
        Command::Stage | Command::Engine { .. } | Command::Reset | Command::ExecuteManeuver { .. }
    )
}

/// Mailbox for one client during a driver quantum. Continuous controls are
/// last-value-wins; event-like commands remain ordered. Warp and pause are
/// state votes, so an older vote can be replaced without losing a stage or
/// toggle event behind it.
#[derive(Default)]
struct PendingInput {
    latest: Option<ClientInput>,
    commands: Vec<Command>,
    /// `engine_active` echo of the packet that contributed the newest edge
    /// command, with its sequence. Compared against the newest echo to tell
    /// a fresh user toggle apart from a stale pre-edge echo (see `take`).
    edge_echo: Option<bool>,
    edge_seq: u64,
    latest_seq: u64,
}

impl PendingInput {
    fn push(&mut self, input: ClientInput, seq: u64) -> bool {
        let mut input = input;
        let commands = std::mem::take(&mut input.commands);
        let mut merged = std::mem::take(&mut self.commands);
        let mut edge_seen = false;
        for command in commands {
            match command {
                Command::SetWarp { .. } => {
                    merged.retain(|queued| !matches!(queued, Command::SetWarp { .. }));
                    merged.push(command);
                }
                Command::Pause { .. } => {
                    merged.retain(|queued| !matches!(queued, Command::Pause { .. }));
                    merged.push(command);
                }
                // Stage and explicit engine commands are edge/event-like:
                // every one must reach the authoritative state in order.
                event => {
                    edge_seen |= is_edge_command(&event);
                    merged.push(event);
                }
            }
        }
        if merged
            .iter()
            .filter(|command| is_edge_command(command))
            .count()
            > MAX_PENDING_EDGE_COMMANDS
        {
            self.commands = merged;
            return false;
        }
        // Pushes arrive in sequence order (the driver sorts the slice), so
        // the newest edge packet's echo is simply the last one observed.
        if edge_seen {
            self.edge_echo = Some(input.engine_active);
            self.edge_seq = seq;
        }
        self.latest = Some(input);
        self.latest_seq = seq;
        self.commands = merged;
        true
    }

    fn take(&mut self) -> Option<ClientInput> {
        let mut input = self.latest.take()?;
        let mut commands = std::mem::take(&mut self.commands);
        // `apply_input` ignores the merged state's `engine_active` whenever
        // an edge is present, so a newer user toggle would be lost. But a
        // same-valued echo is stale/pre-edge and must NOT override the edge
        // result (Stage is a toggle: re-applying the old echo would undo it).
        // Append an explicit trailing edge only when the newest echo is both
        // strictly newer than the newest edge and different from the edge
        // packet's echo — proof the user toggled after the edge.
        if self.latest_seq > self.edge_seq
            && commands.iter().any(is_edge_command)
            && Some(input.engine_active) != self.edge_echo
        {
            commands.push(Command::Engine {
                active: input.engine_active,
            });
        }
        input.commands = commands;
        self.edge_echo = None;
        self.edge_seq = 0;
        self.latest_seq = 0;
        Some(input)
    }
}

/// Shared sim driver: one authoritative tick order for every transport.
/// stdio and TCP only differ in how frames arrive and where snapshots go.
struct Driver {
    sim: Sim,
    autopilot: AutopilotHost,
    upstream: IngressReceiver,
    subscribers: Vec<(String, Arc<OutboundMailbox>)>,
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
    fn apply_client_input(&mut self, id: &str, input: &ClientInput) -> bool {
        let takeover = self.sim.is_pilot(id)
            && input.validate().is_ok()
            && client_input_takes_over(self.sim.last_client_inputs.get(id), input);
        let force_snapshot = self.sim.apply_input(id, input);
        if takeover {
            self.autopilot.scheduler.cancel_all();
        }
        force_snapshot
    }

    fn broadcast_snapshot(&mut self) {
        let snapshot = self.sim.snapshot();
        let frame = match thessa_flight_net::encode_snapshot(&snapshot) {
            Ok(frame) => frame,
            Err(error) => {
                eprintln!("[server] snapshot encode error: {error}");
                return;
            }
        };
        self.subscribers.retain(|(_, mailbox)| {
            let accepted = mailbox.replace_snapshot(frame.clone());
            if !accepted {
                mailbox.close();
            }
            accepted
        });
    }

    /// Drain a bounded ingress slice. The limit only prevents a hot producer
    /// from monopolising the sim thread; per-client continuous input is still
    /// coalesced and event commands are retained in order.
    fn drain_upstream(&mut self) -> (bool, bool) {
        let mut pending = BTreeMap::<String, PendingInput>::new();
        let mut force_snapshot = false;
        let departed = self.upstream.take_leaves();
        let mut finish_connections = departed.clone();
        for id in &departed {
            self.release_client(id);
            force_snapshot = true;
        }
        let (events, continuous, saturated) = self
            .upstream
            .take_batch(MAX_UPSTREAM_MESSAGES_PER_ITERATION);
        let mut messages = Vec::with_capacity(events.len() + continuous.len());
        messages.extend(events.into_iter().map(|message| {
            let sequence = message.sequence();
            (sequence, message)
        }));
        messages.extend(continuous.into_iter().map(|(id, sequence, input)| {
            (
                sequence,
                Upstream::Input {
                    id,
                    input,
                    sequence,
                },
            )
        }));
        // The latest-value map and the ordered event queue are two storage
        // paths, not two independent phases. Sequence metadata restores the
        // original per-connection order before any control transition is
        // applied. A guidance/autopilot transition flushes that client's
        // pending manual input first, so an older packet cannot cancel a new
        // script after it starts.
        messages.sort_unstable_by_key(|(sequence, _)| *sequence);
        for (sequence, message) in messages {
            match message {
                Upstream::Input { id, input, .. } => {
                    if departed.contains(&id) || !self.upstream.is_connection_live(&id) {
                        continue;
                    }
                    if !pending.entry(id.clone()).or_default().push(input, sequence) {
                        eprintln!("[server] input edge queue overflow for {id}; disconnecting");
                        finish_connections.insert(id.clone());
                        self.release_client(&id);
                        force_snapshot = true;
                    }
                }
                Upstream::Guidance { id, input, .. } => {
                    if departed.contains(&id) || !self.upstream.is_connection_live(&id) {
                        continue;
                    }
                    self.flush_pending_input(&id, &mut pending, &mut force_snapshot);
                    if self.sim.is_pilot(&id) && input.validate().is_ok() {
                        self.autopilot.scheduler.cancel_all();
                    }
                    force_snapshot |= self.sim.apply_guidance(&id, &input);
                }
                Upstream::Autopilot { id, input, .. } => {
                    if departed.contains(&id) || !self.upstream.is_connection_live(&id) {
                        continue;
                    }
                    self.flush_pending_input(&id, &mut pending, &mut force_snapshot);
                    force_snapshot |= self.sim.apply_autopilot(&id, &input, &mut self.autopilot);
                }
                Upstream::Subscribe { id, mailbox, .. } => {
                    if departed.contains(&id) || !self.upstream.is_connection_live(&id) {
                        // A Hello can be followed by EOF before this bounded
                        // queue is drained. The persistent token guard keeps
                        // that stale Subscribe from resurrecting a voter.
                        mailbox.close();
                        finish_connections.insert(id);
                        continue;
                    }
                    if self.sim.clients.len() >= MAX_CLIENTS && !self.sim.clients.contains_key(&id)
                    {
                        eprintln!("[server] client limit reached; rejecting {id}");
                        mailbox.close();
                        self.upstream.close_connection(&id);
                        finish_connections.insert(id);
                        continue;
                    }
                    self.sim.register(&id);
                    let welcome = thessa_flight_net::Welcome {
                        tick: self.sim.authority.world_tick.0,
                        flight_time_s: self.sim.authority.flight_time_s,
                    };
                    match thessa_flight_net::encode_welcome(&welcome) {
                        Ok(frame) => {
                            if mailbox.push_reliable(frame) {
                                if self.upstream.mark_active(&id) {
                                    self.subscribers.push((id, mailbox));
                                } else {
                                    mailbox.close();
                                    self.release_client(&id);
                                    finish_connections.insert(id);
                                }
                            } else {
                                self.release_client(&id);
                                finish_connections.insert(id);
                            }
                        }
                        Err(error) => {
                            eprintln!("[server] welcome encode error: {error}");
                            self.release_client(&id);
                            finish_connections.insert(id);
                        }
                    }
                    force_snapshot = true;
                }
            }
        }
        for (id, mut queued) in pending {
            if departed.contains(&id) || !self.sim.clients.contains_key(&id) {
                continue;
            }
            if let Some(input) = queued.take() {
                force_snapshot |= self.apply_client_input(&id, &input);
            }
        }
        self.upstream.finish_closed(&finish_connections);
        (force_snapshot, saturated)
    }

    fn flush_pending_input(
        &mut self,
        id: &str,
        pending: &mut BTreeMap<String, PendingInput>,
        force_snapshot: &mut bool,
    ) {
        if let Some(mut queued) = pending.remove(id)
            && let Some(input) = queued.take()
        {
            *force_snapshot |= self.apply_client_input(id, &input);
        }
    }

    fn release_client(&mut self, id: &str) {
        let was_pilot = self.sim.is_pilot(id);
        if was_pilot {
            self.autopilot.scheduler.cancel_all();
        }
        self.upstream.close_connection(id);
        self.upstream.remove_latest_input(id);
        self.sim.unregister(id);
        self.subscribers.retain(|(other, mailbox)| {
            if other == id {
                mailbox.close();
                false
            } else {
                true
            }
        });
    }

    /// One iteration; `Ok(true)` asks for orderly shutdown (empty room in
    /// exit mode). Inputs are coalesced before the pacing target is sampled.
    fn iterate(&mut self) -> Result<bool, String> {
        let warp_before_inputs = self.sim.requested_warp();
        let (mut force_snapshot, ingress_saturated) = self.drain_upstream();
        let autopilot_events = self.sim.take_autopilot_events();
        match self
            .sim
            .wake_autopilot(&mut self.autopilot, &autopilot_events)
        {
            Ok(changed) => force_snapshot |= changed,
            Err(error) => {
                self.autopilot.scheduler.cancel_all();
                force_snapshot |= self.sim.fail_autopilot(error);
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
            let backlog_s = self.sim.authority.backlog_s();
            let mut chunk =
                pacing_demand_s(self.pacing_target_s, self.sim.advanced_s, backlog_s, tick_s);
            if let Some(wake) = self.sim.next_autopilot_wake(&self.autopilot) {
                chunk = clip_pacing_chunk_to_wake(
                    chunk,
                    self.sim.authority.flight_time_s,
                    backlog_s,
                    wake,
                    tick_s,
                );
            }
            // A whole tick can already be queued even when the fresh pacing
            // demand is zero (for example after a clipped scheduler wake).
            // Pass zero elapsed time through to the authority so it services
            // that physical backlog instead of skipping the call forever.
            let pacing_work_pending = pacing_work_pending(chunk, backlog_s, tick_s);
            let advanced_before = self.sim.advanced_s;
            let mut advanced_delta = 0.0;
            let mut budget_exhausted = false;
            if pacing_work_pending
                && let Err(error) = self
                    .sim
                    .advance_chunk_with_budget(chunk, Some(SIM_WORK_BUDGET))
            {
                // Latched in the authority (engine cut + flight_error);
                // the loop survives so peers see the stop, not a hang.
                eprintln!("[server] advance failed: {error}");
            }
            if pacing_work_pending {
                // Sim::advanced_s is cumulative; use its delta so a pending
                // bake is distinguishable from an already-running flight.
                advanced_delta = self.sim.advanced_s - advanced_before;
                budget_exhausted = self.sim.authority.work_budget_exhausted;
            }
            let autopilot_events = self.sim.take_autopilot_events();
            match self
                .sim
                .wake_autopilot(&mut self.autopilot, &autopilot_events)
            {
                Ok(changed) => force_snapshot |= changed,
                Err(error) => {
                    self.autopilot.scheduler.cancel_all();
                    force_snapshot |= self.sim.fail_autopilot(error);
                }
            }
            let lag_s = (self.pacing_target_s
                - (self.sim.advanced_s + self.sim.authority.backlog_s()))
            .max(0.0);
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
    let wire_out = Arc::new(OutboundMailbox::new());
    let writer_out = wire_out.clone();
    let writer = std::thread::spawn(move || {
        let mut stdout = std::io::stdout().lock();
        while let Some(frame) = writer_out.blocking_next() {
            if stdout.write_all(&frame).is_err() {
                break;
            }
        }
        let _ = stdout.flush();
    });

    let (upstream_tx, upstream_rx) = IngressSender::new();
    let local_id = upstream_tx
        .reserve_connection("local")
        .ok_or("could not reserve embedded connection")?;
    if !upstream_tx.mark_active_for_sender(&local_id) {
        return Err("could not activate embedded connection".into());
    }

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
    let welcome_frame = thessa_flight_net::encode_welcome(&welcome).map_err(|e| e.to_string())?;
    if !wire_out.push_reliable(welcome_frame) {
        return Err("stdout writer unavailable during handshake".into());
    }
    drop(locked);

    // Input pump thread: stdin bytes -> decoded inputs. EOF becomes a
    // Leave, which ends the driver loop after the queued inputs drain
    // (channel FIFO, no drain race). Own lock: the handshake lock above
    // is dropped, so no buffered byte is stranded between the two.
    let input_id = local_id.clone();
    std::thread::spawn(move || {
        let stdin = std::io::stdin();
        let mut locked = stdin.lock();
        // Keep the decoder that performed the handshake: it may contain the
        // partial prefix/body of the first pipelined input frame.
        let mut decoder = decoder;
        let mut buffer = [0u8; 65536];
        if !enqueue_client_inputs(&input_id, post_handshake_frames, &upstream_tx) {
            let _ = upstream_tx.send_leave(&input_id);
            return;
        }
        loop {
            match locked.read(&mut buffer) {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        if !enqueue_client_inputs(&input_id, frames, &upstream_tx) {
                            let _ = upstream_tx.send_leave(&input_id);
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
        let _ = upstream_tx.send_leave(&input_id);
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
    driver.sim.register(&local_id);
    driver.sim.wall_started = Instant::now();
    driver.subscribers.push((local_id, wire_out));
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
async fn handle_tcp_conn(stream: tokio::net::TcpStream, peer: String, upstream: IngressSender) {
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
    let label = format!("{}@{peer}", hello_msg.client_name);
    let Some(id) = upstream.reserve_connection(&label) else {
        eprintln!("[server] client limit reached; rejecting {label}");
        return;
    };
    eprintln!("[server] tcp hello from {label} as {id}");
    let mailbox = Arc::new(OutboundMailbox::new());
    if !upstream.send_subscribe(&id, mailbox.clone()) {
        let _ = upstream.send_leave(&id);
        return;
    }
    // Writer task: subscribed frames -> socket. Ends when the driver
    // prunes us (Leave processed) or the socket breaks.
    let writer_mailbox = mailbox.clone();
    let mut write_task = tokio::spawn(async move {
        while let Some(frame) = writer_mailbox.next().await {
            if writer.write_all(&frame).await.is_err() {
                break;
            }
        }
    });
    if !enqueue_client_inputs(&id, post_handshake_frames, &upstream) {
        let _ = upstream.send_leave(&id);
        write_task.abort();
        return;
    }
    // Reader loop: socket -> inputs. Any end votes Leave. The select also
    // aborts this reader when the outbound writer detects a dead peer, so a
    // closed mailbox cannot leave a zombie connection feeding ingress.
    let reader_id = id.clone();
    let reader_upstream = upstream.clone();
    let mut read_task = tokio::spawn(async move {
        loop {
            match reader.read(&mut buffer).await {
                Ok(0) => break,
                Ok(n) => match decoder.push(&buffer[..n]) {
                    Ok(frames) => {
                        if !enqueue_client_inputs(&reader_id, frames, &reader_upstream) {
                            break;
                        }
                    }
                    Err(error) => {
                        eprintln!("[server] tcp frame error from {reader_id}: {error}");
                        break;
                    }
                },
                Err(_) => break,
            }
        }
    });
    tokio::select! {
        _ = &mut read_task => {
            let _ = upstream.send_leave(&id);
            write_task.abort();
        }
        _ = &mut write_task => {
            let _ = upstream.send_leave(&id);
            read_task.abort();
        }
    }
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
        let (upstream_tx, upstream_rx) = IngressSender::new();
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

    fn test_driver() -> (Driver, IngressSender) {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let (upstream, receiver) = IngressSender::new();
        assert!(upstream.reserve_exact_connection("pilot".into()));
        assert!(upstream.mark_active_for_sender("pilot"));
        (
            Driver {
                sim,
                autopilot: AutopilotHost::new().expect("autopilot host"),
                upstream: receiver,
                subscribers: Vec::new(),
                pacing_target_s: 0.0,
                last_pacing: Instant::now(),
                last_snapshot: Instant::now(),
                last_status: Instant::now(),
                exit_when_empty: false,
            },
            upstream,
        )
    }

    fn timed_wait_graph(seconds: f64) -> AutopilotGraph {
        AutopilotGraph {
            nodes: vec![GraphNode {
                id: thessa_autopilot::NodeId(1),
                name: "timed-wait".into(),
                kind: NodeKind::Wait,
                ports: Vec::new(),
                config: Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::At(SimTime(seconds)),
                }),
            }],
            edges: Vec::new(),
        }
    }

    fn event_timed_wait_graph(seconds: f64, event: &str) -> AutopilotGraph {
        AutopilotGraph {
            nodes: vec![GraphNode {
                id: thessa_autopilot::NodeId(1),
                name: "event-timed-wait".into(),
                kind: NodeKind::Wait,
                ports: Vec::new(),
                config: Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::All(vec![
                        WaitCondition::At(SimTime(seconds)),
                        WaitCondition::Event(event.into()),
                    ]),
                }),
            }],
            edges: Vec::new(),
        }
    }

    #[test]
    fn maneuver_execution_flies_plan_to_completion() {
        use thessa_maneuver::ManeuverNode;

        // Control-subtracted Dv measured AT CUTOFF: the same flight with
        // and without execution, sampled at the same sim time, so orbital
        // dynamics cancel and the burn remains. Raw inertial Dvx is
        // meaningless (orbital rotation adds hundreds of m/s per minute);
        // engine spool-down tail after handoff is vehicle physics,
        // explicitly out of executor scope.
        fn fly(with_execution: bool, until_s: Option<f64>) -> (f64, bool, f64, bool, f64) {
            let config: SystemConfig =
                toml::from_str(include_str!("../../../data/system.toml")).expect("system");
            let ephemeris = config.bake().expect("bake");
            let reference_body = ephemeris.body_id("thessa").expect("thessa");
            // Drift mode (declared vacuum): no aerodynamic forces at all,
            // so attitude differences between the runs cannot leak drag
            // into the Δv metric. The engine auto-arms on first throttle
            // via the guidance path; cold rails bakes stall chunks until
            // the worker serves them (bounded retries below).
            let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
            sim.register("pilot");
            // One 100 m/s node along +X inertial at t=10 s (settle window
            // covers the initial slew). Chunk size matters: the executor
            // polls once per chunk, so chunks must resolve the burn (tens
            // of ms here) — production driver quanta (20 ms) do; 1 s chunks
            // would overshoot any small burn by a full chunk of thrust.
            let plan = ManeuverPlan::new(
                vec![ManeuverNode::new(SimTime(40.0), glam::DVec3::new(100.0, 0.0, 0.0)).unwrap()],
                sim.authority.state.position_inertial_m,
                sim.authority.state.velocity_inertial_mps,
                SimTime(0.0),
            )
            .unwrap();
            if with_execution {
                sim.start_maneuver_execution(plan).expect("starts");
                assert!(!sim.authority.scheduler.is_empty());
            }
            let initial_vx = sim.authority.state.velocity_inertial_mps.x;
            let mut saw_burn = false;
            let mut max_thrust = 0.0_f64;
            // 70 sim-seconds in 0.05 s chunks: settle, burn, cutoff, latch.
            // Stalled chunks (cold rails bake) retry briefly instead of
            // silently contributing zero time.
            for _ in 0..1400 {
                let mut advanced = 0.0;
                for _ in 0..200 {
                    advanced += sim.advance_chunk(0.05).expect("advance");
                    if advanced > 0.0
                        || sim.authority.flight_error.is_some()
                        || !sim.authority.bake.has_pending()
                    {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
                let t = sim.authority.flight_time_s;
                if let Some((_, propulsion)) = &sim.guidance {
                    if propulsion.normalized > 0.5 {
                        saw_burn = true;
                    }
                }
                max_thrust = max_thrust.max(sim.authority.thrust_n());
                if with_execution && sim.maneuver_execution.is_none() {
                    break;
                }
                if let Some(until) = until_s {
                    if t >= until {
                        break;
                    }
                }
            }
            (
                sim.authority.state.velocity_inertial_mps.x - initial_vx,
                saw_burn,
                max_thrust,
                with_execution && sim.maneuver_execution.is_none(),
                sim.authority.flight_time_s,
            )
        }
        // Exec run first (finds the cutoff time), then the control run
        // sampled at the same sim time.
        let (exec_dvx, saw_burn, max_thrust, done, t_done) = fly(true, None);
        assert!(done, "execution must complete and hand off");
        assert!(
            saw_burn,
            "executor must command full throttle during the burn"
        );
        assert!(max_thrust > 0.0, "engine must produce thrust");
        let (ctrl_dvx, _, _, _, _) = fly(false, Some(t_done));
        let burn_dvx = exec_dvx - ctrl_dvx;
        assert!(
            (85.0..=130.0).contains(&burn_dvx),
            "burn-attributed dvx {burn_dvx} for a 100 m/s node"
        );
    }

    #[test]
    fn maneuver_execution_rejects_bad_plans() {
        use thessa_maneuver::ManeuverNode;

        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
        sim.register("pilot");
        let empty =
            ManeuverPlan::new(vec![], glam::DVec3::ZERO, glam::DVec3::X, SimTime(0.0)).unwrap();
        assert!(sim.start_maneuver_execution(empty).is_err());
        let stale = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(5.0), glam::DVec3::X).unwrap()],
            glam::DVec3::ZERO,
            glam::DVec3::X,
            SimTime(0.0),
        )
        .unwrap();
        for _ in 0..10 {
            sim.advance_chunk(1.0).expect("advance");
        }
        assert!(sim.start_maneuver_execution(stale).is_err());
        assert!(sim.maneuver_execution.is_none());
    }

    #[test]
    fn execute_maneuver_command_starts_and_rejects() {
        use thessa_flight_net::ManeuverNodeCommand;

        fn fresh_sim() -> Sim {
            let config: SystemConfig =
                toml::from_str(include_str!("../../../data/system.toml")).expect("system");
            let ephemeris = config.bake().expect("bake");
            let reference_body = ephemeris.body_id("thessa").expect("thessa");
            let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
            sim.register("pilot");
            sim
        }
        // Valid command starts execution and arms the wake.
        let mut sim = fresh_sim();
        let command = Command::ExecuteManeuver {
            nodes: vec![ManeuverNodeCommand {
                epoch_s: 30.0,
                delta_v_mps: [10.0, 0.0, 0.0],
            }],
        };
        assert!(sim.apply_input("pilot", &input(vec![command])));
        assert!(sim.maneuver_execution.is_some());
        assert!(!sim.authority.scheduler.is_empty());
        // Oversize is refused with a wake notice, nothing starts.
        // (apply_input returns its snapshot flag, not acceptance.)
        let mut sim = fresh_sim();
        let big = Command::ExecuteManeuver {
            nodes: vec![
                ManeuverNodeCommand {
                    epoch_s: 30.0,
                    delta_v_mps: [1.0, 0.0, 0.0],
                };
                17
            ],
        };
        let _ = sim.apply_input("pilot", &input(vec![big]));
        assert!(sim.maneuver_execution.is_none());
        assert!(
            sim.authority
                .wake_notice
                .as_ref()
                .is_some_and(|notice| notice.contains("cap"))
        );
        // Non-finite node is refused the same way.
        let mut sim = fresh_sim();
        let bad = Command::ExecuteManeuver {
            nodes: vec![ManeuverNodeCommand {
                epoch_s: f64::NAN,
                delta_v_mps: [1.0, 0.0, 0.0],
            }],
        };
        let _ = sim.apply_input("pilot", &input(vec![bad]));
        assert!(sim.maneuver_execution.is_none());
        assert!(sim.authority.wake_notice.is_some());
    }

    #[test]
    fn burn_execution_flies_plan_to_completion() {
        use thessa_maneuver::{BurnSegment, EngineSpec, FiniteBurnPlan, SegmentDirection};

        // Same control-subtracted harness as the node test: exec run finds
        // the cutoff time, the control run sampled there cancels orbital
        // dynamics, the burn remains.
        fn fly(with_execution: bool, until_s: Option<f64>) -> (f64, bool, f64, bool, f64) {
            let config: SystemConfig =
                toml::from_str(include_str!("../../../data/system.toml")).expect("system");
            let ephemeris = config.bake().expect("bake");
            let reference_body = ephemeris.body_id("thessa").expect("thessa");
            let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
            sim.register("pilot");
            // Two 5 s full-throttle +X segments at t=40/60 s.
            let engine = EngineSpec {
                thrust_n: 100_000.0,
                exhaust_velocity_mps: 4_400.0,
            };
            let segment = |start: f64| BurnSegment {
                start: SimTime(start),
                duration_s: 5.0,
                planned_dv_mps: 50.0,
                direction: SegmentDirection::Inertial(glam::DVec3::X),
                throttle_01: 1.0,
            };
            let plan = FiniteBurnPlan::new(
                vec![segment(40.0), segment(60.0)],
                engine,
                20_000.0,
                sim.authority.state.position_inertial_m,
                sim.authority.state.velocity_inertial_mps,
                SimTime(0.0),
            )
            .unwrap();
            if with_execution {
                sim.start_burn_execution(plan).expect("starts");
                assert!(!sim.authority.scheduler.is_empty());
            }
            let initial_vx = sim.authority.state.velocity_inertial_mps.x;
            let mut saw_burn = false;
            let mut max_thrust = 0.0_f64;
            for _ in 0..1400 {
                let mut advanced = 0.0;
                for _ in 0..200 {
                    advanced += sim.advance_chunk(0.05).expect("advance");
                    if advanced > 0.0
                        || sim.authority.flight_error.is_some()
                        || !sim.authority.bake.has_pending()
                    {
                        break;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(5));
                }
                if let Some((_, propulsion)) = &sim.guidance {
                    if propulsion.normalized > 0.5 {
                        saw_burn = true;
                    }
                }
                max_thrust = max_thrust.max(sim.authority.thrust_n());
                if with_execution && sim.burn_execution.is_none() {
                    break;
                }
                if let Some(until) = until_s {
                    if sim.authority.flight_time_s >= until {
                        break;
                    }
                }
            }
            (
                sim.authority.state.velocity_inertial_mps.x - initial_vx,
                saw_burn,
                max_thrust,
                with_execution && sim.burn_execution.is_none(),
                sim.authority.flight_time_s,
            )
        }
        let (exec_dvx, saw_burn, max_thrust, done, t_done) = fly(true, None);
        assert!(done, "burn execution must complete and hand off");
        assert!(saw_burn, "executor must command throttle during arcs");
        assert!(max_thrust > 0.0, "engine must produce thrust");
        let (ctrl_dvx, _, _, _, _) = fly(false, Some(t_done));
        let burn_dvx = exec_dvx - ctrl_dvx;
        assert!(
            burn_dvx > 0.5,
            "burn-attributed dvx {burn_dvx} for two 5 s arcs"
        );
    }

    #[test]
    fn burn_execution_rejects_bad_plans() {
        use thessa_maneuver::ManeuverNode;
        use thessa_maneuver::{BurnSegment, EngineSpec, FiniteBurnPlan, SegmentDirection};

        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
        sim.register("pilot");
        let engine = EngineSpec {
            thrust_n: 100_000.0,
            exhaust_velocity_mps: 4_400.0,
        };
        let segment = BurnSegment {
            start: SimTime(40.0),
            duration_s: 5.0,
            planned_dv_mps: 50.0,
            direction: SegmentDirection::Inertial(glam::DVec3::X),
            throttle_01: 1.0,
        };
        let empty = FiniteBurnPlan::new(
            vec![],
            engine,
            20_000.0,
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(0.0),
        )
        .unwrap();
        assert!(sim.start_burn_execution(empty).is_err());
        let stale = FiniteBurnPlan::new(
            vec![BurnSegment {
                start: SimTime(5.0),
                ..segment
            }],
            engine,
            20_000.0,
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(10.0),
        )
        .unwrap();
        // Flight clock starts at 0: a plan starting at t=5 is... fresh
        // here; force staleness by advancing the clock past the segment.
        sim.authority.flight_time_s = 50.0;
        assert!(sim.start_burn_execution(stale).is_err());
        assert!(sim.burn_execution.is_none());
        // Mutual exclusion with node execution (both directions).
        sim.authority.flight_time_s = 0.0;
        let live = FiniteBurnPlan::new(
            vec![segment],
            engine,
            20_000.0,
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(0.0),
        )
        .unwrap();
        sim.start_burn_execution(live).expect("burn starts");
        let node_plan = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(40.0), glam::DVec3::X).unwrap()],
            sim.authority.state.position_inertial_m,
            sim.authority.state.velocity_inertial_mps,
            SimTime(0.0),
        )
        .unwrap();
        assert!(sim.start_maneuver_execution(node_plan).is_err());
        sim.clear_autopilot_controls();
        assert!(sim.burn_execution.is_none());
    }

    #[test]
    fn execute_burn_plan_command_starts_and_rejects() {
        use thessa_flight_net::{BurnDirectionCommand, BurnSegmentCommand};

        fn fresh_sim() -> Sim {
            let config: SystemConfig =
                toml::from_str(include_str!("../../../data/system.toml")).expect("system");
            let ephemeris = config.bake().expect("bake");
            let reference_body = ephemeris.body_id("thessa").expect("thessa");
            let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
            sim.register("pilot");
            sim
        }
        fn segment(start_s: f64) -> BurnSegmentCommand {
            BurnSegmentCommand {
                start_s,
                duration_s: 5.0,
                planned_dv_mps: 50.0,
                direction: BurnDirectionCommand::Inertial {
                    unit: [1.0, 0.0, 0.0],
                },
                throttle_01: 1.0,
            }
        }
        // Valid command (inertial + RTN segments) starts execution.
        let mut sim = fresh_sim();
        let command = Command::ExecuteBurnPlan {
            engine_thrust_n: 100_000.0,
            engine_exhaust_velocity_mps: 4_400.0,
            initial_mass_kg: 20_000.0,
            segments: vec![
                segment(30.0),
                BurnSegmentCommand {
                    direction: BurnDirectionCommand::Rtn {
                        central: "thessa".into(),
                        radial: 0.0,
                        transverse: 1.0,
                        normal: 0.0,
                    },
                    ..segment(60.0)
                },
            ],
        };
        assert!(sim.apply_input("pilot", &input(vec![command])));
        assert!(sim.burn_execution.is_some());
        assert!(!sim.authority.scheduler.is_empty());
        // Oversize is refused with a wake notice, nothing starts.
        let mut sim = fresh_sim();
        let big = Command::ExecuteBurnPlan {
            engine_thrust_n: 100_000.0,
            engine_exhaust_velocity_mps: 4_400.0,
            initial_mass_kg: 20_000.0,
            segments: vec![segment(30.0); 65],
        };
        let _ = sim.apply_input("pilot", &input(vec![big]));
        assert!(sim.burn_execution.is_none());
        assert!(
            sim.authority
                .wake_notice
                .as_ref()
                .is_some_and(|notice| notice.contains("cap"))
        );
        // Unknown RTN central and dead engine are refused the same way.
        let mut sim = fresh_sim();
        let lost = Command::ExecuteBurnPlan {
            engine_thrust_n: 100_000.0,
            engine_exhaust_velocity_mps: 4_400.0,
            initial_mass_kg: 20_000.0,
            segments: vec![BurnSegmentCommand {
                direction: BurnDirectionCommand::Rtn {
                    central: "nope".into(),
                    radial: 0.0,
                    transverse: 1.0,
                    normal: 0.0,
                },
                ..segment(30.0)
            }],
        };
        let _ = sim.apply_input("pilot", &input(vec![lost]));
        assert!(sim.burn_execution.is_none());
        assert!(sim.authority.wake_notice.is_some());
        let mut sim = fresh_sim();
        let dead = Command::ExecuteBurnPlan {
            engine_thrust_n: 0.0,
            engine_exhaust_velocity_mps: 4_400.0,
            initial_mass_kg: 20_000.0,
            segments: vec![segment(30.0)],
        };
        let _ = sim.apply_input("pilot", &input(vec![dead]));
        assert!(sim.burn_execution.is_none());
        assert!(sim.authority.wake_notice.is_some());
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
        assert!(sim.wake_autopilot(&mut host, &[]).expect("wake script"));
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
    fn neutral_input_and_warp_votes_do_not_cancel_a_waiting_script() {
        let (mut driver, _) = test_driver();
        driver.sim.register("pilot");
        let neutral = input(Vec::new());
        driver.apply_client_input("pilot", &neutral);
        assert!(driver.sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::StartScript {
                    source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
                },
            },
            &mut driver.autopilot,
        ));
        assert_eq!(driver.autopilot.scheduler.pending(), 1);

        driver.apply_client_input("pilot", &neutral);
        driver.apply_client_input("pilot", &input(vec![Command::SetWarp { factor: 128.0 }]));
        assert_eq!(driver.autopilot.scheduler.pending(), 1);

        driver.sim.plan_demand = Some(ControlDemand {
            force_body_n: DVec3::X * 100.0,
            moment_body_nm: DVec3::Y * 50.0,
            propulsion: PropulsionDemand::new(0.8).unwrap(),
        });
        driver.sim.guidance = Some((
            GuidanceIntent::AngularRate {
                rate_body_rps: DVec3::X,
            },
            PropulsionDemand::new(0.8).unwrap(),
        ));
        driver
            .sim
            .authority
            .set_propulsion_target(PropulsionDemand::new(0.8).unwrap())
            .unwrap();
        let mut manual = neutral;
        manual.control_input = [0.25, 0.0, 0.0];
        driver.apply_client_input("pilot", &manual);
        assert_eq!(driver.autopilot.scheduler.pending(), 0);
        assert!(driver.sim.plan_demand.is_none());
        assert!(driver.sim.guidance.is_none());
        assert_eq!(driver.sim.authority.thrust_n(), 0.0);
    }

    #[test]
    fn cancel_and_graph_submit_drop_old_script_continuations() {
        let (mut driver, _) = test_driver();
        driver.sim.register("pilot");
        let start = AutopilotInput {
            tick: 0,
            command: AutopilotCommand::StartScript {
                source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
            },
        };
        assert!(
            driver
                .sim
                .apply_autopilot("pilot", &start, &mut driver.autopilot)
        );
        assert_eq!(driver.autopilot.scheduler.pending(), 1);
        assert!(driver.sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 1,
                command: AutopilotCommand::Cancel,
            },
            &mut driver.autopilot,
        ));
        assert_eq!(driver.autopilot.scheduler.pending(), 0);
        driver.sim.authority.flight_time_s = 2.0;
        assert!(
            driver
                .autopilot
                .scheduler
                .wake(&driver.autopilot.engine, SimTime(2.0), None)
                .unwrap()
                .is_empty()
        );
        assert!(driver.sim.guidance.is_none());

        assert!(
            driver
                .sim
                .apply_autopilot("pilot", &start, &mut driver.autopilot)
        );
        assert_eq!(driver.autopilot.scheduler.pending(), 1);
        assert!(driver.sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 2,
                command: AutopilotCommand::SubmitGraph {
                    graph: timed_wait_graph(10.0),
                },
            },
            &mut driver.autopilot,
        ));
        assert_eq!(driver.autopilot.scheduler.pending(), 0);
        assert!(driver.sim.graph_runner.is_some());
        driver.sim.authority.flight_time_s = 3.0;
        assert!(
            driver
                .autopilot
                .scheduler
                .wake(&driver.autopilot.engine, SimTime(3.0), None)
                .unwrap()
                .is_empty()
        );
        assert!(driver.sim.guidance.is_none());
    }

    #[test]
    fn graph_time_wake_clips_high_warp_before_bulk_stepping() {
        let (mut driver, _) = test_driver();
        driver.sim.register("pilot");
        assert!(driver.sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::SubmitGraph {
                    graph: event_timed_wait_graph(0.05, "impact"),
                },
            },
            &mut driver.autopilot,
        ));
        assert!(driver.sim.next_autopilot_wake(&driver.autopilot).is_none());
        driver
            .sim
            .wake_autopilot(&mut driver.autopilot, &[AutopilotEvent::Impact])
            .expect("remember graph event");
        let wake = driver
            .sim
            .next_autopilot_wake(&driver.autopilot)
            .expect("graph time wake");
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        let clipped = clip_pacing_chunk_to_wake(
            100.0,
            driver.sim.authority.flight_time_s,
            driver.sim.authority.backlog_s(),
            wake,
            tick_s,
        );
        assert!(clipped < 100.0);
        assert!(driver.sim.authority.flight_time_s + clipped <= 0.05 + tick_s);
    }

    #[test]
    fn server_stores_data_only_landing_and_impact_site_declarations() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");

        assert!(sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::StartScript {
                    source: "return Landing.site(0, 2, 0, 300);".into(),
                },
            },
            &mut host,
        ));
        assert_eq!(
            sim.landing_site,
            Some(LandingSite {
                center_dir: [0.0, 1.0, 0.0],
                radius_m: 300.0,
            })
        );
        assert!(sim.landing_obstacles.is_none());
        assert!(sim.plan_runner.is_none());

        assert!(sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 1,
                command: AutopilotCommand::StartScript {
                    source: "return Impact.site(1, 0, 0, 50);".into(),
                },
            },
            &mut host,
        ));
        assert_eq!(
            sim.impact_site,
            Some(ImpactSite {
                center_dir: [1.0, 0.0, 0.0],
                radius_m: 50.0,
            })
        );
        assert!(sim.impact_obstacles.is_none());
        assert!(sim.authority.wake_notice.as_deref().is_some_and(|notice| {
            notice.contains("IMPACT SITE") && notice.contains("radius=50.0m")
        }));
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
                thessa_autopilot::TrajectorySegment::Burn {
                    duration_s: 0.1,
                    demand: ControlDemand {
                        force_body_n: DVec3::Y * 100.0,
                        moment_body_nm: DVec3::X * 100.0,
                        propulsion: PropulsionDemand::new(0.0).unwrap(),
                    },
                },
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
        sim.advance_chunk(thessa_sim_core::WORLD_TICK_S)
            .expect("execute explicit burn");
        assert!(sim.authority.flight_error.is_none());
        assert!(sim.authority.state.angular_velocity_body_rps.x > 0.0);
        assert_eq!(
            sim.authority
                .last_forces
                .as_ref()
                .expect("explicit force sample")
                .total_force_body_n
                .y,
            100.0
        );
        assert!(sim.authority.state.angular_velocity_body_rps.is_finite());

        sim.authority.flight_time_s = 0.13;
        sim.poll_plan(None).expect("advance to guidance cursor");
        assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 2);
        assert!(sim.plan_demand.is_none());
        assert!(sim.guidance.is_some());

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

        sim.autopilot_events.push_back(AutopilotEvent::Impact);
        let events = sim.take_autopilot_events();
        assert_eq!(events, vec![AutopilotEvent::Impact]);
        assert!(sim.poll_plan(Some("impact")).expect("wake plan"));
        assert_eq!(sim.plan_runner.as_ref().unwrap().segment_index(), 1);
        assert_eq!(sim.control_mode, ControlMode::Rate);
    }

    #[test]
    fn typed_authority_wakes_preserve_order_and_duplicate_events() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, false).expect("sim");
        sim.register("pilot");
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        sim.authority
            .scheduler
            .arm(ScheduledKind::Alarm, SimTime(tick_s));
        sim.authority
            .scheduler
            .arm(ScheduledKind::Alarm, SimTime(tick_s));
        sim.advance_chunk(tick_s * 2.0).expect("advance alarms");
        assert_eq!(
            sim.take_autopilot_events(),
            vec![AutopilotEvent::Alarm, AutopilotEvent::Alarm]
        );
    }

    #[test]
    fn server_validates_and_executes_graph_ir_on_submit() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");
        let graph = AutopilotGraph {
            nodes: vec![
                thessa_autopilot::GraphNode {
                    id: thessa_autopilot::NodeId(1),
                    name: "source".into(),
                    kind: thessa_autopilot::NodeKind::Source,
                    ports: vec![thessa_autopilot::Port::output(
                        "value",
                        thessa_autopilot::PortType::Number,
                    )],
                    config: None,
                },
                thessa_autopilot::GraphNode {
                    id: thessa_autopilot::NodeId(2),
                    name: "sink".into(),
                    kind: thessa_autopilot::NodeKind::Sink,
                    ports: vec![thessa_autopilot::Port::input(
                        "value",
                        thessa_autopilot::PortType::Number,
                        true,
                    )],
                    config: None,
                },
            ],
            edges: vec![thessa_autopilot::GraphEdge {
                from: thessa_autopilot::PortRef {
                    node: thessa_autopilot::NodeId(1),
                    port: "value".into(),
                },
                to: thessa_autopilot::PortRef {
                    node: thessa_autopilot::NodeId(2),
                    port: "value".into(),
                },
            }],
        };
        let input = AutopilotInput {
            tick: 0,
            command: AutopilotCommand::SubmitGraph {
                graph: graph.clone(),
            },
        };
        assert!(sim.apply_autopilot("pilot", &input, &mut host));
        assert_eq!(sim.autopilot_graph, Some(graph));
        assert!(
            sim.graph_runner
                .as_ref()
                .is_some_and(GraphRunner::is_complete)
        );
        assert!(
            sim.authority
                .wake_notice
                .as_deref()
                .is_some_and(|notice| notice.starts_with("AUTOPILOT GRAPH READY"))
        );
    }

    #[test]
    fn server_applies_configured_graph_guidance_through_authority() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");
        let graph = AutopilotGraph {
            nodes: vec![thessa_autopilot::GraphNode {
                id: thessa_autopilot::NodeId(1),
                name: "rate-controller".into(),
                kind: thessa_autopilot::NodeKind::Controller {
                    actuator_groups: vec![thessa_flight_control::ActuatorGroup::Rcs],
                },
                ports: vec![thessa_autopilot::Port::output(
                    "target",
                    thessa_autopilot::PortType::AngularRateTarget,
                )],
                config: Some(GraphNodeConfig::Guidance {
                    intent: GuidanceIntent::AngularRate {
                        rate_body_rps: DVec3::new(0.1, 0.0, 0.0),
                    },
                    propulsion: PropulsionDemand::new(0.0).unwrap(),
                }),
            }],
            edges: Vec::new(),
        };
        assert!(sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::SubmitGraph { graph },
            },
            &mut host,
        ));
        assert_eq!(sim.control_mode, ControlMode::Rate);
        assert!(matches!(
            sim.guidance,
            Some((
                GuidanceIntent::AngularRate { .. },
                PropulsionDemand { normalized: 0.0 }
            ))
        ));
        assert!(
            sim.graph_runner
                .as_ref()
                .is_some_and(GraphRunner::is_complete)
        );
    }

    #[test]
    fn server_wakes_a_native_graph_wait_from_an_authoritative_event() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let mut host = AutopilotHost::new().expect("autopilot host");
        sim.register("pilot");
        let graph = AutopilotGraph {
            nodes: vec![
                thessa_autopilot::GraphNode {
                    id: thessa_autopilot::NodeId(1),
                    name: "impact".into(),
                    kind: thessa_autopilot::NodeKind::Wait,
                    ports: vec![thessa_autopilot::Port::output(
                        "done",
                        thessa_autopilot::PortType::Unit,
                    )],
                    config: None,
                },
                thessa_autopilot::GraphNode {
                    id: thessa_autopilot::NodeId(2),
                    name: "sink".into(),
                    kind: thessa_autopilot::NodeKind::Sink,
                    ports: vec![thessa_autopilot::Port::input(
                        "done",
                        thessa_autopilot::PortType::Unit,
                        true,
                    )],
                    config: None,
                },
            ],
            edges: vec![thessa_autopilot::GraphEdge {
                from: thessa_autopilot::PortRef {
                    node: thessa_autopilot::NodeId(1),
                    port: "done".into(),
                },
                to: thessa_autopilot::PortRef {
                    node: thessa_autopilot::NodeId(2),
                    port: "done".into(),
                },
            }],
        };
        assert!(sim.apply_autopilot(
            "pilot",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::SubmitGraph { graph },
            },
            &mut host,
        ));
        assert!(!sim.graph_runner.as_ref().unwrap().is_complete());

        sim.autopilot_events.push_back(AutopilotEvent::Impact);
        let events = sim.take_autopilot_events();
        assert_eq!(events, vec![AutopilotEvent::Impact]);
        sim.wake_autopilot(&mut host, &events).expect("wake graph");
        assert!(sim.graph_runner.as_ref().unwrap().is_complete());

        sim.authority.wake_notice = Some("AUTOPILOT GRAPH WAIT node=impact".into());
        assert!(sim.take_autopilot_events().is_empty());
    }

    #[test]
    fn coalescing_keeps_latest_controls_and_all_event_commands() {
        let mut pending = PendingInput::default();
        let mut first = input(vec![Command::Stage]);
        first.control_input = [0.1, 0.0, 0.0];
        pending.push(first, 1);
        let mut second = input(vec![
            Command::Stage,
            Command::Pause { paused: true },
            Command::SetWarp { factor: 128.0 },
        ]);
        second.control_input = [0.2, 0.0, 0.0];
        pending.push(second, 2);
        let mut third = input(vec![
            Command::Pause { paused: false },
            Command::SetWarp { factor: 256.0 },
        ]);
        third.control_input = [0.3, 0.0, 0.0];
        pending.push(third, 3);

        let merged = pending.take().expect("coalesced input");
        assert_eq!(merged.control_input, [0.3, 0.0, 0.0]);
        assert_eq!(merged.commands[0], Command::Stage);
        assert_eq!(merged.commands[1], Command::Stage);
        assert_eq!(merged.commands[2], Command::Pause { paused: false });
        assert_eq!(merged.commands[3], Command::SetWarp { factor: 256.0 });
    }

    #[test]
    fn coalesced_trailing_toggle_after_an_edge_is_preserved() {
        // [Stage(seq1, echo=false), continuous(seq2, echo=true)]: the user
        // toggled after the edge. The merged batch must carry the newer
        // absolute intent as a trailing edge instead of dropping it.
        let mut pending = PendingInput::default();
        pending.push(input(vec![Command::Stage]), 1);
        let mut trailing = input(vec![]);
        trailing.engine_active = true;
        pending.push(trailing, 2);
        let merged = pending.take().expect("merged input");
        assert_eq!(
            merged.commands.last(),
            Some(&Command::Engine { active: true })
        );
    }

    #[test]
    fn coalesced_stale_echo_does_not_undo_a_toggle() {
        // [Stage(seq1, echo=false), continuous(seq2, echo=false)]: no new
        // user intent — the echo must not override the toggle result.
        let mut pending = PendingInput::default();
        pending.push(input(vec![Command::Stage]), 1);
        pending.push(input(vec![]), 2);
        let merged = pending.take().expect("merged input");
        assert_eq!(merged.commands, vec![Command::Stage]);
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
        pending.push(first, 1);
        pending.push(second, 2);
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
    fn pacing_demand_does_not_double_count_fractional_authority_backlog() {
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut authority = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
        let targets = [(0.012, 1_u64), (0.020, 2), (0.025, 3), (0.030, 3)];

        for (target_s, expected_ticks) in targets {
            let advanced_s = authority.flight_time_s;
            let backlog_s = authority.backlog_s();
            let demand_s = pacing_demand_s(target_s, advanced_s, backlog_s, tick_s);
            authority
                .advance(&ephemeris, ControlMode::Direct, demand_s)
                .expect("target advancement");

            let expected_time_s = expected_ticks as f64 * tick_s;
            assert!(
                (authority.flight_time_s - expected_time_s).abs() < 1.0e-12,
                "target={target_s} advanced={} expected={expected_time_s} backlog={}",
                authority.flight_time_s,
                authority.backlog_s()
            );
            assert!(authority.backlog_s() < tick_s);
            assert!(
                authority.flight_time_s + authority.backlog_s() <= target_s + 1.0e-12,
                "target={target_s} was overrun: covered={}",
                authority.flight_time_s + authority.backlog_s()
            );
            assert!(
                target_s - (authority.flight_time_s + authority.backlog_s()) < tick_s,
                "target={target_s} left more than one tick unserved: covered={}",
                authority.flight_time_s + authority.backlog_s()
            );
        }
    }

    #[test]
    fn zero_elapsed_advance_services_a_queued_tick_at_a_scheduler_wake() {
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let mut authority = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
        authority.accumulator_s = tick_s;
        authority
            .scheduler
            .arm(thessa_sim_core::ScheduledKind::Alarm, SimTime(0.001));

        let clipped = clip_pacing_chunk_to_wake(
            0.0,
            authority.flight_time_s,
            authority.backlog_s(),
            SimTime(0.001),
            tick_s,
        );
        assert_eq!(clipped, 0.0);
        assert!(pacing_work_pending(clipped, authority.backlog_s(), tick_s));

        authority
            .advance(&ephemeris, ControlMode::Direct, 0.0)
            .expect("service queued tick");
        assert_eq!(authority.steps_this_frame, 1);
        assert!(authority.backlog_s() < tick_s);
        assert!(
            authority
                .wake_notice
                .as_deref()
                .is_some_and(|notice| notice.contains("WAKE ALARM"))
        );
    }

    #[test]
    fn pacing_wake_clip_reaches_the_next_fixed_tick_without_overshoot() {
        let tick_s = thessa_sim_core::WORLD_TICK_S;
        let flight_time_s = tick_s;
        let backlog_s = 0.003666666666666667;
        let wake = SimTime(flight_time_s + tick_s * 0.25);
        let clipped =
            clip_pacing_chunk_to_wake(tick_s * 4.0, flight_time_s, backlog_s, wake, tick_s);

        assert!((clipped - (tick_s - backlog_s)).abs() < 1.0e-12);
        assert!(clipped < tick_s);
    }

    #[test]
    fn first_client_owns_controls_and_disconnect_transfers_in_order() {
        let (mut driver, _ingress) = test_driver();
        driver.sim.register("first");
        driver.sim.register("second");
        assert_eq!(driver.sim.pilot_owner.as_deref(), Some("first"));

        let mut manual = input(Vec::new());
        manual.control_input = [1.0, 0.0, 0.0];
        manual.throttle = 1.0;
        manual.engine_active = true;
        assert!(!driver.sim.apply_input("first", &manual));
        let state_before_spectator = driver.sim.authority.state;
        let mut spectator_command = manual.clone();
        spectator_command.commands = vec![Command::Stage, Command::Reset];
        assert!(!driver.sim.apply_input("second", &spectator_command));
        assert_eq!(driver.sim.authority.state, state_before_spectator);
        assert_eq!(driver.sim.authority.control_input, DVec3::X);

        let spectator_vote = input(vec![Command::Pause { paused: true }]);
        assert!(driver.sim.apply_input("second", &spectator_vote));
        assert!(driver.sim.paused());

        let invalid_guidance = GuidanceInput {
            tick: 0,
            intent: GuidanceIntent::AngularRate {
                rate_body_rps: DVec3::splat(f64::NAN),
            },
            propulsion: PropulsionDemand {
                normalized: f64::NAN,
            },
        };
        assert!(!driver.sim.apply_guidance("second", &invalid_guidance));
        assert!(driver.sim.authority.flight_error.is_none());
        assert!(!driver.sim.apply_autopilot(
            "second",
            &AutopilotInput {
                tick: 0,
                command: AutopilotCommand::StartScript {
                    source: String::new(),
                },
            },
            &mut driver.autopilot,
        ));

        driver.release_client("first");
        assert_eq!(driver.sim.pilot_owner.as_deref(), Some("second"));
        assert_eq!(driver.sim.authority.control_input, DVec3::ZERO);
        assert!(!driver.sim.authority.engine_active);
        assert_eq!(driver.autopilot.scheduler.pending(), 0);
    }

    #[test]
    fn invalid_input_is_rejected_before_any_state_or_takeover_mutation() {
        let (mut driver, _ingress) = test_driver();
        driver.sim.register("pilot");
        let mut invalid = input(vec![Command::Reset]);
        invalid.control_input[0] = f64::NAN;
        invalid.throttle = f64::INFINITY;
        invalid.sas_target_xyzw[3] = f64::NAN;
        let state = driver.sim.authority.state;
        let control = driver.sim.authority.control_input;
        let engine = driver.sim.authority.engine_active;
        assert!(!driver.apply_client_input("pilot", &invalid));
        assert_eq!(driver.sim.authority.state, state);
        assert_eq!(driver.sim.authority.control_input, control);
        assert_eq!(driver.sim.authority.engine_active, engine);
        assert!(driver.sim.authority.flight_error.is_none());
        assert!(driver.sim.last_client_inputs.is_empty());
    }

    #[test]
    fn outbound_mailbox_keeps_welcome_order_and_latest_snapshot_only() {
        let mailbox = OutboundMailbox::new();
        assert!(mailbox.push_reliable(vec![1]));
        assert!(mailbox.push_reliable(vec![2]));
        for value in 3..=100 {
            assert!(mailbox.replace_snapshot(vec![value]));
        }
        assert_eq!(mailbox.try_next(), Some(vec![1]));
        assert_eq!(mailbox.try_next(), Some(vec![2]));
        assert_eq!(mailbox.try_next(), Some(vec![100]));
        assert_eq!(mailbox.try_next(), None);

        for _ in 0..RELIABLE_OUTBOUND_CAPACITY {
            assert!(mailbox.push_reliable(vec![0]));
        }
        assert!(!mailbox.push_reliable(vec![0]));
    }

    #[test]
    fn ingress_coalesces_continuous_input_and_reserves_leave_cleanup() {
        let (ingress, mut receiver) = IngressSender::new();
        assert!(ingress.reserve_exact_connection("pilot".into()));
        assert!(ingress.mark_active_for_sender("pilot"));
        for tick in 0..10_000 {
            let mut value = input(Vec::new());
            value.tick = tick;
            assert!(ingress.send_input("pilot", value));
        }
        let (events, continuous, saturated) = receiver.take_batch(MAX_INGRESS_MESSAGES);
        assert!(events.is_empty());
        assert_eq!(continuous.len(), 1);
        assert!(!saturated);

        for _ in 0..MAX_INGRESS_MESSAGES {
            assert!(ingress.send_input("pilot", input(vec![Command::Stage])));
        }
        // Fairness: a full shared queue drops the edge with a warning but
        // keeps the sender's connection alive (the flood may come from a
        // different client). Only a closed connection refuses.
        assert!(ingress.send_input("pilot", input(vec![Command::Stage])));
        assert!(ingress.send_leave("pilot"));
        assert!(receiver.take_leaves().contains("pilot"));
    }

    #[test]
    fn eof_before_subscribe_does_not_resurrect_a_queued_connection() {
        let (mut driver, ingress) = test_driver();
        let id = ingress
            .reserve_connection("peer")
            .expect("connection token");
        let mailbox = Arc::new(OutboundMailbox::new());
        assert!(ingress.send_subscribe(&id, mailbox.clone()));
        assert!(ingress.send_leave(&id));

        let _ = driver.drain_upstream();
        assert!(driver.sim.clients.is_empty());
        assert!(driver.subscribers.is_empty());
        assert!(
            !ingress
                .connections
                .lock()
                .expect("connection registry")
                .contains_key(&id)
        );
        assert!(mailbox.try_next().is_none());
    }

    #[test]
    fn sequence_order_preserves_input_edges_and_script_transitions() {
        let (mut driver, ingress) = test_driver();
        driver.sim.register("pilot");

        let mut continuous = input(Vec::new());
        continuous.control_input = [0.1, 0.0, 0.0];
        continuous.throttle = 0.1;
        assert!(ingress.send_input("pilot", continuous));
        let mut edge = input(vec![Command::Stage]);
        edge.control_input = [0.8, 0.0, 0.0];
        edge.throttle = 0.8;
        assert!(ingress.send_input("pilot", edge));
        let _ = driver.drain_upstream();
        assert_eq!(
            driver.sim.authority.control_input,
            DVec3::new(0.8, 0.0, 0.0)
        );
        assert!(driver.sim.authority.engine_active);

        let (mut driver, ingress) = test_driver();
        driver.sim.register("pilot");
        let mut edge = input(vec![Command::Stage]);
        edge.control_input = [0.8, 0.0, 0.0];
        edge.throttle = 0.8;
        assert!(ingress.send_input("pilot", edge));
        let mut continuous = input(Vec::new());
        continuous.control_input = [0.1, 0.0, 0.0];
        continuous.throttle = 0.1;
        assert!(ingress.send_input("pilot", continuous));
        let _ = driver.drain_upstream();
        assert_eq!(
            driver.sim.authority.control_input,
            DVec3::new(0.1, 0.0, 0.0)
        );
        assert!(driver.sim.authority.engine_active);

        let (mut driver, ingress) = test_driver();
        driver.sim.register("pilot");
        let mut before_script = input(Vec::new());
        before_script.control_input = [0.1, 0.0, 0.0];
        assert!(ingress.send_input("pilot", before_script));
        assert!(ingress.send_autopilot(
            "pilot",
            AutopilotInput {
                tick: 0,
                command: AutopilotCommand::StartScript {
                    source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
                },
            },
        ));
        let _ = driver.drain_upstream();
        assert_eq!(driver.autopilot.scheduler.pending(), 1);

        let (mut driver, ingress) = test_driver();
        driver.sim.register("pilot");
        assert!(ingress.send_autopilot(
            "pilot",
            AutopilotInput {
                tick: 0,
                command: AutopilotCommand::StartScript {
                    source: "await sim.sleep(1); return Guidance.angularRate(0.1, 0, 0);".into(),
                },
            },
        ));
        let mut deliberate_manual = input(Vec::new());
        deliberate_manual.control_input = [0.1, 0.0, 0.0];
        assert!(ingress.send_input("pilot", deliberate_manual));
        let _ = driver.drain_upstream();
        assert_eq!(driver.autopilot.scheduler.pending(), 0);
    }

    #[test]
    fn ingress_watermark_defers_latest_state_behind_a_second_event_slice() {
        let (mut driver, ingress) = test_driver();
        driver.sim.register("pilot");
        let guidance = GuidanceInput {
            tick: 0,
            intent: GuidanceIntent::AngularRate {
                rate_body_rps: DVec3::ZERO,
            },
            propulsion: PropulsionDemand::new(0.0).expect("zero propulsion"),
        };
        for _ in 0..=MAX_UPSTREAM_MESSAGES_PER_ITERATION {
            assert!(ingress.send_guidance("pilot", guidance.clone()));
        }
        let mut manual = input(Vec::new());
        manual.control_input = [0.4, 0.0, 0.0];
        assert!(ingress.send_input("pilot", manual));

        let (_, first_saturated) = driver.drain_upstream();
        assert!(first_saturated);
        assert_eq!(driver.sim.authority.control_input, DVec3::ZERO);
        let (_, second_saturated) = driver.drain_upstream();
        assert!(!second_saturated);
        assert_eq!(
            driver.sim.authority.control_input,
            DVec3::new(0.4, 0.0, 0.0)
        );
    }

    #[test]
    fn lowering_warp_rebases_old_pacing_debt() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let reference_body = ephemeris.body_id("thessa").expect("thessa");
        let sim = Sim::new(ephemeris, reference_body, false, true).expect("sim");
        let (upstream_tx, upstream_rx) = IngressSender::new();
        assert!(upstream_tx.reserve_exact_connection("pilot".into()));
        assert!(upstream_tx.mark_active_for_sender("pilot"));
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
        assert!(upstream_tx.send_input("pilot", input(vec![Command::SetWarp { factor: 64.0 }]),));

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
        let (upstream_tx, upstream_rx) = IngressSender::new();
        assert!(upstream_tx.reserve_exact_connection("pilot".into()));
        assert!(upstream_tx.mark_active_for_sender("pilot"));
        let snapshot_mailbox = Arc::new(OutboundMailbox::new());
        let mut driver = Driver {
            sim,
            autopilot: AutopilotHost::new().expect("autopilot host"),
            upstream: upstream_rx,
            subscribers: vec![("pilot".into(), snapshot_mailbox.clone())],
            pacing_target_s: 0.0,
            last_pacing: Instant::now(),
            last_snapshot: Instant::now(),
            last_status: Instant::now(),
            exit_when_empty: false,
        };
        driver.sim.register("pilot");
        assert!(upstream_tx.send_input(
            "pilot",
            input(vec![
                Command::SetWarp { factor: MAX_WARP },
                Command::Pause { paused: true },
            ]),
        ));

        let started = Instant::now();
        assert!(!driver.iterate().expect("driver iteration"));
        assert!(started.elapsed() < Duration::from_millis(100));
        let frame = snapshot_mailbox.try_next().expect("forced snapshot");
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
        pending.push(input(vec![Command::SetWarp { factor: 64.0 }]), 1);
        pending.push(input(vec![Command::Reset]), 2);
        pending.push(input(vec![Command::SetWarp { factor: 128.0 }]), 3);
        let merged = pending.take().expect("merged input");
        assert_eq!(merged.commands.len(), 2);
        assert_eq!(merged.commands[0], Command::Reset);
        assert_eq!(merged.commands[1], Command::SetWarp { factor: 128.0 });
    }
}
