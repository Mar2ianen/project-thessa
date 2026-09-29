//! Authoritative flight state and its transport-independent operations.

use super::*;

/// One connected pilot: warp vote and pause vote. Effective warp is the
/// minimum of all votes (nobody gets dragged faster than they asked);
/// the sim pauses while anyone votes pause.
#[derive(Debug, Clone, Copy)]
pub(super) struct ClientVote {
    pub(super) warp: f64,
    paused: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AutopilotEvent {
    Impact,
    Horizon,
    Node,
    Alarm,
    Stage,
    Separate,
    EngineReady,
}

impl AutopilotEvent {
    pub(super) fn name(self) -> &'static str {
        match self {
            Self::Impact => "impact",
            Self::Horizon => "horizon",
            Self::Node => "node",
            Self::Alarm => "alarm",
            Self::Stage => "stage",
            Self::Separate => "separate",
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

pub(super) fn client_input_takes_over(previous: Option<&ClientInput>, input: &ClientInput) -> bool {
    if input.commands.iter().any(|command| {
        matches!(
            command,
            Command::Stage
                | Command::Engine { .. }
                | Command::Part { .. }
                | Command::Reset
                | Command::ExecuteManeuver { .. }
                | Command::ExecuteBurnPlan { .. }
                | Command::Separate { .. }
                | Command::Dock { .. }
                | Command::Undock { .. }
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
        || input.reaction_wheels_enabled != previous.reaction_wheels_enabled
        || input.gear_down != previous.gear_down
        || input.parachutes_armed != previous.parachutes_armed
}

pub(super) fn legacy_propulsion_echo_changed(
    previous: Option<&ClientInput>,
    input: &ClientInput,
    has_engine_command: bool,
) -> bool {
    previous.is_none_or(|previous| {
        input.throttle != previous.throttle
            || (!has_engine_command && input.engine_active != previous.engine_active)
    })
}

/// Driver around the authority: inputs in, snapshots out, warp accounting.
///
/// The primary vehicle keeps the single-vehicle fast path (`authority`).
/// Separated clusters spawn additional [`FlightAuthority`] entries in
/// `fleet` keyed by stable [`VehicleId`](thessa_sim_core::VehicleId)
/// values; dock sessions and fixed-joint handles live in `dock_graph` and
/// `dock_joints`. Secondaries fly passive (fresh default controls, no
/// autopilot graphs); commanding them is a later slice.
pub(super) struct Sim {
    pub(super) authority: FlightAuthority,
    pub(super) fleet: std::collections::BTreeMap<u32, FlightAuthority>,
    pub(super) next_vehicle_id: u32,
    pub(super) dock_graph: DockGraph,
    pub(super) dock_joints: std::collections::BTreeMap<(u32, u32), JointId>,
    pub(super) ephemeris: BakedEphemeris,
    pub(super) control_mode: ControlMode,
    /// Latest typed guidance command. Legacy ClientInput clears this so the
    /// two input protocols cannot fight over the same vehicle.
    pub(super) guidance: Option<(GuidanceIntent, PropulsionDemand)>,
    pub(super) plan_demand: Option<ControlDemand>,
    pub(super) autopilot_graph: Option<AutopilotGraph>,
    pub(super) graph_runner: Option<GraphRunner>,
    /// Currently parked autopilot phase (node, name, config, park time).
    /// The per-tick phase law steers from its IR parameters; completion
    /// publishes the node-name event plus the domain event, the watchdog
    /// fails the graph. Cleared on retire, takeover, and cancel.
    pub(super) phase_park: Option<PhasePark>,
    /// A parked Execute burn delegated to `maneuver_execution`: the
    /// executor owns guidance until it clears, then the burn retires.
    pub(super) burn_delegated: bool,
    pub(super) graph_block: NativeGraphBlock,
    /// Declared autopilot targets stay data-only until a later landing/
    /// impact planner consumes them and asks the field for obstacle evidence.
    pub(super) landing_site: Option<LandingSite>,
    pub(super) landing_obstacles: Option<ObstacleReport>,
    pub(super) impact_site: Option<ImpactSite>,
    pub(super) impact_obstacles: Option<ObstacleReport>,
    pub(super) plan_runner: Option<TrajectoryPlanRunner>,
    /// Active maneuver-plan execution (ExecuteManeuver block). Drives
    /// `guidance` through a `NodeExecutor`; cleared on completion/abort,
    /// after latching a zero-throttle hold so handoff is never abrupt.
    pub(super) maneuver_execution: Option<NodeExecutor>,
    /// Active finite-burn execution (ExecuteBurnPlan block). Drives
    /// `guidance` through a `SegmentExecutor` with the same handoff
    /// contract. Mutually exclusive with `maneuver_execution`: starting
    /// one refuses while the other is active rather than fighting it.
    pub(super) burn_execution: Option<SegmentExecutor>,
    pub(super) clients: std::collections::HashMap<String, ClientVote>,
    /// Connection-order pilot lease. The first registered client owns all
    /// vehicle controls; on departure the lease moves to the earliest client
    /// still connected, making transfer deterministic across transports.
    pub(super) pilot_owner: Option<String>,
    pub(super) client_order: Vec<String>,
    pub(super) last_client_inputs: std::collections::HashMap<String, ClientInput>,
    pub(super) autopilot_events: VecDeque<AutopilotEvent>,
    pub(super) advanced_s: f64,
    pub(super) compute_s: f64,
    pub(super) wall_started: Instant,
    pub(super) steps: u64,
    pub(super) rails_s: f64,
}

impl Sim {
    pub(super) fn new(
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
            fleet: std::collections::BTreeMap::new(),
            next_vehicle_id: 1,
            dock_graph: DockGraph::new(),
            dock_joints: std::collections::BTreeMap::new(),
            ephemeris,
            control_mode: ControlMode::Navball,
            guidance: None,
            plan_demand: None,
            autopilot_graph: None,
            graph_runner: None,
            graph_block: NativeGraphBlock::default(),
            phase_park: None,
            burn_delegated: false,
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
    pub(super) fn register(&mut self, id: &str) {
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

    pub(super) fn is_pilot(&self, id: &str) -> bool {
        self.pilot_owner.as_deref() == Some(id)
    }

    /// Drop a client's votes on disconnect. The caller cancels the host
    /// scheduler before this method; changing ownership also clears all
    /// stale vehicle control state.
    pub(super) fn unregister(&mut self, id: &str) {
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
    pub(super) fn effective_warp_limit(&self) -> f64 {
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

    pub(super) fn paused(&self) -> bool {
        self.clients.values().any(|vote| vote.paused)
    }

    /// Requested (consensual) warp for pacing the loop.
    pub(super) fn requested_warp(&self) -> f64 {
        self.effective_warp_limit()
    }

    pub(super) fn apply_input(&mut self, id: &str, input: &ClientInput) -> bool {
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
        // Engine/stage edges first clear current propulsion on manual takeover;
        // restore the coalesced legacy throttle for their resulting active state.
        if legacy_propulsion_echo_changed(previous.as_ref(), input, has_engine_command)
            || has_engine_command
        {
            self.authority.set_legacy_propulsion(input.throttle, active);
        }
        // These legacy state echoes are applied only when the client changes
        // them. Otherwise a stale last-value packet could undo an intervening
        // authoritative Part command (for example, one emitted by staging).
        let changed = |current: bool, previous: Option<bool>| {
            previous.is_none_or(|previous| current != previous)
        };
        if changed(
            input.sas_enabled,
            previous.as_ref().map(|previous| previous.sas_enabled),
        ) {
            self.authority.sas_enabled = input.sas_enabled;
        }
        if changed(
            input.rcs_enabled,
            previous.as_ref().map(|previous| previous.rcs_enabled),
        ) {
            self.authority.rcs_enabled = input.rcs_enabled;
        }
        if changed(
            input.reaction_wheels_enabled,
            previous
                .as_ref()
                .map(|previous| previous.reaction_wheels_enabled),
        ) {
            self.authority.reaction_wheels_enabled = input.reaction_wheels_enabled;
        }
        if changed(
            input.gear_down,
            previous.as_ref().map(|previous| previous.gear_down),
        ) {
            self.authority.set_gear_down(input.gear_down);
        }
        if changed(
            input.parachutes_armed,
            previous.as_ref().map(|previous| previous.parachutes_armed),
        ) {
            self.authority.set_parachutes_armed(input.parachutes_armed);
        }
        let mut engine_command_seen = false;
        for command in &input.commands {
            // A jointed stack has no combined control model yet: thrust and
            // effector commands on a jointed primary fail closed instead of
            // driving one side of the constraint.
            if self.is_jointed(VehicleId::PRIMARY.0)
                && matches!(
                    command,
                    Command::Stage
                        | Command::Engine { .. }
                        | Command::Part { .. }
                        | Command::ExecuteManeuver { .. }
                        | Command::ExecuteBurnPlan { .. }
                )
            {
                self.authority.wake_notice =
                    Some("jointed vehicles cannot thrust: separate or undock first".into());
                continue;
            }
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
                Command::Part { command } => {
                    force_snapshot = true;
                    if let Err(error) = self.authority.apply_part_command(command) {
                        self.authority.wake_notice =
                            Some(format!("part command rejected: {error}"));
                    }
                }
                Command::Separate {
                    vehicle_id,
                    link_name,
                } => match self.separate_vehicle(vehicle_id.0, link_name) {
                    Ok(spawned) => {
                        force_snapshot = true;
                        self.autopilot_events.push_back(AutopilotEvent::Separate);
                        self.authority.wake_notice =
                            Some(format!("separated into {} vehicles", spawned.len() + 1));
                    }
                    Err(error) => {
                        self.authority.wake_notice = Some(format!("separate rejected: {error}"));
                    }
                },
                Command::Dock {
                    vehicle_id,
                    port_id,
                    target_vehicle_id,
                    target_port_id,
                } => match self.dock_vehicles(
                    vehicle_id.0,
                    port_id,
                    target_vehicle_id.0,
                    target_port_id,
                ) {
                    Ok(()) => {
                        force_snapshot = true;
                        self.authority.wake_notice =
                            Some(format!("docking session opened: {port_id}"));
                    }
                    Err(error) => {
                        self.authority.wake_notice = Some(format!("dock rejected: {error}"));
                    }
                },
                Command::Undock {
                    vehicle_id,
                    port_id,
                } => match self.undock_vehicle_port(vehicle_id.0, port_id) {
                    Ok(()) => {
                        force_snapshot = true;
                        self.authority.wake_notice = Some(format!("undocked {port_id}"));
                    }
                    Err(error) => {
                        self.authority.wake_notice = Some(format!("undock rejected: {error}"));
                    }
                },
                Command::ExecuteManeuver { nodes } => {
                    // Wire cap: node vectors are unbounded on the transport.
                    let rejected = |sim: &mut Self, reason: String| {
                        sim.authority.wake_notice = Some(format!("maneuver rejected: {reason}"));
                    };
                    if nodes.len() > thessa_flight_net::MAX_MANEUVER_NODES {
                        rejected(
                            self,
                            format!(
                                "{} nodes over cap {}",
                                nodes.len(),
                                thessa_flight_net::MAX_MANEUVER_NODES
                            ),
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
                    let rejected = |sim: &mut Self, reason: String| {
                        sim.authority.wake_notice = Some(format!("burn plan rejected: {reason}"));
                    };
                    if segments.len() > thessa_flight_net::MAX_BURN_SEGMENTS {
                        rejected(
                            self,
                            format!(
                                "{} segments over cap {}",
                                segments.len(),
                                thessa_flight_net::MAX_BURN_SEGMENTS
                            ),
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

    /// Advance one requested wall quantum (or a benchmark chunk); returns
    /// sim-seconds actually advanced. The optional budget only makes ordinary
    /// physics cooperative; it never changes the fixed solver dt.
    pub(super) fn advance_chunk(&mut self, chunk_s: f64) -> Result<f64, String> {
        self.advance_chunk_with_budget(chunk_s, None)
    }

    pub(super) fn advance_chunk_with_budget(
        &mut self,
        chunk_s: f64,
        budget: Option<Duration>,
    ) -> Result<f64, String> {
        if self.paused() || self.authority.flight_error.is_some() {
            return Ok(0.0);
        }
        // Maneuver executions poll before stepping so commands ride the
        // freshest thrust measurement from the previous tick. Parked
        // autopilot phases steer the same way through their IR laws.
        self.poll_maneuver_execution()?;
        self.poll_burn_execution()?;
        self.poll_phase_law()?;
        let mut chunk_s = chunk_s;
        if self.plan_runner.is_some() {
            let now = SimTime(self.authority.flight_time_s);
            if let Some(until) = self.prepare_plan(None)? {
                chunk_s = chunk_s.min((until.0 - now.0).max(0.0));
            }
        }
        // Jointed scenes have a bounded per-frame service quantum. Keep the
        // whole fleet on one simulation clock while that scene is active by
        // applying the same cap to the primary and passive vehicles.
        if !self.dock_joints.is_empty() {
            chunk_s = chunk_s.min(JOINTED_SCENE_DEBT_CAP_S);
        }
        let before = self.authority.flight_time_s;
        let started = Instant::now();
        let plan_demand = self.plan_demand;
        let guidance = self.guidance.clone();
        // A jointed primary rides the shared scene instead of its own
        // stepper: exactly one integrator owns a body per tick.
        let primary_pair = self.jointed_pair_with(VehicleId::PRIMARY.0);
        let result: Result<(), String> = if let Some(pair) = primary_pair {
            self.step_jointed_pair(pair, chunk_s)
        } else if let Some(demand) = plan_demand {
            self.authority
                .advance_control_demand_with_budget(&self.ephemeris, demand, chunk_s, budget)
                .map_err(|error| error.to_string())
        } else if let Some((intent, propulsion)) = guidance {
            // Typed guidance uses the same authoritative stepper; the legacy
            // mode is only the compatibility representation used by traces.
            self.authority
                .advance_guidance_with_budget(&self.ephemeris, &intent, propulsion, chunk_s, budget)
                .map_err(|error| error.to_string())
        } else {
            self.authority
                .advance_with_budget(&self.ephemeris, self.control_mode, chunk_s, budget)
                .map_err(|error| error.to_string())
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
        // Jointed non-primary pairs ride their own scene ticks; every other
        // secondary serves the same chunk through passive fixed steps.
        // Secondary stepping is unbounded by the CPU budget (fleet sizes stay
        // small); cooperative budgeting across the fleet is later work.
        let jointed_pairs: Vec<(u32, u32)> = self.dock_joints.keys().copied().collect();
        for pair in &jointed_pairs {
            if pair.0 == VehicleId::PRIMARY.0 {
                continue;
            }
            if let Err(error) = self.step_jointed_pair(*pair, advanced) {
                eprintln!("[server] jointed pair {pair:?} failed: {error}");
                self.authority.wake_notice = Some(format!("jointed pair failed: {error}"));
            }
        }
        let secondary_ids: Vec<u32> = self.fleet.keys().copied().collect();
        for id in secondary_ids {
            if self.is_jointed(id) {
                continue;
            }
            let Some(secondary) = self.fleet.get_mut(&id) else {
                continue;
            };
            if secondary.flight_error.is_some() {
                continue;
            }
            if let Err(error) =
                secondary.advance_with_budget(&self.ephemeris, ControlMode::Direct, advanced, None)
            {
                secondary.engine_active = false;
                secondary.flight_error = Some(error.to_string());
                eprintln!("[server] secondary vehicle {id} failed: {error}");
                continue;
            }
            self.steps += secondary.steps_this_frame as u64;
            self.rails_s += secondary.rails_advanced_this_frame;
        }
        self.poll_dock_sessions(advanced);
        self.autopilot_events.extend(
            self.authority
                .take_wake_events()
                .into_iter()
                .map(|event| AutopilotEvent::from(event.kind)),
        );
        self.poll_graph(None)?;
        Ok(advanced)
    }

    pub(super) fn effective_warp(&self) -> f64 {
        let wall_s = self.wall_started.elapsed().as_secs_f64();
        if wall_s <= 0.0 {
            0.0
        } else {
            self.advanced_s / wall_s
        }
    }

    pub(super) fn wall_s(&self) -> f64 {
        self.wall_started.elapsed().as_secs_f64()
    }

    /// Serve-path launch site: canonical field plus the COAST survey
    /// bookmark, derived exactly like the client survey (same recipe, same
    /// scan), so terrain collisions and spawn state match without ever
    /// transferring world state. Bench paths (`--measure`, `--drift`,
    /// `--vacuum`) skip this deliberately: they place the craft in orbit
    /// and must not pay field-build time or terrain checks.
    pub(super) fn init_launch_site(&mut self) -> Result<(), String> {
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

    pub(super) fn snapshot(&self) -> Snapshot {
        Self::snapshot_of(
            &self.authority,
            self.paused(),
            self.effective_warp(),
            self.compute_s,
            self.wall_s(),
        )
    }

    fn snapshot_of(
        authority: &FlightAuthority,
        paused: bool,
        effective_warp: f64,
        compute_s: f64,
        wall_s: f64,
    ) -> Snapshot {
        Snapshot {
            tick: authority.world_tick.0,
            flight_time_s: authority.flight_time_s,
            state: authority.state,
            throttle: authority.throttle,
            engine_active: authority.engine_active,
            paused,
            effective_warp,
            server_compute_s: compute_s,
            server_wall_s: wall_s,
            steps_this_frame: authority.steps_this_frame,
            rails_advanced_s: authority.rails_advanced_this_frame,
            reaction_wheel_torque_body_nm: authority.reaction_wheel_telemetry().to_array(),
            parachutes: authority.parachute_telemetry().to_vec(),
            wake_notice: authority.wake_notice.clone(),
            flight_error: authority.flight_error.clone(),
        }
    }

    /// Fleet snapshots for non-primary vehicles in ascending id order. The
    /// primary keeps its dedicated snapshot path.
    pub(super) fn fleet_snapshot(&self) -> FleetSnapshot {
        let paused = self.paused();
        let effective_warp = self.effective_warp();
        let compute_s = self.compute_s;
        let wall_s = self.wall_s();
        FleetSnapshot {
            vehicles: self
                .fleet
                .iter()
                .map(|(id, authority)| FleetVehicleSnapshot {
                    vehicle_id: VehicleId::new(*id),
                    snapshot: Self::snapshot_of(
                        authority,
                        paused,
                        effective_warp,
                        compute_s,
                        wall_s,
                    ),
                })
                .collect(),
        }
    }
}

/// Fixed pressure-equalization interval for server dock sessions (s).
/// Orifice/valve sizing is future work (`details/06`); the constant lives
/// here explicitly instead of spread through call sites.
pub(super) const DOCK_PRESSURE_EQUALIZATION_S: f64 = 30.0;
/// Terrain-clearance hysteresis arming the shared dock scene. In orbit the
/// scene stays armed-but-inactive so free flight is preserved; it only
/// hosts dock-partner bodies and fixed joints.
pub(super) const DOCK_SCENE_ENTER_M: f64 = 5_000.0;
pub(super) const DOCK_SCENE_EXIT_M: f64 = 10_000.0;
/// Per-tick cap for jointed-scene cruise debt (s). Unserved warp debt sheds
/// exactly like the CPU work budget: effective warp reports the result.
/// Fixed solver step for jointed-scene cruise (s).
pub(super) const JOINTED_SCENE_DEBT_CAP_S: f64 = 0.25;
pub(super) const JOINTED_SCENE_DT_S: f64 = 1.0 / 120.0;
/// Upper bound on authoritative vehicles (primary plus spawned clusters).
pub(super) const MAX_FLEET_VEHICLES: usize = 64;
/// Solver body config for dock-partner sync: full CCD, never sleep, so a
/// jointed pair cannot freeze mid-protocol.
pub(super) const DOCK_BODY_CONFIG: DynamicBodyConfig = DynamicBodyConfig {
    full_ccd: true,
    can_sleep: false,
};

impl Sim {
    pub(super) fn vehicle_authority(&self, id: u32) -> Result<&FlightAuthority, String> {
        if id == VehicleId::PRIMARY.0 {
            Ok(&self.authority)
        } else {
            self.fleet
                .get(&id)
                .ok_or_else(|| format!("unknown vehicle {id}"))
        }
    }

    /// Jointed pair containing a vehicle, if any. Joints never outlive
    /// their session: a pair is jointed exactly when stepping is shared.
    pub(super) fn jointed_pair_with(&self, id: u32) -> Option<(u32, u32)> {
        self.dock_joints
            .keys()
            .find(|pair| pair.0 == id || pair.1 == id)
            .copied()
    }

    pub(super) fn is_jointed(&self, id: u32) -> bool {
        self.jointed_pair_with(id).is_some()
    }

    /// Split one authoritative vehicle along a named structural link.
    /// Returns the spawned vehicle ids. The cluster containing the source
    /// root body keeps the source id with fresh default controls; every
    /// other cluster spawns a passive secondary authority sharing the fleet
    /// clock. Active dock sessions block the split: undock first.
    pub(super) fn separate_vehicle(
        &mut self,
        vehicle_id: u32,
        link_name: &str,
    ) -> Result<Vec<u32>, String> {
        if link_name.trim().is_empty() {
            return Err("link name must be non-empty".into());
        }
        if self.dock_graph.has_vehicle(VehicleId::new(vehicle_id)) {
            return Err(format!(
                "vehicle {vehicle_id} must undock before it separates"
            ));
        }
        let root_name = {
            let authority = self.vehicle_authority(vehicle_id)?;
            let assembly = authority
                .vehicle
                .assembly
                .as_ref()
                .ok_or_else(|| format!("vehicle {vehicle_id} has no part assembly graph"))?;
            assembly.body_names[assembly.root_body].clone()
        };
        let reference_body = self.vehicle_authority(vehicle_id)?.reference_body;
        let source_time = self.vehicle_authority(vehicle_id)?.flight_time_s;
        let source_tick = self.vehicle_authority(vehicle_id)?.world_tick;
        let definitions = {
            let authority = self.vehicle_authority(vehicle_id)?;
            authority
                .vehicle
                .split_definitions_after_link_failure(
                    link_name,
                    authority.state,
                    &authority.resource_state,
                )
                .map_err(|error| error.to_string())?
        };
        if self.fleet.len() + definitions.len() > MAX_FLEET_VEHICLES {
            return Err("fleet is full".into());
        }
        let keep_index = definitions
            .iter()
            .position(|(definition, _, _)| {
                definition
                    .assembly
                    .as_ref()
                    .is_some_and(|assembly| assembly.body_names.contains(&root_name))
            })
            .ok_or_else(|| "split lost the root body".to_string())?;
        let mut prepared = Vec::with_capacity(definitions.len());
        for (definition, state, resource_state) in definitions {
            let mut authority =
                FlightAuthority::new_with_vehicle(&self.ephemeris, reference_body, definition)
                    .map_err(|error| format!("cluster authority: {error}"))?;
            authority.state = state;
            authority.resource_state = resource_state;
            authority.flight_time_s = source_time;
            authority.world_tick = source_tick;
            prepared.push(authority);
        }
        if vehicle_id == VehicleId::PRIMARY.0 {
            self.cancel_autopilot_tasks();
            self.clear_autopilot_controls();
            self.authority.flight_error = None;
        }
        let mut spawned = Vec::new();
        for (index, authority) in prepared.into_iter().enumerate() {
            if index == keep_index {
                if vehicle_id == VehicleId::PRIMARY.0 {
                    self.authority = authority;
                } else {
                    self.fleet.insert(vehicle_id, authority);
                }
            } else {
                let id = self.next_vehicle_id;
                self.next_vehicle_id += 1;
                self.fleet.insert(id, authority);
                spawned.push(id);
            }
        }
        Ok(spawned)
    }

    /// Open a persisted docking session between two authored ports. Frames
    /// resolve from server definitions; the protocol advances on later
    /// ticks through [`poll_dock_sessions`](Self::poll_dock_sessions).
    pub(super) fn dock_vehicles(
        &mut self,
        a: u32,
        port_a: &str,
        b: u32,
        port_b: &str,
    ) -> Result<(), String> {
        let spec_a = {
            let authority = self.vehicle_authority(a)?;
            authority
                .vehicle
                .docking_ports
                .iter()
                .find(|port| port.id == port_a)
                .cloned()
                .ok_or_else(|| format!("vehicle {a} has no docking port '{port_a}'"))?
        };
        let spec_b = {
            let authority = self.vehicle_authority(b)?;
            authority
                .vehicle
                .docking_ports
                .iter()
                .find(|port| port.id == port_b)
                .cloned()
                .ok_or_else(|| format!("vehicle {b} has no docking port '{port_b}'"))?
        };
        self.dock_graph
            .begin_session(
                VehicleId::new(a),
                spec_a,
                VehicleId::new(b),
                spec_b,
                DOCK_PRESSURE_EQUALIZATION_S,
            )
            .map_err(|error| error.to_string())?;
        Ok(())
    }

    /// End the session holding a vehicle port, removing its fixed joint
    /// first when one is installed. Both solved states survive.
    pub(super) fn undock_vehicle_port(
        &mut self,
        vehicle_id: u32,
        port_id: &str,
    ) -> Result<(), String> {
        let pair = self
            .dock_graph
            .pairs()
            .into_iter()
            .find(|pair| {
                (pair.0 == vehicle_id || pair.1 == vehicle_id)
                    && self
                        .dock_graph
                        .session(VehicleId::new(pair.0), VehicleId::new(pair.1))
                        .is_ok_and(|session| {
                            session.port_a.id == port_id || session.port_b.id == port_id
                        })
            })
            .ok_or_else(|| format!("vehicle {vehicle_id} has no docking session on '{port_id}'"))?;
        self.release_joint(pair, format!("undocked {port_id}"));
        Ok(())
    }

    /// Remove a fixed joint and end its session. The pair returns to
    /// independent exact flight; the reason surfaces on the primary notice.
    pub(super) fn release_joint(&mut self, pair: (u32, u32), reason: String) {
        if let Some(joint) = self.dock_joints.remove(&pair) {
            let host: Option<&mut FlightAuthority> = if pair.0 == VehicleId::PRIMARY.0 {
                Some(&mut self.authority)
            } else {
                self.fleet.get_mut(&pair.0)
            };
            if let Some(host) = host
                && let Some(runtime) = host.contact_runtime_mut()
            {
                let _ = runtime.undock(joint);
                let _ = runtime.remove_partner(pair.1 as u64);
            }
        }
        let _ = self
            .dock_graph
            .end_session(VehicleId::new(pair.0), VehicleId::new(pair.1));
        self.authority.wake_notice = Some(reason);
    }

    /// True vacuum for jointed cruise: sampled density at or below the same
    /// cutoff the flight stepper uses for its exact-vacuum fast paths.
    /// Unsampleable evidence fails closed (no joint).
    pub(super) fn authority_is_vacuum(&self, authority: &FlightAuthority) -> bool {
        let time = SimTime(authority.flight_time_s);
        let Ok(body) = self.ephemeris.body(authority.reference_body) else {
            return false;
        };
        let Ok(body_state) = self.ephemeris.body_state(authority.reference_body, time) else {
            return false;
        };
        let altitude_m = (authority.state.position_inertial_m - body_state.position_inertial)
            .length()
            - body.radius_m;
        let Ok(sample) = authority.atmosphere.sample(altitude_m) else {
            return false;
        };
        sample.density_kg_m3 <= authority.atmosphere.vacuum_cutoff_density_kg_m3
    }

    /// Gravity-only wrench for jointed cruise. Exact in vacuum (the only
    /// regime joints install in); thrust and aero while jointed are rejected
    /// at the command boundary because the docked stack has no combined
    /// control model yet.
    /// Step one jointed pair through the host scene with gravity wrenches,
    /// capped per-tick debt like the CPU work budget. Unserved warp debt
    /// sheds; effective warp reports the result.
    pub(super) fn step_jointed_pair(
        &mut self,
        pair: (u32, u32),
        debt_s: f64,
    ) -> Result<(), String> {
        if debt_s <= 0.0 {
            return Ok(());
        }
        let vacuum = self
            .vehicle_authority(pair.0)
            .is_ok_and(|authority| self.authority_is_vacuum(authority))
            && self
                .vehicle_authority(pair.1)
                .is_ok_and(|authority| self.authority_is_vacuum(authority));
        if !vacuum {
            self.release_joint(pair, "joint released: atmosphere".into());
            return Ok(());
        }
        let (
            host_state,
            host_props,
            host_geom,
            partner_state,
            partner_props,
            partner_geom,
            host_time,
            host_mass,
        ) = {
            let host = self.vehicle_authority(pair.0)?;
            let partner = self.vehicle_authority(pair.1)?;
            (
                host.state,
                host.vehicle.mass_properties,
                host.vehicle.collision_geometry.clone(),
                partner.state,
                partner.vehicle.mass_properties,
                partner.vehicle.collision_geometry.clone(),
                host.flight_time_s,
                host.vehicle.mass_properties.mass_kg,
            )
        };
        let cap = debt_s.min(JOINTED_SCENE_DEBT_CAP_S);
        let (served, partner_state) = {
            let host: &mut FlightAuthority = if pair.0 == VehicleId::PRIMARY.0 {
                &mut self.authority
            } else {
                self.fleet
                    .get_mut(&pair.0)
                    .ok_or_else(|| format!("joint host {} is gone", pair.0))?
            };
            let runtime = host
                .contact_runtime_mut()
                .ok_or_else(|| "dock scene is gone".to_string())?;
            let host_id = runtime
                .sync_body(host_state, host_props, &host_geom, DOCK_BODY_CONFIG)
                .map_err(|error| error.to_string())?;
            let tag = pair.1 as u64;
            let partner_id = runtime
                .sync_partner(
                    tag,
                    partner_state,
                    partner_props,
                    &partner_geom,
                    DOCK_BODY_CONFIG,
                )
                .map_err(|error| error.to_string())?;
            let field = GravityField::from_ephemeris(&self.ephemeris);
            let partner_mass = partner_props.mass_kg;
            let mut primary_state = host_state;
            let mut secondary_state = partner_state;
            let mut served = 0.0;
            while served < cap {
                let step = (cap - served).min(JOINTED_SCENE_DT_S);
                let gravity_a = field
                    .acceleration(
                        primary_state.position_inertial_m,
                        SimTime(host_time + served),
                    )
                    .map_err(|error| error.to_string())?;
                let gravity_b = field
                    .acceleration(
                        secondary_state.position_inertial_m,
                        SimTime(host_time + served),
                    )
                    .map_err(|error| error.to_string())?;
                primary_state = runtime
                    .step_many(
                        step,
                        &[
                            (
                                host_id,
                                ExternalWrench {
                                    force_inertial_n: gravity_a * host_mass,
                                    torque_inertial_nm: DVec3::ZERO,
                                },
                            ),
                            (
                                partner_id,
                                ExternalWrench {
                                    force_inertial_n: gravity_b * partner_mass,
                                    torque_inertial_nm: DVec3::ZERO,
                                },
                            ),
                        ],
                    )
                    .map_err(|error| error.to_string())?;
                secondary_state = runtime
                    .partner_state(tag)
                    .map_err(|error| error.to_string())?;
                served += step;
            }
            host.state = primary_state;
            host.flight_time_s = host_time + served;
            host.rails.invalidate();
            (served, secondary_state)
        };
        let mut partner = self
            .fleet
            .remove(&pair.1)
            .ok_or_else(|| format!("joint partner {} is gone", pair.1))?;
        partner.state = partner_state;
        partner.flight_time_s += served;
        partner.rails.invalidate();
        self.fleet.insert(pair.1, partner);
        Ok(())
    }

    /// Advance every persisted dock session in simulation time. Capture and
    /// alignment gate on live port kinematics; the fixed joint installs only
    /// in vacuum, and hard dock follows the installed joint (never the
    /// reverse), so protocol truth cannot outrun the constraint.
    pub(super) fn poll_dock_sessions(&mut self, step_s: f64) {
        if step_s <= 0.0 {
            return;
        }
        for pair in self.dock_graph.pairs() {
            let snapshot = (|| -> Result<DockingPortState, String> {
                let session = self
                    .dock_graph
                    .session(VehicleId::new(pair.0), VehicleId::new(pair.1))
                    .map_err(|error| error.to_string())?;
                Ok(session.state)
            })();
            let Ok(proto_state) = snapshot else {
                continue;
            };
            match proto_state {
                DockingPortState::Free | DockingPortState::SoftCapture => {
                    self.poll_dock_approach(pair);
                }
                DockingPortState::Aligned => {
                    self.poll_dock_install(pair);
                }
                DockingPortState::HardDock => {
                    self.poll_dock_engage(pair);
                }
                DockingPortState::OuterStructureEngaged => {
                    let advanced = (|| -> Result<bool, String> {
                        let session = self
                            .dock_graph
                            .session_mut(VehicleId::new(pair.0), VehicleId::new(pair.1))
                            .map_err(|error| error.to_string())?;
                        session
                            .advance_pressure_equalization(step_s)
                            .map_err(|error| error.to_string())?;
                        Ok(session.state == DockingPortState::PressureEqualized)
                    })();
                    match advanced {
                        Ok(true) => {
                            self.authority.wake_notice = Some("docking pressure equalized".into());
                        }
                        Err(error) => {
                            self.authority.wake_notice =
                                Some(format!("docking equalization failed: {error}"));
                        }
                        Ok(false) => {}
                    }
                }
                DockingPortState::PressureEqualized => {
                    let vacuum = self
                        .vehicle_authority(pair.0)
                        .is_ok_and(|authority| self.authority_is_vacuum(authority))
                        && self
                            .vehicle_authority(pair.1)
                            .is_ok_and(|authority| self.authority_is_vacuum(authority));
                    if !vacuum {
                        self.release_joint(pair, "joint released: atmosphere".into());
                    }
                }
            }
        }
    }

    /// Gate capture and alignment on live port kinematics. Out-of-tolerance
    /// evidence retries silently on later ticks; corrupt evidence surfaces.
    fn poll_dock_approach(&mut self, pair: (u32, u32)) {
        let kinematics = (|| -> Result<(DockingKinematics, DockingPortState), String> {
            let session = self
                .dock_graph
                .session(VehicleId::new(pair.0), VehicleId::new(pair.1))
                .map_err(|error| error.to_string())?;
            let host = self.vehicle_authority(pair.0)?;
            let partner = self.vehicle_authority(pair.1)?;
            let kinematics = DockingKinematics::between(
                host.state,
                &session.port_a,
                partner.state,
                &session.port_b,
            )
            .map_err(|error| error.to_string())?;
            Ok((kinematics, session.state))
        })();
        let Ok((kinematics, proto_state)) = kinematics else {
            return;
        };
        let outcome = (|| -> Result<(), String> {
            let session = self
                .dock_graph
                .session_mut(VehicleId::new(pair.0), VehicleId::new(pair.1))
                .map_err(|error| error.to_string())?;
            match proto_state {
                DockingPortState::Free => session
                    .begin_soft_capture(kinematics.relative_velocity_mps().length())
                    .map_err(|error| error.to_string()),
                DockingPortState::SoftCapture => {
                    session.align(kinematics).map_err(|error| error.to_string())
                }
                _ => Ok(()),
            }
        })();
        if let Err(error) = outcome {
            // Gate misses retry silently; structural failures surface.
            if error.contains("non-finite") || error.contains("unknown") {
                self.authority.wake_notice = Some(format!("docking approach failed: {error}"));
            }
        }
    }

    /// Install the fixed joint for an aligned pair, then hard-dock. Vacuum
    /// gates the install; the joint handle is recorded before the protocol
    /// advances so truth never outruns the constraint.
    fn poll_dock_install(&mut self, pair: (u32, u32)) {
        let vacuum = self
            .vehicle_authority(pair.0)
            .is_ok_and(|authority| self.authority_is_vacuum(authority))
            && self
                .vehicle_authority(pair.1)
                .is_ok_and(|authority| self.authority_is_vacuum(authority));
        if !vacuum {
            return;
        }
        let install = (|| -> Result<JointId, String> {
            let (
                host_state,
                host_props,
                host_geom,
                partner_state,
                partner_props,
                partner_geom,
                spec_a,
                spec_b,
            ) = {
                let session = self
                    .dock_graph
                    .session(VehicleId::new(pair.0), VehicleId::new(pair.1))
                    .map_err(|error| error.to_string())?;
                let host = self.vehicle_authority(pair.0)?;
                let partner = self.vehicle_authority(pair.1)?;
                (
                    host.state,
                    host.vehicle.mass_properties,
                    host.vehicle.collision_geometry.clone(),
                    partner.state,
                    partner.vehicle.mass_properties,
                    partner.vehicle.collision_geometry.clone(),
                    session.port_a.clone(),
                    session.port_b.clone(),
                )
            };
            let host: &mut FlightAuthority = if pair.0 == VehicleId::PRIMARY.0 {
                &mut self.authority
            } else {
                self.fleet
                    .get_mut(&pair.0)
                    .ok_or_else(|| format!("joint host {} is gone", pair.0))?
            };
            if host.contact_runtime().is_none() {
                host.enable_contact_mode(DOCK_SCENE_ENTER_M, DOCK_SCENE_EXIT_M)
                    .map_err(|error| error.to_string())?;
            }
            let runtime = host
                .contact_runtime_mut()
                .ok_or_else(|| "dock scene is gone".to_string())?;
            let tag = pair.1 as u64;
            runtime
                .sync_body(host_state, host_props, &host_geom, DOCK_BODY_CONFIG)
                .map_err(|error| error.to_string())?;
            runtime
                .sync_partner(
                    tag,
                    partner_state,
                    partner_props,
                    &partner_geom,
                    DOCK_BODY_CONFIG,
                )
                .map_err(|error| error.to_string())?;
            runtime
                .dock_partner(
                    tag,
                    spec_a.local_position_m,
                    spec_a.local_orientation,
                    spec_b.local_position_m,
                    spec_b.local_orientation,
                )
                .map_err(|error| error.to_string())
        })();
        match install {
            Ok(joint) => {
                self.dock_joints.insert(pair, joint);
                let outcome = (|| -> Result<(), String> {
                    let session = self
                        .dock_graph
                        .session_mut(VehicleId::new(pair.0), VehicleId::new(pair.1))
                        .map_err(|error| error.to_string())?;
                    session.hard_dock().map_err(|error| error.to_string())?;
                    session
                        .engage_outer_structure()
                        .map_err(|error| error.to_string())
                })();
                match outcome {
                    Ok(()) => {
                        self.authority.wake_notice = Some("hard dock".into());
                    }
                    Err(error) => {
                        self.release_joint(pair, format!("docking protocol failed: {error}"));
                    }
                }
            }
            Err(error) => {
                self.authority.wake_notice = Some(format!("docking install failed: {error}"));
            }
        }
    }

    /// Retry outer-structure engagement for a hard-docked pair.
    fn poll_dock_engage(&mut self, pair: (u32, u32)) {
        let outcome = (|| -> Result<(), String> {
            let session = self
                .dock_graph
                .session_mut(VehicleId::new(pair.0), VehicleId::new(pair.1))
                .map_err(|error| error.to_string())?;
            session
                .engage_outer_structure()
                .map_err(|error| error.to_string())
        })();
        if let Err(error) = outcome {
            self.authority.wake_notice = Some(format!("docking engage failed: {error}"));
        }
    }
}
