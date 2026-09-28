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
pub(super) struct Sim {
    pub(super) authority: FlightAuthority,
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
            reaction_wheel_torque_body_nm: authority.reaction_wheel_telemetry().to_array(),
            parachutes: authority.parachute_telemetry().to_vec(),
            wake_notice: authority.wake_notice.clone(),
            flight_error: authority.flight_error.clone(),
        }
    }
}
