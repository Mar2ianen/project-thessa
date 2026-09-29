//! Shared authoritative simulation driver for every transport.

use super::*;

/// Shared sim driver: one authoritative tick order for every transport.
/// stdio and TCP only differ in how frames arrive and where snapshots go.
pub(super) struct Driver {
    pub(super) sim: Sim,
    pub(super) autopilot: AutopilotHost,
    pub(super) upstream: IngressReceiver,
    pub(super) subscribers: Vec<(String, Arc<OutboundMailbox>)>,
    /// Desired authoritative sim time generated from real wall time and the
    /// current consensus warp. Actual service catches this target when the
    /// machine is fast enough; when it is not, the driver stays busy between
    /// quanta and effective warp falls honestly.
    pub(super) pacing_target_s: f64,
    pub(super) last_pacing: Instant,
    pub(super) last_snapshot: Instant,
    pub(super) last_status: Instant,
    pub(super) exit_when_empty: bool,
}

impl Driver {
    pub(super) fn apply_client_input(&mut self, id: &str, input: &ClientInput) -> bool {
        let takeover = self.sim.is_pilot(id)
            && input.validate().is_ok()
            && client_input_takes_over(self.sim.last_client_inputs.get(id), input);
        let force_snapshot = self.sim.apply_input(id, input);
        if takeover {
            self.autopilot.scheduler.cancel_all();
        }
        force_snapshot
    }

    pub(super) fn broadcast_snapshot(&mut self) {
        let snapshot = self.sim.snapshot();
        let frame = match thessa_flight_net::encode_snapshot(&snapshot) {
            Ok(frame) => frame,
            Err(error) => {
                eprintln!("[server] snapshot encode error: {error}");
                return;
            }
        };
        let frame = Arc::new(frame);
        self.subscribers.retain(|(_, mailbox)| {
            let accepted = mailbox.replace_snapshot(frame.clone());
            if !accepted {
                mailbox.close();
            }
            accepted
        });
        if self.sim.fleet.is_empty() {
            return;
        }
        let fleet = self.sim.fleet_snapshot();
        let frame = match thessa_flight_net::encode_fleet_snapshot(&fleet) {
            Ok(frame) => frame,
            Err(error) => {
                eprintln!("[server] fleet snapshot encode error: {error}");
                return;
            }
        };
        let frame = Arc::new(frame);
        self.subscribers.retain(|(_, mailbox)| {
            let accepted = mailbox.replace_fleet_snapshot(frame.clone());
            if !accepted {
                mailbox.close();
            }
            accepted
        });
    }

    /// Drain a bounded ingress slice. The limit only prevents a hot producer
    /// from monopolising the sim thread; per-client continuous input is still
    /// coalesced and event commands are retained in order.
    pub(super) fn drain_upstream(&mut self) -> (bool, bool) {
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

    pub(super) fn flush_pending_input(
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

    pub(super) fn release_client(&mut self, id: &str) {
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
    pub(super) fn iterate(&mut self) -> Result<bool, String> {
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
