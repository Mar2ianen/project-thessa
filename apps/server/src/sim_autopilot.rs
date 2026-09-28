//! Script, graph, maneuver, and phase-law operations on the server sim.

use super::*;

impl Sim {
    pub(super) fn apply_guidance(&mut self, id: &str, input: &GuidanceInput) -> bool {
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

    pub(super) fn apply_guidance_command(
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

    pub(super) fn apply_autopilot(
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

    pub(super) fn submit_graph(&mut self, graph: AutopilotGraph) -> bool {
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

    pub(super) fn poll_graph(&mut self, event: Option<&str>) -> Result<(), String> {
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
        self.reconcile_phase_park(&state);
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

    /// Adopt newly parked phases; retire the active one when the runner
    /// moved past it or terminated, latching a zero-throttle attitude hold
    /// so no stale full command rides the gap between phases.
    pub(super) fn reconcile_phase_park(&mut self, state: &thessa_autopilot::GraphRunState) {
        for (node, name, config) in self.graph_block.take_parked_phases() {
            self.phase_park = Some(PhasePark {
                node,
                name,
                config,
                parked_at_s: self.authority.flight_time_s,
            });
            self.burn_delegated = false;
        }
        let retired = match state {
            thessa_autopilot::GraphRunState::Complete
            | thessa_autopilot::GraphRunState::Failed { .. }
            | thessa_autopilot::GraphRunState::Aborted { .. } => true,
            thessa_autopilot::GraphRunState::Waiting { node, .. } => self
                .phase_park
                .as_ref()
                .is_some_and(|park| park.node != *node),
            thessa_autopilot::GraphRunState::Progress { .. } => false,
        };
        if retired && self.phase_park.is_some() {
            self.phase_park = None;
            self.burn_delegated = false;
            self.guidance = Some((
                GuidanceIntent::Attitude {
                    target_body_to_inertial: self.authority.state.orientation_body_to_inertial,
                    roll_policy: RollPolicy::Hold,
                },
                PropulsionDemand::new(0.0).expect("zero propulsion always builds"),
            ));
        }
    }

    pub(super) fn apply_script_step(
        &mut self,
        step: ScriptSchedulerStep,
        host: &mut AutopilotHost,
    ) -> bool {
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

    pub(super) fn declare_landing_site(&mut self, site: LandingSite) -> bool {
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

    pub(super) fn declare_impact_site(&mut self, site: ImpactSite) -> bool {
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

    pub(super) fn apply_script_guidance(&mut self, intent: GuidanceIntent) -> bool {
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
    pub(super) fn clear_autopilot_controls(&mut self) {
        self.plan_demand = None;
        self.guidance = None;
        // A parked phase owns guidance through its law; takeover retires
        // it alongside every other automation handle.
        self.phase_park = None;
        self.burn_delegated = false;
        // Manual takeover disengages in-flight executions, same as any
        // other automation (MechJeb-style disengage on stick input).
        self.maneuver_execution = None;
        self.burn_execution = None;
        self.control_mode = ControlMode::Direct;
        self.authority.control_input = DVec3::ZERO;
        self.authority.sas_enabled = false;
        self.authority.stop_propulsion();
    }

    pub(super) fn cancel_autopilot_tasks(&mut self) {
        self.plan_runner = None;
        self.graph_runner = None;
        // The block's one-shot wait parking is only meaningful while its
        // runner is alive. Reset it here so a later graph re-parks its
        // waits instead of completing them immediately from stale state.
        // (Single-pass runners execute each node once; true loop/retry
        // repetition is still a missing Loop combinator, not this set.)
        self.graph_block = NativeGraphBlock::default();
    }

    pub(super) fn fail_autopilot(&mut self, error: impl Into<String>) -> bool {
        let error = error.into();
        self.cancel_autopilot_tasks();
        self.clear_autopilot_controls();
        self.authority.flight_error = Some(error.clone());
        eprintln!("[server] autopilot rejected: {error}");
        true
    }

    pub(super) fn wake_autopilot(
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

    pub(super) fn take_autopilot_events(&mut self) -> Vec<AutopilotEvent> {
        self.autopilot_events.drain(..).collect()
    }

    pub(super) fn next_autopilot_wake(&self, host: &AutopilotHost) -> Option<SimTime> {
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
}
