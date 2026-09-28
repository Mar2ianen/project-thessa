//! Flight phase, burn, maneuver, and trajectory-plan execution.

use super::*;

impl Sim {
    pub(super) fn start_maneuver_execution(&mut self, plan: ManeuverPlan) -> Result<(), String> {
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

    /// Drive the parked autopilot phase for one tick: steer from its IR
    /// parameters, enforce its watchdog, publish completion. Runs before
    /// physics alongside the maneuver executors so commands ride the
    /// freshest state. Rendezvous flies without a target (single vehicle)
    /// as a hold bounded by the watchdog; multi-vehicle targeting is the
    /// missing server input, not a law gap.
    pub(super) fn poll_phase_law(&mut self) -> Result<(), String> {
        use thessa_autopilot::authority as laws;
        let Some(park) = self.phase_park.clone() else {
            return Ok(());
        };
        if let Some(limit) = laws::phase_watchdog_s(&park.config)
            && self.authority.flight_time_s - park.parked_at_s > limit
        {
            self.phase_park = None;
            self.burn_delegated = false;
            self.fail_autopilot(format!(
                "phase {:?} exceeded watchdog {limit:.0}s",
                park.node
            ));
            return Ok(());
        }
        // Execute burns delegate the impulse to the proven NodeExecutor:
        // the executor owns guidance until it clears, then the burn
        // retires through the normal event path.
        if let GraphNodeConfig::ExecutePhase {
            phase: thessa_autopilot::execute::ExecutePhase::Burn { delta_v_mps, .. },
        } = &park.config
        {
            if !self.burn_delegated {
                if self.maneuver_execution.is_some()
                    || self.plan_demand.is_some()
                    || self.burn_execution.is_some()
                {
                    self.phase_park = None;
                    self.fail_autopilot("burn phase conflicts with an active execution");
                    return Ok(());
                }
                let now = SimTime(self.authority.flight_time_s);
                let node = thessa_maneuver::ManeuverNode::new(now, DVec3::from(*delta_v_mps))
                    .map_err(|error| format!("burn phase node: {error}"))?;
                let plan = ManeuverPlan::new(
                    vec![node],
                    self.authority.state.position_inertial_m,
                    self.authority.state.velocity_inertial_mps,
                    now,
                )
                .map_err(|error| format!("burn phase plan: {error}"))?;
                self.maneuver_execution =
                    Some(NodeExecutor::new(&plan).map_err(|error| format!("burn phase: {error}"))?);
                self.burn_delegated = true;
                return Ok(());
            }
            if self.maneuver_execution.is_none() {
                self.burn_delegated = false;
                self.poll_graph(Some(&park.name))?;
                self.poll_graph(Some(thessa_autopilot::execute::event::BURN_COMPLETE))?;
                self.latch_phase_hold();
            }
            return Ok(());
        }
        let body = self
            .ephemeris
            .body(self.authority.reference_body)
            .map_err(|error| format!("phase law reference body: {error}"))?;
        // Laws steer in the body frame: subtract the reference body's
        // inertial motion (feeding inertial state straight in reads
        // interplanetary distances as altitude).
        let now = SimTime(self.authority.flight_time_s);
        let body_state = self
            .ephemeris
            .body_state(self.authority.reference_body, now)
            .map_err(|error| format!("phase law body state: {error}"))?;
        let live = laws::LiveState {
            position_m: self.authority.state.position_inertial_m - body_state.position_inertial,
            velocity_mps: self.authority.state.velocity_inertial_mps - body_state.velocity_inertial,
            orientation_body_to_inertial: self.authority.state.orientation_body_to_inertial,
            body_mu_m3_s2: body.mu,
            body_radius_m: body.radius_m,
            target: None,
        };
        let Some(tick) = laws::phase_tick(&park.config, &live) else {
            return Ok(());
        };
        self.guidance = Some((tick.intent, tick.propulsion));
        if tick.done {
            self.poll_graph(Some(&park.name))?;
            if let Some(event) = laws::completion_event(&park.config) {
                self.poll_graph(Some(event))?;
            }
            self.latch_phase_hold();
        }
        Ok(())
    }

    /// Zero-throttle attitude hold: the safe latch between phases and on
    /// retirement, so no stale full command rides a gap.
    pub(super) fn latch_phase_hold(&mut self) {
        self.guidance = Some((
            GuidanceIntent::Attitude {
                target_body_to_inertial: self.authority.state.orientation_body_to_inertial,
                roll_policy: RollPolicy::Hold,
            },
            PropulsionDemand::new(0.0).expect("zero propulsion always builds"),
        ));
    }

    /// Poll the active execution before stepping: integrate measured thrust
    /// acceleration (total minus gravity, same tick) and map the command to
    /// typed guidance. Idle/done latch a zero-throttle attitude hold, never
    /// an abrupt handoff.
    pub(super) fn poll_maneuver_execution(&mut self) -> Result<(), String> {
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
    pub(super) fn execution_guidance(
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
    pub(super) fn start_burn_execution(&mut self, plan: FiniteBurnPlan) -> Result<(), String> {
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
    pub(super) fn poll_burn_execution(&mut self) -> Result<(), String> {
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
        if let Some(segment) = executor.active_segment()
            && let thessa_maneuver::SegmentDirection::Rtn { central: body, .. } = segment.direction
        {
            central = self
                .ephemeris
                .body_state(body, now)
                .map_err(|error| format!("burn central body: {error}"))?;
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

    pub(super) fn start_plan(&mut self, plan: TrajectoryPlan, host: &mut AutopilotHost) -> bool {
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

    pub(super) fn prepare_plan(&mut self, event: Option<&str>) -> Result<Option<SimTime>, String> {
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

    pub(super) fn poll_plan(&mut self, event: Option<&str>) -> Result<bool, String> {
        self.prepare_plan(event).map(|_| true)
    }

    pub(super) fn apply_plan_action(&mut self, action: PlanAction) -> Result<bool, String> {
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
}
