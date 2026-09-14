//! Node execution: turn a `ManeuverPlan` into timed direction+throttle
//! commands through real propulsion (MechJeb `Node Executor` analog).
//!
//! The executor is a pure state machine over `(plan, time, measured
//! inertial acceleration)`: orient along the burn vector ahead of each
//! node, burn at full throttle, integrate measured acceleration, cut off
//! when the planned Δv is realized, coast to the next node. No teleported
//! velocity anywhere — a finite engine with a perfect accelerometer
//! converges to the plan; an underperforming one just burns longer, and
//! the residual is reported, not hidden.
//!
//! Outputs are plain data (`point_inertial`, `throttle_01`) with explicit
//! frames. Mapping to `GuidanceIntent::VelocityDirection` in the inertial
//! frame plus `PropulsionDemand` is a thin runtime adapter (kept out of
//! this MIT crate so the planning layer never depends on vehicle code).

use glam::DVec3;
use serde::{Deserialize, Serialize};
use thessa_sim_core::SimTime;

use crate::{ManeuverNode, ManeuverPlan, PlanError};

/// Settle time before each node: point first, burn on time.
pub const SETTLE_LEAD_S: f64 = 30.0;

/// One poll's command to the runtime adapter.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExecutionCommand {
    /// Inertial burn direction (unit). Zero when idle/done: hold attitude.
    pub point_inertial: DVec3,
    /// Throttle 0..=1 for the upcoming tick.
    pub throttle_01: f64,
}

/// Executor state snapshot for telemetry / graph-block status ports.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExecutorOutput {
    pub command: ExecutionCommand,
    pub done: bool,
    pub active_node: Option<usize>,
    /// Accumulated Δv on the active node (m/s).
    pub accumulated_mps: f64,
    /// Residual on the last completed node (m/s, signed along-track
    /// overshoot positive). Zero before the first completion.
    pub last_residual_mps: f64,
}

/// Pure node-execution state machine. Owns no clock and no vehicle: the
/// caller feeds sim time and measured inertial acceleration per tick.
#[derive(Debug, Clone)]
pub struct NodeExecutor {
    nodes: Vec<ManeuverNode>,
    index: usize,
    accumulated_mps: f64,
    last_time_s: Option<f64>,
    last_residual_mps: f64,
}

impl NodeExecutor {
    pub fn new(plan: &ManeuverPlan) -> Result<Self, PlanError> {
        Ok(Self {
            nodes: plan.nodes.clone(),
            index: 0,
            accumulated_mps: 0.0,
            last_time_s: None,
            last_residual_mps: 0.0,
        })
    }

    /// `(epoch, delta_v)` pairs to arm as scheduler `ManeuverNode` wakes.
    pub fn to_scheduler_events(&self) -> Vec<(SimTime, DVec3)> {
        self.nodes
            .iter()
            .map(|node| (node.epoch, node.delta_v_mps))
            .collect()
    }

    pub fn poll(
        &mut self,
        now: SimTime,
        measured_accel_inertial_mps2: DVec3,
    ) -> Result<ExecutorOutput, PlanError> {
        if !now.0.is_finite() || !measured_accel_inertial_mps2.is_finite() {
            return Err(PlanError::NonFiniteNode);
        }
        // Skip exhausted and zero-magnitude nodes without burning.
        while self.index < self.nodes.len() && self.nodes[self.index].magnitude_mps() == 0.0 {
            self.index += 1;
        }
        if self.index >= self.nodes.len() {
            return Ok(ExecutorOutput {
                command: ExecutionCommand {
                    point_inertial: DVec3::ZERO,
                    throttle_01: 0.0,
                },
                done: true,
                active_node: None,
                accumulated_mps: 0.0,
                last_residual_mps: self.last_residual_mps,
            });
        }
        let node = self.nodes[self.index];
        let target = node.magnitude_mps();
        let direction = node.delta_v_mps / target;
        if now.0 < node.epoch.0 - SETTLE_LEAD_S {
            // Far future: idle, hold whatever attitude the graph commands.
            self.last_time_s = Some(now.0);
            return Ok(ExecutorOutput {
                command: ExecutionCommand {
                    point_inertial: DVec3::ZERO,
                    throttle_01: 0.0,
                },
                done: false,
                active_node: Some(self.index),
                accumulated_mps: 0.0,
                last_residual_mps: self.last_residual_mps,
            });
        }
        if now.0 < node.epoch.0 {
            // Settle window: point along the burn, do not burn yet. The
            // burn starts at the node epoch (not centered: centering needs
            // thrust feedforward the executor deliberately does not assume).
            self.last_time_s = Some(now.0);
            return Ok(ExecutorOutput {
                command: ExecutionCommand {
                    point_inertial: direction,
                    throttle_01: 0.0,
                },
                done: false,
                active_node: Some(self.index),
                accumulated_mps: 0.0,
                last_residual_mps: self.last_residual_mps,
            });
        }
        // Burn phase: integrate what the accelerometer reports.
        let dt = match self.last_time_s {
            Some(last) => (now.0 - last).max(0.0),
            None => 0.0,
        };
        self.last_time_s = Some(now.0);
        self.accumulated_mps += measured_accel_inertial_mps2.length() * dt;
        if self.accumulated_mps >= target {
            self.last_residual_mps = self.accumulated_mps - target;
            self.index += 1;
            self.accumulated_mps = 0.0;
            let finished = self.index >= self.nodes.len();
            return Ok(ExecutorOutput {
                command: ExecutionCommand {
                    point_inertial: if finished { DVec3::ZERO } else { direction },
                    throttle_01: 0.0,
                },
                done: finished,
                active_node: if finished { None } else { Some(self.index) },
                accumulated_mps: 0.0,
                last_residual_mps: self.last_residual_mps,
            });
        }
        Ok(ExecutorOutput {
            command: ExecutionCommand {
                point_inertial: direction,
                throttle_01: 1.0,
            },
            done: false,
            active_node: Some(self.index),
            accumulated_mps: self.accumulated_mps,
            last_residual_mps: self.last_residual_mps,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn single_node_plan() -> ManeuverPlan {
        ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(100.0), DVec3::new(100.0, 0.0, 0.0)).unwrap()],
            DVec3::ZERO,
            DVec3::X,
            SimTime(0.0),
        )
        .unwrap()
    }

    /// Perfect engine (2 m/s^2 along the burn vector while throttle is up):
    /// cutoff must land within one tick of overshoot, residual reported.
    #[test]
    fn perfect_burn_cuts_off_on_target() {
        let plan = single_node_plan();
        let mut executor = NodeExecutor::new(&plan).unwrap();
        let events = executor.to_scheduler_events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, SimTime(100.0));
        let mut time = 0.0;
        let mut done_at = None;
        let mut accel = DVec3::ZERO;
        for _ in 0..500 {
            let output = executor.poll(SimTime(time), accel).unwrap();
            if time < 70.0 {
                // Before the settle window: idle.
                assert_eq!(output.command.throttle_01, 0.0);
                assert_eq!(output.command.point_inertial, DVec3::ZERO);
            } else if time < 100.0 {
                // Settle window: pointed, not burning.
                assert_eq!(output.command.throttle_01, 0.0);
                assert_eq!(output.command.point_inertial, DVec3::X);
            }
            accel = if output.command.throttle_01 > 0.0 {
                DVec3::X * 2.0
            } else {
                DVec3::ZERO
            };
            time += 1.0;
            if output.done {
                done_at = Some(time);
                break;
            }
        }
        // 100 m/s at 2 m/s^2 starting at t=100: cutoff within one tick.
        let done_at = done_at.expect("burn completes");
        assert!((done_at - 151.0).abs() <= 2.0, "cutoff at {done_at}");
        let residual = executor.last_residual_mps;
        assert!(residual >= 0.0 && residual <= 2.0, "residual {residual}");
    }

    #[test]
    fn two_nodes_sequence_in_order() {
        let plan = ManeuverPlan::new(
            vec![
                ManeuverNode::new(SimTime(10.0), DVec3::X * 20.0).unwrap(),
                ManeuverNode::new(SimTime(100.0), DVec3::Y * 30.0).unwrap(),
            ],
            DVec3::ZERO,
            DVec3::X,
            SimTime(0.0),
        )
        .unwrap();
        let mut executor = NodeExecutor::new(&plan).unwrap();
        // Fast engine: 10 m/s^2; node 1 (20 m/s) burns t=10..12.
        let mut time = 0.0;
        let mut accel = DVec3::ZERO;
        let mut saw_node_two = false;
        for _ in 0..500 {
            let output = executor.poll(SimTime(time), accel).unwrap();
            if output.active_node == Some(1) {
                saw_node_two = true;
                // The cutoff tick itself may still point along the finished
                // burn with throttle already at zero; after that it tracks.
                if output.command.throttle_01 > 0.0 || time >= 70.0 {
                    assert_eq!(output.command.point_inertial, DVec3::Y);
                }
            }
            accel = if output.command.throttle_01 > 0.0 {
                output.command.point_inertial * 10.0
            } else {
                DVec3::ZERO
            };
            time += 1.0;
            if output.done {
                break;
            }
        }
        assert!(saw_node_two, "second node must activate");
        assert!(executor.poll(SimTime(time), DVec3::ZERO).unwrap().done);
    }

    #[test]
    fn empty_plan_is_immediately_done() {
        let plan = ManeuverPlan::new(vec![], DVec3::ZERO, DVec3::X, SimTime(0.0)).unwrap();
        let mut executor = NodeExecutor::new(&plan).unwrap();
        let output = executor.poll(SimTime(0.0), DVec3::ZERO).unwrap();
        assert!(output.done);
        assert_eq!(output.command.throttle_01, 0.0);
    }
}
