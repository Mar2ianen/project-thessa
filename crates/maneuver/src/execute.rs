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
use thessa_sim_core::{BodyState, SimTime};

use crate::{
    ManeuverNode, ManeuverPlan, PlanError,
    thrust::{BurnSegment, FiniteBurnPlan, SegmentDirection, ThrustPlanError},
};

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
        // Burn phase: integrate what the accelerometer reports ALONG the
        // burn direction. Magnitude would also swallow drag/lift during
        // the transition polls (in-band coast drag alone can exceed a
        // small node); the along-track projection counts thrust minus
        // drag, which is exactly the realized Δv. Clamped per tick:
        // expended Δv never un-accumulates.
        let dt = match self.last_time_s {
            Some(last) => (now.0 - last).max(0.0),
            None => 0.0,
        };
        self.last_time_s = Some(now.0);
        self.accumulated_mps += (measured_accel_inertial_mps2.dot(direction)).max(0.0) * dt;
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

/// Executor state snapshot for arc telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SegmentOutput {
    pub command: ExecutionCommand,
    pub done: bool,
    pub active_segment: Option<usize>,
    /// Accumulated Δv on the active segment (m/s, accelerometer-closed).
    pub accumulated_mps: f64,
    /// Planned-minus-accumulated on the last completed segment (m/s,
    /// positive = underburn). Arcs cut off ON TIME (phasing-critical);
    /// shortfall is reported for the graph's retry/fallback path, never
    /// burned through.
    pub last_shortfall_mps: f64,
}

/// Live flight sample for segment steering: inertial velocity always,
/// plus position and the LVLH reference body for RTN segments (ignored
/// by inertial/prograde/retrograde segments — pass `BodyState::ORIGIN`
/// when the plan has none).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteeringSample {
    pub velocity_inertial_mps: DVec3,
    pub position_inertial_m: DVec3,
    pub central: BodyState,
}

/// Time-schedule segment executor for [`FiniteBurnPlan`]: points per
/// segment (inertial fixed, or velocity-aligned at poll time for
/// prograde/retrograde steering), burns the scheduled throttle over the
/// scheduled window, cuts off on time. Unlike nodes, an underperforming
/// engine does NOT stretch the arc — multi-day phasing would break.
/// Owns no clock and no vehicle.
#[derive(Debug, Clone)]
pub struct SegmentExecutor {
    segments: Vec<BurnSegment>,
    index: usize,
    accumulated_mps: f64,
    last_time_s: Option<f64>,
    last_shortfall_mps: f64,
    last_point: DVec3,
}

impl SegmentExecutor {
    pub fn new(plan: &FiniteBurnPlan) -> Result<Self, ThrustPlanError> {
        Ok(Self {
            segments: plan.segments.clone(),
            index: 0,
            accumulated_mps: 0.0,
            last_time_s: None,
            last_shortfall_mps: 0.0,
            last_point: DVec3::ZERO,
        })
    }

    /// `(start, end, planned Δv)` scheduler windows for every segment.
    pub fn to_scheduler_events(&self) -> Vec<(SimTime, SimTime, f64)> {
        self.segments
            .iter()
            .map(|segment| (segment.start, segment.end(), segment.planned_dv_mps))
            .collect()
    }

    /// Currently active segment (copied): lets the runtime resolve
    /// per-segment data (e.g., the LVLH central body state) before polling.
    pub fn active_segment(&self) -> Option<BurnSegment> {
        self.segments.get(self.index).copied()
    }

    /// Resolve the pointing direction now (velocity-aligned steering needs
    /// the live velocity, RTN needs position plus the central state; a
    /// degenerate sample holds the last attitude instead of erroring
    /// mid-burn).
    fn resolve_point(&mut self, segment: &BurnSegment, sample: &SteeringSample) -> DVec3 {
        let velocity = sample.velocity_inertial_mps;
        let point = match segment.direction {
            SegmentDirection::Inertial(fixed) => fixed,
            SegmentDirection::Prograde => {
                if velocity.length_squared() > 0.0 {
                    velocity.normalize()
                } else {
                    self.last_point
                }
            }
            SegmentDirection::Retrograde => {
                if velocity.length_squared() > 0.0 {
                    -velocity.normalize()
                } else {
                    self.last_point
                }
            }
            SegmentDirection::Rtn {
                central: _,
                radial,
                transverse,
                normal,
            } => {
                let position_rel = sample.position_inertial_m - sample.central.position_inertial;
                let velocity_rel = velocity - sample.central.velocity_inertial;
                match thessa_sim_core::rtn_basis(position_rel, velocity_rel) {
                    Some((basis_r, basis_t, basis_c)) => {
                        basis_r * radial + basis_t * transverse + basis_c * normal
                    }
                    None => self.last_point,
                }
            }
        };
        if point.length_squared() > 0.0 {
            self.last_point = point;
        }
        point
    }

    pub fn poll(
        &mut self,
        now: SimTime,
        measured_accel_inertial_mps2: DVec3,
        sample: &SteeringSample,
    ) -> Result<SegmentOutput, ThrustPlanError> {
        if !now.0.is_finite()
            || !measured_accel_inertial_mps2.is_finite()
            || !sample.velocity_inertial_mps.is_finite()
            || !sample.position_inertial_m.is_finite()
        {
            return Err(ThrustPlanError::InvalidSegment);
        }
        // Skip exhausted and zero-duration segments without burning.
        while self.index < self.segments.len() && self.segments[self.index].duration_s == 0.0 {
            self.index += 1;
        }
        if self.index >= self.segments.len() {
            return Ok(SegmentOutput {
                command: ExecutionCommand {
                    point_inertial: DVec3::ZERO,
                    throttle_01: 0.0,
                },
                done: true,
                active_segment: None,
                accumulated_mps: 0.0,
                last_shortfall_mps: self.last_shortfall_mps,
            });
        }
        let segment = self.segments[self.index];
        let end = segment.end();
        if now.0 >= end.0 {
            // Time cutoff (never Δv-stretched): record shortfall, advance.
            self.last_shortfall_mps = segment.planned_dv_mps - self.accumulated_mps;
            self.index += 1;
            self.accumulated_mps = 0.0;
            self.last_time_s = Some(now.0);
            let finished = self.index >= self.segments.len();
            return Ok(SegmentOutput {
                command: ExecutionCommand {
                    point_inertial: DVec3::ZERO,
                    throttle_01: 0.0,
                },
                done: finished,
                active_segment: if finished { None } else { Some(self.index) },
                accumulated_mps: 0.0,
                last_shortfall_mps: self.last_shortfall_mps,
            });
        }
        let point = self.resolve_point(&segment, sample);
        if now.0 < segment.start.0 - SETTLE_LEAD_S {
            self.last_time_s = Some(now.0);
            return Ok(SegmentOutput {
                command: ExecutionCommand {
                    point_inertial: DVec3::ZERO,
                    throttle_01: 0.0,
                },
                done: false,
                active_segment: Some(self.index),
                accumulated_mps: 0.0,
                last_shortfall_mps: self.last_shortfall_mps,
            });
        }
        if now.0 < segment.start.0 {
            self.last_time_s = Some(now.0);
            return Ok(SegmentOutput {
                command: ExecutionCommand {
                    point_inertial: point,
                    throttle_01: 0.0,
                },
                done: false,
                active_segment: Some(self.index),
                accumulated_mps: 0.0,
                last_shortfall_mps: self.last_shortfall_mps,
            });
        }
        // Burn window: schedule throttle, accumulate along-track.
        let dt = match self.last_time_s {
            Some(last) => (now.0 - last).max(0.0),
            None => 0.0,
        };
        self.last_time_s = Some(now.0);
        if point.length_squared() > 0.0 {
            self.accumulated_mps +=
                (measured_accel_inertial_mps2.dot(point.normalize())).max(0.0) * dt;
        }
        Ok(SegmentOutput {
            command: ExecutionCommand {
                point_inertial: point,
                throttle_01: segment.throttle_01,
            },
            done: false,
            active_segment: Some(self.index),
            accumulated_mps: self.accumulated_mps,
            last_shortfall_mps: self.last_shortfall_mps,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thrust::EngineSpec;
    use thessa_sim_core::{BodyId, BodyState};

    fn cruise_sample() -> SteeringSample {
        SteeringSample {
            velocity_inertial_mps: DVec3::X * 1_000.0,
            position_inertial_m: DVec3::X * 1.1e9,
            central: BodyState::ORIGIN,
        }
    }

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
        assert!((0.0..=2.0).contains(&residual), "residual {residual}");
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

    fn chem_engine() -> EngineSpec {
        EngineSpec {
            thrust_n: 100_000.0,
            exhaust_velocity_mps: 4_400.0,
        }
    }

    fn single_segment_plan() -> FiniteBurnPlan {
        FiniteBurnPlan::new(
            vec![BurnSegment {
                start: SimTime(100.0),
                duration_s: 50.0,
                planned_dv_mps: 100.0,
                direction: SegmentDirection::Inertial(DVec3::X),
                throttle_01: 1.0,
            }],
            chem_engine(),
            20_000.0,
            DVec3::ZERO,
            DVec3::X,
            SimTime(0.0),
        )
        .unwrap()
    }

    /// Timed segment: settle window points, burn window burns at schedule,
    /// cutoff lands exactly on the end tick (never Δv-stretched).
    #[test]
    fn timed_segment_cuts_off_on_schedule() {
        let plan = single_segment_plan();
        let mut executor = SegmentExecutor::new(&plan).unwrap();
        assert_eq!(executor.to_scheduler_events().len(), 1);
        // 2 m/s^2 for 50 s realizes exactly the planned 100 m/s.
        let mut time = 0.0;
        let mut accel = DVec3::ZERO;
        let mut burn_ticks = 0;
        let mut done_at = None;
        for _ in 0..500 {
            let output = executor
                .poll(SimTime(time), accel, &cruise_sample())
                .unwrap();
            if time < 70.0 {
                assert_eq!(output.command.throttle_01, 0.0);
                assert_eq!(output.command.point_inertial, DVec3::ZERO);
            } else if time < 100.0 {
                assert_eq!(output.command.throttle_01, 0.0);
                assert_eq!(output.command.point_inertial, DVec3::X);
            } else if time < 150.0 {
                assert_eq!(output.command.throttle_01, 1.0);
                assert_eq!(output.command.point_inertial, DVec3::X);
                burn_ticks += 1;
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
        assert_eq!(done_at, Some(151.0));
        assert_eq!(burn_ticks, 50);
        // Perfect engine: zero shortfall.
        let shortfall = executor.last_shortfall_mps;
        assert!(shortfall.abs() <= 2.0, "shortfall {shortfall}");
    }

    /// Underperforming engine (half thrust): the arc still cuts off on
    /// time, and the shortfall is reported, not burned through.
    #[test]
    fn underburn_cuts_off_on_time_with_shortfall() {
        let plan = single_segment_plan();
        let mut executor = SegmentExecutor::new(&plan).unwrap();
        let mut time = 0.0;
        let mut accel = DVec3::ZERO;
        loop {
            let output = executor
                .poll(SimTime(time), accel, &cruise_sample())
                .unwrap();
            accel = if output.command.throttle_01 > 0.0 {
                DVec3::X * 1.0
            } else {
                DVec3::ZERO
            };
            time += 1.0;
            if output.done {
                break;
            }
            assert!(time < 500.0, "must finish on schedule");
        }
        // Realized ~50 of planned 100: shortfall ~+50, arc NOT stretched.
        let shortfall = executor.last_shortfall_mps;
        assert!((45.0..=55.0).contains(&shortfall), "shortfall {shortfall}");
    }

    /// Prograde steering follows the live velocity; a degenerate velocity
    /// holds the last attitude instead of erroring mid-burn.
    #[test]
    fn prograde_steering_tracks_velocity() {
        let mut plan = single_segment_plan();
        plan.segments[0].direction = SegmentDirection::Prograde;
        let mut executor = SegmentExecutor::new(&plan).unwrap();
        let mut sample = cruise_sample();
        sample.velocity_inertial_mps = DVec3::Y * 5_000.0;
        let out = executor.poll(SimTime(120.0), DVec3::ZERO, &sample).unwrap();
        assert_eq!(out.command.point_inertial, DVec3::Y);
        assert_eq!(out.command.throttle_01, 1.0);
        // Degenerate velocity: holds +Y, keeps burning on schedule.
        sample.velocity_inertial_mps = DVec3::ZERO;
        let out = executor.poll(SimTime(121.0), DVec3::ZERO, &sample).unwrap();
        assert_eq!(out.command.point_inertial, DVec3::Y);
        assert_eq!(out.command.throttle_01, 1.0);
    }

    /// RTN steering resolves the same basis as the propagation RHS: a pure
    /// in-track component on a circular orbit points prograde; radial
    /// flight (no orbit plane) holds attitude instead of erroring.
    #[test]
    fn rtn_steering_matches_propagation_basis() {
        let mut plan = single_segment_plan();
        plan.segments[0].direction = SegmentDirection::Rtn {
            central: BodyId(0),
            radial: 0.0,
            transverse: 1.0,
            normal: 0.0,
        };
        let mut executor = SegmentExecutor::new(&plan).unwrap();
        // Circular-orbit sample around an origin central body.
        let sample = SteeringSample {
            velocity_inertial_mps: DVec3::Y * 5_000.0,
            position_inertial_m: DVec3::X * 1.1e9,
            central: BodyState::ORIGIN,
        };
        let out = executor.poll(SimTime(120.0), DVec3::ZERO, &sample).unwrap();
        assert_eq!(out.command.point_inertial, DVec3::Y);
        // Radial flight: no plane, hold last (+Y from the previous poll).
        let radial_sample = SteeringSample {
            velocity_inertial_mps: DVec3::X * 5_000.0,
            position_inertial_m: DVec3::X * 1.1e9,
            central: BodyState::ORIGIN,
        };
        let out = executor
            .poll(SimTime(121.0), DVec3::ZERO, &radial_sample)
            .unwrap();
        assert_eq!(out.command.point_inertial, DVec3::Y);
    }
}
