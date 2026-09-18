//! `ExecuteManeuver` as a parameterized subgraph: run the impulsive nodes
//! of a validated maneuver plan in epoch order with per-node ignition
//! waits and burnout verification.
//!
//! The profile carries plain node data (epoch + delta-v), never the plan
//! type itself: `thessa-maneuver` stays below the graph VM (docs/07
//! §7.5), and the authority — which sees both crates — translates a
//! [`ManeuverPlan`](https://docs.rs/) into nodes. Burn execution itself
//! belongs to the authority executor; graph burn phases are markers the
//! executor reads, and completion flows back as `burn-complete` domain
//! events (one generation per wait, retired by the runner's per-wait
//! consumption). Only plans the search exactly revalidated may fly:
//! the builder rejects plans without a measured arrival miss.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{
    AutopilotGraph, Bakeability, GraphBlock, GraphEdge, GraphNode, GraphNodeConfig,
    GraphNodeOutcome, GraphValue, NodeId, NodeKind, Port, PortRef, PortType, WaitCondition,
};

/// Domain events for the execute subgraph. The authority publishes one
/// `burn-complete` generation per burn node; the runner retires each
/// generation at exactly one wait.
pub mod event {
    /// Burn node finished, residual within limits.
    pub const BURN_COMPLETE: &str = "burn-complete";
    /// Whole plan verified after the last node.
    pub const PLAN_VERIFIED: &str = "plan-verified";
    /// Engine-out: thrust lost with nodes remaining.
    pub const ENGINE_OUT: &str = "engine-out";
    /// Guidance deviation beyond the abort gate.
    pub const GUIDANCE_DEVIATION: &str = "guidance-deviation";
}

/// One impulsive node to execute: ignition epoch plus delta-v vector.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExecuteNode {
    pub epoch: thessa_sim_core::SimTime,
    pub delta_v_mps: [f64; 3],
}

impl ExecuteNode {
    pub fn validate(&self) -> Result<(), ExecuteProfileError> {
        if !self.epoch.0.is_finite() {
            return Err(ExecuteProfileError::NonFinite("node epoch"));
        }
        if self.delta_v_mps.iter().any(|v| !v.is_finite()) {
            return Err(ExecuteProfileError::NonFinite("node delta-v"));
        }
        Ok(())
    }
}

/// Typed execute inputs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecuteProfile {
    /// Nodes in ignition order (epochs must be non-decreasing).
    pub nodes: Vec<ExecuteNode>,
    /// Measured arrival miss from exact revalidation. `None` means the
    /// plan was never revalidated — it cannot fly.
    pub predicted_miss_m: Option<f64>,
    /// Watchdog: no single phase may run longer than this.
    pub max_phase_time_s: f64,
}

impl ExecuteProfile {
    pub fn validate(&self) -> Result<(), ExecuteProfileError> {
        if self.nodes.is_empty() {
            return Err(ExecuteProfileError::EmptyPlan);
        }
        if self.nodes.len() > 64 {
            return Err(ExecuteProfileError::TooManyNodes(self.nodes.len()));
        }
        for node in &self.nodes {
            node.validate()?;
        }
        for pair in self.nodes.windows(2) {
            if pair[1].epoch.0 < pair[0].epoch.0 {
                return Err(ExecuteProfileError::UnorderedNodes);
            }
        }
        match self.predicted_miss_m {
            None => return Err(ExecuteProfileError::UnvalidatedPlan),
            Some(miss) if !miss.is_finite() || miss < 0.0 => {
                return Err(ExecuteProfileError::InvalidMiss(miss));
            }
            _ => {}
        }
        if !self.max_phase_time_s.is_finite() || self.max_phase_time_s <= 0.0 {
            return Err(ExecuteProfileError::NonPositive {
                name: "max_phase_time_s",
                value: self.max_phase_time_s,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExecuteProfileError {
    EmptyPlan,
    TooManyNodes(usize),
    UnorderedNodes,
    UnvalidatedPlan,
    InvalidMiss(f64),
    NonFinite(&'static str),
    NonPositive { name: &'static str, value: f64 },
}

impl std::fmt::Display for ExecuteProfileError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyPlan => write!(formatter, "execute plan has no nodes"),
            Self::TooManyNodes(n) => write!(formatter, "execute plan has {n} nodes, more than 64"),
            Self::UnorderedNodes => write!(formatter, "execute nodes must be in ignition order"),
            Self::UnvalidatedPlan => write!(
                formatter,
                "plan was never exactly revalidated (no measured miss)"
            ),
            Self::InvalidMiss(miss) => write!(formatter, "invalid predicted miss {miss}"),
            Self::NonFinite(name) => write!(formatter, "{name} must be finite"),
            Self::NonPositive { name, value } => {
                write!(formatter, "{name} must be positive, got {value}")
            }
        }
    }
}

impl std::error::Error for ExecuteProfileError {}

/// One execute phase read by the authority executor.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ExecutePhase {
    /// Arm node `index` for ignition (valves, gimbal unlock, settle).
    Arm { node: u32 },
    /// Burn node `index` with the profiled delta-v. The authority
    /// executor realizes the impulse; the graph waits for `burn-complete`.
    Burn { node: u32, delta_v_mps: [f64; 3] },
    /// Post-plan verification against the predicted miss.
    Verify,
    /// Engine cutoff and safeing after an abort trigger.
    Abort,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ExecuteBuildError {
    InvalidProfile(ExecuteProfileError),
    InvalidGraph(Vec<crate::GraphError>),
}

impl std::fmt::Display for ExecuteBuildError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProfile(error) => write!(formatter, "invalid execute profile: {error}"),
            Self::InvalidGraph(errors) => {
                write!(formatter, "execute graph failed validation: {errors:?}")
            }
        }
    }
}

impl std::error::Error for ExecuteBuildError {}

fn phase_node(id: u32, name: &str, phase: ExecutePhase) -> GraphNode {
    GraphNode {
        id: NodeId(id),
        name: name.into(),
        kind: NodeKind::Custom {
            bakeability: Bakeability::Live,
            wait_capable: true,
        },
        ports: vec![
            Port::input("in", PortType::Unit, true),
            Port::output("out", PortType::Unit),
        ],
        config: Some(GraphNodeConfig::ExecutePhase { phase }),
    }
}

/// Build the execute subgraph for a validated profile. Node ids pack the
/// per-node chain first (arm/wait/burn/wait repeating), then verify,
/// sink, and the parallel abort branch — deterministic for a given plan.
pub fn execute_graph(profile: &ExecuteProfile) -> Result<AutopilotGraph, ExecuteBuildError> {
    profile
        .validate()
        .map_err(ExecuteBuildError::InvalidProfile)?;
    let mut nodes = vec![GraphNode {
        id: NodeId(0),
        name: "arm-plan".into(),
        kind: NodeKind::Source,
        ports: vec![Port::output("out", PortType::Unit)],
        config: None,
    }];
    let mut edges = Vec::new();
    let mut previous_out = 0u32;
    let mut next_id = 1u32;
    for (index, node) in profile.nodes.iter().enumerate() {
        let arm = next_id;
        next_id += 1;
        let wait_ignition = next_id;
        next_id += 1;
        let burn = next_id;
        next_id += 1;
        let wait_burnout = next_id;
        next_id += 1;
        nodes.push(phase_node(
            arm,
            &format!("arm-node-{index}"),
            ExecutePhase::Arm { node: index as u32 },
        ));
        nodes.push(GraphNode {
            id: NodeId(wait_ignition),
            name: format!("wait-ignition-{index}"),
            kind: NodeKind::Wait,
            ports: vec![
                Port::input("in", PortType::Unit, true),
                Port::output("out", PortType::Unit),
            ],
            config: Some(GraphNodeConfig::Wait {
                condition: WaitCondition::At(node.epoch),
            }),
        });
        nodes.push(phase_node(
            burn,
            &format!("burn-node-{index}"),
            ExecutePhase::Burn {
                node: index as u32,
                delta_v_mps: node.delta_v_mps,
            },
        ));
        nodes.push(GraphNode {
            id: NodeId(wait_burnout),
            name: format!("wait-burnout-{index}"),
            kind: NodeKind::Wait,
            ports: vec![
                Port::input("in", PortType::Unit, true),
                Port::output("out", PortType::Unit),
            ],
            config: Some(GraphNodeConfig::Wait {
                condition: WaitCondition::Event(event::BURN_COMPLETE.into()),
            }),
        });
        for (from, to) in [
            (previous_out, arm),
            (arm, wait_ignition),
            (wait_ignition, burn),
            (burn, wait_burnout),
        ] {
            edges.push(GraphEdge {
                from: PortRef {
                    node: NodeId(from),
                    port: "out".into(),
                },
                to: PortRef {
                    node: NodeId(to),
                    port: "in".into(),
                },
            });
        }
        previous_out = wait_burnout;
    }
    let verify = next_id;
    next_id += 1;
    let sink = next_id;
    next_id += 1;
    let abort_wait = next_id;
    next_id += 1;
    let abort = next_id;
    nodes.push(phase_node(verify, "verify-plan", ExecutePhase::Verify));
    nodes.push(GraphNode {
        id: NodeId(sink),
        name: "plan-executed".into(),
        kind: NodeKind::Sink,
        ports: vec![Port::input("in", PortType::Unit, true)],
        config: None,
    });
    nodes.push(GraphNode {
        id: NodeId(abort_wait),
        name: "wait-abort-trigger".into(),
        kind: NodeKind::Wait,
        ports: vec![Port::output("out", PortType::Unit)],
        config: Some(GraphNodeConfig::Wait {
            condition: WaitCondition::Any(vec![
                WaitCondition::Event(event::ENGINE_OUT.into()),
                WaitCondition::Event(event::GUIDANCE_DEVIATION.into()),
            ]),
        }),
    });
    nodes.push(phase_node(abort, "abort-cutoff", ExecutePhase::Abort));
    edges.push(GraphEdge {
        from: PortRef {
            node: NodeId(previous_out),
            port: "out".into(),
        },
        to: PortRef {
            node: NodeId(verify),
            port: "in".into(),
        },
    });
    edges.push(GraphEdge {
        from: PortRef {
            node: NodeId(verify),
            port: "out".into(),
        },
        to: PortRef {
            node: NodeId(sink),
            port: "in".into(),
        },
    });
    edges.push(GraphEdge {
        from: PortRef {
            node: NodeId(abort_wait),
            port: "out".into(),
        },
        to: PortRef {
            node: NodeId(abort),
            port: "in".into(),
        },
    });
    let graph = AutopilotGraph { nodes, edges };
    graph.validate().map_err(ExecuteBuildError::InvalidGraph)?;
    Ok(graph)
}

/// Minimal executor for execute phase blocks in tests and native hosts.
/// Burn phases complete immediately (the authority executor realizes the
/// impulse); waits follow the runner contract (park first, complete on
/// wake); abort terminates the graph with attribution.
#[derive(Debug, Default)]
pub struct ExecuteBlock {
    calls: BTreeMap<NodeId, u32>,
}

impl GraphBlock for ExecuteBlock {
    fn execute(
        &mut self,
        node: &GraphNode,
        _inputs: &BTreeMap<String, GraphValue>,
    ) -> GraphNodeOutcome {
        let calls = self.calls.entry(node.id).or_default();
        *calls += 1;
        if node.kind == NodeKind::Wait {
            match &node.config {
                Some(GraphNodeConfig::Wait { condition }) if *calls == 1 => {
                    return GraphNodeOutcome::Wait {
                        condition: condition.clone(),
                    };
                }
                _ => {
                    return GraphNodeOutcome::Complete {
                        outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
                    };
                }
            }
        }
        if node.kind == NodeKind::Sink {
            return GraphNodeOutcome::Complete {
                outputs: BTreeMap::new(),
            };
        }
        match &node.config {
            Some(GraphNodeConfig::ExecutePhase { phase }) => match phase {
                ExecutePhase::Abort => GraphNodeOutcome::Abort {
                    diagnostic: crate::Diagnostic {
                        kind: crate::DiagnosticKind::Warning,
                        code: "execute-abort".into(),
                        message: "maneuver execution aborted through engine cutoff".into(),
                        value: None,
                        limit: None,
                    },
                },
                ExecutePhase::Arm { .. } | ExecutePhase::Burn { .. } | ExecutePhase::Verify => {
                    GraphNodeOutcome::Complete {
                        outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
                    }
                }
            },
            _ => GraphNodeOutcome::Complete {
                outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BlockStatus, GraphRunState, GraphRunner};
    use thessa_sim_core::SimTime;

    fn test_profile() -> ExecuteProfile {
        ExecuteProfile {
            nodes: vec![
                ExecuteNode {
                    epoch: SimTime(100.0),
                    delta_v_mps: [100.0, 0.0, 0.0],
                },
                ExecuteNode {
                    epoch: SimTime(500.0),
                    delta_v_mps: [0.0, -50.0, 10.0],
                },
            ],
            predicted_miss_m: Some(1500.0),
            max_phase_time_s: 3_600.0,
        }
    }

    #[test]
    fn profile_demands_a_validated_ordered_plan() {
        test_profile().validate().expect("sane profile validates");
        let bad = |mutate: fn(&mut ExecuteProfile)| {
            let mut profile = test_profile();
            mutate(&mut profile);
            profile.validate().expect_err("bad profile must fail")
        };
        bad(|p| p.nodes.clear());
        bad(|p| p.predicted_miss_m = None);
        bad(|p| p.predicted_miss_m = Some(-1.0));
        bad(|p| p.nodes[1].epoch = SimTime(50.0));
        bad(|p| p.nodes[0].delta_v_mps = [f64::NAN, 0.0, 0.0]);
        bad(|p| p.max_phase_time_s = 0.0);
    }

    #[test]
    fn execute_graph_chains_every_node_then_verifies() {
        let first = execute_graph(&test_profile()).expect("graph builds");
        first.validate().expect("built graph validates");
        let second = execute_graph(&test_profile()).expect("graph rebuilds");
        assert_eq!(first, second);
        // Source + 2 nodes x (arm, wait-ignition, burn, wait-burnout) +
        // verify + sink + abort branch (wait + cutoff).
        assert_eq!(first.nodes.len(), 1 + 2 * 4 + 4);
        assert_eq!(first.edges.len(), 2 * 4 + 3);
        // Ignition waits are absolute-time; burnout waits are per-burn
        // event generations on one shared name.
        let mut ignition_epochs = Vec::new();
        let mut burnout_waits = 0;
        for node in &first.nodes {
            if let Some(GraphNodeConfig::Wait { condition }) = &node.config {
                match condition {
                    WaitCondition::At(time) => ignition_epochs.push(time.0),
                    WaitCondition::Event(name) if name == event::BURN_COMPLETE => {
                        burnout_waits += 1
                    }
                    _ => {}
                }
            }
        }
        assert_eq!(ignition_epochs, vec![100.0, 500.0]);
        assert_eq!(burnout_waits, 2);
        // Serde round trip preserves the build bit-for-bit.
        let json = serde_json::to_string(&first).expect("serializes");
        let back: AutopilotGraph = serde_json::from_str(&json).expect("deserializes");
        assert_eq!(first, back);
    }

    #[test]
    fn execute_block_runs_two_burns_to_verified() {
        let graph = execute_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = ExecuteBlock::default();
        // Arm + ignition wait park on the first epoch.
        let state = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Ignition: burn runs, burnout parks on its event generation.
        let state = runner.poll(SimTime(100.0), None, &mut block).unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // First burnout generation retires exactly one wait; the second
        // node still needs its own ignition time + generation. Note the
        // ignition At-waits also park for one poll each: time alone does
        // not skip the wait boundary.
        let state = runner
            .poll(SimTime(500.0), Some(event::BURN_COMPLETE), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Second ignition time reached, second burn ran, second burnout
        // parks on its own fresh generation.
        let state = runner
            .poll(SimTime(500.0), Some(event::BURN_COMPLETE), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Second burnout generation verifies the plan; the abort watcher
        // stays armed: guarded executed plan, terminal Waiting on 11.
        let state = runner
            .poll(SimTime(600.0), Some(event::BURN_COMPLETE), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Waiting {
                node: NodeId(11),
                ..
            }
        ));
        assert_eq!(runner.status(NodeId(9)), Some(BlockStatus::Ok));
    }

    #[test]
    fn engine_out_aborts_through_cutoff() {
        let graph = execute_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = ExecuteBlock::default();
        let _ = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        let state = runner
            .poll(SimTime(120.0), Some(event::ENGINE_OUT), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Aborted {
                node: Some(NodeId(12)),
                ..
            }
        ));
    }
}
