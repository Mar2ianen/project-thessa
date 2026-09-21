//! Rendezvous approach as a parameterized subgraph: close with a target,
//! null the relative velocity, and hold station — stopping before any
//! contact. Docking is deliberately out of scope: the project has no
//! docking ports (no capture hardware, no docked topology, no docked
//! rigid-body compounding), so no graph here may promise a `Dock` node.
//! When ports exist, a contact-gated dock phase will extend the hold this
//! module ends at; until then the terminal sink is a guarded hold.
//!
//! Like the sibling subgraphs, this module builds typed IR out of the
//! standard primitives plus authority-executed phases, and proves the
//! event script with a native test block.

use std::collections::BTreeMap;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    AutopilotGraph, Bakeability, GraphBlock, GraphEdge, GraphNode, GraphNodeConfig,
    GraphNodeOutcome, GraphValue, NodeId, NodeKind, Port, PortRef, PortType, WaitCondition,
};

/// Domain events for the rendezvous subgraph.
pub mod event {
    /// Hold point inside the approach sphere reached.
    pub const PROXIMITY: &str = "proximity";
    /// Relative velocity nulled within tolerance.
    pub const VELOCITY_MATCHED: &str = "velocity-matched";
    /// Station-keeping hold stable for the profile dwell.
    pub const KEEP_STABLE: &str = "keep-stable";
    /// Collision warning: predicted miss below the safety floor.
    pub const COLLISION_WARNING: &str = "collision-warning";
    /// Guidance deviation beyond the profile abort gate.
    pub const GUIDANCE_DEVIATION: &str = "guidance-deviation";
}

/// Typed rendezvous-approach inputs. Distances are target-relative.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RendezvousProfile {
    /// Hold distance (m) for proximity operations. Must be positive —
    /// zero is contact, and contact needs docking ports we do not have.
    pub hold_distance_m: f64,
    /// Closing-rate limit (m/s) inside the approach sphere.
    pub closing_rate_limit_mps: f64,
    /// Relative-velocity tolerance (m/s) for the matched gate.
    pub match_tolerance_mps: f64,
    /// Watchdog: no single phase may run longer than this.
    pub max_phase_time_s: f64,
}

impl RendezvousProfile {
    pub fn validate(&self) -> Result<(), RendezvousProfileError> {
        let finite = [
            ("hold_distance_m", self.hold_distance_m),
            ("closing_rate_limit_mps", self.closing_rate_limit_mps),
            ("match_tolerance_mps", self.match_tolerance_mps),
            ("max_phase_time_s", self.max_phase_time_s),
        ];
        for (name, value) in finite {
            if !value.is_finite() {
                return Err(RendezvousProfileError::NonFinite(name));
            }
        }
        if self.hold_distance_m <= 0.0 {
            return Err(RendezvousProfileError::NonPositive {
                name: "hold_distance_m",
                value: self.hold_distance_m,
            });
        }
        if self.closing_rate_limit_mps <= 0.0 {
            return Err(RendezvousProfileError::NonPositive {
                name: "closing_rate_limit_mps",
                value: self.closing_rate_limit_mps,
            });
        }
        if self.match_tolerance_mps <= 0.0 {
            return Err(RendezvousProfileError::NonPositive {
                name: "match_tolerance_mps",
                value: self.match_tolerance_mps,
            });
        }
        if self.max_phase_time_s <= 0.0 {
            return Err(RendezvousProfileError::NonPositive {
                name: "max_phase_time_s",
                value: self.max_phase_time_s,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RendezvousProfileError {
    NonFinite(&'static str),
    NonPositive { name: &'static str, value: f64 },
}

impl std::fmt::Display for RendezvousProfileError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite(name) => write!(formatter, "{name} must be finite"),
            Self::NonPositive { name, value } => {
                write!(formatter, "{name} must be positive, got {value}")
            }
        }
    }
}

impl std::error::Error for RendezvousProfileError {}

/// One rendezvous phase executed by the authority. Each variant carries the
/// profile parameters its executor needs — the executable IR is
/// self-contained, so two different profiles can never build the same
/// graph. `max_phase_time_s` rides every phase: the watchdog binds the
/// executor, not the planner.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum RendezvousPhase {
    /// Close to the hold point within the closing-rate limit.
    Approach {
        hold_distance_m: f64,
        closing_rate_limit_mps: f64,
        max_phase_time_s: f64,
    },
    /// Null relative velocity against the target.
    MatchVelocity {
        match_tolerance_mps: f64,
        max_phase_time_s: f64,
    },
    /// Hold station at the hold point for the dwell.
    StationKeep {
        hold_distance_m: f64,
        match_tolerance_mps: f64,
        max_phase_time_s: f64,
    },
    /// Back away along the approach corridor after an abort trigger.
    /// Done at a multiple of the hold distance (no docking ports exist,
    /// so "away" needs an explicit range to retire against).
    BackAway {
        hold_distance_m: f64,
        max_phase_time_s: f64,
    },
}

/// Velocity-match gate for tests and authority guards: relative speed at
/// or below tolerance.
pub fn velocity_matched(
    current_velocity_mps: DVec3,
    target_velocity_mps: DVec3,
    tolerance_mps: f64,
) -> bool {
    if !tolerance_mps.is_finite() || tolerance_mps < 0.0 {
        return false;
    }
    (current_velocity_mps - target_velocity_mps).length() <= tolerance_mps
}

/// Approach gate: inside the hold sphere and closing (or holding) no
/// faster than the profile limit. Range rate is negative on closing: an
/// opening drift fails however slow, and a -100 m/s plunge fails a 2 m/s
/// limit that the old `rate <= +limit` comparison let straight through.
pub fn approach_gate_ok(range_m: f64, range_rate_mps: f64, profile: &RendezvousProfile) -> bool {
    approach_gate_ok_params(
        range_m,
        range_rate_mps,
        profile.hold_distance_m,
        profile.closing_rate_limit_mps,
    )
}

/// Scalar form of [`approach_gate_ok`] so phase laws can evaluate the gate
/// from node parameters without reconstructing a profile.
pub fn approach_gate_ok_params(
    range_m: f64,
    range_rate_mps: f64,
    hold_distance_m: f64,
    closing_rate_limit_mps: f64,
) -> bool {
    if !range_m.is_finite() || !range_rate_mps.is_finite() {
        return false;
    }
    range_m <= hold_distance_m && range_rate_mps <= 0.0 && (-range_rate_mps) <= closing_rate_limit_mps
}

#[derive(Debug, Clone, PartialEq)]
pub enum RendezvousBuildError {
    InvalidProfile(RendezvousProfileError),
    InvalidGraph(Vec<crate::GraphError>),
}

impl std::fmt::Display for RendezvousBuildError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProfile(error) => {
                write!(formatter, "invalid rendezvous profile: {error}")
            }
            Self::InvalidGraph(errors) => {
                write!(formatter, "rendezvous graph failed validation: {errors:?}")
            }
        }
    }
}

impl std::error::Error for RendezvousBuildError {}

fn phase_node(id: u32, name: &str, phase: RendezvousPhase) -> GraphNode {
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
        config: Some(GraphNodeConfig::RendezvousPhase { phase }),
    }
}

fn wait_event_node(id: u32, name: &str, event_name: &str) -> GraphNode {
    GraphNode {
        id: NodeId(id),
        name: name.into(),
        kind: NodeKind::Wait,
        ports: vec![
            Port::input("in", PortType::Unit, true),
            Port::output("out", PortType::Unit),
        ],
        config: Some(GraphNodeConfig::Wait {
            condition: WaitCondition::Event(event_name.into()),
        }),
    }
}

fn edge(from: u32, to: u32) -> GraphEdge {
    GraphEdge {
        from: PortRef {
            node: NodeId(from),
            port: "out".into(),
        },
        to: PortRef {
            node: NodeId(to),
            port: "in".into(),
        },
    }
}

/// Build the rendezvous-approach subgraph for a validated profile. The
/// terminal sink is a guarded hold — never a dock: see the module docs.
pub fn rendezvous_graph(
    profile: &RendezvousProfile,
) -> Result<AutopilotGraph, RendezvousBuildError> {
    profile
        .validate()
        .map_err(RendezvousBuildError::InvalidProfile)?;
    let graph = AutopilotGraph {
        nodes: vec![
            GraphNode {
                id: NodeId(0),
                name: "commit-rendezvous".into(),
                kind: NodeKind::Source,
                ports: vec![Port::output("out", PortType::Unit)],
                config: None,
            },
            phase_node(
                1,
                "approach",
                RendezvousPhase::Approach {
                    hold_distance_m: profile.hold_distance_m,
                    closing_rate_limit_mps: profile.closing_rate_limit_mps,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(2, "wait-proximity", event::PROXIMITY),
            phase_node(
                3,
                "match-velocity",
                RendezvousPhase::MatchVelocity {
                    match_tolerance_mps: profile.match_tolerance_mps,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(4, "wait-matched", event::VELOCITY_MATCHED),
            phase_node(
                5,
                "station-keep",
                RendezvousPhase::StationKeep {
                    hold_distance_m: profile.hold_distance_m,
                    match_tolerance_mps: profile.match_tolerance_mps,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(6, "wait-stable", event::KEEP_STABLE),
            GraphNode {
                id: NodeId(7),
                name: "rendezvous-hold".into(),
                kind: NodeKind::Sink,
                ports: vec![Port::input("in", PortType::Unit, true)],
                config: None,
            },
            // Parallel abort branch: collision warning or deviation backs
            // the chaser away along the corridor, then terminates.
            GraphNode {
                id: NodeId(8),
                name: "wait-abort-trigger".into(),
                kind: NodeKind::Wait,
                ports: vec![Port::output("out", PortType::Unit)],
                config: Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::Any(vec![
                        WaitCondition::Event(event::COLLISION_WARNING.into()),
                        WaitCondition::Event(event::GUIDANCE_DEVIATION.into()),
                    ]),
                }),
            },
            phase_node(
                9,
                "back-away",
                RendezvousPhase::BackAway {
                    hold_distance_m: profile.hold_distance_m,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
        ],
        edges: vec![
            edge(0, 1),
            edge(1, 2),
            edge(2, 3),
            edge(3, 4),
            edge(4, 5),
            edge(5, 6),
            edge(6, 7),
            edge(8, 9),
        ],
    };
    graph
        .validate()
        .map_err(RendezvousBuildError::InvalidGraph)?;
    Ok(graph)
}

/// Minimal executor for rendezvous phase blocks in tests and native
/// hosts. Approach/match/keep phases complete with a neutral guidance
/// action (the authority steers against live relative state); waits
/// follow the runner contract; back-away terminates with attribution.
#[derive(Debug, Default)]
pub struct RendezvousBlock {
    actions: Vec<crate::GraphControlAction>,
    calls: BTreeMap<NodeId, u32>,
}

impl GraphBlock for RendezvousBlock {
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
            Some(GraphNodeConfig::RendezvousPhase { phase }) => {
                use crate::GraphControlAction as Action;
                use thessa_flight_control::{GuidanceIntent, PilotAxes, PropulsionDemand};
                match phase {
                    RendezvousPhase::BackAway { .. } => GraphNodeOutcome::Abort {
                        diagnostic: crate::Diagnostic {
                            kind: crate::DiagnosticKind::Warning,
                            code: "rendezvous-abort".into(),
                            message: "rendezvous backed away along the corridor".into(),
                            value: None,
                            limit: None,
                        },
                    },
                    RendezvousPhase::Approach { .. }
                    | RendezvousPhase::MatchVelocity { .. }
                    | RendezvousPhase::StationKeep { .. } => {
                        self.actions.push(Action::Guidance {
                            intent: GuidanceIntent::ManualAxes(PilotAxes::default()),
                            propulsion: PropulsionDemand::new(0.0)
                                .expect("zero propulsion always builds"),
                        });
                        GraphNodeOutcome::Complete {
                            outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
                        }
                    }
                }
            }
            _ => GraphNodeOutcome::Complete {
                outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
            },
        }
    }

    fn take_control_actions(&mut self) -> Vec<crate::GraphControlAction> {
        std::mem::take(&mut self.actions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BlockStatus, GraphRunState, GraphRunner};
    use thessa_sim_core::SimTime;

    fn test_profile() -> RendezvousProfile {
        RendezvousProfile {
            hold_distance_m: 100.0,
            closing_rate_limit_mps: 2.0,
            match_tolerance_mps: 0.1,
            max_phase_time_s: 3_600.0,
        }
    }

    #[test]
    fn profile_accepts_a_sane_approach_and_rejects_every_bad_field() {
        test_profile().validate().expect("sane profile validates");
        let bad = |mutate: fn(&mut RendezvousProfile)| {
            let mut profile = test_profile();
            mutate(&mut profile);
            profile.validate().expect_err("bad profile must fail")
        };
        // Zero hold is contact, and contact needs docking ports we do
        // not have — the profile refuses it structurally.
        bad(|p| p.hold_distance_m = 0.0);
        bad(|p| p.hold_distance_m = f64::NAN);
        bad(|p| p.closing_rate_limit_mps = -1.0);
        bad(|p| p.match_tolerance_mps = 0.0);
        bad(|p| p.max_phase_time_s = f64::INFINITY);
    }

    #[test]
    fn gates_open_only_inside_safe_corridors() {
        use glam::DVec3;
        // Matched within tolerance, open beyond it.
        assert!(velocity_matched(
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(1.05, 0.0, 0.0),
            0.1
        ));
        assert!(!velocity_matched(
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(1.5, 0.0, 0.0),
            0.1
        ));
        assert!(!velocity_matched(
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            -0.1
        ));
        // Approach gate: negative range rate closes, positive opens.
        let profile = test_profile();
        assert!(approach_gate_ok(50.0, -1.0, &profile));
        assert!(approach_gate_ok(50.0, 0.0, &profile));
        assert!(!approach_gate_ok(50.0, 1.0, &profile));
        assert!(!approach_gate_ok(150.0, -1.0, &profile));
        assert!(!approach_gate_ok(50.0, -5.0, &profile));
        assert!(!approach_gate_ok(50.0, -100.0, &profile));
        assert!(!approach_gate_ok(f64::NAN, -1.0, &profile));
    }

    #[test]
    fn distinct_profiles_build_distinct_executable_graphs() {
        let base = rendezvous_graph(&test_profile()).expect("graph builds");
        let other = rendezvous_graph(&RendezvousProfile {
            closing_rate_limit_mps: 0.5,
            max_phase_time_s: 1_800.0,
            ..test_profile()
        })
        .expect("graph builds");
        assert_ne!(base, other);
        let approach = |graph: &AutopilotGraph| {
            graph
                .nodes
                .iter()
                .find(|node| node.name == "approach")
                .expect("approach node exists")
                .config
                .clone()
                .expect("approach node is configured")
        };
        assert_ne!(approach(&base), approach(&other));
    }

    #[test]
    fn rendezvous_graph_ends_at_a_hold_never_a_dock() {
        let first = rendezvous_graph(&test_profile()).expect("graph builds");
        first.validate().expect("built graph validates");
        let second = rendezvous_graph(&test_profile()).expect("graph rebuilds");
        assert_eq!(first, second);
        // Main chain plus an independent abort branch.
        assert_eq!(first.nodes.len(), 10);
        assert_eq!(first.edges.len(), 8);
        // The terminal sink is a hold: no dock node, no dock edge, no
        // dock event anywhere in the vocabulary this graph speaks.
        let names: Vec<&str> = first.nodes.iter().map(|node| node.name.as_str()).collect();
        assert!(names.contains(&"rendezvous-hold"));
        assert!(!names.iter().any(|name| name.contains("dock")));
        // Serde round trip preserves the build bit-for-bit.
        let json = serde_json::to_string(&first).expect("serializes");
        let back: AutopilotGraph = serde_json::from_str(&json).expect("deserializes");
        assert_eq!(first, back);
        assert!(
            rendezvous_graph(&RendezvousProfile {
                hold_distance_m: 0.0,
                ..test_profile()
            })
            .is_err()
        );
    }

    #[test]
    fn rendezvous_block_flies_the_nominal_script_to_hold() {
        let graph = rendezvous_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = RendezvousBlock::default();
        // Commit and approach complete on their own; proximity parks.
        let state = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Proximity -> match burn -> parks on matched.
        let state = runner
            .poll(SimTime(300.0), Some(event::PROXIMITY), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Matched -> keep -> parks on stable.
        let state = runner
            .poll(SimTime(600.0), Some(event::VELOCITY_MATCHED), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Stable -> hold achieved. The abort watcher stays armed: guarded
        // hold, terminal Waiting on node 8.
        let state = runner
            .poll(SimTime(900.0), Some(event::KEEP_STABLE), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Waiting {
                node: NodeId(8),
                ..
            }
        ));
        assert!(!block.take_control_actions().is_empty());
        assert_eq!(runner.status(NodeId(7)), Some(BlockStatus::Ok));
    }

    #[test]
    fn collision_warning_backs_away_with_attribution() {
        let graph = rendezvous_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = RendezvousBlock::default();
        let _ = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        let state = runner
            .poll(SimTime(350.0), Some(event::COLLISION_WARNING), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Aborted {
                node: Some(NodeId(9)),
                ..
            }
        ));
    }
}
