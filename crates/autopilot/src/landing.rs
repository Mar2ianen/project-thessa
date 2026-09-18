//! MechJeb-class landing guidance as a reusable parameterized subgraph.
//!
//! `LandAt` is UX vocabulary (docs/07 §7.4), not an isolated mode: this
//! module builds a typed [`AutopilotGraph`] out of the standard
//! primitives (sequence edges, event waits, parallel abort branch) plus
//! phase blocks the authority executes. It mirrors [`crate::ascent`] in
//! structure: the graph never touches flight state — phases emit
//! [`GraphControlAction`] values and park on named domain events the
//! authority publishes (trajectory prediction, terrain clearance and
//! touchdown sensing stay on the authority side of the boundary).
//!
//! Phases: deorbit burn → coast to entry interface → braking (suicide)
//! burn → terminal descent → touchdown → landed. A parallel branch
//! watches fuel-low / guidance-deviation / terrain-warning events and
//! aborts to orbit through engine cutoff. The touchdown target is a
//! validated [`LandingSite`]: an [`ImpactSite`] forecast can never be
//! wired into its input without an explicit adapter.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::{
    AutopilotGraph, Bakeability, GraphBlock, GraphEdge, GraphNode, GraphNodeConfig,
    GraphNodeOutcome, GraphValue, LandingSite, NodeId, NodeKind, Port, PortRef, PortType,
    WaitCondition,
};

/// Domain events published by the authority and consumed by the landing
/// subgraph. Names are the cross-crate contract: the graph waits, the
/// authority wakes.
pub mod event {
    /// Deorbit burn complete, committed to entry.
    pub const DEORBIT_COMPLETE: &str = "deorbit-complete";
    /// Entry interface reached, braking program may start.
    pub const ENTRY_INTERFACE: &str = "entry-interface";
    /// Braking burn started (suicide-burn gate opened).
    pub const BRAKING_START: &str = "braking-start";
    /// Terminal descent gate: low altitude, low rate, site in view.
    pub const TOUCHDOWN_APPROACH: &str = "touchdown-approach";
    /// Weight on wheels, touchdown speed within limits.
    pub const TOUCHDOWN: &str = "touchdown";
    /// Propellant reserve hit the divert/abort floor.
    pub const FUEL_LOW: &str = "fuel-low";
    /// Guidance deviation beyond the profile abort gate.
    pub const GUIDANCE_DEVIATION: &str = "guidance-deviation";
    /// Terrain clearance below the profile floor outside the site.
    pub const TERRAIN_WARNING: &str = "terrain-warning";
}

/// Typed landing inputs. Altitudes are above the target-body surface.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LandingProfile {
    /// Touchdown target: validated site with footprint radius.
    pub site: LandingSite,
    /// Touchdown vertical-speed limit (m/s, positive value).
    pub touchdown_speed_limit_mps: f64,
    /// Entry-interface altitude (m): braking program arms here.
    pub entry_altitude_m: f64,
    /// Net braking deceleration (m/s², thrust minus gravity): sizes the
    /// suicide-burn gate. Must be positive — hovering is not landing.
    pub net_braking_decel_mps2: f64,
    /// Throttle for powered phases, `-0.2 .. 1.2` propulsion range.
    pub burn_throttle: f64,
    /// Watchdog: no single phase may run longer than this.
    pub max_phase_time_s: f64,
}

impl LandingProfile {
    pub fn validate(&self) -> Result<(), LandingProfileError> {
        self.site
            .validate()
            .map_err(|_| LandingProfileError::InvalidSite)?;
        let finite = [
            ("touchdown_speed_limit_mps", self.touchdown_speed_limit_mps),
            ("entry_altitude_m", self.entry_altitude_m),
            ("net_braking_decel_mps2", self.net_braking_decel_mps2),
            ("burn_throttle", self.burn_throttle),
            ("max_phase_time_s", self.max_phase_time_s),
        ];
        for (name, value) in finite {
            if !value.is_finite() {
                return Err(LandingProfileError::NonFinite(name));
            }
        }
        if self.touchdown_speed_limit_mps <= 0.0 {
            return Err(LandingProfileError::NonPositive {
                name: "touchdown_speed_limit_mps",
                value: self.touchdown_speed_limit_mps,
            });
        }
        if self.entry_altitude_m <= 0.0 {
            return Err(LandingProfileError::NonPositive {
                name: "entry_altitude_m",
                value: self.entry_altitude_m,
            });
        }
        if self.net_braking_decel_mps2 <= 0.0 {
            return Err(LandingProfileError::NonPositive {
                name: "net_braking_decel_mps2",
                value: self.net_braking_decel_mps2,
            });
        }
        if !(0.0..=1.2).contains(&self.burn_throttle) {
            return Err(LandingProfileError::OutOfRange {
                name: "burn_throttle",
                value: self.burn_throttle,
                min: 0.0,
                max: 1.2,
            });
        }
        if self.max_phase_time_s <= 0.0 {
            return Err(LandingProfileError::NonPositive {
                name: "max_phase_time_s",
                value: self.max_phase_time_s,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LandingProfileError {
    InvalidSite,
    NonFinite(&'static str),
    NonPositive {
        name: &'static str,
        value: f64,
    },
    OutOfRange {
        name: &'static str,
        value: f64,
        min: f64,
        max: f64,
    },
}

impl std::fmt::Display for LandingProfileError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSite => write!(formatter, "landing site is invalid"),
            Self::NonFinite(name) => write!(formatter, "{name} must be finite"),
            Self::NonPositive { name, value } => {
                write!(formatter, "{name} must be positive, got {value}")
            }
            Self::OutOfRange {
                name,
                value,
                min,
                max,
            } => {
                write!(formatter, "{name}={value} is outside [{min}, {max}]")
            }
        }
    }
}

impl std::error::Error for LandingProfileError {}

/// One landing phase executed by the authority. The graph sequences phases
/// with event waits; each phase carries only what the executor needs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LandingPhase {
    /// Retrograde deorbit burn until the entry interface is targeted.
    DeorbitBurn { throttle: f64 },
    /// Unpowered coast to the entry interface.
    CoastToEntry,
    /// Suicide-burn braking at the gate altitude until the terminal gate.
    BrakingBurn,
    /// Final vertical descent to the site at touchdown speed.
    TerminalDescent,
    /// Engine cutoff and safeing after touchdown or an abort trigger.
    AbortToOrbit,
}

/// Braking distance (m) to shed `speed_mps` down to `target_mps` at
/// constant net deceleration: the suicide-burn gate. Returns `None` for
/// non-physical inputs (already slow enough is distance zero, not none).
pub fn braking_distance_m(speed_mps: f64, target_mps: f64, net_decel_mps2: f64) -> Option<f64> {
    if !speed_mps.is_finite() || !target_mps.is_finite() || !net_decel_mps2.is_finite() {
        return None;
    }
    if speed_mps < 0.0 || target_mps < 0.0 || net_decel_mps2 <= 0.0 {
        return None;
    }
    if speed_mps <= target_mps {
        return Some(0.0);
    }
    Some((speed_mps * speed_mps - target_mps * target_mps) / (2.0 * net_decel_mps2))
}

/// Time to impact (s) at constant descent rate. The authority clocks the
/// terminal gate against this; the graph only waits for the event.
pub fn time_to_impact_s(altitude_m: f64, descent_rate_mps: f64) -> Option<f64> {
    if !altitude_m.is_finite() || !descent_rate_mps.is_finite() {
        return None;
    }
    if altitude_m < 0.0 || descent_rate_mps <= 0.0 {
        return None;
    }
    Some(altitude_m / descent_rate_mps)
}

/// Braking-gate predicate for tests and authority guards: braking must
/// start when the remaining altitude reaches the suicide distance plus
/// the profile margin.
pub fn braking_gate_open(
    altitude_m: f64,
    descent_rate_mps: f64,
    profile: &LandingProfile,
    margin_m: f64,
) -> bool {
    if !margin_m.is_finite() || margin_m < 0.0 {
        return false;
    }
    braking_distance_m(
        descent_rate_mps,
        profile.touchdown_speed_limit_mps,
        profile.net_braking_decel_mps2,
    )
    .is_some_and(|distance| altitude_m <= distance + margin_m)
}

#[derive(Debug, Clone, PartialEq)]
pub enum LandingBuildError {
    InvalidProfile(LandingProfileError),
    InvalidGraph(Vec<crate::GraphError>),
}

impl std::fmt::Display for LandingBuildError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProfile(error) => write!(formatter, "invalid landing profile: {error}"),
            Self::InvalidGraph(errors) => {
                write!(formatter, "landing graph failed validation: {errors:?}")
            }
        }
    }
}

impl std::error::Error for LandingBuildError {}

fn source_out(name: &str) -> GraphNode {
    GraphNode {
        id: NodeId(0),
        name: name.into(),
        kind: NodeKind::Source,
        ports: vec![Port::output("out", PortType::Unit)],
        config: None,
    }
}

fn phase_node(id: u32, name: &str, phase: LandingPhase) -> GraphNode {
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
        config: Some(GraphNodeConfig::LandingPhase { phase }),
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

/// Build the landing subgraph for a validated profile. Node ids are fixed
/// so builds are deterministic across calls and machines. The finished
/// graph always validates: construction failure returns the profile or
/// graph error instead of an un-runnable graph.
pub fn landing_graph(profile: &LandingProfile) -> Result<AutopilotGraph, LandingBuildError> {
    profile
        .validate()
        .map_err(LandingBuildError::InvalidProfile)?;
    let mut commit = source_out("commit-landing");
    commit.id = NodeId(0);
    let graph = AutopilotGraph {
        nodes: vec![
            commit,
            phase_node(
                1,
                "deorbit-burn",
                LandingPhase::DeorbitBurn {
                    throttle: profile.burn_throttle,
                },
            ),
            wait_event_node(2, "wait-entry", event::ENTRY_INTERFACE),
            phase_node(3, "braking-burn", LandingPhase::BrakingBurn),
            wait_event_node(4, "wait-terminal", event::TOUCHDOWN_APPROACH),
            phase_node(5, "terminal-descent", LandingPhase::TerminalDescent),
            wait_event_node(6, "wait-touchdown", event::TOUCHDOWN),
            GraphNode {
                id: NodeId(7),
                name: "landed".into(),
                kind: NodeKind::Sink,
                ports: vec![Port::input("in", PortType::Unit, true)],
                config: None,
            },
            // Parallel abort branch: no incoming edges, so it arms
            // alongside the main chain and fires on any trigger.
            GraphNode {
                id: NodeId(8),
                name: "wait-abort-trigger".into(),
                kind: NodeKind::Wait,
                ports: vec![Port::output("out", PortType::Unit)],
                config: Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::Any(vec![
                        WaitCondition::Event(event::FUEL_LOW.into()),
                        WaitCondition::Event(event::GUIDANCE_DEVIATION.into()),
                        WaitCondition::Event(event::TERRAIN_WARNING.into()),
                    ]),
                }),
            },
            phase_node(9, "abort-to-orbit", LandingPhase::AbortToOrbit),
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
    graph.validate().map_err(LandingBuildError::InvalidGraph)?;
    Ok(graph)
}

/// Minimal executor for landing phase blocks in tests and native hosts.
/// Real flight execution lives in the authority; this block proves the
/// subgraph runs end to end on the event contract: phases complete
/// immediately with their control action, waits park on their condition.
///
/// Wait semantics follow the runner contract: the first execution parks
/// the node, the re-execution after the wake condition completes it. The
/// per-node call count is the continuation state the trait provides for.
#[derive(Debug, Default)]
pub struct LandingBlock {
    actions: Vec<crate::GraphControlAction>,
    calls: BTreeMap<NodeId, u32>,
}

impl GraphBlock for LandingBlock {
    fn execute(
        &mut self,
        node: &GraphNode,
        _inputs: &BTreeMap<String, GraphValue>,
    ) -> GraphNodeOutcome {
        let calls = self.calls.entry(node.id).or_default();
        *calls += 1;
        // Wait nodes park on their configured event condition on first
        // execution and complete when the runner wakes them; phases below
        // never wait.
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
        // Sinks consume completion and emit nothing: outputs must match
        // declared output ports, and a sink declares none.
        if node.kind == NodeKind::Sink {
            return GraphNodeOutcome::Complete {
                outputs: BTreeMap::new(),
            };
        }
        match &node.config {
            Some(GraphNodeConfig::LandingPhase { phase }) => {
                use crate::GraphControlAction as Action;
                use thessa_flight_control::{GuidanceIntent, PilotAxes, PropulsionDemand};
                let action = match phase {
                    LandingPhase::DeorbitBurn { throttle } => {
                        let Ok(propulsion) = PropulsionDemand::new(*throttle) else {
                            return GraphNodeOutcome::Fail {
                                diagnostic: crate::Diagnostic {
                                    kind: crate::DiagnosticKind::InvalidState,
                                    code: "landing-throttle".into(),
                                    message: "deorbit throttle out of range".into(),
                                    value: Some(*throttle),
                                    limit: Some(1.2),
                                },
                            };
                        };
                        Action::Guidance {
                            intent: GuidanceIntent::ManualAxes(PilotAxes {
                                propulsion: *throttle,
                                ..PilotAxes::default()
                            }),
                            propulsion,
                        }
                    }
                    LandingPhase::AbortToOrbit => {
                        // Orbit climb already commanded by the trigger
                        // event path; the abort outcome terminates the
                        // whole graph through the runner — a half-aborted
                        // landing is worse than none, and the diagnostic
                        // carries the cause.
                        return GraphNodeOutcome::Abort {
                            diagnostic: crate::Diagnostic {
                                kind: crate::DiagnosticKind::Warning,
                                code: "landing-abort".into(),
                                message: "landing aborted to orbit".into(),
                                value: None,
                                limit: None,
                            },
                        };
                    }
                    LandingPhase::CoastToEntry
                    | LandingPhase::BrakingBurn
                    | LandingPhase::TerminalDescent => Action::Guidance {
                        intent: GuidanceIntent::ManualAxes(PilotAxes::default()),
                        propulsion: PropulsionDemand::new(0.0)
                            .expect("zero propulsion always builds"),
                    },
                };
                self.actions.push(action);
                GraphNodeOutcome::Complete {
                    outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
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

    fn test_site() -> LandingSite {
        LandingSite::new([0.0, 0.0, 1.0], 500.0).expect("valid site")
    }

    fn test_profile() -> LandingProfile {
        LandingProfile {
            site: test_site(),
            touchdown_speed_limit_mps: 2.0,
            entry_altitude_m: 50_000.0,
            net_braking_decel_mps2: 5.0,
            burn_throttle: 1.0,
            max_phase_time_s: 3_600.0,
        }
    }

    #[test]
    fn profile_accepts_a_sane_landing_and_rejects_every_bad_field() {
        test_profile().validate().expect("sane profile validates");
        let bad = |mutate: fn(&mut LandingProfile)| {
            let mut profile = test_profile();
            mutate(&mut profile);
            profile.validate().expect_err("bad profile must fail")
        };
        bad(|p| p.touchdown_speed_limit_mps = 0.0);
        bad(|p| p.touchdown_speed_limit_mps = f64::NAN);
        bad(|p| p.entry_altitude_m = -1.0);
        bad(|p| p.net_braking_decel_mps2 = 0.0);
        bad(|p| p.burn_throttle = 1.5);
        bad(|p| p.max_phase_time_s = f64::INFINITY);
    }

    #[test]
    fn impact_forecast_cannot_pose_as_a_touchdown_target() {
        use crate::{ImpactSite, PortType};
        // The type boundary is structural: an impact site carries a
        // different GraphValue variant than a landing site, so no edge
        // can silently feed a forecast into a touchdown input.
        let impact = ImpactSite::new([0.0, 0.0, 1.0], 500.0).expect("valid forecast");
        let impact_value = GraphValue::ImpactSite(impact);
        assert_eq!(impact_value.port_type(), PortType::ImpactSite);
        assert_ne!(
            impact_value.port_type(),
            GraphValue::LandingSite(test_site()).port_type()
        );
    }

    #[test]
    fn braking_math_gates_the_suicide_burn() {
        // v² law: 100 m/s to 2 m/s at 5 m/s² needs ~996 m.
        let distance = braking_distance_m(100.0, 2.0, 5.0).expect("physical braking distance");
        assert!((distance - 999.6).abs() < 0.1);
        // Already slow enough is zero distance, not an error.
        assert_eq!(braking_distance_m(1.0, 2.0, 5.0), Some(0.0));
        // Non-physical inputs are None, never negative distances.
        assert!(braking_distance_m(100.0, 2.0, 0.0).is_none());
        assert!(braking_distance_m(-5.0, 2.0, 5.0).is_none());
        assert!(braking_distance_m(f64::NAN, 2.0, 5.0).is_none());
        // Time to impact is linear and gated on descent.
        assert_eq!(time_to_impact_s(1_000.0, 10.0), Some(100.0));
        assert!(time_to_impact_s(1_000.0, 0.0).is_none());
        assert!(time_to_impact_s(-1.0, 10.0).is_none());
        // The gate opens exactly at distance plus margin.
        let profile = test_profile();
        assert!(braking_gate_open(999.6 + 50.0, 100.0, &profile, 50.0));
        assert!(!braking_gate_open(999.6 + 50.1, 100.0, &profile, 50.0));
        assert!(!braking_gate_open(500.0, 100.0, &profile, f64::NAN));
    }

    #[test]
    fn landing_graph_validates_deterministic_and_round_trips() {
        let first = landing_graph(&test_profile()).expect("graph builds");
        first.validate().expect("built graph validates");
        let second = landing_graph(&test_profile()).expect("graph rebuilds");
        assert_eq!(first, second);
        // Main chain plus an independent abort branch.
        assert_eq!(first.nodes.len(), 10);
        assert_eq!(first.edges.len(), 8);
        assert!(
            first
                .edges
                .iter()
                .any(|edge| edge.from.node == NodeId(8) && edge.to.node == NodeId(9))
        );
        // Every wait parks on a named domain event from the contract.
        let waits: Vec<&str> = first
            .nodes
            .iter()
            .filter_map(|node| match &node.config {
                Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::Event(name),
                }) => Some(name.as_str()),
                _ => None,
            })
            .collect();
        assert!(waits.contains(&event::ENTRY_INTERFACE));
        assert!(waits.contains(&event::TOUCHDOWN));
        // Serde round trip preserves the build bit-for-bit.
        let json = serde_json::to_string(&first).expect("serializes");
        let back: AutopilotGraph = serde_json::from_str(&json).expect("deserializes");
        assert_eq!(first, back);
        assert!(
            landing_graph(&LandingProfile {
                entry_altitude_m: -5.0,
                ..test_profile()
            })
            .is_err()
        );
    }

    #[test]
    fn landing_block_flies_the_nominal_event_script_to_touchdown() {
        let graph = landing_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = LandingBlock::default();
        // Commit and deorbit complete on their own; entry wait parks.
        let state = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Entry interface -> braking runs -> parks on terminal gate.
        let state = runner
            .poll(SimTime(600.0), Some(event::ENTRY_INTERFACE), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Terminal gate -> descent runs -> parks on touchdown.
        let state = runner
            .poll(SimTime(900.0), Some(event::TOUCHDOWN_APPROACH), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Touchdown -> landed. The abort watcher stays armed: terminal
        // state is Waiting on node 8, a guarded touchdown.
        let state = runner
            .poll(SimTime(1_000.0), Some(event::TOUCHDOWN), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Waiting {
                node: NodeId(8),
                ..
            }
        ));
        assert!(!block.take_control_actions().is_empty());
        assert_eq!(runner.status(NodeId(5)), Some(BlockStatus::Ok));
        assert_eq!(runner.status(NodeId(7)), Some(BlockStatus::Ok));
    }

    #[test]
    fn fuel_low_aborts_to_orbit_through_cutoff() {
        let graph = landing_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = LandingBlock::default();
        let _ = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        let state = runner
            .poll(SimTime(700.0), Some(event::FUEL_LOW), &mut block)
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
