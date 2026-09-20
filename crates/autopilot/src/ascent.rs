//! MechJeb-class ascent guidance as a reusable parameterized subgraph.
//!
//! `Ascent` is UX vocabulary (docs/07 §7.4), not an isolated mode: this
//! module builds a typed [`AutopilotGraph`] out of the standard
//! primitives (sequence edges, event waits, parallel abort branch) plus
//! phase blocks the authority executes. The graph never touches flight
//! state — phases emit [`GraphControlAction`] values and park on named
//! domain events the authority publishes (telemetry stays on the
//! authority side of the boundary).
//!
//! Phases: vertical rise → gravity turn (pitch program) → MECO at target
//! apoapsis → coast → circularization burn → orbit achieved. A parallel
//! branch watches engine-out / guidance-deviation events and aborts
//! through engine cutoff. Staging rides the same event mechanism the
//! survey bridge already uses: the authority publishes `stage-N-sep`,
//! the graph routes booster branches from there (docs/02 §2.8).

use std::collections::BTreeMap;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    AutopilotGraph, Bakeability, GraphBlock, GraphEdge, GraphNode, GraphNodeConfig,
    GraphNodeOutcome, GraphValue, NodeId, NodeKind, Port, PortRef, PortType, WaitCondition,
};

/// Domain events published by the authority and consumed by the ascent
/// subgraph. Names are the cross-crate contract: the graph waits, the
/// authority wakes.
pub mod event {
    /// Liftoff commit (hold-down release, throttle up).
    pub const LIFTOFF: &str = "liftoff";
    /// Tower/launch-mount cleared, safe to pitch over.
    pub const TOWER_CLEARED: &str = "tower-cleared";
    /// Gravity-turn program engaged.
    pub const TURN_START: &str = "turn-start";
    /// Main-engine cutoff: predicted apoapsis reached the target.
    pub const MECO: &str = "meco";
    /// Coast arc approaching apoapsis, circularization imminent.
    pub const APOAPSIS_APPROACH: &str = "apoapsis-approach";
    /// Circularization burn complete, target orbit achieved.
    pub const CIRC_COMPLETE: &str = "circ-complete";
    /// Engine-out: thrust lost while phases remain.
    pub const ENGINE_OUT: &str = "engine-out";
    /// Guidance deviation beyond the profile abort gate.
    pub const GUIDANCE_DEVIATION: &str = "guidance-deviation";
}

/// Typed ascent inputs. Altitudes are above the launch-body surface;
/// apoapsis/periapsis are radii-class targets the authority resolves
/// against the departure well.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AscentProfile {
    /// Target apoapsis altitude (m) at MECO.
    pub target_apoapsis_m: f64,
    /// Target periapsis altitude (m) after circularization.
    pub target_periapsis_m: f64,
    /// End of the vertical segment: pitchover starts here.
    pub turn_start_altitude_m: f64,
    /// Pitch program reaches horizontal here (below MECO altitude).
    pub turn_end_altitude_m: f64,
    /// Liftoff throttle in the documented `-0.2 .. 1.2` propulsion range.
    pub liftoff_throttle: f64,
    /// Watchdog: no single phase may run longer than this.
    pub max_phase_time_s: f64,
}

impl AscentProfile {
    pub fn validate(&self) -> Result<(), AscentProfileError> {
        let finite = [
            ("target_apoapsis_m", self.target_apoapsis_m),
            ("target_periapsis_m", self.target_periapsis_m),
            ("turn_start_altitude_m", self.turn_start_altitude_m),
            ("turn_end_altitude_m", self.turn_end_altitude_m),
            ("liftoff_throttle", self.liftoff_throttle),
            ("max_phase_time_s", self.max_phase_time_s),
        ];
        for (name, value) in finite {
            if !value.is_finite() {
                return Err(AscentProfileError::NonFinite(name));
            }
        }
        if self.target_apoapsis_m <= 0.0 {
            return Err(AscentProfileError::NonPositive {
                name: "target_apoapsis_m",
                value: self.target_apoapsis_m,
            });
        }
        if self.target_periapsis_m <= 0.0 {
            return Err(AscentProfileError::NonPositive {
                name: "target_periapsis_m",
                value: self.target_periapsis_m,
            });
        }
        if self.target_periapsis_m > self.target_apoapsis_m {
            return Err(AscentProfileError::PeriapsisAboveApoapsis {
                periapsis: self.target_periapsis_m,
                apoapsis: self.target_apoapsis_m,
            });
        }
        if self.turn_start_altitude_m < 0.0 {
            return Err(AscentProfileError::NonPositive {
                name: "turn_start_altitude_m",
                value: self.turn_start_altitude_m,
            });
        }
        if self.turn_end_altitude_m <= self.turn_start_altitude_m {
            return Err(AscentProfileError::TurnOrder {
                start: self.turn_start_altitude_m,
                end: self.turn_end_altitude_m,
            });
        }
        if self.turn_end_altitude_m >= self.target_apoapsis_m {
            return Err(AscentProfileError::TurnEndsPastApoapsis {
                turn_end: self.turn_end_altitude_m,
                apoapsis: self.target_apoapsis_m,
            });
        }
        if !(0.0..=1.2).contains(&self.liftoff_throttle) {
            return Err(AscentProfileError::OutOfRange {
                name: "liftoff_throttle",
                value: self.liftoff_throttle,
                min: 0.0,
                max: 1.2,
            });
        }
        if self.max_phase_time_s <= 0.0 {
            return Err(AscentProfileError::NonPositive {
                name: "max_phase_time_s",
                value: self.max_phase_time_s,
            });
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AscentProfileError {
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
    PeriapsisAboveApoapsis {
        periapsis: f64,
        apoapsis: f64,
    },
    TurnOrder {
        start: f64,
        end: f64,
    },
    TurnEndsPastApoapsis {
        turn_end: f64,
        apoapsis: f64,
    },
}

impl std::fmt::Display for AscentProfileError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
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
            Self::PeriapsisAboveApoapsis {
                periapsis,
                apoapsis,
            } => write!(
                formatter,
                "target periapsis {periapsis} is above target apoapsis {apoapsis}"
            ),
            Self::TurnOrder { start, end } => {
                write!(formatter, "turn end {end} must be above turn start {start}")
            }
            Self::TurnEndsPastApoapsis { turn_end, apoapsis } => write!(
                formatter,
                "turn end {turn_end} must be below target apoapsis {apoapsis}"
            ),
        }
    }
}

impl std::error::Error for AscentProfileError {}

/// One ascent phase executed by the authority. The graph sequences phases
/// One ascent phase executed by the authority. Each variant carries the
/// profile parameters its executor needs — the executable IR is
/// self-contained, so two different profiles can never build the same
/// graph. `max_phase_time_s` rides every phase: the watchdog binds the
/// executor, not the planner.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum AscentPhase {
    /// Vertical rise at liftoff throttle until tower clearance.
    VerticalRise {
        throttle: f64,
        max_phase_time_s: f64,
    },
    /// Pitch-program gravity turn until MECO. The turn altitudes drive
    /// [`pitch_program_rad`] — without them the node is not executable.
    GravityTurn {
        turn_start_altitude_m: f64,
        turn_end_altitude_m: f64,
        max_phase_time_s: f64,
    },
    /// Unpowered coast to the target apoapsis.
    Coast {
        target_apoapsis_m: f64,
        max_phase_time_s: f64,
    },
    /// Circularization burn at apoapsis into the target periapsis.
    Circularize {
        target_periapsis_m: f64,
        max_phase_time_s: f64,
    },
    /// Engine cutoff and safeing after an abort trigger.
    Abort { max_phase_time_s: f64 },
}

/// Pitch above the local horizon (rad) for the gravity-turn program:
/// vertical below the turn start, a smooth cosine blend to horizontal at
/// the turn end, horizontal above. Monotone by construction; the
/// authority evaluates it against measured altitude every tick.
pub fn pitch_program_rad(profile: &AscentProfile, altitude_m: f64) -> f64 {
    if !(altitude_m.is_finite()) {
        return std::f64::consts::FRAC_PI_2;
    }
    if altitude_m <= profile.turn_start_altitude_m {
        return std::f64::consts::FRAC_PI_2;
    }
    if altitude_m >= profile.turn_end_altitude_m {
        return 0.0;
    }
    let span = profile.turn_end_altitude_m - profile.turn_start_altitude_m;
    if span <= 0.0 || !span.is_finite() {
        return 0.0;
    }
    let t = ((altitude_m - profile.turn_start_altitude_m) / span).clamp(0.0, 1.0);
    // Cosine blend: zero slope at both ends, so neither the vertical
    // handoff nor the horizontal capture jerks the steering loop.
    std::f64::consts::FRAC_PI_2 * 0.5 * (1.0 + (std::f64::consts::PI * t).cos())
}

/// Predicted apoapsis radius (m) from a two-body osculating state, or
/// `None` for unbound/degenerate inputs. The authority publishes MECO
/// when this reaches the target; the graph only waits for the event.
pub fn predict_apoapsis_m(mu: f64, r_vec: DVec3, v_vec: DVec3) -> Option<f64> {
    if !mu.is_finite() || mu <= 0.0 {
        return None;
    }
    let r = r_vec.length();
    let v2 = v_vec.length_squared();
    if !r.is_finite() || r <= 0.0 || !v2.is_finite() {
        return None;
    }
    let energy: f64 = v2 / 2.0 - mu / r;
    if !energy.is_finite() || energy >= 0.0 {
        return None;
    }
    let semi_major: f64 = -mu / (2.0 * energy);
    let h2: f64 = r_vec.cross(v_vec).length_squared();
    if !h2.is_finite() {
        return None;
    }
    let p: f64 = h2 / mu;
    let ecc_sq: f64 = (1.0 - p / semi_major).max(0.0);
    if !ecc_sq.is_finite() {
        return None;
    }
    let apoapsis = semi_major * (1.0 + ecc_sq.sqrt());
    apoapsis.is_finite().then_some(apoapsis)
}

/// MECO predicate for tests and authority guards: predicted apoapsis at
/// or above the target radius.
pub fn apoapsis_reached(mu: f64, r_vec: DVec3, v_vec: DVec3, target_radius_m: f64) -> bool {
    predict_apoapsis_m(mu, r_vec, v_vec).is_some_and(|apoapsis| apoapsis >= target_radius_m)
}

#[derive(Debug, Clone, PartialEq)]
pub enum AscentBuildError {
    InvalidProfile(AscentProfileError),
    InvalidGraph(Vec<crate::GraphError>),
}

impl std::fmt::Display for AscentBuildError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidProfile(error) => write!(formatter, "invalid ascent profile: {error}"),
            Self::InvalidGraph(errors) => {
                write!(formatter, "ascent graph failed validation: {errors:?}")
            }
        }
    }
}

impl std::error::Error for AscentBuildError {}

fn done_out(name: &str) -> GraphNode {
    GraphNode {
        id: NodeId(0),
        name: name.into(),
        kind: NodeKind::Source,
        ports: vec![Port::output("out", PortType::Unit)],
        config: None,
    }
}

fn phase_node(id: u32, name: &str, phase: AscentPhase) -> GraphNode {
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
        config: Some(GraphNodeConfig::AscentPhase { phase }),
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

/// Build the ascent subgraph for a validated profile. Node ids are fixed
/// so builds are deterministic across calls and machines. The finished
/// graph always validates: construction failure returns the profile or
/// graph error instead of an un-runnable graph.
pub fn ascent_graph(profile: &AscentProfile) -> Result<AutopilotGraph, AscentBuildError> {
    profile
        .validate()
        .map_err(AscentBuildError::InvalidProfile)?;
    let mut liftoff = done_out("liftoff");
    liftoff.id = NodeId(0);
    let graph = AutopilotGraph {
        nodes: vec![
            liftoff,
            phase_node(
                1,
                "vertical-rise",
                AscentPhase::VerticalRise {
                    throttle: profile.liftoff_throttle,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(2, "wait-tower", event::TOWER_CLEARED),
            phase_node(
                3,
                "gravity-turn",
                AscentPhase::GravityTurn {
                    turn_start_altitude_m: profile.turn_start_altitude_m,
                    turn_end_altitude_m: profile.turn_end_altitude_m,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(4, "wait-meco", event::MECO),
            phase_node(
                5,
                "coast",
                AscentPhase::Coast {
                    target_apoapsis_m: profile.target_apoapsis_m,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            wait_event_node(6, "wait-apoapsis", event::APOAPSIS_APPROACH),
            phase_node(
                7,
                "circularize",
                AscentPhase::Circularize {
                    target_periapsis_m: profile.target_periapsis_m,
                    max_phase_time_s: profile.max_phase_time_s,
                },
            ),
            GraphNode {
                id: NodeId(8),
                name: "orbit-achieved".into(),
                kind: NodeKind::Sink,
                ports: vec![Port::input("in", PortType::Unit, true)],
                config: None,
            },
            // Parallel abort branch: no incoming edges, so it arms
            // alongside the main chain and fires on either trigger.
            GraphNode {
                id: NodeId(9),
                name: "wait-abort-trigger".into(),
                kind: NodeKind::Wait,
                ports: vec![Port::output("out", PortType::Unit)],
                config: Some(GraphNodeConfig::Wait {
                    condition: WaitCondition::Any(vec![
                        WaitCondition::Event(event::ENGINE_OUT.into()),
                        WaitCondition::Event(event::GUIDANCE_DEVIATION.into()),
                    ]),
                }),
            },
            phase_node(10, "abort-cutoff", AscentPhase::Abort {
                max_phase_time_s: profile.max_phase_time_s,
            }),
        ],
        edges: vec![
            edge(0, 1),
            edge(1, 2),
            edge(2, 3),
            edge(3, 4),
            edge(4, 5),
            edge(5, 6),
            edge(6, 7),
            edge(7, 8),
            edge(9, 10),
        ],
    };
    graph.validate().map_err(AscentBuildError::InvalidGraph)?;
    Ok(graph)
}

/// Minimal executor for ascent phase blocks in tests and native hosts.
/// Real flight execution lives in the authority; this block proves the
/// subgraph runs end to end on the event contract: phases complete
/// immediately with their control action, waits park on their condition.
///
/// Wait semantics follow the runner contract: the first execution parks
/// the node, the re-execution after the wake condition completes it. The
/// per-node call count is the continuation state the trait provides for.
#[derive(Debug, Default)]
pub struct AscentBlock {
    actions: Vec<crate::GraphControlAction>,
    calls: BTreeMap<NodeId, u32>,
}

impl GraphBlock for AscentBlock {
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
            Some(GraphNodeConfig::AscentPhase { phase }) => {
                use crate::GraphControlAction as Action;
                use thessa_flight_control::{GuidanceIntent, PilotAxes, PropulsionDemand};
                let action = match phase {
                    AscentPhase::VerticalRise {
                        throttle,
                        ..
                    } => {
                        let Ok(propulsion) = PropulsionDemand::new(*throttle) else {
                            return GraphNodeOutcome::Fail {
                                diagnostic: crate::Diagnostic {
                                    kind: crate::DiagnosticKind::InvalidState,
                                    code: "ascent-throttle".into(),
                                    message: "vertical-rise throttle out of range".into(),
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
                    AscentPhase::Abort { .. } => {
                        // Engine cutoff already commanded by the trigger
                        // event path; the abort outcome terminates the
                        // whole graph through the runner, not just this
                        // branch — a half-aborted ascent is worse than
                        // none, and the diagnostic carries the cause.
                        return GraphNodeOutcome::Abort {
                            diagnostic: crate::Diagnostic {
                                kind: crate::DiagnosticKind::Warning,
                                code: "ascent-abort".into(),
                                message: "ascent aborted through engine cutoff".into(),
                                value: None,
                                limit: None,
                            },
                        };
                    }
                    AscentPhase::GravityTurn { .. }
                    | AscentPhase::Coast { .. }
                    | AscentPhase::Circularize { .. } => {
                        Action::Guidance {
                            intent: GuidanceIntent::ManualAxes(PilotAxes::default()),
                            propulsion: PropulsionDemand::new(0.0)
                                .expect("zero propulsion always builds"),
                        }
                    }
                };
                self.actions.push(action);
                GraphNodeOutcome::Complete {
                    outputs: BTreeMap::from([("out".into(), GraphValue::Unit)]),
                }
            }
            // Wait nodes park on their configured event condition; the
            // runner resumes them when the authority publishes the event.
            Some(GraphNodeConfig::Wait { condition }) => GraphNodeOutcome::Wait {
                condition: condition.clone(),
            },
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

    fn test_profile() -> AscentProfile {
        AscentProfile {
            target_apoapsis_m: 200_000.0,
            target_periapsis_m: 190_000.0,
            turn_start_altitude_m: 1_000.0,
            turn_end_altitude_m: 120_000.0,
            liftoff_throttle: 1.0,
            max_phase_time_s: 3_600.0,
        }
    }

    #[test]
    fn profile_accepts_a_sane_ascent_and_rejects_every_bad_field() {
        test_profile().validate().expect("sane profile validates");
        let bad = |mutate: fn(&mut AscentProfile)| {
            let mut profile = test_profile();
            mutate(&mut profile);
            profile.validate().expect_err("bad profile must fail")
        };
        bad(|p| p.target_apoapsis_m = f64::NAN);
        bad(|p| p.target_apoapsis_m = 0.0);
        bad(|p| p.target_periapsis_m = 250_000.0);
        bad(|p| p.turn_start_altitude_m = -1.0);
        bad(|p| p.turn_end_altitude_m = 1_000.0);
        bad(|p| p.turn_end_altitude_m = 200_000.0);
        bad(|p| p.liftoff_throttle = 1.5);
        bad(|p| p.max_phase_time_s = 0.0);
    }

    #[test]
    fn pitch_program_is_vertical_then_monotone_to_horizontal() {
        let profile = test_profile();
        let vertical = std::f64::consts::FRAC_PI_2;
        assert_eq!(pitch_program_rad(&profile, 0.0), vertical);
        assert_eq!(pitch_program_rad(&profile, 1_000.0), vertical);
        assert_eq!(pitch_program_rad(&profile, 120_000.0), 0.0);
        assert_eq!(pitch_program_rad(&profile, 500_000.0), 0.0);
        // Mid-turn is mid-pitch, and the program never climbs back up.
        let mid = pitch_program_rad(&profile, 60_500.0);
        assert!((mid - vertical / 2.0).abs() < 1.0e-9);
        let mut previous = vertical;
        let mut altitude = 1_000.0;
        while altitude <= 120_000.0 {
            let current = pitch_program_rad(&profile, altitude);
            assert!(current <= previous + 1.0e-12);
            previous = current;
            altitude += 1_000.0;
        }
        // Non-finite altitude fails safe to vertical, not to a NaN steer.
        assert_eq!(pitch_program_rad(&profile, f64::NAN), vertical);
    }

    #[test]
    fn apoapsis_predictor_reports_circular_elliptical_and_unbound() {
        let mu: f64 = 3.986_004_418e14;
        // Circular: apoapsis is the radius.
        let r = DVec3::new(6_571_000.0, 0.0, 0.0);
        let v = DVec3::new(0.0, (mu / 6_571_000.0).sqrt(), 0.0);
        let apo = predict_apoapsis_m(mu, r, v).expect("circular predicts");
        assert!((apo - 6_571_000.0).abs() < 1.0);
        assert!(apoapsis_reached(mu, r, v, 6_571_000.0));
        assert!(!apoapsis_reached(mu, r, v, 6_571_001.0));
        // Elliptical perigee kick: vis-viva apoapsis is exact two-body.
        let rp = 6_571_000.0;
        let ra = 6_771_000.0;
        let a = (rp + ra) / 2.0;
        let vp = (mu * (2.0 / rp - 1.0 / a)).sqrt();
        let apo2 = predict_apoapsis_m(mu, DVec3::new(rp, 0.0, 0.0), DVec3::new(0.0, vp, 0.0))
            .expect("elliptical predicts");
        assert!((apo2 - ra).abs() / ra < 1.0e-9);
        // Hyperbolic and degenerate inputs have no apoapsis.
        let vesc = (2.0 * mu / rp).sqrt();
        assert!(
            predict_apoapsis_m(
                mu,
                DVec3::new(rp, 0.0, 0.0),
                DVec3::new(0.0, vesc * 1.1, 0.0)
            )
            .is_none()
        );
        assert!(predict_apoapsis_m(0.0, r, v).is_none());
        assert!(predict_apoapsis_m(mu, DVec3::ZERO, v).is_none());
    }

    #[test]
    fn ascent_graph_validates_deterministic_and_round_trips() {
        let first = ascent_graph(&test_profile()).expect("graph builds");
        first.validate().expect("built graph validates");
        let second = ascent_graph(&test_profile()).expect("graph rebuilds");
        assert_eq!(first, second);
        // Main chain plus an independent abort branch.
        assert_eq!(first.nodes.len(), 11);
        assert_eq!(first.edges.len(), 9);
        assert!(
            first
                .edges
                .iter()
                .any(|edge| edge.from.node == NodeId(9) && edge.to.node == NodeId(10))
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
        assert!(waits.contains(&event::TOWER_CLEARED));
        assert!(waits.contains(&event::MECO));
        // Serde round trip preserves the build bit-for-bit.
        let json = serde_json::to_string(&first).expect("serializes");
        let back: AutopilotGraph = serde_json::from_str(&json).expect("deserializes");
        assert_eq!(first, back);
        assert!(
            ascent_graph(&AscentProfile {
                target_apoapsis_m: -5.0,
                ..test_profile()
            })
            .is_err()
        );
    }

    #[test]
    fn distinct_profiles_build_distinct_executable_graphs() {
        // The profile must survive into the IR: two profiles that differ
        // only in turn altitudes or watchdog must not build the same nodes.
        fn phase_config(graph: &AutopilotGraph, name: &str) -> GraphNodeConfig {
            graph
                .nodes
                .iter()
                .find(|node| node.name == name)
                .expect("phase node exists")
                .config
                .clone()
                .expect("phase node is configured")
        }
        let base = ascent_graph(&test_profile()).expect("graph builds");
        let other = ascent_graph(&AscentProfile {
            turn_end_altitude_m: 100_000.0,
            max_phase_time_s: 1_800.0,
            ..test_profile()
        })
        .expect("graph builds");
        assert_ne!(base, other);
        assert_ne!(
            phase_config(&base, "gravity-turn"),
            phase_config(&other, "gravity-turn")
        );
        // A hand-built node with a broken watchdog must fail validation.
        let mut broken = base.clone();
        broken.nodes[3].config = Some(GraphNodeConfig::AscentPhase {
            phase: AscentPhase::GravityTurn {
                turn_start_altitude_m: 1_000.0,
                turn_end_altitude_m: 500.0,
                max_phase_time_s: 3_600.0,
            },
        });
        assert!(broken.validate().is_err());
    }

    #[test]
    fn ascent_block_flies_the_nominal_event_script_to_orbit() {
        let graph = ascent_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = AscentBlock::default();
        // Liftoff, rise and turn phases complete on their own; waits park.
        let state = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Tower cleared -> turn runs -> parks on MECO.
        let state = runner
            .poll(SimTime(10.0), Some(event::TOWER_CLEARED), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // MECO -> coast -> parks on apoapsis approach.
        let state = runner
            .poll(SimTime(500.0), Some(event::MECO), &mut block)
            .unwrap();
        assert!(matches!(state, GraphRunState::Waiting { .. }));
        // Apoapsis approach -> circularize -> orbit achieved. The main
        // chain is done, but the abort watcher stays armed: the terminal
        // state is Waiting on node 9, not Complete — a guarded orbit, not
        // an unguarded one. (Complete would require every node Ok,
        // including the never-firing watcher.)
        let state = runner
            .poll(SimTime(2_000.0), Some(event::APOAPSIS_APPROACH), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Waiting {
                node: NodeId(9),
                ..
            }
        ));
        // Phases emitted guidance actions for the authority to consume.
        assert!(!block.take_control_actions().is_empty());
        assert_eq!(runner.status(NodeId(7)), Some(BlockStatus::Ok));
        assert_eq!(runner.status(NodeId(8)), Some(BlockStatus::Ok));
    }

    #[test]
    fn engine_out_aborts_through_cutoff() {
        let graph = ascent_graph(&test_profile()).expect("graph builds");
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = AscentBlock::default();
        let _ = runner.poll(SimTime(0.0), None, &mut block).unwrap();
        let state = runner
            .poll(SimTime(30.0), Some(event::ENGINE_OUT), &mut block)
            .unwrap();
        assert!(matches!(
            state,
            GraphRunState::Aborted {
                node: Some(NodeId(10)),
                ..
            }
        ));
    }
}
