//! Typed, deterministic autopilot graph IR and event-driven wait primitives.
//!
//! The graph is deliberately independent of JavaScript and the renderer. A
//! native block, a future QuickJS block, and a UI-created graph all compile
//! to the same validated representation before they can control a vehicle.

use std::{collections::BTreeMap, error::Error, fmt};

use serde::{Deserialize, Serialize};
use thessa_flight_control::{ActuatorGroup, ControlDemand, GuidanceIntent, TrajectoryPlanId};
use thessa_sim_core::SimTime;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct NodeId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PortDirection {
    Input,
    Output,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PortType {
    Unit,
    Bool,
    Number,
    Vehicle,
    Body,
    OrbitTarget,
    TrajectoryPlan,
    AttitudeTarget,
    AngularRateTarget,
    FlightPathTarget,
    PositionTarget,
    PropulsionTarget,
    DockingTarget,
    FlightEnvelope,
    Diagnostic,
    Event,
    Any,
}

impl PortType {
    pub fn accepts(self, produced: Self) -> bool {
        self == Self::Any || produced == Self::Any || self == produced
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Port {
    pub name: String,
    pub ty: PortType,
    pub direction: PortDirection,
    pub required: bool,
}

impl Port {
    pub fn input(name: impl Into<String>, ty: PortType, required: bool) -> Self {
        Self {
            name: name.into(),
            ty,
            direction: PortDirection::Input,
            required,
        }
    }

    pub fn output(name: impl Into<String>, ty: PortType) -> Self {
        Self {
            name: name.into(),
            ty,
            direction: PortDirection::Output,
            required: false,
        }
    }
}

/// Static block category. Runtime implementations may attach their own data
/// around this IR; the category contains enough information for validation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum NodeKind {
    Source,
    Sink,
    Sequence,
    Branch,
    Wait,
    Parallel,
    Join,
    Controller {
        actuator_groups: Vec<ActuatorGroup>,
    },
    Custom {
        bakeability: Bakeability,
        wait_capable: bool,
    },
}

impl NodeKind {
    pub fn wait_capable(&self) -> bool {
        matches!(
            self,
            Self::Wait
                | Self::Custom {
                    wait_capable: true,
                    ..
                }
        )
    }

    pub fn actuator_groups(&self) -> &[ActuatorGroup] {
        match self {
            Self::Controller { actuator_groups } => actuator_groups,
            _ => &[],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GraphNode {
    pub id: NodeId,
    pub name: String,
    pub kind: NodeKind,
    pub ports: Vec<Port>,
}

impl GraphNode {
    pub fn port(&self, name: &str) -> Option<&Port> {
        self.ports.iter().find(|port| port.name == name)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PortRef {
    pub node: NodeId,
    pub port: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GraphEdge {
    pub from: PortRef,
    pub to: PortRef,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct AutopilotGraph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GraphValidation {
    pub topological_order: Vec<NodeId>,
}

impl AutopilotGraph {
    pub fn validate(&self) -> Result<GraphValidation, Vec<GraphError>> {
        let mut errors = Vec::new();
        let mut nodes = BTreeMap::new();
        for node in &self.nodes {
            if nodes.insert(node.id, node).is_some() {
                errors.push(GraphError::DuplicateNode(node.id));
            }
            let mut names = std::collections::HashSet::new();
            for port in &node.ports {
                if port.name.trim().is_empty() || !names.insert(port.name.clone()) {
                    errors.push(GraphError::DuplicatePort {
                        node: node.id,
                        port: port.name.clone(),
                    });
                }
            }
        }

        let mut incoming = BTreeMap::<NodeId, usize>::new();
        let mut adjacency = BTreeMap::<NodeId, Vec<NodeId>>::new();
        for node in &self.nodes {
            incoming.insert(node.id, 0);
            adjacency.insert(node.id, Vec::new());
        }
        let mut connected_inputs = std::collections::HashSet::<(NodeId, String)>::new();
        for edge in &self.edges {
            let Some(from_node) = nodes.get(&edge.from.node) else {
                errors.push(GraphError::UnknownNode(edge.from.node));
                continue;
            };
            let Some(to_node) = nodes.get(&edge.to.node) else {
                errors.push(GraphError::UnknownNode(edge.to.node));
                continue;
            };
            let Some(from) = from_node.port(&edge.from.port) else {
                errors.push(GraphError::UnknownPort(edge.from.clone()));
                continue;
            };
            let Some(to) = to_node.port(&edge.to.port) else {
                errors.push(GraphError::UnknownPort(edge.to.clone()));
                continue;
            };
            if from.direction != PortDirection::Output {
                errors.push(GraphError::WrongDirection(edge.from.clone()));
            }
            if to.direction != PortDirection::Input {
                errors.push(GraphError::WrongDirection(edge.to.clone()));
            }
            if !to.ty.accepts(from.ty) {
                errors.push(GraphError::TypeMismatch {
                    from: edge.from.clone(),
                    produced: from.ty,
                    to: edge.to.clone(),
                    expected: to.ty,
                });
            }
            if !connected_inputs.insert((to_node.id, to.name.clone())) {
                errors.push(GraphError::MultipleDrivers(edge.to.clone()));
            }
            if let Some(degree) = incoming.get_mut(&to_node.id) {
                *degree += 1;
            }
            if let Some(neighbors) = adjacency.get_mut(&from_node.id) {
                neighbors.push(to_node.id);
            }
        }
        for node in &self.nodes {
            for port in &node.ports {
                if port.direction == PortDirection::Input
                    && port.required
                    && !connected_inputs.contains(&(node.id, port.name.clone()))
                {
                    errors.push(GraphError::MissingInput {
                        node: node.id,
                        port: port.name.clone(),
                    });
                }
            }
        }
        let mut owners = BTreeMap::<ActuatorGroup, NodeId>::new();
        for node in &self.nodes {
            for group in node.kind.actuator_groups() {
                if let Some(previous) = owners.insert(*group, node.id) {
                    errors.push(GraphError::ActuatorConflict {
                        group: *group,
                        first: previous,
                        second: node.id,
                    });
                }
            }
        }

        let mut queue = incoming
            .iter()
            .filter_map(|(node, degree)| (*degree == 0).then_some(*node))
            .collect::<Vec<_>>();
        queue.sort_unstable();
        let mut topological_order = Vec::with_capacity(self.nodes.len());
        while let Some(node) = queue.pop() {
            topological_order.push(node);
            if let Some(neighbors) = adjacency.get(&node) {
                for neighbor in neighbors {
                    let degree = incoming.get_mut(neighbor).expect("adjacency node exists");
                    *degree -= 1;
                    if *degree == 0 {
                        queue.push(*neighbor);
                    }
                }
                queue.sort_unstable();
            }
        }
        if topological_order.len() != self.nodes.len() {
            let mut colors = BTreeMap::<NodeId, VisitColor>::new();
            let mut stack = Vec::new();
            let mut cycle_without_wait = false;
            for node in &self.nodes {
                if !matches!(colors.get(&node.id), Some(VisitColor::Black)) {
                    if let Err(error) =
                        visit_cycle(node.id, &adjacency, &nodes, &mut colors, &mut stack)
                    {
                        cycle_without_wait |= GraphError::is_cycle_error(&error);
                    }
                }
            }
            if cycle_without_wait {
                errors.push(GraphError::CycleWithoutWait);
            }
        }
        if errors.is_empty() {
            Ok(GraphValidation { topological_order })
        } else {
            Err(errors)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum VisitColor {
    Gray,
    Black,
}

fn visit_cycle(
    node: NodeId,
    adjacency: &BTreeMap<NodeId, Vec<NodeId>>,
    nodes: &BTreeMap<NodeId, &GraphNode>,
    colors: &mut BTreeMap<NodeId, VisitColor>,
    stack: &mut Vec<NodeId>,
) -> Result<(), GraphError> {
    match colors.get(&node) {
        Some(VisitColor::Black) => return Ok(()),
        Some(VisitColor::Gray) => {
            let start = stack.iter().position(|id| *id == node).unwrap_or(0);
            let has_wait = stack[start..]
                .iter()
                .any(|id| nodes.get(id).is_some_and(|node| node.kind.wait_capable()));
            return if has_wait {
                Ok(())
            } else {
                Err(GraphError::CycleWithoutWait)
            };
        }
        None => {}
    }
    colors.insert(node, VisitColor::Gray);
    stack.push(node);
    if let Some(neighbors) = adjacency.get(&node) {
        for neighbor in neighbors {
            visit_cycle(*neighbor, adjacency, nodes, colors, stack)?;
        }
    }
    stack.pop();
    colors.insert(node, VisitColor::Black);
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GraphError {
    DuplicateNode(NodeId),
    DuplicatePort {
        node: NodeId,
        port: String,
    },
    UnknownNode(NodeId),
    UnknownPort(PortRef),
    WrongDirection(PortRef),
    TypeMismatch {
        from: PortRef,
        produced: PortType,
        to: PortRef,
        expected: PortType,
    },
    MultipleDrivers(PortRef),
    MissingInput {
        node: NodeId,
        port: String,
    },
    ActuatorConflict {
        group: ActuatorGroup,
        first: NodeId,
        second: NodeId,
    },
    CycleWithoutWait,
}

impl GraphError {
    fn is_cycle_error(error: &Self) -> bool {
        matches!(error, Self::CycleWithoutWait)
    }
}

impl fmt::Display for GraphError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateNode(id) => write!(formatter, "duplicate graph node {id:?}"),
            Self::DuplicatePort { node, port } => {
                write!(formatter, "duplicate or empty port {port:?} on {node:?}")
            }
            Self::UnknownNode(id) => write!(formatter, "unknown graph node {id:?}"),
            Self::UnknownPort(port) => write!(formatter, "unknown graph port {port:?}"),
            Self::WrongDirection(port) => {
                write!(formatter, "wrong direction for graph port {port:?}")
            }
            Self::TypeMismatch {
                from,
                produced,
                to,
                expected,
            } => write!(
                formatter,
                "cannot connect {from:?} ({produced:?}) to {to:?} ({expected:?})"
            ),
            Self::MultipleDrivers(port) => write!(formatter, "multiple drivers for {port:?}"),
            Self::MissingInput { node, port } => write!(
                formatter,
                "required input {port:?} on {node:?} is unconnected"
            ),
            Self::ActuatorConflict {
                group,
                first,
                second,
            } => write!(
                formatter,
                "actuator group {group:?} is owned by both {first:?} and {second:?}"
            ),
            Self::CycleWithoutWait => {
                write!(formatter, "graph cycle has no wait or yield boundary")
            }
        }
    }
}

impl Error for GraphError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DiagnosticKind {
    Warning,
    ConstraintViolation,
    NoSolution,
    Saturated,
    InvalidState,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Diagnostic {
    pub kind: DiagnosticKind,
    pub code: String,
    pub message: String,
    pub value: Option<f64>,
    pub limit: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BlockStatus {
    Ok,
    Waiting,
    Failed,
    Degraded,
}

/// Conditions are registered once and evaluated only when the scheduler has
/// a relevant time or domain event. There is no graph polling contract here.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WaitCondition {
    At(SimTime),
    Event(String),
    Any(Vec<WaitCondition>),
    All(Vec<WaitCondition>),
}

impl WaitCondition {
    pub fn at(time: SimTime) -> Self {
        Self::At(time)
    }

    pub fn validate(&self) -> Result<(), WaitError> {
        match self {
            Self::At(time) if !time.0.is_finite() => Err(WaitError::InvalidTime),
            Self::Event(name) if name.trim().is_empty() => Err(WaitError::EmptyEvent),
            Self::Any(conditions) | Self::All(conditions) if conditions.is_empty() => {
                Err(WaitError::EmptyComposite)
            }
            Self::Any(conditions) | Self::All(conditions) => {
                for condition in conditions {
                    condition.validate()?;
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    fn is_due(&self, now: SimTime, event: Option<&str>) -> bool {
        match self {
            Self::At(time) => time.0 <= now.0,
            Self::Event(name) => event.is_some_and(|candidate| candidate == name),
            Self::Any(conditions) => conditions
                .iter()
                .any(|condition| condition.is_due(now, event)),
            Self::All(conditions) => conditions
                .iter()
                .all(|condition| condition.is_due(now, event)),
        }
    }

    fn next_time(&self) -> Option<SimTime> {
        match self {
            Self::At(time) => Some(*time),
            Self::Event(_) => None,
            Self::Any(conditions) => conditions
                .iter()
                .filter_map(Self::next_time)
                .min_by(|a, b| a.0.total_cmp(&b.0)),
            Self::All(conditions) => conditions
                .iter()
                .filter_map(Self::next_time)
                .max_by(|a, b| a.0.total_cmp(&b.0)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct WaitId(pub u64);

#[derive(Debug, Clone, PartialEq)]
pub struct WaitSet {
    next_id: u64,
    waits: BTreeMap<WaitId, WaitCondition>,
}

impl Default for WaitSet {
    fn default() -> Self {
        Self {
            next_id: 1,
            waits: BTreeMap::new(),
        }
    }
}

impl WaitSet {
    pub fn register(&mut self, condition: WaitCondition) -> Result<WaitId, WaitError> {
        condition.validate()?;
        let id = WaitId(self.next_id);
        self.next_id = self.next_id.checked_add(1).ok_or(WaitError::IdExhausted)?;
        self.waits.insert(id, condition);
        Ok(id)
    }

    pub fn cancel(&mut self, id: WaitId) -> bool {
        self.waits.remove(&id).is_some()
    }

    pub fn len(&self) -> usize {
        self.waits.len()
    }

    pub fn next_time(&self) -> Option<SimTime> {
        self.waits
            .values()
            .filter_map(WaitCondition::next_time)
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }

    /// Resolve due waits and remove them atomically. A domain event may wake
    /// an event guard early; unrelated waits remain parked.
    pub fn wake(&mut self, now: SimTime, event: Option<&str>) -> Vec<WaitId> {
        let due = self
            .waits
            .iter()
            .filter_map(|(id, condition)| condition.is_due(now, event).then_some(*id))
            .collect::<Vec<_>>();
        for id in &due {
            self.waits.remove(id);
        }
        due
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Bakeability {
    Pure,
    Guarded,
    Live,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrajectorySegment {
    Coast {
        duration_s: f64,
    },
    Burn {
        duration_s: f64,
        demand: ControlDemand,
    },
    Guidance {
        duration_s: f64,
        intent: GuidanceIntent,
    },
    Wait {
        condition: WaitCondition,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrajectoryPlan {
    pub id: TrajectoryPlanId,
    pub segments: Vec<TrajectorySegment>,
    pub bakeability: Bakeability,
}

impl TrajectoryPlan {
    pub fn validate(&self) -> Result<(), PlanError> {
        if self.segments.is_empty() {
            return Err(PlanError::Empty);
        }
        for segment in &self.segments {
            match segment {
                TrajectorySegment::Coast { duration_s }
                | TrajectorySegment::Burn { duration_s, .. }
                | TrajectorySegment::Guidance { duration_s, .. }
                    if !duration_s.is_finite() || *duration_s <= 0.0 =>
                {
                    return Err(PlanError::InvalidDuration(*duration_s));
                }
                TrajectorySegment::Burn { demand, .. } => {
                    demand.validate().map_err(PlanError::Control)?;
                }
                TrajectorySegment::Guidance { intent, .. } => {
                    intent.validate().map_err(PlanError::Control)?;
                }
                TrajectorySegment::Wait { condition } => {
                    condition.validate().map_err(PlanError::Wait)?;
                    if self.bakeability == Bakeability::Pure && contains_domain_event(condition) {
                        return Err(PlanError::LiveGuardInPurePlan);
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }
}

fn contains_domain_event(condition: &WaitCondition) -> bool {
    match condition {
        WaitCondition::Event(_) => true,
        WaitCondition::At(_) => false,
        WaitCondition::Any(conditions) | WaitCondition::All(conditions) => {
            conditions.iter().any(contains_domain_event)
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum WaitError {
    InvalidTime,
    EmptyEvent,
    EmptyComposite,
    IdExhausted,
}

impl fmt::Display for WaitError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTime => write!(formatter, "wait time must be finite"),
            Self::EmptyEvent => write!(formatter, "wait event must not be empty"),
            Self::EmptyComposite => write!(formatter, "composite wait must contain a condition"),
            Self::IdExhausted => write!(formatter, "wait id space exhausted"),
        }
    }
}

impl Error for WaitError {}

#[derive(Debug, Clone, PartialEq)]
pub enum PlanError {
    Empty,
    InvalidDuration(f64),
    Control(thessa_flight_control::ControlError),
    Wait(WaitError),
    LiveGuardInPurePlan,
}

impl fmt::Display for PlanError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(formatter, "trajectory plan has no segments"),
            Self::InvalidDuration(duration) => {
                write!(formatter, "invalid segment duration {duration}")
            }
            Self::Control(error) => write!(formatter, "control demand is invalid: {error}"),
            Self::Wait(error) => write!(formatter, "wait condition is invalid: {error}"),
            Self::LiveGuardInPurePlan => {
                write!(formatter, "pure plan contains a live domain-event guard")
            }
        }
    }
}

impl Error for PlanError {}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_flight_control::ActuatorGroup;

    fn graph_with_edge(from: PortType, to: PortType) -> AutopilotGraph {
        AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(1),
                    name: "source".into(),
                    kind: NodeKind::Source,
                    ports: vec![Port::output("out", from)],
                },
                GraphNode {
                    id: NodeId(2),
                    name: "sink".into(),
                    kind: NodeKind::Sink,
                    ports: vec![Port::input("in", to, true)],
                },
            ],
            edges: vec![GraphEdge {
                from: PortRef {
                    node: NodeId(1),
                    port: "out".into(),
                },
                to: PortRef {
                    node: NodeId(2),
                    port: "in".into(),
                },
            }],
        }
    }

    #[test]
    fn graph_accepts_matching_typed_ports() {
        let validation = graph_with_edge(PortType::Vehicle, PortType::Vehicle)
            .validate()
            .expect("valid graph");
        assert_eq!(validation.topological_order, vec![NodeId(1), NodeId(2)]);
    }

    #[test]
    fn graph_rejects_type_mismatch_and_missing_input() {
        let error = graph_with_edge(PortType::Vehicle, PortType::OrbitTarget)
            .validate()
            .expect_err("invalid graph");
        assert!(
            error
                .iter()
                .any(|error| matches!(error, GraphError::TypeMismatch { .. }))
        );
    }

    #[test]
    fn graph_rejects_controller_ownership_conflicts() {
        let graph = AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(1),
                    name: "a".into(),
                    kind: NodeKind::Controller {
                        actuator_groups: vec![ActuatorGroup::Rcs],
                    },
                    ports: Vec::new(),
                },
                GraphNode {
                    id: NodeId(2),
                    name: "b".into(),
                    kind: NodeKind::Controller {
                        actuator_groups: vec![ActuatorGroup::Rcs],
                    },
                    ports: Vec::new(),
                },
            ],
            edges: Vec::new(),
        };
        let error = graph.validate().expect_err("conflict");
        assert!(
            error
                .iter()
                .any(|error| matches!(error, GraphError::ActuatorConflict { .. }))
        );
    }

    #[test]
    fn graph_cycle_requires_a_wait_boundary() {
        let ports = || {
            vec![
                Port::input("in", PortType::Unit, false),
                Port::output("out", PortType::Unit),
            ]
        };
        let mut graph = AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(1),
                    name: "loop-a".into(),
                    kind: NodeKind::Custom {
                        bakeability: Bakeability::Live,
                        wait_capable: false,
                    },
                    ports: ports(),
                },
                GraphNode {
                    id: NodeId(2),
                    name: "loop-b".into(),
                    kind: NodeKind::Custom {
                        bakeability: Bakeability::Live,
                        wait_capable: false,
                    },
                    ports: ports(),
                },
            ],
            edges: vec![
                GraphEdge {
                    from: PortRef {
                        node: NodeId(1),
                        port: "out".into(),
                    },
                    to: PortRef {
                        node: NodeId(2),
                        port: "in".into(),
                    },
                },
                GraphEdge {
                    from: PortRef {
                        node: NodeId(2),
                        port: "out".into(),
                    },
                    to: PortRef {
                        node: NodeId(1),
                        port: "in".into(),
                    },
                },
            ],
        };
        assert!(graph.validate().is_err());
        graph.nodes[0].kind = NodeKind::Wait;
        assert!(graph.validate().is_ok());
    }

    #[test]
    fn wait_set_parks_until_time_or_domain_event() {
        let mut waits = WaitSet::default();
        let time = waits.register(WaitCondition::At(SimTime(10.0))).unwrap();
        let event = waits
            .register(WaitCondition::Event("docked".into()))
            .unwrap();
        assert_eq!(waits.next_time(), Some(SimTime(10.0)));
        assert_eq!(waits.wake(SimTime(9.0), Some("impact")), Vec::new());
        assert_eq!(waits.wake(SimTime(9.0), Some("docked")), vec![event]);
        assert_eq!(waits.wake(SimTime(10.0), None), vec![time]);
        assert_eq!(waits.len(), 0);
    }

    #[test]
    fn all_waits_schedule_the_latest_known_time() {
        let mut waits = WaitSet::default();
        waits
            .register(WaitCondition::All(vec![
                WaitCondition::At(SimTime(10.0)),
                WaitCondition::At(SimTime(20.0)),
            ]))
            .unwrap();
        assert_eq!(waits.next_time(), Some(SimTime(20.0)));
        assert!(waits.wake(SimTime(10.0), None).is_empty());
        assert_eq!(waits.wake(SimTime(20.0), None).len(), 1);
    }

    #[test]
    fn pure_plan_rejects_live_event_guard() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(1),
            segments: vec![TrajectorySegment::Wait {
                condition: WaitCondition::Event("impact".into()),
            }],
            bakeability: Bakeability::Pure,
        };
        assert!(matches!(
            plan.validate(),
            Err(PlanError::LiveGuardInPurePlan)
        ));
    }
}
