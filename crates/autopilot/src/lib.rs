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
    /// Data-first touchdown target. Terrain validation is an authority-side
    /// concern and is intentionally separate from graph type checking.
    LandingSite,
    /// Data-first predicted contact target. It shares the same center/radius
    /// shape as a landing site but remains a distinct graph type.
    ImpactSite,
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

/// A surface-centered site supplied by an autopilot planner. The center is a
/// unit direction in the reference body's frame; the radius describes the
/// footprint to inspect when the authority later declares obstacles.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct LandingSite {
    pub center_dir: [f64; 3],
    pub radius_m: f64,
}

impl LandingSite {
    pub fn new(center_dir: [f64; 3], radius_m: f64) -> Result<Self, SiteError> {
        Ok(Self {
            center_dir: normalize_site_center(center_dir)?,
            radius_m: validate_site_radius(radius_m)?,
        })
    }

    pub fn validate(self) -> Result<(), SiteError> {
        validate_site_center(self.center_dir)?;
        validate_site_radius(self.radius_m).map(|_| ())
    }
}

/// A predicted impact footprint. It is intentionally separate from
/// [`LandingSite`] so a graph cannot silently connect a forecast impact to a
/// planned touchdown input without an explicit adapter.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImpactSite {
    pub center_dir: [f64; 3],
    pub radius_m: f64,
}

impl ImpactSite {
    pub fn new(center_dir: [f64; 3], radius_m: f64) -> Result<Self, SiteError> {
        Ok(Self {
            center_dir: normalize_site_center(center_dir)?,
            radius_m: validate_site_radius(radius_m)?,
        })
    }

    pub fn validate(self) -> Result<(), SiteError> {
        validate_site_center(self.center_dir)?;
        validate_site_radius(self.radius_m).map(|_| ())
    }
}

fn normalize_site_center(center_dir: [f64; 3]) -> Result<[f64; 3], SiteError> {
    let length_sq = center_dir[0] * center_dir[0]
        + center_dir[1] * center_dir[1]
        + center_dir[2] * center_dir[2];
    if !length_sq.is_finite() || length_sq <= 1.0e-24 {
        return Err(SiteError::InvalidCenter);
    }
    let length = length_sq.sqrt();
    Ok([
        center_dir[0] / length,
        center_dir[1] / length,
        center_dir[2] / length,
    ])
}

fn validate_site_center(center_dir: [f64; 3]) -> Result<(), SiteError> {
    let length_sq = center_dir[0] * center_dir[0]
        + center_dir[1] * center_dir[1]
        + center_dir[2] * center_dir[2];
    if !length_sq.is_finite() || (length_sq - 1.0).abs() > 1.0e-9 {
        return Err(SiteError::InvalidCenter);
    }
    Ok(())
}

fn validate_site_radius(radius_m: f64) -> Result<f64, SiteError> {
    if !radius_m.is_finite() || radius_m < 0.0 {
        return Err(SiteError::InvalidRadius);
    }
    Ok(radius_m)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SiteError {
    InvalidCenter,
    InvalidRadius,
}

impl fmt::Display for SiteError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCenter => write!(formatter, "site center must be finite and nonzero"),
            Self::InvalidRadius => write!(formatter, "site radius must be finite and non-negative"),
        }
    }
}

impl Error for SiteError {}

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
                if !matches!(colors.get(&node.id), Some(VisitColor::Black))
                    && let Err(error) =
                        visit_cycle(node.id, &adjacency, &nodes, &mut colors, &mut stack)
                {
                    cycle_without_wait |= GraphError::is_cycle_error(&error);
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

/// Small value boundary used by native graph execution. Domain-specific
/// blocks may carry richer payloads out of band, but every edge still crosses
/// this type-checked boundary before a downstream block is run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GraphValue {
    Unit,
    Bool(bool),
    Number(f64),
    LandingSite(LandingSite),
    ImpactSite(ImpactSite),
    /// A typed token for domain values whose payload belongs to the host
    /// block (Vehicle, OrbitTarget, and so on).
    Typed(PortType),
}

impl GraphValue {
    pub fn port_type(&self) -> PortType {
        match self {
            Self::Unit => PortType::Unit,
            Self::Bool(_) => PortType::Bool,
            Self::Number(_) => PortType::Number,
            Self::LandingSite(_) => PortType::LandingSite,
            Self::ImpactSite(_) => PortType::ImpactSite,
            Self::Typed(ty) => *ty,
        }
    }

    fn validate(&self) -> Result<(), GraphValueError> {
        match self {
            Self::Number(value) if !value.is_finite() => Err(GraphValueError::NonFiniteNumber),
            Self::LandingSite(site) => site.validate().map_err(|_| GraphValueError::InvalidSite),
            Self::ImpactSite(site) => site.validate().map_err(|_| GraphValueError::InvalidSite),
            Self::Typed(PortType::Any) => Err(GraphValueError::UntypedToken),
            _ => Ok(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphValueError {
    NonFiniteNumber,
    InvalidSite,
    UntypedToken,
}

impl fmt::Display for GraphValueError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteNumber => write!(formatter, "graph value contains a non-finite number"),
            Self::InvalidSite => write!(formatter, "graph value contains an invalid surface site"),
            Self::UntypedToken => write!(formatter, "graph value cannot use the Any port type"),
        }
    }
}

impl Error for GraphValueError {}

/// Result returned by a native block at one execution boundary. Waiting does
/// not consume a physics tick: the runner parks the node and lets the owner
/// wake it with simulation time or a domain event.
#[derive(Debug, Clone, PartialEq)]
pub enum GraphNodeOutcome {
    Complete {
        outputs: BTreeMap<String, GraphValue>,
    },
    Wait {
        condition: WaitCondition,
    },
    Fail {
        diagnostic: Diagnostic,
    },
    Abort {
        diagnostic: Diagnostic,
    },
}

/// Native block boundary. The runner owns graph state and dependency order;
/// the block owns its deterministic domain behavior and any continuation
/// state needed after a wait.
pub trait GraphBlock {
    fn execute(
        &mut self,
        node: &GraphNode,
        inputs: &BTreeMap<String, GraphValue>,
    ) -> GraphNodeOutcome;
}

#[derive(Debug, Clone, PartialEq)]
pub enum GraphRunState {
    Progress {
        executed: Vec<NodeId>,
    },
    Waiting {
        node: NodeId,
        condition: WaitCondition,
    },
    Complete,
    Failed {
        node: NodeId,
        diagnostic: Diagnostic,
    },
    Aborted {
        node: NodeId,
        diagnostic: Diagnostic,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum GraphExecutionError {
    UnknownNode(NodeId),
    MissingOutput {
        from: PortRef,
        to: PortRef,
    },
    InvalidOutput {
        port: PortRef,
        expected: PortType,
        produced: PortType,
    },
    InvalidValue {
        port: PortRef,
        error: GraphValueError,
    },
    InvalidWait {
        node: NodeId,
        error: WaitError,
    },
    Deadlock,
}

impl fmt::Display for GraphExecutionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownNode(id) => {
                write!(formatter, "graph execution references unknown node {id:?}")
            }
            Self::MissingOutput { from, to } => {
                write!(
                    formatter,
                    "graph node {from:?} did not produce value for {to:?}"
                )
            }
            Self::InvalidOutput {
                port,
                expected,
                produced,
            } => write!(
                formatter,
                "graph output {port:?} produced {produced:?}, expected {expected:?}"
            ),
            Self::InvalidValue { port, error } => {
                write!(formatter, "graph output {port:?} is invalid: {error}")
            }
            Self::InvalidWait { node, error } => {
                write!(
                    formatter,
                    "graph node {node:?} returned an invalid wait: {error}"
                )
            }
            Self::Deadlock => write!(formatter, "graph has no runnable node or wait boundary"),
        }
    }
}

impl Error for GraphExecutionError {}

/// Deterministic graph scheduler for native blocks. Nodes are considered in
/// ascending `NodeId` order, so independent parallel-ready blocks execute in
/// a stable order while a Join naturally waits for all incoming edges.
#[derive(Debug, Clone, PartialEq)]
pub struct GraphRunner {
    graph: AutopilotGraph,
    statuses: BTreeMap<NodeId, BlockStatus>,
    outputs: BTreeMap<(NodeId, String), GraphValue>,
    waiting: BTreeMap<NodeId, WaitCondition>,
    terminal: Option<GraphRunState>,
}

impl GraphRunner {
    pub fn new(graph: AutopilotGraph) -> Result<Self, Vec<GraphError>> {
        graph.validate()?;
        Ok(Self {
            graph,
            statuses: BTreeMap::new(),
            outputs: BTreeMap::new(),
            waiting: BTreeMap::new(),
            terminal: None,
        })
    }

    pub fn graph(&self) -> &AutopilotGraph {
        &self.graph
    }

    pub fn status(&self, node: NodeId) -> Option<BlockStatus> {
        self.statuses.get(&node).copied()
    }

    pub fn output(&self, node: NodeId, port: &str) -> Option<&GraphValue> {
        self.outputs.get(&(node, port.to_string()))
    }

    pub fn is_complete(&self) -> bool {
        matches!(self.terminal, Some(GraphRunState::Complete))
    }

    pub fn abort(&mut self, diagnostic: Diagnostic) {
        if self.terminal.is_none() {
            self.terminal = Some(GraphRunState::Aborted {
                node: NodeId(0),
                diagnostic,
            });
        }
    }

    /// Run all currently ready blocks until the graph completes, parks, or
    /// reaches an explicit failure. `event` is delivered only to the node
    /// whose wait condition is parked, never polled through every node.
    pub fn poll<B: GraphBlock>(
        &mut self,
        now: SimTime,
        event: Option<&str>,
        block: &mut B,
    ) -> Result<GraphRunState, GraphExecutionError> {
        if !now.0.is_finite() {
            return Err(GraphExecutionError::Deadlock);
        }
        if let Some(terminal) = &self.terminal {
            return Ok(terminal.clone());
        }
        let ready_waits = self
            .waiting
            .iter()
            .filter_map(|(node, condition)| condition.is_due(now, event).then_some(*node))
            .collect::<Vec<_>>();
        for node in ready_waits {
            self.waiting.remove(&node);
            self.statuses.remove(&node);
        }

        let mut executed = Vec::new();
        loop {
            let Some(node) = self.next_ready_node()? else {
                if let Some((node, condition)) = self.waiting.iter().next() {
                    return Ok(GraphRunState::Waiting {
                        node: *node,
                        condition: condition.clone(),
                    });
                }
                if self.statuses.len() == self.graph.nodes.len()
                    && self
                        .statuses
                        .values()
                        .all(|status| *status == BlockStatus::Ok)
                {
                    self.terminal = Some(GraphRunState::Complete);
                    return Ok(GraphRunState::Complete);
                }
                if executed.is_empty() {
                    return Err(GraphExecutionError::Deadlock);
                }
                return Ok(GraphRunState::Progress { executed });
            };
            let inputs = self.inputs_for(node)?;
            let graph_node = self
                .graph
                .nodes
                .iter()
                .find(|candidate| candidate.id == node)
                .cloned()
                .ok_or(GraphExecutionError::UnknownNode(node))?;
            match block.execute(&graph_node, &inputs) {
                GraphNodeOutcome::Complete { outputs } => {
                    self.store_outputs(&graph_node, outputs)?;
                    self.statuses.insert(node, BlockStatus::Ok);
                    executed.push(node);
                }
                GraphNodeOutcome::Wait { condition } => {
                    condition
                        .validate()
                        .map_err(|error| GraphExecutionError::InvalidWait { node, error })?;
                    self.statuses.insert(node, BlockStatus::Waiting);
                    self.waiting.insert(node, condition);
                }
                GraphNodeOutcome::Fail { diagnostic } => {
                    self.statuses.insert(node, BlockStatus::Failed);
                    let state = GraphRunState::Failed { node, diagnostic };
                    self.terminal = Some(state.clone());
                    return Ok(state);
                }
                GraphNodeOutcome::Abort { diagnostic } => {
                    self.statuses.insert(node, BlockStatus::Failed);
                    let state = GraphRunState::Aborted { node, diagnostic };
                    self.terminal = Some(state.clone());
                    return Ok(state);
                }
            }
        }
    }

    fn next_ready_node(&self) -> Result<Option<NodeId>, GraphExecutionError> {
        Ok(self
            .graph
            .nodes
            .iter()
            .filter(|node| !self.statuses.contains_key(&node.id))
            .filter(|node| {
                self.graph
                    .edges
                    .iter()
                    .filter(|edge| edge.to.node == node.id)
                    .all(|edge| {
                        self.statuses
                            .get(&edge.from.node)
                            .is_some_and(|status| *status == BlockStatus::Ok)
                    })
            })
            .map(|node| node.id)
            .min())
    }

    fn inputs_for(
        &self,
        node: NodeId,
    ) -> Result<BTreeMap<String, GraphValue>, GraphExecutionError> {
        let mut inputs = BTreeMap::new();
        for edge in self.graph.edges.iter().filter(|edge| edge.to.node == node) {
            let key = (edge.from.node, edge.from.port.clone());
            let Some(value) = self.outputs.get(&key).cloned() else {
                return Err(GraphExecutionError::MissingOutput {
                    from: edge.from.clone(),
                    to: edge.to.clone(),
                });
            };
            inputs.insert(edge.to.port.clone(), value);
        }
        Ok(inputs)
    }

    fn store_outputs(
        &mut self,
        node: &GraphNode,
        outputs: BTreeMap<String, GraphValue>,
    ) -> Result<(), GraphExecutionError> {
        for (name, value) in outputs {
            let port = node
                .port(&name)
                .filter(|port| port.direction == PortDirection::Output)
                .ok_or_else(|| GraphExecutionError::InvalidOutput {
                    port: PortRef {
                        node: node.id,
                        port: name.clone(),
                    },
                    expected: PortType::Any,
                    produced: value.port_type(),
                })?;
            value
                .validate()
                .map_err(|error| GraphExecutionError::InvalidValue {
                    port: PortRef {
                        node: node.id,
                        port: name.clone(),
                    },
                    error,
                })?;
            if !port.ty.accepts(value.port_type()) {
                return Err(GraphExecutionError::InvalidOutput {
                    port: PortRef {
                        node: node.id,
                        port: name.clone(),
                    },
                    expected: port.ty,
                    produced: value.port_type(),
                });
            }
            self.outputs.insert((node.id, name), value);
        }
        Ok(())
    }
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

    pub fn is_empty(&self) -> bool {
        self.waits.is_empty()
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanExecutionMode {
    Baked,
    Live,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlanDeoptimizationReason {
    GuardInvalidated,
    ManualOverride,
    LiveInterrupt,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlanAction {
    Coast {
        until: SimTime,
    },
    Burn {
        until: SimTime,
        demand: ControlDemand,
    },
    Guidance {
        until: SimTime,
        intent: GuidanceIntent,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlanPoll {
    Action {
        mode: PlanExecutionMode,
        action: PlanAction,
    },
    Waiting {
        mode: PlanExecutionMode,
        condition: WaitCondition,
    },
    Complete {
        mode: PlanExecutionMode,
    },
}

/// Deterministic plan cursor used by the authority layer. It advances only
/// when the owner presents a simulation time or a relevant domain event; it
/// does not own a physics loop and never sleeps a worker thread.
#[derive(Debug, Clone, PartialEq)]
pub struct TrajectoryPlanRunner {
    plan: TrajectoryPlan,
    segment_index: usize,
    segment_start: SimTime,
    mode: PlanExecutionMode,
    deoptimization: Option<PlanDeoptimizationReason>,
}

impl TrajectoryPlanRunner {
    pub fn new(plan: TrajectoryPlan, start: SimTime) -> Result<Self, PlanError> {
        plan.validate()?;
        validate_plan_time(start)?;
        let mode = match plan.bakeability {
            Bakeability::Live => PlanExecutionMode::Live,
            Bakeability::Pure | Bakeability::Guarded => PlanExecutionMode::Baked,
        };
        Ok(Self {
            plan,
            segment_index: 0,
            segment_start: start,
            mode,
            deoptimization: None,
        })
    }

    pub fn plan(&self) -> &TrajectoryPlan {
        &self.plan
    }

    pub fn mode(&self) -> PlanExecutionMode {
        self.mode
    }

    pub fn segment_index(&self) -> usize {
        self.segment_index
    }

    pub fn deoptimization_reason(&self) -> Option<PlanDeoptimizationReason> {
        self.deoptimization
    }

    /// Switch the current plan to live execution after a guard, pilot
    /// override, or other runtime condition invalidates baked assumptions.
    /// The cursor is retained so the authority can continue from the same
    /// declarative segment under live control.
    pub fn deoptimize(&mut self, reason: PlanDeoptimizationReason) -> bool {
        let changed = self.mode != PlanExecutionMode::Live;
        self.mode = PlanExecutionMode::Live;
        self.deoptimization = Some(reason);
        changed
    }

    /// Return the current action, park on a guard, or advance through all
    /// segments that are already complete at `now`.
    pub fn poll(&mut self, now: SimTime, event: Option<&str>) -> Result<PlanPoll, PlanError> {
        validate_plan_time(now)?;
        loop {
            let Some(segment) = self.plan.segments.get(self.segment_index) else {
                return Ok(PlanPoll::Complete { mode: self.mode });
            };
            match segment {
                TrajectorySegment::Coast { duration_s } => {
                    let until = plan_segment_end(self.segment_start, *duration_s)?;
                    if now.0 >= until.0 {
                        self.segment_index += 1;
                        self.segment_start = until;
                        continue;
                    }
                    return Ok(PlanPoll::Action {
                        mode: self.mode,
                        action: PlanAction::Coast { until },
                    });
                }
                TrajectorySegment::Burn { duration_s, demand } => {
                    let until = plan_segment_end(self.segment_start, *duration_s)?;
                    if now.0 >= until.0 {
                        self.segment_index += 1;
                        self.segment_start = until;
                        continue;
                    }
                    return Ok(PlanPoll::Action {
                        mode: self.mode,
                        action: PlanAction::Burn {
                            until,
                            demand: *demand,
                        },
                    });
                }
                TrajectorySegment::Guidance { duration_s, intent } => {
                    let until = plan_segment_end(self.segment_start, *duration_s)?;
                    if now.0 >= until.0 {
                        self.segment_index += 1;
                        self.segment_start = until;
                        continue;
                    }
                    return Ok(PlanPoll::Action {
                        mode: self.mode,
                        action: PlanAction::Guidance {
                            until,
                            intent: intent.clone(),
                        },
                    });
                }
                TrajectorySegment::Wait { condition } => {
                    if condition.is_due(now, event) {
                        self.segment_index += 1;
                        self.segment_start = now;
                        continue;
                    }
                    return Ok(PlanPoll::Waiting {
                        mode: self.mode,
                        condition: condition.clone(),
                    });
                }
            }
        }
    }
}

fn validate_plan_time(time: SimTime) -> Result<(), PlanError> {
    time.0
        .is_finite()
        .then_some(())
        .ok_or(PlanError::InvalidStartTime(time.0))
}

fn plan_segment_end(start: SimTime, duration: f64) -> Result<SimTime, PlanError> {
    let end = start.offset(duration);
    end.0
        .is_finite()
        .then_some(end)
        .ok_or(PlanError::TimeOverflow { start, duration })
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
    InvalidStartTime(f64),
    TimeOverflow { start: SimTime, duration: f64 },
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
            Self::InvalidStartTime(time) => write!(formatter, "invalid plan time {time}"),
            Self::TimeOverflow { start, duration } => write!(
                formatter,
                "plan segment end overflows from start {} by {} seconds",
                start.0, duration
            ),
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
    use thessa_flight_control::{ActuatorGroup, PilotAxes};

    #[derive(Default)]
    struct TestBlock {
        calls: BTreeMap<NodeId, u32>,
    }

    impl GraphBlock for TestBlock {
        fn execute(
            &mut self,
            node: &GraphNode,
            inputs: &BTreeMap<String, GraphValue>,
        ) -> GraphNodeOutcome {
            let calls = self.calls.entry(node.id).or_default();
            *calls += 1;
            if node.kind == NodeKind::Wait && *calls == 1 {
                return GraphNodeOutcome::Wait {
                    condition: WaitCondition::At(SimTime(10.0)),
                };
            }
            let number = inputs
                .values()
                .filter_map(|value| match value {
                    GraphValue::Number(value) => Some(*value),
                    _ => None,
                })
                .sum::<f64>();
            let mut outputs = BTreeMap::new();
            for port in node
                .ports
                .iter()
                .filter(|port| port.direction == PortDirection::Output)
            {
                outputs.insert(
                    port.name.clone(),
                    match port.ty {
                        PortType::Number => {
                            GraphValue::Number(if number == 0.0 { 2.0 } else { number })
                        }
                        _ => GraphValue::Unit,
                    },
                );
            }
            GraphNodeOutcome::Complete { outputs }
        }
    }

    #[test]
    fn surface_sites_normalize_centers_but_keep_landing_and_impact_types_distinct() {
        let landing = LandingSite::new([0.0, 2.0, 0.0], 125.0).unwrap();
        let impact = ImpactSite::new([0.0, 2.0, 0.0], 125.0).unwrap();
        assert_eq!(landing.center_dir, [0.0, 1.0, 0.0]);
        assert_eq!(impact.center_dir, landing.center_dir);
        assert_eq!(landing.radius_m, impact.radius_m);
        assert!(!PortType::LandingSite.accepts(PortType::ImpactSite));
        assert!(PortType::LandingSite.accepts(PortType::LandingSite));
    }

    #[test]
    fn surface_sites_reject_degenerate_centers_and_radii() {
        assert_eq!(
            LandingSite::new([0.0, 0.0, 0.0], 1.0),
            Err(SiteError::InvalidCenter)
        );
        assert_eq!(
            ImpactSite::new([1.0, 0.0, 0.0], -1.0),
            Err(SiteError::InvalidRadius)
        );
        assert!(
            LandingSite {
                center_dir: [2.0, 0.0, 0.0],
                radius_m: 1.0,
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn graph_runner_executes_sequence_and_join_in_stable_order() {
        let graph = AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(3),
                    name: "right".into(),
                    kind: NodeKind::Custom {
                        bakeability: Bakeability::Pure,
                        wait_capable: false,
                    },
                    ports: vec![
                        Port::input("value", PortType::Number, true),
                        Port::output("right", PortType::Number),
                    ],
                },
                GraphNode {
                    id: NodeId(1),
                    name: "source".into(),
                    kind: NodeKind::Source,
                    ports: vec![Port::output("value", PortType::Number)],
                },
                GraphNode {
                    id: NodeId(4),
                    name: "join".into(),
                    kind: NodeKind::Join,
                    ports: vec![
                        Port::input("left", PortType::Number, true),
                        Port::input("right", PortType::Number, true),
                        Port::output("sum", PortType::Number),
                    ],
                },
                GraphNode {
                    id: NodeId(2),
                    name: "left".into(),
                    kind: NodeKind::Custom {
                        bakeability: Bakeability::Pure,
                        wait_capable: false,
                    },
                    ports: vec![
                        Port::input("value", PortType::Number, true),
                        Port::output("left", PortType::Number),
                    ],
                },
            ],
            edges: vec![
                GraphEdge {
                    from: PortRef {
                        node: NodeId(1),
                        port: "value".into(),
                    },
                    to: PortRef {
                        node: NodeId(2),
                        port: "value".into(),
                    },
                },
                GraphEdge {
                    from: PortRef {
                        node: NodeId(1),
                        port: "value".into(),
                    },
                    to: PortRef {
                        node: NodeId(3),
                        port: "value".into(),
                    },
                },
                GraphEdge {
                    from: PortRef {
                        node: NodeId(2),
                        port: "left".into(),
                    },
                    to: PortRef {
                        node: NodeId(4),
                        port: "left".into(),
                    },
                },
                GraphEdge {
                    from: PortRef {
                        node: NodeId(3),
                        port: "right".into(),
                    },
                    to: PortRef {
                        node: NodeId(4),
                        port: "right".into(),
                    },
                },
            ],
        };
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = TestBlock::default();
        assert_eq!(
            runner.poll(SimTime(0.0), None, &mut block).unwrap(),
            GraphRunState::Complete
        );
        assert_eq!(
            runner.output(NodeId(4), "sum"),
            Some(&GraphValue::Number(4.0))
        );
        assert_eq!(runner.status(NodeId(2)), Some(BlockStatus::Ok));
        assert_eq!(runner.status(NodeId(3)), Some(BlockStatus::Ok));
    }

    #[test]
    fn graph_runner_parks_a_wait_and_resumes_only_on_its_condition() {
        let graph = AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(1),
                    name: "wait".into(),
                    kind: NodeKind::Wait,
                    ports: vec![Port::output("done", PortType::Unit)],
                },
                GraphNode {
                    id: NodeId(2),
                    name: "sink".into(),
                    kind: NodeKind::Sink,
                    ports: vec![Port::input("done", PortType::Unit, true)],
                },
            ],
            edges: vec![GraphEdge {
                from: PortRef {
                    node: NodeId(1),
                    port: "done".into(),
                },
                to: PortRef {
                    node: NodeId(2),
                    port: "done".into(),
                },
            }],
        };
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = TestBlock::default();
        assert!(matches!(
            runner.poll(SimTime(0.0), None, &mut block).unwrap(),
            GraphRunState::Waiting {
                node: NodeId(1),
                condition: WaitCondition::At(SimTime(10.0))
            }
        ));
        assert!(matches!(
            runner.poll(SimTime(9.0), None, &mut block).unwrap(),
            GraphRunState::Waiting {
                node: NodeId(1),
                ..
            }
        ));
        assert_eq!(
            runner.poll(SimTime(10.0), None, &mut block).unwrap(),
            GraphRunState::Complete
        );
        assert_eq!(block.calls.get(&NodeId(1)), Some(&2));
    }

    #[test]
    fn graph_runner_keeps_independent_parallel_work_running_around_waits() {
        let graph = AutopilotGraph {
            nodes: vec![
                GraphNode {
                    id: NodeId(1),
                    name: "waiting-branch".into(),
                    kind: NodeKind::Wait,
                    ports: vec![Port::output("done", PortType::Unit)],
                },
                GraphNode {
                    id: NodeId(2),
                    name: "parallel-branch".into(),
                    kind: NodeKind::Source,
                    ports: vec![Port::output("done", PortType::Unit)],
                },
            ],
            edges: Vec::new(),
        };
        let mut runner = GraphRunner::new(graph).unwrap();
        let mut block = TestBlock::default();
        assert!(matches!(
            runner.poll(SimTime(0.0), None, &mut block).unwrap(),
            GraphRunState::Waiting {
                node: NodeId(1),
                condition: WaitCondition::At(SimTime(10.0))
            }
        ));
        assert_eq!(block.calls.get(&NodeId(1)), Some(&1));
        assert_eq!(block.calls.get(&NodeId(2)), Some(&1));
        assert_eq!(runner.status(NodeId(2)), Some(BlockStatus::Ok));
        assert_eq!(
            runner.poll(SimTime(10.0), None, &mut block).unwrap(),
            GraphRunState::Complete
        );
    }

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

    #[test]
    fn plan_runner_emits_absolute_boundaries_for_baked_segments() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(7),
            segments: vec![
                TrajectorySegment::Coast { duration_s: 5.0 },
                TrajectorySegment::Burn {
                    duration_s: 2.0,
                    demand: ControlDemand::zero(),
                },
            ],
            bakeability: Bakeability::Pure,
        };
        let mut runner = TrajectoryPlanRunner::new(plan, SimTime(100.0)).unwrap();

        assert!(matches!(
            runner.poll(SimTime(100.0), None).unwrap(),
            PlanPoll::Action {
                mode: PlanExecutionMode::Baked,
                action: PlanAction::Coast {
                    until: SimTime(105.0)
                },
            }
        ));
        assert!(matches!(
            runner.poll(SimTime(105.0), None).unwrap(),
            PlanPoll::Action {
                mode: PlanExecutionMode::Baked,
                action: PlanAction::Burn {
                    until: SimTime(107.0),
                    ..
                },
            }
        ));
        assert_eq!(
            runner.poll(SimTime(107.0), None).unwrap(),
            PlanPoll::Complete {
                mode: PlanExecutionMode::Baked
            }
        );
    }

    #[test]
    fn plan_runner_parks_on_a_guard_and_resumes_after_the_event() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(8),
            segments: vec![
                TrajectorySegment::Wait {
                    condition: WaitCondition::Event("impact".into()),
                },
                TrajectorySegment::Guidance {
                    duration_s: 2.0,
                    intent: GuidanceIntent::ManualAxes(PilotAxes::default()),
                },
            ],
            bakeability: Bakeability::Guarded,
        };
        let mut runner = TrajectoryPlanRunner::new(plan, SimTime(20.0)).unwrap();
        assert!(matches!(
            runner.poll(SimTime(20.0), Some("other")),
            Ok(PlanPoll::Waiting {
                mode: PlanExecutionMode::Baked,
                condition: WaitCondition::Event(ref name),
            }) if name == "impact"
        ));
        assert!(matches!(
            runner.poll(SimTime(21.0), Some("impact")).unwrap(),
            PlanPoll::Action {
                mode: PlanExecutionMode::Baked,
                action: PlanAction::Guidance {
                    until: SimTime(23.0),
                    ..
                },
            }
        ));
    }

    #[test]
    fn plan_runner_deoptimizes_without_losing_its_cursor() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(9),
            segments: vec![TrajectorySegment::Coast { duration_s: 4.0 }],
            bakeability: Bakeability::Guarded,
        };
        let mut runner = TrajectoryPlanRunner::new(plan, SimTime(30.0)).unwrap();
        assert_eq!(runner.segment_index(), 0);
        assert!(runner.deoptimize(PlanDeoptimizationReason::GuardInvalidated));
        assert_eq!(runner.mode(), PlanExecutionMode::Live);
        assert_eq!(
            runner.deoptimization_reason(),
            Some(PlanDeoptimizationReason::GuardInvalidated)
        );
        assert!(matches!(
            runner.poll(SimTime(30.0), None).unwrap(),
            PlanPoll::Action {
                mode: PlanExecutionMode::Live,
                action: PlanAction::Coast {
                    until: SimTime(34.0)
                },
            }
        ));
    }

    #[test]
    fn plan_runner_rejects_non_finite_start_time() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(10),
            segments: vec![TrajectorySegment::Coast { duration_s: 1.0 }],
            bakeability: Bakeability::Pure,
        };
        assert!(matches!(
            TrajectoryPlanRunner::new(plan, SimTime(f64::NAN)),
            Err(PlanError::InvalidStartTime(time)) if time.is_nan()
        ));
    }

    #[test]
    fn plan_runner_rejects_an_overflowing_segment_boundary() {
        let plan = TrajectoryPlan {
            id: TrajectoryPlanId(11),
            segments: vec![TrajectorySegment::Coast {
                duration_s: f64::MAX,
            }],
            bakeability: Bakeability::Pure,
        };
        let mut runner = TrajectoryPlanRunner::new(plan, SimTime(f64::MAX / 2.0)).unwrap();
        assert!(matches!(
            runner.poll(SimTime(f64::MAX / 2.0), None),
            Err(PlanError::TimeOverflow { .. })
        ));
    }
}
