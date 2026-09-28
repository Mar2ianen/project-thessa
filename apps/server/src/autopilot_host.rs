//! QuickJS scheduler and structural native graph host.

use std::collections::BTreeMap;
use thessa_autopilot::{
    GraphBlock, GraphControlAction, GraphNode, GraphNodeConfig, GraphNodeOutcome, GraphValue,
    NodeKind, PortDirection, PortType, WaitCondition,
};
use thessa_autopilot_js::{ScriptEngine, ScriptLimits, ScriptScheduler};

/// QuickJS is intentionally pinned to the authoritative driver thread:
/// `rquickjs` contexts and persistent continuations are not `Send`.
pub(super) struct AutopilotHost {
    pub(super) scheduler: ScriptScheduler,
    pub(super) engine: ScriptEngine,
}

/// Structural native graph host used until domain-specific blocks are
/// registered. It forwards matching typed values and makes a wait node an
/// event boundary; it never receives mutable access to flight state.
#[derive(Debug, Default)]
pub(super) struct NativeGraphBlock {
    waited: std::collections::BTreeSet<thessa_autopilot::NodeId>,
    control_actions: Vec<GraphControlAction>,
    parked_phases: Vec<(thessa_autopilot::NodeId, String, GraphNodeConfig)>,
}

/// A parked autopilot phase node awaiting its law's completion.
#[derive(Debug, Clone)]
pub(super) struct PhasePark {
    pub(super) node: thessa_autopilot::NodeId,
    pub(super) name: String,
    pub(super) config: GraphNodeConfig,
    pub(super) parked_at_s: f64,
}

impl GraphBlock for NativeGraphBlock {
    fn execute(
        &mut self,
        node: &GraphNode,
        inputs: &BTreeMap<String, GraphValue>,
    ) -> GraphNodeOutcome {
        let waiting_node = matches!(node.kind, NodeKind::Wait)
            || matches!(
                node.kind,
                NodeKind::Custom {
                    wait_capable: true,
                    ..
                }
            );
        if waiting_node && self.waited.insert(node.id) {
            // Parked phase nodes report for the per-tick authority law;
            // plain waits carry no steering parameters.
            if let Some(config) = node.config.as_ref()
                && matches!(
                    config,
                    GraphNodeConfig::AscentPhase { .. }
                        | GraphNodeConfig::LandingPhase { .. }
                        | GraphNodeConfig::RendezvousPhase { .. }
                        | GraphNodeConfig::ExecutePhase { .. }
                )
            {
                self.parked_phases
                    .push((node.id, node.name.clone(), config.clone()));
            }
            return GraphNodeOutcome::Wait {
                condition: match node.config.as_ref() {
                    Some(GraphNodeConfig::Wait { condition }) => condition.clone(),
                    _ => WaitCondition::Event(node.name.clone()),
                },
            };
        }
        match node.config.as_ref() {
            Some(GraphNodeConfig::Guidance { intent, propulsion }) => {
                self.control_actions.push(GraphControlAction::Guidance {
                    intent: intent.clone(),
                    propulsion: *propulsion,
                })
            }
            Some(GraphNodeConfig::Demand { demand }) => {
                self.control_actions
                    .push(GraphControlAction::Demand { demand: *demand });
            }
            _ => {}
        }
        let mut outputs = BTreeMap::new();
        for port in node
            .ports
            .iter()
            .filter(|port| port.direction == PortDirection::Output)
        {
            let configured_number = match node.config.as_ref() {
                Some(GraphNodeConfig::Number { value }) if port.ty == PortType::Number => {
                    Some(GraphValue::Number(*value))
                }
                _ => None,
            };
            let value = configured_number.or_else(|| {
                inputs
                    .get(&port.name)
                    .cloned()
                    .or_else(|| {
                        (inputs.len() == 1)
                            .then(|| inputs.values().next().cloned())
                            .flatten()
                    })
                    .or(match port.ty {
                        PortType::Unit => Some(GraphValue::Unit),
                        PortType::Bool => Some(GraphValue::Bool(false)),
                        PortType::Number => Some(GraphValue::Number(0.0)),
                        PortType::Any => Some(GraphValue::Unit),
                        ty => Some(GraphValue::Typed(ty)),
                    })
            });
            let Some(value) = value else {
                return GraphNodeOutcome::Fail {
                    diagnostic: thessa_autopilot::Diagnostic {
                        kind: thessa_autopilot::DiagnosticKind::NoSolution,
                        code: "GRAPH_OUTPUT_UNRESOLVED".into(),
                        message: format!(
                            "graph node {:?} has no value for output {:?}",
                            node.id, port.name
                        ),
                        value: None,
                        limit: None,
                    },
                };
            };
            outputs.insert(port.name.clone(), value);
        }
        GraphNodeOutcome::Complete { outputs }
    }

    fn take_control_actions(&mut self) -> Vec<GraphControlAction> {
        std::mem::take(&mut self.control_actions)
    }
}

impl NativeGraphBlock {
    pub(super) fn take_parked_phases(
        &mut self,
    ) -> Vec<(thessa_autopilot::NodeId, String, GraphNodeConfig)> {
        std::mem::take(&mut self.parked_phases)
    }
}

impl AutopilotHost {
    pub(super) fn new() -> Result<Self, String> {
        Ok(Self {
            engine: ScriptEngine::new(ScriptLimits::default())
                .map_err(|error| format!("autopilot init: {error}"))?,
            scheduler: ScriptScheduler::default(),
        })
    }
}
