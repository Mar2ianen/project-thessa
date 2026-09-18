//! `ManeuverPlan`: the typed port contract between planning blocks
//! (`PlanTransfer`, `Circularize`) and execution (`ExecuteManeuver`).
//!
//! A plan is an ordered schedule of impulsive nodes in inertial space.
//! Impulsive here describes the PLAN representation, not the execution:
//! the executor burns finite engines over time to realize each node.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use thessa_sim_core::{BodyId, SimTime};

/// One impulsive node: burn `delta_v_mps` (inertial frame) at `epoch`.
/// Nodes must order non-decreasing in time; simultaneous nodes are merged
/// by validation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ManeuverNode {
    pub epoch: SimTime,
    pub delta_v_mps: DVec3,
}

impl ManeuverNode {
    pub fn new(epoch: SimTime, delta_v_mps: DVec3) -> Result<Self, PlanError> {
        if !epoch.0.is_finite() || !delta_v_mps.is_finite() {
            return Err(PlanError::NonFiniteNode);
        }
        Ok(Self { epoch, delta_v_mps })
    }

    pub fn magnitude_mps(&self) -> f64 {
        self.delta_v_mps.length()
    }
}

/// A gravity-assist encounter on the flown route: informational only, never
/// executed (an unpowered flyby needs no burn; a powered one carries its
/// burn as a regular node at the same epoch). Lets the executor, telemetry
/// and downstream blocks see WHERE the free bend happened.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlybyEvent {
    pub body: BodyId,
    pub epoch: SimTime,
    /// Periapsis radius the route was corrected to (m).
    pub periapsis_m: f64,
    /// Powered-flyby burn applied at periapsis (m/s), 0 when unpowered.
    pub burn_mps: f64,
}

/// Ordered burn schedule plus the departure state it was planned from, so
/// execution can verify it is still flying the right plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ManeuverPlan {
    pub nodes: Vec<ManeuverNode>,
    pub departure_position_m: DVec3,
    pub departure_velocity_mps: DVec3,
    pub departure_epoch: SimTime,
    /// Predicted arrival miss distance (m) from exact revalidation, when
    /// the producing search performed one. Plans built by pure two-body
    /// ops carry `None`: unvalidated, fly only after `search` revalidates.
    pub predicted_miss_m: Option<f64>,
    /// Gravity assists on the route (empty for direct transfers).
    pub flybys: Vec<FlybyEvent>,
}

impl ManeuverPlan {
    pub fn new(
        nodes: Vec<ManeuverNode>,
        departure_position_m: DVec3,
        departure_velocity_mps: DVec3,
        departure_epoch: SimTime,
    ) -> Result<Self, PlanError> {
        if !departure_position_m.is_finite()
            || !departure_velocity_mps.is_finite()
            || !departure_epoch.0.is_finite()
        {
            return Err(PlanError::NonFiniteNode);
        }
        for pair in nodes.windows(2) {
            if pair[1].epoch.0 < pair[0].epoch.0 {
                return Err(PlanError::UnorderedNodes);
            }
        }
        Ok(Self {
            nodes,
            departure_position_m,
            departure_velocity_mps,
            departure_epoch,
            predicted_miss_m: None,
            flybys: Vec::new(),
        })
    }

    /// Attach the flown gravity-assist encounters (in time order).
    pub fn with_flybys(mut self, flybys: Vec<FlybyEvent>) -> Self {
        self.flybys = flybys;
        self
    }

    pub fn total_dv_mps(&self) -> f64 {
        self.nodes.iter().map(ManeuverNode::magnitude_mps).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.iter().all(|node| node.magnitude_mps() == 0.0)
    }
}

/// Validation outcome for graph-block admission: either executable or a
/// named reason the block must take its abort path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanValidation {
    Executable,
    Empty,
    Stale { now_s: f64, first_node_s: f64 },
}

impl ManeuverPlan {
    pub fn validate_for_execution(&self, now: SimTime) -> PlanValidation {
        if self.is_empty() {
            return PlanValidation::Empty;
        }
        match self.nodes.first() {
            Some(node) if node.epoch.0 < now.0 => PlanValidation::Stale {
                now_s: now.0,
                first_node_s: node.epoch.0,
            },
            Some(_) => PlanValidation::Executable,
            None => PlanValidation::Empty,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PlanError {
    NonFiniteNode,
    UnorderedNodes,
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteNode => write!(formatter, "non-finite maneuver node"),
            Self::UnorderedNodes => write!(formatter, "maneuver nodes out of time order"),
        }
    }
}

impl std::error::Error for PlanError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn total_dv_sums_nodes() {
        let plan = ManeuverPlan::new(
            vec![
                ManeuverNode::new(SimTime(100.0), DVec3::new(100.0, 0.0, 0.0)).unwrap(),
                ManeuverNode::new(SimTime(200.0), DVec3::new(0.0, 50.0, 0.0)).unwrap(),
            ],
            DVec3::ZERO,
            DVec3::X,
            SimTime(0.0),
        )
        .unwrap();
        assert_eq!(plan.total_dv_mps(), 150.0);
    }

    #[test]
    fn unordered_nodes_rejected() {
        let nodes = vec![
            ManeuverNode::new(SimTime(200.0), DVec3::X).unwrap(),
            ManeuverNode::new(SimTime(100.0), DVec3::X).unwrap(),
        ];
        assert_eq!(
            ManeuverPlan::new(nodes, DVec3::ZERO, DVec3::X, SimTime(0.0)),
            Err(PlanError::UnorderedNodes)
        );
    }

    #[test]
    fn stale_plan_takes_abort_path() {
        let plan = ManeuverPlan::new(
            vec![ManeuverNode::new(SimTime(100.0), DVec3::X * 10.0).unwrap()],
            DVec3::ZERO,
            DVec3::X,
            SimTime(0.0),
        )
        .unwrap();
        assert_eq!(
            plan.validate_for_execution(SimTime(50.0)),
            PlanValidation::Executable
        );
        assert!(matches!(
            plan.validate_for_execution(SimTime(150.0)),
            PlanValidation::Stale { .. }
        ));
    }
}
