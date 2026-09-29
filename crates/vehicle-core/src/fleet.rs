//! Server-authoritative fleet identity and dock-graph ownership.
//!
//! [`VehicleId`] is the stable wire identity for one authoritative vehicle
//! (`0` is the primary single-vehicle fast path). [`DockGraph`] owns the
//! persisted docking protocol sessions per vehicle pair; the contact
//! backend owns the mechanical fixed joints and the server maps them onto
//! these pairs. Everything here is solver-free and fails closed: unknown
//! vehicles, self-docking, and duplicate sessions are errors, never silent
//! merges.

use std::{collections::BTreeMap, error::Error, fmt};

use serde::{Deserialize, Serialize};

use crate::{DockingError, DockingPortSpec, DockingSession};

/// Stable identity for one authoritative vehicle on the wire and in the
/// dock graph. Zero is the primary vehicle every legacy peer knows.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Default,
)]
pub struct VehicleId(pub u32);

impl VehicleId {
    pub const PRIMARY: Self = Self(0);

    pub const fn new(id: u32) -> Self {
        Self(id)
    }

    pub const fn is_primary(self) -> bool {
        self.0 == 0
    }
}

impl fmt::Display for VehicleId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "vehicle-{}", self.0)
    }
}

/// Fleet ownership failure modes.
#[derive(Debug, Clone, PartialEq)]
pub enum FleetError {
    UnknownVehicle(String),
    SelfDock,
    DuplicateSession(String),
    UnknownSession(String),
    Docking(String),
}

impl fmt::Display for FleetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownVehicle(message) => write!(formatter, "unknown vehicle: {message}"),
            Self::SelfDock => write!(formatter, "a vehicle cannot dock with itself"),
            Self::DuplicateSession(message) => {
                write!(formatter, "duplicate docking session: {message}")
            }
            Self::UnknownSession(message) => {
                write!(formatter, "unknown docking session: {message}")
            }
            Self::Docking(message) => write!(formatter, "docking error: {message}"),
        }
    }
}

impl Error for FleetError {}

impl From<DockingError> for FleetError {
    fn from(error: DockingError) -> Self {
        Self::Docking(error.to_string())
    }
}

/// Persisted docking sessions keyed by ordered vehicle pair. Joint handles
/// stay with the contact backend; this graph answers who is docked to whom
/// and in which protocol state.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct DockGraph {
    sessions: BTreeMap<(u32, u32), DockingSession>,
}

impl DockGraph {
    pub fn new() -> Self {
        Self::default()
    }

    /// Canonical pair key: ordered ids, never a vehicle with itself.
    pub fn ordered(a: VehicleId, b: VehicleId) -> Result<(u32, u32), FleetError> {
        if a == b {
            return Err(FleetError::SelfDock);
        }
        Ok((a.0.min(b.0), a.0.max(b.0)))
    }

    /// Open a session for a compatible port pair. The ports travel with the
    /// session so later kinematics gates read the authored limits, not
    /// caller-supplied world state.
    pub fn begin_session(
        &mut self,
        a: VehicleId,
        port_a: DockingPortSpec,
        b: VehicleId,
        port_b: DockingPortSpec,
        pressure_equalization_required_s: f64,
    ) -> Result<(), FleetError> {
        let pair = Self::ordered(a, b)?;
        if self.sessions.contains_key(&pair) {
            return Err(FleetError::DuplicateSession(format!(
                "vehicles {} and {} already share a session",
                pair.0, pair.1
            )));
        }
        // Keep port-to-vehicle assignment stable under key ordering: the
        // lower id always owns port_a.
        let (port_first, port_second) = if a.0 < b.0 {
            (port_a, port_b)
        } else {
            (port_b, port_a)
        };
        let session =
            DockingSession::new(port_first, port_second, pressure_equalization_required_s)?;
        self.sessions.insert(pair, session);
        Ok(())
    }

    pub fn session(&self, a: VehicleId, b: VehicleId) -> Result<&DockingSession, FleetError> {
        let pair = Self::ordered(a, b)?;
        self.sessions.get(&pair).ok_or_else(|| {
            FleetError::UnknownSession(format!("no session for vehicles {} and {}", pair.0, pair.1))
        })
    }

    pub fn session_mut(
        &mut self,
        a: VehicleId,
        b: VehicleId,
    ) -> Result<&mut DockingSession, FleetError> {
        let pair = Self::ordered(a, b)?;
        self.sessions.get_mut(&pair).ok_or_else(|| {
            FleetError::UnknownSession(format!("no session for vehicles {} and {}", pair.0, pair.1))
        })
    }

    /// Drop a session, returning it when the pair was tracked.
    pub fn end_session(
        &mut self,
        a: VehicleId,
        b: VehicleId,
    ) -> Result<Option<DockingSession>, FleetError> {
        let pair = Self::ordered(a, b)?;
        Ok(self.sessions.remove(&pair))
    }

    /// All tracked pairs in canonical order.
    pub fn pairs(&self) -> Vec<(u32, u32)> {
        self.sessions.keys().copied().collect()
    }

    pub fn has_vehicle(&self, id: VehicleId) -> bool {
        self.sessions
            .keys()
            .any(|pair| pair.0 == id.0 || pair.1 == id.0)
    }

    pub fn is_empty(&self) -> bool {
        self.sessions.is_empty()
    }

    pub fn len(&self) -> usize {
        self.sessions.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};

    fn port(id: &str) -> DockingPortSpec {
        DockingPortSpec::d1(id, DVec3::X, DQuat::IDENTITY).expect("port")
    }

    #[test]
    fn dock_graph_orders_pairs_and_rejects_self_and_duplicates() {
        let mut graph = DockGraph::new();
        assert!(graph.is_empty());
        graph
            .begin_session(
                VehicleId::new(3),
                port("a"),
                VehicleId::new(1),
                port("b"),
                1.0,
            )
            .expect("session");
        assert_eq!(graph.len(), 1);
        // Canonical order regardless of argument order; lower id owns port_a.
        assert_eq!(graph.pairs(), vec![(1, 3)]);
        assert_eq!(
            graph
                .session(VehicleId::new(1), VehicleId::new(3))
                .unwrap()
                .port_a
                .id,
            "b"
        );
        assert!(graph.has_vehicle(VehicleId::new(3)));
        assert!(!graph.has_vehicle(VehicleId::new(7)));
        assert!(
            graph
                .begin_session(
                    VehicleId::new(1),
                    port("a"),
                    VehicleId::new(3),
                    port("b"),
                    1.0
                )
                .is_err()
        );
        assert!(
            graph
                .begin_session(
                    VehicleId::new(2),
                    port("a"),
                    VehicleId::new(2),
                    port("b"),
                    1.0
                )
                .is_err()
        );
        assert!(graph.session(VehicleId::new(1), VehicleId::new(2)).is_err());
        let removed = graph
            .end_session(VehicleId::new(3), VehicleId::new(1))
            .expect("ordered end");
        assert!(removed.is_some());
        assert!(graph.is_empty());
    }
}
