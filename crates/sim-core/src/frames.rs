use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::BodyId;

/// Explicit frame labels prevent render-local coordinates from becoming
/// authoritative universe coordinates by accident.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReferenceFrame {
    SystemBarycentricInertial,
    BodyCenteredInertial(BodyId),
    BodyFixed(BodyId),
    LocalTangent(BodyId),
    VehicleBody(u64),
    RenderLocal,
}

/// Position and velocity with a first-class reference-frame label.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StateVector {
    pub position: DVec3,
    pub velocity: DVec3,
    pub frame: ReferenceFrame,
}

impl StateVector {
    pub const fn new(position: DVec3, velocity: DVec3, frame: ReferenceFrame) -> Self {
        Self {
            position,
            velocity,
            frame,
        }
    }
}
