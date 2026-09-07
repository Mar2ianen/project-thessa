use serde::{Deserialize, Serialize};

/// Simulation time measured from the versioned ephemeris epoch, in SI seconds.
///
/// This type intentionally does not expose wall-clock time. A `SimTime` value
/// is deterministic input to all physics evaluation.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SimTime(pub f64);

impl SimTime {
    pub const EPOCH: Self = Self(0.0);

    pub const fn seconds(self) -> f64 {
        self.0
    }

    pub const fn offset(self, seconds: f64) -> Self {
        Self(self.0 + seconds)
    }
}
