use serde::{Deserialize, Serialize};

/// Simulation time measured from the versioned ephemeris epoch, in SI seconds.
///
/// This type intentionally does not expose wall-clock time. A `SimTime` value
/// is deterministic input to all physics evaluation.
#[derive(Debug, Clone, Copy, Default, PartialEq, PartialOrd, Serialize, Deserialize)]
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

/// Shared world-clock lattice. Integration stages may sample between ticks;
/// authoritative scheduled updates count integer ticks.
pub const WORLD_TICK_HZ: u64 = 120;
pub const WORLD_TICK_S: f64 = 1.0 / WORLD_TICK_HZ as f64;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct WorldTick(pub u64);
impl WorldTick {
    pub fn time(self) -> SimTime {
        SimTime(self.0 as f64 * WORLD_TICK_S)
    }
    /// Restore a legacy seconds-based state onto the nearest world tick.
    /// Reject non-finite/negative epochs and counts beyond exact f64 integers.
    pub fn from_time(time: SimTime) -> Option<Self> {
        let ticks = (time.0 * WORLD_TICK_HZ as f64).round();
        (time.0.is_finite() && time.0 >= 0.0 && ticks <= (1_u64 << 52) as f64)
            .then_some(Self(ticks as u64))
    }
    pub fn checked_add(self, ticks: u64) -> Option<Self> {
        self.0
            .checked_add(ticks)
            .filter(|t| *t <= (1_u64 << 52))
            .map(Self)
    }
}
