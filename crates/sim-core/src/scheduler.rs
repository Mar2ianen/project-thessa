//! Event scheduler in simulation time (AGENTS.md section 8).
//!
//! Sleeping programs must not poll every physics tick: the on-rails bake
//! arms its [`OnRailsWake`] here, maneuver nodes and alarms arm alongside,
//! and the flight loop drains only due events as [`SimTime`] advances.
//! Pure data, deterministic order, no wall clock, no async runtime.

use glam::DVec3;

use crate::{BodyId, SimTime};

/// What fires when its epoch arrives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScheduledKind {
    /// Baked coast predicts surface contact: arm contact handling instead of
    /// checking distance every tick.
    RailsImpact { body: BodyId },
    /// Baked coast runs out: extend the bake instead of watching the horizon.
    RailsHorizon,
    /// Impulsive maneuver node (reserved for the maneuver planner; the flight
    /// loop does not execute it yet).
    ManeuverNode { delta_v_mps: DVec3 },
    /// Generic autopilot/script alarm.
    Alarm,
}

/// One armed event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScheduledEvent {
    pub id: u64,
    /// Armed epoch in simulation seconds.
    pub time: SimTime,
    pub kind: ScheduledKind,
}

impl ScheduledEvent {
    /// Rails-wake kinds share one slot: a rebake replaces the previous wake
    /// instead of stacking duplicates.
    fn is_rails_wake(&self) -> bool {
        matches!(
            self.kind,
            ScheduledKind::RailsImpact { .. } | ScheduledKind::RailsHorizon
        )
    }
}

/// Deterministic time-ordered queue. Events sort by epoch (total order),
/// ties break by arm order. Small counts: a sorted `Vec` beats a map.
#[derive(Debug, Clone, Default)]
pub struct EventScheduler {
    next_id: u64,
    events: Vec<ScheduledEvent>,
}

impl EventScheduler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    /// Arm an event, keeping time order. Returns its id for `cancel`.
    pub fn arm(&mut self, kind: ScheduledKind, time: SimTime) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let event = ScheduledEvent { id, time, kind };
        let position = self
            .events
            .iter()
            .position(|existing| {
                existing
                    .time
                    .seconds()
                    .total_cmp(&time.seconds())
                    .then(existing.id.cmp(&id))
                    .is_ge()
            })
            .unwrap_or(self.events.len());
        self.events.insert(position, event);
        id
    }

    /// Arm a rails wake, replacing any previous rails wake first: one baked
    /// path owns exactly one wake condition.
    pub fn arm_rails_wake(&mut self, kind: ScheduledKind, time: SimTime) -> u64 {
        debug_assert!(
            matches!(
                kind,
                ScheduledKind::RailsImpact { .. } | ScheduledKind::RailsHorizon
            ),
            "only rails wakes replace the rails slot"
        );
        self.events.retain(|event| !event.is_rails_wake());
        self.arm(kind, time)
    }

    /// Drop all rails wakes (rebake without a wake, or leaving the rails).
    pub fn clear_rails_wakes(&mut self) {
        self.events.retain(|event| !event.is_rails_wake());
    }

    /// Cancel one event by id. True when something was removed.
    pub fn cancel(&mut self, id: u64) -> bool {
        let before = self.events.len();
        self.events.retain(|event| event.id != id);
        self.events.len() != before
    }

    /// Earliest armed event without removing it.
    pub fn next(&self) -> Option<ScheduledEvent> {
        self.events.first().copied()
    }

    /// Remove and return every event at or before `now`, in order.
    pub fn drain_due(&mut self, now: SimTime) -> Vec<ScheduledEvent> {
        let boundary = self
            .events
            .iter()
            .position(|event| event.time.seconds().total_cmp(&now.seconds()).is_gt())
            .unwrap_or(self.events.len());
        self.events.drain(..boundary).collect()
    }
}
