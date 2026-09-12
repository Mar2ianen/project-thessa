//! Background rails-bake orchestration without a runtime dependency.
//!
//! The flight loop must never block on a trajectory bake: it requests one
//! when baked coverage runs thin and polls for completion on later ticks.
//! [`BakeQueue`] abstracts the worker behind a trait object so the same
//! advance loop runs on the Bevy pool (client), a thread join handle
//! (server) and inline synchronous bakes (tests) with zero generic ripple.

use thessa_sim_core::{BodyId, OnRailsCache, SimTime, TestParticleState, TickIntegratorConfig};

/// Everything a bake worker needs: owned and `Send` by construction.
#[derive(Debug, Clone)]
pub struct RailsBakeRequest {
    pub initial: TestParticleState,
    pub time: SimTime,
    pub config: TickIntegratorConfig,
    pub impact_bodies: Vec<BodyId>,
}

/// A finished bake ready for the state/key check before adoption.
#[derive(Debug)]
pub struct BakedRails {
    pub rails: OnRailsCache,
    pub bake_seconds: f64,
}

/// Worker behind the flight loop. Implementations must be non-blocking on
/// both calls: `request_bake` queues (or drops when busy), `poll_bake`
/// harvests at most one finished bake per call.
pub trait BakeQueue {
    /// Queue a bake unless one is already in flight. Dropping under load
    /// is correct: coverage only grows stale, the loop re-requests.
    fn request_bake(&mut self, request: RailsBakeRequest);
    /// Take a finished bake, if any. Errors are bake failures (bad state,
    /// horizon exhausted), never transport faults.
    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>>;
    /// True while a bake is in flight (guides re-request pacing).
    fn has_pending(&self) -> bool;
}

/// Synchronous bake: runs the worker inline on request, hands the result
/// out on the next poll. Deterministic single-threaded behavior for tests
/// and the headless no-worker fallback the client already had.
#[derive(Debug, Default)]
pub struct InlineBakeQueue {
    pending: Option<Result<BakedRails, String>>,
    busy: bool,
}

impl InlineBakeQueue {
    pub fn new() -> Self {
        Self::default()
    }
}

impl BakeQueue for InlineBakeQueue {
    fn request_bake(&mut self, _request: RailsBakeRequest) {
        // Inline execution needs the ephemeris, which the queue does not
        // own; the runtime performs the bake itself and calls
        // `deliver` below. The flag only paces re-requests.
        self.busy = true;
    }

    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>> {
        self.busy = false;
        self.pending.take()
    }

    fn has_pending(&self) -> bool {
        self.busy || self.pending.is_some()
    }
}

impl InlineBakeQueue {
    /// Hand a synchronously computed bake to the queue (called by the
    /// runtime right after `request_bake` in inline mode).
    pub fn deliver(&mut self, baked: Result<BakedRails, String>) {
        self.pending = Some(baked);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inline_queue_holds_one_bake_until_polled() {
        let mut queue = InlineBakeQueue::new();
        assert!(!queue.has_pending());
        assert!(queue.poll_bake().is_none());
        queue.request_bake(RailsBakeRequest {
            initial: TestParticleState {
                position: glam::DVec3::ZERO,
                velocity: glam::DVec3::ZERO,
            },
            time: SimTime(0.0),
            config: TickIntegratorConfig::default(),
            impact_bodies: Vec::new(),
        });
        assert!(queue.has_pending());
        queue.deliver(Err("horizon exhausted".to_string()));
        assert!(queue.has_pending());
        let result = queue.poll_bake().expect("baked");
        assert!(result.is_err());
        assert!(!queue.has_pending());
    }
}
