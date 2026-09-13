//! Background rails-bake orchestration without a runtime dependency.
//!
//! The flight loop must never block on a trajectory bake: it requests one
//! when baked coverage runs thin and polls for completion on later ticks.
//! [`BakeQueue`] abstracts the worker behind a trait object so the same
//! advance loop runs on the Bevy pool (client), a thread join handle
//! (server) and inline synchronous bakes (tests) with zero generic ripple.

use thessa_sim_core::{
    BakedEphemeris, BodyId, OnRailsCache, SimTime, TestParticleState, TickIntegratorConfig,
};

/// Everything a bake worker needs: owned and `Send` by construction.
/// The ephemeris travels with the request so thread workers never borrow
/// from the flight loop (same clone the Bevy task already performed).
#[derive(Debug, Clone)]
pub struct RailsBakeRequest {
    pub initial: TestParticleState,
    pub time: SimTime,
    pub config: TickIntegratorConfig,
    pub impact_bodies: Vec<BodyId>,
    pub ephemeris: BakedEphemeris,
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
pub trait BakeQueue: Send + Sync {
    /// Queue a bake unless one is already in flight. Dropping under load
    /// is correct: coverage only grows stale, the loop re-requests.
    fn request_bake(&mut self, request: RailsBakeRequest);
    /// Take a finished bake, if any. Errors are bake failures (bad state,
    /// horizon exhausted), never transport faults.
    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>>;
    /// True while a bake is in flight (guides re-request pacing).
    fn has_pending(&self) -> bool;
    /// Drop in-flight work (launch reset, universe edit).
    fn reset(&mut self);
}

pub(crate) fn run_bake(request: &RailsBakeRequest) -> Result<BakedRails, String> {
    let started = std::time::Instant::now();
    let mut rails = OnRailsCache::new();
    rails
        .bake_tick(
            &request.ephemeris,
            request.initial,
            request.time,
            request.config,
            &request.impact_bodies,
        )
        .map_err(|e| e.to_string())?;
    Ok(BakedRails {
        rails,
        bake_seconds: started.elapsed().as_secs_f64(),
    })
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
    fn request_bake(&mut self, request: RailsBakeRequest) {
        if self.busy {
            return;
        }
        self.busy = true;
        self.pending = Some(run_bake(&request));
    }

    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>> {
        self.busy = false;
        self.pending.take()
    }

    fn has_pending(&self) -> bool {
        self.busy || self.pending.is_some()
    }

    fn reset(&mut self) {
        self.busy = false;
        self.pending = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::SystemConfig;

    fn request() -> RailsBakeRequest {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        RailsBakeRequest {
            initial: TestParticleState {
                position: glam::DVec3::new(7.0e6, 0.0, 0.0),
                velocity: glam::DVec3::new(0.0, 7500.0, 0.0),
            },
            time: SimTime(0.0),
            config: TickIntegratorConfig::default(),
            impact_bodies: Vec::new(),
            ephemeris,
        }
    }

    #[test]
    fn inline_queue_bakes_synchronously_and_holds_until_polled() {
        let mut queue = InlineBakeQueue::new();
        assert!(!queue.has_pending());
        assert!(queue.poll_bake().is_none());
        queue.request_bake(request());
        assert!(queue.has_pending());
        // A second request while busy is dropped, never queued.
        queue.request_bake(request());
        let result = queue.poll_bake().expect("baked");
        // Empty impact set over a live ephemeris still produces a path.
        assert!(result.is_ok(), "bake failed: {result:?}");
        assert!(!queue.has_pending());
        queue.request_bake(request());
        queue.reset();
        assert!(!queue.has_pending());
        assert!(queue.poll_bake().is_none());
    }
}
