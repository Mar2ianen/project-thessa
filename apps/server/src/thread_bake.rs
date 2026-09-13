//! Thread-worker rails-bake queue for the headless server.
//!
//! One bake at most: a request while busy is dropped (coverage only grows
//! stale; the flight loop re-requests). Finished bakes are harvested with a
//! non-blocking `is_finished` + `join` pair, so the tick loop never blocks.

use std::thread::JoinHandle;
use thessa_flight_authority::{BakeQueue, BakedRails, RailsBakeRequest};
use thessa_sim_core::OnRailsCache;

pub struct ThreadBakeQueue {
    job: Option<JoinHandle<Result<BakedRails, String>>>,
}

impl ThreadBakeQueue {
    pub fn new() -> Self {
        Self { job: None }
    }

    fn take_finished(&mut self) -> Option<Result<BakedRails, String>> {
        let job = self.job.take()?;
        if job.is_finished() {
            Some(job.join().unwrap_or_else(|_| Err("bake thread panicked".into())))
        } else {
            self.job = Some(job);
            None
        }
    }
}

impl Default for ThreadBakeQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl BakeQueue for ThreadBakeQueue {
    fn request_bake(&mut self, request: RailsBakeRequest) {
        if self.job.is_some() {
            return;
        }
        self.job = Some(std::thread::spawn(move || {
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
        }));
    }

    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>> {
        self.take_finished()
    }

    fn has_pending(&self) -> bool {
        self.job.is_some()
    }

    fn reset(&mut self) {
        // Dropping the handle detaches a running bake; its result is
        // discarded and the state/key check never sees it.
        self.job = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{SimTime, SystemConfig, TestParticleState, TickIntegratorConfig};

    #[test]
    fn thread_bake_finishes_and_harvests_without_blocking() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let body = ephemeris.body_id("thessa").expect("thessa");
        let origin = ephemeris
            .body_state(body, SimTime::EPOCH)
            .expect("body state");
        let mut queue = ThreadBakeQueue::new();
        assert!(!queue.has_pending());
        queue.request_bake(RailsBakeRequest {
            initial: TestParticleState {
                position: origin.position_inertial + glam::DVec3::Z * 500_000.0,
                velocity: origin.velocity_inertial + glam::DVec3::X * 3000.0,
            },
            time: SimTime::EPOCH,
            config: TickIntegratorConfig::default(),
            impact_bodies: vec![body],
            ephemeris,
        });
        assert!(queue.has_pending());
        // Second request while busy is dropped.
        let before = queue.has_pending();
        assert!(before);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
        let baked = loop {
            if let Some(result) = queue.poll_bake() {
                break result;
            }
            assert!(std::time::Instant::now() < deadline, "bake thread stuck");
            std::thread::sleep(std::time::Duration::from_millis(5));
        };
        let baked = baked.expect("bake must succeed");
        assert!(baked.bake_seconds >= 0.0);
        assert!(!queue.has_pending());
        queue.reset();
    }
}
