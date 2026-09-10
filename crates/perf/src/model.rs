//! Portable measurement model shared by client overlays, captures, and the
//! future authoritative-server metrics endpoint.
//!
//! Client GPU metrics must never leak into simulation APIs: [`SimBudget`] and
//! [`WorldCounters`] are GPU-free by construction, while [`GpuFrame`] lives
//! only on [`FrameRecord`] alongside them.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::DEFAULT_RING_CAPACITY_FRAMES;

/// Format version written into every capture header.
pub const CAPTURE_FORMAT_VERSION: u32 = 1;

/// Instrumentation density. Expensive scopes stay compile-time/dev-only;
/// runtime toggling covers the rest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProfilingLevel {
    /// Only frame wall time for basic FPS / health reporting.
    Off,
    /// Top-level CPU scopes, sim budget, world counters, ring buffer.
    #[default]
    Normal,
    /// Additional subsystem scopes, queue depths, cache stats, event markers.
    Detailed,
}

/// Fixed-step simulation budget for one render frame (spec section 7).
///
/// Lets the game distinguish render slowdown from a simulation that cannot
/// keep up with requested warp. Time warp must never silently change physics
/// because the frame rate fell; consumers must check `effective_warp` vs
/// `requested_warp` and `backlog_s`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SimBudget {
    /// Fixed solver step in simulation seconds (e.g. 1/120).
    pub fixed_dt_s: f64,
    /// Solver steps executed this render frame.
    pub steps_this_frame: u32,
    /// Wall-clock time spent inside simulation this frame.
    pub sim_cpu_s: f64,
    /// Simulation time advanced this frame (`steps * fixed_dt_s` normally).
    pub sim_time_advanced_s: f64,
    /// Requested time-warp factor.
    pub requested_warp: f64,
    /// Resolved/effective warp actually achieved.
    pub effective_warp: f64,
    /// Unprocessed simulation backlog in seconds.
    pub backlog_s: f64,
}

impl Default for SimBudget {
    fn default() -> Self {
        Self {
            fixed_dt_s: 1.0 / 120.0,
            steps_this_frame: 0,
            sim_cpu_s: 0.0,
            sim_time_advanced_s: 0.0,
            requested_warp: 1.0,
            effective_warp: 1.0,
            backlog_s: 0.0,
        }
    }
}

impl SimBudget {
    /// Build from observed counters. `effective_warp` is clamped to
    /// `requested_warp` so a fast machine never reports warp overshoot.
    pub fn from_steps(
        fixed_dt_s: f64,
        steps_this_frame: u32,
        sim_cpu_s: f64,
        requested_warp: f64,
        backlog_s: f64,
        frame_wall_s: f64,
    ) -> Self {
        let advanced = fixed_dt_s * steps_this_frame as f64;
        let effective = if frame_wall_s > 1e-9 {
            (advanced / frame_wall_s).min(requested_warp.max(0.0))
        } else {
            requested_warp
        };
        Self {
            fixed_dt_s,
            steps_this_frame,
            sim_cpu_s: sim_cpu_s.max(0.0),
            sim_time_advanced_s: advanced,
            requested_warp,
            effective_warp: effective.max(0.0),
            backlog_s: backlog_s.max(0.0),
        }
    }

    /// True when the simulation could not keep up with requested warp.
    pub fn is_warp_limited(&self) -> bool {
        self.requested_warp > 1.0
            && self.effective_warp < self.requested_warp * 0.95
            && self.backlog_s > 0.0
    }
}

/// Workload counters describing actual work, not preset labels (spec 8).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct WorldCounters {
    pub active_bodies: u32,
    pub active_vehicles: u32,
    pub active_aero_panels: u32,
    pub terrain_patches_visible: u32,
    pub terrain_patches_generated: u32,
    pub terrain_vertices: u64,
    pub terrain_triangles: u64,
    pub terrain_cache_hits: u64,
    pub terrain_cache_misses: u64,
    pub landmark_zones: u32,
    pub streaming_queued: u32,
    pub assets_loaded: u32,
    pub assets_pending: u32,
    pub lights_visible: u32,
    pub rt_instances: u32,
}

/// Process / cache memory sample (spec 9).
///
/// On integrated GPUs a driver-reported VRAM budget may be a UMA reservation,
/// not separate memory. `gpu_mem_bytes == None` means unknown — never a
/// synthesized misleading number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MemorySample {
    /// Resident set size in bytes, when inexpensive and portable to read.
    pub rss_bytes: Option<u64>,
    pub asset_cache_bytes: Option<u64>,
    pub terrain_cache_bytes: Option<u64>,
    /// `None` encodes `unknown`.
    pub gpu_mem_bytes: Option<u64>,
}

/// GPU timing for one frame (spec 5).
///
/// CPU submission time is never presented as GPU time: when timestamps are
/// unsupported, `available` is false and consumers must render "unavailable".
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct GpuFrame {
    pub available: bool,
    /// Whole GPU frame in seconds, when the backend reported it.
    pub frame_s: Option<f64>,
    /// Subsystem scopes (`render.terrain`, `render.atmosphere`, ...).
    #[serde(default)]
    pub scopes: BTreeMap<String, f64>,
}

impl GpuFrame {
    pub fn unavailable() -> Self {
        Self {
            available: false,
            frame_s: None,
            scopes: BTreeMap::new(),
        }
    }
}

/// One measured frame: timestamps, timings, counters, memory.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameRecord {
    pub frame_index: u64,
    /// Seconds since collector creation (monotonic, not wall clock).
    pub wall_timestamp_s: f64,
    pub sim_time_s: f64,
    /// Whole client frame (seconds).
    pub frame_wall_s: f64,
    /// CPU portion of the frame (seconds).
    pub frame_cpu_s: f64,
    pub gpu: GpuFrame,
    /// Named CPU scopes in seconds (`sim.aero` -> 0.0004).
    #[serde(default)]
    pub cpu_scopes: BTreeMap<String, f64>,
    pub sim: SimBudget,
    pub world: WorldCounters,
    pub memory: MemorySample,
}

/// Lightweight point marker for expensive transitions (spec 14).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EventMarker {
    pub timestamp_s: f64,
    pub frame_index: u64,
    pub name: String,
    pub detail: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BuildInfo {
    pub commit: String,
    pub version: String,
    pub profile: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlatformInfo {
    pub os: String,
    pub cpu: String,
    /// `None` / `"unknown"` when the backend cannot report reliably.
    pub gpu: Option<String>,
    pub backend: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DisplayInfo {
    pub width: u32,
    pub height: u32,
    pub resolution_scale: f32,
    pub vsync: bool,
}

/// Requested preset plus resolved settings actually used by the renderer.
///
/// The perf system never interprets quality itself; it carries
/// `ResolvedGraphicsSettings`-equivalent metadata so captures from two
/// commits stay comparable (spec 12-13).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GraphicsInfo {
    pub preset: String,
    pub ray_tracing: String,
    #[serde(default)]
    pub resolved: BTreeMap<String, String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CaptureMetadata {
    pub format_version: u32,
    pub build: BuildInfo,
    pub platform: PlatformInfo,
    pub display: DisplayInfo,
    pub graphics: GraphicsInfo,
    pub scenario: String,
    /// Frames retained in the rolling window at capture time.
    pub window_frames: usize,
}

/// Best-effort local metadata. Callers should override GPU/backend/display
/// from the real renderer; unknown values stay `unknown` instead of guessing.
pub fn default_capture_metadata() -> CaptureMetadata {
    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    CaptureMetadata {
        format_version: CAPTURE_FORMAT_VERSION,
        build: BuildInfo {
            commit: option_env!("THESSA_BUILD_COMMIT")
                .unwrap_or("unknown")
                .to_string(),
            version: env!("CARGO_PKG_VERSION").to_string(),
            profile: profile.to_string(),
        },
        platform: PlatformInfo {
            os: std::env::consts::OS.to_string(),
            cpu: std::env::consts::ARCH.to_string(),
            gpu: None,
            backend: "unknown".to_string(),
        },
        display: DisplayInfo {
            width: 0,
            height: 0,
            resolution_scale: 1.0,
            vsync: false,
        },
        graphics: GraphicsInfo {
            preset: "unknown".to_string(),
            ray_tracing: "off".to_string(),
            resolved: BTreeMap::new(),
        },
        scenario: "unknown".to_string(),
        window_frames: DEFAULT_RING_CAPACITY_FRAMES,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn warp_limited_detection() {
        let ok = SimBudget::from_steps(1.0 / 120.0, 12, 0.001, 1.0, 0.0, 1.0 / 60.0);
        assert!(!ok.is_warp_limited());
        let limited = SimBudget {
            requested_warp: 100.0,
            effective_warp: 80.0,
            backlog_s: 0.004,
            ..SimBudget::default()
        };
        assert!(limited.is_warp_limited());
    }

    #[test]
    fn gpu_unavailable_is_explicit() {
        let gpu = GpuFrame::unavailable();
        assert!(!gpu.available);
        assert!(gpu.frame_s.is_none());
    }
}
