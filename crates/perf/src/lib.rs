//! Named performance measurements for Project Thessa.
//!
//! Implements the first vertical slice of `docs/15_PERFORMANCE_MONITORING.md`:
//!
//! - stable hierarchical scope names (`frame.cpu`, `sim.aero`, `render.atmosphere`, ...);
//! - monotonic CPU timing with RAII scopes;
//! - fixed-capacity rolling ring buffer (default ~30 s at 60 fps);
//! - p50 / p95 / p99 / max statistics for long-tail hitch detection;
//! - simulation fixed-step / time-warp budget counters;
//! - world / terrain workload counters;
//! - memory samples (`unknown` when the backend cannot report reliably);
//! - explicit GPU-unavailable reporting (never present CPU submit as GPU time);
//! - short JSON / CSV captures with build + graphics metadata;
//! - lightweight event markers.
//!
//! The crate intentionally depends only on `std` + `serde`. It must stay usable
//! from the authoritative simulation, the Bevy client, and the Tokio server
//! without pulling Bevy, Tokio, or GPU APIs into the measurement model
//! (see `AGENTS.md` module boundaries).
//!
//! # Core rule from the spec
//!
//! > named measurements are first-class data; overlays, logs, captures and
//! > external profilers are consumers of the same measurements.

#![forbid(unsafe_code)]

mod capture;
mod memory;
mod model;
mod ring;
mod stats;

pub use capture::{PerfCapture, capture_stem};
pub use memory::current_rss_bytes;
pub use model::{
    BuildInfo, CaptureMetadata, DisplayInfo, EventMarker, FrameRecord, GpuFrame, GraphicsInfo,
    MemorySample, PlatformInfo, ProfilingLevel, SimBudget, WorldCounters, default_capture_metadata,
};
pub use ring::RingBuffer;
pub use stats::{FrameStats, percentile_sorted, summarize};

use std::collections::{BTreeMap, VecDeque};
use std::time::Instant;

/// Default ring capacity: ~30 s at 60 fps.
pub const DEFAULT_RING_CAPACITY_FRAMES: usize = 1800;
/// Lower bound suggested by the spec (10 s at 60 fps).
pub const MIN_RING_CAPACITY_FRAMES: usize = 600;
/// Cap for retained event markers per collector.
pub const MAX_EVENT_MARKERS: usize = 256;

/// Canonical top-level scope names from the spec.
/// Exact names may evolve, but captures from two commits must stay comparable,
/// so prefer adding new names over renaming existing ones.
pub mod scopes {
    pub const FRAME_CPU: &str = "frame.cpu";
    pub const FRAME_GPU: &str = "frame.gpu";
    pub const SIM_TOTAL: &str = "sim.total";
    pub const SIM_GRAVITY: &str = "sim.gravity";
    pub const SIM_AERO: &str = "sim.aero";
    pub const SIM_RIGID_BODY: &str = "sim.rigid_body";
    pub const SIM_CONTACTS: &str = "sim.contacts";
    pub const SIM_GUIDANCE: &str = "sim.guidance";
    pub const SIM_FACTORY: &str = "sim.factory";
    pub const WORLD_TOTAL: &str = "world.total";
    pub const WORLD_TERRAIN_SAMPLING: &str = "world.terrain_sampling";
    pub const WORLD_TERRAIN_MESHING: &str = "world.terrain_meshing";
    pub const WORLD_LANDMARKS: &str = "world.landmarks";
    pub const WORLD_STREAMING: &str = "world.streaming";
    pub const RENDER_TOTAL: &str = "render.total";
    pub const RENDER_TERRAIN: &str = "render.terrain";
    pub const RENDER_ATMOSPHERE: &str = "render.atmosphere";
    pub const RENDER_CLOUDS: &str = "render.clouds";
    pub const RENDER_WATER: &str = "render.water";
    pub const RENDER_SOLARI: &str = "render.solari";
    pub const RENDER_SHADOWS: &str = "render.shadows";
    pub const RENDER_POSTPROCESS: &str = "render.postprocess";
    pub const RENDER_UI: &str = "render.ui";
    pub const IO_ASSETS: &str = "io.assets";
    pub const IO_PERSISTENCE: &str = "io.persistence";
}

/// RAII CPU timing guard.
///
/// Created via [`PerfCollector::scope`]; records the elapsed wall time into the
/// in-progress frame on drop. Uses a monotonic high-resolution clock
/// (`Instant`), never simulation time.
pub struct CpuScopeGuard<'a> {
    collector: Option<&'a mut PerfCollector>,
    name: &'static str,
    start: Instant,
    detailed: bool,
}

impl Drop for CpuScopeGuard<'_> {
    fn drop(&mut self) {
        if let Some(collector) = self.collector.take() {
            // Detailed scopes are dropped silently when the level does not
            // include them, keeping disabled instrumentation cheap.
            if !self.detailed || collector.level == ProfilingLevel::Detailed {
                let elapsed_s = self.start.elapsed().as_secs_f64();
                collector.record_cpu_scope(self.name, elapsed_s);
            }
        }
    }
}

/// In-progress frame builder owned by [`PerfCollector`].
#[derive(Debug, Default)]
struct PendingFrame {
    cpu_scopes: BTreeMap<String, f64>,
    sim: SimBudget,
    world: WorldCounters,
    memory: MemorySample,
    gpu: GpuFrame,
    sim_time_s: f64,
}

/// Rolling performance collector.
///
/// Lifecycle per render frame:
///
/// ```text
/// collector.begin_frame();
/// {
///     let _g = collector.scope("sim.total");
///     ... simulation work ...
/// }
/// collector.set_sim_budget(...);
/// collector.set_world_counters(...);
/// collector.end_frame(frame_wall_s, sim_time_s);
/// ```
///
/// `begin_frame` clears the pending map; `end_frame` pushes one [`FrameRecord`]
/// into the fixed-capacity ring. When `level == Off`, only the frame wall time
/// is retained so basic FPS / health reporting keeps working.
#[derive(Debug)]
pub struct PerfCollector {
    level: ProfilingLevel,
    ring: VecDeque<FrameRecord>,
    capacity: usize,
    pending: PendingFrame,
    frame_start: Option<Instant>,
    next_frame_index: u64,
    events: Vec<EventMarker>,
    boot: Instant,
}

impl PerfCollector {
    /// Create a collector with an explicit ring capacity (frames).
    pub fn new(capacity: usize) -> Self {
        Self {
            level: ProfilingLevel::Normal,
            ring: VecDeque::with_capacity(capacity.min(1 << 20)),
            capacity: capacity.max(1),
            pending: PendingFrame::default(),
            frame_start: None,
            next_frame_index: 0,
            events: Vec::new(),
            boot: Instant::now(),
        }
    }

    /// Default collector: [`DEFAULT_RING_CAPACITY_FRAMES`] frames.
    pub fn with_default_capacity() -> Self {
        Self::new(DEFAULT_RING_CAPACITY_FRAMES)
    }

    pub fn level(&self) -> ProfilingLevel {
        self.level
    }

    pub fn set_level(&mut self, level: ProfilingLevel) {
        self.level = level;
    }

    pub fn len(&self) -> usize {
        self.ring.len()
    }

    pub fn is_empty(&self) -> bool {
        self.ring.is_empty()
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Mark the start of a client frame. Cheap; only resets pending state.
    pub fn begin_frame(&mut self) {
        self.pending = PendingFrame::default();
        self.frame_start = Some(Instant::now());
    }

    /// Open a named CPU scope for the in-progress frame.
    ///
    /// Top-level scopes should have negligible overhead. Prefer a small number
    /// of useful scopes over thousands of noisy ones in release builds.
    pub fn scope(&mut self, name: &'static str) -> CpuScopeGuard<'_> {
        CpuScopeGuard {
            collector: Some(self),
            name,
            start: Instant::now(),
            detailed: false,
        }
    }

    /// Open a detailed/dev-only scope. Silently discarded unless the level is
    /// [`ProfilingLevel::Detailed`], so disabled instrumentation avoids map
    /// inserts and string allocation (names are `&'static str`).
    pub fn detailed_scope(&mut self, name: &'static str) -> CpuScopeGuard<'_> {
        CpuScopeGuard {
            collector: Some(self),
            name,
            start: Instant::now(),
            detailed: true,
        }
    }

    /// Record a CPU scope duration directly (seconds). Used when the timed
    /// region cannot hold a guard across an await point or system boundary.
    ///
    /// Parallel worker durations must be reported as their own scopes; do not
    /// sum parallel task durations and label the result as frame latency.
    pub fn record_cpu_scope(&mut self, name: &str, duration_s: f64) {
        if self.level == ProfilingLevel::Off {
            return;
        }
        let duration_s = duration_s.max(0.0);
        *self.pending.cpu_scopes.entry(name.to_string()).or_default() += duration_s;
    }

    pub fn set_sim_budget(&mut self, budget: SimBudget) {
        self.pending.sim = budget;
    }

    pub fn set_world_counters(&mut self, counters: WorldCounters) {
        self.pending.world = counters;
    }

    pub fn set_memory_sample(&mut self, sample: MemorySample) {
        self.pending.memory = sample;
    }

    pub fn set_gpu_frame(&mut self, gpu: GpuFrame) {
        self.pending.gpu = gpu;
    }

    pub fn set_sim_time_s(&mut self, sim_time_s: f64) {
        self.pending.sim_time_s = sim_time_s;
    }

    /// Record a lightweight point marker (LOD rebuild, warp change, ...).
    /// Markers are diagnostic only, never authoritative gameplay events.
    pub fn push_event(&mut self, name: &str, detail: Option<String>) {
        if self.level == ProfilingLevel::Off {
            return;
        }
        if self.events.len() >= MAX_EVENT_MARKERS {
            self.events.remove(0);
        }
        self.events.push(EventMarker {
            timestamp_s: self.boot.elapsed().as_secs_f64(),
            frame_index: self.next_frame_index,
            name: name.to_string(),
            detail,
        });
    }

    /// Close the frame and push it into the ring.
    ///
    /// `frame_wall_s` is the whole client frame time in seconds (monotonic
    /// clock). `frame_cpu_s` is the measured CPU portion where known;
    /// pass `frame_wall_s` when no finer split is available.
    pub fn end_frame(&mut self, frame_wall_s: f64, frame_cpu_s: f64, sim_time_s: f64) {
        let frame_wall_s = frame_wall_s.max(0.0);
        let frame_cpu_s = frame_cpu_s.max(0.0).min(frame_wall_s.max(frame_cpu_s));
        self.pending.sim_time_s = sim_time_s;
        // Fill memory lazily when the caller did not provide a sample.
        if self.pending.memory.rss_bytes.is_none() {
            self.pending.memory.rss_bytes = current_rss_bytes();
        }
        let pending = std::mem::take(&mut self.pending);
        let record = FrameRecord {
            frame_index: self.next_frame_index,
            wall_timestamp_s: self.boot.elapsed().as_secs_f64(),
            sim_time_s,
            frame_wall_s,
            frame_cpu_s,
            gpu: pending.gpu,
            cpu_scopes: if self.level == ProfilingLevel::Off {
                BTreeMap::new()
            } else {
                pending.cpu_scopes
            },
            sim: pending.sim,
            world: pending.world,
            memory: pending.memory,
        };
        self.next_frame_index += 1;
        if self.ring.len() >= self.capacity {
            self.ring.pop_front();
        }
        self.ring.push_back(record);
        self.frame_start = None;
    }

    /// Convenience: close the frame using the `begin_frame` timestamp.
    /// Returns the measured wall time.
    pub fn end_frame_auto(&mut self, sim_time_s: f64) -> f64 {
        let wall = self
            .frame_start
            .map(|t| t.elapsed().as_secs_f64())
            .unwrap_or(0.0);
        let cpu_total: f64 = self.pending.cpu_scopes.values().sum();
        let cpu = if cpu_total > 0.0 {
            cpu_total.min(wall)
        } else {
            wall
        };
        self.end_frame(wall, cpu, sim_time_s);
        wall
    }

    /// Iterate frames from oldest to newest.
    pub fn frames(&self) -> impl Iterator<Item = &FrameRecord> {
        self.ring.iter()
    }

    /// Latest frame, if any.
    pub fn latest(&self) -> Option<&FrameRecord> {
        self.ring.back()
    }

    /// Retained event markers, oldest first.
    pub fn events(&self) -> &[EventMarker] {
        &self.events
    }

    /// Statistics over whole-frame wall time in the current window.
    pub fn frame_wall_stats(&self) -> Option<FrameStats> {
        let values: Vec<f64> = self.ring.iter().map(|f| f.frame_wall_s).collect();
        summarize(&values)
    }

    /// Statistics over CPU frame time in the current window.
    pub fn frame_cpu_stats(&self) -> Option<FrameStats> {
        let values: Vec<f64> = self.ring.iter().map(|f| f.frame_cpu_s).collect();
        summarize(&values)
    }

    /// Statistics for one named CPU scope across the window.
    pub fn scope_stats(&self, scope: &str) -> Option<FrameStats> {
        let values: Vec<f64> = self
            .ring
            .iter()
            .map(|f| f.cpu_scopes.get(scope).copied().unwrap_or(0.0))
            .collect();
        summarize(&values)
    }

    /// Build a portable capture sharing one in-memory model for JSON and CSV.
    pub fn snapshot_capture(&self, metadata: CaptureMetadata) -> PerfCapture {
        PerfCapture {
            metadata,
            frames: self.ring.iter().cloned().collect(),
            events: self.events.clone(),
        }
    }

    /// Answer the spec acceptance questions from the current window.
    pub fn summary_text(&self) -> String {
        let Some(wall) = self.frame_wall_stats() else {
            return "PERF: no frames recorded yet".to_string();
        };
        let cpu = self.frame_cpu_stats();
        let latest = self.latest();
        let gpu_line = match latest {
            Some(f) if f.gpu.available => match f.gpu.frame_s {
                Some(t) => format!("GPU {:5.2} ms (measured)", t * 1000.0),
                None => "GPU n/a (query pending)".to_string(),
            },
            _ => "GPU unavailable on this backend".to_string(),
        };
        let sim_line = match latest {
            Some(f) => format!(
                "SIM steps {:>2} cpu {:5.2}ms rails {:.3}s warp x{:.1}/x{:.1} backlog {:5.2}ms",
                f.sim.steps_this_frame,
                f.sim.sim_cpu_s * 1000.0,
                f.sim.rails_time_advanced_s,
                f.sim.requested_warp,
                f.sim.effective_warp,
                f.sim.backlog_s * 1000.0
            ),
            None => "SIM n/a".to_string(),
        };
        format!(
            "FRAME {:5.2}ms {:4.0}fps cpu {:5.2}ms | {} | p50 {:5.2} p95 {:5.2} p99 {:5.2} max {:5.2} | {}",
            wall.current * 1000.0,
            fps(wall.current),
            cpu.map(|s| s.current * 1000.0).unwrap_or(0.0),
            gpu_line,
            wall.p50 * 1000.0,
            wall.p95 * 1000.0,
            wall.p99 * 1000.0,
            wall.max * 1000.0,
            sim_line,
        )
    }
}

fn fps(frame_s: f64) -> f64 {
    if frame_s > 1e-9 { 1.0 / frame_s } else { 0.0 }
}

/// Open a named CPU scope on a collector.
///
/// ```rust,ignore
/// perf_scope!(collector, "sim.aero");
/// perf_scope!(collector, thessa_perf::scopes::SIM_AERO);
/// ```
#[macro_export]
macro_rules! perf_scope {
    ($collector:expr, $name:expr) => {
        let _perf_scope_guard = $collector.scope($name);
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_overwrites_oldest_frames() {
        let mut collector = PerfCollector::new(4);
        for i in 0..6 {
            collector.begin_frame();
            collector.end_frame(0.008 + i as f64 * 0.001, 0.003, i as f64);
        }
        assert_eq!(collector.len(), 4);
        assert_eq!(collector.frames().next().unwrap().frame_index, 2);
        assert_eq!(collector.latest().unwrap().frame_index, 5);
    }

    #[test]
    fn scope_guard_records_duration() {
        let mut collector = PerfCollector::with_default_capacity();
        collector.begin_frame();
        {
            let _guard = collector.scope("sim.aero");
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        collector.end_frame(0.016, 0.004, 0.0);
        let frame = collector.latest().unwrap();
        assert!(frame.cpu_scopes["sim.aero"] >= 0.001);
    }

    #[test]
    fn off_level_keeps_frame_time_but_drops_scopes() {
        let mut collector = PerfCollector::new(8);
        collector.set_level(ProfilingLevel::Off);
        collector.begin_frame();
        collector.record_cpu_scope("sim.aero", 0.002);
        collector.push_event("warp changed", None);
        collector.end_frame(0.016, 0.016, 0.0);
        let frame = collector.latest().unwrap();
        assert!(frame.cpu_scopes.is_empty());
        assert!(collector.events().is_empty());
        assert!((frame.frame_wall_s - 0.016).abs() < 1e-12);
    }

    #[test]
    fn warp_budget_effective_warp_math() {
        // effective = advanced / wall when wall > 0, else requested.
        let budget = SimBudget {
            fixed_dt_s: 1.0 / 120.0,
            steps_this_frame: 12,
            sim_cpu_s: 0.0017,
            sim_time_advanced_s: 0.1,
            rails_time_advanced_s: 0.0,
            requested_warp: 100.0,
            effective_warp: 96.0,
            backlog_s: 0.0041,
        };
        assert!((budget.sim_time_advanced_s - 12.0 / 120.0).abs() < 1e-9);
        assert!(budget.effective_warp <= budget.requested_warp);
    }
}
