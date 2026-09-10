//! Short human-inspectable captures sharing one in-memory model.
//!
//! Recommended outputs (spec 11):
//!
//! ```text
//! perf-YYYYMMDD-HHMMSS.json
//! perf-YYYYMMDD-HHMMSS.csv
//! ```
//!
//! JSON carries structured metadata and nested scopes. CSV carries one row per
//! frame for quick plotting and commit-to-commit comparison. Capture writing
//! must not block the render thread: build the [`PerfCapture`] from the ring,
//! then serialize after the frame or on a background task.

use std::fmt::Write as _;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::model::{CaptureMetadata, EventMarker, FrameRecord};

/// One short profiling capture: metadata header + frames + event markers.
/// This is a performance trace, not a save game: no internal object state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PerfCapture {
    pub metadata: CaptureMetadata,
    pub frames: Vec<FrameRecord>,
    #[serde(default)]
    pub events: Vec<EventMarker>,
}

impl PerfCapture {
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// One row per frame. Scope columns are the union of scope names in this
    /// capture, prefixed with `scope:` and measured in milliseconds.
    pub fn to_csv(&self) -> String {
        let mut scope_names: Vec<String> = Vec::new();
        for frame in &self.frames {
            for name in frame.cpu_scopes.keys() {
                if !scope_names.iter().any(|n| n == name) {
                    scope_names.push(name.clone());
                }
            }
        }
        scope_names.sort();

        let mut out = String::new();
        out.push_str(
            "frame,wall_s,sim_time_s,frame_ms,cpu_ms,gpu_ms,gpu_available,steps,fixed_dt_s,sim_cpu_ms,sim_advanced_s,requested_warp,effective_warp,backlog_ms,bodies,vehicles,patches_visible,patches_generated,triangles,cache_hits,cache_misses,streaming_queued,assets_pending,rss_bytes",
        );
        for name in &scope_names {
            let _ = write!(out, ",scope:{name}_ms");
        }
        out.push('\n');

        for frame in &self.frames {
            let gpu_ms = frame
                .gpu
                .frame_s
                .map(|t| format!("{:.4}", t * 1000.0))
                .unwrap_or_default();
            let rss = frame
                .memory
                .rss_bytes
                .map(|b| b.to_string())
                .unwrap_or_else(|| "unknown".to_string());
            let _ = write!(
                out,
                "{},{:.4},{:.4},{:.4},{:.4},{},{},{},{:.6},{:.4},{:.4},{:.2},{:.2},{:.4},{},{},{},{},{},{},{},{},{},{}",
                frame.frame_index,
                frame.wall_timestamp_s,
                frame.sim_time_s,
                frame.frame_wall_s * 1000.0,
                frame.frame_cpu_s * 1000.0,
                gpu_ms,
                frame.gpu.available,
                frame.sim.steps_this_frame,
                frame.sim.fixed_dt_s,
                frame.sim.sim_cpu_s * 1000.0,
                frame.sim.sim_time_advanced_s,
                frame.sim.requested_warp,
                frame.sim.effective_warp,
                frame.sim.backlog_s * 1000.0,
                frame.world.active_bodies,
                frame.world.active_vehicles,
                frame.world.terrain_patches_visible,
                frame.world.terrain_patches_generated,
                frame.world.terrain_triangles,
                frame.world.terrain_cache_hits,
                frame.world.terrain_cache_misses,
                frame.world.streaming_queued,
                frame.world.assets_pending,
                rss,
            );
            for name in &scope_names {
                let ms = frame.cpu_scopes.get(name).copied().unwrap_or(0.0) * 1000.0;
                let _ = write!(out, ",{ms:.4}");
            }
            out.push('\n');
        }
        out
    }

    /// Serialize after the frame; prefer calling from a background task for
    /// large captures.
    pub fn write_json(&self, path: &Path) -> std::io::Result<()> {
        let json = self
            .to_json_pretty()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, json)
    }

    pub fn write_csv(&self, path: &Path) -> std::io::Result<()> {
        std::fs::write(path, self.to_csv())
    }
}

/// `perf-YYYYMMDD-HHMMSS` stem from system time (UTC-ish, no extra deps).
/// Falls back to a frame counter suffix when the clock is unavailable.
pub fn capture_stem() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // Convert to calendar fields without chrono: days since epoch.
    let (y, mo, d, h, mi, s) = unix_to_ymd_hms(secs);
    format!("perf-{y:04}{mo:02}{d:02}-{h:02}{mi:02}{s:02}")
}

fn unix_to_ymd_hms(secs: u64) -> (i32, u32, u32, u32, u32, u32) {
    let days = (secs / 86_400) as i64;
    let tod = (secs % 86_400) as u32;
    // Howard Hinnant's civil-from-days algorithm.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let mut y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    y += i64::from(m <= 2);
    (y as i32, m, d, tod / 3600, (tod % 3600) / 60, tod % 60)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::*;
    use std::collections::BTreeMap;

    fn sample_capture() -> PerfCapture {
        PerfCapture {
            metadata: default_capture_metadata(),
            frames: vec![FrameRecord {
                frame_index: 0,
                wall_timestamp_s: 0.016,
                sim_time_s: 1.0,
                frame_wall_s: 0.0086,
                frame_cpu_s: 0.0031,
                gpu: GpuFrame::unavailable(),
                cpu_scopes: BTreeMap::from([("sim.aero".to_string(), 0.0004)]),
                sim: SimBudget::default(),
                world: WorldCounters::default(),
                memory: MemorySample {
                    rss_bytes: Some(1234),
                    ..Default::default()
                },
            }],
            events: vec![EventMarker {
                timestamp_s: 0.016,
                frame_index: 0,
                name: "warp changed".to_string(),
                detail: Some("x1 -> x10".to_string()),
            }],
        }
    }

    #[test]
    fn json_roundtrip_preserves_frames_and_events() {
        let capture = sample_capture();
        let json = capture.to_json_pretty().unwrap();
        let parsed = PerfCapture::from_json(&json).unwrap();
        assert_eq!(parsed, capture);
        assert!(json.contains("format_version"));
    }

    #[test]
    fn csv_has_header_and_scope_columns() {
        let csv = sample_capture().to_csv();
        let mut lines = csv.lines();
        let header = lines.next().unwrap();
        assert!(header.contains("frame_ms"));
        assert!(header.contains("scope:sim.aero_ms"));
        let row = lines.next().unwrap();
        assert!(row.contains("8.6000") || row.contains("8.6"));
    }

    #[test]
    fn capture_stem_format() {
        let stem = capture_stem();
        assert!(stem.starts_with("perf-"));
        assert!(stem.contains('-'));
    }
}
