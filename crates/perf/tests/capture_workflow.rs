//! End-to-end workflow from the spec acceptance target:
//! fly, press one key, and answer CPU/GPU-bound, subsystem, hitch,
//! terrain spike, warp keep-up, memory, and build/settings provenance.

use thessa_perf::{
    GpuFrame, MemorySample, PerfCollector, SimBudget, WorldCounters, default_capture_metadata,
};

#[test]
fn capture_workflow_answers_acceptance_questions() {
    let mut collector = PerfCollector::new(300);
    // Simulate 120 frames at ~60 fps with one 70 ms terrain hitch.
    for i in 0..120 {
        collector.begin_frame();
        let hitch = i == 60;
        let wall = if hitch {
            0.070
        } else {
            0.008 + (i % 5) as f64 * 0.0002
        };
        collector.record_cpu_scope("sim.total", 0.0017);
        collector.record_cpu_scope("render.terrain", if hitch { 0.060 } else { 0.002 });
        collector.set_sim_budget(SimBudget::from_steps(
            1.0 / 120.0,
            2,
            0.0017,
            1.0,
            0.0,
            wall,
        ));
        collector.set_world_counters(WorldCounters {
            active_bodies: 22,
            active_vehicles: 1,
            terrain_patches_visible: if hitch { 312 } else { 40 },
            terrain_patches_generated: if hitch { 312 } else { 2 },
            terrain_triangles: 1_000_000,
            ..WorldCounters::default()
        });
        collector.set_memory_sample(MemorySample {
            rss_bytes: Some(512 * 1_048_576),
            ..Default::default()
        });
        collector.set_gpu_frame(GpuFrame::unavailable());
        if hitch {
            collector.push_event("terrain LOD rebuild", None);
        }
        collector.end_frame(wall, wall, i as f64 * 0.016);
    }

    // CPU vs GPU bound: GPU explicitly unavailable, CPU ~= wall.
    let latest = collector.latest().unwrap();
    assert!(!latest.gpu.available);
    assert!(latest.gpu.frame_s.is_none());

    // Long-tail hitch visible in p99/max but not median.
    let wall_stats = collector.frame_wall_stats().unwrap();
    assert!(wall_stats.p50 < 0.020);
    assert!(wall_stats.max >= 0.069);
    assert!(wall_stats.p99 > wall_stats.p50);

    // Terrain spike correlated with the hitch frame.
    let spike = collector
        .frames()
        .find(|f| f.world.terrain_patches_generated == 312)
        .unwrap();
    assert!(spike.frame_wall_s >= 0.069);

    // Warp keep-up answerable from the budget.
    assert!(!latest.sim.is_warp_limited());

    // Capture carries build/settings provenance.
    let mut metadata = default_capture_metadata();
    metadata.scenario = "test-flight".to_string();
    let capture = collector.snapshot_capture(metadata);
    assert_eq!(capture.frames.len(), 120);
    assert_eq!(capture.events.len(), 1);
    let json = capture.to_json_pretty().unwrap();
    assert!(json.contains("format_version"));
    let parsed = thessa_perf::PerfCapture::from_json(&json).unwrap();
    assert_eq!(parsed.frames.len(), 120);
    let csv = capture.to_csv();
    assert!(csv.lines().count() == 121); // header + 120 frames

    // Round-trip through files (serialization happens off the frame thread).
    let dir = std::env::temp_dir().join("thessa-perf-test");
    std::fs::create_dir_all(&dir).unwrap();
    let json_path = dir.join("perf-test.json");
    let csv_path = dir.join("perf-test.csv");
    capture.write_json(&json_path).unwrap();
    capture.write_csv(&csv_path).unwrap();
    assert!(json_path.exists() && csv_path.exists());
    std::fs::remove_file(json_path).ok();
    std::fs::remove_file(csv_path).ok();

    // One-line summary for logs.
    let summary = collector.summary_text();
    assert!(summary.contains("p95"));
    assert!(summary.contains("SIM"));
}
