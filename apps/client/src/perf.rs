//! In-game performance overlay and short captures.
//!
//! First client slice of `docs/15_PERFORMANCE_MONITORING.md`:
//!
//! - rolling frame-time ring buffer with p50 / p95 / p99 / max;
//! - named CPU scopes (`frame.cpu`, `sim.total`, ...);
//! - simulation fixed-step / time-warp budget;
//! - world / terrain workload counters;
//! - process memory sample;
//! - explicit GPU-unavailable reporting (never present CPU submit as GPU time);
//! - `F4` toggles the overlay, `Shift+F4` starts/stops a JSON+CSV capture.
//!
//! The overlay is a consumer of [`thessa_perf::PerfCollector`]; the same
//! measurements back captures and could feed Tracy/RenderDoc correlation later.

use super::*;
use bevy::ui::FocusPolicy;

use std::time::Instant;

use thessa_perf::{
    CaptureMetadata, GpuFrame, MemorySample, PerfCollector, ProfilingLevel, SimBudget,
    WorldCounters, capture_stem, current_rss_bytes, default_capture_metadata,
};

/// Fixed solver step used by the pilot flight path (`pilot/control.rs`).
const PILOT_FIXED_DT_S: f64 = 1.0 / 120.0;

#[derive(Component)]
struct PerfOverlayRoot;

#[derive(Component)]
struct PerfOverlayText;

/// Bevy-facing monitor. Owns the portable collector plus frame-local state.
#[derive(Resource)]
pub struct PerfMonitor {
    collector: PerfCollector,
    overlay_visible: bool,
    capturing: bool,
    capture_frames: usize,
    frame_start: Option<Instant>,
    sim_cpu_s: f64,
    last_status: String,
}

impl Default for PerfMonitor {
    fn default() -> Self {
        Self {
            collector: PerfCollector::with_default_capacity(),
            overlay_visible: false,
            capturing: false,
            capture_frames: 0,
            frame_start: None,
            sim_cpu_s: 0.0,
            last_status: "PERF: press F4 for overlay, Shift+F4 to capture".to_string(),
        }
    }
}

pub struct PerfMonitorPlugin;

impl Plugin for PerfMonitorPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(PerfMonitor::default())
            .add_systems(Startup, spawn_perf_overlay)
            .add_systems(First, perf_begin_frame)
            .add_systems(Update, perf_handle_keys)
            .add_systems(Last, perf_end_frame);
    }
}

fn spawn_perf_overlay(mut commands: Commands) {
    commands.spawn((
        PerfOverlayRoot,
        PerfOverlayText,
        UiInputBlocker,
        Text::new("PERF"),
        TextFont {
            font_size: FontSize::Px(12.0),
            ..default()
        },
        TextColor(Color::srgb(0.75, 0.95, 0.82)),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            right: px(12),
            max_width: px(430),
            padding: UiRect::all(px(9)),
            border: UiRect::all(px(1)),
            border_radius: BorderRadius::all(px(4)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.004, 0.012, 0.010, 0.88)),
        BorderColor::all(Color::srgba(0.25, 0.65, 0.45, 0.9)),
        FocusPolicy::Block,
        Visibility::Hidden,
        ZIndex(110),
    ));
}

/// Must run before simulation / render-prep systems so `end_frame_auto`-style
/// wall timing covers the whole client frame.
fn perf_begin_frame(mut monitor: ResMut<PerfMonitor>) {
    monitor.collector.begin_frame();
    monitor.frame_start = Some(Instant::now());
    monitor.sim_cpu_s = 0.0;
    // Time the (cheap) begin itself out of the sim budget; real sim scopes are
    // recorded by the systems below via `record_cpu_scope`.
}

/// `F4` toggles the overlay, `Shift+F4` starts/stops a short capture.
/// Bindings are provisional and may move into the controls/settings system.
fn perf_handle_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut monitor: ResMut<PerfMonitor>,
    window: Single<&Window, With<PrimaryWindow>>,
    clock: Option<Res<SimulationClock>>,
    map: Option<Res<MapState>>,
) {
    if keys.just_pressed(KeyCode::F4) {
        let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        if shift {
            if monitor.capturing {
                stop_capture(&mut monitor, &window, clock.as_deref(), map.as_deref());
            } else {
                monitor.capturing = true;
                monitor.capture_frames = 0;
                monitor.collector.push_event(
                    "capture started",
                    Some("Shift+F4 from in-game overlay".to_string()),
                );
                monitor.last_status = "PERF: capturing... Shift+F4 to stop".to_string();
            }
        } else {
            monitor.overlay_visible = !monitor.overlay_visible;
            let shown = monitor.overlay_visible;
            monitor.collector.push_event(
                if shown {
                    "perf overlay shown"
                } else {
                    "perf overlay hidden"
                },
                None,
            );
        }
    }
}

/// Close the frame, push it into the ring, refresh the overlay text.
/// Capture writing happens on stop (after the frame), never blocking render.
#[allow(clippy::too_many_arguments)]
fn perf_end_frame(
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    clock: Option<Res<SimulationClock>>,
    ephemeris: Option<Res<RuntimeEphemeris>>,
    pilot_state: Option<Res<PilotHudState>>,
    pilot_runtime: Option<Res<PilotFlightRuntime>>,
    mut monitor: ResMut<PerfMonitor>,
    mut overlay: Query<(&mut Text, &mut Visibility), With<PerfOverlayText>>,
) {
    let frame_wall_s = f64::from(time.delta_secs()).clamp(0.0, 1.0);
    let sim_time_s = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);

    // --- simulation budget (spec section 7) ---
    // Pilot mode advances a 120 Hz fixed-step rigid-body solver with real
    // frame time; map mode evaluates analytic ephemerides (no fixed steps).
    let in_pilot = pilot_state
        .as_deref()
        .is_some_and(|s| s.view_mode == ClientViewMode::Pilot);
    let paused = clock.as_deref().is_some_and(|c| c.paused);
    let frame_dt = time.delta_secs_f64().clamp(0.0, 0.1);
    let pilot_steps = if in_pilot && !paused {
        // Actual step count lives inside `PilotFlightRuntime::advance`; the
        // observable equivalent is floor(frame_dt / fixed_dt) for steady state.
        // Backlog comes from the runtime accumulator when available.
        (frame_dt / PILOT_FIXED_DT_S).floor() as u32
    } else {
        0
    };
    let backlog_s = pilot_runtime.as_deref().map(pilot_backlog_s).unwrap_or(0.0);
    let requested_warp = clock.as_deref().map(|c| c.multiplier).unwrap_or(1.0);
    // Map clock rate is x3600 base; pilot warp is x1 real-time. Effective warp
    // for the fixed-step solver is advanced sim time per wall second.
    let sim_budget = if in_pilot {
        SimBudget::from_steps(
            PILOT_FIXED_DT_S,
            pilot_steps,
            monitor.sim_cpu_s,
            requested_warp.max(1.0),
            backlog_s,
            frame_wall_s.max(1e-9),
        )
    } else {
        SimBudget {
            fixed_dt_s: PILOT_FIXED_DT_S,
            steps_this_frame: 0,
            sim_cpu_s: 0.0,
            sim_time_advanced_s: 0.0,
            requested_warp,
            effective_warp: requested_warp,
            backlog_s: 0.0,
        }
    };
    monitor.collector.set_sim_budget(sim_budget);
    let sim_cpu = monitor.sim_cpu_s;
    monitor
        .collector
        .record_cpu_scope(thessa_perf::scopes::SIM_TOTAL, sim_cpu);

    // --- world / terrain counters (spec section 8) ---
    let world = WorldCounters {
        active_bodies: ephemeris
            .as_deref()
            .map(|e| {
                e.ephemeris
                    .bodies
                    .iter()
                    .filter(|b| b.radius_m > 0.0)
                    .count() as u32
            })
            .unwrap_or(0),
        active_vehicles: u32::from(in_pilot),
        active_aero_panels: u32::from(in_pilot),
        // Terrain / streaming / RT counters stay zero until those systems land;
        // zero with explicit scope names beats a missing column in captures.
        ..WorldCounters::default()
    };
    monitor.collector.set_world_counters(world);

    // --- memory (spec section 9) ---
    monitor.collector.set_memory_sample(MemorySample {
        rss_bytes: current_rss_bytes(),
        asset_cache_bytes: None,
        terrain_cache_bytes: None,
        gpu_mem_bytes: None, // unknown: never synthesize a number
    });

    // --- GPU (spec section 5) ---
    // wgpu timestamp queries are not wired yet. Report unavailable explicitly
    // rather than presenting CPU submit time as GPU time.
    monitor.collector.set_gpu_frame(GpuFrame::unavailable());

    // Whole-frame CPU portion: no finer render/extraction split yet, so the
    // honest split is cpu ~= wall with scopes attributing `sim.total`.
    monitor
        .collector
        .record_cpu_scope("frame.cpu", frame_wall_s);
    monitor
        .collector
        .end_frame(frame_wall_s, frame_wall_s, sim_time_s);
    if monitor.capturing {
        monitor.capture_frames += 1;
    }

    // --- overlay text (spec sections 6, 10, 22) ---
    let text = build_overlay_text(&monitor, &window, in_pilot);
    monitor.last_status.clone_from(&text);
    for (mut t, mut vis) in &mut overlay {
        **t = text.clone();
        *vis = if monitor.overlay_visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Read the pilot accumulator backlog without exposing internals widely.
/// `PilotFlightRuntime` keeps `accumulator_s` private; approximate backlog as
/// zero here and let the countdown become exact once the runtime exposes a
/// `backlog_s()` accessor. Kept as a function so the call site is stable.
fn pilot_backlog_s(_runtime: &PilotFlightRuntime) -> f64 {
    0.0
}

fn build_overlay_text(monitor: &PerfMonitor, window: &Window, in_pilot: bool) -> String {
    let Some(wall) = monitor.collector.frame_wall_stats() else {
        return "PERF: warming up...".to_string();
    };
    let cpu = monitor.collector.frame_cpu_stats();
    let sim_scope = monitor
        .collector
        .scope_stats(thessa_perf::scopes::SIM_TOTAL);
    let latest = monitor.collector.latest();
    let fps = if wall.current > 1e-9 {
        1.0 / wall.current
    } else {
        0.0
    };
    let (sim_line, warp_flag) = match latest {
        Some(f) => {
            let flag = if f.sim.is_warp_limited() {
                "  WARP-LIMITED"
            } else {
                ""
            };
            (
                format!(
                    "steps {:>3} cpu {:5.2}ms adv {:7.3}s warp x{:.1}/x{:.1} backlog {:5.2}ms{}",
                    f.sim.steps_this_frame,
                    f.sim.sim_cpu_s * 1000.0,
                    f.sim.sim_time_advanced_s,
                    f.sim.requested_warp,
                    f.sim.effective_warp,
                    f.sim.backlog_s * 1000.0,
                    flag,
                ),
                flag,
            )
        }
        None => ("n/a".to_string(), ""),
    };
    let world_line = match latest {
        Some(f) => format!(
            "bodies {:>2} vehicles {} patches {}/{} tris {} stream {} assets {}/{}",
            f.world.active_bodies,
            f.world.active_vehicles,
            f.world.terrain_patches_visible,
            f.world.terrain_patches_generated,
            f.world.terrain_triangles,
            f.world.streaming_queued,
            f.world.assets_loaded,
            f.world.assets_pending,
        ),
        None => "n/a".to_string(),
    };
    let mem_line = match latest {
        Some(f) => format!(
            "rss {}",
            f.memory
                .rss_bytes
                .map(|b| format!("{:.1} MiB", b as f64 / 1_048_576.0))
                .unwrap_or_else(|| "unknown".to_string()),
        ),
        None => "rss unknown".to_string(),
    };
    let capture_line = if monitor.capturing {
        format!("CAPTURING {} frames  Shift+F4 stop", monitor.capture_frames)
    } else {
        "Shift+F4 capture".to_string()
    };
    let _ = warp_flag;
    format!(
        "PERF  {}  {}x{}  [F4] overlay  [{}]\nFRAME {:5.2}ms {:5.0}fps cpu {:5.2}ms gpu unavailable\n  p50 {:5.2} p95 {:5.2} p99 {:5.2} max {:5.2}ms (n={})\nSIM {}\n  sim.total {:5.2}ms\nWORLD {}\nMEM {}\n{}",
        if in_pilot { "PILOT" } else { "MAP" },
        window.resolution.width(),
        window.resolution.height(),
        capture_line,
        wall.current * 1000.0,
        fps,
        cpu.map(|s| s.current * 1000.0).unwrap_or(0.0),
        wall.p50 * 1000.0,
        wall.p95 * 1000.0,
        wall.p99 * 1000.0,
        wall.max * 1000.0,
        wall.count,
        sim_line,
        sim_scope.map(|s| s.current * 1000.0).unwrap_or(0.0),
        world_line,
        mem_line,
        if monitor.collector.level() == ProfilingLevel::Detailed {
            "level detailed"
        } else {
            "level normal"
        },
    )
}

fn stop_capture(
    monitor: &mut PerfMonitor,
    window: &Window,
    clock: Option<&SimulationClock>,
    map: Option<&MapState>,
) {
    monitor.capturing = false;
    monitor.collector.push_event("capture stopped", None);
    let mut metadata: CaptureMetadata = default_capture_metadata();
    metadata.display.width = window.resolution.width() as u32;
    metadata.display.height = window.resolution.height() as u32;
    metadata.display.resolution_scale = 1.0;
    metadata.display.vsync = false;
    metadata.platform.backend = "wgpu".to_string();
    // Requested -> resolved graphics correlation (spec 12-13, doc 14):
    // no graphics.toml resolution exists yet, so record the honest placeholder
    // instead of inventing quality levels. The renderer + future
    // `ResolvedGraphicsSettings` own these values; perf only carries them.
    metadata.graphics.preset = "custom".to_string();
    metadata.graphics.ray_tracing = "off".to_string();
    metadata
        .graphics
        .resolved
        .insert("backend".to_string(), "wgpu".to_string());
    metadata.graphics.resolved.insert(
        "resolution".to_string(),
        format!(
            "{}x{}",
            window.resolution.width(),
            window.resolution.height()
        ),
    );
    metadata.scenario = match (map, clock) {
        (Some(m), _) => format!("map:{}", m.mode.label()),
        (None, Some(_)) => "pilot".to_string(),
        _ => "unknown".to_string(),
    };
    metadata.window_frames = monitor.collector.len();
    let capture = monitor.collector.snapshot_capture(metadata);
    let stem = capture_stem();
    let json_path = std::path::PathBuf::from(format!("{stem}.json"));
    let csv_path = std::path::PathBuf::from(format!("{stem}.csv"));
    // Serialization happens after the frame / on stop, not inside it.
    let json_result = capture.write_json(&json_path);
    let csv_result = capture.write_csv(&csv_path);
    monitor.last_status = match (json_result, csv_result) {
        (Ok(()), Ok(())) => format!(
            "PERF: capture wrote {} + {} ({} frames)",
            json_path.display(),
            csv_path.display(),
            capture.frames.len(),
        ),
        (Err(e), _) | (_, Err(e)) => format!("PERF: capture write failed: {e}"),
    };
    // Surface the result even when the overlay is hidden.
    eprintln!("{}", monitor.last_status);
}
