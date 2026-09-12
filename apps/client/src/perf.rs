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

use bevy::tasks::{IoTaskPool, Task, block_on, poll_once};
use std::time::Instant;

use super::atmosphere::{GraphicsResolved, RayTracingActive};
use thessa_perf::{
    CaptureMetadata, GpuFrame, MemorySample, PerfCollector, ProfilingLevel, SimBudget,
    WorldCounters, capture_stem, current_rss_bytes, default_capture_metadata,
};

/// Fixed solver step used by the pilot flight path (`pilot/control.rs`).
const PILOT_FIXED_DT_S: f64 = thessa_sim_core::WORLD_TICK_S;

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
    capture_start: u64,
    export_task: Option<Task<String>>,
    overlay_updated: Option<Instant>,
    frame_start: Option<Instant>,
    sim_cpu_s: f64,
    last_status: String,
}

impl PerfMonitor {
    /// Record a named CPU scope from another client subsystem (e.g. the
    /// atmosphere plugin reporting `render.atmosphere`).
    pub(super) fn record_cpu_scope(&mut self, name: &str, seconds: f64) {
        self.collector.record_cpu_scope(name, seconds);
    }

    /// Record a lightweight capture event marker from another subsystem.
    pub(super) fn push_event(&mut self, name: &str, detail: Option<String>) {
        self.collector.push_event(name, detail);
    }
}

impl Default for PerfMonitor {
    fn default() -> Self {
        Self {
            collector: PerfCollector::with_default_capacity(),
            overlay_visible: false,
            capturing: false,
            capture_frames: 0,
            capture_start: 0,
            export_task: None,
            overlay_updated: None,
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
#[allow(clippy::too_many_arguments)]
fn perf_handle_keys(
    keys: Res<ButtonInput<KeyCode>>,
    mut monitor: ResMut<PerfMonitor>,
    window: Single<&Window, With<PrimaryWindow>>,
    clock: Option<Res<SimulationClock>>,
    map: Option<Res<MapState>>,
    pilot: Res<PilotHudState>,
    survey: Res<terrain::SurfaceSurvey>,
    graphics: Option<Res<GraphicsResolved>>,
) {
    if let Some(task) = monitor.export_task.as_mut()
        && let Some(status) = block_on(poll_once(task))
    {
        eprintln!("{status}");
        monitor.last_status = status;
        monitor.export_task = None;
    }
    if keys.just_pressed(KeyCode::F4) {
        let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        if shift {
            if monitor.capturing {
                stop_capture(
                    &mut monitor,
                    &window,
                    clock.as_deref(),
                    map.as_deref(),
                    if survey.active {
                        "surface"
                    } else if pilot.view_mode == ClientViewMode::Pilot {
                        "pilot"
                    } else {
                        "map"
                    },
                    graphics.as_deref(),
                );
            } else if monitor.export_task.is_none() {
                monitor.capture_start = monitor
                    .collector
                    .latest()
                    .map(|f| f.frame_index + 1)
                    .unwrap_or(0);
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
/// Capture serialization and disk writes run on the IO pool.
#[allow(clippy::too_many_arguments)]
fn perf_end_frame(
    time: Res<Time<Real>>,
    window: Single<&Window, With<PrimaryWindow>>,
    clock: Option<Res<SimulationClock>>,
    ephemeris: Option<Res<RuntimeEphemeris>>,
    pilot_state: Option<Res<PilotHudState>>,
    pilot_runtime: Option<Res<PilotFlightRuntime>>,
    terrain: Option<Res<terrain::WorldTerrain>>,
    survey: Res<terrain::SurfaceSurvey>,
    visible_tiles: Query<(&ViewVisibility, &Mesh3d), With<terrain::SurfaceTile>>,
    meshes: Res<Assets<Mesh>>,
    rt_instances: Query<(), With<bevy::solari::prelude::RaytracingMesh3d>>,
    rt_active: Option<Res<RayTracingActive>>,
    mut monitor: ResMut<PerfMonitor>,
    mut overlay: Query<(&mut Text, &mut Visibility), With<PerfOverlayText>>,
) {
    let frame_wall_s = time.delta_secs_f64();
    let sim_time_s = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);

    // --- simulation budget (spec section 7) ---
    // Both views share solver ticks and direct cached-coast advancement.
    let in_pilot = pilot_state
        .as_deref()
        .is_some_and(|s| s.view_mode == ClientViewMode::Pilot);
    let pilot_steps = pilot_runtime
        .as_deref()
        .map(|r| r.steps_this_frame)
        .unwrap_or(0);
    let backlog_s = pilot_runtime
        .as_deref()
        .map(PilotFlightRuntime::backlog_s)
        .unwrap_or(0.0);
    let requested_warp = clock.as_deref().map(|c| c.multiplier).unwrap_or(1.0);
    // Effective warp is advanced sim time per wall second in either view.
    let sim_budget = if pilot_runtime.is_some() {
        SimBudget::from_steps(
            PILOT_FIXED_DT_S,
            pilot_steps,
            monitor.sim_cpu_s,
            requested_warp,
            backlog_s,
            frame_wall_s.max(1e-9),
        )
        .with_rails_time(
            pilot_runtime
                .as_deref()
                .map_or(0.0, |r| r.rails_advanced_this_frame),
            frame_wall_s.max(1e-9),
        )
    } else {
        SimBudget {
            fixed_dt_s: PILOT_FIXED_DT_S,
            steps_this_frame: 0,
            sim_cpu_s: 0.0,
            sim_time_advanced_s: 0.0,
            rails_time_advanced_s: 0.0,
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
    let mut world = WorldCounters {
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
        active_vehicles: u32::from(pilot_runtime.is_some()),
        active_aero_panels: if pilot_runtime.is_some() {
            pilot_runtime
                .as_deref()
                .map(PilotFlightRuntime::panel_count)
                .unwrap_or(0)
        } else {
            0
        },
        // Terrain / streaming / RT counters stay zero until those systems land;
        // zero with explicit scope names beats a missing column in captures.
        ..terrain.as_deref().map(|w| w.counters).unwrap_or_default()
    };
    world.terrain_patches_visible = 0;
    world.terrain_vertices = 0;
    world.terrain_triangles = 0;
    for (visibility, mesh) in &visible_tiles {
        if !visibility.get() {
            continue;
        }
        world.terrain_patches_visible += 1;
        if let Some(mesh) = meshes.get(&mesh.0) {
            world.terrain_vertices += mesh.count_vertices() as u64;
            world.terrain_triangles += mesh.indices().map(|i| i.len() / 3).unwrap_or(0) as u64;
        }
    }
    world.rt_instances = rt_instances.iter().count() as u32;
    monitor.collector.set_world_counters(world);

    // --- memory (spec section 9) ---
    monitor.collector.set_memory_sample(MemorySample {
        rss_bytes: current_rss_bytes(),
        asset_cache_bytes: None,
        terrain_cache_bytes: terrain.as_deref().map(|w| w.cache_bytes),
        gpu_mem_bytes: None, // unknown: never synthesize a number
    });

    // --- GPU (spec section 5) ---
    // wgpu timestamp queries are not wired yet. Report unavailable explicitly
    // rather than presenting CPU submit time as GPU time.
    monitor.collector.set_gpu_frame(GpuFrame::unavailable());

    // Elapsed main-app schedule interval, excludes vsync between frames.
    // This is wall duration of CPU work, not OS per-thread CPU utilization.
    let cpu_s = monitor
        .frame_start
        .map(|t| t.elapsed().as_secs_f64())
        .unwrap_or(0.0);
    monitor.collector.record_cpu_scope("frame.cpu", cpu_s);
    monitor.collector.end_frame(frame_wall_s, cpu_s, sim_time_s);
    if monitor.capturing {
        monitor.capture_frames += 1;
    }

    // --- overlay text (spec sections 6, 10, 22) ---
    for (_, mut vis) in &mut overlay {
        *vis = if monitor.overlay_visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    if monitor.overlay_visible
        && monitor
            .overlay_updated
            .is_none_or(|t| t.elapsed().as_secs_f32() >= 0.25)
    {
        let text = build_overlay_text(
            &monitor,
            &window,
            if survey.active {
                "SURFACE"
            } else if in_pilot {
                "PILOT"
            } else {
                "MAP"
            },
            rt_active.as_deref().is_some_and(|flag| flag.0),
            if let Some(reason) = pilot_runtime
                .as_deref()
                .and_then(PilotFlightRuntime::stop_reason)
            {
                Some(reason)
            } else if clock.as_deref().is_some_and(|clock| clock.paused) {
                Some("paused")
            } else {
                None
            },
        );
        monitor.overlay_updated = Some(Instant::now());
        for (mut t, _) in &mut overlay {
            **t = text.clone();
        }
    }
}

impl PerfMonitor {
    pub(super) fn record_sim(&mut self, seconds: f64) {
        self.sim_cpu_s += seconds;
    }
    pub(super) fn record_scope(&mut self, name: &str, seconds: f64) {
        self.collector.record_cpu_scope(name, seconds);
    }
}

fn build_overlay_text(
    monitor: &PerfMonitor,
    window: &Window,
    view: &str,
    rt_on: bool,
    stop_reason: Option<&str>,
) -> String {
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
            let flag = if stop_reason.is_some() {
                "  STOPPED / PAUSED"
            } else if f.sim.is_warp_limited() {
                "  WARP-LIMITED"
            } else {
                ""
            };
            (
                format!(
                    "steps {:>3} cpu {:5.2}ms adv {:7.3}s rails {:.3}s warp x{:.1}/x{:.1} backlog {:5.2}ms{}",
                    f.sim.steps_this_frame,
                    f.sim.sim_cpu_s * 1000.0,
                    f.sim.sim_time_advanced_s,
                    f.sim.rails_time_advanced_s,
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
            "bodies {:>2} vehicles {} visible {} new/frame {} tris {} queue {} assets {}/{} RT {}",
            f.world.active_bodies,
            f.world.active_vehicles,
            f.world.terrain_patches_visible,
            f.world.terrain_patches_generated,
            f.world.terrain_triangles,
            f.world.streaming_queued,
            f.world.assets_loaded,
            f.world.assets_pending,
            f.world.rt_instances,
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
    let status = stop_reason
        .map(|reason| format!("\nSIM: {reason}"))
        .unwrap_or_default();
    format!(
        "PERF  {}  {}x{}  [F4] overlay  [{}]  [Shift+F12 RT:{}]\nFRAME {:5.2}ms {:5.0}fps cpu {:5.2}ms gpu unavailable\n  p50 {:5.2} p95 {:5.2} p99 {:5.2} max {:5.2}ms (n={})\nSIM {}\n  sim.total {:5.2}ms\nWORLD {}\nMEM {}\n{}{status}",
        view,
        window.resolution.physical_width(),
        window.resolution.physical_height(),
        capture_line,
        if rt_on { "on" } else { "off" },
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
    _clock: Option<&SimulationClock>,
    map: Option<&MapState>,
    view: &str,
    graphics: Option<&GraphicsResolved>,
) {
    monitor.capturing = false;
    monitor.collector.push_event("capture stopped", None);
    let mut metadata: CaptureMetadata = default_capture_metadata();
    metadata.display.width = window.resolution.physical_width();
    metadata.display.height = window.resolution.physical_height();
    metadata.platform.backend = "wgpu".to_string();
    // Requested -> resolved graphics correlation (perf spec section 12, visual
    // atmosphere spec sections 12-13): the perf system never interprets
    // quality itself, it carries ResolvedGraphicsSettings as metadata.
    if let Some(resolved) = graphics {
        metadata.display.resolution_scale = resolved.0.resolution_scale;
        metadata.display.vsync = resolved.0.vsync;
        metadata.graphics.preset = resolved.0.preset_label.clone();
        metadata.graphics.ray_tracing = resolved.0.ray_tracing.as_str().to_string();
        metadata.graphics.resolved = resolved.0.as_meta_map();
    } else {
        metadata.graphics.preset = "unknown".to_string();
        metadata.graphics.ray_tracing = "unknown".to_string();
        metadata
            .graphics
            .resolved
            .insert("backend".to_string(), "wgpu".to_string());
    }
    metadata.graphics.resolved.insert(
        "resolution".to_string(),
        format!(
            "{}x{}",
            window.resolution.physical_width(),
            window.resolution.physical_height()
        ),
    );
    metadata.scenario = if view == "map" {
        format!("map:{}", map.map(|m| m.mode.label()).unwrap_or("unknown"))
    } else {
        view.to_string()
    };
    let mut capture = monitor.collector.snapshot_capture(metadata);
    capture
        .frames
        .retain(|f| f.frame_index >= monitor.capture_start);
    capture
        .events
        .retain(|e| e.frame_index >= monitor.capture_start);
    capture.metadata.window_frames = capture.frames.len();
    let stem = capture_stem();
    let json_path = std::path::PathBuf::from(format!("{stem}.json"));
    let csv_path = std::path::PathBuf::from(format!("{stem}.csv"));
    monitor.export_task = Some(IoTaskPool::get().spawn(async move {
        let json_result = capture.write_json(&json_path);
        let csv_result = capture.write_csv(&csv_path);
        match (json_result, csv_result) {
            (Ok(()), Ok(())) => format!(
                "PERF: capture wrote {} + {} ({} frames)",
                json_path.display(),
                csv_path.display(),
                capture.frames.len(),
            ),
            (Err(e), _) | (_, Err(e)) => format!("PERF: capture write failed: {e}"),
        }
    }));
    monitor.last_status = "PERF: writing capture…".into();
}
