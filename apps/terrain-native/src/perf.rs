//! Optional serial headless capture, adapting native GPU intervals into the
//! existing portable performance model. Not a second measurement framework.
//! Hosts the windowed overlay/short-capture adapter over the same collector.
use crate::scene::{Scene, SceneMeshes};
use thessa_perf::{
    CaptureMetadata, GpuFrame, PerfCollector, ProfilingLevel, SimBudget, WorldCounters,
    capture_stem, default_capture_metadata,
};
use thessa_render_wgpu::{Backend, BackendError, timing::GpuIntervals};

pub struct HeadlessPerf {
    collector: PerfCollector,
    gpu: Option<GpuIntervals>,
}

/// Terrain/streaming/RT counters shared by the headless capture and the
/// windowed overlay. Unknown counts retain defaults; nothing is synthesized.
pub(crate) fn world_counters(scene: &Scene, meshes: Option<&SceneMeshes>) -> WorldCounters {
    let jobs = scene.stream.pending_jobs();
    WorldCounters {
        terrain_patches_visible: scene.stream.visible().len() as u32,
        terrain_patches_wanted: scene.stream.wanted_count() as u32,
        terrain_cache_entries: scene.stream.cache_entries().0 as u32,
        terrain_jobs_in_flight: (jobs.0 + jobs.1) as u32,
        rt_instances: meshes
            .and_then(SceneMeshes::opaque)
            .map_or(0, |mesh| mesh.rt_stats().instances),
        ..Default::default()
    }
}

impl HeadlessPerf {
    pub fn new(backend: &Backend) -> Self {
        Self {
            collector: PerfCollector::with_default_capacity(),
            gpu: GpuIntervals::new(backend, 9),
        }
    }

    pub fn mark(&self, encoder: &mut wgpu::CommandEncoder, index: u32) {
        if let Some(gpu) = &self.gpu {
            gpu.mark(encoder, index);
        }
    }

    pub fn resolve(&self, encoder: &mut wgpu::CommandEncoder) {
        if let Some(gpu) = &self.gpu {
            gpu.resolve(encoder);
        }
    }

    /// Readback/serialization are outside measured frame samples. The normal
    /// path never constructs this adapter, queries, buffers or collector.
    pub fn record(
        &mut self,
        backend: &Backend,
        scene: &Scene,
        meshes: Option<&SceneMeshes>,
        wall_s: f64,
        cpu_submit_s: f64,
        stream_s: f64,
    ) -> Result<(), BackendError> {
        self.collector.begin_frame();
        self.collector.record_cpu_scope("world.streaming", stream_s);
        self.collector.record_cpu_scope("frame.cpu", cpu_submit_s);
        let gpu = match &self.gpu {
            Some(gpu) => {
                let values = gpu.read(backend)?;
                let names = [
                    "render.surface_and_shadows",
                    "render.rt_as_build",
                    "render.scene_classify_and_raster",
                    "render.motion_vectors",
                    "render.temporal_resolve",
                    "render.scene_overlays",
                    "render.display_transform",
                    "render.ui",
                ];
                GpuFrame {
                    available: true,
                    frame_s: Some(values.iter().sum()),
                    scopes: names
                        .into_iter()
                        .zip(values)
                        .map(|(n, v)| (n.to_string(), v))
                        .collect(),
                }
            }
            None => GpuFrame::unavailable(),
        };
        self.collector.set_gpu_frame(gpu);
        if let Some(budget) = scene.flight_budget(wall_s) {
            self.collector.set_sim_budget(budget);
        }
        self.collector
            .set_world_counters(world_counters(scene, meshes));
        self.collector
            .end_frame(wall_s, cpu_submit_s, scene.sample.time.0);
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn write(
        &self,
        path: &std::path::Path,
        backend: &Backend,
        scene: &Scene,
        display: &thessa_render_wgpu::display::DisplayTransform,
        sky_layers: &crate::sky_layers::SkyLayers,
        motion: Option<&thessa_render_wgpu::motion::SceneMotion>,
        temporal: Option<&thessa_render_wgpu::scene_temporal::SceneTemporal>,
        width: u32,
        height: u32,
        scenario: String,
    ) -> std::io::Result<()> {
        let mut meta = base_capture_metadata(backend, scene, display, sky_layers, motion, temporal);
        meta.display.width = width;
        meta.display.height = height;
        meta.display.vsync = false;
        meta.graphics
            .resolved
            .extend(scene.survey_capture_metadata(width, height));
        meta.graphics.resolved.insert("measurement_boundary".into(),"Serial offscreen frames after 30 warmup; frame.cpu ends after submit, excluding GPU wait; frame.wall includes GPU completion but excludes timestamp map/readback and capture writing. GPU command intervals include barriers/scheduling: surface_and_shadows (persistent topology classification/transactions/emission/deformation when selected, then shadow depth); rt_as_build; scene_classify_and_raster (opaque geometry and clear background); motion_vectors (opaque-depth diagnostics, empty when disabled); temporal_resolve (history compute, color copy-back and depth snapshot, empty when disabled); scene_overlays (plume, stars, cloud/aurora); display_transform (pre-exposed HDR to SDR, empty when HDR off); ui (after display). Layout v8 separates resolve from overlays; compare scope names and settings, not just totals, with older captures. Scene raster uses RGBA16Float when native_hdr=true. UI extra egui command buffers and queue-write uploads before the first mark are outside GPU intervals. Auto exposure and bloom remain unapplied in this manual HDR slice even when shared settings request them. Unknown world/sim counters retain defaults; no per-frame geometry readback. Up to 1800 measured frames retained".into());
        meta.scenario = scenario;
        meta.window_frames = self.collector.len();
        let capture = self.collector.snapshot_capture(meta);
        capture.write_json(path)?;
        capture.write_csv(&path.with_extension("csv"))?;
        println!(
            "perf_capture={} frames={} gpu_timestamps={}; diagnostic readback excluded from samples",
            path.display(),
            capture.frames.len(),
            self.gpu.is_some()
        );
        Ok(())
    }
}

/// Shared capture metadata for both adapters: platform/adapter identity and
/// the requested->resolved graphics correlation (perf spec section 12). Each
/// caller adds its own display/vsync, measurement boundary and scenario.
pub(crate) fn base_capture_metadata(
    backend: &Backend,
    scene: &Scene,
    display: &thessa_render_wgpu::display::DisplayTransform,
    sky_layers: &crate::sky_layers::SkyLayers,
    motion: Option<&thessa_render_wgpu::motion::SceneMotion>,
    temporal: Option<&thessa_render_wgpu::scene_temporal::SceneTemporal>,
) -> CaptureMetadata {
    let mut meta = default_capture_metadata();
    let info = backend.adapter.get_info();
    meta.platform.gpu = Some(info.name);
    meta.platform.backend = format!("{:?}; {}; {}", info.backend, info.driver, info.driver_info);
    meta.graphics.preset = format!("{:?}", scene.requested.preset);
    let resolved = backend.resolve_graphics(&scene.requested);
    meta.graphics.ray_tracing = format!("{:?}", resolved.ray_tracing);
    meta.graphics.resolved.extend(resolved.as_meta_map());
    meta.graphics.resolved.insert(
        "native_material_relief_residual_pages".into(),
        scene.material_relief_resident_pages.to_string(),
    );
    meta.graphics.resolved.insert(
        "native_diffuse_irradiance".into(),
        if resolved.surface_quality == thessa_graphics::Quality::Low {
            "isotropic"
        } else {
            "sh9"
        }
        .into(),
    );
    meta.graphics
        .resolved
        .insert("settings".into(), format!("{resolved:?}"));
    meta.graphics.resolved.insert(
        "requested_toml".into(),
        toml::to_string(&scene.requested).unwrap_or_default(),
    );
    meta.graphics.resolved.insert(
        "gpu_scope_layout".into(),
        "native-topology-v8-temporal".into(),
    );
    meta.graphics.resolved.extend(sky_layers.metadata());
    meta.graphics
        .resolved
        .extend(scene.surface_plants_metadata());
    for (key, value) in [
        ("native_material_resident_levels", format!("{:?}", scene.material_resident_levels)),
        ("native_material_reference_height_m", scene.stream.material_reference_height_m().to_string()),
        ("native_material_reference_height_source", "detail enabled: canonical field at eye direction / fixed 32 m wavelength, cached per selection epoch; disabled: legacy datum; not exact per-tile or collision height".into()),
        ("native_docking_fixture", scene.docking.is_some().to_string()),
        ("native_docking_clock", "isolated fixed 120 Hz replay; not flight authority or an ephemeris trajectory".into()),
        ("native_map_open", scene.map_active().to_string()),
        ("native_terrain_environment_samples", resolved.terrain_environment_samples().to_string()),
        ("native_terrain_environment_cloud_budget", format!("{:?}", resolved.terrain_environment_cloud_budget())),
        ("native_terrain_environment_scope", "render.scene_classify_and_raster; shared sky GGX visible-normal quadrature, including reflected cloud transport; no scene-object terrain reflections".into()),
        ("native_night_lights_extent", format!("{:?}", resolved.night_light_mask_size())),
        ("native_night_lights_source", "static inhabited-land R8 optical proxy; primary terrain only, not activity/electric power or secondary-ray emission".into()),
        ("native_hdr", display.enabled().to_string()),
        ("native_motion_vectors", motion.is_some().to_string()),
        ("native_taa", temporal.is_some().to_string()),
        ("native_taa_target_bytes", temporal.map_or(0, |t| t.target_bytes()).to_string()),
        ("native_taa_submitted_frames", temporal.map_or(0, |t| t.submitted_frames()).to_string()),
        ("native_taa_global_reset_frames", temporal.map_or(0, |t| t.reset_frames()).to_string()),
        ("native_taa_history_age_limit", temporal.map_or(0, |t| t.history_age_limit()).to_string()),
        ("native_taa_neighborhood_width", temporal.map_or(0, |t| t.neighborhood_width()).to_string()),
        ("native_motion_target_bytes", motion.map_or(0, |m| m.target_bytes()).to_string()),
        ("native_primary_jitter", motion.is_some_and(|m| m.jitter_enabled()).to_string()),
        ("native_jitter_submitted_frames", motion.map_or(0, |m| m.submitted_frames()).to_string()),
        ("native_jitter_scope", "Halton primary terrain/meshes only; motion excludes sampling jitter; resolved only when native_taa=true; secondary view unchanged".into()),
        ("native_motion_scope", "render.motion_vectors; stored opaque depth + alpha mask, before temporal resolve and translucent overlays".into()),
        ("native_taa_scope", "render.temporal_resolve; opaque history and depth snapshot before overlays, including copy-back; no upscaling or generated frames".into()),
        (
            "native_hdr_target_bytes",
            display.target_bytes().to_string(),
        ),
        (
            "native_scene_format",
            format!("{:?}", display.scene_format()),
        ),
        (
            "native_tonemapping",
            display.tone_mapping()
            .into(),
        ),
        ("native_auto_exposure_applied", "false".into()),
        ("native_bloom_applied", "false".into()),
        ("native_plume_light_applied", "false".into()),
        ("native_plume_light_status", "unavailable".into()),
        ("native_plume_enabled", resolved.plume_enabled.to_string()),
        ("native_plume_quality", format!("{:?}", resolved.plume_quality)),
        ("native_plume_scope", "render.scene_overlays (includes depth-clipped plume color-load pass when active)".into()),
    ] {
        meta.graphics.resolved.insert(key.into(), value);
    }
    meta
}

/// In-window overlay and short-capture mode over the same shared collector.
/// Owned by the window state, so ring history resets with a recreated window
/// surface — the same session-only contract as the egui HUD state. Frames are
/// always recorded (cheap at `Off`: wall/CPU only); scopes run at `Normal`
/// while the overlay or a capture is active, and F5 forces a level. GPU
/// timestamps stay explicitly unavailable in the windowed overlay: the
/// shared interval readback is a serial blocking poll that would stall the
/// live present loop, and CPU submit time must never be presented as GPU
/// time. Deterministic GPU measurement remains the headless `--perf-capture`
/// path.
#[derive(Debug)]
pub struct WindowPerf {
    collector: PerfCollector,
    overlay: bool,
    target: ProfilingLevel,
    forced_level: bool,
    capturing: bool,
    capture_start: u64,
    frames_since_start: u64,
    status: String,
    view: &'static str,
    width: u32,
    height: u32,
    vsync: bool,
}

impl Default for WindowPerf {
    fn default() -> Self {
        Self::new()
    }
}

impl WindowPerf {
    pub fn new() -> Self {
        let mut perf = Self {
            collector: PerfCollector::with_default_capacity(),
            overlay: false,
            target: ProfilingLevel::Off,
            forced_level: false,
            capturing: false,
            capture_start: 0,
            frames_since_start: 0,
            status: "PERF: F4 overlay · Shift+F4 capture".into(),
            view: "",
            width: 0,
            height: 0,
            vsync: false,
        };
        perf.apply_level();
        perf
    }

    #[cfg(feature = "hud")]
    pub fn overlay_text(&self, status_line: Option<&str>) -> Option<String> {
        self.overlay.then(|| self.text(status_line))
    }

    pub fn capturing(&self) -> bool {
        self.capturing
    }

    pub fn status(&self) -> &str {
        &self.status
    }

    pub fn toggle_overlay(&mut self) {
        self.overlay = !self.overlay;
        self.apply_level();
    }

    /// `F5`: force an explicit level until it is cycled back to `Off`
    /// through `Detailed`; overlay/capture activation no longer changes it.
    pub fn cycle_level(&mut self) {
        self.forced_level = true;
        self.target = match self.target {
            ProfilingLevel::Off => ProfilingLevel::Normal,
            ProfilingLevel::Normal => ProfilingLevel::Detailed,
            ProfilingLevel::Detailed => {
                self.forced_level = false;
                ProfilingLevel::Off
            }
        };
        self.apply_level();
        let event = format!("{:?}", self.target);
        self.collector
            .push_event("profiling level changed", Some(event));
    }

    fn apply_level(&mut self) {
        let target = if self.forced_level {
            self.target
        } else if self.overlay || self.capturing {
            ProfilingLevel::Normal
        } else {
            ProfilingLevel::Off
        };
        self.target = target;
        self.collector.set_level(target);
    }

    pub fn start_capture(&mut self) {
        if self.capturing {
            return;
        }
        self.capture_start = self
            .collector
            .latest()
            .map_or(0, |frame| frame.frame_index + 1);
        self.frames_since_start = 0;
        self.capturing = true;
        self.apply_level();
        self.collector
            .push_event("capture started", Some("window overlay".into()));
        self.status = "PERF: capturing… Shift+F4 to stop".into();
    }

    /// Stop and synchronously write the retained frames/events since the
    /// capture started. The caller builds capture metadata (it owns the
    /// backend/display/sky state) and picks the output directory; writes are
    /// small ring snapshots and rare, so an in-thread write is preferred
    /// over another executor dependency.
    pub fn stop_capture(&mut self, metadata: CaptureMetadata, base: &std::path::Path) -> String {
        if !self.capturing {
            return "PERF: no active capture".into();
        }
        self.capturing = false;
        self.status = "PERF: writing capture…".into();
        // The stop event must be recorded while the level still reflects the
        // active capture (Off after apply_level would silently drop it).
        self.collector.push_event("capture stopped", None);
        self.apply_level();
        let mut capture = self.collector.snapshot_capture(metadata);
        capture
            .frames
            .retain(|frame| frame.frame_index >= self.capture_start);
        capture
            .events
            .retain(|event| event.frame_index >= self.capture_start);
        capture.metadata.window_frames = capture.frames.len();
        let frames = capture.frames.len();
        let stem = capture_stem();
        let json_path = base.join(format!("{stem}.json"));
        let csv_path = base.join(format!("{stem}.csv"));
        match (capture.write_json(&json_path), capture.write_csv(&csv_path)) {
            (Ok(()), Ok(())) => {
                self.status = format!("PERF: capture wrote {frames} frames");
                self.status.clone()
            }
            (error, _) => {
                let text = format!("PERF: capture write failed: {error:?}");
                self.status = text.clone();
                text
            }
        }
    }

    pub fn begin_frame(&mut self, view: &'static str, vsync: bool, width: u32, height: u32) {
        self.view = view;
        self.vsync = vsync;
        self.width = width;
        self.height = height;
        self.collector.begin_frame();
    }

    pub fn record_scope(&mut self, name: &str, seconds: f64) {
        self.collector.record_cpu_scope(name, seconds);
    }

    pub fn end_frame(
        &mut self,
        wall_s: f64,
        cpu_s: f64,
        sim_time_s: f64,
        budget: Option<SimBudget>,
        counters: WorldCounters,
    ) {
        if let Some(budget) = budget {
            self.collector.set_sim_budget(budget);
        }
        self.collector.set_world_counters(counters);
        self.collector
            .record_cpu_scope(thessa_perf::scopes::FRAME_CPU, cpu_s);
        if self.capturing {
            self.frames_since_start += 1;
        }
        self.collector.end_frame(wall_s, cpu_s, sim_time_s);
    }

    #[cfg(feature = "hud")]
    fn text(&self, status_line: Option<&str>) -> String {
        let Some(wall) = self.collector.frame_wall_stats() else {
            return "PERF: warming up…".to_string();
        };
        let cpu = self.collector.frame_cpu_stats();
        let latest = self.collector.latest();
        let fps = if wall.current > 1e-9 {
            1.0 / wall.current
        } else {
            0.0
        };
        // The windowed overlay never measures GPU intervals (see struct doc):
        // report unavailability instead of passing CPU submit wait off as GPU.
        let gpu_line = "gpu unavailable (windowed; --perf-capture measures GPU)";
        let sim_line = match latest {
            Some(frame) => {
                let budget = &frame.sim;
                let server = if budget.has_server_sample() {
                    format!(
                        " srv cpu {:.1}s wall {:.1}s",
                        budget.server_compute_s, budget.server_wall_s
                    )
                } else {
                    String::new()
                };
                format!(
                    "steps {:>2} cpu {:5.2}ms rails {:.3}s warp x{:.1}/x{:.1} backlog {:5.2}ms{}",
                    budget.steps_this_frame,
                    budget.sim_cpu_s * 1000.0,
                    budget.rails_time_advanced_s,
                    budget.requested_warp,
                    budget.effective_warp,
                    budget.backlog_s * 1000.0,
                    server,
                )
            }
            None => "n/a".to_string(),
        };
        let world_line = match latest {
            Some(frame) => format!(
                "tiles {}/{} · jobs {} · cache {} · tris {} · RT {}",
                frame.world.terrain_patches_visible,
                frame.world.terrain_patches_wanted,
                frame.world.terrain_jobs_in_flight,
                frame.world.terrain_cache_entries,
                frame.world.terrain_triangles,
                frame.world.rt_instances,
            ),
            None => "n/a".to_string(),
        };
        let mem_line = match latest {
            Some(frame) => format!(
                "rss {}",
                frame.memory.rss_bytes.map_or_else(
                    || "unknown".to_string(),
                    |bytes| format!("{:.1} MiB", bytes as f64 / 1_048_576.0)
                ),
            ),
            None => "rss unknown".to_string(),
        };
        let capture_line = if self.capturing {
            format!(
                "CAPTURING {} frames  Shift+F4 stop",
                self.frames_since_start
            )
        } else {
            "Shift+F4 capture".to_string()
        };
        let scope_line = if self.collector.level() == ProfilingLevel::Detailed {
            let mut scopes: Vec<_> = latest
                .into_iter()
                .flat_map(|frame| frame.cpu_scopes.iter())
                .filter(|(name, seconds)| {
                    *name != thessa_perf::scopes::FRAME_CPU && **seconds >= 0.000_01
                })
                .collect();
            scopes.sort_by(|a, b| b.1.total_cmp(a.1));
            let scopes = scopes
                .into_iter()
                .take(7)
                .map(|(name, seconds)| format!("{name} {:.2}ms", seconds * 1000.0))
                .collect::<Vec<_>>()
                .join("  ");
            format!("scopes {scopes}")
        } else {
            "scopes need F5 detailed".to_string()
        };
        let level_line = match self.collector.level() {
            ProfilingLevel::Off => "off",
            ProfilingLevel::Normal => "normal",
            ProfilingLevel::Detailed => "detailed",
        };
        let status = status_line
            .map(|reason| format!("SIM: {reason}"))
            .unwrap_or_default();
        format!(
            "{}  {}x{}  [F4] overlay  [Shift+F4] capture  [F5] {}\nFRAME {:5.2}ms {:5.0}fps cpu {:5.2}ms {gpu_line}\n  p50 {:5.2} p95 {:5.2} p99 {:5.2} max {:5.2}ms (n={})\nSIM {}\nWORLD {}\nMEM {}\n{}\n{}{}{}",
            capture_line,
            self.width,
            self.height,
            level_line,
            wall.current * 1000.0,
            fps,
            cpu.map_or(0.0, |stats| stats.current * 1000.0),
            wall.p50 * 1000.0,
            wall.p95 * 1000.0,
            wall.p99 * 1000.0,
            wall.max * 1000.0,
            wall.count,
            sim_line,
            world_line,
            mem_line,
            scope_line,
            self.view,
            if status.is_empty() { "" } else { "\n" },
            status,
        )
    }
}

#[cfg(all(test, feature = "hud"))]
mod tests {
    use super::*;

    fn push_frames(perf: &mut WindowPerf, count: usize) {
        for index in 0..count {
            perf.begin_frame("SURFACE", false, 800, 600);
            perf.record_scope("sim.total", 0.000_5 + index as f64 * 0.000_1);
            perf.end_frame(0.016, 0.004, index as f64, None, WorldCounters::default());
        }
    }

    fn scratch_dir(name: &str) -> std::path::PathBuf {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target/test-tmp/window-perf")
            .join(name);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn off_level_keeps_frame_stats_and_scopes_need_activation() {
        let mut perf = WindowPerf::new();
        push_frames(&mut perf, 3);
        // Off retains wall/cpu in the ring but silently drops scopes.
        assert_eq!(
            perf.collector.scope_stats("sim.total").unwrap().current,
            0.0
        );
        assert_eq!(
            perf.collector.frame_cpu_stats().unwrap().current,
            0.004,
            "CPU frame time survives the Off level"
        );
        // F4 overlay activation raises the collection level.
        perf.toggle_overlay();
        assert_eq!(perf.collector.level(), ProfilingLevel::Normal);
        perf.begin_frame("SURFACE", false, 800, 600);
        perf.record_scope("sim.total", 0.0005);
        perf.end_frame(0.016, 0.004, 0.0, None, WorldCounters::default());
        assert!(perf.collector.scope_stats("sim.total").unwrap().current > 0.0);
        let text = perf.overlay_text(Some("paused")).unwrap();
        assert!(text.contains("[F4] overlay"));
        assert!(text.contains("[Shift+F4] capture"));
        assert!(text.contains("[F5] normal"), "{text}");
        assert!(text.contains("p50"));
        assert!(text.contains("paused"));
        assert!(text.contains("gpu unavailable"), "{text}");
        // Closing the overlay releases the collector back to Off.
        perf.toggle_overlay();
        assert_eq!(perf.collector.level(), ProfilingLevel::Off);
    }

    #[test]
    fn f5_forced_level_survives_overlay_close_until_cycled_off() {
        let mut perf = WindowPerf::new();
        assert_eq!(perf.collector.level(), ProfilingLevel::Off);
        perf.cycle_level();
        assert_eq!(perf.collector.level(), ProfilingLevel::Normal);
        perf.cycle_level();
        assert_eq!(perf.collector.level(), ProfilingLevel::Detailed);
        perf.toggle_overlay();
        perf.toggle_overlay();
        assert_eq!(
            perf.collector.level(),
            ProfilingLevel::Detailed,
            "F5-forced level survives overlay close"
        );
        perf.cycle_level();
        assert_eq!(perf.collector.level(), ProfilingLevel::Off);
        // Activation raises the level again after the forced cycle ends.
        perf.start_capture();
        assert_eq!(perf.collector.level(), ProfilingLevel::Normal);
        let dir = scratch_dir("forced");
        let _ = perf.stop_capture(default_capture_metadata(), &dir);
        assert_eq!(perf.collector.level(), ProfilingLevel::Off);
    }

    #[test]
    fn capture_retains_started_frames_and_writes_json_and_csv() {
        let mut perf = WindowPerf::new();
        push_frames(&mut perf, 3);
        perf.start_capture();
        assert!(perf.capturing());
        assert!(perf.status().contains("capturing"));
        push_frames(&mut perf, 2);
        let dir = scratch_dir("retain");
        let status = perf.stop_capture(default_capture_metadata(), &dir);
        // Ring contents recorded before the capture started are excluded.
        assert!(status.contains("wrote 2 frames"), "{status}");
        assert_eq!(perf.collector.level(), ProfilingLevel::Off);
        let json_path = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|entry| {
                let path = entry.unwrap().path();
                (path.extension()? == "json").then_some(path)
            })
            .next()
            .expect("capture json written");
        let capture =
            thessa_perf::PerfCapture::from_json(&std::fs::read_to_string(json_path).unwrap())
                .unwrap();
        assert_eq!(capture.frames.len(), 2);
        assert!(capture.frames[0].frame_index >= 3);
        assert!(
            capture
                .events
                .iter()
                .any(|event| event.name == "capture started")
        );
        assert!(
            capture
                .events
                .iter()
                .any(|event| event.name == "capture stopped")
        );
    }
}
