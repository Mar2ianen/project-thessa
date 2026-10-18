//! Optional serial headless capture, adapting native GPU intervals into the
//! existing portable performance model. Not a second measurement framework.
use crate::scene::{Scene, SceneMeshes};
use thessa_perf::{GpuFrame, PerfCollector, WorldCounters, default_capture_metadata};
use thessa_render_wgpu::{Backend, BackendError, timing::GpuIntervals};

pub struct HeadlessPerf {
    collector: PerfCollector,
    gpu: Option<GpuIntervals>,
}

impl HeadlessPerf {
    pub fn new(backend: &Backend) -> Self {
        Self {
            collector: PerfCollector::with_default_capacity(),
            gpu: GpuIntervals::new(backend, 6),
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
        let jobs = scene.stream.pending_jobs();
        self.collector.set_world_counters(WorldCounters {
            terrain_patches_visible: scene.stream.visible().len() as u32,
            terrain_patches_wanted: scene.stream.wanted_count() as u32,
            terrain_cache_entries: scene.stream.cache_entries().0 as u32,
            terrain_jobs_in_flight: (jobs.0 + jobs.1) as u32,
            rt_instances: meshes
                .and_then(SceneMeshes::opaque)
                .map_or(0, |mesh| mesh.rt_stats().instances),
            ..Default::default()
        });
        self.collector
            .end_frame(wall_s, cpu_submit_s, scene.sample.time.0);
        Ok(())
    }

    pub fn write(
        &self,
        path: &std::path::Path,
        backend: &Backend,
        scene: &Scene,
        display: &thessa_render_wgpu::display::DisplayTransform,
        scenario: String,
    ) -> std::io::Result<()> {
        let mut meta = default_capture_metadata();
        let info = backend.adapter.get_info();
        meta.platform.gpu = Some(info.name);
        meta.platform.backend =
            format!("{:?}; {}; {}", info.backend, info.driver, info.driver_info);
        meta.display.width = 1280;
        meta.display.height = 720;
        meta.display.vsync = false;
        meta.graphics.preset = format!("{:?}", scene.requested.preset);
        let resolved = backend.resolve_graphics(&scene.requested);
        meta.graphics.ray_tracing = format!("{:?}", resolved.ray_tracing);
        meta.graphics
            .resolved
            .insert("settings".into(), format!("{resolved:?}"));
        meta.graphics.resolved.insert(
            "requested_toml".into(),
            toml::to_string(&scene.requested).unwrap_or_default(),
        );
        meta.graphics
            .resolved
            .insert("gpu_scope_layout".into(), "native-topology-v4".into());
        for (key, value) in [
            ("native_hdr", display.enabled().to_string()),
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
                if display.enabled() {
                    "reinhard"
                } else {
                    "none"
                }
                .into(),
            ),
            ("native_auto_exposure_applied", "false".into()),
            ("native_bloom_applied", "false".into()),
        ] {
            meta.graphics.resolved.insert(key.into(), value);
        }
        meta.graphics.resolved.insert("measurement_boundary".into(),"Serial offscreen frames after 30 warmup; frame.cpu ends after submit, excluding GPU wait; frame.wall includes GPU completion but excludes timestamp map/readback and capture writing. GPU command intervals include barriers/scheduling: surface_and_shadows (persistent topology classification/transactions/emission/deformation when selected, then shadow depth); rt_as_build; scene_classify_and_raster; display_transform (pre-exposed HDR to SDR, empty when HDR off); ui (after display). Scene raster uses RGBA16Float when native_hdr=true; compare quality/format as well as scope names with older captures. UI extra egui command buffers and queue-write uploads before the first mark are outside GPU intervals. Auto exposure and bloom remain unapplied in this manual HDR slice even when shared settings request them. Unknown world/sim counters retain defaults; no per-frame geometry readback. Up to 1800 measured frames retained".into());
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
