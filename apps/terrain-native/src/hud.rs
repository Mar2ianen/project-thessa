//! App-owned egui HUD. The GPU pass loads the terrain's existing color target;
//! it never imports/copies the scene or creates another GPU device.
use thessa_render_wgpu::Backend;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub vehicle_preview: bool,
    pub chase: bool,
    pub reset_camera: bool,
    pub angle: f64,
    pub distance: f64,
    pub throttle: f64,
    pub engine: bool,
    pub precision: bool,
    pub sas: bool,
    pub cycle_mode: bool,
    pub exposure_ev100: f32,
    pub vsync: bool,
}

impl Controls {
    pub fn from_scene(scene: &crate::scene::Scene) -> Self {
        Self {
            vehicle_preview: scene.vehicle_preview(),
            chase: scene.camera.chase(),
            reset_camera: false,
            angle: scene.angle,
            distance: if scene.vehicle_preview() {
                f64::from(scene.camera.distance_m())
            } else {
                scene.distance
            },
            throttle: scene.input.throttle,
            engine: scene.input.engine_active,
            precision: scene.input.precision,
            sas: scene.sas_enabled(),
            cycle_mode: false,
            exposure_ev100: scene.requested.renderer.exposure_ev100,
            vsync: scene.requested.renderer.vsync,
        }
    }

    /// Graphics edits remain requests and must go through capability resolution.
    pub fn apply(self, scene: &mut crate::scene::Scene) {
        scene.camera.set_chase(self.chase);
        if self.reset_camera {
            if scene.vehicle_preview() {
                scene.camera.reset();
            } else {
                scene.angle = 0.0;
                scene.distance = 30_000.0;
            }
            scene.requested.renderer.exposure_ev100 = self.exposure_ev100;
            scene.requested.renderer.vsync = self.vsync;
            return;
        }
        scene.angle = self.angle;
        if scene.vehicle_preview() {
            scene.camera.set_distance_m(self.distance as f32);
        } else {
            scene.distance = self.distance;
        }
        if scene.live_flight() {
            scene.input.set_throttle(self.throttle);
            scene.input.engine_active = self.engine;
            scene.input.precision = self.precision;
            scene.set_sas(self.sas);
            if self.cycle_mode {
                scene.cycle_control_mode();
            }
        }
        scene.requested.renderer.exposure_ev100 = self.exposure_ev100;
        scene.requested.renderer.vsync = self.vsync;
    }
}

#[derive(Default)]
pub struct Stats {
    pub live_flight: bool,
    pub flight_stopped: bool,
    pub chase: bool,
    pub throttle: f64,
    pub engine: bool,
    pub control_mode: Option<thessa_flight_authority::ControlMode>,
    pub stick: [f64; 3],
    pub submitted_frames: u64,
    pub previous_cpu_ms: f64,
    pub sim_time_s: f64,
    pub tiles: usize,
    pub cache: (usize, usize),
    pub pending: (usize, usize),
    pub publications: u64,
    pub regenerated: u64,
    pub upload_bytes: u64,
    pub capacity_rejections: u64,
}

impl Stats {
    pub fn from_scene(scene: &crate::scene::Scene, frames: u64, cpu_ms: f64) -> Self {
        Self {
            submitted_frames: frames,
            live_flight: scene.live_flight(),
            flight_stopped: scene.flight_stopped(),
            chase: scene.camera.chase(),
            throttle: scene.input.throttle,
            engine: scene.input.engine_active,
            control_mode: scene.live_flight().then_some(scene.control_mode),
            stick: scene.input.stick_scaled().to_array(),
            previous_cpu_ms: cpu_ms,
            sim_time_s: scene.sample.time.0,
            tiles: scene.stream.visible().len(),
            cache: scene.stream.cache_entries(),
            pending: scene.stream.pending_jobs(),
            publications: scene.publications,
            regenerated: scene.regenerated,
            upload_bytes: scene.material_upload_bytes,
            capacity_rejections: scene.capacity_rejections,
        }
    }
}

pub struct Hud {
    pub context: egui::Context,
    renderer: egui_wgpu::Renderer,
    adapter: String,
}

impl Hud {
    pub fn new(backend: &Backend, format: wgpu::TextureFormat) -> Self {
        let context = context();
        Self {
            context,
            renderer: egui_wgpu::Renderer::new(
                &backend.device,
                format,
                egui_wgpu::RendererOptions::default(),
            ),
            adapter: backend.adapter_name.clone(),
        }
    }

    pub fn run(
        &self,
        input: egui::RawInput,
        controls: &mut Controls,
        stats: &Stats,
    ) -> egui::FullOutput {
        widgets(&self.context, input, controls, stats, &self.adapter)
    }

    /// Encode after terrain, on the same target/encoder. Auxiliary command
    /// buffers from egui callbacks (if introduced) must precede this encoder.
    pub fn encode(
        &mut self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        mut output: egui::FullOutput,
        size: [u32; 2],
    ) -> Vec<wgpu::CommandBuffer> {
        for (id, deltas) in &output.textures_delta.set {
            for delta in deltas {
                self.renderer
                    .update_texture(&backend.device, &backend.queue, *id, delta);
            }
        }
        let jobs = self
            .context
            .tessellate(output.shapes, output.pixels_per_point);
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: size,
            pixels_per_point: output.pixels_per_point,
        };
        let commands =
            self.renderer
                .update_buffers(&backend.device, &backend.queue, encoder, &jobs, &screen);
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("terrain-egui-hud-load-existing-color"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                ..Default::default()
            });
            self.renderer
                .render(&mut pass.forget_lifetime(), &jobs, &screen);
        }
        // Meshes/bind groups recorded above retain the resources they use.
        for id in &output.textures_delta.free {
            self.renderer.free_texture(id);
        }
        // egui 0.37 asserts in debug builds if handled deltas are simply
        // dropped. Acknowledge them only after uploads and frees above.
        output.textures_delta.clear();
        commands
    }
}

fn context() -> egui::Context {
    let context = egui::Context::default();
    context.set_visuals(egui::Visuals::dark());
    // Reuse the shipped game's OFL-1.1 font instead of relying on egui's
    // (empty in 0.37) defaults or adding another set of font assets/licenses.
    let mut fonts = egui::FontDefinitions::empty();
    let name = "thessa-noto-sans".to_owned();
    fonts.font_data.insert(
        name.clone(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/NotoSans-Regular.ttf"
        ))),
    );
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts.families.insert(family, vec![name.clone()]);
    }
    context.set_fonts(fonts);
    context
}

fn widgets(
    context: &egui::Context,
    input: egui::RawInput,
    controls: &mut Controls,
    stats: &Stats,
    adapter: &str,
) -> egui::FullOutput {
    context.run_ui(input, |root| {
        let context = root.ctx();
        egui::Window::new("Thessa | terrain migration")
            .default_pos([16.0, 16.0])
            .default_width(300.0)
            .resizable(false)
            .show(context, |ui| {
                ui.label("Owned renderer · shared-device egui HUD");
                ui.small(adapter);
                ui.label(format!("Presentation time: {:.3} s", stats.sim_time_s));
                if stats.live_flight {
                    ui.colored_label(
                        if stats.flight_stopped {
                            egui::Color32::RED
                        } else {
                            egui::Color32::LIGHT_GREEN
                        },
                        if stats.flight_stopped {
                            "Local flight stopped (last valid presentation)"
                        } else if stats.chase {
                            "Local flight · COAST · Navball hold · chase"
                        } else {
                            "Local flight · COAST · Navball hold"
                        },
                    );
                }
                ui.separator();
                ui.label(format!("Resident terrain: {} tiles", stats.tiles));
                ui.label(format!(
                    "Cache H/M: {}/{} · pending: {}/{}",
                    stats.cache.0, stats.cache.1, stats.pending.0, stats.pending.1
                ));
                ui.label(format!(
                    "Publications: {} · regenerated: {}",
                    stats.publications, stats.regenerated
                ));
                ui.label(format!(
                    "Material upload: {:.2} MiB · refusals: {}",
                    stats.upload_bytes as f64 / (1024.0 * 1024.0),
                    stats.capacity_rejections
                ));
                ui.label(format!("Submitted frames: {}", stats.submitted_frames));
                ui.label(format!(
                    "Previous host frame: {:.2} ms (includes waits)",
                    stats.previous_cpu_ms
                ));
                ui.small("Host wall time, not GPU timing or game FPS.");
                ui.separator();
                if stats.live_flight {
                    ui.label(format!(
                        "Input: throttle {:.0}% · engine {} · {} · stick ({:+.1}, {:+.1}, {:+.1})",
                        stats.throttle * 100.0,
                        if stats.engine { "ON" } else { "OFF" },
                        stats
                            .control_mode
                            .map(|mode| mode.label())
                            .unwrap_or("—"),
                        stats.stick[0],
                        stats.stick[1],
                        stats.stick[2],
                    ));
                    ui.add(
                        egui::Slider::new(&mut controls.throttle, 0.0..=1.0).text("Throttle"),
                    );
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut controls.engine, "Engine");
                        ui.checkbox(&mut controls.precision, "Precision");
                        ui.checkbox(&mut controls.sas, "SAS");
                    });
                    ui.horizontal(|ui| {
                        if ui.button("Cycle control law").clicked() {
                            controls.cycle_mode = true;
                        }
                        ui.small("WASDQE stick · Shift/Ctrl slew · X/Z presets · Space engine · C law · T SAS");
                    });
                }
                if controls.vehicle_preview {
                    ui.checkbox(&mut controls.chase, "Chase camera (follow craft attitude)");
                } else {
                    ui.add(
                        egui::Slider::new(
                            &mut controls.angle,
                            -std::f64::consts::PI..=std::f64::consts::PI,
                        )
                        .clamping(egui::SliderClamping::Edits)
                        .text("Orbit / rad"),
                    );
                }
                ui.add(
                    egui::Slider::new(
                        &mut controls.distance,
                        if controls.vehicle_preview {
                            8.0..=180.0
                        } else {
                            1_000.0..=200_000.0
                        },
                    )
                    .logarithmic(true)
                    .text("Distance / m"),
                );
                if ui.button("Reset camera").clicked() {
                    controls.reset_camera = true;
                    if controls.vehicle_preview {
                        controls.chase = false;
                    } else {
                        controls.angle = 0.0;
                        controls.distance = 30_000.0;
                    }
                }
                ui.add(
                    egui::Slider::new(&mut controls.exposure_ev100, 1.0..=20.0)
                        .text("Exposure / EV100"),
                );
                ui.checkbox(&mut controls.vsync, "VSync");
                ui.small("Session-only graphics requests; graphics.toml is not modified.");
                ui.separator();
                ui.small("Arrow keys: orbit/zoom when UI does not capture input. Esc: exit.");
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "Migration viewer, not the complete game client.",
                );
            });
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_render_wgpu::terrain::OffscreenTarget;

    fn controls() -> Controls {
        Controls {
            vehicle_preview: false,
            chase: false,
            reset_camera: false,
            angle: 8.0,
            distance: 30_000.0,
            throttle: 0.0,
            engine: false,
            precision: false,
            sas: true,
            cycle_mode: false,
            exposure_ev100: 13.0,
            vsync: false,
        }
    }

    #[test]
    fn vehicle_hud_reset_restores_shared_camera_and_preserves_survey() {
        let context = context();
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 720.0),
            )),
            ..Default::default()
        };
        // Vehicle preview: the orbit slider is hidden (the shared quaternion
        // orbit has no single-angle widget) and reset restores the owned
        // pilot camera instead of touching survey state.
        let mut controls = controls();
        controls.vehicle_preview = true;
        controls.distance = 32.0;
        widgets(&context, input, &mut controls, &Stats::default(), "test")
            .drop_without_applying_deltas();
        assert_eq!(controls.distance, 32.0);
        assert!(!controls.reset_camera);
    }

    fn input(width: u32, height: u32, scale: f32) -> egui::RawInput {
        let mut input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(width as f32 / scale, height as f32 / scale),
            )),
            ..Default::default()
        };
        input
            .viewports
            .get_mut(&egui::ViewportId::ROOT)
            .unwrap()
            .native_pixels_per_point = Some(scale);
        input
    }

    #[test]
    fn passive_hud_preserves_camera_and_only_captures_pointer_over_widgets() {
        let context = context();
        let before = controls();
        let mut controls = before;
        for _ in 0..3 {
            widgets(
                &context,
                input(1280, 720, 1.0),
                &mut controls,
                &Stats::default(),
                "test GPU",
            )
            .drop_without_applying_deltas();
        }
        // Arrow-key orbit is unbounded. Merely showing a slider must not clamp
        // it, round it, reset the camera or edit graphics requests.
        assert_eq!(controls, before);
        context.fonts_mut(|fonts| {
            for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                assert!(fonts.has_glyphs(
                    &egui::FontId::new(14.0, family),
                    "Thessa 0123456789 · Орбита"
                ));
            }
        });
        let mut hover = input(1280, 720, 1.0);
        hover
            .events
            .push(egui::Event::PointerMoved(egui::pos2(50.0, 80.0)));
        widgets(
            &context,
            hover,
            &mut controls,
            &Stats::default(),
            "test GPU",
        )
        .drop_without_applying_deltas();
        assert!(context.egui_wants_pointer_input());
        let mut outside = input(1280, 720, 1.0);
        outside
            .events
            .push(egui::Event::PointerMoved(egui::pos2(1200.0, 680.0)));
        widgets(
            &context,
            outside,
            &mut controls,
            &Stats::default(),
            "test GPU",
        )
        .drop_without_applying_deltas();
        assert!(!context.egui_wants_pointer_input());
        assert!(!context.egui_wants_keyboard_input());
        assert_eq!(controls, before);
    }

    #[test]
    #[ignore = "requires a wgpu adapter; run on development hardware"]
    fn shared_device_hud_loads_existing_color_across_resize_dpi_and_recreation() {
        let backend = Backend::bring_up().expect("hardware HUD regression needs an adapter");
        // Recreate the complete egui context/renderer as window suspension does.
        for _ in 0..2 {
            let mut hud = Hud::new(&backend, OffscreenTarget::FORMAT);
            for (width, height, scale) in [(800, 600, 1.0), (1280, 900, 1.5), (960, 720, 1.0)] {
                let target = OffscreenTarget::new(&backend, width, height).unwrap();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                {
                    let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &target.color,
                            resolve_target: None,
                            depth_slice: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color {
                                    r: 0.04,
                                    g: 0.1,
                                    b: 0.2,
                                    a: 1.0,
                                }),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        ..Default::default()
                    });
                }
                backend.queue.submit([encoder.finish()]);
                let before = target.read_rgba8(&backend).unwrap();
                let mut controls = controls();
                for _ in 0..3 {
                    let output = hud.run(
                        input(width, height, scale),
                        &mut controls,
                        &Stats::default(),
                    );
                    assert_eq!(output.pixels_per_point, scale);
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    let mut commands = hud.encode(
                        &backend,
                        &mut encoder,
                        &target.color,
                        output,
                        [width, height],
                    );
                    commands.push(encoder.finish());
                    backend.queue.submit(commands);
                }
                let after = target.read_rgba8(&backend).unwrap();
                assert!(
                    before
                        .as_chunks::<4>()
                        .0
                        .iter()
                        .zip(after.as_chunks::<4>().0.iter())
                        .filter(|(a, b)| a != b)
                        .count()
                        > 500
                );
                // An accidental Clear/DontCare, incorrect target or fullscreen
                // blit would destroy the terrain outside the overlay.
                for y in 0..height {
                    let offset = ((y * width + width - 1) * 4) as usize;
                    assert_eq!(&before[offset..offset + 4], &after[offset..offset + 4]);
                }
            }
        }
    }
}
