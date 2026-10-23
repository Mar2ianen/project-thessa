//! Toolkit-owned native window. Surface loss/resize never rebuilds terrain.
use crate::scene::Scene;
use std::{collections::HashSet, error::Error, sync::Arc, time::Instant};
use thessa_render_wgpu::{Backend, terrain::TerrainRenderer};
use winit::{
    application::ApplicationHandler,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

struct State {
    window: Arc<Window>,
    instance: wgpu::Instance,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    backend: Backend,
    renderer: TerrainRenderer,
    sky_layers: crate::sky_layers::SkyLayers,
    display: thessa_render_wgpu::display::DisplayTransform,
    motion: Option<thessa_render_wgpu::motion::SceneMotion>,
    temporal: Option<thessa_render_wgpu::scene_temporal::SceneTemporal>,
    mesh_renderer: Option<crate::scene::SceneMeshes>,
    depth: wgpu::TextureView,
    depth_texture: wgpu::Texture,
    drawable: bool,
    exposure: f32,
    perf: crate::perf::WindowPerf,
    #[cfg(feature = "hud")]
    hud: Option<WindowHud>,
    submitted_frames: u64,
    previous_cpu_ms: f64,
    last_flight_wall_s: Option<f64>,
}

#[cfg(feature = "hud")]
struct WindowHud {
    gpu: crate::hud::Hud,
    input: egui_winit::State,
}

impl State {
    fn new(
        event_loop: &ActiveEventLoop,
        scene: &mut Scene,
        hud_enabled: bool,
        width: u32,
        height: u32,
    ) -> Result<Self, Box<dyn Error>> {
        #[cfg(not(feature = "hud"))]
        let _ = hud_enabled;
        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("Thessa — owned terrain migration viewer")
                    .with_decorations(true)
                    .with_inner_size(winit::dpi::PhysicalSize::new(width, height)),
            )?,
        );
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_with_display_handle(
            Box::new(event_loop.owned_display_handle()),
        ));
        let surface = instance.create_surface(window.clone())?;
        // Graphics-aware bring-up: the RT feature is negotiated here, from the
        // requested settings, instead of appearing available from the adapter
        // probe alone.
        let backend = Backend::for_surface_graphics(instance.clone(), &surface, &scene.requested)?;
        let size = window.inner_size();
        let mut config = surface
            .get_default_config(&backend.adapter, size.width.max(1), size.height.max(1))
            .ok_or("surface unsupported")?;
        config.format = surface
            .get_capabilities(&backend.adapter)
            .formats
            .into_iter()
            .find(|f| f.is_srgb())
            .ok_or("no sRGB surface format")?;
        let resolved = backend.resolve_graphics(&scene.requested);
        config.present_mode = if resolved.vsync {
            wgpu::PresentMode::AutoVsync
        } else {
            wgpu::PresentMode::AutoNoVsync
        };
        let exposure = crate::manual_exposure(resolved.exposure_ev100);
        if scene.requested.renderer.hdr && !resolved.hdr {
            eprintln!("native HDR target unsupported: using direct LDR fallback");
        }
        surface.configure(&backend.device, &config);
        let display = thessa_render_wgpu::display::DisplayTransform::new(
            &backend,
            &resolved,
            config.format,
            config.width,
            config.height,
        )?;
        let mut renderer = scene.renderer(&backend, display.scene_format())?;
        let sky_layers = scene.sky_layers(&backend, display.scene_format(), &mut renderer);
        let mesh_renderer =
            scene.mesh_renderer_with_shadows(&backend, display.scene_format(), &renderer)?;
        eprintln!(
            "native display: hdr={} target_bytes={} manual exposure; auto exposure and bloom not applied",
            display.enabled(),
            display.target_bytes()
        );
        let depth_texture = depth(&backend, config.width, config.height);
        let depth = depth_texture.create_view(&Default::default());
        let motion = thessa_render_wgpu::motion::SceneMotion::new(
            &backend,
            &resolved,
            [config.width, config.height],
        )?;
        eprintln!(
            "native motion diagnostics: enabled={} target_bytes={} jitter={}; opaque-only",
            motion.is_some(),
            motion.as_ref().map_or(0, |m| m.target_bytes()),
            motion.as_ref().is_some_and(|m| m.jitter_enabled()),
        );
        let temporal = thessa_render_wgpu::scene_temporal::SceneTemporal::new(
            &backend,
            &resolved,
            [config.width, config.height],
        )?;
        if resolved.temporal_jitter && !resolved.temporal_taa {
            eprintln!(
                "primary jitter diagnostics are enabled without a temporal resolve; visible subpixel trembling is expected, not antialiasing"
            );
        }
        #[cfg(feature = "hud")]
        let hud = hud_enabled.then(|| {
            let gpu = crate::hud::Hud::new(&backend, config.format);
            let input = egui_winit::State::new(
                gpu.context.clone(),
                egui::ViewportId::ROOT,
                window.as_ref(),
                Some(window.scale_factor() as f32),
                window.theme(),
                Some(backend.device.limits().max_texture_dimension_2d as usize),
            );
            WindowHud { gpu, input }
        });
        Ok(Self {
            window,
            instance,
            surface,
            config,
            backend,
            renderer,
            sky_layers,
            display,
            motion,
            temporal,
            mesh_renderer,
            depth,
            depth_texture,
            drawable: size.width > 0 && size.height > 0,
            exposure,
            perf: crate::perf::WindowPerf::new(),
            #[cfg(feature = "hud")]
            hud,
            submitted_frames: 0,
            previous_cpu_ms: 0.0,
            last_flight_wall_s: None,
        })
    }

    /// Stop any active window capture and write it with this window's
    /// platform/display metadata. Shared with the Shift+F4 key path.
    fn stop_window_capture(&mut self, scene: &Scene) -> String {
        let mut metadata = crate::perf::base_capture_metadata(
            &self.backend,
            scene,
            &self.display,
            &self.sky_layers,
            self.motion.as_ref(),
            self.temporal.as_ref(),
        );
        metadata.display.vsync = scene.requested.renderer.vsync;
        // Empty base preserves the legacy relative "stem.json" cwd contract.
        self.perf.stop_capture(metadata, std::path::Path::new(""))
    }

    fn resize(&mut self, width: u32, height: u32) {
        self.drawable = width > 0 && height > 0;
        if !self.drawable {
            return;
        }
        if let Err(error) = self.display.resize(&self.backend, width, height) {
            self.drawable = false;
            eprintln!("display resize rejected: {error}");
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.backend.device, &self.config);
        self.depth_texture = depth(&self.backend, width, height);
        self.depth = self.depth_texture.create_view(&Default::default());
        if let Some(temporal) = &mut self.temporal
            && let Err(error) = temporal.resize(&self.backend, [width, height])
        {
            self.drawable = false;
            eprintln!("temporal resize rejected: {error}");
            return;
        }
        if let Some(motion) = &mut self.motion
            && let Err(error) = motion.resize(&self.backend, [width, height])
        {
            self.drawable = false;
            eprintln!("motion resize rejected: {error}");
        }
    }

    fn draw(
        &mut self,
        scene: &mut Scene,
        now_s: f64,
        throttle_up: bool,
        throttle_down: bool,
    ) -> Result<bool, Box<dyn Error>> {
        let cpu_started = Instant::now();
        if !self.drawable {
            return Ok(false);
        }
        #[allow(unused_mut)]
        let (frame, mut reconfigure) = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame) => (frame, false),
            wgpu::CurrentSurfaceTexture::Suboptimal(frame) => (frame, true),
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(false);
            }
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.resize(self.config.width, self.config.height);
                return Ok(false);
            }
            wgpu::CurrentSurfaceTexture::Lost => {
                self.surface = self.instance.create_surface(self.window.clone())?;
                self.resize(self.config.width, self.config.height);
                return Ok(false);
            }
            wgpu::CurrentSurfaceTexture::Validation => {
                return Err("surface validation failure".into());
            }
        };
        let view_name: &'static str = if scene.map_active() {
            "MAP"
        } else if scene.vehicle_preview() {
            "VEHICLE"
        } else {
            "SURFACE"
        };
        self.perf.begin_frame(
            view_name,
            scene.requested.renderer.vsync,
            self.config.width,
            self.config.height,
        );
        if scene.docking.is_some() {
            let elapsed = self
                .last_flight_wall_s
                .replace(now_s)
                .map_or(0.0, |last| (now_s - last).clamp(0.0, 0.05));
            scene.advance_docking(elapsed)?;
        } else if scene.live_flight() {
            let elapsed = self
                .last_flight_wall_s
                .replace(now_s)
                .map_or(0.0, |last| (now_s - last).clamp(0.0, 0.1));
            let sim_started = Instant::now();
            scene.push_live_input(elapsed, throttle_up, throttle_down);
            scene.advance_flight(elapsed, Some(std::time::Duration::from_millis(8)))?;
            self.perf.record_scope(
                thessa_perf::scopes::SIM_TOTAL,
                sim_started.elapsed().as_secs_f64(),
            );
        }
        #[cfg(feature = "hud")]
        let ui_started = Instant::now();
        #[cfg(feature = "hud")]
        let hud_output = if let Some(hud) = &mut self.hud {
            egui_winit::update_viewport_info(
                hud.input
                    .egui_input_mut()
                    .viewports
                    .entry(egui::ViewportId::ROOT)
                    .or_default(),
                &hud.gpu.context,
                &self.window,
                self.submitted_frames == 0,
            );
            let mut controls = crate::hud::Controls::from_scene(scene);
            let before = controls;
            let mut stats =
                crate::hud::Stats::from_scene(scene, self.submitted_frames, self.previous_cpu_ms);
            stats.perf = self.perf.overlay_text(flight_stop_status(scene));
            let mut output = hud.gpu.run(
                hud.input.take_egui_input(&self.window),
                &mut controls,
                &stats,
            );
            controls.apply(scene);
            hud.input
                .handle_platform_output(&self.window, std::mem::take(&mut output.platform_output));
            if before.exposure_ev100 != controls.exposure_ev100 || before.vsync != controls.vsync {
                let resolved = self.backend.resolve_graphics(&scene.requested);
                self.exposure = crate::manual_exposure(resolved.exposure_ev100);
                let mode = if resolved.vsync {
                    wgpu::PresentMode::AutoVsync
                } else {
                    wgpu::PresentMode::AutoNoVsync
                };
                if self.config.present_mode != mode {
                    self.config.present_mode = mode;
                    // Configure only after presenting the acquired frame.
                    reconfigure = true;
                }
            }
            Some(output)
        } else {
            None
        };
        let render_started = Instant::now();
        scene.tick(
            &self.backend,
            &mut self.renderer,
            self.config.width,
            self.config.height,
            self.exposure,
            now_s,
        )?;
        let view = scene.view(self.config.width, self.config.height, self.exposure);
        let jitter = self
            .motion
            .as_ref()
            .map_or(thessa_render_core::temporal::JitterOffset::ZERO, |m| {
                m.jitter()
            });
        self.renderer.update_view_jittered(
            &self.backend,
            &view,
            [self.config.width, self.config.height],
            jitter,
        )?;
        self.renderer.update_shadows(
            &self.backend,
            scene
                .view(self.config.width, self.config.height, self.exposure)
                .world_position_m,
            &scene.lighting(),
        )?;
        self.renderer
            .update_ocean(&self.backend, scene.sample.time.0)?;
        scene.update_plume(
            &self.backend,
            &mut self.renderer,
            self.config.width,
            self.config.height,
            self.exposure,
        )?;
        scene.update_sky_layers(
            &self.backend,
            &self.sky_layers,
            self.config.width,
            self.config.height,
            self.exposure,
        )?;
        if let Some(mesh) = &mut self.mesh_renderer {
            scene.update_mesh(
                &self.backend,
                mesh,
                self.config.width,
                self.config.height,
                self.exposure,
            )?;
            mesh.update_primary_jitter(
                &self.backend,
                &view,
                [self.config.width, self.config.height],
                jitter,
            )?;
            mesh.sync_rt_terrain(&self.backend, &self.renderer)?;
        }
        let color = frame.texture.create_view(&Default::default());
        let mut encoder = self
            .backend
            .device
            .create_command_encoder(&Default::default());
        self.renderer.encode_surface(&mut encoder);
        self.renderer.encode_shadows(
            &mut encoder,
            self.mesh_renderer
                .as_ref()
                .and_then(crate::scene::SceneMeshes::opaque),
        );
        if let Some(mesh) = &mut self.mesh_renderer {
            mesh.encode_rt(&mut encoder);
        }
        self.renderer.encode_opaque_scene_ops(
            &mut encoder,
            self.display.scene_color(&color),
            &self.depth,
            self.mesh_renderer
                .as_ref()
                .and_then(crate::scene::SceneMeshes::opaque),
            self.sky_layers.enabled() || self.motion.is_some(),
        );
        if let Some(motion) = &self.motion {
            motion.encode(
                &self.backend,
                &mut encoder,
                &self.depth,
                self.display.scene_color(&color),
                &view,
            )?;
        }
        if let (Some(temporal), Some(motion), Some(color)) = (
            &mut self.temporal,
            &self.motion,
            self.display.scene_texture(),
        ) {
            temporal.encode(
                &self.backend,
                &mut encoder,
                color,
                &self.depth_texture,
                motion,
                &view,
                scene.temporal_epoch(&self.renderer, self.mesh_renderer.as_ref()),
            )?;
        }
        self.renderer
            .encode_plume(&mut encoder, self.display.scene_color(&color), &self.depth);
        self.sky_layers.encode(
            &self.backend,
            &mut encoder,
            self.display.scene_color(&color),
            &self.depth,
        );
        self.display.encode(&mut encoder, &color);
        self.perf.record_scope(
            thessa_perf::scopes::RENDER_TOTAL,
            render_started.elapsed().as_secs_f64(),
        );
        #[cfg(feature = "hud")]
        let commands = if let (Some(hud), Some(output)) = (&mut self.hud, hud_output) {
            let commands = hud.gpu.encode(
                &self.backend,
                &mut encoder,
                &color,
                output,
                [self.config.width, self.config.height],
            );
            self.perf.record_scope(
                thessa_perf::scopes::RENDER_UI,
                ui_started.elapsed().as_secs_f64(),
            );
            commands
        } else {
            Vec::new()
        };
        #[cfg(not(feature = "hud"))]
        let commands = Vec::<wgpu::CommandBuffer>::new();
        let cpu_s = cpu_started.elapsed().as_secs_f64();
        self.window.pre_present_notify();
        self.backend.queue.submit(
            commands
                .into_iter()
                .chain(std::iter::once(encoder.finish())),
        );
        if let Some(motion) = &mut self.motion {
            motion.commit(view);
        }
        if let Some(temporal) = &mut self.temporal {
            temporal.commit();
        }
        self.backend.queue.present(frame);
        self.submitted_frames += 1;
        // Overlay wall includes present/vsync wait (same definition as the
        // legacy client's `time.delta` sample); CPU stops before submit.
        let wall_s = cpu_started.elapsed().as_secs_f64();
        let budget = scene.flight_budget(wall_s);
        let sim_time_s = scene.sample.time.0;
        let counters = crate::perf::world_counters(scene, self.mesh_renderer.as_ref());
        self.perf
            .end_frame(wall_s, cpu_s, sim_time_s, budget, counters);
        self.previous_cpu_ms = wall_s * 1000.0;
        if reconfigure {
            self.resize(self.config.width, self.config.height);
        }
        Ok(true)
    }
}

fn depth(backend: &Backend, width: u32, height: u32) -> wgpu::Texture {
    backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("native-terrain-depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    })
}

struct App {
    scene: Scene,
    state: Option<State>,
    error: Option<String>,
    frame_limit: Option<u32>,
    frames: u32,
    started: Option<Instant>,
    hud_enabled: bool,
    width: u32,
    height: u32,
    right_held: bool,
    middle_held: bool,
    right_consumed: bool,
    middle_consumed: bool,
    last_cursor: Option<(f64, f64)>,
    pointer_consumed: bool,
    held: HashSet<KeyCode>,
    shift_held: bool,
    audio_options: crate::audio::Options,
    audio: Option<crate::audio::NativeAudio>,
}

impl App {
    fn release_held_input(&mut self) {
        self.held.clear();
        self.scene.release_manual_input();
        self.scene.push_live_input(0.0, false, false);
        if self.scene.server_flight() {
            // Deliver neutral axes before a suspended window stops polling.
            if let Err(error) = self.scene.advance_flight(0.0, None) {
                eprintln!("{error}");
            }
        }
        self.right_held = false;
        self.middle_held = false;
        self.last_cursor = None;
        self.pointer_consumed = false;
        self.shift_held = false;
        if let Some(state) = &mut self.state {
            state.last_flight_wall_s = None;
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        match State::new(
            event_loop,
            &mut self.scene,
            self.hud_enabled,
            self.width,
            self.height,
        ) {
            Ok(state) => {
                self.state = Some(state);
                self.audio = match crate::audio::NativeAudio::open(self.audio_options) {
                    Ok(audio) => audio,
                    Err(error) => {
                        eprintln!("{error}");
                        None
                    }
                };
                // Selection history survives surface suspension in Scene; its
                // monotonic clock must survive too, or debounce can stall
                // demand until a reset clock catches the previous timestamp.
                self.started.get_or_insert_with(Instant::now);
            }
            Err(e) => {
                self.error = Some(e.to_string());
                event_loop.exit();
            }
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        self.release_held_input();
        self.audio = None;
        self.state = None;
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        // egui-winit's clipboard borrows the native display handle. run_app
        // consumes/destroys EventLoop, while App itself lives beyond it, so
        // teardown must happen here while the display is still valid.
        self.state = None;
        self.audio = None;
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if self
            .state
            .as_ref()
            .is_none_or(|state| state.window.id() != id)
        {
            return;
        }
        if matches!(event, WindowEvent::Focused(false)) {
            self.release_held_input();
        }
        let Some(state) = &mut self.state else {
            return;
        };
        if state.window.id() != id {
            return;
        }
        #[cfg(feature = "hud")]
        let consumed = state
            .hud
            .as_mut()
            .is_some_and(|hud| hud.input.on_window_event(&state.window, &event).consumed);
        #[cfg(not(feature = "hud"))]
        let consumed = false;
        #[cfg(feature = "hud")]
        if let WindowEvent::KeyboardInput { event, .. } = &event
            && event.state == ElementState::Pressed
            && let Some(action) = hud_action(event.physical_key, event.repeat, consumed)
            && let Some(hud) = &mut state.hud
        {
            let was_blocked = hud.gpu.view.blocks_scene_input();
            hud.gpu.view.apply(action);
            if was_blocked != hud.gpu.view.blocks_scene_input() {
                self.held.clear();
                self.last_cursor = None;
                self.right_held = false;
                self.middle_held = false;
                self.scene.release_manual_input();
            }
        }
        #[cfg(feature = "hud")]
        let consumed = consumed
            || state
                .hud
                .as_ref()
                .is_some_and(|hud| hud.gpu.view.blocks_scene_input());
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size.width, size.height),
            WindowEvent::Occluded(occluded) => {
                if occluded {
                    state.drawable = false;
                    if let Some(audio) = &mut self.audio {
                        audio.update(None);
                    }
                } else {
                    let size = state.window.inner_size();
                    state.resize(size.width, size.height);
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == ElementState::Pressed
                    && let PhysicalKey::Code(code) = event.physical_key
                    && matches!(code, KeyCode::ShiftLeft | KeyCode::ShiftRight)
                {
                    self.shift_held = true;
                }
                #[cfg(feature = "hud")]
                if !event.repeat
                    && event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyM)
                    && !consumed
                    && state.hud.is_some()
                {
                    self.scene.set_map_open(!self.scene.map_active());
                    self.held.clear();
                    self.last_cursor = None;
                    self.right_held = false;
                    self.middle_held = false;
                }
                if event.state == ElementState::Pressed
                    && !event.repeat
                    && !consumed
                    && let Some(action) = perf_key(event.physical_key, self.shift_held)
                {
                    match action {
                        PerfKey::Overlay => state.perf.toggle_overlay(),
                        PerfKey::Level => state.perf.cycle_level(),
                        PerfKey::Capture => {
                            let status = if state.perf.capturing() {
                                state.stop_window_capture(&self.scene)
                            } else {
                                state.perf.start_capture();
                                state.perf.status().to_owned()
                            };
                            eprintln!("{status}");
                        }
                    }
                }
                if let PhysicalKey::Code(code) = event.physical_key {
                    if event.state == ElementState::Pressed && !consumed && !self.scene.map_active()
                    {
                        self.held.insert(code);
                    } else {
                        self.held.remove(&code);
                    }
                }
                if event.state == ElementState::Pressed {
                    let vehicle = self.scene.vehicle_preview();
                    match camera_shortcut(event.physical_key, consumed || self.scene.map_active()) {
                        Some(Shortcut::Exit) => event_loop.exit(),
                        Some(Shortcut::Orbit(delta)) => {
                            if vehicle {
                                self.scene.camera.orbit_yaw(delta as f32);
                            } else {
                                self.scene.survey.orbit_yaw(delta as f32);
                            }
                        }
                        Some(Shortcut::Zoom(factor)) => {
                            if vehicle {
                                self.scene.camera.zoom_by_factor(factor as f32);
                            } else {
                                self.scene.survey.zoom_by_factor(factor as f32);
                            }
                        }
                        // Repeats keep orbiting/zooming while held but never
                        // re-fire toggles or resets.
                        Some(Shortcut::ToggleChase) => {
                            if vehicle && !event.repeat {
                                self.scene.camera.toggle_chase();
                            }
                        }
                        Some(Shortcut::ResetView) if !event.repeat => {
                            if vehicle {
                                self.scene.camera.reset();
                            } else {
                                self.scene.survey.reset();
                            }
                        }
                        Some(Shortcut::CycleSite) if !vehicle && !event.repeat => {
                            self.scene
                                .select_site((self.scene.site + 1) % self.scene.sites.len());
                        }
                        _ => {}
                    }
                    let clock_key = matches!(
                        event.physical_key,
                        PhysicalKey::Code(
                            KeyCode::F8
                                | KeyCode::BracketLeft
                                | KeyCode::BracketRight
                                | KeyCode::Backspace
                        )
                    );
                    let flight_consumed = consumed || (self.scene.map_active() && !clock_key);
                    apply_discrete_flight_input(
                        &mut self.scene,
                        event.physical_key,
                        event.repeat,
                        flight_consumed,
                    );
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let pressed = state == ElementState::Pressed;
                match button {
                    MouseButton::Right => {
                        self.right_held = pressed;
                        if pressed {
                            self.right_consumed = consumed;
                        }
                    }
                    MouseButton::Middle => {
                        self.middle_held = pressed;
                        if pressed {
                            self.middle_consumed = consumed;
                        }
                    }
                    _ => {}
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.pointer_consumed = consumed;
                let (x, y) = (position.x, position.y);
                if let Some((last_x, last_y)) = self.last_cursor {
                    let (dx, dy) = ((x - last_x) as f32, (y - last_y) as f32);
                    if drag_delta(
                        self.right_held,
                        self.right_consumed,
                        !self.scene.map_active(),
                        dx,
                        dy,
                    )
                    .is_some()
                    {
                        if self.scene.vehicle_preview() {
                            self.scene.camera.drag_orbit(dx, dy);
                        } else {
                            let look = self.held.contains(&KeyCode::ShiftLeft)
                                || self.held.contains(&KeyCode::ShiftRight);
                            self.scene.survey.drag(dx, dy, look);
                        }
                    }
                    if drag_delta(
                        self.middle_held,
                        self.middle_consumed,
                        self.scene.vehicle_preview() && !self.scene.map_active(),
                        dx,
                        dy,
                    )
                    .is_some()
                    {
                        self.scene.camera.pan_by(dx, dy);
                    }
                }
                self.last_cursor = Some((x, y));
            }
            WindowEvent::CursorLeft { .. } => {
                self.last_cursor = None;
            }
            WindowEvent::MouseWheel { delta, .. } => {
                if !consumed && !self.scene.map_active() {
                    if self.scene.vehicle_preview() {
                        self.scene.camera.zoom(wheel_lines(delta));
                    } else {
                        self.scene.survey.zoom(wheel_lines(delta));
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                let map_was_active = self.scene.map_active();
                #[cfg(feature = "hud")]
                let help_was_active = state
                    .hud
                    .as_ref()
                    .is_some_and(|hud| hud.gpu.view.blocks_scene_input());
                #[allow(unused_mut)]
                let mut keyboard_blocked = !state.window.has_focus();
                #[allow(unused_mut)]
                let mut pointer_blocked = keyboard_blocked || self.pointer_consumed;
                #[cfg(feature = "hud")]
                if let Some(hud) = &state.hud {
                    keyboard_blocked |= hud.gpu.context.egui_wants_keyboard_input();
                    keyboard_blocked |= hud.gpu.view.blocks_scene_input();
                    pointer_blocked |= hud.gpu.context.egui_wants_pointer_input();
                    pointer_blocked |= hud.gpu.view.blocks_scene_input();
                }
                let (throttle_up, throttle_down) = condition_flight_input(
                    &mut self.scene,
                    &self.held,
                    self.last_cursor.map(|(x, y)| [x as f32, y as f32]),
                    [state.config.width as f32, state.config.height as f32],
                    keyboard_blocked,
                    pointer_blocked,
                    self.right_held || self.middle_held,
                );
                match state.draw(
                    &mut self.scene,
                    self.started.unwrap().elapsed().as_secs_f64(),
                    throttle_up,
                    throttle_down,
                ) {
                    Ok(true) => {
                        #[cfg(feature = "hud")]
                        let help_changed = help_was_active
                            != state
                                .hud
                                .as_ref()
                                .is_some_and(|hud| hud.gpu.view.blocks_scene_input());
                        #[cfg(not(feature = "hud"))]
                        let help_changed = false;
                        if map_was_active != self.scene.map_active() || help_changed {
                            // Window close buttons have the same release contract
                            // as M/F1, including any cursor sampled over the UI.
                            self.held.clear();
                            self.last_cursor = None;
                            self.right_held = false;
                            self.middle_held = false;
                            self.scene.release_manual_input();
                        }
                        let docking_impact =
                            self.scene.docking.as_mut().and_then(|d| d.take_impact());
                        if let Some(audio) = &mut self.audio {
                            let frame = self.scene.audio_frame().or_else(|| {
                                (self.audio_options.demo && !self.scene.live_flight()).then(|| {
                                    crate::audio::Frame::demo_at(thessa_sim_core::SimTime(
                                        self.started.unwrap().elapsed().as_secs_f64(),
                                    ))
                                })
                            });
                            audio.update(frame);
                            if let Some(speed) = docking_impact {
                                audio.docking_impact(speed);
                            }
                        }
                        self.frames += 1;
                        if self.frame_limit.is_some_and(|n| self.frames >= n) {
                            let seconds = self.started.unwrap().elapsed().as_secs_f64();
                            println!(
                                "window_presented_frames={} elapsed_s={seconds:.3} presented_fps={:.1} hud={} (presentation requests, not display feedback; vsync/compositor included; not GPU-only or game FPS)",
                                self.frames,
                                self.frames as f64 / seconds,
                                self.hud_enabled,
                            );
                            println!(
                                "canonical_tiles={} geometry_publications={} regenerated_patches={} cache_entries={:?} pending_jobs={:?} capacity_rejections={}",
                                self.scene.stream.visible().len(),
                                self.scene.publications,
                                self.scene.regenerated,
                                self.scene.stream.cache_entries(),
                                self.scene.stream.pending_jobs(),
                                self.scene.capacity_rejections
                            );
                            if let Some(status) = self.scene.flight_telemetry() {
                                println!("{status}");
                            }
                            println!(
                                "shadow_atlas_bytes={} shadow_active_layers={} shadow_rendered_passes={} terrain_shadow_draws={:?}",
                                state.renderer.shadows().map_or(0, |s| s.storage_bytes()),
                                state.renderer.shadows().map_or(0, |s| s.active_passes()),
                                state.renderer.last_shadow_passes(),
                                state.renderer.last_shadow_draws()
                            );
                            event_loop.exit();
                        }
                    }
                    Ok(false) => {
                        if let Some(audio) = &mut self.audio {
                            audio.update(None);
                        }
                    }
                    Err(e) => {
                        self.error = Some(e.to_string());
                        event_loop.exit();
                    }
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(state) = &self.state
            && state.drawable
        {
            state.window.request_redraw();
        }
    }
}

/// Installed-subsystem key mapping shared with the old pilot
/// (`R` RCS, `Y` wheels, `G` gear, `P` parachutes). Pure mapping; the scene
/// owns application and gating.
fn part_toggle_for_key(key: PhysicalKey) -> Option<crate::scene::PartToggle> {
    use crate::scene::PartToggle;
    match key {
        PhysicalKey::Code(KeyCode::KeyR) => Some(PartToggle::Rcs),
        PhysicalKey::Code(KeyCode::KeyY) => Some(PartToggle::ReactionWheels),
        PhysicalKey::Code(KeyCode::KeyG) => Some(PartToggle::Gear),
        PhysicalKey::Code(KeyCode::KeyP) => Some(PartToggle::Parachutes),
        _ => None,
    }
}

#[cfg(feature = "hud")]
fn hud_action(key: PhysicalKey, repeat: bool, consumed: bool) -> Option<crate::hud::Action> {
    if repeat || consumed {
        return None;
    }
    match key {
        PhysicalKey::Code(KeyCode::F1) => Some(crate::hud::Action::Help),
        PhysicalKey::Code(KeyCode::F2) => Some(crate::hud::Action::Visibility),
        PhysicalKey::Code(KeyCode::F3) => Some(crate::hud::Action::Telemetry),
        _ => None,
    }
}

/// `F4` toggles the overlay, `Shift+F4` starts/stops a short capture, `F5`
/// cycles the profiling level (shared bindings with the legacy pilot).
/// Pure mapping; the window state applies it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PerfKey {
    Overlay,
    Capture,
    Level,
}

fn perf_key(key: PhysicalKey, shift: bool) -> Option<PerfKey> {
    match key {
        PhysicalKey::Code(KeyCode::F4) => Some(if shift {
            PerfKey::Capture
        } else {
            PerfKey::Overlay
        }),
        PhysicalKey::Code(KeyCode::F5) => Some(PerfKey::Level),
        _ => None,
    }
}

/// One-word overlay status line, mirroring the legacy overlay's stop/pause
/// flag. Live flight is assumed; stopped sessions keep the last presentation.
#[cfg(feature = "hud")]
fn flight_stop_status(scene: &Scene) -> Option<&'static str> {
    if scene.flight_stopped() {
        Some("stopped")
    } else if scene.flight_clock().is_some_and(|clock| {
        clock.authority_paused.is_some_and(|paused| paused) || clock.requested_paused
    }) {
        Some("paused")
    } else {
        None
    }
}

/// Discrete flight inputs sharing factors with the old pilot. Repeats never
/// re-fire presets or toggles; slew and stick read the held set every frame
/// instead. Free function so the event arm can borrow the scene while the
/// window state is already borrowed.
fn apply_discrete_flight_input(scene: &mut Scene, key: PhysicalKey, repeat: bool, consumed: bool) {
    if consumed || repeat || !scene.live_flight() {
        return;
    }
    if let Some(toggle) = part_toggle_for_key(key) {
        scene.apply_part_toggle(toggle);
        return;
    }
    match key {
        PhysicalKey::Code(KeyCode::F7) => {
            if let Err(error) = scene.set_flyby(!scene.flyby.active) {
                eprintln!("{error}");
            }
        }
        PhysicalKey::Code(KeyCode::F8) => {
            if let Some(clock) = scene.flight_clock() {
                scene.request_pause(!clock.requested_paused);
            }
        }
        PhysicalKey::Code(KeyCode::BracketLeft | KeyCode::BracketRight) => {
            if let Some(clock) = scene.flight_clock() {
                let factor = if key == PhysicalKey::Code(KeyCode::BracketRight) {
                    (clock.requested_warp.max(0.125) * 2.0).min(thessa_flight_net::MAX_TIME_WARP)
                } else {
                    (clock.requested_warp * 0.5).max(0.125)
                };
                if let Err(error) = scene.request_warp(factor) {
                    eprintln!("{error}");
                }
            }
        }
        PhysicalKey::Code(KeyCode::Backspace) => {
            if let Err(error) = scene.reset_flight() {
                eprintln!("{error}");
            }
        }
        PhysicalKey::Code(KeyCode::KeyX) => {
            scene.cancel_flyby();
            scene.input.cut();
        }
        PhysicalKey::Code(KeyCode::KeyZ) => {
            scene.cancel_flyby();
            scene.input.full();
        }
        PhysicalKey::Code(KeyCode::Space) => {
            scene.cancel_flyby();
            scene.input.toggle_engine();
        }
        PhysicalKey::Code(KeyCode::KeyC) => scene.cycle_control_mode(),
        PhysicalKey::Code(KeyCode::KeyT) => scene.toggle_sas(),
        PhysicalKey::Code(KeyCode::CapsLock) => scene.input.toggle_precision(),
        _ => {}
    }
}

/// Held stick/throttle keys sampled every frame. Built fresh from the held
/// set so a lost release cannot wedge an axis on.
/// One ownership gate for held input, independent of event cadence. Camera/UI
/// gestures can never also steer Mouse Aim; a missing pointer is neutral.
fn condition_flight_input(
    scene: &mut Scene,
    held: &HashSet<KeyCode>,
    cursor: Option<[f32; 2]>,
    extent: [f32; 2],
    keyboard_blocked: bool,
    pointer_blocked: bool,
    camera_gesture: bool,
) -> (bool, bool) {
    let keyboard_blocked = keyboard_blocked || scene.map_active();
    let pointer_blocked = pointer_blocked || scene.map_active();
    let (keys, up, down) = held_flight_keys(held);
    scene.input.keys = if keyboard_blocked {
        Default::default()
    } else {
        keys
    };
    scene.input.sas_inverted = !keyboard_blocked && held.contains(&KeyCode::KeyF);
    scene.input.mouse_direction = thessa_flight_authority::input::mouse_aim_direction(
        cursor.filter(|_| !keyboard_blocked && !pointer_blocked && !camera_gesture),
        extent,
    );
    (up && !keyboard_blocked, down && !keyboard_blocked)
}

fn held_flight_keys(
    held: &HashSet<KeyCode>,
) -> (thessa_flight_authority::input::StickKeys, bool, bool) {
    use thessa_flight_authority::input::StickKeys;
    let held = |code| held.contains(&code);
    (
        StickKeys {
            nose_up: held(KeyCode::KeyS),
            nose_down: held(KeyCode::KeyW),
            yaw_right: held(KeyCode::KeyD),
            yaw_left: held(KeyCode::KeyA),
            roll_right: held(KeyCode::KeyE),
            roll_left: held(KeyCode::KeyQ),
        },
        held(KeyCode::ShiftLeft) || held(KeyCode::ShiftRight),
        held(KeyCode::ControlLeft) || held(KeyCode::ControlRight),
    )
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Shortcut {
    Exit,
    /// Shared-controller yaw step in radians for the active camera.
    Orbit(f64),
    /// Multiplicative zoom factor for the active camera.
    Zoom(f64),
    /// Vehicle-only chase toggle (matches the old pilot `V` key).
    ToggleChase,
    /// Restore the active camera default (matches `Backquote` in the pilot).
    ResetView,
    CycleSite,
}

fn camera_shortcut(key: PhysicalKey, consumed: bool) -> Option<Shortcut> {
    if consumed {
        return None;
    }
    match key {
        PhysicalKey::Code(KeyCode::Escape) => Some(Shortcut::Exit),
        PhysicalKey::Code(KeyCode::ArrowLeft) => Some(Shortcut::Orbit(-0.05)),
        PhysicalKey::Code(KeyCode::ArrowRight) => Some(Shortcut::Orbit(0.05)),
        PhysicalKey::Code(KeyCode::ArrowUp) => Some(Shortcut::Zoom(1.0 / 1.1)),
        PhysicalKey::Code(KeyCode::ArrowDown) => Some(Shortcut::Zoom(1.1)),
        PhysicalKey::Code(KeyCode::KeyV) => Some(Shortcut::ToggleChase),
        PhysicalKey::Code(KeyCode::Backquote) => Some(Shortcut::ResetView),
        PhysicalKey::Code(KeyCode::F6) => Some(Shortcut::CycleSite),
        _ => None,
    }
}

/// Pure drag gate: a drag orbits/pans only while its button is held from a
/// press outside the UI and the requested gesture is enabled.
fn drag_delta(
    held: bool,
    press_consumed: bool,
    enabled: bool,
    dx: f32,
    dy: f32,
) -> Option<(f32, f32)> {
    if held && !press_consumed && enabled && (dx != 0.0 || dy != 0.0) {
        Some((dx, dy))
    } else {
        None
    }
}

fn wheel_lines(delta: MouseScrollDelta) -> f32 {
    match delta {
        MouseScrollDelta::LineDelta(_, lines) => lines,
        MouseScrollDelta::PixelDelta(position) => position.y as f32 / 16.0,
    }
}

pub fn run(
    scene: Scene,
    frame_limit: Option<u32>,
    hud_enabled: bool,
    width: u32,
    height: u32,
    audio_options: crate::audio::Options,
) -> Result<(), Box<dyn Error>> {
    let event_loop = EventLoop::new()?;
    let mut app = App {
        scene,
        state: None,
        error: None,
        frame_limit,
        frames: 0,
        started: None,
        hud_enabled,
        width,
        height,
        right_held: false,
        middle_held: false,
        right_consumed: false,
        middle_consumed: false,
        last_cursor: None,
        pointer_consumed: false,
        held: HashSet::new(),
        shift_held: false,
        audio_options,
        audio: None,
    };
    event_loop.run_app(&mut app)?;
    if let Some(error) = app.error {
        return Err(error.into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn flyby_key_is_an_edge_and_discrete_manual_takeover_is_not_overwritten() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        let key = PhysicalKey::Code(KeyCode::F7);
        apply_discrete_flight_input(&mut scene, key, true, false);
        assert!(!scene.flyby.active);
        apply_discrete_flight_input(&mut scene, key, false, true);
        assert!(!scene.flyby.active);
        apply_discrete_flight_input(&mut scene, key, false, false);
        assert!(scene.flyby.active);
        apply_discrete_flight_input(&mut scene, key, true, false);
        assert!(scene.flyby.active);
        apply_discrete_flight_input(&mut scene, PhysicalKey::Code(KeyCode::KeyX), false, false);
        scene.push_live_input(0.1, false, false);
        assert!(!scene.flyby.active && scene.input.throttle == 0.0);
        apply_discrete_flight_input(&mut scene, key, false, false);
        assert!(scene.flyby.active);
        apply_discrete_flight_input(&mut scene, PhysicalKey::Code(KeyCode::Space), false, false);
        scene.push_live_input(0.1, false, false);
        assert!(!scene.flyby.active && !scene.input.engine_active);
    }
    #[test]
    #[cfg(feature = "hud")]
    fn help_hud_telemetry_keys_are_edges_and_keep_existing_camera_and_flight_bindings() {
        for (key, expected) in [
            (KeyCode::F1, crate::hud::Action::Help),
            (KeyCode::F2, crate::hud::Action::Visibility),
            (KeyCode::F3, crate::hud::Action::Telemetry),
        ] {
            let key = PhysicalKey::Code(key);
            assert_eq!(hud_action(key, false, false), Some(expected));
            assert_eq!(hud_action(key, true, false), None);
            assert_eq!(hud_action(key, false, true), None);
            assert_eq!(camera_shortcut(key, false), None);
        }
        assert_eq!(
            hud_action(PhysicalKey::Code(KeyCode::F8), false, false),
            None
        );
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene.input.set_throttle(0.4);
        scene.input.engine_active = true;
        scene.input.sas_inverted = true;
        scene.input.keys.nose_up = true;
        scene.release_manual_input();
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
        assert_eq!(scene.input.throttle, 0.4);
        assert!(scene.input.engine_active && scene.input.effective_sas());
        let held = [KeyCode::KeyW, KeyCode::KeyF, KeyCode::ShiftLeft]
            .into_iter()
            .collect();
        let mut view = crate::hud::View::default();
        view.apply(crate::hud::Action::Help);
        let gate = view.blocks_scene_input();
        let slew = condition_flight_input(
            &mut scene,
            &held,
            Some([1000.0, 500.0]),
            [1000.0, 1000.0],
            gate,
            gate,
            false,
        );
        scene.push_live_input(0.1, slew.0, slew.1);
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
        assert_eq!(slew, (false, false));
        assert_eq!(scene.input.throttle, 0.4);
        assert!(scene.input.engine_active && scene.input.effective_sas());
    }
    #[test]
    #[cfg(feature = "hud")]
    fn map_owns_input_without_pausing_relaunching_or_moving_the_flight_camera() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene.control_mode = thessa_flight_authority::ControlMode::MouseAim;
        scene.input.set_throttle(0.4);
        scene.input.mouse_direction = glam::Vec3::X;
        scene.input.sas_inverted = true;
        let camera = scene.camera;
        let time = scene.sample.time;
        scene.set_map_open(true);
        let held = [KeyCode::KeyW, KeyCode::KeyF, KeyCode::ShiftLeft]
            .into_iter()
            .collect();
        let slew = condition_flight_input(
            &mut scene,
            &held,
            Some([1000.0, 500.0]),
            [1000.0, 1000.0],
            false,
            false,
            false,
        );
        scene.push_live_input(0.1, slew.0, slew.1);
        assert_eq!(slew, (false, false));
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
        assert!(scene.input.effective_sas());
        assert_eq!(scene.input.throttle, 0.4);
        assert_eq!(scene.sample.time, time);
        assert_eq!(scene.camera, camera);
        scene.advance_flight(0.025, None).unwrap();
        assert!(
            scene.sample.time.0 > time.0,
            "map must not pause the authority"
        );
        let data = scene.map_snapshot().unwrap();
        assert!(data.craft.is_some());
        scene.set_map_open(false);
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
        assert!(scene.map_snapshot().is_none());
        assert_eq!(scene.camera, camera);
    }

    #[test]
    fn mouse_aim_and_sas_have_exclusive_camera_widget_and_focus_ownership() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene.control_mode = thessa_flight_authority::ControlMode::MouseAim;
        let held = [KeyCode::KeyF, KeyCode::ShiftLeft].into_iter().collect();
        let mut condition = |keyboard, pointer, gesture| {
            let slew = condition_flight_input(
                &mut scene,
                &held,
                Some([1000.0, 500.0]),
                [1000.0, 1000.0],
                keyboard,
                pointer,
                gesture,
            );
            scene.push_live_input(0.0, false, false);
            (scene.control_axes(), scene.input.effective_sas(), slew)
        };
        let (axes, sas, slew) = condition(false, false, false);
        assert!(axes.y > 0.6);
        assert!(!sas);
        assert_eq!(slew, (true, false));
        for (keyboard, pointer, gesture) in [
            (false, true, false),
            (false, false, true),
            (true, false, false),
        ] {
            let (axes, sas, slew) = condition(keyboard, pointer, gesture);
            assert_eq!(axes, glam::DVec3::ZERO);
            assert_eq!(
                sas, keyboard,
                "keyboard capture releases the held SAS override"
            );
            assert_eq!(slew, (!keyboard, false));
        }
        let held = [KeyCode::KeyW].into_iter().collect();
        condition_flight_input(
            &mut scene,
            &held,
            Some([1000.0, 500.0]),
            [1000.0, 1000.0],
            false,
            false,
            false,
        );
        assert_eq!(
            scene.control_axes(),
            glam::DVec3::NEG_X,
            "manual keys take over cursor steering"
        );
        scene.request_pause(true);
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
        scene.request_pause(false);
        condition_flight_input(
            &mut scene,
            &HashSet::new(),
            None,
            [1000.0, 1000.0],
            false,
            false,
            false,
        );
        assert_eq!(scene.control_axes(), glam::DVec3::ZERO);
    }

    #[test]
    fn time_shortcuts_are_edge_triggered_and_do_not_collide_with_the_camera() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        for key in [
            KeyCode::F8,
            KeyCode::BracketLeft,
            KeyCode::BracketRight,
            KeyCode::Backspace,
        ] {
            assert_eq!(camera_shortcut(PhysicalKey::Code(key), false), None);
            let clock = scene.flight_clock();
            apply_discrete_flight_input(&mut scene, PhysicalKey::Code(key), true, false);
            apply_discrete_flight_input(&mut scene, PhysicalKey::Code(key), false, true);
            assert_eq!(scene.flight_clock(), clock);
        }
        apply_discrete_flight_input(&mut scene, PhysicalKey::Code(KeyCode::F8), false, false);
        assert!(scene.flight_clock().unwrap().requested_paused);
        apply_discrete_flight_input(
            &mut scene,
            PhysicalKey::Code(KeyCode::BracketRight),
            false,
            false,
        );
        assert_eq!(scene.flight_clock().unwrap().requested_warp, 2.0);
        apply_discrete_flight_input(
            &mut scene,
            PhysicalKey::Code(KeyCode::BracketLeft),
            false,
            false,
        );
        assert_eq!(scene.flight_clock().unwrap().requested_warp, 1.0);
        let mut app = App {
            scene,
            state: None,
            error: None,
            frame_limit: None,
            frames: 0,
            started: None,
            hud_enabled: false,
            width: 1280,
            height: 720,
            right_held: true,
            middle_held: true,
            right_consumed: false,
            middle_consumed: false,
            last_cursor: Some((10.0, 20.0)),
            pointer_consumed: false,
            held: [KeyCode::KeyW, KeyCode::ShiftLeft].into_iter().collect(),
            shift_held: true,
            audio_options: Default::default(),
            audio: None,
        };
        app.scene.input.keys.nose_down = true;
        app.scene.input.mouse_direction = glam::Vec3::X;
        app.scene.input.sas_inverted = true;
        app.scene.push_live_input(0.0, false, false);
        assert_eq!(app.scene.input.stick_scaled(), glam::DVec3::NEG_X);
        let time = app.scene.sample.time;
        let throttle = app.scene.input.throttle;
        app.release_held_input();
        assert!(app.held.is_empty());
        assert!(!app.right_held && !app.middle_held);
        assert!(app.last_cursor.is_none());
        assert_eq!(app.scene.input.stick_scaled(), glam::DVec3::ZERO);
        assert_eq!(app.scene.input.mouse_direction, glam::Vec3::NEG_Z);
        assert!(!app.scene.input.sas_inverted);
        assert_eq!(
            app.scene.input.throttle, throttle,
            "focus loss must not invent an engine cut"
        );
        assert_eq!(app.scene.sample.time, time);
    }

    #[test]
    fn perf_keys_map_f4_shift_f4_and_f5() {
        assert_eq!(
            perf_key(PhysicalKey::Code(KeyCode::F4), false),
            Some(PerfKey::Overlay)
        );
        assert_eq!(
            perf_key(PhysicalKey::Code(KeyCode::F4), true),
            Some(PerfKey::Capture)
        );
        assert_eq!(
            perf_key(PhysicalKey::Code(KeyCode::F5), false),
            Some(PerfKey::Level)
        );
        assert_eq!(perf_key(PhysicalKey::Code(KeyCode::F3), false), None);
    }

    #[test]
    fn focused_widgets_block_camera_and_exit_shortcuts() {
        for (key, expected) in [
            (KeyCode::Escape, Shortcut::Exit),
            (KeyCode::ArrowLeft, Shortcut::Orbit(-0.05)),
            (KeyCode::ArrowRight, Shortcut::Orbit(0.05)),
            (KeyCode::ArrowUp, Shortcut::Zoom(1.0 / 1.1)),
            (KeyCode::ArrowDown, Shortcut::Zoom(1.1)),
            (KeyCode::KeyV, Shortcut::ToggleChase),
            (KeyCode::Backquote, Shortcut::ResetView),
            (KeyCode::F6, Shortcut::CycleSite),
        ] {
            assert_eq!(camera_shortcut(PhysicalKey::Code(key), true), None);
            assert_eq!(
                camera_shortcut(PhysicalKey::Code(key), false),
                Some(expected)
            );
        }
        assert_eq!(
            camera_shortcut(PhysicalKey::Code(KeyCode::KeyA), false),
            None
        );
    }

    #[test]
    fn part_keys_match_the_old_pilot_bindings() {
        use crate::scene::PartToggle;
        for (key, expected) in [
            (KeyCode::KeyR, Some(PartToggle::Rcs)),
            (KeyCode::KeyY, Some(PartToggle::ReactionWheels)),
            (KeyCode::KeyG, Some(PartToggle::Gear)),
            (KeyCode::KeyP, Some(PartToggle::Parachutes)),
            (KeyCode::KeyT, None),
            (KeyCode::KeyC, None),
            (KeyCode::Space, None),
        ] {
            assert_eq!(part_toggle_for_key(PhysicalKey::Code(key)), expected);
        }
    }

    #[test]
    fn drag_and_wheel_gestures_stay_out_of_survey_and_ui() {
        assert_eq!(drag_delta(true, false, true, 3.0, -2.0), Some((3.0, -2.0)));
        assert_eq!(drag_delta(true, false, true, 0.0, 0.0), None);
        assert_eq!(drag_delta(false, false, true, 3.0, 0.0), None);
        assert_eq!(drag_delta(true, true, true, 3.0, 0.0), None);
        assert_eq!(drag_delta(true, false, false, 3.0, 0.0), None);
        assert_eq!(wheel_lines(MouseScrollDelta::LineDelta(0.0, 2.0)), 2.0);
        assert!(
            !wheel_lines(MouseScrollDelta::PixelDelta(
                winit::dpi::PhysicalPosition::new(0.0, f64::NAN)
            ))
            .is_finite()
        );
    }
}
