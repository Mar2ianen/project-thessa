//! Toolkit-owned native window. Surface loss/resize never rebuilds terrain.
use crate::scene::Scene;
use std::{error::Error, sync::Arc, time::Instant};
use thessa_render_wgpu::{Backend, terrain::TerrainRenderer};
use winit::{
    application::ApplicationHandler,
    event::{ElementState, WindowEvent},
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
    mesh_renderer: Option<thessa_render_wgpu::mesh::MeshRenderer>,
    depth: wgpu::TextureView,
    drawable: bool,
    exposure: f32,
    #[cfg(feature = "hud")]
    hud: Option<WindowHud>,
    submitted_frames: u64,
    previous_cpu_ms: f64,
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
    ) -> Result<Self, Box<dyn Error>> {
        #[cfg(not(feature = "hud"))]
        let _ = hud_enabled;
        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("Thessa — owned terrain migration viewer")
                    .with_decorations(true)
                    .with_inner_size(winit::dpi::PhysicalSize::new(1280, 720)),
            )?,
        );
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_with_display_handle(
            Box::new(event_loop.owned_display_handle()),
        ));
        let surface = instance.create_surface(window.clone())?;
        let backend = Backend::for_surface(instance.clone(), &surface)?;
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
        let resolved = thessa_graphics::ResolvedGraphicsSettings::from_requested(
            &scene.requested,
            &backend.caps,
        );
        config.present_mode = if resolved.vsync {
            wgpu::PresentMode::AutoVsync
        } else {
            wgpu::PresentMode::AutoNoVsync
        };
        let exposure = crate::manual_exposure(resolved.exposure_ev100);
        surface.configure(&backend.device, &config);
        let renderer = scene.renderer(&backend, config.format)?;
        let mesh_renderer = scene.mesh_renderer(&backend, config.format)?;
        let depth = depth(&backend, config.width, config.height);
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
            mesh_renderer,
            depth,
            drawable: size.width > 0 && size.height > 0,
            exposure,
            #[cfg(feature = "hud")]
            hud,
            submitted_frames: 0,
            previous_cpu_ms: 0.0,
        })
    }

    fn resize(&mut self, width: u32, height: u32) {
        self.drawable = width > 0 && height > 0;
        if !self.drawable {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.backend.device, &self.config);
        self.depth = depth(&self.backend, width, height);
    }

    fn draw(&mut self, scene: &mut Scene, now_s: f64) -> Result<bool, Box<dyn Error>> {
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
            let mut output = hud.gpu.run(
                hud.input.take_egui_input(&self.window),
                &mut controls,
                &crate::hud::Stats::from_scene(scene, self.submitted_frames, self.previous_cpu_ms),
            );
            controls.apply(scene);
            hud.input
                .handle_platform_output(&self.window, std::mem::take(&mut output.platform_output));
            if before.exposure_ev100 != controls.exposure_ev100 || before.vsync != controls.vsync {
                let resolved = thessa_graphics::ResolvedGraphicsSettings::from_requested(
                    &scene.requested,
                    &self.backend.caps,
                );
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
        scene.tick(
            &self.backend,
            &mut self.renderer,
            self.config.width,
            self.config.height,
            self.exposure,
            now_s,
        )?;
        self.renderer.update_view(
            &self.backend,
            &scene.view(self.config.width, self.config.height, self.exposure),
        )?;
        if let Some(mesh) = &mut self.mesh_renderer {
            scene.update_mesh(
                &self.backend,
                mesh,
                self.config.width,
                self.config.height,
                self.exposure,
            )?;
        }
        let color = frame.texture.create_view(&Default::default());
        let mut encoder = self
            .backend
            .device
            .create_command_encoder(&Default::default());
        self.renderer.encode(&mut encoder, &color, &self.depth);
        if let Some(mesh) = &self.mesh_renderer {
            mesh.encode(&mut encoder, &color, &self.depth);
        }
        #[cfg(feature = "hud")]
        let commands = if let (Some(hud), Some(output)) = (&mut self.hud, hud_output) {
            hud.gpu.encode(
                &self.backend,
                &mut encoder,
                &color,
                output,
                [self.config.width, self.config.height],
            )
        } else {
            Vec::new()
        };
        #[cfg(not(feature = "hud"))]
        let commands = Vec::<wgpu::CommandBuffer>::new();
        self.window.pre_present_notify();
        self.backend.queue.submit(
            commands
                .into_iter()
                .chain(std::iter::once(encoder.finish())),
        );
        self.backend.queue.present(frame);
        self.submitted_frames += 1;
        self.previous_cpu_ms = cpu_started.elapsed().as_secs_f64() * 1000.0;
        if reconfigure {
            self.resize(self.config.width, self.config.height);
        }
        Ok(true)
    }
}

fn depth(backend: &Backend, width: u32, height: u32) -> wgpu::TextureView {
    backend
        .device
        .create_texture(&wgpu::TextureDescriptor {
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
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

struct App {
    scene: Scene,
    state: Option<State>,
    error: Option<String>,
    frame_limit: Option<u32>,
    frames: u32,
    started: Option<Instant>,
    hud_enabled: bool,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        match State::new(event_loop, &mut self.scene, self.hud_enabled) {
            Ok(state) => {
                self.state = Some(state);
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
        self.state = None;
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        // egui-winit's clipboard borrows the native display handle. run_app
        // consumes/destroys EventLoop, while App itself lives beyond it, so
        // teardown must happen here while the display is still valid.
        self.state = None;
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
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
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size.width, size.height),
            WindowEvent::Occluded(occluded) => {
                if occluded {
                    state.drawable = false;
                } else {
                    let size = state.window.inner_size();
                    state.resize(size.width, size.height);
                }
            }
            WindowEvent::KeyboardInput { event, .. } if event.state == ElementState::Pressed => {
                match camera_shortcut(event.physical_key, consumed) {
                    Some(Shortcut::Exit) => event_loop.exit(),
                    Some(Shortcut::Orbit(delta)) => self.scene.angle += delta,
                    Some(Shortcut::Zoom(factor)) => {
                        let (min, max, _) = self.scene.camera_distance_limits();
                        self.scene.distance = (self.scene.distance * factor).clamp(min, max)
                    }
                    _ => {}
                }
            }
            WindowEvent::RedrawRequested => match state.draw(
                &mut self.scene,
                self.started.unwrap().elapsed().as_secs_f64(),
            ) {
                Ok(true) => {
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
                        event_loop.exit();
                    }
                }
                Ok(false) => {}
                Err(e) => {
                    self.error = Some(e.to_string());
                    event_loop.exit();
                }
            },
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

#[derive(Debug, PartialEq)]
enum Shortcut {
    Exit,
    Orbit(f64),
    Zoom(f64),
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
        _ => None,
    }
}

pub fn run(
    scene: Scene,
    frame_limit: Option<u32>,
    hud_enabled: bool,
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
    fn focused_widgets_block_camera_and_exit_shortcuts() {
        for (key, expected) in [
            (KeyCode::Escape, Shortcut::Exit),
            (KeyCode::ArrowLeft, Shortcut::Orbit(-0.05)),
            (KeyCode::ArrowRight, Shortcut::Orbit(0.05)),
            (KeyCode::ArrowUp, Shortcut::Zoom(1.0 / 1.1)),
            (KeyCode::ArrowDown, Shortcut::Zoom(1.1)),
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
}
