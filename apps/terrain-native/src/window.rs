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
    depth: wgpu::TextureView,
    drawable: bool,
    exposure: f32,
}

impl State {
    fn new(event_loop: &ActiveEventLoop, scene: &Scene) -> Result<Self, Box<dyn Error>> {
        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("Thessa — owned terrain migration viewer")
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
        let depth = depth(&backend, config.width, config.height);
        Ok(Self {
            window,
            instance,
            surface,
            config,
            backend,
            renderer,
            depth,
            drawable: size.width > 0 && size.height > 0,
            exposure,
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

    fn draw(&mut self, scene: &Scene) -> Result<bool, Box<dyn Error>> {
        if !self.drawable {
            return Ok(false);
        }
        let (frame, reconfigure) = match self.surface.get_current_texture() {
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
        self.renderer.update_view(
            &self.backend,
            &scene.view(self.config.width, self.config.height, self.exposure),
        )?;
        let color = frame.texture.create_view(&Default::default());
        let mut encoder = self
            .backend
            .device
            .create_command_encoder(&Default::default());
        self.renderer.encode(&mut encoder, &color, &self.depth);
        self.window.pre_present_notify();
        self.backend.queue.submit([encoder.finish()]);
        self.backend.queue.present(frame);
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
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        match State::new(event_loop, &self.scene) {
            Ok(state) => {
                self.state = Some(state);
                self.started = Some(Instant::now());
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

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        let Some(state) = &mut self.state else {
            return;
        };
        if state.window.id() != id {
            return;
        }
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
                match event.physical_key {
                    PhysicalKey::Code(KeyCode::Escape) => event_loop.exit(),
                    PhysicalKey::Code(KeyCode::ArrowLeft) => self.scene.angle -= 0.05,
                    PhysicalKey::Code(KeyCode::ArrowRight) => self.scene.angle += 0.05,
                    PhysicalKey::Code(KeyCode::ArrowUp) => {
                        self.scene.distance = (self.scene.distance / 1.1).max(1_000.0)
                    }
                    PhysicalKey::Code(KeyCode::ArrowDown) => {
                        self.scene.distance = (self.scene.distance * 1.1).min(200_000.0)
                    }
                    _ => {}
                }
            }
            WindowEvent::RedrawRequested => match state.draw(&self.scene) {
                Ok(true) => {
                    self.frames += 1;
                    if self.frame_limit.is_some_and(|n| self.frames >= n) {
                        let seconds = self.started.unwrap().elapsed().as_secs_f64();
                        println!(
                            "window_presented_frames={} elapsed_s={seconds:.3} presented_fps={:.1} (vsync/compositor included; not GPU-only or game FPS)",
                            self.frames,
                            self.frames as f64 / seconds
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

pub fn run(scene: Scene, frame_limit: Option<u32>) -> Result<(), Box<dyn Error>> {
    let event_loop = EventLoop::new()?;
    let mut app = App {
        scene,
        state: None,
        error: None,
        frame_limit,
        frames: 0,
        started: None,
    };
    event_loop.run_app(&mut app)?;
    if let Some(error) = app.error {
        return Err(error.into());
    }
    Ok(())
}
