//! Native distant-star and depth-clipped cloud consumers of shared optical data.
//! No ECS, new simulation state, plugin toggles or wall-clock animation.
use glam::{DQuat, DVec3};
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_wgpu::{
    Backend, BackendError,
    view::{ViewParams, ViewUniformBlock},
};

const STAR_COUNT: u32 = 500;
const STAR_CAPACITY: u32 = STAR_COUNT + 3;
const CLOUD_BASE_M: f32 = 3000.0;
const CLOUD_THICKNESS_M: f32 = 3000.0;
const CLOUD_EXTINCTION_PER_M: f32 = 0.0006;

pub struct SkyLayers {
    active: Option<Active>,
}
struct Active {
    view: wgpu::Buffer,
    frame: wgpu::Buffer,
    radius: f64,
    stars: Option<Stars>,
    clouds: Option<Clouds>,
    volume: Option<Volume>,
}
struct Volume {
    pipeline: wgpu::RenderPipeline,
    emission: wgpu::Buffer,
    aurora: bool,
    placeholder: thessa_render_wgpu::clouds::CloudBinding,
}
struct Stars {
    pipeline: wgpu::RenderPipeline,
    bind: wgpu::BindGroup,
    buffer: wgpu::Buffer,
    catalogue: Vec<(DVec3, [f64; 3])>,
    steps: u32,
}
struct Clouds {
    binding: thessa_render_wgpu::clouds::CloudBinding,
    cast_shadows: bool,
    steps: u32,
    sun_steps: u32,
    layers: u32,
    animate: bool,
    bytes: u64,
}

fn bytes(values: impl IntoIterator<Item = f32>) -> Vec<u8> {
    values.into_iter().flat_map(f32::to_le_bytes).collect()
}

fn catalogue() -> Vec<(DVec3, [f64; 3])> {
    // Synthetic fixed distant catalogue, not ephemeris bodies. Uniform sphere
    // sampling avoids the old cube-normalized distribution's diagonal bias.
    let mut seed = 0x5eed2026u32;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        f64::from(seed) / f64::from(u32::MAX)
    };
    (0..STAR_COUNT)
        .map(|_| {
            let z = 2.0 * random() - 1.0;
            let azimuth = random() * std::f64::consts::TAU;
            let radial = (1.0 - z * z).sqrt();
            let direction = DVec3::new(radial * azimuth.cos(), z, radial * azimuth.sin());
            let colour = thessa_atmosphere::blackbody_rgb(2500.0 + random() * 12000.0);
            let flux = 8e-6 * random().powi(4).max(0.0005);
            (direction, colour.map(|c| c * flux))
        })
        .collect()
}

fn cloud_phases(time_s: f64, animate: bool) -> [f32; 2] {
    if !animate {
        return [0.0; 2];
    }
    // Wrap each deck independently in f64, before narrowing. Multiplying an
    // already-wrapped low-deck phase by 2.5 jumps the high deck at each wrap.
    [1.0, 2.5].map(|rate| ((time_s * 5e-6 * rate) / std::f64::consts::TAU).rem_euclid(1.0) as f32)
}

fn stellar_transmission(
    radius: f64,
    camera: DVec3,
    direction: DVec3,
    optics: Option<&thessa_atmosphere::AtmosphereOptics>,
    steps: u32,
) -> [f64; 3] {
    // Solid-body occultation is independent of the atmosphere toggle and of
    // the resident terrain patch footprint. Never show far-side stars through it.
    if !thessa_atmosphere::clip_to_planet(0.0, 1e30, camera, direction, DVec3::ZERO, radius)
        .is_some_and(|(_, end)| end == 1e30)
    {
        return [0.0; 3];
    }
    optics.map_or([1.0; 3], |o| {
        thessa_atmosphere::transmittance(o, camera, direction, steps)
    })
}

impl SkyLayers {
    pub fn new(
        backend: &Backend,
        settings: &ResolvedGraphicsSettings,
        format: wgpu::TextureFormat,
        radius: f64,
    ) -> Self {
        let aurora = settings.aurora_shell && settings.aurora_intensity > 0.0;
        if !settings.stars && !settings.clouds_enabled && !aurora {
            return Self { active: None };
        }
        let device = &backend.device;
        let view = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("native-sky-view"),
            size: 464,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let frame = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("native-sky-frame"),
            size: 176,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("native-stars-clouds"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}\n{}",
                    thessa_render_wgpu::clouds::SHADER,
                    thessa_atmosphere::AURORA_WGSL,
                    include_str!("sky_layers.wgsl")
                )
                .into(),
            ),
        });
        let pipeline = |star: bool| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(if star {
                    "native-stars"
                } else {
                    "native-cloud-volume"
                }),
                layout: None,
                vertex: wgpu::VertexState {
                    module: &module,
                    entry_point: Some(if star { "star_vertex" } else { "volume_vertex" }),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                primitive: Default::default(),
                depth_stencil: star.then_some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(false),
                    depth_compare: Some(wgpu::CompareFunction::LessEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(if star {
                        "star_fragment"
                    } else {
                        "cloud_fragment"
                    }),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(if star {
                            wgpu::BlendState {
                                color: wgpu::BlendComponent {
                                    src_factor: wgpu::BlendFactor::One,
                                    dst_factor: wgpu::BlendFactor::One,
                                    operation: wgpu::BlendOperation::Add,
                                },
                                alpha: wgpu::BlendComponent::OVER,
                            }
                        } else {
                            wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING
                        }),
                        write_mask: if star {
                            wgpu::ColorWrites::RED
                                | wgpu::ColorWrites::GREEN
                                | wgpu::ColorWrites::BLUE
                        } else {
                            wgpu::ColorWrites::ALL
                        },
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let stars = settings.stars.then(|| {
            let pipeline = pipeline(true);
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("distant-star-flux"),
                size: u64::from(STAR_CAPACITY) * 32,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: view.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: frame.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: buffer.as_entire_binding(),
                    },
                ],
            });
            Stars {
                pipeline,
                bind,
                buffer,
                catalogue: catalogue(),
                steps: match settings.stars_quality {
                    Quality::Low => 8,
                    Quality::Medium => 16,
                    Quality::High => 32,
                },
            }
        });
        let clouds=settings.clouds_enabled.then(|| {
            let (width,height)=settings.clouds_quality.cloud_size();
            let data=thessa_atmosphere::clouds::bake_clouds(0x5eed2026,width,height,settings.clouds_coverage,settings.clouds_opacity,(1.0,1.0));
            let texture=device.create_texture(&wgpu::TextureDescriptor {label:Some("canonical-cloud-weather"),size:wgpu::Extent3d {width,height,depth_or_array_layers:1},mip_level_count:1,sample_count:1,dimension:wgpu::TextureDimension::D2,format:wgpu::TextureFormat::Rgba8Unorm,usage:wgpu::TextureUsages::TEXTURE_BINDING|wgpu::TextureUsages::COPY_DST,view_formats:&[]});
            backend.queue.write_texture(texture.as_image_copy(),&data,wgpu::TexelCopyBufferLayout {offset:0,bytes_per_row:Some(width*4),rows_per_image:Some(height)},texture.size());
            let sampler=device.create_sampler(&wgpu::SamplerDescriptor {address_mode_u:wgpu::AddressMode::Repeat,address_mode_v:wgpu::AddressMode::ClampToEdge,mag_filter:wgpu::FilterMode::Linear,min_filter:wgpu::FilterMode::Linear,..Default::default()});
            if settings.clouds_volumetric && settings.cloud_integrator==thessa_graphics::CloudIntegratorRequest::Stochastic {eprintln!("native_clouds: stochastic integrator unsupported; using resolved deterministic march fallback");}
            Clouds {binding:thessa_render_wgpu::clouds::CloudBinding {frame:frame.clone(),weather:texture.create_view(&Default::default()),sampler},cast_shadows:settings.clouds_cast_shadows,steps:if settings.clouds_volumetric {settings.clouds_steps.clamp(1,64)}else{1},sun_steps:match settings.clouds_quality {Quality::Low=>0,Quality::Medium=>2,Quality::High=>4},layers:settings.clouds_layers.clamp(1,2),animate:settings.clouds_animate,bytes:u64::from(width)*u64::from(height)*4}
        });
        let volume = (clouds.is_some() || aurora).then(|| {
            let mut placeholder = thessa_render_wgpu::clouds::CloudBinding::disabled(device);
            placeholder.frame = frame.clone();
            Volume {
                pipeline: pipeline(false),
                emission: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("native-aurora-optics"),
                    size: 192,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }),
                aurora,
                placeholder,
            }
        });
        Self {
            active: Some(Active {
                view,
                frame,
                radius,
                stars,
                clouds,
                volume,
            }),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &self,
        backend: &Backend,
        view: &ViewParams,
        extent: [u32; 2],
        time_s: f64,
        body_from_inertial: DQuat,
        lighting: &SurfaceLighting,
        optics: Option<&thessa_atmosphere::AtmosphereOptics>,
        sources: &[thessa_atmosphere::CelestialLight],
    ) -> Result<(), BackendError> {
        let Some(active) = &self.active else {
            return Ok(());
        };
        let block = ViewUniformBlock::new(view)
            .ok_or_else(|| BackendError::Device("invalid sky-layer view".into()))?;
        if extent.contains(&0) || !time_s.is_finite() || !body_from_inertial.is_finite() {
            return Err(BackendError::Device("invalid sky-layer frame".into()));
        }
        backend
            .queue
            .write_buffer(&active.view, 0, &block.to_bytes());
        let cloud = active.clouds.as_ref();
        let phase = cloud_phases(time_s, cloud.is_some_and(|c| c.animate));
        let values = view
            .world_position_m
            .to_array()
            .map(|v| v as f32)
            .into_iter()
            .chain([
                active.radius as f32,
                CLOUD_BASE_M,
                CLOUD_THICKNESS_M,
                CLOUD_EXTINCTION_PER_M,
                cloud.map_or(1, |c| c.steps) as f32,
                phase[0],
                cloud.map_or(0, |c| c.layers) as f32,
                cloud.map_or(0, |c| c.sun_steps) as f32,
                f32::from(cloud.is_some_and(|c| c.cast_shadows)),
                extent[0] as f32,
                extent[1] as f32,
                phase[1],
                0.0,
            ])
            .chain(lighting.directions.into_iter().flatten())
            .chain(lighting.colors_lux.into_iter().flatten())
            .chain(lighting.ambient_lux);
        backend.queue.write_buffer(&active.frame, 0, &bytes(values));
        if let Some(stars) = &active.stars {
            let mut data = Vec::with_capacity(STAR_CAPACITY as usize * 32);
            for &(direction, flux) in &stars.catalogue {
                let direction = body_from_inertial * direction;
                let trans = stellar_transmission(
                    active.radius,
                    view.world_position_m,
                    direction,
                    optics,
                    stars.steps,
                );
                data.extend(bytes(
                    direction
                        .to_array()
                        .map(|x| x as f32)
                        .into_iter()
                        .chain([0.0])
                        .chain((0..3).map(|i| (flux[i] * trans[i]) as f32))
                        .chain([0.0]),
                ));
            }
            for source in sources.iter().take(3) {
                let direction = source.direction_to_star;
                let trans = stellar_transmission(
                    active.radius,
                    view.world_position_m,
                    direction,
                    optics,
                    stars.steps,
                );
                let flux = source.weighted_rgb();
                data.extend(bytes(
                    direction
                        .to_array()
                        .map(|x| x as f32)
                        .into_iter()
                        .chain([source.angular_radius_rad as f32])
                        .chain((0..3).map(|i| {
                            (flux[i] * trans[i] * crate::environment::LUX_PER_W_M2) as f32
                        }))
                        .chain([0.0]),
                ));
            }
            data.resize(STAR_CAPACITY as usize * 32, 0);
            backend.queue.write_buffer(&stars.buffer, 0, &data);
        }
        Ok(())
    }

    pub fn enabled(&self) -> bool {
        self.active.is_some()
    }

    pub fn update_aurora(
        &self,
        backend: &Backend,
        input: Option<&(thessa_atmosphere::AtmosphereOptics, u32, bool)>,
        time_s: f64,
    ) -> Result<(), BackendError> {
        let Some(volume) = self
            .active
            .as_ref()
            .and_then(|a| a.volume.as_ref())
            .filter(|v| v.aurora)
        else {
            return Ok(());
        };
        let Some((optics, steps, animate)) = input else {
            backend.queue.write_buffer(&volume.emission, 0, &[0; 192]);
            return Ok(());
        };
        let Some(params) = &optics.emission.aurora else {
            backend.queue.write_buffer(&volume.emission, 0, &[0; 192]);
            return Ok(());
        };
        if optics.absorption.len() > 4 || !time_s.is_finite() {
            return Err(BackendError::Device(
                "native aurora requires finite time and at most four optical absorption layers"
                    .into(),
            ));
        }
        optics
            .validate()
            .map_err(|e| BackendError::Device(e.to_string()))?;
        let phase = thessa_atmosphere::aurora_phase(params, if *animate { time_s } else { 0.0 });
        let mut values = vec![
            optics.inner_radius_m,
            params.altitude_min_m,
            params.altitude_max_m,
            f64::from((*steps).clamp(1, 256)),
        ];
        values.extend(params.magnetic_pole.normalize().to_array());
        values.push(params.oval_latitude_rad);
        values.extend([params.oval_width_rad, phase[0], phase[1], phase[2]]);
        values.extend(params.color_bottom_rgb);
        values.push(crate::environment::LUX_PER_W_M2);
        values.extend(params.color_top_rgb);
        values.push(0.0);
        values.extend([
            optics.outer_radius_m,
            optics.rayleigh.scale_height_m,
            optics.mie.scale_height_m,
            optics.absorption.len() as f64,
        ]);
        values.extend(optics.rayleigh.beta_rgb);
        values.push(0.0);
        values.extend(optics.mie.beta_rgb);
        values.push(0.0);
        for layer in &optics.absorption {
            values.extend(layer.sigma_rgb);
            values.push(layer.scale_height_m);
        }
        values.resize(48, 0.0);
        backend.queue.write_buffer(
            &volume.emission,
            0,
            &bytes(values.into_iter().map(|v| v as f32)),
        );
        Ok(())
    }

    pub fn cloud_binding(&self) -> Option<thessa_render_wgpu::clouds::CloudBinding> {
        self.active
            .as_ref()?
            .clouds
            .as_ref()
            .map(|c| c.binding.clone())
    }
    pub fn storage_bytes(&self) -> u64 {
        self.active.as_ref().map_or(0, |a| {
            640 + a
                .stars
                .as_ref()
                .map_or(0, |_| u64::from(STAR_CAPACITY) * 32)
                + a.clouds.as_ref().map_or(0, |c| c.bytes)
                + u64::from(a.volume.is_some()) * 196
        })
    }
    pub fn metadata(&self) -> std::collections::BTreeMap<String, String> {
        let stars = self.active.as_ref().and_then(|a| a.stars.as_ref());
        let cloud = self.active.as_ref().and_then(|a| a.clouds.as_ref());
        [
            ("native_sky_layer_bytes", self.storage_bytes().to_string()),
            (
                "native_aurora_depth_composition",
                self.active
                    .as_ref()
                    .and_then(|a| a.volume.as_ref())
                    .is_some_and(|v| v.aurora)
                    .to_string(),
            ),
            (
                "native_background_star_count",
                if stars.is_some() { STAR_COUNT } else { 0 }.to_string(),
            ),
            (
                "native_stellar_transmission_steps",
                stars.map_or(0, |s| s.steps).to_string(),
            ),
            (
                "native_cloud_mode",
                cloud
                    .map_or("disabled", |c| {
                        if c.steps == 1 {
                            "shell_column"
                        } else {
                            "deterministic_march"
                        }
                    })
                    .into(),
            ),
            (
                "native_cloud_steps",
                cloud.map_or(0, |c| c.steps).to_string(),
            ),
            (
                "native_cloud_sun_steps",
                cloud.map_or(0, |c| c.sun_steps).to_string(),
            ),
            (
                "native_cloud_layers",
                cloud.map_or(0, |c| c.layers).to_string(),
            ),
            (
                "native_cloud_ground_shadows",
                cloud.is_some_and(|c| c.cast_shadows).to_string(),
            ),
        ]
        .into_iter()
        .map(|(key, value)| (key.into(), value))
        .collect()
    }
    pub fn encode(
        &self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        let Some(active) = &self.active else {
            return;
        };
        let attachment = || {
            Some(wgpu::RenderPassColorAttachment {
                view: color,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })
        };
        if let Some(stars) = &active.stars {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("native-distant-stars"),
                color_attachments: &[attachment()],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });
            pass.set_pipeline(&stars.pipeline);
            pass.set_bind_group(0, &stars.bind, &[]);
            pass.draw(0..6, 0..STAR_CAPACITY);
        }
        if let Some(volume) = &active.volume {
            let mut entries = vec![
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: active.view.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: wgpu::BindingResource::TextureView(depth),
                },
            ];
            entries.extend(
                active
                    .clouds
                    .as_ref()
                    .map_or(&volume.placeholder, |c| &c.binding)
                    .entries(),
            );
            entries.push(wgpu::BindGroupEntry {
                binding: 9,
                resource: volume.emission.as_entire_binding(),
            });
            let bind = backend
                .device
                .create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("native-depth-clipped-clouds"),
                    layout: &volume.pipeline.get_bind_group_layout(0),
                    entries: &entries,
                });
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("native-depth-clipped-clouds"),
                color_attachments: &[attachment()],
                ..Default::default()
            });
            pass.set_pipeline(&volume.pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.draw(0..3, 0..1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shader_is_portable() {
        let source = format!(
            "{}\n{}\n{}",
            thessa_render_wgpu::clouds::SHADER,
            thessa_atmosphere::AURORA_WGSL,
            include_str!("sky_layers.wgsl")
        );
        let module = naga::front::wgsl::parse_str(&source)
            .unwrap_or_else(|e| panic!("{}", e.emit_to_string(&source)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&module)
        .unwrap();
    }
    #[test]
    fn catalogue_is_stable_finite_and_quality_independent() {
        let a = catalogue();
        assert_eq!(a, catalogue());
        assert_eq!(a.len(), STAR_COUNT as usize);
        for (direction, flux) in a {
            assert!((direction.length() - 1.0).abs() < 1e-12);
            assert!(flux.iter().all(|&v| v.is_finite() && v >= 0.0));
        }
    }

    #[test]
    fn solid_body_occults_stars_without_scattering() {
        let camera = DVec3::Z * 3_201_000.0;
        assert_eq!(
            stellar_transmission(3_200_000.0, camera, -DVec3::Z, None, 8),
            [0.0; 3]
        );
        assert_eq!(
            stellar_transmission(3_200_000.0, camera, DVec3::Z, None, 8),
            [1.0; 3]
        );
    }

    #[test]
    fn cloud_warp_reverse_pause_and_differential_wrap_are_continuous() {
        let period = std::f64::consts::TAU / 5e-6;
        let before = cloud_phases(period - 0.1, true);
        let after = cloud_phases(period + 0.1, true);
        for layer in 0..2 {
            let delta = (after[layer] - before[layer]).rem_euclid(1.0);
            assert!(
                delta < 1e-5,
                "deck {layer} jumped at the other deck's period: {delta}"
            );
        }
        let positive = cloud_phases(1000.0, true);
        let negative = cloud_phases(-1000.0, true);
        for layer in 0..2 {
            assert!((positive[layer] + negative[layer] - 1.0).abs() < 1e-7);
        }
        assert_eq!(cloud_phases(1000.0, true), positive);
        assert_eq!(cloud_phases(period * 10.0, false), [0.0; 2]);
    }

    #[test]
    #[ignore = "hardware: fixed 1280x720 native sky-layer GPU/producer budgets, not game FPS"]
    fn native_sky_layer_target_size_cost() {
        use thessa_graphics::{RayTracingRequest, RequestedGraphics};
        use thessa_render_wgpu::{terrain::OffscreenTarget, timing::GpuIntervals};
        let mut request = RequestedGraphics::default();
        request.upper_atmosphere.aurora = false;
        request.renderer.ray_tracing = RayTracingRequest::Off;
        let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
        let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
        let display = thessa_render_wgpu::display::DisplayTransform::new(
            &backend,
            &backend.resolve_graphics(&request),
            OffscreenTarget::FORMAT,
            1280,
            720,
        )
        .unwrap();
        let timing = GpuIntervals::new(&backend, 2);
        let camera = DVec3::Z * 3_201_000.0;
        let view = ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(
                camera,
                camera + DVec3::Z,
                DVec3::Y,
            )
            .inverse(),
            world_position_m: camera,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                1.0,
                1280.0 / 720.0,
                0.1,
                1e7,
            ),
            exposure: 0.00001,
        };
        let light = SurfaceLighting {
            ambient_lux: [1000.0; 4],
            directions: [[0.0, 0.0, 1.0, 0.0]; 3],
            colors_lux: [[100000.0, 100000.0, 100000.0, 0.0], [0.0; 4], [0.0; 4]],
        };
        let optics =
            thessa_atmosphere::nitrogen_oxygen_optics(3_200_000.0, 101325.0, 8000.0).unwrap();
        for mode in [
            "disabled",
            "stars-low",
            "stars-high",
            "clouds-low",
            "clouds-medium",
            "clouds-high",
            "aurora-low",
            "aurora-high",
            "combined-high",
        ] {
            let combined = mode == "combined-high";
            request.atmosphere.stars = mode.starts_with("stars") || combined;
            request.clouds.enabled = mode.starts_with("clouds") || combined;
            request.upper_atmosphere.aurora = mode.starts_with("aurora") || combined;
            let quality = if mode.ends_with("high") {
                Quality::High
            } else if mode.ends_with("medium") {
                Quality::Medium
            } else {
                Quality::Low
            };
            request.atmosphere.stars_quality = quality;
            request.upper_atmosphere.aurora_quality = if quality == Quality::High {
                thessa_graphics::AuroraQuality::High
            } else {
                thessa_graphics::AuroraQuality::Low
            };
            request.clouds.quality = quality;
            request.clouds.volumetric = quality != Quality::Low;
            request.clouds.ray_steps = if quality == Quality::High { 32 } else { 16 };
            let layers = SkyLayers::new(
                &backend,
                &backend.resolve_graphics(&request),
                display.scene_format(),
                3_200_000.0,
            );
            let mut view = view;
            if request.upper_atmosphere.aurora {
                let lat = 67.0f64.to_radians();
                let up = DVec3::new(lat.cos(), lat.sin(), 0.0);
                let camera = up * 3_201_000.0;
                view.world_position_m = camera;
                view.world_from_view =
                    glam::dcamera::rh::view::look_at_mat4(camera, camera + up, DVec3::Z).inverse();
            }
            let mut emission_optics = optics.clone();
            emission_optics.emission.aurora = Some(crate::environment::aurora_params(1.0));
            let emission_input = (
                emission_optics,
                if quality == Quality::High { 32 } else { 8 },
                true,
            );
            let mut gpu = Vec::new();
            let mut wall = Vec::new();
            let mut producer = Vec::new();
            for frame in 0..24 {
                let start = std::time::Instant::now();
                layers
                    .update(
                        &backend,
                        &view,
                        [1280, 720],
                        0.0,
                        DQuat::IDENTITY,
                        &light,
                        Some(&optics),
                        &[],
                    )
                    .unwrap();
                layers
                    .update_aurora(&backend, Some(&emission_input), 0.0)
                    .unwrap();
                let cpu = start.elapsed().as_secs_f64() * 1000.0;
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                {
                    let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: None,
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: display.scene_color(&target.color),
                            depth_slice: None,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &target.depth,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.0),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        ..Default::default()
                    });
                }
                if let Some(t) = &timing {
                    t.mark(&mut encoder, 0);
                }
                layers.encode(
                    &backend,
                    &mut encoder,
                    display.scene_color(&target.color),
                    &target.depth,
                );
                if let Some(t) = &timing {
                    t.mark(&mut encoder, 1);
                    t.resolve(&mut encoder);
                }
                backend.queue.submit([encoder.finish()]);
                backend
                    .device
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
                let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                let measured = timing
                    .as_ref()
                    .map(|t| t.read(&backend).unwrap()[0] * 1000.0);
                if frame >= 4 {
                    producer.push(cpu);
                    wall.push(elapsed);
                    if let Some(v) = measured {
                        gpu.push(v);
                    }
                }
            }
            producer.sort_by(f64::total_cmp);
            wall.sort_by(f64::total_cmp);
            gpu.sort_by(f64::total_cmp);
            println!(
                "native_sky_cost adapter={} mode={mode} resolution=1280x720 measured=20 producer_p50_ms={:.6} producer_p95_ms={:.6} host_submit_wait_p50_ms={:.6} gpu_p50_ms={:?} gpu_p95_ms={:?} storage_bytes={}; clear outside GPU scope, no readback in wall sample, no terrain/game FPS claim",
                backend.adapter_name,
                producer[10],
                producer[19],
                wall[10],
                gpu.get(10),
                gpu.get(19),
                layers.storage_bytes()
            );
        }
    }

    #[test]
    #[ignore = "hardware: shared cloud primary/shadow/reflection queries vs f64 shell integration"]
    fn shared_cloud_transport_handles_inside_orbit_limb_and_rebased_origins() {
        use thessa_graphics::{RayTracingRequest, RequestedGraphics};
        let mut request = RequestedGraphics::default();
        request.upper_atmosphere.aurora = false;
        request.renderer.ray_tracing = RayTracingRequest::Off;
        request.atmosphere.stars = false;
        request.clouds.enabled = true;
        request.clouds.volumetric = true;
        request.clouds.ray_steps = 64;
        request.clouds.cast_shadows = true;
        let backend = Backend::bring_up_for_graphics(&request).unwrap();
        let settings = backend.resolve_graphics(&request);
        let radius = 3_200_000.0;
        let camera = DVec3::Z * (radius + 1000.0);
        let mut layers = SkyLayers::new(
            &backend,
            &settings,
            wgpu::TextureFormat::Rgba16Float,
            radius,
        );
        let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: None,
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        backend.queue.write_texture(
            texture.as_image_copy(),
            &[255; 4],
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4),
                rows_per_image: Some(1),
            },
            texture.size(),
        );
        layers
            .active
            .as_mut()
            .unwrap()
            .clouds
            .as_mut()
            .unwrap()
            .binding
            .weather = texture.create_view(&Default::default());
        let view = ViewParams {
            world_from_view: glam::DMat4::from_translation(camera),
            world_position_m: camera,
            clip_from_view: glam::DMat4::IDENTITY,
            exposure: 1.0,
        };
        let light = SurfaceLighting {
            ambient_lux: [std::f32::consts::PI; 4],
            colors_lux: [[0.0; 4]; 3],
            directions: [[0.0, 0.0, 1.0, 0.0]; 3],
        };
        layers
            .update(
                &backend,
                &view,
                [1, 1],
                0.0,
                DQuat::IDENTITY,
                &light,
                None,
                &[],
            )
            .unwrap();
        let source = format!(
            "{}\n{}",
            thessa_render_wgpu::clouds::SHADER,
            r#"
            @group(0) @binding(9) var<storage,read_write> output:array<vec4<f32>>;
            @compute @workgroup_size(1) fn query() {
                output[0]=cloud_transport(vec3(0.0),vec3(0.0,0.0,1.0),1e30);
                output[1]=cloud_transport(vec3(0.0,0.0,3500.0),vec3(0.0,0.0,1.0),1e30);
                output[2]=cloud_transport(vec3(0.0,0.0,12000.0),vec3(0.0,0.0,-1.0),1e30);
                output[3]=cloud_transport(vec3(0.0,0.0,12000.0),normalize(vec3(1.0,0.0,-0.075)),1e30);
                output[4]=cloud_transport(vec3(0.0),vec3(0.0,0.0,-1.0),1e30);
                output[5]=cloud_transport(vec3(0.0),vec3(0.0,0.0,1.0),1500.0);
                output[6]=vec4(cloud_sun_visibility(vec3(0.0),0u));
            }
        "#
        );
        let shader = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: None,
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
        let pipeline = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: None,
                layout: None,
                module: &shader,
                entry_point: Some("query"),
                compilation_options: Default::default(),
                cache: None,
            });
        let buffer = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 7 * 16,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let readback = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 7 * 16,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let binding = layers.cloud_binding().unwrap();
        let mut entries = binding.entries().to_vec();
        entries.push(wgpu::BindGroupEntry {
            binding: 9,
            resource: buffer.as_entire_binding(),
        });
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
                entries: &entries,
            });
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        encoder.copy_buffer_to_buffer(&buffer, 0, &readback, 0, 7 * 16);
        backend.queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        readback
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
        backend
            .device
            .poll(wgpu::PollType::wait_indefinitely())
            .unwrap();
        rx.recv().unwrap().unwrap();
        let mapped = readback.slice(..).get_mapped_range().unwrap();
        let values: Vec<_> = mapped
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        for (index, (offset, direction, stop)) in [
            (DVec3::ZERO, DVec3::Z, 1e30),
            (DVec3::Z * 3500.0, DVec3::Z, 1e30),
            (DVec3::Z * 12000.0, -DVec3::Z, 1e30),
            (
                DVec3::Z * 12000.0,
                DVec3::new(1.0, 0.0, -0.075).normalize(),
                1e30,
            ),
            (DVec3::ZERO, -DVec3::Z, 1e30),
            (DVec3::ZERO, DVec3::Z, 1500.0),
        ]
        .into_iter()
        .enumerate()
        {
            let origin = camera + offset;
            let mut total_tau = 0.0;
            for layer in 0..settings.clouds_layers {
                let thickness = f64::from(CLOUD_THICKNESS_M) / if layer == 0 { 1.0 } else { 3.0 };
                let bottom = radius + f64::from(CLOUD_BASE_M) + f64::from(layer) * 6000.0;
                let slab =
                    thessa_atmosphere::CloudSlab::new(radius, bottom, bottom + thickness).unwrap();
                let mut from = 0.0;
                for _ in 0..2 {
                    let Some((a, b)) = thessa_atmosphere::intersect_shell(
                        origin + direction * from,
                        direction,
                        DVec3::ZERO,
                        slab,
                    ) else {
                        break;
                    };
                    let a = a + from;
                    let b = b + from;
                    let Some((a, b)) = thessa_atmosphere::clip_to_planet(
                        a,
                        b.min(stop),
                        origin,
                        direction,
                        DVec3::ZERO,
                        radius,
                    ) else {
                        break;
                    };
                    total_tau += thessa_atmosphere::clouds::integrate_slab_unclamped_simpson(
                        |t| {
                            thessa_atmosphere::vertical_profile(
                                ((origin + direction * t).length() - bottom) / thickness,
                            )
                        },
                        a,
                        b,
                        8192,
                        f64::from(CLOUD_EXTINCTION_PER_M) / if layer == 0 { 1.0 } else { 3.0 },
                    )
                    .1;
                    from = b + 0.001;
                    if from >= stop {
                        break;
                    }
                }
            }
            let expected = (-total_tau).exp();
            let actual = f64::from(values[index * 4 + 3]);
            assert!(
                (actual - expected).abs() < 0.003,
                "query {index}: {actual} vs {expected}"
            );
            assert!(
                (f64::from(values[index * 4]) - (1.0 - expected)).abs() < 0.003,
                "shared reflected in-scatter {index}"
            );
        }
        assert!(
            (values[24] - values[3]).abs() < 0.003,
            "ground shadow samples the same column as primary/reflected rays"
        );
    }

    fn linear(value: u8) -> f64 {
        let s = f64::from(value) / 255.0;
        if s <= 0.04045 {
            s / 12.92
        } else {
            ((s + 0.055) / 1.055).powf(2.4)
        }
    }

    #[test]
    #[ignore = "hardware: depth-clipped aurora vs f64 transport and cloud/emitter ordering"]
    fn aurora_depth_and_cloud_order_match_finite_segment_oracle() {
        use thessa_graphics::{RayTracingRequest, RequestedGraphics};
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let mut request = RequestedGraphics::default();
        request.renderer.ray_tracing = RayTracingRequest::Off;
        request.atmosphere.stars = false;
        request.clouds.enabled = false;
        request.upper_atmosphere.aurora = true;
        request.upper_atmosphere.aurora_quality = thessa_graphics::AuroraQuality::High;
        let backend = Backend::bring_up_for_graphics(&request).unwrap();
        let radius = 3_200_000.0;
        let mut optics = thessa_atmosphere::nitrogen_oxygen_optics(radius, 0.0, 8000.0).unwrap();
        optics.absorption.clear();
        optics.emission.aurora = Some(thessa_atmosphere::AuroraParams {
            magnetic_pole: DVec3::Y,
            oval_latitude_rad: std::f64::consts::FRAC_PI_2,
            oval_width_rad: 0.15,
            altitude_min_m: 90000.0,
            altitude_max_m: 250000.0,
            activity: 0.8,
            color_bottom_rgb: [1e-6; 3],
            color_top_rgb: [1e-6; 3],
        });
        let target = OffscreenTarget::new(&backend, 33, 33).unwrap();
        let draw = |layers: &SkyLayers, camera: DVec3, direction: DVec3, distance: f64| {
            let view = ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(
                    camera,
                    camera + direction,
                    DVec3::Z,
                )
                .inverse(),
                world_position_m: camera,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 1e7),
                exposure: 0.1,
            };
            let light = SurfaceLighting {
                ambient_lux: [0.0; 4],
                colors_lux: [[0.0; 4]; 3],
                ..Default::default()
            };
            layers
                .update(
                    &backend,
                    &view,
                    [33, 33],
                    0.0,
                    DQuat::IDENTITY,
                    &light,
                    None,
                    &[],
                )
                .unwrap();
            layers
                .update_aurora(&backend, Some(&(optics.clone(), 32, true)), 0.0)
                .unwrap();
            let clip = view.clip_from_view * glam::DVec4::new(0.0, 0.0, -distance, 1.0);
            let depth = (clip.z / clip.w) as f32;
            // Use a reconstructed representable depth for the independent
            // oracle too: 32-bit conventional far depth quantizes orbital ranges.
            let a = f64::from(view.clip_from_view.z_axis.z as f32);
            let b = f64::from(view.clip_from_view.w_axis.z as f32);
            let actual_distance = (b / (f64::from(depth) + a)).abs();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: None,
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &target.color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &target.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(depth),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
            }
            layers.encode(&backend, &mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = target.read_rgba8(&backend).unwrap();
            (linear(pixels[(33 * 16 + 16) * 4]), actual_distance, pixels)
        };
        let clear = SkyLayers::new(
            &backend,
            &backend.resolve_graphics(&request),
            OffscreenTarget::FORMAT,
            radius,
        );
        let orbit = DVec3::Y * (radius + 400000.0);
        for distance in [50000.0, 200000.0, 400000.0] {
            let (measured, actual_distance, pixels) = draw(&clear, orbit, -DVec3::Y, distance);
            let expected = thessa_atmosphere::aurora_radiance_to(
                &optics,
                orbit,
                -DVec3::Y,
                actual_distance,
                0.0,
                128,
            )[0] * crate::environment::LUX_PER_W_M2
                * 0.1;
            println!(
                "aurora_depth requested_m={distance} depth_reconstructed_m={actual_distance} measured_linear={measured} expected_linear={expected}"
            );
            assert!(
                (measured - expected).abs() < 0.006,
                "finite foreground emission must follow scene depth"
            );
            if distance == 50000.0 {
                assert_eq!(measured, 0.0);
            }
            if distance == 400000.0 {
                let directory = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("target/test-tmp/sky-layers");
                std::fs::create_dir_all(&directory).unwrap();
                let mut png = png::Encoder::new(
                    std::fs::File::create(directory.join("aurora-orbit-depth.png")).unwrap(),
                    33,
                    33,
                );
                png.set_color(png::ColorType::Rgba);
                png.set_depth(png::BitDepth::Eight);
                png.write_header()
                    .unwrap()
                    .write_image_data(&pixels)
                    .unwrap();
            }
        }
        request.clouds.enabled = true;
        request.clouds.volumetric = true;
        request.clouds.ray_steps = 24;
        request.clouds.quality = Quality::Medium;
        let mut cloudy = SkyLayers::new(
            &backend,
            &backend.resolve_graphics(&request),
            OffscreenTarget::FORMAT,
            radius,
        );
        let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: None,
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        backend.queue.write_texture(
            texture.as_image_copy(),
            &[255; 4],
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4),
                rows_per_image: Some(1),
            },
            texture.size(),
        );
        cloudy
            .active
            .as_mut()
            .unwrap()
            .clouds
            .as_mut()
            .unwrap()
            .binding
            .weather = texture.create_view(&Default::default());
        let orbit_clear = draw(&clear, orbit, -DVec3::Y, 400000.0).0;
        let orbit_cloudy = draw(&cloudy, orbit, -DVec3::Y, 400000.0).0;
        assert!(
            (orbit_clear - orbit_cloudy).abs() < 0.003,
            "a cloud behind the emitter must not erase foreground aurora"
        );
        let ground = DVec3::Y * (radius + 1000.0);
        let ground_clear = draw(&clear, ground, DVec3::Y, 400000.0).0;
        let ground_cloudy = draw(&cloudy, ground, DVec3::Y, 400000.0).0;
        let transmission = thessa_atmosphere::integrate_slab(
            |h| thessa_atmosphere::vertical_profile(h / 3000.0),
            0.0,
            3000.0,
            24,
            f64::from(CLOUD_EXTINCTION_PER_M),
        )
        .0;
        assert!(
            (ground_cloudy - ground_clear * transmission).abs() < 0.006,
            "foreground clouds must attenuate the distant emitter: {ground_cloudy} vs {}",
            ground_clear * transmission
        );
    }

    #[test]
    #[ignore = "hardware: stellar flux across resolution and cloud Beer-Lambert column vs shared f64 oracle"]
    fn stellar_flux_and_cloud_column_match_numeric_oracles() {
        use thessa_graphics::{RayTracingRequest, RequestedGraphics};
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let mut request = RequestedGraphics::default();
        request.upper_atmosphere.aurora = false;
        request.renderer.ray_tracing = RayTracingRequest::Off;
        let backend = Backend::bring_up_for_graphics(&request).unwrap();
        let radius = 3_200_000.0;
        let camera = DVec3::Z * (radius + 1000.0);
        let view = |w: u32, h: u32, exposure| ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(
                camera,
                camera + DVec3::Z,
                DVec3::Y,
            )
            .inverse(),
            world_position_m: camera,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                1.0,
                f64::from(w) / f64::from(h),
                0.1,
                1e7,
            ),
            exposure,
        };
        let draw = |layers: &SkyLayers, w, h, exposure, light: &SurfaceLighting| {
            let target = OffscreenTarget::new(&backend, w, h).unwrap();
            let view = view(w, h, exposure);
            layers
                .update(
                    &backend,
                    &view,
                    [w, h],
                    0.0,
                    DQuat::IDENTITY,
                    light,
                    None,
                    &[],
                )
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: None,
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &target.color,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &target.depth,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: wgpu::StoreOp::Store,
                        }),
                        stencil_ops: None,
                    }),
                    ..Default::default()
                });
            }
            layers.encode(&backend, &mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let settings = backend.resolve_graphics(&request);
        let mut stars = SkyLayers::new(&backend, &settings, OffscreenTarget::FORMAT, radius);
        stars
            .active
            .as_mut()
            .unwrap()
            .stars
            .as_mut()
            .unwrap()
            .catalogue = vec![(DVec3::Z, [1e-6; 3])];
        for (w, h) in [(129, 65), (257, 129)] {
            let view = view(w, h, 20.0);
            let pixels = draw(&stars, w, h, 20.0, &SurfaceLighting::default());
            let omega = 4.0
                / (f64::from(w)
                    * f64::from(h)
                    * (view.clip_from_view.x_axis.x * view.clip_from_view.y_axis.y).abs());
            let measured = pixels
                .as_chunks::<4>()
                .0
                .iter()
                .map(|p| linear(p[0]))
                .sum::<f64>()
                * omega
                / 20.0;
            println!("stellar_flux {w}x{h} measured_lux={measured} expected_lux=0.000001");
            assert!(
                (measured - 1e-6).abs() < 3e-8,
                "flux changes with resolution or PSF normalization: {measured}"
            );
        }
        request.atmosphere.stars = false;
        request.clouds.enabled = true;
        request.clouds.volumetric = true;
        request.clouds.quality = Quality::Medium;
        request.clouds.ray_steps = 32;
        let settings = backend.resolve_graphics(&request);
        let mut clouds = SkyLayers::new(&backend, &settings, OffscreenTarget::FORMAT, radius);
        let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("constant-density-oracle"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::COPY_DST | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        backend.queue.write_texture(
            texture.as_image_copy(),
            &[255; 4],
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(4),
                rows_per_image: Some(1),
            },
            texture.size(),
        );
        clouds
            .active
            .as_mut()
            .unwrap()
            .clouds
            .as_mut()
            .unwrap()
            .binding
            .weather = texture.create_view(&Default::default());
        let light = SurfaceLighting {
            ambient_lux: [std::f32::consts::PI; 4],
            colors_lux: [[0.0; 4]; 3],
            ..Default::default()
        };
        let pixels = draw(&clouds, 33, 33, 0.5, &light);
        let (transmission, _) = thessa_atmosphere::integrate_slab(
            |h| thessa_atmosphere::vertical_profile(h / f64::from(CLOUD_THICKNESS_M)),
            0.0,
            f64::from(CLOUD_THICKNESS_M),
            settings.clouds_steps,
            f64::from(CLOUD_EXTINCTION_PER_M),
        );
        let expected = 0.5 * (1.0 - transmission);
        let measured = linear(pixels[((33 * 16 + 16) * 4) as usize]);
        println!("cloud_column measured_linear={measured} expected_linear={expected}");
        assert!(
            (measured - expected).abs() < 0.005,
            "cloud column differs from shared f64 transport"
        );
    }

    #[test]
    #[ignore = "hardware: native stars/clouds execute, depth occludes and disabled allocates nothing"]
    fn native_layers_obey_depth_origin_and_disabled_contracts() {
        use thessa_graphics::{RayTracingRequest, RequestedGraphics};
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let mut requested = RequestedGraphics::default();
        requested.upper_atmosphere.aurora = false;
        requested.renderer.ray_tracing = RayTracingRequest::Off;
        requested.atmosphere.stars = false;
        requested.clouds.enabled = false;
        let backend = Backend::bring_up_for_graphics(&requested).unwrap();
        let settings = backend.resolve_graphics(&requested);
        let disabled = SkyLayers::new(&backend, &settings, OffscreenTarget::FORMAT, 3_200_000.0);
        assert!(!disabled.enabled());
        assert_eq!(disabled.storage_bytes(), 0);
        let target = OffscreenTarget::new(&backend, 129, 65).unwrap();
        for clouds in [false, true] {
            requested.atmosphere.stars = !clouds;
            requested.clouds.enabled = clouds;
            requested.clouds.quality = Quality::Low;
            requested.clouds.coverage = 0.0;
            let settings = backend.resolve_graphics(&requested);
            let layers = SkyLayers::new(&backend, &settings, OffscreenTarget::FORMAT, 3_200_000.0);
            let camera = DVec3::Z * 3_201_000.0;
            let mut view = ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(
                    camera,
                    camera + DVec3::Z,
                    DVec3::Y,
                )
                .inverse(),
                world_position_m: camera,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    1.0,
                    129.0 / 65.0,
                    0.1,
                    1e7,
                ),
                exposure: if clouds { 0.0001 } else { 100.0 },
            };
            let light = SurfaceLighting {
                ambient_lux: [0.0; 4],
                directions: [[0.0, 0.0, 1.0, 0.0]; 3],
                colors_lux: [[1000.0, 1000.0, 1000.0, 0.0], [0.0; 4], [0.0; 4]],
            };
            let draw = |view: &ViewParams, depth: f32| {
                layers
                    .update(
                        &backend,
                        view,
                        [129, 65],
                        0.0,
                        DQuat::IDENTITY,
                        &light,
                        None,
                        &[],
                    )
                    .unwrap();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                {
                    let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: None,
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &target.color,
                            depth_slice: None,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &target.depth,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(depth),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        ..Default::default()
                    });
                }
                layers.encode(&backend, &mut encoder, &target.color, &target.depth);
                backend.queue.submit([encoder.finish()]);
                target.read_rgba8(&backend).unwrap()
            };
            let visible = draw(&view, 1.0);
            assert!(
                visible
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|p| p[..3].iter().any(|&c| c > 5)),
                "clouds={clouds}: actual nonempty rendering"
            );
            let directory =
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/sky-layers");
            std::fs::create_dir_all(&directory).unwrap();
            let file = std::fs::File::create(directory.join(if clouds {
                "native-clouds.png"
            } else {
                "native-stars.png"
            }))
            .unwrap();
            let mut png = png::Encoder::new(file, 129, 65);
            png.set_color(png::ColorType::Rgba);
            png.set_depth(png::BitDepth::Eight);
            png.write_header()
                .unwrap()
                .write_image_data(&visible)
                .unwrap();
            let blocked = draw(&view, 0.0);
            assert!(
                blocked
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .all(|p| p[..3] == [0, 0, 0]),
                "foreground depth must occlude clouds and stars"
            );
            if !clouds {
                view.world_position_m += DVec3::X * 1e6;
                view.world_from_view.w_axis = view.world_position_m.extend(1.0);
                assert_eq!(
                    visible,
                    draw(&view, 1.0),
                    "distant stars must not acquire camera parallax"
                );
            }
        }
    }
}
