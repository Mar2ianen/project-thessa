//! Persistent portable spectral sea. GPU FFTs and slope-moment mips are
//! published once per changed simulation sample, never per camera update.
use crate::{Backend, BackendError};
use thessa_render_core::ocean::{LAYERS, PERIODS_M, SeaState, Spectrum};

#[derive(Debug, Clone)]
pub struct OceanBinding {
    pub(crate) frame: wgpu::Buffer,
    pub(crate) field: wgpu::TextureView,
    pub(crate) sampler: wgpu::Sampler,
}
impl OceanBinding {
    pub(crate) fn entries(&self) -> [wgpu::BindGroupEntry<'_>; 3] {
        [
            wgpu::BindGroupEntry {
                binding: 0,
                resource: self.frame.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&self.field),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&self.sampler),
            },
        ]
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct OceanStats {
    pub evaluations: u64,
    pub fft_passes: u32,
    pub size: u32,
    pub storage_bytes: u64,
    pub height_bound_m: f64,
    /// Generated-patch selection count; conforming candidate count is an
    /// explicit GPU extent diagnostic, not read back in ordinary frames.
    pub deformed_patches: u32,
    pub geometry_bytes: u64,
    pub moment_scale: f32,
}

pub(crate) struct OceanGeometry {
    pub original: wgpu::Buffer,
    pub slots: wgpu::Buffer,
    pub params: wgpu::Buffer,
    pub pipeline: wgpu::ComputePipeline,
    pub bind: wgpu::BindGroup,
    pub field_bind: wgpu::BindGroup,
    pub selected: Vec<(usize, u64)>,
    pub capacity: usize,
}

impl OceanGeometry {
    pub fn new(
        backend: &Backend,
        capacity: usize,
        vertices: &wgpu::Buffer,
        frames: &wgpu::Buffer,
        materials: &crate::material::MaterialArray,
        field: &OceanField,
    ) -> Self {
        let device = &backend.device;
        let make = |label, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        let original = make(
            "ocean-base-near-slots",
            capacity as u64 * 1089 * 32,
            wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        );
        let slots = make(
            "ocean-near-slot-list",
            capacity as u64 * 4,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        );
        let params = make(
            "ocean-deformation-params",
            16,
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let material_kernel = geometry_material_shader(7);
        let code = format!(
            "{}\n{}\n{}\n{}\n{}",
            thessa_rcbt_core::shaders::PRECISION_WGSL,
            thessa_rcbt_core::shaders::TILE_FRAME_WGSL,
            material_kernel,
            shader(1),
            include_str!("ocean_deform.wgsl")
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ocean-deform"),
            source: wgpu::ShaderSource::Wgsl(code.into()),
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("ocean-deform"),
            layout: None,
            module: &module,
            entry_point: Some("deform_ocean"),
            compilation_options: Default::default(),
            cache: None,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ocean-deform"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: original.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: vertices.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: frames.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: slots.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: wgpu::BindingResource::TextureView(&materials.view),
                },
                wgpu::BindGroupEntry {
                    binding: 6,
                    resource: wgpu::BindingResource::Sampler(&materials.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 7,
                    resource: materials.directory.as_entire_binding(),
                },
            ],
        });
        let field_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ocean-deform-field"),
            layout: &pipeline.get_bind_group_layout(1),
            entries: &field.binding.entries(),
        });
        Self {
            original,
            slots,
            params,
            pipeline,
            bind,
            field_bind,
            selected: Vec::new(),
            capacity,
        }
    }
}

pub struct OceanField {
    spectrum: Spectrum,
    coefficients: wgpu::Buffer,
    params: wgpu::Buffer,
    pipelines: [wgpu::ComputePipeline; 3],
    bind: wgpu::BindGroup,
    mip_pipeline: wgpu::ComputePipeline,
    mip_binds: Vec<wgpu::BindGroup>,
    texture: wgpu::Texture,
    binding: OceanBinding,
    epoch_s: Option<f64>,
    time_s: Option<f64>,
    frame_key: Option<([f64; 3], f64)>,
    stats: OceanStats,
}

pub fn shader(group: u32) -> String {
    include_str!("ocean_sample.wgsl").replace("@group(2)", &format!("@group({group})"))
}

/// Shared compute variant of the canonical material sampler (no implicit
/// derivatives in a geometry pass). Both geometry consumers use the same code.
pub(crate) fn geometry_material_shader(directory_binding: u32) -> String {
    thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
        .replace("@binding(16)", &format!("@binding({directory_binding})"))
        .replace("textureSampleGrad(material_texture, material_sampler, uv, i32(layer),\n                dx * extent * (125.0 / 128.0), dy * extent * (125.0 / 128.0))", "textureSampleLevel(material_texture, material_sampler, uv, i32(layer), 0.0)")
}

impl OceanField {
    pub fn new(backend: &Backend, state: SeaState, size: u32) -> Result<Self, BackendError> {
        let spectrum = Spectrum::new(state, size as usize)
            .ok_or_else(|| BackendError::Device("invalid ocean sea state/size".into()))?;
        let slope_bound = spectrum
            .modes
            .chunks_exact((size * size) as usize)
            .map(|modes| {
                modes
                    .iter()
                    .map(|m| {
                        2.0 * m.initial[0].hypot(m.initial[1])
                            * m.wave_number[0].hypot(m.wave_number[1])
                    })
                    .sum::<f64>()
            })
            .fold(0.0_f64, f64::max);
        if slope_bound > 32_000.0 {
            return Err(BackendError::Device(
                "ocean slopes exceed portable half-float range; reduce significant height".into(),
            ));
        }
        // Keep small moments normal in f16, but guarantee that even the
        // coefficient-sum slope envelope cannot overflow f16 at steep inputs.
        let moment_scale = (16_000.0 / slope_bound.powi(2)).min(1024.0) as f32;
        let device = &backend.device;
        let count = u64::from(size).pow(2) * LAYERS as u64;
        let buffer = |label, bytes, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: bytes,
                usage,
                mapped_at_creation: false,
            })
        };
        let coefficients = buffer(
            "ocean-coefficients",
            count * 32,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        );
        let evolved = buffer("ocean-evolved", count * 24, wgpu::BufferUsages::STORAGE);
        let rows = buffer("ocean-rows", count * 24, wgpu::BufferUsages::STORAGE);
        let params = buffer(
            "ocean-time",
            16,
            wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        );
        let levels = size.ilog2() + 1;
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("ocean-height-slopes-moments"),
            size: wgpu::Extent3d {
                width: size,
                height: size,
                depth_or_array_layers: LAYERS as u32,
            },
            mip_level_count: levels,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba16Float,
            usage: wgpu::TextureUsages::STORAGE_BINDING
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ocean-ifft"),
            source: wgpu::ShaderSource::Wgsl(include_str!("ocean_fft.wgsl").into()),
        });
        // All entry points share an explicit layout; auto layouts would omit
        // resources unused by a particular FFT stage.
        let mut entries = Vec::new();
        for binding in 0..5 {
            entries.push(wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                count: None,
                ty: match binding {
                    3 => wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format: wgpu::TextureFormat::Rgba16Float,
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                    },
                    _ => wgpu::BindingType::Buffer {
                        ty: if binding == 4 {
                            wgpu::BufferBindingType::Uniform
                        } else {
                            wgpu::BufferBindingType::Storage {
                                read_only: binding == 0,
                            }
                        },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                },
            });
        }
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("ocean-fft-layout"),
            entries: &entries,
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipelines = ["evolve", "fft_rows", "fft_columns"].map(|entry| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry),
                compilation_options: Default::default(),
                cache: None,
            })
        });
        let mip_view = |level| {
            texture.create_view(&wgpu::TextureViewDescriptor {
                base_mip_level: level,
                mip_level_count: Some(1),
                ..Default::default()
            })
        };
        let output = mip_view(0);
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("ocean-fft"),
            layout: &layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: coefficients.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: evolved.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: rows.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::TextureView(&output),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: params.as_entire_binding(),
                },
            ],
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("ocean-mips"),
            source: wgpu::ShaderSource::Wgsl(include_str!("ocean_mips.wgsl").into()),
        });
        let mip_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("ocean-mips"),
            layout: None,
            module: &module,
            entry_point: Some("reduce"),
            compilation_options: Default::default(),
            cache: None,
        });
        let mip_binds = (1..levels)
            .map(|level| {
                device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("ocean-mip"),
                    layout: &mip_pipeline.get_bind_group_layout(0),
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: wgpu::BindingResource::TextureView(&mip_view(level - 1)),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::TextureView(&mip_view(level)),
                        },
                    ],
                })
            })
            .collect();
        let binding = OceanBinding {
            frame: buffer(
                "ocean-surface-frame",
                96,
                wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            ),
            field: texture.create_view(&Default::default()),
            sampler: device.create_sampler(&wgpu::SamplerDescriptor {
                address_mode_u: wgpu::AddressMode::Repeat,
                address_mode_v: wgpu::AddressMode::Repeat,
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                mipmap_filter: wgpu::MipmapFilterMode::Linear,
                ..Default::default()
            }),
        };
        let stats = OceanStats {
            moment_scale,
            size,
            height_bound_m: spectrum.height_bound_m,
            storage_bytes: count * 80
                + (0..levels)
                    .map(|l| u64::from(size >> l).pow(2) * LAYERS as u64 * 8)
                    .sum::<u64>()
                + 112,
            ..Default::default()
        };
        Ok(Self {
            spectrum,
            coefficients,
            params,
            pipelines,
            bind,
            mip_pipeline,
            mip_binds,
            texture,
            binding,
            epoch_s: None,
            time_s: None,
            frame_key: None,
            stats,
        })
    }
    pub fn binding(&self) -> &OceanBinding {
        &self.binding
    }
    pub fn stats(&self) -> OceanStats {
        self.stats
    }
    pub fn spectrum(&self) -> &Spectrum {
        &self.spectrum
    }
    pub fn texture(&self) -> &wgpu::Texture {
        &self.texture
    }

    pub fn update_frame(
        &mut self,
        queue: &wgpu::Queue,
        camera: glam::DVec3,
        radius_m: f64,
    ) -> Result<(), BackendError> {
        if !camera.is_finite()
            || !radius_m.is_finite()
            || radius_m <= 0.0
            || !(radius_m as f32).is_finite()
        {
            return Err(BackendError::Device(
                "invalid ocean body/camera frame".into(),
            ));
        }
        let key = (camera.to_array(), radius_m);
        if self.frame_key == Some(key) {
            return Ok(());
        }
        let mut words = [0.0_f32; 24];
        for (band, period) in PERIODS_M.into_iter().enumerate() {
            words[band * 4..band * 4 + 3]
                .copy_from_slice(&camera.to_array().map(|p| p.rem_euclid(period) as f32));
        }
        words[12..15].copy_from_slice(&PERIODS_M.map(|v| v as f32));
        words[15] = self.stats.size as f32;
        words[16..19].copy_from_slice(&self.spectrum.unresolved_slope_variance.map(|v| v as f32));
        words[20..24].copy_from_slice(&[
            ((1.333_f32 - 1.0) / (1.333 + 1.0)).powi(2),
            0.05,
            radius_m as f32,
            self.stats.moment_scale,
        ]);
        queue.write_buffer(
            &self.binding.frame,
            0,
            words.map(f32::to_le_bytes).as_flattened(),
        );
        self.frame_key = Some(key);
        Ok(())
    }

    /// Returns false for a paused/repeated sample, allocating/submitting nothing.
    pub fn update(&mut self, backend: &Backend, time_s: f64) -> Result<bool, BackendError> {
        self.update_profiled(backend, time_s, None)
    }

    /// Optional four-mark hardware diagnostic: evolution, FFT, mip reduction.
    /// Caller supplies `GpuIntervals::new(backend,4)`, completes the submission
    /// and reads it. Ordinary updates allocate no timestamp resources.
    pub fn update_profiled(
        &mut self,
        backend: &Backend,
        time_s: f64,
        timing: Option<&crate::timing::GpuIntervals>,
    ) -> Result<bool, BackendError> {
        self.stats.fft_passes = 0;
        if !time_s.is_finite() {
            return Err(BackendError::Device(
                "nonfinite ocean simulation sample".into(),
            ));
        }
        if self.time_s == Some(time_s) {
            return Ok(false);
        }
        if self
            .epoch_s
            .is_none_or(|epoch| (time_s - epoch).abs() > 60.0)
        {
            let scale = f64::from(self.stats.size).powi(2);
            let data = self
                .spectrum
                .modes
                .iter()
                .flat_map(|m| {
                    [
                        (m.initial[0] * scale) as f32,
                        (m.initial[1] * scale) as f32,
                        (m.opposite[0] * scale) as f32,
                        (m.opposite[1] * scale) as f32,
                        m.wave_number[0] as f32,
                        m.wave_number[1] as f32,
                        m.omega as f32,
                        m.phase(time_s) as f32,
                    ]
                })
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<_>>();
            backend.queue.write_buffer(&self.coefficients, 0, &data);
            self.epoch_s = Some(time_s);
        }
        let data = [
            self.stats.size,
            LAYERS as u32,
            ((time_s - self.epoch_s.unwrap()) as f32).to_bits(),
            self.stats.moment_scale.to_bits(),
        ]
        .map(u32::to_le_bytes);
        backend
            .queue
            .write_buffer(&self.params, 0, data.as_flattened());
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if let Some(t) = timing {
            t.mark(&mut encoder, 0);
        }
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_bind_group(0, &self.bind, &[]);
            pass.set_pipeline(&self.pipelines[0]);
            pass.dispatch_workgroups((self.stats.size.pow(2) * LAYERS as u32).div_ceil(64), 1, 1);
        }
        if let Some(t) = timing {
            t.mark(&mut encoder, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_bind_group(0, &self.bind, &[]);
            for pipeline in &self.pipelines[1..] {
                pass.set_pipeline(pipeline);
                pass.dispatch_workgroups(self.stats.size, LAYERS as u32, 1);
            }
        }
        if let Some(t) = timing {
            t.mark(&mut encoder, 2);
        }
        for (i, bind) in self.mip_binds.iter().enumerate() {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.mip_pipeline);
            pass.set_bind_group(0, bind, &[]);
            let size = self.stats.size >> (i + 1);
            pass.dispatch_workgroups(size.div_ceil(8), size.div_ceil(8), LAYERS as u32);
        }
        if let Some(t) = timing {
            t.mark(&mut encoder, 3);
            t.resolve(&mut encoder);
        }
        backend.queue.submit([encoder.finish()]);
        self.time_s = Some(time_s);
        self.stats.evaluations += 1;
        self.stats.fft_passes = 3 + self.mip_binds.len() as u32;
        Ok(true)
    }
}
