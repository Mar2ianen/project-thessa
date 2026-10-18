//! Production opt-in consumer of the existing persistent transaction pipeline.
//! Canonical residency remains owned by TerrainRenderer; no second streamer.
use crate::{
    Backend, BackendError,
    conforming_surface::{ConformingSurface, HeightSurfaceSource},
    view::ViewUniformBlock,
};
use thessa_rcbt_wgpu::sphere::SphereGpu;

pub(crate) struct TerrainTopology {
    pub gpu: SphereGpu,
    pub surface: ConformingSurface,
    pub shadow: Option<wgpu::BindGroup>,
    classify: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    policy: wgpu::Buffer,
    waves: u32,
    split: f32,
    view_ready: std::cell::Cell<bool>,
}
impl TerrainTopology {
    pub fn new(
        backend: &Backend,
        settings: &thessa_graphics::ResolvedGraphicsSettings,
        height: &HeightSurfaceSource,
        materials: &crate::material::MaterialBinding,
        ocean: Option<&crate::ocean::OceanField>,
        atlas: Option<&crate::shadows::ShadowAtlas>,
    ) -> Result<Self, BackendError> {
        let config = settings.terrain_topology;
        if !(4096..=131072).contains(&config.capacity)
            || !config.capacity.is_power_of_two()
            || !(1..=4).contains(&config.waves)
        {
            return Err(BackendError::Device(
                "invalid resolved conforming terrain budget".into(),
            ));
        }
        let gpu =
            SphereGpu::with_uniform_depth(&backend.device, &backend.queue, config.capacity, 58, 8)
                .map_err(|e| BackendError::Device(e.into()))?;
        gpu.set_area_policy(&backend.queue, config.split_area_px, config.merge_area_px)
            .map_err(|e| BackendError::Device(e.into()))?;
        let mut surface = ConformingSurface::new(backend, &gpu.pool, height)?;
        surface.set_ocean(backend, settings, ocean, materials)?;
        let policy = backend.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("canonical-topology-view"),
            size: 80,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let module = backend
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("canonical-topology-classification"),
                source: wgpu::ShaderSource::Wgsl(include_str!("terrain_topology.wgsl").into()),
            });
        let classify = backend
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some("canonical-topology-classification"),
                layout: None,
                module: &module,
                entry_point: Some("classify"),
                compilation_options: Default::default(),
                cache: None,
            });
        let bind = backend
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("canonical-topology-classification"),
                layout: &classify.get_bind_group_layout(0),
                entries: &[&surface.vertices, &gpu.areas, &policy]
                    .into_iter()
                    .enumerate()
                    .map(|(binding, b)| wgpu::BindGroupEntry {
                        binding: binding as u32,
                        resource: b.as_entire_binding(),
                    })
                    .collect::<Vec<_>>(),
            });
        let shadow = atlas.map(|atlas| {
            backend
                .device
                .create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("canonical-topology-shadow"),
                    layout: &atlas.conforming_layout,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: surface.vertices.as_entire_binding(),
                    }],
                })
        });
        Ok(Self {
            gpu,
            surface,
            shadow,
            classify,
            bind,
            policy,
            waves: config.waves,
            split: config.split_area_px,
            view_ready: std::cell::Cell::new(false),
        })
    }
    pub fn update_view(
        &self,
        backend: &Backend,
        block: &ViewUniformBlock,
        camera: [f64; 3],
        extent: [u32; 2],
        height_bound_m: f32,
    ) -> Result<(), BackendError> {
        if extent.contains(&0) || !height_bound_m.is_finite() || height_bound_m < 0.0 {
            return Err(BackendError::Device(
                "invalid conforming viewport/height bound".into(),
            ));
        }
        self.surface.update_camera(backend, camera)?;
        let data = block
            .clip_from_world
            .to_cols_array()
            .into_iter()
            .chain([
                extent[0] as f32,
                extent[1] as f32,
                height_bound_m,
                self.split,
            ])
            .flat_map(f32::to_le_bytes)
            .collect::<Vec<_>>();
        backend.queue.write_buffer(&self.policy, 0, &data);
        self.view_ready.set(true);
        Ok(())
    }
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        if !self.view_ready.get() {
            return;
        }
        for _ in 0..self.waves {
            self.surface.encode_datum(encoder);
            self.encode_classification(encoder);
            self.gpu.encode_batch_plan_from_areas(encoder);
            self.gpu.encode_batch_commit(encoder, true);
            self.gpu.encode_batch_publish(encoder);
        }
        self.surface.encode(encoder);
    }
    fn encode_classification(&self, encoder: &mut wgpu::CommandEncoder) {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&self.classify);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.dispatch_workgroups(self.surface.slot_count().div_ceil(64), 1, 1);
    }
    pub fn storage_bytes(&self) -> u64 {
        self.gpu.storage_bytes() + self.surface.storage_bytes() + self.policy.size()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn classifier_is_portable() {
        let s = include_str!("terrain_topology.wgsl");
        let m =
            naga::front::wgsl::parse_str(s).unwrap_or_else(|e| panic!("{}", e.emit_to_string(s)));
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::empty(),
        )
        .validate(&m)
        .unwrap();
    }

    #[test]
    #[ignore = "hardware: actual emitted Earth-scale triangle areas vs independent f64 projection"]
    fn emitted_area_classifier_matches_f64_projection_and_relief_is_monotone() {
        use crate::terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer};
        use glam::{DVec3, DVec4};
        use thessa_rcbt_core::{
            HeightPage,
            tile_anchor::{TileAnchor, TileKey},
        };
        let backend = Backend::bring_up().unwrap();
        let radius = 6371000.0;
        let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let patches = (0..6)
            .map(|face| TerrainPatch {
                anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
                page: &page,
            })
            .collect::<Vec<_>>();
        let mut owner = TerrainRenderer::with_capacity(
            &backend,
            &patches,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 6,
                height_grid_size: 2,
                material_layers: 1,
            },
        )
        .unwrap();
        let height = owner.height_surface_source(&backend).unwrap();
        let settings = thessa_graphics::ResolvedGraphicsSettings {
            terrain_topology: thessa_graphics::ResolvedTerrainTopology {
                capacity: 4096,
                waves: 1,
                split_area_px: 256.0,
                merge_area_px: 32.0,
            },
            ..Default::default()
        };
        let topology = TerrainTopology::new(
            &backend,
            &settings,
            &height,
            &owner.material_binding(),
            None,
            None,
        )
        .unwrap();
        let eye = DVec3::new(0.0, 0.0, radius * 2.5);
        let view = crate::view::ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
                .inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.4, 1.0, 1.0, 1e8),
            world_position_m: eye,
            exposure: 1.0,
        };
        let block = ViewUniformBlock::new(&view).unwrap();
        let read = |source: &wgpu::Buffer| {
            let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size: source.size(),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            encoder.copy_buffer_to_buffer(source, 0, &staging, 0, source.size());
            backend.queue.submit([encoder.finish()]);
            crate::terrain::map_buffer(&backend, &staging).unwrap()
        };
        let classify = |bound| {
            topology
                .update_view(&backend, &block, eye.to_array(), [257, 257], bound)
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            topology.surface.encode_datum(&mut encoder);
            topology.encode_classification(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            read(&topology.gpu.areas)
                .as_chunks::<4>()
                .0
                .iter()
                .map(|p| f32::from_le_bytes(*p))
                .collect::<Vec<_>>()
        };
        let areas = classify(0.0);
        let vertices = read(&topology.surface.vertices);
        let mut max_error = 0.0f64;
        let mut live = 0;
        for (slot, &gpu) in areas.iter().enumerate() {
            let mut positions = [DVec4::ZERO; 3];
            let mut valid = false;
            for (i, p) in positions.iter_mut().enumerate() {
                let offset = (slot * 3 + i) * 48;
                let f = |at| {
                    f64::from(f32::from_le_bytes(
                        vertices[offset + at..offset + at + 4].try_into().unwrap(),
                    ))
                };
                *p = DVec4::new(f(0), f(4), f(8), 1.0);
                valid |= f(16) != 0.0 || f(20) != 0.0 || f(24) != 0.0;
            }
            if !valid {
                assert_eq!(gpu, 0.0);
                continue;
            }
            live += 1;
            let projected = positions.map(|p| {
                let q = block.clip_from_world.as_dmat4() * p;
                q.truncate().truncate() / q.w * 128.5
            });
            let expected = (projected[1] - projected[0])
                .perp_dot(projected[2] - projected[0])
                .abs()
                * 0.5;
            max_error = max_error.max((expected - f64::from(gpu)).abs());
        }
        let envelope = 8.0 * f64::from(f32::EPSILON) * 257.0 * 257.0;
        assert_eq!(live, 3072);
        assert!(
            max_error <= envelope,
            "area error {max_error} > {envelope} px^2"
        );
        let relief = classify(1000000.0);
        assert!(areas.iter().zip(&relief).all(|(a, b)| a <= b));
        assert!(areas.iter().zip(&relief).any(|(a, b)| a < b));
        assert_eq!(
            topology.surface.publication().revision,
            0,
            "classification scratch must not publish intermediate geometry to consumers"
        );
        println!(
            "canonical_area_oracle adapter={} triangles={live} max_error_px2={max_error} envelope_px2={envelope}; f64 oracle over emitted Earth-scale positions, no absolute-f32 planetary projection",
            backend.adapter_name
        );
        // Replay the new actionable-area request selection through the same CPU
        // oracle, including far decimation, a close rebase and pool exhaustion.
        use thessa_rcbt_core::{
            Node,
            conforming::{Record, SphereTopology},
        };
        use thessa_rcbt_wgpu::sphere::PLAN_WORDS;
        let mut cpu = SphereTopology::uniform(4096, 58, 8).unwrap();
        let mut split = 0;
        let mut merged = 0;
        let logical = |records: &[Record]| {
            records
                .iter()
                .filter(|r| r.address[0] != 0)
                .map(|r| {
                    let id = |r: &Record| u64::from(r.address[0]) | (u64::from(r.address[1]) << 32);
                    (
                        id(r),
                        r.neighbors[..3]
                            .iter()
                            .map(|&slot| {
                                if slot == u32::MAX {
                                    0
                                } else {
                                    id(&records[slot as usize])
                                }
                            })
                            .collect::<Vec<_>>(),
                    )
                })
                .collect::<std::collections::BTreeMap<_, _>>()
        };
        for frame in 0..80 {
            let eye = if frame < 30 {
                eye
            } else {
                DVec3::new(radius + 100.0, 0.0, 0.0)
            };
            let mut view = view;
            view.world_position_m = eye;
            view.world_from_view =
                glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y).inverse();
            topology
                .update_view(
                    &backend,
                    &ViewUniformBlock::new(&view).unwrap(),
                    eye.to_array(),
                    [257, 257],
                    0.0,
                )
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            topology.encode(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let words = read(&topology.gpu.plans)
                .as_chunks::<4>()
                .0
                .iter()
                .map(|p| u32::from_le_bytes(*p))
                .collect::<Vec<_>>();
            for plan in words[4..]
                .as_chunks::<PLAN_WORDS>()
                .0
                .iter()
                .take(words[1] as usize)
            {
                if plan[5] != 1 {
                    continue;
                }
                let node =
                    Node::from_heap_id(u64::from(plan[6]) | (u64::from(plan[7]) << 32)).unwrap();
                if plan[0] == 0 {
                    assert_eq!(cpu.split(node).unwrap(), plan[3] as usize);
                    split += 1;
                } else {
                    cpu.merge(node).unwrap();
                    merged += 1;
                }
            }
            let words = read(&topology.gpu.pool)
                .as_chunks::<4>()
                .0
                .iter()
                .map(|p| u32::from_le_bytes(*p))
                .collect::<Vec<_>>();
            let records = words
                .as_chunks::<8>()
                .0
                .iter()
                .map(|w| Record {
                    address: w[..4].try_into().unwrap(),
                    neighbors: w[4..].try_into().unwrap(),
                })
                .collect::<Vec<_>>();
            cpu.validate().unwrap();
            assert_eq!(
                logical(&records),
                logical(&cpu.records()),
                "canonical-area transaction replay frame {frame}"
            );
        }
        assert!(
            split > 0 && merged > 0,
            "actionable requests must actually refine and decimate: {split}/{merged}"
        );
        println!(
            "canonical_area_request_replay frames=80 splits={split} merges={merged} capacity=4096; independent CPU topology/neighbor oracle after every wave, far-to-close Earth-scale camera rebase"
        );
    }
}
