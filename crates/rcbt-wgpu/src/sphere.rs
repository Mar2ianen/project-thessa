//! Correctness-first persistent GPU sphere experiment; not a production backend.
//! Single-writer conforming updates + existing OCBT packed allocator ABI. CPU
//! publishes camera/policy or a logical request, never rebuilt triangle lists.
use thessa_rcbt_core::conforming::{Record, SphereTopology};
use wgpu::util::DeviceExt;

pub const GEOMETRY_WGSL: &str = include_str!("sphere_geometry.wgsl");

pub struct SphereGpu {
    pub pool: wgpu::Buffer,
    pub indices: wgpu::Buffer,
    pub state: wgpu::Buffer,
    pub params: wgpu::Buffer,
    pub ranks: wgpu::Buffer,
    pub bits: wgpu::Buffer,
    areas: wgpu::Buffer,
    classify: wgpu::ComputePipeline,
    update: wgpu::ComputePipeline,
    index: wgpu::ComputePipeline,
    bind: wgpu::BindGroup,
    capacity: u32,
    max_depth: u8,
}

impl SphereGpu {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        capacity: u32,
        max_depth: u8,
    ) -> Result<Self, &'static str> {
        if capacity > 131072 {
            return Err("prototype pool exceeds existing OCBT-128K layout");
        }
        let initial = SphereTopology::new(capacity as usize, max_depth)?;
        let records = initial.records();
        let mut allocator = crate::ocbt::OcbtPoolMirror::new(17).map_err(|_| "OCBT init")?;
        for (slot, record) in records.iter().enumerate() {
            if record.address[0] != 0 {
                allocator.set_bit(slot, true).map_err(|_| "OCBT bit init")?;
            }
        }
        let make = |label, data: &[u8], usage| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(label),
                contents: data,
                usage: usage | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
            })
        };
        let pool = make(
            "sphere-bisectors",
            &encode_records(&records),
            wgpu::BufferUsages::STORAGE,
        );
        let ranks = make(
            "sphere-ocbt-ranks",
            &allocator.tree_bytes(),
            wgpu::BufferUsages::STORAGE,
        );
        let bits = make(
            "sphere-ocbt-bits",
            &allocator.bitfield_bytes(),
            wgpu::BufferUsages::STORAGE,
        );
        let mut initial_indices = vec![0; capacity as usize];
        for (i, index) in initial_indices
            .iter_mut()
            .take(allocator.bit_count())
            .enumerate()
        {
            *index = allocator.decode_bit(i).map_err(|_| "initial rank decode")? as u32;
        }
        let indices = make(
            "sphere-active-indices",
            &words(&initial_indices),
            wgpu::BufferUsages::STORAGE,
        );
        let areas = make(
            "sphere-classified-areas",
            &vec![0; capacity as usize * 4],
            wgpu::BufferUsages::STORAGE,
        );
        let state = make(
            "sphere-counters-and-indirect",
            &[0; 48],
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::INDIRECT,
        );
        let params = make("sphere-policy", &[0; 128], wgpu::BufferUsages::UNIFORM);
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("sphere-update-layout"),
            entries: &(0..7)
                .map(|binding| wgpu::BindGroupLayoutEntry {
                    binding,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: if binding == 0 {
                            wgpu::BufferBindingType::Uniform
                        } else {
                            wgpu::BufferBindingType::Storage { read_only: false }
                        },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                })
                .collect::<Vec<_>>(),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let source = format!(
            "{}\n{}\n{}",
            GEOMETRY_WGSL,
            include_str!("ocbt_pool.wgsl"),
            include_str!("sphere_update.wgsl")
        );
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("independent-conforming-sphere"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let pipeline = |name| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(name),
                layout: Some(&pipeline_layout),
                module: &shader,
                entry_point: Some(name),
                compilation_options: Default::default(),
                cache: None,
            })
        };
        let update = pipeline("update");
        let index = pipeline("index_pool");
        let classify = pipeline("classify");
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[&params, &pool, &ranks, &bits, &indices, &state, &areas]
                .into_iter()
                .enumerate()
                .map(|(binding, buffer)| wgpu::BindGroupEntry {
                    binding: binding as u32,
                    resource: buffer.as_entire_binding(),
                })
                .collect::<Vec<_>>(),
        });
        let gpu = Self {
            pool,
            indices,
            state,
            params,
            ranks,
            bits,
            areas,
            classify,
            update,
            index,
            bind,
            capacity,
            max_depth,
        };
        gpu.set_request(queue, 2, 0);
        Ok(gpu)
    }

    /// Manual oracle request: 0=split address, 1=merge parent, 2=no mutation.
    pub fn set_request(&self, queue: &wgpu::Queue, kind: u32, id: u64) {
        queue.write_buffer(
            &self.params,
            0,
            &words(&[
                self.capacity,
                u32::from(self.max_depth),
                0,
                kind,
                id as u32,
                (id >> 32) as u32,
                0,
                0,
            ]),
        );
    }

    /// Unit sphere only. Callers supply an ordinary world→clip matrix and
    /// outside-sphere camera. Error/height/planet precision are NOT implemented.
    pub fn set_view(
        &self,
        queue: &wgpu::Queue,
        clip: [f32; 16],
        camera: [f32; 3],
        size: [u32; 2],
        split_area_px: f32,
        merge_area_px: f32,
    ) -> Result<(), &'static str> {
        if !clip.iter().chain(camera.iter()).all(|x| x.is_finite())
            || camera.iter().map(|v| v * v).sum::<f32>() <= 1.0
            || size.contains(&0)
            || !split_area_px.is_finite()
            || !merge_area_px.is_finite()
            || merge_area_px <= 0.0
            || split_area_px <= 4.0 * merge_area_px
        {
            return Err("invalid sphere policy/view");
        }
        let mut data = words(&[self.capacity, u32::from(self.max_depth), 1, 2, 0, 0, 0, 0]);
        data.extend(words(&clip.map(f32::to_bits)));
        data.extend(words(&[
            camera[0].to_bits(),
            camera[1].to_bits(),
            camera[2].to_bits(),
            0,
        ]));
        data.extend(words(&[
            split_area_px.to_bits(),
            merge_area_px.to_bits(),
            (size[0] as f32).to_bits(),
            (size[1] as f32).to_bits(),
        ]));
        queue.write_buffer(&self.params, 0, &data);
        Ok(())
    }

    /// No CPU readback, topology upload, full reduction or pool recreation.
    /// Three dispatches: parallel classification, one bounded serial topology
    /// transaction, parallel rank decode. Not large_cbt's parallel mutation.
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder) {
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.classify);
            pass.set_bind_group(0, &self.bind, &[]);
            pass.dispatch_workgroups(self.capacity.div_ceil(64), 1, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.update);
            pass.set_bind_group(0, &self.bind, &[]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&Default::default());
            pass.set_pipeline(&self.index);
            pass.set_bind_group(0, &self.bind, &[]);
            pass.dispatch_workgroups(self.capacity.div_ceil(64), 1, 1);
        }
    }

    pub fn storage_bytes(&self) -> u64 {
        [
            &self.pool,
            &self.ranks,
            &self.bits,
            &self.indices,
            &self.state,
            &self.params,
            &self.areas,
        ]
        .iter()
        .map(|b| b.size())
        .sum()
    }
}

pub fn encode_records(records: &[Record]) -> Vec<u8> {
    records
        .iter()
        .flat_map(|r| r.address.into_iter().chain(r.neighbors))
        .flat_map(u32::to_le_bytes)
        .collect()
}
fn words(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

#[cfg(test)]
mod tests {
    #[test]
    fn independent_sphere_shader_validates_without_hardware() {
        let source = format!(
            "{}\n{}\n{}",
            super::GEOMETRY_WGSL,
            include_str!("ocbt_pool.wgsl"),
            include_str!("sphere_update.wgsl")
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
}
