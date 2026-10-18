//! Correctness-first persistent GPU sphere experiment; not a production backend.
//! Conflict-filtered parallel transactions + existing OCBT packed allocator ABI. CPU
//! publishes camera/policy or a logical request, never rebuilt triangle lists.
use thessa_rcbt_core::conforming::{Record, SphereTopology};
use wgpu::util::DeviceExt;

pub const GEOMETRY_WGSL: &str = include_str!("sphere_geometry.wgsl");
pub const BATCH_REQUESTS: u32 = 256;
pub const PLAN_WORDS: usize = 200;

fn shader_source() -> String {
    format!(
        "{}\n{}\n{}\n{}\nconst REQUESTS: u32 = {BATCH_REQUESTS}u;",
        GEOMETRY_WGSL,
        include_str!("ocbt_pool.wgsl"),
        include_str!("sphere_update.wgsl"),
        include_str!("sphere_batch.wgsl")
    )
}

pub struct SphereGpu {
    pub pool: wgpu::Buffer,
    pub indices: wgpu::Buffer,
    pub state: wgpu::Buffer,
    pub params: wgpu::Buffer,
    pub ranks: wgpu::Buffer,
    pub bits: wgpu::Buffer,
    /// Explicit diagnostic readback: four header words then PLAN_WORDS per
    /// request. Kind/address/status identify each committed CPU replay operation.
    pub plans: wgpu::Buffer,
    claims: wgpu::Buffer,
    batch: [wgpu::ComputePipeline; 7],
    serial_commit: wgpu::ComputePipeline,
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
        Self::with_uniform_depth(device, queue, capacity, max_depth, 0)
    }

    /// One-time uniform conforming seed. Not a per-frame CPU geometry path.
    pub fn with_uniform_depth(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        capacity: u32,
        max_depth: u8,
        local_depth: u8,
    ) -> Result<Self, &'static str> {
        if capacity > 131072 {
            return Err("prototype pool exceeds existing OCBT-128K layout");
        }
        let initial = SphereTopology::uniform(capacity as usize, max_depth, local_depth)?;
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
        let plans = make(
            "sphere-transaction-plans",
            &vec![0; (4 + BATCH_REQUESTS as usize * PLAN_WORDS) * 4],
            wgpu::BufferUsages::STORAGE,
        );
        let claims = make(
            "sphere-record-claims",
            &vec![0; capacity as usize * 4],
            wgpu::BufferUsages::STORAGE,
        );
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("sphere-update-layout"),
            entries: &(0..9)
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
        let source = shader_source();
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
        let batch = [
            "clear_claims",
            "plan_transactions",
            "validate_claims",
            "reserve_transactions",
            "allocate_transactions",
            "commit_parallel",
            "finish_transactions",
        ]
        .map(pipeline);
        let serial_commit = pipeline("commit_serial");
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[
                &params, &pool, &ranks, &bits, &indices, &state, &areas, &plans, &claims,
            ]
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
            plans,
            claims,
            batch,
            serial_commit,
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

    /// Diagnostic policy: request splits or diamond merges in each scanned
    /// window, independent of pixel area. Bounded by the same depth/pool rules.
    pub fn set_sweep(&self, queue: &wgpu::Queue, merge: bool) {
        queue.write_buffer(
            &self.params,
            0,
            &words(&[
                self.capacity,
                u32::from(self.max_depth),
                3,
                u32::from(merge),
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

    /// Parallel planning/claiming, a bounded serial admission prefix, parallel
    /// predecode + commit, then final counters and active-index publication.
    /// Uses `set_view` or diagnostic `set_sweep`; manual addresses use `encode`.
    /// `parallel_commit=false` executes IDENTICAL accepted plans serially for
    /// deterministic byte-oracles and same-workload mutation measurements.
    pub fn encode_batch(&self, encoder: &mut wgpu::CommandEncoder, parallel_commit: bool) {
        self.encode_batch_plan(encoder);
        self.encode_batch_commit(encoder, parallel_commit);
        self.encode_batch_publish(encoder);
    }

    fn dispatch(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        pipeline: &wgpu::ComputePipeline,
        groups: u32,
    ) {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.dispatch_workgroups(groups, 1, 1);
    }

    /// Explicit profiling boundary. Must be followed by commit, then publish,
    /// on this pool before reading/drawing or starting another update.
    pub fn encode_batch_plan(&self, encoder: &mut wgpu::CommandEncoder) {
        self.dispatch(encoder, &self.classify, self.capacity.div_ceil(64));
        for (i, pipeline) in self.batch[..5].iter().enumerate() {
            let groups = match i {
                0 => self.capacity.max(BATCH_REQUESTS).div_ceil(64),
                3 => 1,
                _ => BATCH_REQUESTS.div_ceil(64),
            };
            self.dispatch(encoder, pipeline, groups);
        }
    }

    /// Complete the previously planned/reserved transaction wave.
    pub fn encode_batch_commit(&self, encoder: &mut wgpu::CommandEncoder, parallel: bool) {
        self.dispatch(
            encoder,
            if parallel {
                &self.batch[5]
            } else {
                &self.serial_commit
            },
            if parallel {
                BATCH_REQUESTS.div_ceil(64)
            } else {
                1
            },
        );
    }

    /// Publish counters, cursor, indirect command and the new active indices.
    pub fn encode_batch_publish(&self, encoder: &mut wgpu::CommandEncoder) {
        self.dispatch(encoder, &self.batch[6], 1);
        self.dispatch(encoder, &self.index, self.capacity.div_ceil(64));
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
            &self.plans,
            &self.claims,
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
        let source = super::shader_source();
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
