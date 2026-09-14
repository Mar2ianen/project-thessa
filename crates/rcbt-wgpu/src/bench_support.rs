//! Shared harness for the `rcbt-wgpu` benches. Bench-only tooling: small
//! GPU helpers (context, layouts, pipelines, readback) used by `gpu_cbt`
//! and `crossover`. Workload runners stay in the benches themselves.

/// Shared GPU objects for one bench run.
pub struct GpuCtx<'a> {
    pub device: &'a wgpu::Device,
    pub queue: &'a wgpu::Queue,
    pub apply_pipe: &'a wgpu::ComputePipeline,
    pub decode_pipe: &'a wgpu::ComputePipeline,
    pub apply_layout: &'a wgpu::BindGroupLayout,
    pub decode_layout: &'a wgpu::BindGroupLayout,
}

/// Wall-clock breakdown of one GPU replay. `submit_ms` covers enqueue only;
/// real GPU completion lands in `readback_ms` (blocking poll), and
/// `write_buffer` only stages — so read `submit` vs `readback` as
/// host-overhead vs device+sync, not as kernel time. Kernel-only time needs
/// timestamp queries (future work, see docs/22).
#[derive(Default, Debug)]
pub struct GpuPhases {
    pub alloc_ms: f64,
    pub upload_ms: f64,
    pub submit_ms: f64,
    pub readback_ms: f64,
}

impl GpuPhases {
    pub fn total_ms(&self) -> f64 {
        self.alloc_ms + self.upload_ms + self.submit_ms + self.readback_ms
    }
}

pub const WORKGROUP: u32 = 64;

pub fn groups(n: usize) -> u32 {
    ((n as u32).div_ceil(WORKGROUP)).max(1)
}

pub fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 4);
    for w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

fn layout_entry(binding: u32, read_only: bool, uniform: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: if uniform {
                wgpu::BufferBindingType::Uniform
            } else {
                wgpu::BufferBindingType::Storage { read_only }
            },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

/// Layout for apply-style kernels: uniform + active + sums + read-only ops.
pub fn create_apply_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("rcbt-apply-layout"),
        entries: &[
            layout_entry(0, true, true),
            layout_entry(1, false, false),
            layout_entry(2, false, false),
            layout_entry(3, true, false),
        ],
    })
}

/// Layout for decode-style kernels: uniform + active + sums + read-write out.
pub fn create_decode_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("rcbt-decode-layout"),
        entries: &[
            layout_entry(0, true, true),
            layout_entry(1, false, false),
            layout_entry(2, false, false),
            layout_entry(3, false, false),
        ],
    })
}

pub fn create_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    src: &str,
    entry_point: &str,
) -> wgpu::ComputePipeline {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("rcbt-bench-shader"),
        source: wgpu::ShaderSource::Wgsl(src.into()),
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("rcbt-bench-pl"),
        bind_group_layouts: &[Some(layout)],
        immediate_size: 0,
    });
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(entry_point),
        layout: Some(&pipeline_layout),
        module: &module,
        entry_point: Some(entry_point),
        compilation_options: Default::default(),
        cache: None,
    })
}

pub fn readback_pairs(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    src: &wgpu::Buffer,
    count: usize,
) -> Vec<(u64, u8)> {
    readback_pairs_reused(device, queue, src, count, None).0
}

/// Readback with an optional caller-owned reusable staging buffer (sized in
/// bytes beforehand). Returns the pairs plus whether the reusable buffer was
/// used — on unified-memory hardware the copy behind this is a plain memcpy.
pub fn readback_pairs_reused(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    src: &wgpu::Buffer,
    count: usize,
    reusable: Option<&wgpu::Buffer>,
) -> (Vec<(u64, u8)>, bool) {
    let bytes = count * 8;
    let owned;
    let staging = match reusable {
        Some(buf) => buf,
        None => {
            owned = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rcbt-readback"),
                size: bytes.max(8) as u64,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            &owned
        }
    };
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("rcbt-readback-copy"),
    });
    if bytes > 0 {
        encoder.copy_buffer_to_buffer(src, 0, staging, 0, bytes as u64);
    }
    queue.submit(Some(encoder.finish()));
    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .expect("device poll");
    rx.recv().expect("map callback").expect("map success");
    let out = {
        let view = slice.get_mapped_range();
        let (_, words, _) = unsafe { view.align_to::<u32>() };
        words
            .as_chunks::<2>()
            .0
            .iter()
            .map(|w| (w[0] as u64, w[1] as u8))
            .collect()
    };
    staging.unmap();
    (out, reusable.is_some())
}
