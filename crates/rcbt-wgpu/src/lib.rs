//! Portable wgpu adapter for the backend-neutral RCBT contracts.
//!
//! The adapter owns all wgpu handles. `rcbt-core` remains usable by a server,
//! a CPU renderer, or another graphics API.
//!
//! [`heap`] is the CPU mirror of the GPU heap layout (pure Rust, shared init
//! and verification code). [`shaders`] holds the real sparse-commit CBT
//! WGSL kernels (`apply_ops` + `decode_all`); the legacy `RCBT_WGSL` touch
//! kernels below stay only as a dispatch bring-up target. [`ocbt`] and the
//! compact core buffers are transport layouts; they do not replace the
//! authoritative CPU topology.

pub mod bench_support;
pub mod heap;
pub mod microstore;
pub mod microstore_sample;
pub mod ocbt;
pub mod shaders;

use std::borrow::Cow;
use std::num::NonZeroU64;
use std::sync::Arc;

use thessa_rcbt_core::{
    BufferBinding, BufferDesc, BufferUsage, CbtBackend, CbtBarrier, CbtBindings, CbtCapabilities,
    CbtKernel, CbtMetrics, DispatchSize, HeightPage, compact::CompactTree,
};

const WORKGROUP_SIZE: u32 = 64;

/// A reference to a storage resource allocated by this adapter.
#[derive(Clone)]
pub struct WgpuBuffer {
    buffer: Arc<wgpu::Buffer>,
    size_bytes: u64,
    writable: bool,
}

impl std::fmt::Debug for WgpuBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WgpuBuffer")
            .field("size_bytes", &self.size_bytes)
            .finish()
    }
}

#[derive(Debug)]
pub struct WgpuPipeline {
    pipeline: wgpu::ComputePipeline,
}

pub struct WgpuCommands {
    encoder: wgpu::CommandEncoder,
    dispatches: u64,
    bytes_touched: u64,
}

#[derive(Debug)]
pub enum WgpuError {
    ZeroSizedBuffer,
    BufferRangeOutOfBounds,
    BufferNotWritable,
    MissingBinding(u32),
    Device(String),
}

impl std::fmt::Display for WgpuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ZeroSizedBuffer => f.write_str("RCBT buffers must have non-zero size"),
            Self::BufferRangeOutOfBounds => f.write_str("RCBT buffer binding exceeds its resource"),
            Self::BufferNotWritable => f.write_str("RCBT buffer was not created for uploads"),
            Self::MissingBinding(slot) => write!(f, "RCBT binding slot {slot} is missing"),
            Self::Device(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for WgpuError {}

/// Portable compute backend. Capability reporting is conservative and based
/// on adapter limits/features, not on vendor names.
pub struct WgpuBackend {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    bind_group_layout: wgpu::BindGroupLayout,
}

impl WgpuBackend {
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("thessa-rcbt-storage-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        Self {
            device,
            queue,
            bind_group_layout,
        }
    }

    pub fn workgroup_size() -> u32 {
        WORKGROUP_SIZE
    }

    pub fn upload(
        &self,
        buffer: &WgpuBuffer,
        offset_bytes: u64,
        data: &[u8],
    ) -> Result<(), WgpuError> {
        if !buffer.writable {
            return Err(WgpuError::BufferNotWritable);
        }
        let end = offset_bytes
            .checked_add(data.len() as u64)
            .ok_or(WgpuError::BufferRangeOutOfBounds)?;
        if end > buffer.size_bytes {
            return Err(WgpuError::BufferRangeOutOfBounds);
        }
        self.queue.write_buffer(&buffer.buffer, offset_bytes, data);
        Ok(())
    }

    /// Upload the stable baked page encoding. The page address and cache
    /// lifetime stay with the terrain adapter; this method only transfers the
    /// immutable payload to a caller-owned GPU buffer.
    pub fn upload_height_page(
        &self,
        buffer: &WgpuBuffer,
        offset_bytes: u64,
        page: &HeightPage,
    ) -> Result<(), WgpuError> {
        self.upload(buffer, offset_bytes, &page.to_bytes())
    }

    /// Upload the exact precision-reduced CBT buffers. The scalar tree stays
    /// authoritative; this is a render/GPU transport operation and writes the
    /// active bitset plus concatenated per-level rank buffer.
    pub fn upload_compact_tree(
        &self,
        active_buffer: &WgpuBuffer,
        sums_buffer: &WgpuBuffer,
        tree: &CompactTree,
    ) -> Result<(), WgpuError> {
        self.upload(active_buffer, 0, &tree.active_buffer_bytes())?;
        self.upload(sums_buffer, 0, &tree.sums_buffer_bytes())
    }

    /// Upload the two raw buffers of the byte-compatible large_cbt OCBT
    /// memory-pool layout.
    pub fn upload_ocbt_pool(
        &self,
        tree_buffer: &WgpuBuffer,
        bitfield_buffer: &WgpuBuffer,
        mirror: &crate::ocbt::OcbtPoolMirror,
    ) -> Result<(), WgpuError> {
        self.upload(tree_buffer, 0, &mirror.tree_bytes())?;
        self.upload(bitfield_buffer, 0, &mirror.bitfield_bytes())
    }

    fn entry_point(kernel: CbtKernel) -> &'static str {
        match kernel {
            CbtKernel::Classify => "classify",
            CbtKernel::Bisect => "bisect",
            CbtKernel::Simplify => "simplify",
            CbtKernel::Reduce => "reduce",
            CbtKernel::CompactLeaves => "compact_leaves",
            CbtKernel::BuildDrawList => "build_draw_list",
            CbtKernel::GenerateVertices => "generate_vertices",
        }
    }
}

impl CbtBackend for WgpuBackend {
    type Buffer = WgpuBuffer;
    type Pipeline = WgpuPipeline;
    type Commands = WgpuCommands;
    type Error = WgpuError;

    fn capabilities(&self) -> CbtCapabilities {
        CbtCapabilities {
            subgroup_size: None,
            subgroup_ballot: false,
            // wgpu's portable feature set does not expose a portable shader
            // u64 storage contract yet; never over-report it from an unrelated
            // feature bit.
            storage_u64: false,
            atomic_u64: false,
            indirect_draw: true,
            indirect_count: false,
            persistent_mapping: false,
            device_address: false,
            cooperative_matrix: false,
        }
    }

    fn create_buffer(&self, desc: BufferDesc) -> Result<Self::Buffer, Self::Error> {
        if desc.size_bytes == 0 {
            return Err(WgpuError::ZeroSizedBuffer);
        }
        let (usage, writable) = match desc.usage {
            BufferUsage::Storage => (
                wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                true,
            ),
            BufferUsage::Uniform => (
                wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                true,
            ),
            BufferUsage::Indirect => (
                wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::STORAGE,
                false,
            ),
            BufferUsage::Readback => (
                wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                false,
            ),
        };
        let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("thessa-rcbt-buffer"),
            size: desc.size_bytes,
            usage,
            mapped_at_creation: false,
        });
        Ok(WgpuBuffer {
            buffer: Arc::new(buffer),
            size_bytes: desc.size_bytes,
            writable,
        })
    }

    fn create_pipeline(&self, kernel: CbtKernel) -> Result<Self::Pipeline, Self::Error> {
        let shader = self
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("thessa-rcbt-portable-kernels"),
                source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(RCBT_WGSL)),
            });
        let layout = self
            .device
            .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("thessa-rcbt-pipeline-layout"),
                bind_group_layouts: &[Some(&self.bind_group_layout)],
                immediate_size: 0,
            });
        let pipeline = self
            .device
            .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(Self::entry_point(kernel)),
                layout: Some(&layout),
                module: &shader,
                entry_point: Some(Self::entry_point(kernel)),
                compilation_options: Default::default(),
                cache: None,
            });
        Ok(WgpuPipeline { pipeline })
    }

    fn begin_commands(&self) -> Self::Commands {
        WgpuCommands {
            encoder: self
                .device
                .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                    label: Some("thessa-rcbt-commands"),
                }),
            dispatches: 0,
            bytes_touched: 0,
        }
    }

    fn dispatch(
        &self,
        commands: &mut Self::Commands,
        pipeline: &Self::Pipeline,
        bindings: &CbtBindings<Self::Buffer>,
        groups: DispatchSize,
    ) -> Result<(), Self::Error> {
        let binding = bindings
            .buffers
            .iter()
            .find(|binding| binding.slot == 0)
            .ok_or(WgpuError::MissingBinding(0))?;
        let end = binding
            .offset_bytes
            .checked_add(binding.size_bytes)
            .ok_or(WgpuError::BufferRangeOutOfBounds)?;
        if end > binding.resource.size_bytes || binding.size_bytes < 4 {
            return Err(WgpuError::BufferRangeOutOfBounds);
        }
        let invocations = (groups.x as u64)
            .checked_mul(groups.y as u64)
            .and_then(|value| value.checked_mul(groups.z as u64))
            .and_then(|value| value.checked_mul(WORKGROUP_SIZE as u64))
            .ok_or(WgpuError::BufferRangeOutOfBounds)?;
        if invocations > binding.size_bytes / 4 {
            return Err(WgpuError::BufferRangeOutOfBounds);
        }
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("thessa-rcbt-bind-group"),
            layout: &self.bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &binding.resource.buffer,
                    offset: binding.offset_bytes,
                    size: NonZeroU64::new(binding.size_bytes),
                }),
            }],
        });
        let mut pass = commands
            .encoder
            .begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("thessa-rcbt-compute-pass"),
                timestamp_writes: None,
            });
        pass.set_pipeline(&pipeline.pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.dispatch_workgroups(groups.x, groups.y, groups.z);
        commands.dispatches += 1;
        let total_groups = (groups.x as u64)
            .saturating_mul(groups.y as u64)
            .saturating_mul(groups.z as u64);
        commands.bytes_touched = commands.bytes_touched.saturating_add(
            binding
                .size_bytes
                .min(binding.resource.size_bytes)
                .saturating_mul(total_groups),
        );
        Ok(())
    }

    fn barrier(&self, _commands: &mut Self::Commands, _barrier: CbtBarrier) {
        // wgpu orders commands between compute passes. Keeping this operation
        // explicit in the semantic API lets native adapters use stricter
        // barriers without leaking those details into rcbt-core.
    }

    fn submit(&self, commands: Self::Commands) -> Result<CbtMetrics, Self::Error> {
        self.queue.submit(Some(commands.encoder.finish()));
        Ok(CbtMetrics {
            dispatches: commands.dispatches,
            active_leaves: 0,
            bytes_touched: commands.bytes_touched,
            uploaded_bytes: 0,
        })
    }
}

/// Portable baseline kernel source. It deliberately does not encode terrain
/// semantics; it provides a valid dispatch target for backend/integration
/// bring-up and later differential tests.
pub const RCBT_WGSL: &str = r#"
@group(0) @binding(0) var<storage, read_write> data: array<atomic<u32>>;

fn touch(index: u32) {
    atomicAdd(&data[index], 1u);
}

@compute @workgroup_size(64)
fn classify(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn bisect(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn simplify(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn reduce(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn compact_leaves(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn build_draw_list(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
@compute @workgroup_size(64)
fn generate_vertices(@builtin(global_invocation_id) id: vec3<u32>) { touch(id.x); }
"#;

/// Convenience constructor for a single storage binding.
pub fn storage_binding(buffer: WgpuBuffer) -> CbtBindings<WgpuBuffer> {
    CbtBindings {
        buffers: vec![BufferBinding {
            slot: 0,
            offset_bytes: 0,
            size_bytes: buffer.size_bytes,
            resource: buffer,
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::RCBT_WGSL;

    #[test]
    fn portable_kernel_source_is_valid_wgsl() {
        naga::front::wgsl::parse_str(RCBT_WGSL).expect("RCBT WGSL must remain portable");
    }
}
