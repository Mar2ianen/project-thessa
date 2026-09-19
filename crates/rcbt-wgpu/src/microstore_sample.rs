//! GPU mip generation and packed sample-time filtering (follow-up to the
//! decode parity prototype).
//!
//! Two kernels from backend-neutral `thessa-microstore-core::wgsl`:
//!
//! - `mip_downsample` builds the next mip level from u32-per-texel data
//!   with integer box filtering (bit-exact with the CPU `MipChain`);
//! - `sample_packed` filters normalized UVs straight from the packed page
//!   (offset table + block headers, no expanded cache): the true
//!   sample-time access pattern, bilinearly mixed in f32.
//!
//! Outputs round to bytes; expect at most one code level of f32-vs-f64
//! drift against the CPU mirror on filtered samples (exact on mips).

use std::borrow::Cow;
use std::sync::Arc;

use thessa_microstore_core::{
    EncodedPage,
    wgsl::{
        MICROSTORE_ANISO_WGSL, MICROSTORE_LOD_WGSL, MICROSTORE_MIP_WGSL, MICROSTORE_SAMPLE_WGSL,
    },
};

use crate::WgpuError;

const WORKGROUP: u32 = 64;

fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 4);
    for w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

fn readback_u32(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    src: &wgpu::Buffer,
    count: usize,
) -> Result<Vec<u32>, WgpuError> {
    let bytes = count * 4;
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("microstore-sample-staging"),
        size: bytes.max(4) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("microstore-sample-copy"),
    });
    if bytes > 0 {
        encoder.copy_buffer_to_buffer(src, 0, &staging, 0, bytes as u64);
    }
    queue.submit(Some(encoder.finish()));
    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device
        .poll(wgpu::PollType::wait_indefinitely())
        .map_err(|e| WgpuError::Device(format!("device poll failed: {e:?}")))?;
    rx.recv()
        .map_err(|_| WgpuError::Device("staging map channel closed".into()))?
        .map_err(|e| WgpuError::Device(format!("staging map failed: {e:?}")))?;
    let view = slice.get_mapped_range();
    let mut out = Vec::with_capacity(count);
    let (chunks, _) = view.as_chunks::<4>();
    for chunk in chunks {
        out.push(u32::from_le_bytes(*chunk));
    }
    drop(view);
    staging.unmap();
    Ok(out)
}

fn storage(device: &wgpu::Device, label: &str, size: u64, copy_src: bool) -> wgpu::Buffer {
    let mut usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
    if copy_src {
        usage |= wgpu::BufferUsages::COPY_SRC;
    }
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some(label),
        size: size.max(4),
        usage,
        mapped_at_creation: false,
    })
}

fn compute_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    src: &str,
    entry: &str,
) -> wgpu::ComputePipeline {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("microstore-sample-shader"),
        source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(src)),
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("microstore-sample-pl"),
        bind_group_layouts: &[Some(layout)],
        immediate_size: 0,
    });
    device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some(entry),
        layout: Some(&pipeline_layout),
        module: &module,
        entry_point: Some(entry),
        compilation_options: Default::default(),
        cache: None,
    })
}

fn entry(binding: u32, read_only: bool, uniform: bool) -> wgpu::BindGroupLayoutEntry {
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

/// One mip level built on the GPU from u32-per-texel data.
pub struct GpuMips {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

impl GpuMips {
    /// Compile the downsample kernel.
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("microstore-mip-layout"),
            entries: &[
                entry(0, true, true),
                entry(1, true, false),
                entry(2, false, false),
            ],
        });
        let pipeline = compute_pipeline(&device, &layout, MICROSTORE_MIP_WGSL, "mip_downsample");
        Self {
            device,
            queue,
            layout,
            pipeline,
        }
    }

    /// Downsample `src` (`src_w` x `src_h` u32 texels) to the next level.
    /// Returns `(dst_w, dst_h, texels)` with halved extents (min 1).
    pub fn downsample(
        &self,
        src: &[u32],
        src_w: u32,
        src_h: u32,
    ) -> Result<(u32, u32, Vec<u32>), WgpuError> {
        let dst_w = src_w.div_ceil(2).max(1);
        let dst_h = src_h.div_ceil(2).max(1);
        let src_buf = storage(&self.device, "mip-src", (src.len() * 4) as u64, false);
        let dst_buf = storage(
            &self.device,
            "mip-dst",
            (dst_w as usize * dst_h as usize * 4) as u64,
            true,
        );
        let params_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("mip-params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        self.queue.write_buffer(&src_buf, 0, &words_to_bytes(src));
        self.queue.write_buffer(
            &params_buf,
            0,
            &words_to_bytes(&[src_w, src_h, dst_w, dst_h]),
        );
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mip-bg"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: src_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: dst_buf.as_entire_binding(),
                },
            ],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("mip-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("mip-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            let n = dst_w as usize * dst_h as usize;
            pass.dispatch_workgroups(n.div_ceil(WORKGROUP as usize) as u32, 1, 1);
        }
        self.queue.submit(Some(encoder.finish()));
        let out = readback_u32(
            &self.device,
            &self.queue,
            &dst_buf,
            dst_w as usize * dst_h as usize,
        )?;
        Ok((dst_w, dst_h, out))
    }
}

/// Sample-time packed filtering: UVs in, filtered bytes out.
pub struct PackedSampler {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

impl PackedSampler {
    /// Compile the sample kernel.
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("microstore-sample-layout"),
            entries: &[
                entry(0, true, true),
                entry(1, true, false),
                entry(2, true, false),
                entry(3, true, false),
                entry(4, false, false),
            ],
        });
        let pipeline = compute_pipeline(&device, &layout, MICROSTORE_SAMPLE_WGSL, "sample_packed");
        Self {
            device,
            queue,
            layout,
            pipeline,
        }
    }

    /// Filter `uvs` (normalized) against one packed page. Returns rounded
    /// bytes, one per sample, comparable with
    /// [`thessa_microstore_core::sample_rounded`] on the decoded field
    /// within one code level (f32-vs-f64 rounding).
    pub fn sample(&self, page: &EncodedPage, uvs: &[[f32; 2]]) -> Result<Vec<u32>, WgpuError> {
        use thessa_microstore_core::wgsl::{block_base_table, padded_upload_bytes};
        page.validate()
            .map_err(|e| WgpuError::Device(format!("invalid page: {e}")))?;
        let mut words = padded_upload_bytes(page);
        words.extend_from_slice(&[0u8; 4]);
        let table = block_base_table(page);
        let page_buf = storage(&self.device, "sample-page", words.len() as u64, false);
        let table_buf = storage(
            &self.device,
            "sample-table",
            (table.len() * 4) as u64,
            false,
        );
        let mut uv_bytes = Vec::with_capacity(uvs.len() * 8);
        for uv in uvs {
            uv_bytes.extend_from_slice(&uv[0].to_le_bytes());
            uv_bytes.extend_from_slice(&uv[1].to_le_bytes());
        }
        let uv_buf = storage(
            &self.device,
            "sample-uv",
            uv_bytes.len().max(4) as u64,
            false,
        );
        let params_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("sample-params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let out_buf = storage(&self.device, "sample-out", (uvs.len() * 4) as u64, true);
        self.queue.write_buffer(&page_buf, 0, &words);
        self.queue
            .write_buffer(&table_buf, 0, &words_to_bytes(&table));
        if !uv_bytes.is_empty() {
            self.queue.write_buffer(&uv_buf, 0, &uv_bytes);
        }
        self.queue.write_buffer(
            &params_buf,
            0,
            &words_to_bytes(&[page.width, page.height, page.blocks_x, page.blocks_y]),
        );
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("sample-bg"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: page_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: table_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: uv_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("sample-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("sample-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups(uvs.len().div_ceil(WORKGROUP as usize).max(1) as u32, 1, 1);
        }
        self.queue.submit(Some(encoder.finish()));
        readback_u32(&self.device, &self.queue, &out_buf, uvs.len())
    }
}

/// Best-effort GPU handle for tests and benches: `None` (with a SKIP
/// message at the call site) when no adapter exists, so GPU-less CI stays
/// green while real hardware runs the full parity suite.
#[cfg(test)]
pub(crate) fn jacobian_set(count: usize, seed: u64) -> Vec<[f32; 4]> {
    // Seeded footprints from sub-texel to 32 texels, axis-aligned and
    // diagonal, plus exact powers of two (log2 boundary probes).
    let mut out = vec![
        [1.0 / 64.0, 0.0, 0.0, 1.0 / 64.0],
        [4.0 / 64.0, 0.0, 0.0, 4.0 / 64.0],
        [8.0 / 64.0, 0.0, 0.0, 1.0 / 64.0],
        [2.0 / 64.0, 2.0 / 64.0, 0.0, 0.0],
    ];
    let mut s = seed | 1;
    let mut next = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };
    while out.len() < count {
        let exp = next() * 6.0 - 1.0;
        let m = 2f64.powf(exp) as f32 / 64.0;
        let ang = next() as f32 * std::f32::consts::TAU;
        out.push([m * ang.cos(), m * ang.sin(), -m * ang.sin(), m * ang.cos()]);
    }
    out
}

#[cfg(test)]
pub(crate) fn sample_uvs(count: usize, seed: u64) -> Vec<[f32; 2]> {
    // Seeded UVs plus exact corners/edges/center (filtering seams
    // hide at clamp boundaries, so probe them explicitly).
    let mut uvs = vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [0.5, 0.5]];
    let mut s = seed | 1;
    let mut next = || {
        s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };
    while uvs.len() < count {
        uvs.push([next() as f32, next() as f32]);
    }
    uvs
}

#[cfg(test)]
pub(crate) fn try_device() -> Option<(Arc<wgpu::Device>, Arc<wgpu::Queue>, String)> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
    }))
    .ok()?;
    let name = adapter.get_info().name;
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("microstore-test"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        experimental_features: wgpu::ExperimentalFeatures::default(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
    }))
    .ok()?;
    Some((Arc::new(device), Arc::new(queue), name))
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_microstore_core::{EncodeMode, fixtures, sample_rounded};

    #[test]
    fn mip_and_sample_wgsl_parse_with_naga() {
        for (name, src, entry) in [
            ("mip", MICROSTORE_MIP_WGSL, "mip_downsample"),
            ("sample", MICROSTORE_SAMPLE_WGSL, "sample_packed"),
            ("lod", MICROSTORE_LOD_WGSL, "lod_select"),
            ("aniso", MICROSTORE_ANISO_WGSL, "sample_aniso_packed"),
        ] {
            let module =
                naga::front::wgsl::parse_str(src).unwrap_or_else(|e| panic!("{name}: {e:?}"));
            assert!(
                module.entry_points.iter().any(|e| e.name == entry),
                "{name} entry"
            );
        }
    }

    #[test]
    fn gpu_aniso_uniform_is_exact_for_all_taps() {
        // No texel boundaries to straddle on a uniform field: every tap
        // decodes the same value, so float fuzz cannot flip anything and
        // the result must be bit-exact for every tap count.
        let Some((device, queue, _)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for uniform aniso test");
            return;
        };
        let sampler = AnisoSampler::new(device, queue);
        // Constant field (not dithered): every tap decodes 150 whatever
        // the float fuzz does to positions.
        let field = thessa_microstore_core::ScalarField::new(32, 32, vec![150u8; 1024])
            .expect("constant field");
        let page = thessa_microstore_core::EncodedPage::encode(
            &field,
            EncodeMode::Adaptive { max_abs_error: 2.0 },
        );
        let expect = page.decode().data[0];
        let uvs = sample_uvs(64, 0x4F1A);
        let jacs = jacobian_set(64, 0x4C4A);
        for taps in [1u32, 2, 4, 8] {
            let gpu = sampler
                .sample(&page, &uvs, &jacs, taps)
                .expect("aniso sample");
            assert!(
                gpu.iter().all(|v| *v as i32 == expect as i32),
                "taps {taps}: uniform must decode exact"
            );
        }
    }

    #[test]
    fn gpu_mip_chain_matches_cpu_box_exactly() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for mip parity test");
            return;
        };
        eprintln!("mip parity on {name}");
        let mips = GpuMips::new(device, queue);
        for (fixture_name, field) in fixtures::all(64, 64) {
            // u32-per-texel level 0 straight from the fixture bytes.
            let level0: Vec<u32> = field.data.iter().map(|v| *v as u32).collect();
            let (w, h, gpu) = mips.downsample(&level0, 64, 64).expect("mip");
            assert_eq!((w, h), (32, 32));
            let chain = thessa_microstore_core::MipChain::build(&field, 1);
            let expect: Vec<u32> = chain.levels[1].data.iter().map(|v| *v as u32).collect();
            assert_eq!(gpu, expect, "{fixture_name} mip exact");
        }
    }

    #[test]
    fn gpu_packed_samples_match_cpu_within_one_level() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for sample parity test");
            return;
        };
        eprintln!("sample parity on {name}");
        let sampler = PackedSampler::new(device, queue);
        let uvs = sample_uvs(256, 0x5A4E);
        for (fixture_name, field) in fixtures::all(64, 64) {
            for mode in [
                EncodeMode::Residual8,
                EncodeMode::Residual4,
                EncodeMode::Adaptive { max_abs_error: 2.0 },
            ] {
                let page = EncodedPage::encode(&field, mode);
                let decoded = page.decode();
                let gpu = sampler.sample(&page, &uvs).expect("sample");
                assert_eq!(gpu.len(), uvs.len());
                for (i, (uv, got)) in uvs.iter().zip(gpu.iter()).enumerate() {
                    let want = sample_rounded(&decoded, uv[0], uv[1]) as i32;
                    let diff = (*got as i32 - want).abs();
                    assert!(
                        diff <= 1,
                        "{fixture_name} {mode:?} uv{i}: gpu {got} cpu {want}"
                    );
                }
            }
        }
    }
}

/// GPU mip-level selector: footprints in, levels out.
pub struct LodSelector {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

impl LodSelector {
    /// Compile the selection kernel.
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("microstore-lod-layout"),
            entries: &[
                entry(0, true, true),
                entry(1, true, false),
                entry(2, false, false),
            ],
        });
        let pipeline = compute_pipeline(&device, &layout, MICROSTORE_LOD_WGSL, "lod_select");
        Self {
            device,
            queue,
            layout,
            pipeline,
        }
    }

    /// Select levels for `jacs` (`[dudx, dudy, dvdx, dvdy]` each) on a
    /// `width` x `height` level 0 with `max_level` clamp. Returns one u32
    /// level per footprint, comparable with
    /// [`thessa_microstore_core::lod_level`] up to log2 boundary fuzz
    /// (GPUs vary here too; callers allow +-1).
    pub fn select(
        &self,
        jacs: &[[f32; 4]],
        width: u32,
        height: u32,
        max_level: u32,
    ) -> Result<Vec<u32>, WgpuError> {
        let mut jac_bytes = Vec::with_capacity(jacs.len() * 16);
        for j in jacs {
            for v in j {
                jac_bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        let jac_buf = storage(
            &self.device,
            "lod-jac",
            jac_bytes.len().max(4) as u64,
            false,
        );
        let params_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("lod-params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let out_buf = storage(&self.device, "lod-out", (jacs.len() * 4) as u64, true);
        if !jac_bytes.is_empty() {
            self.queue.write_buffer(&jac_buf, 0, &jac_bytes);
        }
        self.queue.write_buffer(
            &params_buf,
            0,
            &words_to_bytes(&[width, height, max_level, 0]),
        );
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("lod-bg"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: jac_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("lod-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("lod-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups(jacs.len().div_ceil(WORKGROUP as usize).max(1) as u32, 1, 1);
        }
        self.queue.submit(Some(encoder.finish()));
        readback_u32(&self.device, &self.queue, &out_buf, jacs.len())
    }
}

/// GPU anisotropic packed sampler: UVs plus Jacobians in, filtered bytes
/// out, with per-dispatch tap selection (1/2/4/8).
pub struct AnisoSampler {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

impl AnisoSampler {
    /// Compile the anisotropic sample kernel.
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("microstore-aniso-layout"),
            entries: &[
                entry(0, true, true),
                entry(1, true, false),
                entry(2, true, false),
                entry(3, true, false),
                entry(4, true, false),
                entry(5, false, false),
            ],
        });
        let pipeline = compute_pipeline(
            &device,
            &layout,
            MICROSTORE_ANISO_WGSL,
            "sample_aniso_packed",
        );
        Self {
            device,
            queue,
            layout,
            pipeline,
        }
    }

    /// Filter `uvs` with per-sample Jacobians at `taps` (quantized to
    /// 1/2/4/8 like the CPU mirror). Returns rounded bytes comparable
    /// with [`thessa_microstore_core::sample_aniso`] on the decoded field
    /// within a couple of code levels (tap-average f32 accumulation).
    #[allow(clippy::too_many_arguments)]
    pub fn sample(
        &self,
        page: &EncodedPage,
        uvs: &[[f32; 2]],
        jacs: &[[f32; 4]],
        taps: u32,
    ) -> Result<Vec<u32>, WgpuError> {
        use thessa_microstore_core::wgsl::{block_base_table, padded_upload_bytes};
        assert_eq!(uvs.len(), jacs.len(), "uvs and jacobians pair up");
        page.validate()
            .map_err(|e| WgpuError::Device(format!("invalid page: {e}")))?;
        let mut words = padded_upload_bytes(page);
        words.extend_from_slice(&[0u8; 4]);
        let table = block_base_table(page);
        let page_buf = storage(&self.device, "aniso-page", words.len() as u64, false);
        let table_buf = storage(&self.device, "aniso-table", (table.len() * 4) as u64, false);
        let mut uv_bytes = Vec::with_capacity(uvs.len() * 8);
        for uv in uvs {
            uv_bytes.extend_from_slice(&uv[0].to_le_bytes());
            uv_bytes.extend_from_slice(&uv[1].to_le_bytes());
        }
        let mut jac_bytes = Vec::with_capacity(jacs.len() * 16);
        for j in jacs {
            for v in j {
                jac_bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        let uv_buf = storage(
            &self.device,
            "aniso-uv",
            uv_bytes.len().max(4) as u64,
            false,
        );
        let jac_buf = storage(
            &self.device,
            "aniso-jac",
            jac_bytes.len().max(4) as u64,
            false,
        );
        let params_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("aniso-params"),
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let out_buf = storage(&self.device, "aniso-out", (uvs.len() * 4) as u64, true);
        self.queue.write_buffer(&page_buf, 0, &words);
        self.queue
            .write_buffer(&table_buf, 0, &words_to_bytes(&table));
        if !uv_bytes.is_empty() {
            self.queue.write_buffer(&uv_buf, 0, &uv_bytes);
            self.queue.write_buffer(&jac_buf, 0, &jac_bytes);
        }
        self.queue.write_buffer(
            &params_buf,
            0,
            &words_to_bytes(&[
                page.width,
                page.height,
                page.blocks_x,
                page.blocks_y,
                taps,
                0,
                0,
                0,
            ]),
        );
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("aniso-bg"),
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: page_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: table_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: uv_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: jac_buf.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("aniso-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("aniso-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups(uvs.len().div_ceil(WORKGROUP as usize).max(1) as u32, 1, 1);
        }
        self.queue.submit(Some(encoder.finish()));
        readback_u32(&self.device, &self.queue, &out_buf, uvs.len())
    }
}

#[cfg(test)]
mod lod_aniso_tests {
    use super::{AnisoSampler, LodSelector, jacobian_set, try_device};
    use thessa_microstore_core::{EncodeMode, UvJacobian, fixtures, lod_level, sample_aniso};

    #[test]
    fn gpu_lod_matches_cpu_within_one_level() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for LOD parity test");
            return;
        };
        eprintln!("lod parity on {name}");
        let selector = LodSelector::new(device, queue);
        let jacs = jacobian_set(256, 0x10D);
        let gpu = selector.select(&jacs, 64, 64, 7).expect("lod select");
        let mut exact = 0;
        for (j, got) in jacs.iter().zip(gpu.iter()) {
            let jac = UvJacobian {
                dudx: j[0],
                dudy: j[1],
                dvdx: j[2],
                dvdy: j[3],
            };
            let want = lod_level(&jac, [64.0, 64.0], 7);
            let diff = (*got as i32 - want as i32).abs();
            assert!(diff <= 1, "footprint {j:?}: gpu {got} cpu {want}");
            exact += (diff == 0) as usize;
        }
        eprintln!(
            "lod exact {exact}/{} (rest off by one at log2 boundaries)",
            jacs.len()
        );
        assert!(
            exact * 2 >= jacs.len(),
            "mostly exact, fuzz only at boundaries"
        );
    }

    #[test]
    fn gpu_aniso_matches_cpu_on_real_data() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for aniso parity test");
            return;
        };
        eprintln!("aniso parity on {name}");
        let sampler = AnisoSampler::new(device, queue);
        let field = fixtures::coast(64, 64, 0xA41);
        let page = thessa_microstore_core::EncodedPage::encode(
            &field,
            EncodeMode::Adaptive { max_abs_error: 2.0 },
        );
        let decoded = page.decode();
        let jacs = jacobian_set(128, 0x4EED);
        let uvs: Vec<[f32; 2]> = {
            let mut s = 0x5EDu64;
            let mut out = Vec::with_capacity(128);
            for _ in 0..128 {
                s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z = s;
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                let a = (z ^ (z >> 31)) as f64 / u64::MAX as f64;
                s = s.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z2 = s;
                z2 = (z2 ^ (z2 >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z2 = (z2 ^ (z2 >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                let b = (z2 ^ (z2 >> 31)) as f64 / u64::MAX as f64;
                out.push([a as f32, b as f32]);
            }
            out
        };
        for taps in [1u32, 2, 4, 8] {
            let gpu = sampler
                .sample(&page, &uvs, &jacs[..uvs.len()], taps)
                .expect("aniso sample");
            let mut within_one = 0usize;
            let mut worst = 0i32;
            for ((uv, j), got) in uvs.iter().zip(jacs.iter()).zip(gpu.iter()) {
                let jac = UvJacobian {
                    dudx: j[0],
                    dudy: j[1],
                    dvdx: j[2],
                    dvdy: j[3],
                };
                let want = sample_aniso(&decoded, uv[0], uv[1], &jac, taps)
                    .round()
                    .clamp(0.0, 255.0) as i32;
                let diff = (*got as i32 - want).abs();
                worst = worst.max(diff);
                within_one += (diff <= 1) as usize;
            }
            // Taps landing within an ulp of a texel boundary flip floor()
            // between f32/FMA evaluations (GPUs vary here too): near-total
            // agreement plus a bounded tail, never broad corruption.
            eprintln!(
                "taps {taps}: {within_one}/{} within 1 level, worst {worst}",
                uvs.len()
            );
            assert!(
                within_one * 20 >= uvs.len() * 19,
                "taps {taps}: only {within_one}/{} within 1 level",
                uvs.len()
            );
            assert!(worst <= 32, "taps {taps}: runaway drift {worst}");
        }
    }
}
