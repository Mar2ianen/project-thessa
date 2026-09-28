//! GPU decode parity prototype for microscaled pages on the portable wgpu
//! backend (doc 41 §18 Phase B).
//!
//! The adapter owns all wgpu handles; the WGSL text and the offset-table
//! layout live in backend-neutral `thessa-microstore-core::wgsl`. This
//! module uploads one [`EncodedPage`] (headers + packed payload, plus the
//! CPU-built block offset table) and runs the `decode_page` kernel with one
//! thread per texel — the exact shape a material shader will use later.
//!
//! This is a parity oracle, not the sample-time path: it decodes whole
//! pages through dispatch plus synchronous staging readback so every GPU
//! texel can be checked against the CPU reference decoder. The reported
//! milliseconds therefore cover allocation, submit, sync, and readback —
//! not the cost of a material-shader sample, which additionally needs
//! bilinear/anisotropic filtering and mip policy the current RGBA path
//! gets from hardware.
//!
//! The current RGBA material path is untouched: this is an additive A/B
//! experiment, and every GPU result is checked against the CPU reference
//! decoder texel-for-texel.

use std::borrow::Cow;
use std::sync::Arc;

use thessa_microstore_core::{
    EncodedPage,
    wgsl::{MICROSTORE_DECODE_WGSL, block_base_table, padded_upload_bytes, upload_size},
};

use crate::WgpuError;

const WORKGROUP: u32 = 64;
const ENTRY: &str = "decode_page";
/// Zero slack past the page payload: the Residual6 decoder reads the byte
/// after the last payload byte for the final code, so the upload always
/// carries one spare word. Decoder overreads are then in-bounds by
/// construction instead of by WebGPU robustness rules.
const OVERREAD_SLACK_BYTES: usize = 4;

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

/// Decode pipeline for microscaled pages: uniform params, read-only page
/// words, read-only block offsets, read-write texel output.
pub struct MicrostoreDecode {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    layout: wgpu::BindGroupLayout,
    pipeline: wgpu::ComputePipeline,
}

impl MicrostoreDecode {
    /// Compile the sample-time decoder. Shader-module creation validates
    /// the WGSL on the driver; use the naga test for driver-free checks.
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("microstore-decode-layout"),
            entries: &[
                layout_entry(0, true, true),
                layout_entry(1, true, false),
                layout_entry(2, true, false),
                layout_entry(3, false, false),
            ],
        });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("microstore-decode-shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(MICROSTORE_DECODE_WGSL)),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("microstore-decode-pl"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(ENTRY),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some(ENTRY),
            compilation_options: Default::default(),
            cache: None,
        });
        Self {
            device,
            queue,
            layout,
            pipeline,
        }
    }

    fn storage(&self, label: &str, size: u64, copy_src: bool) -> wgpu::Buffer {
        let mut usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
        if copy_src {
            usage |= wgpu::BufferUsages::COPY_SRC;
        }
        self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: size.max(4),
            usage,
            mapped_at_creation: false,
        })
    }

    /// Upload one page and decode it on the GPU. Returns row-major decoded
    /// bytes over the true extent, directly comparable with
    /// [`EncodedPage::decode`]. Validates the page structure first: the
    /// shader indexes blocks with no further bounds checks.
    pub fn decode_page(&self, page: &EncodedPage) -> Result<Vec<u8>, WgpuError> {
        page.validate()
            .map_err(|e| WgpuError::Device(format!("invalid page: {e}")))?;
        let mut words = padded_upload_bytes(page);
        words.extend_from_slice(&[0u8; OVERREAD_SLACK_BYTES]);
        let table = block_base_table(page);
        self.decode_raw(
            &words,
            &table,
            page.width,
            page.height,
            page.blocks_x,
            page.blocks_y,
        )
    }

    /// Decode from caller-arranged upload bytes: `words` holds block
    /// images at arbitrary byte offsets listed by `table` (one u32 per
    /// block, row-major), exactly the shape a slab allocator produces for
    /// incremental variable-size updates. `words` must cover every byte
    /// the table addresses (plus slack for the final code); the buffer is
    /// zero-padded to a u32 multiple internally.
    pub fn decode_scattered(
        &self,
        words: &[u8],
        table: &[u32],
        width: u32,
        height: u32,
        blocks_x: u32,
        blocks_y: u32,
    ) -> Result<Vec<u8>, WgpuError> {
        let mut padded = words.to_vec();
        while !padded.len().is_multiple_of(4) {
            padded.push(0);
        }
        padded.extend_from_slice(&[0u8; OVERREAD_SLACK_BYTES]);
        self.decode_raw(&padded, table, width, height, blocks_x, blocks_y)
    }

    fn decode_raw(
        &self,
        words: &[u8],
        table: &[u32],
        width: u32,
        height: u32,
        blocks_x: u32,
        blocks_y: u32,
    ) -> Result<Vec<u8>, WgpuError> {
        let texels = width as usize * height as usize;
        let groups_x = texels.div_ceil(WORKGROUP as usize);
        let group_limit = self.device.limits().max_compute_workgroups_per_dimension as usize;
        if groups_x > group_limit {
            return Err(WgpuError::DispatchWorkgroupsTooLarge {
                x: groups_x as u32,
                limit: group_limit as u32,
            });
        }
        let table = table.to_vec();
        let table_bytes = words_to_bytes(&table);
        let params = words_to_bytes(&[width, height, blocks_x, blocks_y]);

        let page_buf = self.storage("microstore-page", words.len() as u64, false);
        let table_buf = self.storage("microstore-table", table_bytes.len() as u64, false);
        let params_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("microstore-params"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let out_buf = self.storage("microstore-out", (texels * 4) as u64, true);
        self.queue.write_buffer(&page_buf, 0, words);
        self.queue.write_buffer(&table_buf, 0, &table_bytes);
        self.queue.write_buffer(&params_buf, 0, &params);

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("microstore-decode-bg"),
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
                    resource: out_buf.as_entire_binding(),
                },
            ],
        });
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("microstore-decode-enc"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("microstore-decode-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.dispatch_workgroups(groups_x as u32, 1, 1);
        }
        let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("microstore-staging"),
            size: (texels * 4).max(4) as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        encoder.copy_buffer_to_buffer(&out_buf, 0, &staging, 0, (texels * 4) as u64);
        self.queue.submit(Some(encoder.finish()));

        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| WgpuError::Device(format!("device poll failed: {e:?}")))?;
        rx.recv()
            .map_err(|_| WgpuError::Device("staging map channel closed".into()))?
            .map_err(|e| WgpuError::Device(format!("staging map failed: {e:?}")))?;
        let view = slice.get_mapped_range();
        let mut out = Vec::with_capacity(texels);
        let (chunks, _) = view.as_chunks::<4>();
        for chunk in chunks {
            out.push(chunk[0]);
        }
        drop(view);
        staging.unmap();
        Ok(out)
    }
}

fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 4);
    for w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

/// CPU-side upload accounting for telemetry (doc §9): no GPU needed.
/// Returns `(padded page bytes, offset-table bytes, params bytes)`.
pub fn upload_accounting(page: &EncodedPage) -> (usize, usize, usize) {
    let up = upload_size(page);
    (up.padded_bytes, up.table_bytes, up.params_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_microstore_core::{EncodeMode, fixtures};

    #[test]
    fn decode_wgsl_parses_with_naga() {
        // Driver-free shader check: the kernel must be valid WGSL before
        // any GPU ever sees it.
        let module =
            naga::front::wgsl::parse_str(MICROSTORE_DECODE_WGSL).expect("microstore WGSL parses");
        let entry = module
            .entry_points
            .iter()
            .find(|e| e.name == ENTRY)
            .expect("decode_page entry point");
        assert_eq!(entry.workgroup_size, [64, 1, 1]);
    }

    #[test]
    fn upload_accounting_matches_wire() {
        for (_, field) in fixtures::all(24, 16) {
            let page = EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
            let (padded, table, params) = upload_accounting(&page);
            assert_eq!(padded, page.encoded_bytes().div_ceil(4) * 4);
            assert_eq!(table, page.blocks.len() * 4);
            assert_eq!(params, 16);
        }
    }

    fn try_device() -> Option<(Arc<wgpu::Device>, Arc<wgpu::Queue>, String)> {
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

    #[test]
    fn gpu_decode_matches_cpu_on_all_fixtures_and_modes() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for microstore decode test");
            return;
        };
        eprintln!("microstore decode on {name}");
        let decoder = MicrostoreDecode::new(device, queue);
        for (fixture_name, field) in fixtures::all(64, 64) {
            for mode in [
                EncodeMode::Raw8,
                EncodeMode::Residual8,
                EncodeMode::Residual4,
                EncodeMode::Residual6,
                EncodeMode::Residual2,
                EncodeMode::Adaptive { max_abs_error: 2.0 },
            ] {
                let page = EncodedPage::encode(&field, mode);
                let gpu = decoder.decode_page(&page).expect("gpu decode");
                assert_eq!(gpu, page.decode().data, "{fixture_name} {mode:?}");
            }
        }
    }

    #[test]
    fn decode_rejects_structurally_invalid_pages_without_gpu() {
        // Validation happens before any upload: no adapter needed.
        let field = fixtures::uniform(8, 8, 1);
        let mut page = EncodedPage::encode(&field, EncodeMode::Residual4);
        page.blocks.pop();
        // Build a decoder-free check through validate(); decode_page
        // needs a device, so the unit contract is validated here and the
        // device path reuses the same call.
        assert!(page.validate().is_err());
    }

    #[test]
    fn gpu_decode_handles_odd_extents() {
        let Some((device, queue, _)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for microstore odd-extent test");
            return;
        };
        let decoder = MicrostoreDecode::new(device, queue);
        for (w, h) in [(1, 1), (5, 7), (13, 29)] {
            let field = fixtures::noise(w, h, 0xBEEF);
            let page = EncodedPage::encode(&field, EncodeMode::Residual4);
            let gpu = decoder.decode_page(&page).expect("gpu decode");
            assert_eq!(gpu, page.decode().data, "{w}x{h}");
        }
    }

    #[test]
    fn gpu_decode_scattered_allocator_layout() {
        // End-to-end variable-size story on hardware: blocks placed at
        // slab-allocator offsets in scrambled order (fragmented buffer,
        // not the compact wire image), decoded through the offset table.
        // Must match the CPU reference bit-exactly.
        use thessa_microstore_core::{SlabAllocator, wgsl::block_base_table};
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for scattered layout test");
            return;
        };
        eprintln!("scattered layout on {name}");
        let decoder = MicrostoreDecode::new(device, queue);
        let field = fixtures::coast(64, 64, 0x5CA7);
        let page = EncodedPage::encode(&field, EncodeMode::Adaptive { max_abs_error: 2.0 });
        page.validate().expect("encodes valid");
        let wire = page.to_bytes();
        let bases = block_base_table(&page);
        // Scrambled placement order fragments the buffer on purpose.
        let mut order: Vec<usize> = (0..page.blocks.len()).collect();
        order.reverse();
        let mut slab = SlabAllocator::new(wire.len() as u64 + 4096);
        let mut scattered = vec![0u8; wire.len() + 4096];
        let mut table = vec![0u32; page.blocks.len()];
        for i in order {
            let base = bases[i] as usize;
            let len = 3 + page.blocks[i].codec.payload_len();
            let at = slab.alloc(i as u64, len as u64).expect("slab fits") as usize;
            scattered[at..at + len].copy_from_slice(&wire[base..base + len]);
            table[i] = at as u32;
        }
        slab.check_invariants();
        let gpu = decoder
            .decode_scattered(
                &scattered,
                &table,
                page.width,
                page.height,
                page.blocks_x,
                page.blocks_y,
            )
            .expect("scattered decode");
        assert_eq!(gpu, page.decode().data);
    }
}
