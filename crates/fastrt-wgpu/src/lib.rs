//! Portable wgpu backend for FastRT batched geometric queries.
//!
//! [`BatchSlab`] evaluates the slab ray-AABB intersection over a Cartesian
//! product of rays and boxes on the GPU (one thread per pair, f32). The f64
//! CPU oracle (`thessa_plume_core::cbt_volume::ray_intersect_slab`) remains
//! truth; the GPU is a measured accelerator with a tested error envelope,
//! never a replacement semantic. Inputs must be finite (debug-asserted on
//! pack); empty ray or box lists return an empty vector without dispatch.

#![forbid(unsafe_code)]

use std::sync::Arc;

const WORKGROUP: u32 = 256;
const SHADER: &str = include_str!("batch_slab.wgsl");

/// Failure of the batched intersect path.
#[derive(Debug, Clone, PartialEq)]
pub enum SlabError {
    /// wgpu/device/queue/mapping failure with context.
    Device(String),
    /// `rays * boxes` exceeds the `u32` dispatch domain.
    TooLarge(u64),
}

impl std::fmt::Display for SlabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Device(detail) => write!(f, "fastrt-wgpu device failure: {detail}"),
            Self::TooLarge(pairs) => write!(f, "fastrt-wgpu pair count overflow: {pairs}"),
        }
    }
}

impl std::error::Error for SlabError {}

/// One decoded pair result, in ray-major order (`pair = ray * nboxes + box`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlabHit {
    /// Entry distance in metres, clamped at 0 like the CPU oracle.
    pub t_enter: f32,
    /// Exit distance in metres.
    pub t_exit: f32,
    pub hit: bool,
}

/// Cached compute pipeline for [`BatchSlab::intersect`]. The module,
/// layout, and pipeline are built once; device buffers and the bind group
/// live in a capacity pool rebuilt only on growth, so steady-state calls
/// pay upload + dispatch + readback only.
pub struct BatchSlab {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    pool: Option<Pool>,
}

/// Device buffers plus their bind group for one pair capacity (power of
/// two). Dispatch always covers exactly the requested pairs; the shader
/// guards the tail via `arrayLength`, so over-capacity threads never exist.
struct Pool {
    pairs: u64,
    bmin: wgpu::Buffer,
    bmax: wgpu::Buffer,
    org: wgpu::Buffer,
    dir: wgpu::Buffer,
    out: wgpu::Buffer,
    staging: wgpu::Buffer,
    dims: wgpu::Buffer,
    bind: wgpu::BindGroup,
}

impl BatchSlab {
    pub fn new(device: Arc<wgpu::Device>, queue: Arc<wgpu::Queue>) -> Self {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("fastrt-batch-slab"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("fastrt-batch-slab-layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("fastrt-batch-slab-pipeline-layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("fastrt-batch-slab-pipeline"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            cache: None,
        });
        Self {
            device,
            queue,
            pipeline,
            layout,
            pool: None,
        }
    }

    fn build_pool(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        capacity_pairs: u64,
    ) -> Pool {
        let storage = |label: &str, read_only: bool| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: capacity_pairs * 16,
                usage: if read_only {
                    wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST
                } else {
                    wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC
                },
                mapped_at_creation: false,
            })
        };
        let bmin = storage("fastrt-boxes-min", true);
        let bmax = storage("fastrt-boxes-max", true);
        let org = storage("fastrt-rays-org", true);
        let dir = storage("fastrt-rays-dir", true);
        let out = storage("fastrt-pairs-out", false);
        let staging = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("fastrt-pairs-staging"),
            size: capacity_pairs * 16,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let dims = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("fastrt-batch-dims"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("fastrt-batch-slab-bind"),
            layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: bmin.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: bmax.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: org.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: dir.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: out.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 5,
                    resource: dims.as_entire_binding(),
                },
            ],
        });
        Pool {
            pairs: capacity_pairs,
            bmin,
            bmax,
            org,
            dir,
            out,
            staging,
            dims,
            bind,
        }
    }

    /// Intersect every ray with every box. Returns ray-major hits
    /// (`out[ray * boxes + box]`). Buffers persist across calls (grown to
    /// the next power of two on demand); steady-state cost is upload +
    /// dispatch + synchronous readback — benches measure this end to end,
    /// not dispatch alone.
    pub fn intersect(
        &mut self,
        origins: &[[f32; 3]],
        dirs: &[[f32; 3]],
        boxes_min: &[[f32; 3]],
        boxes_max: &[[f32; 3]],
    ) -> Result<Vec<SlabHit>, SlabError> {
        assert_eq!(
            origins.len(),
            dirs.len(),
            "origins and dirs must pair one to one"
        );
        assert_eq!(
            boxes_min.len(),
            boxes_max.len(),
            "box minima and maxima must pair one to one"
        );
        let pairs = origins.len() as u64 * boxes_min.len() as u64;
        if pairs == 0 {
            return Ok(Vec::new());
        }
        if pairs > u32::MAX as u64 {
            return Err(SlabError::TooLarge(pairs));
        }
        debug_assert_finite(origins, dirs, boxes_min, boxes_max);

        let pack = |triples: &[[f32; 3]]| -> Vec<u8> {
            // Manual little-endian pack: keeps this crate dependency-free
            // (no bytemuck) at the cost of a pack pass the benches measure.
            let mut v = Vec::with_capacity(triples.len() * 16);
            for t in triples {
                v.extend_from_slice(&t[0].to_le_bytes());
                v.extend_from_slice(&t[1].to_le_bytes());
                v.extend_from_slice(&t[2].to_le_bytes());
                v.extend_from_slice(&[0u8; 4]);
            }
            v
        };
        let capacity = pairs.next_power_of_two();
        if self.pool.as_ref().is_none_or(|pool| pool.pairs < pairs) {
            self.pool = Some(Self::build_pool(&self.device, &self.layout, capacity));
        }
        let pool = self.pool.as_ref().expect("pool built above");
        let out_size = pairs * 16;
        let nrays = origins.len() as u32;
        let nboxes = boxes_min.len() as u32;
        let mut dims = [0u8; 16];
        dims[0..4].copy_from_slice(&nboxes.to_le_bytes());
        dims[4..8].copy_from_slice(&nrays.to_le_bytes());
        self.queue.write_buffer(&pool.dims, 0, &dims);
        self.queue.write_buffer(&pool.bmin, 0, &pack(boxes_min));
        self.queue.write_buffer(&pool.bmax, 0, &pack(boxes_max));
        self.queue.write_buffer(&pool.org, 0, &pack(origins));
        self.queue.write_buffer(&pool.dir, 0, &pack(dirs));
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("fastrt-batch-slab-encoder"),
            });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("fastrt-batch-slab-pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &pool.bind, &[]);
            pass.dispatch_workgroups(pairs.div_ceil(WORKGROUP as u64) as u32, 1, 1);
        }
        encoder.copy_buffer_to_buffer(&pool.out, 0, &pool.staging, 0, out_size);
        self.queue.submit(Some(encoder.finish()));

        let (tx, rx) = std::sync::mpsc::channel();
        pool.staging
            .slice(..out_size)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .map_err(|e| SlabError::Device(format!("device poll failed: {e:?}")))?;
        rx.recv()
            .map_err(|_| SlabError::Device("staging map channel closed".into()))?
            .map_err(|e| SlabError::Device(format!("staging map failed: {e:?}")))?;
        let view = pool
            .staging
            .slice(..out_size)
            .get_mapped_range()
            .map_err(|e| SlabError::Device(format!("staging map range failed: {e:?}")))?;
        let mut hits = Vec::with_capacity(pairs as usize);
        let (quads, _) = view.as_chunks::<16>();
        for quad in quads.iter().take(pairs as usize) {
            let word = |i: usize| {
                f32::from_le_bytes([
                    quad[4 * i],
                    quad[4 * i + 1],
                    quad[4 * i + 2],
                    quad[4 * i + 3],
                ])
            };
            hits.push(SlabHit {
                t_enter: word(0),
                t_exit: word(1),
                hit: word(2) != 0.0,
            });
        }
        drop(view);
        pool.staging.unmap();
        Ok(hits)
    }
}

fn debug_assert_finite(
    origins: &[[f32; 3]],
    dirs: &[[f32; 3]],
    mins: &[[f32; 3]],
    maxs: &[[f32; 3]],
) {
    debug_assert!(
        origins
            .iter()
            .chain(dirs)
            .chain(mins)
            .chain(maxs)
            .all(|t| t.iter().all(|x| x.is_finite())),
        "fastrt-wgpu slab inputs must be finite (NaN is not in the GPU contract)"
    );
}

/// Headless device for tests and benches: `None` (with a printed SKIP at
/// call sites) when no adapter exists, so adapter-less CI stays green.
pub fn try_device() -> Option<(Arc<wgpu::Device>, Arc<wgpu::Queue>, String)> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
        // Headless compute: exact adapter limits, no fingerprint bucketing.
        apply_limit_buckets: false,
    }))
    .ok()?;
    let name = adapter.get_info().name;
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("fastrt-batch-slab"),
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

    fn lcg(state: &mut u64) -> f64 {
        // Deterministic 64-bit LCG; regression inputs must not drift.
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 11) as f64) / ((1u64 << 53) as f64)
    }

    #[test]
    fn gpu_slab_matches_f64_oracle_within_envelope() {
        let Some((device, queue, name)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for fastrt-wgpu parity test");
            return;
        };
        eprintln!("adapter: {name}");
        let mut backend = BatchSlab::new(device, queue);
        // 512 rays x 64 boxes, all finite, mixed hits/misses/grazes.
        let mut rng = 0xF1571277u64;
        let nrays = 512;
        let nboxes = 64;
        let mut origins = Vec::with_capacity(nrays);
        let mut dirs = Vec::with_capacity(nrays);
        for _ in 0..nrays {
            origins.push([
                (lcg(&mut rng) * 40.0 - 20.0) as f32,
                (lcg(&mut rng) * 40.0 - 20.0) as f32,
                (lcg(&mut rng) * 40.0 - 20.0) as f32,
            ]);
            let mut d = [
                (lcg(&mut rng) * 2.0 - 1.0) as f32,
                (lcg(&mut rng) * 2.0 - 1.0) as f32,
                (lcg(&mut rng) * 2.0 - 1.0) as f32,
            ];
            if d[0] == 0.0 && d[1] == 0.0 && d[2] == 0.0 {
                d[0] = 1.0;
            }
            dirs.push(d);
        }
        let mut mins = Vec::with_capacity(nboxes);
        let mut maxs = Vec::with_capacity(nboxes);
        for _ in 0..nboxes {
            let c = [
                lcg(&mut rng) * 20.0 - 10.0,
                lcg(&mut rng) * 20.0 - 10.0,
                lcg(&mut rng) * 20.0 - 10.0,
            ];
            let e = 0.5 + lcg(&mut rng) * 4.0;
            mins.push([(c[0] - e) as f32, (c[1] - e) as f32, (c[2] - e) as f32]);
            maxs.push([(c[0] + e) as f32, (c[1] + e) as f32, (c[2] + e) as f32]);
        }
        let hits = backend
            .intersect(&origins, &dirs, &mins, &maxs)
            .expect("batch intersect runs");
        assert_eq!(hits.len(), nrays * nboxes);

        // Error envelope vs the f64 oracle: hit agreement outside the
        // grazing lane, 2e-3 relative t accuracy on agreed hits.
        let mut agree = 0u64;
        let mut graze_mismatch = 0u64;
        let mut worst_rel = 0.0f64;
        for (r, (org, dir)) in origins.iter().zip(dirs.iter()).enumerate() {
            let org64 = [org[0] as f64, org[1] as f64, org[2] as f64];
            let dir64 = [dir[0] as f64, dir[1] as f64, dir[2] as f64];
            for b in 0..nboxes {
                let oracle = thessa_plume_core::cbt_volume::ray_intersect_slab(
                    [mins[b][0] as f64, mins[b][1] as f64, mins[b][2] as f64],
                    [maxs[b][0] as f64, maxs[b][1] as f64, maxs[b][2] as f64],
                    org64,
                    dir64,
                );
                let gpu = hits[r * nboxes + b];
                match oracle {
                    None => {
                        if !gpu.hit {
                            agree += 1;
                        } else {
                            // A GPU hit where f64 misses is only acceptable
                            // for a grazing-strength interval: t_exit ≈ 0.
                            assert!(
                                gpu.t_exit.abs() < 1e-3,
                                "spurious GPU hit r={r} b={b}: t_exit={}",
                                gpu.t_exit
                            );
                            graze_mismatch += 1;
                        }
                    }
                    Some((t0, t1)) => {
                        if !gpu.hit {
                            // Symmetric lane: oracle interval thinner than f32
                            // rounding can resolve.
                            assert!(
                                (t1 - t0) < 1e-3 * (1.0 + t1.abs()),
                                "missed GPU hit r={r} b={b}: oracle=({t0},{t1})"
                            );
                            graze_mismatch += 1;
                        } else {
                            agree += 1;
                            let rel = ((gpu.t_enter as f64 - t0).abs() / (1.0 + t0.abs()))
                                .max((gpu.t_exit as f64 - t1).abs() / (1.0 + t1.abs()));
                            worst_rel = worst_rel.max(rel);
                            assert!(
                                rel < 2e-3,
                                "t drift r={r} b={b}: gpu=({},{}) oracle=({t0},{t1})",
                                gpu.t_enter,
                                gpu.t_exit
                            );
                        }
                    }
                }
            }
        }
        let total = (nrays * nboxes) as u64;
        eprintln!("agree={agree} graze={graze_mismatch} worst_rel={worst_rel:.2e}");
        assert!(
            graze_mismatch * 1000 < total,
            "grazing lane must stay under 0.1%: {graze_mismatch}/{total}"
        );
    }

    #[test]
    fn empty_inputs_dispatch_nothing() {
        let Some((device, queue, _)) = try_device() else {
            eprintln!("SKIP: no GPU adapter for fastrt-wgpu empty test");
            return;
        };
        let mut backend = BatchSlab::new(device, queue);
        let out = backend
            .intersect(&[], &[], &[[0.0; 3]], &[[1.0; 3]])
            .expect("empty rays return empty");
        assert!(out.is_empty());
        let out = backend
            .intersect(&[[0.0; 3]], &[[1.0, 0.0, 0.0]], &[], &[])
            .expect("empty boxes return empty");
        assert!(out.is_empty());
    }
}
