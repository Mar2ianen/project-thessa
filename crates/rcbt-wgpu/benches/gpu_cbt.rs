//! GPU sparse-commit CBT vs native CPU Tree vs libcbt (serial + sparse).
//! Run with `cargo bench -p thessa-rcbt-wgpu --bench gpu_cbt`.
//!
//! Same refine workloads replayed in lockstep on all five sides; every leaf
//! list is asserted equal to the CPU oracle. GPU time is end-to-end wall
//! clock around submit + blocking readback. Columns per depth:
//! cpu-native / libcbt-public / libcbt-sparse / gpu / gpu+readback.
//! Without a GPU device the bench prints SKIP and exits successfully.

use std::{hint::black_box, time::Instant};

use thessa_rcbt_core::{Node, Tree};
use thessa_rcbt_ffi::LibcbtTree;
use thessa_rcbt_wgpu::heap::CpuMirror;
use thessa_rcbt_wgpu::shaders::{CBT_APPLY_WGSL, CBT_DECODE_WGSL};

const WORKGROUP: u32 = 64;

fn node(id: u64, depth: u8) -> Node {
    Node::new(id, depth).expect("bench node in range")
}

fn groups(n: usize) -> u32 {
    ((n as u32).div_ceil(WORKGROUP)).max(1)
}

/// Refine batches: split every leaf, level by level, down to `depth`.
fn gen_refine(depth: u8) -> Vec<Vec<(u64, u8)>> {
    let mut tree = Tree::new(24).unwrap();
    let mut out = Vec::new();
    for _ in 0..depth {
        let batch: Vec<(u64, u8)> = tree.leaves().iter().map(|l| (l.id(), l.depth())).collect();
        for (id, d) in &batch {
            tree.split(node(*id, *d)).unwrap();
        }
        out.push(batch);
    }
    out
}

fn main() {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
    }));
    let Ok(adapter) = adapter else {
        println!("SKIP: no GPU adapter available");
        return;
    };
    let info = adapter.get_info();
    println!("adapter: {} ({:?})", info.name, info.backend);
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("rcbt-bench"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        experimental_features: wgpu::ExperimentalFeatures::default(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
    }))
    .expect("bench device");

    // Shared bind-group layout shape: uniform + active + sums + slot3.
    // Slot 3 differs (read-only ops vs read-write out), so two layouts.
    let entry = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: if binding == 0 {
                wgpu::BufferBindingType::Uniform
            } else {
                wgpu::BufferBindingType::Storage { read_only }
            },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let apply_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("rcbt-apply-layout"),
        entries: &[
            entry(0, true),
            entry(1, false),
            entry(2, false),
            entry(3, true),
        ],
    });
    let decode_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("rcbt-decode-layout"),
        entries: &[
            entry(0, true),
            entry(1, false),
            entry(2, false),
            entry(3, false),
        ],
    });
    let mk_pipeline = |layout: &wgpu::BindGroupLayout, src: &str, ep: &str| {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rcbt-bench-shader"),
            source: wgpu::ShaderSource::Wgsl(src.into()),
        });
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rcbt-bench-pl"),
            bind_group_layouts: &[Some(layout)],
            immediate_size: 0,
        });
        device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some(ep),
            layout: Some(&pl),
            module: &module,
            entry_point: Some(ep),
            compilation_options: Default::default(),
            cache: None,
        })
    };
    let apply_pipe = mk_pipeline(&apply_layout, CBT_APPLY_WGSL, "apply_ops");
    let decode_pipe = mk_pipeline(&decode_layout, CBT_DECODE_WGSL, "decode_all");
    let ctx = GpuCtx {
        device: &device,
        queue: &queue,
        apply_pipe: &apply_pipe,
        decode_pipe: &decode_pipe,
        apply_layout: &apply_layout,
        decode_layout: &decode_layout,
    };

    // Mixed-op correctness prologue on real hardware before timing.
    {
        let mirror_depth = 8u8;
        let mut mirror = CpuMirror::new(mirror_depth).unwrap();
        mirror.split(1).unwrap();
        mirror.split(2).unwrap();
        mirror.merge_children(2).unwrap();
        let expected = mirror.leaves();
        assert_eq!(expected, vec![(2, 1), (3, 1)]);
        let (got, _) = run_batches(
            &ctx,
            mirror_depth,
            &[vec![(1, 0, 0)], vec![(2, 1, 0)], vec![(2, 1, 1)]],
            true,
        );
        assert_eq!(got, expected, "GPU mixed split/merge parity");
        println!("prologue: GPU mixed split/merge parity OK");
    }

    println!("| workload | impl | leaves | ms | parity |");
    println!("|---|---|---|---|---|");
    for depth in [12_u8, 14, 16, 18] {
        let batches = gen_refine(depth);
        // Native oracle replay (timed separately).
        let mut oracle = Tree::new(depth).unwrap();
        let t = Instant::now();
        for batch in &batches {
            for (id, d) in batch {
                oracle.split(node(*id, *d)).unwrap();
            }
            black_box(oracle.leaf_count());
        }
        let native_ms = t.elapsed().as_secs_f64() * 1000.0;
        let expected: Vec<(u64, u8)> = oracle
            .leaves()
            .iter()
            .map(|l| (l.id(), l.depth()))
            .collect();

        // libcbt-public: per-op FFI writes + full cbt_Update decode+reduce.
        let mut cbt = LibcbtTree::new(depth).expect("libcbt tree");
        let t = Instant::now();
        for batch in &batches {
            for (id, d) in batch {
                cbt.split(*id, *d);
            }
            cbt.reduce();
            black_box(cbt.node_count());
        }
        let public_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(
            cbt.leaves(),
            expected,
            "libcbt-public refine/d{depth} parity"
        );

        // libcbt-sparse: one batch FFI call + reduce-only, no decode pass.
        let mut sparse = LibcbtTree::new(depth).expect("libcbt sparse tree");
        let t = Instant::now();
        for batch in &batches {
            let packed: Vec<(u64, i64, u8)> =
                batch.iter().map(|(id, d)| (*id, *d as i64, 0)).collect();
            sparse.apply_batch_ops(&packed);
            sparse.reduce_only();
            black_box(sparse.node_count());
        }
        let sparse_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(
            sparse.leaves(),
            expected,
            "libcbt-sparse refine/d{depth} parity"
        );

        // GPU replay of split-only batches, two readback regimes: per-batch
        // (paranoid verification) and final-only (what a renderer pays).
        let split_batches: Vec<Vec<(u64, u8, u8)>> = batches
            .iter()
            .map(|b| b.iter().map(|(id, d)| (*id, *d, 0)).collect())
            .collect();
        let t = Instant::now();
        let (got, phases_v) = run_batches(&ctx, depth, &split_batches, true);
        let gpu_verify_ms = t.elapsed().as_secs_f64() * 1000.0;
        let parity = got == expected;
        assert!(parity, "GPU refine/d{depth} parity");
        let t = Instant::now();
        let (got_fast, phases_f) = run_batches(&ctx, depth, &split_batches, false);
        let gpu_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(got_fast, expected, "GPU fast-path refine/d{depth} parity");
        println!(
            "| refine/d{depth} | gpu+readback | {} | {gpu_verify_ms:.1} | {} |",
            expected.len(),
            if parity { "OK" } else { "MISMATCH" }
        );
        println!(
            "| refine/d{depth} | gpu | {} | {gpu_ms:.1} | OK |",
            expected.len()
        );
        // UMA/batched path, gated by capability (integrated GPU), never by
        // vendor. Discrete GPUs keep the staging path above as the number.
        if matches!(info.device_type, wgpu::DeviceType::IntegratedGpu) {
            let t = Instant::now();
            let (got_uma, phases_uma) = run_batches_uma(&ctx, depth, &split_batches);
            let uma_ms = t.elapsed().as_secs_f64() * 1000.0;
            assert_eq!(got_uma, expected, "GPU-UMA refine/d{depth} parity");
            println!(
                "| refine/d{depth} | gpu-uma | {} | {uma_ms:.1} | OK |",
                expected.len()
            );
            println!(
                "| refine/d{depth} | uma-phases(alloc/upload/submit/readback) | {:.2}/{:.2}/{:.2}/{:.2} |",
                phases_uma.alloc_ms,
                phases_uma.upload_ms,
                phases_uma.submit_ms,
                phases_uma.readback_ms
            );
        } else {
            println!("| refine/d{depth} | gpu-uma | skipped (not integrated GPU) |");
        }
        println!(
            "| refine/d{depth} | phases(alloc/upload/submit/readback) | {:.2}/{:.2}/{:.2}/{:.2} |",
            phases_f.alloc_ms, phases_f.upload_ms, phases_f.submit_ms, phases_f.readback_ms
        );
        black_box(phases_v.total_ms());
        println!(
            "| refine/d{depth} | cpu-native | {} | {native_ms:.1} | OK |",
            expected.len()
        );
        println!(
            "| refine/d{depth} | libcbt-public | {} | {public_ms:.1} | OK |",
            expected.len()
        );
        println!(
            "| refine/d{depth} | libcbt-sparse | {} | {sparse_ms:.1} | OK |",
            expected.len()
        );
    }
}

/// Shared GPU objects for one bench run (keeps `run_batches` small).
struct GpuCtx<'a> {
    device: &'a wgpu::Device,
    queue: &'a wgpu::Queue,
    apply_pipe: &'a wgpu::ComputePipeline,
    decode_pipe: &'a wgpu::ComputePipeline,
    apply_layout: &'a wgpu::BindGroupLayout,
    decode_layout: &'a wgpu::BindGroupLayout,
}

/// Wall-clock breakdown of one GPU replay. `submit_ms` covers enqueue only;
/// real GPU completion lands in `readback_ms` (blocking poll), and
/// `write_buffer` only stages — so read `submit` vs `readback` as
/// host-overhead vs device+sync, not as kernel time. Kernel-only time needs
/// timestamp queries (future work, see docs/22).
#[derive(Default, Debug)]
struct GpuPhases {
    alloc_ms: f64,
    upload_ms: f64,
    submit_ms: f64,
    readback_ms: f64,
}

impl GpuPhases {
    fn total_ms(&self) -> f64 {
        self.alloc_ms + self.upload_ms + self.submit_ms + self.readback_ms
    }
}

/// Replay split/merge batches on GPU, returning the final decoded leaf list.
/// Each batch: upload ops -> apply -> decode with fresh count. With
/// `verify_each`, every batch is read back (paranoid mode); otherwise only
/// the final state leaves the GPU (renderer mode).
fn run_batches(
    ctx: &GpuCtx<'_>,
    max_depth: u8,
    batches: &[Vec<(u64, u8, u8)>],
    verify_each: bool,
) -> (Vec<(u64, u8)>, GpuPhases) {
    let mut phases = GpuPhases::default();
    let mirror = CpuMirror::new(max_depth).unwrap();
    let active_len = mirror.active_byte_len() as u64;
    let sums_len = mirror.sums_byte_len() as u64;
    let mk_storage = |label: &str, size: u64| {
        ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let active = mk_storage("rcbt-active", active_len);
    let sums = mk_storage("rcbt-sums", sums_len);
    ctx.queue
        .write_buffer(&active, 0, &words_to_bytes(mirror.active_words()));
    ctx.queue
        .write_buffer(&sums, 0, &words_to_bytes(mirror.sums()));
    let params = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-params"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    // Worst-case leaf count for the out buffer: full level max_depth.
    let max_leaves = 1_usize << max_depth;
    let out = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-out"),
        size: (max_leaves * 8) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });

    // Track the oracle count on the CPU side by replaying into the mirror.
    let mut shadow = CpuMirror::new(max_depth).unwrap();
    let mut last: Vec<(u64, u8)> = vec![(1, 0)];

    for batch in batches {
        // Upload ops as vec4 array (id, depth, kind, pad).
        let mut words = Vec::with_capacity(batch.len() * 4);
        for (id, depth, kind) in batch {
            words.push(*id as u32);
            words.push(*depth as u32);
            words.push(*kind as u32);
            words.push(0);
            match kind {
                0 => shadow.split(*id).unwrap(),
                _ => shadow.merge_children(*id).unwrap(),
            }
        }
        let t = Instant::now();
        let ops = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        phases.alloc_ms += t.elapsed().as_secs_f64() * 1000.0;
        let t = Instant::now();
        ctx.queue.write_buffer(&ops, 0, &words_to_bytes(&words));
        let count = shadow.node_count();
        ctx.queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[max_depth as u32, count, 0, 0]),
        );
        phases.upload_ms += t.elapsed().as_secs_f64() * 1000.0;

        let t = Instant::now();
        let apply_bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rcbt-apply-bg"),
            layout: ctx.apply_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: active.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: sums.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: ops.as_entire_binding(),
                },
            ],
        });
        let decode_bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rcbt-decode-bg"),
            layout: ctx.decode_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: active.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: sums.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: out.as_entire_binding(),
                },
            ],
        });
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("rcbt-batch"),
            });
        phases.alloc_ms += t.elapsed().as_secs_f64() * 1000.0;
        let t = Instant::now();
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rcbt-apply"),
                timestamp_writes: None,
            });
            pass.set_pipeline(ctx.apply_pipe);
            pass.set_bind_group(0, &apply_bg, &[]);
            pass.dispatch_workgroups(groups(batch.len()), 1, 1);
        }
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rcbt-decode"),
                timestamp_writes: None,
            });
            pass.set_pipeline(ctx.decode_pipe);
            pass.set_bind_group(0, &decode_bg, &[]);
            pass.dispatch_workgroups(groups(count as usize), 1, 1);
        }
        ctx.queue.submit(Some(encoder.finish()));
        phases.submit_ms += t.elapsed().as_secs_f64() * 1000.0;
        if verify_each {
            let t = Instant::now();
            last = readback_pairs(ctx.device, ctx.queue, &out, count as usize);
            phases.readback_ms += t.elapsed().as_secs_f64() * 1000.0;
        }
    }
    if !verify_each {
        // One final decode+readback so parity is still asserted end to end.
        // The count comes from the CPU shadow mirror: zero host round-trips
        // during the batch loop.
        let final_count = shadow.node_count() as usize;
        let mut encoder = ctx
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("rcbt-final-decode"),
            });
        let decode_bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rcbt-decode-bg-final"),
            layout: ctx.decode_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: active.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: sums.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: out.as_entire_binding(),
                },
            ],
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rcbt-decode-final"),
                timestamp_writes: None,
            });
            pass.set_pipeline(ctx.decode_pipe);
            pass.set_bind_group(0, &decode_bg, &[]);
            pass.dispatch_workgroups(groups(final_count), 1, 1);
        }
        ctx.queue.submit(Some(encoder.finish()));
        let t = Instant::now();
        last = readback_pairs(ctx.device, ctx.queue, &out, final_count);
        phases.readback_ms += t.elapsed().as_secs_f64() * 1000.0;
    }
    (last, phases)
}

/// UMA/batched fast path: one submit for the whole workload.
///
/// Differences vs `run_batches(verify_each=false)` (kept side by side so the
/// win is measured, not claimed):
/// - a single command encoder holds all apply passes (one submit total,
///   not one per batch);
/// - decode runs ONCE at the end with the final count (the per-batch
///   decodes of the baseline are pure waste: counts are known upfront from
///   the deterministic shadow replay);
/// - the readback staging buffer is created once and reused.
///
/// Deliberately NOT claimed: zero-copy. Portable wgpu forbids MAP_READ on
/// STORAGE bindings and restricts `mapped_at_creation` to MAP_WRITE|COPY_SRC
/// (see wgpu-core validation), so the out->staging copy stays. On unified
/// memory that copy is a plain memcpy; the measured columns show exactly
/// how much of the bill it is.
fn run_batches_uma(
    ctx: &GpuCtx<'_>,
    max_depth: u8,
    batches: &[Vec<(u64, u8, u8)>],
) -> (Vec<(u64, u8)>, GpuPhases) {
    let mut phases = GpuPhases::default();
    let t = Instant::now();
    let mirror = CpuMirror::new(max_depth).unwrap();
    let mk_storage = |label: &str, size: u64| {
        ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let active = mk_storage("rcbt-active", mirror.active_byte_len() as u64);
    let sums = mk_storage("rcbt-sums", mirror.sums_byte_len() as u64);
    let params = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-params"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let max_leaves = 1_usize << max_depth;
    let out = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-out"),
        size: (max_leaves * 8) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    // Reusable staging for the single final readback (no per-batch alloc).
    let staging = ctx.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-readback-reused"),
        size: (max_leaves * 8) as u64,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    phases.alloc_ms += t.elapsed().as_secs_f64() * 1000.0;

    // Deterministic shadow replay first: final count known upfront.
    let mut shadow = CpuMirror::new(max_depth).unwrap();
    let mut packed: Vec<Vec<u32>> = Vec::with_capacity(batches.len());
    for batch in batches {
        let mut words = Vec::with_capacity(batch.len() * 4);
        for (id, depth, kind) in batch {
            words.push(*id as u32);
            words.push(*depth as u32);
            words.push(*kind as u32);
            words.push(0);
            match kind {
                0 => shadow.split(*id).unwrap(),
                _ => shadow.merge_children(*id).unwrap(),
            }
        }
        packed.push(words);
    }
    let final_count = shadow.node_count();

    let t = Instant::now();
    ctx.queue
        .write_buffer(&active, 0, &words_to_bytes(mirror.active_words()));
    ctx.queue
        .write_buffer(&sums, 0, &words_to_bytes(mirror.sums()));
    ctx.queue.write_buffer(
        &params,
        0,
        &words_to_bytes(&[max_depth as u32, final_count, 0, 0]),
    );
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rcbt-uma-batch"),
        });
    // One encoder, one submit: every apply pass back to back, then a single
    // decode. wgpu orders passes within one submission; no host round-trip
    // between batches at all.
    for words in &packed {
        let ops = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        ctx.queue.write_buffer(&ops, 0, &words_to_bytes(words));
        let apply_bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rcbt-apply-bg"),
            layout: ctx.apply_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: params.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: active.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: sums.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: ops.as_entire_binding(),
                },
            ],
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("rcbt-apply"),
                timestamp_writes: None,
            });
            pass.set_pipeline(ctx.apply_pipe);
            pass.set_bind_group(0, &apply_bg, &[]);
            pass.dispatch_workgroups(groups(words.len() / 4), 1, 1);
        }
    }
    let decode_bg = ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("rcbt-decode-bg"),
        layout: ctx.decode_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: params.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: active.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: sums.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: out.as_entire_binding(),
            },
        ],
    });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("rcbt-decode"),
            timestamp_writes: None,
        });
        pass.set_pipeline(ctx.decode_pipe);
        pass.set_bind_group(0, &decode_bg, &[]);
        pass.dispatch_workgroups(groups(final_count as usize), 1, 1);
    }
    phases.upload_ms += t.elapsed().as_secs_f64() * 1000.0;
    let t = Instant::now();
    ctx.queue.submit(Some(encoder.finish()));
    phases.submit_ms += t.elapsed().as_secs_f64() * 1000.0;
    let t = Instant::now();
    let (last, _) = readback_pairs_reused(
        ctx.device,
        ctx.queue,
        &out,
        final_count as usize,
        Some(&staging),
    );
    phases.readback_ms += t.elapsed().as_secs_f64() * 1000.0;
    (last, phases)
}

fn readback_pairs(
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
fn readback_pairs_reused(
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

fn words_to_bytes(words: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(words.len() * 4);
    for w in words {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}
