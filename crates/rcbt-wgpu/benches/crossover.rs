//! Dirty-op sweep on a fixed tree: T_cpu(k), T_sparse(k), T_gpu(k).
//! Run with `cargo bench -p thessa-rcbt-wgpu --bench crossover`.
//!
//! Fixed full tree at depth 18 inside depth-20 heaps (262144 leaves, splits
//! valid to depth 19). Each k dirty ops are pre-generated once against a
//! driver mirror and replayed on all three sides with leaf-set parity. The
//! backend answers "where to commit this frame" from the measured
//! k_crossover, not from a hardcoded threshold.
//!
//! A second section sweeps the ancestor-combining cutoff at fixed k=4096.
//! Without a GPU device the bench prints SKIP and exits successfully.

use std::{hint::black_box, time::Instant};

use thessa_rcbt_core::packed::PackedTree;
use thessa_rcbt_core::{Node, Tree};
use thessa_rcbt_ffi::LibcbtTree;
use thessa_rcbt_wgpu::bench_support::{
    GpuCtx, create_apply_layout, create_decode_layout, create_pipeline, groups, readback_pairs,
    words_to_bytes,
};
use thessa_rcbt_wgpu::heap::CpuMirror;
use thessa_rcbt_wgpu::shaders::{CBT_APPLY_COMBINED_WGSL, CBT_APPLY_WGSL, CBT_DECODE_WGSL};

const MAX_DEPTH: u8 = 20;
const BASE_DEPTH: u8 = 18;

// Dynamic wave scenarios share the driver's constants with `dynamic.rs`.
const WAVE_DEPTH: u8 = 16;
const WAVE_BASE: u8 = 10;
const WAVE_MIN: u8 = 8;
const WAVE_FRAMES: usize = 120;
const WAVE_SPLIT_EPS: f64 = 1e-5;
const WAVE_MERGE_EPS: f64 = 2.5e-6;

fn wave_h(x: f64, t: f64) -> f64 {
    use std::f64::consts::TAU;
    (TAU * 3.0 * x + 0.9 * t).sin()
        + 0.35 * (TAU * 7.0 * x - 1.7 * t + 1.3).sin()
        + 0.12 * (TAU * 13.0 * x + 2.6 * t + 4.1).sin()
}

fn wave_leaf_err(id: u64, depth: u8, t: f64, cam: Option<f64>) -> f64 {
    let w = 2f64.powi(-(depth as i32));
    let a = (id - (1u64 << depth)) as f64 * w;
    let m = a + 0.5 * w;
    let e = (wave_h(m, t) - 0.5 * (wave_h(a, t) + wave_h(a + w, t))).abs();
    match cam {
        Some(c) => e / (0.05 + (m - c).abs()),
        None => e,
    }
}

/// Wave frames driven against a CPU mirror oracle. Returns per-frame
/// batches, per-frame expected leaf counts, and the final leaf list.
/// Batches from one frame are internally consistent in any apply order
/// (split needs err above SPLIT, merge needs both below MERGE), and only
/// share commutative upper-ancestor adds — safe for concurrent GPU replay.
fn drive_waves(camera_sweep: bool) -> (Vec<OpBatch>, Vec<u32>, LeafSet) {
    let mut driver = CpuMirror::new(WAVE_DEPTH).unwrap();
    driver.reset_full(WAVE_BASE).unwrap();
    let mut batches = Vec::with_capacity(WAVE_FRAMES);
    let mut counts = Vec::with_capacity(WAVE_FRAMES);
    for f in 0..WAVE_FRAMES {
        let t = f as f64 / 60.0;
        let cam = camera_sweep.then(|| f as f64 / WAVE_FRAMES as f64);
        let mut batch = OpBatch::new();
        let leaves = driver.leaves();
        for (id, depth) in &leaves {
            if *depth < WAVE_DEPTH && wave_leaf_err(*id, *depth, t, cam) > WAVE_SPLIT_EPS {
                batch.push((*id, *depth, 0));
            }
        }
        for (id, depth) in &leaves {
            if id & 1 == 1 || *depth == 0 {
                continue;
            }
            let sib = (*id + 1, *depth);
            if !driver.is_active(sib.0) {
                continue;
            }
            let parent_depth = *depth - 1;
            if parent_depth < WAVE_MIN {
                continue;
            }
            if wave_leaf_err(*id, *depth, t, cam) < WAVE_MERGE_EPS
                && wave_leaf_err(sib.0, sib.1, t, cam) < WAVE_MERGE_EPS
            {
                batch.push((id >> 1, parent_depth, 1));
            }
        }
        for (id, _, kind) in &batch {
            match kind {
                0 => driver.split(*id).unwrap(),
                _ => driver.merge_children(*id).unwrap(),
            }
        }
        counts.push(driver.node_count());
        black_box(driver.node_count());
        batches.push(batch);
    }
    let expected = driver.leaves();
    (batches, counts, expected)
}

fn node(id: u64, depth: u8) -> Node {
    Node::new(id, depth).expect("bench node in range")
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*, deterministic across runs.
        let mut x = self.0 | 1;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

/// Exactly `k` valid sparse ops (split leaf / merge valid pair) plus the
/// expected final leaf list. Half the attempts split, half merge; the live
/// set stays near full so every k up to 16384 is reachable.
///
/// All ops sit on ONE level with pairwise-disjoint depth-17 parents. This
/// is load-bearing, not incidental: sequentially-valid ops can still be
/// ancestor/descendant-related (split a node one op created for another),
/// which a sequential CPU replays fine but concurrent GPU threads race on
/// (bit set vs clear, store vs add on the same node). Shared upper ancestors
/// are safe — they only ever see commutative atomic adds. A production
/// classifier must guarantee the same property or run a resolve phase; see
/// docs/22 follow-ups.
/// One validated sparse batch plus the expected leaf list.
type OpBatch = Vec<(u64, u8, u8)>;
/// Ordered leaf list in `(heap id, depth)` form shared by all sides.
type LeafSet = Vec<(u64, u8)>;

fn gen_sparse_ops(k: usize, seed: u64) -> (OpBatch, LeafSet) {
    let mut driver = CpuMirror::new(MAX_DEPTH).unwrap();
    driver.reset_full(BASE_DEPTH).unwrap();
    let mut rng = Rng(seed);
    let mut touched = std::collections::HashSet::new();
    let mut ops = Vec::with_capacity(k);
    let mut attempts = 0;
    while ops.len() < k && attempts < k * 50 + 1000 {
        attempts += 1;
        let r = rng.next();
        // Depth-17 parents: 131072 disjoint regions, far more than max k.
        let parent = (1u64 << 17) + r % (1u64 << 17);
        if !touched.insert(parent) {
            continue;
        }
        // Untouched parent => both children are pristine depth-18 leaves.
        if r & 1 == 0 {
            let child = parent * 2 + (r >> 9) % 2;
            driver.split(child).expect("pristine child splits");
            ops.push((child, 18, 0));
        } else if driver.merge_children(parent).is_ok() {
            ops.push((parent, 17, 1));
        } else {
            touched.remove(&parent);
        }
    }
    assert_eq!(ops.len(), k, "sparse workload must reach exactly k ops");
    (ops, driver.leaves())
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
        label: Some("rcbt-crossover"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        experimental_features: wgpu::ExperimentalFeatures::default(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
    }))
    .expect("bench device");

    let apply_layout = create_apply_layout(&device);
    let decode_layout = create_decode_layout(&device);
    let apply_pipe = create_pipeline(&device, &apply_layout, CBT_APPLY_WGSL, "apply_ops");
    let decode_pipe = create_pipeline(&device, &decode_layout, CBT_DECODE_WGSL, "decode_all");
    let ctx = GpuCtx {
        device: &device,
        queue: &queue,
        apply_pipe: &apply_pipe,
        decode_pipe: &decode_pipe,
        apply_layout: &apply_layout,
        decode_layout: &decode_layout,
    };

    // Heap-sized buffers, allocated once and reused for every k.
    let probe = CpuMirror::new(MAX_DEPTH).unwrap();
    let mk_storage = |label: &str, size: u64| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let active = mk_storage("rcbt-active", probe.active_byte_len() as u64);
    let sums = mk_storage("rcbt-sums", probe.sums_byte_len() as u64);
    let params = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-params"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    // Worst case: base leaves plus every op splitting (+1 each).
    let max_leaves = (1_usize << BASE_DEPTH) + 16384;
    let out = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rcbt-out"),
        size: (max_leaves * 8) as u64,
        usage: wgpu::BufferUsages::STORAGE
            | wgpu::BufferUsages::COPY_DST
            | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });

    println!("| dirty_ops k | impl | ms | parity |");
    println!("|---|---|---|---|");
    let mut cross_cpu: Option<usize> = None;
    let mut cross_sparse: Option<usize> = None;
    for k in [4_usize, 16, 32, 64, 256, 1024, 4096, 16384] {
        let (ops, expected) = gen_sparse_ops(k, 0x9E3779B97F4A7C15 + k as u64);

        // CPU native: fresh full tree, timed apply only.
        let mut tree = Tree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
        let t = Instant::now();
        for (id, depth, kind) in &ops {
            match kind {
                0 => {
                    tree.split(node(*id, *depth)).unwrap();
                }
                _ => {
                    tree.merge(node(*id, *depth)).unwrap();
                }
            }
        }
        black_box(tree.leaf_count());
        let cpu_ms = t.elapsed().as_secs_f64() * 1000.0;
        let got_cpu: Vec<(u64, u8)> = tree.leaves().iter().map(|l| (l.id(), l.depth())).collect();
        assert_eq!(got_cpu, expected, "cpu k={k} parity");

        // CPU packed: same ops on the dense bitfield tree (8 MiB working
        // set, no allocation or pointer chasing in the hot path).
        let mut packed_tree = PackedTree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
        let t = Instant::now();
        for (id, depth, kind) in &ops {
            match kind {
                0 => {
                    packed_tree.split(node(*id, *depth)).unwrap();
                }
                _ => {
                    packed_tree.merge(node(*id, *depth)).unwrap();
                }
            }
        }
        black_box(packed_tree.leaf_count());
        let packed_ms = t.elapsed().as_secs_f64() * 1000.0;
        let got_packed: Vec<(u64, u8)> = packed_tree
            .leaves()
            .iter()
            .map(|l| (l.id(), l.depth()))
            .collect();
        assert_eq!(got_packed, expected, "packed k={k} parity");

        // libcbt sparse: fresh full heap, batch FFI + reduce-only.
        let mut sparse = LibcbtTree::new(MAX_DEPTH).unwrap();
        sparse.reset_to_depth(BASE_DEPTH);
        sparse.reduce_only();
        let packed: Vec<(u64, i64, u8)> =
            ops.iter().map(|(id, d, k)| (*id, *d as i64, *k)).collect();
        let t = Instant::now();
        sparse.apply_batch_ops(&packed);
        sparse.reduce_only();
        black_box(sparse.node_count());
        let sparse_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(sparse.leaves(), expected, "sparse k={k} parity");

        // GPU full: init upload (untimed setup) + apply + decode + readback.
        let mut init = CpuMirror::new(MAX_DEPTH).unwrap();
        init.reset_full(BASE_DEPTH).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let words = pack_ops(&ops);
        let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let t = Instant::now();
        queue.write_buffer(&ops_buf, 0, &words_to_bytes(&words));
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[MAX_DEPTH as u32, expected.len() as u32, 0, 0]),
        );
        submit_apply(&ctx, &active, &sums, &params, &ops_buf, groups(ops.len()));
        submit_decode(&ctx, &active, &sums, &params, &out, groups(expected.len()));
        let got_gpu = readback_pairs(&device, &queue, &out, expected.len());
        let gpu_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(got_gpu, expected, "gpu k={k} parity");

        // GPU commit-only: same apply, submit + poll, no decode/readback in
        // the timed section; parity via one untimed verify afterwards.
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let t = Instant::now();
        queue.write_buffer(&ops_buf, 0, &words_to_bytes(&words));
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[MAX_DEPTH as u32, expected.len() as u32, 0, 0]),
        );
        submit_apply(&ctx, &active, &sums, &params, &ops_buf, groups(ops.len()));
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("device poll");
        let commit_ms = t.elapsed().as_secs_f64() * 1000.0;
        submit_decode(&ctx, &active, &sums, &params, &out, groups(expected.len()));
        let verify = readback_pairs(&device, &queue, &out, expected.len());
        assert_eq!(verify, expected, "gpu-commit k={k} parity");

        println!("| k={k} | cpu-native | {cpu_ms:.3} | OK |");
        println!("| k={k} | cpu-packed | {packed_ms:.3} | OK |");
        println!("| k={k} | libcbt-sparse | {sparse_ms:.3} | OK |");
        println!("| k={k} | gpu-full | {gpu_ms:.3} | OK |");
        println!("| k={k} | gpu-commit | {commit_ms:.3} | OK |");
        if cross_cpu.is_none() && gpu_ms < cpu_ms {
            cross_cpu = Some(k);
        }
        if cross_sparse.is_none() && gpu_ms < sparse_ms {
            cross_sparse = Some(k);
        }
    }
    match cross_cpu {
        Some(k) => println!("k_crossover(gpu-full < cpu-native): k={k}"),
        None => println!("k_crossover(gpu-full < cpu-native): none in sweep"),
    }
    match cross_sparse {
        Some(k) => println!("k_crossover(gpu-full < libcbt-sparse): k={k}"),
        None => println!("k_crossover(gpu-full < libcbt-sparse): none in sweep"),
    }

    // Ancestor-combining cutoff sweep at fixed k=4096.
    println!("| cutoff | impl | ms | parity |");
    println!("|---|---|---|---|");
    let (ops4096, expected4096) = gen_sparse_ops(4096, 0xC0FFEE);
    let combined_pipe = create_pipeline(
        &device,
        &apply_layout,
        CBT_APPLY_COMBINED_WGSL,
        "apply_ops_combined",
    );
    for cutoff in [0_u32, 6, 8, 10] {
        let mut init = CpuMirror::new(MAX_DEPTH).unwrap();
        init.reset_full(BASE_DEPTH).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let words = pack_ops(&ops4096);
        let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let t = Instant::now();
        queue.write_buffer(&ops_buf, 0, &words_to_bytes(&words));
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[MAX_DEPTH as u32, expected4096.len() as u32, cutoff, 0]),
        );
        submit_apply_with(
            &ctx,
            &combined_pipe,
            &active,
            &sums,
            &params,
            &ops_buf,
            groups(ops4096.len()),
        );
        submit_decode(
            &ctx,
            &active,
            &sums,
            &params,
            &out,
            groups(expected4096.len()),
        );
        let got = readback_pairs(&device, &queue, &out, expected4096.len());
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(got, expected4096, "combined cutoff={cutoff} parity");
        println!("| cutoff={cutoff} | gpu-combined | {ms:.3} | OK |");
    }

    // Dense batch: split a FULL level-15 front (32768 ops, one batch) — the
    // worst case for upper-node atomic contention. Apply-only timing
    // (submit + poll, no decode); parity via one untimed verify each.
    println!("| dense-32k | impl | apply_ms | parity |");
    println!("|---|---|---|---|");
    let mut driver = CpuMirror::new(MAX_DEPTH).unwrap();
    driver.reset_full(15).unwrap();
    let dense: Vec<(u64, u8, u8)> = driver.leaves().iter().map(|(id, d)| (*id, *d, 0)).collect();
    assert_eq!(dense.len(), 32768);
    // CPU packed on the identical dense batch (fresh full-15 tree).
    {
        let mut pt = PackedTree::at_depth(MAX_DEPTH, 15).unwrap();
        let t = Instant::now();
        for (id, depth, _) in &dense {
            pt.split(node(*id, *depth)).unwrap();
        }
        black_box(pt.leaf_count());
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        let got: Vec<(u64, u8)> = pt.leaves().iter().map(|l| (l.id(), l.depth())).collect();
        let mut m = CpuMirror::new(MAX_DEPTH).unwrap();
        m.reset_full(15).unwrap();
        for (id, _, _) in &dense {
            m.split(*id).unwrap();
        }
        assert_eq!(got, m.leaves(), "dense packed parity");
        println!("| dense-32k | cpu-packed | {ms:.3} | OK |");
    }
    let dense_words = pack_ops(&dense);
    let dense_expected = {
        let mut m = CpuMirror::new(MAX_DEPTH).unwrap();
        m.reset_full(15).unwrap();
        for (id, _, _) in &dense {
            m.split(*id).unwrap();
        }
        m.leaves()
    };
    // Baseline apply.
    {
        let mut init = CpuMirror::new(MAX_DEPTH).unwrap();
        init.reset_full(15).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (dense_words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let t = Instant::now();
        queue.write_buffer(&ops_buf, 0, &words_to_bytes(&dense_words));
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[MAX_DEPTH as u32, dense_expected.len() as u32, 0, 0]),
        );
        submit_apply_with(
            &ctx,
            &apply_pipe,
            &active,
            &sums,
            &params,
            &ops_buf,
            groups(dense.len()),
        );
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("device poll");
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        submit_decode(
            &ctx,
            &active,
            &sums,
            &params,
            &out,
            groups(dense_expected.len()),
        );
        let got = readback_pairs(&device, &queue, &out, dense_expected.len());
        assert_eq!(got, dense_expected, "dense baseline parity");
        println!("| dense-32k | gpu-apply | {ms:.3} | OK |");
    }
    // Combined apply at several cutoffs.
    for cutoff in [0_u32, 6, 8, 10] {
        let mut init = CpuMirror::new(MAX_DEPTH).unwrap();
        init.reset_full(15).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rcbt-ops"),
            size: (dense_words.len() * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let t = Instant::now();
        queue.write_buffer(&ops_buf, 0, &words_to_bytes(&dense_words));
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[MAX_DEPTH as u32, dense_expected.len() as u32, cutoff, 0]),
        );
        submit_apply_with(
            &ctx,
            &combined_pipe,
            &active,
            &sums,
            &params,
            &ops_buf,
            groups(dense.len()),
        );
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("device poll");
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        submit_decode(
            &ctx,
            &active,
            &sums,
            &params,
            &out,
            groups(dense_expected.len()),
        );
        let got = readback_pairs(&device, &queue, &out, dense_expected.len());
        assert_eq!(got, dense_expected, "dense combined cutoff={cutoff} parity");
        println!("| dense-32k | gpu-combined cutoff={cutoff} | {ms:.3} | OK |");
    }

    // Dynamic wave scenarios on GPU: same mixed batches the `dynamic` CPU
    // bench drives, replayed here with per-frame readback (renderer cost).
    println!("| dynamic | impl | frames | avg_ops_frame | ms | parity |");
    println!("|---|---|---|---|---|---|");
    for camera_sweep in [false, true] {
        let name = if camera_sweep {
            "waves+camera"
        } else {
            "waves"
        };
        let (frames, expected_counts, expected_final) = drive_waves(camera_sweep);
        let total_ops: usize = frames.iter().map(Vec::len).sum();
        let avg = total_ops as f64 / frames.len() as f64;
        let mut init = CpuMirror::new(WAVE_DEPTH).unwrap();
        init.reset_full(WAVE_BASE).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let t = Instant::now();
        for (batch, count) in frames.iter().zip(expected_counts.iter()) {
            let words = pack_ops(batch);
            let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rcbt-ops"),
                size: (words.len() * 4).max(16) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&ops_buf, 0, &words_to_bytes(&words));
            queue.write_buffer(
                &params,
                0,
                &words_to_bytes(&[WAVE_DEPTH as u32, *count, 0, 0]),
            );
            submit_apply(
                &ctx,
                &active,
                &sums,
                &params,
                &ops_buf,
                groups(batch.len().max(1)),
            );
            submit_decode(&ctx, &active, &sums, &params, &out, groups(*count as usize));
            let got = readback_pairs(&device, &queue, &out, *count as usize);
            assert_eq!(got.len(), *count as usize, "{name} gpu frame count");
            black_box(got.len());
        }
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        // Final full-equality check against the driver oracle.
        let mut verify = CpuMirror::new(WAVE_DEPTH).unwrap();
        verify.reset_full(WAVE_BASE).unwrap();
        for batch in &frames {
            for (id, _, kind) in batch {
                match kind {
                    0 => verify.split(*id).unwrap(),
                    _ => verify.merge_children(*id).unwrap(),
                }
            }
        }
        assert_eq!(verify.leaves(), expected_final, "{name} gpu-dynamic parity");
        println!(
            "| {name} | gpu-dynamic | {} | {avg:.0} | {ms:.2} | OK |",
            frames.len()
        );
        // Commit-only twin: same applies, submit + poll, no decode/readback
        // in the timed section. The gap between the two columns is pure
        // host-sync cost — the quantitative case for a persistent GPU-side
        // leaf list (docs/22 follow-up 2).
        let mut init = CpuMirror::new(WAVE_DEPTH).unwrap();
        init.reset_full(WAVE_BASE).unwrap();
        queue.write_buffer(&active, 0, &words_to_bytes(init.active_words()));
        queue.write_buffer(&sums, 0, &words_to_bytes(init.sums()));
        let t = Instant::now();
        for batch in frames.iter() {
            if batch.is_empty() {
                continue;
            }
            let words = pack_ops(batch);
            let ops_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rcbt-ops"),
                size: (words.len() * 4).max(16) as u64,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            queue.write_buffer(&ops_buf, 0, &words_to_bytes(&words));
            queue.write_buffer(&params, 0, &words_to_bytes(&[WAVE_DEPTH as u32, 0, 0, 0]));
            submit_apply(&ctx, &active, &sums, &params, &ops_buf, groups(batch.len()));
            device
                .poll(wgpu::PollType::wait_indefinitely())
                .expect("device poll");
            black_box(batch.len());
        }
        let commit_ms = t.elapsed().as_secs_f64() * 1000.0;
        // Untimed verify: one decode + readback, asserted but not billed.
        queue.write_buffer(
            &params,
            0,
            &words_to_bytes(&[WAVE_DEPTH as u32, expected_final.len() as u32, 0, 0]),
        );
        submit_decode(
            &ctx,
            &active,
            &sums,
            &params,
            &out,
            groups(expected_final.len()),
        );
        let verify_commit = readback_pairs(&device, &queue, &out, expected_final.len());
        assert_eq!(
            verify_commit, expected_final,
            "{name} gpu-dynamic-commit parity"
        );
        println!(
            "| {name} | gpu-dynamic-commit | {} | {avg:.0} | {commit_ms:.2} | OK |",
            frames.len()
        );
    }
}

fn pack_ops(ops: &[(u64, u8, u8)]) -> Vec<u32> {
    let mut words = Vec::with_capacity(ops.len() * 4);
    for (id, depth, kind) in ops {
        words.push(*id as u32);
        words.push(*depth as u32);
        words.push(*kind as u32);
        words.push(0);
    }
    words
}

fn bind_all(
    ctx: &GpuCtx<'_>,
    layout: &wgpu::BindGroupLayout,
    params: &wgpu::Buffer,
    active: &wgpu::Buffer,
    sums: &wgpu::Buffer,
    slot3: &wgpu::Buffer,
) -> wgpu::BindGroup {
    ctx.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("rcbt-crossover-bg"),
        layout,
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
                resource: slot3.as_entire_binding(),
            },
        ],
    })
}

fn submit_apply(
    ctx: &GpuCtx<'_>,
    active: &wgpu::Buffer,
    sums: &wgpu::Buffer,
    params: &wgpu::Buffer,
    ops: &wgpu::Buffer,
    workgroups: u32,
) {
    submit_apply_with(ctx, ctx.apply_pipe, active, sums, params, ops, workgroups);
}

fn submit_apply_with(
    ctx: &GpuCtx<'_>,
    pipe: &wgpu::ComputePipeline,
    active: &wgpu::Buffer,
    sums: &wgpu::Buffer,
    params: &wgpu::Buffer,
    ops: &wgpu::Buffer,
    workgroups: u32,
) {
    let bg = bind_all(ctx, ctx.apply_layout, params, active, sums, ops);
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rcbt-crossover-apply"),
        });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("rcbt-apply"),
            timestamp_writes: None,
        });
        pass.set_pipeline(pipe);
        pass.set_bind_group(0, &bg, &[]);
        pass.dispatch_workgroups(workgroups, 1, 1);
    }
    ctx.queue.submit(Some(encoder.finish()));
}

fn submit_decode(
    ctx: &GpuCtx<'_>,
    active: &wgpu::Buffer,
    sums: &wgpu::Buffer,
    params: &wgpu::Buffer,
    out: &wgpu::Buffer,
    workgroups: u32,
) {
    let bg = bind_all(ctx, ctx.decode_layout, params, active, sums, out);
    let mut encoder = ctx
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rcbt-crossover-decode"),
        });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some("rcbt-decode"),
            timestamp_writes: None,
        });
        pass.set_pipeline(ctx.decode_pipe);
        pass.set_bind_group(0, &bg, &[]);
        pass.dispatch_workgroups(workgroups, 1, 1);
    }
    ctx.queue.submit(Some(encoder.finish()));
}
