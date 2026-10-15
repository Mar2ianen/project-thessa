//! Batched slab throughput: scalar f32 CPU loop vs the wgpu batch path.
//! Run with `cargo bench -p thessa-fastrt-wgpu --bench batch_slab`.
//! Prints SKIP without a GPU adapter. Both columns include their full
//! end-to-end cost (CPU: loop only; GPU: pack + upload + dispatch +
//! synchronous readback), so the ratio answers "which submits this batch
//! faster", not "which ALU is wider".

use std::{hint::black_box, time::Instant};

use thessa_fastrt_wgpu::{BatchSlab, try_device};

fn lcg(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    ((*state >> 11) as f64) / ((1u64 << 53) as f64)
}

type Triples = Vec<[f32; 3]>;
type Workload = (Triples, Triples, Triples, Triples);

fn workload(nrays: usize, nboxes: usize, seed: u64) -> Workload {
    let mut rng = seed;
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
        if d == [0.0, 0.0, 0.0] {
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
        let e = (0.5 + lcg(&mut rng) * 4.0) as f32;
        mins.push([c[0] as f32 - e, c[1] as f32 - e, c[2] as f32 - e]);
        maxs.push([c[0] as f32 + e, c[1] as f32 + e, c[2] as f32 + e]);
    }
    (origins, dirs, mins, maxs)
}

fn cpu_batch(origins: &[[f32; 3]], dirs: &[[f32; 3]], mins: &[[f32; 3]], maxs: &[[f32; 3]]) -> u64 {
    // Same slab math as the WGSL path, scalar f32: measures parallelization
    // and bandwidth, not precision (that is the parity test's job).
    let mut hits = 0u64;
    for r in 0..origins.len() {
        let (o, d) = (origins[r], dirs[r]);
        if d == [0.0, 0.0, 0.0] {
            continue;
        }
        for b in 0..mins.len() {
            let (mn, mx) = (mins[b], maxs[b]);
            let mut tmin = f32::NEG_INFINITY;
            let mut tmax = f32::INFINITY;
            let mut miss = false;
            for i in 0..3 {
                if d[i] == 0.0 {
                    if o[i] < mn[i] || o[i] > mx[i] {
                        miss = true;
                    }
                } else {
                    let inv = 1.0 / d[i];
                    let (mut t0, mut t1) = ((mn[i] - o[i]) * inv, (mx[i] - o[i]) * inv);
                    if t0 > t1 {
                        std::mem::swap(&mut t0, &mut t1);
                    }
                    tmin = tmin.max(t0);
                    tmax = tmax.min(t1);
                    if tmax < tmin {
                        miss = true;
                    }
                }
            }
            if !miss && tmax >= 0.0 {
                hits += 1;
            }
        }
    }
    hits
}

fn main() {
    println!("fastrt batch slab: scalar f32 CPU vs wgpu (end to end with readback)");
    println!(
        "{:>16} {:>12} {:>12} {:>12} {:>12}",
        "pairs", "cpu_ms", "gpu_ms", "cpu_Mpairs/s", "gpu_Mpairs/s"
    );
    let Some((device, queue, name)) = try_device() else {
        println!("SKIP: no GPU adapter available");
        return;
    };
    eprintln!("adapter: {name}");
    let mut backend = BatchSlab::new(device, queue);
    for (nrays, nboxes, iters) in [(4_096usize, 256usize, 10usize), (16_384, 256, 5)] {
        let (origins, dirs, mins, maxs) = workload(nrays, nboxes, 0x51A8);
        let pairs = (nrays * nboxes) as f64;
        // Warm up both paths (pipeline compile + allocator paths on GPU).
        black_box(cpu_batch(&origins, &dirs, &mins, &maxs));
        black_box(
            backend
                .intersect(&origins, &dirs, &mins, &maxs)
                .expect("warmup batch runs"),
        );
        let started = Instant::now();
        let mut hits = 0u64;
        for _ in 0..iters {
            hits += black_box(cpu_batch(
                black_box(&origins),
                black_box(&dirs),
                black_box(&mins),
                black_box(&maxs),
            ));
        }
        let cpu_ms = started.elapsed().as_secs_f64() * 1000.0 / iters as f64;
        let started = Instant::now();
        let mut gpu_hits = 0usize;
        for _ in 0..iters {
            gpu_hits += backend
                .intersect(
                    black_box(&origins),
                    black_box(&dirs),
                    black_box(&mins),
                    black_box(&maxs),
                )
                .expect("bench batch runs")
                .iter()
                .filter(|h| black_box(h.hit))
                .count();
        }
        let gpu_ms = started.elapsed().as_secs_f64() * 1000.0 / iters as f64;
        // Cross-check the counted hits agree (same workload, same math;
        // FMA contraction may flip grazing pairs, hence a 0.1% lane).
        // gpu_hits accumulates over iters: normalize before comparing.
        let cpu_once = cpu_batch(&origins, &dirs, &mins, &maxs);
        let total = (nrays * nboxes) as u64;
        let gpu_per_iter = gpu_hits as u64 / iters as u64;
        let drift = gpu_per_iter.abs_diff(cpu_once);
        assert!(
            drift * 1000 <= total,
            "gpu/cpu hit counts diverge: gpu={gpu_per_iter}/iter cpu={cpu_once}"
        );
        println!(
            "{:>16} {:>12.2} {:>12.2} {:>12.1} {:>12.1}",
            nrays * nboxes,
            cpu_ms,
            gpu_ms,
            pairs / 1e6 / (cpu_ms / 1e3),
            pairs / 1e6 / (gpu_ms / 1e3),
        );
        black_box((hits, gpu_hits));
    }
}
