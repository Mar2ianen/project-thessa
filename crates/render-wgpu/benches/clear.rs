//! Backend submit+sync floor: clear + readback latency on fixed targets.
//! Run with `cargo bench -p thessa-render-wgpu --bench clear`.
//! Prints SKIP without a GPU adapter. Measures one submit with a present-
//! shaped barrier (render pass + copy + synchronous map), i.e. the fixed
//! per-frame tax every future render loop pays before any draw work.

use std::{hint::black_box, time::Instant};

use thessa_render_wgpu::Backend;

fn main() {
    let backend = match Backend::bring_up() {
        Ok(backend) => backend,
        Err(thessa_render_wgpu::BackendError::NoAdapter) => {
            println!("SKIP: no GPU adapter available");
            return;
        }
        Err(other) => panic!("bring-up failed: {other}"),
    };
    eprintln!("adapter: {}", backend.adapter_name);
    println!("render-wgpu clear+readback floor (submit+sync, exact pixels)");
    println!("{:>16} {:>12}", "target", "ms");
    for (w, h, iters) in [(64u32, 64u32, 50usize), (256, 256, 50), (1024, 1024, 20)] {
        // Warm up (first submit compiles nothing here, but it shakes out
        // allocator and first-map paths).
        black_box(
            backend
                .clear_readback(w, h, [0.0, 0.0, 0.0, 1.0])
                .expect("warmup clear runs"),
        );
        let started = Instant::now();
        let mut bytes = 0usize;
        for _ in 0..iters {
            bytes += black_box(
                backend
                    .clear_readback(black_box(w), black_box(h), [0.1, 0.2, 0.3, 1.0])
                    .expect("bench clear runs"),
            )
            .len();
        }
        let ms = started.elapsed().as_secs_f64() * 1000.0 / iters as f64;
        println!("{w}x{h:>13} {ms:>12.3}");
        black_box(bytes);
    }
}
