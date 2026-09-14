//! Incremental compact OCBT mirror versus the upstream full reduction path.
//!
//! Run with:
//! `cargo bench -p thessa-rcbt-wgpu --bench ocbt_vs_reduce`

use std::hint::black_box;
use std::time::Instant;

use thessa_rcbt_large_ffi::{LargeOcbt, Variant};
use thessa_rcbt_wgpu::ocbt::OcbtPoolMirror;

const FRAMES: usize = 256;
const OPS_PER_FRAME: usize = 64;
const BIT_COUNT: usize = 1 << 20;

fn report(label: &str, started: Instant, operations: usize) {
    let seconds = started.elapsed().as_secs_f64();
    println!(
        "{label}: {operations} operations in {:.2} ms ({:.3}M operations/s)",
        seconds * 1000.0,
        operations as f64 / seconds / 1_000_000.0
    );
}

fn main() {
    let mut mirror = OcbtPoolMirror::new(20).unwrap();
    let started = Instant::now();
    for frame in 0..FRAMES {
        for op in 0..OPS_PER_FRAME {
            let bit = (frame * OPS_PER_FRAME * 17 + op * 65_537) % BIT_COUNT;
            mirror.set_bit(bit, ((frame + op) & 1) == 0).unwrap();
        }
        black_box(mirror.bit_count());
    }
    report(
        "thessa OCBT incremental path",
        started,
        FRAMES * OPS_PER_FRAME,
    );

    let mut reference = LargeOcbt::new(Variant::Ocbt1m).unwrap();
    let started = Instant::now();
    for frame in 0..FRAMES {
        for op in 0..OPS_PER_FRAME {
            let bit = (frame * OPS_PER_FRAME * 17 + op * 65_537) % BIT_COUNT;
            reference.set_bit(bit, ((frame + op) & 1) == 0);
        }
        reference.reduce();
        black_box(reference.bit_count());
    }
    report(
        "large_cbt OCBT-1M full reduce",
        started,
        FRAMES * OPS_PER_FRAME,
    );

    println!(
        "footprints: thessa={} bytes, large_cbt={} bytes",
        mirror.memory_footprint(),
        reference.memory_footprint()
    );
}
