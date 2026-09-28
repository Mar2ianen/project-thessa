//! Hangar compile throughput: representative Juno-style stack to panels.
//! Run with `cargo bench -p thessa-fuselage --bench compile`.
use std::{hint::black_box, time::Instant};

use thessa_fuselage::{
    BodyCompileOptions, compile_body, dream_chaser_style_body, juno_style_stack,
};

fn main() {
    let options = BodyCompileOptions::default();
    bench("Juno round stack", &juno_style_stack().unwrap(), &options);
    bench(
        "Dream Chaser lifting body",
        &dream_chaser_style_body().unwrap(),
        &options,
    );
}

fn bench(name: &str, body: &thessa_fuselage::ProceduralBody, options: &BodyCompileOptions) {
    // Warmup so the timing loop measures steady state, not first call.
    let compiled = compile_body(body, options).unwrap();
    eprintln!(
        "{name}: {} panels, {} zones",
        compiled.panels.len(),
        compiled.summary.zone_count
    );
    let iterations = 200;
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(compile_body(black_box(body), options).unwrap());
    }
    let elapsed = started.elapsed();
    eprintln!(
        "{name}: {:?} total for {iterations} iterations ({:?}/compile)",
        elapsed,
        elapsed / iterations
    );
}
