use std::{hint::black_box, time::Instant};
fn main() {
    let bytes = include_bytes!("../../../assets/models/north_american_x-15_plane.glb");
    let mut times = Vec::new();
    for _ in 0..10 {
        let start = Instant::now();
        let source =
            thessa_render_assets::decode_glb(black_box(bytes), Default::default()).unwrap();
        times.push(start.elapsed().as_secs_f64() * 1000.0);
        black_box(source);
    }
    times.sort_by(f64::total_cmp);
    println!(
        "X15 full static source decode: n=10 median_ms={:.3} p90_ms={:.3}; excludes file IO, upload and source drop",
        times[5], times[9]
    );
}
