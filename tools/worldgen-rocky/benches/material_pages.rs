//! Serial 512-page material-generation workload, not renderer/frame timing.
use std::{hint::black_box, time::Instant};
use thessa_worldgen_rocky::{field, lod, offline::FrozenSurface, spec_recipe};

fn main() {
    let recipe = toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
        .expect("recipe");
    let source = field::field_from_manifest(&spec_recipe::manifest_from_spec(&recipe).unwrap())
        .expect("source");
    let frozen = FrozenSurface::read_gzip(
        &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
    )
    .expect("frozen surface");
    let field = source.with_frozen_erosion(&frozen).expect("matched source");
    let mut timings = Vec::new();
    let mut payload = 0usize;
    let started = Instant::now();
    for i in 0..512u32 {
        let level = 10 + (i % 5) as u8;
        let n = 1u32 << level;
        let key = lod::TileKey {
            face: (i % 6) as u8,
            level,
            x: (n / 2 + i * 17) % n,
            y: (n / 3 + i * 31) % n,
        };
        let page_started = Instant::now();
        let page = lod::build_gpu_material_page(black_box(&field), key);
        payload += page.rgba.len();
        black_box(page);
        timings.push(page_started.elapsed().as_secs_f64() * 1000.0);
    }
    let total_s = started.elapsed().as_secs_f64();
    timings.sort_by(f64::total_cmp);
    println!(
        "{}",
        serde_json::json!({
            "workload": "512 serial canonical material pages, 128x128, six faces, levels 10..14",
            "profile": "release/bench", "frozen_erosion": true,
            "geometry_scale_m": 256.0, "count": timings.len(), "payload_bytes": payload,
            "total_s": total_s, "page_p50_ms": timings[256], "page_p95_ms": timings[486],
            "page_max_ms": timings[511], "not_frame_time_or_loading_benchmark": true,
        })
    );
}
