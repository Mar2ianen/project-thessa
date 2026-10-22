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
    if std::env::args().any(|arg| arg == "--ground-detail") {
        let mut paired = [Vec::new(), Vec::new()];
        for i in 0..32u32 {
            let level = [17, 20, 21, 22][i as usize % 4];
            let key = lod::TileKey {
                face: 3,
                level,
                x: (14936 << (level - 16)) + i / 4,
                y: 47207 << (level - 16),
            };
            for enabled in if (i / 4) % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let start = Instant::now();
                black_box(lod::build_gpu_material_page_with_ground_detail(
                    &field, key, true, enabled,
                ));
                paired[usize::from(enabled)].push(start.elapsed().as_secs_f64() * 1000.0);
            }
        }
        for (enabled, timings) in paired.iter_mut().enumerate() {
            timings.sort_by(f64::total_cmp);
            println!(
                "{}",
                serde_json::json!({
                    "workload": "32 paired frozen-erosion land pages, face 3, levels 17/20/21/22; counterbalanced",
                    "ground_detail": enabled == 1, "normal_maps": true, "count": timings.len(),
                    "page_p50_ms": timings[16], "page_p95_ms": timings[30], "page_max_ms": timings[31],
                    "not_frame_time_or_loading_benchmark": true,
                })
            );
        }
        return;
    }
    let mut timings = Vec::new();
    let mut payload = 0usize;
    let mut normal_payload = 0usize;
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
        normal_payload += page.normal.len();
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
            "geometry_scale_m": 256.0, "count": timings.len(),
            "albedo_payload_bytes": payload, "normal_payload_bytes": normal_payload,
            "payload_bytes": payload + normal_payload,
            "total_s": total_s, "page_p50_ms": timings[256], "page_p95_ms": timings[486],
            "page_max_ms": timings[511], "not_frame_time_or_loading_benchmark": true,
        })
    );
    // A paired close-land workload actually resolves the sub-8 m bands.
    // Counterbalance order; the broad baseline above often filters them out.
    let mut paired = [Vec::new(), Vec::new()];
    for i in 0..64u32 {
        let level = 14 + (i % 4) as u8;
        let n = 1u32 << level;
        let key = lod::TileKey {
            face: 3,
            level,
            x: (14936u64 * u64::from(n) / 65536) as u32 + i / 4,
            y: (47207u64 * u64::from(n) / 65536) as u32,
        };
        for enabled in if i % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            let start = Instant::now();
            black_box(lod::build_gpu_material_page_with_normals(
                &field, key, enabled,
            ));
            paired[usize::from(enabled)].push(start.elapsed().as_secs_f64() * 1000.0);
        }
    }
    for (enabled, timings) in paired.iter_mut().enumerate() {
        timings.sort_by(f64::total_cmp);
        println!(
            "{}",
            serde_json::json!({
                "workload": "64 paired close-land pages, face 3, levels 14..17, counterbalanced order",
                "normal_maps": enabled == 1, "count": timings.len(),
                "page_p50_ms": timings[32], "page_p95_ms": timings[60],
                "page_max_ms": timings[63], "total_s": timings.iter().sum::<f64>() / 1000.0,
                "not_frame_time_or_loading_benchmark": true,
            })
        );
    }
}
