//! Run with cargo run --release -p thessa-worldgen-rocky --example profile_world -- OUT_DIR.
//! Measurements use the same collector/JSON/CSV schema as the client's F4 monitor.
use std::{hint::black_box, path::PathBuf, time::Instant};
use thessa_perf::{
    MemorySample, PerfCollector, WorldCounters, current_rss_bytes, default_capture_metadata,
};
use thessa_worldgen_rocky::{
    field::field_from_manifest,
    lod::{TileKey, build_surface_texture, build_tile},
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::dir_from_latlon,
};
fn main() {
    let out = PathBuf::from(std::env::args().nth(1).expect("output directory"));
    std::fs::create_dir_all(&out).unwrap();
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let start = Instant::now();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    println!(
        "field build {:.2} ms; datum {:.2} m",
        start.elapsed().as_secs_f64() * 1000.0,
        field.sea_offset_m
    );
    let dir = dir_from_latlon(29.0, 48.0); // independent ray coverage checked in LOD tests
    black_box(dir);
    let mut perf = PerfCollector::with_default_capacity();
    for frame in 0..80 {
        perf.begin_frame();
        let start = Instant::now();
        for (name, mode) in [
            ("world.height_1024", 0),
            ("world.surface_1024", 1),
            ("world.full_1024", 2),
        ] {
            let _scope = perf.scope(name);
            for i in 0..1024 {
                let dir = dir_from_latlon(
                    -65.0 + (i % 131) as f64,
                    -180.0 + ((i * 137 + frame) % 360) as f64,
                );
                match mode {
                    0 => {
                        black_box(field.height_m(dir, 32.0));
                    }
                    1 => {
                        black_box(field.sample_surface(dir, 32.0));
                    }
                    _ => {
                        black_box(field.sample(dir, 32.0));
                    }
                }
            }
        }
        let tile;
        {
            let _scope = perf.scope("world.terrain_meshing");
            tile = build_tile(
                &field,
                TileKey {
                    face: 0,
                    level: 12,
                    x: 2048 + frame as u32,
                    y: 2048,
                },
                24,
            );
        }
        {
            let _scope = perf.scope("world.terrain_materials");
            black_box(build_surface_texture(
                &field,
                TileKey {
                    face: 0,
                    level: 12,
                    x: 2048 + frame as u32,
                    y: 2048,
                },
                64,
            ));
        }
        perf.set_world_counters(WorldCounters {
            terrain_patches_generated: 1,
            terrain_vertices: tile.positions.len() as u64,
            terrain_triangles: (tile.indices.len() / 3) as u64,
            ..Default::default()
        });
        perf.set_memory_sample(MemorySample {
            rss_bytes: current_rss_bytes(),
            ..Default::default()
        });
        let elapsed = start.elapsed().as_secs_f64();
        perf.end_frame(elapsed, elapsed, 0.0);
    }
    let mut metadata = default_capture_metadata();
    metadata.scenario =
        "worldgen seed 7: 1024 sample batches + 24-cell tile; release CPU benchmark".into();
    let capture = perf.snapshot_capture(metadata);
    capture.write_json(&out.join("worldgen-perf.json")).unwrap();
    capture.write_csv(&out.join("worldgen-perf.csv")).unwrap();
    for scope in [
        "world.height_1024",
        "world.surface_1024",
        "world.full_1024",
        "world.terrain_meshing",
        "world.terrain_materials",
    ] {
        let s = perf.scope_stats(scope).unwrap();
        println!(
            "{scope}: p50 {:.3} ms; p95 {:.3} ms; p99 {:.3} ms",
            s.p50 * 1000.0,
            s.p95 * 1000.0,
            s.p99 * 1000.0
        );
    }
}
