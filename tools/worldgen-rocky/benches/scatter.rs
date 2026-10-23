//! Bounded CPU scatter cost, not a renderer or old/new speedup comparison.
//! Cargo wiring: [[bench]] name = "scatter", harness = false.
use std::{hint::black_box, time::Instant};
use thessa_worldgen_rocky::{
    biomes::{Biome, Geology},
    ecology::VegetationCover,
    lod::TileKey,
    scatter::{ScatterPolicy, ScatterPopulation, ScatterSurface, scatter_region},
};

fn site(dir: [f64; 3]) -> ScatterSurface {
    ScatterSurface {
        ground_height_m: 100.0 + 10.0 * dir[1],
        slope_rise_run: 0.1,
        water_height_m: Some(0.0),
        biome: Biome::EjectaField,
        geology: Geology::Basaltic,
        rocky01: 1.0,
        vegetation: VegetationCover {
            ground01: 0.5,
            canopy01: 0.25,
            reeds01: 0.25,
        },
    }
}

fn main() {
    if std::env::args().any(|arg| arg == "--prototypes") {
        let prototypes = thessa_worldgen_rocky::offline::plant_prototypes();
        let counts: Vec<_> = prototypes
            .iter()
            .map(|p| {
                use thessa_worldgen_rocky::lod::{cross, dot, sub};
                let foliage_area_m2: f64 = p
                    .indices
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .filter(|tri| {
                        p.stem
                            .as_ref()
                            .is_none_or(|stem| tri[0] >= stem.vertex_count)
                    })
                    .map(|tri| {
                        let [a, b, c] = tri.map(|i| p.positions[i as usize].map(f64::from));
                        let n = cross(sub(b, a), sub(c, a));
                        dot(n, n).sqrt() * 0.5
                    })
                    .sum();
                serde_json::json!({
                    "kind": format!("{:?}", p.kind), "vertices": p.positions.len(),
                    "triangles": p.indices.len() / 3,
                    "producer_payload_bytes": p.positions.len() * 24 + p.indices.len() * 4,
                    "foliage_area_m2": foliage_area_m2,
                })
            })
            .collect();
        for _ in 0..32 {
            black_box(thessa_worldgen_rocky::offline::plant_prototypes());
        }
        let mut times = Vec::new();
        for _ in 0..1024 {
            let start = Instant::now();
            black_box(thessa_worldgen_rocky::offline::plant_prototypes());
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        times.sort_by(f64::total_cmp);
        println!(
            "{}",
            serde_json::json!({
                "workload": "1024 serial complete three-kind prototype builds after 32 warmups",
                "prototypes": counts, "p50_ms": times[512], "p95_ms": times[972],
                "max_ms": times[1023], "not_frame_time_or_placement": true,
            })
        );
        return;
    }
    println!("scenario,population,density_scale,candidates,surface_samples,items,p50_ms,p95_ms");
    for (name, tile) in [
        (
            "centre-L12",
            TileKey {
                face: 0,
                level: 12,
                x: 2048,
                y: 2048,
            },
        ),
        (
            "corner-L12",
            TileKey {
                face: 0,
                level: 12,
                x: 0,
                y: 0,
            },
        ),
        (
            "centre-L10",
            TileKey {
                face: 0,
                level: 10,
                x: 512,
                y: 512,
            },
        ),
    ] {
        for population in [ScatterPopulation::Rocks, ScatterPopulation::Plants] {
            for density_scale in [0.0, 0.25, 1.0] {
                let policy = ScatterPolicy {
                    density_scale,
                    ..Default::default()
                };
                let baseline =
                    scatter_region(7, 3_200_000.0, tile, population, policy, site).unwrap();
                let mut children: Vec<_> = tile
                    .children()
                    .into_iter()
                    .rev()
                    .flat_map(|key| {
                        scatter_region(7, 3_200_000.0, key, population, policy, site)
                            .unwrap()
                            .items
                    })
                    .collect();
                children.sort_unstable_by_key(|item| item.id);
                assert_eq!(baseline.items, children);
                let mut times = Vec::new();
                for _ in 0..15 {
                    let start = Instant::now();
                    black_box(
                        scatter_region(
                            black_box(7),
                            3_200_000.0,
                            black_box(tile),
                            population,
                            black_box(policy),
                            |dir| site(black_box(dir)),
                        )
                        .unwrap(),
                    );
                    times.push(start.elapsed().as_secs_f64() * 1000.0);
                }
                times.sort_by(f64::total_cmp);
                println!(
                    "{name},{population:?},{density_scale},{},{},{},{:.6},{:.6}",
                    baseline.candidates_visited,
                    baseline.surface_samples,
                    baseline.items.len(),
                    times[7],
                    times[14]
                );
            }
        }
    }
}
