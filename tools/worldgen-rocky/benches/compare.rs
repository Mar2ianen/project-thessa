//! Legacy CPU tiles vs RCBT pages on identical selections.
//! Run with `cargo bench -p thessa-worldgen-rocky --bench compare`.
//!
//! Selection is shared, so the table isolates build cost:
//! legacy = mesh + per-tile textures, rcbt = topology plan + compact pages.
use std::{hint::black_box, time::Instant};

use thessa_worldgen_rocky::{
    backend::{BackendKind, TerrainRequest, compare, create_backend},
    field::field_from_manifest,
    spec_recipe::{SpecRecipe, manifest_from_spec},
};

fn main() {
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    let radius = field.params.radius_m;

    let scenarios = [
        (
            "survey-5km",
            TerrainRequest {
                eye_m: [radius + 5_000.0, 0.0, 0.0],
                radius_m: radius,
                max_level: 17,
                budget: 192,
                build_limit: 10,
                mesh_cells: 24,
                page_grid: 9,
                page_error_m: 100.0,
            },
        ),
        (
            "cruise-50km",
            TerrainRequest {
                eye_m: [radius + 50_000.0, 0.0, 0.0],
                radius_m: radius,
                max_level: 14,
                budget: 192,
                build_limit: 10,
                mesh_cells: 24,
                page_grid: 9,
                page_error_m: 100.0,
            },
        ),
        (
            "pathological-2km",
            TerrainRequest {
                eye_m: [radius + 2_000.0, 0.0, 0.0],
                radius_m: radius,
                max_level: 17,
                budget: 384,
                build_limit: 10,
                mesh_cells: 32,
                page_grid: 17,
                page_error_m: 100.0,
            },
        ),
    ];

    println!(
        "| scenario | backend | selected | built | select_ms | topo_ms | build_ms | total_ms | payload_bytes | max_err_m |"
    );
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for (name, req) in scenarios {
        // Warmup + timing of the full comparison for stability.
        let started = Instant::now();
        let cmp = compare(&field, &req);
        let wall_ms = started.elapsed().as_secs_f64() * 1000.0;
        for out in [&cmp.legacy, &cmp.rcbt] {
            println!(
                "| {name} | {} | {} | {} | {:.2} | {:.2} | {:.1} | {:.1} | {} | {:.2} |",
                out.backend,
                out.selected,
                out.built,
                out.select_ms,
                out.topology_ms,
                out.build_ms,
                out.total_ms(),
                out.payload_bytes,
                out.max_error_m,
            );
            black_box(out.payload_bytes);
        }
        println!(
            "| {name} | speedup x{:.2} (build+topo) | payload ratio {:.3} | wall {wall_ms:.0} ms |",
            cmp.build_speedup(),
            cmp.payload_ratio(),
        );
    }

    // BackendKind factory smoke check (legacy default, rcbt opt-in).
    assert_eq!(create_backend(BackendKind::Legacy).name(), "legacy-cpu");
    assert_eq!(create_backend(BackendKind::Rcbt).name(), "rcbt-pages");
}
