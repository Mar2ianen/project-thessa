//! Tile-build cost breakdown: mesh vs texture vs single-sample rates.
//! Run with `cargo bench -p thessa-worldgen-rocky --bench tiles`.
use std::{hint::black_box, time::Instant};
use thessa_worldgen_rocky::{
    field::field_from_manifest,
    lod::{TileKey, build_surface_texture, build_tile},
    spec_recipe::{SpecRecipe, manifest_from_spec},
};

fn main() {
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    // Single sample_surface rate (texture inner loop unit).
    let dir = TileKey {
        face: 1,
        level: 14,
        x: 100,
        y: 200,
    }
    .direction(0.5, 0.5);
    let started = Instant::now();
    let n = 2000;
    for _ in 0..n {
        black_box(field.sample_surface(dir, 2.0));
    }
    println!(
        "sample_surface: {:.2} us/call",
        started.elapsed().as_secs_f64() * 1e6 / n as f64
    );
    // Section breakdown of one texel.
    let started = Instant::now();
    for _ in 0..n {
        black_box(field.height_m(dir, 2.0));
    }
    println!(
        "height_m: {:.2} us/call",
        started.elapsed().as_secs_f64() * 1e6 / n as f64
    );
    let sample = field.sample_surface(dir, 2.0);
    let started = Instant::now();
    for _ in 0..n {
        black_box(thessa_worldgen_rocky::appearance::surface_appearance(
            &field, &sample, dir,
        ));
    }
    println!(
        "surface_appearance: {:.2} us/call",
        started.elapsed().as_secs_f64() * 1e6 / n as f64
    );
    use thessa_worldgen_rocky::lod::texture_cells_for_level;
    for (level, cells) in [(10u8, 24usize), (12, 24), (14, 32), (16, 32)] {
        let key = TileKey {
            face: 1,
            level,
            x: 100,
            y: 200,
        };
        let started = Instant::now();
        let tile = build_tile(&field, key, cells);
        let mesh_ms = started.elapsed().as_secs_f64() * 1000.0;
        let tsize = texture_cells_for_level(level);
        let started = Instant::now();
        let tex = build_surface_texture(&field, key, tsize);
        let tex_ms = started.elapsed().as_secs_f64() * 1000.0;
        println!(
            "L{level} cells={cells} tex={tsize}: mesh {mesh_ms:.1} ms ({:.1}k verts), texture {tex_ms:.1} ms",
            tile.positions.len() as f64 / 1000.0,
        );
        black_box(tex);
    }
}
