//! Diagnose close-up flatness of GPU material pages (doc 41 follow-up).
//!
//! Run with:
//! `cargo run --release -p thessa-worldgen-rocky --example dump_material_pages -- OUT_DIR`
//!
//! Finds a land direction, builds the real 128x128 `GpuMaterialPage` for the
//! tiles containing it at levels 10..=16, and prints per-channel statistics
//! plus texel size. PNGs go next to the stats for visual inspection.

use std::path::PathBuf;

use thessa_worldgen_rocky::{
    appearance::{surface_appearance, surface_grain},
    field::{PlanetField, field_from_manifest},
    lod::{TileKey, build_gpu_material_page},
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::dir_from_latlon,
};

fn tile_containing(dir: [f64; 3], level: u8) -> TileKey {
    let ax = dir[0].abs();
    let ay = dir[1].abs();
    let az = dir[2].abs();
    let (face, u, v) = if ax >= ay && ax >= az {
        if dir[0] > 0.0 {
            (0u8, -dir[2] / dir[0], dir[1] / dir[0])
        } else {
            (1u8, dir[2] / -dir[0], dir[1] / -dir[0])
        }
    } else if ay >= ax && ay >= az {
        if dir[1] > 0.0 {
            (2u8, dir[0] / dir[1], -dir[2] / dir[1])
        } else {
            (3u8, dir[0] / -dir[1], dir[2] / -dir[1])
        }
    } else if dir[2] > 0.0 {
        (4u8, dir[0] / dir[2], dir[1] / dir[2])
    } else {
        (5u8, dir[0] / dir[2], dir[1] / dir[2])
    };
    let n = 1u64 << level;
    let q = |t: f64| (((t + 1.0) * 0.5 * n as f64) as u32).min(n as u32 - 1);
    TileKey {
        face,
        level,
        x: q(u),
        y: q(v),
    }
}

fn find_land(field: &PlanetField, lo: f64, hi: f64) -> [f64; 3] {
    for lat in (-55..55).step_by(3) {
        for lon in (-180..180).step_by(3) {
            let dir = dir_from_latlon(lat as f64, lon as f64);
            let h = field.height_m(dir, 500.0);
            if (lo..hi).contains(&h) {
                return dir;
            }
        }
    }
    panic!("no land found in {lo}..{hi}");
}

fn stats(data: &[u8], channel: usize) -> (f64, f64, u8, u8) {
    let n = data.len() / 4;
    let mut sum = 0u64;
    let mut min = u8::MAX;
    let mut max = u8::MIN;
    for i in 0..n {
        let v = data[i * 4 + channel];
        sum += v as u64;
        min = min.min(v);
        max = max.max(v);
    }
    let mean = sum as f64 / n as f64;
    let var = (0..n)
        .map(|i| (data[i * 4 + channel] as f64 - mean).powi(2))
        .sum::<f64>()
        / n as f64;
    (mean, var.sqrt(), min, max)
}

fn write_png(path: &std::path::Path, rgba: &[u8]) {
    use png::{BitDepth, ColorType, Encoder};
    let file = std::fs::File::create(path).unwrap();
    let mut enc = Encoder::new(file, 128, 128);
    enc.set_color(ColorType::Rgba);
    enc.set_depth(BitDepth::Eight);
    enc.write_header().unwrap().write_image_data(rgba).unwrap();
}

fn main() {
    let out = PathBuf::from(std::env::args().nth(1).expect("output directory"));
    std::fs::create_dir_all(&out).unwrap();
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    let radius = field.params.radius_m;
    // Vegetation survey: where (if anywhere) is there green on Thessa?
    {
        let mut best = (0.0f32, [1.0, 0.0, 0.0]);
        for lat in (-55..55).step_by(2) {
            for lon in (-180..180).step_by(2) {
                let dir = dir_from_latlon(lat as f64, lon as f64);
                let s = field.sample_surface(dir, 500.0);
                if s.height_m <= 0.0 {
                    continue;
                }
                let a = surface_appearance(&field, &s, dir);
                if a.vegetation > best.0 {
                    best = (a.vegetation, dir);
                }
            }
        }
        let s = field.sample_surface(best.1, 500.0);
        println!(
            "max vegetation {:.2} at h={:.0} T={:.1} moist={:.2}",
            best.0, s.height_m, s.temperature_k, s.moisture01
        );
        // Frost frontier: partial vegetation + sub-zero, where frost on
        // grass should read.
        let mut found = false;
        for lat in (-55..55).step_by(2) {
            for lon in (-180..180).step_by(2) {
                let dir = dir_from_latlon(lat as f64, lon as f64);
                let s = field.sample_surface(dir, 500.0);
                if s.height_m <= 0.0 {
                    continue;
                }
                let a = surface_appearance(&field, &s, dir);
                if (0.15..0.70).contains(&a.vegetation) && s.temperature_k < 272.0 {
                    println!(
                        "frost frontier veg={:.2} snow={:.2} at h={:.0} T={:.1} moist={:.2}",
                        a.vegetation, a.snow, s.height_m, s.temperature_k, s.moisture01
                    );
                    found = true;
                    break;
                }
            }
            if found {
                break;
            }
        }
    }
    for (name, lo, hi) in [("high", 1500.0, 3000.0), ("low", 100.0, 800.0)] {
        let dir = find_land(&field, lo, hi);
        let probe = field.sample_surface(dir, 500.0);
        println!(
            "== {name} h={:.0} T={:.1} moist={:.2}",
            probe.height_m, probe.temperature_k, probe.moisture01
        );
        println!(
            "{:>5} {:>10} {:>8} {:>22} {:>22} {:>22} {:>22}",
            "level",
            "span_m",
            "texel_m",
            "R(mean/std/min/max)",
            "G(mean/std/min/max)",
            "B(mean/std/min/max)",
            "A(mean/std/min/max)"
        );
        for level in 10u8..=16u8 {
            let key = tile_containing(dir, level);
            let page = build_gpu_material_page(&field, key);
            let span = key.span_m(radius);
            let texel = span / 128.0;
            // Raw grain amplitude on the same texel grid, to separate "field
            // has no variation" from "appearance discards it".
            let mut gmin = f64::INFINITY;
            let mut gmax = f64::NEG_INFINITY;
            let mut gsum = 0.0;
            for y in 0..128 {
                for x in 0..128 {
                    let g = surface_grain(
                        &field,
                        key.direction((x as f64 - 1.0) / 125.0, (y as f64 - 1.0) / 125.0),
                    );
                    gmin = gmin.min(g);
                    gmax = gmax.max(g);
                    gsum += g;
                }
            }
            println!(
                "  grain range [{gmin:+.3}, {gmax:+.3}] mean {:+.3}",
                gsum / 16384.0
            );
            let fmt = |c: usize| {
                let (m, s, lo, hi) = stats(&page.rgba, c);
                format!("{m:6.1}/{s:5.1}/{lo:3}/{hi:3}")
            };
            println!(
                "{level:>5} {span:>10.0} {texel:>8.1} {:>22} {:>22} {:>22} {:>22}",
                fmt(0),
                fmt(1),
                fmt(2),
                fmt(3)
            );
            if level == 12 || level == 14 || level == 16 {
                write_png(
                    &out.join(format!("material_{name}_L{level}.png")),
                    &page.rgba,
                );
            }
        }
    }
}
