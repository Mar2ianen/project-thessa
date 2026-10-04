//! Filtered-appearance survey frames (CPU): global equirect albedo plus
//! bookmark crops through the shared `surface_appearance_filtered`
//! evaluator. Flat albedo on purpose — palette and transitions are what the
//! material review judges; lighting/shadows are separate missing systems.
//! Usage: cargo run -p thessa-worldgen-rocky --example survey_frame -- <out-prefix>
use thessa_worldgen_rocky::{appearance, field, spec_recipe};

fn dir_from_latlon(lat_deg: f64, lon_deg: f64) -> [f64; 3] {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    [lat.cos() * lon.cos(), lat.sin(), lat.cos() * lon.sin()]
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let prefix = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/home/chechulin/tmp/survey".to_string());
    let recipe: spec_recipe::SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))?;
    let manifest = spec_recipe::manifest_from_spec(&recipe)?;
    let planet = field::field_from_manifest(&manifest)?;
    // Global 960x480 equirect albedo.
    let (gw, gh) = (960_usize, 480_usize);
    let mut ppm = format!("P6\n{gw} {gh}\n255\n").into_bytes();
    for r in 0..gh {
        let lat = 90.0 - (r as f64 + 0.5) * 180.0 / gh as f64;
        for c in 0..gw {
            let lon = (c as f64 + 0.5) * 360.0 / gw as f64 - 180.0;
            let dir = dir_from_latlon(lat, lon);
            let sample = planet.sample_surface(dir, 32.0);
            let mat = appearance::surface_appearance_filtered(&planet, &sample, dir, 16000.0);
            ppm.extend_from_slice(&[
                (mat.albedo_srgb[0].clamp(0.0, 1.0) * 255.0) as u8,
                (mat.albedo_srgb[1].clamp(0.0, 1.0) * 255.0) as u8,
                (mat.albedo_srgb[2].clamp(0.0, 1.0) * 255.0) as u8,
            ]);
        }
        if r % 96 == 0 {
            eprintln!("row {r}/{gh}");
        }
    }
    let global = format!("{prefix}_global.ppm");
    std::fs::write(&global, ppm)?;
    println!("wrote: {global}");
    // Bookmark crops: 240x240 windows, ~0.15 deg/px.
    for (name, lat0, lon0) in [
        ("coast", 23.0, 165.0),
        ("highlands", 38.0, -171.0),
        ("volcanic", -16.0, 159.0),
    ] {
        let (w, h) = (240_usize, 240_usize);
        let step = 0.15;
        let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
        for r in 0..h {
            for c in 0..w {
                let dir = dir_from_latlon(
                    lat0 + (r as f64 - h as f64 / 2.0) * step,
                    lon0 + (c as f64 - w as f64 / 2.0) * step,
                );
                let sample = planet.sample_surface(dir, 32.0);
                let mat = appearance::surface_appearance_filtered(&planet, &sample, dir, 16000.0);
                ppm.extend_from_slice(&[
                    (mat.albedo_srgb[0].clamp(0.0, 1.0) * 255.0) as u8,
                    (mat.albedo_srgb[1].clamp(0.0, 1.0) * 255.0) as u8,
                    (mat.albedo_srgb[2].clamp(0.0, 1.0) * 255.0) as u8,
                ]);
            }
        }
        let path = format!("{prefix}_{name}.ppm");
        std::fs::write(&path, ppm)?;
        println!("wrote: {path}");
    }
    Ok(())
}
