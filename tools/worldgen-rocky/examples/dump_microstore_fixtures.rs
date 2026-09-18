//! Dump real Thessa field data as vendored fixtures for the microstore
//! codec experiment (doc 41 fixture 8: "real current Thessa captures").
//!
//! Run with:
//! `cargo run --release -p thessa-worldgen-rocky --example dump_microstore_fixtures -- OUT_DIR`
//!
//! The example surveys the Thessa height field for an ocean, a coast, and
//! a mountain window, then writes raw little-endian grids: f32 heights,
//! u8 sRGB albedo triples, and u8 roughness. A README with provenance
//! (recipe, seed, window coordinates, wavelengths) goes next to them.
//! The microstore crate vendors these bytes with `include_bytes!`, so its
//! tests stay hermetic and dependency-free.

use std::path::{Path, PathBuf};

use thessa_worldgen_rocky::{
    appearance::surface_appearance,
    field::{PlanetField, field_from_manifest},
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::dir_from_latlon,
};

const HEIGHT_GRID: u32 = 65;
const MATERIAL_GRID: u32 = 128;
const WINDOW_DEG: f64 = 2.0;
const HEIGHT_WAVELENGTH_M: f64 = 2000.0;
const MATERIAL_WAVELENGTH_M: f64 = 1000.0;

fn sample_window(
    field: &PlanetField,
    lat0: f64,
    lon0: f64,
    span_deg: f64,
    n: u32,
    wavelength_m: f64,
) -> Vec<f64> {
    let mut out = Vec::with_capacity((n * n) as usize);
    for iy in 0..n {
        let lat = lat0 + (iy as f64 / (n - 1) as f64 - 0.5) * span_deg;
        for ix in 0..n {
            let lon = lon0 + (ix as f64 / (n - 1) as f64 - 0.5) * span_deg;
            out.push(field.height_m(dir_from_latlon(lat, lon), wavelength_m));
        }
    }
    out
}

fn window_stats(field: &PlanetField, lat0: f64, lon0: f64) -> (f64, f64) {
    let hs = sample_window(field, lat0, lon0, 3.0, 5, 16_000.0);
    let min = hs.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = hs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    (min, max)
}

fn write_f32_grid(path: &Path, w: u32, h: u32, data: &[f32]) {
    assert_eq!(data.len(), w as usize * h as usize);
    let mut out = Vec::with_capacity(8 + data.len() * 4);
    out.extend_from_slice(&w.to_le_bytes());
    out.extend_from_slice(&h.to_le_bytes());
    for v in data {
        out.extend_from_slice(&v.to_le_bytes());
    }
    std::fs::write(path, out).unwrap();
}

fn write_u8_grid(path: &Path, w: u32, h: u32, channels: usize, data: &[u8]) {
    assert_eq!(data.len(), w as usize * h as usize * channels);
    let mut out = Vec::with_capacity(8 + data.len());
    out.extend_from_slice(&w.to_le_bytes());
    out.extend_from_slice(&h.to_le_bytes());
    out.extend_from_slice(data);
    std::fs::write(path, out).unwrap();
}

fn main() {
    let out = PathBuf::from(std::env::args().nth(1).expect("output directory"));
    std::fs::create_dir_all(&out).unwrap();
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    println!("seed {}; datum_radius_m {:.0}", field.params.seed, field.params.radius_m);

    // Coarse survey for one window of each class.
    let mut ocean = None;
    let mut coast = None;
    let mut mountain = None;
    'survey: for lat in (-60..=60).step_by(6) {
        for lon in (-180..170).step_by(6) {
            let (min, max) = window_stats(&field, lat as f64, lon as f64);
            if ocean.is_none() && max < -500.0 {
                ocean = Some((lat, lon, min, max));
            }
            if coast.is_none() && min < -200.0 && max > 800.0 {
                coast = Some((lat, lon, min, max));
            }
            if mountain.is_none() && max - min > 6000.0 && max > 3000.0 {
                mountain = Some((lat, lon, min, max));
            }
            if ocean.is_some() && coast.is_some() && mountain.is_some() {
                break 'survey;
            }
        }
    }
    let (olat, olon, omin, omax) = ocean.expect("ocean window");
    let (clat, clon, cmin, cmax) = coast.expect("coast window");
    let (mlat, mlon, mmin, mmax) = mountain.expect("mountain window");
    println!("ocean    at ({olat}, {olon}) coarse min {omin:.0} max {omax:.0}");
    println!("coast    at ({clat}, {clon}) coarse min {cmin:.0} max {cmax:.0}");
    println!("mountain at ({mlat}, {mlon}) coarse min {mmin:.0} max {mmax:.0}");

    // Height grids over the three windows.
    for (name, lat, lon) in [
        ("ocean", olat as f64, olon as f64),
        ("coast", clat as f64, clon as f64),
        ("mountain", mlat as f64, mlon as f64),
    ] {
        let hs = sample_window(&field, lat, lon, WINDOW_DEG, HEIGHT_GRID, HEIGHT_WAVELENGTH_M);
        let min = hs.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = hs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        println!("{name}: {HEIGHT_GRID}x{HEIGHT_GRID} min {min:.1} max {max:.1}");
        write_f32_grid(
            &out.join(format!("thessa_height_{name}_{HEIGHT_GRID}x{HEIGHT_GRID}.f32")),
            HEIGHT_GRID,
            HEIGHT_GRID,
            &hs.iter().map(|v| *v as f32).collect::<Vec<_>>(),
        );
    }

    // Material planes over the coast window (richest statistics).
    let n = MATERIAL_GRID;
    let mut albedo = Vec::with_capacity((n * n * 3) as usize);
    let mut rough = Vec::with_capacity((n * n) as usize);
    for iy in 0..n {
        let lat = clat as f64 + (iy as f64 / (n - 1) as f64 - 0.5) * WINDOW_DEG;
        for ix in 0..n {
            let lon = clon as f64 + (ix as f64 / (n - 1) as f64 - 0.5) * WINDOW_DEG;
            let dir = dir_from_latlon(lat, lon);
            let sample = field.sample_surface(dir, MATERIAL_WAVELENGTH_M);
            let app = surface_appearance(&field, &sample, dir);
            for c in app.albedo_srgb {
                albedo.push((c.clamp(0.0, 1.0) * 255.0).round() as u8);
            }
            rough.push((app.roughness.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
    }
    write_u8_grid(
        &out.join(format!("thessa_albedo_coast_{n}x{n}.rgb")),
        n,
        n,
        3,
        &albedo,
    );
    write_u8_grid(
        &out.join(format!("thessa_rough_coast_{n}x{n}.r8")),
        n,
        n,
        1,
        &rough,
    );

    std::fs::write(
        out.join("README.md"),
        format!(
            "# Vendored Thessa fixtures for the microstore codec experiment\n\
             \n\
             Generated by `thessa-worldgen-rocky --example dump_microstore_fixtures`.\n\
             Recipe: data/worldgen/worldgen_recipe.toml, seed {}, datum {:.0} m.\n\
             Windows are 2.0x2.0 deg lat/lon patches:\n\
             - ocean ({olat}, {olon}), height wavelength {HEIGHT_WAVELENGTH_M:.0} m\n\
             - coast ({clat}, {clon}), height wavelength {HEIGHT_WAVELENGTH_M:.0} m\n\
             - mountain ({mlat}, {mlon}), height wavelength {HEIGHT_WAVELENGTH_M:.0} m\n\
             Material planes cover the coast window at {MATERIAL_WAVELENGTH_M:.0} m\n\
             wavelength via `sample_surface` + `surface_appearance` (albedo\n\
             sRGB bytes, roughness bytes). Layout: u32 LE width, u32 LE\n\
             height, then f32 LE heights / interleaved RGB / single bytes.\n",
            field.params.seed, field.params.radius_m,
        ),
    )
    .unwrap();
    println!("wrote fixtures to {}", out.display());
}
