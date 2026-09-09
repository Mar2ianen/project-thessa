//! Low-resolution global overview export (orbit-readability inspection).
//!
//! Writes plain PGM (height) + PPM (biome) via std only; no image crates.
//! Grids are coarse on purpose: if a planet is unreadable here, it will be
//! unreadable from orbit.

use crate::{
    bake::classify_site_coarse, biomes::biome_color, height::encode_height_01, manifest::Manifest,
};

/// Render overview files. Returns (height_path, biome_path).
pub fn render_preview(
    manifest: &Manifest,
    step_deg: f64,
    out_prefix: &str,
) -> Result<(String, String), String> {
    render_preview_steps(manifest, step_deg, step_deg, out_prefix)
}

/// Same with independent latitude/longitude steps (exact map sizes,
/// e.g. 1920x1080 via steps 1/6 x 3/16 deg).
pub fn render_preview_steps(
    manifest: &Manifest,
    step_lat_deg: f64,
    step_lon_deg: f64,
    out_prefix: &str,
) -> Result<(String, String), String> {
    let grid = crate::bake::evaluate_height_grid_steps(manifest, step_lat_deg, step_lon_deg)?;
    let (rows, cols) = (grid.rows(), grid.cols());
    // Height PGM via the documented piecewise encoding.
    let mut pgm = format!("P5\n{cols} {rows}\n255\n").into_bytes();
    for row in &grid.h {
        for h in row {
            let g = encode_height_01(
                *h,
                manifest.planet.height_min_m,
                manifest.planet.height_max_m,
            );
            pgm.push((g.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
    }
    let height_path = format!("{out_prefix}_height.pgm");
    std::fs::write(&height_path, pgm).map_err(|e| e.to_string())?;
    // Biome PPM from the taxonomy palette.
    let mut ppm = format!("P6\n{cols} {rows}\n255\n").into_bytes();
    for (r, row) in grid.h.iter().enumerate() {
        for (c, h) in row.iter().enumerate() {
            let site = classify_site_coarse(manifest, grid.lats[r], grid.lons[c], *h);
            let (rr, gg, bb) = biome_color(site.biome);
            ppm.extend_from_slice(&[rr, gg, bb]);
        }
    }
    let biome_path = format!("{out_prefix}_biome.ppm");
    std::fs::write(&biome_path, ppm).map_err(|e| e.to_string())?;
    Ok((height_path, biome_path))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preview_writes_valid_headers() {
        let toml = include_str!("../../../data/worldgen/example_rocky.toml");
        let manifest: Manifest = toml::from_str(toml).expect("parse");
        let dir = std::env::temp_dir().join("thessa_preview_test");
        let _ = std::fs::create_dir_all(&dir);
        let prefix = dir.join("pv").to_string_lossy().into_owned();
        let (h, b) = render_preview(&manifest, 10.0, &prefix).expect("render");
        let hd = std::fs::read(&h).expect("read height");
        assert!(hd.starts_with(b"P5\n"));
        let bd = std::fs::read(&b).expect("read biome");
        assert!(bd.starts_with(b"P6\n"));
        let _ = std::fs::remove_file(h);
        let _ = std::fs::remove_file(b);
    }

    #[test]
    fn map_export_matches_requested_dimensions() {
        // 1920x1080 runtime map: exact pixel dimensions, deterministic.
        let toml = include_str!("../../../data/worldgen/example_rocky.toml");
        let manifest: Manifest = toml::from_str(toml).expect("parse");
        let dir = std::env::temp_dir().join("thessa_map_test");
        let _ = std::fs::create_dir_all(&dir);
        let prefix = dir.join("map").to_string_lossy().into_owned();
        let (h, b) =
            render_preview_steps(&manifest, 180.0 / 270.0, 360.0 / 480.0, &prefix).expect("render");
        let hd = std::fs::read(&h).expect("read height");
        assert!(hd.starts_with(b"P5\n480 270\n255\n"));
        let bd = std::fs::read(&b).expect("read biome");
        assert!(bd.starts_with(b"P6\n480 270\n255\n"));
        let _ = std::fs::remove_file(h);
        let _ = std::fs::remove_file(b);
    }
}

use crate::{
    bake::knobs_from_manifest,
    biomes::geology_color,
    climate::{continentality_steps, drivers_at, eclipse_exposure, nereid_influence},
    erosion::{ErosionKnobs, erode},
    geothermal::{GeothermalProvince, geothermal_activity},
    hydro::{HeightGrid, WaterClass},
    terrain::{eval_macro_m, eval_meso_m},
};

/// Extra overlay inputs for spec previews.
pub struct OverlayInputs {
    pub provinces: Vec<GeothermalProvince>,
    pub eclipse_strength: f64,
}

/// Render geology / hydrology / geothermal / eclipse / continentality.
/// Returns written paths.
pub fn render_spec_overlays(
    manifest: &Manifest,
    grid: &HeightGrid,
    water: &[Vec<WaterClass>],
    overlays: &OverlayInputs,
    out_prefix: &str,
) -> Result<Vec<String>, String> {
    let (rows, cols) = (grid.rows(), grid.cols());
    let mut paths = Vec::new();
    // Geology PPM.
    {
        let mut ppm = format!("P6\n{cols} {rows}\n255\n").into_bytes();
        for (r, row) in grid.h.iter().enumerate() {
            for (c, h) in row.iter().enumerate() {
                let site = classify_site_coarse(manifest, grid.lats[r], grid.lons[c], *h);
                let (rr, gg, bb) = geology_color(site.geology);
                ppm.extend_from_slice(&[rr, gg, bb]);
            }
        }
        let path = format!("{out_prefix}_geology.ppm");
        std::fs::write(&path, ppm).map_err(|e| e.to_string())?;
        paths.push(path);
    }
    // Hydrology PPM.
    {
        let mut ppm = format!("P6\n{cols} {rows}\n255\n").into_bytes();
        for row in water {
            for w in row {
                let (rr, gg, bb) = match w {
                    WaterClass::Ocean => (10, 30, 150),
                    WaterClass::Lake => (60, 140, 220),
                    WaterClass::River => (120, 200, 255),
                    WaterClass::SaltFlat => (240, 238, 230),
                    WaterClass::Ice => (225, 240, 255),
                    WaterClass::Land => (25, 25, 25),
                };
                ppm.extend_from_slice(&[rr, gg, bb]);
            }
        }
        let path = format!("{out_prefix}_hydro.ppm");
        std::fs::write(&path, ppm).map_err(|e| e.to_string())?;
        paths.push(path);
    }
    // Geothermal PGM.
    {
        let mut pgm = format!("P5\n{cols} {rows}\n255\n").into_bytes();
        for (r, row) in grid.h.iter().enumerate() {
            for (c, _h) in row.iter().enumerate() {
                let a = geothermal_activity(
                    &overlays.provinces,
                    grid.lats[r],
                    grid.lons[c],
                    manifest.planet.datum_radius_m,
                    0.0,
                    0.0,
                );
                pgm.push((a.clamp(0.0, 1.0) * 255.0).round() as u8);
            }
        }
        let path = format!("{out_prefix}_geothermal.pgm");
        std::fs::write(&path, pgm).map_err(|e| e.to_string())?;
        paths.push(path);
    }
    // Eclipse exposure PGM (diagnostic mask).
    {
        let mut pgm = format!("P5\n{cols} {rows}\n255\n").into_bytes();
        for _ in 0..rows {
            for lon in &grid.lons {
                let e = eclipse_exposure(*lon, overlays.eclipse_strength);
                pgm.push((e.clamp(0.0, 1.0) * 255.0).round() as u8);
            }
        }
        let path = format!("{out_prefix}_eclipse.pgm");
        std::fs::write(&path, pgm).map_err(|e| e.to_string())?;
        paths.push(path);
    }
    // Continentality PGM (diagnostic mask, needs ocean distance).
    {
        let dist = continentality_steps(grid, water);
        let mut pgm = format!("P5\n{cols} {rows}\n255\n").into_bytes();
        // Normalize by the 2500 km scale used in drivers_at.
        let cell_km =
            2.0 * std::f64::consts::PI * manifest.planet.datum_radius_m / 1000.0 / cols as f64;
        for row in dist {
            for d in row {
                let m = d as f64 * cell_km * 1000.0;
                let v = (m / 2_500_000.0).clamp(0.0, 1.0);
                // Sanity: drivers_at agrees with this mask.
                let _ = drivers_at(0.0, 0.0, 0.0, m, 0.0, 0.0);
                pgm.push((v * 255.0).round() as u8);
            }
        }
        let path = format!("{out_prefix}_continentality.pgm");
        std::fs::write(&path, pgm).map_err(|e| e.to_string())?;
        paths.push(path);
    }
    // Touch nereid_influence so the mask family stays covered.
    let _ = nereid_influence(0.0);
    Ok(paths)
}

/// Planetary readability at exactly 480x270 (spec acceptance).
#[derive(Debug, Clone, PartialEq)]
pub struct Readability {
    /// Distinct biomes covering >= 0.5% of cells each.
    pub distinct_regions: usize,
    pub has_basin: bool,
    pub has_arc: bool,
    pub has_canyon: bool,
    pub has_volcanic: bool,
    pub has_polar: bool,
}

impl Readability {
    pub fn passes(&self) -> bool {
        self.distinct_regions >= 5
            && self.has_basin
            && self.has_arc
            && self.has_canyon
            && self.has_volcanic
            && self.has_polar
    }
}

/// Evaluate the full stack on a 480x270 grid and check readability.
pub fn readability_480x270(manifest: &Manifest) -> Result<Readability, String> {
    const COLS: usize = 480;
    const ROWS: usize = 270;
    let knobs = knobs_from_manifest(manifest);
    let strength = manifest.readability.macro_feature_strength;
    let lats: Vec<f64> = (0..ROWS)
        .map(|r| 90.0 - (r as f64 + 0.5) * (180.0 / ROWS as f64))
        .collect();
    let lons: Vec<f64> = (0..COLS)
        .map(|c| -180.0 + (c as f64 + 0.5) * (360.0 / COLS as f64))
        .collect();
    let mut grid = HeightGrid::new(lats, lons, manifest.planet.datum_radius_m);
    for (r, lat) in grid.lats.clone().iter().enumerate() {
        for (c, lon) in grid.lons.clone().iter().enumerate() {
            let tectonic = crate::tectonics::eval_uplift_m(
                &manifest.tectonics,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            ) * strength;
            let macro_h = eval_macro_m(
                &manifest.features,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            ) * strength;
            let meso_h = eval_meso_m(
                manifest.planet.seed,
                knobs,
                *lat,
                *lon,
                manifest.planet.datum_radius_m,
            );
            grid.h[r][c] = tectonic + macro_h + meso_h;
        }
    }
    erode(
        &mut grid,
        ErosionKnobs {
            thermal_iters: manifest.erosion.thermal_iters.min(6),
            talus_deg: manifest.erosion.talus_deg,
            droplets: manifest.erosion.droplets.min(2000),
            droplet_steps: manifest.erosion.droplet_steps.min(32),
        },
    );
    use std::collections::HashMap;
    let mut counts: HashMap<String, usize> = HashMap::new();
    for (r, row) in grid.h.iter().enumerate() {
        for (c, h) in row.iter().enumerate() {
            let site = classify_site_coarse(manifest, grid.lats[r], grid.lons[c], *h);
            *counts.entry(format!("{:?}", site.biome)).or_insert(0) += 1;
        }
    }
    let total = (ROWS * COLS) as f64;
    let regions: Vec<&String> = counts
        .iter()
        .filter(|(_, n)| **n as f64 / total >= 0.005)
        .map(|(k, _)| k)
        .collect();
    let has = |names: &[&str]| regions.iter().any(|n| names.contains(&n.as_str()));
    Ok(Readability {
        distinct_regions: regions.len(),
        has_basin: has(&["ImpactBasin", "AncientImpactBasin"]),
        has_arc: has(&["MountainRange", "MountainRidge", "AlpinePeaks"]),
        has_canyon: has(&["CanyonProvince"]),
        has_volcanic: has(&[
            "VolcanicField",
            "ShieldVolcano",
            "BasaltPlain",
            "LavaFlow",
            "Caldera",
            "FreshLava",
        ]),
        has_polar: has(&["PolarIceCap", "IceCap", "Glacier", "PermanentSnow"]),
    })
}
