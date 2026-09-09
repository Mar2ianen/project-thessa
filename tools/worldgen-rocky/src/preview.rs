//! Low-resolution global overview export (orbit-readability inspection).
//!
//! Writes plain PGM (height) + PPM (biome) via std only; no image crates.
//! Grids are coarse on purpose: if a planet is unreadable here, it will be
//! unreadable from orbit.

use crate::{
    bake::{classify_site_coarse, evaluate_height_grid},
    biomes::biome_color,
    height::encode_height_01,
    manifest::Manifest,
};

/// Render overview files. Returns (height_path, biome_path).
pub fn render_preview(
    manifest: &Manifest,
    step_deg: f64,
    out_prefix: &str,
) -> Result<(String, String), String> {
    let grid = evaluate_height_grid(manifest, step_deg)?;
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
}
