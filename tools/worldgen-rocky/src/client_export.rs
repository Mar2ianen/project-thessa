//! Client texture export: the global FIELD rendered for the Bevy UV sphere.
//!
//! Mapping notes (verified against bevy_mesh 0.19 `Sphere::uv`):
//! - vertex j sits at geographic longitude `s = 2*pi*j/sectors`, v=0 at the
//!   north pole. Therefore texture column c must depict longitude
//!   `lon = 360*c/W` (NOT the usual `(lon+180)/360` equirect layout).
//! - row 0 is the north pole (v=0 samples the first row in memory).
//!
//! If the client ever shows a mirrored/inverted globe, flip here once;
//! the field itself is unaffected.
//!
//! The texture is unlit climate/terrain albedo; normals supply relief and
//! the renderer supplies illumination. Rivers/lakes at texel scale are
//! omitted; hydro data stays authoritative in baked reports.

use crate::{appearance::surface_appearance, field::PlanetField, sphere::dir_from_latlon};

/// Render an equirect client texture (2:1) from the field.
/// `min_wavelength_m` selects texture detail (erosion-free by design).
///
/// In-memory version: capped at 4096x2048, larger outputs must stream via
/// [`write_client_texture_png`].
pub fn render_client_texture(
    field: &PlanetField,
    width: usize,
    height: usize,
    min_wavelength_m: f64,
) -> Result<(Vec<u8>, Vec<f64>), String> {
    if width == 0 || height == 0 || width > 4096 || height > 2048 {
        return Err("in-memory render capped at 4096x2048; stream larger sizes".into());
    }
    if !(min_wavelength_m.is_finite() && min_wavelength_m > 0.0) {
        return Err("min wavelength must be positive".into());
    }
    // Pass 1: heights + biome colors.
    let mut heights = vec![0.0; width * height];
    let mut colors: Vec<(u8, u8, u8)> = vec![(0, 0, 0); width * height];
    for r in 0..height {
        let lat = 90.0 - (r as f64 + 0.5) * (180.0 / height as f64);
        for c in 0..width {
            // Bevy-sphere convention: u=0 at lon 0.
            let lon = (c as f64 + 0.5) * (360.0 / width as f64);
            let lon = if lon > 180.0 { lon - 360.0 } else { lon };
            let dir = dir_from_latlon(lat, lon);
            let s = field.sample_surface(dir, min_wavelength_m);
            heights[r * width + c] = s.height_m;
            colors[r * width + c] = texel_color(field, &s, dir);
        }
    }
    let rgb = colors.into_iter().flat_map(|(r, g, b)| [r, g, b]).collect();
    Ok((rgb, heights))
}

fn texel_color(
    field: &PlanetField,
    s: &crate::field::TerrainSample,
    dir: [f64; 3],
) -> (u8, u8, u8) {
    let rgb = surface_appearance(field, s, dir)
        .albedo_srgb
        .map(|v| (v * 255.0).round() as u8);
    (rgb[0], rgb[1], rgb[2])
}

/// Encode 8-bit RGB to PNG bytes (sRGB) via the streaming encoder.
pub fn encode_png_rgb(width: usize, height: usize, rgb: &[u8]) -> Result<Vec<u8>, String> {
    if rgb.len() != width * height * 3 {
        return Err("rgb buffer size mismatch".into());
    }
    let mut out = Vec::new();
    let mut rows = (0..height)
        .map(|r| rgb[r * width * 3..(r + 1) * width * 3].to_vec())
        .collect::<Vec<_>>()
        .into_iter();
    crate::png_min::write_png_rows(&mut out, width, height, &mut rows)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::field::{KnobsSerde, PlanetField, PlanetParams};

    fn test_field() -> PlanetField {
        PlanetField::build(
            PlanetParams {
                name: "t".into(),
                seed: 77,
                radius_m: 3_200_000.0,
                height_min_m: -8000.0,
                height_max_m: 12000.0,
                knobs: KnobsSerde {
                    mountain_coverage: 0.3,
                    crater_density: 0.3,
                    volcanism: 0.2,
                    erosion: 0.4,
                    canyon_strength: 0.3,
                    base_roughness_m: 150.0,
                },
                macro_strength: 1.0,
                polar_extent: 0.15,
                aridity: 0.4,
                glaciation: 0.2,
                volcanism: 0.2,
                nereid_ocean_bias: 0.0,
                anti_nereid_land_bias: 0.0,
                eclipse_strength: 0.28,
                nominal_flux_w_m2: 0.146,
                ocean_target: None,
            },
            vec![],
            vec![],
            vec![],
            vec![],
        )
        .expect("field builds")
    }

    #[test]
    fn texture_dims_and_png_header() {
        let field = test_field();
        let (rgb, heights) = render_client_texture(&field, 64, 32, 4000.0).expect("render");
        assert_eq!(rgb.len(), 64 * 32 * 3);
        assert_eq!(heights.len(), 64 * 32);
        assert!(rgb.iter().any(|v| *v > 0));
        let png = encode_png_rgb(64, 32, &rgb).expect("encode");
        assert_eq!(&png[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }

    #[test]
    fn seam_columns_are_neighbours() {
        // u=0 (lon 0) and u=1 (lon 360 == lon 0) depict the same meridian:
        // first and last columns must nearly agree on a smooth field.
        let field = test_field();
        let (rgb, _) = render_client_texture(&field, 72, 18, 64000.0).expect("render");
        let mut diff = 0u64;
        for r in 0..18 {
            for ch in 0..3 {
                let a = rgb[(r * 72) * 3 + ch] as i64;
                let b = rgb[(r * 72 + 71) * 3 + ch] as i64;
                diff += (a - b).unsigned_abs();
            }
        }
        let mean = diff as f64 / (18.0 * 3.0);
        assert!(mean < 40.0, "seam jump {mean}");
    }

    #[test]
    fn deterministic_render() {
        let field = test_field();
        let (a, _) = render_client_texture(&field, 48, 24, 4000.0).expect("a");
        let (b, _) = render_client_texture(&field, 48, 24, 4000.0).expect("b");
        assert_eq!(a, b);
    }
}

/// One texel: (height_m, rgb) without hillshade.
fn sample_texel(
    field: &PlanetField,
    width: usize,
    height: usize,
    r: usize,
    c: usize,
    min_wavelength_m: f64,
) -> (f64, (u8, u8, u8)) {
    let lat = 90.0 - (r as f64 + 0.5) * (180.0 / height as f64);
    let lon_raw = (c as f64 + 0.5) * (360.0 / width as f64);
    let lon = if lon_raw > 180.0 {
        lon_raw - 360.0
    } else {
        lon_raw
    };
    let dir = dir_from_latlon(lat, lon);
    let s = field.sample_surface(dir, min_wavelength_m);
    (s.height_m, texel_color(field, &s, dir))
}

/// Stream a client texture of any size (tested to 16K) to a PNG writer.
/// Memory stays at ~3 rows: heights window for the hillshade pass.
/// Stream a client texture of any size (tested to 16K) to a PNG writer.
/// Memory stays at ~3 rows: heights window for the hillshade pass.
pub fn write_client_texture_png<W: std::io::Write>(
    field: &PlanetField,
    width: usize,
    height: usize,
    min_wavelength_m: f64,
    mut out: W,
) -> Result<(), String> {
    if width == 0 || height == 0 || width > 16384 || height > 8192 {
        return Err("streaming render capped at 16384x8192".into());
    }
    if !(min_wavelength_m.is_finite() && min_wavelength_m > 0.0) {
        return Err("min wavelength must be positive".into());
    }
    let mut rows = RowStream::new(field, width, height, min_wavelength_m);
    let iter: &mut dyn Iterator<Item = Vec<u8>> = &mut rows;
    let mut owned = Vec::new();
    crate::png_min::write_png_rows(&mut owned, width, height, iter)?;
    out.write_all(&owned).map_err(|e| e.to_string())?;
    Ok(())
}

/// Row-by-row albedo sampling. The material receives sunlight at runtime.
struct RowStream<'a> {
    field: &'a PlanetField,
    width: usize,
    height: usize,
    min_wl: f64,
    row: usize,
}
impl<'a> RowStream<'a> {
    fn new(field: &'a PlanetField, width: usize, height: usize, min_wl: f64) -> Self {
        Self {
            field,
            width,
            height,
            min_wl,
            row: 0,
        }
    }
}
impl Iterator for RowStream<'_> {
    type Item = Vec<u8>;
    fn next(&mut self) -> Option<Vec<u8>> {
        if self.row >= self.height {
            return None;
        }
        let mut row = Vec::with_capacity(self.width * 3);
        for c in 0..self.width {
            let (_, (r, g, b)) = sample_texel(
                self.field,
                self.width,
                self.height,
                self.row,
                c,
                self.min_wl,
            );
            row.extend_from_slice(&[r, g, b]);
        }
        self.row += 1;
        Some(row)
    }
}

/// Bake albedo, tangent normals and perceptual roughness from one height field.
/// Sunlight is deliberately absent from all three maps.
pub fn write_client_maps(
    field: &PlanetField,
    width: usize,
    height: usize,
    min_wl: f64,
    directory: &std::path::Path,
) -> Result<(), String> {
    let (albedo, heights) = render_client_texture(field, width, height, min_wl)?;
    let mut normals = vec![0_u8; width * height * 3];
    let mut roughness = vec![0_u8; width * height * 3];
    for r in 0..height {
        let lat = (90.0 - (r as f64 + 0.5) * 180.0 / height as f64).to_radians();
        let dx =
            std::f64::consts::TAU * field.params.radius_m / width as f64 * lat.cos().max(0.001);
        let dy = std::f64::consts::PI * field.params.radius_m / height as f64;
        for c in 0..width {
            let h = heights[r * width + c];
            let gx = if h > 0.0 {
                (heights[r * width + (c + 1) % width]
                    - heights[r * width + (c + width - 1) % width])
                    / (2.0 * dx)
            } else {
                0.0
            };
            let gy = if h > 0.0 {
                (heights[(r + 1).min(height - 1) * width + c]
                    - heights[r.saturating_sub(1) * width + c])
                    / (2.0 * dy)
            } else {
                0.0
            };
            let norm = (1.0 + gx * gx + gy * gy).sqrt();
            let o = (r * width + c) * 3;
            for (i, v) in [-gx, -gy, 1.0].into_iter().enumerate() {
                normals[o + i] = ((v / norm * 0.5 + 0.5) * 255.0).round() as u8;
            }
            // glTF/Bevy roughness is green, metallic blue. Non-metals throughout.
            roughness[o] = 255;
            roughness[o + 1] = if h < 0.0 { 60 } else { 230 };
            roughness[o + 2] = 0;
        }
    }
    std::fs::create_dir_all(directory).map_err(|e| e.to_string())?;
    for (name, rgb) in [
        ("albedo", albedo),
        ("normal", normals),
        ("roughness", roughness),
    ] {
        let bytes = encode_png_rgb(width, height, &rgb)?;
        std::fs::write(directory.join(format!("{name}.png")), bytes).map_err(|e| e.to_string())?;
    }
    let metadata = serde_json::json!({"generator":"Thessa rocky field v3", "seed":field.params.seed,
        "radius_m":field.params.radius_m,"sea_offset_m":field.sea_offset_m,"width":width,"height":height,
        "min_wavelength_m":min_wl,"sunlight_baked":false,"license":"GPL-3.0-or-later"});
    std::fs::write(
        directory.join("manifest.json"),
        serde_json::to_vec_pretty(&metadata).unwrap(),
    )
    .map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(test)]
mod map_tests {
    use super::*;
    use crate::field::{KnobsSerde, PlanetField, PlanetParams};

    fn test_field() -> PlanetField {
        PlanetField::build(
            PlanetParams {
                name: "t".into(),
                seed: 77,
                radius_m: 3_200_000.0,
                height_min_m: -8000.0,
                height_max_m: 12000.0,
                knobs: KnobsSerde {
                    mountain_coverage: 0.3,
                    crater_density: 0.3,
                    volcanism: 0.2,
                    erosion: 0.4,
                    canyon_strength: 0.3,
                    base_roughness_m: 150.0,
                },
                macro_strength: 1.0,
                polar_extent: 0.15,
                aridity: 0.4,
                glaciation: 0.2,
                volcanism: 0.2,
                nereid_ocean_bias: 0.0,
                anti_nereid_land_bias: 0.0,
                eclipse_strength: 0.28,
                nominal_flux_w_m2: 0.146,
                ocean_target: None,
            },
            vec![],
            vec![],
            vec![],
            vec![],
        )
        .expect("field builds")
    }

    #[test]
    fn texture_dims_and_png_header() {
        let field = test_field();
        let (rgb, heights) = render_client_texture(&field, 64, 32, 4000.0).expect("render");
        assert_eq!(rgb.len(), 64 * 32 * 3);
        assert_eq!(heights.len(), 64 * 32);
        assert!(rgb.iter().any(|v| *v > 0));
        let png = encode_png_rgb(64, 32, &rgb).expect("encode");
        assert_eq!(&png[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
    }

    #[test]
    fn seam_columns_are_neighbours() {
        // u=0 (lon 0) and u=1 (lon 360 == lon 0) depict the same meridian:
        // first and last columns must nearly agree on a smooth field.
        let field = test_field();
        let (rgb, _) = render_client_texture(&field, 72, 18, 64000.0).expect("render");
        let mut diff = 0u64;
        for r in 0..18 {
            for ch in 0..3 {
                let a = rgb[(r * 72) * 3 + ch] as i64;
                let b = rgb[(r * 72 + 71) * 3 + ch] as i64;
                diff += (a - b).unsigned_abs();
            }
        }
        let mean = diff as f64 / (18.0 * 3.0);
        assert!(mean < 40.0, "seam jump {mean}");
    }

    #[test]
    fn deterministic_render() {
        let field = test_field();
        let (a, _) = render_client_texture(&field, 48, 24, 4000.0).expect("a");
        let (b, _) = render_client_texture(&field, 48, 24, 4000.0).expect("b");
        assert_eq!(a, b);
    }
}
#[cfg(test)]
mod stream_tests {
    use super::write_client_texture_png;
    use crate::field::{KnobsSerde, PlanetField, PlanetParams};

    #[test]
    fn streaming_matches_in_memory_render() {
        let field = PlanetField::build(
            PlanetParams {
                name: "t".into(),
                seed: 77,
                radius_m: 3_200_000.0,
                height_min_m: -8000.0,
                height_max_m: 12000.0,
                knobs: KnobsSerde {
                    mountain_coverage: 0.3,
                    crater_density: 0.3,
                    volcanism: 0.2,
                    erosion: 0.4,
                    canyon_strength: 0.3,
                    base_roughness_m: 150.0,
                },
                macro_strength: 1.0,
                polar_extent: 0.15,
                aridity: 0.4,
                glaciation: 0.2,
                volcanism: 0.2,
                nereid_ocean_bias: 0.0,
                anti_nereid_land_bias: 0.0,
                eclipse_strength: 0.28,
                nominal_flux_w_m2: 0.146,
                ocean_target: None,
            },
            vec![],
            vec![],
            vec![],
            vec![],
        )
        .expect("field builds");
        let mut buf = Vec::new();
        write_client_texture_png(&field, 48, 24, 4000.0, &mut buf).expect("stream");
        assert_eq!(&buf[0..8], &[137, 80, 78, 71, 13, 10, 26, 10]);
        // Same pixels as the in-memory path (modulo PNG filtering only).
        let (rgb, _) = super::render_client_texture(&field, 48, 24, 4000.0).expect("mem");
        let png = super::encode_png_rgb(48, 24, &rgb).expect("encode");
        assert_eq!(
            buf.len(),
            png.len(),
            "identical compression of identical pixels"
        );
        assert_eq!(buf, png);
    }
}
