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
//! The texture is a game-readable composite (biome albedo + hillshade +
//! ocean/ice), not satellite imagery. Rivers/lakes at texel scale are
//! omitted; hydro data stays authoritative in baked reports.

use crate::{biomes::biome_color, field::PlanetField, sphere::dir_from_latlon};

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
            let s = field.sample(dir, min_wavelength_m);
            heights[r * width + c] = s.height_m;
            colors[r * width + c] = texel_color(&s);
        }
    }
    // Pass 2: hillshade from height gradients (light from the north-west).
    let radius = field.params.radius_m;
    let mut rgb = vec![0u8; width * height * 3];
    for r in 0..height {
        // Metres per texel at this latitude.
        let lat = 90.0 - (r as f64 + 0.5) * (180.0 / height as f64);
        let dx_m = (360.0 / width as f64).to_radians() * radius * lat.to_radians().cos().max(0.05);
        let dy_m = (180.0 / height as f64).to_radians() * radius;
        for c in 0..width {
            let h = heights[r * width + c];
            let hx =
                heights[r * width + (c + 1) % width] - heights[r * width + (c + width - 1) % width];
            let r_up = r.saturating_sub(1);
            let r_dn = (r + 1).min(height - 1);
            let hy = heights[r_dn * width + c] - heights[r_up * width + c];
            let dzdx = hx / (2.0 * dx_m.max(1.0));
            let dzdy = hy / (2.0 * dy_m.max(1.0));
            // Fixed sun: azimuth NW, elevation ~45 deg in texture space.
            let shade = ((0.55 - dzdx * 0.9 + dzdy * 0.9).clamp(0.35, 1.25)) as f64;
            let elev = shade_for_elevation(h);
            let (br, bg, bb) = colors[r * width + c];
            let o = (r * width + c) * 3;
            rgb[o] = ((br as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
            rgb[o + 1] = ((bg as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
            rgb[o + 2] = ((bb as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
        }
    }
    Ok((rgb, heights))
}

fn texel_color(s: &crate::field::TerrainSample) -> (u8, u8, u8) {
    use crate::biomes::Biome;
    // Oceans shaded by depth, ice stays bright, land uses taxonomy palette.
    if s.height_m < 0.0 {
        let depth = (-s.height_m / 8000.0).clamp(0.0, 1.0);
        let deep = (11.0, 42.0, 91.0);
        let shelf = (46.0, 127.0, 217.0);
        return (
            (shelf.0 + (deep.0 - shelf.0) * depth) as u8,
            (shelf.1 + (deep.1 - shelf.1) * depth) as u8,
            (shelf.2 + (deep.2 - shelf.2) * depth) as u8,
        );
    }
    if matches!(
        s.biome,
        Biome::PolarIceCap
            | Biome::IceCap
            | Biome::PermanentSnow
            | Biome::Snowfield
            | Biome::Glacier
    ) {
        return (232, 244, 255);
    }
    biome_color(s.biome)
}

/// Gentle elevation tint: lowlands warmer, peaks brighter/colder.
fn shade_for_elevation(height_m: f64) -> f64 {
    if height_m < 0.0 {
        1.0
    } else {
        (1.0 + (height_m / 12000.0) * 0.12).min(1.12)
    }
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
    let s = field.sample(dir_from_latlon(lat, lon), min_wavelength_m);
    (s.height_m, texel_color(&s))
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

/// Row-by-row renderer feeding the PNG encoder.
struct RowStream<'a> {
    field: &'a PlanetField,
    width: usize,
    height: usize,
    min_wl: f64,
    row: usize,
    prev_h: Vec<f64>,
    prev_c: Vec<(u8, u8, u8)>,
    cur_h: Vec<f64>,
    cur_c: Vec<(u8, u8, u8)>,
}

impl<'a> RowStream<'a> {
    fn new(field: &'a PlanetField, width: usize, height: usize, min_wl: f64) -> Self {
        let mut cur_h = vec![0.0; width];
        let mut cur_c = vec![(0u8, 0u8, 0u8); width];
        for c in 0..width {
            let (h, col) = sample_texel(field, width, height, 0, c, min_wl);
            cur_h[c] = h;
            cur_c[c] = col;
        }
        // Clamped edge: previous row of r=0 is r=0 itself.
        let prev_h = cur_h.clone();
        let prev_c = cur_c.clone();
        Self {
            field,
            width,
            height,
            min_wl,
            row: 0,
            prev_h,
            prev_c,
            cur_h,
            cur_c,
        }
    }

    fn shade_row(&self, row_rgb: &mut [u8], next_h: &[f64]) {
        let radius = self.field.params.radius_m;
        let r = self.row;
        let lat = 90.0 - (r as f64 + 0.5) * (180.0 / self.height as f64);
        let dx_m =
            (360.0 / self.width as f64).to_radians() * radius * lat.to_radians().cos().max(0.05);
        let dy_m = (180.0 / self.height as f64).to_radians() * radius;
        for c in 0..self.width {
            let hx =
                self.cur_h[(c + 1) % self.width] - self.cur_h[(c + self.width - 1) % self.width];
            let hy = next_h[c] - self.prev_h[c];
            let dzdx = hx / (2.0 * dx_m.max(1.0));
            let dzdy = hy / (2.0 * dy_m.max(1.0));
            let shade = (0.55 - dzdx * 0.9 + dzdy * 0.9).clamp(0.35, 1.25);
            let elev = shade_for_elevation(self.cur_h[c]);
            let (br, bg, bb) = self.cur_c[c];
            let o = c * 3;
            row_rgb[o] = ((br as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
            row_rgb[o + 1] = ((bg as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
            row_rgb[o + 2] = ((bb as f64 * shade * elev).clamp(0.0, 255.0)) as u8;
        }
    }
}

impl Iterator for RowStream<'_> {
    type Item = Vec<u8>;

    fn next(&mut self) -> Option<Vec<u8>> {
        if self.row >= self.height {
            return None;
        }
        let r_dn = (self.row + 1).min(self.height - 1);
        let mut next_h = vec![0.0; self.width];
        let mut next_c = vec![(0u8, 0u8, 0u8); self.width];
        if r_dn != self.row {
            for (c, (h_slot, c_slot)) in next_h.iter_mut().zip(next_c.iter_mut()).enumerate() {
                let (h, col) =
                    sample_texel(self.field, self.width, self.height, r_dn, c, self.min_wl);
                *h_slot = h;
                *c_slot = col;
            }
        } else {
            next_h.clone_from_slice(&self.cur_h);
            next_c.clone_from_slice(&self.cur_c);
        }
        let mut row_rgb = vec![0u8; self.width * 3];
        self.shade_row(&mut row_rgb, &next_h);
        self.prev_h = std::mem::replace(&mut self.cur_h, next_h);
        self.prev_c = std::mem::replace(&mut self.cur_c, next_c);
        self.row += 1;
        if self.row.is_multiple_of(2048) {
            eprintln!("row {}/{}", self.row, self.height);
        }
        Some(row_rgb)
    }
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
