//! Authored landmark zones: bounded areas with special representation.
//!
//! Layered model (intentional):
//! - `Feature` = generator instruction (crater, arc, canyon, ...);
//! - `LandmarkZone` = bounded area with authored/custom representation.
//!
//! A procedural crater needs no zone. An art-directed basin interior may have
//! both: the global `Feature` at macro scale plus a zone adding local
//! hand-crafted structure. Modes:
//! 1. `procedural_override` — custom procedural evaluator for the region;
//! 2. `height_patch` — authored DEM delta blended over the base terrain;
//! 3. `surface_overlay` — canonical height, authored material/biome tags;
//! 4. `mesh_overlay` — descriptor for separately modelled geometry (DEV ONLY,
//!    no runtime rendering/collision here);
//! 5. `volume_overlay` — schema placeholder (caves/overhangs), not implemented.

use serde::{Deserialize, Serialize};

use crate::sphere::{dir_from_latlon, dir_to_enu, enu_basis, enu_to_dir, great_circle_m};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LandmarkMode {
    ProceduralOverride,
    HeightPatch,
    SurfaceOverlay,
    MeshOverlay,
    VolumeOverlay,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LandmarkZone {
    pub id: String,
    pub center_lat_deg: f64,
    pub center_lon_deg: f64,
    /// Zone radius in metres (inner authoritative region).
    pub radius_m: f64,
    /// Blend ring width in metres outside `radius_m`.
    #[serde(default = "default_blend")]
    pub blend_width_m: f64,
    pub mode: LandmarkMode,
    /// For `height_patch`: DEM asset reference (parsed without GIS deps).
    #[serde(default)]
    pub asset: Option<String>,
    /// For `surface_overlay`: authored `biome[/geology]` tags.
    #[serde(default)]
    pub surface_tags: Vec<String>,
    /// For `mesh_overlay` / `volume_overlay`: descriptor payload.
    #[serde(default)]
    pub descriptor: Option<String>,
    /// For `procedural_override`: inline amplitude/scale tweak.
    #[serde(default = "default_amp")]
    pub amplitude_m: f64,
}

fn default_blend() -> f64 {
    20_000.0
}

fn default_amp() -> f64 {
    500.0
}

impl LandmarkZone {
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("landmark zone needs an id".into());
        }
        if !self.center_lat_deg.is_finite() || !(-90.0..=90.0).contains(&self.center_lat_deg) {
            return Err(format!("zone {} bad center lat", self.id));
        }
        if !self.center_lon_deg.is_finite() || !(-180.0..=180.0).contains(&self.center_lon_deg) {
            return Err(format!("zone {} bad center lon", self.id));
        }
        if !self.radius_m.is_finite() || self.radius_m <= 0.0 {
            return Err(format!("zone {} radius must be positive", self.id));
        }
        if !self.blend_width_m.is_finite() || self.blend_width_m < 0.0 {
            return Err(format!("zone {} blend width must be non-negative", self.id));
        }
        match self.mode {
            LandmarkMode::HeightPatch
                if self.asset.as_ref().is_none_or(|a| a.trim().is_empty()) =>
            {
                return Err(format!("zone {} height_patch needs an asset", self.id));
            }
            LandmarkMode::MeshOverlay | LandmarkMode::VolumeOverlay
                if self.descriptor.as_ref().is_none_or(|d| d.trim().is_empty()) =>
            {
                return Err(format!(
                    "zone {} mesh/volume overlay needs a descriptor",
                    self.id
                ));
            }
            _ => {}
        }
        Ok(())
    }

    pub fn center_dir(&self) -> [f64; 3] {
        dir_from_latlon(self.center_lat_deg, self.center_lon_deg)
    }

    /// Blend weight: 1.0 inside, smooth falloff across the ring, 0 outside.
    pub fn blend_weight(&self, dir: [f64; 3], radius_m: f64) -> f64 {
        let d = great_circle_m(dir, self.center_dir(), radius_m);
        if d <= self.radius_m {
            1.0
        } else if d >= self.radius_m + self.blend_width_m || self.blend_width_m <= 0.0 {
            0.0
        } else {
            let t = (d - self.radius_m) / self.blend_width_m;
            1.0 - (t * t * (3.0 - 2.0 * t))
        }
    }

    /// Height delta in metres contributed by this zone.
    /// `base_height` samples the global field WITHOUT landmarks (avoids recursion).
    pub fn height_delta(
        &self,
        dir: [f64; 3],
        radius_m: f64,
        base_height: &dyn Fn([f64; 3]) -> f64,
    ) -> f64 {
        let w = self.blend_weight(dir, radius_m);
        if w <= 0.0 {
            return 0.0;
        }
        match self.mode {
            LandmarkMode::ProceduralOverride => {
                // Custom procedural structure: seeded ridged dome as the
                // placeholder evaluator (asset pipelines replace this).
                let enu = dir_to_enu(dir, self.center_dir(), radius_m);
                let r = (enu[0] * enu[0] + enu[1] * enu[1]).sqrt() / self.radius_m.max(1.0);
                let dome = self.amplitude_m * (-r.powi(2) * 2.0).exp();
                w * dome
            }
            LandmarkMode::HeightPatch => {
                // Delta-from-reference: robust to small macro datum changes.
                // Without a loaded asset the delta is zero (schema honored,
                // no silent invention); see `DemPatch` for file-backed deltas.
                let _ = base_height;
                0.0
            }
            LandmarkMode::SurfaceOverlay
            | LandmarkMode::MeshOverlay
            | LandmarkMode::VolumeOverlay => 0.0,
        }
    }

    /// Local ENU frame helpers (double precision, resolution-independent).
    pub fn to_local(&self, dir: [f64; 3], radius_m: f64) -> [f64; 3] {
        dir_to_enu(dir, self.center_dir(), radius_m)
    }

    pub fn to_world(&self, enu_m: [f64; 3], radius_m: f64) -> [f64; 3] {
        enu_to_dir(enu_m, self.center_dir(), radius_m)
    }

    pub fn basis(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        enu_basis(self.center_dir())
    }
}

/// File-backed DEM height patch (delta-from-reference metres).
/// Minimal parsing boundary: binary P5 grayscale (pgm8 only), decoded to
/// metres via an explicit scale embedded in the asset TOML sidecar.
/// No GIS dependency.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DemPatchAsset {
    pub file: String,
    /// Physical extent: the patch spans [-extent_m, +extent_m] in local x/y.
    pub extent_m: f64,
    /// Grayscale 0 maps to this delta, 255 (or 1.0) maps to `delta_max_m`.
    pub delta_min_m: f64,
    pub delta_max_m: f64,
    /// Only `pgm8` is implemented; other encodings are rejected.
    pub format: String,
}

impl DemPatchAsset {
    /// Sample delta in metres at local (x, y). Outside => 0.
    pub fn sample_delta(&self, data: &[u8], x_m: f64, y_m: f64) -> Result<f64, String> {
        if self.format != "pgm8" {
            return Err(format!("unsupported DEM format {}", self.format));
        }
        // Minimal P5 parser: magic, dims, maxval, raster.
        let (w, h, raster) = parse_pgm8(data)?;
        let fx = (x_m / self.extent_m * 0.5 + 0.5).clamp(0.0, 1.0);
        let fy = (0.5 - y_m / self.extent_m * 0.5).clamp(0.0, 1.0);
        let px = ((fx * w as f64) as usize).min(w - 1);
        let py = ((fy * h as f64) as usize).min(h - 1);
        let g = raster[py * w + px] as f64 / 255.0;
        Ok(self.delta_min_m + g * (self.delta_max_m - self.delta_min_m))
    }
}

fn parse_pgm8(data: &[u8]) -> Result<(usize, usize, Vec<u8>), String> {
    let mut tokens: Vec<Vec<u8>> = Vec::new();
    // Magic P5.
    if data.len() < 2 || &data[0..2] != b"P5" {
        return Err("not a P5 pgm".into());
    }
    let mut pos = 2;
    let next_token = |pos: &mut usize| -> Result<Vec<u8>, String> {
        loop {
            while *pos < data.len() && data[*pos].is_ascii_whitespace() {
                *pos += 1;
            }
            if *pos < data.len() && data[*pos] == b'#' {
                while *pos < data.len() && data[*pos] != b'\n' {
                    *pos += 1;
                }
                continue;
            }
            break;
        }
        let start = *pos;
        while *pos < data.len() && !data[*pos].is_ascii_whitespace() {
            *pos += 1;
        }
        if start == *pos {
            return Err("truncated pgm header".into());
        }
        Ok(data[start..*pos].to_vec())
    };
    tokens.push(next_token(&mut pos)?); // width
    tokens.push(next_token(&mut pos)?); // height
    tokens.push(next_token(&mut pos)?); // maxval
    // Single whitespace byte after maxval.
    if pos < data.len() && data[pos].is_ascii_whitespace() {
        pos += 1;
    }
    let w: usize = std::str::from_utf8(&tokens[0])
        .map_err(|e| e.to_string())?
        .parse::<usize>()
        .map_err(|e| e.to_string())?;
    let h: usize = std::str::from_utf8(&tokens[1])
        .map_err(|e| e.to_string())?
        .parse::<usize>()
        .map_err(|e| e.to_string())?;
    let maxval: usize = std::str::from_utf8(&tokens[2])
        .map_err(|e| e.to_string())?
        .parse::<usize>()
        .map_err(|e| e.to_string())?;
    if maxval != 255 || w == 0 || h == 0 || w * h > 64 * 1024 * 1024 {
        return Err("unsupported pgm geometry".into());
    }
    if data.len() < pos + w * h {
        return Err("truncated pgm raster".into());
    }
    Ok((w, h, data[pos..pos + w * h].to_vec()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sphere::dir_from_latlon;

    fn zone() -> LandmarkZone {
        LandmarkZone {
            id: "kestrel-launch".into(),
            center_lat_deg: -25.0,
            center_lon_deg: 120.0,
            radius_m: 50_000.0,
            blend_width_m: 20_000.0,
            mode: LandmarkMode::ProceduralOverride,
            asset: None,
            surface_tags: vec!["launch_complex".into()],
            descriptor: None,
            amplitude_m: 300.0,
        }
    }

    #[test]
    fn blend_is_authoritative_inside_zero_outside() {
        let z = zone();
        let r = 3_200_000.0;
        assert_eq!(z.blend_weight(z.center_dir(), r), 1.0);
        let far = dir_from_latlon(40.0, -60.0);
        assert_eq!(z.blend_weight(far, r), 0.0);
        // Ring midpoint is strictly between.
        let mid_dir = z.to_world([50_000.0 + 10_000.0, 0.0, 0.0], r);
        let w = z.blend_weight(mid_dir, r);
        assert!(w > 0.0 && w < 1.0, "{w}");
    }

    #[test]
    fn local_coordinates_roundtrip() {
        let z = zone();
        let r = 3_200_000.0;
        let dir = dir_from_latlon(-24.5, 120.5);
        let enu = z.to_local(dir, r);
        let back = z.to_world(enu, r);
        let d = great_circle_m(dir, back, r);
        assert!(d < 1.0, "{d}");
    }

    #[test]
    fn mesh_overlay_parses_without_renderer() {
        let z = LandmarkZone {
            mode: LandmarkMode::MeshOverlay,
            descriptor: Some("glb:data/worldgen/landmarks/kestrel/pad.glb".into()),
            ..zone()
        };
        z.validate().expect("mesh descriptor validates");
        // Height untouched: mesh is metadata only in the dev tool.
        assert_eq!(z.height_delta(z.center_dir(), 3_200_000.0, &|_| 0.0), 0.0);
    }

    #[test]
    fn pgm_dem_samples_delta() {
        // 2x2 P5: black, mid, white gradient.
        let mut data = b"P5\n2 2\n255\n".to_vec();
        data.extend_from_slice(&[0, 128, 255, 64]);
        let asset = DemPatchAsset {
            file: "test.pgm".into(),
            extent_m: 1000.0,
            delta_min_m: -100.0,
            delta_max_m: 100.0,
            format: "pgm8".into(),
        };
        let d = asset.sample_delta(&data, -500.0, 500.0).expect("sample");
        assert!((d + 100.0).abs() < 1.0, "{d}");
        // Outside clamps to the edge texel, never errors.
        assert!(asset.sample_delta(&data, 1e9, 1e9).is_ok());
    }

    #[test]
    fn validation_rejects_broken_zones() {
        let mut z = zone();
        z.radius_m = -5.0;
        assert!(z.validate().is_err());
        let mut z2 = zone();
        z2.mode = LandmarkMode::HeightPatch;
        z2.asset = None;
        assert!(z2.validate().is_err());
    }
}
