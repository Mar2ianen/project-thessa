//! WORLD00: a bounded coastline search and CPU-only final-field patch audit.
//! No erosion rebake, flight-authority dependency, or renderer is involved.
use serde::Serialize;
use serde_json::{Value, json};
use thessa_worldgen_rocky::{
    appearance,
    field::{PlanetField, field_from_manifest},
    offline::FrozenSurface,
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::{dir_from_latlon, enu_to_dir, great_circle_m, latlon_from_dir},
};

const CUTOFF_M: f64 = 32.0;
const HEIGHT_TOLERANCE_M: f64 = 1e-6;
const BRACKET_TOLERANCE_M: f64 = 0.001;
const GRID_STRIDE: usize = 4;
const PATCH_CELLS: usize = 64;
const PATCH_WIDTH_M: f64 = 2000.0;
const BOOKMARK: [f64; 2] = [23.0, 165.0];

fn canonical_field() -> Result<PlanetField, Box<dyn std::error::Error>> {
    // Same recipe -> manifest -> field -> source-checked v3 attachment as
    // flight-authority::canonical_world_field, without depending on game code.
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))?;
    let source = field_from_manifest(&manifest_from_spec(&recipe)?)?;
    let frozen = FrozenSurface::read_gzip(
        &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
    )?;
    Ok(source.with_frozen_erosion(&frozen)?)
}

#[derive(Clone, Copy, Serialize)]
struct Point {
    lat_lon_deg: [f64; 2],
    height_m: f64,
}

impl Point {
    fn at(field: &PlanetField, lat_lon_deg: [f64; 2]) -> Self {
        Self {
            lat_lon_deg,
            height_m: field.height_m(dir_from_latlon(lat_lon_deg[0], lat_lon_deg[1]), CUTOFF_M),
        }
    }

    fn dir(self) -> [f64; 3] {
        dir_from_latlon(self.lat_lon_deg[0], self.lat_lon_deg[1])
    }
}

#[derive(Serialize)]
struct Crossing {
    original_grid_endpoints: [Point; 2],
    final_bracket_endpoints: [Point; 2],
    midpoint: Point,
    bracket_width_m: f64,
    bisections: usize,
    bookmark_distance_m: f64,
}

fn bisect(field: &PlanetField, a: Point, b: Point) -> Result<Crossing, String> {
    if (a.height_m < 0.0) == (b.height_m < 0.0) {
        return Err("bisection requires opposite land/submerged endpoint signs".into());
    }
    let original_grid_endpoints = [a, b];
    let (mut low, mut high) = if a.height_m < 0.0 { (a, b) } else { (b, a) };
    for bisections in 0..=64 {
        // The stored regional longitudes are unwrapped; interpolation stays
        // within the original grid edge, including across a longitude seam.
        let midpoint = Point::at(
            field,
            std::array::from_fn(|i| (low.lat_lon_deg[i] + high.lat_lon_deg[i]) * 0.5),
        );
        let bracket_width_m = great_circle_m(low.dir(), high.dir(), field.params.radius_m);
        if bracket_width_m <= BRACKET_TOLERANCE_M && midpoint.height_m.abs() <= HEIGHT_TOLERANCE_M {
            return Ok(Crossing {
                original_grid_endpoints,
                final_bracket_endpoints: [low, high],
                midpoint,
                bracket_width_m,
                bisections,
                bookmark_distance_m: great_circle_m(
                    midpoint.dir(),
                    dir_from_latlon(BOOKMARK[0], BOOKMARK[1]),
                    field.params.radius_m,
                ),
            });
        }
        if midpoint.height_m < 0.0 {
            low = midpoint;
        } else {
            high = midpoint;
        }
    }
    Err("zero crossing did not converge; do not substitute a lowland bookmark".into())
}

fn search(field: &PlanetField) -> Result<(Vec<Crossing>, Value), String> {
    let region = field
        .regional_erosion()
        .iter()
        .find(|p| p.recipe.id == "humid-coastal-lowland")
        .ok_or("bundled coastal region missing")?;
    let lats: Vec<_> = region.lats.iter().step_by(GRID_STRIDE).copied().collect();
    let lons: Vec<_> = region.lons.iter().step_by(GRID_STRIDE).copied().collect();
    let mut grid = Vec::new();
    for &lat in &lats {
        for &lon in &lons {
            grid.push(Point::at(field, [lat, lon]));
        }
    }
    let mut crossings = Vec::new();
    // Stable row-major order, east edge before north edge; stable sorting
    // below retains this tie-break for equal-distance candidates.
    for r in 0..lats.len() {
        for c in 0..lons.len() {
            let a = grid[r * lons.len() + c];
            for (nr, nc) in [(r, c + 1), (r + 1, c)] {
                if nr < lats.len() && nc < lons.len() {
                    let b = grid[nr * lons.len() + nc];
                    if (a.height_m < 0.0) != (b.height_m < 0.0) {
                        crossings.push(bisect(field, a, b)?);
                    }
                }
            }
        }
    }
    crossings.sort_by(|a, b| a.bookmark_distance_m.total_cmp(&b.bookmark_distance_m));
    let min_h = grid
        .iter()
        .map(|p| p.height_m)
        .fold(f64::INFINITY, f64::min);
    let max_h = grid
        .iter()
        .map(|p| p.height_m)
        .fold(f64::NEG_INFINITY, f64::max);
    let report = json!({
        "region": region.recipe,
        "lat_bounds_deg": [lats[0], lats[lats.len()-1]],
        "unwrapped_lon_bounds_deg": [lons[0], lons[lons.len()-1]],
        "grid_vertices": [lons.len(), lats.len()], "regional_vertex_stride": GRID_STRIDE,
        "nominal_spacing_m": region.recipe.spacing_m * GRID_STRIDE as f64,
        "final_height_min_max_m": [min_h, max_h], "crossing_edges": crossings.len(),
        "selection": "nearest bisected grid-edge crossing to [23N,165E]; stable row/east/north tie order",
        "not_a_continuous_nearest_coastline_certificate": true,
    });
    Ok((crossings, report))
}

fn distribution(mut values: Vec<f64>) -> [f64; 6] {
    values.sort_by(f64::total_cmp);
    let last = values.len() - 1;
    [0, last / 10, last / 2, last * 9 / 10, last * 99 / 100, last].map(|i| values[i])
}

fn patch(field: &PlanetField, crossing: &Crossing) -> Result<Value, String> {
    let center = crossing.midpoint.dir();
    let spacing_m = PATCH_WIDTH_M / PATCH_CELLS as f64;
    let mut heights = Vec::new();
    let mut slopes = Vec::new();
    let mut land_slopes = Vec::new();
    let mut land = None;
    let mut land_distance = f64::INFINITY;
    for r in 0..=PATCH_CELLS {
        for c in 0..=PATCH_CELLS {
            let enu = [
                c as f64 * spacing_m - PATCH_WIDTH_M * 0.5,
                r as f64 * spacing_m - PATCH_WIDTH_M * 0.5,
                0.0,
            ];
            let dir = enu_to_dir(enu, center, field.params.radius_m);
            let height = field.height_m(dir, CUTOFF_M);
            // Unlike sample().slope_hint this stencil evaluates the actual
            // final field, including spatially varying frozen displacement.
            let (slope, _) = field.surface_geometry(dir, CUTOFF_M);
            heights.push(height);
            slopes.push(slope);
            if height > 0.0 {
                land_slopes.push(slope);
            }
            // A reproducible nearby land camera target, not a pad certificate.
            let distance = enu[0].hypot(enu[1]);
            if height >= 2.0 && (128.0..=1000.0).contains(&distance) && distance < land_distance {
                land_distance = distance;
                land = Some((dir, enu));
            }
        }
    }
    let submerged = heights.iter().filter(|h| **h < 0.0).count();
    if submerged == 0 || submerged == heights.len() {
        return Err("selected 2 km patch has no sampled land/water contrast".into());
    }
    let (dir, enu) = land.ok_or("no >=2 m land camera target 128..1000 m from crossing")?;
    let (lat, lon) = latlon_from_dir(dir);
    let mut sample = field.sample_surface(dir, CUTOFF_M);
    // Match material-page geometry scale for cover/optical state; report
    // this separately from the final 32 m-cutoff slope distribution.
    let (material_slope, curvature) = field.surface_geometry(dir, 256.0);
    sample.slope_hint = material_slope;
    sample.curvature_per_m = curvature;
    let cover = appearance::surface_appearance(field, &sample, dir);
    let corners: Vec<_> = [
        [-1000.0, -1000.0],
        [1000.0, -1000.0],
        [1000.0, 1000.0],
        [-1000.0, 1000.0],
    ]
    .map(|[e, n]| {
        let (lat, lon) = latlon_from_dir(enu_to_dir([e, n, 0.0], center, field.params.radius_m));
        [lat, lon]
    })
    .into();
    Ok(json!({
        "center_lat_lon_deg": crossing.midpoint.lat_lon_deg,
        "enu_width_height_m": [PATCH_WIDTH_M, PATCH_WIDTH_M],
        "grid_vertices": [PATCH_CELLS+1, PATCH_CELLS+1], "enu_spacing_m": spacing_m,
        "corner_lat_lon_deg_sw_se_ne_nw": corners,
        "sample_count": heights.len(), "submerged_vertices": submerged,
        "submerged_vertex_fraction": submerged as f64 / heights.len() as f64,
        "final_height_min_p10_p50_p90_p99_max_m": distribution(heights.clone()),
        "final_slope_rise_run_min_p10_p50_p90_p99_max": distribution(slopes),
        "land_final_slope_rise_run_min_p10_p50_p90_p99_max": distribution(land_slopes),
        "height_rows_south_to_north_west_to_east_m": heights.chunks(PATCH_CELLS+1).collect::<Vec<_>>(),
        "land_camera_target": {
            "lat_lon_deg": [lat, lon], "enu_from_crossing_m": enu,
            "distance_to_selected_crossing_m": great_circle_m(dir, center, field.params.radius_m),
            "height_m": sample.height_m, "temperature_k": sample.temperature_k,
            "moisture01": sample.moisture01, "wetland_potential01": sample.wetland_potential01,
            "geothermal_flux_w_m2": sample.geothermal_flux_w_m2,
            "biome": format!("{:?}", sample.biome), "geology": format!("{:?}", sample.geology),
            "erosion_displacement_m": sample.erosion_displacement_m,
            "local_deposition_m": sample.local_deposition_m,
            "final_slope_rise_run_32m": field.surface_geometry(dir, CUTOFF_M).0,
            "material_geometry_scale_m": 256.0, "material_slope_rise_run": material_slope,
            "vegetation": cover.vegetation, "snow_suitability": cover.snow,
            "snow_cover": cover.snow_cover, "frost_cover": cover.frost_cover,
            "albedo_srgb": cover.albedo_srgb, "roughness": cover.roughness,
        },
    }))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args_os().len() != 1 {
        return Err("surface_patch_audit takes no arguments; WORLD00 is a fixed scenario".into());
    }
    let field = canonical_field()?;
    let (crossings, search_report) = search(&field)?;
    let crossing = crossings
        .first()
        .ok_or("no zero crossing inside bundled coastal region; broader search required")?;
    let report = json!({
        "scenario": "WORLD00-coastal-2km-v1", "audit_only_not_universal_geometry_gate": true,
        "recipe": "data/worldgen/worldgen_recipe.toml",
        "frozen_artifact": "data/worldgen/thessa-erosion-v3.surface.json.gz",
        "source_identity_checked_by_attachment": field.has_frozen_erosion(),
        "seed": field.params.seed, "radius_m": field.params.radius_m,
        "final_height_min_wavelength_m": CUTOFF_M,
        "height_residual_tolerance_m": HEIGHT_TOLERANCE_M,
        "bracket_width_tolerance_m": BRACKET_TOLERANCE_M,
        "old_bookmark": Point::at(&field, BOOKMARK), "search": search_report,
        "crossing": crossing, "patch": patch(&field, crossing)?,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world00_canonical_site_numeric_regression() {
        let field = canonical_field().expect("matched bundled canonical source");
        assert_eq!(field.params.seed, 7);
        assert_eq!(field.params.radius_m, 3_200_000.0);
        let (crossings, _) = search(&field).expect("bounded coastal scan");
        assert_eq!(crossings.len(), 954);
        let crossing = &crossings[0];
        for (actual, expected) in crossing
            .midpoint
            .lat_lon_deg
            .into_iter()
            .zip([23.567917375616958, 165.13942625682847])
        {
            assert!((actual - expected).abs() < 1e-8);
        }
        assert!(crossing.final_bracket_endpoints[0].height_m < 0.0);
        assert!(crossing.final_bracket_endpoints[1].height_m >= 0.0);
        assert!(crossing.bracket_width_m <= BRACKET_TOLERANCE_M);
        assert!(crossing.midpoint.height_m.abs() <= HEIGHT_TOLERANCE_M);
        let patch = patch(&field, crossing).expect("sampled coastal contrast and land target");
        assert_eq!(patch["sample_count"], 4225);
        assert_eq!(patch["submerged_vertices"], 566);
        let heights = &patch["final_height_min_p10_p50_p90_p99_max_m"];
        assert!((heights[0].as_f64().unwrap() + 10.742522750814077).abs() < 1e-5);
        assert!((heights[5].as_f64().unwrap() - 17.731800419943614).abs() < 1e-5);
        let land = &patch["land_camera_target"];
        assert!((land["height_m"].as_f64().unwrap() - 4.557292563542587).abs() < 1e-5);
        assert_eq!(land["enu_from_crossing_m"], json!([-31.25, -125.0, 0.0]));
        assert_eq!(land["geology"], "Regolith");
        assert_eq!(land["biome"], "TemperateForest");
        // This pins one artifact/scenario; it is not a universal error gate.
    }
}
