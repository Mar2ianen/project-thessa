//! Equal-area audit of the live canonical field, not the eroded bake grid.
use std::{collections::BTreeMap, time::Instant};
use thessa_worldgen_rocky::{appearance, client_export, field, lod, spec_recipe};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let recipe: spec_recipe::SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))?;
    let started = Instant::now();
    let planet = field::field_from_manifest(&spec_recipe::manifest_from_spec(&recipe)?)?;
    let build_ms = started.elapsed().as_secs_f64() * 1000.0;
    let mut heights = Vec::new();
    let (mut temperature, mut flux, mut snow, mut vegetation) = (0.0, 0.0, 0.0, 0.0);
    let mut wetland_potential = 0.0;
    let mut biomes = BTreeMap::<String, usize>::new();
    const N: usize = 16384;
    for i in 0..N {
        let y = 1.0 - 2.0 * (i as f64 + 0.5) / N as f64;
        let angle = i as f64 * 2.399963229728653;
        let r = (1.0 - y * y).sqrt();
        let dir = [r * angle.cos(), y, r * angle.sin()];
        let sample = planet.sample_surface(dir, 32.0);
        let material = appearance::surface_appearance(&planet, &sample, dir);
        heights.push(sample.height_m);
        temperature += sample.temperature_k;
        flux += sample.geothermal_flux_w_m2;
        snow += f64::from(material.snow);
        vegetation += f64::from(material.vegetation);
        wetland_potential += sample.wetland_potential01;
        *biomes.entry(format!("{:?}", sample.biome)).or_default() += 1;
    }
    heights.sort_by(f64::total_cmp);
    // Measure actual landmark neighbourhoods, not only recipe dimensions.
    // The spherical exponential map works across the dateline and near poles.
    let landmarks: Vec<_> = planet
        .features
        .iter()
        .map(|feature| {
            let center =
                thessa_worldgen_rocky::sphere::dir_from_latlon(feature.lat_deg, feature.lon_deg);
            let (east, north, _) = thessa_worldgen_rocky::sphere::enu_basis(center);
            let mut min_h = f64::INFINITY;
            let mut max_h = f64::NEG_INFINITY;
            let mut min_delta = f64::INFINITY;
            let mut max_delta = f64::NEG_INFINITY;
            let half_extent = feature.reach_m() * 0.65;
            for row in -24..=24 {
                for col in -24..=24 {
                    let x = col as f64 / 24.0 * half_extent;
                    let y = row as f64 / 24.0 * half_extent;
                    let distance = x.hypot(y);
                    let angle = distance / planet.params.radius_m;
                    let dir = if distance == 0.0 {
                        center
                    } else {
                        std::array::from_fn(|i| {
                            center[i] * angle.cos()
                                + (east[i] * x + north[i] * y) / distance * angle.sin()
                        })
                    };
                    let h = planet.height_m(dir, 32.0);
                    let (lat, lon) = thessa_worldgen_rocky::sphere::latlon_from_dir(dir);
                    let delta = thessa_worldgen_rocky::features::eval_feature_height_m(
                        feature,
                        lat,
                        lon,
                        planet.params.radius_m,
                    );
                    min_h = min_h.min(h);
                    max_h = max_h.max(h);
                    min_delta = min_delta.min(delta);
                    max_delta = max_delta.max(delta);
                }
            }
            serde_json::json!({
                "id": feature.id, "seed": feature.seed,
                "center_lat_lon_deg": [feature.lat_deg, feature.lon_deg],
                "shape": feature.feature,
                "survey_grid": [49, 49], "survey_half_extent_m": half_extent,
                "sampled_height_min_max_m": [min_h, max_h],
                "sampled_feature_delta_min_max_m": [min_delta, max_delta],
                "bounds_are_sampled_not_certified": true,
            })
        })
        .collect();
    let mut pages_ms = Vec::new();
    for level in [4, 8, 12, 16] {
        let key = lod::TileKey {
            face: 0,
            level,
            x: (1 << level) / 2,
            y: (1 << level) / 2,
        };
        let started = Instant::now();
        std::hint::black_box(lod::build_gpu_material_page(&planet, key));
        pages_ms.push((level, started.elapsed().as_secs_f64() * 1000.0));
    }
    let report = serde_json::json!({
        "profile": if cfg!(debug_assertions) { "debug" } else { "release" },
        "measurement_kind": "single_run_cpu_field_audit_not_renderer_benchmark",
        "recipe_source": "data/worldgen/worldgen_recipe.toml",
        "landmarks": landmarks,
        "seed": recipe.planet.seed, "equal_area_samples": N, "min_wavelength_m": 32.0,
        "build_ms": build_ms, "ocean_fraction": heights.iter().filter(|h| **h < 0.0).count() as f64 / N as f64,
        "height_min_p10_p50_p90_p99_max_m": [heights[0], heights[N/10], heights[N/2], heights[N*9/10], heights[N*99/100], heights[N-1]],
        "temperature_mean_k": temperature / N as f64, "geothermal_mean_w_m2": flux / N as f64,
        "snow_coverage_mean": snow / N as f64, "vegetation_coverage_mean": vegetation / N as f64,
        "regional_wetland_potential_area_mean": wetland_potential / N as f64,
        "biome_counts": biomes, "material_page_level_build_ms": pages_ms,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if let Some(path) = std::env::args().nth(1) {
        let (rgb, _) = client_export::render_client_texture(&planet, 480, 270, 4000.0)?;
        std::fs::write(path, client_export::encode_png_rgb(480, 270, &rgb)?)?;
    }
    Ok(())
}
