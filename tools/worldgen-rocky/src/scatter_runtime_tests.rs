//! CPU-only tests of the attached canonical surface, not a foliage renderer.
use super::*;
use crate::{
    ecology::vegetation_cover,
    field::{PlanetField, field_from_manifest},
    offline::FrozenSurface,
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::{dir_from_latlon, enu_to_dir},
};
use std::sync::{Arc, OnceLock};

fn canonical_field() -> &'static Arc<PlanetField> {
    static FIELD: OnceLock<Arc<PlanetField>> = OnceLock::new();
    FIELD.get_or_init(|| {
        // Same source-checked attachment as launch_site::canonical_world_field
        // and the WORLD00 example, without a game-crate dependency or rebake.
        let recipe: SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let source = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
        let frozen = FrozenSurface::read_gzip(
            &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
        )
        .unwrap();
        Arc::new(source.with_frozen_erosion(&frozen).unwrap())
    })
}

// Existing WORLD00 land target and the L14 cube window containing it.
const LAND: [f64; 2] = [23.565679258039204, 165.13881581863882];
const LAND_WINDOW: TileKey = TileKey {
    face: 1,
    level: 14,
    x: 6018,
    y: 11888,
};

fn query(region: TileKey, policy: ScatterPolicy) -> ScatterBatch {
    let field = canonical_field();
    scatter_region(
        field.params.seed,
        field.params.radius_m,
        region,
        ScatterPopulation::Plants,
        policy,
        |dir| canonical_plant_surface(field, dir),
    )
    .unwrap()
}

#[test]
fn world00_ground_and_metric_slope_are_not_the_zero_semantic_hint() {
    let field = canonical_field();
    assert!(field.has_frozen_erosion());
    assert!(!field.regional_erosion().is_empty());
    assert_eq!(PLANT_SURFACE_WAVELENGTH_M, 32.0);
    let dir = dir_from_latlon(LAND[0], LAND[1]);
    assert!(
        crate::sphere::great_circle_m(dir, LAND_WINDOW.direction(0.5, 0.5), field.params.radius_m)
            < 300.0
    );
    let sample = field.sample_surface(dir, PLANT_SURFACE_WAVELENGTH_M);
    let surface = canonical_plant_surface(field, dir);
    assert_eq!(sample.slope_hint, 0.0);
    assert_eq!(surface.ground_height_m, field.height_m(dir, 32.0));
    // Reuse the dated WORLD00 source/site values, not a new terrain correction.
    assert!((surface.ground_height_m - 4.557292563542587).abs() < 1e-5);
    assert!((surface.slope_rise_run - 0.002384743).abs() < 1e-9);
    assert_eq!(surface.slope_rise_run, field.surface_geometry(dir, 32.0).0);
    assert_eq!(surface.water_height_m, Some(0.0));
    assert_eq!(surface.rocky01, 0.0);

    // WORLD00's original submerged crossing endpoint is the known ocean case.
    let ocean_dir = dir_from_latlon(23.568374132769776, 165.13942625682847);
    let ocean = canonical_plant_surface(field, ocean_dir);
    assert!((ocean.ground_height_m - -2.0759892266078985).abs() < 1e-5);
    assert_eq!(ocean.water_height_m, Some(0.0));
    assert_eq!(ocean.vegetation.total(), 0.0);
}

#[test]
fn ecology_matches_fixed_material_geometry_not_placement_or_query_lod() {
    let field = canonical_field();
    assert_eq!(crate::lod::MATERIAL_SLOPE_WAVELENGTH_M, 256.0);
    let center = dir_from_latlon(LAND[0], LAND[1]);
    let mut distinct_cover = false;
    for north in -4..=4 {
        for east in -4..=4 {
            let dir = enu_to_dir(
                [east as f64 * 250.0, north as f64 * 250.0, 0.0],
                center,
                field.params.radius_m,
            );
            let mut sample = field.sample_surface(dir, 32.0);
            let (slope, curvature) =
                field.surface_geometry(dir, crate::lod::MATERIAL_SLOPE_WAVELENGTH_M);
            sample.slope_hint = slope;
            sample.curvature_per_m = curvature;
            let surface = canonical_plant_surface(field, dir);
            assert_eq!(surface.vegetation, vegetation_cover(&sample, slope));
            let appearance = crate::appearance::surface_appearance(field, &sample, dir);
            assert_eq!(appearance.vegetation, surface.vegetation.total() as f32);
            distinct_cover |= (surface.vegetation.reeds01
                - vegetation_cover(&sample, surface.slope_rise_run).reeds01)
                .abs()
                > 1e-6;
        }
    }
    // Prevent a vacuous fixture where both scales happen to yield full cover.
    assert!(distinct_cover);
}

#[test]
fn attached_hydrology_uses_its_finer_grid_and_excludes_saline_basins() {
    let field = canonical_field();
    assert_eq!(field.hydrology_grid_step_deg(), 0.5);
    // A bundled basin cell, not a fabricated historical plant ID. The coarse
    // context is 2 degrees; using its dimensions silently misses this lake.
    let dir = dir_from_latlon(-81.5, -9.0);
    assert!((field.water_height_m(dir) - 413.0495928638749).abs() < 1e-9);
    let surface = canonical_plant_surface(field, dir);
    assert_eq!(surface.water_height_m, Some(field.water_height_m(dir)));
    assert_eq!(surface.ground_height_m, field.height_m(dir, 32.0));
    assert!(surface.ground_height_m < surface.water_height_m.unwrap());

    let salt = field
        .lake_basins()
        .iter()
        .find(|lake| lake.salt_flat)
        .unwrap();
    let [r, c] = salt.cells[0];
    let step = field.hydrology_grid_step_deg();
    assert_eq!(
        field.water_height_m(dir_from_latlon(
            -90.0 + r as f64 * step,
            -180.0 + c as f64 * step
        )),
        0.0
    );
}

#[test]
fn accepted_unsorted_frozen_lake_cells_match_sorted_control_after_attachment() {
    let mut frozen = FrozenSurface::read_gzip(
        &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
    )
    .unwrap();
    let index = frozen
        .lakes
        .iter()
        .position(|lake| !lake.salt_flat && lake.cells.len() > 1)
        .unwrap();
    let sorted_cells = frozen.lakes[index].cells.clone();
    assert!(sorted_cells.windows(2).all(|pair| pair[0] <= pair[1]));
    frozen.lakes[index].cells.reverse();
    assert_ne!(frozen.lakes[index].cells, sorted_cells);
    // Ordering is not a frozen validation requirement. Do not tighten it or
    // change the historical offline wrapper to make this accepted input work.
    frozen
        .validate()
        .expect("unsorted lake cells remain valid frozen input");
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let source = field_from_manifest(&manifest_from_spec(&recipe).unwrap()).unwrap();
    let attached = source.with_frozen_erosion(&frozen).unwrap();
    for &[r, c] in &sorted_cells {
        let dir = dir_from_latlon(frozen.lats[r], frozen.lons[c]);
        let expected = canonical_field().water_height_m(dir);
        assert!(expected > 0.0);
        assert_eq!(
            attached.water_height_m(dir),
            expected,
            "lake cell [{r}, {c}]"
        );
    }
    assert_eq!(attached.lake_basins()[index].cells, sorted_cells);
    assert_eq!(
        frozen.lakes[index].cells,
        sorted_cells.into_iter().rev().collect::<Vec<_>>()
    );
}

#[test]
fn attached_lake_rejects_otherwise_eligible_plants_at_final_ground_height() {
    let field = canonical_field();
    // Bundled fresh basin 241 around [30 S, 68 W], in a warm enough band
    // that ecology alone does not already suppress every candidate.
    let region = TileKey {
        face: 4,
        level: 14,
        x: 11501,
        y: 3090,
    };
    assert!((field.water_height_m(dir_from_latlon(-30.0, -68.0)) - 613.8868872547769).abs() < 1e-9);
    let dry_control = scatter_region(
        field.params.seed,
        field.params.radius_m,
        region,
        ScatterPopulation::Plants,
        ScatterPolicy::default(),
        |dir| {
            let mut surface = canonical_plant_surface(field, dir);
            surface.water_height_m = None; // Test-only counterfactual.
            surface
        },
    )
    .unwrap();
    let wet = query(region, ScatterPolicy::default());
    let expected: Vec<_> = dry_control
        .items
        .iter()
        .copied()
        .filter(|item| item.ground_height_m > field.water_height_m(item.direction))
        .collect();
    assert!(!dry_control.items.is_empty());
    assert!(expected.len() < dry_control.items.len());
    assert_eq!(wet.items, expected);
}

#[test]
fn attached_field_parent_children_density_and_stored_ground_are_exact() {
    let field = canonical_field();
    let policy = ScatterPolicy::default();
    let parent = query(LAND_WINDOW, policy);
    assert!(!parent.items.is_empty());
    assert_eq!(parent.candidates_visited, 4096);
    let mut children = Vec::new();
    for child in LAND_WINDOW.children().into_iter().rev() {
        children.extend(query(child, policy).items);
    }
    children.sort_unstable_by_key(|item| item.id);
    assert_eq!(parent.items, children);
    assert_eq!(parent, query(LAND_WINDOW, policy));
    for item in &parent.items {
        assert_eq!(item.ground_height_m, field.height_m(item.direction, 32.0));
        assert_eq!(item.id.0 & 1, ScatterPopulation::Plants as u64);
        let local = canonical_plant_surface(field, item.direction);
        assert!(item.ground_height_m > local.water_height_m.unwrap());
        assert!(local.slope_rise_run <= policy.max_plant_slope_rise_run);
    }
    let sparse = query(
        LAND_WINDOW,
        ScatterPolicy {
            density_scale: 0.25,
            ..policy
        },
    );
    assert!(!sparse.items.is_empty() && sparse.items.len() < parent.items.len());
    for item in sparse.items {
        let index = parent
            .items
            .binary_search_by_key(&item.id, |item| item.id)
            .unwrap();
        assert_eq!(item, parent.items[index]);
    }
}

#[test]
fn placement_slope_gate_uses_final_geometry_not_zero_or_material_slope() {
    let permissive = query(LAND_WINDOW, ScatterPolicy::default());
    let permissive_count = permissive.items.len();
    let mut slopes: Vec<_> = permissive
        .items
        .iter()
        .map(|item| canonical_field().surface_geometry(item.direction, 32.0).0)
        .collect();
    slopes.sort_by(f64::total_cmp);
    let policy = ScatterPolicy {
        max_plant_slope_rise_run: slopes[slopes.len() / 2],
        ..Default::default()
    };
    let material_filtered: Vec<_> = permissive
        .items
        .iter()
        .copied()
        .filter(|item| {
            canonical_field()
                .surface_geometry(item.direction, crate::lod::MATERIAL_SLOPE_WAVELENGTH_M)
                .0
                <= policy.max_plant_slope_rise_run
        })
        .collect();
    let expected: Vec<_> = permissive
        .items
        .into_iter()
        .filter(|item| {
            canonical_field().surface_geometry(item.direction, 32.0).0
                <= policy.max_plant_slope_rise_run
        })
        .collect();
    let batch = query(LAND_WINDOW, policy);
    assert!(!expected.is_empty());
    assert!(expected.len() < permissive_count);
    assert_ne!(expected, material_filtered);
    assert_eq!(batch.items, expected);
}

#[test]
#[ignore = "CPU cost diagnostic; run only after concurrent native GPU gates finish"]
fn canonical_plant_batch_cost() {
    use std::{hint::black_box, time::Instant};
    let field = canonical_field(); // Source checking/loading is outside timing.
    let region = LAND_WINDOW.parent().unwrap().parent().unwrap(); // L12
    for density_scale in [0.0, 0.25, 1.0] {
        let policy = ScatterPolicy {
            density_scale,
            ..Default::default()
        };
        let baseline = query(region, policy);
        let mut children: Vec<_> = region
            .children()
            .into_iter()
            .rev()
            .flat_map(|child| query(child, policy).items)
            .collect();
        children.sort_unstable_by_key(|item| item.id);
        assert_eq!(baseline.items, children);
        for item in &baseline.items {
            assert_eq!(item.ground_height_m, field.height_m(item.direction, 32.0));
        }
        let mut times = Vec::new();
        for _ in 0..15 {
            let start = Instant::now();
            black_box(query(black_box(region), black_box(policy)));
            times.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        times.sort_by(f64::total_cmp);
        eprintln!(
            "WORLD03-canonical-plants-L12,{density_scale},{},{},{},{:.6},{:.6}",
            baseline.candidates_visited,
            baseline.surface_samples,
            baseline.items.len(),
            times[7],
            times[14]
        );
    }
}
