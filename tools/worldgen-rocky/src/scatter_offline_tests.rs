//! Regressions against the actual offline consumer, not an alternate placement API.
use super::*;

fn fixture() -> FrozenSurface {
    let world = FrozenSurface {
        schema: 2,
        seed: 7,
        radius_m: 3_200_000.0,
        recipe: OfflineRecipe {
            grid_step_deg: 10.0,
            vegetation_tiles: vec![TileKey {
                face: 0,
                level: 12,
                x: 2048,
                y: 2048,
            }],
            ..Default::default()
        },
        lats: (0..19).map(|r| -90.0 + r as f64 * 10.0).collect(),
        lons: (0..36).map(|c| -180.0 + c as f64 * 10.0).collect(),
        heights_m: vec![vec![100.0; 36]; 19],
        source: None,
        material_rgba: vec![vec![[128; 4]; 36]; 19],
        ecological_cover: vec![
            vec![
                crate::ecology::VegetationCover {
                    ground01: 0.5,
                    canopy01: 0.25,
                    reeds01: 0.25,
                };
                36
            ];
            19
        ],
        rivers: Vec::new(),
        lakes: Vec::new(),
        shore_segments: Vec::new(),
        vegetation: Vec::new(),
        prototypes: plant_prototypes(),
        tiles: Vec::new(),
        sediment_volume_error_m3: 0.0,
        regional_erosion: Vec::new(),
    };
    world.validate().unwrap();
    world
}

#[test]
fn offline_union_lod_density_and_final_height_keep_world_addresses() {
    let mut world = fixture();
    let tile = world.recipe.vegetation_tiles[0];
    let parent = bake_plants(&world).unwrap();
    assert!(parent.len() > 4000);
    assert!(parent.windows(2).all(|p| p[0].id < p[1].id));
    for kind in [
        crate::scatter::ScatterKind::FernClump,
        crate::scatter::ScatterKind::ReedClump,
        crate::scatter::ScatterKind::LycopsidTree,
    ] {
        assert!(parent.iter().any(|plant| plant.kind == kind));
    }
    world.recipe.vegetation_tiles = tile.children().into_iter().rev().collect();
    assert_eq!(parent, bake_plants(&world).unwrap());
    world.recipe.vegetation_tiles.extend([tile, tile]);
    // Overlaps/duplicates are eliminated before spending the union budget.
    world.recipe.scatter.max_candidates = 65536;
    assert_eq!(parent, bake_plants(&world).unwrap());
    world.recipe.scatter.density_scale = 0.25;
    let low = bake_plants(&world).unwrap();
    assert!(!low.is_empty());
    assert!(low.iter().all(|plant| parent.contains(plant)));
    world.recipe.scatter.density_scale = 1.0;
    world
        .heights_m
        .iter_mut()
        .flatten()
        .for_each(|h| *h += 25.0);
    let raised = bake_plants(&world).unwrap();
    assert_eq!(parent.len(), raised.len());
    for (mut before, after) in parent.into_iter().zip(&raised) {
        before.ground_height_m = world.height_m(after.direction);
        assert!((before.ground_height_m - 125.0).abs() < 1e-12);
        assert_eq!(before, *after);
        assert_eq!(after.ground_height_m, world.height_m(after.direction));
    }
    world.vegetation = raised;
    world.validate().unwrap();
    let restored: FrozenSurface =
        serde_json::from_slice(&serde_json::to_vec(&world).unwrap()).unwrap();
    assert_eq!(restored.vegetation, world.vegetation);
    // Historical instances stay readable without inventing stable IDs for them.
    let mut old = serde_json::to_value(&world.vegetation[0]).unwrap();
    old.as_object_mut().unwrap().remove("id");
    assert!(
        serde_json::from_value::<PlantInstance>(old)
            .unwrap()
            .id
            .is_none()
    );
    world.vegetation.swap(0, 1);
    assert!(world.validate().is_err());
}

#[test]
fn offline_water_ecology_slope_and_aggregate_exhaustion_are_explicit() {
    let mut world = fixture();
    world.recipe.scatter.max_items = 4000;
    assert!(
        bake_plants(&world)
            .unwrap_err()
            .contains("ItemBudgetExceeded")
    );
    world.recipe.scatter.max_items = 100_000;
    world.recipe.scatter.max_candidates = 65536;
    let tile = world.recipe.vegetation_tiles[0];
    world.recipe.vegetation_tiles.push(TileKey {
        x: tile.x + 1,
        ..tile
    });
    assert!(
        bake_plants(&world)
            .unwrap_err()
            .contains("CandidateBudgetExceeded")
    );
    world.recipe.vegetation_tiles.pop();
    // A lake occupies the actual candidate cell, while the parent/analytic
    // source remains dry. The coarse basin descriptor is an explicit limitation.
    world.lakes.push(crate::hydro::LakeBasin {
        id: 0,
        max_depth_m: 100.0,
        surface_height_m: 200.0,
        area_m2: 1.0,
        capacity_m3: 1.0,
        cells: vec![[9, 18]],
        salt_flat: false,
        aridity01: 0.0,
        inflow_m3_yr: None,
        evaporation_m3_yr: None,
    });
    assert!(bake_plants(&world).unwrap().is_empty());
    world.lakes[0].salt_flat = true;
    assert!(!bake_plants(&world).unwrap().is_empty());
    world.lakes.clear();
    world
        .heights_m
        .iter_mut()
        .flatten()
        .for_each(|h| *h = -10.0);
    assert!(bake_plants(&world).unwrap().is_empty());
    world
        .heights_m
        .iter_mut()
        .flatten()
        .for_each(|h| *h = 100.0);
    world
        .ecological_cover
        .iter_mut()
        .flatten()
        .for_each(|cover| {
            *cover = crate::ecology::VegetationCover {
                ground01: 0.0,
                canopy01: 0.0,
                reeds01: 0.0,
            };
        });
    assert!(bake_plants(&world).unwrap().is_empty());
    world
        .ecological_cover
        .iter_mut()
        .flatten()
        .for_each(|cover| cover.ground01 = 1.0);
    // A known angular ramp on the *final* DEM has exactly 0.5 rise/run at
    // this equatorial fixture. No stale source-height stencil may accept it.
    for (r, row) in world.heights_m.iter_mut().enumerate() {
        row.fill(100.0 + world.lats[r].to_radians() * world.radius_m * 0.5);
    }
    assert!((frozen_slope(&world, tile.direction(0.5, 0.5)) - 0.5).abs() < 1e-6);
    assert!(bake_plants(&world).unwrap().is_empty());
}

#[test]
fn offline_mixed_cover_roundoff_and_tolerated_overshoot_preserve_identity() {
    use crate::ecology::VegetationCover;
    let mut world = fixture();
    let tile = world.recipe.vegetation_tiles[0];
    for cover in [
        VegetationCover {
            ground01: 0.1,
            canopy01: 0.2,
            reeds01: 0.7,
        },
        VegetationCover {
            ground01: 0.1,
            canopy01: 0.2,
            reeds01: (1.0 + 1e-12) - (0.1 + 0.2),
        },
        VegetationCover {
            ground01: 1.0 + 1e-12,
            canopy01: 0.0,
            reeds01: 0.0,
        },
    ] {
        world
            .ecological_cover
            .iter_mut()
            .flatten()
            .for_each(|c| *c = cover);
        world.recipe.vegetation_tiles = vec![tile];
        world.recipe.scatter.density_scale = 1.0;
        world.validate().unwrap();
        let parent = bake_plants(&world).unwrap();
        assert!(parent.len() > 4000);
        world.recipe.vegetation_tiles = tile.children().into_iter().rev().collect();
        assert_eq!(parent, bake_plants(&world).unwrap());
        world.recipe.scatter.density_scale = 0.25;
        let low = bake_plants(&world).unwrap();
        assert!(!low.is_empty());
        assert!(low.iter().all(|plant| parent.contains(plant)));
    }
    world.ecological_cover[9][18].ground01 = 1.0 + 2e-12;
    assert!(world.validate().is_err());
}

#[test]
fn bundled_historical_v3_surface_loads_without_fabricating_instance_ids() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../data/worldgen/thessa-erosion-v3.surface.json.gz");
    let world = FrozenSurface::load(&path).unwrap();
    assert_eq!(world.schema, 4);
    assert!(!world.vegetation.is_empty());
    assert!(world.vegetation.iter().all(|plant| plant.id.is_none()));
    world.validate().unwrap();
    eprintln!(
        "historical bundled v3: schema={} retained plants={}",
        world.schema,
        world.vegetation.len()
    );
}
