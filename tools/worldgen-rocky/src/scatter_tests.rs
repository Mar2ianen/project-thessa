use super::*;

const RADIUS_M: f64 = 3_200_000.0;

#[test]
fn descriptor_capacity_overflow_is_an_error_without_allocating() {
    let mut items = Vec::<u8>::new();
    assert_eq!(
        try_reserve_items(&mut items, usize::MAX),
        Err(ScatterError::AllocationFailed {
            additional: usize::MAX
        })
    );
    assert!(items.is_empty());
    assert_eq!(items.capacity(), 0);
}

fn region(face: u8, level: u8, x: u32, y: u32) -> TileKey {
    TileKey { face, level, x, y }
}

fn surface(dir: [f64; 3]) -> ScatterSurface {
    ScatterSurface {
        ground_height_m: 100.0 + 10.0 * dir[1],
        slope_rise_run: 0.1,
        water_height_m: Some(0.0),
        biome: Biome::EjectaField,
        geology: Geology::Basaltic,
        rocky01: 1.0,
        vegetation: VegetationCover {
            ground01: 0.5,
            canopy01: 0.25,
            reeds01: 0.25,
        },
    }
}

fn query(tile: TileKey, population: ScatterPopulation, scale: f64) -> ScatterBatch {
    scatter_region(
        7,
        RADIUS_M,
        tile,
        population,
        ScatterPolicy {
            density_scale: scale,
            ..Default::default()
        },
        surface,
    )
    .unwrap()
}

#[test]
fn version_one_address_and_transform_fixture() {
    let tile = region(0, 12, 2048, 2048);
    let batch = query(tile, ScatterPopulation::Plants, 1.0);
    let first = batch.items[0];
    assert_eq!(first.id, ScatterId(19_241_453_486_085));
    assert_eq!(first.kind, ScatterKind::FernClump);
    assert_eq!(first.scale.to_bits(), 4_603_645_195_746_004_074);
    assert_eq!(first.yaw_rad.to_bits(), 4_615_060_769_497_609_545);
    let expected_direction = [
        0.9999999999974043,
        2.489983330627086e-7,
        -2.2648326307658282e-6,
    ];
    assert!(
        first
            .direction
            .into_iter()
            .zip(expected_direction)
            .all(|(a, b)| (a - b).abs() < 1e-15)
    );
    assert_eq!(first.id.cell().unwrap().level, CANDIDATE_LEVEL);
    let different_seed = scatter_region(
        8,
        RADIUS_M,
        tile,
        ScatterPopulation::Plants,
        Default::default(),
        surface,
    )
    .unwrap();
    assert_ne!(batch.items, different_seed.items);
    assert!(ScatterId(0).cell().is_none());
    assert!(ScatterId(u64::MAX).cell().is_none());
}

#[test]
fn understory_colonies_are_bounded_continuous_and_world_addressed() {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for seed in [7, 42, 2026] {
        // A dry-land latitude circle: adjacent probes are about 3 cm apart,
        // while the full path crosses multiple eight-metre colonies.
        for i in 0..1024 {
            let direction = crate::lod::normalize([1.0, 0.2, i as f64 * 1e-8]);
            let near = crate::lod::normalize([1.0, 0.2, (i + 1) as f64 * 1e-8]);
            let density = understory_patch_density(seed, RADIUS_M, direction);
            assert_eq!(density, understory_patch_density(seed, RADIUS_M, direction));
            assert!((0.15..=1.85).contains(&density));
            assert!((density - understory_patch_density(seed, RADIUS_M, near)).abs() < 0.02);
            min = min.min(density);
            max = max.max(density);
        }
        // Crossing a cube-face ownership boundary must not change the field.
        let a = crate::lod::normalize([1.0, 1.0 - 1e-10, 0.2]);
        let b = crate::lod::normalize([1.0, 1.0 + 1e-10, 0.2]);
        assert!(
            (understory_patch_density(seed, RADIUS_M, a)
                - understory_patch_density(seed, RADIUS_M, b))
            .abs()
                < 0.001
        );
    }
    assert!(
        max - min > 1.0,
        "colonies must include gaps and dense areas"
    );
}

#[test]
fn understory_has_disjoint_ids_nested_density_and_exact_window_unions() {
    let tile = region(0, 18, 131_072, 131_072);
    let high = query(tile, ScatterPopulation::Understory, 1.0);
    assert!(!high.items.is_empty());
    assert_eq!(high.candidates_visited, 4096);
    assert_eq!(high, query(tile, ScatterPopulation::Understory, 1.0));
    let legacy = query(tile, ScatterPopulation::Plants, 1.0);
    for item in &high.items {
        assert_eq!(item.id.cell().unwrap().level, UNDERSTORY_CANDIDATE_LEVEL);
        assert_eq!(item.id.0 & 1, 1);
        assert!(matches!(
            item.kind,
            ScatterKind::FernClump | ScatterKind::ReedClump
        ));
        assert!(!legacy.items.iter().any(|old| old.id == item.id));
    }
    let mut previous = Vec::new();
    for density in [0.0, 0.25, 0.5, 1.0] {
        let items = query(tile, ScatterPopulation::Understory, density).items;
        assert!(items.iter().all(|item| high.items.contains(item)));
        assert!(previous.iter().all(|item| items.contains(item)));
        previous = items;
    }
    let mut children: Vec<_> = tile
        .children()
        .into_iter()
        .rev()
        .flat_map(|child| query(child, ScatterPopulation::Understory, 1.0).items)
        .collect();
    children.sort_unstable_by_key(|item| item.id);
    assert_eq!(high.items, children);
    // Fine windows still own a candidate once, rather than cloning a parent-cell plant.
    let cell = high.items[0].id.cell().unwrap();
    let parent = query(cell, ScatterPopulation::Understory, 1.0);
    let mut fine: Vec<_> = cell
        .children()
        .into_iter()
        .flat_map(|child| query(child, ScatterPopulation::Understory, 1.0).items)
        .collect();
    fine.sort_unstable_by_key(|item| item.id);
    assert_eq!(parent.items, fine);
}

#[test]
fn understory_uses_ground_or_reed_habitat_and_explicit_spacing_and_budgets() {
    let tile = region(0, 18, 131_072, 131_072);
    let run = |cover: VegetationCover, spacing: f64| {
        scatter_region(
            7,
            RADIUS_M,
            tile,
            ScatterPopulation::Understory,
            ScatterPolicy {
                understory_spacing_m: spacing,
                ..Default::default()
            },
            |dir| {
                let mut s = surface(dir);
                s.vegetation = cover;
                s
            },
        )
    };
    assert!(
        run(
            VegetationCover {
                ground01: 0.0,
                canopy01: 1.0,
                reeds01: 0.0
            },
            1.0
        )
        .unwrap()
        .items
        .is_empty()
    );
    let fern = run(
        VegetationCover {
            ground01: 1.0,
            canopy01: 0.0,
            reeds01: 0.0,
        },
        1.0,
    )
    .unwrap();
    assert!(fern.items.iter().all(|i| i.kind == ScatterKind::FernClump));
    let reeds = run(
        VegetationCover {
            ground01: 0.0,
            canopy01: 0.0,
            reeds01: 1.0,
        },
        1.0,
    )
    .unwrap();
    assert_eq!(fern.items.len(), reeds.items.len());
    assert!(reeds.items.iter().all(|i| i.kind == ScatterKind::ReedClump));
    let sparse = run(
        VegetationCover {
            ground01: 1.0,
            canopy01: 0.0,
            reeds01: 0.0,
        },
        2.0,
    )
    .unwrap();
    assert!(sparse.items.len() < fern.items.len());
    assert!(sparse.items.iter().all(|i| fern.items.contains(i)));
    let empty = VegetationCover {
        ground01: 0.0,
        canopy01: 0.0,
        reeds01: 0.0,
    };
    for spacing in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(run(empty, spacing).is_err());
    }
    assert!(matches!(
        run(empty, 0.001),
        Err(ScatterError::DensityCapacityExceeded { .. })
    ));
    let result = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Understory,
        ScatterPolicy {
            max_candidates: 4095,
            ..Default::default()
        },
        |_| panic!("budget rejection must precede sampling"),
    );
    assert_eq!(
        result,
        Err(ScatterError::CandidateBudgetExceeded {
            required: 4096,
            limit: 4095
        })
    );
    eprintln!(
        "understory full_ground_count={} sparse_2m_count={} candidates={}",
        fern.items.len(),
        sparse.items.len(),
        fern.candidates_visited
    );
}

#[test]
fn parent_and_reversed_children_have_exact_ids_and_transforms() {
    for population in [ScatterPopulation::Rocks, ScatterPopulation::Plants] {
        // Centre, cube edges/corners, and both polar faces; no lat/lon singularity.
        for tile in [
            region(0, 12, 2048, 2048),
            region(0, 12, 0, 0),
            region(1, 12, 4095, 2000),
            region(2, 12, 2047, 2047),
            region(3, 12, 2048, 2048),
            region(4, 12, 4095, 4095),
            region(5, 12, 0, 0),
        ] {
            let parent = query(tile, population, 1.0);
            assert!(!parent.items.is_empty());
            let mut children: Vec<_> = tile
                .children()
                .into_iter()
                .rev()
                .flat_map(|child| query(child, population, 1.0).items)
                .collect();
            children.sort_unstable_by_key(|item| item.id);
            assert_eq!(parent.items, children, "{tile:?} {population:?}");
            assert_eq!(parent, query(tile, population, 1.0));
        }
    }
}

#[test]
fn regression_density_must_keep_existing_transforms() {
    let tile = region(0, 12, 2048, 2048);
    for population in [ScatterPopulation::Rocks, ScatterPopulation::Plants] {
        let high = query(tile, population, 1.0);
        let mut previous = Vec::new();
        for scale in [0.0, 0.125, 0.25, 0.5, 1.0] {
            let items = query(tile, population, scale).items;
            assert!(items.iter().all(|item| {
                high.items
                    .binary_search_by_key(&item.id, |v| v.id)
                    .is_ok_and(|i| high.items[i] == *item)
            }));
            assert!(previous.iter().all(|item| items.contains(item)));
            previous = items;
        }
    }
}

#[test]
fn regression_density_must_not_silently_clip_at_4000() {
    let batch = query(region(0, 11, 1024, 1024), ScatterPopulation::Plants, 1.0);
    assert!(batch.items.len() > 25_000, "got {}", batch.items.len());
    assert_eq!(batch.candidates_visited, 262_144);
}

#[test]
fn candidate_subwindows_through_level_27_own_each_point_once() {
    // Windows finer than the candidate lattice must filter the fixed jitter,
    // not manufacture new candidates. Dense fixture makes this non-vacuous.
    let (cell, parent) = (0..8)
        .find_map(|offset| {
            let cell = region(2, CANDIDATE_LEVEL, 524288 + offset, 524288);
            let batch = query(cell, ScatterPopulation::Plants, 8.0);
            (!batch.items.is_empty()).then_some((cell, batch))
        })
        .unwrap();
    assert_eq!(parent.items.len(), 1);
    let item = parent.items[0];
    let mut owner = cell;
    for _ in CANDIDATE_LEVEL..27 {
        let children = owner.children();
        let batches = children.map(|tile| query(tile, ScatterPopulation::Plants, 8.0));
        assert_eq!(
            batches.iter().map(|batch| batch.items.len()).sum::<usize>(),
            1
        );
        let index = batches
            .iter()
            .position(|batch| !batch.items.is_empty())
            .unwrap();
        assert_eq!(batches[index].items[0], item);
        owner = children[index];
    }
}

#[test]
fn cube_face_seams_and_polar_neighbourhoods_have_unique_interior_owners() {
    let mut ids = std::collections::BTreeSet::new();
    let mut positions = std::collections::BTreeSet::new();
    let mut count = 0;
    for face in 0..6 {
        for (x, y) in [
            (0, 0),
            (16383, 0),
            (0, 16383),
            (16383, 16383),
            (8191, 8191),
            (8192, 8191),
            (8191, 8192),
            (8192, 8192),
        ] {
            for item in query(region(face, 14, x, y), ScatterPopulation::Plants, 1.0).items {
                count += 1;
                assert!(ids.insert(item.id));
                assert!(positions.insert(item.direction.map(f64::to_bits)));
                let d = item.direction.map(f64::abs);
                let major = match face {
                    0 | 1 => 0,
                    2 | 3 => 1,
                    _ => 2,
                };
                assert!((0..3).filter(|i| *i != major).all(|i| d[major] > d[i]));
                assert!((crate::lod::dot(item.direction, item.direction) - 1.0).abs() < 1e-15);
            }
        }
    }
    assert!(count > 5000);
}

#[test]
fn final_local_water_height_ecology_and_slope_filter_candidates_not_tile_centres() {
    let tile = region(0, 12, 2048, 2048);
    let base = query(tile, ScatterPopulation::Plants, 1.0);
    let local = |dir: [f64; 3]| {
        let mut s = surface(dir);
        let u = (-dir[2] / dir[0]) * 4096.0 * 0.5;
        if u < 0.25 {
            s.water_height_m = Some(200.0);
        } else if u < 0.5 {
            s.slope_rise_run = 0.5;
        } else if u < 0.75 {
            s.vegetation = VegetationCover {
                ground01: 0.0,
                canopy01: 0.0,
                reeds01: 0.0,
            };
        }
        s
    };
    let accepted = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Plants,
        Default::default(),
        local,
    )
    .unwrap();
    let expected: Vec<_> = base
        .items
        .iter()
        .copied()
        .filter(|item| {
            let s = local(item.direction);
            s.water_height_m.unwrap() < s.ground_height_m
                && s.slope_rise_run <= 0.25
                && s.vegetation.total() > 0.0
        })
        .collect();
    assert_eq!(accepted.items, expected);
    assert!(accepted.items.len() > 1000 && accepted.items.len() < base.items.len() / 2);
    // Ground height changes do not resample horizontal placement, scale, yaw or
    // identity. Every surviving item's height is the final callback result.
    let raised = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Plants,
        Default::default(),
        |dir| {
            let mut s = surface(dir);
            s.ground_height_m += 37.0;
            s
        },
    )
    .unwrap();
    assert_eq!(base.items.len(), raised.items.len());
    for (a, b) in base.items.iter().zip(&raised.items) {
        assert_eq!(a.id, b.id);
        assert_eq!(a.direction, b.direction);
        assert_eq!(a.kind, b.kind);
        assert_eq!(a.scale, b.scale);
        assert_eq!(a.yaw_rad, b.yaw_rad);
        assert_eq!(
            b.ground_height_m,
            surface(b.direction).ground_height_m + 37.0
        );
    }
}

#[test]
fn rocks_use_local_biome_geology_slope_and_separate_population_ids() {
    let tile = region(0, 12, 2048, 2048);
    let rocks = query(tile, ScatterPopulation::Rocks, 1.0);
    assert!(rocks.items.iter().all(|i| i.id.0 & 1 == 0));
    assert!(
        rocks
            .items
            .iter()
            .any(|i| i.kind == ScatterKind::VolcanicBlock)
    );
    let talus = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Rocks,
        Default::default(),
        |dir| {
            let mut s = surface(dir);
            s.slope_rise_run = 0.8;
            s
        },
    )
    .unwrap();
    assert!(
        talus
            .items
            .iter()
            .any(|i| i.kind == ScatterKind::TalusField)
    );
    let dunes = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Rocks,
        Default::default(),
        |dir| {
            let mut s = surface(dir);
            s.biome = Biome::DuneField;
            s
        },
    )
    .unwrap();
    assert!(dunes.items.len() < rocks.items.len() / 10);
    assert!(
        query(tile, ScatterPopulation::Plants, 1.0)
            .items
            .iter()
            .all(|i| i.id.0 & 1 == 1)
    );
    let submerged = scatter_region(
        7,
        RADIUS_M,
        tile,
        ScatterPopulation::Rocks,
        Default::default(),
        |dir| {
            let mut s = surface(dir);
            s.water_height_m = Some(200.0);
            s
        },
    )
    .unwrap();
    assert!(submerged.items.is_empty());
}

#[test]
fn budgets_and_density_capacity_fail_explicitly_without_partial_results() {
    let tile = region(0, 12, 2048, 2048);
    let run = |policy| {
        scatter_region(
            7,
            RADIUS_M,
            tile,
            ScatterPopulation::Plants,
            policy,
            surface,
        )
    };
    assert_eq!(
        run(ScatterPolicy {
            max_candidates: 65535,
            ..Default::default()
        }),
        Err(ScatterError::CandidateBudgetExceeded {
            required: 65536,
            limit: 65535
        })
    );
    assert_eq!(
        run(ScatterPolicy {
            max_items: 4000,
            ..Default::default()
        }),
        Err(ScatterError::ItemBudgetExceeded { limit: 4000 })
    );
    assert!(matches!(
        run(ScatterPolicy {
            density_scale: 10.0,
            ..Default::default()
        }),
        Err(ScatterError::DensityCapacityExceeded { .. })
    ));
    assert!(matches!(query_disabled_global().items.as_slice(), []));
    assert!(matches!(
        scatter_region(
            7,
            RADIUS_M,
            TileKey::root(0),
            ScatterPopulation::Plants,
            Default::default(),
            surface
        ),
        Err(ScatterError::CandidateBudgetExceeded { .. })
    ));
}

fn query_disabled_global() -> ScatterBatch {
    scatter_region(
        7,
        RADIUS_M,
        TileKey::root(0),
        ScatterPopulation::Plants,
        ScatterPolicy {
            density_scale: 0.0,
            max_candidates: 0,
            max_items: 0,
            ..Default::default()
        },
        |_| panic!("disabled scatter must not sample"),
    )
    .unwrap()
}

#[test]
fn malformed_regions_policies_and_sampled_surfaces_fail_closed() {
    for tile in [
        region(6, 0, 0, 0),
        region(0, 28, 0, 0),
        region(0, 12, 4096, 0),
        region(0, 255, 0, 0),
    ] {
        assert!(
            scatter_region(
                7,
                RADIUS_M,
                tile,
                ScatterPopulation::Plants,
                Default::default(),
                surface
            )
            .is_err()
        );
    }
    let tile = region(0, 12, 2048, 2048);
    for scale in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(
            scatter_region(
                7,
                RADIUS_M,
                tile,
                ScatterPopulation::Plants,
                ScatterPolicy {
                    density_scale: scale,
                    ..Default::default()
                },
                surface
            )
            .is_err()
        );
    }
    for radius in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e200] {
        assert!(
            scatter_region(
                7,
                radius,
                tile,
                ScatterPopulation::Plants,
                Default::default(),
                surface
            )
            .is_err()
        );
    }
    for invalid in 0..5 {
        assert!(matches!(
            scatter_region(
                7,
                RADIUS_M,
                tile,
                ScatterPopulation::Plants,
                Default::default(),
                |dir| {
                    let mut s = surface(dir);
                    match invalid {
                        0 => s.ground_height_m = f64::NAN,
                        1 => s.slope_rise_run = -1.0,
                        2 => s.water_height_m = Some(f64::NAN),
                        3 => s.vegetation.ground01 = -1.0,
                        _ => s.vegetation.canopy01 = 2.0,
                    }
                    s
                }
            ),
            Err(ScatterError::InvalidSurface(_))
        ));
    }
}

fn spherical_area_km2(tile: TileKey) -> f64 {
    let n = (1_u64 << tile.level) as f64;
    let (u0, v0) = (2.0 * tile.x as f64 / n - 1.0, 2.0 * tile.y as f64 / n - 1.0);
    let (u1, v1) = (u0 + 2.0 / n, v0 + 2.0 / n);
    let integral = |u: f64, v: f64| (u * v).atan2((1.0 + u * u + v * v).sqrt());
    (integral(u1, v1) - integral(u0, v1) - integral(u1, v0) + integral(u0, v0)) * RADIUS_M.powi(2)
        / 1e6
}

#[test]
fn physical_density_corrects_cube_jacobian_without_equal_tile_area_assumption() {
    for tile in [
        region(0, 12, 2048, 2048),
        region(0, 12, 0, 0),
        region(2, 12, 4095, 2048),
    ] {
        let expected = spherical_area_km2(tile) * PLANT_DENSITY_PER_KM2;
        let mut total = 0usize;
        for seed in 0..16 {
            total += scatter_region(
                seed,
                RADIUS_M,
                tile,
                ScatterPopulation::Plants,
                Default::default(),
                surface,
            )
            .unwrap()
            .items
            .len();
        }
        let mean = total as f64 / 16.0;
        // Bernoulli/stratified population, not an exact-count or Poisson oracle.
        let envelope = 6.0 * (expected / 16.0).sqrt() + 1.0;
        assert!(
            (mean - expected).abs() <= envelope,
            "{tile:?}: expected {expected}, mean {mean}, bound {envelope}"
        );
        eprintln!(
            "density {tile:?}: area_km2={:.9} expected={expected:.3} mean={mean:.3} six_sigma={envelope:.3}",
            spherical_area_km2(tile)
        );
    }
}
