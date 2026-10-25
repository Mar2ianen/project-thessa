use crate::selection::*;
use glam::DVec3;
use std::{
    collections::BTreeSet,
    sync::{Arc, OnceLock},
};
use thessa_worldgen_rocky::{
    field::{KnobsSerde, PlanetField, PlanetParams},
    lod::{self, TileKey},
};

pub(crate) fn field() -> Arc<PlanetField> {
    static FIELD: OnceLock<Arc<PlanetField>> = OnceLock::new();
    FIELD
        .get_or_init(|| {
            Arc::new(
                PlanetField::build(
                    PlanetParams {
                        hydrology: Default::default(),
                        surface_climate: Default::default(),
                        name: "stream-test".into(),
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
                .unwrap(),
            )
        })
        .clone()
}

pub(crate) fn view() -> SelectionView {
    SelectionView {
        eye: DVec3::Z * 3_230_000.0,
        stream_origin: DVec3::Z * 3_200_000.0,
        forward: DVec3::NEG_Z,
        radius: 3_200_000.0,
        frustum: None,
        material_clip_from_eye: None,
    }
}

#[test]
fn extracted_selection_matches_existing_lod_calls_and_debounces_orbit() {
    let field = field();
    let mut history = SelectionHistory::default();
    let policy = SelectionPolicy {
        coarse_level: 2,
        coarse_budget: 24,
        fine_level: 4,
        fine_budget: 48,
    };
    let v = view();
    let demand = history
        .update(&field, v, 0.0, &[], true, true, policy)
        .unwrap();
    let coarse = lod::select_tiles_with_height_and_frustum(
        v.eye.to_array(),
        v.radius,
        2,
        24,
        |d| field.height_m(d, 32.0),
        None,
        1.0,
    );
    let fine = lod::select_tiles_with_history(
        v.eye.to_array(),
        v.radius,
        4,
        48,
        |d| field.height_m(d, 32.0),
        None,
        1.0,
        &[],
    );
    assert_eq!(demand.fine, fine);
    assert_eq!(
        demand.wanted,
        complete_terrain_cover(&coarse.into_iter().chain(fine).collect::<Vec<_>>())
    );
    assert!(
        history
            .update(&field, v, 0.7, &demand.fine, false, true, policy)
            .is_none()
    );
    let mut orbit = v;
    orbit.eye += DVec3::X * 1000.0;
    assert!(
        history
            .update(&field, orbit, 0.71, &demand.fine, false, true, policy)
            .is_some()
    );
    assert_eq!(
        history.detail_bias(),
        1.0,
        "camera orbit is not craft speed"
    );
    orbit.eye += DVec3::X * 1000.0;
    assert!(
        history
            .update(&field, orbit, 0.8, &demand.fine, false, true, policy)
            .is_none()
    );
    assert!(
        history
            .update(&field, orbit, 0.9, &demand.fine, false, true, policy)
            .is_some()
    );
    orbit.stream_origin += DVec3::X * 1000.0;
    history.update(&field, orbit, 1.0, &demand.fine, false, true, policy);
    assert!(history.detail_bias() > 1.0);
}

#[test]
fn cover_and_dependencies_preserve_siblings_and_stop_at_resident_parent() {
    let root = TileKey::root(4);
    let fine = TileKey {
        level: 4,
        x: 8,
        y: 8,
        ..root
    };
    let cover = complete_terrain_cover(&[root, fine, fine]);
    assert!(cover.contains(&fine));
    assert_eq!(
        cover
            .iter()
            .map(|k| 4f64.powi(-i32::from(k.level)))
            .sum::<f64>(),
        1.0
    );
    for (i, a) in cover.iter().enumerate() {
        for b in &cover[i + 1..] {
            assert!(!tiles_overlap(*a, *b));
        }
    }
    let parent = fine.parent().unwrap();
    assert_eq!(
        spawn_dependencies(&[fine], true, |k| k == parent),
        BTreeSet::from([fine, parent])
    );
    assert_eq!(
        spawn_dependencies(&[fine], false, |_| false),
        BTreeSet::from([fine])
    );
    assert_eq!(
        material_build_dependency(fine, |k| k == root),
        fine.parent().unwrap().parent().unwrap()
    );
}

#[test]
fn material_refinement_preserves_area_and_has_fixed_budget() {
    let pages = material_view_selection(&[TileKey::root(4)], view(), 1e-6);
    assert!(pages.len() > 1 && pages.len() <= 512);
    assert!(pages.iter().all(|k| k.level <= 17));
    assert_eq!(
        pages
            .iter()
            .map(|k| 4f64.powi(-i32::from(k.level)))
            .sum::<f64>(),
        1.0
    );
}

#[test]
fn close_material_refinement_reaches_configured_tiers_without_growing_budget() {
    let key = TileKey {
        face: 4,
        level: 17,
        x: 65536,
        y: 65536,
    };
    let dir = DVec3::from(key.direction(0.5, 0.5));
    let view = SelectionView {
        eye: dir * (3_200_000.0 + 2.0),
        stream_origin: dir * 3_200_000.0,
        forward: -dir,
        radius: 3_200_000.0,
        frustum: None,
        material_clip_from_eye: None,
    };
    for limit in [17, 20, 21, 22, 23, 24, 255] {
        let pages = material_view_selection_with_limit(&[key], view, 1e-5, limit);
        let budget = match limit.min(24) {
            17..=20 => 512,
            21 => 1024,
            _ => 2048,
        };
        assert!(pages.len() <= budget);
        // The broad, unculled window hits its existing leaf budget at L23.
        assert_eq!(pages.iter().map(|k| k.level).max().unwrap(), limit.min(23));
        let relative_area: f64 = pages
            .iter()
            .map(|k| 4f64.powi(i32::from(key.level) - i32::from(k.level)))
            .sum();
        assert_eq!(relative_area, 1.0);
        assert!(pages.iter().all(|p| p.face == key.face));
    }
    // A focused near-floor window reaches the new optical ceiling under
    // the same budget; broad views must not grow memory to force that level.
    let near = TileKey {
        level: 20,
        x: 1 << 19,
        y: 1 << 19,
        ..key
    };
    let near_dir = DVec3::from(near.direction(0.5, 0.5));
    let near_view = SelectionView {
        eye: near_dir * (view.radius + 2.0),
        stream_origin: near_dir * view.radius,
        forward: -near_dir,
        ..view
    };
    let pages = material_view_selection_with_limit(&[near], near_view, 1e-5, 24);
    assert_eq!(pages.iter().map(|k| k.level).max(), Some(24));
    assert!(pages.len() <= 2048);
    assert_eq!(
        pages
            .iter()
            .map(|k| 4f64.powi(i32::from(near.level) - i32::from(k.level)))
            .sum::<f64>(),
        1.0
    );
    assert_eq!(
        material_view_selection(&[key], view, 1e-5),
        material_view_selection_with_limit(&[key], view, 1e-5, 17)
    );
}

#[test]
fn elevated_material_reference_measures_distance_to_ground_not_the_body_datum() {
    let key = TileKey {
        face: 4,
        level: 17,
        x: 65536,
        y: 65536,
    };
    let dir = DVec3::from(key.direction(0.5, 0.5));
    let radius = 3_200_000.0;
    for elevation in [-1000.0, 0.0, 1000.0] {
        let view = SelectionView {
            eye: dir * (radius + elevation + 2.0),
            stream_origin: dir * (radius + elevation),
            forward: -dir,
            radius,
            frustum: Some(lod::SelectionFrustum {
                forward: (-dir).to_array(),
                cos_limit: 0.0,
            }),
            material_clip_from_eye: None,
        };
        let refined = material_view_selection_at_height(&[key], view, 0.001, 22, elevation);
        assert!(refined.len() <= 2048);
        assert_eq!(refined.iter().map(|k| k.level).max().unwrap(), 22);
        assert_eq!(
            refined
                .iter()
                .map(|k| 4f64.powi(i32::from(key.level) - i32::from(k.level)))
                .sum::<f64>(),
            1.0
        );
        let legacy = material_view_selection_with_limit(&[key], view, 0.001, 22);
        if elevation == 0.0 {
            assert_eq!(legacy, refined);
            assert_eq!(
                material_demand(refined.iter().copied(), view),
                material_demand_at_height(refined.iter().copied(), view, 0.0)
            );
        } else {
            assert_eq!(
                legacy,
                vec![key],
                "datum-distance bug must be exposed by the fixture"
            );
        }
    }
}

#[test]
fn standing_material_priority_admits_visible_floor_before_underfoot_pages() {
    let underfoot = TileKey {
        face: 4,
        level: 22,
        x: 1 << 21,
        y: 1 << 21,
    };
    let visible = TileKey {
        x: underfoot.x + 8,
        ..underfoot
    };
    let up = DVec3::from(underfoot.direction(0.5, 0.5));
    let forward = (DVec3::X - up * 0.025).normalize();
    for elevation in [-1000.0, 0.0, 1000.0] {
        let view = SelectionView {
            eye: up * (3_200_000.0 + elevation + 2.0),
            stream_origin: up * (3_200_000.0 + elevation),
            forward,
            radius: 3_200_000.0,
            frustum: Some(lod::SelectionFrustum {
                forward: forward.to_array(),
                cos_limit: 0.7,
            }),
            material_clip_from_eye: None,
        };
        assert_eq!(
            material_demand_at_height([underfoot, visible], view, elevation)[0],
            visible
        );
        let legacy = SelectionView {
            frustum: None,
            material_clip_from_eye: None,
            ..view
        };
        assert_eq!(
            material_demand_at_height([underfoot, visible], legacy, elevation)[0],
            underfoot
        );
    }
}

#[test]
fn material_frame_rejects_below_screen_prefetch_without_losing_cover() {
    let radius = 3_200_000.0;
    for face in 0..6 {
        let root = TileKey::root(face);
        let up = DVec3::from(root.direction(0.5, 0.5));
        let forward = (DVec3::from(root.direction(0.501, 0.5)) - up)
            .reject_from_normalized(up)
            .normalize();
        for elevation in [-1000.0, 0.0, 1000.0] {
            let hidden = TileKey {
                face,
                level: 24,
                x: 8_388_610,
                y: 8_388_608,
            };
            let visible = TileKey {
                x: 8_388_621,
                ..hidden
            };
            let clip =
                glam::dcamera::rh::proj::directx::perspective(0.9, 16.0 / 9.0, 0.1, 100000.0)
                    * glam::dcamera::rh::view::look_at_mat4(DVec3::ZERO, forward, up);
            let view = SelectionView {
                eye: up * (radius + elevation + 2.0),
                stream_origin: up * (radius + elevation),
                forward,
                radius,
                frustum: Some(lod::SelectionFrustum {
                    forward: forward.to_array(),
                    cos_limit: ((0.45_f64.tan() * 16.0 / 9.0).atan() + 0.6).cos(),
                }),
                material_clip_from_eye: Some(clip),
            };
            let project = |key: TileKey| {
                let p = DVec3::from(key.direction(0.5, 0.5)) * (radius + elevation) - view.eye;
                let p = clip * p.extend(1.0);
                p.truncate() / p.w
            };
            assert!(project(hidden).y < -4.0 && project(visible).y.abs() < 0.9);
            assert_eq!(
                material_demand_at_height([hidden, visible], view, elevation)[0],
                visible
            );
            let legacy = SelectionView {
                material_clip_from_eye: None,
                ..view
            };
            assert_eq!(
                material_demand_at_height([hidden, visible], legacy, elevation)[0],
                hidden
            );
            let base = TileKey {
                face,
                level: 20,
                x: 1 << 19,
                y: 1 << 19,
            };
            let pages = material_view_selection_at_height(&[base], view, 0.0007, 24, elevation);
            assert!(pages.len() <= 2048);
            assert_eq!(
                pages
                    .iter()
                    .map(|key| 4_f64.powi(i32::from(base.level) - i32::from(key.level)))
                    .sum::<f64>(),
                1.0
            );
            assert_eq!(
                pages,
                material_view_selection_at_height(&[base], legacy, 0.0007, 24, elevation),
                "camera-frame priority must preserve the shared soil-height prefetch cover"
            );
        }
    }
}
