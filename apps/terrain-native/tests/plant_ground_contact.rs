// SPDX-License-Identifier: GPL-3.0-or-later
//! Fixed-window diagnostic, not a viewer residency or foliage-rendering test.
//! See docs/artifacts/plant-ground-contact-oracle-notes.md for the boundary.

use glam::DVec3;
use thessa_flight_authority::canonical_world_field;
use thessa_rcbt_core::{
    shaders,
    tile_anchor::{TileAnchor, TileKey as AnchorKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer, VERTICES_PER_PATCH},
    view::ViewParams,
};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
    scatter::{
        PLANT_SURFACE_WAVELENGTH_M, ScatterBatch, ScatterKind, ScatterPolicy, ScatterPopulation,
        canonical_plant_surface, scatter_region,
    },
};

const WINDOW: TileKey = TileKey {
    face: 1,
    level: 14,
    x: 6018,
    y: 11888,
};
const GRID: usize = 33;
const QUANTIZATION_BUDGET_M: f64 = 0.5;

fn fixture_anchor(radius_m: f64) -> TileAnchor {
    TileAnchor::new(
        AnchorKey::new(WINDOW.face, WINDOW.level, WINDOW.x, WINDOW.y).unwrap(),
        radius_m,
    )
    .unwrap()
}

fn plants(field: &PlanetField) -> ScatterBatch {
    let batch = scatter_region(
        field.params.seed,
        field.params.radius_m,
        WINDOW,
        ScatterPopulation::Plants,
        ScatterPolicy::default(),
        |dir| canonical_plant_surface(field, dir),
    )
    .unwrap();
    assert_eq!(batch.candidates_visited, 4096);
    assert!(
        !batch.items.is_empty(),
        "fixture must exercise actual plants"
    );
    for item in &batch.items {
        assert!(DVec3::from_array(item.direction).is_finite());
        assert!((DVec3::from_array(item.direction).length_squared() - 1.0).abs() < 1e-15);
        assert_eq!(
            item.ground_height_m,
            field.height_m(item.direction, PLANT_SURFACE_WAVELENGTH_M),
            "descriptor height must remain the uncorrected canonical height"
        );
        assert!(item.ground_height_m.is_finite());
    }
    batch
}

// CBT_GEOMETRY_WGSL writes row-major 33x33 vertices, each two vec4<f32>:
// tile-local xyz / 1, body-fixed normal xyz / height. "Local" is an offset
// in BODY XYZ, not ENU components: precision.wgsl already uses the face axes
// and radial normal. tile_frame.wgsl adds the anchor without a basis rotation.
fn body_vertex(anchor: &TileAnchor, vertex: &[f32; 8]) -> DVec3 {
    DVec3::from_array(anchor.anchor_body_m)
        + DVec3::new(vertex[0].into(), vertex[1].into(), vertex[2].into())
}

// Exact step=1 interior indices from CBT_CLASSIFY_WGSL::classify_active.
// No skirt indices: the candidates lie strictly inside their canonical tile.
fn surface_triangles(vertices: &[DVec3]) -> Vec<[DVec3; 3]> {
    assert_eq!(vertices.len(), GRID * GRID);
    let mut triangles = Vec::with_capacity(2 * (GRID - 1).pow(2));
    for y in 0..GRID - 1 {
        for x in 0..GRID - 1 {
            let a = y * GRID + x;
            let b = a + 1;
            let c = a + GRID;
            let d = c + 1;
            triangles.push([vertices[a], vertices[b], vertices[c]]);
            triangles.push([vertices[b], vertices[d], vertices[c]]);
        }
    }
    triangles
}

// Independent f64 ray/plane intersection followed by triangle barycentrics.
// Ray starts at the body centre; a unit direction makes t radial metres.
// This does not sample/interpolate the CPU height page at a plant location.
fn radial_hit(direction: DVec3, [a, b, c]: [DVec3; 3]) -> Option<f64> {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    let area_squared = normal.length_squared();
    let denominator = normal.dot(direction);
    if !area_squared.is_finite() || area_squared == 0.0 || denominator == 0.0 {
        return None;
    }
    let t = normal.dot(a) / denominator;
    if !t.is_finite() || t <= 0.0 {
        return None;
    }
    let ap = direction * t - a;
    let u = ap.cross(ac).dot(normal) / area_squared;
    let v = ab.cross(ap).dot(normal) / area_squared;
    // Roundoff allowance only, not a root-contact envelope. Planet-sized t
    // minus a loses O(epsilon * radius) metres, divided by triangle edge scale.
    let tolerance = 64.0 * f64::EPSILON * t.max(1.0) / ab.length().max(ac.length());
    (u.is_finite()
        && v.is_finite()
        && u >= -tolerance
        && v >= -tolerance
        && u + v <= 1.0 + tolerance)
        .then_some(t)
}

fn report(label: &str, errors_m: &mut [f64]) {
    assert!(!errors_m.is_empty());
    assert!(errors_m.iter().all(|x| x.is_finite()));
    errors_m.sort_by(f64::total_cmp);
    // Nearest-rank quantiles, including the median (no averaging).
    let percentile = |percent: usize| errors_m[(percent * errors_m.len()).div_ceil(100) - 1];
    let below = errors_m.iter().filter(|&&x| x < 0.0).count();
    let above = errors_m.iter().filter(|&&x| x > 0.0).count();
    let max_abs = errors_m.iter().copied().map(f64::abs).fold(0.0, f64::max);
    eprintln!(
        "{label}: count={} min_m={:.9} p50_m={:.9} p95_m={:.9} max_m={:.9} max_abs_m={:.9} below_ground={} above_ground={} exactly_zero={}",
        errors_m.len(),
        errors_m[0],
        percentile(50),
        percentile(95),
        errors_m[errors_m.len() - 1],
        max_abs,
        below,
        above,
        errors_m.len() - below - above,
    );
}

#[test]
fn canonical_fixture_has_nonempty_uncorrected_plant_descriptors() {
    let field = canonical_world_field().expect("source-checked bundled canonical field");
    let _ = plants(&field);
}

#[test]
fn body_xyz_offsets_use_the_exact_anchor_without_an_enu_rotation() {
    for face in 0..6 {
        let key = TileKey { face, ..WINDOW };
        let anchor = TileAnchor::new(
            AnchorKey::new(face, key.level, key.x, key.y).unwrap(),
            3_200_000.0,
        )
        .unwrap();
        for uv in [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]] {
            let expected = DVec3::from_array(key.direction(uv[0], uv[1])) * 3_200_123.0;
            let projected = DVec3::from_array(anchor.project_body_m(uv, 123.0));
            assert!(projected.distance(expected) < 2e-9);
            let local = projected - DVec3::from_array(anchor.anchor_body_m);
            let vertex = [
                local.x as f32,
                local.y as f32,
                local.z as f32,
                1.0,
                0.0,
                1.0,
                0.0,
                123.0,
            ];
            let tolerance = 2.0 * f64::from(f32::EPSILON) * local.length();
            assert!(body_vertex(&anchor, &vertex).distance(expected) < tolerance);
        }
    }
}

#[test]
fn radial_plane_triangle_known_cases_and_signed_clearance() {
    let plane = [
        DVec3::new(-2.0, -2.0, 10.0),
        DVec3::new(2.0, -2.0, 10.0),
        DVec3::new(0.0, 2.0, 10.0),
    ];
    assert_eq!(radial_hit(DVec3::Z, plane), Some(10.0));
    assert_eq!(
        radial_hit(DVec3::Z, [plane[2], plane[1], plane[0]]),
        Some(10.0)
    );
    assert_eq!(radial_hit(DVec3::X, plane), None); // Parallel.
    assert_eq!(radial_hit(-DVec3::Z, plane), None); // Behind ray origin.
    assert_eq!(
        radial_hit(DVec3::new(1.0, 0.0, 1.0).normalize(), plane),
        None
    );
    assert_eq!(radial_hit(DVec3::Z, [plane[0]; 3]), None); // Degenerate.
    let direction = plane[0].normalize(); // Boundary vertex and oblique ray.
    assert!((radial_hit(direction, plane).unwrap() - plane[0].length()).abs() < 1e-13);
    let terrain_radius = radial_hit(DVec3::Z, plane).unwrap();
    assert_eq!((8.0 + 1.5) - terrain_radius, -0.5);
    assert_eq!((8.0 + 2.5) - terrain_radius, 0.5);
}

#[test]
fn regular_cell_uses_bc_diagonal_not_bilinear_or_ad_diagonal() {
    let mut vertices = vec![DVec3::ZERO; GRID * GRID];
    vertices[0] = DVec3::new(-1.0, -1.0, 10.0); // a
    vertices[1] = DVec3::new(1.0, -1.0, 12.0); // b
    vertices[GRID] = DVec3::new(-1.0, 1.0, 12.0); // c
    vertices[GRID + 1] = DVec3::new(1.0, 1.0, 10.0); // d
    let triangles = surface_triangles(&vertices);
    assert_eq!(triangles.len(), 2048);
    assert_eq!(triangles[0], [vertices[0], vertices[1], vertices[GRID]]);
    assert_eq!(
        triangles[1],
        [vertices[1], vertices[GRID + 1], vertices[GRID]]
    );
    // Shared diagonal is at z=12; bilinear centre would be 11 and a-d 10.
    assert_eq!(radial_hit(DVec3::Z, triangles[0]), Some(12.0));
    assert_eq!(radial_hit(DVec3::Z, triangles[1]), Some(12.0));
    // Planet-scale translated tilted plane: height interpolation in cube UV
    // is not the radial ray/triangle-plane intersection.
    let r = 3_200_000.0;
    let tilted = [
        DVec3::new(-2.0, -2.0, r + 9.0),
        DVec3::new(2.0, -2.0, r + 11.0),
        DVec3::new(0.0, 2.0, r + 10.0),
    ];
    assert_eq!(radial_hit(DVec3::Z, tilted), Some(r + 10.0));
}

#[test]
#[ignore = "requires a wgpu adapter"]
fn fixed_l14_gpu_patch_plant_root_contact() {
    let field = canonical_world_field().expect("source-checked bundled canonical field");
    let batch = plants(&field);
    let radius = field.params.radius_m;
    let anchor = fixture_anchor(radius);
    assert_eq!(GRID, shaders::GRID_SIZE);
    assert_eq!(VERTICES_PER_PATCH as usize, GRID * GRID);
    // Same producer and arguments as terrain-stream::height::bake_page.
    let page = lod::bake_height_page(&field, WINDOW, GRID as u32, QUANTIZATION_BUDGET_M).unwrap();
    let wavelength = (WINDOW.span_m(radius) / (GRID - 1) as f64).max(32.0);
    assert_eq!(wavelength, PLANT_SURFACE_WAVELENGTH_M);
    let backend = Backend::bring_up().expect("hardware diagnostic requires an adapter");
    eprintln!("adapter={:?}", backend.adapter.get_info());
    // Material scale is telemetry only: lod's 256 m constant is crate-private;
    // the canonical adapter, not this test, supplies it to the actual sampler.
    eprintln!(
        "fixed_regular_patch={WINDOW:?} seed={} radius_m={radius} grid={GRID}x{GRID} quantization_budget_m={QUANTIZATION_BUDGET_M} page_base_m={} residual_scale_m={} stored_max_error_m={} source_min_m={} source_max_m={} height_wavelength_m={wavelength} plant_wavelength_m={PLANT_SURFACE_WAVELENGTH_M} material_slope_wavelength_m=256 density_scale=1 candidates={} surface_samples={}",
        field.params.seed,
        page.base_height_m(),
        page.residual_scale_m(),
        page.max_residual_error_m(),
        page.min_height_m(),
        page.max_height_m(),
        batch.candidates_visited,
        batch.surface_samples,
    );
    let mut terrain = TerrainRenderer::new(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    assert!(terrain.conforming_surface().is_none());
    let normal = DVec3::from_array(anchor.normal);
    let center = DVec3::from_array(anchor.anchor_body_m) + normal * f64::from(page.base_height_m());
    let eye = center + normal * 600.0;
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            eye,
            center,
            DVec3::from_array(anchor.raw_axis_v),
        )
        .inverse(),
        world_position_m: eye,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 10_000.0),
        exposure: 1.0,
    };
    terrain.update_view(&backend, &view).unwrap();
    let target = OffscreenTarget::new(&backend, 64, 64).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    // Classification/render must have run before readback. This count uniquely
    // identifies step=1 for one patch (2048 interior + 256 skirt triangles).
    let draw = terrain.read_draw(&backend).unwrap();
    assert_eq!(draw[0], shaders::TRIANGLES_PER_PATCH as u32 * 3);
    eprintln!(
        "regular_step=1 drawn_triangles={} oracle_interior_triangles=2048 skirts_excluded=256",
        draw[0] / 3
    );
    let readback = terrain.read_vertices(&backend).unwrap();
    assert_eq!(readback.len(), GRID * GRID * 8);
    let mut vertices = Vec::with_capacity(GRID * GRID);
    let mut max_frame_error_m = 0.0_f64;
    let mut max_gpu_height_error_m = 0.0_f64;
    for (i, vertex) in readback.as_chunks::<8>().0.iter().enumerate() {
        assert!(vertex.iter().all(|x| x.is_finite()));
        assert_eq!(vertex[3], 1.0);
        let gpu_normal = DVec3::new(vertex[4].into(), vertex[5].into(), vertex[6].into());
        assert!((gpu_normal.length_squared() - 1.0).abs() < 1e-5);
        let position = body_vertex(&anchor, vertex);
        let direction = DVec3::from_array(WINDOW.direction(
            (i % GRID) as f64 / (GRID - 1) as f64,
            (i / GRID) as f64 / (GRID - 1) as f64,
        ));
        let height = f64::from(vertex[7]);
        let source_height = field.height_m(direction.to_array(), wavelength).max(0.0);
        let height_error = (height - source_height).abs();
        let decode_allowance = 4.0
            * f64::from(f32::EPSILON)
            * (f64::from(page.base_height_m()).abs() + height.abs() + 1.0);
        assert!(height_error <= QUANTIZATION_BUDGET_M + decode_allowance);
        max_gpu_height_error_m = max_gpu_height_error_m.max(height_error);
        let expected = direction * (radius + height);
        let error = position.distance(expected);
        // Frame/ABI sanity only: a conservative f32 local-arithmetic allowance
        // scaled to the fixture's raw span and height, NOT a contact envelope.
        let frame_allowance =
            128.0 * f64::from(f32::EPSILON) * (radius * anchor.half_extent + height.abs() + 1.0);
        assert!(
            error <= frame_allowance,
            "vertex {i}: frame error {error} > {frame_allowance}"
        );
        max_frame_error_m = max_frame_error_m.max(error);
        vertices.push(position);
    }
    eprintln!(
        "vertex_frame_max_error_m={max_frame_error_m:.9} gpu_height_vs_source_max_error_m={max_gpu_height_error_m:.9}"
    );
    let triangles = surface_triangles(&vertices);
    let kinds = [
        ScatterKind::FernClump,
        ScatterKind::ReedClump,
        ScatterKind::LycopsidTree,
    ];
    let mut by_kind: [Vec<f64>; 3] = Default::default();
    let mut errors = Vec::with_capacity(batch.items.len());
    for item in &batch.items {
        let direction = DVec3::from_array(item.direction);
        let terrain_radius = triangles
            .iter()
            .filter_map(|&t| radial_hit(direction, t))
            .min_by(f64::total_cmp)
            .unwrap_or_else(|| panic!("root {:?} has no generated-patch triangle hit", item.id));
        // No height offset, prototype minimum-Y adjustment, or terrain snap.
        let root = direction * (radius + item.ground_height_m);
        let signed_m = root.dot(direction) - terrain_radius;
        assert!(signed_m.is_finite());
        errors.push(signed_m);
        let kind_index = kinds
            .iter()
            .position(|&kind| kind == item.kind)
            .expect("plant kind");
        by_kind[kind_index].push(signed_m);
    }
    report("radial_root_minus_generated_terrain", &mut errors);
    for (kind, mut values) in kinds.into_iter().zip(by_kind) {
        eprintln!("kind={kind:?} count={}", values.len());
        if !values.is_empty() {
            report(&format!("kind={kind:?}"), &mut values);
        }
    }
    // Deliberately no unmeasured contact-accuracy acceptance threshold.
}
