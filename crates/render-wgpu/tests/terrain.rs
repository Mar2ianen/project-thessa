use glam::{DVec3, Vec3};
use thessa_rcbt_core::{
    HeightPage,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer, VERTICES_PER_PATCH},
    view::ViewParams,
};

fn backend() -> Option<Backend> {
    match Backend::bring_up() {
        Ok(b) => Some(b),
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU");
            None
        }
        Err(e) => panic!("{e}"),
    }
}

fn anchor(face: u8, level: u8, x: u32, y: u32) -> TileAnchor {
    TileAnchor::new(TileKey::new(face, level, x, y).unwrap(), 6_371_000.0).unwrap()
}

#[test]
fn quantized_step_has_no_false_slope_two_cells_before_cliff() {
    let Some(backend) = backend() else { return };
    let samples: Vec<_> = (0..33 * 33)
        .map(|i| if i % 33 >= 16 { 100.0 } else { 0.0 })
        .collect();
    let page = HeightPage::bake(&samples, 33, 0.01).unwrap();
    let tiles = (0..6)
        .map(|face| anchor(face, 10, 512, 512))
        .collect::<Vec<_>>();
    let patches = tiles
        .iter()
        .map(|a| TerrainPatch {
            anchor: *a,
            page: &page,
        })
        .collect::<Vec<_>>();
    let renderer = TerrainRenderer::new(&backend, &patches, OffscreenTarget::FORMAT).unwrap();
    let output = renderer.read_vertices(&backend).unwrap();
    for (ordinal, a) in tiles.iter().enumerate() {
        for y in 0..33 {
            for x in [0, 1, 14, 18, 31, 32] {
                let i = (ordinal * VERTICES_PER_PATCH as usize + y * 33 + x) * 8;
                let normal = Vec3::from_slice(&output[i + 4..i + 7]);
                assert!(normal.is_finite());
                assert!((normal.length() - 1.0).abs() < 1e-5);
                let body = DVec3::from(a.project_body_m([x as f64 / 32.0, y as f64 / 32.0], 0.0));
                let expected = body.normalize().as_vec3();
                assert!(
                    (normal - expected).length() < 1e-5,
                    "spurious ringing face={ordinal}, ({x},{y}), normal={normal}, radial={expected}"
                );
                assert!(
                    (output[i + 7] - page.sample(x as f32 / 32.0, y as f32 / 32.0)).abs() < 1e-4
                );
            }
        }
    }
}

#[test]
fn planar_quantized_pages_match_normals_across_parent_child_lod_seam() {
    let Some(backend) = backend() else { return };
    let parent = anchor(4, 10, 512, 512);
    let child = anchor(4, 11, 1026, 1024); // adjacent to parent's east edge
    let make_page = |a: TileAnchor| {
        let k = a.tile;
        let side = (1u64 << k.level) as f64;
        let samples = (0..33 * 33)
            .map(|i| {
                let u = (k.x as f64 + (i % 33) as f64 / 32.0) / side;
                let v = (k.y as f64 + (i / 33) as f64 / 32.0) / side;
                // ~500 m / 1000 m relief per parent tile: a real slope,
                // not a nearly radial normal that passes by accident.
                500.0 * (u - 0.5) * 1024.0 + 1_000.0 * (v - 0.5) * 1024.0
            })
            .collect::<Vec<_>>();
        HeightPage::bake(&samples, 33, 0.05).unwrap()
    };
    let pages = [make_page(parent), make_page(child)];
    let renderer = TerrainRenderer::new(
        &backend,
        &[
            TerrainPatch {
                anchor: parent,
                page: &pages[0],
            },
            TerrainPatch {
                anchor: child,
                page: &pages[1],
            },
        ],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    let output = renderer.read_vertices(&backend).unwrap();
    for y in 0..=16 {
        let p = (y * 33 + 32) * 8;
        let c = (VERTICES_PER_PATCH as usize + y * 2 * 33) * 8;
        let pn = Vec3::from_slice(&output[p + 4..p + 7]);
        let cn = Vec3::from_slice(&output[c + 4..c + 7]);
        assert!(
            (pn - cn).length() < 1e-4,
            "LOD normal seam at {y}: {pn} vs {cn}"
        );
        assert!((output[p + 7] - output[c + 7]).abs() < 0.05);
    }
}

#[test]
fn owned_geometry_matches_cpu_height_and_raster_is_not_clear() {
    let Some(backend) = backend() else { return };
    let a = anchor(4, 10, 512, 512);
    let page = HeightPage::bake(&[120.0, 123.0, 121.0, 125.0], 2, 0.001).unwrap();
    let renderer = TerrainRenderer::new(
        &backend,
        &[TerrainPatch {
            anchor: a,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    let output = renderer.read_vertices(&backend).unwrap();
    for y in 0..33 {
        for x in 0..33 {
            let i = (y * 33 + x) * 8;
            let h = page.sample(x as f32 / 32.0, y as f32 / 32.0);
            let expected =
                DVec3::from(a.project_body_m([x as f64 / 32.0, y as f64 / 32.0], h as f64))
                    - DVec3::from(a.anchor_body_m);
            let position = Vec3::from_slice(&output[i..i + 3]).as_dvec3();
            assert!((position - expected).length() < 0.003);
        }
    }
    let center = DVec3::from(a.anchor_body_m);
    let eye = center + DVec3::from(a.normal) * 20_000.0;
    let params = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y).inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e6),
        world_position_m: eye,
        exposure: 1.0,
    };
    renderer.update_view(&backend, &params).unwrap();
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let render = || {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    let pixels = render();
    let background = &pixels[..4];
    let changed = pixels
        .as_chunks::<4>()
        .0
        .iter()
        .filter(|p| p.as_slice() != background)
        .count();
    assert!(changed > 10_000, "empty/missing raster: {changed} pixels");
    let draw = renderer.read_draw(&backend).unwrap();
    assert_eq!(draw[0], 2304 * 3);
    assert_eq!(draw[1..4], [1, 0, 0]);
    assert_eq!(render(), pixels, "static frame must be pixel deterministic");
    // Looking away must zero the GPU-generated indirect draw.
    let mut away = params;
    away.world_from_view =
        glam::dcamera::rh::view::look_to_mat4(eye, (eye - center).normalize(), DVec3::Y).inverse();
    renderer.update_view(&backend, &away).unwrap();
    render();
    assert_eq!(renderer.read_draw(&backend).unwrap()[0], 0);
}

#[test]
fn owned_dirty_page_updates_preserve_clean_geometry_and_validate_atomically() {
    let Some(backend) = backend() else { return };
    let page = HeightPage::bake(&[100.0; 4], 2, 0.001).unwrap();
    let lifted = HeightPage::bake(&[110.0; 4], 2, 0.001).unwrap();
    let too_big = HeightPage::bake(&[120.0; 9], 3, 0.001).unwrap();
    let patches = (0..3)
        .map(|face| TerrainPatch {
            anchor: anchor(face, 0, 0, 0),
            page: &page,
        })
        .collect::<Vec<_>>();
    let renderer = TerrainRenderer::new(&backend, &patches, OffscreenTarget::FORMAT).unwrap();
    let before = renderer.read_vertices(&backend).unwrap();
    assert!(
        renderer
            .update_pages(&backend, &[(0, &lifted), (3, &lifted)])
            .is_err()
    );
    assert!(
        renderer
            .update_pages(&backend, &[(1, &lifted), (1, &page)])
            .is_err()
    );
    assert!(
        renderer
            .update_pages(&backend, &[(0, &lifted), (1, &too_big)])
            .is_err()
    );
    assert_eq!(renderer.read_vertices(&backend).unwrap(), before);
    renderer.update_pages(&backend, &[(1, &lifted)]).unwrap();
    let after = renderer.read_vertices(&backend).unwrap();
    let stride = VERTICES_PER_PATCH as usize * 8;
    assert_eq!(before[..stride], after[..stride]);
    assert_eq!(before[stride * 2..], after[stride * 2..]);
    for vertex in after[stride..stride * 2].as_chunks::<8>().0 {
        assert_eq!(vertex[7], 110.0);
    }
    let oversized = (0..5000)
        .map(|_| TerrainPatch {
            anchor: anchor(4, 10, 512, 512),
            page: &page,
        })
        .collect::<Vec<_>>();
    assert!(TerrainRenderer::new(&backend, &oversized, OffscreenTarget::FORMAT).is_err());
}
