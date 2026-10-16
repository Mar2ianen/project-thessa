use glam::{DVec3, Vec3};
use thessa_rcbt_core::{
    HeightPage,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{
        OffscreenTarget, ResidentTerrainPatch, TerrainCapacity, TerrainPatch, TerrainRenderer,
        VERTICES_PER_PATCH,
    },
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

#[test]
fn moving_body_and_origin_preserve_camera_relative_gpu_output_without_geometry_rebuilds() {
    use glam::DQuat;
    use thessa_render_core::BodyFrame;
    let Some(backend) = backend() else { return };
    let anchor = anchor(4, 10, 512, 512);
    let center = DVec3::from(anchor.anchor_body_m);
    let eye = center + DVec3::from(anchor.normal) * 20_000.0;
    let body_view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y).inverse(),
        world_position_m: eye,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e6),
        exposure: 1.0,
    };
    let page = HeightPage::bake(&[120.0, 123.0, 121.0, 125.0], 2, 0.001).unwrap();
    let mut renderer = TerrainRenderer::new(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    let geometry = renderer.read_vertices(&backend).unwrap();
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let mut reference = None;
    for step in 0..8 {
        let rotation = DQuat::from_rotation_y(step as f64 * 0.37);
        let body = DVec3::new(1.5e11 + step as f64 * 1e8, -1e12, 2e12);
        let frame = BodyFrame::new(body, rotation, body + DVec3::new(128.0, -64.0, 256.0)).unwrap();
        let render_view = ViewParams {
            world_from_view: frame.render_from_body() * body_view.world_from_view,
            world_position_m: frame.body_to_render_point(eye),
            ..body_view
        };
        let view = frame.view_in_body(&render_view);
        assert!((view.world_position_m - eye).length() < 4e-9);
        renderer.update_view(&backend, &view).unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        let pixels = target.read_rgba8(&backend).unwrap();
        assert_eq!(renderer.read_draw(&backend).unwrap()[0], 2304 * 3);
        if let Some(expected) = &reference {
            assert_eq!(
                &pixels, expected,
                "body motion leaked into body-attached camera"
            );
        } else {
            assert!(
                pixels
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .filter(|p| p.as_slice() != &pixels[..4])
                    .count()
                    > 10_000
            );
            reference = Some(pixels);
        }
        assert_eq!(renderer.read_vertices(&backend).unwrap(), geometry);
        // Rotation/translation changes are camera work, not a height upload.
        let retained = renderer
            .publish_partition(
                &backend,
                &[ResidentTerrainPatch {
                    anchor,
                    page: &page,
                    generation: 0,
                }],
            )
            .unwrap();
        assert_eq!(retained.regenerated, 0);
    }
}

#[test]
fn neutral_height_source_dirty_and_reinserted_versions_regenerate_gpu_geometry() {
    use thessa_rcbt_core::height_source::HeightPageStore;
    let Some(backend) = backend() else { return };
    let anchor = anchor(4, 0, 0, 0);
    let id = 12;
    let mut source = HeightPageStore::default();
    source.set_page(id, HeightPage::bake(&[10.0; 4], 2, 0.01).unwrap());
    let mut renderer = TerrainRenderer::with_capacity(
        &backend,
        &[TerrainPatch {
            anchor,
            page: source.get(id).unwrap(),
        }],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
    )
    .unwrap();
    let publish = |renderer: &mut TerrainRenderer, source: &HeightPageStore| {
        renderer
            .publish_partition(
                &backend,
                &[ResidentTerrainPatch {
                    anchor,
                    page: source.get(id).unwrap(),
                    generation: source.page_version(id).unwrap(),
                }],
            )
            .unwrap()
            .regenerated
    };
    assert_eq!(publish(&mut renderer, &source), 1);
    assert_eq!(publish(&mut renderer, &source), 0);
    let snapshot = source.clone();
    source.set_page(id, HeightPage::bake(&[500.0; 4], 2, 0.01).unwrap());
    assert_eq!(publish(&mut renderer, &source), 1);
    for vertex in renderer.read_vertices(&backend).unwrap().as_chunks::<8>().0 {
        assert_eq!(vertex[7], 500.0);
    }
    let version = source.page_version(id).unwrap();
    source.remove_page(id);
    renderer.publish_partition(&backend, &[]).unwrap();
    source.set_page(id, HeightPage::bake(&[900.0; 4], 2, 0.01).unwrap());
    assert!(source.page_version(id).unwrap() > version);
    assert_eq!(publish(&mut renderer, &source), 1);
    for vertex in renderer.read_vertices(&backend).unwrap().as_chunks::<8>().0 {
        assert_eq!(vertex[7], 900.0);
    }
    assert_ne!(snapshot.get(id), source.get(id));
}

#[test]
fn partition_reuses_slots_rejects_atomically_and_clears_sparse_and_empty_draws() {
    let Some(backend) = backend() else { return };
    let a = anchor(4, 10, 512, 512);
    let b = anchor(4, 10, 513, 512);
    let c = anchor(4, 10, 514, 512);
    let id = |a: TileAnchor| {
        let mut id = 8 + u64::from(a.tile.face);
        for bit in (0..a.tile.level).rev() {
            id = (id << 2) | u64::from(((a.tile.x >> bit) & 1) * 2 + ((a.tile.y >> bit) & 1));
        }
        id
    };
    let page = HeightPage::bake(&[100.0; 4], 2, 0.01).unwrap();
    let high = HeightPage::bake(&[500.0; 4], 2, 0.01).unwrap();
    let too_big = HeightPage::bake(&[0.0; 16], 4, 0.01).unwrap();
    let resident = |anchor, generation| ResidentTerrainPatch {
        anchor,
        page: &page,
        generation,
    };
    let mut renderer = TerrainRenderer::with_capacity(
        &backend,
        &[
            TerrainPatch {
                anchor: a,
                page: &page,
            },
            TerrainPatch {
                anchor: b,
                page: &page,
            },
        ],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 3,
            height_grid_size: 3,
            material_layers: 1,
        },
    )
    .unwrap();
    let before = renderer.read_vertices(&backend).unwrap();
    let stats = renderer
        .publish_partition(&backend, &[resident(b, 0), resident(a, 0)])
        .unwrap();
    assert_eq!((stats.active, stats.regenerated, stats.retained), (2, 0, 2));
    assert_eq!(renderer.slot(id(a)), Some(0));
    assert_eq!(renderer.slot(id(b)), Some(1));
    assert_eq!(renderer.read_vertices(&backend).unwrap(), before);
    for invalid in [
        vec![resident(a, 1), resident(a, 1)],
        vec![resident(a, 1), resident(anchor(4, 0, 0, 0), 1)],
        vec![
            ResidentTerrainPatch {
                anchor: a,
                page: &high,
                generation: 1,
            },
            ResidentTerrainPatch {
                anchor: c,
                page: &too_big,
                generation: 1,
            },
        ],
        vec![
            resident(a, 0),
            resident(b, 0),
            resident(c, 0),
            resident(anchor(0, 0, 0, 0), 0),
        ],
    ] {
        assert!(renderer.publish_partition(&backend, &invalid).is_err());
        assert_eq!(renderer.slot(id(a)), Some(0));
        assert_eq!(renderer.slot(id(c)), None);
        assert_eq!(renderer.read_vertices(&backend).unwrap(), before);
    }
    let center = DVec3::from(b.anchor_body_m);
    let eye = center + DVec3::from(b.normal) * 40_000.0;
    renderer
        .update_view(
            &backend,
            &ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y)
                    .inverse(),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e6),
                world_position_m: eye,
                exposure: 1.0,
            },
        )
        .unwrap();
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let draw = |renderer: &TerrainRenderer| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        renderer.read_draw(&backend).unwrap()[0]
    };
    assert_eq!(draw(&renderer), 2 * 2304 * 3);
    let stats = renderer
        .publish_partition(&backend, &[resident(b, 0)])
        .unwrap();
    assert_eq!((stats.regenerated, stats.dispatch_slots), (0, 2));
    assert_eq!(
        draw(&renderer),
        2304 * 3,
        "inactive interior slot must not draw"
    );
    assert!(renderer.update_pages(&backend, &[(0, &high)]).is_err());
    let stats = renderer
        .publish_partition(
            &backend,
            &[
                resident(b, 0),
                ResidentTerrainPatch {
                    anchor: c,
                    page: &high,
                    generation: 1,
                },
            ],
        )
        .unwrap();
    assert_eq!((stats.regenerated, stats.retained), (1, 1));
    assert_eq!(renderer.slot(id(c)), Some(0));
    let output = renderer.read_vertices(&backend).unwrap();
    for vertex in output[..VERTICES_PER_PATCH as usize * 8].as_chunks::<8>().0 {
        assert_eq!(vertex[7], 500.0);
    }
    let center_normal = Vec3::from_slice(&output[(16 * 33 + 16) * 8 + 4..][..3]);
    assert!(
        (center_normal - DVec3::from(c.normal).as_vec3()).length() < 1e-5,
        "reused slot must generate the new tile, not its previous anchor"
    );
    renderer.update_pages(&backend, &[(0, &page)]).unwrap();
    assert_eq!(
        renderer
            .publish_partition(
                &backend,
                &[
                    resident(b, 0),
                    ResidentTerrainPatch {
                        anchor: c,
                        page: &high,
                        generation: 1
                    }
                ]
            )
            .unwrap()
            .regenerated,
        1,
        "manual update invalidates the source-generation shortcut"
    );
    renderer.publish_partition(&backend, &[]).unwrap();
    assert_eq!(draw(&renderer), 0);
    assert_eq!(
        renderer
            .publish_partition(&backend, &[resident(a, 0)])
            .unwrap()
            .regenerated,
        1
    );
    assert_eq!(draw(&renderer), 2304 * 3);
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
    let mut renderer = TerrainRenderer::new(
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
    let render = |renderer: &TerrainRenderer| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    let pixels = render(&renderer);
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
    assert_eq!(
        render(&renderer),
        pixels,
        "static frame must be pixel deterministic"
    );
    // Looking away must zero the GPU-generated indirect draw.
    let mut away = params;
    away.world_from_view =
        glam::dcamera::rh::view::look_to_mat4(eye, (eye - center).normalize(), DVec3::Y).inverse();
    renderer.update_view(&backend, &away).unwrap();
    render(&renderer);
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
    let mut renderer = TerrainRenderer::new(&backend, &patches, OffscreenTarget::FORMAT).unwrap();
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
