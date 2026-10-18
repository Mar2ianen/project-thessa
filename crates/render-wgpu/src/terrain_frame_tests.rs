use super::*;

#[test]
fn shadow_sphere_encloses_dense_surface_heights_and_rejects_custom_frames() {
    let page = HeightPage::bake(&[-200.0, 50.0, 150.0, -100.0], 2, 0.1).unwrap();
    for radius in [1000.0, 6_000_000.0] {
        for face in 0..6 {
            for level in [0, 1, 5, 16] {
                let side = 1 << level;
                for x in [0, side / 2, side - 1] {
                    let anchor = TileAnchor::new(
                        thessa_rcbt_core::tile_anchor::TileKey::new(face, level, x, side - 1 - x)
                            .unwrap(),
                        radius,
                    )
                    .unwrap();
                    let bound = PatchShadowBound::new(&anchor, &page).unwrap();
                    for y in 0..33 {
                        for x in 0..33 {
                            for height in [-200.0, 150.0] {
                                let p = glam::DVec3::from_array(anchor.project_body_m(
                                    [f64::from(x) / 32.0, f64::from(y) / 32.0],
                                    height,
                                ));
                                assert!(
                                    p.distance(bound.center_body_m) <= bound.radius_m,
                                    "face={face} level={level}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    let mut anchor = anchors()[0];
    anchor.anchor_body_m[0] += 0.001;
    assert!(
        PatchShadowBound::new(&anchor, &page).is_none(),
        "unknown/custom frames must not be silently culled"
    );
}

fn anchors() -> Vec<TileAnchor> {
    (0..1024)
        .map(|i| {
            TileAnchor::new(
                thessa_rcbt_core::tile_anchor::TileKey::new(4, 5, i % 32, i / 32).unwrap(),
                6_000_000.0,
            )
            .unwrap()
        })
        .collect()
}

#[test]
#[ignore = "hardware: generated f32 shadow vertices fit analytic bounds; disabled shadows retain no bounds"]
fn generated_shadow_vertices_fit_bounds_and_disabled_has_no_bound_storage() {
    let backend = Backend::bring_up().unwrap();
    let page = HeightPage::bake(&[-200.0, 50.0, 150.0, -100.0], 2, 0.1).unwrap();
    for radius in [1000.0, 6_000_000.0] {
        let mut patches = Vec::new();
        for face in 0..6 {
            for level in [0, 5, 16] {
                let side = 1 << level;
                patches.push(TerrainPatch {
                    anchor: TileAnchor::new(
                        thessa_rcbt_core::tile_anchor::TileKey::new(
                            face,
                            level,
                            side / 2,
                            side / 2,
                        )
                        .unwrap(),
                        radius,
                    )
                    .unwrap(),
                    page: &page,
                });
            }
        }
        let mut requested = thessa_graphics::RequestedGraphics::default();
        requested.shadows.map_size = 512;
        requested.ocean.enabled = false;
        let settings = backend.resolve_graphics(&requested);
        let terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &patches,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: patches.len() as u32,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        let vertices = terrain.read_vertices(&backend).unwrap();
        for (slot, patch) in patches.iter().enumerate() {
            let bound = terrain.shadow_bounds[slot].unwrap();
            for vertex in vertices[slot * VERTICES_PER_PATCH as usize * 8
                ..(slot + 1) * VERTICES_PER_PATCH as usize * 8]
                .as_chunks::<8>()
                .0
            {
                let local = glam::DVec3::new(
                    f64::from(vertex[0]),
                    f64::from(vertex[1]),
                    f64::from(vertex[2]),
                );
                assert!(
                    local.length() <= bound.radius_m,
                    "actual generated shader geometry outside bound: {:?}",
                    patch.anchor.tile
                );
            }
        }
        requested.shadows.enabled = false;
        let settings = backend.resolve_graphics(&requested);
        let mut disabled = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &patches,
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: patches.len() as u32,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        assert_eq!(disabled.shadow_bounds.capacity(), 0);
        assert!(disabled.shadows().is_none());
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        disabled.encode_shadows(&mut encoder, None);
        assert_eq!(disabled.last_shadow_draws(), Default::default());
    }
}
// Exact pre-optimization encoding oracle; intentionally keeps per-tile Vecs.
fn legacy(anchors: &[TileAnchor], camera: [f64; 3]) -> Vec<u8> {
    let mut out = Vec::with_capacity(anchors.len() * 112);
    for anchor in anchors {
        out.extend(bytes(
            anchor
                .to_gpu(camera)
                .unwrap()
                .words()
                .into_iter()
                .flatten()
                .map(f32::to_bits),
        ));
    }
    out
}

fn supplied_scalar(anchors: &[TileAnchor], camera: [f64; 3]) -> Vec<u8> {
    let mut out = Vec::with_capacity(anchors.len() * 112);
    for anchor in anchors {
        for word in anchor.to_gpu(camera).unwrap().words() {
            for val in word {
                out.extend_from_slice(&val.to_le_bytes());
            }
        }
    }
    out
}
#[test]
fn staged_frames_preserve_legacy_bytes_reuse_capacity_and_recover_from_error() {
    let anchors = anchors();
    let mut out = Vec::with_capacity(anchors.len() * 112);
    let pointer = out.as_ptr();
    let capacity = out.capacity();
    for camera in [[0.0; 3], [1e12, -2e11, 7e10], [0.0, 0.0, 6_000_010.0]] {
        encode_frames(&mut out, &anchors, camera).unwrap();
        assert_eq!(out, legacy(&anchors, camera));
        assert_eq!(out, supplied_scalar(&anchors, camera));
        assert_eq!(out.as_ptr(), pointer);
        assert_eq!(out.capacity(), capacity);
    }
    assert!(encode_frames(&mut out, &anchors, [f64::NAN; 3]).is_err());
    encode_frames(&mut out, &anchors[..7], [0.0; 3]).unwrap();
    assert_eq!(out, legacy(&anchors[..7], [0.0; 3]));
    assert_eq!(out.as_ptr(), pointer);
    encode_frames(&mut out, &[], [0.0; 3]).unwrap();
    assert!(out.is_empty());
}
#[test]
#[ignore = "release CPU diagnostic: identical 1024-anchor legacy/fresh/staged serialization"]
fn frame_serialization_cost() {
    let anchors = anchors();
    let mut staged = Vec::with_capacity(anchors.len() * 112);
    let mut timings = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    for i in 0..220 {
        let camera = [(i as f64 * 0.01).sin() * 100.0, 0.0, 6_000_010.0];
        for offset in 0..4 {
            let mode = (i + offset) % 4;
            let start = std::time::Instant::now();
            match mode {
                0 => {
                    std::hint::black_box(legacy(std::hint::black_box(&anchors), camera));
                }
                1 => {
                    std::hint::black_box(supplied_scalar(std::hint::black_box(&anchors), camera));
                }
                2 => {
                    std::hint::black_box(
                        frame_bytes(std::hint::black_box(&anchors), camera).unwrap(),
                    );
                }
                _ => {
                    encode_frames(&mut staged, std::hint::black_box(&anchors), camera).unwrap();
                    std::hint::black_box(&staged);
                }
            }
            if i >= 20 {
                timings[mode].push(start.elapsed().as_secs_f64() * 1e6);
            }
        }
    }
    for (name, mut samples) in [
        "legacy_per_tile_vec",
        "supplied_scalar_fresh_vec",
        "packed_fresh_vec",
        "packed_reused_staging",
    ]
    .into_iter()
    .zip(timings)
    {
        samples.sort_by(f64::total_cmp);
        eprintln!(
            "terrain_frame_encoding mode={name} anchors=1024 bytes={} samples=200 warmup=20 p50_us={} p95_us={}; CPU encode only, no driver/uploads, allocation counts not instrumented",
            staged.len(),
            samples[100],
            samples[190]
        );
    }
}

#[test]
#[ignore = "hardware: projection-only view changes reuse uploaded terrain frames"]
fn unchanged_origin_does_not_serialize_frames_and_changed_origin_recovers() {
    let backend = Backend::bring_up().unwrap();
    let anchor = anchors()[0];
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let mut terrain = TerrainRenderer::new(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
    )
    .unwrap();
    let mut params = ViewParams {
        world_from_view: glam::DMat4::from_translation(glam::DVec3::Z * 6_000_010.0),
        world_position_m: glam::DVec3::Z * 6_000_010.0,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.1, 1.0, 0.1, 1e7),
        exposure: 1.0,
    };
    terrain.update_view(&backend, &params).unwrap();
    let capacity = terrain.frame_staging.capacity();
    // Private scratch is not published state; clearing it makes any accidental
    // repeat serialization observable without adding production counters.
    terrain.frame_staging.clear();
    params.exposure = 0.75;
    params.world_from_view = glam::DMat4::from_rotation_translation(
        glam::DQuat::from_rotation_z(0.4),
        params.world_position_m,
    );
    terrain.update_view(&backend, &params).unwrap();
    assert!(terrain.frame_staging.is_empty());
    params.world_position_m.x += 0.1;
    params.world_from_view.w_axis = params.world_position_m.extend(1.0);
    terrain.update_view(&backend, &params).unwrap();
    assert_eq!(
        terrain.frame_staging,
        legacy(&[anchor], params.world_position_m.to_array())
    );
    assert_eq!(terrain.frame_staging.capacity(), capacity);
    let published = terrain.frame_staging.clone();
    let camera = terrain.camera;
    params.world_position_m.x = f64::NAN;
    assert!(terrain.update_view(&backend, &params).is_err());
    assert_eq!(terrain.frame_staging, published);
    assert_eq!(terrain.camera, camera);
}
