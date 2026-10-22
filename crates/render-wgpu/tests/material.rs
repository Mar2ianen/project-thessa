use glam::DVec3;
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    material_page::CbtMaterialPage,
    material_source::MaterialPageStore,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{OffscreenTarget, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

#[test]
#[ignore = "requires a wgpu adapter; run with --ignored"]
fn close_material_raster_keeps_subtexel_motion_in_all_cube_charts() {
    let backend = Backend::bring_up().expect("requires the local hardware adapter");
    let height = HeightPage::bake(&[0.0; 4], 2, 0.01).unwrap();
    let settings = thessa_graphics::ResolvedGraphicsSettings {
        terrain_normal_maps: false,
        specular_reflections: false,
        shadow_enabled: false,
        ..Default::default()
    };
    let target = OffscreenTarget::new(&backend, 65, 65).unwrap();
    // A sharp but linearly filtered step exposes coordinate quantization that
    // a broad colour ramp hides. White ambient PI makes output equal albedo.
    let rgba: Vec<_> = (0..128 * 128)
        .flat_map(|i| [if i % 128 < 64 { 32 } else { 224 }, 0, 0, 255])
        .collect();
    let material = CbtMaterialPage::from_rgba8(rgba).unwrap();
    for face in 0..6 {
        // Away from face centre, to catch direction/chart cancellation.
        let key = TileKey::new(face, 17, 113_491, 95_110).unwrap();
        let anchor = TileAnchor::new(key, 3_200_000.0).unwrap();
        let mut renderer = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor,
                page: &height,
            }],
            OffscreenTarget::FORMAT,
            thessa_render_wgpu::terrain::TerrainCapacity {
                patches: 1,
                material_layers: 1,
                height_grid_size: 2,
            },
            Some(&settings),
        )
        .unwrap();
        renderer
            .update_lighting(
                &backend,
                &SurfaceLighting {
                    ambient_lux: [std::f32::consts::PI; 4],
                    directions: [[0.0; 4]; 3],
                    colors_lux: [[0.0; 4]; 3],
                },
            )
            .unwrap();
        let id = thessa_rcbt_core::conforming_surface::page_id(key).unwrap();
        let pages = [(id, (1, material.clone()))].into_iter().collect();
        renderer.update_materials(&backend, &[id], &pages).unwrap();
        let mut reds = Vec::new();
        for step in -4..=4 {
            let u = 0.5 + f64::from(step) / 4096.0;
            let center = DVec3::from(anchor.project_body_m([u, 0.5], 0.0));
            let eye = center + DVec3::from(anchor.normal) * 2.0;
            // A tiny field of view isolates magnification, not mip choice.
            let view = ViewParams {
                world_from_view: glam::dcamera::rh::view::look_at_mat4(
                    eye,
                    center,
                    DVec3::from(anchor.raw_axis_v),
                )
                .inverse(),
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    0.01, 1.0, 0.01, 1000.0,
                ),
                world_position_m: eye,
                exposure: 1.0,
            };
            renderer.update_view(&backend, &view).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            let pixels = target.read_rgba8(&backend).unwrap();
            let actual = pixels[(32 * 65 + 32) * 4];
            let decode = |v: f64| ((v / 255.0 + 0.055) / 1.055).powf(2.4);
            let t = 0.5 + (u - 0.5) * 125.0;
            let linear = decode(32.0) * (1.0 - t) + decode(224.0) * t;
            let expected = ((1.055 * linear.powf(1.0 / 2.4) - 0.055) * 255.0).round() as u8;
            assert!(
                actual.abs_diff(expected) <= 2,
                "face={face} step={step} actual={actual} expected={expected}"
            );
            reds.push(actual);
        }
        assert!(
            reds.windows(2).all(|pair| pair[0] < pair[1]),
            "centimetre-scale camera motion must not stick to world-f32 steps: face={face} {reds:?}"
        );
    }
}

#[test]
fn material_publication_reuses_uploads_falls_back_and_rejects_invalid_snapshots() {
    let backend = match Backend::bring_up() {
        Ok(b) => b,
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    let anchor = TileAnchor::new(TileKey::new(4, 10, 512, 512).unwrap(), 6_371_000.0).unwrap();
    let page = HeightPage::bake(&[120.0; 4], 2, 0.01).unwrap();
    let mut renderer = TerrainRenderer::with_material_capacity(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
        2,
    )
    .unwrap();
    let center = DVec3::from(anchor.anchor_body_m);
    let eye = center + DVec3::from(anchor.normal) * 20_000.0;
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
    let render = |renderer: &TerrainRenderer| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        target.read_rgba8(&backend).unwrap()
    };
    let fallback = render(&renderer);
    let red = CbtMaterialPage::from_rgba8([255, 0, 0, 255].repeat(128 * 128)).unwrap();
    let blue = CbtMaterialPage::from_rgba8([0, 0, 255, 255].repeat(128 * 128)).unwrap();
    let mut source = MaterialPageStore::default();
    source.set_page(12, red.clone());
    source.set_priority(vec![12]);
    let stats = renderer
        .update_materials(&backend, source.priority(), source.source_pages())
        .unwrap();
    assert_eq!(
        (stats.pages, stats.bytes, stats.residents),
        (1, red.byte_len(), 1)
    );
    let red_frame = render(&renderer);
    let mid = (128 * 257 + 128) * 4;
    assert!(red_frame[mid] > red_frame[mid + 2] + 30);
    assert_ne!(
        fallback, red_frame,
        "resident parent must affect actual raster"
    );
    source.set_priority(vec![12, 12]);
    assert_eq!(
        renderer
            .update_materials(&backend, source.priority(), source.source_pages())
            .unwrap()
            .pages,
        0
    );
    assert_eq!(render(&renderer), red_frame);
    // Invalid IDs and incomplete decoded mips must not mutate previous state.
    let mut pages = source.source_pages().clone();
    pages.insert(16, (2, blue.clone()));
    assert!(renderer.update_materials(&backend, &[12], &pages).is_err());
    assert_eq!(render(&renderer), red_frame);
    pages.remove(&16);
    pages.insert(
        12,
        (
            2,
            CbtMaterialPage::from_decoded_mips(vec![vec![0; 128 * 128 * 4]]).unwrap(),
        ),
    );
    assert!(renderer.update_materials(&backend, &[12], &pages).is_err());
    assert_eq!(render(&renderer), red_frame);
    let old_snapshot = source.clone();
    source.set_page(12, blue);
    assert_eq!(
        renderer
            .update_materials(&backend, source.priority(), source.source_pages())
            .unwrap()
            .pages,
        1
    );
    let blue_frame = render(&renderer);
    assert!(blue_frame[mid + 2] > blue_frame[mid] + 30);
    // Removal invalidates the GPU directory even though the layer stays warm.
    source.remove_page(12);
    source.set_priority(vec![]);
    let stats = renderer
        .update_materials(&backend, source.priority(), source.source_pages())
        .unwrap();
    assert_eq!((stats.pages, stats.residents), (0, 0));
    assert_eq!(render(&renderer), fallback);
    source.set_page(12, red);
    source.set_priority(vec![12]);
    assert!(source.source_pages()[&12].0 > old_snapshot.source_pages()[&12].0);
    assert_eq!(
        renderer
            .update_materials(&backend, source.priority(), source.source_pages())
            .unwrap()
            .pages,
        1
    );
    assert_eq!(render(&renderer), red_frame);
    assert_eq!(old_snapshot.source_pages()[&12].1.mips()[0][0], 255);
}

#[test]
fn saturated_material_cache_renders_priority_detail_not_only_coarse_cover() {
    let backend = match Backend::bring_up() {
        Ok(backend) => backend,
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU");
            return;
        }
        Err(error) => panic!("{error}"),
    };
    let key = TileKey::new(4, 10, 512, 512).unwrap();
    let anchor = TileAnchor::new(key, 6_371_000.0).unwrap();
    let height = HeightPage::bake(&[120.0; 4], 2, 0.01).unwrap();
    let mut renderer = TerrainRenderer::with_material_capacity(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &height,
        }],
        OffscreenTarget::FORMAT,
        4,
    )
    .unwrap();
    let center = DVec3::from(anchor.anchor_body_m);
    let eye = center + DVec3::from(anchor.normal) * 2_000.0;
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
    let near = thessa_rcbt_core::conforming_surface::page_id(key).unwrap();
    let red = CbtMaterialPage::from_rgba8([255, 0, 0, 255].repeat(128 * 128)).unwrap();
    let blue = CbtMaterialPage::from_rgba8([0, 0, 255, 255].repeat(128 * 128)).unwrap();
    let mut source = MaterialPageStore::default();
    source.set_pages([8, 12, 32, 33, 34, 35].map(|id| (id, red.clone())));
    source.set_page(near, blue);
    source.set_priority(vec![near, 32, 33, 34, 35]);
    let stats = renderer
        .update_materials(&backend, source.priority(), source.source_pages())
        .unwrap();
    assert_eq!(stats.residents, 4);
    assert_eq!(stats.resident_levels[0], 2, "both faces keep support");
    assert_eq!(
        stats.resident_levels[10], 1,
        "close detail must actually reach the GPU directory"
    );
    let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    renderer.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    let image = target.read_rgba8(&backend).unwrap();
    let mid = (128 * 257 + 128) * 4;
    assert!(
        image[mid + 2] > image[mid] + 30,
        "priority detail must affect RGB, not only counters: {:?}",
        &image[mid..mid + 4]
    );
    let next = renderer
        .update_materials(&backend, source.priority(), source.source_pages())
        .unwrap();
    assert_eq!(next.pages, 0, "stationary demand must not reupload");
    assert_eq!(next.resident_levels, stats.resident_levels);
}

#[test]
#[ignore = "diagnostic hardware: counterbalanced normal-map raster and 128-page upload costs, not a performance assertion"]
fn material_normal_gpu_and_upload_cost() {
    use thessa_graphics::{Quality, ResolvedGraphicsSettings};
    use thessa_rcbt_core::material_page::MaterialPages;
    use thessa_render_wgpu::{terrain::TerrainCapacity, timing::GpuIntervals};
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let timer = GpuIntervals::new(&backend, 2).expect("timestamp queries required");
    let anchor = TileAnchor::new(TileKey::new(4, 10, 512, 512).unwrap(), 6_371_000.0).unwrap();
    let height = HeightPage::bake(&[120.0; 4], 2, 0.01).unwrap();
    let normal = (0..128 * 128)
        .flat_map(|i| {
            [
                (128.0 + 60.0 * (i as f64 * 0.071).sin()) as u8,
                (128.0 + 60.0 * (i as f64 * 0.043).cos()) as u8,
            ]
        })
        .collect();
    let page = CbtMaterialPage::from_rgba8_with_normals(
        [153, 153, 153, 255].repeat(128 * 128),
        Some(normal),
    )
    .unwrap();
    let ids: Vec<_> = std::iter::once(12)
        .chain((0..127).map(|i| {
            thessa_rcbt_core::conforming_surface::page_id(
                TileKey::new((i % 6) as u8, 3, (i / 6 % 8) as u32, (i / 48) as u32).unwrap(),
            )
            .unwrap()
        }))
        .collect();
    let mut pages: MaterialPages = ids.iter().map(|&id| (id, (1, page.clone()))).collect();
    let configs = [
        (false, Quality::High),
        (true, Quality::Low),
        (true, Quality::Medium),
        (true, Quality::High),
    ];
    let mut terrains = configs.map(|(enabled, quality)| {
        let settings = ResolvedGraphicsSettings {
            terrain_normal_maps: enabled,
            surface_quality: quality,
            specular_reflections: false,
            shadow_enabled: false,
            ..Default::default()
        };
        let mut terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor,
                page: &height,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 1,
                material_layers: 128,
                height_grid_size: 2,
            },
            Some(&settings),
        )
        .unwrap();
        terrain.update_materials(&backend, &ids, &pages).unwrap();
        let center = DVec3::from(anchor.anchor_body_m);
        let eye = center + DVec3::from(anchor.normal) * 2000.0;
        terrain
            .update_view_for_extent(
                &backend,
                &ViewParams {
                    world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y)
                        .inverse(),
                    clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                        1.0,
                        1280.0 / 720.0,
                        1.0,
                        1e6,
                    ),
                    world_position_m: eye,
                    exposure: 1.0,
                },
                [1280, 720],
            )
            .unwrap();
        terrain
    });
    let target = OffscreenTarget::new(&backend, 1280, 720).unwrap();
    let mut raster: [Vec<f64>; 4] = std::array::from_fn(|_| Vec::new());
    for index in [0, 1, 2, 3, 3, 2, 1, 0] {
        for frame in 0..20 {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timer.mark(&mut encoder, 0);
            terrains[index].encode(&mut encoder, &target.color, &target.depth);
            timer.mark(&mut encoder, 1);
            timer.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let ms = timer.read(&backend).unwrap()[0] * 1000.0;
            if frame >= 4 {
                raster[index].push(ms);
            }
        }
    }
    for (i, values) in raster.iter_mut().enumerate() {
        values.sort_by(f64::total_cmp);
        eprintln!(
            "material_normal_raster adapter={:?} enabled={} quality={:?} extent=1280x720 measurements=32 median_ms={:.6} p95_ms={:.6}; regular close-land patch, diffuse only, no sky/clouds/shadows/specular, classify+raster not whole-client time",
            backend.adapter.get_info(),
            configs[i].0,
            configs[i].1,
            values[16],
            values[30]
        );
    }
    let mut enqueue: [Vec<f64>; 2] = std::array::from_fn(|_| Vec::new());
    let mut drained: [Vec<f64>; 2] = std::array::from_fn(|_| Vec::new());
    for round in 0..12 {
        for index in if round % 2 == 0 { [0, 3] } else { [3, 0] } {
            let enabled = usize::from(configs[index].0);
            for (generation, _) in pages.values_mut() {
                *generation += 1;
            }
            let start = std::time::Instant::now();
            let stats = terrains[index]
                .update_materials(&backend, &ids, &pages)
                .unwrap();
            let queued_ms = start.elapsed().as_secs_f64() * 1000.0;
            let submission = backend.queue.submit([]);
            backend
                .device
                .poll(wgpu::PollType::Wait {
                    submission_index: Some(submission),
                    timeout: None,
                })
                .unwrap();
            let drained_ms = start.elapsed().as_secs_f64() * 1000.0;
            assert_eq!(stats.pages, 128);
            assert_eq!(stats.normal_pages, enabled * 128);
            assert_eq!(stats.bytes, 128 * (87_380 + enabled * 43_690));
            if round >= 2 {
                enqueue[enabled].push(queued_ms);
                drained[enabled].push(drained_ms);
            }
        }
    }
    for enabled in 0..2 {
        enqueue[enabled].sort_by(f64::total_cmp);
        drained[enabled].sort_by(f64::total_cmp);
        eprintln!(
            "material_normal_upload enabled={} pages=128 bytes={} measurements=10 enqueue_median_ms={:.6} enqueue_p95_ms={:.6} drained_median_ms={:.6} drained_p95_ms={:.6}; counterbalanced repeated dirty snapshots, enqueue is CPU validation/residency/write_texture, drained includes queue submit and GPU wait, not isolated DMA or streaming latency",
            enabled == 1,
            128 * (87_380 + enabled * 43_690),
            enqueue[enabled][5],
            enqueue[enabled][9],
            drained[enabled][5],
            drained[enabled][9]
        );
    }
}

#[test]
fn material_normal_plane_perturbs_actual_raster_diffuse_and_clears_on_removal() {
    let backend = match Backend::bring_up() {
        Ok(b) => b,
        Err(BackendError::NoAdapter) => {
            eprintln!("SKIP: no GPU");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    for (normal_maps, water) in [(true, false), (false, false), (true, true)] {
        let anchor = TileAnchor::new(TileKey::new(4, 10, 512, 512).unwrap(), 6_371_000.0).unwrap();
        let page = HeightPage::bake(&[if water { 0.0 } else { 120.0 }; 4], 2, 0.01).unwrap();
        let settings = thessa_graphics::ResolvedGraphicsSettings {
            terrain_normal_maps: normal_maps,
            specular_reflections: false,
            shadow_enabled: false,
            ..Default::default()
        };
        let mut renderer = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor,
                page: &page,
            }],
            OffscreenTarget::FORMAT,
            thessa_render_wgpu::terrain::TerrainCapacity {
                patches: 1,
                material_layers: 2,
                height_grid_size: 2,
            },
            Some(&settings),
        )
        .unwrap();
        // A hard +X sun makes the flat +Z terrain dark; a normal tilted toward +X
        // can only brighten it through the production fragment path.
        renderer
            .update_lighting(
                &backend,
                &SurfaceLighting {
                    ambient_lux: [0.05, 0.05, 0.05, 0.0],
                    directions: [[1.0, 0.0, 0.0, 0.0], [0.0; 4], [0.0; 4]],
                    colors_lux: [[1.5, 1.5, 1.5, 0.0], [0.0; 4], [0.0; 4]],
                },
            )
            .unwrap();
        let center = DVec3::from(anchor.anchor_body_m);
        let eye = center + DVec3::from(anchor.normal) * 20_000.0;
        renderer
            .update_view(
                &backend,
                &ViewParams {
                    world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y)
                        .inverse(),
                    clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                        1.0, 1.0, 1.0, 1e6,
                    ),
                    world_position_m: eye,
                    exposure: 1.0,
                },
            )
            .unwrap();
        let target = OffscreenTarget::new(&backend, 257, 257).unwrap();
        let render = |renderer: &TerrainRenderer| {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let rgba = [255, 0, 0, if water { 40 } else { 255 }].repeat(128 * 128);
        let flat = CbtMaterialPage::from_rgba8(rgba.clone()).unwrap();
        // 230/255 decodes to +0.804 in x; 128/255 stays a flat y slope.
        let tilted =
            CbtMaterialPage::from_rgba8_with_normals(rgba, Some([230, 128].repeat(128 * 128)))
                .unwrap();
        let mut source = MaterialPageStore::default();
        source.set_page(12, flat.clone());
        source.set_priority(vec![12]);
        let stats = renderer
            .update_materials(&backend, source.priority(), source.source_pages())
            .unwrap();
        assert_eq!((stats.pages, stats.normal_pages), (1, 0));
        let flat_frame = render(&renderer);
        source.set_page(12, tilted.clone());
        let stats = renderer
            .update_materials(&backend, source.priority(), source.source_pages())
            .unwrap();
        assert_eq!(
            (stats.pages, stats.normal_pages),
            (1, usize::from(normal_maps))
        );
        assert_eq!(
            stats.bytes,
            flat.byte_len() + if normal_maps { 43_690 } else { 0 }
        );
        let tilted_frame = render(&renderer);
        let mid = (128 * 257 + 128) * 4;
        if normal_maps && !water {
            assert!(
                tilted_frame[mid] > flat_frame[mid] + 20,
                "normal page must brighten the +X-lit slope: flat={} tilted={}",
                flat_frame[mid],
                tilted_frame[mid]
            );
            // Independent Lambert reference, below saturation. The constant
            // normal page avoids residency/filter ambiguity at the centre.
            let n = DVec3::from(anchor.normal).normalize();
            let t = (DVec3::X - n * n.x).normalize();
            let b = n.cross(t);
            let x = 230.0 / 255.0 * 2.0 - 1.0;
            let y = 128.0 / 255.0 * 2.0 - 1.0;
            let z = (1.0_f64 - x * x - y * y).sqrt();
            let normal = (n + t * (x / z) + b * (y / z)).normalize();
            let linear = (0.05 + 1.5 * normal.x.max(0.0)) / std::f64::consts::PI;
            let reference = ((1.055 * linear.powf(1.0 / 2.4) - 0.055) * 255.0).round() as u8;
            assert!(
                tilted_frame[mid].abs_diff(reference) <= 2,
                "Lambert normal oracle: actual={} reference={reference}",
                tilted_frame[mid]
            );
        } else {
            assert_eq!(
                tilted_frame, flat_frame,
                "disabled or water material normals must be inert"
            );
        }
        // Removing the normal plane must clear the directory entry rather than
        // keep sampling the warm slot.
        source.set_page(12, flat);
        assert_eq!(
            renderer
                .update_materials(&backend, source.priority(), source.source_pages())
                .unwrap()
                .pages,
            1
        );
        assert_eq!(render(&renderer), flat_frame);
        if normal_maps && !water {
            // A lone normal-bearing fine page is only barely resident at this
            // location near its corner. Its uncovered weight must stay flat,
            // not be divided out to produce the full tilted response.
            source.set_page(51, tilted);
            source.set_priority(vec![51]);
            renderer
                .update_materials(&backend, source.priority(), source.source_pages())
                .unwrap();
            let sparse = render(&renderer);
            assert!(
                sparse[mid].abs_diff(flat_frame[mid]) <= 5,
                "sparse normal coverage amplified: flat={} sparse={}",
                flat_frame[mid],
                sparse[mid]
            );
        }
    }
}
