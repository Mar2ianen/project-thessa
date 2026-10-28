//! Real native full/reduced fern sources versus bounded depth-only geometry.
use super::*;
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_core::ViewParams;
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainRenderer},
};

#[test]
#[ignore = "software Vulkan: actual authored plant shadow silhouettes versus full geometry"]
fn actual_plant_shadow_lods_preserve_near_detail_and_bound_silhouette_changes() {
    compare_plant_shadows(false, false, false);
}
#[test]
#[ignore = "software Vulkan: actual reed caster and all plant organs versus full geometry"]
fn actual_reed_shadow_lods_preserve_receiver_and_coverage() {
    compare_plant_shadows(true, true, false);
}
#[test]
#[ignore = "software Vulkan: combined actual fern/tree/reed hierarchy over twelve shadow layers"]
fn actual_combined_plant_hierarchy_preserves_receivers() {
    compare_plant_shadows(true, false, false);
}
#[test]
#[ignore = "physical GPU: bounded authored fern/tree/reed hierarchy; explicit authorization required"]
fn physical_plant_hierarchy_preserves_receivers() {
    compare_plant_shadows(true, false, true);
}

#[derive(Clone, Copy)]
enum CaptureMode {
    Selected,
    Full,
    NoSamples,
}

fn compare_plant_shadows(include_reeds: bool, isolate_reeds: bool, physical: bool) {
    assert_eq!(
        std::path::Path::new("/dev/dri/renderD128").exists(),
        physical
    );
    let backend = if physical {
        let mut request = thessa_graphics::RequestedGraphics::default();
        request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
        Backend::bring_up_for_graphics_profiled(&request).unwrap()
    } else {
        Backend::bring_up().unwrap()
    };
    assert_eq!(
        backend.adapter.get_info().device_type == wgpu::DeviceType::Cpu,
        !physical
    );
    eprintln!("plant shadows adapter={:?}", backend.adapter.get_info());
    let output = std::path::PathBuf::from(std::env::var_os("THESSA_WORKLOAD_OUTPUT").unwrap());
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.shadows.mesh_lod_error_texels = 0.5;
    request.shadows.raster_sample_culling = true;
    request.shadows.cascades = if include_reeds && !isolate_reeds {
        4
    } else {
        2
    };
    let cascades = request.shadows.cascades as usize;
    request.shadows.map_size = 512;
    request.shadows.max_distance_m = if isolate_reeds {
        4096.0
    } else if include_reeds {
        12000.0
    } else {
        256.0
    };
    request.shadows.first_cascade_distance_m = Some(25.0);
    let settings = backend.resolve_graphics(&request);
    assert_eq!(settings.native_shadow_map_size, 512);
    let page = thessa_rcbt_core::HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patch = thessa_render_wgpu::terrain::TerrainPatch {
        anchor: thessa_rcbt_core::tile_anchor::TileAnchor::new(
            thessa_rcbt_core::tile_anchor::TileKey::new(0, 0, 0, 0).unwrap(),
            1000.0,
        )
        .unwrap(),
        page: &page,
    };
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &[patch],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    terrain.publish_partition(&backend, &[]).unwrap();
    let mut lights = SurfaceLighting {
        directions: [
            DVec3::new(0.4, 1.0, 0.2).normalize(),
            DVec3::new(-0.5, 0.1, 0.7).normalize(),
            DVec3::new(0.1, 0.0, 1.0).normalize(),
        ]
        .map(|d| [d.x as f32, d.y as f32, d.z as f32, 0.0]),
        colors_lux: [[1.0; 4]; 3],
        ..Default::default()
    };
    let mut items = Vec::new();
    for kind in [
        thessa_worldgen_rocky::scatter::ScatterKind::FernClump,
        thessa_worldgen_rocky::scatter::ScatterKind::LycopsidTree,
        thessa_worldgen_rocky::scatter::ScatterKind::ReedClump,
    ] {
        if kind == thessa_worldgen_rocky::scatter::ScatterKind::ReedClump && !include_reeds {
            continue;
        }
        for form in 0..if kind == thessa_worldgen_rocky::scatter::ScatterKind::ReedClump {
            1
        } else {
            3
        } {
            let id = (0..10_000)
                .map(thessa_worldgen_rocky::scatter::ScatterId)
                .find(|&id| thessa_worldgen_rocky::offline::plant_form(id, kind) == form)
                .unwrap();
            items.push(ScatterItem {
                id,
                kind,
                direction: DVec3::Y.to_array(),
                ground_height_m: 0.0,
                scale: 1.0,
                yaw_rad: 0.0,
            });
        }
    }
    let target = OffscreenTarget::new(&backend, 257, 193).unwrap();
    let mut cases = Vec::new();
    let mut timings = Vec::new();
    let mut changed = 0_usize;
    let mut saved = 0_u64;
    for reduced in [false, true] {
        let radius = 3_200_000.0;
        let mut prepared = convert_with_detail(
            items.clone(),
            radius,
            reduced.then_some((DVec3::X * radius, 48.0, 0.0)),
        )
        .unwrap();
        append_shadow_lods(&mut prepared);
        assert!(!prepared.shadow_lods.is_empty());
        if isolate_reeds {
            // Isolate the reed approximation; the six-form fixture independently
            // checks all fern/tree levels at its unchanged layout and distance.
            prepared.shadow_lods.retain(|lod| {
                prepared.descriptors[prepared.ranges[lod.primitive].start as usize].kind
                    == thessa_worldgen_rocky::scatter::ScatterKind::ReedClump
            });
            assert!(prepared.shadow_lods.iter().any(|lod| {
                prepared.descriptors[prepared.ranges[lod.primitive].start as usize].kind
                    == thessa_worldgen_rocky::scatter::ScatterKind::ReedClump
            }));
        }
        let ground_asset = MeshAsset {
            primitives: vec![Primitive {
                ambient_visibility: vec![],
                material: 0,
                vertices: [
                    [-32.0, -4.0, -40.0],
                    [32.0, -4.0, -40.0],
                    [32.0, -4.0, 0.0],
                    [-32.0, -4.0, 0.0],
                ]
                .map(|position| Vertex {
                    position,
                    normal: [0.0, 1.0, 0.0],
                    uv: [0.0; 2],
                })
                .to_vec(),
                indices: vec![0, 2, 1, 0, 3, 2],
            }],
            materials: vec![Material {
                base_color: [0.2, 0.14, 0.08, 1.0],
                metallic: 0.0,
                roughness: 1.0,
                ..Default::default()
            }],
            textures: vec![],
            images: vec![],
        };
        let mut ground = MeshRenderer::with_surface_and_terrain(
            &backend,
            &ground_asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: settings.surface_quality,
                reflections: false,
                shadows: terrain.shadows(),
                ray_tracing: Some(&settings),
            },
            None,
        )
        .unwrap();
        let mut mesh = MeshRenderer::with_static_surface_and_terrain(
            &backend,
            &prepared.asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: prepared.instances.len(),
                quality: settings.surface_quality,
                reflections: false,
                shadows: terrain.shadows(),
                ray_tracing: Some(&settings),
            },
            None,
        )
        .unwrap();
        for (case, origin) in [DVec3::ZERO, DVec3::new(3e12, -2e12, 1e12)]
            .into_iter()
            .enumerate()
        {
            let view = ViewParams {
                world_from_view: glam::DMat4::from_translation(origin),
                world_position_m: origin,
                clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                    1.1,
                    257.0 / 193.0,
                    0.1,
                    300.0,
                ),
                exposure: 1.0,
            };
            let instances: Vec<_> = prepared
                .instances
                .iter()
                .enumerate()
                .map(|(i, _)| MeshInstance {
                    position_m: origin
                        + DVec3::new(
                            (i % 3) as f64 * 12.0 - 12.0,
                            -4.0,
                            -12.0 - (i / 3) as f64 * 12.0,
                        ),
                    rotation: DQuat::from_rotation_y(i as f64 * 0.31),
                    scale: 0.8 + (i % 3) as f64 * 0.2,
                })
                .collect();
            if case == 1 {
                lights.colors_lux[1] = [0.0; 4];
            }
            terrain.update_view(&backend, &view).unwrap();
            terrain.update_lighting(&backend, &lights).unwrap();
            terrain.update_shadows(&backend, origin, &lights).unwrap();
            mesh.update_selected(&backend, &view, &lights, &instances, Some(&prepared.ranges))
                .unwrap();
            ground
                .update(
                    &backend,
                    &view,
                    &lights,
                    &[MeshInstance {
                        position_m: origin,
                        rotation: DQuat::IDENTITY,
                        scale: 1.0,
                    }],
                )
                .unwrap();
            let mut capture = |terrain: &mut TerrainRenderer,
                               mesh: &MeshRenderer,
                               mode: CaptureMode| {
                terrain.invalidate_shadows();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                match mode {
                    CaptureMode::Full => terrain.encode_shadows_reference(&mut encoder, Some(mesh)),
                    CaptureMode::NoSamples => {
                        terrain.encode_shadows_without_sample_culling(&mut encoder, Some(mesh))
                    }
                    CaptureMode::Selected => terrain.encode_shadows(&mut encoder, Some(mesh)),
                }
                let stats = terrain.last_shadow_draws();
                terrain.encode_scene(&mut encoder, &target.color, &target.depth, Some(mesh));
                // The receiver is absent from the plant-only depth oracle, so
                // its coverage cannot hide missing foliage in the mask check.
                ground.encode(&mut encoder, &target.color, &target.depth);
                backend.queue.submit([encoder.finish()]);
                (
                    terrain.shadows().unwrap().read_depth(&backend).unwrap(),
                    target.read_rgba8(&backend).unwrap(),
                    stats,
                )
            };
            mesh.set_shadow_lods(&backend, &[]).unwrap();
            let full = capture(&mut terrain, &mesh, CaptureMode::Selected);
            mesh.set_shadow_lods(&backend, &prepared.shadow_lods)
                .unwrap();
            let reference = capture(&mut terrain, &mesh, CaptureMode::Full);
            assert_eq!(
                full.0, reference.0,
                "LOD installation changed full shadow oracle"
            );
            assert_eq!(
                full.1, reference.1,
                "LOD installation changed primary geometry"
            );
            let baseline = if include_reeds && !isolate_reeds {
                mesh.set_shadow_lods(&backend, &original_shadow_lods(&prepared))
                    .unwrap();
                let previous = capture(&mut terrain, &mesh, CaptureMode::Selected);
                let changed_from_full = full
                    .0
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .zip(previous.0.as_chunks::<4>().0)
                    .filter(|(a, b)| {
                        (f32::from_le_bytes(**a) < 1.0) != (f32::from_le_bytes(**b) < 1.0)
                    })
                    .count();
                eprintln!(
                    "combined original-tier coverage changes from full geometry: {changed_from_full}"
                );
                mesh.set_shadow_lods(&backend, &prepared.shadow_lods)
                    .unwrap();
                previous
            } else {
                full
            };
            let no_samples = capture(&mut terrain, &mesh, CaptureMode::NoSamples);
            let lod = capture(&mut terrain, &mesh, CaptureMode::Selected);
            assert_eq!(no_samples.0, lod.0, "sample culling changed plant atlas");
            assert_eq!(no_samples.1, lod.1, "sample culling changed plant colour");
            eprintln!(
                "plant sample culling exact: reduced={reduced} origin={case} rejected={} before_triangles={} after_triangles={}",
                lod.2.mesh_sample_rejected_instances,
                no_samples.2.mesh_submitted_triangles,
                lod.2.mesh_submitted_triangles
            );
            // Sample selection can remove every eligible proxy draw exactly;
            // retain the geometry-selection witness in the no-sample oracle.
            assert!(no_samples.2.mesh_lod_instances > 0);
            assert!(no_samples.2.mesh_lod_triangles_saved > 0);
            assert!(no_samples.2.mesh_lod_max_error_microtexels <= 500_000);
            assert!(lod.2.mesh_lod_max_error_microtexels <= 500_000);
            saved += no_samples.2.mesh_lod_triangles_saved;
            // A one-texel raster neighbourhood checks coverage in both directions;
            // this is measured sample coverage, not a general radiometric bound.
            let mask = |bytes: &[u8]| {
                bytes
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|p| f32::from_le_bytes(*p) < 1.0)
                    .collect::<Vec<_>>()
            };
            let a = mask(&baseline.0);
            let b = mask(&lod.0);
            let mut unmatched_indices = std::collections::BTreeSet::new();
            let mut differences = 0;
            let mut occupied_union = 0;
            for layer in 0..3 * cascades {
                if lights.colors_lux[layer / cascades][..3]
                    .iter()
                    .all(|&v| v == 0.0)
                {
                    continue;
                }
                for y in 0..512_i32 {
                    for x in 0..512_i32 {
                        let i = layer * 512 * 512 + (y * 512 + x) as usize;
                        occupied_union += usize::from(a[i] || b[i]);
                        if a[i] == b[i] {
                            continue;
                        }
                        differences += 1;
                        let target = if a[i] { &b } else { &a };
                        let found = (-1..=1).any(|dy| {
                            (-1..=1).any(|dx| {
                                let xx = x + dx;
                                let yy = y + dy;
                                (0..512).contains(&xx)
                                    && (0..512).contains(&yy)
                                    && target[layer * 512 * 512 + (yy * 512 + xx) as usize]
                            })
                        });
                        if !found {
                            unmatched_indices.insert(i);
                        }
                    }
                }
            }
            let unmatched = unmatched_indices.len();
            let mut max_component = 0;
            while let Some(seed) = unmatched_indices.pop_first() {
                let mut stack = vec![seed];
                let mut size = 0;
                while let Some(i) = stack.pop() {
                    size += 1;
                    let layer = i / (512 * 512);
                    let y = ((i / 512) % 512) as i32;
                    let x = (i % 512) as i32;
                    for dy in -1..=1 {
                        for dx in -1..=1 {
                            let xx = x + dx;
                            let yy = y + dy;
                            if (0..512).contains(&xx) && (0..512).contains(&yy) {
                                let j = layer * 512 * 512 + (yy * 512 + xx) as usize;
                                if unmatched_indices.remove(&j) {
                                    stack.push(j);
                                }
                            }
                        }
                    }
                }
                max_component = max_component.max(size);
            }
            // Subpixel laminae can miss all pixel centres after a bounded shift.
            // Reject larger missing regions and measure final filtered radiance too.
            eprintln!(
                "plant_shadow_fixture reeds={include_reeds} isolated={isolate_reeds} cascades={cascades} reduced={reduced} case={case} differences={differences} occupied={occupied_union} unmatched={unmatched} max_component={max_component}"
            );
            assert!(
                max_component <= 2,
                "missing component {max_component}, reduced={reduced}, origin={case}"
            );
            assert!(
                unmatched * 1000 <= occupied_union,
                "unmatched={unmatched} occupied={occupied_union}"
            );
            assert!(
                differences * 100 <= occupied_union,
                "coverage changes exceed 1%: differences={differences} union={occupied_union} reeds={include_reeds} reduced={reduced} case={case}"
            );
            let mut colour_max = 0_u8;
            let mut colour_sum = 0_u64;
            for (a, b) in baseline
                .1
                .as_chunks::<4>()
                .0
                .iter()
                .zip(lod.1.as_chunks::<4>().0.iter())
            {
                for c in 0..3 {
                    let d = a[c].abs_diff(b[c]);
                    colour_max = colour_max.max(d);
                    colour_sum += u64::from(d);
                }
            }
            let colour_mae = colour_sum as f64 / (257.0 * 193.0 * 3.0);
            // These fixture budgets allow filtered shadow-edge changes; the
            // independent geometric certificate remains at half a light texel.
            assert!(
                colour_max <= 16 && colour_mae <= 0.1,
                "colour max={colour_max} mae={colour_mae}"
            );
            if !reduced && case == 0 {
                for (extension, bytes) in [
                    ("full.depth", &baseline.0),
                    ("lod.depth", &lod.0),
                    ("full.rgba", &baseline.1),
                    ("lod.rgba", &lod.1),
                ] {
                    let mut f = std::fs::OpenOptions::new()
                        .write(true)
                        .create_new(true)
                        .open(output.with_extension(extension))
                        .unwrap();
                    use std::io::Write;
                    f.write_all(bytes).unwrap();
                }
            }
            changed += differences;
            // Validation failure must leave installed GPU geometry intact.
            let mut invalid = prepared.shadow_lods.clone();
            invalid.last_mut().unwrap().max_error_m = f64::NAN;
            assert!(mesh.set_shadow_lods(&backend, &invalid).is_err());
            let after_failure = capture(&mut terrain, &mesh, CaptureMode::Selected);
            assert_eq!(lod.0, after_failure.0);
            let mut expected_stats = lod.2;
            expected_stats.mesh_indirect_upload_bytes = 0;
            assert_eq!(expected_stats, after_failure.2);
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_shadows(&mut encoder, Some(&mesh));
            assert_eq!(terrain.last_shadow_passes(), 0);
            let table: toml::Table = [
                ("reduced", toml::Value::Boolean(reduced)),
                ("sample_culling_exact", toml::Value::Boolean(true)),
                (
                    "no_sample_lod_instances",
                    toml::Value::Integer(no_samples.2.mesh_lod_instances as i64),
                ),
                (
                    "no_sample_lod_triangles_saved",
                    toml::Value::Integer(no_samples.2.mesh_lod_triangles_saved as i64),
                ),
                (
                    "no_sample_max_error_microtexels",
                    toml::Value::Integer(no_samples.2.mesh_lod_max_error_microtexels as i64),
                ),
                (
                    "sample_rejected_instances",
                    toml::Value::Integer(lod.2.mesh_sample_rejected_instances as i64),
                ),
                (
                    "no_sample_triangles",
                    toml::Value::Integer(no_samples.2.mesh_submitted_triangles as i64),
                ),
                (
                    "sample_triangles",
                    toml::Value::Integer(lod.2.mesh_submitted_triangles as i64),
                ),
                ("origin_case", toml::Value::Integer(case as i64)),
                (
                    "different_coverage_texels",
                    toml::Value::Integer(differences as i64),
                ),
                (
                    "unmatched_after_one_texel",
                    toml::Value::Integer(unmatched as i64),
                ),
                (
                    "occupied_union",
                    toml::Value::Integer(occupied_union as i64),
                ),
                (
                    "max_unmatched_component",
                    toml::Value::Integer(max_component as i64),
                ),
                ("colour_max_u8", toml::Value::Integer(i64::from(colour_max))),
                ("colour_mae_u8", toml::Value::Float(colour_mae)),
                (
                    "triangles_saved",
                    toml::Value::Integer(lod.2.mesh_lod_triangles_saved as i64),
                ),
                (
                    "max_error_microtexels",
                    toml::Value::Integer(lod.2.mesh_lod_max_error_microtexels as i64),
                ),
            ]
            .into_iter()
            .map(|(k, v)| (k.into(), v))
            .collect();
            cases.push(toml::Value::Table(table));
            if physical && case == 0 {
                let gpu = thessa_render_wgpu::timing::GpuIntervals::new(&backend, 2)
                    .expect("physical timing capability");
                for (block, sample_culling) in [false, true, false, true, true, false, true, false]
                    .into_iter()
                    .enumerate()
                {
                    let mut gpu_ms = Vec::new();
                    let mut wall_ms = Vec::new();
                    for sample in 0..30 {
                        let start = std::time::Instant::now();
                        let mut encoder =
                            backend.device.create_command_encoder(&Default::default());
                        gpu.mark(&mut encoder, 0);
                        if sample_culling {
                            terrain.invalidate_shadows();
                            terrain.encode_shadows(&mut encoder, Some(&mesh));
                        } else {
                            terrain
                                .encode_shadows_without_sample_culling(&mut encoder, Some(&mesh));
                        }
                        assert!(
                            terrain.last_shadow_draws().mesh_submitted_triangles <= 4_000_000,
                            "reject oversized diagnostic before submission"
                        );
                        gpu.mark(&mut encoder, 1);
                        gpu.resolve(&mut encoder);
                        backend.queue.submit([encoder.finish()]);
                        backend
                            .device
                            .poll(wgpu::PollType::wait_indefinitely())
                            .unwrap();
                        let elapsed = start.elapsed().as_secs_f64() * 1000.0;
                        let time = gpu.read(&backend).unwrap()[0] * 1000.0;
                        if sample >= 5 {
                            assert!(
                                time < 50.0,
                                "stop diagnostic on unexpected long GPU interval"
                            );
                            gpu_ms.push(time);
                            wall_ms.push(elapsed);
                        }
                    }
                    gpu_ms.sort_by(f64::total_cmp);
                    wall_ms.sort_by(f64::total_cmp);
                    let table: toml::Table = [
                        ("reduced", toml::Value::Boolean(reduced)),
                        ("sample_culling", toml::Value::Boolean(sample_culling)),
                        ("block", toml::Value::Integer(block as i64)),
                        ("samples", toml::Value::Integer(25)),
                        ("gpu_p50_ms", toml::Value::Float(gpu_ms[12])),
                        ("gpu_p95_ms", toml::Value::Float(gpu_ms[23])),
                        ("wall_p50_ms", toml::Value::Float(wall_ms[12])),
                    ]
                    .into_iter()
                    .map(|(k, v)| (k.into(), v))
                    .collect();
                    eprintln!(
                        "physical shadow cost reduced={reduced} sample_culling={sample_culling} block={block} gpu_p50_ms={} wall_p50_ms={}",
                        gpu_ms[12], wall_ms[12]
                    );
                    timings.push(toml::Value::Table(table));
                }
            }
            lights.colors_lux[1] = [1.0; 4];
        }
    }
    assert!(
        saved > 0 && (include_reeds || changed > 0),
        "must witness an actual approximation"
    );
    let mut report: toml::Table = [("scope".into(),toml::Value::String("Actual native plant forms and a ground receiver; separate reed case isolates reed proxy with other sources full; CPU-only or explicitly authorized physical Vulkan, as recorded. Two-sided raster sample comparison with at most two connected unmatched texels, <0.1% unmatched and <1% changed coverage; final colour including the ground receiver max<=16/255 and mean<=0.1/255 in this fixture. Full oracle exact, primary unchanged, invalid update and cache checks. No whole-frame or general radiometric acceptance.".into())), ("cases".into(),toml::Value::Array(cases))].into_iter().collect();
    report.insert("timings".into(), toml::Value::Array(timings));
    report.insert("physical_gpu".into(), toml::Value::Boolean(physical));
    report.insert(
        "reference".into(),
        toml::Value::String(
            if include_reeds && !isolate_reeds {
                "original-one-tier"
            } else {
                "full-geometry"
            }
            .into(),
        ),
    );
    report.insert("includes_reeds".into(), toml::Value::Boolean(include_reeds));
    report.insert("isolated_reeds".into(), toml::Value::Boolean(isolate_reeds));
    report.insert("cascades".into(), toml::Value::Integer(cascades as i64));
    report.insert(
        "shadow_distance_m".into(),
        toml::Value::Float(request.shadows.max_distance_m),
    );
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)
        .unwrap();
    file.write_all(toml::to_string_pretty(&report).unwrap().as_bytes())
        .unwrap();
    eprintln!("{}", toml::to_string_pretty(&report).unwrap());
}
