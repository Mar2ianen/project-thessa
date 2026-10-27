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
    assert!(!std::path::Path::new("/dev/dri").exists());
    let backend = Backend::bring_up().unwrap();
    assert_eq!(
        backend.adapter.get_info().device_type,
        wgpu::DeviceType::Cpu
    );
    let output = std::path::PathBuf::from(std::env::var_os("THESSA_WORKLOAD_OUTPUT").unwrap());
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.shadows.mesh_lod_error_texels = 0.5;
    request.shadows.cascades = 2;
    request.shadows.map_size = 512;
    request.shadows.max_distance_m = 256.0;
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
    ] {
        for form in 0..3 {
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
            let mut capture = |terrain: &mut TerrainRenderer, mesh: &MeshRenderer, full: bool| {
                terrain.invalidate_shadows();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                if full {
                    terrain.encode_shadows_reference(&mut encoder, Some(mesh));
                } else {
                    terrain.encode_shadows(&mut encoder, Some(mesh));
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
            let full = capture(&mut terrain, &mesh, false);
            mesh.set_shadow_lods(&backend, &prepared.shadow_lods)
                .unwrap();
            let reference = capture(&mut terrain, &mesh, true);
            assert_eq!(
                full.0, reference.0,
                "LOD installation changed full shadow oracle"
            );
            assert_eq!(
                full.1, reference.1,
                "LOD installation changed primary geometry"
            );
            let lod = capture(&mut terrain, &mesh, false);
            assert!(lod.2.mesh_lod_instances > 0);
            assert!(lod.2.mesh_lod_triangles_saved > 0);
            assert!(lod.2.mesh_lod_max_error_microtexels <= 500_000);
            saved += lod.2.mesh_lod_triangles_saved;
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
            let a = mask(&full.0);
            let b = mask(&lod.0);
            let mut unmatched_indices = std::collections::BTreeSet::new();
            let mut differences = 0;
            let mut occupied_union = 0;
            for layer in 0..6 {
                if lights.colors_lux[layer / 2][..3].iter().all(|&v| v == 0.0) {
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
                "coverage changes exceed 1% of this fixture"
            );
            let mut colour_max = 0_u8;
            let mut colour_sum = 0_u64;
            for (a, b) in full
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
                    ("full.depth", &full.0),
                    ("lod.depth", &lod.0),
                    ("full.rgba", &full.1),
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
            let after_failure = capture(&mut terrain, &mesh, false);
            assert_eq!(lod.0, after_failure.0);
            assert_eq!(lod.2, after_failure.2);
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_shadows(&mut encoder, Some(&mesh));
            assert_eq!(terrain.last_shadow_passes(), 0);
            let table: toml::Table = [
                ("reduced", toml::Value::Boolean(reduced)),
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
            lights.colors_lux[1] = [1.0; 4];
        }
    }
    assert!(
        changed > 0 && saved > 0,
        "must witness an actual approximation"
    );
    let report: toml::Table = [("scope".into(),toml::Value::String("Actual native full/reduced six-form plants and a ground receiver; software Vulkan only. Two-sided raster sample comparison with at most two connected unmatched texels, <0.1% unmatched and <1% changed coverage; final colour including the ground receiver max<=16/255 and mean<=0.1/255 in this fixture. Full oracle exact, primary unchanged, invalid update and cache checks. No target-GPU timing or general radiometric acceptance.".into())), ("cases".into(),toml::Value::Array(cases))].into_iter().collect();
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
