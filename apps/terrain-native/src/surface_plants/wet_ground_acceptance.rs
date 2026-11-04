//! Reviewer-owned geometry acceptance cases for native ground cover.
use super::*;

#[test]
fn curved_blades_reach_a_point_without_degenerate_tip_triangles() {
    let base = DVec3::ZERO;
    let length = 0.20;
    let lean = DVec3::X * 0.07;
    let tip = base + DVec3::Y * length + lean;
    for segments in [1, 2, 3, 4] {
        let mut mesh = blank(0);
        curved_blade(
            &mut mesh,
            base,
            BladePose {
                up: DVec3::Y,
                lean,
                side: DVec3::Z,
            },
            BladeShape {
                length,
                half_width: 0.008,
                segments,
                taper_start: None,
            },
            DVec3::ZERO,
        );
        assert!(
            mesh.vertices
                .iter()
                .any(|v| { DVec3::from(v.position.map(f64::from)).distance(tip) < 1e-7 }),
            "a blade must end at its centerline tip, without a blunt cross-section"
        );
        assert!(
            mesh.vertices.iter().any(|v| v.uv[1] >= 0.999),
            "blade UV.y records longitudinal position from root=0 to tip=1"
        );
        assert!(
            mesh.vertices
                .iter()
                .filter(|v| v.uv[1].abs() <= 1e-6)
                .all(|v| v.position[1].abs() <= 1e-7),
            "UV.y=0 must identify the actual root rather than a raised blade ring"
        );
        for tri in mesh.indices.as_chunks::<3>().0 {
            let p = tri.map(|i| DVec3::from(mesh.vertices[i as usize].position.map(f64::from)));
            assert!(
                (p[1] - p[0]).cross(p[2] - p[0]).length() > 1e-10,
                "collapsing a tip must not publish a zero-area triangle"
            );
        }
        assert!(mesh.vertices.iter().all(|v| {
            let n = DVec3::from(v.normal.map(f64::from));
            n.is_finite() && (n.length() - 1.0).abs() < 1e-5
        }));
    }
}

#[test]
fn near_camera_grass_roots_follow_the_same_soil_surface() {
    let eye = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818);
    let dir = eye.normalize();
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.terrain_ground_detail = true;
    request.renderer.surface_quality = thessa_graphics::Quality::High;
    let settings = ResolvedGraphicsSettings::from_requested(
        &request,
        &thessa_graphics::Capabilities::unknown(),
    );
    let (prepared, _) = prepare(
        Context::SurveyWithUnderstory {
            canopy: survey_window(dir),
            understory: understory_window(dir),
        },
        &settings,
        &field,
    )
    .unwrap();
    let prepared = prepared.unwrap();
    let mut roots = 0;
    let mut worst_float = f64::NEG_INFINITY;
    let mut worst_burial = f64::INFINITY;
    for (mesh, range) in prepared.asset.primitives.iter().zip(&prepared.ranges) {
        let material = &prepared.asset.materials[mesh.material];
        if !GRASS_PALETTES.contains(&material.base_color) {
            continue;
        }
        for instance in &prepared.instances[range.start as usize..range.end as usize] {
            if instance.position_m.distance(eye) > 10.0 {
                continue;
            }
            for vertex in mesh.vertices.iter().filter(|v| v.uv[1].abs() <= 1e-6) {
                let p = instance.position_m
                    + instance.rotation
                        * (DVec3::from(vertex.position.map(f64::from)) * instance.scale);
                let clearance = p.length() - ground_position(&field, p).length();
                worst_float = worst_float.max(clearance);
                worst_burial = worst_burial.min(clearance);
                roots += 1;
            }
        }
    }
    assert!(
        roots >= 100,
        "the contact fixture must inspect near-camera roots"
    );
    assert!(
        worst_float <= 0.006 && worst_burial >= -0.020,
        "root clearance must be within -20..6 mm, sampled={roots}, min={worst_burial}, max={worst_float}"
    );
}

#[test]
fn moss_does_not_publish_large_untextured_sheet_triangles() {
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let dir = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818).normalize();
    let (east, north, _) = enu_basis(dir.to_array());
    let center = dir * field.params.radius_m;
    let frame = GroundPatchFrame {
        field: &field,
        center,
        east: east.into(),
        north: north.into(),
        anchor: center,
    };
    let mut mesh = blank(0);
    moss_cushion(
        &mut mesh,
        &frame,
        [0.20, 0.17],
        0.08,
        0.37,
        field.params.seed,
        812,
    )
    .unwrap();
    assert!(
        mesh.indices.len() >= 48,
        "the fixture must contain resolved moss structure"
    );
    let largest_area = mesh
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|tri| {
            let p = tri.map(|i| DVec3::from(mesh.vertices[i as usize].position.map(f64::from)));
            (p[1] - p[0]).cross(p[2] - p[0]).length() * 0.5
        })
        .fold(0.0_f64, f64::max);
    assert!(
        largest_area <= 0.0008,
        "untextured moss needs leaf-scale structure, not broad green facets: largest area={largest_area} m²"
    );
    let tallest = mesh
        .vertices
        .iter()
        .map(|v| {
            let p = center + DVec3::from(v.position.map(f64::from));
            p.length() - ground_position(&field, p).length()
        })
        .fold(0.0_f64, f64::max);
    assert!(
        (0.015..=0.12).contains(&tallest),
        "moss must have bounded volume: {tallest} m"
    );
}

#[test]
#[ignore = "small actual-GPU moss macro specimen; optional THESSA_TEST_ARTIFACT_DIR"]
fn capture_textured_moss_macro_and_verify_cutout_coverage() {
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let dir = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818).normalize();
    let (east, north, _) = enu_basis(dir.to_array());
    let center = ground_position(&field, dir * field.params.radius_m);
    let frame = GroundPatchFrame {
        field: &field,
        center,
        east: east.into(),
        north: north.into(),
        anchor: center,
    };
    let mut prepared = Prepared {
        asset: MeshAsset {
            primitives: vec![],
            materials: vec![],
            images: vec![],
            textures: vec![],
        },
        shadow_lods: vec![],
        descriptors: vec![],
        instances: vec![],
        ranges: vec![],
        reduced_primitives: vec![],
        local_ground_start: None,
    };
    let material = super::super::moss::materials(&mut prepared).unwrap()[0];
    let mut primitive = blank(material);
    moss_cushion(
        &mut primitive,
        &frame,
        [0.10, 0.085],
        0.08,
        0.37,
        field.params.seed,
        812,
    )
    .unwrap();
    let local = DMat3::from_cols(east.into(), dir, -DVec3::from(north)).transpose();
    for v in &mut primitive.vertices {
        v.position = (local * DVec3::from(v.position.map(f64::from)))
            .as_vec3()
            .to_array();
        v.normal = (local * DVec3::from(v.normal.map(f64::from)))
            .as_vec3()
            .to_array();
    }
    prepared.asset.primitives.push(primitive);
    prepared.asset.validate().unwrap();
    let images = render_ground_specimen(
        &mut prepared.asset,
        DVec3::new(0.12, 0.16, 0.36),
        DVec3::new(0.0, 0.02, 0.0),
        &[
            thessa_render_core::mesh::AlphaMode::Mask,
            thessa_render_core::mesh::AlphaMode::Opaque,
        ],
    );
    let background = &images[0][..3];
    let visible = images[0]
        .as_chunks::<4>()
        .0
        .iter()
        .filter(|p| {
            p[..3]
                .iter()
                .zip(background)
                .any(|(a, b)| a.abs_diff(*b) > 8)
        })
        .count();
    assert!(
        visible > 500,
        "cutout must retain actual moss, visible pixels={visible}"
    );
    let changed = images[0]
        .as_chunks::<4>()
        .0
        .iter()
        .zip(images[1].as_chunks::<4>().0)
        .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 8))
        .count();
    assert!(
        changed > 500,
        "alpha mask must remove rectangular carrier pixels: {changed}"
    );
    eprintln!("actual moss macro: 24 shoots, cutout_vs_opaque_changed_pixels={changed}");
    if let Some(dir) = std::env::var_os("THESSA_TEST_ARTIFACT_DIR") {
        let dir = std::path::PathBuf::from(dir);
        std::fs::create_dir_all(&dir).unwrap();
        for (name, data) in ["cutout", "opaque-control"].into_iter().zip(images) {
            let mut png = png::Encoder::new(
                std::fs::File::create(dir.join(format!("moss-{name}.png"))).unwrap(),
                640,
                480,
            );
            png.set_color(png::ColorType::Rgba);
            png.set_depth(png::BitDepth::Eight);
            png.write_header().unwrap().write_image_data(&data).unwrap();
        }
    }
}

#[test]
fn fallen_fronds_have_opposed_pinnae_and_follow_uneven_canonical_soil() {
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let dir = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818).normalize();
    let center = ground_position(&field, dir * field.params.radius_m);
    let (east, _, _) = enu_basis(dir.to_array());
    let mut mesh = blank(0);
    let source = &super::super::litter::source().unwrap().primitives[0];
    super::super::litter::append_frond(
        &mut mesh,
        &field,
        source,
        center,
        DVec3::from(east),
        2.1,
        center,
    );
    assert!(
        mesh.indices.len() >= source.indices.len()
            && mesh.indices.len() <= source.indices.len() * 64
    );
    assert!(
        source
            .vertices
            .iter()
            .all(|s| mesh.vertices.iter().any(|v| v.uv == s.uv)),
        "source atlas coordinates must survive conformation"
    );
    let mut min_height = f64::INFINITY;
    let mut max_height = f64::NEG_INFINITY;
    let min_curl = source
        .vertices
        .iter()
        .map(|v| v.position[1])
        .fold(f32::INFINITY, f32::min);
    let max_curl = source
        .vertices
        .iter()
        .map(|v| v.position[1])
        .fold(f32::NEG_INFINITY, f32::max);
    let max_clearance = f64::from(max_curl - min_curl) * 2.1 + 0.002;
    assert!(max_clearance > 0.020, "preserve volumetric source curl");
    assert!(
        mesh.vertices
            .iter()
            .any(|v| DVec3::from(v.normal.map(f64::from)).dot(dir).abs() < 0.95),
        "source shading normals must not collapse to radial-up"
    );
    for vertex in &mesh.vertices {
        let p = center + DVec3::from(vertex.position.map(f64::from));
        let soil = ground_position(&field, p);
        let clearance = p.length() - soil.length();
        assert!(
            (0.0005..max_clearance).contains(&clearance),
            "litter must stay on soil, clearance={clearance}"
        );
        min_height = min_height.min(soil.length());
        max_height = max_height.max(soil.length());
    }
    assert!(
        max_height - min_height > 0.01,
        "fixture must exercise uneven soil, not a flat carrier"
    );
    for tri in mesh.indices.as_chunks::<3>().0 {
        let p = tri.map(|i| DVec3::from(mesh.vertices[i as usize].position.map(f64::from)));
        assert!((p[1] - p[0]).cross(p[2] - p[0]).length() > 1e-9);
    }
}

fn render_ground_specimen(
    asset: &mut MeshAsset,
    eye: DVec3,
    focus: DVec3,
    modes: &[thessa_render_core::mesh::AlphaMode],
) -> Vec<Vec<u8>> {
    render_ground_specimen_quality(asset, eye, focus, modes, thessa_graphics::Quality::Low)
}

fn render_ground_specimen_quality(
    asset: &mut MeshAsset,
    eye: DVec3,
    focus: DVec3,
    modes: &[thessa_render_core::mesh::AlphaMode],
    quality: thessa_graphics::Quality,
) -> Vec<Vec<u8>> {
    use thessa_rcbt_core::lighting::SurfaceLighting;
    use thessa_render_wgpu::{
        Backend,
        mesh::{MeshRenderer, SurfaceSettings},
        terrain::OffscreenTarget,
        view::ViewParams,
    };
    asset.validate().unwrap();
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 640, 480).unwrap();
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, focus, DVec3::Y).inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
            0.8,
            640.0 / 480.0,
            0.005,
            10.0,
        ),
        world_position_m: eye,
        exposure: 1.0,
    };
    let lights = SurfaceLighting {
        ambient_lux: [std::f32::consts::PI; 4],
        directions: [[0.3, 0.8, 0.51961523, 0.0], [0.0; 4], [0.0; 4]],
        colors_lux: [[2.0, 2.0, 2.0, 0.0], [0.0; 4], [0.0; 4]],
    };
    // High appearance frames use the real shared caster/receiver pass. The Low
    // coverage oracle deliberately keeps its original unshadowed illumination.
    let shadow_support = (quality == thessa_graphics::Quality::High).then(|| {
        use thessa_rcbt_core::{
            HeightPage,
            tile_anchor::{TileAnchor, TileKey},
        };
        use thessa_render_wgpu::terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer};
        let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let patch = TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(0, 0, 0, 0).unwrap(), 1000.0).unwrap(),
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
            None,
        )
        .unwrap();
        terrain.publish_partition(&backend, &[]).unwrap();
        let mut requested = thessa_graphics::RequestedGraphics::default();
        requested.shadows.cascades = 1;
        requested.shadows.map_size = 2048;
        requested.shadows.normal_bias_m = 0.001;
        requested.shadows.quality = thessa_graphics::Quality::High;
        let mut settings = backend.resolve_graphics(&requested);
        // Specimen-only metre scale; production shadow ranges are unchanged.
        settings.shadow_max_distance_m = (eye.length() * 1.5).max(1.0);
        let mut atlas = thessa_render_wgpu::shadows::ShadowAtlas::new(&backend, &settings)
            .unwrap()
            .unwrap();
        atlas.update(&backend, focus, &lights).unwrap();
        (terrain, atlas)
    });
    let mut images = Vec::new();
    for mode in modes {
        for m in &mut asset.materials {
            m.alpha_mode = *mode;
        }
        let mut renderer = MeshRenderer::with_surface(
            &backend,
            asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality,
                reflections: quality == thessa_graphics::Quality::High,
                shadows: shadow_support.as_ref().map(|(_, atlas)| atlas),
                ray_tracing: None,
            },
        )
        .unwrap();
        renderer
            .update(
                &backend,
                &view,
                &lights,
                &[MeshInstance {
                    position_m: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    scale: 1.0,
                }],
            )
            .unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        if let Some((terrain, atlas)) = &shadow_support {
            atlas.encode(&mut encoder, terrain, Some(&renderer));
        }
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("ground detail specimen"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &target.color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.06,
                            g: 0.065,
                            b: 0.07,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &target.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
        renderer.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
        images.push(target.read_rgba8(&backend).unwrap());
    }
    images
}

#[test]
#[ignore = "small actual-GPU dead-frond specimen on canonical reference soil"]
fn capture_fallen_frond_on_canonical_reference_soil() {
    for form in 0..3 {
        for length in [0.4, 2.1, 6.0] {
            capture_fallen_frond_case(form, length);
        }
    }
}

#[test]
#[ignore = "actual-GPU comparison of dead-frond mapped and geometric normals"]
fn capture_fallen_frond_material_controls() {
    for form in 0..3 {
        capture_fallen_frond_case(form, 2.1);
    }
}

fn capture_fallen_frond_case(form: usize, length: f64) {
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let dir = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818).normalize();
    let (east, north, _) = enu_basis(dir.to_array());
    let center = ground_position(&field, dir * field.params.radius_m);
    let local = DMat3::from_cols(east.into(), dir, -DVec3::from(north)).transpose();
    let mut litter = blank(0);
    let source = super::super::litter::source().unwrap();
    super::super::litter::append_frond(
        &mut litter,
        &field,
        &source.primitives[form],
        center,
        DVec3::from(east),
        length,
        center,
    );
    let mut soil = blank(1);
    let span = 3.0 * length / 2.1;
    let side = (span / 0.046875).ceil() as u32 + 1;
    for y in 0..side {
        for x in 0..side {
            let point = center
                + DVec3::from(east) * (x as f64 / (side - 1) as f64 * span - span * 0.5)
                - DVec3::from(north) * (y as f64 / (side - 1) as f64 * span - span * 0.5);
            soil.vertices.push(Vertex {
                position: (ground_position(&field, point) - center)
                    .as_vec3()
                    .to_array(),
                normal: dir.as_vec3().to_array(),
                uv: [0.0; 2],
            });
        }
    }
    for y in 0..side - 1 {
        for x in 0..side - 1 {
            let a = y * side + x;
            soil.indices
                .extend([a, a + side, a + 1, a + 1, a + side, a + side + 1]);
        }
    }
    for mesh in [&mut litter, &mut soil] {
        for v in &mut mesh.vertices {
            v.position = (local * DVec3::from(v.position.map(f64::from)))
                .as_vec3()
                .to_array();
            v.normal = (local * DVec3::from(v.normal.map(f64::from)))
                .as_vec3()
                .to_array();
        }
    }
    let mut asset = source.clone();
    let mut dry = asset.materials[0].clone();
    dry.metallic = 0.0;
    dry.roughness = 0.85;
    asset.materials = vec![
        dry,
        Material {
            base_color: [0.05, 0.06, 0.055, 1.0],
            metallic: 0.0,
            roughness: 0.95,
            double_sided: true,
            ..Default::default()
        },
    ];
    asset.primitives = vec![litter, soil];
    let images = render_ground_specimen(
        &mut asset,
        DVec3::new(0.1, 3.0, 0.35) * (length / 2.1),
        DVec3::new(0.0, 0.02, 0.0),
        &[thessa_render_core::mesh::AlphaMode::Mask],
    );
    let appearance = render_ground_specimen_quality(
        &mut asset,
        DVec3::new(0.1, 2.1, 1.6) * (length / 2.1),
        DVec3::new(0.0, 0.02, 0.0),
        &[thessa_render_core::mesh::AlphaMode::Mask],
        thessa_graphics::Quality::High,
    );
    let contact_appearance = render_ground_specimen_quality(
        &mut asset,
        DVec3::new(0.1, 0.45, 2.1) * (length / 2.1),
        DVec3::new(0.0, 0.02, 0.0),
        &[thessa_render_core::mesh::AlphaMode::Mask],
        thessa_graphics::Quality::High,
    );
    let normal_texture = asset.materials[0].normal_texture.take();
    let geometric_appearance = render_ground_specimen_quality(
        &mut asset,
        DVec3::new(0.1, 2.1, 1.6) * (length / 2.1),
        DVec3::new(0.0, 0.02, 0.0),
        &[thessa_render_core::mesh::AlphaMode::Mask],
        thessa_graphics::Quality::High,
    );
    asset.materials[0].normal_texture = normal_texture;
    asset.primitives.pop();
    let alone = render_ground_specimen(
        &mut asset,
        DVec3::new(0.1, 3.0, 0.35) * (length / 2.1),
        DVec3::new(0.0, 0.02, 0.0),
        &[thessa_render_core::mesh::AlphaMode::Mask],
    );
    let background = &alone[0][..3];
    let mut visible = 0;
    let mut retained = 0;
    for (solo, ground) in alone[0]
        .as_chunks::<4>()
        .0
        .iter()
        .zip(images[0].as_chunks::<4>().0)
    {
        if solo[..3]
            .iter()
            .zip(background)
            .any(|(a, b)| a.abs_diff(*b) > 8)
        {
            visible += 1;
            retained += usize::from(
                solo[..3]
                    .iter()
                    .zip(&ground[..3])
                    .all(|(a, b)| a.abs_diff(*b) <= 3),
            );
        }
    }
    eprintln!(
        "fallen_frond form={form} length={length} visible={visible} retained_over_reference_soil={retained}"
    );
    if let Some(directory) = std::env::var_os("THESSA_TEST_ARTIFACT_DIR") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        crate::write_rgba_png(
            &directory.join(format!(
                "fallen-frond-f{form}-l{length:.1}-high-contact.png"
            )),
            640,
            480,
            &contact_appearance[0],
        )
        .unwrap();
        crate::write_rgba_png(
            &directory.join(format!(
                "fallen-frond-f{form}-l{length:.1}-high-geometric-normals.png"
            )),
            640,
            480,
            &geometric_appearance[0],
        )
        .unwrap();
        crate::write_rgba_png(
            &directory.join(format!(
                "fallen-frond-f{form}-l{length:.1}-high-appearance.png"
            )),
            640,
            480,
            &appearance[0],
        )
        .unwrap();
        crate::write_rgba_png(
            &directory.join(format!("fallen-frond-f{form}-l{length:.1}-no-soil.png")),
            640,
            480,
            &alone[0],
        )
        .unwrap();
        crate::write_rgba_png(
            &directory.join(format!(
                "fallen-frond-f{form}-l{length:.1}-reference-soil.png"
            )),
            640,
            480,
            &images[0],
        )
        .unwrap();
    }
    assert!(
        visible > 500 && retained * 10 >= visible * 9,
        "frond carrier must not disappear into soil: visible={visible},retained={retained}"
    );
}
