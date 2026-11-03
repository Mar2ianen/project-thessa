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
    use thessa_rcbt_core::lighting::SurfaceLighting;
    use thessa_render_wgpu::{
        Backend, mesh::MeshRenderer, terrain::OffscreenTarget, view::ViewParams,
    };
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
    let backend = Backend::bring_up().unwrap();
    let target = OffscreenTarget::new(&backend, 640, 480).unwrap();
    let eye = DVec3::new(0.12, 0.16, 0.36);
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            eye,
            DVec3::new(0.0, 0.02, 0.0),
            DVec3::Y,
        )
        .inverse(),
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
    let mut images = Vec::new();
    for mode in [
        thessa_render_core::mesh::AlphaMode::Mask,
        thessa_render_core::mesh::AlphaMode::Opaque,
    ] {
        for m in &mut prepared.asset.materials {
            m.alpha_mode = mode;
        }
        let mut renderer =
            MeshRenderer::new(&backend, &prepared.asset, OffscreenTarget::FORMAT, 1).unwrap();
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
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("moss macro specimen"),
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
