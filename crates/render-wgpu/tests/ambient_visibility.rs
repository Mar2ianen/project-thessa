use glam::{DVec3, Vec3};
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_render_core::mesh::{AlphaMode, Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_render_wgpu::{
    Backend,
    ambient_visibility::AmbientVisibility,
    display::DisplayTransform,
    mesh::{MeshRenderer, SurfaceSettings},
    msaa::SceneMsaa,
    terrain::OffscreenTarget,
    view::{ViewParams, ViewUniformBlock},
};

fn fixture(transmission: f32, emission: [f32; 3]) -> MeshAsset {
    let mut asset = MeshAsset {
        materials: vec![
            Material {
                base_color: [0.5, 0.5, 0.5, 1.],
                metallic: 0.,
                double_sided: true,
                diffuse_transmission: transmission,
                emission_rgb: emission,
                ..Default::default()
            },
            Material {
                base_color: [0.4, 0.4, 0.4, 1.],
                metallic: 0.,
                double_sided: true,
                alpha_mode: AlphaMode::Mask,
                ..Default::default()
            },
        ],
        primitives: vec![],
        images: vec![],
        textures: vec![],
    };
    let mut quad = |p: [[f32; 3]; 4], normal: [f32; 3], material| {
        asset.primitives.push(Primitive {
            ambient_visibility: vec![],
            vertices: p
                .into_iter()
                .map(|position| Vertex {
                    position,
                    normal,
                    uv: [0.; 2],
                })
                .collect(),
            indices: vec![0, 1, 2, 0, 2, 3],
            material,
        })
    };
    quad(
        [[-4., 0., -4.], [-4., 0., 4.], [4., 0., 4.], [4., 0., -4.]],
        [0., 1., 0.],
        0,
    );
    for sign in [-1., 1.] {
        let s = sign * 0.5;
        quad(
            [[s, 0., -0.5], [s, 1., -0.5], [s, 1., 0.5], [s, 0., 0.5]],
            [sign, 0., 0.],
            1,
        );
        quad(
            [[-0.5, 0., s], [0.5, 0., s], [0.5, 1., s], [-0.5, 1., s]],
            [0., 0., sign],
            1,
        );
    }
    quad(
        [
            [-0.5, 1., -0.5],
            [-0.5, 1., 0.5],
            [0.5, 1., 0.5],
            [0.5, 1., -0.5],
        ],
        [0., 1., 0.],
        1,
    );
    asset
}
fn half(v: u16) -> f32 {
    let sign = if v & 0x8000 != 0 { -1. } else { 1. };
    let exponent = (v >> 10) & 31;
    let mantissa = f32::from(v & 1023);
    sign * if exponent == 0 {
        mantissa * 2f32.powi(-24)
    } else {
        (1. + mantissa / 1024.) * 2f32.powi(i32::from(exponent) - 15)
    }
}
fn rgba(bytes: &[u8], pixel: usize) -> [f32; 4] {
    std::array::from_fn(|i| {
        half(u16::from_le_bytes(
            bytes[pixel * 8 + i * 2..pixel * 8 + i * 2 + 2]
                .try_into()
                .unwrap(),
        ))
    })
}

// Independent finite-range cosine integral of the fixture's closed box.
fn oracle(p: Vec3, radius: f32) -> f32 {
    let mut blocked = 0.;
    let count = 65536u32;
    for i in 0..count {
        let u = (i as f64 + 0.5) / f64::from(count);
        let phi = std::f64::consts::TAU * f64::from(i.reverse_bits()) / 4294967296.;
        let direction = DVec3::new(u.sqrt() * phi.cos(), (1. - u).sqrt(), u.sqrt() * phi.sin());
        let origin = p.as_dvec3() + DVec3::Y * 0.002;
        let mut near = 0f64;
        let mut far = f64::from(radius);
        for axis in 0..3 {
            let lo = if axis == 1 { 0. } else { -0.5 };
            let hi = if axis == 1 { 1. } else { 0.5 };
            let a = (lo - origin[axis]) / direction[axis];
            let b = (hi - origin[axis]) / direction[axis];
            near = near.max(a.min(b));
            far = far.min(a.max(b));
        }
        if far >= near && near > 0. {
            blocked += 1. - (near / f64::from(radius)).powi(2);
        }
    }
    (blocked / f64::from(count)) as f32
}

#[test]
#[ignore = "GPU: bounded current-frame diffuse-sky visibility and independent box oracle"]
fn diffuse_sky_visibility_preserves_other_light_and_round_trips_with_identical_depth() {
    let backend = Backend::bring_up().unwrap();
    let extent = [129, 129];
    let eye = DVec3::new(2., 2., 4.);
    let view = ViewParams {
        world_position_m: eye,
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            eye,
            DVec3::new(0., 0., 0.),
            DVec3::Y,
        )
        .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 100.),
        exposure: 1.,
    };
    let camera = ViewUniformBlock::new(&view).unwrap();
    for (samples, probes) in [(1, 8), (1, 16), (1, 32), (4, 8), (4, 16), (4, 32)] {
        let settings = ResolvedGraphicsSettings {
            ambient_visibility_samples: probes,
            ambient_visibility_radius_m: 2.,
            raster_msaa_samples: samples,
            hdr: true,
            shadow_enabled: false,
            specular_reflections: false,
            ..Default::default()
        };
        let off_settings = ResolvedGraphicsSettings {
            ambient_visibility_samples: 0,
            ..settings.clone()
        };
        assert!(
            AmbientVisibility::new(&backend, &off_settings, [0, 0])
                .unwrap()
                .is_none()
        );
        let target = OffscreenTarget::new(&backend, extent[0], extent[1]).unwrap();
        let display = DisplayTransform::new(
            &backend,
            &settings,
            OffscreenTarget::FORMAT,
            extent[0],
            extent[1],
        )
        .unwrap();
        let msaa = SceneMsaa::new(&backend, &settings, display.scene_format(), extent).unwrap();
        let mut visibility = AmbientVisibility::new(&backend, &settings, extent)
            .unwrap()
            .unwrap();
        assert_eq!(
            visibility.target_bytes(),
            129 * 129 * 8 * if samples == 4 { 5 } else { 1 } + 144
        );
        for (scenario, transmission, emission, ambient, direct) in [
            ("sky", 0., [0.; 3], std::f32::consts::PI, 0.),
            ("baked", 0., [0.; 3], std::f32::consts::PI, 0.),
            ("direct_emission", 0., [0.3, 0.7, 1.2], 0., 3.),
            ("back_transmission", 1., [0.; 3], std::f32::consts::PI, 0.),
        ] {
            let mut asset = fixture(transmission, emission);
            if scenario == "baked" {
                asset.primitives[0].ambient_visibility = vec![0.25; 4];
            }
            let make = |settings: &ResolvedGraphicsSettings| {
                MeshRenderer::with_surface(
                    &backend,
                    &asset,
                    SurfaceSettings {
                        format: display.scene_format(),
                        capacity: 1,
                        quality: Quality::High,
                        reflections: false,
                        shadows: None,
                        ray_tracing: Some(settings),
                    },
                )
                .unwrap()
            };
            let mut mesh = make(&settings);
            let mut old = make(&off_settings);
            let lights = thessa_rcbt_core::lighting::SurfaceLighting {
                ambient_lux: [ambient, ambient, ambient, 0.],
                directions: [[0., 1., 0., 0.], [0.; 4], [0.; 4]],
                colors_lux: [[direct, direct, direct, 0.], [0.; 4], [0.; 4]],
            };
            let instances = [MeshInstance {
                position_m: DVec3::ZERO,
                rotation: glam::DQuat::IDENTITY,
                scale: 1.,
            }];
            mesh.update(&backend, &view, &lights, &instances).unwrap();
            old.update(&backend, &view, &lights, &instances).unwrap();
            let draw = |mesh: &MeshRenderer,
                        visibility: Option<&mut AmbientVisibility>,
                        enabled: bool| {
                let color = msaa
                    .as_ref()
                    .map_or(display.scene_color(&target.color), |s| &s.color);
                let depth = msaa.as_ref().map_or(&target.depth, SceneMsaa::depth);
                let resolve = msaa.as_ref().map(|_| display.scene_color(&target.color));
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                {
                    let mut attachments = vec![Some(wgpu::RenderPassColorAttachment {
                        view: color,
                        depth_slice: None,
                        resolve_target: resolve,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })];
                    if let Some(v) = visibility.as_ref() {
                        attachments.push(Some(v.attachment()));
                    }
                    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        color_attachments: &attachments,
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: depth,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        ..Default::default()
                    });
                    mesh.draw_opaque_into_pass(&mut pass);
                }
                if let Some(msaa) = &msaa {
                    msaa.resolve_depth(&mut encoder, &target.depth);
                }
                if let Some(v) = visibility {
                    v.encode(
                        &backend,
                        &mut encoder,
                        color,
                        resolve,
                        &target.depth,
                        msaa.as_ref().map(SceneMsaa::depth),
                        &view,
                        thessa_render_core::temporal::JitterOffset::ZERO,
                        enabled,
                    )
                    .unwrap();
                }
                display.encode(&mut encoder, &target.color);
                backend.queue.submit([encoder.finish()]);
                (
                    thessa_render_wgpu::readback::read_color(
                        &backend,
                        display.scene_texture().unwrap(),
                    )
                    .unwrap(),
                    thessa_render_wgpu::readback::read_depth(&backend, &target.depth_texture)
                        .unwrap(),
                    target.read_rgba8(&backend).unwrap(),
                )
            };
            let baseline = draw(&old, None, false);
            let a = draw(&mesh, Some(&mut visibility), false);
            let b = draw(&mesh, Some(&mut visibility), true);
            let c = draw(&mesh, Some(&mut visibility), false);
            assert!(
                baseline.0 == a.0,
                "exporting the receiver must not change beauty: {scenario} MSAA{samples}"
            );
            assert!(a.0 == c.0, "Off/On/Off round trip: {scenario}");
            assert!(
                a.1 == b.1 && b.1 == c.1,
                "visibility cannot modify scene depth"
            );
            if scenario != "sky" {
                // Box still has diffuse sky; compare only the floor receiver.
                let q = camera
                    .clip_from_world
                    .project_point3(Vec3::new(0., 0., 0.85) - eye.as_vec3());
                let x = ((q.x * 0.5 + 0.5) * 129.) as usize;
                let y = ((-q.y * 0.5 + 0.5) * 129.) as usize;
                let pixel = y * 129 + x;
                assert_eq!(
                    rgba(&a.0, pixel),
                    rgba(&b.0, pixel),
                    "direct/emission/back transmission are outside this visibility term"
                );
            } else {
                let receiver =
                    thessa_render_wgpu::readback::read_color(&backend, &visibility.texture)
                        .unwrap();
                let q = camera
                    .clip_from_world
                    .project_point3(Vec3::new(0., 0., 0.85) - eye.as_vec3());
                let x = ((q.x * 0.5 + 0.5) * 129.) as usize;
                let y = ((-q.y * 0.5 + 0.5) * 129.) as usize;
                let pixel = y * 129 + x;
                let depth = f32::from_le_bytes(a.1[pixel * 4..pixel * 4 + 4].try_into().unwrap());
                let ndc = Vec3::new(
                    (x as f32 + 0.5) / 129. * 2. - 1.,
                    1. - (y as f32 + 0.5) / 129. * 2.,
                    depth,
                );
                let p = camera.world_from_clip.project_point3(ndc) + eye.as_vec3();
                // Nearest covered MSAA depth belongs to a subpixel sample, not
                // necessarily the pixel centre used by the single-depth pass.
                let adjacent = camera
                    .world_from_clip
                    .project_point3(ndc + Vec3::X * (2. / 129.))
                    + eye.as_vec3();
                assert!(
                    p.y.abs() < (adjacent - p).length(),
                    "oracle receiver must remain within one footprint of the floor: {p}"
                );
                let component = rgba(&receiver, pixel)[0];
                let actual = (rgba(&a.0, pixel)[0] - rgba(&b.0, pixel)[0]) / component;
                let reference = oracle(Vec3::new(p.x, 0., p.z), 2.);
                eprintln!(
                    "ambient_visibility MSAA{samples} probes={probes} actual={actual:.5} finite_box_oracle={reference:.5} receiver={p:?}"
                );
                assert!(
                    actual > 0.03 && (actual - reference).abs() < 0.06,
                    "bounded screen-space approximation must occlude the contact and remain near the independent oracle"
                );
                if let Some(directory) = std::env::var_os("THESSA_TEST_ARTIFACT_DIR") {
                    let out = std::path::PathBuf::from(directory)
                        .join(format!("fixture-msaa{samples}-q{probes}"));
                    for (name, frame) in [("off", &a), ("on", &b)] {
                        let file = std::fs::File::create(out.with_extension(format!("{name}.png")))
                            .unwrap();
                        let mut encoder = png::Encoder::new(file, 129, 129);
                        encoder.set_color(png::ColorType::Rgba);
                        encoder.set_depth(png::BitDepth::Eight);
                        encoder
                            .write_header()
                            .unwrap()
                            .write_image_data(&frame.2)
                            .unwrap();
                    }
                }
            }
        }
    }
}

#[test]
#[ignore = "GPU: regular Low terrain exports diffuse sky without an atmospheric field and retains depth"]
fn low_terrain_receiver_has_an_explicit_field_free_fallback() {
    use thessa_rcbt_core::{
        HeightPage,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer};
    let backend = Backend::bring_up().unwrap();
    let size = [65, 65];
    let mut settings = ResolvedGraphicsSettings {
        ambient_visibility_samples: 8,
        ambient_visibility_radius_m: 2.,
        surface_quality: Quality::Low,
        hdr: true,
        atmosphere_enabled: false,
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    settings.ocean.enabled = false;
    let height = HeightPage::bake(&[0.; 4], 2, 0.01).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), 1000.).unwrap(),
            page: &height,
        })
        .collect::<Vec<_>>();
    let mut terrain = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        wgpu::TextureFormat::Rgba16Float,
        TerrainCapacity {
            patches: 6,
            material_layers: 6,
            height_grid_size: 2,
        },
        Some(&settings),
    )
    .unwrap();
    let eye = DVec3::Z * 1200.;
    let view = ViewParams {
        world_position_m: eye,
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
            .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1., 1., 0.1, 1e5),
        exposure: 1.,
    };
    terrain.update_view(&backend, &view).unwrap();
    terrain
        .update_lighting(
            &backend,
            &thessa_rcbt_core::lighting::SurfaceLighting {
                ambient_lux: [1.; 4],
                directions: [[0.; 4]; 3],
                colors_lux: [[0.; 4]; 3],
            },
        )
        .unwrap();
    let target = OffscreenTarget::new(&backend, size[0], size[1]).unwrap();
    let display = DisplayTransform::new(
        &backend,
        &settings,
        OffscreenTarget::FORMAT,
        size[0],
        size[1],
    )
    .unwrap();
    let mut visibility = AmbientVisibility::new(&backend, &settings, size)
        .unwrap()
        .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode_surface(&mut encoder);
    // The receiver must retain depth even when no sky/motion pass requested it.
    terrain.encode_opaque_scene_with_ambient_ops(
        &mut encoder,
        display.scene_color(&target.color),
        &target.depth,
        None,
        false,
        None,
        Some(&visibility),
    );
    visibility
        .encode(
            &backend,
            &mut encoder,
            display.scene_color(&target.color),
            None,
            &target.depth,
            None,
            &view,
            thessa_render_core::temporal::JitterOffset::ZERO,
            true,
        )
        .unwrap();
    backend.queue.submit([encoder.finish()]);
    let depth = thessa_render_wgpu::readback::read_depth(&backend, &target.depth_texture).unwrap();
    assert!(
        depth
            .as_chunks::<4>()
            .0
            .iter()
            .all(|d| f32::from_le_bytes(*d) < 1.)
    );
    let receiver = thessa_render_wgpu::readback::read_color(&backend, &visibility.texture).unwrap();
    assert!(
        receiver
            .as_chunks::<8>()
            .0
            .iter()
            .all(|p| half(u16::from_le_bytes(p[..2].try_into().unwrap())) > 0.)
    );
}
