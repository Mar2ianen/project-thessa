use glam::{DVec3, Vec3};
use thessa_graphics::{Quality, ResolvedGraphicsSettings};
use thessa_render_core::mesh::{Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_render_wgpu::{
    Backend,
    display::DisplayTransform,
    mesh::{MeshRenderer, SurfaceSettings},
    msaa::SceneMsaa,
    terrain::OffscreenTarget,
    view::{ViewParams, ViewUniformBlock},
};
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
#[test]
#[ignore = "GPU: MSAA subpixel leaf attributes stay in their physical convex domain"]
fn subpixel_triangle_does_not_extrapolate_negative_baked_visibility() {
    let backend = Backend::bring_up().unwrap();
    let extent = [65, 65];
    let settings = ResolvedGraphicsSettings {
        raster_msaa_samples: 4,
        hdr: true,
        shadow_enabled: false,
        specular_reflections: false,
        ..Default::default()
    };
    let eye = DVec3::new(0., 0., 3.);
    let view = ViewParams {
        world_position_m: eye,
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
            .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1., 1., 0.1, 100.),
        exposure: 1.,
    };
    let block = ViewUniformBlock::new(&view).unwrap();
    let vertices = [[-0.8, -0.014], [0.8, -0.014], [-0.8, -0.008]].map(|[x, y]| Vertex {
        position: (block.world_from_clip.project_point3(Vec3::new(x, y, 0.5)) + eye.as_vec3())
            .to_array(),
        normal: [0., 0., 1.],
        uv: [0., 0.],
    });
    let asset = MeshAsset {
        primitives: vec![Primitive {
            vertices: vertices.into(),
            indices: vec![0, 1, 2],
            material: 0,
            ambient_visibility: vec![1., 1., 0.],
        }],
        materials: vec![Material {
            metallic: 0.,
            double_sided: true,
            ..Default::default()
        }],
        images: vec![],
        textures: vec![],
    };
    let target = OffscreenTarget::new(&backend, extent[0], extent[1]).unwrap();
    let display = DisplayTransform::new(
        &backend,
        &settings,
        OffscreenTarget::FORMAT,
        extent[0],
        extent[1],
    )
    .unwrap();
    let msaa = SceneMsaa::new(&backend, &settings, display.scene_format(), extent)
        .unwrap()
        .unwrap();
    let mut mesh = MeshRenderer::with_surface(
        &backend,
        &asset,
        SurfaceSettings {
            format: display.scene_format(),
            capacity: 1,
            quality: Quality::Low,
            reflections: false,
            shadows: None,
            ray_tracing: Some(&settings),
        },
    )
    .unwrap();
    mesh.update(
        &backend,
        &view,
        &thessa_rcbt_core::lighting::SurfaceLighting {
            ambient_lux: [std::f32::consts::PI; 4],
            directions: [[0.; 4]; 3],
            colors_lux: [[0.; 4]; 3],
        },
        &[MeshInstance {
            position_m: DVec3::ZERO,
            rotation: glam::DQuat::IDENTITY,
            scale: 1.,
        }],
    )
    .unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &msaa.color,
                depth_slice: None,
                resolve_target: Some(display.scene_color(&target.color)),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: msaa.depth(),
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            ..Default::default()
        });
        mesh.draw_into_pass(&mut pass);
    }
    backend.queue.submit([encoder.finish()]);
    let raw = thessa_render_wgpu::readback::read_color(&backend, display.scene_texture().unwrap())
        .unwrap();
    let values: Vec<f32> = raw
        .as_chunks::<8>()
        .0
        .iter()
        .flat_map(|p| {
            (0..3).map(|i| half(u16::from_le_bytes(p[i * 2..i * 2 + 2].try_into().unwrap())))
        })
        .collect();
    let min = values.iter().copied().fold(f32::INFINITY, f32::min);
    let max = values.iter().copied().fold(0., f32::max);
    let negative = values.iter().filter(|v| **v < 0.).count();
    eprintln!("subpixel_leaf msaa=4 min={min} max={max} negative_channels={negative}");
    if let Some(directory) = std::env::var_os("THESSA_TEST_ARTIFACT_DIR") {
        std::fs::write(
            std::path::PathBuf::from(directory).join("centroid-fixture.rgba16f"),
            raw,
        )
        .unwrap();
    }
    assert!(
        min >= 0. && max <= 1.,
        "coverage interpolation must stay inside the [0,1] vertex visibility domain"
    );
    assert!(max > 0., "the subpixel leaf must cover actual samples");
}
