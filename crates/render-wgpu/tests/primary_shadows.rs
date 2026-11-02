//! Existing shared TLAS corrects primary ground visibility missed by the atlas.
use glam::{DQuat, DVec3};
use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::{ViewParams, mesh::*};
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
};

#[test]
#[ignore = "GPU: ray-query adapter, actual primary ground receives existing scene visibility"]
fn primary_ground_uses_shared_rays_with_exact_raster_fallback() {
    run_case(false);
    run_case(true);
}

fn run_case(water: bool) {
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Local;
    request.raytracing.primary_shadows = true;
    request.renderer.specular_reflections = false;
    request.renderer.surface_quality = Quality::Low;
    request.renderer.terrain_normal_maps = false;
    request.temporal.motion_vectors = water;
    request.ocean.enabled = water;
    request.ocean.quality = Quality::Low;
    request.ocean.geometry_patches = 0;
    request.ocean.significant_height_m = 0.0;
    request.shadows.enabled = true;
    request.shadows.cascades = 1;
    request.shadows.max_distance_m = 100.;
    request.shadows.map_size = 256;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    eprintln!("primary_shadow adapter={}", backend.adapter_name);
    let target = OffscreenTarget::new(&backend, 129, 129).unwrap();
    let anchor = TileAnchor::new(TileKey::new(0, 17, 65536, 65536).unwrap(), 1e6).unwrap();
    let center = DVec3::from_array(anchor.anchor_body_m);
    let up = center.normalize();
    let right = up.any_orthonormal_vector();
    let light_direction = (up + right).normalize();
    let normal = light_direction;
    let u = normal.any_orthonormal_vector() * 0.2;
    let v = normal.cross(u);
    let asset = MeshAsset {
        primitives: vec![Primitive {
            vertices: [-u - v, u - v, u + v, -u + v]
                .map(|p| Vertex {
                    position: p.as_vec3().to_array(),
                    normal: normal.as_vec3().to_array(),
                    uv: [0.; 2],
                })
                .to_vec(),
            indices: vec![0, 1, 2, 0, 2, 3],
            material: 0,
            ambient_visibility: vec![],
        }],
        materials: vec![Material {
            double_sided: true,
            metallic: 0.,
            ..Default::default()
        }],
        textures: vec![],
        images: vec![],
    };
    let camera = center + up * 10.;
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(camera, center, right).inverse(),
        world_position_m: camera,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1., 0.1, 1000.),
        exposure: 1.,
    };
    let page = HeightPage::bake(&[0.; 4], 2, 0.001).unwrap();
    let lights = SurfaceLighting {
        ambient_lux: [0.05; 4],
        directions: [
            light_direction.as_vec3().extend(0.).to_array(),
            (up - right).normalize().as_vec3().extend(0.).to_array(),
            [0.; 4],
        ],
        colors_lux: [
            [std::f32::consts::PI, 0., 0., 0.],
            [0., 0., std::f32::consts::PI, 0.],
            [0.; 4],
        ],
    };
    let mut outputs = Vec::new();
    for mode in [RayTracingRequest::Off, RayTracingRequest::Local] {
        request.renderer.ray_tracing = mode;
        let settings = backend.resolve_graphics(&request);
        let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
            &backend,
            &[TerrainPatch {
                anchor,
                page: &page,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 1,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
            water.then_some(thessa_render_core::ocean::SeaState {
                gravity_m_s2: 6.4,
                depth_m: 100.,
                wind_speed_m_s: 10.,
                wind_direction_rad: 0.,
                significant_height_m: 0.,
                seed: 17,
            }),
        )
        .unwrap();
        assert_eq!(
            terrain.primary_shadow_stats().allocated,
            mode == RayTracingRequest::Local
        );
        if water {
            let page = thessa_rcbt_core::material_page::CbtMaterialPage::from_rgba8(
                [100, 130, 70, 10].repeat(128 * 128),
            )
            .unwrap();
            terrain
                .update_materials(&backend, &[8], &[(8, (1, page))].into())
                .unwrap();
            terrain.update_ocean(&backend, 0.).unwrap();
            assert!(terrain.ocean_stats().size > 0);
        }
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_lighting(&backend, &lights).unwrap();
        terrain.update_shadows(&backend, camera, &lights).unwrap();
        terrain.share_environment(&backend, None);
        let mut mesh = MeshRenderer::with_surface(
            &backend,
            &asset,
            SurfaceSettings {
                format: OffscreenTarget::FORMAT,
                capacity: 1,
                quality: Quality::Low,
                reflections: false,
                shadows: terrain.shadows(),
                ray_tracing: Some(&settings),
            },
        )
        .unwrap();
        let mut instance = MeshInstance {
            position_m: center + light_direction * 2.,
            rotation: DQuat::IDENTITY,
            scale: 1.,
        };
        let mut samples = Vec::new();
        for (offset, included) in [(0., true), (4., true), (0., true), (0., false)] {
            instance.position_m = center + light_direction * 2. + right * offset;
            mesh.update_selected(
                &backend,
                &view,
                &lights,
                &[instance],
                Some(&[if included { 0..1 } else { 0..0 }]),
            )
            .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode_surface(&mut encoder);
            // Deliberately omit the small blocker from raster casters.
            terrain.encode_shadows(&mut encoder, None);
            mesh.encode_rt(&mut encoder);
            terrain.encode_opaque_scene_ops(
                &mut encoder,
                &target.color,
                &target.depth,
                Some(&mesh),
                true,
            );
            backend.queue.submit([encoder.finish()]);
            let bytes = target.read_rgba8(&backend).unwrap();
            let at = (64 * 129 + 64) * 4;
            assert_eq!(
                terrain.primary_shadow_stats().active,
                mode == RayTracingRequest::Local && included
            );
            assert_eq!(
                bytes[at + 3],
                if water { 0 } else { 255 },
                "spectral water must really be selected"
            );
            samples.push(bytes[at..at + 4].to_vec());
        }
        eprintln!(
            "water={water} mode={mode:?} center_rgba={samples:?} rt={:?}",
            mesh.rt_stats()
        );
        assert_eq!(
            samples[0], samples[2],
            "restoring geometry restores exact shading"
        );
        if mode == RayTracingRequest::Off {
            assert_eq!(samples[0], samples[1]);
            assert_eq!(samples[0], samples[3]);
            assert!(!mesh.rt_stats().active);
        } else {
            assert!(
                samples[1][0] > samples[0][0] + 40,
                "actual occluder must darken ground"
            );
            assert_eq!(
                samples[0][2], samples[1][2],
                "a blocker of the red star cannot shadow the blue star"
            );
            assert_eq!(
                samples[1], samples[3],
                "empty scene cannot retain old shadows"
            );
        }
        outputs.push(samples);
    }
    assert_eq!(
        outputs[0][0], outputs[1][1],
        "RT miss is the exact raster fallback"
    );
}
