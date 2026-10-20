use glam::DVec3;
use thessa_graphics::{Quality, ResolvedGraphicsSettings, ResolvedTerrainTopology};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_wgpu::{
    Backend,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};

fn scene(
    backend: &Backend,
    settings: &ResolvedGraphicsSettings,
    format: wgpu::TextureFormat,
) -> TerrainRenderer {
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), 1000.0).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    TerrainRenderer::with_capacity_and_lighting(
        backend,
        &patches,
        format,
        TerrainCapacity {
            patches: 6,
            height_grid_size: 2,
            material_layers: 1,
        },
        Some(settings),
    )
    .unwrap()
}

fn lighting(direction: DVec3) -> SurfaceLighting {
    SurfaceLighting {
        directions: [
            [
                direction.x as f32,
                direction.y as f32,
                direction.z as f32,
                0.0,
            ],
            [0.0; 4],
            [0.0; 4],
        ],
        colors_lux: [[20.0, 20.0, 20.0, 0.0], [0.0; 4], [0.0; 4]],
        ..Default::default()
    }
}

fn render(backend: &Backend, terrain: &mut TerrainRenderer, normal: DVec3) -> Vec<u8> {
    let eye = normal * 2500.0;
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, DVec3::ZERO, DVec3::Y)
            .inverse(),
        world_position_m: eye,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e5),
        exposure: 0.001,
    };
    terrain
        .update_view_for_extent(backend, &view, [65, 65])
        .unwrap();
    let target =
        OffscreenTarget::with_format(backend, 65, 65, wgpu::TextureFormat::Rgba8Unorm).unwrap();
    for _ in 0..6 {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        terrain.encode_surface(&mut encoder);
        terrain.encode(&mut encoder, &target.color, &target.depth);
        backend.queue.submit([encoder.finish()]);
    }
    target.read_rgba8(backend).unwrap()
}

fn center(pixels: &[u8]) -> [u8; 3] {
    pixels[(32 * 65 + 32) * 4..][..3].try_into().unwrap()
}

#[test]
#[ignore = "hardware: primary terrain emission reference, disabled/quality/lifecycle, regular and conforming"]
fn native_night_mask_tiers_match_the_shared_linear_emission_law_without_extra_passes() {
    let backend = Backend::bring_up().unwrap();
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut settings = ResolvedGraphicsSettings {
                night_lights_enabled: true,
                night_lights_quality: quality,
                night_lights_intensity: 1.0,
                specular_reflections: false,
                ..Default::default()
            };
            settings.ocean.enabled = false;
            settings.shadow_enabled = false;
            if conforming {
                settings.terrain_topology = ResolvedTerrainTopology {
                    capacity: 4096,
                    waves: 2,
                    split_area_px: 64.0,
                    merge_area_px: 8.0,
                };
            }
            let mut terrain = scene(&backend, &settings, wgpu::TextureFormat::Rgba8Unorm);
            assert_eq!(terrain.conforming_surface().is_some(), conforming);
            terrain
                .update_lighting(&backend, &lighting(-DVec3::X))
                .unwrap();
            let dark = render(&backend, &mut terrain, DVec3::X);
            assert_eq!(
                center(&dark),
                [0; 3],
                "allocated but unpublished mask must stay dark"
            );
            let size = settings.night_light_mask_size().unwrap();
            let bytes = size[0] as usize * size[1] as usize;
            assert_eq!(terrain.night_lights_stats().storage_bytes, bytes as u64);
            terrain
                .update_night_mask(&backend, &vec![255; bytes])
                .unwrap();
            let night = render(&backend, &mut terrain, DVec3::X);
            for (actual, expected) in center(&night).into_iter().zip([128u8, 92, 54]) {
                assert!(
                    actual.abs_diff(expected) <= 2,
                    "tier={quality:?} conforming={conforming} center={:?}",
                    center(&night)
                );
            }
            let before = terrain.night_lights_stats();
            assert!(terrain.update_night_mask(&backend, &[0; 3]).is_err());
            assert_eq!(terrain.night_lights_stats(), before);
            // Conforming topology continues refining the silhouette; the
            // central constant-mask ray is invariant to that independent work.
            assert_eq!(
                center(&render(&backend, &mut terrain, DVec3::X)),
                center(&night),
                "invalid publication must be atomic"
            );
            terrain
                .update_lighting(&backend, &lighting(DVec3::X))
                .unwrap();
            let day = render(&backend, &mut terrain, DVec3::X);
            terrain
                .update_night_mask(&backend, &vec![0; bytes])
                .unwrap();
            assert_eq!(
                center(&render(&backend, &mut terrain, DVec3::X)),
                center(&day),
                "full daylight removes this optical proxy"
            );
            eprintln!(
                "native_night_gpu quality={quality:?} conforming={conforming} linear_center={:?} mask_bytes={bytes} tolerance_u8=2; no-stars and secondary rays not certified",
                center(&night)
            );
        }
    }
}

#[test]
#[ignore = "hardware: longitude orientation and wrap continuity in the existing primary raster shader"]
fn native_night_mask_longitude_seam_and_light_updates_preserve_the_publication() {
    let backend = Backend::bring_up().unwrap();
    let settings = ResolvedGraphicsSettings {
        night_lights_enabled: true,
        night_lights_quality: Quality::Low,
        night_lights_intensity: 1.0,
        specular_reflections: false,
        ..Default::default()
    };
    let mut terrain = scene(&backend, &settings, wgpu::TextureFormat::Rgba8Unorm);
    let [w, h] = settings.night_light_mask_size().unwrap();
    let mask = (0..w * h)
        .map(|i| if i % w < w / 2 { 255 } else { 64 })
        .collect::<Vec<_>>();
    terrain.update_night_mask(&backend, &mask).unwrap();
    for (lon, expected) in [
        (std::f64::consts::FRAC_PI_2, 128u8),
        (3.0 * std::f64::consts::FRAC_PI_2, 32u8),
    ] {
        let normal = DVec3::new(lon.cos(), 0.0, -lon.sin());
        terrain
            .update_lighting(&backend, &lighting(-normal))
            .unwrap();
        assert!(center(&render(&backend, &mut terrain, normal))[0].abs_diff(expected) <= 2);
    }
    let mut seam = Vec::new();
    for lon in [-1e-5_f64, 1e-5] {
        let normal = DVec3::new(lon.cos(), 0.0, -lon.sin());
        terrain
            .update_lighting(&backend, &lighting(-normal))
            .unwrap();
        seam.push(center(&render(&backend, &mut terrain, normal))[0]);
    }
    assert!(
        seam[0].abs_diff(seam[1]) <= 2 && (65..=95).contains(&seam[0]),
        "U must wrap across the shared seam: {seam:?}"
    );
    assert_eq!(
        terrain.night_lights_stats().uploaded_bytes,
        u64::from(w * h),
        "camera and light updates cannot reupload the mask"
    );
}

#[test]
#[ignore = "hardware: disabled and zero-intensity paths allocate no effect texture; HDR pipeline admission"]
fn native_night_mask_disabled_paths_have_no_mask_storage_or_upload() {
    let backend = Backend::bring_up().unwrap();
    for (enabled, intensity) in [(false, 1.0), (true, 0.0)] {
        let settings = ResolvedGraphicsSettings {
            night_lights_enabled: enabled,
            night_lights_intensity: intensity,
            ..Default::default()
        };
        let mut terrain = scene(&backend, &settings, wgpu::TextureFormat::Rgba16Float);
        assert_eq!(terrain.night_lights_stats(), Default::default());
        assert!(terrain.update_night_mask(&backend, &[]).is_err());
    }
}

#[test]
#[ignore = "diagnostic hardware: same regular-sphere primary raster with mask disabled/enabled, actual GPU timestamps"]
fn native_night_mask_gpu_cost() {
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let Some(timer) = thessa_render_wgpu::timing::GpuIntervals::new(&backend, 2) else {
        eprintln!("native_night_gpu_cost unavailable=timestamp_queries");
        return;
    };
    for enabled in [false, true] {
        let mut settings = ResolvedGraphicsSettings {
            night_lights_enabled: enabled,
            night_lights_quality: Quality::High,
            night_lights_intensity: 1.0,
            specular_reflections: false,
            shadow_enabled: false,
            ..Default::default()
        };
        settings.ocean.enabled = false;
        let mut terrain = scene(&backend, &settings, wgpu::TextureFormat::Rgba8Unorm);
        if enabled {
            terrain
                .update_night_mask(&backend, &vec![255; 512 * 256])
                .unwrap();
        }
        terrain
            .update_lighting(&backend, &lighting(-DVec3::X))
            .unwrap();
        let eye = DVec3::X * 2500.0;
        terrain
            .update_view_for_extent(
                &backend,
                &ViewParams {
                    world_from_view: glam::dcamera::rh::view::look_at_mat4(
                        eye,
                        DVec3::ZERO,
                        DVec3::Y,
                    )
                    .inverse(),
                    world_position_m: eye,
                    clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                        1.0,
                        1280.0 / 720.0,
                        1.0,
                        1e5,
                    ),
                    exposure: 0.001,
                },
                [1280, 720],
            )
            .unwrap();
        let target =
            OffscreenTarget::with_format(&backend, 1280, 720, wgpu::TextureFormat::Rgba8Unorm)
                .unwrap();
        let mut samples = Vec::new();
        for frame in 0..36 {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timer.mark(&mut encoder, 0);
            terrain.encode(&mut encoder, &target.color, &target.depth);
            timer.mark(&mut encoder, 1);
            timer.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let interval = timer.read(&backend).unwrap()[0] * 1000.0;
            if frame >= 4 {
                samples.push(interval);
            }
        }
        samples.sort_by(f64::total_cmp);
        eprintln!(
            "native_night_gpu_cost adapter={:?} enabled={enabled} extent=1280x720 samples=32 warmups=4 median_ms={:.6} p95_ms={:.6}; whole isolated regular-sphere classify/raster including fixed cost, not mask-only or whole-client cost",
            backend.adapter.get_info().name,
            samples[16],
            samples[30]
        );
    }
}
