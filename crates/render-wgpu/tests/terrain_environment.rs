use glam::DVec3;
use thessa_graphics::{Quality, ResolvedGraphicsSettings, ResolvedTerrainTopology};
use thessa_rcbt_core::{
    HeightPage,
    lighting::SurfaceLighting,
    material_page::{CbtMaterialPage, MaterialPages},
    tile_anchor::{TileAnchor, TileKey},
};
use thessa_render_core::environment::EnvironmentRadiance;
use thessa_render_wgpu::{
    Backend,
    environment::EnvironmentBinding,
    terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};
use wgpu::util::DeviceExt;

#[path = "terrain_environment/normal_geometry.rs"]
mod normal_geometry;

fn scene(
    backend: &Backend,
    quality: Quality,
    reflections: bool,
    conforming: bool,
) -> TerrainRenderer {
    scene_with_haze(backend, quality, reflections, conforming, false)
}
fn scene_with_haze(
    backend: &Backend,
    quality: Quality,
    reflections: bool,
    conforming: bool,
    haze: bool,
) -> TerrainRenderer {
    let page = HeightPage::bake(&[10.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), 1000.0).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let mut settings = ResolvedGraphicsSettings {
        surface_quality: quality,
        local_haze_extinction_per_m: if haze { 0.002 } else { 0.0 },
        specular_reflections: reflections,
        shadow_enabled: false,
        night_lights_enabled: false,
        clouds_enabled: true,
        clouds_volumetric: true,
        clouds_steps: 24,
        ..Default::default()
    };
    settings.ocean.enabled = false;
    if conforming {
        settings.terrain_topology = ResolvedTerrainTopology {
            capacity: 4096,
            waves: 2,
            split_area_px: 64.0,
            merge_area_px: 8.0,
        };
    }
    let terrain = TerrainRenderer::with_capacity_and_lighting(
        backend,
        &patches,
        wgpu::TextureFormat::Rgba8Unorm,
        TerrainCapacity {
            patches: 6,
            height_grid_size: 2,
            material_layers: 6,
        },
        Some(&settings),
    )
    .unwrap();
    terrain
        .update_lighting(backend, &SurfaceLighting::default())
        .unwrap();
    terrain
}

fn material(backend: &Backend, terrain: &mut TerrainRenderer, roughness: u8, generation: u64) {
    let page = CbtMaterialPage::from_rgba8([0, 0, 0, roughness].repeat(128 * 128)).unwrap();
    let pages = MaterialPages::from_iter((8..14).map(|id| (id, (generation, page.clone()))));
    terrain
        .update_materials(backend, &(8..14).collect::<Vec<_>>(), &pages)
        .unwrap();
}

fn draw(backend: &Backend, terrain: &mut TerrainRenderer, extent: [u32; 2]) -> Vec<u8> {
    let eye = DVec3::Z * 2500.0;
    draw_from(backend, terrain, extent, eye, DVec3::ZERO)
}

fn draw_from(
    backend: &Backend,
    terrain: &mut TerrainRenderer,
    extent: [u32; 2],
    eye: DVec3,
    look_at: DVec3,
) -> Vec<u8> {
    let view = ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, look_at, DVec3::Y).inverse(),
        world_position_m: eye,
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
            1.0,
            f64::from(extent[0]) / f64::from(extent[1]),
            1.0,
            1e5,
        ),
        exposure: 0.1,
    };
    terrain
        .update_view_for_extent(backend, &view, extent)
        .unwrap();
    let target = OffscreenTarget::with_format(
        backend,
        extent[0],
        extent[1],
        wgpu::TextureFormat::Rgba8Unorm,
    )
    .unwrap();
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
#[ignore = "hardware: a zero-slope material normal page is exactly inert at every mip/quality and grazing angle, regular/conforming"]
fn neutral_material_normals_are_exactly_inert_at_grazing_angles() {
    use thessa_rcbt_core::material_page::encode_material_normal_component;
    let backend = Backend::bring_up().unwrap();
    let neutral = encode_material_normal_component(0.0);
    let mut failures = Vec::new();
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut terrain = scene(&backend, quality, true, conforming);
            terrain
                .update_lighting(
                    &backend,
                    &SurfaceLighting {
                        directions: [[-0.464, 0.0, 0.8858307, 0.0], [0.0; 4], [0.0; 4]],
                        colors_lux: [[200.0, 200.0, 200.0, 0.0], [0.0; 4], [0.0; 4]],
                        ..Default::default()
                    },
                )
                .unwrap();
            let rgba = [0, 0, 0, 217].repeat(128 * 128);
            let flat = CbtMaterialPage::from_rgba8(rgba.clone()).unwrap();
            let neutral_page = CbtMaterialPage::from_rgba8_with_normals(
                rgba,
                Some([neutral; 2].repeat(128 * 128)),
            )
            .unwrap();
            for nv in [0.001_f64, 0.01, 0.1, 0.7] {
                let look = DVec3::Z * 1010.0;
                let eye = look + DVec3::new((1.0 - nv * nv).sqrt(), 0.0, nv) * 1500.0;
                // Let conforming geometry settle before comparing materials;
                // subsequent A/B/A equality also guards the scene itself.
                if conforming {
                    for _ in 0..12 {
                        let _ = draw_from(&backend, &mut terrain, [65, 65], eye, look);
                    }
                }
                let mut frames = Vec::new();
                for (generation, page) in [flat.clone(), neutral_page.clone(), flat.clone()]
                    .into_iter()
                    .enumerate()
                {
                    let pages = MaterialPages::from_iter(
                        (8..14).map(|id| (id, (generation as u64 + 1, page.clone()))),
                    );
                    terrain
                        .update_materials(&backend, &(8..14).collect::<Vec<_>>(), &pages)
                        .unwrap();
                    frames.push(draw_from(&backend, &mut terrain, [65, 65], eye, look));
                }
                assert_eq!(
                    frames[0], frames[2],
                    "A/A control must be stable: conforming={conforming} quality={quality:?} nv={nv}"
                );
                let max_error = frames[0]
                    .iter()
                    .zip(&frames[1])
                    .map(|(a, b)| a.abs_diff(*b))
                    .max()
                    .unwrap();
                eprintln!(
                    "neutral_normal conforming={conforming} quality={quality:?} nv={nv} encoded={neutral} max_error_u8={max_error}"
                );
                if max_error != 0 {
                    failures.push((conforming, quality, nv, max_error));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "zero slope must be exactly inert; failures={failures:?}"
    );
}

// Independent one-dimensional Simpson integral of the literal GGX BRDF at
// normal incidence in a uniform radiance field. No importance-sampling mirror.
fn uniform_reference(roughness: f64) -> f64 {
    let r = roughness.clamp(0.02, 1.0);
    let a2 = (r * r).max(0.0025).powi(2);
    let k = (r + 1.0).powi(2) / 8.0;
    let f = |mu: f64| {
        let nh = ((1.0 + mu) * 0.5).sqrt();
        let d = a2 / (std::f64::consts::PI * (nh * nh * (a2 - 1.0) + 1.0).powi(2));
        let gl = mu / (mu * (1.0 - k) + k);
        let fresnel = 0.04 + 0.96 * (1.0 - nh).powi(5);
        0.5 * std::f64::consts::PI * d * gl * fresnel
    };
    let n = 65536;
    let mut sum = f(0.0) + f(1.0);
    for i in 1..n {
        sum += f(i as f64 / n as f64) * if i % 2 == 0 { 2.0 } else { 4.0 };
    }
    sum / (3.0 * n as f64)
}

#[test]
#[ignore = "hardware: terrain receives the shared sky through rough GGX, regular/conforming, all quality tiers and disable lifecycle"]
fn terrain_environment_ggx_matches_uniform_reference_and_releases_the_consumer() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut terrain = scene(&backend, quality, true, conforming);
            assert_eq!(
                terrain.land_environment_samples(),
                match quality {
                    Quality::Low => 4,
                    Quality::Medium => 8,
                    Quality::High => 16,
                }
            );
            for (generation, roughness) in [64, 153, 255u8].into_iter().enumerate() {
                material(&backend, &mut terrain, roughness, generation as u64 + 1);
                assert_eq!(
                    center(&draw(&backend, &mut terrain, [65, 65])),
                    [0; 3],
                    "no field is not an invented sky"
                );
                terrain.share_environment(&backend, Some(&binding));
                let pixel = center(&draw(&backend, &mut terrain, [65, 65]));
                let expected =
                    (uniform_reference(f64::from(roughness) / 255.0) * 100.0 * 0.1 * 255.0).round()
                        as u8;
                let tolerance = match quality {
                    Quality::Low => 7,
                    Quality::Medium => 4,
                    Quality::High => 2,
                };
                assert!(
                    pixel.into_iter().all(|v| v.abs_diff(expected) <= tolerance),
                    "quality={quality:?} conforming={conforming} roughness={roughness} actual={pixel:?} expected={expected} tolerance={tolerance}"
                );
                eprintln!(
                    "terrain_environment quality={quality:?} conforming={conforming} roughness_u8={roughness} actual={pixel:?} reference_u8={expected} tolerance_u8={tolerance}"
                );
                terrain.share_environment(&backend, None);
            }
            let mut off = scene(&backend, quality, false, conforming);
            material(&backend, &mut off, 153, 1);
            assert_eq!(off.land_environment_samples(), 0);
            off.share_environment(&backend, Some(&binding));
            assert_eq!(center(&draw(&backend, &mut off, [65, 65])), [0; 3]);
        }
    }
}

#[test]
#[ignore = "hardware: actual terrain material roughness broadens the reflected directional field instead of adding an unfiltered mirror"]
fn rough_terrain_blurs_environment_and_tracks_publication_without_material_uploads() {
    let backend = Backend::bring_up().unwrap();
    let mut terrain = scene(&backend, Quality::High, true, false);
    let field = |swap: bool| {
        EnvironmentRadiance::new(
            64,
            32,
            (0..32)
                .flat_map(|y| {
                    (0..64).map(move |x| {
                        let elevation = ((y as f32 + 0.5) / 32.0 - 0.5) * std::f32::consts::PI;
                        let azimuth = ((x as f32 + 0.5) / 64.0 - 0.5) * std::f32::consts::TAU;
                        let hot = elevation.cos() * azimuth.sin() > 0.90;
                        if hot ^ swap {
                            [100.0, 0.0, 0.0]
                        } else {
                            [0.0, 0.0, 100.0]
                        }
                    })
                })
                .collect(),
        )
        .unwrap()
    };
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field(false)));
    terrain.share_environment(&backend, Some(&binding));
    material(&backend, &mut terrain, 64, 1);
    let polished = center(&draw(&backend, &mut terrain, [65, 65]));
    material(&backend, &mut terrain, 220, 2);
    let rough = center(&draw(&backend, &mut terrain, [65, 65]));
    assert!(polished[0] > polished[2] + 40, "polished={polished:?}");
    assert!(rough[2] > rough[0] && rough[2] > 10, "rough={rough:?}");
    // Same-size sky writes are shared handles; no material reupload/rebind.
    assert!(!binding.update(&backend, Some(&field(true))));
    let changed = center(&draw(&backend, &mut terrain, [65, 65]));
    assert_eq!(changed, [rough[2], rough[1], rough[0]]);
}

// Direct solid-angle midpoint quadrature of D*G*F/(4*N.V), independent
// of the shader's visible-normal sampling and its PDF correction.
fn grazing_reference(roughness: f64, nv: f64) -> f64 {
    let eye = DVec3::new((1.0 - nv * nv).sqrt(), 0.0, nv);
    let r = roughness.clamp(0.02, 1.0);
    let a2 = (r * r).max(0.0025).powi(2);
    let k = (r + 1.0).powi(2) / 8.0;
    // Integrate the BRDF after cancelling N.V analytically. This independent
    // solid-angle reference remains defined at the exact grazing limit.
    let gv_over_nv = 1.0 / (nv * (1.0 - k) + k);
    let mut sum = 0.0;
    let n = 1024;
    for y in 0..n {
        let mu = (y as f64 + 0.5) / n as f64;
        let sine = (1.0 - mu * mu).sqrt();
        let gl = mu / (mu * (1.0 - k) + k);
        for x in 0..n {
            let phi = std::f64::consts::TAU * (x as f64 + 0.5) / n as f64;
            let h = (eye + DVec3::new(sine * phi.cos(), sine * phi.sin(), mu)).normalize();
            let d = a2 / (std::f64::consts::PI * (h.z * h.z * (a2 - 1.0) + 1.0).powi(2));
            let fresnel = 0.04 + 0.96 * (1.0 - eye.dot(h)).powi(5);
            sum += d * gv_over_nv * gl * fresnel * 0.25;
        }
    }
    sum * std::f64::consts::TAU / (n * n) as f64
}

#[test]
#[ignore = "hardware: terrain grazing reflection matches an independent solid-angle BRDF integral, without surviving-sample renormalization"]
fn grazing_terrain_environment_remains_bounded() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    for conforming in [false, true] {
        let mut terrain = scene(&backend, Quality::High, true, conforming);
        terrain.share_environment(&backend, Some(&binding));
        for (generation, roughness) in [153, 255u8].into_iter().enumerate() {
            material(&backend, &mut terrain, roughness, generation as u64 + 1);
            for nv in [0.7, 0.2, 0.05, 0.01] {
                let look_at = DVec3::Z * 1010.0;
                let eye = look_at + DVec3::new((1.0_f64 - nv * nv).sqrt(), 0.0, nv) * 1500.0;
                let pixel = center(&draw_from(&backend, &mut terrain, [65, 65], eye, look_at));
                let reference = grazing_reference(f64::from(roughness) / 255.0, nv);
                let expected = (reference * 2550.0).round() as u8;
                assert!(reference > 0.0 && reference < 1.0);
                // Sixteen samples have a larger grazing envelope than normal
                // incidence: 8/255 for this fixed radiance/exposure, not a
                // global bound for arbitrary directional HDR sky features.
                assert!(
                    pixel.into_iter().all(|v| v.abs_diff(expected) <= 8),
                    "conforming={conforming} roughness={roughness} nv={nv} actual={pixel:?} expected={expected}"
                );
                eprintln!(
                    "terrain_grazing conforming={conforming} roughness_u8={roughness} nv={nv} actual={pixel:?} reference_u8={expected} tolerance_u8=8"
                );
            }
        }
    }
}

#[test]
#[ignore = "hardware: first-substrate-bounce white/grey furnace deficit, regular/conforming, all material tiers"]
fn rough_dielectric_diffuse_and_specular_share_the_same_directional_energy() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[8.0; 3]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    // Isotropic incoming throughput for aggregate ambient. This independently
    // integrates sixteen cosine-weighted directions rather than the shader's
    // four midpoint directions and VNDF quadrature.
    let average = (0..16)
        .map(|i| grazing_reference(1.0, ((f64::from(i) + 0.5) / 16.0).sqrt()))
        .sum::<f64>()
        / 16.0;
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut terrain = scene(&backend, quality, true, conforming);
            terrain.share_environment(&backend, Some(&binding));
            terrain
                .update_lighting(
                    &backend,
                    &SurfaceLighting {
                        ambient_lux: [
                            8.0 * std::f32::consts::PI,
                            8.0 * std::f32::consts::PI,
                            8.0 * std::f32::consts::PI,
                            0.0,
                        ],
                        ..SurfaceLighting::default()
                    },
                )
                .unwrap();
            for (generation, color) in [255, 128u8].into_iter().enumerate() {
                let page =
                    CbtMaterialPage::from_rgba8([color, color, color, 255].repeat(128 * 128))
                        .unwrap();
                let pages = MaterialPages::from_iter(
                    (8..14).map(|id| (id, (generation as u64 + 1, page.clone()))),
                );
                terrain
                    .update_materials(&backend, &(8..14).collect::<Vec<_>>(), &pages)
                    .unwrap();
                for nv in [0.7, 0.2, 0.05, 0.01] {
                    let look_at = DVec3::Z * 1010.0;
                    let eye = look_at + DVec3::new((1.0_f64 - nv * nv).sqrt(), 0.0, nv) * 1500.0;
                    let pixel = center(&draw_from(&backend, &mut terrain, [65, 65], eye, look_at));
                    let albedo = ((f64::from(color) / 255.0 + 0.055) / 1.055).powf(2.4);
                    let reflected = grazing_reference(1.0, nv);
                    let expected = ((albedo * (1.0 - reflected) * (1.0 - average) + reflected)
                        * 204.0)
                        .round() as u8;
                    // The omitted internal return bounces give a real deficit,
                    // not exact white-furnace equality. Neither substrate may
                    // exceed incoming energy; grey checks the BRDF reference too.
                    let tolerance = if quality == Quality::High { 1 } else { 2 };
                    assert!(
                        pixel.iter().all(|v| v.abs_diff(expected) <= tolerance),
                        "conforming={conforming} quality={quality:?} color={color} nv={nv} actual={pixel:?} reference={expected}"
                    );
                    assert!(
                        pixel.iter().all(|&v| v <= 204),
                        "furnace energy creation: {pixel:?}"
                    );
                }
            }
        }
    }
}

#[test]
#[ignore = "hardware: regular/conforming extreme-grazing raster remains finite without artificial sky floor"]
fn extreme_grazing_terrain_environment_is_not_black() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[100.0; 3]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    for conforming in [false, true] {
        let mut terrain = scene(&backend, Quality::High, true, conforming);
        terrain.share_environment(&backend, Some(&binding));
        material(&backend, &mut terrain, 255, 1);
        for nv in [0.01_f64, 0.001, 0.0001] {
            // Hit the +Z face at its centre for every view angle. The eye sits
            // just above the face plane, so the view direction is nearly
            // tangent; this reproduces the ridge-silhouette case rather than a
            // flat normal-incidence patch. Exact-angle accuracy is covered by
            // the shared-environment compute probe and solid-angle oracle.
            let look_at = DVec3::Z * 1000.0;
            let eye = look_at + DVec3::new((1.0 - nv * nv).sqrt(), 0.0, nv) * 1500.0;
            let pixel = center(&draw_from(&backend, &mut terrain, [65, 65], eye, look_at));
            assert!(
                pixel.iter().all(|&v| v > 0 && v < 128),
                "conforming={conforming} nv={nv} black or overbright silhouette: {pixel:?}"
            );
            eprintln!("terrain_grazing_limit conforming={conforming} nv={nv} actual={pixel:?}");
        }
    }
}

#[test]
#[ignore = "diagnostic hardware: counterbalanced terrain sky-reflection quality cost using real GPU timestamps, not a performance assertion"]
fn terrain_environment_gpu_cost() {
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let Some(timer) = thessa_render_wgpu::timing::GpuIntervals::new(&backend, 2) else {
        eprintln!("terrain_environment_cost unavailable=timestamp_queries");
        return;
    };
    let field = EnvironmentRadiance::new(32, 16, vec![[100.0; 3]; 32 * 16]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    let target =
        OffscreenTarget::with_format(&backend, 1280, 720, wgpu::TextureFormat::Rgba8Unorm).unwrap();
    let configurations = [
        (false, Quality::High, true),
        (true, Quality::Low, true),
        (true, Quality::Medium, true),
        (true, Quality::High, true),
        (false, Quality::High, false),
        (true, Quality::High, false),
    ];
    let mut terrains = configurations.map(|(enabled, quality, sky)| {
        let mut terrain = scene(&backend, quality, enabled, false);
        material(&backend, &mut terrain, 153, 1);
        if sky {
            terrain.share_environment(&backend, Some(&binding));
        } else {
            let page = CbtMaterialPage::from_rgba8([255, 255, 255, 255].repeat(128 * 128)).unwrap();
            let pages = MaterialPages::from_iter((8..14).map(|id| (id, (2, page.clone()))));
            terrain
                .update_materials(&backend, &(8..14).collect::<Vec<_>>(), &pages)
                .unwrap();
            terrain
                .update_lighting(
                    &backend,
                    &SurfaceLighting {
                        ambient_lux: [std::f32::consts::PI; 4],
                        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
                        colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
                    },
                )
                .unwrap();
        }
        let eye = DVec3::Z * 2500.0;
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
                    exposure: 0.1,
                },
                [1280, 720],
            )
            .unwrap();
        terrain
    });
    let mut samples: [Vec<f64>; 6] = std::array::from_fn(|_| Vec::new());
    for index in [0, 1, 2, 3, 4, 5, 5, 4, 3, 2, 1, 0] {
        let terrain = &mut terrains[index];
        for frame in 0..20 {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timer.mark(&mut encoder, 0);
            terrain.encode(&mut encoder, &target.color, &target.depth);
            timer.mark(&mut encoder, 1);
            timer.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let ms = timer.read(&backend).unwrap()[0] * 1000.0;
            if frame >= 4 {
                samples[index].push(ms);
            }
        }
    }
    for (index, values) in samples.iter_mut().enumerate() {
        values.sort_by(f64::total_cmp);
        eprintln!(
            "terrain_environment_cost adapter={:?} quality={:?} samples_per_pixel={} sky={} extent=1280x720 measurements=32 median_ms={:.6} p95_ms={:.6}; sky modes: regular black sphere/material/sky, no direct light/shadows/clouds; no-sky modes: white material plus direct/ambient, no field; classify/raster cost, not whole-client cost",
            backend.adapter.get_info(),
            configurations[index].1,
            terrains[index].land_environment_samples(),
            configurations[index].2,
            values[16],
            values[30]
        );
    }
}

fn uniform_cloud(backend: &Backend) -> thessa_render_wgpu::clouds::CloudBinding {
    cloud_with_extinction(backend, 0.0002)
}

fn cloud_with_extinction(
    backend: &Backend,
    extinction: f32,
) -> thessa_render_wgpu::clouds::CloudBinding {
    let mut records = [[0.0_f32; 4]; 11];
    records[0] = [0.0, 0.0, 2500.0, 1000.0];
    records[1] = [3000.0, 3000.0, extinction, 24.0];
    // Constant external hemispherical source, without self-occlusion quadrature.
    records[2] = [0.0, 1.0, 0.0, 1.0];
    records[10] = [0.0, 0.0, 100.0 * std::f32::consts::PI, 0.0];
    let bytes = records
        .into_iter()
        .flatten()
        .flat_map(f32::to_le_bytes)
        .collect::<Vec<_>>();
    let frame = backend
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("uniform-cloud-test"),
            contents: &bytes,
            usage: wgpu::BufferUsages::UNIFORM,
        });
    let texture = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("uniform-cloud-test"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        },
        wgpu::util::TextureDataOrder::LayerMajor,
        &[0, 0, 0, 128],
    );
    thessa_render_wgpu::clouds::CloudBinding {
        frame,
        weather: texture.create_view(&Default::default()),
        sampler: backend.device.create_sampler(&Default::default()),
        ..thessa_render_wgpu::clouds::CloudBinding::disabled(&backend.device)
    }
}

#[test]
#[ignore = "hardware: nonblack direct-lit material without sky is independent of optically empty cloud publication"]
fn empty_clouds_do_not_change_direct_material_without_a_sky_field() {
    let backend = Backend::bring_up().unwrap();
    let clear = EnvironmentBinding::new(&backend.device);
    let mut cloudy = EnvironmentBinding::new(&backend.device);
    cloudy.set_clouds(cloud_with_extinction(&backend, 0.0));
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut terrain = scene(&backend, quality, true, conforming);
            terrain
                .update_lighting(
                    &backend,
                    &SurfaceLighting {
                        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
                        colors_lux: [[9.0 * std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
                        ..SurfaceLighting::default()
                    },
                )
                .unwrap();
            let page = CbtMaterialPage::from_rgba8([255, 255, 255, 255].repeat(128 * 128)).unwrap();
            let pages = MaterialPages::from_iter((8..14).map(|id| (id, (1, page.clone()))));
            terrain
                .update_materials(&backend, &(8..14).collect::<Vec<_>>(), &pages)
                .unwrap();
            for nv in [1.0_f64, 0.05] {
                let look_at = DVec3::Z * 1010.0;
                let eye = look_at + DVec3::new((1.0 - nv * nv).sqrt(), 0.0, nv) * 1500.0;
                terrain.share_environment(&backend, Some(&clear));
                let a = center(&draw_from(&backend, &mut terrain, [65, 65], eye, look_at));
                terrain.share_environment(&backend, Some(&cloudy));
                let b = center(&draw_from(&backend, &mut terrain, [65, 65], eye, look_at));
                assert!(a.iter().all(|&v| v > 20), "unlit fixture: {a:?}");
                assert_eq!(
                    a, b,
                    "cloud publication changed material: {quality:?} conforming={conforming} nv={nv}"
                );
                let rv = grazing_reference(1.0, nv);
                let rl = grazing_reference(1.0, 1.0);
                // Normal incidence source, roughness=1: D=1/pi and k=0.5.
                let h_cos = ((1.0 + nv) * 0.5).sqrt();
                let fresnel = 0.04 + 0.96 * (1.0 - h_cos).powi(5);
                let spec = fresnel * 2.25 / (nv * 0.5 + 0.5);
                let expected = ((9.0 * (1.0 - rv) * (1.0 - rl) + spec) * 25.5).round() as u8;
                let tolerance = if quality == Quality::High { 1 } else { 2 };
                assert!(
                    a.iter().all(|v| v.abs_diff(expected) <= tolerance),
                    "direct substrate oracle: {quality:?} conforming={conforming} nv={nv} actual={a:?} expected={expected}"
                );
            }
        }
    }
}

// Independently integrate the spherical deck's smooth vertical density and
// Beer-Lambert transport, then the literal BRDF in solid angle. An isotropic
// blue hemispherical boundary and red incident sky separate scattering from transmission.
fn cloud_reference() -> [u8; 3] {
    let r = 153.0_f64 / 255.0;
    let a2 = r.powi(4);
    let k = (r + 1.0).powi(2) / 8.0;
    let mut result = [0.0; 3];
    let bins = 2048;
    for i in 0..bins {
        let mu = (i as f64 + 0.5) / bins as f64;
        let b = 1010.0 * mu;
        let intersection = |radius: f64| -b + (b * b + radius * radius - 1010.0_f64.powi(2)).sqrt();
        let start = intersection(4000.0);
        let stop = intersection(7000.0);
        let dt = (stop - start) / 2048.0;
        let mut tau = 0.0;
        for j in 0..2048 {
            let t = start + (j as f64 + 0.5) * dt;
            let height = ((1010.0_f64.powi(2) + 2.0 * b * t + t * t).sqrt() - 4000.0) / 3000.0;
            tau += (std::f64::consts::PI * height).sin().max(0.0).powf(0.75)
                * (128.0 / 255.0)
                * 0.0002
                * dt;
        }
        let transmission = (-tau).exp();
        let nh = ((1.0 + mu) * 0.5).sqrt();
        let d = a2 / (std::f64::consts::PI * (nh * nh * (a2 - 1.0) + 1.0).powi(2));
        let gl = mu / (mu * (1.0 - k) + k);
        let fresnel = 0.04 + 0.96 * (1.0 - nh).powi(5);
        let weight = 0.5 * std::f64::consts::PI * d * gl * fresnel / bins as f64;
        result[0] += weight * transmission;
        result[2] += weight * 0.5 * (1.0 - transmission);
    }
    result.map(|v| (v * 2550.0).round() as u8)
}

#[test]
#[ignore = "hardware: bounded reflected-cloud integration retains extinction/in-scatter and matches an independent spherical-deck radiative-transfer oracle"]
fn terrain_reflected_cloud_budget_preserves_transport() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[100.0, 0.0, 0.0]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    binding.set_clouds(uniform_cloud(&backend));
    let expected = cloud_reference();
    for conforming in [false, true] {
        let mut terrain = scene(&backend, Quality::High, true, conforming);
        terrain.share_environment(&backend, Some(&binding));
        material(&backend, &mut terrain, 153, 1);
        let pixel = center(&draw(&backend, &mut terrain, [65, 65]));
        assert!(
            pixel[0] > 0 && pixel[2] > 0 && pixel[1] == 0,
            "transmission and scattering must both survive: {pixel:?}"
        );
        assert!(
            pixel
                .into_iter()
                .zip(expected)
                .all(|(a, b)| a.abs_diff(b) <= 8),
            "conforming={conforming} actual={pixel:?} expected={expected:?}"
        );
        eprintln!(
            "terrain_cloud conforming={conforming} actual={pixel:?} reference={expected:?} tolerance_u8=8"
        );
    }
}

#[test]
#[ignore = "hardware: primary terrain shares finite local weather at every material tier, regular/conforming and clear lifecycle"]
fn primary_local_haze_is_shared_and_releases_without_material_uploads() {
    let backend = Backend::bring_up().unwrap();
    let field = EnvironmentRadiance::new(8, 4, vec![[1.0; 3]; 32]).unwrap();
    let mut binding = EnvironmentBinding::new(&backend.device);
    binding.update(&backend, Some(&field));
    let layer = thessa_render_wgpu::environment::LocalHaze {
        extinction_per_m: 0.002,
        height_m: 500.0,
        radius_m: 1000.0,
        camera_m: [0.0, 0.0, 2500.0],
        directional_source: true,
    };
    for conforming in [false, true] {
        for quality in [Quality::Low, Quality::Medium, Quality::High] {
            let mut terrain = scene_with_haze(&backend, quality, false, conforming, true);
            material(&backend, &mut terrain, 153, 1);
            terrain.share_environment(&backend, Some(&binding));
            binding.update_local_haze(&backend, None).unwrap();
            let clear = center(&draw(&backend, &mut terrain, [65, 65]));
            assert_eq!(clear, [0; 3]);
            binding.update_local_haze(&backend, Some(&layer)).unwrap();
            let fog = center(&draw(&backend, &mut terrain, [65, 65]));
            // Center ray enters R+500 and meets actual geometry at R+10.
            let expected = (255.0 * 0.1 * (1.0 - (-0.002_f64 * 490.0).exp())).round() as u8;
            assert!(
                fog.into_iter().all(|v| v.abs_diff(expected) <= 1),
                "{quality:?} conforming={conforming}: {fog:?} vs {expected}"
            );
            binding.update_local_haze(&backend, None).unwrap();
            assert_eq!(center(&draw(&backend, &mut terrain, [65, 65])), clear);
        }
    }
}
