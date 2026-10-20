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

fn scene(
    backend: &Backend,
    quality: Quality,
    reflections: bool,
    conforming: bool,
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
    let gv = nv / (nv * (1.0 - k) + k);
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
            sum += d * gv * gl * fresnel / (4.0 * nv);
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
            for nv in [0.7, 0.2] {
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
        (false, Quality::High),
        (true, Quality::Low),
        (true, Quality::Medium),
        (true, Quality::High),
    ];
    let mut terrains = configurations.map(|(enabled, quality)| {
        let mut terrain = scene(&backend, quality, enabled, false);
        material(&backend, &mut terrain, 153, 1);
        terrain.share_environment(&backend, Some(&binding));
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
    let mut samples: [Vec<f64>; 4] = std::array::from_fn(|_| Vec::new());
    for index in [0, 1, 2, 3, 3, 2, 1, 0] {
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
            "terrain_environment_cost adapter={:?} quality={:?} samples_per_pixel={} extent=1280x720 measurements=32 median_ms={:.6} p95_ms={:.6}; same regular black sphere/material/sky, no direct light/shadows/clouds; whole classify/raster and uncovered sky cost, not whole-client cost",
            backend.adapter.get_info(),
            configurations[index].1,
            terrains[index].land_environment_samples(),
            values[16],
            values[30]
        );
    }
}

fn uniform_cloud(backend: &Backend) -> thessa_render_wgpu::clouds::CloudBinding {
    let mut records = [[0.0_f32; 4]; 11];
    records[0] = [0.0, 0.0, 2500.0, 1000.0];
    records[1] = [3000.0, 3000.0, 0.0002, 24.0];
    records[2] = [0.0, 1.0, 4.0, 1.0];
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
    }
}

// Independently integrate the spherical deck's smooth vertical density and
// Beer-Lambert transport, then the literal BRDF in solid angle. An isotropic
// blue source and red incident sky separate scattering from transmission.
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
        result[2] += weight * (1.0 - transmission);
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
