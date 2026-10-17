use thessa_render_core::ocean::SeaState;
use thessa_render_wgpu::{Backend, ocean::OceanField};

fn backend() -> Option<Backend> {
    match Backend::bring_up() {
        Ok(b) => Some(b),
        Err(thessa_render_wgpu::BackendError::NoAdapter) => None,
        Err(e) => panic!("{e}"),
    }
}

fn half(word: u16) -> f64 {
    let sign = if word & 0x8000 == 0 { 1.0 } else { -1.0 };
    let exponent = (word >> 10) & 31;
    let mantissa = f64::from(word & 1023) / 1024.0;
    sign * if exponent == 31 {
        if mantissa == 0.0 {
            f64::INFINITY
        } else {
            f64::NAN
        }
    } else if exponent == 0 {
        mantissa * 2f64.powi(-14)
    } else {
        (1.0 + mantissa) * 2f64.powi(i32::from(exponent) - 15)
    }
}

fn read(backend: &Backend, field: &OceanField, mip: u32) -> Vec<[f64; 4]> {
    let size = field.stats().size >> mip;
    let stride = (size * 8).div_ceil(256) * 256;
    let buffer = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: u64::from(stride * size * 9),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: field.texture(),
            mip_level: mip,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(size),
            },
        },
        wgpu::Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 9,
        },
    );
    backend.queue.submit([encoder.finish()]);
    let (tx, rx) = std::sync::mpsc::channel();
    buffer.map_async(wgpu::MapMode::Read, .., move |r| {
        tx.send(r).unwrap();
    });
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    rx.recv().unwrap().unwrap();
    let data = buffer.get_mapped_range(..).unwrap();
    let mut out = Vec::new();
    for row in data.chunks_exact(stride as usize) {
        for pixel in row[..size as usize * 8].as_chunks::<8>().0 {
            out.push(std::array::from_fn(|i| {
                half(u16::from_le_bytes([pixel[2 * i], pixel[2 * i + 1]]))
            }));
        }
    }
    drop(data);
    buffer.unmap();
    out
}

#[test]
fn gpu_fft_matches_direct_fourier_sum_and_mips_keep_slope_energy() {
    let Some(backend) = backend() else {
        return;
    };
    let state = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 80.0,
        wind_speed_m_s: 12.0,
        wind_direction_rad: 0.3,
        significant_height_m: 1.5,
        seed: 19,
    };
    for size in [16, 32, 64, 128] {
        let mut field = OceanField::new(&backend, state, size).unwrap();
        let n = size as usize;
        for time in [0.0, 0.125, 72000.0, -45.0, 1e9] {
            assert!(field.update(&backend, time).unwrap());
            let data = read(&backend, &field, 0);
            for layer in 0..9 {
                for (x, y) in [(0, 0), (3, 7), (12, 15)] {
                    let expected = field.spectrum().sample_reference(
                        layer,
                        [x as f64 / n as f64, y as f64 / n as f64],
                        time,
                    );
                    let actual = data[layer * n * n + y * n + x];
                    for channel in 0..3 {
                        assert!(
                            (actual[channel] - expected[channel]).abs()
                                < 0.002 + expected[channel].abs() * 0.002,
                            "time={time} layer={layer} xy={x},{y} channel={channel}: {actual:?} vs {expected:?}"
                        );
                    }
                }
            }
            let top = read(&backend, &field, size.ilog2());
            for layer in 0..9 {
                let moment = data[layer * n * n..(layer + 1) * n * n]
                    .iter()
                    .map(|v| v[3])
                    .sum::<f64>()
                    / (n * n) as f64;
                assert!(
                    (top[layer][3] - moment).abs() < moment * 0.005 + 1e-7,
                    "slope moment mip drift layer={layer}: {} vs {moment}",
                    top[layer][3]
                );
                assert!(
                    top[layer][3] / f64::from(field.stats().moment_scale) + 1e-7
                        >= top[layer][1].powi(2) + top[layer][2].powi(2)
                );
            }
            assert!(!field.update(&backend, time).unwrap());
            assert_eq!(field.stats().fft_passes, 0);
        }
        assert_eq!(field.stats().evaluations, 5);
    }
}

#[test]
fn steep_input_never_overflows_the_portable_moment_storage() {
    let Some(backend) = backend() else {
        return;
    };
    let sea = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 100.0,
        wind_speed_m_s: 1.0,
        wind_direction_rad: 0.3,
        significant_height_m: 20.0,
        seed: 17,
    };
    let mut field = OceanField::new(&backend, sea, 128).unwrap();
    assert!(field.stats().moment_scale < 1024.0);
    field.update(&backend, 1.0).unwrap();
    assert!(
        read(&backend, &field, 0)
            .iter()
            .flatten()
            .all(|v| v.is_finite())
    );
    assert!(
        read(&backend, &field, 7)
            .iter()
            .flatten()
            .all(|v| v.is_finite())
    );
    // Finite representation is not a claim that steep linear waves model
    // breaking accurately; it only pins a public-input numeric contract.
}

#[test]
fn spectral_surface_displaces_shared_geometry_without_drift_and_keeps_land_fixed() {
    use glam::DVec3;
    use thessa_rcbt_core::{
        HeightPage,
        material_page::CbtMaterialPage,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
        view::ViewParams,
    };
    let Some(backend) = backend() else {
        return;
    };
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.ocean.enabled = true;
    requested.ocean.quality = thessa_graphics::Quality::Low;
    requested.ocean.geometry_patches = 1;
    requested.shadows.enabled = false;
    let settings = backend.resolve_graphics(&requested);
    let state = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 80.0,
        wind_speed_m_s: 12.0,
        wind_direction_rad: 0.3,
        significant_height_m: 1.5,
        seed: 19,
    };
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let anchor = TileAnchor::new(TileKey::new(0, 6, 32, 32).unwrap(), 6000.0).unwrap();
    let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 2,
            height_grid_size: 2,
            material_layers: 2,
        },
        Some(&settings),
        Some(state),
    )
    .unwrap();
    let datum = terrain.read_vertices(&backend).unwrap();
    let eye = DVec3::from_array(anchor.anchor_body_m) + DVec3::from_array(anchor.normal) * 80.0;
    let view = ViewParams {
        world_position_m: eye,
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            eye,
            DVec3::from_array(anchor.anchor_body_m),
            DVec3::Y,
        )
        .inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 1e5),
        exposure: 0.7,
    };
    terrain.update_view(&backend, &view).unwrap();
    let material = |roughness| {
        [(
            8u64,
            (
                u64::from(roughness),
                CbtMaterialPage::from_rgba8([20, 40, 80, roughness].repeat(128 * 128)).unwrap(),
            ),
        )]
        .into()
    };
    terrain
        .update_materials(&backend, &[8], &material(30))
        .unwrap();
    terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(terrain.ocean_stats().deformed_patches, 1);
    assert!(terrain.ocean_reason().is_none());
    let waves = terrain.read_vertices(&backend).unwrap();
    let bound = terrain.ocean_stats().height_bound_m;
    let mut moved = 0;
    for (a, b) in waves
        .as_chunks::<8>()
        .0
        .iter()
        .zip(datum.as_chunks::<8>().0)
        .take(1089)
    {
        let distance =
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
        assert!(f64::from(distance) <= bound + 0.001);
        if distance > 0.001 {
            moved += 1;
        }
    }
    assert!(moved > 500, "no ocean geometry displacement: {moved}");
    terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(
        terrain.read_vertices(&backend).unwrap(),
        waves,
        "paused sea must not accumulate"
    );
    terrain.update_ocean(&backend, 0.5).unwrap();
    assert_ne!(terrain.read_vertices(&backend).unwrap(), waves);
    terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(
        terrain.read_vertices(&backend).unwrap(),
        waves,
        "rewinding is deterministic"
    );
    let target = OffscreenTarget::new(&backend, 128, 128).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    let pixels = target.read_rgba8(&backend).unwrap();
    assert!(
        pixels
            .as_chunks::<4>()
            .0
            .iter()
            .any(|p| p.as_slice() != &pixels[..4])
    );
    terrain
        .update_materials(&backend, &[8], &material(255))
        .unwrap();
    terrain.update_ocean(&backend, 0.0).unwrap();
    assert_eq!(
        terrain.read_vertices(&backend).unwrap(),
        datum,
        "rough land/ice cannot become displaced water"
    );
    // A height regeneration supersedes, rather than corrupts, the old backup.
    let raised = HeightPage::bake(&[10.0; 4], 2, 0.001).unwrap();
    terrain.update_pages(&backend, &[(0, &raised)]).unwrap();
    let land = terrain.read_vertices(&backend).unwrap();
    terrain
        .update_materials(&backend, &[8], &material(30))
        .unwrap();
    terrain.update_ocean(&backend, 0.5).unwrap();
    assert_eq!(terrain.read_vertices(&backend).unwrap(), land);
    let other = TileAnchor::new(TileKey::new(0, 6, 32, 33).unwrap(), 6000.0).unwrap();
    let patches = [anchor, other].map(|anchor| thessa_render_wgpu::terrain::ResidentTerrainPatch {
        anchor,
        page: &page,
        generation: 0,
    });
    terrain.publish_partition(&backend, &patches).unwrap();
    let both_datum = terrain.read_vertices(&backend).unwrap();
    terrain.update_ocean(&backend, 0.5).unwrap();
    assert_eq!(terrain.ocean_stats().deformed_patches, 0);
    assert!(terrain.ocean_reason().unwrap().contains("budget admits 1"));
    assert_eq!(
        terrain.read_vertices(&backend).unwrap(),
        both_datum,
        "whole-partition optical fallback must restore original geometry"
    );
    terrain.publish_partition(&backend, &patches[..1]).unwrap();
    terrain.update_ocean(&backend, 0.5).unwrap();
    assert_eq!(terrain.ocean_stats().deformed_patches, 1);
    assert!(terrain.ocean_reason().is_none());
}

#[test]
fn deformed_cube_edge_and_camera_rebase_agree_within_two_millimetres() {
    use glam::DVec3;
    use thessa_rcbt_core::{
        HeightPage,
        material_page::CbtMaterialPage,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
        view::ViewParams,
    };
    let Some(backend) = backend() else {
        return;
    };
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.ocean.enabled = true;
    requested.ocean.quality = thessa_graphics::Quality::Low;
    requested.shadows.enabled = false;
    let settings = backend.resolve_graphics(&requested);
    let state = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 80.0,
        wind_speed_m_s: 12.0,
        wind_direction_rad: 0.3,
        significant_height_m: 1.5,
        seed: 19,
    };
    let anchors = [
        TileAnchor::new(TileKey::new(0, 6, 0, 32).unwrap(), 6000.0).unwrap(),
        TileAnchor::new(TileKey::new(4, 6, 63, 32).unwrap(), 6000.0).unwrap(),
    ];
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = anchors.map(|anchor| TerrainPatch {
        anchor,
        page: &page,
    });
    let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 2,
            height_grid_size: 2,
            material_layers: 2,
        },
        Some(&settings),
        Some(state),
    )
    .unwrap();
    let center = (DVec3::from_array(anchors[0].anchor_body_m)
        + DVec3::from_array(anchors[1].anchor_body_m))
        * 0.5;
    let mut eye = center + center.normalize() * 50.0;
    let material = CbtMaterialPage::from_rgba8([20, 40, 80, 30].repeat(128 * 128)).unwrap();
    terrain
        .update_materials(
            &backend,
            &[8, 12],
            &[(8, (1, material.clone())), (12, (1, material))].into(),
        )
        .unwrap();
    let mut first: Option<Vec<DVec3>> = None;
    for _ in 0..2 {
        let view = ViewParams {
            world_position_m: eye,
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::Y).inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 1e5),
            exposure: 1.0,
        };
        terrain.update_view(&backend, &view).unwrap();
        terrain.update_ocean(&backend, 5.0).unwrap();
        assert_eq!(terrain.ocean_stats().deformed_patches, 2);
        let values = terrain.read_vertices(&backend).unwrap();
        let mut edge = Vec::new();
        for row in 0..33 {
            let a = &values[(row * 33) * 8..];
            let b = &values[(1089 + row * 33 + 32) * 8..];
            let pa = DVec3::from_array(anchors[0].anchor_body_m)
                + DVec3::new(a[0] as f64, a[1] as f64, a[2] as f64);
            let pb = DVec3::from_array(anchors[1].anchor_body_m)
                + DVec3::new(b[0] as f64, b[1] as f64, b[2] as f64);
            assert!(
                (pa - pb).length() < 0.002,
                "cube-edge wave discontinuity: {}",
                (pa - pb).length()
            );
            edge.push(pa);
        }
        if let Some(before) = &first {
            for (a, b) in edge.iter().zip(before) {
                assert!((*a - *b).length() < 0.002, "camera-origin phase drift");
            }
        } else {
            first = Some(edge);
        }
        eye += DVec3::new(0.125, -0.25, 0.125);
    }
}

#[test]
fn displacement_envelope_prevents_five_sample_frustum_rejection_of_an_interior_crest() {
    use glam::{DMat4, DVec3};
    use thessa_rcbt_core::{
        HeightPage,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        terrain::{OffscreenTarget, TerrainCapacity, TerrainPatch, TerrainRenderer},
        view::ViewParams,
    };
    let Some(backend) = backend() else {
        return;
    };
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.ocean.enabled = true;
    request.shadows.enabled = false;
    request.ocean.quality = thessa_graphics::Quality::Low;
    let settings = backend.resolve_graphics(&request);
    let anchor = TileAnchor::new(TileKey::new(0, 17, 65536, 65536).unwrap(), 6_000_000.0).unwrap();
    // A deterministic interior crest isolates the culling envelope from a
    // random wave phase. Its 5 m elevation fits the spectrum's absolute bound;
    // none of the existing classifier's four corners/centre samples contains it.
    let mut heights = vec![0.0; 33 * 33];
    heights[8 * 33 + 8] = 5.0;
    let page = HeightPage::bake(&heights, 33, 0.001).unwrap();
    let sea = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 100.0,
        wind_speed_m_s: 10.0,
        wind_direction_rad: 0.0,
        significant_height_m: 0.4,
        seed: 17,
    };
    let mut terrain = TerrainRenderer::with_capacity_lighting_and_ocean(
        &backend,
        &[TerrainPatch {
            anchor,
            page: &page,
        }],
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 1,
            height_grid_size: 33,
            material_layers: 1,
        },
        Some(&settings),
        Some(sea),
    )
    .unwrap();
    let center = DVec3::from_array(anchor.anchor_body_m);
    let eye = center + DVec3::Z * 200.0;
    let projection = DMat4::from_cols_array(&[
        1.0 / 60.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0 / 3.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -1.0 / 399.9,
        0.0,
        0.0,
        -10.0 / 6.0,
        -0.1 / 399.9,
        1.0,
    ]);
    let view = ViewParams {
        world_position_m: eye,
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, DVec3::X).inverse(),
        clip_from_view: projection,
        exposure: 1.0,
    };
    terrain.update_view(&backend, &view).unwrap();
    terrain.update_ocean(&backend, 0.0).unwrap();
    assert!(terrain.ocean_stats().height_bound_m > 5.0);
    let target = OffscreenTarget::new(&backend, 128, 128).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    terrain.encode(&mut encoder, &target.color, &target.depth);
    backend.queue.submit([encoder.finish()]);
    assert_eq!(
        terrain.read_draw(&backend).unwrap()[0],
        2304 * 3,
        "near grid stays full and the interior crest is conservatively retained"
    );
}

#[test]
#[ignore = "requires a wgpu adapter; release scoped spectral evolution/FFT/mip cost diagnostic"]
fn ocean_fft_cost_diagnostic() {
    use thessa_render_wgpu::timing::GpuIntervals;
    let backend = Backend::bring_up_for_graphics_profiled(&Default::default()).unwrap();
    let sea = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 100.0,
        wind_speed_m_s: 10.0,
        wind_direction_rad: 0.0,
        significant_height_m: 2.0,
        seed: 17,
    };
    for size in [32, 64, 128] {
        let mut field = OceanField::new(&backend, sea, size).unwrap();
        let timing = GpuIntervals::new(&backend, 4);
        let mut host = Vec::new();
        let mut gpu: [Vec<f64>; 3] = Default::default();
        for frame in 0..140 {
            let start = std::time::Instant::now();
            field
                .update_profiled(&backend, frame as f64 / 60.0, timing.as_ref())
                .unwrap();
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            let wall = start.elapsed().as_secs_f64() * 1000.0;
            let times = timing.as_ref().map(|t| t.read(&backend).unwrap());
            if frame >= 20 {
                host.push(wall);
                if let Some(times) = times {
                    for (dst, value) in gpu.iter_mut().zip(times) {
                        dst.push(value * 1000.0);
                    }
                }
            }
        }
        for values in &mut gpu {
            values.sort_by(f64::total_cmp);
        }
        host.sort_by(f64::total_cmp);
        assert!(!field.update(&backend, 139.0 / 60.0).unwrap());
        assert_eq!(field.stats().fft_passes, 0);
        eprintln!(
            "ocean_cost adapter={:?} size={size} layers=9 samples=120 host_p50_ms={} host_p95_ms={} evolution_gpu_p50_ms={:?} fft_gpu_p50_ms={:?} mips_gpu_p50_ms={:?} storage_bytes={} moment_scale={} (initialization, phase-rebase upload and readback excluded; timestamps are command intervals)",
            backend.adapter_name,
            host[60],
            host[114],
            gpu[0].get(60),
            gpu[1].get(60),
            gpu[2].get(60),
            field.stats().storage_bytes,
            field.stats().moment_scale
        );
    }
}
