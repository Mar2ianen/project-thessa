//! Shared-WGSL contract only: raster producer coverage belongs to cloud_terrain.
//! No near-limb angular accuracy claim: at Thessa's radius f32 reconstruction
//! has sub-metre uncertainty. Oracle rays stay well away from the true limb.
use glam::DVec3;
use thessa_render_wgpu::{
    Backend,
    clouds::{CloudBinding, SHADER},
};
use wgpu::util::DeviceExt;

const RADIUS: f64 = 3_200_000.0;
const VISIBILITY_TOLERANCE: f32 = 1e-5;
const OPTICAL_TOLERANCE: f32 = 0.002;
// Sun 0: outward; 1: inward; 2: 0.01 rad below the local horizontal.
const CASES: &[(&str, f64, usize)] = &[
    ("tiny datum reconstruction", -0.01, 0),
    ("sub-metre datum reconstruction", -0.5, 0),
    ("triangle chord proxy", -8.0, 0),
    ("coarser triangle chord proxy", -32.0, 0),
    ("datum daylight", 0.0, 0),
    ("positive altitude below cloud deck", 1500.0, 0),
    ("high altitude daylight", 20_000.0, 0),
    ("night-side inward ray", 1500.0, 1),
    ("below-datum inward ray", -8.0, 1),
    ("high altitude depressed sun clears planet", 20_000.0, 2),
    ("same sun blocked at sea level", 0.0, 2),
];

fn directions(normal: DVec3, depression: f64) -> [DVec3; 3] {
    let tangent = normal.any_orthonormal_vector();
    [normal, -normal, (tangent - depression * normal).normalize()]
}

// Independent f64 closest-point-on-forward-ray oracle, not shader root algebra.
// Only terrain has the producer's datum lower bound; real interior origins in
// generic volume/mesh queries remain blocked even when looking outward.
fn clear_planet(mut origin: DVec3, direction: DVec3, terrain: bool) -> bool {
    if terrain {
        origin = origin.normalize() * origin.length().max(RADIUS);
    } else if origin.length() < RADIUS {
        return false;
    }
    let closest = origin + direction * (-origin.dot(direction)).max(0.0);
    closest.length() >= RADIUS - 1e-8
}

fn probe(
    backend: &Backend,
    pipeline: &wgpu::ComputePipeline,
    normal: DVec3,
    camera: DVec3,
    alpha: u8,
    enabled_and_shadows: [f32; 2],
    depression: f64,
) -> Vec<[f32; 2]> {
    // CloudFrame is eleven vec4s (176 bytes), matching the publication ABI.
    let mut frame = [[0.0_f32; 4]; 11];
    frame[0] = camera.as_vec3().extend(RADIUS as f32).to_array();
    frame[1] = [2000.0, 3000.0, 0.0002, 32.0];
    frame[2] = [0.0, enabled_and_shadows[0], 4.0, enabled_and_shadows[1]];
    for (record, direction) in frame[4..7].iter_mut().zip(directions(normal, depression)) {
        *record = direction.as_vec3().extend(0.0).to_array();
    }
    let bytes = frame
        .into_iter()
        .flatten()
        .flat_map(f32::to_le_bytes)
        .collect::<Vec<_>>();
    assert_eq!(bytes.len(), 176);
    let frame = backend
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cloud-visibility-frame"),
            contents: &bytes,
            usage: wgpu::BufferUsages::UNIFORM,
        });
    let texture = backend.device.create_texture_with_data(
        &backend.queue,
        &wgpu::TextureDescriptor {
            label: Some("cloud-visibility-constant-weather"),
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
        &[0, 0, 0, alpha],
    );
    let clouds = CloudBinding {
        frame,
        weather: texture.create_view(&Default::default()),
        sampler: backend.device.create_sampler(&Default::default()),
    };
    let bytes = CASES
        .iter()
        .flat_map(|&(_, altitude, sun)| {
            (normal * (RADIUS + altitude) - camera)
                .as_vec3()
                .extend(sun as f32)
                .to_array()
                .into_iter()
                .flat_map(f32::to_le_bytes)
        })
        .collect::<Vec<_>>();
    let queries = backend
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cloud-visibility-queries"),
            contents: &bytes,
            usage: wgpu::BufferUsages::STORAGE,
        });
    let output = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cloud-visibility-output"),
        size: CASES.len() as u64 * 8,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("cloud-visibility-readback"),
        size: output.size(),
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let [frame, weather, sampler] = clouds.entries();
    let bind = backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("cloud-visibility-probe"),
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                frame,
                weather,
                sampler,
                wgpu::BindGroupEntry {
                    binding: 8,
                    resource: queries.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 9,
                    resource: output.as_entire_binding(),
                },
            ],
        });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups(CASES.len() as u32, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&output, 0, &readback, 0, output.size());
    backend.queue.submit([encoder.finish()]);
    readback
        .slice(..)
        .map_async(wgpu::MapMode::Read, |result| result.unwrap());
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    let bytes = readback.slice(..).get_mapped_range().unwrap();
    let result = bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|pair| {
            std::array::from_fn(|i| f32::from_le_bytes(pair[i * 4..i * 4 + 4].try_into().unwrap()))
        })
        .collect();
    drop(bytes);
    readback.unmap();
    result
}

#[test]
#[ignore = "hardware: shared cloud WGSL terrain datum/planet contract, strict generic volume semantics and weather"]
fn terrain_datum_visibility_preserves_planet_occlusion_and_cloud_extinction() {
    let backend = Backend::bring_up().unwrap();
    let shader = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cloud-visibility-contract"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{SHADER}\n{}",
                    r#"
@group(0) @binding(8) var<storage,read> queries:array<vec4<f32>>;
@group(0) @binding(9) var<storage,read_write> visibility:array<vec2<f32>>;
@compute @workgroup_size(1) fn probe(@builtin(global_invocation_id) id:vec3<u32>) {
    let q=queries[id.x];
    visibility[id.x]=vec2(cloud_terrain_sun_visibility(q.xyz,u32(q.w)),
                          cloud_sun_visibility(q.xyz,u32(q.w)));
}"#
                )
                .into(),
            ),
        });
    let pipeline = backend
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("cloud-visibility-contract"),
            layout: None,
            module: &shader,
            entry_point: Some("probe"),
            compilation_options: Default::default(),
            cache: None,
        });
    // WORLD00's surveyed site (doc 72), not its old approximate bookmark.
    let lat = 23.565679258039204_f64.to_radians();
    let lon = 165.13881581863882_f64.to_radians();
    let world00 = DVec3::new(lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin());
    for normal in [world00, DVec3::X, DVec3::Y, DVec3::Z] {
        let values = probe(
            &backend,
            &pipeline,
            normal,
            DVec3::ZERO,
            0,
            [1.0, 1.0],
            0.01,
        );
        for (&(name, altitude, sun), actual) in CASES.iter().zip(values) {
            for (component, terrain) in [true, false].into_iter().enumerate() {
                // Tiny generic interior offsets can round onto the datum in
                // f32; -8/-32 m pin strict underground semantics unambiguously.
                if !terrain && altitude.abs() < 1.0 && sun == 0 {
                    continue;
                }
                let expected = if clear_planet(
                    normal * (RADIUS + altitude),
                    directions(normal, 0.01)[sun],
                    terrain,
                ) {
                    1.0
                } else {
                    0.0
                };
                assert!(
                    (actual[component] - expected).abs() <= VISIBILITY_TOLERANCE,
                    "{name}: normal={normal:?} terrain={terrain} actual={actual:?} expected={expected}"
                );
            }
        }
    }
    // Axis-aligned datum/tangent rays are exactly representable. Do not impose
    // an exact grazing classification on non-axis f32 uncertainty bands.
    for normal in [DVec3::X, DVec3::Y, DVec3::Z] {
        for depression in [-0.001, 0.0, 0.001] {
            let values = probe(
                &backend,
                &pipeline,
                normal,
                DVec3::ZERO,
                0,
                [1.0, 1.0],
                depression,
            );
            let expected = if depression > 0.0 { 0.0 } else { 1.0 };
            assert!(
                (values[10][0] - expected).abs() <= VISIBILITY_TOLERANCE,
                "datum tangent +/- perturbation: normal={normal:?} depression={depression} actual={:?}",
                values[10]
            );
        }
    }
    let opaque = probe(
        &backend,
        &pipeline,
        world00,
        DVec3::ZERO,
        255,
        [1.0, 1.0],
        0.01,
    );
    // Independent Simpson quadrature of the radial Beer-Lambert column through
    // the whole deck. This is not the shader's 32-point midpoint quadrature.
    let n = 4096;
    let integral = (0..=n)
        .map(|i| {
            let density = (std::f64::consts::PI * i as f64 / n as f64)
                .sin()
                .max(0.0)
                .powf(0.75);
            density
                * if i == 0 || i == n {
                    1.0
                } else if i % 2 == 0 {
                    2.0
                } else {
                    4.0
                }
        })
        .sum::<f64>()
        / (3.0 * n as f64);
    let expected = (-0.0002 * 3000.0 * integral).exp() as f32;
    let positive = CASES
        .iter()
        .position(|case| case.1 == 1500.0 && case.2 == 0)
        .unwrap();
    for actual in opaque[positive] {
        assert!(
            actual > 0.0 && actual < 0.9,
            "weather must attenuate: {actual}"
        );
        assert!(
            (actual - expected).abs() <= OPTICAL_TOLERANCE,
            "weather={actual} reference={expected}"
        );
    }
    assert!((opaque[positive][0] - opaque[positive][1]).abs() <= VISIBILITY_TOLERANCE);
    // Rebase only at a safe >1 km altitude, not in the datum uncertainty band.
    let camera = world00 * (RADIUS + 2500.0) + world00.any_orthonormal_vector() * 2000.0;
    let rebased = probe(&backend, &pipeline, world00, camera, 255, [1.0, 1.0], 0.01);
    for (a, b) in opaque[positive].into_iter().zip(rebased[positive]) {
        assert!(
            (a - b).abs() <= OPTICAL_TOLERANCE,
            "rebase changed visibility: {a} vs {b}"
        );
    }
    for flags in [[0.0, 1.0], [1.0, 0.0]] {
        let disabled = probe(&backend, &pipeline, world00, DVec3::ZERO, 255, flags, 0.01);
        assert!(
            disabled
                .into_iter()
                .flatten()
                .all(|v| (v - 1.0).abs() <= VISIBILITY_TOLERANCE),
            "disabled clouds/shadows must return one, including interior/night rays: {flags:?}"
        );
    }
    eprintln!(
        "cloud_visibility adapter={:?} radius_m={RADIUS} cases={} orientations=4 visibility_tolerance={VISIBILITY_TOLERANCE} optical_tolerance={OPTICAL_TOLERANCE}; shared compute only, no raster/near-limb/performance certification",
        backend.adapter_name,
        CASES.len()
    );
}
