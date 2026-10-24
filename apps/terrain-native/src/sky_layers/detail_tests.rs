use super::*;

#[test]
#[ignore = "hardware: uploaded nonconstant density, independent f64 columns and all native ray consumers"]
fn shaped_columns_match_f64_and_share_primary_reflection_and_shadow_density() {
    let backend = Backend::bring_up().unwrap();
    let settings = ResolvedGraphicsSettings {
        stars: false,
        aurora_shell: false,
        clouds_enabled: true,
        clouds_quality: Quality::Medium,
        clouds_volumetric: true,
        clouds_steps: 24,
        clouds_layers: 1,
        clouds_coverage: 0.0,
        clouds_opacity: 1.0,
        clouds_cast_shadows: true,
        ..Default::default()
    };
    let radius = 3_200_000.0;
    let camera = DVec3::Z * (radius + 1000.0);
    let mut layers = SkyLayers::new(
        &backend,
        &settings,
        wgpu::TextureFormat::Rgba16Float,
        radius,
    );
    let texture = backend.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("constant-weather-nonconstant-detail"),
        size: wgpu::Extent3d {
            width: 1,
            height: 1,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    backend.queue.write_texture(
        texture.as_image_copy(),
        &[255; 4],
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4),
            rows_per_image: Some(1),
        },
        texture.size(),
    );
    layers
        .active
        .as_mut()
        .unwrap()
        .clouds
        .as_mut()
        .unwrap()
        .binding
        .weather = texture.create_view(&Default::default());
    let lighting = SurfaceLighting {
        directions: [[0.0, 0.0, 1.0, 0.0]; 3],
        ..Default::default()
    };
    layers
        .update(
            &backend,
            &ViewParams {
                world_from_view: glam::DMat4::from_translation(camera),
                world_position_m: camera,
                clip_from_view: glam::DMat4::IDENTITY,
                exposure: 1.0,
            },
            [1, 1],
            0.0,
            DQuat::IDENTITY,
            &lighting,
            None,
            &[],
        )
        .unwrap();
    let module = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("shaped-column-consumers"),
            source: wgpu::ShaderSource::Wgsl(
                format!(
                    "{}\n{}",
                    thessa_render_wgpu::clouds::SHADER,
                    r#"
@group(0) @binding(9) var<storage,read_write> result:array<vec4<f32>>;
@compute @workgroup_size(16) fn probe(@builtin(global_invocation_id) id:vec3<u32>) {
    let p=vec3(f32(id.x)*2048.0,0.0,0.0);let d=vec3(0.0,0.0,1.0);
    result[id.x]=vec4(cloud_transport(p,d,1e30).a,cloud_transport_column(p,d,24u,1u).a,
        cloud_sun_visibility(p,0u),cloud_terrain_sun_visibility(p,0u));
}
"#
                )
                .into(),
            ),
        });
    let pipeline = backend
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: None,
            layout: None,
            module: &module,
            entry_point: Some("probe"),
            compilation_options: Default::default(),
            cache: None,
        });
    let output = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: 256,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: 256,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let binding = layers.cloud_binding().unwrap();
    let mut entries = binding.entries().to_vec();
    entries.push(wgpu::BindGroupEntry {
        binding: 9,
        resource: output.as_entire_binding(),
    });
    let bind = backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &pipeline.get_bind_group_layout(0),
            entries: &entries,
        });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&output, 0, &staging, 0, 256);
    backend.queue.submit([encoder.finish()]);
    let (tx, rx) = std::sync::mpsc::channel();
    staging
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
    backend
        .device
        .poll(wgpu::PollType::wait_indefinitely())
        .unwrap();
    rx.recv().unwrap().unwrap();
    let mapped = staging.slice(..).get_mapped_range().unwrap();
    let measured: Vec<_> = mapped
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let detail = thessa_atmosphere::clouds::bake_density_detail(0x5eed2026, 64).unwrap();
    let sample = |point: DVec3| {
        let p = (point / f64::from(thessa_atmosphere::clouds::DENSITY_DETAIL_PERIOD_M) * 64.0
            - DVec3::splat(0.5))
        .to_array();
        let cell = p.map(|v| v.floor() as i64);
        let f = p.map(|v| v - v.floor());
        let mut sum = 0.0;
        for z in 0..2 {
            for y in 0..2 {
                for x in 0..2 {
                    let corner = [x, y, z];
                    let q = std::array::from_fn::<_, 3, _>(|i| {
                        (cell[i] + corner[i]).rem_euclid(64) as usize
                    });
                    let weight = corner
                        .into_iter()
                        .enumerate()
                        .map(|(i, c)| if c == 0 { 1.0 - f[i] } else { f[i] })
                        .product::<f64>();
                    sum += f64::from(detail.levels[0][(q[2] * 64 + q[1]) * 64 + q[0]]) / 255.0
                        * weight;
                }
            }
        }
        sum / f64::from(detail.mean)
    };
    let slab = thessa_atmosphere::CloudSlab::new(radius, radius + 3000.0, radius + 6000.0).unwrap();
    let mut max_error = 0.0_f64;
    let mut range = [1.0_f32, 0.0_f32];
    for ray in 0..16 {
        let origin = camera + DVec3::X * (ray as f64 * 2048.0);
        let (a, b) =
            thessa_atmosphere::intersect_shell(origin, DVec3::Z, DVec3::ZERO, slab).unwrap();
        let tau = thessa_atmosphere::clouds::integrate_slab_unclamped_simpson(
            |t| {
                let point = origin + DVec3::Z * t;
                thessa_atmosphere::vertical_profile((point.length() - radius - 3000.0) / 3000.0)
                    * sample(point)
            },
            a,
            b,
            8192,
            f64::from(CLOUD_EXTINCTION_PER_M),
        )
        .1;
        let expected = (-tau).exp();
        for consumer in 0..4 {
            let error = (f64::from(measured[ray * 4 + consumer]) - expected).abs();
            max_error = max_error.max(error);
            assert!(
                error < 0.003,
                "ray={ray} consumer={consumer} measured={} expected={expected}",
                measured[ray * 4 + consumer]
            );
        }
        range[0] = range[0].min(measured[ray * 4]);
        range[1] = range[1].max(measured[ray * 4]);
    }
    assert!(
        range[1] - range[0] > 0.05,
        "fixture must exercise nonconstant density: {range:?}"
    );
    eprintln!(
        "shaped_cloud_columns rays=16 consumers=4 max_absolute_transmittance_error={max_error} range={range:?}; 24 GPU midpoints vs 8192 f64 Simpson steps"
    );
}
