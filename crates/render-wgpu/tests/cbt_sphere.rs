//! Experimental persistent topology, no patch geometry, skirts or grid steps.
//! Not a production terrain/ocean/RT replacement or large_cbt parity claim.
use glam::Vec3;
use std::collections::BTreeMap;
use thessa_rcbt_core::{
    Node,
    conforming::{Record, SphereTopology, cube_vertices, root_index},
};
use thessa_rcbt_wgpu::sphere::{BATCH_REQUESTS, PLAN_WORDS, SphereGpu};
use thessa_render_wgpu::{Backend, terrain::OffscreenTarget, timing::GpuIntervals};

fn read(backend: &Backend, buffer: &wgpu::Buffer) -> Vec<u8> {
    let staging = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sphere-oracle-readback"),
        size: buffer.size(),
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    encoder.copy_buffer_to_buffer(buffer, 0, &staging, 0, buffer.size());
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
    let bytes = staging.slice(..).get_mapped_range().unwrap().to_vec();
    staging.unmap();
    bytes
}
fn words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| u32::from_le_bytes(*b))
        .collect()
}
fn records(backend: &Backend, gpu: &SphereGpu) -> Vec<Record> {
    words(&read(backend, &gpu.pool))
        .as_chunks::<8>()
        .0
        .iter()
        .map(|r| Record {
            address: r[..4].try_into().unwrap(),
            neighbors: r[4..].try_into().unwrap(),
        })
        .collect()
}
fn identity(r: Record) -> u64 {
    u64::from(r.address[0]) | (u64::from(r.address[1]) << 32)
}
fn logical(records: &[Record]) -> BTreeMap<u64, [u64; 3]> {
    records
        .iter()
        .filter_map(|&r| {
            let node = Node::from_heap_id(identity(r)).ok()?;
            root_index(node)?;
            Some((
                node.id(),
                std::array::from_fn(|i| identity(records[r.neighbors[i] as usize])),
            ))
        })
        .collect()
}
fn step(backend: &Backend, gpu: &SphereGpu) -> Vec<u32> {
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    gpu.encode(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    words(&read(backend, &gpu.state))
}
fn assert_allocator(backend: &Backend, gpu: &SphereGpu, records: &[Record]) {
    let counts = words(&read(backend, &gpu.ranks));
    let bits = words(&read(backend, &gpu.bits));
    let mut mirror = thessa_rcbt_wgpu::ocbt::OcbtPoolMirror::new(17).unwrap();
    for (slot, r) in records.iter().enumerate() {
        if identity(*r) != 0 {
            mirror.set_bit(slot, true).unwrap();
        }
    }
    assert_eq!(counts, mirror.tree_words());
    assert_eq!(read(backend, &gpu.bits), mirror.bitfield_bytes());
    assert_eq!(bits.iter().map(|b| b.count_ones()).sum::<u32>(), counts[0]);
    let indices = words(&read(backend, &gpu.indices));
    for (i, &slot) in indices[..counts[0] as usize].iter().enumerate() {
        assert_eq!(slot as usize, mirror.decode_bit(i).unwrap());
    }
}

fn assert_geometry(backend: &Backend, gpu: &SphereGpu) {
    let output = backend.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("sphere-vertex-oracle"),
        size: gpu.pool.size() / 32 * 3 * 16,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let source = format!(
        "{}\n{}",
        thessa_rcbt_wgpu::sphere::GEOMETRY_WGSL,
        r#"
@group(0) @binding(0) var<uniform> p:Params;
@group(0) @binding(1) var<storage,read> pool:array<Record>;
@group(0) @binding(2) var<storage,read_write> output:array<vec4<f32>>;
@compute @workgroup_size(64) fn probe(@builtin(global_invocation_id) id:vec3<u32>){
    if(id.x>=arrayLength(&pool)){return;}let r=pool[id.x];
    if(r.address.z<4u||root_of(r)>=12u){return;}
    let t=triangle(r);for(var i=0u;i<3u;i++){output[id.x*3u+i]=vec4(t[i],0.0);}
}"#
    );
    let shader = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("sphere-geometry-probe"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
    let pipeline = backend
        .device
        .create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: None,
            layout: None,
            module: &shader,
            entry_point: Some("probe"),
            compilation_options: Default::default(),
            cache: None,
        });
    // The unused pixel-area helper is eliminated from this entry: only pool
    // and output bindings remain in its automatic layout.
    let bind = backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: gpu.pool.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: output.as_entire_binding(),
                },
            ],
        });
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups((gpu.pool.size() / 32).div_ceil(64) as u32, 1, 1);
    }
    backend.queue.submit([encoder.finish()]);
    let actual = words(&read(backend, &output));
    let snapshot = records(backend, gpu);
    let mut shared = BTreeMap::new();
    let mut max_error = 0.0f64;
    for (slot, r) in snapshot.iter().enumerate() {
        let Ok(node) = Node::from_heap_id(identity(*r)) else {
            continue;
        };
        let Some(vertices) = cube_vertices(node) else {
            continue;
        };
        for (corner, vertex) in vertices.into_iter().enumerate() {
            let bits: [u32; 3] = actual[(slot * 3 + corner) * 4..(slot * 3 + corner) * 4 + 3]
                .try_into()
                .unwrap();
            let p = glam::DVec3::from_array(vertex.map(|v| v as f64)).normalize();
            let q = glam::DVec3::from_array(bits.map(|v| f64::from(f32::from_bits(v))));
            let error = (q - p).abs().max_element();
            max_error = max_error.max(error);
            assert!(
                error < 1e-6,
                "decoded sphere position exceeds unit-coordinate error envelope"
            );
            if let Some(previous) = shared.insert(vertex, bits) {
                assert_eq!(
                    previous, bits,
                    "same integer vertex must be bit-identical across faces/decoding paths"
                );
            }
        }
    }
    println!(
        "sphere_geometry max_absolute_unit_coordinate_error={max_error} bound=1e-6 shared_vertices_bit_identical=true; not a planetary metre/cm guarantee"
    );
}

#[test]
#[ignore = "hardware: GPU persistent compatibility chains versus independent CPU topology after every operation"]
fn conforming_gpu_matches_cpu_and_atomic_exhaustion_recovers() {
    let backend = Backend::bring_up().unwrap();
    let gpu = SphereGpu::new(&backend.device, &backend.queue, 256, 58).unwrap();
    let mut cpu = SphereTopology::new(256, 58).unwrap();
    let mut seed = 19u64;
    let mut saw_chain = false;
    let mut saw_overflow = false;
    for frame in 0..400 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let leaves = cpu.handles();
        let n = cpu.node(leaves[(seed as usize >> 8) % leaves.len()]);
        let merge = frame % 4 == 0 && n.depth() > 4;
        let target = if merge { n.parent().unwrap() } else { n };
        gpu.set_request(&backend.queue, u32::from(merge), target.id());
        let before = records(&backend, &gpu);
        let expected = if merge {
            cpu.merge(target).map(|_| 0)
        } else {
            cpu.split(target)
        };
        let state = step(&backend, &gpu);
        assert_eq!(
            state[3] == 1,
            expected.is_ok(),
            "frame={frame} target={target:?} state={state:?}"
        );
        if let Ok(count) = expected {
            saw_chain |= count > 2;
        } else {
            assert_eq!(
                before,
                records(&backend, &gpu),
                "rejected GPU update must not change any slot"
            );
            saw_overflow |= state[3] == 2;
        }
        cpu.validate().unwrap();
        let snapshot = records(&backend, &gpu);
        assert_eq!(logical(&snapshot), logical(&cpu.records()), "frame={frame}");
        if frame % 17 == 0 {
            assert_allocator(&backend, &gpu, &snapshot);
        }
    }
    assert!(saw_chain && saw_overflow);
    // Recover all the way to roots, then allocate again without resetting pool.
    loop {
        let mut changed = false;
        for h in cpu.handles() {
            if let Some(parent) = cpu.node(h).parent()
                && cpu.merge(parent).is_ok()
            {
                gpu.set_request(&backend.queue, 1, parent.id());
                assert_eq!(step(&backend, &gpu)[3], 1);
                changed = true;
                break;
            }
        }
        if !changed {
            break;
        }
    }
    assert_eq!(cpu.triangle_count(), 12);
    gpu.set_request(&backend.queue, 0, 16);
    assert_eq!(step(&backend, &gpu)[3], 1);
    cpu.split(Node::from_heap_id(16).unwrap()).unwrap();
    assert_eq!(logical(&records(&backend, &gpu)), logical(&cpu.records()));
    let small = SphereGpu::new(&backend.device, &backend.queue, 18, 5).unwrap();
    small.set_request(&backend.queue, 0, 16);
    assert_eq!(step(&backend, &small)[3], 1);
    let before = records(&backend, &small);
    small.set_request(&backend.queue, 0, 18);
    assert_eq!(step(&backend, &small)[3], 2);
    assert_eq!(records(&backend, &small), before);
    small.set_request(&backend.queue, 0, 32);
    assert_eq!(step(&backend, &small)[3], 3);
    assert_eq!(records(&backend, &small), before);
    let deep = SphereGpu::new(&backend.device, &backend.queue, 1024, 58).unwrap();
    let mut reference = SphereTopology::new(1024, 58).unwrap();
    let mut node = Node::from_heap_id(16).unwrap();
    for _ in 4..58 {
        deep.set_request(&backend.queue, 0, node.id());
        assert_eq!(step(&backend, &deep)[3], 1);
        reference.split(node).unwrap();
        assert_eq!(
            logical(&records(&backend, &deep)),
            logical(&reference.records())
        );
        node = node.left_child().unwrap();
    }
    assert!(node.id() > u64::from(u32::MAX));
    reference.validate().unwrap();
    assert_allocator(&backend, &deep, &records(&backend, &deep));
    assert_geometry(&backend, &deep);
    let before = records(&backend, &deep);
    deep.set_request(&backend.queue, 1, (1u64 << 63) | 16);
    assert_eq!(step(&backend, &deep)[3], 4);
    assert_eq!(before, records(&backend, &deep));

    // Generation headroom must reject before any chain allocation/edge write.
    let guarded = SphereGpu::new(&backend.device, &backend.queue, 32, 58).unwrap();
    backend
        .queue
        .write_buffer(&guarded.pool, 12, &u32::MAX.to_le_bytes());
    guarded.set_request(&backend.queue, 0, 16);
    let before = records(&backend, &guarded);
    assert_eq!(step(&backend, &guarded)[3], 5);
    assert_eq!(before, records(&backend, &guarded));
    assert_allocator(&backend, &guarded, &before);
    backend
        .queue
        .write_buffer(&guarded.pool, 12, &1u32.to_le_bytes());
    assert_eq!(step(&backend, &guarded)[3], 1);
}

fn render_pipeline(backend: &Backend, gpu: &SphereGpu) -> (wgpu::RenderPipeline, wgpu::BindGroup) {
    let source = format!(
        "{}\n{}",
        thessa_rcbt_wgpu::sphere::GEOMETRY_WGSL,
        r#"
@group(0) @binding(0) var<uniform> p:Params;
@group(0) @binding(1) var<storage,read> pool:array<Record>;
@group(0) @binding(2) var<storage,read> indices:array<u32>;
struct Vertex { @builtin(position) clip:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) bary:vec3<f32> }
@vertex fn vertex(@builtin(vertex_index) index:u32)->Vertex{
    let r=pool[indices[index/3u]];var out:Vertex;
    if(root_of(r)>=12u){out.clip=vec4(2.0,2.0,2.0,1.0);return out;}
    let normal=triangle(r)[index%3u];out.clip=p.clip*vec4(normal,1.0);out.normal=normal;
    out.bary=vec3(0.0);out.bary[index%3u]=1.0;return out;
}
@fragment fn fragment(in:Vertex)->@location(0) vec4<f32>{
    let width=fwidth(in.bary);let edge=smoothstep(vec3(0.0),width*1.1,in.bary);
    let shade=0.15+0.85*max(0.0,dot(normalize(in.normal),normalize(vec3(0.3,0.5,1.0))));
    return vec4(vec3(0.3,0.7,0.65)*shade*(0.3+0.7*min(edge.x,min(edge.y,edge.z))),1.0);
}"#
    );
    let shader = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("sphere-only-indirect-oracle"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
    let pipeline = backend
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("sphere-only-no-grids-or-skirts"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: OffscreenTarget::FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
    let bind = backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &pipeline.get_bind_group_layout(0),
            entries: &[&gpu.params, &gpu.pool, &gpu.indices]
                .into_iter()
                .enumerate()
                .map(|(binding, buffer)| wgpu::BindGroupEntry {
                    binding: binding as u32,
                    resource: buffer.as_entire_binding(),
                })
                .collect::<Vec<_>>(),
        });
    (pipeline, bind)
}
fn draw(
    encoder: &mut wgpu::CommandEncoder,
    gpu: &SphereGpu,
    target: &OffscreenTarget,
    pipeline: &wgpu::RenderPipeline,
    bind: &wgpu::BindGroup,
) {
    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some("persistent-sphere-raster"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: &target.color,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                store: wgpu::StoreOp::Store,
            },
            depth_slice: None,
        })],
        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
            view: &target.depth,
            depth_ops: Some(wgpu::Operations {
                load: wgpu::LoadOp::Clear(1.0),
                store: wgpu::StoreOp::Store,
            }),
            stencil_ops: None,
        }),
        ..Default::default()
    });
    pass.set_pipeline(pipeline);
    pass.set_bind_group(0, bind, &[]);
    pass.draw_indirect(&gpu.state, 32);
}

#[test]
#[ignore = "hardware: autonomous GPU sphere-only refinement, teleport and indirect wire capture/cost"]
fn autonomous_sphere_indirect_capture_and_scoped_cost() {
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&requested).unwrap();
    let info = backend.adapter.get_info();
    println!(
        "sphere_device backend={:?} driver={:?} driver_info={:?}",
        info.backend, info.driver, info.driver_info
    );
    let gpu = SphereGpu::new(&backend.device, &backend.queue, 2048, 30).unwrap();
    let target = OffscreenTarget::new(&backend, 512, 512).unwrap();
    let (pipeline, bind) = render_pipeline(&backend, &gpu);
    let timing = GpuIntervals::new(&backend, 3).unwrap();
    let mut cpu = SphereTopology::new(2048, 30).unwrap();
    let mut applied = 0;
    let mut teleported = false;
    for frame in 0..420 {
        let camera = if frame < 250 {
            Vec3::new(0.0, 0.0, 3.0)
        } else {
            Vec3::new(-2.2, 0.4, -2.2)
        };
        let clip = glam::camera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 100.0)
            * glam::camera::rh::view::look_at_mat4(camera, Vec3::ZERO, Vec3::Y);
        gpu.set_view(
            &backend.queue,
            clip.to_cols_array(),
            camera.to_array(),
            [512, 512],
            400.0,
            25.0,
        )
        .unwrap();
        let state = step(&backend, &gpu);
        if state[3] == 1 {
            let n = Node::from_heap_id(u64::from(state[1]) | (u64::from(state[2]) << 32)).unwrap();
            if state[0] == 0 {
                cpu.split(n).unwrap();
            } else {
                cpu.merge(n).unwrap();
            }
            applied += 1;
            teleported |= frame >= 250;
        }
        assert_eq!(
            logical(&records(&backend, &gpu)),
            logical(&cpu.records()),
            "autonomous frame={frame}"
        );
        if frame % 40 == 0 {
            cpu.validate().unwrap();
        }
    }
    assert!(applied > 50 && teleported);
    cpu.validate().unwrap();
    assert_allocator(&backend, &gpu, &records(&backend, &gpu));
    assert_geometry(&backend, &gpu);
    let mut times = Vec::new();
    let mut raster = Vec::new();
    let mut baseline_counters = [0u32; 3];
    for frame in 0..130 {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        timing.mark(&mut encoder, 0);
        gpu.encode(&mut encoder);
        timing.mark(&mut encoder, 1);
        draw(&mut encoder, &gpu, &target, &pipeline, &bind);
        timing.mark(&mut encoder, 2);
        timing.resolve(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        let sample = timing.read(&backend).unwrap();
        let result = words(&read(&backend, &gpu.state));
        if result[3] == 1 {
            let n =
                Node::from_heap_id(u64::from(result[1]) | (u64::from(result[2]) << 32)).unwrap();
            if result[0] == 0 {
                cpu.split(n).unwrap();
            } else {
                cpu.merge(n).unwrap();
            }
        }
        if frame == 29 {
            baseline_counters.copy_from_slice(&result[5..8]);
        }
        if frame >= 30 {
            times.push(sample[0] * 1000.0);
            raster.push(sample[1] * 1000.0);
        }
    }
    times.sort_by(f64::total_cmp);
    raster.sort_by(f64::total_cmp);
    let state = words(&read(&backend, &gpu.state));
    cpu.validate().unwrap();
    assert_eq!(logical(&records(&backend, &gpu)), logical(&cpu.records()));
    let pixels = target.read_rgba8(&backend).unwrap();
    assert!(
        pixels
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|p| p[1] > 40)
            .count()
            > 10000,
        "must render a nonempty sphere"
    );
    let directory =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/cbt-sphere");
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("sphere.png");
    let mut encoder = png::Encoder::new(std::fs::File::create(&path).unwrap(), 512, 512);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .unwrap()
        .write_image_data(&pixels)
        .unwrap();
    println!(
        "sphere_prototype adapter={:?} capacity=2048 max_heap_depth=30 triangles={} storage_bytes={} update_and_index_p50_ms={} p95_ms={} raster_p50_ms={} p95_ms={} capture={} measured_split_requests={} measured_merge_requests={} measured_capacity_rejections={} single_writer=true samples=100 warmup=30; absolute GPU scoped costs only, oracle readbacks/replay outside samples, no same-error legacy/large_cbt comparison, heights/materials/ocean/shadows/RT/astronomical precision absent",
        backend.adapter_name,
        state[8] / 3 - 4,
        gpu.storage_bytes(),
        times[50],
        times[95],
        raster[50],
        raster[95],
        path.display(),
        state[5] - baseline_counters[0],
        state[6] - baseline_counters[1],
        state[7] - baseline_counters[2]
    );
}

#[test]
#[ignore = "hardware: conservative height-bound classification policy control"]
fn height_bound_conservatively_refines_without_changing_the_oracle() {
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&requested).unwrap();
    let capacity = 4096u32;
    let max_depth = 30u8;

    // (1) Deterministic per-slot monotonicity on one identical topology.
    // classify runs before update mutates the pool, so a single encode on two
    // identically seeded pools measures the same triangles twice and the only
    // difference is the height bound. Inflated area must never fall below the
    // plain unit-sphere area, which is what makes the bound conservative.
    let seed = |device: &wgpu::Device, queue: &wgpu::Queue| {
        SphereGpu::with_uniform_depth(device, queue, capacity, max_depth, 4).unwrap()
    };
    let plain = seed(&backend.device, &backend.queue);
    let bounded = seed(&backend.device, &backend.queue);
    bounded.set_height_bound(&backend.queue, 0.05).unwrap();
    assert!(bounded.set_height_bound(&backend.queue, -1.0).is_err());
    assert!(bounded.set_height_bound(&backend.queue, f32::NAN).is_err());
    assert!(
        bounded
            .set_height_bound(&backend.queue, f32::INFINITY)
            .is_err()
    );
    let camera = Vec3::new(0.0, 0.0, 2.0);
    let clip = glam::camera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 100.0)
        * glam::camera::rh::view::look_at_mat4(camera, Vec3::ZERO, Vec3::Y);
    // classify runs before update mutates the pool, and both pools are seeded
    // from the same uniform topology, so the two `areas` buffers measure the
    // identical triangles and the only difference is the height bound.
    assert_eq!(
        read(&backend, &plain.pool),
        read(&backend, &bounded.pool),
        "identical seeded topology"
    );
    let mut areas_plain = Vec::new();
    let mut areas_bounded = Vec::new();
    // set_view writes only the first 128 bytes, so the height bound at
    // SURFACE_PARAMS_OFFSET survives; classify needs view mode (config.z == 1).
    for gpu in [&plain, &bounded] {
        gpu.set_view(
            &backend.queue,
            clip.to_cols_array(),
            camera.to_array(),
            [512, 512],
            400.0,
            25.0,
        )
        .unwrap();
    }
    for (gpu, sink) in [(&plain, &mut areas_plain), (&bounded, &mut areas_bounded)] {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        gpu.encode(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        *sink = words(&read(&backend, &gpu.areas));
    }
    let mut strictly_larger = 0u32;
    let mut nonzero = 0u32;
    for (a, b) in areas_plain.iter().zip(&areas_bounded) {
        let a = f32::from_bits(*a);
        let b = f32::from_bits(*b);
        assert!(
            b >= a || (a == 0.0 && b == 0.0),
            "height bound must never lower measured area: plain={a} bounded={b}"
        );
        nonzero += u32::from(a > 0.0);
        strictly_larger += u32::from(b > a);
    }
    let leaves = records(&backend, &plain)
        .iter()
        .filter(|r| r.address[0] != 0)
        .count();
    println!(
        "cbt_height_bound monotonicity leaves={leaves} nonzero_visible_plain={nonzero} strictly_larger={strictly_larger} of {}",
        areas_plain.len()
    );
    assert!(
        nonzero > 0 && strictly_larger == nonzero,
        "every visible triangle must be inflated: visible={nonzero} grew={strictly_larger}"
    );

    // (2) The autonomous policy stays stable and oracle-exact under a nonzero
    // bound: CPU replay still matches GPU bytes and the allocator stays valid.
    let gpu = SphereGpu::new(&backend.device, &backend.queue, 8192, max_depth).unwrap();
    gpu.set_height_bound(&backend.queue, 0.05).unwrap();
    let mut cpu = SphereTopology::new(8192, max_depth).unwrap();
    let mut applied = 0;
    for frame in 0..240 {
        gpu.set_view(
            &backend.queue,
            clip.to_cols_array(),
            camera.to_array(),
            [512, 512],
            400.0,
            25.0,
        )
        .unwrap();
        let state = step(&backend, &gpu);
        if state[3] == 1 {
            let n = Node::from_heap_id(u64::from(state[1]) | (u64::from(state[2]) << 32)).unwrap();
            if state[0] == 0 {
                cpu.split(n).unwrap();
            } else {
                cpu.merge(n).unwrap();
            }
            applied += 1;
        }
        assert_eq!(
            logical(&records(&backend, &gpu)),
            logical(&cpu.records()),
            "bounded autonomous frame={frame}"
        );
    }
    cpu.validate().unwrap();
    assert!(applied > 20, "bounded sweep must actually commit work");
    assert_allocator(&backend, &gpu, &records(&backend, &gpu));
    println!(
        "cbt_height_bound capacity=8192 max_heap_depth={max_depth} height_chord=0.05 camera_distance_radii=2.0 frames=240 committed={applied} inflated_slots={strictly_larger}/{}; conservative LOD policy only, per-leaf emission formula unchanged, no tessellation/analytic-height/large_cbt/Bevy comparison",
        areas_plain.len()
    );
}

fn assert_batch_bytes(backend: &Backend, a: &SphereGpu, b: &SphereGpu, full: bool) {
    for (left, right) in [
        (&a.state, &b.state),
        (&a.plans, &b.plans),
        (&a.ranks, &b.ranks),
        (&a.bits, &b.bits),
    ] {
        assert_eq!(
            read(backend, left),
            read(backend, right),
            "serialized and parallel commits must publish the same bytes"
        );
    }
    if full {
        assert_eq!(read(backend, &a.pool), read(backend, &b.pool));
        assert_eq!(read(backend, &a.indices), read(backend, &b.indices));
    }
}

fn assert_rejected_chain_unchanged(plan: &[u32], before: &[Record], after: &[Record]) {
    let mut touched = std::collections::BTreeSet::new();
    for &slot in &plan[8..8 + plan[2] as usize] {
        let twin = before[slot as usize].neighbors[2];
        for current in [slot, twin] {
            touched.insert(current);
            touched.extend(before[current as usize].neighbors[..3].iter().copied());
        }
    }
    for slot in touched {
        assert_eq!(
            before[slot as usize], after[slot as usize],
            "capacity-rejected whole chain must remain unchanged"
        );
    }
}

#[test]
#[ignore = "hardware: conflict-filtered parallel split/merge waves versus serial byte oracle and CPU geometry"]
fn parallel_waves_match_serial_bytes_and_cpu_topology() {
    let backend = Backend::bring_up().unwrap();
    let parallel =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 2048, 30, 4).unwrap();
    let serial =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 2048, 30, 4).unwrap();
    let mut cpu = SphereTopology::uniform(2048, 30, 4).unwrap();
    let mut max_committed = 0;
    let mut chains = 0;
    let mut conflicts = 0;
    let mut exhausted = 0;
    let mut merged = 0;
    let mut after_teleport = 0;
    for frame in 0..360 {
        for gpu in [&parallel, &serial] {
            if frame < 80 {
                gpu.set_sweep(&backend.queue, false);
            } else if frame < 160 {
                gpu.set_sweep(&backend.queue, true);
            } else {
                let camera = if frame < 260 {
                    Vec3::new(0.0, 0.0, 2.0)
                } else {
                    Vec3::new(-1.5, 0.3, -1.5)
                };
                let clip = glam::camera::rh::proj::directx::perspective(1.0, 1.0, 0.1, 100.0)
                    * glam::camera::rh::view::look_at_mat4(camera, Vec3::ZERO, Vec3::Y);
                gpu.set_view(
                    &backend.queue,
                    clip.to_cols_array(),
                    camera.to_array(),
                    [512, 512],
                    60.0,
                    3.0,
                )
                .unwrap();
            }
        }
        let before = records(&backend, &parallel);
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        parallel.encode_batch(&mut encoder, true);
        serial.encode_batch(&mut encoder, false);
        backend.queue.submit([encoder.finish()]);
        assert_batch_bytes(&backend, &parallel, &serial, true);
        let plans = words(&read(&backend, &parallel.plans));
        let after = records(&backend, &parallel);
        max_committed = max_committed.max(plans[2]);
        for plan in plans[4..]
            .as_chunks::<PLAN_WORDS>()
            .0
            .iter()
            .take(plans[1] as usize)
        {
            match plan[5] {
                1 => {
                    let n = Node::from_heap_id(u64::from(plan[6]) | (u64::from(plan[7]) << 32))
                        .unwrap();
                    if plan[0] == 0 {
                        assert_eq!(cpu.split(n).unwrap(), plan[3] as usize);
                        chains += u32::from(plan[2] > 1);
                    } else {
                        cpu.merge(n).unwrap();
                        merged += 1;
                    }
                    after_teleport += u32::from(frame >= 260);
                }
                2 => {
                    exhausted += 1;
                    assert_rejected_chain_unchanged(plan, &before, &after);
                }
                7 => conflicts += 1,
                0 | 3 | 4 | 5 => {}
                status => panic!("unpublished batch status {status}"),
            }
        }
        cpu.validate().unwrap();
        assert_eq!(
            logical(&after),
            logical(&cpu.records()),
            "parallel frame {frame}"
        );
        if frame == 159 {
            assert_eq!(
                cpu.triangle_count(),
                12,
                "batch decimation must recover the root sphere"
            );
        }
        if frame % 20 == 0 {
            assert_allocator(&backend, &parallel, &after);
        }
    }
    assert!(
        max_committed > 1
            && chains > 0
            && conflicts > 0
            && exhausted > 0
            && merged > 0
            && after_teleport > 0,
        "max_committed={max_committed} chains={chains} conflicts={conflicts} exhausted={exhausted} merged={merged} after_teleport={after_teleport}"
    );
    assert_geometry(&backend, &parallel);
    let parallel_target = OffscreenTarget::new(&backend, 256, 256).unwrap();
    let serial_target = OffscreenTarget::new(&backend, 256, 256).unwrap();
    let (parallel_pipeline, parallel_bind) = render_pipeline(&backend, &parallel);
    let (serial_pipeline, serial_bind) = render_pipeline(&backend, &serial);
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    draw(
        &mut encoder,
        &parallel,
        &parallel_target,
        &parallel_pipeline,
        &parallel_bind,
    );
    draw(
        &mut encoder,
        &serial,
        &serial_target,
        &serial_pipeline,
        &serial_bind,
    );
    backend.queue.submit([encoder.finish()]);
    let pixels = parallel_target.read_rgba8(&backend).unwrap();
    assert_eq!(
        pixels,
        serial_target.read_rgba8(&backend).unwrap(),
        "parallel indirect raster must match the same-topology serial control"
    );
    assert!(
        pixels
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|p| p[1] > 40)
            .count()
            > 2000
    );
    println!(
        "parallel_sphere_oracle frames=360 max_committed_per_wave={max_committed} forced_chains={chains} conflicts={conflicts} rejected_capacity={exhausted} merges={merged} commits_after_teleport={after_teleport} byte_equal_serial=true cpu_closed=true"
    );
}

// Target-size raw GPU snapshot oracle, independent of the planner's links.
fn assert_closed_snapshot(snapshot: &[Record]) {
    let mut edges = BTreeMap::new();
    let mut vertices = std::collections::BTreeSet::new();
    let mut areas = [0u128; 12];
    let mut triangles = 0;
    for (slot, &r) in snapshot.iter().enumerate() {
        let Ok(node) = Node::from_heap_id(identity(r)) else {
            continue;
        };
        let Some(v) = cube_vertices(node) else {
            continue;
        };
        let root = root_index(node).unwrap();
        areas[root as usize] += 1u128 << (58 - node.depth());
        triangles += 1;
        for e in 0..3 {
            vertices.insert(v[e]);
            assert!(edges.insert((v[e], v[(e + 1) % 3]), (slot, e)).is_none());
        }
    }
    for (&(a, b), &(slot, e)) in &edges {
        let &(other, _) = edges
            .get(&(b, a))
            .expect("GPU snapshot must not contain open edges or T-junctions");
        assert_eq!(snapshot[slot].neighbors[e] as usize, other);
    }
    assert_eq!(areas, [1u128 << 54; 12]);
    assert_eq!(
        vertices.len() as isize - (edges.len() / 2) as isize + triangles,
        2
    );
}

#[test]
#[ignore = "hardware: 128K-capacity resident pool, same-workload serial/parallel commit cost and topology oracle"]
fn target_pool_parallel_commit_cost_and_exact_control() {
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&requested).unwrap();
    let parallel =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 131072, 18, 13).unwrap();
    let serial =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 131072, 18, 13).unwrap();
    let timing = GpuIntervals::new(&backend, 4).unwrap();
    let mut samples: [Vec<[f64; 3]>; 2] = Default::default();
    let mut measured_commits = [0u64; 2];
    let mut max_commits = 0;
    let mut csv = String::from("frame,parallel,plan_ms,commit_ms,publish_index_ms,committed\n");
    for frame in 0..130 {
        for gpu in [&parallel, &serial] {
            gpu.set_sweep(&backend.queue, (frame / 16) % 2 == 1);
        }
        // ABBA order, paired identical initial state and exactly identical plans.
        for parallel_first in if frame % 4 == 0 || frame % 4 == 3 {
            [true, false]
        } else {
            [false, true]
        } {
            let gpu = if parallel_first { &parallel } else { &serial };
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timing.mark(&mut encoder, 0);
            gpu.encode_batch_plan(&mut encoder);
            timing.mark(&mut encoder, 1);
            gpu.encode_batch_commit(&mut encoder, parallel_first);
            timing.mark(&mut encoder, 2);
            gpu.encode_batch_publish(&mut encoder);
            timing.mark(&mut encoder, 3);
            timing.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let elapsed = timing.read(&backend).unwrap();
            let plans = words(&read(&backend, &gpu.plans));
            max_commits = max_commits.max(plans[2]);
            if frame >= 30 {
                let mode = usize::from(parallel_first);
                let ms = [
                    elapsed[0] * 1000.0,
                    elapsed[1] * 1000.0,
                    elapsed[2] * 1000.0,
                ];
                samples[mode].push(ms);
                measured_commits[mode] += u64::from(plans[2]);
                csv.push_str(&format!(
                    "{frame},{parallel_first},{},{},{},{}\n",
                    ms[0], ms[1], ms[2], plans[2]
                ));
            }
        }
        assert_batch_bytes(
            &backend,
            &parallel,
            &serial,
            frame % 16 == 0 || frame == 129,
        );
    }
    assert!(max_commits > 1 && measured_commits[1] > 100);
    assert_eq!(measured_commits[0], measured_commits[1]);
    let final_records = records(&backend, &parallel);
    assert_closed_snapshot(&final_records);
    assert_allocator(&backend, &parallel, &final_records);
    let directory =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/cbt-sphere");
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("batch-cost.csv");
    std::fs::write(&path, csv).unwrap();
    let info = backend.adapter.get_info();
    for (mode, sample) in samples.iter().enumerate() {
        let percentile = |scope: usize, index| {
            let mut values = sample
                .iter()
                .map(|s| if scope < 3 { s[scope] } else { s.iter().sum() })
                .collect::<Vec<_>>();
            values.sort_by(f64::total_cmp);
            values[index]
        };
        println!(
            "cbt_batch_cost parallel={} adapter={:?} backend={:?} driver={:?} capacity=131072 initial_triangles=98304 final_triangles={} request_window={} storage_bytes={} samples=100 warmup=30 committed={} max_per_wave={} plan_p50_ms={} commit_p50_ms={} commit_p95_ms={} publish_index_p50_ms={} total_p50_ms={} total_p95_ms={} raw={} byte_equal_serial=true; diagnostic alternating split/merge sweep, classification dispatch inactive, no raster/heights/materials/shadows/ocean/RT, not a game FPS/large_cbt/Bevy comparison",
            mode == 1,
            info.name,
            info.backend,
            info.driver_info,
            words(&read(&backend, &parallel.state))[8] / 3 - 4,
            BATCH_REQUESTS,
            parallel.storage_bytes(),
            measured_commits[mode],
            max_commits,
            percentile(0, 50),
            percentile(1, 50),
            percentile(1, 95),
            percentile(2, 50),
            percentile(3, 50),
            percentile(3, 95),
            path.display()
        );
    }
}

fn surface_snapshot(
    backend: &Backend,
    surface: &thessa_render_wgpu::conforming_surface::ConformingSurface,
    gpu: &SphereGpu,
    pages: &thessa_rcbt_core::height_source::HeightPageStore,
    radius: f64,
    camera: [f64; 3],
) -> (f64, f64, usize) {
    use thessa_rcbt_core::{conforming_surface::SurfaceAddress, tile_anchor::TileAnchor};
    surface.update_camera(backend, camera).unwrap();
    let mut encoder = backend.device.create_command_encoder(&Default::default());
    surface.encode(&mut encoder);
    backend.queue.submit([encoder.finish()]);
    let raw = words(&read(backend, &surface.vertices));
    let records = records(backend, gpu);
    let mut duplicates = BTreeMap::new();
    let mut near_positions = BTreeMap::new();
    let mut max_error = 0.0f64;
    let mut near_error = 0.0f64;
    let mut near = 0;
    for (slot, record) in records.iter().enumerate() {
        let Ok(node) = Node::from_heap_id(identity(*record)) else {
            continue;
        };
        let Some(t) = cube_vertices(node) else {
            continue;
        };
        for (i, cube) in t.into_iter().enumerate() {
            let data = &raw[(slot * 3 + i) * 12..(slot * 3 + i + 1) * 12];
            if let Some(previous) = duplicates.insert(cube, data.to_vec()) {
                assert_eq!(
                    previous, data,
                    "shared seam/corner/deep vertex must be bit identical"
                );
            }
            let address = SurfaceAddress::from_cube(cube).unwrap();
            assert_eq!(data[8..10], address.uv);
            assert_eq!(f32::from_bits(data[7]), f32::from(address.face));
            let sampled = address.sample(pages);
            let height = sampled.map_or(0.0, |(_, _, h)| h);
            assert!(
                (f64::from(f32::from_bits(data[3])) - height).abs() < 0.005,
                "shared HeightPage decoder mismatch"
            );
            assert_eq!(data[10] != u32::MAX, sampled.is_some());
            if let Some((tile, _, _)) = sampled {
                assert_eq!(data[11], u32::from(tile.level));
            }
            let (tile, uv) = address.tile(22).unwrap();
            let expected = TileAnchor::new(tile, radius)
                .unwrap()
                .project_body_m(uv, height);
            let error = (0..3)
                .map(|j| (f64::from(f32::from_bits(data[j])) - (expected[j] - camera[j])).abs())
                .fold(0.0, f64::max);
            assert!(
                error < 8.0,
                "whole-planet arithmetic error {error} cube={cube:?}"
            );
            max_error = max_error.max(error);
            let distance = (0..3)
                .map(|j| (expected[j] - camera[j]).powi(2))
                .sum::<f64>()
                .sqrt();
            if distance < 10_000.0 {
                if let Some(previous) = near_positions.insert(data[..3].to_vec(), cube) {
                    assert_eq!(
                        previous, cube,
                        "distinct local deep vertices must not collapse to one f32 position"
                    );
                }
                assert!(error < 0.02, "near-surface error {error}");
                near_error = near_error.max(error);
                near += 1;
            }
            let normal = (0..3)
                .map(|j| f32::from_bits(data[4 + j]).powi(2))
                .sum::<f32>()
                .sqrt();
            assert!((normal - 1.0).abs() < 1e-5);
        }
    }
    (max_error, near_error, near)
}

#[test]
#[ignore = "hardware: canonical borrowed height pages, seam ownership, depth-58 local planetary precision and residency changes"]
fn conforming_surface_borrows_canonical_pages_and_preserves_deep_precision() {
    use thessa_rcbt_core::{
        HeightPage,
        conforming_surface::{SurfaceAddress, page_id},
        height_source::HeightPageStore,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        conforming_surface::ConformingSurface,
        terrain::{ResidentTerrainPatch, TerrainCapacity, TerrainPatch, TerrainRenderer},
    };
    let backend = Backend::bring_up().unwrap();
    let radius = 6_371_000.0;
    let gpu = SphereGpu::new(&backend.device, &backend.queue, 1024, 58).unwrap();
    let mut node = Node::new(16, 4).unwrap();
    for _ in 4..58 {
        gpu.set_request(&backend.queue, 0, node.id());
        assert_eq!(step(&backend, &gpu)[3], 1);
        node = node.left_child().unwrap();
    }
    let vertex = cube_vertices(node).unwrap()[0];
    let address = SurfaceAddress::from_cube(vertex).unwrap();
    let (fine, _) = address.tile(22).unwrap();
    let mut tiles = (0..6)
        .filter(|&face| face != fine.face)
        .map(|face| TileKey::new(face, 0, 0, 0).unwrap())
        .collect::<Vec<_>>();
    tiles.push(fine);
    let mut pages = HeightPageStore::default();
    for &tile in &tiles {
        let mut heights = Vec::new();
        for y in 0..33 {
            for x in 0..33 {
                heights.push(
                    100.0 + f64::from(tile.face) * 20.0 + f64::from(x) * 0.25
                        - f64::from(y) * 0.125,
                );
            }
        }
        pages.set_page(
            page_id(tile).unwrap(),
            HeightPage::bake(&heights, 33, 0.01).unwrap(),
        );
    }
    let patches = tiles
        .iter()
        .map(|&tile| TerrainPatch {
            anchor: TileAnchor::new(tile, radius).unwrap(),
            page: pages.get(page_id(tile).unwrap()).unwrap(),
        })
        .collect::<Vec<_>>();
    let mut owner = TerrainRenderer::with_capacity(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 8,
            height_grid_size: 33,
            material_layers: 1,
        },
    )
    .unwrap();
    let source = owner.height_surface_source(&backend).unwrap();
    let surface = ConformingSurface::new(&backend, &gpu.pool, &source).unwrap();
    let (tile, uv) = address.tile(22).unwrap();
    let camera = TileAnchor::new(tile, radius)
        .unwrap()
        .project_body_m(uv, 1000.0);
    let (max_error, near_error, near) =
        surface_snapshot(&backend, &surface, &gpu, &pages, radius, camera);
    assert!(near > 50);
    for offset in [-5000.0, 5000.0] {
        surface_snapshot(
            &backend,
            &surface,
            &gpu,
            &pages,
            radius,
            [camera[0], camera[1] + offset, camera[2] - offset],
        );
    }
    for sign in [-1.0, 1.0] {
        let d = (radius + 1000.0) / 3.0f64.sqrt();
        surface_snapshot(&backend, &surface, &gpu, &pages, radius, [sign * d, d, -d]);
    }
    // Reorder/reuse source slots, generation changes and eviction are published
    // by the EXISTING terrain owner; no new texture/residual upload path.
    pages.set_page(
        page_id(fine).unwrap(),
        HeightPage::bake(&[250.0; 33 * 33], 33, 0.01).unwrap(),
    );
    for iteration in 0..3 {
        if iteration == 1 {
            pages.remove_page(page_id(fine).unwrap());
        }
        if iteration == 2 {
            pages.set_page(
                page_id(fine).unwrap(),
                HeightPage::bake(&[300.0; 33 * 33], 33, 0.01).unwrap(),
            );
        }
        let publications = tiles
            .iter()
            .rev()
            .filter_map(|&tile| {
                let id = page_id(tile).unwrap();
                Some(ResidentTerrainPatch {
                    anchor: TileAnchor::new(tile, radius).unwrap(),
                    page: pages.get(id)?,
                    generation: pages.page_version(id).unwrap(),
                })
            })
            .collect::<Vec<_>>();
        owner.publish_partition(&backend, &publications).unwrap();
        surface_snapshot(&backend, &surface, &gpu, &pages, radius, camera);
        let stats = owner.publish_partition(&backend, &publications).unwrap();
        assert_eq!(stats.regenerated, 0);
    }
    // An entirely missing canonical face has an explicit zero-height fallback,
    // never an out-of-bounds slot alias or a stale evicted page.
    pages.remove_page(8);
    pages.remove_page(page_id(fine).unwrap());
    let publications = tiles
        .iter()
        .filter_map(|&tile| {
            let id = page_id(tile).unwrap();
            Some(ResidentTerrainPatch {
                anchor: TileAnchor::new(tile, radius).unwrap(),
                page: pages.get(id)?,
                generation: pages.page_version(id).unwrap(),
            })
        })
        .collect::<Vec<_>>();
    owner.publish_partition(&backend, &publications).unwrap();
    surface_snapshot(&backend, &surface, &gpu, &pages, radius, camera);
    println!(
        "cbt_surface adapter={:?} backend={:?} radius_m={radius} max_heap_depth=58 max_absolute_position_error_m={max_error} near_10km_error_m={near_error} near_vertices={near} shared_vertices_bit_identical=true quantized_height_error_bound_m=0.005 whole_planet_arithmetic_bound_m=8 near_10km_arithmetic_bound_m=0.02; no tessellation/analytic-height/astronomical-rotation guarantee",
        backend.adapter.get_info().name,
        backend.adapter.get_info().backend
    );
}

fn surface_material_pipeline(
    backend: &Backend,
    gpu: &SphereGpu,
    surface: &thessa_render_wgpu::conforming_surface::ConformingSurface,
    materials: &thessa_render_wgpu::material::MaterialBinding,
    clip: [f32; 16],
) -> (wgpu::RenderPipeline, wgpu::BindGroup) {
    use wgpu::util::DeviceExt;
    let source = format!(
        "{}\nvar<private> p:Params;\n{}\n{}",
        thessa_rcbt_wgpu::sphere::GEOMETRY_WGSL,
        thessa_rcbt_core::shaders::MATERIAL_SAMPLE_WGSL
            .replace("@group(0) @binding(16)", "@group(0) @binding(6)"),
        r#"
struct SurfaceVertex { position_height:vec4<f32>, normal_face:vec4<f32>, address:vec4<u32> }
@group(0) @binding(0) var<uniform> clip:mat4x4<f32>;
@group(0) @binding(1) var<storage,read> surface_vertices:array<SurfaceVertex>;
@group(0) @binding(2) var<storage,read> indices:array<u32>;
@group(0) @binding(3) var<storage,read> pool:array<Record>;
@group(0) @binding(4) var material_texture:texture_2d_array<f32>;
@group(0) @binding(5) var material_sampler:sampler;
struct V { @builtin(position) clip:vec4<f32>, @location(0) direction:vec3<f32>, @location(1) normal:vec3<f32> }
@vertex fn vertex(@builtin(vertex_index) index:u32)->V {
    let slot=indices[index/3u];let r=pool[slot];var out:V;
    if(root_of(r)>=12u){out.clip=vec4(2.0,2.0,2.0,1.0);return out;}
    let v=surface_vertices[slot*3u+index%3u];
    out.clip=clip*vec4(v.position_height.xyz,1.0);out.direction=triangle(r)[index%3u];out.normal=v.normal_face.xyz;return out;
}
@fragment fn fragment(in:V)->@location(0) vec4<f32> {
    let address=material_address(normalize(in.direction));
    let value=material_at_level(address,dpdx(address.uv),dpdy(address.uv),2u);
    let light=0.2+0.8*max(0.0,dot(normalize(in.normal),normalize(vec3(0.3,0.5,1.0))));
    return vec4((value.value.rgb+vec3(0.2)*(1.0-value.coverage))*light,1.0);
}"#
    );
    let shader = backend
        .device
        .create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cbt-canonical-material-diagnostic"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
    let pipeline = backend
        .device
        .create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cbt-surface-material-oracle"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vertex"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fragment"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: OffscreenTarget::FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview_mask: None,
            cache: None,
        });
    let matrix = backend
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: None,
            contents: &clip
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<_>>(),
            usage: wgpu::BufferUsages::UNIFORM,
        });
    let mut entries = [&matrix, &surface.vertices, &gpu.indices, &gpu.pool]
        .into_iter()
        .enumerate()
        .map(|(binding, b)| wgpu::BindGroupEntry {
            binding: binding as u32,
            resource: b.as_entire_binding(),
        })
        .collect::<Vec<_>>();
    entries.extend([
        wgpu::BindGroupEntry {
            binding: 4,
            resource: wgpu::BindingResource::TextureView(&materials.texture),
        },
        wgpu::BindGroupEntry {
            binding: 5,
            resource: wgpu::BindingResource::Sampler(&materials.sampler),
        },
        wgpu::BindGroupEntry {
            binding: 6,
            resource: materials.directory.as_entire_binding(),
        },
    ]);
    let bind = backend
        .device
        .create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &pipeline.get_bind_group_layout(0),
            entries: &entries,
        });
    (pipeline, bind)
}

#[test]
#[ignore = "hardware: 128K-slot canonical height emission cost, indirect material raster and independent source updates"]
fn canonical_surface_target_cost_and_shared_material_indirect_raster() {
    use thessa_rcbt_core::{
        HeightPage,
        conforming_surface::page_id,
        height_source::HeightPageStore,
        material_page::{CbtMaterialPage, MaterialPages},
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        conforming_surface::ConformingSurface,
        terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer},
    };
    let mut requested = thessa_graphics::RequestedGraphics::default();
    requested.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    let backend = Backend::bring_up_for_graphics_profiled(&requested).unwrap();
    let radius = 6_371_000.0;
    let gpu =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 131072, 18, 13).unwrap();
    gpu.set_request(&backend.queue, 2, 0);
    step(&backend, &gpu);
    let tiles = (0..6)
        .map(|face| TileKey::new(face, 0, 0, 0).unwrap())
        .collect::<Vec<_>>();
    let mut pages = HeightPageStore::default();
    for &tile in &tiles {
        pages.set_page(
            page_id(tile).unwrap(),
            HeightPage::bake(&[100.0; 33 * 33], 33, 0.01).unwrap(),
        );
    }
    let patches = tiles
        .iter()
        .map(|&tile| TerrainPatch {
            anchor: TileAnchor::new(tile, radius).unwrap(),
            page: pages.get(page_id(tile).unwrap()).unwrap(),
        })
        .collect::<Vec<_>>();
    let mut owner = TerrainRenderer::with_capacity(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 6,
            height_grid_size: 33,
            material_layers: 6,
        },
    )
    .unwrap();
    let surface = ConformingSurface::new(
        &backend,
        &gpu.pool,
        &owner.height_surface_source(&backend).unwrap(),
    )
    .unwrap();
    let camera = [0.0, 0.0, radius * 3.0];
    let (max_error, _, _) = surface_snapshot(&backend, &surface, &gpu, &pages, radius, camera);
    let timer = GpuIntervals::new(&backend, 2).unwrap();
    let mut samples = Vec::new();
    for i in 0..50 {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        timer.mark(&mut encoder, 0);
        surface.encode(&mut encoder);
        timer.mark(&mut encoder, 1);
        timer.resolve(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        let ms = timer.read(&backend).unwrap()[0] * 1000.0;
        if i >= 10 {
            samples.push(ms);
        }
    }
    let raw = samples.iter().map(|v| format!("{v}\n")).collect::<String>();
    samples.sort_by(f64::total_cmp);
    let directory =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/cbt-sphere");
    std::fs::create_dir_all(&directory).unwrap();
    std::fs::write(directory.join("surface-cost-ms.csv"), raw).unwrap();
    let mut materials = MaterialPages::new();
    let demand = (8..14).collect::<Vec<_>>();
    for &id in &demand {
        materials.insert(
            id,
            (
                1,
                CbtMaterialPage::from_rgba8([220, 20, 20, 128].repeat(128 * 128)).unwrap(),
            ),
        );
    }
    owner
        .update_materials(&backend, &demand, &materials)
        .unwrap();
    let binding = owner.material_binding();
    let clip =
        (glam::camera::rh::proj::directx::perspective(1.0, 1.0, 100.0, (radius * 10.0) as f32)
            * glam::camera::rh::view::look_at_mat4(Vec3::ZERO, Vec3::NEG_Z, Vec3::Y))
        .to_cols_array();
    let (pipeline, bind) = surface_material_pipeline(&backend, &gpu, &surface, &binding, clip);
    let target = OffscreenTarget::new(&backend, 512, 512).unwrap();
    let topology_before = read(&backend, &gpu.pool);
    let geometry_before = read(&backend, &surface.vertices);
    let mut totals = Vec::new();
    for color in [[220, 20, 20, 128], [20, 220, 20, 128]] {
        for &id in &demand {
            materials.insert(
                id,
                (
                    u64::from(color[1]),
                    CbtMaterialPage::from_rgba8(color.repeat(128 * 128)).unwrap(),
                ),
            );
        }
        assert_eq!(
            owner
                .update_materials(&backend, &demand, &materials)
                .unwrap()
                .pages,
            6
        );
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        draw(&mut encoder, &gpu, &target, &pipeline, &bind);
        backend.queue.submit([encoder.finish()]);
        let pixels = target.read_rgba8(&backend).unwrap();
        let sum = |channel| {
            pixels
                .as_chunks::<4>()
                .0
                .iter()
                .map(|p| u64::from(p[channel]))
                .sum::<u64>()
        };
        totals.push([sum(0), sum(1)]);
        assert!(
            pixels
                .as_chunks::<4>()
                .0
                .iter()
                .filter(|p| p[0] > 40 || p[1] > 40)
                .count()
                > 10_000
        );
        let mut png = png::Encoder::new(
            std::fs::File::create(directory.join("surface-material.png")).unwrap(),
            512,
            512,
        );
        png.set_color(png::ColorType::Rgba);
        png.set_depth(png::BitDepth::Eight);
        png.write_header()
            .unwrap()
            .write_image_data(&pixels)
            .unwrap();
    }
    assert!(
        totals[0][0] > totals[0][1] * 3 && totals[1][1] > totals[1][0] * 3,
        "borrowed material update must change the actual indirect raster"
    );
    assert_eq!(topology_before, read(&backend, &gpu.pool));
    assert_eq!(geometry_before, read(&backend, &surface.vertices));
    assert_eq!(
        owner
            .update_materials(&backend, &demand, &materials)
            .unwrap()
            .bytes,
        0
    );
    println!(
        "cbt_surface_cost adapter={:?} backend={:?} driver={:?} initial_triangles=98304 capacity=131072 page_grid=33 root_pages=6 samples=40 warmup=10 p50_ms={} p95_ms={} surface_buffer_bytes={} absolute_coordinate_max_error_m={max_error} actual_material_indirect_draw=true raw={} capture={}; emission only, no topology classification/update/shadows/ocean/RT, not game FPS",
        backend.adapter.get_info().name,
        backend.adapter.get_info().backend,
        backend.adapter.get_info().driver_info,
        samples[20],
        samples[38],
        surface.storage_bytes(),
        directory.join("surface-cost-ms.csv").display(),
        directory.join("surface-material.png").display()
    );
}

#[test]
#[ignore = "hardware: CBT surface casts into the shared shadow atlas"]
fn conforming_surface_casts_shared_shadow_depth() {
    use thessa_graphics::{Capabilities, Quality, RequestedGraphics, ResolvedGraphicsSettings};
    use thessa_rcbt_core::{
        HeightPage,
        conforming_surface::page_id,
        height_source::HeightPageStore,
        lighting::SurfaceLighting,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_wgpu::{
        conforming_surface::ConformingSurface,
        terrain::{ResidentTerrainPatch, TerrainCapacity, TerrainPatch, TerrainRenderer},
    };
    let backend = Backend::bring_up().unwrap();
    let radius = 1_000.0;
    // Whole faces cover the sphere. At R=1000 a level-11 tile is only about
    // 2R/2048 = 0.977 m wide in cube coordinates, subpixel in the old 12 km
    // shadow cover; that fixture could not establish depth agreement.
    let tiles = (0..6)
        .map(|face| TileKey::new(face, 0, 0, 0).unwrap())
        .collect::<Vec<_>>();
    let mut pages = HeightPageStore::default();
    for &tile in &tiles {
        pages.set_page(
            page_id(tile).unwrap(),
            HeightPage::bake(&[20.0; 33 * 33], 33, 0.01).unwrap(),
        );
    }
    let patches = tiles
        .iter()
        .map(|&tile| TerrainPatch {
            anchor: TileAnchor::new(tile, radius).unwrap(),
            page: pages.get(page_id(tile).unwrap()).unwrap(),
        })
        .collect::<Vec<_>>();
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
    request.shadows.enabled = true;
    request.shadows.cascades = 1;
    request.shadows.map_size = 512;
    request.shadows.max_distance_m = 2_000.0;
    request.shadows.quality = Quality::Low;
    request.shadows.normal_bias_m = 0.001;
    let settings = ResolvedGraphicsSettings::from_requested(&request, &Capabilities::unknown());
    let mut owner = TerrainRenderer::with_capacity_and_lighting(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 8,
            height_grid_size: 33,
            material_layers: 1,
        },
        Some(&settings),
    )
    .unwrap();
    let publications = tiles
        .iter()
        .map(|&tile| ResidentTerrainPatch {
            anchor: TileAnchor::new(tile, radius).unwrap(),
            page: pages.get(page_id(tile).unwrap()).unwrap(),
            generation: pages.page_version(page_id(tile).unwrap()).unwrap(),
        })
        .collect::<Vec<_>>();
    owner.publish_partition(&backend, &publications).unwrap();
    let camera_tile = tiles[4];
    let camera = TileAnchor::new(camera_tile, radius)
        .unwrap()
        .project_body_m([0.5, 0.5], 300.0);
    let view = thessa_render_wgpu::view::ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(
            glam::DVec3::from_array(camera),
            glam::DVec3::ZERO,
            glam::DVec3::Y,
        )
        .inverse(),
        world_position_m: glam::DVec3::from_array(camera),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(1.0, 1.0, 1.0, 1e6),
        exposure: 1.0,
    };
    owner.update_view(&backend, &view).unwrap();
    let directions = [
        glam::DVec3::Z,
        glam::DVec3::new(0.7, 0.2, 0.4).normalize(),
        glam::DVec3::new(-0.2, 0.9, 0.3).normalize(),
    ];
    let lights = SurfaceLighting {
        directions: directions.map(|d| [d.x as f32, d.y as f32, d.z as f32, 0.0]),
        colors_lux: [[std::f32::consts::PI; 4]; 3],
        ..Default::default()
    };
    owner.update_lighting(&backend, &lights).unwrap();
    owner
        .update_shadows(&backend, view.world_position_m, &lights)
        .unwrap();

    // Eight local bisections give 3072 real triangles plus four virtual roots.
    // Depth-4 roots 16..27 are real geometry even at local_depth=0; only roots
    // 28..31 and free slots are zeroed. Page level is not bisector depth.
    let gpu = SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 4096, 12, 8).unwrap();
    let source = owner.height_surface_source(&backend).unwrap();
    let surface = ConformingSurface::new(&backend, &gpu.pool, &source).unwrap();
    surface.update_camera(&backend, camera).unwrap();

    assert!(owner.shadows().is_some());
    // Emit first: the vertex buffer is written by the surface dispatch, so a
    // readback before it observes an unwritten buffer rather than a topology.
    {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        surface.encode(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        let bytes = read(&backend, &surface.vertices);
        // The emitter zeroes an entire record for a reserved virtual root and
        // never writes an all-zero record for a live leaf, so the sentinel the
        // caster culls is exactly the all-zero record.
        let live = bytes
            .as_chunks::<48>()
            .0
            .iter()
            .filter(|record| record.iter().any(|&b| b != 0))
            .count();
        println!("cbt_shadow_surface live_records={live}");
        assert_eq!(live, 12 * 256 * 3);
    }
    // Legacy full-caster depth as the reference silhouette/depth bound.
    let legacy = {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        owner.encode_shadows_reference(&mut encoder, None);
        backend.queue.submit([encoder.finish()]);
        println!(
            "cbt_shadow_legacy_draws passes={} stats={:?}",
            owner.last_shadow_passes(),
            owner.last_shadow_draws()
        );
        owner.shadows().unwrap().read_depth(&backend).unwrap()
    };

    // CBT caster: emit surface, then cast the vertex stream into the atlas.
    let stats = {
        let atlas = owner.shadows().unwrap();
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        surface.encode(&mut encoder);
        let stats = atlas.encode_conforming_casters(&backend, &mut encoder, &surface);
        backend.queue.submit([encoder.finish()]);
        stats
    };
    let cbt = owner.shadows().unwrap().read_depth(&backend).unwrap();

    assert!(
        stats.draw_calls > 0 && stats.submitted_slots > 0,
        "CBT caster must submit slots"
    );
    let to_f32 = |b: &[u8]| {
        b.as_chunks::<4>()
            .0
            .iter()
            .map(|c| f32::from_le_bytes(*c))
            .collect::<Vec<_>>()
    };
    let legacy_f = to_f32(&legacy);
    let cbt_f = to_f32(&cbt);
    assert_eq!(legacy_f.len(), cbt_f.len());
    // Readback is layer-major. A flat zip would also pair matching texels;
    // explicit chunks additionally let us require coverage in EVERY layer.
    let layer = 512 * 512;
    let legacy_valid = legacy_f
        .chunks(layer)
        .map(|c| c.iter().filter(|&&z| z > 0.0 && z < 1.0).count())
        .collect::<Vec<_>>();
    let cbt_valid = cbt_f
        .chunks(layer)
        .map(|c| c.iter().filter(|&&z| z > 0.0 && z < 1.0).count())
        .collect::<Vec<_>>();
    assert!(
        cbt_valid.iter().sum::<usize>() > 1000,
        "CBT caster must produce nonempty depth: {cbt_valid:?}"
    );
    // Independent canonical oracle: each chord triangle of a sphere with
    // pairwise distances <= D lies between radii sqrt(R^2-D^2/3) and R.
    // This follows from |sum(w_i*v_i)|^2 = R^2-sum(i<j,w_i*w_j*D_ij^2)
    // and sum(i<j,w_i*w_j)<=1/3. Test only rays through the central half-radius
    // disk to avoid the unbounded depth sensitivity at the silhouette.
    let outer_radius = radius + 20.0;
    let cpu = SphereTopology::uniform(4096, 12, 8).unwrap();
    let longest_edge = cpu
        .records()
        .iter()
        .filter_map(|r| {
            let node = Node::from_heap_id(identity(*r)).ok()?;
            cube_vertices(node)
        })
        .map(|t| {
            let v =
                t.map(|p| glam::DVec3::from_array(p.map(|x| x as f64)).normalize() * outer_radius);
            (v[0] - v[1])
                .length()
                .max((v[1] - v[2]).length())
                .max((v[2] - v[0]).length())
        })
        .fold(0.0_f64, f64::max);
    let inner_radius = (outer_radius * outer_radius - longest_edge * longest_edge / 3.0).sqrt();
    let mut canonical_rays = 0;
    let mut max_metric_error = 0.0_f64;
    let mut max_metric_envelope = 0.0_f64;
    let mut both = 0;
    let mut worst = 0.0f32;
    for (sun, (a_layer, b_layer)) in legacy_f.chunks(layer).zip(cbt_f.chunks(layer)).enumerate() {
        assert!(legacy_valid[sun] > 500 && cbt_valid[sun] > 500);
        let d = lights.directions[sun];
        let clip = thessa_render_core::shadow::directional_clip(
            view.world_position_m,
            glam::DVec3::new(f64::from(d[0]), f64::from(d[1]), f64::from(d[2])),
            settings.shadow_max_distance_m,
            settings.shadow_max_distance_m,
            512,
        )
        .unwrap()
        .as_mat4()
        .as_dmat4();
        let inverse = clip.inverse();
        for (pixel, &depth) in b_layer.iter().enumerate() {
            let xy = glam::DVec2::new(
                2.0 * ((pixel % 512) as f64 + 0.5) / 512.0 - 1.0,
                1.0 - 2.0 * ((pixel / 512) as f64 + 0.5) / 512.0,
            );
            let start = inverse.project_point3(xy.extend(0.0)) + view.world_position_m;
            let delta = inverse.project_point3(xy.extend(1.0)) + view.world_position_m - start;
            let ray_length = delta.length();
            let ray = delta / ray_length;
            let along = start.dot(ray);
            let impact2 = (start.length_squared() - along * along).max(0.0);
            if impact2 > (outer_radius * 0.5).powi(2) {
                continue;
            }
            let outer = -along - (outer_radius * outer_radius - impact2).sqrt();
            let inner = -along - (inner_radius * inner_radius - impact2).sqrt();
            let actual = f64::from(depth) * ray_length;
            // Two millimetres cover f32 projection/vertex arithmetic here,
            // separate from the geometry-derived chord envelope in metres.
            assert!(
                depth > 0.0 && depth < 1.0,
                "missing canonical caster sun={sun} pixel={pixel}"
            );
            assert!(
                actual >= outer - 0.002 && actual <= inner + 0.002,
                "canonical depth sun={sun} pixel={pixel}: error={} m, chord envelope={} m",
                actual - outer,
                inner - outer
            );
            canonical_rays += 1;
            max_metric_error = max_metric_error.max((actual - outer).abs());
            max_metric_envelope = max_metric_envelope.max(inner - outer);
        }
        for (&a, &b) in a_layer.iter().zip(b_layer) {
            if a > 0.0 && a < 1.0 && b > 0.0 && b < 1.0 {
                both += 1;
                worst = worst.max((a - b).abs());
            }
        }
    }
    assert!(both > 500, "both casters must overlap in depth: {both}");
    assert!(canonical_rays > 10_000);
    println!(
        "cbt_shadow_cast adapter={:?} slots={} draw_calls={} layers={} overlap={both} legacy_max_depth_delta={worst} canonical_rays={canonical_rays} max_metric_error_m={max_metric_error} max_chord_envelope_m={max_metric_envelope} arithmetic_tolerance_m=0.002 longest_edge_m={longest_edge}; canonical sphere oracle, legacy comparison diagnostic only; not a production mode or large_cbt/Bevy comparison",
        backend.adapter.get_info().name,
        stats.submitted_slots,
        stats.draw_calls,
        legacy_valid.len()
    );
}

// Independent f64 direct Fourier sums followed by box mips, periodic bilinear
// filtering and trilinear mip interpolation. Does not read the GPU field.
fn ocean_height_oracle(field: &thessa_render_wgpu::ocean::OceanField, time: f64) -> Vec<Vec<f64>> {
    let n = field.stats().size as usize;
    let mut mips = vec![
        (0..9 * n * n)
            .map(|i| {
                field.spectrum().sample_reference(
                    i / (n * n),
                    [(i % n) as f64 / n as f64, ((i / n) % n) as f64 / n as f64],
                    time,
                )[0]
            })
            .collect::<Vec<_>>(),
    ];
    let mut side = n;
    while side > 1 {
        let next = side / 2;
        let old = mips.last().unwrap();
        let mut data = Vec::new();
        for layer in 0..9 {
            for y in 0..next {
                for x in 0..next {
                    data.push(
                        (old[layer * side * side + 2 * y * side + 2 * x]
                            + old[layer * side * side + 2 * y * side + 2 * x + 1]
                            + old[layer * side * side + (2 * y + 1) * side + 2 * x]
                            + old[layer * side * side + (2 * y + 1) * side + 2 * x + 1])
                            * 0.25,
                    );
                }
            }
        }
        mips.push(data);
        side = next;
    }
    mips
}

fn sample_ocean_oracle(
    mips: &[Vec<f64>],
    layer: usize,
    position: [f64; 2],
    period: f64,
    footprint: f64,
) -> f64 {
    let n = (mips[0].len() / 9).isqrt();
    let lod = (footprint * n as f64 / period)
        .log2()
        .clamp(0.0, (mips.len() - 1) as f64);
    let sample = |mip: usize| {
        let side = n >> mip;
        let p = position.map(|v| (v / period + 0.5 / n as f64) * side as f64 - 0.5);
        let base = p.map(|v| v.floor() as i64);
        let t = p.map(|v| v - v.floor());
        let at = |x: i64, y: i64| {
            mips[mip][layer * side * side
                + y.rem_euclid(side as i64) as usize * side
                + x.rem_euclid(side as i64) as usize]
        };
        let mix = |a, b, t: f64| a + (b - a) * t;
        mix(
            mix(at(base[0], base[1]), at(base[0] + 1, base[1]), t[0]),
            mix(at(base[0], base[1] + 1), at(base[0] + 1, base[1] + 1), t[0]),
            t[1],
        )
    };
    let lo = lod.floor() as usize;
    let a = sample(lo);
    a + (sample(lod.ceil() as usize) - a) * lod.fract()
}

#[test]
#[ignore = "hardware: canonical CBT ocean geometry, Fourier oracle, seams, fallback and 98K-triangle scoped cost"]
fn conforming_ocean_borrows_field_without_cracks_or_accumulation() {
    use thessa_graphics::{Quality, RayTracingRequest, RequestedGraphics};
    use thessa_rcbt_core::{
        HeightPage,
        material_page::CbtMaterialPage,
        tile_anchor::{TileAnchor, TileKey},
    };
    use thessa_render_core::ocean::{PERIODS_M, SeaState};
    use thessa_render_wgpu::{
        conforming_surface::ConformingSurface,
        ocean::OceanField,
        terrain::{TerrainCapacity, TerrainPatch, TerrainRenderer},
    };
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Off;
    request.ocean.enabled = true;
    request.ocean.quality = Quality::Low;
    request.ocean.geometry_patches = 2;
    request.ocean.geometry_distance_m = 2000.0;
    let backend = Backend::bring_up_for_graphics_profiled(&request).unwrap();
    let settings = backend.resolve_graphics(&request);
    let sea = SeaState {
        gravity_m_s2: 6.4,
        depth_m: 80.0,
        wind_speed_m_s: 12.0,
        wind_direction_rad: 0.3,
        significant_height_m: 1.5,
        seed: 19,
    };
    let mut field = OceanField::new(&backend, sea, settings.ocean_size).unwrap();
    let radius = 1000.0;
    let camera = [0.0, 0.0, 1100.0];
    field
        .update_frame(&backend.queue, glam::DVec3::from_array(camera), radius)
        .unwrap();
    field.update(&backend, 0.0).unwrap();
    let page = HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
    let patches = (0..6)
        .map(|face| TerrainPatch {
            anchor: TileAnchor::new(TileKey::new(face, 0, 0, 0).unwrap(), radius).unwrap(),
            page: &page,
        })
        .collect::<Vec<_>>();
    let mut owner = TerrainRenderer::with_capacity(
        &backend,
        &patches,
        OffscreenTarget::FORMAT,
        TerrainCapacity {
            patches: 6,
            height_grid_size: 2,
            material_layers: 6,
        },
    )
    .unwrap();
    let demand = (8..14).collect::<Vec<_>>();
    let materials = |roughness: u8, generation| {
        demand
            .iter()
            .map(|&id| {
                (
                    id,
                    (
                        generation,
                        CbtMaterialPage::from_rgba8([20, 40, 80, roughness].repeat(128 * 128))
                            .unwrap(),
                    ),
                )
            })
            .collect()
    };
    owner
        .update_materials(&backend, &demand, &materials(30, 1))
        .unwrap();
    let binding = owner.material_binding();
    let source = owner.height_surface_source(&backend).unwrap();
    let gpu = SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 4096, 20, 8).unwrap();
    let datum = ConformingSurface::new(&backend, &gpu.pool, &source).unwrap();
    let mut water = ConformingSurface::new(&backend, &gpu.pool, &source).unwrap();
    for s in [&datum, &water] {
        s.update_camera(&backend, camera).unwrap();
    }
    water
        .set_ocean(&backend, &settings, Some(&field), &binding)
        .unwrap();
    assert_eq!(water.storage_bytes(), datum.storage_bytes() + 32);
    let capture = |surface: &ConformingSurface| {
        let mut encoder = backend.device.create_command_encoder(&Default::default());
        surface.encode(&mut encoder);
        backend.queue.submit([encoder.finish()]);
        read(&backend, &surface.vertices)
    };
    let base = capture(&datum);
    let displaced = capture(&water);
    let extent = words(&read(&backend, water.ocean_extent().unwrap()));
    let footprint = f32::from_bits(extent[0]);
    assert!(footprint > 0.0 && footprint < 2048.0 && extent[1] > 0 && extent[1] <= 4096);
    assert_eq!(
        displaced,
        capture(&water),
        "re-emission must not accumulate waves"
    );
    assert!(!field.update(&backend, 0.0).unwrap());
    assert_eq!(displaced, capture(&water), "paused field is bit stable");
    let mips = ocean_height_oracle(&field, 0.0);
    let base_words = words(&base);
    let water_words = words(&displaced);
    let mut shared = BTreeMap::new();
    let mut duplicates = 0;
    let mut moved = 0;
    let mut checked = 0;
    let mut worst = 0.0_f64;
    for (a, b) in base_words
        .as_chunks::<12>()
        .0
        .iter()
        .zip(water_words.as_chunks::<12>().0)
    {
        assert_eq!(
            &a[3..],
            &b[3..],
            "datum height, normals and address must survive deformation"
        );
        if a.iter().all(|&x| x == 0) {
            assert_eq!(a, b);
            continue;
        }
        let key = (a[7], a[8], a[9]);
        if let Some(old) = shared.insert(key, *b) {
            assert_eq!(old, *b, "shared vertex must be bit-identical");
            duplicates += 1;
        }
        let p = glam::DVec3::from_array(std::array::from_fn(|i| f64::from(f32::from_bits(a[i]))));
        let q = glam::DVec3::from_array(std::array::from_fn(|i| f64::from(f32::from_bits(b[i]))));
        let delta = (q - p).length();
        assert!(delta <= field.spectrum().height_bound_m + 0.002);
        if delta > 0.001 {
            moved += 1;
        }
        if checked < 32 && delta > 0.001 && duplicates % 31 == 0 {
            let radial = (p + glam::DVec3::from_array(camera)).normalize();
            let raw = radial.abs().powf(4.0);
            let w = raw / raw.length();
            let body = p + glam::DVec3::from_array(camera);
            let mut height = 0.0;
            for (band, period) in PERIODS_M.into_iter().enumerate() {
                height +=
                    w.x * sample_ocean_oracle(
                        &mips,
                        band,
                        [body.y, body.z],
                        period,
                        f64::from(footprint),
                    ) + w.y
                        * sample_ocean_oracle(
                            &mips,
                            3 + band,
                            [body.x, body.z],
                            period,
                            f64::from(footprint),
                        )
                        + w.z
                            * sample_ocean_oracle(
                                &mips,
                                6 + band,
                                [body.x, body.y],
                                period,
                                f64::from(footprint),
                            );
            }
            let t = ((p.length() - 1000.0) / 1000.0).clamp(0.0, 1.0);
            height *= 1.0 - t * t * (3.0 - 2.0 * t);
            let error = (q - p - radial * height).length();
            worst = worst.max(error);
            assert!(
                error < 0.004,
                "direct Fourier+filter displacement error={error} m"
            );
            checked += 1;
        }
    }
    assert!(
        duplicates > 1000 && moved > 100 && checked == 32,
        "duplicates={duplicates} moved={moved} oracle={checked}"
    );
    field.update(&backend, 0.5).unwrap();
    assert_ne!(displaced, capture(&water));
    field.update(&backend, 0.0).unwrap();
    assert_eq!(displaced, capture(&water));
    owner
        .update_materials(&backend, &demand, &materials(255, 2))
        .unwrap();
    assert_eq!(
        base,
        capture(&water),
        "dry canonical material must not move"
    );
    owner
        .update_materials(&backend, &demand, &materials(30, 3))
        .unwrap();
    let land = HeightPage::bake(&[5.0; 4], 2, 0.001).unwrap();
    owner
        .update_pages(
            &backend,
            &(0..6).map(|slot| (slot, &land)).collect::<Vec<_>>(),
        )
        .unwrap();
    assert_eq!(
        capture(&datum),
        capture(&water),
        "elevated land must not move"
    );
    owner
        .update_pages(
            &backend,
            &(0..6).map(|slot| (slot, &page)).collect::<Vec<_>>(),
        )
        .unwrap();
    let mut fallback = settings.clone();
    fallback.ocean.geometry_patches = 1;
    water
        .set_ocean(&backend, &fallback, Some(&field), &binding)
        .unwrap();
    assert_eq!(
        base,
        capture(&water),
        "whole-partition budget overflow must be optical-only"
    );
    assert!(words(&read(&backend, water.ocean_extent().unwrap()))[1] > 2048);
    let mut invalid = settings.clone();
    invalid.ocean.geometry_distance_m = f64::NAN;
    assert!(
        water
            .set_ocean(&backend, &invalid, Some(&field), &binding)
            .is_err()
    );
    assert!(
        water
            .set_ocean(&backend, &settings, None, &binding)
            .is_err()
    );
    // Failed configuration preserves the previous optical-only budget policy.
    assert_eq!(base, capture(&water));
    for optical_only in [true, false] {
        fallback = settings.clone();
        if optical_only {
            fallback.ocean.geometry_patches = 0;
        } else {
            fallback.ocean.enabled = false;
        }
        water
            .set_ocean(&backend, &fallback, None, &binding)
            .unwrap();
        assert!(water.ocean_extent().is_none());
        assert_eq!(datum.storage_bytes(), water.storage_bytes());
        assert_eq!(base, capture(&water));
    }
    // Mixed-depth topology uses one extent too: differing incident leaf depths
    // must not choose differing wave LODs at a common canonical vertex.
    water
        .set_ocean(&backend, &settings, Some(&field), &binding)
        .unwrap();
    let before_mixed = records(&backend, &gpu);
    let node = before_mixed
        .iter()
        .find_map(|&r| {
            let node = Node::from_heap_id(identity(r)).ok()?;
            cube_vertices(node)?;
            Some(node)
        })
        .unwrap();
    gpu.set_request(&backend.queue, 0, node.id());
    step(&backend, &gpu);
    let after_mixed = records(&backend, &gpu);
    assert!(
        before_mixed != after_mixed,
        "valid split must change topology"
    );
    let mut cpu = SphereTopology::uniform(4096, 20, 8).unwrap();
    cpu.split(node).unwrap();
    assert_eq!(logical(&after_mixed), logical(&cpu.records()));
    let mixed = words(&capture(&water));
    let mut vertices = BTreeMap::new();
    for v in mixed.as_chunks::<12>().0 {
        if v.iter().any(|&x| x != 0)
            && let Some(old) = vertices.insert((v[7], v[8], v[9]), *v)
        {
            assert_eq!(old, *v);
        }
    }
    let mut calm = OceanField::new(
        &backend,
        SeaState {
            significant_height_m: 0.0,
            ..sea
        },
        settings.ocean_size,
    )
    .unwrap();
    calm.update_frame(&backend.queue, glam::DVec3::from_array(camera), radius)
        .unwrap();
    calm.update(&backend, 0.0).unwrap();
    water
        .set_ocean(&backend, &settings, Some(&calm), &binding)
        .unwrap();
    assert_eq!(
        capture(&datum),
        capture(&water),
        "zero sea state is exact datum"
    );
    water
        .set_ocean(&backend, &settings, Some(&field), &binding)
        .unwrap();
    let far = [0.0, 0.0, 10000.0];
    field
        .update_frame(&backend.queue, glam::DVec3::from_array(far), radius)
        .unwrap();
    for s in [&datum, &water] {
        s.update_camera(&backend, far).unwrap();
    }
    assert_eq!(
        capture(&datum),
        capture(&water),
        "outside geometry distance is exact datum"
    );
    assert_eq!(
        words(&read(&backend, water.ocean_extent().unwrap()))[1],
        0,
        "extent resets, no stale near count"
    );
    field
        .update_frame(&backend.queue, glam::DVec3::from_array(camera), radius)
        .unwrap();
    // Coarsest real roots are valid geometry but cannot carry resolved waves.
    let coarse = SphereGpu::new(&backend.device, &backend.queue, 16, 20).unwrap();
    let mut coarse_surface = ConformingSurface::new(&backend, &coarse.pool, &source).unwrap();
    coarse_surface.update_camera(&backend, camera).unwrap();
    let coarse_base = capture(&coarse_surface);
    assert_eq!(
        coarse_base
            .as_chunks::<144>()
            .0
            .iter()
            .filter(|v| v.iter().any(|&b| b != 0))
            .count(),
        12
    );
    coarse_surface
        .set_ocean(&backend, &settings, Some(&field), &binding)
        .unwrap();
    assert_eq!(coarse_base, capture(&coarse_surface));
    assert!(
        f32::from_bits(words(&read(&backend, coarse_surface.ocean_extent().unwrap()))[0]) >= 2048.0
    );
    owner.publish_partition(&backend, &[]).unwrap();
    for s in [&datum, &water] {
        s.update_camera(&backend, camera).unwrap();
    }
    assert_eq!(
        capture(&datum),
        capture(&water),
        "evicted height cannot acquire invented water geometry"
    );
    let publications = patches
        .iter()
        .map(|p| thessa_render_wgpu::terrain::ResidentTerrainPatch {
            anchor: p.anchor,
            page: p.page,
            generation: 10,
        })
        .collect::<Vec<_>>();
    owner.publish_partition(&backend, &publications).unwrap();
    // Target-size end-to-end emission + extent + deformation timestamps. No
    // claim about production frames, shadow/RT costs or equal-error renderers.
    let large =
        SphereGpu::with_uniform_depth(&backend.device, &backend.queue, 131072, 20, 13).unwrap();
    let mut target = ConformingSurface::new(&backend, &large.pool, &source).unwrap();
    target.update_camera(&backend, camera).unwrap();
    let mut large_settings = settings.clone();
    large_settings.ocean.geometry_patches = 64;
    target
        .set_ocean(&backend, &large_settings, Some(&field), &binding)
        .unwrap();
    let target_datum = ConformingSurface::new(&backend, &large.pool, &source).unwrap();
    target_datum.update_camera(&backend, camera).unwrap();
    let timer = GpuIntervals::new(&backend, 2).unwrap();
    let mut samples = Vec::new();
    let mut baseline_samples = Vec::new();
    for i in 0..50 {
        for (surface, costs) in [
            (&target_datum, &mut baseline_samples),
            (&target, &mut samples),
        ] {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            timer.mark(&mut encoder, 0);
            surface.encode(&mut encoder);
            timer.mark(&mut encoder, 1);
            timer.resolve(&mut encoder);
            backend.queue.submit([encoder.finish()]);
            let ms = timer.read(&backend).unwrap()[0] * 1000.0;
            if i >= 10 {
                costs.push(ms);
            }
        }
    }
    let directory =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/test-tmp/cbt-sphere");
    std::fs::create_dir_all(&directory).unwrap();
    std::fs::write(
        directory.join("surface-ocean-cost-ms.csv"),
        "datum_ms,ocean_ms\n".to_owned()
            + &baseline_samples
                .iter()
                .zip(&samples)
                .map(|(a, b)| format!("{a},{b}\n"))
                .collect::<String>(),
    )
    .unwrap();
    samples.sort_by(f64::total_cmp);
    baseline_samples.sort_by(f64::total_cmp);
    let large_extent = words(&read(&backend, target.ocean_extent().unwrap()));
    assert!(large_extent[1] > 1000 && large_extent[1] <= 131072);
    println!(
        "cbt_ocean adapter={:?} backend={:?} oracle_vertices={checked} max_error_m={worst} bound_m=0.004 duplicate_records={duplicates} moved_records={moved} footprint_m={footprint} candidates={} target_triangles=98304 target_candidates={} target_footprint_m={} samples=40 warmup=10 emission_extent_deform_p50_ms={} p95_ms={} datum_p50_ms={} datum_p95_ms={} extra_buffer_bytes=32; paired same-source baseline, diagnostic only, no production mode or frame-FPS claim",
        backend.adapter.get_info().name,
        backend.adapter.get_info().backend,
        extent[1],
        large_extent[1],
        f32::from_bits(large_extent[0]),
        samples[20],
        samples[38],
        baseline_samples[20],
        baseline_samples[38]
    );
}
