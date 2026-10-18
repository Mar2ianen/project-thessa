//! Experimental persistent topology, no patch geometry, skirts or grid steps.
//! Not a production terrain/ocean/RT replacement or large_cbt parity claim.
use glam::Vec3;
use std::collections::BTreeMap;
use thessa_rcbt_core::{
    Node,
    conforming::{Record, SphereTopology, cube_vertices, root_index},
};
use thessa_rcbt_wgpu::sphere::SphereGpu;
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
