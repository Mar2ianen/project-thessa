//! Minimal ray-query stack probe: one triangle BLAS, one TLAS instance, one
//! compute dispatch. Kept separate from the reflection oracles so a stack
//! regression (wgpu/naga/driver) is distinguishable from a renderer bug.
//!
//! It pins four facts the RT scene silently depends on:
//! - opaque triangle geometry in a single-geometry BLAS is traceable;
//! - instance custom data round-trips into `instance_custom_data`;
//! - the TLAS instance transform is a row-major 3x4 affine (translation in the
//!   fourth column), which is what `RayScene` writes;
//! - barycentrics and the primitive index address the triangle the ray hits.
use thessa_graphics::{RayTracingRequest, RequestedGraphics};
use thessa_render_wgpu::Backend;
use wgpu::util::DeviceExt;

#[derive(Debug, PartialEq)]
struct Trace {
    kind: u32,
    t: f32,
    custom_data: u32,
    geometry: u32,
    primitive: u32,
    barycentrics: [f32; 2],
}

#[test]
fn ray_query_contract_holds_on_this_stack() {
    let mut requested = RequestedGraphics::default();
    requested.renderer.ray_tracing = RayTracingRequest::Local;
    let Ok(backend) = Backend::bring_up_for_graphics(&requested) else {
        eprintln!("SKIP: no ray-query capable adapter");
        return;
    };
    let device = &backend.device;
    let queue = &backend.queue;
    if !device
        .features()
        .contains(wgpu::Features::EXPERIMENTAL_RAY_QUERY)
    {
        eprintln!("SKIP: adapter without ray query");
        return;
    }

    // Triangle in the object-space plane x = 1; a ray along +X at
    // (0, 0.25, 0.25) hits it at t = 1 with barycentrics (0.25, 0.25).
    // Positions are padded to the 16-byte stride the renderer uses.
    let vertices: [f32; 12] = [1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0];
    let indices: [u32; 3] = [0, 1, 2];
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("probe-vertices"),
        contents: &f32_bytes(&vertices),
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("probe-indices"),
        contents: &u32_bytes(&indices),
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::BLAS_INPUT,
    });
    let size = wgpu::BlasTriangleGeometrySizeDescriptor {
        vertex_format: wgpu::VertexFormat::Float32x3,
        vertex_count: 3,
        index_format: Some(wgpu::IndexFormat::Uint32),
        index_count: Some(3),
        // Without OPAQUE a derived ray query never commits the candidate hit
        // and the traversal reports nothing at all.
        flags: wgpu::AccelerationStructureGeometryFlags::OPAQUE,
    };
    let blas = device.create_blas(
        &wgpu::CreateBlasDescriptor {
            label: Some("probe-blas"),
            flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
            update_mode: wgpu::AccelerationStructureUpdateMode::Build,
        },
        wgpu::BlasGeometrySizeDescriptors::Triangles {
            descriptors: vec![size.clone()],
        },
    );
    // Translation-only instance transform: the fourth column of a row-major
    // 3x4 moves the triangle to +X. A column-major reading would leave it at
    // the origin and every ray below would miss.
    let offset = [10.0f32, 20.0, 30.0];
    let transform = [
        1.0, 0.0, 0.0, offset[0], 0.0, 1.0, 0.0, offset[1], 0.0, 0.0, 1.0, offset[2],
    ];
    let mut tlas = device.create_tlas(&wgpu::CreateTlasDescriptor {
        label: Some("probe-tlas"),
        max_instances: 1,
        flags: wgpu::AccelerationStructureFlags::PREFER_FAST_TRACE,
        update_mode: wgpu::AccelerationStructureUpdateMode::Build,
    });
    tlas[0] = Some(wgpu::TlasInstance::new(&blas, transform, 7, 0xFF));

    let out = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("probe-out"),
        size: 32,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("probe-readback"),
        size: 32,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("probe"),
        source: wgpu::ShaderSource::Wgsl(
            format!(
                r#"
enable wgpu_ray_query;
@group(0) @binding(0) var accel: acceleration_structure;
@group(0) @binding(1) var<storage, read_write> words: array<u32>;
@compute @workgroup_size(1)
fn main() {{
    let origin = vec3<f32>({ox}, {oy}, {oz});
    var query: ray_query;
    rayQueryInitialize(
        &query,
        accel,
        RayDesc(0u, 0xFFu, 0.001, 100.0, origin, vec3<f32>(1.0, 0.0, 0.0)),
    );
    rayQueryProceed(&query);
    let hit = rayQueryGetCommittedIntersection(&query);
    words[0] = hit.kind;
    words[1] = bitcast<u32>(hit.t);
    words[2] = hit.instance_custom_data;
    words[3] = hit.geometry_index;
    words[4] = hit.primitive_index;
    words[5] = bitcast<u32>(hit.barycentrics.x);
    words[6] = bitcast<u32>(hit.barycentrics.y);
}}
"#,
                ox = offset[0],
                oy = offset[1] + 0.25,
                oz = offset[2] + 0.25,
            )
            .into(),
        ),
    });
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("probe"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::AccelerationStructure {
                    vertex_return: false,
                },
                count: None,
            },
            storage_entry(1),
        ],
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("probe"),
        layout: &layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::AccelerationStructure(&tlas),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: out.as_entire_binding(),
            },
        ],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("probe"),
        bind_group_layouts: &[Some(&layout)],
        immediate_size: 0,
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("probe"),
        layout: Some(&pipeline_layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: Default::default(),
        cache: None,
    });
    let mut encoder = device.create_command_encoder(&Default::default());
    let entries = [wgpu::BlasBuildEntry {
        blas: &blas,
        geometry: wgpu::BlasGeometries::TriangleGeometries(vec![wgpu::BlasTriangleGeometry {
            size: &size,
            vertex_buffer: &vertex_buffer,
            first_vertex: 0,
            vertex_stride: 16,
            index_buffer: Some(&index_buffer),
            first_index: Some(0),
            transform_buffer: None,
            transform_buffer_offset: None,
        }]),
    }];
    encoder.build_acceleration_structures(&entries, [&tlas]);
    {
        let mut pass = encoder.begin_compute_pass(&Default::default());
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.dispatch_workgroups(1, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&out, 0, &readback, 0, 32);
    queue.submit([encoder.finish()]);

    let slice = readback.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    let data = slice.get_mapped_range().unwrap();
    let u32_at = |index: usize| {
        u32::from_le_bytes([
            data[index * 4],
            data[index * 4 + 1],
            data[index * 4 + 2],
            data[index * 4 + 3],
        ])
    };
    let trace = Trace {
        kind: u32_at(0),
        t: f32::from_bits(u32_at(1)),
        custom_data: u32_at(2),
        geometry: u32_at(3),
        primitive: u32_at(4),
        barycentrics: [f32::from_bits(u32_at(5)), f32::from_bits(u32_at(6))],
    };
    drop(data);
    readback.unmap();

    assert_eq!(
        trace.kind, 1,
        "expected a committed triangle intersection, got {trace:?}"
    );
    assert!(
        (trace.t - 1.0).abs() < 1e-3,
        "translated instance must sit at t = 1, got {trace:?}"
    );
    assert_eq!(trace.custom_data, 7, "instance custom data must round-trip");
    assert_eq!(trace.geometry, 0, "single-geometry BLAS reports geometry 0");
    assert_eq!(trace.primitive, 0, "the only triangle is primitive 0");
    assert!(
        (trace.barycentrics[0] - 0.25).abs() < 1e-3 && (trace.barycentrics[1] - 0.25).abs() < 1e-3,
        "hit at the ray's own barycentrics, got {trace:?}"
    );
}

fn storage_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only: false },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn f32_bytes(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

fn u32_bytes(values: &[u32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_le_bytes()).collect()
}
