//! Bevy render-world bridge for exact CBT leaves and GPU terrain geometry.
//!
//! The main world owns topology decisions and baked [`HeightPage`] payloads.
//! Extraction copies those immutable snapshots into the render world. A single
//! generation-gated compute pass then expands every available leaf page into
//! a fixed local vertex grid and an indexed indirect draw command. Missing
//! pages produce zero-count commands, so the CPU mesh fallback can coexist
//! while the GPU path is being validated.

use std::{borrow::Cow, collections::HashMap};

use bevy::{
    app::{App, Plugin},
    ecs::{change_detection::DetectChanges, schedule::IntoScheduleConfigs},
    prelude::{FromWorld, Res, ResMut, Resource, World},
    render::{
        GpuResourceAppExt, Render, RenderApp, RenderStartup, RenderSystems,
        camera::ExtractedCamera,
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        render_resource::{
            BindGroupEntry, BindGroupLayout, BindGroupLayoutEntry, BindingResource, BindingType,
            Buffer, BufferBindingType, BufferUsages, ColorTargetState, ColorWrites,
            CompareFunction, ComputePassDescriptor, ComputePipeline, DepthBiasState,
            DepthStencilState, DownlevelFlags, MultisampleState, PipelineLayout,
            PipelineLayoutDescriptor, PrimitiveState, PrimitiveTopology, RawBufferVec,
            RawComputePipelineDescriptor, RawFragmentState, RawRenderPipelineDescriptor,
            RawVertexState, RenderPassDescriptor, RenderPipeline, ShaderModule,
            ShaderModuleDescriptor, ShaderSource, ShaderStages, StoreOp, TextureFormat,
        },
        renderer::{
            RenderAdapter, RenderContext, RenderDevice, RenderGraph, RenderGraphSystems,
            RenderQueue, ViewQuery,
        },
        view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms},
    },
};

use thessa_rcbt_core::HeightPage;

#[cfg(feature = "mesh-shaders")]
use bevy::render::render_resource::WgpuFeatures;

use super::{CbtLeafRecord, CbtRenderPages, CbtRenderSurface, CbtRenderTopology};

const GPU_GRID_SIZE: usize = 33;
const GPU_VERTEX_COUNT_PER_PATCH: usize = GPU_GRID_SIZE * GPU_GRID_SIZE;
#[cfg(feature = "mesh-shaders")]
const MESHLET_CELLS: u32 = 8;
#[cfg(feature = "mesh-shaders")]
const MESHLET_GRID_SIZE: u32 = MESHLET_CELLS + 1;
#[cfg(feature = "mesh-shaders")]
const MESHLET_VERTEX_COUNT: u32 = MESHLET_GRID_SIZE * MESHLET_GRID_SIZE;
#[cfg(feature = "mesh-shaders")]
const MESHLET_PRIMITIVE_COUNT: u32 = MESHLET_CELLS * MESHLET_CELLS * 2;
#[cfg(feature = "mesh-shaders")]
const MESHLETS_PER_PATCH: u32 = 4 * 4;

impl ExtractResource for CbtRenderTopology {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        source.clone()
    }
}

impl ExtractResource for CbtRenderPages {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        source.clone()
    }
}

impl ExtractResource for CbtRenderSurface {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        *source
    }
}

/// GPU-side CBT transport and geometry buffers prepared in the Bevy render
/// world.
///
/// The vertex buffer stores two `vec4<f32>` values per vertex: position and
/// normal. The raster consumer uses one procedural indirect draw with one
/// instance per leaf; the GPU reconstructs the shared 32x32 cell index pattern
/// in the vertex shader. This keeps the leaf stream depth-agnostic without
/// issuing one draw command per leaf.
#[derive(Resource)]
pub struct CbtGpuBuffers {
    leaf_records: RawBufferVec<CbtLeafRecord>,
    patch_records: RawBufferVec<CbtLeafRecord>,
    page_metadata: RawBufferVec<[u32; 4]>,
    page_residuals: RawBufferVec<u32>,
    vertices: RawBufferVec<[f32; 8]>,
    draw_list: RawBufferVec<[u32; 4]>,
    params: RawBufferVec<[u32; 4]>,
    surface_transform: RawBufferVec<[f32; 16]>,
    topology_generation: u64,
    pages_generation: u64,
    surface_generation: u64,
    transform_generation: u64,
    generated_topology_generation: u64,
    generated_pages_generation: u64,
    generated_surface_generation: u64,
    leaf_count: u32,
}

impl FromWorld for CbtGpuBuffers {
    fn from_world(_world: &mut World) -> Self {
        let mut leaf_records = RawBufferVec::new(BufferUsages::STORAGE);
        leaf_records.set_label(Some("thessa-cbt-leaf-records"));
        let mut patch_records = RawBufferVec::new(BufferUsages::STORAGE);
        patch_records.set_label(Some("thessa-cbt-patch-records"));
        let mut page_metadata = RawBufferVec::new(BufferUsages::STORAGE);
        page_metadata.set_label(Some("thessa-cbt-height-page-metadata"));
        let mut page_residuals = RawBufferVec::new(BufferUsages::STORAGE);
        page_residuals.set_label(Some("thessa-cbt-height-page-residuals"));
        let mut vertices = RawBufferVec::new(BufferUsages::STORAGE | BufferUsages::VERTEX);
        vertices.set_label(Some("thessa-cbt-generated-vertices"));
        let mut draw_list = RawBufferVec::new(BufferUsages::STORAGE | BufferUsages::INDIRECT);
        draw_list.set_label(Some("thessa-cbt-indirect-draw-list"));
        let mut params = RawBufferVec::new(BufferUsages::UNIFORM);
        params.set_label(Some("thessa-cbt-geometry-params"));
        let mut surface_transform = RawBufferVec::new(BufferUsages::UNIFORM);
        surface_transform.set_label(Some("thessa-cbt-surface-transform"));
        Self {
            leaf_records,
            patch_records,
            page_metadata,
            page_residuals,
            vertices,
            draw_list,
            params,
            surface_transform,
            topology_generation: u64::MAX,
            pages_generation: u64::MAX,
            surface_generation: u64::MAX,
            transform_generation: u64::MAX,
            generated_topology_generation: u64::MAX,
            generated_pages_generation: u64::MAX,
            generated_surface_generation: u64::MAX,
            leaf_count: 0,
        }
    }
}

impl CbtGpuBuffers {
    /// Storage buffer containing `[node_id_lo, node_id_hi, depth, ordinal]`.
    pub fn leaf_buffer(&self) -> Option<&Buffer> {
        self.leaf_records.buffer()
    }

    /// Storage buffer containing the exact leaf records copied by compute.
    pub fn patch_buffer(&self) -> Option<&Buffer> {
        self.patch_records.buffer()
    }

    /// Packed `[base_height_bits, residual_scale_bits, grid_size, word_offset]`
    /// records, one per CBT leaf ordinal.
    pub fn page_metadata_buffer(&self) -> Option<&Buffer> {
        self.page_metadata.buffer()
    }

    /// Two signed 16-bit residuals packed into each storage word.
    pub fn page_residual_buffer(&self) -> Option<&Buffer> {
        self.page_residuals.buffer()
    }

    /// Interleaved `[position vec4, normal vec4]` generated by the GPU.
    pub fn vertex_buffer(&self) -> Option<&Buffer> {
        self.vertices.buffer()
    }

    /// One `DrawIndirect` command. Its `instance_count` is the exact leaf
    /// count; missing pages become degenerate instances in the shader.
    pub fn draw_list_buffer(&self) -> Option<&Buffer> {
        self.draw_list.buffer()
    }

    /// Uniform-compatible `[leaf_count, vertices_per_patch, radius_bits, 0]`.
    pub fn params_buffer(&self) -> Option<&Buffer> {
        self.params.buffer()
    }

    /// Body-space to Bevy render-local transform used by the raster shader.
    pub fn surface_transform_buffer(&self) -> Option<&Buffer> {
        self.surface_transform.buffer()
    }

    pub fn generation(&self) -> u64 {
        self.topology_generation
    }

    pub fn leaf_count(&self) -> u32 {
        self.leaf_count
    }

    pub fn patches_generated_for(&self) -> Option<u64> {
        (self.generated_topology_generation != u64::MAX)
            .then_some(self.generated_topology_generation)
    }

    pub fn geometry_generated_for(&self) -> Option<(u64, u64, u64)> {
        (self.generated_topology_generation != u64::MAX).then_some((
            self.generated_topology_generation,
            self.generated_pages_generation,
            self.generated_surface_generation,
        ))
    }
}

#[derive(Resource)]
struct CbtGpuPipeline {
    bind_group_layout: BindGroupLayout,
    pipeline: ComputePipeline,
}

#[derive(Resource)]
struct CbtGpuRasterPipeline {
    bind_group_layout: BindGroupLayout,
    pipeline_layout: PipelineLayout,
    shader: ShaderModule,
    pipelines: HashMap<TextureFormat, RenderPipeline>,
}

#[cfg(feature = "mesh-shaders")]
#[derive(Resource)]
struct CbtGpuMeshPipeline {
    bind_group_layout: BindGroupLayout,
    view_bind_group_layout: BindGroupLayout,
    pipeline_layout: PipelineLayout,
    shader: ShaderModule,
    pipelines: HashMap<TextureFormat, RenderPipeline>,
}

/// One invocation writes one vertex. The page is sampled in its original
/// quantized representation; no CPU-side height expansion or dense CBT
/// reconstruction is performed.
const CBT_GEOMETRY_WGSL: &str = r#"
struct Params {
    leaf_count: u32,
    vertices_per_patch: u32,
    radius_bits: u32,
    _padding: u32,
};

struct DrawCommand {
    vertex_count: u32,
    instance_count: u32,
    first_vertex: u32,
    first_instance: u32,
};

@group(0) @binding(0) var<storage, read> leaves: array<vec4<u32>>;
@group(0) @binding(1) var<storage, read_write> patches: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(3) var<storage, read> page_residuals: array<u32>;
@group(0) @binding(4) var<storage, read_write> vertices: array<vec4<f32>>;
@group(0) @binding(5) var<storage, read_write> draw_list: array<DrawCommand>;
@group(0) @binding(6) var<uniform> params: Params;

fn residual(page: vec4<u32>, sample_index: u32) -> i32 {
    let word = page_residuals[page.w + sample_index / 2u];
    let raw = (word >> ((sample_index & 1u) * 16u)) & 0xffffu;
    return select(i32(raw), i32(raw) - 65536, raw >= 32768u);
}

fn page_sample(page: vec4<u32>, u: f32, v: f32) -> f32 {
    if (page.z == 0u) {
        return 0.0;
    }
    let max_coord = f32(page.z - 1u);
    let px = clamp(u, 0.0, 1.0) * max_coord;
    let py = clamp(v, 0.0, 1.0) * max_coord;
    let x = u32(floor(px));
    let y = u32(floor(py));
    let x1 = min(x + 1u, page.z - 1u);
    let y1 = min(y + 1u, page.z - 1u);
    let tx = px - f32(x);
    let ty = py - f32(y);
    let grid = page.z;
    let h00 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x)) * bitcast<f32>(page.y);
    let h10 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x1)) * bitcast<f32>(page.y);
    let h01 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x)) * bitcast<f32>(page.y);
    let h11 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x1)) * bitcast<f32>(page.y);
    let top = h00 + (h10 - h00) * tx;
    let bottom = h01 + (h11 - h01) * tx;
    return top + (bottom - top) * ty;
}

fn payload_bit(low: u32, high: u32, bit: u32) -> u32 {
    if (bit < 32u) {
        return (low >> bit) & 1u;
    }
    return (high >> (bit - 32u)) & 1u;
}

// Decode the cube-face and Morton coordinates from the exact heap id without
// using a shader u64. The CPU record stores the id as two u32 words.
fn tile_coordinates(record: vec4<u32>) -> vec4<u32> {
    let depth = record.z;
    if (depth < 3u || ((depth - 3u) & 1u) != 0u) {
        return vec4(0u);
    }
    var low = record.x;
    var high = record.y;
    if (depth < 32u) {
        low = low - (1u << depth);
    } else {
        high = high - (1u << (depth - 32u));
    }
    let path_bits = depth - 3u;
    var face = 0u;
    if (path_bits < 32u) {
        face = (low >> path_bits) | (high << (32u - path_bits));
    } else if (path_bits == 32u) {
        face = high;
    } else {
        face = high >> (path_bits - 32u);
    }
    let level = path_bits / 2u;
    var tile_x = 0u;
    var tile_y = 0u;
    for (var i = 0u; i < 17u; i = i + 1u) {
        if (i < level) {
            let shift = (level - i - 1u) * 2u;
            tile_x = tile_x * 2u + payload_bit(low, high, shift + 1u);
            tile_y = tile_y * 2u + payload_bit(low, high, shift);
        }
    }
    return vec4(tile_x, tile_y, level, face & 7u);
}

fn face_direction(face: u32, a: f32, b: f32) -> vec3<f32> {
    var raw = vec3(a, b, 1.0);
    if (face == 0u) { raw = vec3(1.0, b, -a); }
    if (face == 1u) { raw = vec3(-1.0, b, a); }
    if (face == 2u) { raw = vec3(a, 1.0, -b); }
    if (face == 3u) { raw = vec3(a, -1.0, b); }
    if (face == 4u) { raw = vec3(a, b, 1.0); }
    if (face == 5u) { raw = vec3(-a, b, -1.0); }
    return normalize(raw);
}

fn surface_position(tile: vec4<u32>, u: f32, v: f32, height: f32, radius: f32) -> vec3<f32> {
    let scale = exp2(f32(tile.z));
    let a = 2.0 * (f32(tile.x) + u) / scale - 1.0;
    let b = 2.0 * (f32(tile.y) + v) / scale - 1.0;
    return face_direction(tile.w, a, b) * (radius + height);
}

@compute @workgroup_size(64)
fn build_geometry(@builtin(global_invocation_id) gid: vec3<u32>) {
    let index = gid.x;
    let total = params.leaf_count * params.vertices_per_patch;
    if (index >= total) {
        return;
    }
    let ordinal = index / params.vertices_per_patch;
    let local = index % params.vertices_per_patch;
    let page = page_metadata[ordinal];
    if (local == 0u) {
        patches[ordinal] = leaves[ordinal];
    }
    if (index == 0u) {
        draw_list[0] = DrawCommand(
            6144u,
            params.leaf_count,
            0u,
            0u,
        );
    }
    if (page.z == 0u) {
        vertices[index * 2u] = vec4(0.0);
        vertices[index * 2u + 1u] = vec4(0.0);
        return;
    }
    let tile = tile_coordinates(leaves[ordinal]);
    let gx = local % 33u;
    let gy = local / 33u;
    let uv = vec2(f32(gx) / 32.0, f32(gy) / 32.0);
    let radius = bitcast<f32>(params.radius_bits);
    let h = page_sample(page, uv.x, uv.y);
    let p = surface_position(tile, uv.x, uv.y, h, radius);
    let du = 1.0 / 32.0;
    let puv = vec2(min(uv.x + du, 1.0), uv.y);
    let pvv = vec2(uv.x, min(uv.y + du, 1.0));
    let pu = surface_position(tile, puv.x, puv.y, page_sample(page, puv.x, puv.y), radius);
    let pv = surface_position(tile, pvv.x, pvv.y, page_sample(page, pvv.x, pvv.y), radius);
    let normal = normalize(cross(pu - p, pv - p));
    vertices[index * 2u] = vec4(p, 1.0);
    vertices[index * 2u + 1u] = vec4(normal, 0.0);
}
"#;

/// Procedural direct raster consumer for the generated GPU geometry. It uses
/// one indirect draw and one instance per leaf, matching the important draw
/// submission property of large_cbt while retaining the current page format.
const CBT_RASTER_WGSL: &str = r#"
struct Params {
    leaf_count: u32,
    vertices_per_patch: u32,
    radius_bits: u32,
    _padding: u32,
};

struct ViewUniforms {
    clip_from_world: mat4x4<f32>,
};

@group(0) @binding(0) var<uniform> view: ViewUniforms;
@group(0) @binding(1) var<uniform> render_from_body: mat4x4<f32>;
@group(0) @binding(2) var<storage, read> generated_vertices: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(4) var<uniform> params: Params;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

fn grid_vertex(local: u32) -> u32 {
    let cell = local / 6u;
    let corner = local % 6u;
    let x = cell % 32u;
    let y = cell / 32u;
    let a = y * 33u + x;
    let b = a + 1u;
    let c = a + 33u;
    let d = c + 1u;
    if (corner == 0u) { return a; }
    if (corner == 1u) { return b; }
    if (corner == 2u) { return c; }
    if (corner == 3u) { return b; }
    if (corner == 4u) { return d; }
    return c;
}

@vertex
fn vertex(
    @builtin(vertex_index) vertex_index: u32,
    @builtin(instance_index) instance_index: u32,
) -> VertexOutput {
    var output: VertexOutput;
    let page = page_metadata[instance_index];
    let local = grid_vertex(vertex_index);
    let generated_index = instance_index * params.vertices_per_patch + local;
    let position = generated_vertices[generated_index * 2u];
    let normal = generated_vertices[generated_index * 2u + 1u];
    let render_position = select(vec4(0.0), render_from_body * position, page.z != 0u);
    output.clip_position = view.clip_from_world * render_position;
    output.normal = normalize((render_from_body * vec4(normal.xyz, 0.0)).xyz);
    return output;
}

@fragment
fn fragment(input: VertexOutput) -> @location(0) vec4<f32> {
    let light = normalize(vec3(0.35, 0.8, 0.45));
    let diffuse = 0.24 + 0.76 * max(dot(normalize(input.normal), light), 0.0);
    return vec4(vec3(0.20, 0.34, 0.17) * diffuse, 1.0);
}
"#;

/// Hardware mesh-shader consumer for the same quantized CBT pages. A 33x33
/// patch is emitted as sixteen 8x8 meshlets, so the shader stays within the
/// recommended 256-vertex / 256-primitive minimum while preserving exact page
/// sampling. There is deliberately no task shader yet: a single direct mesh
/// dispatch covers all leaves and the page metadata turns missing pages into
/// zero-output workgroups.
#[cfg(feature = "mesh-shaders")]
const CBT_MESH_WGSL: &str = r#"
enable wgpu_mesh_shader;

struct Params {
    leaf_count: u32,
    vertices_per_patch: u32,
    radius_bits: u32,
    _padding: u32,
};

struct ViewUniforms {
    clip_from_world: mat4x4<f32>,
};

struct MeshVertex {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

struct MeshPrimitive {
    @builtin(triangle_indices) indices: vec3<u32>,
};

struct MeshOutput {
    @builtin(vertex_count) vertex_count: u32,
    @builtin(primitive_count) primitive_count: u32,
    @builtin(vertices) vertices: array<MeshVertex, 81>,
    @builtin(primitives) primitives: array<MeshPrimitive, 128>,
};

@group(0) @binding(0) var<storage, read> leaves: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(3) var<storage, read> page_residuals: array<u32>;
@group(0) @binding(6) var<uniform> params: Params;
@group(1) @binding(0) var<uniform> view: ViewUniforms;
@group(1) @binding(1) var<uniform> render_from_body: mat4x4<f32>;

var<workgroup> mesh_output: MeshOutput;

fn residual(page: vec4<u32>, sample_index: u32) -> i32 {
    let word = page_residuals[page.w + sample_index / 2u];
    let raw = (word >> ((sample_index & 1u) * 16u)) & 0xffffu;
    return select(i32(raw), i32(raw) - 65536, raw >= 32768u);
}

fn page_sample(page: vec4<u32>, u: f32, v: f32) -> f32 {
    if (page.z == 0u) {
        return 0.0;
    }
    let max_coord = f32(page.z - 1u);
    let px = clamp(u, 0.0, 1.0) * max_coord;
    let py = clamp(v, 0.0, 1.0) * max_coord;
    let x = u32(floor(px));
    let y = u32(floor(py));
    let x1 = min(x + 1u, page.z - 1u);
    let y1 = min(y + 1u, page.z - 1u);
    let grid = page.z;
    let h00 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x)) * bitcast<f32>(page.y);
    let h10 = bitcast<f32>(page.x) + f32(residual(page, y * grid + x1)) * bitcast<f32>(page.y);
    let h01 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x)) * bitcast<f32>(page.y);
    let h11 = bitcast<f32>(page.x) + f32(residual(page, y1 * grid + x1)) * bitcast<f32>(page.y);
    let top = h00 + (h10 - h00) * (px - f32(x));
    let bottom = h01 + (h11 - h01) * (px - f32(x));
    return top + (bottom - top) * (py - f32(y));
}

fn payload_bit(low: u32, high: u32, bit: u32) -> u32 {
    if (bit < 32u) {
        return (low >> bit) & 1u;
    }
    return (high >> (bit - 32u)) & 1u;
}

fn tile_coordinates(record: vec4<u32>) -> vec4<u32> {
    let depth = record.z;
    if (depth < 3u || ((depth - 3u) & 1u) != 0u) {
        return vec4(0u);
    }
    var low = record.x;
    var high = record.y;
    if (depth < 32u) {
        low = low - (1u << depth);
    } else {
        high = high - (1u << (depth - 32u));
    }
    let path_bits = depth - 3u;
    var face = 0u;
    if (path_bits < 32u) {
        face = (low >> path_bits) | (high << (32u - path_bits));
    } else if (path_bits == 32u) {
        face = high;
    } else {
        face = high >> (path_bits - 32u);
    }
    let level = path_bits / 2u;
    var tile_x = 0u;
    var tile_y = 0u;
    for (var i = 0u; i < 17u; i = i + 1u) {
        if (i < level) {
            let shift = (level - i - 1u) * 2u;
            tile_x = tile_x * 2u + payload_bit(low, high, shift + 1u);
            tile_y = tile_y * 2u + payload_bit(low, high, shift);
        }
    }
    return vec4(tile_x, tile_y, level, face & 7u);
}

fn face_direction(face: u32, a: f32, b: f32) -> vec3<f32> {
    var raw = vec3(a, b, 1.0);
    if (face == 0u) { raw = vec3(1.0, b, -a); }
    if (face == 1u) { raw = vec3(-1.0, b, a); }
    if (face == 2u) { raw = vec3(a, 1.0, -b); }
    if (face == 3u) { raw = vec3(a, -1.0, b); }
    if (face == 4u) { raw = vec3(a, b, 1.0); }
    if (face == 5u) { raw = vec3(-a, b, -1.0); }
    return normalize(raw);
}

fn surface_position(tile: vec4<u32>, u: f32, v: f32, height: f32, radius: f32) -> vec3<f32> {
    let scale = exp2(f32(tile.z));
    let a = 2.0 * (f32(tile.x) + u) / scale - 1.0;
    let b = 2.0 * (f32(tile.y) + v) / scale - 1.0;
    return face_direction(tile.w, a, b) * (radius + height);
}

@mesh(mesh_output) @workgroup_size(64)
fn build_mesh(
    @builtin(local_invocation_index) invocation: u32,
    @builtin(workgroup_id) workgroup: vec3<u32>,
) {
    let leaf = workgroup.y;
    let meshlet = workgroup.x;
    let page = page_metadata[leaf];
    let valid = page.z != 0u;
    mesh_output.vertex_count = select(0u, 81u, valid);
    mesh_output.primitive_count = select(0u, 128u, valid);
    if (!valid) {
        return;
    }

    let tile = tile_coordinates(leaves[leaf]);
    let radius = bitcast<f32>(params.radius_bits);
    if (invocation < 81u) {
        let local_x = invocation % 9u;
        let local_y = invocation / 9u;
        let patch_x = (meshlet % 4u) * 8u + local_x;
        let patch_y = (meshlet / 4u) * 8u + local_y;
        let uv = vec2(f32(patch_x) / 32.0, f32(patch_y) / 32.0);
        let p = surface_position(tile, uv.x, uv.y, page_sample(page, uv.x, uv.y), radius);
        let du = 1.0 / 32.0;
        let puv = vec2(min(uv.x + du, 1.0), uv.y);
        let pvv = vec2(uv.x, min(uv.y + du, 1.0));
        let pu = surface_position(tile, puv.x, puv.y, page_sample(page, puv.x, puv.y), radius);
        let pv = surface_position(tile, pvv.x, pvv.y, page_sample(page, pvv.x, pvv.y), radius);
        let normal = normalize(cross(pu - p, pv - p));
        mesh_output.vertices[invocation].clip_position =
            view.clip_from_world * render_from_body * vec4(p, 1.0);
        mesh_output.vertices[invocation].normal =
            normalize((render_from_body * vec4(normal, 0.0)).xyz);
    }
    if (invocation < 128u) {
        let cell = invocation / 2u;
        let triangle = invocation & 1u;
        let cell_x = cell % 8u;
        let cell_y = cell / 8u;
        let base = cell_y * 9u + cell_x;
        let right = base + 1u;
        let down = base + 9u;
        let diagonal = down + 1u;
        mesh_output.primitives[invocation].indices = select(
            vec3(base, right, down),
            vec3(right, diagonal, down),
            triangle == 1u,
        );
    }
}

@fragment
fn fragment(input: MeshVertex) -> @location(0) vec4<f32> {
    let light = normalize(vec3(0.35, 0.8, 0.45));
    let diffuse = 0.24 + 0.76 * max(dot(normalize(input.normal), light), 0.0);
    return vec4(vec3(0.20, 0.34, 0.17) * diffuse, 1.0);
}
"#;

/// Installs extraction and GPU preparation for the universal CBT plugin.
pub(super) struct CbtRenderPlugin;

impl Plugin for CbtRenderPlugin {
    fn build(&self, app: &mut App) {
        let has_render_app = app.get_sub_app_mut(RenderApp).is_some();
        if has_render_app {
            app.add_plugins(ExtractResourcePlugin::<CbtRenderTopology>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderPages>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderSurface>::default());
        }
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app.init_gpu_resource::<CbtGpuBuffers>();
            render_app.add_systems(RenderStartup, init_cbt_gpu_pipeline);
            render_app.add_systems(
                Render,
                prepare_cbt_gpu_buffers.in_set(RenderSystems::PrepareResources),
            );
            render_app.add_systems(
                RenderGraph,
                dispatch_cbt_geometry
                    .before(bevy_core_pipeline::schedule::camera_driver)
                    .in_set(RenderGraphSystems::Render),
            );
            if render_app
                .get_schedule(bevy_core_pipeline::Core3d)
                .is_some()
            {
                render_app.add_systems(
                    bevy_core_pipeline::Core3d,
                    draw_cbt_geometry
                        .after(bevy_core_pipeline::core_3d::main_opaque_pass_3d)
                        .before(bevy_core_pipeline::core_3d::main_transparent_pass_3d),
                );
                #[cfg(feature = "mesh-shaders")]
                render_app.add_systems(
                    bevy_core_pipeline::Core3d,
                    draw_cbt_mesh_geometry
                        .after(bevy_core_pipeline::core_3d::main_opaque_pass_3d)
                        .before(bevy_core_pipeline::core_3d::main_transparent_pass_3d),
                );
            }
        }
    }
}

fn init_cbt_gpu_pipeline(mut commands: bevy::prelude::Commands, device: Res<RenderDevice>) {
    let bind_group_layout = device.create_bind_group_layout(
        "thessa-cbt-geometry-layout",
        &[
            storage_binding(0, true),
            storage_binding(1, false),
            storage_binding(2, true),
            storage_binding(3, true),
            storage_binding(4, false),
            storage_binding(5, false),
            BindGroupLayoutEntry {
                binding: 6,
                visibility: ShaderStages::COMPUTE,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    );
    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("thessa-cbt-geometry-pipeline-layout"),
        bind_group_layouts: &[Some(&bind_group_layout)],
        immediate_size: 0,
    });
    let shader = device.create_and_validate_shader_module(ShaderModuleDescriptor {
        label: Some("thessa-cbt-geometry-shader"),
        source: ShaderSource::Wgsl(Cow::Borrowed(CBT_GEOMETRY_WGSL)),
    });
    let pipeline = device.create_compute_pipeline(&RawComputePipelineDescriptor {
        label: Some("thessa-cbt-geometry-pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader,
        entry_point: Some("build_geometry"),
        compilation_options: Default::default(),
        cache: None,
    });
    let raster_bind_group_layout = device.create_bind_group_layout(
        "thessa-cbt-raster-layout",
        &[
            BindGroupLayoutEntry {
                binding: 0,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 1,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 2,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 3,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 4,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    );
    let raster_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("thessa-cbt-raster-pipeline-layout"),
        bind_group_layouts: &[Some(&raster_bind_group_layout)],
        immediate_size: 0,
    });
    let raster_shader = device.create_and_validate_shader_module(ShaderModuleDescriptor {
        label: Some("thessa-cbt-raster-shader"),
        source: ShaderSource::Wgsl(Cow::Borrowed(CBT_RASTER_WGSL)),
    });

    #[cfg(feature = "mesh-shaders")]
    if device
        .features()
        .contains(WgpuFeatures::EXPERIMENTAL_MESH_SHADER)
    {
        let mesh_bind_group_layout = device.create_bind_group_layout(
            "thessa-cbt-mesh-layout",
            &[
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 2,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 3,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 6,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        );
        let mesh_view_bind_group_layout = device.create_bind_group_layout(
            "thessa-cbt-mesh-view-layout",
            &[
                BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: None,
                    },
                    count: None,
                },
                BindGroupLayoutEntry {
                    binding: 1,
                    visibility: ShaderStages::MESH,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        );
        let mesh_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("thessa-cbt-mesh-pipeline-layout"),
            bind_group_layouts: &[
                Some(&mesh_bind_group_layout),
                Some(&mesh_view_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let mesh_shader = device.create_and_validate_shader_module(ShaderModuleDescriptor {
            label: Some("thessa-cbt-mesh-shader"),
            source: ShaderSource::Wgsl(Cow::Borrowed(CBT_MESH_WGSL)),
        });
        commands.insert_resource(CbtGpuMeshPipeline {
            bind_group_layout: mesh_bind_group_layout,
            view_bind_group_layout: mesh_view_bind_group_layout,
            pipeline_layout: mesh_pipeline_layout,
            shader: mesh_shader,
            pipelines: HashMap::default(),
        });
    }

    commands.insert_resource(CbtGpuPipeline {
        bind_group_layout,
        pipeline,
    });
    commands.insert_resource(CbtGpuRasterPipeline {
        bind_group_layout: raster_bind_group_layout,
        pipeline_layout: raster_pipeline_layout,
        shader: raster_shader,
        pipelines: HashMap::default(),
    });
}

fn storage_binding(binding: u32, read_only: bool) -> BindGroupLayoutEntry {
    BindGroupLayoutEntry {
        binding,
        visibility: ShaderStages::COMPUTE,
        ty: BindingType::Buffer {
            ty: BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn pack_page_residuals(page: &HeightPage, output: &mut Vec<u32>) -> u32 {
    let offset = output.len() as u32;
    for pair in page.residuals().chunks(2) {
        let low = pair[0] as u16 as u32;
        let high = pair.get(1).copied().unwrap_or_default() as u16 as u32;
        output.push(low | (high << 16));
    }
    offset
}

fn prepare_cbt_gpu_buffers(
    topology: Option<Res<CbtRenderTopology>>,
    pages: Option<Res<CbtRenderPages>>,
    surface: Option<Res<CbtRenderSurface>>,
    mut gpu: ResMut<CbtGpuBuffers>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let (Some(topology), Some(pages), Some(surface)) = (topology, pages, surface) else {
        return;
    };
    if !topology.is_changed()
        && !pages.is_changed()
        && gpu.topology_generation == topology.generation()
        && gpu.pages_generation == pages.generation()
        && gpu.surface_generation == surface.generation()
        && gpu.transform_generation == surface.transform_generation()
    {
        return;
    }

    if surface.is_changed() || gpu.transform_generation != surface.transform_generation() {
        gpu.surface_transform.clear();
        gpu.surface_transform.push(surface.render_from_body());
        gpu.surface_transform.write_buffer(&device, &queue);
        gpu.transform_generation = surface.transform_generation();
    }

    if !topology.is_changed()
        && !pages.is_changed()
        && gpu.topology_generation == topology.generation()
        && gpu.pages_generation == pages.generation()
        && gpu.surface_generation == surface.generation()
    {
        return;
    }

    let records = topology.records();
    let mut metadata = Vec::with_capacity(records.len());
    let mut residuals = Vec::new();
    for record in records {
        let depth = record[2];
        if depth < 3 || !(depth - 3).is_multiple_of(2) {
            metadata.push([0; 4]);
            continue;
        }
        let node_id = u64::from(record[0]) | (u64::from(record[1]) << 32);
        let Some(page) = pages.get(node_id) else {
            metadata.push([0; 4]);
            continue;
        };
        let word_offset = pack_page_residuals(page, &mut residuals);
        metadata.push([
            page.base_height_m().to_bits(),
            page.residual_scale_m().to_bits(),
            page.grid_size(),
            word_offset,
        ]);
    }
    // Keep the binding valid even while topology exists but no tile page has
    // finished streaming. The shader branches on `grid_size == 0` and emits
    // zero-count draw commands for those leaves.
    if residuals.is_empty() {
        residuals.push(0);
    }

    gpu.leaf_records.clear();
    gpu.leaf_records.extend(records.iter().copied());
    gpu.leaf_records.write_buffer(&device, &queue);

    gpu.patch_records.clear();
    gpu.patch_records
        .extend(std::iter::repeat_n([0; 4], records.len()));
    gpu.patch_records.write_buffer(&device, &queue);

    gpu.page_metadata.clear();
    gpu.page_metadata.extend(metadata);
    gpu.page_metadata.write_buffer(&device, &queue);

    gpu.page_residuals.clear();
    gpu.page_residuals.extend(residuals);
    gpu.page_residuals.write_buffer(&device, &queue);

    if !surface.gpu_mesh_enabled() {
        gpu.vertices.clear();
        gpu.vertices.extend(std::iter::repeat_n(
            [0.0; 8],
            records.len() * GPU_VERTEX_COUNT_PER_PATCH,
        ));
        gpu.vertices.write_buffer(&device, &queue);

        gpu.draw_list.clear();
        gpu.draw_list.push([0; 4]);
        gpu.draw_list.write_buffer(&device, &queue);
    }

    gpu.params.clear();
    gpu.params.push([
        records.len() as u32,
        GPU_VERTEX_COUNT_PER_PATCH as u32,
        surface.radius_m().to_bits(),
        0,
    ]);
    gpu.params.write_buffer(&device, &queue);

    gpu.topology_generation = topology.generation();
    gpu.pages_generation = pages.generation();
    gpu.surface_generation = surface.generation();
    gpu.generated_topology_generation = u64::MAX;
    gpu.generated_pages_generation = u64::MAX;
    gpu.generated_surface_generation = u64::MAX;
    gpu.leaf_count = records.len() as u32;
}

fn dispatch_cbt_geometry(
    surface: Option<Res<CbtRenderSurface>>,
    pipeline: Option<Res<CbtGpuPipeline>>,
    mut gpu: ResMut<CbtGpuBuffers>,
    mut context: RenderContext,
) {
    let (Some(surface), Some(pipeline)) = (surface, pipeline) else {
        return;
    };
    if surface.gpu_mesh_enabled() {
        return;
    }
    if gpu.leaf_count == 0
        || (gpu.generated_topology_generation == gpu.topology_generation
            && gpu.generated_pages_generation == gpu.pages_generation
            && gpu.generated_surface_generation == gpu.surface_generation)
    {
        return;
    }
    let (
        Some(leaf_buffer),
        Some(patch_buffer),
        Some(metadata_buffer),
        Some(residual_buffer),
        Some(vertex_buffer),
        Some(draw_buffer),
        Some(params_buffer),
    ) = (
        gpu.leaf_records.buffer(),
        gpu.patch_records.buffer(),
        gpu.page_metadata.buffer(),
        gpu.page_residuals.buffer(),
        gpu.vertices.buffer(),
        gpu.draw_list.buffer(),
        gpu.params.buffer(),
    )
    else {
        return;
    };
    let bind_group = context.render_device().create_bind_group(
        "thessa-cbt-geometry-bind-group",
        &pipeline.bind_group_layout,
        &[
            BindGroupEntry {
                binding: 0,
                resource: BindingResource::Buffer(leaf_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 1,
                resource: BindingResource::Buffer(patch_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 2,
                resource: BindingResource::Buffer(metadata_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 3,
                resource: BindingResource::Buffer(residual_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 4,
                resource: BindingResource::Buffer(vertex_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 5,
                resource: BindingResource::Buffer(draw_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 6,
                resource: BindingResource::Buffer(params_buffer.as_entire_buffer_binding()),
            },
        ],
    );
    {
        let mut pass = context
            .command_encoder()
            .begin_compute_pass(&ComputePassDescriptor {
                label: Some("thessa-cbt-height-page-geometry"),
                timestamp_writes: None,
            });
        pass.set_pipeline(&pipeline.pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let total = gpu
            .leaf_count
            .saturating_mul(GPU_VERTEX_COUNT_PER_PATCH as u32);
        pass.dispatch_workgroups(total.div_ceil(64), 1, 1);
    }
    gpu.generated_topology_generation = gpu.topology_generation;
    gpu.generated_pages_generation = gpu.pages_generation;
    gpu.generated_surface_generation = gpu.surface_generation;
}

fn draw_cbt_geometry(
    surface: Option<Res<CbtRenderSurface>>,
    gpu: Option<Res<CbtGpuBuffers>>,
    raster: Option<ResMut<CbtGpuRasterPipeline>>,
    view: ViewQuery<(
        &ExtractedCamera,
        &ExtractedView,
        &ViewTarget,
        &ViewDepthTexture,
        &ViewUniformOffset,
    )>,
    render_adapter: Res<RenderAdapter>,
    view_uniforms: Res<ViewUniforms>,
    mut context: RenderContext,
) {
    let (Some(surface), Some(gpu), Some(mut raster)) = (surface, gpu, raster) else {
        return;
    };
    if !surface.gpu_raster_enabled() || surface.gpu_mesh_enabled() || gpu.leaf_count() == 0 {
        return;
    }
    let (camera, extracted_view, target, depth, view_uniform_offset) = view.into_inner();
    let (
        Some(vertex_buffer),
        Some(metadata_buffer),
        Some(draw_buffer),
        Some(params_buffer),
        Some(surface_buffer),
    ) = (
        gpu.vertex_buffer(),
        gpu.page_metadata_buffer(),
        gpu.draw_list_buffer(),
        gpu.params_buffer(),
        gpu.surface_transform_buffer(),
    )
    else {
        return;
    };
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };
    let format = extracted_view.target_format;
    if !raster.pipelines.contains_key(&format) {
        let color_targets = [Some(ColorTargetState {
            format,
            blend: None,
            write_mask: ColorWrites::ALL,
        })];
        let pipeline =
            context
                .render_device()
                .create_render_pipeline(&RawRenderPipelineDescriptor {
                    label: Some("thessa-cbt-raster-pipeline"),
                    layout: Some(&raster.pipeline_layout),
                    vertex: RawVertexState {
                        module: &raster.shader,
                        entry_point: Some("vertex"),
                        compilation_options: Default::default(),
                        buffers: &[],
                    },
                    fragment: Some(RawFragmentState {
                        module: &raster.shader,
                        entry_point: Some("fragment"),
                        compilation_options: Default::default(),
                        targets: &color_targets,
                    }),
                    primitive: PrimitiveState {
                        topology: PrimitiveTopology::TriangleList,
                        strip_index_format: None,
                        front_face: bevy::render::render_resource::FrontFace::Ccw,
                        cull_mode: None,
                        unclipped_depth: false,
                        polygon_mode: bevy::render::render_resource::PolygonMode::Fill,
                        conservative: false,
                    },
                    depth_stencil: Some(DepthStencilState {
                        format: bevy_core_pipeline::core_3d::CORE_3D_DEPTH_FORMAT,
                        depth_write_enabled: Some(true),
                        depth_compare: Some(CompareFunction::GreaterEqual),
                        stencil: Default::default(),
                        bias: DepthBiasState::default(),
                    }),
                    multisample: MultisampleState::default(),
                    multiview_mask: None,
                    cache: None,
                });
        raster.pipelines.insert(format, pipeline);
    }
    let pipeline = raster
        .pipelines
        .get(&format)
        .expect("CBT raster pipeline inserted above");
    let bind_group = context.render_device().create_bind_group(
        "thessa-cbt-raster-bind-group",
        &raster.bind_group_layout,
        &[
            BindGroupEntry {
                binding: 0,
                resource: view_binding,
            },
            BindGroupEntry {
                binding: 1,
                resource: BindingResource::Buffer(surface_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 2,
                resource: BindingResource::Buffer(vertex_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 3,
                resource: BindingResource::Buffer(metadata_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 4,
                resource: BindingResource::Buffer(params_buffer.as_entire_buffer_binding()),
            },
        ],
    );
    let supports_indirect = render_adapter
        .get_downlevel_capabilities()
        .flags
        .contains(DownlevelFlags::INDIRECT_EXECUTION);
    if !supports_indirect {
        return;
    }
    let color_attachments = [Some(target.get_color_attachment())];
    let mut pass = context.begin_tracked_render_pass(RenderPassDescriptor {
        label: Some("thessa-cbt-raster-pass"),
        color_attachments: &color_attachments,
        depth_stencil_attachment: Some(depth.get_attachment(StoreOp::Store)),
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });
    if let Some(viewport) = camera.viewport.as_ref() {
        pass.set_camera_viewport(viewport);
    }
    pass.set_render_pipeline(pipeline);
    pass.set_bind_group(0, &bind_group, &[view_uniform_offset.offset]);
    // One procedural indirect draw, with the leaf ordinal carried by
    // `instance_index`; the vertex shader reconstructs the shared grid index.
    pass.draw_indirect(draw_buffer, 0);
}

#[cfg(feature = "mesh-shaders")]
fn draw_cbt_mesh_geometry(
    surface: Option<Res<CbtRenderSurface>>,
    gpu: Option<Res<CbtGpuBuffers>>,
    mesh: Option<ResMut<CbtGpuMeshPipeline>>,
    view: ViewQuery<(
        &ExtractedCamera,
        &ExtractedView,
        &ViewTarget,
        &ViewDepthTexture,
        &ViewUniformOffset,
    )>,
    view_uniforms: Res<ViewUniforms>,
    mut context: RenderContext,
) {
    let (Some(surface), Some(gpu), Some(mut mesh)) = (surface, gpu, mesh) else {
        return;
    };
    if !surface.gpu_mesh_enabled() || gpu.leaf_count() == 0 {
        return;
    }

    let device = context.render_device();
    if !device
        .features()
        .contains(WgpuFeatures::EXPERIMENTAL_MESH_SHADER)
    {
        return;
    }
    let limits = device.limits();
    if MESHLET_VERTEX_COUNT > limits.max_mesh_output_vertices
        || MESHLET_PRIMITIVE_COUNT > limits.max_mesh_output_primitives
        || 64 > limits.max_mesh_invocations_per_workgroup
        || 64 > limits.max_mesh_invocations_per_dimension
        || MESHLETS_PER_PATCH > limits.max_task_mesh_workgroups_per_dimension
        || gpu.leaf_count() > limits.max_task_mesh_workgroups_per_dimension
        || u64::from(gpu.leaf_count()) * u64::from(MESHLETS_PER_PATCH)
            > u64::from(limits.max_task_mesh_workgroup_total_count)
    {
        return;
    }

    let (camera, extracted_view, target, depth, view_uniform_offset) = view.into_inner();
    let (
        Some(leaf_buffer),
        Some(metadata_buffer),
        Some(residual_buffer),
        Some(params_buffer),
        Some(surface_buffer),
    ) = (
        gpu.leaf_buffer(),
        gpu.page_metadata_buffer(),
        gpu.page_residual_buffer(),
        gpu.params_buffer(),
        gpu.surface_transform_buffer(),
    )
    else {
        return;
    };
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };

    let format = extracted_view.target_format;
    if !mesh.pipelines.contains_key(&format) {
        let color_targets = [Some(ColorTargetState {
            format,
            blend: None,
            write_mask: ColorWrites::ALL,
        })];
        let pipeline = device
            .wgpu_device()
            .create_mesh_pipeline(&wgpu::MeshPipelineDescriptor {
                label: Some("thessa-cbt-mesh-pipeline"),
                layout: Some(&mesh.pipeline_layout),
                task: None,
                mesh: wgpu::MeshState {
                    module: &mesh.shader,
                    entry_point: Some("build_mesh"),
                    compilation_options: Default::default(),
                },
                primitive: PrimitiveState {
                    topology: PrimitiveTopology::TriangleList,
                    strip_index_format: None,
                    front_face: bevy::render::render_resource::FrontFace::Ccw,
                    cull_mode: None,
                    unclipped_depth: false,
                    polygon_mode: bevy::render::render_resource::PolygonMode::Fill,
                    conservative: false,
                },
                depth_stencil: Some(DepthStencilState {
                    format: bevy_core_pipeline::core_3d::CORE_3D_DEPTH_FORMAT,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: DepthBiasState::default(),
                }),
                multisample: MultisampleState::default(),
                fragment: Some(RawFragmentState {
                    module: &mesh.shader,
                    entry_point: Some("fragment"),
                    compilation_options: Default::default(),
                    targets: &color_targets,
                }),
                multiview: None,
                cache: None,
            });
        mesh.pipelines
            .insert(format, RenderPipeline::from(pipeline));
    }
    let pipeline = mesh
        .pipelines
        .get(&format)
        .expect("CBT mesh pipeline inserted above");
    let bind_group = device.create_bind_group(
        "thessa-cbt-mesh-bind-group",
        &mesh.bind_group_layout,
        &[
            BindGroupEntry {
                binding: 0,
                resource: BindingResource::Buffer(leaf_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 2,
                resource: BindingResource::Buffer(metadata_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 3,
                resource: BindingResource::Buffer(residual_buffer.as_entire_buffer_binding()),
            },
            BindGroupEntry {
                binding: 6,
                resource: BindingResource::Buffer(params_buffer.as_entire_buffer_binding()),
            },
        ],
    );
    let view_bind_group = device.create_bind_group(
        "thessa-cbt-mesh-view-bind-group",
        &mesh.view_bind_group_layout,
        &[
            BindGroupEntry {
                binding: 0,
                resource: view_binding,
            },
            BindGroupEntry {
                binding: 1,
                resource: BindingResource::Buffer(surface_buffer.as_entire_buffer_binding()),
            },
        ],
    );

    let color_attachments = [Some(target.get_color_attachment())];
    let mut pass = context
        .command_encoder()
        .begin_render_pass(&RenderPassDescriptor {
            label: Some("thessa-cbt-mesh-pass"),
            color_attachments: &color_attachments,
            depth_stencil_attachment: Some(depth.get_attachment(StoreOp::Store)),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
    if let Some(viewport) = camera.viewport.as_ref() {
        pass.set_viewport(
            viewport.physical_position.x as f32,
            viewport.physical_position.y as f32,
            viewport.physical_size.x as f32,
            viewport.physical_size.y as f32,
            viewport.depth.start,
            viewport.depth.end,
        );
    }
    pass.set_pipeline(pipeline);
    pass.set_bind_group(0, &*bind_group, &[]);
    pass.set_bind_group(1, &*view_bind_group, &[view_uniform_offset.offset]);
    pass.draw_mesh_tasks(MESHLETS_PER_PATCH, gpu.leaf_count(), 1);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_shader_is_portable_wgsl() {
        let module = naga::front::wgsl::parse_str(CBT_GEOMETRY_WGSL)
            .expect("CBT height-page geometry shader must parse");
        naga::valid::Validator::new(Default::default(), Default::default())
            .validate(&module)
            .expect("CBT height-page geometry shader must validate");
    }

    #[test]
    fn raster_shader_is_portable_wgsl() {
        let module =
            naga::front::wgsl::parse_str(CBT_RASTER_WGSL).expect("CBT raster shader must parse");
        naga::valid::Validator::new(Default::default(), Default::default())
            .validate(&module)
            .expect("CBT raster shader must validate");
    }

    #[cfg(feature = "mesh-shaders")]
    #[test]
    fn mesh_shader_is_validated_with_explicit_native_capability() {
        let module = naga::front::wgsl::parse_str(CBT_MESH_WGSL)
            .expect("CBT mesh shader must parse with wgpu_mesh_shader enabled");
        naga::valid::Validator::new(Default::default(), naga::valid::Capabilities::MESH_SHADER)
            .validate(&module)
            .expect("CBT mesh shader must validate with mesh capability");
    }

    #[test]
    fn residual_words_round_trip_signed_samples() {
        let page = HeightPage::bake(&[-3.0, 0.0, 2.0, 7.0], 2, 1.0).unwrap();
        let mut words = Vec::new();
        let offset = pack_page_residuals(&page, &mut words);
        assert_eq!(offset, 0);
        assert_eq!(words.len(), 2);
        assert_eq!(words[0] as u16, page.residuals()[0] as u16);
        assert_eq!((words[1] >> 16) as u16, page.residuals()[3] as u16);
    }
}
