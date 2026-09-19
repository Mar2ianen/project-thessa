//! Bevy render-world bridge for exact CBT leaves and GPU terrain geometry.
//!
//! The main world owns topology decisions and baked [`HeightPage`] payloads.
//! Extraction copies those immutable snapshots into the render world. A single
//! generation-gated compute pass then expands published pages into vertex
//! grids. The classifier compacts selected triangles for one indirect draw.
//! Adapters can retain the last complete presentation while replacement pages
//! stream and the planning tree changes.

use std::{borrow::Cow, collections::HashMap};

use bevy::{
    app::{App, Plugin},
    ecs::schedule::IntoScheduleConfigs,
    prelude::{FromWorld, Res, ResMut, Resource, World},
    render::{
        GpuResourceAppExt, Render, RenderApp, RenderStartup, RenderSystems,
        camera::ExtractedCamera,
        diagnostic::RecordDiagnostics,
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        render_asset::RenderAssets,
        render_resource::{
            BindGroup, BindGroupEntry, BindGroupLayout, BindGroupLayoutEntry, BindingResource,
            BindingType, Buffer, BufferBindingType, BufferId, BufferUsages, ColorTargetState,
            ColorWrites, CompareFunction, ComputePassDescriptor, ComputePipeline, DepthBiasState,
            DepthStencilState, DownlevelFlags, MultisampleState, PipelineLayout,
            PipelineLayoutDescriptor, PrimitiveState, PrimitiveTopology, RawBufferVec,
            RawComputePipelineDescriptor, RawFragmentState, RawRenderPipelineDescriptor,
            RawVertexState, RenderPassDescriptor, RenderPipeline, SamplerBindingType, SamplerId,
            ShaderModule, ShaderModuleDescriptor, ShaderSource, ShaderStages, StoreOp,
            TextureFormat, TextureSampleType, TextureViewDimension, TextureViewId,
        },
        renderer::{
            RenderAdapter, RenderContext, RenderDevice, RenderGraph, RenderGraphSystems,
            RenderQueue, ViewQuery,
        },
        texture::GpuImage,
        view::{ExtractedView, ViewDepthTexture, ViewTarget, ViewUniformOffset, ViewUniforms},
    },
};

use crate::{
    CbtGpuPresentation, CbtRenderMaterialPages,
    material_cache::{SlotCache, SlotChange},
};

impl ExtractResource for CbtGpuPresentation {
    type Source = Self;
    fn extract_resource(source: &Self) -> Self {
        source.clone()
    }
}
use thessa_rcbt_core::HeightPage;
#[path = "material_render.rs"]
mod material_render;

impl ExtractResource for CbtRenderMaterialPages {
    type Source = Self;
    fn extract_resource(source: &Self) -> Self {
        source.clone()
    }
}

#[cfg(feature = "mesh-shaders")]
use bevy::render::render_resource::WgpuFeatures;

use super::{
    CbtLeafRecord, CbtRenderMaterial, CbtRenderPages, CbtRenderSurface, CbtRenderTopology,
    MaterialStorageSetting,
};

impl ExtractResource for MaterialStorageSetting {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        *source
    }
}

impl ExtractResource for CbtRenderMaterial {
    type Source = Self;

    fn extract_resource(source: &Self::Source) -> Self {
        source.clone()
    }
}

const GPU_GRID_SIZE: usize = 33;
const GPU_VERTEX_COUNT_PER_PATCH: usize = GPU_GRID_SIZE * GPU_GRID_SIZE;
const GPU_SURFACE_TRIANGLE_COUNT_PER_PATCH: usize = (GPU_GRID_SIZE - 1) * (GPU_GRID_SIZE - 1) * 2;
const GPU_SKIRT_TRIANGLE_COUNT_PER_PATCH: usize = (GPU_GRID_SIZE - 1) * 4 * 2;
const GPU_TRIANGLE_COUNT_PER_PATCH: usize =
    GPU_SURFACE_TRIANGLE_COUNT_PER_PATCH + GPU_SKIRT_TRIANGLE_COUNT_PER_PATCH;
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
/// normal. The compute stage compacts selected triangle records into
/// `active_triangles`, then writes the vertex count of one indirect draw
/// from the GPU triangle counter. The raster consumer reads the compact triangle
/// records directly; no per-leaf draw/entity is submitted.
/// Identity key for the cached geometry-compute bind group.
///
/// View values change via dynamic offsets / uniform contents without
/// changing the binding itself, so the bind group is rebuilt only when a
/// buffer object is reallocated (new [`BufferId`] after `reserve`) or the
/// pipeline layout changes. This removes per-dispatch driver churn while
/// staying correct across residency changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct GeometryBindKey {
    leaf: BufferId,
    patch: BufferId,
    metadata: BufferId,
    residual: BufferId,
    vertex: BufferId,
    params: BufferId,
    frames: BufferId,
}

/// Identity key for the cached classifier-compute bind group. The view
/// matrix itself flows through the existing dynamic offset, so camera motion
/// alone must not invalidate the group — only buffer reallocations do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ClassifierBindKey {
    view_uniforms: BufferId,
    metadata: BufferId,
    vertex: BufferId,
    active_triangles: BufferId,
    active_count: BufferId,
    draw: BufferId,
    surface: BufferId,
    params: BufferId,
    leaf: BufferId,
    frames: BufferId,
    grid_history: BufferId,
}

/// Identity key for the cached raster bind group. Camera motion uses the
/// dynamic view offset; texture/buffer residency changes (new view/sampler
/// or reallocated storage) invalidate the group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RasterBindKey {
    view_uniforms: BufferId,
    surface: BufferId,
    vertex: BufferId,
    metadata: BufferId,
    params: BufferId,
    active_triangles: BufferId,
    leaf: BufferId,
    lighting: BufferId,
    frames: BufferId,
    material_slots: BufferId,
    albedo_view: TextureViewId,
    albedo_sampler: SamplerId,
    roughness_view: TextureViewId,
    roughness_sampler: SamplerId,
    material_view: TextureViewId,
    material_sampler: SamplerId,
}

fn view_uniform_buffer_id(view_uniforms: &ViewUniforms) -> Option<BufferId> {
    view_uniforms.uniforms.buffer().map(|buffer| buffer.id())
}

#[derive(Resource)]
pub struct CbtGpuBuffers {
    leaf_records: RawBufferVec<CbtLeafRecord>,
    patch_records: RawBufferVec<CbtLeafRecord>,
    page_metadata: RawBufferVec<[u32; 4]>,
    page_residuals: RawBufferVec<u32>,
    vertices: RawBufferVec<[f32; 8]>,
    draw_list: RawBufferVec<[u32; 4]>,
    active_triangles: RawBufferVec<[u32; 4]>,
    active_count: RawBufferVec<[u32; 4]>,
    grid_history: RawBufferVec<[u32; 4]>,
    params: RawBufferVec<[u32; 4]>,
    surface_transform: RawBufferVec<[f32; 16]>,
    lighting: RawBufferVec<[f32; 4]>,
    material_array: Option<material_render::MaterialArray>,
    tile_frames: RawBufferVec<[[f32; 4]; 7]>,
    tile_anchors: Vec<Option<crate::precision::TileAnchor>>,
    topology_generation: u64,
    pages_generation: u64,
    surface_generation: u64,
    transform_generation: u64,
    generated_topology_generation: u64,
    generated_pages_generation: u64,
    generated_surface_generation: u64,
    leaf_count: u32,
    complete_pages: bool,
    geometry_bind_group: Option<BindGroup>,
    geometry_bind_key: Option<GeometryBindKey>,
    /// Stable `node_id -> slot` assignment for height-page residuals.
    /// Topology reorder keeps slots (only the small per-ordinal metadata is
    /// rewritten); a page arrival re-uploads exactly one slot range.
    height_slots: SlotCache,
    /// Packed words reserved per height slot. `metadata.w` stays a word
    /// offset (`slot * stride`), so the shader contract is unchanged.
    height_stride_words: usize,
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
        let mut draw_list = RawBufferVec::new(
            BufferUsages::STORAGE | BufferUsages::INDIRECT | BufferUsages::COPY_SRC,
        );
        draw_list.set_label(Some("thessa-cbt-indirect-draw-list"));
        let mut active_triangles = RawBufferVec::new(BufferUsages::STORAGE);
        active_triangles.set_label(Some("thessa-cbt-active-triangles"));
        let mut active_count = RawBufferVec::new(BufferUsages::STORAGE | BufferUsages::COPY_SRC);
        active_count.set_label(Some("thessa-cbt-active-leaf-count"));
        let mut grid_history = RawBufferVec::new(BufferUsages::STORAGE);
        grid_history.set_label(Some("thessa-cbt-grid-step-history"));
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
            active_triangles,
            active_count,
            grid_history,
            params,
            surface_transform,
            lighting: RawBufferVec::new(BufferUsages::UNIFORM),
            material_array: None,
            tile_frames: RawBufferVec::new(BufferUsages::STORAGE),
            tile_anchors: Vec::new(),
            topology_generation: u64::MAX,
            pages_generation: u64::MAX,
            surface_generation: u64::MAX,
            transform_generation: u64::MAX,
            generated_topology_generation: u64::MAX,
            generated_pages_generation: u64::MAX,
            generated_surface_generation: u64::MAX,
            leaf_count: 0,
            complete_pages: false,
            geometry_bind_group: None,
            geometry_bind_key: None,
            height_slots: SlotCache::new(HEIGHT_SLOT_INITIAL_CAPACITY)
                .expect("height slot capacity is non-zero"),
            height_stride_words: HEIGHT_SLOT_MIN_STRIDE_WORDS,
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

    /// One GPU-written `DrawIndirect` command: three vertices per selected
    /// triangle and one instance, matching the reference's procedural stream.
    pub fn draw_list_buffer(&self) -> Option<&Buffer> {
        self.draw_list.buffer()
    }

    /// GPU-compacted `[leaf_ordinal, a, b, c]` grid triangle records.
    pub fn active_triangles_buffer(&self) -> Option<&Buffer> {
        self.active_triangles.buffer()
    }

    /// Uniform-compatible `[leaf_count, vertices_per_patch, radius_bits, 0]`.
    pub fn params_buffer(&self) -> Option<&Buffer> {
        self.params.buffer()
    }

    /// Persistent per-leaf `[node_id_lo, node_id_hi, grid_step, 0]` history.
    pub fn grid_history_buffer(&self) -> Option<&Buffer> {
        self.grid_history.buffer()
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

    /// Compact material storage telemetry: `(wire_bytes, decoded_bytes,
    /// encode_secs, pages_encoded)`. `None` before the material array is
    /// created; wire is a residency gauge, the rest are lifetimes.
    pub fn material_microstore_stats(&self) -> Option<(u64, u64, f64, u64)> {
        self.material_array.as_ref().map(|array| {
            (
                array.microstore_wire_bytes(),
                array.microstore_decoded_bytes(),
                array.microstore_encode_secs(),
                array.microstore_pages_encoded(),
            )
        })
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
struct CbtGpuClassifier {
    bind_group_layout: BindGroupLayout,
    reset_pipeline: ComputePipeline,
    classify_pipeline: ComputePipeline,
    finalize_pipeline: ComputePipeline,
    classified_view_generation: u64,
    classified_clip_from_world: Option<[f32; 16]>,
    classified_transform_generation: u64,
    classified_topology_generation: u64,
    classified_pages_generation: u64,
    classified_surface_generation: u64,
    cached_bind_group: Option<BindGroup>,
    cached_bind_key: Option<ClassifierBindKey>,
}

#[derive(Resource)]
struct CbtGpuRasterPipeline {
    bind_group_layout: BindGroupLayout,
    pipeline_layout: PipelineLayout,
    shader: ShaderModule,
    pipelines: HashMap<TextureFormat, RenderPipeline>,
    cached_bind_group: Option<BindGroup>,
    cached_bind_key: Option<RasterBindKey>,
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
const CBT_GEOMETRY_WGSL: &str = concat!(
    include_str!("precision.wgsl"),
    include_str!("tile_frame.wgsl"),
    include_str!("surface_sample.wgsl"),
    r#"
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
@group(0) @binding(5) var<uniform> params: Params;
@group(0) @binding(6) var<storage, read> tile_frames: array<CbtTileFrame>;

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
    if (page.z == 0u) {
        vertices[index * 2u] = vec4(0.0);
        vertices[index * 2u + 1u] = vec4(0.0);
        return;
    }
    let gx = local % 33u;
    let gy = local / 33u;
    let uv = vec2(f32(gx) / 32.0, f32(gy) / 32.0);
    let sample = cbt_surface_sample(tile_frames[ordinal].geometry, page, uv);
    vertices[index * 2u] = vec4(sample.local_position, 1.0);
    vertices[index * 2u + 1u] = vec4(sample.normal, sample.height);
}
"#
);

/// Portable GPU leaf classifier and active-triangle builder. It uses five
/// generated patch samples to reject leaves outside the current clip volume,
/// selects a screen-space grid step, and compacts the resulting triangle
/// records into the same indirect draw stream used by the raster consumer.
/// The topology remains authoritative on the CPU; visibility, triangle LOD,
/// and draw count are GPU-owned and rerun with the current camera.
const CBT_CLASSIFY_WGSL: &str = concat!(
    include_str!("precision.wgsl"),
    include_str!("tile_frame.wgsl"),
    r#"
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

struct ViewUniforms {
    clip_from_world: mat4x4<f32>,
};

@group(0) @binding(0) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(1) var<storage, read> generated_vertices: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> active_triangles: array<vec4<u32>>;
@group(0) @binding(3) var<storage, read_write> active_count: array<atomic<u32>>;
@group(0) @binding(4) var<storage, read_write> draw_list: array<DrawCommand>;
@group(0) @binding(5) var<uniform> view: ViewUniforms;
@group(0) @binding(6) var<uniform> render_from_body: mat4x4<f32>;
@group(0) @binding(7) var<uniform> params: Params;
@group(0) @binding(8) var<storage, read> leaves: array<vec4<u32>>;
@group(0) @binding(9) var<storage, read> tile_frames: array<CbtTileFrame>;
@group(0) @binding(10) var<storage, read_write> grid_history: array<vec4<u32>>;

fn corner_index(corner: u32) -> u32 {
    if (corner == 0u) { return 0u; }
    if (corner == 1u) { return 32u; }
    if (corner == 2u) { return 1056u; }
    if (corner == 3u) { return 1088u; }
    return 544u;
}

fn clip_position(ordinal: u32, corner: u32) -> vec4<f32> {
    let position = generated_vertices[
        (ordinal * params.vertices_per_patch + corner_index(corner)) * 2u
    ];
    return view.clip_from_world * cbt_render_position(tile_frames[ordinal], position.xyz, render_from_body);
}

fn intersects_clip(ordinal: u32) -> bool {
    // Reject only when every sample is outside the same homogeneous clip
    // plane. Testing "any sample inside" is cheaper to write but can punch a
    // hole through a large patch that straddles the frustum between samples.
    var all_left = true;
    var all_right = true;
    var all_bottom = true;
    var all_top = true;
    var all_near = true;
    var all_far = true;
    var all_behind = true;
    for (var corner = 0u; corner < 5u; corner = corner + 1u) {
        let position = clip_position(ordinal, corner);
        all_left = all_left && position.x < -position.w;
        all_right = all_right && position.x > position.w;
        all_bottom = all_bottom && position.y < -position.w;
        all_top = all_top && position.y > position.w;
        all_near = all_near && position.z < 0.0;
        all_far = all_far && position.z > position.w;
        all_behind = all_behind && position.w <= 0.0;
    }
    return !(all_left || all_right || all_bottom || all_top || all_near || all_far || all_behind);
}

fn ndc(position: vec4<f32>) -> vec2<f32> {
    return position.xy / max(position.w, 0.000001);
}

fn screen_grid_span(ordinal: u32) -> f32 {
    let top_left = clip_position(ordinal, 0u);
    let top_right = clip_position(ordinal, 1u);
    let bottom_left = clip_position(ordinal, 2u);
    let bottom_right = clip_position(ordinal, 3u);
    if (top_left.w <= 0.0 || top_right.w <= 0.0 || bottom_left.w <= 0.0 || bottom_right.w <= 0.0) {
        return 0.1;
    }
    var span = distance(ndc(top_left), ndc(top_right));
    span = max(span, distance(ndc(top_left), ndc(bottom_left)));
    span = max(span, distance(ndc(top_right), ndc(bottom_right)));
    span = max(span, distance(ndc(bottom_left), ndc(bottom_right)));
    return span;
}

fn classify_grid_span(span: f32) -> u32 {
    if (span < 0.015) { return 8u; }
    if (span < 0.040) { return 4u; }
    if (span < 0.100) { return 2u; }
    return 1u;
}

fn screen_grid_step(ordinal: u32) -> u32 {
    let span = screen_grid_span(ordinal);
    let current = classify_grid_span(span);
    let history = grid_history[ordinal];
    let leaf = leaves[ordinal];
    let valid = history.x == leaf.x && history.y == leaf.y
        && (history.z == 1u || history.z == 2u || history.z == 4u || history.z == 8u);
    if (!valid) { return current; }
    let refine = classify_grid_span(span / 1.2);
    let coarsen = classify_grid_span(span / 0.8);
    if (refine < history.z) { return refine; }
    if (coarsen > history.z) { return coarsen; }
    return history.z;
}

fn edge_vertex(edge: u32, segment: u32, side: u32, step: u32) -> u32 {
    let offset = segment * step + side * step;
    if (edge == 0u) { return offset; }
    if (edge == 1u) { return offset * 33u + 32u; }
    if (edge == 2u) { return 32u * 33u + (32u - offset); }
    return (32u - offset) * 33u;
}

fn skirt_index(local: u32) -> u32 {
    return local | 0x80000000u;
}

@compute @workgroup_size(1)
fn reset_active(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x == 0u) {
        atomicStore(&active_count[0], 0u);
        draw_list[0] = DrawCommand(0u, 1u, 0u, 0u);
    }
}

var<workgroup> selected_step: u32;
var<workgroup> selected_count: u32;
var<workgroup> selected_offset: u32;

// Cooperate on one patch per workgroup instead of serially emitting up to
// 2304 records in each lane. Only the leader classifies and reserves space.
@compute @workgroup_size(64)
fn classify_active(
    @builtin(workgroup_id) group: vec3<u32>,
    @builtin(local_invocation_index) lane: u32,
) {
    let ordinal = group.x;
    if (lane == 0u) {
        selected_count = 0u;
        if (ordinal < params.leaf_count && page_metadata[ordinal].z != 0u) {
            if (intersects_clip(ordinal)) {
                selected_step = screen_grid_step(ordinal);
                grid_history[ordinal] = vec4(
                    leaves[ordinal].x,
                    leaves[ordinal].y,
                    selected_step,
                    0u,
                );
                let cells = 32u / selected_step;
                selected_count = cells * cells * 2u + cells * 8u;
                selected_offset = atomicAdd(&active_count[0], selected_count);
            }
        }
    }
    workgroupBarrier();
    if (selected_count > 0u) {
        let step = selected_step;
        let cells = 32u / step;
        let surface_triangles = cells * cells * 2u;
        let active_ordinal = selected_offset;
        for (var triangle = lane; triangle < selected_count; triangle = triangle + 64u) {
            if (triangle < surface_triangles) {
                let cell = triangle / 2u;
                let x = (cell % cells) * step;
                let y = (cell / cells) * step;
                let a = y * 33u + x;
                let b = a + step;
                let c = a + step * 33u;
                let d = c + step;
                if ((triangle & 1u) == 0u) {
                    active_triangles[active_ordinal + triangle] = vec4(ordinal, a, b, c);
                } else {
                    active_triangles[active_ordinal + triangle] = vec4(ordinal, b, d, c);
                }
            } else {
                let skirt = triangle - surface_triangles;
                let segment = skirt / 2u;
                let edge = segment / cells;
                let edge_segment = segment % cells;
                let a = edge_vertex(edge, edge_segment, 0u, step);
                let b = edge_vertex(edge, edge_segment, 1u, step);
                if ((skirt & 1u) == 0u) {
                    active_triangles[active_ordinal + triangle] =
                        vec4(ordinal, a, skirt_index(a), b);
                } else {
                    active_triangles[active_ordinal + triangle] =
                        vec4(ordinal, b, skirt_index(a), skirt_index(b));
                }
            }
        }
    }
}

@compute @workgroup_size(1)
fn finalize_active(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x == 0u) {
        draw_list[0] = DrawCommand(
            atomicLoad(&active_count[0]) * 3u,
            1u,
            0u,
            0u,
        );
    }
}
"#
);

/// Procedural direct raster consumer for the generated GPU geometry. It uses
/// one indirect draw with a linear triangle vertex stream, matching the draw
/// submission shape of large_cbt while retaining the current page format.
const CBT_RASTER_WGSL: &str = concat!(
    include_str!("precision.wgsl"),
    include_str!("tile_frame.wgsl"),
    include_str!("ocean.wgsl"),
    r#"
struct Params {
    leaf_count: u32,
    vertices_per_patch: u32,
    radius_bits: u32,
    _padding: u32,
};

struct ViewUniforms {
    // Prefix of Bevy ViewUniform, including its photometric exposure.
    clip_from_world: mat4x4<f32>,
    unjittered_clip_from_world: mat4x4<f32>,
    world_from_clip: mat4x4<f32>,
    world_from_view: mat4x4<f32>,
    view_from_world: mat4x4<f32>,
    clip_from_view: mat4x4<f32>,
    view_from_clip: mat4x4<f32>,
    world_position: vec3<f32>,
    exposure: f32,
};

struct Lighting {
    ambient_lux: vec4<f32>,
    directions: array<vec4<f32>, 3>,
    colors_lux: array<vec4<f32>, 3>,
    ocean_wave: vec4<f32>,
    ocean_appearance: vec4<f32>,
    ocean_phase: vec4<f32>,
};
@group(0) @binding(9) var<uniform> lighting: Lighting;

@group(0) @binding(0) var<uniform> view: ViewUniforms;
@group(0) @binding(1) var<uniform> render_from_body: mat4x4<f32>;
@group(0) @binding(2) var<storage, read> generated_vertices: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(4) var<uniform> params: Params;
@group(0) @binding(5) var<storage, read> active_triangles: array<vec4<u32>>;
@group(0) @binding(8) var<storage, read> leaves: array<vec4<u32>>;
@group(0) @binding(11) var<storage, read> tile_frames: array<CbtTileFrame>;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) body_direction: vec3<f32>,
    @location(2) render_position: vec3<f32>,
    @location(3) height_m: f32,
    @location(4) material_uv: vec2<f32>,
    @location(5) @interpolate(flat) material_slot: u32,
};

@group(0) @binding(6) var albedo_texture: texture_2d<f32>;
@group(0) @binding(7) var surface_sampler: sampler;
@group(0) @binding(10) var roughness_texture: texture_2d<f32>;
@group(0) @binding(12) var material_texture: texture_2d_array<f32>;
@group(0) @binding(13) var material_sampler: sampler;
@group(0) @binding(14) var<storage, read> material_slots: array<vec4<u32>>;


fn surface_uv(position: vec3<f32>) -> vec2<f32> {
    let direction = normalize(position);
    // The baked map follows sphere::dir_from_latlon: east-positive longitude
    // is atan2(-z, x), with the zero meridian at texture u=0.
    var longitude = 0.0;
    if (dot(direction.xz, direction.xz) > 0.0) {
        longitude = atan2(-direction.z, direction.x) / (2.0 * 3.14159265);
    }
    let latitude = 0.5 - asin(clamp(direction.y, -1.0, 1.0)) / 3.14159265;
    return vec2(fract(longitude), clamp(latitude, 0.001, 0.999));
}

@vertex
fn vertex(
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOutput {
    var output: VertexOutput;
    let triangle = active_triangles[vertex_index / 3u];
    let corner = vertex_index % 3u;
    let ordinal = triangle.x;
    let page = page_metadata[ordinal];
    var local = triangle.y;
    if (corner == 1u) { local = triangle.z; }
    if (corner == 2u) { local = triangle.w; }
    let skirt = (local & 0x80000000u) != 0u;
    local = local & 0x7fffffffu;
    let generated_index = ordinal * params.vertices_per_patch + local;
    var position = generated_vertices[generated_index * 2u];
    let normal = generated_vertices[generated_index * 2u + 1u];
    let frame = tile_frames[ordinal];
    let body_direction = normalize(frame.geometry.normal.xyz * bitcast<f32>(params.radius_bits) + position.xyz);
    if (skirt) {
        let tile_depth = leaves[ordinal].z;
        // Match the CPU cover's bounded local apron. The exact skirt depth is
        // deliberately screen-irrelevant: it only seals a LOD boundary below
        // the visible surface and remains bounded for coarse horizon leaves.
        let skirt_depth = clamp(
            bitcast<f32>(params.radius_bits)
                / exp2(f32(max(tile_depth, 3u) - 3u) / 2.0)
                / 32.0
                * 1.5,
            24.0,
            256.0,
        );
        position = vec4(position.xyz - body_direction * skirt_depth, 1.0);
    }
    let render_position = select(vec4(0.0), cbt_render_position(frame, position.xyz, render_from_body), page.z != 0u);
    output.clip_position = view.clip_from_world * render_position;
    output.normal = normalize((render_from_body * vec4(normal.xyz, 0.0)).xyz);
    // The equirectangular map is body-fixed. Use the original planet-space
    // position; `render_position` is camera-relative after subtracting the
    // floating-origin eye and would project the map around the camera.
    output.body_direction = body_direction;
    output.render_position = render_position.xyz;
    output.height_m = select(normal.w, 1e9, skirt);
    let material = material_slots[ordinal];
    let tile_uv = vec2(f32(local % 33u), f32(local / 33u)) / 32.0;
    let page_uv = tile_uv * bitcast<f32>(material.y) + bitcast<vec2<f32>>(material.zw);
    output.material_uv = (page_uv * 125.0 + 1.5) / 128.0;
    output.material_slot = material.x;
    return output;
}

@fragment
fn fragment(input: VertexOutput) -> @location(0) vec4<f32> {
    let normal = normalize(input.normal);
    var illuminance = lighting.ambient_lux.rgb;
    for (var i = 0u; i < 3u; i = i + 1u) {
        illuminance += lighting.colors_lux[i].rgb * max(dot(normal, lighting.directions[i].xyz), 0.0);
    }
    // Project after interpolation so a triangle crossing longitude zero
    // cannot interpolate through the opposite side of the planet's map.
    let uv = surface_uv(input.body_direction);
    var dx = dpdx(uv);
    var dy = dpdy(uv);
    dx.x -= round(dx.x);
    dy.x -= round(dy.x);
    // Pages resolve material bands down to 32 m. Once a pixel spans that
    // scale, use the continuous planet map rather than aliasing those bands
    // into different colours in each geometry LOD. The blend follows the
    // physical footprint, never a flat per-tile level or residency flag.
    let direction = normalize(input.body_direction);
    let footprint = max(length(dpdx(direction)), length(dpdy(direction)));
    let footprint_m = footprint * bitcast<f32>(params.radius_bits);
    var detail_weight = 1.0 - smoothstep(16.0, 32.0, footprint_m);
    if (input.material_slot == 0xffffffffu) { detail_weight = 0.0; }
    var albedo = vec3(0.0);
    var roughness = 0.0;
    let page_dx = dpdx(input.material_uv);
    let page_dy = dpdy(input.material_uv);
    if (detail_weight > 0.0) {
        let detail = textureSampleGrad(material_texture, material_sampler,
            input.material_uv, i32(input.material_slot), page_dx, page_dy);
        albedo = detail.rgb * detail_weight;
        roughness = detail.a * detail_weight;
    }
    if (detail_weight < 1.0) {
        albedo += textureSampleGrad(albedo_texture, surface_sampler, uv, dx, dy).rgb
            * (1.0 - detail_weight);
        roughness += textureSampleGrad(roughness_texture, surface_sampler, uv, dx, dy).g
            * (1.0 - detail_weight);
    }
    var radiance = albedo * illuminance / OCEAN_PI;
    // Derivatives must be evaluated outside the non-uniform water branch.
    // Canonical pages clamp ocean to datum; the material map excludes ice.
    let water = (1.0 - smoothstep(0.22, 0.65, roughness))
        * (1.0 - smoothstep(0.25, 1.0, input.height_m))
        * lighting.ocean_appearance.z;
    if (water > 0.0) {
        let rotation = mat3x3<f32>(render_from_body[0].xyz, render_from_body[1].xyz, render_from_body[2].xyz);
        let up = normalize(rotation * direction);
        let eye = ocean_safe_normalize(view.world_position - input.render_position, normal);
        let wave_normal = ocean_wave_normal(direction, normal, rotation,
            lighting.ocean_phase.xy, footprint, lighting.ocean_wave);
        let f0 = lighting.ocean_appearance.x;
        let fresnel = ocean_schlick(f0, max(dot(wave_normal, eye), 0.0));
        var reflection = vec3(0.0);
        var sky_illuminance = lighting.ambient_lux.rgb;
        for (var i = 0u; i < 3u; i = i + 1u) {
            let light = lighting.directions[i].xyz;
            reflection += ocean_sun_reflection(wave_normal, eye, light,
                lighting.colors_lux[i].rgb, max(roughness, 0.16), f0);
            sky_illuminance += lighting.colors_lux[i].rgb
                * max(dot(up, light), 0.0) * lighting.ocean_appearance.y;
        }
        let sky = sky_illuminance / OCEAN_PI;
        let reflected = reflect(-eye, wave_normal);
        reflection += ocean_sky_radiance(reflected, up,
            sky * vec3(0.10, 0.19, 0.38), sky * vec3(0.42, 0.52, 0.60),
            sky * vec3(0.015, 0.018, 0.015)) * fresnel;
        radiance = mix(radiance, radiance * (1.0 - fresnel) + reflection, water);
    }
    // Same pre-exposed linear HDR convention as Bevy PBR.
    return vec4(radiance * view.exposure, 1.0);
}
"#
);

/// Hardware mesh-shader consumer for the same quantized CBT pages. A 33x33
/// patch is emitted as sixteen 8x8 meshlets, so the shader stays within the
/// recommended 256-vertex / 256-primitive minimum while preserving exact page
/// sampling. There is deliberately no task shader yet: a single direct mesh
/// dispatch covers all leaves and the page metadata turns missing pages into
/// zero-output workgroups.
#[cfg(feature = "mesh-shaders")]
const CBT_MESH_WGSL: &str = concat!(
    "enable wgpu_mesh_shader;\n",
    include_str!("precision.wgsl"),
    include_str!("tile_frame.wgsl"),
    include_str!("surface_sample.wgsl"),
    r#"

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
@group(0) @binding(4) var<storage, read> tile_frames: array<CbtTileFrame>;
@group(1) @binding(0) var<uniform> view: ViewUniforms;
@group(1) @binding(1) var<uniform> render_from_body: mat4x4<f32>;

var<workgroup> mesh_output: MeshOutput;

@mesh(mesh_output) @workgroup_size(64)
fn build_mesh(
    @builtin(local_invocation_index) invocation: u32,
    @builtin(workgroup_id) workgroup: vec3<u32>,
) {
    let leaf = workgroup.y;
    let meshlet = workgroup.x;
    let page = page_metadata[leaf];
    let valid = page.z != 0u;
    if (invocation == 0u) {
        mesh_output.vertex_count = select(0u, 81u, valid);
        mesh_output.primitive_count = select(0u, 128u, valid);
    }
    if (!valid) {
        return;
    }

    let frame = tile_frames[leaf];
    for (var i = invocation; i < 81u; i += 64u) {
        let local_x = i % 9u;
        let local_y = i / 9u;
        let patch_x = (meshlet % 4u) * 8u + local_x;
        let patch_y = (meshlet / 4u) * 8u + local_y;
        let uv = vec2(f32(patch_x) / 32.0, f32(patch_y) / 32.0);
        let sample = cbt_surface_sample(frame.geometry, page, uv);
        mesh_output.vertices[i].clip_position =
            view.clip_from_world * cbt_render_position(frame, sample.local_position, render_from_body);
        mesh_output.vertices[i].normal =
            normalize((render_from_body * vec4(sample.normal, 0.0)).xyz);
    }
    for (var i = invocation; i < 128u; i += 64u) {
        let cell = i / 2u;
        let triangle = i & 1u;
        let cell_x = cell % 8u;
        let cell_y = cell / 8u;
        let base = cell_y * 9u + cell_x;
        let right = base + 1u;
        let down = base + 9u;
        let diagonal = down + 1u;
        mesh_output.primitives[i].indices = select(
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
"#
);

/// Installs extraction and GPU preparation for the universal CBT plugin.
pub(super) struct CbtRenderPlugin;

impl Plugin for CbtRenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CbtRenderMaterialPages>()
            .init_resource::<CbtGpuPresentation>();
        let has_render_app = app.get_sub_app_mut(RenderApp).is_some();
        if has_render_app {
            app.add_plugins(ExtractResourcePlugin::<CbtRenderTopology>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderPages>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderSurface>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderMaterial>::default())
                .add_plugins(ExtractResourcePlugin::<CbtRenderMaterialPages>::default())
                .add_plugins(ExtractResourcePlugin::<CbtGpuPresentation>::default())
                .add_plugins(ExtractResourcePlugin::<MaterialStorageSetting>::default());
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
            storage_binding(6, true),
            BindGroupLayoutEntry {
                binding: 5,
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
    let build_pipeline = device.create_compute_pipeline(&RawComputePipelineDescriptor {
        label: Some("thessa-cbt-geometry-pipeline"),
        layout: Some(&pipeline_layout),
        module: &shader,
        entry_point: Some("build_geometry"),
        compilation_options: Default::default(),
        cache: None,
    });
    let classifier_bind_group_layout = device.create_bind_group_layout(
        "thessa-cbt-classifier-layout",
        &[
            storage_binding(0, true),
            storage_binding(1, true),
            storage_binding(2, false),
            storage_binding(3, false),
            storage_binding(4, false),
            BindGroupLayoutEntry {
                binding: 5,
                visibility: ShaderStages::COMPUTE,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: None,
                },
                count: None,
            },
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
            BindGroupLayoutEntry {
                binding: 7,
                visibility: ShaderStages::COMPUTE,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            storage_binding(8, true),
            storage_binding(9, true),
            storage_binding(10, false),
        ],
    );
    let classifier_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: Some("thessa-cbt-classifier-pipeline-layout"),
        bind_group_layouts: &[Some(&classifier_bind_group_layout)],
        immediate_size: 0,
    });
    let classifier_shader = device.create_and_validate_shader_module(ShaderModuleDescriptor {
        label: Some("thessa-cbt-classifier-shader"),
        source: ShaderSource::Wgsl(Cow::Borrowed(CBT_CLASSIFY_WGSL)),
    });
    let classifier_reset_pipeline = device.create_compute_pipeline(&RawComputePipelineDescriptor {
        label: Some("thessa-cbt-classifier-reset-pipeline"),
        layout: Some(&classifier_pipeline_layout),
        module: &classifier_shader,
        entry_point: Some("reset_active"),
        compilation_options: Default::default(),
        cache: None,
    });
    let classifier_pipeline = device.create_compute_pipeline(&RawComputePipelineDescriptor {
        label: Some("thessa-cbt-classifier-pipeline"),
        layout: Some(&classifier_pipeline_layout),
        module: &classifier_shader,
        entry_point: Some("classify_active"),
        compilation_options: Default::default(),
        cache: None,
    });
    let classifier_finalize_pipeline =
        device.create_compute_pipeline(&RawComputePipelineDescriptor {
            label: Some("thessa-cbt-classifier-finalize-pipeline"),
            layout: Some(&classifier_pipeline_layout),
            module: &classifier_shader,
            entry_point: Some("finalize_active"),
            compilation_options: Default::default(),
            cache: None,
        });
    let raster_bind_group_layout = device.create_bind_group_layout(
        "thessa-cbt-raster-layout",
        &[
            BindGroupLayoutEntry {
                binding: 0,
                visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 1,
                visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
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
                // The fragment stage uses radius_bits to convert the
                // direction derivative into a physical material footprint.
                visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 5,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 8,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 6,
                visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                ty: BindingType::Texture {
                    sample_type: TextureSampleType::Float { filterable: true },
                    view_dimension: TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 7,
                visibility: ShaderStages::VERTEX | ShaderStages::FRAGMENT,
                ty: BindingType::Sampler(SamplerBindingType::Filtering),
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 11,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 10,
                visibility: ShaderStages::FRAGMENT,
                ty: BindingType::Texture {
                    sample_type: TextureSampleType::Float { filterable: true },
                    view_dimension: TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 9,
                visibility: ShaderStages::FRAGMENT,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 12,
                visibility: ShaderStages::FRAGMENT,
                ty: BindingType::Texture {
                    sample_type: TextureSampleType::Float { filterable: true },
                    view_dimension: TextureViewDimension::D2Array,
                    multisampled: false,
                },
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 13,
                visibility: ShaderStages::FRAGMENT,
                ty: BindingType::Sampler(SamplerBindingType::Filtering),
                count: None,
            },
            BindGroupLayoutEntry {
                binding: 14,
                visibility: ShaderStages::VERTEX,
                ty: BindingType::Buffer {
                    ty: BufferBindingType::Storage { read_only: true },
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
                    binding: 4,
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
        pipeline: build_pipeline,
    });
    commands.insert_resource(CbtGpuClassifier {
        bind_group_layout: classifier_bind_group_layout,
        reset_pipeline: classifier_reset_pipeline,
        classify_pipeline: classifier_pipeline,
        finalize_pipeline: classifier_finalize_pipeline,
        classified_view_generation: u64::MAX,
        classified_clip_from_world: None,
        classified_transform_generation: u64::MAX,
        classified_topology_generation: u64::MAX,
        classified_pages_generation: u64::MAX,
        classified_surface_generation: u64::MAX,
        cached_bind_group: None,
        cached_bind_key: None,
    });
    commands.insert_resource(CbtGpuRasterPipeline {
        bind_group_layout: raster_bind_group_layout,
        pipeline_layout: raster_pipeline_layout,
        shader: raster_shader,
        pipelines: HashMap::default(),
        cached_bind_group: None,
        cached_bind_key: None,
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

/// Packed words for one GPU height page (`33 x 33` quantized residuals,
/// two `i16` per word). The GPU path always bakes `33`-grid pages; the
/// stride only grows if a larger page is ever published (rare full
/// re-upload, same cost as the old rebuild-everything path).
const HEIGHT_SLOT_MIN_STRIDE_WORDS: usize = (GPU_GRID_SIZE * GPU_GRID_SIZE).div_ceil(2);
const HEIGHT_SLOT_INITIAL_CAPACITY: usize = 1024;
const HEIGHT_SLOT_MAX_CAPACITY: usize = 8192;

fn height_page_words(page: &HeightPage) -> usize {
    page.residuals().len().div_ceil(2).max(1)
}

fn height_leaf_node_id(record: &CbtLeafRecord) -> u64 {
    u64::from(record[0]) | (u64::from(record[1]) << 32)
}

fn height_leaf_renderable(depth: u32) -> bool {
    // Same predicate the old rebuild-everything loop used (`depth < 3`
    // short-circuits before the subtraction); missing pages stay `[0; 4]`
    // so the shader culls those leaves exactly as before.
    depth >= 3 && (depth - 3).is_multiple_of(2)
}

/// Upload height pages through stable GPU slots instead of rebuilding every
/// page on each arrival.
///
/// `node_id -> slot` is stable across topology reorders (the small
/// per-ordinal metadata is rewritten to reference unchanged word offsets),
/// so a reorder uploads no height data at all. A new or changed page packs
/// and uploads exactly one `stride` range via `write_buffer_range`. A full
/// re-upload happens only when the slot table grows or the stride changes.
/// `leaf_records` is rewritten only when the topology itself changed.
///
/// Returns the per-ordinal metadata referencing stable word offsets.
fn sync_height_slots(
    gpu: &mut CbtGpuBuffers,
    device: &RenderDevice,
    queue: &RenderQueue,
    records: &[CbtLeafRecord],
    pages: &CbtRenderPages,
    topology_changed: bool,
) -> Vec<[u32; 4]> {
    // Desired resident pages in topology order with per-page versions.
    let mut desired: Vec<(u64, u64)> = Vec::with_capacity(records.len());
    let mut stride = HEIGHT_SLOT_MIN_STRIDE_WORDS;
    for record in records {
        if !height_leaf_renderable(record[2]) {
            continue;
        }
        let node_id = height_leaf_node_id(record);
        let Some(version) = pages.page_version(node_id) else {
            continue;
        };
        let Some(page) = pages.get(node_id) else {
            continue;
        };
        stride = stride.max(height_page_words(page));
        desired.push((node_id, version));
    }

    let mut full_upload = false;
    if desired.len() > gpu.height_slots.capacity() {
        let grown = desired
            .len()
            .next_power_of_two()
            .max(gpu.height_slots.capacity() * 2)
            .clamp(1, HEIGHT_SLOT_MAX_CAPACITY);
        gpu.height_slots = SlotCache::new(grown).expect("height slot capacity is non-zero");
        full_upload = true;
    }
    if stride != gpu.height_stride_words {
        gpu.height_stride_words = stride;
        full_upload = true;
    }
    let changes: Vec<SlotChange> = gpu.height_slots.update(&desired);

    // Size the CPU mirror to the full slot table; the GPU buffer follows via
    // `reserve` (which reallocates — and drops previous contents — only on
    // growth, in which case a full upload is required anyway).
    let target_len = gpu.height_slots.capacity() * gpu.height_stride_words;
    if gpu.page_residuals.len() != target_len {
        gpu.page_residuals.clear();
        gpu.page_residuals.reserve_internal(target_len);
        gpu.page_residuals
            .extend(std::iter::repeat_n(0, target_len));
        full_upload = true;
    }
    let buffer_id_before = gpu.page_residuals.buffer().map(|buffer| buffer.id());
    gpu.page_residuals.reserve(target_len, device);
    if gpu.page_residuals.buffer().map(|buffer| buffer.id()) != buffer_id_before {
        full_upload = true;
    }

    if full_upload {
        // Zero the mirror (grow/stride change may have left stale words or a
        // fresh zeroed allocation) and pack every resident page once.
        // Desired pages beyond a clamped capacity have no slot and stay
        // unreferenced (their metadata is `[0; 4]`, culled by the shader).
        for index in 0..target_len {
            gpu.page_residuals.set(index as u32, 0);
        }
        for (node_id, _) in &desired {
            if let Some(slot) = gpu.height_slots.slot(*node_id) {
                write_height_slot(gpu, pages, *node_id, slot);
            }
        }
        if target_len == 0 {
            gpu.page_residuals.extend([0]);
        }
        gpu.page_residuals.write_buffer(device, queue);
    } else {
        // Incremental path: only changed slots touch the CPU mirror and the
        // GPU buffer. Topology reorder with unchanged pages yields zero
        // changes here — no height upload at all.
        let mut fell_back_to_full = false;
        for (index, change) in changes.iter().enumerate() {
            write_height_slot(gpu, pages, change.node_id, change.slot);
            if fell_back_to_full {
                continue;
            }
            let start = change.slot as usize * gpu.height_stride_words;
            let end = start + gpu.height_stride_words;
            if gpu
                .page_residuals
                .write_buffer_range(queue, start..end)
                .is_err()
            {
                // Range upload requires an initialized buffer covering the
                // range; pack the remaining changes into the mirror first so
                // the full upload below carries current data for every slot,
                // rather than leaving later slots stale.
                for rest in &changes[index + 1..] {
                    write_height_slot(gpu, pages, rest.node_id, rest.slot);
                }
                gpu.page_residuals.write_buffer(device, queue);
                fell_back_to_full = true;
            }
        }
        if gpu.page_residuals.is_empty() {
            // Keep the binding valid while nothing is resident yet.
            gpu.page_residuals.extend([0]);
            gpu.page_residuals.write_buffer(device, queue);
        }
    }

    if topology_changed {
        gpu.leaf_records.clear();
        gpu.leaf_records.extend(records.iter().copied());
        gpu.leaf_records.write_buffer(device, queue);
    }

    // Per-ordinal metadata is small (4 words per leaf); rebuild it to
    // reference the stable slot offsets. Missing pages stay `[0; 4]` so the
    // shader culls those leaves exactly as before.
    let mut metadata = Vec::with_capacity(records.len());
    for record in records {
        if !height_leaf_renderable(record[2]) {
            metadata.push([0; 4]);
            continue;
        }
        let node_id = height_leaf_node_id(record);
        let (Some(page), Some(slot)) = (pages.get(node_id), gpu.height_slots.slot(node_id)) else {
            metadata.push([0; 4]);
            continue;
        };
        metadata.push([
            page.base_height_m().to_bits(),
            page.residual_scale_m().to_bits(),
            page.grid_size(),
            (slot as usize * gpu.height_stride_words) as u32,
        ]);
    }
    metadata
}

/// Pack one resident page into its slot range of the residuals mirror.
/// Pages larger than the stride cannot occur without a stride change first
/// (which forces a full upload); the words are still truncated defensively
/// so a slot never overlaps its neighbour.
fn write_height_slot(gpu: &mut CbtGpuBuffers, pages: &CbtRenderPages, node_id: u64, slot: u32) {
    let Some(page) = pages.get(node_id) else {
        return;
    };
    let mut packed = Vec::with_capacity(height_page_words(page));
    pack_page_residuals(page, &mut packed);
    let start = slot as usize * gpu.height_stride_words;
    for (offset, word) in packed.iter().enumerate() {
        if offset >= gpu.height_stride_words {
            break;
        }
        gpu.page_residuals.set((start + offset) as u32, *word);
    }
    for offset in packed.len()..gpu.height_stride_words {
        gpu.page_residuals.set((start + offset) as u32, 0);
    }
}

#[allow(clippy::too_many_arguments)]
fn prepare_cbt_gpu_buffers(
    topology: Option<Res<CbtRenderTopology>>,
    material: Option<Res<CbtRenderMaterial>>,
    material_pages: Res<CbtRenderMaterialPages>,
    pages: Option<Res<CbtRenderPages>>,
    surface: Option<Res<CbtRenderSurface>>,
    storage: Option<Res<MaterialStorageSetting>>,
    mut gpu: ResMut<CbtGpuBuffers>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let (Some(topology), Some(pages), Some(surface)) = (topology, pages, surface) else {
        return;
    };
    gpu.complete_pages = !topology.records().is_empty()
        && topology.records().iter().all(|r| {
            let id = u64::from(r[0]) | (u64::from(r[1]) << 32);
            r[2] >= 3 && r[2] <= 37 && (r[2] - 3).is_multiple_of(2) && pages.contains_page(id)
        });
    if surface.gpu_raster_enabled() {
        let policy = storage.map(|s| s.0).unwrap_or_default();
        gpu.material_array
            .get_or_insert_with(|| material_render::MaterialArray::new(&device))
            .prepare_storage(&topology, &material_pages, &device, &queue, policy);
    }
    if let Some(material) = material {
        let values: Vec<_> = std::iter::once(material.ambient_lux)
            .chain(material.light_directions)
            .chain(material.light_colors_lux)
            .chain({
                let ocean = material.ocean.unwrap_or_default();
                let enabled = material.ocean.is_some() && material.roughness.is_some();
                [
                    [
                        ocean.wave_slope.max(0.0),
                        std::f32::consts::TAU * surface.radius_m() / ocean.wavelength_m.max(1.0),
                        ocean.secondary_frequency_ratio,
                        ocean.secondary_slope_ratio,
                    ],
                    [
                        ocean.reflectance.clamp(0.0, 1.0),
                        ocean.sky_scatter_fraction.clamp(0.0, 1.0),
                        f32::from(enabled),
                        0.0,
                    ],
                    [
                        material.ocean_wave_phases[0],
                        material.ocean_wave_phases[1],
                        0.0,
                        0.0,
                    ],
                ]
            })
            .collect();
        if values
            .iter()
            .enumerate()
            .any(|(i, value)| gpu.lighting.get(i as u32) != Some(value))
        {
            gpu.lighting.clear();
            gpu.lighting.extend(values);
            gpu.lighting.write_buffer(&device, &queue);
        }
    }
    if gpu.topology_generation == topology.generation()
        && gpu.pages_generation == pages.generation()
        && gpu.surface_generation == surface.generation()
        && gpu.transform_generation == surface.transform_generation()
    {
        return;
    }

    if gpu.topology_generation != topology.generation()
        || gpu.surface_generation != surface.generation()
    {
        gpu.tile_anchors = topology
            .records()
            .iter()
            .map(|record| {
                let depth = record[2];
                if depth < 3 || !(depth - 3).is_multiple_of(2) {
                    return None;
                }
                let level = (depth - 3) / 2;
                let id = u64::from(record[0]) | (u64::from(record[1]) << 32);
                let face = ((id >> (2 * level)) & 7) as u8;
                let (mut x, mut y) = (0, 0);
                for bit in 0..level {
                    x |= (((id >> (2 * bit + 1)) & 1) as u32) << bit;
                    y |= (((id >> (2 * bit)) & 1) as u32) << bit;
                }
                let key = crate::precision::TileKey::new(face, level as u8, x, y)?;
                crate::precision::TileAnchor::new(key, f64::from(surface.radius_m()))
            })
            .collect();
    }
    if gpu.topology_generation != topology.generation()
        || gpu.surface_generation != surface.generation()
        || gpu.transform_generation != surface.transform_generation()
    {
        let transform = bevy::math::DMat4::from_cols_array(&surface.render_from_body_f64);
        let frames: Vec<_> = gpu
            .tile_anchors
            .iter()
            .map(|anchor| {
                let Some(anchor) = anchor else {
                    return [[0.0; 4]; 7];
                };
                let mut frame = anchor.to_gpu([0.0; 3]).expect("finite tile anchor");
                let relative = transform
                    .transform_point3(bevy::math::DVec3::from_array(anchor.anchor_body_m))
                    .to_array();
                for (axis, (hi, rel)) in frame
                    .anchor_hi_m
                    .iter_mut()
                    .zip(relative.iter())
                    .enumerate()
                {
                    *hi = *rel as f32;
                    frame.anchor_lo_m[axis] = (*rel - f64::from(*hi)) as f32;
                }
                [
                    frame.anchor_hi_m,
                    frame.anchor_lo_m,
                    frame.raw_center,
                    frame.raw_axis_u,
                    frame.raw_axis_v,
                    frame.normal,
                    frame.radius_half_extent_len,
                ]
            })
            .collect();
        gpu.tile_frames.clear();
        gpu.tile_frames.extend(frames);
        // Keep a valid binding even before the first published cover.
        if gpu.tile_anchors.is_empty() {
            gpu.tile_frames.push([[0.0; 4]; 7]);
        }
        gpu.tile_frames.write_buffer(&device, &queue);
    }

    if gpu.transform_generation != surface.transform_generation() {
        gpu.surface_transform.clear();
        gpu.surface_transform.push(surface.render_from_body());
        gpu.surface_transform.write_buffer(&device, &queue);
        gpu.transform_generation = surface.transform_generation();
    }

    if gpu.topology_generation == topology.generation()
        && gpu.pages_generation == pages.generation()
        && gpu.surface_generation == surface.generation()
    {
        return;
    }

    let records = topology.records();
    let topology_changed = gpu.topology_generation != topology.generation();
    // Stable slot upload: one arriving page rewrites a single slot range and
    // the small metadata; a topology reorder with unchanged pages uploads no
    // height data at all.
    let metadata = sync_height_slots(&mut gpu, &device, &queue, records, &pages, topology_changed);

    // Compute owns these outputs. Reserve GPU storage without constructing
    // and uploading a CPU mirror of zeros on each streamed page batch.
    gpu.patch_records.reserve(records.len().max(1), &device);
    // This buffer is GPU-owned state. New capacity is zero initialized by
    // wgpu, while existing ordinals retain their history across view changes.
    gpu.grid_history.reserve(records.len().max(1), &device);

    gpu.page_metadata.clear();
    gpu.page_metadata.extend(metadata);
    gpu.page_metadata.write_buffer(&device, &queue);

    if !surface.gpu_mesh_enabled() {
        gpu.vertices
            .reserve((records.len() * GPU_VERTEX_COUNT_PER_PATCH).max(1), &device);
        gpu.draw_list.reserve(1, &device);
        gpu.active_triangles.reserve(
            (records.len() * GPU_TRIANGLE_COUNT_PER_PATCH).max(1),
            &device,
        );
        gpu.active_count.reserve(1, &device);
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
        Some(params_buffer),
        Some(frames_buffer),
    ) = (
        gpu.leaf_records.buffer(),
        gpu.patch_records.buffer(),
        gpu.page_metadata.buffer(),
        gpu.page_residuals.buffer(),
        gpu.vertices.buffer(),
        gpu.params.buffer(),
        gpu.tile_frames.buffer(),
    )
    else {
        return;
    };
    let key = GeometryBindKey {
        leaf: leaf_buffer.id(),
        patch: patch_buffer.id(),
        metadata: metadata_buffer.id(),
        residual: residual_buffer.id(),
        vertex: vertex_buffer.id(),
        params: params_buffer.id(),
        frames: frames_buffer.id(),
    };
    // Rebuild only when a buffer object was reallocated (new BufferId).
    // Contents changes via queue writes keep the same binding.
    if gpu.geometry_bind_key != Some(key) {
        gpu.geometry_bind_group = Some(context.render_device().create_bind_group(
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
                    resource: BindingResource::Buffer(params_buffer.as_entire_buffer_binding()),
                },
                BindGroupEntry {
                    binding: 6,
                    resource: BindingResource::Buffer(frames_buffer.as_entire_buffer_binding()),
                },
            ],
        ));
        gpu.geometry_bind_key = Some(key);
    }
    let Some(bind_group) = gpu.geometry_bind_group.clone() else {
        return;
    };
    let diagnostics = context.diagnostic_recorder();
    {
        let mut pass = context
            .command_encoder()
            .begin_compute_pass(&ComputePassDescriptor {
                label: Some("thessa-cbt-height-page-geometry"),
                timestamp_writes: None,
            });
        let pass_span = diagnostics
            .as_ref()
            .map(|diagnostics| diagnostics.pass_span(&mut pass, "terrain_geometry"));
        pass.set_pipeline(&pipeline.pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let total = gpu
            .leaf_count
            .saturating_mul(GPU_VERTEX_COUNT_PER_PATCH as u32);
        pass.dispatch_workgroups(total.div_ceil(64), 1, 1);
        if let Some(pass_span) = pass_span {
            pass_span.end(&mut pass);
        }
    }
    gpu.generated_topology_generation = gpu.topology_generation;
    gpu.generated_pages_generation = gpu.pages_generation;
    gpu.generated_surface_generation = gpu.surface_generation;
}

#[allow(clippy::too_many_arguments)]
fn draw_cbt_geometry(
    presentation: Res<CbtGpuPresentation>,
    surface: Option<Res<CbtRenderSurface>>,
    material: Option<Res<CbtRenderMaterial>>,
    gpu: Option<Res<CbtGpuBuffers>>,
    classifier: Option<ResMut<CbtGpuClassifier>>,
    raster: Option<ResMut<CbtGpuRasterPipeline>>,
    view: ViewQuery<(
        &ExtractedCamera,
        &ExtractedView,
        &ViewTarget,
        &ViewDepthTexture,
        &ViewUniformOffset,
    )>,
    render_adapter: Res<RenderAdapter>,
    images: Res<RenderAssets<GpuImage>>,
    view_uniforms: Res<ViewUniforms>,
    mut context: RenderContext,
) {
    let (Some(surface), Some(gpu), Some(mut classifier), Some(mut raster)) =
        (surface, gpu, classifier, raster)
    else {
        return;
    };
    if !surface.gpu_raster_enabled()
        || !surface.gpu_surface_ready()
        || surface.gpu_mesh_enabled()
        || gpu.leaf_count() == 0
    {
        return;
    }
    let mut draw_attempt = presentation.attempt(&surface, material.as_deref());
    if !gpu.complete_pages {
        return;
    }
    let (camera, extracted_view, target, depth, view_uniform_offset) = view.into_inner();
    let (
        Some(leaf_buffer),
        Some(vertex_buffer),
        Some(metadata_buffer),
        Some(draw_buffer),
        Some(active_triangles_buffer),
        Some(active_count_buffer),
        Some(grid_history_buffer),
        Some(params_buffer),
        Some(surface_buffer),
        Some(frames_buffer),
    ) = (
        gpu.leaf_buffer(),
        gpu.vertex_buffer(),
        gpu.page_metadata_buffer(),
        gpu.draw_list_buffer(),
        gpu.active_triangles_buffer(),
        gpu.active_count.buffer(),
        gpu.grid_history_buffer(),
        gpu.params_buffer(),
        gpu.surface_transform_buffer(),
        gpu.tile_frames.buffer(),
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
    // Clone out of the map so the bind-group cache below can mutably
    // borrow `raster` without holding the map borrow across it.
    let pipeline = raster
        .pipelines
        .get(&format)
        .cloned()
        .expect("CBT raster pipeline inserted above");
    // Do not submit grey fallback-textured CBT patches while the canonical
    // albedo is still loading. Main-world visibility waits for the successful
    // draw acknowledgement below, not merely for the CPU image asset.
    let Some(material) = material.as_ref() else {
        return;
    };
    let Some(albedo_image) = images.get(&material.albedo) else {
        return;
    };
    let roughness_image = match material.roughness.as_ref() {
        Some(map) => match images.get(map) {
            Some(image) => image,
            None => return,
        },
        // A valid placeholder keeps the optional material binding portable.
        // The material uniform disables ocean shading when no map is supplied.
        None => albedo_image,
    };
    let Some(lighting_buffer) = gpu.lighting.buffer() else {
        return;
    };
    let Some(material_array) = gpu.material_array.as_ref() else {
        return;
    };
    let Some(material_slots) = material_array.slots.buffer() else {
        return;
    };
    let Some(view_uniform_id) = view_uniform_buffer_id(&view_uniforms) else {
        return;
    };
    let raster_key = RasterBindKey {
        view_uniforms: view_uniform_id,
        surface: surface_buffer.id(),
        vertex: vertex_buffer.id(),
        metadata: metadata_buffer.id(),
        params: params_buffer.id(),
        active_triangles: active_triangles_buffer.id(),
        leaf: leaf_buffer.id(),
        lighting: lighting_buffer.id(),
        frames: frames_buffer.id(),
        material_slots: material_slots.id(),
        albedo_view: albedo_image.texture_view.id(),
        albedo_sampler: albedo_image.sampler.id(),
        roughness_view: roughness_image.texture_view.id(),
        roughness_sampler: roughness_image.sampler.id(),
        material_view: material_array.view.id(),
        material_sampler: material_array.sampler.id(),
    };
    // Camera motion flows through the dynamic view offset; rebuild only on
    // buffer/texture identity change (realloc or residency turnover).
    if raster.cached_bind_key != Some(raster_key) {
        raster.cached_bind_group = Some(context.render_device().create_bind_group(
            "thessa-cbt-raster-bind-group",
            &raster.bind_group_layout,
            &[
                BindGroupEntry {
                    binding: 0,
                    resource: view_binding.clone(),
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
                BindGroupEntry {
                    binding: 5,
                    resource: BindingResource::Buffer(
                        active_triangles_buffer.as_entire_buffer_binding(),
                    ),
                },
                BindGroupEntry {
                    binding: 8,
                    resource: BindingResource::Buffer(leaf_buffer.as_entire_buffer_binding()),
                },
                BindGroupEntry {
                    binding: 6,
                    resource: BindingResource::TextureView(&albedo_image.texture_view),
                },
                BindGroupEntry {
                    binding: 7,
                    resource: BindingResource::Sampler(&albedo_image.sampler),
                },
                BindGroupEntry {
                    binding: 9,
                    resource: BindingResource::Buffer(lighting_buffer.as_entire_buffer_binding()),
                },
                BindGroupEntry {
                    binding: 10,
                    resource: BindingResource::TextureView(&roughness_image.texture_view),
                },
                BindGroupEntry {
                    binding: 11,
                    resource: BindingResource::Buffer(frames_buffer.as_entire_buffer_binding()),
                },
                BindGroupEntry {
                    binding: 12,
                    resource: BindingResource::TextureView(&material_array.view),
                },
                BindGroupEntry {
                    binding: 13,
                    resource: BindingResource::Sampler(&material_array.sampler),
                },
                BindGroupEntry {
                    binding: 14,
                    resource: BindingResource::Buffer(material_slots.as_entire_buffer_binding()),
                },
            ],
        ));
        raster.cached_bind_key = Some(raster_key);
    }
    let Some(bind_group) = raster.cached_bind_group.clone() else {
        return;
    };
    let supports_indirect = render_adapter
        .get_downlevel_capabilities()
        .flags
        .contains(DownlevelFlags::INDIRECT_EXECUTION);
    if !supports_indirect {
        return;
    }
    let diagnostics = context.diagnostic_recorder();
    // Forward direction alone misses roll and aspect-ratio changes. Cache
    // against the actual render view as well as the adapter's camera state.
    let clip_from_world = extracted_view
        .clip_from_world
        .unwrap_or_else(|| {
            extracted_view.clip_from_view * extracted_view.world_from_view.to_matrix().inverse()
        })
        .to_cols_array();
    let needs_classification = classifier.classified_clip_from_world != Some(clip_from_world)
        || classifier.classified_view_generation != surface.view_generation()
        || classifier.classified_transform_generation != surface.transform_generation()
        || classifier.classified_topology_generation != gpu.topology_generation
        || classifier.classified_pages_generation != gpu.pages_generation
        || classifier.classified_surface_generation != gpu.surface_generation;
    if needs_classification {
        // The bind group itself is independent of the view matrix: view
        // changes ride the dynamic offset below. Rebuild only when a bound
        // buffer object is reallocated.
        let classifier_key =
            view_uniform_buffer_id(&view_uniforms).map(|view_uniforms| ClassifierBindKey {
                view_uniforms,
                metadata: metadata_buffer.id(),
                vertex: vertex_buffer.id(),
                active_triangles: active_triangles_buffer.id(),
                active_count: active_count_buffer.id(),
                draw: draw_buffer.id(),
                surface: surface_buffer.id(),
                params: params_buffer.id(),
                leaf: leaf_buffer.id(),
                frames: frames_buffer.id(),
                grid_history: grid_history_buffer.id(),
            });
        if let Some(key) = classifier_key
            && classifier.cached_bind_key != Some(key)
        {
            classifier.cached_bind_group = Some(context.render_device().create_bind_group(
                "thessa-cbt-classifier-bind-group",
                &classifier.bind_group_layout,
                &[
                    BindGroupEntry {
                        binding: 0,
                        resource: BindingResource::Buffer(
                            metadata_buffer.as_entire_buffer_binding(),
                        ),
                    },
                    BindGroupEntry {
                        binding: 1,
                        resource: BindingResource::Buffer(vertex_buffer.as_entire_buffer_binding()),
                    },
                    BindGroupEntry {
                        binding: 2,
                        resource: BindingResource::Buffer(
                            active_triangles_buffer.as_entire_buffer_binding(),
                        ),
                    },
                    BindGroupEntry {
                        binding: 3,
                        resource: BindingResource::Buffer(
                            active_count_buffer.as_entire_buffer_binding(),
                        ),
                    },
                    BindGroupEntry {
                        binding: 4,
                        resource: BindingResource::Buffer(draw_buffer.as_entire_buffer_binding()),
                    },
                    BindGroupEntry {
                        binding: 5,
                        resource: view_binding,
                    },
                    BindGroupEntry {
                        binding: 6,
                        resource: BindingResource::Buffer(
                            surface_buffer.as_entire_buffer_binding(),
                        ),
                    },
                    BindGroupEntry {
                        binding: 7,
                        resource: BindingResource::Buffer(params_buffer.as_entire_buffer_binding()),
                    },
                    BindGroupEntry {
                        binding: 8,
                        resource: BindingResource::Buffer(leaf_buffer.as_entire_buffer_binding()),
                    },
                    BindGroupEntry {
                        binding: 9,
                        resource: BindingResource::Buffer(frames_buffer.as_entire_buffer_binding()),
                    },
                    BindGroupEntry {
                        binding: 10,
                        resource: BindingResource::Buffer(
                            grid_history_buffer.as_entire_buffer_binding(),
                        ),
                    },
                ],
            ));
            classifier.cached_bind_key = Some(key);
        }
        let Some(classifier_bind_group) = classifier.cached_bind_group.clone() else {
            return;
        };
        {
            let mut pass = context
                .command_encoder()
                .begin_compute_pass(&ComputePassDescriptor {
                    label: Some("thessa-cbt-classifier-reset"),
                    timestamp_writes: None,
                });
            pass.set_pipeline(&classifier.reset_pipeline);
            pass.set_bind_group(0, &classifier_bind_group, &[view_uniform_offset.offset]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        {
            let mut pass = context
                .command_encoder()
                .begin_compute_pass(&ComputePassDescriptor {
                    label: Some("thessa-cbt-classifier"),
                    timestamp_writes: None,
                });
            let pass_span = diagnostics
                .as_ref()
                .map(|diagnostics| diagnostics.pass_span(&mut pass, "terrain_classifier"));
            pass.set_pipeline(&classifier.classify_pipeline);
            pass.set_bind_group(0, &classifier_bind_group, &[view_uniform_offset.offset]);
            pass.dispatch_workgroups(gpu.leaf_count(), 1, 1);
            if let Some(pass_span) = pass_span {
                pass_span.end(&mut pass);
            }
        }
        {
            let mut pass = context
                .command_encoder()
                .begin_compute_pass(&ComputePassDescriptor {
                    label: Some("thessa-cbt-classifier-finalize"),
                    timestamp_writes: None,
                });
            pass.set_pipeline(&classifier.finalize_pipeline);
            pass.set_bind_group(0, &classifier_bind_group, &[view_uniform_offset.offset]);
            pass.dispatch_workgroups(1, 1, 1);
        }
        classifier.classified_clip_from_world = Some(clip_from_world);
        classifier.classified_view_generation = surface.view_generation();
        classifier.classified_transform_generation = surface.transform_generation();
        classifier.classified_topology_generation = gpu.topology_generation;
        classifier.classified_pages_generation = gpu.pages_generation;
        classifier.classified_surface_generation = gpu.surface_generation;
    }
    if let Some(diagnostics) = diagnostics.as_ref() {
        diagnostics.record_u32(
            context.command_encoder(),
            &active_count_buffer.slice(0..4),
            "terrain_triangles",
        );
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
    let pass_span = diagnostics
        .as_ref()
        .map(|diagnostics| diagnostics.pass_span(&mut pass, "terrain"));
    if let Some(viewport) = camera.viewport.as_ref() {
        pass.set_camera_viewport(viewport);
    }
    pass.set_render_pipeline(&pipeline);
    pass.set_bind_group(0, &bind_group, &[view_uniform_offset.offset]);
    // Match the reference's linear vertex stream: one instance, three
    // vertices per active triangle, with vertex_index / 3 selecting the record.
    pass.draw_indirect(draw_buffer, 0);
    draw_attempt.submitted = true;
    if let Some(pass_span) = pass_span {
        pass_span.end(&mut pass);
    }
}

#[cfg(feature = "mesh-shaders")]
#[allow(clippy::too_many_arguments)]
fn draw_cbt_mesh_geometry(
    presentation: Res<CbtGpuPresentation>,
    material: Option<Res<CbtRenderMaterial>>,
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
    if !surface.gpu_raster_enabled()
        || !surface.gpu_surface_ready()
        || !surface.gpu_mesh_enabled()
        || gpu.leaf_count() == 0
    {
        return;
    }

    let mut draw_attempt = presentation.attempt(&surface, material.as_deref());
    if !gpu.complete_pages {
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
        Some(frames_buffer),
    ) = (
        gpu.leaf_buffer(),
        gpu.page_metadata_buffer(),
        gpu.page_residual_buffer(),
        gpu.params_buffer(),
        gpu.surface_transform_buffer(),
        gpu.tile_frames.buffer(),
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
                binding: 4,
                resource: BindingResource::Buffer(frames_buffer.as_entire_buffer_binding()),
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
    draw_attempt.submitted = true;
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

    #[test]
    fn classifier_shader_is_portable_wgsl() {
        let module = naga::front::wgsl::parse_str(CBT_CLASSIFY_WGSL)
            .expect("CBT classifier shader must parse");
        naga::valid::Validator::new(Default::default(), Default::default())
            .validate(&module)
            .expect("CBT classifier shader must validate");
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

#[cfg(test)]
#[path = "gpu_tests.rs"]
mod gpu_tests;
