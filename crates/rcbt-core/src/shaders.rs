//! Portable CBT terrain shaders (WGSL text, no backend types).
//!
//! These chunks and assemblies are backend-neutral data: the Bevy
//! bridge, the portable wgpu backend, and the future native backend
//! all consume the same source. Chunk files live in `shaders/`; the
//! three assembled pipelines below mirror the exact concatenation
//! the bridge historically built inline.

/// Geometry ABI mirrored by the fixed-size portable WGSL kernels. Both
/// consumers use these capacities; changing the grid also requires WGSL edits.
pub const GRID_SIZE: usize = 33;
pub const VERTICES_PER_PATCH: usize = GRID_SIZE * GRID_SIZE;
/// Surface cells plus four edge skirts, two triangles per cell/segment.
pub const TRIANGLES_PER_PATCH: usize = (GRID_SIZE - 1) * (GRID_SIZE - 1) * 2 + (GRID_SIZE - 1) * 8;

/// Portable WGSL chunk `precision.wgsl`.
pub const PRECISION_WGSL: &str = include_str!("shaders/precision.wgsl");

/// Portable WGSL chunk `tile_frame.wgsl`.
pub const TILE_FRAME_WGSL: &str = include_str!("shaders/tile_frame.wgsl");

/// Portable WGSL chunk `surface_sample.wgsl`.
pub const SURFACE_SAMPLE_WGSL: &str = include_str!("shaders/surface_sample.wgsl");

/// Portable WGSL chunk `material_sample.wgsl`.
pub const PAGE_DIRECTORY_WGSL: &str = include_str!("shaders/page_directory.wgsl");
pub const MATERIAL_SAMPLE_WGSL: &str = concat!(
    include_str!("shaders/page_directory.wgsl"),
    include_str!("shaders/material_sample.wgsl")
);

/// Portable WGSL chunk `ocean.wgsl`.
pub const OCEAN_WGSL: &str = include_str!("shaders/ocean.wgsl");

/// Portable WGSL chunk `mesh.wgsl`.
pub const MESH_WGSL: &str = include_str!("shaders/mesh.wgsl");

/// One invocation writes one vertex. The page is sampled in its original
/// quantized representation; no CPU-side height expansion or dense CBT
/// reconstruction is performed.
pub const CBT_GEOMETRY_WGSL: &str = concat!(
    include_str!("shaders/precision.wgsl"),
    include_str!("shaders/tile_frame.wgsl"),
    include_str!("shaders/surface_sample.wgsl"),
    r#"
struct Params {
    leaf_count: u32,
    vertices_per_patch: u32,
    radius_bits: u32,
    dirty_count: u32,
};

@group(0) @binding(0) var<storage, read> leaves: array<vec4<u32>>;
@group(0) @binding(1) var<storage, read_write> patches: array<vec4<u32>>;
@group(0) @binding(2) var<storage, read> page_metadata: array<vec4<u32>>;
@group(0) @binding(3) var<storage, read> page_residuals: array<u32>;
@group(0) @binding(4) var<storage, read_write> vertices: array<vec4<f32>>;
@group(0) @binding(5) var<uniform> params: Params;
@group(0) @binding(6) var<storage, read> tile_frames: array<CbtTileFrame>;
@group(0) @binding(7) var<storage, read> dirty_ordinals: array<u32>;

@compute @workgroup_size(64)
fn build_geometry(
    @builtin(global_invocation_id) gid: vec3<u32>,
    @builtin(num_workgroups) num_wg: vec3<u32>,
) {
    // Wide dispatches fan out over y (see `dispatch_extent` on the Rust
    // side); the workgroup grid is x-major, so linearize across it.
    let index = gid.y * (num_wg.x * 64u) + gid.x;
    let total = params.dirty_count * params.vertices_per_patch;
    if (index >= total) {
        return;
    }
    // Dirty-stream index -> leaf ordinal -> ordinal-indexed vertex slot.
    // The vertex/patch/metadata buffers stay ordinal-indexed; only the
    // dispatch iterates the dirty prefix.
    let ordinal = dirty_ordinals[index / params.vertices_per_patch];
    let local = index % params.vertices_per_patch;
    let vbase = ordinal * params.vertices_per_patch + local;
    let page = page_metadata[ordinal];
    if (local == 0u) {
        patches[ordinal] = leaves[ordinal];
    }
    if (page.z == 0u) {
        vertices[vbase * 2u] = vec4(0.0);
        vertices[vbase * 2u + 1u] = vec4(0.0);
        return;
    }
    let gx = local % 33u;
    let gy = local / 33u;
    let uv = vec2(f32(gx) / 32.0, f32(gy) / 32.0);
    let sample = cbt_surface_sample(tile_frames[ordinal].geometry, page, uv);
    vertices[vbase * 2u] = vec4(sample.local_position, 1.0);
    vertices[vbase * 2u + 1u] = vec4(sample.normal, sample.height);
}
"#
);

/// Portable GPU leaf classifier and active-triangle builder. It uses five
/// generated patch samples to reject leaves outside the current clip volume,
/// selects a screen-space grid step, and compacts the resulting triangle
/// records into the same indirect draw stream used by the raster consumer.
/// The topology remains authoritative on the CPU; visibility, triangle LOD,
/// and draw count are GPU-owned and rerun with the current camera.
pub const CBT_CLASSIFY_WGSL: &str = concat!(
    include_str!("shaders/precision.wgsl"),
    include_str!("shaders/tile_frame.wgsl"),
    r#"
override cbt_displacement_bound_m: f32 = 0.0;
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
    mesh_x: u32,
    mesh_y: u32,
    mesh_z: u32,
    _padding: u32,
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
    // The default specialization is exactly the legacy zero margin. Native
    // spectral displacement supplies a coefficient-derived absolute envelope;
    // two sampled points can differ by twice that envelope.
    let rows=transpose(view.clip_from_world*render_from_body);
    let bound=select(0.0,cbt_displacement_bound_m,grid_history[ordinal].w!=0u);
    let margin=vec4(length(rows[0].xyz),length(rows[1].xyz),length(rows[2].xyz),length(rows[3].xyz))*2.0*bound;
    for (var corner = 0u; corner < 5u; corner = corner + 1u) {
        let position = clip_position(ordinal, corner);
        all_left = all_left && position.x < -position.w-margin.x-margin.w;
        all_right = all_right && position.x > position.w+margin.x+margin.w;
        all_bottom = all_bottom && position.y < -position.w-margin.y-margin.w;
        all_top = all_top && position.y > position.w+margin.y+margin.w;
        all_near = all_near && position.z < -margin.z;
        all_far = all_far && position.z > position.w+margin.z+margin.w;
        all_behind = all_behind && position.w <= -margin.w;
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
    // Native near-ocean displacement is filtered for this full-resolution
    // grid; decimating it afterwards would spatially alias the wave geometry.
    if(grid_history[ordinal].w!=0u){return 1u;}
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
        draw_list[0] = DrawCommand(0u, 1u, 0u, 0u, 0u, 1u, 1u, 0u);
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
    @builtin(num_workgroups) num_wg: vec3<u32>,
    @builtin(local_invocation_index) lane: u32,
) {
    // Wide dispatches fan out over y (see `dispatch_extent` on the Rust
    // side): linearize the x-major workgroup grid back into one ordinal.
    let ordinal = group.y * num_wg.x + group.x;
    if (lane == 0u) {
        selected_count = 0u;
        if (ordinal < params.leaf_count && page_metadata[ordinal].z != 0u) {
            if (intersects_clip(ordinal)) {
                selected_step = screen_grid_step(ordinal);
                grid_history[ordinal] = vec4(
                    leaves[ordinal].x,
                    leaves[ordinal].y,
                    selected_step,
                    grid_history[ordinal].w,
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
        let triangles = atomicLoad(&active_count[0]);
        let groups = (triangles + 31u) / 32u;
        let width = min(groups, 256u);
        draw_list[0] = DrawCommand(
            triangles * 3u,
            1u,
            0u,
            0u,
            width,
            max((groups + 255u) / 256u, 1u),
            1u,
            0u,
        );
    }
}
"#
);

/// Named shader hook block; adapters can substitute their surface visibility
/// and reflection policy without rewriting shared shading expressions.
pub const RASTER_SURFACE_HOOKS_WGSL: &str = include_str!("shaders/raster_surface_hooks.wgsl");

/// Procedural direct raster consumer for the generated GPU geometry. It uses
/// one indirect draw with a linear triangle vertex stream, matching the draw
/// submission shape of large_cbt while retaining the current page format.
pub const CBT_RASTER_WGSL: &str = concat!(
    include_str!("shaders/raster_surface_hooks.wgsl"),
    include_str!("shaders/precision.wgsl"),
    include_str!("shaders/tile_frame.wgsl"),
    include_str!("shaders/ocean.wgsl"),
    include_str!("shaders/page_directory.wgsl"),
    include_str!("shaders/material_sample.wgsl"),
    include_str!("shaders/material_normal.wgsl"),
    r#"
const MATERIAL_MAX_LEVEL: f32 = 17.0;
const MATERIAL_ANISOTROPY: f32 = 1.0;
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
    // Cloud-deck shadow sampler: bake-local rotation, sun in planet frame,
    // (deck height m, strength, enabled, -).
    shadow_quat: vec4<f32>,
    shadow_sun: vec4<f32>,
    shadow_params: vec4<f32>,
    // Night city lights: (intensity, enabled, -, -).
    city_params: vec4<f32>,
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
    @location(6) material_chart: vec3<f32>,
    @location(7) @interpolate(flat) material_anchor: vec2<f32>,
    @location(8) @interpolate(flat) material_face: u32,
};

@group(0) @binding(6) var albedo_texture: texture_2d<f32>;
@group(0) @binding(7) var surface_sampler: sampler;
@group(0) @binding(10) var roughness_texture: texture_2d<f32>;
@group(0) @binding(12) var material_texture: texture_2d_array<f32>;
@group(0) @binding(13) var material_sampler: sampler;
@group(0) @binding(14) var<storage, read> material_slots: array<vec4<u32>>;
@group(0) @binding(17) var cloud_texture: texture_2d<f32>;
@group(0) @binding(18) var city_texture: texture_2d<f32>;


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

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// Inverse of the Bevy Sphere mesh mapping (poles on local +Z, u from +X
// toward +Y, v = 0 at the +Z pole): bake-texel coordinates for a shell-local
// direction. Mirrors the CPU sampler in the client; keep the two in sync.
fn shell_mesh_uv(local_dir: vec3<f32>) -> vec2<f32> {
    let u = fract(atan2(local_dir.y, local_dir.x) / 6.2831853);
    let v = 0.5 - asin(clamp(local_dir.z, -1.0, 1.0)) / 3.14159265;
    return vec2(u, clamp(v, 0.0, 1.0));
}

// Native may select the canonical 48-byte stream. The default preserves the
// generated-patch ABI and all existing Bevy/indexed/mesh consumers.
override CBT_CONFORMING: bool = false;
@vertex
fn vertex(
    @builtin(vertex_index) vertex_index: u32,
) -> VertexOutput {
    if (CBT_CONFORMING) {
        let position = generated_vertices[vertex_index * 3u];
        let normal = generated_vertices[vertex_index * 3u + 1u];
        let address = bitcast<vec4<u32>>(generated_vertices[vertex_index * 3u + 2u]);
        var output: VertexOutput;
        output.clip_position = vec4(2.0,2.0,2.0,1.0);
        // Free/virtual slots have zero normal. Missing height pages instead
        // intentionally retain canonical datum geometry and fallback material.
        if (all(normal.xyz == vec3(0.0))) { return output; }
        output.clip_position = view.clip_from_world * vec4(position.xyz,1.0);
        output.normal = normal.xyz;
        output.body_direction = material_direction(vec2<f32>(address.xy)/536870912.0,u32(normal.w));
        output.render_position = position.xyz;
        output.height_m = position.w;
        output.material_slot = 0xffffffffu;
        return output;
    }
    return cbt_vertex(vertex_index);
}

// Both draw backends use the exact same geometry, skirt and material IO.
fn cbt_vertex(vertex_index: u32) -> VertexOutput {
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
    let chart = material_address(frame.geometry.raw_center.xyz);
    output.material_anchor = chart.uv;
    output.material_face = chart.face;
    // Rational interpolation reproduces projection of the interpolated body
    // direction, rather than linearly interpolating spherical chart UVs. The
    // anchor is identical for every vertex of this patch and exactly dyadic.
    let major = max(abs(body_direction.x), max(abs(body_direction.y), abs(body_direction.z)));
    output.material_chart = vec3((tile_uv - 0.5) * frame.geometry.radius_half_extent_len.y * major, major);
    let page_uv = tile_uv * bitcast<f32>(material.y) + bitcast<vec2<f32>>(material.zw);
    output.material_uv = (page_uv * 125.0 + 1.5) / 128.0;
    output.material_slot = material.x;
    return output;
}

@fragment
fn fragment(input: VertexOutput) -> @location(0) vec4<f32> {
    let normal = normalize(input.normal);
    // Shadow filtering follows the raster triangle's plane, not interpolated
    // shading normals (which may differ substantially on coarse bisectors).
    let plane = cross(dpdx(input.render_position), dpdy(input.render_position));
    var receiver_normal = normal;
    if (dot(plane,plane) > 0.0) {
        receiver_normal = normalize(plane) * select(-1.0,1.0,dot(plane,normal)>=0.0);
    }
    let direction = normalize(input.body_direction);
    // Project after interpolation so a triangle crossing longitude zero
    // cannot interpolate through the opposite side of the planet's map.
    let uv = surface_uv(input.body_direction);
    var dx = dpdx(uv);
    var dy = dpdy(uv);
    dx.x -= round(dx.x);
    dy.x -= round(dy.x);
    // Keep local pages until the globe map actually resolves a screen pixel.
    // The old 16-32 m transition magnified kilometre-scale globe texels and
    // replaced a detailed coast with stair-stepped blurry blocks in one view.
    // World-addressed page LOD/mips now filter the intervening scales.
    let footprint = max(length(dpdx(direction)), length(dpdy(direction)));
    var footprint_m = footprint * bitcast<f32>(params.radius_bits);
    var material_texel_m = footprint_m;
    let globe_texel_m = 3.14159265 * bitcast<f32>(params.radius_bits)
        / f32(textureDimensions(albedo_texture).y);
    var address = material_address(direction);
    // A zero divisor is the conforming stream's explicit legacy fallback.
    let chart_offset = input.material_chart.xy / max(input.material_chart.z, 0.000001);
    let chart_dx = dpdx(chart_offset);
    let chart_dy = dpdy(chart_offset);
    if (input.material_chart.z > 0.0) {
        address = MaterialAddress(input.material_anchor, input.material_face, chart_offset);
        // Evaluate normalized cube-chart chords analytically. Differencing
        // rounded absolute directions loses centimetre-scale footprints at a
        // planetary radius, even when the page address itself is precise.
        footprint_m = material_chart_footprint(address, chart_dx, chart_dy, bitcast<f32>(params.radius_bits));
        material_texel_m = material_chart_filter_footprint(address, chart_dx, chart_dy, bitcast<f32>(params.radius_bits), MATERIAL_ANISOTROPY);
    }
    // Project both derivatives in this fragment's face. A derivative across
    // a face-number switch must not become an enormous false mip footprint.
    let address_dx = select(material_face_uv(direction + dpdx(direction), address.face) - address.uv,
        chart_dx, input.material_chart.z > 0.0);
    let address_dy = select(material_face_uv(direction + dpdy(direction), address.face) - address.uv,
        chart_dy, input.material_chart.z > 0.0);
    var detail_weight = 1.0 - smoothstep(globe_texel_m, globe_texel_m * 2.0, footprint_m);
    // A material texel must cover the same physical footprint regardless of
    // which geometry leaf happens to rasterize it. Trilinear page levels are
    // chosen in screen space rather than inheriting a coarse mesh page.
    let material_lod = clamp(log2(2.0 * bitcast<f32>(params.radius_bits)
        / (125.0 * max(material_texel_m, 0.001))), 0.0, MATERIAL_MAX_LEVEL);
    let coarse = material_at_level_raster(address, address_dx, address_dy, u32(floor(material_lod)));
    let fine = material_at_level_raster(address, address_dx, address_dy, u32(ceil(material_lod)));
    let coverage = mix(coarse.coverage, fine.coverage, fract(material_lod));
    let detail = mix(coarse.value, fine.value, fract(material_lod)) / max(coverage, 0.00001);
    detail_weight *= coverage;
    var albedo = vec3(0.0);
    var roughness = 0.0;
    if (detail_weight > 0.0) {
        albedo = detail.rgb * detail_weight;
        roughness = detail.a * detail_weight;
    }
    if (detail_weight < 1.0) {
        albedo += textureSampleGrad(albedo_texture, surface_sampler, uv, dx, dy).rgb
            * (1.0 - detail_weight);
        roughness += textureSampleGrad(roughness_texture, surface_sampler, uv, dx, dy).g
            * (1.0 - detail_weight);
    }
    // Classify before material-normal perturbation: water owns its own wave
    // normal, even where filtering includes a normal-bearing land neighbour.
    let water_material = (1.0 - smoothstep(0.22, 0.65, roughness))
        * (1.0 - smoothstep(0.25, 1.0, input.height_m));
    // Premultiplied surface gradients: uncovered weight is flat. Preserve
    // coverage and the same globe transition as albedo rather than magnifying
    // a tiny resident normal contribution to full strength.
    let tangent = mix(coarse.normal, fine.normal, fract(material_lod))
        * (1.0 - smoothstep(globe_texel_m, globe_texel_m * 2.0, footprint_m));
    let shaded_normal = material_perturb_normal(normal, address, tangent * (1.0 - water_material), render_from_body);
    var direct = vec3(0.0);
    var land_direct = vec3(0.0);
    for (var i = 0u; i < 3u; i = i + 1u) {
        if (any(lighting.colors_lux[i].rgb > vec3(0.0))) {
            let incoming = lighting.colors_lux[i].rgb * max(dot(shaded_normal, lighting.directions[i].xyz), 0.0)
                * cbt_shadow_visibility(input.render_position, receiver_normal, i);
            direct += incoming;
            if (cbt_specular_enabled() && water_material < 1.0 && any(incoming > vec3(0.0))) {
                land_direct += incoming * cbt_land_transmission(shaded_normal,lighting.directions[i].xyz,roughness);
            }
        }
    }
    // Cloud-deck shadows dim the direct sun only; skylight and ambient stay.
    // The deck bake is what the shell impostors show, sampled in shell-local
    // coordinates with a sun-parallax offset for the physical deck height.
    var shadow = 1.0;
    if (lighting.shadow_params.z > 0.5) {
        let sun_body = lighting.shadow_sun.xyz;
        let deck_h = lighting.shadow_params.x;
        let radius = bitcast<f32>(params.radius_bits);
        // Walk from the ground toward the sun to the deck plane (mirrors the
        // CPU sampler): overhead sun samples straight down.
        let t = deck_h / max(dot(sun_body, direction), 0.15);
        let deck_point = normalize(direction * radius + sun_body * t);
        let local = quat_rotate(lighting.shadow_quat, deck_point);
        let cloud_alpha = textureSample(cloud_texture, surface_sampler, shell_mesh_uv(local)).a;
        shadow = 1.0 - lighting.shadow_params.y * smoothstep(0.05, 0.6, cloud_alpha);
    }
    var illuminance = lighting.ambient_lux.rgb + direct * shadow;
    let diffuse = albedo * illuminance / OCEAN_PI;
    let water = water_material * lighting.ocean_appearance.z;
    // Land roughness is a material property, not only an ocean classifier.
    // Reuse the existing dielectric GGX kernel and linear-lux convention;
    // this adds neither a lighting pipeline nor a hidden geometry prepass.
    let eye = ocean_safe_normalize(view.world_position - input.render_position, normal);
    var radiance = diffuse;
    if (cbt_specular_enabled() && water_material < 1.0) {
        var land_reflection = vec3(0.0);
        for (var i = 0u; i < 3u; i = i + 1u) {
            if (any(lighting.colors_lux[i].rgb > vec3(0.0))) {
                land_reflection += ocean_sun_reflection(shaded_normal, eye, lighting.directions[i].xyz,
                    lighting.colors_lux[i].rgb, roughness, 0.04)
                    * cbt_shadow_visibility(input.render_position, receiver_normal, i);
            }
        }
        let environment = cbt_land_environment(input.render_position,shaded_normal,eye,roughness,lighting.ambient_lux.rgb);
        land_reflection += environment.rgb;
        var land_ambient = lighting.ambient_lux.rgb;
        if (any(land_ambient > vec3(0.0))) {
            land_ambient *= cbt_land_ambient_transmission(roughness);
        }
        let land_diffuse = albedo * (land_ambient + land_direct * shadow) / OCEAN_PI;
        radiance = mix(diffuse, land_diffuse * (1.0 - environment.a) + land_reflection,
            1.0 - water_material);
    }
    if (cbt_specular_enabled() && water > 0.0) {
        let rotation = mat3x3<f32>(render_from_body[0].xyz, render_from_body[1].xyz, render_from_body[2].xyz);
        let up = normalize(rotation * direction);
        let wave_normal = ocean_wave_normal(direction, normal, rotation,
            lighting.ocean_phase.xy, footprint, lighting.ocean_wave);
        let f0 = lighting.ocean_appearance.x;
        let fresnel = ocean_schlick(f0, max(dot(wave_normal, eye), 0.0));
        var reflection = vec3(0.0);
        var sky_illuminance = lighting.ambient_lux.rgb;
        for (var i = 0u; i < 3u; i = i + 1u) {
            let light = lighting.directions[i].xyz;
            reflection += ocean_sun_reflection(wave_normal, eye, light,
                lighting.colors_lux[i].rgb, max(roughness, 0.16), f0)
                * cbt_shadow_visibility(input.render_position, normal, i);
            sky_illuminance += lighting.colors_lux[i].rgb
                * max(dot(up, light), 0.0) * lighting.ocean_appearance.y;
        }
        let sky = sky_illuminance / OCEAN_PI;
        let reflected = reflect(-eye, wave_normal);
        reflection += ocean_sky_radiance(reflected, up,
            sky * vec3(0.10, 0.19, 0.38), sky * vec3(0.42, 0.52, 0.60),
            sky * vec3(0.015, 0.018, 0.015)) * fresnel;
        radiance = mix(radiance, diffuse * (1.0 - fresnel) + reflection, water);
    }
    let spectral_ocean = cbt_spectral_ocean(input.render_position,receiver_normal,direction,input.height_m,roughness,albedo,footprint_m);
    radiance = mix(radiance,spectral_ocean.rgb,spectral_ocean.a);
    // Night city lights: warm emissive mask over inhabited land, gated by
    // the terminator. Day diffuse outshines it by orders of magnitude, so no
    // day-side masking is needed.
    if (lighting.city_params.y > 0.5) {
        let sun_dir = lighting.directions[0].xyz;
        let night = 1.0 - smoothstep(-0.15, 0.12, dot(normal, sun_dir));
        if (night > 0.001) {
            let mask = textureSample(city_texture, surface_sampler, uv).r;
            // Warm sodium/LED street mix (1.0, 0.72, 0.42) at a pre-exposure
            // gain of 500.0 for mask = 1: night terrain sits near O(100) lux
            // under the pilot ambient, day diffuse is O(10^4), so this reads
            // clearly after dark and still vanishes by day. Measured against
            // EV100 13 screenshots (gain 60 was 7 stops under and invisible).
            radiance += vec3(1.0, 0.72, 0.42) * (mask * night * lighting.city_params.x * 500.0);
        }
    }
    // Same pre-exposed linear HDR convention as Bevy PBR.
    return vec4(radiance * view.exposure, 1.0);
}
"#
);
