//! GPU counterpart of `thessa_render_core::temporal_resolve`.
//!
//! The pass owns no history, depth copy or presentation state. Callers provide
//! the current jittered colour/depth/motion, previous accumulated colour and the
//! previous raster depth. Output RGB is the resolved pre-overlay colour; alpha
//! stores the accumulated frame age. A motion-invalid current pixel stores age
//! zero: it is displayed, but it must not seed neighbour history.

struct ResolveParams {
    current_world_from_clip: mat4x4<f32>,
    previous_clip_from_world: mat4x4<f32>,
    // xyz: current camera-relative origin minus previous origin.
    // w: previous source is valid for this frame.
    camera_offset: vec4<f32>,
    extent: vec2<f32>,
    current_jitter: vec2<f32>,
    previous_jitter: vec2<f32>,
    depth_tolerance: f32,
    color_sigma: f32,
    min_current_weight: f32,
    // 0: resolved unjittered history, 1: raw previous raster history.
    history_jittered: f32,
    max_history_age: f32,
    neighborhood_width: f32,
}

@group(0) @binding(0) var<uniform> params: ResolveParams;
@group(0) @binding(1) var current_color: texture_2d<f32>;
@group(0) @binding(2) var current_depth: texture_depth_2d;
@group(0) @binding(3) var motion: texture_2d<f32>;
@group(0) @binding(4) var history: texture_2d<f32>;
@group(0) @binding(5) var previous_depth: texture_depth_2d;
@group(0) @binding(6) var history_sampler: sampler;
@group(0) @binding(7) var output: texture_storage_2d<rgba16float, write>;

fn rgb_to_ycocg(rgb: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(
        0.25 * rgb.r + 0.5 * rgb.g + 0.25 * rgb.b,
        0.5 * rgb.r - 0.5 * rgb.b,
        -0.25 * rgb.r + 0.5 * rgb.g - 0.25 * rgb.b,
    );
}

fn ycocg_to_rgb(value: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(
        value.x + value.y - value.z,
        value.x + value.z,
        value.x - value.y - value.z,
    );
}

fn inside(uv: vec2<f32>) -> bool {
    return all(uv >= vec2<f32>(0.0)) && all(uv < vec2<f32>(1.0));
}

// Reject the whole bilinear footprint if any tap is not reusable. Checking
// only the nearest age would still mix masked silhouettes into valid pixels.
fn history_age(uv: vec2<f32>) -> u32 {
    let base = vec2<i32>(floor(uv * params.extent - vec2<f32>(0.5)));
    let limit = vec2<i32>(params.extent) - vec2<i32>(1);
    var age = 64.0;
    for (var y = 0; y <= 1; y = y + 1) {
        for (var x = 0; x <= 1; x = x + 1) {
            let p = clamp(base + vec2<i32>(x, y), vec2<i32>(0), limit);
            age = min(age, textureLoad(history, p, 0).a);
        }
    }
    return u32(round(max(age, 0.0)));
}

@compute @workgroup_size(8, 8)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let extent_u = vec2<u32>(params.extent);
    if (id.x >= extent_u.x || id.y >= extent_u.y) {
        return;
    }
    let extent_i = vec2<i32>(extent_u);
    let pixel = vec2<f32>(id.xy) + vec2<f32>(0.5);
    let uv = pixel / params.extent;
    let current = textureLoad(current_color, vec2<i32>(id.xy), 0);
    if (!all(current.rgb == current.rgb)) {
        textureStore(output, id.xy, vec4<f32>(0.0, 0.0, 0.0, 0.0));
        return;
    }
    let current_rgb = max(current.rgb, vec3<f32>(0.0));
    let depth = textureLoad(current_depth, vec2<i32>(id.xy), 0);
    let motion_value = textureLoad(motion, vec2<i32>(id.xy), 0);
    // Explicitly static far background seeds one fresh sample for bilinear
    // leaf-edge footprints, but never reprojects through finite-depth motion.
    // Moving meshes/ocean still write alpha zero and cannot seed any tap.
    let static_surface = current.a >= 1.0 && depth >= 0.0 && depth <= 1.0;
    var resolved = vec4<f32>(current_rgb, select(0.0, 1.0, static_surface));
    if (params.camera_offset.w > 0.5 && static_surface && depth < 1.0 && motion_value.w > 0.5) {
        let current_ndc = vec3<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth);
        let world_h = params.current_world_from_clip * vec4<f32>(current_ndc, 1.0);
        let world = world_h.xyz / world_h.w;
        let previous_h = params.previous_clip_from_world
            * vec4<f32>(world + params.camera_offset.xyz, 1.0);
        let previous_jitter_uv = params.previous_jitter / params.extent;
        let previous_unjittered = uv - params.current_jitter / params.extent + motion_value.xy;
        let previous_raster = previous_unjittered + previous_jitter_uv;
        let color_uv = previous_unjittered
            + select(vec2<f32>(0.0), previous_jitter_uv, params.history_jittered > 0.5);
        if (previous_h.w > 0.0) {
            let previous_ndc = previous_h.xyz / previous_h.w;
            if (previous_ndc.z >= 0.0
                && previous_ndc.z <= 1.0
                && inside(color_uv)
                && inside(previous_raster))
            {
                let previous_pixel = vec2<i32>(previous_raster * params.extent);
                let previous_z = textureLoad(previous_depth, previous_pixel, 0);
                if (abs(previous_z - previous_ndc.z) <= params.depth_tolerance) {
                    let historical = textureSampleLevel(history, history_sampler, color_uv, 0.0);
                    let historical_age = history_age(color_uv);
                    if (historical_age > 0u && all(historical.rgb == historical.rgb)) {
                        var sum = vec3<f32>(0.0);
                        var sum_sq = vec3<f32>(0.0);
                        let lower = select(0, -1, params.neighborhood_width > 2.0);
                        for (var y = lower; y <= 1; y = y + 1) {
                            for (var x = lower; x <= 1; x = x + 1) {
                                let p = clamp(
                                    vec2<i32>(id.xy) + vec2<i32>(x, y),
                                    vec2<i32>(0),
                                    extent_i - vec2<i32>(1),
                                );
                                let value = rgb_to_ycocg(textureLoad(current_color, p, 0).rgb);
                                sum = sum + value;
                                sum_sq = sum_sq + value * value;
                            }
                        }
                        let count = params.neighborhood_width * params.neighborhood_width;
                        let mean = sum / count;
                        let variance = max(sum_sq / count - mean * mean, vec3<f32>(0.0));
                        let spread = sqrt(variance) * params.color_sigma;
                        if (all(mean == mean) && all(spread == spread)) {
                            let clipped = max(
                                ycocg_to_rgb(clamp(
                                    rgb_to_ycocg(historical.rgb),
                                    mean - spread,
                                    mean + spread,
                                )),
                                vec3<f32>(0.0),
                            );
                            let age = min(historical_age, u32(params.max_history_age));
                            let weight = min(
                                f32(age) / (f32(age) + 1.0),
                                1.0 - params.min_current_weight,
                            );
                            if (weight > 0.0) {
                                resolved = vec4<f32>(
                                    current_rgb * (1.0 - weight) + clipped * weight,
                                    f32(min(age + 1u, u32(params.max_history_age))),
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    textureStore(output, id.xy, resolved);
}
