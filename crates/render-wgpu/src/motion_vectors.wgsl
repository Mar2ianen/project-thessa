struct MotionParams {
    current_world_from_clip: mat4x4<f32>,
    previous_clip_from_world: mat4x4<f32>,
    camera_offset: vec4<f32>, // xyz: origin delta, w: current jitter x in pixels
    extent: vec4<f32>, // xy: target extent, z: previous valid, w: jitter y in pixels
};

@group(0) @binding(0) var<uniform> params: MotionParams;
@group(0) @binding(1) var current_depth: texture_depth_2d;
@group(0) @binding(2) var scene_color: texture_2d<f32>;
@group(0) @binding(3) var motion_out: texture_storage_2d<rgba16float, write>;

@compute @workgroup_size(8, 8)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let extent = vec2<u32>(params.extent.xy);
    if (id.x >= extent.x || id.y >= extent.y) {
        return;
    }

    let pixel = vec2<f32>(id.xy) + vec2<f32>(0.5);
    let uv = pixel / vec2<f32>(extent);
    let depth = textureLoad(current_depth, vec2<i32>(id.xy), 0);
    // extent.z marks a committed previous rendered frame. Scene alpha is an
    // explicit mask, never an inference from depth or a legitimate zero vector.
    var valid = params.extent.z > 0.5 && depth >= 0.0 && depth < 1.0
        && textureLoad(scene_color, vec2<i32>(id.xy), 0).a >= 1.0;
    var motion = vec2<f32>(0.0);

    if (valid) {
        let ndc = vec3<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth);
        let current = params.current_world_from_clip * vec4<f32>(ndc, 1.0);
        if (abs(current.w) <= 1e-12) {
            valid = false;
        } else {
            let world = current.xyz / current.w + params.camera_offset.xyz;
            let previous = params.previous_clip_from_world * vec4<f32>(world, 1.0);
            if (previous.w <= 0.0) {
                valid = false;
            } else {
                let previous_uv = vec2<f32>(
                    previous.x / previous.w * 0.5 + 0.5,
                    0.5 - previous.y / previous.w * 0.5,
                );
                // Reconstruction used the matching jittered inverse. The
                // velocity contract excludes sampling jitter, not camera
                // motion: current unjittered UV = raster UV - jitter / extent.
                let current_jitter = vec2<f32>(params.camera_offset.w, params.extent.w);
                motion = previous_uv - (uv - current_jitter / params.extent.xy);
                // A NaN comparison is false; reject it before publishing a
                // vector that would poison history sampling.
                valid = all(abs(motion) <= vec2<f32>(65504.0));
            }
        }
    }

    if (valid) {
        textureStore(motion_out, vec2<i32>(id.xy), vec4<f32>(motion, 0.0, 1.0));
    } else {
        textureStore(motion_out, vec2<i32>(id.xy), vec4<f32>(0.0, 0.0, 0.0, 0.0));
    }
}
