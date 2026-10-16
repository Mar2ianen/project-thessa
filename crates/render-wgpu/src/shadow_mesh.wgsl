@group(0) @binding(0) var<uniform> shadow_clip:mat4x4<f32>;
@group(1) @binding(0) var<storage,read> shadow_instances:array<mat4x4<f32>>;
@vertex fn vertex(@location(0) position:vec3<f32>,@builtin(instance_index) i:u32)->@builtin(position) vec4<f32> {
    return shadow_clip*shadow_instances[i]*vec4(position,1.0);
}
