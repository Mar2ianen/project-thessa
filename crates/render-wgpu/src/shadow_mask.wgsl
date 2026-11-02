@group(0) @binding(0) var<uniform> shadow_clip:mat4x4<f32>;
@group(1) @binding(0) var<storage,read> shadow_instances:array<mat4x4<f32>>;
@group(2) @binding(0) var<uniform> material:Material;
@group(2) @binding(1) var color_texture:texture_2d<f32>;
@group(2) @binding(2) var color_sampler:sampler;
struct MaskVertex { @builtin(position) clip:vec4<f32>, @location(0) uv:vec2<f32> }
@vertex fn vertex(@location(0) position:vec3<f32>,@location(2) uv:vec2<f32>,@builtin(instance_index) i:u32)->MaskVertex {
    return MaskVertex(shadow_clip*shadow_instances[i]*vec4(position,1.0),uv);
}
@fragment fn fragment(in:MaskVertex) {
    if(textureSample(color_texture,color_sampler,in.uv).a*material.color.a<material.optical.z){discard;}
}
