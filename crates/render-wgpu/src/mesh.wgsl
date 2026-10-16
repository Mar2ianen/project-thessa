struct View { clip:mat4x4<f32>, unjittered:mat4x4<f32>, inverse_clip:mat4x4<f32>,
    world_from_view:mat4x4<f32>, view_from_world:mat4x4<f32>, projection:mat4x4<f32>, inverse_projection:mat4x4<f32>, camera:vec3<f32>, exposure:f32 }
struct Lighting { ambient:vec4<f32>, directions:array<vec4<f32>,3>, colors:array<vec4<f32>,3> }
struct Material { color:vec4<f32>, double_sided:u32, pad0:u32, pad1:u32, pad2:u32 }
@group(0) @binding(0) var<uniform> view:View;
@group(0) @binding(1) var<uniform> light:Lighting;
@group(0) @binding(2) var<storage,read> instances:array<mat4x4<f32>>;
@group(1) @binding(0) var<uniform> material:Material;
@group(1) @binding(1) var color_texture:texture_2d<f32>;
@group(1) @binding(2) var color_sampler:sampler;
struct Output { @builtin(position) clip:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) uv:vec2<f32> }
@vertex fn vertex(@location(0) position:vec3<f32>, @location(1) normal:vec3<f32>, @location(2) uv:vec2<f32>, @builtin(instance_index) i:u32)->Output {
    var out:Output;
    let matrix=instances[i];
    out.clip=view.clip*matrix*vec4(position,1.0);
    out.normal=(matrix*vec4(normal,0.0)).xyz;
    out.uv=uv;
    return out;
}
@fragment fn fragment(in:Output,@builtin(front_facing) front:bool)->@location(0) vec4<f32> {
    if (!front && material.double_sided==0u) { discard; }
    let normal=normalize(in.normal)*select(-1.0,1.0,front);
    let albedo=textureSample(color_texture,color_sampler,in.uv).rgb*material.color.rgb;
    var illuminance=light.ambient.rgb;
    for (var i=0u;i<3u;i++) { illuminance+=light.colors[i].rgb*max(dot(normal,light.directions[i].xyz),0.0); }
    return vec4(albedo*illuminance/3.141592653589793*view.exposure,1.0);
}
