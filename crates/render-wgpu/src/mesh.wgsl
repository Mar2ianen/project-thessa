struct View { clip:mat4x4<f32>, unjittered:mat4x4<f32>, inverse_clip:mat4x4<f32>,
    world_from_view:mat4x4<f32>, view_from_world:mat4x4<f32>, projection:mat4x4<f32>, inverse_projection:mat4x4<f32>, camera:vec3<f32>, exposure:f32 }
struct Lighting { ambient:vec4<f32>, directions:array<vec4<f32>,3>, colors:array<vec4<f32>,3> }
struct Material { color:vec4<f32>, double_sided:u32, quality:u32, reflections:u32, pad:u32, factors:vec4<f32> }
@group(0) @binding(0) var<uniform> view:View;
@group(0) @binding(1) var<uniform> light:Lighting;
@group(0) @binding(2) var<storage,read> instances:array<mat4x4<f32>>;
@group(1) @binding(0) var<uniform> material:Material;
@group(1) @binding(1) var color_texture:texture_2d<f32>;
@group(1) @binding(2) var color_sampler:sampler;
@group(1) @binding(3) var mr_texture:texture_2d<f32>;
@group(1) @binding(4) var mr_sampler:sampler;
@group(1) @binding(5) var normal_texture:texture_2d<f32>;
@group(1) @binding(6) var normal_sampler:sampler;
@group(1) @binding(7) var ao_texture:texture_2d<f32>;
@group(1) @binding(8) var ao_sampler:sampler;
struct Output { @builtin(position) clip:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) uv:vec2<f32>, @location(2) position:vec3<f32> }
@vertex fn vertex(@location(0) position:vec3<f32>, @location(1) normal:vec3<f32>, @location(2) uv:vec2<f32>, @builtin(instance_index) i:u32)->Output {
    var out:Output;
    let matrix=instances[i];
    out.clip=view.clip*matrix*vec4(position,1.0);
    out.position=(matrix*vec4(position,1.0)).xyz;
    out.normal=(matrix*vec4(normal,0.0)).xyz;
    out.uv=uv;
    return out;
}
@fragment fn fragment(in:Output,@builtin(front_facing) front:bool)->@location(0) vec4<f32> {
    if (!front && material.double_sided==0u) { discard; }
    var normal=normalize(in.normal)*select(-1.0,1.0,front);
    let dx=dpdx(in.position); let dy=dpdy(in.position);
    let du=dpdx(in.uv); let dv=dpdy(in.uv);
    let map_normal=textureSample(normal_texture,normal_sampler,in.uv).xyz*2.0-1.0;
    let mr=textureSample(mr_texture,mr_sampler,in.uv);
    let ao=mix(1.0,textureSample(ao_texture,ao_sampler,in.uv).r,material.factors.w);
    if (material.quality==2u && material.pad!=0u) {
        // Derivative tangent frame, including mirrored UVs; degenerate UVs
        // retain the geometric normal rather than normalizing a zero vector.
        let determinant=du.x*dv.y-du.y*dv.x;
        if (abs(determinant)>1e-10) {
            let t=(dx*dv.y-dy*du.y)/determinant;
            let b=(dy*du.x-dx*dv.x)/determinant;
            let tangent=ocean_safe_normalize(t-normal*dot(t,normal),vec3(0.0));
            let bitangent=ocean_safe_normalize(cross(normal,tangent),vec3(0.0))*select(-1.0,1.0,dot(cross(normal,tangent),b)>=0.0);
            normal=ocean_safe_normalize(tangent*map_normal.x*material.factors.z+bitangent*map_normal.y*material.factors.z+normal*map_normal.z,normal);
        }
    }
    let albedo=textureSample(color_texture,color_sampler,in.uv).rgb*material.color.rgb;
    var illuminance=light.ambient.rgb;
    for (var i=0u;i<3u;i++) { if (any(light.colors[i].rgb>vec3(0.0))) { illuminance+=light.colors[i].rgb*max(dot(normal,light.directions[i].xyz),0.0)*shadow_visibility(in.position,normal,i); } }
    if (material.quality==0u && material.reflections==0u) { return vec4(albedo*illuminance*(1.0-material.factors.x)/OCEAN_PI*view.exposure,1.0); }
    let eye=ocean_safe_normalize(-in.position,normal);
    let metallic=material.factors.x*mr.b;
    let roughness=material.factors.y*mr.g;
    let f0=mix(vec3(0.04),albedo,metallic);
    // Isotropic incident environment: ambient_lux = PI * radiance. This is
    // the existing explicit environment input, not an invented coloured sky.
    var fresnel=f0;
    // Low retains the reflection phenomenon and metallic energy partition,
    // but approximates the diffuse/ambient Fresnel term by normal-incidence F0.
    if (material.quality!=0u) { fresnel=f0+(vec3(1.0)-f0)*pow(1.0-max(dot(normal,eye),0.0),5.0); }
    var radiance=(vec3(1.0)-fresnel)*(1.0-metallic)*albedo*(illuminance-light.ambient.rgb*(1.0-ao))/OCEAN_PI;
    if (material.reflections!=0u) {
        radiance+=rt_reflection(in.position,normal,eye,f0,fresnel,roughness,light.ambient.rgb,ao);
        for (var i=0u;i<3u;i++) {
            if (any(light.colors[i].rgb>vec3(0.0))) {
                radiance+=surface_sun_reflection(normal,eye,light.directions[i].xyz,light.colors[i].rgb,roughness,f0)*shadow_visibility(in.position,normal,i);
            }
        }
    }
    return vec4(radiance*view.exposure,1.0);
}
