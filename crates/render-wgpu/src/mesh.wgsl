struct View { clip:mat4x4<f32>, unjittered:mat4x4<f32>, inverse_clip:mat4x4<f32>,
    world_from_view:mat4x4<f32>, view_from_world:mat4x4<f32>, projection:mat4x4<f32>, inverse_projection:mat4x4<f32>, camera:vec3<f32>, exposure:f32 }
struct Lighting { ambient:vec4<f32>, directions:array<vec4<f32>,3>, colors:array<vec4<f32>,3> }
struct Material { color:vec4<f32>, double_sided:u32, quality:u32, reflections:u32, pad:u32, factors:vec4<f32>, optical:vec4<f32> }
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
struct Output { @builtin(position) clip:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) uv:vec2<f32>, @location(2) position:vec3<f32>, @location(3) ambient:f32, @location(4) local_y:f32 }
@vertex fn vertex(@location(0) position:vec3<f32>, @location(1) normal:vec3<f32>, @location(2) uv:vec2<f32>, @location(3) ambient:f32, @builtin(instance_index) i:u32)->Output {
    var out:Output;
    let matrix=instances[i];
    out.clip=view.clip*matrix*vec4(position,1.0);
    out.position=(matrix*vec4(position,1.0)).xyz;
    out.normal=(matrix*vec4(normal,0.0)).xyz;
    out.uv=uv;
    out.ambient=ambient;
    out.local_y=position.y;
    return out;
}
// Raster supplies screen derivatives; a ray hit supplies triangle edges and
// UV differences. Both construct the same tangent frame, including mirrored UVs.
fn surface_map_normal(normal:vec3<f32>, map_normal:vec3<f32>, scale:f32,
    dx:vec3<f32>,dy:vec3<f32>,du:vec2<f32>,dv:vec2<f32>)->vec3<f32> {
    let determinant=du.x*dv.y-du.y*dv.x;
    if (abs(determinant)>1e-10) {
        let t=(dx*dv.y-dy*du.y)/determinant;
        let b=(dy*du.x-dx*dv.x)/determinant;
        let tangent=ocean_safe_normalize(t-normal*dot(t,normal),vec3(0.0));
        let bitangent=ocean_safe_normalize(cross(normal,tangent),vec3(0.0))*select(-1.0,1.0,dot(cross(normal,tangent),b)>=0.0);
        return ocean_safe_normalize(tangent*map_normal.x*scale+bitangent*map_normal.y*scale+normal*map_normal.z,normal);
    }
    return normal;
}
@fragment fn fragment(in:Output,@builtin(front_facing) front:bool)->@location(0) vec4<f32> {
    if (!front && material.double_sided==0u) { discard; }
    var normal=normalize(in.normal)*select(-1.0,1.0,front);
    let dx=dpdx(in.position); let dy=dpdy(in.position);
    // Match terrain receivers: PCF depth gradients and normal offsets follow
    // the raster triangle plane, never smooth or normal-mapped shading normals.
    let plane=cross(dx,dy);
    var receiver_normal=normal;
    if (dot(plane,plane)>0.0) {
        receiver_normal=normalize(plane)*select(-1.0,1.0,dot(plane,normal)>=0.0);
    }
    let du=dpdx(in.uv); let dv=dpdy(in.uv);
    let map_normal=textureSample(normal_texture,normal_sampler,in.uv).xyz*2.0-1.0;
    let mr=textureSample(mr_texture,mr_sampler,in.uv);
    let ao=mix(1.0,textureSample(ao_texture,ao_sampler,in.uv).r,material.factors.w)*in.ambient;
    if (material.quality==2u && material.pad!=0u) {
        normal=surface_map_normal(normal,map_normal,material.factors.z,dx,dy,du,dv);
    }
    let albedo=textureSample(color_texture,color_sampler,in.uv).rgb*material.color.rgb;
    let transmission=material.optical.x;
    var direct_reflect=vec3(0.0);
    var direct_transmit=vec3(0.0);
    var visibility:array<f32,3>;
    for (var i=0u;i<3u;i++) {
        if (any(light.colors[i].rgb>vec3(0.0))) {
            let cosine=dot(normal,light.directions[i].xyz);
            let shadow_normal=select(receiver_normal,-receiver_normal,transmission>0.0 && dot(receiver_normal,light.directions[i].xyz)<0.0);
            visibility[i]=surface_visibility(in.position,shadow_normal,i);
            let fwd=(1.0-transmission)*max(cosine,0.0);
            let bwd=transmission*max(-cosine,0.0);
            direct_reflect+=light.colors[i].rgb*fwd*visibility[i];
            direct_transmit+=light.colors[i].rgb*bwd*visibility[i];
        }
    }
    let amb_up=light.ambient.rgb*ao;
    let diffuse_direct=albedo*(direct_reflect+direct_transmit)/OCEAN_PI;
    let diffuse_ambient=albedo*amb_up/OCEAN_PI;
    let diffuse_total=diffuse_ambient+diffuse_direct;
    if (material.quality==0u && material.reflections==0u) {
        var low_rad = diffuse_total*(1.0-material.factors.x);
        let haze_coeff = light.ambient.w;
        if (haze_coeff > 0.0) {
            let view_dist = length(in.position);
            let haze_dist = max(view_dist - 25.0, 0.0);
            if (haze_dist > 0.0) {
                var total_sky = light.ambient.rgb;
                for (var i = 0u; i < 3u; i = i + 1u) {
                    total_sky += light.colors[i].rgb * max(dot(vec3(0.0, 1.0, 0.0), light.directions[i].xyz), 0.0) * 0.22;
                }
                let sky_rad = total_sky / OCEAN_PI;
                let view_dir = ocean_safe_normalize(-in.position, normal);
                let elevation = max(-view_dir.y, 0.0);
                let sky_horizon = mix(
                    sky_rad * vec3(0.55, 0.62, 0.70),
                    sky_rad * vec3(0.35, 0.42, 0.50),
                    smoothstep(0.0, 0.7, elevation)
                );
                low_rad = mix(sky_horizon, low_rad, exp(-haze_coeff * haze_dist));
            }
        }
        return vec4(low_rad*view.exposure,1.0);
    }
    let eye=ocean_safe_normalize(-in.position,normal);
    let metallic=material.factors.x*mr.b;
    let roughness=material.factors.y*mr.g;
    let f0=mix(vec3(0.04),albedo,metallic);
    var fresnel=f0;
    if (material.quality!=0u) { fresnel=f0+(vec3(1.0)-f0)*pow(1.0-max(dot(normal,eye),0.0),5.0); }
    var radiance=(vec3(1.0)-fresnel)*(1.0-metallic)*diffuse_total;
    if (material.reflections!=0u) {
        radiance+=rt_reflection(in.position,normal,eye,f0,fresnel,roughness,light.ambient.rgb,ao);
        for (var i=0u;i<3u;i++) {
            if (any(light.colors[i].rgb>vec3(0.0))) {
                radiance+=surface_sun_reflection(normal,eye,light.directions[i].xyz,light.colors[i].rgb,roughness,f0)*visibility[i];
            }
        }
    }
    let haze_coeff = light.ambient.w;
    if (haze_coeff > 0.0) {
        let view_dist = length(in.position);
        let haze_dist = max(view_dist - 25.0, 0.0);
        if (haze_dist > 0.0) {
            var total_sky = light.ambient.rgb;
            for (var i = 0u; i < 3u; i = i + 1u) {
                total_sky += light.colors[i].rgb * max(dot(vec3(0.0, 1.0, 0.0), light.directions[i].xyz), 0.0) * 0.22;
            }
            let sky_rad = total_sky / OCEAN_PI;
            let elevation = max(-eye.y, 0.0);
            let sky_horizon = mix(
                sky_rad * vec3(0.55, 0.62, 0.70),
                sky_rad * vec3(0.35, 0.42, 0.50),
                smoothstep(0.0, 0.7, elevation)
            );
            radiance = mix(sky_horizon, radiance, exp(-haze_coeff * haze_dist));
        }
    }
    return vec4(radiance*view.exposure,1.0);
}
