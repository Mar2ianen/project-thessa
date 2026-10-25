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
struct Output { @builtin(position) clip:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) uv:vec2<f32>, @location(2) position:vec3<f32>, @location(3) ambient:f32 }
@vertex fn vertex(@location(0) position:vec3<f32>, @location(1) normal:vec3<f32>, @location(2) uv:vec2<f32>, @location(3) ambient:f32, @builtin(instance_index) i:u32)->Output {
    var out:Output;
    let matrix=instances[i];
    out.clip=view.clip*matrix*vec4(position,1.0);
    out.position=(matrix*vec4(position,1.0)).xyz;
    out.normal=(matrix*vec4(normal,0.0)).xyz;
    out.uv=uv;
    out.ambient=ambient;
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
override MESH_ALL_DOUBLE_SIDED:bool=false;
override MESH_ENV_SAMPLES:u32=16u;
override MESH_CLOUD_DIRECTIONS:u32=4u;
fn mesh_environment(position:vec3<f32>,normal:vec3<f32>,eye:vec3<f32>,f0:vec3<f32>,roughness:f32,ambient:vec3<f32>,occlusion:f32)->vec3<f32> {
    let budget=vec3<u32>(MESH_CLOUD_DIRECTIONS,min(MESH_CLOUD_DIRECTIONS,max(1u,u32(cloud_frame.deck.w))),1u);
    let response=env_specular_response(position,normal,eye,roughness,f0,ambient,MESH_ENV_SAMPLES,budget);
    if(!env_enabled()){return ambient/OCEAN_PI*response.reflectance*occlusion;}
    return response.radiance*occlusion;
}
// A smooth/normal-mapped normal cannot move light across the actual sheet.
fn mesh_diffuse_cosines(normal:vec3<f32>,geometry:vec3<f32>,light_direction:vec3<f32>)->vec2<f32> {
    let shading=dot(normal,light_direction);let side=dot(geometry,light_direction);
    return vec2(select(0.0,max(shading,0.0),side>0.0),select(0.0,max(-shading,0.0),side<0.0));
}
@fragment fn fragment(in:Output,@builtin(front_facing) front:bool)->@location(0) vec4<f32> {
    if (!MESH_ALL_DOUBLE_SIDED && !front && material.double_sided==0u) { discard; }
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
            let cosines=mesh_diffuse_cosines(normal,receiver_normal,light.directions[i].xyz);
            let shadow_normal=select(receiver_normal,-receiver_normal,dot(receiver_normal,light.directions[i].xyz)<0.0);
            visibility[i]=surface_visibility(in.position,shadow_normal,i);
            let fwd=(1.0-transmission)*cosines.x;
            let bwd=transmission*cosines.y;
            direct_reflect+=light.colors[i].rgb*fwd*visibility[i];
            direct_transmit+=light.colors[i].rgb*bwd*visibility[i];
        }
    }
    var ambient=light.ambient.rgb;
    if(material.quality!=0u){ambient=(1.0-transmission)*env_diffuse_irradiance(normal,ambient)+transmission*env_diffuse_irradiance(-normal,ambient);}
    let amb_up=ambient*ao;
    let diffuse_direct=albedo*(direct_reflect+direct_transmit)/OCEAN_PI;
    let diffuse_ambient=albedo*amb_up/OCEAN_PI;
    let diffuse_total=diffuse_ambient+diffuse_direct;
    if (material.quality==0u && material.reflections==0u) {
        let low_rad = env_local_haze(in.position,diffuse_total*(1.0-material.factors.x),light.ambient.rgb);
        return vec4(low_rad*view.exposure,1.0);
    }
    let eye=ocean_safe_normalize(-in.position,normal);
    let metallic=material.factors.x*mr.b;
    var roughness=material.factors.y*mr.g;
    var normal_variance=0.0;
    if(material.quality==2u && material.pad!=0u) {
        // Averaged unit-map normals retain covariance trace 1-|mean(N)|².
        // Split it across the two tangent axes; preserve existing mip storage.
        normal_variance=0.5*max(0.0,1.0-dot(map_normal,map_normal))*material.factors.z*material.factors.z;
    }
    if(material.quality!=0u && material.reflections!=0u){roughness=surface_filtered_roughness_variance(roughness,dpdx(normal),dpdy(normal),normal_variance);}
    let f0=mix(vec3(0.04),albedo,metallic);
    var fresnel=f0;
    if (material.quality!=0u) { fresnel=f0+(vec3(1.0)-f0)*pow(1.0-max(dot(normal,eye),0.0),5.0); }
    var radiance=(vec3(1.0)-fresnel)*(1.0-metallic)*diffuse_total;
    if (material.reflections!=0u) {
        radiance+=rt_reflection(in.position,normal,eye,f0,fresnel,roughness,light.ambient.rgb,ao);
        for (var i=0u;i<3u;i++) {
            if (any(light.colors[i].rgb>vec3(0.0)) && dot(receiver_normal,light.directions[i].xyz)>0.0) {
                radiance+=surface_sun_reflection(normal,eye,light.directions[i].xyz,light.colors[i].rgb,roughness,f0)*visibility[i];
            }
        }
    }
    radiance=env_local_haze(in.position,radiance,light.ambient.rgb);
    return vec4(radiance*view.exposure,1.0);
}
