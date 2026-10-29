// Native ray-query scene consumer for opaque object reflections.
//
// Included (first, so the `enable` directive precedes declarations) only when
// ray tracing resolved on, the negotiated device carries ray query, and
// `ray_scene::plan` accepted the scene against the RT memory budget.
// `rt_reflection` is the named hook `mesh.wgsl` calls; the raster build of the
// same shader substitutes an exact environment-only fallback, so the hook has
// one call site and two implementations.
//
// Contract: the returned RGB is the reflected radiance leaving the surface.
// A ray that hits nothing contributes `env * fresnel`, identical to the
// environment term it replaces, so RT never invents sky or ambient light.
// Hits use barycentric smooth normals/UVs, shared raster textures/samplers,
// sRGB base colour and linear MR/normal/AO. Medium/High carry a pixel cone;
// Low retains the point-sampled material fallback.

struct RtMaterial {
    // rgb = linear base color, w = metallic.
    color: vec4<f32>,
    // x = roughness, y = normal scale, z = AO strength, w = diffuse transmission.
    factors: vec4<f32>,
    // Base colour, metallic/roughness, normal, AO; UINT_MAX means absent.
    textures: vec4<u32>,
    optical: vec4<f32>,
}

struct RtVertex { position:vec4<f32>, normal:vec4<f32>, uv:vec4<f32> }

struct RtRange {
    // x = first index into rt_indices, y = first vertex into rt_positions,
    // z = index count of the primitive, w unused.
    words: vec4<u32>,
}

struct RtPair {
    // x = primitive (range and material record), yzw unused.
    words: vec4<u32>,
    // Rows of the instance's camera-relative 3x4 transform, in the same
    // order the renderer builds them.
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
}

@group(3) @binding(1) var<storage, read> rt_materials: array<RtMaterial>;
@group(3) @binding(2) var<storage, read> rt_ranges: array<RtRange>;
@group(3) @binding(3) var<storage, read> rt_vertices: array<RtVertex>;
@group(3) @binding(4) var<storage, read> rt_indices: array<u32>;
@group(3) @binding(5) var<storage, read> rt_pairs: array<RtPair>;

struct RtSurface {
    position: vec3<f32>,
    normal: vec3<f32>,
    geometry: vec3<f32>,
    albedo: vec3<f32>,
    metallic: f32,
    roughness: f32,
    occlusion: f32,
    dielectric:f32,
    transmission:f32,
    emission:vec3<f32>,
}

// Barycentrics land in the primitive's own asset-local space; the pair rows
// move the point and its normal into the camera-relative space the primary
// surface, the lights and the shadow atlas live in.
fn rt_vector(pair:RtPair, vector:vec3<f32>)->vec3<f32> {
    return vec3(dot(pair.row0.xyz,vector),dot(pair.row1.xyz,vector),dot(pair.row2.xyz,vector));
}

// Project a full pixel-width cone onto the actual hit triangle, including
// grazing incidence. No screen derivatives are evaluated inside ray traversal.
fn rt_hit_footprint(direction:vec3<f32>,geometry:vec3<f32>,width:f32)->mat2x3<f32> {
    let axis=select(vec3(0.0,1.0,0.0),vec3(1.0,0.0,0.0),abs(direction.y)>0.9);
    let x=normalize(cross(direction,axis));
    let y=cross(direction,x);
    let cosine=dot(direction,geometry);
    // shortcut: cap the tangent projection at 10,000x at grazing silhouettes;
    // exact visibility/curved-surface filtering needs additional ray samples.
    let denominator=select(-1.0,1.0,cosine>=0.0)*max(abs(cosine),0.0001);
    return mat2x3<f32>((x-direction*(dot(x,geometry)/denominator))*width,
        (y-direction*(dot(y,geometry)/denominator))*width);
}

fn rt_uv_delta(delta:vec3<f32>,e1:vec3<f32>,e2:vec3<f32>,u1:vec2<f32>,u2:vec2<f32>)->vec2<f32> {
    // Scale before forming area squared: centimetre blades and large instances
    // must not underflow/overflow the same UV Jacobian.
    let scale=max(length(e1),length(e2));
    if(scale<=0.0){return vec2(0.0);}
    let a=e1/scale;let b=e2/scale;let d=delta/scale;
    let n=cross(a,b);let area=dot(n,n);
    if(area<=1e-20){return vec2(0.0);}
    return (u1*dot(cross(d,b),n)+u2*dot(cross(a,d),n))/area;
}

fn rt_surface(hit: RtHit, ray_direction: vec3<f32>,cone_width:f32) -> RtSurface {
    let pair = rt_pairs[hit.pair];
    // TERRAIN_HIT_HOOK
    let range = rt_ranges[pair.words.x];
    let first = range.words.x + hit.triangle * 3u;
    let base = range.words.y;
    let v0 = rt_vertices[base + rt_indices[first]];
    let v1 = rt_vertices[base + rt_indices[first + 1u]];
    let v2 = rt_vertices[base + rt_indices[first + 2u]];
    let p0 = v0.position.xyz;
    let p1 = v1.position.xyz;
    let p2 = v2.position.xyz;
    let local = p0 + (p1 - p0) * hit.barycentrics.x + (p2 - p0) * hit.barycentrics.y;
    let local_normal = normalize(cross(p1 - p0, p2 - p0));
    let position = vec3(
        dot(pair.row0.xyz, local) + pair.row0.w,
        dot(pair.row1.xyz, local) + pair.row1.w,
        dot(pair.row2.xyz, local) + pair.row2.w,
    );
    let normal = normalize(rt_vector(pair,local_normal));
    // Geometric normals are two-sided: face the incoming ray so single-sided
    // assets still shade the reflection instead of lighting a back face.
    let geometry = select(normal, -normal, dot(normal, ray_direction) > 0.0);
    let material = rt_materials[pair.words.x];
    let bary = vec3(1.0-hit.barycentrics.x-hit.barycentrics.y,hit.barycentrics);
    let uv = v0.uv.xy*bary.x + v1.uv.xy*bary.y + v2.uv.xy*bary.z;
    let footprint=rt_hit_footprint(ray_direction,geometry,cone_width);
    let e1=rt_vector(pair,p1-p0);let e2=rt_vector(pair,p2-p0);
    let uv_dx=rt_uv_delta(footprint[0],e1,e2,v1.uv.xy-v0.uv.xy,v2.uv.xy-v0.uv.xy);
    let uv_dy=rt_uv_delta(footprint[1],e1,e2,v1.uv.xy-v0.uv.xy,v2.uv.xy-v0.uv.xy);
    var shading = ocean_safe_normalize(rt_vector(pair,
        v0.normal.xyz*bary.x+v1.normal.xyz*bary.y+v2.normal.xyz*bary.z),geometry);
    shading = select(shading,-shading,dot(shading,geometry)<0.0);
    if (material.textures.z != 0xffffffffu) {
        shading = surface_map_normal(shading,rt_sample(material.textures.z,uv,uv_dx,uv_dy).xyz*2.0-1.0,
            material.factors.y,rt_vector(pair,p1-p0),rt_vector(pair,p2-p0),v1.uv.xy-v0.uv.xy,v2.uv.xy-v0.uv.xy);
    }
    let mr = rt_sample(material.textures.y,uv,uv_dx,uv_dy);
    var surface: RtSurface;
    surface.position = position;
    surface.normal = shading;
    surface.geometry = geometry;
    surface.albedo = material.color.rgb * rt_sample(material.textures.x,uv,uv_dx,uv_dy).rgb;
    surface.metallic = material.color.w * mr.b;
    surface.roughness = material.factors.x * mr.g;
    surface.occlusion = mix(1.0,rt_sample(material.textures.w,uv,uv_dx,uv_dy).r,material.factors.z)*(v0.uv.z*bary.x+v1.uv.z*bary.y+v2.uv.z*bary.z);
    surface.dielectric=material.optical.x;
    surface.transmission=material.factors.w;
    surface.emission=material.optical.yzw;
    return surface;
}

fn rt_reflection(
    position: vec3<f32>,
    normal: vec3<f32>,
    eye: vec3<f32>,
    f0: vec3<f32>,
    fresnel: vec3<f32>,
    roughness: f32,
    ambient: vec3<f32>,
    occlusion: f32,
) -> vec3<f32> {
    var cone_width=0.0;
    var cone_spread=0.0;
    if(!RT_LOW_QUALITY){
        cone_width=max(length(dpdx(position)),length(dpdy(position)));
        let outgoing=reflect(-eye,normal);
        cone_spread=max(length(dpdx(outgoing)),length(dpdy(outgoing)));
    }
    let env = mesh_environment(position,normal,eye,f0,roughness,ambient,occlusion);
    // A single mirror-direction sample cannot represent a wide specular lobe;
    // above the cutoff the estimate yields to the shared environment
    // exactly (mirror = 0 reproduces the raster term bit-for-bit).
    let mirror = clamp((RT_ROUGHNESS_CUTOFF - roughness) / RT_ROUGHNESS_CUTOFF, 0.0, 1.0);
    if mirror <= 0.0 {
        return env;
    }
    var radiance = vec3(0.0);
    var throughput = fresnel;
    var origin = position + normal * RT_BIAS;
    var direction = reflect(-eye, normal);
    for (var bounce = 0u; bounce < RT_BOUNCES; bounce++) {
        let hit = rt_trace(origin, direction, RT_BIAS, RT_TMAX, 0u);
        if !hit.hit {
            if (bounce==0u) { return env; }
            // The miss contributes the environment radiance of that direction
            // through the accumulated chain weight, matching the raster term's
            // operand order (`radiance * fresnel * occlusion`).
            radiance += env_radiance_from(origin,direction, ambient) * throughput;
            break;
        }
        cone_width+=hit.t*cone_spread;
        let surface = rt_surface(hit, direction,cone_width);
        let column=cloud_transport(origin,direction,length(surface.position-origin));
        radiance+=column.rgb*throughput;
        throughput*=column.a;
        radiance+=throughput*surface.emission;
        var ambient=light.ambient.rgb;
        if(!RT_LOW_QUALITY){ambient=(1.0-surface.transmission)*env_diffuse_irradiance(surface.normal,ambient)+surface.transmission*env_diffuse_irradiance(-surface.normal,ambient);}
        var illuminance = ambient;
        var visibility:array<f32,3>;
        for (var i = 0u; i < 3u; i++) {
            if any(light.colors[i].rgb > vec3(0.0)) {
                let shadow_normal=select(surface.geometry,-surface.geometry,dot(surface.geometry,light.directions[i].xyz)<0.0);
                visibility[i]=surface_visibility(surface.position,shadow_normal,i);
                let cosines=mesh_diffuse_cosines(surface.normal,surface.geometry,light.directions[i].xyz);
                illuminance +=
                    light.colors[i].rgb
                    * ((1.0-surface.transmission)*cosines.x+surface.transmission*cosines.y)
                    * visibility[i];
            }
        }
        let hit_f0 = mix(vec3(surface.dielectric), surface.albedo, surface.metallic);
        let hit_fresnel =
            select(surface_schlick_fresnel(hit_f0,dot(surface.normal,-direction)),hit_f0,RT_LOW_QUALITY);
        // Diffuse light leaving the hit toward this ray, then the specular
        // continuation of the same weighted chain.
        radiance +=
            throughput * (vec3(1.0) - hit_fresnel) * (1.0 - surface.metallic) * surface.albedo
            * (illuminance-ambient*(1.0-surface.occlusion)) / OCEAN_PI;
        for (var i=0u;i<3u;i++) {
            if (any(light.colors[i].rgb>vec3(0.0)) && dot(surface.geometry,light.directions[i].xyz)>0.0) {
                radiance += throughput*surface_sun_reflection(surface.normal,-direction,light.directions[i].xyz,
                    light.colors[i].rgb,surface.roughness,hit_f0)*visibility[i];
            }
        }
        let reflected = reflect(direction,surface.normal);
        let continuation = clamp((RT_ROUGHNESS_CUTOFF-surface.roughness)/RT_ROUGHNESS_CUTOFF,0.0,1.0);
        radiance += throughput*hit_fresnel*env_radiance_from(surface.position,reflected,ambient)*surface.occlusion*(1.0-continuation);
        throughput *= hit_fresnel*continuation*surface.occlusion;
        origin = surface.position + surface.geometry * RT_BIAS;
        direction = reflected;
        if all(throughput < vec3(0.002)) {
            break;
        }
    }
    // Everything arriving through this chain is indirect at the primary
    // surface, including a secondary hit's direct stellar radiance. Apply
    // primary AO once here, not only on a miss (or once per bounce).
    return mix(env, radiance * occlusion, mirror);
}
