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
// sRGB base colour and linear MR/normal/AO. Samples have explicit LOD zero:
// a reflected ray has no screen-space texture derivatives.

struct RtMaterial {
    // rgb = linear base color, w = metallic.
    color: vec4<f32>,
    // x = roughness, y = normal scale, z = AO strength, w = diffuse transmission.
    factors: vec4<f32>,
    // Base colour, metallic/roughness, normal, AO; UINT_MAX means absent.
    textures: vec4<u32>,
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
}

// Barycentrics land in the primitive's own asset-local space; the pair rows
// move the point and its normal into the camera-relative space the primary
// surface, the lights and the shadow atlas live in.
fn rt_vector(pair:RtPair, vector:vec3<f32>)->vec3<f32> {
    return vec3(dot(pair.row0.xyz,vector),dot(pair.row1.xyz,vector),dot(pair.row2.xyz,vector));
}

fn rt_surface(hit: RtHit, ray_direction: vec3<f32>) -> RtSurface {
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
    var shading = ocean_safe_normalize(rt_vector(pair,
        v0.normal.xyz*bary.x+v1.normal.xyz*bary.y+v2.normal.xyz*bary.z),geometry);
    shading = select(shading,-shading,dot(shading,geometry)<0.0);
    if (material.textures.z != 0xffffffffu) {
        shading = surface_map_normal(shading,rt_sample(material.textures.z,uv).xyz*2.0-1.0,
            material.factors.y,rt_vector(pair,p1-p0),rt_vector(pair,p2-p0),v1.uv.xy-v0.uv.xy,v2.uv.xy-v0.uv.xy);
    }
    let mr = rt_sample(material.textures.y,uv);
    var surface: RtSurface;
    surface.position = position;
    surface.normal = shading;
    surface.geometry = geometry;
    surface.albedo = material.color.rgb * rt_sample(material.textures.x,uv).rgb;
    surface.metallic = material.color.w * mr.b;
    surface.roughness = material.factors.x * mr.g;
    surface.occlusion = mix(1.0,rt_sample(material.textures.w,uv).r,material.factors.z)*(v0.uv.z*bary.x+v1.uv.z*bary.y+v2.uv.z*bary.z);
    surface.dielectric=0.04;
    surface.transmission=material.factors.w;
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
        let surface = rt_surface(hit, direction);
        let column=cloud_transport(origin,direction,length(surface.position-origin));
        radiance+=column.rgb*throughput;
        throughput*=column.a;
        var illuminance = light.ambient.rgb;
        var visibility:array<f32,3>;
        for (var i = 0u; i < 3u; i++) {
            if any(light.colors[i].rgb > vec3(0.0)) {
                let shadow_normal=select(surface.geometry,-surface.geometry,surface.transmission>0.0 && dot(surface.geometry,light.directions[i].xyz)<0.0);
                visibility[i]=surface_visibility(surface.position,shadow_normal,i);
                illuminance +=
                    light.colors[i].rgb
                    * ((1.0-surface.transmission)*max(dot(surface.normal,light.directions[i].xyz),0.0)
                        +surface.transmission*max(-dot(surface.normal,light.directions[i].xyz),0.0))
                    * visibility[i];
            }
        }
        let hit_f0 = mix(vec3(surface.dielectric), surface.albedo, surface.metallic);
        let hit_fresnel =
            hit_f0 + select((vec3(1.0) - hit_f0) * pow(1.0 - max(dot(surface.normal, -direction), 0.0), 5.0),vec3(0.0),RT_LOW_QUALITY);
        // Diffuse light leaving the hit toward this ray, then the specular
        // continuation of the same weighted chain.
        radiance +=
            throughput * (vec3(1.0) - hit_fresnel) * (1.0 - surface.metallic) * surface.albedo
            * (illuminance-light.ambient.rgb*(1.0-surface.occlusion)) / OCEAN_PI;
        for (var i=0u;i<3u;i++) {
            if (any(light.colors[i].rgb>vec3(0.0))) {
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
