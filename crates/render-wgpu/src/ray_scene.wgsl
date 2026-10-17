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
// Hit shading uses the material's linear base color, the geometric normal and
// the existing surface illumination model (ambient + per-light N·L with the
// shared shadow atlas); textures and UVs are not sampled in this slice.

struct RtMaterial {
    // rgb = linear base color, w = metallic.
    color: vec4<f32>,
    // x = roughness, yzw unused.
    factors: vec4<f32>,
}

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

@group(3) @binding(0) var rt_scene: acceleration_structure;
@group(3) @binding(1) var<storage, read> rt_materials: array<RtMaterial>;
@group(3) @binding(2) var<storage, read> rt_ranges: array<RtRange>;
@group(3) @binding(3) var<storage, read> rt_positions: array<vec4<f32>>;
@group(3) @binding(4) var<storage, read> rt_indices: array<u32>;
@group(3) @binding(5) var<storage, read> rt_pairs: array<RtPair>;

struct RtHit {
    hit: bool,
    t: f32,
    // TLAS slot of the hit instance. Instance custom data carries this slot
    // because a TLAS instance references a whole BLAS: the slot, not the
    // driver's geometry index, identifies which primitive was placed.
    pair: u32,
    triangle: u32,
    barycentrics: vec2<f32>,
}

struct RtSurface {
    position: vec3<f32>,
    normal: vec3<f32>,
    albedo: vec3<f32>,
    metallic: f32,
    roughness: f32,
}

fn rt_trace(origin: vec3<f32>, direction: vec3<f32>, t_min: f32, t_max: f32) -> RtHit {
    var result: RtHit;
    result.hit = false;
    result.t = 0.0;
    result.pair = 0u;
    result.triangle = 0u;
    result.barycentrics = vec2(0.0);
    var query: ray_query;
    rayQueryInitialize(
        &query,
        rt_scene,
        RayDesc(0u, 0xFFu, t_min, t_max, origin, direction),
    );
    rayQueryProceed(&query);
    let intersection = rayQueryGetCommittedIntersection(&query);
    if intersection.kind != RAY_QUERY_INTERSECTION_NONE {
        result.hit = true;
        result.t = intersection.t;
        result.pair = intersection.instance_custom_data;
        result.triangle = intersection.primitive_index;
        result.barycentrics = intersection.barycentrics;
    }
    return result;
}

// Barycentrics land in the primitive's own asset-local space; the pair rows
// move the point and its normal into the camera-relative space the primary
// surface, the lights and the shadow atlas live in.
fn rt_surface(hit: RtHit, ray_direction: vec3<f32>) -> RtSurface {
    let pair = rt_pairs[hit.pair];
    let range = rt_ranges[pair.words.x];
    let first = range.words.x + hit.triangle * 3u;
    let base = range.words.y;
    let p0 = rt_positions[base + rt_indices[first]].xyz;
    let p1 = rt_positions[base + rt_indices[first + 1u]].xyz;
    let p2 = rt_positions[base + rt_indices[first + 2u]].xyz;
    let local = p0 + (p1 - p0) * hit.barycentrics.x + (p2 - p0) * hit.barycentrics.y;
    let local_normal = normalize(cross(p1 - p0, p2 - p0));
    let position = vec3(
        dot(pair.row0.xyz, local) + pair.row0.w,
        dot(pair.row1.xyz, local) + pair.row1.w,
        dot(pair.row2.xyz, local) + pair.row2.w,
    );
    let normal = normalize(vec3(
        dot(pair.row0.xyz, local_normal),
        dot(pair.row1.xyz, local_normal),
        dot(pair.row2.xyz, local_normal),
    ));
    // Geometric normals are two-sided: face the incoming ray so single-sided
    // assets still shade the reflection instead of lighting a back face.
    let geometry = select(normal, -normal, dot(normal, ray_direction) > 0.0);
    let material = rt_materials[pair.words.x];
    var surface: RtSurface;
    surface.position = position;
    surface.normal = geometry;
    surface.albedo = material.color.rgb;
    surface.metallic = material.color.w;
    surface.roughness = material.factors.x;
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
    // Same operand order as the raster environment term, so `mirror = 0`
    // reproduces it exactly instead of approximately.
    let env = ambient / OCEAN_PI * fresnel * occlusion;
    // A single mirror-direction sample cannot represent a wide specular lobe;
    // above the cutoff the estimate yields to the isotropic environment
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
        let hit = rt_trace(origin, direction, RT_BIAS, RT_TMAX);
        if !hit.hit {
            radiance += throughput * env;
            break;
        }
        let surface = rt_surface(hit, direction);
        var illuminance = light.ambient.rgb;
        for (var i = 0u; i < 3u; i++) {
            if any(light.colors[i].rgb > vec3(0.0)) {
                illuminance +=
                    light.colors[i].rgb
                    * max(dot(surface.normal, light.directions[i].xyz), 0.0)
                    * shadow_visibility(surface.position, surface.normal, i);
            }
        }
        let hit_f0 = mix(vec3(0.04), surface.albedo, surface.metallic);
        let hit_fresnel =
            hit_f0
            + (vec3(1.0) - hit_f0) * pow(1.0 - max(dot(surface.normal, -direction), 0.0), 5.0);
        // Diffuse light leaving the hit toward this ray, then the specular
        // continuation of the same weighted chain.
        radiance +=
            throughput * (vec3(1.0) - hit_fresnel) * (1.0 - surface.metallic) * surface.albedo
            * illuminance / OCEAN_PI;
        throughput *= hit_fresnel;
        origin = surface.position + surface.normal * RT_BIAS;
        direction = reflect(direction, surface.normal);
        if all(throughput < vec3(0.002)) {
            break;
        }
    }
    return mix(env, radiance, mirror);
}
