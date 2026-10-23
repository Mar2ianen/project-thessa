// Shared traversal for scene-object shadows and optional textured reflections.
@group(3) @binding(0) var rt_scene:acceleration_structure;

struct RtHit {
    hit:bool,
    t:f32,
    pair:u32,
    triangle:u32,
    barycentrics:vec2<f32>,
}

fn rt_trace(origin:vec3<f32>,direction:vec3<f32>,t_min:f32,t_max:f32,flags:u32)->RtHit {
    var result:RtHit;
    var query:ray_query;
    rayQueryInitialize(&query,rt_scene,RayDesc(flags,0xFFu,t_min,t_max,origin,direction));
    rayQueryProceed(&query);
    let intersection=rayQueryGetCommittedIntersection(&query);
    result.hit=intersection.kind!=RAY_QUERY_INTERSECTION_NONE;
    if (result.hit) {
        result.t=intersection.t;
        result.pair=intersection.instance_custom_data;
        result.triangle=intersection.primitive_index;
        result.barycentrics=intersection.barycentrics;
    }
    return result;
}

// Keep the raster atlas for terrain and other non-RT casters. Neither source
// can unshadow the other. No rays when the resolved shadow effect is disabled.
fn surface_visibility(position:vec3<f32>,normal:vec3<f32>,sun:u32)->f32 {
    let atlas=shadow_visibility(position,normal,sun)*cloud_sun_visibility(position,sun);
    let cosine=dot(normal,light.directions[sun].xyz);
    if (!RT_SHADOWS || atlas<=0.0 || cosine<=0.0) { return atlas; }
    // Exclude the receiver's normal-distance bias along the SAME stellar ray.
    // Moving its origin sideways can hit a disjoint narrow leaf and invent shadow.
    let min_t=RT_BIAS/cosine;
    if (min_t>=RT_TMAX) { return atlas; }
    let hit=rt_trace(position,light.directions[sun].xyz,min_t,RT_TMAX,RAY_FLAG_TERMINATE_ON_FIRST_HIT);
    return select(atlas,0.0,hit.hit);
}
