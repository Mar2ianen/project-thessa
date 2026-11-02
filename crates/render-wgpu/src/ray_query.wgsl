// Shared traversal for scene-object shadows and optional textured reflections.
@group(3) @binding(0) var rt_scene:acceleration_structure;

struct RtHit {
    hit:bool,
    t:f32,
    pair:u32,
    triangle:u32,
    barycentrics:vec2<f32>,
}

fn rt_trace(origin:vec3<f32>,direction:vec3<f32>,t_min:f32,t_max:f32,flags:u32,cone_width:f32,cone_spread:f32)->RtHit {
    var result:RtHit;
    var query:ray_query;
    rayQueryInitialize(&query,rt_scene,RayDesc(flags,0xFFu,t_min,t_max,origin,direction));
    // CANDIDATE_TRAVERSAL
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

// A valid cheaper occlusion remains final; do not trace already-dark or backlit samples.
fn rt_visibility(position:vec3<f32>,normal:vec3<f32>,direction:vec3<f32>,visibility:f32,bias:f32,max_distance:f32)->f32 {
    let min_t=fastrt_visibility_min_distance(visibility,dot(normal,direction),bias,max_distance);
    if(min_t<0.0) { return visibility; }
    let hit=rt_trace(position,direction,min_t,max_distance,RAY_FLAG_TERMINATE_ON_FIRST_HIT,0.0,0.0);
    return select(visibility,0.0,hit.hit);
}
