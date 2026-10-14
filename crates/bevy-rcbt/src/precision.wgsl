// Portable WGSL helper for CBT cube-sphere vertices.
//
// The caller owns the camera-relative anchor.  This helper returns only the
// local offset from a tile's base-radius centre, so it never subtracts two
// radius-sized f32 positions.  Its fields correspond to words()[2..7] in
// precision.rs: raw_center, raw_axis_u, raw_axis_v, normal, and
// radius_half_extent_len = (radius, half_extent, raw_center_length, 0).

struct CbtPrecisionFrame {
    raw_center: vec4<f32>,
    raw_axis_u: vec4<f32>,
    raw_axis_v: vec4<f32>,
    normal: vec4<f32>,
    radius_half_extent_len: vec4<f32>,
};

// Return the camera-relative local offset for a page sample.  The caller
// should add its separately uploaded camera-relative anchor_hi + anchor_lo.
fn cbt_local_offset(frame: CbtPrecisionFrame, uv: vec2<f32>, height: f32) -> vec3<f32> {
    let half_extent = frame.radius_half_extent_len.y;
    let delta_u = 2.0 * (uv.x - 0.5) * half_extent;
    let delta_v = 2.0 * (uv.y - 0.5) * half_extent;
    let delta = frame.raw_axis_u.xyz * delta_u + frame.raw_axis_v.xyz * delta_v;
    let normal = frame.normal.xyz;
    let radial = dot(normal, delta);
    let tangent = delta - normal * radial;
    let q = frame.radius_half_extent_len.z + radial;
    let tangent_squared = dot(tangent, tangent);
    let q_length = sqrt(q * q + tangent_squared);
    // This is (q / q_length - 1) written without cancellation.  The branch
    // avoids a zero denominator at the tile centre.
    var bend = 0.0;
    if (tangent_squared > 0.0) {
        bend = -tangent_squared / (q_length * (q_length + q));
    }
    let radius = frame.radius_half_extent_len.x;
    return tangent * ((radius + height) / q_length)
        + normal * (height + (radius + height) * bend);
}

struct CbtAabb {
    min_point: vec3<f32>,
    max_point: vec3<f32>,
};

// Slab ray-AABB intersection for portable WGSL ray query / bounding volume acceleration.
// Returns vec2<f32>(t_enter, t_exit) if hit (with t_exit >= 0.0 and t_enter <= t_exit), or vec2<f32>(-1.0, -1.0) on miss.
fn cbt_ray_intersect_aabb(aabb_min: vec3<f32>, aabb_max: vec3<f32>, ray_origin: vec3<f32>, ray_dir: vec3<f32>) -> vec2<f32> {
    let inv_d = 1.0 / ray_dir;
    let t0 = (aabb_min - ray_origin) * inv_d;
    let t1 = (aabb_max - ray_origin) * inv_d;
    let t_smaller = min(t0, t1);
    let t_bigger = max(t0, t1);
    let t_min = max(max(t_smaller.x, t_smaller.y), t_smaller.z);
    let t_max = min(min(t_bigger.x, t_bigger.y), t_bigger.z);
    if (t_max < max(t_min, 0.0)) {
        return vec2<f32>(-1.0, -1.0);
    }
    return vec2<f32>(max(t_min, 0.0), t_max);
}
