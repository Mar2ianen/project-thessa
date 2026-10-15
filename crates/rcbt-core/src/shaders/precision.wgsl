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

fn cbt_is_finite_f32(v: f32) -> bool {
    return (v == v) && (abs(v) <= 3.402823466e+38);
}

fn cbt_is_finite_vec3(v: vec3<f32>) -> bool {
    return cbt_is_finite_f32(v.x) && cbt_is_finite_f32(v.y) && cbt_is_finite_f32(v.z);
}

// Slab ray-AABB intersection for portable WGSL ray query / bounding volume acceleration.
// Returns vec2<f32>(t_enter, t_exit) if hit (with t_exit >= 0.0 and t_enter <= t_exit), or vec2<f32>(-1.0, -1.0) on miss.
fn cbt_ray_intersect_aabb(aabb_min: vec3<f32>, aabb_max: vec3<f32>, ray_origin: vec3<f32>, ray_dir: vec3<f32>) -> vec2<f32> {
    if (!cbt_is_finite_vec3(aabb_min) || !cbt_is_finite_vec3(aabb_max) ||
        !cbt_is_finite_vec3(ray_origin) || !cbt_is_finite_vec3(ray_dir)) {
        return vec2<f32>(-1.0, -1.0);
    }
    if (ray_dir.x == 0.0 && ray_dir.y == 0.0 && ray_dir.z == 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }

    var t_min: f32 = -3.402823466e+38;
    var t_max: f32 = 3.402823466e+38;

    // Axis X
    if (ray_dir.x == 0.0) {
        if (ray_origin.x < aabb_min.x || ray_origin.x > aabb_max.x) {
            return vec2<f32>(-1.0, -1.0);
        }
    } else {
        let inv_dx = 1.0 / ray_dir.x;
        var t0x = (aabb_min.x - ray_origin.x) * inv_dx;
        var t1x = (aabb_max.x - ray_origin.x) * inv_dx;
        if (t0x > t1x) {
            let tmp = t0x;
            t0x = t1x;
            t1x = tmp;
        }
        t_min = max(t_min, t0x);
        t_max = min(t_max, t1x);
        if (t_max < t_min) {
            return vec2<f32>(-1.0, -1.0);
        }
    }

    // Axis Y
    if (ray_dir.y == 0.0) {
        if (ray_origin.y < aabb_min.y || ray_origin.y > aabb_max.y) {
            return vec2<f32>(-1.0, -1.0);
        }
    } else {
        let inv_dy = 1.0 / ray_dir.y;
        var t0y = (aabb_min.y - ray_origin.y) * inv_dy;
        var t1y = (aabb_max.y - ray_origin.y) * inv_dy;
        if (t0y > t1y) {
            let tmp = t0y;
            t0y = t1y;
            t1y = tmp;
        }
        t_min = max(t_min, t0y);
        t_max = min(t_max, t1y);
        if (t_max < t_min) {
            return vec2<f32>(-1.0, -1.0);
        }
    }

    // Axis Z
    if (ray_dir.z == 0.0) {
        if (ray_origin.z < aabb_min.z || ray_origin.z > aabb_max.z) {
            return vec2<f32>(-1.0, -1.0);
        }
    } else {
        let inv_dz = 1.0 / ray_dir.z;
        var t0z = (aabb_min.z - ray_origin.z) * inv_dz;
        var t1z = (aabb_max.z - ray_origin.z) * inv_dz;
        if (t0z > t1z) {
            let tmp = t0z;
            t0z = t1z;
            t1z = tmp;
        }
        t_min = max(t_min, t0z);
        t_max = min(t_max, t1z);
        if (t_max < t_min) {
            return vec2<f32>(-1.0, -1.0);
        }
    }

    if (t_max < 0.0) {
        return vec2<f32>(-1.0, -1.0);
    }
    return vec2<f32>(max(t_min, 0.0), t_max);
}
