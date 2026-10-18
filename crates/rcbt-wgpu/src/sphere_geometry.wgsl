struct Record {
    address: vec4<u32>,
    neighbors: vec4<u32>,
}
struct Params {
    config: vec4<u32>, // capacity, max heap depth, manual/view mode, request kind
    requested_address: vec4<u32>,
    clip: mat4x4<f32>,
    camera: vec4<f32>,
    policy: vec4<f32>, // split area, merge area, viewport width/height
}
fn shift_right(id: vec2<u32>, n: u32) -> vec2<u32> {
    if (n == 0u) { return id; }
    if (n < 32u) { return vec2((id.x >> n) | (id.y << (32u - n)), id.y >> n); }
    return vec2(id.y >> (n - 32u), 0u);
}
fn child_id(id: vec2<u32>, right: u32) -> vec2<u32> {
    return vec2((id.x << 1u) | right, (id.y << 1u) | (id.x >> 31u));
}
fn root_of(record: Record) -> u32 {
    return shift_right(record.address.xy, record.address.z - 4u).x - 16u;
}
fn cube_triangle(record: Record) -> array<vec3<i32>,3> {
    let root = root_of(record);
    let face = root / 2u;
    let normals = array<vec3<i32>,6>(vec3(1,0,0), vec3(-1,0,0), vec3(0,1,0), vec3(0,-1,0), vec3(0,0,1), vec3(0,0,-1));
    let us = array<vec3<i32>,6>(vec3(0,1,0), vec3(0,-1,0), vec3(0,0,1), vec3(0,0,-1), vec3(1,0,0), vec3(-1,0,0));
    let vs = array<vec3<i32>,6>(vec3(0,0,1), vec3(0,0,1), vec3(1,0,0), vec3(1,0,0), vec3(0,1,0), vec3(0,1,0));
    let n = normals[face];
    let u = us[face];
    let v = vs[face];
    // 2^28 integer cube coordinates: exact midpoint decoding at 54 local steps.
    var a = (n + u + v) * 268435456;
    var b = (n - u + v) * 268435456;
    var c = (n - u - v) * 268435456;
    if ((root & 1u) != 0u) {
        a = (n - u - v) * 268435456;
        b = (n + u - v) * 268435456;
        c = (n + u + v) * 268435456;
    }
    for (var depth = i32(record.address.z) - 5; depth >= 0; depth--) {
        let mid = (a + c) / 2;
        let old_b = b;
        if ((shift_right(record.address.xy, u32(depth)).x & 1u) == 0u) {
            c = a;
            a = old_b;
        } else {
            a = c;
            c = old_b;
        }
        b = mid;
    }
    return array(a,b,c);
}
fn triangle(record: Record) -> mat3x3<f32> {
    let t = cube_triangle(record);
    return mat3x3(normalize(vec3<f32>(t[0])), normalize(vec3<f32>(t[1])), normalize(vec3<f32>(t[2])));
}
fn pixel_area(record: Record) -> f32 {
    if (record.address.z < 4u || root_of(record) >= 12u) { return 0.0; }
    let t = triangle(record);
    let center = (t[0] + t[1] + t[2]) / 3.0;
    if (dot(center, p.camera.xyz - center) <= 0.0) { return 0.0; }
    let a = p.clip * vec4(t[0], 1.0);
    let b = p.clip * vec4(t[1], 1.0);
    let c = p.clip * vec4(t[2], 1.0);
    if (min(a.w, min(b.w, c.w)) <= 0.0) { return 0.0; }
    let u = b.xy / b.w - a.xy / a.w;
    let v = c.xy / c.w - a.xy / a.w;
    return abs(u.x * v.y - u.y * v.x) * p.policy.z * p.policy.w / 8.0;
}
