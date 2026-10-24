// Recover integer cube coordinates from canonical per-vertex ownership. A
// triangle crossing a cube edge still needs ONE common interpolation chart.
fn conforming_cube(address: vec2<u32>, face: u32) -> vec3<i32> {
    let p = vec2<i32>(address) - vec2(268435456);
    switch face {
        case 0u: { return vec3(268435456, p.y, -p.x); }
        case 1u: { return vec3(-268435456, p.y, p.x); }
        case 2u: { return vec3(p.x, 268435456, -p.y); }
        case 3u: { return vec3(p.x, -268435456, p.y); }
        case 4u: { return vec3(p.x, p.y, 268435456); }
        default: { return vec3(-p.x, p.y, -268435456); }
    }
}
fn conforming_face_uv(cube: vec3<i32>, face: u32) -> vec2<i32> {
    var p = vec2<i32>(0);
    switch face {
        case 0u: { p = vec2(-cube.z, cube.y); }
        case 1u: { p = vec2(cube.z, cube.y); }
        case 2u: { p = vec2(cube.x, -cube.z); }
        case 3u: { p = vec2(cube.x, cube.z); }
        case 4u: { p = cube.xy; }
        default: { p = vec2(-cube.x, cube.y); }
    }
    return p + vec2(268435456);
}
