// Material addressing is body/world space, never the geometry leaf ordinal.
@group(0) @binding(16) var<storage, read> material_directory: array<vec4<u32>>;

struct MaterialAddress {
    uv: vec2<f32>,
    face: u32,
};

fn material_face_uv(d: vec3<f32>, face: u32) -> vec2<f32> {
    var uv = vec2(0.0);
    switch face {
        case 0u: { uv = vec2(-d.z, d.y) / abs(d.x); }
        case 1u: { uv = vec2(d.z, d.y) / abs(d.x); }
        case 2u: { uv = vec2(d.x, -d.z) / abs(d.y); }
        case 3u: { uv = vec2(d.x, d.z) / abs(d.y); }
        case 4u: { uv = vec2(d.x, d.y) / abs(d.z); }
        default: { uv = vec2(-d.x, d.y) / abs(d.z); }
    }
    return uv * 0.5 + 0.5;
}

fn material_address(d: vec3<f32>) -> MaterialAddress {
    let a = abs(d);
    var uv = vec2(0.0);
    var face = 0u;
    if (a.x >= a.y && a.x >= a.z) {
        face = select(1u, 0u, d.x >= 0.0);
        uv = vec2(select(d.z, -d.z, d.x >= 0.0), d.y) / a.x;
    } else if (a.y >= a.z) {
        face = select(3u, 2u, d.y >= 0.0);
        uv = vec2(d.x, select(d.z, -d.z, d.y >= 0.0)) / a.y;
    } else {
        face = select(5u, 4u, d.z >= 0.0);
        uv = vec2(select(-d.x, d.x, d.z >= 0.0), d.y) / a.z;
    }
    return MaterialAddress(clamp(uv * 0.5 + 0.5, vec2(0.0), vec2(0.99999994)), face);
}

fn material_node(face: u32, xy: vec2<u32>, level: u32) -> vec2<u32> {
    var id = vec2(8u + face, 0u);
    for (var bit = level; bit > 0u; bit -= 1u) {
        let pair = (((xy.x >> (bit - 1u)) & 1u) << 1u) | ((xy.y >> (bit - 1u)) & 1u);
        id = vec2((id.x << 2u) | pair, (id.y << 2u) | (id.x >> 30u));
    }
    return id;
}

fn material_layer(id: vec2<u32>) -> u32 {
    let mask = arrayLength(&material_directory) - 1u;
    var index = (id.x * 0x9e3779b9u ^ id.y * 0x85ebca6bu) & mask;
    for (var probe = 0u; probe <= mask; probe += 1u) {
        let entry = material_directory[index];
        if (all(entry.xy == id)) { return entry.z; }
        if (all(entry.xy == vec2(0u))) { break; }
        index = (index + 1u) & mask;
    }
    return 0xffffffffu;
}

// Gradients are evaluated before any residency-dependent control flow.
fn material_at_level(address: MaterialAddress, dx: vec2<f32>, dy: vec2<f32>, requested: u32) -> vec4<f32> {
    var level = requested;
    var extent = exp2(f32(level));
    var id = material_node(address.face, vec2<u32>(address.uv * extent), level);
    loop {
        let layer = material_layer(id);
        if (layer != 0xffffffffu) {
            let uv = (fract(address.uv * extent) * 125.0 + 1.5) / 128.0;
            return textureSampleGrad(material_texture, material_sampler, uv, i32(layer),
                dx * extent * (125.0 / 128.0), dy * extent * (125.0 / 128.0));
        }
        if (level == 0u) { break; }
        id = vec2((id.x >> 2u) | (id.y << 30u), id.y >> 2u);
        level -= 1u;
        extent *= 0.5;
    }
    return vec4(0.0, 0.0, 0.0, -1.0);
}
