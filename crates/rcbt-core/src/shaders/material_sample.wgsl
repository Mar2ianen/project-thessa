// Material addressing is body/world space, never the geometry leaf ordinal.
@group(0) @binding(16) var<storage, read> material_directory: array<vec4<u32>>;

struct MaterialAddress {
    // Split chart coordinate. Keep the small interpolated displacement separate
    // from the world-fixed anchor until after scaling to a material page.
    uv: vec2<f32>,
    face: u32,
    offset: vec2<f32>,
};

struct MaterialCoordinates {
    tile: vec2<f32>,
    local: vec2<f32>,
};

// Exact symmetric angular chord without subtracting nearly equal unit vectors.
// Unlike a linearized Jacobian, it stays bounded at coarse grazing footprints.
fn material_chart_chord(q: vec3<f32>, d: vec3<f32>) -> vec3<f32> {
    let a = q - d * 0.5;
    let la = length(a);
    let lb = length(a + d);
    // Rationalize lb-la before dividing: small d must not lose its low bits.
    let length_delta = (2.0 * dot(a, d) + dot(d, d)) / (lb + la);
    return (d - a * (length_delta / la)) / lb;
}

// Axis permutations/signs do not change chord lengths: all six cube faces.
fn material_chart_footprint(address: MaterialAddress, dx: vec2<f32>, dy: vec2<f32>, radius: f32) -> f32 {
    let q = vec3<f32>((address.uv + address.offset) * 2.0 - 1.0, 1.0);
    let q_dx = vec3<f32>(dx * 2.0, 0.0);
    let q_dy = vec3<f32>(dy * 2.0, 0.0);
    let tangent_dx = material_chart_chord(q, q_dx);
    let tangent_dy = material_chart_chord(q, q_dy);
    return max(length(tangent_dx), length(tangent_dy)) * radius;
}

fn material_coordinates(address: MaterialAddress, extent: f32) -> MaterialCoordinates {
    let base = address.uv * extent;
    let relative = fract(base) + address.offset * extent;
    let tile = clamp(floor(base) + floor(relative), vec2(0.0), vec2(extent - 1.0));
    return MaterialCoordinates(tile, (floor(base) - tile) + relative);
}

// Premultiplied resident contribution; uncovered weight uses the globe map.
struct MaterialSample {
    value: vec4<f32>,
    coverage: f32,
};

fn material_direction(uv: vec2<f32>, face: u32) -> vec3<f32> {
    let p = uv * 2.0 - 1.0;
    switch face {
        case 0u: { return normalize(vec3(1.0, p.y, -p.x)); }
        case 1u: { return normalize(vec3(-1.0, p.y, p.x)); }
        case 2u: { return normalize(vec3(p.x, 1.0, -p.y)); }
        case 3u: { return normalize(vec3(p.x, -1.0, p.y)); }
        case 4u: { return normalize(vec3(p.x, p.y, 1.0)); }
        default: { return normalize(vec3(-p.x, p.y, -1.0)); }
    }
}

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
    return MaterialAddress(clamp(uv * 0.5 + 0.5, vec2(0.0), vec2(0.99999994)), face, vec2(0.0));
}

// Only missing neighbours need a transition to shared coarser data. A complete
// equal-resolution neighbourhood keeps all its detail, including page edges.
// Diagonals prevent a single resident quadrant from leaving a corner seam.
fn material_resident_weight(address: MaterialAddress, level: u32, feather: f32) -> f32 {
    let extent = exp2(f32(level));
    let coordinates = material_coordinates(address, extent);
    let tile = coordinates.tile;
    let local = coordinates.local;
    var weight = 1.0;
    for (var y = -1; y <= 1; y += 1) {
        for (var x = -1; x <= 1; x += 1) {
            if (x == 0 && y == 0) { continue; }
            var distance = 0.0;
            if (x < 0) { distance = max(distance, local.x); }
            if (x > 0) { distance = max(distance, 1.0 - local.x); }
            if (y < 0) { distance = max(distance, local.y); }
            if (y > 0) { distance = max(distance, 1.0 - local.y); }
            if (distance >= feather) { continue; }
            let neighbour_tile = tile + vec2(f32(x), f32(y));
            var id = material_node(address.face, vec2<u32>(max(neighbour_tile, vec2(0.0))), level);
            // Reproject across cube edges rather than clamping to this face.
            // Same-face neighbours already have an exact integer address.
            if (any(neighbour_tile < vec2(0.0)) || any(neighbour_tile >= vec2(extent))) {
                let neighbour_uv = (neighbour_tile + 0.5) / extent;
                let neighbour = material_address(material_direction(neighbour_uv, address.face));
                id = material_node(neighbour.face, vec2<u32>(neighbour.uv * extent), level);
            }
            if (material_layer(id) == 0xffffffffu) {
                weight = min(weight, smoothstep(0.0, feather, distance));
            }
        }
    }
    return weight;
}

// Gradients are evaluated before any residency-dependent control flow.
fn material_at_level(address: MaterialAddress, dx: vec2<f32>, dy: vec2<f32>, requested: u32) -> MaterialSample {
    var level = requested;
    var extent = exp2(f32(level));
    var id = material_node(address.face, vec2<u32>(material_coordinates(address, extent).tile), level);
    var result = MaterialSample(vec4(0.0), 0.0);
    var remaining = 1.0;
    loop {
        let layer = material_layer(id);
        if (layer != 0xffffffffu) {
            let feather = clamp(max(4.0 / 125.0,
                2.0 * extent * max(length(dx), length(dy))), 4.0 / 125.0, 0.125);
            let weight = remaining * material_resident_weight(address, level, feather);
            let uv = (material_coordinates(address, extent).local * 125.0 + 1.5) / 128.0;
            result.value += textureSampleGrad(material_texture, material_sampler, uv, i32(layer),
                dx * extent * (125.0 / 128.0), dy * extent * (125.0 / 128.0)) * weight;
            result.coverage += weight;
            remaining -= weight;
            if (remaining <= 0.00001) { break; }
        }
        if (level == 0u) { break; }
        id = vec2((id.x >> 2u) | (id.y << 30u), id.y >> 2u);
        level -= 1u;
        extent *= 0.5;
    }
    return result;
}
