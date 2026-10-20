// Primary-raster material normal sampling.
//
// This is a deliberate raster-only variant of the shared albedo sampler: the
// residency hierarchy, feathering and page transform stay identical, but the
// normal plane is sampled in the same pass instead of forcing every secondary
// consumer (ray-traced terrain, ocean geometry) to declare unused bindings.
// Normal pages are body-space cube pages; xy is linear RG8 and z is
// reconstructed positive in the page's face tangent frame.
@group(0) @binding(19) var material_normal_texture: texture_2d_array<f32>;
@group(0) @binding(20) var<storage, read> material_normal_directory: array<vec4<u32>>;
const MATERIAL_NORMALS_ENABLED: bool = true;
const MATERIAL_NORMAL_MIP_BIAS: f32 = 0.0;

struct RasterMaterialSample {
    value: vec4<f32>,
    normal: vec2<f32>,
    coverage: f32,
};

fn material_normal_layer(id: vec2<u32>) -> u32 {
    let mask = arrayLength(&material_normal_directory) - 1u;
    var index = (id.x * 0x9e3779b9u ^ id.y * 0x85ebca6bu) & mask;
    for (var probe = 0u; probe <= mask; probe += 1u) {
        let entry = material_normal_directory[index];
        if (all(entry.xy == id)) { return entry.z; }
        if (all(entry.xy == vec2(0u))) { break; }
        index = (index + 1u) & mask;
    }
    return 0xffffffffu;
}

fn material_at_level_raster(address: MaterialAddress, dx: vec2<f32>, dy: vec2<f32>, requested: u32) -> RasterMaterialSample {
    var level = requested;
    var extent = exp2(f32(level));
    var id = material_node(address.face, vec2<u32>(address.uv * extent), level);
    var result = RasterMaterialSample(vec4(0.0), vec2(0.0), 0.0);
    var remaining = 1.0;
    loop {
        let layer = material_layer(id);
        if (layer != 0xffffffffu) {
            let feather = clamp(max(4.0 / 125.0,
                2.0 * extent * max(length(dx), length(dy))), 4.0 / 125.0, 0.125);
            let weight = remaining * material_resident_weight(address, level, feather);
            let uv = (fract(address.uv * extent) * 125.0 + 1.5) / 128.0;
            let footprint = vec2<f32>(extent * (125.0 / 128.0));
            result.value += textureSampleGrad(material_texture, material_sampler, uv, i32(layer),
                dx * footprint, dy * footprint) * weight;
            if (MATERIAL_NORMALS_ENABLED) {
              let normal_layer = material_normal_layer(id);
              if (normal_layer != 0xffffffffu) {
                let normal_footprint = footprint * exp2(MATERIAL_NORMAL_MIP_BIAS);
                let xy = textureSampleGrad(material_normal_texture, material_sampler, uv,
                    i32(normal_layer), dx * normal_footprint, dy * normal_footprint).rg * 2.0 - 1.0;
                // Blend height gradients, not unit-normal XY; absent pages
                // contribute zero rather than amplifying sparse coverage.
                let z = sqrt(max(0.0001, 1.0 - dot(xy, xy)));
                result.normal += xy / z * weight;
              }
            }
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

// Face UV basis matching `material_direction`. Columns are the unit tangent,
// bitangent and face normal in body space.
fn material_face_basis(uv: vec2<f32>, face: u32) -> mat3x3<f32> {
    let p = uv * 2.0 - 1.0;
    var v = vec3(0.0);
    var du = vec3(0.0);
    var dv = vec3(0.0);
    switch face {
        case 0u: { v = vec3(1.0, p.y, -p.x); du = vec3(0.0, 0.0, -1.0); dv = vec3(0.0, 1.0, 0.0); }
        case 1u: { v = vec3(-1.0, p.y, p.x); du = vec3(0.0, 0.0, 1.0); dv = vec3(0.0, 1.0, 0.0); }
        case 2u: { v = vec3(p.x, 1.0, -p.y); du = vec3(1.0, 0.0, 0.0); dv = vec3(0.0, 0.0, -1.0); }
        case 3u: { v = vec3(p.x, -1.0, p.y); du = vec3(1.0, 0.0, 0.0); dv = vec3(0.0, 0.0, 1.0); }
        case 4u: { v = vec3(p.x, p.y, 1.0); du = vec3(1.0, 0.0, 0.0); dv = vec3(0.0, 1.0, 0.0); }
        default: { v = vec3(-p.x, p.y, -1.0); du = vec3(-1.0, 0.0, 0.0); dv = vec3(0.0, 1.0, 0.0); }
    }
    let n = normalize(v);
    let t = normalize(du - n * dot(n, du));
    let b = normalize(dv - n * dot(n, dv) - t * dot(t, dv));
    return mat3x3<f32>(t, b, n);
}

// `tangent` is the negative height gradient in the face tangent frame. Zero means
// no normal page was resident, which returns the geometric normal exactly.
fn material_perturb_normal(geometric: vec3<f32>, address: MaterialAddress, tangent: vec2<f32>, render_from_body: mat4x4<f32>) -> vec3<f32> {
    if (dot(tangent, tangent) <= 0.0) { return geometric; }
    let basis = material_face_basis(address.uv, address.face);
    let rotation = mat3x3<f32>(
        render_from_body[0].xyz,
        render_from_body[1].xyz,
        render_from_body[2].xyz
    );
    let gradient = rotation * (basis[0] * tangent.x + basis[1] * tangent.y);
    // Project the world-space bump gradient onto the actual surface without
    // independently normalizing the axes (which would change slope energy).
    let projected = gradient - geometric * dot(geometric, gradient);
    return normalize(geometric + projected);
}
