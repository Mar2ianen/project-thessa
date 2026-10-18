// Shared sparse canonical cube-page directory ABI (height and material).
// The consumer declares material_directory at its own binding.
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
