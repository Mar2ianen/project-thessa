// Diagnostic CBT surface caster. Reads the canonical per-bisector vertex stream
// emitted by ConformingSurface: three 48-byte records per allocation slot
// (position+height, normal+face, UV+source+level). The emission kernel writes
// every live leaf with no camera culling, so a caster behind the camera still
// shadows a visible point.
//
// The emitted position is tile-anchor-relative to the terrain camera, exactly
// the space `cbt_render_position` produces, and shadow_clip is built about the
// light matrix origin. Those differ by the light's origin, which the legacy
// patch caster removes with its body-rotation matrix. That matrix is the body
// rotation and is identity on this path, so it is inlined here rather than
// retaining a buffer whose contents never change.
@group(0) @binding(0) var<uniform> shadow_clip: mat4x4<f32>;
@group(1) @binding(0) var<storage,read> surface_vertices: array<vec4<f32>>;
@vertex fn vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let vertex = index * 3u;
    let position = surface_vertices[vertex].xyz;
    let address = surface_vertices[vertex + 2u].xyz;
    // Virtual roots are zeroed by the emission kernel. No real leaf sits at the
    // anchor-relative origin with an all-zero canonical address. The address
    // payload is vec4<u32>, so compare it as bits instead of reinterpreting it
    // as floats.
    if (all(position == vec3(0.0)) && all(bitcast<vec3<u32>>(address) == vec3(0u))) {
        return vec4(2.0, 2.0, 2.0, 1.0);
    }
    return shadow_clip * vec4(position, 1.0);
}
