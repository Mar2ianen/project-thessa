// Diagnostic CBT surface caster. Reads the canonical per-bisector vertex stream
// emitted by ConformingSurface: three 48-byte records per allocation slot
// (position+height, normal+face, UV+source+level). The emission kernel writes
// every live leaf with no camera culling, so a caster behind the camera still
// shadows a visible point.
//
// Both the emitted position and shadow_clip use camera-relative coordinates,
// exactly the space cbt_render_position produces with identity body rotation.
// Do not subtract the camera again or introduce a separate light-origin offset.
@group(0) @binding(0) var<uniform> shadow_clip: mat4x4<f32>;
struct SurfaceVertex { position_height:vec4<f32>, normal_face:vec4<f32>, address:vec4<u32> }
@group(1) @binding(0) var<storage,read> surface_vertices: array<SurfaceVertex>;
@vertex fn vertex(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let v = surface_vertices[index];
    // Only the FULL zero record is reserved. A real vertex can sit exactly
    // at the camera with zero UV and source slot; its normal is still nonzero.
    if (all(v.position_height == vec4(0.0)) && all(v.normal_face == vec4(0.0)) && all(v.address == vec4(0u))) {
        return vec4(2.0, 2.0, 2.0, 1.0);
    }
    return shadow_clip * vec4(v.position_height.xyz, 1.0);
}
