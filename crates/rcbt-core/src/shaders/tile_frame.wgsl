// Shared transport for the geometry, classifier and raster consumers.
// CPU subtraction/rotation happens in f64; the stored vertex is tile-local.
struct CbtTileFrame {
    anchor_hi_m: vec4<f32>,
    anchor_lo_m: vec4<f32>,
    geometry: CbtPrecisionFrame,
};

fn cbt_render_position(frame: CbtTileFrame, local: vec3<f32>, transform: mat4x4<f32>) -> vec4<f32> {
    let rotated = (transform * vec4(local, 0.0)).xyz;
    return vec4(frame.anchor_hi_m.xyz + (frame.anchor_lo_m.xyz + rotated), 1.0);
}
