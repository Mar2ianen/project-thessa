struct Frame {
    clip: mat4x4<f32>,
    inverse_clip: mat4x4<f32>,
    camera: vec4<f32>,
    // Shared scene pre-exposure gain; alpha/transmittance stays unexposed.
    exposure: vec4<f32>,
}
@group(0) @binding(0) var<uniform> frame: Frame;
@group(0) @binding(1) var scene_depth: texture_depth_2d;

fn unproject(pixel: vec2<f32>, depth: f32) -> vec3<f32> {
    let extent = vec2<f32>(textureDimensions(scene_depth));
    let ndc = vec2<f32>(pixel.x / extent.x * 2.0 - 1.0, 1.0 - pixel.y / extent.y * 2.0);
    let p = frame.inverse_clip * vec4<f32>(ndc, depth, 1.0);
    return p.xyz / p.w;
}
