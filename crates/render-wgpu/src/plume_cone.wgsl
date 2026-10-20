@group(1) @binding(0) var<uniform> model: mat4x4<f32>;
@group(1) @binding(1) var impostor: texture_2d<f32>;
@group(1) @binding(2) var impostor_sampler: sampler;
struct Output {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
}
@vertex fn vertex(@location(0) position: vec3<f32>, @location(1) uv: vec2<f32>) -> Output {
    var out: Output;
    out.clip = frame.clip * model * vec4<f32>(position, 1.0);
    out.uv = uv;
    return out;
}
@fragment fn fragment(input: Output) -> @location(0) vec4<f32> {
    if input.clip.z > textureLoad(scene_depth, vec2<i32>(input.clip.xy), 0) { discard; }
    let texel = textureSample(impostor, impostor_sampler, input.uv);
    return vec4<f32>(texel.rgb * texel.a, texel.a);
}
