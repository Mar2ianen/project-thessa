// Already exposed once in terrain/mesh/sky. No second exposure or gamma pass.
override TONEMAP_OPERATOR: u32 = 0u; // 0 = Reinhard, 1 = Khronos PBR Neutral

@group(0) @binding(0) var scene:texture_2d<f32>;
@vertex fn vertex(@builtin(vertex_index) i:u32)->@builtin(position) vec4<f32>{
    let p=array<vec2<f32>,3>(vec2(-1.0,-1.0),vec2(3.0,-1.0),vec2(-1.0,3.0));
    return vec4(p[i],0.0,1.0);
}

// Adapted from KhronosGroup/ToneMapping; Copyright 2024 The Khronos Group, Inc.
// Apache-2.0 (LICENSES/Apache-2.0.txt). Modified for Rust/WGSL native HDR.
fn tonemap_pbr_neutral(color: vec3<f32>) -> vec3<f32> {
    let start_compression = 0.8 - 0.04;
    let desaturation = 0.15;

    let x = min(color.r, min(color.g, color.b));
    let offset = select(0.04, x - 6.25 * x * x, x < 0.08);
    var c = color - offset;

    let peak = max(c.r, max(c.g, c.b));
    if (peak < start_compression) {
        return c;
    }

    let d = 1.0 - start_compression;
    let new_peak = 1.0 - d * d / (peak + d - start_compression);
    c = c * (new_peak / peak);

    let g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
    return mix(c, vec3(new_peak), g);
}

@fragment fn fragment(@builtin(position) p:vec4<f32>)->@location(0) vec4<f32>{
    let source=textureLoad(scene,vec2<i32>(p.xy),0).rgb;
    // Defend display output from invalid and overflowing half-float highlights.
    let finite=select(vec3(0.0),source,source==source);
    let x=clamp(finite,vec3(0.0),vec3(65504.0));
    var color=x/(vec3(1.0)+x);
    if (TONEMAP_OPERATOR == 1u) {
        color=tonemap_pbr_neutral(x);
    }
    if !OUTPUT_IS_SRGB {
        // An sRGB attachment encodes once in hardware. Unorm surfaces need the
        // same transfer here; textureLoad never adds an extra source transfer.
        color=select(1.055*pow(color,vec3(1.0/2.4))-0.055,12.92*color,color<=vec3(0.0031308));
    }
    return vec4(color,1.0);
}
