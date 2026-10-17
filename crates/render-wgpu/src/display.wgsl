// Already exposed once in terrain/mesh/sky. No second exposure or gamma pass.
@group(0) @binding(0) var scene:texture_2d<f32>;
@vertex fn vertex(@builtin(vertex_index) i:u32)->@builtin(position) vec4<f32>{
    let p=array<vec2<f32>,3>(vec2(-1.0,-1.0),vec2(3.0,-1.0),vec2(-1.0,3.0));
    return vec4(p[i],0.0,1.0);
}
@fragment fn fragment(@builtin(position) p:vec4<f32>)->@location(0) vec4<f32>{
    let source=textureLoad(scene,vec2<i32>(p.xy),0).rgb;
    // Defend display output from invalid and overflowing half-float highlights.
    let finite=select(vec3(0.0),source,source==source);
    let x=clamp(finite,vec3(0.0),vec3(65504.0));
    var color=x/(vec3(1.0)+x);
    if !OUTPUT_IS_SRGB {
        // An sRGB attachment encodes once in hardware. Unorm surfaces need the
        // same transfer here; textureLoad never adds an extra source transfer.
        color=select(1.055*pow(color,vec3(1.0/2.4))-0.055,12.92*color,color<=vec3(0.0031308));
    }
    return vec4(color,1.0);
}
