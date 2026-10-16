// Group is selected by the opaque consumer; uniforms are camera-relative.
struct ShadowFrame { matrices:array<mat4x4<f32>,12>, radii:vec4<f32>, config:vec4<f32> }
@group(SHADOW_GROUP) @binding(0) var<uniform> shadows:ShadowFrame;
@group(SHADOW_GROUP) @binding(1) var shadow_depth:texture_depth_2d_array;
@group(SHADOW_GROUP) @binding(2) var shadow_sampler:sampler_comparison;
fn shadow_sample(p:vec3<f32>,n:vec3<f32>,sun:u32,cascade:u32)->f32 {
    let layer=sun*u32(shadows.config.x)+cascade;
    let clip=shadows.matrices[layer]*vec4(p+n*shadows.config.y,1.0);
    let q=clip.xyz/clip.w;
    if (any(abs(q.xy)>vec2(1.0)) || q.z<=0.0 || q.z>=1.0) { return 1.0; }
    let uv=q.xy*vec2(0.5,-0.5)+vec2(0.5);
    let radius=i32(shadows.config.z);
    var sum=0.0; var count=0.0;
    for (var y=-radius;y<=radius;y++) { for (var x=-radius;x<=radius;x++) {
        sum+=textureSampleCompareLevel(shadow_depth,shadow_sampler,
            uv+vec2(f32(x),f32(y))*shadows.config.w,i32(layer),q.z);
        count+=1.0;
    } }
    return sum/count;
}
fn shadow_visibility(p:vec3<f32>,n:vec3<f32>,sun:u32)->f32 {
    let distance=length(p);
    let count=u32(shadows.config.x);
    for (var i=0u;i<count;i++) {
        let far=shadows.radii[i];
        if (distance<=far) {
            let a=shadow_sample(p,n,sun,i);
            var b=1.0;
            if (i+1u<count) { b=shadow_sample(p,n,sun,i+1u); }
            return mix(a,b,smoothstep(far*0.9,far,distance));
        }
    }
    return 1.0;
}
