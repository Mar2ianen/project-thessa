// Cosine-weighted screen-space visibility. Off-screen/unknown samples remain
// unoccluded. Single-layer depth cannot resolve hidden or subpixel blockers.
struct Params { clip:mat4x4<f32>, inverse_clip:mat4x4<f32>, controls:vec4<f32> }
@group(0) @binding(0) var<uniform> params:Params;
@group(0) @binding(1) var scene_depth:texture_depth_2d;
@group(0) @binding(2) var diffuse_sky:texture_2d<f32>;
// MSAA_BINDING
fn position(uv:vec2<f32>,depth:f32)->vec3<f32> {
    let h=params.inverse_clip*vec4(uv*vec2(2.0,-2.0)+vec2(-1.0,1.0),depth,1.0);return h.xyz/h.w;
}
fn at(pixel:vec2<i32>)->vec3<f32> {
    let p=clamp(pixel,vec2(0),vec2<i32>(params.controls.zw)-1);
    return position((vec2<f32>(p)+0.5)/params.controls.zw,textureLoad(scene_depth,p,0));
}
fn radical_inverse(input:u32)->f32 {
    var v=input;v=(v<<16u)|(v>>16u);v=((v&0x55555555u)<<1u)|((v&0xAAAAAAAAu)>>1u);
    v=((v&0x33333333u)<<2u)|((v&0xCCCCCCCCu)>>2u);v=((v&0x0F0F0F0Fu)<<4u)|((v&0xF0F0F0F0u)>>4u);
    v=((v&0x00FF00FFu)<<8u)|((v&0xFF00FF00u)>>8u);return f32(v)*2.3283064365386963e-10;
}
@vertex fn vertex(@builtin(vertex_index) i:u32)->@builtin(position) vec4<f32> {
    let p=array<vec2<f32>,3>(vec2(-1.0,-1.0),vec2(3.0,-1.0),vec2(-1.0,3.0));return vec4(p[i],0.0,1.0);
}
@fragment fn fragment(@builtin(position) screen:vec4<f32>)->@location(0) vec4<f32> {
    let pixel=vec2<i32>(screen.xy);let receiver=textureLoad(diffuse_sky,pixel,0);let ambient=receiver.rgb;
    let depth=textureLoad(scene_depth,pixel,0);
    if(params.controls.y==0.0||depth>=1.0||all(ambient==vec3(0.0))){return vec4(0.0);}
    let uv=screen.xy/params.controls.zw;let p=position(uv,depth);
    let l=p-at(pixel-vec2(1,0));let r=at(pixel+vec2(1,0))-p;
    let t=p-at(pixel-vec2(0,1));let b=at(pixel+vec2(0,1))-p;
    let dx=select(l,r,dot(r,r)<dot(l,l));let dy=select(t,b,dot(b,b)<dot(t,t));
    let plane=cross(dx,dy);if(dot(plane,plane)<1e-16){return vec4(0.0);}
    let pixel_span=max(length(dx),length(dy));
    // COVERAGE_CHECK
    var n=normalize(plane);n*=select(-1.0,1.0,dot(n,-p)>=0.0);
    let axis=select(vec3(1.0,0.0,0.0),vec3(0.0,1.0,0.0),abs(n.y)<0.9);
    let tangent=normalize(cross(axis,n));let bitangent=cross(n,tangent);
    var blocked=0.0;let radius=params.controls.x;
    // A finite, numerical self-intersection offset, independent of source light.
    let bias=max(0.002,8.0*1.1920928955078125e-7*length(p));
    let directions=select(8u,4u,SAMPLE_COUNTu==8u);
    let steps=SAMPLE_COUNTu/directions;
    for(var i=0u;i<directions;i++) {
        let u=(f32(i)+0.5)/f32(directions);let phi=6.28318530718*radical_inverse(i);
        let direction=tangent*(sqrt(u)*cos(phi))+bitangent*(sqrt(u)*sin(phi))+n*sqrt(1.0-u);
        var ray_blocked=0.0;
        for(var step=1u;step<=steps;step++) {
            let distance=radius*f32(step)/f32(steps);
            let probe=p+n*bias+direction*distance;
            let clip=params.clip*vec4(probe,1.0);if(clip.w<=0.0){continue;}
            let ndc=clip.xyz/clip.w;let q_uv=ndc.xy*vec2(0.5,-0.5)+0.5;
            if(any(q_uv<vec2(0.0))||any(q_uv>=vec2(1.0))||ndc.z<=0.0||ndc.z>=1.0){continue;}
            let q_pixel=vec2<i32>(q_uv*params.controls.zw);let q_depth=textureLoad(scene_depth,q_pixel,0);
            if(q_depth>=1.0||q_depth>=ndc.z){continue;}
            let q=position((vec2<f32>(q_pixel)+0.5)/params.controls.zw,q_depth);let delta=q-p;
            if(dot(delta,n)>bias && dot(delta,delta)<radius*radius){
                ray_blocked=max(ray_blocked,1.0-dot(delta,delta)/(radius*radius));
            }
        }
        blocked+=ray_blocked;
    }
    // Conservative union with baked visibility: unknown correlation cannot
    // justify applying the same geometry's attenuation a second time.
    let visible=1.0-blocked/f32(directions);
    let baked=clamp(receiver.a,0.0,1.0);
    let correction=max(0.0,baked-visible)/max(baked,1e-6);
    return vec4(ambient*correction,0.0);
}
