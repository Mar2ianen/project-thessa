// App-owned instrument, not a scene render or an authoritative attitude model.
// One native-resolution quad in the existing UI pass, no intermediate target.
struct Frame { north:vec4<f32>, east:vec4<f32>, up:vec4<f32> }
@group(0) @binding(0) var<uniform> frame:Frame;
struct Output { @builtin(position) position:vec4<f32>, @location(0) disk:vec2<f32> }
@vertex fn vertex(@builtin(vertex_index) i:u32)->Output {
    let corners=array(vec2(-1.0,-1.0),vec2(1.0,-1.0),vec2(-1.0,1.0),vec2(-1.0,1.0),vec2(1.0,-1.0),vec2(1.0,1.0));
    let p=corners[i];return Output(vec4(p,0.0,1.0),vec2(p.x,-p.y));
}
// GRID_IMPLEMENTATION
fn linear_from_srgb(c:vec3<f32>)->vec3<f32>{
    return select(c/12.92,pow((c+0.055)/1.055,vec3(2.4)),c>vec3(0.04045));
}
@fragment fn fragment(input:Output)->@location(0) vec4<f32>{
    let r2=dot(input.disk,input.disk);
    let direction=vec3(sqrt(max(1.0-r2,0.000001)),-input.disk.x,-input.disk.y);
    let local=vec3(dot(direction,frame.north.xyz),dot(direction,frame.east.xyz),dot(direction,frame.up.xyz));
    let latitude=asin(clamp(local.z,-1.0,1.0));
    // Longitude is undefined at a pole, but storage/colour must stay finite.
    let longitude=atan2(local.y,select(local.x,1e-8,dot(local.xy,local.xy)<1e-16));
    let uv=vec2(longitude/6.28318530718+0.5,latitude/3.14159265359+0.5);
    // atan2's branch cut is not a giant texture footprint. Use wrapped
    // longitude derivatives in both analytic AA and baked mip selection.
    let dx=dpdx(longitude);let dy=dpdy(longitude);
    let gx=vec2(dx-6.28318530718*round(dx/6.28318530718),dpdx(latitude))/vec2(6.28318530718,3.14159265359);
    let gy=vec2(dy-6.28318530718*round(dy/6.28318530718),dpdy(latitude))/vec2(6.28318530718,3.14159265359);
    let width=max(fwidth(latitude),0.0015);
    let grid=grid_coverage(uv,gx,gy);
    let horizon=1.0-smoothstep(width*0.5,width*1.5,abs(latitude));
    let sky=smoothstep(-width,width,latitude);
    let rocket=frame.north.w>0.5;
    let ground=select(vec3(0.39,0.20,0.075),vec3(0.035,0.045,0.055),rocket);
    let upper=select(vec3(0.055,0.32,0.53),vec3(0.87,0.87,0.82),rocket);
    var color=mix(ground,upper,sky);
    let ink=select(vec3(0.90,0.93,0.87),mix(vec3(0.88),vec3(0.08),sky),rocket);
    color=mix(color,ink,grid*0.75);
    color=mix(color,vec3(0.98,0.64,0.20),horizon);
    color*=0.70+0.30*sqrt(max(1.0-r2,0.0));
    let alpha=1.0-smoothstep(1.0-fwidth(r2),1.0,r2);
    if(r2>=1.0){discard;}
    // Egui blends premultiplied alpha. Convert authored instrument colours
    // only for an sRGB target; unorm targets follow egui's gamma convention.
    if(TARGET_SRGB){color=linear_from_srgb(color);}
    return vec4(color*alpha,alpha);
}
