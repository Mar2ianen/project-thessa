// Portable native consumers. Optical primitives mirror thessa-atmosphere's
// slab/Beer-Lambert/dual-lobe contract; the weather texture is its shared bake.
struct View { clip:mat4x4<f32>, unjittered:mat4x4<f32>, inverse_clip:mat4x4<f32>,
    world_from_view:mat4x4<f32>, view_from_world:mat4x4<f32>, projection:mat4x4<f32>, inverse_projection:mat4x4<f32>, camera:vec3<f32>, exposure:f32 }
struct Frame { camera_radius:vec4<f32>, deck:vec4<f32>, wind:vec4<f32>, viewport:vec4<f32>,
    directions:array<vec4<f32>,3>, colors:array<vec4<f32>,3>, ambient:vec4<f32> }
struct Star { direction:vec4<f32>, flux:vec4<f32> }
@group(0) @binding(0) var<uniform> view:View;
@group(0) @binding(1) var<uniform> frame:Frame;
@group(0) @binding(4) var scene_depth:texture_depth_2d;
@group(0) @binding(8) var<storage,read> stars:array<Star>;
@group(0) @binding(9) var<uniform> aurora_field:AuroraField;
const PI:f32=3.141592653589793;
struct StarVertex { @builtin(position) position:vec4<f32>, @location(0) point:vec2<f32>, @location(1) radiance:vec3<f32>, @location(2) @interpolate(flat) disc:u32 }
@vertex fn star_vertex(@builtin(vertex_index) vertex:u32,@builtin(instance_index) instance:u32)->StarVertex {
    let corners=array<vec2<f32>,6>(vec2(-1.0,-1.0),vec2(1.0,-1.0),vec2(1.0,1.0),vec2(-1.0,-1.0),vec2(1.0,1.0),vec2(-1.0,1.0));
    let s=stars[instance];let clip=view.clip*vec4(s.direction.xyz,0.0);var out:StarVertex;
    out.position=vec4(2.0,2.0,2.0,1.0);out.point=corners[vertex];out.radiance=vec3(0.0);out.disc=0u;
    if(clip.w<=0.0){return out;}
    let alpha=s.direction.w;
    if(alpha>0.0 && abs(view.projection[1][1])*frame.viewport.y*tan(alpha)/(2.0*clip.w)>1.0) {
        let helper=select(vec3(0.0,1.0,0.0),vec3(1.0,0.0,0.0),abs(s.direction.y)>0.9);
        let right=normalize(cross(s.direction.xyz,helper));let up=cross(right,s.direction.xyz);
        let vertex_direction=s.direction.xyz+tan(alpha)*(right*corners[vertex].x+up*corners[vertex].y);
        let projected=view.clip*vec4(vertex_direction,0.0);
        out.position=vec4(projected.xy,projected.w,projected.w);out.disc=1u;
        out.radiance=s.flux.rgb/(PI*sin(alpha)*sin(alpha))*view.exposure;return out;
    }
    out.position=vec4(clip.xy+corners[vertex]*6.0/frame.viewport.xy*clip.w,clip.w,clip.w);
    let cosine=max(0.0001,clip.w);
    let solid_angle=4.0*cosine*cosine*cosine/(frame.viewport.x*frame.viewport.y*abs(view.projection[0][0]*view.projection[1][1]));
    out.radiance=s.flux.rgb/max(solid_angle,1e-12)*view.exposure;
    return out;
}
@fragment fn star_fragment(in:StarVertex)->@location(0) vec4<f32> {
    let r=length(in.point);let edge=max(fwidth(r),0.001);
    if(in.disc!=0u){return vec4(in.radiance*(1.0-smoothstep(1.0-edge,1.0+edge,r)),0.0);}
    return vec4(in.radiance*exp(-4.5*dot(in.point,in.point))/(2.0*PI),0.0);
}
struct Pixel { @builtin(position) position:vec4<f32>, @location(0) ndc:vec2<f32> }
@vertex fn volume_vertex(@builtin(vertex_index) i:u32)->Pixel {
    let p=vec2(f32((i<<1u)&2u),f32(i&2u))*2.0-1.0;var out:Pixel;out.position=vec4(p,0.0,1.0);out.ndc=p;return out;
}
fn aurora_primary(direction:vec3<f32>,distance:f32)->vec3<f32> {
    if(aurora_field.shell.w==0.0){return vec3(0.0);}
    let origin=cloud_frame.camera_radius.xyz;
    let stop=cloud_planet_stop(origin,direction,distance);
    let outer=aurora_sphere(origin,direction,aurora_field.shell.x+aurora_field.shell.z);
    let inner=aurora_sphere(origin,direction,aurora_field.shell.x+aurora_field.shell.y);
    var pieces=array<vec2<f32>,2>(vec2(max(0.0,outer.x),outer.y),vec2(0.0));
    if(inner.y>inner.x){pieces[0].y=min(outer.y,inner.x);pieces[1]=vec2(max(0.0,max(outer.x,inner.y)),outer.y);}
    var radiance=vec3(0.0);let count=u32(aurora_field.shell.w);
    let decks=u32(cloud_frame.wind.y);
    let last_deck=max(1u,decks)-1u;
    let cloud_top=cloud_frame.deck.x+f32(last_deck)*6000.0+cloud_frame.deck.y/select(1.0,3.0,last_deck!=0u);
    // If the emitting shell is wholly above all cloud decks, no cloud density
    // can occur inside either emission interval. Its prefix transmission is
    // constant on that interval, including the far piece on an orbital ray.
    let disjoint=decks==0u || cloud_top<=aurora_field.shell.y;
    for(var segment=0u;segment<2u;segment++) {
        let a=pieces[segment].x;let b=min(stop,pieces[segment].y);if(b<=a){continue;}
        let dt=(b-a)/f32(count);var transmission=aurora_transmittance(aurora_field,origin,direction,a);
        var cloud_prefix=1.0;
        if(disjoint){cloud_prefix=cloud_transmittance_to(vec3(0.0),direction,a);}
        for(var i=0u;i<count;i++) {
            let t=a+(f32(i)+0.5)*dt;let point=origin+direction*t;
            let emission=aurora_emission_at(aurora_field,point);let absorb=exp(-aurora_sigma(aurora_field,point)*dt);
            if(any(emission>vec3(0.0))){
                var cloud_t=cloud_prefix;
                if(!disjoint){cloud_t=cloud_transmittance_to(vec3(0.0),direction,t);}
                radiance+=emission*transmission*sqrt(absorb)*dt*cloud_t;
            }
            transmission*=absorb;
        }
    }
    return radiance*aurora_field.bottom.w;
}
@fragment fn cloud_fragment(in:Pixel)->@location(0) vec4<f32> {
    let p=view.inverse_projection*vec4(in.ndc,0.5,1.0);
    let direction=normalize((view.world_from_view*vec4(p.xyz/p.w,0.0)).xyz);
    var stop=1e30;
    let depth=textureLoad(scene_depth,vec2<i32>(in.position.xy),0);
    if(depth<1.0){
        if(view.projection[2][3]==-1.0 && view.projection[3][3]==0.0){
            // Invert the actual narrowed perspective coefficients. Applying
            // inverse_clip at orbital forward-Z depth loses extra bits to the
            // cancellation in its homogeneous w, beyond depth quantization.
            let ray=p.xyz/p.w;
            stop=abs(view.projection[3][2]/(depth+view.projection[2][2]))*length(ray)/abs(ray.z);
        }else{let hit=view.inverse_clip*vec4(in.ndc,depth,1.0);stop=length(hit.xyz/hit.w);}
    }
    let column=cloud_transport(vec3(0.0),direction,stop);
    // The background supplies clear sky only. Each emission sample receives
    // extinction up to its own distance, never from a cloud behind the emitter.
    return vec4((column.rgb+aurora_primary(direction,stop))*view.exposure,1.0-column.a);
}
