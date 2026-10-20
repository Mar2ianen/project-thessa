// One native density/transport implementation for primary, shadow and reflected rays.
struct CloudFrame { camera_radius:vec4<f32>, deck:vec4<f32>, wind:vec4<f32>, viewport:vec4<f32>, directions:array<vec4<f32>,3>, colors:array<vec4<f32>,3>, ambient:vec4<f32> }
@group(0) @binding(5) var<uniform> cloud_frame:CloudFrame;
@group(0) @binding(6) var cloud_weather:texture_2d<f32>;
@group(0) @binding(7) var cloud_sampler:sampler;
const CLOUD_PI:f32=3.141592653589793;
struct CloudSegment { range:vec2<f32>, bottom:f32, thickness:f32, layer:u32 }
fn cloud_sphere(o:vec3<f32>,d:vec3<f32>,r:f32)->vec2<f32> {
    let b=dot(o,d);let h=length(o);let discriminant=b*b-(h-r)*(h+r);
    if(discriminant<0.0){return vec2(1.0,-1.0);}let root=sqrt(discriminant);return vec2(-b-root,-b+root);
}
fn cloud_density(p:vec3<f32>,piece:CloudSegment)->f32 {
    let radius=length(p);let h=(radius-piece.bottom)/piece.thickness;if(h<=0.0 || h>=1.0){return 0.0;}
    let direction=p/radius;let phase=select(cloud_frame.wind.x,cloud_frame.viewport.z,piece.layer!=0u);
    let uv=vec2(atan2(direction.z,direction.x)/(2.0*CLOUD_PI)+phase,0.5-asin(clamp(direction.y,-1.0,1.0))/CLOUD_PI);
    return textureSampleLevel(cloud_weather,cloud_sampler,uv,0.0).a*pow(max(0.0,sin(CLOUD_PI*h)),0.75);
}
fn cloud_segments(origin:vec3<f32>,direction:vec3<f32>)->array<CloudSegment,4> {
    var pieces:array<CloudSegment,4>;
    for(var layer=0u;layer<u32(cloud_frame.wind.y);layer++) {
        let base=cloud_frame.deck.x+f32(layer)*6000.0;let thickness=cloud_frame.deck.y/select(1.0,3.0,layer!=0u);
        let bottom=cloud_frame.camera_radius.w+base;
        let outer=cloud_sphere(origin,direction,bottom+thickness);let inner=cloud_sphere(origin,direction,bottom);
        var segments=array<vec2<f32>,2>(vec2(max(0.0,outer.x),outer.y),vec2(0.0));
        if(inner.y>inner.x){segments[0].y=min(outer.y,inner.x);segments[1]=vec2(max(0.0,max(outer.x,inner.y)),outer.y);}
        for(var segment=0u;segment<2u;segment++) {pieces[layer*2u+segment]=CloudSegment(segments[segment],bottom,thickness,layer);}
    }
    // Hollow-shell pieces and multiple decks must be ordered even above/in a deck.
    for(var i=0u;i<4u;i++) {for(var j=i+1u;j<4u;j++) {if(pieces[j].range.x<pieces[i].range.x){let swap=pieces[i];pieces[i]=pieces[j];pieces[j]=swap;}}}
    return pieces;
}
fn cloud_planet_stop(origin:vec3<f32>,direction:vec3<f32>,stop:f32)->f32 {
    let body=cloud_sphere(origin,direction,cloud_frame.camera_radius.w);
    if(body.y>max(0.0,body.x)){return min(stop,max(0.0,body.x));}return stop;
}
fn cloud_optical_depth(origin:vec3<f32>,direction:vec3<f32>,taps:u32)->f32 {
    if(cloud_planet_stop(origin,direction,1e30)<1e30){return 1e30;}
    return cloud_column_tau(origin,direction,1e30,taps);
}
fn cloud_column_tau(origin:vec3<f32>,direction:vec3<f32>,distance:f32,taps:u32)->f32 {
    let pieces=cloud_segments(origin,direction);var tau=0.0;
    for(var segment=0u;segment<4u;segment++) {
        let piece=pieces[segment];let a=piece.range.x;let b=min(distance,piece.range.y);if(b<=a){continue;}
        let step=(b-a)/f32(taps);let extinction=cloud_frame.deck.z/select(1.0,3.0,piece.layer!=0u);
        for(var j=0u;j<taps;j++) {tau+=cloud_density(origin+direction*(a+(f32(j)+0.5)*step),piece)*extinction*step;}
    }return tau;
}
fn cloud_transmittance_to(position:vec3<f32>,direction:vec3<f32>,distance:f32)->f32 {
    if(cloud_frame.wind.y==0.0){return 1.0;}
    return exp(-cloud_column_tau(cloud_frame.camera_radius.xyz+position,direction,distance,max(1u,u32(cloud_frame.deck.w))));
}
fn cloud_sun_visibility(position:vec3<f32>,sun:u32)->f32 {
    if(cloud_frame.wind.y==0.0 || cloud_frame.wind.w==0.0){return 1.0;}
    return exp(-cloud_optical_depth(cloud_frame.camera_radius.xyz+position,normalize(cloud_frame.directions[sun].xyz),max(8u,u32(cloud_frame.deck.w))));
}
fn cloud_hg(g:f32,c:f32)->f32 {return (1.0-g*g)/(4.0*CLOUD_PI*pow(max(0.0001,1.0+g*g-2.0*g*c),1.5));}
fn cloud_source(point:vec3<f32>,direction:vec3<f32>,sun_steps:u32)->vec3<f32> {
    var source=cloud_frame.ambient.rgb/CLOUD_PI;
    for(var sun=0u;sun<3u;sun++) {
        if(all(cloud_frame.colors[sun].rgb==vec3(0.0))){continue;}
        let light=normalize(cloud_frame.directions[sun].xyz);
        if(cloud_planet_stop(point,light,1e30)<1e30){continue;}
        let taps=min(sun_steps,u32(cloud_frame.wind.z));var tau=0.0;if(taps>0u){tau=cloud_optical_depth(point,light,taps);}
        let c=dot(direction,light);let phase=0.7*cloud_hg(0.76,c)+0.3*cloud_hg(-0.30,c);
        source+=cloud_frame.colors[sun].rgb*exp(-tau)*phase;
    }
    return source;
}
// Low-cost reflected column: integrate density, hold the physical source at
// the segment midpoint, then solve constant-source Beer-Lambert analytically.
// This is a one-source quadrature, not a new scattering/opacity coefficient.
fn cloud_transport_column(position:vec3<f32>,direction:vec3<f32>,view_steps:u32,sun_steps:u32)->vec4<f32> {
    if(cloud_frame.wind.y==0.0){return vec4(0.0,0.0,0.0,1.0);}
    let origin=cloud_frame.camera_radius.xyz+position;let stop=cloud_planet_stop(origin,direction,1e30);
    let pieces=cloud_segments(origin,direction);var radiance=vec3(0.0);var transmission=1.0;
    for(var segment=0u;segment<4u;segment++) {
        let piece=pieces[segment];let a=piece.range.x;let b=min(stop,piece.range.y);if(b<=a){continue;}
        let count=max(1u,min(view_steps,u32(cloud_frame.deck.w)));let dt=(b-a)/f32(count);
        var density=0.0;
        for(var i=0u;i<count;i++){density+=cloud_density(origin+direction*(a+(f32(i)+0.5)*dt),piece);}
        let absorb=exp(-cloud_frame.deck.z/select(1.0,3.0,piece.layer!=0u)*density*dt);
        let source=cloud_source(origin+direction*((a+b)*0.5),direction,sun_steps);
        radiance+=transmission*(1.0-absorb)*source;transmission*=absorb;
        if(transmission<0.0001){break;}
    }
    return vec4(radiance,transmission);
}
// RGB = unexposed in-scatter, alpha = transmittance. All origins are relative to
// the current camera; the caller clips distance at its actual opaque hit.
fn cloud_transport(position:vec3<f32>,direction:vec3<f32>,distance:f32)->vec4<f32> {
    if(cloud_frame.wind.y==0.0){return vec4(0.0,0.0,0.0,1.0);}
    let origin=cloud_frame.camera_radius.xyz+position;let stop=cloud_planet_stop(origin,direction,distance);
    let pieces=cloud_segments(origin,direction);var radiance=vec3(0.0);var transmission=1.0;
    for(var segment=0u;segment<4u;segment++) {
        let piece=pieces[segment];let a=piece.range.x;let b=min(stop,piece.range.y);if(b<=a){continue;}
        let count=u32(cloud_frame.deck.w);let dt=(b-a)/f32(count);let extinction=cloud_frame.deck.z/select(1.0,3.0,piece.layer!=0u);
        for(var i=0u;i<count;i++) {
            let point=origin+direction*(a+(f32(i)+0.5)*dt);let d=cloud_density(point,piece);
            if(d<=0.0){continue;}let absorb=exp(-extinction*d*dt);let source=cloud_source(point,direction,u32(cloud_frame.wind.z));
            radiance+=transmission*(1.0-absorb)*source;transmission*=absorb;if(transmission<0.0001){break;}
        }
    }return vec4(radiance,transmission);
}
