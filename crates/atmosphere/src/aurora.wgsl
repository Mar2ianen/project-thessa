// MIT. Portable mirror of emission.rs and finite-segment scattering transport.
// No renderer bindings or invented magnetic/weather state in the semantic model.
struct AuroraField {
    shell:vec4<f32>, // body radius, min/max altitude, steps (zero = disabled)
    pole_latitude:vec4<f32>, shape:vec4<f32>, // width, activity, phases 3/7
    bottom:vec4<f32>, top:vec4<f32>,
    optical:vec4<f32>, // outer radius, Rayleigh/Mie heights, absorption count
    rayleigh:vec4<f32>, mie:vec4<f32>, absorption:array<vec4<f32>,4>
}
fn aurora_sphere(o:vec3<f32>,d:vec3<f32>,radius:f32)->vec2<f32> {
    let b=dot(o,d);let r=length(o);let disc=b*b-(r-radius)*(r+radius);
    if(disc<0.0){return vec2(1.0,-1.0);}let root=sqrt(disc);return vec2(-b-root,-b+root);
}
fn aurora_emission_at(field:AuroraField,point:vec3<f32>)->vec3<f32> {
    let radius=length(point);let altitude=radius-field.shell.x;
    if(altitude<=field.shell.y || altitude>=field.shell.z){return vec3(0.0);}
    let direction=point/radius;let pole=normalize(field.pole_latitude.xyz);
    let latitude=abs(asin(clamp(dot(direction,pole),-1.0,1.0)));
    let oval=exp(-pow(latitude-field.pole_latitude.w,2.0)/(2.0*pow(max(0.001,field.shape.x),2.0)));
    if(oval<0.001){return vec3(0.0);}
    let helper=select(vec3(0.0,0.0,1.0),vec3(0.0,1.0,0.0),abs(pole.y)<0.9);
    let right=normalize(cross(helper,pole));let forward=cross(pole,right);
    let horizontal=vec2(dot(direction,right),dot(direction,forward));var longitude=0.0;
    if(dot(horizontal,horizontal)>1e-12){longitude=atan2(horizontal.y,horizontal.x);}
    let curtain=max(0.0,0.6+0.25*sin(3.0*longitude+field.shape.z)+0.15*sin(7.0*longitude+field.shape.w));
    let fraction=(altitude-field.shell.y)/(field.shell.z-field.shell.y);
    let intensity=oval*sin(fraction*3.141592653589793)*field.shape.y*curtain;
    return max(vec3(0.0),mix(field.bottom.rgb,field.top.rgb,fraction)*intensity);
}
fn aurora_sigma(field:AuroraField,point:vec3<f32>)->vec3<f32> {
    let radius=length(point);if(radius>field.optical.x){return vec3(0.0);}
    let altitude=max(0.0,radius-field.shell.x);
    var sigma=field.rayleigh.rgb*exp(-altitude/field.optical.y)+field.mie.rgb*exp(-altitude/field.optical.z);
    for(var i=0u;i<u32(field.optical.w);i++){sigma+=field.absorption[i].rgb*exp(-altitude/field.absorption[i].w);}
    return sigma;
}
fn aurora_transmittance(field:AuroraField,origin:vec3<f32>,direction:vec3<f32>,distance:f32)->vec3<f32> {
    let shell=aurora_sphere(origin,direction,field.optical.x);let a=max(0.0,shell.x);let b=min(distance,shell.y);
    if(b<=a){return vec3(1.0);}
    let count=u32(field.shell.w);let step=(b-a)/f32(count);var tau=vec3(0.0);
    for(var i=0u;i<count;i++){tau+=min(vec3(10.0),aurora_sigma(field,origin+direction*(a+(f32(i)+0.5)*step))*step);}
    return exp(-min(tau,vec3(50.0)));
}
