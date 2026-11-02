// Solar extinction lookup from the same neutral optical model as the sky.
// Four table reads replace integration along each stellar ray.
fn env_sunlight_at(height_m:f32,up:vec3<f32>,direction:vec3<f32>,source:vec3<f32>)->vec3<f32> {
    if(env_meta.sun_table.w==0u || !any(source>vec3(0.0))){return source;}
    let radius=env_meta.sun_domain.x;
    let top=env_meta.sun_domain.y;
    var height=max(height_m,0.0);
    var mu=clamp(dot(up,direction),-1.0,1.0);
    if(height>top){
        if(mu>=0.0){return source;}
        // Cross product retains precision near a radial ray at orbital height.
        let impact=(radius+height)*length(cross(up,direction));
        if(impact<radius){return vec3(0.0);}
        if(impact>=radius+top){return source;}
        height=top;
        mu=-sqrt(max(0.0,1.0-pow(impact/(radius+top),2.0)));
    }
    let rho=sqrt(height*(2.0*radius+height));
    let r=radius+height;
    if(mu < -rho/r){return vec3(0.0);}
    let horizon=sqrt(top*(2.0*radius+top));
    let d_min=top-height;
    let d_max=rho+horizon;
    let delta=(top-height)*(2.0*radius+top+height);
    let r_mu=r*mu;
    let root=sqrt(max(0.0,r_mu*r_mu+delta));
    var distance=-r_mu+root;
    if(r_mu>=0.0 && root+r_mu>0.0){distance=delta/(root+r_mu);}
    let extent=env_meta.sun_table.yz;
    let coordinate=clamp(vec2((distance-d_min)/(d_max-d_min),rho/horizon),vec2(0.0),vec2(1.0))*vec2<f32>(extent-vec2(1u));
    let lo=vec2<u32>(floor(coordinate));
    let hi=min(lo+vec2(1u),extent-vec2(1u));
    let weight=fract(coordinate);
    let offset=env_meta.sun_table.x;
    let a=mix(env_texels[offset+lo.y*extent.x+lo.x].rgb,env_texels[offset+lo.y*extent.x+hi.x].rgb,weight.x);
    let b=mix(env_texels[offset+hi.y*extent.x+lo.x].rgb,env_texels[offset+hi.y*extent.x+hi.x].rgb,weight.x);
    return source*mix(a,b,weight.y);
}
fn env_sunlight(position:vec3<f32>,direction:vec3<f32>,source:vec3<f32>)->vec3<f32> {
    if(env_meta.sun_table.w==0u){return source;}
    let r0=env_meta.sun_domain.x+env_meta.sun_domain.z;
    let up=env_meta.sun_up.xyz;
    let delta=2.0*r0*dot(position,up)+dot(position,position);
    let radius=sqrt(max(0.0,r0*r0+delta));
    let height=env_meta.sun_domain.z+delta/(radius+r0);
    let vector=up+position/r0;
    let radial=select(up,vector/max(length(vector),1e-20),dot(vector,vector)>1e-20);
    return env_sunlight_at(height,radial,direction,source);
}
