fn cbt_spectral_ocean(position:vec3<f32>,normal:vec3<f32>,direction:vec3<f32>,height:f32,roughness:f32,albedo:vec3<f32>,footprint:f32)->vec4<f32>{
    let weight=ocean_material_weight(height,roughness);
    if(weight<=0.0){return vec4(0.0);}
    let wave=ocean_spectral(position,direction,footprint);
    let n=normalize(direction-wave.gradient);
    let eye=ocean_safe_normalize(view.world_position-position,normal);
    let f0=ocean_frame.optics.x;
    let fresnel=ocean_schlick(f0,max(dot(n,eye),0.0));
    // Slope variance is an optical lobe width, not destroyed wave energy.
    let alpha2=pow(ocean_frame.optics.y,4.0)+wave.variance;
    let optical_roughness=clamp(pow(alpha2,0.25),0.02,1.0);
    var illumination=env_ambient_lux(lighting.ambient_lux.rgb);
    var reflection=vec3(0.0);
    for(var i=0u;i<3u;i++){
        let visibility=cbt_shadow_visibility(position,direction,i);
        illumination+=lighting.colors_lux[i].rgb*max(dot(n,lighting.directions[i].xyz),0.0)*visibility;
        if(cbt_specular_enabled()) {reflection+=ocean_sun_reflection(n,eye,lighting.directions[i].xyz,lighting.colors_lux[i].rgb,optical_roughness,f0)*visibility;}
    }
    if(!cbt_specular_enabled()){return vec4(albedo*illumination/OCEAN_PI,weight);}
    reflection+=env_radiance(reflect(-eye,n),lighting.ambient_lux.rgb)*fresnel;
    // The existing ocean material supplies the diffuse/subsurface colour;
    // no new absorption or bathymetry is invented by this render-only slice.
    return vec4(albedo*illumination/OCEAN_PI*(1.0-fresnel)+reflection,weight);
}
