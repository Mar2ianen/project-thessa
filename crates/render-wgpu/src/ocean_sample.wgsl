// Body-fixed periodic coordinates: CPU f64 camera residues avoid reconstructing
// planet-sized f32 positions. Independent fields blend across all cube faces.
struct OceanFrame { camera_mod:array<vec4<f32>,3>, periods:vec4<f32>, omitted:vec4<f32>, optics:vec4<f32> }
@group(2) @binding(0) var<uniform> ocean_frame:OceanFrame;
@group(2) @binding(1) var ocean_field:texture_2d_array<f32>;
@group(2) @binding(2) var ocean_sampler:sampler;
struct OceanSample { height:f32, gradient:vec3<f32>, variance:f32 }
fn ocean_spectral(position:vec3<f32>,radial:vec3<f32>,footprint_m:f32)->OceanSample{
    let raw=pow(abs(radial),vec3(4.0));let denominator=max(length(raw),0.00001);let w=raw/denominator;
    var result=OceanSample(0.0,vec3(0.0),dot(w*w,ocean_frame.omitted.xyz));
    for(var band=0u;band<3u;band++){
        let period=ocean_frame.periods[band];
        let p=(position+ocean_frame.camera_mod[band].xyz)/period+vec3(0.5/ocean_frame.periods.w);
        let lod=max(0.0,log2(max(footprint_m,0.0001)*ocean_frame.periods.w/period));
        let x=textureSampleLevel(ocean_field,ocean_sampler,p.yz,i32(band),lod);
        let y=textureSampleLevel(ocean_field,ocean_sampler,p.xz,i32(3u+band),lod);
        let z=textureSampleLevel(ocean_field,ocean_sampler,p.xy,i32(6u+band),lod);
        result.height+=dot(w,vec3(x.x,y.x,z.x));
        result.gradient+=w.x*vec3(0.0,x.yz)+w.y*vec3(y.y,0.0,y.z)+w.z*vec3(z.yz,0.0);
        // Derivatives of the smooth planet-frame blend, not just its fields.
        let cube=radial*radial*radial;
        let seventh=cube*raw;
        let heights=vec3(x.x,y.x,z.x);
        let blend_gradient=(4.0*cube*heights/denominator
            -4.0*seventh*dot(raw,heights)/(denominator*denominator*denominator))
            /max(ocean_frame.optics.z,1.0);
        result.gradient+=blend_gradient;
        let v=max(vec3(x.w/ocean_frame.optics.w-dot(x.yz,x.yz),y.w/ocean_frame.optics.w-dot(y.yz,y.yz),z.w/ocean_frame.optics.w-dot(z.yz,z.yz)),vec3(0.0));
        result.variance+=dot(w*w,v);
    }
    result.gradient-=radial*dot(radial,result.gradient);
    return result;
}
fn ocean_material_weight(height:f32,roughness:f32)->f32 {
    return (1.0-smoothstep(0.22,0.65,roughness))*(1.0-smoothstep(0.25,1.0,height));
}
