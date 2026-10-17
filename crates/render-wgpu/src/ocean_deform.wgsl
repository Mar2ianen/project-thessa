// Operates only on admitted near-field slots, in the existing generated pool.
// The immutable backup is restored before any new height/source publication.
struct DeformParams { count:u32, radius:f32, distance:f32, unused:f32 }
@group(0) @binding(0) var<storage,read> original:array<vec4<f32>>;
@group(0) @binding(1) var<storage,read_write> vertices:array<vec4<f32>>;
@group(0) @binding(2) var<storage,read> frames:array<CbtTileFrame>;
@group(0) @binding(3) var<storage,read> slots:array<u32>;
@group(0) @binding(4) var<uniform> deform:DeformParams;
@group(0) @binding(5) var material_texture:texture_2d_array<f32>;
@group(0) @binding(6) var material_sampler:sampler;
@compute @workgroup_size(64)
fn deform_ocean(@builtin(global_invocation_id) gid:vec3<u32>){
    if(gid.x>=deform.count*1089u){return;}
    let index=gid.x;let ordinal=slots[index/1089u];let base=ordinal*1089u+index%1089u;
    let p=original[index*2u];let n=original[index*2u+1u];let frame=frames[ordinal];
    if(n.w>=1.0){return;}
    let radial=normalize(frame.geometry.normal.xyz*deform.radius+p.xyz);
    let relative=frame.anchor_hi_m.xyz+(frame.anchor_lo_m.xyz+p.xyz);
    let sample=material_at_level(material_address(radial),vec2(0.0),vec2(0.0),17u);
    let roughness=sample.value.a+1.0-sample.coverage;
    let weight=ocean_material_weight(n.w,roughness)
        *(1.0-smoothstep(deform.distance*0.5,deform.distance,length(relative)));
    if(weight<=0.0){return;}
    // Keep sub-cell waves in shading, not spatially aliased vertex motion.
    let cell_m=2.0*deform.radius*frame.geometry.radius_half_extent_len.y/32.0;
    let wave=ocean_spectral(relative,radial,cell_m*2.0);
    vertices[base*2u]=vec4(p.xyz+radial*wave.height*weight,p.w);
    // Geometry normals remain the datum/base normals. Fragment and reflected
    // hit shading replace only the water normal with the full optical field;
    // adding the gradient here would double-apply it.
    vertices[base*2u+1u]=n;
}
