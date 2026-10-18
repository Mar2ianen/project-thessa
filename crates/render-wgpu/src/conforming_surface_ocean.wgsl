// Optional diagnostic consumer of the existing field/material residency.
// One global near-field band limit keeps duplicate vertices identical even
// across mixed-depth bisectors. Page level is NEVER geometric resolution.
struct SurfaceVertex { position_height:vec4<f32>, normal_face:vec4<f32>, address:vec4<u32> }
struct OceanGeometryPolicy { distance:f32, triangle_budget:u32, unused:vec2<u32> }
struct OceanGeometryExtent { footprint:atomic<u32>, candidates:atomic<u32>, unused:vec2<u32> }
@group(0) @binding(0) var<storage,read_write> surface_vertices:array<SurfaceVertex>;
@group(0) @binding(1) var<storage,read_write> extent:OceanGeometryExtent;
@group(0) @binding(2) var<uniform> policy:OceanGeometryPolicy;
@group(0) @binding(3) var material_texture:texture_2d_array<f32>;
@group(0) @binding(4) var material_sampler:sampler;

fn live(v:SurfaceVertex)->bool {
    return v.address.z!=0xffffffffu && (any(v.position_height!=vec4(0.0))
        ||any(v.normal_face!=vec4(0.0))||any(v.address!=vec4(0u)));
}

@compute @workgroup_size(64)
fn measure_ocean_extent(@builtin(global_invocation_id) gid:vec3<u32>) {
    let slot=gid.x;
    if(slot>=arrayLength(&surface_vertices)/3u){return;}
    let a=surface_vertices[slot*3u];let b=surface_vertices[slot*3u+1u];let c=surface_vertices[slot*3u+2u];
    if(!live(a)||!live(b)||!live(c)){return;}
    let p=a.position_height.xyz;let q=b.position_height.xyz;let r=c.position_height.xyz;
    let center=(p+q+r)/3.0;
    let radius=max(length(p-center),max(length(q-center),length(r-center)));
    // The entire triangle is inside this sphere; admit straddling triangles
    // even if all vertices lie outside the near-field distance.
    if(length(center)-radius>policy.distance){return;}
    let longest=max(length(p-q),max(length(q-r),length(r-p)));
    // Positive f32 bit patterns have the same order as their values.
    atomicMax(&extent.footprint,bitcast<u32>(longest*2.0));
    atomicAdd(&extent.candidates,1u);
}

@compute @workgroup_size(64)
fn deform_conforming_ocean(@builtin(global_invocation_id) gid:vec3<u32>) {
    let vertex=gid.x;
    if(vertex>=arrayLength(&surface_vertices)){return;}
    if(atomicLoad(&extent.candidates)>policy.triangle_budget){return;}
    let v=surface_vertices[vertex];
    if(!live(v)||v.position_height.w>=1.0){return;}
    let relative=v.position_height.xyz;
    let taper=1.0-smoothstep(policy.distance*0.5,policy.distance,length(relative));
    if(taper<=0.0){return;}
    // Face ownership and the phase position come from canonical emission, so
    // every instance of a shared vertex takes exactly the same sampling path.
    let uv=vec2<f32>(v.address.xy)/536870912.0;
    let radial=material_direction(uv,u32(v.normal_face.w));
    let sample=material_at_level(material_address(radial),vec2(0.0),vec2(0.0),17u);
    let roughness=sample.value.a+1.0-sample.coverage;
    let weight=ocean_material_weight(v.position_height.w,roughness)*taper;
    if(weight<=0.0){return;}
    let footprint=bitcast<f32>(atomicLoad(&extent.footprint));
    // Coarse geometry stays optical-only. Avoid a
    // tiny nonzero texel DC residual becoming coherent coarse vertex motion.
    if(footprint>=ocean_frame.periods.x){return;}
    let wave=ocean_spectral(relative,radial,footprint);
    // Preserve datum height, normal, and canonical address for land/optical
    // classification. Re-emission precedes every deformation: no accumulation.
    surface_vertices[vertex].position_height=vec4(relative+radial*wave.height*weight,v.position_height.w);
}
