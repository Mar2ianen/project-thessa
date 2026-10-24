// The same generated 33x33 grid as raster, without view-dependent skirts/LOD.
// Material addressing/filtering is the shared production material kernel.
@group(3) @binding(6) var<storage,read> rt_terrain_vertices:array<vec4<f32>>;
@group(3) @binding(8) var material_texture:texture_2d_array<f32>;
@group(3) @binding(9) var material_sampler:sampler;

fn rt_terrain_surface(hit:RtHit,pair:RtPair,ray_direction:vec3<f32>)->RtSurface {
    let slot=pair.words.x & 0x7fffffffu;
    var v0:vec4<f32>;var v1:vec4<f32>;var v2:vec4<f32>;
    var n0:vec4<f32>;var n1:vec4<f32>;var n2:vec4<f32>;
    var direction:vec3<f32>;
    let bary=vec3(1.0-hit.barycentrics.x-hit.barycentrics.y,hit.barycentrics);
    if(RT_TERRAIN_CONFORMING){
        let base=(slot*2048u+hit.triangle)*9u;
        v0=rt_terrain_vertices[base];v1=rt_terrain_vertices[base+3u];v2=rt_terrain_vertices[base+6u];
        n0=rt_terrain_vertices[base+1u];n1=rt_terrain_vertices[base+4u];n2=rt_terrain_vertices[base+7u];
        let d0=material_direction(vec2<f32>(bitcast<vec4<u32>>(rt_terrain_vertices[base+2u]).xy)/536870912.0,u32(n0.w));
        let d1=material_direction(vec2<f32>(bitcast<vec4<u32>>(rt_terrain_vertices[base+5u]).xy)/536870912.0,u32(n1.w));
        let d2=material_direction(vec2<f32>(bitcast<vec4<u32>>(rt_terrain_vertices[base+8u]).xy)/536870912.0,u32(n2.w));
        direction=normalize(d0*bary.x+d1*bary.y+d2*bary.z);
        n0.w=v0.w;n1.w=v1.w;n2.w=v2.w;
    }else{
    let cell=hit.triangle/2u;
    let a=(cell/32u)*33u+cell%32u;
    var corners=vec3(a,a+1u,a+33u);
    if ((hit.triangle&1u)!=0u) { corners=vec3(a+1u,a+34u,a+33u); }
    v0=rt_terrain_vertices[(slot*1089u+corners.x)*2u];
    v1=rt_terrain_vertices[(slot*1089u+corners.y)*2u];
    v2=rt_terrain_vertices[(slot*1089u+corners.z)*2u];
    n0=rt_terrain_vertices[(slot*1089u+corners.x)*2u+1u];
    n1=rt_terrain_vertices[(slot*1089u+corners.y)*2u+1u];
    n2=rt_terrain_vertices[(slot*1089u+corners.z)*2u+1u];
    let radial=bitcast<vec3<f32>>(pair.words.yzw);
    direction=normalize(radial*RT_TERRAIN_RADIUS+v0.xyz*bary.x+v1.xyz*bary.y+v2.xyz*bary.z);
    }
    let local=v0.xyz*bary.x+v1.xyz*bary.y+v2.xyz*bary.z;
    let geometric=normalize(cross(v1.xyz-v0.xyz,v2.xyz-v0.xyz));
    let geometry=select(geometric,-geometric,dot(geometric,ray_direction)>0.0);
    var normal=normalize(n0.xyz*bary.x+n1.xyz*bary.y+n2.xyz*bary.z);
    normal=select(normal,-normal,dot(normal,geometry)<0.0);
    // Explicit zero footprint matches mesh-hit LOD zero. Ray-cone filtering is
    // future work; no primary screen derivatives are used on secondary hits.
    let sample=material_at_level(material_address(direction),vec2(0.0),vec2(0.0),17u);
    // Same linear fallback globe/roughness as TerrainRenderer, not a new sky.
    let albedo=sample.value.rgb+RT_TERRAIN_FALLBACK*(1.0-sample.coverage);
    var roughness=sample.value.a+1.0-sample.coverage;
    let position=local+vec3(pair.row0.w,pair.row1.w,pair.row2.w);
    var dielectric=0.04;
    // OCEAN_HIT_HOOK
    return RtSurface(position,normal,geometry,albedo,0.0,roughness,1.0,dielectric,0.0);
}
