struct RtMaterial {
    // rgb = linear base color, w = metallic.
    color: vec4<f32>,
    // x = roughness, y = normal scale, z = AO strength, w = diffuse transmission.
    factors: vec4<f32>,
    // Base colour, metallic/roughness, normal, AO; UINT_MAX means absent.
    textures: vec4<u32>,
    optical: vec4<f32>,
}

struct RtVertex { position:vec4<f32>, normal:vec4<f32>, uv:vec4<f32> }

struct RtRange {
    // x = first index into rt_indices, y = first vertex into rt_positions,
    // zw = bitcast base alpha factor and MASK cutoff.
    words: vec4<u32>,
}

struct RtPair {
    // x = primitive (range and material record), yzw unused.
    words: vec4<u32>,
    // Rows of the instance's camera-relative 3x4 transform, in the same
    // order the renderer builds them.
    row0: vec4<f32>,
    row1: vec4<f32>,
    row2: vec4<f32>,
}

@group(3) @binding(1) var<storage, read> rt_materials: array<RtMaterial>;
@group(3) @binding(2) var<storage, read> rt_ranges: array<RtRange>;
@group(3) @binding(3) var<storage, read> rt_vertices: array<RtVertex>;
@group(3) @binding(4) var<storage, read> rt_indices: array<u32>;
@group(3) @binding(5) var<storage, read> rt_pairs: array<RtPair>;

// Barycentrics land in the primitive's own asset-local space; the pair rows
// move the point and its normal into the camera-relative space the primary
// surface, the lights and the shadow atlas live in.
fn rt_vector(pair:RtPair, vector:vec3<f32>)->vec3<f32> {
    return vec3(dot(pair.row0.xyz,vector),dot(pair.row1.xyz,vector),dot(pair.row2.xyz,vector));
}

// Project a full pixel-width cone onto the actual hit triangle, including
// grazing incidence. No screen derivatives are evaluated inside ray traversal.
fn rt_hit_footprint(direction:vec3<f32>,geometry:vec3<f32>,width:f32)->mat2x3<f32> {
    let axis=select(vec3(0.0,1.0,0.0),vec3(1.0,0.0,0.0),abs(direction.y)>0.9);
    let x=normalize(cross(direction,axis));
    let y=cross(direction,x);
    let cosine=dot(direction,geometry);
    // shortcut: cap the tangent projection at 10,000x at grazing silhouettes;
    // exact visibility/curved-surface filtering needs additional ray samples.
    let denominator=select(-1.0,1.0,cosine>=0.0)*max(abs(cosine),0.0001);
    return mat2x3<f32>((x-direction*(dot(x,geometry)/denominator))*width,
        (y-direction*(dot(y,geometry)/denominator))*width);
}

fn rt_uv_delta(delta:vec3<f32>,e1:vec3<f32>,e2:vec3<f32>,u1:vec2<f32>,u2:vec2<f32>)->vec2<f32> {
    // Scale before forming area squared: centimetre blades and large instances
    // must not underflow/overflow the same UV Jacobian.
    let scale=max(length(e1),length(e2));
    if(scale<=0.0){return vec2(0.0);}
    let a=e1/scale;let b=e2/scale;let d=delta/scale;
    let n=cross(a,b);let area=dot(n,n);
    if(area<=1e-20){return vec2(0.0);}
    return (u1*dot(cross(d,b),n)+u2*dot(cross(a,d),n))/area;
}

// Non-opaque candidates use the same sampler, UVs, alpha factor and cutoff as raster.
fn rt_accept_candidate(pair_id:u32,triangle:u32,barycentrics:vec2<f32>,direction:vec3<f32>,width:f32)->bool {
    let pair=rt_pairs[pair_id];
    let range=rt_ranges[pair.words.x];
    let first=range.words.x+triangle*3u;let base=range.words.y;
    let a=rt_vertices[base+rt_indices[first]];
    let b=rt_vertices[base+rt_indices[first+1u]];
    let c=rt_vertices[base+rt_indices[first+2u]];
    let bary=vec3(1.0-barycentrics.x-barycentrics.y,barycentrics);
    let uv=a.uv.xy*bary.x+b.uv.xy*bary.y+c.uv.xy*bary.z;
    let e1=rt_vector(pair,b.position.xyz-a.position.xyz);let e2=rt_vector(pair,c.position.xyz-a.position.xyz);
    let footprint=rt_hit_footprint(direction,normalize(cross(e1,e2)),width);
    let dx=rt_uv_delta(footprint[0],e1,e2,b.uv.xy-a.uv.xy,c.uv.xy-a.uv.xy);
    let dy=rt_uv_delta(footprint[1],e1,e2,b.uv.xy-a.uv.xy,c.uv.xy-a.uv.xy);
    let alpha=rt_sample(rt_materials[pair.words.x].textures.x,uv,dx,dy).a*bitcast<f32>(range.words.z);
    return alpha>=bitcast<f32>(range.words.w);
}
