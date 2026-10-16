@group(0) @binding(0) var<uniform> shadow_clip:mat4x4<f32>;
@group(1) @binding(0) var<storage,read> shadow_vertices:array<vec4<f32>>;
@group(1) @binding(1) var<storage,read> shadow_frames:array<CbtTileFrame>;
@group(1) @binding(2) var<storage,read> shadow_metadata:array<vec4<u32>>;
@group(1) @binding(3) var<uniform> shadow_transform:mat4x4<f32>;
@vertex fn vertex(@builtin(vertex_index) index:u32)->@builtin(position) vec4<f32> {
    // Full resident height geometry, NOT the camera-culled active triangle list.
    // A caster behind the camera must still be able to shadow a visible point.
    let ordinal=index/6144u;
    if (shadow_metadata[ordinal].z==0u) { return vec4(2.0,2.0,2.0,1.0); }
    let cell=(index%6144u)/6u;
    let corner=index%6u;
    let offsets=array<vec2<u32>,6>(vec2(0u,0u),vec2(1u,0u),vec2(0u,1u),vec2(1u,0u),vec2(1u,1u),vec2(0u,1u));
    let xy=vec2(cell%32u,cell/32u)+offsets[corner];
    let local=shadow_vertices[(ordinal*1089u+xy.y*33u+xy.x)*2u].xyz;
    let frame=shadow_frames[ordinal];
    return shadow_clip*cbt_render_position(frame,local,shadow_transform);
}
