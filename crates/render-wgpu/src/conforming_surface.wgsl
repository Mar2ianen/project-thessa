// Canonical data-page lookup and independent camera-local precision charts.
// Exact cube coordinates survive through depth 58; never round face UV before
// determining page ownership or subtracting the local chart's integer origin.
struct SurfaceChart { frame:CbtTileFrame, tile:vec4<u32> }
struct SurfaceVertex { position_height:vec4<f32>, normal_face:vec4<f32>, address:vec4<u32> }
@group(0) @binding(0) var<storage,read> surface_pool:array<Record>;
@group(0) @binding(1) var<storage,read> material_directory:array<vec4<u32>>;
@group(0) @binding(2) var<storage,read> height_metadata:array<vec4<u32>>;
@group(0) @binding(3) var<storage,read> page_residuals:array<u32>;
@group(0) @binding(4) var<storage,read> charts:array<SurfaceChart>;
@group(0) @binding(5) var<storage,read_write> surface_vertices:array<SurfaceVertex>;
@group(0) @binding(6) var<uniform> height_max_level:u32;

fn surface_vertex(cube:vec3<i32>)->SurfaceVertex {
    var face=0u;var ab=vec2<i32>(0);
    if(abs(cube.x)==268435456) {
        face=select(1u,0u,cube.x>=0);ab=vec2(select(cube.z,-cube.z,cube.x>=0),cube.y);
    } else if(abs(cube.y)==268435456) {
        face=select(3u,2u,cube.y>=0);ab=vec2(cube.x,select(cube.z,-cube.z,cube.y>=0));
    } else {
        face=select(5u,4u,cube.z>=0);ab=vec2(select(-cube.x,cube.x,cube.z>=0),cube.y);
    }
    let uv=vec2<u32>(ab+vec2(268435456));
    var layer=0xffffffffu;var level=height_max_level;var page_uv=vec2(0.0);
    var page_id=material_node(face,min(uv/vec2(536870912u>>level),vec2((1u<<level)-1u)),level);
    loop {
        layer=material_layer(page_id);
        if(layer!=0xffffffffu){
            let span=536870912u>>level;
            let xy=min(uv/vec2(span),vec2((1u<<level)-1u));
            page_uv=vec2<f32>(uv-xy*span)/f32(span);break;
        }
        if(level==0u){break;}level--;page_id=shift_right(page_id,2u);
    }
    let direction=normalize(vec3<f32>(cube));
    var height=0.0;var normal=direction;
    if(layer!=0xffffffffu) {
        let side=exp2(f32(level));
        var frame=charts[face].frame.geometry;
        // A page-local chart for the existing shared height normal decoder.
        let xy=min(uv/vec2(536870912u>>level),vec2((1u<<level)-1u));
        let center=(vec2<f32>(xy)+0.5)/side*2.0-1.0;
        frame.raw_center=vec4(frame.raw_axis_u.xyz*center.x+frame.raw_axis_v.xyz*center.y+cross(frame.raw_axis_u.xyz,frame.raw_axis_v.xyz),0.0);
        frame.normal=vec4(normalize(frame.raw_center.xyz),0.0);
        frame.radius_half_extent_len.y=1.0/side;
        frame.radius_half_extent_len.z=length(frame.raw_center.xyz);
        let sample=cbt_surface_sample(frame,height_metadata[layer],page_uv);
        height=sample.height;normal=sample.normal;
    }
    let chart=charts[face];let span=536870912u>>chart.tile.y;
    let local_uv=vec2<f32>(vec2<i32>(uv)-vec2<i32>(chart.tile.zw*span))/f32(span);
    let local=cbt_local_offset(chart.frame.geometry,local_uv,height);
    let position=chart.frame.anchor_hi_m.xyz+(chart.frame.anchor_lo_m.xyz+local);
    return SurfaceVertex(vec4(position,height),vec4(normal,f32(face)),vec4(uv,layer,level));
}
@compute @workgroup_size(64)
fn emit_surface(@builtin(global_invocation_id) gid:vec3<u32>) {
    let slot=gid.x;
    if(slot>=arrayLength(&surface_pool)){return;}
    let r=surface_pool[slot];
    if(r.address.z<4u||root_of(r)>=12u) {
        for(var i=0u;i<3u;i++){surface_vertices[slot*3u+i]=SurfaceVertex(vec4(0.0),vec4(0.0),vec4(0u));}
        return;
    }
    let t=cube_triangle(r);
    for(var i=0u;i<3u;i++){surface_vertices[slot*3u+i]=surface_vertex(t[i]);}
}
