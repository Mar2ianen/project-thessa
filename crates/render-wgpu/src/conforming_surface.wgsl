// Canonical data-page lookup and independent camera-local precision charts.
// Exact cube coordinates survive through depth 58; never round face UV before
// determining page ownership or subtracting the local chart's integer origin.
@group(0) @binding(0) var<storage,read> surface_pool:array<Record>;
@group(0) @binding(1) var<storage,read> material_directory:array<vec4<u32>>;
@group(0) @binding(2) var<storage,read> height_metadata:array<vec4<u32>>;
@group(0) @binding(3) var<storage,read> page_residuals:array<u32>;
@group(0) @binding(4) var<storage,read> charts:array<SurfaceChart>;
@group(0) @binding(5) var<storage,read_write> surface_vertices:array<SurfaceVertex>;
@group(0) @binding(6) var<uniform> height_max_level:u32;

fn emit_slot(slot:u32, clear_inactive:bool) {
    if(slot>=arrayLength(&surface_pool)){return;}
    let r=surface_pool[slot];
    if(r.address.z<4u||root_of(r)>=12u) {
        if(clear_inactive) {
            for(var i=0u;i<3u;i++){surface_vertices[slot*3u+i]=SurfaceVertex(vec4(0.0),vec4(0.0),vec4(0u));}
        }
        return;
    }
    let t=cube_triangle(r);
    for(var i=0u;i<3u;i++){surface_vertices[slot*3u+i]=surface_vertex(t[i]);}
}

@compute @workgroup_size(64)
fn emit_surface(@builtin(global_invocation_id) gid:vec3<u32>) {
    emit_slot(gid.x,true);
}

// Classification alone consumes this scratch; final publication still clears
// every inactive slot for raster/shadow/RT consumers.
@compute @workgroup_size(64)
fn emit_active_scratch(@builtin(global_invocation_id) gid:vec3<u32>) {
    emit_slot(gid.x,false);
}
