// Area plus sampled source/chord error, in screen pixels. Absolute elevation
// is not relief: translating flat terrain must not force endless subdivision.
struct Policy { clip:mat4x4<f32>, extent:vec2<f32>, merge:f32, split:f32 }
@group(0) @binding(0) var<storage,read> vertices:array<SurfaceVertex>;
@group(0) @binding(1) var<storage,read_write> areas:array<f32>;
@group(0) @binding(2) var<uniform> policy:Policy;
@group(0) @binding(3) var<storage,read> material_directory:array<vec4<u32>>;
@group(0) @binding(4) var<storage,read> height_metadata:array<vec4<u32>>;
@group(0) @binding(5) var<storage,read> page_residuals:array<u32>;
@group(0) @binding(6) var<storage,read> charts:array<SurfaceChart>;
@group(0) @binding(7) var<uniform> height_max_level:u32;
@group(0) @binding(8) var<storage,read> surface_pool:array<Record>;

fn side_planes(p:vec4<f32>)->vec4<f32> {
    return vec4(p.w+p.x,p.w-p.x,p.w+p.y,p.w-p.y);
}
fn triangle_score(source:array<SurfaceVertex,3>)->f32 {
    let a=source[0];let b=source[1];let c=source[2];
    // The primary draw culls these same outward-wound back faces. Refining
    // the hidden hemisphere to the viewport budget wastes most of the pool.
    if(dot(cross(b.position_height.xyz-a.position_height.xyz,c.position_height.xyz-a.position_height.xyz),-a.position_height.xyz)<=0.0){return 0.0;}
    let pa=policy.clip*vec4(a.position_height.xyz,1.0);
    let pb=policy.clip*vec4(b.position_height.xyz,1.0);
    let pc=policy.clip*vec4(c.position_height.xyz,1.0);
    // Cull in homogeneous coordinates BEFORE handling a w-straddling edge.
    // Invisible near-eye triangles must not request splits forever.
    if(any(max(side_planes(pa),max(side_planes(pb),side_planes(pc)))<vec4(0.0))
        ||max(pa.z,max(pb.z,pc.z))<0.0
        ||max(pa.w-pa.z,max(pb.w-pb.z,pc.w-pc.z))<0.0){return 0.0;}
    if(min(pa.w,min(pb.w,pc.w))<=0.0){return policy.split*2.0;}
    let e=(pb.xy/pb.w-pa.xy/pa.w)*policy.extent*0.5;
    let f=(pc.xy/pc.w-pa.xy/pa.w)*policy.extent*0.5;
    var score=abs(e.x*f.y-e.y*f.x)*0.5;
    var cube:array<vec3<i32>,3>;
    var datum:array<vec3<f32>,3>;
    for(var i=0u;i<3u;i++) {
        cube[i]=conforming_cube(source[i].address.xy,u32(source[i].normal_face.w));
        datum[i]=source[i].position_height.xyz;
        // The stream contains admitted, band-limited ocean deformation. Its
        // actual geometry contributes area; land-height probes use datum only.
        if(source[i].position_height.w<1.0) {
            datum[i]=surface_vertex(cube[i]).position_height.xyz;
        }
    }
    for(var i=0u;i<3u;i++) {
        let j=(i+1u)%3u;
        let sample=surface_vertex((cube[i]+cube[j])/2).position_height.xyz;
        let actual=policy.clip*vec4(sample,1.0);
        let linear=policy.clip*vec4((datum[i]+datum[j])*0.5,1.0);
        if(min(actual.w,linear.w)<=0.0){score=max(score,policy.split*2.0);continue;}
        let error=(actual.xy/actual.w-linear.xy/linear.w)*policy.extent*0.5;
        // Existing quality areas 1024/256/64 imply sampled height error
        // targets 4/2/1 px. The same 8:1 split/merge hysteresis applies.
        score=max(score,64.0*dot(error,error));
    }
    return min(score,3.0e38);
}
@compute @workgroup_size(64)
fn classify(@builtin(global_invocation_id) id:vec3<u32>) {
    let slot=id.x;if(slot>=arrayLength(&areas)){return;}
    areas[slot]=0.0;
    let a=vertices[slot*3u];let b=vertices[slot*3u+1u];let c=vertices[slot*3u+2u];
    if(all(a.normal_face.xyz==vec3(0.0))){return;}
    var score=triangle_score(array(a,b,c));
    // A midpoint peak can be represented exactly by its children. Their small
    // error must not allow a merge back into the same inaccurate parent.
    var parent=surface_pool[slot];
    if(score<policy.merge && parent.address.z>4u) {
        parent.address=vec4(shift_right(parent.address.xy,1u),parent.address.z-1u,0u);
        let cube=cube_triangle(parent);
        let parent_score=triangle_score(array(surface_vertex(cube[0]),surface_vertex(cube[1]),surface_vertex(cube[2])));
        if(parent_score>policy.split){score=policy.merge;}
    }
    areas[slot]=score;
}
