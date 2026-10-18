// Classify actual camera-relative canonical datum triangles, never projected
// absolute f32 planetary coordinates. Areas feed the existing CBT transactions.
struct Vertex { position:vec4<f32>, normal:vec4<f32>, address:vec4<u32> }
// extra_height_m is the optional shared wave envelope, not the old body-wide
// terrain bound. Height envelopes follow the canonical page publication.
struct Policy { clip:mat4x4<f32>, extent:vec2<f32>, extra_height_m:f32, split:f32 }
@group(0) @binding(0) var<storage,read> vertices:array<Vertex>;
@group(0) @binding(1) var<storage,read_write> areas:array<f32>;
@group(0) @binding(2) var<uniform> policy:Policy;
@group(0) @binding(3) var<storage,read> height_bounds:array<f32>;
@compute @workgroup_size(64)
fn classify(@builtin(global_invocation_id) id:vec3<u32>) {
    let slot=id.x;if(slot>=arrayLength(&areas)){return;}
    areas[slot]=0.0;
    let a=vertices[slot*3u];let b=vertices[slot*3u+1u];let c=vertices[slot*3u+2u];
    if(all(a.normal.xyz==vec3(0.0))){return;}
    let pa=policy.clip*vec4(a.position.xyz,1.0);
    let pb=policy.clip*vec4(b.position.xyz,1.0);
    let pc=policy.clip*vec4(c.position.xyz,1.0);
    if(max(pa.w,max(pb.w,pc.w))<=0.0){return;}
    // A near-plane-straddling triangle is not safely projectable. Retain a
    // split request rather than divide by zero or lose a visible sliver.
    if(min(pa.w,min(pb.w,pc.w))<=0.0){areas[slot]=policy.split*2.0;return;}
    let u=pa.xy/pa.w;let v=pb.xy/pb.w;let w=pc.xy/pc.w;
    if(any(min(u,min(v,w))>vec2(1.0))||any(max(u,max(v,w))<vec2(-1.0))){return;}
    let e=(v-u)*policy.extent*0.5;let f=(w-u)*policy.extent*0.5;
    let longest=max(length(a.position.xyz-b.position.xyz),max(length(b.position.xyz-c.position.xyz),length(c.position.xyz-a.position.xyz)));
    if(longest==0.0){return;}
    var height=height_bounds[0];
    // Same canonical face+resident page at all corners implies containment in
    // that convex page rectangle. Its envelope also includes hidden resident
    // descendants. Never infer a local bound across seams, multiple pages or
    // three missing-page corners (which may enclose an unsampled page).
    let page=a.address.z;
    if(a.normal.w==b.normal.w && a.normal.w==c.normal.w && page!=0xffffffffu
        && page==b.address.z && page==c.address.z && page+1u<arrayLength(&height_bounds)) {
        height=height_bounds[page+1u];
    }
    let relief=1.0+2.0*(height+policy.extra_height_m)/longest;
    areas[slot]=min(abs(e.x*f.y-e.y*f.x)*0.5*relief*relief,3.0e38);
}
