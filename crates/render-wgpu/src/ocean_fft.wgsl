// Independent radix-2, workgroup-resident inverse FFT. Three complex channels
// (height and analytic derivatives) share twiddles and synchronization.
struct Coefficient { initial:vec4<f32>, dispersion:vec4<f32> }
struct Value { h:vec2<f32>, dx:vec2<f32>, dy:vec2<f32> }
struct Params { size:u32, layers:u32, delta_s:f32, moment_scale:f32 }
@group(0) @binding(0) var<storage,read> spectrum:array<Coefficient>;
@group(0) @binding(1) var<storage,read_write> evolved:array<Value>;
@group(0) @binding(2) var<storage,read_write> rows:array<Value>;
@group(0) @binding(3) var output:texture_storage_2d_array<rgba16float,write>;
@group(0) @binding(4) var<uniform> params:Params;
fn complex_mul(a:vec2<f32>,b:vec2<f32>)->vec2<f32>{return vec2(a.x*b.x-a.y*b.y,a.x*b.y+a.y*b.x);}
@compute @workgroup_size(64)
fn evolve(@builtin(global_invocation_id) gid:vec3<u32>){
    let i=gid.x; if(i>=params.size*params.size*params.layers){return;}
    let s=spectrum[i]; let phase=s.dispersion.w+s.dispersion.z*params.delta_s;
    let p=vec2(cos(phase),sin(phase));
    let h=complex_mul(s.initial.xy,p)+complex_mul(s.initial.zw*vec2(1.0,-1.0),p*vec2(1.0,-1.0));
    evolved[i]=Value(h,vec2(-h.y,h.x)*s.dispersion.x,vec2(-h.y,h.x)*s.dispersion.y);
}
var<workgroup> tile:array<Value,128>;
fn reverse_index(i:u32)->u32{return reverseBits(i)>>(32u-countTrailingZeros(params.size));}
fn butterfly(lane:u32){
    for(var span=2u;span<=params.size;span*=2u){
        if(lane<params.size/2u){
            let j=lane%(span/2u);let a=(lane/(span/2u))*span+j;let b=a+span/2u;
            let angle=6.28318530718*f32(j)/f32(span);let w=vec2(cos(angle),sin(angle));
            let left=tile[a];let right=tile[b];
            let h=complex_mul(right.h,w);let dx=complex_mul(right.dx,w);let dy=complex_mul(right.dy,w);
            tile[a]=Value(left.h+h,left.dx+dx,left.dy+dy);
            tile[b]=Value(left.h-h,left.dx-dx,left.dy-dy);
        }
        workgroupBarrier();
    }
}
@compute @workgroup_size(64)
fn fft_rows(@builtin(workgroup_id) group:vec3<u32>,@builtin(local_invocation_index) lane:u32){
    let base=(group.y*params.size+group.x)*params.size;
    for(var j=lane;j<params.size;j+=64u){tile[reverse_index(j)]=evolved[base+j];}
    workgroupBarrier(); butterfly(lane);
    for(var j=lane;j<params.size;j+=64u){
        let v=tile[j];let scale=1.0/f32(params.size);rows[base+j]=Value(v.h*scale,v.dx*scale,v.dy*scale);
    }
}
@compute @workgroup_size(64)
fn fft_columns(@builtin(workgroup_id) group:vec3<u32>,@builtin(local_invocation_index) lane:u32){
    let base=group.y*params.size*params.size+group.x;
    for(var j=lane;j<params.size;j+=64u){tile[reverse_index(j)]=rows[base+j*params.size];}
    workgroupBarrier();butterfly(lane);
    for(var j=lane;j<params.size;j+=64u){
        let v=tile[j];let scale=1.0/f32(params.size);let gradient=vec2(v.dx.x,v.dy.x)*scale;
        // Keep small slope moments out of half-float's coarse subnormal range.
        textureStore(output,vec2<i32>(i32(group.x),i32(j)),i32(group.y),vec4(v.h.x*scale,gradient,dot(gradient,gradient)*params.moment_scale));
    }
}
