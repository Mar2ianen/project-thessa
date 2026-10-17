@group(0) @binding(0) var source:texture_2d_array<f32>;
@group(0) @binding(1) var destination:texture_storage_2d_array<rgba16float,write>;
@compute @workgroup_size(8,8)
fn reduce(@builtin(global_invocation_id) gid:vec3<u32>){
    if(any(gid.xy>=textureDimensions(destination))){return;}
    let p=vec2<i32>(gid.xy*2u);let layer=i32(gid.z);
    let v=(textureLoad(source,p,layer,0)+textureLoad(source,p+vec2(1,0),layer,0)
        +textureLoad(source,p+vec2(0,1),layer,0)+textureLoad(source,p+vec2(1,1),layer,0))*0.25;
    textureStore(destination,vec2<i32>(gid.xy),layer,v);
}
