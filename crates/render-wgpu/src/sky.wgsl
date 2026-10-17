// The same physical field as reflected misses, not a second atmosphere model.
struct SkyView {
    clip:mat4x4<f32>, unjittered:mat4x4<f32>, inverse_clip:mat4x4<f32>,
    world_from_view:mat4x4<f32>, view_from_world:mat4x4<f32>,
    projection:mat4x4<f32>, inverse_projection:mat4x4<f32>, camera:vec3<f32>, exposure:f32,
}
@group(0) @binding(0) var<uniform> sky_view:SkyView;
struct SkyVertex { @builtin(position) position:vec4<f32>, @location(0) ndc:vec2<f32> }
@vertex fn sky_vertex(@builtin(vertex_index) index:u32)->SkyVertex {
    var out:SkyVertex;
    let p=vec2(f32((index<<1u)&2u),f32(index&2u))*2.0-1.0;
    out.position=vec4(p,1.0,1.0);
    out.ndc=p;
    return out;
}
@fragment fn sky_fragment(in:SkyVertex)->@location(0) vec4<f32> {
    // Use z=0.5, not far-plane z=1: an infinite projection can make w=0
    // there. Translation is explicitly excluded; the field is body-fixed.
    let point=sky_view.inverse_projection*vec4(in.ndc,0.5,1.0);
    let direction=(sky_view.world_from_view*vec4(point.xyz/point.w,0.0)).xyz;
    return vec4(env_radiance(direction,vec3(0.0))*sky_view.exposure,1.0);
}
