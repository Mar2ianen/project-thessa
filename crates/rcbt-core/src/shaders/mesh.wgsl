// Consumes the classifier's compact triangle stream, including LOD skirts.
// Vertex and fragment evaluation live in the shared raster module.
@group(0) @binding(15) var<storage, read> draw_arguments: array<u32>;

struct MeshPrimitive {
    @builtin(triangle_indices) indices: vec3<u32>,
};

struct MeshOutput {
    @builtin(vertex_count) vertex_count: u32,
    @builtin(primitive_count) primitive_count: u32,
    @builtin(vertices) vertices: array<VertexOutput, 96>,
    @builtin(primitives) primitives: array<MeshPrimitive, 32>,
};

var<workgroup> mesh_output: MeshOutput;

@mesh(mesh_output) @workgroup_size(32)
fn build_mesh(
    @builtin(local_invocation_index) lane: u32,
    @builtin(workgroup_id) group: vec3<u32>,
) {
    let first = (group.y * 256u + group.x) * 32u;
    let total = draw_arguments[0] / 3u;
    let count = min(total - min(first, total), 32u);
    if (lane == 0u) {
        mesh_output.vertex_count = count * 3u;
        mesh_output.primitive_count = count;
    }
    if (lane < count) {
        let vertex = lane * 3u;
        for (var corner = 0u; corner < 3u; corner += 1u) {
            mesh_output.vertices[vertex + corner] = cbt_vertex((first + lane) * 3u + corner);
        }
        mesh_output.primitives[lane].indices = vec3(vertex, vertex + 1u, vertex + 2u);
    }
}
