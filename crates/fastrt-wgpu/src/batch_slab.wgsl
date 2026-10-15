// Batched slab ray-AABB intersection (FastRT portable backend, f32 WGSL).
//
// One thread per (ray, box) pair. The math mirrors the CPU oracle
// `thessa_plume_core::cbt_volume::ray_intersect_slab`: hit iff the slabs
// overlap and t_exit >= 0, with t_enter clamped at 0 (an origin inside the
// box reports 0). Degenerate all-zero directions always miss, matching the
// oracle. Non-finite inputs are caller error (the host debug-asserts
// finiteness); NaN behavior on GPU is not part of the contract.
//
// Precision note: f32 throughout. The CPU oracle is f64 truth; the error
// envelope is pinned by the host-side parity test, not by this file.

@group(0) @binding(0) var<storage, read> boxes_min: array<vec4<f32>>;
@group(0) @binding(1) var<storage, read> boxes_max: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read> rays_org: array<vec4<f32>>;
@group(0) @binding(3) var<storage, read> rays_dir: array<vec4<f32>>;
// Per pair: (t_enter, t_exit, hit, 0.0).
@group(0) @binding(4) var<storage, read_write> out_hit: array<vec4<f32>>;
// (nboxes, nrays, 0, 0): input buffers are pooled at power-of-two capacity,
// so the shader must not derive counts from arrayLength.
@group(0) @binding(5) var<uniform> dims: vec4<u32>;

@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let pair = gid.x;
    if (pair >= dims.x * dims.y) {
        return;
    }
    let nboxes = dims.x;
    if (nboxes == 0u) {
        return;
    }
    let r = pair / nboxes;
    let b = pair % nboxes;
    let org = rays_org[r].xyz;
    let dir = rays_dir[r].xyz;
    if (dir.x == 0.0 && dir.y == 0.0 && dir.z == 0.0) {
        out_hit[pair] = vec4<f32>(0.0, 0.0, 0.0, 0.0);
        return;
    }
    let bmin = boxes_min[b].xyz;
    let bmax = boxes_max[b].xyz;
    let o = array<f32, 3>(org.x, org.y, org.z);
    let d = array<f32, 3>(dir.x, dir.y, dir.z);
    let mn = array<f32, 3>(bmin.x, bmin.y, bmin.z);
    let mx = array<f32, 3>(bmax.x, bmax.y, bmax.z);
    var tmin = -3.40282347e+38;
    var tmax = 3.40282347e+38;
    var miss = false;
    for (var i = 0u; i < 3u; i++) {
        if (d[i] == 0.0) {
            if (o[i] < mn[i] || o[i] > mx[i]) {
                miss = true;
            }
        } else {
            let inv = 1.0 / d[i];
            var t0 = (mn[i] - o[i]) * inv;
            var t1 = (mx[i] - o[i]) * inv;
            if (t0 > t1) {
                let tmp = t0;
                t0 = t1;
                t1 = tmp;
            }
            tmin = max(tmin, t0);
            tmax = min(tmax, t1);
            if (tmax < tmin) {
                miss = true;
            }
        }
    }
    let hit = !miss && tmax >= 0.0;
    let enter = max(tmin, 0.0);
    out_hit[pair] = select(
        vec4<f32>(enter, tmax, 0.0, 0.0),
        vec4<f32>(enter, tmax, 1.0, 0.0),
        hit,
    );
}
