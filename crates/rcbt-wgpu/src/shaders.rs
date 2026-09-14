//! Real CBT WGSL kernels over the [`crate::heap`] layout.
//!
//! One thread per op for `apply_ops` (sparse commit: only touched nodes plus
//! their ancestor chains move), one thread per leaf for `decode_all`. Only
//! `u32` atomics, no extensions, so the kernels stay inside portable WGSL.
//!
//! Binding layout (shared by both pipelines):
//!
//! ```text
//! 0: uniform  Params { max_depth, count, _, _ }
//! 1: storage  active bits (atomic<u32>, read_write)
//! 2: storage  level sums  (atomic<u32>, read_write)
//! 3: storage  ops (vec4<u32>: id, depth, kind, pad; apply only)
//! 3: storage  out (vec2<u32>: id, depth; decode only)
//! ```

/// `ops[i] = (id, depth, kind, 0)`: kind 0 splits the leaf, kind 1 merges
/// the children of the parent. Batches must be pre-validated (replay the
/// same sequence on the CPU mirror first): overlapping ops in one batch are
/// a caller error, exactly like conflicting native `Update`s.
pub const CBT_APPLY_WGSL: &str = r#"
struct Params {
    max_depth: u32,
    count: u32,
    _p0: u32,
    _p1: u32,
};

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read_write> active_bits: array<atomic<u32>>;
@group(0) @binding(2) var<storage, read_write> sums: array<atomic<u32>>;
@group(0) @binding(3) var<storage, read> ops: array<vec4<u32>>;

fn set_bit(id: u32) {
    let bit = id - 1u;
    atomicOr(&active_bits[bit / 32u], 1u << (bit % 32u));
}

fn clear_bit(id: u32) {
    let bit = id - 1u;
    atomicAnd(&active_bits[bit / 32u], ~(1u << (bit % 32u)));
}

fn add_to_ancestors(id: u32, delta: u32) {
    var a = id / 2u;
    while (a >= 1u) {
        atomicAdd(&sums[a], delta);
        if (a == 1u) {
            break;
        }
        a = a / 2u;
    }
}

@compute @workgroup_size(64)
fn apply_ops(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= arrayLength(&ops)) {
        return;
    }
    let op = ops[i];
    let id = op.x;
    if (op.z == 0u) {
        let left = id * 2u;
        let right = left + 1u;
        set_bit(left);
        set_bit(right);
        clear_bit(id);
        atomicStore(&sums[left], 1u);
        atomicStore(&sums[right], 1u);
        atomicStore(&sums[id], 2u);
        add_to_ancestors(id, 1u);
    } else {
        let left = id * 2u;
        let right = left + 1u;
        clear_bit(left);
        clear_bit(right);
        set_bit(id);
        atomicStore(&sums[left], 0u);
        atomicStore(&sums[right], 0u);
        atomicStore(&sums[id], 1u);
        add_to_ancestors(id, 0xFFFFFFFFu);
    }
}
"#;

/// One thread per leaf index: root-to-leaf walk guided by the sums.
/// `count` (uniform) is the leaf count captured after the last apply.
pub const CBT_DECODE_WGSL: &str = r#"
struct Params {
    max_depth: u32,
    count: u32,
    _p0: u32,
    _p1: u32,
};

@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read_write> active_bits: array<atomic<u32>>;
@group(0) @binding(2) var<storage, read_write> sums: array<atomic<u32>>;
@group(0) @binding(3) var<storage, read_write> out: array<vec2<u32>>;

@compute @workgroup_size(64)
fn decode_all(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= p.count) {
        return;
    }
    var node = 1u;
    var base = 0u;
    var depth = 0u;
    loop {
        if (atomicLoad(&sums[node]) == 1u) {
            out[i] = vec2<u32>(node, depth);
            return;
        }
        let left = atomicLoad(&sums[node * 2u]);
        if (i - base < left) {
            node = node * 2u;
        } else {
            base += left;
            node = node * 2u + 1u;
        }
        depth += 1u;
        if (depth > p.max_depth) {
            out[i] = vec2<u32>(0u, 0xFFFFFFFFu);
            return;
        }
    }
}
"#;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cbt_kernels_parse_as_portable_wgsl() {
        naga::front::wgsl::parse_str(CBT_APPLY_WGSL).expect("apply kernels must parse");
        naga::front::wgsl::parse_str(CBT_DECODE_WGSL).expect("decode kernels must parse");
    }
}
