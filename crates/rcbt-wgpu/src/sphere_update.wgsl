// HPG 2024 §2 refinement/decimation ideas, independently implemented. A bounded
// single writer is an explicit correctness baseline, NOT the parallel paper.
struct State {
    request: vec4<u32>,  // kind, address low/high, status (1=committed)
    counters: vec4<u32>, // changed leaves, split requests, merges, exhaustion
    draw: vec4<u32>,
}
@group(0) @binding(0) var<uniform> p: Params;
@group(0) @binding(1) var<storage, read_write> pool: array<Record>;
@group(0) @binding(2) var<storage, read_write> ranks: array<u32>;
@group(0) @binding(3) var<storage, read_write> bits: array<u32>;
@group(0) @binding(4) var<storage, read_write> indices: array<u32>;
@group(0) @binding(5) var<storage, read_write> state: State;
@group(0) @binding(6) var<storage, read_write> areas: array<f32>;
const NONE = 0xffffffffu;
fn replace_neighbor(slot: u32, old: u32, replacement: u32) {
    for (var e = 0u; e < 3u; e++) {
        if (pool[slot].neighbors[e] == old) { pool[slot].neighbors[e] = replacement; }
    }
}
fn store_child(slot: u32, parent: Record, right: u32, edges: vec3<u32>) {
    let id = child_id(parent.address.xy, right);
    // A reused parent changes generation; an allocated slot keeps its already
    // advanced free-slot generation, matching BisectorPool's handle contract.
    let generation = pool[slot].address.w + select(0u, 1u, slot == parent.neighbors.w);
    pool[slot] = Record(vec4(id, parent.address.z + 1u, generation), vec4(edges, 0u));
}
fn split_pair(a: u32, ar: u32, br: u32) {
    var t = pool[a];
    let b = t.neighbors.z;
    var u = pool[b];
    t.neighbors.w = a;
    u.neighbors.w = b;
    replace_neighbor(t.neighbors.y, a, ar);
    replace_neighbor(u.neighbors.y, b, br);
    store_child(a, t, 0u, vec3(ar, br, t.neighbors.x));
    store_child(ar, t, 1u, vec3(b, a, t.neighbors.y));
    store_child(b, u, 0u, vec3(br, ar, u.neighbors.x));
    store_child(br, u, 1u, vec3(a, b, u.neighbors.y));
    set_occupied(ar, true);
    set_occupied(br, true);
}
fn refine(slot: u32) -> u32 {
    var chain: array<u32,64>;
    var allocated: array<u32,128>;
    var count = 0u;
    var cursor = slot;
    loop {
        if (count >= 64u) { return 4u; }
        let t = pool[cursor];
        if (t.address.z >= p.config.y) { return 3u; }
        if (t.address.w > 0xffffff00u) { return 5u; }
        chain[count] = cursor;
        count++;
        let twin = t.neighbors.z;
        if (pool[twin].neighbors.z == cursor) {
            if (pool[twin].address.z >= p.config.y) { return 3u; }
            if (pool[twin].address.w > 0xffffff00u) { return 5u; }
            break;
        }
        cursor = twin;
    }
    if (rank_count(1u) + count * 2u > p.config.x) {
        state.counters.w++;
        return 2u;
    }
    // Reserve the entire compatibility chain before changing ANY live record.
    for (var i = 0u; i < count * 2u; i++) {
        let free = decode_rank(i, false);
        if (free >= p.config.x || pool[free].address.w > 0xffffff00u) { return 5u; }
        allocated[i] = free;
    }
    var i = 0u;
    for (var c = i32(count) - 1; c >= 0; c--) {
        split_pair(chain[u32(c)], allocated[i], allocated[i + 1u]);
        i += 2u;
    }
    state.counters.x += count * 2u;
    state.counters.y++;
    return 1u;
}
fn diamond(a: u32) -> vec4<u32> {
    let t = pool[a];
    if (t.address.z <= 4u || (t.address.x & 1u) != 0u) { return vec4(NONE); }
    let ar = t.neighbors.x;
    let br = t.neighbors.y;
    let b = pool[br].neighbors.y;
    let r = pool[ar];
    let u = pool[b];
    let v = pool[br];
    if (any(r.address.xy != child_id(shift_right(t.address.xy, 1u), 1u)) ||
        r.address.z != t.address.z || u.address.z != t.address.z || v.address.z != t.address.z ||
        (u.address.x & 1u) != 0u || any(v.address.xy != child_id(shift_right(u.address.xy, 1u), 1u)) ||
        r.neighbors.x != b || u.neighbors.x != br || v.neighbors.x != a) {
        return vec4(NONE);
    }
    return vec4(a, ar, b, br);
}
fn decimate(a: u32) -> u32 {
    let d = diamond(a);
    if (d.x == NONE) { return 4u; }
    for (var i = 0u; i < 4u; i++) {
        if (pool[d[i]].address.w > 0xffffff00u) { return 5u; }
    }
    let t = pool[d.x];
    let r = pool[d.y];
    let u = pool[d.z];
    let v = pool[d.w];
    replace_neighbor(r.neighbors.z, d.y, d.x);
    replace_neighbor(v.neighbors.z, d.w, d.z);
    pool[d.x] = Record(vec4(shift_right(t.address.xy, 1u), t.address.z - 1u, t.address.w + 1u), vec4(t.neighbors.z, r.neighbors.z, d.z, 0u));
    pool[d.z] = Record(vec4(shift_right(u.address.xy, 1u), u.address.z - 1u, u.address.w + 1u), vec4(u.neighbors.z, v.neighbors.z, d.x, 0u));
    pool[d.y] = Record(vec4(0u, 0u, 0u, r.address.w + 1u), vec4(NONE));
    pool[d.w] = Record(vec4(0u, 0u, 0u, v.address.w + 1u), vec4(NONE));
    set_occupied(d.y, false);
    set_occupied(d.w, false);
    state.counters.x += 2u;
    state.counters.z++;
    return 1u;
}
@compute @workgroup_size(1)
fn update() {
    if (p.config.z == 0u && p.config.w < 2u) {
        let id = p.requested_address.xy;
        var valid = any(id != vec2(0u));
        let depth = select(31u - countLeadingZeros(id.x), 63u - countLeadingZeros(id.y), id.y != 0u);
        valid = valid && depth >= 4u && depth <= 58u;
        if (valid) { valid = root_of(Record(vec4(id, depth, 0u), vec4(0u))) < 12u; }
        if (!valid) {
            state.request = vec4(p.config.w, id, 4u);
            state.draw = vec4(rank_count(1u) * 3u, 1u, 0u, 0u);
            return;
        }
    }
    var chosen = NONE;
    var kind = p.config.w;
    var best = p.policy.x;
    for (var i = 0u; i < rank_count(1u); i++) {
        let slot = indices[i];
        let t = pool[slot];
        if (t.address.z < 4u || root_of(t) >= 12u) { continue; }
        if (p.config.z == 0u) {
            let requested = select(p.requested_address.xy, child_id(p.requested_address.xy, 0u), kind == 1u);
            if (all(t.address.xy == requested)) { chosen = slot; break; }
        }
        if (p.config.z == 1u) {
            let area = areas[slot];
            if (area > best && t.address.z < p.config.y) {
                chosen = slot;
                kind = 0u;
                best = area;
            }
            if (chosen == NONE) {
                let d = diamond(slot);
                if (d.x != NONE) {
                    var small = true;
                    for (var j = 0u; j < 4u; j++) { small = small && areas[d[j]] < p.policy.y; }
                    if (small) { chosen = slot; kind = 1u; }
                }
            }
        }
    }
    state.request = vec4(kind, p.requested_address.xy, 0u);
    if (chosen != NONE) {
        let t = pool[chosen];
        let requested = select(t.address.xy, shift_right(t.address.xy, 1u), kind == 1u);
        let status = select(0u, 4u, p.config.z == 0u);
        state.request = vec4(kind, requested, status);
        if (kind == 0u) { state.request.w = refine(chosen); }
        else if (kind == 1u) { state.request.w = decimate(chosen); }
    } else if (p.config.z == 0u && kind < 2u) {
        state.request.w = 4u;
    }
    state.draw = vec4(rank_count(1u) * 3u, 1u, 0u, 0u);
}
@compute @workgroup_size(64)
fn index_pool(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < rank_count(1u)) { indices[gid.x] = decode_rank(gid.x, true); }
}
@compute @workgroup_size(64)
fn classify(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (p.config.z == 1u && gid.x < rank_count(1u)) {
        let slot = indices[gid.x];
        areas[slot] = pixel_area(pool[slot]);
    }
}
