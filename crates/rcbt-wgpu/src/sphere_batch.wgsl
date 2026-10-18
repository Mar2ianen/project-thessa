// Transaction waves over an immutable pre-update snapshot. Plan priorities are
// request ordinals, not racing arrival order. Losers retry in a later window.
// Scratch: four global words followed by REQUESTS records of 200 words:
// kind, slot, chain count, free count, free ordinal, status, address low/high;
// 64 chain slots; 128 predecoded free slots.
@group(0) @binding(7) var<storage, read_write> plans: array<u32>;
@group(0) @binding(8) var<storage, read_write> claims: array<atomic<u32>>;
const PLAN_STRIDE = 200u;
fn plan_base(request: u32) -> u32 { return 4u + request * PLAN_STRIDE; }

@compute @workgroup_size(64)
fn clear_claims(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < p.config.x) { atomicStore(&claims[gid.x], NONE); }
    if (gid.x < REQUESTS) {
        let base = plan_base(gid.x);
        for (var i = 0u; i < 8u; i++) { plans[base + i] = 0u; }
    }
    if (gid.x == 0u) {
        plans[1] = min(REQUESTS, rank_count(1u));
        plans[2] = 0u;
        plans[3] = 0u;
    }
}

// Claim whole records, including the external records whose neighbor fields
// change. Merely locking the split leaves would still permit write/write races
// through a shared outside neighbor (including at cube corners).
fn neighborhood(slot: u32, request: u32, verify: bool) -> bool {
    var owned = true;
    for (var i = 0u; i < 4u; i++) {
        var neighbor = slot;
        if (i < 3u) { neighbor = pool[slot].neighbors[i]; }
        if (verify) { owned = owned && atomicLoad(&claims[neighbor]) == request; }
        else { atomicMin(&claims[neighbor], request); }
    }
    return owned;
}
fn transaction_claims(request: u32, verify: bool) -> bool {
    let base = plan_base(request);
    var owned = true;
    if (plans[base] == 0u) {
        for (var i = 0u; i < plans[base + 2u]; i++) {
            let a = plans[base + 8u + i];
            let b = pool[a].neighbors.z;
            let first = neighborhood(a, request, verify);
            let second = neighborhood(b, request, verify);
            owned = owned && first && second;
        }
    } else {
        let d = diamond(plans[base + 1u]);
        for (var i = 0u; i < 4u; i++) {
            let current = neighborhood(d[i], request, verify);
            owned = owned && current;
        }
    }
    return owned;
}

@compute @workgroup_size(64)
fn plan_transactions(@builtin(global_invocation_id) gid: vec3<u32>) {
    let request = gid.x;
    if (request >= plans[1] || (p.config.z != 1u && p.config.z != 3u && p.config.z != 4u)) { return; }
    let base = plan_base(request);
    var slot = indices[(plans[0] + request) % rank_count(1u)];
    // External canonical-area policy: each request owns disjoint strided ranks
    // over the immutable snapshot. Prefer its largest actionable split, else
    // one mergeable diamond. This reuses the same bounded transaction planner,
    // claims and allocator; no racing append or second request/geometry store.
    // Avoid spending a whole 256-request window on far-off-camera roots while
    // a newly rebased close view waits hundreds of frames for cursor rotation.
    if (p.config.z == 4u) {
        var selected = NONE;var largest = -1.0;var split_found = false;
        for (var rank = request; rank < rank_count(1u); rank += REQUESTS) {
            let candidate = indices[(plans[0] + rank) % rank_count(1u)];
            if (root_of(pool[candidate]) >= 12u) { continue; }
            let area = areas[candidate];
            if (area > p.policy.x && pool[candidate].address.z < p.config.y) {
                if (!split_found || area > largest) { selected = candidate;largest = area;split_found = true; }
            } else if (!split_found && selected == NONE && area < p.policy.y) {
                let d = diamond(candidate);
                if (d.x != NONE) {
                    var small = true;
                    for (var j = 0u; j < 4u; j++) { small = small && areas[d[j]] < p.policy.y; }
                    if (small) { selected = candidate; }
                }
            }
        }
        if (selected == NONE) { return; }slot = selected;
    }
    let t = pool[slot];
    if (root_of(t) >= 12u) { return; }
    var kind = p.config.w;
    if (p.config.z == 1u || p.config.z == 4u) {
        kind = 2u;
        if (areas[slot] > p.policy.x) { kind = 0u; }
        else {
            let d = diamond(slot);
            if (d.x != NONE) {
                var small = true;
                for (var j = 0u; j < 4u; j++) { small = small && areas[d[j]] < p.policy.y; }
                if (small) { kind = 1u; }
            }
        }
    }
    if (kind >= 2u) { return; }
    plans[base] = kind;
    plans[base + 1u] = slot;
    let address = select(t.address.xy, shift_right(t.address.xy, 1u), kind == 1u);
    plans[base + 6u] = address.x;
    plans[base + 7u] = address.y;
    if (kind == 0u) {
        var cursor = slot;
        var count = 0u;
        loop {
            if (count >= 64u) { plans[base + 5u] = 4u; return; }
            let a = pool[cursor];
            if (a.address.z >= p.config.y) { plans[base + 5u] = 3u; return; }
            if (a.address.w > 0xffffff00u) { plans[base + 5u] = 5u; return; }
            plans[base + 8u + count] = cursor;
            count++;
            let b = pool[a.neighbors.z];
            if (b.neighbors.z == cursor) {
                if (b.address.z >= p.config.y) { plans[base + 5u] = 3u; return; }
                if (b.address.w > 0xffffff00u) { plans[base + 5u] = 5u; return; }
                break;
            }
            cursor = a.neighbors.z;
        }
        plans[base + 2u] = count;
        plans[base + 3u] = count * 2u;
    } else {
        let d = diamond(slot);
        if (d.x == NONE) { plans[base + 5u] = 4u; return; }
        for (var i = 0u; i < 4u; i++) {
            if (pool[d[i]].address.w > 0xffffff00u) { plans[base + 5u] = 5u; return; }
        }
    }
    plans[base + 5u] = 6u;
    transaction_claims(request, false);
}

@compute @workgroup_size(64)
fn validate_claims(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= plans[1]) { return; }
    let base = plan_base(gid.x);
    if (plans[base + 5u] == 6u) {
        plans[base + 5u] = select(7u, 8u, transaction_claims(gid.x, true));
    }
}

// Only the short request-prefix admission is serial, not the pool traversal or
// topology changes. No merge frees are reused in this wave: all allocation IDs
// come from the immutable pre-commit bitfield. Each whole chain fits or rejects.
@compute @workgroup_size(1)
fn reserve_transactions() {
    let available = p.config.x - rank_count(1u);
    var reserved = 0u;
    for (var request = 0u; request < plans[1]; request++) {
        let base = plan_base(request);
        if (plans[base + 5u] != 8u) { continue; }
        let needed = plans[base + 3u];
        if (reserved + needed > available) { plans[base + 5u] = 2u; continue; }
        plans[base + 4u] = reserved;
        reserved += needed;
        plans[base + 5u] = 9u;
    }
    plans[3] = reserved;
}

@compute @workgroup_size(64)
fn allocate_transactions(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= plans[1]) { return; }
    let base = plan_base(gid.x);
    if (plans[base + 5u] != 9u) { return; }
    for (var i = 0u; i < plans[base + 3u]; i++) {
        let slot = decode_rank(plans[base + 4u] + i, false);
        if (slot >= p.config.x || pool[slot].address.w > 0xffffff00u) {
            plans[base + 5u] = 5u;
            return;
        }
        plans[base + 72u + i] = slot;
    }
    plans[base + 5u] = 10u;
}

fn commit_transaction(request: u32) {
    let base = plan_base(request);
    if (plans[base + 5u] != 10u) { return; }
    if (plans[base] == 0u) {
        var next = 0u;
        for (var c = i32(plans[base + 2u]) - 1; c >= 0; c--) {
            split_pair(plans[base + 8u + u32(c)], plans[base + 72u + next], plans[base + 73u + next]);
            next += 2u;
        }
        plans[base + 5u] = 1u;
    } else {
        plans[base + 5u] = merge_diamond(plans[base + 1u]);
    }
}
@compute @workgroup_size(64)
fn commit_parallel(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x < plans[1]) { commit_transaction(gid.x); }
}
// Same plans/reservations/IDs and kernels, only one invocation commits all of
// them. This control isolates parallel mutation correctness and scoped cost.
@compute @workgroup_size(1)
fn commit_serial() {
    for (var i = 0u; i < plans[1]; i++) { commit_transaction(i); }
}

@compute @workgroup_size(1)
fn finish_transactions() {
    var committed = 0u;
    var requested = 0u;
    for (var i = 0u; i < plans[1]; i++) {
        let base = plan_base(i);
        let status = plans[base + 5u];
        if (status != 0u) { requested++; }
        if (status == 2u) { state.counters.w++; }
        if (status != 1u) { continue; }
        committed++;
        if (plans[base] == 0u) {
            state.counters.x += plans[base + 3u];
            state.counters.y++;
        } else {
            state.counters.x += 2u;
            state.counters.z++;
        }
    }
    plans[2] = committed;
    plans[0] = (plans[0] + plans[1]) % rank_count(1u);
    // External-area mode scans ALL active ranks in disjoint subsets. Only a
    // completed wave with no requests AND no commits certifies quiescence.
    // Rejection/exhaustion/conflict is not convergence. Preserve other ABIs.
    state.request = vec4(2u, committed, select(0u,requested,p.config.z==4u), 0u);
    state.draw = vec4(rank_count(1u) * 3u, 1u, 0u, 0u);
}
