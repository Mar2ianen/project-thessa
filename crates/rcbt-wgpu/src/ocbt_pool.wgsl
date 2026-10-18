// Independent portable GPU implementation of this crate's OcbtPoolMirror ABI.
// Atomic packed ancestor deltas support disjoint concurrent slot transactions.
// OcbtPoolMirror(depth=17): packed levels 0..10, virtual 128-bit groups.
fn location(node: u32) -> vec3<u32> {
    let level = 31u - countLeadingZeros(node);
    var offset = 0u;
    for (var l = 0u; l < level; l++) {
        offset += (1u << l) * select(select(8u, 16u, l < 10u), 32u, l < 7u);
    }
    let width = select(select(8u, 16u, level < 10u), 32u, level < 7u);
    let bit = offset + (node - (1u << level)) * width;
    return vec3(bit / 32u, bit % 32u, width);
}
fn mask_bits(width: u32) -> u32 {
    if (width == 32u) { return 0xffffffffu; }
    return (1u << width) - 1u;
}
fn rank_count(node: u32) -> u32 {
    let level = 31u - countLeadingZeros(node);
    if (level < 11u) {
        let p = location(node);
        return (atomicLoad(&ranks[p.x]) >> p.y) & mask_bits(p.z);
    }
    let span = 1u << (17u - level);
    let first = (node - (1u << level)) * span;
    var result = 0u;
    var cursor = first;
    while (cursor < first + span) {
        let width = min(32u - cursor % 32u, first + span - cursor);
        result += countOneBits((atomicLoad(&bits[cursor / 32u]) >> (cursor % 32u)) & mask_bits(width));
        cursor += width;
    }
    return result;
}
fn decode_rank(ordinal: u32, occupied: bool) -> u32 {
    var node = 1u;
    var rank = ordinal;
    var span = 65536u;
    while (node < 131072u) {
        let count = rank_count(node * 2u);
        let left = select(span - count, count, occupied);
        if (rank < left) {
            node *= 2u;
        } else {
            node = node * 2u + 1u;
            rank -= left;
        }
        span /= 2u;
    }
    return node - 131072u;
}
fn set_occupied(slot: u32, value: bool) {
    let mask = 1u << (slot % 32u);
    if (value) { atomicOr(&bits[slot / 32u], mask); } else { atomicAnd(&bits[slot / 32u], ~mask); }
    var node = 1024u + (slot >> 7u);
    while (node > 0u) {
        let p = location(node);
        // A field never exceeds its subtree slot count, which fits its packed
        // width, and a free removes an occupied bit exactly once. Consequently
        // +/- one field unit neither carries nor borrows into adjacent fields.
        let delta = 1u << p.y;
        atomicAdd(&ranks[p.x], select(0u - delta, delta, value));
        node /= 2u;
    }
}
