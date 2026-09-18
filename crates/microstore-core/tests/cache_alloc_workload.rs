//! Cache + allocator integration workload: a scripted deterministic
//! operation sequence drives `ResidencyCache` and `SlabAllocator` the way
//! a backend driver would, checking cross-structure invariants after
//! every single op. This is the "real" test: not one function in
//! isolation, but the two structures evolving together like in production.
//!
//! Driver rules (mirror a real backend):
//!
//! - slab id == cache key; every resident page has a placement fitting
//!   its current wire image;
//! - cache-side evictions (budget pressure, explicit evict) free the slab
//!   placement in the same step;
//! - page growth re-allocates (possibly moving); shrinks keep their offset;
//! - periodic compaction applies move plans wholesale.

use thessa_microstore_core::{
    EncodeMode, EncodedPage, ResidencyCache, ScalarField, SlabAllocator, fixtures,
};

const MODE: EncodeMode = EncodeMode::Adaptive { max_abs_error: 2.0 };

fn page_pool() -> Vec<ScalarField> {
    // Twelve small pages with different statistics (deterministic).
    let makers: Vec<(&str, u32, u64)> = vec![
        ("uniform", 32, 1),
        ("gradient", 32, 0),
        ("noise", 32, 11),
        ("noise", 32, 12),
        ("sharp", 32, 0),
        ("checker", 32, 4),
        ("coast", 32, 21),
        ("volcanic", 32, 22),
        ("noise", 48, 13),
        ("coast", 48, 23),
        ("gradient", 48, 0),
        ("sharp", 48, 0),
    ];
    makers
        .into_iter()
        .map(|(kind, size, seed)| match kind {
            "uniform" => fixtures::uniform(size, size, 100),
            "gradient" => fixtures::gradient(size, size),
            "noise" => fixtures::noise(size, size, seed),
            "sharp" => fixtures::sharp_boundary(size, size),
            "checker" => fixtures::checker(size, size, 4),
            "coast" => fixtures::coast(size, size, seed),
            _ => fixtures::volcanic(size, size, seed),
        })
        .collect()
}

fn xorshift(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Reconcile the slab with the cache: free placements of evicted keys,
/// (re)place residents at fitting sizes. Returns false on allocator OOM
/// (the pool below is sized to never hit it; OOM would fail the test).
fn sync(cache: &ResidencyCache, slab: &mut SlabAllocator) -> bool {
    for key in cache.keys() {
        let page = cache.get_ref(key).expect("listed key is resident");
        let wire = page.encoded_bytes() as u64;
        match slab.placement(key) {
            Some((_, size)) if size == wire => {}
            Some(_) => {
                if slab.realloc(key, wire).is_err() {
                    return false;
                }
            }
            None => {
                if slab.alloc(key, wire).is_err() {
                    return false;
                }
            }
        }
    }
    let live: std::collections::BTreeSet<u64> = cache.keys().into_iter().collect();
    let stale: Vec<u64> = (0..64)
        .filter(|id| slab.placement(*id).is_some() && !live.contains(id))
        .collect();
    for id in stale {
        slab.free_id(id).expect("stale placement frees");
    }
    true
}

fn check_all(cache: &ResidencyCache, slab: &SlabAllocator) {
    slab.check_invariants();
    // Every resident page has a fitting placement.
    for key in cache.keys() {
        let wire = cache.get_ref(key).expect("listed").encoded_bytes() as u64;
        let (offset, size) = slab.placement(key).expect("resident placed");
        assert!(size >= wire, "placement {size} fits wire {wire}");
        assert!(offset + size <= slab.stats().capacity);
    }
    // Telemetry matches the wire sum under default (wire) costs.
    let wire_sum: u64 = cache
        .keys()
        .iter()
        .map(|k| cache.get_ref(*k).expect("listed").encoded_bytes() as u64)
        .sum();
    assert_eq!(cache.telemetry().resident_bytes, wire_sum);
}

#[test]
fn cache_allocator_workload_stays_consistent() {
    let pool = page_pool();
    // Budget holds ~6 average pages: pressure is constant.
    let mut cache = ResidencyCache::new(24_000);
    let mut slab = SlabAllocator::new(1 << 20);
    let mut rng = 0xC0FFEEu64;

    for step in 0..1500 {
        let op = xorshift(&mut rng) % 100;
        let key = xorshift(&mut rng) % 12;
        let page_no = (xorshift(&mut rng) % 12) as usize;
        if op < 40 {
            // Insert (possibly replacing).
            let page = EncodedPage::encode(&pool[page_no], MODE);
            cache.insert(key, page);
        } else if op < 65 {
            // Lookup (hits and misses both count).
            cache.get(key);
        } else if op < 85 {
            // Update in place (same or new content).
            let field = &pool[(page_no + step as usize) % pool.len()];
            let _ = cache.update(key, field, MODE);
        } else if op < 92 {
            // Explicit evict (may miss).
            let _ = cache.evict(key);
        } else {
            // Compact the slab wholesale.
            let moves = slab.plan_compact();
            for mv in &moves {
                assert!(mv.to <= mv.from, "compaction only moves down");
            }
            slab.apply_compact(&moves);
        }
        assert!(sync(&cache, &mut slab), "step {step}: allocator OOM");
        check_all(&cache, &slab);
        // Cache never exceeds budget except via a single admitted page.
        let t = cache.telemetry();
        assert!(
            !t.over_budget || t.pages <= 1,
            "step {step}: over budget with {} pages",
            t.pages
        );
    }

    // Final state is non-trivial: pages resident, lookups happened.
    let t = cache.telemetry();
    assert!(t.pages > 0);
    assert!(t.hits + t.misses > 300);
    assert!(t.evictions > 0, "budget pressure must evict something");
}
