//! Per-primitive cost breakdown for both CBT models at depth 16 (65k leaves).
//! Run with `cargo bench -p thessa-rcbt-core --bench cbt_profile`.
//!
//! Isolates: split bit-write vs commit, merge, full leaf listing (decode-all),
//! single decode/encode. Points the optimizer at facts, not guesses.

use std::{hint::black_box, time::Instant};

use thessa_rcbt_core::{Node, Tree};
use thessa_rcbt_ffi::LibcbtTree;

const DEPTH: u8 = 16;

fn ms(f: impl FnOnce()) -> f64 {
    let t = Instant::now();
    f();
    t.elapsed().as_secs_f64() * 1000.0
}

fn main() {
    println!("| primitive | impl | n | ms_total | ns_per_op |");
    println!("|---|---|---|---|---|");

    // --- build a full d16 tree on both sides (excluded from primitive timing)
    let mut native = Tree::new(DEPTH).unwrap();
    for _ in 0..DEPTH {
        let batch: Vec<_> = native
            .leaves()
            .iter()
            .map(|l| thessa_rcbt_core::Update::Split(*l))
            .collect();
        native.apply_batch(&batch).unwrap();
    }
    let mut ffi = LibcbtTree::new(DEPTH).unwrap();
    ffi.reset_to_depth(DEPTH);
    ffi.reduce();
    assert_eq!(native.leaf_count(), ffi.node_count());
    let n = native.leaf_count();

    // --- 1. split commit: 4096 splits driven by a frame snapshot index.
    // (The old revision measured leaf_at here by mistake; random access
    // without a snapshot is O(subtree) per level by construction.)
    let mut t = Tree::at_depth(24, 12).unwrap();
    let snapshot = t.snapshot();
    let dt = ms(|| {
        for i in 0..4096 {
            if let Some(target) = snapshot.index(i % snapshot.len())
                && target.depth() < 24
            {
                let _ = t.split(target);
            }
        }
        black_box(t.leaf_count());
    });
    println!(
        "| split x4096 via snapshot | rust-native | 4096 | {dt:.2} | {:.0} |",
        dt * 1e6 / 4096.0
    );

    let mut f2 = LibcbtTree::at_depth(16, 12).unwrap();
    let dt = ms(|| {
        for i in 0..4096 {
            let (id, depth) = f2.decode(i % f2.node_count());
            if depth < 16 {
                f2.split(id, depth);
            }
        }
        f2.reduce();
        black_box(f2.node_count());
    });
    println!(
        "| split x4096 + 1 reduce | libcbt-c | 4096 | {dt:.2} | {:.0} |",
        dt * 1e6 / 4096.0
    );

    // --- 2. reduce-only cost on the C side at full size
    let dt = ms(|| {
        ffi.reduce();
        black_box(ffi.node_count());
    });
    println!(
        "| reduce (decode-pass + sum) | libcbt-c | {n} | {dt:.2} | {:.0} |",
        dt * 1e6 / n as f64
    );

    // --- 3. full leaf listing
    let dt = ms(|| {
        black_box(native.leaves());
    });
    println!(
        "| leaves() full | rust-native | {n} | {dt:.2} | {:.0} |",
        dt * 1e6 / n as f64
    );

    let dt = ms(|| {
        black_box(ffi.leaves());
    });
    println!(
        "| decode-all | libcbt-c | {n} | {dt:.2} | {:.0} |",
        dt * 1e6 / n as f64
    );

    // --- 4. single random decode / encode, with and without a snapshot
    let dt = ms(|| {
        for i in 0..10_000 {
            black_box(native.leaf_at((i * 7919) % n));
        }
    });
    println!(
        "| leaf_at x10k (no snapshot) | rust-native | 10000 | {dt:.2} | {:.0} |",
        dt * 1e6 / 10_000.0
    );

    let list = native.snapshot();
    let dt = ms(|| {
        for i in 0..10_000 {
            black_box(list.index((i * 7919) % n));
        }
    });
    println!(
        "| snapshot.index x10k | rust-native | 10000 | {dt:.2} | {:.0} |",
        dt * 1e6 / 10_000.0
    );

    let dt = ms(|| {
        for i in 0..10_000 {
            black_box(ffi.decode((i * 7919) % n));
        }
    });
    println!(
        "| decode x10k | libcbt-c | 10000 | {dt:.2} | {:.0} |",
        dt * 1e6 / 10_000.0
    );

    let sample: Vec<Node> = native
        .leaves()
        .into_iter()
        .step_by(61)
        .take(10_000)
        .collect();
    let dt = ms(|| {
        for leaf in &sample {
            let _ = black_box(native.encode_leaf(*leaf));
        }
    });
    println!(
        "| encode_leaf x{} (no snapshot) | rust-native | {} | {dt:.2} | {:.0} |",
        sample.len(),
        sample.len(),
        dt * 1e6 / sample.len() as f64
    );

    let dt = ms(|| {
        for leaf in &sample {
            black_box(list.encode(*leaf));
        }
    });
    println!(
        "| snapshot.encode x{} | rust-native | {} | {dt:.2} | {:.0} |",
        sample.len(),
        sample.len(),
        dt * 1e6 / sample.len() as f64
    );

    // --- 5. merge batch: merge 2048 pairs back (children of d15 parents)
    let mut t3 = Tree::at_depth(24, 16).unwrap();
    let parents: Vec<Node> = t3
        .leaves()
        .iter()
        .step_by(2)
        .filter_map(|l| l.parent())
        .take(2048)
        .collect();
    let dt = ms(|| {
        for p in &parents {
            let _ = t3.merge(*p);
        }
        black_box(t3.leaf_count());
    });
    println!(
        "| merge x{} | rust-native | {} | {dt:.2} | {:.0} |",
        parents.len(),
        parents.len(),
        dt * 1e6 / parents.len() as f64
    );

    let mut f3 = LibcbtTree::at_depth(16, 16).unwrap();
    f3.reduce();
    let dt = ms(|| {
        for i in (0..4096).step_by(2) {
            let (id, depth) = f3.decode(i % f3.node_count());
            // merge the pair containing this leaf via its parent
            if depth > 0 {
                f3.merge_children(id >> 1, depth - 1);
            }
        }
        f3.reduce();
        black_box(f3.node_count());
    });
    println!(
        "| merge-pairs x2048 + reduce | libcbt-c | 2048 | {dt:.2} | {:.0} |",
        dt * 1e6 / 2048.0
    );
}
