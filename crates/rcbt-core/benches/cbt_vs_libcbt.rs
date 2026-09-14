//! Upstream `libcbt` (C, vendored) vs native Rust `Tree` on identical workloads.
//! Run with `cargo bench -p thessa-rcbt-core --bench cbt_vs_libcbt`.
//!
//! Both implementations replay the same operation sequences:
//! split takes a leaf, merge takes the parent, each batch is committed
//! (native: immediate; libcbt: decode pass + sum reduction). Final leaf sets
//! are cross-checked for equality, so the table compares cost, not semantics.

use std::{hint::black_box, time::Instant};

use thessa_rcbt_core::{Node, Tree};
use thessa_rcbt_ffi::LibcbtTree;

#[derive(Debug, Clone, Copy)]
enum Op {
    Split { id: u64, depth: u8 },
    Merge { parent_id: u64, parent_depth: u8 },
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0 | 1;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

fn node(id: u64, depth: u8) -> Node {
    Node::new(id, depth).expect("generated node in range")
}

/// Level-by-level refinement batches from the root to `depth`.
fn gen_refine(depth: u8) -> Vec<Vec<Op>> {
    let mut driver = Tree::new(24).unwrap();
    let mut batches = Vec::new();
    for _ in 0..depth {
        let batch: Vec<Op> = driver
            .leaves()
            .iter()
            .map(|leaf| Op::Split {
                id: leaf.id(),
                depth: leaf.depth(),
            })
            .collect();
        driver
            .apply_batch(
                &batch
                    .iter()
                    .map(|op| match *op {
                        Op::Split { id, depth } => thessa_rcbt_core::Update::Split(node(id, depth)),
                        Op::Merge { .. } => unreachable!(),
                    })
                    .collect::<Vec<_>>(),
            )
            .unwrap();
        batches.push(batch);
    }
    batches
}

/// Sparse moving-camera-like mutation frames, size-bounded by merge pressure.
/// Split depth is capped so the live set fits a workload-sized libcbt heap.
fn gen_frames(
    base: u8,
    split_cap: u8,
    frames: usize,
    ops_per_frame: usize,
    seed: u64,
) -> Vec<Vec<Op>> {
    let mut driver = Tree::at_depth(24, base).unwrap();
    let mut rng = Rng(seed);
    let mut out = Vec::with_capacity(frames);
    for _ in 0..frames {
        let mut batch = Vec::new();
        let mut attempts = 0;
        while batch.len() < ops_per_frame && attempts < ops_per_frame * 6 {
            attempts += 1;
            let leaves = driver.leaves();
            let leaf = leaves[(rng.next() % leaves.len() as u64) as usize];
            let want_merge = rng.next().is_multiple_of(2) || leaves.len() > 4096;
            if want_merge {
                if let Some(parent) = leaf.parent()
                    && let Some([left, right]) = parent.children()
                    && driver.contains(left)
                    && driver.contains(right)
                {
                    driver.merge(parent).unwrap();
                    batch.push(Op::Merge {
                        parent_id: parent.id(),
                        parent_depth: parent.depth(),
                    });
                }
            } else if leaf.depth() < split_cap {
                driver.split(leaf).unwrap();
                batch.push(Op::Split {
                    id: leaf.id(),
                    depth: leaf.depth(),
                });
            }
        }
        out.push(batch);
    }
    out
}

fn gen_oscillate(depth: u8, rounds: usize) -> Vec<Vec<Op>> {
    let mut driver = Tree::at_depth(24, depth).unwrap();
    let mut out = Vec::with_capacity(rounds * 2);
    for _ in 0..rounds {
        let splits: Vec<Op> = driver
            .leaves()
            .iter()
            .map(|leaf| Op::Split {
                id: leaf.id(),
                depth: leaf.depth(),
            })
            .collect();
        for op in &splits {
            match *op {
                Op::Split { id, depth } => {
                    driver.split(node(id, depth)).unwrap();
                }
                Op::Merge { .. } => unreachable!(),
            }
        }
        out.push(splits);
        // Merge every pair back through its parent.
        let mut merges = Vec::new();
        for leaf in driver.leaves() {
            if leaf.id().is_multiple_of(2)
                && let Some(parent) = leaf.parent()
            {
                driver.merge(parent).unwrap();
                merges.push(Op::Merge {
                    parent_id: parent.id(),
                    parent_depth: parent.depth(),
                });
            }
        }
        out.push(merges);
    }
    out
}

fn apply_native(tree: &mut Tree, batches: &[Vec<Op>]) -> usize {
    let mut ops = 0;
    for batch in batches {
        for op in batch {
            match *op {
                Op::Split { id, depth } => {
                    tree.split(node(id, depth)).unwrap();
                }
                Op::Merge {
                    parent_id,
                    parent_depth,
                } => {
                    tree.merge(node(parent_id, parent_depth)).unwrap();
                }
            }
            ops += 1;
        }
        black_box(tree.leaf_count());
    }
    ops
}

fn apply_libcbt(tree: &mut LibcbtTree, batches: &[Vec<Op>]) -> usize {
    let mut ops = 0;
    for batch in batches {
        for op in batch {
            match *op {
                Op::Split { id, depth } => tree.split(id, depth),
                Op::Merge {
                    parent_id,
                    parent_depth,
                } => tree.merge_children(parent_id, parent_depth),
            }
            ops += 1;
        }
        tree.reduce();
        black_box(tree.node_count());
    }
    ops
}

fn check_parity(native: &Tree, ffi: &LibcbtTree, what: &str) {
    let expected: Vec<(u64, u8)> = native
        .leaves()
        .iter()
        .map(|n| (n.id(), n.depth()))
        .collect();
    assert_eq!(
        native.leaf_count(),
        ffi.node_count(),
        "leaf count parity: {what}"
    );
    assert_eq!(expected, ffi.leaves(), "leaf set parity: {what}");
}

fn report(workload: &str, impl_name: &str, leaves: usize, ops: usize, ms: f64) {
    println!(
        "| {workload} | {impl_name} | {leaves} | {ops} | {ms:.1} | {:.0} |",
        ops as f64 / ms.max(1e-9) * 1000.0,
    );
}

fn main() {
    println!("| workload | impl | leaves | ops | ms | ops_per_s |");
    println!("|---|---|---|---|---|---|");

    // 1. Full refinement to increasing depths (4k -> 262k leaves).
    for depth in [12_u8, 14, 16, 18] {
        let batches = gen_refine(depth);
        let total_ops: usize = batches.iter().map(Vec::len).sum();

        let mut native = Tree::new(depth).unwrap();
        let started = Instant::now();
        apply_native(&mut native, &batches);
        let native_ms = started.elapsed().as_secs_f64() * 1000.0;

        let mut ffi = LibcbtTree::new(depth).unwrap();
        let started = Instant::now();
        apply_libcbt(&mut ffi, &batches);
        let ffi_ms = started.elapsed().as_secs_f64() * 1000.0;

        check_parity(&native, &ffi, &format!("refine/{depth}"));
        let name = format!("refine/d{depth}");
        report(
            &name,
            "rust-native",
            native.leaf_count(),
            total_ops,
            native_ms,
        );
        report(&name, "libcbt-c", ffi.node_count(), total_ops, ffi_ms);
        println!(
            "| {name} | speedup(native/libcbt) x{:.2} |",
            ffi_ms / native_ms.max(1e-9)
        );
    }

    // 2. Sparse mutation frames at larger scale than tree.rs.
    // Each side is sized as a competent user would size it: the native tree
    // is adaptive (max depth is not a cost driver), while the libcbt heap is
    // fixed at 2^(max-1) bytes, so it gets a workload-fitted max depth of 16
    // (32 KiB heap for a live set bounded near 4k leaves at depth <= 14).
    let frames = gen_frames(10, 14, 2000, 32, 0x9E3779B97F4A7C15);
    let total_ops: usize = frames.iter().map(Vec::len).sum();

    let mut native = Tree::at_depth(24, 10).unwrap();
    let started = Instant::now();
    apply_native(&mut native, &frames);
    let native_ms = started.elapsed().as_secs_f64() * 1000.0;

    let mut ffi = LibcbtTree::at_depth(16, 10).unwrap();
    let started = Instant::now();
    apply_libcbt(&mut ffi, &frames);
    let ffi_ms = started.elapsed().as_secs_f64() * 1000.0;

    check_parity(&native, &ffi, "frames/2000x32");
    report(
        "frames/2000x32",
        "rust-native",
        native.leaf_count(),
        total_ops,
        native_ms,
    );
    report(
        "frames/2000x32",
        "libcbt-c",
        ffi.node_count(),
        total_ops,
        ffi_ms,
    );
    println!(
        "| frames/2000x32 | speedup(native/libcbt) x{:.2} |",
        ffi_ms / native_ms.max(1e-9)
    );

    // 3. Full decode of every leaf (leaf-list construction).
    // libcbt gets the minimal fitting heap (decode cost is per-leaf there).
    for depth in [14_u8, 16, 18] {
        let mut native = Tree::at_depth(24, depth).unwrap();
        black_box(native.leaf_count());
        let started = Instant::now();
        let native_leaves = native.leaves();
        black_box(native_leaves.len());
        let native_ms = started.elapsed().as_secs_f64() * 1000.0;

        let ffi = LibcbtTree::at_depth(depth, depth).unwrap();
        let started = Instant::now();
        let ffi_leaves = ffi.leaves();
        black_box(ffi_leaves.len());
        let ffi_ms = started.elapsed().as_secs_f64() * 1000.0;

        let expected: Vec<(u64, u8)> = native
            .leaves()
            .iter()
            .map(|n| (n.id(), n.depth()))
            .collect();
        assert_eq!(expected, ffi_leaves, "decode parity d{depth}");
        let name = format!("decode-all/d{depth}");
        report(
            &name,
            "rust-native",
            expected.len(),
            expected.len(),
            native_ms,
        );
        report(
            &name,
            "libcbt-c",
            ffi_leaves.len(),
            ffi_leaves.len(),
            ffi_ms,
        );
        println!(
            "| {name} | speedup(native/libcbt) x{:.2} |",
            ffi_ms / native_ms.max(1e-9)
        );
        let _ = &mut native;
    }

    // 4. Split/merge oscillation over a 4k-leaf tree (workload-fitted heaps).
    let rounds = gen_oscillate(12, 20);
    let total_ops: usize = rounds.iter().map(Vec::len).sum();

    let mut native = Tree::at_depth(24, 12).unwrap();
    let started = Instant::now();
    apply_native(&mut native, &rounds);
    let native_ms = started.elapsed().as_secs_f64() * 1000.0;

    let mut ffi = LibcbtTree::at_depth(14, 12).unwrap();
    let started = Instant::now();
    apply_libcbt(&mut ffi, &rounds);
    let ffi_ms = started.elapsed().as_secs_f64() * 1000.0;

    check_parity(&native, &ffi, "oscillate/20r");
    report(
        "oscillate/20r",
        "rust-native",
        native.leaf_count(),
        total_ops,
        native_ms,
    );
    report(
        "oscillate/20r",
        "libcbt-c",
        ffi.node_count(),
        total_ops,
        ffi_ms,
    );
    println!(
        "| oscillate/20r | speedup(native/libcbt) x{:.2} |",
        ffi_ms / native_ms.max(1e-9)
    );
}
