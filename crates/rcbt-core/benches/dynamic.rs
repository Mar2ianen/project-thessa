//! Dynamic LOD scenarios: animated wave heightfield drives split/merge.
//! Run with `cargo bench -p thessa-rcbt-core --bench dynamic`.
//!
//! This is the workload CBT was born for (Dupuy-style dynamic tessellation:
//! ocean waves, animated terrain): the error metric itself moves every
//! frame, so the tree must chase it. Two scenarios on identical op streams:
//! pure waves, and waves plus a sweeping camera focus. Columns per scenario:
//! cpu-native / cpu-packed / libcbt-public / libcbt-sparse, parity asserted.

use std::{f64::consts::TAU, hint::black_box, time::Instant};

use thessa_rcbt_core::packed::PackedTree;
use thessa_rcbt_core::{Node, Tree};
use thessa_rcbt_ffi::LibcbtTree;

const MAX_DEPTH: u8 = 16;
const BASE_DEPTH: u8 = 10;
const MIN_DEPTH: u8 = 8;
const FRAMES: usize = 120;
const SPLIT_EPS: f64 = 1e-5;
const MERGE_EPS: f64 = 2.5e-6;

#[derive(Debug, Clone, Copy)]
enum Op {
    Split { id: u64, depth: u8 },
    Merge { parent_id: u64, parent_depth: u8 },
}

fn node(id: u64, depth: u8) -> Node {
    Node::new(id, depth).expect("bench node in range")
}

/// Analytic dynamic heightfield: three directional waves, deterministic.
fn wave_h(x: f64, t: f64) -> f64 {
    (TAU * 3.0 * x + 0.9 * t).sin()
        + 0.35 * (TAU * 7.0 * x - 1.7 * t + 1.3).sin()
        + 0.12 * (TAU * 13.0 * x + 2.6 * t + 4.1).sin()
}

/// Linear-interpolation error of a leaf interval, optionally focused by a
/// sweeping camera (error weight grows near the camera center).
fn leaf_err(id: u64, depth: u8, t: f64, cam: Option<f64>) -> f64 {
    let w = 2f64.powi(-(depth as i32));
    let a = (id - (1u64 << depth)) as f64 * w;
    let m = a + 0.5 * w;
    let e = (wave_h(m, t) - 0.5 * (wave_h(a, t) + wave_h(a + w, t))).abs();
    match cam {
        Some(c) => e / (0.05 + (m - c).abs()),
        None => e,
    }
}

/// Drive frames against the packed oracle, recording validated op batches.
/// Decisions come from one implementation only, so every replayed column
/// sees bit-identical input.
fn drive_waves(camera_sweep: bool) -> Vec<Vec<Op>> {
    let mut driver = PackedTree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
    let mut out = Vec::with_capacity(FRAMES);
    for f in 0..FRAMES {
        let t = f as f64 / 60.0;
        let cam = camera_sweep.then(|| f as f64 / FRAMES as f64);
        let mut batch = Vec::new();
        let leaves = driver.leaves();
        for leaf in &leaves {
            if leaf.depth() < MAX_DEPTH && leaf_err(leaf.id(), leaf.depth(), t, cam) > SPLIT_EPS {
                batch.push(Op::Split {
                    id: leaf.id(),
                    depth: leaf.depth(),
                });
            }
        }
        // Merge pass over even-id leaves whose sibling is also a leaf and
        // both are comfortably below threshold.
        for leaf in &leaves {
            if leaf.id() & 1 == 1 {
                continue;
            }
            let sib = node(leaf.id() + 1, leaf.depth());
            if !driver.contains(sib) || leaf.depth() == 0 {
                continue;
            }
            let parent = node(leaf.id() >> 1, leaf.depth() - 1);
            if parent.depth() < MIN_DEPTH {
                continue;
            }
            if leaf_err(leaf.id(), leaf.depth(), t, cam) < MERGE_EPS
                && leaf_err(sib.id(), sib.depth(), t, cam) < MERGE_EPS
            {
                batch.push(Op::Merge {
                    parent_id: parent.id(),
                    parent_depth: parent.depth(),
                });
            }
        }
        for op in &batch {
            match *op {
                Op::Split { id, depth } => {
                    driver.split(node(id, depth)).unwrap();
                }
                Op::Merge {
                    parent_id,
                    parent_depth,
                } => driver.merge(node(parent_id, parent_depth)).unwrap(),
            }
        }
        black_box(driver.leaf_count());
        out.push(batch);
    }
    out
}

fn main() {
    println!("| scenario | impl | frames | avg_ops_frame | ms | parity |");
    println!("|---|---|---|---|---|---|");
    for camera_sweep in [false, true] {
        let name = if camera_sweep {
            "waves+camera"
        } else {
            "waves"
        };
        let frames = drive_waves(camera_sweep);
        let total_ops: usize = frames.iter().map(Vec::len).sum();
        let avg = total_ops as f64 / FRAMES as f64;
        // Expected final state from an independent oracle replay.
        let mut oracle = PackedTree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
        for batch in &frames {
            for op in batch {
                match *op {
                    Op::Split { id, depth } => {
                        oracle.split(node(id, depth)).unwrap();
                    }
                    Op::Merge {
                        parent_id,
                        parent_depth,
                    } => oracle.merge(node(parent_id, parent_depth)).unwrap(),
                }
            }
        }
        let expected: Vec<(u64, u8)> = oracle
            .leaves()
            .iter()
            .map(|l| (l.id(), l.depth()))
            .collect();

        let mut tree = Tree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
        let t = Instant::now();
        for batch in &frames {
            for op in batch {
                match *op {
                    Op::Split { id, depth } => {
                        tree.split(node(id, depth)).unwrap();
                    }
                    Op::Merge {
                        parent_id,
                        parent_depth,
                    } => tree.merge(node(parent_id, parent_depth)).unwrap(),
                }
            }
            black_box(tree.leaf_count());
        }
        let native_ms = t.elapsed().as_secs_f64() * 1000.0;
        let got: Vec<(u64, u8)> = tree.leaves().iter().map(|l| (l.id(), l.depth())).collect();
        assert_eq!(got, expected, "{name} cpu-native parity");

        let mut packed = PackedTree::at_depth(MAX_DEPTH, BASE_DEPTH).unwrap();
        let t = Instant::now();
        for batch in &frames {
            for op in batch {
                match *op {
                    Op::Split { id, depth } => {
                        packed.split(node(id, depth)).unwrap();
                    }
                    Op::Merge {
                        parent_id,
                        parent_depth,
                    } => packed.merge(node(parent_id, parent_depth)).unwrap(),
                }
            }
            black_box(packed.leaf_count());
        }
        let packed_ms = t.elapsed().as_secs_f64() * 1000.0;
        let got: Vec<(u64, u8)> = packed
            .leaves()
            .iter()
            .map(|l| (l.id(), l.depth()))
            .collect();
        assert_eq!(got, expected, "{name} cpu-packed parity");

        let mut public = LibcbtTree::new(MAX_DEPTH).unwrap();
        public.reset_to_depth(BASE_DEPTH);
        public.reduce();
        let t = Instant::now();
        for batch in &frames {
            for op in batch {
                match *op {
                    Op::Split { id, depth } => public.split(id, depth),
                    Op::Merge {
                        parent_id,
                        parent_depth,
                    } => public.merge_children(parent_id, parent_depth),
                }
            }
            public.reduce();
            black_box(public.node_count());
        }
        let public_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(public.leaves(), expected, "{name} libcbt-public parity");

        let mut sparse = LibcbtTree::new(MAX_DEPTH).unwrap();
        sparse.reset_to_depth(BASE_DEPTH);
        sparse.reduce_only();
        let t = Instant::now();
        for batch in &frames {
            let packed_ops: Vec<(u64, i64, u8)> = batch
                .iter()
                .map(|op| match *op {
                    Op::Split { id, depth } => (id, depth as i64, 0),
                    Op::Merge {
                        parent_id,
                        parent_depth,
                    } => (parent_id, parent_depth as i64, 1),
                })
                .collect();
            sparse.apply_batch_ops(&packed_ops);
            sparse.reduce_only();
            black_box(sparse.node_count());
        }
        let sparse_ms = t.elapsed().as_secs_f64() * 1000.0;
        assert_eq!(sparse.leaves(), expected, "{name} libcbt-sparse parity");

        println!("| {name} | cpu-native | {FRAMES} | {avg:.0} | {native_ms:.2} | OK |");
        println!("| {name} | cpu-packed | {FRAMES} | {avg:.0} | {packed_ms:.2} | OK |");
        println!("| {name} | libcbt-public | {FRAMES} | {avg:.0} | {public_ms:.2} | OK |");
        println!("| {name} | libcbt-sparse | {FRAMES} | {avg:.0} | {sparse_ms:.2} | OK |");
        println!("| {name} | leaves_final | {} |", expected.len());
    }
}
