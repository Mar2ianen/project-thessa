//! Sparse dirty-path Rust CBT versus the upstream large_cbt OCBT reduction.
//!
//! Run with:
//! `cargo bench -p thessa-rcbt-core --bench cbt_vs_large`
//!
//! The comparison is intentionally explicit about the different contracts:
//! our packed tree updates only changed paths, while large_cbt's CPU reference
//! rebuilds its packed sums from the dense bitfield. The GPU port will use the
//! same two upstream buffers but move allocation/propagation to WGSL.

use std::{collections::HashMap, hint::black_box, time::Instant};

use thessa_rcbt_core::{BisectorPool, Node, Tree, compact::CompactTree, packed::PackedTree};
use thessa_rcbt_ffi::LibcbtTree;
use thessa_rcbt_large_ffi::{LargeOcbt, Variant};

const FRAMES: usize = 256;
const OPS_PER_FRAME: usize = 64;

#[derive(Clone, Copy)]
struct BitToggle {
    bit: usize,
    state: bool,
}

fn toggles(elements: usize) -> Vec<Vec<BitToggle>> {
    let mut seed = 0x7e57_5eed_u64;
    let mut result = Vec::with_capacity(FRAMES);
    for _ in 0..FRAMES {
        let mut frame = Vec::with_capacity(OPS_PER_FRAME);
        for _ in 0..OPS_PER_FRAME {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            frame.push(BitToggle {
                bit: (seed as usize) % elements,
                state: seed & 1 != 0,
            });
        }
        result.push(frame);
    }
    result
}

fn sparse_nodes() -> Vec<Vec<(Node, bool)>> {
    let mut driver = PackedTree::at_depth(20, 12).unwrap();
    let mut seed = 0x51a7_1ce5_u64;
    let mut result = Vec::with_capacity(FRAMES);
    for _ in 0..FRAMES {
        let mut frame = Vec::with_capacity(OPS_PER_FRAME);
        let mut attempts = 0;
        while frame.len() < OPS_PER_FRAME && attempts < OPS_PER_FRAME * 12 {
            attempts += 1;
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let leaves = driver.leaves();
            let leaf = leaves[(seed as usize) % leaves.len()];
            if seed & 1 != 0 && leaf.depth() < 20 {
                driver.split(leaf).unwrap();
                frame.push((leaf, true));
            } else if let Some(parent) = leaf.parent()
                && let Some([left, right]) = parent.children()
                && driver.contains(left)
                && driver.contains(right)
            {
                driver.merge(parent).unwrap();
                frame.push((parent, false));
            }
        }
        result.push(frame);
    }
    result
}

fn report(name: &str, started: Instant, operations: usize) {
    let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
    println!(
        "| {name} | {operations} | {elapsed_ms:.2} | {:.0} |",
        operations as f64 / elapsed_ms.max(0.000_001) * 1000.0
    );
}

fn node_key(node: Node) -> u64 {
    (node.id() << 6) | u64::from(node.depth())
}

fn main() {
    println!("| implementation | operations | ms | operations/s |");
    println!("|---|---:|---:|---:|");

    let bit_workload = toggles(Variant::Ocbt1m.num_elements());
    let mut large = LargeOcbt::new(Variant::Ocbt1m).unwrap();
    // Warm the dense reference with a stable population; setup is outside the
    // frame comparison so the timed region measures sparse updates + reduce.
    for bit in 0..8192 {
        large.set_bit(bit, true);
    }
    large.reduce();
    let started = Instant::now();
    let mut large_operations = 0;
    for frame in &bit_workload {
        for toggle in frame {
            large.set_bit(toggle.bit, toggle.state);
            large_operations += 1;
        }
        large.reduce();
        black_box(large.bit_count());
    }
    report(
        "large_cbt OCBT-1M set-bit + full reduce",
        started,
        large_operations,
    );

    let workloads = sparse_nodes();

    // Compare the existing BTreeSet topology with the fixed-capacity pool on
    // the same bounded split/merge sequence. The pool benchmark uses cached
    // handles, as an adapter would; resolving a node by scanning every slot
    // is intentionally a separate convenience API and is not the hot path.
    let mut tree = Tree::at_depth(20, 12).unwrap();
    let started = Instant::now();
    let mut tree_operations = 0;
    for frame in &workloads {
        for &(node, split) in frame {
            if split {
                tree.split(node).unwrap();
            } else {
                tree.merge(node).unwrap();
            }
            tree_operations += 1;
        }
        black_box(tree.leaf_count());
    }
    report("thessa Tree BTreeSet topology", started, tree_operations);

    let mut pool = BisectorPool::from_tree(&Tree::at_depth(20, 12).unwrap(), 32_768).unwrap();
    let mut handles = HashMap::with_capacity(pool.active_count() * 2);
    for handle in pool.handles() {
        let node = pool.get(handle).expect("active pool handle").node();
        handles.insert(node_key(node), handle);
    }
    let started = Instant::now();
    let mut pool_operations = 0;
    for frame in &workloads {
        for &(node, split) in frame {
            if split {
                let handle = handles.remove(&node_key(node)).expect("split handle");
                let [left_handle, right_handle] = pool.split(handle).unwrap();
                let [left, right] = node.children().unwrap();
                handles.insert(node_key(left), left_handle);
                handles.insert(node_key(right), right_handle);
            } else {
                let [left, right] = node.children().unwrap();
                let left_handle = handles.remove(&node_key(left)).expect("left handle");
                let right_handle = handles.remove(&node_key(right)).expect("right handle");
                let parent_handle = pool.merge_handles(node, left_handle, right_handle).unwrap();
                handles.insert(node_key(node), parent_handle);
            }
            pool_operations += 1;
        }
        black_box(pool.active_count());
    }
    report(
        "thessa BisectorPool fixed slots + cached handles",
        started,
        pool_operations,
    );
    assert_eq!(pool.nodes(), tree.leaves());
    assert!(pool.is_valid_partition());

    let mut libcbt = LibcbtTree::at_depth(20, 12).unwrap();
    let started = Instant::now();
    let mut libcbt_operations = 0;
    for frame in &workloads {
        for &(node, split) in frame {
            if split {
                libcbt.split(node.id(), node.depth());
            } else {
                libcbt.merge_children(node.id(), node.depth());
            }
            libcbt_operations += 1;
        }
        libcbt.reduce();
        black_box(libcbt.node_count());
    }
    report(
        "libcbt sparse split/merge + full reduce",
        started,
        libcbt_operations,
    );

    let mut packed = PackedTree::at_depth(20, 12).unwrap();
    let started = Instant::now();
    let mut packed_operations = 0;
    for frame in &workloads {
        for &(node, split) in frame {
            if split {
                packed.split(node).unwrap();
            } else {
                packed.merge(node).unwrap();
            }
            packed_operations += 1;
        }
        black_box(packed.leaf_count());
    }
    report("thessa PackedTree dirty-path", started, packed_operations);

    let mut compact = CompactTree::at_depth(20, 12).unwrap();
    let started = Instant::now();
    let mut compact_operations = 0;
    for frame in &workloads {
        for &(node, split) in frame {
            if split {
                compact.split(node).unwrap();
            } else {
                compact.merge(node).unwrap();
            }
            compact_operations += 1;
        }
        black_box(compact.leaf_count());
    }
    report(
        "thessa CompactTree exact packed dirty-path",
        started,
        compact_operations,
    );

    println!(
        "large_cbt OCBT-1M footprint: {} bytes (tree {} + bitfield {})",
        large.memory_footprint(),
        large.buffer_size(0),
        large.buffer_size(1)
    );
    println!(
        "thessa PackedTree footprint: {} bytes",
        packed.footprint_bytes()
    );
    println!(
        "thessa CompactTree footprint: {} bytes",
        compact.footprint_bytes()
    );
}
