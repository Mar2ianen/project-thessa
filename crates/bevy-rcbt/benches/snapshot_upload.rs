//! Snapshot publish parity through the own backend (audit R2/S3).
//!
//! Topology records + page bytes upload via `thessa-render-wgpu` and read
//! back; asserts byte equality with the CPU source — the same slices the
//! Bevy render world `write_buffer`s today — and reports upload GB/s.
//! Run with `cargo bench -p thessa-bevy-rcbt --bench snapshot_upload`.
//! Prints SKIP without a GPU adapter.

use std::{hint::black_box, time::Instant};

use thessa_bevy_rcbt::{CbtRenderPages, CbtRenderTopology};
use thessa_rcbt_core::{HeightPage, Node};
use thessa_render_wgpu::Backend;

// Mirrors `extraction_snapshots.rs` so numbers compare across benches.
const LEAF_COUNT: usize = 8_192;
const PAGE_COUNT: usize = 2_048;
const PAGE_GRID: u32 = 33;
const ITERS: usize = 20;

fn main() {
    let depth = 14;
    let first_id = 1_u64 << depth;
    let nodes: Vec<_> = (0..LEAF_COUNT)
        .map(|index| Node::new(first_id + index as u64, depth).unwrap())
        .collect();
    let mut topology = CbtRenderTopology::default();
    assert!(topology.publish_resident_leaves(&nodes));
    let records: Vec<u8> = topology
        .records()
        .iter()
        .flat_map(|word| word.iter().flat_map(|w| w.to_le_bytes()))
        .collect();

    let sample_count = (PAGE_GRID * PAGE_GRID) as usize;
    let samples: Vec<_> = (0..sample_count)
        .map(|index| 100.0 + (index as f64 * 0.071).sin() * 18.0)
        .collect();
    let source_page = HeightPage::bake(&samples, PAGE_GRID, 0.01).unwrap();
    let mut pages = CbtRenderPages::default();
    for index in 0..PAGE_COUNT {
        pages.set_page(first_id + index as u64, source_page.clone());
    }
    assert_eq!(pages.len(), PAGE_COUNT);
    let one_page = source_page.to_bytes();
    let page_bytes: Vec<u8> = (0..PAGE_COUNT)
        .flat_map(|_| one_page.iter().copied())
        .collect();

    let backend = match Backend::bring_up() {
        Ok(backend) => backend,
        Err(thessa_render_wgpu::BackendError::NoAdapter) => {
            println!("SKIP: no GPU adapter available");
            return;
        }
        Err(other) => panic!("bring-up failed: {other}"),
    };
    eprintln!("adapter: {}", backend.adapter_name);
    // Warm up (first upload shakes out allocator + first-map paths).
    black_box(backend.roundtrip(black_box(&records)).expect("warmup runs"));
    black_box(
        backend
            .roundtrip(black_box(&page_bytes))
            .expect("warmup runs"),
    );
    let started = Instant::now();
    for _ in 0..ITERS {
        let back_records = backend
            .roundtrip(black_box(&records))
            .expect("records roundtrip");
        assert_eq!(back_records, records, "topology byte parity");
        let back_pages = backend
            .roundtrip(black_box(&page_bytes))
            .expect("pages roundtrip");
        assert_eq!(back_pages, page_bytes, "page byte parity");
    }
    let secs = started.elapsed().as_secs_f64() / ITERS as f64;
    let bytes = (records.len() + page_bytes.len()) as f64;
    println!(
        "snapshot_upload records={}B pages={}B: {:.3} ms/iter ({:.2} GB/s)",
        records.len(),
        page_bytes.len(),
        secs * 1000.0,
        bytes / secs / 1e9,
    );
}
