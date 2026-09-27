//! Main-world to render-world snapshot clone cost for CBT topology and pages.
//!
//! The owned baseline models the prior `Vec<CbtLeafRecord> +
//! BTreeMap<u64, (u64, HeightPage)>` extraction payload. The shared path uses
//! the production resource `Clone` implementations, matching Bevy's
//! `ExtractResource` boundary without requiring a GPU adapter.

use std::{collections::BTreeMap, hint::black_box, time::Instant};

use thessa_bevy_rcbt::{CbtRenderPages, CbtRenderTopology};
use thessa_rcbt_core::{HeightPage, Node};

const LEAF_COUNT: usize = 8_192;
const PAGE_COUNT: usize = 2_048;
const ITERATIONS: usize = 512;
const REPETITIONS: usize = 7;
const PAGE_GRID: u32 = 33;

struct SnapshotFixture {
    topology: CbtRenderTopology,
    pages: CbtRenderPages,
    owned_records: Vec<[u32; 4]>,
    owned_pages: BTreeMap<u64, (u64, HeightPage)>,
}

impl SnapshotFixture {
    fn new() -> Self {
        let depth = 14;
        let first_id = 1_u64 << depth;
        let nodes: Vec<_> = (0..LEAF_COUNT)
            .map(|index| Node::new(first_id + index as u64, depth).unwrap())
            .collect();
        let mut topology = CbtRenderTopology::default();
        assert!(topology.publish_resident_leaves(&nodes));

        let sample_count = (PAGE_GRID * PAGE_GRID) as usize;
        let samples: Vec<_> = (0..sample_count)
            .map(|index| 100.0 + (index as f64 * 0.071).sin() * 18.0)
            .collect();
        let source_page = HeightPage::bake(&samples, PAGE_GRID, 0.01).unwrap();
        let mut pages = CbtRenderPages::default();
        let mut owned_pages = BTreeMap::new();
        for index in 0..PAGE_COUNT {
            let id = first_id + index as u64;
            pages.set_page(id, source_page.clone());
            owned_pages.insert(id, (1, source_page.clone()));
        }

        Self {
            owned_records: topology.records().to_vec(),
            topology,
            pages,
            owned_pages,
        }
    }
}

fn shared_snapshot(fixture: &SnapshotFixture) {
    for _ in 0..ITERATIONS {
        black_box((fixture.topology.clone(), fixture.pages.clone()));
    }
}

fn owned_snapshot(fixture: &SnapshotFixture) {
    for _ in 0..ITERATIONS {
        black_box((fixture.owned_records.clone(), fixture.owned_pages.clone()));
    }
}

fn measure(run: impl Fn()) -> f64 {
    let started = Instant::now();
    run();
    started.elapsed().as_secs_f64() * 1.0e6 / ITERATIONS as f64
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

fn main() {
    let fixture = SnapshotFixture::new();
    // Warm allocator and code pages before comparing repeated frame snapshots.
    shared_snapshot(&fixture);
    owned_snapshot(&fixture);
    let shared = median(
        (0..REPETITIONS)
            .map(|_| measure(|| shared_snapshot(&fixture)))
            .collect(),
    );
    let owned = median(
        (0..REPETITIONS)
            .map(|_| measure(|| owned_snapshot(&fixture)))
            .collect(),
    );
    let residual_bytes = PAGE_COUNT * PAGE_GRID as usize * PAGE_GRID as usize * 2;
    println!(
        "leaves={LEAF_COUNT} height_pages={PAGE_COUNT} page_grid={PAGE_GRID} approx_residual_payload={residual_bytes} B"
    );
    println!("shared extraction snapshot clone: {shared:.4} µs/frame");
    println!("owned deep-copy baseline:           {owned:.2} µs/frame");
    println!("clone-cost reduction: {:.1}x", owned / shared);
}
