//! Repeatable moving-camera-like sparse mutation benchmark.
//! Run with `cargo bench -p thessa-rcbt-core --bench tree`.

use std::{hint::black_box, time::Instant};

use thessa_rcbt_core::{CandidateAction, FrameBudget, LeafCandidate, Tree, WorkClass, plan_frame};

fn main() {
    let mut tree = Tree::at_depth(20, 10).expect("benchmark tree");
    let started = Instant::now();
    let frames = 3000;
    let mut operations = 0;
    for frame in 0..frames {
        let leaves = tree.leaves();
        let candidates = leaves
            .iter()
            .copied()
            .take(128)
            .enumerate()
            .map(|(index, node)| LeafCandidate {
                node,
                action: if (index + frame) % 5 == 0 {
                    CandidateAction::Merge
                } else {
                    CandidateAction::Split
                },
                class: if index < 4 {
                    WorkClass::VisibleGeometry
                } else {
                    WorkClass::PredictedGeometry
                },
                projected_error_px: 4.0 + (index % 7) as f32,
                predicted_error_px: 2.0,
                time_to_needed_s: 0.25 + index as f32 * 0.1,
            });
        let plan = plan_frame(&tree, candidates, FrameBudget { max_operations: 32 });
        operations += plan.updates().len();
        tree.apply_batch(plan.updates())
            .expect("planned operations");
        black_box(tree.leaf_count());
    }
    let elapsed = started.elapsed().as_secs_f64();
    println!(
        "frames={frames} operations={operations} updates/s={:.1} final_leaves={}",
        frames as f64 / elapsed,
        tree.leaf_count()
    );
}
