//! Checked game pose publication only, excluding physics, ephemerides, assets,
//! selection, GPU upload/rendering and presentation waits. Timer/barriers included.
use glam::{DQuat, DVec3};
use std::{hint::black_box, time::Instant};
use thessa_flight_authority::presentation::FlightPose;
use thessa_sim_core::SimTime;
fn main() {
    for count in [1, 1024] {
        let mut times = Vec::new();
        for sample in 0..1000 {
            let start = Instant::now();
            for i in 0..count {
                let pose = FlightPose::new(
                    SimTime(black_box(sample as f64)),
                    black_box(DVec3::new(3_200_000.125, i as f64, 100.0)),
                    black_box(DQuat::from_rotation_y(0.5)),
                )
                .unwrap();
                black_box((pose.render_position_m(), pose.visual_orientation()));
            }
            times.push(start.elapsed().as_nanos());
        }
        times.sort_unstable();
        println!(
            "checked poses={count} median_ns={} p95_ns={} n=1000",
            times[500], times[950]
        );
    }
}
