use glam::{DMat4, DQuat, DVec3};
use std::{hint::black_box, time::Instant};
use thessa_render_core::{BodyFrame, ViewParams};

fn main() {
    let center = DVec3::new(1.5e11, -1e12, 2e12);
    let spin = DQuat::from_rotation_y(0.923);
    let frame = BodyFrame::new(center, spin, center + DVec3::splat(128.0)).unwrap();
    let points: Vec<_> = (0..1024)
        .map(|i| DVec3::new(i as f64, 3_200_000.0, -10.0))
        .collect();
    let view = ViewParams {
        world_from_view: DMat4::from_rotation_translation(spin, DVec3::splat(3200.0)),
        world_position_m: DVec3::splat(3200.0),
        clip_from_view: DMat4::IDENTITY,
        exposure: 1.0,
    };
    for (label, mode) in [
        ("1024 body-to-render points (shared frame)", 0),
        ("1024 body-to-render points (direct arithmetic)", 1),
        ("one render-to-body camera", 2),
    ] {
        let mut times = Vec::with_capacity(10000);
        for _ in 0..10000 {
            let start = Instant::now();
            if mode == 0 {
                for &point in &points {
                    black_box(black_box(&frame).body_to_render_point(black_box(point)));
                }
            } else if mode == 1 {
                for &point in &points {
                    black_box(black_box(spin) * black_box(point) - black_box(DVec3::splat(128.0)));
                }
            } else {
                black_box(black_box(&frame).view_in_body(black_box(&view)));
            }
            times.push(start.elapsed().as_nanos());
        }
        times.sort_unstable();
        println!(
            "{label}: n=10000 median_ns={} p95_ns={}",
            times[5000], times[9500]
        );
    }
}
