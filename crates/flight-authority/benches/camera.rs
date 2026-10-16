//! Shared pilot-camera offsets only, excluding physics, ephemerides, assets,
//! selection, GPU upload/rendering and presentation waits. Timer/barriers included.
use glam::{Quat, Vec3};
use std::{hint::black_box, time::Instant};
use thessa_flight_authority::camera::FlightCamera;
fn main() {
    for count in [1, 1024] {
        let mut times = Vec::new();
        for sample in 0..1000 {
            let mut camera = FlightCamera::pilot_default();
            camera.orbit_yaw(black_box(sample as f32) * 0.001);
            let start = Instant::now();
            for i in 0..count {
                let orientation = Quat::from_axis_angle(
                    black_box(Vec3::new(0.3, 0.8, -0.5).normalize()),
                    0.02 * i as f32,
                );
                black_box(camera.offsets(orientation, black_box(Vec3::NEG_Z)));
            }
            times.push(start.elapsed().as_nanos());
        }
        times.sort_unstable();
        println!(
            "camera_offsets={count} median_ns={} p95_ns={} n=1000",
            times[500], times[950]
        );
    }
}
