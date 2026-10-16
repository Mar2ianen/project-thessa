//! Shared flight-input intents only, excluding devices, authority stepping,
//! ephemerides, assets, streaming, GPU work and waits. Timer/barriers included.
use std::{hint::black_box, time::Instant};
use thessa_flight_authority::input::{FlightInput, StickKeys, slew_throttle, stick_input};
fn main() {
    for count in [1, 1024] {
        let mut times = Vec::new();
        for sample in 0..1000 {
            let keys = StickKeys {
                nose_up: black_box(sample) % 2 == 0,
                yaw_right: black_box(sample) % 3 == 0,
                ..StickKeys::default()
            };
            let start = Instant::now();
            for i in 0..count {
                let stick = stick_input(black_box(keys));
                let throttle =
                    slew_throttle(black_box(0.3 + i as f64 * 0.0001), 1.0 / 60.0, true, false);
                black_box((stick, throttle));
            }
            times.push(start.elapsed().as_nanos());
        }
        times.sort_unstable();
        println!(
            "flight_inputs={count} median_ns={} p95_ns={} n=1000",
            times[500], times[950]
        );
    }
    let mut input = FlightInput::new(0.2, false).unwrap();
    let start = Instant::now();
    for _ in 0..100_000 {
        input.slew(1.0 / 60.0, true, false);
        black_box(input.stick_scaled());
    }
    println!(
        "bundled_push_100k_ns={} throttle={}",
        start.elapsed().as_nanos(),
        input.throttle
    );
}
