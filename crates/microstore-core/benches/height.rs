//! Phase E height benchmark (doc §18): real Thessa grids x modes with
//! throughput, residency vs raw f32, error, normal-angle, and shared-edge
//! crack columns.
//!
//! Run with `cargo bench -p thessa-microstore-core --bench height`.

use std::{hint::black_box, time::Instant};

use thessa_microstore_core::{
    HeightGrid, HeightMode, HeightPage,
    height::{
        latlon_window_spacing_m, shared_edge_error, thessa_height_coast, thessa_height_mountain,
        thessa_height_ocean, verify,
    },
};

const ITERS: usize = 100;

/// Physical spacing for a vendored window (see tests/assets README).
fn spacing(lat: f64) -> [f32; 2] {
    latlon_window_spacing_m(lat, 2.0, 65, 3_200_000.0)
}

fn bench_grid(name: &str, lat_deg: f64, grid: &HeightGrid) {
    println!("height {name} {}x{}:", grid.width, grid.height);
    println!(
        "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
        "mode", "ms/enc", "ms/dec", "MiB/s", "B/tex", "maxerr-m", "nrm-deg"
    );
    let modes: Vec<(&str, HeightMode)> = vec![
        ("raw32", HeightMode::Raw32),
        ("r16", HeightMode::Residual16),
        ("r8", HeightMode::Residual8),
        (
            "adapt1",
            HeightMode::Adaptive {
                max_abs_error_m: 1.0,
            },
        ),
        (
            "adapt10",
            HeightMode::Adaptive {
                max_abs_error_m: 10.0,
            },
        ),
        (
            "adapt100",
            HeightMode::Adaptive {
                max_abs_error_m: 100.0,
            },
        ),
    ];
    for (mode_name, mode) in &modes {
        let page = HeightPage::encode(grid, *mode);
        let decoded = page.decode();
        let v = verify(grid, &decoded, page.base, page.ceiling, spacing(lat_deg));

        let started = Instant::now();
        for _ in 0..ITERS {
            let page = HeightPage::encode(black_box(grid), *mode);
            black_box(page);
        }
        let enc_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;

        let page = HeightPage::encode(grid, *mode);
        let started = Instant::now();
        for _ in 0..ITERS {
            let decoded = black_box(&page).decode();
            black_box(decoded);
        }
        let dec_ms = started.elapsed().as_secs_f64() * 1000.0 / ITERS as f64;

        let texels = (grid.width as f64) * (grid.height as f64);
        let mib_s = texels * 2.0 / ((enc_ms + dec_ms) / 1000.0) / 1_048_576.0;
        println!(
            "{:>10} {:>10.3} {:>10.3} {:>10.1} {:>10.3} {:>10.2} {:>10.3}",
            mode_name,
            enc_ms,
            dec_ms,
            mib_s,
            page.bytes_per_texel(),
            v.max_abs_error_m,
            v.normal_max_angle_deg,
        );
    }
}

fn bench_crack() {
    // Two independently encoded pages sharing column 32 of the coast grid.
    let grid = thessa_height_coast();
    let left = HeightGrid::new(
        33,
        65,
        grid.heights
            .as_chunks::<65>()
            .0
            .iter()
            .flat_map(|row| row[..33].to_vec())
            .collect(),
    )
    .expect("left split");
    let right = HeightGrid::new(
        33,
        65,
        grid.heights
            .as_chunks::<65>()
            .0
            .iter()
            .flat_map(|row| row[32..].to_vec())
            .collect(),
    )
    .expect("right split");
    println!("shared-edge crack (coast col 32, metres):");
    for budget in [1.0, 10.0, 100.0] {
        let mode = HeightMode::Adaptive {
            max_abs_error_m: budget,
        };
        let da = HeightPage::encode(&left, mode).decode();
        let db = HeightPage::encode(&right, mode).decode();
        let edge_a: Vec<f32> = da
            .heights
            .as_chunks::<33>()
            .0
            .iter()
            .map(|row| row[32])
            .collect();
        let edge_b: Vec<f32> = db
            .heights
            .as_chunks::<33>()
            .0
            .iter()
            .map(|row| row[0])
            .collect();
        println!(
            "  budget {budget:>6.1} m -> crack {:.3} m",
            shared_edge_error(&edge_a, &edge_b)
        );
    }
}

fn main() {
    println!(
        "microstore Phase E: real 65x65 Thessa height grids, {ITERS} iters (raw f32 = 4.000 B/tex)"
    );
    bench_grid("ocean", -60.0, &thessa_height_ocean());
    bench_grid("coast", -60.0, &thessa_height_coast());
    bench_grid("mountain", -54.0, &thessa_height_mountain());
    bench_crack();
}
