use std::hint::black_box;

use glam::DVec3;
use thessa_sim_core::{
    AeroCase, AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel,
    evaluate_batch,
};

fn main() {
    let panels: Vec<_> = (0..16)
        .map(|index| {
            let x = (index as f64 - 8.0) * 0.4;
            AeroPanel::flat_plate(DVec3::new(x, 0.0, 0.0), 1.5, 1.0).expect("valid panel")
        })
        .collect();
    let geometry = AeroGeometry::new(panels).expect("valid geometry");
    let environment = AeroEnvironment::standard_sea_level();
    let cases: Vec<_> = (0..256)
        .map(|index| {
            let speed = 80.0 + index as f64 * 0.5;
            AeroCase::new(
                AeroState::new(DVec3::new(speed, 0.0, speed * 0.05), DVec3::ZERO),
                environment,
                geometry.clone(),
            )
            .expect("valid case")
        })
        .collect();
    let model = PanelAeroModel::new(AeroConfig::default()).expect("valid aero model");

    let iterations = 1_000;
    let started = std::time::Instant::now();
    for _ in 0..iterations {
        let results = evaluate_batch(black_box(&model), black_box(&cases));
        black_box(results);
    }
    let elapsed = started.elapsed();
    println!(
        "256 vehicles x 16 aero panels: {elapsed:?} total, {:?} average",
        elapsed / iterations
    );
}
