//! Hangar compile throughput: representative wing to panels.
//! Run with `cargo bench -p thessa-aero-surfaces --bench compile`.
use std::{hint::black_box, time::Instant};

use glam::DVec3;
use thessa_aero_surfaces::{
    BendCurve, CompileOptions, FoldJoint, MechanismState, Planform, ProceduralSurface, SectionData,
    SurfaceTopology, aileron, compile_surface,
};

/// Representative airliner-like wing: tapered swept planform, dihedral,
/// washout sections, one aileron, one tip fold.
fn representative_surface() -> ProceduralSurface {
    ProceduralSurface {
        name: "bench-wing".into(),
        span_m: 12.0,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::Single,
        planform: Planform::tapered(3.0, 1.0, 2.5).unwrap(),
        bend: BendCurve::dihedral(12.0, 5.0_f64.to_radians()).unwrap(),
        sections: SectionData::from_stations(vec![
            thessa_aero_surfaces::SectionStation {
                s: 0.0,
                incidence_rad: 2.0_f64.to_radians(),
                thickness_ratio: 0.12,
                profile: None,
            },
            thessa_aero_surfaces::SectionStation {
                s: 1.0,
                incidence_rad: -1.0_f64.to_radians(),
                thickness_ratio: 0.08,
                profile: None,
            },
        ])
        .unwrap(),
        controls: vec![aileron("aileron", (0.55, 0.95)).unwrap().0],
        folds: vec![FoldJoint {
            name: "tip-fold".into(),
            station_s: 0.85,
            axis: DVec3::X,
            deployed_angle_rad: 0.0,
            stowed_angle_rad: 60.0_f64.to_radians(),
            travel_limit_rad: 70.0_f64.to_radians(),
        }],
        structure: None,
    }
}

fn main() {
    let surface = representative_surface();
    let options = CompileOptions::default();
    let mechanism = MechanismState::deployed();
    // Warmup so the timing loop measures steady state, not first call.
    let compiled = compile_surface(&surface, &options, &mechanism).unwrap();
    eprintln!("panels: {}", compiled.panels.len());
    let iterations = 200;
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(compile_surface(black_box(&surface), &options, &mechanism).unwrap());
    }
    let elapsed = started.elapsed();
    eprintln!(
        "compile: {:?} total for {iterations} iterations ({:?}/compile)",
        elapsed,
        elapsed / iterations
    );
}
