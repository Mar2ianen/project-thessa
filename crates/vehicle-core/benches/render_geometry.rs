//! Definition-copy diagnostic. No GPU/physics/IO, no production-speedup claim.
use glam::{DQuat, DVec3};
use std::{hint::black_box, sync::Arc, time::Instant};
use thessa_vehicle_core::{RenderGeometry, RenderMesh, RenderPart};

fn measure(mut work: impl FnMut()) -> f64 {
    let mut times: Vec<_> = (0..200)
        .map(|_| {
            let now = Instant::now();
            work();
            now.elapsed().as_secs_f64() * 1e6
        })
        .collect();
    times.sort_by(f64::total_cmp);
    times[times.len() / 2]
}

fn main() {
    let mesh = Arc::new(RenderMesh {
        positions: vec![DVec3::X; 65536],
        normals: vec![DVec3::Y; 65536],
        indices: [0, 1, 2].repeat(65536),
    });
    let geometry = RenderGeometry {
        parts: (0..64)
            .map(|body| RenderPart {
                name: format!("body-{body}"),
                assembly_body: Some(body),
                position_body_m: DVec3::ZERO,
                orientation_mesh_to_body: DQuat::IDENTITY,
                mesh: mesh.clone(),
            })
            .collect(),
    };
    geometry.validate(Some(64)).unwrap();
    let shared = measure(|| {
        black_box(geometry.clone());
    });
    let copy = measure(|| {
        black_box((*mesh).clone());
    });
    println!(
        "render_geometry parts=64 vertices_per_mesh=65536 indices_per_mesh=196608 shared_clone_median_us={shared:.3} single_deep_mesh_copy_median_us={copy:.3}; n=200; shared payload bytes={} (64 references, not 64 allocations); diagnostic not previous production",
        mesh.positions.len() * 48 + mesh.indices.len() * 4
    );
}
