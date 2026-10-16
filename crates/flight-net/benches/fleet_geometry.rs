//! Bounded catalogue scan/encode diagnostic, not a network/render benchmark.
use glam::{DQuat, DVec3};
use std::{hint::black_box, time::Instant};
use thessa_flight_net::{MAX_FLEET_VEHICLES, geometry::*};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, VehicleId,
    X15StarterProfile,
};

fn measure(mut work: impl FnMut()) -> f64 {
    let mut ns = Vec::new();
    for _ in 0..200 {
        let start = Instant::now();
        work();
        ns.push(start.elapsed().as_nanos());
    }
    ns.sort_unstable();
    ns[100] as f64 / 1000.0
}

fn main() {
    let mut vehicle = X15StarterProfile::new().unwrap().vehicle;
    vehicle.collision_geometry = CollisionGeometry::new(
        (0..MAX_GEOMETRY_PARTS_PER_VEHICLE)
            .map(|i| {
                CollisionPart::new(
                    DVec3::new(i as f64, 0.0, 0.0),
                    DQuat::IDENTITY,
                    CollisionShape::Capsule {
                        axis: thessa_sim_core::CollisionAxis::X,
                        half_segment_m: 1.0,
                        radius_m: 0.5,
                    },
                    CollisionMaterial::default(),
                )
                .unwrap()
            })
            .collect(),
    )
    .unwrap();
    vehicle
        .render_geometry
        .parts
        .push(thessa_sim_core::RenderPart {
            name: "retained-body".into(),
            assembly_body: None,
            position_body_m: DVec3::ZERO,
            orientation_mesh_to_body: DQuat::IDENTITY,
            mesh: std::sync::Arc::new(thessa_sim_core::RenderMesh {
                positions: vec![DVec3::new(0.1, 0.2, 0.3); MAX_RENDER_VERTICES],
                normals: vec![DVec3::Z; MAX_RENDER_VERTICES],
                indices: vec![1023; 4095],
            }),
        });
    let build = || FleetGeometry {
        revision: 1,
        vehicles: (1..MAX_FLEET_VEHICLES as u32)
            .map(|id| FleetVehicleGeometry::from_vehicle(VehicleId::new(id), black_box(&vehicle)))
            .collect(),
    };
    let catalogue = build();
    catalogue.validate().unwrap();
    let bytes = encode_fleet_geometry(&catalogue).unwrap();
    let scan = measure(|| {
        assert!(
            black_box(&catalogue)
                .vehicles
                .iter()
                .all(|d| d.matches_vehicle(d.vehicle_id, black_box(&vehicle)))
        )
    });
    let rebuild = measure(|| {
        black_box(encode_fleet_geometry(&build()).unwrap());
    });
    println!(
        "fleet_geometry members={} contact_parts_per_member={} render_vertices_per_member=1024 render_indices_per_member=4095 encoded_frame_bytes={} cached_borrowed_scan_median_us={scan:.3} rebuild_encode_median_us={rebuild:.3}; n=200 each; excludes IO/UI/GPU/decoder; rebuild is a diagnostic alternative, not the previous production server",
        catalogue.vehicles.len(),
        MAX_GEOMETRY_PARTS_PER_VEHICLE,
        bytes.len()
    );
}
