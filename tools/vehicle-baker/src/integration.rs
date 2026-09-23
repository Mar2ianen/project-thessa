//! Cross-compiler fixture for one procedural lifting-body aircraft.
//!
//! Wire this module into `main.rs` with `#[cfg(test)] mod integration;`.

use std::collections::HashSet;

use super::*;
use thessa_sim_core::{
    AeroCase, AeroConfig, AeroEnvironment, AeroModel, AeroState, PanelAeroModel,
};

const FIXTURE: &str = include_str!("../../../data/vehicles/example_lifting_body_aircraft.toml");

#[test]
fn procedural_lifting_body_and_wing_bake_and_roundtrip_as_one_vehicle() {
    let asset: VehicleAsset = toml::from_str(FIXTURE).expect("aircraft fixture should parse");
    assert_eq!(asset.procedural_surfaces.len(), 1);
    assert_eq!(asset.procedural_bodies.len(), 1);

    // Compile source-side expectations with the same defaults as VehicleAsset::bake.
    let wing = compile_surface(
        &asset.procedural_surfaces[0],
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .expect("procedural wing should compile");
    let body = compile_body(&asset.procedural_bodies[0], &BodyCompileOptions::default())
        .expect("procedural hull should compile");
    let wing_collision = wing
        .collision_parts(&CollisionOptions::default())
        .expect("wing contact parts should compile");
    let body_collision = body_collision_parts(
        &asset.procedural_bodies[0],
        &BodyCollisionOptions::default(),
    )
    .expect("hull contact parts should compile");

    let hull_structure = body
        .structure
        .as_ref()
        .expect("fixture should compile hull structure");
    assert!(hull_structure.mass_kg > 0.0);
    assert_eq!(body.tanks.len(), 1);
    assert!(body.tanks[0].inner_volume_m3 > 0.0);
    assert!(body.tanks[0].propellant_kg > 0.0);
    assert!(!body_collision.is_empty());

    let mut vehicle = asset.bake().expect("combined aircraft should bake");
    assert_eq!(
        vehicle.aero_geometry.panels.len(),
        wing.panels.len() + body.panels.len(),
        "the wing and hull panels should share one baked AeroGeometry"
    );
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.control_surfaces[0].name, "aileron");
    assert!(!vehicle.control_surfaces[0].panel_indices.is_empty());
    assert_eq!(vehicle.control_surfaces[1].name, "body-rudder");
    assert!(!vehicle.control_surfaces[1].panel_indices.is_empty());

    // VehicleAsset::bake appends hull strips after procedural surface panels.
    let body_panel_start = vehicle.aero_geometry.panels.len() - body.panels.len();
    let wing_controlled_panels: HashSet<usize> = vehicle.control_surfaces[0]
        .panel_indices
        .iter()
        .copied()
        .collect();
    let body_controlled_panels: HashSet<usize> = vehicle.control_surfaces[1]
        .panel_indices
        .iter()
        .copied()
        .collect();
    assert!(
        wing_controlled_panels
            .iter()
            .all(|index| *index < body_panel_start),
        "control references must resolve to wing panels, never hull strips"
    );
    assert!(
        body_controlled_panels
            .iter()
            .all(|index| *index >= body_panel_start),
        "body controls must resolve only to the appended hull strips"
    );
    let controlled_panels: HashSet<usize> = wing_controlled_panels
        .union(&body_controlled_panels)
        .copied()
        .collect();
    assert!(
        vehicle.aero_geometry.panels[body_panel_start..]
            .iter()
            .all(|panel| panel.side_force_scale == 0.0),
        "the appended panels should be the fuselage compiler's body strips"
    );

    // No hand-authored contacts are in the fixture, so this count is exactly
    // the compiled wing boxes plus the procedural hull's per-segment contacts.
    assert_eq!(
        vehicle.collision_geometry.parts.len(),
        wing_collision.len() + body_collision.len()
    );
    assert!(vehicle.collision_geometry.validate().is_ok());
    assert_eq!(vehicle.tanks.len(), 1);
    assert!(vehicle.tanks[0].tank.full_propellant_kg > 0.0);
    assert_eq!(
        vehicle.tanks[0].initial_propellant_kg,
        Some(0.85 * vehicle.tanks[0].tank.full_propellant_kg)
    );
    assert!(vehicle.mass_properties.mass_kg > 1200.0 + hull_structure.mass_kg);

    let initial_panels = vehicle.aero_geometry.panels.clone();
    let initial_mass = vehicle.mass_properties.clone();
    let initial_collision = vehicle.collision_geometry.clone();
    let model = PanelAeroModel::new(AeroConfig::default()).expect("panel model config");
    let state = AeroState::new(DVec3::new(100.0, 0.0, -5.0), DVec3::ZERO);
    let environment = AeroEnvironment::standard_sea_level();
    let baseline = model
        .evaluate_detailed(
            &AeroCase::new(state, environment, vehicle.aero_geometry.clone())
                .expect("initial aero case should be valid"),
        )
        .expect("initial panel loads should evaluate")
        .panel_loads
        .expect("detailed evaluation should return local panel loads");

    vehicle
        .apply_control_inputs(&[1.0, 1.0])
        .expect("maximum wing and body commands should be accepted");
    assert_eq!(vehicle.mass_properties, initial_mass);
    assert_eq!(vehicle.collision_geometry, initial_collision);
    for (index, (panel, initial)) in vehicle
        .aero_geometry
        .panels
        .iter()
        .zip(&initial_panels)
        .enumerate()
    {
        let mut expected = *initial;
        if wing_controlled_panels.contains(&index) {
            expected.control_deflection_rad = vehicle.control_surfaces[0].maximum_deflection_rad;
        }
        if body_controlled_panels.contains(&index) {
            expected.control_deflection_rad = vehicle.control_surfaces[1].maximum_deflection_rad;
        }
        assert_eq!(
            *panel, expected,
            "unexpected geometry mutation at panel {index}"
        );
    }

    let deflected = model
        .evaluate_detailed(
            &AeroCase::new(state, environment, vehicle.aero_geometry.clone())
                .expect("deflected aero case should be valid"),
        )
        .expect("deflected panel loads should evaluate")
        .panel_loads
        .expect("detailed evaluation should return local panel loads");
    assert_eq!(baseline.len(), deflected.len());
    let changed_panels: HashSet<usize> = baseline
        .iter()
        .zip(&deflected)
        .enumerate()
        .filter_map(|(index, (before, after))| {
            ((after.force_body_n - before.force_body_n).length() > 1.0e-6).then_some(index)
        })
        .collect();
    assert_eq!(
        changed_panels, controlled_panels,
        "control commands should change aerodynamics only on assigned wing and body panels"
    );
    for index in 0..baseline.len() {
        if !controlled_panels.contains(&index) {
            assert_eq!(
                deflected[index].moment_body_nm, baseline[index].moment_body_nm,
                "control command changed moment on uncontrolled panel {index}"
            );
        }
    }

    // The baked output is the serialized runtime contract; the authoring
    // procedural definitions are intentionally absent from this artifact.
    let json = serde_json::to_string(&vehicle).expect("baked vehicle should serialize");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("baked vehicle should deserialize");
    assert_eq!(round_trip.name, vehicle.name);
    assert_eq!(
        round_trip.aero_geometry.panels.len(),
        vehicle.aero_geometry.panels.len()
    );
    assert_eq!(round_trip.control_surfaces, vehicle.control_surfaces);
    assert_eq!(
        round_trip.collision_geometry.parts.len(),
        vehicle.collision_geometry.parts.len()
    );
    assert_eq!(round_trip.tanks.len(), vehicle.tanks.len());
    assert_eq!(
        round_trip.aero_geometry.panels[vehicle.control_surfaces[0].panel_indices[0]]
            .control_deflection_rad,
        vehicle.aero_geometry.panels[vehicle.control_surfaces[0].panel_indices[0]]
            .control_deflection_rad
    );
    let mass_relative_error =
        (round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs()
            / vehicle.mass_properties.mass_kg;
    assert!(mass_relative_error < 1.0e-12);
    assert!(round_trip.collision_geometry.validate().is_ok());
    assert!(round_trip.validate().is_ok());
}
