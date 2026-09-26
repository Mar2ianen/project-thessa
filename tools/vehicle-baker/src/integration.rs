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
    // Explicit ratings are test design inputs, not defaults baked into the
    // runtime: existing body controls without actuator data remain migratable.
    let body_actuator = thessa_sim_core::ControlSurfaceActuator {
        max_rate_rad_s: 0.4,
        max_torque_nm: 1.0e12,
    };
    let elevator_actuator = thessa_sim_core::ControlSurfaceActuator {
        max_rate_rad_s: 0.3,
        max_torque_nm: 7_500.0,
    };
    // Exercise Windows checkout line endings on every host before injecting
    // test-only TOML tables, then normalize them for platform-independent edits.
    let crlf_fixture = FIXTURE.replace("\r\n", "\n").replace('\n', "\r\n");
    let normalized_fixture = crlf_fixture.replace("\r\n", "\n");
    let actuator_fixture = normalized_fixture
        .replace(
            "maximum_deflection_rad = 0.35\n",
            "maximum_deflection_rad = 0.35\n\n[procedural_bodies.controls.actuator]\nmax_rate_rad_s = 0.4\nmax_torque_nm = 1000000000000.0",
        )
        .replace(
            "maximum_deflection_rad = 0.3\n",
            "maximum_deflection_rad = 0.3\n\n[procedural_bodies.controls.actuator]\nmax_rate_rad_s = 0.3\nmax_torque_nm = 7500.0",
        );
    let asset: VehicleAsset =
        toml::from_str(&actuator_fixture).expect("actuated aircraft fixture should parse");
    assert_eq!(asset.procedural_surfaces.len(), 1);
    assert_eq!(asset.procedural_bodies.len(), 1);
    let body_stations = &asset.procedural_bodies[0].stations;
    let mid_body = body_stations
        .iter()
        .find(|station| station.x_m == 0.0)
        .expect("body fixture should have a center station");
    let aft_station = body_stations.first().expect("body stations");
    let nose_station = body_stations.last().expect("body stations");
    assert!(mid_body.bottom_exponent > mid_body.top_exponent);
    assert!(nose_station.offset_z_m < aft_station.offset_z_m);
    let smooth_belly = thessa_fuselage::outline_point(
        mid_body.half_width_m,
        mid_body.top_height_m,
        mid_body.bottom_height_m,
        mid_body.top_exponent,
        mid_body.top_exponent,
        -std::f64::consts::FRAC_PI_4,
    );
    let chined_belly = thessa_fuselage::outline_point(
        mid_body.half_width_m,
        mid_body.top_height_m,
        mid_body.bottom_height_m,
        mid_body.top_exponent,
        mid_body.bottom_exponent,
        -std::f64::consts::FRAC_PI_4,
    );
    assert!(chined_belly.1 < smooth_belly.1 - 0.05);
    let lifting_body_source = asset.procedural_bodies[0].clone();
    assert_eq!(
        asset.procedural_bodies[0].controls[0].actuator,
        Some(body_actuator)
    );

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
    assert_eq!(vehicle.control_surfaces.len(), 3);
    assert_eq!(vehicle.control_surfaces[0].name, "aileron");
    assert!(!vehicle.control_surfaces[0].panel_indices.is_empty());
    assert_eq!(vehicle.control_surfaces[1].name, "body-rudder");
    assert!(!vehicle.control_surfaces[1].panel_indices.is_empty());
    assert_eq!(
        vehicle.control_surfaces[1].hinge.unwrap().axis_body,
        DVec3::Z
    );
    assert_eq!(vehicle.control_surfaces[1].actuator, Some(body_actuator));
    assert_eq!(vehicle.control_surfaces[2].name, "body-elevator");
    assert!(!vehicle.control_surfaces[2].panel_indices.is_empty());
    assert_eq!(
        vehicle.control_surfaces[2].hinge.unwrap().axis_body,
        -DVec3::Y
    );
    assert_eq!(
        vehicle.control_surfaces[2].actuator,
        Some(elevator_actuator)
    );

    // VehicleAsset::bake appends hull strips after procedural surface panels.
    let body_panel_start = vehicle.aero_geometry.panels.len() - body.panels.len();
    let body_hinge_shift = vehicle.aero_geometry.panels
        [body_panel_start + body.controls[0].panel_indices[0]]
        .position_body_m
        - body.panels[body.controls[0].panel_indices[0]].position_body_m;
    assert!(
        (vehicle.control_surfaces[1].hinge.unwrap().point_body_m
            - (body.controls[0].hinge.unwrap().point_body_m + body_hinge_shift))
            .length()
            < 1.0e-12
    );
    let wing_controlled_panels: HashSet<usize> = vehicle.control_surfaces[0]
        .panel_indices
        .iter()
        .copied()
        .collect();
    let body_rudder_panels: HashSet<usize> = vehicle.control_surfaces[1]
        .panel_indices
        .iter()
        .copied()
        .collect();
    let body_elevator_panels: HashSet<usize> = vehicle.control_surfaces[2]
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
        body_rudder_panels
            .union(&body_elevator_panels)
            .all(|index| *index >= body_panel_start),
        "body controls must resolve only to the appended hull strips"
    );
    let controlled_panels: HashSet<usize> = wing_controlled_panels
        .union(&body_rudder_panels)
        .copied()
        .chain(body_elevator_panels.iter().copied())
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
    let initial_mass = vehicle.mass_properties;
    let initial_collision = vehicle.collision_geometry.clone();
    let model = PanelAeroModel::new(AeroConfig::default()).expect("panel model config");
    let state = AeroState::new(DVec3::new(100.0, 0.0, -5.0), DVec3::ZERO);
    let environment = AeroEnvironment::standard_sea_level();
    let baseline_result = model
        .evaluate_detailed(
            &AeroCase::new(state, environment, vehicle.aero_geometry.clone())
                .expect("initial aero case should be valid"),
        )
        .expect("initial panel loads should evaluate");
    let baseline_pitch_moment_nm = baseline_result.moment_body_nm.y;
    let baseline = baseline_result
        .panel_loads
        .expect("detailed evaluation should return local panel loads");

    // The authored nose droop is aerodynamic geometry: with the body isolated,
    // it shifts the zero-alpha lift down and positive alpha restores lift.
    let body_cl_at = |panels: &[thessa_sim_core::AeroPanel], alpha_deg: f64| {
        let alpha = alpha_deg.to_radians();
        let velocity = DVec3::new(100.0 * alpha.cos(), 0.0, -100.0 * alpha.sin());
        let geometry = AeroGeometry::new(panels.to_vec()).expect("body-only aero geometry");
        let result = model
            .evaluate(
                &AeroCase::new(AeroState::new(velocity, DVec3::ZERO), environment, geometry)
                    .expect("body-only aero case"),
            )
            .expect("body-only aero loads");
        result.force_body_n.z
            / (0.5 * environment.density_kg_m3 * 100.0_f64.powi(2) * body.summary.frontal_area_m2)
    };
    let drooped_cl0 = body_cl_at(&body.panels, 0.0);
    let drooped_cl5 = body_cl_at(&body.panels, 5.0);
    let mut straight_body_source = lifting_body_source;
    for station in &mut straight_body_source.stations {
        station.offset_z_m = 0.0;
    }
    let straight_body = compile_body(&straight_body_source, &BodyCompileOptions::default())
        .expect("straight-centerline lifting body should compile");
    let straight_cl0 = body_cl_at(&straight_body.panels, 0.0);
    assert!(
        drooped_cl0 < straight_cl0 - 0.01,
        "forward droop should shift zero-alpha lift down: drooped={drooped_cl0}, straight={straight_cl0}"
    );
    assert!(
        drooped_cl5 > drooped_cl0,
        "positive angle of attack should recover lift: CL0={drooped_cl0}, CL5={drooped_cl5}"
    );

    // The aft pitch control is a real hinged body strip: deflection must alter
    // local panel loads and the resulting pitching moment, not inject a craft
    // moment directly.
    let mut flap_vehicle = vehicle.clone();
    flap_vehicle
        .apply_control_deflections(&vehicle.aero_geometry, &[0.0, 0.0, 0.2])
        .expect("aft body flap should rotate from reference geometry");
    let flap_result = model
        .evaluate_detailed(
            &AeroCase::new(state, environment, flap_vehicle.aero_geometry.clone())
                .expect("deflected body flap aero case"),
        )
        .expect("deflected body flap loads should evaluate");
    assert!(
        (flap_result.moment_body_nm.y - baseline_pitch_moment_nm).abs() > 1.0e-3,
        "aft flap should change the aerodynamic pitch moment: baseline={baseline_pitch_moment_nm}, deflected={}",
        flap_result.moment_body_nm.y
    );
    let flap_panel_loads = flap_result
        .panel_loads
        .as_ref()
        .expect("deflected detailed evaluation should expose panel loads");
    assert!(
        body_elevator_panels.iter().any(|index| {
            (flap_panel_loads[*index].force_body_n - baseline[*index].force_body_n).length()
                > 1.0e-6
        }),
        "pitch-flap deflection should alter its panel forces"
    );
    for index in 0..baseline.len() {
        if !body_elevator_panels.contains(&index) {
            assert_eq!(
                flap_panel_loads[index].force_body_n, baseline[index].force_body_n,
                "pitch-flap deflection changed an unrelated panel {index}"
            );
        }
    }

    let mut moving_vehicle = vehicle.clone();
    moving_vehicle
        .apply_control_deflections(&vehicle.aero_geometry, &[0.0, 0.2, 0.0])
        .expect("hinged body region should rotate from reference geometry");
    for (index, (moved, original)) in moving_vehicle
        .aero_geometry
        .panels
        .iter()
        .zip(&initial_panels)
        .enumerate()
    {
        if body_rudder_panels.contains(&index) {
            assert_ne!(
                moved.center_of_pressure_body_m,
                original.center_of_pressure_body_m
            );
            assert_eq!(moved.control_deflection_rad, 0.0);
        } else {
            assert_eq!(moved, original, "hinge motion leaked to panel {index}");
        }
    }
    let moving_loads = model
        .evaluate_detailed(
            &AeroCase::new(state, environment, moving_vehicle.aero_geometry.clone())
                .expect("moving geometry aero case should be valid"),
        )
        .expect("moving body geometry loads should evaluate");
    let moving_panel_loads = moving_loads
        .panel_loads
        .as_ref()
        .expect("moving detailed result should expose panel loads");
    for index in 0..baseline.len() {
        let force_delta =
            (moving_panel_loads[index].force_body_n - baseline[index].force_body_n).length();
        if body_rudder_panels.contains(&index) {
            assert!(
                force_delta > 1.0e-6,
                "hinge angle did not alter panel {index}"
            );
        } else {
            assert!(force_delta < 1.0e-10, "hinge motion altered panel {index}");
        }
    }
    let hinge_moments = moving_vehicle
        .control_hinge_moments(&moving_loads)
        .expect("body hinge should receive panel aerodynamic loads");
    assert!(hinge_moments[1].is_finite());
    let (next_angles, saturated) = moving_vehicle
        .advance_control_actuators(&[0.0, 0.0, 0.0], &[0.0, 1.0, 0.0], &hinge_moments, 0.5)
        .expect("rated body actuator should advance");
    assert!(
        saturated,
        "the rate limit should leave the target unreached"
    );
    let expected_angle = 0.4 * (1.0 - hinge_moments[1].abs() / body_actuator.max_torque_nm) * 0.5;
    assert!((next_angles[1] - expected_angle).abs() < 1.0e-12);

    vehicle
        .apply_control_inputs(&[1.0, 1.0, 1.0])
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
        if body_rudder_panels.contains(&index) {
            expected.control_deflection_rad = vehicle.control_surfaces[1].maximum_deflection_rad;
        }
        if body_elevator_panels.contains(&index) {
            expected.control_deflection_rad = vehicle.control_surfaces[2].maximum_deflection_rad;
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
    assert_eq!(
        round_trip.control_surfaces.len(),
        vehicle.control_surfaces.len()
    );
    for (round_trip_control, control) in round_trip
        .control_surfaces
        .iter()
        .zip(&vehicle.control_surfaces)
    {
        assert_eq!(round_trip_control.name, control.name);
        assert_eq!(round_trip_control.panel_indices, control.panel_indices);
        assert_eq!(round_trip_control.kind, control.kind);
        assert_eq!(round_trip_control.parent_index, control.parent_index);
        assert_eq!(round_trip_control.actuator, control.actuator);
        assert!(
            (round_trip_control.minimum_deflection_rad - control.minimum_deflection_rad).abs()
                < 1.0e-12
        );
        assert!(
            (round_trip_control.maximum_deflection_rad - control.maximum_deflection_rad).abs()
                < 1.0e-12
        );
        match (round_trip_control.hinge, control.hinge) {
            (Some(round_trip_hinge), Some(hinge)) => {
                assert!((round_trip_hinge.point_body_m - hinge.point_body_m).length() < 1.0e-12);
                assert!((round_trip_hinge.axis_body - hinge.axis_body).length() < 1.0e-12);
            }
            (None, None) => {}
            _ => panic!("hinge metadata changed during vehicle roundtrip"),
        }
    }
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
