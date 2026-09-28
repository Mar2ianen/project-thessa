use crate::{
    AeroConfig, AeroGeometry, AeroPanel, AeroSimdScratch, AtmosphereConfig, ControlChannels,
    ControlMixing, ControlSurfaceDefinition, FlightStepInput, PanelAeroModel, PanelSoA,
    RigidBodyProperties, RigidBodyState, control_surface_commands, evaluate_flight_forces_soa,
    evaluate_flight_forces_with_aero_result, integrate_rigid_body_step_soa,
    integrate_rigid_body_step_with_aero_result,
};
use glam::{DMat3, DQuat, DVec3};

#[test]
fn baked_mixer_generates_order_independent_commands_for_eight_surfaces() {
    let mixers = [
        ControlMixing {
            pitch: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            roll: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            yaw: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            flap: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            airbrake: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            pitch: 1.0,
            roll: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            yaw: 1.0,
            flap: 1.0,
            ..ControlMixing::default()
        },
        ControlMixing {
            pitch: 1.0,
            roll: 1.0,
            yaw: 1.0,
            flap: 1.0,
            airbrake: 1.0,
        },
    ];
    let surfaces: Vec<_> = mixers
        .into_iter()
        .enumerate()
        .map(|(index, mixing)| {
            ControlSurfaceDefinition::new(format!("surface-{index}"), vec![index], -0.5, 0.5)
                .unwrap()
                .with_mixing(mixing)
        })
        .collect();
    let channels = ControlChannels {
        pitch: 0.2,
        roll: 0.3,
        yaw: 0.4,
        flap: 0.5,
        airbrake: 0.6,
    };
    let expected = [0.2, 0.3, 0.4, 0.5, 0.6, 0.5, 0.9, 1.0];
    let commands = control_surface_commands(&surfaces, channels);
    assert_eq!(commands.len(), 8);
    for (actual, expected) in commands.iter().zip(expected) {
        assert!((actual - expected).abs() < 1.0e-12);
    }

    let permutation = [6, 1, 4, 0, 7, 3, 2, 5];
    let reordered: Vec<_> = permutation
        .into_iter()
        .map(|index| surfaces[index].clone())
        .collect();
    let reordered_commands = control_surface_commands(&reordered, channels);
    assert_eq!(reordered_commands.len(), permutation.len());
    for (actual, original_index) in reordered_commands.iter().zip(permutation) {
        assert!((actual - expected[original_index]).abs() < 1.0e-12);
    }
}

#[test]
fn unmixed_legacy_surfaces_keep_the_four_channel_mapping_without_oob() {
    let surfaces: Vec<_> = (0..8)
        .map(|index| {
            ControlSurfaceDefinition::new(format!("legacy-{index}"), vec![index], -0.5, 0.5)
                .unwrap()
        })
        .collect();
    let commands = control_surface_commands(
        &surfaces,
        ControlChannels {
            pitch: 0.2,
            yaw: -0.3,
            roll: 0.4,
            ..ControlChannels::default()
        },
    );
    assert_eq!(commands, [-0.2, -0.3, -0.4, 0.4, 0.0, 0.0, 0.0, 0.0]);
}

#[test]
fn cached_aero_result_reproduces_force_sample_and_rigid_body_step() {
    let model = PanelAeroModel::new(AeroConfig::default()).unwrap();
    let geometry = AeroGeometry::new(vec![
        AeroPanel::new(DVec3::new(-0.5, 0.0, 0.0), DVec3::X, DVec3::Z, 8.0, 2.0).unwrap(),
    ])
    .unwrap();
    let panels = PanelSoA::from_geometry(&geometry).unwrap();
    let atmosphere = AtmosphereConfig::new(288.15, 101_325.0, 287.05287, 1.4, 9.81).unwrap();
    let state = RigidBodyState::new(
        DVec3::new(1_000.0, -20.0, 500.0),
        DVec3::new(150.0, 3.0, 8.0),
        DQuat::IDENTITY,
        DVec3::new(0.01, -0.02, 0.005),
    )
    .unwrap();
    let properties = RigidBodyProperties::new(1_200.0, DMat3::IDENTITY * 900.0).unwrap();
    let input = FlightStepInput {
        extra_force_body_n: DVec3::new(500.0, -20.0, 100.0),
        extra_moment_body_nm: DVec3::new(10.0, 5.0, -2.0),
        ..FlightStepInput::new(500.0, DVec3::new(0.0, 0.0, -9.81))
    };
    let environment = atmosphere.aero_environment(500.0, DVec3::ZERO).unwrap();
    let aero_state =
        crate::AeroState::new(state.velocity_inertial_mps, state.angular_velocity_body_rps);
    let aero_result = model
        .evaluate_soa_simd(aero_state, environment, &panels, false)
        .unwrap();

    let mut normal_scratch = AeroSimdScratch::default();
    let normal_forces = evaluate_flight_forces_soa(
        &model,
        &panels,
        &mut normal_scratch,
        atmosphere,
        state,
        properties,
        input,
    )
    .unwrap();
    let cached_forces = evaluate_flight_forces_with_aero_result(
        atmosphere,
        state,
        properties,
        input,
        aero_result.clone(),
    )
    .unwrap();
    assert_eq!(cached_forces, normal_forces);

    let mut normal_scratch = AeroSimdScratch::default();
    let normal_step = integrate_rigid_body_step_soa(
        &model,
        &panels,
        &mut normal_scratch,
        atmosphere,
        state,
        properties,
        input,
        1.0 / 120.0,
    )
    .unwrap();
    let cached_step = integrate_rigid_body_step_with_aero_result(
        atmosphere,
        state,
        properties,
        input,
        1.0 / 120.0,
        aero_result,
    )
    .unwrap();
    assert_eq!(cached_step, normal_step);
}
