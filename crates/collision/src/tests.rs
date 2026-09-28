use super::*;
use thessa_sim_core::{
    CollisionPart, CollisionShape, LandingLegSpec, LandingShockAbsorberSpec, TireConstruction,
    WheelBrakeSpec, WheelChassisSpec, WheelLayout, WheelStrutSpec, WheelTireSpec,
};

fn sphere_geometry(radius_m: f64) -> CollisionGeometry {
    CollisionGeometry::new(vec![
        CollisionPart::new(
            DVec3::ZERO,
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m },
            CollisionMaterial::default(),
        )
        .unwrap(),
    ])
    .unwrap()
}

fn wheel_chassis() -> CompiledWheelChassis {
    WheelChassisSpec {
        name: "query-test-wheel".into(),
        mount_position_body_m: DVec3::new(0.0, 0.0, 0.7),
        mount_orientation_body: DQuat::IDENTITY,
        length_m: 1.0,
        layout: WheelLayout::Inline,
        wheel_count: 1,
        structural_mass_kg: 10.0,
        structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(0.2)),
        tire: WheelTireSpec {
            construction: TireConstruction::Pneumatic {
                inflation_pressure_pa: 220_000.0,
                reference_temperature_k: 293.15,
            },
            radius_m: 0.32,
            width_m: 0.2,
            mass_kg: 2.0,
            spin_inertia_kg_m2: 0.08,
            radial_stiffness_n_m: 200_000.0,
            radial_damping_n_s_m: 5_000.0,
            longitudinal_slip_stiffness_n_per_mps: 12_000.0,
            lateral_slip_stiffness_n_per_mps: 9_000.0,
            maximum_deflection_m: 0.08,
            maximum_load_n: 8_000.0,
            surface_friction: 0.9,
        },
        strut: WheelStrutSpec {
            extended_length_m: 0.4,
            stroke_m: 0.15,
            spring_rate_n_m: 30_000.0,
            damping_n_s_m: 2_000.0,
            preload_n: 0.0,
            minimum_force_n: 0.0,
            maximum_force_n: 10_000.0,
            mass_per_wheel_kg: 0.5,
        },
        brake: WheelBrakeSpec {
            maximum_torque_nm: 200.0,
            response_time_s: 0.1,
            mass_per_wheel_kg: 0.3,
        },
        drive: None,
        retraction: None,
    }
    .compile()
    .unwrap()
}

fn landing_leg() -> CompiledLandingLeg {
    LandingLegSpec {
        name: "query-test-leg".into(),
        mount_position_body_m: DVec3::ZERO,
        hinge_axis_body: DVec3::Y,
        stowed_leg_axis_body: DVec3::Z,
        stowed_angle_rad: 0.0,
        deployed_angle_rad: std::f64::consts::PI,
        initially_deployed: true,
        deployment_rate_rad_s: 1.0,
        actuator_max_torque_nm: 10_000.0,
        leg_length_m: 2.0,
        leg_mass_kg: 10.0,
        footpad_radius_m: 0.2,
        footpad_mass_kg: 1.0,
        footpad_friction: 0.8,
        footpad_slip_stiffness_n_per_mps: 5_000.0,
        shock_absorber: LandingShockAbsorberSpec::Reusable {
            stroke_m: 0.2,
            spring_rate_n_m: 20_000.0,
            damping_n_s_m: 1_000.0,
            preload_n: 0.0,
            bottom_out_stiffness_n_m: 200_000.0,
            maximum_force_n: 50_000.0,
        },
    }
    .compile()
    .unwrap()
}

#[test]
fn landing_footpad_query_reports_reaction_friction_and_absorbed_energy() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.25),
            DQuat::IDENTITY,
            DVec3::new(5.0, 5.0, 0.25),
            CollisionMaterial::new(0.4, 0.0).unwrap(),
        )
        .unwrap();
    let properties =
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(10.0))).unwrap();
    let body = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 0.0, 2.15)),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    world.step(1.0 / 120.0, []).unwrap();
    world
        .resync_dynamic_body(
            body,
            RigidBodyState::new(
                DVec3::new(0.0, 0.0, 2.15),
                DVec3::new(1.0, 0.0, -0.2),
                DQuat::IDENTITY,
                DVec3::ZERO,
            )
            .unwrap(),
            properties,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let compiled = landing_leg();
    let state = compiled.spec.initial_state();
    let result = world
        .evaluate_landing_leg_contacts(
            body,
            std::slice::from_ref(&compiled),
            std::slice::from_ref(&state),
            DVec3::ZERO,
            LandingLegDeploymentCommand::All(true),
            0.1,
        )
        .unwrap();
    assert_eq!(result.contacts.len(), 1);
    let contact = result.contacts[0];
    assert!((contact.compression_m - 0.05).abs() < 1.0e-12);
    assert!((contact.compression_rate_mps - 0.2).abs() < 1.0e-12);
    assert!((contact.axial_force_n - 1_200.0).abs() < 1.0e-9);
    assert!((contact.normal_load_n - 1_200.0).abs() < 1.0e-9);
    assert!((contact.contact_friction - 0.6).abs() < 1.0e-12);
    assert!((contact.tangential_force_inertial_n.length() - 720.0).abs() < 1.0e-9);
    assert!(contact.saturated);
    assert!((contact.absorbed_energy_delta_j - 4.0).abs() < 1.0e-12);
    assert!((result.wrench.force_inertial_n.z - 1_200.0).abs() < 1.0e-9);
    assert!(result.wrench.force_inertial_n.x < 0.0);
    assert!((result.contacts[0].contact_point_inertial_m.z).abs() < 1.0e-12);
    let state = result.states[0];
    assert_eq!(state.permanent_crush_m, 0.0);

    let folded = LandingLegState {
        deployment_fraction: 0.0,
        ..state
    };
    let no_contact = world
        .evaluate_landing_leg_contacts(
            body,
            std::slice::from_ref(&compiled),
            std::slice::from_ref(&folded),
            DVec3::ZERO,
            LandingLegDeploymentCommand::PerLeg(&[false]),
            0.1,
        )
        .unwrap();
    assert!(no_contact.contacts.is_empty());
    assert_eq!(no_contact.wrench, ExternalWrench::ZERO);
    assert_eq!(no_contact.states, vec![folded]);
}

#[test]
fn landing_footpad_query_applies_persistent_crush_once_to_leg_geometry() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.25),
            DQuat::IDENTITY,
            DVec3::new(5.0, 5.0, 0.25),
            CollisionMaterial::default(),
        )
        .unwrap();
    let properties =
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(10.0))).unwrap();
    // L + pad radius - ground distance = 0.25 m compression.
    let body = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 0.0, 1.95)),
            properties,
            &sphere_geometry(0.1),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    world.step(1.0 / 120.0, []).unwrap();
    world
        .resync_dynamic_body(
            body,
            RigidBodyState::stationary(DVec3::new(0.0, 0.0, 1.95)),
            properties,
            DynamicBodyConfig::default(),
        )
        .unwrap();

    let mut spec = landing_leg().spec;
    spec.shock_absorber = LandingShockAbsorberSpec::Crushable {
        elastic_stiffness_n_m: 100_000.0,
        damping_n_s_m: 0.0,
        plateau_force_n: 10_000.0,
        maximum_crush_m: 0.25,
        bottom_out_stiffness_n_m: 300_000.0,
        maximum_force_n: 100_000.0,
    };
    let leg = spec.compile().unwrap();
    let first = world
        .evaluate_landing_leg_contacts(
            body,
            std::slice::from_ref(&leg),
            &[leg.spec.initial_state()],
            DVec3::ZERO,
            LandingLegDeploymentCommand::All(true),
            1.0 / 120.0,
        )
        .unwrap();
    assert_eq!(first.contacts.len(), 1);
    assert!((first.contacts[0].compression_m - 0.25).abs() < 1.0e-12);
    assert!((first.contacts[0].axial_force_n - 10_000.0).abs() < 1.0e-9);
    assert!((first.states[0].permanent_crush_m - 0.15).abs() < 1.0e-12);

    // Keep the body at the same pose: reduced leg length changes the
    // query reach, but the shock sees the original total compression and
    // neither loses support nor consumes the same crush a second time.
    let second = world
        .evaluate_landing_leg_contacts(
            body,
            std::slice::from_ref(&leg),
            &first.states,
            DVec3::ZERO,
            LandingLegDeploymentCommand::All(true),
            1.0 / 120.0,
        )
        .unwrap();
    assert_eq!(second.contacts.len(), 1);
    assert!((second.contacts[0].compression_m - 0.25).abs() < 1.0e-12);
    assert!((second.contacts[0].axial_force_n - 10_000.0).abs() < 1.0e-9);
    assert!((second.states[0].permanent_crush_m - 0.15).abs() < 1.0e-12);
}

#[test]
fn collision_frame_round_trips_authoritative_state() {
    let frame = CollisionFrame::new(
        DVec3::new(1.0e9, -2.0e9, 3.0e9),
        DVec3::new(1200.0, -30.0, 8.0),
        DQuat::from_rotation_z(0.7),
    )
    .unwrap();
    let state = RigidBodyState::new(
        frame.origin_inertial_m + DVec3::new(10.0, 20.0, -5.0),
        DVec3::new(1210.0, -25.0, 11.0),
        DQuat::from_rotation_y(0.3),
        DVec3::new(0.1, -0.2, 0.3),
    )
    .unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
    let mut world = CollisionWorld::new(frame).unwrap();
    let id = world
        .insert_dynamic_body(
            state,
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let round_trip = world.body_state(id).unwrap();
    assert!((round_trip.position_inertial_m - state.position_inertial_m).length() < 1.0e-6);
    assert!((round_trip.velocity_inertial_mps - state.velocity_inertial_mps).length() < 1.0e-9);
    assert!(
        round_trip
            .orientation_body_to_inertial
            .dot(state.orientation_body_to_inertial)
            .abs()
            > 1.0 - 1.0e-12
    );
    assert!(
        (round_trip.angular_velocity_body_rps - state.angular_velocity_body_rps).length() < 1.0e-9
    );
}

#[test]
fn dynamic_body_inputs_reject_invalid_public_state_and_inertia() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
    let geometry = sphere_geometry(0.5);
    let state = RigidBodyState::stationary(DVec3::ZERO);

    let invalid_orientation = RigidBodyState {
        orientation_body_to_inertial: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
        ..state
    };
    assert!(matches!(
        world.insert_dynamic_body(
            invalid_orientation,
            properties,
            &geometry,
            DynamicBodyConfig::default()
        ),
        Err(CollisionBackendError::InvalidBodyState(_))
    ));

    let indefinite_inertia = RigidBodyProperties {
        mass_kg: 5.0,
        inertia_body_kg_m2: DMat3::from_diagonal(DVec3::new(1.0, -1.0, -1.0)),
    };
    assert!(matches!(
        world.insert_dynamic_body(
            state,
            indefinite_inertia,
            &geometry,
            DynamicBodyConfig::default()
        ),
        Err(CollisionBackendError::InvalidMassProperties)
    ));
    assert_eq!(world.dynamic_body_count(), 0);

    let id = world
        .insert_dynamic_body(state, properties, &geometry, DynamicBodyConfig::default())
        .unwrap();
    assert!(matches!(
        world.resync_dynamic_body(
            id,
            invalid_orientation,
            properties,
            DynamicBodyConfig::default()
        ),
        Err(CollisionBackendError::InvalidBodyState(_))
    ));
    assert_eq!(world.body_state(id).unwrap(), state);
}

#[test]
fn body_states_unrepresentable_in_the_collision_frame_are_rejected() {
    let origin = DVec3::splat(-f64::MAX);
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(origin, DVec3::ZERO)).unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
    let geometry = sphere_geometry(0.5);
    let initial = RigidBodyState::stationary(origin);
    let id = world
        .insert_dynamic_body(initial, properties, &geometry, DynamicBodyConfig::default())
        .unwrap();
    let unrepresentable = RigidBodyState::stationary(DVec3::splat(f64::MAX));

    assert!(matches!(
        world.resync_dynamic_body(
            id,
            unrepresentable,
            properties,
            DynamicBodyConfig::default()
        ),
        Err(CollisionBackendError::InvalidBodyState(_))
    ));
    assert_eq!(world.dynamic_body_count(), 1);
    assert_eq!(world.body_state(id).unwrap(), initial);
}

#[test]
fn rapier_resolves_gravity_driven_ground_contact() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, -0.5, 0.0),
            DQuat::IDENTITY,
            DVec3::new(10.0, 0.5, 10.0),
            CollisionMaterial::default(),
        )
        .unwrap();
    let mass_kg = 5.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..360 {
        world.step(1.0 / 120.0, [(id, gravity)]).unwrap();
    }
    let solved = world.body_state(id).unwrap();
    assert!((solved.position_inertial_m.y - 0.5).abs() < 0.05);
    assert!(solved.velocity_inertial_mps.length() < 0.2);
}

#[test]
fn wheel_query_resolves_tire_strut_load_and_returns_single_contact_wrench() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(10.0, 10.0, 0.5),
            CollisionMaterial::new(0.5, 0.0).unwrap(),
        )
        .unwrap();
    let properties =
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
    let state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::new(0.1, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let id = world
        .insert_dynamic_body(
            state,
            properties,
            &CollisionGeometry::new(vec![
                CollisionPart::new(
                    DVec3::Z,
                    DQuat::IDENTITY,
                    CollisionShape::Sphere { radius_m: 0.1 },
                    CollisionMaterial::default(),
                )
                .unwrap(),
            ])
            .unwrap(),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    world
        .step(1.0 / 120.0, [(id, ExternalWrench::ZERO)])
        .unwrap();

    let chassis = wheel_chassis();
    let result = world.evaluate_wheel_contacts(id, &chassis, &[0.0]).unwrap();
    assert_eq!(result.contacts.len(), 1);
    let contact = result.contacts[0];
    assert!((contact.radial_penetration_m - 0.02).abs() < 1.0e-8);
    assert!(contact.normal_load_n > 0.0);
    assert!((contact.contact_friction - 0.7).abs() < 1.0e-12);
    assert!(contact.longitudinal_force_n < 0.0);
    assert!(
        contact.longitudinal_force_n.hypot(contact.lateral_force_n)
            <= contact.contact_friction * contact.normal_load_n + 1.0e-9
    );
    assert!(result.wrench.force_inertial_n.z > 0.0);
    assert!(result.wrench.force_inertial_n.x < 0.0);
    assert!(result.wrench.torque_inertial_nm.is_finite());

    let rolling = world
        .evaluate_wheel_contacts(id, &chassis, &[0.1 / chassis.spec.tire.radius_m])
        .unwrap();
    assert!(
        rolling.contacts[0]
            .relative_contact_velocity_inertial_mps
            .x
            .abs()
            < 1.0e-10
    );
    assert!(rolling.contacts[0].longitudinal_force_n.abs() < 1.0e-8);

    assert!(world.evaluate_wheel_contacts(id, &chassis, &[]).is_err());
}

#[test]
fn wheel_query_uses_kinematic_terrain_velocity_for_tire_slip() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    let terrain = world
        .insert_kinematic_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(10.0, 10.0, 0.5),
            CollisionMaterial::default(),
        )
        .unwrap();
    let properties =
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
    let geometry = CollisionGeometry::new(vec![
        CollisionPart::new(
            DVec3::Z,
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )
        .unwrap(),
    ])
    .unwrap();
    let body = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::ZERO),
            properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    world
        .set_next_kinematic_pose(terrain, DVec3::new(0.1, 0.0, -0.5), DQuat::IDENTITY)
        .unwrap();
    world.step(0.1, [(body, ExternalWrench::ZERO)]).unwrap();

    let chassis = wheel_chassis();
    let contact = world
        .evaluate_wheel_contacts(body, &chassis, &[0.0])
        .unwrap()
        .contacts[0];
    assert!((contact.relative_contact_velocity_inertial_mps.x + 1.0).abs() < 1.0e-10);
    assert!(contact.longitudinal_force_n > 0.0);
}

#[test]
fn articulated_wheel_keeps_tire_deflection_separate_from_strut_travel() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(10.0, 10.0, 0.5),
            CollisionMaterial::default(),
        )
        .unwrap();

    let mut chassis = wheel_chassis();
    chassis.spec.mount_position_body_m = DVec3::ZERO;
    chassis = chassis.spec.clone().compile().unwrap();
    let station = chassis.wheel_stations[0];
    let sprung_state = RigidBodyState::stationary(DVec3::new(0.0, 0.0, 0.65));
    let sprung_geometry = CollisionGeometry::new(vec![
        CollisionPart::new(
            DVec3::new(0.0, 0.0, 5.0),
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )
        .unwrap(),
    ])
    .unwrap();
    let sprung = world
        .insert_dynamic_body(
            sprung_state,
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap(),
            &sprung_geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let wheel_mass = chassis.wheel_body_mass_properties(0)[0];
    let wheel_geometry = CollisionGeometry::new(vec![
        CollisionPart::new(
            DVec3::ZERO,
            DQuat::IDENTITY,
            CollisionShape::Sphere {
                radius_m: chassis.spec.tire.radius_m,
            },
            CollisionMaterial::default(),
        )
        .unwrap(),
    ])
    .unwrap();
    let wheel = world
        .insert_dynamic_sensor_body(
            RigidBodyState::stationary(
                sprung_state.position_inertial_m + station.position_body_m + DVec3::Z * 0.05,
            ),
            RigidBodyProperties::new(wheel_mass.mass_kg, wheel_mass.inertia_body_kg_m2).unwrap(),
            &wheel_geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    world
        .attach_suspension_wheel_joint(
            sprung,
            wheel,
            station.position_body_m,
            DVec3::ZERO,
            DVec3::NEG_Z,
            station.axle_axis_body,
            DVec3::NEG_Z,
            station.axle_axis_body,
            [-chassis.spec.strut.stroke_m, 0.0],
        )
        .unwrap();
    world.step(1.0 / 120.0, []).unwrap();

    let binding = ArticulatedWheelBinding {
        wheel_body: wheel,
        wheel_index: station.index,
        nominal_center_sprung_local_m: station.position_body_m,
        slide_axis_sprung_local: DVec3::NEG_Z,
        axle_axis_sprung_local: station.axle_axis_body,
    };
    let result = world
        .evaluate_articulated_wheel_contacts(sprung, &chassis, &[binding])
        .unwrap();
    let contact = result[0].contact.expect("wheel/terrain contact");
    assert!((contact.strut_compression_m - 0.05).abs() < 1.0e-9);
    assert!((contact.radial_penetration_m - 0.02).abs() < 1.0e-9);
    assert!((contact.tire_compression_m - 0.02).abs() < 1.0e-9);
    assert!(result[0].wheel_wrench.force_inertial_n.z > 0.0);
    assert!(result[0].sprung_wrench.force_inertial_n.z > 0.0);
    assert!(
        (result[0].wheel_wrench.force_inertial_n.z + result[0].sprung_wrench.force_inertial_n.z
            - contact.normal_load_n)
            .abs()
            < 1.0e-9,
        "strut reaction must cancel internally, leaving only tire contact load"
    );
}

#[test]
fn one_wheel_contact_settles_under_gravity_without_solid_wheel_impulses() {
    let mut world =
        CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO)).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(10.0, 10.0, 0.5),
            CollisionMaterial::default(),
        )
        .unwrap();
    let mass_kg = 100.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(50.0))).unwrap();
    let geometry = CollisionGeometry::new(vec![
        CollisionPart::new(
            DVec3::Z,
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )
        .unwrap(),
    ])
    .unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::ZERO),
            properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let chassis = wheel_chassis();
    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, 0.0, -9.81 * mass_kg),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..1_200 {
        world
            .step_with_wheel_contacts(1.0 / 240.0, [(id, gravity)], [(id, &chassis, &[0.0][..])])
            .unwrap();
    }
    let solved = world.body_state(id).unwrap();
    let contact = world
        .evaluate_wheel_contacts(id, &chassis, &[0.0])
        .unwrap()
        .contacts[0];
    assert!((contact.normal_load_n - mass_kg * 9.81).abs() < 0.005 * mass_kg * 9.81);
    assert!(solved.velocity_inertial_mps.length() < 0.1);
    assert!(solved.position_inertial_m.z < 0.0);
    assert!(!contact.saturated);
}

#[test]
fn translating_frame_advances_inertial_origin() {
    let frame = CollisionFrame::inertial_at(
        DVec3::new(1.0e9, -2.0e9, 3.0e9),
        DVec3::new(125.0, -7.0, 3.0),
    );
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
    let initial = RigidBodyState::new(
        frame.origin_inertial_m + DVec3::new(2.0, 3.0, 4.0),
        frame.origin_velocity_inertial_mps,
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let mut world = CollisionWorld::new(frame).unwrap();
    let id = world
        .insert_dynamic_body(
            initial,
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();

    let step_s = 2.0;
    world.step(step_s, [(id, ExternalWrench::ZERO)]).unwrap();
    let solved = world.body_state(id).unwrap();
    let expected_position =
        initial.position_inertial_m + frame.origin_velocity_inertial_mps * step_s;

    assert!((solved.position_inertial_m - expected_position).length() < 1.0e-6);
    assert!((solved.velocity_inertial_mps - initial.velocity_inertial_mps).length() < 1.0e-12);
    assert!(
        (world.frame().origin_inertial_m
            - (frame.origin_inertial_m + frame.origin_velocity_inertial_mps * step_s))
            .length()
            < 1.0e-9
    );
}

#[test]
fn free_rapier_motion_matches_symplectic_euler_envelope() {
    // Vacuum force/torque fixture before contacts are enabled: a constant
    // inertial force with zero moment and zero spin must track the
    // authoritative symplectic-Euler translation (v then x) inside a
    // bounded envelope. This pins the wrench conversion and readback, not
    // solver identity: Rapier integrates the same load with its own
    // scheme, so the bound is physical, not bitwise.
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let mass_kg = 10.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(4.0))).unwrap();
    let initial = RigidBodyState::new(
        DVec3::new(0.0, 100.0, 0.0),
        DVec3::new(12.0, 3.0, -1.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let id = world
        .insert_dynamic_body(
            initial,
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let acceleration = DVec3::new(1.5, -9.81, 0.25);
    let wrench = ExternalWrench {
        force_inertial_n: acceleration * mass_kg,
        torque_inertial_nm: DVec3::ZERO,
    };
    let dt = 1.0 / 120.0;
    let steps = 120;
    for _ in 0..steps {
        world.step(dt, [(id, wrench)]).unwrap();
    }
    let solved = world.body_state(id).unwrap();
    // Reference: closed-form constant-acceleration motion plus one
    // symplectic-Euler step offset (v-then-x ordering advances position
    // with the end-of-step velocity).
    let time_s = dt * steps as f64;
    let expected_velocity = initial.velocity_inertial_mps + acceleration * time_s;
    let expected_position = initial.position_inertial_m
        + initial.velocity_inertial_mps * time_s
        + 0.5 * acceleration * time_s * time_s;
    let velocity_error = (solved.velocity_inertial_mps - expected_velocity).length();
    let position_error = (solved.position_inertial_m - expected_position).length();
    assert!(
        velocity_error < 0.05,
        "free-flight velocity drift {velocity_error}"
    );
    assert!(
        position_error < 0.10,
        "free-flight position drift {position_error}"
    );
    assert!(
        (solved.angular_velocity_body_rps - DVec3::ZERO).length() < 1.0e-6,
        "torque-free spin must not appear"
    );
}

#[test]
fn fast_body_does_not_tunnel_through_floor() {
    // Fast-impact regression: a sphere at 60 m/s toward a thin floor must
    // be caught by CCD instead of tunnelling. One 120 Hz step moves
    // 0.5 m; the floor top is at y=0 and the sphere starts 3 m above it.
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, -0.25, 0.0),
            DQuat::IDENTITY,
            DVec3::new(10.0, 0.25, 10.0),
            CollisionMaterial::new(0.7, 0.0).unwrap(),
        )
        .unwrap();
    let mass_kg = 5.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::new(
                DVec3::new(0.0, 3.0, 0.0),
                DVec3::new(0.0, -60.0, 0.0),
                DQuat::IDENTITY,
                DVec3::ZERO,
            )
            .unwrap(),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    for _ in 0..240 {
        world
            .step(1.0 / 120.0, [(id, ExternalWrench::ZERO)])
            .unwrap();
    }
    let solved = world.body_state(id).unwrap();
    assert!(
        solved.position_inertial_m.y > -0.5,
        "fast body tunnelled through the floor: y={}",
        solved.position_inertial_m.y
    );
    assert!(
        (solved.position_inertial_m.y - 0.5).abs() < 0.6,
        "fast body did not settle near the floor: y={}",
        solved.position_inertial_m.y
    );
}

#[test]
fn kinematic_terrain_carries_a_landed_body() {
    // Kinematic seam: an elevator platform rising at 1 m/s must carry a
    // resting sphere with it. The platform pose is prescribed per tick,
    // exactly how the ephemeris/body-rotation model will drive planetary
    // terrain in production.
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let platform = world
        .insert_kinematic_cuboid(
            DVec3::new(0.0, -0.5, 0.0),
            DQuat::IDENTITY,
            DVec3::new(5.0, 0.5, 5.0),
            CollisionMaterial::new(0.9, 0.0).unwrap(),
        )
        .unwrap();
    let mass_kg = 5.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 0.6, 0.0)),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig {
                full_ccd: true,
                can_sleep: false,
            },
        )
        .unwrap();
    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    let dt = 1.0 / 120.0;
    // Settle onto the platform first.
    for _ in 0..120 {
        world.step(dt, [(id, gravity)]).unwrap();
    }
    // Rise 1 m over the next second.
    for step in 1..=120 {
        let lift = step as f64 * dt * 1.0;
        world
            .set_next_kinematic_pose(platform, DVec3::new(0.0, -0.5 + lift, 0.0), DQuat::IDENTITY)
            .unwrap();
        world.step(dt, [(id, gravity)]).unwrap();
    }
    let solved = world.body_state(id).unwrap();
    assert!(
        (solved.position_inertial_m.y - 1.5).abs() < 0.15,
        "landed body did not ride the kinematic platform: y={}",
        solved.position_inertial_m.y
    );
}

#[test]
fn debug_snapshot_reports_settled_contact() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    world
        .insert_static_cuboid(
            DVec3::new(0.0, -0.5, 0.0),
            DQuat::IDENTITY,
            DVec3::new(10.0, 0.5, 10.0),
            CollisionMaterial::default(),
        )
        .unwrap();
    let mass_kg = 5.0;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..360 {
        world.step(1.0 / 120.0, [(id, gravity)]).unwrap();
    }
    let snapshot = world.debug_snapshot().unwrap();
    assert_eq!(snapshot.dynamic_bodies.len(), 1);
    assert_eq!(snapshot.fixed_collider_count, 1);
    assert!(
        snapshot.touching_contact_pairs >= 1,
        "settled body must report a touching pair: {snapshot:?}"
    );
    assert!(
        snapshot.touching_contact_pairs <= snapshot.active_contact_pairs,
        "touching pairs must be a subset of active pairs: {snapshot:?}"
    );
    let json = serde_json::to_string(&snapshot).expect("snapshot must serialize");
    assert!(json.contains("touching_contact_pairs"));
}

#[test]
fn orbital_scale_velocity_survives_a_step() {
    // Rapier's default 400 m/s velocity clamp would silently rewrite a
    // co-moving orbital velocity. The backend disables it: a fast body
    // with no contacts and no wrench must keep its velocity through a
    // step, with position advancing by the full displacement.
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(2.0))).unwrap();
    let initial = RigidBodyState::new(
        DVec3::new(1.0e9, 2.0e9, 3.0e9),
        DVec3::new(1200.0, 51_000.0, -300.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let id = world
        .insert_dynamic_body(
            initial,
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let dt = 1.0 / 120.0;
    world.step(dt, [(id, ExternalWrench::ZERO)]).unwrap();
    let solved = world.body_state(id).unwrap();
    assert!(
        (solved.velocity_inertial_mps - initial.velocity_inertial_mps).length() < 1.0e-6,
        "orbital velocity was clamped: {:?}",
        solved.velocity_inertial_mps
    );
    let expected_position = initial.position_inertial_m + initial.velocity_inertial_mps * dt;
    assert!(
        (solved.position_inertial_m - expected_position).length() < 1.0e-3,
        "orbital displacement is wrong: {:?}",
        solved.position_inertial_m
    );
}

#[test]
fn rejected_trimesh_leaves_no_untracked_body() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    // Out-of-range triangle index fails the mesh build.
    let vertices = vec![DVec3::ZERO, DVec3::X, DVec3::Y];
    let result =
        world.insert_kinematic_trimesh(vertices, vec![[0, 1, 7]], CollisionMaterial::default());
    assert!(result.is_err(), "bad trimesh indices must be rejected");
    assert_eq!(world.kinematic_body_count(), 0);
    assert!(world.debug_snapshot().unwrap().kinematic_bodies.is_empty());
}

#[test]
fn removing_bodies_and_patches_clears_the_scene() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let patch = world
        .insert_static_cuboid(
            DVec3::new(0.0, -0.5, 0.0),
            DQuat::IDENTITY,
            DVec3::new(10.0, 0.5, 10.0),
            CollisionMaterial::default(),
        )
        .unwrap();
    let platform = world
        .insert_kinematic_cuboid(
            DVec3::new(8.0, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::new(1.0, 0.5, 1.0),
            CollisionMaterial::default(),
        )
        .unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let id = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 2.0, 0.0)),
            properties,
            &sphere_geometry(0.5),
            DynamicBodyConfig::default(),
        )
        .unwrap();
    assert_eq!(world.dynamic_body_count(), 1);
    assert_eq!(world.fixed_collider_count(), 1);
    assert_eq!(world.kinematic_body_count(), 1);
    world.remove_dynamic_body(id).unwrap();
    world.remove_static_collider(patch).unwrap();
    world.remove_kinematic_body(platform).unwrap();
    assert_eq!(world.dynamic_body_count(), 0);
    assert_eq!(world.fixed_collider_count(), 0);
    assert_eq!(world.kinematic_body_count(), 0);
    assert!(world.debug_snapshot().unwrap().dynamic_bodies.is_empty());
}
