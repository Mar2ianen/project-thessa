use super::*;
use thessa_sim_core::{
    CollisionPart, CollisionShape, DockingKinematics, DockingPortSpec, DockingPortState,
    DockingSession,
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

fn two_spheres() -> (CollisionWorld, CollisionBodyId, CollisionBodyId) {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let properties =
        RigidBodyProperties::new(5.0, DMat3::from_diagonal(DVec3::splat(0.5))).unwrap();
    let geometry = sphere_geometry(0.5);
    let a = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(-2.0, 0.0, 0.0)),
            properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let b = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(2.0, 0.0, 0.0)),
            properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    (world, a, b)
}

#[test]
fn prismatic_joint_keeps_its_anchor_and_enforces_strut_stroke_limits() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let root_properties =
        RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(20.0))).unwrap();
    let wheel_properties =
        RigidBodyProperties::new(2.0, DMat3::from_diagonal(DVec3::splat(0.2))).unwrap();
    let geometry = sphere_geometry(0.1);
    let root = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::ZERO),
            root_properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let wheel = world
        .insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 0.0, -0.5)),
            wheel_properties,
            &geometry,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let joint = world
        .attach_prismatic_joint(
            root,
            wheel,
            DVec3::new(0.0, 0.0, -0.5),
            DVec3::ZERO,
            DVec3::NEG_Z,
            DVec3::NEG_Z,
            [0.0, 0.2],
        )
        .unwrap();
    let push_down = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, 0.0, -100.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..600 {
        world
            .step(
                1.0 / 240.0,
                [(root, ExternalWrench::ZERO), (wheel, push_down)],
            )
            .unwrap();
    }
    let root_state = world.body_state(root).unwrap();
    let wheel_state = world.body_state(wheel).unwrap();
    let relative = wheel_state.position_inertial_m - root_state.position_inertial_m;
    let extension_m = (relative - DVec3::new(0.0, 0.0, -0.5)).dot(DVec3::NEG_Z);
    assert!((extension_m - 0.2).abs() < 0.02);
    assert!(world.joint_count() == 1);
    world.remove_joint(joint).unwrap();
    assert_eq!(world.joint_count(), 0);
}

fn d1_craft_geometry(port_x_m: f64) -> CollisionGeometry {
    let material = CollisionMaterial {
        friction: 0.45,
        restitution: 0.0,
    };
    let mut parts = vec![
        CollisionPart::new(
            DVec3::ZERO,
            DQuat::IDENTITY,
            CollisionShape::Cuboid {
                half_extents_m: DVec3::new(0.35, 0.45, 0.45),
            },
            material,
        )
        .unwrap(),
    ];
    for index in 0..8 {
        let angle = f64::from(index) * std::f64::consts::TAU / 8.0;
        parts.push(
            CollisionPart::new(
                DVec3::new(
                    port_x_m - port_x_m.signum() * 0.01,
                    0.55 * angle.cos(),
                    0.55 * angle.sin(),
                ),
                DQuat::from_rotation_x(angle),
                CollisionShape::Cuboid {
                    half_extents_m: DVec3::new(0.01, 0.12, 0.12),
                },
                material,
            )
            .unwrap(),
        );
    }
    CollisionGeometry::new(parts).unwrap()
}

fn port_frame_world(state: RigidBodyState, local_position_m: DVec3) -> DVec3 {
    state.position_inertial_m + state.orientation_body_to_inertial * local_position_m
}

#[test]
fn docked_bodies_move_as_one_and_undock_cleanly() {
    let (mut world, a, b) = two_spheres();
    // Dock nose to nose: A's +X port meets B's -X port at the origin.
    let joint = world
        .attach_fixed_joint(
            a,
            b,
            DVec3::new(2.0, 0.0, 0.0),
            DQuat::IDENTITY,
            DVec3::new(-2.0, 0.0, 0.0),
            DQuat::IDENTITY,
        )
        .unwrap();
    assert_eq!(world.joint_count(), 1);
    // Shove A; the joint must drag B along.
    let push = ExternalWrench {
        force_inertial_n: DVec3::new(500.0, 0.0, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..120 {
        world
            .step(1.0 / 120.0, [(a, push), (b, ExternalWrench::ZERO)])
            .unwrap();
    }
    let state_a = world.body_state(a).unwrap();
    let state_b = world.body_state(b).unwrap();
    let separation = (state_b.position_inertial_m - state_a.position_inertial_m).length();
    assert!(
        (separation - 4.0).abs() < 0.05,
        "docked separation must hold, got {separation}"
    );
    assert!(
        state_b.velocity_inertial_mps.x > 1.0,
        "docked partner must be dragged along"
    );
    // Undock with no wrenches anywhere: both clusters coast at their
    // solved velocities (A would otherwise ram B from behind and the
    // contact, not the joint, would do work).
    world.remove_joint(joint).unwrap();
    assert_eq!(world.joint_count(), 0);
    let sep_before = (world.body_state(b).unwrap().position_inertial_m
        - world.body_state(a).unwrap().position_inertial_m)
        .length();
    let v_before = world.body_state(b).unwrap().velocity_inertial_mps;
    for _ in 0..60 {
        world
            .step(
                1.0 / 120.0,
                [(a, ExternalWrench::ZERO), (b, ExternalWrench::ZERO)],
            )
            .unwrap();
    }
    let after_b = world.body_state(b).unwrap();
    let sep_after =
        (after_b.position_inertial_m - world.body_state(a).unwrap().position_inertial_m).length();
    assert!(
        (after_b.velocity_inertial_mps - v_before).length() < 1.0e-6,
        "undocked partner must coast force-free"
    );
    assert!(
        (sep_after - sep_before).abs() < 1.0e-6,
        "undock must not kick the clusters: separation {sep_before} -> {sep_after}"
    );
}

#[test]
fn two_d1_craft_progress_through_rapier_docking_and_undock() {
    let frame = CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO);
    let mut world = CollisionWorld::new(frame).unwrap();
    let properties =
        RigidBodyProperties::new(1_200.0, DMat3::from_diagonal(DVec3::splat(900.0))).unwrap();
    let geometry_a = d1_craft_geometry(0.8);
    let geometry_b = d1_craft_geometry(-0.8);
    let state_a = RigidBodyState::new(
        DVec3::new(-0.805, 0.0, 0.0),
        DVec3::new(0.01, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let state_b = RigidBodyState::new(
        DVec3::new(0.805, 0.0, 0.0),
        DVec3::new(-0.01, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .unwrap();
    let a = world
        .insert_dynamic_body(
            state_a,
            properties,
            &geometry_a,
            DynamicBodyConfig::default(),
        )
        .unwrap();
    let b = world
        .insert_dynamic_body(
            state_b,
            properties,
            &geometry_b,
            DynamicBodyConfig::default(),
        )
        .unwrap();

    let port_a =
        DockingPortSpec::d1("craft-a-d1", DVec3::new(0.8, 0.0, 0.0), DQuat::IDENTITY).unwrap();
    let port_b =
        DockingPortSpec::d1("craft-b-d1", DVec3::new(-0.8, 0.0, 0.0), DQuat::IDENTITY).unwrap();
    let mut docking = DockingSession::new(port_a.clone(), port_b.clone(), 0.25).unwrap();
    let initial_relative_position = port_frame_world(state_b, port_b.local_position_m)
        - port_frame_world(state_a, port_a.local_position_m);
    docking.begin_soft_capture(0.02).unwrap();
    world
        .step(
            1.0 / 120.0,
            [(a, ExternalWrench::ZERO), (b, ExternalWrench::ZERO)],
        )
        .unwrap();
    let solved_a = world.body_state(a).unwrap();
    let solved_b = world.body_state(b).unwrap();
    docking
        .align(DockingKinematics::between(solved_a, &port_a, solved_b, &port_b).unwrap())
        .unwrap();
    assert_eq!(docking.state, DockingPortState::Aligned);

    docking.hard_dock().unwrap();
    let joint = world
        .attach_fixed_joint(
            a,
            b,
            port_a.local_position_m,
            port_a.local_orientation,
            port_b.local_position_m,
            port_b.local_orientation,
        )
        .unwrap();
    docking.engage_outer_structure().unwrap();
    docking.advance_pressure_equalization(0.25).unwrap();
    assert_eq!(docking.state, DockingPortState::PressureEqualized);
    assert_eq!(world.joint_count(), 1);

    let push = ExternalWrench {
        force_inertial_n: DVec3::new(1_500.0, 0.0, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    for _ in 0..120 {
        world
            .step(1.0 / 120.0, [(a, push), (b, ExternalWrench::ZERO)])
            .unwrap();
    }
    let moved_a = world.body_state(a).unwrap();
    let moved_b = world.body_state(b).unwrap();
    let center_separation = (moved_b.position_inertial_m - moved_a.position_inertial_m).length();
    assert!((center_separation - 1.61).abs() < 0.05);
    assert!(moved_b.velocity_inertial_mps.x > 0.5);
    let solved_relative_position = port_frame_world(moved_b, port_b.local_position_m)
        - port_frame_world(moved_a, port_a.local_position_m);
    assert!(solved_relative_position.length() < 0.02);

    docking.undock().unwrap();
    world.remove_joint(joint).unwrap();
    assert_eq!(docking.state, DockingPortState::Free);
    assert!(initial_relative_position.length() < 0.02);
    assert_eq!(world.joint_count(), 0);
}

#[test]
fn revolute_joint_keeps_d1_mechanism_anchor_and_allows_hinge_rotation() {
    let (mut world, a, b) = two_spheres();
    let joint = world
        .attach_revolute_joint(a, b, DVec3::X, DVec3::NEG_X, DVec3::Z, DVec3::Z)
        .unwrap();
    let torque = ExternalWrench {
        force_inertial_n: DVec3::ZERO,
        torque_inertial_nm: DVec3::new(0.0, 0.0, 10.0),
    };
    for _ in 0..120 {
        world
            .step(1.0 / 120.0, [(a, torque), (b, ExternalWrench::ZERO)])
            .unwrap();
    }
    let state_a = world.body_state(a).unwrap();
    let state_b = world.body_state(b).unwrap();
    let anchor_error = ((state_a.position_inertial_m
        + state_a.orientation_body_to_inertial * DVec3::X)
        - (state_b.position_inertial_m + state_b.orientation_body_to_inertial * DVec3::NEG_X))
        .length();
    assert!(
        anchor_error < 0.02,
        "hinge anchor drifted by {anchor_error} m"
    );
    assert!(state_a.angular_velocity_body_rps.z > 0.1);
    assert_eq!(world.joint_count(), 1);
    world.remove_joint(joint).unwrap();
}

#[test]
fn removing_a_body_drops_its_joints() {
    let (mut world, a, b) = two_spheres();
    world
        .attach_fixed_joint(
            a,
            b,
            DVec3::X,
            DQuat::IDENTITY,
            DVec3::NEG_X,
            DQuat::IDENTITY,
        )
        .unwrap();
    world.remove_dynamic_body(a).unwrap();
    assert_eq!(world.joint_count(), 0);
}

#[test]
fn settled_contact_reports_load_evidence_and_drains_once() {
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
    assert_eq!(snapshot.contacts.len(), 1);
    let contact = snapshot.contacts[0];
    assert!(contact.penetration_m >= 0.0);
    assert!(contact.penetration_m < 0.05);
    assert!(contact.approach_speed_mps.abs() < 0.2);
    assert_eq!(snapshot.patches.len(), 1);
    // Drain semantics: first drain takes the step's events, the second
    // is empty until another step records.
    let drained = world.drain_contact_events();
    assert_eq!(drained.len(), 1);
    assert!(world.drain_contact_events().is_empty());
}

#[test]
fn identical_input_sequences_replay_identically() {
    // Same-binary replay gate for the determinism story: two worlds from
    // the same setup and wrench tape must produce identical snapshots.
    fn run_tape() -> String {
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
                RigidBodyState::new(
                    DVec3::new(0.5, 3.0, -0.25),
                    DVec3::new(2.0, -1.0, 0.5),
                    DQuat::from_rotation_y(0.4),
                    DVec3::new(0.5, -0.3, 0.2),
                )
                .unwrap(),
                properties,
                &sphere_geometry(0.5),
                DynamicBodyConfig::default(),
            )
            .unwrap();
        for step in 0..180 {
            let thrust = if step < 60 {
                DVec3::new(30.0, 5.0, -10.0)
            } else {
                DVec3::ZERO
            };
            world
                .step(
                    1.0 / 120.0,
                    [(
                        id,
                        ExternalWrench {
                            force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0) + thrust,
                            torque_inertial_nm: DVec3::new(0.0, 0.0, 1.5),
                        },
                    )],
                )
                .unwrap();
        }
        let snapshot = world.debug_snapshot().unwrap();
        serde_json::to_string(&snapshot).unwrap()
    }
    assert_eq!(run_tape(), run_tape());
}
