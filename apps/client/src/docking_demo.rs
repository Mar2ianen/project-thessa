//! Live two-craft docking fixture for the real Bevy client.
//!
//! This is deliberately opt-in (`--docking-demo`). It runs the same
//! Rapier-backed collision world and authoritative docking state machine used
//! by the backend tests, but publishes the solved poses into visible client
//! entities so approach, capture, hard dock, load transfer, and undock can be
//! inspected in the actual game window.

use bevy::prelude::*;
use glam::DVec3;
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, DockingKinematics,
    DockingPortSpec, DockingPortState, DockingSession, RigidBodyProperties, RigidBodyState,
};

const STEP_S: f64 = 1.0 / 120.0;
const PORT_A_X_M: f64 = 1.4;
const PORT_B_X_M: f64 = -1.4;
const FORCE_AFTER_EQUALIZATION_N: f64 = 350.0;

#[derive(Resource)]
pub struct DockingDemoState {
    world: CollisionWorld,
    body_a: thessa_collision::CollisionBodyId,
    body_b: thessa_collision::CollisionBodyId,
    port_a: DockingPortSpec,
    port_b: DockingPortSpec,
    session: DockingSession,
    joint: Option<thessa_collision::JointId>,
    elapsed_s: f64,
    last_report_s: f64,
    load_elapsed_s: f64,
    finished: bool,
}

#[derive(Component)]
pub(super) struct DockingDemoVisual {
    body: u8,
    local_position_m: Vec3,
}

#[derive(Component)]
pub(super) struct DockingDemoHud;

/// Returns whether the caller requested the live docking fixture.
pub fn requested() -> bool {
    std::env::args().any(|argument| argument == "--docking-demo")
}

pub(super) fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    mut cameras: Query<&mut crate::OrbitCamera>,
) {
    let port_a = DockingPortSpec::d1(
        "demo-craft-a-port",
        glam::DVec3::X * PORT_A_X_M,
        glam::DQuat::IDENTITY,
    )
    .expect("valid demo port A");
    let port_b = DockingPortSpec::d1(
        "demo-craft-b-port",
        glam::DVec3::NEG_X * PORT_A_X_M,
        glam::DQuat::IDENTITY,
    )
    .expect("valid demo port B");
    let mut session =
        DockingSession::new(port_a.clone(), port_b.clone(), 0.5).expect("compatible demo ports");
    session
        .begin_soft_capture(0.04)
        .expect("demo approach is inside capture speed limit");

    let geometry = demo_geometry();
    let properties =
        RigidBodyProperties::new(300.0, glam::DMat3::from_diagonal(glam::DVec3::splat(400.0)))
            .expect("valid demo mass properties");
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(
        glam::DVec3::ZERO,
        glam::DVec3::ZERO,
    ))
    .expect("demo collision world");
    let demo_config = DynamicBodyConfig {
        full_ccd: true,
        can_sleep: false,
    };
    let body_a = world
        .insert_dynamic_body(
            RigidBodyState::new(
                glam::DVec3::new(-1.5, 0.0, 0.0),
                glam::DVec3::new(0.02, 0.0, 0.0),
                glam::DQuat::IDENTITY,
                glam::DVec3::ZERO,
            )
            .expect("valid demo craft A state"),
            properties,
            &geometry,
            demo_config,
        )
        .expect("insert demo craft A");
    let body_b = world
        .insert_dynamic_body(
            RigidBodyState::new(
                glam::DVec3::new(1.5, 0.0, 0.0),
                glam::DVec3::new(-0.02, 0.0, 0.0),
                glam::DQuat::IDENTITY,
                glam::DVec3::ZERO,
            )
            .expect("valid demo craft B state"),
            properties,
            &geometry,
            demo_config,
        )
        .expect("insert demo craft B");

    let body_mesh = meshes.add(Cuboid::new(1.1, 0.9, 0.9));
    let body_a_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.10, 0.22, 0.42),
        metallic: 0.75,
        perceptual_roughness: 0.28,
        ..default()
    });
    let body_b_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.42, 0.16, 0.08),
        metallic: 0.75,
        perceptual_roughness: 0.28,
        ..default()
    });
    let port_mesh = meshes.add(Cuboid::new(0.18, 1.25, 1.25));
    let port_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.58, 0.12),
        emissive: LinearRgba::rgb(0.12, 0.04, 0.005),
        metallic: 0.55,
        perceptual_roughness: 0.22,
        ..default()
    });

    spawn_visual(
        &mut commands,
        body_mesh.clone(),
        body_a_material,
        0,
        Vec3::ZERO,
        "Docking demo craft A",
    );
    spawn_visual(
        &mut commands,
        body_mesh,
        body_b_material,
        1,
        Vec3::ZERO,
        "Docking demo craft B",
    );
    spawn_visual(
        &mut commands,
        port_mesh.clone(),
        port_material.clone(),
        0,
        Vec3::new(PORT_A_X_M as f32, 0.0, 0.0),
        "D1 port A",
    );
    spawn_visual(
        &mut commands,
        port_mesh,
        port_material,
        1,
        Vec3::new(PORT_B_X_M as f32, 0.0, 0.0),
        "D1 port B",
    );

    // Also load the generated CAD-derived D1 assembly into the real client.
    // The simple proxy bodies remain visible as the two craft, while these
    // scenes show the authored docking hardware at each interface.
    let d1_scene =
        asset_server.load("models/thessa-d1-docking-port/thessa_d1_docking_port.glb#Scene0");
    commands.spawn((
        WorldAssetRoot(d1_scene.clone()),
        DockingDemoVisual {
            body: 0,
            local_position_m: Vec3::new(PORT_A_X_M as f32, 0.0, 0.0),
        },
        Name::new("CAD D1 port A"),
        Transform::default(),
    ));
    commands.spawn((
        WorldAssetRoot(d1_scene),
        DockingDemoVisual {
            body: 1,
            local_position_m: Vec3::new(PORT_B_X_M as f32, 0.0, 0.0),
        },
        Name::new("CAD D1 port B"),
        Transform::default(),
    ));

    commands.spawn((
        DockingDemoHud,
        Text::new("DOCKING DEMO\ninitializing"),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
        TextColor(Color::srgb(0.95, 0.78, 0.35)),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(18.0),
            left: Val::Px(18.0),
            ..default()
        },
        ZIndex(200),
    ));

    for mut camera in &mut cameras {
        camera.distance = 9.0;
        camera.target = Vec3::ZERO;
        camera.orbit = Quat::from_rotation_y(0.22) * Quat::from_rotation_x(-0.28);
    }
    commands.insert_resource(DockingDemoState {
        world,
        body_a,
        body_b,
        port_a,
        port_b,
        session,
        joint: None,
        elapsed_s: 0.0,
        last_report_s: 0.0,
        load_elapsed_s: 0.0,
        finished: false,
    });
    eprintln!("[docking-demo] started: two craft approaching at 0.04 m/s relative speed");
}

pub(super) fn step(
    time: Res<Time>,
    mut demo: ResMut<DockingDemoState>,
    mut visuals: Query<(&DockingDemoVisual, &mut Transform)>,
    mut hud: Query<&mut Text, With<DockingDemoHud>>,
    mut audio: MessageWriter<crate::audio::AudioCue>,
) {
    if demo.finished {
        return;
    }
    let frame_dt = time.delta_secs_f64().clamp(0.0, 0.05);
    if frame_dt == 0.0 {
        return;
    }
    demo.elapsed_s += frame_dt;
    let steps = ((frame_dt / STEP_S).ceil() as usize).clamp(1, 8);
    let step_s = frame_dt / steps as f64;
    let body_a = demo.body_a;
    let body_b = demo.body_b;
    let port_a_position = demo.port_a.local_position_m;
    let port_a_orientation = demo.port_a.local_orientation;
    let port_b_position = demo.port_b.local_position_m;
    let port_b_orientation = demo.port_b.local_orientation;
    for _ in 0..steps {
        let wrench_a = if demo.session.state == DockingPortState::PressureEqualized {
            demo.load_elapsed_s += step_s;
            ExternalWrench {
                force_inertial_n: glam::DVec3::new(FORCE_AFTER_EQUALIZATION_N, 0.0, 0.0),
                torque_inertial_nm: glam::DVec3::ZERO,
            }
        } else {
            ExternalWrench::ZERO
        };
        demo.world
            .step(step_s, [(body_a, wrench_a), (body_b, ExternalWrench::ZERO)])
            .expect("docking demo Rapier step");

        let state_a = demo.world.body_state(body_a).expect("craft A readback");
        let state_b = demo.world.body_state(body_b).expect("craft B readback");
        let relative_position = port_world_position(state_b, port_b_position)
            - port_world_position(state_a, port_a_position);
        let relative_orientation =
            state_a.orientation_body_to_inertial.inverse() * state_b.orientation_body_to_inertial;
        let relative_velocity = state_b.velocity_inertial_mps - state_a.velocity_inertial_mps;

        if demo.session.state == DockingPortState::SoftCapture
            && relative_position.length() <= 0.025
        {
            let kinematics =
                DockingKinematics::new(relative_position, relative_orientation, relative_velocity)
                    .expect("finite docking demo kinematics");
            if demo.session.align(kinematics).is_ok() {
                audio.write(crate::audio::AudioCue::DockingImpact {
                    relative_speed_mps: relative_velocity.length(),
                });
                demo.session.hard_dock().expect("demo hard dock transition");
                demo.joint = Some(
                    demo.world
                        .attach_fixed_joint(
                            body_a,
                            body_b,
                            port_a_position,
                            port_a_orientation,
                            port_b_position,
                            port_b_orientation,
                        )
                        .expect("demo Rapier hard dock"),
                );
                demo.session
                    .engage_outer_structure()
                    .expect("demo outer structure");
                eprintln!("[docking-demo] aligned -> hard_dock -> outer_structure_engaged");
            }
        }
        if demo.session.state == DockingPortState::OuterStructureEngaged {
            demo.session
                .advance_pressure_equalization(step_s)
                .expect("demo pressure equalization");
            if demo.session.state == DockingPortState::PressureEqualized {
                eprintln!("[docking-demo] pressure_equalized; applying 350 N load to craft A");
            }
        }
        if demo.session.state == DockingPortState::PressureEqualized && demo.load_elapsed_s >= 2.0 {
            let joint = demo.joint.take().expect("demo joint before undock");
            demo.world.remove_joint(joint).expect("demo Rapier undock");
            demo.session.undock().expect("demo undock transition");
            demo.finished = true;
            eprintln!("[docking-demo] undock complete; Rapier joint removed cleanly");
        }
    }

    let state_a = demo.world.body_state(body_a).expect("craft A readback");
    let state_b = demo.world.body_state(body_b).expect("craft B readback");
    for (visual, mut transform) in &mut visuals {
        let state = if visual.body == 0 { state_a } else { state_b };
        let local_position = DVec3::new(
            visual.local_position_m.x as f64,
            visual.local_position_m.y as f64,
            visual.local_position_m.z as f64,
        );
        let world_position =
            state.position_inertial_m + state.orientation_body_to_inertial * local_position;
        transform.translation = Vec3::new(
            world_position.x as f32,
            world_position.y as f32,
            world_position.z as f32,
        );
        transform.rotation = Quat::from_xyzw(
            state.orientation_body_to_inertial.x as f32,
            state.orientation_body_to_inertial.y as f32,
            state.orientation_body_to_inertial.z as f32,
            state.orientation_body_to_inertial.w as f32,
        );
    }
    let port_gap = (port_world_position(state_b, port_b_position)
        - port_world_position(state_a, port_a_position))
    .length();
    if demo.elapsed_s - demo.last_report_s >= 1.0 {
        demo.last_report_s = demo.elapsed_s;
        eprintln!(
            "[docking-demo] t={:.1}s state={:?} port_gap={:.3}m rel_speed={:.3}m/s craft_a_v={:.3} craft_b_v={:.3}",
            demo.elapsed_s,
            demo.session.state,
            port_gap,
            (state_b.velocity_inertial_mps - state_a.velocity_inertial_mps).length(),
            state_a.velocity_inertial_mps.length(),
            state_b.velocity_inertial_mps.length(),
        );
    }
    for mut text in &mut hud {
        text.0 = format!(
            "DOCKING DEMO\nstate: {:?}\nport gap: {:.3} m\nload time: {:.1} s\n{}",
            demo.session.state,
            port_gap,
            demo.load_elapsed_s,
            if demo.finished {
                "DONE — undocked"
            } else {
                "LIVE RAPIER"
            },
        );
    }
}

fn spawn_visual(
    commands: &mut Commands,
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    body: u8,
    local_position_m: Vec3,
    name: &str,
) {
    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(material),
        DockingDemoVisual {
            body,
            local_position_m,
        },
        Name::new(name.to_string()),
        Transform::default(),
    ));
}

fn demo_geometry() -> CollisionGeometry {
    CollisionGeometry::new(vec![
        CollisionPart::new(
            glam::DVec3::ZERO,
            glam::DQuat::IDENTITY,
            CollisionShape::Cuboid {
                half_extents_m: glam::DVec3::new(0.55, 0.45, 0.45),
            },
            CollisionMaterial::default(),
        )
        .expect("valid demo collision part"),
    ])
    .expect("valid demo geometry")
}

fn port_world_position(state: RigidBodyState, local_position_m: glam::DVec3) -> glam::DVec3 {
    state.position_inertial_m + state.orientation_body_to_inertial * local_position_m
}
