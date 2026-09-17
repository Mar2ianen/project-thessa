use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, RigidBodyProperties,
    RigidBodyState,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;

    world.insert_static_cuboid(
        DVec3::new(0.0, -0.5, 0.0),
        DQuat::IDENTITY,
        DVec3::new(25.0, 0.5, 25.0),
        CollisionMaterial::new(0.8, 0.0)?,
    )?;

    let radius_m = 0.5;
    let mass_kg = 250.0;
    let inertia = 0.4 * mass_kg * radius_m * radius_m;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(inertia)))?;
    let geometry = CollisionGeometry::new(vec![CollisionPart::new(
        DVec3::ZERO,
        DQuat::IDENTITY,
        CollisionShape::Sphere { radius_m },
        CollisionMaterial::new(0.8, 0.0)?,
    )?])?;
    let body = world.insert_dynamic_body(
        RigidBodyState::stationary(DVec3::new(0.0, 8.0, 0.0)),
        properties,
        &geometry,
        DynamicBodyConfig::default(),
    )?;

    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    let dt = 1.0 / 120.0;
    for _ in 0..(6 * 120) {
        world.step(dt, [(body, gravity)])?;
    }

    let state = world.body_state(body)?;
    println!(
        "settled: y={:.6} m speed={:.6} m/s omega={:.6} rad/s",
        state.position_inertial_m.y,
        state.velocity_inertial_mps.length(),
        state.angular_velocity_body_rps.length(),
    );

    if (state.position_inertial_m.y - radius_m).abs() > 0.05 {
        return Err("body did not settle on the floor within the expected error envelope".into());
    }
    Ok(())
}
