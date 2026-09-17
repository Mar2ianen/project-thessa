use std::time::Instant;

use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, RigidBodyProperties,
    RigidBodyState,
};

const BODY_COUNT: usize = 1_024;
const STEPS: usize = 600;
const DT: f64 = 1.0 / 120.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;
    world.insert_static_cuboid(
        DVec3::new(0.0, -0.5, 0.0),
        DQuat::IDENTITY,
        DVec3::new(200.0, 0.5, 200.0),
        CollisionMaterial::new(0.7, 0.0)?,
    )?;

    let radius_m = 0.25;
    let mass_kg = 10.0;
    let inertia = 0.4 * mass_kg * radius_m * radius_m;
    let properties =
        RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(inertia)))?;
    let geometry = CollisionGeometry::new(vec![CollisionPart::new(
        DVec3::ZERO,
        DQuat::IDENTITY,
        CollisionShape::Sphere { radius_m },
        CollisionMaterial::new(0.7, 0.0)?,
    )?])?;

    let mut bodies = Vec::with_capacity(BODY_COUNT);
    for index in 0..BODY_COUNT {
        let x = (index % 32) as f64 * 0.8 - 12.4;
        let z = ((index / 32) % 32) as f64 * 0.8 - 12.4;
        let layer = index / (32 * 32);
        let y = 0.5 + layer as f64 * 0.8;
        bodies.push(world.insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(x, y, z)),
            properties,
            &geometry,
            DynamicBodyConfig {
                full_ccd: false,
                can_sleep: true,
            },
        )?);
    }

    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    let wrenches = bodies
        .iter()
        .copied()
        .map(|body| (body, gravity))
        .collect::<Vec<_>>();

    for _ in 0..120 {
        world.step(DT, wrenches.iter().copied())?;
    }

    let started = Instant::now();
    for _ in 0..STEPS {
        world.step(DT, wrenches.iter().copied())?;
    }
    let elapsed = started.elapsed();
    let body_steps = BODY_COUNT * STEPS;
    let body_steps_per_s = body_steps as f64 / elapsed.as_secs_f64();
    let sim_seconds_per_wall_second = STEPS as f64 * DT / elapsed.as_secs_f64();

    println!(
        "collision bench: bodies={BODY_COUNT} steps={STEPS} elapsed={elapsed:?} body_steps/s={body_steps_per_s:.0} realtime_x={sim_seconds_per_wall_second:.2}"
    );
    Ok(())
}
