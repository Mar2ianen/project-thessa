use std::time::Instant;

use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, RigidBodyProperties,
    RigidBodyState,
};

const DT: f64 = 1.0 / 120.0;
/// Active-body sweep from the integration plan. CI only compiles this bench
/// (`--no-run`); local runs execute every size so the scheduler granularity
/// choice has numbers for both `parallel` settings.
const BODY_COUNTS: [usize; 5] = [1, 8, 64, 256, 1024];

fn body_count_steps(body_count: usize) -> usize {
    // Keep wall time bounded: large scenes run fewer timed steps while still
    // settling through the same warmup.
    (614_400 / body_count.max(1)).clamp(60, 600)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(feature = "parallel")]
    let mode = "parallel";
    #[cfg(not(feature = "parallel"))]
    let mode = "serial";
    println!("collision bench mode={mode} dt={DT}");

    for body_count in BODY_COUNTS {
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

        let side = (body_count as f64).cbrt().ceil() as usize;
        let mut bodies = Vec::with_capacity(body_count);
        for index in 0..body_count {
            let x = (index % side) as f64 * 0.8 - side as f64 * 0.4;
            let z = ((index / side) % side) as f64 * 0.8 - side as f64 * 0.4;
            let layer = index / (side * side);
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

        for _ in 0..60 {
            world.step(DT, wrenches.iter().copied())?;
        }

        let steps = body_count_steps(body_count);
        let started = Instant::now();
        for _ in 0..steps {
            world.step(DT, wrenches.iter().copied())?;
        }
        let elapsed = started.elapsed();
        let body_steps = body_count * steps;
        let body_steps_per_s = body_steps as f64 / elapsed.as_secs_f64();
        let sim_seconds_per_wall_second = steps as f64 * DT / elapsed.as_secs_f64();
        let snapshot = world.debug_snapshot()?;

        println!(
            "collision bench: bodies={body_count} steps={steps} elapsed={elapsed:?} \
             body_steps/s={body_steps_per_s:.0} realtime_x={sim_seconds_per_wall_second:.2} \
             touching={} sleeping={}",
            snapshot.touching_contact_pairs,
            snapshot
                .dynamic_bodies
                .iter()
                .filter(|body| body.sleeping)
                .count(),
        );
    }
    Ok(())
}
