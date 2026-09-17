//! Contact debug probe: lands the compiled X-15 contact compound on a static
//! runway, rides a kinematic platform, and prints a JSON telemetry snapshot.
//!
//! Run with `cargo run -p thessa-collision --example contact_debug`. The
//! output is the same [`CollisionDebugSnapshot`] regression fixtures assert
//! on, so a bad live state can be reproduced outside the renderer and
//! diffed against a known-good capture.

use glam::{DQuat, DVec3};
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionMaterial, RigidBodyProperties, RigidBodyState, x15_contact_geometry,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;

    // Runway slab, top at y=0.
    world.insert_static_cuboid(
        DVec3::new(0.0, -0.5, 0.0),
        DQuat::IDENTITY,
        DVec3::new(60.0, 0.5, 60.0),
        CollisionMaterial::new(0.8, 0.0)?,
    )?;
    // Kinematic service platform parked beside the touchdown zone; it rises
    // 1 m during the probe to exercise the ephemeris-driven terrain seam.
    let platform = world.insert_kinematic_cuboid(
        DVec3::new(15.0, -0.5, 0.0),
        DQuat::IDENTITY,
        DVec3::new(3.0, 0.5, 3.0),
        CollisionMaterial::new(0.8, 0.0)?,
    )?;

    let profile = thessa_sim_core::X15StarterProfile::new().map(|profile| profile.vehicle)?;
    let properties = RigidBodyProperties::new(
        profile.mass_properties.mass_kg,
        profile.mass_properties.inertia_body_kg_m2,
    )?;
    let geometry = x15_contact_geometry()?;
    // Belly-down: the vehicle frame is +Z up while the probe world is +Y
    // up, so rotate body +Z onto world +Y (wings span world Z).
    let belly_down = DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    let body = world.insert_dynamic_body(
        RigidBodyState::new(
            DVec3::new(0.0, 6.0, 0.0),
            DVec3::ZERO,
            belly_down,
            DVec3::ZERO,
        )?,
        properties,
        &geometry,
        DynamicBodyConfig::default(),
    )?;

    let gravity = ExternalWrench {
        force_inertial_n: DVec3::new(0.0, -9.81 * properties.mass_kg, 0.0),
        torque_inertial_nm: DVec3::ZERO,
    };
    let dt = 1.0 / 120.0;
    for step in 0..(8 * 120) {
        if step >= 4 * 120 {
            let lift = (step - 4 * 120) as f64 * dt * 0.25;
            world.set_next_kinematic_pose(
                platform,
                DVec3::new(15.0, -0.5 + lift, 0.0),
                DQuat::IDENTITY,
            )?;
        }
        world.step(dt, [(body, gravity)])?;
        if step % 120 == 119 {
            let snapshot = world.debug_snapshot()?;
            println!("{}", serde_json::to_string_pretty(&snapshot)?);
        }
    }

    let state = world.body_state(body)?;
    println!(
        "x15 settled: y={:.3} m speed={:.3} m/s sleeping={}",
        state.position_inertial_m.y,
        state.velocity_inertial_mps.length(),
        world.dynamic_body_sleeping(body)?,
    );
    if !state.position_inertial_m.y.is_finite() || state.position_inertial_m.y < -1.0 {
        return Err("x-15 compound fell through the runway".into());
    }
    Ok(())
}
