//! Contact debug probe: lands the compiled X-15 contact compound on a
//! kinematic runway platform, rides it upward, and prints JSON telemetry.
//!
//! The craft touches down on the moving platform (not beside it), so the
//! probe exercises the ephemeris-driven kinematic terrain seam end to end:
//! settle under gravity, then carry while the prescribed platform pose
//! rises. Run with `cargo run -p thessa-collision --example contact_debug`.
//! The output is the same [`CollisionDebugSnapshot`] regression fixtures
//! assert on, so a bad live state can be reproduced outside the renderer
//! and diffed against a known-good capture.

use glam::{DQuat, DVec3};
use thessa_collision::{CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench};
use thessa_sim_core::{
    CollisionMaterial, RigidBodyProperties, RigidBodyState, x15_contact_geometry,
};

/// Platform rise rate in m/s during the carry phase.
const PLATFORM_RISE_MPS: f64 = 0.25;
/// X-15 belly-down rest height above the platform top (fuselage keel).
const REST_HEIGHT_M: f64 = 0.65;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;

    // Kinematic runway platform, top at y=0. Its pose is prescribed per tick
    // exactly how the ephemeris/body-rotation model will drive planetary
    // terrain in production.
    let platform = world.insert_kinematic_cuboid(
        DVec3::new(0.0, -0.5, 0.0),
        DQuat::IDENTITY,
        DVec3::new(15.0, 0.5, 15.0),
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
    // Phase 1 (4 s): settle onto the stationary platform.
    for step in 0..(4 * 120) {
        world.step(dt, [(body, gravity)])?;
        if step % 120 == 119 {
            let snapshot = world.debug_snapshot()?;
            println!("{}", serde_json::to_string_pretty(&snapshot)?);
        }
    }
    let settled = world.body_state(body)?;
    if (settled.position_inertial_m.y - REST_HEIGHT_M).abs() > 0.15 {
        return Err(format!(
            "x-15 did not settle on the platform (y={:.3} m)",
            settled.position_inertial_m.y
        )
        .into());
    }

    // Phase 2 (4 s): the platform rises 1 m carrying the landed craft.
    let mut lift_m = 0.0;
    for step in 0..(4 * 120) {
        lift_m = (step + 1) as f64 * dt * PLATFORM_RISE_MPS;
        world.set_next_kinematic_pose(
            platform,
            DVec3::new(0.0, -0.5 + lift_m, 0.0),
            DQuat::IDENTITY,
        )?;
        world.step(dt, [(body, gravity)])?;
        if step % 120 == 119 {
            let snapshot = world.debug_snapshot()?;
            println!("{}", serde_json::to_string_pretty(&snapshot)?);
        }
    }

    let state = world.body_state(body)?;
    let expected_y = REST_HEIGHT_M + lift_m;
    println!(
        "x15 carried: y={:.3} m (platform lift {:.3} m) speed={:.3} m/s touching={}",
        state.position_inertial_m.y,
        lift_m,
        state.velocity_inertial_mps.length(),
        world.touching_contact_pair_count(),
    );
    if (state.position_inertial_m.y - expected_y).abs() > 0.15 {
        return Err(format!(
            "landed x-15 did not ride the platform (y={:.3}, expected {expected_y:.3})",
            state.position_inertial_m.y
        )
        .into());
    }
    if world.touching_contact_pair_count() < 1 {
        return Err("carry phase lost contact with the platform".into());
    }
    Ok(())
}
