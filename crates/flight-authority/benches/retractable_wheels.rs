use std::time::Instant;

use glam::{DMat3, DQuat, DVec3};
use thessa_collision::CollisionFrame;
use thessa_flight_authority::{ContactActivation, ContactRuntime};
use thessa_sim_core::{
    AeroEnvironment, AeroGeometry, AeroResult, CollisionGeometry, CollisionMaterial, CollisionPart,
    CollisionShape, FlightForces, RigidBodyProperties, RigidBodyState, TireConstruction,
    VehicleDefinition, WheelBrakeSpec, WheelChassisRetractionSpec, WheelChassisSpec, WheelLayout,
    WheelStrutSpec, WheelTireSpec,
};

const STEP_S: f64 = 1.0 / 120.0;
const WHEEL_COUNTS: [u16; 4] = [1, 4, 16, 64];

fn vehicle(wheel_count: u16) -> Result<VehicleDefinition, Box<dyn std::error::Error>> {
    let tire_radius_m = 0.25;
    let length_m = 2.0 * tire_radius_m * f64::from(wheel_count.saturating_sub(1)) + 0.5;
    let spec = WheelChassisSpec {
        name: format!("retraction-bench-{wheel_count}"),
        mount_position_body_m: DVec3::new(0.0, 0.0, 1.0),
        mount_orientation_body: DQuat::IDENTITY,
        length_m,
        layout: WheelLayout::Inline,
        wheel_count,
        structural_mass_kg: 20.0,
        structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(4.0)),
        tire: WheelTireSpec {
            construction: TireConstruction::Airless {
                structure: thessa_sim_core::AirlessWheelStructure::Spoked { spoke_count: 24 },
                structure_density_kg_m3: 4_400.0,
                minimum_temperature_k: 80.0,
                maximum_temperature_k: 500.0,
            },
            radius_m: tire_radius_m,
            width_m: 0.2,
            mass_kg: 2.0,
            spin_inertia_kg_m2: 0.08,
            radial_stiffness_n_m: 20_000.0,
            radial_damping_n_s_m: 150.0,
            longitudinal_slip_stiffness_n_per_mps: 500.0,
            lateral_slip_stiffness_n_per_mps: 500.0,
            maximum_deflection_m: 0.06,
            maximum_load_n: 4_000.0,
            surface_friction: 0.85,
        },
        strut: WheelStrutSpec {
            extended_length_m: 0.4,
            stroke_m: 0.15,
            spring_rate_n_m: 30_000.0,
            damping_n_s_m: 1_000.0,
            preload_n: 0.0,
            minimum_force_n: 0.0,
            maximum_force_n: 10_000.0,
            mass_per_wheel_kg: 0.5,
        },
        brake: WheelBrakeSpec {
            maximum_torque_nm: 250.0,
            response_time_s: 0.1,
            mass_per_wheel_kg: 0.3,
        },
        drive: None,
        retraction: Some(WheelChassisRetractionSpec {
            pivot_position_body_m: DVec3::new(0.0, 0.0, 1.0),
            hinge_axis_body: DVec3::Y,
            stowed_angle_rad: -std::f64::consts::FRAC_PI_2,
            deployed_angle_rad: 0.0,
            initially_deployed: false,
            deployment_rate_rad_s: 0.8,
            actuator_max_torque_nm: 12_000.0,
        }),
    };
    let geometry = CollisionGeometry::new(vec![CollisionPart::new(
        DVec3::new(0.0, 0.0, 30.0),
        DQuat::IDENTITY,
        CollisionShape::Sphere { radius_m: 0.1 },
        CollisionMaterial::default(),
    )?])?;
    let mut vehicle = VehicleDefinition::new(
        "retractable-wheel-benchmark",
        AeroGeometry::default(),
        RigidBodyProperties::new(2_000.0, DMat3::from_diagonal(DVec3::splat(2_000.0)))?,
        Vec::new(),
    )?
    .with_collision_geometry(geometry)?
    .with_wheel_chassis(vec![spec])?;
    vehicle.bake_wheel_chassis_masses()?;
    Ok(vehicle)
}

fn zero_forces() -> FlightForces {
    FlightForces {
        environment: AeroEnvironment::standard_sea_level(),
        aero: AeroResult {
            force_body_n: DVec3::ZERO,
            moment_body_nm: DVec3::ZERO,
            dynamic_pressure_pa: 0.0,
            mach: 0.0,
            reynolds_number: 0.0,
            panel_count: 0,
            panel_loads: None,
        },
        total_force_body_n: DVec3::ZERO,
        total_moment_body_nm: DVec3::ZERO,
        total_force_inertial_n: DVec3::ZERO,
        acceleration_inertial_mps2: DVec3::ZERO,
        angular_acceleration_body_rps2: DVec3::ZERO,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("contact-runtime retractable wheel sweep (deployment state, COM split, joints)");
    for wheel_count in WHEEL_COUNTS {
        let vehicle = vehicle(wheel_count)?;
        let chassis = &vehicle.wheel_chassis[0];
        let retraction = chassis.spec.retraction.expect("bench retractable chassis");
        let mut runtime = ContactRuntime::new(
            CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO),
            ContactActivation::new(50.0, 80.0)?,
        )?;
        runtime.attach_static_patch(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(100.0, 100.0, 0.5),
            CollisionMaterial::default(),
        )?;

        let mut state = RigidBodyState::stationary(DVec3::new(0.0, 0.0, 10.0));
        let mut wheel_spin = vec![vec![0.0; usize::from(wheel_count)]];
        let mut brake_states = vec![vec![Default::default(); usize::from(wheel_count)]];
        let mut gear_states = vec![retraction.initial_state()];
        let mut leg_states = Vec::new();
        let forces = zero_forces();
        let steps = (32_000 / usize::from(wheel_count)).clamp(500, 8_000);
        for index in 0..120 {
            (state, _, _, _, _, _) = runtime.step_articulated_vehicle_with_gear(
                STEP_S,
                state,
                DVec3::ZERO,
                &forces,
                &vehicle,
                0.0,
                0.0,
                &mut wheel_spin,
                &mut brake_states,
                &mut gear_states,
                &mut leg_states,
                index < 60,
            )?;
        }

        let started = Instant::now();
        for index in 0..steps {
            (state, _, _, _, _, _) = runtime.step_articulated_vehicle_with_gear(
                STEP_S,
                state,
                DVec3::ZERO,
                &forces,
                &vehicle,
                0.0,
                0.0,
                &mut wheel_spin,
                &mut brake_states,
                &mut gear_states,
                &mut leg_states,
                index < steps / 2,
            )?;
        }
        let elapsed = started.elapsed();
        println!(
            "retractable chassis: wheels={wheel_count} steps={steps} elapsed={elapsed:?} \
             wheel-steps/s={:.0} final_fraction={:.3}",
            f64::from(wheel_count) * steps as f64 / elapsed.as_secs_f64(),
            gear_states[0].deployment_fraction,
        );
    }
    Ok(())
}
