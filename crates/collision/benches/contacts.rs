use std::{collections::BTreeMap, time::Instant};

use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{
    ArticulatedWheelBinding, CollisionFrame, CollisionWorld, DynamicBodyConfig, ExternalWrench,
};
use thessa_sim_core::{
    CollisionGeometry, CollisionMaterial, CollisionPart, CollisionShape, LandingLegSpec,
    LandingLegState, LandingShockAbsorberSpec, RigidBodyProperties, RigidBodyState,
    TireConstruction, WheelBrakeSpec, WheelChassisSpec, WheelLayout, WheelStrutSpec, WheelTireSpec,
};

const DT: f64 = 1.0 / 120.0;
/// Active-body sweep from the integration plan. CI only compiles this bench
/// (`--no-run`); local runs execute every size so the scheduler granularity
/// choice has numbers for both `parallel` settings.
const BODY_COUNTS: [usize; 5] = [1, 8, 64, 256, 1024];
const WHEEL_COUNTS: [u16; 4] = [1, 4, 16, 64];
const LANDING_LEG_COUNTS: [usize; 4] = [3, 4, 8, 16];

fn body_count_steps(body_count: usize) -> usize {
    // Keep wall time bounded: large scenes run fewer timed steps while still
    // settling through the same warmup.
    (614_400 / body_count.max(1)).clamp(60, 600)
}

fn wheel_chassis(
    wheel_count: u16,
) -> Result<thessa_sim_core::CompiledWheelChassis, Box<dyn std::error::Error>> {
    let axle_count = (wheel_count / 2).max(1);
    let radius_m = 0.25;
    let length_m = if wheel_count <= 2 {
        0.5
    } else {
        2.0 * radius_m * f64::from(axle_count - 1) + 0.5
    };
    Ok(WheelChassisSpec {
        name: format!("wheel-bench-{wheel_count}"),
        mount_position_body_m: DVec3::new(0.0, 0.0, 0.7),
        mount_orientation_body: DQuat::IDENTITY,
        length_m,
        layout: if wheel_count == 1 {
            WheelLayout::Inline
        } else {
            WheelLayout::AxlePairs { track_width_m: 0.8 }
        },
        wheel_count,
        structural_mass_kg: 20.0,
        structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(5.0)),
        tire: WheelTireSpec {
            // This authored test wheel is intentionally soft enough for the
            // 120-Hz contact step to keep its radial and suspension modes
            // loaded throughout the articulated sweep.
            construction: TireConstruction::Airless {
                structure: thessa_sim_core::AirlessWheelStructure::Spoked { spoke_count: 24 },
                structure_density_kg_m3: 4_400.0,
                minimum_temperature_k: 80.0,
                maximum_temperature_k: 500.0,
            },
            radius_m,
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
            extended_length_m: 0.5,
            stroke_m: 0.18,
            spring_rate_n_m: 10_000.0,
            damping_n_s_m: 200.0,
            preload_n: 0.0,
            minimum_force_n: 0.0,
            maximum_force_n: 8_000.0,
            mass_per_wheel_kg: 0.5,
        },
        brake: WheelBrakeSpec {
            maximum_torque_nm: 250.0,
            response_time_s: 0.1,
            mass_per_wheel_kg: 0.3,
        },
        drive: None,
        retraction: None,
    }
    .compile()?)
}

fn wheel_query_steps(wheel_count: u16) -> usize {
    (32_000 / usize::from(wheel_count)).clamp(200, 10_000)
}

fn landing_leg_specs(
    leg_count: usize,
) -> Result<Vec<thessa_sim_core::CompiledLandingLeg>, Box<dyn std::error::Error>> {
    let mut legs = Vec::with_capacity(leg_count);
    for index in 0..leg_count {
        let angle = std::f64::consts::TAU * index as f64 / leg_count as f64;
        let spec = LandingLegSpec {
            name: format!("landing-bench-{index}"),
            mount_position_body_m: DVec3::new(angle.cos(), angle.sin(), 0.0),
            hinge_axis_body: DVec3::Y,
            stowed_leg_axis_body: DVec3::Z,
            stowed_angle_rad: 0.0,
            deployed_angle_rad: std::f64::consts::PI,
            initially_deployed: true,
            deployment_rate_rad_s: 0.8,
            actuator_max_torque_nm: 20_000.0,
            leg_length_m: 2.0,
            leg_mass_kg: 18.0,
            footpad_radius_m: 0.2,
            footpad_mass_kg: 3.0,
            footpad_friction: 0.8,
            footpad_slip_stiffness_n_per_mps: 5_000.0,
            shock_absorber: LandingShockAbsorberSpec::Reusable {
                stroke_m: 0.2,
                spring_rate_n_m: 100_000.0,
                damping_n_s_m: 4_000.0,
                preload_n: 0.0,
                bottom_out_stiffness_n_m: 300_000.0,
                maximum_force_n: 100_000.0,
            },
        };
        legs.push(spec.compile()?);
    }
    Ok(legs)
}

fn articulated_wheel_step(
    world: &mut CollisionWorld,
    sprung_body: thessa_collision::CollisionBodyId,
    chassis: &thessa_sim_core::CompiledWheelChassis,
    bindings: &[ArticulatedWheelBinding],
    body_masses: &[(thessa_collision::CollisionBodyId, f64)],
) -> Result<usize, Box<dyn std::error::Error>> {
    let mut wrenches = BTreeMap::new();
    for (body, mass_kg) in body_masses {
        wrenches.insert(
            *body,
            ExternalWrench {
                force_inertial_n: DVec3::new(0.0, 0.0, -9.81 * mass_kg),
                torque_inertial_nm: DVec3::ZERO,
            },
        );
    }
    let wheel_forces = world.evaluate_articulated_wheel_contacts(sprung_body, chassis, bindings)?;
    let loaded_wheels = wheel_forces
        .iter()
        .filter(|force| {
            force
                .contact
                .is_some_and(|contact| contact.normal_load_n > 0.0)
        })
        .count();
    for wheel_force in wheel_forces {
        for (body, wrench) in [
            (sprung_body, wheel_force.sprung_wrench),
            (wheel_force.wheel_body, wheel_force.wheel_wrench),
        ] {
            let accumulated = wrenches.entry(body).or_insert(ExternalWrench::ZERO);
            accumulated.force_inertial_n += wrench.force_inertial_n;
            accumulated.torque_inertial_nm += wrench.torque_inertial_nm;
        }
    }
    world.step(DT, wrenches)?;
    Ok(loaded_wheels)
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

    println!("wheel ray-query benchmark (warm broad phase, fixed terrain)");
    for wheel_count in WHEEL_COUNTS {
        let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;
        world.insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(100.0, 100.0, 0.5),
            CollisionMaterial::new(0.7, 0.0)?,
        )?;
        let properties =
            RigidBodyProperties::new(100.0, DMat3::from_diagonal(DVec3::splat(100.0)))?;
        let geometry = CollisionGeometry::new(vec![CollisionPart::new(
            DVec3::new(0.0, 0.0, 2.0),
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )?])?;
        let body = world.insert_dynamic_body(
            RigidBodyState::stationary(DVec3::ZERO),
            properties,
            &geometry,
            DynamicBodyConfig {
                full_ccd: false,
                can_sleep: true,
            },
        )?;
        world.step(DT, [(body, ExternalWrench::ZERO)])?;
        let chassis = wheel_chassis(wheel_count)?;
        let spin_rates = vec![0.0; usize::from(wheel_count)];
        for _ in 0..20 {
            let _ = world.evaluate_wheel_contacts(body, &chassis, &spin_rates)?;
        }
        let steps = wheel_query_steps(wheel_count);
        let started = Instant::now();
        for _ in 0..steps {
            let _ = world.evaluate_wheel_contacts(body, &chassis, &spin_rates)?;
        }
        let elapsed = started.elapsed();
        let wheel_steps = usize::from(wheel_count) * steps;
        println!(
            "wheel queries: wheels={wheel_count} steps={steps} elapsed={elapsed:?} \
             wheel-steps/s={:.0}",
            wheel_steps as f64 / elapsed.as_secs_f64(),
        );
    }

    println!("articulated wheel benchmark (sensor bodies, slider/spin joints, fixed terrain)");
    for wheel_count in WHEEL_COUNTS {
        let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;
        world.insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(100.0, 100.0, 0.5),
            CollisionMaterial::new(0.7, 0.0)?,
        )?;
        let chassis = wheel_chassis(wheel_count)?;
        let wheel_masses = chassis.wheel_body_mass_properties(0);
        // Keep the per-wheel sprung load within this bench tire's rated
        // capacity across the wheel-count sweep.
        let root_mass_kg = 100.0 * f64::from(wheel_count);
        let root_properties = RigidBodyProperties::new(
            root_mass_kg,
            DMat3::from_diagonal(DVec3::splat(root_mass_kg)),
        )?;
        let root_geometry = CollisionGeometry::new(vec![CollisionPart::new(
            DVec3::new(0.0, 0.0, 3.0),
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )?])?;
        let sprung_load_per_wheel_n = root_mass_kg * 9.81 / f64::from(wheel_count);
        let wheel_load_per_wheel_n = sprung_load_per_wheel_n + wheel_masses[0].mass_kg * 9.81;
        let strut_compression_m = sprung_load_per_wheel_n / chassis.spec.strut.spring_rate_n_m;
        let tire_compression_m = wheel_load_per_wheel_n / chassis.spec.tire.radial_stiffness_n_m;
        let root_height_m = chassis.spec.tire.radius_m
            - tire_compression_m
            - chassis.wheel_stations[0].position_body_m.z
            - strut_compression_m;
        let root_state = RigidBodyState::stationary(DVec3::new(0.0, 0.0, root_height_m));
        let root = world.insert_dynamic_body(
            root_state,
            root_properties,
            &root_geometry,
            DynamicBodyConfig {
                full_ccd: false,
                can_sleep: false,
            },
        )?;
        let wheel_geometry = CollisionGeometry::new(vec![CollisionPart::new(
            DVec3::ZERO,
            DQuat::IDENTITY,
            CollisionShape::Sphere {
                radius_m: chassis.spec.tire.radius_m,
            },
            CollisionMaterial::new(chassis.spec.tire.surface_friction, 0.0)?,
        )?])?;
        let slide_axis = chassis.spec.mount_orientation_body * DVec3::NEG_Z;
        let mut bindings = Vec::with_capacity(usize::from(wheel_count));
        let mut body_masses = vec![(root, root_properties.mass_kg)];
        for (station, wheel_mass) in chassis.wheel_stations.iter().zip(wheel_masses) {
            let properties =
                RigidBodyProperties::new(wheel_mass.mass_kg, wheel_mass.inertia_body_kg_m2)?;
            let wheel_state = RigidBodyState::stationary(
                root_state.position_inertial_m + wheel_mass.center_of_mass_body_m
                    - slide_axis * strut_compression_m,
            );
            let wheel = world.insert_dynamic_sensor_body(
                wheel_state,
                properties,
                &wheel_geometry,
                DynamicBodyConfig {
                    full_ccd: false,
                    can_sleep: false,
                },
            )?;
            body_masses.push((wheel, wheel_mass.mass_kg));
            world.attach_suspension_wheel_joint(
                root,
                wheel,
                station.position_body_m,
                DVec3::ZERO,
                slide_axis,
                station.axle_axis_body,
                slide_axis,
                station.axle_axis_body,
                [-chassis.spec.strut.stroke_m, 0.0],
            )?;
            bindings.push(ArticulatedWheelBinding {
                wheel_body: wheel,
                wheel_index: station.index,
                nominal_center_sprung_local_m: station.position_body_m,
                slide_axis_sprung_local: slide_axis,
                axle_axis_sprung_local: station.axle_axis_body,
            });
        }

        // Populate Rapier's broad phase and settle the freshly attached
        // anchors before applying the first articulated wheel wrench.
        world.step(DT, [])?;
        let initial_contacts =
            world.evaluate_articulated_wheel_contacts(root, &chassis, &bindings)?;
        assert!(
            initial_contacts.iter().any(|force| force
                .contact
                .is_some_and(|contact| contact.normal_load_n > 0.0)),
            "wheel benchmark must initialize in loaded contact: {initial_contacts:?}"
        );
        for _ in 0..120 {
            let _ = articulated_wheel_step(&mut world, root, &chassis, &bindings, &body_masses)?;
        }
        let loaded_wheels = world
            .evaluate_articulated_wheel_contacts(root, &chassis, &bindings)?
            .iter()
            .filter(|force| {
                force
                    .contact
                    .is_some_and(|contact| contact.normal_load_n > 0.0)
            })
            .count();
        assert!(
            loaded_wheels > 0,
            "wheel benchmark must retain loaded contacts: root={:?}, wheel={:?}, contacts={:?}",
            world.body_state(root)?,
            world.body_state(bindings[0].wheel_body)?,
            world.evaluate_articulated_wheel_contacts(root, &chassis, &bindings)?
        );
        let steps = wheel_query_steps(wheel_count);
        let mut loaded_wheel_steps = 0;
        let mut minimum_loaded_wheels = usize::MAX;
        let started = Instant::now();
        for _ in 0..steps {
            let loaded =
                articulated_wheel_step(&mut world, root, &chassis, &bindings, &body_masses)?;
            loaded_wheel_steps += loaded;
            minimum_loaded_wheels = minimum_loaded_wheels.min(loaded);
        }
        let elapsed = started.elapsed();
        assert!(
            minimum_loaded_wheels > 0,
            "wheel benchmark lost all loaded contacts during timing: \
             min={minimum_loaded_wheels}, total={loaded_wheel_steps}"
        );
        let wheel_steps = usize::from(wheel_count) * steps;
        println!(
            "articulated wheels: wheels={wheel_count} loaded={loaded_wheels} \
             loaded_min={minimum_loaded_wheels} \
             steps={steps} elapsed={elapsed:?} \
             wheel-steps/s={:.0}",
            wheel_steps as f64 / elapsed.as_secs_f64(),
        );
    }

    println!("landing-leg benchmark (terrain queries, shock state, foot friction, applied loads)");
    for leg_count in LANDING_LEG_COUNTS {
        let mut world = CollisionWorld::new(CollisionFrame::inertial_at(DVec3::ZERO, DVec3::ZERO))?;
        world.insert_static_cuboid(
            DVec3::new(0.0, 0.0, -0.5),
            DQuat::IDENTITY,
            DVec3::new(100.0, 100.0, 0.5),
            CollisionMaterial::new(0.6, 0.0)?,
        )?;
        let legs = landing_leg_specs(leg_count)?;
        let mass_kg = 1_000.0 + 21.0 * leg_count as f64;
        let properties =
            RigidBodyProperties::new(mass_kg, DMat3::from_diagonal(DVec3::splat(mass_kg * 2.0)))?;
        let geometry = CollisionGeometry::new(vec![CollisionPart::new(
            DVec3::new(0.0, 0.0, 3.0),
            DQuat::IDENTITY,
            CollisionShape::Sphere { radius_m: 0.1 },
            CollisionMaterial::default(),
        )?])?;
        let compression_m = mass_kg * 9.81 / (leg_count as f64 * 100_000.0);
        let root = world.insert_dynamic_body(
            RigidBodyState::stationary(DVec3::new(0.0, 0.0, 2.0 + 0.2 - compression_m)),
            properties,
            &geometry,
            DynamicBodyConfig {
                full_ccd: false,
                can_sleep: false,
            },
        )?;
        world.step(DT, [])?;
        let mut states: Vec<LandingLegState> =
            legs.iter().map(|leg| leg.spec.initial_state()).collect();
        let gravity = ExternalWrench {
            force_inertial_n: DVec3::new(0.0, 0.0, -9.81 * mass_kg),
            torque_inertial_nm: DVec3::ZERO,
        };
        let steps = 5_000 / leg_count;
        let mut loaded_leg_steps = 0usize;
        let mut minimum_loaded_legs = usize::MAX;
        let started = Instant::now();
        for _ in 0..steps {
            let result =
                world.evaluate_landing_leg_contacts(root, &legs, &states, DVec3::ZERO, true, DT)?;
            let loaded = result
                .contacts
                .iter()
                .filter(|contact| contact.normal_load_n > 0.0)
                .count();
            loaded_leg_steps += loaded;
            minimum_loaded_legs = minimum_loaded_legs.min(loaded);
            states = result.states;
            let mut wrench = gravity;
            wrench.force_inertial_n += result.wrench.force_inertial_n;
            wrench.torque_inertial_nm += result.wrench.torque_inertial_nm;
            world.step(DT, [(root, wrench)])?;
        }
        let elapsed = started.elapsed();
        assert!(
            minimum_loaded_legs > 0,
            "landing-leg bench lost all loaded footpads for count={leg_count}"
        );
        let leg_steps = leg_count * steps;
        println!(
            "landing legs: legs={leg_count} loaded_min={minimum_loaded_legs} \
             steps={steps} elapsed={elapsed:?} leg-steps/s={:.0} loaded={loaded_leg_steps}",
            leg_steps as f64 / elapsed.as_secs_f64(),
        );
    }
    Ok(())
}
