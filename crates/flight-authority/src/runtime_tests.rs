use super::*;
use thessa_sim_core::{
    AeroModel, AirlessWheelStructure, ControlHinge, ControlSurfaceActuator, ElectricMotorSpec,
    LandingLegSpec, LandingShockAbsorberSpec, ParachuteCommand, ParachutePhase, ParachuteSpec,
    ReactionWheelBankSpec, RigidBodyProperties, SystemConfig, TireConstruction, VehiclePartCommand,
    WheelBrakeSpec, WheelChassisRetractionSpec, WheelChassisSpec, WheelDriveSpec, WheelLayout,
    WheelStrutSpec, WheelTireSpec,
};

struct NeverReadyBakeQueue {
    pending: bool,
}

impl BakeQueue for NeverReadyBakeQueue {
    fn request_bake(&mut self, _request: RailsBakeRequest) {
        self.pending = true;
    }

    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>> {
        None
    }

    fn has_pending(&self) -> bool {
        self.pending
    }

    fn reset(&mut self) {
        self.pending = false;
    }
}

fn fixture() -> (BakedEphemeris, FlightAuthority) {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let runtime = FlightAuthority::new(&ephemeris, ephemeris.body_id("thessa").unwrap()).unwrap();
    (ephemeris, runtime)
}

#[test]
fn display_loads_is_repeatable_and_does_not_advance_authority_state() {
    let (ephemeris, mut flight) = fixture();
    flight.control_input = DVec3::new(0.3, -0.2, 0.1);
    flight.surface_input = DVec3::new(-0.1, 0.2, -0.3);

    let capture_state = |flight: &FlightAuthority| {
        (
            flight.state,
            flight.sas_target_orientation,
            flight.surface_input,
            flight.control_deflections_rad.clone(),
            flight.vehicle.aero_geometry.clone(),
            flight.aero_panels.clone(),
            flight.reaction_wheel_torque_body_nm,
            flight.actuator_saturated,
            flight.trim_solves,
            flight.trim_second_passes,
            flight.last_gravity_acceleration_inertial_mps2,
            flight.last_forces.clone(),
        )
    };
    let before = capture_state(&flight);
    let time = SimTime(flight.flight_time_s);
    let first = flight
        .display_loads(&ephemeris, time, ControlMode::Direct)
        .expect("first display evaluation");
    assert_eq!(capture_state(&flight), before);
    let second = flight
        .display_loads(&ephemeris, time, ControlMode::Direct)
        .expect("second display evaluation");
    assert_eq!(capture_state(&flight), before);
    assert_eq!(first, second);
}

#[test]
fn disabling_contact_mode_clears_all_contact_telemetry() {
    let (_, mut flight) = fixture();
    flight.last_wheel_gear_actuators.push((
        0,
        WheelChassisActuatorPoint {
            target_fraction: 1.0,
            deployment_fraction: 0.5,
            resisting_torque_nm: 10.0,
            actuator_torque_nm: 10.0,
            stalled: false,
            moving: true,
        },
    ));
    flight
        .last_landing_leg_contacts
        .push(LandingLegContactSample {
            leg_index: 0,
            contact_point_inertial_m: DVec3::ZERO,
            terrain_normal_inertial: DVec3::Z,
            relative_contact_velocity_inertial_mps: DVec3::ZERO,
            compression_m: 0.01,
            compression_rate_mps: 0.0,
            axial_force_n: 100.0,
            normal_load_n: 100.0,
            tangential_force_inertial_n: DVec3::ZERO,
            contact_friction: 0.8,
            permanent_crush_m: 0.0,
            absorbed_energy_delta_j: 0.0,
            actuator_resisting_torque_nm: 0.0,
            bottomed_out: false,
            exhausted: false,
            saturated: false,
        });
    flight.last_landing_leg_actuators.push((
        0,
        LandingGearActuatorPoint {
            target_fraction: 1.0,
            deployment_fraction: 0.5,
            resisting_torque_nm: 10.0,
            actuator_torque_nm: 10.0,
            stalled: false,
            moving: true,
        },
    ));

    flight.disable_contact_mode();

    assert!(flight.wheel_gear_actuator_telemetry().is_empty());
    assert!(flight.landing_leg_contact_telemetry().is_empty());
    assert!(flight.landing_leg_actuator_telemetry().is_empty());
}

#[test]
fn leaving_contact_active_regime_clears_all_contact_telemetry() {
    let (_, mut flight) = fixture();
    assert!(!flight.vehicle.collision_geometry.is_empty());
    flight
        .enable_contact_mode(1.0, 2.0)
        .expect("contact mode enables");
    let planet_radius_m = flight.planet_radius_m;
    let kinematics = |altitude_m: f64| LocalAirKinematics {
        relative_position_inertial_m: DVec3::Z * (planet_radius_m + altitude_m),
        relative_position_body_m: DVec3::Z * (planet_radius_m + altitude_m),
        relative_velocity_inertial_mps: DVec3::ZERO,
        air_velocity_body_mps: DVec3::ZERO,
        surface_velocity_inertial_mps: DVec3::ZERO,
        radial_up: DVec3::Z,
        altitude_m,
    };
    assert!(
        flight
            .poll_contact_activation(kinematics(0.5))
            .expect("contact activation")
    );
    flight.last_wheel_contacts.push(WheelContactSample {
        wheel_index: 0,
        contact_point_inertial_m: DVec3::ZERO,
        terrain_normal_inertial: DVec3::Z,
        forward_axis_inertial: DVec3::X,
        lateral_axis_inertial: DVec3::Y,
        relative_contact_velocity_inertial_mps: DVec3::ZERO,
        radial_penetration_m: 0.0,
        strut_compression_m: 0.0,
        tire_compression_m: 0.0,
        compression_rate_mps: 0.0,
        normal_load_n: 0.0,
        longitudinal_force_n: 0.0,
        lateral_force_n: 0.0,
        contact_friction: 0.8,
        saturated: false,
    });
    flight.last_wheel_drive_points.push((
        0,
        0,
        WheelDrivePoint {
            motor_rpm: 0.0,
            motor_torque_nm: 0.0,
            requested_wheel_torque_per_driven_wheel_nm: 0.0,
            mechanical_power_w: 0.0,
            electrical_power_w: 0.0,
            waste_heat_w: 0.0,
            power_limited: false,
            thermal_limited: false,
            speed_limited: false,
        },
    ));
    flight.last_wheel_gear_actuators.push((
        0,
        WheelChassisActuatorPoint {
            target_fraction: 1.0,
            deployment_fraction: 0.5,
            resisting_torque_nm: 10.0,
            actuator_torque_nm: 10.0,
            stalled: false,
            moving: true,
        },
    ));
    flight.last_landing_leg_actuators.push((
        0,
        LandingGearActuatorPoint {
            target_fraction: 1.0,
            deployment_fraction: 0.5,
            resisting_torque_nm: 10.0,
            actuator_torque_nm: 10.0,
            stalled: false,
            moving: true,
        },
    ));

    assert!(
        !flight
            .poll_contact_activation(kinematics(3.0))
            .expect("contact exit")
    );

    assert!(flight.wheel_contact_telemetry().is_empty());
    assert!(flight.wheel_drive_telemetry().is_empty());
    assert!(flight.wheel_gear_actuator_telemetry().is_empty());
    assert!(flight.landing_leg_actuator_telemetry().is_empty());
}

#[test]
fn flight_authority_advances_and_retracts_foldout_legs_across_ticks() {
    let (_, mut flight) = fixture();
    let leg = LandingLegSpec {
        name: "runtime-foldout-leg".into(),
        mount_position_body_m: DVec3::ZERO,
        hinge_axis_body: DVec3::Y,
        stowed_leg_axis_body: DVec3::Z,
        stowed_angle_rad: 0.0,
        deployed_angle_rad: std::f64::consts::PI,
        initially_deployed: false,
        deployment_rate_rad_s: 1.0,
        actuator_max_torque_nm: 10_000.0,
        leg_length_m: 2.0,
        leg_mass_kg: 10.0,
        footpad_radius_m: 0.2,
        footpad_mass_kg: 1.0,
        footpad_friction: 0.7,
        footpad_slip_stiffness_n_per_mps: 1_000.0,
        shock_absorber: LandingShockAbsorberSpec::Reusable {
            stroke_m: 0.2,
            spring_rate_n_m: 30_000.0,
            damping_n_s_m: 1_000.0,
            preload_n: 0.0,
            bottom_out_stiffness_n_m: 200_000.0,
            maximum_force_n: 60_000.0,
        },
    };
    flight.vehicle = flight
        .vehicle
        .clone()
        .with_landing_legs(vec![leg])
        .expect("foldout leg compiles");
    flight.vehicle.bake_landing_leg_masses().unwrap();
    flight.set_gear_down(true);
    flight.advance_freeflight_landing_gear().unwrap();
    let deployed_fraction = flight.landing_leg_states()[0].deployment_fraction;
    assert!(deployed_fraction > 0.0 && deployed_fraction < 1.0);
    assert!(flight.landing_gear_transitioning());
    assert_eq!(flight.landing_leg_actuator_telemetry().len(), 1);

    flight
        .set_landing_leg_deployed("runtime-foldout-leg", false)
        .expect("named leg retract command");
    assert!(
        flight.gear_down,
        "part command leaves the group target intact"
    );
    flight.advance_freeflight_landing_gear().unwrap();
    assert!(flight.landing_leg_states()[0].deployment_fraction < deployed_fraction);
    assert!(!flight.landing_gear_transitioning());
    assert_eq!(
        flight.landing_leg_actuator_telemetry()[0].1.target_fraction,
        0.0
    );
}

fn install_airless_drive(flight: &mut FlightAuthority) {
    let spec = WheelChassisSpec {
        name: "authority-regolith-wheel".into(),
        mount_position_body_m: DVec3::new(0.0, 0.0, -1.05),
        mount_orientation_body: DQuat::IDENTITY,
        length_m: 1.0,
        layout: WheelLayout::Inline,
        wheel_count: 1,
        structural_mass_kg: 12.0,
        structural_inertia_local_kg_m2: DMat3::from_diagonal(DVec3::splat(0.5)),
        tire: WheelTireSpec {
            construction: TireConstruction::Airless {
                structure: AirlessWheelStructure::Spoked { spoke_count: 24 },
                structure_density_kg_m3: 4_400.0,
                minimum_temperature_k: 80.0,
                maximum_temperature_k: 500.0,
            },
            radius_m: 0.32,
            width_m: 0.18,
            mass_kg: 3.4,
            spin_inertia_kg_m2: 0.11,
            radial_stiffness_n_m: 42_000.0,
            radial_damping_n_s_m: 1_100.0,
            longitudinal_slip_stiffness_n_per_mps: 3_000.0,
            lateral_slip_stiffness_n_per_mps: 2_400.0,
            maximum_deflection_m: 0.075,
            maximum_load_n: 2_400.0,
            surface_friction: 0.85,
        },
        strut: WheelStrutSpec {
            extended_length_m: 0.42,
            stroke_m: 0.16,
            spring_rate_n_m: 31_000.0,
            damping_n_s_m: 2_800.0,
            preload_n: 80.0,
            minimum_force_n: 0.0,
            maximum_force_n: 9_000.0,
            mass_per_wheel_kg: 0.8,
        },
        brake: WheelBrakeSpec {
            maximum_torque_nm: 95.0,
            response_time_s: 0.12,
            mass_per_wheel_kg: 0.4,
        },
        drive: Some(WheelDriveSpec {
            motor: ElectricMotorSpec {
                rated_power_w: 20_000.0,
                peak_torque_nm: 250.0,
                maximum_rpm: 5_000.0,
                efficiency: 0.92,
                cooling_capacity_w: 4_000.0,
                dry_mass_kg: 25.0,
            },
            stall_copper_loss_w: 350.0,
            rotor_inertia_kg_m2: 0.04,
            final_drive_ratio: 5.0,
            drivetrain_efficiency: 0.9,
            driven_wheel_count: 1,
        }),
        retraction: None,
    };
    let base_mass_kg = flight.vehicle.mass_properties.mass_kg;
    let uncentered = spec.clone().compile().unwrap();
    let total_mass_kg = base_mass_kg + uncentered.mass_properties.mass_kg;
    let assembly_com = uncentered.mass_properties.center_of_mass_body_m
        * (uncentered.mass_properties.mass_kg / total_mass_kg);
    flight.vehicle.wheel_chassis = vec![uncentered];
    flight.vehicle.bake_wheel_chassis_masses().unwrap();

    let shift = -assembly_com;
    for panel in &mut flight.vehicle.aero_geometry.panels {
        panel.position_body_m += shift;
        panel.center_of_pressure_body_m += shift;
    }
    for part in &mut flight.vehicle.collision_geometry.parts {
        part.local_position_m += shift;
    }
    for chassis in &mut flight.vehicle.wheel_chassis {
        chassis.spec.mount_position_body_m += shift;
        *chassis = chassis.spec.clone().compile().unwrap();
    }
    let parallel_axis = total_mass_kg
        * (DMat3::IDENTITY * assembly_com.length_squared()
            - DMat3::from_cols(
                assembly_com * assembly_com.x,
                assembly_com * assembly_com.y,
                assembly_com * assembly_com.z,
            ));
    flight.vehicle.mass_properties.inertia_body_kg_m2 -= parallel_axis;
    flight.vehicle.mass_properties = RigidBodyProperties::new(
        total_mass_kg,
        flight.vehicle.mass_properties.inertia_body_kg_m2,
    )
    .unwrap();
    flight.aero_panels = PanelSoA::from_geometry(&flight.vehicle.aero_geometry).unwrap();
    flight.control_reference_geometry = flight.vehicle.aero_geometry.clone();
    flight.vehicle.validate().unwrap();
}

#[test]
fn flight_authority_advances_and_retracts_aircraft_wheel_chassis() {
    let (_, mut flight) = fixture();
    install_airless_drive(&mut flight);
    let mut spec = flight.vehicle.wheel_chassis[0].spec.clone();
    spec.retraction = Some(WheelChassisRetractionSpec {
        pivot_position_body_m: spec.mount_position_body_m,
        hinge_axis_body: DVec3::Y,
        stowed_angle_rad: -std::f64::consts::FRAC_PI_2,
        deployed_angle_rad: 0.0,
        initially_deployed: false,
        deployment_rate_rad_s: 1.0,
        actuator_max_torque_nm: 10_000.0,
    });
    flight.vehicle.wheel_chassis[0] = spec.compile().expect("aircraft gear compiles");
    flight.sync_wheel_gear_runtime_state();

    flight.set_gear_down(true);
    flight.advance_freeflight_landing_gear().unwrap();
    let deployed_fraction = flight.wheel_chassis_states()[0].deployment_fraction;
    assert!(deployed_fraction > 0.0 && deployed_fraction < 1.0);
    assert!(flight.landing_gear_transitioning());
    assert_eq!(flight.wheel_gear_actuator_telemetry().len(), 1);

    flight
        .set_wheel_chassis_deployed("authority-regolith-wheel", false)
        .expect("named wheel-gear retract command");
    assert!(
        flight.gear_down,
        "part command leaves the group target intact"
    );
    flight.advance_freeflight_landing_gear().unwrap();
    assert!(flight.wheel_chassis_states()[0].deployment_fraction < deployed_fraction);
    assert!(!flight.landing_gear_transitioning());
    assert_eq!(
        flight.wheel_gear_actuator_telemetry()[0].1.target_fraction,
        0.0
    );
}

#[test]
fn authority_uses_baked_body_atmosphere() {
    let (ephemeris, flight) = fixture();
    let body = ephemeris.body(flight.reference_body).unwrap();
    let baked = body.atmosphere.as_ref().expect("baked atmosphere");
    assert_eq!(flight.atmosphere.sea_level_pressure_pa, 120_000.0);
    assert_eq!(
        flight.atmosphere.gas_constant_j_kg_k,
        baked.gas_constant_j_kg_k
    );
    assert_eq!(
        flight.atmosphere.heat_capacity_ratio,
        baked.heat_capacity_ratio
    );
    assert_eq!(
        flight.atmosphere.sutherland_reference_viscosity_pa_s,
        baked.sutherland_reference_viscosity_pa_s
    );
    assert_eq!(
        flight.atmosphere.body_rotation_rad_s.z,
        std::f64::consts::TAU / (80.0 * 3_600.0)
    );
}

#[test]
fn typed_guidance_executes_through_the_legacy_authority_adapter() {
    let (ephemeris, mut flight) = fixture();
    let intent = GuidanceIntent::Attitude {
        target_body_to_inertial: DQuat::from_rotation_y(0.1),
        roll_policy: thessa_flight_control::RollPolicy::Hold,
    };
    flight
        .advance_guidance(
            &ephemeris,
            &intent,
            PropulsionDemand::new(0.0).unwrap(),
            FLIGHT_STEP_S,
        )
        .unwrap();
    assert_eq!(flight.sas_target_orientation, DQuat::from_rotation_y(0.1));
    assert!(flight.state.position_inertial_m.is_finite());
    assert!(flight.last_forces.is_some());
}

#[test]
fn typed_propulsion_uses_the_post_allocator_actuator_response() {
    let (ephemeris, mut flight) = fixture();
    let intent = GuidanceIntent::AngularRate {
        rate_body_rps: DVec3::ZERO,
    };
    flight
        .advance_guidance(
            &ephemeris,
            &intent,
            PropulsionDemand::new(0.0).unwrap(),
            FLIGHT_STEP_S,
        )
        .unwrap();
    assert_eq!(flight.throttle, 0.0);
    assert!(flight.thrust_n() > 0.0);
    assert!(flight.thrust_n() < 254_000.0);

    for _ in 0..32 {
        flight
            .advance_guidance(
                &ephemeris,
                &intent,
                PropulsionDemand::new(1.0).unwrap(),
                FLIGHT_STEP_S,
            )
            .unwrap();
    }
    assert_eq!(flight.throttle, 1.0);
    assert!(flight.thrust_n() > 200_000.0);
}

#[test]
fn direction_guidance_resolves_inertial_surface_orbit_and_flight_path_frames() {
    let (ephemeris, mut flight) = fixture();
    let body = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    flight.state.position_inertial_m = body.position_inertial + DVec3::Z * 1.0e7;
    flight.state.velocity_inertial_mps = body.velocity_inertial + DVec3::X;

    flight
        .apply_guidance_intent(
            &ephemeris,
            &GuidanceIntent::VelocityDirection {
                direction: DirectionTarget::new(DVec3::Y, DirectionFrame::Inertial).unwrap(),
                roll_policy: RollPolicy::Hold,
            },
        )
        .unwrap();
    assert!((flight.sas_target_orientation * DVec3::X).dot(DVec3::Y) > 1.0 - 1.0e-12);

    let surface = DirectionTarget::new(DVec3::new(0.0, 1.0, 0.0), DirectionFrame::Surface).unwrap();
    flight
        .apply_guidance_intent(
            &ephemeris,
            &GuidanceIntent::VelocityDirection {
                direction: surface,
                roll_policy: RollPolicy::Fixed,
            },
        )
        .unwrap();
    let up = normalize_direction(
        flight.state.position_inertial_m - body.position_inertial,
        "test surface up",
    )
    .unwrap();
    let (_east, north) = surface_tangent_basis(up).unwrap();
    assert!((flight.sas_target_orientation * DVec3::X).dot(north) > 1.0 - 1.0e-12);
    assert!((flight.sas_target_orientation * DVec3::Z).dot(up) > 1.0 - 1.0e-12);

    flight
        .apply_guidance_intent(
            &ephemeris,
            &GuidanceIntent::VelocityDirection {
                direction: DirectionTarget::new(DVec3::X, DirectionFrame::Orbit).unwrap(),
                roll_policy: RollPolicy::Hold,
            },
        )
        .unwrap();
    let prograde = normalize_direction(
        flight.state.velocity_inertial_mps - body.velocity_inertial,
        "test prograde",
    )
    .unwrap();
    assert!((flight.sas_target_orientation * DVec3::X).dot(prograde) > 1.0 - 1.0e-12);

    flight
        .apply_guidance_intent(
            &ephemeris,
            &GuidanceIntent::FlightPath {
                target: thessa_flight_control::FlightPathTarget {
                    direction: DirectionTarget::new(DVec3::Z, DirectionFrame::Surface).unwrap(),
                    roll_policy: RollPolicy::Fixed,
                },
            },
        )
        .unwrap();
    assert!((flight.sas_target_orientation * DVec3::X).dot(up) > 1.0 - 1.0e-12);
    assert!(flight.sas_target_orientation.is_finite());
}

#[test]
fn state_dependent_guidance_is_partition_invariant_and_does_not_ride_rails() {
    let (ephemeris, mut chunked) = fixture();
    let (_, mut per_tick) = fixture();
    let body = ephemeris
        .body_state(chunked.reference_body, SimTime::EPOCH)
        .unwrap();
    let position = body.position_inertial + DVec3::Z * 1.0e7;
    let velocity = body.velocity_inertial + DVec3::X * 1_000.0;
    for flight in [&mut chunked, &mut per_tick] {
        flight.state.position_inertial_m = position;
        flight.state.velocity_inertial_mps = velocity;
        flight.relative_position_m = position - body.position_inertial;
        flight.set_legacy_propulsion(0.0, false);
    }
    let intent = GuidanceIntent::VelocityDirection {
        direction: DirectionTarget::new(DVec3::X, DirectionFrame::Orbit).unwrap(),
        roll_policy: RollPolicy::Hold,
    };
    let propulsion = PropulsionDemand::new(0.0).unwrap();
    let ticks = 12;
    chunked
        .advance_guidance(
            &ephemeris,
            &intent,
            propulsion,
            FLIGHT_STEP_S * f64::from(ticks),
        )
        .unwrap();
    for _ in 0..ticks {
        per_tick
            .advance_guidance(&ephemeris, &intent, propulsion, FLIGHT_STEP_S)
            .unwrap();
    }
    assert_eq!(chunked.flight_time_s, per_tick.flight_time_s);
    assert!(
        (chunked.state.position_inertial_m - per_tick.state.position_inertial_m).length() < 1e-9
    );
    assert!(
        (chunked.state.velocity_inertial_mps - per_tick.state.velocity_inertial_mps).length()
            < 1e-9
    );
    assert!(
        (chunked.sas_target_orientation * DVec3::X).dot(per_tick.sas_target_orientation * DVec3::X)
            > 1.0 - 1.0e-12
    );
    assert_eq!(chunked.rails_advanced_this_frame, 0.0);
}

#[test]
fn target_frame_guidance_fails_until_a_target_body_is_resolved() {
    let (ephemeris, mut flight) = fixture();
    let result = flight.apply_guidance_intent(
        &ephemeris,
        &GuidanceIntent::VelocityDirection {
            direction: DirectionTarget::new(DVec3::X, DirectionFrame::Target).unwrap(),
            roll_policy: RollPolicy::Hold,
        },
    );
    assert!(
        matches!(result, Err(FlightError::InvalidInput(message)) if message.contains("target body"))
    );
}

#[test]
fn target_frame_guidance_uses_the_target_body_orientation() {
    let (ephemeris, mut flight) = fixture();
    let target_body = ephemeris.body_id("nereid").unwrap();
    let target_state = ephemeris
        .body_state(target_body, SimTime(flight.flight_time_s))
        .unwrap();
    let direction = DirectionTarget::for_target(DVec3::X, target_body.0).unwrap();
    flight
        .apply_guidance_intent(
            &ephemeris,
            &GuidanceIntent::VelocityDirection {
                direction,
                roll_policy: RollPolicy::Hold,
            },
        )
        .unwrap();
    let forward = flight.sas_target_orientation * DVec3::X;
    assert!(forward.dot(target_state.orientation * DVec3::X) > 1.0 - 1.0e-12);
}

#[test]
fn explicit_control_demand_uses_the_native_moment_allocator() {
    let (ephemeris, mut flight) = fixture();
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.rcs_enabled = true;
    let before = flight.state.angular_velocity_body_rps;
    flight
        .advance_control_demand_with_budget(
            &ephemeris,
            ControlDemand {
                force_body_n: DVec3::ZERO,
                moment_body_nm: DVec3::X * 100.0,
                propulsion: PropulsionDemand::new(0.0).unwrap(),
            },
            FLIGHT_STEP_S,
            None,
        )
        .expect("explicit moment demand");
    assert!(flight.state.angular_velocity_body_rps.x > before.x);
    assert!(flight.last_forces.is_some());
    assert!(flight.explicit_moment_demand_nm.is_none());
}

#[test]
fn solver_guard_reads_dominant_frame_not_launch_frame() {
    // Live failure: a Nereid escape at 33 km/s Nereid-relative latched
    // FLIGHT STOPPED because the guard measured 50+ km/s against the
    // LAUNCH body. Same state must pass with the guard in the dominant
    // frame. v_mag sits strictly between the bound and bound+R so the
    // old launch-frame check would trip while the dominant check clears.
    let (ephemeris, mut flight) = fixture();
    let thessa = ephemeris.body_id("thessa").unwrap();
    let nereid = ephemeris.body_id("nereid").unwrap();
    let time = SimTime::EPOCH;
    let thessa_state = ephemeris.body_state(thessa, time).unwrap();
    let nereid_state = ephemeris.body_state(nereid, time).unwrap();
    let giant = ephemeris.body(nereid).unwrap();
    let frame_gap = (thessa_state.velocity_inertial - nereid_state.velocity_inertial).length();
    assert!(frame_gap > 1000.0, "frames must differ, gap {frame_gap}");
    let v_mag = MAX_PILOT_RELATIVE_SPEED_MPS + frame_gap / 2.0;
    let away =
        (thessa_state.velocity_inertial - nereid_state.velocity_inertial).normalize_or_zero();
    flight.state.position_inertial_m =
        nereid_state.position_inertial + DVec3::Z * (giant.radius_m + 100_000.0);
    flight.state.velocity_inertial_mps = thessa_state.velocity_inertial - away * v_mag;
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.control_input = DVec3::ZERO;
    flight.flight_time_s = 0.0;
    assert_eq!(
        ephemeris.dominant_body(flight.state.position_inertial_m, time),
        Some(nereid)
    );
    let launch_relative =
        (flight.state.velocity_inertial_mps - thessa_state.velocity_inertial).length();
    assert!(
        launch_relative > MAX_PILOT_RELATIVE_SPEED_MPS,
        "setup must trip the old guard, got {launch_relative}"
    );
    flight
        .step(
            &ephemeris,
            &GravityField::from_ephemeris(&ephemeris),
            ControlMode::Direct,
        )
        .expect("dominant-frame guard must clear interlunar coast");
}

#[test]
fn warp_budget_keeps_exact_steps_and_does_not_queue_unserved_warp() {
    let (ephemeris, mut limited) = fixture();
    let (_, mut exact) = fixture();
    limited
        .advance_with_budget(
            &ephemeris,
            ControlMode::Navball,
            3.2,
            Some(std::time::Duration::ZERO),
        )
        .unwrap();
    assert_eq!(limited.steps_this_frame, 1);
    assert!(limited.accumulator_s < FLIGHT_STEP_S);
    exact
        .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
        .unwrap();
    assert_eq!(limited.state, exact.state);
    assert_eq!(limited.flight_time_s, exact.flight_time_s);
    limited
        .advance(&ephemeris, ControlMode::Navball, 0.0)
        .unwrap();
    assert_eq!(limited.steps_this_frame, 0);
}

#[test]
fn cold_vacuum_bake_yields_without_replaying_high_warp_ticks() {
    let (ephemeris, mut flight) = fixture();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
    flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.sas_enabled = false;
    flight.rcs_enabled = false;
    flight.control_input = DVec3::ZERO;
    flight.bake = Box::new(NeverReadyBakeQueue { pending: false });

    flight
        .advance_with_budget(
            &ephemeris,
            ControlMode::Direct,
            3_600.0,
            Some(std::time::Duration::from_millis(1)),
        )
        .unwrap();

    assert_eq!(flight.steps_this_frame, 0);
    assert_eq!(flight.rails_advanced_this_frame, 0.0);
    assert_eq!(flight.flight_time_s, 0.0);
    assert!(flight.waiting_for_rails_bake);
    assert!(flight.bake.has_pending());
    assert!(flight.accumulator_s < FLIGHT_STEP_S);
}

#[test]
fn controlled_vacuum_coast_does_not_wait_for_unrelated_bake() {
    let (ephemeris, mut flight) = fixture();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
    flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.control_input = DVec3::Y;
    flight.bake = Box::new(NeverReadyBakeQueue { pending: false });

    flight
        .advance_with_budget(
            &ephemeris,
            ControlMode::Navball,
            3.2,
            Some(std::time::Duration::ZERO),
        )
        .unwrap();

    assert!(flight.steps_this_frame > 0);
    assert!(!flight.waiting_for_rails_bake);
    assert!(flight.flight_time_s >= FLIGHT_STEP_S);
}

#[test]
#[ignore = "wall-clock diagnostic; run with --ignored --nocapture"]
fn profile_warp_frame_budget() {
    let (ephemeris, mut full) = fixture();
    let (_, mut bounded) = fixture();
    let now = std::time::Instant::now();
    full.advance(&ephemeris, ControlMode::Navball, 3.2).unwrap();
    eprintln!(
        "warp unbounded: {:?}, {} steps",
        now.elapsed(),
        full.steps_this_frame
    );
    let now = std::time::Instant::now();
    bounded
        .advance_with_budget(
            &ephemeris,
            ControlMode::Navball,
            3.2,
            Some(std::time::Duration::from_millis(8)),
        )
        .unwrap();
    eprintln!(
        "warp bounded: {:?}, {} steps",
        now.elapsed(),
        bounded.steps_this_frame
    );
    assert!(bounded.steps_this_frame < full.steps_this_frame);
}

#[test]
fn contact_mode_falls_without_rails_batches() {
    let (ephemeris, mut flight) = fixture();
    flight.set_legacy_propulsion(0.0, false);
    // Hover start: kill the 180 m/s launch airspeed so gravity is the
    // only load and the craft must fall straight down.
    let home = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    flight.state.velocity_inertial_mps = home.velocity_inertial;
    flight
        .enable_contact_mode(1_000.0, 2_000.0)
        .expect("contact mode arms");
    let initial_altitude = flight.relative_position_m.length() - flight.planet_radius_m;
    flight
        .advance(&ephemeris, ControlMode::Direct, 60.0 * FLIGHT_STEP_S)
        .expect("contact-active advance");
    assert!(flight.contact_active(), "craft must stay contact-active");
    assert_eq!(
        flight.rails_advanced_this_frame, 0.0,
        "no rails batch may span contact-active ticks"
    );
    assert!(flight.last_forces.is_some());
    assert!(flight.flight_error.is_none());
    let body = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    let altitude = (flight.state.position_inertial_m - body.position_inertial).length()
        - flight.planet_radius_m;
    assert!(
        altitude < initial_altitude - 0.5,
        "contact-active craft must fall, was {initial_altitude:.2} m now {altitude:.2} m"
    );
    let snapshot = flight.contact_snapshot().expect("contact telemetry");
    assert_eq!(snapshot.dynamic_bodies.len(), 1);
}

#[test]
fn powered_authority_tick_wires_airless_wheel_state_and_drive() {
    let (ephemeris, mut flight) = fixture();
    install_airless_drive(&mut flight);
    flight.atmosphere.sea_level_pressure_pa = 1.0e-12;
    assert_eq!(
        flight.atmosphere.sample(0.0).unwrap().density_kg_m3,
        0.0,
        "the wheel-drive regression runs in the declared vacuum path"
    );
    let home = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let up = DVec3::X;
    let (east, north) = surface_tangent_basis(up).unwrap();
    let relative = up * (flight.planet_radius_m + 1.85);
    flight.state = RigidBodyState::new(
        home.position_inertial + relative,
        home.velocity_inertial - up * 0.4,
        DQuat::from_mat3(&DMat3::from_cols(east, north, up)),
        DVec3::ZERO,
    )
    .unwrap();
    flight.relative_position_m = relative;
    flight.set_legacy_propulsion(0.0, false);
    flight.set_wheel_drive_command(1.0).unwrap();
    flight
        .enable_contact_mode(20.0, 40.0)
        .expect("contact mode arms");

    let mut observed_wheel_contact = false;
    for _ in 0..240 {
        flight
            .advance(&ephemeris, ControlMode::Direct, FLIGHT_STEP_S)
            .expect("powered contact tick");
        assert!(flight.flight_error.is_none(), "{:?}", flight.flight_error);
        observed_wheel_contact |= !flight.wheel_contact_telemetry().is_empty();
        if observed_wheel_contact && flight.wheel_drive_telemetry().len() == 1 {
            break;
        }
    }
    assert!(flight.contact_active());
    assert!(observed_wheel_contact, "airless wheel must query terrain");
    assert_eq!(flight.wheel_spin_rates_rad_s().len(), 1);
    assert_eq!(flight.wheel_spin_rates_rad_s()[0].len(), 1);
    assert_eq!(flight.wheel_drive_telemetry().len(), 1);
    assert!(
        flight.wheel_drive_telemetry()[0]
            .2
            .requested_wheel_torque_per_driven_wheel_nm
            > 0.0
    );
}

#[test]
fn contact_mode_lands_on_the_kinematic_terrain_patch() {
    let (ephemeris, mut flight) = fixture();
    let recipe: thessa_worldgen_rocky::spec_recipe::SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = std::sync::Arc::new(
        thessa_worldgen_rocky::field::field_from_manifest(
            &thessa_worldgen_rocky::spec_recipe::manifest_from_spec(&recipe).unwrap(),
        )
        .unwrap(),
    );
    let dir = [1.0, 0.0, 0.0];
    flight.initialize_world_site(field.clone(), dir, &ephemeris);
    // Belly-down hover 2 m above the sampled surface: body +Z (up) onto
    // the radial, gentle 1 m/s descent, no spin, engine off.
    let home = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let up = DVec3::X;
    let surface = flight.planet_radius_m + field.height_m(dir, 32.0).max(0.0);
    let (east, north) = surface_tangent_basis(up).unwrap();
    flight.state = RigidBodyState::new(
        home.position_inertial + up * (surface + 2.0),
        home.velocity_inertial - up * 1.0,
        DQuat::from_mat3(&DMat3::from_cols(east, north, up)),
        DVec3::ZERO,
    )
    .unwrap();
    flight.relative_position_m = up * (surface + 2.0);
    flight.set_legacy_propulsion(0.0, false);
    flight
        .enable_contact_mode(100.0, 200.0)
        .expect("contact mode arms");
    flight
        .advance(&ephemeris, ControlMode::Direct, 480.0 * FLIGHT_STEP_S)
        .expect("contact-active landing");
    assert!(flight.flight_error.is_none(), "{:?}", flight.flight_error);
    assert!(flight.contact_active(), "craft must stay contact-active");
    let home = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    let relative = flight.state.position_inertial_m - home.position_inertial;
    let ground = ground_dir_body_fixed(
        relative,
        flight.flight_time_s,
        flight.body_rotation_period_s,
    );
    let clearance = relative.length()
        - (flight.planet_radius_m + field.height_m(ground.to_array(), 32.0).max(0.0));
    let radial_speed =
        (flight.state.velocity_inertial_mps - home.velocity_inertial).dot(relative.normalize());
    assert!(
        (0.2..=1.2).contains(&clearance),
        "landed clearance {clearance:.3} m must match the fuselage keel envelope"
    );
    assert!(
        radial_speed.abs() < 0.3,
        "landed craft must rest, radial speed {radial_speed:.3} m/s"
    );
    let snapshot = flight.contact_snapshot().expect("contact telemetry");
    assert!(
        snapshot.touching_contact_pairs >= 1,
        "landed craft must report a touching pair: {snapshot:?}"
    );
}

#[test]
fn terrain_patch_stays_body_fixed_under_horizontal_craft_motion() {
    let (ephemeris, mut flight) = fixture();
    flight.body_rotation_period_s = 1.0e30;
    flight.atmosphere.body_rotation_rad_s = DVec3::ZERO;
    let home = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let up = DVec3::X;
    let (east, north) = surface_tangent_basis(up).unwrap();
    let relative = up * (flight.planet_radius_m + 1.85);
    flight.state = RigidBodyState::new(
        home.position_inertial + relative,
        home.velocity_inertial + DVec3::Y * 10.0,
        DQuat::from_mat3(&DMat3::from_cols(east, north, up)),
        DVec3::ZERO,
    )
    .unwrap();
    flight.relative_position_m = relative;
    flight.set_legacy_propulsion(0.0, false);
    flight
        .enable_contact_mode(100.0, 200.0)
        .expect("contact mode arms");

    flight
        .advance(&ephemeris, ControlMode::Direct, 2.0 * FLIGHT_STEP_S)
        .expect("two contact-active ticks");
    let snapshot = flight.contact_snapshot().expect("contact telemetry");
    let ground_velocity = DVec3::from_array(snapshot.kinematic_bodies[0].velocity_inertial_mps);
    let body_now = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    let body_previous = ephemeris
        .body_state(
            flight.reference_body,
            SimTime(flight.flight_time_s - FLIGHT_STEP_S),
        )
        .unwrap();
    let body_velocity =
        (body_now.position_inertial - body_previous.position_inertial) / FLIGHT_STEP_S;
    let ground_velocity_relative_to_body = ground_velocity - body_velocity;
    assert!(
        ground_velocity_relative_to_body.length() < 1.0e-3,
        "craft translation must not drag nonrotating terrain: {ground_velocity_relative_to_body:?}"
    );
}

#[test]
fn terrain_patch_carries_rotating_body_surface_velocity() {
    let (ephemeris, mut flight) = fixture();
    let period_s = 86_400.0;
    let angular_velocity = DVec3::Z * (std::f64::consts::TAU / period_s);
    flight.body_rotation_period_s = period_s;
    flight.atmosphere.body_rotation_rad_s = angular_velocity;
    let home = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let up = DVec3::X;
    let (east, north) = surface_tangent_basis(up).unwrap();
    let relative = up * (flight.planet_radius_m + 1.85);
    flight.state = RigidBodyState::new(
        home.position_inertial + relative,
        home.velocity_inertial + angular_velocity.cross(relative),
        DQuat::from_mat3(&DMat3::from_cols(east, north, up)),
        DVec3::ZERO,
    )
    .unwrap();
    flight.relative_position_m = relative;
    flight.set_legacy_propulsion(0.0, false);
    flight
        .enable_contact_mode(100.0, 200.0)
        .expect("contact mode arms");

    flight
        .advance(&ephemeris, ControlMode::Direct, FLIGHT_STEP_S)
        .expect("rotating-body contact tick");
    let snapshot = flight.contact_snapshot().expect("contact telemetry");
    let ground_state = snapshot.kinematic_bodies[0];
    let ground_velocity = DVec3::from_array(ground_state.velocity_inertial_mps);
    let patch_offset = DVec3::from_array(ground_state.position_inertial_m) - home.position_inertial;
    let expected_velocity = home.velocity_inertial + angular_velocity.cross(patch_offset);
    assert!(
        (ground_velocity - expected_velocity).length() < 0.1,
        "kinematic ground velocity {ground_velocity:?} should follow omega x r = {expected_velocity:?}"
    );
}

#[test]
fn terrain_contact_stops_before_committing_an_underground_pose() {
    let (ephemeris, mut flight) = fixture();
    let recipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
    let field = std::sync::Arc::new(
        thessa_worldgen_rocky::field::field_from_manifest(
            &thessa_worldgen_rocky::spec_recipe::manifest_from_spec(&recipe).unwrap(),
        )
        .unwrap(),
    );
    let dir = [1.0, 0.0, 0.0];
    flight.initialize_world_site(field.clone(), dir, &ephemeris);
    let body = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let r = flight.planet_radius_m + field.height_m(dir, 32.0).max(0.0);
    flight.state.position_inertial_m = body.position_inertial + DVec3::X * (r + 0.1);
    flight.state.velocity_inertial_mps = body.velocity_inertial - DVec3::X * 100.0;
    let before = flight.state;
    let error = flight
        .step(
            &ephemeris,
            &GravityField::from_ephemeris(&ephemeris),
            ControlMode::Direct,
        )
        .unwrap_err();
    assert!(error.to_string().contains("terrain impact"), "{error}");
    assert_eq!(flight.state.position_inertial_m, before.position_inertial_m);
    assert_eq!(flight.flight_time_s, 0.0);
}

#[test]
fn physical_control_actuator_advances_in_vacuum_without_aero_load() {
    let (_, mut flight) = fixture();
    let actuator = ControlSurfaceActuator {
        max_rate_rad_s: 0.6,
        max_torque_nm: 1_000.0,
    };
    flight.vehicle.control_surfaces[0].hinge =
        Some(ControlHinge::new(DVec3::ZERO, DVec3::Y).expect("valid test hinge"));
    flight.vehicle.control_surfaces[0].actuator = Some(actuator);
    flight.control_reference_geometry = flight.vehicle.aero_geometry.clone();
    flight.regime = FlightRegime::Coast;
    flight.rcs_enabled = false;
    flight.command_controls(1.0, 0.0, 0.0);

    let kinematics = LocalAirKinematics {
        relative_position_inertial_m: DVec3::ZERO,
        relative_position_body_m: DVec3::ZERO,
        relative_velocity_inertial_mps: DVec3::ZERO,
        air_velocity_body_mps: DVec3::ZERO,
        surface_velocity_inertial_mps: DVec3::ZERO,
        radial_up: DVec3::Z,
        altitude_m: 100_000.0,
    };
    let moment = flight
        .allocate_controls(kinematics, ControlMode::Direct)
        .expect("coast control allocation")
        .moment_body_nm;

    assert_eq!(moment, DVec3::ZERO);
    let expected_angle = -actuator.max_rate_rad_s * FLIGHT_STEP_S;
    assert!((flight.control_deflections_rad[0] - expected_angle).abs() < 1.0e-12);
    assert!(flight.actuator_saturated);
    let controlled_panel = flight.vehicle.control_surfaces[0].panel_indices[0];
    assert_ne!(
        flight.vehicle.aero_geometry.panels[controlled_panel].position_body_m,
        flight.control_reference_geometry.panels[controlled_panel].position_body_m
    );

    flight
        .reset_control_surfaces()
        .expect("reset actual actuator state to neutral");
    assert!(
        flight
            .control_deflections_rad
            .iter()
            .all(|angle| angle.abs() < 1.0e-12)
    );
    assert_eq!(
        flight.vehicle.aero_geometry,
        flight.control_reference_geometry
    );
}

#[test]
fn reaction_wheels_keep_torque_authority_and_rcs_covers_excess_demand() {
    let (_, mut flight) = fixture();
    flight.vehicle.reaction_wheels = vec![ReactionWheelBankSpec {
        name: "test-wheel-box".into(),
        max_torque_body_nm: DVec3::splat(100.0),
        mass_kg: 10.0,
        position_body_m: DVec3::ZERO,
        inertia_body_kg_m2: DMat3::IDENTITY,
    }];
    flight.reaction_wheels_enabled = true;
    flight.rcs_enabled = false;

    let below_rating = flight
        .allocate_reaction_wheel_residual(DVec3::X * 60.0, DVec3::ZERO)
        .expect("wheel allocation");
    assert_eq!(below_rating, DVec3::X * 60.0);
    assert_eq!(flight.reaction_wheel_telemetry(), DVec3::X * 60.0);
    assert!(!flight.actuator_saturated);
    let (_, wheel_started_rotation) = integrate_attitude_step(
        DQuat::IDENTITY,
        DVec3::ZERO,
        flight.vehicle.mass_properties.inertia_body_kg_m2,
        below_rating,
        FLIGHT_STEP_S,
    )
    .expect("wheel moment integrates through the rigid-body attitude solver");
    assert!(wheel_started_rotation.x > 0.0);

    let above_rating = flight
        .allocate_reaction_wheel_residual(DVec3::X * 150.0, DVec3::ZERO)
        .expect("rated wheel allocation");
    assert_eq!(above_rating, DVec3::X * 100.0);
    assert!(flight.actuator_saturated);

    flight
        .set_reaction_wheel_bank_enabled("test-wheel-box", false)
        .expect("named bank disable");
    let disabled = flight
        .allocate_reaction_wheel_residual(DVec3::X * 60.0, DVec3::ZERO)
        .expect("disabled bank allocation");
    assert_eq!(disabled, DVec3::ZERO);
    assert!(flight.actuator_saturated);

    flight.rcs_enabled = true;
    let combined = flight
        .allocate_reaction_wheel_residual(DVec3::X * 500.0, DVec3::ZERO)
        .expect("wheel plus RCS allocation");
    assert!(
        (combined.x - 500.0).abs() < 1.0e-7,
        "combined torque: {combined:?}"
    );
    assert!(!flight.actuator_saturated);
}

#[test]
fn parachute_commands_target_one_named_vehicle_part() {
    let (_, mut flight) = fixture();
    let parachute = |name: &str| ParachuteSpec {
        name: name.into(),
        reference_area_m2: 20.0,
        drag_coefficient: 1.5,
        reefed_area_fraction: 0.1,
        inflation_time_s: 2.0,
        deploy_pressure_pa: 10_000.0,
        max_deploy_dynamic_pressure_pa: 2_000.0,
        max_canopy_load_n: 100_000.0,
        pack_mass_kg: 12.0,
        position_body_m: DVec3::ZERO,
        inertia_body_kg_m2: DMat3::IDENTITY,
    };
    flight.vehicle.parachutes = vec![parachute("drogue"), parachute("main")];

    flight
        .command_parachute("main", ParachuteCommand::Arm)
        .expect("named chute command");
    assert_eq!(flight.parachute_states[0].phase, ParachutePhase::Stowed);
    assert_eq!(flight.parachute_states[1].phase, ParachutePhase::Armed);
    assert!(
        flight
            .command_parachute("unknown", ParachuteCommand::Arm)
            .is_err()
    );
}

#[test]
fn shared_part_command_api_routes_installed_subsystem_controls() {
    let (_, mut flight) = fixture();
    flight.vehicle.reaction_wheels = vec![ReactionWheelBankSpec {
        name: "trim-bank".into(),
        max_torque_body_nm: DVec3::splat(20.0),
        mass_kg: 5.0,
        position_body_m: DVec3::ZERO,
        inertia_body_kg_m2: DMat3::IDENTITY,
    }];
    flight.vehicle.parachutes = vec![ParachuteSpec {
        name: "main".into(),
        reference_area_m2: 20.0,
        drag_coefficient: 1.5,
        reefed_area_fraction: 0.1,
        inflation_time_s: 2.0,
        deploy_pressure_pa: 10_000.0,
        max_deploy_dynamic_pressure_pa: 2_000.0,
        max_canopy_load_n: 100_000.0,
        pack_mass_kg: 12.0,
        position_body_m: DVec3::ZERO,
        inertia_body_kg_m2: DMat3::IDENTITY,
    }];

    for command in [
        VehiclePartCommand::SetRcsEnabled { enabled: false },
        VehiclePartCommand::SetReactionWheelsEnabled { enabled: true },
        VehiclePartCommand::SetReactionWheelBankEnabled {
            name: "trim-bank".into(),
            enabled: false,
        },
        VehiclePartCommand::SetLandingGearDeployed { deployed: false },
        VehiclePartCommand::Parachute {
            name: "main".into(),
            command: ParachuteCommand::Arm,
        },
    ] {
        flight
            .apply_part_command(&command)
            .expect("valid installed-part command");
    }

    assert!(!flight.rcs_enabled);
    assert!(flight.reaction_wheels_enabled);
    assert_eq!(flight.reaction_wheel_bank_enabled, vec![false]);
    assert!(!flight.gear_down);
    assert_eq!(flight.parachute_states[0].phase, ParachutePhase::Armed);
    assert!(
        flight
            .apply_part_command(&VehiclePartCommand::Parachute {
                name: "missing".into(),
                command: ParachuteCommand::Arm,
            })
            .is_err()
    );
}

#[test]
fn parachute_drag_enters_the_authoritative_rigid_body_force_step() {
    let (ephemeris, mut flight) = fixture();
    flight.vehicle.parachutes = vec![ParachuteSpec {
        name: "authority-main".into(),
        reference_area_m2: 24.0,
        drag_coefficient: 1.5,
        reefed_area_fraction: 0.1,
        inflation_time_s: 2.0,
        deploy_pressure_pa: 5_000.0,
        max_deploy_dynamic_pressure_pa: 2_000.0,
        max_canopy_load_n: 100_000.0,
        pack_mass_kg: 12.0,
        position_body_m: DVec3::new(-2.0, 0.0, 0.0),
        inertia_body_kg_m2: DMat3::IDENTITY,
    }];
    flight.set_parachutes_armed(true);
    flight.sas_enabled = false;
    flight.rcs_enabled = false;
    flight.reaction_wheels_enabled = false;
    flight.set_legacy_propulsion(0.0, false);
    let home = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    flight.relative_position_m = DVec3::Z * (flight.planet_radius_m + 1_000.0);
    flight.state.position_inertial_m = home.position_inertial + flight.relative_position_m;
    flight.state.velocity_inertial_mps = home.velocity_inertial - DVec3::Z * 40.0;

    let gravity = GravityField::from_ephemeris(&ephemeris);
    flight
        .step(&ephemeris, &gravity, ControlMode::Direct)
        .expect("atmospheric step with an armed parachute");

    let load = flight.parachute_telemetry()[0];
    assert_eq!(load.state.phase, ParachutePhase::Reefed);
    assert!(load.force_body_n.z > 0.0);
    let forces = flight.last_forces.as_ref().expect("step force sample");
    assert!(
        (forces.total_force_body_n - forces.aero.force_body_n - load.force_body_n).length()
            < 1.0e-8,
        "parachute load was not composed into the body force: {:?}",
        forces.total_force_body_n
    );
}

#[test]
fn surface_moments_follow_pilot_axes_without_rcs() {
    let (_, mut flight) = fixture();
    let kinematics = LocalAirKinematics {
        relative_position_inertial_m: DVec3::ZERO,
        relative_position_body_m: DVec3::ZERO,
        relative_velocity_inertial_mps: DVec3::X * 180.0,
        air_velocity_body_mps: DVec3::X * 180.0,
        surface_velocity_inertial_mps: DVec3::ZERO,
        radial_up: DVec3::Z,
        altitude_m: 500.0,
    };
    let env = flight
        .atmosphere
        .aero_environment(500.0, DVec3::ZERO)
        .unwrap();
    let flow = AeroState::new(DVec3::X * 180.0, DVec3::ZERO);
    let base = flight
        .aero_model
        .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
        .unwrap()
        .moment_body_nm;
    for command in [
        DVec3::X,
        DVec3::Y,
        DVec3::Z,
        -DVec3::X,
        -DVec3::Y,
        -DVec3::Z,
    ] {
        flight.command_controls(command.x, command.y, command.z);
        flight.surface_input = DVec3::ZERO;
        flight.control_deflections_rad.fill(0.0);
        flight
            .vehicle
            .apply_control_deflections(
                &flight.control_reference_geometry,
                &flight.control_deflections_rad,
            )
            .unwrap();
        for _ in 0..60 {
            flight
                .allocate_controls(kinematics, ControlMode::Direct)
                .unwrap();
        }
        let moment = flight
            .aero_model
            .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
            .unwrap()
            .moment_body_nm;
        assert!(
            (moment - base).dot(crate::control::body_axes(command)) > 0.0,
            "wrong surface sign for {command:?}"
        );
    }
    assert_eq!(
        crate::control::rcs_moment(DVec3::splat(1.0e9), false),
        DVec3::ZERO
    );
    assert_eq!(
        crate::control::rcs_moment(DVec3::splat(1.0e9), true),
        DVec3::new(1120.0, 4000.0, 4000.0)
    );
}

#[test]
fn recorded_high_rate_flight_continues_past_the_previous_stop() {
    // Live capture, 514.808--581.375 s: restore its first physical state,
    // then replay the pilot inputs (not the recorded resulting forces).
    let capture = include_str!("../../../logs/flight-traces/2026-09-09-high-rate-stop.csv");
    let mut lines = capture.lines();
    let columns: Vec<_> = lines.next().unwrap().split(',').collect();
    let column = |name: &str| columns.iter().position(|c| *c == name).unwrap();
    let first: Vec<_> = lines.next().unwrap().split(',').collect();
    let number = |row: &[&str], name: &str| row[column(name)].parse::<f64>().unwrap();
    let vector = |row: &[&str], names: [&str; 3]| {
        DVec3::new(
            number(row, names[0]),
            number(row, names[1]),
            number(row, names[2]),
        )
    };
    let (ephemeris, mut flight) = fixture();
    flight.flight_time_s = (number(&first, "t_s") * 120.0).round() / 120.0;
    flight.state = RigidBodyState::new(
        vector(&first, ["pos_x_m", "pos_y_m", "pos_z_m"]),
        vector(&first, ["vel_x_mps", "vel_y_mps", "vel_z_mps"]),
        DQuat::from_xyzw(
            number(&first, "quat_x"),
            number(&first, "quat_y"),
            number(&first, "quat_z"),
            number(&first, "quat_w"),
        )
        .normalize(),
        vector(&first, ["omega_x_rps", "omega_y_rps", "omega_z_rps"]),
    )
    .unwrap();
    flight.surface_input = vector(&first, ["surface_pitch", "surface_yaw", "surface_roll"]);
    let mut max_rate: f64 = 0.0;
    for line in lines {
        let row: Vec<_> = line.split(',').collect();
        assert_eq!(row[column("control_mode")], "DIRECT / RAW");
        flight.control_input = vector(&row, ["pitch_cmd", "yaw_cmd", "roll_cmd"]);
        flight.throttle = number(&row, "throttle");
        flight.engine_active = row[column("engine")] == "1";
        flight.sas_enabled = row[column("sas")] == "1";
        flight.rcs_enabled = row[column("rcs")] == "1";
        flight
            .advance(&ephemeris, ControlMode::Direct, FLIGHT_STEP_S)
            .unwrap();
        max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
    }
    flight.control_input = DVec3::ZERO;
    flight
        .advance(&ephemeris, ControlMode::Direct, 60.0)
        .unwrap();
    println!(
        "recorded spin replay: t={:.3} s, max rate={max_rate:.6}, final rate={:.6}",
        flight.flight_time_s,
        flight.state.angular_velocity_body_rps.length()
    );
    assert!(flight.flight_time_s > 640.0);
}

#[test]
fn production_flight_is_stable_for_fifteen_minutes_and_independent_of_render_cadence() {
    let (ephemeris, mut flight) = fixture();
    let (_, mut other) = fixture();
    let start = std::time::Instant::now();
    let mut min_alt = f64::INFINITY;
    let mut max_aoa: f64 = 0.0;
    let mut max_rate: f64 = 0.0;
    for frame in 0..45_000 {
        flight
            .advance(&ephemeris, ControlMode::Navball, 0.02)
            .unwrap();
        // Same 0.02 seconds split into two irregular render frames.
        other
            .advance(&ephemeris, ControlMode::Navball, 0.007)
            .unwrap();
        other
            .advance(&ephemeris, ControlMode::Navball, 0.013)
            .unwrap();
        let body = ephemeris
            .body_state(flight.reference_body, SimTime(flight.flight_time_s))
            .unwrap();
        let k = local_air_kinematics(
            flight.atmosphere,
            flight.state,
            body,
            flight.planet_radius_m,
        )
        .unwrap();
        min_alt = min_alt.min(k.altitude_m);
        max_aoa = max_aoa.max(conventional_angle_of_attack_deg(k.air_velocity_body_mps).abs());
        max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
        assert!(
            k.altitude_m > 100.0,
            "surface departure failed at frame {frame}: {}",
            k.altitude_m
        );
        assert!(
            max_aoa < 22.0,
            "uncommanded stall at frame {frame}: {max_aoa}"
        );
        assert!(max_rate < 0.35, "uncommanded spin: {max_rate}");
        assert!(
            (flight.state.position_inertial_m - other.state.position_inertial_m).length() < 1.0e-6
        );
        assert!(
            (flight.state.angular_velocity_body_rps - other.state.angular_velocity_body_rps)
                .length()
                < 1.0e-10
        );
    }
    println!(
        "900 s production path: min altitude {min_alt:.3} m, max AoA {max_aoa:.3} deg, max rate {max_rate:.6} rad/s; two runs {:?}",
        start.elapsed()
    );
}

#[test]
fn sas_allows_full_orbital_turn_and_holds_after_release() {
    for command in [DVec3::Y, DVec3::X] {
        let (ephemeris, mut flight) = fixture();
        let body = ephemeris.body(flight.reference_body).unwrap();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        let radius = body.radius_m + 300_000.0;
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
        flight.state.velocity_inertial_mps =
            origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.control_input = command;
        let axis = if command == DVec3::Y {
            DVec3::NEG_Z
        } else {
            DVec3::NEG_Y
        };
        let mut accumulated_angle = 0.0;
        for _ in 0..5400 {
            let previous = flight.state.orientation_body_to_inertial;
            flight
                .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
                .unwrap();
            let delta =
                (previous.inverse() * flight.state.orientation_body_to_inertial).to_scaled_axis();
            assert!(
                delta.dot(axis) >= -1.0e-8,
                "SAS reversed a commanded orbital turn"
            );
            accumulated_angle += delta.dot(axis);
        }
        assert!(
            accumulated_angle.to_degrees() > 360.0,
            "turn stopped at {} deg",
            accumulated_angle.to_degrees()
        );
        flight.control_input = DVec3::ZERO;
        let released = flight.state.orientation_body_to_inertial;
        flight
            .advance(&ephemeris, ControlMode::Navball, 20.0)
            .unwrap();
        let hold_error = flight
            .state
            .orientation_body_to_inertial
            .angle_between(released)
            .to_degrees();
        println!(
            "orbital turn: {:.2} deg, hold error {hold_error:.6} deg, rate {:?}",
            accumulated_angle.to_degrees(),
            flight.state.angular_velocity_body_rps
        );
        assert!(hold_error < 1.0, "SAS hold error {hold_error}");
        assert!(flight.state.angular_velocity_body_rps.length() < 0.001);
    }
}

#[test]
fn guidance_and_direct_modes_remain_finite_through_control_changes() {
    let (ephemeris, mut flight) = fixture();
    for mode in [
        ControlMode::Navball,
        ControlMode::MouseAim,
        ControlMode::Rate,
        ControlMode::Direct,
    ] {
        for frame in 0..1500 {
            flight.control_input = if (200..250).contains(&frame) {
                DVec3::new(0.4, 0.15, 0.2)
            } else {
                DVec3::ZERO
            };
            flight.advance(&ephemeris, mode, 0.02).unwrap();
            assert!(flight.state.velocity_inertial_mps.is_finite());
        }
    }
}
#[test]
fn bank_rotates_lift_out_of_the_vertical_plane() {
    let (_, flight) = fixture();
    let env = flight
        .atmosphere
        .aero_environment(500.0, DVec3::ZERO)
        .unwrap();
    let mut vertical = vec![];
    for bank in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0] {
        let attitude = DQuat::from_rotation_x(bank.to_radians())
            * DQuat::from_rotation_y(-8.0_f64.to_radians());
        let force = flight
            .aero_model
            .evaluate_state(
                AeroState::new(attitude.inverse() * DVec3::X * 180.0, DVec3::ZERO),
                env,
                &flight.vehicle.aero_geometry,
            )
            .unwrap()
            .force_body_n;
        let inertial = attitude * force;
        assert!(inertial.is_finite());
        vertical.push(inertial.z);
        eprintln!(
            "bank {bank:5.0} deg: vertical aero force {:10.3} N; side {:10.3} N",
            inertial.z, inertial.y
        );
    }
    assert!(vertical[0] > 100.0);
    for (i, angle) in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0]
        .into_iter()
        .enumerate()
    {
        assert!((vertical[i] / vertical[0] - angle.to_radians().cos()).abs() < 1.0e-10);
    }
}

fn circular_orbit_fixture(altitude_m: f64) -> (BakedEphemeris, FlightAuthority) {
    let (ephemeris, mut flight) = fixture();
    let body = ephemeris.body(flight.reference_body).unwrap();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let radius = body.radius_m + altitude_m;
    flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
    flight.state.velocity_inertial_mps =
        origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.control_input = DVec3::ZERO;
    (ephemeris, flight)
}

#[test]
fn circular_orbit_initializer_sets_body_relative_circular_state() {
    let (ephemeris, mut flight) = fixture();
    let altitude_m = 1_000_000.0;
    flight
        .initialize_circular_orbit(&ephemeris, altitude_m, DVec3::Y)
        .unwrap();
    let body = ephemeris.body(flight.reference_body).unwrap();
    let body_state = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let relative = flight.state.position_inertial_m - body_state.position_inertial;
    let relative_velocity = flight.state.velocity_inertial_mps - body_state.velocity_inertial;
    assert!((relative.length() - (body.radius_m + altitude_m)).abs() < 1.0e-6);
    assert!((relative_velocity.length() - (body.mu / relative.length()).sqrt()).abs() < 1.0e-9);
    assert!(relative.dot(relative_velocity).abs() < 1.0e-6);
    assert_eq!(flight.relative_position_m, relative);
    assert!(!flight.engine_active);
}

#[test]
fn coast_threshold_brackets_vacuum() {
    let (_, flight) = fixture();
    let sea_level = flight.atmosphere.sample(0.0).unwrap().density_kg_m3;
    assert!(
        sea_level > COAST_DENSITY_KG_M3,
        "launch pad must be Aero, got {sea_level}"
    );
    let high = flight.atmosphere.sample(300_000.0).unwrap().density_kg_m3;
    assert!(
        high < COAST_DENSITY_KG_M3,
        "300 km must be Coast, got {high}"
    );
    assert_eq!(FlightRegime::Aero.label(), "AERO");
    assert_eq!(FlightRegime::Coast.label(), "COAST");
}

#[test]
fn coast_freezes_trim_and_keeps_rcs_authority() {
    let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
    flight
        .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
        .unwrap();
    assert_eq!(flight.regime(), FlightRegime::Coast);
    // Assisted trim solve is frozen in vacuum: surfaces hold, no stall
    // chase from a near-singular effectiveness matrix.
    assert_eq!(flight.surface_input, DVec3::ZERO);
    // Full yaw stick still turns the craft on RCS alone.
    flight.control_input = DVec3::Y;
    flight
        .advance(&ephemeris, ControlMode::Navball, 2.0)
        .unwrap();
    assert_eq!(flight.regime(), FlightRegime::Coast);
    assert_eq!(flight.surface_input, DVec3::ZERO);
    assert!(
        flight.state.angular_velocity_body_rps.length() > 0.05,
        "RCS must answer in coast, got {:?}",
        flight.state.angular_velocity_body_rps
    );
}

#[test]
fn coast_orbit_stays_bounded_without_stops() {
    let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
    for _ in 0..30_000 {
        // Every step must succeed: leaving the atmosphere never latches
        // FLIGHT STOPPED, it switches the solver to Coast instead.
        flight
            .advance(&ephemeris, ControlMode::Navball, 0.02)
            .unwrap();
        assert_eq!(flight.regime(), FlightRegime::Coast);
    }
    assert!((flight.flight_time_s - 600.0).abs() < 1.0);
    let body = ephemeris.body(flight.reference_body).unwrap();
    let state = ephemeris
        .body_state(flight.reference_body, SimTime(flight.flight_time_s))
        .unwrap();
    let altitude =
        (flight.state.position_inertial_m - state.position_inertial).length() - body.radius_m;
    assert!(
        (280_000.0..320_000.0).contains(&altitude),
        "coast must hold the orbit, altitude drifted to {altitude}"
    );
}

/// Upper-band calibration: reference-area drag vs the full panel
/// sum across attitudes and band altitudes. The relative error is
/// large (attitude-independent drag cannot track panels), but the
/// absolute envelope is what matters: ~23 N worst case is 8e-11 m
/// per tick and dissipative, against 100 kN weight and 254 kN
/// thrust. Moments stay RCS-dominated by orders of magnitude.
#[test]
fn upper_band_drag_matches_full_panels_within_newtons() {
    use thessa_sim_core::AeroState;
    let (ephemeris, mut flight) = fixture();
    let body = ephemeris.body(flight.reference_body).unwrap();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    let mut worst_force = 0.0f64;
    let mut worst_moment = 0.0f64;
    for altitude_m in [280_000.0f64, 300_000.0] {
        let radius = body.radius_m + altitude_m;
        for pitch_deg in [0.0f64, 10.0, 30.0, 90.0] {
            flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
            flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 7000.0;
            flight.state.orientation_body_to_inertial =
                DQuat::from_rotation_y(pitch_deg.to_radians());
            flight.state.angular_velocity_body_rps = DVec3::ZERO;
            let time = SimTime(flight.flight_time_s);
            let home = ephemeris.body_state(flight.reference_body, time).unwrap();
            let kinematics = local_air_kinematics(
                flight.atmosphere,
                flight.state,
                home,
                flight.planet_radius_m,
            )
            .unwrap();
            let density = flight
                .atmosphere
                .sample(kinematics.altitude_m.max(0.0))
                .unwrap()
                .density_kg_m3;
            assert!(
                density > flight.atmosphere.vacuum_cutoff_density_kg_m3
                    && density < COAST_DENSITY_KG_M3,
                "case must sit in the band, got {density:.3e} at {altitude_m}"
            );
            let (band_drag, _) = flight
                .upper_band_drag(kinematics, density)
                .expect("band must engage");
            let environment = flight
                .atmosphere
                .aero_environment(kinematics.altitude_m.max(0.0), DVec3::ZERO)
                .unwrap();
            let aero_state = AeroState::new(
                kinematics.air_velocity_body_mps,
                flight.state.angular_velocity_body_rps,
            );
            let full = flight
                .aero_model
                .evaluate_state(aero_state, environment, &flight.vehicle.aero_geometry)
                .unwrap();
            worst_force = worst_force.max((band_drag - full.force_body_n).length());
            worst_moment = worst_moment.max(full.moment_body_nm.length());
        }
    }
    assert!(
        worst_force < 40.0,
        "band drag diverged {worst_force:.1} N from panels"
    );
    assert!(
        worst_moment < 25.0,
        "band drops {worst_moment:.1} N·m of panel moment"
    );
}

/// Batch unlock: with the declared vacuum, an unpowered drift above
/// the atmosphere top rides rails batches instead of integrating
/// every tick. The altitude is found programmatically (first exact
/// zero density), so the test tracks the model, not a magic number.
#[test]
fn declared_vacuum_unlocks_batches_above_the_top() {
    let (_ephemeris, flight) = fixture();
    let probe_altitude_m = [300_000.0f64, 350_000.0, 400_000.0, 500_000.0, 800_000.0]
        .into_iter()
        .find(|altitude_m| {
            flight
                .atmosphere
                .sample(*altitude_m)
                .map(|sample| sample.density_kg_m3 == 0.0)
                .unwrap_or(false)
        })
        .expect("model must reach declared vacuum somewhere");
    let (ephemeris, mut flight) = circular_orbit_fixture(probe_altitude_m);
    flight.engine_active = false;
    flight.throttle = 0.0;
    let mut rails_total = 0.0;
    let mut steps_total = 0u64;
    for _ in 0..4 {
        flight
            .advance(&ephemeris, ControlMode::Navball, 120.0)
            .unwrap();
        rails_total += flight.rails_advanced_this_frame;
        // steps_this_frame is per advance call; sample before reset.
        steps_total += flight.steps_this_frame as u64;
    }
    assert!(
        rails_total > 0.0,
        "no batch rode at {probe_altitude_m:.0} m drift (steps={steps_total})"
    );
    eprintln!("drift at {probe_altitude_m:.0} m: rails_s={rails_total:.0} steps={steps_total}");
}

#[test]
fn warp_scale_batch_jump_covers_an_hour_without_ticks() {
    // 100kx warp equivalence: one advance call carrying an hour of sim
    // time must ride batch jumps, not 432k ticks. Attitude stays idle
    // (Direct, zero input/rates) so batches stay eligible.
    let (ephemeris, mut flight) = fixture();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
    flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.control_input = DVec3::ZERO;
    flight
        .advance(&ephemeris, ControlMode::Direct, 0.05)
        .expect("coast warms the rails");
    assert!(!flight.rails.is_empty());
    flight
        .advance(&ephemeris, ControlMode::Direct, 3600.0)
        .expect("warp hour advances");
    assert!(
        (flight.rails_advanced_this_frame - 3600.0).abs() < 1.0,
        "warp must ride batches, advanced {}",
        flight.rails_advanced_this_frame
    );
    assert_eq!(
        flight.steps_this_frame, 0,
        "no per-tick solver work at warp"
    );
    assert!(
        (flight.flight_time_s - 3600.05).abs() < 1.0,
        "clock must advance by the warped hour, got {}",
        flight.flight_time_s
    );
}

#[test]
fn unpowered_vacuum_coast_rides_the_shared_rails() {
    // One trajectory, two consumers: in an unpowered vacuum coast the
    // flight loop must sample translation from the baked rails (the same
    // bake the map prediction draws) instead of integrating per tick.
    // Deep space gives exactly-zero sampled density, which is the same
    // condition that permits the aero fast path.
    let (ephemeris, mut flight) = fixture();
    let origin = ephemeris
        .body_state(flight.reference_body, SimTime::EPOCH)
        .unwrap();
    flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
    flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
    flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
    flight.state.angular_velocity_body_rps = DVec3::ZERO;
    flight.sas_target_orientation = DQuat::IDENTITY;
    flight.engine_active = false;
    flight.throttle = 0.0;
    flight.control_input = DVec3::ZERO;
    flight
        .advance(&ephemeris, ControlMode::Direct, 0.05)
        .expect("coast advances");
    assert!(
        !flight.rails.is_empty(),
        "vacuum coast must bake the shared rails"
    );
    // The bake arms its wake in simulation time: the loop waits on the
    // horizon event instead of polling the trajectory.
    let wake = flight.scheduler.next().expect("coast arms a wake");
    assert!(
        matches!(wake.kind, thessa_sim_core::ScheduledKind::RailsHorizon),
        "deep-space coast wake must be the horizon, got {:?}",
        wake.kind
    );
    assert!(
        wake.time.seconds() > flight.flight_time_s,
        "wake must lie ahead"
    );
    // Translation now tracks the rails bake to interpolation precision.
    let (position, _) = flight.inertial_state_m();
    let (sampled, _) = flight
        .rails
        .sample_at(SimTime(flight.flight_time_s))
        .expect("rails cover the flown epoch");
    assert!(
        (position - sampled).length() < 1.0,
        "flown state left the rails: {:?} m",
        (position - sampled).length()
    );
    // A warm cache consumes an entire frame without solver ticks.
    flight.state.angular_velocity_body_rps = DVec3::X * 0.25;
    let before_orientation = flight.state.orientation_body_to_inertial;
    flight
        .advance(&ephemeris, ControlMode::Direct, 3.2)
        .unwrap();
    assert_eq!(flight.steps_this_frame, 0);
    assert!((flight.rails_advanced_this_frame - 3.2).abs() < 1e-10);
    let expected = before_orientation * DQuat::from_rotation_x(0.8);
    assert!(
        flight
            .state
            .orientation_body_to_inertial
            .abs_diff_eq(expected.normalize(), 1e-12)
    );
    // SAS needs to see rotation and must not be skipped for a whole batch.
    flight.sas_enabled = true;
    flight.rcs_enabled = true;
    let before = flight.state;
    let surface_input = flight.surface_input;
    assert_eq!(
        flight
            .try_advance_cached_coast(&ephemeris, ControlMode::Navball, 3.2)
            .unwrap(),
        CoastAdvance::NotEligible
    );
    assert_eq!(flight.state, before);
    assert_eq!(flight.surface_input, surface_input);
    // Lighting the engine invalidates the bake: thrust is a maneuver.
    flight.engine_active = true;
    flight.throttle = 1.0;
    flight
        .advance(&ephemeris, ControlMode::Direct, 0.05)
        .expect("powered flight advances");
    assert!(
        flight.rails_advanced_this_frame == 0.0 && flight.steps_this_frame > 0,
        "thrust must disable riding the gravity forecast"
    );
}
