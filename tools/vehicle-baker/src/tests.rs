use super::*;
use thessa_sim_core::{
    AtmosphereComposition, AtmosphereConfig, CompiledJet, CompiledShaftPowerSource,
    ElectricalPowerCommand, FeedResourceProperties, PropellerDriveCommand, RigidBodyState,
    SolarArrayTracking, SolarFluxSource, StoredPropellant, ThermalCommand, TurbopropCommand,
    flight_condition,
};

#[test]
fn example_vehicle_asset_bakes_to_valid_generic_definition() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_aircraft.toml"))
            .expect("vehicle TOML should parse");
    let vehicle = asset.bake().expect("vehicle asset should bake");
    assert_eq!(vehicle.aero_geometry.panels.len(), 4);
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.collision_geometry.parts.len(), 4);
    assert!(vehicle.collision_geometry.validate().is_ok());
    assert_eq!(vehicle.mass_properties.mass_kg, 1_000.0);
    let json = serde_json::to_string(&vehicle).expect("vehicle JSON should serialize");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON should deserialize");
    assert_eq!(round_trip, vehicle);
}

#[test]
fn assembly_asset_bakes_named_consumer_feed_routes() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_assembly.toml"))
            .expect("assembly vehicle TOML should parse");
    let vehicle = asset.bake().expect("assembly vehicle should bake");
    assert_eq!(vehicle.resource_feed_ports.len(), 1);
    assert_eq!(
        vehicle.resource_feed_ports[0],
        VehicleResourceFeedPort {
            consumer_name: "capsule-engine".into(),
            feed_port_name: "capsule.engine".into(),
            fluid_properties: vec![
                FeedResourceProperties {
                    resource: StoredPropellant::Lox,
                    density_kg_m3: 1_141.0,
                    viscosity_pa_s: 0.0002,
                    source_pressure_pa: 500_000.0,
                    minimum_pressure_pa: 100_000.0,
                },
                FeedResourceProperties {
                    resource: StoredPropellant::LiquidMethane,
                    density_kg_m3: 422.0,
                    viscosity_pa_s: 0.00012,
                    source_pressure_pa: 500_000.0,
                    minimum_pressure_pa: 100_000.0,
                },
            ],
        }
    );
    let assembly = vehicle.assembly.as_ref().expect("baked assembly graph");
    assert_eq!(assembly.resource_edges.len(), 1);
    assert!(assembly.resource_edges[0].open);
    assert!(assembly.resource_edges[0].feed_line.is_some());
    assert!(!assembly.links[0].state.open);
    assert!(
        assembly
            .feed_paths()
            .unwrap()
            .contains(&("stage.tank".into(), "capsule.engine".into()))
    );
}

#[test]
fn assembly_asset_bakes_ownership_and_splits_into_valid_clusters() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_assembly.toml"))
            .expect("assembly vehicle TOML should parse");
    let vehicle = asset.bake().expect("assembly vehicle should bake");
    let ownership = vehicle
        .assembly_ownership
        .as_ref()
        .expect("baked part ownership");
    assert_eq!(
        ownership.panel_bodies.len(),
        vehicle.aero_geometry.panels.len()
    );
    assert_eq!(
        ownership.collision_bodies.len(),
        vehicle.collision_geometry.parts.len()
    );
    assert_eq!(ownership.tank_bodies.len(), vehicle.tanks.len());
    assert_eq!(ownership.body_masses.len(), 2);
    assert!(
        ownership
            .panel_bodies
            .iter()
            .chain(&ownership.collision_bodies)
            .chain(&ownership.tank_bodies)
            .all(|body| *body < 2)
    );
    assert!(ownership.body_masses.iter().all(|body| body.mass_kg > 0.0));
    // JSON round-trip is structural, not bitwise: irrational-derived bake
    // values do not survive stock float formatting exactly (the exact
    // round-trip contract is pinned by the clean-decimal aircraft test).
    let json = serde_json::to_string(&vehicle).expect("vehicle JSON should serialize");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON should deserialize");
    assert_eq!(round_trip.name, vehicle.name);
    assert_eq!(
        round_trip.aero_geometry.panels.len(),
        vehicle.aero_geometry.panels.len()
    );
    let round_ownership = round_trip
        .assembly_ownership
        .as_ref()
        .expect("ownership round-trips");
    assert_eq!(round_ownership.panel_bodies, ownership.panel_bodies);
    assert_eq!(round_ownership.collision_bodies, ownership.collision_bodies);
    assert_eq!(round_ownership.tank_bodies, ownership.tank_bodies);
    assert_eq!(round_ownership.body_masses.len(), 2);
    for (restored, original) in round_ownership
        .body_masses
        .iter()
        .zip(&ownership.body_masses)
    {
        let scale = original.mass_kg.max(1.0);
        assert!((restored.mass_kg - original.mass_kg).abs() < 1e-9 * scale);
        assert!((restored.center_of_mass_body_m - original.center_of_mass_body_m).length() < 1e-9);
    }
    let state = RigidBodyState::stationary(DVec3::ZERO);
    let clusters = vehicle
        .split_definitions_after_link_failure("stack", state, &vehicle.initial_resource_state())
        .expect("assembly should split");
    assert_eq!(clusters.len(), 2);
    assert!(clusters[0].0.name.contains("stage"));
    assert!(clusters[1].0.name.contains("capsule"));
    let mut panels = 0;
    let mut tanks = 0;
    let mut mass_kg = 0.0;
    for (definition, cluster_state, _) in &clusters {
        definition.validate().expect("cluster validates");
        assert!(cluster_state.position_inertial_m.is_finite());
        assert!(cluster_state.velocity_inertial_mps.is_finite());
        panels += definition.aero_geometry.panels.len();
        tanks += definition.tanks.len();
        mass_kg += definition.mass_properties.mass_kg;
    }
    assert_eq!(panels, vehicle.aero_geometry.panels.len());
    assert_eq!(tanks, vehicle.tanks.len());
    let scale = vehicle.mass_properties.mass_kg.max(1.0);
    assert!((mass_kg - vehicle.mass_properties.mass_kg).abs() < 1e-6 * scale);
    // The capsule feed route follows its surviving engine port.
    assert!(clusters[0].0.resource_feed_ports.is_empty());
    assert_eq!(clusters[1].0.resource_feed_ports.len(), 1);
}

#[test]
fn reaction_wheel_asset_bakes_its_torque_ratings_mass_and_mount() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_spacecraft.toml"
    ))
    .expect("spacecraft TOML should parse");
    let vehicle = asset.bake().expect("spacecraft should bake");
    assert_eq!(vehicle.reaction_wheels.len(), 1);
    assert_eq!(
        vehicle.reaction_wheels[0].max_torque_body_nm,
        DVec3::new(250.0, 250.0, 180.0)
    );
    assert_eq!(vehicle.mass_properties.mass_kg, 1_228.0);
    assert!(vehicle.reaction_wheels[0].position_body_m.x > -0.4);
    let json = serde_json::to_string(&vehicle).expect("baked vehicle JSON");
    let decoded: VehicleDefinition = serde_json::from_str(&json).expect("vehicle round trip");
    assert_eq!(decoded.reaction_wheels[0].name, "service-module-wheel-box");
    assert_eq!(
        decoded.reaction_wheels[0].max_torque_body_nm,
        vehicle.reaction_wheels[0].max_torque_body_nm
    );
    assert!(
        (decoded.reaction_wheels[0].position_body_m - vehicle.reaction_wheels[0].position_body_m)
            .length()
            < 1.0e-12
    );
    assert_eq!(
        decoded.mass_properties.mass_kg,
        vehicle.mass_properties.mass_kg
    );
}

#[test]
fn electrical_power_asset_bakes_cell_arrays_sources_loads_and_center_of_mass() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_powered_spacecraft.toml"
    ))
    .expect("powered spacecraft TOML should parse");
    let vehicle = asset.bake().expect("powered spacecraft should bake");

    assert_eq!(vehicle.electrical_power.batteries.len(), 1);
    assert_eq!(vehicle.electrical_power.ultracapacitors.len(), 1);
    assert_eq!(vehicle.electrical_power.solar_arrays.len(), 2);
    assert_eq!(vehicle.electrical_power.reactors.len(), 1);
    assert_eq!(vehicle.electrical_power.consumers.len(), 3);
    assert!((vehicle.electrical_power.solar_arrays[0].area_m2() - 6.0).abs() < 1.0e-12);
    assert!((vehicle.electrical_power.solar_arrays[1].area_m2() - 9.6).abs() < 1.0e-12);
    assert!(vehicle.mass_properties.mass_kg > 2_200.0);
    assert!(vehicle.electrical_power.batteries[0].position_body_m.x < -0.8);
    assert_eq!(
        vehicle.electrical_power.ultracapacitors[0].name,
        "pulse-ionistor-bank"
    );
    assert!(
        (vehicle.electrical_power.ultracapacitors[0].mass_kg() - 4.0e6 / 36_000.0).abs() < 1.0e-9
    );
    assert_eq!(
        vehicle.electrical_power.solar_arrays[1].deployment,
        SolarArrayDeployment::Foldable {
            deployment_rate_per_s: 0.12,
            actuator_power_w: 350.0,
            initial_fraction: 0.0,
        }
    );
    assert_eq!(
        vehicle.electrical_power.solar_arrays[1].tracking,
        SolarArrayTracking::SingleAxis {
            rotation_axis_body: DVec3::X,
            minimum_angle_rad: -std::f64::consts::FRAC_PI_2,
            maximum_angle_rad: std::f64::consts::FRAC_PI_2,
            slew_rate_rad_s: 0.05,
            actuator_power_w: 120.0,
            initial_angle_rad: 0.0,
        }
    );

    let initial_state = vehicle
        .initial_electrical_power_state()
        .expect("power initial state");
    assert_eq!(initial_state.capacitor_energy_j.len(), 1);
    assert_eq!(initial_state.solar_array_tracking_angle_rad.len(), 2);
    let command = ElectricalPowerCommand::idle_for(&vehicle.electrical_power, 1.0);
    let (_, telemetry) = vehicle
        .advance_electrical_power(&initial_state, &command)
        .expect("vehicle power step");
    assert!(telemetry.reactor_available_power_w > 0.0);

    let encoded = serde_json::to_string(&vehicle).expect("baked vehicle JSON");
    let decoded: VehicleDefinition = serde_json::from_str(&encoded).expect("vehicle round trip");
    // COM-shift arithmetic plus JSON text round-trip can move the last float
    // bit; compare discrete authoring exactly and geometry within 1e-9 m.
    assert_eq!(
        decoded.electrical_power.batteries.len(),
        vehicle.electrical_power.batteries.len()
    );
    for (actual, expected) in decoded
        .electrical_power
        .batteries
        .iter()
        .zip(&vehicle.electrical_power.batteries)
    {
        assert_eq!(actual.name, expected.name);
        assert!((actual.capacity_j - expected.capacity_j).abs() < 1.0e-9);
        assert!((actual.position_body_m - expected.position_body_m).length() < 1.0e-9);
    }
    assert_eq!(
        decoded.electrical_power.ultracapacitors.len(),
        vehicle.electrical_power.ultracapacitors.len()
    );
    for (actual, expected) in decoded
        .electrical_power
        .ultracapacitors
        .iter()
        .zip(&vehicle.electrical_power.ultracapacitors)
    {
        assert_eq!(actual.name, expected.name);
        assert!((actual.capacity_j - expected.capacity_j).abs() < 1.0e-9);
        assert!((actual.position_body_m - expected.position_body_m).length() < 1.0e-9);
    }
    assert_eq!(
        decoded.electrical_power.solar_arrays.len(),
        vehicle.electrical_power.solar_arrays.len()
    );
    for (actual, expected) in decoded
        .electrical_power
        .solar_arrays
        .iter()
        .zip(&vehicle.electrical_power.solar_arrays)
    {
        assert_eq!(actual.name, expected.name);
        assert_eq!(actual.cell_count_x, expected.cell_count_x);
        assert_eq!(actual.cell_count_y, expected.cell_count_y);
        assert_eq!(actual.deployment, expected.deployment);
        assert_eq!(actual.tracking, expected.tracking);
        assert!((actual.position_body_m - expected.position_body_m).length() < 1.0e-9);
    }
    assert_eq!(
        decoded.electrical_power.reactors.len(),
        vehicle.electrical_power.reactors.len()
    );
    for (actual, expected) in decoded
        .electrical_power
        .reactors
        .iter()
        .zip(&vehicle.electrical_power.reactors)
    {
        assert_eq!(actual.name, expected.name);
        assert!((actual.position_body_m - expected.position_body_m).length() < 1.0e-9);
    }
    assert_eq!(
        decoded.electrical_power.consumers,
        vehicle.electrical_power.consumers
    );

    // Thermal section: two nodes, one link, one radiator; mass joins COM.
    assert_eq!(vehicle.thermal.nodes.len(), 2);
    assert_eq!(vehicle.thermal.links.len(), 1);
    assert_eq!(vehicle.thermal.radiators.len(), 1);
    assert!(vehicle.mass_properties.mass_kg > 2_200.0);
    let thermal_state = vehicle.initial_thermal_state().expect("thermal state");
    assert_eq!(thermal_state.node_temp_k, vec![280.0, 300.0]);
    // Wire the reactor's waste heat into its block node and step under sun.
    let mut heat_command = ThermalCommand::idle_for(&vehicle.thermal, 60.0);
    heat_command.solar_flux = vec![SolarFluxSource::new(1_360.0, DVec3::Z, 1.0).unwrap()];
    heat_command.internal_heat_w = vec![0.0, 5_000.0];
    let (hot_state, heat_report) = vehicle
        .advance_thermal(&thermal_state, &heat_command)
        .expect("thermal step");
    assert!(hot_state.node_temp_k[1] > 300.0);
    assert!(heat_report.total_internal_heat_w > 0.0);
    assert!(heat_report.total_rejected_heat_w > 0.0);
    assert_eq!(heat_report.nodes.len(), 2);
    assert_eq!(decoded.thermal.nodes.len(), vehicle.thermal.nodes.len());
    assert_eq!(
        decoded.thermal.radiators.len(),
        vehicle.thermal.radiators.len()
    );
    assert!(matches!(
        vehicle.thermal.radiators[0].deployment,
        thessa_sim_core::RadiatorDeployment::Foldable { .. }
    ));
    assert_eq!(
        decoded.mass_properties.mass_kg,
        vehicle.mass_properties.mass_kg
    );
    assert!(
        decoded
            .mass_properties
            .inertia_body_kg_m2
            .to_cols_array()
            .iter()
            .zip(vehicle.mass_properties.inertia_body_kg_m2.to_cols_array())
            .all(|(decoded, baked)| (decoded - baked).abs() < 1.0e-9)
    );
}

#[test]
fn parachute_asset_bakes_pressure_envelopes_mounts_and_pack_mass() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_parachute_vehicle.toml"
    ))
    .expect("parachute vehicle TOML should parse");
    let vehicle = asset.bake().expect("parachute vehicle should bake");
    assert_eq!(vehicle.parachutes.len(), 2);
    assert_eq!(vehicle.parachutes[0].name, "drogue");
    assert_eq!(vehicle.parachutes[0].deploy_pressure_pa, 22_000.0);
    assert_eq!(vehicle.parachutes[1].reference_area_m2, 32.0);
    assert_eq!(vehicle.mass_properties.mass_kg, 1_232.0);
    assert!(vehicle.parachutes[0].position_body_m.x > -2.2);
    let encoded = serde_json::to_string(&vehicle).expect("baked vehicle JSON");
    let decoded: VehicleDefinition = serde_json::from_str(&encoded).expect("vehicle round trip");
    for (decoded, baked) in decoded.parachutes.iter().zip(&vehicle.parachutes) {
        assert_eq!(decoded.name, baked.name);
        assert_eq!(decoded.reference_area_m2, baked.reference_area_m2);
        assert_eq!(decoded.deploy_pressure_pa, baked.deploy_pressure_pa);
        assert!((decoded.position_body_m - baked.position_body_m).length() < 1.0e-12);
    }
    assert_eq!(
        decoded.mass_properties.mass_kg,
        vehicle.mass_properties.mass_kg
    );
}

#[test]
fn wheel_chassis_asset_bakes_mass_and_recenters_its_mount() {
    let doc = r#"
name = "rover-wheel-bake"
mass_kg = 1000.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[wheel_chassis]]
name = "front-bogie"
mount_position_body_m = [1.0, 0.0, -0.5]
mount_orientation_body_xyzw = [0.0, 0.0, 0.0, 1.0]
length_m = 0.8
layout = "inline"
wheel_count = 1
structural_mass_kg = 8.0
structural_inertia_local_kg_m2 = [[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]

[wheel_chassis.tire]
construction = { kind = "pneumatic", inflation_pressure_pa = 220000.0, reference_temperature_k = 293.15 }
radius_m = 0.32
width_m = 0.18
mass_kg = 3.4
spin_inertia_kg_m2 = 0.11
radial_stiffness_n_m = 200000.0
radial_damping_n_s_m = 10000.0
longitudinal_slip_stiffness_n_per_mps = 12000.0
lateral_slip_stiffness_n_per_mps = 9000.0
maximum_deflection_m = 0.08
maximum_load_n = 8000.0
surface_friction = 0.9

[wheel_chassis.strut]
extended_length_m = 0.4
stroke_m = 0.15
spring_rate_n_m = 30000.0
damping_n_s_m = 2000.0
preload_n = 0.0
minimum_force_n = 0.0
maximum_force_n = 10000.0
mass_per_wheel_kg = 0.5

[wheel_chassis.brake]
maximum_torque_nm = 200.0
response_time_s = 0.1
mass_per_wheel_kg = 0.3

[wheel_chassis.drive]
stall_copper_loss_w = 120.0
rotor_inertia_kg_m2 = 0.004
final_drive_ratio = 18.0
drivetrain_efficiency = 0.91
driven_wheel_count = 1

[wheel_chassis.drive.motor]
rated_power_w = 1600.0
peak_torque_nm = 5.0
maximum_rpm = 12000.0
efficiency = 0.94
cooling_capacity_w = 640.0
dry_mass_kg = 5.0

[wheel_chassis.retraction]
pivot_position_body_m = [0.5, 0.0, 0.0]
hinge_axis_body = [0.0, 1.0, 0.0]
stowed_angle_rad = -1.25
deployed_angle_rad = 0.0
initially_deployed = true
deployment_rate_rad_s = 0.7
actuator_max_torque_nm = 500.0

[[landing_legs]]
name = "apollo-foldout"
mount_position_body_m = [-1.0, 0.0, -0.2]
hinge_axis_body = [0.0, 1.0, 0.0]
stowed_leg_axis_body = [0.0, 0.0, 1.0]
stowed_angle_rad = 0.0
deployed_angle_rad = 3.141592653589793
initially_deployed = false
deployment_rate_rad_s = 0.6
actuator_max_torque_nm = 20000.0
leg_length_m = 2.0
leg_mass_kg = 18.0
footpad_radius_m = 0.2
footpad_mass_kg = 3.0
footpad_friction = 0.8
footpad_slip_stiffness_n_per_mps = 5000.0
shock_absorber = { kind = "reusable", stroke_m = 0.2, spring_rate_n_m = 40000.0, damping_n_s_m = 1000.0, preload_n = 0.0, bottom_out_stiffness_n_m = 250000.0, maximum_force_n = 80000.0 }
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("wheel chassis TOML parses");
    let vehicle = asset.bake().expect("wheel chassis asset bakes");
    assert_eq!(vehicle.wheel_chassis.len(), 1);
    assert_eq!(vehicle.wheel_chassis[0].wheel_stations.len(), 1);
    assert_eq!(vehicle.landing_legs.len(), 1);
    assert!((vehicle.mass_properties.mass_kg - 1_038.2).abs() < 1.0e-10);
    assert!(vehicle.wheel_chassis[0].drive.is_some());
    let retraction = vehicle.wheel_chassis[0]
        .spec
        .retraction
        .expect("wheel retraction config bakes");
    assert!(retraction.pivot_position_body_m.is_finite());
    assert!(retraction.initially_deployed);
    assert!(vehicle.wheel_chassis[0].spec.mount_position_body_m.x < 1.01);
    assert!(vehicle.landing_legs[0].spec.mount_position_body_m.x > -1.0);
    vehicle
        .validate()
        .expect("recentered wheel asset validates");
    let split = vehicle
        .wheel_mass_split()
        .expect("recentered vehicle splits into sprung and unsprung masses");
    assert_eq!(split.wheels.len(), 1);
    let recovered_mass = split.sprung_properties.mass_kg + split.wheels[0].mass_kg;
    assert!((recovered_mass - vehicle.mass_properties.mass_kg).abs() < 1.0e-10);
    let split_com = split.sprung_center_of_mass_body_m * split.sprung_properties.mass_kg
        + split.wheels[0].center_of_mass_body_m * split.wheels[0].mass_kg;
    assert!(split_com.length() < 1.0e-10);
    let recovered_inertia = split.sprung_properties.inertia_body_kg_m2
        + parallel_axis(
            split.sprung_properties.mass_kg,
            split.sprung_center_of_mass_body_m,
        )
        + split.wheels[0].inertia_body_kg_m2
        + parallel_axis(
            split.wheels[0].mass_kg,
            split.wheels[0].center_of_mass_body_m,
        );
    for (actual, expected) in recovered_inertia
        .to_cols_array()
        .into_iter()
        .zip(vehicle.mass_properties.inertia_body_kg_m2.to_cols_array())
    {
        assert!((actual - expected).abs() < 1.0e-8);
    }

    let json = serde_json::to_string(&vehicle).expect("vehicle JSON serializes");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON deserializes");
    round_trip
        .validate()
        .expect("serialized wheel vehicle remains internally consistent");
    assert_eq!(round_trip.wheel_chassis[0].spec.name, "front-bogie");
    assert_eq!(round_trip.landing_legs[0].spec.name, "apollo-foldout");
    assert_eq!(
        round_trip.wheel_chassis[0].wheel_stations.len(),
        vehicle.wheel_chassis[0].wheel_stations.len()
    );
    assert!((round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs() < 1.0e-12);

    let axle_pair_asset = doc
        .replace(
            "layout = \"inline\"",
            "layout = { axle_pairs = { track_width_m = 0.6 } }",
        )
        .replace("wheel_count = 1", "wheel_count = 2");
    let axle_pair_vehicle: VehicleDefinition = toml::from_str::<VehicleAsset>(&axle_pair_asset)
        .expect("axle-pair wheel TOML parses")
        .bake()
        .expect("axle-pair wheel chassis bakes");
    assert_eq!(axle_pair_vehicle.wheel_chassis[0].wheel_stations.len(), 2);

    let airless_asset = doc.replace(
            "construction = { kind = \"pneumatic\", inflation_pressure_pa = 220000.0, reference_temperature_k = 293.15 }",
            "construction = { kind = \"airless\", structure = { spoked = { spoke_count = 24 } }, structure_density_kg_m3 = 4400.0, minimum_temperature_k = 80.0, maximum_temperature_k = 500.0 }",
        );
    let airless_vehicle: VehicleDefinition = toml::from_str::<VehicleAsset>(&airless_asset)
        .expect("airless wheel TOML parses")
        .bake()
        .expect("airless wheel chassis bakes");
    assert_eq!(airless_vehicle.wheel_chassis.len(), 1);
}

#[test]
fn vehicle_asset_parses_crushable_landing_cartridge() {
    let doc = r#"
name = "one-shot-lander"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[landing_legs]]
name = "impact-leg"
mount_position_body_m = [0.0, 0.0, 0.0]
hinge_axis_body = [0.0, 1.0, 0.0]
stowed_leg_axis_body = [0.0, 0.0, 1.0]
stowed_angle_rad = 0.0
deployed_angle_rad = 3.141592653589793
initially_deployed = true
deployment_rate_rad_s = 0.5
actuator_max_torque_nm = 12000.0
leg_length_m = 2.0
leg_mass_kg = 18.0
footpad_radius_m = 0.2
footpad_mass_kg = 3.0
footpad_friction = 0.8
footpad_slip_stiffness_n_per_mps = 5000.0
shock_absorber = { kind = "crushable", elastic_stiffness_n_m = 120000.0, damping_n_s_m = 7000.0, plateau_force_n = 30000.0, maximum_crush_m = 0.35, bottom_out_stiffness_n_m = 500000.0, maximum_force_n = 220000.0 }
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("crushable shock TOML parses");
    let vehicle = asset.bake().expect("crushable shock vehicle bakes");
    assert_eq!(vehicle.mass_properties.mass_kg, 1_021.0);
    assert_eq!(vehicle.landing_legs.len(), 1);
    assert!(matches!(
        vehicle.landing_legs[0].spec.shock_absorber,
        LandingShockAbsorberSpec::Crushable { .. }
    ));
    vehicle.validate().expect("one-shot cartridge validates");
}

#[test]
fn electric_propeller_drive_bakes_mass_wrench_and_json() {
    let doc = r#"
name = "electric-prop-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[propeller_drives]]
name = "electric-cruise"
mount_position_body_m = [0.0, 1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reduction_ratio = 2.0

[propeller_drives.propeller]
diameter_m = 2.2
gearbox_efficiency = 0.96

[propeller_drives.source]
kind = "electric"

[propeller_drives.source.spec]
rated_power_w = 90000.0
peak_torque_nm = 420.0
maximum_rpm = 10000.0
efficiency = 0.92
cooling_capacity_w = 10000.0
dry_mass_kg = 32.0
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("drive TOML parses");
    let vehicle = asset.bake().expect("drive asset bakes");
    assert_eq!(vehicle.propeller_drives.len(), 1);
    assert!(matches!(
        vehicle.propeller_drives[0].drive.source,
        CompiledShaftPowerSource::Electric(_)
    ));
    assert!(vehicle.mass_properties.mass_kg > 1_032.0);

    let sample = AtmosphereConfig::default().sample(0.0).expect("atmosphere");
    let condition = thessa_sim_core::flight_condition(&sample, 60.0).expect("condition");
    let ((force, moment), points) = vehicle
        .propeller_drives_wrench_body_n(
            &[PropellerDriveCommand {
                throttle: 1.0,
                source_rpm: 6_000.0,
            }],
            &condition,
        )
        .expect("wrench");
    assert!(force.x > 0.0);
    assert!(moment.z.abs() > 0.0);
    assert_eq!(points.len(), 1);

    let json = serde_json::to_string(&vehicle).expect("vehicle JSON");
    let round_trip: VehicleDefinition = serde_json::from_str(&json).expect("JSON round-trip");
    assert_eq!(round_trip.propeller_drives.len(), 1);
    assert!((round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs() < 1e-9);
}
#[test]
fn turboprop_asset_bakes_mass_wrench_and_serialized_state() {
    let doc = r#"
name = "turboprop-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[turboprops]]
name = "left-turboprop"
mount_position_body_m = [0.0, 1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]

[turboprops.drive]
shaft_rpm_at_full_spool = 12000.0
reduction_ratio = 6.0
power_turbine_mass_kg = 40.0

[turboprops.drive.air]
name = "left-core"
cycle = "turbojet"
fuel = "kerosene"
intake_area_m2 = 0.8
intake = "pitot"
compressor_ratio = 8.0
bypass_ratio = 0.0
fan_pressure_ratio = 1.0
turbine_inlet_temp_k = 1400.0
afterburner = false
reheat_temp_k = 0.0
turbine_material = { density_kg_m3 = 8190.0, yield_strength_pa = 1000000000.0, max_wall_temp_k = 1350.0 }
spool_tau_s = 4.0

[turboprops.drive.air.shaft]
power_turbine_heat_fraction = 0.15

[turboprops.drive.propeller]
diameter_m = 2.4
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("turboprop TOML parses");
    let vehicle = asset.bake().expect("turboprop bakes");
    assert_eq!(vehicle.turboprops.len(), 1);
    assert!(vehicle.mass_properties.mass_kg > 1_040.0);

    let sample = AtmosphereConfig::default().sample(0.0).expect("atmosphere");
    let condition = thessa_sim_core::flight_condition(&sample, 0.0).expect("condition");
    let drive = &vehicle.turboprops[0].drive;
    let (_, balance) = drive
        .air
        .operating_point_at_spool_loaded(&condition, 1.0, 1.0, true, 0.0)
        .expect("takeoff capacity");
    let mut command = TurbopropCommand::running(drive);
    command.propeller_power_w = balance.power_takeoff_capacity_w * 0.1;
    let ((force, moment), next) = vehicle
        .turboprops_wrench_body_n_stateful(&[command], &condition)
        .expect("wrench");
    assert!(force.x > 0.0);
    assert!(moment.z.abs() > 0.0);
    assert_eq!(next.len(), 1);

    let json = serde_json::to_string(&vehicle).expect("serialize");
    let round_trip: VehicleDefinition = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(round_trip.turboprops.len(), 1);
    assert!((round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs() < 1e-9);
}

#[test]
fn pure_fluid_tanks_fuel_cell_and_apu_bake_and_round_trip() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_apu_fuel_cell.toml"
    ))
    .expect("APU/fuel-cell vehicle TOML should parse");
    let vehicle = asset.bake().expect("APU/fuel-cell vehicle should bake");
    assert_eq!(vehicle.auxiliary_power_units.len(), 1);
    assert_eq!(vehicle.electrical_power.fuel_cells.len(), 1);
    assert_eq!(vehicle.rcs_mounts.len(), 1);
    assert_eq!(vehicle.tanks.len(), 4);
    assert_eq!(
        vehicle.tanks[0].resource,
        TankResource::Stored(StoredPropellant::Rp1)
    );
    assert_eq!(
        vehicle.tanks[1].resource,
        TankResource::Stored(StoredPropellant::LiquidHydrogen)
    );
    assert_eq!(
        vehicle.tanks[2].resource,
        TankResource::Stored(StoredPropellant::Lox)
    );
    assert_eq!(
        vehicle.auxiliary_power_units[0]
            .unit
            .engine
            .shaft
            .generator
            .power_w,
        20_000.0
    );
    let json = serde_json::to_string(&vehicle).expect("vehicle JSON serialization");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON deserialization");
    assert_eq!(round_trip.auxiliary_power_units.len(), 1);
    assert_eq!(round_trip.electrical_power.fuel_cells.len(), 1);
    assert_eq!(round_trip.rcs_mounts.len(), 1);
    assert_eq!(
        round_trip.tanks[1].resource,
        TankResource::Stored(StoredPropellant::LiquidHydrogen)
    );
    assert_eq!(
        round_trip.mass_properties.mass_kg,
        vehicle.mass_properties.mass_kg
    );
    assert!(
        (round_trip.electrical_power.fuel_cells[0].position_body_m
            - vehicle.electrical_power.fuel_cells[0].position_body_m)
            .length()
            < 1.0e-12
    );
}

#[test]
fn collision_part_asset_bakes_without_backend_types() {
    let part: CollisionPartAsset = toml::from_str(
        r#"
shape = "capsule"
axis = "x"
half_segment_m = 2.0
radius_m = 0.5
friction = 0.8
"#,
    )
    .expect("collision part TOML should parse");
    let baked = part.bake().expect("collision part should validate");
    assert_eq!(baked.local_position_m, DVec3::ZERO);
    assert_eq!(baked.local_orientation, DQuat::IDENTITY);
    assert_eq!(baked.material.friction, 0.8);
    assert!(matches!(
        baked.shape,
        CollisionShape::Capsule {
            axis: CollisionAxis::X,
            half_segment_m: 2.0,
            radius_m: 0.5,
        }
    ));
}

#[test]
fn example_rocket_bakes_compiled_engines_with_mass() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_rocket.toml"))
            .expect("rocket TOML should parse");
    let vehicle = asset.bake().expect("rocket asset should bake");
    assert_eq!(vehicle.engines.len(), 2);
    assert_eq!(vehicle.tanks.len(), 2);
    // Engine + tank masses aggregate on top of the 2000 kg structure.
    assert!(vehicle.mass_properties.mass_kg > 2000.0);
    let engines_mass: f64 = vehicle
        .engines
        .iter()
        .map(|mount| mount.engine.bake_mass_kg())
        .sum();
    let tanks_mass: f64 = vehicle
        .tanks
        .iter()
        .map(|mount| mount.tank.dry_mass_kg + mount.loaded_propellant_kg())
        .sum();
    assert!(
        (vehicle.mass_properties.mass_kg - 2000.0 - engines_mass - tanks_mass).abs() < 1e-6,
        "baked mass must equal structure plus engines plus tanks"
    );
    // Uniform full-throttle command produces +X thrust at sea level.
    let thrust = vehicle
        .total_thrust_body_n(1.0, 101_325.0, 0.0)
        .expect("thrust evaluates");
    assert!(thrust.x > 1.0e6, "main + booster must clear 1 MN");
    assert_eq!(thrust.y, 0.0);
    assert_eq!(thrust.z, 0.0);
    let json = serde_json::to_string(&vehicle).expect("vehicle JSON serializes");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON deserializes");
    // JSON is a transfer artifact, not a bitwise archive (serde_json
    // float parsing can dust the last ulp): compare structurally with
    // a tight relative tolerance instead of exact equality.
    assert_eq!(round_trip.engines.len(), vehicle.engines.len());
    for (actual, expected) in round_trip.engines.iter().zip(vehicle.engines.iter()) {
        assert_eq!(actual.name, expected.name);
        let mass_delta = (actual.engine.bake_mass_kg() - expected.engine.bake_mass_kg()).abs()
            / expected.engine.bake_mass_kg();
        assert!(mass_delta < 1e-9, "engine mass drift {mass_delta:e}");
    }
    let mass_delta = (round_trip.mass_properties.mass_kg - vehicle.mass_properties.mass_kg).abs()
        / vehicle.mass_properties.mass_kg;
    assert!(mass_delta < 1e-12, "vehicle mass drift {mass_delta:e}");
}

#[test]
fn engine_mount_axis_must_be_unit() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_rocket.toml"))
            .expect("rocket TOML should parse");
    let mut vehicle = asset.bake().expect("rocket asset should bake");
    vehicle.engines[0].thrust_axis_body = [2.0, 0.0, 0.0];
    assert!(vehicle.validate().is_err());
}

#[test]
fn pressure_fed_demands_tank_pressure() {
    // A pressure-fed engine with only a 0.5 MPa tank must refuse: no
    // pump hides the shortfall. Pump-fed engines pass the same tanks.
    let doc = r#"
name = "fed-test"
mass_kg = 500.0
inertia_body_kg_m2 = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[engines]]
name = "fed"
kind = "liquid"
propellant = "lox-methane"
cycle = "pressure-fed"
chamber_pressure_mpa = 2.0
throat_radius_m = 0.05
expansion_ratio = 10.0
nozzle_length_m = 0.5
contour = "conical"
material = "regen-alloy"
cooling = "regenerative"
[[tanks]]
name = "weak-tank"
shape = "sphere"
diameter_m = 1.0
pressure_mpa = 0.5
material = "regen-alloy"
propellant = "lox-methane"
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
    assert!(
        asset.bake().is_err(),
        "0.5 MPa tank cannot pressure-feed a 2.4 MPa circuit"
    );
    let strong = doc.replace("pressure_mpa = 0.5", "pressure_mpa = 3.0");
    let asset: VehicleAsset = toml::from_str(&strong).expect("TOML parses");
    assert!(asset.bake().is_ok());
}

#[test]
fn nuclear_and_rcs_assets_bake() {
    // NTR upper stage plus a hydrazine RCS block on one airframe.
    let doc = r#"
name = "ntr-test"
mass_kg = 8000.0
inertia_body_kg_m2 = [[20000.0, 0.0, 0.0], [0.0, 20000.0, 0.0], [0.0, 0.0, 8000.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[engines]]
name = "ntr-main"
kind = "nuclear"
mount_position_body_m = [-4.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fluid = "hydrogen"
core_temp_k = 2700.0
core_power_mw = 600.0
throat_radius_m = 0.09
expansion_ratio = 60.0
nozzle_length_m = 1.6
contour = "bell"
material = "nickel-superalloy"
cooling = "regenerative"
gimbal_range_rad = 0.05
[[engines]]
name = "rcs-a"
kind = "liquid"
mount_position_body_m = [2.0, 0.0, 1.0]
thrust_axis_body = [0.0, 0.0, -1.0]
propellant = "monoprop-hydrazine"
cycle = "pressure-fed"
chamber_pressure_mpa = 1.0
throat_radius_m = 0.002
expansion_ratio = 60.0
nozzle_length_m = 0.06
contour = "conical"
material = "regen-alloy"
cooling = "regenerative"
min_throttle = 1.0
[[tanks]]
name = "hydrazine-tank"
shape = "sphere"
diameter_m = 0.6
pressure_mpa = 2.0
material = "regen-alloy"
position_body_m = [1.0, 0.0, 0.0]
propellant = "monoprop-hydrazine"
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
    let vehicle = asset.bake().expect("NTR+RCS bakes");
    assert_eq!(vehicle.engines.len(), 2);
    // Reactor-dominated mass far above the 8 t structure.
    assert!(vehicle.mass_properties.mass_kg > 12_000.0);
    // NTR thrust clears 100 kN in vacuum; the RCS block is a small
    // transverse couple, not axial thrust.
    let ntr = vehicle
        .engine_thrust_body_n(0, 1.0, 0.0, 0.0)
        .expect("ntr thrust");
    assert!(ntr.x > 100_000.0);
    let (force, moment) = vehicle
        .wrench_body_n(&[(1.0, 0.0), (1.0, 0.0)], 0.0)
        .expect("wrench");
    assert!((force.x - ntr.x).abs() < 1.0, "axial thrust is the NTR");
    assert!(force.z.abs() < 100.0, "RCS fires transversely");
    assert!(moment.length() > 0.0, "offset RCS must couple");
}

#[test]
fn twin_chamber_system_bakes_with_shared_feed() {
    // Two chambers on one GG feed: totals match the parts, system dry
    // mass lands in the baked vehicle mass, differential throttle
    // couples through the stations.
    let doc = r#"
name = "twin-test"
mass_kg = 2000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 1500.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0
[[tanks]]
name = "rp1-tank"
shape = "sphere"
diameter_m = 2.0
pressure_mpa = 0.5
material = "regen-alloy"
position_body_m = [1.0, 0.0, 0.0]
propellant = "lox-rp1"
[[systems]]
name = "twin"
propellant = "lox-rp1"
cycle = "gas-generator"
chamber_pressure_mpa = 9.7
material = "nickel-superalloy"
cooling = "regenerative"
[[systems.chambers]]
name = "a"
throat_radius_m = 0.134
expansion_ratio = 16.0
nozzle_length_m = 1.5
contour = "bell"
position_body_m = [-3.0, 0.0, 0.5]
thrust_axis_body = [1.0, 0.0, 0.0]
gimbal_range_rad = 0.09
[[systems.chambers]]
name = "b"
throat_radius_m = 0.100
expansion_ratio = 16.0
nozzle_length_m = 1.2
contour = "bell"
position_body_m = [-3.0, 0.0, -0.5]
thrust_axis_body = [1.0, 0.0, 0.0]
gimbal_range_rad = 0.09
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
    let vehicle = asset.bake().expect("twin system bakes");
    assert_eq!(vehicle.systems.len(), 1);
    let system = &vehicle.systems[0].system;
    assert_eq!(system.chambers.len(), 2);
    // System dry (shared turbo booked once) sits inside baked mass.
    let tanks_mass: f64 = vehicle
        .tanks
        .iter()
        .map(|mount| mount.tank.dry_mass_kg + mount.loaded_propellant_kg())
        .sum();
    assert!(
        (vehicle.mass_properties.mass_kg - 2000.0 - tanks_mass - system.dry_mass_kg).abs() < 1e-6
    );
    // Uniform full throttle matches the vacuum total; differential
    // throttle steers about Y.
    let total = vehicle.total_thrust_body_n(1.0, 0.0, 0.0).expect("total");
    assert!((total.x - system.total_thrust_vac_n).abs() / system.total_thrust_vac_n < 1e-9);
    let (force, moment) = vehicle
        .system_wrench_body_n(0, &[1.0, 0.5], 0.0)
        .expect("system wrench");
    let expected = system
        .operating_point(&[1.0, 0.5], 0.0)
        .expect("system point")
        .thrust_n;
    assert!((force.x - expected).abs() / expected < 1e-12);
    assert!(moment.y.abs() > 0.0, "differential must couple");
    assert!(vehicle.system_wrench_body_n(0, &[1.0], 0.0).is_err());
}

#[test]
fn jet_and_estoc_assets_bake() {
    // Turbojet plus an ESTOC sharing the airframe: jet mass lands in
    // baked mass, static thrust is axial, ESTOC rocket branch compiles.
    let doc = r#"
name = "jet-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]
[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0
[[jets]]
name = "cruise-jet"
kind = "jet"
mount_position_body_m = [1.0, -1.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fuel = "kerosene"
intake_area_m2 = 0.5
intake = "pitot"
compressor_ratio = 8.0
turbine_inlet_temp_k = 1400.0
material = "nickel-superalloy"

[jets.shaft]
design_speed_rad_s = 900.0
rotor_inertia_kg_m2 = 3.0

[jets.shaft.starter]
kind = "rocket-bootstrap"
power_w = 12000.0
charge_j = 0.0
resource = "hydrazine"
specific_energy_j_kg = 1000000.0
maximum_shaft_torque_nm = 40.0
mass_kg = 2.0

[jets.shaft.generator]
fitted = true
power_w = 18000.0
efficiency = 0.85
efficiency_map = [{ spool_n = 0.5, efficiency = 0.75 }, { spool_n = 1.0, efficiency = 0.9 }]
maximum_shaft_torque_nm = 30.0
cut_in_spool_n = 0.5
mass_kg = 5.0

[jets.shaft.generator.thermal]
heat_capacity_j_k = 50000.0
conductance_w_k = 20.0
initial_temperature_k = 300.0
maximum_temperature_k = 420.0

[[jets]]
name = "estoc-1"
kind = "estoc"
mount_position_body_m = [0.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fuel = "kerosene"
bulk_fuel = "methane"
boost_coolant_fuel = "hydrogen"
intake_area_m2 = 0.9
intake = "pitot"
compressor_ratio = 12.0
turbine_inlet_temp_k = 1500.0
material = "nickel-superalloy"
rocket_chamber_pressure_mpa = 7.0
rocket_throat_radius_m = 0.09

[jets.precooler]
maximum_heat_flow_w = 20000000.0
effectiveness = 0.85
maximum_compressor_inlet_temp_k = 500.0
pressure_recovery = 0.98
wall_mass_kg = 500.0
wall_specific_heat_j_kg_k = 1000.0
wall_initial_temp_k = 300.0
wall_max_temp_k = 800.0
coolant_inlet_temp_k = 20.0
coolant_max_outlet_temp_k = 400.0
coolant_specific_heat_j_kg_k = 14000.0
maximum_coolant_flow_kg_s = 0.1

[jets.ejector]
capture_area_m2 = 0.06
mixing_length_m = 2.0
mixing_efficiency = 0.95
structure_density_kg_m3 = 2700.0
wall_thickness_m = 0.005
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("TOML parses");
    let vehicle = asset.bake().expect("jets bake");
    assert_eq!(vehicle.jets.len(), 2);
    let jets_mass: f64 = vehicle
        .jets
        .iter()
        .map(|mount| mount.engine.dry_mass_kg())
        .sum();
    assert!(
        (vehicle.mass_properties.mass_kg - 3000.0 - jets_mass).abs() < 1e-6,
        "baked mass must equal structure plus jets"
    );
    assert!(vehicle.jets[1].engine.dry_mass_kg() > vehicle.jets[0].engine.dry_mass_kg());
    let CompiledJet::Air(engine) = &vehicle.jets[0].engine else {
        panic!("first mount is the airbreather");
    };
    assert_eq!(
        engine.shaft.starter.resource,
        Some(StoredPropellant::Hydrazine)
    );
    assert_eq!(engine.shaft.starter.maximum_shaft_torque_nm, Some(40.0));
    assert_eq!(engine.shaft.design_speed_rad_s, Some(900.0));
    assert_eq!(engine.shaft.generator.efficiency_map.len(), 2);
    assert!(engine.shaft.generator.thermal.is_some());
    let CompiledJet::Estoc(engine) = &vehicle.jets[1].engine else {
        panic!("second mount is the ESTOC");
    };
    assert_eq!(engine.bulk_fuel, JetFuel::Methane);
    assert_eq!(engine.boost_coolant_fuel, Some(JetFuel::Hydrogen));
    assert!(engine.precooler.is_some());
}

#[test]
fn multi_spool_turbofan_asset_bakes_and_round_trips() {
    let doc = r#"
name = "two-spool-turbofan"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[jets]]
name = "lp-hp-fan"
kind = "jet"
cycle = "turbofan"
mount_position_body_m = [1.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
fuel = "kerosene"
intake_area_m2 = 0.8
compressor_ratio = 18.0
bypass_ratio = 4.0
fan_pressure_ratio = 1.5
turbine_inlet_temp_k = 1500.0
material = "nickel-superalloy"

[jets.shaft.multi_spool]
low_pressure_design_speed_rad_s = 500.0
low_pressure_rotor_inertia_kg_m2 = 4.0
fan_rotor_inertia_kg_m2 = 2.0
high_pressure_design_speed_rad_s = 1000.0
high_pressure_rotor_inertia_kg_m2 = 2.0
high_pressure_turbine_power_fraction = 0.55
fan_gear_speed_ratio = 0.5
fan_gear_efficiency = 0.95
"#;

    let calibration_doc = doc.replace(
        "[jets.shaft.multi_spool]\nlow_pressure_design_speed_rad_s = 500.0\nlow_pressure_rotor_inertia_kg_m2 = 4.0\nfan_rotor_inertia_kg_m2 = 2.0\nhigh_pressure_design_speed_rad_s = 1000.0\nhigh_pressure_rotor_inertia_kg_m2 = 2.0\nhigh_pressure_turbine_power_fraction = 0.55\nfan_gear_speed_ratio = 0.5\nfan_gear_efficiency = 0.95\n",
        "",
    );
    assert_ne!(
        calibration_doc, doc,
        "calibration fixture removes its shaft train"
    );
    let calibration_asset: VehicleAsset =
        toml::from_str(&calibration_doc).expect("single-spool calibration TOML parses");
    let calibration_vehicle = calibration_asset
        .bake()
        .expect("single-spool calibration asset bakes");
    let CompiledJet::Air(calibration_engine) = &calibration_vehicle.jets[0].engine else {
        panic!("calibration asset compiled as an airbreather");
    };
    let sample = AtmosphereConfig::default()
        .sample(0.0)
        .expect("static calibration atmosphere");
    let condition = flight_condition(&sample, 0.0).expect("static condition");
    let (_, design_balance) = calibration_engine
        .operating_point_at_spool(&condition, 1.0, 1.0, true)
        .expect("turbofan design shaft balance");
    let hp_fraction = design_balance.high_pressure_demand_w
        / (design_balance.high_pressure_demand_w + design_balance.low_pressure_fan_demand_w / 0.95);
    let doc = doc.replace(
        "high_pressure_turbine_power_fraction = 0.55",
        &format!("high_pressure_turbine_power_fraction = {hp_fraction:.12}"),
    );

    let asset: VehicleAsset = toml::from_str(&doc).expect("multi-spool TOML parses");
    let vehicle = asset.bake().expect("multi-spool turbofan bakes");
    let thessa_sim_core::CompiledJet::Air(engine) = &vehicle.jets[0].engine else {
        panic!("asset compiled as an airbreather");
    };
    let spools = engine.shaft.multi_spool.expect("spool train retained");
    assert_eq!(spools.fan_gear_speed_ratio, 0.5);
    assert!((spools.high_pressure_turbine_power_fraction - hp_fraction).abs() < 5.0e-13);

    let json = serde_json::to_string(&vehicle).expect("vehicle serializes");
    let restored: VehicleDefinition = serde_json::from_str(&json).expect("vehicle restores");
    assert_eq!(restored, vehicle);
    let cold = thessa_sim_core::JetCommand::cold(&restored.jets[0].engine);
    assert_eq!(cold.shaft.spool_n, 0.0);
    assert_eq!(cold.shaft.low_pressure_spool_n, Some(0.0));
}

#[test]
fn shaped_solid_grain_profiles_are_authorable_in_toml() {
    let doc = r#"
name = "shaped-solid-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[engines]]
name = "star-booster"
kind = "solid"
outer_radius_m = 0.5
core_radius_m = 0.16
segment_length_m = 1.0
segments = 2
throat_radius_m = 0.12
expansion_ratio = 10.0
nozzle_length_m = 0.9
material = "nickel-superalloy"
grain_geometry = { kind = "star", tip_count = 6, tip_radius_m = 0.30 }

[[engines]]
name = "finocyl-sustainer"
kind = "solid"
outer_radius_m = 0.5
core_radius_m = 0.16
segment_length_m = 1.0
segments = 2
throat_radius_m = 0.12
expansion_ratio = 10.0
nozzle_length_m = 0.9
material = "nickel-superalloy"
grain_geometry = { kind = "finocyl", fin_count = 8, fin_tip_radius_m = 0.32, fin_width_rad = 0.24 }
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("shaped grain TOML parses");
    let vehicle = asset.bake().expect("shaped solid assets bake");
    assert_eq!(vehicle.engines.len(), 2);
    for (engine, expected_geometry) in [
        (
            &vehicle.engines[0].engine,
            SolidGrainGeometry::Star {
                tip_count: 6,
                tip_radius_m: 0.30,
            },
        ),
        (
            &vehicle.engines[1].engine,
            SolidGrainGeometry::Finocyl {
                fin_count: 8,
                fin_tip_radius_m: 0.32,
                fin_width_rad: 0.24,
            },
        ),
    ] {
        let CompiledEngine::Solid(motor) = engine else {
            panic!("expected solid motor");
        };
        assert_eq!(motor.grain_geometry, expected_geometry);
        assert!(motor.burn_curve[0].burn_surface_area_m2 > 0.0);
    }
}

#[test]
fn electric_space_thruster_is_authorable_and_bakes_wrench_mass_and_state() {
    let doc = r#"
name = "electric-spacecraft-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[electric_thrusters]]
name = "aft-ion"
mount_position_body_m = [-1.0, 0.8, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "xenon"
design = { kind = "gridded-ion", accelerator_voltage_v = 1000.0, grid_diameter_m = 0.4, grid_gap_m = 0.002, max_beam_current_density_a_m2 = 100.0, propellant_utilization = 0.95, accelerator_efficiency = 0.9 }
maximum_power_w = 5000.0
maximum_mass_flow_kg_s = 0.00001
power_processor_specific_power_w_kg = 2000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.003
radiator_area_m2 = 10.0
radiator_temperature_k = 700.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0
ionization_efficiency = 0.75
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("electric-thruster TOML parses");
    let vehicle = asset.bake().expect("electric-thruster asset bakes");
    assert_eq!(vehicle.electric_thrusters.len(), 1);
    assert!(vehicle.mass_properties.mass_kg > 3_000.0);
    assert!(vehicle.electric_thrusters[0].engine.dry_mass_kg > 0.0);
    let ((force, moment), points) = vehicle
        .electric_thrusters_wrench_body_n(&[thessa_sim_core::ElectricThrusterCommand {
            available_power_w: 5_000.0,
            requested_mass_flow_kg_s: 1.0e-6,
        }])
        .expect("mounted electric drive wrench");
    assert!(force.x > 0.0);
    assert!(moment.z < 0.0);
    assert_eq!(points.len(), 1);
    assert!(points[0].waste_heat_w <= points[0].radiator_capacity_w);
}

#[test]
fn continuous_and_pulsed_fusion_mounts_are_authorable_and_recentered() {
    let doc = r#"
name = "fusion-spacecraft-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[fusion_torches]]
name = "dt-torch"
mount_position_body_m = [-2.0, 0.5, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reaction = "deuterium-tritium"
working_fluid = "hydrogen"
maximum_fusion_power_w = 100000000.0
fusion_gain = 10.0
maximum_working_flow_kg_s = 0.001
reactor_specific_power_w_kg = 10000.0
plasma_coupling_efficiency = 0.9
magnetic_nozzle_efficiency = 0.8
nozzle_radius_m = 0.5
nozzle_length_m = 2.0
magnetic_field_t = 1.0
coil_current_density_a_m2 = 40000000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.01
radiator_area_m2 = 3000.0
radiator_temperature_k = 1000.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0

[[pulsed_fusion_systems]]
name = "pellet-drive"
mount_position_body_m = [2.0, -0.5, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
reaction = "deuterium-tritium"
working_fluid = "hydrogen"
fuel_mass_per_pulse_kg = 0.000000001
working_fluid_mass_per_pulse_kg = 0.0000001
fusion_gain = 10.0
plasma_coupling_efficiency = 0.9
magnetic_nozzle_efficiency = 0.8
maximum_pulse_frequency_hz = 0.1
pulse_duration_s = 0.01
maximum_charge_power_w = 100000.0
energy_buffer_capacity_pulses = 2
energy_buffer_specific_energy_j_kg = 1000000.0
pulse_system_specific_power_w_kg = 1000000.0
chamber_radius_m = 0.1
chamber_length_m = 0.5
magnetic_field_t = 1.0
coil_current_density_a_m2 = 40000000.0
structure_density_kg_m3 = 2700.0
structure_thickness_m = 0.01
radiator_area_m2 = 10.0
radiator_temperature_k = 1000.0
radiator_emissivity = 0.9
radiator_areal_density_kg_m2 = 8.0
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("fusion TOML parses");
    let vehicle = asset.bake().expect("fusion vehicle bakes");
    assert_eq!(vehicle.fusion_torches.len(), 1);
    assert_eq!(vehicle.pulsed_fusion_systems.len(), 1);
    assert!(vehicle.mass_properties.mass_kg > 3_000.0);
    let torch_mass = vehicle.fusion_torches[0].engine.dry_mass_kg;
    let pulse_mass = vehicle.pulsed_fusion_systems[0].engine.dry_mass_kg;
    let expected_shift = -(DVec3::new(-2.0, 0.5, 0.0) * torch_mass
        + DVec3::new(2.0, -0.5, 0.0) * pulse_mass)
        / (3_000.0 + torch_mass + pulse_mass);
    assert!(
        (DVec3::from_array(vehicle.fusion_torches[0].position_body_m)
            - (DVec3::new(-2.0, 0.5, 0.0) + expected_shift))
            .length()
            < 1e-10
    );
    assert!(
        (DVec3::from_array(vehicle.pulsed_fusion_systems[0].position_body_m)
            - (DVec3::new(2.0, -0.5, 0.0) + expected_shift))
            .length()
            < 1e-10
    );
    let total_first_moment = DVec3::from_array(vehicle.fusion_torches[0].position_body_m)
        * torch_mass
        + DVec3::from_array(vehicle.pulsed_fusion_systems[0].position_body_m) * pulse_mass
        + expected_shift * 3_000.0;
    assert!(total_first_moment.length() < 1e-7);

    let ((force, moment), points) = vehicle
        .fusion_torches_wrench_body_n(&[thessa_sim_core::FusionTorchCommand {
            available_driver_power_w: 20.0e6,
            requested_working_flow_kg_s: 1.0e-4,
        }])
        .expect("torch wrench");
    assert!(force.x > 0.0);
    assert!(moment.is_finite());
    assert_eq!(points.len(), 1);
    let pulse_mount = &vehicle.pulsed_fusion_systems[0];
    let ((pulse_force, pulse_moment), next) = vehicle
        .pulsed_fusion_wrench_body_n_stateful(
            &[(
                thessa_sim_core::PulsedFusionState {
                    pulse_phase_s: pulse_mount.engine.pulse_interval_s - 1.0,
                    stored_driver_energy_j: pulse_mount.engine.driver_energy_per_pulse_j,
                    cumulative_shots: 0,
                },
                thessa_sim_core::PulsedFusionCommand {
                    available_charge_power_w: 100_000.0,
                    armed: true,
                },
            )],
            1.0,
        )
        .expect("pulse wrench");
    assert!(pulse_force.x > 0.0);
    assert!(pulse_moment.is_finite());
    assert_eq!(next[0].1.pulses_fired, 1);
}

#[test]
fn scramjet_cycle_is_authorable_and_analyzer_reaches_hypersonic_rows() {
    let doc = r#"
name = "scramjet-test"
mass_kg = 3000.0
inertia_body_kg_m2 = [[8000.0, 0.0, 0.0], [0.0, 8000.0, 0.0], [0.0, 0.0, 4000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 4.0
chord_m = 1.0

[[jets]]
name = "scramjet"
kind = "jet"
cycle = "scramjet"
fuel = "hydrogen"
intake_area_m2 = 0.5
intake = "ramp"
compressor_ratio = 1.0
turbine_inlet_temp_k = 2300.0
material = "nickel-superalloy"
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("scramjet TOML parses");
    let vehicle = asset.bake().expect("scramjet asset bakes");
    let engine = match &vehicle.jets[0].engine {
        CompiledJet::Air(engine) => engine.as_ref(),
        CompiledJet::Estoc(_) => panic!("scramjet is not ESTOC"),
    };
    assert_eq!(engine.cycle, AirCycle::Scramjet);
    assert_eq!(engine.shaft_reference_power_w, 0.0);

    let atmosphere = AtmosphereConfig::default();
    let rows = analyze_airbreathing(engine, &atmosphere, &[20_000.0], &[0.0, 1.0, 6.0, 8.0], 1.0)
        .expect("scramjet analyzer");
    assert_eq!(rows.len(), 4);
    assert!(rows[0].scramjet_limited);
    assert!(rows[1].scramjet_limited);
    assert!(!rows[2].scramjet_limited);
    assert!(rows[2].thrust_n > 0.0);
    assert!(rows[3].combustion_thermal_limited);
    assert!(!rows[3].drive_limited);
}

#[test]
fn analyzer_options_preserve_explicit_composition() {
    let options = Options::parse(
        [
            "--analyze",
            "--composition",
            "N2/O2/AR/CO2",
            "--source-rpm",
            "2700",
            "--power-takeoff-fraction",
            "0.4",
        ]
        .into_iter()
        .map(String::from),
    )
    .expect("options parse");
    assert!(options.analyze);
    assert_eq!(options.composition, "N2/O2/AR/CO2");
    assert_eq!(options.source_rpm, 2700.0);
    assert_eq!(options.power_takeoff_fraction, 0.4);
    // Default is Thessa air, not a hard-coded Earth scalar.
    let default = Options::parse(std::iter::empty()).expect("defaults parse");
    assert_eq!(default.composition, "N2/O2/AR/CO2");
    assert_eq!(default.source_rpm, 2_400.0);
    assert_eq!(default.power_takeoff_fraction, 0.25);
    assert!(
        Options::parse(
            ["--power-takeoff-fraction", "1.1"]
                .into_iter()
                .map(String::from)
        )
        .is_err()
    );
    // Unknown species are refused instead of becoming Earth air.
    assert!(AtmosphereComposition::parse(&default.composition).is_ok());
    assert!(AtmosphereComposition::parse("XYZ").is_err());
}

#[test]
fn wing_tile_layer_toggle_bakes_mass_and_lumped_thermal_node() {
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "tiled-wing-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[procedural_surfaces]]
name = "wing-right"
span_m = 8.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 2.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 2.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[procedural_surfaces.tile_layer]
tile_size_m = 0.2
thickness_m = 0.01
gap_m = 0.02
density_kg_m3 = 2000.0
specific_heat_j_kg_k = 800.0
emissivity = 0.85
solar_absorptivity = 0.4
max_temp_k = 1500.0
nose_radius_m = 0.05
"#,
    )
    .expect("tiled wing TOML should parse");
    let vehicle = asset.bake().expect("tiled wing should bake");
    // Rectangular 8x2 wing: one panel, 16 m^2 of candidate area per side.
    assert_eq!(vehicle.aero_geometry.panels.len(), 1);
    let pitch_cell_area = 0.5 * 3.0_f64.sqrt() * 0.22_f64.powi(2);
    let tile_face_area = 0.5 * 3.0_f64.sqrt() * 0.2_f64.powi(2);
    let tile_count = 2 * (16.0 / pitch_cell_area).floor() as u64;
    let expected_tile_area = tile_count as f64 * tile_face_area;
    let expected_tiles_kg = expected_tile_area * 0.01 * 2000.0;
    assert!((vehicle.mass_properties.mass_kg - (1000.0 + expected_tiles_kg)).abs() < 1.0e-9);
    // One lumped tile node rides the thermal system with tile material.
    assert_eq!(vehicle.thermal.nodes.len(), 1);
    let tiles = &vehicle.thermal.nodes[0];
    assert_eq!(tiles.name, "wing-right.tiles");
    assert!((tiles.mass_kg - expected_tiles_kg).abs() < 1.0e-9);
    assert_eq!(tiles.max_temp_k, 1500.0);
    assert!((tiles.radiating_area_m2 - expected_tile_area).abs() < 1.0e-9);
    assert!((tiles.solar_exposed_area_m2 - 0.5 * expected_tile_area).abs() < 1.0e-9);
    assert!((tiles.aero_area_m2 - 0.5 * expected_tile_area).abs() < 1.0e-9);
    let state = vehicle.initial_thermal_state().expect("tile state");
    assert_eq!(state.node_temp_k, vec![280.0]);
}

#[test]
fn procedural_surface_bakes_into_merged_panels_and_rebased_controls() {
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "procedural-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 2.0
chord_m = 1.0

[[control_surfaces]]
name = "hand-elevator"
panel_indices = [0]
minimum_deflection_rad = -0.4
maximum_deflection_rad = 0.4
mixing = { pitch = 1.0, roll = 0.0, yaw = 0.0, flap = 0.0, airbrake = 0.0 }

[[procedural_surfaces]]
name = "wing-right"
span_m = 8.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 2.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 2.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "aileron"
span = [0.6, 0.9]
chord = [0.25, 1.0]
hinge_u = 0.25
min_deflection_rad = -0.35
max_deflection_rad = 0.35

[procedural_surfaces.controls.mixing]
pitch = 0.1
roll = 1.0
yaw = -0.2
flap = 0.0
airbrake = 0.0
"#,
    )
    .expect("procedural vehicle TOML should parse");
    let vehicle = asset.bake().expect("procedural asset should bake");
    // One hand panel plus the rectangular compiled wing (no features:
    // splits at 0.6/0.9 with one chord cut inside -> 4 zones).
    assert_eq!(vehicle.aero_geometry.panels.len(), 1 + 4);
    // Hand control keeps index 0; the compiled aileron rebases onto
    // the merged list and owns exactly its region panel.
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.control_surfaces[0].panel_indices, vec![0]);
    assert_eq!(
        vehicle.control_surfaces[0].mixing,
        Some(ControlMixing {
            pitch: 1.0,
            ..ControlMixing::default()
        })
    );
    let aileron = &vehicle.control_surfaces[1];
    assert_eq!(aileron.name, "aileron");
    assert_eq!(aileron.panel_indices.len(), 1);
    assert!(aileron.panel_indices[0] >= 1);
    assert_eq!(
        aileron.mixing,
        Some(ControlMixing {
            pitch: 0.1,
            roll: 1.0,
            yaw: -0.2,
            flap: 0.0,
            airbrake: 0.0,
        })
    );
    let owned = &vehicle.aero_geometry.panels[aileron.panel_indices[0]];
    assert!((owned.area_m2 - 16.0 * 0.3 * 0.75).abs() < 1e-9);
}

#[test]
fn full_procedural_aircraft_merges_wing_and_v_tail() {
    // Hangar-side full-vehicle assembly: a wing plus a canted V-tail
    // half (mount roll through TOML), each with its own controls.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "v-tail-test"
mass_kg = 500.0
inertia_body_kg_m2 = [[500.0, 0.0, 0.0], [0.0, 500.0, 0.0], [0.0, 0.0, 500.0]]

[[procedural_surfaces]]
name = "wing-right"
span_m = 6.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 1.5
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 1.5

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "aileron"
span = [0.5, 0.9]
chord = [0.25, 1.0]
hinge_u = 0.25
min_deflection_rad = -0.4
max_deflection_rad = 0.4
kind = "TrailingEdgeDevice"

[[procedural_surfaces]]
name = "v-tail-right"
span_m = 2.0
origin_body_m = [-2.5, 0.0, 0.2]
mount_roll_rad = 0.7853981633974483

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 1.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 1.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.0
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.0

[[procedural_surfaces.controls]]
name = "ruddervator"
span = [0.3, 0.9]
chord = [0.3, 1.0]
hinge_u = 0.3
min_deflection_rad = -0.4
max_deflection_rad = 0.4

[[procedural_surfaces.folds]]
name = "tip-fold"
station_s = 0.5
axis = [1.0, 0.0, 0.0]
deployed_angle_rad = 0.0
stowed_angle_rad = 0.6
travel_limit_rad = 0.7
deployment_rate_rad_s = 0.1
lock_window_rad = [-0.05, 0.05]
"#,
    )
    .expect("v-tail vehicle TOML should parse");
    let vehicle = asset.bake().expect("v-tail asset should bake");
    // Wing: splits at 0.5/0.9 with a chord cut inside -> 1 + 2 + 1.
    // V-tail: splits at 0.3/0.5/0.9 (fold at 0.5) with chord cuts
    // inside -> 1 + 2 + 2 + 1. Total 10 panels, 2 controls.
    assert_eq!(vehicle.aero_geometry.panels.len(), 10);
    assert_eq!(vehicle.control_surfaces.len(), 2);
    assert_eq!(vehicle.control_surfaces[0].name, "aileron");
    assert_eq!(vehicle.control_surfaces[1].name, "ruddervator");
    // The canted tail panels sit up-out of the body axis.
    let tail_panel = &vehicle.aero_geometry.panels[9];
    assert!(tail_panel.position_body_m.z > 0.5);
    assert!(tail_panel.position_body_m.y > 0.5);
    assert!(tail_panel.lift_axis_body.z > 0.5);
    // Mechanism metadata survives the bake: the fold joint merges
    // under a surface-qualified name, tail panels point at it, and
    // the ruddervator keeps its hinge marker with no parent.
    assert_eq!(vehicle.fold_joints.len(), 1);
    assert_eq!(vehicle.fold_joints[0].name, "v-tail-right.tip-fold");
    assert!((vehicle.fold_joints[0].angle_rad - 0.0).abs() < 1e-12);
    // Operating data rides along: rate, lock window, envelope gate.
    assert!((vehicle.fold_joints[0].deployment_rate_rad_s - 0.1).abs() < 1e-12);
    assert_eq!(vehicle.fold_joints[0].lock_window_rad, (-0.05, 0.05));
    assert_eq!(vehicle.fold_joints[0].max_dynamic_pressure_pa, None);
    let tagged = vehicle
        .aero_geometry
        .panels
        .iter()
        .filter(|panel| panel.fold_index == Some(0))
        .count();
    assert!(tagged > 0);
    assert!(
        vehicle.aero_geometry.panels[..4]
            .iter()
            .all(|panel| panel.fold_index.is_none())
    );
    assert_eq!(
        vehicle.control_surfaces[1].kind,
        thessa_sim_core::ControlKind::Hinge
    );
    assert_eq!(vehicle.control_surfaces[1].parent_index, None);
    // Contact boxes: wing plain plus aileron regions, tail plain,
    // ruddervator, folded, and folded-ruddervator regions.
    assert_eq!(vehicle.collision_geometry.parts.len(), 6);
    assert!(vehicle.collision_geometry.validate().is_ok());
}

#[test]
fn structured_surface_aggregates_mass_and_inertia() {
    // Hand mass 1000 kg plus an 8x2 aluminum wing (264.3648 kg
    // pinned in-crate); inertia sums about the body origin.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "structured-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[procedural_surfaces]]
name = "wing"
span_m = 8.0
origin_body_m = [0.0, 0.0, 0.0]

[procedural_surfaces.planform]
[[procedural_surfaces.planform.stations]]
s = 0.0
x_le = 0.0
x_te = 2.0
[[procedural_surfaces.planform.stations]]
s = 1.0
x_le = 0.0
x_te = 2.0

[procedural_surfaces.bend]
[[procedural_surfaces.bend.stations]]
s = 0.0
z_m = 0.0
[[procedural_surfaces.bend.stations]]
s = 1.0
z_m = 0.0

[procedural_surfaces.sections]
[[procedural_surfaces.sections.stations]]
s = 0.0
incidence_rad = 0.0
thickness_ratio = 0.10
[[procedural_surfaces.sections.stations]]
s = 1.0
incidence_rad = 0.0
thickness_ratio = 0.10

[procedural_surfaces.structure]
skin_gauge_mm = 2.0
spar_depth_fraction = 0.6
spar_web_gauge_mm = 3.0
rib_spacing_m = 0.5
rib_gauge_mm = 1.0
design_limit_lift_n = 12000.0
fuel_box_chord = [0.15, 0.65]
fuel_sump_fraction = 0.03

[procedural_surfaces.structure.skin_material]
name = "Al-7075-T6"
density_kg_m3 = 2810.0
allowable_stress_mpa = 503.0

[procedural_surfaces.structure.spar_material]
name = "Al-7075-T6"
density_kg_m3 = 2810.0
allowable_stress_mpa = 503.0
"#,
    )
    .expect("structured vehicle TOML should parse");
    let vehicle = asset.bake().expect("structured asset should bake");
    // Single-zone wing: every identity below is exact, recomputed
    // from measured values rather than hand arithmetic.
    assert_eq!(vehicle.aero_geometry.panels.len(), 1);
    let wing_mass = vehicle.mass_properties.mass_kg - 1000.0;
    assert!((240.0..260.0).contains(&wing_mass));
    let total = vehicle.mass_properties.mass_kg;
    let com = DVec3::new(-wing_mass / total, 4.0 * wing_mass / total, 0.0);
    assert!(
        (vehicle.aero_geometry.panels[0].center_of_pressure_body_m
            - (DVec3::new(-1.0, 4.0, 0.0) - com))
            .length()
            < 1e-6
    );
    let expected_xy = wing_mass * 1.0 * 4.0 + total * com.x * com.y;
    assert!((vehicle.mass_properties.inertia_body_kg_m2.x_axis.y - expected_xy).abs() < 1e-3);
}

#[test]
fn engine_mass_joins_single_final_com() {
    // Hand 1000 kg at the origin plus one engine at x = 10 m: the
    // reviewer's trap (COM must land at 100*10/1100, not zero).
    // Engine mass is measured back from the baked mount; the shift,
    // recenter, and bake order are what this pins.
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "engine-com-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[engines]]
name = "main"
kind = "liquid"
mount_position_body_m = [10.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "lox-methane"
cycle = "gas-generator"
chamber_pressure_mpa = 12.0
throat_radius_m = 0.15
expansion_ratio = 35.0
nozzle_length_m = 1.8
contour = "bell"
material = "nickel-superalloy"
"#,
    )
    .expect("engine TOML should parse");
    let vehicle = asset.bake().expect("engine asset should bake");
    let engine_mass = vehicle.engines[0].engine.bake_mass_kg();
    assert!(engine_mass > 0.0);
    let total = 1000.0 + engine_mass;
    let com_x = 10.0 * engine_mass / total;
    // Engine station rides the shift; the hand panel at the origin
    // moves to minus the assembly COM.
    assert!((vehicle.engines[0].position_body_m[0] - (10.0 - com_x)).abs() < 1e-9);
    assert!(
        (vehicle.aero_geometry.panels[0].position_body_m - DVec3::new(-com_x, 0.0, 0.0)).length()
            < 1e-9
    );
    // Inertia: hand plus engine point term about the authoring
    // station, minus the single total parallel-axis shift.
    let hand = 1000.0;
    let expected_yy = hand + engine_mass * 10.0 * 10.0 - total * com_x * com_x;
    assert!((vehicle.mass_properties.mass_kg - total).abs() < 1e-9);
    assert!(
        (vehicle.mass_properties.inertia_body_kg_m2.y_axis.y - expected_yy).abs()
            < 1e-6 * expected_yy.abs().max(1.0)
    );
}

#[test]
fn dbg_engine_com() {
    let asset: VehicleAsset = toml::from_str(
        r#"
name = "engine-com-test"
mass_kg = 1000.0
inertia_body_kg_m2 = [[1000.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [0.0, 0.0, 1000.0]]

[[panels]]
position_body_m = [0.0, 0.0, 0.0]
chord_axis_body = [1.0, 0.0, 0.0]
lift_axis_body = [0.0, 0.0, 1.0]
area_m2 = 1.0
chord_m = 1.0

[[engines]]
name = "main"
kind = "liquid"
mount_position_body_m = [10.0, 0.0, 0.0]
thrust_axis_body = [1.0, 0.0, 0.0]
propellant = "lox-methane"
cycle = "gas-generator"
chamber_pressure_mpa = 12.0
throat_radius_m = 0.15
expansion_ratio = 35.0
nozzle_length_m = 1.8
contour = "bell"
material = "nickel-superalloy"
"#,
    )
    .expect("parse");
    eprintln!("engines={}", asset.engines.len());
}
