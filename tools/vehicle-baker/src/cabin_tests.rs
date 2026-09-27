use super::*;
use crate::asset_schema::{AssemblyLinkAsset, resolve_assembly_links};

#[test]
fn fighter_cabin_toml_bakes_seat_mass_and_pilot_authority() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_fighter_cabin.toml"
    ))
    .expect("fighter cabin TOML should parse");
    let vehicle = asset.bake().expect("fighter cabin should bake");

    assert_eq!(vehicle.control_stations.len(), 1);
    assert!(vehicle.control_stations[0].occupied);
    assert_eq!(vehicle.cabin_seats.len(), 1);
    assert_eq!(vehicle.cabin_seats[0].class, CabinSeatClass::Ejection);
    assert_eq!(
        vehicle.cabin_seats[0].role,
        RuntimeCabinSeatRole::FlightCrew
    );
    assert!(vehicle.cabin_seats[0].suited);
    assert_eq!(vehicle.cabin_seats[0].suit_type, CabinSuitType::HoseFed);
    assert_eq!(vehicle.cabin_seats[0].seat_mass_kg, 110.0);
    assert!(vehicle.cabin_seats[0].position_body_m.is_finite());
    assert!(vehicle.control_authority().controllable);
    assert_eq!(
        vehicle.control_authority().reason,
        thessa_sim_core::AuthorityReason::PilotAboard
    );
    assert!(vehicle.mass_properties.mass_kg >= 1_220.0);
}

#[test]
fn regional_cabin_bakes_suit_override_exits_monument_and_empty_stations() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_regional_cabin.toml"
    ))
    .expect("regional cabin TOML should parse");
    let compiled_structure_mass =
        thessa_fuselage::compile_body(&asset.procedural_bodies[0], &BodyCompileOptions::default())
            .expect("regional cabin body should compile")
            .structure
            .expect("regional cabin should have structure")
            .mass_kg;
    let expected_baked_mass = asset.mass_kg + compiled_structure_mass;
    let vehicle = asset.bake().expect("regional cabin should bake");

    assert!((vehicle.mass_properties.mass_kg - expected_baked_mass).abs() < 1e-9);
    assert_eq!(vehicle.cabin_exits.len(), 2);
    assert_eq!(
        vehicle.cabin_exits[0].pair_id,
        vehicle.cabin_exits[1].pair_id
    );
    assert!(vehicle.cabin_exits[0].position_body_m.is_finite());
    assert!(vehicle.cabin_exits[1].position_body_m.is_finite());
    assert_eq!(vehicle.cabin_exits[0].exit_type, CabinExitType::TypeIII);
    assert!(
        (vehicle.cabin_exits[0].position_body_m.y + vehicle.cabin_exits[1].position_body_m.y).abs()
            < 1e-12
    );
    assert_eq!(vehicle.cabin_seats.len(), 13);
    assert!(vehicle.cabin_seats[0].suited);
    assert_eq!(vehicle.cabin_seats[0].suit_mass_kg, 24.0);
    assert_eq!(
        vehicle.cabin_seats[0].suit_type,
        CabinSuitType::SelfContained
    );
    assert_eq!(
        vehicle
            .cabin_seats
            .iter()
            .filter(|seat| seat.role == RuntimeCabinSeatRole::Passenger)
            .count(),
        10
    );
    assert_eq!(
        vehicle
            .cabin_seats
            .iter()
            .filter(|seat| seat.role == RuntimeCabinSeatRole::FlightCrew)
            .count(),
        2
    );
    let recenter_shift_x_m = vehicle.cabins[0].centroid_body_m.x - 5.0;
    assert!(
        (vehicle.cabin_seats[0].position_body_m.x - (1.175 + recenter_shift_x_m)).abs() < 1e-12
    );
    assert!(
        vehicle
            .cabin_seats
            .iter()
            .all(|seat| seat.position_body_m.is_finite())
    );
    assert_eq!(vehicle.cabin_monuments.len(), 1);
    assert!(
        (vehicle.cabin_monuments[0].position_body_m.x - (0.675 + recenter_shift_x_m)).abs() < 1e-12
    );
    assert_eq!(vehicle.cabin_monuments[0].kind, CabinMonumentKind::Galley);
    assert_eq!(vehicle.cabin_monuments[0].mass_kg, 45.0);
    assert!(vehicle.cabin_monuments[0].position_body_m.is_finite());
    assert_eq!(vehicle.control_stations.len(), 2);
    assert!(
        vehicle
            .control_stations
            .iter()
            .all(|station| !station.occupied)
    );
    assert_eq!(
        vehicle.control_authority().reason,
        thessa_sim_core::AuthorityReason::NoPilotNoCore
    );

    let json = serde_json::to_string(&vehicle).expect("baked cabin should serialize");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("baked cabin should deserialize");
    assert_eq!(round_trip.cabin_seats.len(), vehicle.cabin_seats.len());
    assert_eq!(
        round_trip.cabin_monuments.len(),
        vehicle.cabin_monuments.len()
    );
    assert_eq!(
        round_trip.cabin_seats[0].class,
        vehicle.cabin_seats[0].class
    );
    assert_eq!(
        round_trip.cabin_seats[0].suit_type,
        vehicle.cabin_seats[0].suit_type
    );
    assert!(
        (round_trip.cabin_seats[0].position_body_m - vehicle.cabin_seats[0].position_body_m)
            .length()
            < 1e-12
    );
    assert_eq!(
        round_trip.cabin_monuments[0].kind,
        vehicle.cabin_monuments[0].kind
    );
    assert!(
        (round_trip.cabin_monuments[0].position_body_m
            - vehicle.cabin_monuments[0].position_body_m)
            .length()
            < 1e-12
    );
}

#[test]
fn legacy_crew_body_bakes_cabin_and_pilot_station() {
    let asset: VehicleAsset = toml::from_str(include_str!(
        "../../../data/vehicles/example_biprop_crew_body.toml"
    ))
    .expect("crew TOML should parse");
    let vehicle = asset.bake().expect("crew asset should bake");
    assert_eq!(vehicle.tanks.len(), 3);
    assert_eq!(vehicle.cabins.len(), 1);
    assert!(vehicle.cabins[0].air_kg > 0.0);
    assert_eq!(vehicle.control_stations.len(), 1);
    assert!(vehicle.control_stations[0].occupied);
    assert!(vehicle.control_authority().controllable);

    let mut eva = vehicle.clone();
    eva.set_station_occupied(&vehicle.control_stations[0].name, false)
        .expect("debark");
    assert!(!eva.control_authority().controllable);
    assert!(eva.apply_control_inputs(&[]).is_err());
}

#[test]
fn assembled_vehicle_bakes_transforms_and_runtime_connectivity() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_assembly.toml"))
            .expect("assembly TOML should parse");
    assert_eq!(asset.procedural_bodies.len(), 2);
    assert_eq!(asset.assembly.links.len(), 1);
    let links = resolve_assembly_links(&asset.assembly.links).expect("links resolve");
    let compiled = compile_assembly(&asset.procedural_bodies, &links).expect("assembly compiles");
    let vehicle = asset.bake().expect("assembly asset should bake");

    assert_eq!(compiled.root, "stage");
    let stage_mate = compiled.body_transforms[0].transform_point(DVec3::new(4.0, 0.0, 0.0));
    let capsule_mate = compiled.body_transforms[1].transform_point(DVec3::ZERO);
    assert!((stage_mate - capsule_mate).length() < 1e-12);
    assert_eq!(compiled.crew_groups.len(), 1);
    assert_eq!(compiled.crew_groups[0].len(), 2);
    assert!(
        compiled
            .feed_paths
            .iter()
            .any(|path| path.tank == "stage.tank" && path.engine_port == "capsule.engine")
    );

    let runtime = vehicle.assembly.as_ref().expect("baked assembly retained");
    let cabin_volume = runtime
        .volumes
        .iter()
        .find(|volume| volume.name == "capsule.cabin")
        .expect("cabin volume retained in assembly frame");
    assert!(cabin_volume.volume_m3 > 0.0);
    assert!(cabin_volume.centroid_body_m.is_finite());
    let cabin = vehicle
        .cabins
        .iter()
        .find(|cabin| cabin.name == cabin_volume.name)
        .expect("runtime cabin matches assembly volume");
    assert!((cabin.volume_m3 - cabin_volume.volume_m3).abs() < 1e-12);
    let service_cabin = vehicle
        .cabins
        .iter()
        .find(|cabin| cabin.name == "stage.service-bay")
        .expect("service cabin retained");
    assert!((cabin.current_pressure_kpa() - service_cabin.current_pressure_kpa()).abs() < 1e-10);
    assert_eq!(
        runtime.crew_domains().expect("crew domains"),
        vec![vec![
            "stage.service-bay".to_string(),
            "capsule.cabin".to_string()
        ]]
    );
    assert!(
        vehicle
            .assembly_crew_can_pass("stage.service-bay", "capsule.cabin")
            .expect("crew passage query")
    );
    assert!(
        vehicle
            .assembly_cabins_share_air("stage.service-bay", "capsule.cabin")
            .expect("air sharing query")
    );
    assert!(
        vehicle
            .assembly_feed_paths()
            .expect("feed paths")
            .contains(&("stage.tank".into(), "capsule.engine".into()))
    );

    let mut sealed = vehicle.clone();
    sealed
        .set_assembly_hatch_open("stack", false)
        .expect("close connection");
    assert!(
        !sealed
            .assembly_crew_can_pass("stage.service-bay", "capsule.cabin")
            .expect("crew passage query")
    );
    assert!(
        !sealed
            .assembly_cabins_share_air("stage.service-bay", "capsule.cabin")
            .expect("air sharing query")
    );
    assert!(
        !sealed
            .assembly_feed_paths()
            .expect("feed paths")
            .contains(&("stage.tank".into(), "capsule.engine".into()))
    );

    let serialized = serde_json::to_string(&vehicle).expect("vehicle serializes");
    let restored: VehicleDefinition = serde_json::from_str(&serialized).expect("vehicle parses");
    let restored_assembly = restored.assembly.as_ref().expect("assembly round-trips");
    assert_eq!(restored_assembly.root_body, runtime.root_body);
    assert_eq!(restored_assembly.links, runtime.links);
    assert_eq!(restored_assembly.volumes.len(), runtime.volumes.len());
    for (restored, original) in restored_assembly.volumes.iter().zip(&runtime.volumes) {
        assert_eq!(restored.name, original.name);
        assert!((restored.centroid_body_m - original.centroid_body_m).length() < 1e-12);
        assert!((restored.volume_m3 - original.volume_m3).abs() < 1e-12);
        assert_eq!(
            restored.seat_positions_body_m,
            original.seat_positions_body_m
        );
    }
    assert!(vehicle.mass_properties.mass_kg > 1_500.0);

    assert!(
        resolve_assembly_links(&[AssemblyLinkAsset {
            name: "bad".into(),
            parent: "stage".into(),
            child: "capsule.base".into(),
            hatch_open: true,
        }])
        .is_err()
    );
}

#[test]
fn initially_open_hatch_to_dry_volume_vents_cabin_before_mass_bake() {
    fn bake_with_hatch_open(open: bool) -> VehicleDefinition {
        let mut asset: VehicleAsset =
            toml::from_str(include_str!("../../../data/vehicles/example_assembly.toml"))
                .expect("assembly TOML should parse");
        asset.procedural_bodies[1].regions[0].kind = RegionKind::Empty;
        asset.procedural_bodies[1].regions[0].atmosphere = None;
        asset.assembly.links[0].hatch_open = open;
        asset.bake().expect("dry-hatch assembly should bake")
    }

    let sealed = bake_with_hatch_open(false);
    let open = bake_with_hatch_open(true);
    let sealed_cabin = sealed
        .cabins
        .iter()
        .find(|cabin| cabin.name == "stage.service-bay")
        .expect("pressurized stage cabin");
    let open_cabin = open
        .cabins
        .iter()
        .find(|cabin| cabin.name == "stage.service-bay")
        .expect("runtime stage cabin");

    assert!(sealed_cabin.air_kg > 0.0);
    assert_eq!(open_cabin.air_kg, 0.0);
    assert_eq!(
        open_cabin.state,
        thessa_sim_core::CabinPressureState::Vacuum
    );
    assert!(
        (sealed.mass_properties.mass_kg - open.mass_properties.mass_kg - sealed_cabin.air_kg).abs()
            < 1e-9
    );
    open.validate().expect("vented initial assembly is valid");
}
