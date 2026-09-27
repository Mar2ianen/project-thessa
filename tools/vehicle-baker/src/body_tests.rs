use super::*;

#[test]
fn example_body_asset_bakes_panels_tanks_and_contact() {
    let asset: VehicleAsset =
        toml::from_str(include_str!("../../../data/vehicles/example_body.toml"))
            .expect("body TOML should parse");
    assert_eq!(asset.procedural_bodies.len(), 1);
    let vehicle = asset.bake().expect("body asset should bake");
    // Strip panels (two per axial zone), one feed-pipeline tank from
    // the tank region, per-segment contact parts, and mass above the
    // 500 kg hand structure (hull plus tank dry plus propellant).
    assert!(!vehicle.aero_geometry.panels.is_empty());
    assert_eq!(vehicle.aero_geometry.panels.len() % 2, 0);
    assert_eq!(vehicle.tanks.len(), 1);
    assert!(vehicle.tanks[0].tank.full_propellant_kg > 0.0);
    assert_eq!(vehicle.collision_geometry.parts.len(), 4);
    assert!(vehicle.collision_geometry.validate().is_ok());
    assert!(vehicle.mass_properties.mass_kg > 500.0);
    // Body strips mute the shared side-force path (their lateral
    // answer arrives through the orthogonal strips' lift path).
    for panel in &vehicle.aero_geometry.panels {
        assert_eq!(panel.side_force_scale, 0.0);
    }
    let json = serde_json::to_string(&vehicle).expect("vehicle JSON should serialize");
    let round_trip: VehicleDefinition =
        serde_json::from_str(&json).expect("vehicle JSON should deserialize");
    assert_eq!(
        round_trip.aero_geometry.panels.len(),
        vehicle.aero_geometry.panels.len()
    );
}
