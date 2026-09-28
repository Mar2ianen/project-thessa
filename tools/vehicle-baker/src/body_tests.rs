use super::*;

#[test]
fn blunt_body_bakes_heat_shield_mount_with_aft_normal() {
    let doc = r#"
name = "shield-test-capsule"
mass_kg = 500.0
inertia_body_kg_m2 = [
  [500.0, 0.0, 0.0],
  [0.0, 500.0, 0.0],
  [0.0, 0.0, 500.0],
]

[[procedural_bodies]]
name = "capsule"
origin_body_m = [0.0, 0.0, 0.0]

[[procedural_bodies.stations]]
x_m = 0.0
half_width_m = 1.0
top_height_m = 1.0
bottom_height_m = 1.0
top_exponent = 2.0
bottom_exponent = 2.0
offset_y_m = 0.0
offset_z_m = 0.0

[[procedural_bodies.stations]]
x_m = 2.0
half_width_m = 1.0
top_height_m = 1.0
bottom_height_m = 1.0
top_exponent = 2.0
bottom_exponent = 2.0
offset_y_m = 0.0
offset_z_m = 0.0

[[procedural_bodies.heat_shields]]
name = "aft-shield"
end = "aft"
thickness_mm = 50.0

[procedural_bodies.heat_shields.material]
density_kg_m3 = 1800.0
yield_strength_pa = 100000000.0
max_wall_temp_k = 1800.0
"#;
    let asset: VehicleAsset = toml::from_str(doc).expect("shield TOML parses");
    let vehicle = asset.bake().expect("shield vehicle should bake");
    assert_eq!(vehicle.heat_shields.len(), 1);
    let shield = &vehicle.heat_shields[0];
    assert_eq!(shield.name, "aft-shield");
    assert!((shield.diameter_m - 2.0).abs() < 1.0e-9);
    assert_eq!(shield.normal_body_m, -DVec3::X);
    // Ablator disc mass pi·1²·0.05·1800 rides the hull aggregate, not the
    // mount: total mass grows, mounts stay mass-free.
    assert!(vehicle.mass_properties.mass_kg > 500.0 + 200.0);
    // The mount also lands in the shared aero geometry as a blunt disc,
    // so the common flow solution (not a side channel) flies it.
    assert_eq!(vehicle.aero_geometry.blunt_discs.len(), 1);
    let disc = &vehicle.aero_geometry.blunt_discs[0];
    assert!((disc.area_m2 - std::f64::consts::PI).abs() < 1.0e-9);
    assert_eq!(disc.normal_body_m, -DVec3::X);
    assert!((disc.position_body_m - shield.position_body_m).length() < 1.0e-12);
}

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
