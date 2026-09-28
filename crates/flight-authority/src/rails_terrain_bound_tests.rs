use super::*;
use crate::canonical_launch_setup;
use thessa_sim_core::SystemConfig;

fn authority() -> (BakedEphemeris, FlightAuthority) {
    let config: SystemConfig =
        toml::from_str(include_str!("../../../data/system.toml")).expect("system");
    let ephemeris = config.bake().expect("bake");
    let reference_body = ephemeris.body_id("thessa").expect("thessa");
    let flight = FlightAuthority::new(&ephemeris, reference_body).expect("authority");
    (ephemeris, flight)
}

#[test]
fn obstacle_ceiling_declared_without_field() {
    // No field, no ghost batches: the recipe ceiling still refuses low
    // coasts over unmapped mountains, and the live field maximum can
    // only tighten it (the baker clamps into recipe bounds).
    let (ephemeris, mut flight) = authority();
    assert!(flight.terrain_field.is_none());
    assert_eq!(
        flight
            .obstacle_report([0.0, 1.0, 0.0], 100.0)
            .expect("terrain-free report lookup"),
        None
    );
    assert_eq!(flight.recipe_max_elevation_m, 12_000.0);
    assert_eq!(flight.rails_terrain_bound_m(), 12_000.0);
    let (field, sites) = canonical_launch_setup(&ephemeris).expect("launch setup");
    flight.initialize_world_site(field, sites[0], &ephemeris);
    let live = flight.rails_terrain_bound_m();
    assert!((0.0..=12_000.0).contains(&live), "live bound {live}");
}

#[test]
fn terrain_track_certification_returns_coverage_evidence() {
    let (ephemeris, mut flight) = authority();
    let (field, sites) = canonical_launch_setup(&ephemeris).expect("launch setup");
    flight.initialize_world_site(field, sites[0], &ephemeris);
    let track = vec![
        (SimTime::EPOCH, flight.state.position_inertial_m),
        (SimTime::EPOCH, flight.state.position_inertial_m),
    ];
    let coverage = flight
        .certify_terrain_track(&ephemeris, &track)
        .expect("covered terrain track");
    assert!(coverage.obstacles.covers_track());
    assert_eq!(coverage.obstacles.coverage.track_points, 2);
    assert!(coverage.obstacles.coverage.sample_cover_radius_m >= 0.0);
    // Clearance is reported as evidence; the sub-grid withstand gate is
    // deliberately a separate follow-up proof.
    assert!(coverage.min_obstacle_clearance_m.is_finite());
}

#[test]
fn site_obstacle_report_uses_the_loaded_canonical_field() {
    let (ephemeris, mut flight) = authority();
    let (field, sites) = canonical_launch_setup(&ephemeris).expect("launch setup");
    flight.initialize_world_site(field, sites[0], &ephemeris);
    let report = flight
        .obstacle_report(sites[0], 300.0)
        .expect("site report")
        .expect("loaded field report");
    assert_eq!(report.center_dir, sites[0]);
    assert_eq!(report.radius_m, 300.0);
    assert!(report.max_height_m >= report.min_height_m);
    assert!(report.max_slope.is_finite());
}
