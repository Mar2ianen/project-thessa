//! Independent regression gate for inner-deck tangencies and invalid planet rays.
//!
//! Only public production functions are called. The geometry fixture uses
//! *exact integer* coordinates to make the inner-sphere discriminant exactly
//! zero in binary floating-point, rather than relying on epsilon guesses.
//!
//! Run:
//!   cargo test -p thessa-atmosphere --test cloud_tangent_contract -- --nocapture
//!
//! On reviewed HEAD 039d26f, the three exact-tangency tests and invalid-ray
//! tests are intentionally RED. Positive controls guard against overcorrecting.
//! The production WGSL fragment tests inner-sphere crossings with
//! `disc_bot > 0.0`; a zero-discriminant tangent is not a hollow-core crossing.

use glam::DVec3;
use thessa_atmosphere::clouds::{clip_to_planet, intersect_shell, CloudSlab};

fn slab() -> CloudSlab {
    CloudSlab::new(1.0, 5.0, 6.0).expect("well-formed concentric volume")
}

fn assert_near(actual: f64, expected: f64, what: &str) {
    assert!(
        actual.is_finite() && (actual - expected).abs() < 1e-10,
        "{what}: actual={actual:.16} expected={expected:.16} error={:.4e}",
        (actual - expected).abs()
    );
}

#[test]
fn exact_inner_sphere_tangent_from_outside_does_not_erase_back_half_of_cloud() {
    // Ray x=5, y=-10+t has minimal distance exactly 5 at t=10.
    // The hollow sphere r<5 is NEVER entered: the ray stays entirely
    // inside the cloud band from its near outer crossing to its far one.
    //
    // Outer sphere r=6: (5)^2 + (-10+t)^2 = 36,
    // so t = 10 +/- sqrt(11).
    let origin = DVec3::new(5.0, -10.0, 0.0);
    let (entry, exit) = intersect_shell(origin, DVec3::Y, DVec3::ZERO, slab())
        .expect("tangent ray crosses positive-thickness cloud band");

    assert_near(entry, 10.0 - 11.0_f64.sqrt(), "near cloud entry");
    assert_near(exit, 10.0 + 11.0_f64.sqrt(), "far cloud exit after tangent");
}

#[test]
fn exact_tangent_from_within_cloud_must_not_stop_at_empty_core_boundary() {
    // Starts between inner and outer spheres: radius sqrt(34) is 5..6.
    // Still never enters inner sphere r<5, only touches it at t=3.
    // The entire forward interval until outer sphere exit is cloud.
    let origin = DVec3::new(5.0, -3.0, 0.0);
    let (entry, exit) = intersect_shell(origin, DVec3::Y, DVec3::ZERO, slab())
        .expect("observer in cloud and tangent to core still sees cloud ahead");

    assert_near(entry, 0.0, "in-cloud entry");
    assert_near(exit, 3.0 + 11.0_f64.sqrt(), "in-cloud outer exit");
}

#[test]
fn exact_tangent_is_invariant_under_translation_of_world_origin() {
    let center = DVec3::new(10_000.0, -20_000.0, 30_000.0);
    let origin = center + DVec3::new(5.0, -10.0, 0.0);
    let (entry, exit) = intersect_shell(origin, DVec3::Y, center, slab())
        .expect("translated cloud tangent must remain visible");

    assert_near(entry, 10.0 - 11.0_f64.sqrt(), "translated near entry");
    assert_near(exit, 10.0 + 11.0_f64.sqrt(), "translated far exit");
}


#[test]
fn thessa_scale_exact_tangent_preserves_entire_visible_cloud_chord() {
    // Canonical Thessa radius and low-cloud envelope in metres. All integer
    // squares below 2^53 yield an exactly zero inner-sphere discriminant:
    // no epsilon-based near-tangency ambiguity is involved.
    let planet_m = 3_200_000.0;
    let bottom_m = 3_225_600.0;
    let top_m = 3_254_720.0;
    let slab = CloudSlab::new(planet_m, bottom_m, top_m).unwrap();
    let origin = DVec3::new(bottom_m, -2_000_000.0, 0.0);
    let (entry, exit) = intersect_shell(origin, DVec3::Y, DVec3::ZERO, slab)
        .expect("canonical tangent ray must traverse the outer cloud layer");
    let half_chord = (top_m * top_m - bottom_m * bottom_m).sqrt();
    assert_near(entry, 2_000_000.0 - half_chord, "Thessa tangent entry");
    assert_near(exit, 2_000_000.0 + half_chord, "Thessa tangent exit");
}

#[test]
fn ray_just_outside_inner_sphere_keeps_complete_outer_interval() {
    // Control: a nearby ray that MISSES r=5 by 0.01 does not hit the
    // hollow core, hence the expected full outer interval is visible.
    let x = 5.01_f64;
    let (entry, exit) = intersect_shell(
        DVec3::new(x, -10.0, 0.0),
        DVec3::Y,
        DVec3::ZERO,
        slab(),
    )
    .unwrap();
    let half_chord = (36.0 - x * x).sqrt();

    assert_near(entry, 10.0 - half_chord, "missed-core near entry");
    assert_near(exit, 10.0 + half_chord, "missed-core far exit");
}

#[test]
fn ray_crossing_actual_hollow_core_still_stops_at_near_core_entry() {
    // Control: x=4 penetrates the r=5 hollow sphere. A single-segment
    // renderer intentionally stops before the near hollow boundary.
    let (entry, exit) = intersect_shell(
        DVec3::new(4.0, -10.0, 0.0),
        DVec3::Y,
        DVec3::ZERO,
        slab(),
    )
    .unwrap();

    assert_near(entry, 10.0 - 20.0_f64.sqrt(), "core-crossing outer entry");
    assert_near(exit, 7.0, "core-crossing inner entry");
}

#[test]
fn planet_clip_rejects_zero_length_direction_instead_of_fail_open() {
    // Contract: an invalid ray must not be treated as clear of planetary
    // occlusion. intersect_shell rejects zero rays, but clip_to_planet
    // currently normalizes them to zero and returns the interval unchanged.
    assert_eq!(
        clip_to_planet(
            1.0,
            2.0,
            DVec3::new(2.0, 0.0, 0.0),
            DVec3::ZERO,
            DVec3::ZERO,
            1.0,
        ),
        None,
        "invalid zero direction cannot justify exposing clouds",
    );
}

#[test]
fn planet_clip_rejects_nonfinite_origin_instead_of_fail_open() {
    assert_eq!(
        clip_to_planet(
            1.0,
            2.0,
            DVec3::new(f64::NAN, 0.0, 0.0),
            DVec3::Y,
            DVec3::ZERO,
            1.0,
        ),
        None,
        "nonfinite geometry must not bypass planet occlusion",
    );
}
