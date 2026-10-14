//! Regression contracts for the portable plume AABB ray helper.
//!
//! These tests intentionally exercise cases omitted by d8e792c.
//! They may fail until the production implementation is corrected.
//! Run: cargo test -p thessa-plume-core --test ray_aabb_contract

use thessa_plume_core::{PlumeBound, ray_intersect_slab};

const MIN: [f64; 3] = [10.0, -5.0, 20.0];
const MAX: [f64; 3] = [15.0, 5.0, 30.0];

#[test]
fn slab_intersection_is_invariant_under_positive_ray_direction_scaling() {
    // Direction normalization must not affect WHETHER a parametric ray hits.
    // Only the parameter t scales inversely. Both rays intersect the same
    // physical points x=10 and x=15.
    let unit = ray_intersect_slab(MIN, MAX, [0.0, 0.0, 25.0], [1.0, 0.0, 0.0])
        .expect("unit direction must hit");
    let scale = 1.0e-13_f64;
    let scaled = ray_intersect_slab(MIN, MAX, [0.0, 0.0, 25.0], [scale, 0.0, 0.0])
        .expect("small nonzero direction must still hit");

    for (actual, expected) in [(scaled.0 * scale, unit.0), (scaled.1 * scale, unit.1)] {
        assert!(
            (actual - expected).abs() < 1e-10,
            "ray direction scaling changed the physical intersection: {actual} != {expected}"
        );
    }
}

#[test]
fn slab_intersection_rejects_nonfinite_inputs() {
    for (origin, direction) in [
        ([f64::NAN, 0.0, 25.0], [1.0, 0.0, 0.0]),
        ([0.0, 0.0, 25.0], [f64::NAN, 0.0, 0.0]),
        ([0.0, 0.0, 25.0], [f64::INFINITY, 0.0, 0.0]),
        ([f64::INFINITY, 0.0, 25.0], [-1.0, 0.0, 0.0]),
    ] {
        assert_eq!(
            ray_intersect_slab(MIN, MAX, origin, direction),
            None,
            "nonfinite ray must not produce an actionable intersection: origin={origin:?}, direction={direction:?}",
        );
    }
}

#[test]
fn plume_ray_intersection_inherits_slab_contract() {
    let plume = PlumeBound {
        length_m: 25.0,
        max_radius_m: 3.0,
    };
    let hit = plume.ray_intersect_aabb([0.0, 0.0, -5.0], [0.0, 0.0, 1.0e-13]);
    assert!(
        hit.is_some(),
        "the plume must not disappear when the ray direction is scaled"
    );
    assert_eq!(
        plume.ray_intersect_aabb([f64::NAN, 0.0, -5.0], [0.0, 0.0, 1.0]),
        None,
        "invalid plume rays should not be accepted"
    );
}
