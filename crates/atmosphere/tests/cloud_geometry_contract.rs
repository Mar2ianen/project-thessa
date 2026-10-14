//! Independent cloud-geometry regression tests.
//!
//! These exercise the public production CPU geometry API, not a reimplementation
//! of its ray/sphere equations. In particular, the exact planet-surface
//! boundary is intentionally tested: an epsilon-offset camera alone misses
//! the t_planet == 0 failure mode.
//!
//! Run:
//!   cargo test -p thessa-atmosphere --test cloud_geometry_contract -- --nocapture
//!
//! The surface-inward cases are expected to be RED on the reviewed base commit.
//! Do not remove or weaken them to obtain a green test run.

use glam::DVec3;
use thessa_atmosphere::clouds::{bake_clouds, clip_to_planet, intersect_shell, CloudSlab};

const RADIUS_M: f64 = 3_200_000.0;

fn thessa_deck() -> CloudSlab {
    CloudSlab::new(
        RADIUS_M,
        RADIUS_M + 25_600.0,
        RADIUS_M + 54_720.0,
    )
    .expect("physically valid Thessa cloud envelope")
}

fn visible_segment(origin: DVec3, direction: DVec3, center: DVec3) -> Option<(f64, f64)> {
    let (t0, t1) = intersect_shell(origin, direction, center, thessa_deck())?;
    clip_to_planet(t0, t1, origin, direction, center, RADIUS_M)
}

#[test]
fn planet_surface_inward_rays_must_never_reveal_far_side_clouds() {
    let center = DVec3::ZERO;
    let origin = DVec3::new(RADIUS_M, 0.0, 0.0);

    // Includes a nearly grazing ray. Every direction has a strictly inward
    // radial component, hence intersects solid planet before far-side clouds.
    for tangential in [0.0_f64, 0.25, 0.75, 0.999] {
        let radial = -(1.0 - tangential * tangential).sqrt();
        let direction = DVec3::new(radial, tangential, 0.0);
        let raw = intersect_shell(origin, direction, center, thessa_deck())
            .expect("far-side cloud envelope exists geometrically");

        assert!(
            visible_segment(origin, direction, center).is_none(),
            "solid planet must occlude cloud interval {raw:?} for surface-origin inward ray {direction:?}"
        );
    }
}

#[test]
fn planet_occlusion_must_not_change_at_the_exact_surface_boundary() {
    let center = DVec3::ZERO;
    let inward = DVec3::NEG_X;

    // This is a boundary-value regression, not a claim about temporal
    // continuity: a ray starting on or inside the surface and pointing
    // inward must never see the far hemisphere.
    for radial_offset_m in [-0.25_f64, 0.0, 0.25] {
        let origin = DVec3::new(RADIUS_M + radial_offset_m, 0.0, 0.0);
        assert!(
            visible_segment(origin, inward, center).is_none(),
            "far-side cloud leak at surface offset {radial_offset_m} m"
        );
    }
}

#[test]
fn planet_occlusion_is_invariant_under_world_origin_translation() {
    let center = DVec3::new(17_000_000.0, -9_000_000.0, 4_000_000.0);
    let surface = center + DVec3::new(RADIUS_M, 0.0, 0.0);
    let inward = DVec3::NEG_X;

    assert!(
        visible_segment(surface, inward, center).is_none(),
        "planet occlusion cannot depend on world-space origin"
    );
}

#[test]
fn outward_surface_ray_keeps_the_near_cloud_deck() {
    let center = DVec3::ZERO;
    let origin = DVec3::new(RADIUS_M, 0.0, 0.0);
    let outward = DVec3::X;

    let (t0, t1) = visible_segment(origin, outward, center)
        .expect("outward-looking surface observer must see the near cloud deck");
    assert!((t0 - 25_600.0).abs() < 1e-5, "unexpected near cloud entry {t0}");
    assert!((t1 - 54_720.0).abs() < 1e-5, "unexpected near cloud exit {t1}");
}

#[test]
fn cloud_bake_is_resolution_covariant_at_identical_uv_coordinates() {
    // A metamorphic test of the actual bake implementation: doubling both
    // texture dimensions preserves samples at the exact same normalized UV.
    // This makes no assumption about the artistic noise distribution.
    const W: u32 = 64;
    const H: u32 = 32;
    let seed = 0xC10D_0000_0000_0001_u64;
    let small = bake_clouds(seed, W, H, 0.55, 1.0, (1.0, 1.0));
    let large = bake_clouds(seed, 2 * W, 2 * H, 0.55, 1.0, (1.0, 1.0));

    for (x, y) in [
        (0, 0),
        (W - 1, 0),
        (0, H - 1),
        (W - 1, H - 1),
        (W / 2, H / 2),
        (13, 7),
        (47, 26),
    ] {
        let small_offset = ((y * W + x) * 4) as usize;
        let large_offset = (((2 * y) * (2 * W) + 2 * x) * 4) as usize;
        assert_eq!(
            &small[small_offset..small_offset + 4],
            &large[large_offset..large_offset + 4],
            "bake varies at same UV ({x}/{W}, {y}/{H}) when only resolution changes"
        );
    }
}
