//! Volumetric cloud-slab march math (spec `docs/14` section 10).
//!
//! Backend-neutral core for the `[clouds] volumetric` path: a low-deck
//! spherical slab around a planet, marched in SI metres with `f64`. The same
//! functions validate the WGSL approximation (`assets/shaders/cloud_volume.wgsl`)
//! and answer CPU queries (tests, benchmarks). No Bevy, wgpu, or texture
//! types here: density sampling is a caller-provided closure, so the GPU
//! samples its coverage bake while the CPU oracle samples an analytic field.
//!
//! ```text
//! planet center/radius + slab (bottom/top m)
//!     -> intersect_shell (front slab interval)
//!     -> clip_to_planet (terminate at the surface)
//!     -> integrate_slab (Beer-Lambert march over a density closure)
//! ```
//!
//! Warp-safe: every function evaluates directly from its inputs, never from
//! accumulated ticks. Quality (step count) changes representation accuracy,
//! never the slab geometry or the density field.

use glam::DVec3;

/// Spherical cloud slab: absolute planet-centric radii in metres.
/// `top_m > bottom_m > planet_radius_m` always; constructors enforce it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CloudSlab {
    pub planet_radius_m: f64,
    pub bottom_m: f64,
    pub top_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlabError {
    NonFinite,
    NonPositiveRadius,
    TopNotAboveBottom,
    BottomNotAboveSurface,
}

impl std::fmt::Display for SlabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "cloud slab radii must be finite"),
            Self::NonPositiveRadius => write!(f, "cloud slab radii must be positive"),
            Self::TopNotAboveBottom => write!(f, "cloud slab top must be above bottom"),
            Self::BottomNotAboveSurface => {
                write!(f, "cloud slab bottom must be above the planet surface")
            }
        }
    }
}

impl std::error::Error for SlabError {}

impl CloudSlab {
    // Validity guards must reject NaN: `!(x > 0.0)` catches NaN while the lint's
    // suggested `x <= 0.0` would accept it. The negated form is deliberate.
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn new(planet_radius_m: f64, bottom_m: f64, top_m: f64) -> Result<Self, SlabError> {
        for v in [planet_radius_m, bottom_m, top_m] {
            if !v.is_finite() {
                return Err(SlabError::NonFinite);
            }
        }
        if planet_radius_m <= 0.0 || bottom_m <= 0.0 || top_m <= 0.0 {
            return Err(SlabError::NonPositiveRadius);
        }
        if !(top_m > bottom_m) {
            return Err(SlabError::TopNotAboveBottom);
        }
        if !(bottom_m > planet_radius_m) {
            return Err(SlabError::BottomNotAboveSurface);
        }
        Ok(Self {
            planet_radius_m,
            bottom_m,
            top_m,
        })
    }

    pub fn thickness_m(&self) -> f64 {
        self.top_m - self.bottom_m
    }
}

/// Vertical density profile over normalized slab height `h01` in `[0,1]`:
/// zero at both faces, one mid-slab. Sine puff, cheap to mirror in WGSL.
pub fn vertical_profile(h01: f64) -> f64 {
    if !h01.is_finite() {
        return 0.0;
    }
    // Exact faces read exactly zero: sin(pi) is ~1e-16, not 0, and the pow
    // would lift it into a 1e-12 skirt that never fully clears.
    if h01 <= 0.0 || h01 >= 1.0 {
        return 0.0;
    }
    (std::f64::consts::PI * h01).sin().powf(0.75)
}

/// Ray/sphere interval for one radius; `None` on miss. Values may be
/// negative (sphere behind the ray); the caller selects the front part.
// Validity guards must reject NaN: `!(x >= 0.0)` catches NaN while the lint's
// suggested `x < 0.0` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn sphere_interval(origin: DVec3, dir: DVec3, center: DVec3, radius: f64) -> Option<(f64, f64)> {
    let oc = origin - center;
    let b = oc.dot(dir);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if !(disc >= 0.0) {
        return None;
    }
    let sq = disc.sqrt();
    Some((-b - sq, -b + sq))
}

/// Front slab interval `(t0, t1)` with `t1 > max(t0, 0)`: the nearest ray
/// segment inside `[bottom, top]`. Handles cameras outside, inside the slab,
/// and below the deck (looking up or down). `None` when the ray never enters
/// the slab ahead of the origin.
pub fn intersect_shell(
    origin: DVec3,
    dir: DVec3,
    center: DVec3,
    slab: CloudSlab,
) -> Option<(f64, f64)> {
    if !dir.length_squared().is_finite() || dir.length_squared() <= 1e-12 {
        return None;
    }
    let d = dir.normalize_or_zero();
    if d.length_squared() < 0.5 {
        return None;
    }
    let (t0, t1) = sphere_interval(origin, d, center, slab.top_m)?;
    // Front part of the outer ball; the hollow core is carved below.
    let (mut f0, mut f1) = (t0.max(0.0), t1);
    if f1 <= f0 {
        return None;
    }
    // Carve the hollow core out of the FRONT interval only: a core piece
    // behind the camera must not consume the slab ahead (measured: an
    // in-slab observer lost the whole deck to its own inner sphere).
    // Only the nearest surviving slab piece is returned; a far-side piece
    // past a clear sub-deck gap is accepted representation error (the
    // shell fallback has the same single-layer alpha).
    if let Some((i0, i1)) = sphere_interval(origin, d, center, slab.bottom_m)
        && i1 > f0
        && i0 < f1
    {
        if i0 <= f0 {
            // Core covers the start (observer under the deck): the slab
            // resumes at the core exit.
            f0 = f0.max(i1);
        } else {
            // Core stands ahead mid-interval: stop at its entry.
            f1 = f1.min(i0);
        }
    }
    if f1 <= f0 {
        return None;
    }
    Some((f0, f1))
}

/// Terminate a slab interval at the planet surface: clouds behind the limb
/// never contribute. Returns `None` when the surface stands in front of the
/// whole interval (deck hidden behind the planet).
// Validity guards must reject NaN: `!(x > 0.0)` catches NaN while the lint's
// suggested `x <= 0.0` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn clip_to_planet(
    t0: f64,
    t1: f64,
    origin: DVec3,
    dir: DVec3,
    center: DVec3,
    planet_radius_m: f64,
) -> Option<(f64, f64)> {
    if !(t1 > t0) || !planet_radius_m.is_finite() || !(planet_radius_m > 0.0) {
        return None;
    }
    let d = dir.normalize_or_zero();
    let Some((p0, p1)) = sphere_interval(origin, d, center, planet_radius_m) else {
        return Some((t0, t1));
    };
    // Origin inside the planet: below the surface, no sky visible.
    if p0 < 0.0 && p1 > 0.0 {
        return None;
    }
    if p0 > 0.0 {
        if p0 <= t0 {
            return None;
        }
        if p0 < t1 {
            return Some((t0, p0));
        }
    }
    Some((t0, t1))
}

/// Beer-Lambert march over `[t0, t1]`: `steps` midpoint samples of
/// `density_at(t)` (0..~1, non-finite reads as clear), scaled by
/// `extinction_per_m`. Returns `(transmittance, optical_depth)`.
/// `steps` is a budget, never physics: 1..=256, clamped.
// Validity guards must reject NaN: `!(x > y)` catches NaN while the lint's
// suggested `x <= y` would accept it. The negated form is deliberate.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn integrate_slab(
    density_at: impl Fn(f64) -> f64,
    t0: f64,
    t1: f64,
    steps: u32,
    extinction_per_m: f64,
) -> (f64, f64) {
    let steps = steps.clamp(1, 256) as usize;
    if !(t1 > t0) || !t0.is_finite() || !t1.is_finite() {
        return (1.0, 0.0);
    }
    let k = if extinction_per_m.is_finite() {
        extinction_per_m.max(0.0)
    } else {
        0.0
    };
    if k == 0.0 {
        return (1.0, 0.0);
    }
    let dt = (t1 - t0) / steps as f64;
    let mut transmittance = 1.0;
    let mut depth = 0.0;
    for i in 0..steps {
        let t = t0 + (i as f64 + 0.5) * dt;
        let d = density_at(t);
        let d = if d.is_finite() {
            d.clamp(0.0, 2.0)
        } else {
            0.0
        };
        let absorb = (-k * d * dt).exp();
        transmittance *= absorb;
        depth += k * d * dt;
        if transmittance < 1e-4 {
            transmittance = 0.0;
            break;
        }
    }
    (transmittance, depth)
}

/// Representation-error proxy for a march budget: metres per step. Finer
/// steps resolve thinner density gradients; the slab geometry and the field
/// itself do not change with quality. Pinned by a regression test.
pub fn march_step_m(slab: CloudSlab, steps: u32) -> f64 {
    slab.thickness_m() / steps.clamp(1, 256) as f64
}

// ---------------------------------------------------------------------------
// Schneider-style shaping (Horizon Zero Dawn cloudscapes): coverage remap,
// type-dependent height gradient, Beer-Powder lighting, dual-lobe phase.
// The WGSL march mirrors these exactly; the CPU versions below are the
// oracle and the unit-test anchor.

/// Coverage remap of a 0..1 weather field: values below `floor` read clear,
/// above remap to 0..1. Cleans the noise floor so thin haze never films the
/// whole disc (the bake carries the main shaping; this only cuts the skirt).
pub fn remap_coverage(base01: f64, floor: f64) -> f64 {
    if !base01.is_finite() {
        return 0.0;
    }
    if !(floor.is_finite() && floor > 0.0) {
        return base01.clamp(0.0, 1.0);
    }
    ((base01 - floor) / (1.0 - floor)).clamp(0.0, 1.0)
}

/// Cloud-type height gradient over normalized slab height and type
/// `type01` (0 = flat stratus lens, 0.5 = bottom-heavy cumulus, 1 = solid
/// towering core). Exact faces read zero.
pub fn type_gradient(h01: f64, type01: f64) -> f64 {
    if !h01.is_finite() || h01 <= 0.0 || h01 >= 1.0 {
        return 0.0;
    }
    let t = type01.clamp(0.0, 1.0);
    let bottom = 0.30 - 0.28 * t;
    let top_start = 0.55 + 0.20 * t;
    let top_keep = 0.25 + 0.50 * t;
    smoothstep(bottom, bottom + 0.20, h01)
        * (1.0 - (1.0 - top_keep) * smoothstep(top_start, 1.0, h01))
}

/// Legacy two-family gradient kept for the shell-era oracle tests; new code
/// uses [`type_gradient`].
pub fn height_gradient(h01: f64, high_deck: bool) -> f64 {
    type_gradient(h01, if high_deck { 0.15 } else { 0.5 })
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Detail erosion (Schneider subtract form): carve `erosion01` out of the
/// density so edges and tops break into puffs instead of smoothing over.
/// Never adds where the field is clear.
pub fn erode_density(density01: f64, erosion01: f64) -> f64 {
    if !density01.is_finite() {
        return 0.0;
    }
    let d = density01.clamp(0.0, 1.0);
    let e = if erosion01.is_finite() {
        erosion01.clamp(0.0, 0.999)
    } else {
        0.0
    };
    ((d - e) / (1.0 - e).max(1e-6)).clamp(0.0, 1.0)
}

/// Beer-Powder sun throughput for light depth `ld` (extinction-scaled):
/// Beer attenuates through the volume, Powder peaks at mid depth so puff
/// sides facing the sun glow while deep cores stay dark and thin edges go
/// to the ambient term. Zero at zero depth by construction.
pub fn beer_powder(light_depth: f64) -> f64 {
    if !light_depth.is_finite() {
        return 0.0;
    }
    let ld = light_depth.max(0.0);
    (-ld).exp() * (1.0 - (-ld * 2.0).exp())
}

/// Dual-lobe scattering phase: strong forward lobe (silver lining around
/// the sun) plus a weak backward fill. Wraps [`crate::scattering::hg_phase`].
pub fn dual_lobe_phase(cos_theta: f64) -> f64 {
    let c = cos_theta.clamp(-1.0, 1.0);
    0.75 * crate::scattering::hg_phase(0.55, c) + 0.25 * crate::scattering::hg_phase(-0.3, c)
}

/// Height-graded ambient: dark blue-grey base to bright top, scaled by
/// running view transmittance so deep samples darken hard (crevice drama).
/// Top color rides with the sky state (tuned hot so thin edges glow).
pub fn ambient_rgb(
    h01: f64,
    view_transmittance: f64,
    bottom_rgb: [f64; 3],
    top_rgb: [f64; 3],
) -> [f64; 3] {
    let h = h01.clamp(0.0, 1.0);
    let t = view_transmittance.clamp(0.0, 1.0);
    let lift = 0.10 + 0.90 * t;
    [
        (bottom_rgb[0] + (top_rgb[0] - bottom_rgb[0]) * h) * lift,
        (bottom_rgb[1] + (top_rgb[1] - bottom_rgb[1]) * h) * lift,
        (bottom_rgb[2] + (top_rgb[2] - bottom_rgb[2]) * h) * lift,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn thessa_slab() -> CloudSlab {
        // Thessa R = 3200 km, low deck ~1.008..1.014 (visual placement,
        // mirrors the client deck table fractions).
        CloudSlab::new(3_200_000.0, 3_200_000.0 * 1.008, 3_200_000.0 * 1.014).unwrap()
    }

    #[test]
    fn slab_rejects_bad_geometry() {
        assert_eq!(
            CloudSlab::new(100.0, 90.0, 110.0).unwrap_err(),
            SlabError::BottomNotAboveSurface
        );
        assert_eq!(
            CloudSlab::new(100.0, 110.0, 105.0).unwrap_err(),
            SlabError::TopNotAboveBottom
        );
        assert_eq!(
            CloudSlab::new(f64::NAN, 110.0, 120.0).unwrap_err(),
            SlabError::NonFinite
        );
        assert_eq!(
            CloudSlab::new(0.0, 110.0, 120.0).unwrap_err(),
            SlabError::NonPositiveRadius
        );
    }

    #[test]
    fn profile_is_zero_at_faces_one_mid() {
        assert_eq!(vertical_profile(0.0), 0.0);
        assert_eq!(vertical_profile(1.0), 0.0);
        assert!((vertical_profile(0.5) - 1.0).abs() < 1e-12);
        assert_eq!(vertical_profile(f64::NAN), 0.0);
        assert!(vertical_profile(0.25) > vertical_profile(0.05));
    }

    #[test]
    fn orbit_ray_crosses_full_slab() {
        let slab = thessa_slab();
        // Camera well above the deck looking straight down.
        let t = intersect_shell(
            DVec3::new(0.0, 0.0, 4_000_000.0),
            DVec3::new(0.0, 0.0, -1.0),
            DVec3::ZERO,
            slab,
        )
        .expect("hits deck");
        assert!((t.1 - t.0 - slab.thickness_m()).abs() < 1.0, "{t:?}");
    }

    #[test]
    fn miss_returns_none() {
        let slab = thessa_slab();
        assert!(
            intersect_shell(
                DVec3::new(0.0, 0.0, 4_000_000.0),
                DVec3::new(0.0, 1.0, 0.0),
                DVec3::ZERO,
                slab
            )
            .is_none()
        );
        assert!(intersect_shell(DVec3::ZERO, DVec3::ZERO, DVec3::ZERO, slab).is_none());
    }

    #[test]
    fn inside_slab_marches_from_zero() {
        let slab = thessa_slab();
        let mid = (slab.bottom_m + slab.top_m) / 2.0;
        let t = intersect_shell(
            DVec3::new(0.0, 0.0, mid),
            DVec3::new(0.0, 0.0, 1.0),
            DVec3::ZERO,
            slab,
        )
        .expect("inside sees up");
        assert_eq!(t.0, 0.0);
        assert!((t.1 - (slab.top_m - mid)).abs() < 1.0, "{t:?}");
    }

    #[test]
    fn below_deck_looking_up_enters_slab() {
        let slab = thessa_slab();
        // Surface observer looking at the zenith.
        let t = intersect_shell(
            DVec3::new(0.0, 0.0, slab.planet_radius_m),
            DVec3::new(0.0, 0.0, 1.0),
            DVec3::ZERO,
            slab,
        )
        .expect("surface sees deck overhead");
        assert!(
            (t.0 - (slab.bottom_m - slab.planet_radius_m)).abs() < 1.0,
            "{t:?}"
        );
        assert!((t.1 - t.0 - slab.thickness_m()).abs() < 1.0, "{t:?}");
    }

    #[test]
    fn planet_clip_hides_deck_behind_limb() {
        let slab = thessa_slab();
        // Horizontal ray at deck altitude tangent-ish: passes the slab, then
        // the planet occults the far piece. Use a ray that grazes through
        // the slab toward the limb from outside.
        let origin = DVec3::new(0.0, -4_000_000.0, 3_250_000.0);
        let dir = DVec3::new(0.0, 1.0, -0.05).normalize();
        if let Some((t0, t1)) = intersect_shell(origin, dir, DVec3::ZERO, slab) {
            let clipped = clip_to_planet(t0, t1, origin, dir, DVec3::ZERO, slab.planet_radius_m);
            // Either unclipped (misses the globe) or shortened, never longer.
            if let Some((c0, c1)) = clipped {
                assert!(c1 - c0 <= t1 - t0 + 1e-6);
                assert_eq!(c0, t0);
            }
        }
        // Straight-down orbit ray is never clipped (planet below the slab).
        let (t0, t1) = intersect_shell(
            DVec3::new(0.0, 0.0, 4_000_000.0),
            DVec3::new(0.0, 0.0, -1.0),
            DVec3::ZERO,
            slab,
        )
        .unwrap();
        let clipped = clip_to_planet(
            t0,
            t1,
            DVec3::new(0.0, 0.0, 4_000_000.0),
            DVec3::new(0.0, 0.0, -1.0),
            DVec3::ZERO,
            slab.planet_radius_m,
        )
        .unwrap();
        assert!((clipped.1 - clipped.0 - (t1 - t0)).abs() < 1e-6);

        // Ray looking at a slab behind the planet is fully occluded (None).
        let occluded = clip_to_planet(
            10_000_000.0,
            11_000_000.0,
            DVec3::new(0.0, 0.0, 4_000_000.0),
            DVec3::new(0.0, 0.0, -1.0),
            DVec3::ZERO,
            slab.planet_radius_m,
        );
        assert_eq!(occluded, None);
    }

    #[test]
    fn uniform_slab_matches_beer_lambert() {
        // Analytic reference: T = exp(-k*d*L). The midpoint march must land
        // within 2% at 16 steps for a uniform field (regression envelope).
        let (t, _) = integrate_slab(|_| 1.0, 0.0, 1000.0, 16, 0.002);
        let reference = (-0.002 * 1000.0_f64).exp();
        assert!((t - reference).abs() < 0.02, "{t} vs {reference}");
    }

    #[test]
    fn empty_field_stays_transparent() {
        let (t, depth) = integrate_slab(|_| 0.0, 0.0, 5000.0, 8, 0.01);
        assert_eq!(t, 1.0);
        assert_eq!(depth, 0.0);
        // Non-finite densities read as clear, never as NaN out.
        let (t, _) = integrate_slab(|_| f64::NAN, 0.0, 5000.0, 8, 0.01);
        assert_eq!(t, 1.0);
        // Zero extinction is vacuum regardless of density.
        let (t, _) = integrate_slab(|_| 1.0, 0.0, 5000.0, 8, 0.0);
        assert_eq!(t, 1.0);
    }

    #[test]
    fn coarse_steps_converge_to_fine_reference() {
        // 128-step reference vs 8-step budget on a shaped profile: bounded
        // representation error, pinned so quality cuts stay honest.
        let field = |t: f64| vertical_profile(t / 1000.0);
        let (fine, _) = integrate_slab(field, 0.0, 1000.0, 128, 0.004);
        let (coarse, _) = integrate_slab(field, 0.0, 1000.0, 8, 0.004);
        assert!((fine - coarse).abs() < 0.06, "{fine} vs {coarse}");
    }

    #[test]
    fn march_step_pins_budget() {
        let slab = thessa_slab();
        assert!(march_step_m(slab, 8) > march_step_m(slab, 16));
        assert!((march_step_m(slab, 8) - slab.thickness_m() / 8.0).abs() < 1e-9);
    }

    #[test]
    fn integrate_is_deterministic() {
        let f = |t: f64| vertical_profile(t / 500.0);
        assert_eq!(
            integrate_slab(f, 0.0, 500.0, 12, 0.003),
            integrate_slab(f, 0.0, 500.0, 12, 0.003)
        );
    }

    #[test]
    fn coverage_remap_cleans_the_floor() {
        assert_eq!(remap_coverage(0.02, 0.04), 0.0);
        assert_eq!(remap_coverage(1.0, 0.04), 1.0);
        assert_eq!(remap_coverage(0.5, 0.0), 0.5);
        assert_eq!(remap_coverage(f64::NAN, 0.04), 0.0);
        // Monotone: denser weather never thins the deck.
        let mut prev = 0.0;
        for i in 0..=20 {
            let d = remap_coverage(i as f64 / 20.0, 0.04);
            assert!(d >= prev - 1e-12, "{d} after {prev}");
            prev = d;
        }
    }

    #[test]
    fn height_gradient_shapes_type_and_seals_faces() {
        assert_eq!(height_gradient(0.0, false), 0.0);
        assert_eq!(height_gradient(1.0, false), 0.0);
        assert_eq!(height_gradient(0.0, true), 0.0);
        assert_eq!(height_gradient(1.0, true), 0.0);
        // Cumulus carries its mass low, cirrus rides high and wispy.
        assert!(height_gradient(0.3, false) > height_gradient(0.3, true));
        assert!(height_gradient(0.85, true) > height_gradient(0.15, true));
        assert!(height_gradient(0.5, false) > 0.5);
    }

    #[test]
    fn type_gradient_grades_stratus_to_tower() {
        assert_eq!(type_gradient(0.0, 0.5), 0.0);
        assert_eq!(type_gradient(1.0, 0.5), 0.0);
        // Stratus is a thin lens, towers are solid cores (compared high,
        // where the lens has faded but the tower stands).
        assert!(type_gradient(0.8, 0.0) < type_gradient(0.8, 1.0));
        assert!(type_gradient(0.9, 1.0) > type_gradient(0.9, 0.0) + 0.3);
        // Mid type still domes; monotone in height up to the peak.
        assert!(type_gradient(0.4, 0.5) > type_gradient(0.2, 0.5));
    }

    #[test]
    fn erosion_carves_but_never_adds() {
        assert_eq!(erode_density(0.0, 0.5), 0.0);
        assert_eq!(erode_density(1.0, 0.0), 1.0);
        assert_eq!(erode_density(f64::NAN, 0.5), 0.0);
        // Full erosion clears everything short of overcast cores.
        assert!(erode_density(0.5, 0.9) < 0.1);
        assert_eq!(erode_density(1.0, 0.9), 1.0);
    }

    #[test]
    fn beer_powder_peaks_mid_depth() {
        assert_eq!(beer_powder(0.0), 0.0);
        assert_eq!(beer_powder(f64::NAN), 0.0);
        let edge = beer_powder(0.15);
        let mid = beer_powder(0.6);
        let core = beer_powder(4.0);
        // Puff sides glow, deep cores and clear air do not.
        assert!(mid > edge && mid > core, "{edge} {mid} {core}");
        assert!(core < 0.05, "{core}");
    }

    #[test]
    fn dual_phase_prefers_forward_scatter() {
        let fwd = dual_lobe_phase(1.0);
        let side = dual_lobe_phase(0.0);
        let back = dual_lobe_phase(-1.0);
        // Silver lining dominates; the backward lobe fills but never wins.
        assert!(fwd > side && fwd > back, "{fwd} {side} {back}");
        assert!(back > 0.0 && side > 0.0);
    }

    #[test]
    fn ambient_grades_dark_base_to_bright_top() {
        let bottom = [0.30, 0.34, 0.42];
        let top = [0.90, 0.96, 1.10];
        let low = ambient_rgb(0.0, 1.0, bottom, top);
        let high = ambient_rgb(1.0, 1.0, bottom, top);
        assert!(high[0] > low[0] && high[1] > low[1] && high[2] > low[2]);
        // Deep samples darken: the same altitude reads darker inside a core.
        let shallow = ambient_rgb(0.5, 1.0, bottom, top);
        let deep = ambient_rgb(0.5, 0.1, bottom, top);
        assert!(deep[0] < shallow[0] * 0.6);
    }
}
