//! Patched-conic planet endpoints for inter-body transfers.
//!
//! A star-centered Lambert arc between two moons ignores the planet wells
//! at both ends: a moon-only escape burn cannot leave a Nereid-class well
//! (measured: 1.6 AU arrival miss — the craft never escapes, while the
//! broad model prices a 4.3 km/s cruise). The planet patch inserts the
//! missing level: the moon escape (existing helper) flies on a
//! moon-relative v_inf chosen so the planet-relative motion matches the
//! star Lambert asymptote, plus the symmetric arrival pricing. When the
//! endpoint moon orbits the central body directly (lunar tours), there is
//! no intermediate planet and pricing falls back to moon-only —
//! bit-identical to the unpatched path.

use glam::DVec3;
use thessa_sim_core::{BakedEphemeris, BodyId};

/// Walk from `moon` up ONE level to the intermediate well (its direct
/// parent), unless it orbits `central` itself (no patch needed — lunar
/// tours) or the chain is broken (safe fallback to moon-only pricing).
/// Single-level patch: for depth-2 hierarchies (moon/planet/central) the
/// parent frame carries the central-frame asymptote exactly; deeper chains
/// (submoons to the star) approximate the top levels away — full recursive
/// chains are future work, and every real route flies depth ≤ 2.
pub fn planet_of(ephemeris: &BakedEphemeris, central: BodyId, moon: BodyId) -> Option<BodyId> {
    if moon == central {
        return None;
    }
    match ephemeris.body(moon).ok()?.parent {
        None => None,
        Some(parent) if parent == central => None,
        Some(parent) => Some(parent),
    }
}

/// Moon-relative v_inf vector achieving planet-relative outgoing asymptote
/// `v_inf_planet` (all vectors inertial-central frame). Patched-conic: at
/// the moon's orbital radius the planet-relative escape speed is
/// `sqrt(|v_inf|^2 + 2 mu_p/r_m)`, directed along the outgoing asymptote;
/// subtract the moon's planet-relative velocity. `None` on degenerate
/// inputs (no asymptote, no well, no orbit).
pub fn planet_escape_moon_vinf(
    v_inf_planet_mps: DVec3,
    moon_pos_rel_planet_m: DVec3,
    moon_vel_rel_planet_mps: DVec3,
    planet_mu: f64,
) -> Option<DVec3> {
    // NaN-safe positivity spelled without negated comparisons
    // (clippy::neg_cmp_op_on_partial_ord): NaN fails is_finite first.
    if !v_inf_planet_mps.is_finite() {
        return None;
    }
    let v_inf = v_inf_planet_mps.length();
    let orbit_radius = moon_pos_rel_planet_m.length();
    if !v_inf.is_finite() || v_inf <= 0.0 {
        return None;
    }
    if !orbit_radius.is_finite() || orbit_radius <= 0.0 {
        return None;
    }
    if !planet_mu.is_finite() || planet_mu <= 0.0 {
        return None;
    }
    let escape_speed = (v_inf * v_inf + 2.0 * planet_mu / orbit_radius).sqrt();
    if !escape_speed.is_finite() {
        return None;
    }
    let required_planet_rel = v_inf_planet_mps.normalize() * escape_speed;
    Some(required_planet_rel - moon_vel_rel_planet_mps)
}

/// Rendezvous match magnitude at the arrival moon, planet-well AND
/// moon-well inclusive (mirrors the lunar arrival semantics, where the
/// exact propagation falls into the moon well): planet-relative craft
/// speed at the moon's orbit from the incoming asymptote, differenced with
/// the moon, then moon-well fall-in added by energy. `None` on degenerate
/// inputs.
pub fn planet_arrival_match_mag(
    v_inf_planet_mps: DVec3,
    moon_pos_rel_planet_m: DVec3,
    moon_vel_rel_planet_mps: DVec3,
    planet_mu: f64,
    moon_mu: f64,
    park_radius_m: f64,
) -> Option<f64> {
    if !v_inf_planet_mps.is_finite() || !moon_pos_rel_planet_m.is_finite() {
        return None;
    }
    let v_inf = v_inf_planet_mps.length();
    let orbit_radius = moon_pos_rel_planet_m.length();
    if !v_inf.is_finite() || v_inf <= 0.0 {
        return None;
    }
    if !orbit_radius.is_finite() || orbit_radius <= 0.0 {
        return None;
    }
    if !planet_mu.is_finite() || planet_mu <= 0.0 {
        return None;
    }
    if !park_radius_m.is_finite() || park_radius_m <= 0.0 {
        return None;
    }
    if !moon_mu.is_finite() || moon_mu < 0.0 {
        return None;
    }
    let craft_speed = (v_inf * v_inf + 2.0 * planet_mu / orbit_radius).sqrt();
    if !craft_speed.is_finite() {
        return None;
    }
    let craft_planet_rel = v_inf_planet_mps.normalize() * craft_speed;
    let v_inf_moon = (craft_planet_rel - moon_vel_rel_planet_mps).length();
    let magnitude = (v_inf_moon * v_inf_moon + 2.0 * moon_mu / park_radius_m).sqrt();
    if !magnitude.is_finite() {
        return None;
    }
    Some(magnitude)
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::{BakedBody, BodyId, KeplerOrbit};

    /// Three-level hierarchy: star -> planet -> moon -> submoon.
    fn chain_system() -> BakedEphemeris {
        let orbit = |mu: f64, a: f64| {
            KeplerOrbit::new(mu, a, 0.0, 0.0, 0.0, 0.0, 0.0).expect("valid test orbit")
        };
        BakedEphemeris::new(
            "TEST_CHAIN",
            vec![
                BakedBody::fixed(BodyId(0), "star", 1.0e16, 1.0e8),
                BakedBody::orbital(
                    BodyId(1),
                    "planet",
                    1.0e14,
                    1.0e7,
                    BodyId(0),
                    orbit(1.0e16, 2.0e10),
                ),
                BakedBody::orbital(
                    BodyId(2),
                    "moon",
                    1.0e12,
                    1.0e6,
                    BodyId(1),
                    orbit(1.0e14, 1.0e9),
                ),
                BakedBody::orbital(
                    BodyId(3),
                    "sub",
                    1.0e10,
                    1.0e5,
                    BodyId(2),
                    orbit(1.0e12, 1.0e8),
                ),
            ],
        )
        .expect("valid test system")
    }

    #[test]
    fn planet_walk_finds_intermediate_well() {
        let ephemeris = chain_system();
        // Moon -> planet; submoon -> moon (its intermediate well).
        assert_eq!(planet_of(&ephemeris, BodyId(0), BodyId(2)), Some(BodyId(1)));
        assert_eq!(planet_of(&ephemeris, BodyId(0), BodyId(3)), Some(BodyId(2)));
        // Direct orbiter, central itself, unknown body: no patch.
        assert_eq!(planet_of(&ephemeris, BodyId(0), BodyId(1)), None);
        assert_eq!(planet_of(&ephemeris, BodyId(0), BodyId(0)), None);
        assert_eq!(planet_of(&ephemeris, BodyId(0), BodyId(9)), None);
    }

    #[test]
    fn escape_vinf_hits_required_planet_speed() {
        let mu_p = 1.2e17;
        let r_m = 1.0e9;
        let v_inf_p = DVec3::new(5_000.0, 0.0, 0.0);
        // Moon moving WITH the asymptote lends its orbital speed; moving
        // against it costs extra. (Perpendicular cases are symmetric by
        // construction, so the test uses parallel motion.)
        let with = planet_escape_moon_vinf(v_inf_p, DVec3::X * r_m, DVec3::X * 3_000.0, mu_p)
            .expect("patched");
        let against = planet_escape_moon_vinf(v_inf_p, DVec3::X * r_m, DVec3::X * -3_000.0, mu_p)
            .expect("patched");
        // Planet-relative motion after moon escape matches the required
        // escape speed by construction.
        let required = (25.0e6 + 2.0 * mu_p / r_m).sqrt();
        assert!(((DVec3::X * 3_000.0 + with).length() - required).abs() < 1.0e-6);
        // Misaligned moon (moving against the asymptote) costs more.
        assert!(against.length() > with.length());
    }

    #[test]
    fn arrival_match_includes_both_wells() {
        let mag = planet_arrival_match_mag(
            DVec3::X * 5_000.0,
            DVec3::X * 1.0e9,
            DVec3::ZERO,
            1.2e17,
            1.0e12,
            1.0e6,
        )
        .expect("patched");
        // Planet well dominates: craft arrives at ~16.3 km/s planet-rel.
        let craft = (25.0e6_f64 + 2.4e8).sqrt();
        assert!((mag - (craft * craft + 2.0e6).sqrt()).abs() < 1.0);
        assert!(mag > craft);
    }

    #[test]
    fn patch_degenerate_inputs_are_none() {
        assert!(planet_escape_moon_vinf(DVec3::ZERO, DVec3::X, DVec3::Y, 1.0e14).is_none());
        assert!(planet_escape_moon_vinf(DVec3::X, DVec3::ZERO, DVec3::Y, 1.0e14).is_none());
        assert!(planet_escape_moon_vinf(DVec3::X, DVec3::X, DVec3::Y, 0.0).is_none());
        assert!(
            planet_arrival_match_mag(DVec3::ZERO, DVec3::X, DVec3::Y, 1.0e14, 1.0e12, 1.0e6)
                .is_none()
        );
    }
}
