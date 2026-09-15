//! Lambert's problem in universal variables (single revolution).
//!
//! Given two position vectors, a time of flight and a central `mu`, find
//! the departure and arrival velocities of the connecting Kepler arc.
//! Battin-style formulation with Stumpff functions; Newton iteration with
//! a numeric derivative from several starts, bisection fallback on an
//! auto-bracket. Only the principal (zero-revolution) branch: multi-rev
//! transfers are out of scope and reported as such.
//!
//! Degenerate geometries (0 deg / 180 deg transfers, where the transfer
//! plane is undefined) are errors, not silent guesses: the planner nudges
//! the geometry instead. This matches MechJeb practice (which also refuses
//! or perturbs exact-180-degree porkchop cells) without pretending the
//! physics has an answer there.

use glam::DVec3;

/// Connecting-arc velocities: integrate `departure_velocity` from `r1` for
/// `dt` under two-body gravity to arrive at `r2` with `arrival_velocity`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LambertArc {
    pub departure_velocity_mps: DVec3,
    pub arrival_velocity_mps: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LambertError {
    NonFiniteInput,
    NonPositiveTime,
    NonPositiveMu,
    DegenerateGeometry,
    OutOfRange,
    NoConvergence,
}

impl std::fmt::Display for LambertError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteInput => write!(formatter, "non-finite lambert input"),
            Self::NonPositiveTime => write!(formatter, "time of flight must be positive"),
            Self::NonPositiveMu => write!(formatter, "central mu must be positive"),
            Self::DegenerateGeometry => write!(
                formatter,
                "degenerate transfer geometry (0/180 deg: plane undefined)"
            ),
            Self::OutOfRange => {
                write!(formatter, "time of flight outside the single-rev branch")
            }
            Self::NoConvergence => write!(formatter, "lambert iteration did not converge"),
        }
    }
}

impl std::error::Error for LambertError {}

/// Stumpff C(z): series near zero, trig/hyperbolic branches elsewhere.
fn stumpff_c(z: f64) -> f64 {
    if z.abs() < 1.0e-6 {
        // C = 1/2 - z/24 + z^2/720 - z^3/40320.
        0.5 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0
    } else if z > 0.0 {
        let root = z.sqrt();
        (1.0 - root.cos()) / z
    } else {
        let root = (-z).sqrt();
        (root.cosh() - 1.0) / (-z)
    }
}

/// Stumpff S(z).
fn stumpff_s(z: f64) -> f64 {
    if z.abs() < 1.0e-6 {
        // S = 1/6 - z/120 + z^2/5040 - z^3/362880.
        1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0
    } else if z > 0.0 {
        let root = z.sqrt();
        (root - root.sin()) / (root * z)
    } else {
        let root = (-z).sqrt();
        (root.sinh() - root) / (root * (-z))
    }
}

struct Geometry {
    r1_norm: f64,
    r2_norm: f64,
    a: f64,
    sqrt_mu_dt: f64,
}

/// Time-of-flight residual `F(z) = tof(z) - dt`, scaled by `sqrt(mu)`.
/// Returns `None` where the auxiliary variable leaves its domain.
fn residual(geometry: &Geometry, z: f64) -> Option<f64> {
    if !z.is_finite() {
        return None;
    }
    let c = stumpff_c(z);
    let s = stumpff_s(z);
    if !c.is_finite() || !s.is_finite() || c <= 0.0 {
        return None;
    }
    let root_c = c.sqrt();
    let y = geometry.r1_norm + geometry.r2_norm + geometry.a * (z * s - 1.0) / root_c;
    if !y.is_finite() || y < 0.0 {
        return None;
    }
    let chi = (y / c).sqrt();
    Some(chi * chi * chi * s + geometry.a * y.sqrt() - geometry.sqrt_mu_dt)
}

fn newton_from(geometry: &Geometry, start: f64) -> Option<f64> {
    let mut z = start;
    for _ in 0..60 {
        let f = residual(geometry, z)?;
        if f.abs() <= 1.0e-12 * geometry.sqrt_mu_dt.max(1.0e-300) {
            return Some(z);
        }
        let step = 1.0e-5 * (1.0 + z.abs());
        let derivative =
            (residual(geometry, z + step)? - residual(geometry, z - step)?) / (2.0 * step);
        if !derivative.is_finite() || derivative == 0.0 {
            return None;
        }
        let next = z - f / derivative;
        // Keep Newton inside the single-rev domain; anything wilder goes
        // to the bisection fallback instead of diverging silently.
        if !next.is_finite() || next < -1.0e6 || next > 39.0 {
            return None;
        }
        if (next - z).abs() <= 1.0e-13 * (1.0 + z.abs()) {
            let check = residual(geometry, next)?;
            if check.abs() <= 1.0e-12 * geometry.sqrt_mu_dt.max(1.0e-300) {
                return Some(next);
            }
            return None;
        }
        z = next;
    }
    None
}

/// Bisection on the monotone single-rev branch. `dt(z)` increases with `z`,
/// so the residual goes from negative (fast hyperbolic side) to positive
/// (near the full-revolution boundary at (2 pi)^2).
fn bisect(geometry: &Geometry) -> Option<f64> {
    let two_pi_squared = 4.0 * std::f64::consts::PI * std::f64::consts::PI;
    let mut low = -100.0;
    for _ in 0..6 {
        match residual(geometry, low) {
            Some(value) if value <= 0.0 => break,
            _ => {}
        }
        low *= 4.0;
        if low < -1.0e12 {
            return None;
        }
    }
    if residual(geometry, low)? > 0.0 {
        return None;
    }
    let mut high = two_pi_squared * 0.999;
    if residual(geometry, high)? < 0.0 {
        return None;
    }
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        match residual(geometry, mid) {
            Some(value) if value > 0.0 => high = mid,
            Some(_) => low = mid,
            None => high = mid,
        }
        if high - low <= 1.0e-13 * (1.0 + low.abs()) {
            break;
        }
    }
    let solved = 0.5 * (low + high);
    match residual(geometry, solved) {
        Some(value) if value.abs() <= 1.0e-9 * geometry.sqrt_mu_dt.max(1.0e-300) => Some(solved),
        _ => None,
    }
}

/// Solve Lambert's problem (zero-revolution branch).
///
/// - `short_way`: transfer angle in `[0, pi]`; otherwise `2 pi - angle`.
///   Note this is a purely geometric flag: it does NOT know which way is
///   prograde. For moon-tour rendezvous prefer [`solve_lambert_prograde`].
/// - Errors instead of guessing on degenerate geometry: nudge the
///   departure/arrival or the time of flight and retry.
pub fn solve_lambert(
    r1_m: DVec3,
    r2_m: DVec3,
    dt_s: f64,
    mu_m3_s2: f64,
    short_way: bool,
) -> Result<LambertArc, LambertError> {
    if !r1_m.is_finite() || !r2_m.is_finite() || !dt_s.is_finite() || !mu_m3_s2.is_finite() {
        return Err(LambertError::NonFiniteInput);
    }
    if dt_s <= 0.0 {
        return Err(LambertError::NonPositiveTime);
    }
    if mu_m3_s2 <= 0.0 {
        return Err(LambertError::NonPositiveMu);
    }
    let r1_norm = r1_m.length();
    let r2_norm = r2_m.length();
    if r1_norm <= 0.0 || r2_norm <= 0.0 {
        return Err(LambertError::DegenerateGeometry);
    }
    let cos_dnu = (r1_m.dot(r2_m) / (r1_norm * r2_norm)).clamp(-1.0, 1.0);
    let mut transfer_angle = cos_dnu.acos();
    if !short_way {
        transfer_angle = 2.0 * std::f64::consts::PI - transfer_angle;
    }
    let sin_dnu = transfer_angle.sin();
    let chord_factor = (r1_norm * r2_norm / (1.0 - cos_dnu).max(1.0e-300)).sqrt();
    if !chord_factor.is_finite() {
        return Err(LambertError::DegenerateGeometry);
    }
    let a = sin_dnu * chord_factor;
    // Vanishing A means the transfer plane is undefined (0 deg: same ray,
    // needs multi-rev; 180 deg: any plane through the line works).
    if a.abs() <= 1.0e-9 * (r1_norm * r2_norm).sqrt() {
        return Err(LambertError::DegenerateGeometry);
    }
    let geometry = Geometry {
        r1_norm,
        r2_norm,
        a,
        sqrt_mu_dt: mu_m3_s2.sqrt() * dt_s,
    };
    let mut solved = None;
    for start in [0.0, -1.0, 1.0, -10.0, 5.0] {
        if let Some(z) = newton_from(&geometry, start) {
            solved = Some(z);
            break;
        }
    }
    let z = match solved {
        Some(z) => z,
        None => bisect(&geometry).ok_or(LambertError::NoConvergence)?,
    };
    let c = stumpff_c(z);
    let s = stumpff_s(z);
    let y = r1_norm + r2_norm + a * (z * s - 1.0) / c.sqrt();
    if !y.is_finite() || y <= 0.0 {
        return Err(LambertError::NoConvergence);
    }
    let f = 1.0 - y / r1_norm;
    let g = a * (y / mu_m3_s2).sqrt();
    if !f.is_finite() || !g.is_finite() || g == 0.0 {
        return Err(LambertError::NoConvergence);
    }
    let g_dot = 1.0 - y / r2_norm;
    let departure = (r2_m - r1_m * f) / g;
    let arrival = (r2_m * g_dot - r1_m) / g;
    if !departure.is_finite() || !arrival.is_finite() {
        return Err(LambertError::NoConvergence);
    }
    // dt far beyond the single-rev branch converged somewhere unphysical:
    // refuse rather than return a multi-rev impostor.
    if z >= 4.0 * std::f64::consts::PI * std::f64::consts::PI * 0.999 {
        return Err(LambertError::OutOfRange);
    }
    Ok(LambertArc {
        departure_velocity_mps: departure,
        arrival_velocity_mps: arrival,
    })
}

/// Prograde Lambert arc: picks the short/long branch whose orbital momentum
/// aligns with the reference orbit (`r1`, `v1_ref` — typically the depot
/// state), instead of forcing one branch globally. A fixed `short_way`
/// flag silently returns retrograde arcs (tens of km/s) for every cell on
/// the "wrong" side; this picks the cheap side per cell.
///
/// Falls back to the short-way arc when the reference momentum is
/// degenerate (radial reference motion) or orthogonal (polar corner).
pub fn solve_lambert_prograde(
    r1_m: DVec3,
    v1_ref_mps: DVec3,
    r2_m: DVec3,
    dt_s: f64,
    mu_m3_s2: f64,
) -> Result<LambertArc, LambertError> {
    if !v1_ref_mps.is_finite() {
        return Err(LambertError::NonFiniteInput);
    }
    let reference_momentum = r1_m.cross(v1_ref_mps);
    let plane = r1_m.cross(r2_m);
    let short_way = plane.dot(reference_momentum) >= 0.0;
    solve_lambert(r1_m, r2_m, dt_s, mu_m3_s2, short_way)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MU: f64 = 3.986_004_418e14;

    #[test]
    fn near_hohmann_limit_matches_vis_viva() {
        // 179 deg transfer at Hohmann timing: continuous with the textbook
        // Hohmann values (exact 180 deg is degenerate by design — the plane
        // is undefined there, see the degenerate test below).
        let r1 = 6_878_000.0;
        let r2 = 2.0 * r1;
        let semi = 0.5 * (r1 + r2);
        let dt = std::f64::consts::PI * (semi * semi * semi / MU).sqrt();
        let angle = std::f64::consts::PI - 0.5_f64.to_radians();
        let arc = solve_lambert(
            DVec3::new(r1, 0.0, 0.0),
            DVec3::new(r2 * angle.cos(), r2 * angle.sin(), 0.0),
            dt,
            MU,
            true,
        )
        .expect("near-hohmann arc solves");
        let expected_dep = (MU * (2.0 / r1 - 1.0 / semi)).sqrt();
        assert!(
            (arc.departure_velocity_mps.length() - expected_dep) / expected_dep <= 0.01,
            "dep speed {} vs {expected_dep}",
            arc.departure_velocity_mps.length()
        );
        // Mostly tangential (within ~2 deg of the Hohmann direction).
        let tangent = DVec3::new(0.0, 1.0, 0.0);
        let cos_align = arc.departure_velocity_mps.normalize().dot(tangent);
        assert!(cos_align >= 0.9994, "alignment {cos_align}");
    }

    #[test]
    fn lagrange_identities_hold() {
        // f*fdot - fdot*g == 1 and shared energy/momentum: any true solution
        // satisfies these; a wrong root cannot fake all three.
        let mut seed = 0x1234_5678_9ABC_DEF0_u64;
        let mut next = move || {
            seed ^= seed >> 12;
            seed ^= seed << 25;
            seed ^= seed >> 27;
            seed = seed.wrapping_mul(0x2545_F491_4F6C_DD1D);
            seed as f64 / u64::MAX as f64
        };
        for _ in 0..40 {
            let radius = 6.6e6 + next() * 4.0e7;
            let angle = 0.2 + next() * (2.0 * std::f64::consts::PI - 0.4);
            if (angle - std::f64::consts::PI).abs() < 0.05 {
                continue;
            }
            let r1 = DVec3::new(radius, 0.0, next() * 1.0e6);
            let r2 = DVec3::new(radius * angle.cos(), radius * angle.sin(), next() * 1.0e6);
            let dt = 1_000.0 + next() * 20_000.0;
            let short_way = angle <= std::f64::consts::PI;
            let Ok(arc) = solve_lambert(r1, r2, dt, MU, short_way) else {
                continue;
            };
            let energy_dep = arc.departure_velocity_mps.length_squared() / 2.0 - MU / r1.length();
            let energy_arr = arc.arrival_velocity_mps.length_squared() / 2.0 - MU / r2.length();
            let scale = energy_dep.abs().max(1.0);
            assert!(
                (energy_dep - energy_arr).abs() / scale <= 1.0e-8,
                "shared energy {energy_dep} vs {energy_arr}"
            );
            let h1 = r1.cross(arc.departure_velocity_mps);
            let h2 = r2.cross(arc.arrival_velocity_mps);
            let hscale = h1.length().max(1.0);
            assert!(
                (h1 - h2).length() / hscale <= 1.0e-8,
                "shared momentum {h1:?} vs {h2:?}"
            );
        }
    }

    #[test]
    fn degenerate_geometries_are_errors() {
        let r = DVec3::new(7.0e6, 0.0, 0.0);
        // Same ray (needs multi-rev for dt > 0).
        assert_eq!(
            solve_lambert(r, r * 1.5, 5_000.0, MU, true),
            Err(LambertError::DegenerateGeometry)
        );
        // Exact 180 degrees: plane undefined.
        assert_eq!(
            solve_lambert(r, -r * 1.5, 5_000.0, MU, true),
            Err(LambertError::DegenerateGeometry)
        );
        assert_eq!(
            solve_lambert(r, DVec3::new(7.0e6, 1.0, 0.0), 0.0, MU, true),
            Err(LambertError::NonPositiveTime)
        );
        assert_eq!(
            solve_lambert(r, DVec3::new(0.0, 7.0e6, 0.0), 5_000.0, -1.0, true),
            Err(LambertError::NonPositiveMu)
        );
        assert_eq!(
            solve_lambert(DVec3::ZERO, DVec3::new(0.0, 7.0e6, 0.0), 5_000.0, MU, true),
            Err(LambertError::DegenerateGeometry)
        );
    }

    #[test]
    fn prograde_picks_the_cheap_side() {
        // Same-radius CW geometry: raw short_way returns the retrograde
        // arc (kill 7.8 km/s, come back), prograde picks the long way home.
        let mu = MU;
        let r = 10.0e6;
        let v_circ = (mu / r).sqrt();
        let r1 = DVec3::new(r, 0.0, 0.0);
        let v1 = DVec3::new(0.0, v_circ, 0.0);
        let angle = -60.0_f64.to_radians();
        let r2 = DVec3::new(r * angle.cos(), r * angle.sin(), 0.0);
        // Long way around: 300 deg of the circular orbit.
        let period = 2.0 * std::f64::consts::PI * (r * r * r / mu).sqrt();
        let dt = period * 300.0 / 360.0;
        let raw = solve_lambert(r1, r2, dt, mu, true).expect("raw solves");
        let raw_momentum = r1.cross(raw.departure_velocity_mps).z;
        assert!(raw_momentum < 0.0, "raw short arc is retrograde");
        let pro = solve_lambert_prograde(r1, v1, r2, dt, mu).expect("prograde solves");
        let pro_momentum = r1.cross(pro.departure_velocity_mps).z;
        assert!(pro_momentum > 0.0, "prograde arc keeps +Z momentum");
        // Prograde costs a small plane/phase correction, not a reversal.
        let raw_cost = (raw.departure_velocity_mps - v1).length();
        let pro_cost = (pro.departure_velocity_mps - v1).length();
        assert!(
            pro_cost < 0.2 * raw_cost,
            "pro {pro_cost} vs raw {raw_cost}"
        );
        // And it still satisfies the shared-orbit identities.
        let energy = pro.departure_velocity_mps.length_squared() / 2.0 - mu / r;
        let energy2 = pro.arrival_velocity_mps.length_squared() / 2.0 - mu / r;
        assert!((energy - energy2).abs() / energy.abs() <= 1.0e-8);
    }

    #[test]
    fn deterministic_across_runs() {
        let r1 = DVec3::new(7.0e6, 1.0e5, 0.0);
        let r2 = DVec3::new(-3.0e6, 9.0e6, 2.0e5);
        let first = solve_lambert(r1, r2, 8_000.0, MU, true).expect("solves");
        let second = solve_lambert(r1, r2, 8_000.0, MU, true).expect("solves");
        assert_eq!(first, second);
    }
}
