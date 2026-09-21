//! Section profile library and cruise selector.
//!
//! Status: advisory. The solver still consumes one global airfoil model
//! (design doc section 13: per-panel polar data is future work), so the
//! selector output attaches to [`SectionData`](crate::SectionData) as
//! authored metadata through [`ProfilePick::profile_id`] plus explicit
//! thickness, ready for the baker and the future solver. Nothing here
//! feeds back into compiled panel forces today.
//!
//! Method, no magic coefficients: NACA 4-digit geometry is the public
//! Abbott/Von Doenhoff analytic definition (thickness polynomial, parabolic
//! camber arcs); the zero-lift angle is the thin-airfoil integral evaluated
//! numerically; the Cl-max band is an empirical bracket around published 2D
//! wind-tunnel values at Re 3-10M, returned as a band precisely because a
//! single number would pretend CFD accuracy. The cruise selector inverts
//! the thin-airfoil lift line for the camber that delivers a required
//! section Cl at a deck attitude, on the standard 4-digit grid.

use serde::{Deserialize, Serialize};

use crate::{AeroProfileId, SectionData, SurfaceError};

/// NACA 4-digit family parameters: max camber `m`, camber position `p`,
/// max thickness `t`, all as fractions of chord.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Naca4 {
    /// Max camber fraction (0.02 in NACA 2412).
    pub m: f64,
    /// Camber position fraction (0.4 in NACA 2412).
    pub p: f64,
    /// Max thickness fraction (0.12 in NACA 2412).
    pub t: f64,
}

impl Naca4 {
    /// Validate the family box: camber and position inside (0, 0.1] and
    /// [0.2, 0.7]-ish usable ranges, thickness inside (0, 0.3].
    pub fn new(m: f64, p: f64, t: f64) -> Result<Self, SurfaceError> {
        if !m.is_finite() || m < 0.0 || m > 0.1 {
            return Err(SurfaceError::InvalidSection(format!(
                "profile camber must be in [0, 0.1] (got {m})"
            )));
        }
        if !p.is_finite() || p <= 0.0 || p >= 1.0 {
            return Err(SurfaceError::InvalidSection(format!(
                "profile camber position must be in (0, 1) (got {p})"
            )));
        }
        if !t.is_finite() || t <= 0.0 || t > 0.3 {
            return Err(SurfaceError::InvalidSection(format!(
                "profile thickness must be in (0, 0.3] (got {t})"
            )));
        }
        Ok(Self { m, p, t })
    }

    /// Canonical digits, e.g. NACA 2412.
    pub fn digits(m: f64, p: f64, t: f64) -> String {
        format!(
            "NACA{:01.0}{:01.0}{:02.0}",
            (m * 100.0).round(),
            (p * 10.0).round(),
            (t * 100.0).round()
        )
    }

    /// Half-thickness distribution `yt(x)` for chord fractions `x in
    /// [0, 1]`: the textbook 4-digit polynomial (open trailing edge).
    /// Full section thickness is `2 * yt`; the `t` parameter is the full
    /// thickness fraction (0.12 peaks at 0.06 half-thickness).
    pub fn thickness_at(&self, x: f64) -> f64 {
        let x = x.clamp(0.0, 1.0);
        5.0 * self.t
            * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x.powi(2) + 0.2843 * x.powi(3)
                - 0.1015 * x.powi(4))
    }

    /// Camber-line height and slope `(yc, dyc/dx)` at `x`: two parabolic
    /// arcs joined at `p` with matching value and slope.
    pub fn camber_at(&self, x: f64) -> (f64, f64) {
        let x = x.clamp(0.0, 1.0);
        if self.m == 0.0 {
            return (0.0, 0.0);
        }
        if x < self.p {
            let yc = self.m / self.p.powi(2) * (2.0 * self.p * x - x.powi(2));
            let slope = 2.0 * self.m / self.p.powi(2) * (self.p - x);
            (yc, slope)
        } else {
            let yc_den = (1.0 - self.p).powi(2);
            let yc = self.m / yc_den * ((1.0 - 2.0 * self.p) + 2.0 * self.p * x - x.powi(2));
            let slope = 2.0 * self.m / yc_den * (self.p - x);
            (yc, slope)
        }
    }

    /// Thin-airfoil zero-lift angle in radians:
    /// `α0 = -(1/π) ∫₀^π (dyc/dx)(cosθ − 1) dθ`, 512-point trapezoid.
    /// Symmetric sections return exactly 0 by construction.
    pub fn zero_lift_angle_rad(&self) -> f64 {
        if self.m == 0.0 {
            return 0.0;
        }
        const STEPS: usize = 512;
        let mut sum = 0.0;
        for index in 0..=STEPS {
            let theta = index as f64 / STEPS as f64 * std::f64::consts::PI;
            let x = 0.5 * (1.0 - theta.cos());
            let (_, slope) = self.camber_at(x);
            let weight = if index == 0 || index == STEPS {
                0.5
            } else {
                1.0
            };
            sum += weight * slope * (theta.cos() - 1.0);
        }
        -sum / STEPS as f64
    }

    /// Empirical Cl-max bracket `(low, high)` around 2D wind-tunnel values
    /// at Re 3-10M: `1.52 + 6m + 0.8(t − 0.12) ± 0.12`. Anchors: NACA 0012
    /// stalls near 1.5-1.6, 2412 near 1.6-1.7, 4412 near 1.7-1.8. A band,
    /// not a number: stall is the first thing CFD would correct.
    pub fn cl_max_band(&self) -> (f64, f64) {
        let mid = 1.52 + 6.0 * self.m + 0.8 * (self.t - 0.12);
        (mid - 0.12, mid + 0.12)
    }
}

/// What the cruise selector must deliver.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CruiseRequirement {
    /// Section lift coefficient the profile must deliver at cruise.
    pub target_section_cl: f64,
    /// Body attitude at cruise in degrees (deck angle, typically ~2).
    pub deck_angle_deg: f64,
    /// Structural thickness ceiling as a chord fraction.
    pub max_thickness_ratio: f64,
    /// Reynolds number, informational: the Cl-max band is calibrated for
    /// Re 3M-10M; outside, treat the band as wider than stated.
    pub reynolds_number: f64,
}

/// The selector's answer: best standard-grid 4-digit profile for the
/// requirement, i.e. the least camber that still delivers the target Cl
/// at the deck attitude with predicted stall margin.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProfilePick {
    /// Rounded family parameters on the standard 4-digit grid.
    pub family: Naca4,
    /// Thin-airfoil zero-lift angle of the rounded family (radians).
    pub zero_lift_angle_rad: f64,
    /// Empirical Cl-max bracket of the rounded family.
    pub cl_max_low: f64,
    /// Empirical Cl-max bracket of the rounded family.
    pub cl_max_high: f64,
    /// Predicted margin: bracket midpoint minus required Cl. Negative
    /// means the requirement exceeds the bracket: returned honestly, not
    /// hidden, so the caller can relax the requirement or the thickness.
    pub stall_margin: f64,
}

impl ProfilePick {
    /// Canonical profile identifier for [`SectionData`] attachment.
    pub fn profile_id(&self) -> AeroProfileId {
        AeroProfileId(Naca4::digits(self.family.m, self.family.p, self.family.t))
    }

    /// Stamp the pick onto section data: uniform picked thickness plus
    /// the profile reference. Per-station thickness tailoring stays
    /// manual; this sets the consistent baseline.
    pub fn apply_to_sections(&self, sections: &mut SectionData) {
        for station in &mut sections.stations {
            station.thickness_ratio = self.family.t;
            station.profile = Some(self.profile_id());
        }
    }
}

/// Pick the least-camber standard-grid NACA 4-digit profile delivering
/// `target_section_cl` at the deck attitude on the 2D lift line
/// `Cl = 2π(α_deck − α0)`, with thickness at the structural ceiling
/// (rounded down to the grid, never exceeded).
pub fn recommend_cruise_profile(
    requirement: &CruiseRequirement,
) -> Result<ProfilePick, SurfaceError> {
    if !requirement.target_section_cl.is_finite() || requirement.target_section_cl <= 0.0 {
        return Err(SurfaceError::InvalidSection(format!(
            "cruise target Cl must be positive and finite (got {})",
            requirement.target_section_cl
        )));
    }
    if !requirement.deck_angle_deg.is_finite() {
        return Err(SurfaceError::InvalidSection(
            "cruise deck angle must be finite".into(),
        ));
    }
    if !requirement.max_thickness_ratio.is_finite()
        || requirement.max_thickness_ratio < 0.03
        || requirement.max_thickness_ratio > 0.30
    {
        return Err(SurfaceError::InvalidSection(format!(
            "structural thickness ceiling must be in [0.03, 0.30] (got {})",
            requirement.max_thickness_ratio
        )));
    }
    if !requirement.reynolds_number.is_finite() || requirement.reynolds_number <= 0.0 {
        return Err(SurfaceError::InvalidSection(
            "Reynolds number must be positive and finite".into(),
        ));
    }
    // Calibrate camber effectiveness from the 2-percent reference: thin
    // airfoil α0 is linear in camber at fixed position.
    let reference = Naca4::new(0.02, 0.4, 0.12)?;
    let alpha0_per_camber = reference.zero_lift_angle_rad() / 0.02;
    let deck_rad = requirement.deck_angle_deg.to_radians();
    let alpha0_needed_rad = deck_rad - requirement.target_section_cl / (2.0 * std::f64::consts::PI);
    let camber_needed = (alpha0_needed_rad / alpha0_per_camber).clamp(0.0, 0.06);
    // Round to the standard grid: camber to whole percent, thickness down
    // to whole percent so the structural ceiling is never exceeded.
    let m = (camber_needed * 100.0).round() / 100.0;
    let t = (requirement.max_thickness_ratio * 100.0).floor() / 100.0;
    let family = Naca4::new(m, 0.4, t)?;
    let zero_lift_angle_rad = family.zero_lift_angle_rad();
    let (cl_max_low, cl_max_high) = family.cl_max_band();
    let stall_margin = 0.5 * (cl_max_low + cl_max_high) - requirement.target_section_cl;
    Ok(ProfilePick {
        family,
        zero_lift_angle_rad,
        cl_max_low,
        cl_max_high,
        stall_margin,
    })
}
