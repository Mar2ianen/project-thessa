//! Superellipse cross-section math for station-loft fuselages.
//!
//! One section is `|y/w|^n + |z/h|^n = 1` with `n = 2` an ellipse and large
//! `n` a rounded box (SimplePlanes-style blocks are high-`n` sections with
//! small height/width asymmetry). Area is closed-form through the gamma
//! function; perimeter is deterministic Simpson integration over a central-
//! difference speed (the analytic parametrization derivative is singular
//! for `n > 2`, while the curve itself stays smooth).

use serde::{Deserialize, Serialize};

use crate::FuselageError;

/// Closed-form asymmetric area: top and bottom half-superellipses.
/// `n = 2` both halves recovers `pi*w*(ht+hb)/2`; symmetric halves
/// recover [`superellipse_area`].
pub fn section_area_m2(
    half_width_m: f64,
    top_height_m: f64,
    bottom_height_m: f64,
    top_exponent: f64,
    bottom_exponent: f64,
) -> f64 {
    2.0 * half_width_m
        * (top_height_m * exponent_factor(top_exponent)
            + bottom_height_m * exponent_factor(bottom_exponent))
}

/// Shape factor `G(1+1/n)^2/G(1+2/n)` shared by area halves.
fn exponent_factor(exponent: f64) -> f64 {
    gamma(1.0 + 1.0 / exponent).powi(2) / gamma(1.0 + 2.0 / exponent)
}

/// Closed-form superellipse area: `4*w*h*G(1+1/n)^2/G(1+2/n)`.
///
/// `n = 2` recovers `pi*w*h`; `n -> inf` recovers the `2w by 2h` box.
pub fn superellipse_area(half_width_m: f64, half_height_m: f64, exponent: f64) -> f64 {
    4.0 * half_width_m * half_height_m * exponent_factor(exponent)
}

/// Point on the section outline at clock angle `phi` (0 = +Y, toward +Z).
/// Top (`sin >= 0`) and bottom halves use their own height/exponent, so
/// lifting-body sections (round top, flat chined bottom) stay exact.
pub fn outline_point(
    half_width_m: f64,
    top_height_m: f64,
    bottom_height_m: f64,
    top_exponent: f64,
    bottom_exponent: f64,
    phi_rad: f64,
) -> (f64, f64) {
    let (sin, cos) = phi_rad.sin_cos();
    let (height_m, exponent) = if sin >= 0.0 {
        (top_height_m, top_exponent)
    } else {
        (bottom_height_m, bottom_exponent)
    };
    let scale = (cos.abs() / half_width_m).powf(exponent) + (sin.abs() / height_m).powf(exponent);
    let radius = scale.powf(-1.0 / exponent);
    (radius * cos, radius * sin)
}

/// Deterministic polygon perimeter with `samples` (>= 64) outline points.
///
/// Chord summation converges from inside quadratically with no
/// derivatives at all, which keeps high-exponent (boxy, chined) corners
/// honest where the analytic parametrization derivative is singular.
pub fn superellipse_perimeter(
    half_width_m: f64,
    top_height_m: f64,
    bottom_height_m: f64,
    top_exponent: f64,
    bottom_exponent: f64,
    samples: usize,
) -> Result<f64, FuselageError> {
    if samples < 64 {
        return Err(FuselageError::InvalidOptions(format!(
            "perimeter samples must be >= 64 (got {samples})"
        )));
    }
    let step = std::f64::consts::TAU / samples as f64;
    let mut previous = outline_point(
        half_width_m,
        top_height_m,
        bottom_height_m,
        top_exponent,
        bottom_exponent,
        0.0,
    );
    let mut total = 0.0;
    for index in 1..=samples {
        let current = outline_point(
            half_width_m,
            top_height_m,
            bottom_height_m,
            top_exponent,
            bottom_exponent,
            index as f64 * step,
        );
        total += (current.0 - previous.0).hypot(current.1 - previous.1);
        previous = current;
    }
    Ok(total)
}

/// Exact area centroid of a section composed of upper and lower
/// half-superellipses. The y centroid is zero by left/right symmetry;
/// each half's z centroid follows from beta-function area integrals.
pub fn section_centroid_yz(
    half_width_m: f64,
    top_height_m: f64,
    bottom_height_m: f64,
    top_exponent: f64,
    bottom_exponent: f64,
) -> (f64, f64) {
    let top_area = 2.0 * half_width_m * top_height_m * exponent_factor(top_exponent);
    let bottom_area = 2.0 * half_width_m * bottom_height_m * exponent_factor(bottom_exponent);
    let half_centroid = |height: f64, exponent: f64| {
        0.5 * height * gamma(1.0 + 2.0 / exponent).powi(2)
            / (gamma(1.0 + 1.0 / exponent) * gamma(1.0 + 3.0 / exponent))
    };
    let first_moment = top_area * half_centroid(top_height_m, top_exponent)
        - bottom_area * half_centroid(bottom_height_m, bottom_exponent);
    (0.0, first_moment / (top_area + bottom_area))
}

/// Gamma function for positive arguments (Lanczos g=7, 9 coefficients).
/// Powers in [`superellipse_area`] only ever query `1 + small`, but the
/// routine is general over `x > 0` and tested against textbook values.
pub fn gamma(x: f64) -> f64 {
    // Reflection covers 0 < x < 0.5; the compiler only needs x >= 1.
    if x < 0.5 {
        return std::f64::consts::PI / ((std::f64::consts::PI * x).sin() * gamma(1.0 - x));
    }
    // Lanczos approximation (Paul Godfrey coefficients).
    const COEFFS: [f64; 9] = [
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7,
    ];
    let z = x - 1.0;
    let mut series = COEFFS[0];
    for (index, coeff) in COEFFS.iter().enumerate().skip(1) {
        series += coeff / (z + index as f64);
    }
    let t = z + 7.5;
    (std::f64::consts::TAU).sqrt() * t.powf(z + 0.5) * (-t).exp() * series
}

/// One loft station: asymmetric superellipse section plus lateral offset.
///
/// The top (`+Z`) and bottom halves own independent heights and
/// exponents: round tops over flat chined bottoms (lifting bodies) are
/// first-class, while equal halves recover the plain superellipse.
/// Stations run tail-to-nose with strictly increasing `x_m`; the nose is
/// the last station (body `+X` is forward, matching sim-core axes).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BodyStation {
    /// Longitudinal position in body metres.
    pub x_m: f64,
    /// Lateral semi-axis in metres (+Y, shared by both halves).
    pub half_width_m: f64,
    /// Upper vertical semi-axis in metres (+Z).
    pub top_height_m: f64,
    /// Lower vertical semi-axis in metres (−Z).
    pub bottom_height_m: f64,
    /// Upper superellipse exponent: 2 is elliptical, higher is boxier.
    pub top_exponent: f64,
    /// Lower superellipse exponent (chines live here).
    pub bottom_exponent: f64,
    /// Lateral center offset in metres.
    pub offset_y_m: f64,
    /// Vertical center offset in metres (centerline camber/droop).
    pub offset_z_m: f64,
}

impl BodyStation {
    /// Direct station constructor with full validation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        x_m: f64,
        half_width_m: f64,
        top_height_m: f64,
        bottom_height_m: f64,
        top_exponent: f64,
        bottom_exponent: f64,
        offset_y_m: f64,
        offset_z_m: f64,
    ) -> Result<Self, FuselageError> {
        let station = Self {
            x_m,
            half_width_m,
            top_height_m,
            bottom_height_m,
            top_exponent,
            bottom_exponent,
            offset_y_m,
            offset_z_m,
        };
        station.validate()?;
        Ok(station)
    }

    /// Round station (Juno-style tank sections).
    pub fn round(x_m: f64, radius_m: f64) -> Result<Self, FuselageError> {
        Self::new(x_m, radius_m, radius_m, radius_m, 2.0, 2.0, 0.0, 0.0)
    }

    /// Boxy station (SimplePlanes-style block sections).
    pub fn boxy(
        x_m: f64,
        half_width_m: f64,
        half_height_m: f64,
        exponent: f64,
    ) -> Result<Self, FuselageError> {
        Self::new(
            x_m,
            half_width_m,
            half_height_m,
            half_height_m,
            exponent,
            exponent,
            0.0,
            0.0,
        )
    }

    pub fn validate(self) -> Result<(), FuselageError> {
        if !self.x_m.is_finite()
            || !self.half_width_m.is_finite()
            || !self.top_height_m.is_finite()
            || !self.bottom_height_m.is_finite()
            || !self.top_exponent.is_finite()
            || !self.bottom_exponent.is_finite()
            || !self.offset_y_m.is_finite()
            || !self.offset_z_m.is_finite()
        {
            return Err(FuselageError::InvalidSection(
                "station values must be finite".into(),
            ));
        }
        if self.half_width_m <= 0.0 || self.top_height_m <= 0.0 || self.bottom_height_m <= 0.0 {
            return Err(FuselageError::InvalidSection(format!(
                "station at x={} needs positive semi-axes",
                self.x_m
            )));
        }
        for exponent in [self.top_exponent, self.bottom_exponent] {
            if !(2.0..=12.0).contains(&exponent) {
                return Err(FuselageError::InvalidSection(format!(
                    "station exponents must be in [2, 12] (got {exponent})"
                )));
            }
        }
        Ok(())
    }

    /// Outer-mold cross-section area at this station.
    pub fn area_m2(self) -> f64 {
        section_area_m2(
            self.half_width_m,
            self.top_height_m,
            self.bottom_height_m,
            self.top_exponent,
            self.bottom_exponent,
        )
    }

    /// Equivalent circular radius for the section area.
    pub fn equivalent_radius_m(self) -> f64 {
        (self.area_m2() / std::f64::consts::PI).sqrt()
    }

    /// Outline centroid (camber point) relative to the section frame:
    /// `(0, 0)` for symmetric sections, shifted toward the fuller half
    /// for lifting-body sections.
    pub fn camber_yz(self) -> (f64, f64) {
        section_centroid_yz(
            self.half_width_m,
            self.top_height_m,
            self.bottom_height_m,
            self.top_exponent,
            self.bottom_exponent,
        )
    }

    /// Linear interpolation between stations (exact on linear inputs, the
    /// same authoring-exactness contract as the wing compiler: subdivision
    /// refines solver locality without moving geometry).
    pub fn lerp(self, other: Self, t: f64) -> Self {
        Self {
            x_m: self.x_m + (other.x_m - self.x_m) * t,
            half_width_m: self.half_width_m + (other.half_width_m - self.half_width_m) * t,
            top_height_m: self.top_height_m + (other.top_height_m - self.top_height_m) * t,
            bottom_height_m: self.bottom_height_m
                + (other.bottom_height_m - self.bottom_height_m) * t,
            top_exponent: self.top_exponent + (other.top_exponent - self.top_exponent) * t,
            bottom_exponent: self.bottom_exponent
                + (other.bottom_exponent - self.bottom_exponent) * t,
            offset_y_m: self.offset_y_m + (other.offset_y_m - self.offset_y_m) * t,
            offset_z_m: self.offset_z_m + (other.offset_z_m - self.offset_z_m) * t,
        }
    }
}
