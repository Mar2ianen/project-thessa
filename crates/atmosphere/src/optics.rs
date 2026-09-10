//! Physical atmosphere definition shared by raster and ray-aware backends.
//!
//! Core rule (spec section 13-14):
//!
//! ```text
//! AtmosphereOptics -> raster/LUT backend
//!                  -> ray-aware backend (transmittance/sky queries)
//!                  -> future Solari integration (proxies only, never truth)
//! ```
//!
//! All spatial state is `f64` SI in planet-centric metres. Every animated
//! visual evaluates directly from absolute simulation time (warp-safe).

use glam::DVec3;
use serde::{Deserialize, Serialize};

/// One scattering constituent: sea-level extinction `beta_rgb` (m^-1),
/// exponential scale height, and Henyey-Greenstein asymmetry
/// (`g = 0` for Rayleigh).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScatteringLayer {
    pub beta_rgb: [f64; 3],
    pub scale_height_m: f64,
    pub asymmetry_g: f64,
}

/// One broad absorption term (ozone-like), same profile shape.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AbsorptionLayer {
    pub sigma_rgb: [f64; 3],
    pub scale_height_m: f64,
}

/// Upper-atmosphere emission, separate from scattering (spec section 11).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EmissionParams {
    /// Night-side airglow tint (relative radiance scale).
    pub airglow_rgb: [f64; 3],
    pub airglow_scale_height_m: f64,
    pub aurora: Option<AuroraParams>,
}

/// Deterministic analytic aurora configuration. No MHD: oval geometry,
/// altitude range, activity amplitude and procedural curtains driven by
/// absolute simulation time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AuroraParams {
    /// Unit magnetic-pole direction in body-fixed frame.
    pub magnetic_pole: DVec3,
    pub oval_latitude_rad: f64,
    pub oval_width_rad: f64,
    pub altitude_min_m: f64,
    pub altitude_max_m: f64,
    /// 0..=1 activity amplitude.
    pub activity: f64,
    pub color_bottom_rgb: [f64; 3],
    pub color_top_rgb: [f64; 3],
}

/// One atmosphere definition: optical medium around a body (spec section 4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereOptics {
    pub inner_radius_m: f64,
    pub outer_radius_m: f64,
    pub rayleigh: ScatteringLayer,
    pub mie: ScatteringLayer,
    pub absorption: Vec<AbsorptionLayer>,
    pub emission: EmissionParams,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpticsError {
    NonFiniteValue { field: &'static str },
    NonPositive { field: &'static str },
    OuterNotAboveInner,
}

impl std::fmt::Display for OpticsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteValue { field } => write!(f, "optics field {field} must be finite"),
            Self::NonPositive { field } => write!(f, "optics field {field} must be positive"),
            Self::OuterNotAboveInner => write!(f, "outer_radius_m must exceed inner_radius_m"),
        }
    }
}

impl std::error::Error for OpticsError {}

impl AtmosphereOptics {
    pub fn validate(&self) -> Result<(), OpticsError> {
        for (field, v) in [
            ("inner_radius_m", self.inner_radius_m),
            ("outer_radius_m", self.outer_radius_m),
            ("rayleigh.scale_height_m", self.rayleigh.scale_height_m),
            ("mie.scale_height_m", self.mie.scale_height_m),
        ] {
            if !v.is_finite() {
                return Err(OpticsError::NonFiniteValue { field });
            }
            if v <= 0.0 {
                return Err(OpticsError::NonPositive { field });
            }
        }
        if self.outer_radius_m <= self.inner_radius_m {
            return Err(OpticsError::OuterNotAboveInner);
        }
        for layer in [&self.rayleigh, &self.mie] {
            for c in layer.beta_rgb {
                if !c.is_finite() || c < 0.0 {
                    return Err(OpticsError::NonFiniteValue { field: "beta_rgb" });
                }
            }
            if !(-1.0..=1.0).contains(&layer.asymmetry_g) || !layer.asymmetry_g.is_finite() {
                return Err(OpticsError::NonFiniteValue {
                    field: "asymmetry_g",
                });
            }
        }
        Ok(())
    }

    /// Height above the datum in metres (clamped at zero below the surface).
    pub fn height_m(&self, planet_centric_m: DVec3) -> f64 {
        (planet_centric_m.length() - self.inner_radius_m).max(0.0)
    }

    /// Exponential density factor for a scale height at a planet-centric point.
    pub fn density_factor(&self, scale_height_m: f64, planet_centric_m: DVec3) -> f64 {
        (-self.height_m(planet_centric_m) / scale_height_m).exp()
    }
}

/// N2/O2-dominated (Thessa-like) optical baseline.
///
/// Sea-level Rayleigh cross-sections scale with surface pressure; the Mie
/// haze term and an ozone-like absorber are illustrative visual parameters,
/// not spectroscopy. Composition-driven derivation from body config is
/// follow-up work; the architecture point is that both backends consume this
/// single value.
pub fn nitrogen_oxygen_optics(
    body_radius_m: f64,
    surface_pressure_pa: f64,
    rayleigh_scale_height_m: f64,
) -> Result<AtmosphereOptics, OpticsError> {
    let pressure_scale = surface_pressure_pa / 101_325.0;
    // Sea-level Rayleigh extinction for an N2/O2 mix (m^-1, RGB).
    let rayleigh_beta = [5.8e-6, 13.5e-6, 33.1e-6].map(|b| b * pressure_scale);
    let optics = AtmosphereOptics {
        inner_radius_m: body_radius_m,
        outer_radius_m: body_radius_m + rayleigh_scale_height_m * 8.0,
        rayleigh: ScatteringLayer {
            beta_rgb: rayleigh_beta,
            scale_height_m: rayleigh_scale_height_m,
            asymmetry_g: 0.0,
        },
        mie: ScatteringLayer {
            beta_rgb: [3.9e-6, 3.9e-6, 3.9e-6].map(|b| b * pressure_scale.sqrt()),
            scale_height_m: (rayleigh_scale_height_m * 0.15).max(500.0),
            asymmetry_g: 0.76,
        },
        absorption: vec![AbsorptionLayer {
            // Ozone-like Chappuis broad absorber, illustrative magnitude.
            sigma_rgb: [6.0e-7, 1.8e-6, 2.5e-7].map(|s| s * pressure_scale.sqrt()),
            scale_height_m: 25_000.0,
        }],
        emission: EmissionParams {
            airglow_rgb: [0.10, 0.35, 0.22],
            airglow_scale_height_m: 8_000.0,
            aurora: None,
        },
    };
    optics.validate()?;
    Ok(optics)
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn thessa_like() -> AtmosphereOptics {
        // Thessa design reference: 1.20 bar, ~278 K, ~0.5 g gives a scale
        // height around 16 km (much more extended than Earth's ~8 km).
        nitrogen_oxygen_optics(3_200_000.0, 120_000.0, 16_300.0).unwrap()
    }

    #[test]
    fn extended_scale_height_accepted() {
        let optics = thessa_like();
        assert!(optics.outer_radius_m > optics.inner_radius_m);
        assert!(optics.rayleigh.scale_height_m > 12_000.0);
    }

    #[test]
    fn degenerate_shell_rejected() {
        let mut optics = thessa_like();
        optics.outer_radius_m = optics.inner_radius_m;
        assert_eq!(optics.validate(), Err(OpticsError::OuterNotAboveInner));
    }
}
