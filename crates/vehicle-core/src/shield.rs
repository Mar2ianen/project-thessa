//! Blunt heat-shield mounts: force-application geometry for the shared
//! aero pipeline.
//!
//! A detached forebody/aft shield is a flat disc facing its outward normal.
//! Lift and drag come from the common Newtonian disc zones in
//! [`AeroGeometry`](crate::AeroGeometry) (`aero-core` evaluates them in the
//! same summation as the panels: one flow solution, one result) — never from
//! a parallel force path. Shield mass already bakes through the fuselage
//! hull, so mounts never re-add it here. Ablation, recession, and
//! burn-through stay design (`docs/details/11_HEAT_SHIELDS.md` §1.2).

use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

#[derive(Debug, Clone, PartialEq)]
pub enum ShieldError {
    InvalidSpec(String),
}

impl fmt::Display for ShieldError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid heat shield: {message}"),
        }
    }
}

impl Error for ShieldError {}

/// One blunt shield disc retained on the vehicle for aerodynamics.
/// Mass lives in the fuselage hull aggregate; this mount carries the
/// force-application geometry only.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeatShieldMount {
    pub name: String,
    /// Disc center in body metres.
    pub position_body_m: DVec3,
    /// Outward unit normal in body coordinates.
    pub normal_body_m: DVec3,
    pub diameter_m: f64,
}

impl HeatShieldMount {
    pub fn new(
        name: impl Into<String>,
        position_body_m: DVec3,
        normal_body_m: DVec3,
        diameter_m: f64,
    ) -> Result<Self, ShieldError> {
        let mount = Self {
            name: name.into(),
            position_body_m,
            normal_body_m,
            diameter_m,
        };
        mount.validate()?;
        Ok(mount)
    }

    pub fn validate(&self) -> Result<(), ShieldError> {
        if self.name.trim().is_empty() {
            return Err(ShieldError::InvalidSpec(
                "heat-shield mount needs a name".into(),
            ));
        }
        if !self.position_body_m.is_finite() {
            return Err(ShieldError::InvalidSpec(format!(
                "heat shield '{}' has a non-finite position",
                self.name
            )));
        }
        if !self.normal_body_m.is_finite() || (self.normal_body_m.length() - 1.0).abs() > 1.0e-9 {
            return Err(ShieldError::InvalidSpec(format!(
                "heat shield '{}' normal must be unit length",
                self.name
            )));
        }
        if !self.diameter_m.is_finite() || self.diameter_m <= 0.0 {
            return Err(ShieldError::InvalidSpec(format!(
                "heat shield '{}' needs a positive finite diameter",
                self.name
            )));
        }
        Ok(())
    }

    pub fn area_m2(&self) -> f64 {
        std::f64::consts::PI * (0.5 * self.diameter_m).powi(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disc_area_closes_over_diameter() {
        let mount = HeatShieldMount::new("fore-shield", DVec3::new(2.0, 0.0, 0.0), DVec3::X, 2.0)
            .expect("valid disc");
        assert!((mount.area_m2() - std::f64::consts::PI).abs() < 1.0e-12);
    }

    #[test]
    fn bad_mounts_fail_closed() {
        assert!(HeatShieldMount::new("", DVec3::ZERO, DVec3::X, 2.0).is_err());
        assert!(HeatShieldMount::new("s", DVec3::ZERO, DVec3::new(2.0, 0.0, 0.0), 2.0).is_err());
        assert!(HeatShieldMount::new("s", DVec3::ZERO, DVec3::X, 0.0).is_err());
        assert!(
            HeatShieldMount::new("s", DVec3::new(f64::INFINITY, 0.0, 0.0), DVec3::X, 2.0).is_err()
        );
    }
}
