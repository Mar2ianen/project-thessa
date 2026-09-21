//! Out-of-plane bend curve: spanwise elevation `z(s)` in metres.
//!
//! The flat planform stays the canonical parameter domain; the bend maps
//! stations into 3D and builds the local section frame together with the
//! section incidence. Bending changes projected span/area and local frames
//! without changing material surface area: a strip's 3D length is
//! `hypot(dy, dz)`, never `dy` alone.

use serde::{Deserialize, Serialize};

use crate::SurfaceError;

/// One bend station: elevation `z_m` in metres at span coordinate `s`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BendStation {
    /// Normalized span coordinate, strictly increasing through the list.
    pub s: f64,
    /// Out-of-plane elevation in metres (`z`, positive up).
    pub z_m: f64,
}

/// Spanwise bend/elevation curve over the flat planform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BendCurve {
    /// Stations ordered by strictly increasing `s` from `0` to `1`.
    pub stations: Vec<BendStation>,
}

impl BendCurve {
    /// Flat surface: identically zero elevation.
    pub fn flat() -> Self {
        Self {
            stations: vec![
                BendStation { s: 0.0, z_m: 0.0 },
                BendStation { s: 1.0, z_m: 0.0 },
            ],
        }
    }

    /// Constant dihedral/anhedral: linear rise over the span.
    ///
    /// `angle_rad` is the bend angle from the flat plane (positive raises
    /// the tip). Elevation is `tan(angle) * s * span_m`; the compiler, not
    /// the curve, knows the span, so it is a construction parameter here.
    pub fn dihedral(span_m: f64, angle_rad: f64) -> Result<Self, SurfaceError> {
        if !span_m.is_finite() || span_m <= 0.0 {
            return Err(SurfaceError::InvalidSurface(format!(
                "dihedral bend needs a positive span (got {span_m})"
            )));
        }
        if !angle_rad.is_finite() || angle_rad.abs() >= std::f64::consts::FRAC_PI_2 {
            return Err(SurfaceError::InvalidBend(format!(
                "dihedral angle must be finite and below 90 deg (got {angle_rad})"
            )));
        }
        let curve = Self {
            stations: vec![
                BendStation { s: 0.0, z_m: 0.0 },
                BendStation {
                    s: 1.0,
                    z_m: angle_rad.tan() * span_m,
                },
            ],
        };
        curve.validate()?;
        Ok(curve)
    }

    /// Arbitrary elevation polyline: gull wings, canted tips, winglets.
    pub fn polyline(stations: Vec<BendStation>) -> Result<Self, SurfaceError> {
        let curve = Self { stations };
        curve.validate()?;
        Ok(curve)
    }

    /// Check the curve contract: at least two stations, `s` strictly
    /// increasing across exactly `[0, 1]`, finite elevations.
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if self.stations.len() < 2 {
            return Err(SurfaceError::InvalidStations(
                "bend curve needs at least two stations".into(),
            ));
        }
        let mut previous = f64::NEG_INFINITY;
        for station in &self.stations {
            if !station.s.is_finite() || !station.z_m.is_finite() {
                return Err(SurfaceError::InvalidBend(format!(
                    "bend station s={} has non-finite geometry",
                    station.s
                )));
            }
            if station.s <= previous {
                return Err(SurfaceError::InvalidBend(format!(
                    "bend stations must strictly increase in s (got {} after {})",
                    station.s, previous
                )));
            }
            previous = station.s;
        }
        if self.stations.first().map(|station| station.s) != Some(0.0)
            || self.stations.last().map(|station| station.s) != Some(1.0)
        {
            return Err(SurfaceError::InvalidBend(
                "bend stations must span exactly s=0..=1".into(),
            ));
        }
        Ok(())
    }

    /// Elevation in metres at `s` by piecewise-linear interpolation.
    pub fn elevation(&self, s: f64) -> f64 {
        let s = s.clamp(0.0, 1.0);
        let stations = &self.stations;
        if s <= stations[0].s {
            return stations[0].z_m;
        }
        for pair in stations.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            if s <= b.s {
                let span = b.s - a.s;
                if span <= 0.0 {
                    return a.z_m;
                }
                let t = (s - a.s) / span;
                return a.z_m + t * (b.z_m - a.z_m);
            }
        }
        stations.last().expect("validated non-empty").z_m
    }

    /// Elevation slope `dz/ds` of the bend segment containing `s`. Exact
    /// on the piecewise-linear curve; no finite differences.
    pub(crate) fn elevation_slope(&self, s: f64) -> f64 {
        let stations = &self.stations;
        let s = s.clamp(0.0, 1.0);
        for pair in stations.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            if s <= b.s {
                let span = b.s - a.s;
                if span <= 0.0 {
                    return 0.0;
                }
                return (b.z_m - a.z_m) / span;
            }
        }
        0.0
    }

    /// Uniform rescale mapping authored `(s * span_m, elevation(s))` to
    /// physical `(Y, Z)` so the total root-to-tip material length equals
    /// `span_m`: `Y(s) = k * s * span_m`, `Z(s) = k * elevation(s)`.
    ///
    /// Bending then changes projected span/area and local frames without
    /// changing material surface area merely because of orientation. Flat
    /// curves map to exactly `1.0`; smooth curves are length-integrated
    /// with a fine deterministic trapezoid rule.
    pub(crate) fn material_scale(&self, span_m: f64) -> f64 {
        const SAMPLES: usize = 1024;
        let mut raw = 0.0;
        let (mut previous_y, mut previous_z) = (0.0, self.elevation(0.0));
        for index in 1..=SAMPLES {
            let s = index as f64 / SAMPLES as f64;
            let (y, z) = (s * span_m, self.elevation(s));
            raw += (y - previous_y).hypot(z - previous_z);
            previous_y = y;
            previous_z = z;
        }
        if raw <= 0.0 { 1.0 } else { span_m / raw }
    }
}
