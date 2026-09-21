//! Spanwise section data: incidence, thickness, profile extension point.
//!
//! Planform plus bend place the surface in 3D; sections orient each station
//! aerodynamically. Geometric twist is the spanwise variation of incidence,
//! kept as ordinary per-station data rather than a separate primitive.

use serde::{Deserialize, Serialize};

use crate::SurfaceError;

/// Future per-section aerodynamic profile reference.
///
/// The first implementation carries no polar table; the identifier reserves
/// the attachment point so compiled panels can later reference different
/// section profiles instead of one global airfoil model.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AeroProfileId(pub String);

/// One section station: local aerodynamic orientation and thickness.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SectionStation {
    /// Normalized span coordinate, strictly increasing through the list.
    pub s: f64,
    /// Total local incidence in radians, positive leading-edge-up. The
    /// spanwise variation of this value is the geometric twist.
    pub incidence_rad: f64,
    /// Maximum thickness divided by local chord, in `[0, 0.5]`, matching
    /// the solver's supersonic thickness term range.
    pub thickness_ratio: f64,
    /// Optional profile reference; `None` keeps the global airfoil model.
    #[serde(default)]
    pub profile: Option<AeroProfileId>,
}

/// Spanwise section data over `s in [0, 1]`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SectionData {
    /// Stations ordered by strictly increasing `s` from `0` to `1`.
    pub stations: Vec<SectionStation>,
}

impl SectionData {
    /// Uniform section: constant incidence and thickness everywhere.
    pub fn uniform(incidence_rad: f64, thickness_ratio: f64) -> Result<Self, SurfaceError> {
        Self::from_stations(vec![
            SectionStation {
                s: 0.0,
                incidence_rad,
                thickness_ratio,
                profile: None,
            },
            SectionStation {
                s: 1.0,
                incidence_rad,
                thickness_ratio,
                profile: None,
            },
        ])
    }

    /// Arbitrary section schedule (washout, thick root / thin tip, ...).
    pub fn from_stations(stations: Vec<SectionStation>) -> Result<Self, SurfaceError> {
        let data = Self { stations };
        data.validate()?;
        Ok(data)
    }

    /// Check the schedule contract: coverage, ordering, finite angles,
    /// thickness inside the solver range.
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if self.stations.len() < 2 {
            return Err(SurfaceError::InvalidStations(
                "section data needs at least two stations".into(),
            ));
        }
        let mut previous = f64::NEG_INFINITY;
        for station in &self.stations {
            if !station.s.is_finite() || !station.incidence_rad.is_finite() {
                return Err(SurfaceError::InvalidSection(format!(
                    "section station s={} has non-finite data",
                    station.s
                )));
            }
            if station.incidence_rad.abs() >= std::f64::consts::FRAC_PI_2 {
                return Err(SurfaceError::InvalidSection(format!(
                    "section incidence must stay below 90 deg at s={} (got {})",
                    station.s, station.incidence_rad
                )));
            }
            if !station.thickness_ratio.is_finite()
                || !(0.0..=0.5).contains(&station.thickness_ratio)
            {
                return Err(SurfaceError::InvalidSection(format!(
                    "section thickness ratio must be in [0, 0.5] at s={} (got {})",
                    station.s, station.thickness_ratio
                )));
            }
            if station.s <= previous {
                return Err(SurfaceError::InvalidSection(format!(
                    "section stations must strictly increase in s (got {} after {})",
                    station.s, previous
                )));
            }
            previous = station.s;
        }
        if self.stations.first().map(|station| station.s) != Some(0.0)
            || self.stations.last().map(|station| station.s) != Some(1.0)
        {
            return Err(SurfaceError::InvalidSection(
                "section stations must span exactly s=0..=1".into(),
            ));
        }
        Ok(())
    }

    /// Local incidence in radians at `s` by piecewise-linear interpolation.
    pub fn incidence(&self, s: f64) -> f64 {
        interpolate(&self.stations, s, |station| station.incidence_rad)
    }

    /// Thickness ratio at `s` by piecewise-linear interpolation.
    pub fn thickness(&self, s: f64) -> f64 {
        interpolate(&self.stations, s, |station| station.thickness_ratio)
    }
}

fn interpolate(stations: &[SectionStation], s: f64, pick: impl Fn(&SectionStation) -> f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    if s <= stations[0].s {
        return pick(&stations[0]);
    }
    for pair in stations.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if s <= b.s {
            let span = b.s - a.s;
            if span <= 0.0 {
                return pick(a);
            }
            let t = (s - a.s) / span;
            return pick(a) + t * (pick(b) - pick(a));
        }
    }
    pick(stations.last().expect("validated non-empty"))
}
