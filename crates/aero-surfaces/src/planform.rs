//! Flat planform authoring: leading/trailing edge splines over `s in [0, 1]`.
//!
//! A station list is the low-complexity spline of the design doc: a plain
//! trapezoid is two stations, an ogival or compound-delta planform is more
//! stations. Evaluation is piecewise linear; the compiler subdivides smooth
//! regions to its tolerance, so authored stations should mark real geometric
//! features, not dense samples.

use serde::{Deserialize, Serialize};

use crate::SurfaceError;

/// One flat-planform station: chordwise edge positions in metres at the
/// normalized span coordinate `s` (`0` root, `1` tip).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpanStation {
    /// Normalized span coordinate, must strictly increase through the list.
    pub s: f64,
    /// Leading-edge chordwise position in metres (`x`, positive aft).
    pub x_le: f64,
    /// Trailing-edge chordwise position in metres, strictly ahead of `x_le`.
    pub x_te: f64,
}

/// Flat planform: leading/trailing edge boundary splines.
///
/// The planform stays in the `x/y` plane. Out-of-plane shape is the
/// [`BendCurve`](crate::BendCurve); local orientation is
/// [`SectionData`](crate::SectionData).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Planform {
    /// Stations ordered by strictly increasing `s` from `0` to `1`.
    pub stations: Vec<SpanStation>,
}

impl Planform {
    /// Rectangular wing: constant chord, unswept, spanning `span_m`.
    ///
    /// The span itself lives on the surface; the planform only maps `s` to
    /// chordwise positions, so a rectangle is two identical stations.
    pub fn rectangular(chord_m: f64) -> Result<Self, SurfaceError> {
        Self::tapered(chord_m, chord_m, 0.0)
    }

    /// Tapered wing with an optional tip leading-edge offset (sweep driver).
    ///
    /// `tip_le_offset_m` shifts the tip leading edge aft by that amount, so
    /// leading-edge sweep is `atan(tip_le_offset_m / span_m)` once mounted.
    /// `taper = tip / root`; `1.0` with zero offset is a rectangle.
    pub fn tapered(
        root_chord_m: f64,
        tip_chord_m: f64,
        tip_le_offset_m: f64,
    ) -> Result<Self, SurfaceError> {
        let planform = Self {
            stations: vec![
                SpanStation {
                    s: 0.0,
                    x_le: 0.0,
                    x_te: root_chord_m,
                },
                SpanStation {
                    s: 1.0,
                    x_le: tip_le_offset_m,
                    x_te: tip_le_offset_m + tip_chord_m,
                },
            ],
        };
        planform.validate()?;
        Ok(planform)
    }

    /// Arbitrary station list. Validation enforces the spline contract.
    pub fn from_stations(stations: Vec<SpanStation>) -> Result<Self, SurfaceError> {
        let planform = Self { stations };
        planform.validate()?;
        Ok(planform)
    }

    /// Check the spline contract: at least two stations, `s` strictly
    /// increasing across exactly `[0, 1]`, finite positions, positive chord.
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if self.stations.len() < 2 {
            return Err(SurfaceError::InvalidStations(
                "planform needs at least two stations".into(),
            ));
        }
        let mut previous = f64::NEG_INFINITY;
        for station in &self.stations {
            if !station.s.is_finite() || !station.x_le.is_finite() || !station.x_te.is_finite() {
                return Err(SurfaceError::InvalidStations(format!(
                    "planform station s={} has non-finite geometry",
                    station.s
                )));
            }
            if station.s <= previous {
                return Err(SurfaceError::InvalidStations(format!(
                    "planform stations must strictly increase in s (got {} after {})",
                    station.s, previous
                )));
            }
            if station.x_te - station.x_le <= 0.0 {
                return Err(SurfaceError::InvalidChord(format!(
                    "planform chord must be positive at s={} (le={}, te={})",
                    station.s, station.x_le, station.x_te
                )));
            }
            previous = station.s;
        }
        if self.stations.first().map(|station| station.s) != Some(0.0)
            || self.stations.last().map(|station| station.s) != Some(1.0)
        {
            return Err(SurfaceError::InvalidStations(
                "planform stations must span exactly s=0..=1".into(),
            ));
        }
        Ok(())
    }

    /// Leading-edge position at `s` by piecewise-linear interpolation.
    pub fn leading_edge(&self, s: f64) -> f64 {
        self.interpolate(s, |station| station.x_le)
    }

    /// Trailing-edge position at `s` by piecewise-linear interpolation.
    pub fn trailing_edge(&self, s: f64) -> f64 {
        self.interpolate(s, |station| station.x_te)
    }

    /// Chord `x_te - x_le` at `s`. Positive wherever validation passed.
    pub fn chord(&self, s: f64) -> f64 {
        self.trailing_edge(s) - self.leading_edge(s)
    }

    fn interpolate(&self, s: f64, pick: impl Fn(&SpanStation) -> f64) -> f64 {
        let s = s.clamp(0.0, 1.0);
        let stations = &self.stations;
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
}
