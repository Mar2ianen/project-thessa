//! Mechanisms layered over geometry: control regions and fold joints.
//!
//! Mechanization is independent from geometry. A control region is drawn on
//! top of the parent surface (span interval plus chordwise boundaries plus
//! hinge line); a fold joint divides the surface into an inboard parent and
//! an outboard child that receives a rigid transform. Both force hard
//! panelization splits: no compiled panel may straddle regions that can move
//! independently.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::SurfaceError;

/// What kind of mechanism a control region is.
///
/// A trailing-edge device deflects panels about the hinge line at runtime.
/// An all-moving surface (stabilator, all-moving tail) rotates the whole
/// surface rigidly instead; the region covers the full span and chord so
/// the runtime can address every panel, and this marker tells it to rotate
/// rather than deflect. Kinematics beyond the marker (pivot axis, actuator
/// rate) are runtime/FBW concerns, recorded here only as data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ControlRegionKind {
    /// Hinged trailing/leading-edge device (aileron, flap, slat, ...).
    #[default]
    TrailingEdgeDevice,
    /// Whole-surface rotation (stabilator, all-moving tail).
    AllMovingSurface,
}

/// A hinged control region drawn on the parent surface.
///
/// Boundaries follow the parent geometry: span interval plus normalized
/// chordwise boundaries, so curved cuts are representable by stacking
/// regions, not by assuming rectangular flaps. A region may nest inside
/// another (trim tab inside an elevator); nesting is a parent index, never
/// a special case.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlRegion {
    /// Unique region name; becomes the control-surface definition name.
    pub name: String,
    /// Spanwise interval `(start, end)` in `s`, inside `[0, 1]` with
    /// positive width. Tip controls routinely run to `s = 1`.
    pub span: (f64, f64),
    /// Chordwise interval `(start, end)` in normalized `u in [0, 1]`
    /// (leading edge to trailing edge).
    pub chord: (f64, f64),
    /// Hinge line position as a chord fraction in `u`. Must lie inside
    /// `chord` for a plain hinge; slat-style translation presets may place
    /// it ahead of the region, which a future kinematic preset will own.
    pub hinge_u: f64,
    /// Most negative deflection in radians (`<= 0`; exactly `0` is a
    /// one-sided device such as a spoiler, legal since the sim-core limit
    /// model parks negative commands at zero).
    pub min_deflection_rad: f64,
    /// Most positive deflection in radians (must be `> 0`).
    pub max_deflection_rad: f64,
    /// Index of the enclosing region for nested tabs, `None` for top level.
    /// The child interval must sit inside the parent interval.
    #[serde(default)]
    pub parent: Option<usize>,
    /// Trailing-edge device or whole-surface rotation marker.
    #[serde(default)]
    pub kind: ControlRegionKind,
}

/// A fold joint at one span station.
///
/// The outboard child keeps its authored planform, bend, sections, and
/// nested mechanisms; compilation applies a rigid rotation about the hinge
/// axis through the hinge point. Material area is therefore invariant under
/// folding while the external envelope changes exactly by the transform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FoldJoint {
    /// Unique joint name.
    pub name: String,
    /// Fold boundary station, strictly inside `(0, 1)`: a joint at the
    /// exact root or tip would leave one side empty.
    pub station_s: f64,
    /// Hinge axis in surface-local coordinates (need not be unit; must be
    /// non-zero and finite). A 777X-style tip fold uses a roughly
    /// chordwise axis; the compiler normalizes.
    pub axis: DVec3,
    /// Flight (deployed) angle in radians: the reference state the
    /// authoring geometry is drawn in.
    pub deployed_angle_rad: f64,
    /// Stowed angle in radians. Deploying to `deployed_angle_rad`
    /// reconstructs the as-drawn flight geometry exactly.
    pub stowed_angle_rad: f64,
    /// Hard rotation limit, symmetric about the deployed angle. The stowed
    /// angle must lie within it.
    pub travel_limit_rad: f64,
}

impl ControlRegion {
    /// Validate one region against the surface and its siblings.
    ///
    /// `siblings` carries the already-validated regions so nesting can be
    /// checked structurally (parent index in range, child inside parent).
    pub fn validate(&self, siblings: &[ControlRegion]) -> Result<(), SurfaceError> {
        if self.name.trim().is_empty() {
            return Err(SurfaceError::InvalidControlRegion(
                "control region needs a non-empty name".into(),
            ));
        }
        let ((s0, s1), (u0, u1)) = (self.span, self.chord);
        for (label, value) in [
            ("span start", s0),
            ("span end", s1),
            ("chord start", u0),
            ("chord end", u1),
            ("hinge u", self.hinge_u),
        ] {
            if !value.is_finite() {
                return Err(SurfaceError::InvalidControlRegion(format!(
                    "{} '{}' has non-finite {label}",
                    "control region", self.name
                )));
            }
        }
        if !(0.0..=1.0).contains(&s0) || !(0.0..=1.0).contains(&s1) || s0 >= s1 {
            return Err(SurfaceError::InvalidControlRegion(format!(
                "control region '{}' needs a positive span interval inside [0, 1] (got {s0}..{s1})",
                self.name
            )));
        }
        if !(0.0..=1.0).contains(&u0) || !(0.0..=1.0).contains(&u1) || u0 >= u1 {
            return Err(SurfaceError::InvalidControlRegion(format!(
                "control region '{}' needs a positive chord interval inside [0, 1] (got {u0}..{u1})",
                self.name
            )));
        }
        if !(u0..=u1).contains(&self.hinge_u) {
            return Err(SurfaceError::InvalidControlRegion(format!(
                "control region '{}' hinge u={} must lie inside its chord interval {u0}..{u1}",
                self.name, self.hinge_u
            )));
        }
        if !self.min_deflection_rad.is_finite()
            || !self.max_deflection_rad.is_finite()
            || self.min_deflection_rad > 0.0
            || self.max_deflection_rad <= 0.0
        {
            return Err(SurfaceError::InvalidControlRegion(format!(
                "control region '{}' needs min <= 0 < max deflection limits",
                self.name
            )));
        }
        if let Some(parent) = self.parent {
            let enclosing = siblings.get(parent).ok_or_else(|| {
                SurfaceError::InvalidControlRegion(format!(
                    "control region '{}' references missing parent index {parent}",
                    self.name
                ))
            })?;
            if enclosing.parent.is_some() {
                return Err(SurfaceError::InvalidControlRegion(format!(
                    "control region '{}' nests deeper than one level, which this slice does not support",
                    self.name
                )));
            }
            let ((p0, p1), (q0, q1)) = (enclosing.span, enclosing.chord);
            if s0 < p0 || s1 > p1 || u0 < q0 || u1 > q1 {
                return Err(SurfaceError::InvalidControlRegion(format!(
                    "control region '{}' must sit inside parent '{}' (child {s0}..{s1} x {u0}..{u1}, parent {p0}..{p1} x {q0}..{q1})",
                    self.name, enclosing.name
                )));
            }
        }
        Ok(())
    }

    /// Nesting depth: `0` for a top-level region, `1` for a nested tab.
    pub fn depth(&self) -> usize {
        usize::from(self.parent.is_some())
    }
}

impl FoldJoint {
    /// Validate the joint: station strictly inboard of the tip, usable
    /// axis, finite angles, stowed inside the travel limit.
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if self.name.trim().is_empty() {
            return Err(SurfaceError::InvalidFoldJoint(
                "fold joint needs a non-empty name".into(),
            ));
        }
        if !self.station_s.is_finite() || self.station_s <= 0.0 || self.station_s >= 1.0 {
            return Err(SurfaceError::InvalidFoldJoint(format!(
                "fold joint '{}' station must lie inside (0, 1) (got {})",
                self.name, self.station_s
            )));
        }
        if !self.axis.is_finite() || self.axis.length_squared() <= f64::EPSILON {
            return Err(SurfaceError::InvalidFoldJoint(format!(
                "fold joint '{}' needs a non-zero finite hinge axis",
                self.name
            )));
        }
        for (label, value) in [
            ("deployed angle", self.deployed_angle_rad),
            ("stowed angle", self.stowed_angle_rad),
            ("travel limit", self.travel_limit_rad),
        ] {
            if !value.is_finite() {
                return Err(SurfaceError::InvalidFoldJoint(format!(
                    "fold joint '{}' has non-finite {label}",
                    self.name
                )));
            }
        }
        if self.travel_limit_rad <= 0.0 {
            return Err(SurfaceError::InvalidFoldJoint(format!(
                "fold joint '{}' needs a positive travel limit",
                self.name
            )));
        }
        if (self.stowed_angle_rad - self.deployed_angle_rad).abs() > self.travel_limit_rad {
            return Err(SurfaceError::InvalidFoldJoint(format!(
                "fold joint '{}' stowed angle lies outside its travel limit",
                self.name
            )));
        }
        Ok(())
    }
}
