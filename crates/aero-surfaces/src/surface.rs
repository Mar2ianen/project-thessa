//! One procedural surface: the authoring object.
//!
//! A surface owns its span, body mount, planform, bend, sections, and
//! mechanisms. It is the unit the hangar compiles and the unit tests pin.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{BendCurve, ControlRegion, FoldJoint, Planform, SectionData, SurfaceError};

/// Authoring-side procedural aerodynamic surface.
///
/// The flat planform is the parameter domain; bend maps it into 3D; sections
/// orient it; mechanisms divide it. The compiler turns this into panels.
/// Mirroring builds the opposite hand (right wing to left wing): geometry
/// maps `y -> -y` while lift stays up.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProceduralSurface {
    /// Surface name, unique within the vehicle being compiled.
    pub name: String,
    /// Tip span in metres: the flat-planform `y` extent before bending.
    pub span_m: f64,
    /// Body-frame position of the surface root (`s = 0`) in metres.
    pub origin_body_m: DVec3,
    /// Mirror across the body `x/z` plane (`y -> -y`) at compile time.
    /// One authored right wing plus the mirror flag yields the left wing.
    #[serde(default)]
    pub mirror_y: bool,
    /// Flat planform splines `x_le(s)`, `x_te(s)`.
    pub planform: Planform,
    /// Out-of-plane bend `z(s)`.
    pub bend: BendCurve,
    /// Per-station incidence, thickness, and profile references.
    pub sections: SectionData,
    /// Control regions drawn on the surface (may nest one level for tabs).
    #[serde(default)]
    pub controls: Vec<ControlRegion>,
    /// Fold joints from inboard to outboard order is not required; the
    /// compiler sorts them by station.
    #[serde(default)]
    pub folds: Vec<FoldJoint>,
}

impl ProceduralSurface {
    /// Flat rectangular wing with uniform section: the simplest surface.
    pub fn rectangular(
        name: impl Into<String>,
        span_m: f64,
        chord_m: f64,
        origin_body_m: DVec3,
    ) -> Result<Self, SurfaceError> {
        Self {
            name: name.into(),
            span_m,
            origin_body_m,
            mirror_y: false,
            planform: Planform::rectangular(chord_m)?,
            bend: BendCurve::flat(),
            sections: SectionData::uniform(0.0, 0.0)?,
            controls: Vec::new(),
            folds: Vec::new(),
        }
        .validated()
    }

    /// Check the whole authoring object: span, mount, component contracts,
    /// unique mechanism names, at most one fold per station side ordering.
    pub fn validate(&self) -> Result<(), SurfaceError> {
        if self.name.trim().is_empty() {
            return Err(SurfaceError::InvalidSurface(
                "surface needs a non-empty name".into(),
            ));
        }
        if !self.span_m.is_finite() || self.span_m <= 0.0 {
            return Err(SurfaceError::InvalidSurface(format!(
                "surface '{}' needs a positive finite span (got {})",
                self.name, self.span_m
            )));
        }
        if !self.origin_body_m.is_finite() {
            return Err(SurfaceError::InvalidSurface(format!(
                "surface '{}' has a non-finite body origin",
                self.name
            )));
        }
        self.planform.validate()?;
        self.bend.validate()?;
        self.sections.validate()?;
        for (index, region) in self.controls.iter().enumerate() {
            region.validate(&self.controls[..index])?;
        }
        let mut names = std::collections::HashSet::new();
        for region in &self.controls {
            if !names.insert(region.name.clone()) {
                return Err(SurfaceError::InvalidControlRegion(format!(
                    "duplicate control region name '{}' on surface '{}'",
                    region.name, self.name
                )));
            }
        }
        for joint in &self.folds {
            joint.validate()?;
            if !names.insert(joint.name.clone()) {
                return Err(SurfaceError::InvalidFoldJoint(format!(
                    "duplicate mechanism name '{}' on surface '{}'",
                    joint.name, self.name
                )));
            }
        }
        Ok(())
    }

    /// Validate on construction sites that build then check.
    fn validated(self) -> Result<Self, SurfaceError> {
        self.validate()?;
        Ok(self)
    }
}
