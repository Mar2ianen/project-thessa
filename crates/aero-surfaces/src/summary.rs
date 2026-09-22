//! Geometry-only telemetry record for goldens and debug views.
//!
//! The summary carries no forces, only compiled geometry: bounding box,
//! spans, areas, chords, sweep, mechanism records, panel counts, area sums,
//! and the ownership graph. Golden reconstruction tests compare it against
//! independent integrations plus public reference values.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::compile::projected_planform_area;
use crate::{CompiledSurface, ControlRegionKind, ProceduralSurface};

/// One control region's compiled footprint: panel ownership, area, and
/// hinge position (design doc section 12.3 golden record).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlSummary {
    /// Region name from authoring.
    pub name: String,
    /// Panels owned innermost by the region.
    pub panel_count: usize,
    /// Owned area in m^2.
    pub area_m2: f64,
    /// Hinge line position as a chord fraction from authoring.
    pub hinge_u: f64,
    /// Trailing-edge device or whole-surface rotation marker.
    pub kind: ControlRegionKind,
}

/// Compact deterministic record of one compilation.
///
/// All lengths in metres, areas in square metres, angles in radians. The
/// bounding box covers the folded panel corners in body metres for the
/// compiled mechanism state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CompiledSurfaceSummary {
    /// Surface name from authoring.
    pub surface_name: String,
    /// Compiled panel count.
    pub panel_count: usize,
    /// Material surface area: sum of actual 3D panel areas.
    pub material_area_m2: f64,
    /// Projected planform area: sum of panel areas projected onto the
    /// flat `x/y` plane.
    pub projected_area_m2: f64,
    /// Authored tip span (flat-planform extent before bending).
    pub span_m: f64,
    /// Projected span: straight-line root-to-tip distance in the `x/y`
    /// plane after bend and fold transforms.
    pub projected_span_m: f64,
    /// Root chord in metres.
    pub root_chord_m: f64,
    /// Tip chord in metres.
    pub tip_chord_m: f64,
    /// Mean aerodynamic chord from geometry: `integral(c^2 dy) / S`.
    pub mean_aerodynamic_chord_m: f64,
    /// Overall leading-edge sweep in radians.
    pub sweep_rad: f64,
    /// Mapped bend elevation `Z(s)` at quarter stations (root to tip),
    /// after arc-length normalization.
    pub bend_profile_m: [f64; 5],
    /// Bounding-box minimum corner in body metres.
    pub bbox_min_m: DVec3,
    /// Bounding-box maximum corner in body metres.
    pub bbox_max_m: DVec3,
    /// Per control region: owned panel count, area, and hinge position.
    pub control_areas: Vec<ControlSummary>,
    /// Per fold joint: `(name, station, compiled angle)`.
    pub fold_states: Vec<(String, f64, f64)>,
    /// Panels carrying no control ownership.
    pub uncontrolled_panel_count: usize,
    /// Certified error estimate over the compiled zones (m^2-equivalent):
    /// the Richardson total the greedy budget refines under, or the
    /// post-hoc certification pass for tolerance subdivision. Exact on
    /// linear inputs up to floating-point summation; compare compilations
    /// through it, never against a second call into the compiler.
    pub estimated_error_m2: f64,
}

impl CompiledSurfaceSummary {
    /// Build the record from a freshly compiled surface. `bbox` corners are
    /// surface-local folded panel corners; mounting applies afterwards.
    /// `estimated_error_m2` is the refinement total (budget mode) or the
    /// post-hoc certification pass (tolerance mode).
    pub(crate) fn build(
        surface: &ProceduralSurface,
        compiled: &CompiledSurface,
        bbox_min: DVec3,
        bbox_max: DVec3,
        estimated_error_m2: f64,
    ) -> Self {
        let mut material = 0.0;
        let mut uncontrolled = 0;
        for (panel, tag) in compiled.panels.iter().zip(compiled.tags.iter()) {
            material += panel.area_m2;
            if tag.control.is_none() {
                uncontrolled += 1;
            }
        }
        // Projection is planform-plus-bend geometry, independent of
        // panelization and of section incidence (corners never tilt with
        // incidence, so neither may the projected area).
        let projected = projected_planform_area(surface);
        let mut control_areas = Vec::with_capacity(surface.controls.len());
        for (index, region) in surface.controls.iter().enumerate() {
            let (mut count, mut area) = (0, 0.0);
            for (panel, tag) in compiled.panels.iter().zip(compiled.tags.iter()) {
                if tag.control == Some(index) {
                    count += 1;
                    area += panel.area_m2;
                }
            }
            control_areas.push(ControlSummary {
                name: region.name.clone(),
                panel_count: count,
                area_m2: area,
                hinge_u: region.hinge_u,
                kind: region.kind,
            });
        }
        let fold_states = compiled
            .folds
            .iter()
            .map(|fold| {
                let station = surface
                    .folds
                    .iter()
                    .find(|joint| joint.name == fold.name)
                    .map(|joint| joint.station_s)
                    .unwrap_or(f64::NAN);
                (fold.name.clone(), station, fold.angle_rad)
            })
            .collect();
        // Projected span is the true folded-corner extent along the span
        // axis: bending/folding can only shrink it from the authored span.
        let projected_span = bbox_max.y - bbox_min.y;
        // Mapped bend profile: arc-length normalization rescales authored
        // elevations so the material root-to-tip length is the span.
        let bend_k = surface.bend.material_scale(surface.span_m);
        let bend_profile_m = [
            bend_k * surface.bend.elevation(0.0),
            bend_k * surface.bend.elevation(0.25),
            bend_k * surface.bend.elevation(0.5),
            bend_k * surface.bend.elevation(0.75),
            bend_k * surface.bend.elevation(1.0),
        ];
        Self {
            surface_name: surface.name.clone(),
            panel_count: compiled.panels.len(),
            material_area_m2: material,
            projected_area_m2: projected,
            span_m: surface.span_m,
            projected_span_m: projected_span,
            root_chord_m: surface.planform.chord(0.0),
            tip_chord_m: surface.planform.chord(1.0),
            mean_aerodynamic_chord_m: mean_aerodynamic_chord(surface),
            sweep_rad: (surface.planform.leading_edge(1.0) - surface.planform.leading_edge(0.0))
                .atan2(bend_k * surface.span_m),
            bend_profile_m,
            bbox_min_m: bbox_min,
            bbox_max_m: bbox_max,
            control_areas,
            fold_states,
            uncontrolled_panel_count: uncontrolled,
            estimated_error_m2,
        }
    }

    /// Mirror across the body `x/z` plane (surface-local, pre-mount).
    pub(crate) fn mirrored(&self) -> Self {
        let mut mirrored = self.clone();
        let mirror = |point: DVec3| DVec3::new(point.x, -point.y, point.z);
        let mirrored_min = mirror(self.bbox_min_m);
        let mirrored_max = mirror(self.bbox_max_m);
        mirrored.bbox_min_m = mirrored_min.min(mirrored_max);
        mirrored.bbox_max_m = mirrored_min.max(mirrored_max);
        mirrored
    }

    /// Translate the bounding box by the body mount offset.
    pub(crate) fn translate(&mut self, origin: DVec3) {
        self.bbox_min_m += origin;
        self.bbox_max_m += origin;
    }

    /// Reflect the record across the body `y/z` plane (local aft to body
    /// forward map): bounding-box x swaps and fold angles negate with
    /// the hinge axes.
    pub(crate) fn reflect_x(&mut self) {
        let (neg_max, neg_min) = (-self.bbox_max_m.x, -self.bbox_min_m.x);
        self.bbox_min_m.x = neg_max;
        self.bbox_max_m.x = neg_min;
        for state in &mut self.fold_states {
            state.2 = -state.2;
        }
    }

    /// Rotate the bounding box by the mount roll: exact re-AABB over the
    /// eight rotated corners (a rotated box is not a box).
    pub(crate) fn rotate(&mut self, rotation: glam::DQuat) {
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        for corner in 0..8 {
            let point = DVec3::new(
                if corner & 1 == 0 {
                    self.bbox_min_m.x
                } else {
                    self.bbox_max_m.x
                },
                if corner & 2 == 0 {
                    self.bbox_min_m.y
                } else {
                    self.bbox_max_m.y
                },
                if corner & 4 == 0 {
                    self.bbox_min_m.z
                } else {
                    self.bbox_max_m.z
                },
            );
            let rotated = rotation * point;
            min = min.min(rotated);
            max = max.max(rotated);
        }
        self.bbox_min_m = min;
        self.bbox_max_m = max;
    }
}

/// Mean aerodynamic chord by fine deterministic sampling of the authored
/// planform: `MAC = integral(c^2 dy) / S` over the single surface.
/// Independent of the compiler subdivision; tests use coarser closed forms
/// instead.
fn mean_aerodynamic_chord(surface: &ProceduralSurface) -> f64 {
    const SAMPLES: usize = 2048;
    let mut integral = 0.0;
    for index in 0..SAMPLES {
        let a = index as f64 / SAMPLES as f64;
        let b = (index + 1) as f64 / SAMPLES as f64;
        let chord_a = surface.planform.chord(a);
        let chord_b = surface.planform.chord(b);
        integral += 0.5 * (chord_a.powi(2) + chord_b.powi(2)) * (b - a);
    }
    let area: f64 = {
        let mut area = 0.0;
        for index in 0..SAMPLES {
            let a = index as f64 / SAMPLES as f64;
            let b = (index + 1) as f64 / SAMPLES as f64;
            area += 0.5 * (surface.planform.chord(a) + surface.planform.chord(b)) * (b - a);
        }
        area * surface.span_m
    };
    if area <= 0.0 {
        return 0.0;
    }
    integral * surface.span_m / area
}
