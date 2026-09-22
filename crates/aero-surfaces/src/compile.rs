//! Hangar compiler: authoring surface to solver-ready panels.
//!
//! The compiler samples the planform, bend, and section functions, splits at
//! every geometric and mechanism boundary, subdivides smooth intervals to
//! bound zone granularity, and derives per-zone solver inputs. Positions come
//! from planform plus bend exactly; orientation folds section incidence in;
//! ownership comes from the mechanism intervals.
//!
//! Error model: authoring stations are piecewise-linear splines, and every
//! per-zone derivation (trapezoid area on the spanwise material extent,
//! exact segment tangents, mid-interval frames, endpoint sweep) is exact on
//! linear inputs. Base splits sit on every authored station, so zones never
//! span kinks; compilation is exact with respect to the authored polylines
//! up to floating-point summation, at any tolerance. Tolerances therefore
//! bound zone granularity for solver locality (small flat zones track local
//! flow better than large ones) without moving geometry — pinned by the
//! stability test. Convergence toward an analytic reference comes from
//! refining the authoring stations — pinned by the convergence test.

use std::collections::BTreeMap;

use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use thessa_sim_core::{AeroPanel, ControlSurfaceDefinition};

use crate::summary::CompiledSurfaceSummary;
use crate::{CompiledStructure, ProceduralSurface, StructuralLayout, SurfaceError};

/// How smooth span intervals become zones.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RefinementMode {
    /// Fixed per-interval variation tolerances (chord fraction, bend,
    /// incidence, sweep angles). Zones are cheap and plentiful; the knob
    /// bounds zone granularity for solver locality (small flat zones
    /// track local flow better). This is the flight-surface default.
    Tolerance,
    /// Greedy error budget: starting from the hard-boundary intervals,
    /// repeatedly split the span leaf with the largest Richardson error
    /// estimate until the total estimate drops under `budget_m2` or the
    /// panel count hits `max_panels`. The budget is best-effort under the
    /// cap; the achieved total is reported as
    /// [`CompiledSurfaceSummary::estimated_error_m2`]. Yields the minimal
    /// panel set certified under the budget: batch, LOD, and background
    /// use where per-panel flow locality does not matter. Not for flight
    /// surfaces: a handful of huge exact-area zones still misstates local
    /// flow.
    ErrorBudget {
        /// Total acceptable error estimate in m^2-equivalent
        /// (area plus orientation/centroid penalties).
        budget_m2: f64,
        /// Hard panel cap bounding refinement (base boundary intervals
        /// always survive; the cap binds splits, not existence).
        max_panels: usize,
    },
}

/// Subdivision tolerances and mechanism state for one compilation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompileOptions {
    /// Split a span interval while the chord changes by more than this
    /// fraction across it.
    pub max_chord_change_frac: f64,
    /// Split while the bend tangent turns by more than this (radians).
    pub max_bend_angle_rad: f64,
    /// Split while the section incidence changes by more than this.
    pub max_incidence_change_rad: f64,
    /// Split while the leading-edge sweep changes by more than this.
    pub max_sweep_change_rad: f64,
    /// Hard recursion cap per base interval (interval count `<= 2^depth`).
    pub max_depth: u32,
    /// Tolerance subdivision or greedy error-budget optimization.
    pub mode: RefinementMode,
}

impl Default for CompileOptions {
    fn default() -> Self {
        Self {
            max_chord_change_frac: 0.01,
            max_bend_angle_rad: 0.5_f64.to_radians(),
            max_incidence_change_rad: 0.5_f64.to_radians(),
            max_sweep_change_rad: 0.5_f64.to_radians(),
            max_depth: 12,
            mode: RefinementMode::Tolerance,
        }
    }
}

impl CompileOptions {
    fn validate(&self) -> Result<(), SurfaceError> {
        for (label, value) in [
            ("max_chord_change_frac", self.max_chord_change_frac),
            ("max_bend_angle_rad", self.max_bend_angle_rad),
            ("max_incidence_change_rad", self.max_incidence_change_rad),
            ("max_sweep_change_rad", self.max_sweep_change_rad),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(SurfaceError::InvalidOptions(format!(
                    "compile option {label} must be positive and finite (got {value})"
                )));
            }
        }
        if self.max_depth > 20 {
            return Err(SurfaceError::InvalidOptions(format!(
                "compile option max_depth must be at most 20 (got {})",
                self.max_depth
            )));
        }
        if let RefinementMode::ErrorBudget {
            budget_m2,
            max_panels,
        } = &self.mode
        {
            if !budget_m2.is_finite() || *budget_m2 < 0.0 {
                return Err(SurfaceError::InvalidOptions(format!(
                    "error budget must be finite and non-negative (got {budget_m2})"
                )));
            }
            if *max_panels == 0 {
                return Err(SurfaceError::InvalidOptions(
                    "error budget max_panels must be at least 1".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Which fold angles to compile. Angles are parallel to
/// [`ProceduralSurface::folds`](crate::ProceduralSurface) in surface order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MechanismState {
    /// Per-joint angle in radians. Missing entries fall back to the joint's
    /// deployed angle; extra entries are rejected.
    pub fold_angles_rad: Vec<f64>,
}

impl MechanismState {
    /// Compile in the as-drawn flight configuration.
    pub fn deployed() -> Self {
        Self {
            fold_angles_rad: Vec::new(),
        }
    }

    /// Compile with every joint at its stowed angle.
    pub fn stowed(surface: &ProceduralSurface) -> Self {
        Self {
            fold_angles_rad: surface
                .folds
                .iter()
                .map(|joint| joint.stowed_angle_rad)
                .collect(),
        }
    }

    fn resolve(&self, surface: &ProceduralSurface) -> Result<Vec<f64>, SurfaceError> {
        if self.fold_angles_rad.len() > surface.folds.len() {
            return Err(SurfaceError::InvalidOptions(format!(
                "mechanism state has {} fold angles for {} joints",
                self.fold_angles_rad.len(),
                surface.folds.len()
            )));
        }
        surface
            .folds
            .iter()
            .enumerate()
            .map(|(index, joint)| {
                let angle = self
                    .fold_angles_rad
                    .get(index)
                    .copied()
                    .unwrap_or(joint.deployed_angle_rad);
                if !angle.is_finite() {
                    return Err(SurfaceError::InvalidOptions(format!(
                        "fold angle for joint '{}' must be finite",
                        joint.name
                    )));
                }
                if (angle - joint.deployed_angle_rad).abs() > joint.travel_limit_rad + 1e-9 {
                    return Err(SurfaceError::InvalidOptions(format!(
                        "fold angle for joint '{}' lies outside its travel limit",
                        joint.name
                    )));
                }
                Ok(angle)
            })
            .collect()
    }
}

/// Ownership of one compiled panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct PanelTag {
    /// Innermost control region owning the panel (index into the surface's
    /// `controls`), if any. A nested tab owns its panels, not the parent.
    pub control: Option<usize>,
    /// Direct parent region when `control` is a nested tab. Runtime
    /// composition (tab rides parent deflection) is a mixer concern; the
    /// compiler records the chain so the mixer can address it.
    pub control_parent: Option<usize>,
    /// Outboard-most fold joint outboard of whose station the panel sits
    /// (index into the surface's `folds`), if any.
    pub fold: Option<usize>,
}

/// One compiled fold joint: hinge placement plus compiled angle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledFold {
    /// Joint name from authoring.
    pub name: String,
    /// Hinge point in body metres (fold-station section, mid-chord).
    pub hinge_body_m: DVec3,
    /// Hinge axis in body coordinates, unit length.
    pub axis_body: DVec3,
    /// Compiled angle in radians.
    pub angle_rad: f64,
    /// As-drawn flight angle in the same conjugated sign convention as
    /// `angle_rad`, so the runtime delta `angle - deployed` is identity
    /// at the baked state even when the authored deployed angle is
    /// nonzero.
    pub deployed_angle_rad: f64,
    /// Parent joint in the fold hierarchy (surface-local index): the
    /// nearest joint inboard of this one, `None` for root joints. Panels
    /// stay tagged with their deepest joint while the chain rides here.
    pub parent_joint: Option<usize>,
}

/// Compiled surface: runtime-consumable output of the hangar step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledSurface {
    /// Solver panels, one per aerodynamic zone.
    pub panels: Vec<AeroPanel>,
    /// Ownership parallel to `panels`.
    pub tags: Vec<PanelTag>,
    /// One definition per control region (tabs included), referencing the
    /// panels each region owns innermost.
    pub controls: Vec<ControlSurfaceDefinition>,
    /// Compiled fold records in surface order.
    pub folds: Vec<CompiledFold>,
    /// Geometry-only telemetry record for goldens and debug views.
    pub summary: CompiledSurfaceSummary,
    /// Structural mass and fuel volume, present when the surface authors
    /// a [`StructuralLayout`].
    pub structure: Option<CompiledStructure>,
}

/// Surface-local folded point without compiler state: planform plus
/// arc-length-mapped span/elevation, then outboard-first rigid fold
/// rotations about as-drawn hinges. Shared by the compiler and the
/// fold-aware projection math so both see identical geometry.
#[allow(clippy::too_many_arguments)]
pub(crate) fn folded_point_raw(
    planform: &crate::Planform,
    bend: &crate::BendCurve,
    bend_k: f64,
    span_m: f64,
    folds: &[crate::FoldJoint],
    fold_order: &[usize],
    fold_angles: &[f64],
    s: f64,
    u: f64,
) -> DVec3 {
    let span_y = |v: f64| bend_k * v * span_m;
    let bend_z = |v: f64| bend_k * bend.elevation(v);
    let mut point = DVec3::new(
        planform.leading_edge(s) + u * planform.chord(s),
        span_y(s),
        bend_z(s),
    );
    for &order in fold_order {
        let joint = &folds[order];
        if s > joint.station_s {
            let hinge = DVec3::new(
                planform.leading_edge(joint.station_s) + 0.5 * planform.chord(joint.station_s),
                span_y(joint.station_s),
                bend_z(joint.station_s),
            );
            let axis = joint.axis.normalize();
            let angle = fold_angles[order] - joint.deployed_angle_rad;
            point = hinge + DQuat::from_axis_angle(axis, angle) * (point - hinge);
        }
    }
    point
}

/// Fold-aware top-view projection: shadow span extent and area over the
/// authored stations (folded Y is linear within mechanism regions, so
/// station samples are exact). Unsigned span steps: folded-back overlap
/// counts shadow once.
pub(crate) fn projected_folded_envelope(
    surface: &ProceduralSurface,
    fold_angles: &[f64],
) -> (f64, f64, f64) {
    let bend_k = surface.bend.material_scale(surface.span_m);
    let mut order: Vec<usize> = (0..surface.folds.len()).collect();
    order.sort_by(|&a, &b| {
        surface.folds[b]
            .station_s
            .partial_cmp(&surface.folds[a].station_s)
            .expect("validated finite")
    });
    let mut stations = vec![0.0, 1.0];
    for list in [
        surface
            .planform
            .stations
            .iter()
            .map(|station| station.s)
            .collect::<Vec<_>>(),
        surface
            .bend
            .stations
            .iter()
            .map(|station| station.s)
            .collect::<Vec<_>>(),
        surface
            .sections
            .stations
            .iter()
            .map(|station| station.s)
            .collect::<Vec<_>>(),
        surface
            .folds
            .iter()
            .map(|joint| joint.station_s)
            .collect::<Vec<_>>(),
    ] {
        stations.extend(list);
    }
    stations.sort_by(|a, b| a.partial_cmp(b).expect("validated finite"));
    stations.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    let folded_y = |s: f64| {
        folded_point_raw(
            &surface.planform,
            &surface.bend,
            bend_k,
            surface.span_m,
            &surface.folds,
            &order,
            fold_angles,
            s,
            0.5,
        )
        .y
    };
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    let mut area = 0.0;
    for pair in stations.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let (ya, yb) = (folded_y(a), folded_y(b));
        y_min = y_min.min(ya.min(yb));
        y_max = y_max.max(ya.max(yb));
        let chord_avg = 0.5 * (surface.planform.chord(a) + surface.planform.chord(b));
        area += chord_avg * (yb - ya).abs();
    }
    (y_min, y_max, area.max(1e-12))
}

impl CompiledSurface {
    /// Mirror across the body `x/z` plane: right hand to left hand.    ///
    /// Positions and axes map `y -> -y` with lift re-orthogonalized so it
    /// stays up; fold angles negate so equivalent commands keep mirrored
    /// surfaces mirrored. Areas and ownership are untouched.
    pub fn mirrored(&self) -> Self {
        let mirror_point = |point: DVec3| DVec3::new(point.x, -point.y, point.z);
        let panels = self
            .panels
            .iter()
            .map(|panel| {
                let chord = mirror_point(panel.chord_axis_body).normalize();
                let lifted = mirror_point(panel.lift_axis_body);
                let lift = (lifted - chord * lifted.dot(chord)).normalize();
                AeroPanel {
                    position_body_m: mirror_point(panel.position_body_m),
                    center_of_pressure_body_m: mirror_point(panel.center_of_pressure_body_m),
                    chord_axis_body: chord,
                    lift_axis_body: lift,
                    ..*panel
                }
            })
            .collect();
        let folds = self
            .folds
            .iter()
            .map(|fold| CompiledFold {
                name: fold.name.clone(),
                hinge_body_m: mirror_point(fold.hinge_body_m),
                axis_body: mirror_point(fold.axis_body).normalize(),
                angle_rad: -fold.angle_rad,
                deployed_angle_rad: -fold.deployed_angle_rad,
                // Joint indices are order-stable under mirroring.
                parent_joint: fold.parent_joint,
            })
            .collect();
        let mirrored = Self {
            panels,
            tags: self.tags.clone(),
            controls: self.controls.clone(),
            folds,
            summary: self.summary.mirrored(),
            structure: self.structure.as_ref().map(CompiledStructure::mirrored),
        };
        // Mirroring preserves panel order, so control definitions keep
        // addressing the same panel indices untouched.
        mirrored
    }
}

/// Compile one surface with options and mechanism state.
///
/// Result panels plug straight into
/// [`AeroGeometry`](thessa_sim_core::AeroGeometry); control definitions
/// plug into
/// [`VehicleDefinition`](thessa_sim_core::VehicleDefinition).
pub fn compile_surface(
    surface: &ProceduralSurface,
    options: &CompileOptions,
    mechanism: &MechanismState,
) -> Result<CompiledSurface, SurfaceError> {
    surface.validate()?;
    options.validate()?;
    let fold_angles = mechanism.resolve(surface)?;
    let compiler = Compiler::new(surface, options, fold_angles)?.with_aspect_ratio();
    compiler.compile()
}

struct Compiler<'a> {
    surface: &'a ProceduralSurface,
    options: &'a CompileOptions,
    fold_angles: Vec<f64>,
    fold_order: Vec<usize>,
    /// Arc-length rescale so the root-to-tip material length is `span_m`.
    bend_k: f64,
    /// Whole-surface aspect ratio (span^2 / projected area), computed
    /// once: every panel of the surface shares the physically meaningful
    /// planform value for the finite-surface correlation.
    surface_aspect_ratio: f64,
}

/// One spanwise refinement interval with its recursion depth.
#[derive(Debug, Clone, Copy)]
struct SpanLeaf {
    a: f64,
    b: f64,
    depth: u32,
}

/// Heap entry for greedy budget refinement: max-error first, ties broken
/// by span start so the split sequence is deterministic.
#[derive(Debug, Clone, Copy)]
struct HeapLeaf {
    error: f64,
    leaf: SpanLeaf,
}

impl HeapLeaf {
    fn new(error: f64, leaf: SpanLeaf) -> Self {
        Self { error, leaf }
    }

    fn sort_key(&self) -> (u64, u64, u64) {
        (
            self.error.to_bits(),
            self.leaf.a.to_bits(),
            self.leaf.b.to_bits(),
        )
    }
}

impl PartialEq for HeapLeaf {
    fn eq(&self, other: &Self) -> bool {
        self.sort_key() == other.sort_key()
    }
}

impl Eq for HeapLeaf {}

impl PartialOrd for HeapLeaf {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapLeaf {
    // Max-heap on error with the span interval as deterministic tie-break:
    // the greedy split sequence depends only on geometry, never on
    // insertion order.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.sort_key().cmp(&other.sort_key())
    }
}

/// Area centroid of a quadrilateral by two-triangle decomposition.
/// Exact on planar quads; a consistent higher-order center for
/// Richardson whole-vs-halves comparison and honest moment arms.
fn quad_centroid(corners: [DVec3; 4]) -> DVec3 {
    let [p00, p01, p10, p11] = corners;
    let area1 = (p10 - p00).cross(p11 - p00).length();
    let area2 = (p11 - p00).cross(p01 - p00).length();
    let total = area1 + area2;
    if total <= 0.0 {
        return (p00 + p01 + p10 + p11) / 4.0;
    }
    let centroid1 = (p00 + p10 + p11) / 3.0;
    let centroid2 = (p00 + p11 + p01) / 3.0;
    (centroid1 * area1 + centroid2 * area2) / total
}

/// Raw geometric quantities of one zone, backing both panels and the
/// Richardson estimate from a single code path.
#[derive(Debug, Clone, Copy)]
struct ZoneQuantities {
    area_m2: f64,
    centroid: DVec3,
    normal: DVec3,
}
/// Local-frame structural accumulator over emitted span strips:
/// mass-weighted centers plus a point-mass inertia about the
/// surface-local origin. The mount step carries the aggregates into the
/// body frame afterwards.
///
/// Two passes: `add_span_strip` records skin/web/rib/box per full-chord
/// span leaf (control-region chord cuts never reach structure);
/// `finish` sizes spar caps from the Schrenk-distributed limit bending
/// moment and folds every strip mass into the centers and inertia.
#[derive(Debug, Clone)]
struct StructAcc {
    full_area_coefficient: f64,
    box_area_coefficient: f64,
    skin_kg: f64,
    web_kg: f64,
    rib_kg: f64,
    box_m3: f64,
    box_numerator: DVec3,
    rib_inbox_m3: f64,
    zones: Vec<LoadZone>,
    cap_kg: f64,
    mass_kg: f64,
    com_numerator: DVec3,
    inertia_local: glam::DMat3,
}

/// Full-chord span-strip inputs for structure: the compiled strip panel
/// plus its spanwise endpoints, end chords, and mapped span positions.
struct StripInput<'a> {
    panel: &'a thessa_sim_core::AeroPanel,
    a: f64,
    b: f64,
    c_a: f64,
    c_b: f64,
    y_a: f64,
    y_b: f64,
}

impl StructAcc {
    fn new(layout: &StructuralLayout) -> Self {
        Self {
            full_area_coefficient: crate::structure::rib_area_coefficient(),
            box_area_coefficient: crate::structure::rib_area_coefficient_range(
                layout.fuel_box_chord.0,
                layout.fuel_box_chord.1,
            ),
            skin_kg: 0.0,
            web_kg: 0.0,
            rib_kg: 0.0,
            box_m3: 0.0,
            box_numerator: DVec3::ZERO,
            rib_inbox_m3: 0.0,
            zones: Vec::new(),
            cap_kg: 0.0,
            mass_kg: 0.0,
            com_numerator: DVec3::ZERO,
            // Explicit zero: glam matrix Default is identity, and an
            // accumulator starting at identity ghosts +1.0 diagonals.
            inertia_local: glam::DMat3::ZERO,
        }
    }

    /// One full-chord span strip: skin, webs, ribs, and fuel box over
    /// the whole chord. Exact chord-square integral for linear segments
    /// (`(ca^2 + ca*cb + cb^2)/3`, never the averaged-chord square);
    /// thickness stays a mid-strip sample, documented O(h^2).
    fn add_span_strip(&mut self, layout: &StructuralLayout, strip: &StripInput) {
        let panel = strip.panel;
        let s_mid = 0.5 * (strip.a + strip.b);
        let chord_sq_integral =
            (strip.c_a * strip.c_a + strip.c_a * strip.c_b + strip.c_b * strip.c_b) / 3.0
                * panel.span_m;
        let thickness_ratio = panel.thickness_to_chord_ratio;
        let skin_kg = 2.0
            * panel.area_m2
            * (layout.skin_gauge_mm / 1000.0)
            * layout.skin_material.density_kg_m3;
        let web_height_m = thickness_ratio * panel.chord_m * layout.spar_depth_fraction;
        let web_kg = 2.0
            * panel.span_m
            * web_height_m
            * (layout.spar_web_gauge_mm / 1000.0)
            * layout.spar_material.density_kg_m3;
        let rib_kg = chord_sq_integral
            * thickness_ratio
            * self.full_area_coefficient
            * (layout.rib_gauge_mm / 1000.0)
            * layout.skin_material.density_kg_m3
            / layout.rib_spacing_m;
        self.skin_kg += skin_kg;
        self.web_kg += web_kg;
        self.rib_kg += rib_kg;
        // Rib displacement counts only inside the fuel box: the plate
        // fraction over the box chord interval, not the whole section
        // (counting all of it systematically under-reports usable fuel).
        self.rib_inbox_m3 += chord_sq_integral
            * thickness_ratio
            * self.box_area_coefficient
            * (layout.rib_gauge_mm / 1000.0)
            / layout.rib_spacing_m;
        let (box_front, box_rear) = layout.fuel_box_chord;
        let box_m3 = (box_rear - box_front) * chord_sq_integral * thickness_ratio;
        self.box_m3 += box_m3;
        self.box_numerator += panel.center_of_pressure_body_m * box_m3;
        self.zones.push(LoadZone {
            s_mid,
            span_m: panel.span_m,
            chord_m: panel.chord_m,
            web_height_m,
            area_m2: panel.area_m2,
            centroid: panel.center_of_pressure_body_m,
            y_a: strip.y_a,
            y_b: strip.y_b,
            skin_kg,
            web_kg,
            rib_kg,
        });
    }

    /// Cap-sizing pass: Schrenk-distribute the limit lift, integrate
    /// shear and moment tip-to-root, size caps from allowable stress,
    /// then fold all zone masses into centers and inertia.
    fn finish(&mut self, layout: &StructuralLayout) {
        let total_area: f64 = self.zones.iter().map(|zone| zone.area_m2).sum();
        let total_span: f64 = self.zones.iter().map(|zone| zone.span_m).sum();
        let mean_chord = if total_span > 0.0 {
            total_area / total_span
        } else {
            0.0
        };
        let allowable_pa = layout.spar_material.allowable_stress_mpa * 1.0e6;
        // Outboard-first order for the shear/moment integration.
        let mut order: Vec<usize> = (0..self.zones.len()).collect();
        order.sort_by(|&a, &b| {
            self.zones[b]
                .s_mid
                .partial_cmp(&self.zones[a].s_mid)
                .expect("validated finite")
        });
        let mut weights = vec![0.0; self.zones.len()];
        let mut weight_sum = 0.0;
        for (index, zone) in self.zones.iter().enumerate() {
            let share = crate::structure::schrenk_share(zone.chord_m, mean_chord, zone.s_mid);
            weights[index] = share * zone.span_m;
            weight_sum += weights[index];
        }
        let mut shear_n = 0.0;
        let mut moment_nm = 0.0;
        for &index in &order {
            let zone = &self.zones[index];
            let lift_n = if weight_sum > 0.0 {
                layout.design_limit_lift_n * weights[index] / weight_sum
            } else {
                0.0
            };
            let y_centroid = 0.5 * (zone.y_a + zone.y_b);
            // Moment at the inboard edge: outboard shear over the outer
            // segment plus grown shear over the inner segment.
            moment_nm +=
                shear_n * (zone.y_b - y_centroid) + (shear_n + lift_n) * (y_centroid - zone.y_a);
            shear_n += lift_n;
            let cap_area =
                crate::structure::cap_area_m2(moment_nm, allowable_pa, zone.web_height_m);
            let cap_kg = layout.spar_material.density_kg_m3 * 2.0 * cap_area * zone.span_m;
            let zone_mass = zone.skin_kg + zone.web_kg + zone.rib_kg + cap_kg;
            self.cap_kg += cap_kg;
            self.mass_kg += zone_mass;
            self.com_numerator += zone.centroid * zone_mass;
            self.inertia_local += point_inertia(zone_mass, zone.centroid);
        }
    }
}

/// One zone's load-relevant data for the cap-sizing pass.
#[derive(Debug, Clone, Copy)]
struct LoadZone {
    s_mid: f64,
    span_m: f64,
    chord_m: f64,
    web_height_m: f64,
    area_m2: f64,
    centroid: DVec3,
    y_a: f64,
    y_b: f64,
    skin_kg: f64,
    web_kg: f64,
    rib_kg: f64,
}

/// Point-mass inertia about the origin: `m(|r|^2 I - r r^T)`.
fn point_inertia(mass_kg: f64, center: DVec3) -> glam::DMat3 {
    let outer = glam::DMat3::from_cols(center * center.x, center * center.y, center * center.z);
    (glam::DMat3::from_diagonal(DVec3::splat(center.length_squared())) - outer) * mass_kg
}

impl<'a> Compiler<'a> {
    fn new(
        surface: &'a ProceduralSurface,
        options: &'a CompileOptions,
        fold_angles: Vec<f64>,
    ) -> Result<Self, SurfaceError> {
        let mut fold_order: Vec<usize> = (0..surface.folds.len()).collect();
        fold_order.sort_by(|&a, &b| {
            surface.folds[b]
                .station_s
                .partial_cmp(&surface.folds[a].station_s)
                .expect("validated finite")
        });
        Ok(Self {
            surface,
            options,
            fold_angles,
            fold_order,
            bend_k: surface.bend.material_scale(surface.span_m),
            surface_aspect_ratio: 0.0,
        })
    }
    /// Finish construction with the surface aspect ratio. Split out so
    /// `new` stays infallible scaffolding around the fallible resolve.
    /// Full-aircraft shadow geometry for pairs (twice the half span
    /// over twice the half area), own shadow for singles: aspect ratio
    /// follows the folded state, because a folded tip flies (or taxis)
    /// behind different normals than the deployed correlation.
    fn with_aspect_ratio(mut self) -> Self {
        let (y_min, y_max, projected) = projected_folded_envelope(self.surface, &self.fold_angles);
        let pair_factor = match self.surface.topology {
            crate::SurfaceTopology::SymmetricHalf => 2.0,
            crate::SurfaceTopology::Single => 1.0,
        };
        let shadow_span = (y_max - y_min).max(1e-12);
        // Pairing doubles half span and half area alike, which folds to
        // the pair factor: AR = (2b)^2/(2S) = 2b^2/S.
        self.surface_aspect_ratio = pair_factor * shadow_span * shadow_span / projected;
        self
    }

    /// Mapped spanwise position: horizontal projection after arc-length
    /// normalization. Bending shrinks this from the authored `s * span_m`.
    fn span_y(&self, s: f64) -> f64 {
        self.bend_k * s * self.surface.span_m
    }

    /// Mapped elevation after arc-length normalization.
    fn bend_z(&self, s: f64) -> f64 {
        self.bend_k * self.surface.bend.elevation(s)
    }

    fn compile(&self) -> Result<CompiledSurface, SurfaceError> {
        let splits = self.base_splits();
        let (leaves, estimated_error_m2) = match &self.options.mode {
            RefinementMode::Tolerance => {
                let mut leaves = Vec::new();
                for pair in splits.windows(2) {
                    self.subdivide(pair[0], pair[1], 0, &mut leaves);
                }
                // Post-hoc certification pass over the final leaves.
                let mut estimated = 0.0;
                for leaf in &leaves {
                    estimated += self.leaf_error(leaf.a, leaf.b)?;
                }
                (leaves, estimated)
            }
            RefinementMode::ErrorBudget {
                budget_m2,
                max_panels,
            } => self.refine_by_budget(*budget_m2, *max_panels, &splits)?,
        };
        let mut compiled = CompiledSurface {
            panels: Vec::new(),
            tags: Vec::new(),
            controls: Vec::new(),
            folds: self.compiled_folds(),
            summary: CompiledSurfaceSummary::default(),
            structure: None,
        };
        let mut bbox_min = DVec3::splat(f64::INFINITY);
        let mut bbox_max = DVec3::splat(f64::NEG_INFINITY);
        let mut struct_acc = None;
        let layout = self.surface.structure.clone();
        if let Some(layout) = &layout {
            struct_acc = Some(StructAcc::new(layout));
        }
        for leaf in &leaves {
            // Structural strips run on span leaves BEFORE chordwise
            // aero splits: spars, ribs, and fuel boxes span the full
            // chord and must never learn about control-region cuts. A
            // zero-mass control region drawn on the wing must not move
            // structural mass by a single gram (regression-pinned).
            if let (Some(layout), Some(acc)) = (&layout, &mut struct_acc) {
                let (strip, _) = self.zone_panel(leaf.a, leaf.b, 0.0, 1.0)?;
                acc.add_span_strip(
                    layout,
                    &StripInput {
                        panel: &strip,
                        a: leaf.a,
                        b: leaf.b,
                        c_a: self.surface.planform.chord(leaf.a),
                        c_b: self.surface.planform.chord(leaf.b),
                        y_a: self.span_y(leaf.a),
                        y_b: self.span_y(leaf.b),
                    },
                );
            }
            for zone in self.chord_zones(leaf.a, leaf.b) {
                let (mut panel, corners) = self.zone_panel(leaf.a, leaf.b, zone.0, zone.1)?;
                // Local fold ownership rides into the baked panel; the
                // baker rebases it onto the merged joint list.
                panel.fold_index = zone.2.fold;
                for corner in &corners {
                    bbox_min = bbox_min.min(*corner);
                    bbox_max = bbox_max.max(*corner);
                }
                compiled.tags.push(zone.2);
                compiled.panels.push(panel);
            }
        }
        if compiled.panels.is_empty() {
            return Err(SurfaceError::PanelRejected(
                "compilation produced no panels".into(),
            ));
        }
        compiled.controls = self.control_definitions(&compiled.tags)?;
        let (_, _, projected_area_m2) = projected_folded_envelope(self.surface, &self.fold_angles);
        compiled.summary = CompiledSurfaceSummary::build(
            self.surface,
            &compiled,
            bbox_min,
            bbox_max,
            estimated_error_m2,
            projected_area_m2,
        );
        let mut compiled = self.mount(compiled);
        if let (Some(layout), Some(mut acc)) = (layout, struct_acc) {
            acc.finish(&layout);
            compiled.structure = Some(self.mount_structure(acc, &layout));
        }
        Ok(compiled)
    }

    /// Greedy error-budget refinement: split the highest-error leaf until
    /// the total estimate fits the budget or the panel cap binds.
    ///
    /// The heap orders by `(error, span start)` so ties break
    /// deterministically; the emitted leaf order is re-sorted by span.
    /// Returns the final leaves plus their total error estimate.
    fn refine_by_budget(
        &self,
        budget_m2: f64,
        max_panels: usize,
        splits: &[f64],
    ) -> Result<(Vec<SpanLeaf>, f64), SurfaceError> {
        use std::collections::BinaryHeap;

        let mut heap = BinaryHeap::new();
        let mut total_error = 0.0;
        let mut panel_count = 0;
        for pair in splits.windows(2) {
            let leaf = SpanLeaf {
                a: pair[0],
                b: pair[1],
                depth: 0,
            };
            let error = self.leaf_error(leaf.a, leaf.b)?;
            total_error += error;
            panel_count += self.chord_zones(leaf.a, leaf.b).len();
            heap.push(HeapLeaf::new(error, leaf));
        }
        let mut frozen: Vec<SpanLeaf> = Vec::new();
        let mut frozen_error = 0.0;
        while total_error > budget_m2 {
            let Some(best) = heap.pop() else { break };
            let split_panels = panel_count - self.chord_zones(best.leaf.a, best.leaf.b).len();
            let mid = 0.5 * (best.leaf.a + best.leaf.b);
            let left = SpanLeaf {
                a: best.leaf.a,
                b: mid,
                depth: best.leaf.depth + 1,
            };
            let right = SpanLeaf {
                a: mid,
                b: best.leaf.b,
                depth: best.leaf.depth + 1,
            };
            let new_panels = split_panels
                + self.chord_zones(left.a, left.b).len()
                + self.chord_zones(right.a, right.b).len();
            if best.leaf.depth >= self.options.max_depth || new_panels > max_panels {
                // Unsplittable: retire the leaf from play, carrying its
                // error out of the running total into the frozen sum exactly
                // once (double counting here inflates the reported total).
                total_error -= best.error;
                frozen.push(best.leaf);
                frozen_error += best.error;
                continue;
            }
            let left_error = self.leaf_error(left.a, left.b)?;
            let right_error = self.leaf_error(right.a, right.b)?;
            total_error = total_error - best.error + left_error + right_error;
            panel_count = new_panels;
            heap.push(HeapLeaf::new(left_error, left));
            heap.push(HeapLeaf::new(right_error, right));
        }
        let mut leaves: Vec<SpanLeaf> = heap.into_iter().map(|entry| entry.leaf).collect();
        leaves.extend(frozen);
        leaves.sort_by(|x, y| {
            x.a.partial_cmp(&y.a)
                .expect("validated finite")
                .then(x.b.partial_cmp(&y.b).expect("validated finite"))
        });
        Ok((leaves, total_error + frozen_error))
    }

    /// Richardson error estimate for one span leaf: the whole-leaf zone
    /// quantities against the sum of its halves, over identical chord
    /// segments. Exact on linear inputs (estimate ~0); the triangle
    /// inequality makes the leaf sum a conservative total.
    fn leaf_error(&self, a: f64, b: f64) -> Result<f64, SurfaceError> {
        let mid = 0.5 * (a + b);
        let mut error = 0.0;
        for zone in self.chord_zones(a, b) {
            let (u0, u1) = (zone.0, zone.1);
            let whole = self.zone_quantities(a, b, u0, u1)?;
            let left = self.zone_quantities(a, mid, u0, u1)?;
            let right = self.zone_quantities(mid, b, u0, u1)?;
            let area_halves = left.area_m2 + right.area_m2;
            let area_error = (whole.area_m2 - area_halves).abs();
            let normal_halves =
                (left.normal * left.area_m2 + right.normal * right.area_m2).normalize();
            let normal_error = whole.normal.angle_between(normal_halves);
            let centroid_halves = if area_halves > 0.0 {
                (left.centroid * left.area_m2 + right.centroid * right.area_m2) / area_halves
            } else {
                whole.centroid
            };
            let centroid_error = (whole.centroid - centroid_halves).length();
            error +=
                area_error + whole.area_m2 * normal_error + centroid_error * whole.area_m2.sqrt();
        }
        Ok(error)
    }

    /// Raw geometric quantities of one zone, backing both panels and the
    /// Richardson estimate from a single code path.
    fn zone_quantities(
        &self,
        a: f64,
        b: f64,
        u0: f64,
        u1: f64,
    ) -> Result<ZoneQuantities, SurfaceError> {
        let (panel, _) = self.zone_panel(a, b, u0, u1)?;
        Ok(ZoneQuantities {
            area_m2: panel.area_m2,
            centroid: panel.center_of_pressure_body_m,
            normal: panel.lift_axis_body,
        })
    }

    /// Hard split stations: endpoints, every authored station list, every
    /// control span bound, every fold station.
    fn base_splits(&self) -> Vec<f64> {
        let mut splits = vec![0.0, 1.0];
        let push_stations = |splits: &mut Vec<f64>, stations: &[f64]| {
            splits.extend(stations.iter().copied());
        };
        push_stations(
            &mut splits,
            &self
                .surface
                .planform
                .stations
                .iter()
                .map(|station| station.s)
                .collect::<Vec<_>>(),
        );
        push_stations(
            &mut splits,
            &self
                .surface
                .bend
                .stations
                .iter()
                .map(|station| station.s)
                .collect::<Vec<_>>(),
        );
        push_stations(
            &mut splits,
            &self
                .surface
                .sections
                .stations
                .iter()
                .map(|station| station.s)
                .collect::<Vec<_>>(),
        );
        for region in &self.surface.controls {
            splits.push(region.span.0);
            splits.push(region.span.1);
        }
        for joint in &self.surface.folds {
            splits.push(joint.station_s);
        }
        splits.sort_by(|a, b| a.partial_cmp(b).expect("validated finite"));
        let mut deduped: Vec<f64> = Vec::with_capacity(splits.len());
        for split in splits {
            if deduped
                .last()
                .is_none_or(|last: &f64| (split - *last).abs() > 1e-12)
            {
                deduped.push(split);
            }
        }
        deduped
    }

    /// Recursively split `[a, b]` until the smooth-interval metrics pass.
    fn subdivide(&self, a: f64, b: f64, depth: u32, leaves: &mut Vec<SpanLeaf>) {
        if depth >= self.options.max_depth || !self.needs_split(a, b) {
            leaves.push(SpanLeaf { a, b, depth });
            return;
        }
        let mid = 0.5 * (a + b);
        self.subdivide(a, mid, depth + 1, leaves);
        self.subdivide(mid, b, depth + 1, leaves);
    }
    fn needs_split(&self, a: f64, b: f64) -> bool {
        if b - a <= 1e-12 {
            return false;
        }
        let planform = &self.surface.planform;
        let chord_a = planform.chord(a);
        let chord_b = planform.chord(b);
        let chord_ref = chord_a.max(chord_b).max(1e-9);
        if (chord_b - chord_a).abs() / chord_ref > self.options.max_chord_change_frac {
            return true;
        }
        // Secant metrics on (a, mid, b): every evaluated slope lies
        // strictly inside [a, b], so kink vertices at the endpoints can
        // never impersonate interior curvature. One-sided endpoint
        // derivatives did exactly that (right-of-kink intervals saw the
        // left slope) and subdivided to max_depth for free.
        let mid = 0.5 * (a + b);
        let bend_turn = self
            .secant_tangent(a, mid)
            .angle_between(self.secant_tangent(mid, b));
        if bend_turn > self.options.max_bend_angle_rad {
            return true;
        }
        let sections = &self.surface.sections;
        if (sections.incidence(b) - sections.incidence(a)).abs()
            > self.options.max_incidence_change_rad
        {
            return true;
        }
        if (self.secant_sweep(a, mid) - self.secant_sweep(mid, b)).abs()
            > self.options.max_sweep_change_rad
            && (b - a) * self.surface.span_m > 1e-9
        {
            return true;
        }
        false
    }

    /// Unit span secant from `p` to `q` in mapped `(Y, Z)`, embedded in
    /// 3D. Spans a strictly interior segment: kink-safe by construction.
    fn secant_tangent(&self, p: f64, q: f64) -> DVec3 {
        let tangent = DVec3::new(
            0.0,
            self.span_y(q) - self.span_y(p),
            self.bend_z(q) - self.bend_z(p),
        );
        if tangent.length_squared() <= f64::EPSILON {
            DVec3::Y
        } else {
            tangent.normalize()
        }
    }

    /// Leading-edge sweep of the secant from `p` to `q`.
    fn secant_sweep(&self, p: f64, q: f64) -> f64 {
        (self.surface.planform.leading_edge(q) - self.surface.planform.leading_edge(p))
            .atan2(self.span_y(q) - self.span_y(p))
    }
    /// Unit span tangent at `s` from the exact segment derivatives.
    /// Finite differences would smear kink vertices into neighboring
    /// zones; the piecewise-linear spline differentiates exactly.
    fn span_tangent(&self, s: f64) -> DVec3 {
        // dY/ds carries bend_k, dZ/ds carries bend_k: the scale cancels in
        // the normalization, so the raw authored slopes suffice.
        let tangent = DVec3::new(
            0.0,
            self.surface.span_m,
            self.surface.bend.elevation_slope(s),
        );
        if tangent.length_squared() <= f64::EPSILON {
            DVec3::Y
        } else {
            tangent.normalize()
        }
    }

    /// Chordwise zones for one span leaf: split at covering control-region
    /// chord bounds so no panel straddles independently moving regions.
    /// Returns `(u0, u1, tag)` per zone.
    fn chord_zones(&self, a: f64, b: f64) -> Vec<(f64, f64, PanelTag)> {
        let mid_s = 0.5 * (a + b);
        let mut cuts = vec![0.0, 1.0];
        for region in &self.surface.controls {
            if region.span.0 < mid_s && mid_s < region.span.1 {
                cuts.push(region.chord.0);
                cuts.push(region.chord.1);
            }
        }
        cuts.sort_by(|x, y| x.partial_cmp(y).expect("validated finite"));
        let mut deduped: Vec<f64> = Vec::new();
        for cut in cuts {
            if deduped
                .last()
                .is_none_or(|last: &f64| (cut - *last).abs() > 1e-9)
            {
                deduped.push(cut);
            }
        }
        // fold_order sorts joints by station descending, so the first
        // match is the outboard-most joint regardless of authoring order
        // (rposition over the raw array would answer array order).
        let fold = self
            .fold_order
            .iter()
            .copied()
            .find(|&order| self.surface.folds[order].station_s <= a + 1e-9);
        deduped
            .windows(2)
            .map(|pair| {
                let (u0, u1) = (pair[0], pair[1]);
                let mid_u = 0.5 * (u0 + u1);
                let owner = self
                    .surface
                    .controls
                    .iter()
                    .enumerate()
                    .filter(|(_, region)| {
                        region.span.0 < mid_s
                            && mid_s < region.span.1
                            && region.chord.0 <= mid_u
                            && mid_u <= region.chord.1
                    })
                    .max_by_key(|(_, region)| region.depth())
                    .map(|(index, region)| (index, region.parent));
                let (control, control_parent) = owner
                    .map(|(index, parent)| (Some(index), parent))
                    .unwrap_or((None, None));
                (
                    u0,
                    u1,
                    PanelTag {
                        control,
                        control_parent,
                        fold,
                    },
                )
            })
            .collect()
    }

    /// Build one zone panel plus its four folded surface-local corners.
    fn zone_panel(
        &self,
        a: f64,
        b: f64,
        u0: f64,
        u1: f64,
    ) -> Result<(AeroPanel, [DVec3; 4]), SurfaceError> {
        let planform = &self.surface.planform;
        let mid = 0.5 * (a + b);
        let corners = [
            self.fold_point(a, u0),
            self.fold_point(a, u1),
            self.fold_point(b, u0),
            self.fold_point(b, u1),
        ];
        // Area centroid via two triangles. A plain corner average is
        // off-center on tapered zones (bias ~ taper/8 of chord), which
        // both misstates moment arms and makes whole-vs-halves estimates
        // inconsistent; the triangle centroid is exact on planar quads
        // and triangulation-dependence is higher-order.
        let centroid = quad_centroid(corners);
        let chord_a = planform.chord(a) * (u1 - u0);
        let chord_b = planform.chord(b) * (u1 - u0);
        // Spanwise material extent: the mapped (Y, Z) distance. The
        // chordwise edge slant (sweep) belongs to the planform shape, not
        // to the strip width; using the slanted corner distance here
        // would overstate swept-wing area by ~1/cos(sweep).
        let span_3d = (self.span_y(b) - self.span_y(a)).hypot(self.bend_z(b) - self.bend_z(a));
        let area = 0.5 * (chord_a + chord_b) * span_3d;
        let chord_m = 0.5 * (chord_a + chord_b);
        let incidence = self.surface.sections.incidence(mid);
        let tangent = self.span_tangent(mid);
        let chord_dir = (DQuat::from_axis_angle(tangent, incidence) * DVec3::X).normalize();
        let lift_dir = chord_dir.cross(tangent).normalize();
        let sweep = (planform.leading_edge(b) - planform.leading_edge(a))
            .atan2(self.span_y(b) - self.span_y(a));
        if sweep.abs() >= 89.0_f64.to_radians() {
            return Err(SurfaceError::PanelRejected(format!(
                "zone {a}..{b} leading-edge sweep {sweep} rad is too steep; add planform stations"
            )));
        }
        let aspect = self.surface_aspect_ratio;
        let thickness = self.surface.sections.thickness(mid);
        let panel = AeroPanel::new(centroid, chord_dir, lift_dir, area, chord_m)
            .and_then(|panel| panel.with_planform(span_3d, aspect, sweep, 1.0))
            .and_then(|panel| panel.with_center_of_pressure(centroid))
            .and_then(|panel| panel.with_thickness_ratio(thickness))
            .map_err(|error| SurfaceError::PanelRejected(error.to_string()))?;
        Ok((panel, corners))
    }

    /// Surface-local point at `(s, u)`, folded by every outboard joint.
    /// Joints apply outboard-first about their as-drawn hinges so an
    /// inboard fold rigidly carries already-folded outboard geometry.
    fn fold_point(&self, s: f64, u: f64) -> DVec3 {
        folded_point_raw(
            &self.surface.planform,
            &self.surface.bend,
            self.bend_k,
            self.surface.span_m,
            &self.surface.folds,
            &self.fold_order,
            &self.fold_angles,
            s,
            u,
        )
    }
    fn compiled_folds(&self) -> Vec<CompiledFold> {
        let planform = &self.surface.planform;
        let as_drawn_hinge = |station_s: f64| {
            DVec3::new(
                planform.leading_edge(station_s) + 0.5 * planform.chord(station_s),
                self.span_y(station_s),
                self.bend_z(station_s),
            )
        };
        self.surface
            .folds
            .iter()
            .enumerate()
            .map(|(index, joint)| {
                // Carry the joint through every inboard fold, outboard
                // first about as-drawn hinges: the runtime record must sit
                // where the folded geometry actually is, not at the
                // as-drawn station.
                let mut hinge = as_drawn_hinge(joint.station_s);
                let mut axis = joint.axis.normalize();
                for &order in &self.fold_order {
                    let carrier = &self.surface.folds[order];
                    if carrier.station_s < joint.station_s {
                        let carrier_hinge = as_drawn_hinge(carrier.station_s);
                        let carrier_axis = carrier.axis.normalize();
                        let angle = self.fold_angles[order] - carrier.deployed_angle_rad;
                        let rotation = DQuat::from_axis_angle(carrier_axis, angle);
                        hinge = carrier_hinge + rotation * (hinge - carrier_hinge);
                        axis = rotation * axis;
                    }
                }
                CompiledFold {
                    name: joint.name.clone(),
                    hinge_body_m: hinge,
                    axis_body: axis,
                    angle_rad: self.fold_angles[index],
                    // As-drawn here; mount conjugates the sign with the
                    // angle so the runtime delta stays identity.
                    deployed_angle_rad: joint.deployed_angle_rad,
                    // Nearest inboard joint by station (fold_order runs
                    // outboard-first, so the last match below wins).
                    parent_joint: self
                        .surface
                        .folds
                        .iter()
                        .enumerate()
                        .filter(|(other, other_joint)| {
                            *other != index && other_joint.station_s < joint.station_s
                        })
                        .max_by(|(_, a), (_, b)| {
                            a.station_s
                                .partial_cmp(&b.station_s)
                                .expect("validated finite")
                        })
                        .map(|(other, _)| other),
                }
            })
            .collect()
    }

    fn control_definitions(
        &self,
        tags: &[PanelTag],
    ) -> Result<Vec<ControlSurfaceDefinition>, SurfaceError> {
        let mut by_region: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (panel_index, tag) in tags.iter().enumerate() {
            if let Some(region) = tag.control {
                by_region.entry(region).or_default().push(panel_index);
            }
        }
        self.surface
            .controls
            .iter()
            .enumerate()
            .map(|(index, region)| {
                let panels = by_region.remove(&index).unwrap_or_default();
                // Definitions emit in region order, so the region index
                // doubles as the definition index for parent links; the
                // kind marker rides along for the runtime mixer.
                let kind = match region.kind {
                    crate::ControlRegionKind::TrailingEdgeDevice => {
                        thessa_sim_core::ControlKind::Hinge
                    }
                    crate::ControlRegionKind::AllMovingSurface => {
                        thessa_sim_core::ControlKind::AllMoving
                    }
                };
                let mut definition = ControlSurfaceDefinition::new(
                    region.name.clone(),
                    panels,
                    region.min_deflection_rad,
                    region.max_deflection_rad,
                )
                .map_err(|error| SurfaceError::PanelRejected(error.to_string()))?;
                definition = definition.with_kind(kind);
                if let Some(parent) = region.parent {
                    definition = definition.with_parent(parent);
                }
                Ok(definition)
            })
            .collect()
    }

    /// Mount surface-local output into the body frame: mount roll, then
    /// the chordwise reflection, then the optional mirror, then the
    /// origin offset. A zero roll skips the rotation exactly so unrolled
    /// surfaces keep bit-identical golden geometry.
    ///
    /// Frame contract: local +X runs aft (leading edge to trailing
    /// edge) while body +X runs forward, so mounted POINTS map x to -x
    /// and the trailing edge lands aft. Solver axes (chord/lift forward
    /// and lift references) keep their body-frame meaning and skip the
    /// reflection; fold records conjugate (axis and angle) so the
    /// runtime reproduces compiled positions exactly.
    fn mount(&self, mut compiled: CompiledSurface) -> CompiledSurface {
        if self.surface.mount_roll_rad != 0.0 {
            let roll = DQuat::from_axis_angle(DVec3::X, self.surface.mount_roll_rad);
            for panel in &mut compiled.panels {
                panel.position_body_m = roll * panel.position_body_m;
                panel.center_of_pressure_body_m = roll * panel.center_of_pressure_body_m;
                panel.chord_axis_body = (roll * panel.chord_axis_body).normalize();
                panel.lift_axis_body = (roll * panel.lift_axis_body).normalize();
            }
            for fold in &mut compiled.folds {
                fold.hinge_body_m = roll * fold.hinge_body_m;
                fold.axis_body = (roll * fold.axis_body).normalize();
            }
            compiled.summary.rotate(roll);
        }
        for panel in &mut compiled.panels {
            panel.position_body_m.x = -panel.position_body_m.x;
            panel.center_of_pressure_body_m.x = -panel.center_of_pressure_body_m.x;
        }
        for fold in &mut compiled.folds {
            fold.hinge_body_m.x = -fold.hinge_body_m.x;
            fold.axis_body.x = -fold.axis_body.x;
            fold.angle_rad = -fold.angle_rad;
            fold.deployed_angle_rad = -fold.deployed_angle_rad;
        }
        compiled.summary.reflect_x();
        if self.surface.mirror_y {
            compiled = compiled.mirrored();
        }
        let origin = self.surface.origin_body_m;
        for panel in &mut compiled.panels {
            panel.position_body_m += origin;
            panel.center_of_pressure_body_m += origin;
        }
        for fold in &mut compiled.folds {
            fold.hinge_body_m += origin;
        }
        compiled.summary.translate(origin);
        compiled
    }

    /// Mount rotation as a matrix (roll about body x, identity at rest).
    fn mount_matrix(&self) -> glam::DMat3 {
        if self.surface.mount_roll_rad == 0.0 {
            glam::DMat3::IDENTITY
        } else {
            glam::DMat3::from_quat(DQuat::from_axis_angle(
                DVec3::X,
                self.surface.mount_roll_rad,
            ))
        }
    }

    /// Map a surface-local point into the body frame through the exact
    /// mount order: roll, chordwise reflection, mirror, origin offset.
    fn mount_point(&self, point: DVec3) -> DVec3 {
        let mut mapped = self.mount_matrix() * point;
        mapped.x = -mapped.x;
        if self.surface.mirror_y {
            mapped.y = -mapped.y;
        }
        mapped + self.surface.origin_body_m
    }

    /// Carry the local structural aggregates into the body frame: mass
    /// and volumes are invariant, centers map as points. Inertia goes
    /// through the local center of mass first (reverse parallel axis),
    /// then rotates (plus mirror conjugation), then shifts to the body
    /// origin from the mounted center: shifting by the bare origin drops
    /// the origin-COM cross terms whenever they differ. Fuel fill
    /// derives here: one minus sump minus rib displacement, floored.
    fn mount_structure(&self, acc: StructAcc, layout: &StructuralLayout) -> CompiledStructure {
        let center_local = if acc.mass_kg > 0.0 {
            acc.com_numerator / acc.mass_kg
        } else {
            DVec3::ZERO
        };
        let rotation = self.mount_matrix();
        let mut inertia = rotation
            * (acc.inertia_local - point_inertia(acc.mass_kg, center_local))
            * rotation.transpose();
        // Conjugation by diag(-1,1,1) (chordwise reflection) flips the
        // xy- and xz-product signs; the mirror step below conjugates by
        // diag(1,-1,1) the same way. Together they are the full mount map.
        inertia.y_axis.x = -inertia.y_axis.x;
        inertia.x_axis.y = -inertia.x_axis.y;
        inertia.z_axis.x = -inertia.z_axis.x;
        inertia.x_axis.z = -inertia.x_axis.z;
        if self.surface.mirror_y {
            // Conjugation by diag(1,-1,1) flips the xy- and yz-product signs.
            inertia.y_axis.x = -inertia.y_axis.x;
            inertia.x_axis.y = -inertia.x_axis.y;
            inertia.y_axis.z = -inertia.y_axis.z;
            inertia.z_axis.y = -inertia.z_axis.y;
        }
        let center_of_mass_body_m = self.mount_point(center_local);
        inertia += point_inertia(acc.mass_kg, center_of_mass_body_m);
        // Derived fill: box minus in-box rib displacement minus sump,
        // floored. Only rib plate inside the fuel-box chord interval
        // displaces fuel.
        let displacement = if acc.box_m3 > 0.0 {
            acc.rib_inbox_m3 / acc.box_m3
        } else {
            0.0
        };
        let fill = (1.0 - layout.fuel_sump_fraction - displacement).max(0.0);
        let fuel_volume_m3 = acc.box_m3 * fill;
        let fuel_centroid_body_m = if acc.box_m3 > 0.0 {
            self.mount_point(acc.box_numerator / acc.box_m3)
        } else {
            DVec3::ZERO
        };
        CompiledStructure {
            mass_kg: acc.mass_kg,
            skin_mass_kg: acc.skin_kg,
            spar_web_mass_kg: acc.web_kg,
            spar_cap_mass_kg: acc.cap_kg,
            rib_mass_kg: acc.rib_kg,
            center_of_mass_body_m,
            inertia_body_kg_m2: inertia,
            fuel_volume_m3,
            fuel_centroid_body_m,
        }
    }
}
