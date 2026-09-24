//! Golden reconstruction fixtures from public real-vehicle geometry.
//!
//! Purpose (design doc section 12.2): verify that the authoring model can
//! represent real planforms/mechanisms and that compilation preserves their
//! known geometry and kinematics. These fixtures do NOT claim the aero
//! solver reproduces measured flight performance.
//!
//! Each fixture records its public sources alongside every reconstruction
//! assumption, with per-value tolerances reflecting source quality:
//! manufacturer dimensions are tight, drawing-reconstructed values are
//! wide. A failure must read as either a compiler regression or a changed
//! documented assumption, never a silently renormalized fit.

use crate::{
    BendCurve, FoldJoint, Planform, ProceduralSurface, SectionData, SurfaceError, SurfaceTopology,
};

/// Public Boeing 777-9 reference values (manufacturer dimensions).
pub mod boeing_777x {
    /// Extended (flight) wingspan, Boeing ACAP Rev G and boeing.com specs.
    pub const EXTENDED_SPAN_M: f64 = 71.76;
    /// Ground (folded) wingspan, same sources.
    pub const FOLDED_SPAN_M: f64 = 64.84;
    /// Reference wing area, 5,562 sq ft (Boeing specs via Wikipedia).
    pub const WING_AREA_M2: f64 = 516.7;
    /// Folding wingtip length per side, 11 ft (Aviation Week via Wikipedia;
    /// secondary press reports 11.5 ft / 3.5 m for the outer section).
    pub const FOLD_TIP_LENGTH_M: f64 = 3.5;
}

/// Public Dream Chaser reference values (manufacturer/agency statements).
pub mod dream_chaser {
    /// Deployed wingspan, roughly 7 m (Sierra Space vehicle pages and
    /// public NASA material; secondary-source grade, not a tight drawing
    /// dimension).
    pub const DEPLOYED_SPAN_M: f64 = 7.0;
    /// Launch fairing diameter the cargo vehicle folds into (Sierra Space:
    /// stowed inside a 5 m fairing; NASA: Vulcan Centaur five-meter
    /// fairing).
    pub const FAIRING_DIAMETER_M: f64 = 5.0;
}

/// Public Space Shuttle Orbiter reference values (NASA dimensions).
pub mod shuttle_orbiter {
    /// Wingspan, 78 ft (NASA Orbiter fact sheets).
    pub const SPAN_M: f64 = 23.77;
    /// Reference wing area, 2,906 sq ft (NASA).
    pub const WING_AREA_M2: f64 = 269.9;
}

/// Public Concorde reference values (manufacturer dimensions).
pub mod concorde {
    /// Wingspan, 83 ft 8 in (Aerospatiale/BAC specs).
    pub const SPAN_M: f64 = 25.5;
    /// Reference wing area, 3,856 sq ft (same specs).
    pub const WING_AREA_M2: f64 = 358.2;
    /// Calibrated Polhamus vortex-lift factor for the ogival delta.
    ///
    /// The compiled Concorde wing holds aspect ratio 1.813 with effective
    /// leading-edge sweep near 60 deg. AVL (VLM, attached flow) gives a
    /// lift slope of 1.98/rad against the solver's 2.04/rad (+3.0%); the
    /// vortex term `V * |sin a| * sin a * cos a` is second-order at small
    /// angles, so it leaves that slope untouched. `V = 3.0` follows the
    /// Polhamus suction analogy (NASA TN D-3767) vortex constant `Kv` for
    /// ~60 deg sharp-edge sweep and reproduces the `Kp = 1.98` polar
    /// within ~2.5% at 5–15 deg AoA. Pinned by
    /// `concorde_delta_vortex_lift_matches_polhamus_band`.
    pub const VORTEX_LIFT_FACTOR: f64 = 3.0;
}

/// Boeing 777X right half-wing reconstruction.
///
/// Sources: Boeing 777X ACAP Rev G (spans 71.76 / 64.84 m, folding-tip
/// CONOPS appendix), boeing.com 777x specs (71.8 / 64.8 m), Wikipedia
/// Boeing 777X citing Boeing and Aviation Week (11 ft Liebherr folding
/// tips, ~20 s actuation, 5,562 sq ft / 516.7 m^2 area, aspect ratio 10:1).
///
/// Reconstruction assumptions (deliberate simplifications, each with a
/// declared tolerance in the golden test):
///
/// * single trapezoid with taper ratio 0.20 (root 12.0 m, tip 2.4 m at the
///   centerline; no yehudi kink, no engine-mount glove). The centerline
///   root matches Boeing's reference-area convention, which carries the
///   trapezoid through the fuselage.
/// * quarter-chord sweep 31 degrees (787-derived wing with less sweep than
///   the 787; no public 777X sweep value found, so derived LE offset is
///   assumption-grade, not manufacturer-grade).
/// * 5 degrees dihedral (family-typical; envelope-exact by construction
///   either way since the span parameter is material length).
/// * fold station at exactly one tip-length inboard of the tip, hinge
///   about the unswept chordwise axis, stowed straight up (90 degrees).
///   The real hinge line is swept and the stowed angle is not published;
///   the vertical fold reproduces the public folded span within 8 cm.
/// * zero twist, uniform 10 percent thickness (envelope-neutral).
pub fn boeing_777x_half_wing() -> Result<ProceduralSurface, SurfaceError> {
    let semi_span_m = boeing_777x::EXTENDED_SPAN_M / 2.0;
    let dihedral_rad = 5.0_f64.to_radians();
    // Material root-to-tip length whose projection is the semi-span.
    let span_m = semi_span_m / dihedral_rad.cos();
    // Taper 0.20 sized so the centerline trapezoid hits the public area.
    let (root_chord_m, tip_chord_m) = (12.0, 2.4);
    // Quarter-chord sweep 31 deg converted to a tip leading-edge offset:
    // dx_qc = tan(31) * semi, minus the 1/4-chord taper shift.
    let quarter_sweep_rad = 31.0_f64.to_radians();
    let tip_le_offset_m =
        quarter_sweep_rad.tan() * semi_span_m - 0.25 * (tip_chord_m - root_chord_m);
    // One tip-length inboard in material coordinates.
    let fold_station_s = 1.0 - boeing_777x::FOLD_TIP_LENGTH_M / span_m;
    let surface = ProceduralSurface {
        name: "b777x-half-wing-right".into(),
        span_m,
        origin_body_m: glam::DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::SymmetricHalf,
        planform: Planform::tapered(root_chord_m, tip_chord_m, tip_le_offset_m)?,
        bend: BendCurve::dihedral(span_m, dihedral_rad)?,
        sections: SectionData::uniform(0.0, 0.10)?,
        controls: Vec::new(),
        folds: vec![FoldJoint {
            name: "wingtip-fold".into(),
            station_s: fold_station_s,
            axis: glam::DVec3::X,
            deployed_angle_rad: 0.0,
            stowed_angle_rad: 90.0_f64.to_radians(),
            travel_limit_rad: 95.0_f64.to_radians(),
            deployment_rate_rad_s: 0.05,
            lock_window_rad: (-0.03, 0.03),
            max_dynamic_pressure_pa: Some(2000.0),
        }],
        structure: None,
    };
    surface.validate()?;
    Ok(surface)
}

/// Space Shuttle Orbiter right wing reconstruction.
///
/// Sources: NASA Orbiter fact sheets (78 ft / 23.77 m span; 2,906 sq ft /
/// 269.9 m^2 reference area carried to the centerline).
///
/// Reconstruction assumptions: three-station double delta (centerline
/// root chord 18.0 m, kink at 45 percent semi-span with 12.96 m chord,
/// 3.0 m tip) sized so the centerline trapezoid hits the public area;
/// inner leading-edge sweep ~78 deg, outer ~45 deg (family-typical
/// double-delta split, no public kink drawing tied to this fixture, so
/// sweep bands are assumption-grade); flat (dihedral omitted); two
/// elevon preset regions per wing (inboard/outboard trailing-edge
/// devices with pitch-plus-roll mixing data); zero twist, uniform 8
/// percent thickness.
///
/// What this fixture proves: the authoring model spans a kinked delta
/// planform, compilation preserves span/area, elevon regions own exact
/// panel groups with recorded hinges, and inner/outer sweep regimes
/// survive compilation as distinct panel populations.
pub fn shuttle_orbiter_wing() -> Result<ProceduralSurface, SurfaceError> {
    use crate::{SpanStation, preset};

    let semi_span_m = shuttle_orbiter::SPAN_M / 2.0;
    let kink_s = 0.45;
    let kink_y = kink_s * semi_span_m;
    let inner_le_sweep = 78.0_f64.to_radians();
    let outer_le_sweep = 45.0_f64.to_radians();
    let kink_le = inner_le_sweep.tan() * kink_y;
    let tip_le = kink_le + outer_le_sweep.tan() * (semi_span_m - kink_y);
    let (elevon_inboard, _) = preset::elevon("elevon-inboard", (0.35, 0.62), (0.55, 1.0))?;
    let (elevon_outboard, _) = preset::elevon("elevon-outboard", (0.62, 0.95), (0.6, 1.0))?;
    let surface = ProceduralSurface {
        name: "shuttle-orbiter-wing-right".into(),
        span_m: semi_span_m,
        origin_body_m: glam::DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::SymmetricHalf,
        planform: Planform::from_stations(vec![
            SpanStation {
                s: 0.0,
                x_le: 0.0,
                x_te: 18.0,
            },
            SpanStation {
                s: kink_s,
                x_le: kink_le,
                x_te: kink_le + 12.96,
            },
            SpanStation {
                s: 1.0,
                x_le: tip_le,
                x_te: tip_le + 3.0,
            },
        ])?,
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.08)?,
        controls: vec![elevon_inboard, elevon_outboard],
        folds: Vec::new(),
        structure: None,
    };
    surface.validate()?;
    Ok(surface)
}

/// Concorde right wing reconstruction.
///
/// Sources: Aerospatiale/BAC specs (83 ft 8 in / 25.5 m span; 3,856 sq ft
/// / 358.2 m^2 reference area).
///
/// Reconstruction assumptions: seven-station ogival leading edge (gentle
/// root curvature running to a sharp tip, trailing edge nearly straight
/// with slight forward sweep outboard), flat, no controls in this
/// fixture (elevons are covered by the Shuttle fixture; the Concorde
/// point is curved-edge subdivision), zero twist, uniform 6 percent
/// thickness. Station placement is drawing-grade: no public ordinate
/// table is tied to this fixture, so the area band is wide while the
/// span stays manufacturer-tight.
///
/// What this fixture proves: the authoring model spans a strongly curved
/// planform no trapezoid can represent, curved edges drive adaptive
/// subdivision, sweep grows outboard (the ogival signature, opposite to
/// a tapered wing), and span/area survive compilation.
pub fn concorde_wing() -> Result<ProceduralSurface, SurfaceError> {
    use crate::SpanStation;

    let semi_span_m = concorde::SPAN_M / 2.0;
    let surface = ProceduralSurface {
        name: "concorde-wing-right".into(),
        span_m: semi_span_m,
        origin_body_m: glam::DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::SymmetricHalf,
        planform: Planform::from_stations(vec![
            SpanStation {
                s: 0.00,
                x_le: 0.0,
                x_te: 27.0,
            },
            SpanStation {
                s: 0.20,
                x_le: 1.2,
                x_te: 24.2,
            },
            SpanStation {
                s: 0.40,
                x_le: 4.5,
                x_te: 21.0,
            },
            SpanStation {
                s: 0.60,
                x_le: 9.5,
                x_te: 20.5,
            },
            SpanStation {
                s: 0.78,
                x_le: 15.5,
                x_te: 22.0,
            },
            SpanStation {
                s: 0.90,
                x_le: 20.5,
                x_te: 23.7,
            },
            SpanStation {
                s: 1.00,
                x_le: 25.3,
                x_te: 26.2,
            },
        ])?,
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.06)?,
        controls: Vec::new(),
        folds: Vec::new(),
        structure: None,
    };
    surface.validate()?;
    Ok(surface)
}

/// Dream Chaser cargo right wing reconstruction.
///
/// Sources: Sierra Space Dream Chaser pages (9 m long, wings fold for
/// launch inside a 5 m fairing, Wing Deployment System locks after
/// separation), NASA CRS-2 material (wings folded inside the Vulcan
/// Centaur five-meter fairing), public vehicle summaries (roughly 7 m
/// deployed wingspan).
///
/// Reconstruction assumptions: flat tapered half-wing from the
/// centerline (root 2.4 m, tip 1.0 m, ~19 deg leading-edge sweep),
/// whole-wing root fold at 10 percent semi-span rotating 65 degrees up,
/// zero twist, uniform 8 percent thickness. No stabilizer cant is
/// authored yet (the real wings cant steeply as body stabilizers; a
/// follow-up refinement), and no public wing area exists to compare
/// against since the lifting body carries most of the lift.
///
/// What this fixture proves: deployed geometry hits the 7 m span, the
/// root-fold topology stows the wing inside a wing-only 4.5 m envelope
/// (the 5 m fairing circle is shared with the body in the real stack),
/// and material area is invariant through the fold.
pub fn dream_chaser_wing() -> Result<ProceduralSurface, SurfaceError> {
    let semi_span_m = dream_chaser::DEPLOYED_SPAN_M / 2.0;
    let surface = ProceduralSurface {
        name: "dream-chaser-wing-right".into(),
        span_m: semi_span_m,
        origin_body_m: glam::DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::SymmetricHalf,
        planform: Planform::tapered(2.4, 1.0, 1.2)?,
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.08)?,
        controls: Vec::new(),
        folds: vec![FoldJoint {
            name: "wing-fold".into(),
            station_s: 0.1,
            axis: glam::DVec3::X,
            deployed_angle_rad: 0.0,
            stowed_angle_rad: 65.0_f64.to_radians(),
            travel_limit_rad: 70.0_f64.to_radians(),
            deployment_rate_rad_s: 0.1,
            lock_window_rad: (-0.03, 0.03),
            max_dynamic_pressure_pa: None,
        }],
        structure: None,
    };
    surface.validate()?;
    Ok(surface)
}

/// Pathfinder fictional wing reconstruction.
///
/// Deliberately NOT real-world validation ground truth (design doc
/// section 12.2): a single surface with smoothly rising canted tips and
/// embedded controls, useful for feature coverage (bend-plus-controls
/// interaction) without pretending to be a measured aircraft. Numbers
/// below are authoring choices, not manufacturer dimensions.
pub fn pathfinder_wing() -> Result<ProceduralSurface, SurfaceError> {
    use crate::{BendStation, preset};

    let (aileron, _) = preset::aileron("aileron", (0.55, 0.95))?;
    let (flap, _) = preset::flap("flap", (0.15, 0.5))?;
    let surface = ProceduralSurface {
        name: "pathfinder-wing-right".into(),
        span_m: 6.0,
        origin_body_m: glam::DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        topology: SurfaceTopology::SymmetricHalf,
        planform: Planform::tapered(2.0, 0.9, 0.8)?,
        bend: BendCurve::polyline(vec![
            BendStation { s: 0.0, z_m: 0.0 },
            BendStation { s: 0.5, z_m: 0.1 },
            BendStation { s: 0.7, z_m: 0.35 },
            BendStation { s: 1.0, z_m: 1.2 },
        ])?,
        sections: SectionData::uniform(1.0_f64.to_radians(), 0.09)?,
        controls: vec![aileron, flap],
        folds: Vec::new(),
        structure: None,
    };
    surface.validate()?;
    Ok(surface)
}
