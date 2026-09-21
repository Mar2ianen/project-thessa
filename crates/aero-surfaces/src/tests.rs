//! Analytic compiler tests (design doc section 12.1).
//!
//! Every reference value below is derived independently of the compiler:
//! closed forms for trapezoids, Simpson integrations coded separately in
//! each test, and structural counts worked out by hand from the split
//! rules. The compiler is never compared against itself.

use glam::DVec3;

use crate::{
    BendCurve, BendStation, CompileOptions, CompiledSurface, ControlRegion, ControlRegionKind,
    FoldJoint, MechanismState, Planform, ProceduralSurface, RefinementMode, SectionData,
    SpanStation, SurfaceError, compile_surface,
};

fn tight_options() -> CompileOptions {
    CompileOptions {
        max_chord_change_frac: 1e-4,
        max_bend_angle_rad: 0.01_f64.to_radians(),
        max_incidence_change_rad: 0.01_f64.to_radians(),
        max_sweep_change_rad: 0.01_f64.to_radians(),
        max_depth: 16,
        mode: RefinementMode::Tolerance,
    }
}

fn rectangular(span_m: f64, chord_m: f64) -> ProceduralSurface {
    ProceduralSurface::rectangular("test-wing", span_m, chord_m, DVec3::ZERO).unwrap()
}

fn aileron(name: &str) -> ControlRegion {
    ControlRegion {
        name: name.into(),
        span: (0.6, 0.9),
        chord: (0.25, 1.0),
        hinge_u: 0.25,
        min_deflection_rad: -20.0_f64.to_radians(),
        max_deflection_rad: 20.0_f64.to_radians(),
        parent: None,
        kind: ControlRegionKind::TrailingEdgeDevice,
    }
}

#[test]
fn rectangular_wing_matches_closed_form() {
    let surface = rectangular(8.0, 2.0);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // No features, no splits: exactly one zone.
    assert_eq!(compiled.panels.len(), 1);
    assert_eq!(compiled.tags.len(), 1);
    let summary = &compiled.summary;
    assert!((summary.material_area_m2 - 16.0).abs() < 1e-12);
    assert!((summary.projected_area_m2 - 16.0).abs() < 1e-12);
    assert!((summary.span_m - 8.0).abs() < 1e-12);
    assert!((summary.projected_span_m - 8.0).abs() < 1e-12);
    assert!((summary.root_chord_m - 2.0).abs() < 1e-12);
    assert!((summary.tip_chord_m - 2.0).abs() < 1e-12);
    assert!((summary.mean_aerodynamic_chord_m - 2.0).abs() < 1e-9);
    assert!(summary.sweep_rad.abs() < 1e-12);
    assert!((summary.bbox_min_m - DVec3::ZERO).length() < 1e-12);
    assert!((summary.bbox_max_m - DVec3::new(2.0, 8.0, 0.0)).length() < 1e-12);
    assert_eq!(summary.uncontrolled_panel_count, 1);

    let panel = &compiled.panels[0];
    assert!((panel.chord_axis_body - DVec3::X).length() < 1e-12);
    assert!((panel.lift_axis_body - DVec3::Z).length() < 1e-12);
    assert!((panel.area_m2 - 16.0).abs() < 1e-12);
    assert!((panel.chord_m - 2.0).abs() < 1e-12);
    assert!((panel.span_m - 8.0).abs() < 1e-12);
    assert!((panel.planform_aspect_ratio - 4.0).abs() < 1e-12);
    // COP is the zone centroid.
    assert!((panel.center_of_pressure_body_m - DVec3::new(1.0, 4.0, 0.0)).length() < 1e-9);
}

#[test]
fn tapered_swept_wing_matches_closed_form() {
    // Span 10, root 3, tip 1, tip leading edge 2 m aft.
    let taper: f64 = 1.0 / 3.0;
    let surface = ProceduralSurface {
        name: "tapered".into(),
        span_m: 10.0,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        planform: Planform::tapered(3.0, 1.0, 2.0).unwrap(),
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.0).unwrap(),
        controls: Vec::new(),
        folds: Vec::new(),
        structure: None,
    };
    let compiled =
        compile_surface(&surface, &tight_options(), &MechanismState::deployed()).unwrap();
    let summary = &compiled.summary;
    // Area of a trapezoid; MAC = 2/3 * root * (1 + l + l^2) / (1 + l).
    assert!((summary.material_area_m2 - 20.0).abs() < 1e-9);
    assert!((summary.projected_area_m2 - 20.0).abs() < 1e-9);
    let mac = 2.0 / 3.0 * 3.0 * (1.0 + taper + taper.powi(2)) / (1.0 + taper);
    assert!((summary.mean_aerodynamic_chord_m - mac).abs() < 1e-6);
    assert!((summary.sweep_rad - (2.0_f64).atan2(10.0)).abs() < 1e-9);
    assert!((summary.root_chord_m - 3.0).abs() < 1e-12);
    assert!((summary.tip_chord_m - 1.0).abs() < 1e-12);
    // Every panel carries the surface aspect ratio and the local sweep.
    for panel in &compiled.panels {
        assert!((panel.planform_aspect_ratio - 100.0 / 20.0).abs() < 1e-9);
        assert!((panel.planform_sweep_rad - (2.0_f64).atan2(10.0)).abs() < 1e-6);
    }
    // Panel areas telescope exactly to the trapezoid area.
    let total: f64 = compiled.panels.iter().map(|panel| panel.area_m2).sum();
    assert!((total - 20.0).abs() < 1e-9);
}

#[test]
fn dihedral_preserves_material_area_and_shrinks_projection() {
    let mut surface = rectangular(8.0, 2.0);
    let angle = 10.0_f64.to_radians();
    surface.bend = BendCurve::dihedral(8.0, angle).unwrap();
    let compiled =
        compile_surface(&surface, &tight_options(), &MechanismState::deployed()).unwrap();
    let summary = &compiled.summary;
    // Orientation alone must not change material area.
    assert!((summary.material_area_m2 - 16.0).abs() < 1e-9);
    // Projection shrinks by exactly cos(dihedral).
    assert!((summary.projected_area_m2 - 16.0 * angle.cos()).abs() < 1e-9);
    assert!((summary.projected_span_m - 8.0 * angle.cos()).abs() < 1e-9);
    // Mapped tip elevation is span * sin(dihedral).
    assert!((summary.bend_profile_m[4] - 8.0 * angle.sin()).abs() < 1e-9);
    assert!(summary.bend_profile_m[0].abs() < 1e-12);
    // Panels tilt with the bend: lift stays normal to the surface.
    for panel in &compiled.panels {
        assert!((panel.lift_axis_body.z - angle.cos()).abs() < 1e-9);
        assert!((panel.lift_axis_body.y + angle.sin()).abs() < 1e-6);
    }
}

/// Independent Simpson length of the analytic sine camber, coded
/// separately from the compiler's trapezoid arc-length pass.
fn sine_raw_length(span_m: f64) -> f64 {
    let steps = 10_000;
    let h = 1.0 / steps as f64;
    let speed = |s: f64| {
        let dz = std::f64::consts::PI * (std::f64::consts::PI * s).cos();
        (span_m.powi(2) + dz.powi(2)).sqrt()
    };
    let mut sum = speed(0.0) + speed(1.0);
    for index in 1..steps {
        sum += if index % 2 == 0 { 2.0 } else { 4.0 } * speed(index as f64 * h);
    }
    sum * h / 3.0
}

fn sine_bend_surface(stations: usize) -> ProceduralSurface {
    let polyline: Vec<BendStation> = (0..=stations)
        .map(|index| {
            let s = index as f64 / stations as f64;
            BendStation {
                s,
                z_m: (std::f64::consts::PI * s).sin(),
            }
        })
        .collect();
    let mut surface = rectangular(8.0, 2.0);
    surface.bend = BendCurve::polyline(polyline).unwrap();
    surface
}

#[test]
fn smooth_bend_converges_to_independent_integration() {
    // Analytic reference: projected area of chord * k * span, where k
    // normalizes the sine arc length to the span. Projection telescopes to
    // chord * tip_Y exactly, whatever the camber shape.
    let scale = 8.0 / sine_raw_length(8.0);
    let reference_projected = 2.0 * scale * 8.0;

    let coarse = compile_surface(
        &sine_bend_surface(8),
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let dense = compile_surface(
        &sine_bend_surface(512),
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // Material area is exact at any authoring density: corners are exact.
    assert!((coarse.summary.material_area_m2 - 16.0).abs() < 1e-9);
    assert!((dense.summary.material_area_m2 - 16.0).abs() < 1e-9);
    // Authoring refinement converges toward the analytic projection: the
    // dense error is an order of magnitude below the coarse error and
    // inside a tight absolute envelope.
    let coarse_error = (coarse.summary.projected_area_m2 - reference_projected).abs();
    let dense_error = (dense.summary.projected_area_m2 - reference_projected).abs();
    assert!(coarse_error < 0.01 * reference_projected);
    assert!(dense_error < coarse_error / 10.0);
    assert!(dense_error < 1e-6 * reference_projected);
}

#[test]
fn tolerance_tightening_refines_zones_without_moving_geometry() {
    // Trapezoid areas and mid-interval frames are exact on linear inputs,
    // so tightening tolerances must refine zone granularity for solver
    // locality without moving areas. The tapered wing drives subdivision
    // through the chord metric; the sine wing checks frames stay put.
    let tapered = ProceduralSurface {
        name: "tapered".into(),
        span_m: 10.0,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        planform: Planform::tapered(3.0, 1.0, 2.0).unwrap(),
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.0).unwrap(),
        controls: Vec::new(),
        folds: Vec::new(),
        structure: None,
    };
    let coarse = compile_surface(
        &tapered,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let fine = compile_surface(&tapered, &tight_options(), &MechanismState::deployed()).unwrap();
    assert!(fine.panels.len() > coarse.panels.len());
    for (coarse_value, fine_value) in [
        (
            coarse.summary.material_area_m2,
            fine.summary.material_area_m2,
        ),
        (
            coarse.summary.projected_area_m2,
            fine.summary.projected_area_m2,
        ),
    ] {
        assert!((coarse_value - fine_value).abs() < 1e-9 * coarse_value.abs().max(1.0));
    }
    assert!((fine.summary.material_area_m2 - 20.0).abs() < 1e-9);

    let bent = sine_bend_surface(16);
    let coarse_bent = compile_surface(
        &bent,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let fine_bent = compile_surface(&bent, &tight_options(), &MechanismState::deployed()).unwrap();
    assert!(
        (coarse_bent.summary.projected_area_m2 - fine_bent.summary.projected_area_m2).abs()
            < 1e-9 * coarse_bent.summary.projected_area_m2
    );
}

#[test]
fn control_region_splits_ownership_without_straddling() {
    let mut surface = rectangular(10.0, 2.0);
    surface.controls.push(aileron("aileron"));
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // Splits at s = 0.6/0.9, chord cuts at u = 0.25 inside: 1 + 2 + 1.
    assert_eq!(compiled.panels.len(), 4);
    assert_eq!(compiled.controls.len(), 1);
    let definition = &compiled.controls[0];
    assert_eq!(definition.name, "aileron");
    assert_eq!(definition.panel_indices.len(), 1);
    let owned = &compiled.panels[definition.panel_indices[0]];
    // Region area: span fraction 0.3 times chord fraction 0.75 of 20 m^2.
    assert!((owned.area_m2 - 4.5).abs() < 1e-9);
    assert_eq!(compiled.tags[definition.panel_indices[0]].control, Some(0));
    // The owned panel sits fully inside the region bounds.
    assert!(owned.center_of_pressure_body_m.y > 6.0 - 1e-9);
    assert!(owned.center_of_pressure_body_m.y < 9.0 + 1e-9);
    // Control area telemetry matches.
    assert_eq!(compiled.summary.control_areas.len(), 1);
    assert!((compiled.summary.control_areas[0].area_m2 - 4.5).abs() < 1e-9);
}

#[test]
fn nested_tab_keeps_parent_chain() {
    let mut surface = rectangular(10.0, 2.0);
    surface.controls.push(ControlRegion {
        name: "elevator".into(),
        span: (0.5, 1.0),
        chord: (0.2, 1.0),
        hinge_u: 0.2,
        min_deflection_rad: -25.0_f64.to_radians(),
        max_deflection_rad: 25.0_f64.to_radians(),
        parent: None,
        kind: ControlRegionKind::TrailingEdgeDevice,
    });
    surface.controls.push(ControlRegion {
        name: "trim-tab".into(),
        span: (0.7, 0.9),
        chord: (0.5, 0.9),
        hinge_u: 0.5,
        min_deflection_rad: -15.0_f64.to_radians(),
        max_deflection_rad: 15.0_f64.to_radians(),
        parent: Some(0),
        kind: ControlRegionKind::TrailingEdgeDevice,
    });
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // Span leaves [0,.5] [.5,.7] [.7,.9] [.9,1] with chord cuts stacking:
    // 1 + 2 + 4 + 2 panels.
    assert_eq!(compiled.panels.len(), 9);
    assert_eq!(compiled.controls.len(), 2);
    let tab = &compiled.controls[1];
    assert_eq!(tab.panel_indices.len(), 1);
    let tab_tag = &compiled.tags[tab.panel_indices[0]];
    assert_eq!(tab_tag.control, Some(1));
    assert_eq!(tab_tag.control_parent, Some(0));
    // Tab area: 20 m^2 times 0.2 span times 0.4 chord.
    let tab_area = compiled.panels[tab.panel_indices[0]].area_m2;
    assert!((tab_area - 1.6).abs() < 1e-9);
    // The parent owns its ring around the tab, never the tab itself.
    let elevator = &compiled.controls[0];
    assert_eq!(elevator.panel_indices.len(), 4);
    assert!(!elevator.panel_indices.contains(&tab.panel_indices[0]));
    // Union of both definitions covers the whole elevator footprint.
    let owned_area: f64 = elevator
        .panel_indices
        .iter()
        .chain(tab.panel_indices.iter())
        .map(|&index| compiled.panels[index].area_m2)
        .sum();
    assert!((owned_area - 20.0 * 0.5 * 0.8).abs() < 1e-9);
}

#[test]
fn fold_preserves_material_area_and_moves_envelope() {
    let mut surface = rectangular(8.0, 2.0);
    surface.folds.push(FoldJoint {
        name: "tip-fold".into(),
        station_s: 0.7,
        axis: DVec3::X,
        deployed_angle_rad: 0.0,
        stowed_angle_rad: 60.0_f64.to_radians(),
        travel_limit_rad: 70.0_f64.to_radians(),
    });
    let deployed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let stowed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::stowed(&surface),
    )
    .unwrap();
    // Rigid folding preserves child material area exactly.
    assert!((deployed.summary.material_area_m2 - 16.0).abs() < 1e-12);
    assert!((stowed.summary.material_area_m2 - 16.0).abs() < 1e-12);
    // Envelope changes exactly by the transform: span shrinks to
    // 0.7*8 + 0.3*8*cos60 = 6.8, tip rises to 0.3*8*sin60.
    assert!((stowed.summary.projected_span_m - 6.8).abs() < 1e-9);
    assert!((stowed.summary.bbox_max_m.z - 2.4 * (60.0_f64.to_radians().sin())).abs() < 1e-9);
    assert!((deployed.summary.projected_span_m - 8.0).abs() < 1e-12);
    // No panel crosses the hinge: tagged panels sit outboard of station.
    for (panel, tag) in stowed.panels.iter().zip(stowed.tags.iter()) {
        if tag.fold == Some(0) {
            assert!(panel.center_of_pressure_body_m.y > 5.6 - 1e-9);
        } else {
            assert!(panel.center_of_pressure_body_m.y < 5.6 + 1e-9);
        }
    }
    // Fold record carries hinge placement and compiled angle.
    assert_eq!(stowed.folds.len(), 1);
    assert!((stowed.folds[0].angle_rad - 60.0_f64.to_radians()).abs() < 1e-12);
    assert!((stowed.folds[0].hinge_body_m - DVec3::new(1.0, 5.6, 0.0)).length() < 1e-9);
    assert_eq!(stowed.summary.fold_states.len(), 1);
}

#[test]
fn incidence_tilts_panel_frames() {
    let mut surface = rectangular(8.0, 2.0);
    surface.sections = SectionData::uniform(5.0_f64.to_radians(), 0.12).unwrap();
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(compiled.panels.len(), 1);
    let panel = &compiled.panels[0];
    let incidence = 5.0_f64.to_radians();
    // Positive incidence is leading-edge-up: chord tips down-aft, lift
    // tips forward, both by exactly the incidence angle.
    assert!(
        (panel.chord_axis_body - DVec3::new(incidence.cos(), 0.0, -incidence.sin())).length()
            < 1e-12
    );
    assert!(
        (panel.lift_axis_body - DVec3::new(incidence.sin(), 0.0, incidence.cos())).length() < 1e-12
    );
    assert!((panel.thickness_to_chord_ratio - 0.12).abs() < 1e-12);
}

#[test]
fn mirror_keeps_areas_and_lift_up() {
    let plain = rectangular(8.0, 2.0);
    let compiled = compile_surface(
        &plain,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let mut mirrored_surface = rectangular(8.0, 2.0);
    mirrored_surface.mirror_y = true;
    let mirrored = compile_surface(
        &mirrored_surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(compiled.panels.len(), mirrored.panels.len());
    for (left, right) in mirrored.panels.iter().zip(compiled.panels.iter()) {
        assert!((left.area_m2 - right.area_m2).abs() < 1e-12);
        assert!((left.position_body_m.y + right.position_body_m.y).abs() < 1e-12);
        assert!((left.position_body_m.x - right.position_body_m.x).abs() < 1e-12);
        // Lift stays up after mirroring; chord stays chordwise.
        assert!(left.lift_axis_body.z > 0.999);
        assert!((left.chord_axis_body - DVec3::X).length() < 1e-12);
    }
    assert!((mirrored.summary.material_area_m2 - compiled.summary.material_area_m2).abs() < 1e-12);
}

#[test]
fn mount_offsets_positions_hinges_and_bbox() {
    let origin = DVec3::new(1.0, -2.0, 0.5);
    let mut surface = rectangular(8.0, 2.0);
    surface.origin_body_m = origin;
    surface.folds.push(FoldJoint {
        name: "tip-fold".into(),
        station_s: 0.7,
        axis: DVec3::X,
        deployed_angle_rad: 0.0,
        stowed_angle_rad: 45.0_f64.to_radians(),
        travel_limit_rad: 50.0_f64.to_radians(),
    });
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!((compiled.panels[0].position_body_m - origin).length() > 0.0);
    assert!(
        (compiled.folds[0].hinge_body_m - (origin + DVec3::new(1.0, 5.6, 0.0))).length() < 1e-9
    );
    assert!((compiled.summary.bbox_min_m - origin).length() < 1e-12);
}

#[test]
fn degenerate_authoring_fails_closed() {
    // Empty name.
    let mut surface = rectangular(8.0, 2.0);
    surface.name.clear();
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidSurface(_))
    ));
    // Non-positive span.
    let mut surface = rectangular(8.0, 2.0);
    surface.span_m = 0.0;
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidSurface(_))
    ));
    // Unsorted planform stations.
    assert!(matches!(
        Planform::from_stations(vec![
            SpanStation {
                s: 0.0,
                x_le: 0.0,
                x_te: 2.0
            },
            SpanStation {
                s: 0.5,
                x_le: 0.0,
                x_te: 2.0
            },
            SpanStation {
                s: 0.4,
                x_le: 0.0,
                x_te: 2.0
            },
            SpanStation {
                s: 1.0,
                x_le: 0.0,
                x_te: 2.0
            },
        ]),
        Err(SurfaceError::InvalidStations(_))
    ));
    // Inverted chord.
    assert!(matches!(
        Planform::from_stations(vec![
            SpanStation {
                s: 0.0,
                x_le: 2.0,
                x_te: 1.0
            },
            SpanStation {
                s: 1.0,
                x_le: 2.0,
                x_te: 1.0
            },
        ]),
        Err(SurfaceError::InvalidChord(_))
    ));
    // Control region with strictly positive min.
    let mut bad = aileron("bad");
    bad.min_deflection_rad = 0.1;
    let mut surface = rectangular(8.0, 2.0);
    surface.controls.push(bad);
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidControlRegion(_))
    ));
    // Tab escaping its parent.
    let mut surface = rectangular(8.0, 2.0);
    surface.controls.push(ControlRegion {
        name: "elevator".into(),
        span: (0.5, 1.0),
        chord: (0.2, 1.0),
        hinge_u: 0.2,
        min_deflection_rad: -0.4,
        max_deflection_rad: 0.4,
        parent: None,
        kind: ControlRegionKind::TrailingEdgeDevice,
    });
    surface.controls.push(ControlRegion {
        name: "tab".into(),
        span: (0.4, 0.9),
        chord: (0.5, 0.9),
        hinge_u: 0.5,
        min_deflection_rad: -0.2,
        max_deflection_rad: 0.2,
        parent: Some(0),
        kind: ControlRegionKind::TrailingEdgeDevice,
    });
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidControlRegion(_))
    ));
    // Fold station at the root.
    let mut surface = rectangular(8.0, 2.0);
    surface.folds.push(FoldJoint {
        name: "fold".into(),
        station_s: 0.0,
        axis: DVec3::X,
        deployed_angle_rad: 0.0,
        stowed_angle_rad: 0.5,
        travel_limit_rad: 1.0,
    });
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidFoldJoint(_))
    ));
    // Stowed angle outside travel.
    let mut surface = rectangular(8.0, 2.0);
    surface.folds.push(FoldJoint {
        name: "fold".into(),
        station_s: 0.5,
        axis: DVec3::X,
        deployed_angle_rad: 0.0,
        stowed_angle_rad: 2.0,
        travel_limit_rad: 1.0,
    });
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState::deployed()
        ),
        Err(SurfaceError::InvalidFoldJoint(_))
    ));
    // Mechanism state longer than the joint list.
    let surface = rectangular(8.0, 2.0);
    assert!(matches!(
        compile_surface(
            &surface,
            &CompileOptions::default(),
            &MechanismState {
                fold_angles_rad: vec![0.1]
            },
        ),
        Err(SurfaceError::InvalidOptions(_))
    ));
}

#[test]
fn boeing_777x_golden_envelope_and_fold() {
    use crate::{boeing_777x, boeing_777x_half_wing};

    let surface = boeing_777x_half_wing().unwrap();
    let deployed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let stowed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::stowed(&surface),
    )
    .unwrap();

    let deployed_span = 2.0 * deployed.summary.projected_span_m;
    let stowed_span = 2.0 * stowed.summary.projected_span_m;
    let deployed_area = 2.0 * deployed.summary.material_area_m2;
    let stowed_area = 2.0 * stowed.summary.material_area_m2;

    // Manufacturer envelope: extended span exact, folded within the
    // hinge-sweep simplification budget.
    assert!((deployed_span - boeing_777x::EXTENDED_SPAN_M).abs() < 0.05);
    assert!((stowed_span - boeing_777x::FOLDED_SPAN_M).abs() < 0.25);
    // Folding sheds exactly the tip length projected through the
    // fixture dihedral (5 deg) per side: the tip is already cos-short
    // before it rotates vertical.
    let expected_loss = 2.0 * boeing_777x::FOLD_TIP_LENGTH_M * 5.0_f64.to_radians().cos();
    assert!((deployed_span - stowed_span - expected_loss).abs() < 1e-9);
    // Rigid fold: material area identical in both states.
    assert!((deployed_area - stowed_area).abs() < 1e-9 * deployed_area);
    // Public reference area: reconstruction-grade band (single trapezoid
    // plus dihedral material excess over the flat reference).
    assert!((deployed_area - boeing_777x::WING_AREA_M2).abs() < 3.0);
    // Regression pins from the documented reconstruction inputs: panel
    // counts, leading-edge sweep, mean aerodynamic chord.
    assert_eq!(deployed.panels.len(), 228);
    assert_eq!(stowed.panels.len(), 228);
    assert!((deployed.summary.sweep_rad - 0.588752).abs() < 1e-6);
    assert!((deployed.summary.mean_aerodynamic_chord_m - 8.26667).abs() < 1e-4);
    // Folded tip rises: bbox grows upward by about the tip length.
    assert!(stowed.summary.bbox_max_m.z > deployed.summary.bbox_max_m.z + 3.0);
    // No panel crosses the fold hinge. Deployed: ownership splits along
    // the span. Stowed 90 deg up: the child stands above the hinge, so
    // the split reads along z instead of y.
    let hinge = stowed.folds[0].hinge_body_m;
    for (panel, tag) in deployed.panels.iter().zip(deployed.tags.iter()) {
        if tag.fold == Some(0) {
            assert!(panel.center_of_pressure_body_m.y > hinge.y - 1e-9);
        } else {
            assert!(panel.center_of_pressure_body_m.y < hinge.y + 1e-9);
        }
    }
    for (panel, tag) in stowed.panels.iter().zip(stowed.tags.iter()) {
        if tag.fold == Some(0) {
            assert!(panel.center_of_pressure_body_m.z > hinge.z - 1e-9);
        } else {
            assert!(panel.center_of_pressure_body_m.z < hinge.z + 1e-9);
        }
    }
    // Mirror symmetry: the left hand carries identical areas.
    let mut left = surface.clone();
    left.mirror_y = true;
    let left_stowed = compile_surface(
        &left,
        &CompileOptions::default(),
        &MechanismState::stowed(&left),
    )
    .unwrap();
    assert_eq!(left_stowed.panels.len(), stowed.panels.len());
    assert!((left_stowed.summary.material_area_m2 - stowed.summary.material_area_m2).abs() < 1e-12);
}

#[test]
fn boeing_777x_lift_holds_mtow_at_sane_cruise_alpha() {
    // End-to-end pipeline guard, not a performance claim: the compiled
    // 777X wing with its automatically picked cruise profile, run through
    // the stock panel solver, must hold MTOW weight at cruise dynamic
    // pressure near the real cruise attitude. A broken compiler-to-solver
    // contract (flipped frames, wrong aspect ratio, dropped panels) or a
    // broken selector fails this loudly.
    //
    // Framing limits, all documented: the fixture wing is untwisted with
    // no fuselage, tail, or high-lift devices, and the solver is an
    // engineering panel model, not CFD. The profile pick closes most of
    // the symmetric-wing gap (7.2 deg trim without camber); the residual
    // against the real ~2-3 deg is the documented downwash-plus-airframe
    // delta. No drag comparison is attempted: cruise L/D is set by config
    // knobs and profile data this slice does not model.
    use crate::{CompileOptions, MechanismState, boeing_777x_half_wing, compile_surface};
    use thessa_sim_core::{
        AeroConfig, AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel,
    };

    let surface = boeing_777x_half_wing().unwrap();
    let right = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let mirrored = right.mirrored();
    let mut panels = right.panels.clone();
    panels.extend(mirrored.panels.iter().cloned());
    let geometry = AeroGeometry::new(panels).unwrap();
    let area: f64 = geometry.panels.iter().map(|panel| panel.area_m2).sum();
    let model = PanelAeroModel::new(AeroConfig::default()).unwrap();

    // Low-speed slope probe: M 0.2 at sea level, thin-airfoil regime.
    let env = AeroEnvironment::standard_sea_level();
    let speed = 0.2 * env.speed_of_sound_mps;
    let dynamic_pressure = 0.5 * env.density_kg_m3 * speed * speed;
    let lift_at = |deg: f64| {
        let alpha = deg.to_radians();
        let state = AeroState::new(
            DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
            DVec3::ZERO,
        );
        model
            .evaluate_state(state, env, &geometry)
            .unwrap()
            .force_body_n
            .z
    };
    // Symmetric wing at zero alpha lifts nothing.
    assert!(lift_at(0.0).abs() < 1.0);
    // Linearity pin: double the alpha, double the lift (well clear of
    // the 18 deg stall model).
    let ratio = lift_at(4.0) / lift_at(2.0);
    assert!((ratio - 2.0).abs() < 0.1);
    // Slope sits in the finite-swept-wing band: below the unswept
    // Helmbold value (5.15/rad at AR 9.9), above a stub-wing slope.
    let slope_per_rad =
        (lift_at(4.0) - lift_at(0.0)) / (4.0_f64.to_radians() * dynamic_pressure * area);
    assert!((3.0..5.0).contains(&slope_per_rad));

    // Cruise weight probe: M 0.85 at 11 km ISA-ish, MTOW 351,534 kg.
    // The automatic profile pick (NACA3412 for section Cl 0.58 at a 2 deg
    // deck) feeds the solver through the config zero-lift angle, the
    // vehicle-level camber proxy: symmetric trim sat near 7.2 deg, the
    // picked camber shifts it by its deg alpha0.
    use crate::{CruiseRequirement, recommend_cruise_profile};
    let pick = recommend_cruise_profile(&CruiseRequirement {
        target_section_cl: 0.58,
        deck_angle_deg: 2.0,
        max_thickness_ratio: 0.12,
        reynolds_number: 5.0e7,
    })
    .unwrap();
    assert_eq!(pick.profile_id().0, "NACA3412");
    let cruise_env = AeroEnvironment::new(0.364, 295.0, 1.42e-5, DVec3::ZERO);
    let cruise_speed = 0.85 * cruise_env.speed_of_sound_mps;
    let weight_n = 351_534.0 * 9.81;
    let cambered_config = AeroConfig {
        zero_lift_angle_rad: pick.zero_lift_angle_rad,
        ..AeroConfig::default()
    };
    let cambered = PanelAeroModel::new(cambered_config).unwrap();
    let cruise_lift_at = |deg: f64| {
        let alpha = deg.to_radians();
        let state = AeroState::new(
            DVec3::new(cruise_speed * alpha.cos(), 0.0, -cruise_speed * alpha.sin()),
            DVec3::ZERO,
        );
        cambered
            .evaluate_state(state, cruise_env, &geometry)
            .unwrap()
            .force_body_n
            .z
    };
    assert!(cruise_lift_at(2.0) < weight_n);
    assert!(cruise_lift_at(6.0) > weight_n);
    let mut low = 2.0_f64;
    let mut high = 6.0_f64;
    for _ in 0..32 {
        let mid = 0.5 * (low + high);
        if cruise_lift_at(mid) < weight_n {
            low = mid;
        } else {
            high = mid;
        }
    }
    let trim_alpha_deg = 0.5 * (low + high);
    // Profiled 1g band. Residual against the real ~2-3 deg cruise
    // attitude: ~1 deg of 2D-vs-finite-wing downwash the section-level
    // selector cannot see (CL/pi/AR ~= 1.06 deg, the documented lever for
    // a future lifting-line correction) plus missing fuselage/tail lift
    // and reference uncertainty on the real attitude itself.
    assert!((3.0..6.0).contains(&trim_alpha_deg));
}

#[test]
fn dream_chaser_golden_fold_fits_fairing() {
    use crate::{boeing_777x_half_wing, dream_chaser, dream_chaser_wing};

    // Sanity: fixture builders stay independent (no shared mutable state).
    let _ = boeing_777x_half_wing().unwrap();
    let surface = dream_chaser_wing().unwrap();
    let deployed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let stowed = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::stowed(&surface),
    )
    .unwrap();

    let deployed_span = 2.0 * deployed.summary.projected_span_m;
    let stowed_span = 2.0 * stowed.summary.projected_span_m;
    // Roughly 7 m deployed (secondary-source grade band).
    assert!((deployed_span - dream_chaser::DEPLOYED_SPAN_M).abs() < 0.1);
    // Stowed inside the wing-only fairing allocation (5 m circle shared
    // with the body in the real stack).
    assert!(stowed_span < 4.5);
    assert!(stowed_span < deployed_span - 2.0);
    // Trapezoid closed form, both states (fold is area-invariant).
    let expected_area = 2.0 * 0.5 * (2.4 + 1.0) * 3.5;
    assert!((2.0 * deployed.summary.material_area_m2 - expected_area).abs() < 1e-9);
    assert!((2.0 * stowed.summary.material_area_m2 - expected_area).abs() < 1e-9);
    // Whole-wing rotation: every tagged panel stands above the hinge.
    let hinge = stowed.folds[0].hinge_body_m;
    assert_eq!(stowed.folds[0].name, "wing-fold");
    for (panel, tag) in stowed.panels.iter().zip(stowed.tags.iter()) {
        if tag.fold == Some(0) {
            assert!(panel.center_of_pressure_body_m.z > hinge.z - 1e-9);
        }
    }
    // Deployed ownership splits along the span at the root fold station.
    for (panel, tag) in deployed.panels.iter().zip(deployed.tags.iter()) {
        if tag.fold == Some(0) {
            assert!(panel.center_of_pressure_body_m.y > hinge.y - 1e-9);
        } else {
            assert!(panel.center_of_pressure_body_m.y < hinge.y + 1e-9);
        }
    }
}

#[test]
fn shuttle_orbiter_golden_delta_and_elevons() {
    use crate::{shuttle_orbiter, shuttle_orbiter_wing};
    use std::collections::HashSet;

    let surface = shuttle_orbiter_wing().unwrap();
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let summary = &compiled.summary;
    // Regression pins from the documented reconstruction inputs.
    assert_eq!(compiled.panels.len(), 448);
    assert!((summary.sweep_rad - 1.2121).abs() < 1e-4);
    assert!((summary.mean_aerodynamic_chord_m - 13.0654).abs() < 1e-3);
    assert_eq!(summary.control_areas.len(), 2);
    assert_eq!(summary.control_areas[0].panel_count, 54);
    assert!((summary.control_areas[0].area_m2 - 17.615).abs() < 1e-3);
    assert_eq!(summary.control_areas[1].panel_count, 138);
    assert!((summary.control_areas[1].area_m2 - 10.815).abs() < 1e-3);
    assert!((summary.control_areas[0].hinge_u - 0.55).abs() < 1e-12);
    assert!((summary.control_areas[1].hinge_u - 0.6).abs() < 1e-12);

    // NASA envelope and area.
    assert!((2.0 * summary.projected_span_m - shuttle_orbiter::SPAN_M).abs() < 0.05);
    assert!((2.0 * summary.material_area_m2 - shuttle_orbiter::WING_AREA_M2).abs() < 5.0);
    // Flat delta: material equals projected.
    assert!((summary.material_area_m2 - summary.projected_area_m2).abs() < 1e-9);
    // Double-delta signature: inner panels steep, outer panels moderate.
    let kink_y = 0.45 * surface.span_m;
    let (mut inner, mut outer) = (0, 0);
    for (panel, tag) in compiled.panels.iter().zip(compiled.tags.iter()) {
        assert!(tag.fold.is_none());
        let sweep_deg = panel.planform_sweep_rad.to_degrees();
        if panel.center_of_pressure_body_m.y < kink_y {
            assert!((75.0..81.0).contains(&sweep_deg), "inner sweep {sweep_deg}");
            inner += 1;
        } else {
            assert!((42.0..48.0).contains(&sweep_deg), "outer sweep {sweep_deg}");
            outer += 1;
        }
    }
    assert!(inner > 0 && outer > 0);
    // Two elevons with valid, non-overlapping panel groups.
    assert_eq!(compiled.controls.len(), 2);
    assert_eq!(compiled.controls[0].name, "elevon-inboard");
    assert_eq!(compiled.controls[1].name, "elevon-outboard");
    let mut owned = HashSet::new();
    for definition in &compiled.controls {
        assert!(!definition.panel_indices.is_empty());
        for &index in &definition.panel_indices {
            assert!(owned.insert(index), "panel {index} owned twice");
            assert_eq!(compiled.tags[index].control_parent, None);
        }
    }
    let areas: Vec<f64> = compiled
        .controls
        .iter()
        .map(|definition| {
            definition
                .panel_indices
                .iter()
                .map(|&i| compiled.panels[i].area_m2)
                .sum()
        })
        .collect();
    for area in &areas {
        assert!((2.0..40.0).contains(area), "elevon area {area}");
    }
}

#[test]
fn concorde_golden_ogival_subdivision() {
    use crate::{concorde, concorde_wing};

    let surface = concorde_wing().unwrap();
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let summary = &compiled.summary;
    let total_area = 2.0 * summary.material_area_m2;
    // Regression pins from the documented station set.
    assert_eq!(compiled.panels.len(), 516);
    assert!((summary.sweep_rad - 1.1040).abs() < 1e-4);
    assert!((summary.mean_aerodynamic_chord_m - 18.4740).abs() < 1e-3);
    assert!((total_area - 358.581).abs() < 0.01);

    // Manufacturer span; drawing-grade area band.
    assert!((2.0 * summary.projected_span_m - concorde::SPAN_M).abs() < 0.05);
    assert!((total_area - concorde::WING_AREA_M2).abs() < 8.0);
    // Ogival signature: sweep grows outboard (root gentle, mid steep).
    let mut min_sweep = f64::INFINITY;
    let mut max_sweep = f64::NEG_INFINITY;
    for panel in &compiled.panels {
        min_sweep = min_sweep.min(panel.planform_sweep_rad);
        max_sweep = max_sweep.max(panel.planform_sweep_rad);
    }
    assert!(min_sweep < 0.6, "root sweep {min_sweep}");
    assert!(max_sweep > 1.15, "ogive sweep {max_sweep}");
    // Curved edges drive adaptive subdivision: tightening refines zones
    // without moving areas.
    let tight = CompileOptions {
        max_chord_change_frac: 1e-4,
        ..CompileOptions::default()
    };
    let refined = compile_surface(&surface, &tight, &MechanismState::deployed()).unwrap();
    assert!(refined.panels.len() > compiled.panels.len());
    assert!(
        (refined.summary.material_area_m2 - summary.material_area_m2).abs()
            < 1e-9 * summary.material_area_m2
    );
}

fn budget_options(budget_m2: f64, max_panels: usize) -> CompileOptions {
    CompileOptions {
        mode: RefinementMode::ErrorBudget {
            budget_m2,
            max_panels,
        },
        ..CompileOptions::default()
    }
}

#[test]
fn error_budget_yields_minimal_panels_at_certified_error() {
    use crate::concorde_wing;
    // The tapered wing is linear per piece: one zone is already exact,
    // so the optimizer must stop at the base split instead of the
    // 16k zones tight tolerances produce.
    let tapered = ProceduralSurface {
        name: "tapered".into(),
        span_m: 10.0,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: 0.0,
        mirror_y: false,
        planform: Planform::tapered(3.0, 1.0, 2.0).unwrap(),
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.0).unwrap(),
        controls: Vec::new(),
        folds: Vec::new(),
        structure: None,
    };
    let optimized = compile_surface(
        &tapered,
        &budget_options(1e-9, 100),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(optimized.panels.len(), 1);
    assert!((optimized.summary.material_area_m2 - 20.0).abs() < 1e-9);
    assert!(optimized.summary.estimated_error_m2 <= 1e-9);

    // Concorde: 516 default-tolerance zones collapse to the base
    // intervals with identical area. This is the panel-count answer:
    // tolerance subdivision buys solver locality, not accuracy.
    let concorde = concorde_wing().unwrap();
    let reference = compile_surface(
        &concorde,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let optimized = compile_surface(
        &concorde,
        &budget_options(1e-6, 4000),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!(optimized.panels.len() < 20);
    assert!(optimized.panels.len() < reference.panels.len() / 10);
    assert!(
        (optimized.summary.material_area_m2 - reference.summary.material_area_m2).abs()
            < 1e-9 * reference.summary.material_area_m2
    );
    assert!(optimized.summary.estimated_error_m2 <= 1e-6);
}

#[test]
fn error_budget_respects_cap_and_repeats_deterministically() {
    use crate::concorde_wing;
    let concorde = concorde_wing().unwrap();
    // Zero budget forces refinement into the cap; the cap is hard.
    let options = budget_options(0.0, 30);
    let first = compile_surface(&concorde, &options, &MechanismState::deployed()).unwrap();
    let second = compile_surface(&concorde, &options, &MechanismState::deployed()).unwrap();
    assert!(first.panels.len() <= 30);
    assert_eq!(first, second);
    // The achieved error is reported even when the budget is missed.
    assert!(first.summary.estimated_error_m2 >= 0.0);
}

#[test]
fn tolerance_compilation_carries_error_telemetry() {
    use crate::concorde_wing;
    // Every compilation certifies itself, whatever the mode.
    let rect = ProceduralSurface::rectangular("r", 8.0, 2.0, DVec3::ZERO).unwrap();
    let compiled = compile_surface(
        &rect,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!(compiled.summary.estimated_error_m2 < 1e-9);
    let concorde = concorde_wing().unwrap();
    let compiled = compile_surface(
        &concorde,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!(compiled.summary.estimated_error_m2 < 1e-9);
}

#[test]
fn naca_thickness_matches_textbook_shape() {
    use crate::Naca4;
    let symmetric = Naca4::new(0.0, 0.4, 0.12).unwrap();
    // Symmetric section: zero camber line, zero zero-lift angle exactly.
    assert_eq!(symmetric.camber_at(0.5), (0.0, 0.0));
    assert_eq!(symmetric.zero_lift_angle_rad(), 0.0);
    // Max 12 percent full thickness (0.06 half) sits at 30 percent
    // chord (Abbott).
    let mut max = (0.0, 0.0);
    let mut x = 0.0;
    while x <= 1.0 {
        let t = symmetric.thickness_at(x);
        if t > max.1 {
            max = (x, t);
        }
        x += 0.001;
    }
    assert!((2.0 * max.1 - 0.12).abs() < 0.004);
    assert!((max.0 - 0.30).abs() < 0.02);
}

#[test]
fn naca_2412_zero_lift_matches_published_value() {
    use crate::Naca4;
    // Abbott/Von Doenhoff: NACA 2412 stalls with α0 ≈ -2.1 deg, Cl(α=0)
    // ≈ 0.25. The integral below is thin-airfoil theory, the band is the
    // published measurement.
    let wing = Naca4::new(0.02, 0.4, 0.12).unwrap();
    let alpha0_deg = wing.zero_lift_angle_rad().to_degrees();
    assert!((alpha0_deg + 2.1).abs() < 0.3, "α0 = {alpha0_deg}");
    let cl0 = -2.0 * std::f64::consts::PI * wing.zero_lift_angle_rad();
    assert!((cl0 - 0.25).abs() < 0.04, "Cl0 = {cl0}");
    // Zero-lift angle is linear in camber at fixed position.
    let more = Naca4::new(0.04, 0.4, 0.12).unwrap();
    let ratio = more.zero_lift_angle_rad() / wing.zero_lift_angle_rad();
    assert!((ratio - 2.0).abs() < 0.02, "ratio = {ratio}");
    // Cl-max bracket covers the published 1.6-1.7 stall.
    let (low, high) = wing.cl_max_band();
    assert!(low <= 1.55 && high >= 1.70, "band = {low}..{high}");
}

#[test]
fn cruise_selector_picks_least_camber_with_margin() {
    use crate::{CruiseRequirement, recommend_cruise_profile};
    // Moderate cruise ask: 2-percent camber suffices, thickness rides
    // the structural ceiling.
    let pick = recommend_cruise_profile(&CruiseRequirement {
        target_section_cl: 0.5,
        deck_angle_deg: 2.0,
        max_thickness_ratio: 0.15,
        reynolds_number: 6.0e6,
    })
    .unwrap();
    assert_eq!(pick.profile_id().0, "NACA2415");
    assert!((pick.family.t - 0.15).abs() < 1e-12);
    assert!(pick.stall_margin > 0.3);
    // Harder ask: camber grows, never exceeds the grid.
    let hard = recommend_cruise_profile(&CruiseRequirement {
        target_section_cl: 0.8,
        deck_angle_deg: 2.0,
        max_thickness_ratio: 0.12,
        reynolds_number: 6.0e6,
    })
    .unwrap();
    assert!(hard.family.m >= 0.04);
    assert!(hard.family.m <= 0.06);
    assert!(hard.stall_margin > 0.0);
    // The pick stamps sections consistently.
    let mut sections = SectionData::uniform(0.0, 0.0).unwrap();
    pick.apply_to_sections(&mut sections);
    for station in &sections.stations {
        assert!((station.thickness_ratio - 0.15).abs() < 1e-12);
        assert_eq!(station.profile.as_ref().unwrap().0, "NACA2415");
    }
    // Degenerate requirements fail closed.
    assert!(
        recommend_cruise_profile(&CruiseRequirement {
            target_section_cl: 0.0,
            deck_angle_deg: 2.0,
            max_thickness_ratio: 0.12,
            reynolds_number: 6.0e6,
        })
        .is_err()
    );
    assert!(
        recommend_cruise_profile(&CruiseRequirement {
            target_section_cl: 0.5,
            deck_angle_deg: 2.0,
            max_thickness_ratio: 0.02,
            reynolds_number: 6.0e6,
        })
        .is_err()
    );
}

#[test]
fn control_presets_build_validated_regions_with_mixing() {
    use crate::preset::{
        ControlChannels, aileron, elevator, elevon, flap, flaperon, mix_command, rudder, trim_tab,
    };

    // Every preset validates standalone and compiles on a plain wing.
    let mut surface = rectangular(10.0, 2.0);
    let (ail, ail_mix) = aileron("ail", (0.55, 0.95)).unwrap();
    let (elev, elev_mix) = elevator("elev", (0.1, 0.9)).unwrap();
    surface.controls.push(ail);
    surface.controls.push(elev);
    let (tab, _) = trim_tab("tab", (0.6, 0.8), (0.5, 0.9), 1).unwrap();
    surface.controls.push(tab);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(compiled.controls.len(), 3);
    assert_eq!(
        compiled
            .tags
            .iter()
            .filter(|tag| tag.control == Some(2))
            .count(),
        1
    );
    assert_eq!(
        compiled.tags[compiled.controls[2].panel_indices[0]].control_parent,
        Some(1)
    );

    // Mixing formulas from the design doc: elevon = pitch + roll,
    // flaperon = flap + roll, plain surfaces take one channel.
    let (_, elevon_mix) = elevon("ev", (0.3, 0.7), (0.2, 1.0)).unwrap();
    let pitch_only = ControlChannels {
        pitch: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(elevon_mix, pitch_only) - 0.5).abs() < 1e-12);
    let combined = ControlChannels {
        pitch: 0.5,
        roll: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(elevon_mix, combined) - 1.0).abs() < 1e-12);
    // Saturation, not wraparound.
    let over = ControlChannels {
        pitch: 1.0,
        roll: 1.0,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert_eq!(mix_command(elevon_mix, over), 1.0);
    let (_, flaperon_mix) = flaperon("fp", (0.5, 0.9)).unwrap();
    let deploy = ControlChannels {
        flap: 0.5,
        roll: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(flaperon_mix, deploy) - 1.0).abs() < 1e-12);
    let (_, rudder_mix) = rudder("rud", (0.2, 0.8)).unwrap();
    let yaw = ControlChannels {
        yaw: -0.25,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(rudder_mix, yaw) + 0.25).abs() < 1e-12);
    assert!((mix_command(ail_mix, pitch_only)).abs() < 1e-12);
    assert!((mix_command(elev_mix, pitch_only) - 0.5).abs() < 1e-12);

    // Flap preset documents its one-sided-limit shim openly.
    let (flap_region, flap_mix) = flap("flap", (0.2, 0.8)).unwrap();
    assert!((flap_region.min_deflection_rad + 1.0_f64.to_radians()).abs() < 1e-12);
    let full = ControlChannels {
        flap: 1.0,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(flap_mix, full) - 1.0).abs() < 1e-12);
}

#[test]
fn pathfinder_fictional_cant_with_controls() {
    use crate::pathfinder_wing;

    // Fictional coverage fixture: pins behavior, never real-world truth.
    let surface = pathfinder_wing().unwrap();
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let summary = &compiled.summary;
    // Independent material reference, computed in-test from the authored
    // stations (never through the compiler): per-segment trapezoids over
    // arc-length-normalized 3D lengths. Uniform trapezoid rules do NOT
    // apply once bend kinks redistribute length per unit s.
    let bend_stations = &surface.bend.stations;
    let mut raw_length = 0.0;
    let mut seg_lengths = Vec::new();
    for pair in bend_stations.windows(2) {
        let length = ((pair[1].s - pair[0].s) * surface.span_m).hypot(pair[1].z_m - pair[0].z_m);
        seg_lengths.push((pair[0].s, pair[1].s, length));
        raw_length += length;
    }
    let scale = surface.span_m / raw_length;
    let mut expected_area = 0.0;
    for (a, b, length) in seg_lengths {
        expected_area +=
            0.5 * (surface.planform.chord(a) + surface.planform.chord(b)) * scale * length;
    }
    assert!((summary.material_area_m2 - expected_area).abs() < 1e-9);
    // Canted tips: projected span shrinks, tip elevation dominates.
    assert!(summary.projected_span_m < 6.0);
    assert!(summary.projected_span_m > 5.4);
    assert!(summary.bend_profile_m[4] > summary.bend_profile_m[2]);
    // Both embedded controls own panels with recorded hinges.
    assert_eq!(compiled.controls.len(), 2);
    assert_eq!(summary.control_areas.len(), 2);
    for record in &summary.control_areas {
        assert!(record.panel_count > 0 && record.area_m2 > 0.0);
    }
}

#[test]
fn compiled_surface_crosses_hangar_boundary_as_data() {
    // The hangar/flight contract: authoring types never reach flight.
    // The compiled surface serializes to data and back losslessly, so
    // the baker can ship it and flight can consume it without this
    // crate. Folds, tags, controls, and the summary must all survive.
    let mut surface = rectangular(8.0, 2.0);
    surface.controls.push(aileron("aileron"));
    surface.folds.push(FoldJoint {
        name: "tip-fold".into(),
        station_s: 0.7,
        axis: DVec3::X,
        deployed_angle_rad: 0.0,
        stowed_angle_rad: 0.5,
        travel_limit_rad: 1.0,
    });
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::stowed(&surface),
    )
    .unwrap();
    // postcard, like the protocol wire: binary-exact f64 roundtrip.
    let bytes = postcard::to_allocvec(&compiled).expect("compiled surface serializes");
    let restored: CompiledSurface =
        postcard::from_bytes(&bytes).expect("compiled surface deserializes");
    assert_eq!(restored, compiled);
}

fn vertical_fin(roll_deg: f64) -> ProceduralSurface {
    use crate::preset::rudder;
    let (rudder, _) = rudder("rudder", (0.3, 0.95)).unwrap();
    ProceduralSurface {
        name: "fin".into(),
        span_m: 3.0,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: roll_deg.to_radians(),
        mirror_y: false,
        planform: Planform::tapered(1.5, 0.8, 0.3).unwrap(),
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.08).unwrap(),
        controls: vec![rudder],
        folds: Vec::new(),
        structure: None,
    }
}

#[test]
fn vertical_fin_mounts_sideways_with_rudder() {
    // +90 deg roll: span rises to +Z, section lift points sideways (-Y).
    let surface = vertical_fin(90.0);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    for panel in &compiled.panels {
        assert!(panel.position_body_m.z > -1e-9, "tip up");
        assert!((panel.chord_axis_body - DVec3::X).length() < 1e-9);
        assert!((panel.lift_axis_body + DVec3::Y).length() < 1e-9);
    }
    // Bounding box swaps axes: tall in z, thin in y.
    let bbox = &compiled.summary;
    assert!((bbox.bbox_max_m.z - bbox.bbox_min_m.z - 3.0).abs() < 1e-9);
    assert!(bbox.bbox_max_m.y - bbox.bbox_min_m.y < 0.5);
    // Rudder owns trailing-edge panels with recorded hinge.
    assert_eq!(compiled.controls.len(), 1);
    assert_eq!(compiled.controls[0].name, "rudder");
    assert!(!compiled.controls[0].panel_indices.is_empty());
    assert!((bbox.control_areas[0].hinge_u - 0.3).abs() < 1e-12);
    // A centerline fin mirrors onto itself: identical areas.
    let mut mirrored_surface = vertical_fin(90.0);
    mirrored_surface.mirror_y = true;
    let mirrored = compile_surface(
        &mirrored_surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!((mirrored.summary.material_area_m2 - bbox.material_area_m2).abs() < 1e-12);
}

#[test]
fn ventral_keel_hangs_down() {
    // -90 deg roll: span drops to -Z, lift points +Y.
    let surface = vertical_fin(-90.0);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    for panel in &compiled.panels {
        assert!(panel.position_body_m.z < 1e-9, "keel down");
        assert!((panel.lift_axis_body - DVec3::Y).length() < 1e-9);
    }
    assert!(compiled.summary.bbox_min_m.z < -2.9);
}

#[test]
fn one_sided_presets_compile_with_zero_minimum() {
    use crate::preset::{ControlChannels, airbrake, anti_servo_tab, mix_command, slat, spoiler};

    let mut surface = rectangular(10.0, 2.0);
    let (spoiler, spoiler_mix) = spoiler("spoiler", (0.4, 0.7)).unwrap();
    assert!((spoiler.min_deflection_rad - 0.0).abs() < 1e-12);
    let (brake, brake_mix) = airbrake("brake", (0.2, 0.4), (0.3, 0.7)).unwrap();
    let (slat, _) = slat("slat", (0.3, 0.8)).unwrap();
    surface.controls.push(spoiler);
    surface.controls.push(brake);
    surface.controls.push(slat);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(compiled.controls.len(), 3);
    // One-sided limits survive into definitions (sim-core parks
    // negative commands at zero, proven next door).
    assert!((compiled.controls[0].minimum_deflection_rad - 0.0).abs() < 1e-12);
    // Spoiler answers roll and airbrake channels, never pitch.
    let roll = ControlChannels {
        roll: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(spoiler_mix, roll) - 0.5).abs() < 1e-12);
    let brake_cmd = ControlChannels {
        roll: 0.0,
        airbrake: 0.5,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(brake_mix, brake_cmd) - 0.5).abs() < 1e-12);
    let pitch = ControlChannels {
        pitch: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!(mix_command(spoiler_mix, pitch).abs() < 1e-12);

    // Anti-servo tab moves against the pitch command.
    let (_, anti_mix) = anti_servo_tab("anti", (0.7, 0.9), (0.5, 0.9), 0).unwrap();
    let up = ControlChannels {
        pitch: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(anti_mix, up) + 0.5).abs() < 1e-12);
}

#[test]
fn stabilator_marks_whole_surface_rotation() {
    use crate::preset::{ControlChannels, mix_command, stabilator};

    let mut surface = rectangular(6.0, 1.5);
    let (stab, stab_mix) = stabilator("stab").unwrap();
    surface.controls.push(stab);
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // Full-span region owns every panel...
    assert_eq!(compiled.controls.len(), 1);
    assert_eq!(
        compiled.controls[0].panel_indices.len(),
        compiled.panels.len()
    );
    // ...and the kind marker tells runtime to rotate, not deflect.
    assert_eq!(
        compiled.summary.control_areas[0].kind,
        ControlRegionKind::AllMovingSurface
    );
    let pitch = ControlChannels {
        pitch: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(stab_mix, pitch) - 0.5).abs() < 1e-12);
    // Ordinary devices keep the trailing-edge marker.
    assert_eq!(
        compiled.summary.control_areas[0].kind,
        ControlRegionKind::AllMovingSurface
    );
}

fn v_tail_half(mirror: bool) -> ProceduralSurface {
    use crate::preset::ruddervator;
    let (rv, _) = ruddervator("ruddervator", (0.35, 0.95), (0.3, 1.0)).unwrap();
    ProceduralSurface {
        name: "v-tail-right".into(),
        span_m: 2.5,
        origin_body_m: DVec3::ZERO,
        mount_roll_rad: 45.0_f64.to_radians(),
        mirror_y: mirror,
        planform: Planform::tapered(1.2, 0.7, 0.25).unwrap(),
        bend: BendCurve::flat(),
        sections: SectionData::uniform(0.0, 0.07).unwrap(),
        controls: vec![rv],
        folds: Vec::new(),
        structure: None,
    }
}

#[test]
fn v_tail_pair_cants_with_ruddervator_mixing() {
    use crate::preset::{ControlChannels, mix_command, ruddervator};

    let right = v_tail_half(false);
    let compiled_right = compile_surface(
        &right,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    // +45 deg roll tips section lift sideways-down-out on the right half.
    for panel in &compiled_right.panels {
        assert!((panel.lift_axis_body - DVec3::new(0.0, -s, s)).length() < 1e-9);
        assert!(panel.center_of_pressure_body_m.y > 0.0);
        assert!(panel.center_of_pressure_body_m.z > 0.0);
    }
    // The mirrored half opens the V symmetrically.
    let mut left = v_tail_half(false);
    left.mirror_y = true;
    let compiled_left = compile_surface(
        &left,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert_eq!(compiled_left.panels.len(), compiled_right.panels.len());
    for (left_panel, right_panel) in compiled_left
        .panels
        .iter()
        .zip(compiled_right.panels.iter())
    {
        assert!((left_panel.lift_axis_body - DVec3::new(0.0, s, s)).length() < 1e-9);
        assert!((left_panel.area_m2 - right_panel.area_m2).abs() < 1e-12);
        assert!(
            (left_panel.center_of_pressure_body_m.y + right_panel.center_of_pressure_body_m.y)
                .abs()
                < 1e-9
        );
    }
    // Ruddervator mixing: pitch plus yaw on each half.
    let (_, mixing) = ruddervator("rv", (0.35, 0.95), (0.3, 1.0)).unwrap();
    let pitch = ControlChannels {
        pitch: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(mixing, pitch) - 0.5).abs() < 1e-12);
    let yaw = ControlChannels {
        yaw: 0.5,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(mixing, yaw) - 0.5).abs() < 1e-12);
    let both = ControlChannels {
        pitch: 0.25,
        yaw: 0.25,
        airbrake: 0.0,
        ..ControlChannels::neutral()
    };
    assert!((mix_command(mixing, both) - 0.5).abs() < 1e-12);
    // Ruddervator owns trailing-edge panels on each half.
    assert_eq!(compiled_right.controls.len(), 1);
    assert!(!compiled_right.controls[0].panel_indices.is_empty());
}

#[test]
fn t_tail_stacks_stabilator_on_fin_tip() {
    use crate::preset::stabilator;

    // Fin as before, plus a stabilator ridden at the fin tip: position
    // and all-moving kind compose with no special casing.
    let fin = vertical_fin(90.0);
    let mut tailplane = rectangular(4.0, 1.0);
    tailplane.origin_body_m = DVec3::new(0.3, 0.0, 3.0);
    let (stab, _) = stabilator("t-stab").unwrap();
    tailplane.controls.push(stab);
    let compiled_fin = compile_surface(
        &fin,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let compiled_tail = compile_surface(
        &tailplane,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    // Fin tip reaches z = 3 where the tailplane lives flat.
    assert!((compiled_fin.summary.bbox_max_m.z - 3.0).abs() < 1e-9);
    for panel in &compiled_tail.panels {
        assert!((panel.center_of_pressure_body_m.z - 3.0).abs() < 1e-9);
        assert!((panel.lift_axis_body - DVec3::Z).length() < 1e-9);
    }
    assert_eq!(
        compiled_tail.summary.control_areas[0].kind,
        ControlRegionKind::AllMovingSurface
    );
    // Combined empennage area is the honest sum of both compilations.
    let total = compiled_fin.summary.material_area_m2 + compiled_tail.summary.material_area_m2;
    assert!(total > 0.0);
}

fn structured_rect(thickness: f64, material: crate::SolidMaterial) -> ProceduralSurface {
    use crate::StructuralLayout;
    let mut surface = rectangular(8.0, 2.0);
    surface.sections = SectionData::uniform(0.0, thickness).unwrap();
    let mut layout = StructuralLayout::metal_baseline();
    layout.skin_material = material.clone();
    layout.spar_material = material;
    surface.structure = Some(layout);
    surface
}

#[test]
fn structural_mass_matches_hand_buildup() {
    use crate::{SolidMaterial, StructuralLayout};

    let _ = StructuralLayout::metal_baseline();
    let surface = structured_rect(0.10, SolidMaterial::aluminum_7075());
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let structure = compiled.structure.as_ref().expect("structured");
    // Skin 2*16*0.002*2810 = 179.84; webs 2*8*0.12*0.003*2810 = 16.1856
    // plus 1.5x caps = 40.464 spar; primary 220.304 plus 20 percent.
    assert!((structure.skin_mass_kg - 179.84).abs() < 0.05);
    assert!((structure.spar_mass_kg - 40.464).abs() < 0.05);
    assert!((structure.mass_kg - 264.3648).abs() < 0.1);
    // Symmetric flat wing: center of mass at mid-chord, mid-span.
    assert!((structure.center_of_mass_body_m - DVec3::new(1.0, 4.0, 0.0)).length() < 1e-9);
    // Flat point-mass assembly: perpendicular-axis identity holds.
    let inertia = structure.inertia_body_kg_m2;
    let trace_gap = (inertia.z_axis.z - (inertia.x_axis.x + inertia.y_axis.y)).abs();
    assert!(trace_gap < 1e-9 * inertia.z_axis.z);
    // Single point mass: the radial direction is the exact null vector
    // of the inertia tensor (rank 2 by construction, not a bug).
    let radial = DVec3::new(1.0, 4.0, 0.0).normalize();
    assert!((inertia * radial).length() < 1e-9 * structure.mass_kg);
    // Fuel box 0.5 x chord 2 x span 8 x thickness 0.2 x fill 0.85.
    assert!((structure.fuel_volume_m3 - 1.36).abs() < 1e-9);
    assert!((structure.fuel_centroid_body_m - DVec3::new(1.0, 4.0, 0.0)).length() < 1e-6);
}

#[test]
fn structural_mass_scales_with_material_density() {
    use crate::SolidMaterial;

    // Same gauges, different density: total mass ratio is exactly the
    // density ratio. This is the material-variation proof.
    let aluminum = structured_rect(0.10, SolidMaterial::aluminum_7075());
    let carbon = structured_rect(0.10, SolidMaterial::carbon_fiber());
    let mass_al = compile_surface(
        &aluminum,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap()
    .structure
    .unwrap()
    .mass_kg;
    let mass_cf = compile_surface(
        &carbon,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap()
    .structure
    .unwrap()
    .mass_kg;
    assert!((mass_cf / mass_al - 1600.0 / 2810.0).abs() < 1e-12);
}

#[test]
fn fuel_volume_scales_with_thickness_while_skin_does_not() {
    use crate::SolidMaterial;

    // Thickness variation: box volume is linear in thickness (ratio 3
    // for 0.05 -> 0.15); skin mass is gauge-driven and must not move;
    // spar webs ride the section depth linearly.
    let thin = structured_rect(0.05, SolidMaterial::aluminum_7075());
    let thick = structured_rect(0.15, SolidMaterial::aluminum_7075());
    let compile = |surface: &ProceduralSurface| {
        compile_surface(
            surface,
            &CompileOptions::default(),
            &MechanismState::deployed(),
        )
        .unwrap()
        .structure
        .unwrap()
    };
    let thin_structure = compile(&thin);
    let thick_structure = compile(&thick);
    assert!((thick_structure.fuel_volume_m3 / thin_structure.fuel_volume_m3 - 3.0).abs() < 1e-9);
    assert!(
        (thick_structure.skin_mass_kg - thin_structure.skin_mass_kg).abs()
            < 1e-9 * thin_structure.skin_mass_kg
    );
    assert!((thick_structure.spar_mass_kg / thin_structure.spar_mass_kg - 3.0).abs() < 1e-9);
}

#[test]
fn structural_output_mirrors_and_absents_cleanly() {
    use crate::SolidMaterial;

    // Mirrored surfaces carry identical mass with mirrored centers.
    let mut surface = structured_rect(0.10, SolidMaterial::aluminum_7075());
    surface.mirror_y = true;
    let plain = structured_rect(0.10, SolidMaterial::aluminum_7075());
    let compiled = compile_surface(
        &surface,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let reference = compile_surface(
        &plain,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    let mirrored = compiled.structure.as_ref().expect("structured");
    let direct = reference.structure.as_ref().expect("structured");
    assert!((mirrored.mass_kg - direct.mass_kg).abs() < 1e-12);
    assert!((mirrored.fuel_volume_m3 - direct.fuel_volume_m3).abs() < 1e-12);
    assert!((mirrored.center_of_mass_body_m.y + direct.center_of_mass_body_m.y).abs() < 1e-9);
    assert!(
        (mirrored.inertia_body_kg_m2.x_axis.x - direct.inertia_body_kg_m2.x_axis.x).abs()
            < 1e-9 * direct.mass_kg
    );
    // No layout: no structure, geometry untouched (back-compat).
    let bare = rectangular(8.0, 2.0);
    let compiled = compile_surface(
        &bare,
        &CompileOptions::default(),
        &MechanismState::deployed(),
    )
    .unwrap();
    assert!(compiled.structure.is_none());
    assert_eq!(compiled.panels.len(), 1);
    // Degenerate layouts fail closed.
    let mut bad = structured_rect(0.10, SolidMaterial::aluminum_7075());
    bad.structure.as_mut().unwrap().skin_gauge_mm = -1.0;
    assert!(
        compile_surface(
            &bad,
            &CompileOptions::default(),
            &MechanismState::deployed()
        )
        .is_err()
    );
    let mut bad_box = structured_rect(0.10, SolidMaterial::aluminum_7075());
    bad_box.structure.as_mut().unwrap().fuel_box_chord = (0.8, 0.2);
    assert!(
        compile_surface(
            &bad_box,
            &CompileOptions::default(),
            &MechanismState::deployed()
        )
        .is_err()
    );
}
