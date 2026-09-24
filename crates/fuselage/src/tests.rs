//! Fuselage compiler tests: closed forms, goldens, Munk slope bands.
//!
//! Every reference value below is derived independently of the compiler:
//! frustum/sphere closed forms, hand mass buildups, slender-body theory
//! (Munk 1924: pointed-body normal-force slope 2/base-area), and AVL VLM
//! anchors where noted. The compiler is never compared against itself.

use glam::DVec3;

use crate::{
    BodyCompileOptions, BodyControlPlane, BodyControlRegion, BodyPort, BodyStation,
    BodyStructuralLayout, InteriorRegion, PortKind, ProceduralBody, RegionKind, compile_body,
    gamma, superellipse_area, superellipse_perimeter,
};

const TAU: f64 = std::f64::consts::TAU;

fn structured(name: &str, stations: Vec<BodyStation>) -> ProceduralBody {
    let mut body = ProceduralBody::new(name, stations, DVec3::ZERO).unwrap();
    body.structure = Some(BodyStructuralLayout::metal_baseline());
    body.validate().unwrap();
    body
}

#[test]
fn gamma_matches_textbook_values() {
    assert!((gamma(1.0) - 1.0).abs() < 1e-12);
    assert!((gamma(2.0) - 1.0).abs() < 1e-12);
    assert!((gamma(0.5) - std::f64::consts::PI.sqrt()).abs() < 1e-10);
    assert!((gamma(1.5) - 0.5 * std::f64::consts::PI.sqrt()).abs() < 1e-12);
    assert!((gamma(5.0) - 24.0).abs() < 1e-9);
}

#[test]
fn superellipse_area_recovers_circle_and_box() {
    assert!((superellipse_area(1.0, 1.0, 2.0) - std::f64::consts::PI).abs() < 1e-9);
    assert!((superellipse_area(2.0, 3.0, 2.0) - 6.0 * std::f64::consts::PI).abs() < 1e-9);
    // n = 12 is box-like: within 2% of the 2w-by-2h rectangle.
    let boxy = superellipse_area(1.0, 1.0, 12.0);
    assert!((boxy - 4.0).abs() / 4.0 < 0.02, "boxy area = {boxy}");
}

#[test]
fn superellipse_perimeter_recovers_circle_and_box() {
    let circle = superellipse_perimeter(2.0, 2.0, 2.0, 2.0, 2.0, 1024).unwrap();
    assert!((circle - 4.0 * std::f64::consts::PI).abs() < 1e-4);
    // Squircle (n = 4) anchor: true perimeter is ~7.0175 for unit
    // semi-axes (Boersma / squircle constant literature).
    let squircle = superellipse_perimeter(1.0, 1.0, 1.0, 4.0, 4.0, 2048).unwrap();
    assert!((squircle - 7.0175).abs() < 0.01, "squircle = {squircle}");
    // n = 12 is box-like but rounds the corners: ~7.64, strictly
    // between the squircle and the sharp 8.0 box.
    let boxy = superellipse_perimeter(1.0, 1.0, 1.0, 12.0, 12.0, 2048).unwrap();
    assert!(boxy > squircle && boxy < 8.0, "boxy perimeter = {boxy}");
    assert!((boxy - 7.645).abs() < 0.01, "boxy perimeter = {boxy}");
    assert!(superellipse_perimeter(1.0, 1.0, 1.0, 2.0, 2.0, 63).is_err());
}

#[test]
fn cylinder_matches_closed_form_volume_and_wet_area() {
    let body = structured(
        "tank",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
    );
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let summary = &compiled.summary;
    // V = pi r^2 L; wet = lateral 2 pi r L plus two end discs.
    assert!((summary.enclosed_volume_m3 - 4.0 * std::f64::consts::PI).abs() < 1e-6);
    assert!((summary.wetted_area_m2 - 10.0 * std::f64::consts::PI).abs() < 1e-4);
    assert!((summary.frontal_area_m2 - std::f64::consts::PI).abs() < 1e-9);
    assert!((summary.base_area_m2 - std::f64::consts::PI).abs() < 1e-9);
    assert!((summary.center_of_volume_m.x - 2.0).abs() < 1e-9);
    // Four axial zones of 1 m, two strip panels each.
    assert_eq!(summary.zone_count, 4);
    assert_eq!(compiled.panels.len(), 8);
    for panel in &compiled.panels {
        assert!(panel.center_of_pressure_body_m.x >= 0.0);
        assert!(panel.center_of_pressure_body_m.x <= 4.0);
    }
}

#[test]
fn body_control_regions_split_axial_zones_and_bind_one_strip_plane() {
    let mut body = ProceduralBody::new(
        "controlled-barrel",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    body.controls.push(
        BodyControlRegion::new("body-rudder", 1.0, 3.0, BodyControlPlane::Yaw, -0.3, 0.3).unwrap(),
    );
    let compiled = compile_body(
        &body,
        &BodyCompileOptions {
            max_zone_length_m: 4.0,
            ..BodyCompileOptions::default()
        },
    )
    .unwrap();

    assert_eq!(compiled.panels.len(), 6);
    assert_eq!(compiled.controls.len(), 1);
    let control = &compiled.controls[0];
    assert_eq!(control.name, "body-rudder");
    assert_eq!(control.minimum_deflection_rad, -0.3);
    assert_eq!(control.maximum_deflection_rad, 0.3);
    assert_eq!(control.panel_indices, vec![3]);
    let controlled_panel = compiled.panels[control.panel_indices[0]];
    assert!(controlled_panel.lift_axis_body.y > 0.99);
    assert!((1.0..=3.0).contains(&controlled_panel.center_of_pressure_body_m.x));
    assert!(
        compiled
            .panels
            .iter()
            .enumerate()
            .filter(|(index, _)| !control.panel_indices.contains(index))
            .all(|(_, panel)| panel.control_deflection_rad == 0.0)
    );
}

#[test]
fn body_control_validation_rejects_overlapping_same_plane_regions() {
    let mut body = ProceduralBody::new(
        "overlapping-controls",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    body.controls = vec![
        BodyControlRegion::new("pitch-a", 0.5, 2.5, BodyControlPlane::Pitch, -0.2, 0.2).unwrap(),
        BodyControlRegion::new("pitch-b", 2.0, 3.5, BodyControlPlane::Pitch, -0.2, 0.2).unwrap(),
    ];
    assert!(body.validate().is_err());
}

#[test]
fn loft_volume_integrates_cross_section_area_between_station_knots() {
    // Width grows 1 -> 2 while height falls 2 -> 1. Both end areas are
    // 2*pi, but the actual interpolated section peaks at 2.25*pi halfway.
    // Integrating A(x)=pi*(1+x)*(2-x) gives 13*pi/6, not a constant-area
    // or endpoint-frustum estimate.
    let body = ProceduralBody::new(
        "crossing-ellipse-axes",
        vec![
            BodyStation::new(0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
            BodyStation::new(1.0, 2.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    let options = BodyCompileOptions {
        max_zone_length_m: 2.0,
        max_area_change_frac: 0.15,
        ..BodyCompileOptions::default()
    };
    let compiled = compile_body(&body, &options).unwrap();
    let expected_volume = 13.0 * std::f64::consts::PI / 6.0;
    assert!((compiled.summary.enclosed_volume_m3 - expected_volume).abs() < 1e-10);
    assert!(compiled.summary.volume_error_m3 >= 0.0);
    assert!(compiled.summary.volume_error_m3 < expected_volume * 1e-10);
    assert!((compiled.summary.center_of_volume_m.x - 0.5).abs() < 1e-10);
    assert!((compiled.summary.frontal_area_m2 - 2.25 * std::f64::consts::PI).abs() < 1e-10);
}

#[test]
fn loft_skin_area_captures_changing_ellipse_axes() {
    let body = ProceduralBody::new(
        "crossing-ellipse-skin",
        vec![
            BodyStation::new(0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
            BodyStation::new(1.0, 2.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    let compiled = compile_body(
        &body,
        &BodyCompileOptions {
            max_zone_length_m: 2.0,
            ..BodyCompileOptions::default()
        },
    )
    .unwrap();

    // Independent midpoint quadrature of |d r/dx x d r/dtheta| for
    // r=(x,(1+x)cos(theta),(2-x)sin(theta)). Equal endpoint perimeters
    // do not make this loft a cylinder: the cross-sectional axes exchange.
    const X_SAMPLES: usize = 512;
    const THETA_SAMPLES: usize = 4096;
    let dx = 1.0 / X_SAMPLES as f64;
    let dtheta = TAU / THETA_SAMPLES as f64;
    let mut lateral_area = 0.0;
    for x_index in 0..X_SAMPLES {
        let x = (x_index as f64 + 0.5) * dx;
        let width = 1.0 + x;
        let height = 2.0 - x;
        for theta_index in 0..THETA_SAMPLES {
            let theta = (theta_index as f64 + 0.5) * dtheta;
            let (sin, cos) = theta.sin_cos();
            let cross_x = height * cos * cos - width * sin * sin;
            lateral_area +=
                (cross_x.powi(2) + height.powi(2) * cos.powi(2) + width.powi(2) * sin.powi(2))
                    .sqrt()
                    * dx
                    * dtheta;
        }
    }
    let cap_area = 2.0 * (2.0 * std::f64::consts::PI);
    let reference_total = lateral_area + cap_area;
    let actual_error = (compiled.summary.wetted_area_m2 - reference_total).abs();
    assert!(
        actual_error / lateral_area < 2e-5,
        "compiled={}, reference={}",
        compiled.summary.wetted_area_m2,
        reference_total
    );
    assert!(
        compiled.summary.lateral_area_error_m2 < lateral_area * 5e-5,
        "estimated error={}, area={lateral_area}",
        compiled.summary.lateral_area_error_m2
    );
    assert!(actual_error < compiled.summary.lateral_area_error_m2 + lateral_area * 1e-6);
}

#[test]
fn cone_matches_closed_form_volume() {
    let body = structured(
        "nose",
        vec![
            BodyStation::round(0.0, 2.0).unwrap(),
            BodyStation::round(3.0, 1e-3).unwrap(),
        ],
    );
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    // V = pi/3 R^2 L with a near-sharp tip.
    let expected = std::f64::consts::PI / 3.0 * 4.0 * 3.0;
    assert!((compiled.summary.enclosed_volume_m3 - expected).abs() / expected < 0.005);
    // Centroid L/4 from the base for a cone.
    assert!((compiled.summary.center_of_volume_m.x - 0.75).abs() < 0.05);
}

#[test]
fn ogive_nose_is_tangent_and_closed() {
    let body = ProceduralBody::ogive_nose("ogive", 1.0, 1.8, 6, DVec3::ZERO).unwrap();
    // Base radius exact, tangent joint (radius falls toward the tip
    // with vanishing initial slope).
    assert!((body.stations[0].half_width_m - 1.0).abs() < 1e-12);
    let slope = (body.stations[1].half_width_m - body.stations[0].half_width_m)
        / (body.stations[1].x_m - body.stations[0].x_m);
    assert!((-0.15..0.0).contains(&slope), "base slope = {slope}");
    // Near-sharp tip.
    assert!(body.stations.last().unwrap().half_width_m < 0.01);
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    assert!(compiled.summary.enclosed_volume_m3 > 0.0);
    // Tangent-ogive volume band: between its cone and cylinder bounds.
    let cone = std::f64::consts::PI / 3.0 * 1.8;
    let cylinder = std::f64::consts::PI * 1.8;
    assert!(compiled.summary.enclosed_volume_m3 > cone);
    assert!(compiled.summary.enclosed_volume_m3 < cylinder);
}

#[test]
fn hull_mass_matches_hand_buildup() {
    let body = structured(
        "tank",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
    );
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let hull = compiled.structure.as_ref().expect("hull mass");
    // Skin: (lateral 8 pi + discs 2 pi) * 2 mm * 2810 Al-7075.
    let skin = 10.0 * std::f64::consts::PI * 0.002 * 2810.0;
    // Frames: 4 rings at the authored 1 m pitch, perimeter 2 pi each,
    // 40x2 mm. The rings are centered within equal axial bays.
    let frames = 4.0 * TAU * 1.0 * 0.040 * 0.002 * 2810.0;
    assert!((hull.skin_mass_kg - skin).abs() / skin < 0.02);
    assert!((hull.frame_mass_kg - frames).abs() / frames < 0.02);
    assert!((hull.mass_kg - skin - frames).abs() / (skin + frames) < 0.02);
    // Symmetric hull balances at midships.
    assert!((hull.center_of_mass_body_m.x - 2.0).abs() < 0.05);
    assert!(hull.center_of_mass_body_m.y.abs() < 1e-9);
}

#[test]
fn circular_hull_skin_matches_closed_form_area_mass_and_inertia() {
    let radius = 1.0;
    let length = 4.0;
    let gauge_m = 0.002;
    let density = 2810.0;
    let mut body = ProceduralBody::new(
        "closed-form-cylinder-shell",
        vec![
            BodyStation::round(0.0, radius).unwrap(),
            BodyStation::round(length, radius).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    let mut layout = BodyStructuralLayout::metal_baseline();
    layout.skin_gauge_mm = gauge_m * 1000.0;
    layout.frame_gauge_mm = 1.0e-12;
    layout.frame_spacing_m = 10.0;
    body.structure = Some(layout);

    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let hull = compiled.structure.unwrap();
    let areal_density = gauge_m * density;
    let side_area = TAU * radius * length;
    let cap_area = 2.0 * std::f64::consts::PI * radius.powi(2);
    let side_mass = side_area * areal_density;
    let cap_mass = cap_area * areal_density;
    let expected_ix = side_mass * radius.powi(2) + cap_mass * radius.powi(2) / 2.0;
    let expected_transverse = side_mass * (radius.powi(2) / 2.0 + length.powi(2) / 3.0)
        + cap_mass * (radius.powi(2) / 4.0 + length.powi(2) / 2.0);
    let expected_wet_area = side_area + cap_area;

    assert!((hull.skin_mass_kg - (side_mass + cap_mass)).abs() / (side_mass + cap_mass) < 1e-6);
    assert!((hull.wetted_area_m2 - expected_wet_area).abs() / expected_wet_area < 1e-7);
    assert!((hull.inertia_body_kg_m2.x_axis.x - expected_ix).abs() / expected_ix < 1e-5);
    assert!(
        (hull.inertia_body_kg_m2.y_axis.y - expected_transverse).abs() / expected_transverse < 1e-5,
        "Iy={}, expected {expected_transverse}",
        hull.inertia_body_kg_m2.y_axis.y
    );
    assert!(
        (hull.inertia_body_kg_m2.z_axis.z - expected_transverse).abs() / expected_transverse < 1e-5,
        "Iz={}, expected {expected_transverse}",
        hull.inertia_body_kg_m2.z_axis.z
    );
    assert!((hull.center_of_mass_body_m - DVec3::new(length / 2.0, 0.0, 0.0)).length() < 1e-9);
    assert!(compiled.summary.lateral_area_error_m2 < expected_wet_area * 1e-5);
}

#[test]
fn frame_spacing_controls_ring_count_not_station_count() {
    let coarse = structured(
        "coarse-stations",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
    );
    let dense = structured(
        "dense-stations",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(0.5, 1.0).unwrap(),
            BodyStation::round(1.0, 1.0).unwrap(),
            BodyStation::round(2.0, 1.0).unwrap(),
            BodyStation::round(3.0, 1.0).unwrap(),
            BodyStation::round(4.0, 1.0).unwrap(),
        ],
    );
    let a = compile_body(&coarse, &BodyCompileOptions::default()).unwrap();
    let b = compile_body(&dense, &BodyCompileOptions::default()).unwrap();
    let frame_a = a.structure.unwrap().frame_mass_kg;
    let frame_b = b.structure.unwrap().frame_mass_kg;
    assert!((frame_a - frame_b).abs() < 1e-10 * frame_a);
}

#[test]
fn mounted_hull_inertia_preserves_com_offset_cross_terms() {
    let base = structured(
        "offset-inertia",
        vec![
            BodyStation::round(0.0, 0.8).unwrap(),
            BodyStation::round(4.0, 0.8).unwrap(),
        ],
    );
    let local = compile_body(&base, &BodyCompileOptions::default())
        .unwrap()
        .structure
        .unwrap();
    let offset = DVec3::new(1.0, 2.0, -0.5);
    let mut mounted_body = base;
    mounted_body.origin_body_m = offset;
    let mounted = compile_body(&mounted_body, &BodyCompileOptions::default())
        .unwrap()
        .structure
        .unwrap();
    let local_com = local.center_of_mass_body_m;
    let mounted_com = local_com + offset;
    let centroidal = local.inertia_body_kg_m2 - crate::point_inertia(local.mass_kg, local_com);
    let expected = centroidal + crate::point_inertia(local.mass_kg, mounted_com);
    let diff = mounted.inertia_body_kg_m2 - expected;
    assert!(diff.x_axis.length() < 1e-9);
    assert!(diff.y_axis.length() < 1e-9);
    assert!(diff.z_axis.length() < 1e-9);
    assert!(mounted.inertia_body_kg_m2.x_axis.y.abs() > 1.0);
}

#[test]
fn munk_pointed_body_slope_lands_near_two_per_base_area() {
    use thessa_sim_core::{AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel};

    // Ogive-nosed body, fineness ~5: Munk (1924) pointed-body slope is
    // 2.0 per base area; the compiler distributes exactly that through
    // zone growth terms with no per-vehicle tuning.
    let mut stations = vec![
        BodyStation::round(0.0, 1.0).unwrap(),
        BodyStation::round(8.0, 1.0).unwrap(),
    ];
    let rho = (1.0_f64 + 2.0_f64.powi(2)) / 2.0;
    for index in 1..=4 {
        let x = 2.0 * index as f64 / 4.0;
        let radius = (rho.powi(2) - x.powi(2)).sqrt() - (rho - 1.0);
        stations.push(BodyStation::round(8.0 + x, radius.max(1e-3)).unwrap());
    }
    let body = ProceduralBody::new("slender", stations, DVec3::ZERO).unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let geometry = AeroGeometry::new(compiled.panels).unwrap();
    let model = PanelAeroModel::new(thessa_sim_core::AeroConfig::default()).unwrap();
    let env = AeroEnvironment::standard_sea_level();
    let speed = 68.0;
    let dynamic = 0.5 * env.density_kg_m3 * speed * speed;
    let base_area = std::f64::consts::PI;
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
            / (dynamic * base_area)
    };
    assert!(lift_at(0.0).abs() < 1e-9);
    let slope = (lift_at(2.0) - lift_at(0.0)) / 2.0_f64.to_radians();
    assert!((1.5..2.5).contains(&slope), "Munk slope = {slope}");
}

#[test]
fn blunt_cylinder_carries_almost_no_potential_normal_force() {
    use thessa_sim_core::{AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel};

    // Constant section means zero Munk growth terms: the potential
    // normal force nearly vanishes (viscous crossflow arrives only
    // through the solver's separated branch at high alpha).
    let body = ProceduralBody::new(
        "pipe",
        vec![
            BodyStation::round(0.0, 1.0).unwrap(),
            BodyStation::round(6.0, 1.0).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let geometry = AeroGeometry::new(compiled.panels).unwrap();
    let model = PanelAeroModel::new(thessa_sim_core::AeroConfig::default()).unwrap();
    let env = AeroEnvironment::standard_sea_level();
    let speed = 68.0;
    let dynamic = 0.5 * env.density_kg_m3 * speed * speed;
    let base_area = std::f64::consts::PI;
    let alpha = 2.0_f64.to_radians();
    let state = AeroState::new(
        DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
        DVec3::ZERO,
    );
    let slope = model
        .evaluate_state(state, env, &geometry)
        .unwrap()
        .force_body_n
        .z
        / (dynamic * base_area * alpha);
    assert!(slope.abs() < 0.6, "cylinder slope = {slope}");
}

#[test]
fn munk_strips_preserve_boattail_sign_and_geometry_derived_magnitude() {
    let body = ProceduralBody::new(
        "flare-then-boat-tail",
        vec![
            BodyStation::round(0.0, 0.5).unwrap(),
            BodyStation::round(1.0, 1.0).unwrap(),
            BodyStation::round(1.01, 0.1).unwrap(),
            BodyStation::round(2.0, 0.1).unwrap(),
        ],
        DVec3::ZERO,
    )
    .unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let pitch_panel_indices: Vec<_> = compiled
        .panels
        .iter()
        .enumerate()
        .filter_map(|(index, panel)| (panel.lift_axis_body.z > 0.99).then_some(index))
        .collect();
    let expansion: Vec<_> = pitch_panel_indices
        .iter()
        .copied()
        .filter(|index| compiled.panels[*index].center_of_pressure_body_m.x < 1.0)
        .collect();
    assert!(
        !expansion.is_empty(),
        "area expansion strip must be emitted"
    );
    let boat_tail: Vec<_> = pitch_panel_indices
        .iter()
        .copied()
        .filter(|index| (1.0..=1.01).contains(&compiled.panels[*index].center_of_pressure_body_m.x))
        .collect();
    assert!(!boat_tail.is_empty(), "boat-tail strip must be emitted");
    assert!(
        boat_tail
            .iter()
            .any(|index| compiled.panels[*index].lift_interference_factor > 10.0),
        "a sharp geometric area gradient must not be clipped to a tuning cap"
    );

    use thessa_sim_core::{
        AeroCase, AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel,
    };
    let alpha = 2.0_f64.to_radians();
    let speed = 68.0;
    let state = AeroState::new(
        DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
        DVec3::ZERO,
    );
    let case = AeroCase::new(
        state,
        AeroEnvironment::standard_sea_level(),
        AeroGeometry::new(compiled.panels).unwrap(),
    )
    .unwrap();
    let result = PanelAeroModel::new(thessa_sim_core::AeroConfig::default())
        .unwrap()
        .evaluate_detailed(&case)
        .unwrap();
    let loads = result.panel_loads.unwrap();
    assert!(
        expansion
            .iter()
            .any(|index| loads[*index].force_body_n.z < 0.0)
    );
    assert!(
        boat_tail
            .iter()
            .any(|index| loads[*index].force_body_n.z > 0.0)
    );
}

#[test]
fn contact_boxes_bound_asymmetric_loft_and_centerline_offsets() {
    let body = ProceduralBody::new(
        "offset-chine-contact",
        vec![
            BodyStation::new(0.0, 2.0, 0.5, 0.25, 2.0, 6.0, -0.5, -0.3).unwrap(),
            BodyStation::new(2.0, 1.0, 1.0, 0.8, 2.0, 6.0, 1.0, 0.7).unwrap(),
        ],
        DVec3::new(10.0, 20.0, 30.0),
    )
    .unwrap();
    let part =
        crate::body_collision_parts(&body, &crate::BodyCollisionOptions::default()).unwrap()[0];
    let thessa_sim_core::CollisionShape::Cuboid { half_extents_m } = part.shape else {
        panic!("asymmetric offset loft needs a conservative cuboid bound");
    };
    for axial in 0..=100 {
        let station = body.stations[0].lerp(body.stations[1], axial as f64 / 100.0);
        for radial in 0..128 {
            let (y, z) = crate::outline_point(
                station.half_width_m,
                station.top_height_m,
                station.bottom_height_m,
                station.top_exponent,
                station.bottom_exponent,
                TAU * radial as f64 / 128.0,
            );
            let point = body.origin_body_m
                + DVec3::new(station.x_m, station.offset_y_m + y, station.offset_z_m + z);
            let relative = point - part.local_position_m;
            assert!(relative.x.abs() <= half_extents_m.x + 1e-12);
            assert!(relative.y.abs() <= half_extents_m.y + 1e-12);
            assert!(relative.z.abs() <= half_extents_m.z + 1e-12);
        }
    }
}

#[test]
fn juno_style_stack_golden_geometry_and_fuel() {
    use crate::{juno_stack, juno_style_stack};

    let body = juno_style_stack().unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let summary = &compiled.summary;
    assert!((summary.length_m - juno_stack::LENGTH_M).abs() < 1e-9);
    assert!((2.0 * summary.enclosed_volume_m3 / summary.length_m).abs() > 0.0);
    // Barrel tank inner volume: pi * 0.99^2 * 4.0 (10 mm wall inset).
    let tank_inner = std::f64::consts::PI * 0.99_f64.powi(2) * juno_stack::TANK_LENGTH_M;
    let tank = &compiled.tanks[0];
    assert_eq!(compiled.tanks.len(), 1);
    assert_eq!(tank.region_name, "main-tank");
    assert!((tank.inner_volume_m3 - tank_inner).abs() / tank_inner < 0.01);
    assert!(tank.inner_volume_error_m3 >= 0.0);
    assert!(tank.mount.intrinsic_inertia_body_kg_m2.x_axis.x > 0.0);
    assert!((tank.mount.tank.full_propellant_kg - tank_inner * 830.0).abs() < 1e-6);
    assert!((tank.mount.loaded_propellant_kg() - tank_inner * 830.0 * 0.95).abs() < 1e-6);
    // LoxMethane bulk 830 kg/m^3 at 95% fill.
    let expected_prop = tank_inner * 830.0 * 0.95;
    assert!((tank.propellant_kg - expected_prop).abs() / expected_prop < 0.01);
    assert_eq!(compiled.interior.len(), 3);
    assert_eq!(compiled.ports.len(), 3);
    // Engine faces aft, nose hatch faces forward.
    let engine = compiled
        .ports
        .iter()
        .find(|port| port.name == "engine-aft")
        .unwrap();
    assert_eq!(engine.axis_body_m, DVec3::NEG_X);
    let hatch = compiled
        .ports
        .iter()
        .find(|port| port.name == "hatch-forward")
        .unwrap();
    assert_eq!(hatch.axis_body_m, DVec3::X);
    // Tank feed shell participates in the pressure-feed cross-check.
    assert!(tank.mount.tank.max_pressure_pa > 0.0);
}

#[test]
fn simpleplanes_style_block_golden_volumes_and_manifest() {
    use crate::{simpleplanes_style_block, sp_block};

    let body = simpleplanes_style_block().unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let summary = &compiled.summary;
    assert!((summary.length_m - sp_block::LENGTH_M).abs() < 1e-9);
    // Hand buildup from the station loft (frustum segments on
    // superellipse areas): tail taper ~2.43, block 3 m at 2.737 m^2,
    // nose taper ~2.32, tip ~0.37; total ~13.3 m^3.
    assert!((summary.enclosed_volume_m3 - 13.3).abs() < 0.6);
    // Boxy sections compile to cuboid contact parts.
    let parts =
        crate::body_collision_parts(&body, &crate::BodyCollisionOptions::default()).unwrap();
    assert_eq!(parts.len(), 4);
    for part in &parts {
        assert!(matches!(
            part.shape,
            thessa_sim_core::CollisionShape::Cuboid { .. }
        ));
    }
    // Manifest mass rides the assembly COM: dry mass carries +120 kg.
    let baggage = compiled
        .interior
        .iter()
        .find(|region| region.name == "baggage")
        .unwrap();
    assert_eq!(baggage.payload_mass_kg, 120.0);
    let hull = compiled.structure.as_ref().expect("hull mass");
    assert!(hull.mass_kg >= 120.0);
    // Contact primitives conservatively bound every station segment with
    // an axis-aligned cuboid, including round Juno sections.
    let juno = crate::juno_style_stack().unwrap();
    let juno_parts =
        crate::body_collision_parts(&juno, &crate::BodyCollisionOptions::default()).unwrap();
    assert_eq!(juno_parts.len(), 8);
    assert!(
        juno_parts
            .iter()
            .all(|part| matches!(part.shape, thessa_sim_core::CollisionShape::Cuboid { .. }))
    );
}

#[test]
fn shell_and_solid_inertia_match_thin_and_solid_cylinders() {
    // Thin circular tube: Ix = m r^2.
    let tube = crate::tube_inertia(10.0, 1.0, 1.0, 4.0);
    assert!((tube.x_axis.x - 10.0).abs() < 1e-9);
    // Solid circular cylinder: Ix = m r^2 / 2.
    let solid = crate::solid_inertia(10.0, 1.0, 1.0, 4.0);
    assert!((solid.x_axis.x - 5.0).abs() < 1e-9);
    // Thin ring: Ix = m r^2.
    let ring = crate::ring_inertia(2.0, 3.0);
    assert!((ring.x_axis.x - 18.0).abs() < 1e-9);
    assert!((ring.y_axis.y - 9.0).abs() < 1e-9);
    let flat_tube = crate::tube_inertia(3.0, 1.0, 1.0e-5, 0.0);
    assert!((flat_tube.z_axis.z - 1.0).abs() < 2e-4);
}

#[test]
fn compiled_body_crosses_hangar_boundary_as_data() {
    // Binary-exact boundary (postcard, same contract as wing surfaces):
    // the baker ships compiled bodies and flight consumes them without
    // this crate. JSON is covered below for shape only: serde_json
    // 1.0.151 float parsing is not correctly rounded at the last ulp
    // (verified against std parse), so text roundtrips cannot assert
    // bitwise equality anywhere in this workspace.
    let body = crate::juno_style_stack().unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let bytes = postcard::to_allocvec(&compiled).expect("serialize");
    let back: crate::CompiledBody = postcard::from_bytes(&bytes).expect("deserialize");
    assert_eq!(compiled, back);
    let authoring = serde_json::to_string(&body).expect("serialize authoring");
    let authoring_back: ProceduralBody = serde_json::from_str(&authoring).expect("deserialize");
    // Authoring JSON carries full-precision floats through a lossy text
    // parse: compare field by field with tolerance on floats.
    assert_eq!(body.name, authoring_back.name);
    assert_eq!(body.stations.len(), authoring_back.stations.len());
    for (a, b) in body.stations.iter().zip(authoring_back.stations.iter()) {
        assert!((a.x_m - b.x_m).abs() < 1e-12);
        assert!((a.half_width_m - b.half_width_m).abs() < 1e-12);
        assert!((a.top_height_m - b.top_height_m).abs() < 1e-12);
        assert!((a.bottom_height_m - b.bottom_height_m).abs() < 1e-12);
    }
    assert_eq!(body.regions.len(), authoring_back.regions.len());
    assert_eq!(body.ports.len(), authoring_back.ports.len());
}

#[test]
fn compilation_is_deterministic() {
    let body = crate::simpleplanes_style_block().unwrap();
    let options = BodyCompileOptions::default();
    assert_eq!(
        compile_body(&body, &options).unwrap(),
        compile_body(&body, &options).unwrap()
    );
}

#[test]
fn degenerate_authoring_fails_closed() {
    // Single station.
    assert!(
        ProceduralBody::new(
            "stub",
            vec![BodyStation::round(0.0, 1.0).unwrap()],
            DVec3::ZERO
        )
        .is_err()
    );
    // Non-increasing x.
    assert!(
        ProceduralBody::new(
            "fold",
            vec![
                BodyStation::round(2.0, 1.0).unwrap(),
                BodyStation::round(1.0, 1.0).unwrap(),
            ],
            DVec3::ZERO
        )
        .is_err()
    );
    // Bad exponent.
    assert!(BodyStation::new(0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 0.0, 0.0).is_err());
    // Overlapping regions.
    let mut body = ProceduralBody::cylinder("t", 4.0, 1.0, DVec3::ZERO).unwrap();
    body.regions = vec![
        InteriorRegion::new("a", 0.5, 2.5, RegionKind::Empty).unwrap(),
        InteriorRegion::new("b", 2.0, 3.5, RegionKind::Empty).unwrap(),
    ];
    assert!(body.validate().is_err());
    // Port outside the loft and oversize port.
    let mut body = ProceduralBody::cylinder("t", 4.0, 1.0, DVec3::ZERO).unwrap();
    body.ports = vec![BodyPort::new("p", 9.0, 0.0, PortKind::Attachment, 0.2).unwrap()];
    assert!(body.validate().is_err());
    let mut body = ProceduralBody::cylinder("t", 4.0, 1.0, DVec3::ZERO).unwrap();
    body.ports = vec![BodyPort::new("p", 2.0, 0.0, PortKind::Attachment, 5.0).unwrap()];
    assert!(compile_body(&body, &BodyCompileOptions::default()).is_err());
    // Wall thicker than the section.
    let mut walled = ProceduralBody::cylinder("t", 4.0, 0.005, DVec3::ZERO).unwrap();
    walled.structure = Some(BodyStructuralLayout::metal_baseline());
    walled.regions = vec![InteriorRegion::new("tank", 0.5, 3.5, RegionKind::Empty).unwrap()];
    assert!(compile_body(&walled, &BodyCompileOptions::default()).is_err());
    // Tank region without a structural layout (no pressure shell source).
    let mut bare = ProceduralBody::cylinder("t", 4.0, 1.0, DVec3::ZERO).unwrap();
    bare.regions = vec![
        InteriorRegion::new(
            "tank",
            0.5,
            3.5,
            RegionKind::Tank {
                propellant: thessa_sim_core::Propellant::LoxMethane,
                fill_fraction: 1.0,
            },
        )
        .unwrap(),
    ];
    assert!(compile_body(&bare, &BodyCompileOptions::default()).is_err());
}

#[test]
fn asymmetric_section_area_and_centroid_match_hand_calc() {
    // w=1, top h=1, bottom h=0.5, n=2: A = 2*1*(pi/4 + 0.5*pi/4).
    let area = crate::section_area_m2(1.0, 1.0, 0.5, 2.0, 2.0);
    assert!((area - 0.75 * std::f64::consts::PI).abs() < 1e-9);
    // Centroid toward the fuller (top) half: hand value +0.2122.
    let (cy, cz) = crate::section_centroid_yz(1.0, 1.0, 0.5, 2.0, 2.0);
    assert!(cy.abs() < 1e-6, "cy = {cy}");
    assert!((cz - 0.2122).abs() < 1e-3, "cz = {cz}");
    // Symmetric sections center exactly (up to polygon error).
    let (sy, sz) = crate::section_centroid_yz(1.0, 1.0, 1.0, 2.0, 2.0);
    assert!(sy.abs() < 1e-9 && sz.abs() < 1e-9);
}

#[test]
fn asymmetric_superellipse_centroid_matches_dense_polygon_reference() {
    let shape = (1.3, 0.9, 0.55, 4.0, 7.0);
    let (actual_y, actual_z) =
        crate::section_centroid_yz(shape.0, shape.1, shape.2, shape.3, shape.4);
    const SAMPLES: usize = 16_384;
    let mut area2 = 0.0;
    let mut first_moment_z = 0.0;
    let mut previous = crate::outline_point(shape.0, shape.1, shape.2, shape.3, shape.4, 0.0);
    for index in 1..=SAMPLES {
        let current = crate::outline_point(
            shape.0,
            shape.1,
            shape.2,
            shape.3,
            shape.4,
            TAU * index as f64 / SAMPLES as f64,
        );
        let cross = previous.0 * current.1 - current.0 * previous.1;
        area2 += cross;
        first_moment_z += (previous.1 + current.1) * cross;
        previous = current;
    }
    let reference_z = first_moment_z / (3.0 * area2);
    assert!(actual_y.abs() < 1e-14);
    assert!(
        (actual_z - reference_z).abs() < 2e-7,
        "{actual_z} vs {reference_z}"
    );
}

#[test]
fn centerline_droop_shifts_zero_lift_without_touching_symmetry() {
    use thessa_sim_core::{AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel};

    // Same cone forebody, straight vs drooped nose centerline: the
    // straight body lifts nothing at zero alpha, the drooped nose
    // (nose-down camber) pushes down. Tilt only tilts chord axes;
    // symmetric bodies compile exactly as before.
    let forebody = |droop: f64| {
        let stations = vec![
            BodyStation::new(0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
            BodyStation::new(4.0, 1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0).unwrap(),
            BodyStation::new(6.0, 0.5, 0.5, 0.5, 2.0, 2.0, 0.0, droop).unwrap(),
            BodyStation::round(6.5, 0.05).unwrap(),
        ];
        let mut with_droop = stations.clone();
        if droop != 0.0 {
            with_droop[3].offset_z_m = droop * 1.2;
        }
        ProceduralBody::new("droop", with_droop, DVec3::ZERO).unwrap()
    };
    let model = PanelAeroModel::new(thessa_sim_core::AeroConfig::default()).unwrap();
    let env = AeroEnvironment::standard_sea_level();
    let speed = 68.0;
    let dynamic = 0.5 * env.density_kg_m3 * speed * speed;
    let lift0 = |body: &ProceduralBody| {
        let compiled = compile_body(body, &BodyCompileOptions::default()).unwrap();
        let geometry = AeroGeometry::new(compiled.panels).unwrap();
        let state = AeroState::new(DVec3::new(speed, 0.0, 0.0), DVec3::ZERO);
        model
            .evaluate_state(state, env, &geometry)
            .unwrap()
            .force_body_n
            .z
            / (dynamic * std::f64::consts::PI)
    };
    assert!(lift0(&forebody(0.0)).abs() < 1e-9);
    let drooped = lift0(&forebody(-0.5));
    // Nose-down camber band (measured -0.43, regression-pinned wide).
    assert!((-0.6..-0.2).contains(&drooped), "drooped CL0 = {drooped}");
}

#[test]
fn dream_chaser_body_golden_volume_and_lifting_slope() {
    use crate::{dream_chaser_body, dream_chaser_style_body};
    use thessa_sim_core::{AeroEnvironment, AeroGeometry, AeroModel, AeroState, PanelAeroModel};

    let body = dream_chaser_style_body().unwrap();
    let compiled = compile_body(&body, &BodyCompileOptions::default()).unwrap();
    let summary = &compiled.summary;
    assert!((summary.length_m - dream_chaser_body::LENGTH_M).abs() < 1e-9);
    assert!((summary.enclosed_volume_m3 - dream_chaser_body::VOLUME_M3).abs() < 0.3);
    assert!((summary.frontal_area_m2 - dream_chaser_body::FRONTAL_AREA_M2).abs() < 0.05);
    assert_eq!(summary.zone_count, 8);
    assert_eq!(compiled.panels.len(), 16);
    assert_eq!(compiled.interior.len(), 3);
    assert_eq!(compiled.ports.len(), 4);
    let cabin = compiled
        .interior
        .iter()
        .find(|region| region.name == "cabin")
        .unwrap();
    assert!(cabin.volume_m3 > 2.0);
    let docking = compiled
        .ports
        .iter()
        .find(|port| port.name == "docking-nose")
        .unwrap();
    assert_eq!(docking.axis_body_m, DVec3::X);

    // Lifting quality: drooped cambered forebody lifts nose-down at
    // zero alpha and holds a Munk-class slope about the frontal area.
    let geometry = AeroGeometry::new(compiled.panels).unwrap();
    let model = PanelAeroModel::new(thessa_sim_core::AeroConfig::default()).unwrap();
    let env = AeroEnvironment::standard_sea_level();
    let speed = 68.0;
    let dynamic = 0.5 * env.density_kg_m3 * speed * speed;
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
            / (dynamic * summary.frontal_area_m2)
    };
    assert!((-0.3..-0.1).contains(&lift_at(0.0)), "CL0");
    let slope = (lift_at(2.0) - lift_at(0.0)) / 2.0_f64.to_radians();
    let slender_body_slope = 2.0 * summary.base_area_m2 / summary.frontal_area_m2;
    assert!(
        (slope - slender_body_slope).abs() < 0.3,
        "slope = {slope}, slender-body reference = {slender_body_slope}"
    );
    // High-alpha windward lift stays bounded and positive past 10 deg.
    let cl_10 = lift_at(10.0);
    let cl_15 = lift_at(15.0);
    assert!((0.04..0.10).contains(&cl_10));
    assert!((0.18..0.25).contains(&cl_15));
    assert!(cl_15 > cl_10);
}
