use super::*;

#[test]
fn mechanism_metadata_validates_without_touching_forces() {
    use glam::DVec3;

    // All-moving marker plus nested parent ride definitions; the force
    // path ignores them (proved by the unchanged force tests), the
    // mixer will consume them.
    let parent =
        ControlSurfaceDefinition::new("elevator", vec![0], -0.4, 0.4).expect("valid parent");
    let tab = ControlSurfaceDefinition::new("tab", vec![1], -0.2, 0.2)
        .expect("valid tab")
        .with_parent(0);
    assert_eq!(tab.parent_index, Some(0));
    let stab = ControlSurfaceDefinition::new("stab", vec![2], -0.3, 0.3)
        .expect("valid stab")
        .with_kind(ControlKind::AllMoving);
    assert_eq!(stab.kind, ControlKind::AllMoving);
    assert_eq!(
        ControlSurfaceDefinition::new("plain", vec![0], -0.4, 0.4)
            .expect("valid")
            .kind,
        ControlKind::Hinge
    );
    let left = AeroPanel::flat_plate(DVec3::ZERO, 2.0, 1.0).expect("panel");
    assert_eq!(left.fold_index, None);
    let geometry = AeroGeometry::new(vec![
        left,
        AeroPanel::flat_plate(DVec3::new(0.0, 1.0, 0.0), 2.0, 1.0).expect("panel"),
        AeroPanel::flat_plate(DVec3::new(0.0, 2.0, 0.0), 2.0, 1.0).expect("panel"),
    ])
    .expect("geometry");
    let properties =
        RigidBodyProperties::new(1_000.0, glam::DMat3::from_diagonal(DVec3::splat(100.0)))
            .expect("properties");
    // Cyclic parents rejected.
    let cyclic_a = ControlSurfaceDefinition::new("a", vec![0], -0.4, 0.4)
        .expect("valid")
        .with_parent(1);
    let cyclic_b = ControlSurfaceDefinition::new("b", vec![1], -0.4, 0.4)
        .expect("valid")
        .with_parent(0);
    assert!(
        VehicleDefinition::new("cyclic", geometry, properties, vec![cyclic_a, cyclic_b]).is_err()
    );
    // Dangling fold reference rejected.
    let mut bad_panel = left;
    bad_panel.fold_index = Some(3);
    let bad_geometry = AeroGeometry::new(vec![bad_panel]).expect("geometry with tagged panel");
    assert!(
        VehicleDefinition::new(
            "dangling",
            bad_geometry,
            properties,
            vec![parent.clone(), tab.clone(), stab.clone()],
        )
        .is_err()
    );
    // Well-formed mechanism vehicle validates, joints included: joints
    // attach first (untagged panels), then the tag validates against them.
    let good_geometry = AeroGeometry::new(vec![
        AeroPanel::flat_plate(DVec3::ZERO, 2.0, 1.0).expect("panel"),
        AeroPanel::flat_plate(DVec3::new(0.0, 1.0, 0.0), 2.0, 1.0).expect("panel"),
        AeroPanel::flat_plate(DVec3::new(0.0, 2.0, 0.0), 2.0, 1.0).expect("panel"),
    ])
    .expect("geometry");
    let mut vehicle = VehicleDefinition::new(
        "mechanism",
        good_geometry,
        properties,
        vec![parent, tab, stab],
    )
    .expect("mechanism vehicle validates")
    .with_fold_joints(vec![FoldJointRecord {
        name: "wing.tip-fold".into(),
        hinge_body_m: DVec3::new(1.0, 2.0, 0.0),
        axis_body: DVec3::X,
        angle_rad: 0.5,
        deployed_angle_rad: 0.0,
        deployment_rate_rad_s: 0.1,
        lock_window_rad: (-0.05, 0.05),
        max_dynamic_pressure_pa: None,
        parent_joint: None,
    }])
    .expect("joints attach");
    assert_eq!(vehicle.fold_joints.len(), 1);
    vehicle.aero_geometry.panels[0].fold_index = Some(0);
    vehicle.validate().expect("tag resolves against joints");
    assert!(
        FoldJointRecord {
            name: "bad".into(),
            hinge_body_m: DVec3::ZERO,
            axis_body: DVec3::new(1.0, 1.0, 0.0),
            angle_rad: 0.0,
            deployed_angle_rad: 0.0,
            deployment_rate_rad_s: 0.1,
            lock_window_rad: (-0.05, 0.05),
            max_dynamic_pressure_pa: None,
            parent_joint: None,
        }
        .validate()
        .is_err(),
        "non-unit axis rejected"
    );
}

#[test]
fn fold_parent_cycles_rejected() {
    use glam::DVec3;

    let panel = AeroPanel::flat_plate(DVec3::ZERO, 2.0, 1.0).expect("panel");
    let geometry = AeroGeometry::new(vec![panel]).expect("geometry");
    let properties =
        RigidBodyProperties::new(100.0, glam::DMat3::from_diagonal(DVec3::splat(10.0)))
            .expect("properties");
    let joint = |name: &str, parent: Option<usize>| FoldJointRecord {
        name: name.into(),
        hinge_body_m: DVec3::ZERO,
        axis_body: DVec3::X,
        angle_rad: 0.0,
        deployed_angle_rad: 0.0,
        deployment_rate_rad_s: 0.1,
        lock_window_rad: (-0.05, 0.05),
        max_dynamic_pressure_pa: None,
        parent_joint: parent,
    };
    let chained = VehicleDefinition::new("chained", geometry.clone(), properties, vec![])
        .expect("base validates")
        .with_fold_joints(vec![joint("root", None), joint("child", Some(0))]);
    assert!(chained.is_ok());
    let cyclic = VehicleDefinition::new("cyclic", geometry, properties, vec![])
        .expect("base validates")
        .with_fold_joints(vec![joint("a", Some(1)), joint("b", Some(0))]);
    assert!(cyclic.is_err());
}

#[test]
fn diederich_helper_matches_known_slopes() {
    // AR = 2 at 2-D slope 2π: p = 2, slope = 2π*2/(2+2√2) ≈ 2.6026.
    let slope = diederich_lift_slope(2.0 * std::f64::consts::PI, 2.0, 1.0);
    assert!((slope - 2.6026).abs() < 1e-3, "slope = {slope}");
    // High aspect ratio recovers the 2-D slope; the curve is monotone.
    let high = diederich_lift_slope(2.0 * std::f64::consts::PI, 1.0e4, 1.0);
    assert!((high - 2.0 * std::f64::consts::PI).abs() < 0.01);
    let mid = diederich_lift_slope(2.0 * std::f64::consts::PI, 5.0, 1.0);
    assert!(slope < mid && mid < high);
}

#[test]
fn side_force_scale_mutes_side_path_on_all_paths() {
    // Yaw-normal strip at sideslip: the shared beta convention reads
    // pitch-plane-orthogonal flow as sideslip, so body strips mute the
    // side-force path (scale 0) and answer through the lift path only.
    let mut full = AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Y, 4.0, 2.0)
        .expect("valid fin")
        .with_planform(2.0, 1.0, 0.0, 1.0)
        .expect("planform");
    full.side_force_scale = 1.0;
    let mut muted = full;
    muted.side_force_scale = 0.0;
    // Geometry validation covers the scale range through construction.
    for panel in [full, muted] {
        AeroGeometry::new(vec![panel]).expect("valid scale");
    }
    assert!(
        AeroPanel::flat_plate(DVec3::ZERO, 1.0, 1.0)
            .expect("panel")
            .with_side_force_scale(1.5)
            .is_err()
    );
    // Yaw-normal strip under pitch-plane crossflow: the shared beta
    // convention reads it as sideslip (the body-strip double-count),
    // so scale 0 must mute exactly that while lift stays identical.
    let alpha = 5.0_f64.to_radians();
    let speed = 100.0;
    let environment = AeroEnvironment::standard_sea_level();
    let state = AeroState::new(
        DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
        DVec3::ZERO,
    );
    let model = PanelAeroModel::new(AeroConfig::default()).expect("model");
    let case_full =
        AeroCase::new(state, environment, AeroGeometry::new(vec![full]).unwrap()).expect("case");
    let case_muted =
        AeroCase::new(state, environment, AeroGeometry::new(vec![muted]).unwrap()).expect("case");
    let full_result = model.evaluate_detailed(&case_full).expect("result");
    let muted_result = model.evaluate_detailed(&case_muted).expect("result");
    let full_load = &full_result.panel_loads.as_ref().expect("loads")[0];
    let muted_load = &muted_result.panel_loads.as_ref().expect("loads")[0];
    // Muted side force is exactly zero; lift is untouched.
    assert_eq!(muted_load.coefficients.side_force, 0.0);
    assert_eq!(muted_load.coefficients.lift, full_load.coefficients.lift);
    assert!(full_load.coefficients.side_force.abs() > 0.0);
    // Oracle and SIMD paths agree with the AoS path on both scales.
    for (case, expected) in [(&case_full, &full_result), (&case_muted, &muted_result)] {
        let soa = PanelSoA::from_geometry(&case.geometry).expect("soa");
        let oracle = model
            .evaluate_soa_parts(state, environment, &soa, false)
            .expect("oracle");
        assert_eq!(oracle.force_body_n, expected.force_body_n);
        let simd = model
            .evaluate_soa_simd(state, environment, &soa, false)
            .expect("simd");
        let scale = expected.force_body_n.length().max(1.0);
        assert!((simd.force_body_n - expected.force_body_n).length() / scale < 1e-9);
    }
}
