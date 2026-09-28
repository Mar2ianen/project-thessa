use super::*;

/// High alpha must be drag-dominated, not riding an ever-rising post-stall
/// lift curve. Regression for the old alpha^0.65 + tanh model, under which
/// CL climbed to ~1.3 at 90 deg and the craft held high alpha on minimal
/// thrust. Uses an X-15-like slope/stall/max plus the default flat plate.
#[test]
fn x15_post_stall_is_drag_dominated() {
    use crate::{AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel};
    use glam::DVec3;
    fn coefficients_at(
        slope: f64,
        stall_deg: f64,
        max: f64,
        alpha_deg: f64,
        deflection_rad: f64,
    ) -> crate::AeroCoefficients {
        let mut panel = AeroPanel::flat_plate(DVec3::ZERO, 10.0, 2.0).expect("panel");
        panel.control_deflection_rad = deflection_rad;
        let config = AeroConfig {
            lift_slope_per_rad: slope,
            stall_angle_rad: stall_deg.to_radians(),
            max_lift_coefficient: max,
            ..AeroConfig::default()
        };
        // Constant-speed direction sweep: exact alpha at constant Mach,
        // unlike the tan() construction which blows up Mach near 90 deg.
        let alpha = alpha_deg.to_radians();
        let speed = 120.0;
        let case = crate::AeroCase::new(
            AeroState::new(
                DVec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()),
                DVec3::ZERO,
            ),
            AeroEnvironment::standard_sea_level(),
            AeroGeometry::new(vec![panel]).expect("geometry"),
        )
        .expect("case");
        let model = PanelAeroModel::new(config).expect("model");
        model
            .evaluate_detailed(&case)
            .expect("result")
            .panel_loads
            .expect("loads")[0]
            .coefficients
    }
    // Default flat plate (stall 18 deg): past-stall lift stays bounded and
    // well under the old 1.3-at-90-deg blowup; drag explodes.
    let post = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 25.0, 0.0);
    assert!(
        post.lift < AeroConfig::default().max_lift_coefficient,
        "post-stall CL must stay under max: {}",
        post.lift
    );
    let pre = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 15.0, 0.0);
    assert!(
        post.drag > pre.drag * 2.0,
        "CD must jump past stall: 15deg={} 25deg={}",
        pre.drag,
        post.drag
    );
    // ~90 deg behaves like a flat plate: little lift, ~2.0 drag.
    let flat = coefficients_at(2.0 * std::f64::consts::PI, 18.0, 1.35, 90.0, 0.0);
    assert!(
        flat.lift.abs() < 0.3,
        "flat-plate lift near 90deg must be small: {}",
        flat.lift
    );
    assert!(
        flat.drag > 1.0,
        "flat-plate drag near 90deg must be large: {}",
        flat.drag
    );
    // X-15-like config (stall 22 deg): L/D collapses from efficient
    // attached flight to drag-dominated separated flight.
    let cruise = coefficients_at(4.6, 22.0, 1.45, 15.0, 0.0);
    let high_alpha = coefficients_at(4.6, 22.0, 1.45, 30.0, 0.0);
    let ld_cruise = cruise.lift / cruise.drag;
    let ld_high = high_alpha.lift / high_alpha.drag;
    assert!(
        ld_cruise > 5.0,
        "attached flight must stay efficient: L/D={ld_cruise}"
    );
    assert!(
        ld_high < 2.5,
        "separated flight must be drag-dominated: L/D={ld_high}"
    );
    // Control authority fades in separated flow.
    let attached_gain = coefficients_at(4.6, 22.0, 1.45, 10.0, 0.1).lift
        - coefficients_at(4.6, 22.0, 1.45, 10.0, 0.0).lift;
    let separated_gain = coefficients_at(4.6, 22.0, 1.45, 30.0, 0.1).lift
        - coefficients_at(4.6, 22.0, 1.45, 30.0, 0.0).lift;
    assert!(
        separated_gain.abs() < attached_gain.abs() * 0.8,
        "control must fade when stalled: attached={attached_gain} separated={separated_gain}"
    );
    // The whole 0..90 sweep stays finite and continuous (1 deg steps).
    let mut previous = coefficients_at(4.6, 22.0, 1.45, 0.0, 0.0);
    for deg in 1..=90 {
        let current = coefficients_at(4.6, 22.0, 1.45, deg as f64, 0.0);
        assert!(current.lift.is_finite() && current.drag.is_finite());
        assert!(
            (current.lift - previous.lift).abs() < 0.25,
            "CL jump at {deg}deg"
        );
        assert!(
            (current.drag - previous.drag).abs() < 0.35,
            "CD jump at {deg}deg"
        );
        previous = current;
    }
}

/// SoA oracle equivalence: the structure-of-arrays path must reproduce the
/// AoS panel loop (same equations, shared coefficient code) across attached,
/// stalled and supersonic regimes, with and without control deflection.
#[test]
fn aero_soa_oracle_matches_panel_loop() {
    use crate::{
        AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel, PanelSoA,
    };
    use glam::DVec3;
    let wing = AeroPanel::flat_plate(DVec3::new(0.0, 2.0, 0.0), 12.0, 1.5)
        .expect("wing")
        .with_planform(4.0, 2.6, 0.35, 1.0)
        .expect("wing planform");
    let mut tail = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.5), 4.0, 1.0).expect("tail");
    tail.control_deflection_rad = 0.15;
    tail.center_of_pressure_body_m = DVec3::new(-4.2, 0.0, 0.5);
    let mut fin = AeroPanel::flat_plate(DVec3::new(-4.0, 0.0, 0.0), 2.0, 1.2).expect("fin");
    fin.exposure = 0.6;
    let geometry = AeroGeometry::new(vec![wing, tail, fin]).expect("geometry");
    let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
    assert_eq!(soa.count, 3);
    let model = PanelAeroModel::new(AeroConfig::default()).expect("model");
    // Attached, stalled, supersonic, spinning, crosswind: cover every branch.
    let cases = [
        (150.0, 5.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (150.0, 35.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (
            700.0,
            8.0_f64.to_radians(),
            DVec3::new(10.0, -5.0, 2.0),
            DVec3::new(0.1, -0.2, 0.05),
        ),
        (
            120.0,
            -12.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.3),
        ),
    ];
    for (speed, alpha, wind, omega) in cases {
        // Wind blows along body x/z; the y entry doubles as extra coverage
        // of the cross term in the oracle transcription.
        let velocity = DVec3::new(
            speed * alpha.cos() + wind.x,
            wind.y,
            -speed * alpha.sin() + wind.z,
        );
        let state = AeroState::new(velocity, omega);
        let environment =
            AeroEnvironment::new(1.225, 340.294, 1.81e-5, DVec3::new(wind.x, 0.0, wind.z));
        let reference = model
            .evaluate_state(state, environment, &geometry)
            .expect("reference");
        let candidate = model
            .evaluate_soa_parts(state, environment, &soa, false)
            .expect("soa");
        for (got, want, name) in [
            (candidate.force_body_n, reference.force_body_n, "force"),
            (candidate.moment_body_nm, reference.moment_body_nm, "moment"),
        ] {
            let scale = want.length().max(1.0);
            assert!(
                (got - want).length() / scale < 1e-12,
                "{name} diverged at {speed} m/s: {got:?} vs {want:?}"
            );
        }
        assert_eq!(candidate.panel_count, reference.panel_count);
        assert!((candidate.mach - reference.mach).abs() < 1e-15);
    }
}

/// SIMD fast-path agreement: `evaluate_soa_simd` must reproduce the SoA
/// oracle and the AoS panel loop across dispatch shapes (3 lanes = scalar
/// tail only, 7 = quad + tail, 10 = chunk + tail, 16 = two chunks),
/// attached/stalled/supersonic regimes, and the vortex + hypersonic
/// drag-cutoff model. Kernel math differs from the scalar path only in
/// float association, hence 1e-9 relative (the slot-order bug this guards
/// against diverged at O(1)).
#[test]
fn aero_soa_simd_matches_oracle_and_panel_loop() {
    use crate::{
        AeroConfig, AeroEnvironment, AeroGeometry, AeroPanel, AeroState, PanelAeroModel, PanelSoA,
    };
    use glam::DVec3;
    let mut all_panels = Vec::new();
    for i in 0..16 {
        let fi = i as f64;
        let mut panel = AeroPanel::flat_plate(
            DVec3::new(-4.0 + (i % 4) as f64, 2.0 - 0.5 * fi, 0.2 * (fi % 3.0)),
            1.0 + fi,
            0.8 + 0.1 * (fi % 3.0),
        )
        .expect("panel")
        .with_planform(
            2.0 + 0.4 * fi,
            1.5 + 0.5 * (fi % 5.0),
            0.1 * (fi % 4.0),
            1.0,
        )
        .expect("planform")
        .with_thickness_ratio(0.02 + 0.005 * (fi % 3.0))
        .expect("thickness");
        panel.control_deflection_rad = 0.05 * ((i % 3) as f64 - 1.0);
        if i == 7 {
            panel.exposure = 0.0;
        }
        if i == 11 {
            panel.exposure = 0.4;
        }
        all_panels.push(panel);
    }
    let vortex_config = AeroConfig {
        vortex_lift_factor: 0.6,
        separated_control_factor: 0.0,
        drag_only_above_mach: 5.0,
        ..AeroConfig::default()
    };
    let models = [
        PanelAeroModel::new(AeroConfig::default()).expect("default model"),
        PanelAeroModel::new(vortex_config).expect("vortex model"),
    ];
    let cases = [
        (150.0, 5.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (150.0, 35.0_f64.to_radians(), DVec3::ZERO, DVec3::ZERO),
        (
            700.0,
            8.0_f64.to_radians(),
            DVec3::new(10.0, -5.0, 2.0),
            DVec3::new(0.1, -0.2, 0.05),
        ),
        (
            120.0,
            -12.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.3),
        ),
        (
            1800.0,
            10.0_f64.to_radians(),
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 0.5),
        ),
    ];
    for count in [3usize, 7, 10, 16] {
        let geometry = AeroGeometry::new(all_panels[..count].to_vec()).expect("geometry");
        let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
        assert_eq!(soa.count, count);
        for model in &models {
            for (speed, alpha, wind, omega) in cases {
                let velocity = DVec3::new(
                    speed * alpha.cos() + wind.x,
                    wind.y,
                    -speed * alpha.sin() + wind.z,
                );
                let state = AeroState::new(velocity, omega);
                let environment =
                    AeroEnvironment::new(1.225, 340.294, 1.81e-5, DVec3::new(wind.x, 0.0, wind.z));
                let reference = model
                    .evaluate_state(state, environment, &geometry)
                    .expect("reference");
                let oracle = model
                    .evaluate_soa_parts(state, environment, &soa, false)
                    .expect("oracle");
                let candidate = model
                    .evaluate_soa_simd(state, environment, &soa, true)
                    .expect("simd");
                for (got, want, name) in [
                    (candidate.force_body_n, reference.force_body_n, "force"),
                    (candidate.moment_body_nm, reference.moment_body_nm, "moment"),
                ] {
                    let scale = want.length().max(1.0);
                    assert!(
                        (got - want).length() / scale < 1e-9,
                        "{name} vs AoS diverged at {count} panels, {speed} m/s: {got:?} vs {want:?}"
                    );
                }
                for (got, want, name) in [
                    (candidate.force_body_n, oracle.force_body_n, "force"),
                    (candidate.moment_body_nm, oracle.moment_body_nm, "moment"),
                ] {
                    let scale = want.length().max(1.0);
                    assert!(
                        (got - want).length() / scale < 1e-9,
                        "{name} vs oracle diverged at {count} panels, {speed} m/s: {got:?} vs {want:?}"
                    );
                }
                assert_eq!(candidate.panel_count, count);
                assert!((candidate.mach - reference.mach).abs() < 1e-15);
                let loads = candidate.panel_loads.expect("loads recorded");
                assert_eq!(loads.len(), count);
            }
        }
    }
    // Caller-owned scratch must reproduce the allocating variant bit for
    // bit, including vacuum-parked lanes whose kernel inputs stay stale
    // from the previous warm evaluation (their outputs are ignored by
    // force assembly, so reuse is sound).
    use crate::AeroSimdScratch;
    let geometry = AeroGeometry::new(all_panels[..10].to_vec()).expect("geometry");
    let soa = PanelSoA::from_geometry(&geometry).expect("soa layout");
    let model = PanelAeroModel::new(AeroConfig::default()).expect("model");
    let mut scratch = AeroSimdScratch::default();
    let vacuum = AeroEnvironment::new(0.0, 340.294, 1.81e-5, DVec3::ZERO);
    let parked_state = AeroState::new(DVec3::new(150.0, 0.0, -10.0), DVec3::ZERO);
    for state in [
        parked_state,
        AeroState::new(DVec3::ZERO, DVec3::new(0.0, 0.0, 0.1)),
    ] {
        let oracle = model
            .evaluate_soa_parts(state, vacuum, &soa, true)
            .expect("vacuum oracle");
        let fresh = model
            .evaluate_soa_simd(state, vacuum, &soa, true)
            .expect("vacuum simd");
        let reused = model
            .evaluate_soa_simd_scratch(state, vacuum, &soa, true, &mut scratch)
            .expect("vacuum reuse");
        assert_eq!(fresh.force_body_n, reused.force_body_n);
        assert_eq!(fresh.moment_body_nm, reused.moment_body_nm);
        assert_eq!(fresh.force_body_n, oracle.force_body_n);
        assert_eq!(fresh.moment_body_nm, oracle.moment_body_nm);
        assert_eq!(fresh.panel_loads.expect("loads").len(), 10);
    }
}

#[test]
fn batch_states_match_individual_lookups_bitwise() {
    let ephemeris = chain_ephemeris();
    let mut frame = EphemerisFrame::new();
    for seconds in [0.0, 1.0, 8.0 / 120.0, 1_000_000.0, -12_345.678] {
        let time = SimTime(seconds);
        let batch = frame
            .evaluate(&ephemeris, time)
            .expect("batch evaluation")
            .to_vec();
        assert_eq!(batch.len(), ephemeris.bodies.len());
        for body in &ephemeris.bodies {
            let single = ephemeris.body_state(body.id, time).expect("single lookup");
            let batched = batch[body.id.index()];
            assert_eq!(
                batched.position_inertial, single.position_inertial,
                "pos {:?}",
                body.id
            );
            assert_eq!(
                batched.velocity_inertial, single.velocity_inertial,
                "vel {:?}",
                body.id
            );
            assert_eq!(batched, single, "full state {:?}", body.id);
        }
        // Re-evaluating the same frame at a new time must not leak the old
        // generation's completion tags into the new pass.
        assert!(frame.states().len() == ephemeris.bodies.len());
    }
}

#[test]
fn batch_reports_cycles_and_rejects_short_buffers() {
    let mut cyclic = chain_ephemeris();
    cyclic.bodies[1].parent = Some(BodyId(3));
    let mut frame = EphemerisFrame::new();
    assert!(matches!(
        frame.evaluate(&cyclic, SimTime::EPOCH),
        Err(EphemerisError::Cycle(_))
    ));
    // A poisoned frame must still serve a healthy universe afterwards.
    let healthy = chain_ephemeris();
    assert!(frame.evaluate(&healthy, SimTime::EPOCH).is_ok());

    let mut states = vec![BodyState::ORIGIN; 2];
    let mut scratch = EphemerisScratch::new();
    assert!(
        healthy
            .body_states_into(SimTime::EPOCH, &mut states, &mut scratch)
            .is_err()
    );
}

#[test]
fn gravity_and_dominant_from_states_match_naive_paths() {
    let ephemeris = chain_ephemeris();
    let field = GravityField::from_ephemeris(&ephemeris);
    let mut frame = EphemerisFrame::new();
    let positions = [
        DVec3::new(56_000_000.0, 0.0, 0.0),
        DVec3::new(0.0, -49_000_000.0, 1_000_000.0),
    ];
    for seconds in [0.0, 40_000.0] {
        let time = SimTime(seconds);
        let states = frame.evaluate(&ephemeris, time).expect("batch").to_vec();
        for position in positions {
            let naive = field.acceleration(position, time).expect("naive gravity");
            let batched = field
                .acceleration_from_states(position, &states, time)
                .expect("batched gravity");
            assert_eq!(batched, naive);
            assert_eq!(
                ephemeris.dominant_body_from_states(position, &states),
                ephemeris.dominant_body(position, time),
            );
        }
    }
    // A short slice names the missing body instead of panicking.
    let short = &frame.states()[..2];
    assert!(
        field
            .acceleration_from_states(positions[0], short, SimTime(0.0))
            .is_err()
    );
}

#[test]
fn soa_scratch_matches_scalar_panel_loop_within_envelope() {
    // Gate for routing the per-tick flight path through the vectorized SoA
    // kernels. The kernels transcribe the analytic model with a different
    // association order (up to a few ulps on 1e4-scale forces), so this pins
    // an absolute envelope instead of bitwise equality, per AGENTS 10.1:
    // 1e-6 N / 1e-6 N m sit ~9 orders below the decision-relevant scales
    // (254 kN thrust, ~1e3 N m RCS couples and saturation flag) while
    // relative metrics would lie on near-zero loads. The X-15 layout
    // (5 panels) exercises both the 4-wide kernel and the scalar tail.
    let profile = X15StarterProfile::new().expect("x15 profile");
    let mut vehicle = profile.vehicle.clone();
    let model = PanelAeroModel::new(profile.aero_config).expect("aero model");
    let mut panels = PanelSoA::from_geometry(&vehicle.aero_geometry).expect("soa layout");
    let mut scratch = AeroSimdScratch::default();
    let env = |density: f64| AeroEnvironment::new(density, 340.294, 1.81e-5, DVec3::ZERO);
    let flow = |vx: f64, vy: f64, omega: DVec3| AeroState::new(DVec3::new(vx, vy, 0.0), omega);
    let cases: Vec<(AeroState, AeroEnvironment, [f64; 4])> = vec![
        (
            flow(180.0, 0.0, DVec3::ZERO),
            env(1.0),
            [0.0, 0.0, 0.0, 0.0],
        ),
        (
            flow(180.0, -8.0, DVec3::ZERO),
            env(1.0),
            [-0.5, 0.3, 0.2, -0.2],
        ),
        // Past the 22-degree stall angle with rate: separation + omega x r.
        (
            flow(150.0, -70.0, DVec3::new(1.0, 0.5, 0.2)),
            env(0.9),
            [0.8, -0.6, 1.0, -1.0],
        ),
        // Transonic handoff and supersonic thin air with full deflection.
        (
            flow(340.0, 5.0, DVec3::ZERO),
            env(0.8),
            [0.2, 0.0, 0.5, -0.5],
        ),
        (
            flow(680.0, 20.0, DVec3::new(0.1, -0.2, 0.3)),
            env(0.4),
            [1.0, 1.0, -1.0, 1.0],
        ),
        // Declared vacuum parks every lane; still air parks them too.
        (
            flow(7000.0, 0.0, DVec3::ZERO),
            env(0.0),
            [0.4, 0.0, 0.0, 0.0],
        ),
        (flow(0.0, 0.0, DVec3::ZERO), env(1.0), [0.0, 0.0, 0.0, 0.0]),
    ];
    for (state, environment, commands) in cases {
        vehicle
            .apply_control_inputs(&commands)
            .expect("control inputs");
        let geometry = &vehicle.aero_geometry;
        panels.sync_deflections(geometry).expect("deflection sync");
        let scalar = model
            .evaluate_state(state, environment, geometry)
            .expect("scalar evaluation");
        let vector = model
            .evaluate_soa_simd_scratch(state, environment, &panels, false, &mut scratch)
            .expect("soa evaluation");
        assert!(
            (vector.force_body_n - scalar.force_body_n).length() <= 1.0e-6,
            "force envelope {state:?}: {:?} vs {:?}",
            vector.force_body_n,
            scalar.force_body_n,
        );
        assert!(
            (vector.moment_body_nm - scalar.moment_body_nm).length() <= 1.0e-6,
            "moment envelope {state:?}: {:?} vs {:?}",
            vector.moment_body_nm,
            scalar.moment_body_nm,
        );
        assert!(
            (vector.dynamic_pressure_pa - scalar.dynamic_pressure_pa).abs() <= 1.0e-9,
            "q envelope"
        );
        assert!(
            (vector.mach - scalar.mach).abs() <= 1.0e-12,
            "mach envelope"
        );
        assert!(
            (vector.reynolds_number - scalar.reynolds_number).abs() <= 1.0e-6,
            "re envelope"
        );
        assert_eq!(vector.panel_count, scalar.panel_count, "panel count");
    }
}

/// `drag_only_above_mach = -inf` must be rejected. Validation used to accept
/// any infinity as the "disabled" sentinel, but the SIMD coefficient kernels
/// mask only +inf: on the aarch64 NEON tier -inf would produce a NaN lift
/// fade while scalar treats it as no fade at all. +inf stays valid.
#[test]
fn aero_rejects_negative_infinite_drag_cutoff() {
    use crate::{AeroConfig, PanelAeroModel};
    let negative_disabled = AeroConfig {
        drag_only_above_mach: f64::NEG_INFINITY,
        ..AeroConfig::default()
    };
    assert!(
        PanelAeroModel::new(negative_disabled).is_err(),
        "-inf is not the disabled sentinel"
    );
    assert!(
        PanelAeroModel::new(AeroConfig::default()).is_ok(),
        "+inf must remain the disabled sentinel"
    );
    let finite_cutoff = AeroConfig {
        drag_only_above_mach: 2.5,
        ..AeroConfig::default()
    };
    assert!(PanelAeroModel::new(finite_cutoff).is_ok());
}

/// Transonic buffet (Slice F): opt-in lift loss through the panel solver.
mod buffet_tests {
    use super::*;

    fn buffet_models(response: f64) -> (PanelAeroModel, X15StarterProfile) {
        let profile = X15StarterProfile::new().expect("x15 profile");
        let mut config = profile.aero_config;
        config.buffet_response = response;
        let model = PanelAeroModel::new(config).expect("buffet config validates");
        (model, profile)
    }

    fn transonic_high_alpha() -> (AeroState, AeroEnvironment) {
        // M ~ 0.99, body alpha ~26 deg (past the X-15 22 deg stall):
        // deep inside the buffet band on lifting panels.
        let state = AeroState::new(DVec3::new(300.0, 150.0, 0.0), DVec3::ZERO);
        let environment = AeroEnvironment::new(0.8, 340.294, 1.81e-5, DVec3::ZERO);
        (state, environment)
    }

    #[test]
    fn buffet_response_rejects_invalid_range() {
        let profile = X15StarterProfile::new().expect("x15 profile");
        for bad in [f64::NAN, -0.1, 1.1, f64::INFINITY] {
            let mut config = profile.aero_config;
            config.buffet_response = bad;
            assert!(
                PanelAeroModel::new(config).is_err(),
                "response {bad} must not validate"
            );
        }
    }

    #[test]
    fn buffet_is_silent_subsonic_and_bounded_transonic() {
        let (clean, profile) = buffet_models(0.0);
        let (buffeted, _) = buffet_models(1.0);
        let vehicle = profile.vehicle.clone();
        let geometry = &vehicle.aero_geometry;
        let total_area: f64 = geometry.panels.iter().map(|panel| panel.area_m2).sum();

        // Subsonic cruise: no loss whatsoever (well below 1 ulp-scale).
        let cruise = AeroState::new(DVec3::new(180.0, 0.0, 0.0), DVec3::ZERO);
        let env = AeroEnvironment::new(1.0, 340.294, 1.81e-5, DVec3::ZERO);
        let clean_force = clean
            .evaluate_state(cruise, env, geometry)
            .expect("cruise")
            .force_body_n;
        let buffet_force = buffeted
            .evaluate_state(cruise, env, geometry)
            .expect("cruise")
            .force_body_n;
        assert!(
            (clean_force - buffet_force).length() <= 1e-9,
            "subsonic must be untouched"
        );

        // Transonic high alpha: strictly positive loss, bounded by 5% of
        // max lift times dynamic pressure times area (plus solver noise).
        // Moments move with the lost lift through the CoP arms, so only
        // the loss magnitude is pinned here.
        let (state, environment) = transonic_high_alpha();
        let clean = clean
            .evaluate_state(state, environment, geometry)
            .expect("transonic");
        let buffeted = buffeted
            .evaluate_state(state, environment, geometry)
            .expect("transonic");
        let loss = (clean.force_body_n - buffeted.force_body_n).length();
        let speed = DVec3::new(300.0, 150.0, 0.0).length();
        let q = 0.5 * 0.8 * speed * speed;
        let max_lift = profile.aero_config.max_lift_coefficient;
        let bound = 0.05 * max_lift * q * total_area + 1e-6;
        assert!(loss > 1e-6, "band must bite, loss={loss:e}");
        assert!(
            loss <= bound,
            "loss {loss:e} exceeds 5%-of-max-lift {bound:e}"
        );
    }

    #[test]
    fn buffet_soa_matches_scalar_with_response_on() {
        // Response > 0 routes SoA lanes onto the reference scalar path, so
        // parity holds by construction; this pins it instead of trusting
        // the routing.
        let (model, profile) = buffet_models(1.0);
        let vehicle = profile.vehicle.clone();
        let panels = PanelSoA::from_geometry(&vehicle.aero_geometry).expect("soa layout");
        let mut scratch = AeroSimdScratch::default();
        let env = |density: f64| AeroEnvironment::new(density, 340.294, 1.81e-5, DVec3::ZERO);
        let cases = [
            AeroState::new(DVec3::new(180.0, 0.0, 0.0), DVec3::ZERO),
            AeroState::new(DVec3::new(300.0, 150.0, 0.0), DVec3::ZERO),
            AeroState::new(DVec3::new(680.0, 20.0, 0.0), DVec3::new(0.1, -0.2, 0.3)),
        ];
        for state in cases {
            let environment = env(0.8);
            let scalar = model
                .evaluate_state(state, environment, &vehicle.aero_geometry)
                .expect("scalar");
            let vector = model
                .evaluate_soa_simd_scratch(state, environment, &panels, false, &mut scratch)
                .expect("soa");
            assert!(
                (vector.force_body_n - scalar.force_body_n).length() <= 1.0e-6,
                "buffet soa parity: {:?} vs {:?}",
                vector.force_body_n,
                scalar.force_body_n,
            );
        }
    }
}

#[test]
#[ignore = "wall-clock diagnostic; run with --ignored --nocapture"]
fn profile_scalar_vs_soa_dev() {
    let profile = X15StarterProfile::new().expect("x15 profile");
    let vehicle = profile.vehicle.clone();
    let geometry = &vehicle.aero_geometry;
    let model = PanelAeroModel::new(profile.aero_config).expect("aero model");
    let mut panels = PanelSoA::from_geometry(geometry).expect("soa");
    let mut scratch = AeroSimdScratch::default();
    let env = AeroEnvironment::new(1.0, 340.294, 1.81e-5, DVec3::ZERO);
    let state = AeroState::new(DVec3::new(180.0, -8.0, 0.0), DVec3::ZERO);
    // Warmup.
    for _ in 0..200 {
        let _ = model.evaluate_state(state, env, geometry).expect("scalar");
        panels.sync_deflections(geometry).expect("sync");
        let _ = model
            .evaluate_soa_simd_scratch(state, env, &panels, false, &mut scratch)
            .expect("soa");
    }
    let iters = 2000;
    let now = std::time::Instant::now();
    for _ in 0..iters {
        std::hint::black_box(model.evaluate_state(state, env, geometry).expect("scalar"));
    }
    let scalar = now.elapsed();
    let now = std::time::Instant::now();
    for _ in 0..iters {
        panels.sync_deflections(geometry).expect("sync");
        std::hint::black_box(
            model
                .evaluate_soa_simd_scratch(state, env, &panels, false, &mut scratch)
                .expect("soa"),
        );
    }
    let soa = now.elapsed();
    eprintln!("scalar: {:?} total, {:?}/eval", scalar, scalar / iters);
    eprintln!("soa+sync: {:?} total, {:?}/eval", soa, soa / iters);
}
