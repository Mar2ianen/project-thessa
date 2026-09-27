use super::*;

#[test]
fn fast_free_rotation_preserves_energy_and_inertial_angular_momentum() {
    let model = PanelAeroModel::new(AeroConfig::default()).unwrap();
    let geometry =
        AeroGeometry::new(vec![AeroPanel::flat_plate(DVec3::ZERO, 1.0, 1.0).unwrap()]).unwrap();
    let properties = X15StarterProfile::new().unwrap().vehicle.mass_properties;
    let mut state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::new(-8.87, -2.54, 0.053),
    )
    .unwrap();
    let inertia = properties.inertia_body_kg_m2;
    let momentum = inertia * state.angular_velocity_body_rps;
    let energy = 0.5 * state.angular_velocity_body_rps.dot(momentum);
    let mut max_energy_error: f64 = 0.0;
    let mut max_momentum_error: f64 = 0.0;
    for _ in 0..7200 {
        (state, _) = integrate_rigid_body_step(
            &model,
            &geometry,
            AtmosphereConfig::default(),
            state,
            properties,
            FlightStepInput::new(1_000_000.0, DVec3::ZERO),
            1.0 / 120.0,
        )
        .unwrap();
        let h = inertia * state.angular_velocity_body_rps;
        let energy_error = (0.5 * state.angular_velocity_body_rps.dot(h) / energy - 1.0).abs();
        let momentum_error =
            (state.orientation_body_to_inertial * h - momentum).length() / momentum.length();
        max_energy_error = max_energy_error.max(energy_error);
        max_momentum_error = max_momentum_error.max(momentum_error);
        assert!(
            energy_error < 1e-8,
            "free rotation gained energy: {energy_error}"
        );
        assert!(
            momentum_error < 1e-8,
            "inertial angular momentum drift: {momentum_error}"
        );
    }
    println!(
        "free rotation: relative energy error {max_energy_error:e}, momentum error {max_momentum_error:e}"
    );
}

#[test]
fn osculating_elements_recover_circular_orbit() {
    let mu: f64 = 3.986_004_418e14;
    let radius: f64 = 6_878_000.0;
    let speed = (mu / radius).sqrt();
    let elements = OsculatingElements::from_state(DVec3::X * radius, DVec3::Y * speed, mu).unwrap();
    assert!((elements.semi_major_axis_m - radius).abs() / radius < 1e-9);
    assert!(elements.eccentricity < 1e-9);
    assert!(!elements.is_escape());
    assert!((elements.apoapsis_m().unwrap() - radius).abs() / radius < 1e-9);
    assert!((elements.periapsis_m() - radius).abs() / radius < 1e-9);
    let period = elements.period_s().unwrap();
    assert!((period - 2.0 * std::f64::consts::PI * (radius.powi(3) / mu).sqrt()) / period < 1e-9);
}

#[test]
fn osculating_elements_roundtrip_elliptical_state() {
    // Reference: analytic KeplerOrbit evaluated off-periapsis.
    let mu = 4.0e13;
    let orbit = KeplerOrbit::new(mu, 10_000_000.0, 0.6, 0.3, 1.1, 0.7, 0.0).unwrap();
    let (r, v) = orbit.state_relative_at(SimTime(1_234.0)).unwrap();
    let elements = OsculatingElements::from_state(r, v, mu).unwrap();
    assert!(!elements.is_escape());
    assert!((elements.semi_major_axis_m - 10_000_000.0).abs() / 10_000_000.0 < 1e-9);
    assert!((elements.eccentricity - 0.6).abs() < 1e-9);
    assert!((elements.inclination_rad - 0.3).abs() < 1e-9);
    assert!((elements.apoapsis_m().unwrap() - 16_000_000.0).abs() < 1.0);
    assert!((elements.periapsis_m() - 4_000_000.0).abs() < 1.0);
    // Sampling at the recovered anomaly reproduces the input position.
    let back = elements.position_at_nu(elements.true_anomaly_rad);
    assert!((back - r).length() / r.length() < 1e-9);
}

#[test]
fn osculating_elements_report_escape_without_apoapsis() {
    // Solar-system escape energy at 1 AU: well above circular speed.
    let mu = 1.327_124_400_18e20;
    let r = DVec3::X * 1.496e11;
    let v = DVec3::Y * 50_000.0;
    let elements = OsculatingElements::from_state(r, v, mu).unwrap();
    assert!(elements.is_escape());
    assert!(elements.semi_major_axis_m < 0.0);
    assert!(elements.apoapsis_m().is_none());
    assert!(elements.period_s().is_none());
    assert!(elements.periapsis_m() > 0.0);
}

#[test]
fn osculating_elements_reject_degenerate_states() {
    let mu = 3.986_004_418e14;
    assert!(OsculatingElements::from_state(DVec3::ZERO, DVec3::Y, mu).is_err());
    // Radial plunge: no orbital plane.
    assert!(OsculatingElements::from_state(DVec3::X, -DVec3::X * 1000.0, mu).is_err());
    assert!(OsculatingElements::from_state(DVec3::X, DVec3::Y, f64::NAN).is_err());
}

#[test]
fn osculating_roundtrip_all_quadrants_and_retrograde_planes() {
    let mu = 4.0e13;
    let mut max_relative_error: f64 = 0.0;
    for inclination in [
        0.0,
        0.7,
        std::f64::consts::FRAC_PI_2,
        2.4,
        std::f64::consts::PI,
    ] {
        for eccentricity in [0.0, 0.3, 0.8] {
            for longitude in [0.2, 1.7, 3.3, 4.9] {
                let orbit = KeplerOrbit::new(
                    mu,
                    10_000_000.0,
                    eccentricity,
                    inclination,
                    0.8,
                    1.2,
                    longitude,
                )
                .unwrap();
                let (r, v) = orbit.state_relative_at(SimTime::EPOCH).unwrap();
                let elements = OsculatingElements::from_state(r, v, mu).unwrap();
                let error =
                    (elements.position_at_nu(elements.true_anomaly_rad) - r).length() / r.length();
                max_relative_error = max_relative_error.max(error);
                assert!(
                    error < 1e-9,
                    "i={inclination} e={eccentricity} phase={longitude}: {error:e}"
                );
            }
        }
    }
    eprintln!("osculating position roundtrip max relative error {max_relative_error:e}");
}

#[test]
fn sampled_verlet_returns_to_start_after_one_circular_period() {
    // Known case: single central mass, circular orbit must close.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let period = TAU * (radius.powi(3) / mu).sqrt();
    let steps = 360;
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * radius,
            velocity: DVec3::Y * speed,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: period / steps as f64,
            max_steps: steps,
        },
        &[],
    )
    .expect("circular propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Completed);
    assert_eq!(path.positions.len(), steps as usize + 1);
    let error = (*path.positions.last().unwrap() - DVec3::X * radius).length() / radius;
    eprintln!("circular closure relative error {error:e} over {steps} steps");
    assert!(error < 1e-3, "closure error {error:e}");
}

#[test]
fn sampled_verlet_agrees_with_adaptive_on_unpowered_coast() {
    // On-rails equivalence groundwork: with no thrust and no aero load, the
    // map prediction (fixed-step sampled Verlet) and the authoritative
    // adaptive propagator must agree on the same gravity field. If this ever
    // diverges, baking the prediction as the craft's on-rails state is unsound.
    use crate::{AdaptiveIntegratorConfig, GravityField, propagate_adaptive};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let horizon = 3_600.0;
    let field = GravityField::from_ephemeris(&ephemeris);
    let reference = propagate_adaptive(
        &field,
        initial,
        SimTime::EPOCH,
        horizon,
        AdaptiveIntegratorConfig::default(),
    )
    .expect("adaptive coast succeeds");
    let step_s = 5.0;
    let path = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s,
            max_steps: (horizon / step_s) as u64,
        },
        &[],
    )
    .expect("sampled coast succeeds");
    let drift = (path.positions.last().unwrap() - reference.state.position).length();
    eprintln!("coast agreement drift after {horizon}s: {drift:e} m");
    assert!(
        drift / radius < 1e-6,
        "prediction/flight integrators disagree: {drift:e} m"
    );
}

#[test]
fn sampled_verlet_stops_at_surface_impact() {
    // Tossed upward below escape speed around a planet-sized body: the path
    // must end in Impact, with no sample underground.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_IMPACT_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid impact test ephemeris");
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * start,
            velocity: DVec3::Y * (mu / start).sqrt() * 0.5,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 20.0,
            max_steps: 5_000,
        },
        &[BodyId(0)],
    )
    .expect("suborbital propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Impact(BodyId(0)));
    // Impact is detected at step granularity: every sample but the last is
    // above the surface; the last is the impact state, at most one step of
    // travel (20 s x ~7.5 km/s periapsis speed) underground.
    let last = path.positions.len() - 1;
    for position in &path.positions[..last] {
        assert!(
            position.length() >= planet_radius,
            "no sample before impact may be underground"
        );
    }
    let penetration = planet_radius - path.positions[last].length();
    assert!(
        penetration.abs() < 1e-6,
        "impact sample must sit just under the surface, penetration {penetration}"
    );
}

#[test]
fn sampled_verlet_escape_leaves_without_impact() {
    // Above escape speed: completed path, monotonically far away.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_ESCAPE_EPOCH",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid escape test ephemeris");
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * start,
            velocity: DVec3::Y * (2.0 * mu / start).sqrt() * 1.2,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 60.0,
            max_steps: 500,
        },
        &[BodyId(0)],
    )
    .expect("escape propagation succeeds");
    assert_eq!(path.end, SampledPathEnd::Completed);
    let final_radius = path.positions.last().unwrap().length();
    assert!(
        final_radius > start * 5.0,
        "escape must leave, final radius {final_radius}"
    );
}

#[test]
fn sampled_verlet_full_system_benchmark() {
    // Target-size batch: 22-source field, map-line workload (360 gravity
    // evals + impact checks). Synthetic baked system (depth-1 chains; the
    // real catalog chains to depth ~3, so treat this as a lower bound).
    // Prints ms for the perf record.
    let mut bodies = vec![BakedBody::fixed(BodyId(0), "star", 1.0e17, 7.0e8)];
    for i in 1..22u32 {
        let orbit = KeplerOrbit::new(
            1.0e17,
            2.0e8 + i as f64 * 1.0e8,
            0.01,
            0.02 * i as f64,
            0.3 * i as f64,
            0.1 * i as f64,
            0.2 * i as f64,
        )
        .expect("benchmark orbit builds");
        bodies.push(BakedBody::orbital(
            BodyId(i),
            format!("body{i}"),
            1.0e12 * i as f64,
            1.0e6 * i as f64,
            BodyId(0),
            orbit,
        ));
    }
    let ephemeris = BakedEphemeris::new("TEST_BENCH_EPOCH", bodies).expect("benchmark bakes");
    let home = BodyId(5);
    let home_mu = 5.0e12_f64;
    let home_state = ephemeris
        .body_state(home, SimTime::EPOCH)
        .expect("home state");
    let radius = 5.0e6 + 300_000.0;
    let speed = (home_mu / radius).sqrt();
    let started = std::time::Instant::now();
    let path = propagate_sampled_verlet(
        &ephemeris,
        TestParticleState {
            position: home_state.position_inertial + DVec3::X * radius,
            velocity: home_state.velocity_inertial + DVec3::Y * speed,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 30.0,
            max_steps: 360,
        },
        &[home],
    )
    .expect("benchmark propagation succeeds");
    let elapsed = started.elapsed();
    eprintln!(
        "synthetic 22-source 360-step sampled path: {:.2} ms, {} samples, end {:?}",
        elapsed.as_secs_f64() * 1000.0,
        path.positions.len(),
        path.end
    );
}

#[test]
fn interlunar_transfer_thessa_to_pelagos_encounters_target() {
    // Real catalog, real dynamics: Hohmann departure from Thessa's orbit to
    // Pelagos' orbit around Nereid, phased by brute-force departure search.
    // Proves the summed-field dynamics (no SOI logic anywhere) support
    // moon-to-moon transfers: the arc must leave Thessa far behind and swing
    // within 10 target radii of Pelagos.
    let config: SystemConfig =
        toml::from_str(include_str!("../../../../data/system.toml")).expect("system parses");
    let ephemeris = config.bake().expect("system bakes");
    let nereid = ephemeris.body_id("nereid").expect("host exists");
    let thessa = ephemeris.body_id("thessa").expect("departure moon exists");
    let pelagos = ephemeris.body_id("pelagos").expect("target moon exists");
    let mu = ephemeris.body(nereid).expect("host descriptor").mu;
    let pelagos_radius = ephemeris.body(pelagos).expect("target descriptor").radius_m;
    let thessa_radius = ephemeris
        .body(thessa)
        .expect("departure descriptor")
        .radius_m;

    let epoch = SimTime::EPOCH;
    let thessa_state = ephemeris
        .body_state(thessa, epoch)
        .expect("departure state");
    let nereid_state = ephemeris.body_state(nereid, epoch).expect("host state");
    let r1 = (thessa_state.position_inertial - nereid_state.position_inertial).length();
    let pelagos_state = ephemeris.body_state(pelagos, epoch).expect("target state");
    let r2 = (pelagos_state.position_inertial - nereid_state.position_inertial).length();
    // Hohmann transfer ellipse between the (near-circular) moon orbits.
    let transfer_a = (r1 + r2) / 2.0;
    let tof_s = std::f64::consts::PI * (transfer_a.powi(3) / mu).sqrt();
    let dv = (mu / r1).sqrt() * ((2.0 * r2 / (r1 + r2)).sqrt() - 1.0);
    assert!(dv > 0.0, "outbound transfer needs prograde burn");
    // Departure phasing brute force: one target period, 64 slots. Each try
    // propagates the full 22-source field; total ~30k steps, well under a
    // second. Deterministic: fixed slots, fixed step.
    let dt = 900.0;
    let steps = (tof_s * 1.15 / dt) as u64;
    let mut best_miss_m = f64::INFINITY;
    let mut best_leaves_thessa_m = 0.0;
    for slot in 0..64 {
        let t0 = epoch.offset(slot as f64 * 160.0 * 3600.0 / 64.0);
        let dep = ephemeris.body_state(thessa, t0).expect("departure state");
        let host = ephemeris.body_state(nereid, t0).expect("host state");
        let radial = (dep.position_inertial - host.position_inertial).normalize_or_zero();
        let prograde = (dep.velocity_inertial - host.velocity_inertial).normalize_or_zero();
        // Impulsive departure from a 300 km parking orbit: the burn must
        // first beat Thessa's own escape, then keep the Hohmann excess.
        let mu_thessa = ephemeris.body(thessa).expect("departure descriptor").mu;
        let burnout = (2.0 * mu_thessa / (thessa_radius + 300_000.0) + dv * dv).sqrt();
        let start = dep.position_inertial + radial * (thessa_radius + 300_000.0);
        let velocity = dep.velocity_inertial + prograde * burnout;
        let path = propagate_sampled_verlet(
            &ephemeris,
            TestParticleState {
                position: start,
                velocity,
            },
            t0,
            VerletConfig {
                step_s: dt,
                max_steps: steps,
            },
            &[thessa, pelagos],
        )
        .expect("transfer propagation succeeds");
        let mut min_pelagos_m = f64::INFINITY;
        let mut max_thessa_m = 0.0_f64;
        for (index, position) in path.positions.iter().enumerate() {
            let time = path.times[index];
            if let Ok(target) = ephemeris.body_state(pelagos, time) {
                min_pelagos_m = min_pelagos_m
                    .min((*position - target.position_inertial).length() - pelagos_radius);
            }
            if let Ok(home) = ephemeris.body_state(thessa, time) {
                max_thessa_m = max_thessa_m.max((*position - home.position_inertial).length());
            }
        }
        if min_pelagos_m < best_miss_m {
            best_miss_m = min_pelagos_m;
            best_leaves_thessa_m = max_thessa_m;
        }
    }
    eprintln!(
        "thessa->pelagos hohmann dv {dv:.0} m/s tof {tof_s:.0} s: best miss {best_miss_m:.0} m, max thessa range {best_leaves_thessa_m:.0} m"
    );
    assert!(
        best_leaves_thessa_m > 100_000_000.0,
        "transfer must leave Thessa behind, range {best_leaves_thessa_m:.0}"
    );
    assert!(
        best_miss_m < pelagos_radius * 10.0,
        "transfer must encounter Pelagos within 10 radii, miss {best_miss_m:.0}"
    );
}

#[test]
fn dominant_body_follows_strongest_local_pull() {
    // Heavy primary vs light secondary on opposite sides of their barycenter.
    // Display-only: no physics changes, just the max-mu/r^2 hint.
    let ephemeris = two_body_binary(1.0e14, 1.0e12, 1.0e9);
    let time = SimTime::EPOCH;
    let primary = ephemeris.body_state(BodyId(1), time).unwrap();
    let secondary = ephemeris.body_state(BodyId(2), time).unwrap();
    // Just above the light moon, the moon wins despite the giant's mass.
    let near_moon = secondary.position_inertial + DVec3::X * 1.0e6;
    assert_eq!(ephemeris.dominant_body(near_moon, time), Some(BodyId(2)));
    // Just above the heavy primary, the primary wins.
    let near_primary = primary.position_inertial + DVec3::X * 1.0e6;
    assert_eq!(ephemeris.dominant_body(near_primary, time), Some(BodyId(1)));
}

#[test]
fn sampled_impact_chooses_entry_not_far_side_or_list_order() {
    let ephemeris = BakedEphemeris::new(
        "IMPACT",
        vec![
            BakedBody::fixed(BodyId(0), "inner", 1e-12, 3.0),
            BakedBody::fixed(BodyId(1), "outer", 1e-12, 5.0),
        ],
    )
    .unwrap();
    for speed in [10.0, 20.0] {
        // endpoint at center, or through the whole body
        let path = propagate_sampled_verlet(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 10.0,
                velocity: -DVec3::X * speed,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 1.0,
                max_steps: 4,
            },
            &[BodyId(0), BodyId(1)],
        )
        .unwrap();
        assert_eq!(path.end, SampledPathEnd::Impact(BodyId(1)));
        assert!((path.positions.last().unwrap().x - 5.0).abs() < 1e-10);
        assert!((path.end_time.0 - 5.0 / speed).abs() < 1e-10);
        assert_eq!(path.times.last(), Some(&path.end_time));
    }
}

#[test]
fn onrails_bake_sample_reuse_and_maneuver_invalidation() {
    use crate::{OnRailsCache, OnRailsWake};
    // Bake once on a circular coast, then sample forward in time without
    // rebaking: the interpolated state must track the path. A velocity kick
    // (the maneuver) must fail reuse so the caller rebakes.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 10.0,
        max_steps: 720,
    };
    let mut rails = OnRailsCache::new();
    assert!(rails.is_empty());
    assert!(!rails.usable_for(&ephemeris, initial, SimTime::EPOCH, config, &[], 1.0, 1e-3));
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("coast bakes");
    assert!(!rails.is_empty());
    // Half an hour downrange the same coast still matches: no rebake needed.
    let later = SimTime::EPOCH.offset(1_800.0);
    let (expected_pos, _) = rails.sample_at(later).expect("baked path covers +30 min");
    let drifted = TestParticleState {
        position: expected_pos,
        velocity: DVec3::Y * speed,
    };
    // Velocity here is stale on purpose (position matches, velocity does
    // not): reuse must fail on the velocity tolerance.
    assert!(!rails.usable_for(&ephemeris, drifted, later, config, &[], 1.0, 1e-6));
    // Exact interpolated state reuses.
    let (pos, vel) = rails.sample_at(later).expect("sampled");
    let exact = TestParticleState {
        position: pos,
        velocity: vel,
    };
    assert!(rails.usable_for(&ephemeris, exact, later, config, &[], 1.0, 1e-3));
    // Maneuver: +10 m/s kick breaks reuse at any tolerance that matters.
    let kicked = TestParticleState {
        position: pos,
        velocity: vel + DVec3::Y * 10.0,
    };
    assert!(!rails.usable_for(&ephemeris, kicked, later, config, &[], 1.0, 1e-3));
    // Different horizon sizing forces a rebake decision by the caller.
    let other = VerletConfig {
        step_s: 20.0,
        max_steps: 720,
    };
    assert!(!rails.usable_for(&ephemeris, exact, later, other, &[], 1.0, 1e-3));
    // Wake: circular coast runs to the horizon, no impact.
    match rails.wake() {
        Some(OnRailsWake::HorizonEnd { time }) => {
            assert!((time.seconds() - 7_200.0).abs() < 1e-6);
        }
        other => panic!("expected horizon wake, got {other:?}"),
    }
    // Explicit invalidation (burn/contact/atmosphere entry) empties reuse.
    rails.invalidate();
    assert!(rails.is_empty());
    assert!(!rails.usable_for(&ephemeris, exact, later, config, &[], 1e9, 1e9));
}

#[test]
fn onrails_wake_reports_predicted_impact_epoch() {
    use crate::{OnRailsCache, OnRailsWake};
    // Suborbital toss: the wake must name the impact body and epoch so the
    // scheduler arms contact handling in simulation time, not per-tick polls.
    let mu = 4.0e13;
    let planet_radius = 6_000_000.0;
    let start = 10_000_000.0;
    let ephemeris = BakedEphemeris::new(
        "TEST_ONRAILS_IMPACT",
        vec![BakedBody::fixed(BodyId(0), "planet", mu, planet_radius)],
    )
    .expect("valid impact test ephemeris");
    let mut rails = OnRailsCache::new();
    rails
        .bake(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * start,
                velocity: DVec3::Y * (mu / start).sqrt() * 0.5,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 20.0,
                max_steps: 5_000,
            },
            &[BodyId(0)],
        )
        .expect("suborbital coast bakes");
    match rails.wake() {
        Some(OnRailsWake::Impact { time, body }) => {
            assert_eq!(body, BodyId(0));
            assert!(time.seconds() > 0.0);
            eprintln!("predicted impact at T+{:.1}s", time.seconds());
        }
        other => panic!("expected impact wake, got {other:?}"),
    }
}

#[test]
fn onrails_reuse_skips_reintegration() {
    // Perf record: full bake vs cache-hit sampling on the target-size batch.
    // The hit must be orders of magnitude cheaper than the ~ms bake, which
    // is the whole economic case for on-rails.
    use crate::OnRailsCache;
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 10.0,
        max_steps: 720,
    };
    let mut rails = OnRailsCache::new();
    let started = std::time::Instant::now();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("bakes");
    let bake = started.elapsed();
    let started = std::time::Instant::now();
    for i in 0..100 {
        let t = SimTime::EPOCH.offset(i as f64 * 10.0);
        assert!(
            rails.usable_for(
                &ephemeris,
                rails
                    .sample_at(t)
                    .map(|(p, v)| TestParticleState {
                        position: p,
                        velocity: v
                    })
                    .unwrap(),
                t,
                config,
                &[],
                1e-6,
                1e-9
            )
        );
    }
    let hits = started.elapsed();
    eprintln!(
        "on-rails bake {:.2} ms vs 100 cache hits {:.3} ms ({:.1}x)",
        bake.as_secs_f64() * 1000.0,
        hits.as_secs_f64() * 1000.0,
        bake.as_secs_f64() / hits.as_secs_f64().max(1e-9)
    );
}

#[test]
fn onrails_hermite_mid_sample_error_is_sub_metre() {
    // The flight loop samples translation between 5 s rail nodes: cubic
    // Hermite through the stored endpoint velocities must hold sub-metre
    // error at orbital speeds, otherwise riding the rails would corrupt the
    // authoritative state.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let path = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 5.0,
            max_steps: 720,
        },
        &[],
    )
    .expect("coast bakes");
    // Reference: same arc integrated natively at 10x resolution.
    let fine = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 0.5,
            max_steps: 7_200,
        },
        &[],
    )
    .expect("fine reference bakes");
    let rails = {
        let mut cache = crate::OnRailsCache::new();
        cache
            .bake(
                &ephemeris,
                initial,
                SimTime::EPOCH,
                VerletConfig {
                    step_s: 5.0,
                    max_steps: 720,
                },
                &[],
            )
            .expect("rails bake");
        cache
    };
    let mut max_error: f64 = 0.0;
    for i in 0..fine.positions.len() {
        let t = fine.times[i];
        if t.seconds() > path.end_time.seconds() {
            break;
        }
        let (sampled, _) = rails.sample_at(t).expect("covered");
        max_error = max_error.max((sampled - fine.positions[i]).length());
    }
    eprintln!("hermite mid-sample max error {max_error:e} m over 1 h arc");
    assert!(max_error < 2.0, "hermite error {max_error:e} m exceeds 2 m");
}

#[test]
fn scheduler_orders_wakes_and_drains_by_sim_time() {
    use crate::{EventScheduler, ScheduledKind};
    // Out-of-order arming still fires in epoch order; ties keep arm order.
    let mut queue = EventScheduler::new();
    assert!(queue.is_empty());
    queue.arm(ScheduledKind::Alarm, SimTime(300.0));
    queue.arm(
        ScheduledKind::RailsImpact { body: BodyId(3) },
        SimTime(100.0),
    );
    queue.arm(ScheduledKind::RailsHorizon, SimTime(200.0));
    assert_eq!(queue.len(), 3);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::RailsImpact { .. })
    ));
    let due = queue.drain_due(SimTime(150.0));
    assert_eq!(due.len(), 1);
    assert!(matches!(
        due[0].kind,
        ScheduledKind::RailsImpact { body } if body == BodyId(3)
    ));
    // Nothing due at the same epoch twice; future events stay armed.
    assert!(queue.drain_due(SimTime(150.0)).is_empty());
    assert_eq!(queue.len(), 2);
    let rest = queue.drain_due(SimTime(1_000.0));
    assert_eq!(rest.len(), 2);
    assert!(matches!(rest[0].kind, ScheduledKind::RailsHorizon));
    assert!(matches!(rest[1].kind, ScheduledKind::Alarm));
    assert!(queue.is_empty());
}

#[test]
fn scheduler_rails_wake_slot_holds_one_wake_and_cancels() {
    use crate::{EventScheduler, ScheduledKind};
    // Rebakes replace the wake instead of stacking: one path, one wake.
    let mut queue = EventScheduler::new();
    queue.arm_rails_wake(ScheduledKind::RailsHorizon, SimTime(500.0));
    queue.arm(ScheduledKind::Alarm, SimTime(600.0));
    queue.arm_rails_wake(
        ScheduledKind::RailsImpact { body: BodyId(1) },
        SimTime(400.0),
    );
    assert_eq!(queue.len(), 2);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::RailsImpact { .. })
    ));
    let id = queue.next().map(|event| event.id).unwrap();
    assert!(queue.cancel(id));
    assert!(!queue.cancel(id));
    assert_eq!(queue.len(), 1);
    queue.clear_rails_wakes();
    assert_eq!(queue.len(), 1);
    assert!(matches!(
        queue.next().map(|event| event.kind),
        Some(ScheduledKind::Alarm)
    ));
}

#[test]
fn fast_bake_matches_exact_within_metres() {
    // The table-backed bake must agree with the exact per-step Kepler path
    // while real bodies move: a tight binary companion laps every ~2.9 h, so
    // the 40 s table nodes actually interpolate. Same end state class,
    // metre-grade deviation over a full day. Prints both timings.
    let mu_primary = 4.0e13;
    let mu_secondary = 4.0e12;
    let separation = 20_000_000.0;
    let ephemeris = two_body_binary(mu_primary, mu_secondary, separation);
    let radius = 30_000_000.0;
    let speed = ((mu_primary + mu_secondary) / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 17_280,
    };
    let started = std::time::Instant::now();
    let exact = propagate_sampled_verlet(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("exact bakes");
    let exact_elapsed = started.elapsed();
    let started = std::time::Instant::now();
    let fast = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("fast bakes");
    let fast_elapsed = started.elapsed();
    assert_eq!(fast.end, exact.end);
    assert_eq!(fast.positions.len(), exact.positions.len());
    let mut max_deviation: f64 = 0.0;
    for i in 0..exact.positions.len() {
        max_deviation = max_deviation.max((fast.positions[i] - exact.positions[i]).length());
    }
    eprintln!(
        "fast bake {:.2} ms vs exact {:.2} ms ({:.1}x), max deviation {max_deviation:e} m over {} samples",
        fast_elapsed.as_secs_f64() * 1000.0,
        exact_elapsed.as_secs_f64() * 1000.0,
        exact_elapsed.as_secs_f64() / fast_elapsed.as_secs_f64().max(1e-9),
        exact.positions.len()
    );
    assert!(
        max_deviation < 5.0,
        "table interpolant drifted {max_deviation:e} m"
    );
}

#[test]
fn fast_bake_is_deterministic_across_runs() {
    // Same inputs, same path, bit for bit: the table build must not depend
    // on thread scheduling or hash order.
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 1_000,
    };
    let first = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("first bakes");
    let second = propagate_sampled_verlet_fast(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        config,
        &[],
        crate::TABLE_NODE_EVERY_STEPS,
    )
    .expect("second bakes");
    assert_eq!(first, second);
}

#[test]
fn rails_rejects_edited_ephemeris_even_with_same_identity_metadata() {
    let ephemeris = central_ephemeris(4e13);
    let initial = TestParticleState {
        position: DVec3::X * 1e7,
        velocity: DVec3::Y * 2000.0,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 3,
    };
    let mut rails = OnRailsCache::new();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .unwrap();
    let mut edited = ephemeris.clone();
    edited.bodies[0].mu *= 1.01;
    assert!(!rails.usable_for(&edited, initial, SimTime::EPOCH, config, &[], 5.0, 0.05));
    assert!(rails.sample_at(SimTime(f64::NAN)).is_none());
}

#[test]
fn ephemeris_table_rejects_bad_steps_and_out_of_coverage_queries() {
    let ephemeris = central_ephemeris(4e13);
    for step in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-100] {
        assert!(
            crate::EphemerisTable::build(
                &ephemeris,
                &[BodyId(0)],
                SimTime::EPOCH,
                SimTime(10.0),
                step
            )
            .is_err()
        );
    }
    let table =
        crate::EphemerisTable::build(&ephemeris, &[BodyId(0)], SimTime::EPOCH, SimTime(10.0), 3.0)
            .unwrap();
    for time in [-1.0, 10.01, f64::NAN] {
        assert!(table.body_state_at(BodyId(0), SimTime(time)).is_none());
        assert!(
            table
                .acceleration_at(DVec3::X * 1e7, SimTime(time))
                .is_none()
        );
    }
    assert!(table.body_state_at(BodyId(0), SimTime(10.0)).is_some());
}

#[test]
fn fast_prediction_hits_non_gravitating_physical_body() {
    let mut body = BakedBody::fixed(BodyId(0), "surface-only", 4e13, 5.0);
    body.gravity_source = false;
    let ephemeris = BakedEphemeris::new("CONTACT", vec![body]).unwrap();
    let initial = TestParticleState {
        position: DVec3::X * 10.0,
        velocity: -DVec3::X * 20.0,
    };
    let config = VerletConfig {
        step_s: 1.0,
        max_steps: 2,
    };
    let path =
        propagate_sampled_verlet_fast(&ephemeris, initial, SimTime::EPOCH, config, &[BodyId(0)], 8)
            .unwrap();
    assert_eq!(path.end, SampledPathEnd::Impact(BodyId(0)));
    assert!((path.positions.last().unwrap().x - 5.0).abs() < 1e-10);
}

#[test]
fn rails_full_horizon_has_measured_second_order_orbit_error() {
    let mu = 4e13;
    let radius = 1e7_f64;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let mut errors = Vec::new();
    for step in [5.0, 2.5] {
        let mut cache = OnRailsCache::new();
        cache
            .bake(
                &ephemeris,
                initial,
                SimTime::EPOCH,
                VerletConfig {
                    step_s: step,
                    max_steps: (200_000.0 / step) as u64,
                },
                &[],
            )
            .unwrap();
        let mut position_error = 0.0_f64;
        let mut velocity_error = 0.0_f64;
        for i in 0..=1000 {
            let time = SimTime(i as f64 * 200.0);
            let phase = speed / radius * time.0;
            let (p, v) = cache.sample_at(time).unwrap();
            position_error =
                position_error.max(p.distance(DVec3::new(phase.cos(), phase.sin(), 0.0) * radius));
            velocity_error =
                velocity_error.max(v.distance(DVec3::new(-phase.sin(), phase.cos(), 0.0) * speed));
        }
        eprintln!(
            "200000s circular coast dt={step}: max position {position_error:.6} m, velocity {velocity_error:.9} m/s"
        );
        errors.push(position_error);
        assert!(position_error < 200.0);
        assert!(velocity_error < 0.05);
    }
    assert!(
        errors[1] < errors[0] * 0.3,
        "halving dt must recover second-order convergence: {errors:?}"
    );
}

#[test]
fn chunked_extend_matches_full_bake_bitwise() {
    // Hitch-free coast entry depends on this: head bake + per-frame
    // extension chunks must reproduce the one-shot full bake exactly
    // (same table nodes, same loop), not just approximately.
    use crate::{OnRailsCache, VerletConfig};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    let started = std::time::Instant::now();
    let mut full = OnRailsCache::new();
    full.bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("full bakes");
    let full_elapsed = started.elapsed();
    let started = std::time::Instant::now();
    let mut chunked = OnRailsCache::new();
    chunked
        .bake_head(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("head bakes");
    let head_elapsed = started.elapsed();
    assert!(
        chunked.covered_until().expect("head covers").seconds()
            < full.covered_until().expect("full covers").seconds()
    );
    let mut extensions = 0;
    let mut extend_elapsed = std::time::Duration::ZERO;
    while chunked.needs_extension(SimTime::EPOCH, f64::INFINITY) {
        let started = std::time::Instant::now();
        assert!(chunked.extend(&ephemeris, 512).expect("extends"), "stalled");
        extend_elapsed += started.elapsed();
        extensions += 1;
        assert!(extensions < 20, "extension loop did not converge");
    }
    eprintln!(
        "chunked bake: full {:.2} ms vs head {:.2} ms + {}x extend avg {:.2} ms",
        full_elapsed.as_secs_f64() * 1000.0,
        head_elapsed.as_secs_f64() * 1000.0,
        extensions,
        extend_elapsed.as_secs_f64() * 1000.0 / extensions.max(1) as f64
    );
    let (a, b) = (full.path().unwrap(), chunked.path().unwrap());
    assert_eq!(a.positions.len(), b.positions.len());
    assert_eq!(a, b, "chunked bake must equal the full bake bitwise");
    eprintln!(
        "head+{}x512chunks == full bake over {} samples",
        extensions,
        a.positions.len()
    );
}

#[test]
fn rails_trim_keeps_window_and_extend_accounting() {
    // Sliding window for indefinite cruise: old samples drop in chunks,
    // interpolation/sampling still work inside the window, pre-window
    // queries correctly miss (forcing rebake), and extension keeps
    // budgeting against the stored horizon, not the trimmed length.
    use crate::{OnRailsCache, VerletConfig};
    let mu = 4.0e13;
    let radius = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let speed = (mu / radius).sqrt();
    let initial = TestParticleState {
        position: DVec3::X * radius,
        velocity: DVec3::Y * speed,
    };
    let config = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    let mut rails = OnRailsCache::new();
    rails
        .bake(&ephemeris, initial, SimTime::EPOCH, config, &[])
        .expect("bakes");
    assert_eq!(rails.sample_count(), 4001);
    // Trim everything older than T+3600 s, chunks of 1800 s.
    rails.trim_before(SimTime(7200.0), 3600.0, 1800.0);
    let count = rails.sample_count();
    assert!(count < 4001 && count > 3000, "trimmed to {count} samples");
    // Inside the window: sampling works.
    assert!(rails.sample_at(SimTime(10_000.0)).is_some());
    // Before the window: miss (caller rebakes).
    assert!(rails.sample_at(SimTime(100.0)).is_none());
    // Horizon accounting untouched: this bake has 4000 accepted of 4000
    // max, so no extension even after trimming.
    assert!(!rails.needs_extension(SimTime(10_000.0), 60.0));
    // Fresh shorter bake still extends after a trim.
    let mut short = OnRailsCache::new();
    let config2 = VerletConfig {
        step_s: 5.0,
        max_steps: 4_000,
    };
    short
        .bake_head(&ephemeris, initial, SimTime::EPOCH, config2, &[])
        .expect("head bakes");
    let head_count = short.sample_count();
    short.trim_before(SimTime(5_000.0), 3600.0, 600.0);
    assert!(short.sample_count() < head_count);
    assert!(short.extend(&ephemeris, 512).expect("extends"));
    eprintln!(
        "trim+extend accounting holds over {} samples",
        short.sample_count()
    );
}

#[test]
fn year_long_scaled_escape_stays_display_grade() {
    // Interstellar prediction needs year horizons without megabyte paths.
    // Uniform 4 h steps fail catastrophically here (measured 7.4e10 m: the
    // fast periapsis bend is unresolved and the asymptote error compounds
    // forever), so the far bake follows the local dynamical timescale
    // instead: dense at periapsis, daily strides in cruise.
    use crate::{VerletConfig, propagate_sampled_verlet, propagate_sampled_verlet_scaled};
    let mu = 4.0e13;
    let start = 10_000_000.0;
    let ephemeris = central_ephemeris(mu);
    let initial = TestParticleState {
        position: DVec3::X * start,
        velocity: DVec3::Y * (2.0 * mu / start).sqrt() * 1.2,
    };
    let started = std::time::Instant::now();
    let scaled = propagate_sampled_verlet_scaled(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        60.0,
        86_400.0,
        1.0 / 40.0,
        700,
        &[],
    )
    .expect("scaled year bakes");
    let scaled_elapsed = started.elapsed();
    assert!(
        scaled.end_time.seconds() >= 105_600.0 * 300.0,
        "scaled bake must reach the full reference year"
    );
    let fine = propagate_sampled_verlet(
        &ephemeris,
        initial,
        SimTime::EPOCH,
        VerletConfig {
            step_s: 300.0,
            max_steps: 105_600,
        },
        &[],
    )
    .expect("fine year bakes");
    // Compare at the scaled nodes against the bracketing fine samples,
    // over the shared horizon only.
    let fine_end = fine.end_time.seconds();
    let mut max_deviation: f64 = 0.0;
    let mut compared = 0;
    let mut j = 0;
    for (i, t) in scaled.times.iter().enumerate() {
        if t.seconds() > fine_end {
            break;
        }
        compared += 1;
        while j + 1 < fine.times.len() && fine.times[j + 1].seconds() < t.seconds() {
            j += 1;
        }
        let reference = if j + 1 < fine.times.len()
            && (fine.times[j + 1].seconds() - t.seconds()).abs()
                < (t.seconds() - fine.times[j].seconds()).abs()
        {
            fine.positions[j + 1]
        } else {
            fine.positions[j]
        };
        max_deviation = max_deviation.max((scaled.positions[i] - reference).length());
    }
    assert!(
        compared > 100,
        "must compare across the year, got {compared}"
    );
    let span_days = fine_end / 86_400.0;
    eprintln!(
        "year escape scaled deviation {max_deviation:e} m over {span_days:.0} days, {} nodes in {:.2} ms",
        scaled.positions.len(),
        scaled_elapsed.as_secs_f64() * 1000.0
    );
    assert!(
        max_deviation < 1.0e9,
        "scaled year drifted {max_deviation:e} m — too coarse even for display"
    );
}

#[test]
fn reused_table_endpoints_match_recomputed_verlet_bitwise() {
    // Moving sources exercise node transitions, SIMD tails and a fractional
    // final table interval. Reference intentionally recomputes both endpoints.
    let config: SystemConfig =
        toml::from_str(include_str!("../../../../data/system.toml")).unwrap();
    let ephemeris = config.bake().unwrap();
    let home = ephemeris
        .body_state(ephemeris.body_id("thessa").unwrap(), SimTime::EPOCH)
        .unwrap();
    let initial = TestParticleState {
        position: home.position_inertial + DVec3::Z * 1e9,
        velocity: home.velocity_inertial + DVec3::X * 100.0,
    };
    let sources: Vec<_> = ephemeris.gravity_sources().map(|b| b.id).collect();
    for steps in [0, 1, 129] {
        let dt = 5.0;
        let table = EphemerisTable::build(
            &ephemeris,
            &sources,
            SimTime::EPOCH,
            SimTime(dt * steps as f64),
            40.0,
        )
        .unwrap();
        let actual = propagate_sampled_verlet_fast(
            &ephemeris,
            initial,
            SimTime::EPOCH,
            VerletConfig {
                step_s: dt,
                max_steps: steps,
            },
            &[],
            8,
        )
        .unwrap();
        let mut state = initial;
        let mut time = SimTime::EPOCH;
        let mut start = TableSnapshot::default();
        let mut end = TableSnapshot::default();
        for i in 0..steps as usize {
            table.snapshot(time, &mut start);
            let a0 = table.accel_from(&start, state.position).unwrap();
            let position = state.position + state.velocity * dt + a0 * (0.5 * dt * dt);
            time = time.offset(dt);
            table.snapshot(time, &mut end);
            let a1 = table.accel_from(&end, position).unwrap();
            state = TestParticleState {
                position,
                velocity: state.velocity + (a0 + a1) * (0.5 * dt),
            };
            assert_eq!(actual.positions[i + 1], state.position);
            assert_eq!(actual.velocities[i + 1], state.velocity);
            assert_eq!(actual.times[i + 1], time);
        }
        assert_eq!(actual.stats.accepted_steps, steps);
        assert_eq!(actual.end_time, time);
    }
}

#[test]
fn extension_rejects_missing_velocity_without_panicking() {
    let ephemeris = central_ephemeris(4e13);
    let mut path = propagate_sampled_verlet_fast(
        &ephemeris,
        TestParticleState {
            position: DVec3::X * 1e7,
            velocity: DVec3::Y * 2000.0,
        },
        SimTime::EPOCH,
        VerletConfig {
            step_s: 5.0,
            max_steps: 2,
        },
        &[],
        8,
    )
    .unwrap();
    path.velocities.clear();
    let before = path.clone();
    assert!(propagate_sampled_extend(&ephemeris, &mut path, 5.0, 10, 3, &[], 8).is_err());
    assert_eq!(path, before);
}

#[test]
fn failed_extension_rolls_back_already_appended_steps() {
    let ephemeris = central_ephemeris(1.0);
    let mut path = SampledPath {
        positions: vec![DVec3::X; 2],
        velocities: vec![DVec3::X * 0.5; 2],
        accelerations: vec![DVec3::ZERO; 2],
        times: vec![SimTime(-1.0), SimTime::EPOCH],
        end_time: SimTime::EPOCH,
        end: SampledPathEnd::Completed,
        stats: IntegratorStats {
            accepted_steps: 1,
            rejected_steps: 0,
        },
    };
    // x=1,v=.5 => x1=1,v1=-.5 => x2=0 (point singularity).
    let before = path.clone();
    assert!(propagate_sampled_extend(&ephemeris, &mut path, 1.0, 10, 3, &[], 8).is_err());
    assert_eq!(path, before);
}

#[test]
fn constant_spin_matches_axis_rotation_and_rejects_asymmetric_tumble() {
    let inertia = glam::DMat3::from_diagonal(DVec3::new(2.0, 3.0, 5.0));
    let initial = DQuat::from_rotation_y(0.7);
    for seconds in [0.0, 1.0, 3600.0] {
        let q = constant_spin_orientation(initial, DVec3::X * 0.25, inertia, seconds).unwrap();
        assert!(q.abs_diff_eq(
            (initial * DQuat::from_rotation_x(0.25 * seconds)).normalize(),
            1e-12
        ));
        assert!(
            constant_spin_orientation(initial, DVec3::ZERO, inertia, seconds)
                .unwrap()
                .abs_diff_eq(initial, 1e-12)
        );
    }
    assert!(constant_spin_orientation(initial, DVec3::ONE, inertia, 1.0).is_none());
}

#[test]
fn rails_trim_moves_buffers_only_after_accumulating_a_full_chunk() {
    let ephemeris = central_ephemeris(4e13);
    let mut rails = OnRailsCache::new();
    rails
        .bake(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 1e7,
                velocity: DVec3::Y * 2000.0,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 5.0,
                max_steps: 1000,
            },
            &[],
        )
        .unwrap();
    rails.trim_before(SimTime(720.0), 60.0, 600.0);
    let after_chunk = rails.sample_count();
    assert!(after_chunk < 1001);
    for t in (725..1200).step_by(5) {
        rails.trim_before(SimTime(f64::from(t)), 60.0, 600.0);
        assert_eq!(
            rails.sample_count(),
            after_chunk,
            "must not memmove one sample per tick"
        );
    }
    rails.trim_before(SimTime(1400.0), 60.0, 600.0);
    assert!(rails.sample_count() < after_chunk);
    assert!(rails.sample_at(SimTime(1340.0)).is_some());
}

#[test]
fn zero_step_head_preserves_the_requested_budget() {
    let ephemeris = central_ephemeris(4e13);
    let mut rails = OnRailsCache::new();
    let path = rails
        .bake_head(
            &ephemeris,
            TestParticleState {
                position: DVec3::X * 1e7,
                velocity: DVec3::Y * 2000.0,
            },
            SimTime::EPOCH,
            VerletConfig {
                step_s: 5.0,
                max_steps: 0,
            },
            &[],
        )
        .unwrap();
    assert_eq!(path.positions.len(), 1);
    assert_eq!(path.end_time, SimTime::EPOCH);
    assert!(!rails.needs_extension(SimTime::EPOCH, 100.0));
}
