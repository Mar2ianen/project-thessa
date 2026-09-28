use super::*;

/// Degree-2 harmonics (J2/C22): derivation checks, closed forms, frame-path
/// parity, and measured physical effects. The monopole loop is untouched for
/// zero-coefficient bodies (bitwise-identical fast path).
mod harmonics_tests {
    use super::*;
    use thessa_celestial::{
        BodyHarmonics, HARMONICS_TRUNCATION, ellipsoid_harmonics, harmonic_correction,
    };

    pub(super) const EARTH_MU: f64 = 3.986_004_418e14;
    pub(super) const EARTH_R: f64 = 6_378_000.0;
    pub(super) const EARTH_J2: f64 = 1.082_626_68e-3;

    pub(super) fn oblate_body(mu: f64, radius_m: f64, j2: f64) -> BakedBody {
        let mut body = BakedBody::fixed(BodyId(0), "oblate", mu, radius_m);
        body.j2 = j2;
        body
    }

    pub(super) fn triaxial_body() -> BakedBody {
        // Cinder-class shard: strong J2 + C22, static frame (no spin data),
        // pole on inertial +Z.
        let mut body = BakedBody::fixed(BodyId(0), "shard", 1.0e10, 56_000.0);
        body.j2 = 0.279139;
        body.c22 = 0.08165;
        body
    }

    pub(super) fn sectorial_body() -> BakedBody {
        // Pure C22 (no J2) for isolated sectorial closed forms.
        let mut body = BakedBody::fixed(BodyId(0), "sectorial", 1.0e10, 56_000.0);
        body.c22 = 0.08165;
        body
    }

    pub(super) fn single_body_field(body: BakedBody) -> (BakedEphemeris, GravityField<'static>) {
        // Ownership dance: the field borrows the ephemeris, so leak the box
        // for a 'static view (test-only).
        let ephemeris: &'static BakedEphemeris = Box::leak(Box::new(
            BakedEphemeris::new("HARM_TEST", vec![body]).unwrap(),
        ));
        let field = GravityField::from_ephemeris(ephemeris);
        // Reconstruct the owned ephemeris for callers that need it by cloning
        // back (cheap, test-only).
        (ephemeris.clone(), field)
    }

    #[test]
    fn ellipsoid_derivation_special_cases() {
        // Sphere: exactly zero.
        assert_eq!(ellipsoid_harmonics([2.0, 2.0, 2.0]), Some((0.0, 0.0)));
        // Oblate spheroid: J2 > 0, C22 exactly 0.
        let (j2, c22) = ellipsoid_harmonics([4.0, 4.0, 2.0]).unwrap();
        let expected_j2 = 0.6 / 4.0_f64.powf(2.0 / 3.0);
        assert!((j2 - expected_j2).abs() < 1e-15, "oblate J2, got {j2}");
        assert_eq!(c22, 0.0);
        // Order-free: unsorted input sorts internally.
        let (shuffled_j2, shuffled_c22) = ellipsoid_harmonics([2.0, 4.0, 4.0]).unwrap();
        assert_eq!((shuffled_j2, shuffled_c22), (j2, c22));
        // Cinder working values (homogeneous estimate from full lengths).
        let (cinder_j2, cinder_c22) = ellipsoid_harmonics([180.0, 110.0, 70.0]).unwrap();
        assert!(
            (cinder_j2 - 0.2791).abs() < 1e-3,
            "cinder J2, got {cinder_j2}"
        );
        assert!(
            (cinder_c22 - 0.0817).abs() < 1e-3,
            "cinder C22, got {cinder_c22}"
        );
        // Degenerate input is rejected, never NaN.
        assert_eq!(ellipsoid_harmonics([0.0, 1.0, 1.0]), None);
        assert_eq!(ellipsoid_harmonics([-1.0, 2.0, 2.0]), None);
        assert_eq!(ellipsoid_harmonics([f64::INFINITY, 2.0, 2.0]), None);
        assert_eq!(ellipsoid_harmonics([f64::NAN, 2.0, 2.0]), None);
    }

    #[test]
    fn earth_j2_equatorial_polar_closed_forms() {
        let (ephemeris, field) = single_body_field(oblate_body(EARTH_MU, EARTH_R, EARTH_J2));
        let monopole = EARTH_MU / (EARTH_R * EARTH_R);
        // Equator: extra inward pull (3/2)·J2, no transverse components.
        let equatorial = field
            .acceleration(DVec3::new(EARTH_R, 0.0, 0.0), SimTime(0.0))
            .unwrap();
        let expected_eq = -monopole * (1.0 + 1.5 * EARTH_J2);
        assert!((equatorial.x - expected_eq).abs() / monopole < 1e-12);
        assert_eq!(equatorial.y, 0.0);
        assert_eq!(equatorial.z, 0.0);
        // Pole: reduced pull 3·J2, purely axial.
        let polar = field
            .acceleration(DVec3::new(0.0, 0.0, EARTH_R), SimTime(0.0))
            .unwrap();
        let expected_polar = -monopole * (1.0 - 3.0 * EARTH_J2);
        assert!((polar.z - expected_polar).abs() / monopole < 1e-12);
        assert_eq!(polar.x, 0.0);
        assert_eq!(polar.y, 0.0);
        // Frame path is bitwise identical at both points.
        let mut frame = EphemerisFrame::new();
        for point in [DVec3::new(EARTH_R, 0.0, 0.0), DVec3::new(0.0, 0.0, EARTH_R)] {
            let states = frame.evaluate(&ephemeris, SimTime(0.0)).unwrap().to_vec();
            assert_eq!(
                field
                    .acceleration_from_states(point, &states, SimTime(0.0))
                    .unwrap(),
                field.acceleration(point, SimTime(0.0)).unwrap()
            );
        }
    }

    #[test]
    fn c22_long_axis_pulls_short_axis_pushes() {
        let (ephemeris, field) = single_body_field(sectorial_body());
        let r: f64 = 2.0 * 56_000.0;
        let monopole = 1.0e10 / (r * r);
        let c22_term = 9.0 * 1.0e10 * 0.08165 * 56_000.0_f64.powi(2) / r.powi(4);
        // Above the long axis: stronger pull (excess mass below).
        let long = field
            .acceleration(DVec3::new(r, 0.0, 0.0), SimTime(0.0))
            .unwrap();

        assert!((long.x + monopole + c22_term).abs() / monopole < 1e-12);
        assert_eq!(long.y, 0.0);
        assert_eq!(long.z, 0.0);
        // Above the short axis: weaker pull (mass deficit below).
        let short = field
            .acceleration(DVec3::new(0.0, r, 0.0), SimTime(0.0))
            .unwrap();
        assert!((short.y + monopole - c22_term).abs() / monopole < 1e-12);
        assert_eq!(short.x, 0.0);
        assert_eq!(short.z, 0.0);
        // Above the pole: C22 contributes nothing (x²−y² = 0).
        let polar = field
            .acceleration(DVec3::new(0.0, 0.0, r), SimTime(0.0))
            .unwrap();
        assert!((polar.z + monopole).abs() / monopole < 1e-12);
        let _ = ephemeris;
    }

    #[test]
    fn tilted_pole_keeps_equatorial_symmetry() {
        // 90° tilt: pole on inertial −Y. Points on the inertial X axis lie
        // in the body equatorial plane, so acceleration must be purely
        // radial — any transverse component means a frame bug.
        let mut body = oblate_body(EARTH_MU, EARTH_R, EARTH_J2);
        body.axial_tilt_rad = std::f64::consts::FRAC_PI_2;
        let (_, field) = single_body_field(body);
        let monopole = EARTH_MU / (EARTH_R * EARTH_R);
        let point = DVec3::new(EARTH_R, 0.0, 0.0);
        let accel = field.acceleration(point, SimTime(0.0)).unwrap();
        let expected = -monopole * (1.0 + 1.5 * EARTH_J2);
        assert!((accel.x - expected).abs() / monopole < 1e-12);
        assert!(accel.y.abs() / monopole < 1e-12, "spurious Y: {}", accel.y);
        assert!(accel.z.abs() / monopole < 1e-12, "spurious Z: {}", accel.z);
        // On the pole axis (inertial −Y): polar form along −Y.
        let polar = field
            .acceleration(DVec3::new(0.0, -EARTH_R, 0.0), SimTime(0.0))
            .unwrap();
        let expected_polar = monopole * (1.0 - 3.0 * EARTH_J2);
        assert!((polar.y - expected_polar).abs() / monopole < 1e-12);
        assert!(polar.x.abs() / monopole < 1e-12);
        assert!(polar.z.abs() / monopole < 1e-12);
    }

    #[test]
    fn correction_matches_potential_gradient() {
        // Independent cross-check: central differences of the full potential
        // (monopole + U2 + U22) against the analytic acceleration, with tilt,
        // spin phase and prime meridian all nonzero.
        let mut body = BakedBody::fixed(BodyId(0), "lumpy", 2.5e12, 1.0e6);
        body.j2 = 0.05;
        body.c22 = 0.02;
        body.axial_tilt_rad = 0.3;
        body.rotation_period_s = Some(36_000.0);
        body.prime_meridian_rad = 0.3;
        let (ephemeris, field) = single_body_field(body);
        let time = SimTime(12_345.0);
        let radius = 1.0e6;
        let points = [
            DVec3::new(2.0, 0.5, 0.3),
            DVec3::new(-1.0, 2.5, -0.7),
            DVec3::new(0.4, -0.4, 3.0),
            DVec3::new(1.0, 1.0, 1.0),
        ];
        for direction in points {
            let point = direction.normalize() * 2.5 * radius;
            let analytic = field.acceleration(point, time).unwrap();
            let h = 2.5 * radius * 1e-5;
            let mut numeric = DVec3::ZERO;
            for axis in 0..3 {
                let mut plus = point;
                let mut minus = point;
                plus[axis] += h;
                minus[axis] -= h;
                let up = field.potential(plus, time).unwrap();
                let dn = field.potential(minus, time).unwrap();
                // a = −∇U.
                numeric[axis] = -(up - dn) / (2.0 * h);
            }
            let scale = analytic.length().max(1e-30);
            assert!(
                (analytic - numeric).length() / scale < 1e-7,
                "gradient mismatch at {point:?}: analytic={analytic:?} numeric={numeric:?}"
            );
        }
        let _ = ephemeris;
    }

    #[test]
    fn truncation_gate_and_interior_stay_monopole() {
        let (_, field) = single_body_field(triaxial_body());
        let mu = 1.0e10;
        let radius = 56_000.0;
        // Far field: gate skips, correction is None.
        let params = BodyHarmonics::new(&triaxial_body());
        // A part-per-trillion gate: at a million reference radii even
        // Cinder-class coefficients sit far below it.
        let far = DVec3::new(1.0e6 * radius, 0.0, 0.0);
        assert!((params.ref_radius_sq / far.length_squared()) * 0.279139 < HARMONICS_TRUNCATION);
        assert!(harmonic_correction(mu, &params, far, far.length_squared(), 0.0, None).is_none());
        let accel = field.acceleration(far, SimTime(0.0)).unwrap();
        let expected = -mu / far.length_squared();
        assert!((accel.x - expected).abs() / expected.abs() < 1e-15);
        // Interior (r < R): external expansion invalid, monopole-only.
        let deep = DVec3::new(0.5 * radius, 0.0, 0.0);
        assert!(harmonic_correction(mu, &params, deep, deep.length_squared(), 0.0, None).is_none());
        // Zero coefficients: no lane at all.
        let plain = BodyHarmonics::new(&BakedBody::fixed(BodyId(0), "plain", mu, radius));
        assert!(!plain.active);
        assert!(harmonic_correction(mu, &plain, far, far.length_squared(), 0.0, None).is_none());
    }

    #[test]
    fn frame_paths_match_bitwise_with_harmonics() {
        // Spinning triaxial body: every C22 orientation input (pole, spin
        // phase, prime meridian) runs through both accumulation paths.
        let mut body = triaxial_body();
        body.axial_tilt_rad = 0.2;
        body.rotation_period_s = Some(72_000.0);
        body.prime_meridian_rad = 0.1;
        let (ephemeris, field) = single_body_field(body);
        let mut frame = EphemerisFrame::new();
        let radius = 56_000.0;
        let points = [
            DVec3::new(1.5 * radius, 0.2 * radius, 0.1 * radius),
            DVec3::new(-2.0 * radius, 1.0 * radius, -0.5 * radius),
            DVec3::new(0.3 * radius, -0.3 * radius, 3.0 * radius),
        ];
        for seconds in [0.0, 3_600.0, 100_000.0] {
            let time = SimTime(seconds);
            let states = frame.evaluate(&ephemeris, time).unwrap().to_vec();
            for point in points {
                assert_eq!(
                    field
                        .acceleration_from_states(point, &states, time)
                        .unwrap(),
                    field.acceleration(point, time).unwrap(),
                    "lane mismatch at {point:?} t={seconds}"
                );
            }
        }
    }

    #[test]
    fn locked_frame_points_long_axis_at_host() {
        // Tidally locked triaxial moon: orientation rebuilds from the host
        // state each evaluation, and both lanes agree bitwise.
        let host_mu = 1.0e14;
        let orbit = KeplerOrbit::new(host_mu, 10_000_000.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            .expect("valid test orbit");
        let mut moon = BakedBody::orbital(BodyId(1), "locked", 1.0e10, 56_000.0, BodyId(0), orbit);
        moon.j2 = 0.05;
        moon.c22 = 0.02;
        moon.tidal_lock = true;
        let host = BakedBody::fixed(BodyId(0), "host", host_mu, 1_000_000.0);
        let ephemeris = BakedEphemeris::new("LOCK_TEST", vec![host.clone(), moon])
            .expect("valid test ephemeris");
        let field = GravityField::from_ephemeris(&ephemeris);
        let time = SimTime(5_000.0);
        let moon_pos = ephemeris
            .body_state(BodyId(1), time)
            .unwrap()
            .position_inertial;
        let host_pos = ephemeris
            .body_state(BodyId(0), time)
            .unwrap()
            .position_inertial;
        // Probe along the host direction: long-axis physics applies, and the
        // two lanes agree exactly (host read from slice vs fresh lookup).
        let probe = moon_pos + (host_pos - moon_pos).normalize() * 100_000.0;
        let mut frame = EphemerisFrame::new();
        let states = frame.evaluate(&ephemeris, time).unwrap().to_vec();
        let direct = field.acceleration(probe, time).unwrap();
        let framed = field
            .acceleration_from_states(probe, &states, time)
            .unwrap();
        assert_eq!(direct, framed);
        // Off-axis probe differs from the monopole twin (C22 is active).
        let mut plain_moon =
            BakedBody::orbital(BodyId(1), "plain", 1.0e10, 56_000.0, BodyId(0), orbit);
        plain_moon.tidal_lock = true;
        let plain_ephemeris =
            BakedEphemeris::new("LOCK_PLAIN", vec![host.clone(), plain_moon]).unwrap();
        let plain_field = GravityField::from_ephemeris(&plain_ephemeris);
        let delta = (direct - plain_field.acceleration(probe, time).unwrap()).length();
        assert!(delta > 0.0, "locked C22 must perturb the probe");
    }
}

mod harmonics_effect_tests {
    use super::harmonics_tests::{EARTH_J2, EARTH_MU, EARTH_R};
    use super::*;

    /// J2 regresses the ascending node at the textbook rate
    /// dΩ/dt = −(3/2)·J2·(R/p)²·n·cos(i). Circular inclined LEO integrated
    /// with Verlet; the node vector comes straight from sampled (r, v).
    #[test]
    fn j2_regresses_node_at_analytic_rate() {
        // The propagation ephemeris carries the J2 central body itself:
        // a monopole twin would show zero regression.
        let mut central = BakedBody::fixed(BodyId(0), "earth", EARTH_MU, EARTH_R);
        central.j2 = EARTH_J2;
        let ephemeris = BakedEphemeris::new("LEO_J2", vec![central]).unwrap();
        let inclination = 30.0_f64.to_radians();
        let radius = 2.0 * EARTH_R;
        let speed = (EARTH_MU / radius).sqrt();
        // Start on the ascending node: position on +X, velocity tipped by i.
        let initial = TestParticleState {
            position: DVec3::new(radius, 0.0, 0.0),
            velocity: DVec3::new(0.0, speed * inclination.cos(), speed * inclination.sin()),
        };
        let period = std::f64::consts::TAU * (radius.powi(3) / EARTH_MU).sqrt();
        let orbits = 6;
        let steps_per_orbit = 400;
        let path = propagate_sampled_verlet(
            &ephemeris,
            initial,
            SimTime(0.0),
            VerletConfig {
                step_s: period / steps_per_orbit as f64,
                max_steps: (orbits * steps_per_orbit) as u64,
            },
            &[],
        )
        .expect("verlet LEO");
        // Node longitude per orbit from the orbit normal.
        let mut nodes = Vec::new();
        let per_orbit = steps_per_orbit;
        for orbit in 0..orbits {
            let mut normal = DVec3::ZERO;
            let mut count = 0u32;
            for step in 0..per_orbit {
                let index = 1 + orbit * per_orbit + step;
                if index >= path.positions.len() {
                    break;
                }
                let h = path.positions[index].cross(path.velocities[index]);
                if h.length_squared() > 0.0 {
                    normal += h.normalize();
                    count += 1;
                }
            }
            if count == 0 {
                continue;
            }
            normal /= count as f64;
            let node = DVec3::Z.cross(normal);
            if node.length_squared() > 1e-24 {
                let node = node.normalize();
                nodes.push((orbit as f64 * period, node.y.atan2(node.x)));
            }
        }
        assert!(nodes.len() >= 4, "need node samples, got {}", nodes.len());
        // Unwrapped linear fit of Ω(t).
        let (mut slope_num, mut slope_den) = (0.0, 0.0);
        let mut prev = nodes[0].1;
        let mut unwrapped = vec![prev];
        for &(_time, angle) in &nodes[1..] {
            let mut delta = angle - prev;
            while delta > std::f64::consts::PI {
                delta -= std::f64::consts::TAU;
            }
            while delta < -std::f64::consts::PI {
                delta += std::f64::consts::TAU;
            }
            prev += delta;
            unwrapped.push(prev);
        }
        let mean_t = nodes.iter().map(|n| n.0).sum::<f64>() / nodes.len() as f64;
        let mean_o = unwrapped.iter().sum::<f64>() / unwrapped.len() as f64;
        for (index, &(time, _)) in nodes.iter().enumerate() {
            slope_num += (time - mean_t) * (unwrapped[index] - mean_o);
            slope_den += (time - mean_t) * (time - mean_t);
        }
        let measured = slope_num / slope_den;
        let mean_motion = std::f64::consts::TAU / period;
        let analytic =
            -1.5 * EARTH_J2 * (EARTH_R / radius).powi(2) * mean_motion * inclination.cos();
        eprintln!("NODE measured dO/dt={measured:e} analytic={analytic:e}");
        assert!(measured < 0.0, "node must regress, got {measured:e}");
        assert!(
            (measured / analytic - 1.0).abs() < 0.15,
            "nodal rate within 15% of analytic: measured={measured:e} analytic={analytic:e}"
        );
    }

    /// Cinder-class triaxiality vs its monopole twin on a 2R circular orbit:
    /// the divergence band proves the effect is real and bounded. Both
    /// propagations are deterministic; the band has 2x headroom.
    #[test]
    fn cinder_triaxiality_divergence_envelope() {
        let mu: f64 = 1.0e10;
        let radius: f64 = 56_000.0;
        let orbit_radius = 2.0 * radius;
        let speed = (mu / orbit_radius).sqrt();
        let initial = TestParticleState {
            position: DVec3::new(orbit_radius, 0.0, 0.0),
            velocity: DVec3::new(0.0, speed, 0.0),
        };
        let period = std::f64::consts::TAU * (orbit_radius.powi(3) / mu).sqrt();
        let config = VerletConfig {
            step_s: period / 400.0,
            max_steps: 1_200,
        };
        let run = |j2: f64, c22: f64| {
            let mut body = BakedBody::fixed(BodyId(0), "cinder", mu, radius);
            body.j2 = j2;
            body.c22 = c22;
            let ephemeris = BakedEphemeris::new("DIVERGE", vec![body]).unwrap();
            propagate_sampled_verlet(&ephemeris, initial, SimTime(0.0), config, &[]).unwrap()
        };
        let full = run(0.279139, 0.08165);
        let mono = run(0.0, 0.0);
        let end = full.positions.len().min(mono.positions.len()) - 1;
        let divergence = (full.positions[end] - mono.positions[end]).length();
        eprintln!("CINDER 3-orbit divergence = {divergence:.1} m");
        assert!(
            divergence > 5_000.0,
            "triaxiality must move the endpoint kilometers, got {divergence:.1} m"
        );
        assert!(
            divergence < 400_000.0,
            "divergence stays bounded on 3 orbits, got {divergence:.1} m"
        );
    }
}
