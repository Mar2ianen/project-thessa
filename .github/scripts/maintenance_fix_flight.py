from pathlib import Path


def replace_exact(path: str, old: str, new: str, count: int | None = None) -> None:
    p = Path(path)
    text = p.read_text()
    found = text.count(old)
    if count is not None and found != count:
        raise SystemExit(f"{path}: expected {count} occurrences, found {found}")
    if found == 0:
        raise SystemExit(f"{path}: replacement pattern not found")
    p.write_text(text.replace(old, new))


# Pilot: sampled world inputs, frame-coherent atmosphere, coherent trace,
# correct end-frame body velocity and osculating-orbit velocity.
p = Path("apps/client/src/pilot.rs")
text = p.read_text()
old_import = '''use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroPanel, AtmosphereConfig, BakedEphemeris, BodyId, FlightForces,
    FlightStepInput, GravityField, PanelAeroModel, RigidBodyProperties, RigidBodyState,
    VehicleDefinition, integrate_rigid_body_duration,
};'''
new_import = '''use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroPanel, AtmosphereConfig, BakedEphemeris, BodyId, FlightError,
    FlightForces, FlightStepInput, GravityField, PanelAeroModel, RigidBodyProperties,
    RigidBodyState, VehicleDefinition, integrate_rigid_body_duration_sampled,
};'''
if text.count(old_import) != 1:
    raise SystemExit("pilot.rs: import block drifted")
text = text.replace(old_import, new_import)
if text.count("AtmosphereConfig::new(288.15, 108_000.0, 287.05287, 1.4, gravity)") != 1:
    raise SystemExit("pilot.rs: Thessa pressure pattern drifted")
text = text.replace(
    "AtmosphereConfig::new(288.15, 108_000.0, 287.05287, 1.4, gravity)",
    "AtmosphereConfig::new(288.15, 120_000.0, 287.05287, 1.4, gravity)",
)

start = text.index("fn simulate_pilot_flight(")
end = text.index("#[allow(clippy::too_many_arguments)]\nfn pilot_input(", start)
replacement = r'''#[derive(Debug, Clone, Copy)]
struct LocalAirKinematics {
    position_body_m: DVec3,
    air_velocity_body_mps: DVec3,
    surface_velocity_inertial_mps: DVec3,
}

fn local_air_kinematics(
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    relative_position_inertial_m: DVec3,
    relative_velocity_inertial_mps: DVec3,
) -> Option<LocalAirKinematics> {
    if !relative_position_inertial_m.is_finite() || !relative_velocity_inertial_mps.is_finite() {
        return None;
    }
    let orientation_inverse = state.orientation_body_to_inertial.inverse();
    let position_body_m = orientation_inverse * relative_position_inertial_m;
    let rotating_air_velocity_body_mps = atmosphere
        .rotating_air_velocity_body_mps(position_body_m, state.orientation_body_to_inertial)
        .ok()?;
    let relative_velocity_body_mps = orientation_inverse * relative_velocity_inertial_mps;
    let air_velocity_body_mps = relative_velocity_body_mps - rotating_air_velocity_body_mps;
    let surface_velocity_inertial_mps = relative_velocity_inertial_mps
        - state.orientation_body_to_inertial * rotating_air_velocity_body_mps;
    Some(LocalAirKinematics {
        position_body_m,
        air_velocity_body_mps,
        surface_velocity_inertial_mps,
    })
}

fn simulate_pilot_flight(
    time: Res<Time>,
    clock: Res<SimulationClock>,
    ephemeris: Res<RuntimeEphemeris>,
    state: Res<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
) {
    if state.view_mode != ClientViewMode::Pilot || clock.paused {
        return;
    }

    let frame_dt = f64::from(time.delta_secs().clamp(0.0, 0.05));
    if frame_dt <= 0.0 {
        return;
    }
    let Ok(body) = ephemeris.ephemeris.body(runtime.reference_body) else {
        return;
    };

    let pitch = runtime.control_input.x;
    let yaw = runtime.control_input.y;
    let roll = runtime.control_input.z;
    runtime.command_controls(pitch, yaw, roll);

    let reference_body = runtime.reference_body;
    let flight_start_time_s = runtime.flight_time_s;
    let atmosphere = runtime.atmosphere;
    let initial_state = runtime.state;
    let properties = runtime.vehicle.mass_properties;
    let sas_target_orientation = runtime.sas_target_orientation;
    let sas_active = runtime.sas_enabled
        && matches!(
            state.control_mode,
            ControlMode::MouseAim | ControlMode::Navball
        );
    let control_input = runtime.control_input;
    let throttle = runtime.throttle;
    let engine_active = runtime.engine_active;
    let sas_enabled = runtime.sas_enabled;
    let rcs_enabled = runtime.rcs_enabled;
    let thrust_n = runtime.thrust_n();
    let gravity_field = GravityField::from_ephemeris(&ephemeris.ephemeris);
    let mut last_sample: Option<(
        f64,
        f64,
        DVec3,
        DVec3,
        DVec3,
        RigidBodyState,
        DVec3,
    )> = None;

    let integration = integrate_rigid_body_duration_sampled(
        &runtime.aero_model,
        &runtime.vehicle.aero_geometry,
        atmosphere,
        initial_state,
        properties,
        frame_dt,
        0.02,
        |sample_state, elapsed_s| {
            let sample_time_s = flight_start_time_s + elapsed_s;
            let sim_time = SimTime(sample_time_s);
            let body_state = ephemeris
                .ephemeris
                .body_state(reference_body, sim_time)
                .map_err(|error| {
                    FlightError::InvalidInput(format!(
                        "reference body state unavailable during flight substep: {error}"
                    ))
                })?;
            let relative_position =
                sample_state.position_inertial_m - body_state.position_inertial;
            let altitude_m = relative_position.length() - body.radius_m;
            let radial_up = relative_position.try_normalize().ok_or_else(|| {
                FlightError::InvalidInput("vehicle reached the reference-body origin".into())
            })?;
            let relative_velocity_inertial =
                sample_state.velocity_inertial_mps - body_state.velocity_inertial;
            let air = local_air_kinematics(
                atmosphere,
                sample_state,
                relative_position,
                relative_velocity_inertial,
            )
            .ok_or_else(|| {
                FlightError::InvalidInput("local atmosphere kinematics are invalid".into())
            })?;
            let gravity = gravity_field
                .acceleration(sample_state.position_inertial_m, sim_time)
                .map_err(|error| {
                    FlightError::InvalidInput(format!(
                        "gravity unavailable during flight substep: {error}"
                    ))
                })?;

            let sas_moment = if sas_active {
                pilot_sas_moment(sample_state, sas_target_orientation, radial_up)
            } else {
                DVec3::ZERO
            };
            let trim_moment = if sas_active && control_input.length_squared() < 1.0e-8 {
                pilot_aero_trim_moment(air.air_velocity_body_mps)
            } else {
                DVec3::ZERO
            };
            let relative_radial_speed = relative_velocity_inertial.dot(radial_up);
            let contact_moment = if altitude_m <= PILOT_SURFACE_CLEARANCE_M + 0.5
                && relative_radial_speed <= 0.0
            {
                -sample_state.angular_velocity_body_rps
                    * DVec3::new(400_000.0, 700_000.0, 400_000.0)
            } else {
                DVec3::ZERO
            };
            let rate_moment = pilot_rate_moment(
                control_input.x,
                control_input.y,
                control_input.z,
                rcs_enabled,
            );
            let orientation_inverse = sample_state.orientation_body_to_inertial.inverse();
            let input = FlightStepInput {
                altitude_m: altitude_m.max(0.0),
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: air.position_body_m,
                // The core adds frame-correct rigid atmosphere rotation. This term
                // is only the reference body's barycentric translation.
                wind_velocity_body_mps: orientation_inverse * body_state.velocity_inertial,
                extra_force_body_n: DVec3::X * thrust_n,
                extra_moment_body_nm: sas_moment
                    + trim_moment
                    + contact_moment
                    + rate_moment,
            };
            last_sample = Some((
                sample_time_s,
                altitude_m,
                relative_velocity_inertial,
                radial_up,
                air.air_velocity_body_mps,
                sample_state,
                gravity,
            ));
            Ok(input)
        },
    );

    let Ok((next_state, forces)) = integration else {
        runtime.engine_active = false;
        runtime.control_input = DVec3::ZERO;
        return;
    };
    let next_sim_time = SimTime(runtime.flight_time_s + frame_dt);
    let Ok(next_body_state) = ephemeris
        .ephemeris
        .body_state(runtime.reference_body, next_sim_time)
    else {
        return;
    };
    let next_relative = next_state.position_inertial_m - next_body_state.position_inertial;
    let next_altitude = next_relative.length() - body.radius_m;
    let next_relative_speed =
        (next_state.velocity_inertial_mps - next_body_state.velocity_inertial).length();
    if !next_altitude.is_finite()
        || next_altitude.abs() > MAX_PILOT_ALTITUDE_M
        || !next_relative_speed.is_finite()
        || next_relative_speed > MAX_PILOT_RELATIVE_SPEED_MPS
        || !next_state.angular_velocity_body_rps.is_finite()
        || next_state.angular_velocity_body_rps.length() > MAX_PILOT_ANGULAR_RATE_RPS
    {
        runtime.engine_active = false;
        runtime.control_input = DVec3::ZERO;
        return;
    }

    if let Some((
        trace_time_s,
        trace_altitude_m,
        trace_relative_velocity,
        trace_radial_up,
        trace_air_velocity,
        trace_state,
        trace_gravity,
    )) = last_sample
    {
        runtime.last_gravity_acceleration_inertial_mps2 = trace_gravity;
        if let Some(trace) = runtime.trace.as_mut() {
            trace.record(
                trace_time_s,
                trace_altitude_m,
                trace_relative_velocity,
                trace_radial_up,
                trace_air_velocity,
                trace_state,
                control_input,
                throttle,
                engine_active,
                sas_enabled,
                rcs_enabled,
                &forces,
            );
        }
    }
    runtime.state = next_state;
    runtime.last_forces = Some(forces);
    runtime.flight_time_s += frame_dt;

    let next_radius = next_relative.length();
    if next_radius < body.radius_m + PILOT_SURFACE_CLEARANCE_M {
        let radial_fallback = next_relative.try_normalize().unwrap_or(DVec3::Z);
        let contact_position = radial_fallback * (body.radius_m + PILOT_SURFACE_CLEARANCE_M);
        runtime.state.position_inertial_m = next_body_state.position_inertial + contact_position;
        let next_radial_up = contact_position.normalize_or_zero();
        let next_relative_velocity =
            runtime.state.velocity_inertial_mps - next_body_state.velocity_inertial;
        let inward_speed = next_relative_velocity.dot(-next_radial_up);
        if inward_speed > 0.0 {
            runtime.state.velocity_inertial_mps += next_radial_up * inward_speed;
        }
    }

    runtime.render_position = pilot_render_offset(
        (runtime.state.position_inertial_m - next_body_state.position_inertial)
            - runtime.initial_relative_position_m,
    );
    runtime.render_orientation = render_orientation(runtime.state.orientation_body_to_inertial);
}

'''
text = text[:start] + replacement + text[end:]

old_hud = '''    let orientation_inverse = flight.state.orientation_body_to_inertial.inverse();
    let position_body = orientation_inverse * relative_position;
    let velocity_relative_inertial =
        flight.state.velocity_inertial_mps - body_state.velocity_inertial;
    let atmosphere_rotation_body = flight.atmosphere.body_rotation_rad_s.cross(position_body);
    let atmosphere_rotation_inertial =
        flight.state.orientation_body_to_inertial * atmosphere_rotation_body;
    let velocity_surface = velocity_relative_inertial - atmosphere_rotation_inertial;
    let velocity_body = orientation_inverse * velocity_relative_inertial;
    let forces = flight.last_forces.as_ref();
    let environment = forces.map(|forces| forces.environment);
    // The solver's environment contains body translation plus body rotation;
    // use the same explicitly separated terms for the HUD. This prevents the
    // moon's orbital velocity from appearing as a 40 km/s airspeed sample.
    let air_velocity = velocity_body - atmosphere_rotation_body;'''
new_hud = '''    let velocity_relative_inertial =
        flight.state.velocity_inertial_mps - body_state.velocity_inertial;
    let Some(air) = local_air_kinematics(
        flight.atmosphere,
        flight.state,
        relative_position,
        velocity_relative_inertial,
    ) else {
        return FlightUiState::default();
    };
    let velocity_surface = air.surface_velocity_inertial_mps;
    let forces = flight.last_forces.as_ref();
    let environment = forces.map(|forces| forces.environment);
    // Use the same frame transform as the authoritative aero boundary.
    let air_velocity = air.air_velocity_body_mps;'''
if text.count(old_hud) != 1:
    raise SystemExit("pilot.rs: live HUD air-kinematics block drifted")
text = text.replace(old_hud, new_hud)

old_orbit = '''    let (apoapsis_altitude_m, periapsis_altitude_m) =
        estimate_orbit_altitudes(relative_position, velocity_surface, body.mu, body.radius_m);'''
new_orbit = '''    let (apoapsis_altitude_m, periapsis_altitude_m) = estimate_orbit_altitudes(
        relative_position,
        velocity_relative_inertial,
        body.mu,
        body.radius_m,
    );'''
if text.count(old_orbit) != 1:
    raise SystemExit("pilot.rs: orbit estimate block drifted")
text = text.replace(old_orbit, new_orbit)

# The two standalone pilot regression tests calculate local air velocity too.
old_test_air = '''        let air_velocity_body = orientation_inverse * relative_velocity_inertial
            - flight.atmosphere.body_rotation_rad_s.cross(position_body);'''
new_test_air = '''        let rotating_air_velocity_body = flight
            .atmosphere
            .rotating_air_velocity_body_mps(position_body, flight.state.orientation_body_to_inertial)
            .expect("rotating atmosphere evaluates");
        let air_velocity_body =
            orientation_inverse * relative_velocity_inertial - rotating_air_velocity_body;'''
found_test_air = text.count(old_test_air)
if found_test_air == 0:
    raise SystemExit("pilot.rs: test atmosphere-rotation pattern not found")
text = text.replace(old_test_air, new_test_air)

old_bad_position = '''                    position_body_m: relative,
                    wind_velocity_body_mps: flight.state.orientation_body_to_inertial.inverse()
                        * body_state.velocity_inertial,'''
new_good_position = '''                    position_body_m: flight.state.orientation_body_to_inertial.inverse()
                        * relative,
                    wind_velocity_body_mps: flight.state.orientation_body_to_inertial.inverse()
                        * body_state.velocity_inertial,'''
if text.count(old_bad_position) != 1:
    raise SystemExit("pilot.rs: long-run position frame pattern drifted")
text = text.replace(old_bad_position, new_good_position)
if ".body_rotation_rad_s.cross(position_body)" in text:
    raise SystemExit("pilot.rs: stale frame-mixed atmosphere cross remains")
p.write_text(text)

# Aero damping: project rates into panel-local axes.
replace_exact(
    "crates/sim-core/src/aero.rs",
    '''            let reduced_rates = DVec3::new(
                state.angular_velocity_body_rps.x * panel.span_m / (2.0 * local_speed),
                state.angular_velocity_body_rps.y * panel.chord_m / (2.0 * local_speed),
                state.angular_velocity_body_rps.z * panel.span_m / (2.0 * local_speed),
            );''',
    '''            let reduced_rates = DVec3::new(
                state.angular_velocity_body_rps.dot(chord_axis) * panel.span_m
                    / (2.0 * local_speed),
                state.angular_velocity_body_rps.dot(side_axis) * panel.chord_m
                    / (2.0 * local_speed),
                state.angular_velocity_body_rps.dot(lift_axis) * panel.span_m
                    / (2.0 * local_speed),
            );''',
    1,
)

# Regressions for local-axis damping and per-substep input resampling.
p = Path("crates/sim-core/src/tests.rs")
text = p.read_text()
anchor = '''#[test]
fn aero_signed_stabilizer_lift_produces_restoring_pitch_moment() {'''
if text.count(anchor) != 1:
    raise SystemExit("tests.rs: aero insertion anchor drifted")
text = text.replace(
    anchor,
    r'''#[test]
fn aero_dynamic_damping_uses_panel_local_axes() {
    let panel = AeroPanel::new(DVec3::ZERO, DVec3::X, DVec3::Y, 10.0, 2.0)
        .expect("valid vertical panel");
    let environment = AeroEnvironment::standard_sea_level();
    let angular_velocity = DVec3::new(0.0, 0.0, -0.2);
    let case = AeroCase::new(
        AeroState::new(DVec3::new(100.0, 0.0, 0.0), angular_velocity),
        environment,
        AeroGeometry::new(vec![panel]).expect("valid geometry"),
    )
    .expect("valid case");
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        side_force_slope_per_rad: 0.0,
        pitch_damping_coefficient: -1.0,
        ..AeroConfig::default()
    })
    .expect("valid damping model");
    let result = model.evaluate(&case).expect("damping result");
    assert!(result.moment_body_nm.length() > 0.0);
    assert!(result.moment_body_nm.dot(angular_velocity) < 0.0);
}

''' + anchor,
)

anchor = '''#[test]
fn vehicle_definition_round_trips_with_controls() {'''
if text.count(anchor) != 1:
    raise SystemExit("tests.rs: sampled integrator insertion anchor drifted")
text = text.replace(
    anchor,
    r'''#[test]
fn rigid_body_duration_resamples_input_each_substep() {
    let model = PanelAeroModel::new(AeroConfig {
        base_drag_coefficient: 0.0,
        induced_drag_factor: 0.0,
        wave_drag_coefficient: 0.0,
        ..AeroConfig::default()
    })
    .expect("valid model");
    let geometry = AeroGeometry::new(vec![
        AeroPanel::flat_plate(DVec3::ZERO, 1.0, 1.0).expect("valid panel"),
    ])
    .expect("valid geometry");
    let atmosphere = AtmosphereConfig::default();
    let state = RigidBodyState::new(
        DVec3::ZERO,
        DVec3::new(100.0, 0.0, 0.0),
        DQuat::IDENTITY,
        DVec3::ZERO,
    )
    .expect("valid state");
    let properties = RigidBodyProperties::new(
        100.0,
        DMat3::from_diagonal(DVec3::splat(10.0)),
    )
    .expect("valid properties");
    let mut sample_times = Vec::new();
    integrate_rigid_body_duration_sampled(
        &model,
        &geometry,
        atmosphere,
        state,
        properties,
        0.1,
        0.02,
        |_, elapsed_s| {
            sample_times.push(elapsed_s);
            Ok(FlightStepInput::new(elapsed_s * 1_000.0, DVec3::ZERO))
        },
    )
    .expect("sampled duration integrates");
    assert_eq!(sample_times.len(), 5);
    for (index, sample) in sample_times.into_iter().enumerate() {
        assert!((sample - index as f64 * 0.02).abs() < 1.0e-12);
    }
}

''' + anchor,
)
p.write_text(text)

# f32 map math is not bit-identical across architectures.
replace_exact(
    "apps/client/src/navigation.rs",
    '''        assert_eq!(at_epoch + pan - pan, at_epoch);
        assert_eq!(later + pan - pan, later);''',
    '''        assert!(((at_epoch + pan - pan) - at_epoch).length() < 1.0e-5);
        assert!(((later + pan - pan) - later).length() < 1.0e-5);''',
    1,
)

# Cross-platform CI: portability check everywhere, expensive full Bevy tests once.
replace_exact(
    ".github/workflows/ci.yml",
    '''      - name: Run workspace tests
        run: cargo test --workspace''',
    '''      - name: Run simulation kernel tests
        run: cargo test -p thessa-sim-core

      - name: Run full workspace tests
        if: runner.os == 'Linux'
        run: cargo test --workspace''',
    1,
)

# Canonical design docs match the new Thessa baseline.
p = Path("docs/01_CELESTIAL_SYSTEM.md")
text = p.read_text()
replacements = {
    "| **Thessa** | 632 354 km | 80 h | 0.003 | 0.130 M⊕ | 3200 km | 0.52 g | 5.69 km/s | tidal lock |":
        "| **Thessa** | 632 354 km | 80 h | 0.003 | 0.126 M⊕ | 3200 km | 0.50 g | 5.60 km/s | tidal lock |",
    "- mass: 0.13 M⊕;": "- mass: ~0.126 M⊕;",
    "- gravity: ~0.52 g;": "- gravity: 0.500 g;",
    "- surface circular velocity: ~4.02 km/s;": "- surface circular velocity: ~3.96 km/s;",
    "- escape: ~5.69 km/s;": "- escape: ~5.60 km/s;",
    "- atmosphere: target `1.08 bar`, примерно N₂ 76%, O₂ 21%, Ar/CO₂/прочее 3%; composition provisional;":
        "- atmosphere: target `1.20 bar`, примерно N₂ 76%, O₂ 21%, Ar/CO₂/прочее 3%; composition provisional;",
}
for old, new in replacements.items():
    if text.count(old) != 1:
        raise SystemExit(f"docs/01_CELESTIAL_SYSTEM.md: pattern drifted: {old}")
    text = text.replace(old, new)
p.write_text(text)

# Changelog.
p = Path("CHANGELOG.md")
text = p.read_text()
anchor = "### Fixed\n\n"
if text.count(anchor) != 1:
    raise SystemExit("CHANGELOG.md: Fixed section anchor drifted")
addition = '''### Fixed

- Atmospheric rigid rotation is transformed from reference-body inertial axes into the
  current vehicle frame before `omega x r`; aircraft attitude no longer changes the
  physical wind field.
- Multi-substep rigid-body integration resamples body ephemeris, gravity, atmosphere
  and control assists on every substep; pilot flight tracing pairs force samples with
  the exact state that produced them.
- Panel dynamic damping projects angular rates onto each panel's local chord/side/lift
  axes instead of assuming every surface is aligned with vehicle XYZ.
- Pilot osculating apoapsis/periapsis uses body-centered inertial velocity rather than
  rotating-surface velocity, and end-frame guards use the matching body-state velocity.
- Cross-platform map-navigation tests use an explicit f32 tolerance instead of requiring
  bit-identical vector round trips.

'''
p.write_text(text.replace(anchor, addition))
