//! Direct native port of the retained coastal photo-flight fixture.
//! Only pilot stick/throttle demands; the existing authority/controller owns
//! all forces, attitude and motion. Not another autopilot framework.
use glam::{DQuat, DVec3};
use thessa_flight_authority::{FlightAuthority, LocalAirKinematics, local_air_kinematics};
use thessa_sim_core::{BakedEphemeris, SimTime};

#[derive(Debug, Clone, Copy, Default)]
pub struct Flyby {
    pub active: bool,
    pub target_alt_datum_m: f64,
    pub target_speed_mps: f64,
    pub target_heading_rad: Option<f64>,
    pub axes: DVec3,
}

impl Flyby {
    pub fn cancel(&mut self) {
        self.active = false;
        self.axes = DVec3::ZERO;
    }

    pub fn engage(
        flight: &FlightAuthority,
        ephemeris: &BakedEphemeris,
    ) -> Result<Self, &'static str> {
        let k = kinematics(flight, ephemeris)
            .ok_or("flyby needs valid reference-world atmospheric flight")?;
        Ok(Self {
            active: true,
            target_alt_datum_m: (k.altitude_m.max(0.0) + 250.0).clamp(400.0, 900.0),
            target_speed_mps: 200.0,
            target_heading_rad: flight.terrain_field.as_deref().and_then(|field| {
                coastal_heading(field, flight.terrain_spin(), k.relative_position_inertial_m)
            }),
            axes: DVec3::ZERO,
        })
    }

    /// No last_forces requirement: remote guidance reads acknowledged pose and
    /// canonical reference-world kinematics, not invented force observations.
    /// Mutates only this fixture's demand; returns throttle slew direction.
    pub fn update(&mut self, flight: &FlightAuthority, ephemeris: &BakedEphemeris) -> Option<f64> {
        let k = kinematics(flight, ephemeris)?;
        let speed = k.surface_velocity_inertial_mps.length();
        let alt = k.altitude_m.max(0.0);
        let error = self.target_alt_datum_m - alt;
        self.axes = if alt < 300.0 {
            DVec3::new(0.3, 0.0, 0.0)
        } else {
            DVec3::new(
                flyby_pitch_cmd(
                    error,
                    k.surface_velocity_inertial_mps.dot(k.radial_up),
                    speed,
                ),
                0.0,
                flyby_roll_leg(
                    self.target_heading_rad,
                    track_angle_rad(
                        flight.terrain_spin(),
                        k.relative_position_inertial_m,
                        k.surface_velocity_inertial_mps,
                    ),
                    craft_bank_rad(flight.state.orientation_body_to_inertial, k.radial_up),
                    speed,
                    error.abs(),
                ),
            )
        }
        .clamp_length(0.0, 1.0);
        Some(if alt < 300.0 {
            1.0
        } else {
            flyby_throttle_slew(speed, self.target_speed_mps)
        })
    }
}

fn kinematics(flight: &FlightAuthority, ephemeris: &BakedEphemeris) -> Option<LocalAirKinematics> {
    let time = SimTime(flight.flight_time_s);
    if flight.flight_error.is_some()
        || !flight.flight_time_s.is_finite()
        || !flight.state.orientation_body_to_inertial.is_normalized()
        || ephemeris.dominant_body(flight.state.position_inertial_m, time)? != flight.reference_body
    {
        return None;
    }
    let body = ephemeris.body_state(flight.reference_body, time).ok()?;
    let k = local_air_kinematics(
        flight.atmosphere,
        flight.state,
        body,
        flight.planet_radius_m,
    )
    .ok()?;
    let medium = flight.atmosphere.sample(k.altitude_m.max(0.0)).ok()?;
    (medium.density_kg_m3 > 0.0 && k.surface_velocity_inertial_mps.is_finite()).then_some(k)
}

// Retained pilot laws unchanged: datum P-law with vertical-speed damping,
// deadband and low-speed governor, then bank-to-turn through ordinary stick.
// These are controller gains, never extra forces or direct pose changes.
fn flyby_pitch_cmd(alt_err_m: f64, vspeed_mps: f64, speed_mps: f64) -> f64 {
    if !alt_err_m.is_finite() || !vspeed_mps.is_finite() {
        return 0.0;
    }
    let governor = if speed_mps.is_finite() {
        ((speed_mps - 300.0) / 60.0).clamp(0.0, 1.0)
    } else {
        0.0
    };
    if governor <= 0.0 {
        return 0.0;
    }
    let cmd = (0.003 * alt_err_m - 0.025 * vspeed_mps) * governor;
    if cmd.abs() < 0.08 {
        0.0
    } else {
        cmd.clamp(-0.25, 0.25)
    }
}

fn flyby_throttle_slew(speed_mps: f64, target_mps: f64) -> f64 {
    if !speed_mps.is_finite() || !target_mps.is_finite() {
        0.0
    } else if speed_mps < target_mps - 5.0 {
        1.0
    } else if speed_mps > target_mps + 5.0 {
        -1.0
    } else {
        0.0
    }
}

fn craft_bank_rad(orientation: DQuat, up: DVec3) -> f64 {
    // Nose +X, right -Y, up +Z: right-wing-down bank is positive.
    // The old pilot used +Y as right here, reversing feedback; its physics
    // regression only checked |bank| and could not catch that sign error.
    (orientation * DVec3::Y).dot(up).clamp(-1.0, 1.0).asin()
}

// Same field/spin convention as authority launch_site. Probe eight points
// over 12 km for each of sixteen headings, only once on explicit engagement.
fn shore_basis(spin: f64, relative: DVec3) -> Option<(DVec3, DVec3, DVec3)> {
    let dir = DQuat::from_rotation_z(-spin) * relative.try_normalize()?;
    let north = (DVec3::Z - dir * dir.dot(DVec3::Z)).try_normalize()?;
    Some((dir, north, north.cross(dir)))
}

fn shore_trace_score(
    field: &thessa_worldgen_rocky::field::PlanetField,
    spin: f64,
    relative: DVec3,
    heading: f64,
) -> Option<f64> {
    let (dir, north, east) = shore_basis(spin, relative)?;
    let (sin, cos) = heading.sin_cos();
    let mut mean = 0.0;
    let mut max: f64 = 0.0;
    for step in 1..=8 {
        let arc = step as f64 * 1500.0 / relative.length().max(1.0);
        let probe = (dir * arc.cos() + (north * cos + east * sin) * arc.sin()).try_normalize()?;
        let h = field.height_m([probe.x, probe.z, -probe.y], 32.0).abs();
        if !h.is_finite() {
            return None;
        }
        mean += h;
        max = max.max(h);
    }
    Some(mean / 8.0 + 0.2 * max)
}

fn coastal_heading(
    field: &thessa_worldgen_rocky::field::PlanetField,
    spin: f64,
    relative: DVec3,
) -> Option<f64> {
    let mut best: Option<(f64, f64)> = None;
    for i in 0..16 {
        let heading = i as f64 * std::f64::consts::TAU / 16.0;
        let score = shore_trace_score(field, spin, relative, heading)?;
        if best.is_none_or(|(_, old)| score < old) {
            best = Some((heading, score));
        }
    }
    best.map(|(heading, _)| heading)
}

fn track_angle_rad(spin: f64, relative: DVec3, velocity: DVec3) -> Option<f64> {
    let (dir, north, east) = shore_basis(spin, relative)?;
    let v = DQuat::from_rotation_z(-spin) * velocity;
    let horizontal = v - dir * v.dot(dir);
    if horizontal.length() < 0.5 {
        None
    } else {
        Some(east.dot(horizontal).atan2(north.dot(horizontal)))
    }
}

fn wrap_angle_err_rad(error: f64) -> f64 {
    (error + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

fn flyby_bank_cmd(error: f64, bank: f64) -> f64 {
    if !error.is_finite() || !bank.is_finite() || (error.abs() < 0.05 && bank.abs() < 0.05) {
        return 0.0;
    }
    let delta = error.clamp(-0.45, 0.45) - bank;
    if delta.abs() < 0.03 {
        0.0
    } else {
        (2.0 * delta).clamp(-0.35, 0.35)
    }
}

fn flyby_roll_leg(
    target: Option<f64>,
    track: Option<f64>,
    bank: f64,
    speed: f64,
    error_abs: f64,
) -> f64 {
    if speed <= 300.0 {
        return 0.0;
    }
    match (target, track) {
        (Some(target), Some(track)) => flyby_bank_cmd(wrap_angle_err_rad(target - track), bank),
        _ if error_abs < 150.0 => 0.10,
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_flight_authority::ControlMode;

    fn fixture() -> (BakedEphemeris, FlightAuthority) {
        let config: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = config.bake().unwrap();
        let mut flight = FlightAuthority::new(&e, e.body_id("thessa").unwrap()).unwrap();
        let (field, sites) =
            thessa_flight_authority::launch_site::canonical_launch_setup(&e).unwrap();
        flight.initialize_world_site(field, sites[0], &e);
        (e, flight)
    }

    #[test]
    fn retained_laws_pin_deadbands_signs_governor_and_bank_hold() {
        for speed in [100.0, 200.0, 300.0, f64::NAN] {
            assert_eq!(flyby_pitch_cmd(500.0, -50.0, speed), 0.0);
        }
        assert!(flyby_pitch_cmd(150.0, -3.0, 360.0) > 0.0);
        assert!(flyby_pitch_cmd(-150.0, 3.0, 360.0) < 0.0);
        assert_eq!(flyby_pitch_cmd(0.0, 0.0, 360.0), 0.0);
        for (speed, slew) in [(150.0, 1.0), (200.0, 0.0), (210.0, -1.0)] {
            assert_eq!(flyby_throttle_slew(speed, 200.0), slew);
        }
        assert!(flyby_bank_cmd(0.5, 0.0) > 0.0);
        assert_eq!(flyby_bank_cmd(0.4, 0.4), 0.0);
        assert!(flyby_bank_cmd(0.0, 0.3) < 0.0);
        assert_eq!(flyby_roll_leg(Some(1.0), Some(0.0), 0.0, 200.0, 0.0), 0.0);
        assert!(wrap_angle_err_rad(10.0).abs() <= std::f64::consts::PI);
    }

    #[test]
    fn closed_loop_uses_real_authority_and_preserves_launch_safety_envelope() {
        let (e, mut flight) = fixture();
        let mut flyby = Flyby::engage(&flight, &e).unwrap();
        let initial = flight.state;
        assert!(flyby.target_alt_datum_m >= 400.0 && flyby.target_alt_datum_m <= 900.0);
        // Same deterministic 18-second smoke as the retained pilot fixture,
        // now driving the actual native port rather than a second copied loop.
        flight.sas_enabled = true;
        flight.engine_active = true;
        let mut min_alt = f64::INFINITY;
        let mut max_alt: f64 = 0.0;
        let mut max_stick: f64 = 0.0;
        for _ in 0..1080 {
            let before = flight.state;
            let slew = flyby.update(&flight, &e).unwrap();
            assert_eq!(
                before, flight.state,
                "guidance cannot mutate physical state"
            );
            flight.control_input = flyby.axes;
            flight.throttle = thessa_flight_authority::input::slew_throttle(
                flight.throttle,
                1.0 / 60.0,
                slew > 0.0,
                slew < 0.0,
            );
            flight
                .advance(&e, ControlMode::Navball, 1.0 / 60.0)
                .unwrap();
            let alt = kinematics(&flight, &e).unwrap().altitude_m;
            min_alt = min_alt.min(alt);
            max_alt = max_alt.max(alt);
            max_stick = max_stick.max(flyby.axes.length());
        }
        println!(
            "native_flyby_18s min_alt={min_alt:.3} max_alt={max_alt:.3} max_stick={max_stick:.6}"
        );
        assert!(
            min_alt > 300.0 && max_alt < 2000.0,
            "retained launch-envelope regression"
        );
        assert!(max_stick < 0.7);
        assert_ne!(
            flight.state.position_inertial_m,
            initial.position_inertial_m
        );
    }

    #[test]
    fn shore_search_is_bounded_and_better_than_cross_shore() {
        let (e, _) = fixture();
        let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
        let [dir, _, _] =
            thessa_flight_authority::launch_site::survey_bookmarks(&field, &e).unwrap();
        let relative = DVec3::new(dir[0], -dir[2], dir[1]) * (field.params.radius_m + 600.0);
        let heading = coastal_heading(&field, 0.0, relative).unwrap();
        let score = shore_trace_score(&field, 0.0, relative, heading).unwrap();
        let cross = shore_trace_score(&field, 0.0, relative, heading + std::f64::consts::FRAC_PI_2)
            .unwrap();
        assert!(score < cross && score < 150.0);
        assert!(coastal_heading(&field, 0.0, DVec3::Z * field.params.radius_m).is_none());
        assert!(track_angle_rad(0.0, relative, DVec3::ZERO).is_none());
    }

    #[test]
    fn retained_roll_sign_is_pinned_against_the_six_dof_controller() {
        let (e, mut roll) = fixture();
        let (_, mut flat) = fixture();
        roll.control_input = DVec3::new(0.0, 0.0, 0.2);
        for _ in 0..120 {
            roll.advance(&e, ControlMode::Navball, 1.0 / 60.0).unwrap();
            flat.advance(&e, ControlMode::Navball, 1.0 / 60.0).unwrap();
        }
        let rolled = kinematics(&roll, &e).unwrap();
        let level = kinematics(&flat, &e).unwrap();
        let bank = craft_bank_rad(roll.state.orientation_body_to_inertial, rolled.radial_up);
        let reference = craft_bank_rad(flat.state.orientation_body_to_inertial, level.radial_up);
        assert!(
            bank > reference + 0.02,
            "positive roll stick must bank right; roll={bank}, level={reference}"
        );
        assert!(flyby_roll_leg(Some(0.4), Some(0.0), 0.0, 360.0, 0.0) > 0.0);
        assert_eq!(flyby_roll_leg(Some(0.4), Some(0.0), 0.4, 360.0, 0.0), 0.0);
    }

    #[test]
    fn high_speed_shore_turn_reduces_heading_error_through_the_real_actuators() {
        let (e, mut flight) = fixture();
        let k = kinematics(&flight, &e).unwrap();
        let nose = flight.state.orientation_body_to_inertial * DVec3::X;
        let horizontal = (nose - k.radial_up * nose.dot(k.radial_up)).normalize();
        let body = e
            .body_state(flight.reference_body, SimTime(flight.flight_time_s))
            .unwrap();
        flight.state.velocity_inertial_mps = body.velocity_inertial
            + flight
                .atmosphere
                .body_rotation_rad_s
                .cross(k.relative_position_inertial_m)
            + horizontal * 360.0;
        flight.engine_active = true;
        flight.sas_enabled = true;
        let mut flyby = Flyby::engage(&flight, &e).unwrap();
        let k = kinematics(&flight, &e).unwrap();
        let initial_track = track_angle_rad(
            flight.terrain_spin(),
            k.relative_position_inertial_m,
            k.surface_velocity_inertial_mps,
        )
        .unwrap();
        let target = initial_track + 0.25;
        flyby.target_heading_rad = Some(target);
        flyby.target_alt_datum_m = k.altitude_m;
        flyby.target_speed_mps = 360.0;
        for _ in 0..480 {
            let slew = flyby.update(&flight, &e).unwrap();
            flight.throttle = thessa_flight_authority::input::slew_throttle(
                flight.throttle,
                1.0 / 60.0,
                slew > 0.0,
                slew < 0.0,
            );
            flight.control_input = flyby.axes;
            flight
                .advance(&e, ControlMode::Navball, 1.0 / 60.0)
                .unwrap();
        }
        let k = kinematics(&flight, &e).unwrap();
        let final_track = track_angle_rad(
            flight.terrain_spin(),
            k.relative_position_inertial_m,
            k.surface_velocity_inertial_mps,
        )
        .unwrap();
        let final_error = wrap_angle_err_rad(target - final_track);
        println!(
            "native_flyby_turn initial_error=0.25 final_error={final_error:.6} final_alt_m={:.3} speed_mps={:.3} bank_rad={:.6} roll_cmd={:.6}",
            k.altitude_m,
            k.surface_velocity_inertial_mps.length(),
            craft_bank_rad(flight.state.orientation_body_to_inertial, k.radial_up),
            flyby.axes.z
        );
        assert!(
            final_error.abs() < 0.249,
            "bank feedback must turn toward the requested shoreline heading, not away from it"
        );
        let bank = craft_bank_rad(flight.state.orientation_body_to_inertial, k.radial_up);
        assert!(
            (0.1..0.35).contains(&bank) && flyby.axes.z.abs() < 0.03,
            "bank hold must settle instead of continuously barrel-rolling"
        );
        assert!(
            k.surface_velocity_inertial_mps.length() > 300.0,
            "the turn must actually exercise the high-speed leg"
        );
    }

    #[test]
    #[ignore = "CPU diagnostic: run release --ignored --nocapture"]
    fn flyby_guidance_cost() {
        use std::{hint::black_box, time::Instant};
        let (e, flight) = fixture();
        let start = Instant::now();
        let mut flyby = Flyby::engage(&flight, &e).unwrap();
        let engage_us = start.elapsed().as_secs_f64() * 1e6;
        let start = Instant::now();
        for _ in 0..6400 {
            black_box(flyby.update(black_box(&flight), &e)).unwrap();
        }
        println!(
            "native_flyby engage_us={engage_us:.3} update_us={:.3} batch64_us={:.3}; no stepping/GPU or game-FPS/speedup claim",
            start.elapsed().as_secs_f64() * 1e6 / 6400.0,
            start.elapsed().as_secs_f64() * 1e6 / 100.0
        );
    }
}
