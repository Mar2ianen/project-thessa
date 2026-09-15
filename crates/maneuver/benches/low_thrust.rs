//! Low-thrust benchmark on the real system (N-body, 22 sources).
//!
//! - chemical 800 m/s node realized as one finite burn (penalty measured);
//! - the same 3000 m/s node split into per-orbit pieces (schedule + span);
//! - a 30-day full-throttle ion spiral (the multi-day sustained burn);
//! - driving the segment executor over the chemical plan.
//!
//! All numbers are exact-propagated, not modeled.
use std::{hint::black_box, time::Instant};

use thessa_maneuver::{
    BurnSegment, EngineSpec, FiniteBurnPlan, ManeuverNode, ManeuverPlan, SegmentDirection,
    SegmentExecutor, SplitMode, orbit_period, realize_impulsive, validate_finite_burn,
};
use thessa_sim_core::{BakedEphemeris, GravityField, SimTime, SystemConfig};

const ORBIT_RADIUS: f64 = 1.1e9;

fn chem_engine() -> EngineSpec {
    EngineSpec {
        thrust_n: 100_000.0,
        exhaust_velocity_mps: 4_400.0,
    }
}

fn ion_engine() -> EngineSpec {
    EngineSpec {
        thrust_n: 0.5,
        exhaust_velocity_mps: 30_000.0,
    }
}

fn main() {
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml")).unwrap();
    let ephemeris: BakedEphemeris = config.bake().unwrap();
    let field = GravityField::from_ephemeris(&ephemeris);
    let nereid = ephemeris.body_id("nereid").expect("nereid");
    let nereid_mu = ephemeris.body(nereid).unwrap().mu;
    let nereid_state = ephemeris.body_state(nereid, SimTime::EPOCH).unwrap();
    let v_circ = (nereid_mu / ORBIT_RADIUS).sqrt();
    let start_pos = nereid_state.position_inertial + glam::DVec3::X * ORBIT_RADIUS;
    let start_vel = nereid_state.velocity_inertial + glam::DVec3::Y * v_circ;
    println!(
        "sources: {}, parking v_circ={:.0} m/s",
        field.source_count(),
        v_circ
    );
    // Per-orbit spacing from the Nereid-relative osculating period (never
    // from system-frame state against a bare μ — that misframes the orbit).
    let period = orbit_period(
        glam::DVec3::X * ORBIT_RADIUS,
        glam::DVec3::Y * v_circ,
        nereid_mu,
    )
    .expect("bound parking orbit");
    println!("parking period: {:.2} days", period / 86_400.0);

    // A: chemical 800 m/s node -> one finite burn (146 s < 600 s cap).
    let engine = chem_engine();
    let impulsive = ManeuverPlan::new(
        vec![
            ManeuverNode::new(SimTime(SimTime::EPOCH.0 + 3_600.0), glam::DVec3::Y * 800.0).unwrap(),
        ],
        start_pos,
        start_vel,
        SimTime::EPOCH,
    )
    .unwrap();
    let started = Instant::now();
    let plan = realize_impulsive(
        black_box(&impulsive),
        &engine,
        20_000.0,
        600.0,
        SplitMode::PerOrbit {
            orbit_period_s: period,
        },
    )
    .expect("realizes");
    let validation = validate_finite_burn(black_box(&field), &plan, &impulsive).expect("validates");
    println!(
        "chemical 800 m/s: {} segment(s) of {:.1}s, divergence {:.1} m, dV {:.1} m/s, prop {:.1} kg in {:?}",
        plan.segments.len(),
        plan.total_burn_s(),
        validation.divergence_m,
        validation.velocity_divergence_mps,
        validation.propellant_kg,
        started.elapsed(),
    );

    // B: 3000 m/s split into per-orbit pieces (efficient phasing).
    let big_impulsive = ManeuverPlan::new(
        vec![
            ManeuverNode::new(
                SimTime(SimTime::EPOCH.0 + 2_000_000.0),
                glam::DVec3::Y * 3_000.0,
            )
            .unwrap(),
        ],
        start_pos,
        start_vel,
        SimTime::EPOCH,
    )
    .unwrap();
    let started = Instant::now();
    let big = realize_impulsive(
        black_box(&big_impulsive),
        &engine,
        20_000.0,
        120.0,
        SplitMode::PerOrbit {
            orbit_period_s: period,
        },
    )
    .expect("splits");
    let big_validation =
        validate_finite_burn(black_box(&field), &big, &big_impulsive).expect("validates");
    let span_days = (big.segments.last().unwrap().end().0 - big.segments[0].start.0) / 86_400.0;
    println!(
        "chemical 3000 m/s split: {} pieces over {:.1} days, divergence {:.3e} m (includes rephasing), dV {:.0} m/s, prop {:.0} kg in {:?}",
        big.segments.len(),
        span_days,
        big_validation.divergence_m,
        big_validation.velocity_divergence_mps,
        big_validation.propellant_kg,
        started.elapsed(),
    );

    // C: 30-day ion spiral (single sustained arc, prograde steering).
    // Radius measured against Nereid AT THE END EPOCH (it moves ~37 km/s;
    // measuring against its start position would report its own motion).
    let ion = ion_engine();
    let spiral_dur = 30.0 * 86_400.0;
    let spiral_end_epoch = SimTime(SimTime::EPOCH.0 + spiral_dur);
    let spiral = FiniteBurnPlan::new(
        vec![BurnSegment {
            start: SimTime::EPOCH,
            duration_s: spiral_dur,
            planned_dv_mps: ion.thrust_n / 2_000.0 * spiral_dur,
            direction: SegmentDirection::Prograde,
            throttle_01: 1.0,
        }],
        ion,
        2_000.0,
        start_pos,
        start_vel,
        SimTime::EPOCH,
    )
    .unwrap();
    let spiral_ref = ManeuverPlan::new(
        vec![ManeuverNode::new(SimTime::EPOCH, glam::DVec3::Y * 600.0).unwrap()],
        start_pos,
        start_vel,
        SimTime::EPOCH,
    )
    .unwrap();
    let started = Instant::now();
    let spiral_validation =
        validate_finite_burn(black_box(&field), &spiral, &spiral_ref).expect("validates");
    let nereid_end = ephemeris.body_state(nereid, spiral_end_epoch).unwrap();
    let radius_gain = (spiral_validation.end_state.position - nereid_end.position_inertial)
        .length()
        - ORBIT_RADIUS;
    println!(
        "ion 30-day spiral: radius +{:.0} km, dV {:.0} m/s, prop {:.2} kg, final mass {:.1} kg in {:?}",
        radius_gain / 1_000.0,
        spiral_validation.velocity_divergence_mps,
        spiral_validation.propellant_kg,
        spiral_validation.final_mass_kg,
        started.elapsed(),
    );

    // D: drive the segment executor over the chemical plan (near-ideal
    // 5.5 m/s^2 engine against a plan timed for 5.0-6.0: cutoff stays on
    // schedule, the small residual is reported, not burned through).
    let started = Instant::now();
    let mut executor = SegmentExecutor::new(&plan).unwrap();
    let mut time = SimTime::EPOCH.0;
    let mut accel = glam::DVec3::ZERO;
    let mut polls = 0;
    loop {
        let output = executor.poll(SimTime(time), accel, start_vel).unwrap();
        accel = if output.command.throttle_01 > 0.0 {
            output.command.point_inertial * 5.5
        } else {
            glam::DVec3::ZERO
        };
        time += 1.0;
        polls += 1;
        if output.done {
            break;
        }
        assert!(polls < 2_000_000, "executor must finish on schedule");
    }
    println!(
        "executor: {polls} polls, shortfall {:.2} m/s in {:?}",
        executor
            .poll(SimTime(time), glam::DVec3::ZERO, start_vel)
            .unwrap()
            .last_shortfall_mps,
        started.elapsed(),
    );
}
