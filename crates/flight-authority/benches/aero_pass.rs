use std::hint::black_box;
use std::time::Instant;

use glam::{DMat3, DQuat, DVec3};
use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroModel, AeroPanel, AeroSimdScratch, AeroState, AtmosphereConfig,
    FlightStepInput, PanelAeroModel, PanelSoA, RigidBodyProperties, RigidBodyState,
    integrate_rigid_body_step_soa, integrate_rigid_body_step_with_aero_result,
};

const STEP_S: f64 = 1.0 / 120.0;
const ITERATIONS: usize = 8_192;
const REPETITIONS: usize = 5;
const PANEL_COUNT: usize = 64;

struct Workload {
    model: PanelAeroModel,
    geometry: AeroGeometry,
    panels: PanelSoA,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    properties: RigidBodyProperties,
    input: FlightStepInput,
    environment: thessa_sim_core::AeroEnvironment,
}

impl Workload {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let model = PanelAeroModel::new(AeroConfig::default())?;
        let panels = (0..PANEL_COUNT)
            .map(|index| {
                let span_y = (index as f64 - PANEL_COUNT as f64 * 0.5) * 0.4;
                AeroPanel::new(DVec3::new(-1.0, span_y, 0.0), DVec3::X, DVec3::Z, 4.0, 1.5)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let geometry = AeroGeometry::new(panels)?;
        let panels = PanelSoA::from_geometry(&geometry)?;
        let atmosphere = AtmosphereConfig::new(288.15, 101_325.0, 287.05287, 1.4, 9.81)?;
        let state = RigidBodyState::new(
            DVec3::new(0.0, 0.0, 1_000.0),
            DVec3::new(180.0, 0.0, 2.0),
            DQuat::IDENTITY,
            DVec3::new(0.01, 0.005, -0.002),
        )?;
        let properties = RigidBodyProperties::new(1_200.0, DMat3::IDENTITY * 900.0)?;
        let input = FlightStepInput {
            extra_force_body_n: DVec3::new(300.0, 0.0, 80.0),
            extra_moment_body_nm: DVec3::new(10.0, -5.0, 2.0),
            ..FlightStepInput::new(1_000.0, DVec3::new(0.0, 0.0, -9.81))
        };
        let environment = atmosphere.aero_environment(1_000.0, DVec3::ZERO)?;
        Ok(Self {
            model,
            geometry,
            panels,
            atmosphere,
            state,
            properties,
            input,
            environment,
        })
    }
}

fn duplicate_pass(workload: &Workload) {
    let state = workload.state;
    let mut scratch = AeroSimdScratch::default();
    for _ in 0..ITERATIONS {
        let aero_state =
            AeroState::new(state.velocity_inertial_mps, state.angular_velocity_body_rps);
        black_box(
            workload
                .model
                .evaluate_state(aero_state, workload.environment, &workload.geometry)
                .unwrap(),
        );
        let (next, forces) = integrate_rigid_body_step_soa(
            &workload.model,
            &workload.panels,
            &mut scratch,
            workload.atmosphere,
            state,
            workload.properties,
            workload.input,
            STEP_S,
        )
        .unwrap();
        black_box((next, forces));
    }
}

fn reused_pass(workload: &Workload) {
    let state = workload.state;
    let mut scratch = AeroSimdScratch::default();
    for _ in 0..ITERATIONS {
        let aero_state =
            AeroState::new(state.velocity_inertial_mps, state.angular_velocity_body_rps);
        let aero_result = workload
            .model
            .evaluate_soa_simd_scratch(
                aero_state,
                workload.environment,
                &workload.panels,
                false,
                &mut scratch,
            )
            .unwrap();
        let (next, forces) = integrate_rigid_body_step_with_aero_result(
            workload.atmosphere,
            state,
            workload.properties,
            workload.input,
            STEP_S,
            aero_result,
        )
        .unwrap();
        black_box((next, forces));
    }
}

fn measure(mut workload: impl FnMut()) -> f64 {
    let started = Instant::now();
    workload();
    started.elapsed().as_secs_f64() * 1.0e9 / ITERATIONS as f64
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let workload = Workload::new()?;
    duplicate_pass(&workload);
    reused_pass(&workload);
    let mut duplicate_ns = Vec::with_capacity(REPETITIONS);
    let mut reused_ns = Vec::with_capacity(REPETITIONS);
    for _ in 0..REPETITIONS {
        duplicate_ns.push(measure(|| duplicate_pass(&workload)));
        reused_ns.push(measure(|| reused_pass(&workload)));
    }
    duplicate_ns.sort_by(f64::total_cmp);
    reused_ns.sort_by(f64::total_cmp);
    let duplicate = duplicate_ns[REPETITIONS / 2];
    let reused = reused_ns[REPETITIONS / 2];
    println!("panels={PANEL_COUNT} iterations={ITERATIONS} repetitions={REPETITIONS}");
    println!("baseline scalar+SoA duplicate: {duplicate:.1} ns/step");
    println!("cached single SoA aero pass:   {reused:.1} ns/step");
    println!("speedup: {:.2}x", duplicate / reused);
    Ok(())
}
