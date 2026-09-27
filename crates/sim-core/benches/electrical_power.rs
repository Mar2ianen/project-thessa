//! Shared-bus runtime cost for a 64-vessel fleet with batteries,
//! ultracapacitors, a sun-tracking solar array under three stars (one with two
//! overlapping occluders), a reactor, and eight prioritized consumers per
//! vessel. Run with
//! `cargo bench -p thessa-sim-core --bench electrical_power`.

use std::{hint::black_box, time::Instant};

use glam::DVec3;
use thessa_sim_core::*;

const VESSELS: usize = 64;
const ITERATIONS: usize = 10_000;

fn make_system(index: usize) -> ElectricalPowerSystem {
    let battery = BatterySpec {
        name: format!("battery-{index}"),
        capacity_j: 75.0e6,
        initial_charge_fraction: 0.65,
        maximum_charge_power_w: 18.0e3,
        maximum_discharge_power_w: 24.0e3,
        charge_efficiency: 0.94,
        discharge_efficiency: 0.95,
        specific_energy_j_kg: 720.0e3,
        dimensions_body_m: DVec3::new(0.8, 0.6, 0.4),
        position_body_m: DVec3::new(-0.7, 0.0, 0.0),
    };
    let capacitor = UltracapacitorSpec {
        name: format!("ultracap-{index}"),
        capacity_j: 2.0e6,
        initial_charge_fraction: 0.6,
        maximum_charge_power_w: 90.0e3,
        maximum_discharge_power_w: 150.0e3,
        charge_efficiency: 0.97,
        discharge_efficiency: 0.97,
        specific_energy_j_kg: 36.0e3,
        dimensions_body_m: DVec3::new(0.5, 0.5, 0.4),
        position_body_m: DVec3::new(-0.7, 0.45, 0.0),
    };
    let solar = SolarArraySpec {
        name: format!("solar-{index}"),
        cell_count_x: 32,
        cell_count_y: 16,
        cell_size_x_m: 0.1,
        cell_size_y_m: 0.1,
        cell_efficiency: 0.31,
        cell_areal_density_kg_m2: 2.4,
        support_areal_density_kg_m2: 1.0,
        panel_u_axis_body: DVec3::X,
        panel_v_axis_body: DVec3::Y,
        position_body_m: DVec3::new(0.0, 0.0, 0.5),
        deployment: SolarArrayDeployment::Fixed,
        tracking: SolarArrayTracking::SingleAxis {
            rotation_axis_body: DVec3::X,
            minimum_angle_rad: -std::f64::consts::FRAC_PI_2,
            maximum_angle_rad: std::f64::consts::FRAC_PI_2,
            slew_rate_rad_s: 0.1,
            actuator_power_w: 120.0,
            initial_angle_rad: 0.0,
        },
    };
    let reactor = ReactorSpec {
        name: format!("reactor-{index}"),
        rated_thermal_power_w: 80.0e3,
        electric_efficiency: 0.3,
        radiator_capacity_w: 200.0e3,
        initial_fuel_mass_kg: 0.02,
        fuel_specific_energy_j_kg: 8.0e13,
        dry_mass_kg: 380.0,
        dimensions_body_m: DVec3::new(1.1, 1.1, 1.8),
        position_body_m: DVec3::new(0.8, 0.0, -0.3),
    };
    let consumers = (0..8)
        .map(|consumer_index| PowerConsumerSpec {
            name: format!("consumer-{index}-{consumer_index}"),
            rated_power_w: 1.0e3 + consumer_index as f64 * 350.0,
            priority: match consumer_index % 4 {
                0 => PowerPriority::LifeSupport,
                1 => PowerPriority::FlightControl,
                2 => PowerPriority::Propulsion,
                _ => PowerPriority::Utility,
            },
        })
        .collect();
    ElectricalPowerSystem {
        batteries: vec![battery],
        ultracapacitors: vec![capacitor],
        solar_arrays: vec![solar],
        reactors: vec![reactor],
        consumers,
    }
}

fn three_suns() -> Vec<SolarFluxSource> {
    let primary =
        SolarFluxSource::from_luminosity(3.8e26, 1.5e11, DVec3::Z, 1.0).expect("primary sun");
    let secondary_direction = DVec3::new(0.5, 0.3, 0.8).normalize();
    let offset_axis = DVec3::X.cross(secondary_direction).normalize();
    let direction_at_offset =
        |offset_rad: f64| secondary_direction * offset_rad.cos() + offset_axis * offset_rad.sin();
    let secondary = SolarFluxSource::from_luminosity_with_occluders(
        1.2e26,
        5.0e8,
        2.1e11,
        secondary_direction,
        1.0,
        vec![
            SolarOccluder::from_geometry(6.4e6, 4.0e9, direction_at_offset(0.0008))
                .expect("planet occluder"),
            SolarOccluder::from_geometry(1.7e6, 1.2e9, direction_at_offset(-0.0006))
                .expect("moon occluder"),
        ],
    )
    .expect("secondary sun");
    let tertiary = SolarFluxSource::from_luminosity(
        0.4e26,
        3.0e11,
        DVec3::new(-0.4, 0.7, 0.5).normalize(),
        1.0,
    )
    .expect("tertiary sun");
    vec![primary, secondary, tertiary]
}

fn main() {
    let systems: Vec<_> = (0..VESSELS).map(make_system).collect();
    let mut states: Vec<_> = systems
        .iter()
        .map(|system| system.initial_state().expect("valid system"))
        .collect();
    let commands: Vec<_> = systems
        .iter()
        .map(|system| {
            let mut command = ElectricalPowerCommand::idle_for(system, 0.1);
            command.solar_flux = three_suns();
            command.solar_tracking_auto = vec![true];
            command.consumer_power_w = system
                .consumers
                .iter()
                .map(|consumer| consumer.rated_power_w * 0.72)
                .collect();
            command.reactor_power_fraction = vec![0.45];
            command
        })
        .collect();

    let started = Instant::now();
    for _ in 0..ITERATIONS {
        for ((system, state), command) in systems.iter().zip(&mut states).zip(&commands) {
            let (next, telemetry) = system
                .advance(black_box(state), black_box(command))
                .expect("power step");
            *state = black_box(next);
            black_box(telemetry);
        }
    }
    let elapsed = started.elapsed();
    let steps = VESSELS * ITERATIONS;
    println!(
        "{steps} vessel power steps in {elapsed:.3?} ({:.0} steps/s)",
        steps as f64 / elapsed.as_secs_f64()
    );
}
