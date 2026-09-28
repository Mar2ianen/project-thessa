//! Engine and vehicle performance analysis output.

use super::*;

fn jet_analyzer_mach_grid(engine: &CompiledJet) -> &'static [f64] {
    match engine {
        CompiledJet::Estoc(_) => &[0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        CompiledJet::Air(engine) => match engine.cycle {
            thessa_sim_core::AirCycle::Scramjet => &[0.0, 1.0, 2.0, 4.0, 6.0, 8.0],
            thessa_sim_core::AirCycle::Ramjet => &[0.0, 1.0, 2.0, 3.0, 4.0],
            _ => &[0.0, 1.0, 2.0, 3.0],
        },
    }
}

/// Juno Performance Analyzer equivalent for the terminal: thrust/Isp over
/// altitude per engine (plus the uniform-command vehicle total) at fixed
/// throttle. `--analyze-json` emits the same rows as JSON for the future
/// editor UI to consume.
pub(super) fn run_analyzer(
    vehicle: &VehicleDefinition,
    throttle: f64,
    burn_time_s: f64,
    as_json: bool,
    composition: &str,
    source_rpm: f64,
    power_takeoff_fraction: f64,
) -> Result<(), Box<dyn Error>> {
    // Species basis is explicit: the atmosphere derives both its gas
    // properties and its species from the design composition string
    // (default Thessa air), never from a hard-coded oxygen scalar
    // (doc 04 section 10 / 18.6).
    let atmosphere = AtmosphereConfig::from_composition(composition, 288.15, 101_325.0, 9.80665)?;
    let altitudes: Vec<f64> = (0..=10).map(|k| k as f64 * 8000.0).collect();
    let airspeeds_mps = [0.0, 50.0, 100.0, 150.0];
    if as_json {
        let mut rows = Vec::new();
        for mount in &vehicle.engines {
            rows.push(serde_json::json!({
                "engine": mount.name,
                "points": analyze_altitude(
                    &mount.engine,
                    &atmosphere,
                    &altitudes,
                    throttle,
                    burn_time_s,
                )?,
            }));
        }
        for mount in &vehicle.systems {
            let throttles = vec![throttle; mount.system.chambers.len()];
            rows.push(serde_json::json!({
                "system": mount.name,
                "points": mount.system.analyze_system_altitude(
                    &atmosphere,
                    &altitudes,
                    &throttles,
                )?,
            }));
        }
        for mount in &vehicle.jets {
            let air = match &mount.engine {
                CompiledJet::Air(engine) => engine.as_ref(),
                CompiledJet::Estoc(engine) => &engine.air,
            };
            let mach_grid = jet_analyzer_mach_grid(&mount.engine);
            let air_points =
                analyze_airbreathing(air, &atmosphere, &altitudes, mach_grid, throttle)?;
            let estoc_points = match &mount.engine {
                CompiledJet::Air(_) => None,
                CompiledJet::Estoc(engine) => Some(analyze_estoc(
                    engine,
                    &atmosphere,
                    &altitudes,
                    mach_grid,
                    throttle,
                )?),
            };
            rows.push(serde_json::json!({
                "jet": mount.name,
                "points": air_points,
                "estoc_points": estoc_points,
            }));
        }
        for mount in &vehicle.propeller_drives {
            rows.push(serde_json::json!({
                "propeller_drive": mount.name,
                "points": analyze_propeller_drive(
                    &mount.drive,
                    &atmosphere,
                    &altitudes,
                    &airspeeds_mps,
                    throttle,
                    source_rpm,
                )?,
            }));
        }
        for mount in &vehicle.turboprops {
            rows.push(serde_json::json!({
                "turboprop": mount.name,
                "points": analyze_turboprop_drive(
                    &mount.drive,
                    &atmosphere,
                    &altitudes,
                    &airspeeds_mps,
                    throttle,
                    power_takeoff_fraction,
                )?,
            }));
        }
        println!("{}", serde_json::to_string_pretty(&rows)?);
        return Ok(());
    }
    for mount in &vehicle.engines {
        println!("--- analyzer: {} (throttle {throttle})", mount.name);
        println!(
            "{:>10} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "p_amb", "thrust_kN", "isp_s", "sep"
        );
        let curve = analyze_altitude(
            &mount.engine,
            &atmosphere,
            &altitudes,
            throttle,
            burn_time_s,
        )?;
        for point in &curve {
            println!(
                "{:>10.0} {:>10.0} {:>12.1} {:>10.1} {:>5}",
                point.altitude_m,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                if point.separation_risk { "SEP" } else { "" },
            );
        }
    }
    for mount in &vehicle.systems {
        println!(
            "--- analyzer: {} (throttle {throttle}, {} chambers)",
            mount.name,
            mount.system.chambers.len()
        );
        println!(
            "{:>10} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "p_amb", "thrust_kN", "isp_s", "sep"
        );
        let throttles = vec![throttle; mount.system.chambers.len()];
        let curve = mount
            .system
            .analyze_system_altitude(&atmosphere, &altitudes, &throttles)?;
        for point in &curve {
            println!(
                "{:>10.0} {:>10.0} {:>12.1} {:>10.1} {:>5}",
                point.altitude_m,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                if point.separation_any { "SEP" } else { "" },
            );
        }
    }
    for mount in &vehicle.jets {
        let air = match &mount.engine {
            CompiledJet::Air(engine) => engine.as_ref(),
            CompiledJet::Estoc(engine) => &engine.air,
        };
        let mach_grid = jet_analyzer_mach_grid(&mount.engine);
        println!(
            "--- analyzer: {} (throttle {throttle}, air path)",
            mount.name
        );
        println!(
            "{:>10} {:>6} {:>10} {:>12} {:>10} {:>5}",
            "alt_m", "mach", "p_amb", "thrust_kN", "isp_s", "flags"
        );
        let grid = analyze_airbreathing(air, &atmosphere, &altitudes, mach_grid, throttle)?;
        for point in &grid {
            let mut flags = String::new();
            if point.air_limited {
                flags.push('A');
            }
            if point.oxygen_limited {
                flags.push('O');
            }
            if point.drive_limited {
                flags.push('D');
            }
            if point.combustion_thermal_limited {
                flags.push('T');
            }
            if point.scramjet_limited {
                flags.push('M');
            }
            if point.separation_risk {
                flags.push('S');
            }
            println!(
                "{:>10.0} {:>6.1} {:>10.0} {:>12.1} {:>10.0} {:>5}",
                point.altitude_m,
                point.mach,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.isp_s,
                flags,
            );
        }
        if let CompiledJet::Estoc(engine) = &mount.engine {
            println!(
                "--- ESTOC envelope: {} (steady precooler, throttle {throttle})",
                mount.name
            );
            println!(
                "{:>10} {:>6} {:>9} {:>12} {:>10} {:>10} {:>8} {:>7}",
                "alt_m", "mach", "mode", "thrust_kN", "isp_s", "fuel_g/s", "spool_n", "cool"
            );
            let rows = analyze_estoc(engine, &atmosphere, &altitudes, mach_grid, throttle)?;
            for point in &rows {
                let mode = match point.mode {
                    thessa_sim_core::EstocMode::Air => "air",
                    thessa_sim_core::EstocMode::Rocket => "rocket",
                    thessa_sim_core::EstocMode::Ejector => "ejector",
                };
                println!(
                    "{:>10.0} {:>6.1} {:>9} {:>12.1} {:>10.0} {:>10.2} {:>8.3} {:>7}",
                    point.altitude_m,
                    point.mach,
                    mode,
                    point.thrust_n / 1000.0,
                    point.isp_total_s,
                    point.fuel_flow_kg_s * 1000.0,
                    point.spool_n,
                    if point.precooler_saturated { "SAT" } else { "" },
                );
            }
        }
    }
    for mount in &vehicle.propeller_drives {
        println!(
            "--- analyzer: {} (throttle {throttle}, source {source_rpm:.0} RPM)",
            mount.name
        );
        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>10} {:>10} {:>5}",
            "alt_m", "speed", "p_amb", "thrust_kN", "fuel_g/s", "bus_kW", "flags"
        );
        let rows = analyze_propeller_drive(
            &mount.drive,
            &atmosphere,
            &altitudes,
            &airspeeds_mps,
            throttle,
            source_rpm,
        )?;
        for point in &rows {
            let mut flags = String::new();
            if point.oxygen_limited {
                flags.push('O');
            }
            if point.thermal_limited {
                flags.push('T');
            }
            if point.density_limited {
                flags.push('V');
            }
            if point.source_speed_limited {
                flags.push('R');
            }
            println!(
                "{:>10.0} {:>8.0} {:>10.0} {:>12.2} {:>10.2} {:>10.2} {:>5}",
                point.altitude_m,
                point.airspeed_mps,
                point.ambient_pa,
                point.thrust_n / 1000.0,
                point.fuel_flow_kg_s * 1000.0,
                point.electrical_power_w / 1000.0,
                flags,
            );
        }
    }
    for mount in &vehicle.turboprops {
        println!(
            "--- analyzer: {} (throttle {throttle}, PTO {:.0}% of available)",
            mount.name,
            power_takeoff_fraction * 100.0,
        );
        println!(
            "{:>10} {:>8} {:>10} {:>12} {:>12} {:>10} {:>5}",
            "alt_m", "speed", "p_amb", "total_kN", "prop_kN", "PTO_kW", "spool"
        );
        let rows = analyze_turboprop_drive(
            &mount.drive,
            &atmosphere,
            &altitudes,
            &airspeeds_mps,
            throttle,
            power_takeoff_fraction,
        )?;
        for point in &rows {
            println!(
                "{:>10.0} {:>8.0} {:>10.0} {:>12.2} {:>12.2} {:>10.1} {:>5.2}",
                point.altitude_m,
                point.airspeed_mps,
                point.ambient_pa,
                point.total_thrust_n / 1000.0,
                point.propeller_thrust_n / 1000.0,
                point.power_takeoff_w / 1000.0,
                point.spool_n,
            );
        }
    }
    Ok(())
}
