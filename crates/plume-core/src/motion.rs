//! Read-only quasi-steady plume motion in nozzle-local SI axes (+Z exhaust).
//! No force, clock or vehicle integration lives here.
use crate::{PlumeEnvironment, PlumeSource, source::validation_error};

/// Centre position XYZ and emitted axial coordinate (metres).
/// Retarded attitude assumes constant current angular velocity. Crossflow
/// uses projected-width momentum capture of a slender circular gas column.
/// shortcut: no acceleration/history or axial counterflow coupling; replace
/// with accepted nozzle history and calibrated flow when those are available.
pub fn centerline(
    source: &PlumeSource,
    env: &PlumeEnvironment,
    angular_velocity_rps: [f64; 3],
    length_m: f64,
    samples: u32,
) -> Result<Vec<[f64; 4]>, &'static str> {
    if let Some(reason) = validation_error(source, env) {
        return Err(reason);
    }
    if !length_m.is_finite()
        || length_m <= 0.0
        || angular_velocity_rps.iter().any(|v| !v.is_finite())
    {
        return Err("invalid plume motion");
    }
    let speed = source.exhaust_velocity_mps;
    let flow = source.mass_flow_kg_s * source.throttle;
    if speed <= 0.0 || flow <= 0.0 {
        return Ok(Vec::new());
    }
    let [wx, wy, wz] = angular_velocity_rps;
    let omega = wx.hypot(wy).hypot(wz);
    let wind = [
        env.flow_velocity_local_mps[0],
        env.flow_velocity_local_mps[1],
    ];
    let wind_speed = wind[0].hypot(wind[1]);
    // Lateral gas-column mass per metre = mdot/Ve. Capture width = 2*r_exit.
    // dv/dt = k*(w-v)*abs(w-v), integrated analytically from v(0)=0.
    let k = 2.0 * env.density_kg_m3 * source.exit_radius_m * speed / flow;
    let count = samples.clamp(2, 33);
    let mut points = Vec::with_capacity(count as usize);
    for i in 0..count {
        let z = length_m * f64::from(i) / f64::from(count - 1);
        let age = z / speed;
        let (sin, cos) = (-omega * age).sin_cos();
        let axis = if omega > 0.0 {
            let n = [wx / omega, wy / omega, wz / omega];
            [
                n[1] * sin + n[0] * n[2] * (1.0 - cos),
                -n[0] * sin + n[1] * n[2] * (1.0 - cos),
                cos + n[2] * n[2] * (1.0 - cos),
            ]
        } else {
            [0.0, 0.0, 1.0]
        };
        let q = k * wind_speed * age;
        let fraction = if q < 1e-4 {
            q * (0.5 + q * (-1.0 / 3.0 + q * 0.25))
        } else {
            1.0 - q.ln_1p() / q
        };
        let p = [
            axis[0] * z + wind[0] * age * fraction,
            axis[1] * z + wind[1] * age * fraction,
            axis[2] * z,
            z,
        ];
        if p.iter().any(|v| !v.is_finite()) {
            return Err("unrepresentable plume motion");
        }
        points.push(p);
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    #[test]
    fn rotation_is_retarded_and_crossflow_vanishes_in_vacuum() {
        let source = sample_source();
        let mut env = sample_env_sea_level();
        env.flow_velocity_local_mps = [300.0, -200.0, 900.0];
        env.density_kg_m3 = 0.0;
        let straight = centerline(&source, &env, [0.0; 3], 100.0, 17).unwrap();
        assert_eq!(straight[0], [0.0; 4]);
        assert_eq!(straight[16], [0.0, 0.0, 100.0, 100.0]);
        let rate = 2.0;
        let turned = centerline(&source, &env, [0.0, rate, 0.0], 100.0, 17).unwrap();
        let angle = -rate * 100.0 / source.exhaust_velocity_mps;
        assert!((turned[16][0] - 100.0 * angle.sin()).abs() < 1e-12);
        assert!((turned[16][2] - 100.0 * angle.cos()).abs() < 1e-12);
        env.density_kg_m3 = 1.2;
        let bent = centerline(&source, &env, [0.0; 3], 100.0, 17).unwrap();
        let age = 100.0 / source.exhaust_velocity_mps;
        assert!(bent[16][0] > 0.0 && bent[16][0] < 300.0 * age);
        assert!(bent[16][1] < 0.0 && bent[16][1] > -200.0 * age);
        assert_eq!(bent[16][2], 100.0);
        env.flow_velocity_local_mps[0] *= -1.0;
        let mirrored = centerline(&source, &env, [0.0; 3], 100.0, 17).unwrap();
        assert_eq!(mirrored[16][0], -bent[16][0]);
        let mut off = source;
        off.throttle = 0.0;
        assert!(
            centerline(&off, &env, [0.0; 3], 100.0, 17)
                .unwrap()
                .is_empty()
        );
        assert!(centerline(&source, &env, [f64::NAN; 3], 100.0, 17).is_err());
    }

    #[test]
    fn crossflow_solution_matches_independent_time_integration() {
        let source = sample_source();
        let mut env = sample_env_sea_level();
        env.flow_velocity_local_mps = [250.0, 0.0, 0.0];
        let age = 0.08;
        let k = 2.0 * env.density_kg_m3 * source.exit_radius_m * source.exhaust_velocity_mps
            / (source.mass_flow_kg_s * source.throttle);
        let dt = age / 100_000.0;
        let (mut x, mut v) = (0.0, 0.0);
        for _ in 0..100_000 {
            let a = k * (250.0_f64 - v).powi(2);
            let mid_v = v + 0.5 * dt * a;
            x += mid_v * dt;
            v += k * (250.0 - mid_v).powi(2) * dt;
        }
        let points = centerline(
            &source,
            &env,
            [0.0; 3],
            age * source.exhaust_velocity_mps,
            2,
        )
        .unwrap();
        assert!((points[1][0] - x).abs() < 1e-7, "{} vs {x}", points[1][0]);
    }
}
