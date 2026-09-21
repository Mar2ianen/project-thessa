//! Editor analyzer: thrust/Isp versus altitude at fixed throttle — the Juno
//! Performance Analyzer backend. Pure function over the atmosphere provider;
//! the editor calls this live while the player drags sliders.

use serde::{Deserialize, Serialize};

use crate::atmosphere::AtmosphereConfig;

use super::{CompiledEngine, PropulsionError};

/// One altitude row of the editor performance analysis.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AltitudePoint {
    pub altitude_m: f64,
    pub ambient_pa: f64,
    pub thrust_n: f64,
    pub isp_s: f64,
    pub mass_flow_kg_s: f64,
    pub separation_risk: bool,
}

/// Thrust/Isp versus altitude at fixed throttle and (for solids) burn time.
/// Pure function over the atmosphere provider: the editor calls this live
/// while the player drags Juno-style sliders; nothing here touches Bevy.
pub fn analyze_altitude(
    engine: &CompiledEngine,
    atmosphere: &AtmosphereConfig,
    altitudes_m: &[f64],
    throttle: f64,
    burn_time_s: f64,
) -> Result<Vec<AltitudePoint>, PropulsionError> {
    if altitudes_m.is_empty() {
        return Err(PropulsionError::InvalidSpec(
            "analyzer needs at least one altitude".into(),
        ));
    }
    let mut points = Vec::with_capacity(altitudes_m.len());
    for altitude_m in altitudes_m {
        if !altitude_m.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "analyzer altitudes must be finite".into(),
            ));
        }
        let sample = atmosphere
            .sample(*altitude_m)
            .map_err(|error| PropulsionError::InvalidSpec(format!("atmosphere sample: {error}")))?;
        let point = engine.operating_point(throttle, sample.pressure_pa, burn_time_s)?;
        points.push(AltitudePoint {
            altitude_m: *altitude_m,
            ambient_pa: sample.pressure_pa,
            thrust_n: point.thrust_n,
            isp_s: point.isp_s,
            mass_flow_kg_s: point.mass_flow_kg_s,
            separation_risk: point.separation_risk,
        });
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::super::liquid::merlin_like;
    use super::*;

    #[test]
    fn analyzer_curve_falls_with_ambient() {
        // Juno Performance Analyzer contract: thrust rises as the rocket
        // climbs, Isp follows, mass flow stays frozen at fixed throttle.
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let atmosphere = AtmosphereConfig::default();
        let altitudes: Vec<f64> = (0..=10).map(|k| k as f64 * 8000.0).collect();
        let curve = analyze_altitude(&engine, &atmosphere, &altitudes, 1.0, 0.0).expect("analyze");
        assert_eq!(curve.len(), altitudes.len());
        for window in curve.windows(2) {
            assert!(
                window[1].thrust_n >= window[0].thrust_n,
                "thrust must not fall with altitude"
            );
            assert!(
                (window[1].mass_flow_kg_s - window[0].mass_flow_kg_s).abs()
                    / window[0].mass_flow_kg_s
                    < 1e-9
            );
        }
        assert!(curve.last().expect("top").isp_s > curve[0].isp_s);
    }
}
