use std::{error::Error, fmt};

use serde::{Deserialize, Serialize};

use crate::aero::AeroEnvironment;

const EARTH_STANDARD_GRAVITY_MPS2: f64 = 9.80665;
const AIR_GAS_CONSTANT_J_KG_K: f64 = 287.05287;
const AIR_GAMMA: f64 = 1.4;
const SUTHERLAND_REFERENCE_TEMPERATURE_K: f64 = 273.15;
const SUTHERLAND_CONSTANT_K: f64 = 110.4;
const SUTHERLAND_REFERENCE_VISCOSITY_PA_S: f64 = 1.716e-5;

/// A deterministic, hydrostatic atmosphere profile with SI/f64 outputs.
///
/// The default layers are the 1976 standard-atmosphere temperature profile
/// through 47 km, which covers ordinary aircraft, high-altitude supersonic
/// flight and the first shuttle-entry validation points. It is deliberately a
/// provider, not a global singleton, so a planet can supply its own gas
/// constant, gamma and layer table later.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereConfig {
    pub sea_level_temperature_k: f64,
    pub sea_level_pressure_pa: f64,
    pub gas_constant_j_kg_k: f64,
    pub heat_capacity_ratio: f64,
    pub gravity_mps2: f64,
    pub sutherland_reference_temperature_k: f64,
    pub sutherland_constant_k: f64,
    pub sutherland_reference_viscosity_pa_s: f64,
    /// Angular velocity of the atmosphere/body, expressed in body axes.
    /// Zero keeps the provider neutral for non-rotating test worlds.
    pub body_rotation_rad_s: glam::DVec3,
}

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            sea_level_temperature_k: 288.15,
            sea_level_pressure_pa: 101_325.0,
            gas_constant_j_kg_k: AIR_GAS_CONSTANT_J_KG_K,
            heat_capacity_ratio: AIR_GAMMA,
            gravity_mps2: EARTH_STANDARD_GRAVITY_MPS2,
            sutherland_reference_temperature_k: SUTHERLAND_REFERENCE_TEMPERATURE_K,
            sutherland_constant_k: SUTHERLAND_CONSTANT_K,
            sutherland_reference_viscosity_pa_s: SUTHERLAND_REFERENCE_VISCOSITY_PA_S,
            body_rotation_rad_s: glam::DVec3::ZERO,
        }
    }
}

impl AtmosphereConfig {
    pub fn new(
        sea_level_temperature_k: f64,
        sea_level_pressure_pa: f64,
        gas_constant_j_kg_k: f64,
        heat_capacity_ratio: f64,
        gravity_mps2: f64,
    ) -> Result<Self, AtmosphereError> {
        let config = Self {
            sea_level_temperature_k,
            sea_level_pressure_pa,
            gas_constant_j_kg_k,
            heat_capacity_ratio,
            gravity_mps2,
            ..Self::default()
        };
        config.validate()?;
        Ok(config)
    }

    pub fn validate(self) -> Result<(), AtmosphereError> {
        let finite = [
            self.sea_level_temperature_k,
            self.sea_level_pressure_pa,
            self.gas_constant_j_kg_k,
            self.heat_capacity_ratio,
            self.gravity_mps2,
            self.sutherland_reference_temperature_k,
            self.sutherland_constant_k,
            self.sutherland_reference_viscosity_pa_s,
        ];
        if finite.iter().any(|value| !value.is_finite()) {
            return Err(AtmosphereError::InvalidConfig(
                "atmosphere configuration contains a non-finite value".into(),
            ));
        }
        if !self.body_rotation_rad_s.is_finite() {
            return Err(AtmosphereError::InvalidConfig(
                "atmosphere body rotation contains a non-finite value".into(),
            ));
        }
        if self.sea_level_temperature_k <= 0.0
            || self.sea_level_pressure_pa <= 0.0
            || self.gas_constant_j_kg_k <= 0.0
            || self.heat_capacity_ratio <= 1.0
            || self.gravity_mps2 <= 0.0
            || self.sutherland_reference_temperature_k <= 0.0
            || self.sutherland_constant_k < 0.0
            || self.sutherland_reference_viscosity_pa_s <= 0.0
        {
            return Err(AtmosphereError::InvalidConfig(
                "atmosphere configuration has an invalid range".into(),
            ));
        }
        Ok(())
    }

    /// Evaluate the atmosphere at geometric altitude in metres.
    ///
    /// Negative altitude is clamped to sea level. Above the final tabulated
    /// layer the final layer is extrapolated isothermally, keeping the output
    /// positive and deterministic for gameplay edge cases.
    pub fn sample(self, altitude_m: f64) -> Result<AtmosphereSample, AtmosphereError> {
        self.validate()?;
        if !altitude_m.is_finite() {
            return Err(AtmosphereError::InvalidAltitude);
        }

        let altitude_m = altitude_m.max(0.0);
        let mut layer_index = 0;
        for (index, candidate) in ISA_LAYERS.iter().enumerate() {
            if altitude_m >= candidate.base_altitude_m {
                layer_index = index;
            } else {
                break;
            }
        }
        let layer = ISA_LAYERS[layer_index];
        let (base_temperature_k, base_pressure_pa) = layer_base_state(self, layer_index);

        let delta_altitude_m = altitude_m - layer.base_altitude_m;
        let temperature_k = layer_temperature(base_temperature_k, layer, delta_altitude_m);
        let pressure_pa = layer_pressure(
            self,
            base_temperature_k,
            base_pressure_pa,
            layer,
            delta_altitude_m,
            temperature_k,
        );
        let density_kg_m3 = pressure_pa / (self.gas_constant_j_kg_k * temperature_k);
        let speed_of_sound_mps =
            (self.heat_capacity_ratio * self.gas_constant_j_kg_k * temperature_k).sqrt();
        let dynamic_viscosity_pa_s = sutherland_viscosity(
            temperature_k,
            self.sutherland_reference_temperature_k,
            self.sutherland_constant_k,
            self.sutherland_reference_viscosity_pa_s,
        );

        if !temperature_k.is_finite()
            || !pressure_pa.is_finite()
            || !density_kg_m3.is_finite()
            || !speed_of_sound_mps.is_finite()
            || !dynamic_viscosity_pa_s.is_finite()
        {
            return Err(AtmosphereError::NonFiniteSample);
        }

        Ok(AtmosphereSample {
            altitude_m,
            temperature_k,
            pressure_pa,
            density_kg_m3,
            speed_of_sound_mps,
            dynamic_viscosity_pa_s,
        })
    }

    /// Build the aerodynamic environment for a vehicle moving through this
    /// sample. The wind remains expressed in vehicle body axes, matching the
    /// `AeroEnvironment` contract.
    pub fn aero_environment(
        self,
        altitude_m: f64,
        wind_velocity_body_mps: glam::DVec3,
    ) -> Result<AeroEnvironment, AtmosphereError> {
        let sample = self.sample(altitude_m)?;
        Ok(AeroEnvironment::new(
            sample.density_kg_m3,
            sample.speed_of_sound_mps,
            sample.dynamic_viscosity_pa_s,
            wind_velocity_body_mps,
        ))
    }

    /// Add the rigid-body air velocity caused by atmospheric rotation at a
    /// vehicle position. Both vectors are in the vehicle/body axes; callers
    /// can then pass the returned value as `AeroEnvironment` wind.
    pub fn rotating_wind_velocity_body_mps(
        self,
        position_body_m: glam::DVec3,
        local_wind_body_mps: glam::DVec3,
    ) -> Result<glam::DVec3, AtmosphereError> {
        self.validate()?;
        if !position_body_m.is_finite() || !local_wind_body_mps.is_finite() {
            return Err(AtmosphereError::InvalidWind);
        }
        let wind = local_wind_body_mps + self.body_rotation_rad_s.cross(position_body_m);
        if wind.is_finite() {
            Ok(wind)
        } else {
            Err(AtmosphereError::NonFiniteWind)
        }
    }
}

/// Thermodynamic and transport state returned by an atmosphere provider.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AtmosphereSample {
    pub altitude_m: f64,
    pub temperature_k: f64,
    pub pressure_pa: f64,
    pub density_kg_m3: f64,
    pub speed_of_sound_mps: f64,
    pub dynamic_viscosity_pa_s: f64,
}

impl AtmosphereSample {
    pub fn dynamic_pressure_pa(self, speed_mps: f64) -> Result<f64, AtmosphereError> {
        if !speed_mps.is_finite() || speed_mps < 0.0 {
            return Err(AtmosphereError::InvalidSpeed);
        }
        let dynamic_pressure_pa = 0.5 * self.density_kg_m3 * speed_mps * speed_mps;
        if dynamic_pressure_pa.is_finite() {
            Ok(dynamic_pressure_pa)
        } else {
            Err(AtmosphereError::NonFinitePressure)
        }
    }

    pub fn mach(self, speed_mps: f64) -> Result<f64, AtmosphereError> {
        if !speed_mps.is_finite() || speed_mps < 0.0 {
            return Err(AtmosphereError::InvalidSpeed);
        }
        let mach = speed_mps / self.speed_of_sound_mps;
        if mach.is_finite() {
            Ok(mach)
        } else {
            Err(AtmosphereError::NonFiniteMach)
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum AtmosphereError {
    InvalidConfig(String),
    InvalidAltitude,
    InvalidSpeed,
    InvalidWind,
    NonFiniteSample,
    NonFiniteWind,
    NonFinitePressure,
    NonFiniteMach,
}

impl fmt::Display for AtmosphereError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfig(message) => {
                write!(formatter, "invalid atmosphere config: {message}")
            }
            Self::InvalidAltitude => write!(formatter, "atmosphere altitude must be finite"),
            Self::InvalidSpeed => write!(
                formatter,
                "atmosphere speed must be finite and non-negative"
            ),
            Self::InvalidWind => write!(formatter, "atmosphere wind and position must be finite"),
            Self::NonFiniteSample => write!(formatter, "atmosphere sample is non-finite"),
            Self::NonFiniteWind => write!(formatter, "rotating atmosphere wind is non-finite"),
            Self::NonFinitePressure => write!(formatter, "dynamic pressure is non-finite"),
            Self::NonFiniteMach => write!(formatter, "Mach number is non-finite"),
        }
    }
}

impl Error for AtmosphereError {}

#[derive(Debug, Clone, Copy)]
struct AtmosphereLayer {
    base_altitude_m: f64,
    lapse_rate_k_per_m: f64,
}

// 1976 Standard Atmosphere geopotential layer boundaries. Temperature and
// pressure bases are derived from AtmosphereConfig so custom planets do not
// silently retain Earth's sea-level state.
const ISA_LAYERS: [AtmosphereLayer; 5] = [
    AtmosphereLayer {
        base_altitude_m: 0.0,
        lapse_rate_k_per_m: -0.0065,
    },
    AtmosphereLayer {
        base_altitude_m: 11_000.0,
        lapse_rate_k_per_m: 0.0,
    },
    AtmosphereLayer {
        base_altitude_m: 20_000.0,
        lapse_rate_k_per_m: 0.001,
    },
    AtmosphereLayer {
        base_altitude_m: 32_000.0,
        lapse_rate_k_per_m: 0.0028,
    },
    AtmosphereLayer {
        base_altitude_m: 47_000.0,
        lapse_rate_k_per_m: 0.0,
    },
];

fn layer_base_state(config: AtmosphereConfig, layer_index: usize) -> (f64, f64) {
    let mut temperature_k = config.sea_level_temperature_k;
    let mut pressure_pa = config.sea_level_pressure_pa;
    for index in 0..layer_index {
        let layer = ISA_LAYERS[index];
        let next_layer = ISA_LAYERS[index + 1];
        let delta_altitude_m = next_layer.base_altitude_m - layer.base_altitude_m;
        let next_temperature_k =
            (temperature_k + layer.lapse_rate_k_per_m * delta_altitude_m).max(1.0);
        pressure_pa = layer_pressure_from_base(
            config,
            temperature_k,
            pressure_pa,
            layer.lapse_rate_k_per_m,
            delta_altitude_m,
            next_temperature_k,
        );
        temperature_k = next_temperature_k;
    }
    (temperature_k, pressure_pa)
}

fn layer_temperature(
    base_temperature_k: f64,
    layer: AtmosphereLayer,
    delta_altitude_m: f64,
) -> f64 {
    (base_temperature_k + layer.lapse_rate_k_per_m * delta_altitude_m).max(1.0)
}

fn layer_pressure(
    config: AtmosphereConfig,
    base_temperature_k: f64,
    base_pressure_pa: f64,
    layer: AtmosphereLayer,
    delta_altitude_m: f64,
    temperature_k: f64,
) -> f64 {
    layer_pressure_from_base(
        config,
        base_temperature_k,
        base_pressure_pa,
        layer.lapse_rate_k_per_m,
        delta_altitude_m,
        temperature_k,
    )
}

fn layer_pressure_from_base(
    config: AtmosphereConfig,
    base_temperature_k: f64,
    base_pressure_pa: f64,
    lapse_rate_k_per_m: f64,
    delta_altitude_m: f64,
    temperature_k: f64,
) -> f64 {
    if lapse_rate_k_per_m.abs() > f64::EPSILON {
        base_pressure_pa
            * (temperature_k / base_temperature_k)
                .powf(-config.gravity_mps2 / (lapse_rate_k_per_m * config.gas_constant_j_kg_k))
    } else {
        base_pressure_pa
            * (-config.gravity_mps2 * delta_altitude_m
                / (config.gas_constant_j_kg_k * base_temperature_k))
                .exp()
    }
}

fn sutherland_viscosity(
    temperature_k: f64,
    reference_temperature_k: f64,
    sutherland_constant_k: f64,
    reference_viscosity_pa_s: f64,
) -> f64 {
    reference_viscosity_pa_s
        * (temperature_k / reference_temperature_k).powf(1.5)
        * (reference_temperature_k + sutherland_constant_k)
        / (temperature_k + sutherland_constant_k)
}
