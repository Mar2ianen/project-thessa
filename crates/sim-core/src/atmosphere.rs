use std::{error::Error, fmt};

use serde::{Deserialize, Serialize};

use crate::aero::AeroEnvironment;

const EARTH_STANDARD_GRAVITY_MPS2: f64 = 9.80665;
const AIR_GAS_CONSTANT_J_KG_K: f64 = 287.05287;
const AIR_GAMMA: f64 = 1.4;
const SUTHERLAND_REFERENCE_TEMPERATURE_K: f64 = 273.15;
const SUTHERLAND_CONSTANT_K: f64 = 110.4;
const SUTHERLAND_REFERENCE_VISCOSITY_PA_S: f64 = 1.716e-5;
const UNIVERSAL_GAS_CONSTANT_J_MOL_K: f64 = 8.314_462_618;
const BAR_TO_PA: f64 = 100_000.0;

/// Gas properties baked from the composition string in the system design
/// data. Mole fractions are intentionally equal when the design string does
/// not specify fractions (for example, `N2/O2/Ar/CO2`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BakedAtmosphere {
    pub composition: String,
    /// `None` means the design describes a composition but does not define a
    /// playable surface pressure (for example a gas giant entry).
    #[serde(default)]
    pub surface_pressure_pa: Option<f64>,
    pub gas_constant_j_kg_k: f64,
    pub heat_capacity_ratio: f64,
    pub sutherland_reference_temperature_k: f64,
    pub sutherland_constant_k: f64,
    pub sutherland_reference_viscosity_pa_s: f64,
}

impl BakedAtmosphere {
    /// Resolve a human-readable design composition into deterministic mixture
    /// properties. The catalog is deliberately small and explicit: unknown
    /// gases fail during baking instead of silently becoming Earth air.
    pub fn from_design(
        composition: &str,
        surface_pressure_bar: Option<f64>,
    ) -> Result<Self, AtmosphereError> {
        let species = parse_composition(composition)?;
        let (
            gas_constant_j_kg_k,
            heat_capacity_ratio,
            reference_viscosity_pa_s,
            sutherland_constant_k,
        ) = mixture_properties(&species);
        let surface_pressure_pa = match surface_pressure_bar {
            Some(bar) if bar.is_finite() && bar >= 0.0 && (bar * BAR_TO_PA).is_finite() => {
                Some(bar * BAR_TO_PA)
            }
            Some(_) => {
                return Err(AtmosphereError::InvalidConfig(
                    "atmosphere surface pressure must be finite and non-negative".into(),
                ));
            }
            None => None,
        };
        let baked = Self {
            composition: composition.trim().into(),
            surface_pressure_pa,
            gas_constant_j_kg_k,
            heat_capacity_ratio,
            sutherland_reference_temperature_k: SUTHERLAND_REFERENCE_TEMPERATURE_K,
            sutherland_constant_k,
            sutherland_reference_viscosity_pa_s: reference_viscosity_pa_s,
        };
        if !baked.gas_constant_j_kg_k.is_finite()
            || !baked.heat_capacity_ratio.is_finite()
            || !baked.sutherland_constant_k.is_finite()
            || !baked.sutherland_reference_viscosity_pa_s.is_finite()
            || baked.gas_constant_j_kg_k <= 0.0
            || baked.heat_capacity_ratio <= 1.0
            || baked.sutherland_constant_k < 0.0
            || baked.sutherland_reference_viscosity_pa_s <= 0.0
        {
            return Err(AtmosphereError::InvalidConfig(
                "atmosphere composition resolved to invalid gas properties".into(),
            ));
        }
        Ok(baked)
    }
}

#[derive(Debug, Clone, Copy)]
struct GasSpecies {
    molar_mass_kg_mol: f64,
    gamma: f64,
    reference_viscosity_pa_s: f64,
    sutherland_constant_k: f64,
    mole_fraction: f64,
}

fn parse_composition(composition: &str) -> Result<Vec<GasSpecies>, AtmosphereError> {
    let mut components = Vec::new();
    for raw_token in composition.split(|character: char| {
        character == '/' || character == '+' || character == ',' || character.is_whitespace()
    }) {
        let token = raw_token.trim_matches(|character: char| !character.is_ascii_alphanumeric());
        if token.is_empty() {
            continue;
        }
        let normalized = token.to_ascii_uppercase();
        if matches!(
            normalized.as_str(),
            "LOCAL" | "EXOSPHERE" | "PROVISIONAL" | "TRACE" | "TRACES"
        ) {
            continue;
        }
        let gas = match normalized.as_str() {
            "N2" => GasSpecies {
                molar_mass_kg_mol: 0.028_013_4,
                gamma: 1.400,
                reference_viscosity_pa_s: 1.663e-5,
                sutherland_constant_k: 111.0,
                mole_fraction: 0.0,
            },
            "O2" => GasSpecies {
                molar_mass_kg_mol: 0.031_998_8,
                gamma: 1.395,
                reference_viscosity_pa_s: 1.919e-5,
                sutherland_constant_k: 127.0,
                mole_fraction: 0.0,
            },
            "AR" => GasSpecies {
                molar_mass_kg_mol: 0.039_948,
                gamma: 1.667,
                reference_viscosity_pa_s: 2.117e-5,
                sutherland_constant_k: 144.0,
                mole_fraction: 0.0,
            },
            "CO2" => GasSpecies {
                molar_mass_kg_mol: 0.044_009_5,
                gamma: 1.294,
                reference_viscosity_pa_s: 1.370e-5,
                sutherland_constant_k: 222.0,
                mole_fraction: 0.0,
            },
            "SO2" => GasSpecies {
                molar_mass_kg_mol: 0.064_066,
                gamma: 1.290,
                reference_viscosity_pa_s: 1.250e-5,
                sutherland_constant_k: 416.0,
                mole_fraction: 0.0,
            },
            "H2" => GasSpecies {
                molar_mass_kg_mol: 0.002_015_88,
                gamma: 1.405,
                reference_viscosity_pa_s: 8.76e-6,
                sutherland_constant_k: 72.0,
                mole_fraction: 0.0,
            },
            "HE" => GasSpecies {
                molar_mass_kg_mol: 0.004_002_6,
                gamma: 1.667,
                reference_viscosity_pa_s: 1.96e-5,
                sutherland_constant_k: 79.4,
                mole_fraction: 0.0,
            },
            "CH4" => GasSpecies {
                molar_mass_kg_mol: 0.016_042_5,
                gamma: 1.300,
                reference_viscosity_pa_s: 1.10e-5,
                sutherland_constant_k: 170.0,
                mole_fraction: 0.0,
            },
            "NH3" => GasSpecies {
                molar_mass_kg_mol: 0.017_030_5,
                gamma: 1.310,
                reference_viscosity_pa_s: 9.82e-6,
                sutherland_constant_k: 370.0,
                mole_fraction: 0.0,
            },
            "H2O" => GasSpecies {
                molar_mass_kg_mol: 0.018_015_3,
                gamma: 1.330,
                reference_viscosity_pa_s: 1.00e-5,
                sutherland_constant_k: 1_064.0,
                mole_fraction: 0.0,
            },
            _ => {
                return Err(AtmosphereError::InvalidConfig(format!(
                    "unsupported atmosphere gas {token} in composition {composition:?}"
                )));
            }
        };
        components.push((normalized, gas));
    }
    if components.is_empty() {
        return Err(AtmosphereError::InvalidConfig(
            "atmosphere composition contains no recognized gases".into(),
        ));
    }
    let component_count = components.len();
    let thessa_profile = component_count == 4
        && ["N2", "O2", "AR", "CO2"]
            .iter()
            .all(|name| components.iter().any(|(candidate, _)| candidate == name));
    let species = components
        .into_iter()
        .map(|(name, gas)| {
            let mole_fraction = if thessa_profile {
                // Design values from data/worldgen/thessa_v02.toml.
                match name.as_str() {
                    "N2" => 0.735,
                    "O2" => 0.250,
                    "AR" => 0.012,
                    "CO2" => 0.003,
                    _ => unreachable!("validated Thessa composition profile"),
                }
            } else {
                1.0 / component_count as f64
            };
            GasSpecies {
                mole_fraction,
                ..gas
            }
        })
        .collect();
    Ok(species)
}

fn mixture_properties(species: &[GasSpecies]) -> (f64, f64, f64, f64) {
    let mean_molar_mass = species
        .iter()
        .map(|gas| gas.molar_mass_kg_mol * gas.mole_fraction)
        .sum::<f64>();
    let gas_constant = UNIVERSAL_GAS_CONSTANT_J_MOL_K / mean_molar_mass;
    let cp_molar = species
        .iter()
        .map(|gas| {
            gas.mole_fraction * gas.gamma / (gas.gamma - 1.0) * UNIVERSAL_GAS_CONSTANT_J_MOL_K
        })
        .sum::<f64>();
    let heat_capacity_ratio = cp_molar / (cp_molar - UNIVERSAL_GAS_CONSTANT_J_MOL_K);

    // Wilke's mixture rule gives a stable composition-dependent transport
    // coefficient while retaining the same one-parameter Sutherland model in
    // AtmosphereConfig.
    let reference_viscosity_pa_s = species
        .iter()
        .map(|left| {
            let denominator = species
                .iter()
                .map(|right| {
                    let phi = (1.0
                        + (left.reference_viscosity_pa_s / right.reference_viscosity_pa_s).sqrt()
                            * (right.molar_mass_kg_mol / left.molar_mass_kg_mol).powf(0.25))
                    .powi(2)
                        / (8.0 * (1.0 + left.molar_mass_kg_mol / right.molar_mass_kg_mol)).sqrt();
                    right.mole_fraction * phi
                })
                .sum::<f64>();
            left.mole_fraction * left.reference_viscosity_pa_s / denominator
        })
        .sum();
    let sutherland_constant_k = species
        .iter()
        .map(|gas| gas.sutherland_constant_k * gas.mole_fraction)
        .sum::<f64>();
    (
        gas_constant,
        heat_capacity_ratio,
        reference_viscosity_pa_s,
        sutherland_constant_k,
    )
}

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
    /// Angular velocity of the atmosphere/reference body in the inertial
    /// frame used by [`crate::RigidBodyState`]. The flight boundary converts
    /// this vector into vehicle axes before evaluating `omega × r`.
    /// Zero keeps the provider neutral for non-rotating test worlds.
    pub body_rotation_rad_s: glam::DVec3,
    /// Explicit end of the atmosphere: below this density the medium is
    /// declared vacuum and [`sample`](Self::sample) returns exactly zero
    /// density (and pressure). This is what lets rails batches and the
    /// exact-vacuum fast paths engage near a planet instead of only in
    /// deep space where the exponential tail underflows on its own.
    ///
    /// Calibrated, not arbitrary: at the default 1e-10 kg/m³ an X-15-class
    /// vehicle (10 t, CdA ~ 1 m²) at orbital speed sees ~2e-7 m/s² of drag,
    /// i.e. ~1.3 m of drift over the maximum one-hour coast batch — inside
    /// the 5 m batch position tolerance with margin. See the cutoff
    /// regression test pinning this envelope.
    pub vacuum_cutoff_density_kg_m3: f64,
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
            vacuum_cutoff_density_kg_m3: 1.0e-10,
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

    /// Build a runtime provider from baked body composition data.
    pub fn from_baked(
        baked: &BakedAtmosphere,
        sea_level_temperature_k: f64,
        gravity_mps2: f64,
    ) -> Result<Self, AtmosphereError> {
        let sea_level_pressure_pa = baked.surface_pressure_pa.ok_or_else(|| {
            AtmosphereError::InvalidConfig(format!(
                "atmosphere {} has no playable surface pressure",
                baked.composition
            ))
        })?;
        let config = Self {
            sea_level_temperature_k,
            sea_level_pressure_pa,
            gas_constant_j_kg_k: baked.gas_constant_j_kg_k,
            heat_capacity_ratio: baked.heat_capacity_ratio,
            gravity_mps2,
            sutherland_reference_temperature_k: baked.sutherland_reference_temperature_k,
            sutherland_constant_k: baked.sutherland_constant_k,
            sutherland_reference_viscosity_pa_s: baked.sutherland_reference_viscosity_pa_s,
            ..Self::default()
        };
        config.validate()?;
        Ok(config)
    }

    /// Convenience entry point for callers that have not gone through the
    /// system baker yet.
    pub fn from_composition(
        composition: &str,
        sea_level_temperature_k: f64,
        sea_level_pressure_pa: f64,
        gravity_mps2: f64,
    ) -> Result<Self, AtmosphereError> {
        let baked =
            BakedAtmosphere::from_design(composition, Some(sea_level_pressure_pa / BAR_TO_PA))?;
        Self::from_baked(&baked, sea_level_temperature_k, gravity_mps2)
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
            self.vacuum_cutoff_density_kg_m3,
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
            || self.vacuum_cutoff_density_kg_m3 <= 0.0
        {
            return Err(AtmosphereError::InvalidConfig(
                "atmosphere configuration has an invalid range".into(),
            ));
        }
        Ok(())
    }

    /// Approximate geometric altitude where the model density sinks below
    /// the vacuum cutoff: the declared end of the atmosphere for maps, HUD
    /// readouts and batch pre-checks. Found by bisection (the profile is
    /// monotone in practice); sea level when even that is already vacuum.
    pub fn top_altitude_m(self) -> f64 {
        if self
            .sample(0.0)
            .map(|sample| sample.density_kg_m3)
            .unwrap_or(f64::INFINITY)
            < self.vacuum_cutoff_density_kg_m3
        {
            return 0.0;
        }
        let mut low = 0.0;
        let mut high = 4.0e6;
        for _ in 0..80 {
            let mid = 0.5 * (low + high);
            let density = self
                .sample(mid)
                .map(|sample| sample.density_kg_m3)
                .unwrap_or(f64::INFINITY);
            if density < self.vacuum_cutoff_density_kg_m3 {
                high = mid;
            } else {
                low = mid;
            }
        }
        high
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

        // Declared vacuum: below the cutoff the medium is exactly nothing,
        // so exact-vacuum fast paths (skip_aero, rails batches) engage.
        // Temperature and derived acoustics stay on-model (finite, smooth
        // for displays); only the mass terms go to zero.
        let (pressure_pa, density_kg_m3) = if density_kg_m3 < self.vacuum_cutoff_density_kg_m3 {
            (0.0, 0.0)
        } else {
            (pressure_pa, density_kg_m3)
        };

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

    /// Return the atmosphere velocity caused by rigid body rotation at a
    /// vehicle-relative position, expressed in vehicle body axes.
    ///
    /// `body_rotation_rad_s` is stored in inertial/reference-body axes while
    /// `position_body_m` is in vehicle axes. Converting the angular-velocity
    /// vector here prevents a pitched or rolled craft from changing the
    /// physical atmosphere rotation merely because its local basis changed.
    pub fn rotating_air_velocity_body_mps(
        self,
        position_body_m: glam::DVec3,
        orientation_body_to_inertial: glam::DQuat,
    ) -> Result<glam::DVec3, AtmosphereError> {
        self.validate()?;
        if !position_body_m.is_finite() || !orientation_body_to_inertial.is_finite() {
            return Err(AtmosphereError::InvalidWind);
        }
        let orientation_error = (orientation_body_to_inertial.length_squared() - 1.0).abs();
        if orientation_error > 1.0e-6 {
            return Err(AtmosphereError::InvalidWind);
        }
        let rotation_body_rad_s = orientation_body_to_inertial.inverse() * self.body_rotation_rad_s;
        let wind = rotation_body_rad_s.cross(position_body_m);
        if wind.is_finite() {
            Ok(wind)
        } else {
            Err(AtmosphereError::NonFiniteWind)
        }
    }

    /// Add an already same-frame angular-velocity contribution to a local
    /// wind vector.
    ///
    /// This low-level helper is retained for standalone atmosphere tests and
    /// callers whose position and rotation vector are already expressed in
    /// the same axes. Flight integration should prefer
    /// [`Self::rotating_air_velocity_body_mps`].
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
            Self::InvalidWind => write!(formatter, "atmosphere wind/frame data must be finite"),
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

#[cfg(test)]
mod tests {
    use super::*;

    fn thessa_like() -> AtmosphereConfig {
        AtmosphereConfig::new(288.15, 120_000.0, 287.05287, 1.4, 9.0).expect("valid")
    }

    #[test]
    fn composition_bakes_pressure_and_gas_properties() {
        let thessa = BakedAtmosphere::from_design("N2/O2/Ar/CO2 provisional", Some(1.20))
            .expect("Thessa composition");
        assert_eq!(thessa.surface_pressure_pa, Some(120_000.0));
        assert!((thessa.gas_constant_j_kg_k - 284.7326).abs() < 0.01);
        assert!(thessa.heat_capacity_ratio > 1.4);
        assert!(thessa.sutherland_reference_viscosity_pa_s > 0.0);

        let carbon_dioxide =
            BakedAtmosphere::from_design("CO2", Some(1.20)).expect("CO2 composition");
        assert!(carbon_dioxide.gas_constant_j_kg_k < thessa.gas_constant_j_kg_k);
        assert_ne!(
            carbon_dioxide.sutherland_reference_viscosity_pa_s,
            thessa.sutherland_reference_viscosity_pa_s
        );
        let config = AtmosphereConfig::from_baked(&thessa, 288.15, 4.9).expect("runtime config");
        assert_eq!(config.sea_level_pressure_pa, 120_000.0);
        assert_eq!(config.gravity_mps2, 4.9);
    }

    #[test]
    fn composition_rejects_unknown_gas_instead_of_using_air() {
        assert!(BakedAtmosphere::from_design("N2/Unobtanium", Some(1.0)).is_err());
    }

    #[test]
    fn vacuum_cutoff_declares_exact_vacuum_above_top() {
        let atmosphere = thessa_like();
        let top = atmosphere.top_altitude_m();
        assert!(
            (100_000.0..1_000_000.0).contains(&top),
            "Thessa top out of band: {top}"
        );
        let above = atmosphere.sample(top + 10_000.0).expect("sample");
        assert_eq!(above.density_kg_m3, 0.0);
        assert_eq!(above.pressure_pa, 0.0);
        assert!(above.temperature_k.is_finite());
        assert!(above.speed_of_sound_mps.is_finite());
        let below = atmosphere
            .sample((top - 50_000.0).max(0.0))
            .expect("sample");
        assert!(
            below.density_kg_m3 > 0.0,
            "50 km under the top must still be air"
        );
    }

    #[test]
    fn vacuum_cutoff_rejects_nonpositive_thresholds() {
        let mut atmosphere = thessa_like();
        atmosphere.vacuum_cutoff_density_kg_m3 = 0.0;
        assert!(atmosphere.validate().is_err());
        atmosphere.vacuum_cutoff_density_kg_m3 = f64::NAN;
        assert!(atmosphere.validate().is_err());
    }

    #[test]
    fn rotating_air_velocity_is_invariant_under_vehicle_basis_rotation() {
        let atmosphere = AtmosphereConfig {
            body_rotation_rad_s: glam::DVec3::new(0.0, 0.0, 2.0),
            ..AtmosphereConfig::default()
        };
        let orientation = glam::DQuat::from_rotation_y(0.71) * glam::DQuat::from_rotation_x(-0.43);
        let position_body = glam::DVec3::new(3.0, -2.0, 5.0);
        let position_inertial = orientation * position_body;
        let expected_inertial = atmosphere.body_rotation_rad_s.cross(position_inertial);
        let expected_body = orientation.inverse() * expected_inertial;
        let actual = atmosphere
            .rotating_air_velocity_body_mps(position_body, orientation)
            .expect("rotation sample evaluates");
        assert!((actual - expected_body).length() < 1.0e-12);
    }
}
