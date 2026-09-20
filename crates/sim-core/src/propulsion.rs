//! Procedural propulsion backend (`docs/details/04`).
//!
//! Juno-style simple-mode authoring (engine preset + size / throat /
//! chamber-pressure / nozzle sliders) compiled once in the hangar into
//! validated runtime data. The same representation feeds the runtime thrust
//! model, the editor altitude analyzer (Juno's Performance Analyzer
//! equivalent), the vehicle mass budget, and the plume renderer input.
//!
//! Physics core is ideal frozen-flow isentropic nozzle theory (Sutton):
//! characteristic velocity from chamber thermo, thrust coefficient from
//! expansion ratio and ambient pressure, mass flow from throat area. Named
//! engine cycles are feed-system topologies with pressure-rise capability,
//! bypass bookkeeping, and mass/power consequences — not thrust multipliers.
//! Propellant pairs carry chamber temperature, ratio of specific heats, gas
//! constant, bulk density, and characteristic length; performance is derived
//! from those properties, never looked up by fuel name.
//!
//! First-iteration scope: liquid chemical rockets + solid motors with
//! conical/bell nozzles. Mixture-ratio sensitivity, aerospikes, tank/pump
//! detailed components, and the thermal-graph hookup are deferred (see
//! `docs/details/04` section 18).

use std::{error::Error, fmt};

use serde::{Deserialize, Serialize};

use crate::atmosphere::AtmosphereConfig;

/// Standard gravity for specific-impulse bookkeeping (m/s^2).
pub const STANDARD_GRAVITY_MPS2: f64 = 9.80665;

/// Summerfield flow-separation criterion: a nozzle whose exit pressure falls
/// below this fraction of ambient pressure risks asymmetric separation. The
/// backend reports the flag; it never silently derates thrust.
pub const SEPARATION_PRESSURE_RATIO: f64 = 0.4;

/// Nozzle velocity efficiency covering boundary-layer, kinetic, and
/// divergence-residual losses outside the geometric divergence factor.
/// Calibration, not law: the Merlin-1D golden test pins the full chain
/// within a 5% band against published sea-level/vacuum data.
pub const NOZZLE_EFFICIENCY: f64 = 0.95;

/// Gas-generator duct expansion ratio (fixed duct geometry, documented).
const GG_DUCT_EXPANSION_RATIO: f64 = 3.0;
/// Gas-generator turbine exhaust pressure as a fraction of chamber pressure
/// (post-turbine, documented assumption).
const GG_PRESSURE_FRACTION: f64 = 0.6;
/// Injector pressure drop as a fraction of chamber pressure (feed sizing).
const INJECTOR_DROP_FRACTION: f64 = 0.2;
/// Tank pressure assumed for electric-pump power bookkeeping (Pa).
const TANK_PRESSURE_PA: f64 = 300_000.0;
/// Pump hydraulic efficiency for electric-pump power (documented).
const PUMP_EFFICIENCY: f64 = 0.6;
/// Electric pump specific power for mass bookkeeping (W/kg, documented fit).
const PUMP_SPECIFIC_POWER_W_PER_KG: f64 = 5_000.0;
/// Structural safety factor on thin-wall pressure sizing.
const PRESSURE_SAFETY_FACTOR: f64 = 1.5;
/// Chamber length-to-diameter ratio (documented; the contraction ratio then
/// follows from the solved chamber volume instead of being asserted).
const CHAMBER_LENGTH_DIAMETER: f64 = 1.2;

/// Mass-fit: turbomachinery mass per unit flow power (kg per (kg/s * Pa)).
/// Calibration fit; the golden test pins a reference engine inside a wide
/// published-data band rather than at a point.
const MASS_FIT_TURBO_KG_PER_FLOW_POWER: f64 = 4.0e-8;
/// Mass-fit: mount/thrust-structure interface per unit vacuum thrust (kg/N).
const MASS_FIT_MOUNT_KG_PER_N: f64 = 8.0e-5;
/// Mass-fit: valves/lines/controller per unit vacuum thrust (kg/N).
const MASS_FIT_FEED_KG_PER_N: f64 = 3.0e-5;
/// Mass-fit: injector head base mass (kg) plus per-throat-area term.
const MASS_FIT_HEAD_BASE_KG: f64 = 8.0;
const MASS_FIT_HEAD_KG_PER_M2: f64 = 150.0;
/// Mass-fit: gimbal actuator base plus per-thrust term (kg, kg/N).
const MASS_FIT_GIMBAL_BASE_KG: f64 = 4.0;
const MASS_FIT_GIMBAL_KG_PER_N: f64 = 2.0e-5;
/// Solid igniter + hardware flat allowance (kg, documented fit).
const MASS_FIT_SOLID_IGNITER_KG: f64 = 2.0;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Propulsion authoring/compile/runtime errors.
#[derive(Debug, Clone, PartialEq)]
pub enum PropulsionError {
    /// Authoring value outside its physical/engineering domain.
    InvalidSpec(String),
    /// Cycle/material/cooling combination cannot support the design point.
    UnsupportedCombination(String),
    /// Runtime command the engine cannot execute (solid throttle, burnout).
    InvalidCommand(String),
}

impl fmt::Display for PropulsionError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSpec(message) => write!(formatter, "invalid engine spec: {message}"),
            Self::UnsupportedCombination(message) => {
                write!(formatter, "unsupported engine combination: {message}")
            }
            Self::InvalidCommand(message) => {
                write!(formatter, "invalid engine command: {message}")
            }
        }
    }
}

impl Error for PropulsionError {}

// Validity guards use `!(x > 0.0)` so NaN fails closed (repo convention).
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn require_positive(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !(value > 0.0) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite and > 0"
        )));
    }
    Ok(value)
}

#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn require_non_negative(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !(value >= 0.0) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite and >= 0"
        )));
    }
    Ok(value)
}

fn require_unit_interval(value: f64, name: &str) -> Result<f64, PropulsionError> {
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(PropulsionError::InvalidSpec(format!(
            "{name} must be finite in [0, 1]"
        )));
    }
    Ok(value)
}

// ---------------------------------------------------------------------------
// Propellants
// ---------------------------------------------------------------------------

/// Chemical propellant pairs. Each carries reference chamber thermo at its
/// documented design mixture ratio; v1 fixes the design point per pair and
/// mixture-ratio sensitivity is deferred (see module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Propellant {
    /// LOX/RP-1, reference oxidizer-to-fuel ratio ~2.7.
    LoxRp1,
    /// LOX/methane, reference ratio ~3.5.
    LoxMethane,
    /// LOX/hydrogen, reference ratio ~6.0.
    LoxHydrogen,
    /// NTO/MMH storable hypergolic, reference ratio ~1.65.
    NtoMmh,
    /// Ammonium-perchlorate composite solid propellant.
    SolidApcp,
}

/// Reference chamber thermo for a propellant pair (Sutton-typical values;
/// the `characteristic_velocity` calibration test pins each against
/// published c* within 4%).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PropellantThermo {
    /// Ratio of specific heats of the chamber products.
    pub gamma: f64,
    /// Chamber (adiabatic flame) temperature at the reference ratio (K).
    pub chamber_temp_k: f64,
    /// Specific gas constant of the products (J/kg/K).
    pub gas_constant_j_kg_k: f64,
    /// Bulk/storable density for feed-power bookkeeping (kg/m^3).
    pub bulk_density_kg_m3: f64,
    /// Characteristic chamber length Vc/At (m).
    pub characteristic_length_m: f64,
    /// Published characteristic velocity used as the calibration anchor.
    pub reference_c_star_mps: f64,
}

impl Propellant {
    /// Reference thermo for the pair.
    pub fn thermo(self) -> PropellantThermo {
        match self {
            Self::LoxRp1 => PropellantThermo {
                gamma: 1.24,
                chamber_temp_k: 3670.0,
                gas_constant_j_kg_k: 378.0,
                bulk_density_kg_m3: 1030.0,
                characteristic_length_m: 1.1,
                reference_c_star_mps: 1770.0,
            },
            Self::LoxMethane => PropellantThermo {
                gamma: 1.22,
                chamber_temp_k: 3680.0,
                gas_constant_j_kg_k: 405.0,
                bulk_density_kg_m3: 830.0,
                characteristic_length_m: 1.0,
                reference_c_star_mps: 1830.0,
            },
            Self::LoxHydrogen => PropellantThermo {
                gamma: 1.22,
                chamber_temp_k: 3560.0,
                gas_constant_j_kg_k: 616.0,
                bulk_density_kg_m3: 360.0,
                characteristic_length_m: 0.8,
                reference_c_star_mps: 2300.0,
            },
            Self::NtoMmh => PropellantThermo {
                gamma: 1.25,
                chamber_temp_k: 3400.0,
                gas_constant_j_kg_k: 380.0,
                bulk_density_kg_m3: 1190.0,
                characteristic_length_m: 0.9,
                reference_c_star_mps: 1700.0,
            },
            Self::SolidApcp => PropellantThermo {
                gamma: 1.20,
                chamber_temp_k: 3400.0,
                gas_constant_j_kg_k: 300.0,
                bulk_density_kg_m3: 1770.0,
                characteristic_length_m: 0.0,
                reference_c_star_mps: 1520.0,
            },
        }
    }

    /// True for the solid grain path (grain geometry instead of feed).
    pub fn is_solid(self) -> bool {
        matches!(self, Self::SolidApcp)
    }

    /// Reference (design) oxidizer-to-fuel ratio for the pair.
    pub fn reference_mixture_ratio(self) -> Option<f64> {
        match self {
            Self::LoxRp1 => Some(2.7),
            Self::LoxMethane => Some(3.5),
            Self::LoxHydrogen => Some(6.0),
            Self::NtoMmh => Some(1.65),
            Self::SolidApcp => None,
        }
    }

    /// Mixture table: (oxidizer-to-fuel ratio, chamber temp K, gamma, gas
    /// constant J/kg/K). Representative CEA-trend values bracketing the
    /// reference point; refine with project CEA runs. The middle row always
    /// reproduces [`Propellant::thermo`] exactly (pinned by test).
    fn mixture_table(self) -> Option<&'static [(f64, f64, f64, f64)]> {
        match self {
            Self::LoxRp1 => Some(&[
                (2.0, 3450.0, 1.25, 360.0),
                (2.7, 3670.0, 1.24, 378.0),
                (3.4, 3520.0, 1.23, 390.0),
            ]),
            Self::LoxMethane => Some(&[
                (2.8, 3500.0, 1.23, 430.0),
                (3.5, 3680.0, 1.22, 405.0),
                (4.2, 3600.0, 1.21, 385.0),
            ]),
            Self::LoxHydrogen => Some(&[
                (4.5, 3300.0, 1.24, 700.0),
                (6.0, 3560.0, 1.22, 616.0),
                (7.5, 3650.0, 1.20, 540.0),
            ]),
            Self::NtoMmh => Some(&[
                (1.30, 3200.0, 1.26, 400.0),
                (1.65, 3400.0, 1.25, 380.0),
                (2.00, 3450.0, 1.24, 365.0),
            ]),
            Self::SolidApcp => None,
        }
    }

    /// Chamber thermo at an oxidizer-to-fuel ratio: piecewise-linear
    /// interpolation inside the mixture table, hard refusal outside it.
    /// `None` selects the reference ratio.
    pub fn thermo_at_mixture(
        self,
        mixture_ratio: Option<f64>,
    ) -> Result<PropellantThermo, PropulsionError> {
        let reference = self.thermo();
        let Some(table) = self.mixture_table() else {
            if mixture_ratio.is_some() {
                return Err(PropulsionError::InvalidSpec(
                    "solid grain chemistry is fixed; no mixture knob".into(),
                ));
            }
            return Ok(reference);
        };
        let Some(ratio) = mixture_ratio else {
            return Ok(reference);
        };
        if !ratio.is_finite() {
            return Err(PropulsionError::InvalidSpec(
                "mixture ratio must be finite".into(),
            ));
        }
        if ratio < table.first().expect("non-empty table").0
            || ratio > table.last().expect("non-empty table").0
        {
            return Err(PropulsionError::InvalidSpec(format!(
                "mixture ratio {ratio} outside the modeled range [{}, {}]",
                table.first().expect("non-empty table").0,
                table.last().expect("non-empty table").0
            )));
        }
        for window in table.windows(2) {
            let (low, high) = (window[0], window[1]);
            if ratio <= high.0 {
                let span = (high.0 - low.0).max(1e-12);
                let fraction = (ratio - low.0) / span;
                return Ok(PropellantThermo {
                    gamma: low.2 + (high.2 - low.2) * fraction,
                    chamber_temp_k: low.1 + (high.1 - low.1) * fraction,
                    gas_constant_j_kg_k: low.3 + (high.3 - low.3) * fraction,
                    ..reference
                });
            }
        }
        Ok(reference)
    }
}

// ---------------------------------------------------------------------------
// Isentropic nozzle kernel (Sutton ideal frozen flow)
// ---------------------------------------------------------------------------

/// Characteristic velocity c* = sqrt(R*Tc) / Gamma (m/s).
pub fn characteristic_velocity(thermo: &PropellantThermo) -> f64 {
    let gamma = thermo.gamma;
    let gamma_fn = gamma.sqrt() * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)));
    (thermo.gas_constant_j_kg_k * thermo.chamber_temp_k).sqrt() / gamma_fn
}

/// Supersonic area ratio A/A* for a Mach number (isentropic).
fn area_ratio_from_mach(gamma: f64, mach: f64) -> f64 {
    let term = (2.0 / (gamma + 1.0)) * (1.0 + (gamma - 1.0) / 2.0 * mach * mach);
    term.powf((gamma + 1.0) / (2.0 * (gamma - 1.0))) / mach
}

/// Supersonic exit Mach for an expansion ratio. Newton iteration on the log
/// residual with a bisection fallback; deterministic, no tables.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
pub fn mach_from_area_ratio(gamma: f64, expansion_ratio: f64) -> Result<f64, PropulsionError> {
    if !(gamma > 1.0) || !(expansion_ratio >= 1.0) {
        return Err(PropulsionError::InvalidSpec(
            "gamma must be > 1 and expansion ratio >= 1".into(),
        ));
    }
    if expansion_ratio == 1.0 {
        return Ok(1.0);
    }
    let target = expansion_ratio.ln();
    let mut mach = (2.0 + expansion_ratio.ln()).max(1.5);
    for _ in 0..64 {
        let residual = area_ratio_from_mach(gamma, mach).ln() - target;
        if residual.abs() < 1e-13 {
            return Ok(mach);
        }
        let step = 1e-6 * mach.max(1.0);
        let slope = (area_ratio_from_mach(gamma, mach + step).ln()
            - area_ratio_from_mach(gamma, mach - step).ln())
            / (2.0 * step);
        if !slope.is_finite() || slope == 0.0 {
            break;
        }
        let next = mach - residual / slope;
        if !next.is_finite() || next <= 1.0 {
            break;
        }
        mach = next;
    }
    // Bisection fallback on a bracket that always contains the root for
    // finite expansion ratios.
    let mut low = 1.0_f64;
    let mut high = 2.0_f64;
    while area_ratio_from_mach(gamma, high).ln() < target {
        high *= 2.0;
        if high > 1.0e6 {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio out of range".into(),
            ));
        }
    }
    for _ in 0..200 {
        let mid = 0.5 * (low + high);
        if area_ratio_from_mach(gamma, mid).ln() < target {
            low = mid;
        } else {
            high = mid;
        }
    }
    Ok(0.5 * (low + high))
}

/// Exit-plane state for an expansion ratio at chamber conditions.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NozzleExitState {
    pub exit_mach: f64,
    /// Exit-to-chamber pressure ratio.
    pub exit_pressure_ratio: f64,
    /// Exit static temperature (K).
    pub exit_temp_k: f64,
    /// Exhaust velocity at full expansion (m/s).
    pub exhaust_velocity_mps: f64,
}

fn nozzle_exit(
    thermo: &PropellantThermo,
    expansion_ratio: f64,
) -> Result<NozzleExitState, PropulsionError> {
    let exit_mach = mach_from_area_ratio(thermo.gamma, expansion_ratio)?;
    let exit_pressure_ratio = (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach)
        .powf(-thermo.gamma / (thermo.gamma - 1.0));
    let exit_temp_k =
        thermo.chamber_temp_k / (1.0 + (thermo.gamma - 1.0) / 2.0 * exit_mach * exit_mach);
    let exhaust_velocity_mps =
        exit_mach * (thermo.gamma * thermo.gas_constant_j_kg_k * exit_temp_k).sqrt();
    Ok(NozzleExitState {
        exit_mach,
        exit_pressure_ratio,
        exit_temp_k,
        exhaust_velocity_mps,
    })
}

/// Ideal thrust coefficient for chamber pressure `chamber_pa`, exit state,
/// expansion ratio, ambient pressure, and geometric divergence factor.
pub fn thrust_coefficient(
    thermo: &PropellantThermo,
    chamber_pa: f64,
    exit: &NozzleExitState,
    expansion_ratio: f64,
    ambient_pa: f64,
    divergence_factor: f64,
) -> f64 {
    let gamma = thermo.gamma;
    let momentum = divergence_factor
        * NOZZLE_EFFICIENCY
        * ((2.0 * gamma * gamma / (gamma - 1.0))
            * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (gamma - 1.0))
            * (1.0 - exit.exit_pressure_ratio.powf((gamma - 1.0) / gamma)))
        .sqrt();
    let exit_pa = exit.exit_pressure_ratio * chamber_pa;
    momentum + (exit_pa - ambient_pa) / chamber_pa * expansion_ratio
}

// ---------------------------------------------------------------------------
// Cycles, materials, nozzles
// ---------------------------------------------------------------------------

/// Liquid feed/power cycle topology. Each variant gates attainable chamber
/// pressure and carries its own flow bookkeeping; none multiplies thrust.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EngineCycle {
    PressureFed,
    ElectricPump,
    GasGenerator,
    StagedCombustion,
    FullFlowStaged,
}

/// Engineering bounds + flow bookkeeping for a cycle (documented typical
/// demonstrated ranges, generous rather than balance-tuned).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CycleLimits {
    /// Maximum chamber pressure the feed system can sustain (Pa).
    pub max_chamber_pressure_pa: f64,
    /// Gas-generator bypass as a fraction of chamber flow (0 = closed).
    pub gg_bypass_fraction: f64,
    /// Gas-generator duct temperature (K, 0 when no GG duct).
    pub gg_temperature_k: f64,
    /// Deep-throttle combustion-stability floor (throttle fraction).
    pub min_throttle: f64,
    /// Spool/valve first-order time constant (s).
    pub spool_tau_s: f64,
}

impl EngineCycle {
    /// Cycle bounds.
    pub fn limits(self) -> CycleLimits {
        match self {
            Self::PressureFed => CycleLimits {
                max_chamber_pressure_pa: 3.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.30,
                spool_tau_s: 0.4,
            },
            Self::ElectricPump => CycleLimits {
                max_chamber_pressure_pa: 12.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.35,
                spool_tau_s: 0.3,
            },
            Self::GasGenerator => CycleLimits {
                max_chamber_pressure_pa: 21.0e6,
                gg_bypass_fraction: 0.030,
                gg_temperature_k: 1050.0,
                min_throttle: 0.40,
                spool_tau_s: 0.6,
            },
            Self::StagedCombustion => CycleLimits {
                max_chamber_pressure_pa: 30.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.50,
                spool_tau_s: 1.0,
            },
            Self::FullFlowStaged => CycleLimits {
                max_chamber_pressure_pa: 35.0e6,
                gg_bypass_fraction: 0.0,
                gg_temperature_k: 0.0,
                min_throttle: 0.50,
                spool_tau_s: 1.2,
            },
        }
    }
}

/// Chamber/nozzle structural material: density and strength size the walls,
/// temperature gates the cooling mode. Tier labels are forbidden; only
/// physical properties travel here.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ChamberMaterial {
    pub density_kg_m3: f64,
    /// Yield strength for thin-wall sizing (Pa).
    pub yield_strength_pa: f64,
    /// Maximum service wall temperature (K).
    pub max_wall_temp_k: f64,
}

impl ChamberMaterial {
    /// Regeneratively cooled copper-alloy liner class (high conductivity,
    /// fuel-cooled; Mach-diamond-era workhorse assumption).
    pub fn regen_alloy() -> Self {
        Self {
            density_kg_m3: 8900.0,
            yield_strength_pa: 300.0e6,
            max_wall_temp_k: 900.0,
        }
    }

    /// Nickel-superalloy class for high-pressure chambers.
    pub fn nickel_superalloy() -> Self {
        Self {
            density_kg_m3: 8190.0,
            yield_strength_pa: 1000.0e6,
            max_wall_temp_k: 1350.0,
        }
    }

    /// Radiative niobium-alloy class (low strength, high temperature).
    pub fn radiative_niobium() -> Self {
        Self {
            density_kg_m3: 8570.0,
            yield_strength_pa: 300.0e6,
            max_wall_temp_k: 1750.0,
        }
    }

    /// Ablative silica/phenolic class (sacrificial liner, duration-limited).
    pub fn ablative() -> Self {
        Self {
            density_kg_m3: 1800.0,
            yield_strength_pa: 100.0e6,
            max_wall_temp_k: 1800.0,
        }
    }

    pub(crate) fn validate(self) -> Result<(), PropulsionError> {
        require_positive(self.density_kg_m3, "material density")?;
        require_positive(self.yield_strength_pa, "material yield strength")?;
        require_positive(self.max_wall_temp_k, "material max wall temperature")?;
        Ok(())
    }
}

/// Chamber cooling topology: gates attainable pressure/duration physically.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CoolingMode {
    /// Fuel-cooled jacket: full cycle pressure allowed.
    Regenerative,
    /// Wall radiates: heat-flux bound caps chamber pressure at 8 MPa.
    Radiative,
    /// Sacrificial liner: single burns capped at 300 s (throat erosion
    /// beyond that is not modeled, so the compiler refuses instead).
    Ablative,
}

impl CoolingMode {
    /// Maximum cumulative single-burn duration in seconds (`None` =
    /// unlimited by cooling).
    pub fn max_single_burn_s(self) -> Option<f64> {
        match self {
            Self::Regenerative | Self::Radiative => None,
            Self::Ablative => Some(300.0),
        }
    }

    /// Radiative heat-flux pressure cap (Pa, `None` = no cooling cap).
    pub fn pressure_cap_pa(self) -> Option<f64> {
        match self {
            Self::Regenerative | Self::Ablative => None,
            Self::Radiative => Some(8.0e6),
        }
    }

    /// Relative nozzle wall thickness factor (documented: cooled walls run
    /// thin, radiative needs section, ablative carries a liner).
    fn nozzle_wall_factor(self) -> f64 {
        match self {
            Self::Regenerative => 0.5,
            Self::Radiative => 1.2,
            Self::Ablative => 2.0,
        }
    }
}

/// Nozzle contour family. Bell/cone derive the divergence factor from wall
/// geometry; the bell recovers half the residual divergence loss
/// (documented engineering approximation, thrust envelope +/-1% pinned
/// by test). The aerospike is altitude-compensating: the free jet boundary
/// adapts to ambient, so the pressure term never goes overexpanded-wide;
/// ambient eats only the documented base area (base drag).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum NozzleContour {
    Conical,
    Bell,
    Aerospike,
}

/// Aerospike base area as a fraction of equivalent exit area (base drag,
/// documented).
const AEROSPIKE_BASE_FRACTION: f64 = 0.05;

// ---------------------------------------------------------------------------
// Liquid engine spec + compilation
// ---------------------------------------------------------------------------

/// Juno-simple-mode authoring for one liquid chamber/nozzle assembly.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiquidEngineSpec {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    /// Design chamber pressure (Pa).
    pub chamber_pressure_pa: f64,
    /// Nozzle throat radius (m).
    pub throat_radius_m: f64,
    /// Nozzle expansion ratio Ae/At.
    pub expansion_ratio: f64,
    /// Nozzle length throat-to-exit (m); sets the wall angle and mass.
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub chamber_material: ChamberMaterial,
    pub cooling: CoolingMode,
    /// Oxidizer-to-fuel ratio override; `None` = pair reference.
    pub mixture_ratio: Option<f64>,
    /// Characteristic length override (m); `None` = propellant reference.
    pub characteristic_length_m: Option<f64>,
    /// Gimbal half-range (rad, 0 = fixed).
    pub gimbal_range_rad: f64,
    /// Minimum stable throttle; `None` = cycle floor.
    pub min_throttle: Option<f64>,
    /// Restartable after shutdown (ignition shots unlimited).
    pub restartable: bool,
}

impl LiquidEngineSpec {
    /// Validate the authoring values (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine name must not be empty".into(),
            ));
        }
        if self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "solid propellant needs a grain spec, not a liquid spec".into(),
            ));
        }
        require_positive(self.chamber_pressure_pa, "chamber pressure")?;
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.chamber_material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if self.gimbal_range_rad > 0.35 {
            return Err(PropulsionError::InvalidSpec(
                "gimbal range above 0.35 rad needs an explicit actuator design".into(),
            ));
        }
        if let Some(min_throttle) = self.min_throttle {
            require_unit_interval(min_throttle, "min throttle")?;
            if min_throttle < self.cycle.limits().min_throttle {
                return Err(PropulsionError::UnsupportedCombination(format!(
                    "{:?} cannot run stably below throttle {:.2}",
                    self.cycle,
                    self.cycle.limits().min_throttle
                )));
            }
        }
        if let Some(length) = self.characteristic_length_m {
            require_positive(length, "characteristic length")?;
        }
        // Gates the mixture knob early (range refusal lives here too).
        self.propellant.thermo_at_mixture(self.mixture_ratio)?;
        Ok(())
    }

    /// Hangar compile: validate topology, solve the design point, derive
    /// mass/power/thermal interfaces.
    pub fn compile(&self) -> Result<CompiledLiquid, PropulsionError> {
        self.validate()?;
        let limits = self.cycle.limits();
        if self.chamber_pressure_pa > limits.max_chamber_pressure_pa {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "{:?} feed cannot sustain {:.2} MPa (cap {:.1} MPa)",
                self.cycle,
                self.chamber_pressure_pa / 1.0e6,
                limits.max_chamber_pressure_pa / 1.0e6
            )));
        }
        if let Some(cap) = self.cooling.pressure_cap_pa()
            && self.chamber_pressure_pa > cap
        {
            return Err(PropulsionError::UnsupportedCombination(format!(
                "radiative cooling cannot reject the heat flux at {:.2} MPa (cap {:.1} MPa)",
                self.chamber_pressure_pa / 1.0e6,
                cap / 1.0e6
            )));
        }
        let thermo = self.propellant.thermo_at_mixture(self.mixture_ratio)?;
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit_area_m2 = throat_area_m2 * self.expansion_ratio;
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;

        // Wall angle from real geometry; divergence from the angle.
        // The aerospike exhaust leaves near-axially along the plug.
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let wall_angle_rad = wall_slope.atan();
        let conical_divergence = 0.5 * (1.0 + wall_angle_rad.cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical_divergence,
            NozzleContour::Bell => conical_divergence + (1.0 - conical_divergence) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };

        // Main chamber flow + optional gas-generator duct (same isentropic
        // core at duct temperature/expansion; bypass is a flow split, not a
        // multiplier).
        let chamber_flow_kg_s = self.chamber_pressure_pa * throat_area_m2 / c_star;
        let gg_flow_kg_s = limits.gg_bypass_fraction * chamber_flow_kg_s;
        let total_flow_kg_s = chamber_flow_kg_s + gg_flow_kg_s;
        let main = OperatingDesign {
            thrust_sl_n: thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                self.expansion_ratio,
                101_325.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2,
            thrust_vac_n: thrust_coefficient(
                &thermo,
                self.chamber_pressure_pa,
                &exit,
                self.expansion_ratio,
                0.0,
                divergence,
            ) * self.chamber_pressure_pa
                * throat_area_m2,
        };
        let (gg_thrust_sl_n, gg_thrust_vac_n, gg_exit_area_m2, gg_isp_s) = if gg_flow_kg_s > 0.0 {
            let gg_thermo = PropellantThermo {
                chamber_temp_k: limits.gg_temperature_k,
                ..thermo
            };
            let gg_c_star = characteristic_velocity(&gg_thermo);
            let gg_chamber_pa = GG_PRESSURE_FRACTION * self.chamber_pressure_pa;
            let gg_throat_m2 = gg_flow_kg_s * gg_c_star / gg_chamber_pa;
            let gg_exit = nozzle_exit(&gg_thermo, GG_DUCT_EXPANSION_RATIO)?;
            let sl = thrust_coefficient(
                &gg_thermo,
                gg_chamber_pa,
                &gg_exit,
                GG_DUCT_EXPANSION_RATIO,
                101_325.0,
                1.0,
            ) * gg_chamber_pa
                * gg_throat_m2;
            let vac = thrust_coefficient(
                &gg_thermo,
                gg_chamber_pa,
                &gg_exit,
                GG_DUCT_EXPANSION_RATIO,
                0.0,
                1.0,
            ) * gg_chamber_pa
                * gg_throat_m2;
            let isp = (sl + vac) * 0.5 / (gg_flow_kg_s * STANDARD_GRAVITY_MPS2);
            (sl, vac, gg_throat_m2 * GG_DUCT_EXPANSION_RATIO, isp)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };
        let thrust_sl_n = main.thrust_sl_n + gg_thrust_sl_n;
        let thrust_vac_n = main.thrust_vac_n + gg_thrust_vac_n;
        let isp_sl_s = thrust_sl_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);
        let isp_vac_s = thrust_vac_n / (total_flow_kg_s * STANDARD_GRAVITY_MPS2);

        // Mass from geometry + material + documented fits.
        let l_star = self
            .characteristic_length_m
            .unwrap_or(thermo.characteristic_length_m);
        let chamber_volume_m3 = l_star * throat_area_m2;
        let chamber_diameter_m = (4.0 * chamber_volume_m3
            / (std::f64::consts::PI * CHAMBER_LENGTH_DIAMETER))
            .powf(1.0 / 3.0);
        let chamber_length_m = CHAMBER_LENGTH_DIAMETER * chamber_diameter_m;
        let wall_thickness_m = self.chamber_pressure_pa * chamber_diameter_m
            / (2.0 * self.chamber_material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR;
        let chamber_wall_kg = std::f64::consts::PI
            * chamber_diameter_m
            * chamber_length_m
            * wall_thickness_m
            * self.chamber_material.density_kg_m3;
        let injector_kg = std::f64::consts::PI / 4.0
            * chamber_diameter_m
            * chamber_diameter_m
            * 0.02
            * self.chamber_material.density_kg_m3;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + wall_slope * wall_slope * self.nozzle_length_m * self.nozzle_length_m)
            .sqrt();
        let nozzle_area_m2 =
            std::f64::consts::PI * (self.throat_radius_m + exit_radius_m) * slant_m;
        let nozzle_thickness_m = wall_thickness_m
            * (self.throat_radius_m / chamber_diameter_m).min(1.0)
            * self.cooling.nozzle_wall_factor();
        // Aerospike: outer cowl frustum at 0.7x plus the plug cone itself
        // (base 0.6x exit radius over the spike length, same wall gauge).
        let spike_area_m2 = match self.contour {
            NozzleContour::Aerospike => {
                let spike_base_m = 0.6 * exit_radius_m;
                let spike_slant_m = (self.nozzle_length_m * self.nozzle_length_m
                    + spike_base_m * spike_base_m)
                    .sqrt();
                std::f64::consts::PI * spike_base_m * spike_slant_m
            }
            _ => 0.0,
        };
        let cowl_area_m2 = match self.contour {
            NozzleContour::Aerospike => nozzle_area_m2 * 0.7,
            _ => nozzle_area_m2,
        };
        let mut nozzle_kg = cowl_area_m2 * nozzle_thickness_m * self.chamber_material.density_kg_m3
            + spike_area_m2 * nozzle_thickness_m * self.chamber_material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        let turbo_kg = match self.cycle {
            EngineCycle::GasGenerator
            | EngineCycle::StagedCombustion
            | EngineCycle::FullFlowStaged => {
                total_flow_kg_s * self.chamber_pressure_pa * MASS_FIT_TURBO_KG_PER_FLOW_POWER
            }
            EngineCycle::PressureFed | EngineCycle::ElectricPump => 0.0,
        };
        let pump_motor_kg = match self.cycle {
            EngineCycle::ElectricPump => {
                let pump_power_w = total_flow_kg_s
                    * (self.chamber_pressure_pa - TANK_PRESSURE_PA).max(0.0)
                    / (thermo.bulk_density_kg_m3 * PUMP_EFFICIENCY);
                pump_power_w / PUMP_SPECIFIC_POWER_W_PER_KG
            }
            _ => 0.0,
        };
        let head_kg = MASS_FIT_HEAD_BASE_KG + MASS_FIT_HEAD_KG_PER_M2 * throat_area_m2;
        let mount_kg = MASS_FIT_MOUNT_KG_PER_N * thrust_vac_n;
        let feed_kg = MASS_FIT_FEED_KG_PER_N * thrust_vac_n;
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * thrust_vac_n
        } else {
            0.0
        };
        let dry_mass_kg = chamber_wall_kg
            + injector_kg
            + nozzle_kg
            + turbo_kg
            + pump_motor_kg
            + head_kg
            + mount_kg
            + feed_kg
            + gimbal_kg;

        let pump_power_w = match self.cycle {
            EngineCycle::ElectricPump => {
                total_flow_kg_s * (self.chamber_pressure_pa - TANK_PRESSURE_PA).max(0.0)
                    / (thermo.bulk_density_kg_m3 * PUMP_EFFICIENCY)
            }
            _ => 0.0,
        };

        Ok(CompiledLiquid {
            name: self.name.clone(),
            propellant: self.propellant,
            cycle: self.cycle,
            chamber_pressure_pa: self.chamber_pressure_pa,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            exit_area_m2,
            nozzle_length_m: self.nozzle_length_m,
            divergence_factor: divergence,
            wall_angle_rad,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_pressure_pa: exit.exit_pressure_ratio * self.chamber_pressure_pa,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            gg_bypass_fraction: limits.gg_bypass_fraction,
            gg_thrust_sl_n,
            gg_thrust_vac_n,
            gg_exit_area_m2,
            gg_isp_s,
            full_flow_kg_s: total_flow_kg_s,
            thrust_sl_n,
            thrust_vac_n,
            isp_sl_s,
            isp_vac_s,
            dry_mass_kg,
            chamber_diameter_m,
            chamber_length_m,
            wall_thickness_m,
            nozzle_mass_kg: nozzle_kg,
            pump_power_w,
            feed_pressure_required_pa: self.chamber_pressure_pa * (1.0 + INJECTOR_DROP_FRACTION),
            min_throttle: self.min_throttle.unwrap_or(limits.min_throttle),
            spool_tau_s: limits.spool_tau_s,
            restartable: self.restartable,
            gimbal_range_rad: self.gimbal_range_rad,
            cooling: self.cooling,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => AEROSPIKE_BASE_FRACTION * exit_area_m2,
                _ => 0.0,
            },
            nozzle_wall_area_m2: cowl_area_m2 + spike_area_m2,
        })
    }
}

struct OperatingDesign {
    thrust_sl_n: f64,
    thrust_vac_n: f64,
}

// ---------------------------------------------------------------------------
// Solid motors
// ---------------------------------------------------------------------------

/// BATES cylindrical-port grain authoring (Juno fuel-grain equivalent).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidMotorSpec {
    pub name: String,
    /// Ammonium-perchlorate composite today; the grain solver reads thermo
    /// from the pair so future chemistries plug in without a new solver.
    pub propellant: Propellant,
    /// Grain outer radius (m).
    pub outer_radius_m: f64,
    /// Initial port (core) radius (m).
    pub core_radius_m: f64,
    /// Grain length per segment (m).
    pub segment_length_m: f64,
    /// Number of segments.
    pub segments: u32,
    /// Burn-rate coefficient a in r = a * Pc^n (m/s/Pa^n).
    pub burn_rate_coeff: f64,
    /// Burn-rate exponent n (must be < 1 for stable equilibrium).
    pub burn_rate_exponent: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub nozzle_length_m: f64,
    pub contour: NozzleContour,
    pub casing_material: ChamberMaterial,
    /// Inhibited segment ends (neutral trace); exposed ends burn too.
    pub inhibited_ends: bool,
    /// Per-segment initial port radii (m) for a stepped channel
    /// (boost-sustain shaping); `None` = uniform `core_radius_m`.
    pub segment_core_radii_m: Option<Vec<f64>>,
    pub gimbal_range_rad: f64,
    /// Ignition shots carried (solids are single-shot by default).
    pub ignition_shots: u32,
}

impl SolidMotorSpec {
    /// Reference APCP ballistics (documented typical: ~10 mm/s at 7 MPa).
    pub fn apcp_ballistics() -> (f64, f64) {
        (4.0e-5, 0.35)
    }

    /// Validate authoring values (NaN fails closed).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "motor name must not be empty".into(),
            ));
        }
        if !self.propellant.is_solid() {
            return Err(PropulsionError::InvalidSpec(
                "grain ballistics need a solid propellant".into(),
            ));
        }
        require_positive(self.outer_radius_m, "grain outer radius")?;
        require_positive(self.core_radius_m, "grain core radius")?;
        if !(self.core_radius_m < self.outer_radius_m) {
            return Err(PropulsionError::InvalidSpec(
                "grain core must be smaller than the outer radius".into(),
            ));
        }
        require_positive(self.segment_length_m, "grain segment length")?;
        if self.segments == 0 || self.segments > 16 {
            return Err(PropulsionError::InvalidSpec(
                "segments must be in 1..=16".into(),
            ));
        }
        require_positive(self.burn_rate_coeff, "burn-rate coefficient")?;
        if !(self.burn_rate_exponent > 0.0) || !(self.burn_rate_exponent < 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "burn-rate exponent must be in (0, 1): n >= 1 has no stable pressure equilibrium"
                    .into(),
            ));
        }
        require_positive(self.throat_radius_m, "throat radius")?;
        if !(self.expansion_ratio >= 1.0) {
            return Err(PropulsionError::InvalidSpec(
                "expansion ratio must be finite and >= 1".into(),
            ));
        }
        require_positive(self.nozzle_length_m, "nozzle length")?;
        self.casing_material.validate()?;
        require_non_negative(self.gimbal_range_rad, "gimbal range")?;
        if let Some(radii) = &self.segment_core_radii_m {
            if radii.len() != self.segments as usize {
                return Err(PropulsionError::InvalidSpec(
                    "segment port radii must match the segment count".into(),
                ));
            }
            for radius_m in radii {
                require_positive(*radius_m, "segment port radius")?;
                if !(*radius_m < self.outer_radius_m) {
                    return Err(PropulsionError::InvalidSpec(
                        "segment port must be smaller than the outer radius".into(),
                    ));
                }
            }
        }
        if self.ignition_shots == 0 {
            return Err(PropulsionError::InvalidSpec(
                "a solid motor with zero shots can never ignite".into(),
            ));
        }
        Ok(())
    }

    /// Hangar compile: solve the equilibrium burn trace over the web.
    pub fn compile(&self) -> Result<CompiledSolid, PropulsionError> {
        self.validate()?;
        let thermo = self.propellant.thermo();
        let c_star = characteristic_velocity(&thermo);
        let throat_area_m2 = std::f64::consts::PI * self.throat_radius_m * self.throat_radius_m;
        let exit_radius_m = self.throat_radius_m * self.expansion_ratio.sqrt();
        let exit = nozzle_exit(&thermo, self.expansion_ratio)?;
        let wall_slope = (exit_radius_m - self.throat_radius_m) / self.nozzle_length_m;
        let conical_divergence = 0.5 * (1.0 + wall_slope.atan().cos());
        let divergence = match self.contour {
            NozzleContour::Conical => conical_divergence,
            NozzleContour::Bell => conical_divergence + (1.0 - conical_divergence) * 0.5,
            NozzleContour::Aerospike => 0.99,
        };

        // Web-burn trace: time-stepped equilibrium. Each segment regresses
        // its own port (stepped channels shape boost-sustain traces); the
        // common chamber pressure couples them through total burn area:
        // Pc = [Ab(t) a rho c* / At]^(1/(1-n)). Sliver after burn-through
        // is ignored (documented).
        let density = thermo.bulk_density_kg_m3;
        let core_radii: Vec<f64> = match &self.segment_core_radii_m {
            Some(radii) => radii.clone(),
            None => vec![self.core_radius_m; self.segments as usize],
        };
        let webs: Vec<f64> = core_radii
            .iter()
            .map(|core_m| self.outer_radius_m - core_m)
            .collect();
        let burn_area_at = |burned: &[f64]| -> f64 {
            let mut area_m2 = 0.0;
            for (index, core_m) in core_radii.iter().enumerate() {
                if burned[index] >= webs[index] {
                    continue;
                }
                let port_m = core_m + burned[index];
                area_m2 += std::f64::consts::PI * 2.0 * port_m * self.segment_length_m;
                if !self.inhibited_ends {
                    area_m2 += 2.0
                        * std::f64::consts::PI
                        * (self.outer_radius_m * self.outer_radius_m - port_m * port_m);
                }
            }
            area_m2
        };
        let pressure_at = |area_m2: f64| -> Result<f64, PropulsionError> {
            let chamber_pa = (area_m2 * self.burn_rate_coeff * density * c_star / throat_area_m2)
                .powf(1.0 / (1.0 - self.burn_rate_exponent));
            if !chamber_pa.is_finite() || chamber_pa <= 0.0 {
                return Err(PropulsionError::InvalidSpec(
                    "grain equilibrium pressure is non-physical".into(),
                ));
            }
            Ok(chamber_pa)
        };
        let point_at = |time_s: f64,
                        burned_now: &[f64],
                        burned_max_m: f64|
         -> Result<BurnPoint, PropulsionError> {
            let burn_area_m2 = burn_area_at(burned_now);
            let chamber_pa = pressure_at(burn_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mass_flow_kg_s = density * burn_area_m2 * burn_rate_mps;
            let thrust_sl_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                101_325.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            let thrust_vac_n = thrust_coefficient(
                &thermo,
                chamber_pa,
                &exit,
                self.expansion_ratio,
                0.0,
                divergence,
            ) * chamber_pa
                * throat_area_m2;
            Ok(BurnPoint {
                time_s,
                web_burned_m: burned_max_m,
                chamber_pa,
                mass_flow_kg_s,
                thrust_sl_n,
                thrust_vac_n,
            })
        };
        let initial_area_m2 = burn_area_at(&vec![0.0; core_radii.len()]);
        let initial_rate_mps =
            self.burn_rate_coeff * pressure_at(initial_area_m2)?.powf(self.burn_rate_exponent);
        let max_web_m = webs.iter().cloned().fold(0.0_f64, f64::max);
        let dt_s = max_web_m / initial_rate_mps / 400.0;
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(PropulsionError::InvalidSpec(
                "grain step sizing is non-physical".into(),
            ));
        }
        let mut burned = vec![0.0; core_radii.len()];
        let mut time_s = 0.0;
        let mut points = vec![point_at(0.0, &burned, 0.0)?];
        let mut peak_pressure_pa = 0.0_f64;
        let mut peak_thrust_n = 0.0_f64;
        loop {
            let burn_area_m2 = burn_area_at(&burned);
            if burn_area_m2 <= 0.0 {
                break;
            }
            let chamber_pa = pressure_at(burn_area_m2)?;
            let burn_rate_mps = self.burn_rate_coeff * chamber_pa.powf(self.burn_rate_exponent);
            let mut all_done = true;
            let mut burned_max_m = 0.0_f64;
            for (index, web_m) in webs.iter().enumerate() {
                if burned[index] < *web_m {
                    burned[index] = (burned[index] + burn_rate_mps * dt_s).min(*web_m);
                }
                burned_max_m = burned_max_m.max(burned[index]);
                if burned[index] < *web_m {
                    all_done = false;
                }
            }
            time_s += dt_s;
            // Burn-through: the trace terminates at zero thrust rather
            // than erroring on the empty grain.
            let point = if burn_area_at(&burned) <= 0.0 {
                BurnPoint {
                    time_s,
                    web_burned_m: burned_max_m,
                    chamber_pa: 0.0,
                    mass_flow_kg_s: 0.0,
                    thrust_sl_n: 0.0,
                    thrust_vac_n: 0.0,
                }
            } else {
                point_at(time_s, &burned, burned_max_m)?
            };
            peak_pressure_pa = peak_pressure_pa.max(point.chamber_pa);
            peak_thrust_n = peak_thrust_n.max(point.thrust_sl_n);
            points.push(point);
            if all_done {
                break;
            }
            if points.len() > 100_000 {
                return Err(PropulsionError::InvalidSpec(
                    "grain does not burn through".into(),
                ));
            }
        }
        peak_pressure_pa = peak_pressure_pa.max(points[0].chamber_pa);
        peak_thrust_n = peak_thrust_n.max(points[0].thrust_sl_n);
        // Integrate impulse and propellant over the recorded trace.
        let mut total_impulse_ns = 0.0;
        let mut propellant_kg = 0.0;
        for window in points.windows(2) {
            let (a, b) = (window[0], window[1]);
            let dt = b.time_s - a.time_s;
            total_impulse_ns += 0.5 * (a.thrust_vac_n + b.thrust_vac_n) * dt;
            propellant_kg += 0.5 * (a.mass_flow_kg_s + b.mass_flow_kg_s) * dt;
        }
        let burn_time_s = time_s;
        let avg_isp_s = total_impulse_ns / (propellant_kg * STANDARD_GRAVITY_MPS2);

        // Casing: cylinder + hemispherical-cap allowance + shared nozzle path.
        let grain_length_m = self.segment_length_m * self.segments as f64;
        let case_thickness_m = peak_pressure_pa * 2.0 * self.outer_radius_m
            / (2.0 * self.casing_material.yield_strength_pa)
            * PRESSURE_SAFETY_FACTOR;
        let case_kg = 2.0
            * std::f64::consts::PI
            * self.outer_radius_m
            * grain_length_m
            * case_thickness_m
            * self.casing_material.density_kg_m3
            * 1.3;
        let slant_m = (self.nozzle_length_m * self.nozzle_length_m
            + (exit_radius_m - self.throat_radius_m).powi(2))
        .sqrt();
        let mut nozzle_kg = std::f64::consts::PI
            * (self.throat_radius_m + exit_radius_m)
            * slant_m
            * case_thickness_m
            * 0.6
            * self.casing_material.density_kg_m3;
        if self.contour == NozzleContour::Bell {
            nozzle_kg *= 0.85;
        }
        if self.contour == NozzleContour::Aerospike {
            let spike_base_m = 0.6 * exit_radius_m;
            let spike_slant_m =
                (self.nozzle_length_m * self.nozzle_length_m + spike_base_m * spike_base_m).sqrt();
            nozzle_kg = nozzle_kg * 0.7
                + std::f64::consts::PI
                    * spike_base_m
                    * spike_slant_m
                    * case_thickness_m
                    * 0.6
                    * self.casing_material.density_kg_m3;
        }
        let gimbal_kg = if self.gimbal_range_rad > 0.0 {
            MASS_FIT_GIMBAL_BASE_KG + MASS_FIT_GIMBAL_KG_PER_N * peak_thrust_n
        } else {
            0.0
        };
        let dry_mass_kg = case_kg
            + nozzle_kg
            + gimbal_kg
            + MASS_FIT_SOLID_IGNITER_KG
            + MASS_FIT_MOUNT_KG_PER_N * peak_thrust_n
            + MASS_FIT_FEED_KG_PER_N * peak_thrust_n;

        Ok(CompiledSolid {
            name: self.name.clone(),
            propellant: self.propellant,
            throat_area_m2,
            throat_radius_m: self.throat_radius_m,
            expansion_ratio: self.expansion_ratio,
            exit_radius_m,
            divergence_factor: divergence,
            gamma: thermo.gamma,
            chamber_temp_k: thermo.chamber_temp_k,
            gas_constant_j_kg_k: thermo.gas_constant_j_kg_k,
            c_star_mps: c_star,
            exit_mach: exit.exit_mach,
            exit_temp_k: exit.exit_temp_k,
            exhaust_velocity_mps: exit.exhaust_velocity_mps,
            burn_curve: points,
            burn_time_s,
            total_impulse_ns,
            propellant_mass_kg: propellant_kg,
            avg_isp_s,
            peak_pressure_pa,
            peak_thrust_sl_n: peak_thrust_n,
            dry_mass_kg,
            grain_outer_radius_m: self.outer_radius_m,
            grain_length_m,
            ignition_shots: self.ignition_shots,
            gimbal_range_rad: self.gimbal_range_rad,
            contour: self.contour,
            aerospike_base_area_m2: match self.contour {
                NozzleContour::Aerospike => {
                    AEROSPIKE_BASE_FRACTION * throat_area_m2 * self.expansion_ratio
                }
                _ => 0.0,
            },
        })
    }
}

/// One web station of a compiled solid burn trace (sea-level + vacuum
/// thrust stored; altitude replay varies only the pressure term, which is
/// exact for a choked motor at frozen chamber pressure).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BurnPoint {
    pub time_s: f64,
    pub web_burned_m: f64,
    pub chamber_pa: f64,
    pub mass_flow_kg_s: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
}

// ---------------------------------------------------------------------------
// Compiled runtime data
// ---------------------------------------------------------------------------

/// Hangar-compiled liquid engine: design point solved, mass/power/thermal
/// interfaces derived. Runtime evaluation is a pure function of throttle
/// and ambient pressure.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledLiquid {
    pub name: String,
    pub propellant: Propellant,
    pub cycle: EngineCycle,
    pub chamber_pressure_pa: f64,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub exit_area_m2: f64,
    pub nozzle_length_m: f64,
    pub divergence_factor: f64,
    pub wall_angle_rad: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub gg_bypass_fraction: f64,
    pub gg_thrust_sl_n: f64,
    pub gg_thrust_vac_n: f64,
    pub gg_exit_area_m2: f64,
    pub gg_isp_s: f64,
    pub full_flow_kg_s: f64,
    pub thrust_sl_n: f64,
    pub thrust_vac_n: f64,
    pub isp_sl_s: f64,
    pub isp_vac_s: f64,
    pub dry_mass_kg: f64,
    pub chamber_diameter_m: f64,
    pub chamber_length_m: f64,
    pub wall_thickness_m: f64,
    pub nozzle_mass_kg: f64,
    pub pump_power_w: f64,
    pub feed_pressure_required_pa: f64,
    pub min_throttle: f64,
    pub spool_tau_s: f64,
    pub restartable: bool,
    pub gimbal_range_rad: f64,
    pub cooling: CoolingMode,
    pub contour: NozzleContour,
    /// Aerospike base area exposed to ambient (m^2, 0 for bell/cone).
    pub aerospike_base_area_m2: f64,
    /// Nozzle wall area for thermal/radiation bookkeeping (m^2).
    pub nozzle_wall_area_m2: f64,
}

/// Hangar-compiled solid motor with its equilibrium burn trace.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CompiledSolid {
    pub name: String,
    pub propellant: Propellant,
    pub throat_area_m2: f64,
    pub throat_radius_m: f64,
    pub expansion_ratio: f64,
    pub exit_radius_m: f64,
    pub divergence_factor: f64,
    pub gamma: f64,
    pub chamber_temp_k: f64,
    pub gas_constant_j_kg_k: f64,
    pub c_star_mps: f64,
    pub exit_mach: f64,
    pub exit_temp_k: f64,
    pub exhaust_velocity_mps: f64,
    pub burn_curve: Vec<BurnPoint>,
    pub burn_time_s: f64,
    pub total_impulse_ns: f64,
    pub propellant_mass_kg: f64,
    pub avg_isp_s: f64,
    pub peak_pressure_pa: f64,
    pub peak_thrust_sl_n: f64,
    pub dry_mass_kg: f64,
    pub grain_outer_radius_m: f64,
    pub grain_length_m: f64,
    pub ignition_shots: u32,
    pub gimbal_range_rad: f64,
    pub contour: NozzleContour,
    /// Aerospike base area exposed to ambient (m^2, 0 for bell/cone).
    pub aerospike_base_area_m2: f64,
}

/// Compiled engine, either family. One representation for simple-mode
/// presets and advanced component authoring (v1 ships the simple path).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CompiledEngine {
    Liquid(CompiledLiquid),
    Solid(CompiledSolid),
}

/// Instantaneous operating point: the single choke point between the
/// propulsion backend and thrust/mass-flow/plume consumers.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineOperatingPoint {
    pub thrust_n: f64,
    pub mass_flow_kg_s: f64,
    pub isp_s: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exit_mach: f64,
    /// True when Pe/Pa trips the Summerfield separation criterion.
    pub separation_risk: bool,
}

impl CompiledEngine {
    /// Engine display name.
    pub fn name(&self) -> &str {
        match self {
            Self::Liquid(engine) => &engine.name,
            Self::Solid(engine) => &engine.name,
        }
    }

    /// Dry (no propellant) mass in kg.
    pub fn dry_mass_kg(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.dry_mass_kg,
            Self::Solid(engine) => engine.dry_mass_kg,
        }
    }

    /// Mass to aggregate into the vehicle budget at bake time: dry mass
    /// plus full solid propellant (grain burns off in flight; depletion
    /// wiring into the runtime mass model is deferred). Liquid propellant
    /// lives in tank parts, which are future vehicle components.
    pub fn bake_mass_kg(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.dry_mass_kg,
            Self::Solid(engine) => engine.dry_mass_kg + engine.propellant_mass_kg,
        }
    }

    /// Full-throttle mass flow at the design point (kg/s).
    pub fn full_flow_kg_s(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.full_flow_kg_s,
            Self::Solid(engine) => engine
                .burn_curve
                .iter()
                .map(|point| point.mass_flow_kg_s)
                .fold(0.0_f64, f64::max),
        }
    }

    /// Allowed throttle interval (solids are fixed at full by physics).
    pub fn throttle_range(&self) -> (f64, f64) {
        match self {
            Self::Liquid(engine) => (engine.min_throttle, 1.0),
            Self::Solid(_) => (1.0, 1.0),
        }
    }

    /// Spool/valve time constant (s). Solids ignite ballistically: 0.
    pub fn spool_tau_s(&self) -> f64 {
        match self {
            Self::Liquid(engine) => engine.spool_tau_s,
            Self::Solid(_) => 0.0,
        }
    }

    /// Steady-state operating point at effective throttle and ambient
    /// pressure. Liquids scale chamber pressure linearly with throttle
    /// (documented deep-throttle assumption: combustion efficiency held
    /// constant; real variation of a few percent is not modeled). Solids
    /// ignore throttle (must be 1) and read the burn trace at `burn_time_s`.
    pub fn operating_point(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<EngineOperatingPoint, PropulsionError> {
        if !ambient_pa.is_finite() || ambient_pa < 0.0 {
            return Err(PropulsionError::InvalidCommand(
                "ambient pressure must be finite and >= 0".into(),
            ));
        }
        match self {
            Self::Liquid(engine) => {
                if !throttle.is_finite() || throttle < 0.0 || throttle > 1.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "throttle must be finite in [0, 1]".into(),
                    ));
                }
                if throttle == 0.0 {
                    return Ok(EngineOperatingPoint::off(engine));
                }
                if engine.contour == NozzleContour::Aerospike {
                    // Altitude compensation: the free jet boundary tracks
                    // ambient, so the design vacuum thrust survives at any
                    // altitude minus base drag on the plug base.
                    let flow_kg_s = engine.full_flow_kg_s * throttle;
                    let thrust_n = (engine.thrust_vac_n * throttle
                        - ambient_pa * engine.aerospike_base_area_m2)
                        .max(0.0);
                    let isp_s = thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2);
                    return Ok(EngineOperatingPoint {
                        thrust_n,
                        mass_flow_kg_s: flow_kg_s,
                        isp_s,
                        exhaust_velocity_mps: engine.exhaust_velocity_mps,
                        exit_pressure_pa: (engine.exit_pressure_pa * throttle).max(ambient_pa),
                        exit_temp_k: engine.exit_temp_k,
                        exit_mach: engine.exit_mach,
                        separation_risk: false,
                    });
                }
                let chamber_pa = engine.chamber_pressure_pa * throttle;
                let exit_pressure_pa = engine.exit_pressure_pa * throttle;
                let flow_kg_s = engine.full_flow_kg_s * throttle;
                // Main chamber (frozen exit Mach; pressure term exact).
                let thermo = engine.thermo_ref();
                let main_cf = thrust_coefficient(
                    &thermo,
                    chamber_pa,
                    &engine.exit_ref(),
                    engine.expansion_ratio,
                    ambient_pa,
                    engine.divergence_factor,
                );
                let mut thrust_n = main_cf * chamber_pa * engine.throat_area_m2;
                thrust_n += engine.gg_thrust_at(throttle, ambient_pa);
                let isp_s = thrust_n / (flow_kg_s * STANDARD_GRAVITY_MPS2);
                Ok(EngineOperatingPoint {
                    thrust_n,
                    mass_flow_kg_s: flow_kg_s,
                    isp_s,
                    exhaust_velocity_mps: engine.exhaust_velocity_mps,
                    exit_pressure_pa,
                    exit_temp_k: engine.exit_temp_k,
                    exit_mach: engine.exit_mach,
                    separation_risk: exit_pressure_pa < SEPARATION_PRESSURE_RATIO * ambient_pa,
                })
            }
            Self::Solid(engine) => {
                if throttle != 1.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "solid motors run at full thrust or off; partial throttle is not physical"
                            .into(),
                    ));
                }
                if !burn_time_s.is_finite() || burn_time_s < 0.0 {
                    return Err(PropulsionError::InvalidCommand(
                        "burn time must be finite and >= 0".into(),
                    ));
                }
                if burn_time_s >= engine.burn_time_s {
                    return Ok(EngineOperatingPoint::burned_out(engine));
                }
                let point = engine.interpolate(burn_time_s);
                if engine.contour == NozzleContour::Aerospike {
                    let thrust_n =
                        (point.thrust_vac_n - ambient_pa * engine.aerospike_base_area_m2).max(0.0);
                    return Ok(EngineOperatingPoint {
                        thrust_n,
                        mass_flow_kg_s: point.mass_flow_kg_s,
                        isp_s: thrust_n / (point.mass_flow_kg_s * STANDARD_GRAVITY_MPS2),
                        exhaust_velocity_mps: engine.exhaust_velocity_mps,
                        exit_pressure_pa: ambient_pa,
                        exit_temp_k: engine.exit_temp_k,
                        exit_mach: engine.exit_mach,
                        separation_risk: false,
                    });
                }
                let thermo = engine.thermo_ref();
                let cf = thrust_coefficient(
                    &thermo,
                    point.chamber_pa,
                    &engine.exit_state(),
                    engine.expansion_ratio,
                    ambient_pa,
                    engine.divergence_factor,
                );
                let thrust_n = cf * point.chamber_pa * engine.throat_area_m2;
                let exit_pressure_pa = point.chamber_pa * engine.exit_pressure_ratio();
                Ok(EngineOperatingPoint {
                    thrust_n,
                    mass_flow_kg_s: point.mass_flow_kg_s,
                    isp_s: thrust_n / (point.mass_flow_kg_s * STANDARD_GRAVITY_MPS2),
                    exhaust_velocity_mps: engine.exhaust_velocity_mps,
                    exit_pressure_pa,
                    exit_temp_k: engine.exit_temp_k,
                    exit_mach: engine.exit_mach,
                    separation_risk: exit_pressure_pa < SEPARATION_PRESSURE_RATIO * ambient_pa,
                })
            }
        }
    }

    /// Stagnation thermal power released at full throttle (W): mass flow
    /// times cp times chamber temperature. The future thermal graph
    /// consumes this; the nozzle converts part of it into exhaust kinetic
    /// power (see below). Liquids use design flow; solids the burn average.
    pub fn chamber_power_w(&self) -> f64 {
        match self {
            Self::Liquid(engine) => {
                let cp = engine.gamma * engine.gas_constant_j_kg_k / (engine.gamma - 1.0);
                engine.full_flow_kg_s * cp * engine.chamber_temp_k
            }
            Self::Solid(engine) => {
                let cp = engine.gamma * engine.gas_constant_j_kg_k / (engine.gamma - 1.0);
                let avg_flow_kg_s = engine.propellant_mass_kg / engine.burn_time_s;
                avg_flow_kg_s * cp * engine.chamber_temp_k
            }
        }
    }

    /// Exhaust kinetic power at full throttle in vacuum (W). Always below
    /// [`CompiledEngine::chamber_power_w`] (pinned by test): the difference
    /// is residual exhaust enthalpy plus (for GG cycles) duct losses.
    pub fn exhaust_kinetic_power_w(&self) -> f64 {
        match self {
            Self::Liquid(engine) => {
                0.5 * engine.full_flow_kg_s
                    * engine.exhaust_velocity_mps
                    * engine.exhaust_velocity_mps
            }
            Self::Solid(engine) => {
                let avg_flow_kg_s = engine.propellant_mass_kg / engine.burn_time_s;
                0.5 * avg_flow_kg_s * engine.exhaust_velocity_mps * engine.exhaust_velocity_mps
            }
        }
    }

    /// Solid propellant remaining at a burn clock (kg). Liquids return
    /// `None`: their propellant lives in tank parts (see `feed` module).
    pub fn propellant_remaining_kg(&self, burn_time_s: f64) -> Option<f64> {
        match self {
            Self::Liquid(_) => None,
            Self::Solid(engine) => {
                if !burn_time_s.is_finite() || burn_time_s <= 0.0 {
                    return Some(engine.propellant_mass_kg);
                }
                if burn_time_s >= engine.burn_time_s {
                    return Some(0.0);
                }
                let mut consumed_kg = 0.0;
                let mut prev = &engine.burn_curve[0];
                for point in engine.burn_curve.iter().skip(1) {
                    if point.time_s >= burn_time_s {
                        // Partial interval with linear flow: exact integral.
                        let span = (point.time_s - prev.time_s).max(1e-12);
                        let fraction = ((burn_time_s - prev.time_s) / span).clamp(0.0, 1.0);
                        let flow_now = prev.mass_flow_kg_s
                            + (point.mass_flow_kg_s - prev.mass_flow_kg_s) * fraction;
                        consumed_kg += 0.5
                            * (prev.mass_flow_kg_s + flow_now)
                            * (burn_time_s - prev.time_s).max(0.0);
                        break;
                    }
                    consumed_kg += 0.5
                        * (prev.mass_flow_kg_s + point.mass_flow_kg_s)
                        * (point.time_s - prev.time_s);
                    prev = point;
                }
                Some((engine.propellant_mass_kg - consumed_kg).max(0.0))
            }
        }
    }

    /// Plume-renderer input state for an operating point plus geometry.
    pub fn plume_state(&self, point: &EngineOperatingPoint) -> EnginePlumeState {
        match self {
            Self::Liquid(engine) => EnginePlumeState {
                exit_radius_m: engine.exit_radius_m,
                mass_flow_kg_s: point.mass_flow_kg_s,
                exhaust_velocity_mps: point.exhaust_velocity_mps,
                exit_pressure_pa: point.exit_pressure_pa,
                exit_temp_k: point.exit_temp_k,
                exit_mach: point.exit_mach,
                propellant: engine.propellant,
            },
            Self::Solid(engine) => EnginePlumeState {
                exit_radius_m: engine.exit_radius_m,
                mass_flow_kg_s: point.mass_flow_kg_s,
                exhaust_velocity_mps: point.exhaust_velocity_mps,
                exit_pressure_pa: point.exit_pressure_pa,
                exit_temp_k: point.exit_temp_k,
                exit_mach: point.exit_mach,
                propellant: engine.propellant,
            },
        }
    }
}

impl EngineOperatingPoint {
    fn off(engine: &CompiledLiquid) -> Self {
        Self {
            thrust_n: 0.0,
            mass_flow_kg_s: 0.0,
            isp_s: 0.0,
            exhaust_velocity_mps: engine.exhaust_velocity_mps,
            exit_pressure_pa: 0.0,
            exit_temp_k: engine.exit_temp_k,
            exit_mach: engine.exit_mach,
            separation_risk: false,
        }
    }

    fn burned_out(engine: &CompiledSolid) -> Self {
        Self {
            thrust_n: 0.0,
            mass_flow_kg_s: 0.0,
            isp_s: 0.0,
            exhaust_velocity_mps: engine.exhaust_velocity_mps,
            exit_pressure_pa: 0.0,
            exit_temp_k: engine.exit_temp_k,
            exit_mach: engine.exit_mach,
            separation_risk: false,
        }
    }
}

// Reconstructed thermo/exit views for runtime evaluation (no re-solve).
impl CompiledLiquid {
    fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    fn exit_ref(&self) -> NozzleExitState {
        NozzleExitState {
            exit_mach: self.exit_mach,
            exit_pressure_ratio: self.exit_pressure_pa / self.chamber_pressure_pa,
            exit_temp_k: self.exit_temp_k,
            exhaust_velocity_mps: self.exhaust_velocity_mps,
        }
    }

    /// Gas-generator duct thrust at throttle and ambient. Exact for frozen
    /// duct geometry: vacuum endpoint scales with chamber flow, ambient
    /// eats the fixed duct exit area (F = t*F_vac - Pa*Ae_gg).
    fn gg_thrust_at(&self, throttle: f64, ambient_pa: f64) -> f64 {
        if self.gg_bypass_fraction <= 0.0 || throttle == 0.0 {
            return 0.0;
        }
        (self.gg_thrust_vac_n * throttle - ambient_pa * self.gg_exit_area_m2).max(0.0)
    }
}

impl CompiledSolid {
    fn thermo_ref(&self) -> PropellantThermo {
        PropellantThermo {
            gamma: self.gamma,
            chamber_temp_k: self.chamber_temp_k,
            gas_constant_j_kg_k: self.gas_constant_j_kg_k,
            bulk_density_kg_m3: 0.0,
            characteristic_length_m: 0.0,
            reference_c_star_mps: 0.0,
        }
    }

    /// Design-point exit pressure ratio (frozen nozzle geometry: the ratio
    /// depends only on gamma and expansion ratio, never on chamber pressure
    /// or ambient).
    fn exit_pressure_ratio(&self) -> f64 {
        (1.0 + (self.gamma - 1.0) / 2.0 * self.exit_mach * self.exit_mach)
            .powf(-self.gamma / (self.gamma - 1.0))
    }

    fn exit_state(&self) -> NozzleExitState {
        NozzleExitState {
            exit_mach: self.exit_mach,
            exit_pressure_ratio: self.exit_pressure_ratio(),
            exit_temp_k: self.exit_temp_k,
            exhaust_velocity_mps: self.exhaust_velocity_mps,
        }
    }

    fn interpolate(&self, burn_time_s: f64) -> BurnPoint {
        let curve = &self.burn_curve;
        if curve.is_empty() {
            return BurnPoint {
                time_s: 0.0,
                web_burned_m: 0.0,
                chamber_pa: 0.0,
                mass_flow_kg_s: 0.0,
                thrust_sl_n: 0.0,
                thrust_vac_n: 0.0,
            };
        }
        if burn_time_s <= 0.0 {
            return curve[0];
        }
        for window in curve.windows(2) {
            let (a, b) = (window[0], window[1]);
            if burn_time_s <= b.time_s {
                let span = (b.time_s - a.time_s).max(1e-12);
                let fraction = ((burn_time_s - a.time_s) / span).clamp(0.0, 1.0);
                return BurnPoint {
                    time_s: burn_time_s,
                    web_burned_m: a.web_burned_m + (b.web_burned_m - a.web_burned_m) * fraction,
                    chamber_pa: a.chamber_pa + (b.chamber_pa - a.chamber_pa) * fraction,
                    mass_flow_kg_s: a.mass_flow_kg_s
                        + (b.mass_flow_kg_s - a.mass_flow_kg_s) * fraction,
                    thrust_sl_n: a.thrust_sl_n + (b.thrust_sl_n - a.thrust_sl_n) * fraction,
                    thrust_vac_n: a.thrust_vac_n + (b.thrust_vac_n - a.thrust_vac_n) * fraction,
                };
            }
        }
        *curve.last().expect("non-empty burn curve")
    }
}

// ---------------------------------------------------------------------------
// Plume handoff (plain data; plume-core owns the render contract)
// ---------------------------------------------------------------------------

/// Nozzle/exhaust state for the plume renderer. Field-for-field compatible
/// with `thessa-plume-core` `PlumeSource`; the game-side choke point
/// `PlumeSource::from_engine_state` consumes exactly these values so neither
/// engine crate depends on the other.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EnginePlumeState {
    pub exit_radius_m: f64,
    pub mass_flow_kg_s: f64,
    pub exhaust_velocity_mps: f64,
    pub exit_pressure_pa: f64,
    pub exit_temp_k: f64,
    pub exit_mach: f64,
    pub propellant: Propellant,
}

// ---------------------------------------------------------------------------
// Spool / ignition runtime state
// ---------------------------------------------------------------------------

/// Live engine state: ignition, spool lag, solid burn clock. Pure data;
/// advance with [`advance_spool`] using physics seconds (f64 durations, as
/// in the integrator kernels — never `Instant`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct EngineSpool {
    pub running: bool,
    pub throttle_actual: f64,
    pub shots_remaining: Option<u32>,
    pub burn_elapsed_s: f64,
}

impl EngineSpool {
    /// New engine: cold, full shots, zero burn clock.
    pub fn new(engine: &CompiledEngine) -> Self {
        Self {
            running: false,
            throttle_actual: 0.0,
            shots_remaining: match engine {
                CompiledEngine::Liquid(liquid) => {
                    if liquid.restartable {
                        None
                    } else {
                        Some(1)
                    }
                }
                CompiledEngine::Solid(solid) => Some(solid.ignition_shots),
            },
            burn_elapsed_s: 0.0,
        }
    }
}

/// Advance live state toward a throttle command over `dt_s`. First-order
/// spool lag for liquids; ballistic ignition + burn clock for solids.
pub fn advance_spool(
    engine: &CompiledEngine,
    mut state: EngineSpool,
    throttle_cmd: f64,
    dt_s: f64,
) -> Result<EngineSpool, PropulsionError> {
    if !throttle_cmd.is_finite() || !(0.0..=1.0).contains(&throttle_cmd) {
        return Err(PropulsionError::InvalidCommand(
            "throttle command must be finite in [0, 1]".into(),
        ));
    }
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(PropulsionError::InvalidCommand(
            "spool dt must be finite and >= 0".into(),
        ));
    }
    match engine {
        CompiledEngine::Liquid(liquid) => {
            let (floor, _) = engine.throttle_range();
            let target = if throttle_cmd == 0.0 {
                0.0
            } else {
                throttle_cmd.max(floor)
            };
            if !state.running {
                if target == 0.0 {
                    state.throttle_actual = 0.0;
                    return Ok(state);
                }
                if let Some(shots) = state.shots_remaining {
                    if shots == 0 {
                        return Err(PropulsionError::InvalidCommand(
                            "no ignition shots remaining".into(),
                        ));
                    }
                    state.shots_remaining = Some(shots - 1);
                }
                state.running = true;
            } else if target == 0.0 {
                state.running = false;
            }
            let alpha = 1.0 - (-dt_s / liquid.spool_tau_s).exp();
            state.throttle_actual += (target - state.throttle_actual) * alpha;
            if !state.running && state.throttle_actual < 1e-6 {
                state.throttle_actual = 0.0;
            }
            Ok(state)
        }
        CompiledEngine::Solid(solid) => {
            if throttle_cmd != 0.0 && throttle_cmd != 1.0 {
                return Err(PropulsionError::InvalidCommand(
                    "solid throttle commands are 0 (safe) or 1 (fire)".into(),
                ));
            }
            if state.burn_elapsed_s >= solid.burn_time_s {
                state.running = false;
                state.throttle_actual = 0.0;
                return Ok(state);
            }
            if !state.running {
                if throttle_cmd == 0.0 {
                    return Ok(state);
                }
                if let Some(shots) = state.shots_remaining {
                    if shots == 0 {
                        return Err(PropulsionError::InvalidCommand(
                            "motor already burned".into(),
                        ));
                    }
                    state.shots_remaining = Some(shots - 1);
                }
                state.running = true;
            }
            state.throttle_actual = 1.0;
            state.burn_elapsed_s = (state.burn_elapsed_s + dt_s).min(solid.burn_time_s);
            if state.burn_elapsed_s >= solid.burn_time_s {
                state.running = false;
                state.throttle_actual = 0.0;
            }
            Ok(state)
        }
    }
}

// ---------------------------------------------------------------------------
// Editor analyzer (Juno Performance Analyzer backend)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Vehicle mounts
// ---------------------------------------------------------------------------

/// One gimbal control effector for the flight allocator. A normalized
/// command in [-1, 1] spans the gimbal range about `gimbal_axis`
/// (right-hand rule); force/moment scale linearly with the command.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GimbalEffector {
    /// Unit gimbal rotation axis in body axes.
    pub gimbal_axis: [f64; 3],
    /// Force per unit command (N, body axes).
    pub force_per_command_n: [f64; 3],
    /// Moment about the body origin per unit command (N·m, body axes).
    pub moment_per_command_nm: [f64; 3],
}

/// One engine installed on a vehicle: compiled data plus its mount station
/// and thrust axis. Mass aggregates into the vehicle budget at bake time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EngineMount {
    pub name: String,
    pub engine: CompiledEngine,
    /// Mount (throat) station in vehicle body metres.
    pub position_body_m: [f64; 3],
    /// Unit thrust direction in vehicle body axes (usually +X).
    pub thrust_axis_body: [f64; 3],
}

impl EngineMount {
    /// Validate mount data (NaN fails closed; axis must be unit).
    pub fn validate(&self) -> Result<(), PropulsionError> {
        if self.name.trim().is_empty() {
            return Err(PropulsionError::InvalidSpec(
                "engine mount needs a name".into(),
            ));
        }
        if self.position_body_m.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "mount position must be finite".into(),
            ));
        }
        let axis = self.thrust_axis_body;
        if axis.iter().any(|v| !v.is_finite()) {
            return Err(PropulsionError::InvalidSpec(
                "thrust axis must be finite".into(),
            ));
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if (norm - 1.0).abs() > 1e-9 {
            return Err(PropulsionError::InvalidSpec(
                "thrust axis must be unit length".into(),
            ));
        }
        Ok(())
    }

    /// Thrust vector in body axes at this mount's command.
    pub fn thrust_vector_body_n(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<[f64; 3], PropulsionError> {
        self.validate()?;
        let point = self
            .engine
            .operating_point(throttle, ambient_pa, burn_time_s)?;
        Ok([
            self.thrust_axis_body[0] * point.thrust_n,
            self.thrust_axis_body[1] * point.thrust_n,
            self.thrust_axis_body[2] * point.thrust_n,
        ])
    }

    /// Gimbal range of the installed engine (rad, 0 = fixed).
    pub fn gimbal_range_rad(&self) -> f64 {
        match &self.engine {
            CompiledEngine::Liquid(engine) => engine.gimbal_range_rad,
            CompiledEngine::Solid(engine) => engine.gimbal_range_rad,
        }
    }

    /// Gimbal control authority for the flight allocator: moment per gimbal
    /// radian about the two body axes perpendicular to the thrust axis, at
    /// an operating point. Moments are about the body origin (the allocator
    /// translates to the CG); the two effectors share the gimbal range, so
    /// the allocator must coordinate them. Fixed mounts return zero
    /// moments (axes still well-defined).
    pub fn gimbal_authority(
        &self,
        throttle: f64,
        ambient_pa: f64,
        burn_time_s: f64,
    ) -> Result<[GimbalEffector; 2], PropulsionError> {
        use glam::DVec3;
        self.validate()?;
        let thrust_n =
            DVec3::from_array(self.thrust_vector_body_n(throttle, ambient_pa, burn_time_s)?);
        let axis = DVec3::from_array(self.thrust_axis_body);
        let reference = if axis.x.abs() < 0.9 {
            DVec3::X
        } else {
            DVec3::Y
        };
        let gimbal_a = axis.cross(reference).normalize();
        let gimbal_b = axis.cross(gimbal_a).normalize();
        let position = DVec3::from_array(self.position_body_m);
        let mut effectors = Vec::with_capacity(2);
        for gimbal_axis in [gimbal_a, gimbal_b] {
            // dF/ddelta = gimbal_axis x F; moment = r x dF/ddelta.
            let force_per_rad = gimbal_axis.cross(thrust_n);
            let moment_per_rad = position.cross(force_per_rad);
            effectors.push(GimbalEffector {
                gimbal_axis: gimbal_axis.to_array(),
                force_per_command_n: (force_per_rad * self.gimbal_range_rad()).to_array(),
                moment_per_command_nm: (moment_per_rad * self.gimbal_range_rad()).to_array(),
            });
        }
        Ok([effectors[0], effectors[1]])
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn merlin_like() -> LiquidEngineSpec {
        LiquidEngineSpec {
            name: "Merlin-1D class".into(),
            propellant: Propellant::LoxRp1,
            cycle: EngineCycle::GasGenerator,
            chamber_pressure_pa: 9.7e6,
            throat_radius_m: 0.134,
            expansion_ratio: 16.0,
            nozzle_length_m: 1.5,
            contour: NozzleContour::Bell,
            chamber_material: ChamberMaterial::nickel_superalloy(),
            cooling: CoolingMode::Regenerative,
            mixture_ratio: None,
            characteristic_length_m: None,
            gimbal_range_rad: 0.09,
            min_throttle: None,
            restartable: true,
        }
    }

    #[test]
    fn c_star_matches_published_values() {
        // Calibration of the (gamma, Tc, R) triples against Sutton-typical
        // published c*: proves the tabulated thermo is self-consistent.
        for propellant in [
            Propellant::LoxRp1,
            Propellant::LoxMethane,
            Propellant::LoxHydrogen,
            Propellant::NtoMmh,
            Propellant::SolidApcp,
        ] {
            let thermo = propellant.thermo();
            let c_star = characteristic_velocity(&thermo);
            let error = (c_star - thermo.reference_c_star_mps).abs() / thermo.reference_c_star_mps;
            assert!(
                error < 0.04,
                "{propellant:?}: c* {c_star:.0} vs published {:.0}",
                thermo.reference_c_star_mps
            );
        }
    }

    #[test]
    fn sonic_throat_and_mach_solver_round_trip() {
        // Known special cases: eps = 1 chokes exactly; the solver inverts
        // the area-Mach relation to 1e-9 on the Juno bell range.
        let mach = mach_from_area_ratio(1.24, 1.0).expect("sonic throat");
        assert!((mach - 1.0).abs() < 1e-12);
        for (gamma, eps) in [(1.2, 3.0), (1.22, 16.0), (1.24, 35.0), (1.25, 80.0)] {
            let mach = mach_from_area_ratio(gamma, eps).expect("mach solve");
            assert!(mach > 1.0);
            let round_trip = area_ratio_from_mach(gamma, mach);
            assert!(
                (round_trip - eps).abs() / eps < 1e-9,
                "gamma {gamma} eps {eps}: got {round_trip}"
            );
        }
    }

    #[test]
    fn merlin_class_golden_data() {
        // Golden regression: a Merlin-1D-class spec (Pc 9.7 MPa, eps 16,
        // kerolox GG) must reproduce published 845 kN SL / 914 kN vac and
        // 282 s / 311 s within the documented 5% envelope, and land in the
        // published dry-mass band (470 kg with margin for fit spread).
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let sl = engine
            .operating_point(1.0, 101_325.0, 0.0)
            .expect("sea-level point");
        let vac = engine.operating_point(1.0, 0.0, 0.0).expect("vacuum point");
        for (actual, published, label) in [
            (sl.thrust_n, 845_000.0, "SL thrust"),
            (vac.thrust_n, 914_000.0, "vac thrust"),
            (sl.isp_s, 282.0, "SL Isp"),
            (vac.isp_s, 311.0, "vac Isp"),
        ] {
            let error = (actual - published).abs() / published;
            assert!(error < 0.05, "{label}: {actual:.0} vs {published:.0}");
        }
        let mass = engine.dry_mass_kg();
        assert!(
            (200.0..=700.0).contains(&mass),
            "dry mass {mass:.0} kg outside the Merlin band"
        );
        // Vacuum beats sea level; the sea-level bell is not separated.
        assert!(vac.thrust_n > sl.thrust_n);
        assert!(vac.isp_s > sl.isp_s);
        assert!(!sl.separation_risk);
    }

    #[test]
    fn vacuum_sea_level_ordering_and_throttle_linearity() {
        // Regression pin: throttle scales chamber pressure linearly by
        // construction (documented deep-throttle assumption); guards
        // accidental nonlinearity in the eval path.
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let full = engine.operating_point(1.0, 0.0, 0.0).expect("full");
        let half = engine.operating_point(0.5, 0.0, 0.0).expect("half");
        assert!((half.thrust_n * 2.0 - full.thrust_n).abs() / full.thrust_n < 1e-12);
        assert!(
            (half.mass_flow_kg_s * 2.0 - full.mass_flow_kg_s).abs() / full.mass_flow_kg_s < 1e-12
        );
        assert!((half.isp_s - full.isp_s).abs() / full.isp_s < 1e-12);
    }

    #[test]
    fn overexpanded_nozzle_flags_separation() {
        // A sea-level-tuned bell (eps 8) deep in vacuum-throttle... inverse:
        // a big vacuum bell at sea level must trip Summerfield.
        let spec = LiquidEngineSpec {
            expansion_ratio: 80.0,
            ..merlin_like()
        };
        let engine = CompiledEngine::Liquid(spec.compile().expect("compile"));
        let sl = engine.operating_point(1.0, 101_325.0, 0.0).expect("point");
        assert!(sl.separation_risk, "eps-80 bell at SL must flag");
        let vac = engine.operating_point(1.0, 0.0, 0.0).expect("vac");
        assert!(!vac.separation_risk);
    }

    #[test]
    fn aerospike_holds_thrust_across_altitudes() {
        // Altitude compensation: an eps-35 aerospike keeps sea-level thrust
        // within 3% of vacuum (a bell loses ~17% on the same geometry) and
        // never flags separation; base drag is the only altitude term.
        let spike = LiquidEngineSpec {
            contour: NozzleContour::Aerospike,
            ..merlin_like()
        };
        let spike = CompiledEngine::Liquid(spike.compile().expect("spike"));
        let bell = CompiledEngine::Liquid(merlin_like().compile().expect("bell"));
        let spike_sl = spike
            .operating_point(1.0, 101_325.0, 0.0)
            .expect("spike SL");
        let spike_vac = spike.operating_point(1.0, 0.0, 0.0).expect("spike vac");
        let bell_sl = bell.operating_point(1.0, 101_325.0, 0.0).expect("bell SL");
        let bell_vac = bell.operating_point(1.0, 0.0, 0.0).expect("bell vac");
        assert!(!spike_sl.separation_risk);
        let spike_drop = (spike_vac.thrust_n - spike_sl.thrust_n) / spike_vac.thrust_n;
        let bell_drop = (bell_vac.thrust_n - bell_sl.thrust_n) / bell_vac.thrust_n;
        assert!(
            spike_drop < 0.03,
            "aerospike drop {spike_drop:.3} too large"
        );
        assert!(bell_drop > 0.05, "bell control must lose thrust at SL");
        // Base drag accounting: SL loss equals ambient times base area.
        let expected = 101_325.0
            * match &spike {
                CompiledEngine::Liquid(liquid) => liquid.aerospike_base_area_m2,
                CompiledEngine::Solid(_) => 0.0,
            };
        assert!(((spike_vac.thrust_n - spike_sl.thrust_n) - expected).abs() / expected < 1e-9);
    }

    #[test]
    fn cycle_pressure_caps_and_cooling_gates_fire() {
        // Pressure-fed topology cannot hold Merlin pressures; radiative
        // walls refuse them too. The compiler refuses instead of derating.
        let fed = LiquidEngineSpec {
            cycle: EngineCycle::PressureFed,
            ..merlin_like()
        };
        assert!(fed.compile().is_err());
        let rad = LiquidEngineSpec {
            cooling: CoolingMode::Radiative,
            ..merlin_like()
        };
        assert!(rad.compile().is_err());
        // But the same geometry at 2 MPa pressure-fed compiles.
        let small = LiquidEngineSpec {
            cycle: EngineCycle::PressureFed,
            chamber_pressure_pa: 2.0e6,
            ..merlin_like()
        };
        assert!(small.compile().is_ok());
    }

    #[test]
    fn nan_inputs_fail_closed() {
        let mut spec = merlin_like();
        spec.chamber_pressure_pa = f64::NAN;
        assert!(spec.compile().is_err());
        let engine = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        assert!(engine.operating_point(f64::NAN, 0.0, 0.0).is_err());
        assert!(engine.operating_point(1.0, f64::NAN, 0.0).is_err());
    }

    #[test]
    fn bell_beats_cone_within_envelope() {
        // Bell recovers part of the divergence loss: strictly better, and
        // within the documented +/-0.5% recovery envelope vs the cone.
        let cone = LiquidEngineSpec {
            contour: NozzleContour::Conical,
            ..merlin_like()
        };
        let bell = merlin_like();
        let cone = CompiledEngine::Liquid(cone.compile().expect("cone"));
        let bell = CompiledEngine::Liquid(bell.compile().expect("bell"));
        let cone_vac = cone.operating_point(1.0, 0.0, 0.0).expect("cone vac");
        let bell_vac = bell.operating_point(1.0, 0.0, 0.0).expect("bell vac");
        assert!(bell_vac.thrust_n > cone_vac.thrust_n);
        let gain = (bell_vac.thrust_n - cone_vac.thrust_n) / cone_vac.thrust_n;
        assert!(gain < 0.01, "bell recovery {gain:.4} exceeds envelope");
    }

    #[test]
    fn solid_burn_trace_is_physical() {
        // BATES grain: a cylindrical port opens as it burns, so the trace
        // is inherently progressive (thrust rises monotonically) — neutral
        // traces need star/finocyl grains, which are deferred. The test pins
        // the progressive signature, impulse/mass/Isp consistency, and the
        // choked-motor property that altitude replay changes only the
        // pressure term at frozen chamber pressure.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "APCP booster".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.32,
            segment_length_m: 1.5,
            segments: 4,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("solid compiles");
        assert!(motor.burn_time_s > 1.0 && motor.burn_time_s < 60.0);
        // Time-stepped trace: resolved (hundreds of stations) with
        // strictly increasing time.
        assert!(motor.burn_curve.len() >= 100);
        for window in motor.burn_curve.windows(2) {
            assert!(window[1].time_s > window[0].time_s);
        }
        // Progressive signature: vacuum thrust never decreases along the
        // web (the terminal point is the zero-thrust burn-through).
        let live = &motor.burn_curve[..motor.burn_curve.len() - 1];
        for window in live.windows(2) {
            assert!(
                window[1].thrust_vac_n >= window[0].thrust_vac_n,
                "cylindrical-port trace must be progressive"
            );
        }
        let mean_vac = motor.total_impulse_ns / motor.burn_time_s;
        let peak_vac = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        assert!(
            peak_vac / mean_vac < 1.8,
            "chunky-port progressivity must stay bounded"
        );
        // Impulse = Isp * m_prop * g0 (internal consistency).
        let check = motor.avg_isp_s * motor.propellant_mass_kg * STANDARD_GRAVITY_MPS2;
        assert!((check - motor.total_impulse_ns).abs() / motor.total_impulse_ns < 1e-9);
        // Chamber pressure frozen vs ambient: SL and vacuum evaluation at
        // mid-burn differ only by the (Pe - Pa) Ae term.
        let engine = CompiledEngine::Solid(motor);
        let mid = engine
            .operating_point(1.0, 101_325.0, 1.0)
            .expect("mid-burn SL");
        let mid_vac = engine.operating_point(1.0, 0.0, 1.0).expect("mid-burn vac");
        assert!((mid.mass_flow_kg_s - mid_vac.mass_flow_kg_s).abs() / mid.mass_flow_kg_s < 1e-12);
        assert!(engine.operating_point(0.5, 0.0, 1.0).is_err());
        // Past burnout: zero thrust, no error.
        let done = engine.operating_point(1.0, 0.0, 1.0e6).expect("burned out");
        assert_eq!(done.thrust_n, 0.0);
    }

    #[test]
    fn stepped_ports_shape_boost_sustain() {
        // Stepped channel: a thin-web segment burns out early (boost) and
        // the thick-web segment sustains. The test pins the two-phase
        // signature plus integrated-vs-geometric propellant agreement.
        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "boost-sustain".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.12,
            expansion_ratio: 10.0,
            nozzle_length_m: 0.9,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: Some(vec![0.15, 0.35]),
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let motor = spec.compile().expect("stepped grain compiles");
        let engine = CompiledEngine::Solid(motor.clone());
        let peak = motor
            .burn_curve
            .iter()
            .map(|point| point.thrust_vac_n)
            .fold(0.0_f64, f64::max);
        let peak_time = motor
            .burn_curve
            .iter()
            .find(|point| point.thrust_vac_n >= peak * (1.0 - 1e-9))
            .expect("peak station")
            .time_s;
        assert!(
            peak_time < 0.4 * motor.burn_time_s,
            "boost peak must sit in the first 40% of the burn"
        );
        let late = engine
            .operating_point(1.0, 0.0, 0.85 * motor.burn_time_s)
            .expect("late sustain")
            .thrust_n;
        assert!(
            late < 0.55 * peak,
            "sustain phase must drop below 55% of boost peak"
        );
        // Integrated propellant agrees with the geometric grain mass.
        let geometric: f64 = [0.15, 0.35]
            .iter()
            .map(|core_m| std::f64::consts::PI * (0.5 * 0.5 - core_m * core_m) * 1.5 * 1770.0)
            .sum();
        let drift = (motor.propellant_mass_kg - geometric).abs() / geometric;
        assert!(drift < 0.02, "propellant drift {drift:e} too large");
        // Port-count mismatch is refused, not silently broadcast.
        let bad = SolidMotorSpec {
            segment_core_radii_m: Some(vec![0.2]),
            ..spec
        };
        assert!(bad.compile().is_err());
    }

    #[test]
    fn unstable_burn_exponent_is_rejected() {
        let (a, _) = SolidMotorSpec::apcp_ballistics();
        let spec = SolidMotorSpec {
            name: "unstable".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.5,
            core_radius_m: 0.15,
            segment_length_m: 1.5,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: 1.2,
            throat_radius_m: 0.12,
            expansion_ratio: 8.0,
            nozzle_length_m: 0.8,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        assert!(spec.compile().is_err());
    }

    #[test]
    fn gimbal_authority_matches_lever_arm() {
        // +X thrust at x = -3 m with 1 MN: gimbaling must produce ~3 MN·m
        // per radian about the transverse axes and ~0 about the thrust
        // axis; per-command values scale by the installed range.
        let mount = EngineMount {
            name: "lever probe".into(),
            engine: CompiledEngine::Liquid(
                LiquidEngineSpec {
                    name: "lever".into(),
                    gimbal_range_rad: 0.1,
                    ..merlin_like()
                }
                .compile()
                .expect("compile"),
            ),
            position_body_m: [-3.0, 0.0, 0.0],
            thrust_axis_body: [1.0, 0.0, 0.0],
        };
        let authority = mount.gimbal_authority(1.0, 0.0, 0.0).expect("authority");
        let full = mount
            .engine
            .operating_point(1.0, 0.0, 0.0)
            .expect("point")
            .thrust_n;
        for effector in authority {
            let moment = glam::DVec3::from_array(effector.moment_per_command_nm);
            // Per radian (divide the range back out), transverse only.
            let per_rad = moment / 0.1;
            assert!(per_rad.x.abs() < full * 0.01, "no roll authority expected");
            assert!(
                (per_rad.length() - full * 3.0).abs() / (full * 3.0) < 1e-9,
                "moment must equal thrust times lever arm"
            );
        }
    }

    #[test]
    fn thermal_power_ordering_and_depletion() {
        // Energy ordering: kinetic exhaust power stays below released
        // chamber power (the gap is residual enthalpy + duct losses).
        // Depletion: remaining grain falls monotonically to exactly zero.
        let liquid = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        assert!(liquid.exhaust_kinetic_power_w() > 0.0);
        assert!(liquid.exhaust_kinetic_power_w() < liquid.chamber_power_w());
        assert_eq!(liquid.propellant_remaining_kg(10.0), None);

        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let solid = CompiledEngine::Solid(
            SolidMotorSpec {
                name: "depletion probe".into(),
                propellant: Propellant::SolidApcp,
                outer_radius_m: 0.5,
                core_radius_m: 0.32,
                segment_length_m: 1.5,
                segments: 2,
                burn_rate_coeff: a,
                burn_rate_exponent: n,
                throat_radius_m: 0.12,
                expansion_ratio: 8.0,
                nozzle_length_m: 0.8,
                contour: NozzleContour::Conical,
                casing_material: ChamberMaterial::nickel_superalloy(),
                inhibited_ends: true,
                segment_core_radii_m: None,
                gimbal_range_rad: 0.0,
                ignition_shots: 1,
            }
            .compile()
            .expect("solid"),
        );
        assert!(solid.exhaust_kinetic_power_w() < solid.chamber_power_w());
        let burn_time = match &solid {
            CompiledEngine::Solid(motor) => motor.burn_time_s,
            _ => 0.0,
        };
        let total = solid.propellant_remaining_kg(0.0).expect("grain");
        assert!(total > 0.0);
        let mut last = total;
        for step in 1..=10 {
            let remaining = solid
                .propellant_remaining_kg(burn_time * step as f64 / 10.0)
                .expect("grain");
            assert!(remaining <= last + total * 1e-9);
            last = remaining;
        }
        assert_eq!(solid.propellant_remaining_kg(burn_time), Some(0.0));
        assert_eq!(solid.propellant_remaining_kg(burn_time + 100.0), Some(0.0));
    }

    #[test]
    fn spool_ignition_shutdown_and_solid_single_shot() {
        let liquid = CompiledEngine::Liquid(merlin_like().compile().expect("compile"));
        let mut state = EngineSpool::new(&liquid);
        assert!(state.shots_remaining.is_none());
        state = advance_spool(&liquid, state, 1.0, 10.0).expect("ignite");
        assert!(state.running);
        assert!((state.throttle_actual - 1.0).abs() < 1e-6);
        // Below the stability floor clamps up (documented), zero shuts down.
        state = advance_spool(&liquid, state, 0.05, 10.0).expect("clamp");
        assert!(state.running);
        assert!(state.throttle_actual >= 0.40 - 1e-9);
        state = advance_spool(&liquid, state, 0.0, 10.0).expect("shutdown");
        assert!(!state.running);

        let (a, n) = SolidMotorSpec::apcp_ballistics();
        let solid_spec = SolidMotorSpec {
            name: "single shot".into(),
            propellant: Propellant::SolidApcp,
            outer_radius_m: 0.3,
            core_radius_m: 0.1,
            segment_length_m: 1.0,
            segments: 2,
            burn_rate_coeff: a,
            burn_rate_exponent: n,
            throat_radius_m: 0.09,
            expansion_ratio: 8.0,
            nozzle_length_m: 0.6,
            contour: NozzleContour::Conical,
            casing_material: ChamberMaterial::nickel_superalloy(),
            inhibited_ends: true,
            segment_core_radii_m: None,
            gimbal_range_rad: 0.0,
            ignition_shots: 1,
        };
        let solid = CompiledEngine::Solid(solid_spec.compile().expect("solid"));
        let mut state = EngineSpool::new(&solid);
        assert!(advance_spool(&solid, state, 0.5, 0.1).is_err());
        state = advance_spool(&solid, state, 1.0, 0.1).expect("ignite");
        assert!(state.running && state.throttle_actual == 1.0);
        state = advance_spool(&solid, state, 1.0, 1.0e6).expect("burnout");
        assert!(!state.running && state.throttle_actual == 0.0);
        assert!(advance_spool(&solid, state, 1.0, 0.1).is_ok());
    }

    #[test]
    fn mixture_reference_reproduces_design_point() {
        // The middle table row is the reference point exactly: an explicit
        // reference ratio must compile to the same engine as `None`.
        for propellant in [
            Propellant::LoxRp1,
            Propellant::LoxMethane,
            Propellant::LoxHydrogen,
            Propellant::NtoMmh,
        ] {
            let reference = propellant.reference_mixture_ratio().expect("ref ratio");
            let at_ref = propellant
                .thermo_at_mixture(Some(reference))
                .expect("reference ratio compiles");
            assert_eq!(at_ref, propellant.thermo());
        }
        let plain = merlin_like().compile().expect("plain");
        let explicit = LiquidEngineSpec {
            mixture_ratio: Some(2.7),
            ..merlin_like()
        }
        .compile()
        .expect("explicit ref");
        assert!((explicit.thrust_vac_n - plain.thrust_vac_n).abs() / plain.thrust_vac_n < 1e-12);
    }

    #[test]
    fn mixture_shifts_performance_in_documented_direction() {
        // Fuel-rich hydrolox carries more light H2 in the products: higher
        // gas constant wins over the cooler chamber, so c* rises. The test
        // pins the direction and a bounded magnitude, not a point value.
        let h2 = Propellant::LoxHydrogen;
        let rich = h2.thermo_at_mixture(Some(4.5)).expect("fuel-rich");
        let reference = h2.thermo();
        assert!(characteristic_velocity(&rich) > characteristic_velocity(&reference));
        let gain = (characteristic_velocity(&rich) - characteristic_velocity(&reference))
            / characteristic_velocity(&reference);
        assert!(
            gain < 0.08,
            "mixture c* gain {gain:.3} exceeds the table envelope"
        );

        // Oxidizer-rich methane cools and heavies the flow: Isp must drop
        // a few percent relative to the reference compile.
        let mk = |ratio: Option<f64>| {
            CompiledEngine::Liquid(
                LiquidEngineSpec {
                    name: "mixture probe".into(),
                    propellant: Propellant::LoxMethane,
                    cycle: EngineCycle::StagedCombustion,
                    chamber_pressure_pa: 20.0e6,
                    throat_radius_m: 0.15,
                    expansion_ratio: 35.0,
                    nozzle_length_m: 1.8,
                    contour: NozzleContour::Bell,
                    chamber_material: ChamberMaterial::nickel_superalloy(),
                    cooling: CoolingMode::Regenerative,
                    mixture_ratio: ratio,
                    characteristic_length_m: None,
                    gimbal_range_rad: 0.0,
                    min_throttle: None,
                    restartable: true,
                }
                .compile()
                .expect("methalox compiles"),
            )
        };
        let ref_isp = mk(None)
            .operating_point(1.0, 0.0, 0.0)
            .expect("ref point")
            .isp_s;
        let ox_isp = mk(Some(4.2))
            .operating_point(1.0, 0.0, 0.0)
            .expect("ox-rich point")
            .isp_s;
        assert!(ox_isp < ref_isp, "oxidizer-rich methalox must lose Isp");
        assert!((ref_isp - ox_isp) / ref_isp < 0.08);

        // Outside the table and on solids: refusal, not extrapolation.
        assert!(h2.thermo_at_mixture(Some(9.0)).is_err());
        assert!(Propellant::SolidApcp.thermo_at_mixture(Some(1.0)).is_err());
    }

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
