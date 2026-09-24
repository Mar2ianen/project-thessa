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
//! Layout: `error` (validation), `propellant` (thermo + mixture tables),
//! `nozzle` (isentropic kernel + contours), `cycle` (feed topologies),
//! `material` (walls + cooling), `liquid` / `solid` (authoring + compile),
//! `engine` (runtime dispatch + thermal/depletion), `spool` (ignition
//! dynamics), `shaft` (jet shaft state: starter topologies, light-off and
//! self-sustain, generator load), `analyze` (editor altitude curves), `mount` (vehicle mounts +
//! gimbal authority). Every public name from the former single-file module
//! re-exports here unchanged.

mod air;
mod analyze;
mod cycle;
mod electric;
mod engine;
mod error;
mod estoc;
mod fusion;
mod jet;
mod liquid;
mod material;
mod mount;
mod nozzle;
mod nuclear;
mod propellant;
mod rcs;
mod shaft;
mod shaft_power;
mod solid;
mod spool;
mod system;

pub use air::{
    AIR_CP_J_KG_K, AIR_GAMMA, AirAltitudePoint, AirCycle, AirOperatingPoint, AirbreathingSpec,
    CompiledAirbreather, FlightCondition, IntakeKind, JetFuel, advance_jet_spool,
    analyze_airbreathing, flight_condition,
};
pub use analyze::{AltitudePoint, analyze_altitude};
pub use cycle::{CycleLimits, EngineCycle};
pub use electric::{
    CompiledElectricThruster, ElectricPropellant, ElectricThrusterCommand, ElectricThrusterDesign,
    ElectricThrusterMount, ElectricThrusterPoint, ElectricThrusterSpec,
};
pub use engine::{CompiledEngine, EngineOperatingPoint, EnginePlumeState};
pub use error::PropulsionError;
pub use estoc::{
    CompiledEstoc, ESTOC_DEFAULT_SWITCH_MACH_HI, ESTOC_DEFAULT_SWITCH_MACH_LO,
    ESTOC_DEFAULT_TRANSITION_TAU_S, ESTOC_MAX_ROCKET_PC_PA, ESTOC_REINFORCEMENT_FRACTION,
    EstocMode, EstocPoint, EstocSpec, EstocTransient,
};
pub use fusion::{
    CompiledFusionTorch, CompiledPulsedFusion, FusionReaction, FusionTorchCommand,
    FusionTorchMount, FusionTorchOperatingPoint, FusionTorchSpec, PulsedFusionCommand,
    PulsedFusionMount, PulsedFusionOperatingPoint, PulsedFusionSpec, PulsedFusionState,
};
pub use jet::{CompiledJet, JetCommand, JetMount};
pub use liquid::{CompiledLiquid, LiquidEngineSpec};
pub use material::{ChamberMaterial, CoolingMode};
pub use mount::{EngineMount, GimbalEffector};
pub use nozzle::{
    NozzleContour, NozzleExitState, characteristic_velocity, mach_from_area_ratio,
    thrust_coefficient,
};
pub use nuclear::{
    NTR_COOLDOWN_FRACTION, NTR_DEFAULT_RATED_BURN_S, NTR_DEFAULT_SPECIFIC_MASS_KG_PER_MW,
    NTR_DEFAULT_STARTUP_TAU_S, NTR_INLET_TEMP_K, NTR_KINETIC_EFFICIENCY,
    NTR_MAX_CHAMBER_PRESSURE_PA, NtrFluid, NtrSupplement, NuclearThermalSpec,
};
pub use propellant::{Propellant, PropellantThermo};
pub use rcs::{
    ColdGasThrusterSpec, CompiledColdGas, CompiledMonoprop, MonopropThrusterSpec,
    RCS_DEFAULT_MIN_ON_TIME_S, RCS_DEFAULT_RISE_TIME_S, RcsCluster, RcsMount, RcsPulse,
    RcsThruster,
};
pub use shaft::{
    GeneratorSpec, JetShaftState, ShaftBalance, ShaftCommand, ShaftSpec, ShaftTelemetry,
    StarterKind, StarterSpec, advance_jet_shaft, advance_jet_shaft_loaded,
};
pub use shaft_power::{
    CompiledElectricMotor, CompiledPistonEngine, CompiledPropeller, CompiledPropellerDrive,
    CompiledShaftPowerSource, CompiledTurbopropDrive, ElectricMotorPoint, ElectricMotorSpec,
    PistonEngineSpec, PistonOperatingPoint, PropDriveAltitudePoint, PropDrivePoint,
    PropellerDriveCommand, PropellerDriveMount, PropellerDriveSpec, PropellerPoint, PropellerSpec,
    ShaftPowerSourceSpec, TurbopropAltitudePoint, TurbopropCommand, TurbopropDriveSpec,
    TurbopropMount, TurbopropOperatingPoint, analyze_propeller_drive, analyze_turboprop_drive,
    effective_propulsive_isp_s,
};
pub use solid::{BurnPoint, CompiledSolid, SolidGrainGeometry, SolidMotorSpec};
pub use spool::{EngineSpool, advance_spool};
pub use system::{
    ChamberSpec, CompiledChamber, CompiledPropulsionSystem, MAX_SYSTEM_CHAMBERS,
    PropulsionSystemSpec, SystemAltitudePoint, SystemMount, SystemOperatingPoint,
};

pub(crate) use error::{require_non_negative, require_positive, require_unit_interval};
pub(crate) use nozzle::{AEROSPIKE_BASE_FRACTION, nozzle_exit};

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
pub(crate) const GG_DUCT_EXPANSION_RATIO: f64 = 3.0;
/// Gas-generator turbine exhaust pressure as a fraction of chamber pressure
/// (post-turbine, documented assumption).
pub(crate) const GG_PRESSURE_FRACTION: f64 = 0.6;
/// Injector pressure drop as a fraction of chamber pressure (feed sizing).
pub(crate) const INJECTOR_DROP_FRACTION: f64 = 0.2;
/// Tank pressure assumed for electric-pump power bookkeeping (Pa).
pub(crate) const TANK_PRESSURE_PA: f64 = 300_000.0;
/// Pump hydraulic efficiency for electric-pump power (documented).
pub(crate) const PUMP_EFFICIENCY: f64 = 0.6;
/// Electric pump specific power for mass bookkeeping (W/kg, documented fit).
pub(crate) const PUMP_SPECIFIC_POWER_W_PER_KG: f64 = 5_000.0;
/// Structural safety factor on thin-wall pressure sizing.
pub(crate) const PRESSURE_SAFETY_FACTOR: f64 = 1.5;
/// Chamber length-to-diameter ratio (documented; the contraction ratio then
/// follows from the solved chamber volume instead of being asserted).
pub(crate) const CHAMBER_LENGTH_DIAMETER: f64 = 1.2;

/// Mass-fit: turbomachinery mass per unit flow power (kg per (kg/s * Pa)).
/// Calibration fit; the golden test pins a reference engine inside a wide
/// published-data band rather than at a point.
pub(crate) const MASS_FIT_TURBO_KG_PER_FLOW_POWER: f64 = 4.0e-8;
/// Mass-fit: mount/thrust-structure interface per unit vacuum thrust (kg/N).
pub(crate) const MASS_FIT_MOUNT_KG_PER_N: f64 = 8.0e-5;
/// Mass-fit: valves/lines/controller per unit vacuum thrust (kg/N).
pub(crate) const MASS_FIT_FEED_KG_PER_N: f64 = 3.0e-5;
/// Mass-fit: injector head base mass (kg) plus per-throat-area term.
pub(crate) const MASS_FIT_HEAD_BASE_KG: f64 = 8.0;
pub(crate) const MASS_FIT_HEAD_KG_PER_M2: f64 = 150.0;
/// Mass-fit: gimbal actuator base plus per-thrust term (kg, kg/N).
pub(crate) const MASS_FIT_GIMBAL_BASE_KG: f64 = 4.0;
pub(crate) const MASS_FIT_GIMBAL_KG_PER_N: f64 = 2.0e-5;
/// Solid igniter + hardware flat allowance (kg, documented fit).
pub(crate) const MASS_FIT_SOLID_IGNITER_KG: f64 = 2.0;
