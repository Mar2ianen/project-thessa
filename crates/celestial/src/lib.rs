//! Deterministic celestial configuration, ephemerides, gravity fields and
//! simulation-time events.

#![forbid(unsafe_code)]

mod ephemeris;
mod frames;
mod gravity;
mod gravity_tree;
mod scheduler;
mod system;
mod time;
mod units;

use thessa_aero_core::BakedAtmosphere;

pub use ephemeris::{
    BakedBody, BakedEphemeris, BodyId, BodyState, EphemerisError, EphemerisFrame, EphemerisScratch,
    KeplerOrbit, OsculatingElements,
};
pub use frames::{ReferenceFrame, StateVector};
pub use gravity::{
    BodyHarmonics, GravityError, GravityField, HARMONICS_TRUNCATION, harmonic_correction,
};
pub use gravity_tree::{
    GravityNode, GravityNodeFrame, GravitySourceTree, TreeEval, monopole_error_estimate,
    quadrupole_correction, quadrupole_error_estimate,
};
pub use scheduler::{EventScheduler, ScheduledEvent, ScheduledKind};
pub use system::{
    BinaryOrbitConfig, CelestialConfig, OrbitConfig, StarConfig, SystemConfig, SystemMeta,
    SystemSpecError, ellipsoid_harmonics,
};
pub use time::{SimTime, WORLD_TICK_HZ, WORLD_TICK_S, WorldTick};
pub use units::{AU_M, DAY_S, EARTH_MASS_KG, G, JUPITER_MASS_KG, SOLAR_MASS_KG, TAU};
