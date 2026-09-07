//! Canonical SI conversion constants used by the authoritative simulation.

/// Newtonian gravitational constant, m^3 kg^-1 s^-2.
pub const G: f64 = 6.674_30e-11;
/// IAU nominal astronomical unit, m.
pub const AU_M: f64 = 149_597_870_700.0;
/// Nominal solar mass used by the design data, kg.
pub const SOLAR_MASS_KG: f64 = 1.988_47e30;
/// Earth mass used by the design data, kg.
pub const EARTH_MASS_KG: f64 = 5.972_2e24;
/// Jupiter mass used by the design data, kg.
pub const JUPITER_MASS_KG: f64 = 1.898_13e27;
/// Julian day, s.
pub const DAY_S: f64 = 86_400.0;
/// 2*pi as an f64 constant to keep angle handling explicit.
pub const TAU: f64 = std::f64::consts::TAU;
