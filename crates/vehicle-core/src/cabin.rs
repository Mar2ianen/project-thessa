//! Crewed cabin pressure state and control authority (`docs/details/05`).
//!
//! Hangar-side authoring (fuselage regions, atmospheres, suits, and exits)
//! compiles down to runtime cabin inventories/access rules, static exit
//! records, and whether anybody aboard can fly the craft (KSP-like: no
//! pilot at a station and no autopilot core means no control). Power/comm
//! dependencies of cores are recorded as future wiring, not implemented
//! gates.

use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::{error::Error, fmt};

/// Dry-air gas constant in J/kg/K (same source as the hangar compiler).
pub const R_DRY_AIR_J_KG_K: f64 = 287.05;
/// Molar masses in g/mol for the oxygen mass split.
pub const MOLAR_MASS_AIR_G_MOL: f64 = 28.97;
pub const MOLAR_MASS_O2_G_MOL: f64 = 32.0;

/// Cabin pressure failure modes.
#[derive(Debug, Clone, PartialEq)]
pub enum CabinError {
    InvalidCabin(String),
    InsufficientAir { needed_kg: f64, available_kg: f64 },
}

impl fmt::Display for CabinError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidCabin(message) => write!(formatter, "invalid cabin: {message}"),
            Self::InsufficientAir {
                needed_kg,
                available_kg,
            } => write!(
                formatter,
                "insufficient air reserve: need {needed_kg:.3} kg, have {available_kg:.3} kg"
            ),
        }
    }
}

impl Error for CabinError {}

/// Runtime pressure state of one cabin volume. Opening a connected hatch
/// currently resolves the ideal-gas equalization as one discrete transition;
/// finite-rate flow is a later dynamics slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinPressureState {
    Pressurized,
    Vacuum,
}

/// Pressure protection available to one crew member while traversing a
/// connected set of interior volumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CrewSuitMode {
    /// No pressure protection.
    Unsuited,
    /// Pressure suit supplied from vehicle air; it cannot support vacuum.
    HoseFed,
    /// Suit with an independent pressure and breathing supply.
    SelfContained,
}

/// One cabin pressure volume with tracked air inventory. `pressure_kpa` is
/// the authored repressurization target; instantaneous pressure follows from
/// the current air mass, temperature, and volume.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PressurizedCabin {
    pub name: String,
    pub volume_m3: f64,
    /// Cabin air-inventory centroid in the vehicle body frame. The baker
    /// uses the compiled interior centroid; legacy serialized cabins default
    /// to the vehicle origin.
    #[serde(default)]
    pub centroid_body_m: DVec3,
    pub pressure_kpa: f64,
    pub temp_k: f64,
    pub o2_fraction: f64,
    pub air_kg: f64,
    pub state: CabinPressureState,
}

impl PressurizedCabin {
    /// Full-charge inventory plus initial state: air aboard means
    /// pressurized, an empty volume starts at vacuum.
    pub fn new(
        name: impl Into<String>,
        volume_m3: f64,
        pressure_kpa: f64,
        temp_k: f64,
        o2_fraction: f64,
        initial_air_kg: f64,
    ) -> Result<Self, CabinError> {
        let cabin = Self {
            name: name.into(),
            volume_m3,
            centroid_body_m: DVec3::ZERO,
            pressure_kpa,
            temp_k,
            o2_fraction,
            air_kg: initial_air_kg,
            state: CabinPressureState::Pressurized,
        };
        cabin.validate()?;
        if initial_air_kg > cabin.full_charge_kg() {
            return Err(CabinError::InvalidCabin(format!(
                "cabin '{}' initial air exceeds its design charge",
                cabin.name
            )));
        }
        let state = if initial_air_kg > 0.0 {
            CabinPressureState::Pressurized
        } else {
            CabinPressureState::Vacuum
        };
        Ok(Self { state, ..cabin })
    }

    /// Place this cabin's gas inventory at its compiled interior centroid.
    pub fn with_centroid_body_m(mut self, centroid_body_m: DVec3) -> Result<Self, CabinError> {
        if !centroid_body_m.is_finite() {
            return Err(CabinError::InvalidCabin(format!(
                "cabin '{}' centroid must be finite",
                self.name
            )));
        }
        self.centroid_body_m = centroid_body_m;
        Ok(self)
    }

    pub fn validate(&self) -> Result<(), CabinError> {
        if self.name.trim().is_empty() {
            return Err(CabinError::InvalidCabin("cabin needs a name".into()));
        }
        for (label, value, min, max) in [
            ("volume_m3", self.volume_m3, 0.0, f64::INFINITY),
            ("pressure_kpa", self.pressure_kpa, 0.0, 500.0),
            ("temp_k", self.temp_k, 180.0, 350.0),
            ("o2_fraction", self.o2_fraction, 0.0, 1.0),
        ] {
            if !value.is_finite() || value <= min || value > max {
                return Err(CabinError::InvalidCabin(format!(
                    "cabin '{}' needs {label} in ({min}, {max}]",
                    self.name
                )));
            }
        }
        if !self.centroid_body_m.is_finite() {
            return Err(CabinError::InvalidCabin(format!(
                "cabin '{}' centroid must be finite",
                self.name
            )));
        }
        if !self.air_kg.is_finite() || self.air_kg < 0.0 {
            return Err(CabinError::InvalidCabin(format!(
                "cabin '{}' air inventory must be finite and non-negative",
                self.name
            )));
        }
        if !self.current_pressure_kpa().is_finite() {
            return Err(CabinError::InvalidCabin(format!(
                "cabin '{}' current pressure is not finite",
                self.name
            )));
        }
        Ok(())
    }

    /// Air mass at full charge from the ideal gas law over the volume.
    pub fn full_charge_kg(&self) -> f64 {
        self.pressure_kpa * 1000.0 / (R_DRY_AIR_J_KG_K * self.temp_k) * self.volume_m3
    }

    /// Current ideal-gas pressure in kPa from the tracked air inventory.
    pub fn current_pressure_kpa(&self) -> f64 {
        self.air_kg * R_DRY_AIR_J_KG_K * self.temp_k / self.volume_m3 / 1000.0
    }

    /// Oxygen mass within the current air inventory.
    pub fn o2_kg(&self) -> f64 {
        self.air_kg * self.o2_fraction * MOLAR_MASS_O2_G_MOL / MOLAR_MASS_AIR_G_MOL
    }

    /// Dump the inventory overboard (Gemini-style whole-cabin venting).
    /// Returns the dumped mass; venting vacuum is a no-op returning 0.
    pub fn vent(&mut self) -> f64 {
        let dumped = self.air_kg;
        self.air_kg = 0.0;
        self.state = CabinPressureState::Vacuum;
        dumped
    }

    /// Repressurize toward the authored target from reserve: full charge or
    /// refusal (partial fills are future work). Already-over-target gas is
    /// retained and consumes no reserve. Returns consumed reserve mass.
    pub fn repress(&mut self, available_kg: f64) -> Result<f64, CabinError> {
        if !available_kg.is_finite() || available_kg < 0.0 {
            return Err(CabinError::InvalidCabin(
                "air reserve must be finite and non-negative".into(),
            ));
        }
        let needed = (self.full_charge_kg() - self.air_kg).max(0.0);
        if available_kg < needed {
            return Err(CabinError::InsufficientAir {
                needed_kg: needed,
                available_kg,
            });
        }
        if needed > 0.0 {
            self.air_kg += needed;
        }
        self.state = if self.air_kg > 0.0 {
            CabinPressureState::Pressurized
        } else {
            CabinPressureState::Vacuum
        };
        Ok(needed)
    }

    /// Hatch rule: opens with an empty air inventory, or with air aboard when
    /// every occupant in the volume is suited.
    pub fn hatch_may_open(&self, all_occupants_suited: bool) -> bool {
        self.air_kg == 0.0 || all_occupants_suited
    }

    /// EVA needs the hatch rule plus self-contained suits.
    pub fn eva_may_exit(&self, all_suited: bool, suits_self_contained: bool) -> bool {
        self.hatch_may_open(all_suited) && suits_self_contained
    }
}

/// Autopilot core capability tier (concept vocabulary from `docs/details/05`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AutopilotTier {
    /// Stability augmentation only.
    Hold,
    /// Executes maneuvers.
    Fly,
    /// Runs programs.
    Full,
}

/// One autopilot core aboard (power/comm dependencies are future wiring).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlCore {
    pub name: String,
    pub tier: AutopilotTier,
}

impl ControlCore {
    pub fn validate(&self) -> Result<(), CabinError> {
        if self.name.trim().is_empty() {
            return Err(CabinError::InvalidCabin("control core needs a name".into()));
        }
        Ok(())
    }
}

/// One pilot control station (flight-deck seat, pilot couch, cockpit).
/// Occupancy is manifest state: boarding sets it, EVA clears it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlStation {
    pub name: String,
    pub occupied: bool,
}

impl ControlStation {
    pub fn validate(&self) -> Result<(), CabinError> {
        if self.name.trim().is_empty() {
            return Err(CabinError::InvalidCabin(
                "control station needs a name".into(),
            ));
        }
        Ok(())
    }
}

/// Seat bundle class retained from the hangar cabin layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinSeatClass {
    Economy,
    Premium,
    Business,
    First,
    Ejection,
}

/// Operational role of a retained cabin seat record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinSeatRole {
    Passenger,
    FlightCrew,
    CabinAttendant,
}

/// Seat orientation retained for later crew presentation systems.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinSeatStyle {
    Upright,
    Couch,
    Ejection,
}

/// Pressure-suit feed type authored for one seat.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinSuitType {
    HoseFed,
    SelfContained,
}

/// Fitted cabin-equipment category.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinMonumentKind {
    Galley,
    Lavatory,
    Closet,
    FlightDeck,
    AvionicsRack,
}

/// Compiled seat and manifest metadata retained in the baked vehicle.
/// Mass values describe the already-baked manifest and are not independently
/// added to vehicle mass by `sim-core`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinSeat {
    pub name: String,
    pub position_body_m: DVec3,
    pub class: CabinSeatClass,
    pub role: CabinSeatRole,
    pub seat_style: CabinSeatStyle,
    pub occupied: bool,
    pub suited: bool,
    pub suit_type: CabinSuitType,
    pub seat_mass_kg: f64,
    pub occupant_mass_kg: f64,
    pub carry_on_mass_kg: f64,
    pub suit_mass_kg: f64,
}

impl CabinSeat {
    pub fn validate(&self) -> Result<(), CabinError> {
        let masses = [
            self.seat_mass_kg,
            self.occupant_mass_kg,
            self.carry_on_mass_kg,
            self.suit_mass_kg,
        ];
        if self.name.trim().is_empty()
            || !self.position_body_m.is_finite()
            || masses.iter().any(|mass| !mass.is_finite() || *mass < 0.0)
            || (self.occupied && self.occupant_mass_kg <= 0.0)
            || (self.suited && self.suit_mass_kg <= 0.0)
            || (!self.suited && self.suit_mass_kg > 0.0)
            || ((self.class == CabinSeatClass::Ejection)
                != (self.seat_style == CabinSeatStyle::Ejection))
            || (self.class == CabinSeatClass::Ejection && self.role != CabinSeatRole::FlightCrew)
        {
            return Err(CabinError::InvalidCabin(format!(
                "seat '{}' has invalid identity, position, role, or mass data",
                self.name
            )));
        }
        Ok(())
    }
}

/// Compiled fitted-equipment metadata retained in the baked vehicle.
/// Its mass is already included in baked vehicle mass properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinMonument {
    pub name: String,
    pub kind: CabinMonumentKind,
    pub position_body_m: DVec3,
    pub mass_kg: f64,
}

impl CabinMonument {
    pub fn validate(&self) -> Result<(), CabinError> {
        if self.name.trim().is_empty()
            || !self.position_body_m.is_finite()
            || !self.mass_kg.is_finite()
            || self.mass_kg < 0.0
        {
            return Err(CabinError::InvalidCabin(format!(
                "monument '{}' has invalid identity, position, or mass data",
                self.name
            )));
        }
        Ok(())
    }
}

/// Exit classification retained in the baked vehicle for cabin layout and
/// later evacuation systems. Exit dimensions and capacity are screened by
/// the hangar compiler against the applicable aircraft-design rule set.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinExitType {
    #[serde(rename = "type-a")]
    TypeA,
    #[serde(rename = "type-b")]
    TypeB,
    #[serde(rename = "type-c")]
    TypeC,
    #[serde(rename = "type-i")]
    TypeI,
    #[serde(rename = "type-ii")]
    TypeII,
    #[serde(rename = "type-iii")]
    TypeIII,
    #[serde(rename = "type-iv")]
    TypeIV,
}

/// Fuselage side of a cabin exit in the vehicle body frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinExitSide {
    Left,
    Right,
}

/// Static cabin exit metadata. This does not model a moving door or shell
/// cutout; it preserves the validated authored exit in the baked vehicle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinExit {
    pub name: String,
    pub pair_id: String,
    pub position_body_m: DVec3,
    pub side: CabinExitSide,
    pub exit_type: CabinExitType,
    pub opening_width_m: f64,
    pub opening_height_m: f64,
}

impl CabinExit {
    pub fn validate(&self) -> Result<(), CabinError> {
        if self.name.trim().is_empty()
            || self.pair_id.trim().is_empty()
            || !self.position_body_m.is_finite()
            || !self.opening_width_m.is_finite()
            || self.opening_width_m <= 0.0
            || !self.opening_height_m.is_finite()
            || self.opening_height_m <= 0.0
        {
            return Err(CabinError::InvalidCabin(format!(
                "exit '{}' needs a pair ID, finite position, and positive opening dimensions",
                self.name
            )));
        }
        Ok(())
    }
}

/// Why the craft does or does not answer the controls.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AuthorityReason {
    PilotAboard,
    AutopilotCore,
    NoPilotNoCore,
    /// Predates crew/control modeling entirely (legacy assets stay flyable).
    Unrestricted,
}

/// Presence-based control authority: a pilot at a station wins, else any
/// core flies, else nobody does. `declares_crew_systems` marks vehicles
/// with cabins, stations, or cores aboard; without any of those the asset
/// predates crew modeling and stays unrestricted (legacy migration, like
/// empty collision geometry).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlAuthority {
    pub controllable: bool,
    pub reason: AuthorityReason,
}

pub fn control_authority(
    stations: &[ControlStation],
    cores: &[ControlCore],
    declares_crew_systems: bool,
) -> ControlAuthority {
    if stations.iter().any(|station| station.occupied) {
        return ControlAuthority {
            controllable: true,
            reason: AuthorityReason::PilotAboard,
        };
    }
    if !cores.is_empty() {
        return ControlAuthority {
            controllable: true,
            reason: AuthorityReason::AutopilotCore,
        };
    }
    if !declares_crew_systems {
        return ControlAuthority {
            controllable: true,
            reason: AuthorityReason::Unrestricted,
        };
    }
    ControlAuthority {
        controllable: false,
        reason: AuthorityReason::NoPilotNoCore,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_cabin() -> PressurizedCabin {
        PressurizedCabin::new("cabin", 2.5, 101.325, 293.15, 0.21, 3.0).unwrap()
    }

    #[test]
    fn full_charge_matches_ideal_gas() {
        let cabin = test_cabin();
        let expected = 101325.0 / (287.05 * 293.15) * 2.5;
        assert!((cabin.full_charge_kg() - expected).abs() / expected < 1e-12);
        assert!((cabin.o2_kg() - 3.0 * 0.21 * 32.0 / 28.97).abs() < 1e-12);
    }

    #[test]
    fn vent_dumps_and_repress_consumes() {
        let mut cabin = test_cabin();
        assert_eq!(cabin.state, CabinPressureState::Pressurized);
        let dumped = cabin.vent();
        assert!((dumped - 3.0).abs() < 1e-12);
        assert_eq!(cabin.air_kg, 0.0);
        assert_eq!(cabin.state, CabinPressureState::Vacuum);
        assert_eq!(cabin.vent(), 0.0);
        // Short reserve refuses instead of partial-filling.
        assert!(cabin.repress(1.0).is_err());
        let needed = cabin.full_charge_kg();
        let consumed = cabin.repress(needed + 10.0).unwrap();
        assert!((consumed - needed).abs() < 1e-12);
        assert_eq!(cabin.state, CabinPressureState::Pressurized);
    }

    #[test]
    fn hatch_and_eva_rules() {
        let mut cabin = test_cabin();
        // Pressurized: hatch only for suited volumes, EVA only on
        // self-contained suits.
        assert!(!cabin.hatch_may_open(false));
        assert!(cabin.hatch_may_open(true));
        assert!(!cabin.eva_may_exit(true, false));
        assert!(cabin.eva_may_exit(true, true));
        cabin.vent();
        assert!(cabin.hatch_may_open(false));
        // Vacuum hatch is open; suited self-contained crew may still exit.
        assert!(cabin.eva_may_exit(false, true));
        assert!(!cabin.eva_may_exit(false, false));

        // Inventory, rather than a stale serialized label, determines
        // whether opening would expose pressurized gas to vacuum.
        cabin.air_kg = 1.0;
        cabin.state = CabinPressureState::Vacuum;
        assert!(!cabin.hatch_may_open(false));
        assert!(cabin.hatch_may_open(true));
    }

    #[test]
    fn authority_priority_is_pilot_then_core() {
        let pilot = ControlStation {
            name: "left-seat".into(),
            occupied: true,
        };
        let empty_station = ControlStation {
            name: "left-seat".into(),
            occupied: false,
        };
        let core = ControlCore {
            name: "core".into(),
            tier: AutopilotTier::Full,
        };
        assert_eq!(
            control_authority(&[], &[], false),
            ControlAuthority {
                controllable: true,
                reason: AuthorityReason::Unrestricted,
            }
        );
        // Declared crew systems with nobody aboard lock the craft.
        assert!(!control_authority(&[], &[], true).controllable);
        assert_eq!(
            control_authority(std::slice::from_ref(&empty_station), &[], true).reason,
            AuthorityReason::NoPilotNoCore
        );
        assert!(!control_authority(std::slice::from_ref(&empty_station), &[], true).controllable);
        assert_eq!(
            control_authority(
                std::slice::from_ref(&empty_station),
                std::slice::from_ref(&core),
                true
            )
            .reason,
            AuthorityReason::AutopilotCore
        );
        // Pilot wins over the core.
        assert_eq!(
            control_authority(
                std::slice::from_ref(&pilot),
                std::slice::from_ref(&core),
                true
            )
            .reason,
            AuthorityReason::PilotAboard
        );
    }

    #[test]
    fn bad_cabins_fail_closed() {
        assert!(PressurizedCabin::new("", 2.5, 101.0, 293.0, 0.21, 1.0).is_err());
        assert!(PressurizedCabin::new("c", 0.0, 101.0, 293.0, 0.21, 0.0).is_err());
        assert!(PressurizedCabin::new("c", 2.5, 600.0, 293.0, 0.21, 0.0).is_err());
        assert!(PressurizedCabin::new("c", 2.5, 101.0, 293.0, 0.0, 0.0).is_err());
        // Air above the full charge refuses.
        assert!(PressurizedCabin::new("c", 2.5, 101.0, 293.0, 0.21, 1.0e6).is_err());
        assert!(
            ControlCore {
                name: "".into(),
                tier: AutopilotTier::Hold,
            }
            .validate()
            .is_err()
        );
    }
}
