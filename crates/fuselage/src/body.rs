//! Procedural fuselage authoring model.
//!
//! One representation serves both editing depths from the design doc: simple
//! parameter presets (cylinder, cone, ogive, Juno-style stacks,
//! SimplePlanes-style blocks in `golden`) generate station lists, and the
//! advanced mode edits [`BodyStation`] splines directly. Gameplay purpose
//! lives in [`InteriorRegion`] and [`BodyPort`] records, never in the
//! geometric primitive.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::FuselageError;
use crate::section::BodyStation;

/// Default light-aircraft seat mass (kg each) when a crew cabin omits it.
fn default_seat_mass_kg_each() -> f64 {
    12.0
}

/// Pure stored fluid for standalone component tanks (no chamber thermo:
/// just storage density for mass/volume bookkeeping). Component densities
/// match the split-tank table so a manual LOX + methane pair agrees with
/// the auto-split `Bipropellant` region.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum StoredFluid {
    Lox,
    LiquidMethane,
    LiquidHydrogen,
    Rp1,
    Nto,
    Mmh,
    Hydrazine,
    Water,
}

impl StoredFluid {
    /// Storable density in kg/m^3.
    pub fn density_kg_m3(self) -> f64 {
        match self {
            Self::Lox => 1141.0,
            Self::LiquidMethane => 422.0,
            Self::LiquidHydrogen => 71.0,
            Self::Rp1 => 810.0,
            Self::Nto => 1440.0,
            Self::Mmh => 878.0,
            Self::Hydrazine => 1008.0,
            Self::Water => 1000.0,
        }
    }
}

/// Pressure-shell shape for one tank region. The cylinder maps the loft
/// volume to an equivalent diameter over the region length; the sphere
/// sizes from the volume alone and suits compact storable tanks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TankShell {
    Cylinder,
    Sphere,
}

/// How crew places are oriented: upright aircraft-style seats or
/// reclined capsule couches. The tag travels to the compiled interior so
/// renderer and crew systems orient bodies later; anchors already encode
/// the transverse couch rows. Passenger cabins use `Upright` with authored
/// multi-abreast seat blocks; fighter seats use `Ejection`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SeatStyle {
    #[default]
    Upright,
    Couch,
    Ejection,
}

/// Pressure-suit feed: hose-fed suits borrow vehicle air, self-contained
/// suits (EVA) carry their own loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SuitType {
    #[default]
    HoseFed,
    SelfContained,
}

/// What a slice of the usable interior does. Geometry and structure are
/// shared; purpose is assigned per longitudinal region.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RegionKind {
    /// Propellant volume: feeds the tank pipeline with a real propellant.
    /// Optional per-tank pressure/material override the body layout so
    /// one hull can carry dissimilar tanks (cryo + storable, different
    /// pressures or shell alloys). `None` keeps the layout default.
    Tank {
        propellant: thessa_sim_core::Propellant,
        /// Usable fill fraction in `[0, 1]` (ullage and traps excluded).
        fill_fraction: f64,
        #[serde(default)]
        pressure_pa: Option<f64>,
        #[serde(default)]
        material: Option<thessa_sim_core::ChamberMaterial>,
        /// Shell shape; `None` keeps the equivalent cylinder.
        #[serde(default)]
        shell: Option<TankShell>,
    },
    /// Standalone pure-fluid tank (manual oxidizer/fuel placement, water,
    /// RCS monoprop): one region is one tank with the stored density.
    /// Use this for hand-split pairs; use `Bipropellant` for the automatic
    /// mixture-ratio split.
    FluidTank {
        fluid: StoredFluid,
        /// Usable fill fraction in `[0, 1]`.
        fill_fraction: f64,
        #[serde(default)]
        pressure_pa: Option<f64>,
        #[serde(default)]
        material: Option<thessa_sim_core::ChamberMaterial>,
        /// Shell shape; `None` keeps the equivalent cylinder.
        #[serde(default)]
        shell: Option<TankShell>,
    },
    /// Bipropellant volume split into two tanks at compile time:
    /// oxidizer aft, fuel forward, divided axially so sub-volumes match
    /// the mixture ratio and component densities. One region authors
    /// both tanks; the compiler emits `{name}-ox` and `{name}-fuel`.
    Bipropellant {
        propellant: thessa_sim_core::Propellant,
        /// Usable fill fraction in `[0, 1]`, applied to both tanks.
        fill_fraction: f64,
        /// Oxidizer-to-fuel mass ratio; `None` uses the reference ratio.
        #[serde(default)]
        mixture_ratio: Option<f64>,
        /// Shared pressure fallback when per-component values are absent.
        #[serde(default)]
        pressure_pa: Option<f64>,
        #[serde(default)]
        oxidizer_pressure_pa: Option<f64>,
        #[serde(default)]
        fuel_pressure_pa: Option<f64>,
        #[serde(default)]
        material: Option<thessa_sim_core::ChamberMaterial>,
        #[serde(default)]
        oxidizer_material: Option<thessa_sim_core::ChamberMaterial>,
        #[serde(default)]
        fuel_material: Option<thessa_sim_core::ChamberMaterial>,
        /// Shell shape for both sub-tanks; mix shapes via two `FluidTank`
        /// regions instead.
        #[serde(default)]
        shell: Option<TankShell>,
    },
    /// Crew or passenger volume (unfitted shell; mass is future module work).
    Cabin,
    /// Crewed cabin with seats: `seats` places are distributed along the
    /// region; seat plus occupant mass rides the hull at the region
    /// centroid like cargo manifest. Occupant mass defaults to 0
    /// (unoccupied ferry) so crew loading stays explicit. Seat anchors
    /// (one position per place, forward-facing, on the section centerline)
    /// are exposed in the compiled interior for renderer/crew systems.
    /// Suited crews may fly dry cabins; `control_station` marks pilot
    /// posts for control authority. Advanced passenger cabins use the
    /// separate `cabin_layout` authoring model on a `Cabin` region.
    Crew {
        seats: u32,
        #[serde(default = "default_seat_mass_kg_each")]
        seat_mass_kg_each: f64,
        #[serde(default)]
        occupant_mass_kg_each: f64,
        /// Longitudinal pitch between places; `None` spreads evenly.
        #[serde(default)]
        seat_pitch_m: Option<f64>,
        /// Places per transverse row (`None` = single column). Couches
        /// ride side-by-side like Apollo; airplane cabins will use wider
        /// upright rows.
        #[serde(default)]
        abreast: Option<u32>,
        /// Upright seats, reclined couches, or ejection seats.
        #[serde(default)]
        seat_style: SeatStyle,
        /// Crew wear pressure suits (cabin air optional, §7 of the cabin doc).
        #[serde(default)]
        suited: bool,
        /// Suit mass each in kg (counts only when suited).
        #[serde(default)]
        suit_mass_kg_each: f64,
        /// Hose-fed (vehicle air) vs self-contained (EVA-capable).
        #[serde(default)]
        suit_type: SuitType,
        /// Pilot post: occupancy here grants control authority.
        #[serde(default)]
        control_station: bool,
    },
    /// Pressurized cargo volume plus explicit manifest mass.
    Cargo {
        /// Declared cargo/manifest mass carried in this region (kg).
        payload_mass_kg: f64,
    },
    /// Avionics/equipment bay (dry, unpressurized by default).
    Avionics,
    /// Reserved but unequipped volume.
    Empty,
}

/// Default cabin temperature in K (20 C) when a pressurized region omits it.
fn default_cabin_temp_k() -> f64 {
    293.15
}

/// Default oxygen volume fraction (sea-level air) for a pressurized region.
fn default_o2_fraction() -> f64 {
    0.21
}

/// Breathing atmosphere held by a pressurized region (first ECLSS brick:
/// pressure inventory plus oxygen mass for future metabolic bookkeeping).
/// Tanks carry their own `tank_pressure_pa` and refuse this field.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CabinAtmosphere {
    /// Absolute pressure in kPa (sea-level cabin ~101.3).
    pub pressure_kpa: f64,
    /// Temperature in K.
    #[serde(default = "default_cabin_temp_k")]
    pub temp_k: f64,
    /// Oxygen volume fraction.
    #[serde(default = "default_o2_fraction")]
    pub o2_fraction: f64,
}

impl CabinAtmosphere {
    /// Sea-level cabin: 101.325 kPa, 20 C, 21% O2.
    pub fn sea_level() -> Self {
        Self {
            pressure_kpa: 101.325,
            temp_k: default_cabin_temp_k(),
            o2_fraction: default_o2_fraction(),
        }
    }

    pub fn validate(&self, region: &str) -> Result<(), FuselageError> {
        if !self.pressure_kpa.is_finite() || self.pressure_kpa <= 0.0 || self.pressure_kpa > 500.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{region}' needs pressure_kpa in (0, 500]"
            )));
        }
        if !self.temp_k.is_finite() || self.temp_k < 180.0 || self.temp_k > 350.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{region}' needs temp_k in [180, 350]"
            )));
        }
        if !self.o2_fraction.is_finite() || self.o2_fraction <= 0.0 || self.o2_fraction > 1.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{region}' needs o2_fraction in (0, 1]"
            )));
        }
        Ok(())
    }
}

/// Cabin seat bundle. These values are starting points only; each seat block
/// may author its own width and fitted mass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SeatClass {
    Economy,
    Premium,
    Business,
    First,
    Ejection,
}

impl SeatClass {
    pub const fn suggested_width_m(self) -> f64 {
        match self {
            Self::Economy => 0.44,
            Self::Premium => 0.47,
            Self::Business => 0.55,
            Self::First => 0.65,
            Self::Ejection => 0.55,
        }
    }

    pub const fn suggested_mass_kg_each(self) -> f64 {
        match self {
            Self::Economy => 11.0,
            Self::Premium => 18.0,
            Self::Business => 60.0,
            Self::First => 100.0,
            Self::Ejection => 110.0,
        }
    }
}

/// Function of a seat block for crew-complement and control-authority baking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CabinSeatRole {
    #[default]
    Passenger,
    FlightCrew,
    CabinAttendant,
}

/// Suit configuration for one flattened seat index in a [`SeatBlock`].
/// Indices follow compilation order: rows first, then column groups and
/// places from left to right within each row.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SeatSuitOverride {
    pub seat_index: u32,
    pub suited: bool,
    pub suit_mass_kg_each: f64,
    #[serde(default)]
    pub suit_type: SuitType,
}

/// One axial seat bank. `columns` gives the number of seats in each bank,
/// with the authored aisle widths between them, e.g. `[3, 4, 3]`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SeatBlock {
    pub name: String,
    pub class: SeatClass,
    #[serde(default)]
    pub role: CabinSeatRole,
    /// First row's leading edge in body metres.
    pub x0_m: f64,
    pub rows: u32,
    pub columns: Vec<u32>,
    pub pitch_m: f64,
    /// One width between each adjacent column group.
    pub aisle_widths_m: Vec<f64>,
    /// Total clearance budget for both outer walls and armrests.
    pub wall_clearance_m: f64,
    /// Optional override of the class width suggestion.
    #[serde(default)]
    pub seat_width_m: Option<f64>,
    /// Optional override of the class fitted-mass suggestion.
    #[serde(default)]
    pub seat_mass_kg_each: Option<f64>,
    /// Initial manifest occupancy; unoccupied places still carry their seats.
    #[serde(default)]
    pub occupants: u32,
    #[serde(default)]
    pub occupant_mass_kg_each: f64,
    #[serde(default)]
    pub carry_on_kg_each: f64,
    #[serde(default)]
    pub suited: bool,
    #[serde(default)]
    pub suit_mass_kg_each: f64,
    #[serde(default)]
    pub suit_type: SuitType,
    /// Sparse per-place suit configurations overriding the block defaults.
    #[serde(default)]
    pub suit_overrides: Vec<SeatSuitOverride>,
    #[serde(default)]
    pub seat_style: SeatStyle,
}

impl SeatBlock {
    pub fn seat_width_m(&self) -> f64 {
        self.seat_width_m
            .unwrap_or_else(|| self.class.suggested_width_m())
    }

    pub fn seat_mass_kg_each(&self) -> f64 {
        self.seat_mass_kg_each
            .unwrap_or_else(|| self.class.suggested_mass_kg_each())
    }

    fn places(&self, region: &str) -> Result<u32, FuselageError> {
        let seats_per_row = self
            .columns
            .iter()
            .try_fold(0_u32, |sum, columns| sum.checked_add(*columns));
        let Some(places) = seats_per_row.and_then(|per_row| per_row.checked_mul(self.rows)) else {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' in '{region}' has too many places",
                self.name
            )));
        };
        Ok(places)
    }

    fn validate(&self, region: &InteriorRegion) -> Result<u32, FuselageError> {
        if self.name.trim().is_empty()
            || self.rows == 0
            || self.columns.is_empty()
            || self.columns.len() > 8
            || self.columns.contains(&0)
            || self.aisle_widths_m.len() != self.columns.len().saturating_sub(1)
        {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block in '{}' needs a name, rows, non-empty column groups, and one aisle per group boundary",
                region.name
            )));
        }
        let places = self.places(&region.name)?;
        if places == 0 || places > 10_000 {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' in '{}' needs 1..=10000 places",
                self.name, region.name
            )));
        }
        if !self.x0_m.is_finite()
            || !self.pitch_m.is_finite()
            || self.pitch_m <= 0.0
            || !self.wall_clearance_m.is_finite()
            || self.wall_clearance_m < 0.0
            || !self.seat_width_m().is_finite()
            || self.seat_width_m() <= 0.0
            || !self.seat_mass_kg_each().is_finite()
            || self.seat_mass_kg_each() < 0.0
            || self
                .aisle_widths_m
                .iter()
                .any(|width| !width.is_finite() || *width <= 0.0)
            || !self.occupant_mass_kg_each.is_finite()
            || self.occupant_mass_kg_each < 0.0
            || !self.carry_on_kg_each.is_finite()
            || self.carry_on_kg_each < 0.0
            || !self.suit_mass_kg_each.is_finite()
            || self.suit_mass_kg_each < 0.0
        {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' in '{}' has invalid dimensions or mass inputs",
                self.name, region.name
            )));
        }
        if self.occupants > places {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' has {} occupants for {places} places",
                self.name, self.occupants
            )));
        }
        if (self.class == SeatClass::Ejection) != (self.seat_style == SeatStyle::Ejection)
            || (self.class == SeatClass::Ejection && self.role != CabinSeatRole::FlightCrew)
        {
            return Err(FuselageError::InvalidInterior(format!(
                "ejection seat block '{}' must use the ejection style and flight-crew role",
                self.name
            )));
        }
        if self.occupants > 0 && self.occupant_mass_kg_each <= 0.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "occupied seat block '{}' needs occupant_mass_kg_each > 0",
                self.name
            )));
        }
        if self.suited && self.suit_mass_kg_each <= 0.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "suited seat block '{}' needs suit_mass_kg_each > 0",
                self.name
            )));
        }
        if !self.suited && self.suit_mass_kg_each > 0.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "unsuited seat block '{}' cannot carry suit mass",
                self.name
            )));
        }
        if self.suit_overrides.len() > places as usize {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' has more suit overrides than places",
                self.name
            )));
        }
        let mut suit_overrides =
            std::collections::HashMap::with_capacity(self.suit_overrides.len());
        for suit_override in &self.suit_overrides {
            if suit_override.seat_index >= places
                || !suit_override.suit_mass_kg_each.is_finite()
                || suit_override.suit_mass_kg_each < 0.0
                || (suit_override.suited && suit_override.suit_mass_kg_each <= 0.0)
                || (!suit_override.suited && suit_override.suit_mass_kg_each > 0.0)
                || suit_overrides
                    .insert(suit_override.seat_index, suit_override)
                    .is_some()
            {
                return Err(FuselageError::InvalidInterior(format!(
                    "seat block '{}' has an invalid, duplicate, or out-of-range suit override for place {}",
                    self.name, suit_override.seat_index
                )));
            }
        }
        if region.atmosphere.is_none()
            && (0..places).any(|seat_index| {
                !suit_overrides
                    .get(&seat_index)
                    .map(|suit_override| suit_override.suited)
                    .unwrap_or(self.suited)
            })
        {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' has an unsuited place without cabin atmosphere",
                self.name
            )));
        }
        let end_x_m = self.x0_m + self.rows as f64 * self.pitch_m;
        if !end_x_m.is_finite() || self.x0_m < region.x0_m || end_x_m > region.x1_m {
            return Err(FuselageError::InvalidInterior(format!(
                "seat block '{}' lies outside region '{}' axial range",
                self.name, region.name
            )));
        }
        Ok(places)
    }
}

/// Fitted equipment occupying a full-width axial footprint on one deck.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MonumentKind {
    Galley,
    Lavatory,
    Closet,
    FlightDeck,
    AvionicsRack,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Monument {
    pub name: String,
    pub kind: MonumentKind,
    pub x0_m: f64,
    pub x1_m: f64,
    pub mass_kg: f64,
}

/// Side of a floor-level cabin door in the body frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DoorSide {
    Left,
    Right,
}

/// Reference emergency-exit class from 14 CFR 25.807.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ExitType {
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExitSpec {
    pub opening_width_m: f64,
    pub opening_height_m: f64,
    /// Maximum passenger seats credited to one exit by 14 CFR 25.807(g).
    pub passenger_seats: u32,
}

impl ExitType {
    /// Minimum rectangular opening and per-side seat credit from 14 CFR
    /// 25.807(a), (g). Type IV is an overwing exit under that rule.
    pub const fn spec(self) -> ExitSpec {
        const INCH_M: f64 = 0.0254;
        let (width_in, height_in, passenger_seats) = match self {
            Self::TypeA => (42.0, 72.0, 110),
            Self::TypeB => (32.0, 72.0, 75),
            Self::TypeC => (30.0, 48.0, 55),
            Self::TypeI => (24.0, 48.0, 45),
            Self::TypeII => (20.0, 44.0, 40),
            Self::TypeIII => (20.0, 36.0, 35),
            Self::TypeIV => (19.0, 26.0, 9),
        };
        ExitSpec {
            opening_width_m: width_in * INCH_M,
            opening_height_m: height_in * INCH_M,
            passenger_seats,
        }
    }

    const fn size_rank(self) -> u8 {
        match self {
            Self::TypeIV => 0,
            Self::TypeIII => 1,
            Self::TypeII => 2,
            Self::TypeI => 3,
            Self::TypeC => 4,
            Self::TypeB => 5,
            Self::TypeA => 6,
        }
    }
}

/// Floor-level doorway. Left and right exits share an explicit `pair_id`;
/// the smaller rating in each pair limits its passenger-seat credit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinDoor {
    pub name: String,
    pub pair_id: String,
    pub x_m: f64,
    pub side: DoorSide,
    pub rating: ExitType,
    /// Axial clear zone, including the door's opening width.
    pub clear_zone_length_m: f64,
}

/// One passenger or crew deck inside a cabin region.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinDeck {
    pub name: String,
    pub floor_z_m: f64,
    pub min_headroom_m: f64,
    #[serde(default)]
    pub blocks: Vec<SeatBlock>,
    #[serde(default)]
    pub monuments: Vec<Monument>,
    #[serde(default)]
    pub doors: Vec<CabinDoor>,
}

/// Unified cabin plan. Legacy capsule `Crew` regions do not need a layout;
/// advanced passenger cabins attach this value to a `Cabin` region.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CabinLayout {
    pub decks: Vec<CabinDeck>,
}

/// Built-in cabin-layout recipe that can be selected directly from a vehicle
/// asset. Coordinates are fitted to the containing [`InteriorRegion`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum CabinLayoutPreset {
    #[serde(rename = "747-like")]
    B747Like {
        main_floor_z_m: f64,
        upper_floor_z_m: f64,
    },
    #[serde(rename = "concorde-like")]
    ConcordeLike { floor_z_m: f64 },
    Fighter {
        floor_z_m: f64,
        #[serde(default = "one_pilot")]
        pilots: u32,
    },
}

fn one_pilot() -> u32 {
    1
}

impl CabinLayoutPreset {
    /// Expand this recipe over an interior region's authored axial range.
    pub fn build_for_region(self, x0_m: f64, x1_m: f64) -> Result<CabinLayout, FuselageError> {
        match self {
            Self::B747Like {
                main_floor_z_m,
                upper_floor_z_m,
            } => CabinLayout::preset_747_like(x0_m, x1_m, main_floor_z_m, upper_floor_z_m),
            Self::ConcordeLike { floor_z_m } => {
                CabinLayout::preset_concorde_like(x0_m, x1_m, floor_z_m)
            }
            Self::Fighter { floor_z_m, pilots } => {
                CabinLayout::preset_fighter(x0_m, x1_m, floor_z_m, pilots)
            }
        }
    }
}

impl CabinLayout {
    /// 747-like two-deck layout: 360 main-deck and 96 upper-deck passenger
    /// places, bilateral Type-A exit pairs, flight crew, attendants, and
    /// service monuments. Coordinates are in the supplied body axial range.
    pub fn preset_747_like(
        x0_m: f64,
        x1_m: f64,
        main_floor_z_m: f64,
        upper_floor_z_m: f64,
    ) -> Result<Self, FuselageError> {
        if !x0_m.is_finite()
            || !x1_m.is_finite()
            || !main_floor_z_m.is_finite()
            || !upper_floor_z_m.is_finite()
            || x1_m - x0_m < 45.5
        {
            return Err(FuselageError::InvalidInterior(
                "747-like preset needs a finite region at least 45.5 m long and finite deck floors"
                    .into(),
            ));
        }
        let mut main = CabinDeck {
            name: "main".into(),
            floor_z_m: main_floor_z_m,
            min_headroom_m: 2.0,
            blocks: Vec::new(),
            monuments: vec![
                Monument {
                    name: "aft-galley".into(),
                    kind: MonumentKind::Galley,
                    x0_m: x0_m + 0.1,
                    x1_m: x0_m + 1.1,
                    mass_kg: 900.0,
                },
                Monument {
                    name: "aft-lavatory".into(),
                    kind: MonumentKind::Lavatory,
                    x0_m: x0_m + 1.1,
                    x1_m: x0_m + 1.9,
                    mass_kg: 180.0,
                },
            ],
            doors: Vec::new(),
        };
        let mut cursor = x0_m + 2.0;
        let main_segments = [
            ("economy-aft", SeatClass::Economy, 10, vec![3, 4, 3]),
            ("economy-mid-aft", SeatClass::Economy, 9, vec![3, 4, 3]),
            ("economy-mid-forward", SeatClass::Economy, 9, vec![3, 4, 3]),
            ("premium-aft", SeatClass::Premium, 3, vec![2, 4, 2]),
            ("premium-forward", SeatClass::Premium, 3, vec![2, 4, 2]),
            ("business", SeatClass::Business, 8, vec![2, 2]),
        ];
        let main_segment_count = main_segments.len();
        for (index, (name, class, rows, columns)) in main_segments.into_iter().enumerate() {
            main.blocks.push(preset_block(
                name,
                class,
                CabinSeatRole::Passenger,
                cursor,
                rows,
                columns,
                0.81,
                if class == SeatClass::Business {
                    vec![0.51]
                } else {
                    vec![0.51, 0.51]
                },
                0.4,
                false,
                0,
            ));
            cursor += rows as f64 * 0.81;
            if index + 1 < main_segment_count {
                append_exit_pair(
                    &mut main.doors,
                    "main",
                    index + 1,
                    cursor + 0.6,
                    ExitType::TypeA,
                    1.2,
                );
                cursor += 1.2;
            }
        }
        main.blocks.push(preset_block(
            "cabin-attendants",
            SeatClass::Economy,
            CabinSeatRole::CabinAttendant,
            x1_m - 5.0,
            5,
            vec![2],
            0.81,
            vec![],
            0.2,
            false,
            0,
        ));
        main.blocks.push(preset_block(
            "flight-crew",
            SeatClass::Business,
            CabinSeatRole::FlightCrew,
            x1_m - 0.9,
            1,
            vec![2],
            0.81,
            vec![],
            0.2,
            false,
            0,
        ));

        let mut upper = CabinDeck {
            name: "upper".into(),
            floor_z_m: upper_floor_z_m,
            min_headroom_m: 2.0,
            blocks: Vec::new(),
            monuments: vec![
                Monument {
                    name: "upper-galley".into(),
                    kind: MonumentKind::Galley,
                    x0_m: x0_m + 0.1,
                    x1_m: x0_m + 1.1,
                    mass_kg: 450.0,
                },
                Monument {
                    name: "upper-lavatory".into(),
                    kind: MonumentKind::Lavatory,
                    x0_m: x0_m + 1.1,
                    x1_m: x0_m + 1.9,
                    mass_kg: 120.0,
                },
            ],
            doors: Vec::new(),
        };
        cursor = x0_m + 2.0;
        for (index, rows) in [5, 5, 6].into_iter().enumerate() {
            upper.blocks.push(preset_block(
                ["upper-aft", "upper-mid", "upper-forward"][index],
                SeatClass::Economy,
                CabinSeatRole::Passenger,
                cursor,
                rows,
                vec![3, 3],
                0.81,
                vec![0.51],
                0.4,
                false,
                0,
            ));
            cursor += rows as f64 * 0.81;
            if index < 2 {
                append_exit_pair(
                    &mut upper.doors,
                    "upper",
                    index + 1,
                    cursor + 0.6,
                    ExitType::TypeA,
                    1.2,
                );
                cursor += 1.2;
            }
        }
        Ok(Self {
            decks: vec![main, upper],
        })
    }

    /// Concorde-like slender single-deck layout with 100 passenger places,
    /// three bilateral Type-I exit pairs, two pilots, and two attendants.
    pub fn preset_concorde_like(
        x0_m: f64,
        x1_m: f64,
        floor_z_m: f64,
    ) -> Result<Self, FuselageError> {
        if !x0_m.is_finite() || !x1_m.is_finite() || !floor_z_m.is_finite() || x1_m - x0_m < 29.3 {
            return Err(FuselageError::InvalidInterior(
                "Concorde-like preset needs a finite region at least 29.3 m long and a finite deck floor"
                    .into(),
            ));
        }
        let mut deck = CabinDeck {
            name: "main".into(),
            floor_z_m,
            min_headroom_m: 1.8,
            blocks: Vec::new(),
            monuments: vec![
                Monument {
                    name: "aft-galley".into(),
                    kind: MonumentKind::Galley,
                    x0_m: x0_m + 0.1,
                    x1_m: x0_m + 1.0,
                    mass_kg: 400.0,
                },
                Monument {
                    name: "aft-lavatory".into(),
                    kind: MonumentKind::Lavatory,
                    x0_m: x0_m + 1.0,
                    x1_m: x0_m + 1.8,
                    mass_kg: 100.0,
                },
            ],
            doors: Vec::new(),
        };
        let mut cursor = x0_m + 2.0;
        for (index, (name, class, rows)) in [
            ("economy-aft", SeatClass::Economy, 7),
            ("premium-aft", SeatClass::Premium, 6),
            ("economy-forward", SeatClass::Economy, 6),
            ("premium-forward", SeatClass::Premium, 6),
        ]
        .into_iter()
        .enumerate()
        {
            let mut block = preset_block(
                name,
                class,
                CabinSeatRole::Passenger,
                cursor,
                rows,
                vec![2, 2],
                0.86,
                vec![0.43],
                0.2,
                false,
                0,
            );
            // Premium class width is overridden to fit the narrow inner loft.
            if class == SeatClass::Premium {
                block.seat_width_m = Some(0.43);
            }
            deck.blocks.push(block);
            cursor += rows as f64 * 0.86;
            if index < 3 {
                append_exit_pair(
                    &mut deck.doors,
                    "main",
                    index + 1,
                    cursor + 0.4,
                    ExitType::TypeI,
                    0.8,
                );
                cursor += 0.8;
            }
        }
        deck.blocks.push(preset_block(
            "cabin-attendants",
            SeatClass::Economy,
            CabinSeatRole::CabinAttendant,
            x1_m - 2.8,
            2,
            vec![1],
            0.86,
            vec![],
            0.2,
            false,
            0,
        ));
        deck.blocks.push(preset_block(
            "flight-crew",
            SeatClass::Business,
            CabinSeatRole::FlightCrew,
            x1_m - 1.0,
            1,
            vec![2],
            0.86,
            vec![],
            0.2,
            false,
            0,
        ));
        Ok(Self { decks: vec![deck] })
    }

    /// Single- or tandem-seat fighter cockpit. The pilot(s) are occupied,
    /// suited, and use hose-fed suit mass by default.
    pub fn preset_fighter(
        x0_m: f64,
        x1_m: f64,
        floor_z_m: f64,
        pilots: u32,
    ) -> Result<Self, FuselageError> {
        if !x0_m.is_finite()
            || !x1_m.is_finite()
            || !floor_z_m.is_finite()
            || !(1..=2).contains(&pilots)
            || x1_m - x0_m < pilots as f64 * 1.4
        {
            return Err(FuselageError::InvalidInterior(
                "fighter preset needs one or two pilots, a finite deck, and at least 1.4 m per place"
                    .into(),
            ));
        }
        let rows = pilots;
        let pitch = 1.4;
        let occupied_length = rows as f64 * pitch;
        let block = preset_block(
            if pilots == 1 {
                "pilot"
            } else {
                "tandem-pilots"
            },
            SeatClass::Ejection,
            CabinSeatRole::FlightCrew,
            0.5 * (x0_m + x1_m - occupied_length),
            rows,
            vec![1],
            pitch,
            vec![],
            0.1,
            true,
            pilots,
        );
        Ok(Self {
            decks: vec![CabinDeck {
                name: "cockpit".into(),
                floor_z_m,
                min_headroom_m: 0.9,
                blocks: vec![block],
                monuments: Vec::new(),
                doors: Vec::new(),
            }],
        })
    }

    fn validate(&self, region: &InteriorRegion) -> Result<(), FuselageError> {
        if self.decks.is_empty() || self.decks.len() > 4 {
            return Err(FuselageError::InvalidInterior(format!(
                "cabin '{}' needs 1..=4 decks",
                region.name
            )));
        }
        let mut deck_names = std::collections::HashSet::new();
        let mut total_passenger_places = 0_u32;
        let mut total_seat_places = 0_u32;
        let mut total_flight_crew_places = 0_u32;
        let mut total_attendant_places = 0_u32;
        for deck in &self.decks {
            if deck.name.trim().is_empty()
                || !deck_names.insert(deck.name.as_str())
                || !deck.floor_z_m.is_finite()
                || !deck.min_headroom_m.is_finite()
                || deck.min_headroom_m <= 0.0
                || deck.blocks.len() > 256
                || deck.monuments.len() > 256
                || deck.doors.len() > 128
            {
                return Err(FuselageError::InvalidInterior(format!(
                    "cabin '{}' has an invalid deck, duplicate deck name, or exceeds its per-deck record limits",
                    region.name
                )));
            }
            let mut names = std::collections::HashSet::new();
            let mut passenger_places = 0_u32;
            let mut footprints = Vec::new();
            let mut row_positions = Vec::new();
            for block in &deck.blocks {
                if !names.insert(block.name.as_str()) {
                    return Err(FuselageError::InvalidInterior(format!(
                        "deck '{}' repeats seat block name '{}'",
                        deck.name, block.name
                    )));
                }
                let places = block.validate(region)?;
                total_seat_places = total_seat_places.checked_add(places).ok_or_else(|| {
                    FuselageError::InvalidInterior("cabin seat-place count overflow".into())
                })?;
                if total_seat_places > 10_000 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "cabin '{}' exceeds 10000 compiled seat places",
                        region.name
                    )));
                }
                match block.role {
                    CabinSeatRole::Passenger => {
                        passenger_places =
                            passenger_places.checked_add(places).ok_or_else(|| {
                                FuselageError::InvalidInterior(
                                    "passenger-place count overflow".into(),
                                )
                            })?;
                        row_positions.extend(
                            (0..block.rows)
                                .map(|row| block.x0_m + (row as f64 + 0.5) * block.pitch_m),
                        );
                    }
                    CabinSeatRole::FlightCrew => {
                        total_flight_crew_places = total_flight_crew_places
                            .checked_add(places)
                            .ok_or_else(|| {
                                FuselageError::InvalidInterior("pilot-place count overflow".into())
                            })?
                    }
                    CabinSeatRole::CabinAttendant => {
                        total_attendant_places =
                            total_attendant_places.checked_add(places).ok_or_else(|| {
                                FuselageError::InvalidInterior(
                                    "attendant-place count overflow".into(),
                                )
                            })?
                    }
                }
                footprints.push((
                    block.x0_m,
                    block.x0_m + block.rows as f64 * block.pitch_m,
                    format!("seat block '{}'", block.name),
                    FootprintKind::Occupied,
                    None,
                ));
            }
            total_passenger_places = total_passenger_places
                .checked_add(passenger_places)
                .ok_or_else(|| {
                    FuselageError::InvalidInterior("passenger-place count overflow".into())
                })?;
            for monument in &deck.monuments {
                if monument.name.trim().is_empty()
                    || !names.insert(monument.name.as_str())
                    || !monument.x0_m.is_finite()
                    || !monument.x1_m.is_finite()
                    || monument.x0_m >= monument.x1_m
                    || monument.x0_m < region.x0_m
                    || monument.x1_m > region.x1_m
                    || !monument.mass_kg.is_finite()
                    || monument.mass_kg < 0.0
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "deck '{}' has invalid monument '{}'",
                        deck.name, monument.name
                    )));
                }
                footprints.push((
                    monument.x0_m,
                    monument.x1_m,
                    format!("monument '{}'", monument.name),
                    FootprintKind::Occupied,
                    None,
                ));
            }
            let mut door_names = std::collections::HashSet::new();
            let mut exit_pairs: std::collections::HashMap<&str, [Option<&CabinDoor>; 2]> =
                std::collections::HashMap::new();
            for door in &deck.doors {
                if door.name.trim().is_empty()
                    || !door_names.insert(door.name.as_str())
                    || door.pair_id.trim().is_empty()
                    || !door.x_m.is_finite()
                    || !door.clear_zone_length_m.is_finite()
                    || door.clear_zone_length_m <= 0.0
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "deck '{}' has invalid or duplicate door '{}'",
                        deck.name, door.name
                    )));
                }
                let spec = door.rating.spec();
                if door.clear_zone_length_m + 1e-9 < spec.opening_width_m {
                    return Err(FuselageError::InvalidInterior(format!(
                        "door '{}' clear zone is narrower than its {:?} opening",
                        door.name, door.rating
                    )));
                }
                let clear_start = door.x_m - 0.5 * door.clear_zone_length_m;
                let clear_end = door.x_m + 0.5 * door.clear_zone_length_m;
                if clear_start < region.x0_m || clear_end > region.x1_m {
                    return Err(FuselageError::InvalidInterior(format!(
                        "door '{}' clear zone lies outside cabin '{}'",
                        door.name, region.name
                    )));
                }
                let side_index = match door.side {
                    DoorSide::Left => 0,
                    DoorSide::Right => 1,
                };
                let pair = exit_pairs
                    .entry(door.pair_id.as_str())
                    .or_insert([None, None]);
                if pair[side_index].replace(door).is_some() {
                    return Err(FuselageError::InvalidInterior(format!(
                        "exit pair '{}' has more than one {:?} door",
                        door.pair_id, door.side
                    )));
                }
                footprints.push((
                    clear_start,
                    clear_end,
                    format!("door '{}'", door.name),
                    FootprintKind::Door,
                    Some(door.side),
                ));
            }
            for (pair_id, pair) in &exit_pairs {
                if pair[0].is_none() || pair[1].is_none() {
                    return Err(FuselageError::InvalidInterior(format!(
                        "exit pair '{pair_id}' on deck '{}' needs one left and one right door",
                        deck.name
                    )));
                }
            }
            validate_footprint_overlaps(&deck.name, &footprints)?;
            if passenger_places > 0 {
                validate_exit_capacity(
                    &deck.name,
                    passenger_places,
                    &deck.doors,
                    &exit_pairs,
                    &row_positions,
                )?;
            }
        }
        if total_passenger_places > 0 {
            if total_flight_crew_places < 2 {
                return Err(FuselageError::InvalidInterior(format!(
                    "cabin '{}' carries passengers but provides fewer than two flight-crew places",
                    region.name
                )));
            }
            let required_attendants = total_passenger_places.div_ceil(50);
            if total_attendant_places < required_attendants {
                return Err(FuselageError::InvalidInterior(format!(
                    "cabin '{}' needs at least {required_attendants} cabin-attendant places for {total_passenger_places} passenger places",
                    region.name
                )));
            }
        }
        if total_passenger_places > 10_000 {
            return Err(FuselageError::InvalidInterior(format!(
                "cabin '{}' exceeds 10000 passenger places",
                region.name
            )));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FootprintKind {
    Occupied,
    Door,
}

fn validate_footprint_overlaps(
    deck: &str,
    footprints: &[(f64, f64, String, FootprintKind, Option<DoorSide>)],
) -> Result<(), FuselageError> {
    for (index, first) in footprints.iter().enumerate() {
        for second in footprints.iter().skip(index + 1) {
            if first.1 <= second.0 + 1e-9 || second.1 <= first.0 + 1e-9 {
                continue;
            }
            if first.3 == FootprintKind::Door
                && second.3 == FootprintKind::Door
                && first.4 != second.4
            {
                continue;
            }
            return Err(FuselageError::InvalidInterior(format!(
                "{} overlaps {} on deck '{deck}'",
                first.2, second.2
            )));
        }
    }
    Ok(())
}

fn validate_exit_capacity(
    deck_name: &str,
    passenger_places: u32,
    doors: &[CabinDoor],
    pairs: &std::collections::HashMap<&str, [Option<&CabinDoor>; 2]>,
    passenger_rows: &[f64],
) -> Result<(), FuselageError> {
    let mut type_iii_credit = 0_u32;
    let mut other_credit = 0_u32;
    let mut type_iii_stations = Vec::new();
    for pair in pairs.values() {
        let left = pair[0].expect("paired exits were checked");
        let right = pair[1].expect("paired exits were checked");
        let limiting_type = if left.rating.size_rank() <= right.rating.size_rank() {
            left.rating
        } else {
            right.rating
        };
        let credit = limiting_type.spec().passenger_seats;
        if limiting_type == ExitType::TypeIII {
            type_iii_credit += credit;
            type_iii_stations.push(0.5 * (left.x_m + right.x_m));
        } else {
            other_credit += credit;
        }
    }
    let mut type_iii_limit = 70;
    'outer: for (index, first) in type_iii_stations.iter().enumerate() {
        for second in type_iii_stations.iter().skip(index + 1) {
            let rows_between = passenger_rows
                .iter()
                .filter(|row| row.min(*first) < **row && **row < row.max(*second))
                .count();
            if rows_between < 3 {
                type_iii_limit = 65;
                break 'outer;
            }
        }
    }
    let credited_capacity = other_credit + type_iii_credit.min(type_iii_limit);
    if credited_capacity < passenger_places {
        return Err(FuselageError::InvalidInterior(format!(
            "deck '{deck_name}' has {passenger_places} passenger places but paired exits credit only {credited_capacity}"
        )));
    }

    for side in [DoorSide::Left, DoorSide::Right] {
        let mut side_doors: Vec<_> = doors.iter().filter(|door| door.side == side).collect();
        side_doors.sort_by(|left, right| left.x_m.total_cmp(&right.x_m));
        let all_ratings = side_doors.iter().all(|door| door.rating.size_rank() >= 1);
        let type_i_or_larger = side_doors
            .iter()
            .filter(|door| door.rating.size_rank() >= 3)
            .count();
        let type_ii_or_larger = side_doors.iter().any(|door| door.rating.size_rank() >= 2);
        let type_iii_or_larger = side_doors.iter().any(|door| door.rating.size_rank() >= 1);
        let has_type_iv = side_doors
            .iter()
            .any(|door| door.rating == ExitType::TypeIV);
        let complies = match passenger_places {
            1..=9 => has_type_iv || type_iii_or_larger,
            10..=19 => all_ratings && type_iii_or_larger,
            20..=40 => all_ratings && side_doors.len() >= 2 && type_ii_or_larger,
            41..=110 => all_ratings && side_doors.len() >= 2 && type_i_or_larger >= 1,
            _ => all_ratings && type_i_or_larger >= 2,
        };
        if !complies {
            return Err(FuselageError::InvalidInterior(format!(
                "deck '{deck_name}' {:?} exits do not meet 14 CFR 25.807(g) for {passenger_places} passenger places",
                side
            )));
        }
        for adjacent in side_doors.windows(2) {
            let first = adjacent[0];
            let second = adjacent[1];
            let first_edge = first.x_m + 0.5 * first.rating.spec().opening_width_m;
            let second_edge = second.x_m - 0.5 * second.rating.spec().opening_width_m;
            if second_edge - first_edge > 60.0 * 0.3048 + 1e-9 {
                return Err(FuselageError::InvalidInterior(format!(
                    "deck '{deck_name}' {:?} emergency exits are more than 60 ft apart",
                    side
                )));
            }
        }
        if side_doors.iter().any(|door| {
            matches!(
                door.rating,
                ExitType::TypeA | ExitType::TypeB | ExitType::TypeC
            )
        }) && side_doors
            .iter()
            .filter(|door| door.rating.size_rank() >= ExitType::TypeC.size_rank())
            .count()
            < 2
        {
            return Err(FuselageError::InvalidInterior(format!(
                "deck '{deck_name}' needs two Type C-or-larger exits on each side when using Type A/B/C exits"
            )));
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn preset_block(
    name: &str,
    class: SeatClass,
    role: CabinSeatRole,
    x0_m: f64,
    rows: u32,
    columns: Vec<u32>,
    pitch_m: f64,
    aisle_widths_m: Vec<f64>,
    wall_clearance_m: f64,
    suited: bool,
    occupants: u32,
) -> SeatBlock {
    SeatBlock {
        name: name.into(),
        class,
        role,
        x0_m,
        rows,
        columns,
        pitch_m,
        aisle_widths_m,
        wall_clearance_m,
        seat_width_m: None,
        seat_mass_kg_each: None,
        occupants,
        occupant_mass_kg_each: 90.0,
        carry_on_kg_each: if role == CabinSeatRole::Passenger {
            8.0
        } else {
            0.0
        },
        suited,
        suit_mass_kg_each: if suited { 20.0 } else { 0.0 },
        suit_type: SuitType::HoseFed,
        suit_overrides: Vec::new(),
        seat_style: if class == SeatClass::Ejection {
            SeatStyle::Ejection
        } else {
            SeatStyle::Upright
        },
    }
}

fn append_exit_pair(
    doors: &mut Vec<CabinDoor>,
    deck: &str,
    pair_index: usize,
    x_m: f64,
    rating: ExitType,
    clear_zone_length_m: f64,
) {
    let pair_id = format!("{deck}-exit-{pair_index}");
    doors.push(CabinDoor {
        name: format!("{pair_id}-left"),
        pair_id: pair_id.clone(),
        x_m,
        side: DoorSide::Left,
        rating,
        clear_zone_length_m,
    });
    doors.push(CabinDoor {
        name: format!("{pair_id}-right"),
        pair_id,
        x_m,
        side: DoorSide::Right,
        rating,
        clear_zone_length_m,
    });
}

/// One longitudinal interior allocation over `[x0_m, x1_m]` in body metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InteriorRegion {
    pub name: String,
    pub x0_m: f64,
    pub x1_m: f64,
    pub kind: RegionKind,
    /// Pressurized atmosphere held by this region (habitats, cabins, crew;
    /// never tank regions, which size their own shells). `None` vents to
    /// ambient.
    #[serde(default)]
    pub atmosphere: Option<CabinAtmosphere>,
    /// Autopilot core hosted by this region (usually avionics): presence
    /// grants control authority at its tier. Tanks refuse cores.
    #[serde(default)]
    pub control_core: Option<thessa_sim_core::AutopilotTier>,
    /// Advanced cabin plan; absent preserves the existing simple `Crew`
    /// region and capsule compilation path.
    #[serde(default)]
    pub cabin_layout: Option<CabinLayout>,
    /// Built-in cabin recipe; mutually exclusive with `cabin_layout`.
    /// Coordinates are generated from this region's axial range.
    #[serde(default)]
    pub cabin_layout_preset: Option<CabinLayoutPreset>,
}

impl InteriorRegion {
    pub fn new(
        name: impl Into<String>,
        x0_m: f64,
        x1_m: f64,
        kind: RegionKind,
    ) -> Result<Self, FuselageError> {
        let region = Self {
            name: name.into(),
            x0_m,
            x1_m,
            kind,
            atmosphere: None,
            control_core: None,
            cabin_layout: None,
            cabin_layout_preset: None,
        };
        region.validate()?;
        Ok(region)
    }

    /// Pressurized variant: the same allocation holding an atmosphere.
    pub fn pressurized(
        name: impl Into<String>,
        x0_m: f64,
        x1_m: f64,
        kind: RegionKind,
        atmosphere: CabinAtmosphere,
    ) -> Result<Self, FuselageError> {
        let region = Self {
            name: name.into(),
            x0_m,
            x1_m,
            kind,
            atmosphere: Some(atmosphere),
            control_core: None,
            cabin_layout: None,
            cabin_layout_preset: None,
        };
        region.validate()?;
        Ok(region)
    }

    /// Add the advanced deck/block/monument/door plan to a `Cabin` region.
    pub fn with_cabin_layout(mut self, cabin_layout: CabinLayout) -> Result<Self, FuselageError> {
        self.cabin_layout = Some(cabin_layout);
        self.validate()?;
        Ok(self)
    }

    /// Select a built-in layout recipe over this region's axial range.
    pub fn with_cabin_layout_preset(
        mut self,
        cabin_layout_preset: CabinLayoutPreset,
    ) -> Result<Self, FuselageError> {
        self.cabin_layout_preset = Some(cabin_layout_preset);
        self.validate()?;
        Ok(self)
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidInterior(
                "interior region needs a name".into(),
            ));
        }
        if !self.x0_m.is_finite() || !self.x1_m.is_finite() || self.x0_m >= self.x1_m {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{}' needs x0 < x1 (got {}..{})",
                self.name, self.x0_m, self.x1_m
            )));
        }
        match self.kind {
            RegionKind::Tank {
                fill_fraction,
                pressure_pa,
                material,
                ..
            } => {
                if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs fill_fraction in [0, 1]",
                        self.name
                    )));
                }
                if let Some(pressure) = pressure_pa
                    && (!pressure.is_finite() || pressure <= 0.0)
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs pressure_pa > 0",
                        self.name
                    )));
                }
                if let Some(material) = material
                    && material.validate().is_err()
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' has an invalid tank material",
                        self.name
                    )));
                }
            }
            RegionKind::FluidTank {
                fill_fraction,
                pressure_pa,
                material,
                ..
            } => {
                if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs fill_fraction in [0, 1]",
                        self.name
                    )));
                }
                if let Some(pressure) = pressure_pa
                    && (!pressure.is_finite() || pressure <= 0.0)
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs pressure_pa > 0",
                        self.name
                    )));
                }
                if let Some(material) = material
                    && material.validate().is_err()
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' has an invalid tank material",
                        self.name
                    )));
                }
            }
            RegionKind::Bipropellant {
                fill_fraction,
                mixture_ratio,
                pressure_pa,
                oxidizer_pressure_pa,
                fuel_pressure_pa,
                material,
                oxidizer_material,
                fuel_material,
                ..
            } => {
                if !fill_fraction.is_finite() || !(0.0..=1.0).contains(&fill_fraction) {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs fill_fraction in [0, 1]",
                        self.name
                    )));
                }
                // Only true bipropellant pairs can split into two tanks.
                let propellant = match self.kind {
                    RegionKind::Bipropellant { propellant, .. } => propellant,
                    _ => unreachable!("matched bipropellant"),
                };
                if propellant.split_densities().is_none() {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs a bipropellant pair for a split tank",
                        self.name
                    )));
                }
                if let Some(ratio) = mixture_ratio {
                    if !ratio.is_finite() || ratio <= 0.0 {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' needs mixture_ratio > 0",
                            self.name
                        )));
                    }
                    if propellant.thermo_at_mixture(Some(ratio)).is_err() {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' mixture_ratio is outside the modeled table",
                            self.name
                        )));
                    }
                }
                for (label, pressure) in [
                    ("pressure_pa", pressure_pa),
                    ("oxidizer_pressure_pa", oxidizer_pressure_pa),
                    ("fuel_pressure_pa", fuel_pressure_pa),
                ] {
                    if let Some(pressure) = pressure
                        && (!pressure.is_finite() || pressure <= 0.0)
                    {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' needs {label} > 0",
                            self.name
                        )));
                    }
                }
                for material in [material, oxidizer_material, fuel_material]
                    .into_iter()
                    .flatten()
                {
                    if material.validate().is_err() {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' has an invalid tank material",
                            self.name
                        )));
                    }
                }
            }
            RegionKind::Crew {
                seats,
                seat_mass_kg_each,
                occupant_mass_kg_each,
                seat_pitch_m,
                abreast,
                suited,
                suit_mass_kg_each,
                ..
            } => {
                if seats == 0 || seats > 1000 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs seats in [1, 1000]",
                        self.name
                    )));
                }
                if let Some(abreast) = abreast
                    && (abreast == 0 || abreast > seats)
                {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs abreast in [1, seats]",
                        self.name
                    )));
                }
                if seats == 0 || seats > 1000 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs seats in [1, 1000]",
                        self.name
                    )));
                }
                if !seat_mass_kg_each.is_finite() || seat_mass_kg_each < 0.0 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs seat_mass_kg_each >= 0",
                        self.name
                    )));
                }
                if !occupant_mass_kg_each.is_finite() || occupant_mass_kg_each < 0.0 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs occupant_mass_kg_each >= 0",
                        self.name
                    )));
                }
                let suit_mass = suit_mass_kg_each;
                if !suit_mass.is_finite() || suit_mass < 0.0 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' needs suit_mass_kg_each >= 0",
                        self.name
                    )));
                }
                if suited && suit_mass <= 0.0 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' suits need suit_mass_kg_each > 0",
                        self.name
                    )));
                }
                if !suited && suit_mass > 0.0 {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' carries suit mass without suited crew",
                        self.name
                    )));
                }
                if let Some(pitch) = seat_pitch_m {
                    if !pitch.is_finite() || pitch <= 0.0 {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' needs seat_pitch_m > 0",
                            self.name
                        )));
                    }
                    let rows = seats.div_ceil(abreast.unwrap_or(1).max(1));
                    let length = self.x1_m - self.x0_m;
                    if (rows as f64 - 1.0) * pitch > length + 1e-9 {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' seats at {pitch} m pitch do not fit in {length:.2} m",
                            self.name
                        )));
                    }
                }
            }
            RegionKind::Cargo { payload_mass_kg }
                if !payload_mass_kg.is_finite() || payload_mass_kg < 0.0 =>
            {
                return Err(FuselageError::InvalidInterior(format!(
                    "region '{}' needs payload_mass_kg >= 0",
                    self.name
                )));
            }
            RegionKind::Cargo { .. } => {}
            _ => {}
        }
        if let Some(atmosphere) = self.atmosphere {
            atmosphere.validate(&self.name)?;
            match self.kind {
                RegionKind::Tank { .. }
                | RegionKind::FluidTank { .. }
                | RegionKind::Bipropellant { .. } => {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' is a tank and sizes its own shell; atmosphere belongs on habitats",
                        self.name
                    )));
                }
                _ => {}
            }
        }
        // Unsuited crew need cabin air; suited crews may fly dry.
        if let RegionKind::Crew { suited, .. } = self.kind
            && !suited
            && self.atmosphere.is_none()
        {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{}' has unsuited crew without atmosphere (add air or suits)",
                self.name
            )));
        }
        // Tanks size their own shells; they host neither air nor cores.
        if self.control_core.is_some() {
            match self.kind {
                RegionKind::Tank { .. }
                | RegionKind::FluidTank { .. }
                | RegionKind::Bipropellant { .. } => {
                    return Err(FuselageError::InvalidInterior(format!(
                        "region '{}' is a tank and cannot host a control core",
                        self.name
                    )));
                }
                _ => {}
            }
        }
        if self.cabin_layout.is_some() && self.cabin_layout_preset.is_some() {
            return Err(FuselageError::InvalidInterior(format!(
                "region '{}' cannot specify both cabin_layout and cabin_layout_preset",
                self.name
            )));
        }
        if self.cabin_layout.is_some() || self.cabin_layout_preset.is_some() {
            if !matches!(self.kind, RegionKind::Cabin) {
                return Err(FuselageError::InvalidInterior(format!(
                    "advanced cabin layout on '{}' requires kind = cabin; legacy Crew regions remain unchanged",
                    self.name
                )));
            }
            if let Some(layout) = &self.cabin_layout {
                layout.validate(self)?;
            } else if let Some(preset) = self.cabin_layout_preset {
                preset
                    .build_for_region(self.x0_m, self.x1_m)?
                    .validate(self)?;
            }
        }
        Ok(())
    }
}

/// External interface anchor kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PortKind {
    /// Docking/berthing interface (faces fore or aft).
    Docking,
    /// Engine mount station (faces aft).
    EngineMount,
    /// Generic hardpoint: intake, strut, payload pylon (faces radial).
    Attachment,
}

/// One interface anchor on the outer mold.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyPort {
    pub name: String,
    /// Longitudinal station in body metres.
    pub x_m: f64,
    /// Clock angle around +X in radians (0 = +Y, toward +Z).
    pub clock_rad: f64,
    pub kind: PortKind,
    /// Interface diameter in metres (hatch, throat, or bolt circle).
    pub diameter_m: f64,
}

/// Which loft end a detachable part mounts on: tail (`x_first`) or nose
/// (`x_last`). Stations run tail-to-nose with `+X` forward.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BodyEnd {
    Aft,
    Forward,
}

/// A detachable heat shield on one blunt end (KSP-style separate part:
/// authored and tracked independently of the loft primitive, docking
/// ports, and tank shells). Diameter derives from the end section;
/// thickness and ablative mass are authoring inputs. Entry aeroheating
/// itself is future work — this record owns geometry and mass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyHeatShield {
    pub name: String,
    pub end: BodyEnd,
    /// Shield thickness in mm.
    pub thickness_mm: f64,
    /// Ablative shell material (density sizes the mass).
    pub material: thessa_sim_core::ChamberMaterial,
}

impl BodyHeatShield {
    pub fn new(
        name: impl Into<String>,
        end: BodyEnd,
        thickness_mm: f64,
        material: thessa_sim_core::ChamberMaterial,
    ) -> Result<Self, FuselageError> {
        let shield = Self {
            name: name.into(),
            end,
            thickness_mm,
            material,
        };
        shield.validate()?;
        Ok(shield)
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "heat shield needs a name".into(),
            ));
        }
        if !self.thickness_mm.is_finite() || self.thickness_mm <= 0.0 {
            return Err(FuselageError::InvalidBody(format!(
                "heat shield '{}' needs thickness_mm > 0",
                self.name
            )));
        }
        if self.material.validate().is_err() {
            return Err(FuselageError::InvalidBody(format!(
                "heat shield '{}' has an invalid material",
                self.name
            )));
        }
        Ok(())
    }
}

/// Attachment node kind: plain structural stack (resources flow, crew
/// never passes) vs hatch (structural + resources + crew/air when open).
/// A hatch node mates any node kind; a stack node has no door and is
/// always open on its side.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AttachKind {
    Stack,
    Hatch,
}

/// Where on the loft a node sits: blunt ends or an explicit station.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AttachSite {
    AftEnd,
    ForwardEnd,
    /// Clock angle follows fuselage convention: 0 is +Y, positive angles
    /// turn toward +Z. The node sits on the loft outline at this station.
    Station {
        x_m: f64,
        #[serde(default)]
        clock_rad: f64,
    },
}

/// One KSP-style attach node authored separately from the loft
/// primitive, like ports and heat shields. Diameter derives from the
/// local section unless authored explicitly (docking standards).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AttachNode {
    pub name: String,
    pub site: AttachSite,
    pub kind: AttachKind,
    /// Interface diameter in metres (`None` = local section diameter).
    #[serde(default)]
    pub diameter_m: Option<f64>,
}

impl AttachNode {
    pub fn new(
        name: impl Into<String>,
        site: AttachSite,
        kind: AttachKind,
        diameter_m: Option<f64>,
    ) -> Result<Self, FuselageError> {
        let node = Self {
            name: name.into(),
            site,
            kind,
            diameter_m,
        };
        node.validate()?;
        Ok(node)
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "attach node needs a name".into(),
            ));
        }
        if let AttachSite::Station { x_m, clock_rad } = self.site
            && (!x_m.is_finite() || !clock_rad.is_finite())
        {
            return Err(FuselageError::InvalidBody(format!(
                "attach node '{}' station and clock angle must be finite",
                self.name
            )));
        }
        if let Some(diameter) = self.diameter_m
            && (!diameter.is_finite() || diameter <= 0.0)
        {
            return Err(FuselageError::InvalidBody(format!(
                "attach node '{}' needs diameter_m > 0",
                self.name
            )));
        }
        Ok(())
    }
}

/// Which pair of the fuselage compiler's orthogonal normal-force strips is
/// driven by a body-mounted aerodynamic control.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum BodyControlPlane {
    /// Pitch-normal strip, with lift principally along body `+Z`.
    Pitch,
    /// Yaw-normal strip, with lift principally along body `+Y`.
    Yaw,
}

/// An axial fuselage strip region assigned to one normalized control input.
///
/// The selected panels are generated by the body loft compiler; the author
/// does not address global panel indices. Axial bounds are inserted into the
/// compiler's zone schedule so no controlled panel straddles a control edge.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyControlRegion {
    pub name: String,
    pub x0_m: f64,
    pub x1_m: f64,
    pub plane: BodyControlPlane,
    pub minimum_deflection_rad: f64,
    pub maximum_deflection_rad: f64,
    /// Optional physical drive rating. Omission preserves legacy immediate
    /// command response for migration of existing body-control assets.
    #[serde(default)]
    pub actuator: Option<thessa_sim_core::ControlSurfaceActuator>,
}

impl BodyControlRegion {
    pub fn new(
        name: impl Into<String>,
        x0_m: f64,
        x1_m: f64,
        plane: BodyControlPlane,
        minimum_deflection_rad: f64,
        maximum_deflection_rad: f64,
    ) -> Result<Self, FuselageError> {
        let region = Self {
            name: name.into(),
            x0_m,
            x1_m,
            plane,
            minimum_deflection_rad,
            maximum_deflection_rad,
            actuator: None,
        };
        if region.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "body control region needs a name".into(),
            ));
        }
        if !region.x0_m.is_finite() || !region.x1_m.is_finite() || region.x1_m - region.x0_m <= 1e-9
        {
            return Err(FuselageError::InvalidBody(
                "body control needs a finite non-empty axial range".into(),
            ));
        }
        if !region.minimum_deflection_rad.is_finite()
            || !region.maximum_deflection_rad.is_finite()
            || region.minimum_deflection_rad > 0.0
            || region.maximum_deflection_rad <= 0.0
            || region.minimum_deflection_rad <= -std::f64::consts::PI
            || region.maximum_deflection_rad >= std::f64::consts::PI
        {
            return Err(FuselageError::InvalidBody(format!(
                "body control '{}' has invalid deflection limits",
                region.name
            )));
        }
        Ok(region)
    }

    /// Set the physical no-load slew rate and stall torque for this body
    /// control. Values are design inputs in rad/s and N·m.
    pub fn with_actuator(
        mut self,
        actuator: thessa_sim_core::ControlSurfaceActuator,
    ) -> Result<Self, FuselageError> {
        actuator
            .validate()
            .map_err(|error| FuselageError::InvalidBody(error.to_string()))?;
        self.actuator = Some(actuator);
        Ok(self)
    }

    fn validate(&self, x_first: f64, x_last: f64) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "body control region needs a name".into(),
            ));
        }
        if !self.x0_m.is_finite()
            || !self.x1_m.is_finite()
            || self.x1_m - self.x0_m <= 1e-9
            || self.x0_m < x_first
            || self.x1_m > x_last
        {
            return Err(FuselageError::InvalidBody(format!(
                "body control '{}' needs a non-empty axial range within {x_first}..{x_last}",
                self.name
            )));
        }
        if !self.minimum_deflection_rad.is_finite()
            || !self.maximum_deflection_rad.is_finite()
            || self.minimum_deflection_rad > 0.0
            || self.maximum_deflection_rad <= 0.0
            || self.minimum_deflection_rad <= -std::f64::consts::PI
            || self.maximum_deflection_rad >= std::f64::consts::PI
        {
            return Err(FuselageError::InvalidBody(format!(
                "body control '{}' has invalid deflection limits",
                self.name
            )));
        }
        if let Some(actuator) = self.actuator {
            actuator
                .validate()
                .map_err(|error| FuselageError::InvalidBody(error.to_string()))?;
        }
        Ok(())
    }
}

impl BodyPort {
    pub fn new(
        name: impl Into<String>,
        x_m: f64,
        clock_rad: f64,
        kind: PortKind,
        diameter_m: f64,
    ) -> Result<Self, FuselageError> {
        let port = Self {
            name: name.into(),
            x_m,
            clock_rad,
            kind,
            diameter_m,
        };
        port.validate()?;
        Ok(port)
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidInterior(
                "body port needs a name".into(),
            ));
        }
        if !self.x_m.is_finite() || !self.clock_rad.is_finite() || !self.diameter_m.is_finite() {
            return Err(FuselageError::InvalidInterior(
                "body port values must be finite".into(),
            ));
        }
        if self.diameter_m <= 0.0 {
            return Err(FuselageError::InvalidInterior(format!(
                "port '{}' needs diameter_m > 0",
                self.name
            )));
        }
        Ok(())
    }
}

/// Hull shell material (structural skin and frames, not tank pressure
/// shells: those size through [`thessa_sim_core::ChamberMaterial`] in the
/// tank pipeline, the same wall math as every other pressure vessel).
/// The optional yield strength gates pressurized habitats: holding cabin
/// pressure needs a known shell allowable, otherwise the compiler refuses
/// instead of guessing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HullMaterial {
    /// Material name for hangar display and golden records.
    pub name: String,
    /// Mass density in kg/m^3.
    pub density_kg_m3: f64,
    /// Yield strength in MPa for pressure-membrane screening (`None` =
    /// unknown: unpressurized shells only). Composite allowables are
    /// layup-specific and intentionally left unset on the preset.
    #[serde(default)]
    pub yield_strength_mpa: Option<f64>,
}

impl HullMaterial {
    fn checked(name: &str, density_kg_m3: f64, yield_strength_mpa: Option<f64>) -> Self {
        Self {
            name: name.into(),
            density_kg_m3,
            yield_strength_mpa,
        }
    }

    /// Aluminum 7075-T6: 2810 kg/m^3, 503 MPa yield (same source as wing skins).
    pub fn aluminum_7075() -> Self {
        Self::checked("Al-7075-T6", 2810.0, Some(503.0))
    }

    /// Aluminum 2219-T87 tankage-grade: 2840 kg/m^3, 395 MPa yield.
    pub fn aluminum_2219() -> Self {
        Self::checked("Al-2219-T87", 2840.0, Some(395.0))
    }

    /// Quasi-isotropic carbon laminate: 1600 kg/m^3. Pressure allowable
    /// is layup-specific, so it stays unset: pressurized carbon shells
    /// need an explicit layup allowable.
    pub fn carbon_fiber() -> Self {
        Self::checked("CFRP-quasi-iso", 1600.0, None)
    }

    /// Titanium Ti-6Al-4V: 4430 kg/m^3, 880 MPa yield.
    pub fn titanium() -> Self {
        Self::checked("Ti-6Al-4V", 4430.0, Some(880.0))
    }

    /// Stainless 304L: 7900 kg/m^3, 205 MPa yield (annealed).
    pub fn stainless_304() -> Self {
        Self::checked("SS-304L", 7900.0, Some(205.0))
    }

    /// Aluminum-lithium 2195: 2710 kg/m^3, 590 MPa yield (T8).
    pub fn aluminum_lithium_2195() -> Self {
        Self::checked("Al-Li-2195", 2710.0, Some(590.0))
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "hull material needs a name".into(),
            ));
        }
        if !self.density_kg_m3.is_finite() || self.density_kg_m3 <= 0.0 {
            return Err(FuselageError::InvalidBody(
                "hull material density must be finite and > 0".into(),
            ));
        }
        if let Some(yield_mpa) = self.yield_strength_mpa
            && (!yield_mpa.is_finite() || yield_mpa <= 0.0)
        {
            return Err(FuselageError::InvalidBody(
                "hull material yield strength must be finite and > 0".into(),
            ));
        }
        Ok(())
    }
}

/// Structural sizing inputs for one hull.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyStructuralLayout {
    /// Skin material (outer mold shell).
    pub skin_material: HullMaterial,
    /// Skin gauge in mm (manufacturing parameter).
    pub skin_gauge_mm: f64,
    /// Ring-frame pitch in metres.
    pub frame_spacing_m: f64,
    /// Ring-frame gauge in mm.
    pub frame_gauge_mm: f64,
    /// Ring-frame axial width in mm.
    pub frame_width_mm: f64,
    /// Service pressure for tank-region pressure shells (Pa).
    pub tank_pressure_pa: f64,
    /// Tank-region shell material (thin-wall pressure sizing).
    pub tank_material: thessa_sim_core::ChamberMaterial,
    /// Inner-wall inset for usable volume in mm (insulation/liner).
    pub wall_inset_mm: f64,
}

impl BodyStructuralLayout {
    /// Light metal baseline: 2 mm 7075 skin, 1 m frame pitch with
    /// 2x40 mm frames, 0.5 MPa tankage in nickel-superalloy shells,
    /// 10 mm wall inset.
    pub fn metal_baseline() -> Self {
        Self {
            skin_material: HullMaterial::aluminum_7075(),
            skin_gauge_mm: 2.0,
            frame_spacing_m: 1.0,
            frame_gauge_mm: 2.0,
            frame_width_mm: 40.0,
            tank_pressure_pa: 0.5e6,
            tank_material: thessa_sim_core::ChamberMaterial::nickel_superalloy(),
            wall_inset_mm: 10.0,
        }
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        self.skin_material.validate()?;
        for (label, value) in [
            ("skin_gauge_mm", self.skin_gauge_mm),
            ("frame_spacing_m", self.frame_spacing_m),
            ("frame_gauge_mm", self.frame_gauge_mm),
            ("frame_width_mm", self.frame_width_mm),
            ("tank_pressure_pa", self.tank_pressure_pa),
            ("wall_inset_mm", self.wall_inset_mm),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(FuselageError::InvalidBody(format!(
                    "layout {label} must be finite and > 0 (got {value})"
                )));
            }
        }
        // Tank shell sizing validates the material through TankSpec::compile.
        Ok(())
    }
}

/// Procedural fuselage authoring: station loft plus purpose layers.
///
/// Stations run tail-to-nose (`x` ascending, nose last, `+X` forward).
/// Open ends (first/last equivalent radius above zero) close with flat
/// discs; pointed noses and shaped tails are authored as tip stations by
/// the revolve-style constructors below, keeping one representation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProceduralBody {
    pub name: String,
    pub stations: Vec<BodyStation>,
    /// Body origin in vehicle metres (axis-aligned mount, no rotation:
    /// orientation mounts arrive with vehicle assembly).
    pub origin_body_m: DVec3,
    /// Interior allocations over the station range.
    #[serde(default)]
    pub regions: Vec<InteriorRegion>,
    /// Interface anchors on the outer mold.
    #[serde(default)]
    pub ports: Vec<BodyPort>,
    /// Detachable heat shields on the blunt ends (separate parts).
    #[serde(default)]
    pub heat_shields: Vec<BodyHeatShield>,
    /// KSP-style attach nodes for assembly links (separate details).
    #[serde(default)]
    pub attach_nodes: Vec<AttachNode>,
    /// Axial regions assigned to control inputs over generated body strips.
    #[serde(default)]
    pub controls: Vec<BodyControlRegion>,
    /// Structural sizing; `None` skips shell/fuel mass (pure aero shell).
    #[serde(default)]
    pub structure: Option<BodyStructuralLayout>,
}

impl ProceduralBody {
    pub fn new(
        name: impl Into<String>,
        stations: Vec<BodyStation>,
        origin_body_m: DVec3,
    ) -> Result<Self, FuselageError> {
        let body = Self {
            name: name.into(),
            stations,
            origin_body_m,
            regions: Vec::new(),
            ports: Vec::new(),
            heat_shields: Vec::new(),
            attach_nodes: Vec::new(),
            controls: Vec::new(),
            structure: None,
        };
        body.validate()?;
        Ok(body)
    }

    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody(
                "fuselage body needs a name".into(),
            ));
        }
        if !self.origin_body_m.is_finite() {
            return Err(FuselageError::InvalidBody(
                "body origin must be finite".into(),
            ));
        }
        if self.stations.len() < 2 {
            return Err(FuselageError::InvalidBody(
                "fuselage body needs at least 2 stations".into(),
            ));
        }
        for station in &self.stations {
            station.validate()?;
        }
        for pair in self.stations.windows(2) {
            if pair[1].x_m - pair[0].x_m <= 1e-9 {
                return Err(FuselageError::InvalidBody(
                    "stations must run tail-to-nose with strictly increasing x".into(),
                ));
            }
        }
        if let Some(layout) = &self.structure {
            layout.validate()?;
        }
        let (x_first, x_last) = (self.stations[0].x_m, self.stations.last().unwrap().x_m);
        for region in &self.regions {
            region.validate()?;
            if region.x0_m < x_first - 1e-9 || region.x1_m > x_last + 1e-9 {
                return Err(FuselageError::InvalidInterior(format!(
                    "region '{}' lies outside the station range {x_first}..{x_last}",
                    region.name
                )));
            }
        }
        let mut spans: Vec<(f64, f64, &str)> = self
            .regions
            .iter()
            .map(|region| (region.x0_m, region.x1_m, region.name.as_str()))
            .collect();
        spans.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("validated finite"));
        for pair in spans.windows(2) {
            if pair[1].0 < pair[0].1 - 1e-9 {
                return Err(FuselageError::InvalidInterior(format!(
                    "regions '{}' and '{}' overlap",
                    pair[0].2, pair[1].2
                )));
            }
        }
        for port in &self.ports {
            port.validate()?;
            if port.x_m < x_first - 1e-9 || port.x_m > x_last + 1e-9 {
                return Err(FuselageError::InvalidInterior(format!(
                    "port '{}' lies outside the station range",
                    port.name
                )));
            }
        }
        for shield in &self.heat_shields {
            shield.validate()?;
            // A shield needs a blunt end to cover: pointed tips refuse.
            let end_station = match shield.end {
                BodyEnd::Aft => self.stations[0],
                BodyEnd::Forward => *self.stations.last().unwrap(),
            };
            let end_diameter = 2.0
                * end_station
                    .half_width_m
                    .max(end_station.top_height_m)
                    .max(end_station.bottom_height_m);
            if end_diameter < 0.1 {
                return Err(FuselageError::InvalidBody(format!(
                    "heat shield '{}' needs a blunt end (got {:.3} m diameter)",
                    shield.name, end_diameter
                )));
            }
        }
        let mut node_names = std::collections::HashSet::new();
        for node in &self.attach_nodes {
            node.validate()?;
            if !node_names.insert(node.name.as_str()) {
                return Err(FuselageError::InvalidBody(format!(
                    "attach node '{}' is defined twice on '{}'",
                    node.name, self.name
                )));
            }
            if let AttachSite::Station { x_m, .. } = node.site
                && (x_m < x_first - 1e-9 || x_m > x_last + 1e-9)
            {
                return Err(FuselageError::InvalidBody(format!(
                    "attach node '{}' lies outside the station range",
                    node.name
                )));
            }
        }
        let mut control_spans: Vec<_> = self
            .controls
            .iter()
            .map(|control| {
                control.validate(x_first, x_last)?;
                Ok((
                    control.plane,
                    control.x0_m,
                    control.x1_m,
                    control.name.as_str(),
                ))
            })
            .collect::<Result<_, FuselageError>>()?;
        control_spans.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("validated finite"));
        for (index, first) in control_spans.iter().enumerate() {
            for second in control_spans.iter().skip(index + 1) {
                if second.1 >= first.2 {
                    break;
                }
                if first.0 == second.0 {
                    return Err(FuselageError::InvalidBody(format!(
                        "body controls '{}' and '{}' overlap on the {:?} plane",
                        first.3, second.3, first.0
                    )));
                }
            }
        }
        Ok(())
    }

    /// Straight cylinder (Juno-style tank barrel section).
    pub fn cylinder(
        name: impl Into<String>,
        length_m: f64,
        radius_m: f64,
        origin_body_m: DVec3,
    ) -> Result<Self, FuselageError> {
        if !length_m.is_finite() || length_m <= 0.0 || !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(FuselageError::InvalidBody(
                "cylinder needs finite positive length and radius".into(),
            ));
        }
        Self::new(
            name,
            vec![
                BodyStation::round(0.0, radius_m)?,
                BodyStation::round(length_m, radius_m)?,
            ],
            origin_body_m,
        )
    }

    /// Straight cone from base radius to tip (Juno-style nose cone).
    pub fn cone(
        name: impl Into<String>,
        length_m: f64,
        base_radius_m: f64,
        origin_body_m: DVec3,
    ) -> Result<Self, FuselageError> {
        if !length_m.is_finite()
            || length_m <= 0.0
            || !base_radius_m.is_finite()
            || base_radius_m <= 0.0
        {
            return Err(FuselageError::InvalidBody(
                "cone needs finite positive length and base radius".into(),
            ));
        }
        // Near-sharp tip keeps the solid watertight without a zero-area
        // station (which station validation would reject).
        Self::new(
            name,
            vec![
                BodyStation::round(0.0, base_radius_m)?,
                BodyStation::round(length_m, base_radius_m.min(1e-3))?,
            ],
            origin_body_m,
        )
    }

    /// Tangent-ogive nose over `base_radius_m` (Juno-style fairing nose).
    /// Profile: circular arc tangent to the barrel at the base, closing
    /// to a near-sharp tip over `divisions` authored stations.
    pub fn ogive_nose(
        name: impl Into<String>,
        base_radius_m: f64,
        length_m: f64,
        divisions: usize,
        origin_body_m: DVec3,
    ) -> Result<Self, FuselageError> {
        if !base_radius_m.is_finite()
            || base_radius_m <= 0.0
            || !length_m.is_finite()
            || length_m <= 0.0
            || divisions < 2
        {
            return Err(FuselageError::InvalidBody(
                "ogive needs positive radius/length and >= 2 divisions".into(),
            ));
        }
        // Tangent ogive: arc radius rho = (R^2 + L^2) / (2R) centered
        // at the base plane; radius at axial position x from the base is
        // sqrt(rho^2 - x^2) - (rho - R): full R with zero slope at the
        // barrel joint, closing to a near-sharp tip at x = L.
        let rho = (base_radius_m.powi(2) + length_m.powi(2)) / (2.0 * base_radius_m);
        let mut stations = Vec::with_capacity(divisions + 1);
        for index in 0..=divisions {
            let x = length_m * index as f64 / divisions as f64;
            let radius = (rho.powi(2) - x.powi(2)).sqrt() - (rho - base_radius_m);
            stations.push(BodyStation::round(x, radius.max(1e-3))?);
        }
        Self::new(name, stations, origin_body_m)
    }

    /// Section shape at axial `x_m` by linear station interpolation.
    /// Exact on linear inputs: subdivision never moves geometry.
    pub fn section_at(&self, x_m: f64) -> BodyStation {
        let stations = &self.stations;
        if x_m <= stations[0].x_m {
            return stations[0];
        }
        if x_m >= stations.last().unwrap().x_m {
            return *stations.last().unwrap();
        }
        for pair in stations.windows(2) {
            if x_m <= pair[1].x_m {
                let t = (x_m - pair[0].x_m) / (pair[1].x_m - pair[0].x_m);
                return pair[0].lerp(pair[1], t);
            }
        }
        *stations.last().unwrap()
    }
}
