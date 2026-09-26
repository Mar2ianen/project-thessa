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
    Crew {
        seats: u32,
        #[serde(default = "default_seat_mass_kg_each")]
        seat_mass_kg_each: f64,
        #[serde(default)]
        occupant_mass_kg_each: f64,
        /// Longitudinal pitch between places; `None` spreads evenly.
        #[serde(default)]
        seat_pitch_m: Option<f64>,
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

/// One longitudinal interior allocation over `[x0_m, x1_m]` in body metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct InteriorRegion {
    pub name: String,
    pub x0_m: f64,
    pub x1_m: f64,
    pub kind: RegionKind,
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
        };
        region.validate()?;
        Ok(region)
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
            } => {
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
                if let Some(pitch) = seat_pitch_m {
                    if !pitch.is_finite() || pitch <= 0.0 {
                        return Err(FuselageError::InvalidInterior(format!(
                            "region '{}' needs seat_pitch_m > 0",
                            self.name
                        )));
                    }
                    let length = self.x1_m - self.x0_m;
                    if (seats as f64 - 1.0) * pitch > length + 1e-9 {
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HullMaterial {
    /// Material name for hangar display and golden records.
    pub name: String,
    /// Mass density in kg/m^3.
    pub density_kg_m3: f64,
}

impl HullMaterial {
    fn checked(name: &str, density_kg_m3: f64) -> Self {
        Self {
            name: name.into(),
            density_kg_m3,
        }
    }

    /// Aluminum 7075-T6: 2810 kg/m^3 (same source as wing skins).
    pub fn aluminum_7075() -> Self {
        Self::checked("Al-7075-T6", 2810.0)
    }

    /// Aluminum 2219-T87 tankage-grade: 2840 kg/m^3.
    pub fn aluminum_2219() -> Self {
        Self::checked("Al-2219-T87", 2840.0)
    }

    /// Quasi-isotropic carbon laminate: 1600 kg/m^3.
    pub fn carbon_fiber() -> Self {
        Self::checked("CFRP-quasi-iso", 1600.0)
    }

    /// Titanium Ti-6Al-4V: 4430 kg/m^3.
    pub fn titanium() -> Self {
        Self::checked("Ti-6Al-4V", 4430.0)
    }

    /// Stainless 304L: 7900 kg/m^3 (weldable storable/cryo shells).
    pub fn stainless_304() -> Self {
        Self::checked("SS-304L", 7900.0)
    }

    /// Aluminum-lithium 2195: 2710 kg/m^3 (cryo tankage-grade).
    pub fn aluminum_lithium_2195() -> Self {
        Self::checked("Al-Li-2195", 2710.0)
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
