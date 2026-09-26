//! Parametric capsule cabins: a general blunt-body primitive, not a
//! vehicle catalogue. Dial base/top diameter, height (or sphere diameter),
//! crew count, couch rows, and atmosphere to cover Mercury- through
//! Dragon-class missions; those vehicles are example parameter sets,
//! not library presets.
//!
//! A capsule is a blunt frustum (or sphere) pressure vessel with the crew
//! low near the heat shield, couches side-by-side, and cabin air on
//! request. Airplane-like cabins are future work; the transverse `abreast`
//! rows introduced for couches already anticipate multi-aisle layouts.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    BodyStation, BodyStructuralLayout, CabinAtmosphere, FuselageError, InteriorRegion,
    ProceduralBody, RegionKind, SeatStyle,
};
/// Capsule outer-mold shape: blunt conical frustum or full sphere.
/// Stations run tail-to-nose (`+X` forward); the blunt base closes with a
/// flat disc at compile time (the heat-shield plane).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CapsuleShape {
    Frustum {
        base_diameter_m: f64,
        top_diameter_m: f64,
        height_m: f64,
    },
    Sphere {
        diameter_m: f64,
    },
}

/// Authoring inputs for one capsule cabin.
#[derive(Debug, Clone, PartialEq)]
pub struct CapsuleParams {
    pub name: String,
    pub shape: CapsuleShape,
    pub crew: u32,
    pub seat_style: SeatStyle,
    /// Places per transverse row (Apollo 3 side-by-side, Orion/Dragon 2).
    pub abreast: u32,
    /// Couch/seat mass each in kg (couches run ~25-35).
    pub couch_mass_kg_each: f64,
    pub occupant_mass_kg_each: f64,
    /// Crew fly suited (dry cabins allowed) with this suit mass each.
    pub suited: bool,
    pub suit_mass_kg_each: f64,
    pub atmosphere: Option<CabinAtmosphere>,
    pub structure: Option<BodyStructuralLayout>,
    pub origin_body_m: DVec3,
    /// Loft segments along the shape.
    pub divisions: usize,
}

impl CapsuleParams {
    pub fn validate(&self) -> Result<(), FuselageError> {
        if self.name.trim().is_empty() {
            return Err(FuselageError::InvalidBody("capsule needs a name".into()));
        }
        match self.shape {
            CapsuleShape::Frustum {
                base_diameter_m,
                top_diameter_m,
                height_m,
            } => {
                for (label, value) in [
                    ("base_diameter_m", base_diameter_m),
                    ("top_diameter_m", top_diameter_m),
                    ("height_m", height_m),
                ] {
                    if !value.is_finite() || value <= 0.0 {
                        return Err(FuselageError::InvalidBody(format!(
                            "capsule frustum needs {label} > 0"
                        )));
                    }
                }
                if top_diameter_m > base_diameter_m {
                    return Err(FuselageError::InvalidBody(
                        "capsule frustum tapers toward the nose (top <= base)".into(),
                    ));
                }
            }
            CapsuleShape::Sphere { diameter_m } => {
                if !diameter_m.is_finite() || diameter_m <= 0.0 {
                    return Err(FuselageError::InvalidBody(
                        "capsule sphere needs diameter_m > 0".into(),
                    ));
                }
            }
        }
        if self.crew == 0 || self.crew > 10 {
            return Err(FuselageError::InvalidBody(
                "capsule needs crew in [1, 10]".into(),
            ));
        }
        if self.abreast == 0 || self.abreast > self.crew {
            return Err(FuselageError::InvalidBody(
                "capsule needs abreast in [1, crew]".into(),
            ));
        }
        if !self.couch_mass_kg_each.is_finite() || self.couch_mass_kg_each < 0.0 {
            return Err(FuselageError::InvalidInterior(
                "capsule couch mass must be finite and >= 0".into(),
            ));
        }
        if !self.occupant_mass_kg_each.is_finite() || self.occupant_mass_kg_each < 0.0 {
            return Err(FuselageError::InvalidInterior(
                "capsule occupant mass must be finite and >= 0".into(),
            ));
        }
        if self.divisions < 2 {
            return Err(FuselageError::InvalidBody(
                "capsule needs divisions >= 2".into(),
            ));
        }
        if !self.origin_body_m.is_finite() {
            return Err(FuselageError::InvalidBody(
                "capsule origin must be finite".into(),
            ));
        }
        Ok(())
    }
}

/// Build the lofted pressure vessel with crew low near the heat shield,
/// sea-level air when requested, and a nose docking port where the top
/// fits one. Crew ride the wide lower frustum (`5-65%` of height), like
/// real capsules; spheres use the middle `20-80%`.
pub fn capsule_body(params: &CapsuleParams) -> Result<ProceduralBody, FuselageError> {
    params.validate()?;
    let stations = match params.shape {
        CapsuleShape::Frustum {
            base_diameter_m,
            top_diameter_m,
            height_m,
        } => {
            let mut stations = Vec::with_capacity(params.divisions + 1);
            for index in 0..=params.divisions {
                let t = index as f64 / params.divisions as f64;
                let diameter = base_diameter_m + (top_diameter_m - base_diameter_m) * t;
                stations.push(BodyStation::round(
                    height_m * t,
                    (diameter / 2.0).max(1e-3),
                )?);
            }
            stations
        }
        CapsuleShape::Sphere { diameter_m } => {
            let radius = diameter_m / 2.0;
            let mut stations = Vec::with_capacity(params.divisions + 1);
            for index in 0..=params.divisions {
                let x = diameter_m * index as f64 / params.divisions as f64;
                let ring = (radius.powi(2) - (x - radius).powi(2)).sqrt().max(1e-3);
                stations.push(BodyStation::round(x, ring)?);
            }
            stations
        }
    };
    let mut body = ProceduralBody::new(params.name.clone(), stations, params.origin_body_m)?;
    let (crew_x0, crew_x1) = match params.shape {
        CapsuleShape::Frustum { height_m, .. } => (0.05 * height_m, 0.65 * height_m),
        CapsuleShape::Sphere { diameter_m } => (0.20 * diameter_m, 0.80 * diameter_m),
    };
    let crew = InteriorRegion {
        name: params.name.clone(),
        x0_m: crew_x0,
        x1_m: crew_x1,
        kind: crew_kind(params),
        atmosphere: params.atmosphere,
        control_core: None,
    };
    crew.validate()?;
    body.regions = vec![crew];
    // No auto-fitted details: heat shields and docking ports are separate
    // parts the author attaches (KSP-style), never baked into the
    // primitive. See `BodyHeatShield` and `BodyPort`.
    body.structure = params.structure.clone();
    body.validate()?;
    Ok(body)
}

fn crew_kind(params: &CapsuleParams) -> RegionKind {
    RegionKind::Crew {
        seats: params.crew,
        seat_mass_kg_each: params.couch_mass_kg_each,
        occupant_mass_kg_each: params.occupant_mass_kg_each,
        seat_pitch_m: None,
        abreast: Some(params.abreast),
        seat_style: params.seat_style,
        suited: params.suited,
        suit_mass_kg_each: params.suit_mass_kg_each,
        suit_type: crate::SuitType::HoseFed,
        control_station: false,
    }
}
