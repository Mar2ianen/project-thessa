//! Parametric capsule cabins (Mercury/Gemini/Apollo/Orion/Dragon class).
//!
//! A capsule is a blunt frustum (or sphere) pressure vessel with the crew
//! low near the heat shield, couches side-by-side, and a sea-level cabin
//! atmosphere. Dimensions below are representative public values,
//! regression-grade like the other goldens — not copied proprietary lines.
//! Airplane-like cabins are future work; the transverse `abreast` rows
//! introduced for couches already anticipate multi-aisle layouts.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::{
    BodyPort, BodyStation, BodyStructuralLayout, CabinAtmosphere, FuselageError, InteriorRegion,
    PortKind, ProceduralBody, RegionKind, SeatStyle,
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
    let (crew_x0, crew_x1, top_diameter) = match params.shape {
        CapsuleShape::Frustum {
            top_diameter_m,
            height_m,
            ..
        } => (0.05 * height_m, 0.65 * height_m, top_diameter_m),
        CapsuleShape::Sphere { diameter_m } => (0.20 * diameter_m, 0.80 * diameter_m, 0.0),
    };
    let mut crew = InteriorRegion::new(params.name.clone(), crew_x0, crew_x1, crew_kind(params))?;
    crew.atmosphere = params.atmosphere;
    crew.validate()?;
    body.regions = vec![crew];
    // Nose docking hatch where the frustum top fits one (spheres keep
    // their side hatch, outside this slice).
    if top_diameter >= 0.5 {
        let diameter = (0.8 * top_diameter).min(0.8);
        body.ports = vec![BodyPort::new(
            "docking-nose",
            match params.shape {
                CapsuleShape::Frustum { height_m, .. } => height_m,
                CapsuleShape::Sphere { diameter_m } => diameter_m,
            },
            std::f64::consts::FRAC_PI_2,
            PortKind::Docking,
            diameter,
        )?];
    }
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
    }
}

fn base_params(
    name: &str,
    shape: CapsuleShape,
    crew: u32,
    seat_style: SeatStyle,
    abreast: u32,
) -> CapsuleParams {
    CapsuleParams {
        name: name.into(),
        shape,
        crew,
        seat_style,
        abreast,
        couch_mass_kg_each: if seat_style == SeatStyle::Couch {
            30.0
        } else {
            12.0
        },
        occupant_mass_kg_each: 0.0,
        atmosphere: Some(CabinAtmosphere::sea_level()),
        structure: Some(BodyStructuralLayout::metal_baseline()),
        origin_body_m: DVec3::ZERO,
        divisions: 6,
    }
}

/// Mercury-class: 1.89 m blunt bell, single couch.
pub fn mercury() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "mercury-capsule",
        CapsuleShape::Frustum {
            base_diameter_m: 1.89,
            top_diameter_m: 0.75,
            height_m: 2.9,
        },
        1,
        SeatStyle::Couch,
        1,
    ))
}

/// Gemini-class: two ejection seats side-by-side under a 3.05 m frustum.
pub fn gemini() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "gemini-capsule",
        CapsuleShape::Frustum {
            base_diameter_m: 3.05,
            top_diameter_m: 1.0,
            height_m: 3.4,
        },
        2,
        SeatStyle::Upright,
        2,
    ))
}

/// Apollo CM-class: 3.91 m cone, three couches abreast.
pub fn apollo_cm() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "apollo-cm",
        CapsuleShape::Frustum {
            base_diameter_m: 3.91,
            top_diameter_m: 1.0,
            height_m: 3.23,
        },
        3,
        SeatStyle::Couch,
        3,
    ))
}

/// Orion-class (Artemis): 5.0 m cone, four couches in two rows.
pub fn orion() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "orion-cm",
        CapsuleShape::Frustum {
            base_diameter_m: 5.0,
            top_diameter_m: 1.3,
            height_m: 3.3,
        },
        4,
        SeatStyle::Couch,
        2,
    ))
}

/// Crew Dragon-class: 4.0 m cone, four upright seats in two rows.
pub fn crew_dragon() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "crew-dragon",
        CapsuleShape::Frustum {
            base_diameter_m: 4.0,
            top_diameter_m: 1.6,
            height_m: 4.5,
        },
        4,
        SeatStyle::Upright,
        2,
    ))
}

/// Vostok-class: 2.3 m sphere, single couch, no nose dock.
pub fn vostok() -> Result<ProceduralBody, FuselageError> {
    capsule_body(&base_params(
        "vostok-capsule",
        CapsuleShape::Sphere { diameter_m: 2.3 },
        1,
        SeatStyle::Couch,
        1,
    ))
}
