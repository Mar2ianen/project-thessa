//! Golden reconstruction fixtures: Juno-style and SimplePlanes-style bodies.
//!
//! Purpose: verify that the station-loft authoring model spans the two
//! reference editors' core vocabulary (Juno: New Origins round tanks,
//! nose cones, boat-tails with real fuel; SimplePlanes block fuselages
//! with cabin/cargo avionics splits) and that compilation preserves
//! their closed-form geometry. Dimensions below are generic
//! representative values in the spirit of those parts, not copied
//! proprietary part data; tolerances reflect that reconstruction grade.
//! These fixtures do NOT claim the solver reproduces any game's flight
//! model.

use glam::DVec3;

use crate::{
    BodyPort, BodyStation, BodyStructuralLayout, FuselageError, InteriorRegion, PortKind,
    ProceduralBody, RegionKind,
};

/// Juno-style stack reference values (generic 2 m-class sounding-rocket
/// stage: barrel tank, ogive nose, boat-tail, aft engine plate).
pub mod juno_stack {
    /// Overall length in metres.
    pub const LENGTH_M: f64 = 6.8;
    /// Barrel diameter in metres.
    pub const BARREL_DIAMETER_M: f64 = 2.0;
    /// Barrel tank span in metres.
    pub const TANK_LENGTH_M: f64 = 4.0;
}

/// SimplePlanes-style block reference values (generic 4-seat single:
/// boxy constant section with tapered nose and tail).
pub mod sp_block {
    /// Overall length in metres.
    pub const LENGTH_M: f64 = 5.8;
    /// Block half-width / half-height in metres.
    pub const HALF_WIDTH_M: f64 = 0.8;
    /// Block half-width / half-height in metres.
    pub const HALF_HEIGHT_M: f64 = 0.9;
}

/// Juno-style round stack: boat-tail, barrel tank with real propellant,
/// ogive nose, avionics bay, aft engine port and nose-hatch docking.
///
/// Stations run tail-to-nose. The nose keeps its sharp tip (Juno cones
/// are sharp); docking rides the barrel/ogive joint as a forward hatch.
pub fn juno_style_stack() -> Result<ProceduralBody, FuselageError> {
    let mut stations = vec![
        BodyStation::round(0.0, 0.55)?,
        BodyStation::round(0.9, 1.0)?,
        BodyStation::round(4.9, 1.0)?,
    ];
    // Tangent-ogive nose sampled like the constructor (base R=1, L=1.9).
    let rho = (1.0_f64 + 1.9_f64.powi(2)) / 2.0;
    for index in 1..=6 {
        let x = 1.9 * index as f64 / 6.0;
        let radius = (rho.powi(2) - x.powi(2)).sqrt() - (rho - 1.0);
        stations.push(BodyStation::round(4.9 + x, radius.max(1e-3))?);
    }
    let mut body = ProceduralBody::new("juno-style-stack", stations, DVec3::ZERO)?;
    body.regions = vec![
        InteriorRegion::new(
            "main-tank",
            0.9,
            4.9,
            RegionKind::Tank {
                propellant: thessa_sim_core::Propellant::LoxMethane,
                fill_fraction: 0.95,
            },
        )?,
        InteriorRegion::new("avionics", 4.9, 5.9, RegionKind::Avionics)?,
        InteriorRegion::new("nose-void", 5.9, 6.75, RegionKind::Empty)?,
    ];
    body.ports = vec![
        BodyPort::new("engine-aft", 0.0, 0.0, PortKind::EngineMount, 0.5)?,
        BodyPort::new(
            "hatch-forward",
            4.9,
            std::f64::consts::FRAC_PI_2,
            PortKind::Docking,
            0.8,
        )?,
        BodyPort::new(
            "hardpoint-belly",
            2.0,
            -std::f64::consts::FRAC_PI_2,
            PortKind::Attachment,
            0.3,
        )?,
    ];
    body.structure = Some(BodyStructuralLayout::metal_baseline());
    body.validate()?;
    Ok(body)
}

/// SimplePlanes-style block fuselage: boxy constant section (exponent 5)
/// with tapered tail and rounded nose, cabin plus cargo manifest mass,
/// avionics bay, belly hardpoints.
pub fn simpleplanes_style_block() -> Result<ProceduralBody, FuselageError> {
    let stations = vec![
        BodyStation::boxy(0.0, 0.7, 0.8, 5.0)?,
        BodyStation::boxy(1.0, 0.8, 0.9, 5.0)?,
        BodyStation::boxy(4.0, 0.8, 0.9, 5.0)?,
        BodyStation::boxy(5.2, 0.55, 0.6, 4.0)?,
        BodyStation::boxy(5.8, 0.2, 0.25, 3.0)?,
    ];
    let mut body = ProceduralBody::new("simpleplanes-style-block", stations, DVec3::ZERO)?;
    body.regions = vec![
        InteriorRegion::new("cabin", 1.0, 3.2, RegionKind::Cabin)?,
        InteriorRegion::new("avionics", 3.2, 4.0, RegionKind::Avionics)?,
        InteriorRegion::new(
            "baggage",
            4.0,
            5.2,
            RegionKind::Cargo {
                payload_mass_kg: 120.0,
            },
        )?,
        InteriorRegion::new("nose-void", 5.2, 5.8, RegionKind::Empty)?,
    ];
    body.ports = vec![
        BodyPort::new(
            "gear-forward",
            1.5,
            -std::f64::consts::FRAC_PI_2,
            PortKind::Attachment,
            0.25,
        )?,
        BodyPort::new(
            "gear-aft",
            3.5,
            -std::f64::consts::FRAC_PI_2,
            PortKind::Attachment,
            0.25,
        )?,
    ];
    body.structure = Some(BodyStructuralLayout::metal_baseline());
    body.validate()?;
    Ok(body)
}

/// Dream Chaser-style lifting-body fuselage reference values.
///
/// Generic 7 m-class crew lifting body: round top, flat chined bottom,
/// drooped nose centerline, blunt docking nose. Dimensions are
/// representative of the public HL-20/Dream Chaser class (NASA HL-20
/// full-scale ~9 m; this fixture is a notional 7 m vehicle), not copied
/// proprietary lines; tolerances below are regression-grade.
pub mod dream_chaser_body {
    /// Overall length in metres.
    pub const LENGTH_M: f64 = 7.0;
    /// Compiled enclosed volume anchor in m^3 (regression pin).
    pub const VOLUME_M3: f64 = 8.897;
    /// Frontal (reference) area anchor in m^2 (regression pin).
    pub const FRONTAL_AREA_M2: f64 = 1.825;
}

/// Dream Chaser-style lifting body: asymmetric lifting sections (round
/// top, high-exponent chined bottom), nose-droop centerline tilt for
/// camber physics, cabin plus avionics interior, blunt-nose docking
/// port, aft engine port, carbon shell.
///
/// What this fixture proves beyond the Juno/SimplePlanes goldens:
/// asymmetric sections compile with camber-correct centroids, the
/// centerline tilt produces the cambered zero-lift shift (negative CL
/// at zero alpha for a drooped nose), and the Munk slope survives the
/// asymmetry. Chine vortex lift itself is a per-vehicle config factor
/// (see the Concorde `VORTEX_LIFT_FACTOR` calibration method), not a
/// number this fixture invents.
pub fn dream_chaser_style_body() -> Result<ProceduralBody, FuselageError> {
    let station = |x_m: f64,
                   half_width_m: f64,
                   top_height_m: f64,
                   bottom_height_m: f64,
                   top_exponent: f64,
                   bottom_exponent: f64,
                   offset_z_m: f64| {
        BodyStation::new(
            x_m,
            half_width_m,
            top_height_m,
            bottom_height_m,
            top_exponent,
            bottom_exponent,
            0.0,
            offset_z_m,
        )
    };
    let stations = vec![
        station(0.0, 0.75, 0.55, 0.50, 2.5, 5.0, 0.0)?,
        station(2.0, 0.85, 0.70, 0.50, 2.5, 6.0, 0.0)?,
        station(4.0, 0.80, 0.65, 0.42, 2.5, 6.0, 0.05)?,
        station(5.5, 0.60, 0.50, 0.30, 2.5, 6.0, -0.10)?,
        station(6.5, 0.30, 0.28, 0.15, 2.5, 5.0, -0.25)?,
        station(7.0, 0.08, 0.10, 0.06, 2.5, 4.0, -0.30)?,
    ];
    let mut body = ProceduralBody::new("dream-chaser-lifting-body", stations, DVec3::ZERO)?;
    body.regions = vec![
        InteriorRegion::new("cabin", 1.5, 4.5, RegionKind::Cabin)?,
        InteriorRegion::new("avionics", 4.5, 5.5, RegionKind::Avionics)?,
        InteriorRegion::new("nose-void", 5.5, 6.9, RegionKind::Empty)?,
    ];
    body.ports = vec![
        BodyPort::new("engine-aft", 0.0, 0.0, PortKind::EngineMount, 0.4)?,
        BodyPort::new(
            "docking-nose",
            7.0,
            std::f64::consts::FRAC_PI_2,
            PortKind::Docking,
            0.08,
        )?,
        BodyPort::new(
            "skid-forward",
            2.0,
            -std::f64::consts::FRAC_PI_2,
            PortKind::Attachment,
            0.25,
        )?,
        BodyPort::new(
            "skid-aft",
            4.0,
            -std::f64::consts::FRAC_PI_2,
            PortKind::Attachment,
            0.25,
        )?,
    ];
    body.structure = Some(BodyStructuralLayout {
        skin_material: crate::HullMaterial::carbon_fiber(),
        skin_gauge_mm: 1.5,
        frame_spacing_m: 1.0,
        frame_gauge_mm: 1.5,
        frame_width_mm: 30.0,
        tank_pressure_pa: 0.3e6,
        tank_material: thessa_sim_core::ChamberMaterial::nickel_superalloy(),
        wall_inset_mm: 10.0,
    });
    body.validate()?;
    Ok(body)
}
