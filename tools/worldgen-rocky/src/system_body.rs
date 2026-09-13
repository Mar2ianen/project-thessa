//! Celestial body lookup from the system design (`data/system.toml`).
//!
//! The worldgen recipe must NOT duplicate physical body data. Radius, mass,
//! orbit and atmosphere metadata resolve by stable body ID; the recipe keeps
//! only worldgen/art-direction values unless an explicit physical override
//! is requested on the command line.

use serde::Deserialize;

/// Minimal tolerant view of a `[[planet]]` / `[[moon]]` entry.
/// Unknown fields are ignored so system design can evolve freely.
#[derive(Debug, Clone, Deserialize)]
pub struct BodyEntry {
    pub id: String,
    pub host: Option<String>,
    pub semi_major_axis_km: Option<f64>,
    pub period_hours_design: Option<f64>,
    pub eccentricity: Option<f64>,
    pub mass_earth: Option<f64>,
    pub radius_km: Option<f64>,
    #[serde(default)]
    pub rotation_period_hours: Option<f64>,
    #[serde(default)]
    pub tidal_lock: bool,
    #[serde(default)]
    pub atmosphere_bar: Option<f64>,
    #[serde(default)]
    pub atmosphere: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyKind {
    Planet,
    Moon,
}

#[derive(Debug, Clone)]
pub struct ResolvedBody {
    pub kind: BodyKind,
    pub entry: BodyEntry,
}

impl ResolvedBody {
    pub fn radius_m(&self) -> Result<f64, String> {
        match self.entry.radius_km {
            Some(r) if r.is_finite() && r > 0.0 => Ok(r * 1000.0),
            _ => Err(format!("body {} has no valid radius_km", self.entry.id)),
        }
    }
}

#[derive(Debug, Deserialize)]
struct SystemFile {
    #[serde(default)]
    planet: Vec<BodyEntry>,
    #[serde(default)]
    moon: Vec<BodyEntry>,
}

/// Load and resolve a body by stable ID (planets and moons share one namespace).
pub fn resolve_body(system_toml: &str, id: &str) -> Result<ResolvedBody, String> {
    let file: SystemFile = toml::from_str(system_toml).map_err(|e| format!("system TOML: {e}"))?;
    if let Some(entry) = file.planet.iter().find(|b| b.id == id) {
        return Ok(ResolvedBody {
            kind: BodyKind::Planet,
            entry: entry.clone(),
        });
    }
    if let Some(entry) = file.moon.iter().find(|b| b.id == id) {
        return Ok(ResolvedBody {
            kind: BodyKind::Moon,
            entry: entry.clone(),
        });
    }
    Err(format!("unknown body {id:?} (checked planets and moons)"))
}

/// Rocky gate: this dev tool refuses gas/ice giants by size heuristic
/// (rocky terrestres are far below 20 000 km radius).
pub fn require_rocky(body: &ResolvedBody) -> Result<f64, String> {
    let radius_m = body.radius_m()?;
    if radius_m > 20_000_000.0 {
        return Err(format!(
            "body '{}' is not rocky (radius {:.0} km); worldgen-rocky is rocky-only",
            body.entry.id,
            radius_m / 1000.0
        ));
    }
    Ok(radius_m)
}
pub fn check_radius_agreement(
    recipe_radius_m: f64,
    body: &ResolvedBody,
    allow_override: bool,
) -> Result<(), String> {
    let canonical = body.radius_m()?;
    if (recipe_radius_m - canonical).abs() <= 1.0 {
        return Ok(());
    }
    if allow_override {
        return Ok(());
    }
    Err(format!(
        "worldgen radius {recipe_radius_m:.0} m disagrees with canonical body radius {canonical:.0} m \
         for '{}'; pass --override-radius to state an explicit physical override",
        body.entry.id
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYSTEM: &str = include_str!("../../../data/system.toml");

    #[test]
    fn thessa_resolves_as_moon_with_radius() {
        let body = resolve_body(SYSTEM, "thessa").expect("thessa exists");
        assert_eq!(body.kind, BodyKind::Moon);
        assert_eq!(body.radius_m().expect("radius"), 3_200_000.0);
        assert_eq!(body.entry.host.as_deref(), Some("nereid"));
        assert!(body.entry.tidal_lock);
    }

    #[test]
    fn unknown_body_rejected() {
        assert!(resolve_body(SYSTEM, "mordor").is_err());
    }

    #[test]
    fn gas_giant_rejected_as_non_rocky() {
        let nereid = resolve_body(SYSTEM, "nereid").expect("nereid exists");
        assert!(require_rocky(&nereid).is_err());
        let thessa = resolve_body(SYSTEM, "thessa").expect("thessa exists");
        assert!(require_rocky(&thessa).is_ok());
    }

    #[test]
    fn radius_disagreement_rejected_without_override() {
        let body = resolve_body(SYSTEM, "thessa").expect("thessa");
        assert!(check_radius_agreement(3_200_000.0, &body, false).is_ok());
        assert!(check_radius_agreement(3_000_000.0, &body, false).is_err());
        assert!(check_radius_agreement(3_000_000.0, &body, true).is_ok());
    }
}
