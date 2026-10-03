//! Canonical launch-site derivation shared by the client survey and the
//! headless server.
//!
//! Both processes build the same world field from the same baked-in recipe
//! and source-checked frozen erosion artifact
//! and run the same deterministic bookmark scan, so they agree on terrain
//! height and spawn state without ever transferring world state (AGENTS
//! multiplayer rule: input/commands, never world state). The site direction
//! is player intent (survey choice); the field is derived deterministically.

use std::sync::{Arc, OnceLock};

use thessa_sim_core::{BakedEphemeris, SimTime, SystemConfig};
use thessa_worldgen_rocky::{
    field::{PlanetField, field_from_manifest},
    lod,
    spec_recipe::{SpecRecipe, manifest_from_spec},
    sphere::dir_from_latlon,
};

/// Canonical source and frozen erosion, bundled identically in client/server.
/// A mismatch fails startup; there is no silent analytic fallback or cwd lookup.
/// Repeated launch/contact consumers share one immutable field per process.
pub fn canonical_world_field() -> Result<Arc<PlanetField>, String> {
    static FIELD: OnceLock<Result<Arc<PlanetField>, String>> = OnceLock::new();
    FIELD.get_or_init(build_canonical_world_field).clone()
}

fn build_canonical_world_field() -> Result<Arc<PlanetField>, String> {
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
            .map_err(|e| format!("world recipe is invalid: {e}"))?;
    let manifest = manifest_from_spec(&recipe).map_err(|e| format!("world manifest: {e}"))?;
    let source =
        field_from_manifest(&manifest).map_err(|e| format!("canonical world field: {e}"))?;
    let frozen = thessa_worldgen_rocky::offline::FrozenSurface::read_gzip(
        &include_bytes!("../../../data/worldgen/thessa-erosion-v3.surface.json.gz")[..],
    )
    .map_err(|e| format!("bundled frozen surface: {e}"))?;
    source
        .with_frozen_erosion(&frozen)
        .map(Arc::new)
        .map_err(|e| format!("canonical frozen field: {e}"))
}

/// Deterministic survey bookmarks in COAST/HIGHLANDS/VOLCANIC score order.
/// Line-for-line the client survey scan: grid over real terrain, daylight
/// from the brightest configured star (never a named sun), ties broken by
/// strict improvement in scan order.
pub fn survey_bookmarks(
    field: &PlanetField,
    ephemeris: &BakedEphemeris,
) -> Result<[[f64; 3]; 3], String> {
    let system: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
        .map_err(|e| format!("system is invalid: {e}"))?;
    let daylight_star = system
        .star
        .iter()
        .max_by(|a, b| a.luminosity_solar.total_cmp(&b.luminosity_solar))
        .ok_or("at least one configured star")?;
    let planet = ephemeris
        .body_state(
            ephemeris
                .body_id(&canonical_planet_id()?)
                .ok_or("terrain world")?,
            SimTime::EPOCH,
        )
        .map_err(|e| e.to_string())?;
    let star = ephemeris
        .body_state(
            ephemeris
                .body_id(&daylight_star.id)
                .ok_or("daylight star")?,
            SimTime::EPOCH,
        )
        .map_err(|e| e.to_string())?;
    let light = (star.position_inertial - planet.position_inertial).normalize();
    let daylight = [light.x, light.z, -light.y];
    let mut best = [(f64::INFINITY, [1.0, 0.0, 0.0]); 3];
    for lat in (-55..55).step_by(3) {
        for lon in (-180..180).step_by(3) {
            let dir = dir_from_latlon(lat as f64, lon as f64);
            if lod::dot(dir, daylight) < 0.35 {
                continue;
            }
            // Score the existing geological parent so fine erosion cannot
            // silently redirect the survey to an unrefined neighbouring site.
            // Spawn/contact/render height still comes from the final field.
            let s = field.regional_parent_sample(dir, 500.0);
            if s.height_m <= 0.0 || field.height_m(dir, 32.0) <= 0.0 {
                continue;
            }
            let scores = [
                (s.height_m - 100.0).abs() + (s.temperature_k - 284.0).abs() * 90.0,
                (s.height_m - 3200.0).abs() + (s.temperature_k - 268.0).abs() * 50.0,
                (s.height_m - 1500.0).abs() - s.geothermal_flux_w_m2 * 1000.0,
            ];
            for i in 0..3 {
                if scores[i] < best[i].0 {
                    best[i] = (scores[i], dir);
                }
            }
        }
    }
    // A hot sedimentary canyon is not a volcano. Prefer an authored volcanic
    // feature whose substrate really is volcanic and whose centre is daylight
    // land. This affects a camera bookmark, never the authority's terrain.
    let mut volcanic = None;
    for feature in &field.features {
        if !matches!(
            feature.feature,
            thessa_worldgen_rocky::features::Feature::ShieldVolcano { .. }
                | thessa_worldgen_rocky::features::Feature::VolcanicProvince { .. }
        ) {
            continue;
        }
        let dir = dir_from_latlon(feature.lat_deg, feature.lon_deg);
        if lod::dot(dir, daylight) < 0.35 {
            continue;
        }
        let sample = field.sample_surface(dir, 32.0);
        if sample.height_m <= 0.0
            || !matches!(
                sample.geology,
                thessa_worldgen_rocky::biomes::Geology::Basaltic
                    | thessa_worldgen_rocky::biomes::Geology::Hydrothermal
            )
        {
            continue;
        }
        let score = sample.geothermal_flux_w_m2;
        if volcanic
            .as_ref()
            .is_none_or(|(previous, _)| score > *previous)
        {
            volcanic = Some((score, dir));
        }
    }
    if let Some((_, dir)) = volcanic {
        best[2] = (0.0, dir);
    }
    Ok([best[0].1, best[1].1, best[2].1])
}

/// Field plus bookmarks in one call for server startup.
pub fn canonical_launch_setup(
    ephemeris: &BakedEphemeris,
) -> Result<(Arc<PlanetField>, [[f64; 3]; 3]), String> {
    let field = canonical_world_field()?;
    let sites = survey_bookmarks(&field, ephemeris)?;
    Ok((field, sites))
}

fn canonical_planet_id() -> Result<String, String> {
    let recipe: SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
            .map_err(|e| format!("world recipe is invalid: {e}"))?;
    Ok(recipe.planet.id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn launch_setup_is_deterministic_across_calls() {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).expect("system");
        let ephemeris = config.bake().expect("bake");
        let (field_a, sites_a) = canonical_launch_setup(&ephemeris).expect("setup");
        let (field_b, sites_b) = canonical_launch_setup(&ephemeris).expect("setup");
        assert!(field_a.has_frozen_erosion());
        assert!(Arc::ptr_eq(&field_a, &field_b), "reuse immutable field");
        assert_eq!(sites_a, sites_b);
        let volcanic_sample = field_a.sample_surface(sites_a[2], 32.0);
        assert!(
            matches!(
                volcanic_sample.geology,
                thessa_worldgen_rocky::biomes::Geology::Basaltic
                    | thessa_worldgen_rocky::biomes::Geology::Hydrothermal
            ),
            "volcanic survey must not be a hot sedimentary canyon: {:?}",
            volcanic_sample.geology
        );
        // Bookmarks must be real terrain directions, not defaults.
        for dir in sites_a {
            let length = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
            assert!((length - 1.0).abs() < 1.0e-9, "unit direction");
            assert!(field_a.sample_surface(dir, 500.0).height_m > 0.0);
            assert!(field_b.sample_surface(dir, 500.0).height_m > 0.0);
        }
    }

    #[test]
    fn bundled_surface_matches_independently_rebuilt_source() {
        let cached = canonical_world_field().expect("cached frozen field");
        let rebuilt = build_canonical_world_field().expect("independent frozen field");
        assert_eq!(cached.rivers().len(), 736);
        assert_eq!(cached.lake_basins().len(), 252);
        assert_eq!(cached.hydrology_grid_step_deg(), 0.5);
        assert_eq!(cached.regional_erosion().len(), 3);
        for lat in [-80.0, -25.0, 0.0, 30.0, 80.0] {
            for lon in [-180.0, -50.0, 0.0, 80.0, 179.9] {
                let dir = dir_from_latlon(lat, lon);
                assert_eq!(cached.height_m(dir, 32.0), rebuilt.height_m(dir, 32.0));
            }
        }
    }
}
