//! Night-side city lights: a warm emissive mask over inhabited land.
//!
//! The mask bakes once at startup from the streamed world's inhabited
//! regions (optical proxy only: clustered warm dots, never simulated
//! activity). The GPU terrain shader samples it with the same equirect
//! mapping as the albedo globe and gates it by the terminator; the CPU
//! fallback path has no emissive slot, so cities are a GPU-path effect
//! (documented degradation, never a second code path for the mask itself).
//!
//! Disabled (`[night_lights] enabled = false`) inserts no resources and
//! spawns nothing: effectively free.

use super::*;
use crate::atmosphere::GraphicsResolved;
use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use thessa_worldgen_rocky::civilization::InhabitedRegion;

/// Angular radius of a metro cluster at reference population (radians).
/// Real cities sprawl ~10-100 km; the mask only needs to read as clustered
/// light from orbit and high flight, not resolve streets.
const CITY_BASE_RADIUS_RAD: f64 = 0.012;
/// Population reach: the biggest region sprawls this much wider.
const CITY_RADIUS_POP_GAIN: f64 = 0.05;

/// Baked mask resource. Always inserted (possibly dark) so shader bindings
/// stay valid; the resolved intensity gates the effect, not the handle.
/// Warm tint (1.0, 0.72, 0.42) and pre-exposure gain (60.0) live in the
/// terrain shader next to the terminator gate.
#[derive(Resource)]
pub(crate) struct NightLights {
    pub image: Handle<Image>,
}

/// Bake resolution by quality: optical mask only, coarse is fine.
pub(crate) fn mask_size(quality: thessa_graphics::Quality) -> (u32, u32) {
    match quality {
        thessa_graphics::Quality::Low => (128, 64),
        thessa_graphics::Quality::Medium => (256, 128),
        thessa_graphics::Quality::High => (512, 256),
    }
}

fn hash01(x: u32, y: u32, seed: u64) -> f64 {
    let mut h = seed
        .wrapping_add((x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((y as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9));
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    (h as f64) / (u64::MAX as f64)
}

/// Equirect direction matching the terrain globe mapping (`surface_uv` in
/// the GPU shader and `dir_from_latlon` in worldgen): east-positive
/// longitude is `atan2(-z, x)`, v = 0 at the north pole.
fn dir_from_uv(u: f64, v: f64) -> [f64; 3] {
    let lon = u * std::f64::consts::TAU;
    let lat = (0.5 - v) * std::f64::consts::PI;
    [lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin()]
}

/// Settlement envelope (0..1) for a direction: population-weighted cluster
/// falloff over inhabited regions. Pure geometry; the bake speckles blocks
/// inside it.
fn settlement_gate(dir: [f64; 3], regions: &[InhabitedRegion], max_pop: u64) -> f64 {
    let mut gate: f64 = 0.0;
    for r in regions {
        let c = r.center_dir;
        let dot = (dir[0] * c[0] + dir[1] * c[1] + dir[2] * c[2]).clamp(-1.0, 1.0);
        let ang = dot.acos();
        let pop = (r.population as f64 / max_pop.max(1) as f64).cbrt();
        let weight = pop * r.suitability.clamp(0.0, 1.0);
        let radius = CITY_BASE_RADIUS_RAD + CITY_RADIUS_POP_GAIN * pop;
        let t = 1.0 - (ang / radius).clamp(0.0, 1.0);
        gate = gate.max(weight * t * t * (3.0 - 2.0 * t));
    }
    gate
}

/// Bake an R8 city mask: `land_at` decides buildable ground, regions seed
/// population-weighted clusters. Deterministic for fixed inputs.
pub(crate) fn bake_city_mask(
    regions: &[InhabitedRegion],
    land_at: impl Fn([f64; 3]) -> bool,
    w: u32,
    h: u32,
) -> Vec<u8> {
    assert!((w > 0) && (h > 0));
    let max_pop = regions
        .iter()
        .map(|r| r.population)
        .max()
        .unwrap_or(0)
        .max(1);
    let mut out = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        let v = (y as f64 + 0.5) / h as f64;
        for x in 0..w {
            let u = (x as f64 + 0.5) / w as f64;
            let dir = dir_from_uv(u, v);
            let gate = if land_at(dir) {
                settlement_gate(dir, regions, max_pop)
            } else {
                0.0
            };
            // City blocks fill the settled envelope densely (ISS metros read
            // near-solid in the cores with dark lanes out), brightness
            // following the envelope with per-block variation.
            let speckle = hash01(x, y, 0xC17E);
            let lit = if speckle < gate {
                gate * (0.6 + 0.4 * hash01(x, y, 0x51E))
            } else {
                0.0
            };
            out.push((lit.clamp(0.0, 1.0) * 255.0) as u8);
        }
    }
    out
}

pub(crate) struct NightLightsPlugin;

impl Plugin for NightLightsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, bake_night_lights_once);
    }
}

/// Bake on first frames once the streamed field exists. One-time CPU cost
/// (tens of thousands of coarse height reads); the handle stays valid for
/// the session, so shader bindings never dangle.
fn bake_night_lights_once(
    mut done: Local<bool>,
    terrain: Option<Res<terrain::WorldTerrain>>,
    graphics: Option<Res<GraphicsResolved>>,
    mut images: ResMut<Assets<Image>>,
    mut commands: Commands,
) {
    if *done {
        return;
    }
    let Some(world) = terrain.as_deref() else {
        return;
    };
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let (w, h) = mask_size(r.night_lights_quality);
    let field = world.field.clone();
    let started = std::time::Instant::now();
    // Coarse height prefix is enough for a land mask; fine bands would only
    // alias into the 1-8 km texels.
    let land_at = move |dir: [f64; 3]| field.sample_surface(dir, 64_000.0).height_m > 5.0;
    let mask = bake_city_mask(&world.field.inhabited_regions, land_at, w, h);
    let lit_texels = mask.iter().filter(|v| **v > 0).count();
    info!(
        "[night] baked {w}x{h} city mask ({} lit texels) in {:.3}s",
        lit_texels,
        started.elapsed().as_secs_f64()
    );
    let image = images.add(Image::new(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        mask,
        TextureFormat::R8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    ));
    commands.insert_resource(NightLights { image });
    *done = true;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region(dir: [f64; 3], population: u64) -> InhabitedRegion {
        InhabitedRegion {
            id: 1,
            center_dir: dir,
            population,
            height_m: 100.0,
            slope: 0.05,
            temperature_k: 285.0,
            water_distance_m: 5000.0,
            suitability: 0.9,
        }
    }

    #[test]
    fn bake_is_deterministic_and_bounded() {
        let regions = [region([1.0, 0.0, 0.0], 1_000_000)];
        let land = |_: [f64; 3]| true;
        let a = bake_city_mask(&regions, land, 64, 32);
        let b = bake_city_mask(&regions, land, 64, 32);
        assert_eq!(a.len(), 64 * 32);
        assert_eq!(a, b);
    }

    #[test]
    fn empty_world_stays_dark() {
        let water = bake_city_mask(&[], |_: [f64; 3]| true, 32, 16);
        assert!(water.iter().all(|v| *v == 0));
        let regions = [region([1.0, 0.0, 0.0], 1_000_000)];
        let ocean = bake_city_mask(&regions, |_: [f64; 3]| false, 32, 16);
        assert!(ocean.iter().all(|v| *v == 0));
    }

    #[test]
    fn settlement_gate_tracks_population_and_distance() {
        let regions = [region([1.0, 0.0, 0.0], 5_000_000)];
        // Core envelope near full weight (population share 1 x suitability).
        let core = settlement_gate([1.0, 0.0, 0.0], &regions, 5_000_000);
        assert!((core - 0.9).abs() < 1e-9, "{core}");
        // Antipode and empty inputs stay dark.
        assert_eq!(settlement_gate([-1.0, 0.0, 0.0], &regions, 5_000_000), 0.0);
        assert_eq!(settlement_gate([1.0, 0.0, 0.0], &[], 1), 0.0);
        // Monotone falloff along the equator away from the center.
        let mut prev = 2.0;
        for i in 0..=10 {
            let lon = i as f64 * 0.01;
            let g = settlement_gate([lon.cos(), 0.0, -lon.sin()], &regions, 5_000_000);
            assert!(g <= prev + 1e-12, "{g} after {prev}");
            prev = g;
        }
    }

    #[test]
    fn settled_land_lights_up_but_stays_speckled() {
        // Tiled metros across the globe: the lit fraction concentrates
        // instead of depending on a few hashes.
        let mut regions = Vec::new();
        let mut id = 0;
        for lat_deg in [-45.0f64, 0.0, 45.0] {
            for lon_deg in [0.0f64, 60.0, 120.0, 180.0, 240.0, 300.0] {
                let lat = lat_deg.to_radians();
                let lon = lon_deg.to_radians();
                id += 1;
                let mut r = region(
                    [lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin()],
                    2_000_000,
                );
                r.id = id;
                regions.push(r);
            }
        }
        let mask = bake_city_mask(&regions, |_: [f64; 3]| true, 256, 128);
        let lit = mask.iter().filter(|v| **v > 0).count() as f64 / mask.len() as f64;
        assert!(lit > 0.001 && lit < 0.25, "lit fraction {lit:.4}");
    }

    #[test]
    fn mask_uv_matches_globe_mapping() {
        // dir_from_uv inverts surface_uv: equator u = 0 faces +X, v = 0 north.
        let d = dir_from_uv(0.0, 0.5);
        assert!((d[0] - 1.0).abs() < 1e-12 && d[1].abs() < 1e-12);
        let n = dir_from_uv(0.37, 0.0);
        assert!((n[1] - 1.0).abs() < 1e-12, "{n:?}");
        for (u, v) in [(0.0, 0.5), (0.25, 0.25), (0.7, 0.9), (0.99, 0.01)] {
            let d = dir_from_uv(u, v);
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-12, "{u} {v} {d:?}");
        }
    }
}
