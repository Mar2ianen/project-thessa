//! Cheap atmospheric beauties: gas giants, cloud decks, aurora.
//!
//! Raster-portable (no RT, no particles): procedural RGBA baked once on the
//! CPU into ordinary Bevy `Image`s, then ordinary `StandardMaterial`s
//! (alpha-blend shells, additive aurora, opaque band textures). Per-frame
//! cost is a few transform copies; texture re-bakes (storm drift, aurora
//! curtains) run at 0.1-2 Hz and only on higher qualities.
//!
//! Every effect is gated by `graphics.toml` (`[gas_giant]`, `[clouds]`,
//! `[upper_atmosphere]`); quality selects bake size and re-bake rate, never
//! physics. The engine plume lives in `super::plume` (field-first volume
//! path per `docs/38`, cone impostor only on Low).
//!
//! CBT note: cloud decks and gas-giant detail share the terrain LOD address
//! space (`thessa_worldgen_rocky::lod::{cbt_node_for_tile, tile_for_cbt_node}`).
//! Shells are whole-sphere impostors, so the CBT is used for what it is good
//! at here — a deterministic LOD address (`TileKey <-> Node` round-trip,
//! tested below) that picks bake size and re-bake cadence — not for geometry.

use super::*;
use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    mesh::MeshVertexBufferLayoutRef,
    pbr::{Material, MaterialPipeline, MaterialPipelineKey, MaterialPlugin},
    reflect::TypePath,
    render::render_resource::{
        AsBindGroup, Extent3d, Face, RenderPipelineDescriptor, ShaderType,
        SpecializedMeshPipelineError, TextureDimension, TextureFormat,
    },
    shader::ShaderRef,
};
use thessa_graphics::Quality;
use thessa_worldgen_rocky::lod::{TileKey, cbt_node_for_tile, tile_for_cbt_node};

// ---------------------------------------------------------------------------
// Deterministic CPU noise (no new deps).

/// Deterministic lattice hash in [0,1).
fn hash2(seed: u64, x: i32, y: i32) -> f64 {
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

fn smootherstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

/// Tiling value noise in x (longitude wraps), clamped in y.
fn value_noise(seed: u64, x: f64, y: f64, x_period: i32) -> f64 {
    let xi = x.floor() as i32;
    let yi = y.floor() as i32;
    let xf = x - xi as f64;
    let yf = y - yi as f64;
    let wrap = |v: i32| v.rem_euclid(x_period.max(1));
    let a = hash2(seed, wrap(xi), yi);
    let b = hash2(seed, wrap(xi + 1), yi);
    let c = hash2(seed, wrap(xi), yi + 1);
    let d = hash2(seed, wrap(xi + 1), yi + 1);
    let u = smootherstep(xf);
    let v = smootherstep(yf);
    a + (b - a) * u + (c - a) * v + (a - b - c + d) * u * v
}

/// 4-octave fBm, output roughly in [0,1].
fn fbm(seed: u64, u: f64, v: f64, base_freq: f64) -> f64 {
    let mut sum = 0.0;
    let mut amp = 0.5;
    let mut freq = base_freq;
    for o in 0..4 {
        let period = (freq as i32).max(1);
        sum += amp * value_noise(seed + o as u64 * 101, u * freq, v * freq, period);
        amp *= 0.5;
        freq *= 2.0;
    }
    sum
}

fn lerp3(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0);
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

// ---------------------------------------------------------------------------
// Bake-size policy (quality -> pixels + re-bake cadence).

fn gas_giant_size(quality: Quality, big: bool) -> (u32, u32) {
    let w = match quality {
        Quality::Low => 512,
        Quality::Medium => 1024,
        Quality::High => 2048,
    };
    if big { (w, w / 2) } else { (w / 2, w / 4) }
}

fn cloud_size(quality: Quality) -> (u32, u32) {
    match quality {
        Quality::Low => (512, 256),
        Quality::Medium => (1024, 512),
        Quality::High => (2048, 1024),
    }
}

/// CBT LOD address used for a whole-sphere impostor bake: the coarsest tile
/// whose texel density covers the bake. Keeps one address space for terrain
/// tiles and beauty impostors.
fn cbt_level_for_width(width_px: u32) -> u8 {
    // Level L tile spans 2^L texels per face at 1:1; pick L covering width/4
    // (4 faces across the equator seam, roughly).
    let need = (width_px / 4).max(1);
    let level = (32 - need.leading_zeros() as u8).min(12);
    // Keep the shared address space honest: every reported level must map.
    debug_assert!(
        cbt_node_for_tile(TileKey {
            face: 0,
            level,
            x: 0,
            y: 0
        })
        .is_some()
    );
    level
}

// ---------------------------------------------------------------------------
// Gas giants.

const NEREID_ZONES: [[f32; 3]; 4] = [
    [0.96, 0.88, 0.70],
    [0.90, 0.74, 0.52],
    [0.82, 0.60, 0.38],
    [0.94, 0.84, 0.64],
];
const NEREID_BELTS: [[f32; 3]; 3] = [[0.72, 0.48, 0.28], [0.62, 0.38, 0.22], [0.78, 0.56, 0.34]];
const VESPER_ZONES: [[f32; 3]; 3] = [[0.62, 0.78, 0.82], [0.52, 0.68, 0.74], [0.70, 0.82, 0.84]];
const VESPER_BELTS: [[f32; 3]; 2] = [[0.40, 0.56, 0.62], [0.34, 0.48, 0.56]];

#[derive(Debug, Clone, Copy)]
struct GiantStyle {
    zones: &'static [[f32; 3]],
    belts: &'static [[f32; 3]],
    seed: u64,
    band_freq: f64,
    turbulence: f32,
    storm_lat: f32,
    storm_lon: f32,
    storm_size: f32,
}

const NEREID_STYLE: GiantStyle = GiantStyle {
    zones: &NEREID_ZONES,
    belts: &NEREID_BELTS,
    seed: 0x9E12_1D00,
    band_freq: 9.0,
    turbulence: 0.35,
    storm_lat: -0.35,
    storm_lon: 0.55,
    storm_size: 0.16,
};

const VESPER_STYLE: GiantStyle = GiantStyle {
    zones: &VESPER_ZONES,
    belts: &VESPER_BELTS,
    seed: 0x005E_EED0,
    band_freq: 6.0,
    turbulence: 0.22,
    storm_lat: 0.45,
    storm_lon: 2.4,
    storm_size: 0.10,
};

/// Procedural band texture. `drift` slides turbulence/storm longitude
/// (storm-drift animation without a shader: caller re-bakes at ~0.1 Hz).
fn bake_gas_giant(style: GiantStyle, w: u32, h: u32, drift: f64, limb: bool) -> Vec<u8> {
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    for y in 0..h {
        let lat = (y as f64 + 0.5) / h as f64 * 2.0 - 1.0; // -1..1
        for x in 0..w {
            let lon = (x as f64 + 0.5) / w as f64 * std::f64::consts::TAU + drift;
            // Band coordinate with shear turbulence advected by latitude.
            let turb = fbm(
                style.seed,
                lon / std::f64::consts::TAU * style.band_freq,
                lat * style.band_freq * 0.5 + drift * 0.05,
                style.band_freq,
            ) - 0.5;
            let band = (lat * style.band_freq + turb * style.turbulence as f64 * style.band_freq)
                .sin()
                * 0.5
                + 0.5;
            let zone_idx =
                ((lat.abs() * style.zones.len() as f64 * 1.7) as usize) % style.zones.len();
            let belt_idx =
                ((lat.abs() * style.belts.len() as f64 * 2.3 + 0.7) as usize) % style.belts.len();
            let mut rgb = lerp3(
                style.zones[zone_idx],
                style.belts[belt_idx],
                band as f32 * 0.75,
            );
            // Fine grain so belts are not flat gradients.
            let grain = fbm(style.seed ^ 0x9E37, lon * 3.0, lat * 24.0, 24.0) as f32 - 0.5;
            for c in rgb.iter_mut() {
                *c += grain * 0.08;
            }
            // Oval storm: warm core + pale rim.
            let dlat = (lat - style.storm_lat as f64) / style.storm_size as f64;
            let mut dlon = (lon - style.storm_lon as f64 - drift * 0.2) % std::f64::consts::TAU;
            if dlon > std::f64::consts::PI {
                dlon -= std::f64::consts::TAU;
            }
            if dlon < -std::f64::consts::PI {
                dlon += std::f64::consts::TAU;
            }
            let dlon = dlon / (style.storm_size as f64 * 1.8);
            let r2 = dlat * dlat + dlon * dlon;
            if r2 < 1.0 {
                let core = (1.0 - r2.min(1.0)) as f32;
                let storm = lerp3([0.98, 0.92, 0.80], [0.85, 0.45, 0.25], core * 0.6);
                rgb = lerp3(rgb, storm, core * 0.85);
            }
            // Fake limb darkening: gentle poleward rolloff (view-dependent
            // fresnel needs a shader; this keeps contrast honest and cheap).
            if limb {
                let limb_t = 1.0 - 0.18 * lat.abs() as f32;
                for c in rgb.iter_mut() {
                    *c *= limb_t;
                }
            }
            for c in rgb.iter_mut() {
                *c = c.clamp(0.0, 1.0);
            }
            px.extend_from_slice(&[
                (rgb[0] * 255.0) as u8,
                (rgb[1] * 255.0) as u8,
                (rgb[2] * 255.0) as u8,
                255,
            ]);
        }
    }
    px
}

// ---------------------------------------------------------------------------
// Clouds.

/// Single deck bake. Alpha = coverage-shaped domain-warped fBm (EVE-style
/// weather map); RGB near-white with slight density shading so decks read
/// as thick cloud, not haze. Earth-ref (ISS daytime): dense cores pure
/// white, thin skirts cool gray-blue with a warm sun side.
/// `aniso`: (u, v) frequency multipliers — cirrus passes a vertically
/// tightened pair for streaks instead of puffs. The u multiplier must stay
/// integral (the wrap period only tiles integer spans); v is unwarped at
/// the poles and free.
fn bake_clouds(
    seed: u64,
    w: u32,
    h: u32,
    coverage: f32,
    opacity: f32,
    aniso: (f64, f64),
) -> Vec<u8> {
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    // Widen the transition: decks keep hard cores but grow bright skirts.
    // Calibrated so coverage 0.55 reads ~40% broken deck (warp biases the
    // field low, hence the below-unity gain).
    let edge0 = (coverage * 0.76).clamp(0.0, 0.95);
    let (au, av) = aniso;
    for y in 0..h {
        let v = y as f64 / h as f64;
        let stretch = 1.0 - 0.35 * (1.0 - (v * std::f64::consts::PI).sin()); // polar pinch
        for x in 0..w {
            let u = x as f64 / w as f64;
            // Domain warp: a low-frequency offset field bends the main
            // octaves into natural swirls instead of blobby lattice cells.
            // Warp periods (2, 4) divide the base frequencies, so the seam
            // still tiles.
            let wx = fbm(seed ^ 0x9E9, u * 2.0, v * 4.0, 2.0) - 0.5;
            let wy = fbm(seed ^ 0xBEE, u * 2.0 + 5.2, v * 4.0 + 1.3, 2.0) - 0.5;
            // Slanted fronts: shearing u by latitude breaks the zonal
            // banding that pure lat/long octaves print (gas-giant look).
            // Shear is v-only, so the u seam still tiles.
            let us = u + 0.35 * (v - 0.5);
            let d = fbm(
                seed,
                (us * 10.0 + wx * 1.8) * au,
                (v * 20.0 * stretch + wy * 1.8 + 3.1) * av,
                10.0,
            );
            let swirl = fbm(seed ^ 0x51AB, us * 4.0 * au + d * 1.5, v * 8.0 * av, 4.0);
            // Billow octave: small cells riding the large field so decks read
            // as structured tops instead of smooth blobs. Bake-time only.
            // Faded toward the poles: the meridional pinch squeezes this
            // octave into coherent ringing arches there (measured).
            let polar = (v * std::f64::consts::PI).sin().powf(0.7);
            let billow = fbm(
                seed ^ 0xB110,
                (us * 24.0 + d * 2.0) * au,
                (v * 40.0 * stretch + 9.7) * av,
                24.0,
            ) * (0.25 + 0.75 * polar);
            let field = (d * 0.46 + swirl * 0.28 + billow * 0.26) as f32;
            let a = ((field - edge0) / (1.0 - edge0).max(1e-3)).clamp(0.0, 1.0);
            let a = (a * (1.6 - 0.6 * a)).clamp(0.0, 1.0) * opacity;
            // Density shading: white cores, steel-grey body, cool thin skirts.
            // The full 0.55..1.0 span keeps structure under minification:
            // an all-0.9 wash reads as cream from orbit, not cloud.
            let core = a.clamp(0.0, 1.0);
            let brightness = 0.55 + 0.45 * core;
            let cool = (1.0 - core) * 0.25;
            // R warm, B cool: thin edges go steel-blue, cores white.
            let r = (255.0 * brightness).clamp(0.0, 255.0) as u8;
            let g = (255.0 * (brightness - cool * 0.35)).clamp(0.0, 255.0) as u8;
            let b = (255.0 * (brightness - cool * 0.05)).clamp(0.0, 255.0) as u8;
            px.extend_from_slice(&[r, g, b, (a.clamp(0.0, 1.0) * 255.0) as u8]);
        }
    }
    px
}

// ---------------------------------------------------------------------------
// Cloud shadows: analytic sampling of the deck bake.
//
// The shells are ordinary textured spheres, so a ground point's shadow UV is
// just the bake texel the shell renderer samples for the same direction.
// Shadows align with the visible clouds by construction; the only model
// inputs are the deck rotation (shared with the follower below) and a sun
// parallax offset for the physical deck height (the impostor shell rides an
// exaggerated radius, never used here).

/// Physical low-deck altitude for the shadow parallax (metres): cumulus
/// base rides ~3 km above the reference geoid; cirrus is not cast into
/// shadows (thin, high, diffuse).
///
/// Distinct from [`CLOUD_SLAB_LOW`]: the volume-march slab (1.008..1.014 radii,
/// ~25..45 km) and shell impostors ride an exaggerated orbital radius to clear
/// planetary terrain relief and avoid z-fighting from orbit, but ground-level
/// shadow parallax on terrain must use physical cumulus base altitude (3,000 m).
pub(crate) const CLOUD_SHADOW_DECK_HEIGHT_M: f64 = 3000.0;
/// Deep-core shadow depth; thin skirts fade out through the alpha shaping.
pub(crate) const CLOUD_SHADOW_STRENGTH: f32 = 0.45;

/// Deck relief heights as FRACTIONS of the mesh radius: the shell meshes
/// are unit spheres later scaled by the planet radius, so a metre value
/// here would dome by thousands of radii (screenshot-proven giant sheets).
/// 0.00125 rides ~4 km over Thessa; other worlds scale proportionally.
pub(crate) const CLOUD_RELIEF_LOW_FRAC: f32 = 0.00125;
pub(crate) const CLOUD_RELIEF_HIGH_FRAC: f32 = 0.00078;

/// Volume-march slab as FRACTIONS of the planet radius (same unit-sphere
/// contract as relief): the low deck marches a ~19 km slab over Thessa,
/// the cirrus a thinner veil. Centered on the impostor shell scales in
/// [`CLOUD_DECKS`] so the volume and fallback shells coincide on screen.
pub(crate) const CLOUD_SLAB_LOW: (f32, f32) = (1.008, 1.014);
pub(crate) const CLOUD_SLAB_HIGH: (f32, f32) = (1.018, 1.022);
/// Visual extinction per planet radius inside the slab (appearance only,
/// like `density_scale`): cores go near-opaque with modeled crevices, skirts
/// stay translucent. Column-bench calibrated (thin edges glow, dense tops run
/// hot white); lower values wash the disc flat grey (screenshot-measured).
pub(crate) const CLOUD_EXT_LOW: f32 = 1000.0;
pub(crate) const CLOUD_EXT_HIGH: f32 = 350.0;
/// Sun self-shadow taps span a fraction of the slab thickness: turret
/// scale, not system scale. Long taps average whole cloud systems and the
/// local light-and-shadow modeling vanishes (flat white stickers); short
/// taps let bump tops catch sun while crevices shadow.
pub(crate) const CLOUD_SUN_SPAN_FRAC: f32 = 0.8;
/// Billow-detail amplitude perturbing the weather field before the floor
/// clean (Schneider): detail swings the field and carves organic edges plus
/// interior turrets and crevices. Too low reverts to smooth stickers.
pub(crate) const CLOUD_DETAIL_AMP: f32 = 1.6;
/// Coverage floor in the march: the bake carries the coverage shaping,
/// this only cuts the skirt noise so thin haze never films the disc.
pub(crate) const CLOUD_COVERAGE_FLOOR: f32 = 0.08;

/// Alpha channel grid out of an interleaved RGBA8 bake.
fn alpha_grid(px: &[u8]) -> Vec<u8> {
    let (chunks, _) = px.as_chunks::<4>();
    chunks.iter().map(|c| c[3]).collect()
}

/// Deck placement shared by the shell spawner, the follower, and the shadow
/// samplers: (body, deck, shell scale factor, differential spin rad/s).
/// Single source of truth; the spawn loop below reads this table.
///
/// Spins are physical zonal breezes on top of co-rotation (prograde +):
/// ~15 m/s low-deck drift, ~40 m/s cirrus jet. The old 0.004 rad/s spun the
/// deck at 12 km/s ground speed — visibly absurd minute-to-minute drift.
pub(crate) const CLOUD_DECKS: &[(&str, u8, f32, f32)] = &[
    ("thessa", 0, 1.012, 5.0e-6),
    ("thessa", 1, 1.020, 1.25e-5),
    ("pelagos", 0, 1.010, 5.0e-6),
    ("borea", 0, 1.010, 5.0e-6),
];

/// Inverse of the Bevy Sphere mesh mapping (poles on local +Z, u from +X
/// toward +Y, v = 0 at the +Z pole): bake-texel coordinates for a shell-local
/// unit direction. Mirrored in the GPU terrain shader (`shell_mesh_uv`);
/// keep the two in sync.
pub(crate) fn shell_mesh_uv(local_dir: Vec3) -> Vec2 {
    let u = (local_dir.y.atan2(local_dir.x) / std::f32::consts::TAU).fract();
    let u = if u < 0.0 { u + 1.0 } else { u };
    let v = 0.5 - local_dir.z.clamp(-1.0, 1.0).asin() / std::f32::consts::PI;
    Vec2::new(u, v.clamp(0.0, 1.0))
}

/// Differential spin for a deck, if the shell table carries it.
pub(crate) fn deck_spin_rad_s(body: &str, deck: u8) -> Option<f32> {
    CLOUD_DECKS
        .iter()
        .find(|(b, d, _, _)| *b == body && *d == deck)
        .map(|(_, _, _, spin)| *spin)
}

/// Deck world rotation shared with the follower: the deck co-rotates with
/// the planet plus a zonal breeze about the planet's own pole axis (sphere
/// poles sit on local +Z, so latitude lines are the drift direction —
/// trade winds, not a world-Y tumble). When animation is off the deck is
/// planet-fixed. Phase folds in f64 so millennia of warp never jitter f32.
pub(crate) fn deck_world_rotation(
    planet_rotation: Quat,
    spin_rad_s: f32,
    sim_seconds: f64,
    animate: bool,
) -> Quat {
    if !animate {
        return planet_rotation;
    }
    let phase = ((sim_seconds * spin_rad_s as f64) % std::f64::consts::TAU) as f32;
    planet_rotation * Quat::from_rotation_z(phase)
}

/// Body frame -> shell-local bake frame for shadow sampling: the shader
/// applies this to the parallax-corrected body direction. Static decks are
/// the identity (same frame as the planet).
pub(crate) fn shadow_relative_rotation(
    planet_rotation: Quat,
    spin_rad_s: f32,
    sim_seconds: f64,
    animate: bool,
) -> Quat {
    deck_world_rotation(planet_rotation, spin_rad_s, sim_seconds, animate).inverse()
        * planet_rotation
}

/// Sun direction in the planet-fixed frame for body-space shadow math.
pub(crate) fn shadow_sun_body(planet_rotation: Quat, sun_world_dir: Vec3) -> Vec3 {
    planet_rotation.inverse() * sun_world_dir.normalize_or_zero()
}

/// Parallax-corrected sample direction: the deck point whose shadow falls on
/// `ground_dir` (unit) given unit `sun_dir` (toward the sun). Walk from the
/// ground toward the sun to the deck plane: overhead sun samples straight
/// down, low sun reaches sunward (the cloud east of you shadows you at
/// sunset, not the one overhead).
pub(crate) fn shadow_sample_dir(
    ground_dir: Vec3,
    sun_dir: Vec3,
    radius_m: f32,
    deck_height_m: f32,
) -> Vec3 {
    let up = ground_dir.normalize_or_zero();
    let sun = sun_dir.normalize_or_zero();
    let cos_z = up.dot(sun).max(0.15);
    (up * radius_m + sun * (deck_height_m / cos_z)).normalize_or_zero()
}

/// Nearest cloud alpha (0 = clear, 1 = overcast core) from a bake made by
/// [`bake_clouds`]: row-major RGBA8, v = 0 at the first row.
pub(crate) fn cloud_alpha_at(data: &[u8], w: u32, h: u32, uv: Vec2) -> f32 {
    if w == 0 || h == 0 || data.len() < (w * h * 4) as usize {
        return 0.0;
    }
    let x = ((uv.x.fract() + 1.0).fract() * w as f32).floor() as u32 % w;
    let y = ((uv.y.clamp(0.0, 1.0) * h as f32).floor() as u32).min(h - 1);
    data[((y * w + x) * 4 + 3) as usize] as f32 / 255.0
}

/// Shadow factor from deck alpha: clear sky stays 1.0, overcast cores dim
/// toward `1 - CLOUD_SHADOW_STRENGTH`. Skirts fade smoothly, never a hard
/// blob edge.
pub(crate) fn shadow_factor_from_alpha(alpha: f32) -> f32 {
    let t = ((alpha - 0.05) / 0.55).clamp(0.0, 1.0);
    let s = t * t * (3.0 - 2.0 * t);
    1.0 - CLOUD_SHADOW_STRENGTH * s
}

// ---------------------------------------------------------------------------
// Shell relief: radial domes from deck coverage.

/// Radial relief for a shell mesh from a deck coverage grid: dense cores
/// become domes, thin skirts stay low, so decks read as puffy volumes
/// with true silhouettes and parallax instead of painted spheres. Heights
/// sample the same bake the shell shows (u wraps the seam, so the two
/// UV-seam vertex columns displace identically). `relief` is in mesh units
/// (the shells are unit spheres!); domes stay gentle, so the sphere's
/// smooth normals remain valid without recomputation.
pub(crate) fn displace_shell_by_coverage(
    mesh: &mut Mesh,
    alpha: &[u8],
    w: u32,
    h: u32,
    relief_m: f32,
) {
    use bevy::mesh::VertexAttributeValues;
    assert_eq!(alpha.len(), (w * h) as usize);
    let height_at = |u: f32, v: f32| {
        let uu = (u.fract() + 1.0).fract() * w as f32 - 0.5;
        let vv = (v.clamp(0.0, 1.0) * h as f32 - 0.5).clamp(0.0, h as f32 - 1.0);
        let x0 = uu.floor().max(0.0) as u32;
        let y0 = vv.floor().max(0.0) as u32;
        let x1 = (x0 + 1).min(w - 1);
        let y1 = (y0 + 1).min(h - 1);
        let fx = (uu - x0 as f32).clamp(0.0, 1.0);
        let fy = (vv - y0 as f32).clamp(0.0, 1.0);
        let s = |x: u32, y: u32| alpha[(y * w + x) as usize] as f32 / 255.0;
        let top = s(x0, y0) * (1.0 - fx) + s(x1, y0) * fx;
        let bot = s(x0, y1) * (1.0 - fx) + s(x1, y1) * fx;
        top * (1.0 - fy) + bot * fy
    };
    if mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_none() {
        return;
    }
    let uv: Vec<[f32; 2]> = match mesh.attribute(Mesh::ATTRIBUTE_UV_0) {
        Some(VertexAttributeValues::Float32x2(uv)) => uv.clone(),
        _ => return,
    };
    let lifts: Vec<f32> = uv
        .iter()
        .map(|t| {
            let h = height_at(t[0], t[1]);
            // Smoothstepped lift: only real cores dome up, skirts stay put.
            h * h * (3.0 - 2.0 * h) * relief_m
        })
        .collect();
    if let Some(VertexAttributeValues::Float32x3(out)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for (p, lift) in out.iter_mut().zip(lifts.iter()) {
            let dir = Vec3::new(p[0], p[1], p[2]).normalize_or_zero();
            p[0] += dir.x * lift;
            p[1] += dir.y * lift;
            p[2] += dir.z * lift;
        }
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Aurora.

/// Aurora curtain bake over lat/lon. Intensity = oval gaussian x altitude
/// colour mix x curtain octaves x field-aligned rays; caller scales by
/// `intensity` setting. The rays (fine longitude striations, sharpened and
/// irregular) are what reads as curtains instead of a painted band: from
/// orbit or the ground the eye keys on ray structure, not the smooth oval.
fn bake_aurora(
    w: u32,
    h: u32,
    oval_lat_deg: f64,
    oval_width_deg: f64,
    sim_time_s: f64,
    animate: bool,
) -> Vec<u8> {
    let oval = oval_lat_deg.to_radians();
    let width = oval_width_deg.to_radians().max(1e-3);
    let t = if animate { sim_time_s } else { 0.0 };
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    for y in 0..h {
        let lat = (y as f64 + 0.5) / h as f64 * std::f64::consts::PI - std::f64::consts::FRAC_PI_2;
        for x in 0..w {
            let lon = (x as f64 + 0.5) / w as f64 * std::f64::consts::TAU;
            let d = (lat.abs() - oval) / width;
            let band = (-d * d * 0.5).exp();
            let curtain =
                0.6 + 0.25 * (3.0 * lon + t * 0.05).sin() + 0.15 * (7.0 * lon - t * 0.11).sin();
            // Curtain rays: two incommensurate longitude combs, sharpened so
            // most of the oval stays dark lanes between bright rays. The slow
            // phase rides sim time (frozen when not animating: deterministic).
            let comb = 0.5 + 0.5 * (53.0 * lon + t * 0.23).sin();
            let comb2 = 0.5 + 0.5 * (31.0 * lon + 1.7 - t * 0.14).sin();
            let rays = 0.30 + 0.70 * (comb.powf(1.6) * 0.65 + comb2.powf(1.6) * 0.35);
            let k = (band * curtain.max(0.0) * rays) as f32;
            // Green 557.7nm bottom -> red/violet top; bake mixes by |lat| offset.
            let top = ((lat.abs() - oval) / width * 0.5 + 0.5).clamp(0.0, 1.0) as f32;
            let r = (0.20 * (1.0 - top) + 0.90 * top) * k;
            let g = (1.00 * (1.0 - top) + 0.25 * top) * k;
            let b = (0.35 * (1.0 - top) + 0.30 * top) * k;
            px.extend_from_slice(&[
                (r.clamp(0.0, 1.0) * 255.0) as u8,
                (g.clamp(0.0, 1.0) * 255.0) as u8,
                (b.clamp(0.0, 1.0) * 255.0) as u8,
                (k.clamp(0.0, 1.0) * 255.0) as u8,
            ]);
        }
    }
    px
}

// ---------------------------------------------------------------------------
// Bevy resources / entities.

/// Collapse UV-sphere pole columns: pole-ring vertices share one position
/// but span all of u, smearing a full texture row over each pole triangle.
/// Sampling the middle column instead shrinks the wedge to one texel.
/// Bevy puts the UV poles on local +Z; v (latitude) is left alone.
fn collapse_shell_pole_uvs(mesh: &mut Mesh) {
    use bevy::mesh::VertexAttributeValues;
    let poles: Vec<usize> = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(pos)) => pos
            .iter()
            .enumerate()
            .filter(|(_, p)| p[2].abs() >= 1.0 - 1e-6)
            .map(|(i, _)| i)
            .collect(),
        _ => return,
    };
    if poles.is_empty() {
        return;
    }
    if let Some(VertexAttributeValues::Float32x2(uv)) = mesh.attribute_mut(Mesh::ATTRIBUTE_UV_0) {
        for i in poles {
            if let Some(tex) = uv.get_mut(i) {
                tex[0] = 0.5;
            }
        }
    }
}

fn rgba_image(w: u32, h: u32, pixels: Vec<u8>) -> Image {
    Image::new(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        pixels,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}

#[derive(Resource)]
pub(crate) struct BeautyImages {
    nereid: Handle<Image>,
    vesper: Handle<Image>,
    nereid_size: (u32, u32),
    vesper_size: (u32, u32),
    pub(crate) cloud_low: Handle<Image>,
    cloud_high: Option<Handle<Image>>,
    aurora: Handle<Image>,
    aurora_size: (u32, u32),
}

#[cfg(test)]
impl BeautyImages {
    pub(crate) fn test_with_clouds(cloud_low: Handle<Image>) -> Self {
        Self {
            nereid: Handle::default(),
            vesper: Handle::default(),
            nereid_size: (0, 0),
            vesper_size: (0, 0),
            cloud_low,
            cloud_high: None,
            aurora: Handle::default(),
            aurora_size: (0, 0),
        }
    }
}

#[derive(Component)]
pub(crate) struct CloudShell {
    body: &'static str,
    deck: u8,
    spin_rad_s: f32,
    scale_factor: f32,
}

#[derive(Component)]
pub(crate) struct AuroraShell {
    body: &'static str,
    scale_factor: f32,
    /// Curtain layer weight: the stack shares one bake and rotation so rays
    /// stay field-aligned coherent while parallax separates the layers.
    gain: f32,
}

#[derive(Resource, Default)]
struct BeautyTimers {
    storm_rebake_s: f64,
    aurora_rebake_s: f64,
}

pub struct BeautyPlugin;

impl Plugin for BeautyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<CloudVolumeMaterial>::default())
            .init_resource::<BeautyTimers>()
            .add_systems(PostStartup, setup_beauty)
            .add_systems(
                Update,
                (
                    swap_gas_giant_materials,
                    follow_cloud_shells,
                    follow_cloud_volume,
                    follow_aurora_shell,
                    rebake_animated,
                ),
            );
    }
}

fn setup_beauty(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut volume_materials: ResMut<Assets<CloudVolumeMaterial>>,
    graphics: Option<Res<GraphicsResolved>>,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    info!(
        "[beauty] resolved: gas={} clouds={}(vol={} steps={} layers={} cov={:.2} op={:.2} anim={}) aurora={}(q={:?} i={:.2} anim={})",
        r.gas_giant_enabled,
        r.clouds_enabled,
        r.clouds_volumetric,
        r.clouds_steps,
        r.clouds_layers,
        r.clouds_coverage,
        r.clouds_opacity,
        r.clouds_animate,
        r.aurora_shell,
        r.aurora_quality,
        r.aurora_intensity,
        r.aurora_animate,
    );

    // --- Gas giants: bake band textures now, swap onto planet entities later
    // (swap system runs every frame until both swaps land, so order with the
    // map setup does not matter).
    let (nw, nh) = gas_giant_size(r.gas_giant_quality, true);
    let (vw, vh) = gas_giant_size(r.gas_giant_quality, false);
    let (cw, ch) = cloud_size(r.clouds_quality);
    // Impostor bakes share the terrain CBT address space: the chosen widths
    // must round-trip through a TileKey <-> Node mapping.
    for w in [nw, vw, cw] {
        let level = cbt_level_for_width(w);
        let key = TileKey {
            face: 0,
            level,
            x: 0,
            y: 0,
        };
        debug_assert_eq!(
            cbt_node_for_tile(key).and_then(tile_for_cbt_node),
            Some(key)
        );
    }
    let nereid_px = bake_gas_giant(NEREID_STYLE, nw, nh, 0.0, r.gas_giant_limb);
    let vesper_px = bake_gas_giant(VESPER_STYLE, vw, vh, 0.0, r.gas_giant_limb);
    let nereid = images.add(rgba_image(nw, nh, nereid_px));
    let vesper = images.add(rgba_image(vw, vh, vesper_px));

    // --- Clouds.
    let low_px = bake_clouds(
        0xC10D,
        cw,
        ch,
        r.clouds_coverage,
        r.clouds_opacity,
        (1.0, 1.0),
    );
    // Shell meshes: one unit sphere per deck so relief displacement follows
    // each deck's own coverage (a shared mesh would dome all decks alike).
    // Bevy's UV sphere puts its texture poles on local +Z (see
    // SPHERE_POLE_TO_WORLD_UP in main): every pole-ring vertex shares the pole
    // position but spans the full u range, so each pole triangle smears an
    // entire texture row across the sky. From inside the shell (any low
    // flight) that reads as giant white wedges with hard triangle edges
    // (measured on a 30 km maneuver frame). Collapsing pole u to the middle
    // column reduces the wedge to a single texel.
    let mut make_shell = |alpha: Option<(Vec<u8>, u32, u32, f32)>| {
        let mut sphere = Sphere::new(1.0).mesh().uv(96, 48);
        if let Some((grid, w, h, relief)) = alpha {
            displace_shell_by_coverage(&mut sphere, &grid, w, h, relief);
        }
        collapse_shell_pole_uvs(&mut sphere);
        meshes.add(sphere)
    };
    let low_mesh = make_shell(Some((alpha_grid(&low_px), cw, ch, CLOUD_RELIEF_LOW_FRAC)));
    let cloud_low = images.add(rgba_image(cw, ch, low_px));
    let (cloud_high, high_mesh) = if r.clouds_layers >= 2 {
        // Cirrus stays a thin streaky veil over the low deck: coverage well
        // down the shaping curve (measured) plus vertical tightening for
        // streaks instead of puffs. Kept faint so it never dulls the deck.
        let hi_px = bake_clouds(
            0xC1225,
            cw / 2,
            ch / 2,
            (r.clouds_coverage + 0.12).min(0.95),
            r.clouds_opacity * 0.5,
            (1.0, 2.0),
        );
        let hi_mesh = make_shell(Some((
            alpha_grid(&hi_px),
            cw / 2,
            ch / 2,
            CLOUD_RELIEF_HIGH_FRAC,
        )));
        (
            Some(images.add(rgba_image(cw / 2, ch / 2, hi_px))),
            Some(hi_mesh),
        )
    } else {
        (None, None)
    };
    let plain_mesh = make_shell(None);

    // --- Aurora curtain base.
    let (aw, ah) = match r.aurora_quality {
        thessa_graphics::AuroraQuality::Low => (256, 128),
        thessa_graphics::AuroraQuality::High => (512, 256),
    };
    let aurora_px = bake_aurora(aw, ah, 67.0, 3.0, 0.0, false);
    let aurora = images.add(rgba_image(aw, ah, aurora_px));

    commands.insert_resource(BeautyImages {
        nereid,
        vesper,
        nereid_size: (nw, nh),
        vesper_size: (vw, vh),
        cloud_low: cloud_low.clone(),
        cloud_high: cloud_high.clone(),
        aurora,
        aurora_size: (aw, ah),
    });

    // Cloud shells for the three ocean worlds (map + pilot follow logic
    // below copies planet transforms; no hierarchy parenting). Placement
    // comes from CLOUD_DECKS so shadow samplers share the exact table.
    for (body, deck, scale, spin) in CLOUD_DECKS.iter().copied().filter(|(body, deck, _, _)| {
        *deck == 0 || (*body == "thessa" && (*deck as u32) < r.clouds_layers)
    }) {
        // Relief follows the deck's own bake (deck 1 falls back to the low
        // mesh when cirrus is off, but then it never spawns either).
        let mesh = if deck == 0 {
            low_mesh.clone()
        } else {
            high_mesh.clone().unwrap_or_else(|| low_mesh.clone())
        };
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(materials.add(StandardMaterial::default())),
            Transform::default(),
            Visibility::Hidden,
            Name::new(format!("beauty cloud {body} deck{deck}")),
            CloudShell {
                body,
                deck,
                spin_rad_s: spin,
                scale_factor: scale,
            },
        ));
    }

    // Volume decks share meshes and bakes with the shells above (same
    // coverage by construction); one material per deck carries its slab and
    // texture. Low cumulus deck marches volumetrically; cirrus stays a thin shell
    // in both paths (see docs/14_VISUAL_ATMOSPHERE.md).
    if r.clouds_enabled {
        let volume_mesh = meshes.add(Sphere::new(1.0).mesh().uv(96, 48));
        for (body, deck, _, spin) in CLOUD_DECKS.iter().copied().filter(|(_, deck, _, _)| *deck == 0) {
            let tex = cloud_low.clone();
            let (bottom, top, ext) = volume_slab_for_deck(deck);
            commands.spawn((
                Mesh3d(volume_mesh.clone()),
                MeshMaterial3d(volume_materials.add(CloudVolumeMaterial {
                    uniforms: CloudUniforms {
                        center_radius: Vec4::ZERO,
                        slab: Vec4::new(bottom, top, ext, volume_steps(r.clouds_steps)),
                        sun: Vec4::Y,
                        deck_to_local: Vec4::W,
                        params: Vec4::new(
                            r.clouds_opacity,
                            0.0,
                            CLOUD_SUN_SPAN_FRAC,
                            CLOUD_DETAIL_AMP,
                        ),
                        shape: Vec4::new(
                            deck as f32,
                            CLOUD_COVERAGE_FLOOR,
                            volume_mesh_scale_for_deck(deck),
                            cloud_backend_mode(&r),
                        ),
                    },
                    coverage: tex,
                })),
                Transform::default(),
                Visibility::Hidden,
                Name::new(format!("beauty cloud volume {body} deck{deck}")),
                CloudVolume {
                    body,
                    deck,
                    spin_rad_s: spin,
                },
            ));
        }
    }

    // Aurora shell (Thessa only).
    // Aurora curtain stack (thessa only): three concentric shells sharing
    // one bake and rotation keep rays field-aligned coherent, while the
    // radial separation gives curtains parallax depth and a soft vertical
    // falloff instead of a painted band. Gains sum near the main shell.
    for (scale, gain) in [(1.038, 0.45), (1.045, 1.0), (1.052, 0.5)] {
        commands.spawn((
            Mesh3d(plain_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial::default())),
            Transform::default(),
            Visibility::Hidden,
            Name::new("beauty aurora thessa"),
            AuroraShell {
                body: "thessa",
                scale_factor: scale,
                gain,
            },
        ));
    }
}

/// Swap procedural band textures onto the map planet entities once they exist.
fn swap_gas_giant_materials(
    mut done: Local<[bool; 2]>,
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    visuals: Query<(&Name, &MeshMaterial3d<StandardMaterial>)>,
) {
    if done[0] && done[1] {
        return;
    }
    let Some(imgs) = imgs.as_deref() else { return };
    let enabled = graphics
        .as_deref()
        .map(|g| g.0.gas_giant_enabled)
        .unwrap_or(true);
    if !enabled {
        done[0] = true;
        done[1] = true;
        return;
    }
    for (name, mat3d) in &visuals {
        let slot = match name.as_str() {
            "nereid" => Some((0, &imgs.nereid)),
            "vesper" => Some((1, &imgs.vesper)),
            _ => None,
        };
        let Some((idx, tex)) = slot else { continue };
        if done[idx] {
            continue;
        }
        if let Some(mut mat) = materials.get_mut(&mat3d.0) {
            mat.base_color = Color::WHITE;
            mat.base_color_texture = Some(tex.clone());
            mat.perceptual_roughness = 0.85;
            mat.metallic = 0.0;
            done[idx] = true;
        }
    }
}

/// Find a planet visual transform by body name (map view) or PFD name (pilot).
/// Uses computed [`InheritedVisibility`]: the PFD planet carries no explicit
/// `Visibility` (it inherits the preview scene's), so an explicit-visibility
/// query never matches it and pilot flight silently loses every follower.
#[allow(clippy::type_complexity)]
fn planet_transform(
    body: &str,
    visuals: &Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<CloudShell>,
            Without<AuroraShell>,
            Without<CloudVolume>,
        ),
    >,
) -> Option<(Transform, bool)> {
    // Map view entity. Prefer a VISIBLE match: in pilot mode the map visual
    // still exists but is hidden while the PFD planet shows the same world.
    let mut fallback: Option<(Transform, bool)> = None;
    for (name, t, v) in visuals {
        if name.as_str() == body {
            let entry = (*t, v.get());
            if entry.1 {
                return Some(entry);
            }
            fallback = fallback.or(Some(entry));
        }
    }
    // Pilot preview planet (Thessa only).
    if body == "thessa" {
        for (name, t, v) in visuals {
            if name.as_str().starts_with("PFD Thessa planet") {
                let entry = (*t, v.get());
                if entry.1 {
                    return Some(entry);
                }
                fallback = fallback.or(Some(entry));
            }
        }
    }
    fallback
}

fn cloud_material(
    materials: &mut Assets<StandardMaterial>,
    handle: &Handle<StandardMaterial>,
    tex: Handle<Image>,
    opacity: f32,
) {
    if let Some(mut mat) = materials.get_mut(handle) {
        if mat.base_color_texture.is_none() {
            mat.base_color = Color::WHITE;
            mat.base_color_texture = Some(tex);
            mat.alpha_mode = AlphaMode::Blend;
            // Back-face cull: the deck is only ever viewed from outside
            // (the impostor fade hides inside views), so the far hemisphere
            // must not double the alpha. No-cull rendered every ray twice
            // and washed the disc to cream from orbit.
            //
            // NOTE: do not add a normal_map_texture here without solving
            // the night streaks first: tangent-space relief on these
            // shells rendered brown/grey smears across the night sky
            // (screenshot-proven, mechanism unclear - suspect tangent/UV
            // interaction on the collapsed-pole sphere).
            mat.cull_mode = Some(Face::Back);
            mat.perceptual_roughness = 1.0;
            mat.metallic = 0.0;
        }
        // Per-frame: the orbit-impostor fade below drives this, not setup.
        mat.base_color.set_alpha(opacity);
    }
}

/// Orbit-impostor fade for whole-sphere shells (~20 km texels at medium).
/// They only read as clouds from WELL above the deck. Near, inside, or just
/// above it (low flight, climb corridor, deck-skimming orbit) they are giant
/// soap blobs — or, at deck altitude, a full-screen mint veil (measured at
/// 60 km inside the cirrus layer). So the deck stays hidden until the camera
/// clears twice its height and fades in by four times. Orbit and map views
/// keep the full deck; low flight keeps a crisp sky.
fn cloud_shell_fade(cam_frac: f32, deck_frac: f32) -> f32 {
    if !cam_frac.is_finite() || deck_frac <= 0.0 {
        return 1.0;
    }
    let t = ((cam_frac / deck_frac - 2.0) / 2.0).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

// ---------------------------------------------------------------------------
// Cloud volume march (volumetric path; shells stay the fallback).
//
// One [`CloudVolumeMaterial`] per volume deck: the unit-sphere mesh is
// coverage only, every pixel marches the planet-normalized slab in
// `assets/shaders/cloud_volume.wgsl` (Beer-Lambert over the deck bake the
// shells show, so both paths agree on coverage). Sun self-shadowing,
// forward-lobe silver lining, and eclipse-aware day factor run in-shader;
// per-frame uniforms carry planet frame, sun, and march budget.

/// Packed uniforms mirroring `assets/shaders/cloud_volume.wgsl`.
#[derive(Debug, Clone, Copy, ShaderType)]
pub struct CloudUniforms {
    center_radius: Vec4,
    slab: Vec4,
    sun: Vec4,
    deck_to_local: Vec4,
    params: Vec4,
    shape: Vec4,
}

#[derive(Debug, Clone, Asset, TypePath, AsBindGroup)]
pub struct CloudVolumeMaterial {
    #[uniform(0)]
    pub uniforms: CloudUniforms,
    #[texture(1)]
    #[sampler(2)]
    pub coverage: Handle<Image>,
}

impl Material for CloudVolumeMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cloud_volume.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Premultiplied
    }

    fn specialize(
        _pipeline: &MaterialPipeline,
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayoutRef,
        _key: MaterialPipelineKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        // No culling: the camera flies through the slab (surface views look
        // up from below it, climb corridors cross it). The march itself is
        // front-interval based, so far-hemisphere fragments cost one slab
        // march each instead of corrupting the image.
        descriptor.primitive.cull_mode = None;
        Ok(())
    }
}

#[derive(Component)]
pub(crate) struct CloudVolume {
    body: &'static str,
    deck: u8,
    spin_rad_s: f32,
}

/// Volume path active when the effect is enabled and resolved to march
/// (single source of truth shared by setup/followers; shells hide then).
pub(crate) fn use_cloud_volume(r: &ResolvedGraphicsSettings) -> bool {
    r.clouds_enabled && r.clouds_volumetric
}

/// Slab (bottom/top fracs) and extinction for a deck index: 0 marches the
/// low deck, 1 the thin cirrus veil.
pub(crate) fn volume_slab_for_deck(deck: u8) -> (f32, f32, f32) {
    if deck == 0 {
        (CLOUD_SLAB_LOW.0, CLOUD_SLAB_LOW.1, CLOUD_EXT_LOW)
    } else {
        (CLOUD_SLAB_HIGH.0, CLOUD_SLAB_HIGH.1, CLOUD_EXT_HIGH)
    }
}

/// Impostor mesh scale for a volume deck: must cover the slab top plus the
/// per-pixel top variation (0.6..1.4 thicknesses), or rasterization clips
/// tower tops into a flat ring. The shell fallback keeps riding mid-slab.
pub(crate) fn volume_mesh_scale_for_deck(deck: u8) -> f32 {
    let (bottom, top, _) = volume_slab_for_deck(deck);
    top + (top - bottom) * 0.5 + 1e-4
}

/// March steps actually issued: user budget clamped to the shader loop
/// bound (`CLOUD_MARCH_MAX` in WGSL, capped with a note in resolve).
pub(crate) fn volume_steps(steps: u32) -> f32 {
    steps.clamp(4, 24) as f32
}

/// Effective runtime cloud backend for rendering and telemetry:
/// - `March`: Real-time stratified ray marching (Backend A, tuned for Radeon 880M / iGPU)
/// - `Stochastic`: Stochastic volumetric scattering approximation (Backend B, candidate real-time integrator)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EffectiveCloudBackend {
    March,
    Stochastic,
}

impl EffectiveCloudBackend {
    pub(crate) fn as_f32(self) -> f32 {
        match self {
            Self::March => 0.0,
            Self::Stochastic => 1.0,
        }
    }

    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::March => "march",
            Self::Stochastic => "stochastic",
        }
    }
}

/// Pure resolution of effective cloud backend from requested integrator and optional dev environment override.
///
/// CRITICAL: `CloudIntegratorRequest::March` strictly resolves to `March`, completely decoupled
/// from `rt_clouds` or other global RT participation settings.
pub(crate) fn resolve_effective_cloud_backend(
    integrator: thessa_graphics::CloudIntegratorRequest,
    env_override: Option<&str>,
) -> EffectiveCloudBackend {
    if let Some(val) = env_override {
        if val.eq_ignore_ascii_case("pt")
            || val == "1"
            || val.eq_ignore_ascii_case("pathtrace")
            || val.eq_ignore_ascii_case("stochastic")
        {
            return EffectiveCloudBackend::Stochastic;
        } else if val.eq_ignore_ascii_case("march")
            || val == "0"
            || val.eq_ignore_ascii_case("raymarch")
        {
            return EffectiveCloudBackend::March;
        }
    }
    match integrator {
        thessa_graphics::CloudIntegratorRequest::Stochastic => EffectiveCloudBackend::Stochastic,
        thessa_graphics::CloudIntegratorRequest::March => EffectiveCloudBackend::March,
    }
}

/// Cloud backend selection encoded in `shape.w`:
/// 0.0 = Real-time stratified ray marching (Backend A)
/// 1.0 = Stochastic scattering approximation (Backend B)
pub(crate) fn effective_cloud_backend(r: &ResolvedGraphicsSettings) -> EffectiveCloudBackend {
    static BACKEND_OVERRIDE: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
    let override_val = BACKEND_OVERRIDE.get_or_init(|| std::env::var("THESSA_CLOUD_BACKEND").ok());
    resolve_effective_cloud_backend(r.cloud_integrator, override_val.as_deref())
}

pub(crate) fn cloud_backend_mode(r: &ResolvedGraphicsSettings) -> f32 {
    effective_cloud_backend(r).as_f32()
}

/// Equatorial breeze advecting the shader detail: ~15 m/s over a ~20 000 km
/// circumference. The old 0.004 UV/s scrolled noise at 80 km/s — the crawl
/// visible frame to frame.
pub(crate) const CLOUD_WIND_UV_PER_S: f64 = 7.5e-7;

/// Detail-drift clock as a pre-wrapped UV phase: computed in f64, seamless
/// across the wrap (integer UV cycles, no popping), f32-safe at any warp.
/// Deterministic in sim time, therefore warp-safe. Frozen when not animating.
pub(crate) fn wind_phase_uv(sim_seconds: f64, animate: bool) -> f32 {
    if !animate {
        return 0.0;
    }
    (sim_seconds * CLOUD_WIND_UV_PER_S % 1.0) as f32
}

/// Live-terrain fallback for the flown world: when the map/PFD entity match
/// fails (or is hidden mid-transition), place the follower at the true-scale
/// planet the terrain streamer is actually rendering. Uses authoritative
/// terrain state (render center + field radius + sim-time spin), never a
/// debug scene. Only the flown world (`thessa`) has a render center; other
/// bodies keep the entity path (in pilot view they stay hidden with the map).
fn live_planet_transform(
    body: &str,
    world: &terrain::WorldTerrain,
    sim_seconds: f64,
) -> Option<Transform> {
    if body != "thessa" {
        return None;
    }
    let center = world.render_center?;
    let radius = world.field.params.radius_m as f32;
    if radius <= 0.0 {
        return None;
    }
    let spin =
        Quat::from_rotation_y((sim_seconds * std::f64::consts::TAU / (80.0 * 3600.0)) as f32)
            * SPHERE_POLE_TO_WORLD_UP;
    Some(
        Transform::from_translation(center)
            .with_scale(Vec3::splat(radius))
            .with_rotation(spin),
    )
}

/// Planet frame shared by shell followers and effect samplers (e.g. cloud
/// shadows): the visible entity match wins, otherwise the live terrain
/// streamer owns the flown world's placement. Single source of truth so
/// shadows track the shells they shade.
#[allow(clippy::type_complexity)]
pub(crate) fn planet_frame_for_body(
    body: &str,
    visuals: &Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<CloudShell>,
            Without<AuroraShell>,
            Without<CloudVolume>,
        ),
    >,
    terrain: Option<&terrain::WorldTerrain>,
    sim_seconds: f64,
) -> Option<Transform> {
    // Entity match is only trusted when visible; otherwise the live terrain
    // streamer owns the flown world's placement (pilot/survey).
    let (planet, planet_visible) = planet_transform(body, visuals)?;
    if planet_visible {
        return Some(planet);
    }
    terrain.and_then(|w| live_planet_transform(body, w, sim_seconds))
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn follow_cloud_shells(
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    terrain: Option<Res<terrain::WorldTerrain>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    visuals: Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<CloudShell>,
            Without<AuroraShell>,
            Without<CloudVolume>,
        ),
    >,
    cameras: Query<&Transform, (With<Camera3d>, Without<CloudShell>, Without<AuroraShell>)>,
    mut shells: Query<(
        &CloudShell,
        &mut Transform,
        &mut Visibility,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut dbg: Local<u32>,
) {
    let Some(imgs) = imgs.as_deref() else { return };
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let t = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
    let log_now = *dbg < 3;
    *dbg += 1;
    for (shell, mut st, mut vis, mat3d) in &mut shells {
        let tex = if shell.deck == 0 {
            Some(imgs.cloud_low.clone())
        } else {
            imgs.cloud_high.clone()
        };
        let Some(tex) = tex else {
            *vis = Visibility::Hidden;
            continue;
        };
        if !r.clouds_enabled {
            *vis = Visibility::Hidden;
            continue;
        }
        // The volume path owns the low cumulus deck (deck 0) when resolved:
        // that shell stays hidden so both representations never double-draw.
        // Cirrus (deck 1) stays a thin shell in both paths (docs/14_VISUAL_ATMOSPHERE.md).
        if shell.deck == 0 && use_cloud_volume(&r) {
            *vis = Visibility::Hidden;
            continue;
        }
        let Some(planet) = planet_frame_for_body(shell.body, &visuals, terrain.as_deref(), t)
        else {
            if log_now {
                info!("[beauty] cloud {}: no planet frame", shell.body);
            }
            *vis = Visibility::Hidden;
            continue;
        };
        if log_now {
            info!(
                "[beauty] cloud {}: following planet scale={:.3}",
                shell.body, planet.scale.x
            );
        }
        // Impostor fade: no giant soap blobs over low flight. The camera
        // altitude is measured in followed-planet radii, so map exaggeration
        // and true-scale pilot views share one rule.
        let cam_frac = cameras
            .single()
            .map(|c| (c.translation - planet.translation).length() / planet.scale.x.max(1e-6) - 1.0)
            .unwrap_or(f32::INFINITY);
        let fade = cloud_shell_fade(cam_frac, shell.scale_factor - 1.0);
        if fade <= 0.01 {
            *vis = Visibility::Hidden;
            continue;
        }
        cloud_material(&mut materials, &mat3d.0, tex, r.clouds_opacity * fade);
        st.translation = planet.translation;
        let s = planet.scale.x * shell.scale_factor;
        st.scale = Vec3::splat(s);
        // Differential drift: own slow spin on top of the planet rotation.
        st.rotation = deck_world_rotation(planet.rotation, shell.spin_rad_s, t, r.clouds_animate);
        *vis = Visibility::Visible;
    }
}

/// March-path follower: the same planet frame as the shells (visible entity
/// match, else the live terrain streamer), no impostor fade — the march is
/// valid from the surface (looking up), inside the slab, and from orbit.
/// Disabled or shell-resolved configurations hide every volume deck, so the
/// effect costs nothing when off.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn follow_cloud_volume(
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    terrain: Option<Res<terrain::WorldTerrain>>,
    eclipse: Option<Res<crate::atmosphere::EclipseFactor>>,
    sun: Query<&GlobalTransform, With<PrimaryStarLight>>,
    visuals: Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<CloudShell>,
            Without<AuroraShell>,
            Without<CloudVolume>,
        ),
    >,
    mut volume_materials: ResMut<Assets<CloudVolumeMaterial>>,
    mut volumes: Query<(
        &CloudVolume,
        &mut Transform,
        &mut Visibility,
        &MeshMaterial3d<CloudVolumeMaterial>,
    )>,
    mut dbg: Local<u32>,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let t = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
    let log_now = *dbg < 3;
    *dbg += 1;
    let active = use_cloud_volume(&r);
    let sun_dir = sun
        .single()
        .map(|tf| {
            let d = tf.back();
            Vec3::new(d.x, d.y, d.z)
        })
        .unwrap_or(Vec3::Y);
    let sun_vis = eclipse.as_deref().map(|e| e.0).unwrap_or(1.0);
    for (deck, mut st, mut vis, mat3d) in &mut volumes {
        if !active {
            *vis = Visibility::Hidden;
            continue;
        }
        let Some(planet) = planet_frame_for_body(deck.body, &visuals, terrain.as_deref(), t) else {
            if log_now {
                info!("[beauty] cloud volume {}: no planet frame", deck.body);
            }
            *vis = Visibility::Hidden;
            continue;
        };
        if planet.scale.x <= 1e-6 {
            *vis = Visibility::Hidden;
            continue;
        }
        if log_now {
            info!(
                "[beauty] cloud volume {}: following planet scale={:.3}",
                deck.body, planet.scale.x
            );
        }
        st.translation = planet.translation;
        // Impostor mesh covers the slab top (not the mid-slab shell scale),
        // so rasterization never clips tower tops; the march itself is analytic.
        let mesh_scale = volume_mesh_scale_for_deck(deck.deck);
        st.scale = Vec3::splat(planet.scale.x * mesh_scale);
        let deck_world = deck_world_rotation(planet.rotation, deck.spin_rad_s, t, r.clouds_animate);
        st.rotation = deck_world;
        let (bottom, top, ext) = volume_slab_for_deck(deck.deck);
        if let Some(mut mat) = volume_materials.get_mut(&mat3d.0) {
            let to_local = deck_world.inverse();
            mat.uniforms.center_radius = Vec4::new(
                planet.translation.x,
                planet.translation.y,
                planet.translation.z,
                planet.scale.x,
            );
            mat.uniforms.slab = Vec4::new(bottom, top, ext, volume_steps(r.clouds_steps));
            mat.uniforms.sun = Vec4::new(sun_dir.x, sun_dir.y, sun_dir.z, sun_vis);
            mat.uniforms.deck_to_local = Vec4::new(to_local.x, to_local.y, to_local.z, to_local.w);
            mat.uniforms.params = Vec4::new(
                r.clouds_opacity,
                wind_phase_uv(t, r.clouds_animate),
                CLOUD_SUN_SPAN_FRAC,
                CLOUD_DETAIL_AMP,
            );
            mat.uniforms.shape = Vec4::new(
                deck.deck as f32,
                CLOUD_COVERAGE_FLOOR,
                volume_mesh_scale_for_deck(deck.deck),
                cloud_backend_mode(&r),
            );
        }
        *vis = Visibility::Visible;
    }
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn follow_aurora_shell(
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    terrain: Option<Res<terrain::WorldTerrain>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    visuals: Query<
        (&Name, &Transform, &InheritedVisibility),
        (
            Without<CloudShell>,
            Without<AuroraShell>,
            Without<CloudVolume>,
        ),
    >,
    mut shells: Query<(
        &AuroraShell,
        &mut Transform,
        &mut Visibility,
        &MeshMaterial3d<StandardMaterial>,
    )>,
    mut dbg: Local<u32>,
) {
    let Some(imgs) = imgs.as_deref() else { return };
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let log_now = *dbg < 3;
    *dbg += 1;
    for (shell, mut st, mut vis, mat3d) in &mut shells {
        if !r.aurora_shell {
            *vis = Visibility::Hidden;
            continue;
        }
        let t = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
        let planet = match planet_transform(shell.body, &visuals) {
            Some((planet, true)) => planet,
            _ => {
                if let Some(live) = terrain
                    .as_deref()
                    .and_then(|w| live_planet_transform(shell.body, w, t))
                {
                    live
                } else {
                    if log_now {
                        info!("[beauty] aurora {}: planet hidden", shell.body);
                    }
                    *vis = Visibility::Hidden;
                    continue;
                }
            }
        };
        if log_now {
            info!(
                "[beauty] aurora {}: following planet scale={:.3}",
                shell.body, planet.scale.x
            );
        }
        if let Some(mut mat) = materials.get_mut(&mat3d.0) {
            if mat.base_color_texture.is_none() {
                // Aurora is pure emission, but this pipeline's emissive slot
                // proved unreliable (Solari/deferred), so ALL glow goes
                // through base color + base texture with additive blending:
                // unlit white x texture adds light over the night side.
                // Base alpha stays 1 (no Blend-zero trap); the texture RGB
                // carries the oval shape.
                mat.base_color = Color::WHITE;
                mat.base_color_texture = Some(imgs.aurora.clone());
                mat.alpha_mode = AlphaMode::Add;
                mat.cull_mode = None;
                mat.unlit = true;
                mat.emissive = LinearRgba::BLACK;
                mat.emissive_texture = None;
            }
            let i = r.aurora_intensity * shell.gain;
            mat.base_color = Color::LinearRgba(LinearRgba::new(i, i, i, 1.0));
        }
        st.translation = planet.translation;
        st.scale = Vec3::splat(planet.scale.x * shell.scale_factor);
        st.rotation = planet.rotation;
        *vis = Visibility::Visible;
    }
}

/// Low-frequency re-bakes: storm drift (gas giants) and curtain slide (aurora).
fn rebake_animated(
    time: Res<Time>,
    mut timers: ResMut<BeautyTimers>,
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    mut images: ResMut<Assets<Image>>,
) {
    let (Some(imgs), Some(clock)) = (imgs.as_deref(), clock.as_deref()) else {
        return;
    };
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    timers.storm_rebake_s += time.delta_secs_f64();
    timers.aurora_rebake_s += time.delta_secs_f64();

    if r.gas_giant_enabled && r.gas_giant_animate && timers.storm_rebake_s > 10.0 {
        timers.storm_rebake_s = 0.0;
        let drift = clock.sim_seconds * 0.002;
        if let Some(mut img) = images.get_mut(&imgs.nereid) {
            img.data = Some(bake_gas_giant(
                NEREID_STYLE,
                imgs.nereid_size.0,
                imgs.nereid_size.1,
                drift,
                r.gas_giant_limb,
            ));
        }
        if let Some(mut img) = images.get_mut(&imgs.vesper) {
            img.data = Some(bake_gas_giant(
                VESPER_STYLE,
                imgs.vesper_size.0,
                imgs.vesper_size.1,
                drift * 0.7,
                r.gas_giant_limb,
            ));
        }
    }
    let aurora_fast =
        matches!(r.aurora_quality, thessa_graphics::AuroraQuality::High) && r.aurora_animate;
    if r.aurora_shell && aurora_fast && timers.aurora_rebake_s > 0.5 {
        timers.aurora_rebake_s = 0.0;
        if let Some(mut img) = images.get_mut(&imgs.aurora) {
            img.data = Some(bake_aurora(
                imgs.aurora_size.0,
                imgs.aurora_size.1,
                67.0,
                3.0,
                clock.sim_seconds,
                true,
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// Tests: determinism, ranges, CBT address sharing, plume input contract.

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::DVec3;

    #[test]
    fn noise_is_deterministic() {
        assert_eq!(fbm(7, 0.31, 0.77, 6.0), fbm(7, 0.31, 0.77, 6.0));
        assert_ne!(fbm(7, 0.31, 0.77, 6.0), fbm(8, 0.31, 0.77, 6.0));
        assert!((0.0..=1.0).contains(&fbm(7, 0.31, 0.77, 6.0)));
    }

    #[test]
    fn shell_mesh_uv_matches_sphere_mapping() {
        // +X equator (sector 0) is u = 0; going toward +Y wraps u up.
        let uv = shell_mesh_uv(Vec3::X);
        assert!(uv.x.abs() < 1e-6, "{uv:?}");
        assert!((uv.y - 0.5).abs() < 1e-6, "{uv:?}");
        let uv = shell_mesh_uv(Vec3::Y);
        assert!((uv.x - 0.25).abs() < 1e-6, "{uv:?}");
        // Poles (local +Z) sit at v = 0, antipode at v = 1.
        assert!(shell_mesh_uv(Vec3::Z).y.abs() < 1e-6);
        assert!((shell_mesh_uv(-Vec3::Z).y - 1.0).abs() < 1e-6);
        // Seam wraps instead of clamping: -X is u = 0.5 from both sides.
        let a = shell_mesh_uv(Vec3::new(-1.0, 1e-4, 0.0));
        let b = shell_mesh_uv(Vec3::new(-1.0, -1e-4, 0.0));
        assert!((a.x - b.x).abs() < 1e-3, "{a:?} vs {b:?}");
    }

    #[test]
    fn deck_rotation_is_identity_without_animation() {
        let q = Quat::from_rotation_y(0.7);
        assert_eq!(deck_world_rotation(q, 5.0e-6, 1234.0, false), q);
        // Animated deck advances with sim time, deterministically.
        let a = deck_world_rotation(Quat::IDENTITY, 0.004, 100.0, true);
        let b = deck_world_rotation(Quat::IDENTITY, 0.004, 100.0, true);
        assert_eq!(a, b);
        // Chord 2·sin(θ/2) recovers the 0.4 rad advance.
        let chord = (a * Vec3::X - Vec3::X).length();
        let angle = 2.0 * (chord / 2.0).asin();
        assert!((angle - 0.4).abs() < 1e-5, "{angle}");
    }

    #[test]
    fn deck_drift_runs_zonal_and_stays_physical() {
        // Drift spins about the planet pole (local +Z: the sphere poles),
        // so an equator point slides along its latitude line instead of
        // tumbling off it (the old world-Y spin did exactly that).
        let a = deck_world_rotation(Quat::IDENTITY, 0.01, 10.0, true);
        let p = (a * Vec3::X - Vec3::X).length();
        assert!(p > 0.05, "drift must visibly advance, {p}");
        assert!((a * Vec3::X).z.abs() < 1e-6, "zonal drift keeps z");
        // Table spins are breezes (tens of m/s), never km/s jets: even the
        // cirrus jet must stay two orders below the old 0.004 rad/s bug.
        for (body, deck, _, spin) in CLOUD_DECKS.iter().copied() {
            assert!(
                spin < 1.0e-4,
                "{body} deck{deck}: spin {spin} rad/s is a jet, not a breeze"
            );
        }
    }

    #[test]
    fn shadow_parallax_samples_straight_down_at_noon() {
        let up = Vec3::Y;
        let d = shadow_sample_dir(up, Vec3::Y, 3_200_000.0, 3000.0);
        assert!((d - up).length() < 1e-6, "{d:?}");
    }

    #[test]
    fn shadow_parallax_reaches_sunward_at_low_sun() {
        let up = Vec3::Y;
        // Sun 30 deg above the eastern horizon: the cloud east of overhead
        // casts onto us, so the sample shifts east (+X).
        let sun = Vec3::new(0.866, 0.5, 0.0).normalize();
        let d = shadow_sample_dir(up, sun, 3_200_000.0, 3000.0);
        assert!(d.x > 1e-3, "{d:?}");
        assert!(d.y > 0.0 && d.y < 1.0);
        // Grazing sun clamps instead of running to the horizon.
        let flat = shadow_sample_dir(up, Vec3::X, 3_200_000.0, 3000.0);
        assert!(flat.is_finite(), "{flat:?}");
        assert!(flat.x > d.x, "{flat:?} vs {d:?}");
    }

    #[test]
    fn shadow_relative_rotation_matches_deck_spin() {
        // Static deck: body and bake frames coincide.
        let q = Quat::from_rotation_y(0.7) * Quat::from_rotation_x(0.2);
        let rel = shadow_relative_rotation(q, 0.004, 999.0, false);
        assert!((rel - Quat::IDENTITY).length() < 1e-6, "{rel:?}");
        // Animated: undoes exactly the deck spin the follower applies, even
        // under a tilted planet rotation.
        let spin = deck_world_rotation(q, 0.004, 100.0, true);
        let rel = shadow_relative_rotation(q, 0.004, 100.0, true);
        let d = Vec3::new(0.3, 0.8, 0.5).normalize();
        let roundtrip = spin * (rel * d);
        assert!((roundtrip - q * d).length() < 1e-5, "{roundtrip:?}");
    }

    #[test]
    fn shadow_sun_body_inverts_planet_rotation() {
        let q = Quat::from_rotation_y(1.1);
        let sun = Vec3::new(0.2, 0.9, 0.3).normalize();
        let body = shadow_sun_body(q, sun);
        assert!((body.length() - 1.0).abs() < 1e-6);
        assert!(((q * body) - sun).length() < 1e-6, "{body:?}");
    }

    #[test]
    fn shadow_factor_shapes_alpha_without_hard_edges() {
        assert_eq!(shadow_factor_from_alpha(0.0), 1.0);
        assert_eq!(shadow_factor_from_alpha(0.04), 1.0);
        let core = shadow_factor_from_alpha(1.0);
        assert!((core - (1.0 - CLOUD_SHADOW_STRENGTH)).abs() < 1e-6);
        // Monotone: denser deck never brightens the ground.
        let mut prev = 2.0;
        for i in 0..=20 {
            let f = shadow_factor_from_alpha(i as f32 / 20.0);
            assert!(f <= prev + 1e-6, "{f} after {prev}");
            prev = f;
        }
    }

    #[test]
    fn cloud_alpha_samples_bake_texels() {
        // 2x1 bake: clear texel then overcast texel.
        let data = vec![255, 255, 255, 0, 255, 255, 255, 255];
        assert_eq!(cloud_alpha_at(&data, 2, 1, Vec2::new(0.1, 0.5)), 0.0);
        assert_eq!(cloud_alpha_at(&data, 2, 1, Vec2::new(0.9, 0.5)), 1.0);
        // u wraps the seam; empty images read clear.
        assert_eq!(cloud_alpha_at(&data, 2, 1, Vec2::new(1.1, 0.5)), 0.0);
        assert_eq!(cloud_alpha_at(&[], 0, 0, Vec2::ZERO), 0.0);
    }

    #[test]
    fn cloud_deck_table_covers_every_shell_body() {
        // Spawn path and samplers share CLOUD_DECKS; every deck-0 body that
        // the follower can place must be present exactly once here.
        for body in ["thessa", "pelagos", "borea"] {
            let decks: Vec<u8> = CLOUD_DECKS
                .iter()
                .filter(|(b, _, _, _)| *b == body)
                .map(|(_, d, _, _)| *d)
                .collect();
            assert!(decks.contains(&0), "{body}");
        }
    }

    #[test]
    fn gas_giant_bake_is_opaque_and_finite() {
        let px = bake_gas_giant(NEREID_STYLE, 64, 32, 0.0, true);
        assert_eq!(px.len(), 64 * 32 * 4);
        assert!(px.chunks(4).all(|c| c[3] == 255));
        // Storm must warm at least one texel beyond the plain band mix
        // (rust tint: strong red-green spread).
        let warm = px
            .chunks(4)
            .filter(|c| c[0] > 200 && c[0].saturating_sub(c[1]) > 40)
            .count();
        assert!(warm > 0, "storm core missing");
    }

    #[test]
    fn shell_relief_domes_cores_and_seals_the_seam() {
        use bevy::mesh::VertexAttributeValues;
        // Single overcast texel on an 8x4 grid: only neighbors dome up.
        let mut alpha = vec![0u8; 8 * 4];
        alpha[2 * 8 + 3] = 255;
        let mut mesh = Sphere::new(1.0).mesh().uv(8, 4);
        displace_shell_by_coverage(&mut mesh, &alpha, 8, 4, 0.01);
        let pos = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(p)) => p.clone(),
            _ => panic!("positions lost"),
        };
        let radii: Vec<f32> = pos
            .iter()
            .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
            .collect();
        let peak = radii.iter().cloned().fold(0.0f32, f32::max);
        // Texel centers sit half a texel off the vertex grid, so the dome
        // undershoots full relief but must clearly rise above the sphere.
        assert!(peak > 1.0005 && peak <= 1.011, "peak {peak}");
        // Far side stays on the unit sphere.
        let low = radii.iter().cloned().fold(f32::INFINITY, f32::min);
        assert!((low - 1.0).abs() < 1e-4, "low {low}");
        // UV-seam columns (u = 0 vs u = 1, same positions) displace equally.
        let uv = match mesh.attribute(Mesh::ATTRIBUTE_UV_0) {
            Some(VertexAttributeValues::Float32x2(uv)) => uv.clone(),
            _ => panic!("uvs lost"),
        };
        for (i, t) in uv.iter().enumerate() {
            if t[0] == 0.0 {
                let mate = uv
                    .iter()
                    .position(|o| o[0] == 1.0 && (o[1] - t[1]).abs() < 1e-6)
                    .expect("seam mate");
                assert!(
                    (radii[i] - radii[mate]).abs() < 1e-3,
                    "seam radii {} vs {}",
                    radii[i],
                    radii[mate]
                );
            }
        }
    }

    #[test]
    fn shell_relief_stays_mesh_fractional() {
        // Unit-sphere meshes scale by the planet radius at draw time: metre
        // relief values here would dome by thousands of radii (measured as
        // sky-filling sheets). This pins the fractional contract.
        const {
            assert!(CLOUD_RELIEF_LOW_FRAC < 0.1);
            assert!(CLOUD_RELIEF_HIGH_FRAC < 0.1);
        }
    }

    #[test]
    fn cloud_cover_tracks_coverage_knob() {
        // Calibrated shaping curve (visible fraction with alpha > 40):
        // 0.45 -> ~0.7, 0.55 -> ~0.37, 0.6 -> ~0.2, 0.7 -> ~0.03.
        // The default 0.55 reads as broken deck, not overcast.
        let frac = |coverage: f32| {
            let px = bake_clouds(0xC10D, 96, 48, coverage, 1.0, (1.0, 1.0));
            let (chunks, _) = px.as_chunks::<4>();
            chunks.iter().filter(|p| p[3] > 40).count() as f64 / (96.0 * 48.0)
        };
        let mid = frac(0.55);
        assert!((0.25..=0.50).contains(&mid), "low deck {mid:.3}");
        let cirrus = frac(0.63);
        assert!(cirrus < mid * 0.7, "cirrus {cirrus:.3} vs low {mid:.3}");
    }

    #[test]
    fn cloud_alpha_tracks_coverage() {
        let clear = bake_clouds(1, 64, 32, 0.95, 1.0, (1.0, 1.0));
        let overcast = bake_clouds(1, 64, 32, 0.05, 1.0, (1.0, 1.0));
        let mean = |px: &[u8]| {
            px.chunks(4).map(|c| c[3] as f64 / 255.0).sum::<f64>() / (px.len() / 4) as f64
        };
        assert!(mean(&overcast) > mean(&clear) + 0.3);
    }

    #[test]
    fn cloud_cores_stay_white_skirts_cool() {
        let px = bake_clouds(1, 128, 64, 0.45, 0.9, (1.0, 1.0));
        // Dense cores bright; translucent skirts cool gray-blue.
        let mut brightest = 0u8;
        let mut skirt_cool = 0;
        let mut max_a = 0u8;
        for c in px.chunks(4) {
            let (r, g, b, a) = (c[0], c[1], c[2], c[3]);
            max_a = max_a.max(a);
            if a > 140 {
                brightest = brightest.max(r).max(g).max(b);
            }
            if (20..120).contains(&a) && b as i16 >= r as i16 - 8 {
                skirt_cool += 1;
            }
        }
        assert!(
            brightest > 200,
            "dense cores must stay bright, max_a={max_a}"
        );
        assert!(skirt_cool > 0, "thin skirts must cool, not stay warm gray");
    }

    #[test]
    fn cloud_deck_carries_billow_contrast() {
        // The billow octave must put small-scale edges inside decks: mean
        // neighbor alpha steps stay well above a smooth fill. (Global std
        // is dominated by the coverage curve, not the cells.)
        let px = bake_clouds(1, 128, 64, 0.45, 0.9, (1.0, 1.0));
        let w = 128usize;
        let mut edge = 0.0;
        let mut n = 0;
        for y in 0..64 {
            for x in 0..w {
                let a = px[(y * w + x) * 4 + 3] as f64 / 255.0;
                if x + 1 < w {
                    let b = px[(y * w + x + 1) * 4 + 3] as f64 / 255.0;
                    if a > 0.15 || b > 0.15 {
                        edge += (a - b).abs();
                        n += 1;
                    }
                }
            }
        }
        let mean_edge = edge / n.max(1) as f64;
        assert!(
            mean_edge > 0.012,
            "billows must edge the deck, mean step={mean_edge:.4}"
        );
    }

    #[test]
    fn volume_path_needs_enabled_volumetric() {
        let on = ResolvedGraphicsSettings {
            clouds_enabled: true,
            clouds_volumetric: true,
            ..ResolvedGraphicsSettings::default()
        };
        assert!(use_cloud_volume(&on));
        let shells = ResolvedGraphicsSettings {
            clouds_enabled: true,
            clouds_volumetric: false,
            ..ResolvedGraphicsSettings::default()
        };
        assert!(!use_cloud_volume(&shells));
        // Disabled is free: no march even when requested.
        let off = ResolvedGraphicsSettings {
            clouds_enabled: false,
            clouds_volumetric: true,
            ..ResolvedGraphicsSettings::default()
        };
        assert!(!use_cloud_volume(&off));
    }

    #[test]
    fn volume_mesh_covers_the_tallest_towers() {
        // Per-pixel tops reach bottom + 1.4 thicknesses (hvar = 1); the mesh
        // must cover that plus margin, or rasterization rings the deck.
        for deck in [0u8, 1] {
            let (bottom, top, _) = volume_slab_for_deck(deck);
            let mesh = volume_mesh_scale_for_deck(deck);
            assert!(
                mesh > bottom + 1.4 * (top - bottom),
                "deck{deck}: mesh {mesh} must clear tower tops"
            );
        }
    }

    #[test]
    fn volume_slabs_bracket_their_shells() {
        // March slabs stay centered on the impostor shell scales in
        // CLOUD_DECKS so volume and fallback coincide on screen.
        for (body, deck, scale, _) in CLOUD_DECKS.iter().copied() {
            let (bottom, top, ext) = volume_slab_for_deck(deck);
            assert!(bottom < scale && scale < top, "{body} deck{deck}");
            assert!(ext > 0.0);
        }
        let (low_b, low_t, _) = volume_slab_for_deck(0);
        let (high_b, high_t, high_ext) = volume_slab_for_deck(1);
        assert!(high_b > low_t - 0.01, "cirrus rides above the low deck");
        assert!(
            high_t - high_b < low_t - low_b,
            "cirrus is the thinner veil"
        );
        let _ = high_ext;
        // Low deck is the optically thicker slab.
        assert!(volume_slab_for_deck(0).2 > volume_slab_for_deck(1).2);
    }

    #[test]
    fn volume_steps_and_wind_phase_stay_in_range() {
        assert_eq!(volume_steps(0), 4.0);
        assert_eq!(volume_steps(12), 12.0);
        assert_eq!(volume_steps(128), 24.0);
        assert_eq!(wind_phase_uv(123.0, false), 0.0);
        let a = wind_phase_uv(100.0, true);
        assert!((0.0..1.0).contains(&a), "{a}");
        // Pre-wrapped UV phase: a year of warp stays f32-exact and seamless
        // (integer cycles, no popping at the wrap).
        let year = wind_phase_uv(31_556_952.0, true);
        assert!((0.0..1.0).contains(&year), "{year}");
        let cycle = wind_phase_uv(1.0 / CLOUD_WIND_UV_PER_S, true);
        assert!(
            cycle.abs() < 1e-3,
            "full wind cycle must re-phase, got {cycle}"
        );
    }

    #[test]
    fn cloud_volume_wgsl_parses() {
        // The Bevy `#import` prologue (view + vertex IO) is resolved by the
        // asset pipeline at runtime; stub the two bindings this shader
        // consumes so naga validates everything else (helpers, march loop,
        // bindings). Catches syntax/type errors without a GPU.
        let src = include_str!("../../../assets/shaders/cloud_volume.wgsl");
        let (_, body) = src
            .split_once("struct CloudUniforms")
            .expect("shader keeps its uniforms block");
        let stub = "struct BevyView { world_position: vec4<f32> }\n\
            @group(0) @binding(0) var<uniform> view: BevyView;\n\
            struct VertexOutput {\n\
            \t@builtin(position) position: vec4<f32>,\n\
            \t@location(5) world_position: vec3<f32>,\n\
            \t@location(6) uv: vec2<f32>,\n\
            }\nstruct CloudUniforms";
        let assembled = format!("{stub}{body}");
        let module = naga::front::wgsl::parse_str(&assembled)
            .map_err(|e| format!("cloud_volume.wgsl must parse: {e:?}"))
            .unwrap();
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        )
        .validate(&module)
        .expect("cloud_volume.wgsl must validate");
    }

    #[test]
    fn cloud_bake_tiles_the_longitude_seam() {
        // The u=0/u=1 columns are adjacent texels on the sphere: their worst
        // step must look like an interior neighbor step, not a cliff (a far
        // visible seam reads as a painted wall from orbit).
        for aniso in [(1.0, 1.0), (1.0, 2.0)] {
            let px = bake_clouds(0xC10D, 96, 48, 0.55, 0.9, aniso);
            let at = |x: u32, y: u32| px[((y * 96 + x) * 4 + 3) as usize] as f64 / 255.0;
            let mut seam = 0.0f64;
            let mut inner = 0.0f64;
            for y in 0..48 {
                seam = seam.max((at(0, y) - at(95, y)).abs());
                for x in 0..95 {
                    inner = inner.max((at(x, y) - at(x + 1, y)).abs());
                }
            }
            assert!(
                seam <= inner + 0.05,
                "aniso {aniso:?}: seam step {seam:.3} vs interior {inner:.3}"
            );
        }
    }

    #[test]
    fn cloud_shell_fade_hides_deck_until_well_above_it() {
        // Deck at 1.2% of planet radius (Thessa low deck): hidden on the
        // ground, inside, and just above it; full only from 4x up.
        assert_eq!(cloud_shell_fade(0.0, 0.012), 0.0);
        assert_eq!(cloud_shell_fade(0.011, 0.012), 0.0);
        assert_eq!(cloud_shell_fade(0.024, 0.012), 0.0);
        // Fade band between 2x and 4x deck height.
        let mid = cloud_shell_fade(0.036, 0.012);
        assert!(
            mid > 0.0 && mid < 1.0,
            "mid fade must be partial, got {mid}"
        );
        assert_eq!(cloud_shell_fade(0.048, 0.012), 1.0);
        assert_eq!(cloud_shell_fade(10.0, 0.012), 1.0);
        // Degenerate inputs never hide the deck (fail visible, not invisible).
        assert_eq!(cloud_shell_fade(f32::NAN, 0.012), 1.0);
        assert_eq!(cloud_shell_fade(0.5, 0.0), 1.0);
    }

    #[test]
    fn shell_pole_uvs_collapse_to_one_column() {
        use bevy::mesh::VertexAttributeValues;
        let mut mesh = Sphere::new(1.0).mesh().uv(96, 48);
        let pole_count = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(pos)) => {
                pos.iter().filter(|p| p[2].abs() >= 1.0 - 1e-6).count()
            }
            _ => panic!("sphere has positions"),
        };
        assert!(pole_count > 0, "test needs real pole verts");
        collapse_shell_pole_uvs(&mut mesh);
        match mesh.attribute(Mesh::ATTRIBUTE_UV_0) {
            Some(VertexAttributeValues::Float32x2(uv)) => {
                let pos = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
                    Some(VertexAttributeValues::Float32x3(pos)) => pos.clone(),
                    _ => panic!("sphere has positions"),
                };
                for (p, t) in pos.iter().zip(uv.iter()) {
                    if p[2].abs() >= 1.0 - 1e-6 {
                        assert_eq!(t[0], 0.5, "pole u must collapse");
                    }
                }
                // Non-pole UVs untouched: equator still spans u.
                let (eq_min, eq_max) = uv
                    .iter()
                    .zip(pos.iter())
                    .filter(|(_, p)| p[2].abs() < 0.5)
                    .map(|(t, _)| t[0])
                    .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), u| {
                        (lo.min(u), hi.max(u))
                    });
                assert!(eq_max - eq_min > 0.9, "equator u must still span");
            }
            _ => panic!("sphere has uvs"),
        }
    }

    #[test]
    fn aurora_oval_emits_and_equator_does_not_bake() {
        let px = bake_aurora(128, 64, 67.0, 3.0, 0.0, false);
        let at = |lat_deg: f64| {
            let y = (((lat_deg + 90.0) / 180.0 * 64.0) as u32).min(63);
            let row = &px[(y * 128 * 4) as usize..((y + 1) * 128 * 4) as usize];
            row.chunks(4).map(|c| c[1] as u32).sum::<u32>() as f64 / 128.0
        };
        assert!(at(67.0) > at(0.0) + 20.0);
    }

    #[test]
    fn aurora_oval_carries_curtain_rays() {
        // Rays = longitude contrast along the oval row: bright rays between
        // dark lanes. Peaks must stay bright (rays never dim the maxima).
        let row_at = |lon_cells: u32, t: f64| {
            let px = bake_aurora(256, 64, 67.0, 3.0, t, true);
            let y = (((67.0 + 90.0) / 180.0 * 64.0) as u32).min(63);
            (0..lon_cells)
                .map(|x| {
                    let i = (y * 256 * 4 + x * 4) as usize;
                    px[i + 1] as f64
                })
                .collect::<Vec<_>>()
        };
        let row = row_at(256, 0.0);
        let (lo, hi) = row
            .iter()
            .fold((255.0f64, 0.0f64), |(lo, hi), &v| (lo.min(v), hi.max(v)));
        assert!(hi > 150.0, "ray peaks must stay bright, hi={hi}");
        assert!(
            hi - lo > 120.0,
            "rays must carve dark lanes, range={}..{hi}",
            lo
        );
        // Deterministic at fixed time; animated time differs (drift).
        assert_eq!(bake_aurora(64, 32, 67.0, 3.0, 5.0, true).len(), 64 * 32 * 4);
        assert_ne!(
            bake_aurora(64, 32, 67.0, 3.0, 5.0, true),
            bake_aurora(64, 32, 67.0, 3.0, 9.0, true)
        );
    }

    #[test]
    fn magnetic_latitude_matches_dipole() {
        fn magnetic_latitude(dir: DVec3, pole: DVec3) -> f64 {
            (dir.dot(pole) / (dir.length().max(1e-9) * pole.length().max(1e-9)))
                .clamp(-1.0, 1.0)
                .asin()
                .abs()
        }
        let pole = DVec3::Z;
        assert!((magnetic_latitude(DVec3::Z, pole) - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert!(magnetic_latitude(DVec3::X, pole).abs() < 1e-9);
    }

    #[test]
    fn cbt_addresses_roundtrip_for_beauty_lod() {
        for (face, level, x, y) in [(0, 0, 0, 0), (5, 4, 9, 3), (2, 8, 200, 11)] {
            let key = TileKey { face, level, x, y };
            let node = cbt_node_for_tile(key).expect("tile maps to CBT");
            assert_eq!(tile_for_cbt_node(node), Some(key));
        }
        for w in [512, 1024, 2048] {
            let level = cbt_level_for_width(w);
            let key = TileKey {
                face: 0,
                level,
                x: 0,
                y: 0,
            };
            assert!(cbt_node_for_tile(key).is_some(), "w={w}");
        }
    }

    #[test]
    fn effective_cloud_backend_strictly_decoupled_from_rt_clouds() {
        use thessa_graphics::CloudIntegratorRequest;

        // March requested: MUST resolve to March (0.0) regardless of rt_clouds.
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::March, None),
            EffectiveCloudBackend::March
        );
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::March, None).as_f32(),
            0.0
        );
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::March, None).as_str(),
            "march"
        );

        // Stochastic requested: MUST resolve to Stochastic (1.0).
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::Stochastic, None),
            EffectiveCloudBackend::Stochastic
        );
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::Stochastic, None).as_f32(),
            1.0
        );
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::Stochastic, None).as_str(),
            "stochastic"
        );

        // Dev environment override overrides request:
        for val in ["march", "0", "raymarch", "MARCH"] {
            assert_eq!(
                resolve_effective_cloud_backend(CloudIntegratorRequest::Stochastic, Some(val)),
                EffectiveCloudBackend::March,
                "override {val} should force March"
            );
        }
        for val in ["pt", "1", "stochastic", "pathtrace", "STOCHASTIC"] {
            assert_eq!(
                resolve_effective_cloud_backend(CloudIntegratorRequest::March, Some(val)),
                EffectiveCloudBackend::Stochastic,
                "override {val} should force Stochastic"
            );
        }

        // Invalid override string ignored, falls back to requested integrator:
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::March, Some("nonsense")),
            EffectiveCloudBackend::March
        );
        assert_eq!(
            resolve_effective_cloud_backend(CloudIntegratorRequest::Stochastic, Some("nonsense")),
            EffectiveCloudBackend::Stochastic
        );
    }
}
