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
    render::render_resource::{Extent3d, Face, TextureDimension, TextureFormat},
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

/// Single deck bake. Alpha = coverage-shaped fBm; RGB near-white with slight
/// density shading so decks read as thick KSP2-style cloud, not haze.
/// Earth-ref (ISS daytime): dense cores pure white, thin skirts cool gray-blue
/// with a warm sun side — baked once, still one alpha-blend shell.
fn bake_clouds(seed: u64, w: u32, h: u32, coverage: f32, opacity: f32) -> Vec<u8> {
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    // Widen the transition: decks keep hard cores but grow bright skirts.
    let edge0 = (coverage * 0.82).clamp(0.0, 0.95);
    for y in 0..h {
        let v = y as f64 / h as f64;
        let stretch = 1.0 - 0.35 * (1.0 - (v * std::f64::consts::PI).sin()); // polar pinch
        for x in 0..w {
            let u = x as f64 / w as f64;
            let d = fbm(seed, u * 6.0, v * 12.0 * stretch + 3.1, 6.0);
            let swirl = fbm(seed ^ 0x51AB, u * 3.0 + d * 1.5, v * 6.0, 3.0);
            // Billow octave: small cells riding the large field so decks read
            // as structured tops instead of smooth blobs. Bake-time only.
            // Faded toward the poles: the meridional pinch squeezes this
            // octave into coherent ringing arches there (measured).
            let polar = (v * std::f64::consts::PI).sin().powf(0.7);
            let billow = fbm(
                seed ^ 0xB110,
                u * 12.0 + d * 2.0,
                v * 24.0 * stretch + 9.7,
                12.0,
            ) * (0.25 + 0.75 * polar);
            let field = (d * 0.50 + swirl * 0.28 + billow * 0.22) as f32;
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
// Cloud relief: tangent-space normals from the deck coverage.

/// Relief strength for deck normal maps (tangent-space slope gain).
/// Cloud tops are soft: enough for sun modeling on puffs, below noisy.
pub(crate) const CLOUD_NORMAL_STRENGTH: f32 = 6.0;

/// Alpha channel grid out of an interleaved RGBA8 bake.
fn alpha_grid(px: &[u8]) -> Vec<u8> {
    px.chunks_exact(4).map(|c| c[3]).collect()
}

/// Tangent-space normal map from a deck alpha grid (row-major alpha bytes).
/// Height = coverage: dense cores read as domes, thin skirts fall away, so
/// directional sun shades puffy tops and dark crevices instead of a flat
/// decal. Same orientation convention as the globe normal bake
/// (`client_export`: nx from east gradient, ny from row gradient), mirrored
/// by the mikktspace tangents Bevy generates for the shell mesh — keep the
/// baker, the mesh tangents, and this function in sync.
pub(crate) fn bake_cloud_normals(alpha: &[u8], w: u32, h: u32, strength: f32) -> Vec<u8> {
    assert_eq!(alpha.len(), (w * h) as usize);
    let height = |x: i64, y: i64| {
        let xx = x.rem_euclid(w as i64) as u32;
        let yy = y.clamp(0, h as i64 - 1) as u32;
        alpha[(yy * w + xx) as usize] as f32 / 255.0
    };
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    for y in 0..h as i64 {
        for x in 0..w as i64 {
            // Central differences with u-wrap; v clamps at the poles.
            let gx = (height(x + 1, y) - height(x - 1, y)) * 0.5 * strength;
            let gy = (height(x, y + 1) - height(x, y - 1)) * 0.5 * strength;
            let inv = 1.0 / (gx * gx + gy * gy + 1.0).sqrt();
            let n = [-gx * inv, -gy * inv, inv];
            px.extend_from_slice(&[
                ((n[0] * 0.5 + 0.5) * 255.0) as u8,
                ((n[1] * 0.5 + 0.5) * 255.0) as u8,
                ((n[2] * 0.5 + 0.5) * 255.0) as u8,
                255,
            ]);
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

/// Physical low-deck altitude for the shadow parallax (metres).
pub(crate) const CLOUD_SHADOW_DECK_HEIGHT_M: f64 = 3000.0;
/// Deep-core shadow depth; thin skirts fade out through the alpha shaping.
pub(crate) const CLOUD_SHADOW_STRENGTH: f32 = 0.45;

/// Deck placement shared by the shell spawner, the follower, and the shadow
/// samplers: (body, deck, shell scale factor, differential spin rad/s).
/// Single source of truth; the spawn loop below reads this table.
pub(crate) const CLOUD_DECKS: &[(&str, u8, f32, f32)] = &[
    ("thessa", 0, 1.012, 0.004),
    ("thessa", 1, 1.020, 0.0064),
    ("pelagos", 0, 1.010, 0.003),
    ("borea", 0, 1.010, 0.002),
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

/// Deck world rotation shared with the follower: differential spin on top of
/// the planet rotation. When animation is off the deck is planet-fixed.
pub(crate) fn deck_world_rotation(
    planet_rotation: Quat,
    spin_rad_s: f32,
    sim_seconds: f64,
    animate: bool,
) -> Quat {
    if !animate {
        return planet_rotation;
    }
    Quat::from_rotation_y((sim_seconds as f32) * spin_rad_s) * planet_rotation
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

/// Linear normal-map image: normals are direction data, never sRGB.
fn normal_image(w: u32, h: u32, pixels: Vec<u8>) -> Image {
    Image::new(
        Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        pixels,
        TextureFormat::Rgba8Unorm,
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
    pub(crate) cloud_low_nrm: Handle<Image>,
    cloud_high_nrm: Option<Handle<Image>>,
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
            cloud_low_nrm: Handle::default(),
            cloud_high_nrm: None,
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
        app.init_resource::<BeautyTimers>()
            .add_systems(PostStartup, setup_beauty)
            .add_systems(
                Update,
                (
                    swap_gas_giant_materials,
                    follow_cloud_shells,
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
    graphics: Option<Res<GraphicsResolved>>,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    info!(
        "[beauty] resolved: gas={} clouds={}(layers={} cov={:.2} op={:.2} anim={}) aurora={}(q={:?} i={:.2} anim={})",
        r.gas_giant_enabled,
        r.clouds_enabled,
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
    let low_px = bake_clouds(0xC10D, cw, ch, r.clouds_coverage, r.clouds_opacity);
    let cloud_low = images.add(rgba_image(cw, ch, low_px.clone()));
    // Relief for sun modeling on puffy tops: normals from the same alpha
    // field the shells show, so lighting and coverage never disagree.
    let low_nrm = bake_cloud_normals(&alpha_grid(&low_px), cw, ch, CLOUD_NORMAL_STRENGTH);
    let cloud_low_nrm = images.add(normal_image(cw, ch, low_nrm));
    let (cloud_high, cloud_high_nrm) = if r.clouds_layers >= 2 {
        // Cirrus stays a thin veil over the low deck: a small coverage
        // offset lands it far down the shaping curve (measured).
        let hi_px = bake_clouds(
            0xC1225,
            cw / 2,
            ch / 2,
            (r.clouds_coverage + 0.08).min(0.95),
            r.clouds_opacity * 0.7,
        );
        let hi_nrm = bake_cloud_normals(&alpha_grid(&hi_px), cw / 2, ch / 2, CLOUD_NORMAL_STRENGTH);
        (
            Some(images.add(rgba_image(cw / 2, ch / 2, hi_px))),
            Some(images.add(normal_image(cw / 2, ch / 2, hi_nrm))),
        )
    } else {
        (None, None)
    };

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
        cloud_low,
        cloud_high,
        cloud_low_nrm,
        cloud_high_nrm,
        aurora,
        aurora_size: (aw, ah),
    });

    // Shell meshes share one unit sphere; per-shell scale follows the planet.
    // Bevy's UV sphere puts its texture poles on local +Z (see
    // SPHERE_POLE_TO_WORLD_UP in main): every pole-ring vertex shares the pole
    // position but spans the full u range, so each pole triangle smears an
    // entire texture row across the sky. From inside the shell (any low
    // flight) that reads as giant white wedges with hard triangle edges
    // (measured on a 30 km maneuver frame). Collapsing pole u to the middle
    // column reduces the wedge to a single texel.
    let mut shell_sphere = Sphere::new(1.0).mesh().uv(96, 48);
    // Tangents before the pole collapse (degenerate pole UVs would poison
    // mikktspace); the collapse only rewrites UVs afterward.
    shell_sphere
        .generate_tangents()
        .expect("shell sphere has positions, normals and uvs");
    collapse_shell_pole_uvs(&mut shell_sphere);
    let shell_mesh = meshes.add(shell_sphere);

    // Cloud shells for the three ocean worlds (map + pilot follow logic
    // below copies planet transforms; no hierarchy parenting). Placement
    // comes from CLOUD_DECKS so shadow samplers share the exact table.
    for (body, deck, scale, spin) in CLOUD_DECKS.iter().copied().filter(|(body, deck, _, _)| {
        *deck == 0 || (*body == "thessa" && (*deck as u32) < r.clouds_layers)
    }) {
        commands.spawn((
            Mesh3d(shell_mesh.clone()),
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

    // Aurora shell (Thessa only).
    // Aurora curtain stack (thessa only): three concentric shells sharing
    // one bake and rotation keep rays field-aligned coherent, while the
    // radial separation gives curtains parallax depth and a soft vertical
    // falloff instead of a painted band. Gains sum near the main shell.
    for (scale, gain) in [(1.038, 0.45), (1.045, 1.0), (1.052, 0.5)] {
        commands.spawn((
            Mesh3d(shell_mesh.clone()),
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
        (Without<CloudShell>, Without<AuroraShell>),
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
    nrm: Option<Handle<Image>>,
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
            mat.cull_mode = Some(Face::Back);
            mat.perceptual_roughness = 1.0;
            mat.metallic = 0.0;
            // Relief from the same coverage field: sun models puffy tops.
            mat.normal_map_texture = nrm;
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
        (Without<CloudShell>, Without<AuroraShell>),
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
        (Without<CloudShell>, Without<AuroraShell>),
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
        let nrm = if shell.deck == 0 {
            Some(imgs.cloud_low_nrm.clone())
        } else {
            imgs.cloud_high_nrm.clone()
        };
        let Some(tex) = tex else {
            *vis = Visibility::Hidden;
            continue;
        };
        if !r.clouds_enabled {
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
        cloud_material(&mut materials, &mat3d.0, tex, nrm, r.clouds_opacity * fade);
        st.translation = planet.translation;
        let s = planet.scale.x * shell.scale_factor;
        st.scale = Vec3::splat(s);
        // Differential drift: own slow spin on top of the planet rotation.
        st.rotation = deck_world_rotation(planet.rotation, shell.spin_rad_s, t, r.clouds_animate);
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
        (Without<CloudShell>, Without<AuroraShell>),
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
        assert_eq!(deck_world_rotation(q, 0.004, 1234.0, false), q);
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
    fn cloud_normals_are_unit_tangent_and_seamless() {
        // Flat field: straight-up normals.
        let flat = vec![200u8; 16 * 8];
        let px = bake_cloud_normals(&flat, 16, 8, 3.0);
        assert_eq!(px.len(), 16 * 8 * 4);
        for c in px.chunks_exact(4) {
            assert_eq!((c[0], c[1], c[2], c[3]), (127, 127, 255, 255));
            let n = [
                c[0] as f32 / 255.0 * 2.0 - 1.0,
                c[1] as f32 / 255.0 * 2.0 - 1.0,
                c[2] as f32 / 255.0 * 2.0 - 1.0,
            ];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!((len - 1.0).abs() < 0.02, "{n:?}");
        }
        // East ramp (alpha grows with +u): normals tilt toward -u, matching
        // the globe-bake convention (nx from the east gradient, negated).
        let mut ramp = vec![0u8; 16 * 8];
        for y in 0..8 {
            for x in 0..16 {
                ramp[y * 16 + x] = (x * 16) as u8;
            }
        }
        let px = bake_cloud_normals(&ramp, 16, 8, 3.0);
        let mid = &px[(4 * 16 + 8) * 4..][..4];
        assert!(mid[0] < 128, "east ramp tilts -u, got {mid:?}");
        assert!((mid[1] as i16 - 128).abs() <= 2, "no v tilt, got {mid:?}");
        assert!(mid[2] > 200, "mostly up, got {mid:?}");
        // u-wrap: the seam columns sample wrapped neighbors, so a linear
        // ramp yields identical normals on both sides (no visible seam).
        let left = &px[(4 * 16) * 4..][..4];
        let right = &px[(4 * 16 + 15) * 4..][..4];
        for i in 0..3 {
            assert!(
                (left[i] as i16 - right[i] as i16).abs() <= 1,
                "seam {left:?} vs {right:?}"
            );
        }
    }

    #[test]
    fn cloud_cover_tracks_coverage_knob() {
        // Calibrated shaping curve (visible fraction with alpha > 40):
        // 0.45 -> ~0.7, 0.55 -> ~0.37, 0.6 -> ~0.2, 0.7 -> ~0.03.
        // The default 0.55 reads as broken deck, not overcast.
        let frac = |coverage: f32| {
            bake_clouds(0xC10D, 96, 48, coverage, 1.0)
                .chunks_exact(4)
                .filter(|p| p[3] > 40)
                .count() as f64
                / (96.0 * 48.0)
        };
        let mid = frac(0.55);
        assert!((0.25..=0.50).contains(&mid), "low deck {mid:.3}");
        let cirrus = frac(0.63);
        assert!(cirrus < mid * 0.7, "cirrus {cirrus:.3} vs low {mid:.3}");
    }

    #[test]
    fn cloud_alpha_tracks_coverage() {
        let clear = bake_clouds(1, 64, 32, 0.95, 1.0);
        let overcast = bake_clouds(1, 64, 32, 0.05, 1.0);
        let mean = |px: &[u8]| {
            px.chunks(4).map(|c| c[3] as f64 / 255.0).sum::<f64>() / (px.len() / 4) as f64
        };
        assert!(mean(&overcast) > mean(&clear) + 0.3);
    }

    #[test]
    fn cloud_cores_stay_white_skirts_cool() {
        let px = bake_clouds(1, 128, 64, 0.45, 0.9);
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
        let px = bake_clouds(1, 128, 64, 0.45, 0.9);
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
}
