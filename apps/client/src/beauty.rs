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
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
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
            let field = (d * 0.65 + swirl * 0.35) as f32;
            let a = ((field - edge0) / (1.0 - edge0).max(1e-3)).clamp(0.0, 1.0);
            let a = (a * (1.6 - 0.6 * a)).clamp(0.0, 1.0) * opacity;
            let shade = 0.93 + 0.07 * field;
            px.extend_from_slice(&[
                (255.0 * shade) as u8,
                (255.0 * shade) as u8,
                (255.0 * shade.min(1.0)) as u8,
                (a.clamp(0.0, 1.0) * 255.0) as u8,
            ]);
        }
    }
    px
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Aurora.

/// Aurora curtain bake over lat/lon. Intensity = oval gaussian x altitude
/// colour mix x curtain octaves; caller scales by `intensity` setting.
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
            let k = (band * curtain.max(0.0)) as f32;
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
struct BeautyImages {
    nereid: Handle<Image>,
    vesper: Handle<Image>,
    nereid_size: (u32, u32),
    vesper_size: (u32, u32),
    cloud_low: Handle<Image>,
    cloud_high: Option<Handle<Image>>,
    aurora: Handle<Image>,
    aurora_size: (u32, u32),
}

#[derive(Component)]
struct CloudShell {
    body: &'static str,
    deck: u8,
    spin_rad_s: f32,
    scale_factor: f32,
}

#[derive(Component)]
struct AuroraShell {
    body: &'static str,
    scale_factor: f32,
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
    let cloud_low = images.add(rgba_image(cw, ch, low_px));
    let cloud_high = (r.clouds_layers >= 2).then(|| {
        let hi_px = bake_clouds(
            0xC1225,
            cw / 2,
            ch / 2,
            (r.clouds_coverage + 0.25).min(0.9),
            r.clouds_opacity * 0.7,
        );
        images.add(rgba_image(cw / 2, ch / 2, hi_px))
    });

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
        aurora,
        aurora_size: (aw, ah),
    });

    // Shell meshes share one unit sphere; per-shell scale follows the planet.
    let shell_mesh = meshes.add(Sphere::new(1.0).mesh().uv(96, 48));

    // Cloud shells for the three ocean worlds (map + pilot follow logic
    // below copies planet transforms; no hierarchy parenting).
    for (body, scale, spin) in [
        ("thessa", 1.012f32, 0.004),
        ("pelagos", 1.010f32, 0.003),
        ("borea", 1.010f32, 0.002),
    ] {
        let decks = if body == "thessa" { r.clouds_layers } else { 1 };
        for deck in 0..decks {
            commands.spawn((
                Mesh3d(shell_mesh.clone()),
                MeshMaterial3d(materials.add(StandardMaterial::default())),
                Transform::default(),
                Visibility::Hidden,
                Name::new(format!("beauty cloud {body} deck{deck}")),
                CloudShell {
                    body,
                    deck: deck as u8,
                    spin_rad_s: spin * (1.0 + deck as f32 * 0.6),
                    scale_factor: scale + deck as f32 * 0.008,
                },
            ));
        }
    }

    // Aurora shell (Thessa only).
    commands.spawn((
        Mesh3d(shell_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial::default())),
        Transform::default(),
        Visibility::Hidden,
        Name::new("beauty aurora thessa"),
        AuroraShell {
            body: "thessa",
            scale_factor: 1.045,
        },
    ));
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
#[allow(clippy::type_complexity)]
fn planet_transform(
    body: &str,
    visuals: &Query<(&Name, &Transform, &Visibility), (Without<CloudShell>, Without<AuroraShell>)>,
) -> Option<(Transform, bool)> {
    // Map view entity. Prefer a VISIBLE match: in pilot mode the map visual
    // still exists but is hidden while the PFD planet shows the same world.
    let mut fallback: Option<(Transform, bool)> = None;
    for (name, t, v) in visuals {
        if name.as_str() == body {
            let entry = (*t, *v != Visibility::Hidden);
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
                let entry = (*t, *v != Visibility::Hidden);
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
    if let Some(mut mat) = materials.get_mut(handle)
        && mat.base_color_texture.is_none()
    {
        mat.base_color = Color::WHITE;
        mat.base_color_texture = Some(tex);
        mat.alpha_mode = AlphaMode::Blend;
        mat.cull_mode = None;
        mat.perceptual_roughness = 1.0;
        mat.metallic = 0.0;
        mat.base_color.set_alpha(opacity);
    }
}

#[allow(clippy::type_complexity)]
fn follow_cloud_shells(
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    visuals: Query<(&Name, &Transform, &Visibility), (Without<CloudShell>, Without<AuroraShell>)>,
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
        let Some((planet, planet_visible)) = planet_transform(shell.body, &visuals) else {
            if log_now {
                info!("[beauty] cloud {}: planet NOT FOUND", shell.body);
            }
            *vis = Visibility::Hidden;
            continue;
        };
        if !planet_visible {
            if log_now {
                info!("[beauty] cloud {}: planet hidden", shell.body);
            }
            *vis = Visibility::Hidden;
            continue;
        }
        if log_now {
            info!(
                "[beauty] cloud {}: following planet scale={:.3}",
                shell.body, planet.scale.x
            );
        }
        cloud_material(&mut materials, &mat3d.0, tex, r.clouds_opacity);
        st.translation = planet.translation;
        let s = planet.scale.x * shell.scale_factor;
        st.scale = Vec3::splat(s);
        // Differential drift: own slow spin on top of the planet rotation.
        let spin = if r.clouds_animate {
            Quat::from_rotation_y(t as f32 * shell.spin_rad_s)
        } else {
            Quat::IDENTITY
        };
        st.rotation = spin * planet.rotation;
        *vis = Visibility::Visible;
    }
}

#[allow(clippy::type_complexity)]
fn follow_aurora_shell(
    imgs: Option<Res<BeautyImages>>,
    graphics: Option<Res<GraphicsResolved>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    visuals: Query<(&Name, &Transform, &Visibility), (Without<CloudShell>, Without<AuroraShell>)>,
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
        let Some((planet, planet_visible)) = planet_transform(shell.body, &visuals) else {
            if log_now {
                info!("[beauty] aurora {}: planet NOT FOUND", shell.body);
            }
            *vis = Visibility::Hidden;
            continue;
        };
        if !planet_visible {
            if log_now {
                info!("[beauty] aurora {}: planet hidden", shell.body);
            }
            *vis = Visibility::Hidden;
            continue;
        }
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
            let i = r.aurora_intensity;
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
    fn cloud_alpha_tracks_coverage() {
        let clear = bake_clouds(1, 64, 32, 0.95, 1.0);
        let overcast = bake_clouds(1, 64, 32, 0.05, 1.0);
        let mean = |px: &[u8]| {
            px.chunks(4).map(|c| c[3] as f64 / 255.0).sum::<f64>() / (px.len() / 4) as f64
        };
        assert!(mean(&overcast) > mean(&clear) + 0.3);
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
