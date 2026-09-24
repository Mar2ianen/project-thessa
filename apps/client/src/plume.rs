//! Engine plume renderer: field-first, mesh-last (`docs/38`).
//!
//! Rule: exhaust is a compact world-space participating-medium field.
//! Meshes, lights, and future ray queries consume that field, never the
//! source of its shape.
//!
//! Paths by quality (same physical input, representation differs):
//!
//! - Low: `PlumeBackend::Impostor`, the old CPU-baked cone (fixed bands,
//!   throttle scale, sine flicker). Documented fallback artifact.
//! - Medium/High: analytic-volume ribbon. A camera-facing quad is COVERAGE
//!   ONLY; every pixel marches a view ray through the round cross-section
//!   and Beer-Lambert-integrates the `plume-core` mean field (8 evals on
//!   Medium, 14 on High). Shock diamonds emerge from the volume via the
//!   same shock factor as the CPU oracle, never from a decal. Turbulence
//!   spans the azimuth (fixed frame from the axis) and hard-clips outside
//!   the true barrel, so the bound cylinder can never render as a milky
//!   sheet wider than the plume.
//!
//! Lighting proxies derive from the field integral (`radiant_power`), not
//! from bare throttle (doc section 13).
//!
//! Inputs: `EnginePlumeInput` (thin craft-state read; engine-sim will grow
//! it) plus ambient pressure from the same atmosphere model as the flight
//! model. Nozzle deck values are explicitly provisional until compiled
//! vehicle/engine data lands (see `provisional_rocket_source`).

use super::*;
use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    pbr::{Material, MaterialPlugin},
    reflect::TypePath,
    render::render_resource::{AsBindGroup, Extent3d, ShaderType, TextureDimension, TextureFormat},
    shader::ShaderRef,
};
use thessa_plume_core::medium::radiant_power;
use thessa_plume_core::optics::optical_material;
use thessa_plume_core::profile::{
    AxialProfile, build_axial_profile, emission_gain, expansion_fan, shock_amplitude,
    shock_cell_spacing_m, spread_rate,
};
use thessa_plume_core::source::{
    ExhaustFamily, PlumeEnvironment, PlumeSource, RigidTransform, pressure_ratio,
};

// ---------------------------------------------------------------------------
// Inputs.

/// Thin craft-state read for the plume renderer. Engine-sim grows this into
/// nozzle/engine state; the field/render side keeps its shape.
#[derive(Debug, Clone, Copy)]
pub struct EnginePlumeInput {
    pub throttle: f64,
    pub engine_active: bool,
    /// Absolute sim time: warp-consistent animation clock for consumers.
    /// Unread until the volume pass drives advection from input directly.
    #[allow(dead_code)]
    pub sim_time_s: f64,
}

impl EnginePlumeInput {
    pub fn active_amount(self) -> f64 {
        if !self.engine_active {
            0.0
        } else {
            self.throttle.clamp(0.0, 1.0)
        }
    }
}

fn read_plume_input(runtime: &PilotFlightRuntime, clock: &SimulationClock) -> EnginePlumeInput {
    EnginePlumeInput {
        throttle: runtime.throttle,
        engine_active: runtime.engine_active,
        sim_time_s: clock.sim_seconds,
    }
}

/// PROVISIONAL nozzle deck (order-of-magnitude methalox rocket values, NOT a
/// measured engine deck). Exit pressure is set mildly underexpanded at sea
/// level on purpose so the diamond-chain path gets exercised; a matched or
/// overexpanded deck would hide it. Stands in for compiled vehicle/engine
/// data, which does not exist yet. Affects plume SHAPE (impostor and volume
/// alike); flagged for replacement, never presented as calibration.
fn provisional_rocket_source(throttle: f64) -> PlumeSource {
    PlumeSource {
        nozzle_to_vehicle: RigidTransform::IDENTITY,
        exit_radius_m: 0.55,
        mass_flow_kg_s: 480.0,
        exhaust_velocity_mps: 3550.0,
        exit_pressure_pa: 120_000.0,
        exit_temperature_k: 1850.0,
        exit_mach: 3.5,
        throttle: throttle.clamp(0.0, 1.0),
        exhaust: ExhaustFamily::Methalox,
    }
}

/// Provisional X-15 nozzle offset in craft space (nose +Y, top -Z): tail
/// station. From craft geometry, not propulsion; moves into vehicle data
/// with the real nozzle description.
const PROVISIONAL_NOZZLE_LOCAL: Vec3 = Vec3::new(0.0, -7.5, 0.0);

/// Provisional photometric scale from field-integrated radiant power to
/// point-light intensity. Calibrated by eye against the old `1800*throttle`
/// look; flagged for measurement against the HDR frame.
const PROVISIONAL_LUMEN_SCALE: f64 = 12.0;

// ---------------------------------------------------------------------------
// Volume material (Medium/High path).

/// Packed uniforms mirroring `assets/shaders/plume_volume.wgsl`.
#[derive(Debug, Clone, Copy, ShaderType)]
pub struct PlumeUniforms {
    origin_len: Vec4,
    axis_r0: Vec4,
    shape_time: Vec4,
    march: Vec4,
    core_rgb: Vec4,
    mid_rgb: Vec4,
    edge_rgb: Vec4,
}

#[derive(Debug, Clone, Asset, TypePath, AsBindGroup)]
pub struct PlumeVolumeMaterial {
    #[uniform(0)]
    pub uniforms: PlumeUniforms,
}

impl Material for PlumeVolumeMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/plume_volume.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode {
        AlphaMode::Add
    }
}

// ---------------------------------------------------------------------------
// Impostor texture bake (Low path only; fixed-band fallback artifact).

fn bake_plume(w: u32, h: u32, diamonds: bool) -> Vec<u8> {
    let mut px = Vec::with_capacity((w * h * 4) as usize);
    for y in 0..h {
        let v = y as f64 / (h - 1).max(1) as f64;
        let width = (1.0 - v * 0.75).max(0.15);
        for x in 0..w {
            let u = (x as f64 + 0.5) / w as f64 * 2.0 - 1.0;
            let r = (u.abs() / width).min(1.0);
            let body = (1.0 - r * r).max(0.0);
            let axial = (-v * 3.2).exp();
            let mut bright = body * axial;
            if diamonds {
                let cells = (0.5 + 0.5 * (v * 5.0 * std::f64::consts::PI * 2.0).sin()).powi(2);
                bright *= 0.55 + 0.45 * cells * (-v * 2.0).exp();
            }
            let core = (bright * 1.6).min(1.0);
            let r_c = (1.0f64).min(0.35 + core);
            let g_c = 0.85 * core + 0.15 * body;
            let b_c = 0.55 * core * core;
            let a = (bright * 1.25).clamp(0.0, 1.0);
            px.extend_from_slice(&[
                (r_c.clamp(0.0, 1.0) * 255.0) as u8,
                (g_c.clamp(0.0, 1.0) * 255.0) as u8,
                (b_c.clamp(0.0, 1.0) * 255.0) as u8,
                (a * 255.0) as u8,
            ]);
        }
    }
    px
}

/// Deterministic fallback flicker in [0.82, 1.12] (impostor path only; the
/// volume path advects turbulence instead of breathing the whole plume).
fn plume_flicker(sim_time_s: f64, throttle: f64) -> f64 {
    let t = sim_time_s;
    1.0 + 0.10 * (t * 37.0).sin() * throttle + 0.06 * (t * 61.0 + 1.3).sin() * throttle
        - 0.04 * throttle
}

// ---------------------------------------------------------------------------
// Entities / resources.

#[derive(Component)]
struct PlumeCone;

#[derive(Component)]
struct PlumeVolume;

#[derive(Component)]
struct PlumeLight;

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

/// Cached analytic field: rebuilt when quantized inputs move, never per
/// frame unconditionally. Keeps the 1 us profile build off the hot path.
#[derive(Resource)]
struct PlumeFieldCache {
    key: (i64, i64),
    profile: AxialProfile,
    source: PlumeSource,
    env: PlumeEnvironment,
    radiant: [f64; 3],
}

impl PlumeFieldCache {
    fn empty() -> Self {
        let source = provisional_rocket_source(0.0);
        let env = PlumeEnvironment {
            pressure_pa: 0.0,
            density_kg_m3: 0.0,
            temperature_k: 0.0,
            oxygen_fraction: 0.0,
            flow_velocity_local_mps: [0.0; 3],
        };
        Self {
            key: (i64::MIN, i64::MIN),
            profile: AxialProfile::empty(thessa_plume_core::profile::expansion_regime(
                &source, &env,
            )),
            source,
            env,
            radiant: [0.0; 3],
        }
    }
}

pub struct PlumePlugin;

impl Plugin for PlumePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<PlumeVolumeMaterial>::default())
            .insert_resource(PlumeFieldCache::empty())
            .add_systems(PostStartup, setup_plume)
            .add_systems(Update, (update_plume_field, drive_plume_consumers).chain());
    }
}

fn setup_plume(
    mut commands: Commands,
    graphics: Option<Res<GraphicsResolved>>,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut volume_materials: ResMut<Assets<PlumeVolumeMaterial>>,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();

    // Low-path impostor texture (fixed bands = documented fallback artifact).
    let plume_tex = images.add(rgba_image(32, 256, bake_plume(32, 256, r.plume_diamonds)));

    // Impostor cone (craft space: nose +Y, exhaust -Y; apex downstream).
    let cone_h = 7.0;
    let cone = meshes.add(Cone {
        radius: 1.1,
        height: cone_h,
    });
    commands.spawn((
        Mesh3d(cone),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(plume_tex),
            alpha_mode: AlphaMode::Add,
            cull_mode: None,
            unlit: true,
            ..default()
        })),
        Transform::default(),
        Visibility::Hidden,
        Name::new("plume cone"),
        PlumeCone,
    ));

    // Volume ribbon: unit quad in XY, scaled/oriented per frame into a
    // cylindrical billboard around the exhaust axis. Coverage only.
    let ribbon = meshes.add(quad_mesh());
    commands.spawn((
        Mesh3d(ribbon),
        MeshMaterial3d(volume_materials.add(PlumeVolumeMaterial {
            uniforms: PlumeUniforms {
                origin_len: Vec4::new(0.0, 0.0, 0.0, 1.0),
                axis_r0: Vec4::new(0.0, -1.0, 0.0, 0.5),
                shape_time: Vec4::new(1.0, 5.0, 0.0, 0.0),
                march: Vec4::new(0.0, 0.0, 6.0, 300.0),
                core_rgb: Vec4::new(1.0, 1.0, 1.0, 0.4),
                mid_rgb: Vec4::ONE,
                edge_rgb: Vec4::ONE,
            },
        })),
        Transform::default(),
        Visibility::Hidden,
        Name::new("plume volume"),
        PlumeVolume,
    ));

    commands.spawn((
        PointLight {
            intensity: 0.0,
            range: 60.0,
            color: Color::srgb(1.0, 0.62, 0.25),
            ..default()
        },
        Transform::default(),
        Visibility::Hidden,
        Name::new("plume light"),
        PlumeLight,
    ));
}

/// Double-wound unit quad in XY (visible from both sides regardless of cull
/// mode), with UVs. The shader uses world position, not UVs.
fn quad_mesh() -> Mesh {
    let positions = [
        [-1.0, -1.0, 0.0],
        [1.0, -1.0, 0.0],
        [1.0, 1.0, 0.0],
        [-1.0, 1.0, 0.0],
    ];
    let uvs = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let normals = [[0.0, 0.0, 1.0]; 4];
    // Two triangles per winding so culling can never hide the ribbon.
    let indices = [0, 1, 2, 0, 2, 3, 0, 3, 2, 0, 2, 1];
    let mut mesh = Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions.to_vec());
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs.to_vec());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals.to_vec());
    mesh.insert_indices(bevy::mesh::Indices::U32(indices.to_vec()));
    mesh
}

// ---------------------------------------------------------------------------
// Field update + consumers.

/// Rebuild the analytic field when quantized inputs move. Ambient comes from
/// the same atmosphere model as the flight model; sampling failure (above
/// atmosphere top) is vacuum, never a fallback constant.
fn update_plume_field(
    clock: Option<Res<SimulationClock>>,
    runtime: Option<Res<PilotFlightRuntime>>,
    graphics: Option<Res<GraphicsResolved>>,
    mut cache: ResMut<PlumeFieldCache>,
) {
    let (Some(clock), Some(runtime)) = (clock.as_deref(), runtime.as_deref()) else {
        return;
    };
    let _ = graphics;
    let input = read_plume_input(runtime, clock);
    let amount = input.active_amount();

    let altitude_m = (runtime.relative_position_m.length() - runtime.planet_radius_m).max(0.0);
    let env = match runtime.atmosphere.sample(altitude_m) {
        Ok(sample) => PlumeEnvironment {
            pressure_pa: sample.pressure_pa.max(0.0),
            density_kg_m3: sample.density_kg_m3.max(0.0),
            temperature_k: sample.temperature_k.max(0.0),
            oxygen_fraction: 0.21,
            flow_velocity_local_mps: [0.0; 3],
        },
        Err(_) => PlumeEnvironment {
            pressure_pa: 0.0,
            density_kg_m3: 0.0,
            temperature_k: 0.0,
            oxygen_fraction: 0.0,
            flow_velocity_local_mps: [0.0; 3],
        },
    };
    // Quantize: rebuild on material change, not on float noise.
    let key = (
        (amount * 50.0).round() as i64,
        (env.pressure_pa / (env.pressure_pa * 0.02 + 50.0)).round() as i64,
    );
    if key == cache.key {
        return;
    }
    let source = provisional_rocket_source(amount);
    match build_axial_profile(&source, &env, 48) {
        Ok(profile) => {
            cache.radiant = radiant_power(&profile);
            cache.profile = profile;
            cache.source = source;
            cache.env = env;
            cache.key = key;
        }
        Err(_) => {
            cache.radiant = [0.0; 3];
            cache.profile =
                AxialProfile::empty(thessa_plume_core::profile::expansion_regime(&source, &env));
            cache.source = source;
            cache.env = env;
            cache.key = key;
        }
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn drive_plume_consumers(
    cache: Res<PlumeFieldCache>,
    graphics: Option<Res<GraphicsResolved>>,
    clock: Option<Res<SimulationClock>>,
    cameras: Query<
        &Transform,
        (
            With<Camera3d>,
            Without<PlumeCone>,
            Without<PlumeVolume>,
            Without<PlumeLight>,
        ),
    >,
    craft: Query<
        (&Name, &GlobalTransform, &Visibility),
        (
            Without<PlumeCone>,
            Without<PlumeVolume>,
            Without<PlumeLight>,
        ),
    >,
    pilot: Option<Res<PilotHudState>>,
    mut cone: Query<
        (&mut Transform, &mut Visibility),
        (With<PlumeCone>, Without<PlumeVolume>, Without<PlumeLight>),
    >,
    mut volume: Query<
        (&mut Transform, &mut Visibility),
        (With<PlumeVolume>, Without<PlumeCone>, Without<PlumeLight>),
    >,
    mut volume_materials: ResMut<Assets<PlumeVolumeMaterial>>,
    volume_mat: Query<&MeshMaterial3d<PlumeVolumeMaterial>, With<PlumeVolume>>,
    mut light: Query<
        (&mut Transform, &mut Visibility, &mut PointLight),
        (With<PlumeLight>, Without<PlumeCone>, Without<PlumeVolume>),
    >,
) {
    let r = graphics.as_deref().map(|g| g.0.clone()).unwrap_or_default();
    let in_pilot = pilot
        .as_deref()
        .is_some_and(|s| s.view_mode == ClientViewMode::Pilot);
    // Disabled plumes must not pay for the name scan below.
    let enabled = r.plume_enabled && in_pilot;
    // Pilot (metre) space only: map-view transforms are compressed-AU and
    // meaningless for metre-scale plume consumers.
    let mut craft_frame: Option<GlobalTransform> = None;
    if enabled {
        for (name, g, v) in &craft {
            if name.as_str() == "PFD North American X-15" && *v != Visibility::Hidden {
                craft_frame = Some(*g);
                break;
            }
        }
    }
    let show = enabled && craft_frame.is_some() && !cache.profile.is_empty();
    let Ok((mut cone_t, mut cone_v)) = cone.single_mut() else {
        return;
    };
    let Ok((mut vol_t, mut vol_v)) = volume.single_mut() else {
        return;
    };
    let Ok((mut light_t, mut light_v, mut point)) = light.single_mut() else {
        return;
    };
    if !show {
        *cone_v = Visibility::Hidden;
        *vol_v = Visibility::Hidden;
        *light_v = Visibility::Hidden;
        point.intensity = 0.0;
        return;
    }
    let frame = craft_frame.unwrap();
    let craft_rot = Quat::from_mat4(&frame.to_matrix());
    let exhaust = (craft_rot * Vec3::NEG_Y).normalize_or_zero();
    let nozzle = frame.transform_point(PROVISIONAL_NOZZLE_LOCAL);
    let clock_t = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);

    // Field-derived light proxy (doc section 13): radiative integral mapped
    // to intensity by a documented photometric scale.
    let radiant_luma = (cache.radiant[0] + cache.radiant[1] + cache.radiant[2]) / 3.0;
    if r.plume_light {
        point.intensity = (PROVISIONAL_LUMEN_SCALE * radiant_luma) as f32;
        light_t.translation = nozzle;
        *light_v = Visibility::Visible;
    } else {
        point.intensity = 0.0;
        *light_v = Visibility::Hidden;
    }

    let use_volume = !matches!(r.plume_quality, thessa_graphics::Quality::Low);
    if use_volume {
        *cone_v = Visibility::Hidden;
        drive_volume(
            &r,
            &cache,
            clock_t,
            &cameras,
            &mut vol_t,
            &mut vol_v,
            &mut volume_materials,
            &volume_mat,
            nozzle,
            exhaust,
        );
    } else {
        *vol_v = Visibility::Hidden;
        drive_cone(
            &r,
            &cache,
            clock_t,
            craft_rot,
            frame,
            &mut cone_t,
            &mut cone_v,
            nozzle,
        );
    }
}

#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
fn drive_volume(
    r: &ResolvedGraphicsSettings,
    cache: &PlumeFieldCache,
    sim_time_s: f64,
    cameras: &Query<
        &Transform,
        (
            With<Camera3d>,
            Without<PlumeCone>,
            Without<PlumeVolume>,
            Without<PlumeLight>,
        ),
    >,
    vol_t: &mut Transform,
    vol_v: &mut Visibility,
    volume_materials: &mut Assets<PlumeVolumeMaterial>,
    volume_mat: &Query<&MeshMaterial3d<PlumeVolumeMaterial>, With<PlumeVolume>>,
    nozzle: Vec3,
    exhaust: Vec3,
) {
    let profile = &cache.profile;
    let last = profile.stations[profile.stations.len() - 1];
    let len = profile.length_m.max(2.0) as f32;
    let r_tail = last.radius_m.max(0.2) as f32;
    let diameter = (2.0 * cache.source.exit_radius_m) as f32;
    let pi = pressure_ratio(&cache.source, &cache.env);
    let amp = if r.plume_diamonds {
        shock_amplitude(pi) as f32
    } else {
        0.0 // debug override (doc section 9): representation switch only
    };
    let material = optical_material(cache.source.exhaust);
    // Same shared divisor as the builder: uniforms carry chromaticity,
    // absolute brightness lives in the gain uniform (no double count).
    let inv_divisor = 1.0 / thessa_plume_core::hue_divisor(material) as f32;
    let gain = emission_gain(&cache.source) as f32;
    let ext_mean = profile
        .stations
        .iter()
        .map(|s| s.extinction_per_m)
        .sum::<f64>() as f32
        / profile.stations.len().max(1) as f32;
    let steps = match r.plume_quality {
        thessa_graphics::Quality::Low => 4,
        thessa_graphics::Quality::Medium => 8,
        thessa_graphics::Quality::High => 14,
    } as f32;
    let time = if r.plume_flicker {
        sim_time_s as f32
    } else {
        0.0
    };

    // Cylindrical billboard around the exhaust axis.
    let center = nozzle + exhaust * (len / 2.0);
    let cam_pos = cameras
        .iter()
        .next()
        .map(|t| t.translation)
        .unwrap_or(center + Vec3::Z);
    let mut x_axis = exhaust.cross(center - cam_pos).normalize_or_zero();
    if x_axis.length_squared() < 1e-6 {
        x_axis = craft_lateral_fallback(exhaust);
    }
    let z_axis = x_axis.cross(exhaust).normalize_or_zero();
    vol_t.translation = center;
    vol_t.rotation = Quat::from_mat3(&Mat3::from_cols(x_axis, exhaust, z_axis));
    vol_t.scale = Vec3::new(diameter * 0.5 + r_tail * 1.6, len, 1.0);
    *vol_v = Visibility::Visible;

    if let Ok(handle) = volume_mat.single()
        && let Some(mut mat) = volume_materials.get_mut(&handle.0)
    {
        mat.uniforms = PlumeUniforms {
            origin_len: Vec4::new(nozzle.x, nozzle.y, nozzle.z, len),
            axis_r0: Vec4::new(
                exhaust.x,
                exhaust.y,
                exhaust.z,
                cache.source.exit_radius_m as f32,
            ),
            shape_time: Vec4::new(
                r_tail,
                shock_cell_spacing_m(2.0 * cache.source.exit_radius_m, cache.source.exit_mach, pi)
                    as f32,
                amp,
                time,
            ),
            march: Vec4::new(
                gain,
                ext_mean,
                steps,
                cache.source.exhaust_velocity_mps as f32 * 0.1,
            ),
            core_rgb: Vec4::new(
                material.core_rgb[0] as f32 * inv_divisor,
                material.core_rgb[1] as f32 * inv_divisor,
                material.core_rgb[2] as f32 * inv_divisor,
                0.40,
            ),
            mid_rgb: Vec4::new(
                material.mid_rgb[0] as f32 * inv_divisor,
                material.mid_rgb[1] as f32 * inv_divisor,
                material.mid_rgb[2] as f32 * inv_divisor,
                spread_rate(pi) as f32,
            ),
            edge_rgb: Vec4::new(
                material.edge_rgb[0] as f32 * inv_divisor,
                material.edge_rgb[1] as f32 * inv_divisor,
                material.edge_rgb[2] as f32 * inv_divisor,
                expansion_fan(pi) as f32,
            ),
        };
    }
}

fn craft_lateral_fallback(exhaust: Vec3) -> Vec3 {
    // Looking straight down the plume: pick any perpendicular.
    if exhaust.y.abs() < 0.9 {
        exhaust.cross(Vec3::Y).normalize_or_zero()
    } else {
        exhaust.cross(Vec3::X).normalize_or_zero()
    }
}

#[allow(clippy::too_many_arguments)]
fn drive_cone(
    r: &ResolvedGraphicsSettings,
    cache: &PlumeFieldCache,
    sim_time_s: f64,
    craft_rot: Quat,
    frame: GlobalTransform,
    cone_t: &mut Transform,
    cone_v: &mut Visibility,
    nozzle: Vec3,
) {
    // Low-path impostor: fixed geometry scaled by throttle (fallback only).
    let amount = cache.source.throttle;
    let flick = if r.plume_flicker {
        plume_flicker(sim_time_s, amount)
    } else {
        1.0
    };
    let len = (2.0 + 6.5 * amount) * flick;
    cone_t.rotation = craft_rot * Quat::from_rotation_z(std::f32::consts::PI);
    let girth = ((0.35 + 0.65 * amount) * flick.clamp(0.8, 1.2)) as f32;
    cone_t.scale = Vec3::new(girth, (len / 7.0) as f32, girth);
    cone_t.translation = nozzle + (craft_rot * Vec3::NEG_Y) * (len as f32 / 2.0);
    *cone_v = Visibility::Visible;
    let _ = frame;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plume_input_contract_inactive_hides() {
        let off = EnginePlumeInput {
            throttle: 1.0,
            engine_active: false,
            sim_time_s: 10.0,
        };
        assert_eq!(off.active_amount(), 0.0);
        let on = EnginePlumeInput {
            throttle: 0.8,
            engine_active: true,
            sim_time_s: 10.0,
        };
        assert!((on.active_amount() - 0.8).abs() < 1e-12);
        assert!((plume_flicker(10.0, 0.8) - 1.0).abs() < 0.25);
    }

    #[test]
    fn plume_alpha_decays_downstream() {
        let px = bake_plume(16, 64, true);
        let row_alpha = |y: u32| {
            let row = &px[(y * 16 * 4) as usize..((y + 1) * 16 * 4) as usize];
            row.chunks(4).map(|c| c[3] as u32).sum::<u32>() as f64 / 16.0
        };
        assert!(row_alpha(2) > row_alpha(60));
    }

    #[test]
    fn quad_mesh_is_double_wound_with_uvs() {
        let mesh = quad_mesh();
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
        assert!(mesh.attribute(Mesh::ATTRIBUTE_UV_0).is_some());
        let indices = mesh.indices().expect("quad has indices");
        match indices {
            bevy::mesh::Indices::U32(list) => assert_eq!(list.len(), 12),
            _ => panic!("expected U32 indices"),
        }
    }
}
