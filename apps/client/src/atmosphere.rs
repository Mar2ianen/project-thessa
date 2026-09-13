//! Visual atmosphere: Bevy-atmosphere raster path plus experimental Solari RT.
//!
//! Architecture (spec `docs/14_VISUAL_ATMOSPHERE.md` sections 13-14, 22):
//!
//! ```text
//! system.toml body/star data + absolute sim time
//!     -> shared AtmosphereOptics (thessa-atmosphere, no graphics API)
//!     -> Bevy ScatteringMedium / Atmosphere      (raster path)
//!     -> ray-aware CPU queries                   (Solari companion)
//! ```
//!
//! There is exactly one optical definition per body. Changing graphics
//! quality changes budgets (LUT sizes), never physics or climate state.

use super::perf::PerfMonitor;
use super::*;

use bevy::camera::Exposure;
use bevy::light::Atmosphere;
use bevy::light::SunDisk;
use bevy::light::atmosphere::{Falloff, PhaseFunction, ScatteringMedium, ScatteringTerm};
use bevy::pbr::{AtmosphereMode, AtmosphereSettings};
use bevy::post_process::auto_exposure::{
    AutoExposure, AutoExposureCompensationCurve, AutoExposurePlugin,
};
use bevy::solari::prelude::{RaytracingMesh3d, SolariLighting};

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use thessa_atmosphere::{
    AtmosphereOptics, CelestialLight, angular_radius, blackbody_rgb, eclipse_visibility,
    irradiance_at_distance, nitrogen_oxygen_optics,
};
use thessa_graphics::{Quality, RequestedGraphics, ResolvedGraphicsSettings};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, SystemConfig};

/// Solar luminosity (W) and radius (m) for irradiance/angular-size math.
const SOL_LUMINOSITY_W: f64 = 3.828e26;
const SOL_RADIUS_M: f64 = 6.957e8;
/// Approximate luminous efficacy of starlight (lm/W) for lux conversion.
const LUX_PER_W_M2: f64 = 93.0;
/// Fallback N2/O2-air point until per-body composition lands in config.
/// Pressure already comes from `system.toml`; only these two stay fallback.
const FALLBACK_AIR_TEMPERATURE_K: f64 = 278.0;
const FALLBACK_AIR_GAS_CONSTANT: f64 = 287.05;

/// Set in `main()` before plugins run: Solari RT path requested and resolved.
#[derive(Resource, Clone, Copy)]
pub struct RayTracingActive(pub bool);

/// Bevy resource wrappers: the `thessa-graphics` model stays free of Bevy
/// (module boundary), so the client wraps it for the ECS.
#[derive(Resource, Clone)]
pub struct GraphicsRequested(pub RequestedGraphics);

/// Resolved settings resource consumed by the renderer and perf captures.
#[derive(Resource, Clone)]
pub struct GraphicsResolved(pub ResolvedGraphicsSettings);

/// Marker on the Asterion-A directional light spawned in `setup()`.
#[derive(Component)]
pub struct PrimaryStarLight;

/// Marker on the secondary-star fill light spawned by this plugin.
#[derive(Component)]
struct SecondaryStarLight;

/// Marker on the tertiary-star (Asterion C) fill light. Geometrically near B
/// but ~1000x dimmer and red: a night-side point, not a third sun.
#[derive(Component)]
struct ThirdStarLight;

/// Marker on the shared-atmosphere shell entity following Thessa.
#[derive(Component)]
struct ThessaAtmosphereShell;

/// One catalogued star with visual parameters from `system.toml`.
#[derive(Debug, Clone)]
struct StarVisual {
    name: String,
    body: Option<BodyId>,
    temperature_k: f64,
    luminosity_w: f64,
    radius_m: f64,
}

#[derive(Resource)]
struct StellarCatalog {
    stars: Vec<StarVisual>,
}

/// Body ids needed every frame; `None` degrades to fewer lights instead of
/// a broken frame. The reference is the flown world, the occluder its host:
/// no star or planet names anywhere in the renderer.
#[derive(Resource, Default)]
struct AtmosphereBodies {
    reference: Option<BodyId>,
    occluder: Option<BodyId>,
}

/// Star bodies by id, from `system.toml`. Geometry proxies hide stars (their
/// disks come from the atmosphere shader); membership beats name matching so
/// any configured system works.
#[derive(Resource, Default)]
pub(super) struct StarBodyIds {
    pub(super) ids: HashSet<BodyId>,
}

/// Shared optical definition for Thessa plus derived constants.
#[derive(Resource)]
struct ThessaOptics {
    /// Single definition consumed by the medium asset at startup and by
    /// future LUT baking / ray-aware queries; per-frame sync needs only the
    /// scalars below, so this field is intentionally read rarely.
    #[allow(dead_code)]
    optics: AtmosphereOptics,
    scale_height_m: f64,
    body_radius_m: f64,
}

/// Eclipse hysteresis state for ingress/egress event markers.
#[derive(Resource, Default)]
struct EclipseState {
    eclipsed: bool,
}

pub struct AtmospherePlugin;

impl Plugin for AtmospherePlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(AutoExposurePlugin)
            .init_resource::<EclipseState>()
            .add_systems(Update, configure_solari_precision)
            .add_systems(
                Update,
                toggle_ray_tracing
                    .before(sync_rt_view)
                    .before(update_atmosphere_visuals),
            )
            .add_systems(Update, sync_rt_view.after(terrain::TerrainUpdate))
            .add_systems(PostStartup, atmosphere_startup)
            .add_systems(
                Update,
                update_atmosphere_visuals.after(terrain::TerrainUpdate),
            )
            .add_systems(
                PostUpdate,
                sync_solari_meshes
                    .after(bevy::camera::visibility::VisibilitySystems::VisibilityPropagate),
            );
    }
}

/// Solari 0.19's fixed 1 mm secondary-ray epsilon is smaller than f32
/// reconstruction error on kilometre terrain (f32 quantum at 3.2e6 m is
/// ~0.25 m), which shades as full-frame speckle acne — verified live in RT
/// survey shots. Adapt only this pinned shader constant to 10 cm. This is
/// safe as a global patch ONLY because `sync_rt_view` below keeps RT in
/// metre-scale scenes: the compressed map (1 unit = DISTANCE_UNIT_M metres)
/// never feeds Solari, where 0.1 units would mean megametres. Domain physics
/// and collision queries stay untouched. Keep the import path/defs intact,
/// and handle async shader loading explicitly.
fn configure_solari_precision(mut shaders: ResMut<Assets<Shader>>, mut done: Local<bool>) {
    if *done {
        return;
    }
    let target = shaders.iter().find_map(|(id, shader)| {
        shader
            .path
            .contains("raytracing_scene_bindings.wgsl")
            .then_some(id)
    });
    let Some(id) = target else {
        return;
    };
    let mut shader = shaders.get_mut(id).expect("loaded shader");
    if let bevy::shader::Source::Wgsl(source) = &mut shader.source {
        const FROM: &str = "const RAY_T_MIN = 0.001f;";
        if source.contains(FROM) {
            *source = source.replace(FROM, "const RAY_T_MIN = 0.1f;").into();
            eprintln!("[solari] terrain ray epsilon: 0.1 m (Bevy 0.19 precision adapter)");
        } else {
            eprintln!("[solari] shader epsilon contract changed; precision adapter skipped");
        }
    }
    *done = true;
}

/// Device capability is fixed at startup, but lighting can switch live.
fn toggle_ray_tracing(
    keys: Res<ButtonInput<KeyCode>>,
    device: Res<bevy::render::renderer::RenderDevice>,
    mut active: ResMut<RayTracingActive>,
    requested: Res<GraphicsRequested>,
    mut graphics: ResMut<GraphicsResolved>,
    mut perf: ResMut<perf::PerfMonitor>,
) {
    if !keys.just_pressed(KeyCode::F12)
        || !(keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight))
    {
        return;
    }
    if !device
        .features()
        .contains(bevy::solari::SolariPlugins::required_wgpu_features())
    {
        perf.push_event(
            "RT unavailable",
            Some("Device lacks required ray-query features".into()),
        );
        return;
    }
    active.0 = !active.0;
    let on = active.0;
    let resolved = &mut graphics.0;
    resolved.ray_tracing = if on {
        thessa_graphics::ResolvedRayTracing::Local
    } else {
        thessa_graphics::ResolvedRayTracing::Off
    };
    resolved.rt_terrain = on && requested.0.raytracing.terrain;
    resolved.rt_vehicles = on && requested.0.raytracing.vehicles;
    resolved.rt_landmarks = on && requested.0.raytracing.landmarks;
    resolved.rt_atmosphere_queries = on && requested.0.raytracing.atmosphere;
    resolved.rt_clouds = on && requested.0.raytracing.clouds;
    resolved.aurora_lighting = on && requested.0.upper_atmosphere.aurora_lighting;
    perf.push_event(
        "RT toggled",
        Some(if on { "local" } else { "raster" }.into()),
    );
}

/// RT applies only to metre-scale scenes (pilot/survey). The map uses
/// compressed astronomical units and never feeds Solari (see
/// `configure_solari_precision`); `Full` included. Domain physics untouched.
fn sync_rt_view(
    mut commands: Commands,
    active: Res<RayTracingActive>,
    pilot: Res<PilotHudState>,
    survey: Res<terrain::SurfaceSurvey>,
    cameras: Query<(Entity, Option<&SolariLighting>), With<Camera3d>>,
    mut lights: Query<&mut DirectionalLight, With<PrimaryStarLight>>,
    mut grace: Local<u32>,
) {
    let wanted = active.0 && (survey.active || pilot.view_mode == ClientViewMode::Pilot);
    for mut light in &mut lights {
        light.shadow_maps_enabled = !wanted;
    }
    if wanted {
        // BLAS builds lag the toggle by a frame or two; enabling the RT
        // camera immediately renders a geometry-less frame that poisons the
        // temporal reservoirs (visible flash/pop). Hold raster for 2 frames.
        *grace = grace.saturating_add(1);
        if *grace < 3 {
            return;
        }
    } else {
        *grace = 0;
    }
    for (entity, lighting) in &cameras {
        if wanted && lighting.is_none() {
            enable_solari_camera(&mut commands, entity);
        }
        if !wanted && lighting.is_some() {
            commands.entity(entity).remove::<SolariLighting>();
        }
    }
}

/// Surface pressure for a named world from `data/system.toml`
/// (`atmosphere_bar` on its moon/planet entry). Falls back to the 1.20 bar
/// design reference so a config edit never breaks startup; the fallback is
/// logged. The caller passes the flown world's own name, never a literal.
fn body_surface_pressure_pa(body_name: &str) -> f64 {
    const FALLBACK_PA: f64 = 120_000.0;
    let table: toml::Table = match toml::from_str(include_str!("../../../data/system.toml")) {
        Ok(table) => table,
        Err(error) => {
            eprintln!("[atmosphere] system.toml unreadable for pressure ({error}); using 1.20 bar");
            return FALLBACK_PA;
        }
    };
    for key in ["moon", "planet"] {
        if let Some(toml::Value::Array(bodies)) = table.get(key) {
            for body in bodies {
                let matches = body
                    .get("id")
                    .and_then(toml::Value::as_str)
                    .is_some_and(|id| id == body_name);
                if !matches {
                    continue;
                }
                if let Some(bar) = body.get("atmosphere_bar").and_then(toml::Value::as_float) {
                    return bar * 100_000.0;
                }
            }
        }
    }
    eprintln!("[atmosphere] no atmosphere_bar for {body_name}; using 1.20 bar design reference");
    FALLBACK_PA
}

/// One resolved stellar source. Slots (primary/fills) are assigned by
/// irradiance at the observer, brightest first — roles follow physics, never
/// star names, so single suns, binaries and red companions all just work.
#[derive(Debug, Clone)]
struct ResolvedStar {
    name: String,
    light: CelestialLight,
}

fn resolve_star_lights(
    catalog: &[StarVisual],
    ephemeris: &BakedEphemeris,
    observer: bevy::math::DVec3,
    sim_time: SimTime,
) -> Vec<ResolvedStar> {
    let mut out = Vec::new();
    for star in catalog {
        let Some(id) = star.body else { continue };
        let Ok(state) = ephemeris.body_state(id, sim_time) else {
            continue;
        };
        let delta = state.position_inertial - observer;
        let distance = delta.length().max(1.0);
        out.push(ResolvedStar {
            name: star.name.clone(),
            light: CelestialLight {
                direction_to_star: delta / distance,
                irradiance_w_m2: irradiance_at_distance(star.luminosity_w, distance),
                color_rgb: blackbody_rgb(star.temperature_k),
                angular_radius_rad: angular_radius(star.radius_m, distance),
                visibility: 1.0,
            },
        });
    }
    out.sort_by(|a, b| b.light.irradiance_w_m2.total_cmp(&a.light.irradiance_w_m2));
    out
}

/// Map-sphere colors for a star from its effective temperature: sRGB base plus
/// a linear HDR emissive at map-symbol brightness (not physics). Hues come
/// from data; the ×25 scale only matches the established map look.
pub(super) fn star_mesh_colors(temperature_k: f64) -> (Color, LinearRgba) {
    const EMISSIVE_SCALE: f32 = 25.0;
    let tint = blackbody_rgb(temperature_k);
    let srgb = |c: f64| c.clamp(0.0, 1.0).powf(1.0 / 2.2) as f32;
    (
        Color::srgb(srgb(tint[0]), srgb(tint[1]), srgb(tint[2])),
        LinearRgba::rgb(
            tint[0] as f32 * EMISSIVE_SCALE,
            tint[1] as f32 * EMISSIVE_SCALE,
            tint[2] as f32 * EMISSIVE_SCALE,
        ),
    )
}

/// Convert shared optics into a Bevy scattering medium (same definition the
/// CPU queries and any future LUT baking consume). `density_scale` is a
/// renderer-side optical-depth multiplier (appearance only, never physics):
/// it scales every extinction coefficient uniformly, preserving color ratios.
fn medium_from_optics(
    optics: &AtmosphereOptics,
    falloff_resolution: u32,
    phase_resolution: u32,
    density_scale: f32,
) -> ScatteringMedium {
    let height = (optics.outer_radius_m - optics.inner_radius_m).max(1.0);
    let k = density_scale.max(0.0);
    let rgb =
        |beta: [f64; 3]| Vec3::new(beta[0] as f32 * k, beta[1] as f32 * k, beta[2] as f32 * k);
    let mut terms = vec![
        ScatteringTerm {
            absorption: Vec3::ZERO,
            scattering: rgb(optics.rayleigh.beta_rgb),
            falloff: Falloff::Exponential {
                scale: (optics.rayleigh.scale_height_m / height) as f32,
            },
            phase: PhaseFunction::Rayleigh,
        },
        ScatteringTerm {
            absorption: Vec3::ZERO,
            scattering: rgb(optics.mie.beta_rgb),
            falloff: Falloff::Exponential {
                scale: (optics.mie.scale_height_m / height) as f32,
            },
            phase: PhaseFunction::Mie {
                asymmetry: optics.mie.asymmetry_g as f32,
            },
        },
    ];
    for layer in &optics.absorption {
        terms.push(ScatteringTerm {
            absorption: rgb(layer.sigma_rgb),
            scattering: Vec3::ZERO,
            falloff: Falloff::Exponential {
                scale: (layer.scale_height_m / height) as f32,
            },
            phase: PhaseFunction::Isotropic,
        });
    }
    ScatteringMedium::new(falloff_resolution, phase_resolution, terms)
        .with_label("thessa_atmosphere")
}

/// Stellar catalog from the checked-in system configuration.
fn stellar_catalog(config: &SystemConfig, ephemeris: &BakedEphemeris) -> Vec<StarVisual> {
    config
        .star
        .iter()
        .map(|star| StarVisual {
            body: ephemeris.body_id(&star.id),
            name: star.id.clone(),
            temperature_k: star.temperature_k,
            luminosity_w: star.luminosity_solar * SOL_LUMINOSITY_W,
            radius_m: star.radius_solar * SOL_RADIUS_M,
        })
        .collect()
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn atmosphere_startup(
    mut commands: Commands,
    runtime: Option<Res<RuntimeEphemeris>>,
    flight: Option<Res<PilotFlightRuntime>>,
    clock: Option<Res<SimulationClock>>,
    requested: Option<Res<GraphicsRequested>>,
    resolved: Option<Res<GraphicsResolved>>,
    rt_active: Option<Res<RayTracingActive>>,
    cameras: Query<Entity, With<Camera3d>>,
    mut media: ResMut<Assets<ScatteringMedium>>,
    mut exposure_curves: ResMut<Assets<AutoExposureCompensationCurve>>,
    mut meshes: ResMut<Assets<Mesh>>,
    rt_targets: Query<(
        Entity,
        &Mesh3d,
        &MeshMaterial3d<StandardMaterial>,
        Option<&CelestialVisual>,
        Option<&StarMarker>,
    )>,
) {
    let Some(runtime) = runtime.as_deref() else {
        eprintln!("[atmosphere] no ephemeris yet; atmosphere disabled this session");
        return;
    };
    let requested: RequestedGraphics = requested.map(|r| r.0.clone()).unwrap_or_default();
    let resolved: ResolvedGraphicsSettings = resolved.map(|r| r.0.clone()).unwrap_or_default();
    let rt_active = rt_active.is_some_and(|flag| flag.0);

    let config: SystemConfig = match toml::from_str(include_str!("../../../data/system.toml")) {
        Ok(config) => config,
        Err(error) => {
            eprintln!("[atmosphere] system.toml parse failed ({error}); atmosphere disabled");
            return;
        }
    };
    let ephemeris = &runtime.ephemeris;
    let catalog_stars = stellar_catalog(&config, ephemeris);
    commands.insert_resource(StellarCatalog {
        stars: catalog_stars.clone(),
    });
    // The flown world and its host come from live resources, never literals:
    // the renderer works for any reference body in any configured system.
    let reference = flight.as_deref().map(|flight| flight.reference_body);
    let occluder = reference
        .and_then(|id| ephemeris.body(id).ok())
        .and_then(|body| body.parent);
    commands.insert_resource(AtmosphereBodies {
        reference,
        occluder,
    });
    commands.insert_resource(StarBodyIds {
        ids: config
            .star
            .iter()
            .filter_map(|star| ephemeris.body_id(&star.id))
            .collect(),
    });

    // Shared optical definition: body radius and gravity from baked data,
    // pressure from system.toml, N2/O2 air model per design reference.
    let Some(reference_id) = reference else {
        eprintln!("[atmosphere] no flight reference body; atmosphere disabled");
        return;
    };
    let body = match ephemeris.body(reference_id) {
        Ok(body) => body,
        Err(error) => {
            eprintln!("[atmosphere] reference body unreadable ({error}); disabled");
            return;
        }
    };
    let gravity = body.mu / body.radius_m.powi(2);
    let scale_height = FALLBACK_AIR_GAS_CONSTANT * FALLBACK_AIR_TEMPERATURE_K / gravity.max(0.1);
    let pressure_pa = body_surface_pressure_pa(&body.name);
    let optics = match nitrogen_oxygen_optics(body.radius_m, pressure_pa, scale_height) {
        Ok(optics) => optics,
        Err(error) => {
            eprintln!("[atmosphere] optics invalid ({error}); disabled");
            return;
        }
    };
    let (falloff_res, phase_res) = match resolved.atmosphere_quality {
        Quality::Low => (128, 64),
        Quality::Medium => (256, 128),
        Quality::High => (256, 256),
    };
    let medium = media.add(medium_from_optics(
        &optics,
        falloff_res,
        phase_res,
        resolved.atmosphere_density_scale,
    ));
    let log_beta = optics.rayleigh.beta_rgb;
    let log_density = resolved.atmosphere_density_scale;
    commands.insert_resource(ThessaOptics {
        scale_height_m: scale_height,
        body_radius_m: body.radius_m,
        optics,
    });

    // One shell entity follows Thessa in whichever view is active; radii and
    // transform are synced every frame by `update_atmosphere_visuals`.
    // Disabled stays hidden: the updater returns early while off, so a
    // visible stale shell would sit at the origin in metre radii with the
    // camera inside it (fullscreen grey wash).
    let shell = commands
        .spawn((
            ThessaAtmosphereShell,
            Atmosphere {
                inner_radius: body.radius_m as f32,
                outer_radius: (body.radius_m + scale_height * 8.0) as f32,
                // Area-weighted mean linear albedo of the canonical v3 map.
                ground_albedo: Vec3::new(0.1153, 0.1407, 0.1375),
                medium,
            },
            Transform::default(),
            if resolved.atmosphere_enabled {
                Visibility::Visible
            } else {
                Visibility::Hidden
            },
            Name::new(format!("{} atmosphere shell", body.name)),
        ))
        .id();

    for camera in &cameras {
        commands.entity(camera).insert(Exposure {
            ev100: requested.renderer.exposure_ev100,
        });
        // No sky pass while disabled: without a synced shell the LUT space
        // has no valid planet frame, so leave the camera on clear color.
        if !resolved.atmosphere_enabled {
            continue;
        }
        commands
            .entity(camera)
            .insert(atmosphere_settings(resolved.atmosphere_quality));
        if resolved.auto_exposure {
            commands.entity(camera).insert(AutoExposure {
                // Meter the HDR buffer after physical camera exposure.
                range: -8.0..=8.0,
                // Meter to 18% middle gray, not unit-white HDR luminance.
                compensation_curve: exposure_curves.add(
                    AutoExposureCompensationCurve::from_curve(
                        bevy::math::cubic_splines::LinearSpline::new([
                            Vec2::new(-8.0, -2.47),
                            Vec2::new(8.0, -2.47),
                        ]),
                    )
                    .expect("constant exposure compensation"),
                ),
                speed_brighten: 3.0,
                speed_darken: 1.5,
                ..default()
            });
        }
        if rt_active {
            enable_solari_camera(&mut commands, camera);
        }
    }

    // Fill lights for every non-primary star, brightest fill first. Initial
    // disks and tints come from the same resolver the per-frame update uses
    // (true angular sizes, never Bevy's Earth default); the update keeps them
    // exact afterwards.
    let sim_start = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
    let initial_observer = ephemeris
        .body_state(reference_id, SimTime(sim_start))
        .map(|state| state.position_inertial)
        .unwrap_or(bevy::math::DVec3::ZERO);
    let initial = resolve_star_lights(
        &catalog_stars,
        ephemeris,
        initial_observer,
        SimTime(sim_start),
    );
    let fill_initial = |index: usize| -> (f32, Color, f32) {
        initial.get(index).map_or((0.0, Color::WHITE, 0.0), |slot| {
            (
                (slot.light.irradiance_w_m2 * LUX_PER_W_M2) as f32,
                linear_color(slot.light.color_rgb),
                (2.0 * slot.light.angular_radius_rad) as f32,
            )
        })
    };
    let (illum_b, color_b, disk_b) = fill_initial(1);
    commands.spawn((
        SecondaryStarLight,
        DirectionalLight {
            illuminance: illum_b,
            color: color_b,
            shadow_maps_enabled: false,
            ..default()
        },
        bevy::light::SunDisk {
            angular_size: disk_b,
            intensity: 1.0,
        },
        Transform::IDENTITY,
        Name::new("Stellar fill 2"),
    ));

    let (illum_c, color_c, disk_c) = fill_initial(2);
    commands.spawn((
        ThirdStarLight,
        DirectionalLight {
            illuminance: illum_c,
            color: color_c,
            shadow_maps_enabled: false,
            ..default()
        },
        bevy::light::SunDisk {
            angular_size: disk_c,
            intensity: 1.0,
        },
        Transform::IDENTITY,
        Name::new("Stellar fill 3"),
    ));

    if rt_active {
        let targets: Vec<(Entity, Handle<Mesh>, bool)> = rt_targets
            .iter()
            .filter(|(_, _, _, _, star)| star.is_none())
            .map(|(entity, mesh, _, visual, _)| (entity, mesh.0.clone(), visual.is_some()))
            .collect();
        enable_solari_meshes(&mut commands, &resolved, &mut meshes, &targets);
    }

    for note in &resolved.notes {
        eprintln!("[graphics] {note}");
    }
    for reserved in reserved_but_unimplemented(&requested) {
        eprintln!("[graphics] reserved for follow-up, not rendered yet: {reserved}");
    }
    eprintln!(
        "[atmosphere] shell {:?} around {} R={:.0} m H={:.0} m pressure={:.0} Pa density_scale={} rayleigh_beta={:?}; RT {}",
        shell,
        body.name,
        body.radius_m,
        scale_height,
        pressure_pa,
        log_density,
        log_beta,
        if rt_active { "on" } else { "off" },
    );
}

/// Camera atmosphere quality: LUT budgets by preset quality (budgets only,
/// never physics). Raymarched rendering stays reserved for debugging.
#[allow(clippy::field_reassign_with_default)]
fn atmosphere_settings(quality: Quality) -> AtmosphereSettings {
    let mut settings = AtmosphereSettings::default();
    settings.rendering_method = AtmosphereMode::LookupTexture;
    match quality {
        Quality::Low => {
            settings.transmittance_lut_size = UVec2::new(128, 64);
            settings.transmittance_lut_samples = 20;
            settings.sky_view_lut_size = UVec2::new(200, 100);
            settings.sky_view_lut_samples = 8;
            settings.aerial_view_lut_size = UVec3::new(16, 16, 16);
            settings.aerial_view_lut_samples = 6;
        }
        Quality::Medium => {
            settings.transmittance_lut_size = UVec2::new(256, 128);
            settings.sky_view_lut_size = UVec2::new(256, 128);
            settings.sky_view_lut_samples = 12;
            settings.aerial_view_lut_size = UVec3::new(24, 24, 24);
        }
        Quality::High => {
            settings.aerial_view_lut_size = UVec3::new(32, 32, 64);
        }
    }
    settings
}

/// Settings toggles that exist in the schema for the GUI but have no renderer
/// yet. Listed at startup so nobody mistakes them for working switches.
fn reserved_but_unimplemented(requested: &RequestedGraphics) -> Vec<&'static str> {
    let mut pending = Vec::new();
    if requested.atmosphere.multiple_scattering {
        pending.push("atmosphere.multiple_scattering (single-scattering slice)");
    }
    if requested.upper_atmosphere.airglow {
        pending.push("upper_atmosphere.airglow (emission shell follow-up)");
    }
    if requested.upper_atmosphere.aurora {
        pending.push("upper_atmosphere.aurora (emission shell follow-up)");
    }
    if requested.upper_atmosphere.aurora_lighting {
        pending.push("upper_atmosphere.aurora_lighting (RT proxy follow-up)");
    }
    if requested.clouds.enabled {
        pending.push("clouds (separate subsystem, spec section 10)");
    }
    pending
}

/// Solari camera requirements: lighting component plus storage-binding usage.
/// `Msaa::Off` is inserted in `main()` before plugins run.
fn enable_solari_camera(commands: &mut Commands, camera: Entity) {
    use bevy::camera::CameraMainTextureUsages;
    use bevy::render::render_resource::TextureUsages;
    commands.entity(camera).insert((
        SolariLighting::default(),
        CameraMainTextureUsages::default().with(TextureUsages::STORAGE_BINDING),
    ));
}

/// BLAS participation for planet and vehicle meshes, with asset fixups per
/// the Solari contract (UVs, tangents, U32 indices, no second UV set).
fn enable_solari_meshes(
    commands: &mut Commands,
    resolved: &ResolvedGraphicsSettings,
    meshes: &mut Assets<Mesh>,
    targets: &[(Entity, Handle<Mesh>, bool)],
) {
    let mut fixed: HashMap<Handle<Mesh>, bool> = HashMap::new();
    for (entity, handle, is_terrain) in targets {
        let wanted = if *is_terrain {
            resolved.rt_terrain
        } else {
            resolved.rt_vehicles
        };
        if !wanted {
            continue;
        }
        if !*fixed
            .entry(handle.clone())
            .or_insert_with(|| ensure_rt_mesh_compatible(meshes, handle))
        {
            continue;
        }
        commands
            .entity(*entity)
            .insert(RaytracingMesh3d(handle.clone()));
    }
}

/// Bring one mesh asset into the Solari-compatible subset. Logs and keeps the
/// raster path untouched when a mesh cannot be upgraded.
fn ensure_rt_mesh_compatible(meshes: &mut Assets<Mesh>, handle: &Handle<Mesh>) -> bool {
    let Some(mut mesh) = meshes.get_mut(handle) else {
        return false;
    };
    if !mesh.contains_attribute(Mesh::ATTRIBUTE_UV_0) {
        let count = mesh.count_vertices();
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0]; count]);
    }
    if !mesh.contains_attribute(Mesh::ATTRIBUTE_TANGENT) && mesh.generate_tangents().is_err() {
        eprintln!("[solari] tangents unavailable; mesh left raster-only");
        return false;
    }
    if mesh.contains_attribute(Mesh::ATTRIBUTE_UV_1) {
        mesh.remove_attribute(Mesh::ATTRIBUTE_UV_1);
    }
    if let Some(indices) = mesh.indices_mut()
        && let bevy::mesh::Indices::U16(_) = indices
    {
        *indices = bevy::mesh::Indices::U32(indices.iter().map(|i| i as u32).collect());
    }
    true
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn update_atmosphere_visuals(
    clock: Option<Res<SimulationClock>>,
    runtime: Option<Res<RuntimeEphemeris>>,
    pilot: Option<Res<PilotHudState>>,
    bodies: Option<Res<AtmosphereBodies>>,
    catalog: Option<Res<StellarCatalog>>,
    tuning: Option<Res<ThessaOptics>>,
    resolved: Option<Res<GraphicsResolved>>,
    mut eclipse: ResMut<EclipseState>,
    mut primary: Query<
        (&mut DirectionalLight, &mut Transform, &mut SunDisk),
        (
            With<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThirdStarLight>,
            Without<ThessaAtmosphereShell>,
            Without<CelestialVisual>,
        ),
    >,
    mut secondary: Query<
        (&mut DirectionalLight, &mut Transform, &mut SunDisk),
        (
            With<SecondaryStarLight>,
            Without<PrimaryStarLight>,
            Without<ThirdStarLight>,
            Without<ThessaAtmosphereShell>,
            Without<CelestialVisual>,
        ),
    >,
    mut tertiary: Query<
        (&mut DirectionalLight, &mut Transform, &mut SunDisk),
        (
            With<ThirdStarLight>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThessaAtmosphereShell>,
            Without<CelestialVisual>,
        ),
    >,
    mut shell: Query<
        (&mut Atmosphere, &mut Transform),
        (
            With<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThirdStarLight>,
            Without<CelestialVisual>,
        ),
    >,
    visuals: Query<
        (&CelestialVisual, &Transform),
        (
            With<CelestialVisual>,
            Without<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThirdStarLight>,
        ),
    >,
    terrain_data: (Option<Res<terrain::WorldTerrain>>, Res<PilotFlightRuntime>),
    mut camera_settings: Query<
        (&mut AtmosphereSettings, &Transform),
        (
            With<Camera3d>,
            Without<CelestialVisual>,
            Without<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThirdStarLight>,
        ),
    >,
    mut monitor: ResMut<PerfMonitor>,
) {
    let (terrain, flight_runtime) = terrain_data;
    let frame_start = Instant::now();
    let (Some(runtime), Some(bodies), Some(catalog), Some(tuning), Some(graphics)) = (
        runtime.as_deref(),
        bodies.as_deref(),
        catalog.as_deref(),
        tuning.as_deref(),
        resolved.as_deref(),
    ) else {
        return;
    };
    let resolved = &graphics.0;
    if !resolved.atmosphere_enabled {
        return;
    }
    let sim_time = clock.as_deref().map(|c| c.sim_seconds).unwrap_or(0.0);
    let in_pilot = pilot
        .as_deref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    let ephemeris = &runtime.ephemeris;

    // --- celestial geometry from the authoritative ephemeris ---
    let Some(reference_id) = bodies.reference else {
        return;
    };
    let Ok(thessa_state) = ephemeris.body_state(reference_id, SimTime(sim_time)) else {
        return;
    };
    let local_scene = in_pilot
        || terrain
            .as_deref()
            .is_some_and(|w| w.render_center.is_some());
    let offset = if local_scene {
        let origin = terrain
            .as_deref()
            .filter(|w| w.render_center.is_some())
            .map(|w| w.render_origin_m)
            .unwrap_or_else(|| bevy::math::DVec3::from_array(flight_runtime.terrain_origin()));
        let camera = camera_settings
            .iter()
            .next()
            .map(|(_, t)| t.translation.as_dvec3())
            .unwrap_or_default();
        let p = origin + camera;
        bevy::math::DVec3::new(p.x, -p.z, p.y)
    } else {
        bevy::math::DVec3::ZERO
    };
    let observer = thessa_state.position_inertial + offset;
    // Slots follow irradiance, brightest first: roles are physics, not names.
    let mut stars = resolve_star_lights(&catalog.stars, ephemeris, observer, SimTime(sim_time));

    // --- eclipse: host occluder over each source, continuous 0..1 ---
    // One geometric factor feeds direct light, sky scattering and markers
    // together; the disk dims uniformly as a documented first approximation
    // of the partial-disk shape.
    let mut occluder_name = String::new();
    if resolved.eclipses
        && let Some(occ_id) = bodies.occluder
        && let Ok(occ) = ephemeris.body_state(occ_id, SimTime(sim_time))
    {
        let occ_delta = occ.position_inertial - observer;
        let occ_dist = occ_delta.length().max(1.0);
        let occ_radius = ephemeris.body(occ_id).map(|b| b.radius_m).unwrap_or(0.0);
        occluder_name = ephemeris
            .body(occ_id)
            .map(|b| b.name.clone())
            .unwrap_or_default();
        let occ_dir = occ_delta / occ_dist;
        let occ_ang = angular_radius(occ_radius, occ_dist);
        for star in &mut stars {
            star.light.visibility = eclipse_visibility(
                star.light.direction_to_star,
                star.light.angular_radius_rad,
                occ_dir,
                occ_ang,
            );
        }
    }
    // Ingress/egress markers on the primary with hysteresis so penumbra
    // grazing does not spam.
    if let Some(top) = stars.first() {
        if !eclipse.eclipsed && top.light.visibility < 0.02 {
            eclipse.eclipsed = true;
            monitor.push_event(
                "eclipse ingress",
                Some(format!("{} behind {}", top.name, occluder_name)),
            );
        } else if eclipse.eclipsed && top.light.visibility > 0.98 {
            eclipse.eclipsed = false;
            monitor.push_event("eclipse egress", None);
        }
    }

    // --- direct lights: raw-sun scale so the atmosphere LUT does the filtering ---
    // Visible disks use true angular diameters (Bevy defaults to Earth's Sun
    // per light, which would draw every companion as a bogus second sun).
    drive_slot(&mut primary.iter_mut(), stars.first(), 1.0);
    if resolved.multi_star {
        drive_slot(&mut secondary.iter_mut(), stars.get(1), 0.3);
        drive_slot(&mut tertiary.iter_mut(), stars.get(2), 0.3);
    } else {
        drive_slot(&mut secondary.iter_mut(), None, 0.0);
        drive_slot(&mut tertiary.iter_mut(), None, 0.0);
    }

    if resolved.ray_tracing.is_active() {
        // Solari 0.19 computes omega = TAU*(1-cos(diameter/2)) in f32.
        // Subpixel companions otherwise become omega=0 and poison RIS/GI
        // with infinite radiance, even for a disabled (zero-lux) slot.
        for (_, _, mut disk) in primary
            .iter_mut()
            .chain(secondary.iter_mut())
            .chain(tertiary.iter_mut())
        {
            let physical = disk.angular_size;
            disk.angular_size = rt_safe_stellar_diameter(physical);
            disk.intensity *= (physical / disk.angular_size).powi(2);
        }
    }

    // Aerial perspective must cover the visible horizon: the default 32 km
    // ends mid-frame in the metre-scale pilot scene (~80+ km to the horizon
    // from 1 km up) and leaves a hard haze cutoff line.
    for (mut settings, _) in &mut camera_settings {
        settings.aerial_view_lut_max_distance = if local_scene { 200_000.0 } else { 1_000_000.0 };
    }

    // --- shared shell follows Thessa in the active view's units ---
    if let Some(center) = terrain.as_deref().and_then(|w| w.render_center) {
        set_shell_radii(
            &mut shell,
            center,
            tuning.body_radius_m as f32,
            (tuning.body_radius_m + tuning.scale_height_m * 8.0) as f32,
        );
    } else if in_pilot {
        // Metre-scale pilot scene: the preview/flight planet entity carries
        // the exact datum offset, so read it instead of recomputing.
        let center = Some(-Vec3::from_array(
            flight_runtime.terrain_origin().map(|x| x as f32),
        ));
        if let Some(center) = center {
            set_shell_radii(
                &mut shell,
                center,
                tuning.body_radius_m as f32,
                (tuning.body_radius_m + tuning.scale_height_m * 8.0) as f32,
            );
        }
    } else {
        // Compressed map scene: reuse the visual radius the map projection
        // already computed (entity scale), extended by the same 8H/R ratio.
        // The reference world is matched by id, never by name.
        for (visual, transform) in &visuals {
            if Some(visual.id) != bodies.reference {
                continue;
            }
            let visual_r = transform.scale.x.max(1e-6);
            let ratio = (tuning.scale_height_m * 8.0 / tuning.body_radius_m.max(1.0)) as f32;
            set_shell_radii(
                &mut shell,
                transform.translation,
                visual_r,
                visual_r * (1.0 + ratio),
            );
        }
    }

    monitor.record_cpu_scope(
        thessa_perf::scopes::RENDER_ATMOSPHERE,
        frame_start.elapsed().as_secs_f64(),
    );
}

fn rt_safe_stellar_diameter(diameter: f32) -> f32 {
    diameter.max(0.001) // 0.057 degrees; finite f32 solid angle
}

/// Aim one light entity at a resolved star: raw-sun illuminance for the LUT,
/// linear tint from the star's own temperature, true angular disk diameter.
#[allow(clippy::too_many_arguments)]
fn aim_light_slot(
    mut light: Mut<DirectionalLight>,
    mut transform: Mut<Transform>,
    mut disk: Mut<SunDisk>,
    star: &ResolvedStar,
    disk_intensity: f32,
) {
    let travel = (-render_direction(star.light.direction_to_star)).normalize_or_zero();
    if travel != Vec3::ZERO {
        transform.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, travel);
    }
    light.illuminance = (star.light.irradiance_w_m2 * LUX_PER_W_M2 * star.light.visibility) as f32;
    light.color = linear_color(star.light.color_rgb);
    disk.angular_size = (2.0 * star.light.angular_radius_rad) as f32;
    disk.intensity = disk_intensity * star.light.visibility.clamp(0.0, 1.0) as f32;
}

/// Drive one fixed light entity from an optional sorted slot. Missing stars
/// (single-sun systems, disabled multi-star) park the entity dark: zero
/// light, zero disk. Works for any query filter since only item types matter.
///
/// Fill slots render at reduced disk intensity: an unresolved companion at
/// physical intensity 1.0 concentrates its whole flux into subpixel pixels,
/// which the tonemapper clips and bloom inflates into a bogus second sun.
/// Scattering and illuminance are untouched — only disk pixels scale.
fn drive_slot<'w>(
    targets: impl Iterator<
        Item = (
            Mut<'w, DirectionalLight>,
            Mut<'w, Transform>,
            Mut<'w, SunDisk>,
        ),
    >,
    star: Option<&ResolvedStar>,
    disk_intensity: f32,
) {
    for (mut light, transform, mut disk) in targets {
        if let Some(star) = star {
            aim_light_slot(light, transform, disk, star, disk_intensity);
        } else {
            light.illuminance = 0.0;
            disk.intensity = 0.0;
        }
    }
}

/// Aim the shared shell entity at a planet center with view-scale radii.
#[allow(clippy::type_complexity)]
fn set_shell_radii(
    shell: &mut Query<
        (&mut Atmosphere, &mut Transform),
        (
            With<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
            Without<ThirdStarLight>,
            Without<CelestialVisual>,
        ),
    >,
    center: Vec3,
    inner_radius: f32,
    outer_radius: f32,
) {
    for (mut atmosphere, mut transform) in shell {
        atmosphere.inner_radius = inner_radius;
        atmosphere.outer_radius = outer_radius.max(inner_radius * 1.001);
        transform.translation = center;
    }
}

/// Engine Z-up inertial vector into Bevy Y-up render direction.
fn render_direction(inertial: bevy::math::DVec3) -> Vec3 {
    let v = inertial.normalize_or_zero();
    Vec3::new(v.x as f32, v.z as f32, -v.y as f32)
}

fn linear_color(tint: [f64; 3]) -> Color {
    Color::linear_rgb(
        tint[0].clamp(0.0, 1.0) as f32,
        tint[1].clamp(0.0, 1.0) as f32,
        tint[2].clamp(0.0, 1.0) as f32,
    )
}

/// GLB submeshes and terrain tiles arrive after PostStartup. Register only
/// compatible, loaded meshes, and honor terrain/vehicle participation flags.
///
/// Mesh fixups (tangents, index width) run for every loaded mesh whether or
/// not RT is active: the raster PBR path needs the same attributes, and
/// without them models render wrong from boot until the first RT toggle
/// happens to upgrade the shared asset as a side effect.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn sync_solari_meshes(
    mut commands: Commands,
    active: Res<RayTracingActive>,
    graphics: Res<GraphicsResolved>,
    mut meshes: ResMut<Assets<Mesh>>,
    pilot: Res<PilotHudState>,
    survey: Res<terrain::SurfaceSurvey>,
    mut rejected: Local<std::collections::HashSet<bevy::asset::AssetId<Mesh>>>,
    mut fixed: Local<std::collections::HashSet<bevy::asset::AssetId<Mesh>>>,
    query: Query<
        (
            Entity,
            &Mesh3d,
            Option<&CelestialVisual>,
            Option<&terrain::SurfaceTile>,
            Option<&terrain::TerrainBackdrop>,
            Option<&ThessaAtmosphereShell>,
            &InheritedVisibility,
            Option<&RaytracingMesh3d>,
        ),
        (With<MeshMaterial3d<StandardMaterial>>, Without<StarMarker>),
    >,
) {
    for (entity, mesh, body, tile, backdrop, shell, visibility, registered) in &query {
        if !rejected.contains(&mesh.0.id())
            && !fixed.contains(&mesh.0.id())
            && meshes.contains(&mesh.0)
        {
            if ensure_rt_mesh_compatible(&mut meshes, &mesh.0) {
                fixed.insert(mesh.0.id());
            } else {
                rejected.insert(mesh.0.id());
            }
        }
        if !active.0 || (!survey.active && pilot.view_mode != ClientViewMode::Pilot) {
            if registered.is_some() {
                commands.entity(entity).remove::<RaytracingMesh3d>();
            }
            continue;
        }
        // Celestial sky proxies have angular-size placement, not physical
        // distances. They must never enter the local light transport scene.
        // The atmosphere shell is a translucent scattering volume, not an
        // opaque surface: feeding it to the BLAS whites out the frame.
        if !visibility.get()
            || shell.is_some()
            || (body.is_some() && (survey.active || pilot.view_mode == ClientViewMode::Pilot))
        {
            if registered.is_some() {
                commands.entity(entity).remove::<RaytracingMesh3d>();
            }
            continue;
        }
        if registered.is_some() {
            continue;
        }
        if rejected.contains(&mesh.0.id()) {
            continue;
        }
        let is_terrain = body.is_some() || tile.is_some() || backdrop.is_some();
        let wanted = if is_terrain {
            graphics.0.rt_terrain
        } else {
            graphics.0.rt_vehicles
        };
        if wanted && meshes.contains(&mesh.0) {
            // Fixups already ran above; re-running is idempotent but skip
            // the redundant pass on assets we upgraded ourselves.
            let compatible =
                fixed.contains(&mesh.0.id()) || ensure_rt_mesh_compatible(&mut meshes, &mesh.0);
            if compatible {
                fixed.insert(mesh.0.id());
                commands
                    .entity(entity)
                    .insert(RaytracingMesh3d(mesh.0.clone()));
            } else {
                rejected.insert(mesh.0.id());
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::DVec3;
    use thessa_atmosphere::nitrogen_oxygen_optics;

    #[test]
    fn rt_companion_disk_cannot_produce_zero_solid_angle() {
        for diameter in [0.0, 1e-7, 0.0001, 0.008] {
            let size = rt_safe_stellar_diameter(diameter);
            let omega = std::f32::consts::TAU * (1.0 - (size * 0.5).cos());
            assert!(omega > 0.0);
            assert!((100_000.0 / omega).is_finite());
            if diameter >= 0.001 {
                assert_eq!(size, diameter);
            }
        }
    }

    #[test]
    fn checked_in_pressure_matches_design_reference() {
        // system.toml carries atmosphere_bar = 1.20 for Thessa; the renderer
        // must read it by body name instead of hardcoding physics.
        assert!((body_surface_pressure_pa("thessa") - 120_000.0).abs() < 1.0);
        // Unknown worlds fall back loudly rather than silently.
        assert!((body_surface_pressure_pa("no_such_world") - 120_000.0).abs() < 1.0);
    }

    #[test]
    fn star_slots_follow_irradiance_not_names() {
        // Slots sort brightest-first from live geometry: role assignment
        // never depends on star names, so any configured system works.
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = config.bake().unwrap();
        let thessa = ephemeris.body_id("thessa").unwrap();
        let origin = ephemeris
            .body_state(thessa, thessa_sim_core::SimTime::EPOCH)
            .unwrap()
            .position_inertial;
        let stars = resolve_star_lights(
            &stellar_catalog(&config, &ephemeris),
            &ephemeris,
            origin,
            thessa_sim_core::SimTime::EPOCH,
        );
        assert!(stars.len() >= 3);
        for pair in stars.windows(2) {
            assert!(
                pair[0].light.irradiance_w_m2 >= pair[1].light.irradiance_w_m2,
                "slots must sort brightest first"
            );
        }
        assert_eq!(stars[0].name, "asterion_a");
    }

    #[test]
    fn star_mesh_tints_follow_temperature() {
        let (warm_base, warm_glow) = star_mesh_colors(5100.0);
        let warm = warm_base.to_srgba();
        assert!(warm.red > warm.blue, "5100 K mesh must be warm");
        let (hot_base, _) = star_mesh_colors(8700.0);
        let hot = hot_base.to_srgba();
        assert!(hot.blue > hot.red, "8700 K mesh must be blue");
        assert!(warm_glow.red > 0.0 && warm_glow.blue > 0.0);
    }

    #[test]
    fn medium_conversion_preserves_optics() {
        let optics = nitrogen_oxygen_optics(3_200_000.0, 120_000.0, 16_300.0).unwrap();
        let medium = medium_from_optics(&optics, 64, 32, 1.0);
        assert_eq!(medium.terms.len(), 3);
        let rayleigh = &medium.terms[0];
        assert!(rayleigh.scattering.z > rayleigh.scattering.y);
        assert!(rayleigh.scattering.y > rayleigh.scattering.x);
        let mie = &medium.terms[1];
        assert!(matches!(
            mie.phase,
            PhaseFunction::Mie { asymmetry: g } if (g - 0.76).abs() < 1e-6
        ));
    }

    #[test]
    fn medium_density_scale_is_uniform_gain() {
        let optics = nitrogen_oxygen_optics(3_200_000.0, 120_000.0, 16_300.0).unwrap();
        let full = medium_from_optics(&optics, 64, 32, 1.0);
        let half = medium_from_optics(&optics, 64, 32, 0.5);
        let clear = medium_from_optics(&optics, 64, 32, 0.0);
        for (f, h, c) in full
            .terms
            .iter()
            .zip(half.terms.iter())
            .zip(clear.terms.iter())
            .map(|((f, h), c)| (f, h, c))
        {
            assert!((h.scattering - f.scattering * 0.5).length() < 1e-9);
            assert!((h.absorption - f.absorption * 0.5).length() < 1e-9);
            assert_eq!(c.scattering, Vec3::ZERO);
            assert_eq!(c.absorption, Vec3::ZERO);
        }
    }

    #[test]
    fn render_frame_maps_directions() {
        let d = render_direction(DVec3::Z);
        assert!((d - Vec3::Y).length() < 1e-6);
    }

    fn separation_deg(
        ephemeris: &BakedEphemeris,
        observer: BodyId,
        first: BodyId,
        second: BodyId,
        time: thessa_sim_core::SimTime,
    ) -> f64 {
        let origin = ephemeris
            .body_state(observer, time)
            .unwrap()
            .position_inertial;
        let a = (ephemeris.body_state(first, time).unwrap().position_inertial - origin).normalize();
        let b = (ephemeris
            .body_state(second, time)
            .unwrap()
            .position_inertial
            - origin)
            .normalize();
        a.dot(b).clamp(-1.0, 1.0).acos().to_degrees()
    }

    #[test]
    fn epoch_geometry_starts_in_syzygy() {
        // Design epoch: Thessa starts just past a Nereid eclipse of A, in
        // clear daylight (Nereid-A 8..15 deg and opening), while the outer
        // A-BC syzygy holds (A-B ~0 deg, drifting to ~90 deg by day 69).
        // B-C stays a tight ~0.25 deg pair throughout. If the epoch moves,
        // update these bands, not the renderer.
        use thessa_sim_core::SimTime;
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = config.bake().unwrap();
        let thessa = ephemeris.body_id("thessa").unwrap();
        let a = ephemeris.body_id("asterion_a").unwrap();
        let b = ephemeris.body_id("asterion_b").unwrap();
        let c = ephemeris.body_id("asterion_c").unwrap();
        let nereid = ephemeris.body_id("nereid").unwrap();
        let sep_eclipse = separation_deg(&ephemeris, thessa, nereid, a, SimTime::EPOCH);
        assert!(
            (8.0..15.0).contains(&sep_eclipse),
            "epoch must start clear of eclipse: {sep_eclipse}"
        );
        let sep_epoch = separation_deg(&ephemeris, thessa, a, b, SimTime::EPOCH);
        assert!(sep_epoch < 2.0, "epoch A-B separation: {sep_epoch}");
        let sep_d69 = separation_deg(&ephemeris, thessa, a, b, SimTime(69.0 * 86400.0));
        assert!(sep_d69 > 45.0, "day-69 A-B separation: {sep_d69}");
        for days in [0.0, 69.0, 139.0, 208.0] {
            let sep_bc = separation_deg(&ephemeris, thessa, b, c, SimTime(days * 86400.0));
            assert!(
                sep_bc < 0.5,
                "B-C pair must stay tight: {sep_bc} at day {days}"
            );
        }
    }

    #[test]
    fn stellar_disks_match_design_data() {
        use thessa_atmosphere::angular_radius;
        // Bakes the checked-in system: A must be an Earth-like disk from
        // Thessa while B stays an unresolved point. Giving both Bevy's
        // default Earth disk rendered a bogus second sun.
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = config.bake().unwrap();
        let at_epoch = thessa_sim_core::SimTime::EPOCH;
        let thessa = ephemeris.body_id("thessa").unwrap();
        let thessa_pos = ephemeris
            .body_state(thessa, at_epoch)
            .unwrap()
            .position_inertial;
        let mut ang_a = 0.0;
        let mut ang_b = 0.0;
        for star in &config.star {
            let id = ephemeris.body_id(&star.id).unwrap();
            let state = ephemeris.body_state(id, at_epoch).unwrap();
            let distance = (state.position_inertial - thessa_pos).length();
            let radius = ephemeris.body(id).unwrap().radius_m;
            match star.id.as_str() {
                "asterion_a" => ang_a = angular_radius(radius, distance),
                "asterion_b" => ang_b = angular_radius(radius, distance),
                _ => {}
            }
        }
        assert!(
            (2.0 * ang_a - 0.0099).abs() < 0.002,
            "A diameter must be Earth-like: {ang_a}"
        );
        assert!(
            ang_b < ang_a / 10.0,
            "B must be a point next to A: {ang_b} vs {ang_a}"
        );
    }
}
