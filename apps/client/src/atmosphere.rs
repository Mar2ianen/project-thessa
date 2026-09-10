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
use bevy::solari::prelude::{RaytracingMesh3d, SolariLighting};

use std::collections::HashSet;
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
/// Design reference point for Thessa air until composition lands in config.
const THESSA_AIR_TEMPERATURE_K: f64 = 278.0;
const THESSA_AIR_GAS_CONSTANT: f64 = 287.05;

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
/// a broken frame.
#[derive(Resource, Default)]
struct AtmosphereBodies {
    thessa: Option<BodyId>,
    nereid: Option<BodyId>,
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
        app.init_resource::<EclipseState>()
            .add_systems(PostStartup, atmosphere_startup)
            .add_systems(PostUpdate, update_atmosphere_visuals);
    }
}

/// Thessa surface pressure from `data/system.toml` (`atmosphere_bar` on the
/// `thessa` moon entry). Falls back to the 1.20 bar design reference so a
/// config edit never breaks startup; the fallback is logged.
fn thessa_surface_pressure_pa() -> f64 {
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
                let is_thessa = body
                    .get("id")
                    .and_then(toml::Value::as_str)
                    .is_some_and(|id| id == "thessa");
                if !is_thessa {
                    continue;
                }
                if let Some(bar) = body.get("atmosphere_bar").and_then(toml::Value::as_float) {
                    return bar * 100_000.0;
                }
            }
        }
    }
    eprintln!("[atmosphere] no atmosphere_bar for thessa; using 1.20 bar design reference");
    FALLBACK_PA
}

/// Convert shared optics into a Bevy scattering medium (same definition the
/// CPU queries and any future LUT baking consume).
fn medium_from_optics(
    optics: &AtmosphereOptics,
    falloff_resolution: u32,
    phase_resolution: u32,
) -> ScatteringMedium {
    let height = (optics.outer_radius_m - optics.inner_radius_m).max(1.0);
    let rgb = |beta: [f64; 3]| Vec3::new(beta[0] as f32, beta[1] as f32, beta[2] as f32);
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
    requested: Option<Res<GraphicsRequested>>,
    resolved: Option<Res<GraphicsResolved>>,
    rt_active: Option<Res<RayTracingActive>>,
    cameras: Query<Entity, With<Camera3d>>,
    mut media: ResMut<Assets<ScatteringMedium>>,
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
    commands.insert_resource(StellarCatalog {
        stars: stellar_catalog(&config, ephemeris),
    });
    let thessa = ephemeris.body_id("thessa");
    commands.insert_resource(AtmosphereBodies {
        thessa,
        nereid: ephemeris.body_id("nereid"),
    });

    // Shared optical definition: body radius and gravity from baked data,
    // pressure from system.toml, N2/O2 air model per design reference.
    let Some(thessa_id) = thessa else {
        eprintln!("[atmosphere] no thessa body; atmosphere disabled");
        return;
    };
    let body = match ephemeris.body(thessa_id) {
        Ok(body) => body,
        Err(error) => {
            eprintln!("[atmosphere] thessa body unreadable ({error}); disabled");
            return;
        }
    };
    let gravity = body.mu / body.radius_m.powi(2);
    let scale_height = THESSA_AIR_GAS_CONSTANT * THESSA_AIR_TEMPERATURE_K / gravity.max(0.1);
    let optics =
        match nitrogen_oxygen_optics(body.radius_m, thessa_surface_pressure_pa(), scale_height) {
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
    let medium = media.add(medium_from_optics(&optics, falloff_res, phase_res));
    commands.insert_resource(ThessaOptics {
        scale_height_m: scale_height,
        body_radius_m: body.radius_m,
        optics,
    });

    // One shell entity follows Thessa in whichever view is active; radii and
    // transform are synced every frame by `update_atmosphere_visuals`.
    let shell = commands
        .spawn((
            ThessaAtmosphereShell,
            Atmosphere {
                inner_radius: body.radius_m as f32,
                outer_radius: (body.radius_m + scale_height * 8.0) as f32,
                ground_albedo: Vec3::new(0.35, 0.36, 0.33),
                medium,
            },
            Transform::default(),
            Visibility::Visible,
            Name::new("Thessa atmosphere shell"),
        ))
        .id();

    for camera in &cameras {
        commands.entity(camera).insert((
            atmosphere_settings(resolved.atmosphere_quality),
            Exposure {
                ev100: requested.renderer.exposure_ev100,
            },
        ));
        if rt_active {
            enable_solari_camera(&mut commands, camera);
        }
    }

    // Secondary-star fill light; intensities update per frame from ephemeris.
    // Its sky disk is the companion's true angular size (a point, not a sun):
    // leaving Bevy's Earth default would draw a bogus second solar disk.
    commands.spawn((
        SecondaryStarLight,
        DirectionalLight {
            illuminance: 0.0,
            color: Color::WHITE,
            shadow_maps_enabled: false,
            ..default()
        },
        bevy::light::SunDisk {
            angular_size: 0.0004,
            intensity: 1.0,
        },
        Transform::IDENTITY,
        Name::new("Asterion B fill"),
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
        "[atmosphere] shell {:?} around thessa R={:.0} m H={:.0} m; RT {}",
        shell,
        body.radius_m,
        scale_height,
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
        Quality::High => {}
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
    let mut fixed: HashSet<Handle<Mesh>> = HashSet::new();
    for (entity, handle, is_terrain) in targets {
        let wanted = if *is_terrain {
            resolved.rt_terrain
        } else {
            resolved.rt_vehicles
        };
        if !wanted {
            continue;
        }
        if fixed.insert(handle.clone()) {
            ensure_rt_mesh_compatible(meshes, handle);
        }
        commands
            .entity(*entity)
            .insert(RaytracingMesh3d(handle.clone()));
    }
}

/// Bring one mesh asset into the Solari-compatible subset. Logs and keeps the
/// raster path untouched when a mesh cannot be upgraded.
fn ensure_rt_mesh_compatible(meshes: &mut Assets<Mesh>, handle: &Handle<Mesh>) {
    let Some(mut mesh) = meshes.get_mut(handle) else {
        return;
    };
    if !mesh.contains_attribute(Mesh::ATTRIBUTE_UV_0) {
        let count = mesh.count_vertices();
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0]; count]);
    }
    if !mesh.contains_attribute(Mesh::ATTRIBUTE_TANGENT) && mesh.generate_tangents().is_err() {
        eprintln!("[solari] tangents unavailable; mesh left raster-only");
        return;
    }
    if mesh.contains_attribute(Mesh::ATTRIBUTE_UV_1) {
        mesh.remove_attribute(Mesh::ATTRIBUTE_UV_1);
    }
    if let Some(indices) = mesh.indices_mut()
        && let bevy::mesh::Indices::U16(_) = indices
    {
        *indices = bevy::mesh::Indices::U32(indices.iter().map(|i| i as u32).collect());
    }
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
            Without<ThessaAtmosphereShell>,
        ),
    >,
    mut secondary: Query<
        (&mut DirectionalLight, &mut Transform, &mut SunDisk),
        (
            With<SecondaryStarLight>,
            Without<PrimaryStarLight>,
            Without<ThessaAtmosphereShell>,
        ),
    >,
    mut shell: Query<
        (&mut Atmosphere, &mut Transform),
        (
            With<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
        ),
    >,
    visuals: Query<
        (&Name, &Transform),
        (
            With<CelestialVisual>,
            Without<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
        ),
    >,
    planet_names: Query<(&Name, &GlobalTransform)>,
    mut monitor: ResMut<PerfMonitor>,
) {
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
    let Some(thessa_state) = bodies
        .thessa
        .and_then(|id| ephemeris.body_state(id, SimTime(sim_time)).ok())
    else {
        return;
    };
    let observer = thessa_state.position_inertial;
    let mut light_a: Option<CelestialLight> = None;
    let mut light_b: Option<CelestialLight> = None;
    for star in &catalog.stars {
        let Some(id) = star.body else { continue };
        let Ok(state) = ephemeris.body_state(id, SimTime(sim_time)) else {
            continue;
        };
        let delta = state.position_inertial - observer;
        let distance = delta.length().max(1.0);
        let light = CelestialLight {
            direction_to_star: delta / distance,
            irradiance_w_m2: irradiance_at_distance(star.luminosity_w, distance),
            color_rgb: blackbody_rgb(star.temperature_k),
            angular_radius_rad: angular_radius(star.radius_m, distance),
            visibility: 1.0,
        };
        match star.name.as_str() {
            "asterion_a" => light_a = Some(light),
            "asterion_b" => light_b = Some(light),
            _ => {}
        }
    }

    // --- eclipse: Nereid over Asterion A, continuous 0..1 ---
    let mut visibility_a = 1.0;
    if resolved.eclipses
        && let (Some(a), Some(nereid)) = (light_a.as_ref(), bodies.nereid)
        && let Ok(occ) = ephemeris.body_state(nereid, SimTime(sim_time))
    {
        let occ_delta = occ.position_inertial - observer;
        let occ_dist = occ_delta.length().max(1.0);
        let occ_radius = ephemeris.body(nereid).map(|b| b.radius_m).unwrap_or(0.0);
        visibility_a = eclipse_visibility(
            a.direction_to_star,
            a.angular_radius_rad,
            occ_delta / occ_dist,
            angular_radius(occ_radius, occ_dist),
        );
    }
    if let Some(a) = light_a.as_mut() {
        a.visibility = visibility_a;
    }
    // Ingress/egress markers with hysteresis so penumbra grazing does not spam.
    if !eclipse.eclipsed && visibility_a < 0.02 {
        eclipse.eclipsed = true;
        monitor.push_event(
            "eclipse ingress",
            Some("asterion_a behind nereid".to_string()),
        );
    } else if eclipse.eclipsed && visibility_a > 0.98 {
        eclipse.eclipsed = false;
        monitor.push_event("eclipse egress", None);
    }

    // --- direct lights: raw-sun scale so the atmosphere LUT does the filtering ---
    // Visible disks use true angular diameters (Bevy defaults to Earth's Sun
    // per light). A's disk dims with the eclipse factor: uniform dimming is
    // an approximation of the partial-disk shape, documented for later.
    if let Some(a) = light_a.as_ref() {
        for (mut light, mut transform, mut disk) in &mut primary {
            let travel = (-render_direction(a.direction_to_star)).normalize_or_zero();
            if travel != Vec3::ZERO {
                transform.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, travel);
            }
            light.illuminance = (a.irradiance_w_m2 * LUX_PER_W_M2 * a.visibility) as f32;
            light.color = linear_color(a.color_rgb);
            disk.angular_size = (2.0 * a.angular_radius_rad) as f32;
            disk.intensity = a.visibility.clamp(0.0, 1.0) as f32;
        }
    }
    if resolved.multi_star {
        if let Some(b) = light_b.as_ref() {
            for (mut light, mut transform, mut disk) in &mut secondary {
                let travel = (-render_direction(b.direction_to_star)).normalize_or_zero();
                if travel != Vec3::ZERO {
                    transform.rotation = Quat::from_rotation_arc(Vec3::NEG_Z, travel);
                }
                light.illuminance = (b.irradiance_w_m2 * LUX_PER_W_M2) as f32;
                light.color = linear_color(b.color_rgb);
                disk.angular_size = (2.0 * b.angular_radius_rad) as f32;
                disk.intensity = 1.0;
            }
        }
    } else {
        for (mut light, _, mut disk) in &mut secondary {
            light.illuminance = 0.0;
            disk.intensity = 0.0;
        }
    }

    // --- shared shell follows Thessa in the active view's units ---
    if in_pilot {
        // Metre-scale pilot scene: the preview/flight planet entity carries
        // the exact datum offset, so read it instead of recomputing.
        let mut center = None;
        for (name, transform) in &planet_names {
            if name.starts_with("PFD Thessa planet") {
                center = Some(transform.translation());
                break;
            }
        }
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
        for (name, transform) in &visuals {
            if name.as_str() != "thessa" {
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

/// Aim the shared shell entity at a planet center with view-scale radii.
#[allow(clippy::type_complexity)]
fn set_shell_radii(
    shell: &mut Query<
        (&mut Atmosphere, &mut Transform),
        (
            With<ThessaAtmosphereShell>,
            Without<PrimaryStarLight>,
            Without<SecondaryStarLight>,
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

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::DVec3;
    use thessa_atmosphere::nitrogen_oxygen_optics;

    #[test]
    fn checked_in_pressure_matches_design_reference() {
        // system.toml carries atmosphere_bar = 1.20 for Thessa; the renderer
        // must read it instead of hardcoding physics.
        assert!((thessa_surface_pressure_pa() - 120_000.0).abs() < 1.0);
    }

    #[test]
    fn medium_conversion_preserves_optics() {
        let optics = nitrogen_oxygen_optics(3_200_000.0, 120_000.0, 16_300.0).unwrap();
        let medium = medium_from_optics(&optics, 64, 32);
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
    fn render_frame_maps_directions() {
        let d = render_direction(DVec3::Z);
        assert!((d - Vec3::Y).length() < 1e-6);
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
