mod atmosphere;
mod map_ui;
mod navigation;
mod orbits;
mod perf;
mod pilot;
use atmosphere::{
    AtmospherePlugin, GraphicsRequested, GraphicsResolved, PrimaryStarLight, RayTracingActive,
};
mod terrain;
use map_ui::*;
use navigation::*;
use orbits::*;
use perf::PerfMonitorPlugin;
use pilot::*;

use std::{f32::consts::TAU, path::Path};

use bevy::render::RenderPlugin;
use bevy::render::settings::{RenderCreation, WgpuFeatures, WgpuSettings};
use bevy::solari::prelude::SolariPlugins;
use bevy::{
    asset::AssetPlugin,
    core_pipeline::tonemapping::Tonemapping,
    input::mouse::{AccumulatedMouseMotion, MouseScrollUnit, MouseWheel},
    post_process::bloom::Bloom,
    prelude::*,
    window::{PrimaryWindow, WindowResolution},
};
use bevy_brp_extras::BrpExtrasPlugin;
use serde::Deserialize;
use thessa_graphics::{Capabilities, RequestedGraphics, ResolvedGraphicsSettings};
use thessa_sim_core::{BakedEphemeris, BodyId, KeplerOrbit, SimTime, SystemConfig};

const DISTANCE_UNIT_M: f64 = 125_000_000.0;
const NEREID_RADIUS_M: f64 = 68_000_000.0;
const MOUSE_ORBIT_SENSITIVITY: f32 = 0.005;
const MOUSE_PAN_SENSITIVITY: f32 = 0.0018;
const MOUSE_ZOOM_SENSITIVITY: f32 = 0.09;
const NEREID_RING_INNER_RADIUS_M: f64 = 85_000_000.0;
const NEREID_RING_OUTER_RADIUS_M: f64 = 140_000_000.0;
// Bevy's UV sphere is generated with its texture poles on local +Z, while
// the rendered reference frame is Y-up. Keep the planet's spin axis physical
// (world +Y) instead of accidentally rotating the gas bands around a side axis.
const SPHERE_POLE_TO_WORLD_UP: Quat = Quat::from_xyzw(
    -std::f32::consts::FRAC_1_SQRT_2,
    0.0,
    0.0,
    std::f32::consts::FRAC_1_SQRT_2,
);

#[derive(Clone, Copy, PartialEq, Eq)]
enum MapMode {
    SystemOverview,
    Asterion,
    Nereid,
    Orthea,
    Vesper,
    Binary,
}

impl MapMode {
    fn focus_name(self) -> &'static str {
        match self {
            Self::SystemOverview => "system_barycenter",
            Self::Asterion => "asterion_a",
            Self::Nereid => "nereid",
            Self::Orthea => "orthea",
            Self::Vesper => "vesper",
            Self::Binary => "bc_barycenter",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::SystemOverview => "SYSTEM OVERVIEW",
            Self::Asterion => "ASTERION A SYSTEM",
            Self::Nereid => "NEREID MOON SYSTEM",
            Self::Orthea => "ORTHEA MOON SYSTEM",
            Self::Vesper => "VESPER MOON SYSTEM",
            Self::Binary => "ASTERION B/C BINARY",
        }
    }
}

#[derive(Resource)]
struct MapState {
    mode: MapMode,
    focus: BodyId,
    selected: BodyId,
}

fn main() {
    // Requested graphics first: the RT decision below must happen before
    // plugins register, while adapter capabilities only exist post-init.
    // Unknown capability + explicit RT request = fail fast at device creation
    // with a clear note; `auto` stays raster (see thessa-graphics).
    let graphics_text = std::fs::read_to_string("graphics.toml")
        .unwrap_or_else(|_| include_str!("../../../graphics.toml").to_string());
    let requested = RequestedGraphics::from_toml(&graphics_text).unwrap_or_else(|error| {
        eprintln!("[graphics] {error}; falling back to defaults");
        RequestedGraphics::default()
    });
    let resolved = ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::unknown());
    let rt_active = resolved.ray_tracing.is_active();
    if rt_active {
        eprintln!("[graphics] experimental Solari RT path requested; needs RT-capable Vulkan");
    }

    let mut app = App::new();
    let mut default_plugins = DefaultPlugins
        .set(AssetPlugin {
            // Bevy resolves the asset root relative to this package's
            // manifest directory: apps/client -> workspace/assets.
            file_path: "../../assets".into(),
            ..default()
        })
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "Project Thessa — Nereid System".into(),
                resolution: WindowResolution::new(1280, 720),
                present_mode: if resolved.vsync {
                    bevy::window::PresentMode::AutoVsync
                } else {
                    bevy::window::PresentMode::AutoNoVsync
                },
                ..default()
            }),
            ..default()
        });
    if rt_active {
        // `WgpuSettings` travels inside `RenderPlugin::render_creation` in
        // 0.19. Forcing RT features makes device creation fail fast on
        // incapable hardware (explicit opt-in only, never `auto`).
        default_plugins = default_plugins.set(RenderPlugin {
            render_creation: RenderCreation::Automatic(Box::new(WgpuSettings {
                features: WgpuFeatures::default() | SolariPlugins::required_wgpu_features(),
                ..default()
            })),
            ..default()
        });
    }

    app.add_plugins(default_plugins);
    // Solari selects deferred opaque materials globally, including while
    // disabled. Every camera therefore retains a valid deferred raster path.
    // Plugin finish checks device features; unsupported GPUs keep raster.
    app.add_plugins(SolariPlugins);
    app.insert_resource(GraphicsRequested(requested))
        .insert_resource(GraphicsResolved(resolved))
        .insert_resource(RayTracingActive(rt_active))
        .add_plugins(OrbitGizmoPlugin)
        .add_plugins(PilotHudPlugin)
        .add_plugins(PerfMonitorPlugin)
        .add_plugins(AtmospherePlugin)
        .add_plugins(terrain::TerrainPlugin)
        .add_plugins(BrpExtrasPlugin::default())
        .insert_resource(SimulationClock::default())
        .insert_resource(NavigationState::default())
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                update_map_ui_input_state,
                preview_input,
                select_body_with_pointer,
                update_camera,
                update_starfield,
                update_celestial_visuals,
                draw_orbits,
                update_hud,
            )
                .chain()
                .after(pilot::PilotUpdate),
        )
        .run();
}

#[derive(Resource)]
struct SimulationClock {
    sim_seconds: f64,
    multiplier: f64,
    paused: bool,
}

impl Default for SimulationClock {
    fn default() -> Self {
        Self {
            sim_seconds: 0.0,
            multiplier: 1.0,
            paused: false,
        }
    }
}

#[derive(Resource)]
struct RuntimeEphemeris {
    ephemeris: BakedEphemeris,
}

#[derive(Component)]
struct CelestialVisual {
    id: BodyId,
}

#[derive(Component)]
struct StarMarker {
    direction: Vec3,
    radius_factor: f32,
    size: f32,
}

#[derive(Component)]
struct OrbitCamera {
    orbit: Quat,
    distance: f32,
    target: Vec3,
}

#[derive(Component)]
struct Hud;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    rt_active: Option<Res<RayTracingActive>>,
) {
    let rt_active = rt_active.is_some_and(|flag| flag.0);
    let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
        .expect("the checked-in system configuration must parse");
    let ephemeris = config
        .bake()
        .expect("the checked-in system configuration must bake");
    let system_focus = ephemeris
        .body_id("nereid")
        .expect("the baked system must contain Nereid");
    let flight_body = ephemeris
        .body_id("thessa")
        .expect("the baked system must contain the playable world");
    commands.insert_resource(MapState {
        mode: MapMode::Nereid,
        focus: system_focus,
        selected: ephemeris.body_id("thessa").expect("starting world"),
    });

    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.18, 0.22, 0.32),
        brightness: 55.0,
        ..default()
    });

    commands.spawn((
        Camera3d::default(),
        // Keep these across RT toggles: required components are not removed
        // automatically with SolariLighting, and deferred needs MSAA off.
        Msaa::Off,
        bevy::core_pipeline::prepass::DepthPrepass,
        bevy::core_pipeline::prepass::DeferredPrepass,
        Projection::Perspective(PerspectiveProjection {
            far: 1_000_000.0,
            ..default()
        }),
        Camera {
            clear_color: ClearColorConfig::Custom(Color::srgb(0.001, 0.002, 0.008)),
            ..default()
        },
        Tonemapping::TonyMcMapface,
        Bloom {
            intensity: 0.14,
            ..Bloom::NATURAL
        },
        OrbitCamera {
            orbit: Quat::from_rotation_y(0.42) * Quat::from_rotation_x(-0.72),
            distance: 420.0,
            target: Vec3::ZERO,
        },
        Transform::from_xyz(129.0, 276.0, 287.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    // Asterion's illumination is represented by direction, not by a fake
    // nearby star whose size would make the local map physically misleading.
    // Neutral spawn values: the atmosphere plugin derives exact illuminance,
    // tint and disk size from ephemeris geometry on the first frame.
    commands.spawn((
        PrimaryStarLight,
        DirectionalLight {
            illuminance: 88_000.0,
            color: Color::WHITE,
            shadow_maps_enabled: !rt_active,
            ..default()
        },
        bevy::light::SunDisk::OFF,
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.12, -0.70, -0.24)),
    ));

    // The UV sphere duplicates the longitude seam. An icosphere shares the
    // vertices at that seam, which makes a 0/1 texture transition visible as
    // a jagged meridian on close-up planets.
    let sphere_mesh = meshes.add(Sphere::new(1.0).mesh().uv(192, 96));
    // Star spheres are tinted from measured effective temperatures, never
    // hardcoded per system: any configured star list renders correctly.
    let mut star_materials: std::collections::HashMap<String, Handle<StandardMaterial>> =
        std::collections::HashMap::new();
    for star in &config.star {
        let (base_color, emissive) = atmosphere::star_mesh_colors(star.temperature_k);
        star_materials.insert(
            star.id.clone(),
            materials.add(StandardMaterial {
                base_color,
                emissive,
                unlit: true,
                ..default()
            }),
        );
    }
    let visual_materials = build_visual_materials(&mut materials, &asset_server);
    for body in ephemeris.bodies.iter().filter(|body| body.radius_m > 0.0) {
        spawn_visual_body(
            &mut commands,
            sphere_mesh.clone(),
            material_for_body(body, &visual_materials, &star_materials),
            &ephemeris,
            system_focus,
            body,
        );
    }

    spawn_pilot_preview(
        &mut commands,
        sphere_mesh.clone(),
        visual_materials.thessa.clone(),
        &asset_server,
        ephemeris
            .body(flight_body)
            .expect("the playable world body must exist")
            .radius_m,
    );
    // Background starfield variety follows the configured stars: brightest
    // for the sparse bright points, hottest for the blue ones.
    let brightest = config
        .star
        .iter()
        .max_by(|a, b| a.luminosity_solar.total_cmp(&b.luminosity_solar))
        .map(|star| star.id.clone())
        .and_then(|id| star_materials.get(&id).cloned());
    let hottest = config
        .star
        .iter()
        .max_by(|a, b| a.temperature_k.total_cmp(&b.temperature_k))
        .map(|star| star.id.clone())
        .and_then(|id| star_materials.get(&id).cloned());
    if let (Some(bright), Some(blue)) = (brightest, hottest) {
        spawn_starfield(&mut commands, sphere_mesh, bright, blue, &mut materials);
    } else {
        eprintln!("[client] no configured stars; starfield skipped");
    }
    let flight_runtime = PilotFlightRuntime::new(&ephemeris, flight_body)
        .expect("the playable X-15 flight model must initialize")
        .with_trace(Path::new(&format!(
            "target/flight-traces/pilot-flight-{}-{}.csv",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            std::process::id(),
        )));
    commands.insert_resource(RuntimeEphemeris { ephemeris });
    commands.insert_resource(flight_runtime);
    spawn_hud(&mut commands);
}

/// Visual material parameters: data-driven phase 1 over compiled builtins.
/// `data/materials.toml` overlays the builtins field by field; renderer data,
/// never physics. Body-to-material assignment stays in `material_for_body`.
#[derive(Debug, Clone, PartialEq)]
struct MaterialParams {
    base_color_srgb: [f32; 3],
    albedo_texture: Option<String>,
    normal_texture: Option<String>,
    metallic_roughness_texture: Option<String>,
    emissive_linear_rgb: Option<[f32; 3]>,
    perceptual_roughness: f32,
    metallic: f32,
}

/// Partial file override: every field optional, `None` keeps the builtin.
/// Clearing a texture back to "no map" is not expressible yet.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
struct MaterialOverride {
    base_color_srgb: Option<[f32; 3]>,
    albedo_texture: Option<String>,
    normal_texture: Option<String>,
    metallic_roughness_texture: Option<String>,
    emissive_linear_rgb: Option<[f32; 3]>,
    perceptual_roughness: Option<f32>,
    metallic: Option<f32>,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct MaterialsFile {
    materials: std::collections::HashMap<String, MaterialOverride>,
}

impl MaterialParams {
    fn new(base_color_srgb: [f32; 3], perceptual_roughness: f32) -> Self {
        Self {
            base_color_srgb,
            albedo_texture: None,
            normal_texture: None,
            metallic_roughness_texture: None,
            emissive_linear_rgb: None,
            perceptual_roughness,
            metallic: 0.0,
        }
    }

    fn albedo(mut self, path: &str) -> Self {
        self.albedo_texture = Some(path.to_string());
        self
    }

    fn normal(mut self, path: &str) -> Self {
        self.normal_texture = Some(path.to_string());
        self
    }

    fn metallic_roughness(mut self, path: &str) -> Self {
        self.metallic_roughness_texture = Some(path.to_string());
        self
    }

    fn emissive(mut self, rgb: [f32; 3]) -> Self {
        self.emissive_linear_rgb = Some(rgb);
        self
    }

    fn metal(mut self, metallic: f32) -> Self {
        self.metallic = metallic;
        self
    }

    fn overlay(&mut self, file: &MaterialOverride) {
        if let Some(color) = file.base_color_srgb {
            self.base_color_srgb = color;
        }
        if let Some(path) = file.albedo_texture.clone() {
            self.albedo_texture = Some(path);
        }
        if let Some(path) = file.normal_texture.clone() {
            self.normal_texture = Some(path);
        }
        if let Some(path) = file.metallic_roughness_texture.clone() {
            self.metallic_roughness_texture = Some(path);
        }
        if let Some(emissive) = file.emissive_linear_rgb {
            self.emissive_linear_rgb = Some(emissive);
        }
        if let Some(roughness) = file.perceptual_roughness {
            self.perceptual_roughness = roughness;
        }
        if let Some(metallic) = file.metallic {
            self.metallic = metallic;
        }
    }

    fn to_standard(&self, assets: &AssetServer) -> StandardMaterial {
        StandardMaterial {
            base_color: Color::srgb(
                self.base_color_srgb[0],
                self.base_color_srgb[1],
                self.base_color_srgb[2],
            ),
            base_color_texture: self
                .albedo_texture
                .clone()
                .map(|path| load_albedo_image(assets, path)),
            normal_map_texture: self
                .normal_texture
                .clone()
                .map(|path| load_linear_image(assets, path)),
            metallic_roughness_texture: self
                .metallic_roughness_texture
                .clone()
                .map(|path| load_linear_image(assets, path)),
            emissive: self
                .emissive_linear_rgb
                .map(|rgb| LinearRgba::rgb(rgb[0], rgb[1], rgb[2]))
                .unwrap_or(LinearRgba::BLACK),
            perceptual_roughness: self.perceptual_roughness,
            metallic: self.metallic,
            ..default()
        }
    }
}

/// Anisotropic sampler for grazing-angle planet views: without it the 4K
/// global maps blur into an "obviously low-res" look near the horizon.
fn aniso_sampler() -> bevy::image::ImageSampler {
    bevy::image::ImageSampler::Descriptor(bevy::image::ImageSamplerDescriptor {
        anisotropy_clamp: 8,
        ..bevy::image::ImageSamplerDescriptor::linear()
    })
}

fn load_linear_image(assets: &AssetServer, path: impl Into<String>) -> Handle<Image> {
    assets
        .load_builder()
        .with_settings(|settings: &mut bevy::image::ImageLoaderSettings| {
            settings.is_srgb = false;
            settings.sampler = aniso_sampler();
        })
        .load(path.into())
}

fn load_albedo_image(assets: &AssetServer, path: impl Into<String>) -> Handle<Image> {
    assets
        .load_builder()
        .with_settings(|settings: &mut bevy::image::ImageLoaderSettings| {
            settings.is_srgb = true;
            settings.sampler = aniso_sampler();
        })
        .load(path.into())
}

fn builtin_material_params() -> std::collections::HashMap<String, MaterialParams> {
    [
        (
            "khepri",
            MaterialParams::new([0.60, 0.22, 0.08], 0.92).albedo("textures/khepri-surface-v1.png"),
        ),
        (
            "nereid",
            MaterialParams::new([0.95, 0.82, 0.58], 0.82)
                .albedo("textures/nereid-atmosphere-v1.png"),
        ),
        (
            "thessa",
            MaterialParams::new([1.0, 1.0, 1.0], 0.92)
                .albedo("worlds/thessa-v3/albedo.png")
                .normal("worlds/thessa-v3/normal.png")
                .metallic_roughness("worlds/thessa-v3/roughness.png"),
        ),
        (
            "pelagos",
            // Open water: low roughness for a sun-glint response. No
            // metallic: water is dielectric, specular comes from F0.
            MaterialParams::new([0.06, 0.25, 0.44], 0.18).albedo("textures/pelagos-surface-v1.png"),
        ),
        (
            "borea",
            MaterialParams::new([0.78, 0.84, 0.88], 0.98).albedo("textures/borea-surface-v1.png"),
        ),
        (
            "orthea",
            MaterialParams::new([0.48, 0.53, 0.58], 0.96).albedo("textures/orthea-surface-v1.png"),
        ),
        (
            "vesper",
            MaterialParams::new([0.53, 0.70, 0.78], 0.62)
                .albedo("textures/vesper-atmosphere-v1.png"),
        ),
        (
            "janus",
            MaterialParams::new([0.70, 0.62, 0.48], 0.94).albedo("textures/janus-surface-v1.png"),
        ),
        (
            "volcanic",
            // Lava-bearing crust glows on its own; keep the albedo dark so
            // the emissive reads as heat rather than paint.
            MaterialParams::new([0.30, 0.08, 0.03], 0.90).emissive([2.2, 0.35, 0.04]),
        ),
        (
            "metal",
            MaterialParams::new([0.45, 0.38, 0.28], 0.45).metal(0.9),
        ),
        ("ice", MaterialParams::new([0.56, 0.70, 0.78], 0.32)),
        ("minor", MaterialParams::new([0.40, 0.43, 0.46], 0.98)),
    ]
    .into_iter()
    .map(|(key, params)| (key.to_string(), params))
    .collect()
}

fn load_material_overrides() -> std::collections::HashMap<String, MaterialOverride> {
    match toml::from_str::<MaterialsFile>(include_str!("../../../data/materials.toml")) {
        Ok(file) => file.materials,
        Err(error) => {
            eprintln!("[materials] {error}; using compiled builtins");
            std::collections::HashMap::new()
        }
    }
}

fn build_visual_materials(
    materials: &mut Assets<StandardMaterial>,
    asset_server: &AssetServer,
) -> VisualMaterials {
    let builtin = builtin_material_params();
    let overrides = load_material_overrides();
    let mut build = |key: &str| {
        let mut params = builtin
            .get(key)
            .cloned()
            .expect("builtin material table covers every visual slot");
        if let Some(file) = overrides.get(key) {
            params.overlay(file);
        }
        materials.add(params.to_standard(asset_server))
    };
    VisualMaterials {
        khepri: build("khepri"),
        nereid: build("nereid"),
        thessa: build("thessa"),
        pelagos: build("pelagos"),
        borea: build("borea"),
        orthea: build("orthea"),
        vesper: build("vesper"),
        janus: build("janus"),
        volcanic: build("volcanic"),
        metal: build("metal"),
        ice: build("ice"),
        minor: build("minor"),
    }
}

struct VisualMaterials {
    khepri: Handle<StandardMaterial>,
    nereid: Handle<StandardMaterial>,
    thessa: Handle<StandardMaterial>,
    pelagos: Handle<StandardMaterial>,
    borea: Handle<StandardMaterial>,
    orthea: Handle<StandardMaterial>,
    vesper: Handle<StandardMaterial>,
    janus: Handle<StandardMaterial>,
    volcanic: Handle<StandardMaterial>,
    metal: Handle<StandardMaterial>,
    ice: Handle<StandardMaterial>,
    minor: Handle<StandardMaterial>,
}

fn material_for_body(
    body: &thessa_sim_core::BakedBody,
    materials: &VisualMaterials,
    star_materials: &std::collections::HashMap<String, Handle<StandardMaterial>>,
) -> Handle<StandardMaterial> {
    // Configured stars first (tints from measured temperatures); the rest is
    // game content mapping textures to worlds.
    if let Some(handle) = star_materials.get(&body.name) {
        return handle.clone();
    }
    match body.name.as_str() {
        "khepri" => materials.khepri.clone(),
        "nereid" => materials.nereid.clone(),
        "thessa" => materials.thessa.clone(),
        "pelagos" => materials.pelagos.clone(),
        "borea" => materials.borea.clone(),
        "orthea" => materials.orthea.clone(),
        "vesper" => materials.vesper.clone(),
        "janus" => materials.janus.clone(),
        "pyra" => materials.volcanic.clone(),
        "auron" => materials.metal.clone(),
        "mira" | "mora" | "skadi" => materials.ice.clone(),
        _ => materials.minor.clone(),
    }
}

fn spawn_visual_body(
    commands: &mut Commands,
    sphere_mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
    ephemeris: &BakedEphemeris,
    focus: BodyId,
    body: &thessa_sim_core::BakedBody,
) {
    let position = local_position(ephemeris, focus, body.id, SimTime::EPOCH);
    let radius = visual_radius_for_mode(body.radius_m, MapMode::SystemOverview);
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(material),
        Transform::from_translation(position).with_scale(Vec3::splat(radius)),
        Name::new(body.name.clone()),
        CelestialVisual { id: body.id },
        bevy::light::NotShadowCaster,
    ));
}

fn spawn_starfield(
    commands: &mut Commands,
    sphere_mesh: Handle<Mesh>,
    star_material: Handle<StandardMaterial>,
    blue_star_material: Handle<StandardMaterial>,
    materials: &mut Assets<StandardMaterial>,
) {
    let dim_star_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.52, 0.63, 0.82),
        emissive: LinearRgba::rgb(5.0, 8.0, 18.0),
        unlit: true,
        ..default()
    });
    let mut seed = 0x5EED_2026_u32;
    for index in 0..220 {
        let direction = loop {
            let candidate = Vec3::new(
                next_unit(&mut seed) * 2.0 - 1.0,
                next_unit(&mut seed) * 2.0 - 1.0,
                next_unit(&mut seed) * 2.0 - 1.0,
            );
            if candidate.length_squared() > 0.2 {
                break candidate.normalize();
            }
        };
        let distance = 52.0 + next_unit(&mut seed) * 12.0;
        let size = 0.014 + next_unit(&mut seed) * 0.035;
        let material = match index % 11 {
            0 => star_material.clone(),
            1 => blue_star_material.clone(),
            _ => dim_star_material.clone(),
        };
        commands.spawn((
            Mesh3d(sphere_mesh.clone()),
            MeshMaterial3d(material),
            Transform::from_translation(direction * distance).with_scale(Vec3::splat(size)),
            StarMarker {
                direction,
                radius_factor: distance / 60.0,
                size,
            },
        ));
    }
}

fn update_starfield(
    camera: Single<&Transform, (With<Camera3d>, Without<StarMarker>)>,
    survey: Res<terrain::SurfaceSurvey>,
    pilot: Res<PilotHudState>,
    mut stars: Query<(&mut Transform, &StarMarker, &mut Visibility)>,
) {
    // Local stars must lie behind terrain, not 500 metres in front of it.
    let local = survey.active || pilot.view_mode == ClientViewMode::Pilot;
    let radius = if local { 5_000_000.0 } else { 500.0 };
    for (mut transform, star, mut visibility) in &mut stars {
        *visibility = if survey.active {
            Visibility::Hidden
        } else {
            Visibility::Visible
        };
        transform.translation = camera.translation + star.direction * radius * star.radius_factor;
        transform.scale = Vec3::splat(star.size * radius / 60.0);
    }
}

fn update_celestial_visuals(
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    navigation: Res<NavigationState>,
    pilot: Option<Res<PilotHudState>>,
    cameras: Query<&Transform, (With<Camera3d>, Without<CelestialVisual>)>,
    mut visuals: Query<(&mut Transform, &mut Visibility, &CelestialVisual)>,
) {
    let time = SimTime(clock.sim_seconds);
    let pilot_active = pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    // Floating origin: all map visuals are expressed relative to the follow
    // anchor in f64, then narrowed to f32. Without this the barycentric
    // offsets (up to ~50k render units in SystemOverview, ~900 in Nereid
    // scope) quantize to hundreds of kilometres in f32 and the planets
    // visibly jitter while time runs.
    let anchor = camera_anchor_f64(&runtime.ephemeris, &map, &navigation, time);
    let camera_pos = cameras
        .iter()
        .next()
        .map(|t| t.translation)
        .unwrap_or(Vec3::ZERO);
    for (mut transform, mut visibility, visual) in &mut visuals {
        if pilot_active {
            *visibility = Visibility::Hidden;
            continue;
        }
        if let Some(position) =
            map_position_anchored(&runtime.ephemeris, &map, visual.id, time, anchor)
        {
            let body = runtime
                .ephemeris
                .body(visual.id)
                .expect("visual body descriptor");
            transform.translation = position;
            let dist = position.distance(camera_pos);
            transform.scale = Vec3::splat(visual_radius_for_view(body.radius_m, map.mode, dist));
            transform.rotation = visual_rotation(body, time);
            *visibility = Visibility::Visible;
        } else {
            *visibility = Visibility::Hidden;
        }
    }
}

fn map_position(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
) -> Option<Vec3> {
    is_visible_in_view(ephemeris, map, id).then(|| local_position(ephemeris, map.focus, id, time))
}

fn map_position_f64(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
) -> Option<bevy::math::DVec3> {
    if !is_visible_in_view(ephemeris, map, id) {
        return None;
    }
    local_position_f64(ephemeris, map.focus, id, time)
}

fn camera_anchor_f64(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    navigation: &NavigationState,
    time: SimTime,
) -> bevy::math::DVec3 {
    // Anticipate an in-flight FrameSelected request so visuals and camera
    // rebase on the same frame; otherwise the world pops a frame late.
    let following =
        navigation.follow_selected || navigation.request == NavigationRequest::FrameSelected;
    if following {
        map_position_f64(ephemeris, map, map.selected, time).unwrap_or(bevy::math::DVec3::ZERO)
    } else {
        bevy::math::DVec3::ZERO
    }
}

fn map_position_anchored(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
    anchor: bevy::math::DVec3,
) -> Option<Vec3> {
    map_position_f64(ephemeris, map, id, time).map(|p| (p - anchor).as_vec3())
}

fn is_visible_in_view(ephemeris: &BakedEphemeris, map: &MapState, id: BodyId) -> bool {
    if map.mode == MapMode::SystemOverview {
        return true;
    }
    let mut current = Some(id);
    while let Some(current_id) = current {
        if current_id == map.focus {
            return true;
        }
        current = ephemeris.body(current_id).ok().and_then(|body| body.parent);
    }
    false
}

fn local_position(ephemeris: &BakedEphemeris, focus: BodyId, id: BodyId, time: SimTime) -> Vec3 {
    local_position_f64(ephemeris, focus, id, time)
        .expect("visual body state must remain evaluable")
        .as_vec3()
}

fn local_position_f64(
    ephemeris: &BakedEphemeris,
    focus: BodyId,
    id: BodyId,
    time: SimTime,
) -> Option<bevy::math::DVec3> {
    let focus_state = ephemeris.body_state(focus, time).ok()?;
    let state = ephemeris.body_state(id, time).ok()?;
    Some(render_position_f64(
        state.position_inertial - focus_state.position_inertial,
    ))
}

// Rotate the engine's Z-up frame into Bevy's Y-up frame before narrowing to f32.
fn render_position(relative_m: bevy::math::DVec3) -> Vec3 {
    render_position_f64(relative_m).as_vec3()
}

fn render_position_f64(relative_m: bevy::math::DVec3) -> bevy::math::DVec3 {
    let relative = relative_m / DISTANCE_UNIT_M;
    bevy::math::DVec3::new(relative.x, relative.z, -relative.y)
}

fn visual_rotation(body: &thessa_sim_core::BakedBody, time: SimTime) -> Quat {
    if body.name == "thessa" {
        return Quat::from_rotation_y((time.0 * std::f64::consts::TAU / (80.0 * 3600.0)) as f32)
            * SPHERE_POLE_TO_WORLD_UP;
    }
    let tilt = Quat::from_rotation_z(body.axial_tilt_rad as f32);
    if body.tidal_lock {
        if let Some(orbit) = body.orbit
            && let Ok((relative_position, _)) = orbit.state_relative_at(time)
        {
            let relative = render_position(relative_position);
            let flat = Vec2::new(relative.x, relative.z);
            if flat.length_squared() > 1.0e-8 {
                // Local +Z is the body's facing meridian. Point it at the
                // parent while retaining the configured axial tilt.
                let yaw = (-flat.x).atan2(-flat.y);
                return tilt * Quat::from_rotation_y(yaw) * SPHERE_POLE_TO_WORLD_UP;
            }
        }
        return tilt * SPHERE_POLE_TO_WORLD_UP;
    }

    let period_s = body.rotation_period_s.unwrap_or(86_400.0).max(1.0);
    let angle = (TAU as f64 * time.seconds() / period_s) as f32;
    tilt * Quat::from_rotation_y(angle) * SPHERE_POLE_TO_WORLD_UP
}

fn visual_radius_for_mode(radius_m: f64, mode: MapMode) -> f32 {
    if radius_m <= 0.0 {
        return 0.06;
    }
    let ratio = (radius_m / NEREID_RADIUS_M).max(1.0e-6);
    let minimum = match mode {
        MapMode::SystemOverview => 0.16,
        MapMode::Asterion => 0.10,
        _ => 0.075,
    };
    let maximum = match mode {
        MapMode::SystemOverview => 4.5,
        _ => 2.5,
    };
    (1.35 * ratio.powf(0.42)).clamp(minimum, maximum) as f32
}

fn true_radius_units(radius_m: f64) -> f32 {
    if radius_m <= 0.0 {
        return 0.0;
    }
    (radius_m / DISTANCE_UNIT_M) as f32
}

/// Proximity blend from readability exaggeration (far) to true scale (close).
///
/// Far views keep `visual_radius_for_mode` so small moons stay selectable;
/// when the camera closes in the mesh shrinks/grows toward its physical
/// angular size so fly-through reads as approach rather than a clamped bill.
/// Inside the body the true scale wins entirely, which pairs with the HUD
/// proximity notice.
fn visual_radius_for_view(radius_m: f64, mode: MapMode, camera_dist: f32) -> f32 {
    let exaggerated = visual_radius_for_mode(radius_m, mode);
    let physical = true_radius_units(radius_m);
    if !(camera_dist.is_finite()) || physical <= 0.0 {
        return exaggerated;
    }
    // Stars are physically larger than their clamped overview icons; planets
    // are physically smaller. Blend both directions with the same curve.
    let span = (exaggerated * 8.0 + physical * 4.0).max(1.0e-6);
    let t = ((camera_dist - physical * 1.2) / span).clamp(0.0, 1.0);
    // Smoothstep keeps the far field stable and the last approach continuous.
    let smooth = t * t * (3.0 - 2.0 * t);
    physical + (exaggerated - physical) * smooth
}

fn format_sim_time(seconds: f64) -> (u64, u64, u64, u64) {
    let total_seconds = seconds.max(0.0).floor() as u64;
    let days = total_seconds / 86_400;
    let hours = (total_seconds % 86_400) / 3_600;
    let minutes = (total_seconds % 3_600) / 60;
    let seconds = total_seconds % 60;
    (days, hours, minutes, seconds)
}

fn next_unit(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    (*seed >> 8) as f32 / (u32::MAX >> 8) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn render_frame_preserves_distance_and_maps_orbital_normal_up() {
        let point = bevy::math::DVec3::new(3.0, 4.0, 12.0) * DISTANCE_UNIT_M;
        assert_eq!(render_position(point), Vec3::new(3.0, 12.0, -4.0));
        assert_eq!(render_position(point).length(), 13.0);
        assert_eq!(
            render_position(bevy::math::DVec3::Z * DISTANCE_UNIT_M),
            Vec3::Y
        );
    }

    #[test]
    fn visual_radius_preserves_body_order() {
        assert!(
            visual_radius_for_mode(68_000_000.0, MapMode::Nereid)
                > visual_radius_for_mode(3_200_000.0, MapMode::Nereid)
        );
        assert!(
            visual_radius_for_mode(3_200_000.0, MapMode::Nereid)
                > visual_radius_for_mode(2_100_000.0, MapMode::Nereid)
        );
        assert!(
            visual_radius_for_mode(2_100_000.0, MapMode::Nereid)
                > visual_radius_for_mode(12_500.0, MapMode::Nereid)
        );
    }

    #[test]
    fn visual_spin_keeps_uv_sphere_pole_on_world_up_axis() {
        let body = thessa_sim_core::BakedBody::fixed(BodyId(0), "test", 1.0, 1.0);
        let at_epoch = visual_rotation(&body, SimTime::EPOCH);
        let after_half_turn = visual_rotation(&body, SimTime(43_200.0));

        assert!((at_epoch * Vec3::Z - Vec3::Y).length() < 1.0e-5);
        assert!((after_half_turn * Vec3::Z - Vec3::Y).length() < 1.0e-5);
    }

    #[test]
    fn simulation_time_format_is_stable() {
        assert_eq!(format_sim_time(90_061.0), (1, 1, 1, 1));
        assert_eq!(format_sim_time(-1.0), (0, 0, 0, 0));
    }

    #[test]
    fn map_modes_cover_the_documented_body_hierarchy() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("the checked-in system configuration must parse");
        let ephemeris = config.bake().expect("the checked-in system must bake");
        let physical_body_count = |map: &MapState| {
            ephemeris
                .bodies
                .iter()
                .filter(|body| body.radius_m > 0.0)
                .filter(|body| is_visible_in_view(&ephemeris, map, body.id))
                .count()
        };
        for (mode, expected_count) in [
            (MapMode::SystemOverview, 22),
            (MapMode::Asterion, 18),
            (MapMode::Nereid, 9),
            (MapMode::Orthea, 4),
            (MapMode::Vesper, 3),
            (MapMode::Binary, 4),
        ] {
            let focus = ephemeris
                .body_id(mode.focus_name())
                .expect("map mode focus must exist");
            assert_eq!(
                physical_body_count(&MapState {
                    mode,
                    focus,
                    selected: focus,
                }),
                expected_count
            );
        }
    }

    #[test]
    fn tab_cycles_visible_bodies_in_both_directions() {
        let bodies = [BodyId(2), BodyId(4), BodyId(9)];
        assert_eq!(cycle_body(&bodies, BodyId(2), false), Some(BodyId(4)));
        assert_eq!(cycle_body(&bodies, BodyId(2), true), Some(BodyId(9)));
        assert_eq!(cycle_body(&bodies, BodyId(9), false), Some(BodyId(2)));
    }

    #[test]
    fn materials_file_covers_all_builtin_keys() {
        let builtin = builtin_material_params();
        let file: MaterialsFile = toml::from_str(include_str!("../../../data/materials.toml"))
            .expect("materials.toml must parse");
        assert_eq!(file.materials.len(), builtin.len());
        for key in builtin.keys() {
            assert!(
                file.materials.contains_key(key),
                "materials.toml must define {key}"
            );
        }
        let thessa = &file.materials["thessa"];
        assert_eq!(
            thessa.albedo_texture.as_deref(),
            Some("worlds/thessa-v3/albedo.png")
        );
        assert_eq!(
            file.materials["volcanic"].emissive_linear_rgb,
            Some([2.2, 0.35, 0.04])
        );
        assert_eq!(file.materials["metal"].metallic, Some(0.9));
        assert_eq!(file.materials["pelagos"].perceptual_roughness, Some(0.18));
    }

    #[test]
    fn material_override_merges_per_field() {
        let mut params = builtin_material_params()["pelagos"].clone();
        let before = params.clone();
        params.overlay(&MaterialOverride {
            perceptual_roughness: Some(0.5),
            ..Default::default()
        });
        assert_eq!(params.perceptual_roughness, 0.5);
        assert_eq!(params.base_color_srgb, before.base_color_srgb);
        assert_eq!(params.albedo_texture, before.albedo_texture);
        assert_eq!(params.metallic, before.metallic);
    }

    #[test]
    fn proximity_blend_collapses_to_true_scale_inside() {
        // Far views keep the readability exaggeration; at the surface the
        // mesh must be physical so fly-through reads as approach.
        let far = visual_radius_for_view(68_000_000.0, MapMode::Nereid, 100.0);
        assert!(
            (far - visual_radius_for_mode(68_000_000.0, MapMode::Nereid)).abs() < 1.0e-6,
            "far views must keep the readability exaggeration, got {far}"
        );
        let physical = true_radius_units(68_000_000.0);
        let inside = visual_radius_for_view(68_000_000.0, MapMode::Nereid, physical * 0.5);
        assert!(
            (inside - physical).abs() < 1.0e-6,
            "inside the body the scale must be exact, got {inside}"
        );
        let mid = visual_radius_for_view(68_000_000.0, MapMode::Nereid, physical * 4.0);
        assert!(
            mid > physical && mid < far,
            "mid blend must sit between, got {mid}"
        );
    }

    #[test]
    fn anchored_positions_match_focus_frame_without_follow() {
        // Without follow the anchor is zero, so anchored positions equal the
        // legacy focus-relative ones exactly: no visual change in overview.
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("the checked-in system configuration must parse");
        let ephemeris = config.bake().expect("the checked-in system must bake");
        let focus = ephemeris.body_id("nereid").expect("Nereid must exist");
        let map = MapState {
            mode: MapMode::Nereid,
            focus,
            selected: focus,
        };
        let navigation = NavigationState::default();
        let anchor = camera_anchor_f64(&ephemeris, &map, &navigation, SimTime::EPOCH);
        assert_eq!(anchor, bevy::math::DVec3::ZERO);
        for body in &ephemeris.bodies {
            if body.radius_m <= 0.0 {
                continue;
            }
            assert_eq!(
                map_position_anchored(&ephemeris, &map, body.id, SimTime::EPOCH, anchor),
                map_position(&ephemeris, &map, body.id, SimTime::EPOCH),
            );
        }
    }
}
