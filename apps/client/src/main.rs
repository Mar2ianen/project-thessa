mod map_ui;
mod navigation;
mod orbits;
mod perf;
mod pilot;
use map_ui::*;
use navigation::*;
use orbits::*;
use perf::PerfMonitorPlugin;
use pilot::*;

use std::{f32::consts::TAU, path::Path};

use bevy::{
    asset::AssetPlugin,
    core_pipeline::tonemapping::Tonemapping,
    input::mouse::{AccumulatedMouseMotion, MouseScrollUnit, MouseWheel},
    post_process::bloom::Bloom,
    prelude::*,
    window::{PrimaryWindow, WindowResolution},
};
use bevy_brp_extras::BrpExtrasPlugin;
use thessa_sim_core::{BakedEphemeris, BodyId, KeplerOrbit, SimTime, SystemConfig};

const DISTANCE_UNIT_M: f64 = 125_000_000.0;
const NEREID_RADIUS_M: f64 = 68_000_000.0;
const BASE_SIM_RATE_S_PER_REAL_SECOND: f64 = 3_600.0;
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
    App::new()
        .add_plugins(
            DefaultPlugins
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
                        ..default()
                    }),
                    ..default()
                }),
        )
        .add_plugins(OrbitGizmoPlugin)
        .add_plugins(PilotHudPlugin)
        .add_plugins(PerfMonitorPlugin)
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
                advance_simulation,
                update_starfield,
                update_celestial_visuals,
                draw_orbits,
                update_camera,
                update_hud,
            )
                .chain(),
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
    yaw: f32,
    pitch: f32,
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
) {
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
            yaw: 0.42,
            pitch: 0.72,
            distance: 420.0,
            target: Vec3::ZERO,
        },
        Transform::from_xyz(129.0, 276.0, 287.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Asterion's illumination is represented by direction, not by a fake
    // nearby star whose size would make the local map physically misleading.
    commands.spawn((
        DirectionalLight {
            illuminance: 4_500.0,
            color: Color::srgb(1.0, 0.82, 0.63),
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.12, -0.70, -0.24)),
    ));

    // The UV sphere duplicates the longitude seam. An icosphere shares the
    // vertices at that seam, which makes a 0/1 texture transition visible as
    // a jagged meridian on close-up planets.
    let sphere_mesh = meshes.add(Sphere::new(1.0).mesh().uv(64, 32));
    let visual_materials = VisualMaterials {
        asterion_a: materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.65, 0.24),
            emissive: LinearRgba::rgb(25.0, 8.0, 1.5),
            unlit: true,
            ..default()
        }),
        asterion_b: materials.add(StandardMaterial {
            base_color: Color::srgb(0.76, 0.88, 1.0),
            emissive: LinearRgba::rgb(30.0, 38.0, 55.0),
            unlit: true,
            ..default()
        }),
        asterion_c: materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.28, 0.08),
            emissive: LinearRgba::rgb(20.0, 2.0, 0.2),
            unlit: true,
            ..default()
        }),
        khepri: materials.add(StandardMaterial {
            base_color: Color::srgb(0.60, 0.22, 0.08),
            base_color_texture: Some(asset_server.load("textures/khepri-surface-v1.png")),
            perceptual_roughness: 0.92,
            ..default()
        }),
        nereid: materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.82, 0.58),
            base_color_texture: Some(asset_server.load("textures/nereid-atmosphere-v1.png")),
            perceptual_roughness: 0.82,
            ..default()
        }),
        thessa: materials.add(StandardMaterial {
            base_color: Color::srgb(0.85, 0.90, 0.86),
            base_color_texture: Some(asset_server.load("textures/thessa-surface-v2.png")),
            perceptual_roughness: 0.92,
            ..default()
        }),
        pelagos: materials.add(StandardMaterial {
            base_color: Color::srgb(0.06, 0.25, 0.44),
            base_color_texture: Some(asset_server.load("textures/pelagos-surface-v1.png")),
            perceptual_roughness: 0.76,
            metallic: 0.04,
            ..default()
        }),
        borea: materials.add(StandardMaterial {
            base_color: Color::srgb(0.78, 0.84, 0.88),
            base_color_texture: Some(asset_server.load("textures/borea-surface-v1.png")),
            perceptual_roughness: 0.98,
            ..default()
        }),
        orthea: materials.add(StandardMaterial {
            base_color: Color::srgb(0.48, 0.53, 0.58),
            base_color_texture: Some(asset_server.load("textures/orthea-surface-v1.png")),
            perceptual_roughness: 0.96,
            ..default()
        }),
        vesper: materials.add(StandardMaterial {
            base_color: Color::srgb(0.53, 0.70, 0.78),
            base_color_texture: Some(asset_server.load("textures/vesper-atmosphere-v1.png")),
            perceptual_roughness: 0.62,
            ..default()
        }),
        janus: materials.add(StandardMaterial {
            base_color: Color::srgb(0.70, 0.62, 0.48),
            base_color_texture: Some(asset_server.load("textures/janus-surface-v1.png")),
            perceptual_roughness: 0.94,
            ..default()
        }),
        volcanic: materials.add(StandardMaterial {
            base_color: Color::srgb(0.56, 0.20, 0.08),
            perceptual_roughness: 0.90,
            ..default()
        }),
        metal: materials.add(StandardMaterial {
            base_color: Color::srgb(0.66, 0.59, 0.43),
            perceptual_roughness: 0.94,
            metallic: 0.10,
            ..default()
        }),
        ice: materials.add(StandardMaterial {
            base_color: Color::srgb(0.56, 0.70, 0.78),
            perceptual_roughness: 0.88,
            ..default()
        }),
        minor: materials.add(StandardMaterial {
            base_color: Color::srgb(0.40, 0.43, 0.46),
            perceptual_roughness: 0.98,
            ..default()
        }),
    };
    for body in ephemeris.bodies.iter().filter(|body| body.radius_m > 0.0) {
        spawn_visual_body(
            &mut commands,
            sphere_mesh.clone(),
            material_for_body(body, &visual_materials),
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
    spawn_starfield(
        &mut commands,
        sphere_mesh,
        visual_materials.asterion_a.clone(),
        visual_materials.asterion_b.clone(),
        &mut materials,
    );
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

struct VisualMaterials {
    asterion_a: Handle<StandardMaterial>,
    asterion_b: Handle<StandardMaterial>,
    asterion_c: Handle<StandardMaterial>,
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
) -> Handle<StandardMaterial> {
    match body.name.as_str() {
        "asterion_a" => materials.asterion_a.clone(),
        "asterion_b" => materials.asterion_b.clone(),
        "asterion_c" => materials.asterion_c.clone(),
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

fn advance_simulation(
    time: Res<Time>,
    pilot: Option<Res<PilotHudState>>,
    mut clock: ResMut<SimulationClock>,
) {
    if pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot)
    {
        // Atmospheric flight uses real frame time in PilotFlightRuntime. The
        // map clock must not advance by an unrelated x3600 while the player
        // is flying, otherwise returning to Map causes a large time jump.
        return;
    }
    if !clock.paused {
        let frame_seconds = f64::from(time.delta_secs().min(0.1));
        clock.sim_seconds += frame_seconds * BASE_SIM_RATE_S_PER_REAL_SECOND * clock.multiplier;
    }
}

fn update_starfield(
    camera: Single<&Transform, (With<Camera3d>, Without<StarMarker>)>,
    mut stars: Query<(&mut Transform, &StarMarker)>,
) {
    let radius = 500.0;
    for (mut transform, star) in &mut stars {
        transform.translation = camera.translation + star.direction * radius * star.radius_factor;
        transform.scale = Vec3::splat(star.size * radius / 60.0);
    }
}

fn update_celestial_visuals(
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    pilot: Option<Res<PilotHudState>>,
    mut visuals: Query<(&mut Transform, &mut Visibility, &CelestialVisual)>,
) {
    let time = SimTime(clock.sim_seconds);
    let pilot_active = pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    for (mut transform, mut visibility, visual) in &mut visuals {
        if pilot_active {
            *visibility = Visibility::Hidden;
            continue;
        }
        if let Some(position) = map_position(&runtime.ephemeris, &map, visual.id, time) {
            let body = runtime
                .ephemeris
                .body(visual.id)
                .expect("visual body descriptor");
            transform.translation = position;
            transform.scale = Vec3::splat(visual_radius_for_mode(body.radius_m, map.mode));
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
    let focus_state = ephemeris
        .body_state(focus, time)
        .expect("focus body state must remain evaluable");
    let state = ephemeris
        .body_state(id, time)
        .expect("visual body state must remain evaluable");
    render_position(state.position_inertial - focus_state.position_inertial)
}

// Rotate the engine's Z-up frame into Bevy's Y-up frame before narrowing to f32.
fn render_position(relative_m: bevy::math::DVec3) -> Vec3 {
    let relative = relative_m / DISTANCE_UNIT_M;
    Vec3::new(relative.x as f32, relative.z as f32, -relative.y as f32)
}

fn visual_rotation(body: &thessa_sim_core::BakedBody, time: SimTime) -> Quat {
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
}
