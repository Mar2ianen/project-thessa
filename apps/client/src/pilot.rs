use super::*;

mod hud;
use hud::{pilot_hud_buttons, spawn_pilot_hud, update_pilot_hud};

use std::path::Path;

use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    math::{DQuat, DVec3, Mat3},
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    ui::FocusPolicy,
    world_serialization::{WorldAsset, WorldAssetRoot},
};

use thessa_flight_net::{ClientInput, Command, Snapshot};
use thessa_sim_core::{BakedEphemeris, BodyId, OnRailsCache, SimTime, WorldTick};

const HUD_TEXT: Color = Color::srgb(0.91, 0.95, 0.98);
const HUD_MUTED: Color = Color::srgb(0.60, 0.69, 0.76);
const HUD_GREEN: Color = Color::srgb(0.43, 0.87, 0.73);
const HUD_AMBER: Color = Color::srgb(0.98, 0.72, 0.26);
// Pilot preview coordinates are metres around the launch site. Unlike the
// system map's readability curve, this scene keeps the body's authored radius
// and the imported X-15 mesh in the same unit system.
const PILOT_START_ALTITUDE_M: f64 = 500.0;
const PILOT_CAMERA_DEFAULT_DISTANCE_M: f32 = 32.0;
const PILOT_CAMERA_MIN_DISTANCE_M: f32 = 8.0;
const PILOT_CAMERA_MAX_DISTANCE_M: f32 = 180.0;
const X15_SOURCE_LENGTH_M: f32 = 16.77;
const X15_AUTHORED_LENGTH_M: f32 = 15.45;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ClientViewMode {
    Map,
    Pilot,
}

use thessa_flight_authority::{BakeQueue, BakedRails, RailsBakeRequest};
pub(super) use thessa_flight_authority::{
    COAST_DENSITY_KG_M3, ControlMode, FlightAuthority, FlightRegime, X15_STALL_ANGLE_DEG,
    conventional_angle_of_attack_deg, local_air_kinematics,
};

/// Client-owned flight resource: authoritative stepping plus render-view
/// state. Field and method access to the authority flows through [`Deref`],
/// so existing systems keep reading `runtime.state` unchanged; only
/// construction and the view sync below know about the split.
#[derive(Resource)]
pub(super) struct PilotFlightRuntime {
    pub(super) authority: FlightAuthority,
    render_orientation: Quat,
}

impl std::ops::Deref for PilotFlightRuntime {
    type Target = FlightAuthority;

    fn deref(&self) -> &FlightAuthority {
        &self.authority
    }
}

impl std::ops::DerefMut for PilotFlightRuntime {
    fn deref_mut(&mut self) -> &mut FlightAuthority {
        &mut self.authority
    }
}

impl PilotFlightRuntime {
    pub(super) fn new(ephemeris: &BakedEphemeris, reference_body: BodyId) -> Result<Self, String> {
        let authority = FlightAuthority::new(ephemeris, reference_body)?
            .with_bake_queue(Box::new(BevyBakeQueue { job: None }));
        let render_orientation = render_orientation(authority.state.orientation_body_to_inertial);
        Ok(Self {
            authority,
            render_orientation,
        })
    }

    pub(super) fn with_trace(mut self, path: &Path) -> Self {
        self.authority = self.authority.with_trace(path);
        self
    }

    /// Refresh render-view state after an advance. Stepping never reads
    /// these; they exist so the frame loop pays the conversion once.
    pub(super) fn sync_view(&mut self) {
        self.render_orientation = render_orientation(self.state.orientation_body_to_inertial);
    }

    // Inherent forwarders: method paths (`PilotFlightRuntime::backlog_s`)
    // do not resolve through `Deref`, so the few function-pointer call
    // sites get one-line shims. Everything else derefs to the authority.
    pub(super) fn backlog_s(&self) -> f64 {
        self.authority.backlog_s()
    }

    pub(super) fn panel_count(&self) -> u32 {
        self.authority.panel_count()
    }

    pub(super) fn stop_reason(&self) -> Option<&str> {
        self.authority.stop_reason()
    }
}

/// Bevy-pool rails-bake worker: the pre-extraction behavior (spawn on the
/// compute pool, harvest on poll) behind the authority [`BakeQueue`].
pub(super) struct BevyBakeQueue {
    job: Option<bevy::tasks::Task<Result<BakedRails, String>>>,
}

impl BakeQueue for BevyBakeQueue {
    fn request_bake(&mut self, request: RailsBakeRequest) {
        if self.job.is_some() {
            return;
        }
        let Some(pool) = bevy::tasks::AsyncComputeTaskPool::try_get() else {
            return;
        };
        self.job = Some(pool.spawn(async move {
            let started = std::time::Instant::now();
            let mut rails = OnRailsCache::new();
            rails
                .bake_tick(
                    &request.ephemeris,
                    request.initial,
                    request.time,
                    request.config,
                    &request.impact_bodies,
                )
                .map_err(|e| e.to_string())?;
            Ok(BakedRails {
                rails,
                bake_seconds: started.elapsed().as_secs_f64(),
            })
        }));
    }

    fn poll_bake(&mut self) -> Option<Result<BakedRails, String>> {
        let job = self.job.as_mut()?;
        let result = bevy::tasks::block_on(bevy::tasks::poll_once(job))?;
        self.job = None;
        Some(result)
    }

    fn has_pending(&self) -> bool {
        self.job.is_some()
    }

    fn reset(&mut self) {
        self.job = None;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SpeedFrame {
    Surface,
    Air,
    Orbital,
    Target,
}

impl SpeedFrame {
    const ALL: [Self; 4] = [Self::Surface, Self::Air, Self::Orbital, Self::Target];

    fn next(self) -> Self {
        let index = Self::ALL
            .iter()
            .position(|frame| *frame == self)
            .unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }

    fn label(self) -> &'static str {
        match self {
            Self::Surface => "SURFACE",
            Self::Air => "AIR",
            Self::Orbital => "ORBITAL",
            Self::Target => "TARGET",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AltitudeFrame {
    Datum,
    Agl,
}

impl AltitudeFrame {
    fn toggle(self) -> Self {
        match self {
            Self::Datum => Self::Agl,
            Self::Agl => Self::Datum,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum AttitudeFrame {
    #[default]
    Local,
    Orbit,
    Target,
    Inertial,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
struct FlightEnvironment {
    reference_body: Option<String>,
    atmosphere_available: bool,
    terrain_available: bool,
    pressure_pa: Option<f64>,
    density_kg_m3: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum TelemetrySource {
    #[default]
    DemoPreview,
    Live,
}

/// Derived flight telemetry consumed by the pilot display.
///
/// This is a client read model. It is not fed back into sim-core and never
/// becomes authoritative vehicle state.
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
struct FlightUiState {
    source: TelemetrySource,
    sim_time_s: f64,
    vehicle_id: Option<String>,
    vehicle_name: Option<String>,
    position_m: DVec3,
    velocity_m_s: DVec3,
    orientation: DQuat,
    angular_velocity_rad_s: DVec3,
    attitude_frame: AttitudeFrame,
    environment: FlightEnvironment,
    surface_velocity_mps: DVec3,
    orbital_velocity_mps: DVec3,
    gravity_acceleration_mps2: DVec3,
    /// Local-up direction expressed in vehicle body axes. The navball uses
    /// this to project a real sphere instead of moving a flat horizon widget.
    local_up_body: DVec3,
    surface_speed_m_s: Option<f64>,
    air_speed_m_s: Option<f64>,
    orbital_speed_m_s: Option<f64>,
    target_speed_m_s: Option<f64>,
    altitude_datum_m: Option<f64>,
    altitude_agl_m: Option<f64>,
    vertical_speed_m_s: Option<f64>,
    heading_deg: Option<f64>,
    pitch_deg: Option<f64>,
    roll_deg: Option<f64>,
    mach: Option<f64>,
    dynamic_pressure_pa: Option<f64>,
    angle_of_attack_deg: Option<f64>,
    sideslip_deg: Option<f64>,
    apoapsis_altitude_m: Option<f64>,
    periapsis_altitude_m: Option<f64>,
    time_to_apoapsis_s: Option<f64>,
    time_to_periapsis_s: Option<f64>,
    target_name: Option<String>,
    target_distance_m: Option<f64>,
    target_closing_speed_m_s: Option<f64>,
    throttle: Option<f64>,
    thrust_n: Option<f64>,
    twr: Option<f64>,
    g_load: Option<f64>,
    sas_enabled: bool,
    rcs_enabled: bool,
    engine_active: bool,
    gear_down: bool,
    regime: FlightRegime,
    guidance_mode: Option<String>,
    warnings: Vec<String>,
}

impl FlightUiState {
    /// A display-only fixture retained for HUD formatting tests.
    /// It makes the PFD reviewable without pretending that a synthetic craft
    /// has entered sim-core or that any of these values drive physics.
    #[allow(dead_code)]
    fn demo_preview(sim_time_s: f64, reference_body: Option<String>) -> Self {
        Self {
            source: TelemetrySource::DemoPreview,
            sim_time_s,
            vehicle_name: Some("DEMO FLIGHT".into()),
            attitude_frame: AttitudeFrame::Local,
            environment: FlightEnvironment {
                reference_body,
                atmosphere_available: true,
                terrain_available: true,
                pressure_pa: Some(28_400.0),
                density_kg_m3: Some(0.42),
            },
            surface_velocity_mps: DVec3::X * 456.7,
            gravity_acceleration_mps2: DVec3::NEG_Z * 9.80665,
            local_up_body: DVec3::Z,
            surface_speed_m_s: Some(456.7),
            air_speed_m_s: Some(452.4),
            orbital_speed_m_s: Some(2_287.6),
            target_speed_m_s: Some(38.2),
            altitude_datum_m: Some(82_400.0),
            altitude_agl_m: Some(81_900.0),
            vertical_speed_m_s: Some(12.4),
            heading_deg: Some(90.0),
            pitch_deg: Some(4.6),
            roll_deg: Some(0.8),
            mach: Some(1.32),
            dynamic_pressure_pa: Some(28_400.0),
            angle_of_attack_deg: Some(2.1),
            sideslip_deg: Some(0.4),
            apoapsis_altitude_m: Some(125_400.0),
            periapsis_altitude_m: Some(80_100.0),
            time_to_apoapsis_s: Some(754.0),
            time_to_periapsis_s: Some(1_234.0),
            target_name: Some("ASCENT VECTOR".into()),
            target_distance_m: Some(2_100_000.0),
            target_closing_speed_m_s: Some(-38.2),
            throttle: Some(0.78),
            thrust_n: Some(1_320_000.0),
            twr: Some(1.18),
            g_load: Some(1.02),
            sas_enabled: true,
            rcs_enabled: true,
            engine_active: true,
            gear_down: true,
            guidance_mode: Some("MOUSE STEERING".into()),
            ..default()
        }
    }
}

#[derive(Debug, Resource)]
pub(super) struct PilotHudState {
    pub(super) view_mode: ClientViewMode,
    pub(super) control_mode: ControlMode,
    pub(super) speed_frame: SpeedFrame,
    pub(super) altitude_frame: AltitudeFrame,
    flight: FlightUiState,
    mouse_position: Vec2,
    show_modes: bool,
    pointer_over_ui: bool,
    show_help: bool,
    show_telemetry: bool,
    pilot_camera_orbit: Quat,
    pilot_camera_chase: bool,
    ui_hidden: bool,
    precision_controls: bool,
    sas_inverted: bool,
    pilot_camera_distance: f32,
    pilot_camera_pan: Vec2,
    /// Desired direction hand-off for the Mouse Aim flight-assist layer.
    pub(super) desired_direction: Vec3,
}

impl Default for PilotHudState {
    fn default() -> Self {
        Self {
            view_mode: ClientViewMode::Map,
            control_mode: ControlMode::Navball,
            speed_frame: SpeedFrame::Surface,
            altitude_frame: AltitudeFrame::Datum,
            flight: FlightUiState::default(),
            mouse_position: Vec2::ZERO,
            show_modes: false,
            pointer_over_ui: false,
            show_help: false,
            show_telemetry: false,
            pilot_camera_orbit: Quat::from_rotation_x(-0.18),
            pilot_camera_chase: false,
            ui_hidden: false,
            precision_controls: false,
            sas_inverted: false,
            pilot_camera_distance: PILOT_CAMERA_DEFAULT_DISTANCE_M,
            pilot_camera_pan: Vec2::ZERO,
            desired_direction: -Vec3::Z,
        }
    }
}

#[derive(Component)]
struct PilotPreviewVisual;

#[derive(Component)]
pub(super) struct PilotPlanetVisual;

#[derive(Component)]
struct PilotCraftVisual;

#[derive(Component)]
struct PilotEngineFlame;

pub(super) struct PilotHudPlugin;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) struct PilotUpdate;
impl Plugin for PilotHudPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PilotHudState>()
            .add_systems(Startup, spawn_pilot_hud)
            .add_systems(
                Update,
                (
                    pilot_hud_buttons,
                    pilot_input,
                    simulate_pilot_flight,
                    update_flight_ui_state,
                    update_pilot_preview,
                    update_pilot_hud,
                )
                    .chain()
                    .in_set(PilotUpdate),
            );
    }
}

pub(super) fn spawn_pilot_preview(
    commands: &mut Commands,
    sphere_mesh: Handle<Mesh>,
    planet_material: Handle<StandardMaterial>,
    asset_server: &AssetServer,
    planet_radius_m: f64,
) {
    let x15_scene: Handle<WorldAsset> =
        asset_server.load("models/north_american_x-15_plane.glb#Scene0");
    let planet_radius = planet_radius_m.max(1.0) as f32;

    commands
        .spawn((
            PilotPreviewVisual,
            Transform::default(),
            Visibility::Hidden,
            Name::new("PFD preview scene"),
        ))
        .with_children(|preview| {
            preview.spawn((
                PilotPlanetVisual,
                Mesh3d(sphere_mesh.clone()),
                MeshMaterial3d(planet_material),
                Transform::from_xyz(0.0, -(planet_radius + PILOT_START_ALTITUDE_M as f32), 0.0)
                    .with_scale(Vec3::splat(planet_radius)),
                Name::new(format!("PFD Thessa planet R={planet_radius_m:.0} m")),
            ));
            preview
                .spawn((
                    PilotCraftVisual,
                    Transform::default(),
                    Visibility::Inherited,
                    Name::new("PFD North American X-15"),
                ))
                .with_children(|vehicle| {
                    vehicle.spawn((
                        WorldAssetRoot(x15_scene),
                        Transform {
                            rotation: x15_asset_to_craft_rotation(),
                            scale: Vec3::splat(X15_AUTHORED_LENGTH_M / X15_SOURCE_LENGTH_M),
                            ..default()
                        },
                        Name::new("North American X-15 CC-BY-4.0 mesh"),
                    ));
                });
        });
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn update_pilot_preview(
    state: Res<PilotHudState>,
    clock: Res<SimulationClock>,
    runtime: Res<PilotFlightRuntime>,
    mut ambient: ResMut<GlobalAmbientLight>,
    mut camera_color: Single<&mut Camera, With<Camera3d>>,
    mut projection: Single<&mut Projection, With<Camera3d>>,
    mut camera: Single<&mut Transform, With<Camera3d>>,
    mut preview: Query<
        (&mut Transform, &mut Visibility),
        (
            With<PilotPreviewVisual>,
            Without<Camera3d>,
            Without<PilotCraftVisual>,
            Without<PilotEngineFlame>,
            Without<PilotPlanetVisual>,
        ),
    >,
    mut planet: Query<
        &mut Transform,
        (
            With<PilotPlanetVisual>,
            Without<Camera3d>,
            Without<PilotCraftVisual>,
            Without<PilotEngineFlame>,
            Without<PilotPreviewVisual>,
        ),
    >,
    mut craft: Query<
        &mut Transform,
        (
            With<PilotCraftVisual>,
            Without<Camera3d>,
            Without<PilotEngineFlame>,
        ),
    >,
    mut flames: Query<
        (&mut Transform, &mut Visibility),
        (
            With<PilotEngineFlame>,
            Without<Camera3d>,
            Without<PilotCraftVisual>,
        ),
    >,
) {
    let active = state.view_mode == ClientViewMode::Pilot;
    ambient.brightness = if active { 500.0 } else { 55.0 };
    ambient.color = if active {
        Color::srgb(0.65, 0.76, 0.88)
    } else {
        Color::srgb(0.18, 0.22, 0.32)
    };
    camera_color.clear_color = ClearColorConfig::Custom(if active {
        // Presentation-only atmospheric backdrop; fade toward space with altitude.
        let altitude = state
            .flight
            .altitude_datum_m
            .unwrap_or(PILOT_START_ALTITUDE_M);
        let atmosphere = (1.0 - altitude / 80_000.0).clamp(0.0, 1.0) as f32;
        Color::srgb(
            0.008 + 0.09 * atmosphere,
            0.018 + 0.16 * atmosphere,
            0.032 + 0.22 * atmosphere,
        )
    } else {
        Color::srgb(0.001, 0.002, 0.008)
    });
    if active {
        if let Projection::Perspective(perspective) = &mut **projection {
            perspective.near = 0.25;
            perspective.far =
                (runtime.relative_position_m.length() + runtime.planet_radius_m * 2.0) as f32;
        }
        **camera = pilot_camera_transform(
            &state,
            runtime.render_orientation,
            pilot_render_offset(runtime.relative_position_m).normalize(),
        );
    }
    for (mut transform, mut visibility) in &mut preview {
        *visibility = if active {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        if active {
            // Camera input changes the Camera3d and the live flight model
            // moves the craft. The planet remains a stable world-space
            // backdrop instead of rotating as a substitute for camera input.
            transform.translation = Vec3::ZERO;
            transform.rotation = Quat::IDENTITY;
        }
    }
    for mut transform in &mut planet {
        if active {
            // Subtract the camera/craft origin in f64 before conversion.
            // The craft and all its child meshes stay near zero even after
            // an interplanetary flight; no centimetre-scale f32 cancellation.
            transform.translation = pilot_render_offset(-runtime.relative_position_m);
            transform.rotation =
                Quat::from_rotation_y(runtime.terrain_spin() as f32) * SPHERE_POLE_TO_WORLD_UP;
        }
    }
    // Copy the authoritative pose even while paused: a freshly spawned scene
    // must not keep its identity transform when the simulation is stopped.
    for mut transform in &mut craft {
        transform.translation = Vec3::ZERO;
        transform.rotation = runtime.render_orientation;
    }
    for (mut transform, mut visibility) in &mut flames {
        let active = !clock.paused && runtime.engine_active && runtime.throttle > 0.005;
        *visibility = if active {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        if active {
            let plume = (0.35 + runtime.throttle * 1.1).clamp(0.35, 1.45);
            transform.scale = Vec3::splat(plume as f32);
        }
    }
}

fn make_navball_image(local_up_body: DVec3) -> Image {
    const SIZE: u32 = 256;
    let pixels = make_navball_pixels(local_up_body);

    Image::new(
        Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        pixels,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}

fn make_navball_pixels(local_up_body: DVec3) -> Vec<u8> {
    const SIZE: u32 = 256;
    let center = SIZE as f64 * 0.5;
    let radius = center - 1.0;
    let local_up = local_up_body.try_normalize().unwrap_or(DVec3::Z);
    let mut pixels = Vec::with_capacity((SIZE * SIZE * 4) as usize);

    for y in 0..SIZE {
        for x in 0..SIZE {
            let dx = x as f64 + 0.5 - center;
            let dy = y as f64 + 0.5 - center;
            let distance = dx.hypot(dy);
            if distance > radius {
                pixels.extend_from_slice(&[0, 0, 0, 0]);
                continue;
            }

            let normalized_x = dx / radius;
            let normalized_y = dy / radius;
            // The navball is a view of a unit sphere from the vehicle nose
            // (+X). Screen-right is forward x up = -Y; screen-up is +Z. This
            // projection makes the horizon respond to all three attitude
            // axes, including roll, instead of painting a flat semicircle.
            let sphere_depth = (1.0 - normalized_x * normalized_x - normalized_y * normalized_y)
                .max(0.0)
                .sqrt();
            let sphere_direction =
                DVec3::new(sphere_depth, -normalized_x, -normalized_y).normalize();
            let horizon = sphere_direction.dot(local_up);
            let sky_weight = ((horizon + 0.035) / 0.070).clamp(0.0, 1.0);
            let light_direction = DVec3::new(0.42, -0.38, 0.82).normalize();
            let light = (sphere_direction.dot(light_direction) * 0.5 + 0.5).clamp(0.0, 1.0);
            let edge_shade = 0.70 + 0.30 * light;
            let horizon_glow = (1.0 - horizon.abs() * 15.0).clamp(0.0, 1.0);
            let sky = DVec3::new(0.08, 0.52, 0.72);
            let ground = DVec3::new(0.60, 0.32, 0.095);
            let mut colour = ground.lerp(sky, sky_weight);

            // KSP-style pitch ladder: lines are contours on the sphere in the
            // current local-up frame. They curve naturally near the rim and
            // remain stable when the aircraft rolls or pitches.
            for pitch_deg in [
                -60.0_f64, -45.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 45.0, 60.0,
            ] {
                let pitch_level = pitch_deg.to_radians().sin();
                let distance_to_line = (horizon - pitch_level).abs();
                let line_width = if pitch_deg == 0.0 { 0.018 } else { 0.012 };
                if distance_to_line < line_width {
                    let line_strength = 1.0 - distance_to_line / line_width;
                    colour = colour.lerp(DVec3::splat(0.82), line_strength * 0.78);
                }
            }
            colour = colour.lerp(DVec3::new(0.95, 0.62, 0.16), horizon_glow * 0.12);
            let edge_alpha = ((radius - distance) * 5.0).clamp(0.0, 1.0);
            pixels.extend_from_slice(&[
                (colour.x * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (colour.y * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (colour.z * edge_shade * 255.0).clamp(0.0, 255.0) as u8,
                (edge_alpha * 255.0).clamp(0.0, 255.0) as u8,
            ]);
        }
    }

    pixels
}

fn update_navball_image(image: &mut Image, local_up_body: DVec3) {
    if let Some(data) = image.data.as_mut() {
        *data = make_navball_pixels(local_up_body);
    }
}

/// Adopt an authoritative snapshot into the local runtime without
/// stepping: input fields stay local (they feed the next ClientInput),
/// server-owned fields are overwritten, display derivations recomputed.
fn adopt_snapshot(
    runtime: &mut PilotFlightRuntime,
    ephemeris: &BakedEphemeris,
    mode: ControlMode,
    snapshot: &Snapshot,
) {
    runtime.state = snapshot.state;
    runtime.flight_time_s = snapshot.flight_time_s;
    runtime.world_tick = WorldTick(snapshot.tick);
    runtime.accumulator_s = 0.0;
    runtime.throttle = snapshot.throttle;
    runtime.engine_active = snapshot.engine_active;
    runtime.steps_this_frame = snapshot.steps_this_frame;
    runtime.rails_advanced_this_frame = snapshot.rails_advanced_s;
    runtime.wake_notice = snapshot.wake_notice.clone();
    runtime.flight_error = snapshot.flight_error.clone();
    let time = SimTime(snapshot.flight_time_s);
    let Ok(body_state) = ephemeris.body_state(runtime.reference_body, time) else {
        return;
    };
    runtime.relative_position_m = runtime.state.position_inertial_m - body_state.position_inertial;
    runtime.regime = match runtime
        .atmosphere
        .sample((runtime.relative_position_m.length() - runtime.planet_radius_m).max(0.0))
    {
        Ok(sample) if sample.density_kg_m3 < COAST_DENSITY_KG_M3 => FlightRegime::Coast,
        _ => FlightRegime::Aero,
    };
    if let Ok((gravity, forces)) = runtime.display_loads(ephemeris, time, mode) {
        runtime.last_gravity_acceleration_inertial_mps2 = gravity;
        runtime.last_forces = Some(forces);
    }
    runtime.sync_view();
}

fn simulate_pilot_flight(
    time: Res<Time>,
    mut clock: ResMut<SimulationClock>,
    ephemeris: Res<RuntimeEphemeris>,
    state: Res<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
    mut perf: ResMut<crate::perf::PerfMonitor>,
    link: Option<Res<crate::embedded::EmbeddedLink>>,
) {
    runtime.steps_this_frame = 0;
    runtime.rails_advanced_this_frame = 0.0;
    clock.sim_seconds = runtime.flight_time_s;
    clock.tick = runtime.world_tick;
    if let Some(link) = link.as_deref() {
        // Embedded mode: the server steps; this frame only ferries the
        // live input buffer down and adopts the newest snapshot. Pause
        // and error states live server-side, so neither early-returns
        // below may skip adoption.
        let target = runtime.sas_target_orientation;
        link.send_input(&ClientInput {
            tick: runtime.world_tick.0,
            control_input: runtime.control_input.to_array(),
            control_mode: state.control_mode,
            sas_target_xyzw: [target.x, target.y, target.z, target.w],
            throttle: runtime.throttle,
            engine_active: runtime.engine_active,
            sas_enabled: runtime.sas_enabled,
            rcs_enabled: runtime.rcs_enabled,
            gear_down: runtime.gear_down,
            commands: vec![
                Command::SetWarp {
                    factor: clock.multiplier,
                },
                Command::Pause {
                    paused: clock.paused,
                },
            ],
        });
        if let Some(snapshot) = link.latest_snapshot() {
            adopt_snapshot(
                &mut runtime,
                &ephemeris.ephemeris,
                state.control_mode,
                &snapshot,
            );
        }
        runtime.sync_view();
        clock.sim_seconds = runtime.flight_time_s;
        clock.tick = runtime.world_tick;
        return;
    }
    if clock.paused {
        return;
    }

    if runtime.flight_error.is_some() {
        return;
    }
    let frame_dt = time.delta_secs_f64().clamp(0.0, 0.1);
    let started = std::time::Instant::now();
    if let Err(error) = runtime.advance_with_budget(
        &ephemeris.ephemeris,
        state.control_mode,
        frame_dt * clock.multiplier,
        Some(std::time::Duration::from_millis(8)),
    ) {
        runtime.engine_active = false;
        runtime.control_input = DVec3::ZERO;
        runtime.accumulator_s = 0.0;
        perf.push_event("Flight stopped", Some(error.to_string()));
        runtime.flight_error = Some(error.to_string());
    }
    runtime.sync_view();
    clock.sim_seconds = runtime.flight_time_s;
    clock.tick = runtime.world_tick;
    perf.record_sim(started.elapsed().as_secs_f64());
    if let Some(seconds) = runtime.rails_bake_seconds.take() {
        perf.record_scope("simulation.coast_bake", seconds);
        perf.push_event("Coast trajectory baked", Some(format!("{seconds:.3} s")));
    }
}

/// The checked-in GLB scene (before Blender's Y-up -> Z-up import conversion)
/// has nose -X, cockpit/dorsal fin +Y, and lateral +Z. Its root nodes only
/// scale the mesh; they do not rotate it. Map those axes into the craft
/// parent's nose +Y, top -Z, lateral +X exactly once.
fn x15_asset_to_craft_rotation() -> Quat {
    Quat::from_mat3(&Mat3::from_cols(Vec3::NEG_Y, Vec3::NEG_Z, Vec3::X))
}

fn render_orientation(orientation: DQuat) -> Quat {
    let forward = pilot_render_offset(orientation * DVec3::X).normalize_or_zero();
    let lateral = pilot_render_offset(orientation * DVec3::Y).normalize_or_zero();
    let up = pilot_render_offset(orientation * DVec3::Z).normalize_or_zero();
    if forward.length_squared() < 1.0e-8
        || lateral.length_squared() < 1.0e-8
        || up.length_squared() < 1.0e-8
    {
        return Quat::IDENTITY;
    }
    // The model's +Y is its nose, +X its lateral axis and +Z points down so the
    // three visual axes form a right-handed basis around the engine body axes.
    Quat::from_mat3(&Mat3::from_cols(lateral, forward, -up))
}

fn pilot_render_offset(relative_delta_m: DVec3) -> Vec3 {
    // Pilot mode deliberately uses metres around the launch site. The map
    // uses a compressed astronomical unit; reusing that conversion here was
    // the source of the oversized planet and inconsistent craft altitude.
    Vec3::new(
        relative_delta_m.x as f32,
        relative_delta_m.z as f32,
        -relative_delta_m.y as f32,
    )
}

fn pilot_camera_transform(
    state: &PilotHudState,
    craft_orientation: Quat,
    local_up: Vec3,
) -> Transform {
    // Quaternion orbit has no polar clamp and retains camera-up through
    // vertical crossings. Panning uses camera axes, not fixed world axes.
    let forward = craft_orientation * Vec3::Y;
    let basis = if state.pilot_camera_chase {
        Quat::from_mat3(&Mat3::from_cols(
            -craft_orientation * Vec3::X,
            -craft_orientation * Vec3::Z,
            -forward,
        ))
    } else {
        let north = (Vec3::Y - local_up * local_up.y)
            .try_normalize()
            .unwrap_or(Vec3::Z);
        let east = north.cross(local_up).normalize();
        Quat::from_mat3(&Mat3::from_cols(east, local_up, -north))
    };
    let orbit = basis * state.pilot_camera_orbit;
    let up = orbit * Vec3::Y;
    let target = orbit * Vec3::new(state.pilot_camera_pan.x, state.pilot_camera_pan.y, 0.0);
    Transform::from_translation(target + orbit * Vec3::Z * state.pilot_camera_distance)
        .looking_at(target - up * (state.pilot_camera_distance * 0.12), up)
}

fn keyboard_control_input(keys: &ButtonInput<KeyCode>) -> DVec3 {
    // Stick forward (W) lowers the nose; right (D) yaws right.
    DVec3::new(
        (keys.pressed(KeyCode::KeyS) as i8 - keys.pressed(KeyCode::KeyW) as i8) as f64,
        (keys.pressed(KeyCode::KeyD) as i8 - keys.pressed(KeyCode::KeyA) as i8) as f64,
        (keys.pressed(KeyCode::KeyE) as i8 - keys.pressed(KeyCode::KeyQ) as i8) as f64,
    )
}

#[allow(clippy::too_many_arguments)]
fn pilot_input(
    time: Res<Time>,
    mut clock: ResMut<SimulationClock>,
    keys: Res<ButtonInput<KeyCode>>,
    window: Single<&Window, With<PrimaryWindow>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mut state: ResMut<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
    ephemeris: Res<RuntimeEphemeris>,
) {
    let mut scroll = 0.0;

    // A stopped flight soft-locks the session without this: rebuild the
    // launch-site state in place (clock time preserved).
    if runtime.flight_error.is_some()
        && keys.just_pressed(KeyCode::Backspace)
        && let Err(error) = runtime.reset_to_launch_site(&ephemeris.ephemeris)
    {
        runtime.flight_error = Some(error);
    }

    if keys.just_pressed(KeyCode::KeyM) {
        runtime.control_input = DVec3::ZERO;
        state.view_mode = match state.view_mode {
            ClientViewMode::Map => ClientViewMode::Pilot,
            ClientViewMode::Pilot => ClientViewMode::Map,
        };
    }

    if state.view_mode == ClientViewMode::Pilot {
        // Time warp shares the map's ArrowUp/Down binding; the map branch
        // above returns early in pilot view so there is no double handling.
        // High warp only sustains on rails (see MAX_TIME_WARP); off-rails
        // the frame budget caps effective warp automatically.
        if keys.just_pressed(KeyCode::ArrowUp) {
            clock.multiplier = (clock.multiplier * 2.0).min(MAX_TIME_WARP);
        }
        if keys.just_pressed(KeyCode::ArrowDown) {
            clock.multiplier = (clock.multiplier / 2.0).max(0.125);
        }
        if keys.just_pressed(KeyCode::F2) {
            state.ui_hidden = !state.ui_hidden;
        }
        if keys.just_pressed(KeyCode::F3) {
            state.show_telemetry = !state.show_telemetry;
        }
        if keys.just_pressed(KeyCode::F1) {
            state.show_help = !state.show_help;
            state.ui_hidden = false;
        }
        if keys.just_pressed(KeyCode::F8)
            || keys.just_pressed(KeyCode::Pause)
            || keys.just_pressed(KeyCode::Escape)
        {
            clock.paused = !clock.paused;
        }
        for event in mouse_wheel.read() {
            scroll += match event.unit {
                MouseScrollUnit::Line => event.y,
                MouseScrollUnit::Pixel => event.y / MouseScrollUnit::SCROLL_UNIT_CONVERSION_FACTOR,
            };
        }
        let mouse_delta = mouse_motion.delta;
        if mouse_buttons.pressed(MouseButton::Right) {
            state.pilot_camera_orbit = (state.pilot_camera_orbit
                * Quat::from_rotation_y(-mouse_delta.x * 0.005)
                * Quat::from_rotation_x(-mouse_delta.y * 0.005))
            .normalize();
        }
        if mouse_buttons.pressed(MouseButton::Middle) {
            let pan_scale = state.pilot_camera_distance * 0.0018;
            state.pilot_camera_pan += Vec2::new(-mouse_delta.x, mouse_delta.y) * pan_scale;
            state.pilot_camera_pan = state
                .pilot_camera_pan
                .clamp(Vec2::splat(-4.0), Vec2::splat(4.0));
        }
        if scroll != 0.0 && !state.show_help && !state.pointer_over_ui {
            state.pilot_camera_distance = (state.pilot_camera_distance * (-scroll * 0.09).exp())
                .clamp(PILOT_CAMERA_MIN_DISTANCE_M, PILOT_CAMERA_MAX_DISTANCE_M);
        }
        if keys.just_pressed(KeyCode::Backquote) {
            state.pilot_camera_orbit = Quat::from_rotation_x(-0.18);
            state.pilot_camera_distance = PILOT_CAMERA_DEFAULT_DISTANCE_M;
            state.pilot_camera_pan = Vec2::ZERO;
        }
        if keys.just_pressed(KeyCode::KeyV) {
            state.pilot_camera_chase = !state.pilot_camera_chase;
        }
        if keys.just_pressed(KeyCode::CapsLock) {
            state.precision_controls = !state.precision_controls;
        }
        let invert_sas = keys.pressed(KeyCode::KeyF);
        if invert_sas != state.sas_inverted {
            runtime.sas_enabled = !runtime.sas_enabled;
            state.sas_inverted = invert_sas;
        }

        let keyboard_input = keyboard_control_input(&keys);
        // Orbit/pan gestures belong exclusively to the camera. They must not
        // simultaneously command Mouse Aim, otherwise dragging the view also
        // deflects the aircraft and makes the controls feel broken.
        let camera_gesture =
            mouse_buttons.pressed(MouseButton::Right) || mouse_buttons.pressed(MouseButton::Middle);
        let mouse_input = if camera_gesture || state.pointer_over_ui || state.show_modes {
            DVec3::ZERO
        } else {
            DVec3::new(
                f64::from(state.desired_direction.y),
                f64::from(state.desired_direction.x),
                0.0,
            ) * 0.85
        };
        let command_input = match state.control_mode {
            ControlMode::MouseAim if keyboard_input.length_squared() < 1.0e-8 => mouse_input,
            _ => keyboard_input,
        }
        .clamp_length(0.0, 1.0);
        runtime.control_input = if clock.paused || !window.focused || state.show_help {
            DVec3::ZERO
        } else {
            command_input * if state.precision_controls { 0.25 } else { 1.0 }
        };

        let dt = time.delta_secs_f64().min(0.1);
        let throttle_up = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        let throttle_down =
            keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
        runtime.throttle = (runtime.throttle
            + (throttle_up as i8 - throttle_down as i8) as f64 * dt * 0.55)
            .clamp(0.0, 1.0);
        if keys.just_pressed(KeyCode::KeyX) {
            runtime.throttle = 0.0;
        }
        if keys.just_pressed(KeyCode::KeyZ) {
            runtime.throttle = 1.0;
        }
        if keys.just_pressed(KeyCode::KeyT) {
            runtime.sas_enabled = !runtime.sas_enabled;
        }
        if keys.just_pressed(KeyCode::KeyR) {
            runtime.rcs_enabled = !runtime.rcs_enabled;
        }
        if keys.just_pressed(KeyCode::KeyG) {
            runtime.gear_down = !runtime.gear_down;
        }
        if keys.just_pressed(KeyCode::Space) {
            runtime.engine_active = !runtime.engine_active;
        }
    }

    let size = window.resolution.size();
    if size.x > 0.0 && size.y > 0.0 {
        let cursor = window
            .cursor_position()
            .filter(|_| window.focused)
            .unwrap_or(size * 0.5);
        let ndc = Vec2::new(
            (cursor.x / size.x * 2.0 - 1.0).clamp(-1.0, 1.0),
            (1.0 - cursor.y / size.y * 2.0).clamp(-1.0, 1.0),
        );
        state.mouse_position = cursor;
        // Mouse steering requests an angular rate. Keep a neutral center
        // and let the controller allocate physical surface/jet commands.
        let steer = (ndc.abs() - Vec2::splat(0.06)).max(Vec2::ZERO) / 0.94 * ndc.signum();
        state.desired_direction = Vec3::new(steer.x, steer.y, -1.0).normalize();
    }
}

fn update_flight_ui_state(
    clock: Res<SimulationClock>,
    map: Res<MapState>,
    runtime: Res<RuntimeEphemeris>,
    flight_runtime: Res<PilotFlightRuntime>,
    terrain: Res<terrain::WorldTerrain>,
    mut state: ResMut<PilotHudState>,
) {
    state.flight = live_flight_ui_state(&flight_runtime, &runtime.ephemeris, clock.sim_seconds);
    let origin = DVec3::from_array(flight_runtime.terrain_origin());
    let body_dir = DQuat::from_rotation_y(-flight_runtime.terrain_spin()) * origin.normalize();
    let ground = terrain.field.height_m(body_dir.to_array(), 32.0).max(0.0);
    state.flight.altitude_agl_m = Some(origin.length() - terrain.field.params.radius_m - ground);
    // `map` remains in the signature deliberately: the flight HUD and map
    // share the same body selection resource, but a pilot always flies the
    // selected playable world rather than the current map zoom focus.
    let _ = map;
}

fn live_flight_ui_state(
    flight: &PilotFlightRuntime,
    ephemeris: &BakedEphemeris,
    _map_time_s: f64,
) -> FlightUiState {
    let time = SimTime(flight.flight_time_s);
    // Display frame follows the strongest local pull, not the launch site:
    // near Nereid the speeds, datum and apsides read relative to Nereid.
    // Display-only hint — the solver keeps integrating against reference_body.
    let display_body = ephemeris
        .dominant_body(flight.state.position_inertial_m, time)
        .unwrap_or(flight.reference_body);
    let at_home = display_body == flight.reference_body;
    let Ok(body_state) = ephemeris.body_state(display_body, time) else {
        return FlightUiState::default();
    };
    let Ok(body) = ephemeris.body(display_body) else {
        return FlightUiState::default();
    };
    let Ok(kinematics) =
        local_air_kinematics(flight.atmosphere, flight.state, body_state, body.radius_m)
    else {
        return FlightUiState::default();
    };
    let relative_position = kinematics.relative_position_inertial_m;
    let radial_up = kinematics.radial_up;
    let velocity_relative_inertial = kinematics.relative_velocity_inertial_mps;
    // The atmosphere model belongs to the home world: its rotation correction
    // is only valid there. Away from home the surface speed is the plain
    // dominant-relative speed and air data is unavailable, not extrapolated.
    let velocity_surface = if at_home {
        kinematics.surface_velocity_inertial_mps
    } else {
        velocity_relative_inertial
    };
    let forces = flight.last_forces.as_ref();
    let environment = forces.map(|forces| forces.environment);
    // The solver's environment contains body translation plus body rotation;
    // use the same explicitly separated terms for the HUD. This prevents the
    // moon's orbital velocity from appearing as a 40 km/s airspeed sample.
    let air_velocity = kinematics.air_velocity_body_mps;
    let air_speed = if at_home {
        Some(air_velocity.length())
    } else {
        None
    };
    let surface_speed = velocity_surface.length();
    let forward = flight.state.orientation_body_to_inertial * DVec3::X;
    let right = flight.state.orientation_body_to_inertial * DVec3::Y;
    let up = flight.state.orientation_body_to_inertial * DVec3::Z;
    let heading_deg = forward.x.atan2(forward.y).to_degrees().rem_euclid(360.0);
    let pitch_deg = forward.dot(radial_up).clamp(-1.0, 1.0).asin().to_degrees();
    let roll_deg = right.dot(radial_up).atan2(up.dot(radial_up)).to_degrees();
    let altitude_m = kinematics.altitude_m;
    let vertical_speed = velocity_surface.dot(radial_up);
    let gravity = body.mu / relative_position.length().max(1.0).powi(2);
    let force_magnitude = forces
        .map(|forces| forces.total_force_inertial_n.length())
        .unwrap_or_else(|| flight.thrust_n());
    let g_load = (force_magnitude / flight.vehicle.mass_properties.mass_kg / 9.80665).max(0.0);
    let (apoapsis_altitude_m, periapsis_altitude_m) = estimate_orbit_altitudes(
        relative_position,
        velocity_relative_inertial,
        body.mu,
        body.radius_m,
    );

    FlightUiState {
        source: TelemetrySource::Live,
        sim_time_s: flight.flight_time_s,
        vehicle_id: Some("x15-live".into()),
        vehicle_name: Some(flight.vehicle.name.clone()),
        position_m: flight.state.position_inertial_m,
        velocity_m_s: flight.state.velocity_inertial_mps,
        orientation: flight.state.orientation_body_to_inertial,
        angular_velocity_rad_s: flight.state.angular_velocity_body_rps,
        attitude_frame: AttitudeFrame::Local,
        environment: FlightEnvironment {
            reference_body: Some(body.name.to_uppercase()),
            atmosphere_available: true,
            terrain_available: false,
            pressure_pa: if at_home {
                environment.map(|environment| {
                    // The atmosphere sample is deterministic; q and Mach come
                    // from the same environment, avoiding a second atmosphere
                    // approximation in the display path.
                    environment.density_kg_m3 * environment.speed_of_sound_mps.powi(2)
                        / flight.atmosphere.heat_capacity_ratio
                })
            } else {
                None
            },
            density_kg_m3: if at_home {
                environment.map(|environment| environment.density_kg_m3)
            } else {
                None
            },
        },
        surface_velocity_mps: velocity_surface,
        orbital_velocity_mps: velocity_relative_inertial,
        gravity_acceleration_mps2: flight.last_gravity_acceleration_inertial_mps2,
        local_up_body: flight.state.orientation_body_to_inertial.inverse() * radial_up,
        surface_speed_m_s: Some(surface_speed),
        air_speed_m_s: air_speed,
        orbital_speed_m_s: Some(velocity_relative_inertial.length()),
        target_speed_m_s: None,
        altitude_datum_m: Some(altitude_m.max(0.0)),
        // The spherical safety boundary is not a terrain/radar measurement.
        altitude_agl_m: None,
        vertical_speed_m_s: Some(vertical_speed),
        heading_deg: Some(heading_deg),
        pitch_deg: Some(pitch_deg),
        roll_deg: Some(roll_deg),
        mach: if at_home {
            forces.map(|forces| forces.aero.mach)
        } else {
            None
        },
        dynamic_pressure_pa: if at_home {
            forces.map(|forces| forces.aero.dynamic_pressure_pa)
        } else {
            None
        },
        angle_of_attack_deg: if at_home {
            Some(conventional_angle_of_attack_deg(
                kinematics.air_velocity_body_mps,
            ))
        } else {
            None
        },
        sideslip_deg: if at_home {
            Some(
                kinematics
                    .air_velocity_body_mps
                    .y
                    .atan2(kinematics.air_velocity_body_mps.x.abs().max(1.0e-6))
                    .to_degrees(),
            )
        } else {
            None
        },
        apoapsis_altitude_m: Some(apoapsis_altitude_m),
        periapsis_altitude_m: Some(periapsis_altitude_m),
        time_to_apoapsis_s: None,
        time_to_periapsis_s: None,
        target_name: None,
        target_distance_m: None,
        target_closing_speed_m_s: None,
        throttle: Some(flight.throttle),
        thrust_n: Some(flight.thrust_n()),
        twr: Some(flight.thrust_n() / (flight.vehicle.mass_properties.mass_kg * gravity)),
        g_load: Some(g_load),
        sas_enabled: flight.sas_enabled,
        rcs_enabled: flight.rcs_enabled,
        engine_active: flight.engine_active,
        gear_down: flight.gear_down,
        regime: flight.regime(),
        guidance_mode: Some(
            match flight.sas_enabled {
                true => "SAS / PILOT",
                false => "MANUAL PILOT",
            }
            .into(),
        ),
        warnings: {
            let mut warnings = Vec::new();
            if let Some(error) = &flight.flight_error {
                warnings.push(format!("FLIGHT STOPPED: {error}"));
                warnings.push("BACKSPACE resets flight".into());
            }
            if at_home
                && forces.is_some_and(|forces| forces.aero.dynamic_pressure_pa >= 100.0)
                && conventional_angle_of_attack_deg(air_velocity).abs() >= X15_STALL_ANGLE_DEG
            {
                warnings.push("HIGH ANGLE OF ATTACK".into());
            }
            if flight.actuator_saturated {
                warnings.push("CONTROL LIMIT".into());
            }
            if altitude_m < 100.0 && vertical_speed < -1.0 {
                warnings.push("LOW ALTITUDE / DESCENDING".into());
            }
            warnings
        },
    }
}

fn estimate_orbit_altitudes(
    position: DVec3,
    velocity: DVec3,
    mu: f64,
    radius_m: f64,
) -> (f64, f64) {
    let distance = position.length();
    let specific_energy = velocity.length_squared() * 0.5 - mu / distance.max(1.0);
    if specific_energy >= 0.0 {
        let eccentricity =
            (velocity.cross(position.cross(velocity)) / mu - position / distance).length();
        let periapsis = position.cross(velocity).length_squared() / (mu * (1.0 + eccentricity));
        return (f64::INFINITY, periapsis - radius_m);
    }
    let semi_major_axis = -mu / (2.0 * specific_energy);
    let eccentricity_vector = velocity.cross(position.cross(velocity)) / mu - position / distance;
    let eccentricity = eccentricity_vector.length().clamp(0.0, 0.999_999);
    (
        semi_major_axis * (1.0 + eccentricity) - radius_m,
        semi_major_axis * (1.0 - eccentricity) - radius_m,
    )
}

fn primary_speed(flight: &FlightUiState, frame: SpeedFrame) -> Option<f64> {
    match frame {
        SpeedFrame::Surface => flight.surface_speed_m_s,
        SpeedFrame::Air => flight.air_speed_m_s,
        SpeedFrame::Orbital => flight.orbital_speed_m_s,
        SpeedFrame::Target => flight.target_speed_m_s,
    }
}

fn primary_altitude(flight: &FlightUiState, frame: AltitudeFrame) -> Option<f64> {
    match frame {
        AltitudeFrame::Datum => flight.altitude_datum_m,
        AltitudeFrame::Agl => flight.altitude_agl_m,
    }
}

fn heading_cardinal(value: Option<f64>) -> &'static str {
    let heading = value
        .filter(|value| value.is_finite())
        .unwrap_or_default()
        .rem_euclid(360.0);
    match ((heading + 22.5) / 45.0).floor() as u8 {
        0 => "N",
        1 => "NE",
        2 => "E",
        3 => "SE",
        4 => "S",
        5 => "SW",
        6 => "W",
        7 => "NW",
        _ => "N",
    }
}

fn format_angle(value: Option<f64>) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1} deg"))
        .unwrap_or_else(|| "--".into())
}

fn format_speed_value(value_m_s: Option<f64>) -> String {
    value_m_s
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1}"))
        .unwrap_or_else(|| "--".into())
}

fn format_altitude_value(value_m: Option<f64>) -> String {
    altitude_parts(value_m)
        .map(|(number, unit)| format!("{number} {unit}"))
        .unwrap_or_else(|| "--".into())
}

#[cfg(test)]
fn format_altitude_number(value_m: Option<f64>) -> String {
    altitude_parts(value_m)
        .map(|(number, _)| number)
        .unwrap_or_else(|| "--".into())
}

#[cfg(test)]
fn format_altitude_unit(value_m: Option<f64>) -> String {
    altitude_parts(value_m)
        .map(|(_, unit)| unit.into())
        .unwrap_or_else(|| "--".into())
}

fn altitude_parts(value_m: Option<f64>) -> Option<(String, &'static str)> {
    value_m.filter(|value| value.is_finite()).map(|value| {
        if value.abs() < 1_000.0 {
            (format!("{value:.0}"), "m")
        } else {
            (format!("{:.1}", value / 1_000.0), "km")
        }
    })
}

fn format_pressure(value_pa: Option<f64>) -> String {
    value_pa
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() >= 1_000.0 {
                format!("{:.2} kPa", value / 1_000.0)
            } else {
                format!("{value:.0} Pa")
            }
        })
        .unwrap_or_else(|| "--".into())
}

fn format_force(value_n: Option<f64>) -> String {
    value_n
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() >= 1_000_000.0 {
                format!("{:.2} MN", value / 1_000_000.0)
            } else if value.abs() >= 1_000.0 {
                format!("{:.1} kN", value / 1_000.0)
            } else {
                format!("{value:.0} N")
            }
        })
        .unwrap_or_else(|| "--".into())
}

fn format_percent(value: Option<f64>) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{:.0}%", value * 100.0))
        .unwrap_or_else(|| "--".into())
}

fn format_scalar(value: Option<f64>, precision: usize) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.precision$}"))
        .unwrap_or_else(|| "--".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Input mapping + render transform direction: WASD must move the GLB
    /// nose the expected way. Stays client-side (keyboard mapping and the
    /// mesh-axis convention never leave the renderer).
    #[test]
    fn wasd_moves_the_rendered_nose_in_the_expected_direction() {
        for (key, right_component, up_component) in [
            (KeyCode::KeyW, 0.0, -1.0),
            (KeyCode::KeyS, 0.0, 1.0),
            (KeyCode::KeyA, -1.0, 0.0),
            (KeyCode::KeyD, 1.0, 0.0),
        ] {
            let config: SystemConfig =
                toml::from_str(include_str!("../../../data/system.toml")).unwrap();
            let ephemeris = config.bake().unwrap();
            let reference_body = ephemeris.body_id("thessa").unwrap();
            let mut flight = PilotFlightRuntime::new(&ephemeris, reference_body).unwrap();
            let mut baseline = PilotFlightRuntime::new(&ephemeris, reference_body).unwrap();
            let initial = flight.state.orientation_body_to_inertial;
            let forward = pilot_render_offset(initial * DVec3::X);
            let up = pilot_render_offset(initial * DVec3::Z);
            let screen_right = forward.cross(up).normalize();
            let expected = screen_right * right_component + up * up_component;
            let mut keys = ButtonInput::default();
            keys.press(key);
            flight.control_input = keyboard_control_input(&keys);
            flight
                .advance(&ephemeris, ControlMode::Navball, 1.0)
                .unwrap();
            baseline
                .advance(&ephemeris, ControlMode::Navball, 1.0)
                .unwrap();
            flight.sync_view();
            baseline.sync_view();
            // Follow the real GLB nose (-X) through both render transforms.
            let nose = flight.render_orientation * x15_asset_to_craft_rotation() * Vec3::NEG_X;
            let neutral = baseline.render_orientation * x15_asset_to_craft_rotation() * Vec3::NEG_X;
            let response = (nose - neutral).dot(expected);
            assert!(
                response > 0.025,
                "{key:?} moved the model the wrong way: {response}"
            );
        }
    }

    #[test]
    fn control_modes_cycle_forward_and_backward() {
        assert_eq!(ControlMode::MouseAim.next(), ControlMode::Navball);
        assert_eq!(ControlMode::Direct.next(), ControlMode::MouseAim);
        assert_eq!(ControlMode::MouseAim.previous(), ControlMode::Direct);
        assert_eq!(ControlMode::Navball.previous(), ControlMode::MouseAim);
    }

    #[test]
    fn speed_and_altitude_frames_are_independent() {
        assert_eq!(SpeedFrame::Surface.next(), SpeedFrame::Air);
        assert_eq!(SpeedFrame::Target.next(), SpeedFrame::Surface);
        assert_eq!(AltitudeFrame::Datum.toggle(), AltitudeFrame::Agl);
        assert_eq!(AltitudeFrame::Agl.toggle(), AltitudeFrame::Datum);
    }

    #[test]
    fn heading_marker_uses_visual_cardinals_without_numeric_duplication() {
        assert_eq!(heading_cardinal(Some(0.0)), "N");
        assert_eq!(heading_cardinal(Some(90.0)), "E");
        assert_eq!(heading_cardinal(Some(225.0)), "SW");
        assert_eq!(heading_cardinal(Some(-45.0)), "NW");
        assert_eq!(heading_cardinal(None), "N");
    }

    #[test]
    fn missing_preview_values_are_not_formatted_as_physics() {
        assert_eq!(format_speed_value(None), "--");
        assert_eq!(format_altitude_value(None), "--");
        assert_eq!(format_pressure(None), "--");
        assert_eq!(format_percent(None), "--");
    }

    #[test]
    fn demo_preview_is_explicitly_display_only() {
        let preview = FlightUiState::demo_preview(123.0, Some("NEREID".into()));
        assert_eq!(preview.source, TelemetrySource::DemoPreview);
        assert!(preview.vehicle_id.is_none());
        assert_eq!(preview.surface_speed_m_s, Some(456.7));
        assert_eq!(format_altitude_value(preview.altitude_datum_m), "82.4 km");
        assert_eq!(format_angle(preview.angle_of_attack_deg), "2.1 deg");
    }

    #[test]
    fn navball_is_a_dynamic_sphere_projection() {
        let level = make_navball_pixels(DVec3::Z);
        let pitched = make_navball_pixels(DVec3::X);
        assert_eq!(level.len(), 256 * 256 * 4);
        assert_ne!(level, pitched);
        // The projected disk keeps transparent corners, so it remains a
        // sphere-shaped instrument when rendered inside the circular frame.
        assert_eq!(&level[0..4], &[0, 0, 0, 0]);
    }

    #[test]
    fn pilot_camera_orbits_through_both_poles_without_clamping_or_singularities() {
        let mut hud = PilotHudState::default();
        for degrees in 0..=720 {
            hud.pilot_camera_orbit = Quat::from_rotation_x((degrees as f32).to_radians());
            let camera = pilot_camera_transform(&hud, Quat::IDENTITY, Vec3::Y);
            assert!(camera.rotation.is_finite());
            assert!((camera.translation.length() - hud.pilot_camera_distance).abs() < 1.0e-4);
        }
        hud.pilot_camera_orbit = Quat::IDENTITY;
        let first = pilot_camera_transform(&hud, Quat::IDENTITY, Vec3::Y).translation;
        hud.pilot_camera_orbit = Quat::from_rotation_x(std::f32::consts::PI);
        let opposite = pilot_camera_transform(&hud, Quat::IDENTITY, Vec3::Y).translation;
        assert!(first.dot(opposite) < 0.0);
    }

    #[test]
    fn x15_glb_nose_top_and_span_follow_physics_in_every_attitude() {
        // Include the actual asset-child rotation: testing only its parent
        // previously passed with the GLB sideways and pointing backwards.
        for orientation in [
            DQuat::IDENTITY,
            DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            DQuat::from_rotation_x(0.8),
            DQuat::from_rotation_y(-0.7),
            DQuat::from_rotation_z(1.2)
                * DQuat::from_rotation_y(-0.4)
                * DQuat::from_rotation_x(0.6),
        ] {
            let mesh_to_world = render_orientation(orientation) * x15_asset_to_craft_rotation();
            for (asset_axis, body_axis) in [
                (Vec3::NEG_X, DVec3::X), // nose
                (Vec3::Y, DVec3::Z),     // dorsal fin / cockpit
                (Vec3::Z, DVec3::Y),     // span
            ] {
                let expected = pilot_render_offset(orientation * body_axis);
                assert!((mesh_to_world * asset_axis - expected).length() < 1.0e-5);
            }
        }
    }

    #[test]
    fn x15_manual_commands_map_to_ksp_control_surfaces() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let reference_body = ephemeris.body_id("thessa").expect("playable body exists");
        let mut flight =
            PilotFlightRuntime::new(&ephemeris, reference_body).expect("X-15 runtime initializes");

        // KSP's W/S, A/D and Q/E channels are pitch, yaw and roll.  The
        // vehicle asset exposes one elevator, one rudder and split ailerons;
        // assert the actual actuator deflections so an axis/mesh conversion
        // regression cannot silently turn pitch into roll again.
        flight.command_controls(0.5, -0.25, -0.75);
        let panels = &flight.vehicle.aero_geometry.panels;
        let degrees = |radians: f64| radians.to_degrees();
        assert!((degrees(panels[2].control_deflection_rad) + 12.5).abs() < 1.0e-10);
        assert!((degrees(panels[3].control_deflection_rad) + 12.5).abs() < 1.0e-10);
        assert!((degrees(panels[4].control_deflection_rad) + 5.5).abs() < 1.0e-10);
        assert!((degrees(panels[0].control_deflection_rad) - 13.5).abs() < 1.0e-10);
        assert!((degrees(panels[1].control_deflection_rad) + 13.5).abs() < 1.0e-10);
    }

    #[test]
    fn surface_altitude_keeps_meter_resolution() {
        assert_eq!(format_altitude_value(Some(25.0)), "25 m");
        assert_eq!(format_altitude_value(Some(1_300.0)), "1.3 km");
        assert_eq!(format_altitude_number(Some(25.0)), "25");
        assert_eq!(format_altitude_unit(Some(25.0)), "m");
        assert_eq!(format_altitude_number(Some(1_300.0)), "1.3");
        assert_eq!(format_altitude_unit(Some(1_300.0)), "km");
    }

    #[test]
    fn x15_initial_state_is_relative_to_thessa_not_barycentric() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let reference_body = ephemeris.body_id("thessa").expect("playable body exists");
        let flight =
            PilotFlightRuntime::new(&ephemeris, reference_body).expect("X-15 runtime initializes");
        let body_state = ephemeris
            .body_state(reference_body, SimTime::EPOCH)
            .expect("body state evaluates");
        let relative_velocity = flight.state.velocity_inertial_mps - body_state.velocity_inertial;
        assert!(
            (relative_velocity.length() - (180.0_f64.powi(2) + 2.0_f64.powi(2)).sqrt()).abs()
                < 1.0e-9
        );
    }

    #[test]
    fn display_frame_follows_strongest_pull_not_launch_site() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let thessa = ephemeris.body_id("thessa").expect("playable body exists");
        let nereid = ephemeris.body_id("nereid").expect("giant exists");
        let mut flight =
            PilotFlightRuntime::new(&ephemeris, thessa).expect("X-15 runtime initializes");
        // Park 100 km over Nereid, co-moving with it: Thessa-relative speed
        // is tens of km/s here, Nereid-relative is zero.
        let giant = ephemeris
            .body_state(nereid, SimTime::EPOCH)
            .expect("giant state evaluates");
        let giant_body = ephemeris.body(nereid).expect("giant descriptor");
        flight.state.position_inertial_m =
            giant.position_inertial + DVec3::Z * (giant_body.radius_m + 100_000.0);
        flight.state.velocity_inertial_mps = giant.velocity_inertial;
        flight.flight_time_s = 0.0;
        let ui = live_flight_ui_state(&flight, &ephemeris, 0.0);
        assert_eq!(ui.environment.reference_body.as_deref(), Some("NEREID"));
        // Orbital readout is giant-relative (rest), not Thessa-relative.
        assert!(
            ui.orbital_speed_m_s.is_some_and(|v| v < 1.0),
            "orbital speed must read Nereid-relative, got {:?}",
            ui.orbital_speed_m_s
        );
        // Thessa's air model does not extend to Nereid: no extrapolated air.
        assert_eq!(ui.air_speed_m_s, None);
        assert_eq!(ui.mach, None);
        assert!((ui.altitude_datum_m.unwrap_or(f64::NAN) - 100_000.0).abs() < 1.0);
    }

    #[test]
    fn stopped_flight_resets_to_launch_site() {
        use std::sync::Arc;
        use thessa_worldgen_rocky::{
            field::field_from_manifest,
            spec_recipe::{SpecRecipe, manifest_from_spec},
        };
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let reference_body = ephemeris.body_id("thessa").expect("playable body exists");
        let mut flight =
            PilotFlightRuntime::new(&ephemeris, reference_body).expect("X-15 runtime initializes");
        let recipe: SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
                .expect("worldgen recipe parses");
        let manifest = manifest_from_spec(&recipe).expect("world manifest builds");
        let field = Arc::new(field_from_manifest(&manifest).expect("world field builds"));
        flight.initialize_world_site(field, [0.0, 1.0, 0.0], &ephemeris);
        // Simulate the stranded state from the screenshot: error latched,
        // engine cut, vehicle flung far outside the flight envelope.
        flight.flight_time_s = 12_345.0;
        flight.flight_error = Some("flight state exceeded solver bounds".to_string());
        flight.engine_active = false;
        flight.throttle = 1.0;
        flight.state.position_inertial_m += DVec3::new(1.0e9, 0.0, 0.0);
        flight
            .reset_to_launch_site(&ephemeris)
            .expect("reset recovers a stopped flight");
        assert!(flight.flight_error.is_none());
        assert_eq!(flight.flight_time_s, 12_345.0);
        assert!(flight.engine_active);
        assert_eq!(flight.throttle, 0.0);
        let body_state = ephemeris
            .body_state(reference_body, SimTime(flight.flight_time_s))
            .expect("body state evaluates");
        let relative = flight.state.position_inertial_m - body_state.position_inertial;
        let altitude = relative.length() - flight.planet_radius_m;
        assert!(
            (0.0..20_000.0).contains(&altitude),
            "reset must park near the launch site, got {altitude}"
        );
    }

    #[test]
    fn pilot_aoa_is_positive_when_nose_is_above_velocity() {
        let nose_up = DVec3::new(100.0, 0.0, -100.0 * 5.0_f64.to_radians().tan());
        let nose_down = DVec3::new(100.0, 0.0, 100.0 * 5.0_f64.to_radians().tan());
        assert!((conventional_angle_of_attack_deg(nose_up) - 5.0).abs() < 1.0e-12);
        assert!((conventional_angle_of_attack_deg(nose_down) + 5.0).abs() < 1.0e-12);
    }
}
