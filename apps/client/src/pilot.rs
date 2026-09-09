use super::*;

use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    math::{DMat3, DQuat, DVec3, Mat3, Rot2},
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    ui::widget::ImageNode,
    ui::{FocusPolicy, UiTransform},
    world_serialization::{WorldAsset, WorldAssetRoot},
};

use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroPanel, AtmosphereConfig, BakedEphemeris, BodyId, FlightForces,
    FlightStepInput, GravityField, PanelAeroModel, RigidBodyProperties, RigidBodyState,
    VehicleDefinition, integrate_rigid_body_duration,
};

const HUD_BLUE: Color = Color::srgb(0.56, 0.78, 0.94);
const HUD_TEXT: Color = Color::srgb(0.84, 0.92, 0.98);
const HUD_MUTED: Color = Color::srgb(0.54, 0.66, 0.74);
const HUD_GREEN: Color = Color::srgb(0.38, 0.93, 0.51);
const HUD_AMBER: Color = Color::srgb(0.98, 0.72, 0.26);
// Pilot preview coordinates are metres around the launch site. Unlike the
// system map's readability curve, this scene keeps the body's authored radius
// and the imported X-15 mesh in the same unit system.
const PILOT_SURFACE_CLEARANCE_M: f64 = 5.0;
const PILOT_CAMERA_DEFAULT_DISTANCE_M: f32 = 32.0;
const PILOT_CAMERA_MIN_DISTANCE_M: f32 = 8.0;
const PILOT_CAMERA_MAX_DISTANCE_M: f32 = 180.0;
const X15_SOURCE_LENGTH_M: f32 = 16.77;
const X15_AUTHORED_LENGTH_M: f32 = 15.45;
const MAX_PILOT_ALTITUDE_M: f64 = 1.0e9;
const MAX_PILOT_RELATIVE_SPEED_MPS: f64 = 50_000.0;
const MAX_PILOT_ANGULAR_RATE_RPS: f64 = 25.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ClientViewMode {
    Map,
    Pilot,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ControlMode {
    MouseAim,
    Navball,
    Rate,
    Direct,
}

impl ControlMode {
    const ALL: [Self; 4] = [Self::MouseAim, Self::Navball, Self::Rate, Self::Direct];

    fn next(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }

    fn previous(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + Self::ALL.len() - 1) % Self::ALL.len()]
    }

    fn label(self) -> &'static str {
        match self {
            Self::MouseAim => "MOUSE AIM",
            Self::Navball => "NAVBALL / KSP",
            Self::Rate => "RATE CONTROL",
            Self::Direct => "DIRECT / RAW",
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::MouseAim => "point cursor / FBW handles the rest",
            Self::Navball => "direct attitude target control",
            Self::Rate => "command angular rates",
            Self::Direct => "raw actuator input",
        }
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

/// Live, client-owned flight model for the first playable vehicle.
///
/// The authoritative equations stay in `thessa-sim-core`; this resource only
/// supplies game input, the X-15 asset and a render/telemetry bridge. Fuel is
/// intentionally infinite for this slice, while staging still controls the
/// engine so the pilot can exercise the complete input loop.
#[derive(Resource)]
pub(super) struct PilotFlightRuntime {
    pub(super) reference_body: BodyId,
    planet_radius_m: f64,
    vehicle: VehicleDefinition,
    aero_model: PanelAeroModel,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    sas_target_orientation: DQuat,
    initial_relative_position_m: DVec3,
    flight_time_s: f64,
    throttle: f64,
    engine_active: bool,
    sas_enabled: bool,
    rcs_enabled: bool,
    gear_down: bool,
    /// Manual body-axis command: pitch, yaw, roll in normalized units.
    control_input: DVec3,
    render_position: Vec3,
    render_orientation: Quat,
    last_gravity_acceleration_inertial_mps2: DVec3,
    last_forces: Option<FlightForces>,
}

impl PilotFlightRuntime {
    pub(super) fn new(ephemeris: &BakedEphemeris, reference_body: BodyId) -> Result<Self, String> {
        let body = ephemeris
            .body(reference_body)
            .map_err(|error| format!("reference body is unavailable: {error}"))?;
        let body_state = ephemeris
            .body_state(reference_body, SimTime::EPOCH)
            .map_err(|error| format!("reference body state is unavailable: {error}"))?;
        let gravity = body.mu / body.radius_m.powi(2);
        let mut atmosphere = AtmosphereConfig::new(288.15, 108_000.0, 287.05287, 1.4, gravity)
            .map_err(|error| format!("Thessa atmosphere is invalid: {error}"))?;
        atmosphere.body_rotation_rad_s = DVec3::new(0.0, 0.0, TAU as f64 / (80.0 * 3_600.0));

        // Start on the playable body's spherical surface, a few metres above
        // the datum so the first frame is not an underground spawn. Give the
        // X-15 a short, already-airborne test start: the engine and controls
        // are live from the first frame, while the spherical datum remains the
        // collision reference for a genuine surface flight test.
        let initial_relative_position = DVec3::Z * (body.radius_m + PILOT_SURFACE_CLEARANCE_M);
        let initial_position_inertial_m = body_state.position_inertial + initial_relative_position;
        let orientation_body_to_inertial = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let state = RigidBodyState::new(
            initial_position_inertial_m,
            // The ephemeris velocity is the planet's barycentric translation.
            // Add only the local tangential launch velocity here. A small
            // upward component gives the powered test start enough
            // lift to leave the spherical datum. The old 95 m/s component
            // spawned the X-15 at roughly 28 degrees angle of attack and
            // immediately excited an unrecoverable pitch.
            body_state.velocity_inertial + DVec3::Y * 180.0 + DVec3::Z * 12.0,
            orientation_body_to_inertial,
            DVec3::ZERO,
        )
        .map_err(|error| format!("X-15 initial state is invalid: {error}"))?;
        let vehicle = x15_vehicle()?;
        let aero_model = PanelAeroModel::new(AeroConfig {
            lift_slope_per_rad: 4.6,
            control_effectiveness: 0.82,
            stall_angle_rad: 22.0_f64.to_radians(),
            max_lift_coefficient: 1.45,
            base_drag_coefficient: 0.032,
            induced_drag_factor: 0.075,
            wave_drag_coefficient: 0.22,
            side_force_slope_per_rad: 1.10,
            // The X-15 mesh is a visual reference, not a trimmed wind-tunnel
            // polar. Keep the zero-input preview trimmed; restoring and
            // damping moments still come from the actual panel geometry.
            pitching_moment_coefficient: 0.0,
            roll_damping_coefficient: -3.0,
            pitch_damping_coefficient: -4.0,
            yaw_damping_coefficient: -3.0,
            supersonic_lift_slope_factor: 4.0,
            supersonic_wave_drag_factor: 1.15,
            ..AeroConfig::default()
        })
        .map_err(|error| format!("X-15 aero model is invalid: {error}"))?;

        Ok(Self {
            reference_body,
            planet_radius_m: body.radius_m,
            vehicle,
            aero_model,
            atmosphere,
            state,
            sas_target_orientation: orientation_body_to_inertial,
            initial_relative_position_m: initial_relative_position,
            flight_time_s: 0.0,
            throttle: 1.0,
            engine_active: true,
            sas_enabled: true,
            rcs_enabled: true,
            gear_down: true,
            control_input: DVec3::ZERO,
            render_position: Vec3::ZERO,
            render_orientation: render_orientation(orientation_body_to_inertial),
            last_gravity_acceleration_inertial_mps2: DVec3::ZERO,
            last_forces: None,
        })
    }

    fn command_controls(&mut self, pitch: f64, yaw: f64, roll: f64) {
        // Elevator, rudder, left aileron, right aileron. The split ailerons
        // preserve roll authority without assigning a panel to two channels.
        let _ = self
            .vehicle
            .apply_control_inputs(&[pitch, yaw, roll, -roll]);
    }

    fn thrust_n(&self) -> f64 {
        if self.engine_active {
            self.throttle * 254_000.0
        } else {
            0.0
        }
    }
}

fn x15_vehicle() -> Result<VehicleDefinition, String> {
    // Keep the lifting center slightly forward of the body reference point.
    // Together with the inverted horizontal tail this is the small static
    // margin that prevents a zero-input powered start from pitching into a
    // post-stall tumble.
    let wing_left = AeroPanel::new(DVec3::new(0.20, -1.40, 0.0), DVec3::X, DVec3::Z, 9.29, 3.10)
        .and_then(|panel| panel.with_planform(5.70, 3.50, 35.0_f64.to_radians(), 0.86))
        .and_then(|panel| panel.with_thickness_ratio(0.085))
        .map_err(|error| error.to_string())?;
    let wing_right = AeroPanel::new(DVec3::new(0.20, 1.40, 0.0), DVec3::X, DVec3::Z, 9.29, 3.10)
        .and_then(|panel| panel.with_planform(5.70, 3.50, 35.0_f64.to_radians(), 0.86))
        .and_then(|panel| panel.with_thickness_ratio(0.085))
        .map_err(|error| error.to_string())?;
    let tail_left = AeroPanel::new(
        DVec3::new(-4.15, -0.45, 0.12),
        DVec3::X,
        DVec3::Z,
        1.30,
        1.10,
    )
    .and_then(|panel| panel.with_planform(2.25, 3.90, 25.0_f64.to_radians(), 0.94))
    .and_then(|panel| panel.with_lift_sign(-1.0))
    .map_err(|error| error.to_string())?;
    let tail_right = AeroPanel::new(
        DVec3::new(-4.15, 0.45, 0.12),
        DVec3::X,
        DVec3::Z,
        1.30,
        1.10,
    )
    .and_then(|panel| panel.with_planform(2.25, 3.90, 25.0_f64.to_radians(), 0.94))
    .and_then(|panel| panel.with_lift_sign(-1.0))
    .map_err(|error| error.to_string())?;
    let vertical_tail =
        AeroPanel::new(DVec3::new(-3.75, 0.0, 0.72), DVec3::X, DVec3::Y, 2.55, 1.70)
            .and_then(|panel| panel.with_planform(2.35, 2.15, 32.0_f64.to_radians(), 0.92))
            .and_then(|panel| panel.with_thickness_ratio(0.10))
            .map_err(|error| error.to_string())?;
    let geometry = AeroGeometry::new(vec![
        wing_left,
        wing_right,
        tail_left,
        tail_right,
        vertical_tail,
    ])
    .map_err(|error| error.to_string())?;
    let properties = RigidBodyProperties::new(
        10_200.0,
        DMat3::from_diagonal(DVec3::new(31_000.0, 115_000.0, 125_000.0)),
    )
    .map_err(|error| error.to_string())?;
    let controls = vec![
        thessa_sim_core::ControlSurfaceDefinition::new(
            "elevator",
            vec![2, 3],
            -25.0_f64.to_radians(),
            25.0_f64.to_radians(),
        ),
        thessa_sim_core::ControlSurfaceDefinition::new(
            "rudder",
            vec![4],
            -22.0_f64.to_radians(),
            22.0_f64.to_radians(),
        ),
        thessa_sim_core::ControlSurfaceDefinition::new(
            "aileron-left",
            vec![0],
            -18.0_f64.to_radians(),
            18.0_f64.to_radians(),
        ),
        thessa_sim_core::ControlSurfaceDefinition::new(
            "aileron-right",
            vec![1],
            -18.0_f64.to_radians(),
            18.0_f64.to_radians(),
        ),
    ]
    .into_iter()
    .collect::<Result<Vec<_>, _>>()
    .map_err(|error| error.to_string())?;
    VehicleDefinition::new("X-15 / THESSA FLIGHT TEST", geometry, properties, controls)
        .map_err(|error| error.to_string())
}

/// KSP-style SAS attitude hold for the live preview. The controller operates
/// on the authoritative body state and returns a real reaction moment; it
/// never edits orientation directly or hides aerodynamic instability.
fn pilot_sas_moment(state: RigidBodyState, target_orientation: DQuat, radial_up: DVec3) -> DVec3 {
    let body_up = state.orientation_body_to_inertial * DVec3::Z;
    let body_up_error = body_up.cross(radial_up);
    let target_error_body =
        (state.orientation_body_to_inertial.inverse() * target_orientation).to_scaled_axis();
    let local_up_error = state.orientation_body_to_inertial.inverse() * body_up_error;
    let damping = DVec3::new(250_000.0, 900_000.0, 700_000.0);
    let attitude_gain = DVec3::new(600_000.0, 1_500_000.0, 1_000_000.0);
    let horizon_gain = DVec3::splat(50_000.0);
    -state.angular_velocity_body_rps * damping
        + target_error_body * attitude_gain
        + local_up_error * horizon_gain
}

/// Keep the zero-input X-15 preview near a small positive-alpha trim point.
///
/// This is deliberately a bounded moment assist, not an orientation write:
/// the rigid-body integrator still resolves the resulting attitude and the
/// panel aero model remains authoritative.  Without this trim term the
/// temporary five-metre spherical datum can let a stalled craft fall through
/// the surface and build a large pitch rate before the contact correction.
fn pilot_aero_trim_moment(air_velocity_body: DVec3) -> DVec3 {
    let forward_speed = air_velocity_body.x;
    let speed = air_velocity_body.length();
    if !speed.is_finite() || speed < 1.0 {
        return DVec3::ZERO;
    }

    let angle_of_attack = air_velocity_body.z.atan2(forward_speed.max(1.0));
    let sideslip = air_velocity_body.y.atan2(forward_speed.abs().max(1.0));
    let trim_angle = 2.0_f64.to_radians();
    let pitch_error = (angle_of_attack - trim_angle).clamp(-0.45, 0.45);
    let yaw_error = sideslip.clamp(-0.35, 0.35);
    DVec3::new(0.0, -pitch_error * 1_800_000.0, yaw_error * 1_200_000.0)
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
            guidance_mode: Some("MOUSE AIM".into()),
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
    pilot_camera_yaw: f32,
    pilot_camera_pitch: f32,
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
            pilot_camera_yaw: 0.0,
            pilot_camera_pitch: 0.0,
            pilot_camera_distance: PILOT_CAMERA_DEFAULT_DISTANCE_M,
            pilot_camera_pan: Vec2::ZERO,
            desired_direction: -Vec3::Z,
        }
    }
}

#[derive(Component)]
struct PilotHudRoot;

#[derive(Component, Clone, Copy)]
enum PilotReadout {
    Header,
    Control,
    SpeedValue,
    SpeedDetail,
    AltitudeValue,
    AltitudeDetail,
    Vehicle,
    Context,
    Navball,
    NavballFooter,
    Status,
}

#[derive(Component)]
struct PilotAimReticle;

#[derive(Component)]
struct PilotFlightPathMarker;

#[derive(Component)]
struct PilotFlightPathVector;

#[derive(Component)]
struct PilotTapeMarker {
    altitude: bool,
}

#[derive(Component)]
struct PilotRollIndicator;

#[derive(Component)]
struct PilotHeadingMarker;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NavballVector {
    Prograde,
    Retrograde,
    Target,
    Gravity,
}

#[derive(Component)]
struct PilotNavballDisk;

#[derive(Resource)]
struct PilotNavballImage(Handle<Image>);

#[derive(Component)]
struct PilotNavballMarker {
    vector: NavballVector,
}

#[derive(Component)]
struct PilotPreviewVisual;

#[derive(Component)]
struct PilotPlanetVisual;

#[derive(Component)]
struct PilotCraftVisual;

#[derive(Component)]
struct PilotEngineFlame;

pub(super) struct PilotHudPlugin;

impl Plugin for PilotHudPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PilotHudState>()
            .add_systems(Startup, spawn_pilot_hud)
            .add_systems(
                Update,
                (
                    pilot_input,
                    simulate_pilot_flight,
                    update_flight_ui_state,
                    update_pilot_preview,
                    update_pilot_hud,
                )
                    .chain(),
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
                Transform::from_xyz(
                    0.0,
                    -(planet_radius + PILOT_SURFACE_CLEARANCE_M as f32),
                    0.0,
                )
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

fn spawn_pilot_hud(mut commands: Commands, mut images: ResMut<Assets<Image>>) {
    let navball_image = images.add(make_navball_image(DVec3::Z));
    commands.insert_resource(PilotNavballImage(navball_image.clone()));
    commands
        .spawn((
            PilotHudRoot,
            Node {
                position_type: PositionType::Absolute,
                width: percent(100),
                height: percent(100),
                ..default()
            },
            BackgroundColor(Color::srgba(0.001, 0.004, 0.010, 0.025)),
            // Let raw mouse motion reach the camera even when the cursor is
            // over a panel or the navball.
            FocusPolicy::Pass,
            Visibility::Hidden,
            ZIndex(110),
        ))
        .with_children(|root| {
            spawn_panel(
                root,
                PilotReadout::Header,
                "THESSA  //  PRIMARY FLIGHT DISPLAY",
                Node {
                    position_type: PositionType::Absolute,
                    top: px(16),
                    left: px(18),
                    width: px(370),
                    min_height: px(62),
                    ..panel_node()
                },
            );
            spawn_panel(
                root,
                PilotReadout::Control,
                "CONTROL MODE",
                Node {
                    position_type: PositionType::Absolute,
                    top: px(16),
                    right: px(18),
                    width: px(286),
                    min_height: px(132),
                    ..panel_node()
                },
            );
            spawn_speed_tape(root);
            spawn_altitude_tape(root);
            spawn_panel(
                root,
                PilotReadout::Vehicle,
                "VEHICLE / PROPULSION",
                Node {
                    position_type: PositionType::Absolute,
                    left: px(18),
                    bottom: px(24),
                    width: px(272),
                    min_height: px(116),
                    ..panel_node()
                },
            );
            spawn_panel(
                root,
                PilotReadout::Context,
                "TARGET / ORBIT",
                Node {
                    position_type: PositionType::Absolute,
                    right: px(18),
                    bottom: px(24),
                    width: px(286),
                    min_height: px(116),
                    ..panel_node()
                },
            );
            spawn_navball(root, navball_image.clone());
            spawn_center_cues(root);
            spawn_panel(
                root,
                PilotReadout::Status,
                "FLIGHT STATUS",
                Node {
                    position_type: PositionType::Absolute,
                    left: percent(50),
                    top: px(16),
                    width: px(276),
                    margin: UiRect::left(px(-138)),
                    min_height: px(58),
                    ..panel_node()
                },
            );
        });
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn update_pilot_preview(
    state: Res<PilotHudState>,
    clock: Res<SimulationClock>,
    runtime: Res<PilotFlightRuntime>,
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
    if active {
        let target = runtime.render_position
            + Vec3::new(state.pilot_camera_pan.x, state.pilot_camera_pan.y, 0.0);
        // Match the established map camera's elevated KSP-style framing. The
        // horizon stays below the craft instead of filling the whole PFD.
        let pitch = (0.72 + state.pilot_camera_pitch).clamp(0.12, 1.20);
        let horizontal = state.pilot_camera_distance * pitch.cos();
        let orbit_offset = Vec3::new(
            horizontal * state.pilot_camera_yaw.sin(),
            state.pilot_camera_distance * pitch.sin(),
            horizontal * state.pilot_camera_yaw.cos(),
        );
        camera.translation = target + orbit_offset;
        **camera = camera.looking_at(target, Vec3::Y);
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
            // The planet is anchored at the real launch-site origin. It must
            // not follow the craft: doing so made the world appear to rotate
            // or move with the camera and hid scale errors.
            transform.translation = Vec3::new(
                0.0,
                -(runtime.planet_radius_m as f32 + PILOT_SURFACE_CLEARANCE_M as f32),
                0.0,
            );
        }
    }
    // Copy the authoritative pose even while paused: a freshly spawned scene
    // must not keep its identity transform when the simulation is stopped.
    for mut transform in &mut craft {
        transform.translation = runtime.render_position;
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

fn panel_node() -> Node {
    Node {
        padding: UiRect::all(px(10)),
        border: UiRect::all(px(1)),
        border_radius: BorderRadius::all(px(5)),
        flex_direction: FlexDirection::Column,
        ..default()
    }
}

fn spawn_panel(
    parent: &mut ChildSpawnerCommands<'_>,
    readout: PilotReadout,
    title: &'static str,
    node: Node,
) {
    parent
        .spawn((
            node,
            BackgroundColor(Color::srgba(0.004, 0.016, 0.030, 0.42)),
            BorderColor::all(Color::srgba(0.22, 0.58, 0.78, 0.42)),
        ))
        .with_children(|panel| {
            panel.spawn((
                Text::new(title),
                TextFont {
                    font_size: FontSize::Px(10.5),
                    ..default()
                },
                TextColor(HUD_BLUE),
                Node {
                    margin: UiRect::bottom(px(8)),
                    ..default()
                },
            ));
            panel.spawn((
                readout,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(13.0),
                    ..default()
                },
                TextColor(HUD_TEXT),
                Node {
                    flex_grow: 1.0,
                    ..default()
                },
            ));
        });
}

fn spawn_speed_tape(parent: &mut ChildSpawnerCommands<'_>) {
    parent
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(172),
                left: px(18),
                width: px(178),
                height: px(380),
                ..panel_node()
            },
            BackgroundColor(Color::srgba(0.004, 0.016, 0.030, 0.34)),
            BorderColor::all(Color::srgba(0.22, 0.58, 0.78, 0.42)),
        ))
        .with_children(|tape| {
            tape.spawn((
                Text::new("SPEED  /  PRIMARY"),
                TextFont {
                    font_size: FontSize::Px(10.5),
                    ..default()
                },
                TextColor(HUD_BLUE),
            ));
            tape.spawn((
                PilotReadout::SpeedValue,
                Text::new("--"),
                TextFont {
                    font_size: FontSize::Px(30.0),
                    ..default()
                },
                TextColor(HUD_GREEN),
                TextLayout::justify(Justify::Center),
                Node {
                    width: percent(100),
                    margin: UiRect::top(px(8)),
                    ..default()
                },
            ));
            tape.spawn((
                Text::new("m/s"),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(HUD_MUTED),
                TextLayout::justify(Justify::Center),
                Node {
                    width: percent(100),
                    ..default()
                },
            ));
            spawn_tape_scale(tape, false);
            tape.spawn((
                PilotReadout::SpeedDetail,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(12.5),
                    ..default()
                },
                TextColor(HUD_TEXT),
                Node {
                    margin: UiRect::top(px(9)),
                    ..default()
                },
            ));
        });
}

fn spawn_altitude_tape(parent: &mut ChildSpawnerCommands<'_>) {
    parent
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(172),
                right: px(18),
                width: px(178),
                height: px(380),
                ..panel_node()
            },
            BackgroundColor(Color::srgba(0.004, 0.016, 0.030, 0.34)),
            BorderColor::all(Color::srgba(0.22, 0.58, 0.78, 0.42)),
        ))
        .with_children(|tape| {
            tape.spawn((
                Text::new("ALTITUDE  /  PRIMARY"),
                TextFont {
                    font_size: FontSize::Px(10.5),
                    ..default()
                },
                TextColor(HUD_BLUE),
            ));
            tape.spawn((
                PilotReadout::AltitudeValue,
                Text::new("--"),
                TextFont {
                    font_size: FontSize::Px(30.0),
                    ..default()
                },
                TextColor(HUD_GREEN),
                TextLayout::justify(Justify::Center),
                Node {
                    width: percent(100),
                    margin: UiRect::top(px(8)),
                    ..default()
                },
            ));
            tape.spawn((
                Text::new("km"),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(HUD_MUTED),
                TextLayout::justify(Justify::Center),
                Node {
                    width: percent(100),
                    ..default()
                },
            ));
            spawn_tape_scale(tape, true);
            tape.spawn((
                PilotReadout::AltitudeDetail,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(12.5),
                    ..default()
                },
                TextColor(HUD_TEXT),
                Node {
                    margin: UiRect::top(px(9)),
                    ..default()
                },
            ));
        });
}

fn spawn_tape_scale(parent: &mut ChildSpawnerCommands<'_>, altitude: bool) {
    let labels = if altitude {
        ["100", "75", "50", "25", "0"]
    } else {
        ["500", "400", "300", "200", "100"]
    };
    parent
        .spawn((
            Node {
                position_type: PositionType::Relative,
                width: percent(100),
                height: px(156),
                margin: UiRect::top(px(12)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.02, 0.08, 0.10, 0.46)),
            BorderColor::all(Color::srgba(0.20, 0.38, 0.46, 0.42)),
        ))
        .with_children(|scale| {
            scale.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: px(12),
                    top: px(8),
                    bottom: px(8),
                    width: px(5),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.20, 0.35, 0.39, 0.72)),
            ));
            scale.spawn((
                PilotTapeMarker { altitude },
                Node {
                    position_type: PositionType::Absolute,
                    left: px(12),
                    top: px(52),
                    width: px(5),
                    height: px(18),
                    ..default()
                },
                BackgroundColor(HUD_GREEN),
            ));
            for (index, label) in labels.into_iter().enumerate() {
                let top = 7.0 + index as f32 * 35.0;
                scale.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: px(12),
                        top: px(top),
                        width: px(37),
                        height: px(1),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.78, 0.90, 0.94, 0.75)),
                ));
                scale.spawn((
                    Text::new(label),
                    TextFont {
                        font_size: FontSize::Px(11.0),
                        ..default()
                    },
                    TextColor(HUD_TEXT),
                    Node {
                        position_type: PositionType::Absolute,
                        left: px(56),
                        top: px(top - 6.0),
                        ..default()
                    },
                ));
            }
        });
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
            // (+X). Screen-right is body +Y and screen-up is body +Z. This
            // projection makes the horizon respond to all three attitude
            // axes, including roll, instead of painting a flat semicircle.
            let sphere_depth = (1.0 - normalized_x * normalized_x - normalized_y * normalized_y)
                .max(0.0)
                .sqrt();
            let sphere_direction =
                DVec3::new(sphere_depth, normalized_x, -normalized_y).normalize();
            let horizon = sphere_direction.dot(local_up);
            let sky_weight = ((horizon + 0.035) / 0.070).clamp(0.0, 1.0);
            let light_direction = DVec3::new(0.42, -0.38, 0.82).normalize();
            let light = (sphere_direction.dot(light_direction) * 0.5 + 0.5).clamp(0.0, 1.0);
            let edge_shade = 0.46 + 0.54 * light;
            let horizon_glow = (1.0 - horizon.abs() * 15.0).clamp(0.0, 1.0);
            let sky = DVec3::new(0.035, 0.22, 0.36);
            let ground = DVec3::new(0.27, 0.145, 0.075);
            let mut colour = ground.lerp(sky, sky_weight);

            // KSP-style pitch ladder: lines are contours on the sphere in the
            // current local-up frame. They curve naturally near the rim and
            // remain stable when the aircraft rolls or pitches.
            for pitch_deg in [-30.0_f64, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0] {
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

fn spawn_navball(parent: &mut ChildSpawnerCommands<'_>, navball_image: Handle<Image>) {
    parent
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: percent(50),
                bottom: px(22),
                width: px(360),
                height: px(360),
                margin: UiRect::left(px(-180)),
                border: UiRect::all(px(2)),
                border_radius: BorderRadius::MAX,
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(Color::srgba(0.035, 0.14, 0.20, 0.94)),
            BorderColor::all(Color::srgba(0.46, 0.78, 0.90, 0.94)),
        ))
        .with_children(|navball| {
            navball.spawn((
                PilotNavballDisk,
                ImageNode::new(navball_image),
                Node {
                    position_type: PositionType::Absolute,
                    left: px(10),
                    right: px(10),
                    top: px(10),
                    bottom: px(10),
                    ..default()
                },
            ));
            navball.spawn((
                PilotReadout::Navball,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(HUD_TEXT),
                TextLayout::justify(Justify::Center),
                Node {
                    position_type: PositionType::Absolute,
                    top: px(14),
                    left: px(0),
                    width: percent(100),
                    ..default()
                },
            ));
            for (left, height) in [
                (112.0, 7.0),
                (136.0, 11.0),
                (160.0, 16.0),
                (184.0, 11.0),
                (208.0, 7.0),
            ] {
                navball.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: px(left),
                        top: px(30),
                        width: px(2),
                        height: px(height),
                        ..default()
                    },
                    BackgroundColor(Color::srgba(0.78, 0.90, 0.94, 0.52)),
                ));
            }
            navball.spawn((
                PilotRollIndicator,
                UiTransform::from_rotation(Rot2::radians(0.0)),
                Text::new("▼"),
                TextFont {
                    font_size: FontSize::Px(18.0),
                    ..default()
                },
                TextColor(HUD_AMBER),
                Node {
                    position_type: PositionType::Absolute,
                    left: px(163),
                    top: px(18),
                    width: px(34),
                    height: px(24),
                    ..default()
                },
            ));
            navball.spawn((
                Text::new("L                 R"),
                TextFont {
                    font_size: FontSize::Px(9.0),
                    ..default()
                },
                TextColor(HUD_MUTED),
                Node {
                    position_type: PositionType::Absolute,
                    left: px(105),
                    top: px(43),
                    width: px(150),
                    ..default()
                },
            ));
            navball.spawn((
                PilotHeadingMarker,
                Text::new("N"),
                TextFont {
                    font_size: FontSize::Px(18.0),
                    ..default()
                },
                TextColor(HUD_AMBER),
                Node {
                    position_type: PositionType::Absolute,
                    top: px(62),
                    left: px(154),
                    ..default()
                },
            ));
            navball.spawn((
                Text::new("FPV"),
                TextFont {
                    font_size: FontSize::Px(11.0),
                    ..default()
                },
                TextColor(HUD_GREEN),
                Node {
                    position_type: PositionType::Absolute,
                    top: px(151),
                    left: px(146),
                    ..default()
                },
            ));
            navball.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: px(138),
                    top: px(156),
                    width: px(52),
                    height: px(8),
                    border: UiRect::top(px(2)),
                    ..default()
                },
                BorderColor::all(HUD_AMBER),
            ));
            spawn_navball_marker(navball, NavballVector::Prograde, "O", HUD_GREEN);
            spawn_navball_marker(navball, NavballVector::Retrograde, "X", HUD_AMBER);
            spawn_navball_marker(navball, NavballVector::Target, "+", HUD_BLUE);
            spawn_navball_marker(navball, NavballVector::Gravity, "G", HUD_AMBER);
            navball.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: px(164),
                    top: px(136),
                    width: px(1),
                    height: px(48),
                    ..default()
                },
                BackgroundColor(HUD_AMBER),
            ));
            navball.spawn((
                PilotReadout::NavballFooter,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(10.0),
                    ..default()
                },
                TextColor(HUD_MUTED),
                TextLayout::justify(Justify::Center),
                Node {
                    position_type: PositionType::Absolute,
                    bottom: px(18),
                    left: px(0),
                    width: percent(100),
                    ..default()
                },
            ));
        });
}

fn spawn_navball_marker(
    parent: &mut ChildSpawnerCommands<'_>,
    vector: NavballVector,
    marker: &'static str,
    color: Color,
) {
    parent.spawn((
        PilotNavballMarker { vector },
        Text::new(marker),
        TextFont {
            font_size: FontSize::Px(18.0),
            ..default()
        },
        TextColor(color),
        Node {
            position_type: PositionType::Absolute,
            left: px(180.0),
            top: px(180.0),
            ..default()
        },
    ));
}

fn spawn_center_cues(parent: &mut ChildSpawnerCommands<'_>) {
    parent.spawn((
        PilotFlightPathVector,
        UiTransform::from_rotation(Rot2::radians(-0.61)),
        Node {
            position_type: PositionType::Absolute,
            left: percent(50),
            top: percent(50),
            width: px(66),
            height: px(2),
            ..default()
        },
        BackgroundColor(Color::srgba(0.98, 0.72, 0.26, 0.38)),
    ));
    parent
        .spawn((
            PilotAimReticle,
            Node {
                position_type: PositionType::Absolute,
                left: percent(50),
                top: percent(50),
                width: px(24),
                height: px(24),
                margin: UiRect::new(px(-12), px(0), px(-12), px(0)),
                border: UiRect::all(px(1)),
                border_radius: BorderRadius::MAX,
                ..default()
            },
            BorderColor::all(HUD_GREEN),
        ))
        .with_children(|reticle| {
            reticle.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: px(10),
                    top: px(10),
                    width: px(4),
                    height: px(4),
                    border_radius: BorderRadius::MAX,
                    ..default()
                },
                BackgroundColor(HUD_GREEN),
            ));
        });
    parent.spawn((
        PilotFlightPathMarker,
        Node {
            position_type: PositionType::Absolute,
            left: percent(50),
            top: percent(50),
            width: px(13),
            height: px(13),
            margin: UiRect::new(px(-6), px(0), px(-6), px(0)),
            border: UiRect::all(px(1)),
            border_radius: BorderRadius::MAX,
            ..default()
        },
        BorderColor::all(HUD_AMBER),
    ));
}

fn simulate_pilot_flight(
    time: Res<Time>,
    clock: Res<SimulationClock>,
    ephemeris: Res<RuntimeEphemeris>,
    state: Res<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
) {
    if state.view_mode != ClientViewMode::Pilot || clock.paused {
        return;
    }

    let frame_dt = f64::from(time.delta_secs().clamp(0.0, 0.05));
    if frame_dt <= 0.0 {
        return;
    }
    let sim_time = SimTime(runtime.flight_time_s);
    let Ok(body) = ephemeris.ephemeris.body(runtime.reference_body) else {
        return;
    };
    let Ok(body_state) = ephemeris
        .ephemeris
        .body_state(runtime.reference_body, sim_time)
    else {
        return;
    };
    let relative_position = runtime.state.position_inertial_m - body_state.position_inertial;
    let altitude_m = relative_position.length() - body.radius_m;
    let Some(radial_up) = relative_position.try_normalize() else {
        return;
    };
    let orientation_inverse = runtime.state.orientation_body_to_inertial.inverse();
    let position_body = orientation_inverse * relative_position;
    let relative_velocity_inertial =
        runtime.state.velocity_inertial_mps - body_state.velocity_inertial;
    let relative_velocity_body = orientation_inverse * relative_velocity_inertial;
    let air_velocity_body =
        relative_velocity_body - runtime.atmosphere.body_rotation_rad_s.cross(position_body);
    let gravity = match GravityField::from_ephemeris(&ephemeris.ephemeris)
        .acceleration(runtime.state.position_inertial_m, sim_time)
    {
        Ok(value) => value,
        Err(_) => return,
    };
    runtime.last_gravity_acceleration_inertial_mps2 = gravity;

    let pitch = runtime.control_input.x;
    let yaw = runtime.control_input.y;
    let roll = runtime.control_input.z;
    runtime.command_controls(pitch, yaw, roll);

    // SAS is a deliberately small flight-assist layer: it damps body rates,
    // holds the KSP-style attitude target accumulated by the pilot controls
    // and keeps that target close to the local horizon as the craft travels
    // around the body. It does not bypass the aero model or write a different
    // authoritative state.
    let sas_active = runtime.sas_enabled
        && matches!(
            state.control_mode,
            ControlMode::MouseAim | ControlMode::Navball
        );
    let sas_moment = if sas_active {
        pilot_sas_moment(runtime.state, runtime.sas_target_orientation, radial_up)
    } else {
        DVec3::ZERO
    };
    let trim_moment = if sas_active && runtime.control_input.length_squared() < 1.0e-8 {
        pilot_aero_trim_moment(air_velocity_body)
    } else {
        DVec3::ZERO
    };
    // The first playable datum has no landing gear or terrain contact mesh
    // yet.  Damping body rates while descending through the five-metre
    // clearance keeps a soft spherical contact from becoming a spin source.
    let relative_radial_speed = relative_velocity_inertial.dot(radial_up);
    let contact_moment =
        if altitude_m <= PILOT_SURFACE_CLEARANCE_M + 0.5 && relative_radial_speed <= 0.0 {
            -runtime.state.angular_velocity_body_rps * DVec3::new(400_000.0, 700_000.0, 400_000.0)
        } else {
            DVec3::ZERO
        };
    let rate_moment = DVec3::new(roll, pitch, yaw)
        * if runtime.rcs_enabled {
            105_000.0
        } else {
            35_000.0
        };
    let thrust = DVec3::X * runtime.thrust_n();
    let input = FlightStepInput {
        altitude_m: altitude_m.max(0.0),
        gravity_acceleration_inertial_mps2: gravity,
        position_body_m: position_body,
        // The rigid state is global inertial, while the aero contract is
        // local to the rotating planet. Subtract the planet's ephemeris
        // velocity at this boundary instead of counting it as airspeed.
        wind_velocity_body_mps: orientation_inverse * body_state.velocity_inertial,
        extra_force_body_n: thrust,
        extra_moment_body_nm: sas_moment + trim_moment + contact_moment + rate_moment,
    };

    let Ok((next_state, forces)) = integrate_rigid_body_duration(
        &runtime.aero_model,
        &runtime.vehicle.aero_geometry,
        runtime.atmosphere,
        runtime.state,
        runtime.vehicle.mass_properties,
        input,
        frame_dt,
        0.02,
    ) else {
        runtime.engine_active = false;
        runtime.control_input = DVec3::ZERO;
        return;
    };
    let next_sim_time = SimTime(runtime.flight_time_s + frame_dt);
    let Ok(next_body_state) = ephemeris
        .ephemeris
        .body_state(runtime.reference_body, next_sim_time)
    else {
        return;
    };
    // First playable slice has a spherical datum and no terrain collision
    // mesh. Keep the test craft above the datum and remove only inward radial
    // velocity on contact, so a missed take-off cannot bury the camera.
    let next_relative = next_state.position_inertial_m - next_body_state.position_inertial;
    let next_altitude = next_relative.length() - body.radius_m;
    let next_relative_speed =
        (next_state.velocity_inertial_mps - body_state.velocity_inertial).length();
    if !next_altitude.is_finite()
        || next_altitude.abs() > MAX_PILOT_ALTITUDE_M
        || !next_relative_speed.is_finite()
        || next_relative_speed > MAX_PILOT_RELATIVE_SPEED_MPS
        || !next_state.angular_velocity_body_rps.is_finite()
        || next_state.angular_velocity_body_rps.length() > MAX_PILOT_ANGULAR_RATE_RPS
    {
        // A bad control sample must not poison the camera and HUD with an
        // unbounded state. Keep the last valid state and cut the engine so the
        // pilot can inspect the situation or leave Pilot mode safely.
        runtime.engine_active = false;
        runtime.control_input = DVec3::ZERO;
        return;
    }

    runtime.state = next_state;
    runtime.last_forces = Some(forces);
    runtime.flight_time_s += frame_dt;

    let next_radius = next_relative.length();
    if next_radius < body.radius_m + PILOT_SURFACE_CLEARANCE_M {
        let contact_position = next_relative.try_normalize().unwrap_or(radial_up)
            * (body.radius_m + PILOT_SURFACE_CLEARANCE_M);
        runtime.state.position_inertial_m = next_body_state.position_inertial + contact_position;
        let next_radial_up = contact_position.normalize_or_zero();
        let next_relative_velocity =
            runtime.state.velocity_inertial_mps - next_body_state.velocity_inertial;
        let inward_speed = next_relative_velocity.dot(-next_radial_up);
        if inward_speed > 0.0 {
            runtime.state.velocity_inertial_mps += next_radial_up * inward_speed;
        }
    }

    runtime.render_position = pilot_render_offset(
        (runtime.state.position_inertial_m - next_body_state.position_inertial)
            - runtime.initial_relative_position_m,
    );
    runtime.render_orientation = render_orientation(runtime.state.orientation_body_to_inertial);
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
    let right = pilot_render_offset(orientation * DVec3::Y).normalize_or_zero();
    let up = pilot_render_offset(orientation * DVec3::Z).normalize_or_zero();
    if forward.length_squared() < 1.0e-8
        || right.length_squared() < 1.0e-8
        || up.length_squared() < 1.0e-8
    {
        return Quat::IDENTITY;
    }
    // The model's +Y is its nose, +X its right wing and +Z points down so the
    // three visual axes form a right-handed basis around the engine body axes.
    Quat::from_mat3(&Mat3::from_cols(right, forward, -up))
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
) {
    let mut scroll = 0.0;

    if keys.just_pressed(KeyCode::F6) || keys.just_pressed(KeyCode::KeyP) {
        state.view_mode = match state.view_mode {
            ClientViewMode::Map => ClientViewMode::Pilot,
            ClientViewMode::Pilot => ClientViewMode::Map,
        };
    }

    if state.view_mode == ClientViewMode::Pilot {
        if keys.just_pressed(KeyCode::F8) || keys.just_pressed(KeyCode::Pause) {
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
            state.pilot_camera_yaw -= mouse_delta.x * 0.005;
            state.pilot_camera_pitch =
                (state.pilot_camera_pitch + mouse_delta.y * 0.005).clamp(-0.72, 0.72);
        }
        if mouse_buttons.pressed(MouseButton::Middle) {
            let pan_scale = state.pilot_camera_distance * 0.0018;
            state.pilot_camera_pan += Vec2::new(-mouse_delta.x, mouse_delta.y) * pan_scale;
            state.pilot_camera_pan = state
                .pilot_camera_pan
                .clamp(Vec2::splat(-4.0), Vec2::splat(4.0));
        }
        if scroll != 0.0 {
            state.pilot_camera_distance = (state.pilot_camera_distance * (-scroll * 0.09).exp())
                .clamp(PILOT_CAMERA_MIN_DISTANCE_M, PILOT_CAMERA_MAX_DISTANCE_M);
        }
        if keys.just_pressed(KeyCode::Home) {
            state.pilot_camera_yaw = 0.0;
            state.pilot_camera_pitch = 0.0;
            state.pilot_camera_distance = PILOT_CAMERA_DEFAULT_DISTANCE_M;
            state.pilot_camera_pan = Vec2::ZERO;
        }
        let reverse = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        if keys.just_pressed(KeyCode::KeyM) {
            state.control_mode = if reverse {
                state.control_mode.previous()
            } else {
                state.control_mode.next()
            };
        }
        if keys.just_pressed(KeyCode::KeyV) {
            state.speed_frame = state.speed_frame.next();
        }
        if keys.just_pressed(KeyCode::KeyB) {
            state.altitude_frame = state.altitude_frame.toggle();
        }

        let dt = f64::from(time.delta_secs().clamp(0.0, 0.1));
        // KSP convention: W pitches the nose up, S pitches it down.
        let keyboard_pitch =
            (keys.pressed(KeyCode::KeyW) as i8 - keys.pressed(KeyCode::KeyS) as i8) as f64;
        let keyboard_yaw =
            (keys.pressed(KeyCode::KeyD) as i8 - keys.pressed(KeyCode::KeyA) as i8) as f64;
        let keyboard_roll =
            (keys.pressed(KeyCode::KeyE) as i8 - keys.pressed(KeyCode::KeyQ) as i8) as f64;
        let keyboard_input = DVec3::new(keyboard_pitch, keyboard_yaw, keyboard_roll);
        // Orbit/pan gestures belong exclusively to the camera. They must not
        // simultaneously command Mouse Aim, otherwise dragging the view also
        // deflects the aircraft and makes the controls feel broken.
        let camera_gesture =
            mouse_buttons.pressed(MouseButton::Right) || mouse_buttons.pressed(MouseButton::Middle);
        let mouse_input = if camera_gesture {
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
        if state.control_mode == ControlMode::Navball && runtime.sas_enabled {
            // In KSP/SAS mode a held key changes the attitude target; after
            // release SAS keeps the last target instead of snapping back to
            // the spawn attitude or requiring a continuously held key.
            // Body axes are +X roll, +Y pitch, +Z yaw. W/S pitch, A/D yaw and
            // Q/E roll must map to those axes in the same order as the direct
            // rate controller; the old mapping mixed all three channels.
            let target_axis = DVec3::new(command_input.z, -command_input.x, command_input.y);
            runtime.sas_target_orientation = (runtime.sas_target_orientation
                * DQuat::from_scaled_axis(target_axis * (0.72 * dt)))
            .normalize();
            runtime.control_input = DVec3::ZERO;
        } else {
            runtime.control_input = command_input;
        }

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
        let cursor = window.cursor_position().unwrap_or(size * 0.5);
        let ndc = Vec2::new(
            (cursor.x / size.x * 2.0 - 1.0).clamp(-1.0, 1.0),
            (1.0 - cursor.y / size.y * 2.0).clamp(-1.0, 1.0),
        );
        state.mouse_position = cursor;
        // Guidance/FBW will consume this boundary later. It never writes an
        // actuator command and does not alter authoritative physics.
        state.desired_direction = Vec3::new(ndc.x, ndc.y, -1.0).normalize();
    }
}

fn update_flight_ui_state(
    clock: Res<SimulationClock>,
    map: Res<MapState>,
    runtime: Res<RuntimeEphemeris>,
    flight_runtime: Res<PilotFlightRuntime>,
    mut state: ResMut<PilotHudState>,
) {
    let reference_body = runtime
        .ephemeris
        .body(flight_runtime.reference_body)
        .ok()
        .map(|body| body.name.to_uppercase());
    state.flight = live_flight_ui_state(
        &flight_runtime,
        &runtime.ephemeris,
        reference_body,
        clock.sim_seconds,
    );
    // `map` remains in the signature deliberately: the flight HUD and map
    // share the same body selection resource, but a pilot always flies the
    // selected playable world rather than the current map zoom focus.
    let _ = map;
}

fn live_flight_ui_state(
    flight: &PilotFlightRuntime,
    ephemeris: &BakedEphemeris,
    reference_body_name: Option<String>,
    _map_time_s: f64,
) -> FlightUiState {
    let Ok(body_state) = ephemeris.body_state(flight.reference_body, SimTime(flight.flight_time_s))
    else {
        return FlightUiState::default();
    };
    let Ok(body) = ephemeris.body(flight.reference_body) else {
        return FlightUiState::default();
    };
    let relative_position = flight.state.position_inertial_m - body_state.position_inertial;
    let radial_up = relative_position.try_normalize().unwrap_or(DVec3::Z);
    let orientation_inverse = flight.state.orientation_body_to_inertial.inverse();
    let position_body = orientation_inverse * relative_position;
    let velocity_relative_inertial =
        flight.state.velocity_inertial_mps - body_state.velocity_inertial;
    let atmosphere_rotation_body = flight.atmosphere.body_rotation_rad_s.cross(position_body);
    let atmosphere_rotation_inertial =
        flight.state.orientation_body_to_inertial * atmosphere_rotation_body;
    let velocity_surface = velocity_relative_inertial - atmosphere_rotation_inertial;
    let velocity_body = orientation_inverse * velocity_relative_inertial;
    let forces = flight.last_forces.as_ref();
    let environment = forces.map(|forces| forces.environment);
    // The solver's environment contains body translation plus body rotation;
    // use the same explicitly separated terms for the HUD. This prevents the
    // moon's orbital velocity from appearing as a 40 km/s airspeed sample.
    let air_velocity = velocity_body - atmosphere_rotation_body;
    let air_speed = air_velocity.length();
    let surface_speed = velocity_surface.length();
    let forward = flight.state.orientation_body_to_inertial * DVec3::X;
    let right = flight.state.orientation_body_to_inertial * DVec3::Y;
    let up = flight.state.orientation_body_to_inertial * DVec3::Z;
    let heading_deg = forward.x.atan2(forward.y).to_degrees().rem_euclid(360.0);
    let pitch_deg = forward.dot(radial_up).clamp(-1.0, 1.0).asin().to_degrees();
    let roll_deg = right
        .dot(radial_up)
        .atan2(up.dot(radial_up).max(1.0e-6))
        .to_degrees();
    let altitude_m = relative_position.length() - body.radius_m;
    let vertical_speed = velocity_surface.dot(radial_up);
    let gravity = body.mu / relative_position.length().max(1.0).powi(2);
    let force_magnitude = forces
        .map(|forces| forces.total_force_inertial_n.length())
        .unwrap_or_else(|| flight.thrust_n());
    let g_load = (force_magnitude / flight.vehicle.mass_properties.mass_kg / gravity).max(0.0);
    let (apoapsis_altitude_m, periapsis_altitude_m) =
        estimate_orbit_altitudes(relative_position, velocity_surface, body.mu, body.radius_m);

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
            reference_body: reference_body_name,
            atmosphere_available: true,
            terrain_available: false,
            pressure_pa: environment.map(|environment| {
                // The atmosphere sample is deterministic; q and Mach come
                // from the same environment, avoiding a second atmosphere
                // approximation in the display path.
                environment.density_kg_m3
                    * flight.atmosphere.gas_constant_j_kg_k
                    * (environment.speed_of_sound_mps.powi(2)
                        / flight.atmosphere.heat_capacity_ratio)
            }),
            density_kg_m3: environment.map(|environment| environment.density_kg_m3),
        },
        surface_velocity_mps: velocity_surface,
        gravity_acceleration_mps2: flight.last_gravity_acceleration_inertial_mps2,
        local_up_body: flight.state.orientation_body_to_inertial.inverse() * radial_up,
        surface_speed_m_s: Some(surface_speed),
        air_speed_m_s: Some(air_speed),
        orbital_speed_m_s: Some(velocity_relative_inertial.length()),
        target_speed_m_s: None,
        altitude_datum_m: Some(altitude_m.max(0.0)),
        // Until terrain is implemented, the spherical body datum is the
        // actual collision surface and therefore also the honest AGL value.
        altitude_agl_m: Some(altitude_m.max(0.0)),
        vertical_speed_m_s: Some(vertical_speed),
        heading_deg: Some(heading_deg),
        pitch_deg: Some(pitch_deg),
        roll_deg: Some(roll_deg),
        mach: forces.map(|forces| forces.aero.mach),
        dynamic_pressure_pa: forces.map(|forces| forces.aero.dynamic_pressure_pa),
        angle_of_attack_deg: Some(air_velocity.z.atan2(air_velocity.x).to_degrees()),
        sideslip_deg: Some(
            air_velocity
                .y
                .atan2(air_velocity.x.abs().max(1.0e-6))
                .to_degrees(),
        ),
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
        guidance_mode: Some(
            match flight.sas_enabled {
                true => "SAS / PILOT",
                false => "MANUAL PILOT",
            }
            .into(),
        ),
        warnings: Vec::new(),
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
        return (f64::INFINITY, -radius_m);
    }
    let semi_major_axis = -mu / (2.0 * specific_energy);
    let eccentricity_vector = velocity.cross(position.cross(velocity)) / mu - position / distance;
    let eccentricity = eccentricity_vector.length().clamp(0.0, 0.999_999);
    (
        (semi_major_axis * (1.0 + eccentricity) - radius_m).max(0.0),
        (semi_major_axis * (1.0 - eccentricity) - radius_m).max(0.0),
    )
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn update_pilot_hud(
    state: Res<PilotHudState>,
    clock: Res<SimulationClock>,
    window: Single<&Window, With<PrimaryWindow>>,
    navball_image: Res<PilotNavballImage>,
    mut images: ResMut<Assets<Image>>,
    mut roots: Query<&mut Visibility, With<PilotHudRoot>>,
    mut readouts: ParamSet<(
        Query<(&mut Text, &PilotReadout)>,
        Query<&mut Text, With<PilotHeadingMarker>>,
    )>,
    mut nodes: ParamSet<(
        Query<&mut Node, With<PilotAimReticle>>,
        Query<(&mut Node, &PilotTapeMarker)>,
        Query<&mut UiTransform, With<PilotRollIndicator>>,
        Query<&mut Node, With<PilotFlightPathMarker>>,
        Query<(&mut Node, &PilotNavballMarker)>,
    )>,
) {
    let visible = state.view_mode == ClientViewMode::Pilot;
    for mut visibility in &mut roots {
        *visibility = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    if !visible {
        return;
    }

    let cursor = state.mouse_position;
    let cursor_size = window.resolution.size();
    for mut node in nodes.p0().iter_mut() {
        node.left = px(cursor.x - 12.0);
        node.top = px(cursor.y - 12.0);
        node.margin = UiRect::ZERO;
    }
    // Keep the flight-path cue independent from the mouse cursor. It remains
    // stable and readable while the live telemetry catches up each frame.
    for mut node in nodes.p3().iter_mut() {
        node.left = px(cursor_size.x * 0.5 + 48.0);
        node.top = px(cursor_size.y * 0.5 - 34.0);
        node.margin = UiRect::ZERO;
    }

    let flight = &state.flight;
    if let Some(mut image) = images.get_mut(&navball_image.0) {
        update_navball_image(&mut image, flight.local_up_body);
    }
    for (mut node, marker) in nodes.p1().iter_mut() {
        let position = if marker.altitude {
            flight
                .altitude_datum_m
                .map(|value| 1.0 - (value / 100_000.0).clamp(0.0, 1.0))
        } else {
            primary_speed(flight, state.speed_frame)
                .map(|value| 1.0 - ((value - 200.0) / 300.0).clamp(0.0, 1.0))
        };
        node.top = px(8.0 + position.unwrap_or(0.5) as f32 * 132.0);
    }
    let roll = (flight.roll_deg.unwrap_or(0.0) as f32).to_radians();
    for mut transform in nodes.p2().iter_mut() {
        transform.rotation = Rot2::radians(-roll);
    }
    for (mut node, marker) in nodes.p4().iter_mut() {
        let position = navball_marker_position(flight, marker.vector);
        node.left = px(position.x - 8.0);
        node.top = px(position.y - 8.0);
    }
    for mut text in readouts.p1().iter_mut() {
        **text = heading_cardinal(flight.heading_deg).into();
    }
    let (days, hours, minutes, seconds) = format_sim_time(flight.sim_time_s);
    for (mut text, readout) in readouts.p0().iter_mut() {
        let content = match readout {
            PilotReadout::Header => format!(
                "SIM T+{:02}d {:02}h {:02}m {:02}s\n{}  /  {}",
                days,
                hours,
                minutes,
                seconds,
                match flight.source {
                    TelemetrySource::Live => "CONNECTED",
                    TelemetrySource::DemoPreview => "DEMO TELEMETRY",
                },
                flight
                    .environment
                    .reference_body
                    .as_deref()
                    .unwrap_or("NO REFERENCE"),
            ),
            PilotReadout::Control => format!(
                "{} [ON]\n{}\nSAS [{}]  RCS [{}]\nW/S pitch A/D yaw Q/E roll\nSHIFT/CTRL throttle  X/Z engine  Space stage\nF8 pause / resume",
                state.control_mode.label(),
                state.control_mode.description(),
                if flight.sas_enabled { "ON" } else { "OFF" },
                if flight.rcs_enabled { "ON" } else { "OFF" },
            ),
            PilotReadout::SpeedValue => {
                format_speed_value(primary_speed(flight, state.speed_frame))
            }
            PilotReadout::SpeedDetail => format_speed_detail(flight, state.speed_frame),
            PilotReadout::AltitudeValue => {
                format_altitude_value(primary_altitude(flight, state.altitude_frame))
            }
            PilotReadout::AltitudeDetail => format_altitude_detail(flight, state.altitude_frame),
            PilotReadout::Vehicle => {
                if flight.environment.atmosphere_available {
                    format!(
                        "{}\nTHR {}  G {}\nTWR {}  F {}\nGEAR [{}]  SAS [{}]\nENGINE [{}]  RCS [{}]",
                        flight.vehicle_name.as_deref().unwrap_or("NO VEHICLE"),
                        format_percent(flight.throttle),
                        format_scalar(flight.g_load, 2),
                        format_scalar(flight.twr, 2),
                        format_force(flight.thrust_n),
                        if flight.gear_down { "DOWN" } else { "UP" },
                        if flight.sas_enabled { "ON" } else { "OFF" },
                        if flight.engine_active { "ON" } else { "OFF" },
                        if flight.rcs_enabled { "ON" } else { "OFF" },
                    )
                } else {
                    format!(
                        "{}\nAP {}  PE {}\nT+AP {}  T+PE {}\nRCS [ON]",
                        flight.vehicle_name.as_deref().unwrap_or("NO VEHICLE"),
                        format_altitude_value(flight.apoapsis_altitude_m),
                        format_altitude_value(flight.periapsis_altitude_m),
                        format_duration(flight.time_to_apoapsis_s),
                        format_duration(flight.time_to_periapsis_s),
                    )
                }
            }
            PilotReadout::Context => format!(
                "REF {}\nTARGET {}\nRANGE {}\nCLOSING {}\nAP {}  PE {}\n[0] director  [1] target",
                flight.environment.reference_body.as_deref().unwrap_or("--"),
                flight.target_name.as_deref().unwrap_or("--"),
                format_distance_short(flight.target_distance_m),
                format_speed_value(flight.target_closing_speed_m_s),
                format_altitude_value(flight.apoapsis_altitude_m),
                format_altitude_value(flight.periapsis_altitude_m),
            ),
            PilotReadout::Navball => format!(
                "{}  |  {}\nO PRO  X RET  + TGT  G GRV",
                attitude_frame_label(flight.attitude_frame),
                state.speed_frame.label(),
            ),
            PilotReadout::NavballFooter => match flight.source {
                TelemetrySource::Live => "LOCAL  |  CONNECTED".into(),
                TelemetrySource::DemoPreview => "LOCAL  |  DEMO PREVIEW".into(),
            },
            PilotReadout::Status => match flight.source {
                TelemetrySource::Live => format!(
                    "LIVE TELEMETRY\n{} / GUIDANCE ADAPTER READY",
                    if clock.paused { "PAUSED" } else { "RUNNING" }
                ),
                TelemetrySource::DemoPreview => "DISPLAY ONLY\nNO GUIDANCE / NO ACTUATORS".into(),
            },
        };
        **text = content;
    }

    debug_assert!(cursor_size.x >= 0.0 && cursor_size.y >= 0.0);
}

fn navball_marker_position(flight: &FlightUiState, vector: NavballVector) -> Vec2 {
    let inertial_vector = match vector {
        NavballVector::Prograde | NavballVector::Target => flight.surface_velocity_mps,
        NavballVector::Retrograde => -flight.surface_velocity_mps,
        NavballVector::Gravity => flight.gravity_acceleration_mps2,
    };
    let direction = inertial_vector.normalize_or_zero();
    if direction.length_squared() <= 1.0e-8 {
        return Vec2::new(180.0, 180.0);
    }
    let body_direction = (flight.orientation.inverse() * direction).normalize_or_zero();
    // The center of this navball is the nose (+X). Perspective projection
    // keeps markers on the sphere instead of pinning them to static debug
    // coordinates. Directions behind the nose remain visible at the rim,
    // matching the useful “approaching the edge” cue of KSP's navball.
    let depth = body_direction.x.abs().max(0.28);
    let scale = 126.0 / depth;
    Vec2::new(
        ((180.0 + body_direction.y * scale).clamp(32.0, 328.0)) as f32,
        ((180.0 - body_direction.z * scale).clamp(32.0, 328.0)) as f32,
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

fn format_speed_detail(flight: &FlightUiState, selected: SpeedFrame) -> String {
    let mut lines = Vec::with_capacity(8);
    for (frame, label, value) in [
        (SpeedFrame::Surface, "SURF", flight.surface_speed_m_s),
        (SpeedFrame::Air, "AIR", flight.air_speed_m_s),
        (SpeedFrame::Orbital, "ORBIT", flight.orbital_speed_m_s),
        (SpeedFrame::Target, "TGT", flight.target_speed_m_s),
    ] {
        if frame != selected {
            lines.push(format!("{label:<6}{}", format_speed_value(value)));
        }
    }
    lines.push(format!("MACH  {}", format_scalar(flight.mach, 2)));
    lines.push(format!(
        "AoA   {}",
        format_angle(flight.angle_of_attack_deg)
    ));
    lines.push(format!(
        "Q     {}",
        format_pressure(flight.dynamic_pressure_pa)
    ));
    lines.join("\n")
}

fn format_altitude_detail(flight: &FlightUiState, selected: AltitudeFrame) -> String {
    let mut lines = Vec::with_capacity(5);
    if selected != AltitudeFrame::Datum {
        lines.push(format!(
            "DATUM {}",
            format_altitude_value(flight.altitude_datum_m)
        ));
    }
    if selected != AltitudeFrame::Agl {
        lines.push(format!(
            "AGL   {}",
            format_altitude_value(flight.altitude_agl_m)
        ));
    }
    lines.push(format!(
        "V/S   {}",
        format_speed_value(flight.vertical_speed_m_s)
    ));
    lines.push(String::new());
    lines.push("[B] datum / AGL".into());
    lines.join("\n")
}

fn attitude_frame_label(frame: AttitudeFrame) -> &'static str {
    match frame {
        AttitudeFrame::Local => "LOCAL",
        AttitudeFrame::Orbit => "ORBIT",
        AttitudeFrame::Target => "TARGET",
        AttitudeFrame::Inertial => "INERTIAL",
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

#[cfg(test)]
fn format_distance(value_m: Option<f64>) -> String {
    value_m
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() >= 100_000.0 {
                format!("{:.1} km", value / 1_000.0)
            } else {
                format!("{value:.0} m")
            }
        })
        .unwrap_or_else(|| "--".into())
}

#[cfg(test)]
fn format_speed(value_m_s: Option<f64>) -> String {
    value_m_s
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1} m/s"))
        .unwrap_or_else(|| "--".into())
}

fn format_speed_value(value_m_s: Option<f64>) -> String {
    value_m_s
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1}"))
        .unwrap_or_else(|| "--".into())
}

fn format_altitude_value(value_m: Option<f64>) -> String {
    value_m
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() < 1_000.0 {
                format!("{value:.0} m")
            } else {
                format!("{:.1} km", value / 1_000.0)
            }
        })
        .unwrap_or_else(|| "--".into())
}

fn format_distance_short(value_m: Option<f64>) -> String {
    value_m
        .filter(|value| value.is_finite())
        .map(|value| format!("{:.1} km", value / 1_000.0))
        .unwrap_or_else(|| "--".into())
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

fn format_duration(value_s: Option<f64>) -> String {
    value_s
        .filter(|value| value.is_finite() && *value >= 0.0)
        .map(|value| {
            let total = value.round() as u64;
            format!("{:02}:{:02}", total / 60, total % 60)
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
    use thessa_sim_core::integrate_rigid_body_step;

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
    fn selected_primary_values_are_not_repeated_in_detail_readouts() {
        let preview = FlightUiState::demo_preview(0.0, Some("NEREID".into()));
        let speed = format_speed_detail(&preview, SpeedFrame::Surface);
        let altitude = format_altitude_detail(&preview, AltitudeFrame::Datum);
        assert!(!speed.contains("SURF"));
        assert!(!altitude.contains("DATUM 82.4"));
        assert!(!altitude.contains("HDG"));
        assert!(!altitude.contains("PITCH"));
        assert!(!altitude.contains("ROLL"));
    }

    #[test]
    fn missing_preview_values_are_not_formatted_as_physics() {
        assert_eq!(format_speed(None), "--");
        assert_eq!(format_distance(None), "--");
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
        assert!((degrees(panels[2].control_deflection_rad) - 12.5).abs() < 1.0e-10);
        assert!((degrees(panels[3].control_deflection_rad) - 12.5).abs() < 1.0e-10);
        assert!((degrees(panels[4].control_deflection_rad) + 5.5).abs() < 1.0e-10);
        assert!((degrees(panels[0].control_deflection_rad) + 13.5).abs() < 1.0e-10);
        assert!((degrees(panels[1].control_deflection_rad) - 13.5).abs() < 1.0e-10);
    }

    #[test]
    fn surface_altitude_keeps_meter_resolution() {
        assert_eq!(format_altitude_value(Some(25.0)), "25 m");
        assert_eq!(format_altitude_value(Some(1_300.0)), "1.3 km");
    }

    #[test]
    fn x15_runtime_stays_finite_during_a_long_controlled_run() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let reference_body = ephemeris.body_id("thessa").expect("playable body exists");
        let mut flight =
            PilotFlightRuntime::new(&ephemeris, reference_body).expect("X-15 runtime initializes");
        let body = ephemeris.body(reference_body).expect("body exists");
        for step in 0..30_000 {
            let body_state = ephemeris
                .body_state(reference_body, SimTime(flight.flight_time_s))
                .expect("body state evaluates");
            let relative = flight.state.position_inertial_m - body_state.position_inertial;
            let gravity = GravityField::from_ephemeris(&ephemeris)
                .acceleration(
                    flight.state.position_inertial_m,
                    SimTime(flight.flight_time_s),
                )
                .expect("gravity evaluates");
            let phase = step as f64 * 0.013;
            let pitch = phase.sin();
            let yaw = (phase * 0.73).cos();
            let roll = (phase * 1.31).sin();
            flight.command_controls(pitch, yaw, roll);
            let radial_up = relative.try_normalize().unwrap_or(DVec3::Z);
            let body_up = flight.state.orientation_body_to_inertial * DVec3::Z;
            let attitude_error_body =
                flight.state.orientation_body_to_inertial.inverse() * body_up.cross(radial_up);
            let sas_moment = -flight.state.angular_velocity_body_rps * 42_000.0
                + attitude_error_body * 260_000.0;
            let rate_moment = DVec3::new(roll, pitch, yaw) * 105_000.0;
            let (state, forces) = integrate_rigid_body_step(
                &flight.aero_model,
                &flight.vehicle.aero_geometry,
                flight.atmosphere,
                flight.state,
                flight.vehicle.mass_properties,
                FlightStepInput {
                    altitude_m: (relative.length() - body.radius_m).max(0.0),
                    gravity_acceleration_inertial_mps2: gravity,
                    position_body_m: relative,
                    wind_velocity_body_mps: flight.state.orientation_body_to_inertial.inverse()
                        * body_state.velocity_inertial,
                    extra_force_body_n: DVec3::X * flight.thrust_n(),
                    extra_moment_body_nm: sas_moment + rate_moment,
                },
                0.02,
            )
            .expect("X-15 step evaluates");
            assert!(state.position_inertial_m.is_finite());
            assert!(state.velocity_inertial_mps.is_finite());
            assert!(forces.aero.mach.is_finite());
            assert!(
                state.velocity_inertial_mps.length() < 1.0e6,
                "X-15 diverged at step {step}: speed={} m/s mach={} altitude={} m",
                state.velocity_inertial_mps.length(),
                forces.aero.mach,
                relative.length() - body.radius_m,
            );
            flight.state = state;
            flight.flight_time_s += 0.02;
        }
    }

    #[test]
    fn x15_surface_departure_does_not_spin_without_input() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("checked-in system config parses");
        let ephemeris = config.bake().expect("checked-in system bakes");
        let reference_body = ephemeris.body_id("thessa").expect("playable body exists");
        let mut flight =
            PilotFlightRuntime::new(&ephemeris, reference_body).expect("X-15 runtime initializes");
        let body = ephemeris.body(reference_body).expect("body exists");
        for step in 0..1_500 {
            let body_state = ephemeris
                .body_state(reference_body, SimTime(flight.flight_time_s))
                .expect("body state evaluates");
            let relative = flight.state.position_inertial_m - body_state.position_inertial;
            let gravity = GravityField::from_ephemeris(&ephemeris)
                .acceleration(
                    flight.state.position_inertial_m,
                    SimTime(flight.flight_time_s),
                )
                .expect("gravity evaluates");
            flight.command_controls(0.0, 0.0, 0.0);
            let orientation_inverse = flight.state.orientation_body_to_inertial.inverse();
            let position_body = orientation_inverse * relative;
            let relative_velocity_inertial =
                flight.state.velocity_inertial_mps - body_state.velocity_inertial;
            let air_velocity_body = orientation_inverse * relative_velocity_inertial
                - flight.atmosphere.body_rotation_rad_s.cross(position_body);
            let radial_up = relative.try_normalize().unwrap_or(DVec3::Z);
            let sas_moment =
                pilot_sas_moment(flight.state, flight.sas_target_orientation, radial_up);
            let trim_moment = pilot_aero_trim_moment(air_velocity_body);
            let contact_moment = if relative.length() - body.radius_m <= 5.5
                && relative_velocity_inertial.dot(radial_up) <= 0.0
            {
                -flight.state.angular_velocity_body_rps
                    * DVec3::new(400_000.0, 700_000.0, 400_000.0)
            } else {
                DVec3::ZERO
            };
            let (state, _forces) = integrate_rigid_body_step(
                &flight.aero_model,
                &flight.vehicle.aero_geometry,
                flight.atmosphere,
                flight.state,
                flight.vehicle.mass_properties,
                FlightStepInput {
                    altitude_m: (relative.length() - body.radius_m).max(0.0),
                    gravity_acceleration_inertial_mps2: gravity,
                    position_body_m: position_body,
                    wind_velocity_body_mps: orientation_inverse * body_state.velocity_inertial,
                    extra_force_body_n: DVec3::X * flight.thrust_n(),
                    extra_moment_body_nm: sas_moment + trim_moment + contact_moment,
                },
                0.02,
            )
            .expect("X-15 attitude step evaluates");
            let next_body_state = ephemeris
                .body_state(reference_body, SimTime(flight.flight_time_s + 0.02))
                .expect("next body state evaluates");
            let next_relative = state.position_inertial_m - next_body_state.position_inertial;
            if next_relative.length() < body.radius_m + PILOT_SURFACE_CLEARANCE_M {
                let contact_position = next_relative.try_normalize().unwrap_or(radial_up)
                    * (body.radius_m + PILOT_SURFACE_CLEARANCE_M);
                let mut state = state;
                state.position_inertial_m = next_body_state.position_inertial + contact_position;
                let next_relative_velocity =
                    state.velocity_inertial_mps - next_body_state.velocity_inertial;
                let inward_speed = next_relative_velocity.dot(-contact_position.normalize());
                if inward_speed > 0.0 {
                    state.velocity_inertial_mps += contact_position.normalize() * inward_speed;
                }
                flight.state = state;
            } else {
                flight.state = state;
            }
            assert!(
                flight.state.angular_velocity_body_rps.length() < 0.35,
                "X-15 uncommanded attitude rate exceeded 0.35 rad/s at step {step}: {:?}",
                flight.state.angular_velocity_body_rps
            );
            flight.flight_time_s += 0.02;
        }
    }
}
