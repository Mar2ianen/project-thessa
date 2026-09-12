use super::*;

mod control;
mod hud;
use hud::{pilot_hud_buttons, spawn_pilot_hud, update_pilot_hud};

use std::{
    fs::{File, create_dir_all},
    io::{BufWriter, Write},
    path::Path,
};

use bevy::{
    asset::RenderAssetUsages,
    image::Image,
    math::{DQuat, DVec3, Mat3},
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    ui::FocusPolicy,
    world_serialization::{WorldAsset, WorldAssetRoot},
};

use thessa_sim_core::{
    AeroConfig, AtmosphereConfig, AtmosphereError, BakedEphemeris, BodyId, BodyState,
    EventScheduler, FlightError, FlightForces, FlightStepInput, GravityField, OnRailsCache,
    PanelAeroModel, RigidBodyState, ScheduledKind, VehicleDefinition,
};

const HUD_TEXT: Color = Color::srgb(0.91, 0.95, 0.98);
const HUD_MUTED: Color = Color::srgb(0.60, 0.69, 0.76);
const HUD_GREEN: Color = Color::srgb(0.43, 0.87, 0.73);
const HUD_AMBER: Color = Color::srgb(0.98, 0.72, 0.26);
// Pilot preview coordinates are metres around the launch site. Unlike the
// system map's readability curve, this scene keeps the body's authored radius
// and the imported X-15 mesh in the same unit system.
const X15_STALL_ANGLE_DEG: f64 = 22.0;
const PILOT_SURFACE_CLEARANCE_M: f64 = 5.0;
const PILOT_START_ALTITUDE_M: f64 = 500.0;
const PILOT_CAMERA_DEFAULT_DISTANCE_M: f32 = 32.0;
const PILOT_CAMERA_MIN_DISTANCE_M: f32 = 8.0;
const PILOT_CAMERA_MAX_DISTANCE_M: f32 = 180.0;
const X15_SOURCE_LENGTH_M: f32 = 16.77;
const X15_AUTHORED_LENGTH_M: f32 = 15.45;
// Solver guard rail, not physics: interlunar transfers range billions of
// metres from the reference body (outermost catalogued orbit sits near
// 6e9 m), so the cap covers the whole Nereid system plus escape margin.
// f64 inertial state holds precision far beyond it; rendering floats on its
// own origin. Tripping it still latches FLIGHT STOPPED, recoverable with
// Backspace.
const MAX_PILOT_ALTITUDE_M: f64 = 2.0e10;
const MAX_PILOT_RELATIVE_SPEED_MPS: f64 = 50_000.0;
const MAX_PILOT_ANGULAR_RATE_RPS: f64 = 25.0;
// KSP-style attitude keys change the SAS target at a pilotable rate. The old
// 0.72 rad/s value moved the target by more than 40 degrees every second and
// drove the X-15 straight through stall before a player could trim it.
const PILOT_ATTITUDE_COMMAND_RATE_RAD_S: f64 = 0.16;

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

    #[cfg(test)]
    fn next(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + 1) % Self::ALL.len()]
    }

    #[cfg(test)]
    fn previous(self) -> Self {
        let index = Self::ALL.iter().position(|mode| *mode == self).unwrap_or(0);
        Self::ALL[(index + Self::ALL.len() - 1) % Self::ALL.len()]
    }

    fn label(self) -> &'static str {
        match self {
            Self::MouseAim => "MOUSE STEERING",
            Self::Navball => "ATTITUDE HOLD",
            Self::Rate => "RATE CONTROL",
            Self::Direct => "DIRECT / RAW",
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::MouseAim => "Cursor commands pitch/yaw rate; center to hold.",
            Self::Navball => "direct attitude target control",
            Self::Rate => "command angular rates",
            Self::Direct => "raw actuator input",
        }
    }
}

/// Solver regime of the flown vehicle: dense-air 6-DoF flight vs vacuum coast.
///
/// Coast is a solver optimization, never a physics switch: translation stays
/// full multi-body gravity plus thrust, rotation stays torque-free plus RCS.
/// Below the density threshold aero moments are orders of magnitude under RCS
/// authority, so the trim solver freezes the surfaces instead of chasing a
/// near-singular effectiveness matrix. The threshold is a density, not an
/// altitude, so it follows any atmosphere the config provides.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) enum FlightRegime {
    #[default]
    Aero,
    Coast,
}

impl FlightRegime {
    fn label(self) -> &'static str {
        match self {
            Self::Aero => "AERO",
            Self::Coast => "COAST",
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

#[derive(Debug, Clone, Copy)]
struct LocalAirKinematics {
    relative_position_inertial_m: DVec3,
    relative_position_body_m: DVec3,
    relative_velocity_inertial_mps: DVec3,
    air_velocity_body_mps: DVec3,
    surface_velocity_inertial_mps: DVec3,
    radial_up: DVec3,
    altitude_m: f64,
}

/// Sample all local flight kinematics from one rigid-body state and one
/// ephemeris body state. The rotating atmosphere vector is converted into the
/// vehicle frame by sim-core; callers must not cross a body-frame position with
/// an inertial-frame angular velocity directly.
fn local_air_kinematics(
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    body_state: BodyState,
    body_radius_m: f64,
) -> Result<LocalAirKinematics, AtmosphereError> {
    let relative_position_inertial_m = state.position_inertial_m - body_state.position_inertial;
    let radial_up = relative_position_inertial_m
        .try_normalize()
        .unwrap_or(DVec3::Z);
    let orientation_inverse = state.orientation_body_to_inertial.inverse();
    let relative_position_body_m = orientation_inverse * relative_position_inertial_m;
    let relative_velocity_inertial_mps = state.velocity_inertial_mps - body_state.velocity_inertial;
    let relative_velocity_body_mps = orientation_inverse * relative_velocity_inertial_mps;
    let rotating_air_velocity_body_mps = atmosphere.rotating_air_velocity_body_mps(
        relative_position_body_m,
        state.orientation_body_to_inertial,
    )?;
    let air_velocity_body_mps = relative_velocity_body_mps - rotating_air_velocity_body_mps;
    let surface_velocity_inertial_mps = relative_velocity_inertial_mps
        - state.orientation_body_to_inertial * rotating_air_velocity_body_mps;
    let altitude_m = relative_position_inertial_m.length() - body_radius_m;
    Ok(LocalAirKinematics {
        relative_position_inertial_m,
        relative_position_body_m,
        relative_velocity_inertial_mps,
        air_velocity_body_mps,
        surface_velocity_inertial_mps,
        radial_up,
        altitude_m,
    })
}

/// Low-overhead CSV recorder for reproducing bad live-flight states outside
/// the renderer. It is enabled only by the executable; unit tests remain
/// side-effect free.
struct FlightTraceWriter {
    writer: BufWriter<File>,
    samples_since_flush: u32,
}

impl FlightTraceWriter {
    fn create(path: &Path) -> Option<Self> {
        if let Some(parent) = path.parent() {
            create_dir_all(parent).ok()?;
        }
        let file = File::create(path).ok()?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "t_s,altitude_m,relative_speed_mps,vertical_speed_mps,mach,aoa_deg,q_pa,\
             pos_x_m,pos_y_m,pos_z_m,vel_x_mps,vel_y_mps,vel_z_mps,\
             quat_x,quat_y,quat_z,quat_w,omega_x_rps,omega_y_rps,omega_z_rps,\
             pitch_cmd,yaw_cmd,roll_cmd,throttle,engine,sas,rcs,\
             force_x_n,force_y_n,force_z_n,moment_x_nm,moment_y_nm,moment_z_nm,\
             accel_x_mps2,accel_y_mps2,accel_z_mps2,control_mode,surface_pitch,surface_yaw,surface_roll,actuator_saturated,target_quat_x,target_quat_y,target_quat_z,target_quat_w"
        )
        .ok()?;
        Some(Self {
            writer,
            samples_since_flush: 0,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn record(
        &mut self,
        time_s: f64,
        altitude_m: f64,
        relative_velocity_inertial_mps: DVec3,
        radial_up: DVec3,
        air_velocity_body_mps: DVec3,
        state: RigidBodyState,
        controls: DVec3,
        throttle: f64,
        engine_active: bool,
        sas_enabled: bool,
        rcs_enabled: bool,
        forces: &FlightForces,
        mode: ControlMode,
        surfaces: DVec3,
        saturated: bool,
        target: DQuat,
    ) {
        let aoa_deg = conventional_angle_of_attack_deg(air_velocity_body_mps);
        let vertical_speed_mps = relative_velocity_inertial_mps.dot(radial_up);
        let q = forces.aero.dynamic_pressure_pa;
        let p = state.position_inertial_m;
        let v = state.velocity_inertial_mps;
        let qrot = state.orientation_body_to_inertial;
        let omega = state.angular_velocity_body_rps;
        let force = forces.total_force_body_n;
        let moment = forces.total_moment_body_nm;
        let accel = forces.acceleration_inertial_mps2;
        let _ = writeln!(
            self.writer,
            "{time_s:.6},{altitude_m:.6},{:.6},{vertical_speed_mps:.6},{:.6},{aoa_deg:.6},{q:.6},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.9},\
             {:.9},{:.9},{:.9},{:.6},{:.6},{:.6},{throttle:.6},{},{},{},\
             {:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{},{:.9},{:.9},{:.9},{:.9}",
            relative_velocity_inertial_mps.length(),
            forces.aero.mach,
            p.x,
            p.y,
            p.z,
            v.x,
            v.y,
            v.z,
            qrot.x,
            qrot.y,
            qrot.z,
            qrot.w,
            omega.x,
            omega.y,
            omega.z,
            controls.x,
            controls.y,
            controls.z,
            engine_active as u8,
            sas_enabled as u8,
            rcs_enabled as u8,
            force.x,
            force.y,
            force.z,
            moment.x,
            moment.y,
            moment.z,
            accel.x,
            accel.y,
            accel.z,
            mode.label(),
            surfaces.x,
            surfaces.y,
            surfaces.z,
            saturated as u8,
            target.x,
            target.y,
            target.z,
            target.w,
        );
        self.samples_since_flush += 1;
        if self.samples_since_flush >= 30 {
            let _ = self.writer.flush();
            self.samples_since_flush = 0;
        }
    }
}

type RailsBakeTask = bevy::tasks::Task<Result<(OnRailsCache, f64), String>>;

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
    terrain_field: Option<std::sync::Arc<thessa_worldgen_rocky::field::PlanetField>>,
    launch_site_dir: Option<[f64; 3]>,
    vehicle: VehicleDefinition,
    aero_model: PanelAeroModel,
    atmosphere: AtmosphereConfig,
    state: RigidBodyState,
    sas_target_orientation: DQuat,
    render_relative_position_m: DVec3,
    flight_time_s: f64,
    world_tick: thessa_sim_core::WorldTick,
    throttle: f64,
    engine_active: bool,
    sas_enabled: bool,
    rcs_enabled: bool,
    gear_down: bool,
    /// Manual body-axis command: pitch, yaw, roll in normalized units.
    control_input: DVec3,
    surface_input: DVec3,
    actuator_saturated: bool,
    regime: FlightRegime,
    accumulator_s: f64,
    pub(super) steps_this_frame: u32,
    pub(super) rails_advanced_this_frame: f64,
    flight_error: Option<String>,
    render_orientation: Quat,
    last_gravity_acceleration_inertial_mps2: DVec3,
    last_forces: Option<FlightForces>,
    trace: Option<FlightTraceWriter>,
    /// The single baked coast trajectory. In an unpowered vacuum coast the
    /// flight loop samples translation from here (no per-tick integration)
    /// and the map prediction draws from the same path — one trajectory,
    /// two consumers. Any thrust, aero load, burn or contact invalidates it.
    pub(super) rails: OnRailsCache,
    rails_job: Option<RailsBakeTask>,
    rails_bake_seconds: Option<f64>,
    /// Simulation-time event queue: the rails bake arms its wake here, and
    /// `advance` drains due events instead of polling them every tick.
    pub(super) scheduler: EventScheduler,
    /// Last fired wake, for the HUD orbit line.
    pub(super) wake_notice: Option<String>,
}

impl PilotFlightRuntime {
    pub(super) fn initialize_world_site(
        &mut self,
        field: std::sync::Arc<thessa_worldgen_rocky::field::PlanetField>,
        dir: [f64; 3],
        ephemeris: &BakedEphemeris,
    ) {
        let up = DQuat::from_rotation_z(self.terrain_spin()) * DVec3::new(dir[0], -dir[2], dir[1]);
        let north = (DVec3::Z - up * up.z).normalize();
        let east = north.cross(up).normalize();
        let relative = up * (self.planet_radius_m + field.height_m(dir, 32.0).max(0.0) + 500.0);
        let body = ephemeris
            .body_state(self.reference_body, SimTime(self.flight_time_s))
            .expect("launch body");
        self.state.position_inertial_m = body.position_inertial + relative;
        self.state.velocity_inertial_mps = body.velocity_inertial
            + self.atmosphere.body_rotation_rad_s.cross(relative)
            + east * 180.0
            - up * 2.0;
        self.state.orientation_body_to_inertial =
            DQuat::from_mat3(&bevy::math::DMat3::from_cols(east, north, up))
                * DQuat::from_rotation_y(-8.0_f64.to_radians());
        self.sas_target_orientation = self.state.orientation_body_to_inertial;
        self.render_orientation = render_orientation(self.state.orientation_body_to_inertial);
        self.render_relative_position_m = relative;
        self.terrain_field = Some(field);
        self.launch_site_dir = Some(dir);
        self.rails.invalidate();
        self.rails_job = None;
    }
    /// Recover from a stopped flight without restarting the app: rebuild the
    /// launch-site state in place. Clock time is preserved (no rewind);
    /// controls clear and the engine comes back armed at zero throttle.
    pub(super) fn reset_to_launch_site(
        &mut self,
        ephemeris: &BakedEphemeris,
    ) -> Result<(), String> {
        let field = self
            .terrain_field
            .clone()
            .ok_or("no terrain field for reset")?;
        let dir = self.launch_site_dir.ok_or("no launch site for reset")?;
        self.flight_error = None;
        self.engine_active = true;
        self.throttle = 0.0;
        self.control_input = DVec3::ZERO;
        self.surface_input = DVec3::ZERO;
        self.regime = FlightRegime::Aero;
        self.accumulator_s = 0.0;
        self.rails.invalidate();
        self.rails_job = None;
        self.scheduler = EventScheduler::new();
        self.wake_notice = None;
        self.initialize_world_site(field, dir, ephemeris);
        Ok(())
    }
    pub(super) fn terrain_origin(&self) -> [f64; 3] {
        let p = self.render_relative_position_m;
        [p.x, p.z, -p.y]
    }
    /// Authoritative inertial craft state for orbit prediction and HUD.
    pub(super) fn inertial_state_m(&self) -> (DVec3, DVec3) {
        (
            self.state.position_inertial_m,
            self.state.velocity_inertial_mps,
        )
    }
    pub(super) fn terrain_spin(&self) -> f64 {
        self.flight_time_s * std::f64::consts::TAU / (80.0 * 3600.0)
    }
    pub(super) fn stop_reason(&self) -> Option<&str> {
        self.flight_error.as_deref()
    }

    pub(super) fn backlog_s(&self) -> f64 {
        self.accumulator_s
    }
    pub(super) fn regime(&self) -> FlightRegime {
        self.regime
    }
    pub(super) fn panel_count(&self) -> u32 {
        self.vehicle.aero_geometry.panels.len() as u32
    }

    pub(super) fn new(ephemeris: &BakedEphemeris, reference_body: BodyId) -> Result<Self, String> {
        let body = ephemeris
            .body(reference_body)
            .map_err(|error| format!("reference body is unavailable: {error}"))?;
        let body_state = ephemeris
            .body_state(reference_body, SimTime::EPOCH)
            .map_err(|error| format!("reference body state is unavailable: {error}"))?;
        let gravity = body.mu / body.radius_m.powi(2);
        let mut atmosphere = AtmosphereConfig::new(288.15, 120_000.0, 287.05287, 1.4, gravity)
            .map_err(|error| format!("Thessa atmosphere is invalid: {error}"))?;
        atmosphere.body_rotation_rad_s = DVec3::new(0.0, 0.0, TAU as f64 / (80.0 * 3_600.0));

        // Start just above the playable body's spherical datum. Give the X-15
        // a small nose-up launch attitude so its live engine start has a
        // physically meaningful positive angle of attack.
        let initial_relative_position = DVec3::Z * (body.radius_m + PILOT_START_ALTITUDE_M);
        let initial_position_inertial_m = body_state.position_inertial + initial_relative_position;
        let orientation_body_to_inertial = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2)
            * DQuat::from_rotation_y(-8.0_f64.to_radians());
        let state = RigidBodyState::new(
            initial_position_inertial_m,
            // The ephemeris velocity is the planet's barycentric translation.
            // Add only the local tangential launch velocity here. A small
            // launch component gives the powered test start enough clearance
            // without hiding the aerodynamic response behind a large impulse.
            body_state.velocity_inertial + DVec3::Y * 180.0 - DVec3::Z * 2.0,
            orientation_body_to_inertial,
            DVec3::ZERO,
        )
        .map_err(|error| format!("X-15 initial state is invalid: {error}"))?;
        let vehicle = x15_vehicle()?;
        let aero_model = PanelAeroModel::new(AeroConfig {
            lift_slope_per_rad: 4.6,
            control_effectiveness: 0.82,
            stall_angle_rad: X15_STALL_ANGLE_DEG.to_radians(),
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
            terrain_field: None,
            launch_site_dir: None,
            vehicle,
            aero_model,
            atmosphere,
            state,
            sas_target_orientation: orientation_body_to_inertial,
            render_relative_position_m: initial_relative_position,
            flight_time_s: 0.0,
            world_tick: thessa_sim_core::WorldTick::default(),
            throttle: 1.0,
            engine_active: true,
            sas_enabled: true,
            rcs_enabled: true,
            gear_down: true,
            control_input: DVec3::ZERO,
            surface_input: DVec3::ZERO,
            actuator_saturated: false,
            regime: FlightRegime::Aero,
            accumulator_s: 0.0,
            steps_this_frame: 0,
            rails_advanced_this_frame: 0.0,
            flight_error: None,
            render_orientation: render_orientation(orientation_body_to_inertial),
            last_gravity_acceleration_inertial_mps2: DVec3::ZERO,
            last_forces: None,
            trace: None,
            rails: OnRailsCache::new(),
            rails_job: None,
            rails_bake_seconds: None,
            scheduler: EventScheduler::new(),
            wake_notice: None,
        })
    }

    pub(super) fn with_trace(mut self, path: &Path) -> Self {
        self.trace = FlightTraceWriter::create(path);
        self
    }

    fn command_controls(&mut self, pitch: f64, yaw: f64, roll: f64) {
        // Elevator, rudder, left aileron, right aileron. The split ailerons
        // preserve roll authority without assigning a panel to two channels.
        let _ = self
            .vehicle
            // Body +X forward, +Z up implies physical right = -Y.
            // r x F: aft-tail downforce raises the nose (-Y); downforce
            // at -Y rolls right (+X); aft-tail +Y force yaws right (-Z).
            .apply_control_inputs(&[-pitch, yaw, -roll, roll]);
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
    thessa_sim_core::X15StarterProfile::new()
        .map(|profile| profile.vehicle)
        .map_err(|error| error.to_string())
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
            perspective.far = (runtime.render_relative_position_m.length()
                + runtime.planet_radius_m * 2.0) as f32;
        }
        **camera = pilot_camera_transform(
            &state,
            runtime.render_orientation,
            pilot_render_offset(runtime.render_relative_position_m).normalize(),
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
            transform.translation = pilot_render_offset(-runtime.render_relative_position_m);
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

fn simulate_pilot_flight(
    time: Res<Time>,
    mut clock: ResMut<SimulationClock>,
    ephemeris: Res<RuntimeEphemeris>,
    state: Res<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
    mut perf: ResMut<crate::perf::PerfMonitor>,
) {
    runtime.steps_this_frame = 0;
    runtime.rails_advanced_this_frame = 0.0;
    clock.sim_seconds = runtime.flight_time_s;
    clock.tick = runtime.world_tick;
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

fn conventional_angle_of_attack_deg(air_velocity_body: DVec3) -> f64 {
    // The reusable aero contract stores +Z as up and defines its coefficient
    // alpha from the velocity vector. Pilot HUDs conventionally report
    // positive AoA when the nose is above the velocity vector, hence -w/u.
    (-air_velocity_body.z)
        .atan2(air_velocity_body.x)
        .to_degrees()
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
