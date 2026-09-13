//! Authoritative flight runtime: input conditioning, regime selection,
//! the fixed-step advance loop and rails-bake orchestration.
//!
//! Moved verbatim from the client (which keeps only rendering, input
//! mapping and telemetry wiring). The only adaptations are mechanical:
//! Bevy types replaced by glam, the rails-bake worker behind [`BakeQueue`],
//! and render-view fields dropped (the client syncs its view after each
//! advance; stepping never reads them).

use std::{
    f32::consts::TAU,
    fs::{File, create_dir_all},
    io::{BufWriter, Write},
    path::Path,
    sync::Arc,
};

use glam::{DMat3, DQuat, DVec3};
use thessa_sim_core::{
    AeroConfig, AeroModel, AeroState, AtmosphereConfig, AtmosphereError, BakedEphemeris, BodyId,
    BodyState, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_VELOCITY_TOL_MPS, EventScheduler,
    FlightError, FlightForces, FlightStepInput, GravityField, OnRailsCache, PanelAeroModel,
    RigidBodyState, ScheduledKind, SimTime, TestParticleState, TickIntegratorConfig,
    VehicleDefinition, WORLD_TICK_S, X15StarterProfile, evaluate_flight_forces,
    integrate_attitude_step,
};
use thessa_worldgen_rocky::field::PlanetField;

use crate::{BakeQueue, BakedRails, ControlMode, FlightRegime, InlineBakeQueue, RailsBakeRequest};

/// Advance the exact production flight path at a fixed physics cadence.
/// Render frames only contribute elapsed time; they never set solver dt.
pub const FLIGHT_STEP_S: f64 = WORLD_TICK_S;
/// Baked coverage ahead (s) below which a fresh worker bake starts while
/// riding, so sustained warp never stalls at the horizon end.
const PROACTIVE_REBAKE_AHEAD_S: f64 = 86_400.0;
const SURFACE_COMMAND_RATE_S: f64 = 2.4; // 60 deg/s for the 25-degree elevator

/// Trim-conditioning threshold, not a force cutoff. At low density the
/// surface-response matrix becomes ill-conditioned; RCS handles attitude.
/// The flight solver still evaluates residual aero loads at every nonzero
/// density (q grows with speed squared even above this threshold).
pub const COAST_DENSITY_KG_M3: f64 = 1.0e-7;

pub const X15_STALL_ANGLE_DEG: f64 = 22.0;
const PILOT_SURFACE_CLEARANCE_M: f64 = 5.0;
const PILOT_START_ALTITUDE_M: f64 = 500.0;
// Solver guard rail, not physics: interlunar transfers range billions of
// metres from the reference body, so the cap covers the whole Nereid system
// plus escape margin. Tripping it still latches FLIGHT STOPPED.
const MAX_PILOT_ALTITUDE_M: f64 = 2.0e10;
const MAX_PILOT_RELATIVE_SPEED_MPS: f64 = 50_000.0;
const MAX_PILOT_ANGULAR_RATE_RPS: f64 = 25.0;
// KSP-style attitude keys change the SAS target at a pilotable rate.
const PILOT_ATTITUDE_COMMAND_RATE_RAD_S: f64 = 0.16;

#[derive(Debug, Clone, Copy)]
pub struct LocalAirKinematics {
    pub relative_position_inertial_m: DVec3,
    pub relative_position_body_m: DVec3,
    pub relative_velocity_inertial_mps: DVec3,
    pub air_velocity_body_mps: DVec3,
    pub surface_velocity_inertial_mps: DVec3,
    pub radial_up: DVec3,
    pub altitude_m: f64,
}

/// Sample all local flight kinematics from one rigid-body state and one
/// ephemeris body state. The rotating atmosphere vector is converted into the
/// vehicle frame by sim-core; callers must not cross a body-frame position with
/// an inertial-frame angular velocity directly.
pub fn local_air_kinematics(
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
pub struct FlightTraceWriter {
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

pub fn conventional_angle_of_attack_deg(air_velocity_body: DVec3) -> f64 {
    // The reusable aero contract stores +Z as up and defines its coefficient
    // alpha from the velocity vector. Pilot HUDs conventionally report
    // positive AoA when the nose is above the velocity vector, hence -w/u.
    (-air_velocity_body.z)
        .atan2(air_velocity_body.x)
        .to_degrees()
}

/// Authoritative flight model for the first playable vehicle.
///
/// The authoritative equations stay in `thessa-sim-core`; this runtime only
/// supplies game input, the X-15 asset and a telemetry bridge. Fuel is
/// intentionally infinite for this slice, while staging still controls the
/// engine. Render-view state lives client-side; stepping never reads it.
pub struct FlightAuthority {
    pub reference_body: BodyId,
    pub planet_radius_m: f64,
    pub terrain_field: Option<Arc<PlanetField>>,
    pub launch_site_dir: Option<[f64; 3]>,
    pub vehicle: VehicleDefinition,
    pub aero_model: PanelAeroModel,
    pub atmosphere: AtmosphereConfig,
    pub state: RigidBodyState,
    pub sas_target_orientation: DQuat,
    pub relative_position_m: DVec3,
    pub flight_time_s: f64,
    pub world_tick: thessa_sim_core::WorldTick,
    pub throttle: f64,
    pub engine_active: bool,
    pub sas_enabled: bool,
    pub rcs_enabled: bool,
    pub gear_down: bool,
    /// Manual body-axis command: pitch, yaw, roll in normalized units.
    pub control_input: DVec3,
    pub surface_input: DVec3,
    pub actuator_saturated: bool,
    pub regime: FlightRegime,
    pub accumulator_s: f64,
    pub steps_this_frame: u32,
    pub rails_advanced_this_frame: f64,
    pub flight_error: Option<String>,
    pub last_gravity_acceleration_inertial_mps2: DVec3,
    pub last_forces: Option<FlightForces>,
    pub trace: Option<FlightTraceWriter>,
    /// The single baked coast trajectory. In an unpowered vacuum coast the
    /// flight loop samples translation from here (no per-tick integration)
    /// and the map prediction draws from the same path — one trajectory,
    /// two consumers. Any thrust, aero load, burn or contact invalidates it.
    pub rails: OnRailsCache,
    pub bake: Box<dyn BakeQueue>,
    pub rails_bake_seconds: Option<f64>,
    /// Simulation-time event queue: the rails bake arms its wake here, and
    /// `advance` drains due events instead of polling them every tick.
    pub scheduler: EventScheduler,
    /// Last fired wake, for the HUD orbit line.
    pub wake_notice: Option<String>,
}

impl FlightAuthority {
    pub fn initialize_world_site(
        &mut self,
        field: Arc<PlanetField>,
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
            DQuat::from_mat3(&DMat3::from_cols(east, north, up))
                * DQuat::from_rotation_y(-8.0_f64.to_radians());
        self.sas_target_orientation = self.state.orientation_body_to_inertial;
        self.relative_position_m = relative;
        self.terrain_field = Some(field);
        self.launch_site_dir = Some(dir);
        self.rails.invalidate();
        self.bake.reset();
    }
    /// Recover from a stopped flight without restarting the app: rebuild the
    /// launch-site state in place. Clock time is preserved (no rewind);
    /// controls clear and the engine comes back armed at zero throttle.
    pub fn reset_to_launch_site(&mut self, ephemeris: &BakedEphemeris) -> Result<(), String> {
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
        self.bake.reset();
        self.scheduler = EventScheduler::new();
        self.wake_notice = None;
        self.initialize_world_site(field, dir, ephemeris);
        Ok(())
    }
    pub fn terrain_origin(&self) -> [f64; 3] {
        let p = self.relative_position_m;
        [p.x, p.z, -p.y]
    }
    /// Authoritative inertial craft state for orbit prediction and HUD.
    pub fn inertial_state_m(&self) -> (DVec3, DVec3) {
        (
            self.state.position_inertial_m,
            self.state.velocity_inertial_mps,
        )
    }
    pub fn terrain_spin(&self) -> f64 {
        self.flight_time_s * std::f64::consts::TAU / (80.0 * 3600.0)
    }
    pub fn stop_reason(&self) -> Option<&str> {
        self.flight_error.as_deref()
    }

    pub fn backlog_s(&self) -> f64 {
        self.accumulator_s
    }
    pub fn regime(&self) -> FlightRegime {
        self.regime
    }
    pub fn panel_count(&self) -> u32 {
        self.vehicle.aero_geometry.panels.len() as u32
    }

    pub fn new(ephemeris: &BakedEphemeris, reference_body: BodyId) -> Result<Self, String> {
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
            relative_position_m: initial_relative_position,
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
            last_gravity_acceleration_inertial_mps2: DVec3::ZERO,
            last_forces: None,
            trace: None,
            rails: OnRailsCache::new(),
            bake: Box::new(InlineBakeQueue::new()),
            rails_bake_seconds: None,
            scheduler: EventScheduler::new(),
            wake_notice: None,
        })
    }

    pub fn with_trace(mut self, path: &Path) -> Self {
        self.trace = FlightTraceWriter::create(path);
        self
    }

    /// Swap the rails-bake worker (Bevy pool in the client, threads on the
    /// server). The default inline queue bakes synchronously for tests.
    pub fn with_bake_queue(mut self, bake: Box<dyn BakeQueue>) -> Self {
        self.bake = bake;
        self
    }

    pub fn command_controls(&mut self, pitch: f64, yaw: f64, roll: f64) {
        // Elevator, rudder, left aileron, right aileron. The split ailerons
        // preserve roll authority without assigning a panel to two channels.
        let _ = self
            .vehicle
            // Body +X forward, +Z up implies physical right = -Y.
            // r x F: aft-tail downforce raises the nose (-Y); downforce
            // at -Y rolls right (+X); aft-tail +Y force yaws right (-Z).
            .apply_control_inputs(&[-pitch, yaw, -roll, roll]);
    }

    pub fn thrust_n(&self) -> f64 {
        if self.engine_active {
            self.throttle * 254_000.0
        } else {
            0.0
        }
    }
}

fn x15_vehicle() -> Result<VehicleDefinition, String> {
    X15StarterProfile::new()
        .map(|profile| profile.vehicle)
        .map_err(|error| error.to_string())
}

fn body_axes(command: DVec3) -> DVec3 {
    DVec3::new(command.z, -command.x, -command.y)
}

// Each opposed pair has zero net force. Locations and force directions are
// expressed in body metres/newtons; these are prototype jets, not X-15 data.
fn rcs_couples() -> [DVec3; 3] {
    [
        2.0 * DVec3::new(0.0, 1.4, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(-5.0, 0.0, 0.0).cross(DVec3::Z * 400.0),
        2.0 * DVec3::new(5.0, 0.0, 0.0).cross(DVec3::Y * 400.0),
    ]
}

fn rcs_moment(request: DVec3, enabled: bool) -> DVec3 {
    if !enabled {
        return DVec3::ZERO;
    }
    rcs_couples().into_iter().fold(DVec3::ZERO, |sum, couple| {
        sum + couple * (request.dot(couple) / couple.length_squared()).clamp(-1.0, 1.0)
    })
}

impl FlightAuthority {
    /// Advance the exact production flight path at a fixed physics cadence.
    /// Render frames only contribute elapsed time; they never set solver dt.
    pub fn advance(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
    ) -> Result<(), FlightError> {
        self.advance_with_budget(ephemeris, mode, elapsed_s, None)
    }

    /// The client may slow requested warp when CPU-bound. Never skip a physics
    /// step or change solver dt to catch up; only advance the clock by work done.
    /// Wall time determines batching only, never a force or physical state.
    fn sync_world_tick(&mut self) -> Result<(), FlightError> {
        if self.world_tick.time().0 != self.flight_time_s {
            self.world_tick = thessa_sim_core::WorldTick::from_time(SimTime(self.flight_time_s))
                .ok_or_else(|| FlightError::InvalidInput("invalid world tick epoch".into()))?;
            self.flight_time_s = self.world_tick.time().0;
        }
        Ok(())
    }

    fn time_after_ticks(&self, ticks: u64) -> Result<SimTime, FlightError> {
        self.world_tick
            .checked_add(ticks)
            .map(|tick| tick.time())
            .ok_or_else(|| FlightError::InvalidInput("world tick overflow".into()))
    }

    fn commit_ticks(&mut self, ticks: u64) -> Result<(), FlightError> {
        self.world_tick = self
            .world_tick
            .checked_add(ticks)
            .ok_or_else(|| FlightError::InvalidInput("world tick overflow".into()))?;
        self.flight_time_s = self.world_tick.time().0;
        Ok(())
    }

    pub fn advance_with_budget(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), FlightError> {
        self.sync_world_tick()?;
        let started = std::time::Instant::now();
        self.steps_this_frame = 0;
        self.rails_advanced_this_frame = 0.0;
        self.accumulator_s += elapsed_s;
        let gravity_field = GravityField::from_ephemeris(ephemeris);
        while self.accumulator_s + 1.0e-12 >= FLIGHT_STEP_S {
            let coast = self.try_advance_cached_coast(ephemeris, mode, self.accumulator_s)?;
            if coast > 0.0 {
                self.accumulator_s = (self.accumulator_s - coast).max(0.0);
                self.rails_advanced_this_frame += coast;
                // Wake at the event boundary, allowing the owner to react
                // before any further physical work in this frame.
                if self
                    .scheduler
                    .next()
                    .is_some_and(|event| event.time.0 <= self.flight_time_s + FLIGHT_STEP_S)
                {
                    break;
                }
                continue;
            }
            self.step(ephemeris, &gravity_field, mode)?;
            self.steps_this_frame += 1;
            self.accumulator_s = (self.accumulator_s - FLIGHT_STEP_S).max(0.0);
            if budget.is_some_and(|limit| started.elapsed() >= limit) {
                // Excess requested warp is unserved wall-time demand, not
                // elapsed simulation time. Keep only the fractional tick;
                // don't build a catch-up queue that delays later inputs.
                let whole_ticks = ((self.accumulator_s + 1.0e-12) / FLIGHT_STEP_S).floor();
                self.accumulator_s = (self.accumulator_s - whole_ticks * FLIGHT_STEP_S).max(0.0);
                break;
            }
        }
        // Event-driven wakes: due scheduler events fire here instead of being
        // polled every physics tick.
        for event in self.scheduler.drain_due(SimTime(self.flight_time_s)) {
            self.wake_notice = Some(match event.kind {
                ScheduledKind::RailsImpact { .. } => {
                    format!("WAKE IMPACT T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::RailsHorizon => {
                    format!("WAKE HORIZON T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::ManeuverNode { .. } => {
                    format!("WAKE NODE T+{:.0}s", event.time.seconds())
                }
                ScheduledKind::Alarm => {
                    format!("WAKE ALARM T+{:.0}s", event.time.seconds())
                }
            });
        }
        Ok(())
    }

    fn validate_endpoint(
        &mut self,
        ephemeris: &BakedEphemeris,
        next: &mut RigidBodyState,
        next_body: BodyState,
        time: SimTime,
    ) -> Result<(), FlightError> {
        // Solver guard rails read in the display (dominant-pull) frame, not
        // the launch frame: a Nereid escape at 33 km/s is routine flight,
        // while the same speed against the launch body would be nonsense.
        // Statically bounding against the launch world stopped every real
        // interlunar coast at the first handoff.
        let guard_body = ephemeris
            .dominant_body(next.position_inertial_m, time)
            .unwrap_or(self.reference_body);
        let guard_state = ephemeris.body_state(guard_body, time).unwrap_or(next_body);
        let guard_radius = ephemeris
            .body(guard_body)
            .map(|body| body.radius_m)
            .unwrap_or(self.planet_radius_m);
        let relative = next.position_inertial_m - next_body.position_inertial;
        let guard_relative = next.position_inertial_m - guard_state.position_inertial;
        if (guard_relative.length() - guard_radius).abs() > MAX_PILOT_ALTITUDE_M
            || (next.velocity_inertial_mps - guard_state.velocity_inertial).length()
                > MAX_PILOT_RELATIVE_SPEED_MPS
            || next.angular_velocity_body_rps.length() > MAX_PILOT_ANGULAR_RATE_RPS
        {
            return Err(FlightError::InvalidInput(
                "flight state exceeded solver bounds".into(),
            ));
        }
        if let Some(field) = &self.terrain_field {
            let body_dir =
                DQuat::from_rotation_y(-(time.0 * std::f64::consts::TAU / (80.0 * 3600.0)))
                    * DVec3::new(relative.x, relative.z, -relative.y).normalize();
            let surface =
                field.params.radius_m + field.height_m(body_dir.to_array(), 32.0).max(0.0);
            if relative.length() < surface + PILOT_SURFACE_CLEARANCE_M {
                return Err(FlightError::InvalidInput(
                    "terrain impact; contact dynamics are not implemented".into(),
                ));
            }
        }
        if guard_radius > 0.0 && guard_relative.length() <= guard_radius {
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Err(FlightError::InvalidInput(format!(
                "surface contact with {guard_body:?}"
            )));
        }
        // Existing spherical contact boundary; no invented angular damping.
        if relative.length() < self.planet_radius_m + PILOT_SURFACE_CLEARANCE_M {
            let up = relative.normalize();
            next.position_inertial_m = next_body.position_inertial
                + up * (self.planet_radius_m + PILOT_SURFACE_CLEARANCE_M);
            let radial_speed = (next.velocity_inertial_mps - next_body.velocity_inertial).dot(up);
            if radial_speed < 0.0 {
                next.velocity_inertial_mps -= up * radial_speed;
            }
            // Contact clamp rewrote the state off the baked path.
            self.rails.invalidate();
        }
        Ok(())
    }

    /// One read of a valid coast instead of replaying every translation tick.
    /// The curve hull and each body's maximum orbital speed certify that the
    /// entire interval stays outside geometry and in exactly sampled vacuum.
    fn try_advance_cached_coast(
        &mut self,
        ephemeris: &BakedEphemeris,
        mode: ControlMode,
        requested_s: f64,
    ) -> Result<f64, FlightError> {
        if self.thrust_n() != 0.0 || self.rails.is_empty() || self.trace.is_some() {
            return Ok(0.0);
        }
        // Collect a worker bake that finished while earlier batches flew;
        // the batch loop otherwise never polls, and coverage would stall.
        self.poll_rails_bake(ephemeris);
        // A zero controller moment at the first instant is not a promise
        // that SAS stays idle while a spinning craft turns away from target.
        // Do not run the stateful allocator speculatively either: a failed
        // batch would otherwise advance its actuator slew twice in one tick.
        let attitude_hold =
            self.sas_enabled && matches!(mode, ControlMode::Navball | ControlMode::MouseAim);
        if self.rcs_enabled
            && (self.control_input != DVec3::ZERO
                || (mode != ControlMode::Direct
                    && self.state.angular_velocity_body_rps != DVec3::ZERO)
                || (attitude_hold
                    && self.sas_target_orientation != self.state.orientation_body_to_inertial
                    && self.sas_target_orientation != -self.state.orientation_body_to_inertial))
        {
            return Ok(0.0);
        }
        let time = SimTime(self.flight_time_s);
        let mut duration = ((requested_s + 1.0e-12) / FLIGHT_STEP_S).floor() * FLIGHT_STEP_S;
        if let Some(event) = self.scheduler.next() {
            duration = duration
                .min(((event.time.0 - time.0) / FLIGHT_STEP_S).floor().max(0.0) * FLIGHT_STEP_S);
        }
        if let Some(path) = self.rails.path() {
            duration = duration.min(
                ((path.end_time.0 - time.0) / FLIGHT_STEP_S)
                    .floor()
                    .max(0.0)
                    * FLIGHT_STEP_S,
            );
        }
        if duration < 2.0 * FLIGHT_STEP_S {
            return Ok(0.0);
        }
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        if !self.rails.usable_tick_for(
            ephemeris,
            TestParticleState {
                position: self.state.position_inertial_m,
                velocity: self.state.velocity_inertial_mps,
            },
            time,
            TickIntegratorConfig::default(),
            &impact_bodies,
            COAST_RAILS_POSITION_TOL_M,
            COAST_RAILS_VELOCITY_TOL_MPS,
        ) {
            return Ok(0.0);
        }
        // Proactive JIT preparation for sustained warp: when baked coverage
        // ahead drops under a day, start a fresh full-horizon worker bake
        // now so high warp never stalls on a synchronous rebake at the
        // horizon end. One job at most; the swap path validates on arrival.
        if !self.bake.has_pending()
            && let Some(covered) = self.rails.covered_until()
            && covered.0 - time.0 < PROACTIVE_REBAKE_AHEAD_S
        {
            self.spawn_rails_bake(ephemeris);
        }
        let ticks = (duration / FLIGHT_STEP_S).round() as u64;
        let end = self.time_after_ticks(ticks)?;
        let Some((min, max)) = self.rails.position_bounds(time, end) else {
            return Ok(0.0);
        };
        for body in ephemeris
            .bodies
            .iter()
            .filter(|body| body.radius_m > 0.0 || body.id == self.reference_body)
        {
            let center = ephemeris
                .body_state(body.id, time)
                .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let speed = ephemeris
                .maximum_body_speed(body.id)
                .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
            let clearance = center
                .position_inertial
                .distance(center.position_inertial.clamp(min, max))
                - speed * duration
                - body.radius_m;
            let terrain_bound = if body.id == self.reference_body {
                self.terrain_field
                    .as_ref()
                    .map_or(0.0, |field| field.params.height_max_m.max(0.0))
            } else {
                0.0
            };
            if clearance <= terrain_bound + PILOT_SURFACE_CLEARANCE_M {
                return Ok(0.0);
            }
            if body.id == self.reference_body
                && self.atmosphere.sample(clearance)?.density_kg_m3 != 0.0
            {
                return Ok(0.0);
            }
        }
        let Some(orientation) = thessa_sim_core::constant_spin_orientation(
            self.state.orientation_body_to_inertial,
            self.state.angular_velocity_body_rps,
            self.vehicle.mass_properties.inertia_body_kg_m2,
            duration,
        ) else {
            return Ok(0.0);
        };
        self.regime = FlightRegime::Coast;
        let (position, velocity) = self.rails.sample_at(end).expect("bounded coast interval");
        let mut next = RigidBodyState::new(
            position,
            velocity,
            orientation,
            self.state.angular_velocity_body_rps,
        )
        .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let next_home = ephemeris
            .body_state(self.reference_body, end)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.validate_endpoint(ephemeris, &mut next, next_home, end)?;
        // One telemetry sample per batch, never per skipped translation tick.
        let gravity = GravityField::from_ephemeris(ephemeris)
            .acceleration(position, end)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(evaluate_flight_forces(
            &self.aero_model,
            &self.vehicle.aero_geometry,
            self.atmosphere,
            next,
            self.vehicle.mass_properties,
            FlightStepInput {
                altitude_m: (position - next_home.position_inertial).length()
                    - self.planet_radius_m,
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: orientation.inverse() * (position - next_home.position_inertial),
                wind_velocity_body_mps: orientation.inverse() * next_home.velocity_inertial,
                extra_force_body_n: DVec3::ZERO,
                extra_moment_body_nm: DVec3::ZERO,
                skip_aero: true,
            },
        )?);
        self.state = next;
        self.commit_ticks(ticks)?;
        self.relative_position_m = position - next_home.position_inertial;
        Ok(duration)
    }

    fn allocate_controls(
        &mut self,
        kinematics: LocalAirKinematics,
        mode: ControlMode,
    ) -> Result<DVec3, FlightError> {
        let axes = body_axes(self.control_input);
        let assisted = mode != ControlMode::Direct;
        let attitude_hold =
            self.sas_enabled && matches!(mode, ControlMode::Navball | ControlMode::MouseAim);
        let desired_rate = if attitude_hold && axes.length_squared() > 1.0e-8 {
            // Manual input overrides attitude hold. Capture the achieved
            // attitude, so a long turn cannot wind an unreachable target past
            // 180 degrees and make shortest-path SAS reverse the manoeuvre.
            self.sas_target_orientation = self.state.orientation_body_to_inertial;
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S
        } else if attitude_hold {
            let mut error =
                self.state.orientation_body_to_inertial.inverse() * self.sas_target_orientation;
            // q and -q represent the same attitude; use the short rotation.
            if error.w < 0.0 {
                error = -error;
            }
            let angle = error.to_scaled_axis();
            let couples = rcs_couples();
            let inertia = self.vehicle.mass_properties.inertia_body_kg_m2;
            let acceleration = DVec3::new(
                couples[0].x / inertia.x_axis.x,
                couples[1].y / inertia.y_axis.y,
                couples[2].z / inertia.z_axis.z,
            );
            // Brake early enough for the finite jets. A fixed high-gain
            // attitude loop saturates in vacuum and keeps overshooting.
            let braking_rate = (acceleration * angle.abs() * 0.5).sqrt();
            (angle * 1.6)
                .clamp(-braking_rate, braking_rate)
                .clamp_length_max(0.35)
        } else {
            // Capturing here avoids a jump to an old SAS target when re-enabled.
            self.sas_target_orientation = self.state.orientation_body_to_inertial;
            axes * PILOT_ATTITUDE_COMMAND_RATE_RAD_S
        };
        let inertia = self.vehicle.mass_properties.inertia_body_kg_m2;
        let omega = self.state.angular_velocity_body_rps;
        let requested = inertia * ((desired_rate - omega) / 0.35) + omega.cross(inertia * omega);
        // Vacuum fast path: no air load exists, so the trim solve and the
        // response evaluation are skipped outright (exactly zero aero
        // moment); attitude flies on RCS alone.
        if self.regime == FlightRegime::Coast {
            let jets = if assisted {
                rcs_moment(requested, self.rcs_enabled)
            } else {
                let couples = rcs_couples();
                rcs_moment(
                    DVec3::new(couples[0].x, couples[1].y, couples[2].z) * axes,
                    self.rcs_enabled,
                )
            };
            self.actuator_saturated = assisted && (requested - jets).length() > 1_000.0;
            return Ok(jets);
        }
        let environment = self
            .atmosphere
            .aero_environment(kinematics.altitude_m.max(0.0), DVec3::ZERO)?;
        let aero_state = AeroState::new(kinematics.air_velocity_body_mps, omega);
        let mut command = if assisted {
            self.surface_input
        } else {
            self.control_input
        };
        // In coast the surfaces stay where the trim left them: with no air
        // load there is nothing to trim against, and the Newton solve below
        // would invert aerodynamic noise. Direct mode is already Newton-free
        // raw passthrough, so only the assisted solve is gated.
        let solve_trim = assisted && self.regime == FlightRegime::Aero;
        if solve_trim {
            // Linearize actual panel response at this flow/deflection. Two
            // bounded Newton passes handle cross-axis coupling near stall.
            for _ in 0..2 {
                self.command_controls(command.x, command.y, command.z);
                let baseline = self
                    .aero_model
                    .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
                    .moment_body_nm;
                let mut columns = [DVec3::ZERO; 3];
                for axis in 0..3 {
                    let mut probe = command;
                    let delta = if command[axis] > 0.9 { -0.02 } else { 0.02 };
                    probe[axis] += delta;
                    self.command_controls(probe.x, probe.y, probe.z);
                    columns[axis] = (self
                        .aero_model
                        .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
                        .moment_body_nm
                        - baseline)
                        / delta;
                }
                let effectiveness = DMat3::from_cols(columns[0], columns[1], columns[2]);
                if effectiveness.determinant().abs() > 1.0e-6 {
                    command = (command + effectiveness.inverse() * (requested - baseline))
                        .clamp(DVec3::splat(-1.0), DVec3::ONE);
                }
            }
        }
        let max_change = SURFACE_COMMAND_RATE_S * FLIGHT_STEP_S;
        self.surface_input += (command - self.surface_input)
            .clamp(DVec3::splat(-max_change), DVec3::splat(max_change));
        self.command_controls(
            self.surface_input.x,
            self.surface_input.y,
            self.surface_input.z,
        );
        let actual_aero = self
            .aero_model
            .evaluate_state(aero_state, environment, &self.vehicle.aero_geometry)?
            .moment_body_nm;
        let jets = if assisted {
            rcs_moment(requested - actual_aero, self.rcs_enabled)
        } else {
            let couples = rcs_couples();
            rcs_moment(
                DVec3::new(couples[0].x, couples[1].y, couples[2].z) * axes,
                self.rcs_enabled,
            )
        };
        self.actuator_saturated = assisted && (requested - actual_aero - jets).length() > 1_000.0;
        Ok(jets)
    }

    /// Full rigid-body step for powered/aero flight (translation integrated).
    fn integrate_powered_step(
        &mut self,
        gravity: DVec3,
        kinematics: LocalAirKinematics,
        body_state: BodyState,
        jet_moment: DVec3,
        thrust_n: f64,
        skip_aero: bool,
    ) -> Result<(RigidBodyState, FlightForces), FlightError> {
        thessa_sim_core::integrate_rigid_body_step(
            &self.aero_model,
            &self.vehicle.aero_geometry,
            self.atmosphere,
            self.state,
            self.vehicle.mass_properties,
            FlightStepInput {
                altitude_m: kinematics.altitude_m.max(0.0),
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: kinematics.relative_position_body_m,
                wind_velocity_body_mps: self.state.orientation_body_to_inertial.inverse()
                    * body_state.velocity_inertial,
                extra_force_body_n: DVec3::X * thrust_n,
                extra_moment_body_nm: jet_moment,
                skip_aero,
            },
            FLIGHT_STEP_S,
        )
    }

    /// Arm the baked path's wake condition in the simulation-time scheduler:
    /// one path owns exactly one wake (impact epoch or horizon end), so the
    /// flight loop and autopilot wait on events instead of polling.
    fn arm_rails_wake(&mut self) {
        let Some(wake) = self.rails.wake() else {
            return;
        };
        match wake {
            thessa_sim_core::OnRailsWake::Impact { time, body } => {
                self.scheduler
                    .arm_rails_wake(ScheduledKind::RailsImpact { body }, time);
            }
            thessa_sim_core::OnRailsWake::HorizonEnd { time } => {
                self.scheduler
                    .arm_rails_wake(ScheduledKind::RailsHorizon, time);
            }
        }
    }

    /// Spawn a full-horizon worker bake from the live state unless one is
    /// already running. No-op headless (no pool). The swap path validates
    /// ephemeris/config on arrival, so a bake that goes stale mid-flight
    /// is rejected harmlessly instead of corrupting the cache.
    /// Request a full-horizon worker bake from the live state unless one is
    /// already running. The [`BakeQueue`] decides how it runs (Bevy pool in
    /// the client, thread on the server, inline in tests). The swap path
    /// validates ephemeris/config on arrival, so a bake that goes stale
    /// mid-flight is rejected harmlessly instead of corrupting the cache.
    fn spawn_rails_bake(&mut self, ephemeris: &BakedEphemeris) {
        if self.bake.has_pending() {
            return;
        }
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let time = SimTime(self.flight_time_s);
        let config = TickIntegratorConfig::default();
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        self.bake.request_bake(RailsBakeRequest {
            initial,
            time,
            config,
            impact_bodies,
            ephemeris: ephemeris.clone(),
        });
    }

    /// Poll a finished worker bake, adopting it only after the state/key
    /// check (which also rejects bakes from a pre-edit universe).
    /// Poll a finished worker bake, adopting it only after the state/key
    /// check (which also rejects bakes from a pre-edit universe).
    fn poll_rails_bake(&mut self, ephemeris: &BakedEphemeris) {
        let Some(result) = self.bake.poll_bake() else {
            return;
        };
        if let Ok(baked) = result {
            let BakedRails {
                rails,
                bake_seconds: seconds,
            } = baked;
            // A worker started before an ephemeris edit must not replace
            // the common cache with a trajectory from the old universe.
            if let Some(path) = rails.path()
                && rails.usable_tick_for(
                    ephemeris,
                    TestParticleState {
                        position: path.positions[0],
                        velocity: path.velocities[0],
                    },
                    path.times[0],
                    TickIntegratorConfig::default(),
                    &ephemeris
                        .bodies
                        .iter()
                        .filter(|b| b.radius_m > 0.0)
                        .map(|b| b.id)
                        .collect::<Vec<_>>(),
                    0.0,
                    0.0,
                )
            {
                self.rails = rails;
                self.rails_bake_seconds = Some(seconds);
            }
        }
    }

    /// Poll/build the common gravity forecast. Reading the map never runs a
    /// second integrator. Flight adopts it only after the state/key check.
    pub fn prepare_shared_trajectory(&mut self, ephemeris: &BakedEphemeris) -> bool {
        let initial = TestParticleState {
            position: self.state.position_inertial_m,
            velocity: self.state.velocity_inertial_mps,
        };
        let time = SimTime(self.flight_time_s);
        let config = TickIntegratorConfig::default();
        let impact_bodies: Vec<_> = ephemeris
            .bodies
            .iter()
            .filter(|b| b.radius_m > 0.0)
            .map(|b| b.id)
            .collect();
        self.poll_rails_bake(ephemeris);
        if self.rails.usable_tick_for(
            ephemeris,
            initial,
            time,
            config,
            &impact_bodies,
            COAST_RAILS_POSITION_TOL_M,
            COAST_RAILS_VELOCITY_TOL_MPS,
        ) {
            return true;
        }
        // The queue decides how the bake runs (pool, thread, inline).
        self.spawn_rails_bake(ephemeris);
        false
    }

    /// Unpowered vacuum coast on the shared baked trajectory. Translation is
    /// sampled (cubic Hermite) from the rails path the map prediction draws;
    /// attitude keeps integrating under the RCS moment. Returns `None` when
    /// no rails path can serve this step (bake failure, horizon exhausted
    /// twice in a row) so the caller falls back to a normal integrated step.
    /// Flight and map share the same tick-adaptive bake and interpolant.
    fn try_coast_step_on_rails(
        &mut self,
        ephemeris: &BakedEphemeris,
        time: SimTime,
        jet_moment: DVec3,
        body_state: BodyState,
        gravity: DVec3,
    ) -> Result<Option<(RigidBodyState, FlightForces)>, FlightError> {
        if !self.prepare_shared_trajectory(ephemeris) {
            return Ok(None);
        }
        self.arm_rails_wake();
        self.rails.trim_before(time, 3_600.0, 1_800.0);
        let next_time = self.time_after_ticks(1)?;
        if let Some(thessa_sim_core::OnRailsWake::Impact {
            time: impact_time,
            body,
        }) = self.rails.wake()
            && impact_time.0 <= next_time.0
        {
            self.wake_notice = Some(format!("WAKE IMPACT {body:?} T+{:.3}s", impact_time.0));
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Err(FlightError::InvalidInput(format!(
                "coast reached surface contact with {body:?}"
            )));
        }
        let sampled = self.rails.sample_at(next_time);
        if sampled.is_none() {
            // Never rebake across an impact or block on horizon extension.
            // Ordinary physics handles the next step; a future request can
            // build a new horizon on the worker.
            if let Some(wake) = self.rails.wake() {
                self.wake_notice = Some(format!("WAKE {wake:?}"));
            }
            self.rails.invalidate();
            self.scheduler.clear_rails_wakes();
            return Ok(None);
        }
        let Some((position, velocity)) = sampled else {
            // Baking while already inside a body yields a single-sample path
            // with no forward coverage: let the normal step (and its contact
            // handling) deal with it.
            return Ok(None);
        };
        let (orientation, omega) = integrate_attitude_step(
            self.state.orientation_body_to_inertial,
            self.state.angular_velocity_body_rps,
            self.vehicle.mass_properties.inertia_body_kg_m2,
            jet_moment,
            FLIGHT_STEP_S,
        )?;
        let next =
            RigidBodyState::new(position, velocity, orientation, omega).map_err(|error| {
                self.rails.invalidate();
                FlightError::InvalidInput(error.to_string())
            })?;
        // Zero-load bookkeeping for the trace: aero is skipped, thrust is
        // zero, so this only evaluates the (empty) panel response.
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        let forces = match evaluate_flight_forces(
            &self.aero_model,
            &self.vehicle.aero_geometry,
            self.atmosphere,
            self.state,
            self.vehicle.mass_properties,
            FlightStepInput {
                altitude_m: kinematics.altitude_m.max(0.0),
                gravity_acceleration_inertial_mps2: gravity,
                position_body_m: kinematics.relative_position_body_m,
                wind_velocity_body_mps: self.state.orientation_body_to_inertial.inverse()
                    * body_state.velocity_inertial,
                extra_force_body_n: DVec3::ZERO,
                extra_moment_body_nm: jet_moment,
                skip_aero: true,
            },
        ) {
            Ok(forces) => forces,
            Err(error) => {
                self.rails.invalidate();
                return Err(error);
            }
        };
        Ok(Some((next, forces)))
    }

    fn step(
        &mut self,
        ephemeris: &BakedEphemeris,
        gravity_field: &GravityField,
        mode: ControlMode,
    ) -> Result<(), FlightError> {
        self.sync_world_tick()?;
        let time = self.world_tick.time();
        let body_state = ephemeris
            .body_state(self.reference_body, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        // Regime follows sampled density, never altitude: an invalid sample
        // keeps Aero so the existing atmosphere error paths still fire.
        let density_kg_m3 = self
            .atmosphere
            .sample(kinematics.altitude_m.max(0.0))
            .map(|sample| sample.density_kg_m3)
            .unwrap_or(f64::INFINITY);
        self.regime = if density_kg_m3 < COAST_DENSITY_KG_M3 {
            FlightRegime::Coast
        } else {
            FlightRegime::Aero
        };
        let gravity = gravity_field
            .acceleration(self.state.position_inertial_m, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let jet_moment = self.allocate_controls(kinematics, mode)?;
        // Only an exactly empty sampled medium permits zero force.
        // Low density alone does not bound drag at high speed;
        // Coast freezes trim but retains residual panel forces.
        let skip_aero = density_kg_m3 == 0.0;
        let thrust_n = self.thrust_n();
        // Unpowered vacuum coast rides the single baked trajectory instead
        // of integrating translation per tick; the map prediction draws the
        // same path. Attitude (RCS) still integrates at full rate below.
        let (mut next, forces) = if self.regime == FlightRegime::Coast
            && thrust_n == 0.0
            && skip_aero
        {
            match self.try_coast_step_on_rails(ephemeris, time, jet_moment, body_state, gravity)? {
                Some(coasted) => coasted,
                None => self.integrate_powered_step(
                    gravity, kinematics, body_state, jet_moment, thrust_n, skip_aero,
                )?,
            }
        } else {
            self.scheduler.clear_rails_wakes();
            self.integrate_powered_step(
                gravity, kinematics, body_state, jet_moment, thrust_n, skip_aero,
            )?
        };
        let next_body = ephemeris
            .body_state(self.reference_body, self.time_after_ticks(1)?)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        self.validate_endpoint(ephemeris, &mut next, next_body, self.time_after_ticks(1)?)?;
        if let Some(trace) = self.trace.as_mut() {
            trace.record(
                self.flight_time_s,
                kinematics.altitude_m,
                kinematics.relative_velocity_inertial_mps,
                kinematics.radial_up,
                kinematics.air_velocity_body_mps,
                self.state,
                self.control_input,
                self.throttle,
                self.engine_active,
                self.sas_enabled,
                self.rcs_enabled,
                &forces,
                mode,
                self.surface_input,
                self.actuator_saturated,
                self.sas_target_orientation,
            );
        }
        self.state = next;
        self.commit_ticks(1)?;
        self.last_gravity_acceleration_inertial_mps2 = gravity;
        self.last_forces = Some(forces);
        self.relative_position_m = next.position_inertial_m - next_body.position_inertial;
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::SystemConfig;

    fn fixture() -> (BakedEphemeris, FlightAuthority) {
        let config: SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = config.bake().unwrap();
        let runtime =
            FlightAuthority::new(&ephemeris, ephemeris.body_id("thessa").unwrap()).unwrap();
        (ephemeris, runtime)
    }

    #[test]
    fn solver_guard_reads_dominant_frame_not_launch_frame() {
        // Live failure: a Nereid escape at 33 km/s Nereid-relative latched
        // FLIGHT STOPPED because the guard measured 50+ km/s against the
        // LAUNCH body. Same state must pass with the guard in the dominant
        // frame. v_mag sits strictly between the bound and bound+R so the
        // old launch-frame check would trip while the dominant check clears.
        let (ephemeris, mut flight) = fixture();
        let thessa = ephemeris.body_id("thessa").unwrap();
        let nereid = ephemeris.body_id("nereid").unwrap();
        let time = SimTime::EPOCH;
        let thessa_state = ephemeris.body_state(thessa, time).unwrap();
        let nereid_state = ephemeris.body_state(nereid, time).unwrap();
        let giant = ephemeris.body(nereid).unwrap();
        let frame_gap = (thessa_state.velocity_inertial - nereid_state.velocity_inertial).length();
        assert!(frame_gap > 1000.0, "frames must differ, gap {frame_gap}");
        let v_mag = MAX_PILOT_RELATIVE_SPEED_MPS + frame_gap / 2.0;
        let away =
            (thessa_state.velocity_inertial - nereid_state.velocity_inertial).normalize_or_zero();
        flight.state.position_inertial_m =
            nereid_state.position_inertial + DVec3::Z * (giant.radius_m + 100_000.0);
        flight.state.velocity_inertial_mps = thessa_state.velocity_inertial - away * v_mag;
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        flight.flight_time_s = 0.0;
        assert_eq!(
            ephemeris.dominant_body(flight.state.position_inertial_m, time),
            Some(nereid)
        );
        let launch_relative =
            (flight.state.velocity_inertial_mps - thessa_state.velocity_inertial).length();
        assert!(
            launch_relative > MAX_PILOT_RELATIVE_SPEED_MPS,
            "setup must trip the old guard, got {launch_relative}"
        );
        flight
            .step(
                &ephemeris,
                &GravityField::from_ephemeris(&ephemeris),
                ControlMode::Direct,
            )
            .expect("dominant-frame guard must clear interlunar coast");
    }

    #[test]
    fn warp_budget_keeps_exact_steps_and_does_not_queue_unserved_warp() {
        let (ephemeris, mut limited) = fixture();
        let (_, mut exact) = fixture();
        limited
            .advance_with_budget(
                &ephemeris,
                ControlMode::Navball,
                3.2,
                Some(std::time::Duration::ZERO),
            )
            .unwrap();
        assert_eq!(limited.steps_this_frame, 1);
        assert!(limited.accumulator_s < FLIGHT_STEP_S);
        exact
            .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
            .unwrap();
        assert_eq!(limited.state, exact.state);
        assert_eq!(limited.flight_time_s, exact.flight_time_s);
        limited
            .advance(&ephemeris, ControlMode::Navball, 0.0)
            .unwrap();
        assert_eq!(limited.steps_this_frame, 0);
    }

    #[test]
    #[ignore = "wall-clock diagnostic; run with --ignored --nocapture"]
    fn profile_warp_frame_budget() {
        let (ephemeris, mut full) = fixture();
        let (_, mut bounded) = fixture();
        let now = std::time::Instant::now();
        full.advance(&ephemeris, ControlMode::Navball, 3.2).unwrap();
        eprintln!(
            "warp unbounded: {:?}, {} steps",
            now.elapsed(),
            full.steps_this_frame
        );
        let now = std::time::Instant::now();
        bounded
            .advance_with_budget(
                &ephemeris,
                ControlMode::Navball,
                3.2,
                Some(std::time::Duration::from_millis(8)),
            )
            .unwrap();
        eprintln!(
            "warp bounded: {:?}, {} steps",
            now.elapsed(),
            bounded.steps_this_frame
        );
        assert!(bounded.steps_this_frame < full.steps_this_frame);
    }

    #[test]
    fn terrain_contact_stops_before_committing_an_underground_pose() {
        let (ephemeris, mut flight) = fixture();
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        let field = std::sync::Arc::new(
            thessa_worldgen_rocky::field::field_from_manifest(
                &thessa_worldgen_rocky::spec_recipe::manifest_from_spec(&recipe).unwrap(),
            )
            .unwrap(),
        );
        let dir = [1.0, 0.0, 0.0];
        flight.initialize_world_site(field.clone(), dir, &ephemeris);
        let body = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        let r = flight.planet_radius_m + field.height_m(dir, 32.0).max(0.0);
        flight.state.position_inertial_m = body.position_inertial + DVec3::X * (r + 0.1);
        flight.state.velocity_inertial_mps = body.velocity_inertial - DVec3::X * 100.0;
        let before = flight.state;
        let error = flight
            .step(
                &ephemeris,
                &GravityField::from_ephemeris(&ephemeris),
                ControlMode::Direct,
            )
            .unwrap_err();
        assert!(error.to_string().contains("terrain impact"), "{error}");
        assert_eq!(flight.state.position_inertial_m, before.position_inertial_m);
        assert_eq!(flight.flight_time_s, 0.0);
    }

    #[test]
    fn surface_moments_follow_pilot_axes_without_rcs() {
        let (_, mut flight) = fixture();
        let env = flight
            .atmosphere
            .aero_environment(500.0, DVec3::ZERO)
            .unwrap();
        let flow = AeroState::new(DVec3::X * 180.0, DVec3::ZERO);
        let base = flight
            .aero_model
            .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
            .unwrap()
            .moment_body_nm;
        for command in [
            DVec3::X,
            DVec3::Y,
            DVec3::Z,
            -DVec3::X,
            -DVec3::Y,
            -DVec3::Z,
        ] {
            flight.command_controls(command.x, command.y, command.z);
            let moment = flight
                .aero_model
                .evaluate_state(flow, env, &flight.vehicle.aero_geometry)
                .unwrap()
                .moment_body_nm;
            assert!(
                (moment - base).dot(body_axes(command)) > 0.0,
                "wrong surface sign for {command:?}"
            );
        }
        assert_eq!(rcs_moment(DVec3::splat(1.0e9), false), DVec3::ZERO);
        assert_eq!(
            rcs_moment(DVec3::splat(1.0e9), true),
            DVec3::new(1120.0, 4000.0, 4000.0)
        );
    }

    #[test]
    fn recorded_high_rate_flight_continues_past_the_previous_stop() {
        // Live capture, 514.808--581.375 s: restore its first physical state,
        // then replay the pilot inputs (not the recorded resulting forces).
        let capture = include_str!("../../../logs/flight-traces/2026-09-09-high-rate-stop.csv");
        let mut lines = capture.lines();
        let columns: Vec<_> = lines.next().unwrap().split(',').collect();
        let column = |name: &str| columns.iter().position(|c| *c == name).unwrap();
        let first: Vec<_> = lines.next().unwrap().split(',').collect();
        let number = |row: &[&str], name: &str| row[column(name)].parse::<f64>().unwrap();
        let vector = |row: &[&str], names: [&str; 3]| {
            DVec3::new(
                number(row, names[0]),
                number(row, names[1]),
                number(row, names[2]),
            )
        };
        let (ephemeris, mut flight) = fixture();
        flight.flight_time_s = (number(&first, "t_s") * 120.0).round() / 120.0;
        flight.state = RigidBodyState::new(
            vector(&first, ["pos_x_m", "pos_y_m", "pos_z_m"]),
            vector(&first, ["vel_x_mps", "vel_y_mps", "vel_z_mps"]),
            DQuat::from_xyzw(
                number(&first, "quat_x"),
                number(&first, "quat_y"),
                number(&first, "quat_z"),
                number(&first, "quat_w"),
            )
            .normalize(),
            vector(&first, ["omega_x_rps", "omega_y_rps", "omega_z_rps"]),
        )
        .unwrap();
        flight.surface_input = vector(&first, ["surface_pitch", "surface_yaw", "surface_roll"]);
        let mut max_rate: f64 = 0.0;
        for line in lines {
            let row: Vec<_> = line.split(',').collect();
            assert_eq!(row[column("control_mode")], "DIRECT / RAW");
            flight.control_input = vector(&row, ["pitch_cmd", "yaw_cmd", "roll_cmd"]);
            flight.throttle = number(&row, "throttle");
            flight.engine_active = row[column("engine")] == "1";
            flight.sas_enabled = row[column("sas")] == "1";
            flight.rcs_enabled = row[column("rcs")] == "1";
            flight
                .advance(&ephemeris, ControlMode::Direct, FLIGHT_STEP_S)
                .unwrap();
            max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
        }
        flight.control_input = DVec3::ZERO;
        flight
            .advance(&ephemeris, ControlMode::Direct, 60.0)
            .unwrap();
        println!(
            "recorded spin replay: t={:.3} s, max rate={max_rate:.6}, final rate={:.6}",
            flight.flight_time_s,
            flight.state.angular_velocity_body_rps.length()
        );
        assert!(flight.flight_time_s > 640.0);
    }

    #[test]
    fn production_flight_is_stable_for_fifteen_minutes_and_independent_of_render_cadence() {
        let (ephemeris, mut flight) = fixture();
        let (_, mut other) = fixture();
        let start = std::time::Instant::now();
        let mut min_alt = f64::INFINITY;
        let mut max_aoa: f64 = 0.0;
        let mut max_rate: f64 = 0.0;
        for frame in 0..45_000 {
            flight
                .advance(&ephemeris, ControlMode::Navball, 0.02)
                .unwrap();
            // Same 0.02 seconds split into two irregular render frames.
            other
                .advance(&ephemeris, ControlMode::Navball, 0.007)
                .unwrap();
            other
                .advance(&ephemeris, ControlMode::Navball, 0.013)
                .unwrap();
            let body = ephemeris
                .body_state(flight.reference_body, SimTime(flight.flight_time_s))
                .unwrap();
            let k = local_air_kinematics(
                flight.atmosphere,
                flight.state,
                body,
                flight.planet_radius_m,
            )
            .unwrap();
            min_alt = min_alt.min(k.altitude_m);
            max_aoa = max_aoa.max(conventional_angle_of_attack_deg(k.air_velocity_body_mps).abs());
            max_rate = max_rate.max(flight.state.angular_velocity_body_rps.length());
            assert!(
                k.altitude_m > 100.0,
                "surface departure failed at frame {frame}: {}",
                k.altitude_m
            );
            assert!(
                max_aoa < 22.0,
                "uncommanded stall at frame {frame}: {max_aoa}"
            );
            assert!(max_rate < 0.35, "uncommanded spin: {max_rate}");
            assert!(
                (flight.state.position_inertial_m - other.state.position_inertial_m).length()
                    < 1.0e-6
            );
            assert!(
                (flight.state.angular_velocity_body_rps - other.state.angular_velocity_body_rps)
                    .length()
                    < 1.0e-10
            );
        }
        println!(
            "900 s production path: min altitude {min_alt:.3} m, max AoA {max_aoa:.3} deg, max rate {max_rate:.6} rad/s; two runs {:?}",
            start.elapsed()
        );
    }

    #[test]
    fn sas_allows_full_orbital_turn_and_holds_after_release() {
        for command in [DVec3::Y, DVec3::X] {
            let (ephemeris, mut flight) = fixture();
            let body = ephemeris.body(flight.reference_body).unwrap();
            let origin = ephemeris
                .body_state(flight.reference_body, SimTime::EPOCH)
                .unwrap();
            let radius = body.radius_m + 300_000.0;
            flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
            flight.state.velocity_inertial_mps =
                origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
            flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
            flight.sas_target_orientation = DQuat::IDENTITY;
            flight.engine_active = false;
            flight.control_input = command;
            let axis = if command == DVec3::Y {
                DVec3::NEG_Z
            } else {
                DVec3::NEG_Y
            };
            let mut accumulated_angle = 0.0;
            for _ in 0..5400 {
                let previous = flight.state.orientation_body_to_inertial;
                flight
                    .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
                    .unwrap();
                let delta = (previous.inverse() * flight.state.orientation_body_to_inertial)
                    .to_scaled_axis();
                assert!(
                    delta.dot(axis) >= -1.0e-8,
                    "SAS reversed a commanded orbital turn"
                );
                accumulated_angle += delta.dot(axis);
            }
            assert!(
                accumulated_angle.to_degrees() > 360.0,
                "turn stopped at {} deg",
                accumulated_angle.to_degrees()
            );
            flight.control_input = DVec3::ZERO;
            let released = flight.state.orientation_body_to_inertial;
            flight
                .advance(&ephemeris, ControlMode::Navball, 20.0)
                .unwrap();
            let hold_error = flight
                .state
                .orientation_body_to_inertial
                .angle_between(released)
                .to_degrees();
            println!(
                "orbital turn: {:.2} deg, hold error {hold_error:.6} deg, rate {:?}",
                accumulated_angle.to_degrees(),
                flight.state.angular_velocity_body_rps
            );
            assert!(hold_error < 1.0, "SAS hold error {hold_error}");
            assert!(flight.state.angular_velocity_body_rps.length() < 0.001);
        }
    }

    #[test]
    fn guidance_and_direct_modes_remain_finite_through_control_changes() {
        let (ephemeris, mut flight) = fixture();
        for mode in [
            ControlMode::Navball,
            ControlMode::MouseAim,
            ControlMode::Rate,
            ControlMode::Direct,
        ] {
            for frame in 0..1500 {
                flight.control_input = if (200..250).contains(&frame) {
                    DVec3::new(0.4, 0.15, 0.2)
                } else {
                    DVec3::ZERO
                };
                flight.advance(&ephemeris, mode, 0.02).unwrap();
                assert!(flight.state.velocity_inertial_mps.is_finite());
            }
        }
    }
    #[test]
    fn bank_rotates_lift_out_of_the_vertical_plane() {
        let (_, flight) = fixture();
        let env = flight
            .atmosphere
            .aero_environment(500.0, DVec3::ZERO)
            .unwrap();
        let mut vertical = vec![];
        for bank in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0] {
            let attitude = DQuat::from_rotation_x(bank.to_radians())
                * DQuat::from_rotation_y(-8.0_f64.to_radians());
            let force = flight
                .aero_model
                .evaluate_state(
                    AeroState::new(attitude.inverse() * DVec3::X * 180.0, DVec3::ZERO),
                    env,
                    &flight.vehicle.aero_geometry,
                )
                .unwrap()
                .force_body_n;
            let inertial = attitude * force;
            assert!(inertial.is_finite());
            vertical.push(inertial.z);
            eprintln!(
                "bank {bank:5.0} deg: vertical aero force {:10.3} N; side {:10.3} N",
                inertial.z, inertial.y
            );
        }
        assert!(vertical[0] > 100.0);
        for (i, angle) in [0.0_f64, 45.0, 90.0, 135.0, 180.0, 270.0]
            .into_iter()
            .enumerate()
        {
            assert!((vertical[i] / vertical[0] - angle.to_radians().cos()).abs() < 1.0e-10);
        }
    }

    fn circular_orbit_fixture(altitude_m: f64) -> (BakedEphemeris, FlightAuthority) {
        let (ephemeris, mut flight) = fixture();
        let body = ephemeris.body(flight.reference_body).unwrap();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        let radius = body.radius_m + altitude_m;
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * radius;
        flight.state.velocity_inertial_mps =
            origin.velocity_inertial + DVec3::X * (body.mu / radius).sqrt();
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        (ephemeris, flight)
    }

    #[test]
    fn coast_threshold_brackets_vacuum() {
        let (_, flight) = fixture();
        let sea_level = flight.atmosphere.sample(0.0).unwrap().density_kg_m3;
        assert!(
            sea_level > COAST_DENSITY_KG_M3,
            "launch pad must be Aero, got {sea_level}"
        );
        let high = flight.atmosphere.sample(300_000.0).unwrap().density_kg_m3;
        assert!(
            high < COAST_DENSITY_KG_M3,
            "300 km must be Coast, got {high}"
        );
        assert_eq!(FlightRegime::Aero.label(), "AERO");
        assert_eq!(FlightRegime::Coast.label(), "COAST");
    }

    #[test]
    fn coast_freezes_trim_and_keeps_rcs_authority() {
        let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
        flight
            .advance(&ephemeris, ControlMode::Navball, FLIGHT_STEP_S)
            .unwrap();
        assert_eq!(flight.regime(), FlightRegime::Coast);
        // Assisted trim solve is frozen in vacuum: surfaces hold, no stall
        // chase from a near-singular effectiveness matrix.
        assert_eq!(flight.surface_input, DVec3::ZERO);
        // Full yaw stick still turns the craft on RCS alone.
        flight.control_input = DVec3::Y;
        flight
            .advance(&ephemeris, ControlMode::Navball, 2.0)
            .unwrap();
        assert_eq!(flight.regime(), FlightRegime::Coast);
        assert_eq!(flight.surface_input, DVec3::ZERO);
        assert!(
            flight.state.angular_velocity_body_rps.length() > 0.05,
            "RCS must answer in coast, got {:?}",
            flight.state.angular_velocity_body_rps
        );
    }

    #[test]
    fn coast_orbit_stays_bounded_without_stops() {
        let (ephemeris, mut flight) = circular_orbit_fixture(300_000.0);
        for _ in 0..30_000 {
            // Every step must succeed: leaving the atmosphere never latches
            // FLIGHT STOPPED, it switches the solver to Coast instead.
            flight
                .advance(&ephemeris, ControlMode::Navball, 0.02)
                .unwrap();
            assert_eq!(flight.regime(), FlightRegime::Coast);
        }
        assert!((flight.flight_time_s - 600.0).abs() < 1.0);
        let body = ephemeris.body(flight.reference_body).unwrap();
        let state = ephemeris
            .body_state(flight.reference_body, SimTime(flight.flight_time_s))
            .unwrap();
        let altitude =
            (flight.state.position_inertial_m - state.position_inertial).length() - body.radius_m;
        assert!(
            (280_000.0..320_000.0).contains(&altitude),
            "coast must hold the orbit, altitude drifted to {altitude}"
        );
    }

    #[test]
    fn warp_scale_batch_jump_covers_an_hour_without_ticks() {
        // 100kx warp equivalence: one advance call carrying an hour of sim
        // time must ride batch jumps, not 432k ticks. Attitude stays idle
        // (Direct, zero input/rates) so batches stay eligible.
        let (ephemeris, mut flight) = fixture();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
        flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        flight
            .advance(&ephemeris, ControlMode::Direct, 0.05)
            .expect("coast warms the rails");
        assert!(!flight.rails.is_empty());
        flight
            .advance(&ephemeris, ControlMode::Direct, 3600.0)
            .expect("warp hour advances");
        assert!(
            (flight.rails_advanced_this_frame - 3600.0).abs() < 1.0,
            "warp must ride batches, advanced {}",
            flight.rails_advanced_this_frame
        );
        assert_eq!(
            flight.steps_this_frame, 0,
            "no per-tick solver work at warp"
        );
        assert!(
            (flight.flight_time_s - 3600.05).abs() < 1.0,
            "clock must advance by the warped hour, got {}",
            flight.flight_time_s
        );
    }

    #[test]
    fn unpowered_vacuum_coast_rides_the_shared_rails() {
        // One trajectory, two consumers: in an unpowered vacuum coast the
        // flight loop must sample translation from the baked rails (the same
        // bake the map prediction draws) instead of integrating per tick.
        // Deep space gives exactly-zero sampled density, which is the same
        // condition that permits the aero fast path.
        let (ephemeris, mut flight) = fixture();
        let origin = ephemeris
            .body_state(flight.reference_body, SimTime::EPOCH)
            .unwrap();
        flight.state.position_inertial_m = origin.position_inertial + DVec3::Z * 1.0e9;
        flight.state.velocity_inertial_mps = origin.velocity_inertial + DVec3::X * 100.0;
        flight.state.orientation_body_to_inertial = DQuat::IDENTITY;
        flight.state.angular_velocity_body_rps = DVec3::ZERO;
        flight.sas_target_orientation = DQuat::IDENTITY;
        flight.engine_active = false;
        flight.throttle = 0.0;
        flight.control_input = DVec3::ZERO;
        flight
            .advance(&ephemeris, ControlMode::Direct, 0.05)
            .expect("coast advances");
        assert!(
            !flight.rails.is_empty(),
            "vacuum coast must bake the shared rails"
        );
        // The bake arms its wake in simulation time: the loop waits on the
        // horizon event instead of polling the trajectory.
        let wake = flight.scheduler.next().expect("coast arms a wake");
        assert!(
            matches!(wake.kind, thessa_sim_core::ScheduledKind::RailsHorizon),
            "deep-space coast wake must be the horizon, got {:?}",
            wake.kind
        );
        assert!(
            wake.time.seconds() > flight.flight_time_s,
            "wake must lie ahead"
        );
        // Translation now tracks the rails bake to interpolation precision.
        let (position, _) = flight.inertial_state_m();
        let (sampled, _) = flight
            .rails
            .sample_at(SimTime(flight.flight_time_s))
            .expect("rails cover the flown epoch");
        assert!(
            (position - sampled).length() < 1.0,
            "flown state left the rails: {:?} m",
            (position - sampled).length()
        );
        // A warm cache consumes an entire frame without solver ticks.
        flight.state.angular_velocity_body_rps = DVec3::X * 0.25;
        let before_orientation = flight.state.orientation_body_to_inertial;
        flight
            .advance(&ephemeris, ControlMode::Direct, 3.2)
            .unwrap();
        assert_eq!(flight.steps_this_frame, 0);
        assert!((flight.rails_advanced_this_frame - 3.2).abs() < 1e-10);
        let expected = before_orientation * DQuat::from_rotation_x(0.8);
        assert!(
            flight
                .state
                .orientation_body_to_inertial
                .abs_diff_eq(expected.normalize(), 1e-12)
        );
        // SAS needs to see rotation and must not be skipped for a whole batch.
        flight.sas_enabled = true;
        flight.rcs_enabled = true;
        let before = flight.state;
        let surface_input = flight.surface_input;
        assert_eq!(
            flight
                .try_advance_cached_coast(&ephemeris, ControlMode::Navball, 3.2)
                .unwrap(),
            0.0
        );
        assert_eq!(flight.state, before);
        assert_eq!(flight.surface_input, surface_input);
        // Lighting the engine invalidates the bake: thrust is a maneuver.
        flight.engine_active = true;
        flight.throttle = 1.0;
        flight
            .advance(&ephemeris, ControlMode::Direct, 0.05)
            .expect("powered flight advances");
        assert!(
            flight.rails_advanced_this_frame == 0.0 && flight.steps_this_frame > 0,
            "thrust must disable riding the gravity forecast"
        );
    }
}
