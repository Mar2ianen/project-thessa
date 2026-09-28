//! Authoritative flight runtime: input conditioning, regime selection,
//! the fixed-step advance loop and rails-bake orchestration.
//!
//! Moved verbatim from the client (which keeps only rendering, input
//! mapping and telemetry wiring). The only adaptations are mechanical:
//! Bevy types replaced by glam, the rails-bake worker behind [`BakeQueue`],
//! and render-view fields dropped (the client syncs its view after each
//! advance; stepping never reads them).

mod advance;
mod contact_step;
mod control_step;
mod kinematics;
mod rails;
mod subsystems;
mod trace;

pub use kinematics::{LocalAirKinematics, TerrainTrackCoverage, local_air_kinematics};
pub use trace::{FlightTraceWriter, conventional_angle_of_attack_deg};

use std::{path::Path, sync::Arc};

use glam::{DMat3, DQuat, DVec3};
use thessa_collision::{
    CollisionDebugSnapshot, CollisionFrame, ContactSummary, DynamicBodyConfig, KinematicBodyId,
    LandingLegContactSample, WheelContactSample,
};
use thessa_flight_control::{
    ActuatorDynamics, ControlDemand, DirectionFrame, DirectionTarget, GuidanceIntent,
    PropulsionDemand, RollPolicy, SpacecraftControlLaw,
};
use thessa_sim_core::{
    AeroConfig, AeroGeometry, AeroResult, AeroSimdScratch, AeroState, AtmosphereConfig,
    AtmosphereError, AuxiliaryPowerUnitCommand, AuxiliaryPowerUnitState, BakedEphemeris, BodyId,
    BodyState, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_VELOCITY_TOL_MPS, CollisionMaterial,
    ControlChannels, ElectricalPowerCommand, ElectricalPowerState, ElectricalPowerTelemetry,
    EphemerisFrame, EventScheduler, FlightError, FlightForces, FlightStepInput, FusionTorchCommand,
    GravityField, JetCommand, LandingGearActuatorPoint, LandingLegState, OnRailsCache,
    PanelAeroModel, PanelSoA, ParachuteCommand, ParachuteEnvironment, ParachuteLoad,
    ParachutePhase, ParachuteState, PropellerDriveCommand, PulsedFusionCommand, PulsedFusionState,
    ReactionWheelAllocation, RigidBodyState, ScheduledEvent, ScheduledKind, SimTime,
    TestParticleState, TickIntegratorConfig, TurbopropCommand, VehicleDefinition,
    VehiclePartCommand, VehiclePropulsionAllocation, VehicleResourceState, WORLD_TICK_S,
    WheelBrakeState, WheelChassisActuatorPoint, WheelChassisState, WheelDrivePoint,
    X15StarterProfile, allocate_reaction_wheels_with_enabled_banks, control_surface_commands,
    evaluate_flight_forces, evaluate_flight_forces_soa, evaluate_flight_forces_with_aero_result,
    integrate_attitude_step, integrate_rigid_body_step_soa,
    integrate_rigid_body_step_with_aero_result,
};
use thessa_worldgen_rocky::field::{ObstacleReport, ObstacleTrackCertificate, PlanetField};

use crate::{
    BakeQueue, BakedRails, ControlMode, FlightRegime, InlineBakeQueue, RailsBakeRequest,
    contact::{ContactActivation, ContactRuntime},
    control::{
        PILOT_ATTITUDE_COMMAND_RATE_RAD_S, SURFACE_COMMAND_RATE_S, allocate_rcs,
        allocate_rcs_force, attitude_demand, rcs_moment, slew_surface_command, solve_aero_trim,
    },
};

/// Advance the exact production flight path at a fixed physics cadence.
/// Render frames only contribute elapsed time; they never set solver dt.
pub const FLIGHT_STEP_S: f64 = WORLD_TICK_S;
/// Baked coverage ahead (s) below which a fresh worker bake starts while
/// riding, so sustained warp never stalls at the horizon end.
const PROACTIVE_REBAKE_AHEAD_S: f64 = 86_400.0;

/// Trim-conditioning threshold, not a force cutoff. At low density the
/// surface-response matrix becomes ill-conditioned; RCS handles attitude.
/// The flight solver still evaluates residual aero loads at every nonzero
/// density (q grows with speed squared even above this threshold).
pub const COAST_DENSITY_KG_M3: f64 = 1.0e-7;

// Longest single rails-batch jump. Bounds wake/event latency and
// forces periodic cache revalidation on long horizons.
pub const MAX_COAST_BATCH_JUMP_S: f64 = 3600.0;

pub const X15_STALL_ANGLE_DEG: f64 = 22.0;
const PILOT_SURFACE_CLEARANCE_M: f64 = 5.0;
const PILOT_START_ALTITUDE_M: f64 = 500.0;
// Solver guard rail, not physics: interlunar transfers range billions of
// metres from the reference body, so the cap covers the whole Nereid system
// plus escape margin. Tripping it still latches FLIGHT STOPPED.
const MAX_PILOT_ALTITUDE_M: f64 = 2.0e10;
const MAX_PILOT_RELATIVE_SPEED_MPS: f64 = 50_000.0;
const MAX_PILOT_ANGULAR_RATE_RPS: f64 = 25.0;
/// Half-size of the kinematic terrain patch maintained under a
/// contact-active craft. The patch follows its body-fixed anchor and is
/// recentered before the craft reaches its edge.
const CONTACT_PATCH_HALF_M: f64 = 25.0;
const CONTACT_PATCH_HALF_THICK_M: f64 = 1.0;
const CONTACT_PATCH_RECENTER_DISTANCE_M: f64 = CONTACT_PATCH_HALF_M * 0.75;

fn normalize_direction(vector: DVec3, label: &str) -> Result<DVec3, FlightError> {
    if !vector.is_finite() {
        return Err(FlightError::InvalidInput(format!("{label} must be finite")));
    }
    let length = vector.length();
    if !length.is_finite() || length <= 1.0e-12 {
        return Err(FlightError::InvalidInput(format!(
            "{label} must be nonzero"
        )));
    }
    Ok(vector / length)
}

fn surface_tangent_basis(up: DVec3) -> Result<(DVec3, DVec3), FlightError> {
    let helper = if up.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
    let east = normalize_direction(helper.cross(up), "surface east")?;
    let north = normalize_direction(up.cross(east), "surface north")?;
    Ok((east, north))
}

/// Body-fixed ground direction for terrain sampling. This is the exact
/// mapping the endpoint guard uses: inertial relative position into the
/// rotating body frame through the `x/z/-y` axis convention. Sharing it
/// keeps the activation evidence and the guard on the same terrain sample.
fn ground_dir_body_fixed(
    relative_position_inertial_m: DVec3,
    flight_time_s: f64,
    body_rotation_period_s: f64,
) -> DVec3 {
    DQuat::from_rotation_y(-(flight_time_s * std::f64::consts::TAU / body_rotation_period_s))
        * DVec3::new(
            relative_position_inertial_m.x,
            relative_position_inertial_m.z,
            -relative_position_inertial_m.y,
        )
        .normalize()
}

/// Rotation from the terrain sampler's body-fixed axes to inertial axes.
/// `ground_dir_body_fixed` uses an x/z/-y axis remap followed by body spin;
/// this is its exact inverse and lets contact patches retain a body-fixed
/// anchor while the planet rotates.
fn body_fixed_to_inertial_rotation(time_s: f64, body_rotation_period_s: f64) -> DQuat {
    let spin = time_s * std::f64::consts::TAU / body_rotation_period_s;
    DQuat::from_rotation_z(spin) * DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2)
}

#[derive(Debug, Clone, Copy)]
struct ContactPatchAnchor {
    direction_body_fixed: DVec3,
    orientation_body_fixed: DQuat,
    terrain_height_m: f64,
}

/// Assemble the flight-step load input shared by the free-flight integrator
/// and the contact-active tick. Both paths sample identical loads; only the
/// integrator differs.
#[allow(clippy::too_many_arguments)]
fn powered_step_input(
    state: RigidBodyState,
    kinematics: LocalAirKinematics,
    body_velocity_inertial_mps: DVec3,
    gravity: DVec3,
    jet_moment: DVec3,
    rcs_force_body_n: DVec3,
    propulsion_force_body_n: DVec3,
    band_drag_body_n: DVec3,
    parachute_force_body_n: DVec3,
    parachute_moment_body_nm: DVec3,
    skip_aero: bool,
) -> FlightStepInput {
    FlightStepInput {
        altitude_m: kinematics.altitude_m.max(0.0),
        gravity_acceleration_inertial_mps2: gravity,
        position_body_m: kinematics.relative_position_body_m,
        wind_velocity_body_mps: state.orientation_body_to_inertial.inverse()
            * body_velocity_inertial_mps,
        extra_force_body_n: propulsion_force_body_n
            + rcs_force_body_n
            + band_drag_body_n
            + parachute_force_body_n,
        extra_moment_body_nm: jet_moment + parachute_moment_body_nm,
        skip_aero,
    }
}

/// Result of asking the rails fast path to serve the current accumulator.
/// `WaitingForBake` is deliberately distinct from `NotEligible`: the former
/// must yield to the worker instead of falling back to a long per-tick replay.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CoastAdvance {
    NotEligible,
    WaitingForBake,
    Advanced(f64),
}

struct ControlAllocation {
    moment_body_nm: DVec3,
    rcs_duties: Vec<f64>,
    /// Post-actuator SoA sample reused by this tick's rigid-body force path.
    aero_result: Option<AeroResult>,
}

struct ParachuteStep {
    force_body_n: DVec3,
    moment_body_nm: DVec3,
    states: Vec<ParachuteState>,
    loads: Vec<ParachuteLoad>,
}

fn rebase_rigid_body_state(
    state: &mut RigidBodyState,
    frame_shift_body_m: DVec3,
) -> Result<(), FlightError> {
    if frame_shift_body_m == DVec3::ZERO {
        return Ok(());
    }
    // Re-centering geometry by `frame_shift` moves the represented COM by its
    // opposite vector. Preserve the rigid-body velocity at that shifted point.
    let center_shift_body_m = -frame_shift_body_m;
    let orientation = state.orientation_body_to_inertial;
    let offset_inertial_m = orientation * center_shift_body_m;
    let angular_velocity_inertial_rps = orientation * state.angular_velocity_body_rps;
    state.position_inertial_m += offset_inertial_m;
    state.velocity_inertial_mps += angular_velocity_inertial_rps.cross(offset_inertial_m);
    RigidBodyState::new(
        state.position_inertial_m,
        state.velocity_inertial_mps,
        state.orientation_body_to_inertial,
        state.angular_velocity_body_rps,
    )
    .map(|_| ())
}

/// Authoritative flight model for the first playable vehicle.
///
/// The authoritative equations stay in `thessa-sim-core`; this runtime supplies
/// game input, installed-vehicle commands, and a telemetry bridge. Mounted
/// resource consumers draw from reachable tank inventory on each fixed step;
/// the legacy starter-profile propulsion path remains available for the X-15
/// flight slice. Render-view state lives client-side; stepping never reads it.
pub struct FlightAuthority {
    pub reference_body: BodyId,
    pub planet_radius_m: f64,
    /// Spin period used by both terrain orientation and atmospheric rotation.
    /// Tidally locked bodies fall back to their design orbital period.
    body_rotation_period_s: f64,
    pub terrain_field: Option<Arc<PlanetField>>,
    /// Recipe-declared maximum elevation in metres (baked-in data, no field
    /// build). Upper-bounds every baked height by construction (the baker
    /// clamps into `[height_min_m, height_max_m]`), so batch certification
    /// stays sound before terrain loads and for craft that never will.
    pub recipe_max_elevation_m: f64,
    pub launch_site_dir: Option<[f64; 3]>,
    pub vehicle: VehicleDefinition,
    /// Mutable propellant inventory and solid-motor burn clocks.
    pub resource_state: VehicleResourceState,
    /// Authoritative electrical-bus state and command. Physical mounted APU
    /// output is injected into the command each fixed step.
    pub electrical_power_state: ElectricalPowerState,
    pub electrical_power_command: ElectricalPowerCommand,
    pub electrical_power_telemetry: Option<ElectricalPowerTelemetry>,
    /// Persistent mounted APU shaft state and caller-selected commands.
    pub auxiliary_power_unit_states: Vec<AuxiliaryPowerUnitState>,
    pub auxiliary_power_unit_commands: Vec<AuxiliaryPowerUnitCommand>,
    /// Persistent shaft/ESTOC state for installed air-breathing jets.
    pub jet_commands: Vec<JetCommand>,
    /// Requested feed for each installed electric thruster (actual flow is
    /// capped by shared-bus power and reachable inventory).
    pub electric_thruster_requested_flow_kg_s: Vec<f64>,
    /// User-selected continuous fusion working-fluid requests.
    pub fusion_torch_commands: Vec<FusionTorchCommand>,
    /// Persistent pulsed-fusion buffer state and arming/charge commands.
    pub pulsed_fusion_states: Vec<PulsedFusionState>,
    pub pulsed_fusion_commands: Vec<PulsedFusionCommand>,
    /// Installed piston/electric propeller-drive commands.
    pub propeller_drive_commands: Vec<PropellerDriveCommand>,
    /// Installed turboprop shaft state and commands.
    pub turboprop_commands: Vec<TurbopropCommand>,
    pub aero_model: PanelAeroModel,
    /// Compiled SoA geometry and reusable SIMD scratch for the authoritative
    /// per-step aero evaluation. The reference geometry is retained so hinge
    /// motion is applied absolutely rather than accumulated numerically.
    aero_panels: PanelSoA,
    aero_scratch: AeroSimdScratch,
    control_reference_geometry: AeroGeometry,
    control_deflections_rad: Vec<f64>,
    pub atmosphere: AtmosphereConfig,
    pub state: RigidBodyState,
    pub sas_target_orientation: DQuat,
    pub relative_position_m: DVec3,
    pub flight_time_s: f64,
    pub world_tick: thessa_sim_core::WorldTick,
    /// Requested normalized propulsion command exposed in snapshots. The
    /// physical command is kept separately below so typed/autopilot paths can
    /// model spool response without changing the legacy wire semantics.
    pub throttle: f64,
    pub engine_active: bool,
    pub sas_enabled: bool,
    pub rcs_enabled: bool,
    pub reaction_wheels_enabled: bool,
    /// Body-frame moment currently supplied by installed reaction-wheel banks.
    pub reaction_wheel_torque_body_nm: DVec3,
    pub gear_down: bool,
    /// Installed parachutes' automatic pressure-trigger command.
    pub parachutes_armed: bool,
    /// Normalized service-brake command applied to every installed wheel
    /// brake actuator during contact-active ticks.
    pub wheel_brake_command: f64,
    /// Signed traction-motor command; wheel chassis without a drive ignore it.
    pub wheel_drive_command: f64,
    wheel_spin_rad_s: Vec<Vec<f64>>,
    wheel_brake_states: Vec<Vec<WheelBrakeState>>,
    wheel_chassis_states: Vec<WheelChassisState>,
    landing_leg_states: Vec<LandingLegState>,
    reaction_wheel_bank_enabled: Vec<bool>,
    wheel_chassis_deployment_commands: Vec<bool>,
    landing_leg_deployment_commands: Vec<bool>,
    parachute_states: Vec<ParachuteState>,
    last_parachute_loads: Vec<ParachuteLoad>,
    last_wheel_contacts: Vec<WheelContactSample>,
    last_wheel_drive_points: Vec<(usize, u16, WheelDrivePoint)>,
    last_wheel_gear_actuators: Vec<(usize, WheelChassisActuatorPoint)>,
    last_landing_leg_contacts: Vec<LandingLegContactSample>,
    last_landing_leg_actuators: Vec<(usize, LandingGearActuatorPoint)>,
    /// Manual body-axis command: pitch, yaw, roll in normalized units.
    pub control_input: DVec3,
    pub surface_input: DVec3,
    pub actuator_saturated: bool,
    pub regime: FlightRegime,
    pub accumulator_s: f64,
    pub steps_this_frame: u32,
    pub rails_advanced_this_frame: f64,
    /// True when the optional cooperative wall budget stopped this call.
    /// The server uses this only as a yield point: a saturated call is
    /// followed immediately by another call, so the budget is not a duty
    /// cycle or a global CPU/TPS cap.
    pub work_budget_exhausted: bool,
    /// The last advance yielded because an eligible unpowered vacuum coast
    /// is waiting for its background rails bake.
    pub waiting_for_rails_bake: bool,
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
    /// Opt-in contact-active solver. `None` (default) is pure free flight;
    /// `Some` observes terrain evidence with hysteresis and integrates
    /// contact-active ticks through Rapier instead of the free-flight
    /// integrator. Exactly one integrator owns a body per tick.
    contact: Option<ContactRuntime>,
    /// Kinematic terrain patch tracked by `contact`, if any.
    contact_patch: Option<KinematicBodyId>,
    /// Body-fixed sample and tangent frame anchoring the active terrain patch.
    contact_patch_anchor: Option<ContactPatchAnchor>,
    /// Simulation-time event queue: the rails bake arms its wake here, and
    /// `advance` drains due events instead of polling them every tick.
    pub scheduler: EventScheduler,
    /// Last fired wake, for the HUD orbit line.
    pub wake_notice: Option<String>,
    /// Every authoritative wake emitted by the scheduler, retained in due
    /// order for the server/autopilot bridge. The HUD keeps only the latest
    /// string, but event consumers must not lose simultaneous or repeated
    /// domain events.
    pub wake_events: Vec<ScheduledEvent>,
    /// Test hook: force the residual-driven second trim pass every tick.
    /// Production runs adaptive (a singular first pass still breaks exact —
    /// its replay would be identical). The A/B regression test pins the
    /// adaptive path against this reference.
    pub(crate) force_trim_two_pass: bool,
    /// Trim solver telemetry: ticks that entered the Newton solve and ticks
    /// that ran the second pass. The gap is skipped cooperative work, never
    /// dropped physics — every served tick still flies a converged command.
    pub(crate) trim_solves: u64,
    pub(crate) trim_second_passes: u64,
    /// Reusable per-timestamp ephemeris frame. One memoized evaluation per
    /// tick serves the reference body, gravity and guard reads; buffers grow
    /// once and are then reused allocation-free. Telemetry only in the sense
    /// that it never changes physics values — see `EphemerisFrame`.
    ephemeris_frame: EphemerisFrame,
    /// Sum of panel reference areas, compiled once. The upper-band drag
    /// model reads it every tick; the geometry shape never changes at
    /// runtime (control surfaces rotate in place).
    reference_area_m2: f64,
    /// Explicit translation demand supplied by a declarative plan. It is
    /// realized by bounded paired RCS force effectors on the fixed-step path.
    explicit_force_demand_body_n: Option<DVec3>,
    /// Explicit moment demand supplied by a declarative plan. It is consumed
    /// by the same trim/RCS allocator as typed guidance and cleared by the
    /// public demand entry point after its cooperative advance returns.
    explicit_moment_demand_nm: Option<DVec3>,
    /// State-dependent direction guidance must refresh its target before
    /// every physical tick. It also disables the free-translation rails
    /// batch, whose constant-attitude shortcut cannot observe a moving
    /// surface/orbit frame.
    guidance_state_dependent: bool,
    /// Physical propulsion command after the actuator layer. Legacy pilot
    /// input keeps this path immediate; typed guidance enables the response
    /// model explicitly at its boundary.
    propulsion_actual: f64,
    propulsion_target: f64,
    propulsion_dynamics_active: bool,
    propulsion_dynamics: ActuatorDynamics,
    /// Optional per-mount throttle overrides; `None` follows the vessel lever.
    engine_throttle_overrides: Vec<Option<f64>>,
    system_throttle_overrides: Vec<Vec<Option<f64>>>,
    pub last_propulsion_force_body_n: DVec3,
    pub last_propellant_flow_kg_s: f64,
    pub fuel_limited: bool,
}

impl FlightAuthority {
    /// Initialize an opt-in, physically consistent circular orbit benchmark.
    /// Position and velocity are inertial and include the reference body's
    /// ephemeris translation; no render transform or camera state is touched.
    pub fn initialize_circular_orbit(
        &mut self,
        ephemeris: &BakedEphemeris,
        altitude_m: f64,
        plane_normal: DVec3,
    ) -> Result<(), FlightError> {
        if !altitude_m.is_finite() || altitude_m < 0.0 {
            return Err(FlightError::InvalidInput(
                "orbit altitude must be finite and non-negative".into(),
            ));
        }
        let normal = normalize_direction(plane_normal, "orbit plane normal")?;
        let body = ephemeris
            .body(self.reference_body)
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        let radius = body.radius_m + altitude_m;
        if !radius.is_finite() || radius <= 0.0 || !body.mu.is_finite() || body.mu <= 0.0 {
            return Err(FlightError::InvalidInput(
                "reference body has invalid orbit parameters".into(),
            ));
        }
        let body_state = ephemeris
            .body_state(self.reference_body, SimTime(self.flight_time_s))
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        let helper = if normal.z.abs() < 0.9 {
            DVec3::Z
        } else {
            DVec3::X
        };
        let radial = normalize_direction(normal.cross(helper), "orbit radial")?;
        let prograde = normalize_direction(normal.cross(radial), "orbit prograde")?;
        let lateral = radial.cross(prograde);
        let relative_position = radial * radius;
        let state = RigidBodyState::new(
            body_state.position_inertial + relative_position,
            body_state.velocity_inertial + prograde * (body.mu / radius).sqrt(),
            DQuat::from_mat3(&DMat3::from_cols(prograde, lateral, radial)),
            DVec3::ZERO,
        )?;
        self.state = state;
        self.sas_target_orientation = state.orientation_body_to_inertial;
        self.relative_position_m = relative_position;
        self.launch_site_dir = None;
        self.throttle = 0.0;
        self.engine_active = false;
        self.reset_control_surfaces()?;
        self.accumulator_s = 0.0;
        self.flight_error = None;
        self.regime = FlightRegime::Aero;
        self.rails.invalidate();
        self.bake.reset();
        Ok(())
    }

    pub fn initialize_world_site(
        &mut self,
        field: Arc<PlanetField>,
        dir: [f64; 3],
        ephemeris: &BakedEphemeris,
    ) {
        let up = DQuat::from_rotation_z(self.terrain_spin()) * DVec3::new(dir[0], -dir[2], dir[1]);
        // Pole-safe: the old `(Z - up*up.z).normalize()` divides by ~0 when
        // `up ≈ Z` (polar survey bookmark) and spawns NaN state. The shared
        // helper switches reference axis near the poles; identical to the
        // old basis elsewhere. Fallback is unreachable for unit `up`.
        let (east, north) = surface_tangent_basis(up).unwrap_or((DVec3::X, DVec3::Y));
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
        self.set_legacy_propulsion(0.0, true);
        self.reset_control_surfaces()
            .map_err(|error| error.to_string())?;
        self.explicit_force_demand_body_n = None;
        self.explicit_moment_demand_nm = None;
        self.reaction_wheel_torque_body_nm = DVec3::ZERO;
        self.set_parachutes_armed(false);
        self.parachute_states.fill(ParachuteState::default());
        self.last_parachute_loads.fill(ParachuteLoad::default());
        self.regime = FlightRegime::Aero;
        self.accumulator_s = 0.0;
        self.rails.invalidate();
        self.bake.reset();
        self.scheduler = EventScheduler::new();
        self.wake_notice = None;
        self.wake_events.clear();
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
        self.flight_time_s * std::f64::consts::TAU / self.body_rotation_period_s
    }
    pub fn stop_reason(&self) -> Option<&str> {
        self.flight_error.as_deref()
    }

    pub fn backlog_s(&self) -> f64 {
        self.accumulator_s
    }

    pub fn take_wake_events(&mut self) -> Vec<ScheduledEvent> {
        std::mem::take(&mut self.wake_events)
    }
    pub fn regime(&self) -> FlightRegime {
        self.regime
    }
    pub fn panel_count(&self) -> u32 {
        self.vehicle.aero_geometry.panels.len() as u32
    }

    /// Turn an inertial position track into ground directions, declare the
    /// obstacle discs that cover it, and return the sampled terrain margin.
    /// The report radius is half the largest ground spacing plus the normal
    /// surface clearance, so the returned coverage proof is tied to this
    /// exact track rather than to an arbitrary fixed radius.
    pub fn certify_terrain_track(
        &self,
        ephemeris: &BakedEphemeris,
        track: &[(SimTime, DVec3)],
    ) -> Result<TerrainTrackCoverage, String> {
        let field = self
            .terrain_field
            .as_ref()
            .ok_or_else(|| "terrain track certification requires a loaded field".to_string())?;
        if track.is_empty() {
            return Err("terrain track must contain at least one sample".into());
        }
        let mut ground_dirs = Vec::with_capacity(track.len());
        let mut altitudes = Vec::with_capacity(track.len());
        for (time, position) in track {
            if !time.0.is_finite() || !position.is_finite() {
                return Err("terrain track contains non-finite time or position".into());
            }
            let body_state = ephemeris
                .body_state(self.reference_body, *time)
                .map_err(|error| format!("terrain track body state: {error}"))?;
            let relative = *position - body_state.position_inertial;
            let distance = relative.length();
            if !distance.is_finite() || distance <= 0.0 {
                return Err("terrain track contains a degenerate body-relative position".into());
            }
            ground_dirs.push((relative / distance).to_array());
            altitudes.push(distance - self.planet_radius_m);
        }
        let field_radius_m = field.params.radius_m;
        let mut max_spacing_m: f64 = 0.0;
        for pair in ground_dirs.windows(2) {
            let dot = (pair[0][0] * pair[1][0] + pair[0][1] * pair[1][1] + pair[0][2] * pair[1][2])
                .clamp(-1.0, 1.0);
            max_spacing_m = max_spacing_m.max(field_radius_m * dot.acos());
        }
        let desired_geodesic_radius_m = 0.5 * max_spacing_m + PILOT_SURFACE_CLEARANCE_M;
        if desired_geodesic_radius_m >= 0.5 * std::f64::consts::PI * field_radius_m {
            return Err("terrain track spacing is too large for a finite tangent report".into());
        }
        // declare_obstacles takes a tangent-plane radius. Convert the desired
        // spherical radius back through atan so the coverage proof remains
        // valid for large, but still representable, track gaps.
        let report_radius_m =
            (field_radius_m * (desired_geodesic_radius_m / field_radius_m).tan()).max(32.0);
        let obstacles = field
            .certify_obstacle_track(&ground_dirs, report_radius_m)
            .map_err(|error| format!("terrain obstacle coverage: {error}"))?;
        let min_altitude_m = altitudes.iter().copied().fold(f64::INFINITY, f64::min);
        let min_obstacle_clearance_m = altitudes
            .iter()
            .zip(&obstacles.reports)
            .map(|(altitude, report)| altitude - report.max_height_m)
            .fold(f64::INFINITY, f64::min);
        Ok(TerrainTrackCoverage {
            obstacles,
            min_altitude_m,
            min_obstacle_clearance_m,
        })
    }

    /// Declare one landing or impact footprint against the canonical field.
    /// `None` means terrain is not loaded in this authority instance; the
    /// caller can retain the site and resolve it when an observed field is
    /// available without inventing a zero-height fallback.
    pub fn obstacle_report(
        &self,
        center_dir: [f64; 3],
        radius_m: f64,
    ) -> Result<Option<ObstacleReport>, String> {
        let Some(field) = self.terrain_field.as_ref() else {
            return Ok(None);
        };
        field
            .declare_obstacles(center_dir, radius_m)
            .map(Some)
            .map_err(|error| format!("terrain obstacle report: {error}"))
    }

    /// Build the compatibility X-15 flight used by the starter scene.
    pub fn new(ephemeris: &BakedEphemeris, reference_body: BodyId) -> Result<Self, String> {
        let mut authority = Self::new_with_vehicle(ephemeris, reference_body, x15_vehicle()?)?;
        authority.set_legacy_propulsion(1.0, true);
        Ok(authority)
    }

    /// Build an authoritative flight around a baked vehicle definition.
    /// Procedural engines and tanks start inactive with their authored fill
    /// levels; the compatibility constructor above retains the starter's
    /// running legacy engine.
    pub fn new_with_vehicle(
        ephemeris: &BakedEphemeris,
        reference_body: BodyId,
        vehicle: VehicleDefinition,
    ) -> Result<Self, String> {
        vehicle
            .validate()
            .map_err(|error| format!("vehicle definition is invalid: {error}"))?;
        let body = ephemeris
            .body(reference_body)
            .map_err(|error| format!("reference body is unavailable: {error}"))?;
        let body_state = ephemeris
            .body_state(reference_body, SimTime::EPOCH)
            .map_err(|error| format!("reference body state is unavailable: {error}"))?;
        let gravity = body.mu / body.radius_m.powi(2);
        let mut atmosphere = match body.atmosphere.as_ref() {
            Some(baked) => AtmosphereConfig::from_baked(baked, 288.15, gravity),
            // Compatibility path for hand-built test ephemerides and old
            // baked JSON. New system data always carries composition here.
            None => AtmosphereConfig::new(288.15, 120_000.0, 287.05287, 1.4, gravity),
        }
        .map_err(|error| format!("{} atmosphere is invalid: {error}", body.name))?;
        let rotation_period_s = body
            .rotation_period_s
            .or_else(|| body.tidal_lock.then_some(body.design_period_s).flatten())
            .unwrap_or(80.0 * 3_600.0);
        atmosphere.body_rotation_rad_s =
            DVec3::new(0.0, 0.0, std::f64::consts::TAU / rotation_period_s);

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
        let resource_state = vehicle.initial_resource_state();
        let electrical_power_state = vehicle
            .initial_electrical_power_state()
            .map_err(|error| format!("vehicle electrical power system is invalid: {error}"))?;
        let electrical_power_command =
            ElectricalPowerCommand::idle_for(&vehicle.electrical_power, FLIGHT_STEP_S);
        let auxiliary_power_unit_states = vehicle
            .auxiliary_power_units
            .iter()
            .map(|mount| mount.unit.initial_state())
            .collect();
        let auxiliary_power_unit_commands = vehicle
            .auxiliary_power_units
            .iter()
            .map(|mount| AuxiliaryPowerUnitCommand {
                throttle: 0.0,
                starter_engaged: false,
                generator_load_w: mount.unit.engine.shaft.generator.power_w,
                pneumatic_bleed_power_w: 0.0,
                dt_s: FLIGHT_STEP_S,
            })
            .collect();
        let jet_commands = vehicle
            .jets
            .iter()
            .map(|mount| JetCommand::cold(&mount.engine))
            .collect();
        let electric_thruster_requested_flow_kg_s = vec![0.0; vehicle.electric_thrusters.len()];
        let fusion_torch_commands = vehicle
            .fusion_torches
            .iter()
            .map(|_| FusionTorchCommand {
                available_driver_power_w: 0.0,
                requested_working_flow_kg_s: 0.0,
            })
            .collect();
        let pulsed_fusion_states =
            vec![PulsedFusionState::default(); vehicle.pulsed_fusion_systems.len()];
        let pulsed_fusion_commands = vehicle
            .pulsed_fusion_systems
            .iter()
            .map(|_| PulsedFusionCommand {
                available_charge_power_w: 0.0,
                armed: false,
            })
            .collect();
        let propeller_drive_commands = vehicle
            .propeller_drives
            .iter()
            .map(|_| PropellerDriveCommand {
                throttle: 0.0,
                source_rpm: 0.0,
            })
            .collect();
        let turboprop_commands = vehicle
            .turboprops
            .iter()
            .map(|mount| {
                let mut command = TurbopropCommand::cold(&mount.drive);
                command.shaft.throttle = 0.0;
                command.dt_s = FLIGHT_STEP_S;
                command
            })
            .collect();
        let engine_throttle_overrides = vec![None; vehicle.engines.len()];
        let system_throttle_overrides = vehicle
            .systems
            .iter()
            .map(|mount| vec![None; mount.system.chambers.len()])
            .collect();
        let control_reference_geometry = vehicle.aero_geometry.clone();
        let control_deflections_rad = vec![0.0; vehicle.control_surfaces.len()];
        let wheel_spin_rad_s: Vec<Vec<f64>> = vehicle
            .wheel_chassis
            .iter()
            .map(|chassis| vec![0.0; chassis.wheel_stations.len()])
            .collect();
        let wheel_brake_states: Vec<Vec<WheelBrakeState>> = vehicle
            .wheel_chassis
            .iter()
            .map(|chassis| vec![WheelBrakeState::default(); chassis.wheel_stations.len()])
            .collect();
        let wheel_chassis_states: Vec<WheelChassisState> = vehicle
            .wheel_chassis
            .iter()
            .map(|chassis| {
                chassis
                    .spec
                    .retraction
                    .map(|retraction| retraction.initial_state())
                    .unwrap_or_else(WheelChassisState::deployed)
            })
            .collect();
        let landing_leg_states: Vec<LandingLegState> = vehicle
            .landing_legs
            .iter()
            .map(|leg| leg.spec.initial_state())
            .collect();
        let reaction_wheel_bank_enabled = vec![true; vehicle.reaction_wheels.len()];
        let wheel_chassis_deployment_commands = vec![true; vehicle.wheel_chassis.len()];
        let landing_leg_deployment_commands = vec![true; vehicle.landing_legs.len()];
        let parachute_states = vec![ParachuteState::default(); vehicle.parachutes.len()];
        let last_parachute_loads = vec![ParachuteLoad::default(); vehicle.parachutes.len()];
        let recipe_max_elevation_m: f64 = {
            let recipe: thessa_worldgen_rocky::spec_recipe::SpecRecipe =
                toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
                    .map_err(|error| format!("world recipe is invalid: {error}"))?;
            recipe.planet.height_max_m
        };
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
        let reference_area_m2: f64 = vehicle
            .aero_geometry
            .panels
            .iter()
            .map(|panel| panel.area_m2)
            .sum();
        let aero_panels = PanelSoA::from_geometry(&vehicle.aero_geometry)
            .map_err(|error| format!("X-15 SoA aero geometry is invalid: {error}"))?;

        Ok(Self {
            reference_body,
            planet_radius_m: body.radius_m,
            body_rotation_period_s: rotation_period_s,
            terrain_field: None,
            recipe_max_elevation_m,
            launch_site_dir: None,
            vehicle,
            resource_state,
            electrical_power_state,
            electrical_power_command,
            electrical_power_telemetry: None,
            auxiliary_power_unit_states,
            auxiliary_power_unit_commands,
            jet_commands,
            electric_thruster_requested_flow_kg_s,
            fusion_torch_commands,
            pulsed_fusion_states,
            pulsed_fusion_commands,
            propeller_drive_commands,
            turboprop_commands,
            aero_model,
            aero_panels,
            aero_scratch: AeroSimdScratch::default(),
            control_reference_geometry,
            control_deflections_rad,
            atmosphere,
            state,
            sas_target_orientation: orientation_body_to_inertial,
            relative_position_m: initial_relative_position,
            flight_time_s: 0.0,
            world_tick: thessa_sim_core::WorldTick::default(),
            throttle: 0.0,
            engine_active: false,
            sas_enabled: true,
            rcs_enabled: true,
            reaction_wheels_enabled: true,
            reaction_wheel_torque_body_nm: DVec3::ZERO,
            gear_down: true,
            parachutes_armed: false,
            wheel_brake_command: 0.0,
            wheel_drive_command: 0.0,
            wheel_spin_rad_s,
            wheel_brake_states,
            wheel_chassis_states,
            landing_leg_states,
            reaction_wheel_bank_enabled,
            wheel_chassis_deployment_commands,
            landing_leg_deployment_commands,
            parachute_states,
            last_parachute_loads,
            last_wheel_contacts: Vec::new(),
            last_wheel_drive_points: Vec::new(),
            last_wheel_gear_actuators: Vec::new(),
            last_landing_leg_contacts: Vec::new(),
            last_landing_leg_actuators: Vec::new(),
            control_input: DVec3::ZERO,
            surface_input: DVec3::ZERO,
            actuator_saturated: false,
            regime: FlightRegime::Aero,
            accumulator_s: 0.0,
            steps_this_frame: 0,
            rails_advanced_this_frame: 0.0,
            work_budget_exhausted: false,
            waiting_for_rails_bake: false,
            flight_error: None,
            last_gravity_acceleration_inertial_mps2: DVec3::ZERO,
            last_forces: None,
            trace: None,
            rails: OnRailsCache::new(),
            bake: Box::new(InlineBakeQueue::new()),
            rails_bake_seconds: None,
            contact: None,
            contact_patch: None,
            contact_patch_anchor: None,
            scheduler: EventScheduler::new(),
            wake_notice: None,
            wake_events: Vec::new(),
            force_trim_two_pass: false,
            trim_solves: 0,
            trim_second_passes: 0,
            ephemeris_frame: EphemerisFrame::new(),
            reference_area_m2,
            explicit_force_demand_body_n: None,
            explicit_moment_demand_nm: None,
            guidance_state_dependent: false,
            propulsion_actual: 0.0,
            propulsion_target: 0.0,
            propulsion_dynamics_active: false,
            propulsion_dynamics: ActuatorDynamics {
                response_s: 0.12,
                max_rate_per_s: Some(8.0),
                min_command: 0.0,
                max_command: 1.0,
            },
            engine_throttle_overrides,
            system_throttle_overrides,
            last_propulsion_force_body_n: DVec3::ZERO,
            last_propellant_flow_kg_s: 0.0,
            fuel_limited: false,
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

    /// Actual post-actuator control-surface angles in vehicle definition
    /// order. `surface_input` remains the normalized requested command.
    pub fn control_deflections_rad(&self) -> &[f64] {
        &self.control_deflections_rad
    }

    pub fn set_wheel_brake_command(&mut self, command: f64) -> Result<(), FlightError> {
        if !command.is_finite() || !(0.0..=1.0).contains(&command) {
            return Err(FlightError::InvalidInput(
                "wheel brake command must be finite and in [0, 1]".into(),
            ));
        }
        self.wheel_brake_command = command;
        Ok(())
    }

    pub fn set_wheel_drive_command(&mut self, command: f64) -> Result<(), FlightError> {
        if !command.is_finite() || !(-1.0..=1.0).contains(&command) {
            return Err(FlightError::InvalidInput(
                "wheel drive command must be finite and in [-1, 1]".into(),
            ));
        }
        self.wheel_drive_command = command;
        Ok(())
    }

    /// Set the requested landing-gear configuration. Fold-out legs and
    /// retractable wheel chassis advance toward this command on physics ticks.
    pub fn set_gear_down(&mut self, deployed: bool) {
        self.gear_down = deployed;
        self.sync_gear_deployment_commands();
        self.wheel_chassis_deployment_commands.fill(deployed);
        self.landing_leg_deployment_commands.fill(deployed);
    }

    /// Enable or disable one authored reaction-wheel bank without changing
    /// the state of other installed banks.
    pub fn set_reaction_wheel_bank_enabled(
        &mut self,
        name: &str,
        enabled: bool,
    ) -> Result<(), FlightError> {
        self.sync_reaction_wheel_runtime_state();
        let Some(index) = self
            .vehicle
            .reaction_wheels
            .iter()
            .position(|bank| bank.name == name)
        else {
            return Err(FlightError::InvalidInput(format!(
                "vehicle has no reaction-wheel bank named '{name}'"
            )));
        };
        self.reaction_wheel_bank_enabled[index] = enabled;
        Ok(())
    }

    /// Set one authored fold-out landing leg's deployment target.
    pub fn set_landing_leg_deployed(
        &mut self,
        name: &str,
        deployed: bool,
    ) -> Result<(), FlightError> {
        self.sync_gear_deployment_commands();
        let Some(index) = self
            .vehicle
            .landing_legs
            .iter()
            .position(|leg| leg.spec.name == name)
        else {
            return Err(FlightError::InvalidInput(format!(
                "vehicle has no landing leg named '{name}'"
            )));
        };
        self.landing_leg_deployment_commands[index] = deployed;
        Ok(())
    }

    /// Set one authored retractable wheel chassis' deployment target.
    pub fn set_wheel_chassis_deployed(
        &mut self,
        name: &str,
        deployed: bool,
    ) -> Result<(), FlightError> {
        self.sync_gear_deployment_commands();
        let Some(index) = self
            .vehicle
            .wheel_chassis
            .iter()
            .position(|chassis| chassis.spec.name == name)
        else {
            return Err(FlightError::InvalidInput(format!(
                "vehicle has no wheel chassis named '{name}'"
            )));
        };
        if self.vehicle.wheel_chassis[index].spec.retraction.is_none() {
            return Err(FlightError::InvalidInput(format!(
                "wheel chassis '{name}' has no deployment actuator"
            )));
        }
        self.wheel_chassis_deployment_commands[index] = deployed;
        Ok(())
    }

    pub fn reaction_wheel_telemetry(&self) -> DVec3 {
        self.reaction_wheel_torque_body_nm
    }

    pub fn parachute_telemetry(&self) -> &[ParachuteLoad] {
        &self.last_parachute_loads
    }

    /// Arm or disarm every installed pack. This convenience API backs the
    /// current pilot toggle; future staging/action-group dispatch can target
    /// individual packs with [`Self::command_parachute`].
    pub fn set_parachutes_armed(&mut self, armed: bool) {
        self.sync_parachute_runtime_state();
        self.parachutes_armed = armed;
        let command = if armed {
            ParachuteCommand::Arm
        } else {
            ParachuteCommand::Disarm
        };
        for (spec, state) in self
            .vehicle
            .parachutes
            .iter()
            .zip(&mut self.parachute_states)
        {
            spec.apply_command(state, command);
        }
    }

    /// Apply a part-level parachute command by the stable authored component
    /// name. Stage or action-group systems can route their future bindings
    /// through this API without duplicating canopy state transitions.
    pub fn command_parachute(
        &mut self,
        name: &str,
        command: ParachuteCommand,
    ) -> Result<(), FlightError> {
        self.sync_parachute_runtime_state();
        let Some(index) = self
            .vehicle
            .parachutes
            .iter()
            .position(|parachute| parachute.name == name)
        else {
            return Err(FlightError::InvalidInput(format!(
                "vehicle has no parachute named '{name}'"
            )));
        };
        self.vehicle.parachutes[index].apply_command(&mut self.parachute_states[index], command);
        Ok(())
    }

    /// Apply one serialized installed-subsystem command. The same method is
    /// the server endpoint for pilot commands and the future stage/action-group
    /// dispatcher, keeping those producers out of the physical state machines.
    pub fn apply_part_command(&mut self, command: &VehiclePartCommand) -> Result<(), FlightError> {
        match command {
            VehiclePartCommand::SetRcsEnabled { enabled } => {
                self.rcs_enabled = *enabled;
            }
            VehiclePartCommand::SetReactionWheelsEnabled { enabled } => {
                self.reaction_wheels_enabled = *enabled;
            }
            VehiclePartCommand::SetReactionWheelBankEnabled { name, enabled } => {
                self.set_reaction_wheel_bank_enabled(name, *enabled)?;
            }
            VehiclePartCommand::SetLandingGearDeployed { deployed } => {
                self.set_gear_down(*deployed);
            }
            VehiclePartCommand::SetWheelChassisDeployed { name, deployed } => {
                self.set_wheel_chassis_deployed(name, *deployed)?;
            }
            VehiclePartCommand::SetLandingLegDeployed { name, deployed } => {
                self.set_landing_leg_deployed(name, *deployed)?;
            }
            VehiclePartCommand::SetParachutesArmed { armed } => {
                self.set_parachutes_armed(*armed);
            }
            VehiclePartCommand::Parachute { name, command } => {
                self.command_parachute(name, *command)?;
            }
            VehiclePartCommand::SetEngineThrottle { name, throttle } => {
                self.set_engine_throttle(name, *throttle)?;
            }
            VehiclePartCommand::TransferPropellant {
                source_tank,
                destination_tank,
                mass_kg,
            } => {
                self.transfer_propellant(source_tank, destination_tank, *mass_kg)?;
            }
        }
        Ok(())
    }

    /// Live propellant mass for a named tank.
    pub fn tank_propellant_kg(&self, tank_name: &str) -> Result<f64, FlightError> {
        self.vehicle
            .tank_propellant_kg(&self.resource_state, tank_name)
            .map_err(|error| FlightError::InvalidInput(error.to_string()))
    }

    /// Manually transfer propellant and immediately update vehicle mass,
    /// inertia, body-frame geometry, and the center-of-mass state.
    pub fn transfer_propellant(
        &mut self,
        source_name: &str,
        destination_name: &str,
        mass_kg: f64,
    ) -> Result<(), FlightError> {
        let frame_shift = self
            .vehicle
            .transfer_propellant(
                &mut self.resource_state,
                source_name,
                destination_name,
                mass_kg,
            )
            .map_err(|error| FlightError::InvalidInput(error.to_string()))?;
        let mut state = self.state;
        self.apply_resource_frame_shift(frame_shift, &mut state)?;
        self.state = state;
        self.rails.invalidate();
        self.scheduler.clear_rails_wakes();
        Ok(())
    }

    fn parachute_wrench(&self) -> (DVec3, DVec3) {
        self.last_parachute_loads
            .iter()
            .fold((DVec3::ZERO, DVec3::ZERO), |(force, moment), load| {
                (force + load.force_body_n, moment + load.moment_body_nm)
            })
    }

    /// Adopt the server's latest per-canopy state into an embedded client.
    /// A mismatched or malformed vector is ignored rather than resizing a
    /// vehicle from untrusted snapshot data.
    pub fn adopt_parachute_telemetry(&mut self, loads: &[ParachuteLoad]) {
        if loads.len() != self.vehicle.parachutes.len()
            || loads.iter().any(|load| {
                !load.deployment_fraction.is_finite()
                    || !(0.0..=1.0).contains(&load.deployment_fraction)
                    || !load.dynamic_pressure_pa.is_finite()
                    || load.dynamic_pressure_pa < 0.0
                    || !load.force_body_n.is_finite()
                    || !load.moment_body_nm.is_finite()
                    || !load.state.inflation_elapsed_s.is_finite()
                    || load.state.inflation_elapsed_s < 0.0
            })
        {
            return;
        }
        self.last_parachute_loads.copy_from_slice(loads);
        for (state, load) in self.parachute_states.iter_mut().zip(loads) {
            *state = load.state;
        }
    }

    pub fn wheel_spin_rates_rad_s(&self) -> &[Vec<f64>] {
        &self.wheel_spin_rad_s
    }

    pub fn wheel_brake_states(&self) -> &[Vec<WheelBrakeState>] {
        &self.wheel_brake_states
    }

    pub fn wheel_chassis_states(&self) -> &[WheelChassisState] {
        &self.wheel_chassis_states
    }

    pub fn wheel_contact_telemetry(&self) -> &[WheelContactSample] {
        &self.last_wheel_contacts
    }

    pub fn wheel_drive_telemetry(&self) -> &[(usize, u16, WheelDrivePoint)] {
        &self.last_wheel_drive_points
    }

    pub fn wheel_gear_actuator_telemetry(&self) -> &[(usize, WheelChassisActuatorPoint)] {
        &self.last_wheel_gear_actuators
    }

    pub fn landing_leg_states(&self) -> &[LandingLegState] {
        &self.landing_leg_states
    }

    pub fn landing_leg_contact_telemetry(&self) -> &[LandingLegContactSample] {
        &self.last_landing_leg_contacts
    }

    pub fn landing_leg_actuator_telemetry(&self) -> &[(usize, LandingGearActuatorPoint)] {
        &self.last_landing_leg_actuators
    }

    /// Arm the contact-active solver. Distances are the explicit hysteresis
    /// boundary (enter earlier than exit): the craft integrates through
    /// Rapier at or below `enter_distance_m` of terrain evidence and returns
    /// to free flight past `exit_distance_m`. Arming invalidates rails so no
    /// coast batch can span the regime change. The collision frame is
    /// anchored at the current craft pose/velocity.
    pub fn enable_contact_mode(
        &mut self,
        enter_distance_m: f64,
        exit_distance_m: f64,
    ) -> Result<(), FlightError> {
        let activation = ContactActivation::new(enter_distance_m, exit_distance_m)?;
        // Fixed origin: kinematic prescriptions and body resyncs are
        // converted with the pre-step origin but take effect in the post-step
        // one, so a translating origin would stale every prescription by
        // origin_velocity * dt (one tick of orbital motion here). The frame
        // stays inertial either way; f64 needs no follow-frame.
        let frame =
            CollisionFrame::new(self.state.position_inertial_m, DVec3::ZERO, DQuat::IDENTITY)
                .map_err(|error| FlightError::InvalidInput(format!("contact frame: {error}")))?;
        self.contact = Some(ContactRuntime::new(frame, activation)?);
        self.contact_patch = None;
        self.contact_patch_anchor = None;
        self.last_wheel_contacts.clear();
        self.last_wheel_drive_points.clear();
        self.last_wheel_gear_actuators.clear();
        self.last_landing_leg_contacts.clear();
        self.last_landing_leg_actuators.clear();
        self.rails.invalidate();
        self.scheduler.clear_rails_wakes();
        Ok(())
    }

    /// Disarm the contact solver and drop its transient scene. Rails are
    /// invalidated so free flight never resumes on a forecast that spanned
    /// contact-active ticks.
    pub fn disable_contact_mode(&mut self) {
        self.contact = None;
        self.contact_patch = None;
        self.contact_patch_anchor = None;
        self.last_wheel_contacts.clear();
        self.last_wheel_drive_points.clear();
        self.last_wheel_gear_actuators.clear();
        self.last_landing_leg_contacts.clear();
        self.last_landing_leg_actuators.clear();
        self.rails.invalidate();
        self.scheduler.clear_rails_wakes();
    }

    /// Whether the current tick must integrate through the contact solver.
    pub fn contact_active(&self) -> bool {
        self.contact.as_ref().is_some_and(ContactRuntime::is_active)
    }

    /// Debug telemetry for the contact scene. Errors when contact mode is
    /// not enabled.
    pub fn contact_snapshot(&self) -> Result<CollisionDebugSnapshot, FlightError> {
        self.contact
            .as_ref()
            .ok_or_else(|| FlightError::InvalidInput("contact mode is not enabled".into()))?
            .debug_snapshot()
    }

    /// Take this tick's contact load evidence for the damage/telemetry
    /// boundary. Returns an empty vector when contact mode is not enabled.
    pub fn drain_contact_events(&mut self) -> Vec<ContactSummary> {
        self.contact
            .as_mut()
            .map(ContactRuntime::drain_contact_events)
            .unwrap_or_default()
    }

    fn evaluate_forces(
        &mut self,
        state: RigidBodyState,
        input: FlightStepInput,
    ) -> Result<FlightForces, FlightError> {
        self.aero_panels
            .sync_deflections(&self.vehicle.aero_geometry)
            .map_err(FlightError::Aero)?;
        evaluate_flight_forces_soa(
            &self.aero_model,
            &self.aero_panels,
            &mut self.aero_scratch,
            self.atmosphere,
            state,
            self.vehicle.mass_properties,
            input,
        )
    }

    /// Display-only load evaluation at the adopted snapshot state. It reads
    /// current panel geometry and computes the instantaneous RCS/wheel
    /// residual without advancing surface actuators, changing trim state, or
    /// touching any authority state.
    pub fn display_loads(
        &self,
        ephemeris: &BakedEphemeris,
        time: SimTime,
        mode: ControlMode,
    ) -> Result<(DVec3, FlightForces), FlightError> {
        let body_state = ephemeris
            .body_state(self.reference_body, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let kinematics = local_air_kinematics(
            self.atmosphere,
            self.state,
            body_state,
            self.planet_radius_m,
        )?;
        let density_kg_m3 = self
            .atmosphere
            .sample(kinematics.altitude_m.max(0.0))
            .map(|sample| sample.density_kg_m3)
            .unwrap_or(f64::INFINITY);
        let gravity = thessa_sim_core::GravityField::from_ephemeris(ephemeris)
            .acceleration(self.state.position_inertial_m, time)
            .map_err(|e| FlightError::InvalidInput(e.to_string()))?;
        let vacuum = density_kg_m3 == 0.0;
        let band = self.upper_band_drag(kinematics, density_kg_m3);
        let skip_aero = vacuum || band.is_some();
        let (band_drag_body_n, band_q_pa) = band.unwrap_or((DVec3::ZERO, 0.0));
        let (parachute_force_body_n, parachute_moment_body_nm) = self.parachute_wrench();
        let mut forces = evaluate_flight_forces(
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
                extra_force_body_n: DVec3::X * self.thrust_n()
                    + band_drag_body_n
                    + parachute_force_body_n,
                extra_moment_body_nm: parachute_moment_body_nm,
                skip_aero,
            },
        )?;
        let jet_moment = self.display_control_moment(mode, forces.aero.moment_body_nm)?;
        forces.total_moment_body_nm += jet_moment;
        let angular_momentum_body =
            self.vehicle.mass_properties.inertia_body_kg_m2 * self.state.angular_velocity_body_rps;
        forces.angular_acceleration_body_rps2 =
            self.vehicle.mass_properties.inertia_body_kg_m2.inverse()
                * (forces.total_moment_body_nm
                    - self
                        .state
                        .angular_velocity_body_rps
                        .cross(angular_momentum_body));
        if band_q_pa > 0.0 {
            forces.aero.dynamic_pressure_pa = band_q_pa;
        }
        Ok((gravity, forces))
    }

    fn display_control_moment(
        &self,
        mode: ControlMode,
        actual_aero_moment_nm: DVec3,
    ) -> Result<DVec3, FlightError> {
        let attitude = attitude_demand(
            mode,
            self.sas_enabled,
            self.control_input,
            self.state,
            self.sas_target_orientation,
            self.vehicle.mass_properties.inertia_body_kg_m2,
        );
        let explicit_moment = self.explicit_moment_demand_nm;
        let axes = explicit_moment.map_or(attitude.axes, |_| DVec3::ZERO);
        let assisted = explicit_moment.is_some() || mode != ControlMode::Direct;
        let requested = explicit_moment.unwrap_or(attitude.requested_moment_nm);
        let coast = self.regime == FlightRegime::Coast;
        if self.vehicle.reaction_wheels.is_empty() {
            return Ok(allocate_rcs(
                requested,
                if coast {
                    DVec3::ZERO
                } else {
                    actual_aero_moment_nm
                },
                axes,
                assisted,
                self.rcs_enabled,
            )
            .moment_body_nm);
        }

        let (wheel_request, wheel_aero_moment) = if !assisted && axes == DVec3::ZERO {
            (DVec3::ZERO, DVec3::ZERO)
        } else {
            (
                requested,
                if coast {
                    DVec3::ZERO
                } else {
                    actual_aero_moment_nm
                },
            )
        };
        let residual = wheel_request - wheel_aero_moment;
        let wheel = if self.reaction_wheels_enabled {
            let enabled: Vec<_> = (0..self.vehicle.reaction_wheels.len())
                .map(|index| {
                    self.reaction_wheel_bank_enabled
                        .get(index)
                        .copied()
                        .unwrap_or(self.reaction_wheels_enabled)
                })
                .collect();
            allocate_reaction_wheels_with_enabled_banks(
                &self.vehicle.reaction_wheels,
                &enabled,
                residual,
            )
            .map_err(|error| {
                FlightError::InvalidInput(format!("reaction-wheel allocation failed: {error}"))
            })?
            .delivered_torque_body_nm
        } else {
            DVec3::ZERO
        };
        Ok(wheel + rcs_moment(residual - wheel, self.rcs_enabled))
    }
}

fn x15_vehicle() -> Result<VehicleDefinition, String> {
    X15StarterProfile::new()
        .map(|profile| profile.vehicle)
        .map_err(|error| error.to_string())
}

#[cfg(test)]
#[path = "rails_terrain_bound_tests.rs"]
mod rails_terrain_bound_tests;
#[cfg(test)]
#[path = "runtime_tests.rs"]
mod tests;
#[cfg(test)]
#[path = "trim_adaptive_tests.rs"]
mod trim_adaptive_tests;
