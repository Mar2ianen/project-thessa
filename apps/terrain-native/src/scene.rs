//! A real canonical-field resident partition, shared by window/headless paths.
//! Shared terrain streaming and local/server PRIMARY-vehicle presentation.
use crate::celestial::{CelestialPresentation, Sample};
use glam::{DQuat, DVec3};
use thessa_rcbt_core::tile_anchor::{TileAnchor, TileKey};
use thessa_render_wgpu::{
    Backend, BackendError,
    terrain::{ResidentTerrainPatch, TerrainCapacity, TerrainPatch, TerrainRenderer},
    view::ViewParams,
};
use thessa_terrain_stream::{
    runtime::{StreamUpdate, StreamingPolicy, TerrainStream},
    selection::SelectionView,
};

#[path = "fleet_mesh.rs"]
mod fleet_mesh;
pub use fleet_mesh::SceneMeshes;

pub struct Scene {
    #[cfg(feature = "hud")]
    pub map: crate::map::Source,
    primary_presentation: Option<(u64, thessa_flight_authority::presentation::FlightPose)>,
    last_primary_revision: u64,
    last_primary_frame_revision: u64,
    flight: Option<thessa_flight_authority::FlightAuthority>,
    remote: Option<RemoteFlight>,
    clock: FlightClock,
    launch_site: [f64; 3],
    vehicle: Option<(
        thessa_render_core::mesh::MeshAsset,
        thessa_render_core::mesh::MeshInstance,
    )>,
    /// Owned pilot camera (vehicle/preview view only). The terrain-only
    /// survey view uses its own shared quaternion controller.
    pub camera: thessa_flight_authority::camera::FlightCamera,
    /// Owned live flight-input intent (throttle/engine/stick/precision).
    /// Applied to the authority on every rendered frame; headless runs stay
    /// deterministic because no keys are held there.
    pub input: thessa_flight_authority::input::FlightInput,
    /// Assist mode selecting the server-side control law. Cycled with `C` in
    /// the window; the old client owns the same cycle through its HUD modes.
    pub control_mode: thessa_flight_authority::ControlMode,
    pub requested: thessa_graphics::RequestedGraphics,
    pub stream: TerrainStream,
    pub publications: u64,
    pub regenerated: u64,
    pub material_upload_bytes: u64,
    pub capacity_rejections: u64,
    pub center: DVec3,
    pub up: DVec3,
    pub east: DVec3,
    /// Owned survey camera driving the terrain-only view. The legacy
    /// single-angle orbit is retired: framing matches the shipped survey.
    pub survey: thessa_flight_authority::survey::SurveyCamera,
    /// Survey bookmarks in `SURVEY_SITE_LABELS` order with the active index.
    /// The default site is HIGHLANDS, like the previous survey bookmark.
    pub sites: [([f64; 3], &'static str); 3],
    pub site: usize,
    celestial: CelestialPresentation,
    pub sample: Sample,
    /// Body optics plus the field's bounded refresh policy; `None` for a body
    /// without an atmosphere provider, which has no sky to integrate.
    environment: Option<crate::environment::EnvironmentField>,
    ocean_gravity_m_s2: f64,
    /// One GPU publication used by both background and mesh/RT consumers.
    gpu_environment: Option<thessa_render_wgpu::environment::EnvironmentBinding>,
    gpu_environment_revision: Option<u64>,
    lighting_dirty: bool,
}

/// Native stdio host state, not a second simulation or interpolation inbox.
struct RemoteFlight {
    link: Option<thessa_flight_net::embedded::EmbeddedLink>,
    commands: Vec<thessa_flight_net::Command>,
    started: std::time::Instant,
    timeout: std::time::Duration,
    snapshots: u64,
    last_tick: Option<u64>,
    fleet: FleetObserver,
}

/// Bounded own-time authoritative observations, separate from world instances.
/// Retained appearance is optional; secondary clocks may differ from PRIMARY.
#[derive(Default, Clone)]
pub struct FleetObserver {
    pub rows: Vec<thessa_flight_net::presentation::FleetPose>,
    pub updates: u64,
    pub age_s: Option<f64>,
    pub stale: bool,
    pub geometry_revision: u64,
    pub geometry_updates: u64,
    pub geometry: Option<std::sync::Arc<thessa_flight_net::geometry::FleetGeometry>>,
}

impl FleetObserver {
    pub fn matched_geometry(&self) -> Option<&thessa_flight_net::geometry::FleetGeometry> {
        self.geometry
            .as_deref()
            .filter(|g| g.revision == self.geometry_revision)
    }
}

/// Installed-subsystem selector sharing keys with the old pilot
/// (`R` RCS, `Y` reaction wheels, `G` gear, `P` parachutes). The old client
/// forwards the same commands to the server. Native server mode retains their
/// order; local mode applies them to its own authority only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartToggle {
    Rcs,
    ReactionWheels,
    Gear,
    Parachutes,
}

/// Host intent plus acknowledged/observed clock state. No independently advanced
/// simulation clock: FlightAuthority or its server snapshot owns actual time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightClock {
    pub requested_paused: bool,
    pub requested_warp: f64,
    pub authority_paused: Option<bool>,
    pub effective_warp: Option<f64>,
}
impl Default for FlightClock {
    fn default() -> Self {
        Self {
            requested_paused: false,
            requested_warp: 1.0,
            authority_paused: None,
            effective_warp: None,
        }
    }
}

/// Read-only live flight data for the production-HUD path. Reference body
/// only: unlike the old client's display-body switching near other worlds,
/// the native viewer always reads relative to the flown reference body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightData {
    pub surface_speed_mps: f64,
    pub vertical_speed_mps: f64,
    pub altitude_datum_m: f64,
    pub altitude_agl_m: f64,
    /// Local up in the vehicle body frame (nose +X), driving the navball.
    /// Mirrors the old client's instrument input, not a new attitude model.
    pub local_up_body: DVec3,
    pub pole_body: DVec3,
    pub surface_velocity_body: DVec3,
}

impl Scene {
    pub fn map_active(&self) -> bool {
        #[cfg(feature = "hud")]
        {
            self.map.open
        }
        #[cfg(not(feature = "hud"))]
        {
            false
        }
    }
    #[cfg(feature = "hud")]
    pub fn set_map_open(&mut self, open: bool) {
        self.map.set_open(open, self.celestial.ephemeris());
        // Transition cannot leave stale key/cursor/SAS demands in the authority.
        self.release_manual_input();
    }
    pub fn release_manual_input(&mut self) {
        self.input.keys = Default::default();
        self.input.mouse_direction = glam::Vec3::NEG_Z;
        self.input.sas_inverted = false;
    }
    #[cfg(feature = "hud")]
    pub fn map_snapshot(&self) -> Option<crate::map::Snapshot> {
        let ephemeris = self.celestial.ephemeris();
        let flight = self
            .flight
            .as_ref()
            .filter(|_| !self.remote.as_ref().is_some_and(|r| r.snapshots == 0));
        // The map is a newest-authority instrument, not the interpolated 3D
        // scene. Sample its celestial bodies at that SAME epoch: mixing a
        // newer inertial craft with older planets can invent kilometres of
        // apparent altitude from their common orbital velocity alone.
        let time = flight.map_or(self.sample.time, |f| {
            thessa_sim_core::SimTime(f.flight_time_s)
        });
        let mut snapshot = self.map.sample(ephemeris, time)?;
        if let Some(flight) = flight {
            let time = thessa_sim_core::SimTime(flight.flight_time_s);
            let craft = flight.inertial_state_m().0;
            snapshot.craft = Some((craft, time));
            let display =
                thessa_flight_authority::navigation::craft_display_body(ephemeris, flight, time);
            snapshot.orbit_summary =
                Some(thessa_flight_authority::navigation::craft_orbit_summary(
                    ephemeris,
                    flight,
                    time,
                    self.remote.is_none(),
                ));
            snapshot.trajectory_pending = self.remote.is_none() && flight.bake.has_pending();
            if self.remote.is_none()
                && let Some(until) = flight.jit_coast_execution_end(ephemeris, self.control_mode)
                && let Ok(reference) = ephemeris.body_state(display, time)
            {
                let projected = thessa_flight_authority::navigation::project_jit_coast_path(
                    ephemeris,
                    &flight.rails,
                    display,
                    time,
                    craft,
                    until,
                );
                if !projected.points.is_empty() {
                    snapshot.trajectory = Some((reference.position_inertial, projected));
                    snapshot.execution_until = Some(until);
                }
            }
        }
        Some(snapshot)
    }
    pub fn flight_clock(&self) -> Option<FlightClock> {
        self.live_flight().then(|| {
            let mut clock = self.clock;
            if self
                .remote
                .as_ref()
                .is_some_and(|remote| remote.snapshots == 0)
            {
                clock.authority_paused = None;
                clock.effective_warp = None;
            }
            clock
        })
    }

    pub fn flight_budget(&self, frame_wall_s: f64) -> Option<thessa_perf::SimBudget> {
        let flight = self.flight.as_ref()?;
        let mut budget = thessa_perf::SimBudget {
            requested_warp: if self.clock.requested_paused {
                0.0
            } else {
                self.clock.requested_warp
            },
            effective_warp: self.clock.effective_warp.unwrap_or(0.0),
            ..Default::default()
        };
        if self.server_flight() {
            budget.server_effective_warp = budget.effective_warp;
        } else {
            budget.steps_this_frame = flight.steps_this_frame;
            budget.rails_time_advanced_s = flight.rails_advanced_this_frame;
            budget.sim_time_advanced_s = f64::from(flight.steps_this_frame) * budget.fixed_dt_s
                + flight.rails_advanced_this_frame;
            budget.backlog_s = flight.backlog_s();
            // Captures use actual measured frame wall time, not the scripted
            // --flight-step interval used to exercise deterministic headless input.
            budget.effective_warp = if frame_wall_s > 0.0 {
                budget.sim_time_advanced_s / frame_wall_s
            } else {
                0.0
            };
        }
        Some(budget)
    }

    pub fn request_pause(&mut self, paused: bool) {
        if self.live_flight() {
            self.clock.requested_paused = paused;
        }
    }

    pub fn request_warp(&mut self, factor: f64) -> Result<(), String> {
        if !factor.is_finite() || !(0.0..=thessa_flight_net::MAX_TIME_WARP).contains(&factor) {
            return Err("warp must be finite and within 0..=131072".into());
        }
        if !self.live_flight() {
            return Err("warp requires live flight".into());
        }
        self.clock.requested_warp = factor;
        Ok(())
    }

    /// Reset existing authoritative launch state, preserving time. A remote
    /// request never clears a stopped replica or invents an acknowledged pose.
    pub fn reset_flight(&mut self) -> Result<(), String> {
        if let Some(remote) = &mut self.remote {
            if remote.link.as_ref().is_none_or(|link| link.is_closed()) {
                return Err("server link unavailable; restart the server flight session".into());
            }
            if remote.commands.len() + 2 >= thessa_flight_net::MAX_COMMANDS_PER_INPUT {
                return Err("server command queue full; relaunch was not queued".into());
            }
            remote.commands.push(thessa_flight_net::Command::Reset);
        } else {
            let flight = self.flight.as_mut().ok_or("reset requires live flight")?;
            flight.reset_to_launch_site(self.celestial.ephemeris())?;
            flight.steps_this_frame = 0;
            flight.rails_advanced_this_frame = 0.0;
            let pose = thessa_flight_authority::presentation::FlightPose::from_authority(flight)
                .ok_or("invalid reset flight pose")?;
            self.publish_vehicle_pose(pose)?;
            self.clock.effective_warp = Some(0.0);
        }
        self.input.keys = Default::default();
        self.input.throttle = 0.0;
        self.input.engine_active = true;
        Ok(())
    }
    /// Audio observes the same primary flight state as instruments. The optional
    /// debug cabin is attached to this primary cluster, not a simulated interior.
    pub fn audio_frame(&self) -> Option<crate::audio::Frame> {
        if self.flight_stopped()
            || self
                .remote
                .as_ref()
                .is_some_and(|remote| remote.snapshots == 0 || remote.fleet.stale)
        {
            return None;
        }
        let flight = self.flight.as_ref()?;
        let data = self.flight_data()?;
        let sample = flight.atmosphere.sample(data.altitude_datum_m).ok()?;
        let medium = if sample.density_kg_m3 > 0.0 {
            thessa_audio_core::AcousticMedium::gas(
                sample.density_kg_m3,
                sample.speed_of_sound_mps,
                0.0,
            )
            .ok()?
        } else {
            thessa_audio_core::AcousticMedium::VACUUM
        };
        Some(crate::audio::Frame {
            sim_time_s: flight.flight_time_s,
            engine_active: flight.engine_active,
            throttle: flight.throttle,
            medium,
            source: thessa_audio_core::AcousticPoint::stationary(self.center),
            listener: thessa_audio_core::AcousticPoint::stationary(
                self.view(1, 1, 1.0).world_position_m,
            ),
            // A temporary cabin listener rides PRIMARY, matching the legacy
            // diagnostic. Per-node pressure volumes/structural acoustics remain
            // separate from this explicit listener attachment.
            source_cluster: Some(1),
            listener_cluster: Some(1),
            rcs_commanded: flight.rcs_enabled
                && flight.regime() == thessa_flight_authority::FlightRegime::Coast
                && flight.control_input.length_squared() > 0.02,
            gpws_warning: data.altitude_datum_m < 100.0 && data.vertical_speed_mps < -1.0,
        })
    }
    /// Sample surface speed, vertical speed and datum/AGL altitude from the
    /// existing authority state through the shared kinematics (the same
    /// function the old client's HUD consumes). `None` without live flight
    /// or on a failed ephemeris/atmosphere sample; the HUD omits that readout.
    /// Server telemetry reads the newest replica, not the interpolated pose.
    pub fn flight_data(&self) -> Option<FlightData> {
        if self
            .remote
            .as_ref()
            .is_some_and(|remote| remote.snapshots == 0)
        {
            return None;
        }
        let flight = self.flight.as_ref()?;
        let time = thessa_sim_core::SimTime(flight.flight_time_s);
        let body_state = self
            .celestial
            .ephemeris()
            .body_state(flight.reference_body, time)
            .ok()?;
        let kinematics = thessa_flight_authority::local_air_kinematics(
            flight.atmosphere,
            flight.state,
            body_state,
            flight.planet_radius_m,
        )
        .ok()?;
        let surface_speed_mps = kinematics.surface_velocity_inertial_mps.length();
        let vertical_speed_mps = kinematics
            .surface_velocity_inertial_mps
            .dot(kinematics.radial_up);
        let altitude_datum_m = kinematics.altitude_m.max(0.0);
        // Ground lookup mirrors the old client's AGL readout verbatim
        // (render-axes origin, spin-undo, same height call): the field frame
        // needs the mapped direction, not the raw inertial one.
        let origin = DVec3::new(
            flight.relative_position_m.x,
            flight.relative_position_m.z,
            -flight.relative_position_m.y,
        );
        let body_dir = DQuat::from_rotation_y(-flight.terrain_spin()) * origin.normalize();
        let ground = self
            .stream
            .field()
            .height_m(body_dir.to_array(), 32.0)
            .max(0.0);
        Some(FlightData {
            surface_speed_mps,
            vertical_speed_mps,
            altitude_datum_m,
            altitude_agl_m: (altitude_datum_m - ground).max(0.0),
            local_up_body: flight.state.orientation_body_to_inertial.inverse()
                * kinematics.radial_up,
            pole_body: flight.state.orientation_body_to_inertial.inverse()
                * flight
                    .atmosphere
                    .body_rotation_rad_s
                    .try_normalize()
                    .unwrap_or(DVec3::Z),
            surface_velocity_body: flight.state.orientation_body_to_inertial.inverse()
                * kinematics.surface_velocity_inertial_mps,
        })
    }
    pub fn vehicle_preview(&self) -> bool {
        self.vehicle.is_some()
    }
    pub fn load(requested: thessa_graphics::RequestedGraphics) -> Result<Self, String> {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).map_err(|e| e.to_string())?;
        let ephemeris = system.bake().map_err(|e| e.to_string())?;
        let (field, sites) = thessa_flight_authority::canonical_launch_setup(&ephemeris)?;
        let recipe: thessa_worldgen_rocky::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))
                .map_err(|e| e.to_string())?;
        let body = ephemeris
            .body_id(&recipe.planet.id)
            .ok_or("missing terrain body")?;
        let environment = ephemeris.body(body).ok().and_then(|data| {
            let atmosphere = data.atmosphere.as_ref().and_then(|baked| {
                baked
                    .surface_pressure_pa
                    .map(|pressure| crate::environment::BodyAtmosphere {
                        surface_pressure_pa: pressure,
                        gas_constant_j_kg_k: baked.gas_constant_j_kg_k,
                    })
            });
            crate::environment::EnvironmentField::new(data.radius_m, data.mu, atmosphere)
        });
        let data = ephemeris.body(body).map_err(|e| e.to_string())?;
        let ocean_gravity_m_s2 = data.mu / data.radius_m.powi(2);
        let celestial = CelestialPresentation::new(&system, ephemeris, body)?;
        let sample = celestial.sample(thessa_sim_core::SimTime::EPOCH)?;
        let sites = [
            (sites[0], thessa_flight_authority::SURVEY_SITE_LABELS[0]),
            (sites[1], thessa_flight_authority::SURVEY_SITE_LABELS[1]),
            (sites[2], thessa_flight_authority::SURVEY_SITE_LABELS[2]),
        ];
        let stream = TerrainStream::new(field, StreamingPolicy::default())?;
        let mut scene = Self {
            ocean_gravity_m_s2,
            primary_presentation: None,
            last_primary_revision: 0,
            last_primary_frame_revision: 0,
            flight: None,
            #[cfg(feature = "hud")]
            map: crate::map::Source::default(),
            remote: None,
            launch_site: sites[0].0,
            vehicle: None,
            camera: thessa_flight_authority::camera::FlightCamera::pilot_default(),
            input: thessa_flight_authority::input::FlightInput::default(),
            control_mode: thessa_flight_authority::ControlMode::Navball,
            requested,
            stream,
            publications: 0,
            regenerated: 0,
            material_upload_bytes: 0,
            capacity_rejections: 0,
            center: DVec3::ZERO,
            up: DVec3::Y,
            east: DVec3::X,
            survey: thessa_flight_authority::survey::SurveyCamera::survey_default(),
            sites,
            site: 1,
            celestial,
            environment,
            gpu_environment: None,
            gpu_environment_revision: None,
            sample,
            lighting_dirty: false,
            clock: FlightClock::default(),
        };
        scene.select_site(1);
        eprintln!(
            "canonical Thessa: {} resident tiles; highlands={:?}",
            scene.stream.visible().len(),
            scene.center
        );
        Ok(scene)
    }

    /// Select a survey bookmark by index, restoring its per-site framing
    /// (distance + free look, orbit preserved like the shipped survey).
    /// Out-of-range indices are ignored so HUD buttons stay total.
    pub fn select_site(&mut self, site: usize) {
        if self.vehicle_preview() {
            return;
        }
        let Some(&(dir, _)) = self.sites.get(site) else {
            return;
        };
        self.site = site;
        let up = DVec3::from(dir);
        self.center =
            up * (self.stream.radius_m() + self.stream.field().height_m(dir, 32.0).max(0.0));
        self.up = up;
        self.east = DVec3::Y
            .cross(up)
            .try_normalize()
            .unwrap_or_else(|| up.any_orthonormal_vector());
        self.survey.reset_site();
    }

    fn vehicle_source() -> Result<thessa_render_core::mesh::MeshAsset, String> {
        thessa_render_assets::decode_glb(
            include_bytes!("../../../assets/models/north_american_x-15_plane.glb"),
            Default::default(),
        )
    }

    /// Diagnostic preview of the exact initial flight state. No second flight
    /// integrator or decorative position/attitude animation is introduced.
    pub fn enable_vehicle_preview(&mut self) -> Result<(), String> {
        use thessa_flight_authority::presentation::*;
        if self.vehicle.is_some() {
            return Err("vehicle scene already enabled".into());
        }
        if self.sample.time != thessa_sim_core::SimTime::EPOCH {
            return Err("vehicle preview requires epoch pose".into());
        }
        let source = Self::vehicle_source()?;
        let flight = self.celestial.initial_flight()?;
        let pose = FlightPose::from_authority(&flight).ok_or("invalid initial flight pose")?;
        let instance = Self::mesh_instance(pose, self.sample.frame)?;
        eprintln!(
            "X15 initial flight-state preview: {} primitives, {} vertices, {} triangles; CC BY 4.0 cmoreau; native opaque material path (not complete PBR parity)",
            source.primitives.len(),
            source
                .primitives
                .iter()
                .map(|p| p.vertices.len())
                .sum::<usize>(),
            source
                .primitives
                .iter()
                .map(|p| p.indices.len() / 3)
                .sum::<usize>()
        );
        self.center = instance.position_m;
        self.up = self.center.normalize();
        // The compatibility flight starts exactly at the north pole, where
        // geographic east is undefined. Choose a finite tangent for orbit UI.
        self.east = DVec3::Y
            .cross(self.up)
            .try_normalize()
            .unwrap_or_else(|| self.up.any_orthonormal_vector());
        self.camera = thessa_flight_authority::camera::FlightCamera::pilot_default();
        self.vehicle = Some((source, instance));
        Ok(())
    }

    pub fn live_flight(&self) -> bool {
        self.flight.is_some()
    }

    /// Canonical launch assets/replica, shared by local and server hosts.
    fn setup_flight(&mut self) -> Result<(), String> {
        if self.vehicle.is_some() || self.sample.time != thessa_sim_core::SimTime::EPOCH {
            return Err("live flight requires an empty epoch vehicle scene".into());
        }
        let source = Self::vehicle_source()?;
        let mut flight = self.celestial.initial_flight()?;
        flight.initialize_world_site(
            self.stream.field().clone(),
            self.launch_site,
            self.celestial.ephemeris(),
        );
        let pose = thessa_flight_authority::presentation::FlightPose::from_authority(&flight)
            .ok_or("invalid canonical flight pose")?;
        self.vehicle = Some((source, Self::mesh_instance(pose, self.sample.frame)?));
        if let Err(error) = self.publish_vehicle_pose(pose) {
            self.vehicle = None;
            return Err(error);
        }
        self.camera = thessa_flight_authority::camera::FlightCamera::pilot_default();
        self.input =
            thessa_flight_authority::input::FlightInput::new(flight.throttle, flight.engine_active)
                .unwrap_or_default();
        self.input.sas_enabled = flight.sas_enabled;
        self.control_mode = thessa_flight_authority::ControlMode::Navball;
        self.flight = Some(flight);
        self.clock.authority_paused = Some(self.clock.requested_paused);
        self.clock.effective_warp = Some(0.0);
        Ok(())
    }

    pub fn enable_live_flight(&mut self) -> Result<(), String> {
        self.setup_flight()?;
        // Reuse the server's bounded native worker, not synchronous UI bakes
        // or a second forecast scheduler. Remote replicas never install one.
        self.flight.as_mut().unwrap().bake =
            Box::new(thessa_flight_authority::ThreadBakeQueue::new());
        eprintln!(
            "Live local flight: canonical COAST launch; existing FlightAuthority; no network/complete UI parity"
        );
        Ok(())
    }

    /// Explicit server mode: failures never fall back to a local integrator.
    pub fn enable_server_flight(
        &mut self,
        server: Option<&std::path::Path>,
        timeout: std::time::Duration,
    ) -> Result<(), String> {
        if timeout.is_zero() {
            return Err("server timeout must be nonzero".into());
        }
        if self.vehicle.is_some() || self.sample.time != thessa_sim_core::SimTime::EPOCH {
            return Err("server flight requires an empty epoch vehicle scene".into());
        }
        let link = match server {
            Some(path) => thessa_flight_net::embedded::EmbeddedLink::spawn_server(path, timeout)?,
            None => thessa_flight_net::embedded::EmbeddedLink::spawn(timeout)?,
        };
        self.setup_flight()?;
        self.remote = Some(RemoteFlight {
            link: Some(link),
            commands: Vec::new(),
            started: std::time::Instant::now(),
            timeout,
            snapshots: 0,
            last_tick: None,
            fleet: FleetObserver::default(),
        });
        self.clock.authority_paused = None;
        self.clock.effective_warp = None;
        eprintln!(
            "Server flight: canonical COAST; shared stdio link and receive-time interpolation; no local physics stepping or fallback"
        );
        Ok(())
    }

    pub fn server_flight(&self) -> bool {
        self.remote.is_some()
    }

    #[cfg(feature = "hud")]
    pub fn fleet_observer(&self) -> Option<&FleetObserver> {
        self.remote.as_ref().map(|remote| &remote.fleet)
    }

    fn accept_fleet_snapshot(
        &mut self,
        fleet: &thessa_flight_net::FleetSnapshot,
    ) -> Result<(), String> {
        let reference = self
            .flight
            .as_ref()
            .ok_or("missing flight replica")?
            .reference_body;
        let rows = thessa_flight_net::presentation::fleet_poses(
            fleet,
            self.celestial.ephemeris(),
            reference,
        )?;
        let remote = self.remote.as_mut().ok_or("missing server host")?;
        if fleet.geometry_revision < remote.fleet.geometry_revision {
            return Err("fleet geometry reference moved backwards".into());
        }
        if fleet.geometry_revision == remote.fleet.geometry_revision
            && (fleet.vehicles.len() != remote.fleet.rows.len()
                || !fleet
                    .vehicles
                    .iter()
                    .zip(&remote.fleet.rows)
                    .all(|(wire, old)| wire.vehicle_id == old.vehicle_id))
        {
            return Err("fleet membership changed under the same geometry revision".into());
        }
        if let Some(geometry) = &remote.fleet.geometry {
            geometry.matches_snapshot(fleet)?;
        }
        for row in &rows {
            if let Ok(index) = remote
                .fleet
                .rows
                .binary_search_by_key(&row.vehicle_id, |old| old.vehicle_id)
            {
                let old = remote.fleet.rows[index];
                if row.tick < old.tick || row.pose.time().0 < old.pose.time().0 {
                    return Err("fleet vehicle tick/time moved backwards".into());
                }
            }
        }
        // Commit complete membership only after every row is checked. Do not
        // blend births with another ID, infer despawn from silence or touch
        // PRIMARY intent/replica/presentation.
        remote.fleet.rows = rows;
        remote.fleet.geometry_revision = fleet.geometry_revision;
        remote.fleet.updates += 1;
        Ok(())
    }

    fn accept_fleet_geometry(
        &mut self,
        geometry: std::sync::Arc<thessa_flight_net::geometry::FleetGeometry>,
    ) -> Result<(), String> {
        geometry.validate()?;
        let fleet = &mut self.remote.as_mut().ok_or("missing server host")?.fleet;
        if let Some(old) = &fleet.geometry {
            if geometry.revision < old.revision
                || (geometry.revision == old.revision && geometry != *old)
            {
                return Err("fleet geometry revision reused or moved backwards".into());
            }
            if geometry == *old {
                return Ok(());
            }
        }
        if geometry.revision == fleet.geometry_revision
            && (geometry.vehicles.len() != fleet.rows.len()
                || !geometry
                    .vehicles
                    .iter()
                    .zip(&fleet.rows)
                    .all(|(g, p)| g.vehicle_id == p.vehicle_id))
        {
            return Err("fleet geometry/observation membership mismatch".into());
        }
        fleet.geometry = Some(geometry);
        fleet.geometry_updates += 1;
        Ok(())
    }

    #[cfg(any(feature = "hud", test))]
    pub fn server_snapshot_count(&self) -> Option<u64> {
        self.remote.as_ref().map(|remote| remote.snapshots)
    }

    fn stop_server_flight(&mut self, error: String) {
        if let Some(remote) = &mut self.remote {
            remote.link.take(); // Kill/reap our child, retaining the server-mode guard.
        }
        if let Some(flight) = &mut self.flight {
            flight.flight_error = Some(error.clone());
        }
        eprintln!(
            "Native server flight stopped: {error}; preserving last complete visual sample; no local fallback"
        );
    }

    fn accept_server_snapshot(
        &mut self,
        snapshot: &thessa_flight_net::Snapshot,
    ) -> Result<(), String> {
        let flight = self.flight.as_mut().ok_or("missing flight replica")?;
        let remote = self.remote.as_mut().ok_or("missing server host")?;
        if snapshot.geometry_revision == 0
            || snapshot.geometry_revision < self.last_primary_revision
            || snapshot.appearance_frame_revision == 0
            || snapshot.appearance_frame_revision < self.last_primary_frame_revision
        {
            return Err("PRIMARY appearance revision is zero or moved backwards".into());
        }
        if remote.last_tick.is_some_and(|tick| {
            snapshot.tick < tick || snapshot.flight_time_s < flight.flight_time_s
        }) {
            return Err("server snapshot tick/time moved backwards".into());
        }
        thessa_flight_net::presentation::adopt_snapshot(
            flight,
            self.celestial.ephemeris(),
            self.control_mode,
            snapshot,
        )?;
        remote.last_tick = Some(snapshot.tick);
        self.last_primary_revision = snapshot.geometry_revision;
        self.last_primary_frame_revision = snapshot.appearance_frame_revision;
        remote.snapshots += 1;
        self.clock.authority_paused = Some(snapshot.paused);
        self.clock.effective_warp = Some(snapshot.effective_warp);
        Ok(())
    }

    fn poll_server_flight(&mut self) -> Result<(), String> {
        if let Err(error) = self.poll_server_inner() {
            self.stop_server_flight(error);
        }
        Ok(())
    }

    fn poll_server_inner(&mut self) -> Result<(), String> {
        use thessa_flight_net::{
            Command,
            presentation::{checked_snapshot_pose, input_for_server},
        };
        let remote = self.remote.as_mut().ok_or("missing server host")?;
        let Some(link) = &remote.link else {
            return Ok(());
        };
        let flight = self.flight.as_ref().ok_or("missing flight replica")?;
        let mut commands = vec![
            Command::SetWarp {
                factor: self.clock.requested_warp,
            },
            Command::Pause {
                paused: self.clock.requested_paused,
            },
        ];
        commands.append(&mut remote.commands);
        let input = input_for_server(
            flight,
            self.control_mode,
            self.input.throttle,
            self.input.engine_active,
            commands,
        );
        input.validate()?;
        if !link.send_input(&input) || link.is_closed() {
            return Err("embedded server link lost".into());
        }
        let latest = link.latest_snapshot();
        let display = link.interpolated_snapshot();
        let fleet = link.latest_fleet_snapshot();
        let geometry = link.latest_fleet_geometry();
        let age = link.fleet_snapshot_age();
        remote.fleet.age_s = age.map(|age| age.as_secs_f64());
        remote.fleet.stale = age.is_some_and(|age| age > remote.timeout);
        if link
            .snapshot_age()
            .unwrap_or_else(|| remote.started.elapsed())
            > remote.timeout
        {
            return Err("embedded server snapshot deadline exceeded".into());
        }
        if let Some(geometry) = geometry {
            self.accept_fleet_geometry(geometry)?;
        }
        if let Some(fleet) = fleet {
            self.accept_fleet_snapshot(&fleet)?;
        }
        if let Some(snapshot) = latest {
            self.accept_server_snapshot(&snapshot)?;
        }
        if let Some(snapshot) = display {
            let flight = self.flight.as_ref().ok_or("missing flight replica")?;
            if snapshot.flight_time_s > flight.flight_time_s {
                return Err("presentation sample is ahead of server truth".into());
            }
            let pose = checked_snapshot_pose(
                &snapshot,
                self.celestial.ephemeris(),
                flight.reference_body,
            )?;
            self.publish_vehicle_pose(pose)?;
            self.primary_presentation = Some((snapshot.geometry_revision, pose));
        }
        // A checked authoritative physics stop is recoverable via Reset on
        // this same live link. Only transport/protocol failures close it.
        Ok(())
    }

    fn mesh_instance(
        pose: thessa_flight_authority::presentation::FlightPose,
        frame: thessa_render_core::BodyFrame,
    ) -> Result<thessa_render_core::mesh::MeshInstance, String> {
        use thessa_flight_authority::presentation::*;
        let instance = thessa_render_core::mesh::MeshInstance {
            position_m: frame.render_to_body_point(pose.render_position_m()),
            rotation: (frame.rotation().conjugate()
                * (pose.visual_orientation() * x15_asset_to_craft_rotation())
                    .as_dquat()
                    .normalize())
            .normalize(),
            scale: f64::from(X15_AUTHORED_LENGTH_M / X15_SOURCE_LENGTH_M),
        };
        instance
            .camera_relative_matrix(instance.position_m)
            .ok_or("invalid body-local flight mesh pose")?;
        Ok(instance)
    }

    /// Stage body/light/mesh/camera together. Physics authority is never read
    /// back from these visual values, and a failed sample leaves them intact.
    fn publish_vehicle_pose(
        &mut self,
        pose: thessa_flight_authority::presentation::FlightPose,
    ) -> Result<(), String> {
        let sample = self.celestial.sample(pose.time())?;
        let instance = Self::mesh_instance(pose, sample.frame)?;
        let up = instance
            .position_m
            .try_normalize()
            .ok_or("invalid flight camera center")?;
        let east = DVec3::Y
            .cross(up)
            .try_normalize()
            .unwrap_or_else(|| up.any_orthonormal_vector());
        let vehicle = self.vehicle.as_mut().ok_or("missing vehicle source")?;
        vehicle.1 = instance;
        self.center = instance.position_m;
        self.up = up;
        self.east = east;
        self.lighting_dirty |= self.sample.lighting != sample.lighting;
        self.sample = sample;
        self.primary_presentation = Some((0, pose));
        Ok(())
    }

    pub fn advance_flight(
        &mut self,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), String> {
        if self.server_flight() {
            return self.poll_server_flight();
        }
        let Some(flight) = &mut self.flight else {
            return Ok(());
        };
        if !elapsed_s.is_finite() || !(0.0..=0.1).contains(&elapsed_s) {
            return Err("flight elapsed must be finite and within 0..=0.1s".into());
        }
        self.clock.authority_paused = Some(self.clock.requested_paused);
        self.clock.effective_warp = Some(0.0);
        if flight.flight_error.is_some()
            || elapsed_s == 0.0
            || self.clock.requested_paused
            || self.clock.requested_warp == 0.0
        {
            flight.steps_this_frame = 0;
            flight.rails_advanced_this_frame = 0.0;
            return Ok(());
        }
        let before = flight.flight_time_s;
        if let Err(error) = flight.advance_with_budget(
            self.celestial.ephemeris(),
            self.control_mode,
            elapsed_s * self.clock.requested_warp,
            budget,
        ) {
            flight.engine_active = false;
            flight.control_input = DVec3::ZERO;
            flight.accumulator_s = 0.0;
            flight.flight_error = Some(error.to_string());
            eprintln!("Native flight stopped: {error}; preserving last complete visual sample");
            return Ok(());
        }
        let pose = thessa_flight_authority::presentation::FlightPose::from_authority(flight)
            .ok_or("invalid advanced flight pose")?;
        self.clock.effective_warp = Some((flight.flight_time_s - before) / elapsed_s);
        if pose.time() != self.sample.time {
            self.publish_vehicle_pose(pose)?;
        }
        Ok(())
    }

    pub fn flight_telemetry(&self) -> Option<String> {
        self.flight.as_ref().map(|flight|{
            let source = self.remote.as_ref().map_or_else(|| "source=local".to_string(), |remote| format!("source=server received_snapshots={} server_tick={} link_closed={} local_steps=0 fleet_updates={} fleet_last_known={} fleet_stale={} geometry_updates={} geometry_revision={} geometry_matched={}", remote.snapshots, remote.last_tick.unwrap_or(0), remote.link.as_ref().is_none_or(|link| link.is_closed()), remote.fleet.updates, remote.fleet.rows.len(), remote.fleet.stale, remote.fleet.geometry_updates, remote.fleet.geometry_revision, remote.fleet.matched_geometry().is_some()));
            let source = format!("{source} requested_paused={} requested_warp={} authority_paused={:?} effective_warp={:?} sas_requested={} sas_temporary_inversion={}",self.clock.requested_paused,self.clock.requested_warp,self.clock.authority_paused,self.clock.effective_warp,self.input.sas_enabled,self.input.sas_inverted);
            let data = self.flight_data().unwrap_or(FlightData { surface_speed_mps: f64::NAN, vertical_speed_mps: f64::NAN, altitude_datum_m: f64::NAN, altitude_agl_m: f64::NAN, local_up_body: DVec3::NAN, pole_body:DVec3::NAN,surface_velocity_body:DVec3::NAN });
            format!("{source} primary_appearance={} authority_time_s={} presentation_time_s={} relative_position_m={:?} steps_last_advance={} backlog_s={} stopped={} stop_reason={:?} chase={} throttle={:.3} engine={} control_mode={:?} sas={} rcs={} wheels={} gear={} chutes={} spd={:.1} vspeed={:+.1} alt={:.0} agl={:.0}",self.primary_appearance_status(),flight.flight_time_s,self.sample.time.0,flight.relative_position_m,flight.steps_this_frame,flight.backlog_s(),self.flight_stopped(),flight.flight_error,self.camera.chase(),flight.throttle,flight.engine_active,self.control_mode,flight.sas_enabled,flight.rcs_enabled,flight.reaction_wheels_enabled,flight.gear_down,flight.parachutes_armed,data.surface_speed_mps,data.vertical_speed_mps,data.altitude_datum_m,data.altitude_agl_m)})
    }

    pub fn primary_appearance_status(&self) -> &'static str {
        use thessa_flight_net::geometry::RenderAppearance;
        let Some(remote) = &self.remote else {
            return "local";
        };
        let Some(geometry) = &remote.fleet.geometry else {
            return "unknown";
        };
        let Some(primary) = &geometry.primary else {
            return "unknown";
        };
        if self
            .primary_presentation
            .is_none_or(|(revision, _)| revision != geometry.revision)
        {
            return "revision_mismatch";
        }
        match primary.render {
            RenderAppearance::Absent => "absent",
            RenderAppearance::OmittedBudget => "omitted_budget",
            RenderAppearance::Available(_) => "retained",
            RenderAppearance::StarterX15 { .. } => "explicit_starter_x15",
        }
    }
    pub fn flight_stopped(&self) -> bool {
        self.flight
            .as_ref()
            .is_some_and(|flight| flight.flight_error.is_some())
    }

    /// Push the owned input intent into the authority: throttle slew plus the
    /// throttle/engine/stick copy. Runs on every rendered frame in both window
    /// and headless modes; with no held keys and default presets the authority
    /// sees exactly its previous demand, keeping headless runs deterministic.
    /// Inputs stay live even after a flight error, like the old client; the
    /// advance itself still stops on error.
    pub fn push_live_input(&mut self, dt_s: f64, throttle_up: bool, throttle_down: bool) {
        let axes = self.control_axes();
        let Some(flight) = &mut self.flight else {
            return;
        };
        self.input.slew(dt_s, throttle_up, throttle_down);
        if self.remote.is_none() {
            flight.throttle = self.input.throttle;
            flight.engine_active = self.input.engine_active;
        }
        flight.control_input = axes;
        flight.sas_enabled = self.input.effective_sas();
    }

    pub fn control_axes(&self) -> DVec3 {
        if self.map_active() || self.clock.requested_paused || self.clock.requested_warp == 0.0 {
            DVec3::ZERO
        } else {
            self.input
                .command_scaled(self.control_mode == thessa_flight_authority::ControlMode::MouseAim)
        }
    }

    /// Cycle the assist control law (`Navball -> Rate -> Direct -> MouseAim`).
    /// Window `C` key and HUD button share this; the old client owns the same
    /// cycle through its mode HUD.
    pub fn cycle_control_mode(&mut self) {
        self.control_mode = self.control_mode.next();
    }

    pub fn toggle_sas(&mut self) {
        self.input.sas_enabled = !self.input.sas_enabled;
        if let Some(flight) = &mut self.flight {
            flight.sas_enabled = self.input.effective_sas();
        }
    }

    #[cfg(feature = "hud")]
    pub fn sas_enabled(&self) -> bool {
        self.live_flight() && self.input.sas_enabled
    }

    #[cfg(feature = "hud")]
    pub fn set_sas(&mut self, enabled: bool) {
        self.input.sas_enabled = enabled;
        if let Some(flight) = &mut self.flight {
            flight.sas_enabled = self.input.effective_sas();
        }
    }

    /// Live subsystem states `(rcs, wheels, gear_down, chutes_armed)`.
    /// `None` without live flight; the starter states are RCS/wheels on,
    /// gear down, chutes safe.
    pub fn part_states(&self) -> Option<(bool, bool, bool, bool)> {
        self.flight.as_ref().map(|flight| {
            (
                flight.rcs_enabled,
                flight.reaction_wheels_enabled,
                flight.gear_down,
                flight.parachutes_armed,
            )
        })
    }

    /// Flip one subsystem through the same explicit command the old pilot
    /// sends. Built-in commands are infallible for the starter vehicle; an
    /// unexpected rejection stops the flight like a physics error instead
    /// of panicking, preserving the last valid presentation.
    pub fn apply_part_toggle(&mut self, toggle: PartToggle) {
        let Some((rcs, wheels, gear, chutes)) = self.part_states() else {
            return;
        };
        let (toggle, enabled) = match toggle {
            PartToggle::Rcs => (toggle, !rcs),
            PartToggle::ReactionWheels => (toggle, !wheels),
            PartToggle::Gear => (toggle, !gear),
            PartToggle::Parachutes => (toggle, !chutes),
        };
        self.set_part_state(toggle, enabled);
    }

    /// Absolute subsystem demand for HUD checkboxes.
    pub fn set_part_state(&mut self, toggle: PartToggle, enabled: bool) {
        use thessa_sim_core::VehiclePartCommand;
        let command = match toggle {
            PartToggle::Rcs => VehiclePartCommand::SetRcsEnabled { enabled },
            PartToggle::ReactionWheels => VehiclePartCommand::SetReactionWheelsEnabled { enabled },
            PartToggle::Gear => VehiclePartCommand::SetLandingGearDeployed { deployed: enabled },
            PartToggle::Parachutes => VehiclePartCommand::SetParachutesArmed { armed: enabled },
        };
        let Some(flight) = &mut self.flight else {
            return;
        };
        if let Err(error) = flight.apply_part_command(&command) {
            flight.engine_active = false;
            flight.control_input = DVec3::ZERO;
            flight.accumulator_s = 0.0;
            flight.flight_error = Some(error.to_string());
            eprintln!("Native flight stopped: {error}; preserving last complete visual sample");
            return;
        }
        if let Some(remote) = &mut self.remote
            && remote.link.is_some()
        {
            // Reserve the two warp/pause votes emitted by poll_server_inner.
            if remote.commands.len() + 2 >= thessa_flight_net::MAX_COMMANDS_PER_INPUT {
                self.stop_server_flight(
                    "server command queue overflow; restart the session".into(),
                );
            } else {
                remote
                    .commands
                    .push(thessa_flight_net::Command::Part { command });
            }
        }
    }

    #[cfg(test)]
    pub fn mesh_renderer(
        &self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<Option<SceneMeshes>, BackendError> {
        self.vehicle
            .as_ref()
            .map(|(source, _)| SceneMeshes::new(self, backend, source, format))
            .transpose()
    }
    pub fn mesh_renderer_with_shadows(
        &self,
        backend: &Backend,
        format: wgpu::TextureFormat,
        terrain: &TerrainRenderer,
    ) -> Result<Option<SceneMeshes>, BackendError> {
        self.vehicle
            .as_ref()
            .map(|(source, _)| {
                SceneMeshes::with_shadows(
                    self,
                    backend,
                    source,
                    format,
                    terrain.shadows(),
                    terrain.ray_source(),
                )
            })
            .transpose()
    }
    pub fn update_mesh(
        &self,
        backend: &Backend,
        renderer: &mut SceneMeshes,
        width: u32,
        height: u32,
        exposure: f32,
    ) -> Result<(), BackendError> {
        renderer.update(self, backend, width, height, exposure)
    }

    /// Explicit presentation sampling, separate from wall-time streaming
    /// debounce. An unchanged SimTime performs no ephemeris or lighting work.
    /// Stage the full sample before mutation so failure preserves the old frame.
    pub fn set_presentation_time(&mut self, time: thessa_sim_core::SimTime) -> Result<(), String> {
        if self.live_flight() {
            return Err("live presentation time is owned by FlightAuthority".into());
        }
        if self.vehicle.is_some() && time != thessa_sim_core::SimTime::EPOCH {
            return Err(
                "vehicle preview is an initial flight-state snapshot, not sampled celestial motion"
                    .into(),
            );
        }
        if time == self.sample.time {
            return Ok(());
        }
        let sample = self.celestial.sample(time)?;
        self.lighting_dirty |= self.sample.lighting != sample.lighting;
        self.sample = sample;
        Ok(())
    }

    /// Evaluate the field using the budget resolved at renderer creation from
    /// the negotiated device. The field
    /// writes the ambient it integrates back into the published lighting, so
    /// indirect diffuse and reflective terms read one physical source.
    fn update_environment(&mut self, camera_body_m: DVec3) {
        if let Some(field) = &mut self.environment {
            let revision = field.revision();
            field.update_at(
                camera_body_m,
                &self.sample.environment_lights,
                self.sample.time.0,
            );
            self.lighting_dirty |= revision != field.revision();
        }
    }

    /// Compose current direct lighting and the cached sky ambient at the read
    /// boundary. No second cache of direct light that can lag an adopted sample.
    pub fn lighting(&self) -> thessa_rcbt_core::lighting::SurfaceLighting {
        let mut lighting = self.sample.lighting;
        let ambient = self
            .environment_grid()
            .map(thessa_render_core::environment::EnvironmentRadiance::ambient_lux);
        if let Some(ambient) = ambient {
            lighting.ambient_lux = [ambient[0], ambient[1], ambient[2], 0.0];
        }
        lighting
    }

    /// Environment accounting for telemetry; `None` when the scene has no
    /// optics for its body at all (an airless body).
    pub fn environment_stats(&self) -> Option<crate::environment::EnvironmentStats> {
        Some(self.environment.as_ref()?.stats())
    }

    /// Bound environment grid for renderers, if any.
    pub fn environment_grid(
        &self,
    ) -> Option<&thessa_render_core::environment::EnvironmentRadiance> {
        self.environment.as_ref()?.grid()
    }

    pub fn environment_revision(&self) -> u64 {
        self.environment.as_ref().map_or(0, |f| f.revision())
    }

    pub fn sky_layers(
        &mut self,
        backend: &Backend,
        format: wgpu::TextureFormat,
        renderer: &mut TerrainRenderer,
    ) -> crate::sky_layers::SkyLayers {
        let mut settings = backend.resolve_graphics(&self.requested);
        if self
            .environment
            .as_ref()
            .and_then(|field| field.aurora_input())
            .is_none()
        {
            settings.aurora_shell = false;
        }
        let layers =
            crate::sky_layers::SkyLayers::new(backend, &settings, format, self.stream_radius());
        if let Some(clouds) = layers.cloud_binding() {
            let binding = self.gpu_environment.get_or_insert_with(|| {
                thessa_render_wgpu::environment::EnvironmentBinding::new(&backend.device)
            });
            binding.set_clouds(clouds);
            renderer.share_environment(backend, Some(binding));
        }
        layers
    }

    pub fn update_sky_layers(
        &self,
        backend: &Backend,
        layers: &crate::sky_layers::SkyLayers,
        width: u32,
        height: u32,
        exposure: f32,
    ) -> Result<(), BackendError> {
        if !layers.enabled() {
            return Ok(());
        }
        let optics = self
            .environment
            .as_ref()
            .and_then(|field| field.star_optics());
        layers.update(
            backend,
            &self.view(width, height, exposure),
            [width, height],
            self.sample.time.0,
            self.sample.frame.rotation().inverse(),
            &self.lighting(),
            optics.as_ref(),
            &self.sample.environment_lights,
        )?;
        let emission = self
            .environment
            .as_ref()
            .and_then(|field| field.aurora_input());
        layers.update_aurora(backend, emission.as_ref(), self.sample.time.0)
    }

    fn publish_presentation(
        &mut self,
        backend: &Backend,
        renderer: &mut TerrainRenderer,
    ) -> Result<(), BackendError> {
        let revision = self.environment_revision();
        if self.gpu_environment_revision != Some(revision) {
            if self.environment_grid().is_some() || self.gpu_environment.is_some() {
                let mut binding = self.gpu_environment.take().unwrap_or_else(|| {
                    thessa_render_wgpu::environment::EnvironmentBinding::new(&backend.device)
                });
                binding.update(backend, self.environment_grid());
                binding.update_background(
                    backend,
                    self.environment
                        .as_ref()
                        .and_then(|field| field.background_grid()),
                );
                renderer.share_environment(backend, Some(&binding));
                self.gpu_environment = Some(binding);
            }
            self.gpu_environment_revision = Some(revision);
        }
        if self.lighting_dirty {
            renderer.update_lighting(backend, &self.lighting())?;
            self.lighting_dirty = false;
        }
        Ok(())
    }

    pub fn renderer(
        &mut self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<TerrainRenderer, BackendError> {
        let resolved = backend.resolve_graphics(&self.requested);
        if let Some(field) = &mut self.environment {
            field.set_budget(crate::environment::EnvironmentBudget::resolve(&resolved));
        }
        let patches: Vec<_> = self
            .stream
            .visible()
            .iter()
            .map(|&key| TerrainPatch {
                anchor: self.anchor(key),
                page: self.stream.height(key).unwrap().page,
            })
            .collect();
        let ocean = &resolved.ocean;
        let sea = thessa_render_core::ocean::SeaState {
            gravity_m_s2: self.ocean_gravity_m_s2,
            wind_speed_m_s: ocean.wind_speed_m_s,
            wind_direction_rad: ocean.wind_direction_rad,
            significant_height_m: ocean.significant_height_m,
            depth_m: ocean.depth_m,
            seed: ocean.seed,
        };
        let mut renderer = TerrainRenderer::with_capacity_lighting_and_ocean(
            backend,
            &patches,
            format,
            TerrainCapacity {
                patches: 1024,
                height_grid_size: 33,
                material_layers: backend.device.limits().max_texture_array_layers.min(256),
            },
            Some(&resolved),
            Some(sea),
        )?;
        let stats = renderer.update_materials(
            backend,
            self.stream.material_demand(),
            self.stream.materials(),
        )?;
        renderer.update_lighting(backend, &self.lighting())?;
        // A replacement renderer must adopt the current buffers even if the
        // source revision has not changed. No second upload.
        self.publish_presentation(backend, &mut renderer)?;
        renderer.share_environment(backend, self.gpu_environment.as_ref());
        self.lighting_dirty = false;
        eprintln!(
            "material publication: {} pages, {} bytes",
            stats.pages, stats.bytes
        );
        Ok(renderer)
    }

    fn anchor(&self, key: thessa_worldgen_rocky::lod::TileKey) -> TileAnchor {
        TileAnchor::new(
            TileKey::new(key.face, key.level, key.x, key.y).unwrap(),
            self.stream_radius(),
        )
        .unwrap()
    }

    fn stream_radius(&self) -> f64 {
        self.stream.radius_m()
    }

    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        backend: &Backend,
        renderer: &mut TerrainRenderer,
        width: u32,
        height: u32,
        exposure: f32,
        now_s: f64,
    ) -> Result<StreamUpdate, Box<dyn std::error::Error>> {
        let camera = self.view(width, height, exposure);
        self.update_environment(camera.world_position_m);
        self.publish_presentation(backend, renderer)?;
        let forward = camera
            .world_from_view
            .transform_vector3(DVec3::NEG_Z)
            .normalize();
        let half_angle = 0.45f64.max((0.45f64.tan() * width as f64 / height as f64).atan()) + 0.6;
        let update = self.stream.tick(
            SelectionView {
                eye: camera.world_position_m,
                stream_origin: self.center,
                forward,
                radius: self.stream_radius(),
                frustum: Some(thessa_worldgen_rocky::lod::SelectionFrustum {
                    forward: forward.to_array(),
                    cos_limit: half_angle.cos().clamp(-1.0, 1.0),
                }),
            },
            2.0 * 0.45f64.tan() / height as f64,
            now_s,
        )?;
        if update.height_failed > 0 {
            eprintln!(
                "[terrain] {} height worker(s) failed; retrying demand",
                update.height_failed
            );
        }
        if update.material_failed > 0 {
            eprintln!(
                "[terrain] {} material worker(s) disconnected; retrying demand",
                update.material_failed
            );
        }
        if update.geometry_changed {
            let patches: Vec<_> = self
                .stream
                .visible()
                .iter()
                .map(|&key| {
                    let entry = self.stream.height(key).unwrap();
                    ResidentTerrainPatch {
                        anchor: self.anchor(key),
                        page: entry.page,
                        generation: entry.generation,
                    }
                })
                .collect();
            let stats = renderer.publish_partition(backend, &patches)?;
            self.publications += 1;
            self.regenerated += stats.regenerated as u64;
        }
        if update.materials_changed || update.geometry_changed {
            let stats = renderer.update_materials(
                backend,
                self.stream.material_demand(),
                self.stream.materials(),
            )?;
            self.material_upload_bytes += stats.bytes as u64;
        }
        self.capacity_rejections += u64::from(update.capacity_rejected);
        Ok(update)
    }

    pub fn view(&self, width: u32, height: u32, exposure: f32) -> ViewParams {
        if self.vehicle.is_some()
            && let Some(view) = self.vehicle_view(width, height, exposure)
        {
            return view;
        }
        self.survey_view(width, height, exposure)
            .unwrap_or_else(|| {
                elevated_view(
                    self.center,
                    self.up,
                    self.east,
                    f64::from(self.survey.distance_m()),
                    width,
                    height,
                    exposure,
                )
            })
    }

    /// Owned survey framing in body space: the shared orbit offset around the
    /// active bookmark, clamped above local terrain like the shipped survey,
    /// with the free-look quaternion applied after the look-at. Physics state
    /// is never touched here.
    fn survey_view(&self, width: u32, height: u32, exposure: f32) -> Option<ViewParams> {
        use thessa_flight_authority::survey::SurveyCamera;
        let frame = self.sample.frame;
        let center = frame.body_to_render_point(self.center);
        let up = frame.body_to_render_direction(self.up);
        let (offset, site_up) = self.survey.offsets(up)?;
        let mut eye = center + offset.as_dvec3();
        let ground = self
            .stream
            .field()
            .height_m(frame.render_to_body_point(eye).normalize().to_array(), 32.0)
            .max(0.0);
        let minimum = self.stream.radius_m()
            + ground
            + thessa_flight_authority::survey::MIN_ALTITUDE_MARGIN_M;
        let body_eye =
            SurveyCamera::enforce_minimum_altitude(frame.render_to_body_point(eye), minimum);
        eye = frame.body_to_render_point(body_eye);
        let target = center;
        let up_ref = DVec3::new(site_up.x as f64, site_up.y as f64, site_up.z as f64);
        let axis = (eye - target).normalize();
        if !eye.is_finite()
            || !up_ref.is_finite()
            || !axis.is_finite()
            || axis.dot(up_ref).abs() > 1.0 - 1e-6
        {
            return None;
        }
        let base = glam::dcamera::rh::view::look_at_mat4(eye, target, up_ref).inverse();
        let world_from_view = base * glam::DMat4::from_quat(self.survey.look().as_dquat());
        Some(frame.view_in_body(&ViewParams {
            world_from_view,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                0.9,
                width as f64 / height as f64,
                1.0,
                1e8,
            ),
            world_position_m: eye,
            exposure,
        }))
    }

    /// Owned pilot-camera framing of the vehicle, staged in body space. The
    /// craft orientation is recovered from the published mesh instance so the
    /// camera always agrees with the rendered mesh. A degenerate frame falls
    /// back to a finite elevated view; physics state is never touched here.
    fn vehicle_view(&self, width: u32, height: u32, exposure: f32) -> Option<ViewParams> {
        use thessa_flight_authority::presentation::x15_asset_to_craft_rotation;
        let (_, instance) = self.vehicle.as_ref()?;
        let asset = x15_asset_to_craft_rotation().as_dquat().conjugate();
        let craft = (instance.rotation * asset).normalize();
        let up_body = self.center.try_normalize()?;
        let orientation = glam::Quat::from_xyzw(
            craft.x as f32,
            craft.y as f32,
            craft.z as f32,
            craft.w as f32,
        );
        let local_up = glam::Vec3::new(up_body.x as f32, up_body.y as f32, up_body.z as f32);
        let (eye, look_target, up) =
            self.camera
                .offsets(orientation, local_up)
                .unwrap_or_else(|| {
                    // Finite elevated fallback with an orthogonal reference axis; the
                    // primary path above is total for the validated pipeline.
                    let east =
                        glam::Vec3::new(self.east.x as f32, self.east.y as f32, self.east.z as f32);
                    (local_up * self.camera.distance_m(), glam::Vec3::ZERO, east)
                });
        let to_body = |v: glam::Vec3| DVec3::new(v.x as f64, v.y as f64, v.z as f64);
        let eye_m = self.center + to_body(eye);
        let target_m = self.center + to_body(look_target);
        let up_m = to_body(up);
        if !eye_m.is_finite() || !target_m.is_finite() || !up_m.is_finite() {
            return None;
        }
        Some(ViewParams {
            world_from_view: glam::dcamera::rh::view::look_at_mat4(eye_m, target_m, up_m).inverse(),
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(
                0.9,
                width as f64 / height as f64,
                1.0,
                1e8,
            ),
            world_position_m: eye_m,
            exposure,
        })
    }
}

#[cfg(test)]
#[path = "scene_server_tests.rs"]
mod server_tests;

/// Total degenerate fallback for failed survey/vehicle views: an elevated
/// view with a re-orthogonalized reference axis. Never NaNs; the primary
/// shared-controller paths above are total for the validated pipeline.
#[allow(clippy::too_many_arguments)]
fn elevated_view(
    center: DVec3,
    up: DVec3,
    east: DVec3,
    distance: f64,
    width: u32,
    height: u32,
    exposure: f32,
) -> ViewParams {
    let up_n = up.try_normalize().unwrap_or(DVec3::Y);
    let east_n = east
        .try_normalize()
        .unwrap_or_else(|| up_n.any_orthonormal_vector());
    let east_n = (east_n - up_n * east_n.dot(up_n))
        .try_normalize()
        .unwrap_or_else(|| up_n.any_orthonormal_vector());
    let distance = if distance.is_finite() && distance > 0.0 {
        distance
    } else {
        5000.0
    };
    let eye = if center.is_finite() {
        center + up_n * distance
    } else {
        up_n * (3_200_000.0 + distance)
    };
    let target = if center.is_finite() {
        center
    } else {
        eye - up_n * distance
    };
    ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, target, east_n).inverse(),
        clip_from_view: glam::dcamera::rh::proj::directx::perspective(
            0.9,
            width as f64 / height as f64,
            1.0,
            1e8,
        ),
        world_position_m: eye,
        exposure,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;

    #[test]
    #[cfg(feature = "hud")]
    fn map_never_requests_a_hypothetical_bake_or_steps_paused_physics() {
        use std::sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        };
        struct CountQueue {
            requests: Arc<AtomicUsize>,
            pending: bool,
        }
        impl thessa_flight_authority::BakeQueue for CountQueue {
            fn request_bake(&mut self, _: thessa_flight_authority::RailsBakeRequest) {
                assert!(!self.pending);
                self.pending = true;
                self.requests.fetch_add(1, Ordering::SeqCst);
            }
            fn poll_bake(&mut self) -> Option<Result<thessa_flight_authority::BakedRails, String>> {
                None
            }
            fn has_pending(&self) -> bool {
                self.pending
            }
            fn reset(&mut self) {
                self.pending = false;
            }
        }
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        let requests = Arc::new(AtomicUsize::new(0));
        scene.flight.as_mut().unwrap().bake = Box::new(CountQueue {
            requests: requests.clone(),
            pending: false,
        });
        scene.request_pause(true);
        scene.advance_flight(0.025, None).unwrap();
        assert_eq!(
            requests.load(Ordering::SeqCst),
            0,
            "closed map must not request powered-flight forecast work"
        );
        let state = scene.flight.as_ref().unwrap().state;
        let time = scene.sample.time;
        scene.set_map_open(true);
        assert!(scene.map_snapshot().unwrap().trajectory.is_none());
        assert_eq!(
            requests.load(Ordering::SeqCst),
            0,
            "reading UI data must never request a bake"
        );
        for _ in 0..10 {
            scene.advance_flight(0.0, None).unwrap();
        }
        assert_eq!(requests.load(Ordering::SeqCst), 0);
        let map = scene.map_snapshot().unwrap();
        assert!(!map.trajectory_pending && map.trajectory.is_none());
        assert_eq!(scene.flight.as_ref().unwrap().state, state);
        assert_eq!(scene.sample.time, time);
        assert_eq!(scene.flight.as_ref().unwrap().steps_this_frame, 0);
        scene.set_map_open(false);
        scene.advance_flight(0.0, None).unwrap();
        assert_eq!(requests.load(Ordering::SeqCst), 0);
        scene.reset_flight().unwrap();
        scene.set_map_open(true);
        scene.advance_flight(0.0, None).unwrap();
        assert_eq!(
            requests.load(Ordering::SeqCst),
            0,
            "opening a reset map must not fabricate a forecast"
        );
    }

    #[test]
    #[cfg(feature = "hud")]
    fn map_reads_the_actual_physics_jit_cache_without_advancing_or_extending_it() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene
            .flight
            .as_mut()
            .unwrap()
            .initialize_circular_orbit(scene.celestial.ephemeris(), 1_000_000.0, DVec3::Y)
            .unwrap();
        scene.control_mode = thessa_flight_authority::ControlMode::Direct;
        scene.flight.as_mut().unwrap().sas_enabled = false;
        scene.set_map_open(true);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
        let map = loop {
            scene.advance_flight(0.025, None).unwrap();
            let map = scene.map_snapshot().unwrap();
            if map.trajectory.is_some() {
                break map;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "physics JIT did not produce an executable segment: {} contact={} pending={} rails={}",
                scene.flight_telemetry().unwrap(),
                scene.flight.as_ref().unwrap().contact_active(),
                scene.flight.as_ref().unwrap().bake.has_pending(),
                scene.flight.as_ref().unwrap().rails.path().is_some()
            );
            std::thread::sleep(std::time::Duration::from_millis(1));
        };
        scene.request_pause(true);
        let state = scene.flight.as_ref().unwrap().state;
        let time = scene.flight.as_ref().unwrap().flight_time_s;
        scene.advance_flight(0.0, None).unwrap();
        let flight = scene.flight.as_ref().unwrap();
        let e = scene.celestial.ephemeris();
        let display = thessa_flight_authority::navigation::craft_display_body(e, flight, map.time);
        let until = map.execution_until.unwrap();
        let direct = thessa_flight_authority::navigation::project_jit_coast_path(
            e,
            &flight.rails,
            display,
            map.time,
            state.position_inertial_m,
            until,
        );
        assert_eq!(map.trajectory.as_ref().unwrap().1.points, direct.points);
        assert_eq!(map.trajectory.as_ref().unwrap().1.impact, direct.impact);
        assert!(flight.rails_bake_seconds.is_some());
        assert_eq!(flight.state, state);
        assert_eq!(flight.flight_time_s, time);
        assert_eq!(flight.steps_this_frame, 0);
        assert_eq!(scene.clock.effective_warp, Some(0.0));
        let last = map.trajectory.as_ref().unwrap().1.points.last().unwrap();
        let end_position = flight.rails.sample_at(until).unwrap().0
            - e.body_state(display, until).unwrap().position_inertial;
        assert_eq!(
            *last, end_position,
            "map endpoint must be the exact execution interpolant"
        );
        scene.flight.as_mut().unwrap().engine_active = true;
        scene.flight.as_mut().unwrap().throttle = 1.0;
        assert!(
            scene.map_snapshot().unwrap().trajectory.is_none(),
            "powered flight must not retain a hypothetical coast"
        );
    }

    #[test]
    fn pause_and_warp_feed_existing_fixed_step_authority_only() {
        let mut scene = Scene::load(Default::default()).unwrap();
        assert!(scene.flight_clock().is_none());
        assert!(scene.request_warp(2.0).is_err());
        scene.enable_live_flight().unwrap();
        scene.input.engine_active = false;
        scene.push_live_input(0.0, false, false);
        scene.request_warp(4.0).unwrap();
        scene.advance_flight(0.05, None).unwrap();
        assert_eq!(scene.sample.time.0, 0.2);
        assert_eq!(scene.flight.as_ref().unwrap().steps_this_frame, 24);
        let before = scene.flight.as_ref().unwrap().state;
        let pose = scene.sample.clone();
        scene.request_pause(true);
        scene.advance_flight(0.1, None).unwrap();
        assert_eq!(scene.flight.as_ref().unwrap().state, before);
        assert_eq!(scene.sample.time, pose.time);
        assert_eq!(scene.sample.frame, pose.frame);
        assert_eq!(scene.flight_clock().unwrap().effective_warp, Some(0.0));
        let budget = scene.flight_budget(0.05).unwrap();
        assert_eq!(
            (
                budget.steps_this_frame,
                budget.sim_time_advanced_s,
                budget.effective_warp,
                budget.requested_warp
            ),
            (0, 0.0, 0.0, 0.0)
        );
        scene.request_pause(false);
        scene.request_warp(0.0).unwrap();
        scene.advance_flight(0.1, None).unwrap();
        assert_eq!(scene.sample.time, pose.time);
        scene.request_warp(0.5).unwrap();
        scene.advance_flight(0.1, None).unwrap();
        assert_eq!(scene.sample.time.0, 0.25);
        assert_eq!(scene.flight_budget(0.1).unwrap().sim_time_advanced_s, 0.05);
        assert_eq!(scene.flight_clock().unwrap().authority_paused, Some(false));
        let clock = scene.flight_clock();
        for factor in [
            f64::NAN,
            f64::INFINITY,
            -1.0,
            thessa_flight_net::MAX_TIME_WARP + 1.0,
        ] {
            assert!(scene.request_warp(factor).is_err());
            assert_eq!(scene.flight_clock(), clock);
        }
        scene
            .request_warp(thessa_flight_net::MAX_TIME_WARP)
            .unwrap();
        scene
            .advance_flight(0.1, Some(std::time::Duration::ZERO))
            .unwrap();
        let flight = scene.flight.as_ref().unwrap();
        assert_eq!(flight.steps_this_frame, 1);
        assert!(flight.backlog_s() < 1.0 / 120.0);
        assert!(
            scene.sample.time.0 < 0.26,
            "work limit cannot manufacture requested elapsed physics"
        );
    }

    #[test]
    fn stopped_local_flight_relaunch_reuses_canonical_reset_without_rewind() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene.advance_flight(0.1, None).unwrap();
        let time = scene.sample.time;
        scene.flight.as_mut().unwrap().flight_error = Some("fixture stop".into());
        scene.input.throttle = 0.7;
        scene.input.keys.nose_down = true;
        scene.reset_flight().unwrap();
        assert!(!scene.flight_stopped());
        assert_eq!(scene.sample.time, time);
        assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, time.0);
        assert_eq!(scene.input.throttle, 0.0);
        assert!(scene.input.engine_active);
        assert!(!scene.input.keys.any());
        assert_eq!(scene.flight.as_ref().unwrap().control_input, DVec3::ZERO);
        assert_eq!(scene.flight.as_ref().unwrap().backlog_s(), 0.0);
        let center = scene.center;
        scene.reset_flight().unwrap();
        assert_eq!(scene.center, center);
        scene.request_pause(false);
        scene.push_live_input(0.0, false, false);
        scene.advance_flight(0.1, None).unwrap();
        assert!(scene.sample.time.0 > time.0);
    }

    #[test]
    fn audio_observation_follows_primary_without_owning_a_flight_clock() {
        let mut scene = Scene::load(Default::default()).unwrap();
        assert!(scene.audio_frame().is_none());
        scene.enable_live_flight().unwrap();
        scene.input.engine_active = true;
        scene.input.throttle = 0.5;
        scene.push_live_input(0.0, false, false);
        scene.advance_flight(1.0 / 60.0, None).unwrap();
        let frame = scene.audio_frame().unwrap();
        let flight = scene.flight.as_ref().unwrap();
        assert_eq!(frame.sim_time_s, flight.flight_time_s);
        assert_eq!(frame.throttle, flight.throttle);
        assert_eq!(frame.engine_active, flight.engine_active);
        assert_eq!(frame.source.position_m, scene.center);
        assert_eq!(
            frame.listener.position_m,
            scene.view(1, 1, 1.0).world_position_m
        );
        assert_eq!(
            frame.gpws_warning,
            scene
                .flight_data()
                .is_some_and(|d| d.altitude_datum_m < 100.0 && d.vertical_speed_mps < -1.0)
        );
        scene.flight.as_mut().unwrap().flight_error = Some("fixture stopped".into());
        assert!(scene.audio_frame().is_none());
    }

    #[test]
    fn live_canonical_flight_matches_direct_authority_and_publishes_one_complete_sample() {
        use thessa_flight_authority::{ControlMode, presentation::FlightPose};
        let mut scene = Scene::load(Default::default()).unwrap();
        let mut reference = scene.celestial.initial_flight().unwrap();
        reference.initialize_world_site(
            scene.stream.field().clone(),
            scene.launch_site,
            scene.celestial.ephemeris(),
        );
        scene.enable_live_flight().unwrap();
        assert!(std::sync::Arc::ptr_eq(
            scene
                .flight
                .as_ref()
                .unwrap()
                .terrain_field
                .as_ref()
                .unwrap(),
            scene.stream.field()
        ));
        let initial = scene.center;
        let vertices = scene.vehicle.as_ref().unwrap().0.primitives[0]
            .vertices
            .as_ptr();
        for _ in 0..60 {
            reference
                .advance(
                    scene.celestial.ephemeris(),
                    ControlMode::Navball,
                    1.0 / 60.0,
                )
                .unwrap();
            scene.advance_flight(1.0 / 60.0, None).unwrap();
            let flight = scene.flight.as_ref().unwrap();
            assert_eq!(flight.relative_position_m, reference.relative_position_m);
            assert_eq!(
                flight.state.orientation_body_to_inertial,
                reference.state.orientation_body_to_inertial
            );
            assert_eq!(flight.flight_time_s, reference.flight_time_s);
            assert_eq!(scene.sample.time.0, reference.flight_time_s);
            let pose = FlightPose::from_authority(&reference).unwrap();
            assert!(
                (scene.sample.frame.body_to_render_point(scene.center) - pose.render_position_m())
                    .length()
                    < 1e-8
            );
            let instance = scene.vehicle.as_ref().unwrap().1;
            let expected = (pose.visual_orientation()
                * thessa_flight_authority::presentation::x15_asset_to_craft_rotation())
            .as_dquat()
            .normalize();
            assert!(
                (scene.sample.frame.rotation() * instance.rotation)
                    .dot(expected)
                    .abs()
                    > 1.0 - 1e-12
            );
            assert!(
                thessa_render_wgpu::view::ViewUniformBlock::new(&scene.view(1280, 720, 1.0))
                    .is_some()
            );
        }
        assert!((scene.center - initial).length() > 100.0);
        let mut finer = scene.celestial.initial_flight().unwrap();
        finer.initialize_world_site(
            scene.stream.field().clone(),
            scene.launch_site,
            scene.celestial.ephemeris(),
        );
        for _ in 0..120 {
            finer
                .advance(
                    scene.celestial.ephemeris(),
                    ControlMode::Navball,
                    1.0 / 120.0,
                )
                .unwrap();
        }
        assert_eq!(finer.flight_time_s, reference.flight_time_s);
        assert_eq!(finer.relative_position_m, reference.relative_position_m);
        assert_eq!(
            finer.state.orientation_body_to_inertial,
            reference.state.orientation_body_to_inertial
        );
        assert_eq!(
            scene.vehicle.as_ref().unwrap().0.primitives[0]
                .vertices
                .as_ptr(),
            vertices
        );
        let before = (
            scene.sample.clone(),
            scene.center,
            scene.vehicle.as_ref().unwrap().1.position_m,
        );
        let authority_time = scene.flight.as_ref().unwrap().flight_time_s;
        assert!(scene.advance_flight(f64::NAN, None).is_err());
        assert!(scene.advance_flight(1.0, None).is_err());
        assert!(
            scene
                .set_presentation_time(thessa_sim_core::SimTime(50.0))
                .is_err()
        );
        assert_eq!(scene.flight.as_ref().unwrap().flight_time_s, authority_time);
        let invalid =
            FlightPose::new(thessa_sim_core::SimTime(1e308), DVec3::X, DQuat::IDENTITY).unwrap();
        assert!(scene.publish_vehicle_pose(invalid).is_err());
        assert_eq!(scene.sample.time, before.0.time);
        assert_eq!(scene.sample.frame, before.0.frame);
        assert_eq!(scene.sample.lighting, before.0.lighting);
        assert_eq!(scene.center, before.1);
        assert_eq!(scene.vehicle.as_ref().unwrap().1.position_m, before.2);
        scene.flight.as_mut().unwrap().state.velocity_inertial_mps = DVec3::NAN;
        scene.advance_flight(1.0 / 60.0, None).unwrap();
        assert!(scene.flight_stopped());
        assert_eq!(scene.sample.time, before.0.time);
        assert_eq!(scene.center, before.1);
        scene.advance_flight(0.1, None).unwrap();
        assert_eq!(scene.sample.time, before.0.time);
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn live_authority_updates_shared_mesh_terrain_lights_and_tracking_camera_on_gpu() {
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        let target = thessa_render_wgpu::terrain::OffscreenTarget::new(&backend, 257, 257).unwrap();
        let mut terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let mut mesh = scene
            .mesh_renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap()
            .unwrap();
        let render = |scene: &mut Scene, terrain: &mut TerrainRenderer, mesh: &mut SceneMeshes| {
            scene.publish_presentation(&backend, terrain).unwrap();
            let exposure = crate::manual_exposure(13.0);
            terrain
                .update_view(&backend, &scene.view(257, 257, exposure))
                .unwrap();
            scene
                .update_mesh(&backend, mesh, 257, 257, exposure)
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode(&mut encoder, &target.color, &target.depth);
            mesh.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let before = render(&mut scene, &mut terrain, &mut mesh);
        for _ in 0..60 {
            scene.advance_flight(1.0 / 60.0, None).unwrap();
        }
        let after = render(&mut scene, &mut terrain, &mut mesh);
        assert_ne!(before, after);
        scene.camera.set_chase(true);
        let chase = render(&mut scene, &mut terrain, &mut mesh);
        assert_ne!(after, chase, "chase framing must change the shared view");
        assert_eq!(mesh.draw_stats().instances, 1);
        assert_eq!(mesh.draw_stats().triangles, 78465);
        assert!(!scene.lighting_dirty);
        assert!(scene.sample.time.0 > 0.9);
        assert!(!scene.flight_stopped());
    }

    #[test]
    fn initial_flight_snapshot_keeps_valid_close_polar_camera_and_unit_instance() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_vehicle_preview().unwrap();
        let (_, instance) = scene.vehicle.as_ref().unwrap();
        assert!(
            instance
                .camera_relative_matrix(instance.position_m)
                .is_some()
        );
        assert_eq!(scene.up, DVec3::Y);
        assert!(!scene.camera.chase());
        assert_eq!(scene.camera.distance_m(), 32.0);
        for chase in [false, true] {
            scene.camera.set_chase(chase);
            for distance in [8.0, 32.0, 180.0] {
                scene.camera.set_distance_m(distance);
                for yaw in [0.0, 0.5, 1.5, 3.0, 4.5, 6.0] {
                    scene.camera.orbit_yaw(yaw);
                    let view = scene.view(1280, 720, 1.0);
                    assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                    assert!(view.world_position_m.is_finite());
                }
            }
            scene.camera.reset();
        }
    }

    #[test]
    fn survey_view_ignores_shared_camera_while_vehicle_view_tracks_it() {
        let mut scene = Scene::load(Default::default()).unwrap();
        let before = scene.view(1280, 720, 1.0).world_position_m;
        scene.camera.set_chase(true);
        scene.camera.orbit_yaw(1.2);
        scene.camera.zoom_by_factor(2.0);
        scene.camera.pan_by(50.0, -30.0);
        assert_eq!(scene.view(1280, 720, 1.0).world_position_m, before);

        scene.enable_vehicle_preview().unwrap();
        scene.camera.reset();
        scene.camera.set_chase(false);
        let orbit_eye = scene.view(1280, 720, 1.0).world_position_m;
        assert!(((orbit_eye - scene.center).length() - 32.0).abs() < 1e-3);
        scene.camera.orbit_yaw(0.7);
        let yawed = scene.view(1280, 720, 1.0).world_position_m;
        assert!((yawed - orbit_eye).length() > 0.5);
        scene.camera.set_chase(true);
        let chase_eye = scene.view(1280, 720, 1.0).world_position_m;
        assert!((chase_eye - yawed).length() > 0.5);
        scene.camera.reset();
        assert!(scene.camera.chase(), "reset preserves the chase selection");
        assert_eq!(scene.camera.distance_m(), 32.0);
        assert_eq!(scene.camera.pan(), glam::Vec2::ZERO);
    }

    #[test]
    fn survey_sites_reset_framing_without_touching_flight_or_orbit() {
        let mut scene = Scene::load(Default::default()).unwrap();
        assert_eq!(scene.site, 1);
        scene.survey.drag(40.0, -20.0, false);
        let orbit = scene.survey.orbit();
        for site in 0..scene.sites.len() {
            scene.survey.drag(10.0, 15.0, true);
            scene.survey.zoom(3.0);
            scene.select_site(site);
            assert_eq!(scene.site, site);
            assert_eq!(scene.survey.orbit(), orbit);
            assert_eq!(scene.survey.look(), glam::Quat::IDENTITY);
            assert_eq!(scene.survey.distance_m(), 5000.0);
            assert!((scene.center.normalize() - DVec3::from(scene.sites[site].0)).length() < 1e-12);
            assert!(scene.survey_view(1280, 720, 1.0).is_some());
        }
        let center = scene.center;
        let camera = scene.survey;
        scene.select_site(usize::MAX);
        assert_eq!(scene.center, center);
        assert_eq!(scene.survey, camera);
        scene.enable_live_flight().unwrap();
        let center = scene.center;
        let state = scene.flight.as_ref().unwrap().state;
        scene.select_site(0);
        assert_eq!(scene.center, center);
        assert_eq!(
            scene.flight.as_ref().unwrap().state.position_inertial_m,
            state.position_inertial_m
        );
    }

    #[test]
    #[cfg(feature = "hud")]
    fn hud_site_selection_does_not_reapply_stale_distance() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.survey.set_distance_m(80.0);
        scene.survey.drag(10.0, 20.0, true);
        let mut controls = crate::hud::Controls::from_scene(&scene);
        controls.survey_site = Some(0);
        controls.apply(&mut scene);
        assert_eq!(scene.site, 0);
        assert_eq!(scene.survey.distance_m(), 5000.0);
        assert_eq!(scene.survey.look(), glam::Quat::IDENTITY);
        scene.survey.drag(25.0, 30.0, false);
        let mut controls = crate::hud::Controls::from_scene(&scene);
        controls.reset_camera = true;
        controls.apply(&mut scene);
        assert_eq!(scene.survey, Default::default());
    }

    #[test]
    fn survey_eye_matches_pre_extraction_placement_in_render_and_body_frames() {
        let mut scene = Scene::load(Default::default()).unwrap();
        for time in [0.0, 72_000.0, -144_000.0] {
            scene
                .set_presentation_time(thessa_sim_core::SimTime(time))
                .unwrap();
            for site in 0..3 {
                scene.select_site(site);
                for distance in [80.0, 5000.0, 2_000_000.0] {
                    scene.survey.set_distance_m(distance);
                    scene.survey.set_orbit(glam::Quat::from_rotation_x(-0.4));
                    let frame = scene.sample.frame;
                    let up = frame.body_to_render_direction(scene.up);
                    // Original terrain.rs arithmetic, not another call to the shared helper.
                    let north = (DVec3::Y - up * up.y).normalize();
                    let east = north.cross(up).normalize();
                    let basis =
                        glam::Mat3::from_cols(east.as_vec3(), up.as_vec3(), -north.as_vec3());
                    let offset = basis
                        * (scene.survey.orbit() * glam::Vec3::new(0.0, distance * 0.18, distance));
                    let globe_eye = frame.body_to_render_point(scene.center) + offset.as_dvec3();
                    let body_eye = frame.render_to_body_point(globe_eye);
                    let minimum = scene.stream.radius_m()
                        + scene
                            .stream
                            .field()
                            .height_m(body_eye.normalize().to_array(), 32.0)
                            .max(0.0)
                        + 5.0;
                    let expected = if body_eye.length() < minimum {
                        body_eye.normalize() * minimum
                    } else {
                        body_eye
                    };
                    let view = scene.survey_view(1280, 720, 1.0).unwrap();
                    assert!((view.world_position_m - expected).length() < 1e-8);
                    assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                    let forward = view.world_from_view.transform_vector3(DVec3::NEG_Z);
                    assert!((forward - (scene.center - expected).normalize()).length() < 1e-9);
                }
            }
        }
    }

    #[test]
    fn live_input_push_copies_intent_and_slews_with_shared_factors() {
        use thessa_flight_authority::input::StickKeys;
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.push_live_input(0.1, true, false);
        assert_eq!(scene.input.throttle, 0.0, "no flight: no-op");
        scene.enable_live_flight().unwrap();
        // The compatibility starter arms full legacy propulsion, like the old
        // client inherits; the owned intent starts from that authority state.
        assert_eq!(scene.input.throttle, 1.0);
        assert!(scene.input.engine_active);
        assert_eq!(
            scene.control_mode,
            thessa_flight_authority::ControlMode::Navball
        );
        assert!(scene.flight.as_ref().unwrap().sas_enabled);

        scene.input.keys = StickKeys {
            nose_up: true,
            yaw_right: true,
            ..StickKeys::default()
        };
        scene.push_live_input(0.1, false, true);
        assert!((scene.input.throttle - (1.0 - 0.055)).abs() < 1e-12);
        let flight = scene.flight.as_ref().unwrap();
        assert_eq!(flight.throttle, scene.input.throttle);
        assert_eq!(flight.control_input, DVec3::new(1.0, 1.0, 0.0).normalize());
        scene.input.toggle_precision();
        scene.push_live_input(0.0, false, false);
        assert_eq!(
            scene.flight.as_ref().unwrap().control_input,
            DVec3::new(1.0, 1.0, 0.0).normalize() * 0.25
        );

        scene.input.cut();
        scene.input.toggle_engine();
        scene.cycle_control_mode();
        scene.toggle_sas();
        scene.push_live_input(f64::NAN, true, true);
        assert_eq!(scene.input.throttle, 0.0);
        let flight = scene.flight.as_ref().unwrap();
        assert_eq!(flight.throttle, 0.0);
        assert!(!flight.engine_active);
        assert_eq!(
            scene.control_mode,
            thessa_flight_authority::ControlMode::Rate
        );
        assert!(!scene.flight.as_ref().unwrap().sas_enabled);
        scene.input.full();
        scene.toggle_sas();
        assert_eq!(
            scene.input.throttle, 1.0,
            "full stages intent until the next push"
        );
        assert_eq!(
            scene.flight.as_ref().unwrap().throttle,
            0.0,
            "authority keeps the previous demand until pushed"
        );
        scene.push_live_input(0.0, false, false);
        assert_eq!(scene.flight.as_ref().unwrap().throttle, 1.0);
        assert!(scene.flight.as_ref().unwrap().sas_enabled);

        scene.input.set_throttle(2.0);
        assert_eq!(scene.input.throttle, 1.0);
        scene.input.set_throttle(f64::NAN);
        assert_eq!(scene.input.throttle, 1.0);
        let telemetry = scene.flight_telemetry().unwrap();
        for key in [
            "throttle=1.000",
            "engine=false",
            "control_mode=Rate",
            "sas=true",
        ] {
            assert!(telemetry.contains(key), "missing {key} in {telemetry}");
        }
    }

    #[test]
    fn full_thrust_with_engine_outflies_the_glide_like_direct_authority() {
        use thessa_flight_authority::ControlMode;
        let mut scene = Scene::load(Default::default()).unwrap();
        let mut reference = scene.celestial.initial_flight().unwrap();
        reference.initialize_world_site(
            scene.stream.field().clone(),
            scene.launch_site,
            scene.celestial.ephemeris(),
        );
        scene.enable_live_flight().unwrap();
        for _ in 0..600 {
            reference
                .advance(
                    scene.celestial.ephemeris(),
                    ControlMode::Navball,
                    1.0 / 60.0,
                )
                .unwrap();
            scene.push_live_input(1.0 / 60.0, false, false);
            scene.advance_flight(1.0 / 60.0, None).unwrap();
            assert_eq!(
                scene.flight.as_ref().unwrap().relative_position_m,
                reference.relative_position_m
            );
        }
        // Cutting the throttle + engine on the same starter state glides far
        // from the full-thrust trajectory above: same 60 steps, same frames,
        // different energy. The full reference is the direct production
        // authority, so equal thrust + different glide proves the input path
        // drives the solver instead of shadowing it.
        let thrust = reference.relative_position_m;
        let mut glide = scene.celestial.initial_flight().unwrap();
        glide.initialize_world_site(
            scene.stream.field().clone(),
            scene.launch_site,
            scene.celestial.ephemeris(),
        );
        glide.throttle = 0.0;
        glide.engine_active = false;
        for _ in 0..600 {
            glide
                .advance(
                    scene.celestial.ephemeris(),
                    ControlMode::Navball,
                    1.0 / 60.0,
                )
                .unwrap();
        }
        assert!((glide.relative_position_m - thrust).length() > 100.0);
    }

    #[test]
    fn part_toggles_flip_subsystems_and_report_telemetry() {
        use super::PartToggle;
        let mut scene = Scene::load(Default::default()).unwrap();
        assert_eq!(scene.part_states(), None);
        scene.apply_part_toggle(PartToggle::Gear);
        assert_eq!(scene.part_states(), None, "no flight: no-op");
        scene.enable_live_flight().unwrap();
        assert_eq!(scene.part_states(), Some((true, true, true, false)));
        for toggle in [
            PartToggle::Rcs,
            PartToggle::ReactionWheels,
            PartToggle::Gear,
            PartToggle::Parachutes,
        ] {
            scene.apply_part_toggle(toggle);
        }
        assert_eq!(scene.part_states(), Some((false, false, false, true)));
        for toggle in [
            PartToggle::Rcs,
            PartToggle::ReactionWheels,
            PartToggle::Gear,
            PartToggle::Parachutes,
        ] {
            scene.apply_part_toggle(toggle);
        }
        assert_eq!(scene.part_states(), Some((true, true, true, false)));
        scene.set_part_state(PartToggle::Gear, false);
        scene.set_part_state(PartToggle::Parachutes, true);
        assert_eq!(scene.part_states(), Some((true, true, false, true)));
        assert!(!scene.flight_stopped());
        let telemetry = scene.flight_telemetry().unwrap();
        for key in ["rcs=true", "wheels=true", "gear=false", "chutes=true"] {
            assert!(telemetry.contains(key), "missing {key} in {telemetry}");
        }
    }

    #[test]
    #[cfg(feature = "hud")]
    fn hud_part_checkboxes_adopt_on_change_only() {
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        let mut controls = crate::hud::Controls::from_scene(&scene);
        assert!(controls.rcs && controls.wheels && controls.gear && !controls.chutes);
        controls.rcs = false;
        controls.chutes = true;
        controls.apply(&mut scene);
        assert_eq!(scene.part_states(), Some((false, true, true, true)));
        let states = scene.part_states();
        controls.apply(&mut scene);
        assert_eq!(scene.part_states(), states);
    }

    #[test]
    fn flight_data_reports_launch_energy_and_climbs_under_thrust() {
        let mut scene = Scene::load(Default::default()).unwrap();
        assert_eq!(scene.flight_data(), None);
        scene.enable_live_flight().unwrap();
        let data = scene.flight_data().unwrap();
        assert!((data.altitude_agl_m - 500.0).abs() < 1.0);
        assert!((150.0..250.0).contains(&data.surface_speed_mps));
        assert!((-10.0..10.0).contains(&data.vertical_speed_mps));
        assert!(data.altitude_datum_m > data.altitude_agl_m);
        for _ in 0..600 {
            scene.push_live_input(1.0 / 60.0, false, false);
            scene.advance_flight(1.0 / 60.0, None).unwrap();
        }
        let later = scene.flight_data().unwrap();
        assert!(later.altitude_datum_m > data.altitude_datum_m + 100.0);
        assert!(later.surface_speed_mps > data.surface_speed_mps);
        let telemetry = scene.flight_telemetry().unwrap();
        for key in ["spd=", "vspeed=", "alt=", "agl="] {
            assert!(telemetry.contains(key), "missing {key} in {telemetry}");
        }
    }

    #[test]
    fn camera_gestures_never_touch_flight_physics() {
        use thessa_flight_authority::{ControlMode, presentation::FlightPose};
        let mut scene = Scene::load(Default::default()).unwrap();
        let mut reference = scene.celestial.initial_flight().unwrap();
        reference.initialize_world_site(
            scene.stream.field().clone(),
            scene.launch_site,
            scene.celestial.ephemeris(),
        );
        scene.enable_live_flight().unwrap();
        for step in 0..10 {
            scene.camera.orbit_yaw(0.05 * (step as f32 + 1.0));
            scene.camera.zoom_by_factor(1.01);
            scene.camera.pan_by(3.0, -2.0);
            if step == 5 {
                scene.camera.toggle_chase();
            }
            reference
                .advance(
                    scene.celestial.ephemeris(),
                    ControlMode::Navball,
                    1.0 / 60.0,
                )
                .unwrap();
            scene.advance_flight(1.0 / 60.0, None).unwrap();
            let flight = scene.flight.as_ref().unwrap();
            assert_eq!(flight.relative_position_m, reference.relative_position_m);
            assert_eq!(flight.flight_time_s, reference.flight_time_s);
            let pose = FlightPose::from_authority(&reference).unwrap();
            assert_eq!(scene.sample.time, pose.time());
            assert!(scene.view(1280, 720, 1.0).world_position_m.is_finite());
        }
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn actual_x15_preview_draws_into_existing_terrain_color_and_depth() {
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(Default::default()).unwrap();
        assert!(
            scene
                .mesh_renderer(
                    &backend,
                    thessa_render_wgpu::terrain::OffscreenTarget::FORMAT
                )
                .unwrap()
                .is_none()
        );
        scene.enable_vehicle_preview().unwrap();
        assert_eq!(scene.camera.distance_m(), 32.0);
        assert!(
            scene
                .set_presentation_time(thessa_sim_core::SimTime(1.0))
                .is_err()
        );
        let target = thessa_render_wgpu::terrain::OffscreenTarget::new(&backend, 257, 257).unwrap();
        let mut terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let mut mesh = scene
            .mesh_renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap()
            .unwrap();
        let exposure = crate::manual_exposure(13.0);
        let view = scene.view(257, 257, exposure);
        assert!(
            thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some(),
            "invalid close camera: {view:?}; center={:?}; up={:?}; east={:?}",
            scene.center,
            scene.up,
            scene.east
        );
        terrain
            .update_view(&backend, &scene.view(257, 257, exposure))
            .unwrap();
        scene
            .update_mesh(&backend, &mut mesh, 257, 257, exposure)
            .unwrap();
        let mut render = |with_mesh: bool| {
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            terrain.encode(&mut encoder, &target.color, &target.depth);
            if with_mesh {
                mesh.encode(&mut encoder, &target.color, &target.depth);
            }
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let without = render(false);
        let with = render(true);
        assert!(
            without
                .as_chunks::<4>()
                .0
                .iter()
                .zip(with.as_chunks::<4>().0.iter())
                .filter(|(a, b)| a != b)
                .count()
                > 100,
            "real mesh was invisible"
        );
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn sampled_motion_updates_gpu_lighting_without_rebuilding_height_geometry() {
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(thessa_graphics::RequestedGraphics::default()).unwrap();
        let target = thessa_render_wgpu::terrain::OffscreenTarget::new(&backend, 257, 257).unwrap();
        let mut renderer = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let geometry = renderer.read_vertices(&backend).unwrap();
        let exposure = crate::manual_exposure(13.0);
        let render = |scene: &Scene, renderer: &mut TerrainRenderer| {
            renderer
                .update_view(&backend, &scene.view(257, 257, exposure))
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            renderer.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let before = render(&scene, &mut renderer);
        scene
            .set_presentation_time(thessa_sim_core::SimTime(72_000.0))
            .unwrap();
        // Isolate the pose/light publication stage: initial terrain selection
        // itself can legitimately remove out-of-view bootstrap roots.
        scene.publish_presentation(&backend, &mut renderer).unwrap();
        assert!(!scene.lighting_dirty);
        assert_eq!(renderer.read_vertices(&backend).unwrap(), geometry);
        let after = render(&scene, &mut renderer);
        assert_ne!(
            before, after,
            "new sample lighting never reached the terrain GPU pass"
        );
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn unchanged_sky_uploads_once_and_disable_releases_mesh_grid() {
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_vehicle_preview().unwrap();
        let mut terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let camera = scene.view(320, 240, 1.0).world_position_m;
        scene.update_environment(camera);
        scene.publish_presentation(&backend, &mut terrain).unwrap();
        assert!(terrain.sky_enabled());
        assert!(
            scene.environment_grid().is_some(),
            "publish the initial grid on this tick"
        );
        let mut meshes = scene
            .mesh_renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap()
            .unwrap();
        for _ in 0..4 {
            scene.update_environment(camera);
            scene
                .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
                .unwrap();
        }
        assert_eq!(meshes.opaque().unwrap().environment_stats().4, 1);
        assert_eq!(scene.gpu_environment.as_ref().unwrap().uploads(), 1);
        // Replacing either consumer adopts the current buffers without another
        // upload. A scene-wide "already bound" flag would lose the new binding.
        terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        assert!(terrain.sky_enabled());
        let mut replacement = scene
            .mesh_renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap()
            .unwrap();
        scene
            .update_mesh(&backend, &mut replacement, 320, 240, 1.0)
            .unwrap();
        assert_eq!(replacement.opaque().unwrap().environment_stats().4, 1);
        assert_eq!(scene.gpu_environment.as_ref().unwrap().uploads(), 1);
        // Changed resolved policy must not let a replacement renderer bind a
        // stale enabled GPU publication even before the next ordinary tick.
        scene.requested.atmosphere.enabled = false;
        scene.requested.upper_atmosphere.airglow = false;
        scene.requested.upper_atmosphere.aurora = false;
        terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        assert!(!terrain.sky_enabled());
        scene.update_environment(camera);
        scene.publish_presentation(&backend, &mut terrain).unwrap();
        assert!(!terrain.sky_enabled());
        scene
            .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
            .unwrap();
        let stats = meshes.opaque().unwrap().environment_stats();
        assert!(!stats.0);
        assert_eq!(stats.3, 0);
        assert_eq!(stats.4, 2);
        assert_eq!(
            scene.lighting().ambient_lux,
            scene.sample.lighting.ambient_lux
        );
    }

    #[test]
    fn moving_celestial_samples_keep_body_attached_camera_and_failed_sample_preserves_state() {
        let mut scene = Scene::load(thessa_graphics::RequestedGraphics::default()).unwrap();
        let epoch = scene.view(1280, 720, 1.0);
        let initial_lighting = scene.sample.lighting;
        for time in [1.0, 72_000.0, 144_000.0, -72_000.0] {
            scene
                .set_presentation_time(thessa_sim_core::SimTime(time))
                .unwrap();
            let view = scene.view(1280, 720, 1.0);
            // Legacy survey offsets cross f64 -> f32 at the site basis.
            assert!((view.world_position_m - epoch.world_position_m).length() < 0.002);
            assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
            let forward = view.world_from_view.transform_vector3(DVec3::NEG_Z);
            let original_forward = epoch.world_from_view.transform_vector3(DVec3::NEG_Z);
            assert!((forward - original_forward).length() < 1e-6);
        }
        assert_ne!(scene.sample.lighting, initial_lighting);
        assert_eq!(
            scene.lighting().directions,
            scene.sample.lighting.directions
        );
        assert_eq!(
            scene.lighting().colors_lux,
            scene.sample.lighting.colors_lux
        );
        let before = scene.sample.clone();
        let lighting = scene.sample.lighting;
        assert!(
            scene
                .set_presentation_time(thessa_sim_core::SimTime(f64::NAN))
                .is_err()
        );
        assert_eq!(scene.sample.time, before.time);
        assert_eq!(scene.sample.frame, before.frame);
        assert_eq!(scene.sample.lighting, lighting);
        scene.lighting_dirty = false;
        scene.set_presentation_time(before.time).unwrap();
        assert!(
            !scene.lighting_dirty,
            "same-time samples must be free of dirty work"
        );
    }

    #[test]
    fn survey_views_keep_checked_camera_relative_abi() {
        let mut scene = Scene::load(Default::default()).unwrap();
        for distance in [80.0, 5_000.0, 2_000_000.0] {
            scene.survey.set_distance_m(distance);
            for step in 0..32 {
                scene.survey.set_orbit(glam::Quat::from_rotation_y(
                    step as f32 * std::f32::consts::TAU / 32.0,
                ));
                let view = scene.view(1280, 720, 1.0);
                assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                assert!(
                    (view.world_from_view.w_axis.truncate() - view.world_position_m).length()
                        < 1e-6
                );
            }
        }
    }
}
