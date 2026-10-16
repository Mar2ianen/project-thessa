//! A real canonical-field resident partition, shared by window/headless paths.
//! Shared live terrain streaming policy; not the networked game client.
use crate::celestial::{CelestialPresentation, Sample};
use glam::DVec3;
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

pub struct Scene {
    flight: Option<thessa_flight_authority::FlightAuthority>,
    launch_site: [f64; 3],
    vehicle: Option<(
        thessa_render_core::mesh::MeshAsset,
        thessa_render_core::mesh::MeshInstance,
    )>,
    /// Owned pilot camera (vehicle/preview view only). The terrain-only
    /// survey view keeps its legacy angle orbit until the map view migrates.
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
    pub distance: f64,
    pub angle: f64,
    celestial: CelestialPresentation,
    pub sample: Sample,
    lighting_dirty: bool,
}

/// Installed-subsystem selector sharing keys with the old pilot
/// (`R` RCS, `Y` reaction wheels, `G` gear, `P` parachutes). The old client
/// forwards the same commands to the server; the local viewer applies them
/// to its own authority only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartToggle {
    Rcs,
    ReactionWheels,
    Gear,
    Parachutes,
}

impl Scene {
    pub fn vehicle_preview(&self) -> bool {
        self.vehicle.is_some()
    }
    pub fn camera_distance_limits(&self) -> (f64, f64, f64) {
        if self.vehicle_preview() {
            (8.0, 180.0, 32.0)
        } else {
            (1_000.0, 200_000.0, 30_000.0)
        }
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
        let celestial = CelestialPresentation::new(&system, ephemeris, body)?;
        let sample = celestial.sample(thessa_sim_core::SimTime::EPOCH)?;
        let up = DVec3::from(sites[1]); // same HIGHLANDS bookmark as the shipped client
        let center = up * (field.params.radius_m + field.height_m(up.to_array(), 32.0).max(0.0));
        let east = DVec3::Y.cross(up).normalize();
        let stream = TerrainStream::new(field, StreamingPolicy::default())?;
        eprintln!(
            "canonical Thessa: {} resident tiles; highlands={:?}",
            stream.visible().len(),
            center
        );
        Ok(Self {
            flight: None,
            launch_site: sites[0],
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
            center,
            up,
            east,
            distance: 30_000.0,
            angle: 0.0,
            celestial,
            sample,
            lighting_dirty: false,
        })
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
            "X15 initial flight-state preview: {} primitives, {} vertices, {} triangles; CC BY 4.0 cmoreau; opaque native base-color/Lambert shading (not PBR parity)",
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

    /// Canonical production launch setup and fixed-step authority, not a new
    /// physics clock/controller. No network or player-flight-input host yet.
    pub fn enable_live_flight(&mut self) -> Result<(), String> {
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
        self.control_mode = thessa_flight_authority::ControlMode::Navball;
        self.flight = Some(flight);
        eprintln!(
            "Live local flight: canonical COAST launch; existing FlightAuthority, Navball attitude hold; no flight-input/network/UI parity"
        );
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
        Ok(())
    }

    pub fn advance_flight(
        &mut self,
        elapsed_s: f64,
        budget: Option<std::time::Duration>,
    ) -> Result<(), String> {
        let Some(flight) = &mut self.flight else {
            return Ok(());
        };
        if !elapsed_s.is_finite() || !(0.0..=0.1).contains(&elapsed_s) {
            return Err("flight elapsed must be finite and within 0..=0.1s".into());
        }
        if flight.flight_error.is_some() || elapsed_s == 0.0 {
            return Ok(());
        }
        if let Err(error) = flight.advance_with_budget(
            self.celestial.ephemeris(),
            self.control_mode,
            elapsed_s,
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
        if pose.time() != self.sample.time {
            self.publish_vehicle_pose(pose)?;
        }
        Ok(())
    }

    pub fn flight_telemetry(&self) -> Option<String> {
        self.flight.as_ref().map(|flight|format!("authority_time_s={} presentation_time_s={} relative_position_m={:?} steps_last_advance={} backlog_s={} stopped={} stop_reason={:?} chase={} throttle={:.3} engine={} control_mode={:?} sas={} rcs={} wheels={} gear={} chutes={}",flight.flight_time_s,self.sample.time.0,flight.relative_position_m,flight.steps_this_frame,flight.backlog_s(),self.flight_stopped(),flight.flight_error,self.camera.chase(),flight.throttle,flight.engine_active,self.control_mode,flight.sas_enabled,flight.rcs_enabled,flight.reaction_wheels_enabled,flight.gear_down,flight.parachutes_armed))
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
        let Some(flight) = &mut self.flight else {
            return;
        };
        self.input.slew(dt_s, throttle_up, throttle_down);
        flight.throttle = self.input.throttle;
        flight.engine_active = self.input.engine_active;
        flight.control_input = self.input.stick_scaled();
    }

    /// Cycle the assist control law (`Navball -> Rate -> Direct -> MouseAim`).
    /// Window `C` key and HUD button share this; the old client owns the same
    /// cycle through its mode HUD.
    pub fn cycle_control_mode(&mut self) {
        self.control_mode = self.control_mode.next();
    }

    pub fn toggle_sas(&mut self) {
        if let Some(flight) = &mut self.flight {
            flight.sas_enabled = !flight.sas_enabled;
        }
    }

    #[cfg(feature = "hud")]
    pub fn sas_enabled(&self) -> bool {
        self.flight
            .as_ref()
            .is_some_and(|flight| flight.sas_enabled)
    }

    #[cfg(feature = "hud")]
    pub fn set_sas(&mut self, enabled: bool) {
        if let Some(flight) = &mut self.flight {
            flight.sas_enabled = enabled;
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
        }
    }

    pub fn mesh_renderer(
        &self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<Option<thessa_render_wgpu::mesh::MeshRenderer>, BackendError> {
        self.vehicle
            .as_ref()
            .map(|(source, _)| {
                thessa_render_wgpu::mesh::MeshRenderer::new(backend, source, format, 1)
            })
            .transpose()
    }
    pub fn update_mesh(
        &self,
        backend: &Backend,
        renderer: &mut thessa_render_wgpu::mesh::MeshRenderer,
        width: u32,
        height: u32,
        exposure: f32,
    ) -> Result<(), BackendError> {
        let instances = self
            .vehicle
            .as_ref()
            .map(|(_, instance)| std::slice::from_ref(instance))
            .unwrap_or(&[]);
        renderer.update(
            backend,
            &self.view(width, height, exposure),
            &self.sample.lighting,
            instances,
        )
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

    fn publish_presentation(
        &mut self,
        backend: &Backend,
        renderer: &mut TerrainRenderer,
    ) -> Result<(), BackendError> {
        if self.lighting_dirty {
            renderer.update_lighting(backend, &self.sample.lighting)?;
            self.lighting_dirty = false;
        }
        Ok(())
    }

    pub fn renderer(
        &mut self,
        backend: &Backend,
        format: wgpu::TextureFormat,
    ) -> Result<TerrainRenderer, BackendError> {
        let patches: Vec<_> = self
            .stream
            .visible()
            .iter()
            .map(|&key| TerrainPatch {
                anchor: self.anchor(key),
                page: self.stream.height(key).unwrap().page,
            })
            .collect();
        let mut renderer = TerrainRenderer::with_capacity(
            backend,
            &patches,
            format,
            TerrainCapacity {
                patches: 1024,
                height_grid_size: 33,
                material_layers: backend.device.limits().max_texture_array_layers.min(256),
            },
        )?;
        let stats = renderer.update_materials(
            backend,
            self.stream.material_demand(),
            self.stream.materials(),
        )?;
        renderer.update_lighting(backend, &self.sample.lighting)?;
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
        self.publish_presentation(backend, renderer)?;
        let camera = self.view(width, height, exposure);
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
    }

    fn survey_view(&self, width: u32, height: u32, exposure: f32) -> ViewParams {
        let frame = self.sample.frame;
        let render_view = orbit_view(
            frame.body_to_render_point(self.center),
            frame.body_to_render_direction(self.up),
            frame.body_to_render_direction(self.east),
            self.distance,
            self.angle,
            width,
            height,
            exposure,
        );
        frame.view_in_body(&render_view)
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

#[allow(clippy::too_many_arguments)]
fn orbit_view(
    center: DVec3,
    up: DVec3,
    east: DVec3,
    distance: f64,
    angle: f64,
    width: u32,
    height: u32,
    exposure: f32,
) -> ViewParams {
    let north = east.cross(up);
    let tangent = north * angle.cos() + east * angle.sin();
    let eye = center + (up - tangent) * distance;
    ViewParams {
        world_from_view: glam::dcamera::rh::view::look_at_mat4(eye, center, north).inverse(),
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
            scene.sample,
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
        let render = |scene: &mut Scene,
                      terrain: &mut TerrainRenderer,
                      mesh: &mut thessa_render_wgpu::mesh::MeshRenderer| {
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
        assert_eq!(flight.control_input, DVec3::new(1.0, 1.0, 0.0));
        scene.input.toggle_precision();
        scene.push_live_input(0.0, false, false);
        assert_eq!(
            scene.flight.as_ref().unwrap().control_input,
            DVec3::new(0.25, 0.25, 0.0)
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
        assert_eq!(scene.camera_distance_limits(), (8.0, 180.0, 32.0));
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
        let render = |with_mesh: bool| {
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
    fn moving_celestial_samples_keep_body_attached_camera_and_failed_sample_preserves_state() {
        let mut scene = Scene::load(thessa_graphics::RequestedGraphics::default()).unwrap();
        let epoch = scene.view(1280, 720, 1.0);
        let initial_lighting = scene.sample.lighting;
        for time in [1.0, 72_000.0, 144_000.0, -72_000.0] {
            scene
                .set_presentation_time(thessa_sim_core::SimTime(time))
                .unwrap();
            let view = scene.view(1280, 720, 1.0);
            assert!((view.world_position_m - epoch.world_position_m).length() < 1e-8);
            assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
            let forward = view.world_from_view.transform_vector3(DVec3::NEG_Z);
            let original_forward = epoch.world_from_view.transform_vector3(DVec3::NEG_Z);
            assert!((forward - original_forward).length() < 1e-12);
        }
        assert_ne!(scene.sample.lighting, initial_lighting);
        let before = scene.sample;
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
    fn orbit_views_keep_checked_camera_relative_abi() {
        let center = DVec3::Z * 3_200_000.0;
        for distance in [1_000.0, 30_000.0, 200_000.0] {
            for step in 0..32 {
                let view = orbit_view(
                    center,
                    DVec3::Z,
                    DVec3::X,
                    distance,
                    step as f64 * std::f64::consts::TAU / 32.0,
                    1280,
                    720,
                    1.0,
                );
                assert!(thessa_render_wgpu::view::ViewUniformBlock::new(&view).is_some());
                assert!(
                    (view.world_from_view.w_axis.truncate() - view.world_position_m).length()
                        < 1e-6
                );
            }
        }
    }
}
