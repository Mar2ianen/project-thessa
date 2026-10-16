//! App-owned egui HUD. The GPU pass loads the terrain's existing color target;
//! it never imports/copies the scene or creates another GPU device.
use thessa_render_wgpu::Backend;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub vehicle_preview: bool,
    pub chase: bool,
    pub reset_camera: bool,
    pub survey_site: Option<usize>,
    pub survey_distance: f64,
    pub distance: f64,
    pub throttle: f64,
    pub engine: bool,
    pub precision: bool,
    pub sas: bool,
    pub cycle_mode: bool,
    pub rcs: bool,
    pub wheels: bool,
    pub gear: bool,
    pub chutes: bool,
    pub exposure_ev100: f32,
    pub vsync: bool,
}

impl Controls {
    pub fn from_scene(scene: &crate::scene::Scene) -> Self {
        Self {
            vehicle_preview: scene.vehicle_preview(),
            chase: scene.camera.chase(),
            reset_camera: false,
            survey_site: None,
            survey_distance: f64::from(scene.survey.distance_m()),
            distance: f64::from(scene.camera.distance_m()),
            throttle: scene.input.throttle,
            engine: scene.input.engine_active,
            precision: scene.input.precision,
            sas: scene.sas_enabled(),
            cycle_mode: false,
            rcs: scene.part_states().unwrap_or((true, true, true, false)).0,
            wheels: scene.part_states().unwrap_or((true, true, true, false)).1,
            gear: scene.part_states().unwrap_or((true, true, true, false)).2,
            chutes: scene.part_states().unwrap_or((true, true, true, false)).3,
            exposure_ev100: scene.requested.renderer.exposure_ev100,
            vsync: scene.requested.renderer.vsync,
        }
    }

    /// Graphics edits remain requests and must go through capability resolution.
    pub fn apply(self, scene: &mut crate::scene::Scene) {
        scene.camera.set_chase(self.chase);
        if let Some(site) = self.survey_site {
            scene.select_site(site);
        }
        if self.reset_camera {
            if scene.vehicle_preview() {
                scene.camera.reset();
            } else {
                scene.survey.reset();
            }
            scene.requested.renderer.exposure_ev100 = self.exposure_ev100;
            scene.requested.renderer.vsync = self.vsync;
            return;
        }
        if scene.vehicle_preview() {
            scene.camera.set_distance_m(self.distance as f32);
        } else if self.survey_site.is_none() {
            scene.survey.set_distance_m(self.survey_distance as f32);
        }
        if scene.live_flight() {
            scene.input.set_throttle(self.throttle);
            scene.input.engine_active = self.engine;
            scene.input.precision = self.precision;
            scene.set_sas(self.sas);
            if self.cycle_mode {
                scene.cycle_control_mode();
            }
            use crate::scene::PartToggle;
            for (toggle, enabled) in [
                (PartToggle::Rcs, self.rcs),
                (PartToggle::ReactionWheels, self.wheels),
                (PartToggle::Gear, self.gear),
                (PartToggle::Parachutes, self.chutes),
            ] {
                if scene.part_states().is_some_and(|state| {
                    let current = match toggle {
                        PartToggle::Rcs => state.0,
                        PartToggle::ReactionWheels => state.1,
                        PartToggle::Gear => state.2,
                        PartToggle::Parachutes => state.3,
                    };
                    current != enabled
                }) {
                    scene.set_part_state(toggle, enabled);
                }
            }
        }
        scene.requested.renderer.exposure_ev100 = self.exposure_ev100;
        scene.requested.renderer.vsync = self.vsync;
    }
}

#[derive(Default)]
pub struct Stats {
    pub fleet: Option<crate::scene::FleetObserver>,
    pub live_flight: bool,
    pub server_snapshots: Option<u64>,
    pub flight_stopped: bool,
    pub chase: bool,
    pub throttle: f64,
    pub engine: bool,
    pub control_mode: Option<thessa_flight_authority::ControlMode>,
    pub stick: [f64; 3],
    pub parts: (bool, bool, bool, bool),
    pub data: Option<crate::scene::FlightData>,
    pub survey_label: &'static str,
    pub survey_distance: f64,
    pub submitted_frames: u64,
    pub previous_cpu_ms: f64,
    pub sim_time_s: f64,
    pub tiles: usize,
    pub cache: (usize, usize),
    pub pending: (usize, usize),
    pub publications: u64,
    pub regenerated: u64,
    pub upload_bytes: u64,
    pub capacity_rejections: u64,
}

impl Stats {
    pub fn from_scene(scene: &crate::scene::Scene, frames: u64, cpu_ms: f64) -> Self {
        Self {
            submitted_frames: frames,
            live_flight: scene.live_flight(),
            server_snapshots: scene.server_snapshot_count(),
            fleet: scene.fleet_observer().cloned(),
            flight_stopped: scene.flight_stopped(),
            chase: scene.camera.chase(),
            throttle: scene.input.throttle,
            engine: scene.input.engine_active,
            control_mode: scene.live_flight().then_some(scene.control_mode),
            stick: scene.input.stick_scaled().to_array(),
            parts: scene.part_states().unwrap_or((true, true, true, false)),
            data: scene.flight_data(),
            survey_label: scene
                .sites
                .get(scene.site)
                .map(|site| site.1)
                .unwrap_or("—"),
            survey_distance: f64::from(scene.survey.distance_m()),
            previous_cpu_ms: cpu_ms,
            sim_time_s: scene.sample.time.0,
            tiles: scene.stream.visible().len(),
            cache: scene.stream.cache_entries(),
            pending: scene.stream.pending_jobs(),
            publications: scene.publications,
            regenerated: scene.regenerated,
            upload_bytes: scene.material_upload_bytes,
            capacity_rejections: scene.capacity_rejections,
        }
    }
}

pub struct Hud {
    pub context: egui::Context,
    renderer: egui_wgpu::Renderer,
    adapter: String,
    navball: Option<egui::TextureHandle>,
    navball_up: glam::DVec3,
    navball_time_s: f64,
}

impl Hud {
    pub fn new(backend: &Backend, format: wgpu::TextureFormat) -> Self {
        let context = context();
        Self {
            context,
            renderer: egui_wgpu::Renderer::new(
                &backend.device,
                format,
                egui_wgpu::RendererOptions::default(),
            ),
            adapter: backend.adapter_name.clone(),
            navball: None,
            navball_up: glam::DVec3::NAN,
            navball_time_s: f64::NAN,
        }
    }

    pub fn run(
        &mut self,
        input: egui::RawInput,
        controls: &mut Controls,
        stats: &Stats,
    ) -> egui::FullOutput {
        // Same rebuild policy as the shipped pilot HUD: at most 30 Hz and
        // only after real attitude travel, on presentation (not wall) time
        // so headless captures stay deterministic.
        use thessa_flight_authority::navball::{
            MAX_REFRESH_HZ, NAVBALL_TEXTURE_SIZE, REBUILD_TRAVEL_SQUARED, navball_pixels,
        };
        if let Some(data) = stats.data {
            let due = stats.sim_time_s - self.navball_time_s >= 1.0 / MAX_REFRESH_HZ;
            let moved =
                data.local_up_body.distance_squared(self.navball_up) > REBUILD_TRAVEL_SQUARED;
            if self.navball.is_none() || (due && moved) {
                let pixels = navball_pixels(data.local_up_body);
                let image = egui::ColorImage::from_rgba_unmultiplied(
                    [NAVBALL_TEXTURE_SIZE as usize, NAVBALL_TEXTURE_SIZE as usize],
                    &pixels,
                );
                let handle = self.context.load_texture(
                    "thessa-navball",
                    image,
                    egui::TextureOptions::LINEAR,
                );
                self.navball = Some(handle);
                self.navball_up = data.local_up_body;
                self.navball_time_s = stats.sim_time_s;
            }
        } else {
            self.navball = None;
        }
        let navball = self.navball.as_ref();
        widgets(
            &self.context,
            input,
            controls,
            stats,
            &self.adapter,
            navball,
        )
    }

    /// Encode after terrain, on the same target/encoder. Auxiliary command
    /// buffers from egui callbacks (if introduced) must precede this encoder.
    pub fn encode(
        &mut self,
        backend: &Backend,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        mut output: egui::FullOutput,
        size: [u32; 2],
    ) -> Vec<wgpu::CommandBuffer> {
        for (id, deltas) in &output.textures_delta.set {
            for delta in deltas {
                self.renderer
                    .update_texture(&backend.device, &backend.queue, *id, delta);
            }
        }
        let jobs = self
            .context
            .tessellate(output.shapes, output.pixels_per_point);
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: size,
            pixels_per_point: output.pixels_per_point,
        };
        let commands =
            self.renderer
                .update_buffers(&backend.device, &backend.queue, encoder, &jobs, &screen);
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("terrain-egui-hud-load-existing-color"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                ..Default::default()
            });
            self.renderer
                .render(&mut pass.forget_lifetime(), &jobs, &screen);
        }
        // Meshes/bind groups recorded above retain the resources they use.
        for id in &output.textures_delta.free {
            self.renderer.free_texture(id);
        }
        // egui 0.37 asserts in debug builds if handled deltas are simply
        // dropped. Acknowledge them only after uploads and frees above.
        output.textures_delta.clear();
        commands
    }
}

fn context() -> egui::Context {
    let context = egui::Context::default();
    context.set_visuals(egui::Visuals::dark());
    // Reuse the shipped game's OFL-1.1 font instead of relying on egui's
    // (empty in 0.37) defaults or adding another set of font assets/licenses.
    let mut fonts = egui::FontDefinitions::empty();
    let name = "thessa-noto-sans".to_owned();
    fonts.font_data.insert(
        name.clone(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!(
            "../../../assets/fonts/NotoSans-Regular.ttf"
        ))),
    );
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts.families.insert(family, vec![name.clone()]);
    }
    context.set_fonts(fonts);
    context
}

fn widgets(
    context: &egui::Context,
    input: egui::RawInput,
    controls: &mut Controls,
    stats: &Stats,
    adapter: &str,
    navball: Option<&egui::TextureHandle>,
) -> egui::FullOutput {
    context.run_ui(input, |root| {
        let context = root.ctx();
        egui::Window::new("Thessa | terrain migration")
            .default_pos([16.0, 16.0])
            .default_width(300.0)
            .resizable(false)
            .show(context, |ui| {
                ui.label("Owned renderer · shared-device egui HUD");
                ui.small(adapter);
                ui.label(format!("Presentation time: {:.3} s", stats.sim_time_s));
                if stats.live_flight {
                    ui.colored_label(
                        if stats.flight_stopped {
                            egui::Color32::RED
                        } else if stats.server_snapshots == Some(0) {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::LIGHT_GREEN
                        },
                        if stats.server_snapshots.is_some() {
                            if stats.flight_stopped { "Server flight stopped (last valid presentation)" }
                            else if stats.server_snapshots == Some(0) { "Waiting for first server snapshot (no local stepping)" }
                            else { "Server flight · PRIMARY · COAST (no local stepping)" }
                        } else if stats.flight_stopped { "Local flight stopped (last valid presentation)" }
                        else if stats.chase { "Local flight · COAST · chase" }
                        else { "Local flight · COAST" },
                    );
                    if let Some(count) = stats.server_snapshots {
                        ui.small(format!("Server snapshots: {count} · readout = newest acknowledgement; scene = interpolated"));
                    }
                }
                ui.separator();
                ui.label(format!("Resident terrain: {} tiles", stats.tiles));
                ui.label(format!(
                    "Cache H/M: {}/{} · pending: {}/{}",
                    stats.cache.0, stats.cache.1, stats.pending.0, stats.pending.1
                ));
                ui.label(format!(
                    "Publications: {} · regenerated: {}",
                    stats.publications, stats.regenerated
                ));
                ui.label(format!(
                    "Material upload: {:.2} MiB · refusals: {}",
                    stats.upload_bytes as f64 / (1024.0 * 1024.0),
                    stats.capacity_rejections
                ));
                ui.label(format!("Submitted frames: {}", stats.submitted_frames));
                ui.label(format!(
                    "Previous host frame: {:.2} ms (includes waits)",
                    stats.previous_cpu_ms
                ));
                ui.small("Host wall time, not GPU timing or game FPS.");
                ui.separator();
                if stats.live_flight {
                    if let (Some(data), Some(navball)) = (stats.data, navball) {
                        ui.horizontal(|ui| {
                            ui.image((navball.id(), egui::vec2(96.0, 96.0)));
                            ui.vertical(|ui| {
                                ui.label(format!("Speed {:.1} m/s", data.surface_speed_mps));
                                ui.label(format!(
                                    "Alt {:.0} m (AGL {:.0} m)",
                                    data.altitude_datum_m, data.altitude_agl_m
                                ));
                                ui.label(format!("V/S {:+.1} m/s", data.vertical_speed_mps));
                            });
                        });
                    } else if let Some(data) = stats.data {
                        ui.label(format!(
                            "Speed {:.1} m/s · Alt {:.0} m (AGL {:.0} m) · V/S {:+.1} m/s",
                            data.surface_speed_mps,
                            data.altitude_datum_m,
                            data.altitude_agl_m,
                            data.vertical_speed_mps,
                        ));
                    }
                    ui.label(format!(
                        "Input: throttle {:.0}% · engine {} · {} · stick ({:+.1}, {:+.1}, {:+.1})",
                        stats.throttle * 100.0,
                        if stats.engine { "ON" } else { "OFF" },
                        stats
                            .control_mode
                            .map(|mode| mode.label())
                            .unwrap_or("—"),
                        stats.stick[0],
                        stats.stick[1],
                        stats.stick[2],
                    ));
                    ui.add(
                        egui::Slider::new(&mut controls.throttle, 0.0..=1.0).text("Throttle"),
                    );
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut controls.engine, "Engine");
                        ui.checkbox(&mut controls.precision, "Precision");
                        ui.checkbox(&mut controls.sas, "SAS");
                    });
                    ui.horizontal(|ui| {
                        if ui.button("Cycle control law").clicked() {
                            controls.cycle_mode = true;
                        }
                        ui.small("WASDQE stick · Shift/Ctrl slew · X/Z presets · Space engine · C law · T SAS");
                    });
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut controls.rcs, "RCS");
                        ui.checkbox(&mut controls.wheels, "Wheels");
                        ui.checkbox(&mut controls.gear, "Gear");
                        ui.checkbox(&mut controls.chutes, "Chutes");
                    });
                    ui.small(format!(
                        "Systems: RCS {} · wheels {} · gear {} · chutes {} (R/Y/G/P)",
                        if stats.parts.0 { "on" } else { "off" },
                        if stats.parts.1 { "on" } else { "off" },
                        if stats.parts.2 { "DOWN" } else { "UP" },
                        if stats.parts.3 { "ARMED" } else { "safe" },
                    ));
                }
                if controls.vehicle_preview {
                    ui.checkbox(&mut controls.chase, "Chase camera (follow craft attitude)");
                } else {
                    ui.horizontal(|ui| {
                        for (index, label) in
                            thessa_flight_authority::SURVEY_SITE_LABELS.iter().enumerate()
                        {
                            if ui.button(*label).clicked() {
                                controls.survey_site = Some(index);
                            }
                        }
                    });
                    ui.small(format!(
                        "Survey: {} · {:.1} km (F6 cycles sites)",
                        stats.survey_label, stats.survey_distance / 1000.0,
                    ));
                }
                if controls.vehicle_preview {
                    ui.add(
                        egui::Slider::new(&mut controls.distance, 8.0..=180.0)
                            .logarithmic(true)
                            .text("Distance / m"),
                    );
                } else {
                    ui.add(
                        egui::Slider::new(&mut controls.survey_distance, 80.0..=2_000_000.0)
                            .logarithmic(true)
                            .text("Distance / m"),
                    );
                }
                if ui.button("Reset camera").clicked() {
                    controls.reset_camera = true;
                    if controls.vehicle_preview {
                        controls.chase = false;
                    }
                }
                ui.add(
                    egui::Slider::new(&mut controls.exposure_ev100, 1.0..=20.0)
                        .text("Exposure / EV100"),
                );
                ui.checkbox(&mut controls.vsync, "VSync");
                ui.small("Session-only graphics requests; graphics.toml is not modified.");
                ui.separator();
                ui.small("Arrow keys: orbit/zoom when UI does not capture input. Esc: exit.");
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "Migration viewer, not the complete game client.",
                );
            });
        if let Some(fleet) = &stats.fleet {
            egui::Window::new("Fleet | authoritative observer")
                .anchor(egui::Align2::RIGHT_TOP, [-16.0, 16.0])
                .default_width(330.0)
                .resizable(false)
                .show(context, |ui| {
                    ui.small("Additional vehicles · contact inspector / retained body shells");
                    if stats.flight_stopped {
                        ui.colored_label(egui::Color32::RED, "Frozen: server session stopped");
                    } else if fleet.updates == 0 {
                        ui.label("Membership not received (not an empty fleet)");
                    } else if fleet.stale {
                        ui.colored_label(egui::Color32::YELLOW, "Stale membership: rows withheld, not despawned");
                    } else {
                        ui.label(format!("{} secondaries · {} updates · receive age {:.3} s", fleet.rows.len(), fleet.updates, fleet.age_s.unwrap_or(0.0)));
                        ui.small("Each row uses its own sim time/reference-body frame.");
                        if fleet.matched_geometry().is_none() { ui.colored_label(egui::Color32::YELLOW, format!("Waiting for geometry revision {}", fleet.geometry_revision)); }
                        egui::ScrollArea::vertical().max_height(300.0).show(ui, |ui| {
                            for (index, row) in fleet.rows.iter().enumerate() {
                                ui.separator();
                                ui.label(format!("{} · tick {} · t={:.3} s{}", row.vehicle_id, row.tick, row.pose.time().0, if row.stopped { " · stopped" } else { "" }));
                                ui.small(format!("Reference radius {:.3} km", row.pose.relative_position_m().length() / 1000.0));
                                if let Some(geometry) = fleet.matched_geometry().and_then(|g| g.vehicles.iter().find(|g| g.vehicle_id == row.vehicle_id)) {
                                    ui.label(&geometry.display_name);
                                    match &geometry.render {
                                        thessa_flight_net::geometry::RenderAppearance::Absent => { ui.small("No retained appearance mesh"); }
                                        thessa_flight_net::geometry::RenderAppearance::OmittedBudget => { ui.small("Appearance omitted: bounded wire representation"); }
                                        thessa_flight_net::geometry::RenderAppearance::Available(mesh) => { ui.small(format!("{} retained render parts · neutral finish", mesh.parts.len())); }
                                    }
                                    match &geometry.contact {
                                        thessa_flight_net::geometry::ContactGeometry::Absent => { ui.small("No authoritative contact geometry"); }
                                        thessa_flight_net::geometry::ContactGeometry::OmittedBudget => { ui.small("Contact geometry omitted: wire budget exceeded"); }
                                        thessa_flight_net::geometry::ContactGeometry::Available(contact) => {
                                            ui.push_id(row.vehicle_id.0, |ui| { egui::CollapsingHeader::new(format!("{} contact parts · inspect", contact.parts.len())).default_open(index == 0).show(ui, |ui| crate::contact_preview::show(ui, contact)); });
                                        }
                                    }
                                }
                            }
                        });
                    }
                });
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_render_wgpu::terrain::OffscreenTarget;

    fn fleet_with_contact_geometry() -> crate::scene::FleetObserver {
        let geometry = std::sync::Arc::new(thessa_flight_net::geometry::FleetGeometry {
            revision: 1,
            vehicles: vec![
                thessa_flight_net::geometry::FleetVehicleGeometry::from_vehicle(
                    thessa_sim_core::VehicleId::new(1),
                    &thessa_sim_core::X15StarterProfile::new().unwrap().vehicle,
                ),
            ],
        });
        crate::scene::FleetObserver {
            rows: vec![thessa_flight_net::presentation::FleetPose {
                vehicle_id: thessa_sim_core::VehicleId::new(1),
                tick: 120,
                pose: thessa_flight_authority::presentation::FlightPose::new(
                    thessa_sim_core::SimTime(1.0),
                    glam::DVec3::X * 3.2e6,
                    glam::DQuat::IDENTITY,
                )
                .unwrap(),
                stopped: false,
            }],
            updates: 1,
            geometry_revision: 1,
            geometry_updates: 1,
            geometry: Some(geometry),
            ..Default::default()
        }
    }

    fn controls() -> Controls {
        Controls {
            vehicle_preview: false,
            chase: false,
            reset_camera: false,
            survey_site: None,
            survey_distance: 5000.0,
            distance: 30_000.0,
            throttle: 0.0,
            engine: false,
            precision: false,
            sas: true,
            cycle_mode: false,
            rcs: true,
            wheels: true,
            gear: true,
            chutes: false,
            exposure_ev100: 13.0,
            vsync: false,
        }
    }

    #[test]
    fn vehicle_hud_reset_restores_shared_camera_and_preserves_survey() {
        let context = context();
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1280.0, 720.0),
            )),
            ..Default::default()
        };
        // Vehicle preview: the orbit slider is hidden (the shared quaternion
        // orbit has no single-angle widget) and reset restores the owned
        // pilot camera instead of touching survey state.
        let mut controls = controls();
        controls.vehicle_preview = true;
        controls.distance = 32.0;
        widgets(
            &context,
            input,
            &mut controls,
            &Stats::default(),
            "test",
            None,
        )
        .drop_without_applying_deltas();
        assert_eq!(controls.distance, 32.0);
        assert!(!controls.reset_camera);
    }

    #[test]
    fn navball_branch_renders_without_touching_controls() {
        use crate::scene::FlightData;
        let context = context();
        let pixels = thessa_flight_authority::navball::navball_pixels(glam::DVec3::Z);
        let handle = context.load_texture(
            "test-navball",
            egui::ColorImage::from_rgba_unmultiplied([128, 128], &pixels),
            egui::TextureOptions::LINEAR,
        );
        let stats = Stats {
            live_flight: true,
            data: Some(FlightData {
                surface_speed_mps: 180.0,
                vertical_speed_mps: -2.0,
                altitude_datum_m: 602.0,
                altitude_agl_m: 500.0,
                local_up_body: glam::DVec3::Z,
            }),
            ..Stats::default()
        };
        let before = controls();
        let mut controls = before;
        widgets(
            &context,
            input(1280, 720, 1.0),
            &mut controls,
            &stats,
            "test GPU",
            Some(&handle),
        )
        .drop_without_applying_deltas();
        assert_eq!(controls, before);
    }

    fn input(width: u32, height: u32, scale: f32) -> egui::RawInput {
        let mut input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(width as f32 / scale, height as f32 / scale),
            )),
            ..Default::default()
        };
        input
            .viewports
            .get_mut(&egui::ViewportId::ROOT)
            .unwrap()
            .native_pixels_per_point = Some(scale);
        input
    }

    #[test]
    fn passive_hud_preserves_camera_and_only_captures_pointer_over_widgets() {
        let context = context();
        let before = controls();
        let mut controls = before;
        for _ in 0..3 {
            widgets(
                &context,
                input(1280, 720, 1.0),
                &mut controls,
                &Stats::default(),
                "test GPU",
                None,
            )
            .drop_without_applying_deltas();
        }
        // Merely showing the HUD must not edit either camera or graphics requests.
        assert_eq!(controls, before);
        context.fonts_mut(|fonts| {
            for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                assert!(fonts.has_glyphs(
                    &egui::FontId::new(14.0, family),
                    "Thessa 0123456789 · Орбита"
                ));
            }
        });
        let mut hover = input(1280, 720, 1.0);
        hover
            .events
            .push(egui::Event::PointerMoved(egui::pos2(50.0, 80.0)));
        widgets(
            &context,
            hover,
            &mut controls,
            &Stats::default(),
            "test GPU",
            None,
        )
        .drop_without_applying_deltas();
        assert!(context.egui_wants_pointer_input());
        let mut outside = input(1280, 720, 1.0);
        outside
            .events
            .push(egui::Event::PointerMoved(egui::pos2(1200.0, 680.0)));
        widgets(
            &context,
            outside,
            &mut controls,
            &Stats::default(),
            "test GPU",
            None,
        )
        .drop_without_applying_deltas();
        assert!(!context.egui_wants_pointer_input());
        assert!(!context.egui_wants_keyboard_input());
        assert_eq!(controls, before);
    }

    #[test]
    #[ignore = "requires a wgpu adapter; run on development hardware"]
    fn shared_device_hud_loads_existing_color_across_resize_dpi_and_recreation() {
        let backend = Backend::bring_up().expect("hardware HUD regression needs an adapter");
        // Recreate the complete egui context/renderer as window suspension does.
        for fleet in [false, true] {
            for _ in 0..2 {
                let mut hud = Hud::new(&backend, OffscreenTarget::FORMAT);
                for (width, height, scale) in [(800, 600, 1.0), (1280, 900, 1.5), (960, 720, 1.0)] {
                    let target = OffscreenTarget::new(&backend, width, height).unwrap();
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    {
                        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &target.color,
                                resolve_target: None,
                                depth_slice: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color {
                                        r: 0.04,
                                        g: 0.1,
                                        b: 0.2,
                                        a: 1.0,
                                    }),
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            ..Default::default()
                        });
                    }
                    backend.queue.submit([encoder.finish()]);
                    let before = target.read_rgba8(&backend).unwrap();
                    let mut controls = controls();
                    for _ in 0..3 {
                        let output = hud.run(
                            input(width, height, scale),
                            &mut controls,
                            &Stats {
                                fleet: fleet.then(fleet_with_contact_geometry),
                                ..Default::default()
                            },
                        );
                        assert_eq!(output.pixels_per_point, scale);
                        let mut encoder =
                            backend.device.create_command_encoder(&Default::default());
                        let mut commands = hud.encode(
                            &backend,
                            &mut encoder,
                            &target.color,
                            output,
                            [width, height],
                        );
                        commands.push(encoder.finish());
                        backend.queue.submit(commands);
                    }
                    let after = target.read_rgba8(&backend).unwrap();
                    assert!(
                        before
                            .as_chunks::<4>()
                            .0
                            .iter()
                            .zip(after.as_chunks::<4>().0.iter())
                            .filter(|(a, b)| a != b)
                            .count()
                            > 500
                    );
                    // An accidental Clear/DontCare, incorrect target or fullscreen
                    // blit would destroy the terrain outside the overlay.
                    // The right-anchored inspector legitimately casts a shadow
                    // near the top/right edge. Keep the original full-column
                    // oracle without it; with it, test a 32x32 lower-right patch
                    // outside BOTH windows and shadows, without a color tolerance.
                    let ys = if fleet {
                        height - 32..height
                    } else {
                        0..height
                    };
                    let xs = if fleet {
                        width - 32..width
                    } else {
                        width - 1..width
                    };
                    for y in ys {
                        for x in xs.clone() {
                            let offset = ((y * width + x) * 4) as usize;
                            assert_eq!(&before[offset..offset + 4], &after[offset..offset + 4]);
                        }
                    }
                }
            }
        }
    }
}
