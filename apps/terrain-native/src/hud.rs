//! App-owned egui HUD. The GPU pass loads the terrain's existing color target;
//! it never imports/copies the scene or creates another GPU device.
use thessa_render_wgpu::Backend;
#[path = "fastnavball.rs"]
mod fastnavball;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Controls {
    pub map_open: bool,
    pub instrument_layout: thessa_graphics::InstrumentLayout,
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
    pub paused: bool,
    pub warp: f64,
    pub reset_flight: bool,
    pub exposure_ev100: f32,
    pub vsync: bool,
}

impl Controls {
    pub fn from_scene(scene: &crate::scene::Scene) -> Self {
        Self {
            map_open: scene.map_active(),
            instrument_layout: scene.requested.instruments.layout,
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
            paused: scene
                .flight_clock()
                .is_some_and(|clock| clock.requested_paused),
            warp: scene
                .flight_clock()
                .map_or(1.0, |clock| clock.requested_warp),
            reset_flight: false,
            exposure_ev100: scene.requested.renderer.exposure_ev100,
            vsync: scene.requested.renderer.vsync,
        }
    }

    /// Graphics edits remain requests and must go through capability resolution.
    pub fn apply(self, scene: &mut crate::scene::Scene) {
        if self.map_open != scene.map_active() {
            scene.set_map_open(self.map_open);
        }
        scene.requested.instruments.layout = self.instrument_layout;
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
            scene.request_pause(self.paused);
            if let Err(error) = scene.request_warp(self.warp) {
                eprintln!("{error}");
            }
            if self.reset_flight {
                if let Err(error) = scene.reset_flight() {
                    eprintln!("{error}");
                }
                return;
            }
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
    pub map: Option<crate::map::Snapshot>,
    pub clock: Option<crate::scene::FlightClock>,
    pub instruments: thessa_graphics::ResolvedInstruments,
    pub fleet: Option<crate::scene::FleetObserver>,
    pub live_flight: bool,
    pub server_snapshots: Option<u64>,
    pub primary_appearance: &'static str,
    pub flight_stopped: bool,
    pub chase: bool,
    pub throttle: f64,
    pub engine: bool,
    pub control_mode: Option<thessa_flight_authority::ControlMode>,
    pub stick: [f64; 3],
    pub parts: (bool, bool, bool, bool),
    pub data: Option<crate::scene::FlightData>,
    pub display_body_name: Option<String>,
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
        let data = scene.flight_data();
        Self {
            map: scene.map_snapshot(),
            instruments: thessa_graphics::resolve_instruments(scene.requested.instruments),
            clock: scene.flight_clock(),
            submitted_frames: frames,
            live_flight: scene.live_flight(),
            server_snapshots: scene.server_snapshot_count(),
            primary_appearance: scene.primary_appearance_status(),
            fleet: scene.fleet_observer().cloned(),
            flight_stopped: scene.flight_stopped(),
            chase: scene.camera.chase(),
            throttle: scene.input.throttle,
            engine: scene.input.engine_active,
            control_mode: scene.live_flight().then_some(scene.control_mode),
            stick: scene.control_axes().to_array(),
            parts: scene.part_states().unwrap_or((true, true, true, false)),
            data,
            display_body_name: data
                .and_then(|data| scene.body_name(data.display_body).map(str::to_owned)),
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
    pub view: View,
    pub context: egui::Context,
    renderer: egui_wgpu::Renderer,
    adapter: String,
    format: wgpu::TextureFormat,
}

impl Hud {
    pub fn new(backend: &Backend, format: wgpu::TextureFormat) -> Self {
        let context = context();
        Self {
            view: View::default(),
            context,
            renderer: egui_wgpu::Renderer::new(
                &backend.device,
                format,
                egui_wgpu::RendererOptions::default(),
            ),
            adapter: backend.adapter_name.clone(),
            format,
        }
    }

    pub fn run(
        &mut self,
        input: egui::RawInput,
        controls: &mut Controls,
        stats: &Stats,
    ) -> egui::FullOutput {
        let navball = stats
            .data
            .filter(|_| {
                stats.live_flight
                    && stats.instruments.enabled
                    && !controls.map_open
                    && self.view.visible
                    && !self.view.help
            })
            .map(|data| fastnavball::Paint {
                frame: thessa_flight_authority::navball::InstrumentFrame::new(
                    data.local_up_body,
                    data.pole_body,
                    data.surface_velocity_body,
                ),
                settings: stats.instruments,
                format: self.format,
            });
        if navball.is_none() {
            self.renderer
                .callback_resources
                .remove::<fastnavball::FastNavball>();
        }
        widgets(
            &self.context,
            input,
            controls,
            stats,
            &self.adapter,
            navball,
            &mut self.view,
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

pub(super) fn context() -> egui::Context {
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    Help,
    Visibility,
    Telemetry,
}

pub struct View {
    pub visible: bool,
    pub help: bool,
    pub telemetry: bool,
    map: crate::map::View,
    speed_frame: thessa_flight_authority::instruments::SpeedFrame,
    altitude_frame: thessa_flight_authority::instruments::AltitudeFrame,
}
impl Default for View {
    fn default() -> Self {
        Self {
            visible: true,
            help: false,
            telemetry: true,
            map: Default::default(),
            speed_frame: Default::default(),
            altitude_frame: Default::default(),
        }
    }
}
impl View {
    pub fn apply(&mut self, action: Action) {
        match action {
            Action::Help => {
                self.help = !self.visible || !self.help;
                self.visible = true;
            }
            Action::Visibility => self.visible = !self.visible,
            Action::Telemetry => self.telemetry = !self.telemetry,
        }
    }
    pub fn blocks_scene_input(&self) -> bool {
        self.visible && self.help
    }
}

fn help(context: &egui::Context, open: &mut bool) {
    egui::Window::new("Thessa | controls (F1)").open(open)
        .order(egui::Order::Foreground).anchor(egui::Align2::CENTER_CENTER,[0.0,0.0])
        .default_width(600.0).resizable(true).show(context,|ui| {
            egui::ScrollArea::vertical().max_height(500.0).show(ui,|ui| {
                ui.heading("Native scene and flight controls");
                for (keys,action) in [
                    ("F1 / F2 / F3","Help / flight HUD visibility / telemetry and fleet panels"),
                    ("M","System map / scene; requires enabled HUD"),
                    ("W S A D Q E","Manual pitch / yaw / roll; takes over Mouse Aim"),
                    ("Shift / Ctrl, X / Z","Throttle slew, cut / full"),
                    ("Space, CapsLock","Engine intent, precision controls"),
                    ("C, T, hold F","Control law, SAS base demand, temporary SAS inversion"),
                    ("R Y G P","RCS / reaction wheels / landing gear / parachutes"),
                    ("F8, [ / ], Backspace","Pause vote, warp half / double, canonical relaunch without rewinding time"),
                    ("Arrows, V, Backquote","Scene camera orbit / zoom, chase toggle, reset camera"),
                    ("Right / middle drag, wheel","Scene orbit / vehicle pan, zoom (outside widgets)"),
                    ("F6, Shift + right drag","Survey site cycle, survey free look"),
                    ("Map canvas","Right-drag orbit, middle-drag pan, wheel zoom; selector/click selects; Frame body / PRIMARY follows"),
                    ("Escape","Exit scene when input is not captured; use M or the close button to leave the map"),
                ] {ui.horizontal_wrapped(|ui| {ui.strong(keys);ui.label(action);});}
                ui.separator();
                ui.label("Help owns scene/flight input but does not pause the authority or cut thrust. Close it with F1 or the window close button. F2 hides the flight HUD, not the system map; F1 always restores visible help.");
                ui.label("Mouse Aim uses a 6% cursor deadband. Camera gestures and widgets do not steer. Focus loss releases manual input and temporary SAS inversion without changing base SAS/throttle/engine intent.");
                ui.label("Server requests are not acknowledgements. A lost link freezes the last presentation; it never starts local physics. Remote rails/force data are not fabricated. This viewer is not complete client parity.");
            });
        });
}

fn widgets(
    context: &egui::Context,
    input: egui::RawInput,
    controls: &mut Controls,
    stats: &Stats,
    adapter: &str,
    navball: Option<fastnavball::Paint>,
    view: &mut View,
) -> egui::FullOutput {
    context.run_ui(input, |root| {
        let context = root.ctx();
        if view.visible && view.help {help(context,&mut view.help);return;}
        if controls.map_open && let Some(data)=&stats.map {
            view.map.show(context,&mut controls.map_open,data);
            return;
        }
        if !view.visible {return;}
        if view.telemetry {
        egui::Window::new("Thessa | terrain migration")
            .default_open(!stats.live_flight)
            .default_pos([16.0, 16.0])
            .default_width(300.0)
            .resizable(false)
            .show(context, |ui| {
                ui.label("Owned renderer · shared-device egui HUD");
                ui.small("F1 help · F2 HUD · F3 telemetry");
                if ui.button("System map (M)").clicked() {controls.map_open=true;}
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
                        ui.small(format!("PRIMARY appearance: {}", stats.primary_appearance));
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
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut controls.paused, "Pause vote (F8)");
                        if ui.button("Relaunch (Backspace)").clicked() { controls.reset_flight = true; }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Warp request");
                        ui.add(egui::DragValue::new(&mut controls.warp).range(0.0..=thessa_flight_net::MAX_TIME_WARP).speed(0.125));
                        if ui.button("½ [").clicked() { controls.warp = (controls.warp * 0.5).max(0.125); }
                        if ui.button("2× ]").clicked() { controls.warp = (controls.warp.max(0.125) * 2.0).min(thessa_flight_net::MAX_TIME_WARP); }
                    });
                    if let Some(clock) = stats.clock {
                        ui.small(format!("Authority paused: {:?} · observed warp: {:?}", clock.authority_paused, clock.effective_warp));
                    }
                    if let Some(data) = stats.data {
                        ui.label(format!(
                            "Speed {:.1} m/s · Alt {:.0} m (AGL {} m) · V/S {:+.1} m/s",
                            data.surface_speed_mps,
                            data.altitude_datum_m,
                            optional(data.altitude_agl_m, 0),
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
                        ui.checkbox(&mut controls.sas, "SAS").on_hover_text("Base demand; hold F to temporarily invert. Releasing focus/key restores this demand. SAS is not acknowledged by the lean server snapshot.");
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
        }
        if let (Some(navball),Some(data))=(navball,stats.data){flight_deck(context,navball,data,controls,stats,view);}
        if view.telemetry && let Some(fleet) = &stats.fleet {
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
                                        thessa_flight_net::geometry::RenderAppearance::StarterX15 { .. } => { ui.small("Explicit starter asset"); }
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

fn flight_deck(
    context: &egui::Context,
    navball: fastnavball::Paint,
    data: crate::scene::FlightData,
    controls: &mut Controls,
    stats: &Stats,
    view: &mut View,
) {
    use thessa_flight_authority::instruments::{AltitudeFrame, SpeedFrame};
    use thessa_graphics::InstrumentLayout;
    let mut navball = navball;
    let screen = context.content_rect();
    let side = (navball.settings.diameter_points as f32)
        .min((screen.height() - 85.0).max(80.0))
        .min((screen.width() - 244.0).max(64.0));
    egui::Area::new(egui::Id::unique("native-flight-deck"))
        .anchor(egui::Align2::CENTER_BOTTOM, [0.0, -10.0])
        .show(context, |ui| {
            egui::Frame::new()
                .fill(egui::Color32::from_black_alpha(225))
                .corner_radius(12.0)
                .inner_margin(10.0)
                .show(ui, |ui| {
                    ui.set_max_width((screen.width() - 40.0).max(280.0));
                    ui.horizontal(|ui| {
                        ui.selectable_value(
                            &mut controls.instrument_layout,
                            InstrumentLayout::Aircraft,
                            "AIRCRAFT",
                        );
                        ui.selectable_value(
                            &mut controls.instrument_layout,
                            InstrumentLayout::Rocket,
                            "FDAI",
                        );
                        egui::ComboBox::from_id_salt("instrument-speed-frame").selected_text(view.speed_frame.label()).show_ui(ui, |ui| {
                            for frame in SpeedFrame::ALL {ui.selectable_value(&mut view.speed_frame, frame, frame.label());}
                        });
                        ui.selectable_value(&mut view.altitude_frame, AltitudeFrame::Datum, "DATUM");
                        ui.selectable_value(&mut view.altitude_frame, AltitudeFrame::Agl, "AGL");
                        ui.checkbox(&mut controls.paused, "PAUSE");
                        if ui.small_button("RELAUNCH").clicked() {
                            controls.reset_flight = true;
                        }
                        if stats.flight_stopped {
                            ui.colored_label(egui::Color32::RED, "DATA FROZEN");
                        }
                    });
                    navball.frame.prograde = data.velocity_body(view.speed_frame).and_then(|v| v.try_normalize());
                    ui.small(format!("Frame: {} · local attitude / {} velocity cues{}",stats.display_body_name.as_deref().unwrap_or("body-relative"),view.speed_frame.label(),if data.at_reference_body {""} else {" · rotation provider unavailable; SURFACE = body-relative"}));
                    if let Some(clock) = stats.clock {
                        ui.small(format!(
                            "T+{:.3}s · request x{:.3} · authority pause {:?}, rate {:?}",
                            stats.sim_time_s,
                            clock.requested_warp,
                            clock.authority_paused,
                            clock.effective_warp
                        ));
                    }
                    ui.horizontal(|ui| {
                        ui.vertical(|ui| {
                            ui.set_width(100.0);
                            if navball.settings.layout == InstrumentLayout::Aircraft {
                                instrument_readout(
                                    ui,
                                    view.speed_frame.label(),
                                    optional(data.speed(view.speed_frame), 1),
                                );
                                if let Some(speed) = data.speed(view.speed_frame) { tape(ui, speed, 10.0, "m/s"); }
                            } else {
                                instrument_readout(
                                    ui,
                                    view.speed_frame.label(),
                                    optional(data.speed(view.speed_frame), 1),
                                );
                                instrument_readout(
                                    ui,
                                    "SURF VERT m/s",
                                    format!("{:+.1}", data.vertical_speed_mps),
                                );
                                let horizontal = (data.surface_speed_mps.powi(2)
                                    - data.vertical_speed_mps.powi(2))
                                .max(0.0)
                                .sqrt();
                                instrument_readout(ui, "SURF HORIZ m/s", format!("{horizontal:.1}"));
                            }
                            ui.separator();
                            ui.small(format!("THR {:03.0}%", stats.throttle * 100.0));
                            ui.add(
                                egui::ProgressBar::new(stats.throttle as f32).desired_width(90.0),
                            );
                            ui.checkbox(&mut controls.engine, "ENGINE");
                            ui.checkbox(&mut controls.sas, "SAS").on_hover_text("Base demand; hold F to temporarily invert. Releasing focus/key restores this demand. SAS is not acknowledged by the lean server snapshot.");
                        });
                        ui.vertical(|ui| {
                            navball.show(ui, side);
                            let heading = navball
                                .frame
                                .heading_rad
                                .map(|a| format!("HDG {:03.0}°", a.to_degrees().rem_euclid(360.0)))
                                .unwrap_or_else(|| "HDG —".into());
                            ui.horizontal(|ui| {
                                ui.monospace(heading);
                                ui.small("PRO / RETRO");
                            });
                        });
                        ui.vertical(|ui| {
                            ui.set_width(100.0);
                            instrument_readout(
                                ui,
                                    if view.altitude_frame == AltitudeFrame::Datum {"DATUM m"} else {"AGL m"},
                                    optional(data.altitude(view.altitude_frame), 0),
                            );
                            if navball.settings.layout == InstrumentLayout::Aircraft
                                && let Some(altitude) = data.altitude(view.altitude_frame) {
                                tape(ui, altitude, 100.0, "m");
                            }
                            instrument_readout(ui, "AGL m", optional(data.altitude_agl_m, 0));
                            if navball.settings.layout == InstrumentLayout::Aircraft {
                                instrument_readout(
                                    ui,
                                    "V/S m/s",
                                    format!("{:+.1}", data.vertical_speed_mps),
                                );
                            } else {
                                instrument_readout(
                                    ui,
                                    "PITCH deg",
                                    format!(
                                        "{:+.1}",
                                        navball.frame.up.x.clamp(-1.0, 1.0).asin().to_degrees()
                                    ),
                                );
                                let roll = if navball.frame.up.y.hypot(navball.frame.up.z) > 1e-6 {
                                    format!(
                                        "{:+.1}",
                                        navball.frame.up.y.atan2(navball.frame.up.z).to_degrees()
                                    )
                                } else {
                                    "—".into()
                                };
                                instrument_readout(ui, "ROLL deg", roll);
                            }
                            ui.separator();
                            ui.checkbox(&mut controls.rcs, "RCS");
                            if navball.settings.layout == InstrumentLayout::Aircraft {
                                ui.checkbox(&mut controls.gear, "GEAR");
                            } else {
                                ui.checkbox(&mut controls.wheels, "R/WHEELS");
                            }
                            ui.small(format!("SIM {:.1} s", stats.sim_time_s));
                        });
                    });
                    flight_readouts(ui, data);
                });
        });
}

fn optional(value: Option<f64>, digits: usize) -> String {
    value
        .filter(|v| v.is_finite())
        .map_or_else(|| "—".into(), |v| format!("{v:.digits$}"))
}

fn flight_readouts(ui: &mut egui::Ui, data: crate::scene::FlightData) {
    ui.small(format!(
        "Mach {} · q {} Pa · AoA {}° · beta {}°",
        optional(data.air.and_then(|a| a.mach), 2),
        optional(data.air.and_then(|a| a.dynamic_pressure_pa), 0),
        optional(data.air.map(|a| a.angle_of_attack_deg), 1),
        optional(data.air.map(|a| a.sideslip_deg), 1)
    ));
    ui.small(format!(
        "Thrust {} N · TWR {} (central-body g) · load {} g (non-gravitational)",
        optional(data.loads.map(|l| l.thrust_n), 0),
        optional(data.loads.map(|l| l.twr), 2),
        optional(data.loads.map(|l| l.g_load), 2)
    ));
    egui::CollapsingHeader::new("Air / orbit / subsystem details").show(ui, |ui| {
        egui::ScrollArea::vertical().max_height(100.0).show(ui, |ui| {
            if let Some(air) = data.air {
                ui.small(format!("Air {} m/s · p {} Pa · density {} kg/m³", optional(Some(air.velocity_body_mps.length()),1), optional(air.pressure_pa,0), optional(air.density_kg_m3,4)));
            } else { ui.small("Air unavailable: no matching local atmosphere observation; replica model loads are not server observations."); }
            if data.loads.is_none() {ui.small("Thrust / TWR / g-load unavailable: not in the lean server snapshot or no local force sample yet.");}
            ui.small(format!("Instantaneous two-body: Ap {} km · Pe {} km (not a JIT path or solved impact)", optional(data.apoapsis_altitude_m.map(|v|v/1000.0),1), optional(data.periapsis_altitude_m.map(|v|v/1000.0),1)));
            ui.small(format!("Reaction-wheel torque {:?} N·m · latest authority sample", data.reaction_wheel_torque_body_nm.to_array()));
            match data.parachutes {
                Some(p) if p.installed == 0 => {ui.small("Parachutes: none installed / transmitted");}
                Some(p) => {ui.small(format!("Parachutes: {} armed / {} open / {} failed · {:.0} N · q {:.0} Pa",p.armed,p.open,p.failed,p.drag_n,p.dynamic_pressure_pa));}
                None => {ui.small("Parachute observation unavailable or rejected; no replica-local substitute");}
            }
        });
    });
}

fn instrument_readout(ui: &mut egui::Ui, label: &str, value: String) {
    ui.small(label);
    ui.label(
        egui::RichText::new(value)
            .monospace()
            .size(20.0)
            .color(egui::Color32::from_rgb(195, 240, 210)),
    );
}

fn tape(ui: &mut egui::Ui, value: f64, step: f64, unit: &str) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(88.0, 100.0), egui::Sense::hover());
    let painter = ui.painter();
    let base = (value / step).floor() * step;
    for i in -3..=3 {
        let mark = base + i as f64 * step;
        let y = rect.center().y - ((mark - value) / step) as f32 * 20.0;
        if y < rect.top() + 6.0 || y > rect.bottom() - 6.0 {
            continue;
        }
        painter.text(
            egui::pos2(rect.left() + 4.0, y),
            egui::Align2::LEFT_CENTER,
            format!("{mark:.0}"),
            egui::FontId::monospace(12.0),
            egui::Color32::LIGHT_GRAY,
        );
        painter.line_segment(
            [
                egui::pos2(rect.right() - 12.0, y),
                egui::pos2(rect.right(), y),
            ],
            egui::Stroke::new(1.0, egui::Color32::GRAY),
        );
    }
    painter.line_segment(
        [
            egui::pos2(rect.left(), rect.center().y),
            egui::pos2(rect.right(), rect.center().y),
        ],
        egui::Stroke::new(1.0, egui::Color32::LIGHT_GREEN),
    );
    ui.small(unit);
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_render_wgpu::terrain::OffscreenTarget;

    fn instrument_fixture(vertical_speed_mps: f64) -> crate::scene::FlightData {
        use thessa_flight_authority::{
            FlightAuthority,
            instruments::{self, Source},
        };
        let config: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = config.bake().unwrap();
        let flight = FlightAuthority::new(&e, e.body_id("thessa").unwrap()).unwrap();
        crate::scene::FlightData {
            surface_speed_mps: 180.0,
            vertical_speed_mps,
            altitude_datum_m: 602.0,
            altitude_agl_m: Some(500.0),
            local_up_body: glam::DVec3::Z,
            pole_body: glam::DVec3::X,
            surface_velocity_body: glam::DVec3::X * 180.0,
            ..instruments::sample(&flight, &e, Source::LocalAuthority).unwrap()
        }
    }

    #[test]
    fn display_frames_are_session_only_and_unavailable_readouts_stay_explicit() {
        use thessa_flight_authority::instruments::{AltitudeFrame, SpeedFrame};
        let context = context();
        let before = controls();
        let mut controls = before;
        let data = instrument_fixture(2.0);
        let stats = Stats {
            live_flight: true,
            data: Some(data),
            display_body_name: Some("NEREID".into()),
            ..Default::default()
        };
        let paint = fastnavball::Paint {
            frame: data.attitude(),
            settings: stats.instruments,
            format: OffscreenTarget::FORMAT,
        };
        for frame in SpeedFrame::ALL {
            let mut view = View {
                speed_frame: frame,
                altitude_frame: AltitudeFrame::Agl,
                ..Default::default()
            };
            for _ in 0..3 {
                widgets(
                    &context,
                    input(800, 600, 1.0),
                    &mut controls,
                    &stats,
                    "test GPU",
                    Some(paint),
                    &mut view,
                )
                .drop_without_applying_deltas();
            }
            assert_eq!(view.speed_frame, frame);
            assert_eq!(view.altitude_frame, AltitudeFrame::Agl);
            assert_eq!(
                controls, before,
                "instrument choices must not write flight demands or graphics settings"
            );
        }
        assert_eq!(optional(None, 1), "—");
        assert_eq!(optional(Some(f64::NAN), 1), "—");
        assert_eq!(optional(Some(f64::INFINITY), 1), "—");
    }

    fn fleet_with_contact_geometry() -> crate::scene::FleetObserver {
        let geometry = std::sync::Arc::new(thessa_flight_net::geometry::FleetGeometry {
            primary: None,
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
            map_open: false,
            instrument_layout: thessa_graphics::InstrumentLayout::Aircraft,
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
            paused: false,
            warp: 1.0,
            reset_flight: false,
            exposure_ev100: 13.0,
            vsync: false,
        }
    }

    #[test]
    fn visibility_and_help_are_presentation_only_and_hidden_hud_has_no_meshes() {
        let context = context();
        let mut view = View::default();
        let mut controls = controls();
        let before = controls;
        view.apply(Action::Help);
        assert!(view.blocks_scene_input());
        view.apply(Action::Visibility);
        assert!(!view.blocks_scene_input());
        for _ in 0..3 {
            let output = widgets(
                &context,
                input(1280, 720, 1.0),
                &mut controls,
                &Stats::default(),
                "test",
                None,
                &mut view,
            );
            let empty = context
                .tessellate(output.shapes.clone(), output.pixels_per_point)
                .is_empty();
            output.drop_without_applying_deltas();
            assert!(empty);
        }
        view.apply(Action::Help);
        assert!(view.visible && view.help);
        for _ in 0..3 {
            widgets(
                &context,
                input(1280, 720, 1.0),
                &mut controls,
                &Stats::default(),
                "test",
                None,
                &mut view,
            )
            .drop_without_applying_deltas();
        }
        let output = widgets(
            &context,
            input(1280, 720, 1.0),
            &mut controls,
            &Stats::default(),
            "test",
            None,
            &mut view,
        );
        let empty = context
            .tessellate(output.shapes.clone(), output.pixels_per_point)
            .is_empty();
        output.drop_without_applying_deltas();
        assert!(!empty);
        view.apply(Action::Help);
        assert!(!view.blocks_scene_input());
        view.apply(Action::Telemetry);
        assert!(!view.telemetry);
        assert_eq!(
            controls, before,
            "visibility must not edit flight/camera/graphics requests"
        );
    }

    #[test]
    fn hiding_flight_hud_never_hides_the_map_or_changes_its_input_ownership() {
        let context = context();
        let mut view = View::default();
        view.apply(Action::Visibility);
        let mut scene = crate::scene::Scene::load(Default::default()).unwrap();
        scene.set_map_open(true);
        let mut controls = Controls::from_scene(&scene);
        let before = controls;
        for _ in 0..3 {
            widgets(
                &context,
                input(1280, 800, 1.0),
                &mut controls,
                &Stats::from_scene(&scene, 0, 0.0),
                "test",
                None,
                &mut view,
            )
            .drop_without_applying_deltas();
        }
        let output = widgets(
            &context,
            input(1280, 800, 1.0),
            &mut controls,
            &Stats::from_scene(&scene, 0, 0.0),
            "test",
            None,
            &mut view,
        );
        let visible = !context
            .tessellate(output.shapes.clone(), output.pixels_per_point)
            .is_empty();
        output.drop_without_applying_deltas();
        assert!(visible);
        assert!(scene.map_active());
        assert_eq!(controls, before);
        assert!(!view.visible && !view.blocks_scene_input());
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
            &mut View::default(),
        )
        .drop_without_applying_deltas();
        assert_eq!(controls.distance, 32.0);
        assert!(!controls.reset_camera);
    }

    #[test]
    fn navball_branch_renders_without_touching_controls() {
        let context = context();
        let navball = fastnavball::Paint {
            frame: thessa_flight_authority::navball::InstrumentFrame::new(
                glam::DVec3::Z,
                glam::DVec3::X,
                glam::DVec3::X,
            ),
            settings: Default::default(),
            format: OffscreenTarget::FORMAT,
        };
        let stats = Stats {
            live_flight: true,
            data: Some(instrument_fixture(-2.0)),
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
            Some(navball),
            &mut View::default(),
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
                &mut View::default(),
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
            &mut View::default(),
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
            &mut View::default(),
        )
        .drop_without_applying_deltas();
        assert!(!context.egui_wants_pointer_input());
        assert!(!context.egui_wants_keyboard_input());
        assert_eq!(controls, before);
    }

    #[test]
    #[ignore = "requires a wgpu adapter; run on development hardware"]
    fn gpu_flight_deck_shares_target_handles_dpi_and_releases_disabled_resources() {
        use thessa_graphics::{InstrumentLayout, InstrumentSettings, NavballRenderer};
        let backend = Backend::bring_up().unwrap();
        for (layout, renderer) in [
            (InstrumentLayout::Aircraft, NavballRenderer::Analytic),
            (InstrumentLayout::Rocket, NavballRenderer::Baked),
        ] {
            let mut hud = Hud::new(&backend, OffscreenTarget::FORMAT);
            let before = controls();
            let mut controls = before;
            let mut stats = Stats {
                live_flight: true,
                instruments: thessa_graphics::resolve_instruments(InstrumentSettings {
                    layout,
                    navball_renderer: renderer,
                    ..Default::default()
                }),
                data: Some(instrument_fixture(2.0)),
                ..Default::default()
            };
            use thessa_flight_authority::instruments::{AltitudeFrame, SpeedFrame};
            for (width, height, scale, speed, altitude) in [
                (800, 600, 1.0, SpeedFrame::Surface, AltitudeFrame::Datum),
                (1280, 900, 1.5, SpeedFrame::Orbital, AltitudeFrame::Datum),
                (800, 600, 1.0, SpeedFrame::Air, AltitudeFrame::Agl),
                (1280, 900, 1.5, SpeedFrame::Target, AltitudeFrame::Agl),
            ] {
                hud.view.speed_frame = speed;
                hud.view.altitude_frame = altitude;
                if speed == SpeedFrame::Target {
                    let data = stats.data.as_mut().unwrap();
                    data.air = None;
                    data.loads = None;
                    data.altitude_agl_m = None;
                }
                let target = OffscreenTarget::new(&backend, width, height).unwrap();
                for _ in 0..3 {
                    let output = hud.run(input(width, height, scale), &mut controls, &stats);
                    let mut encoder = backend.device.create_command_encoder(&Default::default());
                    {
                        let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &target.color,
                                resolve_target: None,
                                depth_slice: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            ..Default::default()
                        });
                    }
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
                assert!(
                    hud.renderer
                        .callback_resources
                        .get::<fastnavball::FastNavball>()
                        .is_some()
                );
                let rgba = target.read_rgba8(&backend).unwrap();
                assert!(
                    rgba.as_chunks::<4>()
                        .0
                        .iter()
                        .filter(|p| p[0] > 80 || p[1] > 80 || p[2] > 80)
                        .count()
                        > 1000
                );
                // Outside both the collapsed debug header and bottom deck.
                assert_eq!(
                    &rgba[((height / 2 * width + width - 1) * 4) as usize..][..4],
                    &[0, 0, 0, 255]
                );
                assert_eq!(
                    controls, before,
                    "passive presentation must not change flight commands"
                );
            }
            hud.view.visible = false;
            let output = hud.run(input(800, 600, 1.0), &mut controls, &stats);
            assert!(
                hud.context
                    .tessellate(output.shapes.clone(), output.pixels_per_point)
                    .is_empty()
            );
            output.drop_without_applying_deltas();
            assert!(
                hud.renderer
                    .callback_resources
                    .get::<fastnavball::FastNavball>()
                    .is_none()
            );
            hud.view.visible = true;
            hud.view.help = true;
            let output = hud.run(input(800, 600, 1.0), &mut controls, &stats);
            output.drop_without_applying_deltas();
            assert!(
                hud.renderer
                    .callback_resources
                    .get::<fastnavball::FastNavball>()
                    .is_none()
            );
            hud.view.help = false;
            stats.instruments.enabled = false;
            hud.run(input(800, 600, 1.0), &mut controls, &stats)
                .drop_without_applying_deltas();
            assert!(
                hud.renderer
                    .callback_resources
                    .get::<fastnavball::FastNavball>()
                    .is_none()
            );
        }
    }

    #[test]
    #[ignore = "requires a wgpu adapter; run on development hardware"]
    fn shared_device_hud_loads_existing_color_across_resize_dpi_and_recreation() {
        let backend = Backend::bring_up().expect("hardware HUD regression needs an adapter");
        // Recreate the complete egui context/renderer as window suspension does.
        let mut map_scene = crate::scene::Scene::load(Default::default()).unwrap();
        map_scene.set_map_open(true);
        for (fleet, map) in [(false, false), (true, false), (false, true)] {
            for _ in 0..2 {
                let mut hud = Hud::new(&backend, OffscreenTarget::FORMAT);
                let sizes = if map {
                    [(1280, 900, 1.0), (1920, 1440, 1.5), (1400, 1000, 1.0)]
                } else {
                    [(800, 600, 1.0), (1280, 900, 1.5), (960, 720, 1.0)]
                };
                for (width, height, scale) in sizes {
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
                    controls.map_open = map;
                    for _ in 0..3 {
                        let output = hud.run(
                            input(width, height, scale),
                            &mut controls,
                            &Stats {
                                map: map.then(|| map_scene.map_snapshot().unwrap()),
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
                    let ys = if fleet || map {
                        height - 32..height
                    } else {
                        0..height
                    };
                    let xs = if fleet || map {
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

    #[test]
    #[ignore = "hardware: display-referred egui colors must not be tone mapped with the HDR world"]
    fn hdr_display_precedes_ui_without_changing_opaque_ui_colors() {
        let backend = Backend::bring_up().unwrap();
        let mut reference = None;
        for enabled in [false, true] {
            let mut requested = thessa_graphics::RequestedGraphics::default();
            requested.renderer.hdr = enabled;
            let settings = backend.resolve_graphics(&requested);
            let display = thessa_render_wgpu::display::DisplayTransform::new(
                &backend,
                &settings,
                OffscreenTarget::FORMAT,
                320,
                200,
            )
            .unwrap();
            let target = OffscreenTarget::new(&backend, 320, 200).unwrap();
            let mut hud = Hud::new(&backend, OffscreenTarget::FORMAT);
            let output = hud.context.run_ui(input(320, 200, 1.0), |ui| {
                let paint = ui.ctx().layer_painter(egui::LayerId::new(
                    egui::Order::Foreground,
                    egui::Id::unique("display-ui-oracle"),
                ));
                for (i, color) in [
                    egui::Color32::WHITE,
                    egui::Color32::from_rgb(128, 64, 32),
                    egui::Color32::RED,
                ]
                .into_iter()
                .enumerate()
                {
                    paint.rect_filled(
                        egui::Rect::from_min_size(
                            egui::pos2(20.0 + i as f32 * 80.0, 20.0),
                            egui::vec2(60.0, 60.0),
                        ),
                        0.0,
                        color,
                    );
                }
            });
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
                let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: display.scene_color(&target.color),
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    ..Default::default()
                });
            }
            display.encode(&mut encoder, &target.color);
            let mut commands =
                hud.encode(&backend, &mut encoder, &target.color, output, [320, 200]);
            commands.push(encoder.finish());
            backend.queue.submit(commands);
            let pixels = target.read_rgba8(&backend).unwrap();
            assert_eq!(
                &pixels[((40 * 320 + 40) * 4)..][..4],
                &[255; 4],
                "UI whites must remain white, not Reinhard gray"
            );
            if let Some(reference) = &reference {
                assert_eq!(reference, &pixels);
            } else {
                reference = Some(pixels);
            }
        }
    }
}
