//! Native map presentation of baked ephemerides and the authority's existing rails.
//! No force, trajectory integration, simulation clock or astronomical f32 state.
use glam::DVec3;
use std::sync::Arc;
use thessa_flight_authority::navigation::{
    MapCamera, MapMode, cycle_body, kepler_polyline, visible,
};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime};

struct Entry {
    id: BodyId,
    name: String,
    radius_m: f64,
    parent: Option<BodyId>,
    scopes: [bool; 6],
    orbit: Arc<Vec<DVec3>>,
}

#[derive(Default)]
pub struct Source {
    pub open: bool,
    catalog: Vec<Entry>,
}

impl Source {
    pub fn set_open(&mut self, open: bool, ephemeris: &BakedEphemeris) {
        self.open = open;
        if !open || !self.catalog.is_empty() {
            return;
        }
        self.catalog = ephemeris
            .bodies
            .iter()
            .map(|body| Entry {
                id: body.id,
                name: body.name.clone(),
                radius_m: body.radius_m,
                parent: body.parent,
                scopes: MapMode::ALL.map(|mode| {
                    ephemeris
                        .body_id(mode.focus_name())
                        .is_some_and(|focus| visible(ephemeris, mode, focus, body.id))
                }),
                orbit: Arc::new(
                    body.orbit
                        .map(|orbit| kepler_polyline(orbit, 512, orbit.semi_major_axis_m * 0.00075))
                        .unwrap_or_default(),
                ),
            })
            .collect();
    }

    pub fn sample(&self, ephemeris: &BakedEphemeris, time: SimTime) -> Option<Snapshot> {
        if !self.open {
            return None;
        }
        let bodies = self
            .catalog
            .iter()
            .filter_map(|entry| {
                Some(Body {
                    id: entry.id,
                    name: entry.name.clone(),
                    radius_m: entry.radius_m,
                    parent: entry.parent,
                    scopes: entry.scopes,
                    orbit: entry.orbit.clone(),
                    position_m: ephemeris.body_state(entry.id, time).ok()?.position_inertial,
                })
            })
            .collect();
        Some(Snapshot {
            bodies,
            focuses: MapMode::ALL.map(|mode| ephemeris.body_id(mode.focus_name())),
            time,
            craft: None,
            trajectory: None,
        })
    }
}

pub struct Body {
    pub id: BodyId,
    pub name: String,
    pub radius_m: f64,
    pub parent: Option<BodyId>,
    pub scopes: [bool; 6],
    pub orbit: Arc<Vec<DVec3>>,
    pub position_m: DVec3,
}

pub struct Snapshot {
    pub bodies: Vec<Body>,
    pub focuses: [Option<BodyId>; 6],
    pub time: SimTime,
    /// Newest authoritative PRIMARY state, with its own time; never predicted locally remotely.
    pub craft: Option<(DVec3, SimTime)>,
    /// Already projected through the canonical ephemeris at every sample's own time.
    pub trajectory: Option<(DVec3, thessa_flight_authority::navigation::ProjectedPath)>,
}
impl Snapshot {
    fn body(&self, id: BodyId) -> Option<&Body> {
        self.bodies.iter().find(|body| body.id == id)
    }
}

pub struct View {
    mode: usize,
    selected: Option<BodyId>,
    camera: MapCamera,
    fit: bool,
    follow: bool,
    follow_primary: bool,
}
impl Default for View {
    fn default() -> Self {
        Self {
            mode: 0,
            selected: None,
            camera: MapCamera::default(),
            fit: true,
            follow: true,
            follow_primary: false,
        }
    }
}
impl View {
    fn fit_scope(&mut self, data: &Snapshot, focus: DVec3) {
        self.camera.center_m = DVec3::ZERO;
        self.camera.half_height_m = data
            .bodies
            .iter()
            .filter(|b| b.scopes[self.mode])
            .map(|b| (b.position_m - focus).length() + b.radius_m)
            .fold(1.0, f64::max)
            * 1.3;
        self.fit = false;
        self.follow = false;
        self.follow_primary = false;
    }

    pub fn show(&mut self, context: &egui::Context, open: &mut bool, data: &Snapshot) {
        egui::Window::new("System map | baked ephemerides")
            .open(open)
            .default_pos([32.0, 32.0])
            .default_size([1000.0, 660.0])
            .show(context, |ui| {
                ui.horizontal_wrapped(|ui| {
                    for (index, mode) in MapMode::ALL.into_iter().enumerate() {
                        if ui.selectable_label(self.mode == index, mode.label()).clicked() {
                            self.mode = index;
                            self.fit = true;
                        }
                    }
                });
                let Some(focus_body) = data.focuses[self.mode].and_then(|id| data.body(id)) else {
                    ui.label("Missing baked focus");
                    return;
                };
                let focus = focus_body.position_m;
                let bodies: Vec<_> = data.bodies.iter()
                    .filter(|body| body.radius_m > 0.0 && body.scopes[self.mode]).collect();
                let ids: Vec<_> = bodies.iter().map(|body| body.id).collect();
                if self.selected.is_none_or(|id| !ids.contains(&id)) {
                    self.selected = ids.first().copied();
                }
                ui.horizontal_wrapped(|ui| {
                    if ui.button("Previous").clicked() {
                        self.selected = self.selected.and_then(|id| cycle_body(&ids, id, true));
                    }
                    egui::ComboBox::from_id_salt("map-body")
                        .selected_text(self.selected.and_then(|id| data.body(id)).map_or("—", |body| body.name.as_str()))
                        .show_ui(ui, |ui| {
                            for body in &bodies {
                                ui.selectable_value(&mut self.selected, Some(body.id), &body.name);
                            }
                        });
                    if ui.button("Next").clicked() {
                        self.selected = self.selected.and_then(|id| cycle_body(&ids, id, false));
                    }
                    if ui.button("Fit scope").clicked() { self.fit = true; }
                    if ui.button("Frame body").clicked()
                        && let Some(body) = self.selected.and_then(|id| data.body(id))
                    {
                        self.camera.half_height_m = (body.radius_m * 4.0).max(1.0);
                        self.follow = true;
                        self.follow_primary = false;
                        self.fit = false;
                    }
                    if ui.checkbox(&mut self.follow, "Follow body").changed() {
                        self.follow_primary = false;
                    }
                    if let Some((craft, _)) = data.craft
                        && ui.button("Frame PRIMARY").clicked()
                    {
                        self.follow = false;
                        self.follow_primary = true;
                        self.fit = false;
                        self.camera.half_height_m = data.bodies.iter().filter(|b| b.radius_m > 0.0)
                            .min_by(|a, b| a.position_m.distance_squared(craft).total_cmp(&b.position_m.distance_squared(craft)))
                            .map_or(1e7, |body| body.radius_m * 2.5);
                    }
                });
                ui.small(format!("t={:.3}s · {:.3} km half-height · right-drag orbit · middle-drag pan · wheel zoom · click selects · M closes", data.time.0, self.camera.half_height_m / 1000.0));
                if let Some((_, time)) = data.craft {
                    ui.small(format!("PRIMARY newest authority t={:.3}s · {}", time.0,
                        if data.trajectory.is_some() { "shared full-field rails (reference-body frame)" }
                        else { "no current shared rails path; no synthetic forecast" }));
                }
                let size = egui::vec2(ui.available_width().max(1.0), ui.available_height().max(120.0));
                let (response, painter) = ui.allocate_painter(size, egui::Sense::click_and_drag());
                let rect = response.rect;
                if self.fit {
                    self.fit_scope(data, focus);
                    self.camera.half_height_m *= (rect.height() / rect.width()).max(1.0) as f64;
                }
                if self.follow && let Some(body) = self.selected.and_then(|id| data.body(id)) {
                    self.camera.center_m = body.position_m - focus;
                }
                if self.follow_primary && let Some((craft, _)) = data.craft {
                    self.camera.center_m = craft - focus;
                }
                if response.dragged_by(egui::PointerButton::Secondary) {
                    let d = response.drag_delta();
                    self.camera.drag_orbit(d.x.into(), d.y.into());
                }
                if response.dragged_by(egui::PointerButton::Middle) {
                    let d = response.drag_delta();
                    self.follow = false;
                    self.follow_primary = false;
                    self.camera.pan(d.x.into(), d.y.into(), rect.height().into());
                }
                if response.hovered() {
                    self.camera.zoom(f64::from(ui.input(|i| i.smooth_scroll_delta.y)) / 16.0);
                }
                self.paint(&painter, &response, data, &bodies, focus);
            });
    }

    fn paint(
        &mut self,
        painter: &egui::Painter,
        response: &egui::Response,
        data: &Snapshot,
        bodies: &[&Body],
        focus: DVec3,
    ) {
        let rect = response.rect;
        let painter = painter.with_clip_rect(rect);
        painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(5, 9, 18));
        let extent = [rect.width().into(), rect.height().into()];
        let pixels = |p: [f64; 2]| {
            egui::pos2(
                rect.left() + p[0] as f32 * rect.width(),
                rect.top() + p[1] as f32 * rect.height(),
            )
        };
        let project = |p| {
            self.camera.project(p, extent).and_then(|p| {
                ((0.0..=1.0).contains(&p[0]) && (0.0..=1.0).contains(&p[1]))
                    .then(|| pixels([p[0], p[1]]))
            })
        };
        let line = |a, b, color, width| {
            if let Some(points) = self.camera.project_segment(a, b, extent) {
                painter.line_segment(points.map(pixels), (width, color));
            }
        };
        for body in bodies {
            if let Some(parent) = body.parent.and_then(|id| data.body(id))
                // Match the legacy scope boundary: no unrelated parent's orbit.
                && parent.scopes[self.mode]
            {
                let origin = parent.position_m - focus;
                let color = if self.selected == Some(body.id) {
                    egui::Color32::from_rgb(100, 160, 225)
                } else {
                    egui::Color32::from_rgb(42, 65, 92)
                };
                for edge in body.orbit.windows(2) {
                    line(origin + edge[0], origin + edge[1], color, 1.0);
                }
            }
        }
        if let Some((origin, path)) = &data.trajectory {
            for edge in path.points.windows(2) {
                line(
                    *origin - focus + edge[0],
                    *origin - focus + edge[1],
                    egui::Color32::LIGHT_GREEN,
                    1.5,
                );
            }
            if let Some(p) = path.impact.and_then(|p| project(*origin - focus + p)) {
                painter.circle_stroke(p, 6.0, (2.0, egui::Color32::RED));
            }
        }
        let mut hovered = None;
        let mut nearest = 14.0_f32;
        for body in bodies {
            if let Some(p) = project(body.position_m - focus) {
                let selected = self.selected == Some(body.id);
                let radius =
                    (body.radius_m / self.camera.half_height_m * rect.height() as f64 * 0.5)
                        .clamp(3.0, rect.height() as f64 * 2.0) as f32;
                painter.circle_stroke(
                    p,
                    radius,
                    (
                        if selected { 2.0 } else { 1.0 },
                        if selected {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::LIGHT_BLUE
                        },
                    ),
                );
                if let Some(cursor) = response.hover_pos() {
                    let distance = cursor.distance(p);
                    if distance < nearest {
                        nearest = distance;
                        hovered = Some(body.id);
                    }
                }
            }
        }
        let mut labelled = Vec::<egui::Rect>::new();
        if let Some((craft, _)) = data.craft
            && let Some(p) = project(craft - focus)
        {
            painter.circle_filled(p, 4.0, egui::Color32::LIGHT_GREEN);
            labelled.push(painter.text(
                p,
                egui::Align2::RIGHT_BOTTOM,
                "PRIMARY",
                egui::FontId::proportional(12.0),
                egui::Color32::LIGHT_GREEN,
            ));
        }
        // Never draw a pile of unreadable labels at system scale. Selection and
        // hover have priority; the complete body list remains in the selector.
        let mut priority = bodies.to_vec();
        priority.sort_by(|a, b| {
            let rank = |body: &Body| (self.selected == Some(body.id), hovered == Some(body.id));
            rank(b)
                .cmp(&rank(a))
                .then_with(|| b.radius_m.total_cmp(&a.radius_m))
        });
        for body in priority {
            if let Some(p) = project(body.position_m - focus) {
                let galley = painter.layout_no_wrap(
                    body.name.clone(),
                    egui::FontId::proportional(12.0),
                    egui::Color32::WHITE,
                );
                let size = galley.size();
                let positions = [
                    p + egui::vec2(6.0, 4.0),
                    p + egui::vec2(6.0, -size.y - 4.0),
                    p - egui::vec2(size.x + 6.0, size.y + 4.0),
                ];
                if let Some(label) = positions
                    .into_iter()
                    .map(|p| egui::Rect::from_min_size(p, size))
                    .find(|label| {
                        rect.contains_rect(*label)
                            && !labelled
                                .iter()
                                .any(|other| other.expand(2.0).intersects(*label))
                    })
                {
                    painter.galley(label.min, galley, egui::Color32::WHITE);
                    labelled.push(label);
                }
            }
        }
        if response.clicked() && hovered.is_some() {
            self.selected = hovered;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn render(
        context: &egui::Context,
        view: &mut View,
        data: &Snapshot,
        events: Vec<egui::Event>,
    ) -> egui::FullOutput {
        context.run_ui(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(1280.0, 800.0),
                )),
                events,
                ..Default::default()
            },
            |root| view.show(root.ctx(), &mut true, data),
        )
    }
    fn label_center(shapes: &[egui::epaint::ClippedShape], label: &str) -> egui::Pos2 {
        shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::Shape::Text(text) if text.galley.job.text == label => {
                    Some(text.pos + text.galley.size() * 0.5)
                }
                _ => None,
            })
            .expect("test control label must be rendered")
    }
    #[test]
    fn all_scopes_tessellate_finitely_at_system_and_meter_zoom_and_buttons_select() {
        let mut scene = crate::scene::Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        scene.set_map_open(true);
        let data = scene.map_snapshot().unwrap();
        let context = crate::hud::context();
        let mut view = View::default();
        for mode in 0..6 {
            view.mode = mode;
            view.fit = true;
            for _ in 0..3 {
                render(&context, &mut view, &data, vec![]).drop_without_applying_deltas();
            }
            let selected = view.selected.unwrap();
            let body = data.body(selected).unwrap();
            assert!(body.scopes[mode]);
            let focus = data.body(data.focuses[mode].unwrap()).unwrap().position_m;
            view.camera.center_m = body.position_m - focus;
            view.camera.half_height_m = 1.0;
            let output = render(&context, &mut view, &data, vec![]);
            let jobs = context.tessellate(output.shapes.clone(), output.pixels_per_point);
            let vertices: Vec<_> = jobs
                .iter()
                .flat_map(|job| match &job.primitive {
                    egui::epaint::Primitive::Mesh(mesh) => mesh.vertices.as_slice(),
                    _ => &[],
                })
                .collect();
            assert!(vertices.len() > 100);
            assert!(
                vertices.iter().all(|v| v.pos.is_finite()
                    && v.pos.x.abs() < 10000.0
                    && v.pos.y.abs() < 10000.0)
            );
            output.drop_without_applying_deltas();
        }
        let output = render(&context, &mut view, &data, vec![]);
        let next = label_center(&output.shapes, "Next");
        output.drop_without_applying_deltas();
        let before = view.selected.unwrap();
        for pressed in [true, false] {
            render(
                &context,
                &mut view,
                &data,
                vec![
                    egui::Event::PointerMoved(next),
                    egui::Event::PointerButton {
                        pos: next,
                        button: egui::PointerButton::Primary,
                        pressed,
                        modifiers: Default::default(),
                    },
                ],
            )
            .drop_without_applying_deltas();
        }
        assert_ne!(view.selected, Some(before));
        let output = render(&context, &mut view, &data, vec![]);
        let frame = label_center(&output.shapes, "Frame PRIMARY");
        output.drop_without_applying_deltas();
        for pressed in [true, false] {
            render(
                &context,
                &mut view,
                &data,
                vec![
                    egui::Event::PointerMoved(frame),
                    egui::Event::PointerButton {
                        pos: frame,
                        button: egui::PointerButton::Primary,
                        pressed,
                        modifiers: Default::default(),
                    },
                ],
            )
            .drop_without_applying_deltas();
        }
        assert!(view.follow_primary);
        let focus = data
            .body(data.focuses[view.mode].unwrap())
            .unwrap()
            .position_m;
        assert_eq!(view.camera.center_m, data.craft.unwrap().0 - focus);
        assert_eq!(
            scene.sample.time, data.time,
            "map must not own a simulation clock"
        );
    }

    #[test]
    #[ignore = "benchmark: native 32-body map producer, cached orbit storage and toolkit geometry"]
    fn map_target_size_cost() {
        use std::{hint::black_box, time::Instant};
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = system.bake().unwrap();
        let mut source = Source::default();
        let start = Instant::now();
        source.set_open(true, &e);
        let catalog_ms = start.elapsed().as_secs_f64() * 1000.0;
        let vertices: usize = source.catalog.iter().map(|entry| entry.orbit.len()).sum();
        let start = Instant::now();
        for i in 0..1000 {
            black_box(source.sample(&e, SimTime(i as f64))).unwrap();
        }
        let sample_us = start.elapsed().as_secs_f64() * 1000.0;
        let data = source.sample(&e, SimTime(0.0)).unwrap();
        let context = crate::hud::context();
        let mut view = View::default();
        for _ in 0..3 {
            render(&context, &mut view, &data, vec![]).drop_without_applying_deltas();
        }
        let start = Instant::now();
        let mut mesh_vertices = 0;
        for _ in 0..100 {
            let output = render(&context, &mut view, &data, vec![]);
            let jobs = context.tessellate(output.shapes.clone(), output.pixels_per_point);
            mesh_vertices = jobs
                .iter()
                .map(|job| match &job.primitive {
                    egui::epaint::Primitive::Mesh(m) => m.vertices.len(),
                    _ => 0,
                })
                .sum::<usize>();
            black_box(jobs);
            output.drop_without_applying_deltas();
        }
        let toolkit_us = start.elapsed().as_secs_f64() * 10000.0;
        source.set_open(false, &e);
        let start = Instant::now();
        for _ in 0..10000 {
            assert!(black_box(source.sample(&e, SimTime(0.0))).is_none());
        }
        let closed_ns = start.elapsed().as_secs_f64() * 100000.0;
        println!(
            "native_map bodies={} orbit_vertices={vertices} orbit_payload_bytes={} catalog_ms={catalog_ms:.3} sample_us={sample_us:.3} toolkit_us={toolkit_us:.3} mesh_vertices={mesh_vertices} closed_ns={closed_ns:.3}; CPU producer/toolkit diagnostic, no game-FPS claim",
            e.bodies.len(),
            vertices * std::mem::size_of::<DVec3>()
        );
    }
    #[test]
    fn closed_map_has_no_catalog_or_samples_and_reopen_reuses_paths() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = system.bake().unwrap();
        let mut source = Source::default();
        assert!(source.sample(&e, SimTime(0.0)).is_none());
        assert!(source.catalog.is_empty());
        source.set_open(true, &e);
        let a = source.sample(&e, SimTime(0.0)).unwrap();
        source.set_open(false, &e);
        assert!(source.sample(&e, SimTime(10.0)).is_none());
        source.set_open(true, &e);
        let b = source.sample(&e, SimTime(86400.0)).unwrap();
        assert_eq!(a.bodies.len(), e.bodies.len());
        for (a, b) in a.bodies.iter().zip(&b.bodies) {
            assert!(Arc::ptr_eq(&a.orbit, &b.orbit));
            assert_eq!(
                b.position_m,
                e.body_state(b.id, SimTime(86400.0))
                    .unwrap()
                    .position_inertial
            );
        }
    }
}
