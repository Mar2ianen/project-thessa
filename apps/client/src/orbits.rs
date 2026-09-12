use super::*;
use thessa_sim_core::OsculatingElements;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(super) struct OrbitGizmoConfigGroup;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(super) struct SelectedOrbitGizmoConfigGroup;

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(super) struct NereidRingGizmoConfigGroup;

/// Registers the three visual layers used by the orbit renderer.
///
/// The parent app should add `OrbitGizmoPlugin` after `DefaultPlugins`.
pub(super) struct OrbitGizmoPlugin;

impl Plugin for OrbitGizmoPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            OrbitGizmoConfigGroup,
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 0.85,
                    perspective: false,
                    joints: GizmoLineJoint::None,
                    ..default()
                },
                depth_bias: -0.01,
                ..default()
            },
        )
        .insert_gizmo_config(
            SelectedOrbitGizmoConfigGroup,
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 1.35,
                    perspective: false,
                    joints: GizmoLineJoint::Round(2),
                    ..default()
                },
                depth_bias: -0.02,
                ..default()
            },
        )
        .insert_gizmo_config(
            NereidRingGizmoConfigGroup,
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 1.15,
                    perspective: false,
                    joints: GizmoLineJoint::Round(2),
                    ..default()
                },
                depth_bias: -0.015,
                ..default()
            },
        );
    }
}

const ORBIT_FADE_START_CAMERA_DISTANCE: f32 = 160.0;
const ORBIT_FADE_END_CAMERA_DISTANCE: f32 = 40.0;
const ORBIT_DIM_START_FACTOR: f32 = 0.75;
const ORBIT_DIM_END_FACTOR: f32 = 1.50;
const ORBIT_DIM_STRENGTH: f32 = 0.72;

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_orbits(
    survey: Res<terrain::SurfaceSurvey>,
    flight: Res<PilotFlightRuntime>,
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    navigation: Res<NavigationState>,
    pilot: Option<Res<PilotHudState>>,
    cameras: Query<(&OrbitCamera, &Transform), With<Camera3d>>,
    mut orbit_gizmos: Gizmos<OrbitGizmoConfigGroup>,
    mut selected_gizmos: Gizmos<SelectedOrbitGizmoConfigGroup>,
    mut ring_gizmos: Gizmos<NereidRingGizmoConfigGroup>,
    mut prediction_cache: Local<CraftPredictionCache>,
    mut fallback_rails: Local<thessa_sim_core::OnRailsCache>,
    mut perf: ResMut<perf::PerfMonitor>,
) {
    if survey.active {
        return;
    }
    if pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot)
    {
        return;
    }
    let time = SimTime(clock.sim_seconds);
    let (camera_distance, camera_rotation, camera_pos) = cameras
        .iter()
        .next()
        .map(|(camera, transform)| (camera.distance, transform.rotation, transform.translation))
        .unwrap_or((ORBIT_FADE_START_CAMERA_DISTANCE, Quat::IDENTITY, Vec3::ZERO));
    let anchor = camera_anchor_f64(&runtime.ephemeris, &map, &navigation, time);
    // Marker and prediction follow the display frame (strongest local pull),
    // so the X-15 sits on the world it actually orbits. The offset is rebuilt
    // from the inertial states: terrain_origin is reference-relative and goes
    // stale the moment the display body changes.
    let display_body = craft_display_body(&runtime.ephemeris, &flight, time);
    if let Some(body) = runtime.ephemeris.body(display_body).ok()
        && let Some(center) = map_position_anchored(&runtime.ephemeris, &map, body.id, time, anchor)
        && let Ok(display_state) = runtime.ephemeris.body_state(display_body, time)
    {
        let scale = visual_radius_for_view(body.radius_m, map.mode, center.distance(camera_pos))
            as f64
            / body.radius_m;
        let (r_craft, _) = flight.inertial_state_m();
        let relative = r_craft - display_state.position_inertial;
        let offset = bevy::math::DVec3::new(relative.x, relative.z, -relative.y);
        let position = center + (offset * scale).as_vec3();
        let marker_size = (camera_distance * 0.007).max(0.00008);
        selected_gizmos.circle(
            Isometry3d::new(position, camera_rotation),
            marker_size,
            Color::srgb(0.3, 1.0, 0.78),
        );
        selected_gizmos.text(
            Isometry3d::new(
                position + camera_rotation * Vec3::Y * marker_size * 2.0,
                camera_rotation,
            ),
            "X-15",
            marker_size,
            Vec2::ZERO,
            Color::srgb(0.3, 1.0, 0.78),
        );
    }
    let prediction_started = std::time::Instant::now();
    draw_craft_orbit_prediction(
        &runtime.ephemeris,
        &map,
        &flight,
        time,
        anchor,
        camera_pos,
        camera_rotation,
        &mut prediction_cache,
        &mut fallback_rails,
        &mut selected_gizmos,
    );
    perf.record_scope(
        "simulation.trajectory_prediction",
        prediction_started.elapsed().as_secs_f64(),
    );
    let selected_position =
        map_position_anchored(&runtime.ephemeris, &map, map.selected, time, anchor);
    // Keep labels readable at overview distance without letting a framed body
    // turn its name into a giant foreground overlay. Close views only need the
    // selected body's label; the HUD still shows the current focus and target.
    let close_view = camera_distance < 30.0;
    let label_size = if close_view {
        camera_distance * 0.012
    } else {
        (camera_distance * 0.005).clamp(0.05, 0.18)
    };

    for body in &runtime.ephemeris.bodies {
        if !is_visible_in_view(&runtime.ephemeris, &map, body.id) {
            continue;
        }
        let Some(body_position) =
            map_position_anchored(&runtime.ephemeris, &map, body.id, time, anchor)
        else {
            continue;
        };
        let body_radius =
            visual_radius_for_view(body.radius_m, map.mode, body_position.distance(camera_pos));
        if !close_view || body.id == map.selected {
            orbit_gizmos.text(
                Isometry3d::new(
                    body_position + Vec3::Y * (body_radius + label_size * 0.75),
                    camera_rotation,
                ),
                &body.name.to_uppercase(),
                label_size,
                Vec2::ZERO,
                if body.id == map.selected {
                    selected_orbit_color()
                } else {
                    Color::srgba(0.72, 0.82, 0.94, 0.82)
                },
            );
        }

        let Some(orbit) = body.orbit else {
            continue;
        };
        // A map focus is the local origin. Its parent is intentionally out of
        // scope, so drawing that parent-relative orbit would create a huge
        // off-screen line across the local map.
        if body.id == map.focus {
            continue;
        }
        let Some(points) =
            orbit_points_in_map(&runtime.ephemeris, &map, body.id, time, anchor, 192)
        else {
            continue;
        };

        if body.id == map.selected {
            selected_gizmos.linestrip(points, selected_orbit_color());
        } else {
            let fade = selected_position
                .zip(orbit_parent_position_in_map(
                    &runtime.ephemeris,
                    &map,
                    body.id,
                    time,
                    anchor,
                ))
                .map(|(selected_position, parent_position)| {
                    orbit_fade_factor(
                        camera_distance,
                        selected_position.distance(parent_position)
                            + (orbit.semi_major_axis_m / DISTANCE_UNIT_M) as f32,
                    )
                })
                .unwrap_or(1.0);
            orbit_gizmos.linestrip(
                points,
                orbit_color(body.name.as_str()).with_alpha(0.42 * fade),
            );
        }
    }

    if let Some(nereid_id) = runtime.ephemeris.body_id("nereid")
        && let Some(nereid_position) =
            map_position_anchored(&runtime.ephemeris, &map, nereid_id, time, anchor)
    {
        let nereid = runtime
            .ephemeris
            .body(nereid_id)
            .expect("Nereid must exist in the baked system");
        let visual_radius = visual_radius_for_view(
            nereid.radius_m,
            map.mode,
            nereid_position.distance(camera_pos),
        );
        let ring_rotation = Quat::from_euler(EulerRot::XYZ, 0.36, 0.08, 0.22);
        for (radius_m, segments) in [
            (NEREID_RING_INNER_RADIUS_M, 180),
            (NEREID_RING_OUTER_RADIUS_M, 220),
        ] {
            let radius = (radius_m / nereid.radius_m) as f32 * visual_radius;
            let points = ring_points(radius, segments)
                .into_iter()
                .map(|point| nereid_position + ring_rotation * point);
            ring_gizmos.linestrip(points, Color::srgba(0.90, 0.60, 0.25, 0.72));
        }
    }

    if let Some(selected_position) = selected_position {
        let body_radius = runtime
            .ephemeris
            .body(map.selected)
            .map(|body| {
                visual_radius_for_view(
                    body.radius_m,
                    map.mode,
                    selected_position.distance(camera_pos),
                )
            })
            .unwrap_or(0.15);
        let marker_radius = (body_radius * 1.9).max(0.22);
        selected_gizmos.linestrip(
            ring_points(marker_radius, 48)
                .into_iter()
                .map(|point| selected_position + point),
            Color::srgba(0.98, 0.78, 0.30, 0.90),
        );
    }
}

pub(super) fn orbit_points_in_map(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
    anchor: bevy::math::DVec3,
    segments: usize,
) -> Option<Vec<Vec3>> {
    let body = ephemeris.body(id).ok()?;
    let orbit = body.orbit?;
    let parent = body.parent?;
    // Focus-relative parent in f64 (evaluable even out of scope), rebased to
    // the follow anchor before narrowing. Visibility still gates the orbit
    // itself via the caller + parent helper below.
    let parent_f64 = local_position_f64(ephemeris, map.focus, parent, time)?;
    // Preserve the scope-boundary rule: skip orbits whose parent is out of
    // scope, except the selected focus body itself.
    let parent_visible = map_position_anchored(ephemeris, map, parent, time, anchor);
    let show_unrelated = parent_visible.is_some() || (id == map.selected && id == map.focus);
    if !show_unrelated {
        return None;
    }
    Some(
        kepler_orbit_points_f64(orbit, segments)
            .into_iter()
            .map(|point| (parent_f64 + point - anchor).as_vec3())
            .collect(),
    )
}

fn orbit_parent_position_in_map(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
    anchor: bevy::math::DVec3,
) -> Option<Vec3> {
    let parent = ephemeris.body(id).ok()?.parent?;
    // Parent in focus frame (f64), then rebased to the follow anchor before
    // f32 conversion so distant scopes do not quantize.
    let parent_f64 = local_position_f64(ephemeris, map.focus, parent, time)?;
    let visible = map_position_anchored(ephemeris, map, parent, time, anchor);
    if visible.is_some() {
        return visible;
    }
    // Keep the scope boundary for unrelated orbits, but show the selected
    // body's real orbit even when its out-of-scope parent is the map target.
    (id == map.selected && id == map.focus).then(|| (parent_f64 - anchor).as_vec3())
}

pub(super) fn orbit_color(name: &str) -> Color {
    match name {
        "asterion_a" | "asterion_b" | "asterion_c" => Color::srgba(0.48, 0.56, 0.68, 0.42),
        "janus" | "mora" => Color::srgba(0.38, 0.48, 0.61, 0.42),
        "khepri" | "orthea" | "vesper" => Color::srgba(0.40, 0.57, 0.69, 0.42),
        _ => Color::srgba(0.32, 0.47, 0.60, 0.42),
    }
}

fn selected_orbit_color() -> Color {
    Color::srgba(1.0, 0.72, 0.28, 0.94)
}

/// Display frame for craft readouts: the dominant gravity source at the
/// craft position (a readout hint, never a physics switch). Falls back to the
/// launch reference when no source resolves.
pub(super) fn craft_display_body(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    time: SimTime,
) -> BodyId {
    let (r, _) = flight.inertial_state_m();
    ephemeris
        .dominant_body(r, time)
        .unwrap_or(flight.reference_body)
}

/// Live osculating elements of the flown vehicle around its display body,
/// or `None` for degenerate states (radial plunge, unresolvable bodies).
pub(super) fn craft_elements(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    time: SimTime,
) -> Option<OsculatingElements> {
    let display = craft_display_body(ephemeris, flight, time);
    let body = ephemeris.body(display).ok()?;
    let state = ephemeris.body_state(display, time).ok()?;
    let (r, v) = flight.inertial_state_m();
    OsculatingElements::from_state(
        r - state.position_inertial,
        v - state.velocity_inertial,
        body.mu,
    )
    .ok()
}

/// One-line orbit readout for the map HUD: frame body, apsides, period, or
/// escape energy. The frame tag is the point: speeds without a body are the
/// bug being fixed here.
pub(super) fn craft_orbit_summary(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    time: SimTime,
) -> String {
    let display = craft_display_body(ephemeris, flight, time);
    let (Some(elements), Ok(body)) = (
        craft_elements(ephemeris, flight, time),
        ephemeris.body(display),
    ) else {
        return "ORBIT n/a".to_string();
    };
    let frame = body.name.to_uppercase();
    if elements.is_escape() {
        let v_inf = (elements.central_mu / elements.semi_major_axis_m.abs()).sqrt() / 1000.0;
        return format!("ORBIT {frame} ESC  v_inf {v_inf:.1} km/s");
    }
    let ap = (elements.apoapsis_m().unwrap_or(f64::NAN) - body.radius_m) / 1000.0;
    let pe = (elements.periapsis_m() - body.radius_m) / 1000.0;
    // A sub-surface periapsis is an impact trajectory, not an orbit: say so
    // instead of printing a negative Pe next to a period.
    if pe < 0.0 {
        return format!("ORBIT {frame} Ap {ap:.1} km → IMPACT");
    }
    let base = match elements.period_s() {
        Some(period) if period < 48.0 * 3600.0 => {
            format!(
                "ORBIT {frame} Ap {ap:.1} km  Pe {pe:.1} km  T {:.1} h",
                period / 3600.0
            )
        }
        _ => format!("ORBIT {frame} Ap {ap:.1} km  Pe {pe:.1} km"),
    };
    // Scheduler readout: next armed wake as a countdown, plus the last fired
    // wake. The loop waits on events; the HUD shows what it waits on.
    let mut wake = String::new();
    if let Some(next) = flight.scheduler.next() {
        let countdown = (next.time.seconds() - time.seconds()).max(0.0);
        let label = match next.kind {
            thessa_sim_core::ScheduledKind::RailsImpact { .. } => "IMPACT",
            thessa_sim_core::ScheduledKind::RailsHorizon => "HORIZON",
            thessa_sim_core::ScheduledKind::ManeuverNode { .. } => "NODE",
            thessa_sim_core::ScheduledKind::Alarm => "ALARM",
        };
        wake = format!("  WKE {label} T-{}", format_countdown(countdown));
    }
    if let Some(notice) = flight.wake_notice.as_deref() {
        wake.push_str("  [");
        wake.push_str(notice);
        wake.push(']');
    }
    format!("{base}{wake}")
}

/// Format a countdown duration as H:MM:SS (hours unbounded, for multi-day
/// coast wakes).
fn format_countdown(seconds: f64) -> String {
    let total = seconds.max(0.0).round() as u64;
    format!(
        "{}:{:02}:{:02}",
        total / 3600,
        (total / 60) % 60,
        total % 60
    )
}

/// Bound-orbit path sampled forward from the current anomaly, clipped where
/// it hits the surface. Drawing the full ellipse for a suborbital arc puts
/// half the line inside the planet; the clipped arc ends at impact instead.
/// Returns the central-relative points and whether the end is an impact.
pub(super) fn bound_prediction_arc(
    elements: OsculatingElements,
    body_radius_m: f64,
    steps: usize,
) -> (Vec<bevy::math::DVec3>, bool) {
    let mut points = Vec::with_capacity(steps + 1);
    let mut impacted = false;
    for i in 0..=steps {
        let nu = elements.true_anomaly_rad + i as f64 / steps as f64 * std::f64::consts::TAU;
        let p = elements.position_at_nu(nu);
        if p.length() < body_radius_m {
            impacted = true;
            break;
        }
        points.push(p);
    }
    (points, impacted)
}

/// Body-relative f64 prediction polyline (projected anew for every camera frame): the numeric propagation behind it costs
/// milliseconds, so it refreshes every few frames while the gizmo redraws
/// the cached points each frame.
#[derive(Default)]
pub(super) struct CraftPredictionCache {
    tick: u64,
    points: Vec<bevy::math::DVec3>,
    impact: Option<bevy::math::DVec3>,
    display: Option<BodyId>,
}

/// Predicted craft path integrated through the full summed gravity field
/// (no SOI switch), not a two-body ellipse. Where third-body pull matters
/// the line bends away from the osculating ellipse instead of lying: escape
/// legs fly off, suborbital arcs stop at the surface with an impact marker.
#[allow(clippy::too_many_arguments)]
pub(super) fn draw_craft_orbit_prediction(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    flight: &PilotFlightRuntime,
    time: SimTime,
    anchor: bevy::math::DVec3,
    camera_pos: Vec3,
    camera_rotation: Quat,
    cache: &mut CraftPredictionCache,
    fallback_rails: &mut thessa_sim_core::OnRailsCache,
    gizmos: &mut Gizmos<SelectedOrbitGizmoConfigGroup>,
) {
    cache.tick += 1;
    let display = craft_display_body(ephemeris, flight, time);
    let Some(center) = map_position_anchored(ephemeris, map, display, time, anchor) else {
        return;
    };
    let Ok(body) = ephemeris.body(display) else {
        return;
    };
    // Match the globe's current scale, including its proximity transition.
    let scale = visual_radius_for_view(body.radius_m, map.mode, center.distance(camera_pos)) as f64
        / body.radius_m;
    let to_map_vec =
        |p: bevy::math::DVec3| center + (bevy::math::DVec3::new(p.x, p.z, -p.y) * scale).as_vec3();
    if cache.tick.is_multiple_of(30) || cache.points.is_empty() || cache.display != Some(display) {
        // Cut the line where it leaves the current view, never mid-frame:
        // the old fixed 25x rule ended escape legs inside the overview.
        let view_cut_m = (center.distance(camera_pos) as f64 * DISTANCE_UNIT_M * 3.0).max(1.0);
        refresh_prediction_cache(
            ephemeris,
            flight,
            time,
            display,
            body.radius_m,
            view_cut_m,
            cache,
            fallback_rails,
        );
    }
    if cache.points.len() > 1 {
        gizmos.linestrip(
            cache.points.iter().copied().map(to_map_vec),
            Color::srgba(1.0, 0.72, 0.28, 0.9),
        );
    }
    if let Some(impact) = cache.impact {
        gizmos.circle(
            Isometry3d::new(to_map_vec(impact), camera_rotation),
            (center.distance(camera_pos) * 0.004).max(0.00006),
            Color::srgb(1.0, 0.3, 0.2),
        );
    }
}

/// Recompute the cached polyline from the single shared trajectory. In an
/// unpowered vacuum coast the flight loop already rides `flight.rails`, so
/// the line is projected from that exact bake — zero extra integration, and
/// the flown path and the drawn line cannot disagree. Outside coast (aero,
/// thrust) the flight integrates per tick, and the line falls back to a
/// coarse display-only bake in `fallback_rails`, sized from the osculating
/// period (one revolution, bound) or a deep-space horizon (escape/unknown).
/// Samples anchor to the display body's position at each sample epoch and
/// stop past 25x the initial radius or the view cut; an impact end snaps
/// onto the surface. Analytic bound arc is the fallback if propagation fails.
#[allow(clippy::too_many_arguments)]
fn refresh_prediction_cache(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    time: SimTime,
    display: BodyId,
    body_radius_m: f64,
    view_cut_m: f64,
    cache: &mut CraftPredictionCache,
    fallback_rails: &mut thessa_sim_core::OnRailsCache,
) {
    use thessa_sim_core::{
        COAST_RAILS_MAX_STEPS, COAST_RAILS_POSITION_TOL_M, COAST_RAILS_STEP_S,
        COAST_RAILS_VELOCITY_TOL_MPS, TestParticleState, VerletConfig,
    };
    cache.display = Some(display);
    cache.points.clear();
    cache.impact = None;
    let (r_craft, v_craft) = flight.inertial_state_m();
    let initial = TestParticleState {
        position: r_craft,
        velocity: v_craft,
    };
    let impact_bodies: Vec<BodyId> = ephemeris
        .bodies
        .iter()
        .filter(|b| b.radius_m > 0.0)
        .map(|b| b.id)
        .collect();
    // The flight loop's own bake serves the line directly in coast.
    let rails_config = VerletConfig {
        step_s: COAST_RAILS_STEP_S,
        max_steps: COAST_RAILS_MAX_STEPS,
    };
    if flight.rails.usable_for(
        ephemeris,
        initial,
        time,
        rails_config,
        &impact_bodies,
        COAST_RAILS_POSITION_TOL_M,
        COAST_RAILS_VELOCITY_TOL_MPS,
    ) {
        project_rails_path(
            ephemeris,
            flight.rails.path().expect("usable rails hold a path"),
            display,
            time,
            r_craft,
            view_cut_m,
            cache,
        );
        return;
    }
    // Powered/aero flight: coarse display-only bake in the fallback cache.
    let elements = craft_elements(ephemeris, flight, time);
    // Escape/deep-space legs ride a timescale-following bake: a year of
    // interstellar cruise in hundreds of samples instead of millions.
    // Uniform 4 h steps were measured at 7.4e10 m asymptote error (the fast
    // periapsis bend never resolves); the scaled bake holds ~1e8 m over a
    // year, subpixel at any zoom that fits it.
    if elements.as_ref().is_none_or(|elements| elements.is_escape()) {
        let scaled_config = VerletConfig {
            step_s: thessa_sim_core::DISPLAY_SCALED_H_MIN_S,
            max_steps: thessa_sim_core::DISPLAY_SCALED_MAX_SAMPLES,
        };
        let path = if fallback_rails.usable_for(
            ephemeris, initial, time, scaled_config, &impact_bodies, 5.0, 0.05,
        ) {
            fallback_rails.path().expect("usable cache holds a path")
        } else {
            match fallback_rails.bake_scaled(ephemeris, initial, time, &impact_bodies) {
                Ok(path) => path,
                Err(_) => return,
            }
        };
        project_rails_path(ephemeris, path, display, time, r_craft, view_cut_m, cache);
        return;
    }
    let (step_s, max_steps) = match elements {
        // Bound arcs always cover a full revolution: the step scales with
        // the period (clamped for sanity), so long-period loops close
        // instead of stopping mid-frame past an arbitrary hour cutoff.
        Some(elements) if !elements.is_escape() => match elements.period_s() {
            Some(period) => ((period / 300.0).clamp(5.0, 3600.0), 300),
            None => (600.0, 200),
        },
        _ => (900.0, 400),
    };
    let config = VerletConfig { step_s, max_steps };
    let path =
        if fallback_rails.usable_for(ephemeris, initial, time, config, &impact_bodies, 5.0, 0.05) {
            fallback_rails.path().expect("usable cache holds a path")
        } else {
            match fallback_rails.bake(ephemeris, initial, time, config, &impact_bodies) {
                Ok(path) => path,
                Err(_) => {
                    if let Some(elements) = elements
                        && !elements.is_escape()
                    {
                        let (arc, _) = bound_prediction_arc(elements, body_radius_m, 144);
                        cache.points = arc;
                    }
                    return;
                }
            }
        };
    project_rails_path(ephemeris, path, display, time, r_craft, view_cut_m, cache);
}

/// Project a baked inertial path into display-body-relative polyline points
/// for the map gizmo, applying the view cut and the impact marker.
fn project_rails_path(
    ephemeris: &BakedEphemeris,
    path: &thessa_sim_core::SampledPath,
    display: BodyId,
    time: SimTime,
    r_craft: bevy::math::DVec3,
    view_cut_m: f64,
    cache: &mut CraftPredictionCache,
) {
    let Ok(home_now) = ephemeris.body_state(display, time) else {
        return;
    };
    let initial_radius = (r_craft - home_now.position_inertial).length().max(1.0);
    let cut_m = (initial_radius * 25.0).max(view_cut_m);
    // Do not redraw flown history or submit 40,000 gizmo vertices every
    // refresh. Keep the current craft point and a bounded future polyline.
    cache.points.push(r_craft - home_now.position_inertial);
    let mut reached_end = false;
    for index in future_path_indices(&path.times, time, 1024) {
        let Ok(home) = ephemeris.body_state(display, path.times[index]) else {
            break;
        };
        let relative = path.positions[index] - home.position_inertial;
        if relative.length() > cut_m {
            break;
        }
        cache.points.push(relative);
        reached_end = index + 1 == path.positions.len();
    }
    if matches!(path.end, thessa_sim_core::SampledPathEnd::Impact(_)) && reached_end {
        cache.impact = cache.points.last().copied();
    }
}

fn future_path_indices(times: &[SimTime], now: SimTime, budget: usize) -> Vec<usize> {
    let first = times.partition_point(|time| time.0 <= now.0);
    if first == times.len() || budget < 2 {
        return Vec::new();
    }
    let stride = (times.len() - first).div_ceil(budget - 1);
    let mut indices: Vec<_> = (first..times.len()).step_by(stride).collect();
    let last = times.len() - 1;
    if indices.last() != Some(&last) {
        indices.push(last);
    }
    indices
}

fn orbit_fade_factor(camera_distance: f32, distance_from_selection: f32) -> f32 {
    let close_zoom = smoothstep(
        ORBIT_FADE_START_CAMERA_DISTANCE,
        ORBIT_FADE_END_CAMERA_DISTANCE,
        camera_distance,
    );
    let distant = smoothstep(
        camera_distance * ORBIT_DIM_START_FACTOR,
        camera_distance * ORBIT_DIM_END_FACTOR,
        distance_from_selection,
    );
    1.0 - ORBIT_DIM_STRENGTH * close_zoom * distant
}

fn smoothstep(edge0: f32, edge1: f32, value: f32) -> f32 {
    let width = (edge1 - edge0).max(f32::EPSILON);
    let t = ((value - edge0) / width).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub(super) fn kepler_orbit_points_f64(
    orbit: KeplerOrbit,
    segments: usize,
) -> Vec<bevy::math::DVec3> {
    let period = orbit.period_s();
    let base_segments = segments.clamp(24, 64);
    let max_segments = segments.clamp(base_segments, 512);
    let tolerance = orbit_tessellation_tolerance(orbit) as f64;
    let mut points = Vec::with_capacity(max_segments + 1);
    let first = orbit_point_at_fraction_f64(orbit, period, 0.0);
    points.push(first);

    for index in 0..base_segments {
        let start_fraction = index as f64 / base_segments as f64;
        let end_fraction = (index + 1) as f64 / base_segments as f64;
        let start = if index == 0 {
            first
        } else {
            *points.last().expect("adaptive orbit has a previous point")
        };
        let end = orbit_point_at_fraction_f64(orbit, period, end_fraction);
        append_adaptive_orbit_interval_f64(
            orbit,
            period,
            start_fraction,
            end_fraction,
            start,
            end,
            tolerance,
            max_segments,
            &mut points,
        );
    }
    points
}

fn orbit_point_at_fraction_f64(
    orbit: KeplerOrbit,
    period: f64,
    fraction: f64,
) -> bevy::math::DVec3 {
    let (position, _) = orbit
        .state_relative_at(SimTime(period * fraction))
        .expect("baked orbit path must be evaluable");
    render_position_f64(position)
}

#[allow(clippy::too_many_arguments)]
fn append_adaptive_orbit_interval_f64(
    orbit: KeplerOrbit,
    period: f64,
    start_fraction: f64,
    end_fraction: f64,
    start: bevy::math::DVec3,
    end: bevy::math::DVec3,
    tolerance: f64,
    max_segments: usize,
    points: &mut Vec<bevy::math::DVec3>,
) {
    let midpoint_fraction = (start_fraction + end_fraction) * 0.5;
    let midpoint = orbit_point_at_fraction_f64(orbit, period, midpoint_fraction);
    let chord_midpoint = start.lerp(end, 0.5);
    let should_subdivide =
        midpoint.distance(chord_midpoint) > tolerance && points.len() < max_segments;

    if should_subdivide {
        append_adaptive_orbit_interval_f64(
            orbit,
            period,
            start_fraction,
            midpoint_fraction,
            start,
            midpoint,
            tolerance,
            max_segments,
            points,
        );
        append_adaptive_orbit_interval_f64(
            orbit,
            period,
            midpoint_fraction,
            end_fraction,
            midpoint,
            end,
            tolerance,
            max_segments,
            points,
        );
    } else {
        points.push(end);
    }
}

fn orbit_tessellation_tolerance(orbit: KeplerOrbit) -> f32 {
    let radius = (orbit.semi_major_axis_m / DISTANCE_UNIT_M) as f32;
    (radius * 0.00075).max(0.0015)
}

pub(super) fn ring_points(radius: f32, segments: usize) -> Vec<Vec3> {
    (0..=segments)
        .map(|index| {
            let angle = index as f32 / segments as f32 * TAU;
            Vec3::new(radius * angle.cos(), 0.0, radius * angle.sin())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_sim_core::OsculatingElements;

    fn circular_elements(radius_m: f64, mu: f64) -> OsculatingElements {
        let speed = (mu / radius_m).sqrt();
        OsculatingElements::from_state(
            bevy::math::DVec3::X * radius_m,
            bevy::math::DVec3::Y * speed,
            mu,
        )
        .expect("circular state has elements")
    }

    #[test]
    fn bound_arc_draws_full_loop_for_stable_orbit() {
        let mu = 4.0e13;
        let (arc, impacted) =
            bound_prediction_arc(circular_elements(10_000_000.0, mu), 6_000_000.0, 144);
        assert!(!impacted);
        assert_eq!(arc.len(), 145);
    }

    #[test]
    fn bound_arc_clips_suborbital_path_at_impact() {
        // Tossed straight up at half escape speed: falls back, Pe underground.
        let mu = 4.0e13;
        let radius = 10_000_000.0;
        let elements = OsculatingElements::from_state(
            bevy::math::DVec3::X * radius,
            bevy::math::DVec3::Y * (mu / radius).sqrt() * 0.5,
            mu,
        )
        .expect("suborbital state has elements");
        assert!(elements.periapsis_m() < 6_000_000.0);
        let (arc, impacted) = bound_prediction_arc(elements, 6_000_000.0, 144);
        assert!(impacted, "suborbital arc must end in impact");
        assert!(arc.len() > 1 && arc.len() < 145);
        for p in &arc {
            assert!(p.length() >= 6_000_000.0, "no point may be underground");
        }
    }
}

#[cfg(test)]
mod future_path_tests {
    use super::*;
    #[test]
    fn long_rails_show_future_only_and_keep_endpoint_with_bounded_vertices() {
        let times: Vec<_> = (0..40_001).map(|i| SimTime(i as f64 * 5.0)).collect();
        let indices = future_path_indices(&times, SimTime(80_002.0), 1024);
        assert!(indices.len() <= 1024);
        assert_eq!(indices[0], 16_001);
        assert_eq!(indices.last(), Some(&40_000));
        assert!(indices.iter().all(|i| times[*i].0 > 80_002.0));
        assert!(future_path_indices(&times, SimTime(200_000.0), 1024).is_empty());
    }
}
