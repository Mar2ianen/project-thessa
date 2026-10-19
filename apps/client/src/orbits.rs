use super::*;
#[cfg(test)]
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
        pilot
            .as_ref()
            .map_or(ControlMode::Navball, |state| state.control_mode),
        time,
        anchor,
        camera_pos,
        camera_rotation,
        &mut prediction_cache,
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
    thessa_flight_authority::navigation::craft_display_body(ephemeris, flight, time)
}

/// One-line orbit readout for the map HUD: frame body, apsides, period, or
/// escape energy. The frame tag is the point: speeds without a body are the
/// bug being fixed here.
pub(super) fn craft_orbit_summary(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    time: SimTime,
) -> String {
    thessa_flight_authority::navigation::craft_orbit_summary(ephemeris, flight, time, true)
}

/// Bound-orbit path sampled forward from the current anomaly, clipped where
/// it hits the surface. Drawing the full ellipse for a suborbital arc puts
/// half the line inside the planet; the clipped arc ends at impact instead.
/// Returns the central-relative points and whether the end is an impact.
#[cfg(test)]
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

/// Body-relative f64 JIT polyline, reprojected from the execution cache each
/// camera frame without propagation or renderer-owned bake requests.
#[derive(Default)]
pub(super) struct CraftPredictionCache {
    points: Vec<bevy::math::DVec3>,
    impact: Option<bevy::math::DVec3>,
    display: Option<BodyId>,
}

/// The optimizer's executable cached segment, not a hypothetical coast.
#[allow(clippy::too_many_arguments)]
pub(super) fn draw_craft_orbit_prediction(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    flight: &PilotFlightRuntime,
    mode: ControlMode,
    time: SimTime,
    anchor: bevy::math::DVec3,
    camera_pos: Vec3,
    camera_rotation: Quat,
    cache: &mut CraftPredictionCache,
    gizmos: &mut Gizmos<SelectedOrbitGizmoConfigGroup>,
) {
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
    // Projection changes with simulation time, even when the bake is unchanged.
    // Refresh the bounded polyline each frame; no physics work happens here.
    refresh_prediction_cache(ephemeris, flight, mode, time, display, cache);
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

/// Read the same optimizer guards as execution; rendering never bakes.
fn refresh_prediction_cache(
    ephemeris: &BakedEphemeris,
    flight: &PilotFlightRuntime,
    mode: ControlMode,
    time: SimTime,
    display: BodyId,
    cache: &mut CraftPredictionCache,
) {
    cache.display = Some(display);
    cache.points.clear();
    cache.impact = None;
    let Some(until) = flight.jit_coast_execution_end(ephemeris, mode) else {
        return;
    };
    if time.0 != flight.flight_time_s {
        return;
    }
    let (position, _) = flight.inertial_state_m();
    let projected = thessa_flight_authority::navigation::project_jit_coast_path(
        ephemeris,
        &flight.rails,
        display,
        time,
        position,
        until,
    );
    cache.points = projected.points;
    cache.impact = projected.impact;
}

#[cfg(test)]
use thessa_flight_authority::navigation::future_path_indices;

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
    thessa_flight_authority::navigation::kepler_polyline(
        orbit,
        segments,
        (orbit.semi_major_axis_m * 0.00075).max(DISTANCE_UNIT_M * 0.0015),
    )
    .into_iter()
    .map(render_position_f64)
    .collect()
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
