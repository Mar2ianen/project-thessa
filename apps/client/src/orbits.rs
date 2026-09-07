use super::*;

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

pub(super) fn draw_orbits(
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    cameras: Query<(&OrbitCamera, &GlobalTransform), With<Camera3d>>,
    mut orbit_gizmos: Gizmos<OrbitGizmoConfigGroup>,
    mut selected_gizmos: Gizmos<SelectedOrbitGizmoConfigGroup>,
    mut ring_gizmos: Gizmos<NereidRingGizmoConfigGroup>,
) {
    let time = SimTime(clock.sim_seconds);
    let (camera_distance, camera_rotation) = cameras
        .iter()
        .next()
        .map(|(camera, transform)| (camera.distance, transform.rotation()))
        .unwrap_or((ORBIT_FADE_START_CAMERA_DISTANCE, Quat::IDENTITY));
    let selected_position = map_position(&runtime.ephemeris, &map, map.selected, time);
    // Keep labels readable at overview distance without letting a framed body
    // turn its name into a giant foreground overlay. Close views only need the
    // selected body's label; the HUD still shows the current focus and target.
    let close_view = camera_distance < 30.0;
    let label_size = if close_view {
        0.06
    } else {
        (camera_distance * 0.005).clamp(0.05, 0.18)
    };

    for body in &runtime.ephemeris.bodies {
        if !is_visible_in_view(&runtime.ephemeris, &map, body.id) {
            continue;
        }
        let Some(body_position) = map_position(&runtime.ephemeris, &map, body.id, time) else {
            continue;
        };
        let body_radius = visual_radius_for_mode(body.radius_m, map.mode);
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
        let Some(points) = orbit_points_in_map(&runtime.ephemeris, &map, body.id, time, 192) else {
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
        && let Some(nereid_position) = map_position(&runtime.ephemeris, &map, nereid_id, time)
    {
        let nereid = runtime
            .ephemeris
            .body(nereid_id)
            .expect("Nereid must exist in the baked system");
        let visual_radius = visual_radius_for_mode(nereid.radius_m, map.mode);
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
            .map(|body| visual_radius_for_mode(body.radius_m, map.mode))
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
    segments: usize,
) -> Option<Vec<Vec3>> {
    let body = ephemeris.body(id).ok()?;
    let orbit = body.orbit?;
    let parent_position = orbit_parent_position_in_map(ephemeris, map, id, time)?;
    Some(
        kepler_orbit_points(orbit, segments)
            .into_iter()
            .map(|point| parent_position + point)
            .collect(),
    )
}

fn orbit_parent_position_in_map(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    id: BodyId,
    time: SimTime,
) -> Option<Vec3> {
    let parent = ephemeris.body(id).ok()?.parent?;
    map_position(ephemeris, map, parent, time).or_else(|| {
        // Keep the scope boundary for unrelated orbits, but show the selected
        // body's real orbit even when its out-of-scope parent is the map target.
        (id == map.selected && id == map.focus)
            .then(|| local_position(ephemeris, map.focus, parent, time))
    })
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

pub(super) fn kepler_orbit_points(orbit: KeplerOrbit, segments: usize) -> Vec<Vec3> {
    let period = orbit.period_s();
    let base_segments = segments.clamp(24, 64);
    let max_segments = segments.clamp(base_segments, 512);
    let tolerance = orbit_tessellation_tolerance(orbit);
    let mut points = Vec::with_capacity(max_segments + 1);
    let first = orbit_point_at_fraction(orbit, period, 0.0);
    points.push(first);

    for index in 0..base_segments {
        let start_fraction = index as f64 / base_segments as f64;
        let end_fraction = (index + 1) as f64 / base_segments as f64;
        let start = if index == 0 {
            first
        } else {
            *points.last().expect("adaptive orbit has a previous point")
        };
        let end = orbit_point_at_fraction(orbit, period, end_fraction);
        append_adaptive_orbit_interval(
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

fn orbit_point_at_fraction(orbit: KeplerOrbit, period: f64, fraction: f64) -> Vec3 {
    let (position, _) = orbit
        .state_relative_at(SimTime(period * fraction))
        .expect("baked orbit path must be evaluable");
    render_position(position)
}

#[allow(clippy::too_many_arguments)]
fn append_adaptive_orbit_interval(
    orbit: KeplerOrbit,
    period: f64,
    start_fraction: f64,
    end_fraction: f64,
    start: Vec3,
    end: Vec3,
    tolerance: f32,
    max_segments: usize,
    points: &mut Vec<Vec3>,
) {
    let midpoint_fraction = (start_fraction + end_fraction) * 0.5;
    let midpoint = orbit_point_at_fraction(orbit, period, midpoint_fraction);
    let chord_midpoint = start.lerp(end, 0.5);
    let should_subdivide =
        midpoint.distance(chord_midpoint) > tolerance && points.len() < max_segments;

    if should_subdivide {
        append_adaptive_orbit_interval(
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
        append_adaptive_orbit_interval(
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
