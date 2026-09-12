use super::*;

use bevy::ui::FocusPolicy;

/// Shared input state for navigation. The navigation systems should read this before handling
/// pointer motion/scroll and map shortcuts. UI updates it from computed UI rectangles each frame.
#[derive(Resource, Default)]
pub(super) struct MapUiState {
    pub(super) pointer_over_ui: bool,
    pub(super) search_active: bool,
}

#[derive(Component)]
pub(super) struct UiInputBlocker;

/// The HUD is screen-space. World labels are drawn by the 3D gizmo pass so
/// they are refreshed with the same camera pass as the orbits.
pub(super) fn spawn_hud(commands: &mut Commands) {
    commands.insert_resource(MapUiState::default());
    commands.spawn((
        Hud,
        UiInputBlocker,
        Text::new("PROJECT THESSA"),
        TextFont {
            font_size: FontSize::Px(13.0),
            ..default()
        },
        TextColor(Color::srgb(0.82, 0.90, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(12),
            max_width: px(390),
            padding: UiRect::all(px(9)),
            border: UiRect::all(px(1)),
            border_radius: BorderRadius::all(px(4)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.008, 0.018, 0.045, 0.88)),
        BorderColor::all(Color::srgba(0.18, 0.36, 0.58, 0.9)),
        FocusPolicy::Block,
        ZIndex(100),
    ));
}

/// Must run before navigation input if navigation uses `MapUiState::pointer_over_ui`.
pub(super) fn update_map_ui_input_state(
    window: Single<&Window, With<PrimaryWindow>>,
    mut state: ResMut<MapUiState>,
    blockers: Query<
        (&ComputedNode, &UiGlobalTransform, &InheritedVisibility),
        With<UiInputBlocker>,
    >,
) {
    let Some(cursor) = window.cursor_position() else {
        state.pointer_over_ui = false;
        return;
    };
    state.pointer_over_ui = blockers
        .iter()
        .any(|(node, transform, visible)| visible.get() && node.contains_point(*transform, cursor));
}

#[allow(clippy::too_many_arguments)]
pub(super) fn update_hud(
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    navigation: Res<NavigationState>,
    pilot: Option<Res<PilotHudState>>,
    survey: Res<terrain::SurfaceSurvey>,
    flight: Res<PilotFlightRuntime>,
    cameras: Query<&Transform, With<Camera3d>>,
    mut query: Query<(&mut Text, &mut Visibility), With<Hud>>,
) {
    let pilot_visible = pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    let focus = runtime
        .ephemeris
        .body(map.focus)
        .map(|body| body.name.to_uppercase())
        .unwrap_or_else(|_| "UNKNOWN".into());
    let selected = runtime
        .ephemeris
        .body(map.selected)
        .map(|body| body.name.to_uppercase())
        .unwrap_or_else(|_| "NONE".into());
    let (days, hours, minutes, seconds) = format_sim_time(clock.sim_seconds);
    let status = if clock.paused { "PAUSED" } else { "RUN" };
    let orbit = craft_orbit_summary(&runtime.ephemeris, &flight, SimTime(clock.sim_seconds));
    let proximity = proximity_notice(
        &runtime.ephemeris,
        &map,
        &navigation,
        &cameras,
        SimTime(clock.sim_seconds),
    );
    let content = format!(
        "THESSA  /  {}\nFOCUS {}  >  {}\nT+{:03}d {:02}h {:02}m {:02}s  {}  x{:.3}\n{}\n{}\n[LMB] select  [double] frame\n[RMB] orbit  [MMB] pan  [WHEEL] zoom\n[SPACE] pause  [UP/DOWN] rate  [0-5] scope\n[TAB] next  [ENTER] frame  [M] flight",
        map.mode.label(),
        focus,
        selected,
        days,
        hours,
        minutes,
        seconds,
        status,
        clock.multiplier,
        orbit,
        proximity,
    );
    for (mut text, mut visibility) in &mut query {
        *visibility = if pilot_visible || survey.active {
            Visibility::Hidden
        } else {
            Visibility::Visible
        };
        if pilot_visible || survey.active {
            continue;
        }
        **text = content.clone();
    }
}

/// Camera-vs-body notice for the map HUD: entering a body reads as approach
/// (the mesh blends to true scale) rather than a clamped fly-through, so say
/// so. Inside the physical radius the scale is exact; inside the exaggerated
/// icon the view is mid-blend. Nearest body wins; empty when clear.
fn proximity_notice(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    navigation: &NavigationState,
    cameras: &Query<&Transform, With<Camera3d>>,
    time: SimTime,
) -> String {
    let Some(transform) = cameras.iter().next() else {
        return String::new();
    };
    // Fresh component transform (this frame), not the propagated global one.
    let camera_pos = transform.translation;
    let anchor = camera_anchor_f64(ephemeris, map, navigation, time);
    let mut inside: Option<(&str, f32)> = None;
    let mut near: Option<(&str, f32)> = None;
    for body in &ephemeris.bodies {
        if body.radius_m <= 0.0 || !is_visible_in_view(ephemeris, map, body.id) {
            continue;
        }
        let Some(position) = map_position_anchored(ephemeris, map, body.id, time, anchor) else {
            continue;
        };
        let distance = position.distance(camera_pos);
        let physical = true_radius_units(body.radius_m);
        if physical > 0.0 && distance < physical {
            if inside.is_none_or(|(_, best)| distance < best) {
                inside = Some((body.name.as_str(), distance));
            }
            continue;
        }
        let visual = visual_radius_for_view(body.radius_m, map.mode, distance);
        if distance < visual * 1.5 && near.is_none_or(|(_, best)| distance < best) {
            near = Some((body.name.as_str(), distance));
        }
    }
    if let Some((name, _)) = inside {
        return format!("INSIDE {} — TRUE SCALE", name.to_uppercase());
    }
    if let Some((name, _)) = near {
        return format!("PROXIMITY {} — TRUE SCALE BLEND", name.to_uppercase());
    }
    String::new()
}
