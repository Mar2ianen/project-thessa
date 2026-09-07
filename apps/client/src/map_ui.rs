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

pub(super) fn update_hud(
    clock: Res<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    map: Res<MapState>,
    mut query: Query<&mut Text, With<Hud>>,
) {
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
    let content = format!(
        "THESSA  /  {}\nFOCUS {}  >  {}\nT+{:03}d {:02}h {:02}m {:02}s  {}  x{:.3}\n[LMB] select  [double] frame\n[RMB] orbit  [MMB] pan  [WHEEL] zoom\n[SPACE] pause  [UP/DOWN] rate  [0-5] scope\n[TAB] next  [ENTER] frame  [F6] pilot HUD preview",
        map.mode.label(),
        focus,
        selected,
        days,
        hours,
        minutes,
        seconds,
        status,
        clock.multiplier,
    );
    for mut text in &mut query {
        **text = content.clone();
    }
}
