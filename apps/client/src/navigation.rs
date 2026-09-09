use super::*;

const CAMERA_MIN_DISTANCE: f32 = 0.02;
const CAMERA_FIT_MARGIN: f32 = 1.18;
const CAMERA_FRAME_MARGIN: f32 = 1.85;
const CAMERA_MAX_DISTANCE_MULTIPLIER: f32 = 4.0;
const CAMERA_SMOOTHING_RATE: f32 = 12.0;
const CAMERA_DISTANCE_SMOOTHING_RATE: f32 = 10.0;
const CAMERA_FAR_MARGIN: f32 = 1.20;
const DOUBLE_CLICK_WINDOW_S: f64 = 0.34;

#[derive(Clone, Copy, PartialEq, Eq)]
enum NavigationRequest {
    None,
    FitScope,
    FrameSelected,
}

#[derive(Clone, Copy)]
struct ClickRecord {
    body: BodyId,
    at_s: f64,
}

/// Camera state which does not belong in the shared camera component.
///
/// `pan_offset` is always relative to the current anchor (the map focus or
/// the followed body). It is never an absolute world target, so a distant
/// selected body cannot be clamped back to a local-mode limit.
#[derive(Resource)]
pub(super) struct NavigationState {
    pan_offset: Vec3,
    follow_selected: bool,
    last_mode: Option<MapMode>,
    last_focus: Option<BodyId>,
    last_selected: Option<BodyId>,
    pointer_selection_pending: bool,
    request: NavigationRequest,
    last_click: Option<ClickRecord>,
}

impl Default for NavigationState {
    fn default() -> Self {
        Self {
            pan_offset: Vec3::ZERO,
            follow_selected: false,
            last_mode: None,
            last_focus: None,
            last_selected: None,
            pointer_selection_pending: false,
            request: NavigationRequest::None,
            last_click: None,
        }
    }
}

pub(super) fn preview_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut clock: ResMut<SimulationClock>,
    runtime: Res<RuntimeEphemeris>,
    mut map: ResMut<MapState>,
    ui: Option<Res<MapUiState>>,
    pilot: Option<Res<PilotHudState>>,
    mut navigation: ResMut<NavigationState>,
) {
    if pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot)
    {
        // Pilot mode owns KSP-style Space staging, throttle and SAS keys.
        // Do not let the map clock consume those same keys.
        return;
    }
    if keys.just_pressed(KeyCode::Space) {
        clock.paused = !clock.paused;
    }
    if keys.just_pressed(KeyCode::ArrowUp) {
        clock.multiplier = (clock.multiplier * 2.0).min(32.0);
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        clock.multiplier = (clock.multiplier / 2.0).max(0.125);
    }
    if keys.just_pressed(KeyCode::KeyT) {
        clock.sim_seconds = 0.0;
    }

    // Search owns the keyboard while it is active.
    if ui.as_ref().is_some_and(|state| state.search_active) {
        return;
    }
    let requested_mode = if keys.just_pressed(KeyCode::Digit0) {
        Some(MapMode::SystemOverview)
    } else if keys.just_pressed(KeyCode::Digit1) {
        Some(MapMode::Asterion)
    } else if keys.just_pressed(KeyCode::Digit2) {
        Some(MapMode::Nereid)
    } else if keys.just_pressed(KeyCode::Digit3) {
        Some(MapMode::Orthea)
    } else if keys.just_pressed(KeyCode::Digit4) {
        Some(MapMode::Vesper)
    } else if keys.just_pressed(KeyCode::Digit5) {
        Some(MapMode::Binary)
    } else {
        None
    };
    if let Some(mode) = requested_mode {
        map.mode = mode;
        map.focus = runtime
            .ephemeris
            .body_id(mode.focus_name())
            .expect("map mode focus must exist in the baked system");
        map.selected = map.focus;
        request_fit_scope(&mut navigation);
    }

    let reverse_tab = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let cycle_direction = if keys.just_pressed(KeyCode::Tab) {
        Some(reverse_tab)
    } else if keys.just_pressed(KeyCode::BracketLeft) {
        Some(true)
    } else if keys.just_pressed(KeyCode::BracketRight) {
        Some(false)
    } else {
        None
    };
    if let Some(reverse) = cycle_direction {
        let bodies = visible_physical_body_ids(&runtime.ephemeris, &map);
        if let Some(next) = cycle_body(&bodies, map.selected, reverse) {
            map.selected = next;
            request_frame_selected(&mut navigation);
        }
    }
    if keys.just_pressed(KeyCode::Enter) {
        request_frame_selected(&mut navigation);
    }
    if keys.just_pressed(KeyCode::Home) {
        request_fit_scope(&mut navigation);
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn select_body_with_pointer(
    buttons: Res<ButtonInput<MouseButton>>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<Camera3d>>,
    clock: Res<SimulationClock>,
    time: Res<Time>,
    runtime: Res<RuntimeEphemeris>,
    mut map: ResMut<MapState>,
    ui: Option<Res<MapUiState>>,
    pilot: Option<Res<PilotHudState>>,
    mut navigation: ResMut<NavigationState>,
) {
    // Do not treat a click that was already held while the window opened as a
    // selection. The first rendered frame belongs to the deterministic startup
    // framing request.
    if navigation.last_mode.is_none() {
        return;
    }
    let pointer_blocked = ui
        .as_ref()
        .is_some_and(|state| state.pointer_over_ui || state.search_active)
        || pilot
            .as_ref()
            .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    if pointer_blocked || !buttons.just_pressed(MouseButton::Left) {
        return;
    }
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let (camera, camera_transform) = camera.into_inner();
    let sim_time = SimTime(clock.sim_seconds);
    let mut nearest = None;
    for body_id in visible_physical_body_ids(&runtime.ephemeris, &map) {
        let Some(position) = map_position(&runtime.ephemeris, &map, body_id, sim_time) else {
            continue;
        };
        let Ok(viewport_position) = camera.world_to_viewport_with_depth(camera_transform, position)
        else {
            continue;
        };
        if viewport_position.z <= 0.0 {
            continue;
        }
        let distance = cursor.distance(viewport_position.truncate());
        if distance <= 32.0
            && nearest
                .as_ref()
                .is_none_or(|(_, nearest_distance)| distance < *nearest_distance)
        {
            nearest = Some((body_id, distance));
        }
    }

    let Some((selected, _)) = nearest else {
        navigation.last_click = None;
        return;
    };
    let now_s = time.elapsed_secs_f64();
    let double_click = navigation
        .last_click
        .is_some_and(|click| click.body == selected && now_s - click.at_s <= DOUBLE_CLICK_WINDOW_S);
    navigation.last_click = Some(ClickRecord {
        body: selected,
        at_s: now_s,
    });
    map.selected = selected;

    if double_click {
        request_frame_selected(&mut navigation);
        navigation.pointer_selection_pending = false;
    } else {
        // A single click changes selection only. update_camera consumes this
        // marker and will not turn it into an implicit camera move.
        navigation.pointer_selection_pending = true;
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn update_camera(
    time: Res<Time>,
    clock: Res<SimulationClock>,
    keys: Res<ButtonInput<KeyCode>>,
    map: Res<MapState>,
    runtime: Res<RuntimeEphemeris>,
    window: Single<&Window, With<PrimaryWindow>>,
    ui: Option<Res<MapUiState>>,
    pilot: Option<Res<PilotHudState>>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mut navigation: ResMut<NavigationState>,
    mut query: Query<(&mut Transform, &mut OrbitCamera, &mut Projection), With<Camera3d>>,
) {
    let delta = time.delta_secs().clamp(0.0, 0.1);
    let mouse_delta = mouse_motion.delta;
    let pilot_active = pilot
        .as_ref()
        .is_some_and(|state| state.view_mode == ClientViewMode::Pilot);
    let pointer_blocked = pilot_active || ui.as_ref().is_some_and(|state| state.pointer_over_ui);
    let keyboard_blocked = pilot_active || ui.as_ref().is_some_and(|state| state.search_active);

    // Always consume wheel messages. Otherwise a wheel event over the UI can
    // be replayed as soon as the pointer leaves it.
    let mut scroll = 0.0;
    for event in mouse_wheel.read() {
        scroll += match event.unit {
            MouseScrollUnit::Line => event.y,
            MouseScrollUnit::Pixel => event.y / MouseScrollUnit::SCROLL_UNIT_CONVERSION_FACTOR,
        };
    }

    // Pilot owns the same Camera3d while it is visible. Its input system
    // updates the camera around the preview vehicle; the map camera must not
    // overwrite that transform later in the frame.
    if pilot_active {
        return;
    }

    let first_frame = navigation.last_mode.is_none();
    let mode_changed = navigation.last_mode.is_some_and(|mode| mode != map.mode)
        || navigation
            .last_focus
            .is_some_and(|focus| focus != map.focus);
    let selected_changed = navigation
        .last_selected
        .is_some_and(|selected| selected != map.selected);
    let mut request = navigation.request;
    if first_frame && request == NavigationRequest::None {
        request = NavigationRequest::FitScope;
        navigation.request = NavigationRequest::FitScope;
    }

    // A selection made by the UI worker frames/follows the new body. A single
    // pointer click is the one intentional exception.
    if !first_frame
        && selected_changed
        && !navigation.pointer_selection_pending
        && request == NavigationRequest::None
    {
        request = NavigationRequest::FrameSelected;
        navigation.request = NavigationRequest::FrameSelected;
    }
    if !first_frame && mode_changed && request == NavigationRequest::None {
        navigation.request = NavigationRequest::FitScope;
    }
    navigation.pointer_selection_pending = false;

    let sim_time = SimTime(clock.sim_seconds);
    let bounds = scope_bounds(&runtime.ephemeris, &map, sim_time);

    for (mut transform, mut camera, mut projection) in &mut query {
        let (fov, aspect) = projection_view(projection.as_ref(), &window);
        let fit_distance = fit_distance_for_extent(bounds.extent, fov, aspect);
        let (min_distance, max_distance) =
            zoom_limits(&runtime.ephemeris, &map, bounds.extent, fov, aspect);

        if !pointer_blocked {
            if mouse_buttons.pressed(MouseButton::Right) {
                cancel_frame_request(&mut navigation);
                camera.yaw -= mouse_delta.x * MOUSE_ORBIT_SENSITIVITY;
                camera.pitch =
                    (camera.pitch + mouse_delta.y * MOUSE_ORBIT_SENSITIVITY).clamp(0.12, 1.52);
            }
            if mouse_buttons.pressed(MouseButton::Middle) {
                cancel_automatic_camera(&mut navigation, camera.target);
                let right = transform.rotation * Vec3::X;
                let up = transform.rotation * Vec3::Y;
                let pan_scale = camera.distance.max(min_distance) * MOUSE_PAN_SENSITIVITY;
                navigation.pan_offset += (-right * mouse_delta.x + up * mouse_delta.y) * pan_scale;
                navigation.pan_offset = clamp_pan_offset(
                    navigation.pan_offset,
                    bounds.extent.max(camera.distance) * 2.0 + 4.0,
                );
            }
            if scroll != 0.0 {
                cancel_frame_request(&mut navigation);
                camera.distance = (camera.distance * (-scroll * MOUSE_ZOOM_SENSITIVITY).exp())
                    .clamp(min_distance, max_distance);
            }
        }

        if !keyboard_blocked {
            let manual_camera_key = keys.pressed(KeyCode::KeyQ)
                || keys.pressed(KeyCode::KeyE)
                || keys.pressed(KeyCode::KeyR)
                || keys.pressed(KeyCode::KeyF)
                || keys.pressed(KeyCode::KeyW)
                || keys.pressed(KeyCode::KeyS);
            if manual_camera_key {
                cancel_frame_request(&mut navigation);
            }
            if keys.pressed(KeyCode::KeyQ) {
                camera.yaw += delta * 0.65;
            }
            if keys.pressed(KeyCode::KeyE) {
                camera.yaw -= delta * 0.65;
            }
            if keys.pressed(KeyCode::KeyR) {
                camera.pitch = (camera.pitch + delta * 0.45).clamp(0.12, 1.52);
            }
            if keys.pressed(KeyCode::KeyF) {
                camera.pitch = (camera.pitch - delta * 0.45).clamp(0.12, 1.52);
            }
            if keys.pressed(KeyCode::KeyW) {
                camera.distance = (camera.distance - delta * camera.distance.max(1.0) * 0.8)
                    .clamp(min_distance, max_distance);
            }
            if keys.pressed(KeyCode::KeyS) {
                camera.distance = (camera.distance + delta * camera.distance.max(1.0) * 0.8)
                    .clamp(min_distance, max_distance);
            }
        }

        // Manual input may have cancelled an automatic frame request above.
        // Refresh the local copy before applying the camera request.
        request = navigation.request;
        let selected_position = map_position(&runtime.ephemeris, &map, map.selected, sim_time);
        match request {
            NavigationRequest::FitScope => {
                navigation.follow_selected = false;
                navigation.pan_offset = Vec3::ZERO;
                if first_frame {
                    // The initial map is already a deliberate camera request;
                    // do not animate from the placeholder transform and make
                    // the user watch the scene zoom in on every launch.
                    camera.distance = fit_distance.clamp(min_distance, max_distance);
                    camera.target = Vec3::ZERO;
                    navigation.request = NavigationRequest::None;
                } else {
                    camera.distance = smooth_scalar(
                        camera.distance,
                        fit_distance,
                        CAMERA_DISTANCE_SMOOTHING_RATE,
                        delta,
                    )
                    .clamp(min_distance, max_distance);
                    if (camera.distance - fit_distance).abs() <= fit_distance.max(1.0) * 0.002 {
                        navigation.request = NavigationRequest::None;
                    }
                }
            }
            NavigationRequest::FrameSelected => {
                if let Some(position) = selected_position {
                    navigation.follow_selected = true;
                    navigation.pan_offset = Vec3::ZERO;
                    let radius = runtime
                        .ephemeris
                        .body(map.selected)
                        .map(|body| visual_radius_for_mode(body.radius_m, map.mode))
                        .unwrap_or(0.15);
                    let frame_distance = frame_distance_for_radius(radius, fov, aspect);
                    camera.distance = smooth_scalar(
                        camera.distance,
                        frame_distance,
                        CAMERA_DISTANCE_SMOOTHING_RATE,
                        delta,
                    )
                    .clamp(min_distance, max_distance);
                    camera.target =
                        smooth_vec3(camera.target, position, CAMERA_SMOOTHING_RATE, delta);
                    if camera.target.distance(position) <= radius.max(0.02) * 0.01
                        && (camera.distance - frame_distance).abs()
                            <= frame_distance.max(1.0) * 0.002
                    {
                        navigation.request = NavigationRequest::None;
                    }
                }
            }
            NavigationRequest::None => {}
        }

        let anchor = if navigation.follow_selected {
            selected_position.unwrap_or(Vec3::ZERO)
        } else {
            Vec3::ZERO
        };
        let desired_target = anchor + navigation.pan_offset;
        // A focused KSP-style camera follows the selected body's current
        // position exactly. Smoothing this anchor makes a moving moon slowly
        // walk out of the centre of the view, most visibly while zooming out.
        camera.target = if navigation.follow_selected {
            desired_target
        } else {
            smooth_vec3(camera.target, desired_target, CAMERA_SMOOTHING_RATE, delta)
        };
        camera.distance = camera.distance.clamp(min_distance, max_distance);

        let horizontal = camera.distance * camera.pitch.cos();
        let orbit_offset = Vec3::new(
            horizontal * camera.yaw.sin(),
            camera.distance * camera.pitch.sin(),
            horizontal * camera.yaw.cos(),
        );
        transform.translation = camera.target + orbit_offset;
        *transform = transform.looking_at(camera.target, Vec3::Y);

        update_projection_clip_planes(
            &mut projection,
            camera.distance,
            bounds.extent,
            navigation.pan_offset.length(),
        );
    }

    navigation.last_mode = Some(map.mode);
    navigation.last_focus = Some(map.focus);
    navigation.last_selected = Some(map.selected);
}

pub(super) fn visible_physical_body_ids(ephemeris: &BakedEphemeris, map: &MapState) -> Vec<BodyId> {
    ephemeris
        .bodies
        .iter()
        .filter(|body| body.radius_m > 0.0)
        .filter(|body| is_visible_in_view(ephemeris, map, body.id))
        .map(|body| body.id)
        .collect()
}

pub(super) fn cycle_body(bodies: &[BodyId], selected: BodyId, reverse: bool) -> Option<BodyId> {
    if bodies.is_empty() {
        return None;
    }
    let index = bodies.iter().position(|body| *body == selected);
    Some(match index {
        Some(index) if reverse => bodies[(index + bodies.len() - 1) % bodies.len()],
        Some(index) => bodies[(index + 1) % bodies.len()],
        None if reverse => *bodies.last().expect("non-empty body list"),
        None => bodies[0],
    })
}

pub(super) fn request_fit_scope(navigation: &mut NavigationState) {
    navigation.request = NavigationRequest::FitScope;
}

pub(super) fn request_frame_selected(navigation: &mut NavigationState) {
    navigation.request = NavigationRequest::FrameSelected;
}

fn cancel_automatic_camera(navigation: &mut NavigationState, current_target: Vec3) {
    navigation.request = NavigationRequest::None;
    if navigation.follow_selected {
        navigation.follow_selected = false;
        navigation.pan_offset = current_target;
    }
}

fn cancel_frame_request(navigation: &mut NavigationState) {
    // Orbiting and zooming are still performed around the selected body. Only
    // an explicit middle-button pan should release the follow anchor.
    navigation.request = NavigationRequest::None;
}

#[derive(Clone, Copy, Debug)]
struct ScopeBounds {
    extent: f32,
}

fn scope_bounds(ephemeris: &BakedEphemeris, map: &MapState, time: SimTime) -> ScopeBounds {
    let extent = visible_physical_body_ids(ephemeris, map)
        .into_iter()
        .filter_map(|id| {
            let position = map_position(ephemeris, map, id, time)?;
            let radius = ephemeris
                .body(id)
                .ok()
                .map(|body| visual_radius_for_mode(body.radius_m, map.mode))
                .unwrap_or(0.0);
            Some(position.length() + radius)
        })
        .fold(0.0, f32::max);
    ScopeBounds {
        extent: extent.max(CAMERA_MIN_DISTANCE),
    }
}

fn projection_view(projection: &Projection, window: &Window) -> (f32, f32) {
    let window_aspect = if window.height() > 0.0 {
        (window.width() / window.height()).max(0.1)
    } else {
        16.0 / 9.0
    };
    match projection {
        Projection::Perspective(perspective) => (
            perspective.fov.clamp(0.1, 3.0),
            perspective.aspect_ratio.max(window_aspect).max(0.1),
        ),
        _ => (std::f32::consts::FRAC_PI_4, window_aspect),
    }
}

fn limiting_half_fov(fov: f32, aspect: f32) -> f32 {
    let vertical = (fov * 0.5).clamp(0.05, 1.45);
    let horizontal = (vertical.tan() * aspect.max(0.1)).atan();
    vertical.min(horizontal).max(0.02)
}

fn fit_distance_for_extent(extent: f32, fov: f32, aspect: f32) -> f32 {
    extent.max(CAMERA_MIN_DISTANCE) * CAMERA_FIT_MARGIN / limiting_half_fov(fov, aspect).tan()
}

fn frame_distance_for_radius(radius: f32, fov: f32, aspect: f32) -> f32 {
    radius.max(CAMERA_MIN_DISTANCE) * CAMERA_FRAME_MARGIN / limiting_half_fov(fov, aspect).tan()
}

fn zoom_limits(
    ephemeris: &BakedEphemeris,
    map: &MapState,
    extent: f32,
    fov: f32,
    aspect: f32,
) -> (f32, f32) {
    let selected_radius = ephemeris
        .body(map.selected)
        .ok()
        .filter(|body| body.radius_m > 0.0)
        .map(|body| visual_radius_for_mode(body.radius_m, map.mode))
        .unwrap_or(CAMERA_MIN_DISTANCE);
    let min_distance = (selected_radius * 0.42).max(CAMERA_MIN_DISTANCE);
    let fit_distance = fit_distance_for_extent(extent, fov, aspect);
    let max_distance = (fit_distance * CAMERA_MAX_DISTANCE_MULTIPLIER).max(min_distance * 8.0);
    (
        min_distance,
        max_distance.max(min_distance + CAMERA_MIN_DISTANCE),
    )
}

fn clamp_pan_offset(offset: Vec3, limit: f32) -> Vec3 {
    let limit = limit.max(CAMERA_MIN_DISTANCE);
    let length = offset.length();
    if length > limit {
        offset / length * limit
    } else {
        offset
    }
}

fn smooth_scalar(current: f32, target: f32, rate: f32, delta: f32) -> f32 {
    if delta <= 0.0 {
        target
    } else {
        current.lerp(target, 1.0 - (-rate * delta).exp())
    }
}

fn smooth_vec3(current: Vec3, target: Vec3, rate: f32, delta: f32) -> Vec3 {
    current.lerp(
        target,
        if delta <= 0.0 {
            1.0
        } else {
            1.0 - (-rate * delta).exp()
        },
    )
}

fn update_projection_clip_planes(
    projection: &mut Projection,
    camera_distance: f32,
    scope_extent: f32,
    pan_extent: f32,
) {
    let far =
        ((camera_distance + scope_extent + pan_extent + 4.0) * CAMERA_FAR_MARGIN).max(1_000.0);
    let near = (camera_distance * 0.001).clamp(0.001, 0.1).min(far * 0.25);
    match projection {
        Projection::Perspective(perspective) => {
            perspective.near = near;
            perspective.far = far;
        }
        Projection::Orthographic(orthographic) => {
            orthographic.near = -far;
            orthographic.far = far;
        }
        Projection::Custom(_) => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_distance_covers_every_rendered_body_in_checked_in_system() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("the checked-in system configuration must parse");
        let ephemeris = config.bake().expect("the checked-in system must bake");

        for mode in [
            MapMode::SystemOverview,
            MapMode::Asterion,
            MapMode::Nereid,
            MapMode::Orthea,
            MapMode::Vesper,
            MapMode::Binary,
        ] {
            let focus = ephemeris
                .body_id(mode.focus_name())
                .expect("map mode focus must exist");
            let map = MapState {
                mode,
                focus,
                selected: focus,
            };
            let bounds = scope_bounds(&ephemeris, &map, SimTime::EPOCH);
            let fit =
                fit_distance_for_extent(bounds.extent, std::f32::consts::FRAC_PI_4, 16.0 / 9.0);
            let visible_extent = visible_physical_body_ids(&ephemeris, &map)
                .into_iter()
                .map(|id| {
                    let position = map_position(&ephemeris, &map, id, SimTime::EPOCH)
                        .expect("visible body must have a map position");
                    let radius = visual_radius_for_mode(
                        ephemeris.body(id).expect("body descriptor").radius_m,
                        mode,
                    );
                    position.length() + radius
                })
                .fold(0.0, f32::max);
            assert!(
                fit * limiting_half_fov(std::f32::consts::FRAC_PI_4, 16.0 / 9.0).tan()
                    >= visible_extent * CAMERA_FIT_MARGIN - 1.0e-4
            );
            assert!(bounds.extent >= visible_extent - 1.0e-5);
        }
    }

    #[test]
    fn following_selected_body_preserves_scope_origin_and_pan_offset() {
        let config: SystemConfig = toml::from_str(include_str!("../../../data/system.toml"))
            .expect("the checked-in system configuration must parse");
        let ephemeris = config.bake().expect("the checked-in system must bake");
        let focus = ephemeris
            .body_id("nereid")
            .expect("Nereid must exist in the checked-in system");
        let selected = ephemeris
            .body_id("thessa")
            .expect("Thessa must exist in the checked-in system");
        let map = MapState {
            mode: MapMode::Nereid,
            focus,
            selected,
        };
        let at_epoch = map_position(&ephemeris, &map, selected, SimTime::EPOCH)
            .expect("selected body must be visible");
        let later = map_position(&ephemeris, &map, selected, SimTime(86_400.0))
            .expect("selected body must remain visible");
        let pan = Vec3::new(0.4, -0.2, 0.1);
        assert_ne!(at_epoch, later, "Thessa should move on its baked orbit");
        assert!((at_epoch + pan - pan - at_epoch).length() < 1.0e-5);
        assert!((later + pan - pan - later).length() < 1.0e-5);
        assert_ne!(at_epoch + pan, later + pan);
    }

    #[test]
    fn far_follow_target_is_not_clamped_to_a_local_mode_limit() {
        let selected = Vec3::new(50_000.0, 0.0, -25_000.0);
        let pan = Vec3::new(3.0, 2.0, -1.0);
        let target = selected + pan;
        assert_eq!(target, Vec3::new(50_003.0, 2.0, -25_001.0));
        assert_eq!(clamp_pan_offset(pan, 100.0), pan);
    }

    #[test]
    fn zoom_and_orbit_cancel_frame_request_without_releasing_focus_anchor() {
        let mut navigation = NavigationState {
            follow_selected: true,
            request: NavigationRequest::FrameSelected,
            ..default()
        };

        cancel_frame_request(&mut navigation);

        assert!(matches!(navigation.request, NavigationRequest::None));
        assert!(navigation.follow_selected);
        assert_eq!(navigation.pan_offset, Vec3::ZERO);
    }
}
