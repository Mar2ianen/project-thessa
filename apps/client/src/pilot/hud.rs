//! Derived flight instruments. Layout flows within a bounded dock so resizing
//! the window cannot put the speed, altitude and control cards on top of each other.
use super::*;

const NAVBALL_SIZE: f32 = 240.0;
const PANEL: Color = Color::srgba(0.035, 0.052, 0.074, 0.94);
const EDGE: Color = Color::srgba(0.48, 0.60, 0.69, 0.40);

#[derive(Component)]
pub(super) struct PilotHudRoot;
#[derive(Component)]
pub(super) struct HelpPanel;
#[derive(Component)]
pub(super) struct DataPanel;
#[derive(Component)]
pub(super) struct PitchLabel(f64);
#[derive(Component)]
pub(super) struct RollPointer;
#[derive(Component)]
pub(super) struct ThrottleFill;
#[derive(Component)]
pub(super) struct AimReticle;
#[derive(Component)]
pub(super) struct FlightPathCue;
#[derive(Component, Clone, Copy)]
pub(super) enum Readout {
    Header,
    Mode,
    Status,
    SpeedLabel,
    Speed,
    AirData,
    AltLabel,
    Altitude,
    Vertical,
    Propulsion,
    Orbit,
    Heading,
    Attitude,
    Help,
}
#[derive(Component)]
pub(super) struct VectorMarker {
    retrograde: bool,
}
#[derive(Resource)]
pub(super) struct NavballTexture {
    image: Handle<Image>,
    last_up: DVec3,
}

fn label(
    parent: &mut ChildSpawnerCommands<'_>,
    font: &Handle<Font>,
    value: &str,
    size: f32,
    color: Color,
) -> Entity {
    parent
        .spawn((
            Text::new(value),
            TextFont {
                font: font.clone().into(),
                font_size: FontSize::Px(size),
                ..default()
            },
            TextColor(color),
            Node {
                flex_shrink: 0.0,
                ..default()
            },
        ))
        .id()
}
fn readout(
    parent: &mut ChildSpawnerCommands<'_>,
    font: &Handle<Font>,
    kind: Readout,
    size: f32,
    color: Color,
) {
    parent.spawn((
        kind,
        Text::new(""),
        TextFont {
            font: font.clone().into(),
            font_size: FontSize::Px(size),
            ..default()
        },
        TextColor(color),
        Node { ..default() },
    ));
}
fn card() -> Node {
    Node {
        padding: UiRect::all(px(16)),
        border: UiRect::all(px(1)),
        border_radius: BorderRadius::all(px(12)),
        flex_direction: FlexDirection::Column,
        row_gap: px(6),
        ..default()
    }
}

pub(super) fn spawn_pilot_hud(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    assets: Res<AssetServer>,
) {
    let font = assets.load("fonts/NotoSans-Regular.ttf");
    let texture = images.add(make_navball_image(DVec3::Z));
    commands.insert_resource(NavballTexture {
        image: texture.clone(),
        last_up: DVec3::Z,
    });
    commands
        .spawn((
            PilotHudRoot,
            Node {
                position_type: PositionType::Absolute,
                width: percent(100),
                height: percent(100),
                padding: UiRect::all(px(16)),
                ..default()
            },
            FocusPolicy::Pass,
            Visibility::Hidden,
            ZIndex(110),
        ))
        .with_children(|root| {
            root.spawn(Node {
                width: percent(100),
                align_items: AlignItems::Start,
                column_gap: px(12),
                ..default()
            })
            .with_children(|top| {
                top.spawn(Node {
                    flex_grow: 1.0,
                    flex_basis: px(0),
                    min_width: px(0),
                    flex_direction: FlexDirection::Column,
                    row_gap: px(6),
                    ..default()
                })
                .with_children(|left| {
                    label(left, &font, "THESSA  /  X-15", 14.0, HUD_TEXT);
                    readout(left, &font, Readout::Header, 12.0, HUD_GREEN);
                });
                top.spawn((
                    Node {
                        width: px(232),
                        align_items: AlignItems::Center,
                        padding: UiRect::axes(px(12), px(8)),
                        border: UiRect::all(px(2)),
                        border_radius: BorderRadius::all(px(7)),
                        flex_direction: FlexDirection::Column,
                        ..default()
                    },
                    BackgroundColor(PANEL),
                    BorderColor::all(EDGE),
                ))
                .with_children(|alt| {
                    readout(alt, &font, Readout::AltLabel, 11.0, HUD_MUTED);
                    readout(alt, &font, Readout::Altitude, 32.0, HUD_TEXT);
                    readout(alt, &font, Readout::Vertical, 12.0, HUD_MUTED);
                });
                top.spawn(Node {
                    flex_grow: 1.0,
                    flex_basis: px(0),
                    min_width: px(0),
                    align_items: AlignItems::End,
                    flex_direction: FlexDirection::Column,
                    ..default()
                })
                .with_children(|right| {
                    readout(right, &font, Readout::Mode, 12.0, HUD_MUTED);
                });
            });
            root.spawn(Node {
                position_type: PositionType::Absolute,
                top: px(128),
                left: px(16),
                right: px(16),
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|status| {
                readout(status, &font, Readout::Status, 15.0, HUD_AMBER);
            });
            root.spawn(Node {
                position_type: PositionType::Absolute,
                bottom: px(32),
                left: px(0),
                right: px(0),
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|center| {
                spawn_navball(center, texture.clone(), &font);
            });
            root.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    left: px(16),
                    bottom: px(42),
                    width: px(132),
                    padding: UiRect::all(px(10)),
                    ..card()
                },
                BackgroundColor(PANEL),
                BorderColor::all(EDGE),
            ))
            .with_children(|engine| {
                label(engine, &font, "THROTTLE", 11.0, HUD_MUTED);
                readout(engine, &font, Readout::Propulsion, 16.0, HUD_GREEN);
                engine
                    .spawn((
                        Node {
                            width: percent(100),
                            height: px(5),
                            border_radius: BorderRadius::MAX,
                            ..default()
                        },
                        BackgroundColor(EDGE),
                    ))
                    .with_children(|track| {
                        track.spawn((
                            ThrottleFill,
                            Node {
                                height: percent(100),
                                width: percent(100),
                                border_radius: BorderRadius::MAX,
                                ..default()
                            },
                            BackgroundColor(HUD_GREEN),
                        ));
                    });
                label(engine, &font, "Shift / Ctrl · X / Z", 10.0, HUD_MUTED);
            });
            for (right, title, kind) in [
                (false, "AIR DATA", Readout::AirData),
                (true, "ORBIT / FLIGHT", Readout::Orbit),
            ] {
                root.spawn((
                    DataPanel,
                    Node {
                        position_type: PositionType::Absolute,
                        top: percent(34),
                        left: if right { Val::Auto } else { px(16) },
                        right: if right { px(16) } else { Val::Auto },
                        width: px(178),
                        display: Display::None,
                        ..card()
                    },
                    BackgroundColor(PANEL),
                    BorderColor::all(EDGE),
                ))
                .with_children(|panel| {
                    label(panel, &font, title, 12.0, HUD_TEXT);
                    readout(panel, &font, kind, 14.0, HUD_MUTED);
                    if right {
                        readout(panel, &font, Readout::Attitude, 12.0, HUD_MUTED);
                    }
                });
            }
            root.spawn(Node {
                position_type: PositionType::Absolute,
                bottom: px(9),
                left: px(16),
                right: px(16),
                justify_content: JustifyContent::Center,
                ..default()
            })
            .with_children(|footer| {
                label(
                    footer,
                    &font,
                    "F1 Controls    F3 Telemetry    F8 Pause    F6 Map",
                    11.0,
                    HUD_MUTED,
                );
            });
            root.spawn((
                HelpPanel,
                Node {
                    position_type: PositionType::Absolute,
                    top: px(128),
                    right: px(16),
                    width: px(350),
                    max_width: percent(94),
                    max_height: percent(75),
                    overflow: Overflow::scroll_y(),
                    display: Display::None,
                    ..card()
                },
                BackgroundColor(PANEL),
                BorderColor::all(EDGE),
                ZIndex(10),
            ))
            .with_children(|help| {
                label(help, &font, "FLIGHT CONTROLS", 16.0, HUD_TEXT);
                readout(help, &font, Readout::Help, 13.0, HUD_MUTED);
            });
            root.spawn((
                AimReticle,
                Node {
                    position_type: PositionType::Absolute,
                    width: px(22),
                    height: px(22),
                    border: UiRect::all(px(1)),
                    border_radius: BorderRadius::MAX,
                    ..default()
                },
                BorderColor::all(HUD_GREEN),
            ));
            root.spawn((
                FlightPathCue,
                Text::new("⊙"),
                TextFont {
                    font: font.clone().into(),
                    font_size: FontSize::Px(24.0),
                    ..default()
                },
                TextColor(HUD_GREEN),
                Node {
                    position_type: PositionType::Absolute,
                    display: Display::None,
                    ..default()
                },
            ));
        });
}

fn spawn_navball(parent: &mut ChildSpawnerCommands<'_>, image: Handle<Image>, font: &Handle<Font>) {
    parent
        .spawn(Node {
            width: px(NAVBALL_SIZE + 20.0),
            flex_shrink: 0.0,
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            ..default()
        })
        .with_children(|instrument| {
            instrument
                .spawn((
                    Node {
                        width: px(156),
                        align_items: AlignItems::Center,
                        flex_direction: FlexDirection::Column,
                        padding: UiRect::axes(px(10), px(4)),
                        border: UiRect::all(px(2)),
                        border_radius: BorderRadius::all(px(6)),
                        margin: UiRect::bottom(px(-4)),
                        ..default()
                    },
                    BackgroundColor(PANEL),
                    BorderColor::all(EDGE),
                    ZIndex(2),
                ))
                .with_children(|speed| {
                    readout(speed, font, Readout::SpeedLabel, 11.0, HUD_GREEN);
                    readout(speed, font, Readout::Speed, 23.0, HUD_GREEN);
                });
            instrument
                .spawn((
                    Node {
                        width: px(NAVBALL_SIZE),
                        height: px(NAVBALL_SIZE),
                        border_radius: BorderRadius::MAX,
                        overflow: Overflow::clip(),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.20, 0.25, 0.29)),
                ))
                .with_children(|ball| {
                    ball.spawn((
                        ImageNode::new(image),
                        Node {
                            position_type: PositionType::Absolute,
                            left: px(12),
                            top: px(12),
                            width: px(NAVBALL_SIZE - 24.0),
                            height: px(NAVBALL_SIZE - 24.0),
                            ..default()
                        },
                    ));
                    for degrees in [-60.0_f32, -30.0, 0.0, 30.0, 60.0] {
                        let angle = degrees.to_radians();
                        let r = NAVBALL_SIZE * 0.5 - 6.0;
                        ball.spawn((
                            Node {
                                position_type: PositionType::Absolute,
                                left: px(NAVBALL_SIZE * 0.5 + angle.sin() * r - 2.0),
                                top: px(NAVBALL_SIZE * 0.5 - angle.cos() * r - 2.0),
                                width: px(4),
                                height: px(4),
                                border_radius: BorderRadius::MAX,
                                ..default()
                            },
                            BackgroundColor(HUD_TEXT),
                        ));
                    }
                    ball.spawn((
                        RollPointer,
                        Node {
                            position_type: PositionType::Absolute,
                            width: px(5),
                            height: px(5),
                            border_radius: BorderRadius::MAX,
                            ..default()
                        },
                        BackgroundColor(HUD_AMBER),
                    ));
                    for pitch in [-60.0_f64, -30.0, 0.0, 30.0, 60.0] {
                        ball.spawn((
                            PitchLabel(pitch),
                            Text::new(format!("{pitch:+.0}")),
                            TextFont {
                                font: font.clone().into(),
                                font_size: FontSize::Px(10.0),
                                ..default()
                            },
                            TextColor(Color::srgba(0.9, 0.96, 1.0, 0.75)),
                            Node {
                                position_type: PositionType::Absolute,
                                ..default()
                            },
                        ));
                    }
                    for (left, width) in [
                        (NAVBALL_SIZE * 0.5 - 34.0, 26.0),
                        (NAVBALL_SIZE * 0.5 + 8.0, 26.0),
                    ] {
                        ball.spawn((
                            Node {
                                position_type: PositionType::Absolute,
                                left: px(left),
                                top: px(NAVBALL_SIZE * 0.5),
                                width: px(width),
                                height: px(2),
                                ..default()
                            },
                            BackgroundColor(HUD_AMBER),
                        ));
                    }
                    ball.spawn((
                        Node {
                            position_type: PositionType::Absolute,
                            left: px(NAVBALL_SIZE * 0.5 - 4.0),
                            top: px(NAVBALL_SIZE * 0.5 - 4.0),
                            width: px(8),
                            height: px(8),
                            border: UiRect::all(px(2)),
                            border_radius: BorderRadius::MAX,
                            ..default()
                        },
                        BorderColor::all(HUD_AMBER),
                    ));
                    for (retrograde, glyph) in [(false, "⊙"), (true, "⊗")] {
                        ball.spawn((
                            VectorMarker { retrograde },
                            Text::new(glyph),
                            TextFont {
                                font: font.clone().into(),
                                font_size: FontSize::Px(22.0),
                                ..default()
                            },
                            TextColor(HUD_GREEN),
                            Node {
                                position_type: PositionType::Absolute,
                                ..default()
                            },
                        ));
                    }
                });
            instrument
                .spawn((
                    Node {
                        padding: UiRect::axes(px(16), px(3)),
                        margin: UiRect::top(px(-7)),
                        border: UiRect::all(px(2)),
                        border_radius: BorderRadius::all(px(5)),
                        ..default()
                    },
                    BackgroundColor(PANEL),
                    BorderColor::all(EDGE),
                    ZIndex(2),
                ))
                .with_children(|heading| readout(heading, font, Readout::Heading, 14.0, HUD_GREEN));
        });
}

fn marker_direction(flight: &FlightUiState, frame: SpeedFrame) -> Option<DVec3> {
    // Markers and the selected speed always use the same reference vector.
    let inertial = match frame {
        SpeedFrame::Surface | SpeedFrame::Air => flight.surface_velocity_mps,
        SpeedFrame::Orbital => flight.orbital_velocity_mps,
        SpeedFrame::Target => return None,
    };
    if inertial.length() < 1.0 {
        return None;
    }
    Some((flight.orientation.inverse() * inertial).normalize())
}

fn marker_position(direction: DVec3) -> Option<Vec2> {
    if direction.x < 0.0 {
        return None;
    }
    // Same orthographic projection as the texture, including circular bounds.
    let radius = (NAVBALL_SIZE - 24.0) * 0.5;
    Some(
        Vec2::splat(NAVBALL_SIZE * 0.5)
            + Vec2::new(-direction.y as f32, -direction.z as f32) * radius,
    )
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub(super) fn update_pilot_hud(
    state: Res<PilotHudState>,
    clock: Res<SimulationClock>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<Camera3d>>,
    runtime: Res<PilotFlightRuntime>,
    mut texture: ResMut<NavballTexture>,
    mut images: ResMut<Assets<Image>>,
    mut roots: Query<&mut Visibility, With<PilotHudRoot>>,
    mut texts: Query<(&mut Text, &mut TextColor, &Readout)>,
    mut nodes: ParamSet<(
        Query<&mut Node, With<HelpPanel>>,
        Query<&mut Node, With<AimReticle>>,
        Query<(&mut Node, &VectorMarker)>,
        Query<&mut Node, With<ThrottleFill>>,
        Query<&mut Node, With<FlightPathCue>>,
        Query<&mut Node, With<DataPanel>>,
        Query<(&mut Node, &PitchLabel)>,
        Query<&mut Node, With<RollPointer>>,
    )>,
) {
    let visible = state.view_mode == ClientViewMode::Pilot;
    for mut root in &mut roots {
        *root = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    if !visible {
        return;
    }
    let flight = &state.flight;
    // Do not regenerate/upload 256x256 pixels on every paused render frame.
    if texture.last_up.distance_squared(flight.local_up_body) > 1.0e-8 {
        if let Some(mut image) = images.get_mut(&texture.image) {
            update_navball_image(&mut image, flight.local_up_body);
        }
        texture.last_up = flight.local_up_body;
    }
    for mut node in &mut nodes.p0() {
        node.display = if state.show_help {
            Display::Flex
        } else {
            Display::None
        };
    }
    for mut node in &mut nodes.p1() {
        node.display = if state.control_mode == ControlMode::MouseAim && window.focused {
            Display::Flex
        } else {
            Display::None
        };
        node.left = px(state.mouse_position.x - 11.0);
        node.top = px(state.mouse_position.y - 11.0);
    }
    for (mut node, marker) in &mut nodes.p2() {
        let position = marker_direction(flight, state.speed_frame)
            .and_then(|d| marker_position(if marker.retrograde { -d } else { d }));
        node.display = if position.is_some() {
            Display::Flex
        } else {
            Display::None
        };
        if let Some(p) = position {
            node.left = px(p.x - 11.0);
            node.top = px(p.y - 15.0);
        }
    }
    for mut node in &mut nodes.p3() {
        node.width = percent((flight.throttle.unwrap_or(0.0) * 100.0) as f32);
    }
    let direction = flight.surface_velocity_mps.try_normalize();
    for mut node in &mut nodes.p4() {
        let position = direction.and_then(|d| {
            camera
                .0
                .world_to_viewport(
                    camera.1,
                    runtime.render_position + pilot_render_offset(d * 1000.0),
                )
                .ok()
        });
        node.display = if position.is_some() {
            Display::Flex
        } else {
            Display::None
        };
        if let Some(p) = position {
            node.left = px(p.x - 12.0);
            node.top = px(p.y - 16.0);
        }
    }
    for mut node in &mut nodes.p5() {
        node.display = if state.show_telemetry {
            Display::Flex
        } else {
            Display::None
        };
    }
    for (mut node, pitch) in &mut nodes.p6() {
        let up = flight.local_up_body.normalize_or_zero();
        let meridian = (up - DVec3::X * up.x).normalize_or_zero();
        let theta = pitch.0.to_radians() - up.x.clamp(-1.0, 1.0).asin();
        let position = if meridian.length_squared() > 0.5 {
            marker_position(DVec3::X * theta.cos() + meridian * theta.sin())
        } else {
            None
        };
        node.display = if position.is_some() {
            Display::Flex
        } else {
            Display::None
        };
        if let Some(p) = position {
            node.left = px(p.x + 10.0);
            node.top = px(p.y - 8.0);
        }
    }
    for mut node in &mut nodes.p7() {
        let roll = flight.roll_deg.unwrap_or(0.0).to_radians() as f32;
        let radius = NAVBALL_SIZE * 0.5 - 6.0;
        node.left = px(NAVBALL_SIZE * 0.5 + roll.sin() * radius - 2.5);
        node.top = px(NAVBALL_SIZE * 0.5 - roll.cos() * radius - 2.5);
    }
    let (_, hours, minutes, seconds) = format_sim_time(flight.sim_time_s);
    for (mut text, mut color, kind) in &mut texts {
        **text = match kind {
            Readout::Header => format!(
                "T+{hours:02}:{minutes:02}:{seconds:02}\n{} · {}",
                flight.environment.reference_body.as_deref().unwrap_or("--"),
                if clock.paused {
                    "PAUSED"
                } else {
                    "FLIGHT TEST"
                }
            ),
            Readout::Mode => format!(
                "{}  [M]\nSAS {}  ·  RCS {}",
                state.control_mode.label(),
                on_off(flight.sas_enabled),
                on_off(flight.rcs_enabled)
            ),
            Readout::Status => {
                color.0 = if flight.warnings.is_empty() {
                    HUD_MUTED
                } else {
                    HUD_AMBER
                };
                if clock.paused {
                    "PAUSED  /  F8 to resume".into()
                } else {
                    flight.warnings.join("   ·   ")
                }
            }
            Readout::SpeedLabel => format!("{}   [V]", state.speed_frame.label()),
            Readout::Speed => format!(
                "{} m/s",
                format_speed_value(primary_speed(flight, state.speed_frame))
            ),
            Readout::AirData => format!(
                "Mach {}\nLoad {} g\nAoA {}\nq {}",
                format_scalar(flight.mach, 2),
                format_scalar(flight.g_load, 1),
                format_angle(flight.angle_of_attack_deg),
                format_pressure(flight.dynamic_pressure_pa)
            ),
            Readout::AltLabel => format!(
                "{} ALTITUDE   [B]",
                if state.altitude_frame == AltitudeFrame::Datum {
                    "DATUM"
                } else {
                    "AGL"
                }
            ),
            Readout::Altitude => {
                format_altitude_value(primary_altitude(flight, state.altitude_frame))
            }
            Readout::Vertical => format!(
                "V/S {} m/s",
                flight
                    .vertical_speed_m_s
                    .map(|v| format!("{v:+.1}"))
                    .unwrap_or_else(|| "--".into())
            ),
            Readout::Propulsion => format!(
                "{}  ·  {}",
                format_percent(flight.throttle),
                if flight.engine_active { "ON" } else { "OFF" }
            ),
            Readout::Orbit => format!(
                "AP {}\nPE {}\nAGL {}\nThrust {}\nGear {}",
                format_altitude_value(flight.apoapsis_altitude_m),
                format_altitude_value(flight.periapsis_altitude_m),
                format_altitude_value(flight.altitude_agl_m),
                format_force(flight.thrust_n),
                if flight.gear_down { "down" } else { "up" }
            ),
            Readout::Heading => format!(
                "HDG {:03.0}°  {}",
                flight.heading_deg.unwrap_or(0.0),
                heading_cardinal(flight.heading_deg)
            ),
            Readout::Attitude => format!(
                "LOCAL   ·   P {:+.0}°   R {:+.0}°",
                flight.pitch_deg.unwrap_or(0.0),
                flight.roll_deg.unwrap_or(0.0)
            ),
            Readout::Help => format!(
                "{}\n\nW / S   Nose down / up\nA / D   Yaw left / right\nQ / E   Roll\nShift / Ctrl   Throttle\nZ / X   Full throttle / cutoff\nSpace   Toggle engine\nT / R   SAS / RCS       G   Gear\n\nRMB drag   Orbit camera\nMMB drag   Pan       Wheel   Zoom\nHome   Reset camera\n\nV   Speed frame       B   Datum / AGL\nM / Shift+M   Flight mode\nF3   Extra telemetry\nF8   Pause       F6   Map       F1   Close",
                state.control_mode.description()
            ),
        };
    }
}

fn on_off(value: bool) -> &'static str {
    if value { "ON" } else { "OFF" }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn vector_cues_use_selected_reference_and_hide_unavailable_directions() {
        let mut flight = FlightUiState::demo_preview(0.0, None);
        flight.orientation = DQuat::IDENTITY;
        flight.surface_velocity_mps = DVec3::X * 200.0;
        flight.orbital_velocity_mps = DVec3::Y * 300.0;
        assert_eq!(
            marker_direction(&flight, SpeedFrame::Surface),
            Some(DVec3::X)
        );
        assert_eq!(
            marker_direction(&flight, SpeedFrame::Orbital),
            Some(DVec3::Y)
        );
        assert!(marker_direction(&flight, SpeedFrame::Target).is_none());
        assert!(marker_position(DVec3::NEG_X).is_none());
        assert_eq!(
            marker_position(DVec3::X),
            Some(Vec2::splat(NAVBALL_SIZE * 0.5))
        );
        flight.surface_velocity_mps = DVec3::ZERO;
        assert!(marker_direction(&flight, SpeedFrame::Surface).is_none());
    }
}
