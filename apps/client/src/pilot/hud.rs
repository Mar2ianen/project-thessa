//! Derived flight instruments. Layout flows within a bounded dock so resizing
//! the window cannot put the speed, altitude and control cards on top of each other.
use super::*;

const NAVBALL_SIZE: f32 = 256.0;
const PANEL: Color = Color::srgba(0.035, 0.052, 0.074, 0.94);
const EDGE: Color = Color::srgba(0.48, 0.60, 0.69, 0.40);

#[derive(Component)]
pub(super) struct PilotHudRoot;
#[derive(Component)]
pub(super) struct HelpPanel;
#[derive(Component)]
pub(super) struct DataPanel;
#[derive(Component)]
pub(super) struct HudSurface;
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

#[derive(Component, Clone, Copy, PartialEq, Eq)]
pub(super) enum Action {
    Speed,
    Altitude,
    Sas,
    Rcs,
    Gear,
    Engine,
    Mode,
    SetMode(ControlMode),
    Map,
    Pause,
    Help,
    Telemetry,
    Camera,
    Precision,
    ThrottleUp,
    ThrottleDown,
}
#[derive(Component)]
pub(super) struct ModePanel;

#[derive(Resource)]
pub(super) struct FlightIcons(std::collections::BTreeMap<&'static str, Handle<Image>>);
impl FlightIcons {
    fn load(assets: &AssetServer) -> Self {
        Self(
            [
                "sas",
                "rcs",
                "gear",
                "engine",
                "mouse",
                "attitude",
                "rate",
                "direct",
                "map",
                "pause",
                "play",
                "help",
                "data",
                "camera",
                "precision",
                "plus",
                "minus",
                "altitude",
                "vertical",
                "speed",
                "warning",
            ]
            .into_iter()
            .map(|name| (name, assets.load(format!("ui/flight/{name}.png"))))
            .collect(),
        )
    }
    fn get(&self, name: &str) -> Handle<Image> {
        self.0[name].clone()
    }
}
#[derive(Component)]
pub(super) struct ActionIcon(Action);
#[derive(Component)]
pub(super) struct StateLamp(Action);

fn mode_icon(mode: ControlMode) -> &'static str {
    match mode {
        ControlMode::MouseAim => "mouse",
        ControlMode::Navball => "attitude",
        ControlMode::Rate => "rate",
        ControlMode::Direct => "direct",
    }
}
fn action_icon(action: Action) -> &'static str {
    match action {
        Action::Sas => "sas",
        Action::Rcs => "rcs",
        Action::Gear => "gear",
        Action::Engine => "engine",
        Action::Mode => "attitude",
        Action::SetMode(mode) => mode_icon(mode),
        Action::Map => "map",
        Action::Pause => "pause",
        Action::Help => "help",
        Action::Telemetry => "data",
        Action::Camera => "camera",
        Action::Precision => "precision",
        Action::ThrottleUp => "plus",
        Action::ThrottleDown => "minus",
        Action::Speed => "speed",
        Action::Altitude => "altitude",
    }
}
fn icon(
    parent: &mut ChildSpawnerCommands<'_>,
    icons: &FlightIcons,
    name: &str,
    size: f32,
    color: Color,
) {
    parent.spawn((
        ImageNode {
            color,
            ..ImageNode::new(icons.get(name))
        },
        Node {
            width: px(size),
            height: px(size),
            flex_shrink: 0.0,
            ..default()
        },
    ));
}
fn button(
    parent: &mut ChildSpawnerCommands<'_>,
    font: &Handle<Font>,
    icons: &FlightIcons,
    action: Action,
    title: &str,
) {
    let caption = matches!(action, Action::SetMode(_));
    parent
        .spawn((
            Button,
            action,
            Node {
                min_width: px(42),
                height: px(40),
                padding: UiRect::axes(px(9), px(7)),
                column_gap: px(10),
                border: UiRect::all(px(1)),
                border_radius: BorderRadius::all(px(6)),
                align_items: AlignItems::Center,
                justify_content: JustifyContent::Center,
                ..default()
            },
            BackgroundColor(PANEL),
            BorderColor::all(EDGE),
        ))
        .with_children(|b| {
            b.spawn((
                ActionIcon(action),
                ImageNode::new(icons.get(action_icon(action))),
                Node {
                    width: px(24),
                    height: px(24),
                    flex_shrink: 0.0,
                    ..default()
                },
            ));
            if caption {
                label(b, font, title, 13.0, HUD_TEXT);
            }
            if matches!(
                action,
                Action::Sas
                    | Action::Rcs
                    | Action::Gear
                    | Action::Engine
                    | Action::Precision
                    | Action::Camera
                    | Action::Telemetry
            ) {
                b.spawn((
                    StateLamp(action),
                    Node {
                        position_type: PositionType::Absolute,
                        right: px(4),
                        bottom: px(4),
                        width: px(4),
                        height: px(4),
                        border_radius: BorderRadius::MAX,
                        ..default()
                    },
                    BackgroundColor(EDGE),
                ));
            }
        });
}
fn action_active(
    action: Action,
    state: &PilotHudState,
    runtime: &PilotFlightRuntime,
    clock: &SimulationClock,
) -> bool {
    match action {
        Action::Sas => runtime.sas_enabled,
        Action::Rcs => runtime.rcs_enabled,
        Action::Gear => runtime.gear_down,
        Action::Engine => runtime.engine_active,
        Action::Pause => clock.paused,
        Action::Help => state.show_help,
        Action::Telemetry => state.show_telemetry,
        Action::Camera => state.pilot_camera_chase,
        Action::Precision => state.precision_controls,
        Action::Mode => state.show_modes,
        Action::SetMode(mode) => state.control_mode == mode,
        _ => false,
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
pub(super) fn pilot_hud_buttons(
    mut state: ResMut<PilotHudState>,
    mut runtime: ResMut<PilotFlightRuntime>,
    mut clock: ResMut<SimulationClock>,
    mut buttons: Query<
        (
            &Interaction,
            &Action,
            &mut BackgroundColor,
            &mut BorderColor,
            Ref<Interaction>,
        ),
        With<Button>,
    >,
    mut panels: Query<&mut Node, With<ModePanel>>,
    icons: Res<FlightIcons>,
    surfaces: Query<&Interaction, With<HudSurface>>,
    mut glyphs: Query<(&ActionIcon, &mut ImageNode)>,
    mut lamps: Query<(&StateLamp, &mut BackgroundColor), Without<Button>>,
) {
    let visible = state.view_mode == ClientViewMode::Pilot && !state.ui_hidden;
    state.pointer_over_ui = visible
        && surfaces
            .iter()
            .any(|interaction| *interaction != Interaction::None);
    for (interaction, action, mut background, mut border, changed) in &mut buttons {
        if visible && *interaction != Interaction::None {
            state.pointer_over_ui = true;
        }
        if visible && *interaction == Interaction::Pressed && changed.is_changed() {
            match action {
                Action::Speed => state.speed_frame = state.speed_frame.next(),
                Action::Altitude => state.altitude_frame = state.altitude_frame.toggle(),
                Action::Sas => runtime.sas_enabled = !runtime.sas_enabled,
                Action::Rcs => runtime.rcs_enabled = !runtime.rcs_enabled,
                Action::Gear => runtime.gear_down = !runtime.gear_down,
                Action::Engine => runtime.engine_active = !runtime.engine_active,
                Action::Mode => state.show_modes = !state.show_modes,
                Action::SetMode(mode) => {
                    state.control_mode = *mode;
                    state.show_modes = false;
                    runtime.sas_target_orientation = runtime.state.orientation_body_to_inertial;
                }
                Action::Map => state.view_mode = ClientViewMode::Map,
                Action::Pause => clock.paused = !clock.paused,
                Action::Help => state.show_help = !state.show_help,
                Action::Telemetry => state.show_telemetry = !state.show_telemetry,
                Action::Camera => state.pilot_camera_chase = !state.pilot_camera_chase,
                Action::Precision => state.precision_controls = !state.precision_controls,
                Action::ThrottleUp => runtime.throttle = (runtime.throttle + 0.1).min(1.0),
                Action::ThrottleDown => runtime.throttle = (runtime.throttle - 0.1).max(0.0),
            }
        }
        let enabled = action_active(*action, &state, &runtime, &clock);
        background.0 = if *interaction == Interaction::Hovered {
            Color::srgb(0.14, 0.23, 0.28)
        } else if enabled {
            Color::srgba(0.08, 0.28, 0.22, 0.95)
        } else {
            PANEL
        };
        *border = BorderColor::all(if enabled { HUD_GREEN } else { EDGE });
    }
    for (glyph, mut image) in &mut glyphs {
        image.color = if action_active(glyph.0, &state, &runtime, &clock) {
            HUD_GREEN
        } else {
            HUD_TEXT
        };
        let name = match glyph.0 {
            Action::Mode => mode_icon(state.control_mode),
            Action::Pause if clock.paused => "play",
            _ => action_icon(glyph.0),
        };
        image.image = icons.get(name);
    }
    for (lamp, mut color) in &mut lamps {
        color.0 = if action_active(lamp.0, &state, &runtime, &clock) {
            HUD_GREEN
        } else {
            EDGE
        };
    }
    for mut panel in &mut panels {
        panel.display = if state.show_modes && visible {
            Display::Flex
        } else {
            Display::None
        };
    }
}

fn cue_node() -> Node {
    Node {
        position_type: PositionType::Absolute,
        width: px(16),
        height: px(16),
        border: UiRect::all(px(2)),
        border_radius: BorderRadius::MAX,
        ..default()
    }
}
fn cue_arms(parent: &mut ChildSpawnerCommands<'_>, retrograde: bool) {
    for (left, top, width, height) in [(-8, 5, 7, 2), (13, 5, 7, 2), (5, -8, 2, 7)] {
        parent.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(left),
                top: px(top),
                width: px(width),
                height: px(height),
                ..default()
            },
            BackgroundColor(HUD_GREEN),
        ));
    }
    if retrograde {
        parent.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(2),
                top: px(5),
                width: px(8),
                height: px(2),
                ..default()
            },
            BackgroundColor(HUD_GREEN),
        ));
        parent.spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(5),
                top: px(2),
                width: px(2),
                height: px(8),
                ..default()
            },
            BackgroundColor(HUD_GREEN),
        ));
    }
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
        padding: UiRect::all(px(12)),
        border: UiRect::all(px(1)),
        border_radius: BorderRadius::all(px(8)),
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
    let icons = FlightIcons::load(&assets);
    let texture = images.add(make_navball_image(DVec3::Z));
    let bezel = assets.load("ui/flight/navball-bezel.png");
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
                ..default()
            },
            FocusPolicy::Pass,
            Visibility::Hidden,
            ZIndex(110),
        ))
        .with_children(|root| {
            root.spawn(Node {
                position_type: PositionType::Absolute,
                top: px(16),
                left: px(18),
                flex_direction: FlexDirection::Column,
                row_gap: px(3),
                ..default()
            })
            .with_children(|mission| {
                label(mission, &font, "THESSA / X-15", 13.0, HUD_TEXT);
                readout(mission, &font, Readout::Header, 11.0, HUD_MUTED);
            });
            root.spawn(Node {
                position_type: PositionType::Absolute,
                bottom: px(14),
                left: px(0),
                right: px(0),
                align_items: AlignItems::Center,
                flex_direction: FlexDirection::Column,
                row_gap: px(8),
                ..default()
            })
            .with_children(|dock| {
                readout(dock, &font, Readout::Status, 13.0, HUD_AMBER);
                dock.spawn((
                    HudSurface,
                    Interaction::None,
                    Node {
                        align_items: AlignItems::End,
                        column_gap: px(8),
                        ..default()
                    },
                ))
                .with_children(|instruments| {
                    instruments
                        .spawn(Node {
                            width: px(164),
                            flex_direction: FlexDirection::Column,
                            row_gap: px(6),
                            ..default()
                        })
                        .with_children(|left| {
                            spawn_data(left, &font, "AIR DATA", Readout::AirData);
                            left.spawn((
                                Node {
                                    padding: UiRect::all(px(10)),
                                    ..card()
                                },
                                BackgroundColor(PANEL),
                                BorderColor::all(EDGE),
                            ))
                            .with_children(|power| {
                                power
                                    .spawn(Node {
                                        align_items: AlignItems::Center,
                                        column_gap: px(7),
                                        ..default()
                                    })
                                    .with_children(|row| {
                                        icon(row, &icons, "engine", 18.0, HUD_MUTED);
                                        label(row, &font, "THRUST", 10.0, HUD_MUTED);
                                    });
                                readout(power, &font, Readout::Propulsion, 28.0, HUD_GREEN);
                                power
                                    .spawn((
                                        Node {
                                            width: percent(100),
                                            height: px(7),
                                            border_radius: BorderRadius::all(px(2)),
                                            margin: UiRect::vertical(px(3)),
                                            ..default()
                                        },
                                        BackgroundColor(Color::srgb(0.015, 0.025, 0.035)),
                                    ))
                                    .with_children(|track| {
                                        track.spawn((
                                            ThrottleFill,
                                            Node {
                                                width: percent(100),
                                                height: percent(100),
                                                border_radius: BorderRadius::all(px(2)),
                                                ..default()
                                            },
                                            BackgroundColor(HUD_GREEN),
                                        ));
                                    });
                                power
                                    .spawn(Node {
                                        column_gap: px(6),
                                        ..default()
                                    })
                                    .with_children(|row| {
                                        button(row, &font, &icons, Action::ThrottleDown, "");
                                        button(row, &font, &icons, Action::ThrottleUp, "");
                                        button(row, &font, &icons, Action::Engine, "");
                                    });
                            });
                            left.spawn((
                                Node {
                                    padding: UiRect::all(px(10)),
                                    row_gap: px(6),
                                    ..card()
                                },
                                BackgroundColor(PANEL),
                                BorderColor::all(EDGE),
                            ))
                            .with_children(|tools| {
                                for actions in [
                                    [Action::Camera, Action::Telemetry, Action::Precision],
                                    [Action::Pause, Action::Map, Action::Help],
                                ] {
                                    tools
                                        .spawn(Node {
                                            column_gap: px(6),
                                            ..default()
                                        })
                                        .with_children(|row| {
                                            for action in actions {
                                                button(row, &font, &icons, action, "");
                                            }
                                        });
                                }
                            });
                        });
                    spawn_navball(instruments, texture.clone(), bezel, &font, &icons);
                    instruments
                        .spawn(Node {
                            width: px(164),
                            flex_direction: FlexDirection::Column,
                            row_gap: px(6),
                            ..default()
                        })
                        .with_children(|right| {
                            spawn_data(right, &font, "ORBIT", Readout::Orbit);
                            right
                                .spawn((
                                    Button,
                                    Action::Altitude,
                                    Node {
                                        align_items: AlignItems::Start,
                                        padding: UiRect::all(px(10)),
                                        ..card()
                                    },
                                    BackgroundColor(PANEL),
                                    BorderColor::all(EDGE),
                                ))
                                .with_children(|alt| {
                                    alt.spawn(Node {
                                        column_gap: px(7),
                                        align_items: AlignItems::Center,
                                        ..default()
                                    })
                                    .with_children(|row| {
                                        icon(row, &icons, "altitude", 18.0, HUD_MUTED);
                                        readout(row, &font, Readout::AltLabel, 10.0, HUD_MUTED);
                                    });
                                    readout(alt, &font, Readout::Altitude, 27.0, HUD_TEXT);
                                    alt.spawn(Node {
                                        column_gap: px(6),
                                        align_items: AlignItems::Center,
                                        ..default()
                                    })
                                    .with_children(|row| {
                                        icon(row, &icons, "vertical", 16.0, HUD_GREEN);
                                        readout(row, &font, Readout::Vertical, 13.0, HUD_GREEN);
                                    });
                                });
                            right
                                .spawn((
                                    Node {
                                        padding: UiRect::all(px(10)),
                                        ..card()
                                    },
                                    BackgroundColor(PANEL),
                                    BorderColor::all(EDGE),
                                ))
                                .with_children(|controls| {
                                    controls
                                        .spawn(Node {
                                            column_gap: px(6),
                                            ..default()
                                        })
                                        .with_children(|row| {
                                            for (action, title) in [
                                                (Action::Sas, "SAS"),
                                                (Action::Rcs, "RCS"),
                                                (Action::Gear, "GEAR"),
                                            ] {
                                                row.spawn(Node {
                                                    align_items: AlignItems::Center,
                                                    flex_direction: FlexDirection::Column,
                                                    row_gap: px(3),
                                                    ..default()
                                                })
                                                .with_children(|toggle| {
                                                    button(toggle, &font, &icons, action, "");
                                                    label(toggle, &font, title, 9.0, HUD_MUTED);
                                                });
                                            }
                                        });
                                });
                            right
                                .spawn((
                                    Button,
                                    Action::Mode,
                                    Node {
                                        height: px(48),
                                        column_gap: px(8),
                                        align_items: AlignItems::Center,
                                        padding: UiRect::all(px(10)),
                                        border: UiRect::all(px(1)),
                                        border_radius: BorderRadius::all(px(8)),
                                        ..default()
                                    },
                                    BackgroundColor(PANEL),
                                    BorderColor::all(EDGE),
                                ))
                                .with_children(|mode| {
                                    mode.spawn((
                                        ActionIcon(Action::Mode),
                                        ImageNode::new(icons.get("attitude")),
                                        Node {
                                            width: px(24),
                                            height: px(24),
                                            flex_shrink: 0.0,
                                            ..default()
                                        },
                                    ));
                                    readout(mode, &font, Readout::Mode, 11.0, HUD_TEXT);
                                });
                        });
                });
                dock.spawn((
                    ModePanel,
                    HudSurface,
                    Interaction::None,
                    Node {
                        position_type: PositionType::Absolute,
                        bottom: px(60),
                        left: percent(50),
                        margin: UiRect::left(px(152)),
                        width: px(244),
                        display: Display::None,
                        padding: UiRect::all(px(8)),
                        ..card()
                    },
                    BackgroundColor(PANEL),
                    BorderColor::all(EDGE),
                    ZIndex(20),
                ))
                .with_children(|modes| {
                    for mode in ControlMode::ALL {
                        button(modes, &font, &icons, Action::SetMode(mode), mode.label());
                    }
                });
            });
            root.spawn((
                HelpPanel,
                HudSurface,
                Interaction::None,
                Node {
                    position_type: PositionType::Absolute,
                    top: px(16),
                    right: px(16),
                    width: px(480),
                    max_width: percent(94),
                    max_height: percent(94),
                    overflow: Overflow::scroll_y(),
                    display: Display::None,
                    ..card()
                },
                BackgroundColor(PANEL),
                BorderColor::all(EDGE),
                ZIndex(30),
            ))
            .with_children(|help| {
                label(help, &font, "FLIGHT CONTROLS", 16.0, HUD_TEXT);
                help.spawn(Node {
                    flex_wrap: FlexWrap::Wrap,
                    column_gap: px(12),
                    row_gap: px(10),
                    margin: UiRect::vertical(px(8)),
                    ..default()
                })
                .with_children(|legend| {
                    for (name, caption) in [
                        ("sas", "SAS · T"),
                        ("rcs", "RCS · R"),
                        ("gear", "Gear · G"),
                        ("engine", "Engine · Space"),
                        ("camera", "Camera · V"),
                        ("data", "Data · F3"),
                        ("precision", "Precision · Caps"),
                        ("pause", "Pause · Esc"),
                        ("map", "Map · M"),
                    ] {
                        legend
                            .spawn(Node {
                                width: px(134),
                                column_gap: px(7),
                                align_items: AlignItems::Center,
                                ..default()
                            })
                            .with_children(|item| {
                                icon(item, &icons, name, 22.0, HUD_GREEN);
                                label(item, &font, caption, 11.0, HUD_TEXT);
                            });
                    }
                });
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
            root.spawn((FlightPathCue, cue_node(), BorderColor::all(HUD_GREEN)))
                .with_children(|cue| cue_arms(cue, false));
        });
    commands.insert_resource(icons);
}

fn spawn_data(
    parent: &mut ChildSpawnerCommands<'_>,
    font: &Handle<Font>,
    title: &str,
    kind: Readout,
) {
    parent
        .spawn((
            DataPanel,
            Node {
                display: Display::None,
                padding: UiRect::all(px(10)),
                ..card()
            },
            BackgroundColor(PANEL),
            BorderColor::all(EDGE),
        ))
        .with_children(|panel| {
            label(panel, font, title, 10.0, HUD_MUTED);
            readout(panel, font, kind, 12.0, HUD_TEXT);
        });
}

fn spawn_navball(
    parent: &mut ChildSpawnerCommands<'_>,
    image: Handle<Image>,
    bezel: Handle<Image>,
    font: &Handle<Font>,
    icons: &FlightIcons,
) {
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
                    Button,
                    Action::Speed,
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
                    speed
                        .spawn(Node {
                            column_gap: px(5),
                            align_items: AlignItems::Center,
                            ..default()
                        })
                        .with_children(|row| {
                            icon(row, icons, "speed", 13.0, HUD_GREEN);
                            readout(row, font, Readout::SpeedLabel, 10.0, HUD_GREEN);
                        });
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
                    ball.spawn((
                        ImageNode::new(bezel),
                        Node {
                            position_type: PositionType::Absolute,
                            width: px(NAVBALL_SIZE),
                            height: px(NAVBALL_SIZE),
                            ..default()
                        },
                    ));
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
                    for retrograde in [false, true] {
                        ball.spawn((
                            VectorMarker { retrograde },
                            cue_node(),
                            BorderColor::all(HUD_GREEN),
                        ))
                        .with_children(|cue| cue_arms(cue, retrograde));
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
    let visible = state.view_mode == ClientViewMode::Pilot && !state.ui_hidden;
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
            node.left = px(p.x - 8.0);
            node.top = px(p.y - 8.0);
        }
    }
    for mut node in &mut nodes.p3() {
        node.width = percent((flight.throttle.unwrap_or(0.0) * 100.0) as f32);
    }
    let direction = flight.surface_velocity_mps.try_normalize();
    for mut node in &mut nodes.p4() {
        let position = direction
            .filter(|_| state.control_mode == ControlMode::MouseAim)
            .and_then(|d| {
                camera
                    .0
                    .world_to_viewport(camera.1, pilot_render_offset(d * 1000.0))
                    .ok()
                    .filter(|p| p.y < window.resolution.height() - NAVBALL_SIZE - 160.0)
            });
        node.display = if position.is_some() {
            Display::Flex
        } else {
            Display::None
        };
        if let Some(p) = position {
            node.left = px(p.x - 8.0);
            node.top = px(p.y - 8.0);
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
            Readout::Mode => format!("{} · {}", state.control_mode.label(), flight.regime.label()),
            Readout::Status => {
                color.0 = if flight.warnings.is_empty() {
                    HUD_MUTED
                } else {
                    HUD_AMBER
                };
                if clock.paused {
                    "PAUSED".into()
                } else {
                    flight.warnings.join("   ·   ")
                }
            }
            Readout::SpeedLabel => state.speed_frame.label().into(),
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
                "ALT · {}",
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
                "{} m/s",
                flight
                    .vertical_speed_m_s
                    .map(|v| format!("{v:+.1}"))
                    .unwrap_or_else(|| "--".into())
            ),
            Readout::Propulsion => format_percent(flight.throttle),
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
            Readout::Help => format!(
                "{}\n\nW / S   Nose down / up\nA / D   Yaw left / right     Q / E   Roll\nShift / Ctrl   Throttle     Z / X   Full / zero\nSpace   Engine     G   Gear     R   RCS\nT   Toggle SAS     Hold F   Invert SAS\nCaps Lock   Precision controls\n\nRMB drag   Free orbit     MMB drag   Pan\nWheel   Zoom     `   Reset camera\nV   Free / chase camera     M   Orbital map\nEsc / F8   Pause     F2   Hide interface\nF3   Extra telemetry     F1   This layout\n\nClick speed: surface / air / orbit / target\nClick altimeter: datum / AGL\nMode button beside navball: control scheme\nSAS / RCS / GEAR: green means enabled\nIcons to the left: camera, data, precision,\npause, map, help. + / −: throttle.\n\nFuel is unlimited; contact and gear forces\nare not yet simulated. F4: performance; Shift+F4: capture.\nF6: surface survey; M: return to map.\nSurvey: Shift+RMB look around; RMB orbit.\nShift+F12: RT / raster (supported GPUs).",
                state.control_mode.description()
            ),
        };
    }
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
