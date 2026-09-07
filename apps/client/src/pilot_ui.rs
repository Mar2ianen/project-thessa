use super::*;

use bevy::ui::FocusPolicy;

/// Client-facing read model for the primary flight display.
///
/// This is deliberately derived telemetry, not authoritative vehicle state. A
/// future flight/runtime adapter should compute it from the authoritative f64
/// simulation snapshot and publish it to the client. Values stay in SI units
/// until formatting at the UI boundary.
#[derive(Debug, Clone)]
pub(super) struct PilotTelemetrySnapshot {
    pub(super) vehicle_label: String,
    pub(super) attitude_reference: &'static str,
    pub(super) velocity_reference: &'static str,
    pub(super) heading_deg: Option<f64>,
    pub(super) pitch_deg: Option<f64>,
    pub(super) roll_deg: Option<f64>,
    pub(super) altitude_datum_m: Option<f64>,
    pub(super) altitude_agl_m: Option<f64>,
    pub(super) vertical_speed_m_s: Option<f64>,
    pub(super) surface_speed_m_s: Option<f64>,
    pub(super) orbital_speed_m_s: Option<f64>,
    pub(super) true_airspeed_m_s: Option<f64>,
    pub(super) mach: Option<f64>,
    pub(super) dynamic_pressure_pa: Option<f64>,
    pub(super) throttle: Option<f64>,
    pub(super) g_load: Option<f64>,
    pub(super) angle_of_attack_deg: Option<f64>,
    pub(super) sideslip_deg: Option<f64>,
    pub(super) apoapsis_altitude_m: Option<f64>,
    pub(super) periapsis_altitude_m: Option<f64>,
}

/// Bridge point between the future vehicle snapshot adapter and the Bevy HUD.
///
/// `preview_visible` exists so the HUD layout can be iterated before the first
/// controllable vehicle is wired in. Preview mode never fabricates physics: it
/// renders unavailable values as `--`.
#[derive(Resource, Default)]
pub(super) struct PilotHudState {
    pub(super) telemetry: Option<PilotTelemetrySnapshot>,
    preview_visible: bool,
}

#[derive(Component)]
struct PilotHudRoot;

#[derive(Component)]
struct PilotHudReadout;

/// Minimal PFD shell. The final attitude sphere/tapes will replace parts of
/// this textual readout without changing the telemetry contract.
pub(super) struct PilotHudPlugin;

impl Plugin for PilotHudPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PilotHudState>()
            .add_systems(Startup, spawn_pilot_hud)
            .add_systems(Update, (toggle_pilot_hud_preview, update_pilot_hud).chain());
    }
}

fn spawn_pilot_hud(mut commands: Commands) {
    commands
        .spawn((
            PilotHudRoot,
            UiInputBlocker,
            Node {
                position_type: PositionType::Absolute,
                bottom: px(12),
                left: px(12),
                max_width: px(820),
                padding: UiRect::all(px(10)),
                border: UiRect::all(px(1)),
                border_radius: BorderRadius::all(px(4)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.006, 0.014, 0.030, 0.92)),
            BorderColor::all(Color::srgba(0.20, 0.48, 0.64, 0.92)),
            FocusPolicy::Block,
            Visibility::Hidden,
            ZIndex(110),
        ))
        .with_children(|parent| {
            parent.spawn((
                PilotHudReadout,
                Text::new(""),
                TextFont {
                    font_size: FontSize::Px(13.0),
                    ..default()
                },
                TextColor(Color::srgb(0.82, 0.92, 0.98)),
            ));
        });
}

fn toggle_pilot_hud_preview(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<PilotHudState>,
) {
    if keys.just_pressed(KeyCode::F6) {
        state.preview_visible = !state.preview_visible;
    }
}

fn update_pilot_hud(
    state: Res<PilotHudState>,
    mut roots: Query<&mut Visibility, With<PilotHudRoot>>,
    mut readouts: Query<&mut Text, With<PilotHudReadout>>,
) {
    let visible = state.preview_visible || state.telemetry.is_some();
    for mut visibility in &mut roots {
        *visibility = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    if !visible {
        return;
    }

    let content = state
        .telemetry
        .as_ref()
        .map(format_telemetry)
        .unwrap_or_else(preview_text);
    for mut text in &mut readouts {
        **text = content.clone();
    }
}

fn preview_text() -> String {
    "PILOT / PRIMARY FLIGHT DISPLAY  [F6] close\n\
NO VEHICLE TELEMETRY — layout/contract preview only\n\
ATT REF --       VEL REF --\n\
HDG --           PITCH --         ROLL --\n\
ALT DATUM --     AGL --           V/S --\n\
SPD SURF --      ORBIT --         AIR --    MACH --\n\
AOA --           BETA --          Q --\n\
THR --           G --             APO --    PERI --"
        .into()
}

fn format_telemetry(snapshot: &PilotTelemetrySnapshot) -> String {
    format!(
        "PILOT / {}  [F6] preview\n\
ATT REF {:<8}  VEL REF {:<8}\n\
HDG {:>8}  PITCH {:>8}  ROLL {:>8}\n\
ALT DATUM {:>10}  AGL {:>10}  V/S {:>10}\n\
SPD SURF {:>10}  ORBIT {:>10}  AIR {:>10}  MACH {:>6}\n\
AOA {:>8}  BETA {:>8}  Q {:>10}\n\
THR {:>7}  G {:>7}  APO {:>10}  PERI {:>10}",
        snapshot.vehicle_label,
        snapshot.attitude_reference,
        snapshot.velocity_reference,
        format_angle(snapshot.heading_deg),
        format_angle(snapshot.pitch_deg),
        format_angle(snapshot.roll_deg),
        format_distance(snapshot.altitude_datum_m),
        format_distance(snapshot.altitude_agl_m),
        format_speed(snapshot.vertical_speed_m_s),
        format_speed(snapshot.surface_speed_m_s),
        format_speed(snapshot.orbital_speed_m_s),
        format_speed(snapshot.true_airspeed_m_s),
        format_scalar(snapshot.mach, 2),
        format_angle(snapshot.angle_of_attack_deg),
        format_angle(snapshot.sideslip_deg),
        format_pressure(snapshot.dynamic_pressure_pa),
        format_percent(snapshot.throttle),
        format_scalar(snapshot.g_load, 2),
        format_distance(snapshot.apoapsis_altitude_m),
        format_distance(snapshot.periapsis_altitude_m),
    )
}

fn format_angle(value: Option<f64>) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1}°"))
        .unwrap_or_else(|| "--".into())
}

fn format_distance(value_m: Option<f64>) -> String {
    value_m
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() >= 100_000.0 {
                format!("{:.1} km", value / 1_000.0)
            } else {
                format!("{value:.0} m")
            }
        })
        .unwrap_or_else(|| "--".into())
}

fn format_speed(value_m_s: Option<f64>) -> String {
    value_m_s
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.1} m/s"))
        .unwrap_or_else(|| "--".into())
}

fn format_pressure(value_pa: Option<f64>) -> String {
    value_pa
        .filter(|value| value.is_finite())
        .map(|value| {
            if value.abs() >= 1_000.0 {
                format!("{:.2} kPa", value / 1_000.0)
            } else {
                format!("{value:.0} Pa")
            }
        })
        .unwrap_or_else(|| "--".into())
}

fn format_percent(value: Option<f64>) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{:.0}%", value * 100.0))
        .unwrap_or_else(|| "--".into())
}

fn format_scalar(value: Option<f64>, precision: usize) -> String {
    value
        .filter(|value| value.is_finite())
        .map(|value| format!("{value:.precision$}"))
        .unwrap_or_else(|| "--".into())
}
