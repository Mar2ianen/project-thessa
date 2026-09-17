//! Contact debug visualization: dynamic bodies, terrain patches, and contact
//! normals drawn from the authoritative [`FlightAuthority::contact_snapshot`].
//!
//! Everything is anchored to the craft (`snapshot - craft`, converted by
//! [`pilot_render_offset`]), so astronomical inertial coordinates never reach
//! f32: the same subtraction the planet backdrop relies on. Terrain boxes use
//! the inertial orientation mapped through the sim->render basis change, so
//! they agree with the unrotated planet backdrop rather than the rotated
//! craft mesh. Drawn only while contact-active; quiet otherwise.

use super::*;
use bevy::math::{DVec3, Isometry3d, Mat3};
use thessa_flight_authority::ContactPartyKind;

use crate::pilot::{PilotFlightRuntime, PilotUpdate, pilot_render_offset};

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(super) struct ContactGizmoConfigGroup;

/// Registers the contact debug layer. Add after `DefaultPlugins`.
pub(super) struct ContactGizmoPlugin;

impl Plugin for ContactGizmoPlugin {
    fn build(&self, app: &mut App) {
        app.insert_gizmo_config(
            ContactGizmoConfigGroup,
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 1.5,
                    perspective: false,
                    joints: GizmoLineJoint::Round(2),
                    ..default()
                },
                depth_bias: -0.03,
                ..default()
            },
        )
        .add_systems(Update, draw_contact_gizmos.in_set(PilotUpdate));
    }
}

/// Sim->render basis change used by [`pilot_render_offset`]:
/// (x, y, z) -> (x, z, -y). Rotations conjugate through it.
fn sim_to_render_mat3(sim: Mat3) -> Mat3 {
    let basis = Mat3::from_cols(Vec3::X, -Vec3::Z, Vec3::Y);
    basis * sim * basis.transpose()
}

fn render_quat(sim_xyzw: [f64; 4]) -> Quat {
    let (x, y, z, w) = (
        sim_xyzw[0] as f32,
        sim_xyzw[1] as f32,
        sim_xyzw[2] as f32,
        sim_xyzw[3] as f32,
    );
    let length_squared = x * x + y * y + z * z + w * w;
    if !length_squared.is_finite() || length_squared <= 1.0e-12 {
        return Quat::IDENTITY;
    }
    let sim = Mat3::from_quat(Quat::from_xyzw(x, y, z, w).normalize());
    Quat::from_mat3(&sim_to_render_mat3(sim))
}

fn anchor(craft_inertial_m: DVec3, point_inertial_m: [f64; 3]) -> Vec3 {
    pilot_render_offset(
        DVec3::new(
            point_inertial_m[0],
            point_inertial_m[1],
            point_inertial_m[2],
        ) - craft_inertial_m,
    )
}

fn draw_contact_gizmos(
    runtime: Res<PilotFlightRuntime>,
    mut gizmos: Gizmos<ContactGizmoConfigGroup>,
) {
    if !runtime.contact_active() {
        return;
    }
    let Ok(snapshot) = runtime.contact_snapshot() else {
        return;
    };
    let craft = runtime.state.position_inertial_m;
    let body_color = Color::srgb(0.3, 0.9, 1.0);
    let patch_color = Color::srgb(1.0, 0.7, 0.2);
    let contact_color = Color::srgb(0.5, 1.0, 0.3);

    for body in snapshot
        .dynamic_bodies
        .iter()
        .chain(snapshot.kinematic_bodies.iter())
    {
        gizmos.sphere(
            Isometry3d::from_translation(anchor(craft, body.position_inertial_m)),
            0.35,
            body_color,
        );
    }
    for patch in &snapshot.patches {
        let center = anchor(craft, patch.center_inertial_m);
        let rotation = render_quat(patch.orientation_xyzw);
        // Corner offsets: R_render * P * v for sim axes v. P maps
        // (x, y, z) -> (x, z, -y), so the Y axis flips into -Z.
        let hx = rotation * Vec3::new(patch.half_extents_m[0] as f32, 0.0, 0.0);
        let hy = rotation * Vec3::new(0.0, 0.0, -(patch.half_extents_m[1] as f32));
        let hz = rotation * Vec3::new(0.0, patch.half_extents_m[2] as f32, 0.0);
        let corners = [
            center - hx - hy - hz,
            center + hx - hy - hz,
            center + hx + hy - hz,
            center - hx + hy - hz,
            center - hx - hy + hz,
            center + hx - hy + hz,
            center + hx + hy + hz,
            center - hx + hy + hz,
        ];
        for (a, b) in [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ] {
            gizmos.line(corners[a], corners[b], patch_color);
        }
    }
    for contact in &snapshot.contacts {
        let start = match contact.a.kind {
            ContactPartyKind::Dynamic | ContactPartyKind::KinematicTerrain => snapshot
                .dynamic_bodies
                .iter()
                .chain(snapshot.kinematic_bodies.iter())
                .find(|body| {
                    body.id_raw == contact.a.id_raw
                        && ((contact.a.kind == ContactPartyKind::Dynamic) != body.kinematic)
                })
                .map(|body| anchor(craft, body.position_inertial_m)),
            ContactPartyKind::StaticTerrain => snapshot
                .patches
                .iter()
                .find(|patch| {
                    patch.party.kind == ContactPartyKind::StaticTerrain
                        && patch.party.id_raw == contact.a.id_raw
                })
                .map(|patch| anchor(craft, patch.center_inertial_m)),
        };
        if let Some(start) = start {
            let normal = DVec3::new(
                contact.normal_inertial[0],
                contact.normal_inertial[1],
                contact.normal_inertial[2],
            );
            let length_m = (0.5 + contact.penetration_m * 20.0).clamp(0.5, 5.0);
            let end = start + pilot_render_offset(normal * length_m);
            gizmos.arrow(start, end, contact_color);
        }
    }
}
