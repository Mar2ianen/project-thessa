use super::*;

#[test]
fn known_quarter_turn_translates_points_but_not_directions() {
    let frame = BodyFrame::new(
        DVec3::new(10.0, 20.0, 30.0),
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        DVec3::new(9.0, 18.0, 27.0),
    )
    .unwrap();
    let point = frame.body_to_render_point(DVec3::X);
    assert!((point - DVec3::new(1.0, 3.0, 3.0)).length() < 1e-14);
    assert!((frame.body_to_render_direction(DVec3::X) - DVec3::Y).length() < 1e-14);
    assert!((frame.render_to_body_point(point) - DVec3::X).length() < 1e-14);
}

#[test]
fn astronomical_common_translation_is_removed_before_rotation() {
    let spin = DQuat::from_rotation_y(0.923);
    let shift = DVec3::new(1.5e11, -1e12, 2e12);
    let offset = DVec3::new(0.03125, -128.0, 1024.0);
    let local = BodyFrame::new(offset, spin, DVec3::ZERO).unwrap();
    let far = BodyFrame::new(shift + offset, spin, shift).unwrap();
    assert_eq!(far, local);
    let point = DVec3::new(0.015625, 3_200_000.0, -3.0);
    assert_eq!(
        local.body_to_render_point(point),
        far.body_to_render_point(point)
    );
    assert!((far.render_to_body_point(far.body_to_render_point(point)) - point).length() < 3e-9);
}

#[test]
fn moving_origin_and_body_rotation_share_one_camera_mapping() {
    let projection = DMat4::from_scale(DVec3::new(2.0, 3.0, 1.0));
    for step in 0..100 {
        let rotation = DQuat::from_rotation_y(step as f64 * 0.019);
        let center = DVec3::new(1.5e11 + step as f64 * 1e5, -1e12, 2e12);
        let frame =
            BodyFrame::new(center, rotation, center + DVec3::new(64.0, -128.0, 256.0)).unwrap();
        let eye_body = DVec3::new(10.0, 100.0, 3_230_000.0);
        let orientation_body = DQuat::from_rotation_x(0.37);
        let eye_render = frame.body_to_render_point(eye_body);
        let render_view = ViewParams {
            world_from_view: DMat4::from_rotation_translation(
                rotation * orientation_body,
                eye_render,
            ),
            world_position_m: eye_render,
            clip_from_view: projection,
            exposure: 0.25,
        };
        let body_view = frame.view_in_body(&render_view);
        assert!((body_view.world_position_m - eye_body).length() < 4e-9);
        assert!((body_view.world_from_view.w_axis.truncate() - eye_body).length() < 4e-9);
        let forward = body_view.world_from_view.transform_vector3(DVec3::NEG_Z);
        assert!((forward - orientation_body * DVec3::NEG_Z).length() < 1e-14);
        assert_eq!(body_view.clip_from_view, projection);
        assert_eq!(body_view.exposure, 0.25);
    }
}

#[test]
fn invalid_pose_and_subtraction_overflow_are_rejected() {
    for (body, q, origin) in [
        (DVec3::NAN, DQuat::IDENTITY, DVec3::ZERO),
        (DVec3::ZERO, DQuat::IDENTITY, DVec3::INFINITY),
        (DVec3::ZERO, DQuat::NAN, DVec3::ZERO),
        (
            DVec3::ZERO,
            DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0),
            DVec3::ZERO,
        ),
        (
            DVec3::splat(f64::MAX),
            DQuat::IDENTITY,
            DVec3::splat(-f64::MAX),
        ),
    ] {
        assert!(BodyFrame::new(body, q, origin).is_none());
    }
}

#[test]
fn accepted_quaternion_roundoff_has_bounded_planet_scale_error() {
    let q = DQuat::from_array(
        DQuat::from_rotation_y(0.923)
            .to_array()
            .map(|v| v * (1.0 + 2e-13)),
    );
    let frame = BodyFrame::new(DVec3::ZERO, q, DVec3::ZERO).unwrap();
    let point = DVec3::new(10.0, 10.0, 3_200_000.0);
    assert!(
        (frame.render_to_body_point(frame.body_to_render_point(point)) - point).length() < 1e-5
    );
}
