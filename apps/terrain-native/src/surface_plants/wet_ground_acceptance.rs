//! Reviewer-owned geometry acceptance cases for native ground cover.
use super::*;

#[test]
fn curved_blades_reach_a_point_without_degenerate_tip_triangles() {
    let base = DVec3::ZERO;
    let length = 0.20;
    let lean = DVec3::X * 0.07;
    let tip = base + DVec3::Y * length + lean;
    for segments in [1, 2, 3, 4] {
        let mut mesh = blank(0);
        curved_blade(
            &mut mesh,
            base,
            BladePose {
                up: DVec3::Y,
                lean,
                side: DVec3::Z,
            },
            BladeShape {
                length,
                half_width: 0.008,
                segments,
                taper_start: None,
            },
            DVec3::ZERO,
        );
        assert!(
            mesh.vertices
                .iter()
                .any(|v| { DVec3::from(v.position.map(f64::from)).distance(tip) < 1e-7 }),
            "a blade must end at its centerline tip, without a blunt cross-section"
        );
        assert!(
            mesh.vertices.iter().any(|v| v.uv[1] >= 0.999),
            "blade UV.y records longitudinal position from root=0 to tip=1"
        );
        assert!(
            mesh.vertices
                .iter()
                .filter(|v| v.uv[1].abs() <= 1e-6)
                .all(|v| v.position[1].abs() <= 1e-7),
            "UV.y=0 must identify the actual root rather than a raised blade ring"
        );
        for tri in mesh.indices.as_chunks::<3>().0 {
            let p = tri.map(|i| DVec3::from(mesh.vertices[i as usize].position.map(f64::from)));
            assert!(
                (p[1] - p[0]).cross(p[2] - p[0]).length() > 1e-10,
                "collapsing a tip must not publish a zero-area triangle"
            );
        }
        assert!(mesh.vertices.iter().all(|v| {
            let n = DVec3::from(v.normal.map(f64::from));
            n.is_finite() && (n.length() - 1.0).abs() < 1e-5
        }));
    }
}

#[test]
fn near_camera_grass_roots_follow_the_same_soil_surface() {
    let eye = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818);
    let dir = eye.normalize();
    let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
    let mut request = thessa_graphics::RequestedGraphics::default();
    request.renderer.terrain_ground_detail = true;
    request.renderer.surface_quality = thessa_graphics::Quality::High;
    let settings = ResolvedGraphicsSettings::from_requested(
        &request,
        &thessa_graphics::Capabilities::unknown(),
    );
    let (prepared, _) = prepare(
        Context::SurveyWithUnderstory {
            canopy: survey_window(dir),
            understory: understory_window(dir),
        },
        &settings,
        &field,
    )
    .unwrap();
    let prepared = prepared.unwrap();
    let mut roots = 0;
    let mut worst_float = f64::NEG_INFINITY;
    let mut worst_burial = f64::INFINITY;
    for (mesh, range) in prepared.asset.primitives.iter().zip(&prepared.ranges) {
        let material = &prepared.asset.materials[mesh.material];
        if !GRASS_PALETTES.contains(&material.base_color) {
            continue;
        }
        for instance in &prepared.instances[range.start as usize..range.end as usize] {
            if instance.position_m.distance(eye) > 10.0 {
                continue;
            }
            for vertex in mesh.vertices.iter().filter(|v| v.uv[1].abs() <= 1e-6) {
                let p = instance.position_m
                    + instance.rotation
                        * (DVec3::from(vertex.position.map(f64::from)) * instance.scale);
                let clearance = p.length() - ground_position(&field, p).length();
                worst_float = worst_float.max(clearance);
                worst_burial = worst_burial.min(clearance);
                roots += 1;
            }
        }
    }
    assert!(
        roots >= 100,
        "the contact fixture must inspect near-camera roots"
    );
    assert!(
        worst_float <= 0.006 && worst_burial >= -0.020,
        "root clearance must be within -20..6 mm, sampled={roots}, min={worst_burial}, max={worst_float}"
    );
}
