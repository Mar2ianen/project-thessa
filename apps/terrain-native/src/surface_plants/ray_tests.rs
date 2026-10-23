// SPDX-License-Identifier: GPL-3.0-or-later
//! Actual plant geometry, independent f64 visibility; not a preference test.
use super::*;
use glam::DMat4;
use std::collections::BTreeSet;
use thessa_graphics::{RayTracingRequest, RequestedGraphics};
use thessa_rcbt_core::lighting::SurfaceLighting;
use thessa_render_wgpu::{
    Backend,
    mesh::{MeshRenderer, SurfaceSettings},
    terrain::OffscreenTarget,
    view::ViewParams,
};
use thessa_worldgen_rocky::scatter::{ScatterId, ScatterKind};

struct Triangle {
    a: DVec3,
    ab: DVec3,
    ac: DVec3,
    normals: [DVec3; 3],
}

impl Triangle {
    fn hit(&self, origin: DVec3, direction: DVec3, min_t: f64) -> Option<(f64, DVec3)> {
        let p = direction.cross(self.ac);
        let det = self.ab.dot(p);
        if det == 0.0 {
            return None;
        }
        let delta = origin - self.a;
        let u = delta.dot(p) / det;
        let q = delta.cross(self.ab);
        let v = direction.dot(q) / det;
        let t = self.ac.dot(q) / det;
        (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t > min_t)
            .then_some((t, DVec3::new(1.0 - u - v, u, v)))
    }
}

fn closest(
    triangles: &[Triangle],
    origin: DVec3,
    direction: DVec3,
    min_t: f64,
) -> Option<(usize, f64, DVec3)> {
    triangles
        .iter()
        .enumerate()
        .filter_map(|(i, tri)| {
            tri.hit(origin, direction, min_t)
                .map(|(t, bary)| (i, t, bary))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

fn srgb(linear: f64) -> u8 {
    let v = if linear <= 0.0031308 {
        12.92 * linear
    } else {
        1.055 * linear.powf(1.0 / 2.4) - 0.055
    };
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

#[test]
fn visibility_reference_known_triangle_cases() {
    let triangle = Triangle {
        a: DVec3::new(0.0, 0.0, -2.0),
        ab: DVec3::X,
        ac: DVec3::Y,
        normals: [DVec3::Z; 3],
    };
    let hit = triangle
        .hit(DVec3::new(0.25, 0.25, 0.0), -DVec3::Z, 0.0)
        .unwrap();
    assert_eq!(hit, (2.0, DVec3::new(0.5, 0.25, 0.25)));
    assert!(
        triangle
            .hit(DVec3::new(1.0, 1.0, 0.0), -DVec3::Z, 0.0)
            .is_none()
    );
    assert!(triangle.hit(DVec3::ZERO, DVec3::X, 0.0).is_none());
    assert!(triangle.hit(DVec3::ZERO, DVec3::Z, 0.0).is_none());
    assert!(triangle.hit(DVec3::ZERO, -DVec3::Z, 2.0).is_none());
}

#[test]
#[ignore = "hardware: actual fern/reed/lycopsid RT shadow visibility versus independent f64 triangle rays"]
fn plant_ray_visibility_matches_actual_triangle_geometry() {
    const SIZE: u32 = 257;
    let mut request = RequestedGraphics::default();
    request.renderer.ray_tracing = RayTracingRequest::Local;
    request.renderer.specular_reflections = false;
    request.shadows.enabled = true;
    let backend = Backend::bring_up_for_graphics(&request).unwrap();
    let mut checked = 0;
    let mut occluded = 0;
    let mut bias_disagreements = [0; 2];
    for (kind, height, distance) in [
        (ScatterKind::FernClump, 0.6, 2.5),
        (ScatterKind::ReedClump, 0.75, 3.0),
        (ScatterKind::LycopsidTree, 4.5, 12.0),
    ] {
        let mut prepared = convert(
            vec![ScatterItem {
                id: ScatterId(1),
                kind,
                direction: DVec3::Y.to_array(),
                ground_height_m: 0.0,
                scale: 1.0,
                yaw_rad: 0.0,
            }],
            3_200_000.0,
        )
        .unwrap();
        // Isolate visibility from organ reflectance/maps and environment transport.
        prepared.asset.materials[0] = Material {
            base_color: [0.5, 0.5, 0.5, 1.0],
            metallic: 0.0,
            roughness: 1.0,
            double_sided: true,
            ..Default::default()
        };
        let instance = MeshInstance {
            position_m: DVec3::new(0.0, -height, -distance),
            rotation: DQuat::IDENTITY,
            scale: 1.0,
        };
        let view = ViewParams {
            world_from_view: DMat4::IDENTITY,
            world_position_m: DVec3::ZERO,
            clip_from_view: glam::dcamera::rh::proj::directx::perspective(0.9, 1.0, 0.1, 100.0),
            exposure: 1.0,
        };
        let primitive = &prepared.asset.primitives[0];
        let triangles: Vec<_> = primitive
            .indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|indices| {
                let vertices = indices.map(|i| primitive.vertices[i as usize]);
                let [a, b, c] = vertices
                    .map(|v| DVec3::from_array(v.position.map(f64::from)) + instance.position_m);
                Triangle {
                    a,
                    ab: b - a,
                    ac: c - a,
                    normals: vertices.map(|v| DVec3::from_array(v.normal.map(f64::from))),
                }
            })
            .collect();
        let focal = f64::from(SIZE) / (2.0 * (0.9f64 / 2.0).tan());
        // Project actual triangle centres to select foliage as well as thick stems.
        let pixels: BTreeSet<_> = triangles
            .iter()
            .step_by(7)
            .filter_map(|tri| {
                let p = tri.a + (tri.ab + tri.ac) / 3.0;
                let x = (f64::from(SIZE) / 2.0 + focal * p.x / -p.z).floor() as i32;
                let y = (f64::from(SIZE) / 2.0 - focal * p.y / -p.z).floor() as i32;
                (x >= 0 && y >= 0 && x < SIZE as i32 && y < SIZE as i32)
                    .then_some((x as u32, y as u32))
            })
            .collect();
        for (bias_index, bias_m) in [0.01, 0.0001].into_iter().enumerate() {
            request.raytracing.ray_bias_m = bias_m;
            let settings = backend.resolve_graphics(&request);
            let mut mesh = MeshRenderer::with_surface(
                &backend,
                &prepared.asset,
                SurfaceSettings {
                    format: OffscreenTarget::FORMAT,
                    capacity: 1,
                    quality: settings.surface_quality,
                    reflections: false,
                    shadows: None,
                    ray_tracing: Some(&settings),
                },
            )
            .unwrap();
            for sun in [
                DVec3::new(0.6, 0.7, 0.4).normalize(),
                DVec3::new(-0.7, 0.2, 0.5).normalize(),
            ] {
                let lights = SurfaceLighting {
                    ambient_lux: [0.1, 0.1, 0.1, 0.0],
                    directions: [
                        [sun.x as f32, sun.y as f32, sun.z as f32, 0.0],
                        [0.0; 4],
                        [0.0; 4],
                    ],
                    colors_lux: [[std::f32::consts::PI; 4], [0.0; 4], [0.0; 4]],
                };
                mesh.update_selected(
                    &backend,
                    &view,
                    &lights,
                    &[instance],
                    Some(&prepared.ranges),
                )
                .unwrap();
                let target = OffscreenTarget::new(&backend, SIZE, SIZE).unwrap();
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                {
                    let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &target.color,
                            depth_slice: None,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &target.depth,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.0),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        ..Default::default()
                    });
                }
                mesh.encode(&mut encoder, &target.color, &target.depth);
                backend.queue.submit([encoder.finish()]);
                let rgba = target.read_rgba8(&backend).unwrap();
                let mut errors = Vec::new();
                let mut samples = 0;
                let mut physical_disagreement = 0;
                let mut leaked = 0;
                let mut extra_blocked = 0;
                let mut shadow_edge_samples = 0;
                for &(x, y) in &pixels {
                    let direction = DVec3::new(
                        (f64::from(x) + 0.5 - f64::from(SIZE) / 2.0) / focal,
                        -(f64::from(y) + 0.5 - f64::from(SIZE) / 2.0) / focal,
                        -1.0,
                    )
                    .normalize();
                    let Some((index, t, bary)) = closest(&triangles, DVec3::ZERO, direction, 0.0)
                    else {
                        continue;
                    };
                    if bary.min_element() < 0.03 {
                        continue;
                    }
                    let tri = &triangles[index];
                    let mut normal = (tri.normals[0] * bary.x
                        + tri.normals[1] * bary.y
                        + tri.normals[2] * bary.z)
                        .normalize();
                    let mut geometry = tri.ab.cross(tri.ac).normalize();
                    if geometry.dot(direction) > 0.0 {
                        normal = -normal;
                        geometry = -geometry;
                    }
                    let position = direction * t;
                    let bias = f64::from(settings.rt_ray_bias_m);
                    let cosine = geometry.dot(sun);
                    let hit = (cosine > 0.0)
                        .then(|| closest(&triangles, position, sun, bias / cosine))
                        .flatten();
                    let visible = cosine <= 0.0 || hit.is_none();
                    let physical_visible =
                        closest(&triangles, position + geometry * 1e-7, sun, 1e-7).is_none();
                    assert_eq!(
                        physical_visible,
                        closest(&triangles, position + geometry * 1e-8, sun, 1e-8).is_none(),
                        "physical-reference offset convergence"
                    );
                    physical_disagreement +=
                        usize::from(visible != physical_visible && normal.dot(sun) > 0.0);
                    leaked += usize::from(visible && !physical_visible && normal.dot(sun) > 0.0);
                    extra_blocked +=
                        usize::from(!visible && physical_visible && normal.dot(sun) > 0.0);
                    let ndotl = normal.dot(sun).max(0.0);
                    let fresnel = 0.04 + 0.96 * (1.0 - normal.dot(-direction).max(0.0)).powi(5);
                    let expected = srgb(
                        (1.0 - fresnel)
                            * 0.5
                            * (0.1 / std::f64::consts::PI + if visible { ndotl } else { 0.0 }),
                    );
                    let offset = ((y * SIZE + x) * 4) as usize;
                    let actual = &rgba[offset..offset + 3];
                    // A float raster interpolation can cross a narrow blocker's
                    // edge; pin interior visibility separately from edge coverage.
                    let shadow_edge = hit.is_some_and(|(_, _, bary)| bary.min_element() < 0.03);
                    shadow_edge_samples += usize::from(shadow_edge);
                    if !shadow_edge && actual.iter().any(|c| c.abs_diff(expected) > 2) {
                        errors.push((x, y, index, actual.to_vec(), expected));
                    }
                    samples += 1;
                    occluded += usize::from(!visible && ndotl > 0.1);
                }
                eprintln!(
                    "plant_visibility {kind:?} bias_m={bias_m} sun={sun:?} samples={samples} shadow_edges={shadow_edge_samples} biased_vs_physical={physical_disagreement} leaked={leaked} extra_blocked={extra_blocked} gpu_errors={} first={:?}",
                    errors.len(),
                    &errors[..errors.len().min(8)]
                );
                assert_eq!(
                    extra_blocked, 0,
                    "receiver exclusion must not invent a blocker on a clear physical ray"
                );
                assert!(
                    samples >= 15,
                    "actual prototype fixture must exercise visible geometry"
                );
                assert!(
                    errors.is_empty(),
                    "GPU must agree with independent biased-geometry oracle"
                );
                checked += samples;
                bias_disagreements[bias_index] += physical_disagreement;
            }
            assert!(mesh.rt_stats().active);
        }
    }
    assert!(
        checked >= 200 && occluded >= 10,
        "must exercise both lit and blocked surfaces: {checked}/{occluded}"
    );
    assert_eq!(
        bias_disagreements,
        [5, 1],
        "finite receiver exclusion still skips genuine nearby blockers; not complete visibility equality"
    );
}
