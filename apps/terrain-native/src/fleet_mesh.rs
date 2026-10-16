//! Game-side source/epoch selection for the existing single opaque mesh pass.
use super::*;
use std::{collections::BTreeMap, ops::Range};
use thessa_flight_authority::presentation::{FlightPose, inertial_to_render_axes};
use thessa_flight_net::geometry::{FleetGeometry, RenderAppearance};
use thessa_render_core::mesh::{Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_render_wgpu::mesh::{MeshDrawStats, MeshRenderer};
use thessa_sim_core::{BakedEphemeris, BodyId, SimTime, VehicleId};

#[derive(Clone, Copy)]
struct Track {
    previous: Option<FlightPose>,
    latest: FlightPose,
}

impl Track {
    fn push(&mut self, pose: FlightPose) {
        if pose.time().0 > self.latest.time().0 {
            self.previous = Some(self.latest);
        } else if pose.time().0 < self.latest.time().0 {
            // Reset/relaunch is not interpolation across a discontinuity.
            self.previous = None;
        }
        self.latest = pose;
    }

    fn at(self, time: SimTime, ephemeris: &BakedEphemeris, body: BodyId) -> Option<MeshInstance> {
        let latest = self.latest;
        let (position, rotation) = if time == latest.time() {
            (
                latest.relative_position_m(),
                latest.orientation_body_to_inertial(),
            )
        } else {
            let previous = self.previous?;
            if !(previous.time().0..=latest.time().0).contains(&time.0)
                || latest.time() == previous.time()
            {
                return None;
            }
            let alpha = (time.0 - previous.time().0) / (latest.time().0 - previous.time().0);
            // Interpolate inertial endpoints, then subtract the ephemeris at
            // the COMMON epoch. Never interpolate already-subtracted offsets
            // against a moving body or extrapolate a secondary's clock debt.
            let before = previous.relative_position_m()
                + ephemeris
                    .body_state(body, previous.time())
                    .ok()?
                    .position_inertial;
            let after = latest.relative_position_m()
                + ephemeris
                    .body_state(body, latest.time())
                    .ok()?
                    .position_inertial;
            let reference = ephemeris.body_state(body, time).ok()?.position_inertial;
            (
                before.lerp(after, alpha) - reference,
                previous
                    .orientation_body_to_inertial()
                    .slerp(latest.orientation_body_to_inertial(), alpha)
                    .normalize(),
            )
        };
        Some(MeshInstance {
            position_m: inertial_to_render_axes(position),
            rotation: DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2) * rotation,
            scale: 1.0,
        })
    }
}

/// Bounds are game wire/graphics policy, not a second GPU backend or pass.
pub struct SceneMeshes {
    inner: MeshRenderer,
    format: wgpu::TextureFormat,
    enabled: bool,
    min_pixels: f32,
    revision: u64,
    observed_updates: u64,
    tracks: BTreeMap<VehicleId, Track>,
    primary_primitives: usize,
    radii: BTreeMap<VehicleId, (f64, Range<usize>)>,
    instances: Vec<MeshInstance>,
    ranges: Vec<Range<u32>>,
    pub source_publications: u64,
    pub epoch_withheld: usize,
    pub size_culled: usize,
}

/// Flatten part transforms once per catalogue, never once per frame. Meshes
/// remain actual retained sources, with an explicitly neutral untextured finish.
fn append_catalogue(
    source: &mut MeshAsset,
    catalogue: &FleetGeometry,
    radii: &mut BTreeMap<VehicleId, (f64, Range<usize>)>,
) -> Result<(), BackendError> {
    let material = source.materials.len();
    source.materials.push(Material {
        base_color: [0.65, 0.65, 0.65, 1.0],
        metallic: 0.0,
        ..Default::default()
    });
    for vehicle in &catalogue.vehicles {
        let RenderAppearance::Available(geometry) = &vehicle.render else {
            continue;
        };
        let mut radius: f64 = 0.0;
        let start = source.primitives.len();
        for part in &geometry.parts {
            let mut vertices = Vec::with_capacity(part.mesh.positions.len());
            for (position, normal) in part.mesh.positions.iter().zip(&part.mesh.normals) {
                let position = part.position_body_m + part.orientation_mesh_to_body * *position;
                let narrow = position.as_vec3();
                if !narrow.is_finite() || (narrow.as_dvec3() - position).length() > 0.001 {
                    return Err(BackendError::Device(
                        "fleet vertex exceeds 1 mm narrowing envelope".into(),
                    ));
                }
                radius = radius.max(position.length());
                vertices.push(Vertex {
                    position: narrow.to_array(),
                    normal: (part.orientation_mesh_to_body * *normal)
                        .as_vec3()
                        .to_array(),
                    uv: [0.0; 2],
                });
            }
            source.primitives.push(Primitive {
                vertices,
                indices: part.mesh.indices.clone(),
                material,
            });
        }
        radii.insert(vehicle.vehicle_id, (radius, start..source.primitives.len()));
    }
    Ok(())
}

impl SceneMeshes {
    pub fn new(
        scene: &Scene,
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
    ) -> Result<Self, BackendError> {
        let resolved = thessa_graphics::ResolvedGraphicsSettings::from_requested(
            &scene.requested,
            &backend.caps,
        );
        Ok(Self {
            inner: MeshRenderer::new(backend, source, format, 1)?,
            format,
            enabled: resolved.fleet_meshes,
            min_pixels: resolved.fleet_min_pixels,
            revision: 0,
            observed_updates: 0,
            tracks: BTreeMap::new(),
            primary_primitives: source.primitives.len(),
            radii: BTreeMap::new(),
            instances: Vec::with_capacity(64),
            ranges: vec![0..1; source.primitives.len()],
            source_publications: 0,
            epoch_withheld: 0,
            size_culled: 0,
        })
    }

    pub fn draw_stats(&self) -> MeshDrawStats {
        self.inner.draw_stats()
    }
    pub fn encode(
        &self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        self.inner.encode(encoder, color, depth);
    }

    pub fn update(
        &mut self,
        scene: &Scene,
        backend: &Backend,
        width: u32,
        height: u32,
        exposure: f32,
    ) -> Result<(), BackendError> {
        let view = scene.view(width, height, exposure);
        let Some((primary, primary_instance)) = &scene.vehicle else {
            return self
                .inner
                .update(backend, &view, &scene.sample.lighting, &[]);
        };
        if !self.enabled {
            return self.inner.update(
                backend,
                &view,
                &scene.sample.lighting,
                std::slice::from_ref(primary_instance),
            );
        }
        let observer = scene.remote.as_ref().map(|r| &r.fleet);
        let matched = observer.and_then(FleetObserver::matched_geometry);
        if let Some(catalogue) = matched
            && catalogue.revision != self.revision
        {
            let mut radii = BTreeMap::new();
            // Empty catalogues don't re-upload PRIMARY; removal does retire the
            // old fleet source. All resources use the same existing device.
            if catalogue
                .vehicles
                .iter()
                .any(|v| matches!(v.render, RenderAppearance::Available(_)))
                || !self.radii.is_empty()
            {
                let mut source = primary.clone();
                append_catalogue(&mut source, catalogue, &mut radii)?;
                let inner = MeshRenderer::new(backend, &source, self.format, 64)?;
                self.inner = inner;
                self.source_publications += 1;
            }
            self.revision = catalogue.revision;
            self.tracks.clear();
            self.observed_updates = 0;
            self.radii = radii;
            let primitive_count =
                self.primary_primitives + self.radii.values().map(|(_, r)| r.len()).sum::<usize>();
            self.ranges.resize(primitive_count, 0..0);
        }
        if let Some(observer) = observer
            && !self.radii.is_empty()
            && observer.updates != self.observed_updates
        {
            self.tracks
                .retain(|id, _| observer.rows.iter().any(|r| r.vehicle_id == *id));
            for row in &observer.rows {
                self.tracks
                    .entry(row.vehicle_id)
                    .and_modify(|track| track.push(row.pose))
                    .or_insert(Track {
                        previous: None,
                        latest: row.pose,
                    });
            }
            self.observed_updates = observer.updates;
        }
        self.instances.clear();
        self.instances.push(*primary_instance);
        self.ranges.fill(0..0);
        self.ranges[..self.primary_primitives].fill(0..1);
        self.epoch_withheld = 0;
        self.size_culled = 0;
        if observer.is_some_and(|o| !o.stale) && matched.is_some() {
            let body = scene.flight.as_ref().expect("server flight").reference_body;
            for (id, (radius, primitives)) in &self.radii {
                let Some(instance) = self
                    .tracks
                    .get(id)
                    .and_then(|t| t.at(scene.sample.time, scene.celestial.ephemeris(), body))
                else {
                    self.epoch_withheld += 1;
                    continue;
                };
                // Angular-size quality fallback. Near/intersecting bounds are
                // never culled. This estimates footprint, not exact visibility.
                let distance = (instance.position_m - view.world_position_m).length();
                let pixels = radius * f64::from(height) * view.clip_from_view.y_axis.y.abs()
                    / (distance - radius).max(1e-6);
                if pixels < f64::from(self.min_pixels) {
                    self.size_culled += 1;
                    continue;
                }
                let index = self.instances.len() as u32;
                self.instances.push(instance);
                self.ranges[primitives.clone()].fill(index..index + 1);
            }
        }
        self.inner.update_selected(
            backend,
            &view,
            &scene.sample.lighting,
            &self.instances,
            Some(&self.ranges),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use thessa_sim_core::{RenderGeometry, RenderMesh, RenderPart};

    fn triangle() -> Arc<RenderMesh> {
        Arc::new(RenderMesh {
            positions: vec![
                DVec3::new(-0.8, -0.8, 0.0),
                DVec3::new(0.8, -0.8, 0.0),
                DVec3::new(0.0, 0.8, 0.0),
            ],
            normals: vec![DVec3::Z; 3],
            indices: vec![0, 1, 2],
        })
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn heterogeneous_world_meshes_share_one_pass_and_withhold_unpaired_epochs() {
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        // Minimal primary source outside the camera, so only known fleet
        // shapes contribute. No production X-15/terrain/material comparison.
        scene.vehicle.as_mut().unwrap().0 = MeshAsset {
            primitives: vec![Primitive {
                vertices: triangle()
                    .positions
                    .iter()
                    .map(|p| Vertex {
                        position: (*p + DVec3::splat(1e4)).as_vec3().to_array(),
                        normal: [0.0, 0.0, 1.0],
                        uv: [0.0; 2],
                    })
                    .collect(),
                indices: vec![0, 1, 2],
                material: 0,
            }],
            materials: vec![Material::default()],
            textures: vec![],
            images: vec![],
        };
        let view = scene.view(320, 240, 1.0);
        let camera_rotation = DQuat::from_mat3(&glam::DMat3::from_mat4(view.world_from_view));
        let craft_rotation = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2) * camera_rotation;
        let rows: Vec<_> = [(-2.0, 1), (2.0, 2)]
            .into_iter()
            .map(|(x, id)| {
                let render_position =
                    view.world_position_m + camera_rotation * DVec3::new(x, 0.0, -10.0);
                let relative = DVec3::new(render_position.x, -render_position.z, render_position.y);
                thessa_flight_net::presentation::FleetPose {
                    vehicle_id: VehicleId::new(id),
                    tick: 0,
                    stopped: false,
                    pose: FlightPose::new(SimTime(0.0), relative, craft_rotation).unwrap(),
                }
            })
            .collect();
        let mut vehicle = scene.flight.as_ref().unwrap().vehicle.clone();
        vehicle.render_geometry = RenderGeometry {
            parts: vec![RenderPart {
                name: "triangle".into(),
                assembly_body: None,
                position_body_m: DVec3::ZERO,
                orientation_mesh_to_body: DQuat::IDENTITY,
                mesh: triangle(),
            }],
        };
        let a = thessa_flight_net::geometry::FleetVehicleGeometry::from_vehicle(
            VehicleId::new(1),
            &vehicle,
        );
        Arc::make_mut(&mut vehicle.render_geometry.parts[0].mesh).positions[2].y = 0.1;
        let b = thessa_flight_net::geometry::FleetVehicleGeometry::from_vehicle(
            VehicleId::new(2),
            &vehicle,
        );
        let catalogue = Arc::new(FleetGeometry {
            revision: 1,
            vehicles: vec![a, b],
        });
        catalogue.validate().unwrap();
        scene.remote = Some(RemoteFlight {
            link: None,
            commands: vec![],
            started: std::time::Instant::now(),
            timeout: std::time::Duration::from_secs(5),
            snapshots: 1,
            last_tick: Some(0),
            fleet: FleetObserver {
                rows,
                updates: 1,
                geometry_revision: 1,
                geometry_updates: 1,
                geometry: Some(catalogue),
                ..Default::default()
            },
        });
        let mut meshes = scene
            .mesh_renderer(&backend, OffscreenTarget::FORMAT)
            .unwrap()
            .unwrap();
        let clear = |encoder: &mut wgpu::CommandEncoder, target: &OffscreenTarget| {
            let _clear = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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
        };
        let render = |scene: &Scene, meshes: &mut SceneMeshes, width, height| {
            let target = OffscreenTarget::new(&backend, width, height).unwrap();
            scene
                .update_mesh(&backend, meshes, width, height, 1.0)
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, &target);
            meshes.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let image = render(&scene, &mut meshes, 320, 240);
        assert_eq!(meshes.draw_stats().instances, 3);
        assert_eq!(meshes.draw_stats().draw_calls, 3);
        assert_eq!(meshes.source_publications, 1);
        assert_eq!(meshes.epoch_withheld, 0);
        let colored = |bytes: &[u8], left: bool| {
            bytes
                .as_chunks::<4>()
                .0
                .iter()
                .enumerate()
                .filter(|(i, pixel)| (*i % 320 < 160) == left && pixel[..3].iter().any(|c| *c != 0))
                .count()
        };
        let left = colored(&image, true);
        let right = colored(&image, false);
        assert!(
            left > 100 && right > 50 && left > right,
            "different shapes must render on their own sides: {left}/{right}"
        );
        assert_eq!(image, render(&scene, &mut meshes, 320, 240));
        let before = meshes.draw_stats();
        assert!(
            meshes
                .inner
                .update_selected(
                    &backend,
                    &view,
                    &scene.sample.lighting,
                    &meshes.instances,
                    Some(&[])
                )
                .is_err()
        );
        assert_eq!(
            meshes.draw_stats(),
            before,
            "invalid range publication must be atomic"
        );
        assert_eq!(
            meshes.source_publications, 1,
            "no static upload at pose cadence"
        );
        let _resized = render(&scene, &mut meshes, 257, 193);
        scene.remote.as_mut().unwrap().fleet.stale = true;
        let empty = render(&scene, &mut meshes, 320, 240);
        assert!(empty.as_chunks::<4>().0.iter().all(|p| p[..3] == [0, 0, 0]));
        scene.remote.as_mut().unwrap().fleet.stale = false;
        scene.sample.time = SimTime(0.01);
        let empty = render(&scene, &mut meshes, 320, 240);
        assert_eq!(meshes.epoch_withheld, 2);
        assert!(empty.as_chunks::<4>().0.iter().all(|p| p[..3] == [0, 0, 0]));
        scene.sample.time = SimTime(0.0);
        scene.requested.renderer.fleet_meshes = false;
        let mut disabled = scene
            .mesh_renderer(&backend, OffscreenTarget::FORMAT)
            .unwrap()
            .unwrap();
        assert_eq!(empty, render(&scene, &mut disabled, 320, 240));
        assert_eq!(disabled.source_publications, 0);
        assert!(disabled.radii.is_empty() && disabled.tracks.is_empty());
        // Target-size update/submit diagnostic: repeated synthetic poses, NOT
        // a 63-vehicle physical replay or a visibility-quality benchmark.
        let fleet = &mut scene.remote.as_mut().unwrap().fleet;
        let template = fleet.rows[0];
        let geometry = fleet.geometry.as_ref().unwrap().vehicles[0].clone();
        fleet.rows = (1..64)
            .map(|id| thessa_flight_net::presentation::FleetPose {
                vehicle_id: VehicleId::new(id),
                ..template
            })
            .collect();
        fleet.geometry = Some(Arc::new(FleetGeometry {
            revision: 2,
            vehicles: (1..64)
                .map(|id| {
                    let mut g = geometry.clone();
                    g.vehicle_id = VehicleId::new(id);
                    g
                })
                .collect(),
        }));
        fleet.geometry_revision = 2;
        fleet.updates += 1;
        let target = OffscreenTarget::new(&backend, 320, 240).unwrap();
        let mut times = Vec::new();
        for sample in 0..110 {
            let start = std::time::Instant::now();
            scene
                .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
                .unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            clear(&mut encoder, &target);
            meshes.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            backend
                .device
                .poll(wgpu::PollType::wait_indefinitely())
                .unwrap();
            if sample >= 10 {
                times.push(start.elapsed().as_secs_f64() * 1e6);
            }
        }
        assert_eq!(meshes.draw_stats().instances, 64);
        assert_eq!(meshes.draw_stats().draw_calls, 64);
        assert_eq!(meshes.source_publications, 2);
        times.sort_by(f64::total_cmp);
        eprintln!(
            "fleet_target_workload secondaries=63 primitives=64 synthetic_overlapping_triangles resolution=320x240 samples=100 warmup=10 update_submit_gpu_completion_median_us={:.3}; no readback, source upload excluded, not physical fleet or production FPS",
            times[50]
        );
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target/test-tmp/fleet-world-oracle.png");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let file = std::fs::File::create(&path).unwrap();
        let mut encoder = png::Encoder::new(file, 320, 240);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        encoder
            .write_header()
            .unwrap()
            .write_image_data(&image)
            .unwrap();
        eprintln!(
            "Heterogeneous mesh GPU oracle (synthetic poses, not physical fleet replay): {}",
            path.display()
        );
    }
    #[test]
    fn epoch_sampling_brackets_inertial_motion_without_extrapolation_or_glb_calibration() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let a = SimTime(1234.0);
        let b = SimTime(1234.1);
        let mid = SimTime(1234.05);
        let origin = ephemeris.body_state(body, a).unwrap().position_inertial
            + DVec3::new(3.2e6, 100.0, 200.0);
        let end = origin + DVec3::new(5.0, -2.0, 1.0);
        let pose = |time, point, rotation| {
            FlightPose::new(
                time,
                point - ephemeris.body_state(body, time).unwrap().position_inertial,
                rotation,
            )
            .unwrap()
        };
        let mut track = Track {
            previous: None,
            latest: pose(a, origin, DQuat::IDENTITY),
        };
        assert!(track.at(mid, &ephemeris, body).is_none());
        track.push(pose(b, end, DQuat::from_rotation_z(0.2)));
        let sample = track.at(mid, &ephemeris, body).unwrap();
        let expected = inertial_to_render_axes(
            origin.lerp(end, 0.5) - ephemeris.body_state(body, mid).unwrap().position_inertial,
        );
        assert!((sample.position_m - expected).length() < 0.001);
        let expected_forward = inertial_to_render_axes(DQuat::from_rotation_z(0.1) * DVec3::X);
        assert!((sample.rotation * DVec3::X - expected_forward).length() < 1e-10);
        assert!(track.at(SimTime(b.0 + 0.01), &ephemeris, body).is_none());
        assert!(track.at(SimTime(a.0 - 0.01), &ephemeris, body).is_none());
        track.push(pose(a, origin, DQuat::IDENTITY));
        assert!(track.at(mid, &ephemeris, body).is_none());
    }
}
