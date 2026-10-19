//! Game-side source/epoch selection for the existing single opaque mesh pass.
use super::*;
use std::sync::Arc;
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
    terrain: Option<thessa_render_wgpu::terrain_ray::TerrainRaySource>,
    terrain_reason: Option<String>,
    shadows: Option<thessa_render_wgpu::shadows::ShadowAtlas>,
    /// Resolved against the *negotiated* device, so an RT feature that was
    /// not requested at device creation cannot appear as available here.
    resolved: thessa_graphics::ResolvedGraphicsSettings,
    inner: Option<MeshRenderer>,
    environment_revision: Option<u64>,
    format: wgpu::TextureFormat,
    enabled: bool,
    min_pixels: f32,
    revision: u64,
    // Retain the publication, not the latest packet, so the layout tolerance
    // cannot accumulate drift across revisions. Payload storage is Arc-shared.
    source_geometry: Option<Arc<FleetGeometry>>,
    observed_updates: u64,
    tracks: BTreeMap<VehicleId, Track>,
    primary_primitives: usize,
    radii: BTreeMap<VehicleId, (f64, Range<usize>, DVec3)>,
    instances: Vec<MeshInstance>,
    ranges: Vec<Range<u32>>,
    pub source_publications: u64,
    pub epoch_withheld: usize,
    pub size_culled: usize,
}

// Compare against the retained GPU source, not each preceding packet: small
// errors never accumulate across updates. COM recentering changes all origins
// equally; the 1 nm subtraction envelope is far below the 1 mm mesh budget.
const RENDER_ORIGIN_ERROR_M: f64 = 1e-9;

fn in_body_frame(mut instance: MeshInstance, frame: thessa_render_core::BodyFrame) -> MeshInstance {
    instance.position_m = frame.render_to_body_point(instance.position_m);
    instance.rotation = (frame.rotation().conjugate() * instance.rotation).normalize();
    instance
}

fn same_render_layout(before: &FleetGeometry, after: &FleetGeometry) -> bool {
    same_primary_layout(before, after)
        && before.vehicles.len() == after.vehicles.len()
        && before
            .vehicles
            .iter()
            .zip(&after.vehicles)
            .all(|(a, b)| same_vehicle_layout(a, b))
}

fn same_primary_layout(before: &FleetGeometry, after: &FleetGeometry) -> bool {
    match (&before.primary, &after.primary) {
        (None, None) => true,
        (Some(a), Some(b)) => same_vehicle_layout(a, b),
        _ => false,
    }
}

fn same_vehicle_layout(
    a: &thessa_flight_net::geometry::FleetVehicleGeometry,
    b: &thessa_flight_net::geometry::FleetVehicleGeometry,
) -> bool {
    a.vehicle_id == b.vehicle_id
        && match (&a.render, &b.render) {
            (RenderAppearance::Available(a), RenderAppearance::Available(b)) => {
                let anchor_a = a.parts[0].position_body_m;
                let anchor_b = b.parts[0].position_body_m;
                a.parts.len() == b.parts.len()
                    && a.parts.iter().zip(&b.parts).all(|(a, b)| {
                        ((a.position_body_m - anchor_a) - (b.position_body_m - anchor_b)).length()
                            <= RENDER_ORIGIN_ERROR_M
                            && a.orientation_mesh_to_body == b.orientation_mesh_to_body
                            && (Arc::ptr_eq(&a.mesh, &b.mesh) || a.mesh == b.mesh)
                    })
            }
            (RenderAppearance::Available(_), _) | (_, RenderAppearance::Available(_)) => false,
            (RenderAppearance::StarterX15 { .. }, RenderAppearance::StarterX15 { .. }) => true,
            _ => a.render == b.render,
        }
}

fn render_root(
    pose: FlightPose,
    geometry: &thessa_sim_core::RenderGeometry,
    published_anchor: DVec3,
) -> Option<FlightPose> {
    let rotation = pose.orientation_body_to_inertial();
    FlightPose::new(
        pose.time(),
        pose.relative_position_m()
            + rotation * (geometry.parts.first()?.position_body_m - published_anchor),
        rotation,
    )
}

/// Flatten transforms in the publication's body frame. The first part's origin
/// identifies later common translations, but isn't the interpolation origin:
/// tracking an offset part would change the circular arc of a rotating body.
/// Meshes remain actual retained sources, with a neutral untextured finish.
fn append_catalogue(
    source: &mut MeshAsset,
    catalogue: &FleetGeometry,
    radii: &mut BTreeMap<VehicleId, (f64, Range<usize>, DVec3)>,
    include_fleet: bool,
) -> Result<(), BackendError> {
    let material = source.materials.len();
    source.materials.push(Material {
        base_color: [0.65, 0.65, 0.65, 1.0],
        metallic: 0.0,
        ..Default::default()
    });
    for vehicle in catalogue
        .primary
        .iter()
        .chain(catalogue.vehicles.iter().filter(|_| include_fleet))
    {
        let RenderAppearance::Available(geometry) = &vehicle.render else {
            continue;
        };
        let mut radius: f64 = 0.0;
        let start = source.primitives.len();
        let anchor = geometry.parts[0].position_body_m;
        for part in &geometry.parts {
            let mut vertices = Vec::with_capacity(part.mesh.positions.len());
            for (position, normal) in part.mesh.positions.iter().zip(&part.mesh.normals) {
                let position = part.position_body_m + part.orientation_mesh_to_body * *position;
                let narrow = position.as_vec3();
                if !narrow.is_finite()
                    || (narrow.as_dvec3() - position).length() > 0.001 - RENDER_ORIGIN_ERROR_M
                {
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
        radii.insert(
            vehicle.vehicle_id,
            (
                radius + RENDER_ORIGIN_ERROR_M,
                start..source.primitives.len(),
                anchor,
            ),
        );
    }
    Ok(())
}

impl SceneMeshes {
    #[cfg(test)]
    pub fn new(
        scene: &Scene,
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
    ) -> Result<Self, BackendError> {
        Self::with_shadows(scene, backend, source, format, None, None)
    }

    pub fn with_shadows(
        scene: &Scene,
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
        shadows: Option<&thessa_render_wgpu::shadows::ShadowAtlas>,
        terrain: Option<thessa_render_wgpu::terrain_ray::TerrainRaySource>,
    ) -> Result<Self, BackendError> {
        let resolved = backend.resolve_graphics(&scene.requested);
        let (quality, enabled, min_pixels) = (
            resolved.surface_quality,
            resolved.fleet_meshes,
            resolved.fleet_min_pixels,
        );
        Ok(Self {
            shadows: shadows.cloned(),
            environment_revision: None,
            inner: if scene.server_flight() {
                None
            } else {
                Some(MeshRenderer::with_surface_and_terrain(
                    backend,
                    source,
                    thessa_render_wgpu::mesh::SurfaceSettings {
                        format,
                        capacity: 1,
                        quality,
                        reflections: resolved.specular_reflections,
                        shadows,
                        ray_tracing: Some(&resolved),
                    },
                    terrain.as_ref(),
                )?)
            },
            resolved,
            terrain,
            terrain_reason: None,
            format,
            enabled,
            min_pixels,
            revision: 0,
            source_geometry: None,
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
        self.inner.as_ref().map_or(
            MeshDrawStats {
                instances: 0,
                draw_calls: 0,
                triangles: 0,
                frame_upload_bytes: 0,
            },
            MeshRenderer::draw_stats,
        )
    }
    pub fn opaque(&self) -> Option<&MeshRenderer> {
        self.inner.as_ref()
    }
    pub fn sync_rt_terrain(
        &mut self,
        backend: &Backend,
        terrain: &thessa_render_wgpu::terrain::TerrainRenderer,
    ) -> Result<(), BackendError> {
        if let Some(inner) = &mut self.inner {
            inner.sync_rt_terrain(backend, terrain)?;
            let reason = inner.rt_terrain_reason();
            if reason != self.terrain_reason.as_deref() {
                if let Some(reason) = reason {
                    eprintln!("ray_terrain_fallback={reason}");
                } else {
                    eprintln!("ray_terrain_fallback=cleared; resident terrain readmitted");
                }
                self.terrain_reason = reason.map(str::to_owned);
            }
        }
        Ok(())
    }
    pub fn encode_rt(&mut self, encoder: &mut wgpu::CommandEncoder) {
        if let Some(inner) = &mut self.inner {
            inner.encode_rt(encoder);
        }
    }

    #[allow(dead_code)]
    pub fn encode(
        &mut self,
        encoder: &mut wgpu::CommandEncoder,
        color: &wgpu::TextureView,
        depth: &wgpu::TextureView,
    ) {
        if let Some(inner) = &mut self.inner {
            inner.encode(encoder, color, depth);
        }
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
        self.update_environment(scene, backend);
        let Some((primary, primary_instance)) = &scene.vehicle else {
            return if let Some(inner) = &mut self.inner {
                inner.update(backend, &view, &scene.lighting(), &[])
            } else {
                Ok(())
            };
        };
        if !scene.server_flight() {
            return self.inner.as_mut().expect("local source").update(
                backend,
                &view,
                &scene.lighting(),
                std::slice::from_ref(primary_instance),
            );
        }
        let observer = scene.remote.as_ref().map(|r| &r.fleet);
        let matched = observer.and_then(FleetObserver::matched_geometry);
        let catalogue = observer.and_then(|o| o.geometry.as_deref());
        if let Some(catalogue) = catalogue
            && catalogue.revision != self.revision
        {
            let changed = self.source_geometry.as_deref().is_none_or(|before| {
                if self.enabled {
                    !same_render_layout(before, catalogue)
                } else {
                    // Disabled fleet rendering never rebuilds PRIMARY for
                    // unrelated fleet revisions or retains fleet GPU ranges.
                    !same_primary_layout(before, catalogue)
                }
            });
            if changed {
                let mut radii = BTreeMap::new();
                let builtin = catalogue
                    .primary
                    .as_ref()
                    .is_some_and(|p| matches!(p.render, RenderAppearance::StarterX15 { .. }));
                let mut source = if builtin {
                    primary.clone()
                } else {
                    MeshAsset {
                        primitives: vec![],
                        materials: vec![],
                        textures: vec![],
                        images: vec![],
                    }
                };
                let primary_primitives = source.primitives.len();
                append_catalogue(&mut source, catalogue, &mut radii, self.enabled)?;
                let mut inner = if source.primitives.is_empty() {
                    None
                } else {
                    Some(MeshRenderer::with_surface_and_terrain(
                        backend,
                        &source,
                        thessa_render_wgpu::mesh::SurfaceSettings {
                            format: self.format,
                            capacity: 64,
                            quality: self.resolved.surface_quality,
                            reflections: self.resolved.specular_reflections,
                            shadows: self.shadows.as_ref(),
                            ray_tracing: Some(&self.resolved),
                        },
                        self.terrain.as_ref(),
                    )?)
                };
                if let (Some(next), Some(previous)) = (&mut inner, &mut self.inner) {
                    next.adopt_rt_terrain(previous);
                }
                if self.inner.is_some() || inner.is_some() {
                    self.source_publications += 1;
                }
                self.inner = inner;
                self.environment_revision = None;
                self.primary_primitives = primary_primitives;
                self.tracks.clear();
                self.observed_updates = 0;
                self.radii = radii;
                let primitive_count = self.primary_primitives
                    + self.radii.values().map(|(_, r, _)| r.len()).sum::<usize>();
                self.ranges.resize(primitive_count, 0..0);
                self.source_geometry = if self.enabled {
                    observer.and_then(|o| o.geometry.clone())
                } else {
                    Some(Arc::new(FleetGeometry {
                        revision: catalogue.revision,
                        primary: catalogue.primary.clone(),
                        vehicles: vec![],
                    }))
                };
            }
            self.revision = catalogue.revision;
        }
        if let Some(observer) = observer
            && self.enabled
            && !self.radii.is_empty()
            && observer.updates != self.observed_updates
            && let Some(catalogue) = matched
        {
            self.tracks
                .retain(|id, _| observer.rows.iter().any(|r| r.vehicle_id == *id));
            // Adoption already checked identical, sorted membership for the
            // matched revision. Walk it once, not an ID cross-product scan.
            for (row, geometry) in observer.rows.iter().zip(&catalogue.vehicles) {
                let RenderAppearance::Available(geometry) = &geometry.render else {
                    continue;
                };
                let Some((_, _, anchor)) = self.radii.get(&row.vehicle_id) else {
                    continue;
                };
                let Some(pose) = render_root(row.pose, geometry, *anchor) else {
                    continue;
                };
                self.tracks
                    .entry(row.vehicle_id)
                    .and_modify(|track| track.push(pose))
                    .or_insert(Track {
                        previous: None,
                        latest: pose,
                    });
            }
            self.observed_updates = observer.updates;
        }
        self.instances.clear();
        self.ranges.fill(0..0);
        self.epoch_withheld = 0;
        self.size_culled = 0;
        if let (Some(catalogue), Some((revision, pose))) = (catalogue, scene.primary_presentation)
            && revision == catalogue.revision
            && let Some(primary) = &catalogue.primary
        {
            match &primary.render {
                RenderAppearance::StarterX15 { origin_body_m } => {
                    let mut instance = *primary_instance;
                    instance.position_m +=
                        scene
                            .sample
                            .frame
                            .render_to_body_direction(inertial_to_render_axes(
                                pose.orientation_body_to_inertial() * *origin_body_m,
                            ));
                    self.instances.push(instance);
                    self.ranges[..self.primary_primitives].fill(0..1);
                }
                RenderAppearance::Available(geometry) => {
                    if let Some((_, primitives, anchor)) = self.radii.get(&VehicleId::PRIMARY)
                        && let Some(root) = render_root(pose, geometry, *anchor)
                        && let Some(instance) = (Track {
                            previous: None,
                            latest: root,
                        })
                        .at(
                            scene.sample.time,
                            scene.celestial.ephemeris(),
                            scene.flight.as_ref().expect("flight").reference_body,
                        )
                    {
                        let index = self.instances.len() as u32;
                        self.instances
                            .push(in_body_frame(instance, scene.sample.frame));
                        self.ranges[primitives.clone()].fill(index..index + 1);
                    }
                }
                _ => {}
            }
        }
        if self.enabled && observer.is_some_and(|o| !o.stale) && matched.is_some() {
            let body = scene.flight.as_ref().expect("server flight").reference_body;
            for (id, (radius, primitives, _)) in &self.radii {
                if id.is_primary() {
                    continue;
                }
                let Some(instance) = self
                    .tracks
                    .get(id)
                    .and_then(|t| t.at(scene.sample.time, scene.celestial.ephemeris(), body))
                else {
                    self.epoch_withheld += 1;
                    continue;
                };
                let instance = in_body_frame(instance, scene.sample.frame);
                // Angular-size quality fallback. Near/intersecting bounds are
                // never culled. This estimates footprint, not exact visibility.
                let distance = (instance.position_m - view.world_position_m).length();
                let pixels = radius * f64::from(height) * view.clip_from_view.y_axis.y.abs()
                    / (distance - radius).max(1e-6);
                let potential_caster = self
                    .shadows
                    .as_ref()
                    .is_some_and(|s| distance - radius <= s.caster_distance_m());
                if pixels < f64::from(self.min_pixels) && !potential_caster {
                    self.size_culled += 1;
                    continue;
                }
                let index = self.instances.len() as u32;
                self.instances.push(instance);
                self.ranges[primitives.clone()].fill(index..index + 1);
            }
        }
        self.update_environment(scene, backend);
        if let Some(inner) = &mut self.inner {
            inner.update_selected(
                backend,
                &view,
                &scene.lighting(),
                &self.instances,
                Some(&self.ranges),
            )
        } else {
            Ok(())
        }
    }

    fn update_environment(&mut self, scene: &Scene, backend: &Backend) {
        let revision = scene.environment_revision();
        if self.environment_revision != Some(revision)
            && let Some(inner) = &mut self.inner
        {
            // An initially disabled renderer already owns the placeholder.
            if let Some(binding) = scene
                .gpu_environment
                .as_ref()
                .filter(|_| scene.gpu_environment_revision == Some(revision))
            {
                inner.share_environment(backend, binding);
            } else if scene.environment_grid().is_some() || self.environment_revision.is_some() {
                inner.update_environment(backend, scene.environment_grid());
            }
            self.environment_revision = Some(revision);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use thessa_sim_core::{RenderGeometry, RenderMesh, RenderPart};

    fn builtin_primary() -> thessa_flight_net::geometry::FleetVehicleGeometry {
        thessa_flight_net::geometry::FleetVehicleGeometry {
            vehicle_id: VehicleId::PRIMARY,
            display_name: "explicit fixture asset".into(),
            contact: thessa_flight_net::geometry::ContactGeometry::Absent,
            render: RenderAppearance::StarterX15 {
                origin_body_m: DVec3::ZERO,
            },
        }
    }

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn authoritative_primary_replaces_x15_and_shares_the_rotating_body_frame_with_fleet() {
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let backend = Backend::bring_up().unwrap();
        let mut scene = Scene::load(Default::default()).unwrap();
        scene.enable_live_flight().unwrap();
        let time = SimTime(1234.0);
        scene.sample = scene.celestial.sample(time).unwrap();
        let frame = scene.sample.frame;
        let view = scene.view(320, 240, 1.0);
        let camera_rotation = DQuat::from_mat3(&glam::DMat3::from_mat4(view.world_from_view));
        let pose = |x| {
            let body_position = view.world_position_m + camera_rotation * DVec3::new(x, 0.0, -10.0);
            let render = frame.body_to_render_point(body_position);
            FlightPose::new(
                time,
                DVec3::new(render.x, -render.z, render.y),
                DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2)
                    * frame.rotation()
                    * camera_rotation,
            )
            .unwrap()
        };
        let primary_pose = pose(-2.0);
        let mut catalogue = layout();
        let mut primary = catalogue.vehicles.remove(0);
        primary.vehicle_id = VehicleId::PRIMARY;
        let RenderAppearance::Available(render) = &mut primary.render else {
            unreachable!()
        };
        render.parts.truncate(1);
        render.parts[0].position_body_m = DVec3::ZERO;
        render.parts[0].orientation_mesh_to_body = DQuat::IDENTITY;
        let mut secondary = primary.clone();
        secondary.vehicle_id = VehicleId::new(1);
        catalogue.primary = Some(primary);
        catalogue.vehicles = vec![secondary];
        catalogue.validate().unwrap();
        scene.primary_presentation = Some((1, primary_pose));
        scene.remote = Some(RemoteFlight {
            link: None,
            commands: vec![],
            started: std::time::Instant::now(),
            timeout: std::time::Duration::from_secs(5),
            snapshots: 1,
            last_tick: Some(0),
            parachutes: None,
            fleet: FleetObserver {
                rows: vec![thessa_flight_net::presentation::FleetPose {
                    vehicle_id: VehicleId::new(1),
                    tick: 0,
                    stopped: false,
                    pose: pose(2.0),
                }],
                updates: 1,
                geometry_revision: 1,
                ..Default::default()
            },
        });
        let mut meshes = scene
            .mesh_renderer(&backend, OffscreenTarget::FORMAT)
            .unwrap()
            .unwrap();
        let image = |scene: &Scene, meshes: &mut SceneMeshes| {
            let target = OffscreenTarget::new(&backend, 320, 240).unwrap();
            scene.update_mesh(&backend, meshes, 320, 240, 1.0).unwrap();
            let mut encoder = backend.device.create_command_encoder(&Default::default());
            {
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
            }
            meshes.encode(&mut encoder, &target.color, &target.depth);
            backend.queue.submit([encoder.finish()]);
            target.read_rgba8(&backend).unwrap()
        };
        let black = image(&scene, &mut meshes);
        assert_eq!(
            meshes.draw_stats().triangles,
            0,
            "no implicit X-15 while appearance is unknown"
        );
        assert!(meshes.inner.is_none() && meshes.source_geometry.is_none());
        scene.remote.as_mut().unwrap().fleet.geometry = Some(Arc::new(catalogue.clone()));
        let actual = image(&scene, &mut meshes);
        assert_eq!(
            meshes.draw_stats().triangles,
            2,
            "actual PRIMARY + secondary, not the 78k-triangle compatibility asset"
        );
        assert_eq!(meshes.draw_stats().instances, 2);
        let count = |bytes: &[u8], left: bool| {
            bytes
                .as_chunks::<4>()
                .0
                .iter()
                .enumerate()
                .filter(|(i, p)| (*i % 320 < 160) == left && p[..3] != [0, 0, 0])
                .count()
        };
        assert!(
            count(&actual, true) > 100 && count(&actual, false) > 100,
            "both meshes must use the same non-epoch rotating body frame"
        );
        let shift = DVec3::new(0.125, -0.25, 0.5);
        let RenderAppearance::Available(render) = &mut catalogue.primary.as_mut().unwrap().render
        else {
            unreachable!()
        };
        render.parts[0].position_body_m -= shift;
        catalogue.revision = 2;
        scene.remote.as_mut().unwrap().fleet.geometry = Some(Arc::new(catalogue.clone()));
        scene.remote.as_mut().unwrap().fleet.geometry_revision = 2;
        scene.primary_presentation = Some((
            2,
            FlightPose::new(
                time,
                primary_pose.relative_position_m()
                    + primary_pose.orientation_body_to_inertial() * shift,
                primary_pose.orientation_body_to_inertial(),
            )
            .unwrap(),
        ));
        assert_eq!(actual, image(&scene, &mut meshes));
        assert_eq!(
            meshes.source_publications, 1,
            "PRIMARY COM correction reuses the existing GPU source"
        );
        let paired = scene.primary_presentation;
        scene.primary_presentation.as_mut().unwrap().0 = 1;
        let mismatch = image(&scene, &mut meshes);
        assert_eq!(
            count(&mismatch, true),
            0,
            "never attach a new PRIMARY COM frame to an old pose"
        );
        assert!(count(&mismatch, false) > 100);
        scene.primary_presentation = paired;
        scene.requested.renderer.fleet_meshes = false;
        let mut disabled = scene
            .mesh_renderer(&backend, OffscreenTarget::FORMAT)
            .unwrap()
            .unwrap();
        let only_primary = image(&scene, &mut disabled);
        assert!(count(&only_primary, true) > 100 && count(&only_primary, false) == 0);
        assert_eq!(disabled.draw_stats().triangles, 1);
        assert!(
            disabled.tracks.is_empty()
                && disabled
                    .source_geometry
                    .as_ref()
                    .unwrap()
                    .vehicles
                    .is_empty()
        );
        catalogue.primary = None;
        catalogue.revision = 3;
        scene.remote.as_mut().unwrap().fleet.geometry = Some(Arc::new(catalogue));
        scene.remote.as_mut().unwrap().fleet.geometry_revision = 3;
        scene.primary_presentation.as_mut().unwrap().0 = 3;
        assert_eq!(
            black,
            image(&scene, &mut disabled),
            "missing PRIMARY appearance is invisible, not a guessed X-15"
        );
        assert!(disabled.inner.is_none());
        for render in [RenderAppearance::Absent, RenderAppearance::OmittedBudget] {
            let mut catalogue = (**scene
                .remote
                .as_ref()
                .unwrap()
                .fleet
                .geometry
                .as_ref()
                .unwrap())
            .clone();
            catalogue.revision += 1;
            let mut primary = builtin_primary();
            primary.render = render;
            catalogue.primary = Some(primary);
            scene.primary_presentation.as_mut().unwrap().0 = catalogue.revision;
            scene.remote.as_mut().unwrap().fleet.geometry_revision = catalogue.revision;
            scene.remote.as_mut().unwrap().fleet.geometry = Some(Arc::new(catalogue));
            assert_eq!(
                black,
                image(&scene, &mut disabled),
                "absent/over-budget PRIMARY must never substitute a compatibility asset"
            );
            assert!(disabled.inner.is_none());
        }
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("target/test-tmp/primary-fleet-body-frame-oracle.png");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let mut encoder = png::Encoder::new(std::fs::File::create(&path).unwrap(), 320, 240);
        encoder.set_color(png::ColorType::Rgba);
        encoder.set_depth(png::BitDepth::Eight);
        encoder
            .write_header()
            .unwrap()
            .write_image_data(&actual)
            .unwrap();
        eprintln!(
            "PRIMARY/fleet nonzero-epoch body-frame oracle: {}",
            path.display()
        );
    }

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

    fn layout() -> FleetGeometry {
        use thessa_flight_net::geometry::{ContactGeometry, FleetVehicleGeometry};
        FleetGeometry {
            revision: 1,
            primary: None,
            vehicles: vec![FleetVehicleGeometry {
                vehicle_id: VehicleId::new(1),
                display_name: "shell".into(),
                contact: ContactGeometry::Absent,
                render: RenderAppearance::Available(RenderGeometry {
                    parts: [DVec3::new(2.0, -3.0, 4.0), DVec3::new(6.0, 7.0, 8.0)]
                        .into_iter()
                        .map(|position_body_m| RenderPart {
                            name: "part".into(),
                            assembly_body: None,
                            position_body_m,
                            orientation_mesh_to_body: DQuat::from_rotation_z(0.3),
                            mesh: triangle(),
                        })
                        .collect(),
                }),
            }],
        }
    }

    fn render_parts(catalogue: &mut FleetGeometry) -> &mut Vec<RenderPart> {
        let RenderAppearance::Available(geometry) = &mut catalogue.vehicles[0].render else {
            unreachable!()
        };
        &mut geometry.parts
    }

    #[test]
    fn layout_reuses_common_translations_but_not_deformation_or_source_changes() {
        let before = layout();
        before.validate().unwrap();
        let mut shifted = before.clone();
        shifted.revision += 1;
        shifted.vehicles[0].display_name = "renamed".into();
        for part in render_parts(&mut shifted) {
            part.position_body_m -= DVec3::new(0.125, -0.25, 0.5);
            part.name = "renamed part".into();
            // Network decoding allocates new Arcs; equal payloads still reuse
            // the GPU publication, not just in-process pointer-equal sources.
            part.mesh = Arc::new((*part.mesh).clone());
        }
        assert!(same_render_layout(&before, &shifted));
        let mut changed = shifted.clone();
        render_parts(&mut changed)[1].position_body_m.x += 0.001;
        assert!(!same_render_layout(&before, &changed));
        let mut changed = shifted.clone();
        render_parts(&mut changed)[0].orientation_mesh_to_body = DQuat::IDENTITY;
        assert!(!same_render_layout(&before, &changed));
        for field in 0..3 {
            let mut changed = shifted.clone();
            let mesh = Arc::make_mut(&mut render_parts(&mut changed)[0].mesh);
            match field {
                0 => mesh.positions[0].x += 0.01,
                1 => mesh.normals[0] = DVec3::Y,
                _ => mesh.indices.swap(0, 1),
            }
            assert!(!same_render_layout(&before, &changed));
        }
        let mut changed = shifted.clone();
        render_parts(&mut changed).pop();
        assert!(!same_render_layout(&before, &changed));
        let mut changed = shifted.clone();
        changed.vehicles[0].vehicle_id = VehicleId::new(2);
        assert!(!same_render_layout(&before, &changed));
        for unavailable in [RenderAppearance::Absent, RenderAppearance::OmittedBudget] {
            let mut changed = shifted.clone();
            changed.vehicles[0].render = unavailable;
            assert!(!same_render_layout(&before, &changed));
        }
        assert!(!same_render_layout(
            &before,
            &FleetGeometry {
                revision: 2,
                primary: None,
                vehicles: vec![]
            }
        ));
    }

    #[test]
    fn layout_error_is_bounded_against_the_publication_not_previous_packet() {
        let before = layout();
        let mut previous = before.clone();
        for step in 1..=8 {
            let mut changed = before.clone();
            render_parts(&mut changed)[1].position_body_m.x +=
                f64::from(step) * RENDER_ORIGIN_ERROR_M * 0.25;
            assert!(same_render_layout(&previous, &changed));
            if step >= 5 {
                assert!(!same_render_layout(&before, &changed));
            }
            previous = changed;
        }
    }

    #[test]
    fn rotating_world_root_keeps_bracket_across_com_recentering() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        let body = ephemeris.body_id("thessa").unwrap();
        let mut catalogue = layout();
        let RenderAppearance::Available(geometry) = &mut catalogue.vehicles[0].render else {
            unreachable!()
        };
        let a = SimTime(1234.0);
        let b = SimTime(1234.1);
        let origin = DVec3::new(3e12, -2e12, 1e12);
        let pose = |time, point, rotation| {
            FlightPose::new(
                time,
                point - ephemeris.body_state(body, time).unwrap().position_inertial,
                rotation,
            )
            .unwrap()
        };
        let first = pose(a, origin, DQuat::from_rotation_y(0.7));
        let last = pose(
            b,
            origin + DVec3::new(4.0, 2.0, 1.0),
            DQuat::from_rotation_y(1.1),
        );
        let anchor = geometry.parts[0].position_body_m;
        // The reference is the existing body-origin interpolation, not an
        // interpolated part-origin chord. Pure rotation must remain unchanged.
        let mut reference = Track {
            previous: None,
            latest: first,
        };
        reference.push(last);
        let mut recentered = Track {
            previous: None,
            latest: render_root(first, geometry, anchor).unwrap(),
        };
        let shift = DVec3::new(0.125, -0.25, 0.5);
        for part in &mut geometry.parts {
            part.position_body_m -= shift;
        }
        let shifted_pose = FlightPose::new(
            last.time(),
            last.relative_position_m() + last.orientation_body_to_inertial() * shift,
            last.orientation_body_to_inertial(),
        )
        .unwrap();
        recentered.push(render_root(shifted_pose, geometry, anchor).unwrap());
        for step in 0..=10 {
            let time = SimTime(a.0 + (b.0 - a.0) * f64::from(step) / 10.0);
            let expected = reference.at(time, &ephemeris, body).unwrap();
            let actual = recentered.at(time, &ephemeris, body).unwrap();
            assert!((expected.position_m - actual.position_m).length() < 0.001);
            assert!((expected.rotation * DVec3::X - actual.rotation * DVec3::X).length() < 1e-12);
        }
        assert!(
            recentered
                .at(SimTime(b.0 + 0.01), &ephemeris, body)
                .is_none()
        );
        // New sources keep their publication's body frame, not a part origin.
        let expected = geometry.parts[0].position_body_m
            + geometry.parts[0].orientation_mesh_to_body * geometry.parts[0].mesh.positions[0];
        let mut source = MeshAsset {
            primitives: vec![],
            materials: vec![],
            textures: vec![],
            images: vec![],
        };
        append_catalogue(&mut source, &catalogue, &mut BTreeMap::new(), true).unwrap();
        assert!(
            (glam::Vec3::from(source.primitives[0].vertices[0].position).as_dvec3() - expected)
                .length()
                < 0.001
        );
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
            primary: Some(builtin_primary()),
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
            parachutes: None,
            fleet: FleetObserver {
                rows,
                updates: 1,
                geometry_revision: 1,
                geometry_updates: 1,
                geometry: Some(catalogue),
                ..Default::default()
            },
        });
        let primary_pose = scene.primary_presentation.unwrap().1;
        scene.primary_presentation = Some((1, primary_pose));
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
        let shift = DVec3::new(0.125, -0.25, 0.5);
        let fleet = &mut scene.remote.as_mut().unwrap().fleet;
        let mut shifted = (**fleet.geometry.as_ref().unwrap()).clone();
        shifted.revision = 2;
        for vehicle in &mut shifted.vehicles {
            let RenderAppearance::Available(geometry) = &mut vehicle.render else {
                unreachable!()
            };
            for part in &mut geometry.parts {
                part.position_body_m -= shift;
            }
        }
        for row in &mut fleet.rows {
            row.pose = FlightPose::new(
                SimTime(0.1),
                row.pose.relative_position_m() + row.pose.orientation_body_to_inertial() * shift,
                row.pose.orientation_body_to_inertial(),
            )
            .unwrap();
        }
        fleet.geometry = Some(Arc::new(shifted));
        fleet.updates += 1;
        // Independently coalesced catalogue/pose slots may disagree briefly;
        // don't consume these poses under the wrong render frame.
        scene.sample.time = SimTime(0.1);
        let unmatched = render(&scene, &mut meshes, 320, 240);
        assert!(
            unmatched
                .as_chunks::<4>()
                .0
                .iter()
                .all(|p| p[..3] == [0, 0, 0])
        );
        scene.remote.as_mut().unwrap().fleet.geometry_revision = 2;
        scene.primary_presentation = Some((2, primary_pose));
        assert_eq!(
            image,
            render(&scene, &mut meshes, 320, 240),
            "COM origin change must not move any rendered surface"
        );
        assert_eq!(
            meshes.source_publications, 1,
            "COM shift updates matrices, not GPU source buffers"
        );
        scene.sample.time = SimTime(0.05);
        assert_eq!(
            image,
            render(&scene, &mut meshes, 320, 240),
            "COM-only revisions preserve the world-space bracket"
        );
        assert_eq!(meshes.epoch_withheld, 0);
        assert!(meshes.tracks.values().all(|track| track.previous.is_some()));
        // Synthetic renderer-level reset for the existing out-of-bracket case.
        scene.sample.time = SimTime(0.0);
        let fleet = &mut scene.remote.as_mut().unwrap().fleet;
        for row in &mut fleet.rows {
            row.pose = FlightPose::new(
                SimTime(0.0),
                row.pose.relative_position_m(),
                row.pose.orientation_body_to_inertial(),
            )
            .unwrap();
        }
        fleet.updates += 1;
        assert_eq!(image, render(&scene, &mut meshes, 320, 240));
        let before = meshes.draw_stats();
        assert!(
            meshes
                .inner
                .as_mut()
                .unwrap()
                .update_selected(
                    &backend,
                    &view,
                    &scene.lighting(),
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
        assert_eq!(
            disabled.source_publications, 1,
            "PRIMARY publication is independent of the disabled fleet effect"
        );
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
            revision: 3,
            primary: Some(builtin_primary()),
            vehicles: (1..64)
                .map(|id| {
                    let mut g = geometry.clone();
                    g.vehicle_id = VehicleId::new(id);
                    g
                })
                .collect(),
        }));
        fleet.geometry_revision = 3;
        fleet.updates += 1;
        scene.primary_presentation = Some((3, primary_pose));
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
        // Full admitted vertex/part workload. Repeated triangles intentionally
        // hold visibility constant; this is upload/selection work, not a fleet
        // scene or an aesthetic comparison. Fresh Arcs model decoded packets.
        let mut full = layout();
        full.primary = Some(builtin_primary());
        let part = render_parts(&mut full)[0].clone();
        let positions = triangle().positions.clone();
        *render_parts(&mut full) = (0..8)
            .map(|i| {
                let mut part = part.clone();
                part.position_body_m = DVec3::new(f64::from(i) * 0.01, 0.0, 0.0);
                part.orientation_mesh_to_body = DQuat::IDENTITY;
                part.mesh = Arc::new(RenderMesh {
                    positions: (0..128).map(|i| positions[i % 3]).collect(),
                    normals: vec![DVec3::Z; 128],
                    indices: (0..510).map(|i| i % 3).collect(),
                });
                part
            })
            .collect();
        full.vehicles = (1..64)
            .map(|id| {
                let mut vehicle = full.vehicles[0].clone();
                vehicle.vehicle_id = VehicleId::new(id);
                vehicle
            })
            .collect();
        full.revision = 4;
        full.validate().unwrap();
        let fleet = &mut scene.remote.as_mut().unwrap().fleet;
        fleet.geometry = Some(Arc::new(full.clone()));
        fleet.geometry_revision = full.revision;
        fleet.updates += 1;
        scene.primary_presentation = Some((4, primary_pose));
        scene
            .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
            .unwrap();
        assert_eq!(meshes.draw_stats().draw_calls, 505);
        let first_full = render(&scene, &mut meshes, 320, 240);
        let mut timings = [Vec::new(), Vec::new()];
        let mut revision = 4;
        for force_rebuild in [false, true, true, false] {
            let publications = meshes.source_publications;
            for sample in 0..13 {
                revision += 1;
                let shift = DVec3::new(revision as f64 * 0.125, -0.25, 0.5);
                let mut packet = full.clone();
                packet.revision = revision;
                for vehicle in &mut packet.vehicles {
                    let RenderAppearance::Available(geometry) = &mut vehicle.render else {
                        unreachable!()
                    };
                    for part in &mut geometry.parts {
                        part.position_body_m -= shift;
                        part.mesh = Arc::new((*part.mesh).clone());
                    }
                }
                packet.validate().unwrap();
                let fleet = &mut scene.remote.as_mut().unwrap().fleet;
                fleet.geometry = Some(Arc::new(packet));
                fleet.geometry_revision = revision;
                fleet.updates += 1;
                for row in &mut fleet.rows {
                    row.pose = FlightPose::new(
                        template.pose.time(),
                        template.pose.relative_position_m()
                            + template.pose.orientation_body_to_inertial() * shift,
                        template.pose.orientation_body_to_inertial(),
                    )
                    .unwrap();
                }
                scene.primary_presentation = Some((revision, primary_pose));
                if force_rebuild {
                    meshes.source_geometry = None;
                }
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
                if sample >= 3 {
                    timings[usize::from(force_rebuild)].push(start.elapsed().as_secs_f64() * 1e6);
                }
                assert_eq!(meshes.draw_stats().instances, 64);
                assert_eq!(meshes.draw_stats().draw_calls, 505);
            }
            assert_eq!(
                meshes.source_publications - publications,
                if force_rebuild { 13 } else { 0 }
            );
            assert_eq!(
                first_full,
                render(&scene, &mut meshes, 320, 240),
                "COM reuse/rebuild must produce identical pixels"
            );
        }
        for times in &mut timings {
            times.sort_by(f64::total_cmp);
        }
        eprintln!(
            "fleet_com_workload secondaries=63 parts_each=8 vertices_each=1024 indices_each=4080 resolution=320x240 ABBA samples_each=20 warmup_per_block=3 reuse_median_us={:.3} forced_source_rebuild_median_us={:.3}; fresh decoded-equivalent payloads, includes layout scan/update/clear/submit/GPU completion, excludes packet preparation/readback; forced invalidation is a diagnostic alternative, not an exact prior executable or production FPS",
            timings[0][10], timings[1][10]
        );
        // Explicit membership removal retires the retained source and tracks.
        let fleet = &mut scene.remote.as_mut().unwrap().fleet;
        fleet.geometry = Some(Arc::new(FleetGeometry {
            revision: revision + 1,
            primary: Some(builtin_primary()),
            vehicles: vec![],
        }));
        fleet.geometry_revision = revision + 1;
        fleet.rows.clear();
        fleet.updates += 1;
        scene.primary_presentation = Some((revision + 1, primary_pose));
        let publications = meshes.source_publications;
        assert_eq!(empty, render(&scene, &mut meshes, 320, 240));
        assert_eq!(meshes.source_publications, publications + 1);
        assert!(meshes.radii.is_empty() && meshes.tracks.is_empty());
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
