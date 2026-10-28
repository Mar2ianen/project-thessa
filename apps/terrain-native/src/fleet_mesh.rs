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
    plant_context: Option<crate::surface_plants::Context>,
    plants: Option<crate::surface_plants::Prepared>,
    plant_jobs: crate::surface_plants::PreparationQueue,
    plant_capacity: usize,
    plant_epoch: u64,
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

pub(super) fn empty_asset() -> MeshAsset {
    MeshAsset {
        primitives: vec![],
        materials: vec![],
        textures: vec![],
        images: vec![],
    }
}

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
                ambient_visibility: vec![],
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
        let context = scene.surface_plants_context(&resolved);
        let (plants, diagnostics) =
            crate::surface_plants::prepare(context, &resolved, scene.stream.field())
                .map_err(BackendError::Device)?;
        Self::with_prepared_plants(
            scene,
            backend,
            source,
            format,
            shadows,
            terrain,
            resolved,
            context,
            plants,
            diagnostics,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn with_prepared_plants(
        scene: &Scene,
        backend: &Backend,
        source: &MeshAsset,
        format: wgpu::TextureFormat,
        shadows: Option<&thessa_render_wgpu::shadows::ShadowAtlas>,
        terrain: Option<thessa_render_wgpu::terrain_ray::TerrainRaySource>,
        resolved: thessa_graphics::ResolvedGraphicsSettings,
        context: crate::surface_plants::Context,
        plants: Option<crate::surface_plants::Prepared>,
        mut diagnostics: crate::surface_plants::Diagnostics,
    ) -> Result<Self, BackendError> {
        let started = std::time::Instant::now();
        // A whole raster batch survives an undersized optional TLAS budget.
        // Withhold RT only for this consumer, before any GPU publication; never
        // truncate instances or mutate the scene/global resolved policy.
        let plant_rt_admitted = plants
            .as_ref()
            .is_none_or(|plants| plants.rt_admitted(&resolved));
        let source = plants.as_ref().map_or(source, |plants| &plants.asset);
        let capacity = plants.as_ref().map_or_else(
            || if scene.docking.is_some() { 2 } else { 1 },
            |plants| plants.instances.len(),
        );
        let (quality, enabled, min_pixels) = (
            resolved.surface_quality,
            resolved.fleet_meshes,
            resolved.fleet_min_pixels,
        );
        let mut mesh_settings = resolved.clone();
        if !plant_rt_admitted {
            mesh_settings.ray_tracing = thessa_graphics::ResolvedRayTracing::Off;
        }
        let mut meshes = Self {
            plant_context: scene.vehicle.is_none().then_some(context),
            shadows: shadows.cloned(),
            environment_revision: None,
            inner: if scene.server_flight() || source.primitives.is_empty() {
                None
            } else {
                Some((if plants.is_some() {
                    MeshRenderer::with_static_surface_and_terrain
                } else {
                    MeshRenderer::with_surface_and_terrain
                })(
                    backend,
                    source,
                    thessa_render_wgpu::mesh::SurfaceSettings {
                        format,
                        capacity,
                        quality,
                        reflections: resolved.specular_reflections,
                        shadows,
                        ray_tracing: Some(&mesh_settings),
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
            instances: if scene.vehicle.is_some() {
                Vec::with_capacity(64)
            } else {
                Vec::new()
            },
            ranges: vec![0..1; source.primitives.len()],
            source_publications: 0,
            epoch_withheld: 0,
            size_culled: 0,
            plants,
            plant_jobs: Default::default(),
            plant_capacity: capacity,
            plant_epoch: 1,
        };
        if let (Some(inner), Some(plants)) = (&mut meshes.inner, &meshes.plants)
            && shadows.is_some()
            && !plants.shadow_lods.is_empty()
        {
            inner.set_shadow_lods(backend, &plants.shadow_lods)?;
        }
        diagnostics.publication_ms = started.elapsed().as_secs_f64() * 1000.0;
        *scene.surface_plants_diagnostics.borrow_mut() = diagnostics;
        Ok(meshes)
    }

    pub fn draw_stats(&self) -> MeshDrawStats {
        self.inner
            .as_ref()
            .map_or_else(MeshDrawStats::default, MeshRenderer::draw_stats)
    }

    pub fn opaque(&self) -> Option<&MeshRenderer> {
        self.inner.as_ref()
    }
    pub fn temporal_epoch(&self) -> u64 {
        self.plant_epoch
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

    /// Applied after the ordinary mesh update so all source replacements and
    /// early local/server branches share the same primary sampling contract.
    pub fn update_primary_jitter(
        &mut self,
        scene: &Scene,
        backend: &Backend,
        view: &thessa_render_core::ViewParams,
        extent: [u32; 2],
        jitter: thessa_render_core::temporal::JitterOffset,
    ) -> Result<(), BackendError> {
        if let Some(inner) = &mut self.inner {
            inner.update_primary_jitter(backend, view, extent, jitter)?;
        }
        let stats = self.draw_stats();
        let mut diagnostics = scene.surface_plants_diagnostics.borrow_mut();
        diagnostics.submitted_instances = stats.instances;
        diagnostics.draw_calls = stats.draw_calls;
        diagnostics.frame_upload_bytes = stats.frame_upload_bytes;
        Ok(())
    }

    pub fn update(
        &mut self,
        scene: &Scene,
        backend: &Backend,
        width: u32,
        height: u32,
        exposure: f32,
    ) -> Result<(), BackendError> {
        if let Some(context) = self.plant_context {
            let next = scene.surface_plants_context(&self.resolved);
            let completed = self.plant_jobs.poll(next);
            if next != context {
                use crate::surface_plants::Context;
                let near_move = matches!(
                    (context, next),
                    (Context::SurveyWithUnderstory { canopy: a, .. }, Context::SurveyWithUnderstory { canopy: b, .. }) if a == b
                );
                let ready = if near_move {
                    if completed.is_none() {
                        self.plant_jobs
                            .request(next, &self.resolved, scene.stream.field());
                    }
                    completed
                } else {
                    // Mode/site changes release incompatible plants immediately.
                    Some(crate::surface_plants::prepare(
                        next,
                        &self.resolved,
                        scene.stream.field(),
                    ))
                };
                if let Some(ready) = ready {
                    let epoch = self.plant_epoch.wrapping_add(1);
                    let (plants, mut diagnostics) = ready.map_err(BackendError::Device)?;
                    let reuse = self.inner.is_some()
                        && self.plants.as_ref().zip(plants.as_ref()).is_some_and(
                            |(before, after)| {
                                after.instances.len() <= self.plant_capacity
                                    && before.same_prototypes(after)
                                    && before.rt_admitted(&self.resolved)
                                        == after.rt_admitted(&self.resolved)
                            },
                        );
                    if reuse {
                        let started = std::time::Instant::now();
                        self.plants = plants;
                        self.plant_context = Some(next);
                        diagnostics.renderer_reused = true;
                        diagnostics.publication_ms = started.elapsed().as_secs_f64() * 1000.0;
                        *scene.surface_plants_diagnostics.borrow_mut() = diagnostics;
                    } else {
                        let empty = empty_asset();
                        let source = scene.vehicle.as_ref().map_or(&empty, |(source, _)| source);
                        let mut replacement = Self::with_prepared_plants(
                            scene,
                            backend,
                            source,
                            self.format,
                            self.shadows.as_ref(),
                            self.terrain.clone(),
                            self.resolved.clone(),
                            next,
                            plants,
                            diagnostics,
                        )?;
                        replacement.plant_jobs = std::mem::take(&mut self.plant_jobs);
                        *self = replacement;
                    }
                    self.plant_epoch = epoch;
                }
            }
            let mut diagnostics = scene.surface_plants_diagnostics.borrow_mut();
            diagnostics.preparation_job_pending = self.plant_jobs.pending().is_some();
            diagnostics.preparation_jobs_started = self.plant_jobs.started;
            diagnostics.preparation_jobs_discarded = self.plant_jobs.discarded;
        }
        let view = scene.view(width, height, exposure);
        self.update_environment(scene, backend);
        let Some((primary, primary_instance)) = &scene.vehicle else {
            return if let Some(inner) = &mut self.inner {
                if let Some(plants) = &self.plants {
                    // Aggregated moss/litter/water draws add instances without scatter descriptors.
                    debug_assert!(plants.descriptors.len() <= plants.instances.len());
                    inner.update_selected(
                        backend,
                        &view,
                        &scene.lighting(),
                        &plants.instances,
                        Some(&plants.ranges),
                    )?;
                    let stats = inner.draw_stats();
                    let mut diagnostics = scene.surface_plants_diagnostics.borrow_mut();
                    diagnostics.submitted_instances = stats.instances;
                    diagnostics.draw_calls = stats.draw_calls;
                    diagnostics.frame_upload_bytes = stats.frame_upload_bytes;
                    Ok(())
                } else {
                    inner.update(backend, &view, &scene.lighting(), &[])
                }
            } else {
                Ok(())
            };
        };
        if let Some(demo) = &scene.docking {
            let instances = demo
                .instances(*primary_instance)
                .map_err(|e| BackendError::Device(e.to_string()))?;
            return self
                .inner
                .as_mut()
                .expect("local docking source")
                .update_selected(
                    backend,
                    &view,
                    &scene.lighting(),
                    &instances,
                    Some(&demo.ranges),
                );
        }
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

    #[test]
    #[ignore = "requires a wgpu adapter"]
    fn survey_plants_retain_body_fixed_descriptors_and_submit_grouped_ranges() {
        survey_plant_lifecycle(false);
    }

    #[test]
    #[ignore = "hardware: bounded near-eye understory reuses grouped raster/shadow paths, follows window boundaries and restores exact descriptors"]
    fn near_understory_reuses_the_shared_plant_and_shadow_lifecycle() {
        survey_plant_lifecycle(true);
    }

    fn survey_plant_lifecycle(understory: bool) {
        use thessa_render_wgpu::terrain::OffscreenTarget;
        let backend = Backend::bring_up().unwrap();
        let mut requested = thessa_graphics::RequestedGraphics::default();
        requested.surface_plants.enabled = true;
        requested.surface_plants.understory = understory;
        requested.shadows.enabled = true;
        requested.shadows.quality = thessa_graphics::Quality::Low;
        requested.shadows.cascades = 1;
        requested.shadows.map_size = 256;
        let mut scene = Scene::load(requested).unwrap();
        scene.select_fixed_survey_site([23.565679258039204, 165.13881581863882]);
        if understory {
            scene.survey.set_distance_m(80.0);
            scene.survey_eye_agl_m = Some(2.0);
        }
        let settings = backend.resolve_graphics(&scene.requested);
        let page = thessa_rcbt_core::HeightPage::bake(&[0.0; 4], 2, 0.001).unwrap();
        let mut terrain = TerrainRenderer::with_capacity_and_lighting(
            &backend,
            &[TerrainPatch {
                anchor: TileAnchor::new(TileKey::new(0, 0, 0, 0).unwrap(), scene.stream.radius_m())
                    .unwrap(),
                page: &page,
            }],
            OffscreenTarget::FORMAT,
            TerrainCapacity {
                patches: 1,
                height_grid_size: 2,
                material_layers: 1,
            },
            Some(&settings),
        )
        .unwrap();
        terrain.publish_partition(&backend, &[]).unwrap();
        let mut meshes = scene
            .mesh_renderer_with_shadows(&backend, OffscreenTarget::FORMAT, &terrain)
            .unwrap()
            .unwrap();
        // Freeze atlas camera/lights to isolate mesh-source cache invalidation.
        // This tests real depth encoding/cache lifetime, not pixel occlusion.
        let shadow_camera = scene.center + scene.up * 50.0;
        let lighting = scene.lighting();
        let draw_shadows =
            |scene: &Scene, meshes: &mut SceneMeshes, terrain: &mut TerrainRenderer| {
                scene.update_mesh(&backend, meshes, 320, 240, 1.0).unwrap();
                terrain
                    .update_shadows(&backend, shadow_camera, &lighting)
                    .unwrap();
                assert!(terrain.shadows().unwrap().storage_bytes() > 0);
                assert!(terrain.shadows().unwrap().active_passes() > 0);
                let mut encoder = backend.device.create_command_encoder(&Default::default());
                terrain.encode_surface(&mut encoder);
                terrain.encode_shadows(&mut encoder, meshes.opaque());
                backend.queue.submit([encoder.finish()]);
                terrain.last_shadow_passes()
            };
        let plants = meshes.plants.as_ref().unwrap();
        let descriptors = plants.descriptors.clone();
        let instances = plants.instances.clone();
        let draw_calls = plants.asset.primitives.len();
        let triangles: u64 = plants
            .asset
            .primitives
            .iter()
            .zip(&plants.ranges)
            .map(|(p, r)| p.indices.len() as u64 / 3 * u64::from(r.end - r.start))
            .sum();
        let preparation_ms = scene.surface_plants_diagnostics.borrow().preparation_ms;
        assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
        assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
        assert_eq!(meshes.draw_stats().instances as usize, descriptors.len());
        assert_eq!(meshes.draw_stats().draw_calls, draw_calls);
        assert_eq!(meshes.draw_stats().triangles, triangles);
        scene.survey.set_distance_m(80.0);
        scene.set_presentation_time(SimTime(1234.0)).unwrap();
        assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
        assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
        assert_eq!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
        assert_eq!(meshes.plants.as_ref().unwrap().instances, instances);
        assert_eq!(
            scene.surface_plants_diagnostics.borrow().preparation_ms,
            preparation_ms
        );
        #[cfg(feature = "hud")]
        {
            let window = crate::surface_plants::survey_window(scene.up);
            scene.set_map_open(true);
            assert!(
                draw_shadows(&scene, &mut meshes, &mut terrain) > 0,
                "removing plant casters must invalidate cached depth"
            );
            assert!(meshes.inner.is_none() && meshes.plants.is_none());
            assert_eq!(
                scene.surface_plants_metadata()["native_surface_plants_candidate_visits"],
                "0"
            );
            assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
            scene.set_map_open(false);
            assert!(
                draw_shadows(&scene, &mut meshes, &mut terrain) > 0,
                "returning to survey must regenerate and readmit the same window"
            );
            assert_eq!(
                scene.surface_plants_diagnostics.borrow().window,
                Some(window)
            );
            assert_eq!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
            assert_eq!(meshes.plants.as_ref().unwrap().instances, instances);
            let preparation_ms = scene.surface_plants_diagnostics.borrow().preparation_ms;
            assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
            assert_eq!(
                scene.surface_plants_diagnostics.borrow().preparation_ms,
                preparation_ms
            );
        }
        if understory {
            let near = scene.surface_plants_diagnostics.borrow().understory_window;
            scene.survey.set_distance_m(160.0);
            let request_started = std::time::Instant::now();
            assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
            let request_ms = request_started.elapsed().as_secs_f64() * 1000.0;
            assert_eq!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
            assert!(meshes.plant_jobs.pending().is_some());
            assert_eq!(meshes.plant_jobs.started, 1);
            let metadata = scene.surface_plants_metadata();
            assert_eq!(
                metadata["native_surface_plants_status"],
                "pending_understory_preparation_retaining_applied_window"
            );
            assert_eq!(
                metadata["native_surface_plants_preparation_job_pending"],
                "true"
            );
            assert!(metadata.contains_key("native_surface_understory_requested_window"));
            let wait_for_preparation = |scene: &Scene, meshes: &mut SceneMeshes| {
                let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
                while meshes.plant_jobs.pending().is_some() {
                    assert!(
                        std::time::Instant::now() < deadline,
                        "plant worker timed out"
                    );
                    scene.update_mesh(&backend, meshes, 320, 240, 1.0).unwrap();
                    std::thread::sleep(std::time::Duration::from_millis(1));
                }
            };
            wait_for_preparation(&scene, &mut meshes);
            assert_eq!(meshes.plant_jobs.started, 1);
            assert_ne!(
                scene.surface_plants_diagnostics.borrow().understory_window,
                near
            );
            assert_ne!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
            let (oracle, _) = crate::surface_plants::prepare(
                scene.surface_plants_context(&settings),
                &settings,
                scene.stream.field(),
            )
            .unwrap();
            let oracle = oracle.unwrap();
            assert_eq!(
                meshes.plants.as_ref().unwrap().descriptors,
                oracle.descriptors
            );
            assert_eq!(meshes.plants.as_ref().unwrap().instances, oracle.instances);
            {
                let diagnostics = scene.surface_plants_diagnostics.borrow();
                eprintln!(
                    "understory_async request_and_shadow_submit_ms={request_ms:.3} worker_preparation_ms={:.3} publication_ms={:.3}",
                    diagnostics.preparation_ms, diagnostics.publication_ms
                );
            }
            assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
            assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
            scene.survey.set_distance_m(80.0);
            assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
            assert!(meshes.plant_jobs.pending().is_some());
            wait_for_preparation(&scene, &mut meshes);
            assert_eq!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
            assert_eq!(meshes.plants.as_ref().unwrap().instances, instances);
            assert!(scene.surface_plants_diagnostics.borrow().renderer_reused);
            assert!(draw_shadows(&scene, &mut meshes, &mut terrain) > 0);
            assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
            #[cfg(feature = "hud")]
            {
                scene.survey.set_distance_m(160.0);
                scene
                    .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
                    .unwrap();
                assert!(meshes.plant_jobs.pending().is_some());
                scene.set_map_open(true);
                scene
                    .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
                    .unwrap();
                assert!(meshes.plants.is_none() && meshes.inner.is_none());
                wait_for_preparation(&scene, &mut meshes);
                assert!(meshes.plants.is_none() && meshes.inner.is_none());
                assert_eq!(meshes.plant_jobs.discarded, 1);
                scene.survey.set_distance_m(80.0);
                scene.set_map_open(false);
                scene
                    .update_mesh(&backend, &mut meshes, 320, 240, 1.0)
                    .unwrap();
                assert_eq!(meshes.plants.as_ref().unwrap().descriptors, descriptors);
            }
        }
        // A new selected window replaces the retained source once, not per frame.
        scene.select_site(1);
        assert!(
            draw_shadows(&scene, &mut meshes, &mut terrain) > 0,
            "replacing the selected plant window must invalidate cached depth"
        );
        let window = crate::surface_plants::survey_window(scene.up);
        assert_eq!(
            scene.surface_plants_diagnostics.borrow().window,
            Some(window)
        );
        let stats = scene.surface_plants_diagnostics.borrow();
        assert_eq!(stats.candidates, 9 * 4096 + stats.understory_candidates);
        drop(stats);
        let preparation_ms = scene.surface_plants_diagnostics.borrow().preparation_ms;
        assert_eq!(draw_shadows(&scene, &mut meshes, &mut terrain), 0);
        assert_eq!(
            scene.surface_plants_diagnostics.borrow().preparation_ms,
            preparation_ms
        );
        // Settings are restart-required: test disabled constructor recreation,
        // not live enable/quality mutation of the existing renderer.
        scene.requested.surface_plants.enabled = false;
        assert!(
            scene
                .mesh_renderer(&backend, OffscreenTarget::FORMAT)
                .unwrap()
                .is_none()
        );
    }

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
                ambient_visibility: vec![],
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
        // One primary primitive and one primitive for each of the two fleet shapes.
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
    #[test]
    #[ignore = "software Vulkan: real coastal primary plan/counts, no raster submission"]
    fn report_coastal_primary_workload() {
        assert!(
            !std::path::Path::new("/dev/dri").exists(),
            "use isolated CPU-only ICD"
        );
        let output = std::path::PathBuf::from(
            std::env::var_os("THESSA_WORKLOAD_OUTPUT").expect("set a NEW report path"),
        );
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(output)
            .unwrap();
        let mut request = thessa_graphics::RequestedGraphics::from_toml(include_str!(
            "../../../data/graphics/surface-showcase.toml"
        ))
        .unwrap();
        request.renderer.ray_tracing = thessa_graphics::RayTracingRequest::Off;
        let backend = Backend::bring_up_for_graphics(&request).unwrap();
        assert_eq!(
            backend.adapter.get_info().device_type,
            wgpu::DeviceType::Cpu
        );
        let mut scene = Scene::load(request).unwrap();
        scene.select_fixed_survey_site([23.565679258039204, 165.13881581863882]);
        scene.survey.set_distance_m(80.0);
        scene.survey_eye_agl_m = Some(2.0);
        let mut terrain = scene
            .renderer(
                &backend,
                thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            )
            .unwrap();
        let mut meshes = SceneMeshes::with_shadows(
            &scene,
            &backend,
            &empty_asset(),
            thessa_render_wgpu::terrain::OffscreenTarget::FORMAT,
            terrain.shadows(),
            None,
        )
        .unwrap();
        let mut samples = Vec::new();
        for sample in 0..35 {
            let start = std::time::Instant::now();
            scene
                .update_mesh(&backend, &mut meshes, 1280, 720, 1.0)
                .unwrap();
            if sample >= 5 {
                samples.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            backend.queue.submit([]);
        }
        samples.sort_by(f64::total_cmp);
        let stats = meshes.draw_stats();
        let view = scene.view(1280, 720, 1.0);
        terrain
            .update_view_for_extent(&backend, &view, [1280, 720])
            .unwrap();
        terrain
            .update_shadows(&backend, view.world_position_m, &scene.lighting())
            .unwrap();
        let shadow_started = std::time::Instant::now();
        let mut shadow_encoder = backend.device.create_command_encoder(&Default::default());
        terrain.encode_shadows(&mut shadow_encoder, meshes.opaque());
        let shadow_encode_ms = shadow_started.elapsed().as_secs_f64() * 1000.0;
        let shadow_stats = terrain.last_shadow_draws();
        // Count real encoded caster commands, but never submit a raster pass.
        drop(shadow_encoder);
        let mut layers = Vec::new();
        let mut caster_groups = Vec::new();
        let mut observed_encoder = backend.device.create_command_encoder(&Default::default());
        let observed = terrain.shadows().unwrap().encode_casters_observed(
            &mut observed_encoder,
            &terrain,
            meshes.opaque(),
            true,
            |layer, primitive, stats| {
                let mut entry: toml::Table = [
                    ("layer", layer as i64),
                    ("triangles", stats.mesh_submitted_triangles as i64),
                    ("candidate_triangles", stats.mesh_candidate_triangles as i64),
                    ("instances", stats.mesh_submitted_instances as i64),
                    ("lod_instances", stats.mesh_lod_instances as i64),
                    ("saved_triangles", stats.mesh_lod_triangles_saved as i64),
                    ("draw_calls", stats.mesh_draw_calls as i64),
                ]
                .into_iter()
                .map(|(k, v)| (k.to_owned(), toml::Value::Integer(v)))
                .collect();
                if let Some(p) = primitive {
                    entry.insert("primitive".into(), toml::Value::Integer(p as i64));
                    caster_groups.push(toml::Value::Table(entry));
                } else {
                    layers.push(toml::Value::Table(entry));
                }
            },
        );
        drop(observed_encoder);
        let mut expected_observed = shadow_stats;
        expected_observed.mesh_indirect_upload_bytes = 0;
        assert_eq!(observed, expected_observed);
        let previous_lods =
            crate::surface_plants::original_shadow_lods(meshes.plants.as_ref().unwrap());
        meshes
            .inner
            .as_mut()
            .unwrap()
            .set_shadow_lods(&backend, &previous_lods)
            .unwrap();
        terrain.invalidate_shadows();
        let mut previous_encoder = backend.device.create_command_encoder(&Default::default());
        terrain.encode_shadows(&mut previous_encoder, meshes.opaque());
        let previous_stats = terrain.last_shadow_draws();
        drop(previous_encoder);
        assert_eq!(
            previous_stats.mesh_submitted_triangles, 283_223_492,
            "same-build original shadow plan must reproduce doc 82"
        );
        meshes
            .inner
            .as_mut()
            .unwrap()
            .set_shadow_lods(&backend, &meshes.plants.as_ref().unwrap().shadow_lods)
            .unwrap();
        let plants = meshes.plants.as_ref().unwrap();
        let mut groups: Vec<_> = plants
            .asset
            .primitives
            .iter()
            .zip(&plants.ranges)
            .enumerate()
            .map(|(i, (p, r))| {
                let table: toml::Table = [
                    ("primitive", i as i64),
                    ("material", p.material as i64),
                    ("source_triangles", (p.indices.len() / 3) as i64),
                    ("instances", i64::from(r.end - r.start)),
                    (
                        "candidate_triangles",
                        (p.indices.len() / 3) as i64 * i64::from(r.end - r.start),
                    ),
                ]
                .into_iter()
                .map(|(k, v)| (k.to_owned(), toml::Value::Integer(v)))
                .collect();
                toml::Value::Table(table)
            })
            .collect();
        groups.sort_by_key(|v| std::cmp::Reverse(v["candidate_triangles"].as_integer().unwrap()));
        let mut report = toml::Table::new();
        report.insert("scope".into(), toml::Value::String("Actual retained coastal producer/serialized GPU matrices and primary plan on isolated software Vulkan. No raster/AS pass. CPU update includes extraction/serialization/culling/uploads, not GPU frame time.".into()));
        report.insert(
            "adapter".into(),
            toml::Value::String(format!("{:?}", backend.adapter.get_info())),
        );
        for (key, value) in [
            ("width", 1280),
            ("height", 720),
            ("eye_agl_m", 2),
            ("survey_distance_m", 80),
            ("candidate_instances", i64::from(stats.candidate_instances)),
            ("visible_instances", i64::from(stats.instances)),
            ("candidate_triangles", stats.candidate_triangles as i64),
            ("submitted_triangles", stats.triangles as i64),
            ("draw_calls", stats.draw_calls as i64),
            ("draw_commands", stats.draw_commands as i64),
            ("shadow_passes", terrain.last_shadow_passes() as i64),
            (
                "shadow_candidate_triangles",
                shadow_stats.mesh_candidate_triangles as i64,
            ),
            (
                "shadow_submitted_triangles",
                shadow_stats.mesh_submitted_triangles as i64,
            ),
            (
                "shadow_previous_triangles",
                previous_stats.mesh_submitted_triangles as i64,
            ),
            (
                "shadow_previous_draw_calls",
                previous_stats.mesh_draw_calls as i64,
            ),
            ("shadow_draw_calls", shadow_stats.mesh_draw_calls as i64),
            (
                "shadow_draw_commands",
                shadow_stats.mesh_draw_commands as i64,
            ),
            (
                "shadow_bounds_cpu_storage_bytes",
                meshes
                    .opaque()
                    .map_or(0, MeshRenderer::shadow_bounds_cpu_storage_bytes)
                    as i64,
            ),
            ("shadow_indirect", i64::from(shadow_stats.mesh_indirect)),
            (
                "shadow_indirect_upload_bytes",
                shadow_stats.mesh_indirect_upload_bytes as i64,
            ),
            (
                "shadow_indirect_storage_bytes",
                meshes
                    .opaque()
                    .map_or(0, MeshRenderer::shadow_indirect_storage_bytes) as i64,
            ),
            (
                "shadow_lod_instances",
                shadow_stats.mesh_lod_instances as i64,
            ),
            (
                "shadow_lod_triangles_saved",
                shadow_stats.mesh_lod_triangles_saved as i64,
            ),
            (
                "shadow_lod_max_error_microtexels",
                shadow_stats.mesh_lod_max_error_microtexels as i64,
            ),
            (
                "shadow_prototype_bytes",
                scene
                    .surface_plants_diagnostics
                    .borrow()
                    .shadow_prototype_bytes as i64,
            ),
            (
                "prototype_bytes",
                scene.surface_plants_diagnostics.borrow().prototype_bytes as i64,
            ),
            ("frame_upload_bytes", stats.frame_upload_bytes as i64),
        ] {
            report.insert(key.into(), toml::Value::Integer(value));
        }
        for (key, value) in [
            ("native_update_cpu_p50_ms", samples[15]),
            ("native_update_cpu_p95_ms", samples[28]),
            ("native_update_cpu_max_ms", samples[29]),
        ] {
            report.insert(key.into(), toml::Value::Float(value));
        }
        report.insert(
            "shadow_encode_cpu_ms".into(),
            toml::Value::Float(shadow_encode_ms),
        );
        report.insert("candidate_groups".into(), toml::Value::Array(groups));
        report.insert("shadow_layers".into(), toml::Value::Array(layers));
        report.insert("shadow_groups".into(), toml::Value::Array(caster_groups));
        assert!(stats.triangles > 0 && stats.triangles < stats.candidate_triangles);
        assert!(shadow_stats.mesh_lod_instances > 0);
        assert!(shadow_stats.mesh_submitted_triangles < shadow_stats.mesh_candidate_triangles / 4);
        assert!(shadow_stats.mesh_lod_max_error_microtexels <= 500_000);
        assert!(scene.surface_plants_diagnostics.borrow().prototype_bytes <= 48_000_000);
        use std::io::Write;
        file.write_all(toml::to_string_pretty(&report).unwrap().as_bytes())
            .unwrap();
        file.sync_all().unwrap();
        eprintln!("{}", toml::to_string_pretty(&report).unwrap());
    }
}
