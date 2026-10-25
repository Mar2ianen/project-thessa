//! Bounded WORLD04 survey adapter. No terrain streamer, physics, or new pass.
use glam::{DMat3, DQuat, DVec3};
use std::{collections::BTreeMap, ops::Range};
use thessa_graphics::ResolvedGraphicsSettings;
use thessa_render_core::mesh::{Material, MeshAsset, MeshInstance, Primitive, Vertex};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::TileKey,
    scatter::{
        ScatterItem, ScatterPolicy, ScatterPopulation, canonical_plant_surface, scatter_region,
    },
    sphere::enu_basis,
};

mod ground;

pub const WINDOW_LEVEL: u8 = 14;
pub const UNDERSTORY_WINDOW_LEVEL: u8 = 18;
/// 7x7 L18 windows cover a ~270 m square around the near-eye cell.
pub const UNDERSTORY_WINDOW_RADIUS: i64 = 3;
/// Near-ground clumps are only meaningful with the eye close to the surface.
pub const UNDERSTORY_MAX_EYE_AGL_M: f64 = 4.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Context {
    Disabled,
    Withheld(&'static str),
    Survey(TileKey),
    SurveyWithUnderstory {
        canopy: TileKey,
        understory: TileKey,
    },
}

pub type Preparation = Result<(Option<Prepared>, Diagnostics), String>;

/// One shared-executor job; camera changes coalesce to the next desired window.
#[derive(Default)]
pub struct PreparationQueue {
    pending: Option<(Context, thessa_terrain_stream::JobHandle<Preparation>)>,
    pub started: u64,
    pub discarded: u64,
}

impl PreparationQueue {
    pub fn pending(&self) -> Option<Context> {
        self.pending.as_ref().map(|(context, _)| *context)
    }

    pub fn request(
        &mut self,
        context: Context,
        settings: &ResolvedGraphicsSettings,
        field: &std::sync::Arc<PlanetField>,
    ) {
        if self.pending.is_none() {
            let settings = settings.clone();
            let field = field.clone();
            self.pending = Some((
                context,
                thessa_terrain_stream::JobHandle::spawn(move || {
                    prepare(context, &settings, &field)
                }),
            ));
            self.started += 1;
        }
    }

    pub fn poll(&mut self, desired: Context) -> Option<Preparation> {
        let (context, job) = self.pending.as_ref()?;
        let result = match job.try_take() {
            Ok(result) => result,
            Err(std::sync::mpsc::TryRecvError::Empty) => return None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                Err("surface plant worker disconnected".into())
            }
        };
        let matches = *context == desired;
        self.pending = None;
        if matches {
            Some(result)
        } else {
            self.discarded += 1;
            None
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Diagnostics {
    pub reason: &'static str,
    pub window: Option<TileKey>,
    pub candidates: u64,
    pub samples: u64,
    pub accepted: usize,
    pub retained: usize,
    pub reduced_instances: usize,
    pub preparation_ms: f64,
    pub prototype_bytes: usize,
    pub prototype_texture_bytes: usize,
    pub instance_buffer_bytes: usize,
    pub submitted_instances: u32,
    pub draw_calls: usize,
    pub frame_upload_bytes: usize,
    pub rt_policy: &'static str,
    pub ground_window: Option<TileKey>,
    pub ground_pebbles: usize,
    pub ground_candidates: u64,
    pub ground_samples: u64,
    pub understory_window: Option<TileKey>,
    pub understory_windows: usize,
    pub understory_candidates: u64,
    pub understory_samples: u64,
    pub understory_accepted: usize,
    pub replaced_coarse_clumps: usize,
    pub preparation_job_pending: bool,
    pub preparation_jobs_started: u64,
    pub preparation_jobs_discarded: u64,
    pub publication_ms: f64,
    pub renderer_reused: bool,
}

impl Diagnostics {
    pub fn metadata(&self) -> BTreeMap<String, String> {
        [
            ("native_surface_ground_pebbles", self.ground_pebbles.to_string()),
            ("native_surface_ground_samples", self.ground_samples.to_string()),
            ("native_surface_ground_window", self.ground_window.map_or_else(|| "none".into(), |w| format!("{},{},{},{}",w.face,w.level,w.x,w.y))),
            ("native_surface_ground_candidates", self.ground_candidates.to_string()),
            ("native_surface_plants_status", self.reason.to_string()),
            ("native_surface_plants_window", self.window.map_or_else(|| "none".into(), |w| format!("{},{},{},{}", w.face, w.level, w.x, w.y))),
            ("native_surface_plants_candidate_visits", self.candidates.to_string()),
            ("native_surface_plants_surface_samples", self.samples.to_string()),
            ("native_surface_plants_accepted", self.accepted.to_string()),
            ("native_surface_plants_retained", self.retained.to_string()),
            ("native_surface_plants_reduced_instances", self.reduced_instances.to_string()),
            ("native_surface_plants_preparation_cpu_ms", self.preparation_ms.to_string()),
            ("native_surface_plants_prototype_payload_bytes", self.prototype_bytes.to_string()),
            ("native_surface_plants_prototype_texture_bytes", self.prototype_texture_bytes.to_string()),
            ("native_surface_plants_instance_buffer_bytes", self.instance_buffer_bytes.to_string()),
            ("native_surface_plants_submitted_instances", self.submitted_instances.to_string()),
            ("native_surface_plants_draw_calls", self.draw_calls.to_string()),
            ("native_surface_plants_frame_upload_bytes", self.frame_upload_bytes.to_string()),
            ("native_surface_plants_rt_policy", if self.rt_policy.is_empty() { "no_plant_rt" } else { self.rt_policy }.to_string()),
            ("native_surface_understory_window", self.understory_window.map_or_else(|| "none".into(),|w|format!("{},{},{},{}",w.face,w.level,w.x,w.y))),
            ("native_surface_understory_windows", self.understory_windows.to_string()),
            ("native_surface_understory_candidate_visits", self.understory_candidates.to_string()),
            ("native_surface_understory_surface_samples", self.understory_samples.to_string()),
            ("native_surface_understory_accepted", self.understory_accepted.to_string()),
            ("native_surface_understory_replaced_coarse_clumps", self.replaced_coarse_clumps.to_string()),
            ("native_surface_plants_preparation_job_pending", self.preparation_job_pending.to_string()),
            ("native_surface_plants_preparation_jobs_started", self.preparation_jobs_started.to_string()),
            ("native_surface_plants_preparation_jobs_discarded", self.preparation_jobs_discarded.to_string()),
            ("native_surface_plants_publication_cpu_ms", self.publication_ms.to_string()),
            ("native_surface_plants_renderer_reused", self.renderer_reused.to_string()),
            ("native_surface_plants_contract", if self.understory_window.is_some() {
                "WORLD11 <=9 same-face L14 large-plant windows plus <=49 same-face L18 near-eye windows; scale-aware full/reduced leaf geometry / disjoint L24 ground-clump IDs with world-addressed 8 m colony occupancy; canonical 32 m descriptors / optional shared optical soil pose; no collision; shared opaque/shadow/environment path"
            } else {"WORLD11 <=9 same-face L14 windows / fixed64 v1; scale-aware full/reduced leaf geometry; canonical 32 m descriptors / optional shared optical soil pose; world-local IDs; no collision; shared opaque/shadow/environment path"}.into()),
        ].into_iter().map(|(k, v)| (k.into(), v)).collect()
    }
}

pub struct Prepared {
    pub asset: MeshAsset,
    /// Grouped by prototype kind, with exact world-local identity/height metadata.
    pub descriptors: Vec<ScatterItem>,
    pub instances: Vec<MeshInstance>,
    pub ranges: Vec<Range<u32>>,
    reduced_primitives: Vec<bool>,
}

impl Prepared {
    /// The producer is fixed for this executable; represented kinds identify
    /// its exact immutable geometry/material layout, independent of instances.
    pub fn same_prototypes(&self, other: &Self) -> bool {
        self.reduced_primitives == other.reduced_primitives
            && self.ranges.len() == other.ranges.len()
            && self.ranges.iter().zip(&other.ranges).all(|(a, b)| {
                self.descriptors.get(a.start as usize).is_some_and(|item| {
                    other
                        .descriptors
                        .get(b.start as usize)
                        .is_some_and(|other| {
                            item.kind == other.kind
                                && (item.kind
                                    != thessa_worldgen_rocky::scatter::ScatterKind::Pebble
                                    || ground::variant(item.id) == ground::variant(other.id))
                                && thessa_worldgen_rocky::offline::plant_form(item.id, item.kind)
                                    == thessa_worldgen_rocky::offline::plant_form(
                                        other.id, other.kind,
                                    )
                        })
                })
            })
    }

    pub fn rt_admitted(&self, settings: &ResolvedGraphicsSettings) -> bool {
        !settings.ray_tracing.is_active()
            || !settings.rt_vehicles
            || self.rt_budget_fits(settings.rt_max_instances)
    }

    /// TLAS demand is primitive × selected range length, not raster capacity.
    /// Check before renderer construction/update: the existing mesh updater
    /// publishes raster state before its optional RT update can reject a batch.
    pub fn rt_budget_fits(&self, max_instances: u32) -> bool {
        self.ranges
            .iter()
            .map(|r| u64::from(r.end - r.start))
            .sum::<u64>()
            <= u64::from(max_instances)
    }
}

/// Same dominant-axis cube chart as lod::TileKey::direction and material_address.
/// Ties choose X, then Y, then Z; clamping owns the outer face edge once.
pub fn survey_window(dir: DVec3) -> TileKey {
    window_at_level(dir, WINDOW_LEVEL)
}

pub fn understory_window(dir: DVec3) -> TileKey {
    window_at_level(dir, UNDERSTORY_WINDOW_LEVEL)
}

fn window_at_level(dir: DVec3, level: u8) -> TileKey {
    let a = dir.abs();
    let (face, u, v) = if a.x >= a.y && a.x >= a.z {
        if dir.x >= 0.0 {
            (0, -dir.z / a.x, dir.y / a.x)
        } else {
            (1, dir.z / a.x, dir.y / a.x)
        }
    } else if a.y >= a.z {
        if dir.y >= 0.0 {
            (2, dir.x / a.y, -dir.z / a.y)
        } else {
            (3, dir.x / a.y, dir.z / a.y)
        }
    } else if dir.z >= 0.0 {
        (4, dir.x / a.z, dir.y / a.z)
    } else {
        (5, -dir.x / a.z, dir.y / a.z)
    };
    let n = 1_u32 << level;
    let cell = |v| (((v + 1.0) * 0.5 * f64::from(n)) as u32).min(n - 1);
    TileKey {
        face,
        level,
        x: cell(u),
        y: cell(v),
    }
}

fn instance(item: &ScatterItem, radius_m: f64) -> MeshInstance {
    let (east, north, up) = enu_basis(item.direction);
    let basis = DMat3::from_cols(east.into(), up.into(), -DVec3::from(north));
    MeshInstance {
        position_m: DVec3::from(item.direction) * (radius_m + item.ground_height_m),
        rotation: (DQuat::from_mat3(&basis) * DQuat::from_rotation_y(item.yaw_rad)).normalize(),
        scale: item.scale,
    }
}

pub fn prepare(
    context: Context,
    settings: &ResolvedGraphicsSettings,
    field: &PlanetField,
) -> Preparation {
    let (window, near_window) = match context {
        Context::Disabled => {
            return Ok((
                None,
                Diagnostics {
                    reason: "disabled",
                    ..Default::default()
                },
            ));
        }
        Context::Withheld(reason) => {
            return Ok((
                None,
                Diagnostics {
                    reason,
                    ..Default::default()
                },
            ));
        }
        Context::Survey(window) => (window, None),
        Context::SurveyWithUnderstory { canopy, understory } => (
            canopy,
            (settings.surface_understory_enabled || settings.terrain_ground_pebbles_max_items > 0)
                .then_some(understory),
        ),
    };
    // A high/unspecified eye AGL deliberately falls back to the coarse survey;
    // it is not an error and must not retain a stale near layer.
    let started = std::time::Instant::now();
    let max_items = usize::try_from(settings.surface_plants_max_items)
        .map_err(|_| "surface plant item budget does not fit usize")?;
    if window.level != WINDOW_LEVEL
        || window.face >= 6
        || window.x >= 1 << window.level
        || window.y >= 1 << window.level
    {
        return Err("invalid canopy window".into());
    }
    if let Some(near) = near_window
        && (near.level != UNDERSTORY_WINDOW_LEVEL
            || near.face >= 6
            || near.x >= 1 << near.level
            || near.y >= 1 << near.level)
    {
        return Err("invalid near-ground window".into());
    }
    let mut batch = thessa_worldgen_rocky::scatter::ScatterBatch {
        items: Vec::new(),
        candidates_visited: 0,
        surface_samples: 0,
    };
    for region in neighboring_windows(window, 1)
        .into_iter()
        .filter(|_| settings.surface_plants_enabled)
    {
        let part = scatter_region(
            field.params.seed,
            field.params.radius_m,
            region,
            ScatterPopulation::Plants,
            ScatterPolicy {
                density_scale: settings.surface_plants_density_scale,
                max_candidates: settings
                    .surface_plants_max_candidates
                    .checked_sub(batch.candidates_visited)
                    .ok_or("canopy candidate budget exhausted")?,
                max_items: max_items
                    .checked_sub(batch.items.len())
                    .ok_or("canopy item budget exhausted")?,
                ..Default::default()
            },
            |dir| canonical_plant_surface(field, dir),
        )
        .map_err(|e| e.to_string())?;
        batch.candidates_visited += part.candidates_visited;
        batch.surface_samples += part.surface_samples;
        batch
            .items
            .try_reserve(part.items.len())
            .map_err(|_| "canopy descriptor allocation failed")?;
        batch.items.extend(part.items);
    }
    batch.items.sort_unstable_by_key(|item| item.id);
    let mut diagnostics = Diagnostics {
        reason: if batch.items.is_empty() {
            "no_eligible_plants"
        } else {
            "applied_survey_only"
        },
        window: Some(window),
        candidates: batch.candidates_visited,
        samples: batch.surface_samples,
        accepted: batch.items.len(),
        retained: batch.items.len(),
        ..Default::default()
    };
    if let Some(near) = near_window.filter(|_| settings.surface_understory_enabled) {
        diagnostics.understory_window = Some(near);
        let windows = neighboring_windows(near, UNDERSTORY_WINDOW_RADIUS);
        // Replace coarse clumps only inside the fine cover; keep all trees and
        // the coarse far fallback. Default-off preserves every original ID.
        let coarse_count = batch.items.len();
        batch.items.retain(|item| {
            if item.kind == thessa_worldgen_rocky::scatter::ScatterKind::LycopsidTree {
                return true;
            }
            let cell = item.id.cell().expect("shared scatter generated a valid ID");
            let shift = cell.level - near.level;
            !windows.contains(&TileKey {
                face: cell.face,
                level: near.level,
                x: cell.x >> shift,
                y: cell.y >> shift,
            })
        });
        diagnostics.replaced_coarse_clumps = coarse_count - batch.items.len();
        diagnostics.understory_windows = windows.len();
        for region in windows {
            let remaining_candidates = settings
                .surface_understory_max_candidates
                .checked_sub(diagnostics.understory_candidates)
                .ok_or("understory candidate budget exhausted")?;
            let remaining_items = usize::try_from(settings.surface_understory_max_items)
                .map_err(|_| "understory item budget does not fit usize")?
                .checked_sub(diagnostics.understory_accepted)
                .ok_or("understory item budget exhausted")?;
            let near_batch = scatter_region(
                field.params.seed,
                field.params.radius_m,
                region,
                ScatterPopulation::Understory,
                ScatterPolicy {
                    density_scale: settings.surface_plants_density_scale,
                    max_candidates: remaining_candidates,
                    max_items: remaining_items,
                    ..Default::default()
                },
                |dir| canonical_plant_surface(field, dir),
            )
            .map_err(|e| e.to_string())?;
            diagnostics.understory_candidates += near_batch.candidates_visited;
            diagnostics.understory_samples += near_batch.surface_samples;
            diagnostics.understory_accepted += near_batch.items.len();
            batch
                .items
                .try_reserve(near_batch.items.len())
                .map_err(|_| "understory descriptor allocation failed")?;
            batch.items.extend(near_batch.items);
        }
        diagnostics.candidates += diagnostics.understory_candidates;
        diagnostics.samples += diagnostics.understory_samples;
        diagnostics.accepted += diagnostics.understory_accepted;
        diagnostics.retained = batch.items.len();
        diagnostics.reason = if batch.items.is_empty() {
            "no_eligible_plants"
        } else {
            "applied_survey_with_near_understory"
        };
        batch.items.sort_unstable_by_key(|item| item.id);
    }
    if let Some(near) = near_window.filter(|_| settings.terrain_ground_pebbles_max_items > 0) {
        diagnostics.ground_window = Some(near);
        for region in neighboring_windows(near, 1) {
            let part = scatter_region(
                field.params.seed,
                field.params.radius_m,
                region,
                ScatterPopulation::GroundPebbles,
                ScatterPolicy {
                    density_scale: settings.terrain_ground_pebbles_density_scale,
                    max_candidates: settings
                        .terrain_ground_pebbles_max_candidates
                        .checked_sub(diagnostics.ground_candidates)
                        .ok_or("ground candidate budget exhausted")?,
                    max_items: (settings.terrain_ground_pebbles_max_items as usize)
                        .checked_sub(diagnostics.ground_pebbles)
                        .ok_or("ground item budget exhausted")?,
                    ..Default::default()
                },
                |dir| {
                    let mut surface = canonical_plant_surface(field, dir);
                    surface.ground_height_m = field.height_m(dir, 32.0).max(0.0)
                        + if settings.terrain_ground_relief_max_level > 17 {
                            thessa_worldgen_rocky::lod::ground_relief_height_m(field, dir, 0.012)
                        } else {
                            0.0
                        };
                    surface
                },
            )
            .map_err(|e| e.to_string())?;
            diagnostics.ground_candidates += part.candidates_visited;
            diagnostics.ground_samples += part.surface_samples;
            diagnostics.ground_pebbles += part.items.len();
            batch
                .items
                .try_reserve(part.items.len())
                .map_err(|_| "ground descriptor allocation failed")?;
            batch.items.extend(part.items);
        }
        if diagnostics.ground_pebbles > 0 && diagnostics.retained == 0 {
            diagnostics.reason = "applied_ground_pebbles_only";
        }
    }
    if batch.items.is_empty() {
        diagnostics.preparation_ms = started.elapsed().as_secs_f64() * 1000.0;
        return Ok((None, diagnostics));
    }
    let anchor = near_window.unwrap_or(window);
    let anchor_position = DVec3::from(anchor.direction(0.5, 0.5)) * field.params.radius_m;
    // The context updates at L18 cell boundaries. Expand full-detail coverage
    // by a conservative half-diagonal, so the actual eye never gets coarse leaves.
    let uncertainty_m = if near_window.is_some() {
        2.0_f64.sqrt() * field.params.radius_m / (1_u64 << anchor.level) as f64
    } else {
        0.0
    };
    let mut prepared = convert_with_detail(
        batch.items,
        field.params.radius_m,
        Some((
            anchor_position,
            settings.surface_plants_detail_distance_m,
            uncertainty_m,
        )),
    )?;
    if settings.terrain_ground_relief_max_level > 17 {
        for (instance, item) in prepared.instances.iter_mut().zip(&prepared.descriptors) {
            if item.kind != thessa_worldgen_rocky::scatter::ScatterKind::Pebble {
                instance.position_m += DVec3::from(item.direction)
                    * thessa_worldgen_rocky::lod::ground_relief_height_m(
                        field,
                        item.direction,
                        0.012,
                    );
            }
        }
    }
    diagnostics.reduced_instances = prepared
        .ranges
        .iter()
        .zip(&prepared.reduced_primitives)
        .filter(|(_, reduced)| **reduced)
        .map(|(r, _)| (r.start, r.end))
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .map(|(start, end)| (end - start) as usize)
        .sum();
    diagnostics.prototype_bytes = prepared
        .asset
        .primitives
        .iter()
        .map(|p| {
            p.vertices.len() * std::mem::size_of::<Vertex>()
                + p.indices.len() * std::mem::size_of::<u32>()
                + p.ambient_visibility.len() * std::mem::size_of::<f32>()
        })
        .sum();
    diagnostics.prototype_texture_bytes = prepared
        .asset
        .images
        .iter()
        .map(|image| image.rgba8.len())
        .sum();
    // Existing MeshRenderer's 64-byte rigid instance ABI. Not total GPU memory:
    // material/uniform/RT/shadow allocations are owned by the shared renderer.
    diagnostics.instance_buffer_bytes = prepared.instances.len() * 64;
    diagnostics.rt_policy = if settings.ray_tracing.is_active() && settings.rt_vehicles {
        if prepared.rt_admitted(settings) {
            "shared_policy_with_preflighted_instance_budget"
        } else {
            "withheld_instance_budget_raster_fallback"
        }
    } else {
        "shared_policy_mesh_rt_not_requested"
    };
    diagnostics.preparation_ms = started.elapsed().as_secs_f64() * 1000.0;
    Ok((Some(prepared), diagnostics))
}

fn neighboring_windows(center: TileKey, radius: i64) -> std::collections::BTreeSet<TileKey> {
    let edge = (1_i64 << center.level) - 1;
    (-radius..=radius)
        .flat_map(|dy| {
            (-radius..=radius).map(move |dx| TileKey {
                x: (i64::from(center.x) + dx).clamp(0, edge) as u32,
                y: (i64::from(center.y) + dy).clamp(0, edge) as u32,
                ..center
            })
        })
        .collect()
}

#[cfg(test)]
fn convert(items: Vec<ScatterItem>, radius_m: f64) -> Result<Prepared, String> {
    convert_with_detail(items, radius_m, None)
}

fn convert_with_detail(
    mut items: Vec<ScatterItem>,
    radius_m: f64,
    detail: Option<(DVec3, f64, f64)>,
) -> Result<Prepared, String> {
    if let Some((anchor, _, _)) = detail {
        // Opaque near-first publication reduces overdraw. Organ ranges retain
        // the paired canonical descriptors and rigid instances.
        let distance = |item: &ScatterItem| {
            (DVec3::from(item.direction) * (radius_m + item.ground_height_m) - anchor)
                .length_squared()
        };
        items.sort_by(|a, b| {
            distance(a)
                .total_cmp(&distance(b))
                .then_with(|| a.id.cmp(&b.id))
        });
    }
    let mut prepared = Prepared {
        asset: MeshAsset {
            primitives: vec![],
            materials: vec![],
            textures: vec![],
            images: vec![],
        },
        descriptors: Vec::with_capacity(items.len()),
        instances: Vec::with_capacity(items.len()),
        ranges: vec![],
        reduced_primitives: vec![],
    };
    let reduced = |item: &ScatterItem| {
        item.kind != thessa_worldgen_rocky::scatter::ScatterKind::ReedClump
            && detail.is_some_and(|(anchor, distance, uncertainty)| {
                (DVec3::from(item.direction) * radius_m - anchor).length()
                    > distance
                        * item.scale
                        * f64::from(
                            thessa_worldgen_rocky::offline::plant_form_scale(
                                thessa_worldgen_rocky::offline::plant_form(item.id, item.kind),
                            )
                            .into_iter()
                            .fold(1.0_f32, f32::max),
                        )
                        + uncertainty
            })
    };
    let full = thessa_worldgen_rocky::offline::plant_prototype_forms(false)
        .into_iter()
        .map(|(p, form)| (p, false, form));
    let coarse = detail
        .into_iter()
        .flat_map(|_| thessa_worldgen_rocky::offline::plant_prototype_forms(true))
        .filter(|(p, _)| p.kind != thessa_worldgen_rocky::scatter::ScatterKind::ReedClump)
        .map(|(p, form)| (p, true, form));
    for (prototype, is_reduced, form) in full.chain(coarse) {
        let start = u32::try_from(prepared.instances.len()).map_err(|_| "plant range overflow")?;
        for item in items.iter().filter(|i| {
            i.kind == prototype.kind
                && reduced(i) == is_reduced
                && thessa_worldgen_rocky::offline::plant_form(i.id, i.kind) == form
        }) {
            prepared.instances.push(instance(item, radius_m));
            prepared.descriptors.push(*item);
        }
        let end = u32::try_from(prepared.instances.len()).map_err(|_| "plant range overflow")?;
        if start == end {
            continue;
        }
        let stem_end = prototype
            .stem
            .as_ref()
            .map_or(0, |stem| stem.vertex_count as usize);
        let dry_start = prototype
            .senescent
            .as_ref()
            .map_or(prototype.positions.len(), |s| s.first_vertex as usize);
        let parts = [
            (0..stem_end, true, false),
            (stem_end..dry_start, false, false),
            (dry_start..prototype.positions.len(), false, true),
        ];
        for (vertices, stem, dry) in parts {
            if vertices.is_empty() {
                continue;
            }
            let [base, normal, mr] =
                crate::surface_plant_materials::append(&mut prepared.asset, stem);
            let material = prepared.asset.materials.len();
            let base_color = if dry {
                let color = prototype.senescent.as_ref().unwrap().albedo_srgb;
                let [r, g, b] = color.map(|s| {
                    if s <= 0.04045 {
                        s / 12.92
                    } else {
                        ((s + 0.055) / 1.055).powf(2.4)
                    }
                });
                [r, g, b, 1.0]
            } else if stem {
                [1.0; 4]
            } else {
                match prototype.kind {
                    thessa_worldgen_rocky::scatter::ScatterKind::FernClump => {
                        [0.48, 0.50, 0.56, 1.0]
                    }
                    thessa_worldgen_rocky::scatter::ScatterKind::LycopsidTree => {
                        [0.55, 0.65, 0.60, 1.0]
                    }
                    _ => [0.75, 0.75, 0.75, 1.0],
                }
            };
            let diffuse_transmission = if dry {
                prototype.senescent.as_ref().unwrap().diffuse_transmission
            } else if stem {
                0.0
            } else if prototype.kind == thessa_worldgen_rocky::scatter::ScatterKind::FernClump {
                0.34
            } else {
                0.45
            };
            prepared.asset.materials.push(Material {
                base_color,
                base_texture: (!dry).then_some(base),
                normal_texture: Some(normal),
                metallic_roughness_texture: Some(mr),
                metallic: 0.0,
                roughness: if stem || dry { 0.85 } else { 0.62 },
                double_sided: true,
                // Authored total scattering and partition, not a measured
                // fossil spectrum; R + T never exceeds the source albedo.
                diffuse_transmission,
                ..Default::default()
            });
            prepared.asset.primitives.push(Primitive {
                ambient_visibility: prototype
                    .ambient_visibility
                    .get(vertices.clone())
                    .unwrap_or(&[])
                    .to_vec(),
                vertices: vertices
                    .clone()
                    .map(|index| Vertex {
                        position: prototype.positions[index],
                        normal: prototype.normals[index],
                        uv: prototype.uvs.get(index).copied().unwrap_or([0.0; 2]),
                    })
                    .collect(),
                indices: prototype
                    .indices
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .filter(|triangle| vertices.contains(&(triangle[0] as usize)))
                    .flat_map(|triangle| triangle.map(|i| i - vertices.start as u32))
                    .collect(),
                material,
            });
            prepared.ranges.push(start..end);
            prepared.reduced_primitives.push(is_reduced);
        }
    }
    ground::append(&mut prepared, &items, radius_m)?;
    if prepared.descriptors.len() != items.len() {
        return Err(
            "plant descriptor has no unique prototype; refusing partial representation".into(),
        );
    }
    prepared.asset.validate().map_err(str::to_owned)?;
    Ok(prepared)
}

#[cfg(test)]
mod ray_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use thessa_worldgen_rocky::scatter::ScatterKind;
    const LAND: [f64; 2] = [23.565679258039204, 165.13881581863882];

    fn settings(quality: thessa_graphics::Quality) -> ResolvedGraphicsSettings {
        let mut requested = thessa_graphics::RequestedGraphics::default();
        requested.surface_plants.enabled = true;
        requested.surface_plants.quality = quality;
        ResolvedGraphicsSettings::from_requested(
            &requested,
            &thessa_graphics::Capabilities::unknown(),
        )
    }

    #[test]
    fn preparation_queue_coalesces_without_waiting_and_rejects_stale_results() {
        let scene = crate::scene::Scene::load(Default::default()).unwrap();
        let a = Context::Survey(survey_window(DVec3::X));
        let b = Context::Survey(survey_window(DVec3::Y));
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let mut queue = PreparationQueue {
            pending: Some((a, thessa_terrain_stream::JobHandle::new(rx))),
            started: 1,
            discarded: 0,
        };
        // An unfinished worker never waits or queues extra camera requests.
        for desired in [b, a, b, Context::Disabled] {
            assert!(queue.poll(desired).is_none());
            queue.request(
                desired,
                &settings(thessa_graphics::Quality::High),
                scene.stream.field(),
            );
            assert_eq!(queue.pending(), Some(a));
            assert_eq!(queue.started, 1);
        }
        assert!(tx.send(Ok((None, Diagnostics::default()))).is_ok());
        assert!(queue.poll(b).is_none());
        assert!(queue.pending().is_none());
        assert_eq!(queue.discarded, 1);
        // Current errors propagate; stale errors (including lost workers) do not.
        for (context, desired, disconnected) in
            [(a, a, false), (a, b, false), (a, b, true), (a, a, true)]
        {
            let (tx, rx) = std::sync::mpsc::sync_channel(1);
            queue.pending = Some((context, thessa_terrain_stream::JobHandle::new(rx)));
            if !disconnected {
                assert!(tx.send(Err("explicit budget error".into())).is_ok());
            }
            drop(tx);
            let result = queue.poll(desired);
            assert!(queue.pending().is_none());
            if context == desired {
                assert!(result.unwrap().is_err());
            } else {
                assert!(result.is_none());
            }
        }
    }

    #[test]
    fn disabled_and_withheld_do_no_work_and_keep_zero_resources() {
        let scene = crate::scene::Scene::load(Default::default()).unwrap();
        for context in [
            Context::Disabled,
            Context::Withheld("unsupported_vehicle_view"),
            Context::Withheld("unsupported_map_view"),
        ] {
            let (prepared, stats) = prepare(
                context,
                &settings(thessa_graphics::Quality::High),
                scene.stream.field(),
            )
            .unwrap();
            assert!(prepared.is_none());
            assert_eq!(
                (
                    stats.candidates,
                    stats.samples,
                    stats.accepted,
                    stats.retained
                ),
                (0, 0, 0, 0)
            );
            assert_eq!((stats.prototype_bytes, stats.instance_buffer_bytes), (0, 0));
            assert_eq!(stats.prototype_texture_bytes, 0);
            assert_eq!(stats.preparation_ms, 0.0);
        }
    }

    #[test]
    fn near_understory_is_nested_canonical_bounded_and_reuses_prototype_groups() {
        let scene = crate::scene::Scene::load(Default::default()).unwrap();
        let field = scene.stream.field();
        let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(LAND[0], LAND[1]);
        let canopy = survey_window(dir.into());
        let near = understory_window(dir.into());
        let context = Context::SurveyWithUnderstory {
            canopy,
            understory: near,
        };
        let mut previous = Vec::new();
        for quality in [
            thessa_graphics::Quality::Low,
            thessa_graphics::Quality::Medium,
            thessa_graphics::Quality::High,
        ] {
            let mut requested = thessa_graphics::RequestedGraphics::default();
            requested.surface_plants.enabled = true;
            requested.surface_plants.understory = true;
            requested.surface_plants.quality = quality;
            let resolved = ResolvedGraphicsSettings::from_requested(
                &requested,
                &thessa_graphics::Capabilities::unknown(),
            );
            let (prepared, stats) = prepare(context, &resolved, field).unwrap();
            let prepared = prepared.unwrap();
            assert_eq!(stats.understory_windows, 49);
            assert_eq!(stats.understory_candidates, 200_704);
            assert_eq!(stats.candidates, 237_568);
            assert!(stats.understory_accepted > 0);
            assert_eq!(stats.retained, prepared.instances.len());
            assert_eq!(
                stats.accepted - stats.replaced_coarse_clumps,
                stats.retained
            );
            assert!(
                previous
                    .iter()
                    .all(|item| prepared.descriptors.contains(item))
            );
            let (coarse, _) = prepare(Context::Survey(canopy), &settings(quality), field).unwrap();
            let trees: Vec<_> = coarse
                .unwrap()
                .descriptors
                .into_iter()
                .filter(|i| i.kind == ScatterKind::LycopsidTree)
                .collect();
            for item in &prepared.descriptors {
                if item.kind == ScatterKind::LycopsidTree {
                    assert!(trees.contains(item));
                } else {
                    let cell = item.id.cell().unwrap();
                    assert!(
                        cell.level == thessa_worldgen_rocky::scatter::UNDERSTORY_CANDIDATE_LEVEL
                            || cell.level == thessa_worldgen_rocky::scatter::CANDIDATE_LEVEL
                    );
                    if cell.level == thessa_worldgen_rocky::scatter::CANDIDATE_LEVEL {
                        assert!(
                            cell.face != near.face
                                || (cell.x >> 2).abs_diff(near.x) > UNDERSTORY_WINDOW_RADIUS as u32
                                || (cell.y >> 2).abs_diff(near.y) > UNDERSTORY_WINDOW_RADIUS as u32
                        );
                    }
                }
                assert_eq!(item.ground_height_m, field.height_m(item.direction, 32.0));
            }
            for tree in trees {
                assert!(prepared.descriptors.contains(&tree));
            }
            assert_eq!(prepared.ranges.len(), prepared.asset.primitives.len());
            assert!(prepared.ranges.len() <= 31);
            assert!(stats.prototype_bytes > 0 && stats.prototype_bytes <= 6_000_000);
            let (_, disabled) = prepare(Context::Disabled, &resolved, field).unwrap();
            assert_eq!(
                (
                    disabled.understory_candidates,
                    disabled.understory_samples,
                    disabled.understory_accepted
                ),
                (0, 0, 0)
            );
            assert_eq!(disabled.preparation_ms, 0.0);
            let mut bad = resolved.clone();
            bad.surface_understory_max_candidates = 4095;
            assert!(prepare(context, &bad, field).is_err());
            let (coarse_only, fallback) =
                prepare(Context::Survey(canopy), &resolved, field).unwrap();
            assert!(coarse_only.is_some());
            assert_eq!(fallback.understory_windows, 0);
            assert_eq!(fallback.understory_candidates, 0);
            assert_eq!(fallback.understory_accepted, 0);
            eprintln!(
                "near_understory quality={quality:?} clumps={} trees={} total={} samples={} prep_ms={}",
                stats.understory_accepted,
                prepared
                    .descriptors
                    .iter()
                    .filter(|i| i.kind == ScatterKind::LycopsidTree)
                    .count(),
                stats.retained,
                stats.understory_samples,
                stats.preparation_ms
            );
            previous = prepared.descriptors;
        }
    }

    #[test]
    fn world00_nested_quality_exact_ground_and_grouped_range_ownership() {
        let scene = crate::scene::Scene::load(Default::default()).unwrap();
        let field = scene.stream.field();
        let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(LAND[0], LAND[1]);
        let window = survey_window(dir.into());
        assert_eq!(
            window,
            TileKey {
                face: 1,
                level: 14,
                x: 6018,
                y: 11888
            }
        );
        let mut previous: Vec<ScatterItem> = vec![];
        for quality in [
            thessa_graphics::Quality::Low,
            thessa_graphics::Quality::Medium,
            thessa_graphics::Quality::High,
        ] {
            let (prepared, stats) =
                prepare(Context::Survey(window), &settings(quality), field).unwrap();
            let prepared = prepared.unwrap();
            assert_eq!(stats.candidates, 9 * 4096);
            assert!(stats.samples > 0 && !prepared.descriptors.is_empty());
            assert_eq!(prepared.descriptors.len(), stats.accepted);
            assert!(
                previous.len() < prepared.descriptors.len(),
                "WORLD00 must strictly increase from Low to Medium to High"
            );
            for survivor in previous {
                assert!(prepared.descriptors.contains(&survivor));
            }
            for (item, instance) in prepared.descriptors.iter().zip(&prepared.instances) {
                assert_eq!(item.ground_height_m, field.height_m(item.direction, 32.0));
                assert_eq!(
                    instance.position_m,
                    DVec3::from(item.direction) * (field.params.radius_m + item.ground_height_m)
                );
                assert_eq!(instance.scale, item.scale);
                assert!(
                    (instance.rotation * DVec3::Y - DVec3::from(item.direction)).length() < 1e-14
                );
            }
            assert_eq!(prepared.ranges.len(), prepared.asset.primitives.len());
            let mut prototype_payload = 0;
            let mut seen = std::collections::BTreeMap::new();
            for ((range, primitive), reduced) in prepared
                .ranges
                .iter()
                .zip(&prepared.asset.primitives)
                .zip(&prepared.reduced_primitives)
            {
                assert!(range.end > range.start);
                let group = &prepared.descriptors[range.start as usize..range.end as usize];
                assert!(group.iter().all(|d| d.kind == group[0].kind));
                assert_eq!(
                    group
                        .iter()
                        .map(|item| item.id)
                        .collect::<std::collections::BTreeSet<_>>()
                        .len(),
                    group.len()
                );
                let form = thessa_worldgen_rocky::offline::plant_form(group[0].id, group[0].kind);
                assert!(
                    group
                        .iter()
                        .all(|d| thessa_worldgen_rocky::offline::plant_form(d.id, d.kind) == form)
                );
                let prototype = thessa_worldgen_rocky::offline::plant_prototype_forms(*reduced)
                    .into_iter()
                    .find(|(p, f)| p.kind == group[0].kind && *f == form)
                    .unwrap()
                    .0;
                let offset = *seen
                    .entry((group[0].kind as u8, *reduced, form))
                    .or_insert(0usize);
                let end = offset + primitive.vertices.len();
                let material = &prepared.asset.materials[primitive.material];
                let stem = prototype
                    .stem
                    .as_ref()
                    .is_some_and(|s| offset < s.vertex_count as usize);
                let dry = prototype
                    .senescent
                    .as_ref()
                    .filter(|s| offset >= s.first_vertex as usize);
                let expected_transmission = if let Some(dry) = dry {
                    dry.diffuse_transmission
                } else if stem {
                    0.0
                } else if prototype.kind == thessa_worldgen_rocky::scatter::ScatterKind::FernClump {
                    0.34
                } else {
                    0.45
                };
                assert_eq!(material.diffuse_transmission, expected_transmission);
                assert_eq!(material.metallic, 0.0);
                assert_eq!(material.base_texture.is_some(), dry.is_none());
                assert!(material.normal_texture.is_some());
                assert!(material.metallic_roughness_texture.is_some());
                if !stem && dry.is_none() {
                    let texture = &prepared.asset.textures[material.base_texture.unwrap()];
                    let image = &prepared.asset.images[texture.image];
                    let mut reflectance = [0.0_f64; 3];
                    for pixel in image.rgba8.as_chunks::<4>().0 {
                        for channel in 0..3 {
                            let encoded = f64::from(pixel[channel]) / 255.0;
                            let linear = if encoded <= 0.04045 {
                                encoded / 12.92
                            } else {
                                ((encoded + 0.055) / 1.055).powf(2.4)
                            };
                            reflectance[channel] +=
                                linear * f64::from(material.base_color[channel]);
                        }
                    }
                    assert!(
                        reflectance[1] > reflectance[0] && reflectance[0] > reflectance[2],
                        "green foliage must retain its source's spectral order: {reflectance:?}"
                    );
                }
                for (i, v) in primitive.vertices.iter().enumerate() {
                    assert_eq!(v.position, prototype.positions[offset + i]);
                    assert_eq!(v.normal, prototype.normals[offset + i]);
                    assert_eq!(
                        v.uv,
                        prototype.uvs.get(offset + i).copied().unwrap_or([0.0; 2])
                    );
                }
                let indices: Vec<_> = prototype
                    .indices
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .filter(|t| (offset..end).contains(&(t[0] as usize)))
                    .flat_map(|t| t.map(|i| i - offset as u32))
                    .collect();
                assert_eq!(indices, primitive.indices);
                assert_eq!(
                    primitive.ambient_visibility,
                    prototype.ambient_visibility[offset..end]
                );
                seen.insert((group[0].kind as u8, *reduced, form), end);
                prototype_payload += primitive.vertices.len() * std::mem::size_of::<Vertex>()
                    + primitive.indices.len() * std::mem::size_of::<u32>()
                    + primitive.ambient_visibility.len() * std::mem::size_of::<f32>();
            }
            for ((kind, reduced, form), end) in seen {
                let prototype = thessa_worldgen_rocky::offline::plant_prototype_forms(reduced)
                    .into_iter()
                    .find(|(p, f)| p.kind as u8 == kind && *f == form)
                    .unwrap()
                    .0;
                assert_eq!(end, prototype.positions.len());
            }
            assert_eq!(stats.prototype_bytes, prototype_payload);
            assert_eq!(stats.prototype_texture_bytes, 3_219_456);
            prepared.asset.validate().unwrap();
            eprintln!("WORLD04 {quality:?}: {stats:?}");
            previous = prepared.descriptors;
        }
    }

    #[test]
    fn prototype_orientation_yaw_poles_and_seam_are_right_handed() {
        for dir in [
            DVec3::X,
            DVec3::Y,
            -DVec3::Y,
            DVec3::new(-1.0, 0.0, 1e-12).normalize(),
            DVec3::new(-1.0, 0.0, -1e-12).normalize(),
        ] {
            let (east, north, up) = enu_basis(dir.to_array());
            let mut item = ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(1),
                kind: ScatterKind::FernClump,
                direction: dir.to_array(),
                ground_height_m: 123.0,
                scale: 0.8,
                yaw_rad: 0.0,
            };
            let zero = instance(&item, 3_200_000.0);
            assert!((zero.rotation * DVec3::X - DVec3::from(east)).length() < 1e-14);
            assert!((zero.rotation * DVec3::Y - DVec3::from(up)).length() < 1e-14);
            assert!((zero.rotation * DVec3::Z + DVec3::from(north)).length() < 1e-14);
            item.yaw_rad = std::f64::consts::FRAC_PI_2;
            let quarter = instance(&item, 3_200_000.0);
            assert!((quarter.rotation * DVec3::X - DVec3::from(north)).length() < 1e-14);
            let matrix = DMat3::from_quat(quarter.rotation);
            assert!((matrix.determinant() - 1.0).abs() < 1e-14);
            let window = survey_window(dir);
            assert!(window.face < 6 && window.x < 16384 && window.y < 16384);
        }
        let prototypes = thessa_worldgen_rocky::offline::plant_prototype_forms(false);
        for (prototype, form) in prototypes {
            if form
                != thessa_worldgen_rocky::offline::plant_form(
                    thessa_worldgen_rocky::scatter::ScatterId(1),
                    prototype.kind,
                )
            {
                continue;
            }
            let item = ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(1),
                kind: prototype.kind,
                direction: DVec3::Y.to_array(),
                ground_height_m: 0.0,
                scale: 0.8,
                yaw_rad: 0.0,
            };
            let prepared = convert(vec![item], 3_200_000.0).unwrap();
            assert_eq!(
                prepared
                    .asset
                    .primitives
                    .iter()
                    .map(|p| p.vertices.len())
                    .sum::<usize>(),
                prototype.positions.len()
            );
            assert!(prototype.positions.iter().all(|p| p[1]
                >= if prototype.kind == ScatterKind::LycopsidTree {
                    -0.15
                } else {
                    0.0
                }));
            // The producer's fern fronds now meet the local ground plane
            // (they previously began at 0.1 m); the adapter still adds no offset.
            let min_y = prototype
                .positions
                .iter()
                .map(|p| p[1])
                .fold(f32::INFINITY, f32::min);
            assert!(
                (if prototype.kind == ScatterKind::LycopsidTree {
                    -0.15..=0.02
                } else {
                    0.0..=0.02
                })
                .contains(&min_y),
                "{:?} prototype must sit on its local ground plane, got {min_y}",
                prototype.kind
            );
            let root = prepared.instances[0];
            let radial_gap = prepared.asset.primitives[0]
                .vertices
                .iter()
                .map(|v| {
                    let point = root.position_m
                        + root.rotation * (DVec3::from(glam::Vec3::from(v.position)) * root.scale);
                    (point - root.position_m).dot(DVec3::from(item.direction))
                })
                .fold(f64::INFINITY, f64::min);
            assert!(
                (radial_gap - f64::from(min_y) * root.scale).abs() < 1e-8,
                "intrinsic prototype/root gap only; this is NOT a resident-terrain residual"
            );
        }
    }

    #[test]
    fn opaque_near_first_order_keeps_canonical_descriptors_and_instance_ranges() {
        let radius = 3_200_000.0;
        let items: Vec<_> = [30.0, 1.0, 10.0]
            .into_iter()
            .map(|h| ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(h as u64),
                kind: ScatterKind::ReedClump,
                direction: DVec3::X.to_array(),
                ground_height_m: h,
                scale: 1.0,
                yaw_rad: 0.0,
            })
            .collect();
        let original = convert(items.clone(), radius).unwrap();
        let sorted =
            convert_with_detail(items.clone(), radius, Some((DVec3::X * radius, 48.0, 0.0)))
                .unwrap();
        assert_eq!(sorted.descriptors, [items[1], items[2], items[0]]);
        assert!(original.same_prototypes(&sorted));
        assert_eq!(sorted.ranges, original.ranges);
        for (descriptor, pose) in sorted.descriptors.iter().zip(&sorted.instances) {
            assert_eq!(*pose, instance(descriptor, radius));
        }
        for (a, b) in original
            .asset
            .primitives
            .iter()
            .zip(&sorted.asset.primitives)
        {
            assert_eq!(a.vertices, b.vertices);
            assert_eq!(a.indices, b.indices);
        }
    }

    #[test]
    fn leaf_lod_keeps_identity_scale_and_ground_and_prevents_wrong_source_reuse() {
        let make = |metres: f64, scale| ScatterItem {
            id: thessa_worldgen_rocky::scatter::ScatterId(metres as u64),
            kind: ScatterKind::FernClump,
            direction: DVec3::new(
                (metres / 3_200_000.0).sin(),
                (metres / 3_200_000.0).cos(),
                0.0,
            )
            .to_array(),
            ground_height_m: 10.0,
            scale,
            yaw_rad: 0.4,
        };
        let items = vec![make(20.0, 1.0), make(80.0, 1.0), make(80.0, 2.0)];
        let mixed = convert_with_detail(
            items.clone(),
            3_200_000.0,
            Some((DVec3::Y * 3_200_000.0, 48.0, 0.0)),
        )
        .unwrap();
        assert_eq!(mixed.reduced_primitives, [false, false, true, true]);
        assert_eq!(mixed.descriptors, [items[0], items[2], items[1]]);
        for (d, transform) in mixed.descriptors.iter().zip(&mixed.instances) {
            assert_eq!(*transform, instance(d, 3_200_000.0));
        }
        let full = convert(items, 3_200_000.0).unwrap();
        assert!(!full.same_prototypes(&mixed));
        let only_full = convert(vec![make(80.0, 1.0)], 3_200_000.0).unwrap();
        let only_reduced = convert_with_detail(
            vec![make(80.0, 1.0)],
            3_200_000.0,
            Some((DVec3::Y * 3_200_000.0, 48.0, 0.0)),
        )
        .unwrap();
        assert_eq!(only_full.ranges.len(), only_reduced.ranges.len());
        assert!(!only_full.same_prototypes(&only_reduced));
        assert!(mixed.same_prototypes(&mixed));
        let edge = TileKey {
            face: 0,
            level: WINDOW_LEVEL,
            x: 0,
            y: 0,
        };
        assert_eq!(neighboring_windows(edge, 1).len(), 4);
    }

    #[test]
    fn planetary_roots_are_subtracted_in_f64_before_renderer_narrowing() {
        for lat_lon in [LAND, [90.0, 0.0], [-90.0, 180.0], [0.0, -180.0]] {
            let direction = thessa_worldgen_rocky::sphere::dir_from_latlon(lat_lon[0], lat_lon[1]);
            let item = ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(1),
                kind: ScatterKind::FernClump,
                direction,
                ground_height_m: 4.557292563542587,
                scale: 0.8,
                yaw_rad: 0.73,
            };
            let root = instance(&item, 3_200_000.0);
            for offset in [
                DVec3::new(0.01, -0.02, 0.03),
                DVec3::new(80.125, 50.25, -1800.5),
            ] {
                let camera = root.position_m + offset;
                let matrix = root.camera_relative_matrix(camera).unwrap();
                let actual = matrix.transform_point3(glam::Vec3::ZERO).as_dvec3();
                let expected = root.position_m - camera;
                // Absolute f32 translation envelope at these <=1801 m offsets;
                // this would fail by centimetres if planet coordinates narrowed first.
                assert!((actual - expected).length() < 0.0001);
                for local in [DVec3::Y, DVec3::new(0.4, 9.0, -0.7)] {
                    let actual = matrix.transform_point3(local.as_vec3()).as_dvec3();
                    let expected = expected + root.rotation * (local * root.scale);
                    assert!((actual - expected).length() < 0.0002);
                }
            }
        }
    }

    #[test]
    fn selected_window_round_trips_all_cube_charts() {
        for face in 0..6 {
            for (x, y) in [(0, 0), (16383, 16383), (6018, 11888), (8192, 8192)] {
                let key = TileKey {
                    face,
                    level: WINDOW_LEVEL,
                    x,
                    y,
                };
                assert_eq!(survey_window(key.direction(0.5, 0.5).into()), key);
            }
        }
    }

    #[test]
    fn rt_preflight_counts_grouped_ranges_and_never_truncates_raster() {
        let items: Vec<_> = (0..300)
            .map(|i| ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(i + 1),
                kind: if i % 2 == 0 {
                    ScatterKind::FernClump
                } else {
                    ScatterKind::ReedClump
                },
                direction: DVec3::Y.to_array(),
                ground_height_m: 10.0,
                scale: 1.0,
                yaw_rad: 0.0,
            })
            .collect();
        let smaller = convert(items[..40].to_vec(), 3_200_000.0).unwrap();
        let fern_only = convert(
            items
                .iter()
                .copied()
                .filter(|item| item.kind == ScatterKind::FernClump)
                .collect(),
            3_200_000.0,
        )
        .unwrap();
        let prepared = convert(items, 3_200_000.0).unwrap();
        assert!(prepared.same_prototypes(&smaller));
        assert!(!prepared.same_prototypes(&fern_only));
        assert!(!fern_only.same_prototypes(&prepared));
        assert!(!prepared.rt_budget_fits(0));
        assert!(!prepared.rt_budget_fits(256));
        assert!(!prepared.rt_budget_fits(449));
        assert!(prepared.rt_budget_fits(450));
        assert!(prepared.rt_budget_fits(4096));
        let mut settings = settings(thessa_graphics::Quality::High);
        settings.ray_tracing = thessa_graphics::ResolvedRayTracing::Local;
        settings.rt_vehicles = true;
        settings.rt_max_instances = 256;
        assert!(!prepared.rt_admitted(&settings));
        settings.rt_max_instances = 450;
        assert!(prepared.rt_admitted(&settings));
        settings.rt_max_instances = 0;
        settings.rt_vehicles = false;
        assert!(
            prepared.rt_admitted(&settings),
            "no generic mesh TLAS demand"
        );
        settings.rt_vehicles = true;
        settings.ray_tracing = thessa_graphics::ResolvedRayTracing::Off;
        assert!(prepared.rt_admitted(&settings));
        assert_eq!(prepared.descriptors.len(), 300);
        assert_eq!(prepared.instances.len(), 300);
        let demand: u32 = prepared.ranges.iter().map(|r| r.end - r.start).sum();
        assert_eq!(demand, 450);
        let unique: std::collections::BTreeSet<_> =
            prepared.ranges.iter().map(|r| (r.start, r.end)).collect();
        assert_eq!(unique.iter().map(|(a, b)| b - a).sum::<u32>(), 300);
        assert!(
            prepared
                .asset
                .materials
                .iter()
                .all(|m| m.double_sided && m.metallic == 0.0)
        );
    }

    #[test]
    fn budgets_error_without_partial_batch_and_empty_water_has_no_prototypes() {
        let scene = crate::scene::Scene::load(Default::default()).unwrap();
        let mut settings = settings(thessa_graphics::Quality::High);
        let dir = thessa_worldgen_rocky::sphere::dir_from_latlon(LAND[0], LAND[1]);
        let context = Context::Survey(survey_window(dir.into()));
        settings.surface_plants_max_candidates = 4095;
        assert!(
            prepare(context, &settings, scene.stream.field())
                .err()
                .unwrap()
                .contains("CandidateBudgetExceeded")
        );
        settings.surface_plants_max_candidates = 9 * 4096;
        settings.surface_plants_max_items = 0;
        assert!(
            prepare(context, &settings, scene.stream.field())
                .err()
                .unwrap()
                .contains("ItemBudgetExceeded")
        );
        settings.surface_plants_max_items = 9 * 4096;
        let ocean = thessa_worldgen_rocky::sphere::dir_from_latlon(0.0, 0.0);
        let (prepared, stats) = prepare(
            Context::Survey(survey_window(ocean.into())),
            &settings,
            scene.stream.field(),
        )
        .unwrap();
        assert!(prepared.is_none());
        assert_eq!(stats.prototype_bytes, 0);
    }
}
