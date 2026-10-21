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

pub const WINDOW_LEVEL: u8 = 14;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Context {
    Disabled,
    Withheld(&'static str),
    Survey(TileKey),
}

#[derive(Clone, Debug, Default)]
pub struct Diagnostics {
    pub reason: &'static str,
    pub window: Option<TileKey>,
    pub candidates: u64,
    pub samples: u64,
    pub accepted: usize,
    pub retained: usize,
    pub preparation_ms: f64,
    pub prototype_bytes: usize,
    pub instance_buffer_bytes: usize,
    pub submitted_instances: u32,
    pub draw_calls: usize,
    pub frame_upload_bytes: usize,
    pub rt_policy: &'static str,
}

impl Diagnostics {
    pub fn metadata(&self) -> BTreeMap<String, String> {
        [
            ("native_surface_plants_status", self.reason.to_string()),
            ("native_surface_plants_window", self.window.map_or_else(|| "none".into(), |w| format!("{},{},{},{}", w.face, w.level, w.x, w.y))),
            ("native_surface_plants_candidate_visits", self.candidates.to_string()),
            ("native_surface_plants_surface_samples", self.samples.to_string()),
            ("native_surface_plants_accepted", self.accepted.to_string()),
            ("native_surface_plants_retained", self.retained.to_string()),
            ("native_surface_plants_preparation_cpu_ms", self.preparation_ms.to_string()),
            ("native_surface_plants_prototype_payload_bytes", self.prototype_bytes.to_string()),
            ("native_surface_plants_instance_buffer_bytes", self.instance_buffer_bytes.to_string()),
            ("native_surface_plants_submitted_instances", self.submitted_instances.to_string()),
            ("native_surface_plants_draw_calls", self.draw_calls.to_string()),
            ("native_surface_plants_frame_upload_bytes", self.frame_upload_bytes.to_string()),
            ("native_surface_plants_rt_policy", if self.rt_policy.is_empty() { "no_plant_rt" } else { self.rt_policy }.to_string()),
            ("native_surface_plants_contract", "WORLD04 fixed L14 window / fixed64 v1; canonical 32 m ground; world-local IDs; no collision; shared opaque/shadow/environment path".into()),
        ].into_iter().map(|(k, v)| (k.into(), v)).collect()
    }
}

pub struct Prepared {
    pub asset: MeshAsset,
    /// Grouped by prototype kind, with exact world-local identity/height metadata.
    pub descriptors: Vec<ScatterItem>,
    pub instances: Vec<MeshInstance>,
    pub ranges: Vec<Range<u32>>,
}

impl Prepared {
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
    let n = 1_u32 << WINDOW_LEVEL;
    let cell = |v| (((v + 1.0) * 0.5 * f64::from(n)) as u32).min(n - 1);
    TileKey {
        face,
        level: WINDOW_LEVEL,
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
) -> Result<(Option<Prepared>, Diagnostics), String> {
    let window = match context {
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
        Context::Survey(window) => window,
    };
    let started = std::time::Instant::now();
    let max_items = usize::try_from(settings.surface_plants_max_items)
        .map_err(|_| "surface plant item budget does not fit usize")?;
    let batch = scatter_region(
        field.params.seed,
        field.params.radius_m,
        window,
        ScatterPopulation::Plants,
        ScatterPolicy {
            density_scale: settings.surface_plants_density_scale,
            max_candidates: settings.surface_plants_max_candidates,
            max_items,
            ..Default::default()
        },
        |dir| canonical_plant_surface(field, dir),
    )
    .map_err(|e| e.to_string())?;
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
    if batch.items.is_empty() {
        diagnostics.preparation_ms = started.elapsed().as_secs_f64() * 1000.0;
        return Ok((None, diagnostics));
    }
    let prepared = convert(batch.items, field.params.radius_m)?;
    diagnostics.prototype_bytes = prepared
        .asset
        .primitives
        .iter()
        .map(|p| {
            p.vertices.len() * std::mem::size_of::<Vertex>()
                + p.indices.len() * std::mem::size_of::<u32>()
        })
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

fn convert(items: Vec<ScatterItem>, radius_m: f64) -> Result<Prepared, String> {
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
    };
    for prototype in thessa_worldgen_rocky::offline::plant_prototypes() {
        let start = u32::try_from(prepared.instances.len()).map_err(|_| "plant range overflow")?;
        for item in items.iter().filter(|i| i.kind == prototype.kind) {
            prepared.instances.push(instance(item, radius_m));
            prepared.descriptors.push(*item);
        }
        let end = u32::try_from(prepared.instances.len()).map_err(|_| "plant range overflow")?;
        if start == end {
            continue;
        }
        // Match the existing terrain's continuous IEC sRGB -> linear convention
        // (lod::linear), without quantizing the producer's f32 albedo to bytes.
        let linear = |v: f32| {
            if v <= 0.04045 {
                v / 12.92
            } else {
                ((v + 0.055) / 1.055).powf(2.4)
            }
        };
        let [r, g, b] = prototype.albedo_srgb.map(linear);
        let material = prepared.asset.materials.len();
        prepared.asset.materials.push(Material {
            base_color: [r, g, b, 1.0],
            metallic: 0.0,
            roughness: 0.92,
            // Fronds/crowns are triangle sheets, not closed solid leaves.
            double_sided: true,
            ..Default::default()
        });
        if prototype.positions.len() != prototype.normals.len() {
            return Err("plant prototype position/normal mismatch".into());
        }
        prepared.asset.primitives.push(Primitive {
            vertices: prototype
                .positions
                .into_iter()
                .zip(prototype.normals)
                .map(|(position, normal)| Vertex {
                    position,
                    normal,
                    uv: [0.0; 2],
                })
                .collect(),
            indices: prototype.indices,
            material,
        });
        prepared.ranges.push(start..end);
    }
    if prepared.descriptors.len() != items.len() {
        return Err(
            "plant descriptor has no unique prototype; refusing partial representation".into(),
        );
    }
    prepared.asset.validate().map_err(str::to_owned)?;
    Ok(prepared)
}

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
            assert_eq!(stats.preparation_ms, 0.0);
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
            assert_eq!(stats.candidates, 4096);
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
            let mut end = 0;
            let mut prototype_payload = 0;
            for (range, primitive) in prepared.ranges.iter().zip(&prepared.asset.primitives) {
                assert_eq!(range.start, end);
                assert!(range.end > range.start);
                let group = &prepared.descriptors[range.start as usize..range.end as usize];
                assert!(group.iter().all(|d| d.kind == group[0].kind));
                assert!(group.windows(2).all(|w| w[0].id < w[1].id));
                let prototype = thessa_worldgen_rocky::offline::plant_prototypes()
                    .into_iter()
                    .find(|p| p.kind == group[0].kind)
                    .unwrap();
                assert_eq!(
                    primitive
                        .vertices
                        .iter()
                        .map(|v| v.position)
                        .collect::<Vec<_>>(),
                    prototype.positions
                );
                assert_eq!(primitive.indices, prototype.indices);
                let material = &prepared.asset.materials[primitive.material];
                assert_eq!(material.metallic, 0.0);
                let linear = |v: f32| {
                    if v <= 0.04045 {
                        v / 12.92
                    } else {
                        ((v + 0.055) / 1.055).powf(2.4)
                    }
                };
                for (actual, expected) in material.base_color[..3]
                    .iter()
                    .zip(prototype.albedo_srgb.map(linear))
                {
                    assert!((actual - expected).abs() < 1e-7);
                }
                end = range.end;
                prototype_payload += primitive.vertices.len() * std::mem::size_of::<Vertex>()
                    + primitive.indices.len() * std::mem::size_of::<u32>();
            }
            assert_eq!(end as usize, prepared.instances.len());
            // Exact producer payload actually published, not a stale pin.
            assert_eq!(stats.prototype_bytes, prototype_payload);
            assert!(prototype_payload > 0);
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
        let prototypes = thessa_worldgen_rocky::offline::plant_prototypes();
        for prototype in prototypes {
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
                prepared.asset.primitives[0].vertices.len(),
                prototype.positions.len()
            );
            assert!(prototype.positions.iter().all(|p| p[1] >= 0.0));
            // The producer's fern fronds now meet the local ground plane
            // (they previously began at 0.1 m); the adapter still adds no offset.
            let min_y = prototype
                .positions
                .iter()
                .map(|p| p[1])
                .fold(f32::INFINITY, f32::min);
            assert!(
                (0.0..=0.02).contains(&min_y),
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
        let prepared = convert(items, 3_200_000.0).unwrap();
        assert!(!prepared.rt_budget_fits(0));
        assert!(!prepared.rt_budget_fits(256));
        assert!(!prepared.rt_budget_fits(299));
        assert!(prepared.rt_budget_fits(300));
        assert!(prepared.rt_budget_fits(4096));
        let mut settings = settings(thessa_graphics::Quality::High);
        settings.ray_tracing = thessa_graphics::ResolvedRayTracing::Local;
        settings.rt_vehicles = true;
        settings.rt_max_instances = 256;
        assert!(!prepared.rt_admitted(&settings));
        settings.rt_max_instances = 300;
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
        assert_eq!(prepared.ranges, vec![0..150, 150..300]);
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
        settings.surface_plants_max_candidates = 4096;
        settings.surface_plants_max_items = 0;
        assert!(
            prepare(context, &settings, scene.stream.field())
                .err()
                .unwrap()
                .contains("ItemBudgetExceeded")
        );
        settings.surface_plants_max_items = 4096;
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
