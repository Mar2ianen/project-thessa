//! Bounded cover geometry in the existing preparation/opaque/shadow publication.
use super::*;
use thessa_worldgen_rocky::{lod, rng, scatter::ScatterKind, surface_water::fill_closed_basin};

fn blank(material: usize) -> Primitive {
    Primitive {
        vertices: vec![],
        indices: vec![],
        ambient_visibility: vec![],
        material,
    }
}
fn vertex(mesh: &mut Primitive, p: DVec3, n: DVec3, anchor: DVec3) {
    mesh.indices.push(mesh.vertices.len() as u32);
    mesh.vertices.push(Vertex {
        position: (p - anchor).as_vec3().to_array(),
        normal: n.as_vec3().to_array(),
        uv: [0.0; 2],
    });
}
fn publish(prepared: &mut Prepared, mut mesh: Primitive, anchor: DVec3) {
    if mesh.indices.is_empty() {
        return;
    }
    let mut unique = std::collections::BTreeMap::new();
    let mut vertices = Vec::new();
    for index in &mut mesh.indices {
        let v = mesh.vertices[*index as usize];
        let key = (v.position.map(f32::to_bits), v.normal.map(f32::to_bits));
        *index = *unique.entry(key).or_insert_with(|| {
            let i = vertices.len() as u32;
            vertices.push(v);
            i
        });
    }
    mesh.vertices = vertices;
    let start = prepared.instances.len() as u32;
    prepared.instances.push(MeshInstance {
        position_m: anchor,
        rotation: DQuat::IDENTITY,
        scale: 1.0,
    });
    prepared.asset.primitives.push(mesh);
    prepared.ranges.push(start..start + 1);
    prepared.reduced_primitives.push(false);
}

pub(super) fn append(
    prepared: &mut Prepared,
    settings: &ResolvedGraphicsSettings,
    field: &PlanetField,
    window: TileKey,
    stats: &mut Diagnostics,
) -> Result<(), String> {
    if !settings.terrain_ground_detail || settings.surface_quality == thessa_graphics::Quality::Low
    {
        return Ok(());
    }
    let high = settings.surface_quality == thessa_graphics::Quality::High;
    let moss_limit = if high { 384 } else { 128 };
    let pool_limit = if high { 24 } else { 8 };
    let anchor = DVec3::from(window.direction(0.5, 0.5)) * field.params.radius_m;
    let mut sites: Vec<_> = prepared
        .descriptors
        .iter()
        .filter(|d| d.kind == ScatterKind::Pebble)
        .copied()
        .collect();
    // Stable world-address order distributes admission over the whole window;
    // nearest-to-cell-centre ordering placed all pools behind some cameras.
    sites.sort_by_cached_key(|item| {
        (
            rng::hash01(field.params.seed, 7291, item.id.0 as i64, 0).to_bits(),
            item.id,
        )
    });
    let start = prepared.asset.primitives.len();
    let moss_material = prepared.asset.materials.len();
    prepared.asset.materials.push(Material {
        base_color: [0.045, 0.058, 0.028, 1.0],
        metallic: 0.0,
        roughness: 0.92,
        double_sided: true,
        ..Default::default()
    });
    let mut moss = blank(moss_material);
    for d in sites.iter().take(2048) {
        if stats.ground_moss_clumps >= moss_limit {
            break;
        }
        let cover = lod::ground_cover_at(field, d.direction, 0.012);
        if cover.moss < 0.18 {
            continue;
        }
        let up = DVec3::from(d.direction);
        let (east, north, _) = enu_basis(d.direction);
        let east = DVec3::from(east);
        let north = DVec3::from(north);
        let hash = |c| rng::hash01(field.params.seed, c, d.id.0 as i64, 0);
        // Shoot lengths and cushion radius are authored optical morphology.
        for shoot in 0..8 {
            let angle = (hash(7000 + shoot) * std::f64::consts::TAU) + shoot as f64 * 2.4;
            let radial = east * angle.cos() + north * angle.sin();
            let offset = radial * (0.02 + hash(7040 + shoot) * 0.12);
            let dir = (up * field.params.radius_m + offset).normalize();
            let height = field.height_m(dir.to_array(), 32.0).max(0.0)
                + lod::ground_relief_height_m(field, dir.to_array(), 0.012);
            let base = dir * (field.params.radius_m + height - 0.002);
            let length = 0.018 + hash(7080 + shoot) * 0.035;
            let half = radial * 0.007;
            let tip = base + up * length + radial * 0.008;
            for p in [
                base - half,
                tip - half * 0.15,
                base + half,
                base + half,
                tip - half * 0.15,
                tip + half * 0.15,
            ] {
                vertex(&mut moss, p, up.cross(radial).normalize(), anchor);
            }
        }
        stats.ground_moss_clumps += 1;
    }
    publish(prepared, moss, anchor);
    if settings.terrain_ground_puddles {
        let material = prepared.asset.materials.len();
        let f0 = ((1.333_f32 - 1.0) / (1.333 + 1.0)).powi(2);
        prepared.asset.materials.push(Material {
            base_color: [0.025, 0.022, 0.016, 1.0],
            metallic: 0.0,
            roughness: 0.075,
            dielectric_f0: f0,
            ..Default::default()
        });
        let mut water = blank(material);
        let mut centers: Vec<DVec3> = vec![];
        let side = 17;
        let radius = 1.25;
        let spacing = radius * 2.0 / (side - 1) as f64;
        for d in sites
            .iter()
            .filter(|d| rng::hash01(field.params.seed, 7200, d.id.0 as i64, 0) < 0.10)
            .take(256)
        {
            if stats.ground_puddles >= pool_limit {
                break;
            }
            let up = DVec3::from(d.direction);
            let center = up * field.params.radius_m;
            if centers
                .iter()
                .any(|p| p.distance_squared(center) < (radius * 2.0_f64).powi(2))
            {
                continue;
            }
            let cover = lod::ground_cover_at(field, d.direction, 0.012);
            if cover.wetness < 0.25 {
                continue;
            }
            let (east, north, _) = enu_basis(d.direction);
            let east = DVec3::from(east);
            let north = DVec3::from(north);
            let base_height = field.height_m(d.direction, 32.0).max(0.0);
            let mut heights = Vec::with_capacity(side * side);
            for y in 0..side {
                for x in 0..side {
                    let offset = east * (x as f64 * spacing - radius)
                        + north * (y as f64 * spacing - radius);
                    let dir = (center + offset).normalize().to_array();
                    heights.push(
                        field.height_m(dir, 32.0).max(0.0)
                            + lod::ground_relief_height_m(field, dir, 0.012)
                            - base_height,
                    );
                    stats.ground_water_samples += 1;
                }
            }
            let Some(pool) = fill_closed_basin(
                &heights,
                side,
                spacing,
                (cover.wetness * 1.8).min(0.95),
                0.045,
            ) else {
                continue;
            };
            stats.ground_water_basins += pool.basin_count;
            for tri in pool.triangles {
                // ENU uses east,north,up; x,z triangle winding is corrected here.
                for v in [tri[0], tri[2], tri[1]] {
                    vertex(
                        &mut water,
                        center + east * v[0] + north * v[2] + up * (base_height + v[1]),
                        up,
                        anchor,
                    );
                }
            }
            centers.push(center);
            stats.ground_puddles += 1;
        }
        stats.ground_water_triangles = water.indices.len() / 3;
        publish(prepared, water, anchor);
    }
    prepared.local_ground_start = Some(start);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn equal_counts_cannot_reuse_different_local_cover_geometry() {
        let make = |height: f64| {
            let rock = ScatterItem {
                id: thessa_worldgen_rocky::scatter::ScatterId(1),
                kind: ScatterKind::Pebble,
                direction: [0.0, 1.0, 0.0],
                ground_height_m: 0.0,
                scale: 0.1,
                yaw_rad: 0.0,
            };
            let mut p = convert(vec![rock], 3_200_000.0).unwrap();
            let start = p.asset.primitives.len();
            let mut mesh = blank(0);
            for v in [DVec3::ZERO, DVec3::X, DVec3::new(0.0, height, 1.0)] {
                vertex(&mut mesh, v, DVec3::Y, DVec3::ZERO);
            }
            publish(&mut p, mesh, DVec3::ZERO);
            p.local_ground_start = Some(start);
            p
        };
        let first = make(0.0);
        assert!(first.same_prototypes(&make(0.0)));
        assert!(!first.same_prototypes(&make(0.001)));
    }
    #[test]
    fn coastal_cover_publishes_bounded_water_and_moss_without_extra_passes_or_images() {
        let eye = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818);
        let forward = DVec3::new(0.4133582597898216, 0.9039379739103636, 0.10968631813657403);
        let dir = eye.normalize().to_array();
        let context = Context::SurveyWithUnderstory {
            canopy: survey_window(dir.into()),
            understory: understory_window(dir.into()),
        };
        let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
        let mut request = thessa_graphics::RequestedGraphics::default();
        request.renderer.terrain_ground_detail = true;
        request.renderer.terrain_ground_puddles = true;
        let settings = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (prepared, stats) = prepare(context, &settings, &field).unwrap();
        let prepared = prepared.unwrap();
        eprintln!(
            "wet_ground moss={} puddles={} water_samples={} water_triangles={} payload={} primitives={}",
            stats.ground_moss_clumps,
            stats.ground_puddles,
            stats.ground_water_samples,
            stats.ground_water_triangles,
            stats.prototype_bytes,
            prepared.asset.primitives.len()
        );
        assert!(stats.ground_moss_clumps > 0 && stats.ground_moss_clumps <= 384);
        assert!(stats.ground_puddles > 0 && stats.ground_puddles <= 24);
        assert!(stats.ground_water_samples <= 256 * 17 * 17);
        assert!(stats.prototype_bytes < 7_000_000 && prepared.asset.primitives.len() <= 36);
        let mut visible = 0;
        for (primitive, range) in prepared.asset.primitives.iter().zip(&prepared.ranges) {
            if prepared.asset.materials[primitive.material].dielectric_f0 >= 0.03 {
                continue;
            }
            let anchor = prepared.instances[range.start as usize].position_m;
            for tri in primitive.indices.as_chunks::<3>().0 {
                let points = tri
                    .map(|i| DVec3::from(primitive.vertices[i as usize].position.map(f64::from)));
                let normal = DVec3::from(primitive.vertices[tri[0] as usize].normal.map(f64::from));
                assert!(
                    (points[1] - points[0])
                        .cross(points[2] - points[0])
                        .dot(normal)
                        > 0.0
                );
                let center = anchor + (points[0] + points[1] + points[2]) / 3.0;
                if (center - eye).normalize().dot(forward) > 0.85 {
                    visible += 1;
                }
            }
        }
        eprintln!("water triangles in showcase cone: {visible}");
        assert!(visible > 0);
        prepared.asset.validate().unwrap();
        assert!(prepared.asset.images.is_empty());
        assert!(prepared.same_prototypes(&prepared));
        assert_eq!(
            prepared.descriptors.len(),
            stats.ground_pebbles,
            "local geometry must not invent scatter descriptors"
        );
        request.renderer.terrain_ground_puddles = false;
        let no_water = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (dry, stats) = prepare(context, &no_water, &field).unwrap();
        assert_eq!(
            (
                stats.ground_puddles,
                stats.ground_water_samples,
                stats.ground_water_triangles
            ),
            (0, 0, 0)
        );
        assert!(!prepared.same_prototypes(&dry.unwrap()));
        request.renderer.terrain_ground_puddles = true;
        request.renderer.surface_quality = thessa_graphics::Quality::Low;
        let low = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (none, stats) = prepare(context, &low, &field).unwrap();
        assert!(none.is_none());
        assert_eq!(
            (
                stats.ground_moss_clumps,
                stats.ground_puddles,
                stats.ground_water_samples
            ),
            (0, 0, 0)
        );
        request.renderer.terrain_ground_detail = false;
        let off = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (none, stats) = prepare(context, &off, &field).unwrap();
        assert!(none.is_none());
        assert_eq!(
            (
                stats.ground_moss_clumps,
                stats.ground_puddles,
                stats.ground_water_samples
            ),
            (0, 0, 0)
        );
    }
}
