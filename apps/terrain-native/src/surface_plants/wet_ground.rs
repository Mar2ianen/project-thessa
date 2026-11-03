//! Bounded cover geometry in the existing preparation/opaque/shadow publication.
use super::*;
use thessa_worldgen_rocky::{lod, rng, scatter::ScatterKind, surface_water::fill_closed_basin};

const ORGANIC_PALETTES: [[f32; 4]; 3] = [
    [0.028, 0.068, 0.014, 1.0],
    [0.043, 0.085, 0.017, 1.0],
    [0.024, 0.052, 0.011, 1.0],
];
const PALETTE_COUNT: usize = ORGANIC_PALETTES.len();
const GRASS_PALETTES: [[f32; 4]; 3] = [
    [0.025, 0.073, 0.010, 1.0],
    [0.039, 0.083, 0.012, 1.0],
    [0.018, 0.054, 0.009, 1.0],
];

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
fn folded_leaf(mesh: &mut Primitive, points: [DVec3; 5], anchor: DVec3) {
    let faces = [[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]];
    let mut normals = [DVec3::ZERO; 5];
    for face in faces {
        let [a, b, c] = face.map(|i| points[i]);
        let area = (b - a).cross(c - a);
        for i in face {
            normals[i] += area;
        }
    }
    let start = mesh.vertices.len() as u32;
    for (p, n) in points.into_iter().zip(normals) {
        mesh.vertices.push(Vertex {
            position: (p - anchor).as_vec3().to_array(),
            normal: n.normalize().as_vec3().to_array(),
            uv: [0.0; 2],
        });
    }
    mesh.indices
        .extend(faces.into_iter().flatten().map(|i| start + i as u32));
}

fn organic_triangle(mesh: &mut Primitive, mut vertices: [(DVec3, DVec3); 3], anchor: DVec3) {
    let [a, b, c] = vertices.map(|v| v.0);
    let expected = vertices.iter().map(|v| v.1).sum::<DVec3>();
    if (b - a).cross(c - a).dot(expected) < 0.0 {
        vertices.swap(1, 2);
    }
    for (position, normal) in vertices {
        vertex(mesh, position, normal, anchor);
    }
}

#[derive(Clone, Copy)]
struct GroundPatchFrame<'a> {
    field: &'a PlanetField,
    center: DVec3,
    east: DVec3,
    north: DVec3,
    anchor: DVec3,
}

#[derive(Clone, Copy)]
struct BladePose {
    up: DVec3,
    lean: DVec3,
    side: DVec3,
}

// Four-segment curved blades follow AMD's patch geometry idea: each leaf keeps
// a persistent root, a bent centreline, a pointed silhouette and a narrow
// width profile instead of collapsing to a flat diamond at grazing angles.
fn curved_blade(
    mesh: &mut Primitive,
    base: DVec3,
    pose: BladePose,
    length: f64,
    half_width: f64,
    anchor: DVec3,
) {
    let BladePose { up, lean, side } = pose;
    let control = base + up * (length * 0.56) + lean * 0.30;
    let tip = base + up * length + lean;
    let mut left = [DVec3::ZERO; 5];
    let mut right = [DVec3::ZERO; 5];
    let mut normals = [DVec3::ZERO; 5];
    for i in 0..5 {
        let t = i as f64 * 0.25;
        let one_minus = 1.0 - t;
        let center =
            base * (one_minus * one_minus) + control * (2.0 * one_minus * t) + tip * (t * t);
        let tangent = (control - base) * (2.0 * one_minus) + (tip - control) * (2.0 * t);
        let taper = (one_minus.sqrt() * 0.86 + 0.14).min(1.0);
        let width = half_width * taper;
        left[i] = center - side * width;
        right[i] = center + side * width;
        let mut normal = side.cross(tangent).normalize();
        if normal.dot(up) < 0.0 {
            normal = -normal;
        }
        normals[i] = (normal * 0.82 + up * 0.18).normalize();
    }
    for i in 0..4 {
        organic_triangle(
            mesh,
            [
                (left[i], normals[i]),
                (right[i], normals[i]),
                (left[i + 1], normals[i + 1]),
            ],
            anchor,
        );
        organic_triangle(
            mesh,
            [
                (right[i], normals[i]),
                (right[i + 1], normals[i + 1]),
                (left[i + 1], normals[i + 1]),
            ],
            anchor,
        );
    }
}

// Low, lobed cushions break up the scanned soil without becoming flat decals.
// Each point follows the same rendered soil height source as rocks and litter.
fn moss_cushion(
    mesh: &mut Primitive,
    frame: &GroundPatchFrame<'_>,
    radii: [f64; 2],
    height: f64,
    phase: f64,
    seed: u64,
    id: i64,
) {
    let GroundPatchFrame {
        field,
        center,
        east,
        north,
        anchor,
    } = *frame;
    let [radius_x, radius_z] = radii;
    let up = center.normalize();
    let soil = ground_position(field, center);
    const SIDES: usize = 10;
    let mut lower = [(DVec3::ZERO, DVec3::ZERO); SIDES];
    let mut middle = [(DVec3::ZERO, DVec3::ZERO); SIDES];
    let mut upper = [(DVec3::ZERO, DVec3::ZERO); SIDES];
    for i in 0..SIDES {
        let angle = std::f64::consts::TAU * i as f64 / SIDES as f64;
        let lobe =
            1.0 + 0.11 * (3.0 * angle + phase).sin() + 0.045 * (5.0 * angle - phase * 0.7).cos();
        let radial = (east * angle.cos() + north * angle.sin()).normalize();
        let sample = |scale: f64, lift: f64| {
            let surface = ground_position(
                field,
                soil + east * (radius_x * scale * lobe * angle.cos())
                    + north * (radius_z * scale * lobe * angle.sin()),
            );
            surface + up * lift
        };
        lower[i] = (sample(1.0, 0.002), (radial * 0.62 + up * 0.78).normalize());
        middle[i] = (
            sample(0.76, height * (0.34 + 0.035 * lobe)),
            (radial * 0.44 + up * 0.90).normalize(),
        );
        upper[i] = (
            sample(0.43, height * (0.72 + 0.055 * lobe)),
            (radial * 0.24 + up * 0.97).normalize(),
        );
    }
    let apex = (soil + up * height, up);
    for i in 0..SIDES {
        let j = (i + 1) % SIDES;
        organic_triangle(mesh, [lower[i], lower[j], middle[j]], anchor);
        organic_triangle(mesh, [lower[i], middle[j], middle[i]], anchor);
        organic_triangle(mesh, [middle[i], middle[j], upper[j]], anchor);
        organic_triangle(mesh, [middle[i], upper[j], upper[i]], anchor);
        organic_triangle(mesh, [upper[i], upper[j], apex], anchor);
    }
    let hash = |channel| rng::hash01(seed, channel, id, 0);
    for leaf in 0..8 {
        let angle = phase + hash(7900 + leaf) * std::f64::consts::TAU;
        let radial = (east * angle.cos() + north * angle.sin()).normalize();
        let side = up.cross(radial).normalize();
        let root_radius = 0.018 + hash(7940 + leaf) * 0.055;
        let soil_root = ground_position(
            field,
            soil + east * (angle.cos() * root_radius) + north * (angle.sin() * root_radius),
        ) + up * 0.003;
        let length = 0.028 + hash(7980 + leaf) * 0.034;
        let width = 0.002 + hash(8020 + leaf) * 0.0018;
        let lean = radial * (0.008 + hash(8060 + leaf) * 0.018);
        curved_blade(
            mesh,
            soil_root,
            BladePose { up, lean, side },
            length,
            width,
            anchor,
        );
    }
}

fn grass_tuft(
    mesh: &mut Primitive,
    frame: &GroundPatchFrame<'_>,
    seed: u64,
    id: i64,
    height_scale: f64,
) {
    let GroundPatchFrame {
        field,
        center,
        east,
        north,
        anchor,
    } = *frame;
    let up = center.normalize();
    let soil = ground_position(field, center);
    let hash = |channel| rng::hash01(seed, channel, id, 0);
    for blade in 0..14 {
        let angle = hash(7500 + blade) * std::f64::consts::TAU;
        let radial = (east * angle.cos() + north * angle.sin()).normalize();
        let side = up.cross(radial).normalize();
        let root_angle = hash(7540 + blade) * std::f64::consts::TAU;
        let root_radius = 0.02 + hash(7580 + blade) * 0.105;
        let base = ground_position(
            field,
            soil + east * (root_angle.cos() * root_radius)
                + north * (root_angle.sin() * root_radius),
        ) + up * 0.0015;
        let length = (0.11 + hash(7620 + blade) * 0.11) * height_scale;
        let width = 0.0038 + hash(7660 + blade) * 0.0028;
        let bend = radial * (0.030 + hash(7700 + blade) * 0.050);
        curved_blade(
            mesh,
            base,
            BladePose {
                up,
                lean: bend,
                side,
            },
            length,
            width,
            anchor,
        );
    }
}

fn ground_position(field: &PlanetField, p: DVec3) -> DVec3 {
    let dir = p.normalize();
    let height = field.height_m(dir.to_array(), 32.0).max(0.0)
        + lod::ground_relief_height_m(field, dir.to_array(), 0.012);
    dir * (field.params.radius_m + height)
}

// Authored fallen pinnae, not simulated litter mass or decomposition. Each
// corner follows the same soil source as pebbles, cushions and sampled pools.
fn fallen_leaf(
    mesh: &mut Primitive,
    field: &PlanetField,
    center: DVec3,
    along: DVec3,
    length: f64,
    width: f64,
    anchor: DVec3,
) {
    let up = center.normalize();
    let across = up.cross(along).normalize();
    let at = |x: f64, y: f64, lift: f64| {
        ground_position(field, center + along * x + across * y) + up * lift
    };
    let root = at(-length * 0.5, 0.0, 0.001);
    let tip = at(length * 0.5, 0.0, 0.002);
    let left = at(0.0, -width, 0.001);
    let right = at(length * 0.1, width, 0.001);
    let ridge = at(0.0, 0.0, width * 0.35 + 0.002);
    folded_leaf(mesh, [root, left, tip, right, ridge], anchor);
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
    let moss_limit = if high { 3400 } else { 1200 };
    let grass_limit = if high { 4800 } else { 1600 };
    let pool_limit = if high { 24 } else { 4 };
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
    let moss_materials = ORGANIC_PALETTES.map(|base_color| {
        let material = prepared.asset.materials.len();
        prepared.asset.materials.push(Material {
            base_color,
            metallic: 0.0,
            roughness: 0.98,
            double_sided: true,
            ..Default::default()
        });
        material
    });
    let mut moss_meshes = moss_materials.map(blank);
    // Stable world-addressed candidates are admitted near-first; the underlying
    // damp/moss field creates patches instead of a uniform scatter carpet.
    let half_span = window.span_m(field.params.radius_m) * 0.5;
    let mut moss_sites = sites.clone();
    moss_sites.sort_by_cached_key(|item| {
        let distance2 =
            (DVec3::from(item.direction) * field.params.radius_m - anchor).length_squared();
        let rank = rng::hash01(field.params.seed, 7291, item.id.0 as i64, 0)
            * (1.0 + distance2 / half_span.powi(2));
        (rank.to_bits(), item.id)
    });
    for d in &moss_sites {
        if stats.ground_moss_clumps >= moss_limit {
            break;
        }
        let cover = lod::ground_cover_at(field, d.direction, 0.012);
        if cover.moss < 0.12 {
            continue;
        }
        let up = DVec3::from(d.direction);
        let (east, north, _) = enu_basis(d.direction);
        let east = DVec3::from(east);
        let north = DVec3::from(north);
        let frame = GroundPatchFrame {
            field,
            center: up * (field.params.radius_m + d.ground_height_m),
            east,
            north,
            anchor,
        };
        let hash = |c| rng::hash01(field.params.seed, c, d.id.0 as i64, 0);
        let palette = ((hash(7804) * PALETTE_COUNT as f64) as usize).min(PALETTE_COUNT - 1);
        moss_cushion(
            &mut moss_meshes[palette],
            &frame,
            [0.085 + hash(7800) * 0.105, 0.072 + hash(7801) * 0.092],
            0.052 + cover.moss * 0.025 + hash(7802) * 0.022,
            hash(7803) * std::f64::consts::TAU,
            field.params.seed,
            d.id.0 as i64,
        );
        stats.ground_moss_clumps += 1;
    }
    for mesh in moss_meshes {
        let before = prepared.asset.primitives.len();
        publish(prepared, mesh, anchor);
        stats.ground_moss_meshes += usize::from(prepared.asset.primitives.len() > before);
    }
    let litter_material = prepared.asset.materials.len();
    prepared.asset.materials.push(Material {
        base_color: [0.052, 0.034, 0.018, 1.0],
        roughness: 0.85,
        double_sided: true,
        ..Default::default()
    });
    let mut litter = blank(litter_material);
    for d in sites.iter().take(if high { 2048 } else { 512 }) {
        let cover = lod::ground_cover_at(field, d.direction, 0.012);
        if cover.wetness < 0.20 || cover.moss > 0.70 {
            continue;
        }
        let hash = |c| rng::hash01(field.params.seed, c, d.id.0 as i64, 0);
        let up = DVec3::from(d.direction);
        let (east, north, _) = enu_basis(d.direction);
        let angle = hash(7310) * std::f64::consts::TAU;
        let along = DVec3::from(east) * angle.cos() + DVec3::from(north) * angle.sin();
        let center = up * field.params.radius_m + along * (0.12 + hash(7311) * 0.25);
        fallen_leaf(
            &mut litter,
            field,
            center,
            along,
            0.08 + hash(7312) * 0.16,
            0.012 + hash(7313) * 0.018,
            anchor,
        );
        stats.ground_litter_fragments += 1;
    }
    publish(prepared, litter, anchor);
    let grass_materials = GRASS_PALETTES.map(|base_color| {
        let material = prepared.asset.materials.len();
        prepared.asset.materials.push(Material {
            base_color,
            metallic: 0.0,
            roughness: 0.88,
            double_sided: true,
            ..Default::default()
        });
        material
    });
    let mut grass_meshes = grass_materials.map(blank);
    let mut grass_sites = sites.clone();
    grass_sites.sort_by_cached_key(|item| {
        let distance2 =
            (DVec3::from(item.direction) * field.params.radius_m - anchor).length_squared();
        let rank = rng::hash01(field.params.seed, 7421, item.id.0 as i64, 0)
            * (1.0 + distance2 / half_span.powi(2));
        (rank.to_bits(), item.id)
    });
    for d in &grass_sites {
        if stats.ground_grass_tufts >= grass_limit {
            break;
        }
        let cover = lod::ground_cover_at(field, d.direction, 0.012);
        if cover.wetness < 0.12 {
            continue;
        }
        let hash = |channel| rng::hash01(field.params.seed, channel, d.id.0 as i64, 0);
        // Dampness and the broader moss field control local density. A separate
        // three-way pigment palette keeps this low stratum from reading as one
        // repeated green object.
        let density = cover.wetness * (0.94 - 0.46 * cover.moss).clamp(0.35, 0.94);
        if hash(7422) > density {
            continue;
        }
        let palette = ((hash(7423) * PALETTE_COUNT as f64) as usize).min(PALETTE_COUNT - 1);
        let (east, north, _) = enu_basis(d.direction);
        let frame = GroundPatchFrame {
            field,
            center: DVec3::from(d.direction) * (field.params.radius_m + d.ground_height_m),
            east: DVec3::from(east),
            north: DVec3::from(north),
            anchor,
        };
        grass_tuft(
            &mut grass_meshes[palette],
            &frame,
            field.params.seed,
            d.id.0 as i64,
            if high { 1.0 } else { 0.84 },
        );
        stats.ground_grass_tufts += 1;
    }
    for mesh in grass_meshes {
        let before = prepared.asset.primitives.len();
        publish(prepared, mesh, anchor);
        stats.ground_grass_meshes += usize::from(prepared.asset.primitives.len() > before);
    }
    if settings.terrain_ground_puddles {
        let material = prepared.asset.materials.len();
        let f0 = ((1.333_f32 - 1.0) / (1.333 + 1.0)).powi(2);
        prepared.asset.materials.push(Material {
            base_color: [0.008, 0.006, 0.004, 1.0],
            metallic: 0.0,
            roughness: 0.008,
            dielectric_f0: f0,
            ..Default::default()
        });
        let mut water = blank(material);
        let mut centers: Vec<DVec3> = vec![];
        let side = 17;
        let radius = 1.20;
        let spacing = radius * 2.0 / (side - 1) as f64;
        let mut pool_sites = sites.clone();
        pool_sites.sort_by_cached_key(|item| {
            let distance2 =
                (DVec3::from(item.direction) * field.params.radius_m - anchor).length_squared();
            let rank = rng::hash01(field.params.seed, 7291, item.id.0 as i64, 0)
                * (0.20 + distance2 / (half_span * 0.5).powi(2));
            (rank.to_bits(), item.id)
        });
        for d in pool_sites
            .iter()
            .filter(|d| rng::hash01(field.params.seed, 7200, d.id.0 as i64, 0) < 0.55)
            .take(256)
        {
            if stats.ground_puddles >= pool_limit {
                break;
            }
            let up = DVec3::from(d.direction);
            let center = up * field.params.radius_m;
            if centers
                .iter()
                .any(|p| p.distance_squared(center) < (radius * 1.15_f64).powi(2))
            {
                continue;
            }
            let cover = lod::ground_cover_at(field, d.direction, 0.012);
            if cover.wetness < 0.18 {
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
                (cover.wetness * 2.2).min(0.98),
                0.055,
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
    fn coastal_cover_fits_shared_budget_with_full_canopy_and_understory() {
        let request = thessa_graphics::RequestedGraphics::from_toml(include_str!(
            "../../../../data/graphics/surface-showcase.toml"
        ))
        .unwrap();
        let settings = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let field = thessa_flight_authority::launch_site::canonical_world_field().unwrap();
        let dir = DVec3::new(-2835051.776623983, 1279289.634545036, -752292.6983244818)
            .normalize()
            .to_array();
        let context = Context::SurveyWithUnderstory {
            canopy: survey_window(dir.into()),
            understory: understory_window(dir.into()),
        };
        let (prepared, stats) = prepare(context, &settings, &field).unwrap();
        let prepared = prepared.unwrap();
        assert!(stats.retained > 1000 && stats.understory_accepted > 1000);
        assert!(
            stats.ground_litter_fragments > 0
                && stats.ground_moss_clumps > 0
                && stats.ground_grass_tufts > 0
                && stats.ground_puddles > 0
        );
        assert!(stats.ground_moss_clumps > 192);
        assert!(stats.prototype_bytes < 48_000_000 && prepared.asset.primitives.len() <= 37);
        prepared.asset.validate().unwrap();
        eprintln!(
            "full_coastal_cover payload={} ranges={} litter={}",
            stats.prototype_bytes,
            prepared.ranges.len(),
            stats.ground_litter_fragments
        );
    }
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
            "wet_ground moss={} grass={} puddles={} water_samples={} water_triangles={} payload={} primitives={}",
            stats.ground_moss_clumps,
            stats.ground_grass_tufts,
            stats.ground_puddles,
            stats.ground_water_samples,
            stats.ground_water_triangles,
            stats.prototype_bytes,
            prepared.asset.primitives.len()
        );
        assert!(stats.ground_moss_clumps > 0 && stats.ground_moss_clumps <= 3400);
        assert!(stats.ground_grass_tufts > 0 && stats.ground_grass_tufts <= 4800);
        assert!(stats.ground_moss_meshes <= PALETTE_COUNT);
        assert!(stats.ground_grass_meshes <= PALETTE_COUNT);
        assert!(stats.ground_litter_fragments > 0 && stats.ground_litter_fragments <= 2048);
        assert!(stats.ground_puddles > 0 && stats.ground_puddles <= 24);
        assert!(stats.ground_water_samples <= 256 * 17 * 17);
        assert!(stats.prototype_bytes < 48_000_000 && prepared.asset.primitives.len() <= 37);
        let local = &prepared.asset.primitives[prepared.local_ground_start.unwrap()..];
        let moss_anchor = prepared.instances
            [prepared.ranges[prepared.local_ground_start.unwrap()].start as usize]
            .position_m;
        let moss_meshes: Vec<_> = local
            .iter()
            .filter(|mesh| prepared.asset.materials[mesh.material].roughness == 0.98)
            .collect();
        assert!(!moss_meshes.is_empty());
        let near_moss_vertices: usize = moss_meshes
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .filter(|v| (moss_anchor + DVec3::from(v.position.map(f64::from))).distance(eye) < 10.0)
            .count();
        let distant_moss_vertices: usize = moss_meshes
            .iter()
            .flat_map(|mesh| &mesh.vertices)
            .filter(|v| {
                (moss_anchor + DVec3::from(v.position.map(f64::from))).distance(eye) >= 30.0
            })
            .count();
        eprintln!(
            "moss_near_eye vertices={near_moss_vertices} distant={distant_moss_vertices} total={}",
            moss_meshes
                .iter()
                .map(|mesh| mesh.vertices.len())
                .sum::<usize>()
        );
        assert!(
            near_moss_vertices >= 900,
            "the near floor needs resolved moss shoots"
        );
        assert!(
            distant_moss_vertices >= 1000,
            "bounded near priority must retain moss in neighboring cells"
        );
        assert_eq!(
            moss_meshes
                .iter()
                .map(|mesh| mesh.indices.len())
                .sum::<usize>(),
            stats.ground_moss_clumps * (50 + 8 * 8) * 3,
            "each compact rounded cushion has three dome rings and eight curved leaflets"
        );
        let litter = local
            .iter()
            .find(|mesh| prepared.asset.materials[mesh.material].roughness == 0.85)
            .unwrap();
        assert_eq!(litter.indices.len(), stats.ground_litter_fragments * 12);
        let litter_range = prepared
            .asset
            .primitives
            .iter()
            .position(|mesh| std::ptr::eq(mesh, litter))
            .unwrap();
        let litter_anchor =
            prepared.instances[prepared.ranges[litter_range].start as usize].position_m;
        for v in &litter.vertices {
            let p = litter_anchor + DVec3::from(v.position.map(f64::from));
            let clearance = p.length() - ground_position(&field, p).length();
            assert!(
                (0.0005..=0.014).contains(&clearance),
                "litter clearance {clearance}"
            );
            assert!(DVec3::from(v.normal.map(f64::from)).dot(p.normalize()) > 0.0);
        }
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
        request.renderer.surface_quality = thessa_graphics::Quality::Medium;
        let medium = ResolvedGraphicsSettings::from_requested(
            &request,
            &thessa_graphics::Capabilities::unknown(),
        );
        let (_, stats) = prepare(context, &medium, &field).unwrap();
        assert!(stats.ground_moss_clumps > 0 && stats.ground_moss_clumps <= 1200);
        assert!(stats.ground_grass_tufts > 0 && stats.ground_grass_tufts <= 1600);
        assert!(stats.ground_litter_fragments > 0 && stats.ground_litter_fragments <= 512);
        assert!(stats.ground_puddles > 0 && stats.ground_puddles <= 4);
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
                stats.ground_grass_tufts,
                stats.ground_moss_meshes,
                stats.ground_grass_meshes,
                stats.ground_litter_fragments,
                stats.ground_puddles,
                stats.ground_water_samples
            ),
            (0, 0, 0, 0, 0, 0, 0)
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
                stats.ground_grass_tufts,
                stats.ground_moss_meshes,
                stats.ground_grass_meshes,
                stats.ground_litter_fragments,
                stats.ground_puddles,
                stats.ground_water_samples
            ),
            (0, 0, 0, 0, 0, 0, 0)
        );
    }
}
