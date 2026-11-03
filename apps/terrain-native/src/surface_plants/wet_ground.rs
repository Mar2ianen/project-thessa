//! Bounded cover geometry in the existing preparation/opaque/shadow publication.
use super::*;
use thessa_worldgen_rocky::{lod, rng, scatter::ScatterKind, surface_water::fill_closed_basin};

#[cfg(test)]
#[path = "wet_ground_acceptance.rs"]
mod acceptance_tests;

const ORGANIC_PALETTES: [[f32; 4]; 3] = [
    [0.039, 0.095, 0.020, 1.0],
    [0.060, 0.119, 0.024, 1.0],
    [0.034, 0.073, 0.015, 1.0],
];
const PALETTE_COUNT: usize = ORGANIC_PALETTES.len();
const GRASS_BLADES_PER_TUFT: u32 = 14;
const GRASS_TUFTS_PER_HIGH_PATCH: usize = 4;
const GRASS_SHAPE_VARIANTS: usize = 4;
const GRASS_PALETTES: [[f32; 4]; 3] = [
    [0.035, 0.102, 0.014, 1.0],
    [0.055, 0.116, 0.017, 1.0],
    [0.026, 0.076, 0.012, 1.0],
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

fn organic_triangle_uv(
    mesh: &mut Primitive,
    mut vertices: [(DVec3, DVec3, [f32; 2]); 3],
    anchor: DVec3,
) {
    let [a, b, c] = vertices.map(|v| v.0);
    let expected = vertices.iter().map(|v| v.1).sum::<DVec3>();
    if (b - a).cross(c - a).dot(expected) < 0.0 {
        vertices.swap(1, 2);
    }
    for (position, normal, uv) in vertices {
        mesh.indices.push(mesh.vertices.len() as u32);
        mesh.vertices.push(Vertex {
            position: (position - anchor).as_vec3().to_array(),
            normal: normal.as_vec3().to_array(),
            uv,
        });
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

#[derive(Clone, Copy)]
struct BladeShape {
    length: f64,
    half_width: f64,
    segments: usize,
    taper_start: Option<f64>,
}

// Curved blades follow AMD's patch geometry idea: each leaf keeps a persistent
// root, bent centreline, pointed silhouette and narrow tapered width profile.
fn curved_blade(
    mesh: &mut Primitive,
    base: DVec3,
    pose: BladePose,
    shape: BladeShape,
    anchor: DVec3,
) {
    let BladeShape {
        length,
        half_width,
        segments,
        taper_start,
    } = shape;
    let BladePose { up, lean, side } = pose;
    let control = base + up * (length * 0.56) + lean * 0.30;
    let tip = base + up * length + lean;
    debug_assert!((1..=4).contains(&segments));
    let sample = |t: f64| {
        let one_minus = 1.0 - t;
        let center =
            base * (one_minus * one_minus) + control * (2.0 * one_minus * t) + tip * (t * t);
        let tangent = (control - base) * (2.0 * one_minus) + (tip - control) * (2.0 * t);
        let taper = if let Some(start) = taper_start {
            let x = ((t - start) / (1.0 - start)).clamp(0.0, 1.0);
            let smooth = x * x * (3.0 - 2.0 * x);
            1.0 - smooth * smooth
        } else {
            one_minus.sqrt() * 0.92
        };
        let width = half_width * taper;
        let mut normal = side.cross(tangent).normalize();
        if normal.dot(up) < 0.0 {
            normal = -normal;
        }
        let normal = (normal * 0.82 + up * 0.18).normalize();
        (center - side * width, center + side * width, normal)
    };
    let mut rings = [(0.0, DVec3::ZERO, DVec3::ZERO, DVec3::ZERO); 4];
    for (i, ring) in rings.iter_mut().take(segments).enumerate() {
        let t = if segments == 1 {
            0.04
        } else {
            0.04 + 0.76 * i as f64 / (segments - 1) as f64
        };
        let (left, right, normal) = sample(t);
        *ring = (t, left, right, normal);
    }
    let root_normal = rings[0].3;
    let (_, left, right, normal) = rings[0];
    organic_triangle_uv(
        mesh,
        [
            (base, root_normal, [0.0, 0.0]),
            (left, normal, [0.0, rings[0].0 as f32]),
            (right, normal, [1.0, rings[0].0 as f32]),
        ],
        anchor,
    );
    for pair in rings[..segments].windows(2) {
        let (previous_t, previous_left, previous_right, previous_normal) = pair[0];
        let (t, left, right, normal) = pair[1];
        let previous_v = previous_t as f32;
        let v = t as f32;
        organic_triangle_uv(
            mesh,
            [
                (previous_left, previous_normal, [0.0, previous_v]),
                (previous_right, previous_normal, [1.0, previous_v]),
                (left, normal, [0.0, v]),
            ],
            anchor,
        );
        organic_triangle_uv(
            mesh,
            [
                (previous_right, previous_normal, [1.0, previous_v]),
                (right, normal, [1.0, v]),
                (left, normal, [0.0, v]),
            ],
            anchor,
        );
    }
    let (previous_t, previous_left, previous_right, previous_normal) = rings[segments - 1];
    let (_, _, tip_normal) = sample(1.0);
    organic_triangle_uv(
        mesh,
        [
            (previous_left, previous_normal, [0.0, previous_t as f32]),
            (previous_right, previous_normal, [1.0, previous_t as f32]),
            (tip, tip_normal, [0.5, 1.0]),
        ],
        anchor,
    );
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
            BladeShape {
                length,
                half_width: width,
                segments: 4,
                taper_start: None,
            },
            anchor,
        );
    }
}

fn grass_tuft(mesh: &mut Primitive, seed: u64, patch_id: i64, height_scale: f64) {
    let up = DVec3::Y;
    let east = DVec3::X;
    let north = DVec3::Z;
    let hash = |channel| rng::hash01(seed, channel, patch_id, 0);
    let fan_phase = hash(7480) * std::f64::consts::TAU;
    for blade in 0..GRASS_BLADES_PER_TUFT {
        let sector = blade as f64 * std::f64::consts::TAU / GRASS_BLADES_PER_TUFT as f64;
        let jitter = (hash(7500 + blade) - 0.5) * 0.55;
        let angle = fan_phase + sector + jitter;
        let radial = (east * angle.cos() + north * angle.sin()).normalize();
        let side = up.cross(radial).normalize();
        // The shared crown stays on the sampled soil; radial cantilever spread
        // restores the tuft footprint without offsetting any blade root.
        let base = DVec3::ZERO;
        let length = (0.13 + hash(7620 + blade) * 0.15) * height_scale;
        let width = (0.0038 + hash(7660 + blade) * 0.0028) * 1.6;
        let bend = radial * (0.10 + hash(7700 + blade) * 0.12) * height_scale;
        curved_blade(
            mesh,
            base,
            BladePose {
                up,
                lean: bend,
                side,
            },
            BladeShape {
                length,
                half_width: width,
                segments: 3,
                taper_start: Some(0.40),
            },
            DVec3::ZERO,
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
    compact_vertices(&mut mesh);
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

fn compact_vertices(mesh: &mut Primitive) {
    let mut unique = std::collections::BTreeMap::new();
    let mut vertices = Vec::new();
    for index in &mut mesh.indices {
        let v = mesh.vertices[*index as usize];
        let key = (
            v.position.map(f32::to_bits),
            v.normal.map(f32::to_bits),
            v.uv.map(f32::to_bits),
        );
        *index = *unique.entry(key).or_insert_with(|| {
            let i = vertices.len() as u32;
            vertices.push(v);
            i
        });
    }
    mesh.vertices = vertices;
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
    let moss_limit = if high { 4250 } else { 1200 };
    let grass_limit = if high {
        GRASS_TUFTS_PER_HIGH_PATCH * 5300
    } else {
        1600
    };
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
            [
                (0.085 + hash(7800) * 0.105) * 1.30,
                (0.072 + hash(7801) * 0.092) * 1.30,
            ],
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
    let grass_tufts_per_patch = if high { GRASS_TUFTS_PER_HIGH_PATCH } else { 1 };
    let mut grass_meshes = Vec::with_capacity(PALETTE_COUNT * GRASS_SHAPE_VARIANTS);
    for (palette, material) in grass_materials.into_iter().enumerate() {
        for variant in 0..GRASS_SHAPE_VARIANTS {
            let mut mesh = blank(material);
            let shape_id = palette * GRASS_SHAPE_VARIANTS + variant;
            grass_tuft(
                &mut mesh,
                field.params.seed ^ (0x6772_6173_735f_0000 + shape_id as u64 * 0x9e37),
                shape_id as i64,
                if high { 1.08 } else { 0.84 },
            );
            grass_meshes.push(mesh);
        }
    }
    let mut grass_instances: Vec<Vec<MeshInstance>> =
        (0..grass_meshes.len()).map(|_| Vec::new()).collect();
    let mut grass_sites = sites.clone();
    grass_sites.sort_by_cached_key(|item| {
        let distance2 =
            (DVec3::from(item.direction) * field.params.radius_m - anchor).length_squared();
        let rank = rng::hash01(field.params.seed, 7421, item.id.0 as i64, 0)
            * (1.0 + distance2 / half_span.powi(2));
        (rank.to_bits(), item.id)
    });
    'sites: for d in &grass_sites {
        if stats.ground_grass_tufts >= grass_limit {
            break;
        }
        let cover = lod::ground_cover_at(field, d.direction, 0.012);
        if cover.wetness < 0.12 {
            continue;
        }
        let hash = |channel| rng::hash01(field.params.seed, channel, d.id.0 as i64, 0);
        // Moisture controls occupancy, but moss is not a competing mask: the
        // two low strata can coexist. Stable per-site hashes retain patchiness.
        // A separate three-way palette keeps grass from reading as one object.
        let density = cover.wetness;
        if hash(7422) > density {
            continue;
        }
        let (east, north, up) = enu_basis(d.direction);
        let east = DVec3::from(east);
        let north = DVec3::from(north);
        let radial = DVec3::from(up);
        let surface = radial * field.params.radius_m;
        let east_tangent = ground_position(field, surface + east) - ground_position(field, surface);
        let north_tangent =
            ground_position(field, surface + north) - ground_position(field, surface);
        let ground_normal = east_tangent.cross(north_tangent).normalize();
        let ground_normal = if ground_normal.dot(radial) < 0.0 {
            -ground_normal
        } else {
            ground_normal
        };
        let east_axis =
            (east_tangent - ground_normal * east_tangent.dot(ground_normal)).normalize();
        let basis = DMat3::from_cols(east_axis, ground_normal, east_axis.cross(ground_normal));
        let scale = if high {
            0.88 + hash(7425) * 0.24
        } else {
            0.92 + hash(7425) * 0.16
        };
        for tuft in 0..grass_tufts_per_patch {
            if stats.ground_grass_tufts >= grass_limit {
                break 'sites;
            }
            let tuft_hash = |channel| {
                rng::hash01(
                    field.params.seed,
                    channel + tuft as u32 * 31,
                    d.id.0 as i64,
                    0,
                )
            };
            let palette =
                ((tuft_hash(7423) * PALETTE_COUNT as f64) as usize).min(PALETTE_COUNT - 1);
            let variant = ((tuft_hash(7426) * GRASS_SHAPE_VARIANTS as f64) as usize)
                .min(GRASS_SHAPE_VARIANTS - 1);
            let shape = palette * GRASS_SHAPE_VARIANTS + variant;
            let angle = tuft_hash(7430) * std::f64::consts::TAU;
            let radius = if tuft == 0 {
                0.0
            } else {
                0.10 + tuft_hash(7431) * 0.11
            };
            let site = DVec3::from(d.direction) * field.params.radius_m
                + east * (angle.cos() * radius)
                + north * (angle.sin() * radius);
            let position = ground_position(field, site);
            let yaw = tuft_hash(7424) * std::f64::consts::TAU;
            grass_instances[shape].push(MeshInstance {
                position_m: position,
                rotation: (DQuat::from_mat3(&basis) * DQuat::from_rotation_y(yaw)).normalize(),
                scale,
            });
            stats.ground_grass_tufts += 1;
        }
    }
    for (mut mesh, instances) in grass_meshes.into_iter().zip(grass_instances) {
        if instances.is_empty() {
            continue;
        }
        compact_vertices(&mut mesh);
        let start = prepared.instances.len() as u32;
        prepared.instances.extend(instances);
        let end = prepared.instances.len() as u32;
        prepared.asset.primitives.push(mesh);
        prepared.ranges.push(start..end);
        prepared.reduced_primitives.push(false);
        stats.ground_grass_meshes += 1;
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
        assert!(stats.ground_moss_clumps > 0 && stats.ground_moss_clumps <= 4250);
        assert!(
            stats.ground_grass_tufts > 0
                && stats.ground_grass_tufts <= 5300 * GRASS_TUFTS_PER_HIGH_PATCH
        );
        assert!(stats.ground_moss_meshes <= PALETTE_COUNT);
        assert!(stats.ground_grass_meshes <= PALETTE_COUNT * GRASS_SHAPE_VARIANTS);
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
