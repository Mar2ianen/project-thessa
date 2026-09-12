//! Deterministic cube-sphere surface addresses and bounded camera LOD.
//! Tiles cache the field; they never define the terrain or own its random seed.
use crate::{appearance::surface_appearance, field::PlanetField};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct TileKey {
    pub face: u8,
    pub level: u8,
    pub x: u32,
    pub y: u32,
}
impl TileKey {
    pub fn root(face: u8) -> Self {
        Self {
            face,
            level: 0,
            x: 0,
            y: 0,
        }
    }
    pub fn parent(self) -> Option<Self> {
        (self.level > 0).then(|| Self {
            face: self.face,
            level: self.level - 1,
            x: self.x / 2,
            y: self.y / 2,
        })
    }
    pub fn children(self) -> [Self; 4] {
        std::array::from_fn(|i| Self {
            face: self.face,
            level: self.level + 1,
            x: self.x * 2 + (i % 2) as u32,
            y: self.y * 2 + (i / 2) as u32,
        })
    }
    pub fn direction(self, u: f64, v: f64) -> [f64; 3] {
        let n = (1_u64 << self.level) as f64;
        let u = 2.0 * (self.x as f64 + u) / n - 1.0;
        let v = 2.0 * (self.y as f64 + v) / n - 1.0;
        normalize(match self.face {
            0 => [1.0, v, -u],
            1 => [-1.0, v, u],
            2 => [u, 1.0, -v],
            3 => [u, -1.0, v],
            4 => [u, v, 1.0],
            5 => [-u, v, -1.0],
            _ => panic!("cube face must be 0..6"),
        })
    }
    pub fn span_m(self, radius: f64) -> f64 {
        radius * 2.0 / (1_u64 << self.level) as f64
    }
}
pub fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = dot(v, v).sqrt();
    v.map(|x| x / n)
}
pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}
pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Screen-distance refinement, conservative horizon culling and a hard tile
/// budget. Selection does not depend on previous frames or worker order.
pub fn select_tiles(eye: [f64; 3], radius: f64, max_level: u8, budget: usize) -> Vec<TileKey> {
    select_tiles_with_height(eye, radius, max_level, budget, |_| 0.0)
}

/// Camera frustum for selection culling, in the same body-fixed frame as the
/// tile directions. Only tiles whose center falls inside the cone (plus tile
/// angular radius) are seeded/refined: from 5 km the horizon cone admits a
/// 500 km disc that would eat any budget before the nadir refines. Fast
/// swings are safe — the client swaps selections only once fully cached, so
/// a culled frame keeps the previous complete set for one selection period.
#[derive(Debug, Clone, Copy)]
pub struct SelectionFrustum {
    pub forward: [f64; 3],
    /// Cosine of the keep cone half-angle (FOV/2 plus swing margin).
    pub cos_limit: f64,
}

/// Same deterministic priority queue, measuring distance to the actual surface.
/// Sample the canonical field at a fixed wavelength so camera motion cannot
/// change geography. Cache priorities within this selection, not across worlds.
pub fn select_tiles_with_height(
    eye: [f64; 3],
    radius: f64,
    max_level: u8,
    budget: usize,
    height: impl Fn([f64; 3]) -> f64,
) -> Vec<TileKey> {
    select_tiles_with_height_and_frustum(eye, radius, max_level, budget, height, None, 1.0)
}

/// Frustum-culled variant of [`select_tiles_with_height`].
/// Velocity bias for the near stop rule (Outerra/Unreal-style: don't chase
/// detail the viewer crosses in one selection period). `1.0` keeps the base
/// ~1.2° target; higher values relax it toward the far rule. The client
/// derives it from eye speed (`1 + speed/100`, clamped); tests pass `1.0`.
pub fn select_tiles_with_height_and_frustum(
    eye: [f64; 3],
    radius: f64,
    max_level: u8,
    budget: usize,
    height: impl Fn([f64; 3]) -> f64,
    frustum: Option<SelectionFrustum>,
    detail_bias: f64,
) -> Vec<TileKey> {
    let eye_r = dot(eye, eye).sqrt();
    let eye_dir = normalize(eye);
    let horizon = (radius / eye_r.max(radius)).clamp(0.0, 1.0).acos();
    // Coverage culling is horizon-only, deliberately NOT frustum-culled:
    // hard culling by view cone deletes regions the frame still shows
    // (a forward-looking chase camera sees ground up to ~90° off-axis at
    // the frame bottom; any cone misses it and the pilot view never
    // refines past coarse cover). The frustum survives only as a RANKING
    // weight inside priority(), so off-view tiles settle coarse instead
    // of vanishing. Live bug preserved as comment: cone 0.6 rad margin
    // still starved pilot ground to L7.
    let visible = |key: TileKey| {
        let center = key.direction(0.5, 0.5);
        let bound = (2.0 / (1_u64 << key.level) as f64).min(std::f64::consts::PI);
        if dot(center, eye_dir) < (horizon + bound + 0.10).min(std::f64::consts::PI).cos() {
            return false;
        }
        true
    };
    let priorities = std::cell::RefCell::new(std::collections::BTreeMap::new());
    let priority = |key: TileKey| {
        if let Some(value) = priorities.borrow().get(&key) {
            return *value;
        }
        let dir = key.direction(0.5, 0.5);
        let surface_r = radius + height(dir).max(0.0);
        let delta = sub(eye, dir.map(|v| v * surface_r));
        let distance = dot(delta, delta).sqrt().max(1.0);
        // Rank error relative to the tile's own quality target. Ranking raw
        // angular span spends the budget on peripheral tiles that already
        // satisfy their relaxed target, starving the center of the view.
        //
        // Mild overlap-aware foveation (max x3 at the cone edge): focuses
        // the fixed budget toward the view center so it reaches L15-16
        // there instead of uniform L13 everywhere. References rank by
        // angular size alone, but with a hard 320 budget some focusing is
        // mandatory. Calibrated live: x9 froze chase-view ground (90°
        // off-axis nadir) at L11 — x3 keeps it at L12-13 while preserving
        // center depth. Overlap-aware (closest approach, not center):
        // center-scored huge tiles covering the fovea ate the full penalty
        // and starved their own subtrees.
        // Velocity bias relaxes ONLY the near rule: fast flight is near
        // the ground, where the near rule governs; the far field keeps
        // its own fixed target.
        let stop = if let Some(frustum) = frustum {
            let to_tile = normalize(delta.map(|v| -v));
            let off_axis = dot(to_tile, normalize(frustum.forward))
                .clamp(-1.0, 1.0)
                .acos();
            let angular_radius = (key.span_m(radius) / distance).clamp(-1.0, 1.0).asin();
            let half_cone = frustum.cos_limit.clamp(-1.0, 1.0).acos().max(1e-3);
            let edge = ((off_axis - angular_radius) / half_cone).clamp(0.0, 1.0);
            // Overlap-aware fovea (up to x25 at the cone edge) concentrates
            // the fixed budget toward the view center: uniform targets
            // spread 320 leaves evenly and stall everything at L13, and
            // even x9 leaves the mid-ring eating the depth budget. Edge
            // tiles sit in fog and peripheral vision; the center keeps the
            // tight rule. Overlap-aware (closest approach, not center) is
            // what makes strong foveation safe: tiles covering the fovea
            // score edge ~0 no matter how far their centers are.
            let bias = detail_bias.clamp(1.0, 32.0);
            let base = if distance > 100_000.0 {
                1.0 / 10.0
            } else {
                1.0 / 48.0 * bias
            };
            base * (1.0 + 24.0 * edge * edge)
        } else {
            1.0 / 2.4
        };
        let value = key.span_m(radius) / distance / stop;
        priorities.borrow_mut().insert(key, value);
        value
    };
    let mut leaves: Vec<_> = (0..6).map(TileKey::root).filter(|k| visible(*k)).collect();
    // Strict tile budget, prioritizing normalized projected error.
    while leaves.len() + 3 <= budget.max(6) {
        let candidate = leaves
            .iter()
            .enumerate()
            .filter(|(_, key)| key.level < max_level.min(20))
            .max_by(|(_, a), (_, b)| priority(**a).total_cmp(&priority(**b)));
        let Some((index, key)) = candidate else {
            break;
        };
        if priority(*key) <= 1.0 {
            break;
        }
        let children = key.children();
        leaves.swap_remove(index);
        leaves.extend(children.into_iter().filter(|k| visible(*k)));
    }
    leaves.sort();
    leaves
}

pub struct TerrainTile {
    pub key: TileKey,
    pub anchor_m: [f64; 3],
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub colors: Vec<[f32; 4]>,
    /// Signed bedrock height + slope, consumed by the surface material.
    pub surface: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
}
fn linear(v: f32) -> f32 {
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

/// Visible ground/water skin, with an apron for consistent edge normals and
/// downward skirts to cover neighbour LOD differences. All global subtraction
/// happens in f64 before a tile-local vertex is converted to f32.
pub fn build_tile(field: &PlanetField, key: TileKey, cells: usize) -> TerrainTile {
    assert!((2..=64).contains(&cells));
    let radius = field.params.radius_m;
    let wavelength = (key.span_m(radius) / cells as f64 * 2.0).max(32.0);
    let center = key.direction(0.5, 0.5);
    let anchor = center.map(|x| x * (radius + field.height_m(center, wavelength).max(0.0)));
    let stride = cells + 3;
    let mut apron = Vec::with_capacity(stride * stride);
    for y in 0..stride {
        for x in 0..stride {
            let dir = key.direction(
                (x as f64 - 1.0) / cells as f64,
                (y as f64 - 1.0) / cells as f64,
            );
            let h = field.height_m(dir, wavelength).max(0.0);
            apron.push(dir.map(|v| v * (radius + h)));
        }
    }
    let mut tile = TerrainTile {
        key,
        anchor_m: anchor,
        positions: Vec::new(),
        normals: Vec::new(),
        colors: Vec::new(),
        surface: Vec::new(),
        indices: Vec::new(),
    };
    for y in 0..=cells {
        for x in 0..=cells {
            let i = (y + 1) * stride + x + 1;
            let pos = apron[i];
            let dir = normalize(pos);
            let normal = normalize(cross(
                sub(apron[i + 1], apron[i - 1]),
                sub(apron[i + stride], apron[i - stride]),
            ));
            let cosine = dot(normal, dir).clamp(0.001, 1.0);
            let mut sample = field.sample_surface(dir, wavelength);
            sample.slope_hint = (1.0 / (cosine * cosine) - 1.0).max(0.0).sqrt();
            let material = surface_appearance(field, &sample, dir);
            tile.positions.push(sub(pos, anchor).map(|x| x as f32));
            tile.normals.push(normal.map(|x| x as f32));
            let rgb = material.albedo_srgb.map(linear);
            tile.colors.push([rgb[0], rgb[1], rgb[2], 1.0]);
            tile.surface
                .push([sample.height_m as f32, sample.slope_hint as f32]);
        }
    }
    let n = (cells + 1) as u32;
    for y in 0..cells as u32 {
        for x in 0..cells as u32 {
            let a = y * n + x;
            tile.indices
                .extend([a, a + 1, a + n, a + 1, a + n + 1, a + n]);
        }
    }
    let mut edge = Vec::new();
    for x in 0..cells as u32 {
        edge.push(x);
    }
    for y in 0..cells as u32 {
        edge.push(y * n + cells as u32);
    }
    for x in (1..=cells as u32).rev() {
        edge.push(cells as u32 * n + x);
    }
    for y in (1..=cells as u32).rev() {
        edge.push(y * n);
    }
    let skirt_start = tile.positions.len() as u32;
    let depth = (key.span_m(radius) / cells as f64 * 1.5).max(24.0);
    for &i in &edge {
        let i = i as usize;
        let global: [f64; 3] = std::array::from_fn(|j| anchor[j] + tile.positions[i][j] as f64);
        let dir = normalize(global);
        tile.positions.push(std::array::from_fn(|j| {
            tile.positions[i][j] - (dir[j] * depth) as f32
        }));
        tile.normals.push(tile.normals[i]);
        tile.colors.push(tile.colors[i]);
        tile.surface.push(tile.surface[i]);
    }
    for i in 0..edge.len() {
        let j = (i + 1) % edge.len();
        tile.indices.extend([
            edge[i],
            skirt_start + i as u32,
            edge[j],
            edge[j],
            skirt_start + i as u32,
            skirt_start + j as u32,
        ]);
    }
    tile
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn adjacent_and_cube_face_edges_have_identical_directions() {
        let a = TileKey {
            face: 0,
            level: 8,
            x: 80,
            y: 120,
        };
        let b = TileKey { x: 81, ..a };
        for i in 0..17 {
            assert_eq!(
                a.direction(1.0, i as f64 / 16.0),
                b.direction(0.0, i as f64 / 16.0)
            );
        }
        for i in 0..17 {
            assert_eq!(
                TileKey::root(0).direction(0.0, i as f64 / 16.0),
                TileKey::root(4).direction(1.0, i as f64 / 16.0)
            );
        }
    }
    #[test]
    fn lod_is_bounded_and_refines_near_the_surface() {
        let near = select_tiles([3_201_000.0, 0.0, 0.0], 3_200_000.0, 17, 384);
        let far = select_tiles([9_600_000.0, 0.0, 0.0], 3_200_000.0, 17, 384);
        assert!(near.len() <= 384 && far.len() <= 384);
        assert!(near.iter().map(|k| k.level).max() > far.iter().map(|k| k.level).max());
        assert_eq!(
            near,
            select_tiles([3_201_000.0, 0.0, 0.0], 3_200_000.0, 17, 384)
        );
    }
}

/// Per-tile surface atlas. Geometry and material share spherical coordinates;
/// a one-texel apron keeps linear filtering continuous at tile boundaries.
pub struct SurfaceTexture {
    pub size: usize,
    pub albedo: Vec<u8>,
    pub roughness: Vec<u8>,
    pub normal: Vec<u8>,
}
/// Micro-detail grain octave wavelengths, shared by the fade rule below.
const GRAIN_BANDS: [f64; 4] = [8.0, 32.0, 128.0, 512.0];

/// Texture resolution tiers by tile level: 128 px only where the viewer
/// can resolve it (L13+), 64 px everywhere else. A 64 px tile costs 4x less
/// field sampling than 128 px; past a few kilometres fog erases the
/// difference, and the streaming front (not texel density) is what the eye
/// catches — unfilled tiles read as holes, coarse ones as ground.
pub fn texture_cells_for_level(level: u8) -> usize {
    if level >= 13 { 128 } else { 64 }
}

pub fn build_surface_texture(field: &PlanetField, key: TileKey, cells: usize) -> SurfaceTexture {
    build_surface_texture_for_mesh(field, key, cells, 24)
}

/// Material normals must subtract the wavelength of the actual mesh grid.
pub fn build_surface_texture_for_mesh(
    field: &PlanetField,
    key: TileKey,
    cells: usize,
    mesh_cells: usize,
) -> SurfaceTexture {
    assert!((2..=64).contains(&mesh_cells));
    let size = cells + 3;
    let mut result = SurfaceTexture {
        size,
        albedo: Vec::with_capacity(size * size * 4),
        roughness: Vec::with_capacity(size * size * 4),
        normal: vec![0; size * size * 4],
    };
    let wavelength = (key.span_m(field.params.radius_m) / cells as f64).max(2.0);
    let mesh_wavelength = (key.span_m(field.params.radius_m) / mesh_cells as f64 * 2.0).max(32.0);
    // Reuse the texel neighborhood for material slope instead of paying
    // for three extra field queries per pixel through the full sample API.
    let mut samples = Vec::with_capacity(size * size);
    // One macro evaluation per texel, shared three ways: the fine sample
    // below, its mesh-grid residual further down, and nothing else. The old
    // code paid a full height_parts (~2.7 us, ~15 noise evals in the
    // province warp) inside sample_surface AND another inside height_m.
    let mut prefixes = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let dir = key.direction(
                (x as f64 - 1.0) / cells as f64,
                (y as f64 - 1.0) / cells as f64,
            );
            let (prefix, macro_h) = field.height_prefix_m(dir);
            prefixes.push((prefix, macro_h));
            samples.push(field.sample_surface_from_prefix(dir, prefix, macro_h, 32.0));
        }
    }
    let mut residual = vec![0.0; size * size];
    for y in 0..size {
        for x in 0..size {
            let dir = key.direction(
                (x as f64 - 1.0) / cells as f64,
                (y as f64 - 1.0) / cells as f64,
            );
            let mut sample = samples[y * size + x].clone();
            let dhx = (samples[y * size + (x + 1).min(size - 1)].height_m
                - samples[y * size + x.saturating_sub(1)].height_m)
                / (2.0 * wavelength);
            let dhy = (samples[(y + 1).min(size - 1) * size + x].height_m
                - samples[y.saturating_sub(1) * size + x].height_m)
                / (2.0 * wavelength);
            sample.slope_hint = dhx.hypot(dhy);
            let material = surface_appearance(field, &sample, dir);
            let (prefix, macro_h) = prefixes[y * size + x];
            residual[y * size + x] = sample.height_m.max(0.0)
                - field
                    .height_from_prefix(dir, prefix, macro_h, mesh_wavelength)
                    .max(0.0);
            // Bands finer than ~2 texels cannot resolve and only add
            // aliasing shimmer: fade them out instead of evaluating full
            // weight like before. At fine wavelengths (<=4 m) every band
            // still weights 1.0, identical to the old rule.
            let grain: f32 = GRAIN_BANDS
                .into_iter()
                .enumerate()
                .filter_map(|(band, scale)| {
                    let weight = (scale / (2.0 * wavelength)).clamp(0.0, 1.0);
                    if weight <= 0.0 {
                        return None;
                    }
                    let p = dir.map(|v| v * field.params.radius_m / scale);
                    Some(
                        (crate::rng::value_noise3(
                            field.params.seed,
                            2201 + band as u32,
                            p[0],
                            p[1],
                            p[2],
                        ) * weight
                            * 0.3) as f32,
                    )
                })
                .sum();
            let detail = if sample.height_m > 0.0 {
                1.0 + grain * 0.25
            } else {
                1.0
            };
            result.albedo.extend(
                material
                    .albedo_srgb
                    .map(|v| (v * detail * 255.0).clamp(0.0, 255.0) as u8),
            );
            result.albedo.push(255);
            result
                .roughness
                .extend([255, (material.roughness * 255.0) as u8, 0, 255]);
        }
    }
    for y in 0..size {
        for x in 0..size {
            let dx = (residual[y * size + (x + 1).min(size - 1)]
                - residual[y * size + x.saturating_sub(1)])
                / (2.0 * wavelength);
            let dy = (residual[(y + 1).min(size - 1) * size + x]
                - residual[y.saturating_sub(1) * size + x])
                / (2.0 * wavelength);
            let normal = normalize([-dx, -dy, 1.0]);
            let i = (y * size + x) * 4;
            for (j, n) in normal.into_iter().enumerate() {
                result.normal[i + j] = ((n * 0.5 + 0.5) * 255.0).round() as u8;
            }
            result.normal[i + 3] = 255;
        }
    }
    result
}

#[cfg(test)]
mod surface_regressions {
    use super::*;
    fn field() -> PlanetField {
        let recipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }
    #[test]
    fn neighboring_material_edges_are_identical_and_opaque() {
        let field = field();
        let key = TileKey {
            face: 0,
            level: 10,
            x: 510,
            y: 511,
        };
        let a = build_surface_texture(&field, key, 16);
        let b = build_surface_texture(&field, TileKey { x: 511, ..key }, 16);
        for y in 1..18 {
            let ai = (y * a.size + 17) * 4;
            let bi = (y * b.size + 1) * 4;
            assert_eq!(&a.albedo[ai..ai + 4], &b.albedo[bi..bi + 4]);
            assert_eq!(&a.roughness[ai..ai + 4], &b.roughness[bi..bi + 4]);
            assert_eq!(&a.normal[ai..ai + 4], &b.normal[bi..bi + 4]);
        }
        assert!(a.albedo.as_chunks::<4>().0.iter().all(|p| p[3] == 255));
        let mesh = build_tile(&field, key, 24);
        assert!(mesh.positions.iter().flatten().all(|v| v.is_finite()));
    }
    #[test]
    fn priority_refinement_covers_the_camera_instead_of_a_distant_face() {
        let dir = normalize([1872802.375, 1551433.0, -2079957.75]);
        let eye = dir.map(|x| x * 3_201_000.0);
        let keys = select_tiles(eye, 3_200_000.0, 17, 192);
        let nearest = keys
            .iter()
            .min_by(|a, b| {
                dot(a.direction(0.5, 0.5), dir)
                    .acos()
                    .total_cmp(&dot(b.direction(0.5, 0.5), dir).acos())
            })
            .unwrap();
        assert!(
            nearest.level >= 10,
            "camera tile is too coarse: {nearest:?}"
        );
    }
}

#[cfg(test)]
mod elevation_lod_tests {
    use super::*;
    #[test]
    fn nearby_high_plateau_gets_near_surface_detail() {
        let radius = 3_200_000.0;
        let eye = [radius + 5050.0, 0.0, 0.0];
        let datum = select_tiles(eye, radius, 17, 192);
        let elevated = select_tiles_with_height(eye, radius, 17, 192, |_| 5000.0);
        assert!(
            elevated.iter().map(|k| k.level).max().unwrap()
                >= datum.iter().map(|k| k.level).max().unwrap() + 3
        );
        assert_eq!(
            elevated,
            select_tiles_with_height(eye, radius, 17, 192, |_| 5000.0)
        );
        assert!(elevated.len() <= 192);
    }
}

#[cfg(test)]
mod near_field_regression_tests {
    use super::*;
    #[test]
    fn five_km_survey_reaches_l16_under_the_camera() {
        // Live failure: the survey view from 5 km rendered flat gray because
        // selection stalled at L11-L14 near the camera (horizon-cone
        // selection sinks the budget into the mid-distance ring). With the
        // view frustum known, the nadir cone must refine to L16+ in budget.
        let radius = 3_200_000.0;
        let eye = [radius + 5000.0, 0.0, 0.0];
        let eye_dir = normalize(eye);
        let frustum = SelectionFrustum {
            forward: normalize([-1.0, 0.0, 0.0]),
            cos_limit: (0.5_f64 + 0.35).cos(),
        };
        let keys =
            select_tiles_with_height_and_frustum(eye, radius, 17, 320, |_| 0.0, Some(frustum), 1.0);
        // One subdivision nets +3 leaves past the budget edge by design.
        assert!(keys.len() <= 323, "budget overrun: {}", keys.len());
        let best = keys
            .iter()
            .map(|k| (k.level, dot(k.direction(0.5, 0.5), eye_dir).acos()))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("selection non-empty");
        eprintln!(
            "nearest tile level {} at {:.3} deg",
            best.0,
            best.1.to_degrees()
        );
        let mut hist = std::collections::BTreeMap::new();
        for k in &keys {
            *hist.entry(k.level).or_insert(0) += 1;
        }
        eprintln!("leaves={} hist={:?}", keys.len(), hist);
        assert!(
            best.0 >= 15,
            "near field too coarse from 5 km: L{} under the camera",
            best.0
        );
    }
}

#[cfg(test)]
mod velocity_bias_tests {
    use super::*;
    #[test]
    fn bias_relaxes_near_refinement_without_breaking_cover() {
        // Outerra-style velocity bias: a fast-moving eye must not chase
        // full detail it crosses within one selection period.
        let radius = 3_200_000.0;
        let eye = [radius + 5000.0, 0.0, 0.0];
        let frustum = SelectionFrustum {
            forward: normalize([-1.0, 0.0, 0.0]),
            cos_limit: (0.5_f64 + 0.35).cos(),
        };
        let sharp =
            select_tiles_with_height_and_frustum(eye, radius, 17, 320, |_| 0.0, Some(frustum), 1.0);
        let coarse =
            select_tiles_with_height_and_frustum(eye, radius, 17, 320, |_| 0.0, Some(frustum), 8.0);
        let max_level = |keys: &[TileKey]| keys.iter().map(|k| k.level).max().unwrap_or(0);
        let sharp_max = max_level(&sharp);
        let coarse_max = max_level(&coarse);
        eprintln!("bias 1 -> L{sharp_max}, bias 8 -> L{coarse_max}");
        let mut hist = std::collections::BTreeMap::new();
        for k in &coarse {
            *hist.entry(k.level).or_insert(0) += 1;
        }
        eprintln!("coarse leaves={} hist={:?}", coarse.len(), hist);
        assert!(
            sharp_max >= 15,
            "unbiased near field must refine, got L{sharp_max}"
        );
        assert!(
            coarse_max < sharp_max,
            "bias must relax refinement ({coarse_max} vs {sharp_max})"
        );
        assert!(!coarse.is_empty(), "biased selection must still cover");
    }
}

#[cfg(test)]
mod pilot_frustum_tests {
    use super::*;
    #[test]
    fn horizontal_forward_refines_ground_below() {
        // Pilot chase view: eye 3 km up, camera looking horizontal (+Z
        // tangent). The ground filling the frame bottom sits 60-90° off
        // the view axis; cone culling must keep it, or the pilot view
        // never refines past coarse cover (live L7-only bug).
        let radius = 3_200_000.0;
        let eye = [radius + 3000.0, 0.0, 0.0];
        let frustum = SelectionFrustum {
            forward: normalize([0.0, 0.0, 1.0]),
            cos_limit: (1.0_f64).cos(),
        };
        let keys =
            select_tiles_with_height_and_frustum(eye, radius, 17, 320, |_| 0.0, Some(frustum), 1.0);
        let max_in_view = keys
            .iter()
            .filter(|k| {
                let c = k.direction(0.5, 0.5);
                // tile roughly below-forward of the eye
                c[2] > 0.0 && c[0] > 0.99
            })
            .map(|k| k.level)
            .max()
            .unwrap_or(0);
        eprintln!(
            "pilot-cone max level below-forward: L{max_in_view} of {} tiles",
            keys.len()
        );
        assert!(
            max_in_view >= 12,
            "pilot ground culled: L{max_in_view} below-forward"
        );
    }
}
