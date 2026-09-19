//! Deterministic cube-sphere surface addresses and bounded camera LOD.
//! Tiles cache the field; they never define the terrain or own its random seed.
use crate::{
    appearance::{surface_appearance, surface_grain_height},
    field::PlanetField,
};
use serde::{Deserialize, Serialize};
use thessa_rcbt_core::{HeightPage, HeightPageError, Node};

/// The CBT domain adapter reserves three binary path bits for a cube face.
/// Six of the eight depth-three leaves are used by faces 0..=5; the two
/// remaining leaves are intentionally inert. A quadtree level appends an
/// `(x_bit, y_bit)` Morton pair, so a tile at level `L` maps to CBT depth
/// `3 + 2L` without a hash or a camera-dependent identifier.
pub const CBT_FACE_DEPTH: u8 = 3;

pub fn cbt_node_for_tile(key: TileKey) -> Option<Node> {
    if key.face >= 6 || CBT_FACE_DEPTH.checked_add(key.level.checked_mul(2)?)? > 58 {
        return None;
    }
    let mut id = (1_u64 << CBT_FACE_DEPTH) | key.face as u64;
    for bit in (0..key.level).rev() {
        id = (id << 1) | ((key.x as u64 >> bit) & 1);
        id = (id << 1) | ((key.y as u64 >> bit) & 1);
    }
    Node::new(id, CBT_FACE_DEPTH + key.level * 2).ok()
}

/// Return the cube tile represented by an even-depth CBT leaf. Odd-depth
/// leaves are valid binary topology but are only half-way through a quadtree
/// split and therefore have no complete tile address.
pub fn tile_for_cbt_node(node: Node) -> Option<TileKey> {
    if node.depth() < CBT_FACE_DEPTH || !(node.depth() - CBT_FACE_DEPTH).is_multiple_of(2) {
        return None;
    }
    let tile_level = (node.depth() - CBT_FACE_DEPTH) / 2;
    let path_bits = node.depth() - CBT_FACE_DEPTH;
    let path_mask = (1_u64 << path_bits).saturating_sub(1);
    let face = ((node.id() - (1_u64 << node.depth())) >> path_bits) as u8;
    if face >= 6 {
        return None;
    }
    let morton = (node.id() - (1_u64 << node.depth())) & path_mask;
    let mut x = 0_u32;
    let mut y = 0_u32;
    for bit in 0..tile_level {
        let shift = (tile_level - bit - 1) * 2;
        x = (x << 1) | ((morton >> (shift + 1)) & 1) as u32;
        y = (y << 1) | ((morton >> shift) & 1) as u32;
    }
    Some(TileKey {
        face,
        level: tile_level,
        x,
        y,
    })
}

/// Bake a compact non-negative surface-height page from the canonical field.
/// This matches the visible tile path, which clamps below-datum samples to
/// the spherical datum. The page has no address or renderer dependency; the
/// caller owns the `TileKey` next to the payload.
pub fn bake_height_page(
    field: &PlanetField,
    key: TileKey,
    grid_size: u32,
    max_error_m: f64,
) -> Result<HeightPage, HeightPageError> {
    let cells = grid_size.saturating_sub(1).max(1) as usize;
    let wavelength = (key.span_m(field.params.radius_m) / cells as f64).max(32.0);
    let grid = grid_size as usize;
    let samples = (0..grid)
        .flat_map(|y| {
            (0..grid).map(move |x| {
                let u = x as f64 / cells as f64;
                let v = y as f64 / cells as f64;
                field.height_m(key.direction(u, v), wavelength).max(0.0)
            })
        })
        .collect::<Vec<_>>();
    HeightPage::bake(&samples, grid_size, max_error_m)
}

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
    select_tiles_with_history(
        eye,
        radius,
        max_level,
        budget,
        height,
        frustum,
        detail_bias,
        &[],
    )
}

/// History-aware variant of [`select_tiles_with_height_and_frustum`].
///
/// A node that was previously split gets a lower split threshold, which keeps
/// small camera/distance oscillations from repeatedly replacing its children.
/// The previous state is used only as a deterministic priority bias; the
/// returned set is still rebuilt from the current eye and remains bounded by
/// `budget`.
pub fn select_tiles_with_history(
    eye: [f64; 3],
    radius: f64,
    max_level: u8,
    budget: usize,
    height: impl Fn([f64; 3]) -> f64,
    frustum: Option<SelectionFrustum>,
    detail_bias: f64,
    previous: &[TileKey],
) -> Vec<TileKey> {
    let eye_r = dot(eye, eye).sqrt();
    let eye_dir = normalize(eye);
    let horizon = (radius / eye_r.max(radius)).clamp(0.0, 1.0).acos();
    let previously_split: std::collections::BTreeSet<_> = previous
        .iter()
        .filter_map(|key| {
            let mut ancestor = key.parent()?;
            let mut ancestors = Vec::with_capacity(usize::from(key.level));
            ancestors.push(ancestor);
            while let Some(parent) = ancestor.parent() {
                ancestors.push(parent);
                ancestor = parent;
            }
            Some(ancestors)
        })
        .flatten()
        .collect();
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
        // Overlap-aware foveation focuses the fixed budget toward two useful
        // regions: the camera look direction and the nearby ground below a
        // pilot/chase camera. The minimum of the two edge scores avoids the
        // old failure mode where one horizontal cone refined the horizon but
        // left the terrain under the aircraft at cover LOD. The edge weight
        // is deliberately bounded; it changes selection priority, not the
        // physical height field or the authoritative surface query.
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
            // Keep two small foveas: the camera look direction for the
            // horizon/forward field, and the ray from the eye toward the
            // nearby surface for a chase/pilot view. A single horizontal
            // fovea makes the ground below the aircraft look like fog and
            // spends the fixed budget on distant tiles instead. The minimum
            // edge score protects both useful regions without changing the
            // conservative horizon visibility test.
            // Cap 32: beyond it coarser tiles churn harder at tile borders
            // than they save (measured U-curve at 10 km/s), so depth alone
            // is not the answer there — hysteresis is (separate change).
            let camera_edge = ((off_axis - angular_radius) / half_cone).clamp(0.0, 1.0);
            let ground_focal = eye_dir.map(|v| -v);
            let ground_axis = dot(to_tile, ground_focal).clamp(-1.0, 1.0).acos();
            let ground_edge = ((ground_axis - angular_radius) / 0.85).clamp(0.0, 1.0);
            let edge = camera_edge.min(ground_edge);
            let bias = detail_bias.clamp(1.0, 32.0);
            // Keep the visual ground under a fast pilot at full-ish detail;
            // spend the velocity reduction on the distant field instead.
            // Applying the full bias to every near tile made a 400 m/s flight
            // look like a low-resolution globe even though the fixed tile
            // budget still had enough candidates for L14/L15.
            let near_bias = 1.0 + (bias - 1.0) * 0.35;
            let base = if distance > 100_000.0 {
                1.0 / 10.0 * bias
            } else {
                1.0 / 48.0 * near_bias
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
            .max_by(|(_, a), (_, b)| {
                let a_threshold = if previously_split.contains(a) {
                    0.8
                } else {
                    1.0
                };
                let b_threshold = if previously_split.contains(b) {
                    0.8
                } else {
                    1.0
                };
                (priority(**a) / a_threshold).total_cmp(&(priority(**b) / b_threshold))
            });
        let Some((index, key)) = candidate else {
            break;
        };
        let threshold = if previously_split.contains(key) {
            0.8
        } else {
            1.0
        };
        if priority(*key) / threshold <= 1.0 {
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
    pub height_page: HeightPage,
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
    let mut height_samples = Vec::with_capacity((cells + 1) * (cells + 1));
    for y in 0..=cells {
        for x in 0..=cells {
            let pos = apron[(y + 1) * stride + x + 1];
            height_samples.push(
                (pos.iter().map(|value| value * value).sum::<f64>().sqrt() - radius).max(0.0),
            );
        }
    }
    let height_page = HeightPage::bake(&height_samples, (cells + 1) as u32, 0.5)
        .expect("tile height page must fit its error budget");
    let mut tile = TerrainTile {
        key,
        anchor_m: anchor,
        height_page,
        // Exact counts: no reallocation churn on the worker pool.
        positions: Vec::with_capacity((cells + 1) * (cells + 1) + 4 * cells),
        normals: Vec::with_capacity((cells + 1) * (cells + 1) + 4 * cells),
        colors: Vec::with_capacity((cells + 1) * (cells + 1) + 4 * cells),
        surface: Vec::with_capacity((cells + 1) * (cells + 1) + 4 * cells),
        indices: Vec::with_capacity(cells * cells * 6 + 4 * cells * 6),
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
    let mut edge = Vec::with_capacity(4 * cells);
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
    // A coarse tile used to extrude its skirt by the full cell width. At the
    // horizon that produced kilometre-deep walls: they hid gaps, but also
    // projected dark LOD seams into the sun-shadow map. Keep enough cover for
    // the neighbouring-level height delta while bounding the shadow-casting
    // geometry to a local apron.
    let depth = (key.span_m(radius) / cells as f64 * 1.5).clamp(24.0, 256.0);
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

    #[test]
    fn history_same_view_is_stable() {
        let eye = [3_201_000.0, 0.0, 0.0];
        let first = select_tiles_with_history(eye, 3_200_000.0, 17, 192, |_| 0.0, None, 1.0, &[]);
        let second =
            select_tiles_with_history(eye, 3_200_000.0, 17, 192, |_| 0.0, None, 1.0, &first);
        assert_eq!(first, second);
    }

    #[test]
    fn history_reduces_small_oscillation_churn() {
        use std::collections::BTreeSet;
        let eye = |y| [3_201_000.0, y, 0.0];
        let positions = [0.0, 2.0, -2.0, 2.0, -2.0];
        let stateless: Vec<_> = positions
            .iter()
            .map(|&y| select_tiles(eye(y), 3_200_000.0, 17, 192))
            .collect();
        let mut history = Vec::new();
        let mut previous = Vec::new();
        for &y in &positions {
            previous = select_tiles_with_history(
                eye(y),
                3_200_000.0,
                17,
                192,
                |_| 0.0,
                None,
                1.0,
                &previous,
            );
            history.push(previous.clone());
        }
        let churn = |sets: &[Vec<TileKey>]| {
            sets.windows(2)
                .map(|pair| {
                    let a: BTreeSet<_> = pair[0].iter().copied().collect();
                    let b: BTreeSet<_> = pair[1].iter().copied().collect();
                    a.symmetric_difference(&b).count()
                })
                .sum::<usize>()
        };
        assert!(churn(&history) <= churn(&stateless));
    }

    #[test]
    fn history_large_view_change_remains_bounded_and_refines_new_side() {
        let first = select_tiles_with_history(
            [3_201_000.0, 0.0, 0.0],
            3_200_000.0,
            17,
            192,
            |_| 0.0,
            None,
            1.0,
            &[],
        );
        let second = select_tiles_with_history(
            [0.0, 3_201_000.0, 0.0],
            3_200_000.0,
            17,
            192,
            |_| 0.0,
            None,
            1.0,
            &first,
        );
        assert!(second.len() <= 195);
        for (i, a) in second.iter().enumerate() {
            for b in &second[i + 1..] {
                assert!(!is_ancestor(*a, *b) && !is_ancestor(*b, *a));
            }
        }
        let new_dir = normalize([0.0, 1.0, 0.0]);
        assert!(
            second
                .iter()
                .filter(|key| dot(key.direction(0.5, 0.5), new_dir) > 0.99)
                .map(|key| key.level)
                .max()
                .unwrap_or(0)
                >= 10
        );
    }

    fn is_ancestor(ancestor: TileKey, descendant: TileKey) -> bool {
        ancestor.face == descendant.face
            && ancestor.level <= descendant.level
            && (ancestor.x == descendant.x >> (descendant.level - ancestor.level))
            && (ancestor.y == descendant.y >> (descendant.level - ancestor.level))
    }

    #[test]
    fn cbt_morton_mapping_round_trips_cube_tiles() {
        for key in [
            TileKey::root(0),
            TileKey {
                face: 5,
                level: 1,
                x: 1,
                y: 0,
            },
            TileKey {
                face: 2,
                level: 7,
                x: 93,
                y: 41,
            },
        ] {
            let node = cbt_node_for_tile(key).expect("valid cube tile CBT address");
            assert_eq!(tile_for_cbt_node(node), Some(key));
        }
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

/// Fixed-size material page consumed by the GPU texture-array path.
///
/// The RGB channels are the canonical surface albedo in sRGB bytes. Alpha is
/// the canonical perceptual roughness encoded from its linear 0..1 value.
/// Pages are always 128x128: texels 1..=126 cover the 125-cell tile span and
/// the outer texels form a one-texel sampling border. The raster shader maps a
/// tile-local coordinate `local_uv` with
/// `(1.5 + local_uv * 125.0) / 128.0`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpuMaterialPage {
    pub size: u32,
    pub rgba: Vec<u8>,
}

pub const GPU_MATERIAL_PAGE_SIZE: u32 = 128;
const GPU_MATERIAL_PAGE_CELLS: usize = 125;
// Material classification must not depend on the page's LOD.  This is long
// enough to reject unresolved grain while retaining the slope that drives
// rock/vegetation/snow transitions.
const MATERIAL_SLOPE_WAVELENGTH_M: f64 = 256.0;

/// Build the fixed 128x128 material page for one canonical cube-sphere tile.
///
/// This deliberately shares the surface prefix path with
/// [`build_surface_texture_for_mesh`], but does not build mesh positions,
/// normals, residuals, or a second fine-field sample. A single canonical
/// prefix sample per texel is paired with a fixed-scale material slope, so the
/// result is independent of tile LOD. The border samples use the same field
/// coordinates as adjacent pages, so filtering across tile edges is
/// continuous.
pub fn build_gpu_material_page(field: &PlanetField, key: TileKey) -> GpuMaterialPage {
    let size = GPU_MATERIAL_PAGE_SIZE as usize;
    let cells = GPU_MATERIAL_PAGE_CELLS;
    let mut dirs = Vec::with_capacity(size * size);
    let mut samples = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            // Texture centres are x+0.5; inverting the shader transform puts
            // the first interior centre at local_uv=0 and the last at 1.
            let local_u = (x as f64 - 1.0) / cells as f64;
            let local_v = (y as f64 - 1.0) / cells as f64;
            let dir = key.direction(local_u, local_v);
            // Evaluate the canonical prefix at the actual direction.  A
            // tile-local interpolated prefix changes phase at LOD boundaries
            // and produces square blocks when a page is sampled beside a
            // page from another level.  This is one prefix evaluation per
            // texel; the fixed-scale slope uses three lightweight base-height
            // evaluations and does not run five full semantic samples.
            let (prefix, macro_h) = field.height_prefix_m(dir);
            dirs.push(dir);
            samples.push(field.sample_surface_from_prefix(
                dir,
                prefix,
                macro_h,
                TEXTURE_DETAIL_MIN_WL_M,
            ));
        }
    }

    let mut rgba = Vec::with_capacity(size * size * 4);
    for y in 0..size {
        for x in 0..size {
            let index = y * size + x;
            let slope = field.slope_hint(dirs[index], MATERIAL_SLOPE_WAVELENGTH_M);
            let material = {
                let sample = &mut samples[index];
                sample.slope_hint = slope;
                surface_appearance(field, sample, dirs[index])
            };
            rgba.extend(
                material
                    .albedo_srgb
                    .map(|channel| (channel.clamp(0.0, 1.0) * 255.0).round() as u8),
            );
            rgba.push((material.roughness.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
    }
    GpuMaterialPage {
        size: GPU_MATERIAL_PAGE_SIZE,
        rgba,
    }
}

/// Macro prefix evaluated on a coarse grid (`step` texels) covering the
/// tile plus one node past each far edge, so every texel bilinearly
/// interpolates between bracketing nodes with uniform weights.
fn coarse_prefix_grid(
    field: &PlanetField,
    key: TileKey,
    cells: usize,
    size: usize,
    step: usize,
) -> (Vec<(f64, f64)>, usize) {
    let grid = (size - 1) / step + 2;
    let mut prefix_grid = Vec::with_capacity(grid * grid);
    for gy in 0..grid {
        for gx in 0..grid {
            let dir = key.direction(
                ((gx * step) as f64 - 1.0) / cells as f64,
                ((gy * step) as f64 - 1.0) / cells as f64,
            );
            prefix_grid.push(field.height_prefix_m(dir));
        }
    }
    (prefix_grid, grid)
}

/// Bilinear sample of a [`coarse_prefix_grid`] at texel `(x, y)`.
/// Returns `(prefix_m, macro_h)`.
fn sample_prefix_grid(
    prefix_grid: &[(f64, f64)],
    grid: usize,
    step: usize,
    x: usize,
    y: usize,
) -> (f64, f64) {
    let gx = (x / step).min(grid - 2);
    let gy = (y / step).min(grid - 2);
    let tx = (x - gx * step) as f64 / step as f64;
    let ty = (y - gy * step) as f64 / step as f64;
    let (p00, m00) = prefix_grid[gy * grid + gx];
    let (p10, m10) = prefix_grid[gy * grid + gx + 1];
    let (p01, m01) = prefix_grid[(gy + 1) * grid + gx];
    let (p11, m11) = prefix_grid[(gy + 1) * grid + gx + 1];
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    (
        lerp(lerp(p00, p10, tx), lerp(p01, p11, tx), ty),
        lerp(lerp(m00, m10, tx), lerp(m01, m11, tx), ty),
    )
}

/// Fine-sample cutoff shared by the texture height and the mesh-grid
/// residual below. When `mesh_wavelength` floors to this value both
/// evaluations run identical detail bands, so the residual is exactly
/// zero and the second evaluation is skipped (pinned by
/// `deep_tile_material_normals_are_flat`).
const TEXTURE_DETAIL_MIN_WL_M: f64 = 32.0;

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
    let mesh_wavelength =
        (key.span_m(field.params.radius_m) / mesh_cells as f64 * 2.0).max(TEXTURE_DETAIL_MIN_WL_M);
    // Identical detail bands on both sides make the residual exactly zero
    // (verified by diagnostic, pinned by test): skip the second evaluation.
    let flat_residual = mesh_wavelength <= TEXTURE_DETAIL_MIN_WL_M;
    // Reuse the texel neighborhood for material slope instead of paying
    // for three extra field queries per pixel through the full sample API.
    let mut samples = Vec::with_capacity(size * size);
    // The macro prefix is smooth by construction (features, provinces and
    // uplift without meso/micro bands), so it is evaluated on a coarse grid
    // and bilinearly interpolated: ~16x fewer ~4 us macro evaluations per
    // tile. The interpolation error is bounded by test against direct
    // evaluation; the mesh-grid residual below cancels the shared prefix
    // term exactly, so only height classification near thresholds can shift,
    // within the pinned centimetre bound.
    const PREFIX_STEP: usize = 4;
    let (prefix_grid, grid) = coarse_prefix_grid(field, key, cells, size, PREFIX_STEP);
    let at = |x: usize, y: usize| sample_prefix_grid(&prefix_grid, grid, PREFIX_STEP, x, y);
    // Directions are pure function of (x, y): compute once, reuse in the
    // residual loop below instead of re-normalizing per texel.
    let mut dirs = Vec::with_capacity(size * size);
    let mut grain_heights = Vec::with_capacity(size * size);
    for y in 0..size {
        for x in 0..size {
            let dir = key.direction(
                (x as f64 - 1.0) / cells as f64,
                (y as f64 - 1.0) / cells as f64,
            );
            dirs.push(dir);
            grain_heights.push(surface_grain_height(field, dir, wavelength));
            let (prefix, macro_h) = at(x, y);
            samples.push(field.sample_surface_from_prefix(
                dir,
                prefix,
                macro_h,
                TEXTURE_DETAIL_MIN_WL_M,
            ));
        }
    }
    let mut residual = vec![0.0; size * size];
    for y in 0..size {
        for x in 0..size {
            let dir = dirs[y * size + x];
            let mut sample = samples[y * size + x].clone();
            let dhx = (samples[y * size + (x + 1).min(size - 1)].height_m
                - samples[y * size + x.saturating_sub(1)].height_m)
                / (2.0 * wavelength);
            let dhy = (samples[(y + 1).min(size - 1) * size + x].height_m
                - samples[y.saturating_sub(1) * size + x].height_m)
                / (2.0 * wavelength);
            sample.slope_hint = dhx.hypot(dhy);
            let material = surface_appearance(field, &sample, dir);
            // Same interpolated prefix as the fine sample above: the shared
            // term cancels in the residual, isolating mesh-grid detail.
            // At or below the detail cutoff both sides run identical bands
            // and the residual is exactly zero (see above).
            let (prefix, macro_h) = at(x, y);
            residual[y * size + x] = if flat_residual {
                0.0
            } else {
                sample.height_m.max(0.0)
                    - field
                        .height_from_prefix(dir, prefix, macro_h, mesh_wavelength)
                        .max(0.0)
            };
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
            let grain_dx = (grain_heights[y * size + (x + 1).min(size - 1)]
                - grain_heights[y * size + x.saturating_sub(1)])
                / (2.0 * wavelength);
            let grain_dy = (grain_heights[(y + 1).min(size - 1) * size + x]
                - grain_heights[y.saturating_sub(1) * size + x])
                / (2.0 * wavelength);
            let normal = normalize([-dx - grain_dx, -dy - grain_dy, 1.0]);
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
    fn gpu_material_page_has_fixed_layout_and_continuous_inner_edges() {
        let field = field();
        let key = TileKey {
            face: 0,
            level: 10,
            x: 510,
            y: 511,
        };
        let a = build_gpu_material_page(&field, key);
        let b = build_gpu_material_page(&field, TileKey { x: 511, ..key });
        assert_eq!(a.size, GPU_MATERIAL_PAGE_SIZE);
        assert_eq!(
            a.rgba.len(),
            (GPU_MATERIAL_PAGE_SIZE * GPU_MATERIAL_PAGE_SIZE * 4) as usize
        );
        // The inner endpoint of a is x=126 (local_u=1), and the inner start
        // of b is x=1 (local_u=0); the outer texels remain their padding.
        for y in 1..127 {
            let ai = (y * a.size as usize + 126) * 4;
            let bi = (y * b.size as usize + 1) * 4;
            assert_eq!(&a.rgba[ai..ai + 4], &b.rgba[bi..bi + 4], "y={y}");
        }
    }

    #[test]
    fn gpu_material_page_is_identical_on_parent_child_shared_edges() {
        let field = field();
        let parent = TileKey {
            face: 2,
            level: 9,
            x: 173,
            y: 211,
        };
        let child = TileKey {
            face: parent.face,
            level: parent.level + 1,
            x: parent.x * 2,
            y: parent.y * 2,
        };
        let a = build_gpu_material_page(&field, parent);
        let b = build_gpu_material_page(&field, child);
        let size = GPU_MATERIAL_PAGE_SIZE as usize;
        // The child's left edge covers the first half of the parent's left
        // edge. Their texel centres are the same directions at every other
        // parent row (including the apron endpoint).
        for y in 1..=63 {
            let ai = (y * size + 1) * 4;
            let bi = ((1 + 2 * (y - 1)) * size + 1) * 4;
            assert_eq!(&a.rgba[ai..ai + 4], &b.rgba[bi..bi + 4], "y={y}");
        }
    }

    #[test]
    fn gpu_material_page_preserves_canonical_ocean_and_land_appearance() {
        let field = field();
        let key = TileKey::root(0);
        let page = build_gpu_material_page(&field, key);
        let size = page.size as usize;
        let cells = 125.0;
        let mut ocean = None;
        let mut land = None;
        for y in 1..127 {
            for x in 1..127 {
                let dir = key.direction((x as f64 - 1.0) / cells, (y as f64 - 1.0) / cells);
                let mut sample = field.sample_surface(dir, TEXTURE_DETAIL_MIN_WL_M);
                if sample.height_m < -100.0 {
                    let material = surface_appearance(&field, &sample, dir);
                    if material.roughness < 0.3 {
                        ocean = Some((x, y, material));
                    }
                } else if sample.height_m > 1000.0 {
                    sample.slope_hint = field.slope_hint(dir, MATERIAL_SLOPE_WAVELENGTH_M);
                    land = Some((x, y, surface_appearance(&field, &sample, dir)));
                }
            }
        }
        let (ox, oy, ocean) = ocean.expect("reference page must contain warm ocean");
        let (lx, ly, land) = land.expect("reference page must contain land");
        let ocean_pixel = &page.rgba[(oy * size + ox) * 4..(oy * size + ox) * 4 + 4];
        let land_pixel = &page.rgba[(ly * size + lx) * 4..(ly * size + lx) * 4 + 4];
        assert_eq!(ocean_pixel[3], (ocean.roughness * 255.0).round() as u8);
        assert_eq!(land_pixel[3], (land.roughness * 255.0).round() as u8);
        for (actual, expected) in ocean_pixel[..3].iter().zip(
            ocean
                .albedo_srgb
                .into_iter()
                .map(|v| (v * 255.0).round() as u8),
        ) {
            assert!(
                (*actual as i16 - expected as i16).abs() <= 8,
                "ocean channel: actual={actual}, expected={expected}, at=({ox},{oy})"
            );
        }
        for (actual, expected) in land_pixel[..3].iter().zip(
            land.albedo_srgb
                .into_iter()
                .map(|v| (v * 255.0).round() as u8),
        ) {
            assert!(
                (*actual as i16 - expected as i16).abs() <= 8,
                "land channel: actual={actual}, expected={expected}, at=({lx},{ly})"
            );
        }
    }
    #[test]
    fn material_pages_carry_readable_close_up_variation() {
        // Regression pin for flat close terrain: snow used to cover as an
        // exact constant (std 0.0) and tundra varied by ~1 LSB. Both now
        // carry patch + micro-relief structure; this fails if appearance
        // ever collapses back to a fill.
        fn tile_containing(dir: [f64; 3], level: u8) -> TileKey {
            let ax = dir[0].abs();
            let ay = dir[1].abs();
            let az = dir[2].abs();
            let (face, u, v) = if ax >= ay && ax >= az {
                if dir[0] > 0.0 {
                    (0u8, -dir[2] / dir[0], dir[1] / dir[0])
                } else {
                    (1u8, dir[2] / -dir[0], dir[1] / -dir[0])
                }
            } else if ay >= ax && ay >= az {
                if dir[1] > 0.0 {
                    (2u8, dir[0] / dir[1], -dir[2] / dir[1])
                } else {
                    (3u8, dir[0] / -dir[1], dir[2] / -dir[1])
                }
            } else if dir[2] > 0.0 {
                (4u8, dir[0] / dir[2], dir[1] / dir[2])
            } else {
                (5u8, dir[0] / dir[2], dir[1] / dir[2])
            };
            let n = 1u64 << level;
            let q = |t: f64| (((t + 1.0) * 0.5 * n as f64) as u32).min(n as u32 - 1);
            TileKey {
                face,
                level,
                x: q(u),
                y: q(v),
            }
        }
        fn channel_stats(rgba: &[u8], channel: usize) -> (f64, u8, u8) {
            let n = rgba.len() / 4;
            let mean = rgba
                .chunks_exact(4)
                .map(|px| px[channel] as f64)
                .sum::<f64>()
                / n as f64;
            let var = rgba
                .chunks_exact(4)
                .map(|px| (px[channel] as f64 - mean).powi(2))
                .sum::<f64>()
                / n as f64;
            let min = rgba.chunks_exact(4).map(|px| px[channel]).min().unwrap();
            let max = rgba.chunks_exact(4).map(|px| px[channel]).max().unwrap();
            (var.sqrt(), min, max)
        }
        let field = field();
        // High snowfield (h ~= 2117 m) and low tundra (h ~= 217 m).
        for dir in [
            [
                -0.5735764363510462,
                -0.8191520442889918,
                7.024285468436542e-17,
            ],
            [
                -0.44575261109709685,
                -0.8191520442889918,
                0.36096334722141254,
            ],
        ] {
            let page = build_gpu_material_page(&field, tile_containing(dir, 14));
            let (std_r, _, _) = channel_stats(&page.rgba, 0);
            let (_, lo_b, hi_b) = channel_stats(&page.rgba, 2);
            eprintln!("dir={dir:?} std_r={std_r:.1} b_range={}", hi_b - lo_b);
            assert!(std_r > 3.0, "snow/tundra page must vary, std_r={std_r:.1}");
            assert!(
                hi_b - lo_b > 15,
                "snow/tundra page needs tonal range, b={lo_b}..{hi_b}"
            );
        }
    }
    #[test]
    fn deep_tile_material_normals_keep_grain_and_coarse_relief() {
        // At/below the detail cutoff the canonical residual is exactly zero.
        // The material layer nevertheless carries filtered, non-authoritative
        // grain normals so close ground does not render as a flat green sheet.
        // A coarse tile (mesh wavelength above the cutoff) must also keep real
        // field relief, guarding against an over-eager skip.
        let field = field();
        let deep = build_surface_texture_for_mesh(
            &field,
            TileKey {
                face: 1,
                level: 14,
                x: 100,
                y: 200,
            },
            128,
            32,
        );
        assert!(deep.normal.as_chunks::<4>().0.iter().all(|p| p[3] == 255));
        assert!(
            deep.normal
                .as_chunks::<4>()
                .0
                .iter()
                .any(|p| p != &[128, 128, 255, 255])
        );
        let coarse = build_surface_texture_for_mesh(
            &field,
            TileKey {
                face: 1,
                level: 10,
                x: 100,
                y: 200,
            },
            64,
            32,
        );
        assert!(
            coarse
                .normal
                .as_chunks::<4>()
                .0
                .iter()
                .any(|p| p != &[128, 128, 255, 255]),
            "coarse tile lost its relief normals"
        );
    }

    #[test]
    fn gpu_height_page_matches_the_canonical_tile_domain() {
        let field = field();
        let key = TileKey {
            face: 4,
            level: 12,
            x: 1777,
            y: 2041,
        };
        let page = bake_height_page(&field, key, 33, 0.5).expect("page quantization budget");
        assert_eq!(page.grid_size(), 33);
        assert!(page.max_residual_error_m() <= 0.5);
        for (u, v) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)] {
            let expected = field
                .height_m(
                    key.direction(u, v),
                    key.span_m(field.params.radius_m) / 32.0,
                )
                .max(0.0);
            assert!((f64::from(page.sample(u as f32, v as f32)) - expected).abs() <= 0.5);
        }
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
        let refinement_score =
            |keys: &[TileKey]| keys.iter().map(|key| u32::from(key.level)).sum::<u32>();
        let sharp_score = refinement_score(&sharp);
        let coarse_score = refinement_score(&coarse);
        eprintln!(
            "bias 1 -> L{sharp_max}, score {sharp_score}; bias 8 -> L{coarse_max}, score {coarse_score}"
        );
        assert!(
            sharp_max >= 15,
            "unbiased near field must refine, got L{sharp_max}"
        );
        assert!(
            coarse_score < sharp_score,
            "bias must relax aggregate refinement (score {coarse_score} vs {sharp_score})"
        );
        assert!(!coarse.is_empty(), "biased selection must still cover");
    }
}

#[cfg(test)]
mod pilot_frustum_tests {
    use super::*;
    #[test]
    fn horizontal_forward_refines_ground_below() {
        // Pilot chase view: eye 500 m up, camera looking horizontal (+Z
        // tangent). The ground filling the frame bottom sits 60-90° off
        // the view axis; cone culling must keep it, or the pilot view
        // never refines past coarse cover (live L7-only bug).
        let radius = 3_200_000.0;
        let eye = [radius + 500.0, 0.0, 0.0];
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
            max_in_view >= 14,
            "pilot ground culled: L{max_in_view} below-forward"
        );
    }
}

#[cfg(test)]
mod coarse_prefix_tests {
    use super::*;

    fn field() -> PlanetField {
        let recipe: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&recipe).unwrap())
            .unwrap()
    }

    #[test]
    fn coarse_prefix_interpolation_error_within_span_fraction() {
        // Bound vs tile span: bilerp of the smooth macro prefix converges
        // second-order (measured 1.9 m at L7 → 1e-5 m at L14 → 2e-7 at L17),
        // and the mesh-grid residual cancels the shared term, so only sea
        // classification inside this band can shift. span/20000 keeps cover
        // tiles 4e-5 off-span (invisible) and fine tiles sub-millimetre
        // (below f32 height quantization at kilometre radii).
        let field = field();
        for (level, cells) in [(7u8, 24usize), (10, 24), (14, 32), (17, 32)] {
            let mut worst = 0.0f64;
            for face in 0..6u8 {
                let key = TileKey {
                    face,
                    level,
                    x: 100,
                    y: 200,
                };
                let size = cells + 3;
                let (grid_vals, grid) = coarse_prefix_grid(&field, key, cells, size, 4);
                let mut x = 0;
                while x < size {
                    let mut y = 0;
                    while y < size {
                        let dir = key.direction(
                            (x as f64 - 1.0) / cells as f64,
                            (y as f64 - 1.0) / cells as f64,
                        );
                        let (direct, _) = field.height_prefix_m(dir);
                        let (interp, _) = sample_prefix_grid(&grid_vals, grid, 4, x, y);
                        worst = worst.max((direct - interp).abs());
                        y += 3;
                    }
                    x += 3;
                }
            }
            eprintln!("L{level}: coarse prefix worst error: {worst:.6e} m");
            let span = field.params.radius_m * 2.0 / (1u64 << level) as f64;
            assert!(
                worst <= span / 20000.0,
                "L{level} prefix error {worst:.6e} m over span {span:.0}"
            );
        }
    }
}

#[cfg(test)]
mod fast_track_demand_tests {
    use super::*;
    use std::collections::BTreeSet;

    fn field() -> PlanetField {
        let r: crate::spec_recipe::SpecRecipe =
            toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml")).unwrap();
        crate::field::field_from_manifest(&crate::spec_recipe::manifest_from_spec(&r).unwrap())
            .unwrap()
    }

    /// New-tile demand for one wall second of level flight at ~3.7 km/s
    /// (62 m per 60 Hz frame): size of the wanted set plus fresh tiles
    /// across 60 consecutive selections.
    fn demand_per_second(field: &PlanetField, altitude_m: f64, bias: f64) -> (usize, usize) {
        let radius = field.params.radius_m;
        let frustum = |_eye: [f64; 3]| SelectionFrustum {
            forward: normalize([-1.0, 0.0, 0.0]),
            cos_limit: 0.5,
        };
        let select = |eye: [f64; 3]| -> BTreeSet<TileKey> {
            select_tiles_with_height_and_frustum(
                eye,
                radius,
                17,
                288,
                |dir| field.height_m(dir, 32.0),
                Some(frustum(eye)),
                bias,
            )
            .into_iter()
            .collect()
        };
        let mut eye = [radius + altitude_m, 0.0, 0.0];
        let mut prev = select(eye);
        let wanted = prev.len();
        let mut total_new = 0;
        for _ in 0..60 {
            eye[1] += 62.0;
            let next = select(eye);
            total_new += next.difference(&prev).count();
            prev = next;
        }
        (wanted, total_new)
    }

    #[test]
    fn fast_low_flight_demand_fits_worker_throughput() {
        // Regression for >1 km/s surface flight: at the old client bias
        // cap (8x) a 500 m ground track at orbital speed demanded ~970
        // fresh tiles/s against ~200/s worker throughput (perpetual
        // catch-up, pop-in, holes). Bias 32 (the LOD-internal cap) drops
        // it into the serviceable range. Bounds carry ~2x margin over
        // calibration (10/s at 5 km, 174/s at 500 m).
        let field = field();
        let (wanted_hi, new_hi) = demand_per_second(&field, 5000.0, 32.0);
        assert!(wanted_hi <= 150, "wanted {wanted_hi} at 5 km");
        assert!(new_hi <= 60, "demand {new_hi}/s at 5 km");
        let (wanted_lo, new_lo) = demand_per_second(&field, 500.0, 32.0);
        assert!(wanted_lo <= 200, "wanted {wanted_lo} at 500 m");
        assert!(new_lo <= 300, "demand {new_lo}/s at 500 m");
    }

    #[test]
    fn velocity_bias_monotonically_reduces_demand() {
        // Structural: deepening the velocity bias must never grow the
        // wanted set or the fresh-tile demand at fixed speed/altitude.
        let field = field();
        for altitude_m in [500.0, 5000.0] {
            let (w8, n8) = demand_per_second(&field, altitude_m, 8.0);
            let (w32, n32) = demand_per_second(&field, altitude_m, 32.0);
            assert!(w32 <= w8, "alt {altitude_m}: wanted {w32} > {w8}");
            assert!(n32 <= n8, "alt {altitude_m}: demand {n32} > {n8}");
        }
    }
}
