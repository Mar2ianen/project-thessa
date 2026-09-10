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
    let eye_r = dot(eye, eye).sqrt();
    let eye_dir = normalize(eye);
    let horizon = (radius / eye_r.max(radius)).clamp(0.0, 1.0).acos();
    let mut queue: Vec<_> = (0..6).map(TileKey::root).collect();
    let mut leaves = Vec::new();
    while let Some(key) = queue.pop() {
        let center = key.direction(0.5, 0.5);
        let angular_bound = (2.0 / (1_u64 << key.level) as f64).min(std::f64::consts::PI);
        if dot(center, eye_dir)
            < (horizon + angular_bound + 0.10)
                .min(std::f64::consts::PI)
                .cos()
        {
            continue;
        }
        let delta = sub(eye, center.map(|x| x * radius));
        let distance = dot(delta, delta).sqrt();
        if key.level < max_level.min(20)
            && distance < key.span_m(radius) * 2.4
            && queue.len() + leaves.len() + 4 <= budget.max(6)
        {
            queue.extend(key.children());
        } else {
            leaves.push(key);
        }
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
