//! Canonical data addresses for persistent bisectors, independent of allocation
//! slots and render refinement. Shared-edge ownership is X, then Y, then Z.
use crate::{
    HeightPage,
    conforming::CUBE_SCALE,
    height_source::HeightPageStore,
    tile_anchor::{GpuTileFrame, TileAnchor, TileKey},
};

pub const FACE_DENOMINATOR: u32 = 2 * CUBE_SCALE as u32;
pub const SURFACE_PAGE_LEVEL_MAX: u8 = crate::tile_anchor::GPU_STABLE_LEVEL_MAX;
pub const PRECISION_LEVEL: u8 = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceAddress {
    pub face: u8,
    /// Exact numerator over FACE_DENOMINATOR, including the closed upper edge.
    pub uv: [u32; 2],
}
impl SurfaceAddress {
    pub fn from_cube(cube: [i64; 3]) -> Option<Self> {
        if cube.iter().any(|v| v.unsigned_abs() > CUBE_SCALE as u64) {
            return None;
        }
        let [x, y, z] = cube;
        let (face, a, b) = if x.abs() == CUBE_SCALE {
            if x >= 0 { (0, -z, y) } else { (1, z, y) }
        } else if y.abs() == CUBE_SCALE {
            if y >= 0 { (2, x, -z) } else { (3, x, z) }
        } else if z.abs() == CUBE_SCALE {
            if z >= 0 { (4, x, y) } else { (5, -x, y) }
        } else {
            return None;
        };
        Some(Self {
            face,
            uv: [(a + CUBE_SCALE) as u32, (b + CUBE_SCALE) as u32],
        })
    }
    pub fn tile(self, level: u8) -> Option<(TileKey, [f64; 2])> {
        if level > SURFACE_PAGE_LEVEL_MAX || self.uv.iter().any(|&v| v > FACE_DENOMINATOR) {
            return None;
        }
        let span = FACE_DENOMINATOR >> level;
        let xy = self.uv.map(|v| (v / span).min((1 << level) - 1));
        let local = std::array::from_fn(|i| f64::from(self.uv[i] - xy[i] * span) / f64::from(span));
        Some((TileKey::new(self.face, level, xy[0], xy[1])?, local))
    }
    pub fn sample(self, pages: &HeightPageStore) -> Option<(TileKey, &HeightPage, f64)> {
        for level in (0..=SURFACE_PAGE_LEVEL_MAX).rev() {
            let (tile, uv) = self.tile(level)?;
            if let Some(page) = pages.get(page_id(tile)?) {
                return Some((
                    tile,
                    page,
                    f64::from(page.sample(uv[0] as f32, uv[1] as f32)),
                ));
            }
        }
        None
    }
}

/// Existing material-directory identity: face root 8..13, then x/y bit pairs.
pub fn page_id(tile: TileKey) -> Option<u64> {
    TileKey::new(tile.face, tile.level, tile.x, tile.y)?;
    if tile.level > SURFACE_PAGE_LEVEL_MAX {
        return None;
    }
    let mut id = 8 + u64::from(tile.face);
    for bit in (0..tile.level).rev() {
        id = (id << 2) | u64::from(((tile.x >> bit) & 1) * 2 + ((tile.y >> bit) & 1));
    }
    Some(id)
}

/// Six camera-local precision charts reuse TileAnchor's f64 subtraction and
/// stable bend formula, independently of which height pages happen to reside.
/// All duplicate cube vertices select the same chart, even across root faces.
pub fn precision_charts(radius_m: f64, camera: [f64; 3]) -> Option<[(TileKey, GpuTileFrame); 6]> {
    if camera.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let mut charts = Vec::new();
    let [x, y, z] = camera;
    for face in 0..6 {
        let (a, b, axis) = match face {
            0 => (-z, y, x.abs()),
            1 => (z, y, x.abs()),
            2 => (x, -z, y.abs()),
            3 => (x, z, y.abs()),
            4 => (x, y, z.abs()),
            _ => (-x, y, z.abs()),
        };
        let coord = |v: f64| {
            if axis == 0.0 {
                0.5
            } else {
                (v / axis * 0.5 + 0.5).clamp(0.0, 1.0)
            }
        };
        let side = 1u32 << PRECISION_LEVEL;
        let tile = TileKey::new(
            face,
            PRECISION_LEVEL,
            ((coord(a) * f64::from(side)) as u32).min(side - 1),
            ((coord(b) * f64::from(side)) as u32).min(side - 1),
        )?;
        charts.push((tile, TileAnchor::new(tile, radius_m)?.to_gpu(camera)?));
    }
    charts.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Node,
        conforming::{SphereTopology, cube_vertices},
    };
    #[test]
    fn exact_addresses_match_canonical_projection_and_deep_samples() {
        let mut mesh = SphereTopology::new(1024, 58).unwrap();
        let mut node = Node::new(16, 4).unwrap();
        for _ in 4..58 {
            mesh.split(node).unwrap();
            node = node.left_child().unwrap();
        }
        for record in mesh.records().iter().filter(|r| r.address[0] != 0) {
            let n = Node::from_heap_id(
                u64::from(record.address[0]) | (u64::from(record.address[1]) << 32),
            )
            .unwrap();
            let Some(vertices) = cube_vertices(n) else {
                continue;
            };
            for cube in vertices {
                let a = SurfaceAddress::from_cube(cube).unwrap();
                let (tile, uv) = a.tile(22).unwrap();
                let p = TileAnchor::new(tile, 6_371_000.0)
                    .unwrap()
                    .project_body_m(uv, 0.0);
                let len = cube.iter().map(|v| (*v as f64).powi(2)).sum::<f64>().sqrt();
                for i in 0..3 {
                    assert!((p[i] - cube[i] as f64 / len * 6_371_000.0).abs() < 1e-8);
                }
            }
        }
        assert!(SurfaceAddress::from_cube([0; 3]).is_none());
    }
    #[test]
    fn residency_is_not_bisector_identity_and_boundaries_are_closed() {
        let mut pages = HeightPageStore::default();
        let root = TileKey::new(0, 0, 0, 0).unwrap();
        pages.set_page(
            page_id(root).unwrap(),
            HeightPage::bake(&[10.0; 4], 2, 0.01).unwrap(),
        );
        let a = SurfaceAddress::from_cube([CUBE_SCALE, CUBE_SCALE, -CUBE_SCALE]).unwrap();
        assert_eq!(a.face, 0);
        assert_eq!(a.tile(0).unwrap().1, [1.0, 1.0]);
        assert_eq!(a.sample(&pages).unwrap().2, 10.0);
        let (tile, _) = a.tile(22).unwrap();
        pages.set_page(
            page_id(tile).unwrap(),
            HeightPage::bake(&[20.0; 4], 2, 0.01).unwrap(),
        );
        assert_eq!(a.sample(&pages).unwrap().2, 20.0);
        pages.remove_page(page_id(tile).unwrap());
        assert_eq!(a.sample(&pages).unwrap().2, 10.0);
        assert!(precision_charts(1.0, [f64::NAN; 3]).is_none());
    }
}
