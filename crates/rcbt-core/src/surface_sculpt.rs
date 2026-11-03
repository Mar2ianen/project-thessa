//! Sparse, immutable-snapshot-friendly spherical sculpt height layer.
//!
//! This is an independent Rust implementation of sparse cube-face pages and
//! world-direction-evaluated angular stamps. The algorithmic comparison is
//! OpenEngine's MIT source at `Engine/Modules/CBTTerrain/Include/CBTTerrain/`
//! (`SphereSculptLayer.h` / `SphereSculptPaging.h`, revision bdf5c27); no
//! upstream source code or CBT kernel is copied here.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::tile_anchor::TileKey;

const PAGE_CELLS: u64 = 32;
const PAGE_SIDE: usize = PAGE_CELLS as usize + 1;
const PAGE_TEXELS: usize = PAGE_SIDE * PAGE_SIDE;
const PAGE_BYTES: usize = PAGE_TEXELS * std::mem::size_of::<f64>();
const MAX_CPU_LEVEL: u8 = 31;
const MIN_FACE_COMPONENT: f64 = 0.577_350_269_189_625_8;
const MAX_BRUSH_RADIUS_FRACTION: f64 = 0.01;

/// A bounded sparse additive height field on a radius-scaled cube-sphere grid.
///
/// Pages store 33×33 endpoint samples (32 cells plus shared edges). The map key
/// is the same face/level/x/y tuple carried by [`TileKey`], while the values
/// remain CPU-side f64 metres for canonical-field composition.
pub struct SurfaceSculpt {
    radius_m: f64,
    target_texel_m: f64,
    level: u8,
    max_pages: usize,
    pages: BTreeMap<(u8, u8, u32, u32), Vec<f64>>,
    revision: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SurfaceSculptError {
    InvalidRadius,
    InvalidTargetTexel,
    InvalidCapacity,
    InvalidDirection,
    InvalidBrushRadius,
    BrushTooSmall,
    PageLevelTooLarge,
    CandidateLimitExceeded,
    PageCapacityExceeded,
    NonFiniteResult,
    RevisionExhausted,
}

impl fmt::Display for SurfaceSculptError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRadius => f.write_str("planet radius must be finite and positive"),
            Self::InvalidTargetTexel => {
                f.write_str("target texel size must be finite and positive")
            }
            Self::InvalidCapacity => f.write_str("page capacity must be nonzero"),
            Self::InvalidDirection => f.write_str("direction must be finite and nonzero"),
            Self::InvalidBrushRadius => f.write_str("brush radius is outside the supported range"),
            Self::BrushTooSmall => {
                f.write_str("brush diameter must cover at least four target texels")
            }
            Self::PageLevelTooLarge => f.write_str("radius-scaled page level exceeds CPU limit 31"),
            Self::CandidateLimitExceeded => {
                f.write_str("brush page candidate bound exceeds page capacity")
            }
            Self::PageCapacityExceeded => f.write_str("brush would exceed allocated page capacity"),
            Self::NonFiniteResult => f.write_str("brush would produce a non-finite height"),
            Self::RevisionExhausted => f.write_str("sculpt revision counter exhausted"),
        }
    }
}

impl std::error::Error for SurfaceSculptError {}

impl SurfaceSculpt {
    /// Create a sparse sculpt grid sized for the body radius and requested
    /// projected face-cell spacing. `max_pages` bounds both candidate work and
    /// retained page payload; this CPU-only layer uses level 31 as its ceiling.
    pub fn new(
        radius_m: f64,
        target_texel_m: f64,
        max_pages: usize,
    ) -> Result<Self, SurfaceSculptError> {
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(SurfaceSculptError::InvalidRadius);
        }
        if !target_texel_m.is_finite() || target_texel_m <= 0.0 {
            return Err(SurfaceSculptError::InvalidTargetTexel);
        }
        if max_pages == 0 {
            return Err(SurfaceSculptError::InvalidCapacity);
        }
        let pages_ratio = radius_m / (16.0 * target_texel_m);
        let level = if pages_ratio <= 1.0 {
            0
        } else {
            let value = pages_ratio.log2().ceil();
            if !value.is_finite() || value > f64::from(MAX_CPU_LEVEL) {
                return Err(SurfaceSculptError::PageLevelTooLarge);
            }
            value as u8
        };
        Ok(Self {
            radius_m,
            target_texel_m,
            level,
            max_pages,
            pages: BTreeMap::new(),
            revision: 0,
        })
    }

    /// Add a smooth angular stamp centered on a unit direction. Input radius is
    /// metres on the body surface; zero-strength edits are a no-op.
    pub fn apply_dab(
        &mut self,
        center_unit: [f64; 3],
        brush_radius_m: f64,
        delta_m: f64,
    ) -> Result<Vec<TileKey>, SurfaceSculptError> {
        let center = normalize(center_unit).ok_or(SurfaceSculptError::InvalidDirection)?;
        if !brush_radius_m.is_finite()
            || brush_radius_m <= 0.0
            || brush_radius_m / self.radius_m > MAX_BRUSH_RADIUS_FRACTION
        {
            return Err(SurfaceSculptError::InvalidBrushRadius);
        }
        if !delta_m.is_finite() {
            return Err(SurfaceSculptError::NonFiniteResult);
        }
        if brush_radius_m < 2.0 * self.target_texel_m {
            return Err(SurfaceSculptError::BrushTooSmall);
        }
        if delta_m == 0.0 {
            return Ok(Vec::new());
        }
        let next_revision = self
            .revision
            .checked_add(1)
            .ok_or(SurfaceSculptError::RevisionExhausted)?;

        let angle = brush_radius_m / self.radius_m;
        let candidates = self.candidate_pages(center, angle)?;
        let mut planned = Vec::new();
        for key in candidates {
            let tile = tile_key(key);
            let mut values = self
                .pages
                .get(&key)
                .cloned()
                .unwrap_or_else(|| vec![0.0; PAGE_TEXELS]);
            let mut touched = false;
            for y in 0..PAGE_SIDE {
                for x in 0..PAGE_SIDE {
                    let direction = page_sample_direction(tile, x as u64, y as u64);
                    let cosine = dot(center, direction).clamp(-1.0, 1.0);
                    let sine = cross_length(center, direction);
                    let distance = sine.atan2(cosine);
                    if distance >= angle {
                        continue;
                    }
                    let t = (distance / angle).clamp(0.0, 1.0);
                    let smooth = t * t * (3.0 - 2.0 * t);
                    let index = y * PAGE_SIDE + x;
                    let value = values[index] + delta_m * (1.0 - smooth);
                    if !value.is_finite() {
                        return Err(SurfaceSculptError::NonFiniteResult);
                    }
                    values[index] = value;
                    touched = true;
                }
            }
            if touched {
                planned.push((key, values));
            }
        }

        let additions = planned
            .iter()
            .filter(|(key, _)| !self.pages.contains_key(key))
            .count();
        if self.pages.len().saturating_add(additions) > self.max_pages {
            return Err(SurfaceSculptError::PageCapacityExceeded);
        }
        if planned.is_empty() {
            return Ok(Vec::new());
        }

        let touched = planned.iter().map(|(key, _)| tile_key(*key)).collect();
        for (key, values) in planned {
            self.pages.insert(key, values);
        }
        self.revision = next_revision;
        Ok(touched)
    }

    /// Sample the additive sculpt offset at any finite nonzero direction.
    /// Missing pages are exactly zero; page seams interpolate endpoint values.
    pub fn sample(&self, unit_direction: [f64; 3]) -> Result<f64, SurfaceSculptError> {
        let direction = normalize(unit_direction).ok_or(SurfaceSculptError::InvalidDirection)?;
        let (face, uv) = face_uv(direction);
        let pages_per_axis = 1_u64 << self.level;
        let total_cells = PAGE_CELLS * pages_per_axis;
        let px = (uv[0] * pages_per_axis as f64)
            .floor()
            .clamp(0.0, (pages_per_axis - 1) as f64) as u32;
        let py = (uv[1] * pages_per_axis as f64)
            .floor()
            .clamp(0.0, (pages_per_axis - 1) as f64) as u32;
        let tile = TileKey::new(face, self.level, px, py).expect("derived sculpt tile is valid");
        let key = key_for(tile);
        let Some(values) = self.pages.get(&key) else {
            return Ok(0.0);
        };
        let gx = (uv[0] * total_cells as f64 - f64::from(px) * PAGE_CELLS as f64)
            .clamp(0.0, PAGE_CELLS as f64);
        let gy = (uv[1] * total_cells as f64 - f64::from(py) * PAGE_CELLS as f64)
            .clamp(0.0, PAGE_CELLS as f64);
        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let x1 = (x0 + 1).min(PAGE_CELLS as usize);
        let y1 = (y0 + 1).min(PAGE_CELLS as usize);
        let tx = gx - x0 as f64;
        let ty = gy - y0 as f64;
        let at = |x: usize, y: usize| values[y * PAGE_SIDE + x];
        let a = at(x0, y0) + (at(x1, y0) - at(x0, y0)) * tx;
        let b = at(x0, y1) + (at(x1, y1) - at(x0, y1)) * tx;
        Ok(a + (b - a) * ty)
    }

    pub fn allocated_pages(&self) -> usize {
        self.pages.len()
    }

    /// Retained page sample payload, excluding allocator/map metadata.
    pub fn allocated_bytes(&self) -> usize {
        self.pages.len().saturating_mul(PAGE_BYTES)
    }

    pub fn radius_m(&self) -> f64 {
        self.radius_m
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    fn candidate_pages(
        &self,
        center: [f64; 3],
        angular_radius: f64,
    ) -> Result<BTreeSet<(u8, u8, u32, u32)>, SurfaceSculptError> {
        let pages_per_axis = 1_u64 << self.level;
        let chord = 2.0 * (angular_radius * 0.5).sin();
        let mut out = BTreeSet::new();
        let mut candidate_count = 0usize;
        for face in 0..6_u8 {
            let (axis, sign, u_axis, u_sign, v_axis, v_sign) = face_axes(face);
            let axis_center = center[axis] * sign;
            let denom_max = (axis_center + chord).min(1.0);
            if denom_max < MIN_FACE_COMPONENT {
                continue;
            }
            let denom_min = (axis_center - chord).max(MIN_FACE_COMPONENT);
            let u_bounds = ratio_bounds(center[u_axis] * u_sign, chord, denom_min, denom_max);
            let v_bounds = ratio_bounds(center[v_axis] * v_sign, chord, denom_min, denom_max);
            let u_range = page_range(u_bounds, pages_per_axis);
            let v_range = page_range(v_bounds, pages_per_axis);
            let width = u64::from(u_range.1) - u64::from(u_range.0) + 1;
            let height = u64::from(v_range.1) - u64::from(v_range.0) + 1;
            let count = width
                .checked_mul(height)
                .ok_or(SurfaceSculptError::CandidateLimitExceeded)?;
            candidate_count = candidate_count
                .checked_add(
                    usize::try_from(count)
                        .map_err(|_| SurfaceSculptError::CandidateLimitExceeded)?,
                )
                .ok_or(SurfaceSculptError::CandidateLimitExceeded)?;
            if candidate_count > self.max_pages {
                return Err(SurfaceSculptError::CandidateLimitExceeded);
            }
            for y in v_range.0..=v_range.1 {
                for x in u_range.0..=u_range.1 {
                    out.insert((face, self.level, x, y));
                }
            }
        }
        Ok(out)
    }
}

fn normalize(v: [f64; 3]) -> Option<[f64; 3]> {
    if v.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let scale = v.iter().fold(0.0_f64, |a, x| a.max(x.abs()));
    if scale == 0.0 {
        return None;
    }
    let scaled = v.map(|x| x / scale);
    let length = dot(scaled, scaled).sqrt();
    Some(scaled.map(|x| x / length))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross_length(a: [f64; 3], b: [f64; 3]) -> f64 {
    let c = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    dot(c, c).sqrt()
}

fn face_axes(face: u8) -> (usize, f64, usize, f64, usize, f64) {
    match face {
        0 => (0, 1.0, 2, -1.0, 1, 1.0),
        1 => (0, -1.0, 2, 1.0, 1, 1.0),
        2 => (1, 1.0, 0, 1.0, 2, -1.0),
        3 => (1, -1.0, 0, 1.0, 2, 1.0),
        4 => (2, 1.0, 0, 1.0, 1, 1.0),
        _ => (2, -1.0, 0, -1.0, 1, 1.0),
    }
}

fn face_uv(direction: [f64; 3]) -> (u8, [f64; 2]) {
    let a = direction.map(f64::abs);
    let (face, denom, u, v) = if a[0] >= a[1] && a[0] >= a[2] {
        let positive = direction[0] >= 0.0;
        let face = if positive { 0 } else { 1 };
        let (ua, us) = if positive { (2, -1.0) } else { (2, 1.0) };
        (face, a[0], direction[ua] * us, direction[1])
    } else if a[1] >= a[2] {
        let positive = direction[1] >= 0.0;
        let face = if positive { 2 } else { 3 };
        let (va, vs) = if positive { (2, -1.0) } else { (2, 1.0) };
        (face, a[1], direction[0], direction[va] * vs)
    } else {
        let positive = direction[2] >= 0.0;
        let face = if positive { 4 } else { 5 };
        let (ua, us) = if positive { (0, 1.0) } else { (0, -1.0) };
        (face, a[2], direction[ua] * us, direction[1])
    };
    (face, [0.5 * (u / denom + 1.0), 0.5 * (v / denom + 1.0)])
}

fn ratio_bounds(numerator: f64, extent: f64, denom_min: f64, denom_max: f64) -> (f64, f64) {
    let values = [
        (numerator - extent) / denom_min,
        (numerator - extent) / denom_max,
        (numerator + extent) / denom_min,
        (numerator + extent) / denom_max,
    ];
    let low = values.into_iter().fold(f64::INFINITY, f64::min);
    let high = values.into_iter().fold(f64::NEG_INFINITY, f64::max);
    (0.5 * (low + 1.0), 0.5 * (high + 1.0))
}

fn page_range(bounds: (f64, f64), pages_per_axis: u64) -> (u32, u32) {
    let last = pages_per_axis - 1;
    let low = (bounds.0.clamp(0.0, 1.0) * pages_per_axis as f64).floor() as u64;
    let high = (bounds.1.clamp(0.0, 1.0) * pages_per_axis as f64).floor() as u64;
    (low.min(last) as u32, high.min(last) as u32)
}

fn page_sample_direction(tile: TileKey, local_x: u64, local_y: u64) -> [f64; 3] {
    let pages_per_axis = 1_u64 << tile.level;
    let cells = PAGE_CELLS * pages_per_axis;
    let u = (f64::from(tile.x) * PAGE_CELLS as f64 + local_x as f64) / cells as f64;
    let v = (f64::from(tile.y) * PAGE_CELLS as f64 + local_y as f64) / cells as f64;
    let p = [2.0 * u - 1.0, 2.0 * v - 1.0];
    let raw = match tile.face {
        0 => [1.0, p[1], -p[0]],
        1 => [-1.0, p[1], p[0]],
        2 => [p[0], 1.0, -p[1]],
        3 => [p[0], -1.0, p[1]],
        4 => [p[0], p[1], 1.0],
        _ => [-p[0], p[1], -1.0],
    };
    normalize(raw).expect("cube face sample is nonzero")
}

fn key_for(tile: TileKey) -> (u8, u8, u32, u32) {
    (tile.face, tile.level, tile.x, tile.y)
}

fn tile_key(key: (u8, u8, u32, u32)) -> TileKey {
    TileKey::new(key.0, key.1, key.2, key.3).expect("stored sculpt tile is valid")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir(face: u8, u: f64, v: f64) -> [f64; 3] {
        let p = [2.0 * u - 1.0, 2.0 * v - 1.0];
        normalize(match face {
            0 => [1.0, p[1], -p[0]],
            1 => [-1.0, p[1], p[0]],
            2 => [p[0], 1.0, -p[1]],
            3 => [p[0], -1.0, p[1]],
            4 => [p[0], p[1], 1.0],
            _ => [-p[0], p[1], -1.0],
        })
        .unwrap()
    }

    #[test]
    fn radius_and_target_texel_scale_the_virtual_level() {
        assert_eq!(SurfaceSculpt::new(1_000.0, 10.0, 64).unwrap().level, 3);
        assert_eq!(SurfaceSculpt::new(2_000.0, 10.0, 64).unwrap().level, 4);
        assert_eq!(SurfaceSculpt::new(1_000.0, 20.0, 64).unwrap().level, 2);
    }

    #[test]
    fn stamp_has_exact_center_peak_and_zero_elsewhere() {
        let mut layer = SurfaceSculpt::new(2_000.0, 10.0, 32).unwrap();
        assert_eq!(layer.sample([0.0, 0.0, 1.0]).unwrap(), 0.0);
        let changed = layer.apply_dab([0.0, 0.0, 9.0], 20.0, 3.5).unwrap();
        assert!(!changed.is_empty());
        assert_eq!(layer.sample([0.0, 0.0, 1.0]).unwrap(), 3.5);
        assert_eq!(layer.sample([0.0, 0.0, -1.0]).unwrap(), 0.0);
        assert!(
            layer
                .pages
                .values()
                .flat_map(|page| page.iter())
                .all(|height| (0.0..=3.5).contains(height))
        );
        assert_eq!(layer.revision(), 1);
    }

    #[test]
    fn cross_face_stamp_writes_both_shared_edge_pages() {
        let mut layer = SurfaceSculpt::new(2_000.0, 10.0, 64).unwrap();
        let edge = normalize([1.0, 0.0, 1.0]).unwrap();
        let touched = layer.apply_dab(edge, 20.0, 2.0).unwrap();
        assert!(touched.iter().any(|tile| tile.face == 0));
        assert!(touched.iter().any(|tile| tile.face == 4));
        assert!((layer.sample(edge).unwrap() - 2.0).abs() < 1e-12);
        let before = layer
            .sample(normalize([1.0 - 1e-10, 0.0, 1.0]).unwrap())
            .unwrap();
        let after = layer
            .sample(normalize([1.0, 0.0, 1.0 - 1e-10]).unwrap())
            .unwrap();
        assert!((before - after).abs() < 1e-8);
    }

    #[test]
    fn capacity_refusal_is_transactional_and_zero_delta_is_free() {
        let mut layer = SurfaceSculpt::new(2_000.0, 10.0, 1).unwrap();
        assert!(layer.apply_dab(dir(4, 0.28, 0.28), 20.0, 1.0).is_ok());
        let old_value = layer.sample(dir(4, 0.28, 0.28)).unwrap();
        let old_revision = layer.revision();
        assert_eq!(layer.allocated_pages(), 1);
        assert_eq!(layer.allocated_bytes(), PAGE_BYTES);
        assert!(matches!(
            layer.apply_dab(dir(4, 0.72, 0.72), 20.0, 1.0),
            Err(SurfaceSculptError::PageCapacityExceeded)
        ));
        assert_eq!(layer.sample(dir(4, 0.28, 0.28)).unwrap(), old_value);
        assert_eq!(layer.revision(), old_revision);
        assert_eq!(
            layer.apply_dab(dir(4, 0.28, 0.28), 20.0, 0.0).unwrap(),
            Vec::new()
        );
        assert_eq!(layer.revision(), old_revision);
    }

    #[test]
    fn invalid_inputs_and_unrepresentable_brushes_are_rejected() {
        assert!(matches!(
            SurfaceSculpt::new(0.0, 1.0, 1),
            Err(SurfaceSculptError::InvalidRadius)
        ));
        let mut layer = SurfaceSculpt::new(1_000.0, 10.0, 4).unwrap();
        assert!(matches!(
            layer.sample([0.0; 3]),
            Err(SurfaceSculptError::InvalidDirection)
        ));
        assert!(matches!(
            layer.apply_dab([0.0, 0.0, 1.0], 1.0, 1.0),
            Err(SurfaceSculptError::BrushTooSmall)
        ));
        assert!(matches!(
            layer.apply_dab([0.0, 0.0, 1.0], 11.0, 1.0),
            Err(SurfaceSculptError::InvalidBrushRadius)
        ));
        assert_eq!(layer.allocated_pages(), 0);
        assert_eq!(layer.revision(), 0);
    }
}

#[cfg(test)]
mod planet_contact_acceptance {
    use super::*;

    #[test]
    fn centimetre_stamps_are_continuous_at_all_cube_edges_and_corners() {
        let mut centers = Vec::new();
        for zero_axis in 0..3 {
            for a in [-1.0, 1.0] {
                for b in [-1.0, 1.0] {
                    let mut d = [0.0; 3];
                    d[(zero_axis + 1) % 3] = a;
                    d[(zero_axis + 2) % 3] = b;
                    centers.push(normalize(d).unwrap());
                }
            }
        }
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    centers.push(normalize([x, y, z]).unwrap());
                }
            }
        }
        for center in centers {
            let mut layer = SurfaceSculpt::new(3_200_000.0, 0.025, 64).unwrap();
            layer.apply_dab(center, 0.20, -0.04).unwrap();
            let peak = layer.sample(center).unwrap();
            assert!(
                (peak + 0.04).abs() < 0.001,
                "submillimetre peak error at {center:?}: {peak}"
            );
            for axis in 0..3 {
                let mut a = center;
                let mut b = center;
                a[axis] += 1e-10;
                b[axis] -= 1e-10;
                let delta = (layer.sample(a).unwrap() - layer.sample(b).unwrap()).abs();
                assert!(delta < 0.0001, "face seam at {center:?}: {delta} m");
            }
            assert!(layer.allocated_bytes() <= 64 * PAGE_BYTES);
        }
    }
}

#[cfg(test)]
mod workload {
    use super::*;
    #[test]
    #[ignore = "manual CPU workload; timings are not whole-renderer performance"]
    fn bounded_planet_track_workload() {
        let radius = 3_200_000.0;
        let mut layer = SurfaceSculpt::new(radius, 0.025, 128).unwrap();
        let begin = std::time::Instant::now();
        for i in 0..128 {
            let x = i as f64 * 0.08;
            layer
                .apply_dab([x / radius, 0.0, 1.0], 0.10, -0.01)
                .unwrap();
        }
        let write_ms = begin.elapsed().as_secs_f64() * 1000.0;
        let begin = std::time::Instant::now();
        let mut sum = 0.0;
        for i in 0..100_000 {
            sum += layer
                .sample([(i % 1024) as f64 * 0.01 / radius, 0.0, 1.0])
                .unwrap();
        }
        let sample_ms = begin.elapsed().as_secs_f64() * 1000.0;
        assert!(sum.is_finite() && sum < 0.0);
        assert!(layer.allocated_pages() <= 128);
        eprintln!(
            "surface sculpt CPU: 128 authored circular dabs over 10.16m write_ms={write_ms:.3}; 100000 samples_ms={sample_ms:.3}; retained_pages={} sample_bytes={}; excludes renderer, page rebuilding and GPU uploads",
            layer.allocated_pages(),
            layer.allocated_bytes()
        );
    }
}
