//! Bounded static optical standing-water reconstruction, not hydrology state.
#[derive(Debug)]
pub struct BasinSurface {
    pub level_m: f64,
    pub basin_count: usize,
    pub spill_m: f64,
    /// Independent clipped triangles: local x, water elevation, local z.
    pub triangles: Vec<[[f64; 3]; 3]>,
}

/// Reuse priority-flood drainage to find each closed component and its spill.
/// Water levels stay below local spill; shores clip the sampled ground triangles.
pub fn fill_closed_basin(
    heights: &[f64],
    side: usize,
    spacing_m: f64,
    availability: f64,
    max_depth_m: f64,
) -> Option<BasinSurface> {
    if !(3..=33).contains(&side)
        || heights.len() != side * side
        || heights.iter().any(|h| !h.is_finite())
        || !spacing_m.is_finite()
        || spacing_m <= 0.0
        || !availability.is_finite()
        || availability <= 0.0
        || !max_depth_m.is_finite()
        || max_depth_m <= 0.0
    {
        return None;
    }
    let adjacent = |r: usize, c: usize| {
        let at = |dr: isize, dc: isize| {
            let nr = r.checked_add_signed(dr)?;
            let nc = c.checked_add_signed(dc)?;
            (nr < side && nc < side).then_some((nr, nc))
        };
        // The anti-diagonal is exactly the clipped terrain's triangle edge.
        [
            at(-1, 0),
            at(1, 0),
            at(0, -1),
            at(0, 1),
            at(-1, 1),
            at(1, -1),
        ]
        .into_iter()
        .flatten()
    };
    let rows: Vec<Vec<f64>> = heights.chunks_exact(side).map(|v| v.to_vec()).collect();
    let filled = crate::hydro::priority_filled_heights(&rows, adjacent, |r, c| {
        r == 0 || c == 0 || r == side - 1 || c == side - 1
    });
    let mut visited = vec![false; side * side];
    let mut levels = vec![None; side * side];
    let mut basin_count = 0;
    let mut level = f64::NEG_INFINITY;
    let mut spill = f64::NEG_INFINITY;
    for start in 0..side * side {
        let rim = filled[start / side][start % side];
        if visited[start] || rim <= heights[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut cells = Vec::new();
        visited[start] = true;
        while let Some(i) = stack.pop() {
            cells.push(i);
            for (r, c) in adjacent(i / side, i % side) {
                let j = r * side + c;
                if !visited[j] && filled[r][c] == rim && heights[j] < rim {
                    visited[j] = true;
                    stack.push(j);
                }
            }
        }
        let low = cells
            .iter()
            .map(|&i| heights[i])
            .fold(f64::INFINITY, f64::min);
        let water =
            low + ((rim - low - 0.001).max(0.0) * availability.clamp(0.0, 1.0)).min(max_depth_m);
        if water - low < 0.006 {
            continue;
        }
        basin_count += 1;
        level = level.max(water);
        spill = spill.max(rim);
        for i in cells {
            levels[i] = Some(water);
        }
    }
    if basin_count == 0 {
        return None;
    }
    let at = |x: usize, y: usize| {
        [
            (x as f64 - (side - 1) as f64 * 0.5) * spacing_m,
            heights[y * side + x],
            (y as f64 - (side - 1) as f64 * 0.5) * spacing_m,
        ]
    };
    let mut triangles = Vec::new();
    for y in 0..side - 1 {
        for x in 0..side - 1 {
            let [a, b, c, d] = [at(x, y), at(x + 1, y), at(x, y + 1), at(x + 1, y + 1)];
            for (tri, ids) in [
                (
                    [a, c, b],
                    [y * side + x, (y + 1) * side + x, y * side + x + 1],
                ),
                (
                    [b, c, d],
                    [y * side + x + 1, (y + 1) * side + x, (y + 1) * side + x + 1],
                ),
            ] {
                let Some(level) = ids
                    .iter()
                    .find_map(|&i| levels[i].filter(|water| heights[i] < *water))
                else {
                    continue;
                };
                let mut polygon = Vec::with_capacity(4);
                for i in 0..3 {
                    let a = tri[i];
                    let b = tri[(i + 1) % 3];
                    let inside = a[1] < level;
                    if inside {
                        polygon.push([a[0], level, a[2]]);
                    }
                    if inside != (b[1] < level) {
                        let t = (level - a[1]) / (b[1] - a[1]);
                        polygon.push([a[0] + (b[0] - a[0]) * t, level, a[2] + (b[2] - a[2]) * t]);
                    }
                }
                for i in 1..polygon.len().saturating_sub(1) {
                    let t = [polygon[0], polygon[i], polygon[i + 1]];
                    let area = (t[1][2] - t[0][2]) * (t[2][0] - t[0][0])
                        - (t[1][0] - t[0][0]) * (t[2][2] - t[0][2]);
                    if area > 1e-12 {
                        triangles.push(t);
                    }
                }
            }
        }
    }
    (!triangles.is_empty()).then_some(BasinSurface {
        level_m: level,
        basin_count,
        spill_m: spill,
        triangles,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lower_patch_outlet_does_not_empty_disjoint_closed_hollows() {
        let mut h = vec![0.2; 81];
        h[4 * 9] = 0.0;
        h[4 * 9 + 2] = 0.0;
        h[4 * 9 + 6] = 0.1;
        let water = fill_closed_basin(&h, 9, 0.1, 1.0, 0.03).unwrap();
        assert_eq!(water.basin_count, 2);
        let mut levels: Vec<_> = water.triangles.iter().map(|t| t[0][1]).collect();
        levels.sort_by(f64::total_cmp);
        levels.dedup();
        assert_eq!(levels.len(), 2);
        assert!((levels[0] - 0.03).abs() < 1e-12 && (levels[1] - 0.13).abs() < 1e-12);
        for tri in water.triangles {
            assert!(tri.iter().all(|p| p[1] == tri[0][1]));
        }
    }
    #[test]
    fn bowl_is_level_bounded_and_clipped_to_its_shoreline() {
        let side = 17;
        let spacing = 0.1;
        let heights: Vec<_> = (0..side * side)
            .map(|i| {
                let x = (i % side) as f64 * spacing - 0.8;
                let z = (i / side) as f64 * spacing - 0.8;
                0.1 * (x * x + z * z)
            })
            .collect();
        let water = fill_closed_basin(&heights, side, spacing, 0.8, 0.03).unwrap();
        assert!((water.level_m - 0.03).abs() < 1e-12 && water.level_m < water.spill_m);
        let area: f64 = water
            .triangles
            .iter()
            .map(|t| {
                for v in t {
                    assert_eq!(v[1], water.level_m);
                    assert!(v[0].abs() < 0.8 && v[2].abs() < 0.8);
                }
                ((t[1][0] - t[0][0]) * (t[2][2] - t[0][2])
                    - (t[1][2] - t[0][2]) * (t[2][0] - t[0][0]))
                    .abs()
                    * 0.5
            })
            .sum();
        assert!((area - std::f64::consts::PI * 0.3).abs() < 0.025);
        assert!(fill_closed_basin(&heights, side, spacing, 0.0, 0.03).is_none());
        assert!(fill_closed_basin(&[0.0; 289], side, spacing, 1.0, 0.03).is_none());
        let slope: Vec<_> = (0..289).map(|i| (i % 17) as f64 * 0.01).collect();
        assert!(fill_closed_basin(&slope, side, spacing, 1.0, 0.03).is_none());
    }
}
