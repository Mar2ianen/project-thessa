//! Tectonics-lite: plate boundaries as authored polylines.
//!
//! The base planet shape comes from causal uplift/subsidence, not from an
//! AI-painted heightmap. Boundaries are defined by geographic paths with
//! physical widths and amplitudes (metres). Everything is deterministic.

use serde::{Deserialize, Serialize};

/// Boundary kinematics. All effects in metres of vertical displacement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryKind {
    /// Divergent: broad swell + narrow central graben dip.
    Ridge,
    /// Continental rift: swell + wider graben (future ocean).
    Rift,
    /// Convergent: deep trench at the line + volcanic arc offset aside.
    Subduction,
    /// Strike-slip: low alternating ridge/valley texture along the line.
    Transform,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Boundary {
    pub id: String,
    pub kind: BoundaryKind,
    /// Polyline in [lat_deg, lon_deg] pairs, at least 2 points.
    pub points: Vec<[f64; 2]>,
    /// Half-width of the deformation zone, metres.
    pub width_m: f64,
    /// Total amplitude (uplift positive / subsidence magnitude), metres.
    pub rate_m: f64,
    /// Which side gets the volcanic arc (subduction only): +1 / -1.
    #[serde(default = "default_arc_side")]
    pub arc_side: f64,
}

fn default_arc_side() -> f64 {
    1.0
}

impl Boundary {
    pub fn validate(&self) -> Result<(), String> {
        if self.id.trim().is_empty() {
            return Err("boundary needs an id".into());
        }
        if self.points.len() < 2 {
            return Err(format!("boundary {} needs >= 2 points", self.id));
        }
        for [lat, lon] in &self.points {
            if !lat.is_finite() || !(-90.0..=90.0).contains(lat) {
                return Err(format!("boundary {} bad lat", self.id));
            }
            if !lon.is_finite() || !(-180.0..=180.0).contains(lon) {
                return Err(format!("boundary {} bad lon", self.id));
            }
        }
        if !self.width_m.is_finite() || self.width_m <= 0.0 {
            return Err(format!("boundary {} width must be positive", self.id));
        }
        if !self.rate_m.is_finite() || self.rate_m < 0.0 {
            return Err(format!(
                "boundary {} rate must be finite non-negative",
                self.id
            ));
        }
        Ok(())
    }
}

/// Distance in metres from (lat,lon) to a polyline + closest segment tangent.
/// Equirectangular local approximation is fine for authoring-scale zones.
fn dist_to_polyline_m(
    lat_deg: f64,
    lon_deg: f64,
    points: &[[f64; 2]],
    radius_m: f64,
) -> (f64, f64, f64) {
    let cos_lat = lat_deg.to_radians().cos().max(0.05);
    let px = lon_deg.to_radians() * radius_m * cos_lat;
    let py = lat_deg.to_radians() * radius_m;
    let mut best_d = f64::INFINITY;
    let mut best_side = 0.0;
    let mut best_along = 0.0;
    for pair in points.windows(2) {
        let ax = pair[0][1].to_radians() * radius_m * cos_lat;
        let ay = pair[0][0].to_radians() * radius_m;
        let bx = pair[1][1].to_radians() * radius_m * cos_lat;
        let by = pair[1][0].to_radians() * radius_m;
        let abx = bx - ax;
        let aby = by - ay;
        let len2 = (abx * abx + aby * aby).max(1.0);
        let t = ((px - ax) * abx + (py - ay) * aby) / len2;
        let tc = t.clamp(0.0, 1.0);
        let cx = ax + abx * tc;
        let cy = ay + aby * tc;
        let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
        if d < best_d {
            best_d = d;
            // Signed side via 2D cross product (which side of travel direction).
            best_side = (abx * (py - ay) - aby * (px - ax)).signum();
            best_along = tc;
        }
    }
    (best_d, best_side, best_along)
}

/// Vertical displacement in metres. Pure function of inputs.
pub fn eval_uplift_m(boundaries: &[Boundary], lat_deg: f64, lon_deg: f64, radius_m: f64) -> f64 {
    let mut total = 0.0;
    for b in boundaries {
        let (d, side, _along) = dist_to_polyline_m(lat_deg, lon_deg, &b.points, radius_m);
        let w = b.width_m;
        let g = (-(d / w).powi(2)).exp();
        total += match b.kind {
            BoundaryKind::Ridge => {
                // Broad swell with a narrow axial graben.
                b.rate_m * (-(d / (w * 2.5)).powi(2)).exp()
                    - b.rate_m * 0.35 * (-(d / (w * 0.35)).powi(2)).exp()
            }
            BoundaryKind::Rift => {
                b.rate_m * 0.6 * (-(d / (w * 3.0)).powi(2)).exp()
                    - b.rate_m * 0.5 * (-(d / (w * 0.8)).powi(2)).exp()
            }
            BoundaryKind::Subduction => {
                // Trench at the line, arc bump offset to arc_side.
                let trench = -b.rate_m * (-(d / (w * 0.5)).powi(2)).exp();
                let arc_center = w * 1.8 * b.arc_side.signum();
                // Offset distance measured along the signed side axis approx:
                // reuse d with side sign to shift the gaussian.
                let arc_d = (d * side - arc_center).abs();
                let arc = b.rate_m * 0.8 * (-(arc_d / w).powi(2)).exp();
                trench * g.max(0.2) + arc
            }
            BoundaryKind::Transform => b.rate_m * 0.15 * g * (side * (d / w * 3.0).sin()),
        };
    }
    if total.is_finite() { total } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ridge() -> Vec<Boundary> {
        vec![Boundary {
            id: "r".into(),
            kind: BoundaryKind::Ridge,
            points: vec![[-10.0, 0.0], [10.0, 0.0]],
            width_m: 200_000.0,
            rate_m: 2500.0,
            arc_side: 1.0,
        }]
    }

    #[test]
    fn ridge_uplifts_axis_and_dips_graben() {
        let b = ridge();
        let on = eval_uplift_m(&b, 0.0, 0.0, 3_200_000.0);
        let far = eval_uplift_m(&b, 0.0, 40.0, 3_200_000.0);
        assert!(on > 1000.0, "swell dominates on axis: {on}");
        assert!(far.abs() < 1.0, "far field quiet: {far}");
    }

    #[test]
    fn subduction_makes_trench_and_arc() {
        let b = vec![Boundary {
            id: "s".into(),
            kind: BoundaryKind::Subduction,
            points: vec![[-10.0, 0.0], [10.0, 0.0]],
            width_m: 150_000.0,
            rate_m: 6000.0,
            arc_side: 1.0,
        }];
        let r = 3_200_000.0;
        let trench = eval_uplift_m(&b, 0.0, 0.0, r);
        assert!(trench < -2000.0, "trench at the line: {trench}");
        // Arc sits ~270 km to the +side (west of a south->north line here).
        let arc_lon = -f64::to_degrees(270_000.0 / r);
        let arc = eval_uplift_m(&b, 0.0, arc_lon, r);
        assert!(arc > 1500.0, "volcanic arc offset aside: {arc}");
    }

    #[test]
    fn deterministic_and_finite() {
        let b = ridge();
        let a = eval_uplift_m(&b, 3.0, 2.0, 3_200_000.0);
        assert_eq!(a, eval_uplift_m(&b, 3.0, 2.0, 3_200_000.0));
        assert!(a.is_finite());
    }

    #[test]
    fn validation_rejects_garbage() {
        let mut b = ridge().pop().unwrap();
        b.points = vec![[0.0, 0.0]];
        assert!(b.validate().is_err());
    }
}
