//! Game map policy and f64 presentation geometry, without a renderer or clock.
//! Ephemerides/rails own paths; this module only selects, samples and projects.
use glam::{DQuat, DVec3};
use thessa_sim_core::{BakedEphemeris, BodyId, KeplerOrbit, SampledPath, SampledPathEnd, SimTime};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapMode {
    SystemOverview,
    Asterion,
    Nereid,
    Orthea,
    Vesper,
    Binary,
}
impl MapMode {
    pub const ALL: [Self; 6] = [
        Self::SystemOverview,
        Self::Asterion,
        Self::Nereid,
        Self::Orthea,
        Self::Vesper,
        Self::Binary,
    ];
    pub fn focus_name(self) -> &'static str {
        match self {
            Self::SystemOverview => "system_barycenter",
            Self::Asterion => "asterion_a",
            Self::Nereid => "nereid",
            Self::Orthea => "orthea",
            Self::Vesper => "vesper",
            Self::Binary => "bc_barycenter",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::SystemOverview => "SYSTEM OVERVIEW",
            Self::Asterion => "ASTERION A SYSTEM",
            Self::Nereid => "NEREID MOON SYSTEM",
            Self::Orthea => "ORTHEA MOON SYSTEM",
            Self::Vesper => "VESPER MOON SYSTEM",
            Self::Binary => "ASTERION B/C BINARY",
        }
    }
}

pub fn visible(ephemeris: &BakedEphemeris, mode: MapMode, focus: BodyId, id: BodyId) -> bool {
    if mode == MapMode::SystemOverview {
        return true;
    }
    let mut current = Some(id);
    // Baked trees are acyclic; bound the walk even for a malformed external tree.
    for _ in 0..=ephemeris.bodies.len() {
        let Some(id) = current else {
            return false;
        };
        if id == focus {
            return true;
        }
        current = ephemeris.body(id).ok().and_then(|body| body.parent);
    }
    false
}

pub fn cycle_body(bodies: &[BodyId], selected: BodyId, reverse: bool) -> Option<BodyId> {
    if bodies.is_empty() {
        return None;
    }
    let index = bodies.iter().position(|body| *body == selected);
    Some(match index {
        Some(index) if reverse => bodies[(index + bodies.len() - 1) % bodies.len()],
        Some(index) => bodies[(index + 1) % bodies.len()],
        None if reverse => *bodies.last()?,
        None => bodies[0],
    })
}

/// Orthographic SI map camera. No absolute f32 astronomical coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MapCamera {
    pub center_m: DVec3,
    pub orbit: DQuat,
    pub half_height_m: f64,
}
impl Default for MapCamera {
    fn default() -> Self {
        Self {
            center_m: DVec3::ZERO,
            orbit: DQuat::from_rotation_x(0.35),
            half_height_m: 1e9,
        }
    }
}
impl MapCamera {
    pub fn project(self, point: DVec3, extent: [f64; 2]) -> Option<[f64; 3]> {
        if !point.is_finite()
            || !self.center_m.is_finite()
            || !self.orbit.is_finite()
            || (self.orbit.length_squared() - 1.0).abs() > 1e-8
            || !self.half_height_m.is_finite()
            || self.half_height_m <= 0.0
            || extent.iter().any(|v| !v.is_finite() || *v <= 0.0)
        {
            return None;
        }
        let p = self.orbit.conjugate() * (point - self.center_m);
        let hh = self.half_height_m;
        let result = [
            0.5 + p.x / hh * (extent[1] / extent[0]) * 0.5,
            0.5 - p.y / hh * 0.5,
            p.z / hh,
        ];
        result.iter().all(|v| v.is_finite()).then_some(result)
    }
    /// Clip in f64 before narrowing to toolkit pixels. Astronomical offscreen
    /// endpoints must not become huge f32 raster vertices near a framed moon.
    pub fn project_segment(self, a: DVec3, b: DVec3, extent: [f64; 2]) -> Option<[[f64; 2]; 2]> {
        let a = self.project(a, extent)?;
        let b = self.project(b, extent)?;
        let delta = [b[0] - a[0], b[1] - a[1]];
        if delta.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let (mut start, mut end) = (0.0_f64, 1.0_f64);
        for axis in 0..2 {
            if delta[axis] == 0.0 {
                if !(0.0..=1.0).contains(&a[axis]) {
                    return None;
                }
            } else {
                let lo = -a[axis] / delta[axis];
                let hi = (1.0 - a[axis]) / delta[axis];
                start = start.max(lo.min(hi));
                end = end.min(lo.max(hi));
                if start > end {
                    return None;
                }
            }
        }
        Some(
            [start, end]
                .map(|t| std::array::from_fn(|axis| (a[axis] + delta[axis] * t).clamp(0.0, 1.0))),
        )
    }
    pub fn drag_orbit(&mut self, dx: f64, dy: f64) {
        if dx.is_finite() && dy.is_finite() {
            self.orbit = (self.orbit
                * DQuat::from_rotation_y(-dx * 0.004)
                * DQuat::from_rotation_x(-dy * 0.004))
            .normalize();
        }
    }
    pub fn pan(&mut self, dx: f64, dy: f64, height: f64) {
        if dx.is_finite() && dy.is_finite() && height.is_finite() && height > 0.0 {
            let next = self.center_m
                + self.orbit * DVec3::new(-dx, dy, 0.0) * (2.0 * self.half_height_m / height);
            if next.is_finite() {
                self.center_m = next;
            }
        }
    }
    pub fn zoom(&mut self, lines: f64) {
        if lines.is_finite() {
            self.half_height_m = (self.half_height_m * (-0.1 * lines).exp()).clamp(1.0, 1e17);
        }
    }
}

/// Existing adaptive mean-anomaly sampling, now in SI and renderer-independent.
/// A complete closed curve is retained even when its subdivision budget ends.
/// The caller can measure the residual; budget exhaustion is not error parity.
pub fn kepler_polyline(orbit: KeplerOrbit, segments: usize, tolerance_m: f64) -> Vec<DVec3> {
    let base = segments.clamp(24, 64);
    let max = segments.clamp(base, 512);
    let at = |fraction| {
        orbit
            .state_relative_at(SimTime(orbit.period_s() * fraction))
            .expect("baked orbit")
            .0
    };
    let mut points = Vec::with_capacity(max + 1);
    points.push(at(0.0));
    let mut budget = max - base;
    let mut stack = Vec::new();
    for i in 0..base {
        let start = i as f64 / base as f64;
        let end = (i + 1) as f64 / base as f64;
        stack.push((start, end, *points.last().unwrap(), at(end)));
        while let Some((a, b, p, q)) = stack.pop() {
            let mid = (a + b) * 0.5;
            let r = at(mid);
            if budget > 0
                && tolerance_m.is_finite()
                && tolerance_m > 0.0
                && r.distance(p.lerp(q, 0.5)) > tolerance_m
            {
                budget -= 1;
                stack.push((mid, b, r, q));
                stack.push((a, mid, p, r));
            } else {
                points.push(q);
            }
        }
    }
    points
}

pub fn future_path_indices(times: &[SimTime], now: SimTime, budget: usize) -> Vec<usize> {
    let first = times.partition_point(|time| time.0 <= now.0);
    if first == times.len() || budget < 2 {
        return Vec::new();
    }
    let stride = (times.len() - first).div_ceil(budget - 1);
    let mut indices: Vec<_> = (first..times.len()).step_by(stride).collect();
    let last = times.len() - 1;
    if indices.last() != Some(&last) {
        indices.push(last);
    }
    indices
}

#[derive(Default)]
pub struct ProjectedPath {
    pub points: Vec<DVec3>,
    pub impact: Option<DVec3>,
}
/// Reprojects the existing shared full-field bake at each point's own time.
/// Never integrate a second forecast or interpret a two-body ellipse as rails.
pub fn project_rails_path(
    ephemeris: &BakedEphemeris,
    path: &SampledPath,
    display: BodyId,
    time: SimTime,
    craft: DVec3,
    view_cut_m: f64,
) -> ProjectedPath {
    let mut result = ProjectedPath::default();
    let Ok(home) = ephemeris.body_state(display, time) else {
        return result;
    };
    if path.times.is_empty()
        || path.times.len() != path.positions.len()
        || !time.0.is_finite()
        || !craft.is_finite()
        || time.0 < path.times[0].0
        || time.0 > path.end_time.0
    {
        return result;
    }
    let initial = craft - home.position_inertial;
    let cut = (initial.length().max(1.0) * 25.0).max(view_cut_m);
    result.points.push(initial);
    let mut end = false;
    for index in future_path_indices(&path.times, time, 1024) {
        let Ok(home) = ephemeris.body_state(display, path.times[index]) else {
            break;
        };
        let relative = path.positions[index] - home.position_inertial;
        if !relative.is_finite() || relative.length() > cut {
            break;
        }
        result.points.push(relative);
        end = index + 1 == path.positions.len();
    }
    if end && matches!(path.end, SampledPathEnd::Impact(_)) {
        result.impact = result.points.last().copied();
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn camera_clips_before_f32_and_gestures_preserve_a_finite_unit_frame() {
        let mut camera = MapCamera {
            orbit: DQuat::IDENTITY,
            half_height_m: 10.0,
            ..Default::default()
        };
        let extent = [100.0, 100.0];
        assert_eq!(
            camera.project_segment(
                DVec3::new(-1e13, 0.0, 0.0),
                DVec3::new(1e13, 0.0, 0.0),
                extent
            ),
            Some([[0.0, 0.5], [1.0, 0.5]])
        );
        assert!(
            camera
                .project_segment(
                    DVec3::new(-20.0, 20.0, 0.0),
                    DVec3::new(20.0, 20.0, 0.0),
                    extent
                )
                .is_none()
        );
        camera.pan(10.0, -20.0, 100.0);
        assert_eq!(camera.center_m, DVec3::new(-2.0, -4.0, 0.0));
        camera.zoom(f64::MAX);
        assert_eq!(camera.half_height_m, 1.0);
        camera.zoom(-f64::MAX);
        assert_eq!(camera.half_height_m, 1e17);
        for _ in 0..10000 {
            camera.drag_orbit(5.0, -3.0);
        }
        assert!((camera.orbit.length_squared() - 1.0).abs() < 1e-12);
        let before = camera;
        camera.drag_orbit(f64::NAN, 1.0);
        camera.pan(f64::INFINITY, 0.0, 100.0);
        camera.zoom(f64::NAN);
        assert_eq!(camera, before);
    }
    #[test]
    fn rails_projection_uses_each_sample_epoch_and_keeps_only_future_bounded_points() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let e = system.bake().unwrap();
        let body = e.body_id("thessa").unwrap();
        let times: Vec<_> = (0..4000).map(|i| SimTime(i as f64 * 10.0)).collect();
        let relative = DVec3::new(10e6, 0.0, 0.0);
        let positions = times
            .iter()
            .map(|time| e.body_state(body, *time).unwrap().position_inertial + relative)
            .collect();
        let mut path = SampledPath {
            positions,
            velocities: Vec::new(),
            accelerations: Vec::new(),
            end_time: *times.last().unwrap(),
            times,
            end: SampledPathEnd::Impact(body),
            stats: Default::default(),
        };
        let now = SimTime(100.0);
        let craft = e.body_state(body, now).unwrap().position_inertial + relative;
        let projected = project_rails_path(&e, &path, body, now, craft, f64::INFINITY);
        assert!(projected.points.len() <= 1025);
        assert!(
            projected
                .points
                .iter()
                .all(|point| point.distance(relative) < 0.001)
        );
        assert_eq!(projected.impact, projected.points.last().copied());
        let indices = future_path_indices(&path.times, now, 1024);
        assert_eq!(indices[0], 11);
        assert_eq!(indices.last(), Some(&3999));
        assert!(indices.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(future_path_indices(&path.times, now, 1).is_empty());
        path.positions[3999] += DVec3::splat(1e12);
        let clipped = project_rails_path(&e, &path, body, now, craft, 1e9);
        assert!(clipped.impact.is_none());
        assert!(
            project_rails_path(&e, &path, body, SimTime(-1.0), craft, 1e9)
                .points
                .is_empty()
        );
        assert!(
            project_rails_path(&e, &path, body, SimTime(40000.0), craft, 1e9)
                .points
                .is_empty()
        );
        assert!(
            project_rails_path(&e, &path, body, now, DVec3::NAN, 1e9)
                .points
                .is_empty()
        );
        path.positions.clear();
        assert!(
            project_rails_path(&e, &path, body, now, craft, 1e9)
                .points
                .is_empty()
        );
    }
    #[test]
    fn checked_system_scopes_and_camera_precision_are_bounded() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        for (mode, count) in MapMode::ALL.into_iter().zip([30, 17, 9, 3, 3, 13]) {
            let focus = ephemeris.body_id(mode.focus_name()).unwrap();
            assert_eq!(
                ephemeris
                    .bodies
                    .iter()
                    .filter(|body| body.radius_m > 0.0 && visible(&ephemeris, mode, focus, body.id))
                    .count(),
                count
            );
        }
        let camera = MapCamera {
            center_m: DVec3::splat(1e13),
            orbit: DQuat::IDENTITY,
            half_height_m: 10.0,
        };
        let pixel = camera
            .project(
                camera.center_m + DVec3::new(0.03125, 0.0625, 0.0),
                [1280.0, 720.0],
            )
            .unwrap();
        assert!((pixel[0] - 0.50087890625).abs() < 1e-15);
        assert!((pixel[1] - 0.496875).abs() < 1e-15);
        assert!(camera.project(DVec3::NAN, [1280.0, 720.0]).is_none());
        assert!(camera.project(DVec3::ZERO, [0.0, 720.0]).is_none());
    }
    #[test]
    fn adaptive_paths_close_with_a_hard_vertex_budget_and_dense_oracle() {
        let system: thessa_sim_core::SystemConfig =
            toml::from_str(include_str!("../../../data/system.toml")).unwrap();
        let ephemeris = system.bake().unwrap();
        for orbit in ephemeris.bodies.iter().filter_map(|body| body.orbit) {
            let points = kepler_polyline(orbit, 512, orbit.semi_major_axis_m * 0.00075);
            assert!(points.len() <= 513);
            assert!(points[0].distance(*points.last().unwrap()) < orbit.semi_major_axis_m * 1e-12);
            let mut max_error = 0.0_f64;
            for i in 0..2048 {
                let p = orbit
                    .state_relative_at(SimTime(orbit.period_s() * i as f64 / 2048.0))
                    .unwrap()
                    .0;
                let error = points
                    .windows(2)
                    .map(|w| {
                        let edge = w[1] - w[0];
                        p.distance(
                            w[0] + edge
                                * ((p - w[0]).dot(edge) / edge.length_squared()).clamp(0.0, 1.0),
                        )
                    })
                    .fold(f64::INFINITY, f64::min);
                max_error = max_error.max(error);
            }
            assert!(
                max_error < orbit.semi_major_axis_m * 0.001,
                "absolute error bound is 0.001a; error={max_error} a={}",
                orbit.semi_major_axis_m
            );
        }
    }
}
