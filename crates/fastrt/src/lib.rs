//! Sparse ray-tracing classification for FastRT (Stage 1, backend-neutral).
//!
//! The raster scene and the RT scene are not required to match
//! (`docs/39_BEVY_EXIT_AND_ENGINE_MIGRATION.md` §17): every candidate keeps
//! its raster representation, and this crate decides which RT
//! representation — if any — an accelerator should build for it. There is
//! no accelerator here yet (no BLAS/TLAS, no budgets, no dispatch); this is
//! the pure policy layer that later stages consume.
//!
//! Contract:
//!
//! - state: none, all functions are pure over caller-owned slices;
//! - inputs: [`Candidate`]s with caller-assigned `u64` keys, f64-SI bounds,
//!   visibility, requested RT effects, and triangle costs; an explicit
//!   [`Policy`] carrying the full-representation budget (no magic
//!   defaults — callers construct it from measured backend limits);
//! - outputs: one [`RtClass`] per candidate plus [`classify_frame`], which
//!   returns key-sorted results so identical inputs always produce
//!   identical outputs regardless of submission order.
//!
//! This crate deliberately knows nothing about Bevy, wgpu, Vulkan, meshes,
//! or effects. Units are SI metres (`f64`); triangle counts are exact
//! `u64` and only ever compared, never arithmetically combined, so
//! `u64::MAX` costs cannot overflow the policy.

#![forbid(unsafe_code)]

/// Axis-aligned bounds in metres. Invariant: every component is finite and
/// `min <= max` per axis. Degenerate (zero-volume) bounds are valid — a flat
/// quad still has a surface — only non-finite or inverted bounds are
/// rejected at construction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds3 {
    min: [f64; 3],
    max: [f64; 3],
}

impl Bounds3 {
    /// Returns `None` when any component is non-finite or `min > max` on
    /// any axis. Never panics.
    pub fn new(min: [f64; 3], max: [f64; 3]) -> Option<Self> {
        for (lo, hi) in min.iter().zip(max.iter()) {
            if !lo.is_finite() || !hi.is_finite() {
                return None;
            }
            if lo > hi {
                return None;
            }
        }
        Some(Self { min, max })
    }

    pub fn min(&self) -> [f64; 3] {
        self.min
    }

    pub fn max(&self) -> [f64; 3] {
        self.max
    }

    /// Smallest bounds enclosing both inputs. Inputs are valid by
    /// construction, so the output is always valid.
    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: std::array::from_fn(|axis| self.min[axis].min(other.min[axis])),
            max: std::array::from_fn(|axis| self.max[axis].max(other.max[axis])),
        }
    }

    /// Closed containment: boundary points count as inside, matching the
    /// slab intersector convention used by the tile/plume AABB helpers.
    pub fn contains_point(&self, point: [f64; 3]) -> bool {
        std::iter::zip(self.min, self.max)
            .zip(point)
            .all(|((lo, hi), p)| p.is_finite() && p >= lo && p <= hi)
    }

    /// Surface area in m². Used later to weigh proxy cost; degenerate axes
    /// contribute zero, they do not poison the sum.
    pub fn surface_area(&self) -> f64 {
        let extent = [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ];
        2.0 * (extent[0] * extent[1] + extent[1] * extent[2] + extent[2] * extent[0])
    }
}

/// Which representation the RT accelerator should build for a candidate.
/// Names follow `docs/39` §17.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtClass {
    /// Raster only; no RT representation is built.
    RasterOnly,
    /// Full representation goes to the accelerator whole.
    RtFull,
    /// A cheaper caller-supplied proxy goes instead of the full mesh.
    RtProxy,
    /// No static decision: the owning effect resolves per frame (e.g. no
    /// proxy exists but the full mesh exceeds budget).
    RtConditional,
    /// Culled upstream (invisible); neither raster nor RT may assume it.
    Ignore,
}

/// One classifiable piece of scene geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Candidate {
    /// Opaque caller key (same `u64`-identity style as the microstore and
    /// residency layers). Keys must be unique within a frame; duplicates
    /// are classified independently and both reported.
    pub key: u64,
    pub bounds: Bounds3,
    /// Culled before raster: never reaches any representation.
    pub visible: bool,
    /// Bitmask of RT effects requesting this geometry, one bit per effect;
    /// `0` means no effect wants it.
    pub rt_effects: u32,
    /// Triangle count of the full (raster) representation.
    pub full_tris: u64,
    /// Triangle count of the cheaper RT proxy; `0` means no proxy exists.
    pub proxy_tris: u64,
}

/// Explicit policy. Constructed by the caller from measured backend limits;
/// there is intentionally no `Default` so no budget hides in the library.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Policy {
    /// Full representations at or below this triangle count go to the
    /// accelerator whole ([`RtClass::RtFull`]).
    pub full_budget_tris: u64,
}

impl Policy {
    pub fn new(full_budget_tris: u64) -> Self {
        Self { full_budget_tris }
    }
}

/// Classify one candidate. Decision order is part of the contract:
///
/// 1. invisible → [`RtClass::Ignore`];
/// 2. no requested effects → [`RtClass::RasterOnly`];
/// 3. full cost within budget (inclusive) → [`RtClass::RtFull`];
/// 4. a strictly cheaper proxy exists → [`RtClass::RtProxy`];
/// 5. otherwise → [`RtClass::RtConditional`] (effect resolves per frame).
pub fn classify(candidate: &Candidate, policy: &Policy) -> RtClass {
    if !candidate.visible {
        return RtClass::Ignore;
    }
    if candidate.rt_effects == 0 {
        return RtClass::RasterOnly;
    }
    if candidate.full_tris <= policy.full_budget_tris {
        return RtClass::RtFull;
    }
    if candidate.proxy_tris > 0 && candidate.proxy_tris < candidate.full_tris {
        return RtClass::RtProxy;
    }
    RtClass::RtConditional
}

/// Classify a whole frame. Returns `(key, class)` pairs sorted by key, so
/// output is identical for identical candidate sets regardless of input
/// order. Runs in O(n log n) time in the sort; classification itself is a
/// single linear pass with no allocation besides the output vector.
pub fn classify_frame(candidates: &[Candidate], policy: &Policy) -> Vec<(u64, RtClass)> {
    let mut out: Vec<(u64, RtClass)> = candidates
        .iter()
        .map(|candidate| (candidate.key, classify(candidate, policy)))
        .collect();
    out.sort_by_key(|(key, _)| *key);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds(min: [f64; 3], max: [f64; 3]) -> Bounds3 {
        Bounds3::new(min, max).expect("test bounds must be valid")
    }

    fn candidate(key: u64) -> Candidate {
        Candidate {
            key,
            bounds: bounds([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
            visible: true,
            rt_effects: 0b11,
            full_tris: 100,
            proxy_tris: 10,
        }
    }

    #[test]
    fn bounds_rejects_nonfinite_and_inverted() {
        assert!(Bounds3::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).is_some());
        assert!(Bounds3::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]).is_some());
        assert!(Bounds3::new([f64::NAN, 0.0, 0.0], [1.0, 1.0, 1.0]).is_none());
        assert!(Bounds3::new([0.0, 0.0, 0.0], [1.0, f64::INFINITY, 1.0]).is_none());
        assert!(Bounds3::new([2.0, 0.0, 0.0], [1.0, 1.0, 1.0]).is_none());
        assert!(Bounds3::new([0.0, 0.0, -1.0], [1.0, 1.0, -2.0]).is_none());
    }

    #[test]
    fn bounds_union_contains_and_area() {
        let a = bounds([0.0, 0.0, 0.0], [1.0, 2.0, 3.0]);
        let b = bounds([-1.0, 1.0, 2.0], [0.5, 4.0, 5.0]);
        let u = a.union(&b);
        assert_eq!(u.min(), [-1.0, 0.0, 0.0]);
        assert_eq!(u.max(), [1.0, 4.0, 5.0]);
        assert!(u.contains_point([0.0, 0.0, 0.0]));
        assert!(u.contains_point([-1.0, 4.0, 5.0]));
        assert!(!u.contains_point([1.0, 4.0, 5.1]));
        assert!(!u.contains_point([f64::NAN, 0.0, 0.0]));
        // Unit cube: 6 faces of 1 m².
        assert_eq!(bounds([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).surface_area(), 6.0);
        // Flat quad 2x3: two faces of 6 m², degenerate axis contributes 0.
        assert_eq!(
            bounds([0.0, 0.0, 5.0], [2.0, 3.0, 5.0]).surface_area(),
            12.0
        );
        // Point bounds: zero area, still valid.
        assert_eq!(bounds([1.0, 1.0, 1.0], [1.0, 1.0, 1.0]).surface_area(), 0.0);
    }

    #[test]
    fn invisible_is_ignored_before_anything_else() {
        let policy = Policy::new(1_000_000);
        let mut c = candidate(1);
        c.visible = false;
        c.rt_effects = 0;
        c.full_tris = 0;
        assert_eq!(classify(&c, &policy), RtClass::Ignore);
    }

    #[test]
    fn no_effects_is_raster_only() {
        let policy = Policy::new(0);
        let mut c = candidate(1);
        c.rt_effects = 0;
        c.full_tris = u64::MAX;
        assert_eq!(classify(&c, &policy), RtClass::RasterOnly);
    }

    #[test]
    fn budget_boundary_is_inclusive() {
        let policy = Policy::new(100);
        assert_eq!(classify(&candidate(1), &policy), RtClass::RtFull);
        let over = Policy::new(99);
        assert_eq!(classify(&candidate(1), &over), RtClass::RtProxy);
    }

    #[test]
    fn proxy_must_be_strictly_cheaper() {
        let policy = Policy::new(10);
        let mut c = candidate(1);
        c.full_tris = 100;
        c.proxy_tris = 0;
        assert_eq!(classify(&c, &policy), RtClass::RtConditional);
        c.proxy_tris = 100;
        assert_eq!(classify(&c, &policy), RtClass::RtConditional);
        c.proxy_tris = 99;
        assert_eq!(classify(&c, &policy), RtClass::RtProxy);
    }

    #[test]
    fn extreme_costs_cannot_overflow() {
        let policy = Policy::new(u64::MAX);
        let mut c = candidate(1);
        c.full_tris = u64::MAX;
        c.proxy_tris = u64::MAX;
        assert_eq!(classify(&c, &policy), RtClass::RtFull);
        let zero = Policy::new(0);
        assert_eq!(classify(&c, &zero), RtClass::RtConditional);
    }

    #[test]
    fn frame_output_is_key_sorted_and_stable() {
        let policy = Policy::new(50);
        let mut hidden = candidate(7);
        hidden.visible = false;
        let mut plain = candidate(3);
        plain.rt_effects = 0;
        let big = candidate(9);
        let out = classify_frame(&[big, hidden, plain], &policy);
        assert_eq!(
            out,
            vec![
                (3, RtClass::RasterOnly),
                (7, RtClass::Ignore),
                (9, RtClass::RtProxy),
            ]
        );
        // Same set, shuffled input: identical output.
        let out2 = classify_frame(&[plain, big, hidden], &policy);
        assert_eq!(out, out2);
    }

    #[test]
    fn frame_regression_pins_documented_policy() {
        // Regression case: mixed fleet over a 1k budget. If this changes,
        // the policy decision order changed and docs/39 §17 needs an update.
        let policy = Policy::new(1_000);
        let terrain = Candidate {
            key: 11,
            bounds: bounds([-500.0, 0.0, -500.0], [500.0, 60.0, 500.0]),
            visible: true,
            rt_effects: 0b001,
            full_tris: 40_000,
            proxy_tris: 512,
        };
        let craft = Candidate {
            key: 4,
            bounds: bounds([-8.0, -2.0, -8.0], [8.0, 4.0, 8.0]),
            visible: true,
            rt_effects: 0b111,
            full_tris: 800,
            proxy_tris: 0,
        };
        let culled = Candidate {
            key: 9,
            bounds: bounds([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]),
            visible: false,
            rt_effects: 0b111,
            full_tris: 10,
            proxy_tris: 0,
        };
        let out = classify_frame(&[terrain, craft, culled], &policy);
        assert_eq!(
            out,
            vec![
                (4, RtClass::RtFull),
                (9, RtClass::Ignore),
                (11, RtClass::RtProxy),
            ]
        );
    }
}
