//! The existing client's selection, spawn and material dependency policies.
use glam::{DMat4, DVec3};
use std::{collections::BTreeSet, sync::Arc};
use thessa_worldgen_rocky::{
    field::PlanetField,
    lod::{self, TileKey},
};

#[derive(Clone, Copy)]
pub struct SelectionView {
    pub eye: DVec3,
    pub stream_origin: DVec3,
    pub forward: DVec3,
    pub radius: f64,
    pub frustum: Option<lod::SelectionFrustum>,
    /// Actual projection of camera-relative body axes. Material priority must
    /// not inherit the geometry cone's offscreen prefetch margin.
    pub material_clip_from_eye: Option<DMat4>,
}

impl PartialEq for SelectionView {
    fn eq(&self, other: &Self) -> bool {
        self.eye == other.eye
            && self.stream_origin == other.stream_origin
            && self.forward == other.forward
            && self.radius == other.radius
            && self.material_clip_from_eye == other.material_clip_from_eye
            && self.frustum.map(|f| (f.forward, f.cos_limit))
                == other.frustum.map(|f| (f.forward, f.cos_limit))
    }
}

struct SelectionMemo {
    source: Arc<PlanetField>,
    view: SelectionView,
    policy: SelectionPolicy,
    exact_cover: bool,
    detail_bias: f64,
    previous_fine: Vec<TileKey>,
}

/// Visual/cache budgets, not simulation laws. Defaults preserve the client.
#[derive(Clone, Copy, PartialEq)]
pub struct SelectionPolicy {
    pub coarse_level: u8,
    pub coarse_budget: usize,
    pub fine_level: u8,
    pub fine_budget: usize,
}
impl Default for SelectionPolicy {
    fn default() -> Self {
        Self {
            coarse_level: 7,
            coarse_budget: 96,
            fine_level: 17,
            fine_budget: 288,
        }
    }
}

#[derive(Default)]
pub struct SelectionHistory {
    at: f64,
    eye: DVec3,
    stream_origin: DVec3,
    forward: DVec3,
    valid: bool,
    memo: Option<SelectionMemo>,
    pub eye_speed_mps: f64,
}

pub struct Demand {
    pub wanted: Vec<TileKey>,
    pub fine: Vec<TileKey>,
}

impl SelectionHistory {
    pub fn detail_bias(&self) -> f64 {
        (1.0 + self.eye_speed_mps.max(0.0) / 100.0).clamp(1.0, 32.0)
    }

    /// Motion/debounce/hysteresis policy moved from `run_tile_selection`.
    /// `now_s` is a monotonic presentation clock, never authoritative sim time.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        field: &Arc<PlanetField>,
        view: SelectionView,
        now_s: f64,
        previous_fine: &[TileKey],
        empty: bool,
        exact_cover: bool,
        policy: SelectionPolicy,
    ) -> Option<Demand> {
        let mut moved = !self.valid;
        if self.valid {
            let dt = (now_s - self.at).max(1e-3);
            let speed = (view.stream_origin - self.stream_origin).length() / dt;
            self.eye_speed_mps += (speed - self.eye_speed_mps).clamp(-2000.0, 2000.0) * 0.25;
            let swing = (view.forward.normalize_or_zero() - self.forward).length();
            moved = (view.eye - self.eye).length() > 300.0 || swing > 0.12;
        }
        if !(now_s - self.at > 0.75 || moved || empty) || !(now_s - self.at >= 0.15 || empty) {
            return None;
        }
        let detail_bias = self.detail_bias();
        // Preserve the sampling/debounce clock even when immutable demand is
        // reused, so future motion never averages over the whole idle period.
        self.at = now_s;
        self.eye = view.eye;
        self.stream_origin = view.stream_origin;
        self.forward = view.forward.normalize_or_zero();
        self.valid = true;
        if !empty
            && self.memo.as_ref().is_some_and(|memo| {
                Arc::ptr_eq(&memo.source, field)
                    && memo.view == view
                    && memo.policy == policy
                    && memo.exact_cover == exact_cover
                    && memo.detail_bias == detail_bias
                    && memo.previous_fine == previous_fine
            })
        {
            return None;
        }

        let mut wanted = lod::select_tiles_with_height_and_frustum(
            view.eye.to_array(),
            view.radius,
            policy.coarse_level,
            policy.coarse_budget,
            |dir| field.height_m(dir, 32.0),
            None,
            detail_bias,
        );
        let fine = lod::select_tiles_with_history(
            view.eye.to_array(),
            view.radius,
            policy.fine_level,
            policy.fine_budget,
            |dir| field.height_m(dir, 32.0),
            view.frustum,
            detail_bias,
            previous_fine,
        );
        wanted.extend(fine.iter().copied());
        wanted.sort();
        wanted.dedup();
        if exact_cover {
            wanted = complete_terrain_cover(&wanted);
        }
        // Holding the Arc pins the immutable recipe, not an address that could
        // be reused after source replacement. Hysteresis input is part of the key.
        self.memo = Some(SelectionMemo {
            source: field.clone(),
            view,
            policy,
            exact_cover,
            detail_bias,
            previous_fine: previous_fine.to_vec(),
        });
        Some(Demand { wanted, fine })
    }
}

pub fn tiles_overlap(a: TileKey, b: TileKey) -> bool {
    if a.face != b.face {
        return false;
    }
    let (coarse, fine) = if a.level <= b.level { (a, b) } else { (b, a) };
    let shift = fine.level - coarse.level;
    coarse.x == fine.x >> shift && coarse.y == fine.y >> shift
}

/// Fill all siblings along a refinement path; never drop most of a coarse tile.
pub fn complete_terrain_cover(wanted: &[TileKey]) -> Vec<TileKey> {
    fn visit(key: TileKey, wanted: &[TileKey], cover: &mut Vec<TileKey>) {
        if wanted
            .iter()
            .any(|fine| fine.level > key.level && tiles_overlap(key, *fine))
        {
            for child in key.children() {
                visit(child, wanted, cover);
            }
        } else {
            cover.push(key);
        }
    }
    let roots: BTreeSet<_> = wanted
        .iter()
        .copied()
        .filter(|key| {
            !wanted
                .iter()
                .any(|other| other.level < key.level && tiles_overlap(*key, *other))
        })
        .collect();
    let mut cover = Vec::new();
    for root in roots {
        visit(root, wanted, &mut cover);
    }
    cover.sort();
    cover
}

pub fn spawn_dependencies(
    wanted: &[TileKey],
    exact_cover: bool,
    resident: impl Fn(TileKey) -> bool,
) -> BTreeSet<TileKey> {
    let mut dependencies: BTreeSet<_> = wanted.iter().copied().collect();
    if exact_cover {
        for key in wanted {
            let mut ancestor = key.parent();
            while let Some(parent) = ancestor {
                dependencies.insert(parent);
                if resident(parent) {
                    break;
                }
                ancestor = parent.parent();
            }
        }
    }
    dependencies
}

pub fn height_priority(
    field: &PlanetField,
    view: SelectionView,
    key: TileKey,
) -> (u8, bool, std::cmp::Reverse<u64>) {
    let dir = DVec3::from(key.direction(0.5, 0.5));
    let surface = view.radius + field.height_m(dir.to_array(), 32.0).max(0.0);
    let distance = (view.eye - dir * surface).length().max(1.0);
    (
        key.level.min(8),
        (dir * view.radius - view.eye)
            .normalize_or_zero()
            .dot(view.forward)
            < -0.15,
        std::cmp::Reverse((key.span_m(view.radius) / distance).to_bits()),
    )
}

pub fn material_view_selection(
    base: &[TileKey],
    view: SelectionView,
    pixel_angle: f64,
) -> Vec<TileKey> {
    material_view_selection_with_limit(base, view, pixel_angle, 17)
}

/// Bounded optical refinement; persistent CPU/GPU cache budgets are unchanged.
pub fn material_view_selection_with_limit(
    base: &[TileKey],
    view: SelectionView,
    pixel_angle: f64,
    max_level: u8,
) -> Vec<TileKey> {
    material_view_selection_at_height(base, view, pixel_angle, max_level, 0.0)
}

// Clip a conservative page sphere for material admission. The shared prefetch
// cover omits the material matrix and retains the wider geometry cone.
fn material_page_in_view(key: TileKey, view: SelectionView, surface_radius: f64) -> bool {
    let delta = DVec3::from(key.direction(0.5, 0.5)) * surface_radius - view.eye;
    let bound = key.span_m(view.radius) * 0.75;
    if let Some(clip) = view.material_clip_from_eye {
        return thessa_render_core::shadow::sphere_intersects_clip(clip, delta, bound);
    }
    let Some(frustum) = view.frustum else {
        return true;
    };
    let distance = delta.length();
    if distance <= bound {
        return true;
    }
    let sine = (bound / distance).clamp(0.0, 1.0);
    let cosine = (1.0 - sine * sine).sqrt();
    let limit = frustum.cos_limit.clamp(-1.0, 1.0);
    let threshold = if limit <= -cosine {
        -1.0
    } else {
        limit * cosine - (1.0 - limit * limit).sqrt() * sine
    };
    delta.dot(DVec3::from(frustum.forward)) / distance >= threshold
}

/// Optical-detail distance reference only. The caller obtains the local
/// elevation from the same canonical field used by geometry selection; this
/// does not move geometry or change the body datum/texel metric.
pub(crate) fn material_view_selection_at_height(
    base: &[TileKey],
    view: SelectionView,
    pixel_angle: f64,
    max_level: u8,
    reference_height_m: f64,
) -> Vec<TileKey> {
    let max_level = max_level.clamp(17, 24);
    let surface_radius = view.radius + reference_height_m;
    let mut cover: BTreeSet<_> = base.iter().copied().collect();
    let error = |key: TileKey| {
        let center = DVec3::from(key.direction(0.5, 0.5)) * surface_radius;
        let distance = ((center - view.eye).length() - key.span_m(view.radius) * 0.75).max(1.0);
        key.span_m(view.radius) / 125.0 / (distance * pixel_angle).max(0.001)
    };
    // Keep complete soil-height prefetch coverage in the wider cone, but spend
    // limited splits on unresolved actual-frame pages before offscreen pages.
    let prefetch_view = SelectionView {
        material_clip_from_eye: None,
        ..view
    };
    let split_candidate = |key: &TileKey| {
        key.level < max_level && material_page_in_view(*key, prefetch_view, surface_radius)
    };
    // More temporary leaves prevent the broad geometry cover from consuming
    // the entire near-floor refinement budget. Admission still retains 512.
    let leaf_budget = match max_level {
        17..=20 => 512,
        21 => 1024,
        _ => 2048,
    };
    while cover.len() + 3 <= leaf_budget {
        let candidate = cover
            .iter()
            .copied()
            .filter(|key| split_candidate(key) && error(*key) > 1.0)
            .max_by(|a, b| {
                let in_frame = |key| {
                    view.material_clip_from_eye.is_none()
                        || material_page_in_view(key, view, surface_radius)
                };
                in_frame(*a)
                    .cmp(&in_frame(*b))
                    .then_with(|| error(*a).total_cmp(&error(*b)))
            });
        let Some(key) = candidate else {
            break;
        };
        cover.remove(&key);
        cover.extend(key.children());
    }
    let mut pages: Vec<_> = cover.into_iter().collect();
    pages.sort_by(|a, b| error(*b).total_cmp(&error(*a)).then(a.cmp(b)));
    pages
}

/// Existing material scheduler order, shared by both application hosts.
pub fn material_demand(
    keys: impl IntoIterator<Item = TileKey>,
    view: SelectionView,
) -> Vec<TileKey> {
    material_demand_at_height(keys, view, 0.0)
}

pub(crate) fn material_demand_at_height(
    keys: impl IntoIterator<Item = TileKey>,
    view: SelectionView,
    reference_height_m: f64,
) -> Vec<TileKey> {
    let mut wanted: Vec<_> = keys
        .into_iter()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    wanted.sort_by_cached_key(|key| {
        let delta =
            DVec3::from(key.direction(0.5, 0.5)) * (view.radius + reference_height_m) - view.eye;
        (
            if view.material_clip_from_eye.is_some() || view.frustum.is_some() {
                !material_page_in_view(*key, view, view.radius + reference_height_m)
            } else {
                delta.normalize_or_zero().dot(view.forward) < -0.15
            },
            delta.length_squared().to_bits(),
            std::cmp::Reverse(key.level),
            *key,
        )
    });
    wanted.truncate(512);
    wanted
}

pub fn material_build_dependency(
    key: TileKey,
    mut resident: impl FnMut(TileKey) -> bool,
) -> TileKey {
    if resident(key) {
        return key;
    }
    let mut ancestor = key.parent();
    let mut next_level = key.level.saturating_sub(4);
    while let Some(candidate) = ancestor {
        if resident(candidate) {
            next_level = key.level.min(candidate.level + 2);
            break;
        }
        ancestor = candidate.parent();
    }
    let mut next = key;
    while next.level > next_level {
        next = next.parent().expect("material ancestor");
    }
    next
}

#[cfg(test)]
mod memo_tests {
    use super::*;

    #[test]
    fn immutable_demand_reuse_matches_uncached_selection_and_all_input_changes() {
        let source = crate::tests::field();
        let replacement = Arc::new(
            PlanetField::build(source.params.clone(), vec![], vec![], vec![], vec![]).unwrap(),
        );
        let mut cached = SelectionHistory::default();
        let mut reference = SelectionHistory::default();
        let mut view = crate::tests::view();
        let mut policy = SelectionPolicy {
            coarse_level: 2,
            coarse_budget: 24,
            fine_level: 4,
            fine_budget: 48,
        };
        let mut fine = Vec::new();
        let mut wanted = Vec::new();
        let mut hits = 0;
        for tick in 0..120 {
            match tick {
                24 => view.eye.x += 1.0,
                36 => view.forward = DVec3::new(0.2, 0.0, -1.0).normalize(),
                48 => {
                    view.frustum = Some(lod::SelectionFrustum {
                        forward: view.forward.to_array(),
                        cos_limit: 0.6,
                    })
                }
                60 => view.material_clip_from_eye = Some(DMat4::IDENTITY),
                72 => policy.fine_budget = 60,
                84 => view.stream_origin.x += 50.0,
                _ => {}
            }
            let field = if tick < 96 { &source } else { &replacement };
            let exact_cover = tick < 108;
            let now = tick as f64 * 0.3;
            reference.memo = None;
            let a = cached.update(field, view, now, &fine, tick == 0, exact_cover, policy);
            let b = reference.update(field, view, now, &fine, tick == 0, exact_cover, policy);
            if let Some(b) = b {
                if let Some(a) = a {
                    assert_eq!((&a.fine, &a.wanted), (&b.fine, &b.wanted), "tick={tick}");
                } else {
                    assert_eq!(
                        (&fine, &wanted),
                        (&b.fine, &b.wanted),
                        "cached demand went stale at tick={tick}"
                    );
                    hits += 1;
                }
                fine = b.fine;
                wanted = b.wanted;
            } else {
                assert!(a.is_none(), "debounce changed at tick={tick}");
            }
            assert_eq!(cached.detail_bias(), reference.detail_bias());
        }
        assert!(
            hits >= 8,
            "must witness reuse, not just matching recomputations: {hits}"
        );
        assert!(cached.memo.as_ref().unwrap().previous_fine.len() <= 60);
    }

    #[test]
    fn empty_demand_still_bootstraps_and_memo_pins_immutable_source() {
        let mut field = crate::tests::field();
        let mut history = SelectionHistory::default();
        let policy = SelectionPolicy {
            coarse_level: 1,
            coarse_budget: 12,
            fine_level: 2,
            fine_budget: 24,
        };
        let view = crate::tests::view();
        let a = history
            .update(&field, view, 0.0, &[], true, true, policy)
            .unwrap();
        let b = history
            .update(&field, view, 1.0, &a.fine, false, true, policy)
            .unwrap();
        assert_eq!(a.fine, b.fine);
        assert!(
            history
                .update(&field, view, 2.0, &b.fine, false, true, policy)
                .is_none()
        );
        assert!(
            history
                .update(&field, view, 3.0, &b.fine, true, true, policy)
                .is_some()
        );
        assert!(Arc::get_mut(&mut field).is_none());
    }
}
