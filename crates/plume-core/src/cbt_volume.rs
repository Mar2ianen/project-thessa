//! Plume-local RCBT adapter (`docs/38` section 7.1).
//!
//! The plume is NOT forced into an octree API. Binary CBT nodes address
//! plume-local subregions; a later residual-brick layer will hang payloads
//! off these addresses. Long rocket plumes are strongly anisotropic, so the
//! split policy refines the longest axis first and wastes no octree-style
//! 8-way fan-out.
//!
//! Coordinates: plume-local metres, +Z downstream from the nozzle exit,
//! radius from the axis. The root bound comes from the analytic profile
//! (its length and max station radius), so topology follows physics state,
//! never a hard-coded volume.

use thessa_rcbt_core::Node;

use crate::profile::AxialProfile;

/// Plume bound in local metres: axial `[0, length_m]`, radial disc
/// `[0, max_radius_m]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeBound {
    pub length_m: f64,
    pub max_radius_m: f64,
}

impl PlumeBound {
    /// Bound from an analytic profile. Empty profiles (engine off) yield a
    /// degenerate zero bound: there is nothing to refine.
    pub fn from_profile(profile: &AxialProfile) -> Self {
        let max_radius = profile
            .stations
            .iter()
            .map(|station| station.radius_m)
            .fold(0.0_f64, f64::max);
        Self {
            length_m: profile.length_m.max(0.0),
            max_radius_m: max_radius.max(0.0),
        }
    }

    pub fn is_degenerate(self) -> bool {
        !(self.length_m > 0.0) || !(self.max_radius_m > 0.0)
    }
}

/// One binary subregion: axial span × radial span (a hollow-cylinder shell
/// segment; `r0 == 0` is the solid core).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeRegion {
    pub z0_m: f64,
    pub z1_m: f64,
    pub r0_m: f64,
    pub r1_m: f64,
}

impl PlumeRegion {
    pub fn root(bound: PlumeBound) -> Self {
        Self {
            z0_m: 0.0,
            z1_m: bound.length_m.max(0.0),
            r0_m: 0.0,
            r1_m: bound.max_radius_m.max(0.0),
        }
    }

    pub fn axial_len_m(self) -> f64 {
        (self.z1_m - self.z0_m).max(0.0)
    }

    pub fn radial_len_m(self) -> f64 {
        (self.r1_m - self.r0_m).max(0.0)
    }

    /// Split axis: the longer extent wins (ties go axial, matching the
    /// documented root→Z→Z→radial example). Returns `true` for axial.
    pub fn split_axial(self) -> bool {
        self.axial_len_m() >= self.radial_len_m()
    }

    /// Bisect along the longest axis. Left child takes the lower half.
    pub fn split(self) -> [Self; 2] {
        if self.split_axial() {
            let mid = 0.5 * (self.z0_m + self.z1_m);
            [Self { z1_m: mid, ..self }, Self { z0_m: mid, ..self }]
        } else {
            let mid = 0.5 * (self.r0_m + self.r1_m);
            [Self { r1_m: mid, ..self }, Self { r0_m: mid, ..self }]
        }
    }
}

/// Region addressed by a binary CBT node: descend from the root, taking the
/// left child on a `0` path bit and the right child on `1`. Path bits are
/// read from the heap id below the root depth, mirroring the deterministic
/// addressing the terrain adapter uses for tiles.
pub fn region_for_node(bound: PlumeBound, node: Node) -> Option<PlumeRegion> {
    if bound.is_degenerate() {
        return None;
    }
    let mut region = PlumeRegion::root(bound);
    // Heap id bits from just below the root to the node depth select
    // left/right at each level.
    for level in 0..node.depth() {
        let shift = node.depth() - level - 1;
        let bit = (node.id() >> shift) & 1;
        let [left, right] = region.split();
        region = if bit == 0 { left } else { right };
    }
    Some(region)
}

/// Residual-error heuristic for refinement policy: axial emission gradient
/// across the region times region size. Smaller regions of a smooth field
/// score lower; the policy refines where this exceeds its threshold.
/// Analytic-only leaves (today: all of them) carry no brick.
pub fn residual_error_estimate(profile: &AxialProfile, region: PlumeRegion) -> f64 {
    let Some(a) = profile.evaluate(region.z0_m) else {
        return 0.0;
    };
    let Some(b) = profile.evaluate(region.z1_m) else {
        return 0.0;
    };
    let emission_delta: f64 = (0..3)
        .map(|channel| (a.emission_rgb[channel] - b.emission_rgb[channel]).abs())
        .sum();
    let size = region.axial_len_m() + region.radial_len_m();
    (emission_delta * size).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::build_axial_profile;
    use crate::source::tests::{sample_env_sea_level, sample_source};

    fn live_bound() -> PlumeBound {
        let profile = build_axial_profile(&sample_source(), &sample_env_sea_level(), 48).unwrap();
        PlumeBound::from_profile(&profile)
    }

    #[test]
    fn root_covers_bound_and_children_partition_it() {
        let bound = live_bound();
        assert!(!bound.is_degenerate());
        let root = PlumeRegion::root(bound);
        assert_eq!(root.z0_m, 0.0);
        assert_eq!(root.z1_m, bound.length_m);
        assert_eq!(root.r1_m, bound.max_radius_m);
        let [left, right] = root.split();
        // Partition: no gap, no overlap along the split axis.
        if root.split_axial() {
            assert_eq!(left.z1_m, right.z0_m);
            assert_eq!(left.r0_m, right.r0_m);
            assert_eq!(left.r1_m, right.r1_m);
        } else {
            assert_eq!(left.r1_m, right.r0_m);
        }
    }

    #[test]
    fn long_thin_plume_splits_axially_first() {
        let region = PlumeRegion {
            z0_m: 0.0,
            z1_m: 20.0,
            r0_m: 0.0,
            r1_m: 1.0,
        };
        assert!(region.split_axial());
        let [left, _] = region.split();
        assert_eq!(left.z1_m, 10.0);
    }

    #[test]
    fn node_addresses_are_deterministic_and_nested() {
        let bound = live_bound();
        let root = Node::root();
        let [left, right] = root.children().unwrap();
        let root_region = region_for_node(bound, root).unwrap();
        let left_region = region_for_node(bound, left).unwrap();
        let right_region = region_for_node(bound, right).unwrap();
        // Children nest exactly inside the parent.
        assert!(left_region.z0_m >= root_region.z0_m);
        assert!(left_region.z1_m <= root_region.z1_m);
        assert!(right_region.z0_m >= root_region.z0_m);
        assert!(right_region.z1_m <= root_region.z1_m);
        // Siblings are disjoint along the split axis.
        assert!((left_region.z1_m <= right_region.z0_m) || (left_region.r1_m <= right_region.r0_m));
        assert_eq!(region_for_node(bound, left), Some(left_region));
    }

    #[test]
    fn degenerate_bound_maps_to_nothing() {
        let bound = PlumeBound {
            length_m: 0.0,
            max_radius_m: 0.0,
        };
        assert!(bound.is_degenerate());
        assert_eq!(region_for_node(bound, Node::root()), None);
    }

    #[test]
    fn error_shrinks_with_refinement() {
        let profile = build_axial_profile(&sample_source(), &sample_env_sea_level(), 48).unwrap();
        let bound = PlumeBound::from_profile(&profile);
        let root_error = residual_error_estimate(&profile, PlumeRegion::root(bound));
        let [left, _] = PlumeRegion::root(bound).split();
        let child_error = residual_error_estimate(&profile, left);
        assert!(child_error <= root_error);
        assert!(root_error >= 0.0);
    }
}
