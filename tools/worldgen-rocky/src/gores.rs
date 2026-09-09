//! Orange-slice gores: authoring/import format only.
//!
//! Runtime must never know about gores; it consumes baked spherical/tiled
//! data. This module validates gore sets and reconciles seams:
//! - continuous fields (height, roughness): smooth blend across overlap
//! - discrete IDs (biomes): NEVER blur; nearest/majority ownership wins

/// Layout of one gore set. All normalized fractions, no pixels.
#[derive(Debug, Clone, Copy)]
pub struct GoreLayout {
    pub count: u32,
    pub overlap_fraction: f64,
}

impl GoreLayout {
    pub fn validate(&self) -> Result<(), String> {
        if !(4..=16).contains(&self.count) {
            return Err("gore count must be 4..=16".into());
        }
        if !self.overlap_fraction.is_finite() || !(0.0..=0.25).contains(&self.overlap_fraction) {
            return Err("overlap must be within 0..=0.25".into());
        }
        Ok(())
    }

    /// Width of one gore in normalized longitude [0,1], including overlap.
    pub fn gore_width01(&self) -> f64 {
        1.0 / f64::from(self.count) * (1.0 + self.overlap_fraction)
    }

    /// Which gore owns a normalized longitude for discrete IDs.
    /// Ownership ignores overlap: hard cut at the mid-line, no blending.
    pub fn owner_for_lon01(&self, lon01: f64) -> u32 {
        let w = lon01.rem_euclid(1.0);
        ((w * f64::from(self.count)).floor() as u32).min(self.count - 1)
    }

    /// Blend weight of gore `g` at `lon01` for continuous fields.
    /// 1.0 deep inside, smoothstep ramp across the overlap band.
    pub fn blend_weight(&self, gore: u32, lon01: f64) -> f64 {
        let n = f64::from(self.count);
        let w = self.gore_width01();
        let start = f64::from(gore) / n - (w - 1.0 / n) / 2.0;
        // Distance from gore center, wrapped.
        let center = start + w / 2.0;
        let mut d = (lon01 - center).rem_euclid(1.0);
        if d > 0.5 {
            d = 1.0 - d;
        }
        let half = w / 2.0;
        let fade = half * self.overlap_fraction.max(1e-9);
        if d <= half - fade {
            1.0
        } else if d >= half {
            0.0
        } else {
            let t = (half - d) / fade;
            t * t * (3.0 - 2.0 * t)
        }
    }
}

/// Reconcile two neighbor samples of a continuous field.
/// Returns blended value; weights come from [`GoreLayout::blend_weight`].
pub fn reconcile_continuous(a: f64, b: f64, weight_a: f64) -> f64 {
    a * weight_a + b * (1.0 - weight_a)
}

/// Check neighbor agreement within tolerance (metres for height).
pub fn agree_within(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ownership_covers_globe_without_gaps() {
        let layout = GoreLayout {
            count: 8,
            overlap_fraction: 0.06,
        };
        layout.validate().unwrap();
        let mut owners = std::collections::HashSet::new();
        for i in 0..800 {
            owners.insert(layout.owner_for_lon01(i as f64 / 800.0));
        }
        assert_eq!(owners.len(), 8);
        // Wrap continuity: 0.999 and 0.001 are neighbours (7 and 0).
        assert_eq!(layout.owner_for_lon01(0.999), 7);
        assert_eq!(layout.owner_for_lon01(0.001), 0);
    }

    #[test]
    fn blend_is_one_inside_zero_outside() {
        let layout = GoreLayout {
            count: 8,
            overlap_fraction: 0.06,
        };
        // Center of gore 2.
        let center = (2.0 + 0.5) / 8.0;
        assert!((layout.blend_weight(2, center) - 1.0).abs() < 1e-12);
        // Far gore contributes nothing there.
        assert_eq!(layout.blend_weight(5, center), 0.0);
    }

    #[test]
    fn invalid_layouts_rejected() {
        assert!(
            GoreLayout {
                count: 2,
                overlap_fraction: 0.0
            }
            .validate()
            .is_err()
        );
        assert!(
            GoreLayout {
                count: 8,
                overlap_fraction: 0.9
            }
            .validate()
            .is_err()
        );
    }
}
