//! Deterministic landmark generators with recognizable shapes.
//!
//! Every size is in physical metres. No pixels, no generic fractal blobs:
//! each feature has an explicit profile (bowl + rim, cone + caldera,
//! path-following incision, ...). Branching/asymmetry comes from seeded
//! noise on the feature-local coordinate, never from global octaves.

use serde::{Deserialize, Serialize};

use crate::rng;

/// One landmark. Sizes in metres, amplitudes in metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Feature {
    MountainRange {
        length_m: f64,
        width_m: f64,
        height_m: f64,
        branches: u32,
    },
    RidgeChain {
        length_m: f64,
        width_m: f64,
        height_m: f64,
    },
    Plateau {
        radius_m: f64,
        height_m: f64,
    },
    Canyon {
        length_m: f64,
        width_m: f64,
        depth_m: f64,
        tributaries: u32,
    },
    Crater {
        rim_radius_m: f64,
        depth_m: f64,
        central_peak: bool,
        ejecta: bool,
    },
    ImpactBasin {
        radius_m: f64,
        depth_m: f64,
        rings: u32,
    },
    ShieldVolcano {
        radius_m: f64,
        height_m: f64,
        caldera: bool,
    },
    Caldera {
        radius_m: f64,
        depth_m: f64,
    },
    LavaField {
        radius_m: f64,
        thickness_m: f64,
    },
    DuneField {
        wavelength_m: f64,
        amplitude_m: f64,
        extent_m: f64,
    },
    GlacierValley {
        length_m: f64,
        width_m: f64,
        depth_m: f64,
    },
    Escarpment {
        length_m: f64,
        height_m: f64,
    },
}

/// A feature pinned to the globe.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacedFeature {
    pub id: String,
    pub seed: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Local +x axis bearing, radians east of north.
    pub rotation_rad: f64,
    #[serde(flatten)]
    pub feature: Feature,
}

impl PlacedFeature {
    pub fn validate(&self) -> Result<(), String> {
        if !self.lat_deg.is_finite() || !(-90.0..=90.0).contains(&self.lat_deg) {
            return Err(format!("feature {} bad lat", self.id));
        }
        if !self.lon_deg.is_finite() || !(-180.0..=180.0).contains(&self.lon_deg) {
            return Err(format!("feature {} bad lon", self.id));
        }
        if !self.rotation_rad.is_finite() {
            return Err(format!("feature {} bad rotation", self.id));
        }
        match &self.feature {
            Feature::MountainRange {
                length_m,
                width_m,
                height_m,
                ..
            }
            | Feature::RidgeChain {
                length_m,
                width_m,
                height_m,
            } => {
                positive(length_m, "length_m")?;
                positive(width_m, "width_m")?;
                positive(height_m, "height_m")?;
            }
            Feature::Plateau { radius_m, height_m } => {
                positive(radius_m, "radius_m")?;
                positive(height_m, "height_m")?;
            }
            Feature::Canyon {
                length_m,
                width_m,
                depth_m,
                ..
            }
            | Feature::GlacierValley {
                length_m,
                width_m,
                depth_m,
            } => {
                positive(length_m, "length_m")?;
                positive(width_m, "width_m")?;
                positive(depth_m, "depth_m")?;
            }
            Feature::Crater {
                rim_radius_m,
                depth_m,
                ..
            } => {
                positive(rim_radius_m, "rim")?;
                positive(depth_m, "depth")?;
            }
            Feature::ImpactBasin {
                radius_m, depth_m, ..
            } => {
                positive(radius_m, "radius")?;
                positive(depth_m, "depth")?;
            }
            Feature::ShieldVolcano {
                radius_m, height_m, ..
            } => {
                positive(radius_m, "radius")?;
                positive(height_m, "height")?;
            }
            Feature::Caldera { radius_m, depth_m } => {
                positive(radius_m, "radius")?;
                positive(depth_m, "depth")?;
            }
            Feature::LavaField {
                radius_m,
                thickness_m,
            } => {
                positive(radius_m, "radius")?;
                non_negative(*thickness_m, "thickness")?;
            }
            Feature::DuneField {
                wavelength_m,
                amplitude_m,
                extent_m,
            } => {
                positive(wavelength_m, "wavelength")?;
                non_negative(*amplitude_m, "amplitude")?;
                positive(extent_m, "extent")?;
            }
            Feature::Escarpment { length_m, height_m } => {
                positive(length_m, "length")?;
                positive(height_m, "height")?;
            }
        }
        Ok(())
    }

    /// Maximum reach in metres (for culling + readability checks).
    pub fn reach_m(&self) -> f64 {
        match &self.feature {
            Feature::MountainRange {
                length_m, width_m, ..
            }
            | Feature::RidgeChain {
                length_m, width_m, ..
            } => length_m.max(*width_m),
            Feature::Plateau { radius_m, .. }
            | Feature::Crater {
                rim_radius_m: radius_m,
                ..
            }
            | Feature::ImpactBasin { radius_m, .. }
            | Feature::ShieldVolcano { radius_m, .. }
            | Feature::Caldera { radius_m, .. }
            | Feature::LavaField { radius_m, .. } => radius_m * 2.5,
            Feature::Canyon {
                length_m, width_m, ..
            }
            | Feature::GlacierValley {
                length_m, width_m, ..
            } => length_m.max(*width_m) * 1.5,
            Feature::DuneField { extent_m, .. } => *extent_m,
            Feature::Escarpment { length_m, .. } => *length_m,
        }
    }
}

fn positive(value: &f64, name: &str) -> Result<(), String> {
    if value.is_finite() && *value > 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be positive finite"))
    }
}

fn non_negative(value: f64, name: &str) -> Result<(), String> {
    if value.is_finite() && value >= 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be finite non-negative"))
    }
}

/// Local east/north offsets in metres from feature center.
fn local_xy_m(
    lat_deg: f64,
    lon_deg: f64,
    center_lat: f64,
    center_lon: f64,
    radius_m: f64,
) -> (f64, f64) {
    let dlat = (lat_deg - center_lat).to_radians();
    let mut dlon = (lon_deg - center_lon).to_radians();
    if dlon > std::f64::consts::PI {
        dlon -= 2.0 * std::f64::consts::PI;
    }
    if dlon < -std::f64::consts::PI {
        dlon += 2.0 * std::f64::consts::PI;
    }
    (
        dlon * radius_m * center_lat.to_radians().cos(),
        dlat * radius_m,
    )
}

fn rotate(x: f64, y: f64, rotation_rad: f64) -> (f64, f64) {
    let (s, c) = rotation_rad.sin_cos();
    (x * c - y * s, x * s + y * c)
}

/// Height contribution in metres. Pure function of inputs.
pub fn eval_feature_height_m(pf: &PlacedFeature, lat_deg: f64, lon_deg: f64, radius_m: f64) -> f64 {
    let (dx, dy) = local_xy_m(lat_deg, lon_deg, pf.lat_deg, pf.lon_deg, radius_m);
    if dx * dx + dy * dy > pf.reach_m().powi(2) {
        return 0.0;
    }
    let (x, y) = rotate(dx, dy, pf.rotation_rad);
    let out = match &pf.feature {
        Feature::MountainRange {
            length_m,
            width_m,
            height_m,
            branches,
        } => {
            // Elongated asymmetric uplift + seeded branching ridges.
            let along = (-(x / (length_m * 0.5)).powi(2)).exp();
            let across = (-(y / (width_m * 0.5)).powi(2)).exp();
            let branch = 0.6
                + 0.4
                    * rng::fbm(
                        pf.seed,
                        21,
                        x / width_m * (2.0 + f64::from(*branches)),
                        y / width_m,
                        3,
                    );
            let asym = 1.0 + 0.25 * (x / length_m).tanh();
            height_m * along * across * branch.max(0.0) * asym
        }
        Feature::RidgeChain {
            length_m,
            width_m,
            height_m,
        } => {
            let along = (-(x / (length_m * 0.5)).powi(2)).exp();
            let across = (-(y / (width_m * 0.5)).powi(2)).exp();
            height_m * along * across
        }
        Feature::Plateau { radius_m, height_m } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            if r >= 1.2 {
                0.0
            } else {
                height_m * smoothstep(1.2, 0.75, r)
            }
        }
        Feature::Canyon {
            length_m,
            width_m,
            depth_m,
            tributaries,
        } => {
            let along = (-(x / (length_m * 0.5)).powi(2)).exp();
            let wobble = 0.7 + 0.3 * rng::fbm(pf.seed, 22, x / length_m * 6.0, 0.0, 2);
            let across = (-(y / (width_m * 0.5 * wobble)).powi(2)).exp();
            let trib = 1.0
                + 0.15
                    * f64::from(*tributaries)
                    * rng::fbm(pf.seed, 23, x / width_m, y / width_m, 2).max(0.0);
            -depth_m * along * across * trib
        }
        Feature::GlacierValley {
            length_m,
            width_m,
            depth_m,
        } => {
            // U-shaped valley: flat floor, steep sides.
            let along = (-(x / (length_m * 0.5)).powi(2)).exp();
            let r = (y / (width_m * 0.5)).abs();
            let profile = if r < 0.6 {
                1.0
            } else {
                smoothstep(1.6, 0.6, r)
            };
            -depth_m * along * profile
        }
        Feature::Crater {
            rim_radius_m,
            depth_m,
            central_peak,
            ejecta,
        } => crater_profile(
            x,
            y,
            *rim_radius_m,
            *depth_m,
            *central_peak,
            *ejecta,
            pf.seed,
        ),
        Feature::ImpactBasin {
            radius_m,
            depth_m,
            rings,
        } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            let bowl = -depth_m * (-r.powi(2) * 1.5).exp();
            let mut ring_sum = 0.0;
            for ring in 1..=*rings {
                let rr = f64::from(ring) / f64::from(rings + 1);
                let d = (r - rr).abs();
                ring_sum += depth_m * 0.12 * (-(d * 12.0).powi(2)).exp();
            }
            bowl + ring_sum
        }
        Feature::ShieldVolcano {
            radius_m,
            height_m,
            caldera,
        } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            // Wide low-slope cone: linear-ish falloff, not gaussian spike.
            let cone = height_m * (1.0 - r).max(0.0).powf(1.6);
            let pit = if *caldera {
                -height_m * 0.25 * (-(r * 6.0).powi(2)).exp()
            } else {
                0.0
            };
            cone + pit
        }
        Feature::Caldera { radius_m, depth_m } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            let rim = depth_m * 0.3 * (-((r - 1.0) * 6.0).powi(2)).exp();
            let floor = -depth_m * smoothstep(1.0, 0.55, r);
            rim + floor
        }
        Feature::LavaField {
            radius_m,
            thickness_m,
        } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            if r >= 1.0 {
                0.0
            } else {
                thickness_m
                    * (0.5 + 0.5 * rng::fbm(pf.seed, 24, x / radius_m * 8.0, y / radius_m * 8.0, 3))
            }
        }
        Feature::DuneField {
            wavelength_m,
            amplitude_m,
            extent_m,
        } => {
            let r = (x * x + y * y).sqrt() / extent_m;
            if r >= 1.0 {
                0.0
            } else {
                let phase = x / wavelength_m * std::f64::consts::TAU;
                amplitude_m
                    * (phase.sin() * 0.6
                        + 0.4 * rng::value_noise(pf.seed, 25, x / wavelength_m, y / wavelength_m))
                    * (1.0 - r)
            }
        }
        Feature::Escarpment { length_m, height_m } => {
            let along = (-(x / (length_m * 0.5)).powi(2)).exp();
            // Step function smoothed across strike.
            height_m * along * smoothstep(-0.5, 0.5, y / length_m * 8.0) - height_m * 0.5 * along
        }
    };
    if out.is_finite() { out } else { 0.0 }
}

fn crater_profile(
    x: f64,
    y: f64,
    rim_radius_m: f64,
    depth_m: f64,
    central_peak: bool,
    ejecta: bool,
    seed: u64,
) -> f64 {
    let r = (x * x + y * y).sqrt() / rim_radius_m;
    // Bowl interior.
    let bowl = if r < 1.0 {
        -depth_m * (1.0 - r * r)
    } else {
        0.0
    };
    // Raised rim just outside r=1.
    let rim = depth_m * 0.35 * (-((r - 1.0) * 5.0).powi(2)).exp();
    // Central peak for large/complex craters.
    let peak = if central_peak {
        depth_m * 0.4 * (-(r * 5.0).powi(2)).exp()
    } else {
        0.0
    };
    // Ejecta blanket: radially streaked, decaying to ~2.2 radii.
    let ejecta_h = if ejecta && (1.0..2.2).contains(&r) {
        let angle = y.atan2(x);
        let streak = 0.6 + 0.4 * rng::fbm(seed, 26, angle * 3.0, r * 4.0, 2);
        depth_m * 0.08 * (-(r - 1.0) * 2.2).exp() * streak.max(0.0)
    } else {
        0.0
    };
    bowl + rim + peak + ejecta_h
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crater() -> PlacedFeature {
        PlacedFeature {
            id: "t".into(),
            seed: 5,
            lat_deg: 0.0,
            lon_deg: 0.0,
            rotation_rad: 0.0,
            feature: Feature::Crater {
                rim_radius_m: 50_000.0,
                depth_m: 3000.0,
                central_peak: true,
                ejecta: true,
            },
        }
    }

    #[test]
    fn crater_bowl_below_rim() {
        let r = 3_200_000.0;
        let f = crater();
        let center = eval_feature_height_m(&f, 0.0, 0.0, r);
        // Rim point ~1 deg of rim radius east.
        let rim_lon = f64::to_degrees(50_000.0 / r);
        let rim = eval_feature_height_m(&f, 0.0, rim_lon, r);
        assert!(center < 0.0, "bowl below datum");
        assert!(rim > 0.0, "rim raised");
        assert!(center < rim);
    }

    #[test]
    fn features_are_deterministic_and_finite() {
        let f = crater();
        let a = eval_feature_height_m(&f, 0.2, 0.3, 3_200_000.0);
        assert_eq!(a, eval_feature_height_m(&f, 0.2, 0.3, 3_200_000.0));
        assert!(a.is_finite());
    }

    #[test]
    fn far_field_is_zero() {
        let f = crater();
        assert_eq!(eval_feature_height_m(&f, 45.0, 90.0, 3_200_000.0), 0.0);
    }

    #[test]
    fn shield_volcano_is_cone_with_caldera_pit() {
        let f = PlacedFeature {
            id: "v".into(),
            seed: 1,
            lat_deg: 10.0,
            lon_deg: 20.0,
            rotation_rad: 0.0,
            feature: Feature::ShieldVolcano {
                radius_m: 100_000.0,
                height_m: 4000.0,
                caldera: true,
            },
        };
        let r = 3_200_000.0;
        let summit = eval_feature_height_m(&f, 10.0, 20.0, r);
        assert!(
            summit > 2000.0 && summit < 4000.0,
            "caldera pit lowers summit: {summit}"
        );
    }

    #[test]
    fn validation_rejects_garbage() {
        let mut f = crater();
        f.lat_deg = 999.0;
        assert!(f.validate().is_err());
    }
}
