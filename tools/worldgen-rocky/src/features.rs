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
    Archipelago {
        chain_length_m: f64,
        island_radius_m: f64,
        islands: u32,
    },
    VolcanicProvince {
        radius_m: f64,
        swell_m: f64,
        shields: u32,
    },
    SaltBasin {
        radius_m: f64,
        depth_m: f64,
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
            Feature::Archipelago {
                chain_length_m,
                island_radius_m,
                islands,
            } => {
                positive(chain_length_m, "chain")?;
                positive(island_radius_m, "island")?;
                if *islands == 0 || *islands > 64 {
                    return Err("islands must be 1..=64".into());
                }
            }
            Feature::VolcanicProvince {
                radius_m,
                swell_m,
                shields,
            } => {
                positive(radius_m, "radius")?;
                non_negative(*swell_m, "swell")?;
                if *shields > 32 {
                    return Err("shields must be <= 32".into());
                }
            }
            Feature::SaltBasin { radius_m, depth_m } => {
                positive(radius_m, "radius")?;
                positive(depth_m, "depth")?;
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
            Feature::Archipelago { chain_length_m, .. } => *chain_length_m,
            Feature::VolcanicProvince { radius_m, .. } => radius_m * 2.5,
            Feature::SaltBasin { radius_m, .. } => radius_m * 2.0,
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
    let center = crate::sphere::dir_from_latlon(center_lat, center_lon);
    let point = crate::sphere::dir_from_latlon(lat_deg, lon_deg);
    let angle = crate::sphere::angular_distance(center, point);
    if angle < 1e-12 {
        return (0.0, 0.0);
    }
    if angle.sin().abs() < 1e-12 {
        return (std::f64::consts::PI * radius_m, 0.0);
    }
    let (east, north, _) = crate::sphere::enu_basis(center);
    let dot = |v: [f64; 3]| point.iter().zip(v).map(|(a, b)| a * b).sum::<f64>();
    let scale = angle * radius_m / angle.sin();
    (dot(east) * scale, dot(north) * scale)
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
    let reach = pf.reach_m();
    let feather = 1.0 - smoothstep(reach * 0.65, reach, (dx * dx + dy * dy).sqrt());
    // Weathered, irregular footprints; broad features must not end at a hard
    // culling disk or a straight longitude boundary.
    let x = x + reach * 0.045 * rng::fbm(pf.seed, 610, x / reach * 7.0, y / reach * 7.0, 3);
    let y = y + reach * 0.045 * rng::fbm(pf.seed, 611, x / reach * 7.0, y / reach * 7.0, 3);
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
        Feature::Archipelago {
            chain_length_m,
            island_radius_m,
            islands,
        } => {
            // Chain of small volcanic cones along local x, seeded jitter.
            let mut h = 0.0;
            for i in 0..*islands {
                let t = if *islands > 1 {
                    f64::from(i) / f64::from(islands - 1) - 0.5
                } else {
                    0.0
                };
                let jx = rng::hash11(pf.seed, 31, i as i64, 0) * chain_length_m * 0.05;
                let jy = rng::hash11(pf.seed, 32, i as i64, 0) * chain_length_m * 0.08;
                let cx = t * chain_length_m + jx;
                let cy = jy;
                let d = ((x - cx).powi(2) + (y - cy).powi(2)).sqrt() / island_radius_m;
                // Cone above water + shallow shelf apron below datum.
                h += 1800.0 * (1.0 - d).max(0.0).powf(1.4)
                    - 500.0 * (-(d * 0.5).powi(2)).exp() * (1.0 - (1.0 - d).max(0.0));
            }
            h
        }
        Feature::VolcanicProvince {
            radius_m,
            swell_m,
            shields,
        } => {
            let r = (x * x + y * y).sqrt() / radius_m;
            let swell = swell_m * (-r.powi(2) * 1.2).exp();
            let mut cones = 0.0;
            for i in 0..*shields {
                let ax = rng::hash11(pf.seed, 33, i as i64, 0) * radius_m * 0.6;
                let ay = rng::hash11(pf.seed, 34, i as i64, 0) * radius_m * 0.6;
                let d = ((x - ax).powi(2) + (y - ay).powi(2)).sqrt() / (radius_m * 0.18);
                cones += swell_m * 1.4 * (1.0 - d).max(0.0).powf(1.6);
            }
            swell + cones
        }
        Feature::SaltBasin { radius_m, depth_m } => {
            // Flat-floored playa: steep rim descent, dead-flat interior.
            let r = (x * x + y * y).sqrt() / radius_m;
            -depth_m * smoothstep(1.0, 0.7, r)
        }
    };
    if out.is_finite() { out * feather } else { 0.0 }
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

#[cfg(test)]
mod composite_tests {
    use super::*;

    fn placed(feature: Feature) -> PlacedFeature {
        PlacedFeature {
            id: "t".into(),
            seed: 11,
            lat_deg: 0.0,
            lon_deg: 0.0,
            rotation_rad: 0.0,
            feature,
        }
    }

    #[test]
    fn archipelago_builds_islands_above_water() {
        let f = placed(Feature::Archipelago {
            chain_length_m: 400_000.0,
            island_radius_m: 30_000.0,
            islands: 5,
        });
        let r = 3_200_000.0;
        // Tallest point along the chain must clear the water.
        let mut peak = f64::NEG_INFINITY;
        for i in 0..41 {
            let lon_deg = (-200_000.0 + i as f64 * 10_000.0) / r * 180.0 / std::f64::consts::PI;
            peak = peak.max(eval_feature_height_m(&f, 0.0, lon_deg, r));
        }
        assert!(peak > 500.0, "island cone: {peak}");
    }

    #[test]
    fn salt_basin_floor_is_flat_and_below_rim() {
        let f = placed(Feature::SaltBasin {
            radius_m: 200_000.0,
            depth_m: 800.0,
        });
        let r = 3_200_000.0;
        let center = eval_feature_height_m(&f, 0.0, 0.0, r);
        let edge_lon = f64::to_degrees(190_000.0 / r);
        let edge = eval_feature_height_m(&f, 0.0, edge_lon, r);
        assert!((center + 800.0).abs() < 1.0, "flat floor: {center}");
        assert!(edge > center, "rim above floor");
    }

    #[test]
    fn volcanic_province_has_relief() {
        let f = placed(Feature::VolcanicProvince {
            radius_m: 300_000.0,
            swell_m: 1200.0,
            shields: 6,
        });
        let h = eval_feature_height_m(&f, 0.0, 0.0, 3_200_000.0);
        assert!(h > 500.0, "province relief: {h}");
        assert!(h.is_finite());
    }
}
