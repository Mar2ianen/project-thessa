//! Direct GPL port of the retained inhabited-land optical proxy. This bake is
//! presentation, not simulated electricity, settlement growth or activity.
use thessa_worldgen_rocky::civilization::InhabitedRegion;

const CITY_BASE_RADIUS_RAD: f64 = 0.012;
const CITY_RADIUS_POP_GAIN: f64 = 0.05;

fn hash01(x: u32, y: u32, seed: u64) -> f64 {
    let mut h = seed
        .wrapping_add((x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
        .wrapping_add((y as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9));
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^= h >> 31;
    (h as f64) / (u64::MAX as f64)
}

fn dir_from_uv(u: f64, v: f64) -> [f64; 3] {
    let lon = u * std::f64::consts::TAU;
    let lat = (0.5 - v) * std::f64::consts::PI;
    [lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin()]
}

fn settlement_gate(dir: [f64; 3], regions: &[InhabitedRegion], max_pop: u64) -> f64 {
    let mut gate: f64 = 0.0;
    for r in regions {
        let c = r.center_dir;
        let dot = (dir[0] * c[0] + dir[1] * c[1] + dir[2] * c[2]).clamp(-1.0, 1.0);
        let ang = dot.acos();
        let pop = (r.population as f64 / max_pop.max(1) as f64).cbrt();
        let weight = pop * r.suitability.clamp(0.0, 1.0);
        let radius = CITY_BASE_RADIUS_RAD + CITY_RADIUS_POP_GAIN * pop;
        let t = 1.0 - (ang / radius).clamp(0.0, 1.0);
        gate = gate.max(weight * t * t * (3.0 - 2.0 * t));
    }
    gate
}

/// Deterministic R8 mask, unchanged optical law from the retained client.
pub fn bake_city_mask(
    regions: &[InhabitedRegion],
    land_at: impl Fn([f64; 3]) -> bool,
    w: u32,
    h: u32,
) -> Vec<u8> {
    assert!(w > 0 && h > 0);
    let max_pop = regions
        .iter()
        .map(|r| r.population)
        .max()
        .unwrap_or(0)
        .max(1);
    let mut out = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        let v = (y as f64 + 0.5) / h as f64;
        for x in 0..w {
            let u = (x as f64 + 0.5) / w as f64;
            let dir = dir_from_uv(u, v);
            let gate = if land_at(dir) {
                settlement_gate(dir, regions, max_pop)
            } else {
                0.0
            };
            let speckle = hash01(x, y, 0xC17E);
            let lit = if speckle < gate {
                gate * (0.6 + 0.4 * hash01(x, y, 0x51E))
            } else {
                0.0
            };
            out.push((lit.clamp(0.0, 1.0) * 255.0) as u8);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region(dir: [f64; 3], population: u64) -> InhabitedRegion {
        InhabitedRegion {
            id: 1,
            center_dir: dir,
            population,
            height_m: 100.0,
            slope: 0.05,
            temperature_k: 285.0,
            water_distance_m: 5000.0,
            suitability: 0.9,
        }
    }

    #[test]
    fn bake_is_deterministic_and_empty_or_ocean_worlds_stay_dark() {
        let regions = [region([1.0, 0.0, 0.0], 1_000_000)];
        let a = bake_city_mask(&regions, |_| true, 64, 32);
        assert_eq!(a.len(), 64 * 32);
        assert_eq!(a, bake_city_mask(&regions, |_| true, 64, 32));
        assert!(
            bake_city_mask(&[], |_| true, 32, 16)
                .iter()
                .all(|v| *v == 0)
        );
        assert!(
            bake_city_mask(&regions, |_| false, 32, 16)
                .iter()
                .all(|v| *v == 0)
        );
    }

    #[test]
    fn settlement_gate_tracks_population_and_distance() {
        let regions = [region([1.0, 0.0, 0.0], 5_000_000)];
        assert!((settlement_gate([1.0, 0.0, 0.0], &regions, 5_000_000) - 0.9).abs() < 1e-9);
        assert_eq!(settlement_gate([-1.0, 0.0, 0.0], &regions, 5_000_000), 0.0);
        assert_eq!(settlement_gate([1.0, 0.0, 0.0], &[], 1), 0.0);
        let mut prev = 2.0;
        for i in 0..=10 {
            let lon = i as f64 * 0.01;
            let g = settlement_gate([lon.cos(), 0.0, -lon.sin()], &regions, 5_000_000);
            assert!(g <= prev + 1e-12);
            prev = g;
        }
    }

    fn metros() -> Vec<InhabitedRegion> {
        let mut regions = Vec::new();
        for lat_deg in [-45.0f64, 0.0, 45.0] {
            for lon_deg in [0.0f64, 60.0, 120.0, 180.0, 240.0, 300.0] {
                let lat = lat_deg.to_radians();
                let lon = lon_deg.to_radians();
                let mut r = region(
                    [lat.cos() * lon.cos(), lat.sin(), -lat.cos() * lon.sin()],
                    2_000_000,
                );
                r.id = regions.len() as u32 + 1;
                regions.push(r);
            }
        }
        regions
    }

    #[test]
    fn settled_land_stays_speckled_and_uv_matches_globe_mapping() {
        let mask = bake_city_mask(&metros(), |_| true, 256, 128);
        let lit = mask.iter().filter(|v| **v > 0).count() as f64 / mask.len() as f64;
        assert!(lit > 0.001 && lit < 0.25, "lit fraction {lit:.4}");
        let d = dir_from_uv(0.0, 0.5);
        assert!((d[0] - 1.0).abs() < 1e-12 && d[1].abs() < 1e-12);
        assert!((dir_from_uv(0.37, 0.0)[1] - 1.0).abs() < 1e-12);
        for (u, v) in [(0.0, 0.5), (0.25, 0.25), (0.7, 0.9), (0.99, 0.01)] {
            let d = dir_from_uv(u, v);
            assert!((d.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn retained_bake_bytes_match_the_legacy_fixture_at_all_tiers() {
        // Recorded by compiling the retained and native pure producers
        // independently and comparing every byte for this 18-metro fixture.
        for ([width, height], expected) in [
            ([128, 64], 0xea1722e8d5f85f50_u64),
            ([256, 128], 0x9b6f39222b2dbb42),
            ([512, 256], 0xb5633a1038d1de0f),
        ] {
            let mask = bake_city_mask(&metros(), |_| true, width, height);
            let hash = mask.iter().fold(0xcbf29ce484222325_u64, |h, b| {
                (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
            });
            assert_eq!(hash, expected);
        }
    }

    #[test]
    fn resolved_disabled_and_zero_gain_do_no_bake_work() {
        use thessa_graphics::{Capabilities, Quality, RequestedGraphics, ResolvedGraphicsSettings};
        let mut requested = RequestedGraphics::default();
        for (quality, size) in [
            (Quality::Low, [128, 64]),
            (Quality::Medium, [256, 128]),
            (Quality::High, [512, 256]),
        ] {
            requested.night_lights.quality = quality;
            let resolved =
                ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::default());
            assert_eq!(resolved.night_light_mask_size(), Some(size));
        }
        requested.night_lights.enabled = false;
        assert!(
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::default())
                .night_light_mask_size()
                .is_none()
        );
        for intensity in [f32::NAN, f32::INFINITY, -1.0] {
            assert!(
                ResolvedGraphicsSettings {
                    night_lights_enabled: true,
                    night_lights_intensity: intensity,
                    ..Default::default()
                }
                .night_light_mask_size()
                .is_none()
            );
        }
        requested.night_lights.enabled = true;
        requested.night_lights.intensity = 0.0;
        assert!(
            ResolvedGraphicsSettings::from_requested(&requested, &Capabilities::default())
                .night_light_mask_size()
                .is_none()
        );
    }

    #[test]
    #[ignore = "diagnostic: retained optical bake CPU cost; 18 metros, all land, all quality tiers"]
    fn night_mask_cost() {
        for [width, height] in [[128, 64], [256, 128], [512, 256]] {
            let regions = metros();
            let started = std::time::Instant::now();
            let mask = bake_city_mask(&regions, |_| true, width, height);
            eprintln!(
                "native_night_bake extent={width}x{height} regions=18 elapsed_ms={:.6} bytes={} lit={}; one startup bake, not frame cost or terrain sampling",
                started.elapsed().as_secs_f64() * 1000.0,
                mask.len(),
                mask.iter().filter(|v| **v > 0).count()
            );
            std::hint::black_box(mask);
        }
    }
}
