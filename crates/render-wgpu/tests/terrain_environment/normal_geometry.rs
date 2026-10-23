//! Independent single-bounce geometric reference, not a production normal model.
//! A periodic triangular height field has slopes +/-s, unit x period, peak at
//! x=0.5, and is extruded in y. Facets are opaque Lambertian with reflectance 0.5.
//! Exact first-peak visibility intervals are checked against explicit ray/segment
//! intersections before projected-area, reciprocity and passive-energy tests.
use glam::DVec3;
use std::f64::consts::PI;

const RHO: f64 = 0.5;

fn direction(mu: f64, phi: f64) -> DVec3 {
    let r = (1.0 - mu * mu).max(0.0).sqrt();
    DVec3::new(r * phi.cos(), r * phi.sin(), mu)
}

fn directions() -> Vec<DVec3> {
    [0.001, 0.01, 0.1, 0.5, 1.0]
        .into_iter()
        .flat_map(|mu| [0.0, 0.73, PI / 2.0, PI, PI * 1.3].map(|phi| direction(mu, phi)))
        .collect()
}

fn visible_intervals(s: f64, w: DVec3) -> [[f64; 2]; 2] {
    assert!(s >= 0.0 && w.z > 0.0);
    if s == 0.0 || w.x == 0.0 {
        return [[0.0, 0.5], [0.5, 1.0]];
    }
    // A first-hit ray clears the next periodic peak. All subsequent peaks are
    // lower relative to the rising ray. The visible subset is an exact interval
    // because both the source facet and the ray are linear in x.
    let width = (w.z / (s * w.x.abs() + w.z)).min(0.5);
    if w.x > 0.0 {
        [
            if w.z >= s * w.x { [0.0, 0.5] } else { [0.0; 2] },
            [0.5, 0.5 + width],
        ]
    } else {
        [
            [0.5 - width, 0.5],
            if w.z >= -s * w.x {
                [0.5, 1.0]
            } else {
                [0.5; 2]
            },
        ]
    }
}

fn ray_visible(s: f64, x: f64, w: DVec3) -> bool {
    // Independent explicit intersections, without the interval/peak formula.
    // Five periods cover the first forward peak for either ray direction;
    // later periods cannot re-occlude an upward ray above the first peak.
    let z = s * if x < 0.5 { x } else { 1.0 - x };
    let local_slope = if x < 0.5 { s } else { -s };
    if w.z - local_slope * w.x <= 0.0 {
        return false;
    }
    for period in -2..=2 {
        for [ax, az, ex, ez] in [
            [f64::from(period), 0.0, 0.5, s * 0.5],
            [f64::from(period) + 0.5, s * 0.5, 0.5, -s * 0.5],
        ] {
            let determinant = w.x * ez - w.z * ex;
            if determinant.abs() < 1e-15 {
                continue;
            }
            let dx = ax - x;
            let dz = az - z;
            let t = (dx * ez - dz * ex) / determinant;
            let u = (dx * w.z - dz * w.x) / determinant;
            if t > 1e-10 && (0.0..=1.0).contains(&u) {
                return false;
            }
        }
    }
    true
}

fn projected_area(s: f64, w: DVec3) -> f64 {
    [s, -s]
        .into_iter()
        .zip(visible_intervals(s, w))
        .map(|(slope, [a, b])| (b - a) * (w.z - slope * w.x).max(0.0))
        .sum()
}

fn brdf(s: f64, l: DVec3, v: DVec3) -> f64 {
    // Integrate joint-visible facet area * local cos_l * local cos_v, then
    // divide by projected macro areas. Local area/dx = sqrt(1+s*s); each local
    // normal cosine contributes the inverse of that length.
    let scale = (1.0 + s * s).sqrt();
    let sum: f64 = [s, -s]
        .into_iter()
        .zip(visible_intervals(s, l))
        .zip(visible_intervals(s, v))
        .map(|((slope, il), iv)| {
            let width = (il[1].min(iv[1]) - il[0].max(iv[0])).max(0.0);
            width * (l.z - slope * l.x).max(0.0) * (v.z - slope * v.x).max(0.0) / scale
        })
        .sum();
    RHO / PI * sum / (l.z * v.z)
}

fn reflectance(s: f64, l: DVec3, n_mu: usize, n_phi: usize) -> f64 {
    let mut total = 0.0;
    for i in 0..n_mu {
        let mu = (i as f64 + 0.5) / n_mu as f64;
        for j in 0..n_phi {
            let v = direction(mu, 2.0 * PI * (j as f64 + 0.5) / n_phi as f64);
            total += brdf(s, l, v) * mu;
        }
    }
    total * 2.0 * PI / (n_mu * n_phi) as f64
}

#[test]
fn projected_microgeometry_visibility_matches_independent_ray_intersections() {
    let mut cases = 0;
    for s in [0.0, 0.02, 0.2, 1.0] {
        for w in directions() {
            let intervals = visible_intervals(s, w);
            for i in 0..128 {
                let x = (f64::from(i) + 0.5) / 128.0;
                let [a, b] = intervals[usize::from(x >= 0.5)];
                assert_eq!(
                    a < x && x < b,
                    ray_visible(s, x, w),
                    "s={s} w={w:?} x={x} interval={a}..{b}"
                );
                cases += 1;
            }
            assert!(
                (projected_area(s, w) - w.z).abs() < 1e-14,
                "all first-hit facets must cover the macro projected area: s={s} w={w:?}"
            );
        }
    }
    assert_eq!(cases, 12_800);
}

#[test]
fn projected_microgeometry_is_reciprocal_and_flat_limit_is_lambertian() {
    let directions = directions();
    for s in [0.0, 0.02, 0.2, 1.0] {
        for &l in &directions {
            for &v in &directions {
                let f = brdf(s, l, v);
                assert!(f.is_finite() && f >= 0.0);
                assert!((f - brdf(s, v, l)).abs() < 1e-12);
                if s == 0.0 {
                    assert!((f - RHO / PI).abs() < 1e-14);
                }
            }
        }
    }
}

#[test]
fn projected_microgeometry_single_bounce_is_passive_and_quadrature_converges() {
    for s in [0.0, 0.02, 0.2, 1.0] {
        for (mu, phi) in [(0.001, 0.0), (0.1, 0.0), (0.5, PI / 2.0), (1.0, 0.0)] {
            let l = direction(mu, phi);
            let coarse = reflectance(s, l, 64, 128);
            let fine = reflectance(s, l, 128, 256);
            assert!((0.0..=RHO + 1e-12).contains(&fine));
            // Absolute reflected/input energy fraction for this finite fixture,
            // not a bound for arbitrary geometry/HDR or a relative near-zero error.
            assert!((coarse - fine).abs() < 0.0001);
            eprintln!(
                "projected_microgeometry s={s} mu={mu} phi={phi} rho={RHO} coarse={coarse} fine={fine} convergence={}",
                (coarse - fine).abs()
            );
        }
    }
}
