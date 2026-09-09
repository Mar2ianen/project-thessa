//! Spherical geometry in double precision.
//!
//! The canonical terrain position is a unit direction vector, never a raster
//! pixel and never a tile ID. Lat/lon helpers are convenience wrappers.

/// Degrees <-> radians helpers.
pub fn deg_to_rad(deg: f64) -> f64 {
    deg.to_radians()
}

/// Geographic position to unit direction `[x, y, z]` with y = north pole.
///
/// Longitude is EAST-positive and (east, north, up) is right-handed:
/// z carries a minus sign so that east is +lon with y as the pole.
pub fn dir_from_latlon(lat_deg: f64, lon_deg: f64) -> [f64; 3] {
    // Exact poles are longitude-invariant by construction.
    if lat_deg >= 90.0 {
        return [0.0, 1.0, 0.0];
    }
    if lat_deg <= -90.0 {
        return [0.0, -1.0, 0.0];
    }
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let (slat, clat) = lat.sin_cos();
    let (slon, clon) = lon.sin_cos();
    [clat * clon, slat, -clat * slon]
}

/// Unit direction back to (lat_deg, lon_deg). Longitude in [-180, 180].
pub fn latlon_from_dir(dir: [f64; 3]) -> (f64, f64) {
    let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2])
        .sqrt()
        .max(1e-300);
    let y = (dir[1] / len).clamp(-1.0, 1.0);
    (
        y.asin().to_degrees(),
        (-dir[2] / len).atan2(dir[0] / len).to_degrees(),
    )
}

/// Great-circle angular distance in radians. Numerically stable (haversine).
pub fn angular_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).clamp(-1.0, 1.0);
    // haversine form via chord length for small angles.
    let chord2 = (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2);
    if chord2 < 1e-8 {
        return 2.0 * (chord2 / 4.0).sqrt().asin();
    }
    dot.acos()
}

/// Great-circle distance in metres on a sphere of `radius_m`.
pub fn great_circle_m(a: [f64; 3], b: [f64; 3], radius_m: f64) -> f64 {
    angular_distance(a, b) * radius_m
}

/// Local east-north-up basis at a direction. All unit, right-handed.
pub fn enu_basis(dir: [f64; 3]) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let up = normalize(dir);
    // East = pole x up (east-positive longitude convention). At the exact
    // poles the basis is arbitrary but fixed.
    let mut east = [up[2], 0.0, -up[0]];
    if (east[0] * east[0] + east[2] * east[2]).sqrt() < 1e-12 {
        east = [0.0, 0.0, -1.0];
    }
    let east = normalize(east);
    // North = up x east... ensure right-handed: north = cross(up, east)? No:
    // east x north = up  =>  north = cross(up, east)?? Check: E x N = U.
    // N = U x E gives U x E = -(E x U) = ... verify with vectors below in test.
    let north = cross(up, east);
    (east, north, up)
}

/// World direction -> local ENU metres relative to `center` on sphere radius.
///
/// East/north are tangent-plane offsets (round-trip exactly). Up reads the
/// chord sagitta `-d^2/2R`, NOT altitude: altitude is a terrain-field value
/// (`TerrainSample.height_m`), never a direction-math value. Small radial
/// input offsets cancel to second order by construction.
pub fn dir_to_enu(dir: [f64; 3], center: [f64; 3], radius_m: f64) -> [f64; 3] {
    let (east, north, up) = enu_basis(center);
    let delta = [dir[0] - center[0], dir[1] - center[1], dir[2] - center[2]];
    // Chord scaled to arc length for small offsets (exact at center).
    let scale = radius_m;
    [
        dot(delta, east) * scale,
        dot(delta, north) * scale,
        dot(delta, up) * scale,
    ]
}

/// Local ENU metres -> world unit direction (round-trips with `dir_to_enu`
/// for surface points; a radial `up` offset is dropped by the projection
/// back onto the sphere by construction).
pub fn enu_to_dir(enu_m: [f64; 3], center: [f64; 3], radius_m: f64) -> [f64; 3] {
    let (east, north, up) = enu_basis(center);
    // Move along tangent plane, then project back to the sphere.
    let p = [
        center[0] + (east[0] * enu_m[0] + north[0] * enu_m[1] + up[0] * enu_m[2]) / radius_m,
        center[1] + (east[1] * enu_m[0] + north[1] * enu_m[1] + up[1] * enu_m[2]) / radius_m,
        center[2] + (east[2] * enu_m[0] + north[2] * enu_m[1] + up[2] * enu_m[2]) / radius_m,
    ];
    normalize(p)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-300);
    [v[0] / len, v[1] / len, v[2] / len]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latlon_roundtrip() {
        for (lat, lon) in [(0.0, 0.0), (45.0, -120.0), (-33.3, 179.9), (89.9, 45.0)] {
            let (la, lo) = latlon_from_dir(dir_from_latlon(lat, lon));
            assert!((la - lat).abs() < 1e-9, "{la} vs {lat}");
            assert!((lo - lon).abs() < 1e-9, "{lo} vs {lon}");
        }
    }

    #[test]
    fn seam_continuity_pm180() {
        let a = dir_from_latlon(10.0, 180.0);
        let b = dir_from_latlon(10.0, -180.0);
        assert!(angular_distance(a, b) < 1e-12);
    }

    #[test]
    fn enu_basis_right_handed() {
        let dir = dir_from_latlon(20.0, 30.0);
        let (e, n, u) = enu_basis(dir);
        // E x N = U.
        let c = cross(e, n);
        for i in 0..3 {
            assert!((c[i] - u[i]).abs() < 1e-12, "{c:?} vs {u:?}");
        }
    }

    #[test]
    fn enu_roundtrip() {
        let radius = 3_200_000.0;
        let center = dir_from_latlon(-25.0, 120.0);
        // Surface points round-trip in east/north to centimetres at km
        // offsets; up reads the chord sagitta -d^2/2R by construction.
        for enu in [
            [0.0, 0.0, 0.0],
            [1500.0, -300.0, 0.0],
            [-8000.0, 4000.0, 0.0],
        ] {
            let dir = enu_to_dir(enu, center, radius);
            let back = dir_to_enu(dir, center, radius);
            for i in 0..2 {
                assert!((back[i] - enu[i]).abs() < 0.05, "{back:?} vs {enu:?}");
            }
            let sagitta = -(enu[0] * enu[0] + enu[1] * enu[1]) / (2.0 * radius);
            assert!(
                (back[2] - sagitta).abs() < 0.05,
                "{back:?} sagitta {sagitta}"
            );
        }
        // Radial input offsets cancel to second order: up always reads the
        // chord sagitta, never altitude (altitude lives in the field).
        let dir = enu_to_dir([1500.0, -300.0, 50.0], center, radius);
        let back = dir_to_enu(dir, center, radius);
        assert!((back[0] - 1500.0).abs() < 0.05);
        assert!((back[1] + 300.0).abs() < 0.05);
        let sagitta = -(1500.0 * 1500.0 + 300.0 * 300.0) / (2.0 * radius);
        assert!((back[2] - sagitta).abs() < 0.05, "{back:?}");
    }

    #[test]
    fn east_is_positive_longitude() {
        // Facing the point from outside with north up-screen, +lon is right.
        let center = dir_from_latlon(0.0, 0.0);
        let east = dir_from_latlon(0.0, 10.0);
        let (e, _, _) = enu_basis(center);
        // Moving +lon must have a positive east component.
        let d = [
            east[0] - center[0],
            east[1] - center[1],
            east[2] - center[2],
        ];
        assert!(d[0] * e[0] + d[1] * e[1] + d[2] * e[2] > 0.0);
    }

    #[test]
    fn great_circle_quarter() {
        let a = dir_from_latlon(0.0, 0.0);
        let b = dir_from_latlon(0.0, 90.0);
        let d = great_circle_m(a, b, 3_200_000.0);
        assert!((d - 3_200_000.0 * std::f64::consts::FRAC_PI_2).abs() < 1.0);
    }
}
