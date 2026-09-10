//! Analytic single-scattering queries shared by raster and ray-aware backends.
//!
//! The same functions validate the shader approximation, feed future LUT
//! baking, and answer ray-aware queries (camera rays, light/shadow rays,
//! reflection probes) without tracing individual particles (spec §4, §13).
//! Numeric integration with fixed step counts: deterministic in the inputs,
//! therefore warp-safe.

use glam::DVec3;

use crate::lights::CelestialLight;
use crate::optics::AtmosphereOptics;

/// Rayleigh phase function: 3/(16π)·(1+cos²θ).
pub fn rayleigh_phase(cos_theta: f64) -> f64 {
    3.0 / (16.0 * std::f64::consts::PI) * (1.0 + cos_theta * cos_theta)
}

/// Henyey-Greenstein phase for Mie-like haze.
pub fn hg_phase(asymmetry_g: f64, cos_theta: f64) -> f64 {
    let g = asymmetry_g.clamp(-0.89, 0.89);
    let g2 = g * g;
    let denom = (1.0 + g2 - 2.0 * g * cos_theta).powf(1.5).max(1e-9);
    (1.0 - g2) / (4.0 * std::f64::consts::PI * denom)
}

/// Total extinction coefficient (scattering + absorption) at a point, per RGB.
fn extinction_rgb(optics: &AtmosphereOptics, point_m: DVec3) -> [f64; 3] {
    let mut sigma = [0.0; 3];
    let dr = optics.density_factor(optics.rayleigh.scale_height_m, point_m);
    let dm = optics.density_factor(optics.mie.scale_height_m, point_m);
    for (c, s) in sigma.iter_mut().enumerate() {
        *s += optics.rayleigh.beta_rgb[c] * dr + optics.mie.beta_rgb[c] * dm;
        for layer in &optics.absorption {
            *s += layer.sigma_rgb[c] * optics.density_factor(layer.scale_height_m, point_m);
        }
    }
    sigma
}

/// Scattering-only coefficient at a point: (rayleigh, mie) per RGB.
fn scattering_split(optics: &AtmosphereOptics, point_m: DVec3) -> ([f64; 3], [f64; 3]) {
    let dr = optics.density_factor(optics.rayleigh.scale_height_m, point_m);
    let dm = optics.density_factor(optics.mie.scale_height_m, point_m);
    let mut sr = [0.0; 3];
    let mut sm = [0.0; 3];
    for c in 0..3 {
        sr[c] = optics.rayleigh.beta_rgb[c] * dr;
        sm[c] = optics.mie.beta_rgb[c] * dm;
    }
    (sr, sm)
}

/// Ray/sphere intersection: nearest positive `t` interval, if any.
fn intersect_outer(optics: &AtmosphereOptics, origin_m: DVec3, dir: DVec3) -> Option<(f64, f64)> {
    let r = optics.outer_radius_m;
    let b = origin_m.dot(dir);
    let c = origin_m.length_squared() - r * r;
    let disc = b * b - c;
    if disc < 0.0 {
        return None;
    }
    let root = disc.sqrt();
    let t0 = -b - root;
    let t1 = -b + root;
    if t1 <= 0.0 {
        return None;
    }
    Some((t0.max(0.0), t1))
}

/// Spectral transmittance along a view segment (fraction surviving per channel).
///
/// Stable for cameras below, inside, near, and far outside the shell: rays
/// missing the shell return unity, clamped integration avoids overflow.
pub fn transmittance(
    optics: &AtmosphereOptics,
    camera_m: DVec3,
    view_dir: DVec3,
    steps: u32,
) -> [f64; 3] {
    let dir = view_dir.normalize_or_zero();
    if dir == DVec3::ZERO {
        return [1.0; 3];
    }
    let Some((t0, t1)) = intersect_outer(optics, camera_m, dir) else {
        return [1.0; 3];
    };
    let n = steps.clamp(1, 512) as usize;
    let dt = ((t1 - t0) / n as f64).max(0.0);
    let mut tau = [0.0_f64; 3];
    for i in 0..n {
        let p = camera_m + dir * (t0 + (i as f64 + 0.5) * dt);
        let sigma = extinction_rgb(optics, p);
        for c in 0..3 {
            tau[c] += sigma[c] * dt;
        }
    }
    tau.map(|t| (-t.min(50.0)).exp())
}

/// Single-scattering sky radiance plus view transmittance.
///
/// Marches the view ray, accumulating Rayleigh + Mie inscatter from every
/// visible light. Returns radiance in arbitrary consistent units shared with
/// the raster shader reference (same math, fewer steps in-shader).
#[derive(Debug, Clone, Copy)]
pub struct SkyResult {
    pub radiance_rgb: [f64; 3],
    pub transmittance_rgb: [f64; 3],
}

pub fn sky_radiance(
    optics: &AtmosphereOptics,
    camera_m: DVec3,
    view_dir: DVec3,
    lights: &[CelestialLight],
    view_steps: u32,
    sun_steps: u32,
) -> SkyResult {
    let dir = view_dir.normalize_or_zero();
    let mut result = SkyResult {
        radiance_rgb: [0.0; 3],
        transmittance_rgb: [1.0; 3],
    };
    if dir == DVec3::ZERO {
        return result;
    }
    let Some((t0, t1)) = intersect_outer(optics, camera_m, dir) else {
        return result;
    };
    let nv = view_steps.clamp(1, 256) as usize;
    let dt = ((t1 - t0) / nv as f64).max(0.0);
    let mut view_tau = [0.0_f64; 3];
    for i in 0..nv {
        let p = camera_m + dir * (t0 + (i as f64 + 0.5) * dt);
        let step_tau = extinction_rgb(optics, p).map(|s| (s * dt).min(10.0));
        let (sr, sm) = scattering_split(optics, p);
        // Transmittance from the sample toward the camera.
        let t_view = view_tau.map(|t| (-t).exp());
        for light in lights {
            let w = light.weighted_rgb();
            if w == [0.0; 3] {
                continue;
            }
            let sun_t = transmittance(optics, p, light.direction_to_star, sun_steps);
            let cos_theta = dir.dot(light.direction_to_star).clamp(-1.0, 1.0);
            let pr = rayleigh_phase(cos_theta);
            let pm = hg_phase(optics.mie.asymmetry_g, cos_theta);
            for c in 0..3 {
                let scatter = sr[c] * pr + sm[c] * pm;
                result.radiance_rgb[c] += t_view[c] * scatter * w[c] * sun_t[c] * dt;
            }
        }
        for c in 0..3 {
            view_tau[c] += step_tau[c];
        }
    }
    result.transmittance_rgb = view_tau.map(|t| (-t.min(50.0)).exp());
    // Weak night-side airglow limb term (spec section 11).
    let glow = airglow_radiance(optics, camera_m, dir);
    for (rad, g) in result.radiance_rgb.iter_mut().zip(glow) {
        *rad += g;
    }
    result
}

/// Aerial perspective between two points: surviving fraction plus inscatter
/// toward `light`. Used for distant terrain haze.
pub fn aerial_perspective(
    optics: &AtmosphereOptics,
    from_m: DVec3,
    to_m: DVec3,
    light: &CelestialLight,
    steps: u32,
) -> ([f64; 3], [f64; 3]) {
    let delta = to_m - from_m;
    let dist = delta.length();
    if dist <= 0.0 {
        return ([1.0; 3], [0.0; 3]);
    }
    let dir = delta / dist;
    let n = steps.clamp(1, 256) as usize;
    let dt = dist / n as f64;
    let w = light.weighted_rgb();
    let mut tau = [0.0_f64; 3];
    let mut inscatter = [0.0; 3];
    for i in 0..n {
        let p = from_m + dir * ((i as f64 + 0.5) * dt);
        let sigma = extinction_rgb(optics, p);
        let t_here = tau.map(|t| (-t).exp());
        if w != [0.0; 3] {
            let sun_t = transmittance(optics, p, light.direction_to_star, 8);
            let cos_theta = dir.dot(light.direction_to_star).clamp(-1.0, 1.0);
            let (sr, sm) = scattering_split(optics, p);
            let pr = rayleigh_phase(cos_theta);
            let pm = hg_phase(optics.mie.asymmetry_g, cos_theta);
            for c in 0..3 {
                inscatter[c] += t_here[c] * (sr[c] * pr + sm[c] * pm) * w[c] * sun_t[c] * dt;
            }
        }
        for c in 0..3 {
            tau[c] += (sigma[c] * dt).min(10.0);
        }
    }
    (tau.map(|t| (-t.min(50.0)).exp()), inscatter)
}

/// Weak emissive limb term, strongest looking tangentially through the upper
/// shell on the night side. Deterministic in geometry; no time dependence.
pub fn airglow_radiance(optics: &AtmosphereOptics, camera_m: DVec3, view_dir: DVec3) -> [f64; 3] {
    let dir = view_dir.normalize_or_zero();
    let Some((_, t1)) = intersect_outer(optics, camera_m, dir) else {
        return [0.0; 3];
    };
    // Grazing rays traverse more emissive shell: approximate with the
    // fraction of the segment spent above one scale height.
    let h_cam = optics.height_m(camera_m);
    let grazing =
        (1.0 - (h_cam / (optics.outer_radius_m - optics.inner_radius_m)).clamp(0.0, 1.0)).max(0.0);
    let k = 2.0e-7 * (t1 * 0.02).min(1.0) * (0.25 + 0.75 * grazing);
    [
        optics.emission.airglow_rgb[0] * k,
        optics.emission.airglow_rgb[1] * k,
        optics.emission.airglow_rgb[2] * k,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eclipse::eclipse_visibility;
    use crate::lights::blackbody_rgb;
    use crate::optics::nitrogen_oxygen_optics;
    use std::f64::consts::PI;

    fn thessa_optics() -> AtmosphereOptics {
        nitrogen_oxygen_optics(3_200_000.0, 120_000.0, 16_300.0).unwrap()
    }

    fn noon_light() -> CelestialLight {
        CelestialLight {
            direction_to_star: DVec3::Y,
            irradiance_w_m2: 600.0,
            color_rgb: blackbody_rgb(5100.0),
            angular_radius_rad: 0.01,
            visibility: 1.0,
        }
    }

    fn surface_camera() -> DVec3 {
        DVec3::new(0.0, 3_200_500.0, 0.0)
    }

    #[test]
    fn noon_zenith_is_blue_dominant() {
        let sky = sky_radiance(
            &thessa_optics(),
            surface_camera(),
            DVec3::Y,
            &[noon_light()],
            16,
            8,
        );
        let [r, g, b] = sky.radiance_rgb;
        assert!(b > g && g > r, "noon zenith must be blue: [{r}, {g}, {b}]");
    }

    #[test]
    fn sunset_horizon_shifts_red() {
        // Sun 2 degrees above the horizon, looking toward it.
        let elev = 2.0_f64.to_radians();
        let sun_dir = DVec3::new(elev.cos(), elev.sin(), 0.0);
        let mut light = noon_light();
        light.direction_to_star = sun_dir;
        let view = DVec3::new(1.0, 0.03, 0.0).normalize();
        let sky = sky_radiance(&thessa_optics(), surface_camera(), view, &[light], 16, 8);
        let [r, g, b] = sky.radiance_rgb;
        assert!(r > b, "sunset horizon must redden: [{r}, {g}, {b}]");
        let _ = g;
    }

    #[test]
    fn night_side_is_dark_airglow_only() {
        let mut light = noon_light();
        light.visibility = 0.0;
        let sky = sky_radiance(
            &thessa_optics(),
            surface_camera(),
            DVec3::Y,
            &[light],
            12,
            6,
        );
        let total: f64 = sky.radiance_rgb.iter().sum();
        let day = sky_radiance(
            &thessa_optics(),
            surface_camera(),
            DVec3::Y,
            &[noon_light()],
            12,
            6,
        );
        let day_total: f64 = day.radiance_rgb.iter().sum();
        assert!(
            total < day_total * 1e-3,
            "night must be ~dark: {total} vs {day_total}"
        );
    }

    #[test]
    fn terminator_is_smooth_and_finite() {
        let optics = thessa_optics();
        let mut prev: Option<[f64; 3]> = None;
        // Sweep the sun from -10 to +10 degrees elevation, fixed view.
        for i in 0..=100 {
            let elev = (-10.0 + 20.0 * i as f64 / 100.0).to_radians();
            let sun_dir = DVec3::new(elev.cos(), elev.sin(), 0.0);
            let mut light = noon_light();
            light.direction_to_star = sun_dir;
            let sky = sky_radiance(&optics, surface_camera(), DVec3::Y, &[light], 12, 6);
            for c in sky.radiance_rgb {
                assert!(c.is_finite() && c >= 0.0);
            }
            if let Some(p) = prev {
                let jump: f64 = sky
                    .radiance_rgb
                    .iter()
                    .zip(p.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();
                assert!(jump < 60.0, "terminator jump at step {i}: {jump}");
            }
            prev = Some(sky.radiance_rgb);
        }
    }

    #[test]
    fn eclipse_dims_sky_proportionally() {
        let optics = thessa_optics();
        let full = sky_radiance(&optics, surface_camera(), DVec3::Y, &[noon_light()], 12, 6);
        let mut eclipsed = noon_light();
        eclipsed.visibility = eclipse_visibility(DVec3::Y, 0.01, DVec3::Y, 0.05);
        assert_eq!(eclipsed.visibility, 0.0);
        let dark = sky_radiance(&optics, surface_camera(), DVec3::Y, &[eclipsed], 12, 6);
        let full_total: f64 = full.radiance_rgb.iter().sum();
        let dark_total: f64 = dark.radiance_rgb.iter().sum();
        assert!(dark_total < full_total * 1e-3);
    }

    #[test]
    fn queries_are_deterministic_in_inputs() {
        let optics = thessa_optics();
        let a = sky_radiance(&optics, surface_camera(), DVec3::Y, &[noon_light()], 12, 6);
        let b = sky_radiance(&optics, surface_camera(), DVec3::Y, &[noon_light()], 12, 6);
        assert_eq!(a.radiance_rgb, b.radiance_rgb);
    }

    #[test]
    fn all_camera_regimes_stay_finite() {
        let optics = thessa_optics();
        let r = optics.inner_radius_m;
        let positions = [
            DVec3::new(0.0, r + 500.0, 0.0),     // surface
            DVec3::new(0.0, r + 8_000.0, 0.0),   // inside
            DVec3::new(0.0, r + 200_000.0, 0.0), // near outer boundary
            DVec3::new(0.0, r * 4.0, 0.0),       // far outside
            DVec3::new(r * 4.0, 0.0, 0.0),       // far, limb view
        ];
        let views = [DVec3::Y, DVec3::NEG_Y, DVec3::X, DVec3::Z];
        for pos in positions {
            for view in views {
                let sky = sky_radiance(&optics, pos, view, &[noon_light()], 8, 4);
                for c in sky.radiance_rgb.into_iter().chain(sky.transmittance_rgb) {
                    assert!(c.is_finite() && c >= 0.0, "pos {pos} view {view}: {c}");
                }
            }
        }
        let _ = PI;
    }

    #[test]
    fn transmittance_orders_by_wavelength() {
        let t = transmittance(&thessa_optics(), surface_camera(), DVec3::Y, 16);
        assert!(t[0] > t[1] && t[1] > t[2], "red survives best: {t:?}");
        assert!(t.iter().all(|c| (0.0..=1.0).contains(c)));
    }
}
