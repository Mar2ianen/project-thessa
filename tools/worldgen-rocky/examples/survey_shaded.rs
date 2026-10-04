//! Shaded survey frames (CPU diagnostic, not shipped rendering): relief
//! crops with sun diffuse + sky ambient, and an orthographic day-side disc
//! for orbit-view comparison. Lighting is a documented survey approximation
//! (sRGB-space lambert + constant ambient, no shadows/atmosphere).
//! Usage: survey_shaded --out <prefix> [--disc <lat> <lon>] [--sun-el <deg> --sun-az <deg>]
use thessa_worldgen_rocky::{appearance, field, spec_recipe};

fn dir_from_latlon(lat_deg: f64, lon_deg: f64) -> [f64; 3] {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    [lat.cos() * lon.cos(), lat.sin(), lat.cos() * lon.sin()]
}

/// Surface normal in local east/north/up from height central differences.
/// `min_wavelength_m` must match the render texel: sub-texel relief in the
/// endpoints would alias into per-pixel normal noise (checkerboard shading).
fn normal_enu(
    planet: &field::PlanetField,
    lat_deg: f64,
    lon_deg: f64,
    step_deg: f64,
    min_wavelength_m: f64,
) -> [f64; 3] {
    let h_e = planet.height_m(
        dir_from_latlon(lat_deg, lon_deg + step_deg),
        min_wavelength_m,
    );
    let h_w = planet.height_m(
        dir_from_latlon(lat_deg, lon_deg - step_deg),
        min_wavelength_m,
    );
    let h_n = planet.height_m(
        dir_from_latlon(lat_deg + step_deg, lon_deg),
        min_wavelength_m,
    );
    let h_s = planet.height_m(
        dir_from_latlon(lat_deg - step_deg, lon_deg),
        min_wavelength_m,
    );
    let m_per_deg = planet.params.radius_m * std::f64::consts::PI / 180.0;
    let dhdx = (h_e - h_w) / (2.0 * step_deg * m_per_deg);
    let dhdy = (h_n - h_s) / (2.0 * step_deg * m_per_deg);
    let inv = 1.0 / (dhdx * dhdx + dhdy * dhdy + 1.0).sqrt();
    [-dhdx * inv, -dhdy * inv, inv]
}

fn shade(albedo: [f32; 3], normal_enu: [f64; 3], sun_el_deg: f64, sun_az_deg: f64) -> [u8; 3] {
    let el = sun_el_deg.to_radians();
    let az = sun_az_deg.to_radians();
    // Sun in east/north/up, azimuth clockwise from north.
    let sun = [el.cos() * az.sin(), el.cos() * az.cos(), el.sin()];
    let diffuse =
        (normal_enu[0] * sun[0] + normal_enu[1] * sun[1] + normal_enu[2] * sun[2]).clamp(0.0, 1.0);
    // Warm daylight + cool sky ambient, survey-only approximation.
    let light = [1.0, 0.96, 0.90];
    let ambient = [0.30, 0.34, 0.42];
    std::array::from_fn(|i| {
        let v = albedo[i] as f64 * (ambient[i] + light[i] * diffuse);
        (v.clamp(0.0, 1.0) * 255.0) as u8
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut out = String::from("/home/chechulin/tmp/shaded");
    let mut disc: Option<(f64, f64)> = None;
    let mut sun_el = 45.0f64;
    let mut sun_az = 135.0f64;
    let mut sites: Vec<(String, f64, f64)> = vec![
        ("coast".into(), 23.0, 165.0),
        ("highlands".into(), 38.0, -171.0),
        ("volcanic".into(), -16.0, 159.0),
    ];
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--out" => {
                out = args[i + 1].clone();
                i += 1;
            }
            "--disc" => {
                disc = Some((args[i + 1].parse()?, args[i + 2].parse()?));
                i += 2;
            }
            "--sun-el" => {
                sun_el = args[i + 1].parse()?;
                i += 1;
            }
            "--sun-az" => {
                sun_az = args[i + 1].parse()?;
                i += 1;
            }
            "--site" => {
                sites.push((
                    args[i + 1].clone(),
                    args[i + 2].parse()?,
                    args[i + 3].parse()?,
                ));
                i += 3;
            }
            _ => {}
        }
        i += 1;
    }
    let recipe: spec_recipe::SpecRecipe =
        toml::from_str(include_str!("../../../data/worldgen/worldgen_recipe.toml"))?;
    let manifest = spec_recipe::manifest_from_spec(&recipe)?;
    let planet = field::field_from_manifest(&manifest)?;
    if let Some((c_lat, c_lon)) = disc {
        // Orthographic day-side disc, camera over (c_lat, c_lon).
        let size = 480_usize;
        let r_ang = 68.0_f64.to_radians();
        let mut ppm = format!("P6\n{size} {size}\n255\n").into_bytes();
        let c = dir_from_latlon(c_lat, c_lon);
        // Camera basis: view dir -c, up = north projected.
        let north = dir_from_latlon(c_lat + 1.0, c_lon);
        let up = {
            let d = north[0] * c[0] + north[1] * c[1] + north[2] * c[2];
            let mut u = [
                north[0] - d * c[0],
                north[1] - d * c[1],
                north[2] - d * c[2],
            ];
            let l = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
            u = [u[0] / l, u[1] / l, u[2] / l];
            u
        };
        let right = [
            c[1] * up[2] - c[2] * up[1],
            c[2] * up[0] - c[0] * up[2],
            c[0] * up[1] - c[1] * up[0],
        ];
        let el = sun_el.to_radians();
        let az = sun_az.to_radians();
        // Fixed sun in world frame derived from sub-camera angles.
        let sun_w = [
            c[0] * el.sin() + (right[0] * az.sin() + up[0] * az.cos()) * el.cos(),
            c[1] * el.sin() + (right[1] * az.sin() + up[1] * az.cos()) * el.cos(),
            c[2] * el.sin() + (right[2] * az.sin() + up[2] * az.cos()) * el.cos(),
        ];
        for r in 0..size {
            for cc in 0..size {
                let x = (cc as f64 / size as f64 - 0.5) * 2.0 * r_ang.tan();
                let y = (0.5 - r as f64 / size as f64) * 2.0 * r_ang.tan();
                let rr = x * x + y * y;
                if rr > 1.0 {
                    ppm.extend_from_slice(&[0, 0, 0]);
                    continue;
                }
                let z = (1.0 - rr).sqrt();
                let dir = [
                    c[0] * z + right[0] * x + up[0] * y,
                    c[1] * z + right[1] * x + up[1] * y,
                    c[2] * z + right[2] * x + up[2] * y,
                ];
                // Disc-center pixel footprint: 2*68 deg over 480 px, ~30 km.
                // Sampling and appearance share one texel so discrete
                // classification follows the rendered surface, not 32 m noise.
                let sample = planet.sample_surface(dir, 30000.0);
                let mat = appearance::surface_appearance_filtered(&planet, &sample, dir, 30000.0);
                // World-space normal from local ENU gradient is overkill per
                // disc pixel; use radial normal with terminator shading.
                let cos_inc = dir[0] * sun_w[0] + dir[1] * sun_w[1] + dir[2] * sun_w[2];
                let day = (cos_inc * 4.0).clamp(0.0, 1.0);
                let v: [u8; 3] = std::array::from_fn(|k| {
                    let lit = mat.albedo_srgb[k] as f64 * (0.06 + 0.94 * day);
                    (lit.clamp(0.0, 1.0) * 255.0) as u8
                });
                ppm.extend_from_slice(&v);
            }
        }
        let path = format!("{out}_disc.ppm");
        std::fs::write(&path, ppm)?;
        println!("wrote: {path} (sun_el={sun_el} sun_az={sun_az})");
        return Ok(());
    }
    // Shaded relief crops matching survey_frame bookmarks (plus --site extras).
    for (name, lat0, lon0) in sites {
        let (w, h) = (240_usize, 240_usize);
        let step = 0.15;
        let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
        for r in 0..h {
            for c in 0..w {
                let lat = lat0 + (h as f64 / 2.0 - r as f64) * step;
                let lon = lon0 + (c as f64 - w as f64 / 2.0) * step;
                let dir = dir_from_latlon(lat, lon);
                // Crop pixel footprint at 0.15 deg/px is ~16 km. Sample the
                // field at the render texel: finer detail would alias into
                // per-pixel classification speckle (salt/desert/rock flips).
                let sample = planet.sample_surface(dir, 16000.0);
                let mat = appearance::surface_appearance_filtered(&planet, &sample, dir, 16000.0);
                let n = normal_enu(&planet, lat, lon, 0.15, 16000.0);
                ppm.extend_from_slice(&shade(mat.albedo_srgb, n, sun_el, sun_az));
            }
            if r % 80 == 0 {
                eprintln!("{name} row {r}/{h}");
            }
        }
        let path = format!("{prefix}_{name}.ppm", prefix = out);
        std::fs::write(&path, ppm)?;
        println!("wrote: {path}");
    }
    Ok(())
}
