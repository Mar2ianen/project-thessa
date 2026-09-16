// Portable, render-only ocean reflection for the direct raster path.
//
// Contract
// --------
// `ocean_shade` returns a linear reflected contribution.  Add its result to
// the fragment's linear HDR accumulator before applying the view exposure.
// The sky arguments are radiances. `sun_illuminance` is the linear RGB
// directional illuminance in lux used by the existing scene lighting; its
// direct term is `f_r * NdotL * sun_illuminance`, matching that convention.
//
// `surface_direction` is the unit direction from the body centre to the
// fragment in canonical body space.  `geometric_normal`, `view_direction`,
// and `sun_direction` are render-space unit directions; view and sun point
// away from the surface. `body_to_render` is the rotation-only 3x3 part of
// the body's render transform. Keeping the wave phase on
// `surface_direction` avoids camera-relative/floating-origin discontinuities
// and does not mutate the authoritative terrain height field.
//
// OceanControls fields:
//   wave.x = base wave slope (metres/metre, dimensionless)
//   wave.y = wave number in radians per unit of body-direction chord
//   wave.z = frequency ratio of the second direction (normally ~1.73)
//   wave.w = slope ratio of the second direction (0..1)
//   appearance.x = perceptual GGX roughness (0..1)
//   appearance.y = dielectric normal-incidence reflectance F0 (normally ~0.02)
//   appearance.z = sky reflection multiplier
//   appearance.w = sun reflection multiplier
//
// `wave_phase` supplies two wrapped phases in radians.  The caller should
// derive them from simulation time (or precompute them on the CPU) and wrap
// them to a bounded interval.  This lets a paused simulation remain still and
// avoids a f32 timer accumulating phase error or jumping at a timer modulus.
//
// A body-direction wave number can be chosen from a physical wavelength as
// approximately `2*pi*body_radius / wavelength`.  This is a visual normal
// approximation: it does not displace vertices, alter collision, or model a
// physical spectrum.  The large_cbt reference's FFT simulation still costs
// its compute passes and memory when physically displaced wave geometry is
// desired.  This helper is deliberately the cheap fallback: two analytic
// directional components, with two cosine evaluations and no texture reads.

const OCEAN_PI: f32 = 3.141592653589793;
const OCEAN_EPSILON: f32 = 1.0e-6;

struct OceanControls {
    wave: vec4<f32>,
    appearance: vec4<f32>,
};

fn ocean_safe_normalize(value: vec3<f32>, fallback: vec3<f32>) -> vec3<f32> {
    let length_squared = dot(value, value);
    if (length_squared <= OCEAN_EPSILON) {
        return fallback;
    }
    return value * inverseSqrt(length_squared);
}

fn ocean_schlick(f0: f32, cosine: f32) -> f32 {
    let base = clamp(f0, 0.0, 1.0);
    let one_minus_cosine = 1.0 - clamp(cosine, 0.0, 1.0);
    let squared = one_minus_cosine * one_minus_cosine;
    return base + (1.0 - base) * squared * squared * one_minus_cosine;
}

// The directions are fixed in body space.  Projecting their gradients onto
// the current geometric tangent plane gives a continuous sphere-wide normal
// without a longitude seam or a pole-specific tangent basis.
fn ocean_wave_normal(
    surface_direction: vec3<f32>,
    geometric_normal: vec3<f32>,
    body_to_render: mat3x3<f32>,
    wave_phase: vec2<f32>,
    wave_footprint: f32,
    wave: vec4<f32>,
) -> vec3<f32> {
    let body_direction = ocean_safe_normalize(surface_direction, vec3(0.0, 1.0, 0.0));
    let body_normal = body_direction;
    let normal = ocean_safe_normalize(
        geometric_normal,
        ocean_safe_normalize(body_to_render * body_normal, body_normal),
    );
    let slope = max(wave.x, 0.0);
    let wave_number = max(wave.y, 0.0);
    if (slope <= OCEAN_EPSILON || wave_number <= OCEAN_EPSILON) {
        return normal;
    }

    let direction_a = ocean_safe_normalize(vec3(0.81, 0.17, 0.56), vec3(1.0, 0.0, 0.0));
    let direction_b = ocean_safe_normalize(vec3(-0.29, 0.34, 0.90), vec3(0.0, 0.0, 1.0));
    let frequency_ratio = max(wave.z, 0.01);
    let phase_a = dot(body_direction, direction_a) * wave_number + wave_phase.x;
    let phase_b = dot(body_direction, direction_b) * wave_number * frequency_ratio + wave_phase.y;

    // Interpolated body directions provide a cheap conservative footprint.
    // Smoothly remove slopes once a phase spans about a pixel, preventing
    // shimmer as coarse CBT leaves cover the distant ocean.
    let footprint = max(wave_footprint, 0.0);
    let filter_a = 1.0 - smoothstep(0.35, 1.5, wave_number * footprint);
    let filter_b = 1.0
        - smoothstep(0.35, 1.5, wave_number * frequency_ratio * footprint);

    // These are surface gradients.  Subtracting their normal component makes
    // the perturbation valid on a curved body instead of tilting the sphere's
    // radial normal toward the centre of the chosen plane wave.
    // `slope` is already the dimensionless height derivative (metres per
    // metre); wave number changes phase frequency without silently changing
    // the advertised wave steepness.
    let gradient = direction_a * (cos(phase_a) * slope * filter_a)
        + direction_b * (cos(phase_b) * slope * wave.w * filter_b);
    let tangent_gradient = gradient - body_normal * dot(body_normal, gradient);
    let render_gradient = body_to_render * tangent_gradient;
    return ocean_safe_normalize(normal - render_gradient, normal);
}

// A tiny analytic sky lookup used when the caller does not bind a cubemap.
// Zenith, horizon, and ground are linear radiance colours. `sky_up` is the
// render-space radial up for this planet surface, so the horizon follows the
// spherical body at every latitude instead of assuming global +Y.
fn ocean_sky_radiance(
    direction: vec3<f32>,
    sky_up: vec3<f32>,
    sky_zenith: vec3<f32>,
    sky_horizon: vec3<f32>,
    sky_ground: vec3<f32>,
) -> vec3<f32> {
    let sky_direction = ocean_safe_normalize(direction, vec3(0.0, 1.0, 0.0));
    let up = ocean_safe_normalize(sky_up, vec3(0.0, 1.0, 0.0));
    let elevation = dot(sky_direction, up);
    if (elevation < 0.0) {
        return sky_ground;
    }
    let t = smoothstep(0.0, 0.7, elevation);
    return mix(sky_horizon, sky_zenith, t);
}

// One bounded-cost GGX directional reflection.  `sun_illuminance` is the
// directional illuminance of a sun disk, so the result is BRDF * NdotL * E
// and remains compatible with the existing linear HDR lighting accumulator.
fn ocean_sun_reflection(
    normal: vec3<f32>,
    view: vec3<f32>,
    sun: vec3<f32>,
    sun_illuminance: vec3<f32>,
    roughness: f32,
    f0: f32,
) -> vec3<f32> {
    let n_dot_v = max(dot(normal, view), 0.0);
    let n_dot_l = max(dot(normal, sun), 0.0);
    if (n_dot_v <= OCEAN_EPSILON || n_dot_l <= OCEAN_EPSILON) {
        return vec3(0.0, 0.0, 0.0);
    }

    let half_vector = ocean_safe_normalize(view + sun, normal);
    let n_dot_h = max(dot(normal, half_vector), 0.0);
    let h_dot_v = max(dot(half_vector, view), 0.0);
    let perceptual_roughness = clamp(roughness, 0.02, 1.0);
    let alpha = max(perceptual_roughness * perceptual_roughness, 0.0025);
    let alpha_squared = alpha * alpha;
    let denominator = n_dot_h * n_dot_h * (alpha_squared - 1.0) + 1.0;
    let distribution = alpha_squared / max(OCEAN_PI * denominator * denominator, OCEAN_EPSILON);

    // Schlick's inexpensive Smith masking approximation.
    let k = (perceptual_roughness + 1.0) * (perceptual_roughness + 1.0) * 0.125;
    let visibility_v = n_dot_v / max(n_dot_v * (1.0 - k) + k, OCEAN_EPSILON);
    let visibility_l = n_dot_l / max(n_dot_l * (1.0 - k) + k, OCEAN_EPSILON);
    let fresnel = ocean_schlick(f0, h_dot_v);
    let brdf_times_cosine = fresnel * distribution * visibility_v * visibility_l * n_dot_l
        / max(4.0 * n_dot_v * n_dot_l, OCEAN_EPSILON);
    return sun_illuminance * brdf_times_cosine;
}

// Cheap reflected ocean contribution for a direct raster fragment shader.
// The caller can use `ocean_wave_normal` separately when it needs the normal
// for additional local lighting; this function computes it once internally.
fn ocean_shade(
    surface_direction: vec3<f32>,
    geometric_normal: vec3<f32>,
    body_to_render: mat3x3<f32>,
    view_direction: vec3<f32>,
    sun_direction: vec3<f32>,
    sun_illuminance: vec3<f32>,
    sky_zenith: vec3<f32>,
    sky_horizon: vec3<f32>,
    sky_ground: vec3<f32>,
    wave_phase: vec2<f32>,
    wave_footprint: f32,
    controls: OceanControls,
) -> vec3<f32> {
    let base_normal = ocean_safe_normalize(geometric_normal, vec3(0.0, 1.0, 0.0));
    let normal = ocean_wave_normal(
        surface_direction,
        base_normal,
        body_to_render,
        wave_phase,
        wave_footprint,
        controls.wave,
    );
    let view = ocean_safe_normalize(view_direction, base_normal);
    let sun_vector = ocean_safe_normalize(sun_direction, base_normal);
    let n_dot_v = max(dot(normal, view), 0.0);
    if (n_dot_v <= OCEAN_EPSILON) {
        return vec3(0.0, 0.0, 0.0);
    }

    let roughness = clamp(controls.appearance.x, 0.02, 1.0);
    let f0 = clamp(controls.appearance.y, 0.0, 1.0);
    let sky_strength = max(controls.appearance.z, 0.0);
    let sun_strength = max(controls.appearance.w, 0.0);
    let reflected_direction = reflect(-view, normal);
    // A single cone-centre sample is a stable low-cost approximation to a
    // rough reflection footprint.  The multiplier reduces the lobe's energy
    // as the cone widens; a bound sky cubemap can replace this approximation.
    let cone = roughness * roughness * 0.5;
    let sky_direction = ocean_safe_normalize(
        mix(reflected_direction, normal, cone),
        normal,
    );
    let sky_up = ocean_safe_normalize(
        body_to_render * ocean_safe_normalize(surface_direction, vec3(0.0, 1.0, 0.0)),
        normal,
    );
    let sky = ocean_sky_radiance(
        sky_direction,
        sky_up,
        sky_zenith,
        sky_horizon,
        sky_ground,
    )
        * ocean_schlick(f0, n_dot_v)
        * sky_strength
        * (1.0 - 0.35 * roughness);
    let sun_reflection = ocean_sun_reflection(
        normal,
        view,
        sun_vector,
        sun_illuminance,
        roughness,
        f0,
    ) * sun_strength;
    return max(sky + sun_reflection, vec3(0.0, 0.0, 0.0));
}
